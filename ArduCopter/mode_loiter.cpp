/**
 * @file mode_loiter.cpp
 * @brief Loiter flight mode implementation for ArduCopter
 * 
 * @details This file implements GPS-assisted position and altitude hold (Loiter mode).
 *          Loiter mode maintains the vehicle's horizontal position using GPS coordinates
 *          and altitude using barometric pressure sensor. The mode integrates multiple
 *          control libraries to achieve stable position hold with pilot override capability.
 * 
 * Key Features:
 * - GPS position hold: Maintains current latitude/longitude when sticks are centered
 * - Altitude hold: Maintains barometric altitude using vertical position controller
 * - Pilot override: Stick inputs reposition vehicle while holding new position when released
 * - Wind compensation: Automatically adjusts attitude to counter wind drift
 * - EKF integration: Uses Extended Kalman Filter for position estimation and fusion
 * 
 * Library Integration:
 * - AC_WPNav (loiter_nav): Handles horizontal position control and pilot repositioning
 * - AC_PosControl (pos_control): Manages vertical position and velocity control
 * - AC_AttitudeControl (attitude_control): Converts desired movement to motor commands
 * - AP_AHRS/AP_NavEKF: Provides position estimates from GPS, barometer, and IMU fusion
 * 
 * GPS/EKF Requirements:
 * - Requires GPS lock with sufficient satellite count and HDOP
 * - EKF must report position estimate as healthy (good innovation consistency)
 * - Falls back to stabilize mode if GPS/EKF health degrades during flight
 * 
 * Coordinate System:
 * - Position control operates in NED (North-East-Down) frame
 * - GPS coordinates converted to local NE position relative to origin
 * - Altitude measured as cm above origin (negative down)
 * 
 * @note This mode is called at the main loop rate (typically 400Hz)
 * @warning GPS/EKF failures can cause sudden mode transitions - always monitor GPS health
 * 
 * @see AC_WPNav for horizontal position control implementation
 * @see AC_PosControl for vertical position control implementation
 * @see AP_NavEKF3 for position estimation algorithms
 * 
 * Source: ArduCopter/mode_loiter.cpp
 */

#include "Copter.h"

#if MODE_LOITER_ENABLED

/*
 * Init and run calls for loiter flight mode
 */

/**
 * @brief Initialize Loiter mode controller and set initial position target
 * 
 * @details Initializes the Loiter flight mode by setting up horizontal and vertical
 *          position controllers. This function is called once when entering Loiter mode
 *          from another flight mode. It establishes the current position as the initial
 *          loiter target and prepares all control loops for position hold.
 * 
 * Initialization Sequence:
 * 1. Apply SIMPLE mode transformation if enabled (pilot-relative control)
 * 2. Convert current pilot stick inputs to desired lean angles
 * 3. Set initial pilot acceleration inputs (typically zero if sticks centered)
 * 4. Initialize loiter target to current GPS position (lat/lon from EKF)
 * 5. Initialize vertical position controller if not already active
 * 6. Set vertical speed/acceleration limits from pilot parameters
 * 7. Reset precision loiter state if enabled
 * 
 * Position Target Initialization:
 * - loiter_nav->init_target() captures current EKF position estimate as hold point
 * - This becomes the GPS coordinate the vehicle will attempt to maintain
 * - Wind compensation begins immediately to counteract drift from target
 * 
 * Vertical Controller Setup:
 * - Initializes altitude hold at current barometric altitude
 * - Sets climb rate to zero (holding altitude)
 * - Configures max climb/descent rates from PILOT_SPEED_UP/DN parameters
 * - Sets acceleration limit from PILOT_ACCEL_Z parameter
 * 
 * @param[in] ignore_checks If true, skip pre-arm and safety checks (typically false)
 * 
 * @return true Always returns true - mode initialization cannot fail
 *              (GPS/EKF checks performed before mode entry, not during init)
 * 
 * @note Called once on mode entry, not every loop
 * @note Assumes GPS lock and EKF health already verified by mode selection logic
 * @warning Initial target set to current position - vehicle may drift if GPS updating slowly
 * 
 * @see AC_WPNav::init_target() for horizontal position target initialization
 * @see AC_PosControl::init_U_controller() for vertical controller setup
 * @see Copter::update_simple_mode() for SIMPLE mode coordinate transformation
 */
bool ModeLoiter::init(bool ignore_checks)
{
    float target_roll_rad, target_pitch_rad;
    
    // Apply SIMPLE mode transform to pilot inputs
    // SIMPLE mode rotates pilot inputs to be relative to initial takeoff heading
    // instead of current vehicle heading, making control more intuitive for beginners
    update_simple_mode();

    // Convert pilot stick input to desired lean angles (radians)
    // Respects ANGLE_MAX parameter and altitude-hold-specific lean angle limits
    // If sticks centered: target angles will be zero (no pilot-commanded movement)
    get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, loiter_nav->get_angle_max_rad(), attitude_control->get_althold_lean_angle_max_rad());

    // Process pilot's roll and pitch input and convert to desired acceleration
    // This initializes the loiter controller with current pilot input state
    // If sticks are centered, desired acceleration is zero = hold position
    loiter_nav->set_pilot_desired_acceleration_rad(target_roll_rad, target_pitch_rad);

    // Initialize loiter target position to current GPS/EKF position estimate
    // This captures the current latitude/longitude as the position hold target
    // The vehicle will attempt to return to this position whenever sticks are centered
    loiter_nav->init_target();

    // Initialize the vertical position controller if not already active
    // The U-axis controller manages altitude hold using barometric altitude
    // Only initializes if controller not already running (smooth transitions)
    if (!pos_control->is_active_U()) {
        pos_control->init_U_controller();
    }

    // Set vertical speed and acceleration limits from parameters
    // Limits maximum climb/descent rates and acceleration for smooth flight
    // - First parameter: Maximum descent speed (negative value in cm/s)
    // - Second parameter: Maximum climb speed (positive value in cm/s)  
    // - Third parameter: Maximum vertical acceleration (cm/s²)
    pos_control->set_max_speed_accel_U_cm(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_U_cmss(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

#if AC_PRECLAND_ENABLED
    // Reset precision loiter state on mode entry
    // Precision loiter uses IR-LOCK or similar sensor for high-accuracy positioning
    // State will be re-evaluated in run() based on sensor availability
    _precision_loiter_active = false;
#endif

    return true;
}

#if AC_PRECLAND_ENABLED
/**
 * @brief Determine if precision loiter should be active based on sensor and pilot state
 * 
 * @details Evaluates multiple conditions to decide whether to use precision loiter
 *          (high-accuracy positioning using IR-LOCK or similar visual target) versus
 *          standard GPS-based loiter. Precision loiter provides centimeter-level
 *          accuracy for positioning over a known target (e.g., landing pad marker).
 * 
 * Activation Requirements (all must be true):
 * - Precision loiter feature enabled via parameter (PLND_ENABLED)
 * - Vehicle not on ground (land_complete_maybe false)
 * - Pilot stick inputs minimal (<50 cm/s² desired acceleration)
 * - Precision landing sensor has acquired target (valid bearing/distance)
 * 
 * Pilot Override:
 * - Large stick deflections (>50 cm/s²) disable precision loiter
 * - Allows pilot to manually reposition away from precision target
 * - Returns to GPS loiter when precision conditions not met
 * 
 * @return true Precision loiter should be active (use visual target positioning)
 * @return false Use standard GPS-based loiter control
 * 
 * @note Called at main loop rate (400Hz) to continuously evaluate precision loiter state
 * @note Threshold of 50 cm/s² allows small stick movements without disabling precision mode
 * 
 * @see precision_loiter_xy() for precision positioning control loop
 * @see AP_PrecLand::target_acquired() for sensor target detection
 */
bool ModeLoiter::do_precision_loiter()
{
    // Check if precision loiter feature is enabled in parameters
    if (!_precision_loiter_enabled) {
        return false;
    }
    
    // Don't attempt precision positioning when vehicle is on the ground
    // Prevents unnecessary movements during landing or while landed
    if (copter.ap.land_complete_maybe) {
        return false;        // don't move on the ground
    }
    
    // If the pilot *really* wants to move the vehicle, let them...
    // Large stick deflections (>50 cm/s² acceleration) override precision positioning
    // This allows pilot to manually fly away from the precision target if needed
    if (loiter_nav->get_pilot_desired_acceleration_NE_cmss().length() > 50.0f) {
        return false;
    }
    
    // Check if precision landing sensor has acquired a valid target
    // Target must be visible and within sensor range with good signal quality
    if (!copter.precland.target_acquired()) {
        return false; // we don't have a good vector
    }
    
    return true;
}

/**
 * @brief Execute precision loiter horizontal positioning using visual target sensor
 * 
 * @details Performs high-accuracy horizontal position control using precision landing
 *          sensor (IR-LOCK, optical flow, or similar) instead of GPS. Achieves
 *          centimeter-level accuracy over a visual target marker. Position controller
 *          tracks both target position and velocity for smooth tracking of moving targets.
 * 
 * Algorithm:
 * 1. Clear any pilot-commanded acceleration inputs (precision mode is autonomous)
 * 2. Get target position from precision landing sensor in NE frame (cm)
 * 3. Get target velocity if target is moving (typically zero for static landing pad)
 * 4. Command position controller with target position, velocity, and zero acceleration
 * 5. Run NE (North-East) position controller to generate thrust vector
 * 
 * Target Position:
 * - Expressed in local NE frame relative to EKF origin
 * - If sensor fails to provide position, uses current vehicle position estimate
 * - Position updated at sensor rate (typically 50Hz for IR-LOCK)
 * 
 * Target Velocity:
 * - Allows tracking of moving targets (e.g., ship deck landing)
 * - Typically zero for stationary landing pad markers
 * - Improves tracking performance by feedforward compensation
 * 
 * @note Called at 400Hz when do_precision_loiter() returns true
 * @note Bypasses standard loiter_nav controller - direct position control
 * @note Requires precision landing sensor with valid target lock
 * 
 * @warning Position jumps can occur if target lost and reacquired - pilot should monitor
 * 
 * @see do_precision_loiter() for conditions that activate precision positioning
 * @see AP_PrecLand::get_target_position_cm() for sensor position measurement
 * @see AC_PosControl::input_pos_vel_accel_NE_cm() for position setpoint interface
 */
void ModeLoiter::precision_loiter_xy()
{
    // Clear any pilot desired acceleration - precision loiter is autonomous
    // Pilot inputs are ignored during precision positioning (except large overrides)
    loiter_nav->clear_pilot_desired_acceleration();
    
    Vector2f target_pos, target_vel;
    
    // Get target position from precision landing sensor
    // Position is in local NE frame (cm relative to EKF origin)
    // If sensor fails to provide position, fall back to current vehicle position
    if (!copter.precland.get_target_position_cm(target_pos)) {
        target_pos = pos_control->get_pos_estimate_NEU_cm().xy().tofloat();
    }
    
    // Get the velocity of the target (for moving target tracking)
    // Velocity in cm/s in local NE frame
    // Remains zero for stationary targets (typical landing pad)
    copter.precland.get_target_velocity_cms(pos_control->get_vel_estimate_NEU_cms().xy(), target_vel);

    Vector2f zero;  // Zero acceleration command (position/velocity tracking only)
    Vector2p landing_pos = target_pos.topostype();  // Convert to position type
    
    // Command position controller: position setpoint, velocity feedforward, zero acceleration
    // Target velocity will remain zero if landing target is stationary
    // Position controller will generate thrust vector to minimize position error
    pos_control->input_pos_vel_accel_NE_cm(landing_pos, target_vel, zero);
    
    // Run the NE (North-East) position controller
    // Generates thrust vector commands for attitude controller
    // Operates at 400Hz for smooth, responsive positioning
    pos_control->update_NE_controller();
}
#endif

/**
 * @brief Main Loiter mode control loop - executes GPS position hold with pilot override
 * 
 * @details This is the primary control loop for Loiter mode, called at the main scheduler
 *          rate (typically 400Hz). Implements GPS-assisted position hold by:
 *          - Maintaining horizontal position using GPS/EKF when sticks centered
 *          - Allowing pilot repositioning via stick inputs (new position held when released)
 *          - Holding altitude using barometric pressure sensor and vertical position control
 *          - Compensating for wind to maintain GPS coordinates
 *          - Managing smooth transitions between landed, takeoff, and flying states
 * 
 * Control Flow:
 * 1. Set vertical speed/acceleration limits from parameters
 * 2. Process pilot inputs (roll, pitch, yaw rate, climb rate)
 * 3. Apply SIMPLE mode transformation if enabled
 * 4. Convert pilot inputs to desired accelerations for position controller
 * 5. Determine vehicle state (motor stopped, landed, takeoff, flying)
 * 6. Execute state-specific control logic
 * 7. Update horizontal position controller (GPS loiter or precision loiter)
 * 8. Update vertical position controller (altitude hold or climb)
 * 9. Command attitude controller with thrust vector and yaw rate
 * 
 * Pilot Control:
 * - Roll/Pitch sticks: Reposition vehicle horizontally (max rate limited by LOIT_SPEED)
 * - Yaw stick: Rotate vehicle heading
 * - Throttle stick: Climb or descend (max rate limited by PILOT_SPEED_UP/DN)
 * - Sticks centered: Hold current GPS position and altitude
 * 
 * Position Hold Algorithm:
 * - AC_WPNav::loiter_nav maintains horizontal position via PI controller
 * - Position error (GPS vs target) generates velocity correction
 * - Velocity error generates acceleration command
 * - Acceleration converted to lean angles by position controller
 * - Wind compensation automatically adjusts lean angle to maintain position
 * 
 * State Machine:
 * - MotorStopped: Vehicle disarmed, all controllers reset
 * - Landed_Ground_Idle: Motors armed but not spinning, on ground
 * - Landed_Pre_Takeoff: Motors spooled up, ready for takeoff
 * - Takeoff: Climbing to takeoff altitude, position hold active
 * - Flying: Normal loiter operation with full pilot authority
 * 
 * GPS/EKF Integration:
 * - Position estimates from AP_NavEKF3 (GPS + barometer + IMU fusion)
 * - EKF innovations (measurement residuals) monitored for consistency
 * - Poor GPS/EKF health can trigger failsafe mode transition
 * - Requires minimum GPS satellite count and acceptable HDOP
 * 
 * Wind Compensation:
 * - Position controller automatically leans into wind to maintain GPS position
 * - Lean angle increases with wind speed up to ANGLE_MAX limit
 * - If wind exceeds vehicle capability, position will drift
 * 
 * Altitude Hold:
 * - Uses barometric altitude estimate from EKF
 * - PI controller minimizes altitude error
 * - Rangefinder used for terrain following if enabled and available
 * - Pilot can override altitude with throttle stick (proportional to stick deflection)
 * 
 * @note Called at main loop rate (typically 400Hz) for smooth, responsive control
 * @note Thread-safe: Accesses shared state via AP::ahrs(), AP::gps(), etc.
 * @warning GPS/EKF health must be good for safe loiter - monitor EKF innovations
 * @warning High winds may exceed vehicle capability - monitor lean angle
 * 
 * @see AC_WPNav::update() for horizontal position control implementation
 * @see AC_PosControl::update_U_controller() for vertical position control
 * @see AC_AttitudeControl::input_thrust_vector_rate_heading_rads() for attitude control
 * @see get_alt_hold_state() for state machine logic
 */
void ModeLoiter::run()
{
    float target_roll_rad, target_pitch_rad;
    float target_yaw_rate_rads = 0.0f;
    float target_climb_rate_cms = 0.0f;

    // Set vertical speed and acceleration limits from parameters
    // These limits constrain maximum climb/descent rates for smooth, safe flight
    // Updated every loop to allow parameter changes without mode transition
    // U-axis is vertical (up) in NED coordinate frame
    pos_control->set_max_speed_accel_U_cm(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // Apply SIMPLE mode transform to pilot inputs if enabled
    // SIMPLE mode rotates pilot roll/pitch commands to be relative to initial takeoff heading
    // Makes control more intuitive - forward stick always moves toward original front
    // No effect if SIMPLE mode disabled (standard body-frame control)
    update_simple_mode();

    // Convert pilot stick input to desired lean angles (radians)
    // Stick deflection -> lean angle with exponential curve and ANGLE_MAX limit
    // Uses loiter-specific angle limit (may be different from stabilize ANGLE_MAX)
    // Respects altitude-hold lean angle limit to ensure altitude hold authority
    // Zero stick input = zero lean angle = no pilot-commanded horizontal movement
    get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, loiter_nav->get_angle_max_rad(), attitude_control->get_althold_lean_angle_max_rad());

    // Process pilot's roll and pitch input and convert to desired acceleration
    // Lean angles converted to horizontal acceleration commands for position controller
    // Position controller will add this to position-hold acceleration to get total command
    // This allows smooth blending between position hold and pilot repositioning
    loiter_nav->set_pilot_desired_acceleration_rad(target_roll_rad, target_pitch_rad);

    // Get pilot's desired yaw rate from rudder stick
    // Positive = clockwise rotation, negative = counter-clockwise (NED frame)
    // Zero when stick centered = hold current heading
    // Rate limited by ACRO_YAW_P parameter
    target_yaw_rate_rads = get_pilot_desired_yaw_rate_rads();

    // Get pilot desired climb rate from throttle stick
    // Throttle centered = zero climb rate = hold altitude
    // Throttle up = climb at rate proportional to stick deflection
    // Throttle down = descend at rate proportional to stick deflection
    // Constrain to maximum climb/descent speeds from parameters
    target_climb_rate_cms = get_pilot_desired_climb_rate();
    target_climb_rate_cms = constrain_float(target_climb_rate_cms, -get_pilot_speed_dn(), g.pilot_speed_up);

    // Relax loiter target if we might be landed
    // Softens position controller gains when on or near ground
    // Prevents aggressive corrections that could cause vehicle to tip over
    // Allows small position drift without strong reaction when landed
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Loiter State Machine Determination
    // Determine current vehicle state based on:
    // - Motor spool state (stopped, spooling, throttle unlimited)
    // - Climb rate command (zero = holding, non-zero = climbing/descending)
    // - Takeoff state (is takeoff in progress?)
    // - Land complete flags (on ground detection)
    // Returns: MotorStopped, Landed_Ground_Idle, Landed_Pre_Takeoff, Takeoff, or Flying
    AltHoldModeState loiter_state = get_alt_hold_state(target_climb_rate_cms);

    // Loiter State Machine
    // Each state implements different control behavior appropriate for vehicle state
    // Ensures smooth transitions between ground, takeoff, flying, and landing
    switch (loiter_state) {

    case AltHoldModeState::MotorStopped:
        // Vehicle is disarmed or motors stopped
        // Reset all controller integrators to prevent wind-up
        // Ensures clean state when motors restart
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate();
        pos_control->relax_U_controller(0.0f);   // forces throttle output to decay to zero
        loiter_nav->init_target();  // Reset position target to prevent jumps on motor start
        break;

    case AltHoldModeState::Landed_Ground_Idle:
        // Vehicle is armed but motors not spinning (ground idle)
        // Reset yaw to current heading to prevent rotation on spool-up
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHoldModeState::Landed_Pre_Takeoff:
        // Motors spooling up but vehicle still on ground
        // Smoothly reset rate controller I-terms to prevent sudden movements
        // Initialize position target to current location before takeoff
        // Relax throttle to zero - vehicle weight still supported by ground
        attitude_control->reset_rate_controller_I_terms_smoothly();
        loiter_nav->init_target();  // Set loiter target to current GPS position
        pos_control->relax_U_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHoldModeState::Takeoff:
        // Vehicle is taking off - climbing to takeoff altitude
        // Horizontal position hold active during takeoff for stable climb
        
        // Initiate take-off sequence if not already running
        // Starts climb to PILOT_TKOFF_ALT (default 0 = disabled, typical 100-250cm)
        // Constrains altitude to reasonable range (0-1000cm)
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // Get avoidance adjusted climb rate
        // Object avoidance can reduce climb rate if obstacle detected above
        // Falls back to pilot climb rate if avoidance not active or no obstacles
        target_climb_rate_cms = get_avoidance_adjusted_climbrate_cms(target_climb_rate_cms);

        // Set position controller targets adjusted for pilot input
        // Manages climb profile and transitions to position hold when target alt reached
        // Allows pilot to adjust climb rate during takeoff with throttle stick
        takeoff.do_pilot_takeoff(target_climb_rate_cms);

        // Run loiter position controller during takeoff
        // Maintains horizontal position hold while climbing
        // Updates thrust vector for attitude controller
        // Allows pilot to reposition with roll/pitch stick during takeoff
        loiter_nav->update();
        break;

    case AltHoldModeState::Flying:
        // Normal flight state - full position and altitude control active
        // Vehicle is airborne with all control authority available
        
        // Set motors to full throttle range (unlimited)
        // Allows position controller full authority to maintain position
        // Motors can command from minimum to maximum throttle as needed
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

#if AC_PRECLAND_ENABLED
        // Precision Loiter: High-accuracy positioning using visual target sensor
        // Provides centimeter-level accuracy over IR-LOCK or similar beacon
        // Automatically activates when target visible and pilot inputs minimal
        
        bool precision_loiter_old_state = _precision_loiter_active;
        
        // Check if conditions met for precision loiter
        // Requires: target acquired, not on ground, minimal pilot input
        if (do_precision_loiter()) {
            // Use precision positioning - track visual target instead of GPS
            // Position controller directly commands position to maintain over target
            precision_loiter_xy();
            _precision_loiter_active = true;
        } else {
            // Fall back to standard GPS-based loiter
            _precision_loiter_active = false;
        }
        
        // Detect transition from precision to GPS loiter
        if (precision_loiter_old_state && !_precision_loiter_active) {
            // Precision loiter was active, not any more - pilot taking control or target lost
            // Re-initialize position target to prevent position jump
            // New target set to current GPS position for smooth transition
            loiter_nav->init_target();
        }
        
        // Run standard GPS loiter controller if not using precision positioning
        // Maintains GPS position hold with wind compensation
        // Processes pilot repositioning inputs
        // Updates thrust vector for attitude controller
        if (!_precision_loiter_active) {
            loiter_nav->update();
        }
#else
        // Precision loiter not compiled in - always use GPS-based position hold
        // Standard loiter navigation: maintain GPS coordinates with wind compensation
        // Integrates pilot repositioning commands
        loiter_nav->update();
#endif

        // Get avoidance adjusted climb rate
        // Object avoidance system can modify climb rate to avoid obstacles
        // Uses rangefinders or proximity sensors to detect obstacles
        // Reduces climb rate if obstacle detected in path
        target_climb_rate_cms = get_avoidance_adjusted_climbrate_cms(target_climb_rate_cms);

#if AP_RANGEFINDER_ENABLED
        // Update the vertical offset based on surface measurement
        // Terrain following: adjusts altitude target to maintain constant height above ground
        // Uses rangefinder to measure distance to ground
        // Only active if SURFACE_TRACKING parameter enabled
        copter.surface_tracking.update_surface_offset();
#endif

        // Send the commanded climb rate to the position controller
        // Integrates climb rate to update altitude target
        // Altitude hold: climb rate = 0 maintains current altitude
        // Pilot climb/descent: non-zero climb rate adjusts altitude target
        // Position controller will generate thrust to achieve target altitude
        pos_control->set_pos_target_U_from_climb_rate_cm(target_climb_rate_cms);
        break;
    }

    // Call attitude controller with desired thrust vector and yaw rate
    // Thrust vector from position controller specifies desired acceleration direction
    // Attitude controller converts thrust vector to roll/pitch angles and motor commands
    // Yaw rate command rotates vehicle heading independent of position control
    // Third parameter (false) = don't use yaw angle control, use rate control
    // 
    // Integration: Position hold (loiter_nav) -> Thrust vector -> Attitude control -> Motors
    // Wind compensation: Position error generates thrust vector leaning into wind
    attitude_control->input_thrust_vector_rate_heading_rads(loiter_nav->get_thrust_vector(), target_yaw_rate_rads, false);
    
    // Run the vertical position controller and set output throttle
    // Generates throttle command to achieve target altitude and climb rate
    // PI controller minimizes altitude error from barometric altitude estimate
    // Output passed to motors along with attitude control roll/pitch/yaw
    // Called every loop (400Hz) for responsive altitude control
    pos_control->update_U_controller();
}

/**
 * @brief Get horizontal distance to loiter target position
 * 
 * @details Returns the 2D distance from current vehicle position to the loiter
 *          target position. This is the position error that the loiter controller
 *          is attempting to minimize. Used for telemetry display and monitoring
 *          position hold performance.
 * 
 * Distance Calculation:
 * - Computed in NE (North-East) plane, ignoring altitude difference
 * - Uses EKF position estimate for current vehicle position
 * - Target position is the GPS coordinate being held (or last pilot reposition)
 * - Distance increases when wind pushes vehicle away from target
 * - Distance decreases as controller corrects position back to target
 * 
 * @return float Distance to loiter target in meters
 * 
 * @note Returns 0.0 if loiter target not initialized
 * @note Large distances (>5m) may indicate GPS glitch or excessive wind
 * 
 * @see loiter_nav->get_distance_to_target_cm() for internal distance calculation
 */
float ModeLoiter::wp_distance_m() const
{
    return loiter_nav->get_distance_to_target_cm() * 0.01f;
}

/**
 * @brief Get bearing from vehicle to loiter target position
 * 
 * @details Returns the compass bearing (degrees) from current vehicle position
 *          to the loiter target position. Indicates the direction the vehicle
 *          needs to move to return to the target position. Used for telemetry
 *          display and navigation indication.
 * 
 * Bearing Convention:
 * - 0° = North
 * - 90° = East  
 * - 180° = South
 * - 270° = West
 * - Range: 0-360 degrees
 * 
 * Usage:
 * - Displayed to pilot via telemetry/OSD
 * - Indicates wind direction pushing vehicle (opposite of bearing)
 * - Used by ground station to show position hold error direction
 * 
 * @return float Bearing to loiter target in degrees (0-360)
 * 
 * @note Bearing is in earth frame (North referenced), not body frame
 * @note Undefined if distance to target is zero (vehicle at target)
 * 
 * @see loiter_nav->get_bearing_to_target_rad() for internal bearing calculation
 */
float ModeLoiter::wp_bearing_deg() const
{
    return degrees(loiter_nav->get_bearing_to_target_rad());
}

#endif
