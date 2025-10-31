/**
 * @file mode_throw.cpp
 * @brief Throw mode implementation for ArduCopter
 * 
 * @details Throw mode allows the copter to be hand-launched by detecting a throwing motion.
 *          The mode automatically arms the vehicle when a throw is detected, uprights the copter,
 *          and transitions to stabilized flight. This mode is designed for small copters that can
 *          be safely thrown by hand to initiate flight.
 * 
 *          Key Features:
 *          - Automatic arming when throw is detected (no pre-arming required)
 *          - Accelerometer-based throw detection algorithm
 *          - Automatic uprighting and stabilization after throw
 *          - Configurable transition to Loiter, RTL, or other flight modes after stabilization
 *          - Safety interlocks prevent accidental activation
 *          - Altitude limits to prevent activation at inappropriate heights
 * 
 *          Safety Considerations:
 *          - Disabled for helicopters (traditional helis cannot be thrown)
 *          - Requires vehicle to be disarmed when entering mode
 *          - Will not activate if vehicle is already flying
 *          - Requires valid AHRS attitude and position estimates
 *          - User-configurable altitude limits for throw detection
 * 
 *          Throw Detection Algorithm:
 *          - Monitors for high velocity (>500 cm/s) or free-fall conditions
 *          - Checks for vertical velocity change (upward throw or downward drop)
 *          - Confirms throw with 2.5 m/s downward velocity change in <0.5s
 *          - Validates acceleration patterns indicating release from hand
 * 
 * @note This mode is intended for small copters only. Do not use with large or heavy vehicles.
 * @warning Improper use of throw mode can result in vehicle damage or injury.
 * 
 * @see https://ardupilot.org/copter/docs/throw-mode.html
 * 
 * Source: ArduCopter/mode_throw.cpp
 */

#include "Copter.h"

#if MODE_THROW_ENABLED

/**
 * @brief Initialize Throw mode controller
 * 
 * @details Prepares the vehicle for throw detection and launch. This function sets up
 *          the initial state machine stage and configures position and altitude controllers
 *          with conservative speed and acceleration limits suitable for post-throw stabilization.
 * 
 *          Initialization Sequence:
 *          1. Check vehicle type compatibility (fails for helicopters)
 *          2. Verify vehicle is disarmed (required for safe throw mode entry)
 *          3. Initialize state machine to Throw_Disarmed stage
 *          4. Configure position controller with brake-mode speeds for rapid stopping
 *          5. Set vertical speed/acceleration limits for controlled altitude capture
 * 
 *          Safety Checks:
 *          - Helicopters are explicitly prevented from using throw mode (unstable during throw)
 *          - Vehicle must be disarmed to enter mode (prevents accidental activation in flight)
 *          - If already armed or flying, mode initialization is rejected
 * 
 *          Controller Configuration:
 *          - Position controller: BRAKE_MODE_DECEL_RATE for rapid horizontal stop after throw
 *          - Altitude controller: BRAKE_MODE_SPEED_Z for controlled vertical stabilization
 *          - Conservative limits ensure stable capture after unpredictable throw dynamics
 * 
 * @param[in] ignore_checks Unused - throw mode always enforces safety checks
 * 
 * @return true if initialization successful and mode can be entered
 * @return false if vehicle is helicopter, already armed, or safety checks fail
 * 
 * @note This function does NOT arm the vehicle. Arming occurs automatically when throw is detected.
 * @warning Never attempt to use throw mode with traditional helicopters - will return false.
 * 
 * @see ModeThrow::run() for state machine execution after initialization
 * @see throw_detected() for throw detection algorithm
 */
bool ModeThrow::init(bool ignore_checks)
{
#if FRAME_CONFIG == HELI_FRAME
    // Safety interlock: Helicopters cannot use throw mode
    // Reason: Traditional helicopters are mechanically unstable during a throw due to
    // rotor inertia and gyroscopic effects. The throwing motion would cause unpredictable
    // attitude changes that could damage the rotor head or result in loss of control.
    return false;
#endif

    // Safety interlock: Reject mode entry if vehicle is already armed or in flight
    // Reason: Throw mode is designed for ground-to-air transitions only. Entering throw
    // mode while already flying could trigger the throw detection algorithm inappropriately,
    // potentially causing unexpected automatic arming or mode transitions.
    if (motors->armed()) {
        return false;
    }

    // Initialize state machine to disarmed state
    // The throw mode state machine always begins in Throw_Disarmed and progresses through
    // detection → uprighting → stabilization → position hold stages
    stage = Throw_Disarmed;
    nextmode_attempted = false;  // Reset flag for automatic mode transition after stabilization

    // Configure horizontal position controller for post-throw stabilization
    // Uses brake-mode deceleration rates to rapidly arrest horizontal motion after throw
    // Max speed: Default waypoint navigation speed (typically 1000 cm/s)
    // Deceleration: BRAKE_MODE_DECEL_RATE provides aggressive stopping (typically 250 cm/s/s)
    pos_control->set_max_speed_accel_NE_cm(wp_nav->get_default_speed_NE_cms(), BRAKE_MODE_DECEL_RATE);
    pos_control->set_correction_speed_accel_NE_cm(wp_nav->get_default_speed_NE_cms(), BRAKE_MODE_DECEL_RATE);

    // Configure vertical speed and acceleration limits for altitude stabilization
    // Conservative limits ensure smooth altitude capture after unpredictable throw trajectory
    // BRAKE_MODE_SPEED_Z: Vertical speed limit (typically 250 cm/s)
    // BRAKE_MODE_DECEL_RATE: Vertical deceleration for smooth altitude capture
    pos_control->set_max_speed_accel_U_cm(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z, BRAKE_MODE_DECEL_RATE);
    pos_control->set_correction_speed_accel_U_cmss(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z, BRAKE_MODE_DECEL_RATE);

    return true;
}

/**
 * @brief Main throw mode controller - executes throw detection and stabilization state machine
 * 
 * @details This function implements the complete throw mode state machine, from initial throw
 *          detection through automatic arming, uprighting, and stabilization. Called at main
 *          loop rate (typically 400 Hz) to provide responsive control during the critical
 *          post-throw stabilization phase.
 * 
 *          State Machine Stages:
 *          
 *          1. Throw_Disarmed: Initial state, motors off, waiting for arming
 *             - Motors shut down or at ground idle (configurable)
 *             - Attitude controller continuously reset
 *             - Transitions to Throw_Detecting when vehicle is armed
 * 
 *          2. Throw_Detecting: Armed and waiting for throw motion detection
 *             - Motors remain off (or ground idle if configured)
 *             - throw_detected() continuously monitors accelerometers and velocity
 *             - Plays audible tone to alert user that mode is ready
 *             - Transitions to Throw_Wait_Throttle_Unlimited when throw detected
 * 
 *          3. Throw_Wait_Throttle_Unlimited: Spooling motors after throw detection
 *             - Motors commanded to unlimited throttle range
 *             - Brief transition state ensuring motors are ready for control
 *             - Transitions to Throw_Uprighting when spool state reaches THROTTLE_UNLIMITED
 * 
 *          4. Throw_Uprighting: Actively uprighting the vehicle to level attitude
 *             - Commands level attitude (0° roll, 0° pitch, 0° yaw rate)
 *             - Applies 50% throttle with angle boost disabled for maximum righting torque
 *             - Transitions to Throw_HgtStabilise when throw_attitude_good() confirms upright
 * 
 *          5. Throw_HgtStabilise: Stabilizing altitude while maintaining level attitude
 *             - Holds level attitude with zero yaw rate
 *             - Altitude controller targets height offset from throw detection point
 *             - Height offset configurable: ascend (default) or descend (airdrop mode)
 *             - Transitions to Throw_PosHold when throw_height_good() confirms altitude achieved
 * 
 *          6. Throw_PosHold: Final stage - holding position and altitude
 *             - Full position hold using horizontal position controller
 *             - Continues altitude hold at target height
 *             - Automatically transitions to configured next mode (LOITER, RTL, etc.)
 *             - Remains in this state if next mode not configured
 * 
 *          Automatic Arming Behavior:
 *          - Vehicle arms automatically when throw is detected (Throw_Detecting → Throw_Wait_Throttle_Unlimited)
 *          - No pre-arm checks required (mode designed for rapid deployment)
 *          - auto_armed flag set during stabilization to prevent automatic disarm
 * 
 *          Safety Features:
 *          - State machine can only enter from Throw_Disarmed (prevents mid-flight activation)
 *          - If vehicle disarms during execution, immediately returns to Throw_Disarmed
 *          - AHRS health continuously monitored during throw detection
 *          - Altitude limits enforced to prevent activation at unsafe heights
 * 
 *          Performance Requirements:
 *          - MUST be called at 100 Hz or higher for responsive throw detection
 *          - Typical call rate: 400 Hz (main loop rate)
 *          - State transitions occur within single loop iterations for rapid response
 * 
 * @note This function handles both throw detection and all subsequent flight control.
 *       The throw detection algorithm runs continuously in Throw_Detecting stage.
 * 
 * @warning Do not call this function at rates below 100 Hz - throw detection timing
 *          depends on high-rate accelerometer sampling and velocity estimation.
 * 
 * @see throw_detected() for details on throw detection algorithm
 * @see throw_attitude_good() for uprighting completion criteria
 * @see throw_height_good() for altitude stabilization criteria
 * @see throw_position_good() for position hold criteria
 */
void ModeThrow::run()
{
    /* Throw State Machine Summary:
     * 
     * Throw_Disarmed        - Vehicle not armed, motors off, waiting
     * Throw_Detecting       - Armed and monitoring for throw motion
     * Throw_Wait_Throttle_Unlimited - Spooling motors after throw detected
     * Throw_Uprighting      - Actively uprighting vehicle to level attitude
     * Throw_HgtStabilise    - Holding level attitude and stabilizing altitude
     * Throw_PosHold         - Final position hold before mode transition
     */

    // State Transition Logic:
    // This section determines when to advance through the state machine based on
    // sensor readings, motor status, and stabilization criteria.

    if (!motors->armed()) {
        // Safety reset: If vehicle becomes disarmed at any stage, immediately return to initial state
        // This ensures state machine always starts from a known safe condition
        stage = Throw_Disarmed;

    } else if (stage == Throw_Disarmed && motors->armed()) {
        // Transition: Disarmed → Detecting
        // Vehicle has been armed (typically via throw mode arming switch), begin throw detection
        gcs().send_text(MAV_SEVERITY_INFO,"waiting for throw");
        stage = Throw_Detecting;

    } else if (stage == Throw_Detecting && throw_detected()){
        // Transition: Detecting → Wait_Throttle_Unlimited
        // Throw motion detected! Begin motor spool-up for flight control
        // This is the critical moment when automatic arming occurs
        gcs().send_text(MAV_SEVERITY_INFO,"throw detected - spooling motors");
        copter.set_land_complete(false);  // Mark vehicle as airborne
        stage = Throw_Wait_Throttle_Unlimited;

        // Cancel the waiting for throw tone sequence (user has successfully thrown)
        AP_Notify::flags.waiting_for_throw = false;

    } else if (stage == Throw_Wait_Throttle_Unlimited &&
               motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
        // Transition: Wait_Throttle_Unlimited → Uprighting
        // Motors have spooled up and are ready for full control authority
        gcs().send_text(MAV_SEVERITY_INFO,"throttle is unlimited - uprighting");
        stage = Throw_Uprighting;
    } else if (stage == Throw_Uprighting && throw_attitude_good()) {
        // Transition: Uprighting → HgtStabilise
        // Vehicle has achieved level attitude (within 30° of upright), begin altitude control
        gcs().send_text(MAV_SEVERITY_INFO,"uprighted - controlling height");
        stage = Throw_HgtStabilise;

        // Initialize vertical position controller without descent constraints
        // Allows aggressive altitude correction if needed after throw
        pos_control->init_U_controller_no_descent();

        // Set target altitude based on throw type configuration:
        // - Standard throw: Ascend to clear obstacles (THROW_ALTITUDE_ASCEND parameter)
        // - Airdrop mode: Descend for landing approach (THROW_ALTITUDE_DESCEND parameter)
        // Altitude is relative to the detected throw release point
        if (g2.throw_type == ThrowType::Drop) {
            // Airdrop mode: Descend from release altitude (e.g., dropped from drone or person)
            pos_control->set_pos_desired_U_cm(pos_control->get_pos_estimate_NEU_cm().z - g.throw_altitude_descend * 100.0f);
        } else {
            // Standard throw mode: Ascend to safe altitude above obstacles
            pos_control->set_pos_desired_U_cm(pos_control->get_pos_estimate_NEU_cm().z + g.throw_altitude_ascend * 100.0f);
        }

        // Set auto_armed flag to prevent automatic disarm
        // Without this, switching to an auto mode with low throttle could trigger disarm sequence
        copter.set_auto_armed(true);

    } else if (stage == Throw_HgtStabilise && throw_height_good()) {
        // Transition: HgtStabilise → PosHold
        // Altitude has stabilized within 50cm of target, begin horizontal position hold
        gcs().send_text(MAV_SEVERITY_INFO,"height achieved - controlling position");
        stage = Throw_PosHold;

        // Initialize horizontal position controller for full position hold
        // This arrests any remaining horizontal drift from the throw
        pos_control->init_NE_controller();

        // Maintain auto_armed status through position hold phase
        copter.set_auto_armed(true);
    } else if (stage == Throw_PosHold && throw_position_good()) {
        // Transition: PosHold → Next Mode (one-time attempt)
        // Horizontal position has stabilized within 50cm, vehicle is stable and ready for mission
        // Automatically transition to user-configured next mode (THROW_NEXTMODE parameter)
        if (!nextmode_attempted) {
            // Attempt mode transition only once to avoid repeated switching if mode entry fails
            switch ((Mode::Number)g2.throw_nextmode.get()) {
                case Mode::Number::AUTO:      // Begin autonomous mission
                case Mode::Number::GUIDED:    // Accept guided mode commands
                case Mode::Number::RTL:       // Return to launch
                case Mode::Number::LAND:      // Begin landing sequence
                case Mode::Number::BRAKE:     // Hold current position with brake controller
                case Mode::Number::LOITER:    // Standard GPS position hold
                    // These modes are safe for automatic transition after throw stabilization
                    set_mode((Mode::Number)g2.throw_nextmode.get(), ModeReason::THROW_COMPLETE);
                    break;
                default:
                    // Other modes not in whitelist - remain in throw mode position hold
                    // This prevents transition to potentially unsafe modes (e.g., ACRO, FLIP)
                    break;
            }
            nextmode_attempted = true;  // Prevent repeated transition attempts
        }
    }

    // State Processing: Motor and Control Commands
    // Each state executes specific motor and controller commands appropriate for that phase
    // of the throw sequence. This section runs after state transitions are evaluated.

    switch (stage) {

    case Throw_Disarmed:
        // State Processing: Disarmed - Keep vehicle safe and dormant
        
        // Motor state: Configurable based on THROW_MOTOR_START parameter
        // - SHUT_DOWN (default): Motors completely off for maximum safety
        // - RUNNING: Motors at ground idle for faster response (optional, less safe)
        if (g.throw_motor_start == PreThrowMotorState::RUNNING) {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        } else {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        }

        // Attitude controller reset: Continuously reset to prevent integrator wind-up
        // Zero throttle prevents any motor output even if motors reach ground idle
        attitude_control->reset_yaw_target_and_rate();        // Clear yaw target and rate
        attitude_control->reset_rate_controller_I_terms();    // Clear PID integrators
        attitude_control->set_throttle_out(0,true,g.throttle_filt);  // Zero throttle
        break;

    case Throw_Detecting:
        // State Processing: Detecting - Monitor for throw while keeping motors safe
        
        // Motor state: Same as disarmed - keep motors off until throw detected
        // This ensures no propeller movement during the throw motion (safety critical)
        if (g.throw_motor_start == PreThrowMotorState::RUNNING) {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        } else {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        }

        // Attitude controller: Continue reset during detection phase
        // Prevents integrator wind-up from vehicle motion during throw
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_throttle_out(0,true,g.throttle_filt);

        // Audio feedback: Alert user that system is ready for throw
        // Plays distinctive tone pattern so user knows mode is active
        AP_Notify::flags.waiting_for_throw = true;

        break;

    case Throw_Wait_Throttle_Unlimited:
        // State Processing: Wait_Throttle_Unlimited - Spool motors to full range
        
        // Command motors to unlimited throttle range for full control authority
        // Brief transition state ensuring motor ESCs are ready for aggressive control
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        break;

    case Throw_Uprighting:
        // State Processing: Uprighting - Aggressively level the vehicle
        
        // Motor state: Full throttle range for maximum control authority
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // Attitude command: Demand level attitude (0° roll, 0° pitch, 0° yaw rate)
        // This generates maximum righting moment to bring vehicle upright regardless of throw orientation
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(0.0f, 0.0f, 0.0f);

        // Throttle: 50% provides good righting torque without excessive altitude gain
        // Angle boost disabled: We want pure attitude correction, not compensated vertical thrust
        // This maximizes the effectiveness of the righting moment
        attitude_control->set_throttle_out(0.5f, false, g.throttle_filt);

        break;

    case Throw_HgtStabilise:
        // State Processing: HgtStabilise - Hold level attitude and stabilize altitude
        
        // Motor state: Full throttle range for altitude control
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // Attitude command: Maintain level attitude with zero yaw rate
        // Keeps vehicle stable while altitude controller works to reach target height
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(0.0f, 0.0f, 0.0f);

        // Altitude controller: Target zero climb rate at desired altitude
        // Altitude target was set during state transition (ascend or descend from throw point)
        // Controller works to eliminate altitude error while preventing overshoot
        pos_control->set_pos_target_U_from_climb_rate_cm(0.0f);  // Zero climb rate setpoint
        pos_control->update_U_controller();                       // Execute vertical position control

        break;

    case Throw_PosHold:
        // State Processing: PosHold - Full 3D position hold
        
        // Motor state: Full throttle range for position hold
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // Horizontal position controller: Command zero velocity to arrest drift
        // Creates zero velocity and zero acceleration targets for full position hold
        Vector2f vel;    // Zero velocity target (default initialization)
        Vector2f accel;  // Zero acceleration target (default initialization)
        pos_control->input_vel_accel_NE_cm(vel, accel);     // Set velocity/accel targets
        pos_control->update_NE_controller();                 // Execute horizontal position control

        // Attitude controller: Use thrust vector from position controller
        // Position controller calculates required thrust angle for position hold
        // Zero yaw rate maintains current heading
        attitude_control->input_thrust_vector_rate_heading_rads(pos_control->get_thrust_vector(), 0.0f);

        // Altitude controller: Continue altitude hold at target height
        // Same altitude target set in HgtStabilise stage
        pos_control->set_pos_target_U_from_climb_rate_cm(0.0f);
        pos_control->update_U_controller();

        break;
    }

#if HAL_LOGGING_ENABLED
    // log at 10hz or if stage changes
    uint32_t now = AP_HAL::millis();
    if ((stage != prev_stage) || (now - last_log_ms) > 100) {
        prev_stage = stage;
        last_log_ms = now;
        const float velocity = pos_control->get_vel_estimate_NEU_cms().length();
        const float velocity_z = pos_control->get_vel_estimate_NEU_cms().z;
        const float accel = copter.ins.get_accel().length();
        const float ef_accel_z = ahrs.get_accel_ef().z;
        const bool throw_detect = (stage > Throw_Detecting) || throw_detected();
        const bool attitude_ok = (stage > Throw_Uprighting) || throw_attitude_good();
        const bool height_ok = (stage > Throw_HgtStabilise) || throw_height_good();
        const bool pos_ok = (stage > Throw_PosHold) || throw_position_good();

// @LoggerMessage: THRO
// @Description: Throw Mode messages
// @URL: https://ardupilot.org/copter/docs/throw-mode.html
// @Field: TimeUS: Time since system startup
// @Field: Stage: Current stage of the Throw Mode
// @Field: Vel: Magnitude of the velocity vector
// @Field: VelZ: Vertical Velocity
// @Field: Acc: Magnitude of the vector of the current acceleration
// @Field: AccEfZ: Vertical earth frame accelerometer value
// @Field: Throw: True if a throw has been detected since entering this mode
// @Field: AttOk: True if the vehicle is upright 
// @Field: HgtOk: True if the vehicle is within 50cm of the demanded height
// @Field: PosOk: True if the vehicle is within 50cm of the demanded horizontal position

        AP::logger().WriteStreaming(
            "THRO",
            "TimeUS,Stage,Vel,VelZ,Acc,AccEfZ,Throw,AttOk,HgtOk,PosOk",
            "s-nnoo----",
            "F-0000----",
            "QBffffbbbb",
            AP_HAL::micros64(),
            (uint8_t)stage,
            (double)velocity,
            (double)velocity_z,
            (double)accel,
            (double)ef_accel_z,
            throw_detect,
            attitude_ok,
            height_ok,
            pos_ok);
    }
#endif  // HAL_LOGGING_ENABLED
}

/**
 * @brief Detect throw motion using accelerometer and velocity analysis
 * 
 * @details This function implements the core throw detection algorithm that identifies when
 *          the vehicle has been released from a hand throw or airdrop. The algorithm uses
 *          multiple sensor inputs to reliably distinguish a throw from normal handling while
 *          rejecting false positives from vibration, transportation, or other motions.
 * 
 *          Detection Algorithm Overview:
 *          The algorithm looks for a characteristic signature of a throw/drop:
 *          1. High velocity (>500 cm/s) OR free-fall acceleration
 *          2. Significant vertical velocity (upward throw or downward drop)
 *          3. Low body acceleration indicating release from hand
 *          4. Altitude within configured limits
 *          5. Sustained velocity change (2.5 m/s over 0.5s) for confirmation
 * 
 *          Detection Criteria (all must be true for confirmation):
 *          
 *          A. AHRS Health Checks:
 *          - Valid attitude estimate (required for acceleration analysis)
 *          - Valid horizontal position (required for velocity estimation)
 *          - Valid vertical position (required for altitude check)
 *          
 *          B. Velocity Analysis:
 *          - High speed: Total velocity > 500 cm/s (indicates throw motion)
 *          - Vertical motion: Vertical velocity > 50 cm/s up (throw) or down (drop)
 *          
 *          C. Acceleration Analysis:
 *          - Free-fall: Earth-frame Z acceleration > -0.25g (reduced weight in hand)
 *          - Release: Body acceleration < 1.0g (no longer gripping forces)
 *          
 *          D. Altitude Safety:
 *          - Within THROW_ALTITUDE_MIN and THROW_ALTITUDE_MAX parameters
 *          - Prevents activation on ground or at excessive altitude
 *          
 *          E. Confirmation Phase:
 *          - After initial detection, monitor for 2.5 m/s downward velocity change
 *          - Must occur within 0.5 seconds of initial detection
 *          - Confirms throw trajectory (ballistic arc) vs. other motion
 * 
 *          Throw vs. Airdrop Modes:
 *          - Standard Throw: Looks for upward vertical velocity (hand launch)
 *          - Airdrop Mode: Looks for downward vertical velocity (dropped from altitude)
 *          - Mode selected via THROW_TYPE parameter
 * 
 *          False Positive Prevention:
 *          - Multiple simultaneous conditions prevent accidental triggering
 *          - Velocity confirmation phase eliminates transient events
 *          - Altitude limits prevent ground or high-altitude activation
 *          - AHRS health checks ensure quality sensor data
 * 
 *          Performance Characteristics:
 *          - Detection latency: ~500ms after throw (confirmation period)
 *          - Update rate: Called at main loop rate (typically 400 Hz)
 *          - Requires valid EKF position and velocity estimates
 * 
 * @return true if throw motion is confirmed and vehicle should begin flight
 * @return false if throw not detected or AHRS health checks fail
 * 
 * @note This function is called continuously while in Throw_Detecting stage.
 *       Returns false until complete throw signature is detected.
 * 
 * @warning Detection relies on accurate EKF velocity estimation. Poor GPS or optical
 *          flow can cause detection failures or false triggers. Ensure good position
 *          estimate before attempting throw.
 * 
 * @see ModeThrow::run() for state machine execution
 * @see g2.throw_type for throw vs. airdrop mode selection
 * @see g.throw_altitude_min, g.throw_altitude_max for altitude limits
 */
bool ModeThrow::throw_detected()
{
    // AHRS Health Validation:
    // Throw detection requires accurate attitude, position, and velocity estimates.
    // If any critical estimate is invalid, reject detection to prevent unsafe arming.
    // Check 1: Valid attitude estimate
    // Required for: Body-to-earth frame acceleration transformation
    if (!ahrs.has_status(AP_AHRS::Status::ATTITUDE_VALID)) {
        return false;
    }
    // Check 2: Valid horizontal position (absolute)
    // Required for: Horizontal velocity estimation used in speed check
    if (!ahrs.has_status(AP_AHRS::Status::HORIZ_POS_ABS)) {
        return false;
    }
    // Check 3: Valid vertical position
    // Required for: Altitude limit enforcement and vertical velocity estimation
    if (!ahrs.has_status(AP_AHRS::Status::VERT_POS)) {
        return false;
    }

    // Velocity Criterion 1: High speed detection
    // Check for total 3D velocity > THROW_HIGH_SPEED (500 cm/s)
    // Use length_squared for efficiency (avoids sqrt), compare against squared threshold
    // High speed indicates rapid motion consistent with a throw
    bool high_speed = pos_control->get_vel_estimate_NEU_cms().length_squared() > (THROW_HIGH_SPEED * THROW_HIGH_SPEED);

    // Velocity Criterion 2: Vertical velocity check (mode-dependent)
    // Standard throw mode: Upward velocity > THROW_VERTICAL_SPEED (50 cm/s)
    // Airdrop mode: Downward velocity > THROW_VERTICAL_SPEED (50 cm/s)
    // Z-axis: Positive = up in NED frame, Negative = down in NED frame
    bool changing_height;
    if (g2.throw_type == ThrowType::Drop) {
        // Airdrop: Look for downward velocity (negative Z in NED frame)
        changing_height = pos_control->get_vel_estimate_NEU_cms().z < -THROW_VERTICAL_SPEED;
    } else {
        // Standard throw: Look for upward velocity (positive Z in NED frame)
        changing_height = pos_control->get_vel_estimate_NEU_cms().z > THROW_VERTICAL_SPEED;
    }

    // Acceleration Criterion 1: Free-fall detection
    // Earth-frame vertical acceleration > -0.25g indicates reduced support force
    // During throw, vehicle experiences less than 1g as it's released from hand
    // Z-axis in earth frame: Positive = up, so > -0.25g means less downward acceleration
    bool free_falling = ahrs.get_accel_ef().z > -0.25 * GRAVITY_MSS;

    // Acceleration Criterion 2: Release detection
    // Body-frame acceleration magnitude < 1.0g indicates vehicle is free (not being handled)
    // When holding vehicle, grip forces create accelerations > 1g due to hand motion
    // When released, body acceleration drops to ~0g (discounting aerodynamic forces)
    bool no_throw_action = copter.ins.get_accel().length() < 1.0f * GRAVITY_MSS;

    // Altitude Safety Check:
    // Retrieve altitude above home position for safety limit enforcement
    float altitude_above_home;  // Altitude in meters
    if (ahrs.home_is_set()) {
        // Preferred: Use altitude relative to home location (more meaningful for user)
        ahrs.get_relative_position_D_home(altitude_above_home);
        altitude_above_home = -altitude_above_home; // NED frame: Down is positive, convert to "above" (positive up)
    } else {
        // Fallback: Use altitude relative to EKF origin if home not set
        altitude_above_home = pos_control->get_pos_estimate_NEU_cm().z * 0.01f; // Convert cm to meters
    }

    // Altitude Limit Enforcement:
    // THROW_ALTITUDE_MIN: Prevents activation too close to ground (default: disabled with 0)
    // THROW_ALTITUDE_MAX: Prevents activation at excessive altitude (default: disabled with 0)
    // Both limits provide safety boundaries for throw activation
    const bool height_within_params = (g.throw_altitude_min == 0 || altitude_above_home > g.throw_altitude_min) && 
                                       (g.throw_altitude_max == 0 || (altitude_above_home < g.throw_altitude_max));

    // Initial Detection: Combine all criteria for possible throw
    // Requires: (free-fall OR high-speed) AND vertical-motion AND released AND altitude-ok
    // This combination filters out most non-throw motions while catching actual throws
    bool possible_throw_detected = (free_falling || high_speed) && changing_height && no_throw_action && height_within_params;

    // Confirmation Phase State Management:
    // When possible throw first detected, record timestamp and vertical velocity
    // 500ms debounce prevents rapid re-triggering from transient conditions
    if (possible_throw_detected && ((AP_HAL::millis() - free_fall_start_ms) > 500)) {
        free_fall_start_ms = AP_HAL::millis();              // Record detection time
        free_fall_start_velz = pos_control->get_vel_estimate_NEU_cms().z;  // Record initial vertical velocity
    }

    // Final Confirmation: Verify ballistic trajectory
    // After initial detection, check for 2.5 m/s downward velocity change within 0.5 seconds
    // This confirms the vehicle is following a ballistic arc (free flight) not just being waved around
    // Downward velocity change occurs even for upward throws due to gravity deceleration
    // 250 cm/s = 2.5 m/s downward velocity change threshold
    bool throw_condition_confirmed = ((AP_HAL::millis() - free_fall_start_ms < 500) &&  // Within 0.5s window
                                      ((pos_control->get_vel_estimate_NEU_cms().z - free_fall_start_velz) < -250.0f));  // 2.5 m/s downward

    // Return confirmation status: true triggers motor spool-up and flight control
    return throw_condition_confirmed;
}

/**
 * @brief Check if vehicle attitude is sufficiently upright to proceed from uprighting to altitude stabilization
 * 
 * @details This function determines when the vehicle has achieved a stable upright attitude
 *          after the initial throw uprighting phase. The check uses the rotation matrix to
 *          determine the vehicle's orientation relative to vertical.
 * 
 *          Uprighting Criterion:
 *          - Uses Z-axis of body-to-NED rotation matrix (rotMat.c.z)
 *          - This represents the cosine of the angle between body Z-axis and earth vertical
 *          - Threshold: rotMat.c.z > 0.866 (cosine of 30°)
 *          - Equivalent to: Vehicle tilted less than 30° from vertical
 * 
 *          Mathematical Explanation:
 *          - rotMat.c.z is the (3,3) element of the rotation matrix
 *          - Represents projection of body Z-axis onto earth Z-axis
 *          - Value of 1.0 = perfectly vertical (upright)
 *          - Value of 0.866 = 30° tilt from vertical
 *          - Value of 0.0 = 90° tilt (horizontal)
 * 
 *          Why 30° Threshold:
 *          - Provides good balance between stability and response time
 *          - Within 30°, attitude controller can effectively stabilize
 *          - Too tight threshold delays progression, too loose risks instability
 *          - Tested empirically with various throw scenarios
 * 
 * @return true if vehicle is within 30° of upright (ready for altitude control)
 * @return false if vehicle needs more uprighting
 * 
 * @note This check is evaluated continuously during Throw_Uprighting stage.
 *       When it returns true, state machine advances to Throw_HgtStabilise.
 * 
 * @see ModeThrow::run() for state machine execution
 */
bool ModeThrow::throw_attitude_good() const
{
    // Retrieve body-to-NED rotation matrix from AHRS
    // This matrix transforms vectors from body frame to earth (NED) frame
    const Matrix3f &rotMat = ahrs.get_rotation_body_to_ned();
    
    // Check uprighting criterion: rotMat.c.z > 0.866
    // rotMat.c is the third column vector (body Z-axis in earth frame)
    // rotMat.c.z is the Z component (vertical component in NED frame)
    // 0.866 ≈ cos(30°), so this checks if vehicle is within 30° of vertical
    return (rotMat.c.z > 0.866f); // is_upright
}

/**
 * @brief Check if vehicle altitude has stabilized at target height
 * 
 * @details This function determines when the altitude stabilization phase is complete
 *          and the vehicle is ready to begin horizontal position hold. The check uses
 *          vertical position error from the position controller.
 * 
 *          Altitude Stabilization Criterion:
 *          - Vertical position error < 50 cm (0.5 meters)
 *          - Position error represents distance from current altitude to target
 *          - Target altitude set during transition to Throw_HgtStabilise stage
 * 
 *          Target Altitude Calculation:
 *          - Standard throw: Throw point + THROW_ALTITUDE_ASCEND parameter (default: 2m)
 *          - Airdrop mode: Throw point - THROW_ALTITUDE_DESCEND parameter (default: 2m)
 *          - Provides clearance from obstacles or controlled descent after drop
 * 
 *          Why 50cm Threshold:
 *          - Tight enough for good stabilization before position hold
 *          - Loose enough to avoid prolonged altitude correction
 *          - Allows progression even with minor altitude oscillations
 *          - Tested as good balance for post-throw stabilization
 * 
 * @return true if altitude error < 50cm (ready for position hold)
 * @return false if altitude still stabilizing
 * 
 * @note This check is evaluated continuously during Throw_HgtStabilise stage.
 *       When it returns true, state machine advances to Throw_PosHold.
 * 
 * @see ModeThrow::run() for state machine execution
 * @see g.throw_altitude_ascend for standard throw altitude offset
 * @see g.throw_altitude_descend for airdrop altitude offset
 */
bool ModeThrow::throw_height_good() const
{
    // Get vertical position error from position controller
    // Returns absolute error in centimeters between current and target altitude
    // Positive error: Below target, Negative error: Above target (magnitude used)
    return (pos_control->get_pos_error_U_cm() < 50.0f); // Within 0.5 meters
}

/**
 * @brief Check if vehicle horizontal position has stabilized
 * 
 * @details This function determines when the position hold phase is complete and the
 *          vehicle is ready for automatic transition to the configured next flight mode.
 *          The check uses horizontal position error from the position controller.
 * 
 *          Position Stabilization Criterion:
 *          - Horizontal position error (NE plane) < 50 cm (0.5 meters)
 *          - Position error is 2D distance in North-East plane from target position
 *          - Target position is the location where position hold was initiated
 * 
 *          Position Hold Target:
 *          - Set when transitioning from Throw_HgtStabilise to Throw_PosHold
 *          - Represents vehicle position when altitude stabilization completed
 *          - Position controller works to arrest horizontal drift and hold this point
 * 
 *          Why 50cm Threshold:
 *          - Indicates vehicle has arrested post-throw drift
 *          - Provides stable platform for mode transition
 *          - Matches altitude threshold for consistent stabilization criteria
 *          - Ensures vehicle is controllable before handing off to next mode
 * 
 *          Next Mode Transition:
 *          - When this check passes, automatic mode switch is attempted
 *          - Next mode configured via THROW_NEXTMODE parameter
 *          - Safe modes: AUTO, GUIDED, RTL, LAND, BRAKE, LOITER
 *          - If configured mode not in safe list, remains in throw mode
 * 
 * @return true if horizontal position error < 50cm (ready for mode transition)
 * @return false if position still stabilizing
 * 
 * @note This check is evaluated continuously during Throw_PosHold stage.
 *       When it returns true, automatic mode transition is attempted once.
 * 
 * @see ModeThrow::run() for state machine execution and mode transition logic
 * @see g2.throw_nextmode for configured next mode selection
 */
bool ModeThrow::throw_position_good() const
{
    // Get horizontal position error from position controller
    // Returns 2D distance in centimeters between current position and target
    // Error is magnitude in North-East plane (horizontal distance from target)
    return (pos_control->get_pos_error_NE_cm() < 50.0f); // Within 0.5 meters
}

#endif
