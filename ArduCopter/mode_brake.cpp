/**
 * @file mode_brake.cpp
 * @brief Brake flight mode implementation for aggressive vehicle deceleration
 * 
 * Brake mode provides maximum deceleration to rapidly stop vehicle movement.
 * This mode aggressively stops the vehicle using the position controller with
 * high deceleration limits (BRAKE_MODE_DECEL_RATE). It is designed to bring
 * the vehicle to a complete stop as quickly as possible from high-speed flight.
 * 
 * Key characteristics:
 * - Uses position controller with aggressive deceleration limits
 * - Stops both horizontal (NE) and vertical (U) movement
 * - Automatically transitions to Loiter mode after vehicle stops (if timeout set)
 * - Used internally by PosHold mode for rapid stopping
 * - Can be triggered as emergency stop mode
 * - Provides fastest possible stop without compromising vehicle stability
 * 
 * The mode operates by commanding zero velocity to the position controller
 * while allowing maximum deceleration rates, causing the vehicle to arrest
 * motion as rapidly as possible while maintaining attitude control.
 * 
 * @note This mode is typically not selected directly by pilots but rather
 *       triggered automatically by other modes or as an emergency response.
 * 
 * Source: ArduCopter/mode_brake.cpp
 */

#include "Copter.h"

#if MODE_BRAKE_ENABLED

/*
 * Init and run calls for brake flight mode
 */

/**
 * @brief Initialize Brake flight mode controller
 * 
 * @details Sets up the position controller for aggressive deceleration by
 *          configuring maximum speed and acceleration limits. The horizontal
 *          (NE) controller is initialized with the current velocity as the
 *          maximum speed and BRAKE_MODE_DECEL_RATE as the deceleration limit,
 *          allowing rapid stopping from any initial velocity. The vertical (U)
 *          controller is configured with fixed limits for controlled altitude
 *          hold during braking.
 *          
 *          Initialization sequence:
 *          1. Configure NE position controller with current velocity magnitude
 *             as max speed and aggressive deceleration rate
 *          2. Initialize NE position controller to begin tracking
 *          3. Configure vertical speed limits (BRAKE_MODE_SPEED_Z)
 *          4. Initialize vertical position controller if not already active
 *          5. Clear any existing timeout settings
 *          
 *          The position controller will use these limits to compute motor
 *          commands that aggressively arrest vehicle motion while maintaining
 *          stable attitude control.
 * 
 * @param[in] ignore_checks If true, skip pre-arm checks (currently unused in this mode)
 * 
 * @return true Always returns true as Brake mode can always be initialized
 * 
 * @note BRAKE_MODE_DECEL_RATE defines the aggressive deceleration limit
 *       (typically much higher than normal flight modes for rapid stopping)
 * @note The current velocity magnitude is used as the initial speed limit,
 *       allowing the controller to decelerate from any starting velocity
 * @note Vertical speed is limited to BRAKE_MODE_SPEED_Z for controlled
 *       altitude hold during horizontal braking maneuver
 * 
 * @see ModeBrake::run()
 * @see BRAKE_MODE_DECEL_RATE
 * @see BRAKE_MODE_SPEED_Z
 */
bool ModeBrake::init(bool ignore_checks)
{
    // Initialize horizontal position controller with current velocity as max speed
    // and aggressive deceleration rate (BRAKE_MODE_DECEL_RATE) for rapid stopping
    pos_control->set_max_speed_accel_NE_cm(pos_control->get_vel_estimate_NEU_cms().length(), BRAKE_MODE_DECEL_RATE);
    pos_control->set_correction_speed_accel_NE_cm(pos_control->get_vel_estimate_NEU_cms().length(), BRAKE_MODE_DECEL_RATE);

    // Initialize NE position controller to begin position/velocity tracking
    pos_control->init_NE_controller();

    // Configure vertical speed and deceleration limits for controlled altitude hold
    // Uses BRAKE_MODE_SPEED_Z for both ascent and descent speed limits
    pos_control->set_max_speed_accel_U_cm(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z, BRAKE_MODE_DECEL_RATE);
    pos_control->set_correction_speed_accel_U_cmss(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z, BRAKE_MODE_DECEL_RATE);

    // Initialize vertical position controller if not already active from previous mode
    if (!pos_control->is_active_U()) {
        pos_control->init_U_controller();
    }

    // Clear timeout setting - no automatic mode transition unless explicitly set
    _timeout_ms = 0;

    return true;
}

/**
 * @brief Run Brake flight mode controller - executes aggressive deceleration
 * 
 * @details Main control loop for Brake mode that commands zero velocity to the
 *          position controller, causing aggressive deceleration using the limits
 *          configured in init(). This function implements the fastest possible
 *          vehicle stop by:
 *          
 *          1. Commanding zero horizontal velocity (NE) to position controller
 *          2. Commanding zero vertical velocity (U) for altitude hold
 *          3. Using aggressive deceleration rates set during initialization
 *          4. Maintaining attitude control throughout the braking maneuver
 *          5. Optionally transitioning to Loiter after timeout expires
 *          
 *          The position controller computes the required attitude and thrust
 *          to achieve maximum deceleration while maintaining stability. Zero
 *          velocity and acceleration vectors are commanded, causing the controller
 *          to apply maximum deceleration (up to BRAKE_MODE_DECEL_RATE) to arrest
 *          vehicle motion.
 *          
 *          Safety features:
 *          - Disarmed/landed state immediately relaxes control and sets safe outputs
 *          - Landing detection softens position targets to allow gentle touchdown
 *          - Automatic timeout transition to Loiter or Alt Hold prevents indefinite braking
 *          
 *          This mode is typically used:
 *          - Internally by PosHold mode for rapid stopping before position hold
 *          - As emergency stop mode when rapid deceleration needed
 *          - By external controllers (e.g., companion computers) for precise stops
 * 
 * @return void
 * 
 * @note This function should be called at 100Hz or higher for smooth control
 * @note Motor spool state set to THROTTLE_UNLIMITED for full control authority
 * @note Zero yaw rate commanded (0.0f) to maintain current heading during brake
 * @note If timeout enabled via timeout_to_loiter_ms(), will auto-transition when expired
 * 
 * @warning This mode applies aggressive deceleration - ensure vehicle has sufficient
 *          control authority and is not near obstacles during braking maneuver
 * 
 * @see ModeBrake::init()
 * @see ModeBrake::timeout_to_loiter_ms()
 * @see BRAKE_MODE_DECEL_RATE
 */
void ModeBrake::run()
{
    // Safety check: if vehicle is disarmed or landed, set safe outputs and exit
    // This prevents any control action when the vehicle is not airborne
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();  // Set safe outputs for ground state
        pos_control->relax_U_controller(0.0f);  // Relax altitude controller
        return;
    }

    // Enable full motor throttle range for maximum control authority during braking
    // This ensures the position controller has full power to execute rapid deceleration
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // If vehicle may be landing, soften position targets to allow gentle touchdown
    // Prevents aggressive position holding that could cause hard landings
    if (copter.ap.land_complete_maybe) {
        pos_control->soften_for_landing_NE();
    }

    // Command zero horizontal velocity and acceleration to position controller
    // This causes aggressive deceleration using BRAKE_MODE_DECEL_RATE limit
    // The controller will compute optimal attitude to stop vehicle as quickly as possible
    Vector2f vel;   // Zero velocity target (default constructed to 0,0)
    Vector2f accel; // Zero acceleration target (default constructed to 0,0)
    pos_control->input_vel_accel_NE_cm(vel, accel);
    pos_control->update_NE_controller();

    // Execute attitude control using thrust vector from position controller
    // Zero yaw rate (0.0f) maintains current heading during braking maneuver
    attitude_control->input_thrust_vector_rate_heading_rads(pos_control->get_thrust_vector(), 0.0f);

    // Command zero climb rate for altitude hold during horizontal braking
    pos_control->set_pos_target_U_from_climb_rate_cm(0.0f);
    pos_control->update_U_controller();

    // Automatic mode transition after timeout expires (if configured)
    // MAV_CMD_SOLO_BTN_PAUSE_CLICK (Solo only) can be used to set the timeout
    // Attempts Loiter first (GPS required), falls back to Alt Hold if GPS unavailable
    if (_timeout_ms != 0 && millis()-_timeout_start >= _timeout_ms) {
        if (!copter.set_mode(Mode::Number::LOITER, ModeReason::BRAKE_TIMEOUT)) {
            copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::BRAKE_TIMEOUT);
        }
    }
}

/**
 * @brief Configure automatic mode transition timeout for Brake mode
 * 
 * @details Sets a timeout duration after which Brake mode will automatically
 *          transition to Loiter mode (or Alt Hold if Loiter unavailable).
 *          This prevents the vehicle from remaining in Brake mode indefinitely
 *          after stopping, allowing automatic transition to a position-holding
 *          mode once the vehicle has come to rest.
 *          
 *          When the timeout expires, the mode transition logic attempts:
 *          1. Transition to Loiter mode (requires GPS position fix)
 *          2. If Loiter fails, fallback to Alt Hold mode
 *          
 *          This function is typically called:
 *          - By PosHold mode to automatically resume position hold after braking
 *          - By external MAVLink commands (e.g., MAV_CMD_SOLO_BTN_PAUSE_CLICK)
 *          - By companion computers orchestrating flight behavior
 *          
 *          Setting timeout_ms to 0 disables automatic transition, causing the
 *          vehicle to remain in Brake mode until manually switched by the pilot
 *          or external controller.
 * 
 * @param[in] timeout_ms Timeout duration in milliseconds before automatic transition.
 *                       Set to 0 to disable automatic mode transition.
 * 
 * @return void
 * 
 * @note MAV_CMD_SOLO_BTN_PAUSE_CLICK (Solo drone specific) uses this function
 *       to implement pause button behavior
 * @note Timeout countdown begins immediately when this function is called,
 *       using current system time (millis()) as reference
 * @note The mode transition occurs in run() when timeout expires
 * 
 * @see ModeBrake::run()
 */
void ModeBrake::timeout_to_loiter_ms(uint32_t timeout_ms)
{
    // Record current time as timeout start reference
    _timeout_start = millis();
    // Store timeout duration (0 = disabled)
    _timeout_ms = timeout_ms;
}

#endif
