/**
 * @file Attitude.cpp
 * @brief Attitude control integration and pilot input transformation for ArduCopter
 *
 * @details This file implements the interface layer between pilot inputs, flight mode logic,
 *          and the AC_AttitudeControl library. It handles:
 *          - Rate controller scheduling and timing integration
 *          - Throttle control and hover throttle learning
 *          - Pilot input scaling and transformation
 *          - Coordinate frame transformations for attitude control
 *
 *          The functions in this file bridge high-level flight commands with low-level
 *          attitude controllers, ensuring proper timing, scaling, and frame conversions.
 *
 * @note This code runs at main loop rate (typically 400Hz for attitude, 100Hz for throttle)
 * @warning Modifications to throttle or attitude control can affect vehicle stability
 *
 * @see AC_AttitudeControl for low-level attitude control implementation
 * @see AC_PosControl for position control integration
 */

#include "Copter.h"

/*************************************************************
 *  Attitude Rate controllers and timing
 ****************************************************************/

/**
 * @brief Update rate controller when run from main thread (normal operation)
 *
 * @details This function integrates the attitude rate controller with the main scheduler loop.
 *          It performs several critical tasks:
 *          1. Updates loop timing for attitude and position controllers based on actual loop execution time
 *          2. Updates motor library timing for proper PWM/DShot output generation
 *          3. Executes rate controller if not using dedicated rate thread
 *          4. Resets temporary rate controller inputs (sysid, feedforward)
 *
 *          The rate controller converts desired angular rates (from attitude controller)
 *          into motor outputs, forming the innermost control loop in the attitude control hierarchy.
 *
 *          Control Loop Hierarchy:
 *          Position Control → Attitude Control → Rate Control → Motor Mixing → ESC Outputs
 *
 * @note Called at main loop rate (typically 400Hz)
 * @note If using_rate_thread is true, rate_controller_run() is called from fast loop instead
 *
 * @warning Critical timing-sensitive code path - affects vehicle stability and control precision
 *
 * @see Copter::fast_loop() for rate thread execution path
 * @see AC_AttitudeControl::rate_controller_run() for rate control implementation
 * @see AC_AttitudeControl::rate_controller_target_reset() for temporary input reset
 */
void Copter::run_rate_controller_main()
{
    // set attitude and position controller loop time
    const float last_loop_time_s = AP::scheduler().get_last_loop_time_s();
    pos_control->set_dt_s(last_loop_time_s);
    attitude_control->set_dt_s(last_loop_time_s);

    if (!using_rate_thread) {
        motors->set_dt_s(last_loop_time_s);
        // only run the rate controller if we are not using the rate thread
        attitude_control->rate_controller_run();
    }
    // reset sysid and other temporary inputs
    attitude_control->rate_controller_target_reset();
}

/*************************************************************
 *  throttle control
 ****************************************************************/

/**
 * @brief Update estimated throttle required to hover (hover throttle learning algorithm)
 *
 * @details This function implements an adaptive hover throttle learning algorithm that continuously
 *          estimates the throttle position required to maintain a level hover. This learned value is
 *          critical for altitude control performance as it:
 *          - Provides feedforward throttle for position control
 *          - Adapts to changing vehicle mass (battery consumption, payload changes)
 *          - Accounts for air density variations (altitude, temperature, humidity)
 *          - Improves altitude hold stability by reducing integrator wind-up
 *
 *          Learning Algorithm:
 *          The function uses a low-pass filter (time constant 0.01) to estimate hover throttle when:
 *          1. Vehicle is armed and airborne (not landed)
 *          2. Not in standby mode
 *          3. Flight mode uses automatic throttle control
 *          4. Vehicle is in level hover (not climbing/descending)
 *          5. Vertical velocity is near zero (< 0.6 m/s)
 *          6. Roll/pitch angles are small (< 5 degrees from trim)
 *
 *          Safety Guards:
 *          - Does not learn during manual throttle operation (prevents learning bad values)
 *          - Does not learn during climb/descent (prevents biased estimates)
 *          - Requires valid velocity estimate from EKF (prevents learning during GPS glitches)
 *          - Requires level attitude to avoid gravity vector projection errors
 *
 *          Integration with Motor Library:
 *          The learned hover throttle is stored in AP_Motors and used by:
 *          - AC_PosControl for altitude hold feedforward
 *          - Flight modes for smooth transitions to automatic control
 *          - Failsafe modes for initial throttle on mode entry
 *
 * @note Called at 100Hz from main scheduler loop
 * @note Learning rate: TC=0.01 means ~10 seconds to reach 63% of new hover throttle value
 * @note For helicopters, automatically accounts for hover roll trim compensation
 *
 * @warning Does not learn in manual throttle modes or Drift mode
 * @warning Requires accurate velocity estimates from EKF - will not update during GPS glitches
 * @warning Learning may be slower at high vibration levels (velocity estimate quality check)
 *
 * @see AP_Motors::update_throttle_hover() for low-pass filter implementation
 * @see AP_Motors::get_throttle_hover() to retrieve current learned value
 * @see AC_PosControl for usage in altitude control feedforward
 * @see Mode::has_manual_throttle() for flight mode classification
 */
void Copter::update_throttle_hover()
{
    // Safety check: if not armed or landed or on standby then exit
    // Prevents learning during ground operation or when motors are stopped
    if (!motors->armed() || ap.land_complete || standby_active) {
        return;
    }

    // Do not update in manual throttle modes or Drift mode
    // Learning during manual throttle could capture pilot error or non-hover conditions
    // Drift mode excluded because it uses manual throttle control
    if (flightmode->has_manual_throttle() || (copter.flightmode->mode_number() == Mode::Number::DRIFT)) {
        return;
    }

    // Do not update while climbing or descending
    // Z-axis velocity in NEU (North-East-Up) frame from position controller
    // Non-zero desired Z velocity indicates intentional altitude change
    // Learning during climb/descent would create biased estimates (throttle != hover throttle)
    if (!is_zero(pos_control->get_vel_desired_NEU_cms().z)) {
        return;
    }

    // Do not update if no vertical velocity estimate from EKF
    // vel_d_ms: Down velocity in m/s (NED frame, positive = descending)
    // Requires reliable EKF velocity estimate and acceptable vibration levels
    // High vibration flag prevents learning during bad accelerometer data conditions
    float vel_d_ms;
    if (!AP::ahrs().get_velocity_D(vel_d_ms, vibration_check.high_vibes)) {
        return;
    }

    // Get current throttle output (0.0 to 1.0 range)
    // This is the actual motor throttle being commanded, not pilot input
    float throttle = motors->get_throttle();

    // Calculate average throttle if we are in a level hover
    // Level hover conditions (all must be satisfied):
    // 1. Throttle > 0 (motors running)
    // 2. Vertical velocity < 0.6 m/s (nearly stationary in altitude)
    // 3. Roll angle within 5° of trim (accounts for helicopter hover roll compensation)
    // 4. Pitch angle within 5° of level (vehicle not significantly tilted)
    //
    // For helicopters: roll trim compensation is applied to account for tail rotor thrust
    // requiring a small roll angle to maintain position during hover
    if ((throttle > 0.0f) && (fabsf(vel_d_ms) < 0.6) &&
        (fabsf(ahrs.get_roll_rad() - attitude_control->get_roll_trim_rad()) < radians(5)) && (labs(ahrs.get_pitch_rad()) < radians(5))) {
        
        // Update hover throttle estimate with low-pass filter (time constant = 0.01)
        // This smoothly adjusts the learned value toward current throttle
        // Time constant of 0.01 at 100Hz → ~10 seconds to 63% convergence
        motors->update_throttle_hover(0.01f);
        
#if HAL_GYROFFT_ENABLED
        // If GyroFFT is enabled, also update the frequency domain hover learning
        // This helps with notch filter optimization for hover flight conditions
        gyro_fft.update_freq_hover(0.01f, motors->get_throttle_out());
#endif
    }
}

/**
 * @brief Transform pilot's throttle input to desired climb rate
 *
 * @details Converts pilot throttle stick position to desired vertical velocity in cm/s.
 *          This function implements a non-linear input scaling with configurable deadzone
 *          centered around mid-stick position. Key features:
 *
 *          Input Scaling Algorithm:
 *          - Mid-stick position → 0 cm/s (hover/altitude hold)
 *          - Above mid-stick → positive climb rate (0 to PILOT_SPEED_UP cm/s)
 *          - Below mid-stick → negative descent rate (-PILOT_SPEED_DN to 0 cm/s)
 *          - Deadzone around mid-stick → 0 cm/s (prevents drift from stick centering errors)
 *
 *          Deadzone Implementation:
 *          Throttle input range: 0-1000 (representing stick position)
 *          Deadzone: ±THROTTLE_DEADZONE around mid-stick
 *          Within deadzone: Output = 0 (altitude hold)
 *          Outside deadzone: Linear scaling from deadzone edge to max rate
 *
 *          Toy Mode Integration:
 *          If toy mode is enabled (beginner-friendly mode), throttle input is adjusted to:
 *          - Prevent throttle from being reduced too quickly
 *          - Slow descent rate when close to ground
 *          - Provide more stable altitude control for new pilots
 *
 *          Failsafe Handling:
 *          Returns 0 cm/s if RC input is invalid (failsafe condition)
 *          This causes altitude hold behavior during radio loss
 *
 * @return Desired climb rate in cm/s (positive = up, negative = down, 0 = hold altitude)
 *         Range: -PILOT_SPEED_DN to +PILOT_SPEED_UP cm/s (typically ±250 to ±500 cm/s)
 *
 * @note Called by altitude-controlling flight modes (AltHold, Loiter, Auto, etc.)
 * @note Throttle input range: 0-1000, where get_throttle_mid() defines hover position
 * @note Units: cm/s (centimeters per second) for consistency with position controller
 *
 * @warning Returns 0 during RC failsafe - vehicle will attempt altitude hold
 * @warning Deadzone constrained to 0-400 to prevent misconfiguration
 *
 * @see get_throttle_mid() for mid-stick position calculation
 * @see get_pilot_speed_dn() for descent rate parameter selection
 * @see AC_PosControl::set_vel_desired_UP_cms() for usage in altitude control
 * @see g.pilot_speed_up parameter for maximum climb rate configuration
 * @see g2.pilot_speed_dn parameter for maximum descent rate configuration
 * @see g.throttle_deadzone parameter for deadzone size configuration
 */
float Copter::get_pilot_desired_climb_rate()
{
    // Throttle failsafe check: return 0 if RC input is invalid
    // During RC failsafe, returning 0 causes altitude hold behavior
    // This is safer than continuing last commanded rate
    if (!rc().has_valid_input()) {
        return 0.0f;
    }

    // Get raw throttle input from pilot (0-1000 range)
    // This represents stick position, not actual motor throttle
    float throttle_control = copter.channel_throttle->get_control_in();

#if TOY_MODE_ENABLED
    // Toy mode adjustment for beginner-friendly flight
    // Modifies throttle input to:
    // - Prevent rapid throttle reduction after arming
    // - Reduce descent rate when close to ground (safer landing)
    // - Provide more predictable altitude control for new pilots
    if (g2.toy_mode.enabled()) {
        g2.toy_mode.throttle_adjust(throttle_control);
    }
#endif

    // Constrain throttle to valid range (0-1000)
    // Protects against invalid input values from RC or toy mode
    throttle_control = constrain_float(throttle_control, 0.0f, 1000.0f);

    // Constrain deadzone parameter to reasonable range (0-400)
    // Prevents misconfiguration that could make altitude control difficult
    // Maximum 400 allows up to 40% of stick range as deadzone
    g.throttle_deadzone.set(constrain_int16(g.throttle_deadzone, 0, 400));

    // Initialize output climb rate
    float desired_rate = 0.0f;
    
    // Calculate deadzone boundaries around mid-stick position
    // Mid-stick is typically 500, but can be adjusted for radio calibration
    const float mid_stick = get_throttle_mid();
    const float deadband_top = mid_stick + g.throttle_deadzone;
    const float deadband_bottom = mid_stick - g.throttle_deadzone;

    // Three-region input scaling:
    // Check if throttle is above, below, or in the deadband
    
    if (throttle_control < deadband_bottom) {
        // DESCENT REGION: Below the deadband
        // Linear scaling from full descent rate at bottom stick to 0 at deadband bottom
        // Formula: rate = max_descent * (input - deadband_bottom) / deadband_bottom
        // Example: stick at 0, deadband_bottom at 450
        //          rate = -500 * (0 - 450) / 450 = -500 cm/s (full descent)
        // get_pilot_speed_dn() returns configured descent rate (typically 250-500 cm/s)
        desired_rate = get_pilot_speed_dn() * (throttle_control - deadband_bottom) / deadband_bottom;
        
    } else if (throttle_control > deadband_top) {
        // CLIMB REGION: Above the deadband
        // Linear scaling from 0 at deadband top to full climb rate at top stick
        // Formula: rate = max_climb * (input - deadband_top) / (1000 - deadband_top)
        // Example: stick at 1000, deadband_top at 550
        //          rate = 250 * (1000 - 550) / (1000 - 550) = 250 cm/s (full climb)
        // g.pilot_speed_up is configured climb rate (typically 250-500 cm/s)
        desired_rate = g.pilot_speed_up * (throttle_control - deadband_top) / (1000.0f - deadband_top);
        
    } else {
        // DEADZONE REGION: Within deadband around mid-stick
        // Return 0 for altitude hold behavior
        // Prevents small stick movements or centering errors from causing altitude drift
        desired_rate = 0.0f;
    }

    return desired_rate;
}

/**
 * @brief Get a safe throttle level that should not cause takeoff
 *
 * @details Returns a throttle value that is high enough to spin motors (for pre-arm checks,
 *          ESC calibration, etc.) but low enough that it should not generate sufficient thrust
 *          to cause the vehicle to takeoff. This is used in situations where motors need to be
 *          running but the vehicle must remain on the ground.
 *
 *          The returned throttle is calculated as half of the learned hover throttle, which
 *          provides a reasonable safety margin below the thrust required for hover.
 *
 *          Typical Use Cases:
 *          - Pre-arm motor checks
 *          - ESC calibration verification
 *          - Ground resonance testing
 *          - Motor output verification without flight risk
 *
 * @return Throttle value (0.0 to 1.0) that should not cause takeoff
 *         Typically 0.2-0.3 for most multicopters (half of hover throttle ~0.4-0.6)
 *         Returns 0 if hover throttle is not yet learned or is invalid
 *
 * @note Uses learned hover throttle, so value adapts to vehicle weight and configuration
 * @note Returns 0 if hover throttle is negative or zero (safety measure)
 *
 * @warning Should only be used on the ground - not suitable for in-flight throttle limiting
 * @warning Actual thrust depends on battery voltage, motor condition, and propeller efficiency
 *
 * @see motors->get_throttle_hover() for learned hover throttle value
 * @see update_throttle_hover() for hover throttle learning algorithm
 */
float Copter::get_non_takeoff_throttle()
{
    return MAX(0,motors->get_throttle_hover()/2.0f);
}

/**
 * @brief Smooth transition from pilot-controlled throttle to autopilot altitude control
 *
 * @details This function initializes the altitude controller's PID integrator when transitioning
 *          from manual throttle control to automatic altitude control. Without this initialization,
 *          the altitude controller would start with zero integrator, causing an abrupt throttle
 *          change and altitude perturbation during the mode transition.
 *
 *          Transition Smoothing Algorithm:
 *          The function pre-loads the altitude controller's vertical acceleration PID integrator
 *          with a value that represents the current throttle error. This prevents the integrator
 *          from needing to "wind up" from zero, resulting in a smooth, bump-free transition.
 *
 *          Mathematical Approach:
 *          1. Get current pilot throttle (last value sent to attitude controller)
 *          2. Calculate error: (pilot_throttle - hover_throttle)
 *          3. Convert to acceleration units: error * 1000.0 (throttle is 0-1, accel in cm/s²)
 *          4. Initialize PID integrator with this error term
 *
 *          This approach assumes the pilot was maintaining altitude at the transition moment,
 *          so the difference between pilot throttle and hover throttle represents the
 *          steady-state error that the integrator would eventually reach.
 *
 *          Physical Interpretation:
 *          - Pilot throttle > hover: Vehicle was climbing or compensating for added load
 *          - Pilot throttle < hover: Vehicle was descending or lightly loaded
 *          - Integrator pre-load maintains this compensation during transition
 *
 *          Usage Scenarios:
 *          - Switching from STABILIZE to ALT_HOLD
 *          - Switching from manual throttle mode to LOITER
 *          - Engaging altitude hold in any flight mode
 *          - Recovering from manual throttle override
 *
 * @note Called when transitioning from manual to automatic throttle control
 * @note Integrator value is in cm/s² (vertical acceleration in NED frame)
 * @note Prevents altitude bump during mode transitions
 *
 * @warning Only call this during mode transitions - not during normal altitude control
 * @warning Assumes pilot was maintaining stable altitude at transition moment
 *
 * @see AC_PID_2D::set_integrator() for PID integrator initialization
 * @see AC_PosControl::get_accel_U_pid() for vertical acceleration PID controller access
 * @see AC_AttitudeControl::get_throttle_in() for last pilot throttle value
 * @see Mode transition handlers in mode_*.cpp for usage examples
 */
void Copter::set_accel_throttle_I_from_pilot_throttle()
{
    // Get last throttle input sent to attitude controller (0.0 to 1.0)
    // This is the pilot's throttle stick position at the moment of transition
    // Constrain to valid range as safety measure
    float pilot_throttle = constrain_float(attitude_control->get_throttle_in(), 0.0f, 1.0f);
    
    // Initialize altitude controller PID integrator with throttle error
    // Calculation: (pilot_throttle - hover_throttle) * 1000.0
    // - Throttle difference represents steady-state error pilot was compensating for
    // - Multiply by 1000.0 to convert throttle units (0-1) to acceleration units (cm/s²)
    // - This pre-loads integrator so altitude controller continues where pilot left off
    // - Result: smooth transition without altitude bump or throttle jump
    pos_control->get_accel_U_pid().set_integrator((pilot_throttle - motors->get_throttle_hover()) * 1000.0f);
}

/**
 * @brief Rotate 2D vector from body frame to North-East frame
 *
 * @details Performs a 2D coordinate frame transformation to convert vectors from the vehicle's
 *          body frame (forward/right) to the earth-fixed North-East frame. This is essential
 *          for converting pilot stick inputs and vehicle-relative commands into earth-fixed
 *          navigation commands.
 *
 *          Coordinate Frame Definitions:
 *          - Body Frame: X = forward (nose direction), Y = right (starboard)
 *          - North-East Frame: X = North, Y = East (earth-fixed, horizontal plane)
 *          - Z-axis: Not transformed (remains Up in both frames)
 *
 *          Rotation Matrix Application:
 *          The transformation uses a 2D rotation matrix based on vehicle yaw angle:
 *          [NE_x]   [cos(yaw)  -sin(yaw)] [body_x]
 *          [NE_y] = [sin(yaw)   cos(yaw)] [body_y]
 *
 *          Mathematical Derivation:
 *          NE_x = body_x * cos(yaw) - body_y * sin(yaw)
 *          NE_y = body_x * sin(yaw) + body_y * cos(yaw)
 *
 *          Common Use Cases:
 *          - Simple/Super-Simple mode: Transform pilot inputs from vehicle to earth frame
 *          - Pilot input commands: Convert stick inputs to earth-relative navigation
 *          - Velocity commands: Convert body-relative to earth-relative velocities
 *          - Waypoint navigation: Transform vehicle-relative vectors to earth frame
 *
 *          Example - Simple Mode:
 *          Pilot pushes forward stick (body_x = 1, body_y = 0)
 *          Vehicle is facing East (yaw = 90°)
 *          Result: NE_x = 0, NE_y = 1 (moves East regardless of vehicle heading)
 *
 * @param[in,out] x Input: body frame X (forward), Output: North-East frame X (North)
 * @param[in,out] y Input: body frame Y (right), Output: North-East frame Y (East)
 *
 * @note Uses current vehicle yaw from AHRS (Attitude Heading Reference System)
 * @note This is a 2D transformation - does not affect vertical (Z) component
 * @note Yaw angle convention: 0° = North, 90° = East, 180° = South, 270° = West
 *
 * @warning Requires valid AHRS yaw estimate - accuracy depends on compass/GPS health
 * @warning Magnetic declination is already compensated in AHRS yaw calculation
 *
 * @see ahrs.cos_yaw() for cached cosine of yaw angle
 * @see ahrs.sin_yaw() for cached sine of yaw angle  
 * @see Mode::get_pilot_desired_lean_angles() for usage in Simple mode
 * @see update_simple_mode() for related simple mode transformations
 */
void Copter::rotate_body_frame_to_NE(float &x, float &y)
{
    // Apply 2D rotation matrix transformation:
    // [NE_x]   [cos(yaw)  -sin(yaw)] [body_x]
    // [NE_y] = [sin(yaw)   cos(yaw)] [body_y]
    //
    // North-East X component = body_forward * cos(yaw) - body_right * sin(yaw)
    // This represents how much the body-frame vector projects onto the North axis
    float ne_x = x * ahrs.cos_yaw() - y * ahrs.sin_yaw();
    
    // North-East Y component = body_forward * sin(yaw) + body_right * cos(yaw)
    // This represents how much the body-frame vector projects onto the East axis
    float ne_y = x * ahrs.sin_yaw() + y * ahrs.cos_yaw();
    
    // Update passed-by-reference parameters with transformed values
    // Input body-frame coordinates are replaced with earth-frame coordinates
    x = ne_x;
    y = ne_y;
}

/**
 * @brief Get configured maximum descent rate from pilot input
 *
 * @details Returns the maximum descent rate that the pilot can command via throttle stick.
 *          Provides backward compatibility with older configurations where only a single
 *          pilot speed parameter existed. This function implements a fallback mechanism:
 *
 *          Decision Logic:
 *          - If PILOT_SPEED_DN parameter is configured (non-zero), return that value
 *          - If PILOT_SPEED_DN is zero (unconfigured), use PILOT_SPEED_UP as fallback
 *
 *          Rationale for Fallback:
 *          Older ArduCopter versions only had PILOT_SPEED_UP parameter for both climb and
 *          descent. By using PILOT_SPEED_UP as fallback, configurations migrated from older
 *          versions maintain consistent behavior. This ensures:
 *          - No unexpected behavior changes after firmware updates
 *          - Symmetric climb/descent rates if user hasn't configured them separately
 *          - Smooth upgrade path for legacy configurations
 *
 *          Configuration Flexibility:
 *          Separate climb/descent rates allow tuning for:
 *          - Asymmetric vehicle performance (climb often slower due to rotor efficiency)
 *          - Safety considerations (slower descent reduces ground impact risk)
 *          - Pilot preference (some prefer faster descent for efficiency)
 *          - Battery conservation (slower descent may be more efficient)
 *
 *          Typical Values:
 *          - Conservative: 150 cm/s descent, 250 cm/s climb
 *          - Standard: 250 cm/s both directions
 *          - Aggressive: 250 cm/s descent, 500 cm/s climb
 *
 * @return Maximum descent rate in cm/s (positive value representing downward velocity magnitude)
 *         Returns absolute value to ensure positive rate regardless of parameter sign
 *
 * @note Returns absolute value of parameter to handle potential negative values
 * @note Used by get_pilot_desired_climb_rate() for throttle stick scaling
 * @note const function - does not modify vehicle state
 *
 * @see g2.pilot_speed_dn parameter for descent rate configuration (0 = use PILOT_SPEED_UP)
 * @see g.pilot_speed_up parameter for climb rate configuration and fallback
 * @see get_pilot_desired_climb_rate() for usage in pilot input transformation
 */
uint16_t Copter::get_pilot_speed_dn() const
{
    // Check if separate descent rate is configured
    // g2.pilot_speed_dn = 0 means use fallback (legacy behavior)
    if (g2.pilot_speed_dn == 0) {
        // Fallback to climb rate for backward compatibility
        // Ensures configurations from older firmware versions work correctly
        // abs() handles potential negative parameter values (defensive programming)
        return abs(g.pilot_speed_up);
    } else {
        // Use configured descent rate
        // abs() ensures positive rate regardless of parameter sign
        // (parameter should be positive, but abs() provides safety)
        return abs(g2.pilot_speed_dn);
    }
}
