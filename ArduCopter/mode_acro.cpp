/**
 * @file mode_acro.cpp
 * @brief Acro flight mode implementation for ArduCopter
 * 
 * @details Acro mode provides rate-controlled manual flight for expert pilots,
 *          offering maximum agility and aerobatic capability. Unlike stabilize mode,
 *          pilot inputs directly command angular rates (deg/s) rather than angles,
 *          with no automatic self-leveling. Throttle is passed through directly
 *          without altitude control.
 *          
 *          Key characteristics:
 *          - Rate control: Stick inputs = desired rotation rates
 *          - No self-leveling: Aircraft maintains any attitude until commanded otherwise
 *          - Direct throttle: No altitude hold or compensation
 *          - Optional trainer mode: ACRO_TRAINER parameter enables auto-leveling limits
 *          - Air mode support: Maintains attitude control at zero throttle
 *          
 *          This mode is ideal for aerobatic flight, FPV racing, and experienced pilots
 *          who want direct rate control without stabilization assistance.
 * 
 * @see ModeAcro class definition in mode.h
 * @see AC_AttitudeControl for rate controller implementation
 */

#include "Copter.h"

#include "mode.h"

#if MODE_ACRO_ENABLED
/**
 * @brief Main control loop for Acro flight mode
 * 
 * @details Executes rate-controlled manual flight at the main loop frequency (typically 400Hz).
 *          This function converts pilot stick inputs into desired angular rates, manages motor
 *          spool states, and commands the attitude controller to track the requested rates.
 *          
 *          Control flow:
 *          1. Convert normalized pilot inputs (-1 to 1) to desired body-frame rates (rad/s)
 *          2. Manage motor spool state based on arming status and throttle position
 *          3. Reset attitude controller integrators when landed or disarmed
 *          4. Command attitude controller to track desired rates
 *          5. Pass through pilot throttle directly (no angle boost compensation)
 *          
 *          Motor spool states:
 *          - SHUT_DOWN: Motors stopped, reset all controllers and zero throttle
 *          - GROUND_IDLE: Landed, smoothly reset integrators and zero throttle
 *          - THROTTLE_UNLIMITED: Flying, allow full throttle range
 *          - SPOOLING_UP/DOWN: Transition states, no action
 *          
 *          Rate control options (ACRO_OPTIONS parameter):
 *          - RATE_LOOP_ONLY: Bypasses full attitude stabilization for purest rate control
 *          - Standard: Uses attitude stabilization with rate commands
 *          
 * @note This function is called at main loop rate (typically 400Hz)
 * @note Air mode allows attitude control even at zero throttle for improved aerobatic capability
 * 
 * @warning Acro mode provides no self-leveling - pilot must manually control all axes
 * @warning Throttle is passed through directly - no altitude hold or stability assistance
 * 
 * @see get_pilot_desired_rates_rads() for rate conversion algorithm
 * @see AC_AttitudeControl::input_rate_bf_roll_pitch_yaw_rads() for rate tracking
 */
void ModeAcro::run()
{
    // Convert normalized pilot inputs to desired body-frame angular rates (rad/s)
    // Pilot stick inputs are scaled by configured max rates and expo curves
    float target_roll_rads, target_pitch_rads, target_yaw_rads;
    get_pilot_desired_rates_rads(channel_roll->norm_input_dz(), channel_pitch->norm_input_dz(), channel_yaw->norm_input_dz(), target_roll_rads, target_pitch_rads, target_yaw_rads);

    // Determine desired motor spool state based on arming status, throttle position, and air mode
    if (!motors->armed()) {
        // Vehicle disarmed: Stop motors completely for safety
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (copter.ap.throttle_zero
               || (copter.air_mode == AirMode::AIRMODE_ENABLED && motors->get_spool_state() == AP_Motors::SpoolState::SHUT_DOWN)) {
        // Throttle at zero (attempting to land) or air mode requiring ground idle for spoolup
        // Note: throttle_zero is never true when air mode is active during flight, but motors
        // must pass through GROUND_IDLE state to properly initialize the spoolup sequence
        
        // In air mode, only an actual landing (weight on ground) will spool down the motors,
        // allowing continuous attitude control during zero-throttle maneuvers
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        // Flying with throttle above zero: Allow full throttle range for aerobatic maneuvers
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    // Get pilot's direct throttle input (0.0 to 1.0, no altitude compensation in Acro mode)
    float pilot_desired_throttle = get_pilot_desired_throttle();

    // Handle attitude controller state and throttle based on actual motor spool state
    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors completely stopped (disarmed or emergency stop)
        // Perform hard reset of attitude controller to prevent integrator windup
        attitude_control->reset_target_and_rate(true);
        attitude_control->reset_rate_controller_I_terms();
        pilot_desired_throttle = 0.0f;  // Force zero throttle output
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Motors at ground idle (landed or landing in progress)
        // Smoothly reset attitude controller to prepare for takeoff without control jumps
        attitude_control->reset_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pilot_desired_throttle = 0.0f;  // Force zero throttle output while on ground
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // Motors spooled up and flying - full throttle authority available
        // Clear landing detection flag when throttle is applied and motors are not limited
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // Motors transitioning between states - maintain current controller state
        // No action needed during transition
        break;
    }

    // Command attitude controller to track desired body-frame angular rates
    if (g2.acro_options.get() & uint8_t(AcroOptions::RATE_LOOP_ONLY)) {
        // RATE_LOOP_ONLY option: Send rate commands with minimal attitude stabilization
        // This bypasses the full attitude controller for the purest rate control response,
        // preferred by expert acro pilots and racers who want maximum responsiveness
        attitude_control->input_rate_bf_roll_pitch_yaw_2_rads(target_roll_rads, target_pitch_rads, target_yaw_rads);
    } else {
        // Standard Acro: Send rate commands through full attitude stabilization
        // Provides smoother rate tracking with better handling of disturbances
        // while still maintaining direct rate control feel
        attitude_control->input_rate_bf_roll_pitch_yaw_rads(target_roll_rads, target_pitch_rads, target_yaw_rads);
    }

    // Output pilot's direct throttle command without angle boost compensation
    // Acro mode provides no altitude hold or throttle compensation - pilot has full manual control
    // Parameters: throttle value (0.0-1.0), angle_boost (false), throttle_filter_constant
    attitude_control->set_throttle_out(pilot_desired_throttle, false, copter.g.throttle_filt);
}

/**
 * @brief Initialize Acro flight mode
 * 
 * @details Performs one-time initialization when entering Acro mode. This function
 *          configures air mode if enabled via ACRO_OPTIONS parameter and always
 *          returns success since Acro mode has no pre-flight requirements.
 *          
 *          Air mode configuration:
 *          When ACRO_OPTIONS includes AIR_MODE flag, enables continuous attitude control
 *          even at zero throttle. This allows advanced aerobatic maneuvers like:
 *          - Power-off rolls and flips
 *          - Zero-throttle knife-edge flight  
 *          - Maintaining attitude control during free-fall
 *          
 *          Air mode maintains motor authority for attitude correction by keeping motors
 *          spinning at minimum throttle instead of completely stopping them.
 * 
 * @param[in] ignore_checks Unused in Acro mode (no initialization checks required)
 * 
 * @return true Always returns true - Acro mode has no initialization failure conditions
 * 
 * @note This function is called once when the pilot switches to Acro mode
 * @note Air mode can also be controlled via auxiliary switch (see air_mode_aux_changed())
 * 
 * @see exit() for cleanup when leaving Acro mode
 * @see air_mode_aux_changed() for runtime air mode control
 */
bool ModeAcro::init(bool ignore_checks)
{
    // Enable air mode if configured in ACRO_OPTIONS parameter
    if (g2.acro_options.get() & uint8_t(AcroOptions::AIR_MODE)) {
        disable_air_mode_reset = false;  // Allow exit() to disable air mode when leaving Acro
        copter.air_mode = AirMode::AIRMODE_ENABLED;
    }

    return true;  // Acro mode always initializes successfully (no pre-checks required)
}

/**
 * @brief Cleanup when exiting Acro flight mode
 * 
 * @details Called when switching from Acro mode to another flight mode. Disables
 *          air mode if it was enabled by Acro mode configuration, unless an auxiliary
 *          switch has taken manual control of air mode.
 *          
 *          Air mode reset logic:
 *          - If air mode was enabled via ACRO_OPTIONS and no aux switch override exists,
 *            disable air mode when leaving Acro mode
 *          - If pilot used an auxiliary switch to control air mode, preserve the pilot's
 *            setting across mode changes (disable_air_mode_reset flag)
 * 
 * @note This function is called once when switching away from Acro mode
 * @note disable_air_mode_reset flag is cleared to reset state for next Acro mode entry
 * 
 * @see init() for Acro mode initialization
 * @see air_mode_aux_changed() for auxiliary switch control
 */
void ModeAcro::exit()
{
    // Disable air mode when leaving Acro, unless auxiliary switch has taken control
    if (!disable_air_mode_reset && (g2.acro_options.get() & uint8_t(AcroOptions::AIR_MODE))) {
        copter.air_mode = AirMode::AIRMODE_DISABLED;
    }
    // Reset the disable flag for next mode entry
    disable_air_mode_reset = false;
}

/**
 * @brief Handle auxiliary switch control of air mode during Acro flight
 * 
 * @details Called when pilot uses an auxiliary switch to manually enable/disable air mode
 *          while flying in Acro mode. Sets flag to prevent exit() from automatically
 *          disabling air mode, preserving the pilot's manual air mode selection when
 *          changing flight modes.
 *          
 *          This allows pilots to:
 *          - Override ACRO_OPTIONS air mode configuration during flight
 *          - Maintain their air mode preference across mode switches
 *          - Have manual control take precedence over parameter configuration
 * 
 * @note This function is called when pilot toggles air mode auxiliary switch
 * @note The disable_air_mode_reset flag persists until mode exit, giving pilot control priority
 * 
 * @see exit() for air mode cleanup logic
 * @see init() for initial air mode configuration
 */
void ModeAcro::air_mode_aux_changed()
{
    // Set flag to preserve pilot's manual air mode selection across mode changes
    disable_air_mode_reset = true;
}

/**
 * @brief Get hover throttle value for Acro mode
 * 
 * @details Returns the estimated throttle value needed to maintain hover in Acro mode.
 *          If ACRO_THR_MID parameter is configured (positive value), uses that value
 *          to allow pilots to set a custom mid-throttle point for their specific vehicle.
 *          Otherwise, falls back to the global learned hover throttle value.
 *          
 *          Note: While Acro mode doesn't perform altitude hold, this value is used
 *          internally by the attitude controller for throttle scaling and compensation
 *          calculations during aggressive maneuvers.
 * 
 * @return Hover throttle value (0.0 to 1.0), either ACRO_THR_MID parameter or learned value
 * 
 * @note ACRO_THR_MID parameter allows per-mode hover throttle customization
 * @note This value doesn't affect direct throttle passthrough in Acro mode
 * 
 * @see Mode::throttle_hover() for global learned hover throttle
 */
float ModeAcro::throttle_hover() const
{
    // Use ACRO_THR_MID parameter if configured (positive value)
    if (is_positive(g2.acro_thr_mid)) {
        return g2.acro_thr_mid;
    }
    // Otherwise use global learned hover throttle value
    return Mode::throttle_hover();
}

/**
 * @brief Transform normalized pilot inputs into desired angular rates with optional trainer mode
 * 
 * @details Core rate conversion algorithm for Acro mode. Converts normalized pilot stick inputs
 *          (-1.0 to 1.0) into desired body-frame angular rates (rad/s) with optional exponential
 *          curves for smoother control feel. When ACRO_TRAINER parameter is enabled, adds
 *          auto-leveling corrections to help pilots learn acro flight safely.
 *          
 *          Algorithm flow:
 *          1. Apply circular limit to roll/pitch inputs (prevents >100% control authority)
 *          2. Scale inputs by maximum rate parameters (ACRO_RP_RATE, ACRO_Y_RATE)
 *          3. Apply exponential curves (ACRO_RP_EXPO, ACRO_Y_EXPO) for fine control near center
 *          4. If ACRO_TRAINER enabled, calculate auto-leveling correction rates
 *          5. Blend trainer corrections with pilot commands based on stick position
 *          
 *          ACRO_TRAINER modes:
 *          - OFF (0): Pure rate control, no auto-leveling assistance
 *          - LEVELING (1): Applies auto-leveling when sticks centered, disabled when flying inverted
 *          - LIMITED (2): Hard angle limits prevent excessive tilt, forces recovery at limits
 *          
 *          Trainer mode enables pilots to safely learn acro flight by:
 *          - Auto-leveling the aircraft when sticks are released (LEVELING mode)
 *          - Preventing excessive angles that could lead to loss of control (LIMITED mode)
 *          - Gradually reducing assistance as pilot gains confidence
 *          
 *          Exponential curves (ACRO_RP_EXPO, ACRO_Y_EXPO):
 *          - 0.0: Linear response (direct rate proportional to stick)
 *          - 0.5: Moderate expo (reduced sensitivity near center, normal at extremes)
 *          - 1.0: Maximum expo (very fine control near center, aggressive at extremes)
 * 
 * @param[in]  roll_in_norm   Normalized roll input from pilot (-1.0 to 1.0, right positive)
 * @param[in]  pitch_in_norm  Normalized pitch input from pilot (-1.0 to 1.0, forward positive)
 * @param[in]  yaw_in_norm    Normalized yaw input from pilot (-1.0 to 1.0, right positive)
 * @param[out] roll_out_rads  Desired roll rate in rad/s (body frame)
 * @param[out] pitch_out_rads Desired pitch rate in rad/s (body frame)
 * @param[out] yaw_out_rads   Desired yaw rate in rad/s (body frame)
 * 
 * @note This function is called at main loop rate (typically 400Hz)
 * @note All output rates are in body frame (not earth frame)
 * @note ACRO_TRAINER mode provides training wheels for learning acro flight
 * 
 * @warning Disabling ACRO_TRAINER without sufficient experience can lead to loss of control
 * @warning LIMITED trainer mode enforces angle limits but allows inverted flight
 * 
 * @see input_expo() for exponential curve implementation
 * @see AC_AttitudeControl::euler_rate_to_ang_vel() for frame conversions
 */
void ModeAcro::get_pilot_desired_rates_rads(float roll_in_norm, float pitch_in_norm, float yaw_in_norm, float &roll_out_rads, float &pitch_out_rads, float &yaw_out_rads)
{
    float rate_delta_max_rads;
    Vector3f rate_ef_level_rads, rate_bf_level_rads, rate_bf_request_rads;

    // Apply circular limit to pitch and roll inputs to prevent diagonal stick commands
    // from exceeding 100% authority (sqrt(pitch^2 + roll^2) <= 1.0)
    float norm_in_length = norm(pitch_in_norm, roll_in_norm);

    if (norm_in_length > 1.0) {
        // Diagonal stick input exceeds unit circle - scale back proportionally
        // This ensures maximum rate is never exceeded regardless of stick combination
        float ratio = 1.0 / norm_in_length;
        roll_in_norm *= ratio;
        pitch_in_norm *= ratio;
    }

    // Calculate desired body-frame angular rates from pilot inputs
    // Inputs are scaled by maximum rate parameters and shaped by exponential curves

    // Roll rate: Convert normalized input to rad/s with expo curve
    // ACRO_RP_RATE parameter sets maximum roll rate (deg/s)
    // ACRO_RP_EXPO parameter shapes response curve for fine control near center
    rate_bf_request_rads.x = radians(g2.command_model_acro_rp.get_rate()) * input_expo(roll_in_norm, g2.command_model_acro_rp.get_expo());

    // Pitch rate: Convert normalized input to rad/s with expo curve
    // Uses same rate and expo parameters as roll for consistent feel
    rate_bf_request_rads.y = radians(g2.command_model_acro_rp.get_rate()) * input_expo(pitch_in_norm, g2.command_model_acro_rp.get_expo());

    // Yaw rate: Convert normalized input to rad/s with separate expo curve
    // ACRO_Y_RATE and ACRO_Y_EXPO allow independent yaw axis tuning
    rate_bf_request_rads.z = radians(g2.command_model_acro_y.get_rate()) * input_expo(yaw_in_norm, g2.command_model_acro_y.get_expo());

    // ACRO_TRAINER: Calculate auto-leveling corrections to help pilots learn acro flight
    // When enabled, applies earth-frame rate corrections to return aircraft to level
    // The corrections are blended with pilot commands based on stick position and mode

    if (g.acro_trainer != (uint8_t)Trainer::OFF) {

        // Get current attitude target from attitude controller (represents vehicle orientation)
        const Vector3f att_target_euler_rad = attitude_control->get_att_target_euler_rad();

        // Calculate auto-leveling rate for roll axis (earth frame)
        // Proportional correction: larger tilt angle = faster leveling rate
        // ACRO_BAL_ROLL parameter scales the leveling aggressiveness (deg/s per degree of tilt)
        float roll_angle_rad = wrap_PI(att_target_euler_rad.x);  // Normalize to [-PI, PI]
        rate_ef_level_rads.x = -constrain_float(roll_angle_rad, -ACRO_LEVEL_MAX_ANGLE_RAD, ACRO_LEVEL_MAX_ANGLE_RAD) * g.acro_balance_roll;

        // Calculate auto-leveling rate for pitch axis (earth frame)
        // Same proportional correction as roll, using ACRO_BAL_PITCH parameter
        float pitch_angle_rad = wrap_PI(att_target_euler_rad.y);  // Normalize to [-PI, PI]
        rate_ef_level_rads.y = -constrain_float(pitch_angle_rad, -ACRO_LEVEL_MAX_ANGLE_RAD, ACRO_LEVEL_MAX_ANGLE_RAD) * g.acro_balance_pitch;

        // No auto-leveling for yaw axis - pilots maintain full yaw control in trainer mode
        rate_ef_level_rads.z = 0;

        // ACRO_TRAINER LIMITED mode: Enforce hard angle limits to prevent excessive tilt
        // When aircraft exceeds configured maximum lean angle, aggressively commands recovery
        if (g.acro_trainer == (uint8_t)Trainer::LIMITED) {
            const float angle_max_rad = attitude_control->lean_angle_max_rad();
            
            // Roll angle limit enforcement using square-root controller for smooth recovery
            if (roll_angle_rad > angle_max_rad) {
                // Aircraft tilted right beyond limit - add correction rate to bring it back
                // sqrt_controller provides time-optimal trajectory with constrained acceleration
                rate_ef_level_rads.x += sqrt_controller(angle_max_rad - roll_angle_rad, radians(g2.command_model_acro_rp.get_rate()) / ACRO_LEVEL_MAX_OVERSHOOT_RAD, attitude_control->get_accel_roll_max_radss(), G_Dt);
            } else if (roll_angle_rad < -angle_max_rad) {
                // Aircraft tilted left beyond limit - add correction rate to bring it back
                rate_ef_level_rads.x += sqrt_controller(-angle_max_rad - roll_angle_rad, radians(g2.command_model_acro_rp.get_rate()) / ACRO_LEVEL_MAX_OVERSHOOT_RAD, attitude_control->get_accel_roll_max_radss(), G_Dt);
            }

            // Pitch angle limit enforcement using square-root controller
            if (pitch_angle_rad > angle_max_rad) {
                // Aircraft pitched forward beyond limit - add correction rate
                rate_ef_level_rads.y += sqrt_controller(angle_max_rad - pitch_angle_rad, radians(g2.command_model_acro_rp.get_rate()) / ACRO_LEVEL_MAX_OVERSHOOT_RAD, attitude_control->get_accel_pitch_max_radss(), G_Dt);
            } else if (pitch_angle_rad < -angle_max_rad) {
                // Aircraft pitched backward beyond limit - add correction rate
                rate_ef_level_rads.y += sqrt_controller(-angle_max_rad - pitch_angle_rad, radians(g2.command_model_acro_rp.get_rate()) / ACRO_LEVEL_MAX_OVERSHOOT_RAD, attitude_control->get_accel_pitch_max_radss(), G_Dt);
            }
        }

        // Convert earth-frame leveling rates to body-frame rates
        // This transformation accounts for current vehicle attitude so corrections work properly
        // in all orientations (including inverted flight)
        attitude_control->euler_rate_to_ang_vel(attitude_control->get_attitude_target_quat(), rate_ef_level_rads, rate_bf_level_rads);

        // Blend pilot rate commands with trainer auto-leveling corrections
        // Different blending strategies for LIMITED vs LEVELING trainer modes
        if (g.acro_trainer == (uint8_t)Trainer::LIMITED) {
            // LIMITED mode: Always add full correction rates (hard angle limits)
            // Pilot commands are not reduced - corrections are additive to enforce limits
            rate_bf_request_rads.x += rate_bf_level_rads.x;
            rate_bf_request_rads.y += rate_bf_level_rads.y;
            rate_bf_request_rads.z += rate_bf_level_rads.z;
        } else {
            // LEVELING mode: Blend corrections based on stick position and aircraft orientation
            // When sticks centered (near zero) → full auto-leveling
            // When sticks deflected → pilot has full authority
            // cos_pitch factor disables leveling when inverted (pitch > 90°)
            float acro_level_mix = constrain_float(1-float(MAX(MAX(abs(roll_in_norm), abs(pitch_in_norm)), abs(yaw_in_norm))), 0, 1) * ahrs.cos_pitch();

            // Scale leveling rates by blend factor (reduces leveling when pilot provides input)
            rate_bf_level_rads = rate_bf_level_rads * acro_level_mix;

            // Add leveling corrections to pilot commands with reversal prevention
            // Reversal prevention ensures leveling doesn't fight pilot commands when inverted
            // by limiting correction magnitude to avoid sign changes
            
            // Roll axis: Prevent leveling from reversing pilot's intended direction
            rate_delta_max_rads = fabsf(fabsf(rate_bf_request_rads.x)-fabsf(rate_bf_level_rads.x));
            rate_bf_request_rads.x += rate_bf_level_rads.x;
            rate_bf_request_rads.x = constrain_float(rate_bf_request_rads.x, -rate_delta_max_rads, rate_delta_max_rads);

            // Pitch axis: Prevent leveling from reversing pilot's intended direction
            rate_delta_max_rads = fabsf(fabsf(rate_bf_request_rads.y)-fabsf(rate_bf_level_rads.y));
            rate_bf_request_rads.y += rate_bf_level_rads.y;
            rate_bf_request_rads.y = constrain_float(rate_bf_request_rads.y, -rate_delta_max_rads, rate_delta_max_rads);

            // Yaw axis: Prevent leveling from reversing pilot's intended direction
            rate_delta_max_rads = fabsf(fabsf(rate_bf_request_rads.z)-fabsf(rate_bf_level_rads.z));
            rate_bf_request_rads.z += rate_bf_level_rads.z;
            rate_bf_request_rads.z = constrain_float(rate_bf_request_rads.z, -rate_delta_max_rads, rate_delta_max_rads);
        }
    }

    // Return final body-frame angular rate commands (rad/s)
    // These rates include pilot inputs, expo curves, and trainer corrections (if enabled)
    roll_out_rads = rate_bf_request_rads.x;   // Roll rate (positive = right roll)
    pitch_out_rads = rate_bf_request_rads.y;  // Pitch rate (positive = nose up)
    yaw_out_rads = rate_bf_request_rads.z;    // Yaw rate (positive = nose right)
}
#endif
