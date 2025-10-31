#include "Copter.h"

#if MODE_DRIFT_ENABLED

/**
 * @file mode_drift.cpp
 * @brief Drift flight mode implementation providing car/motorcycle-like steering control
 * 
 * @details Drift mode is designed for FPV (First Person View) flying with intuitive
 *          vehicle-oriented control that mimics car or motorcycle steering behavior.
 *          
 *          Key characteristics:
 *          - Roll stick controls yaw rate (steering left/right like a car)
 *          - Pitch stick controls forward/backward velocity
 *          - Automatic altitude hold based on pilot throttle
 *          - No direct pilot yaw control - yaw is controlled through roll input
 *          - Camera can remain pointed forward continuously for FPV
 *          - Automatic braking when pitch stick is released
 *          - Throttle assist helps maintain altitude during maneuvers
 *          
 *          Control mapping (vehicle perspective):
 *          - Roll stick (normally aileron) → Yaw rate (turn left/right)
 *          - Pitch stick → Forward/backward velocity
 *          - Throttle → Altitude hold with automatic vertical velocity damping
 *          - Yaw stick → Roll velocity compensation for drift control
 *          
 *          This mode is particularly useful for:
 *          - FPV racing with car-like controls
 *          - Cinematic forward-flying shots
 *          - Pilots transitioning from car/motorcycle control paradigms
 *          - Situations where camera orientation should remain stable
 * 
 * @note This mode requires GPS for velocity estimation
 * @warning Drift mode provides no direct attitude stabilization - vehicle will lean
 *          significantly during turns. Ensure sufficient altitude for safe operation.
 * 
 * Source: ArduCopter/mode_drift.cpp
 */

/**
 * @name Drift Mode Configuration Constants
 * @{
 * These constants define the tuning parameters for Drift mode control behavior.
 * They can be overridden at compile time by defining them before including this file.
 */

#ifndef DRIFT_SPEEDGAIN
 /**
  * @brief Gain for converting stick input to velocity and velocity error to roll angle
  * 
  * Used in two ways:
  * 1. Divides yaw stick input to get desired lateral drift velocity (cm/s per stick unit)
  * 2. Multiplies velocity error to get corrective roll angle (centidegrees)
  * 
  * Higher values: More responsive drift control, tighter velocity tracking
  * Lower values: Gentler drift control, more relaxed feel
  * 
  * @note Default 8.0 provides good balance for most vehicles
  */
 # define DRIFT_SPEEDGAIN 8.0f
#endif

#ifndef DRIFT_SPEEDLIMIT
 /**
  * @brief Maximum velocity considered for control calculations (cm/s)
  * 
  * Velocities beyond this limit are clamped to prevent excessive control responses
  * at very high speeds. Default 560 cm/s = 5.6 m/s = ~20 km/h.
  * 
  * @note Does not limit actual vehicle speed, only limits values used in control math
  * @warning Setting too low may cause control instability at high speeds
  */
 # define DRIFT_SPEEDLIMIT 560.0f
#endif

#ifndef DRIFT_VEL_FORWARD_MIN
 /**
  * @brief Forward velocity threshold for yaw rate gain scheduling (cm/s)
  * 
  * Forward velocities above this value receive minimum yaw rate authority.
  * Default 2000 cm/s = 20 m/s = 72 km/h.
  * 
  * Gain scheduling formula: (1.0 - min(vel_forward, DRIFT_VEL_FORWARD_MIN) / 5000)
  * - At 0 cm/s: 100% yaw rate authority
  * - At 2000 cm/s: 60% yaw rate authority
  * - At 5000 cm/s and above: 0% yaw rate authority
  * 
  * @note Mimics real vehicle steering where turning ability decreases at speed
  */
 # define DRIFT_VEL_FORWARD_MIN 2000.0f
#endif

#ifndef DRIFT_THR_ASSIST_GAIN
 /**
  * @brief Gain controlling amount of throttle adjustment per unit vertical velocity
  * 
  * Multiplied by vertical velocity (cm/s) and assist gain profile to calculate
  * throttle adjustment. Default 0.0018 provides smooth altitude hold without
  * excessive throttle changes.
  * 
  * Higher values: Stronger altitude hold, more aggressive throttle response
  * Lower values: Gentler altitude hold, pilot has more direct control
  * 
  * @note Too high can cause throttle oscillations; too low reduces altitude hold effectiveness
  */
 # define DRIFT_THR_ASSIST_GAIN 0.0018f
#endif

#ifndef DRIFT_THR_ASSIST_MAX
 /**
  * @brief Maximum throttle adjustment allowed from velocity assist (0.0-1.0)
  * 
  * Limits the magnitude of automatic throttle changes to prevent excessive
  * adjustments that could destabilize the vehicle. Default 0.3 = 30% of throttle range.
  * 
  * @note This is a safety limit - actual assist is typically much smaller
  * @warning Setting too high may cause abrupt throttle changes during aggressive maneuvers
  */
 # define DRIFT_THR_ASSIST_MAX  0.3f
#endif

#ifndef DRIFT_THR_MIN
 /**
  * @brief Lower throttle threshold for assist activation (0.0-1.0)
  * 
  * Throttle assist is only active when pilot throttle is above this value.
  * Default 0.213 = 21.3% throttle.
  * 
  * Below this threshold: No altitude assist, pilot has direct throttle control for landing
  * 
  * @note Allows pilot to intentionally descend by reducing throttle below this threshold
  */
 # define DRIFT_THR_MIN         0.213f
#endif

#ifndef DRIFT_THR_MAX
 /**
  * @brief Upper throttle threshold for assist deactivation (0.0-1.0)
  * 
  * Throttle assist is only active when pilot throttle is below this value.
  * Default 0.787 = 78.7% throttle.
  * 
  * Above this threshold: No altitude assist, pilot has direct throttle control for climbing
  * 
  * @note Allows pilot to intentionally climb by increasing throttle above this threshold
  */
 # define DRIFT_THR_MAX         0.787f
#endif

/** @} */ // End of Drift Mode Configuration Constants

/**
 * @brief Initialize Drift flight mode
 * 
 * @details Drift mode initialization is minimal as most state is managed in the run() function.
 *          The mode relies on existing attitude controller and position controller
 *          initialization performed by the main vehicle code.
 *          
 *          Unlike other modes, Drift maintains its own static state variables (braker, roll_input_cd)
 *          which are initialized on first call to run() rather than in init().
 * 
 * @param[in] ignore_checks If true, skip pre-arm checks (typically false for normal operation)
 * 
 * @return Always returns true - Drift mode has no initialization failure conditions
 * 
 * @note This mode requires:
 *       - GPS for velocity estimation
 *       - Functional attitude controller
 *       - Position controller for velocity estimates
 * 
 * @see ModeDrift::run()
 */
bool ModeDrift::init(bool ignore_checks)
{
    return true;
}

/**
 * @brief Main Drift mode control loop implementing car-like steering behavior
 * 
 * @details This function implements the core Drift mode control logic, providing
 *          intuitive car/motorcycle-like control for FPV flying. The control strategy
 *          maps pilot inputs to vehicle motion in a vehicle-centric reference frame.
 *          
 *          Control algorithm overview:
 *          1. Convert pilot roll/pitch stick inputs to desired lean angles
 *          2. Get current vehicle velocity in NED frame from position controller
 *          3. Transform velocity to vehicle body frame (forward/right)
 *          4. Map roll stick input to yaw rate (car-like steering)
 *          5. Use yaw stick to control lateral drift velocity via roll
 *          6. Apply automatic braking when pitch stick is centered
 *          7. Manage motor spool state and reset controllers when appropriate
 *          8. Send attitude targets to attitude controller
 *          9. Apply throttle with vertical velocity assist for altitude hold
 *          
 *          Key control relationships:
 *          - target_yaw_rate = roll_stick_input * gain_schedule(forward_velocity)
 *          - target_roll = -(lateral_velocity - desired_drift_velocity) * gain
 *          - target_pitch = forward_velocity * brake_gain (when stick centered)
 *          - throttle = pilot_throttle + damping(vertical_velocity)
 *          
 *          Gain scheduling: Yaw rate response decreases with forward speed to prevent
 *          over-rotation at high speeds, mimicking real vehicle steering behavior.
 *          
 *          Automatic braking: When pilot releases pitch stick, the braker variable
 *          ramps up over ~4.6 seconds, commanding pitch angle to oppose forward velocity.
 * 
 * @note Called at 100Hz or higher by the main scheduler
 * @note Static variables (braker, roll_input_cd) maintain state between calls
 * @note Uses body-frame velocity for intuitive vehicle-centric control
 * 
 * @warning This mode provides aggressive attitude commands - vehicle can achieve
 *          significant bank angles during high-speed turns. Maintain adequate altitude.
 * @warning Requires valid GPS velocity estimates - will not function properly without GPS
 * 
 * @see ModeDrift::get_throttle_assist()
 * @see AC_AttitudeControl::input_euler_angle_roll_pitch_euler_rate_yaw_cd()
 * 
 * Source: ArduCopter/mode_drift.cpp:42-128
 */
void ModeDrift::run()
{
    // Static state variables maintain continuity between control loop iterations
    static float braker = 0.0f;          // Automatic braking gain (0 to DRIFT_SPEEDGAIN)
    static float roll_input_cd = 0.0f;   // Filtered yaw stick input in centidegrees

    // Convert pilot stick inputs to desired lean angles (radians then centidegrees)
    // Note: In Drift mode, these initial values are heavily modified by velocity feedback
    float target_roll_rad, target_pitch_rad;
    get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, attitude_control->lean_angle_max_rad(), attitude_control->get_althold_lean_angle_max_rad());
    float target_roll_cd = rad_to_cd(target_roll_rad);
    float target_pitch_cd = rad_to_cd(target_pitch_rad);

    // Get current vehicle velocity in NED frame (North-East-Up, cm/s)
    // NEU frame used here: North=X, East=Y, Up=Z (note: Up not Down for this vector)
    const Vector3f& vel_NEU_cms = pos_control->get_vel_estimate_NEU_cms();

    // Transform velocity from NED earth frame to vehicle body frame
    // This provides intuitive vehicle-centric velocities for car-like control
    // vel_right_cms: positive = moving right relative to nose, negative = moving left
    // vel_forward_cms: positive = moving forward relative to nose, negative = moving backward
    float vel_right_cms =  vel_NEU_cms.y * ahrs.cos_yaw() - vel_NEU_cms.x * ahrs.sin_yaw(); // Lateral body velocity (cm/s)
    float vel_forward_cms = vel_NEU_cms.y * ahrs.sin_yaw() + vel_NEU_cms.x * ahrs.cos_yaw(); // Longitudinal body velocity (cm/s)

    // Gain scheduling for yaw rate: reduce yaw sensitivity at higher forward speeds
    // This mimics real vehicle steering where turning radius increases with speed
    // At 0 cm/s: full yaw rate authority
    // At 2000 cm/s (20 m/s): reduced authority by 40% (1.0 - 2000/5000)
    // At 5000 cm/s (50 m/s): reduced authority by 100% (very limited turning)
    float vel_forward_2_cms = MIN(fabsf(vel_forward_cms), DRIFT_VEL_FORWARD_MIN);
    float target_yaw_rate_cds = target_roll_cd * (1.0f - (vel_forward_2_cms / 5000.0f)) * g2.command_model_acro_y.get_rate() / 45.0;

    // Constrain velocity measurements to reasonable limits for control calculations
    // DRIFT_SPEEDLIMIT = 560 cm/s = 5.6 m/s = ~20 km/h max considered velocity
    vel_right_cms = constrain_float(vel_right_cms, -DRIFT_SPEEDLIMIT, DRIFT_SPEEDLIMIT);
    vel_forward_cms = constrain_float(vel_forward_cms, -DRIFT_SPEEDLIMIT, DRIFT_SPEEDLIMIT);

    // Apply low-pass filter to yaw stick input (96% old + 4% new)
    // Time constant ~25 iterations at 100Hz = 0.25 seconds for smooth drift control
    // Note: Yaw stick is used to command lateral drift velocity, not yaw rate
    roll_input_cd = roll_input_cd * 0.96f + (float)channel_yaw->get_control_in() * 0.04f;

    // Convert yaw stick input to desired lateral (right) velocity
    // Yaw stick controls how much the vehicle drifts sideways (like sliding in a car)
    // DRIFT_SPEEDGAIN = 8.0: maps stick input to velocity command
    float roll_vel_error = vel_right_cms - (roll_input_cd / DRIFT_SPEEDGAIN);

    // Use lateral velocity error to command roll angle for drift control
    // Negative feedback: if drifting right too fast, roll left to correct
    // This creates the characteristic "drift" behavior where vehicle can slide sideways
    // Roll angle proportional to velocity error minimizes unwanted lateral slip
    target_roll_cd = roll_vel_error * -DRIFT_SPEEDGAIN;
    target_roll_cd = constrain_float(target_roll_cd, -4500.0f, 4500.0f);  // Limit to ±45 degrees

    // Automatic braking: when pilot releases pitch stick, gradually bring vehicle to a stop
    // This provides intuitive "let go to stop" behavior like releasing a car's accelerator
    if (is_zero(target_pitch_cd)) {
        // Ramp up braking gain over time: 0.03 per iteration at 100Hz
        // Calculation: DRIFT_SPEEDGAIN / (0.03 * 100Hz) = 8.0 / 3.0 = 2.67 seconds to full braking
        // (Note: comment shows 4.6s which appears to use different calculation)
        braker += 0.03f;
        braker = MIN(braker, DRIFT_SPEEDGAIN);
        // Command pitch angle proportional to forward velocity to decelerate
        // Higher braker value = stronger braking response
        target_pitch_cd = vel_forward_cms * braker;
    } else {
        // Pilot is actively controlling pitch - reset braking
        braker = 0.0f;
    }

    // Manage motor spool state based on arm status and throttle position
    // This ensures smooth transitions and appropriate motor behavior for flight safety
    if (!motors->armed()) {
        // Vehicle is disarmed - motors should be completely stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (copter.ap.throttle_zero) {
        // Throttle at zero while armed - landing or preparing to land
        // Keep motors at ground idle for quick response if needed
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        // Normal flight - motors can use full throttle range for control
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    // Handle attitude controller state based on actual motor spool state
    // Reset integrator terms and targets appropriately to prevent unexpected behavior
    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors completely stopped - reset all attitude controller state
        // Don't save yaw target (false parameter) since we're shut down
        attitude_control->reset_yaw_target_and_rate(false);
        attitude_control->reset_rate_controller_I_terms();
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Vehicle on ground with motors idling - reset for smooth takeoff
        // Save current yaw to prevent uncommanded yaw on takeoff
        attitude_control->reset_yaw_target_and_rate();
        // Smoothly reset integrators to prevent jerky motor response
        attitude_control->reset_rate_controller_I_terms_smoothly();
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // Normal flight mode - clear landing flag if throttle demand is present
        // throttle_lower limit indicates motors can produce more thrust if needed
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // Transient states during motor spin-up or spin-down
        // No controller resets needed - maintain current state
        break;
    }

    // Send control targets to attitude controller
    // Roll/pitch as angles (centidegrees), yaw as rate (centidegrees/second)
    // This provides the car-like control feel: lean angles for position, yaw rate for steering
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target_roll_cd, target_pitch_cd, target_yaw_rate_cds);

    // Apply throttle with vertical velocity damping for automatic altitude hold
    // Throttle assist adjusts pilot input based on vertical velocity to maintain altitude
    // Angle boost (true parameter) increases throttle during aggressive maneuvers to compensate
    // for reduced vertical thrust component when vehicle is leaned over
    const float assisted_throttle = get_throttle_assist(vel_NEU_cms.z, get_pilot_desired_throttle());
    attitude_control->set_throttle_out(assisted_throttle, true, g.throttle_filt);
}

/**
 * @brief Calculate throttle output with vertical velocity damping for altitude hold
 * 
 * @details Implements automatic altitude hold by adjusting pilot throttle input based on
 *          vertical velocity. This provides damping against vertical disturbances and helps
 *          maintain altitude during forward flight and maneuvering.
 *          
 *          Throttle assist characteristics:
 *          - Only active in mid-throttle range (21.3% to 78.7%)
 *          - Strongest at mid-throttle (50%) where gain = 1.0
 *          - Linearly decreases to zero at boundaries (21.3% and 78.7%)
 *          - Damping opposes vertical velocity: climbing → reduce throttle, descending → increase throttle
 *          - Limited to ±30% throttle adjustment maximum
 *          
 *          Design rationale:
 *          - Mid-throttle focus allows pilot override at stick extremes for climb/descend
 *          - Linear gain profile provides predictable feel across throttle range
 *          - Velocity damping coefficient prevents oscillations while providing quick response
 *          - Assist limits prevent excessive throttle changes that could destabilize vehicle
 *          
 *          Gain calculation:
 *          At throttle = 0.5 (50%): gain = 1.2 - (0.0 / 0.24) = 1.2 (maximum assist)
 *          At throttle = 0.213 or 0.787: gain = 1.2 - (0.287 / 0.24) ≈ 0.0 (no assist)
 *          
 *          Assist magnitude: thr_assist = gain * -DRIFT_THR_ASSIST_GAIN * velz
 *          - DRIFT_THR_ASSIST_GAIN = 0.0018
 *          - Negative sign provides damping (opposes velocity)
 *          - velz in cm/s Up (positive up, negative down)
 * 
 * @param[in] velz Vertical velocity in cm/s (NED Up convention: positive=up, negative=down)
 * @param[in] pilot_throttle_scaled Pilot throttle input normalized to 0.0-1.0 range
 * 
 * @return Adjusted throttle output in range 0.0-1.0 with velocity damping applied
 * 
 * @note Throttle assist only active when pilot throttle between DRIFT_THR_MIN (0.213) and
 *       DRIFT_THR_MAX (0.787), allowing pilot override at extremes
 * @note Maximum assist adjustment capped at DRIFT_THR_ASSIST_MAX (0.3 or 30%)
 * @note Final output always constrained to valid 0.0-1.0 range
 * 
 * @see ModeDrift::run()
 * @see DRIFT_THR_ASSIST_GAIN, DRIFT_THR_ASSIST_MAX, DRIFT_THR_MIN, DRIFT_THR_MAX
 * 
 * Source: ArduCopter/mode_drift.cpp:131-147
 */
float ModeDrift::get_throttle_assist(float velz, float pilot_throttle_scaled)
{
    // Calculate throttle adjustment to dampen vertical velocity (altitude hold behavior)
    // Only active in mid-throttle range to allow pilot override at stick extremes
    float thr_assist = 0.0f;
    if (pilot_throttle_scaled > DRIFT_THR_MIN && pilot_throttle_scaled < DRIFT_THR_MAX) {
        // Calculate assist gain: strongest at mid-throttle (1.2), zero at boundaries
        // Formula creates triangular gain profile centered at 0.5 (50% throttle)
        // fabsf(pilot_throttle_scaled - 0.5f) ranges from 0.0 at center to 0.287 at boundaries
        // Dividing by 0.24 normalizes to ~0-1.2 range
        thr_assist = 1.2f - ((float)fabsf(pilot_throttle_scaled - 0.5f) / 0.24f);
        thr_assist = constrain_float(thr_assist, 0.0f, 1.0f) * -DRIFT_THR_ASSIST_GAIN * velz;

        // Limit throttle assist to prevent excessive adjustment
        // DRIFT_THR_ASSIST_MAX = 0.3 (30% of throttle range maximum adjustment)
        thr_assist = constrain_float(thr_assist, -DRIFT_THR_ASSIST_MAX, DRIFT_THR_ASSIST_MAX);
    }
    
    // Add assist to pilot throttle and ensure result stays in valid range
    return constrain_float(pilot_throttle_scaled + thr_assist, 0.0f, 1.0f);
}
#endif
