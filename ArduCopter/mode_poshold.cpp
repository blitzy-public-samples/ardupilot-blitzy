/**
 * @file mode_poshold.cpp
 * @brief PosHold flight mode implementation for ArduCopter
 * 
 * @details PosHold is an intelligent position hold mode that provides more intuitive control
 *          than standard Loiter mode, especially for aggressive flying. It implements a
 *          brake-then-hold algorithm:
 *          
 *          1. When pilot provides stick input, vehicle responds with direct attitude control
 *          2. When pilot releases sticks, vehicle first BRAKES to zero velocity using
 *             dynamically calculated brake angles based on current velocity
 *          3. Once velocity reaches zero (or brake timeout occurs), vehicle transitions
 *             to GPS position hold (Loiter control)
 *          
 *          This brake-then-hold behavior is more intuitive than Loiter's immediate position
 *          hold because it allows the vehicle to naturally decelerate before locking position,
 *          reducing the jarring effect of instant position control engagement.
 *          
 *          Key Features:
 *          - Dynamic brake angle calculation based on velocity and configurable brake rate
 *          - Wind compensation estimation during loiter phase
 *          - Smooth transitions between pilot override, braking, and loiter states
 *          - Independent roll and pitch state machines for asymmetric control
 *          - Gradual mixing between control modes to avoid sudden attitude changes
 * 
 * @note This mode requires GPS and should only be enabled on vehicles with reliable
 *       position estimation (GPS, optical flow, or other positioning system)
 * 
 * @warning Aggressive brake rates can cause instability. The poshold_brake_rate_degs
 *          parameter should be tuned conservatively (default 8 deg/s recommended)
 */

#include "Copter.h"

#if MODE_POSHOLD_ENABLED

/**
 * @name PosHold Mode Constants
 * @{
 */

/** @brief Velocity threshold (cm/s) below which it is safe to transition from braking to loiter
 *  @details When vehicle velocity drops below this threshold during braking, the brake timeout
 *           is reduced to 500ms to quickly transition to loiter mode
 */
#define POSHOLD_SPEED_0                         10      // speed below which it is always safe to switch to loiter

/** @brief Maximum braking duration (ms) before forcing transition to loiter
 *  @details Prevents extended braking in edge cases where velocity doesn't reach zero.
 *           Calculated brake timeout is capped at this value.
 */
#define POSHOLD_BRAKE_TIME_ESTIMATE_MAX_MS      6000    // Maximum duration (ms) allowed for braking before transitioning to loiter

/** @brief Blend duration (ms) for transitioning from brake to loiter control
 *  @details Controls are gradually mixed from pure brake angles to pure loiter controller
 *           output over this time period for smooth transition
 */
#define POSHOLD_BRAKE_TO_LOITER_TIME_MS         1500    // Duration (ms) over which braking is blended into loiter control during BRAKE_TO_LOITER phase

/** @brief Delay (ms) after entering loiter before wind compensation estimation begins
 *  @details Allows vehicle to stabilize in loiter before starting to estimate wind compensation
 *           lean angles based on position controller output
 */
#define POSHOLD_WIND_COMP_START_TIME_MS         1500    // Delay (ms) after entering loiter before wind compensation begins updating

/** @brief Blend duration (ms) for transitioning from controller to pilot override
 *  @details When pilot takes control from brake or loiter, control is gradually mixed
 *           from autopilot output to pilot input over this duration
 */
#define POSHOLD_CONTROLLER_TO_PILOT_MIX_TIME_MS 500     // Duration (ms) over which control is blended from autopilot to pilot input during CONTROLLER_TO_PILOT_OVERRIDE

/** @brief Low-pass filter factor for smoothing pilot stick return to center
 *  @details Applied as: filtered = filtered * (1 - factor) when stick returns toward neutral.
 *           Smaller values = slower smoothing. Used to prevent abrupt attitude changes.
 */
#define POSHOLD_SMOOTH_RATE_FACTOR              0.0125f // Low-pass filter factor for smoothing pilot roll/pitch input as it returns to center

/** @brief Time constant for wind compensation low-pass filter
 *  @details Applied as: wind_comp = wind_comp * (1 - TC) + new_value * TC
 *           Smaller values = slower filter response, more smoothing
 */
#define TC_WIND_COMP                            0.0025f // Time constant for filtering wind compensation lean angle estimates (used in low-pass filter)

// definitions that are independent of main loop rate

/** @brief Stick angle threshold (centidegrees) for enabling smooth stick release filtering
 *  @details If pilot stick input exceeds this angle, the smooth release filter is bypassed
 *           and the filtered angle immediately snaps to the raw input for responsive control
 */
#define POSHOLD_STICK_RELEASE_SMOOTH_ANGLE      1800    // max angle required (in centi-degrees) after which the smooth stick release effect is applied

/** @brief Maximum velocity (cm/s) for updating wind compensation estimates
 *  @details Wind compensation is only updated when vehicle is moving slowly to avoid
 *           incorporating dynamic maneuvering accelerations into wind estimate
 */
#define POSHOLD_WIND_COMP_ESTIMATE_SPEED_MAX    10      // wind compensation estimates will only run when velocity is at or below this speed in cm/s

/** @brief Maximum wind compensation as fraction of angle_max
 *  @details Limits wind compensation to 2/3 of maximum lean angle to ensure pilot
 *           always has authority to override and maneuver the vehicle
 */
#define POSHOLD_WIND_COMP_LEAN_PCT_MAX          0.6666f // wind compensation no more than 2/3rds of angle max to ensure pilot can always override

/** @} */ // end of PosHold Mode Constants

/**
 * @brief Initialize PosHold flight mode
 * 
 * @details Performs all necessary initialization when entering PosHold mode:
 *          - Configures vertical position controller speed and acceleration limits
 *          - Initializes altitude controller if not already active
 *          - Resets pilot roll/pitch lean angle tracking to zero
 *          - Calculates brake gain based on configured brake rate parameter
 *          - Sets initial roll/pitch mode state (LOITER if landed, PILOT_OVERRIDE if flying)
 *          - Initializes loiter navigation controller target
 *          - Resets wind compensation estimates
 *          
 *          The initialization logic differentiates between landed and flying states:
 *          - If landed (ap.land_complete): Start in LOITER mode for immediate position hold
 *          - If flying: Start in PILOT_OVERRIDE to avoid sudden attitude changes that could
 *            cause a hard twitch when transitioning from another flight mode
 * 
 * @param[in] ignore_checks Not used in PosHold mode (included for interface compatibility)
 * 
 * @return true Always returns true - PosHold initialization cannot fail
 * 
 * @note This function is called once when the pilot switches to PosHold mode
 * @note Brake gain formula: (15 * brake_rate_degs + 95) * 0.01, empirically derived
 *       to provide appropriate brake response across typical brake rate range (4-20 deg/s)
 * 
 * @see ModePosHold::run()
 * @see ModePosHold::init_wind_comp_estimate()
 */
bool ModePosHold::init(bool ignore_checks)
{
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_U_cm(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_U_cmss(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise the vertical position controller
    if (!pos_control->is_active_U()) {
        pos_control->init_U_controller();
    }

    // initialise lean angles to current attitude
    pilot_roll_cd = 0.0f;
    pilot_pitch_cd = 0.0f;

    // compute brake_gain
    brake.gain = (15.0f * (float)g.poshold_brake_rate_degs + 95.0f) * 0.01f;

    if (copter.ap.land_complete) {
        // if landed begin in loiter mode
        roll_mode = RPMode::LOITER;
        pitch_mode = RPMode::LOITER;
    } else {
        // if not landed start in pilot override to avoid hard twitch
        roll_mode = RPMode::PILOT_OVERRIDE;
        pitch_mode = RPMode::PILOT_OVERRIDE;
    }

    // initialise loiter
    loiter_nav->clear_pilot_desired_acceleration();
    loiter_nav->init_target();

    // initialise wind_comp each time PosHold is switched on
    init_wind_comp_estimate();

    return true;
}

/**
 * @brief Main PosHold mode controller - executes brake-then-hold algorithm
 * 
 * @details Implements the core PosHold flight mode logic using independent roll and pitch
 *          state machines. Called every iteration of the main loop (typically 400Hz).
 *          
 *          **Brake-Then-Hold Algorithm Overview:**
 *          
 *          The mode operates through several distinct states for each axis (roll/pitch):
 *          
 *          1. **PILOT_OVERRIDE**: Pilot has stick input, direct attitude control
 *             - Vehicle responds to pilot stick commands
 *             - When sticks return near center, transitions to BRAKE
 *          
 *          2. **BRAKE**: Dynamic braking to zero velocity
 *             - Calculates brake angle proportional to current velocity
 *             - Brake angle opposes velocity to decelerate vehicle
 *             - Estimates time required to complete braking (1.5x time to level attitude)
 *             - Transitions to BRAKE_READY_TO_LOITER when brake complete
 *          
 *          3. **BRAKE_READY_TO_LOITER**: Waiting for both axes to complete braking
 *             - When BOTH roll and pitch reach this state, transitions to BRAKE_TO_LOITER
 *          
 *          4. **BRAKE_TO_LOITER**: Gradual transition from brake to loiter control
 *             - Blends brake angles with loiter controller output over 1.5 seconds
 *             - Provides smooth handoff from braking to position hold
 *             - Transitions to full LOITER when blend complete
 *          
 *          5. **LOITER**: GPS position hold using loiter controller
 *             - Maintains current GPS position
 *             - Updates wind compensation estimates
 *             - If pilot moves stick, transitions to CONTROLLER_TO_PILOT_OVERRIDE
 *          
 *          6. **CONTROLLER_TO_PILOT_OVERRIDE**: Gradual transition back to pilot control
 *             - Blends loiter output with pilot input over 500ms
 *             - Prevents abrupt attitude changes when pilot takes control
 *             - Transitions to PILOT_OVERRIDE when blend complete
 *          
 *          **Roll and Pitch Independence:**
 *          Roll and pitch axes maintain separate state machines, allowing asymmetric
 *          behavior (e.g., pilot can control roll while pitch is in loiter). When in
 *          BRAKE_TO_LOITER or LOITER, both axes are synchronized and controlled together.
 *          
 *          **Wind Compensation:**
 *          During LOITER, the controller estimates wind-induced lean angles by low-pass
 *          filtering the position controller's acceleration commands. These estimates are
 *          added to pilot inputs and brake angles in other modes to maintain position
 *          despite wind.
 *          
 *          **Altitude Control:**
 *          Vertical position is controlled independently using the standard altitude hold
 *          controller, supporting takeoff, landing, and climb/descent via throttle stick.
 * 
 * @note Called at main loop rate (typically 400Hz for most flight controllers)
 * @note Roll and pitch state machines operate independently until LOITER mode
 * @note All angles in centidegrees (0.01 degrees), velocities in cm/s, times in milliseconds
 * 
 * @warning This function must complete within the scheduler time slice. Excessive
 *          computation time will delay other critical tasks.
 * 
 * @see ModePosHold::init()
 * @see ModePosHold::update_brake_angle_from_velocity()
 * @see ModePosHold::update_wind_comp_estimate()
 */
void ModePosHold::run()
{
    const uint32_t now_ms = AP_HAL::millis();
    float controller_to_pilot_roll_mix; // mix of controller and pilot controls.  0 = fully last controller controls, 1 = fully pilot controls
    float controller_to_pilot_pitch_mix;    // mix of controller and pilot controls.  0 = fully last controller controls, 1 = fully pilot controls
    const Vector3f& vel_neu_cms = pos_control->get_vel_estimate_NEU_cms();

    // enforce minimum allowed value for poshold_brake_rate_degs
    if (g.poshold_brake_rate_degs < POSHOLD_BRAKE_RATE_MIN) {
        g.poshold_brake_rate_degs.set_and_save(POSHOLD_BRAKE_RATE_MIN);
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_U_cm(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    loiter_nav->clear_pilot_desired_acceleration();

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    float target_roll_rad, target_pitch_rad;
    get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, attitude_control->lean_angle_max_rad(), attitude_control->get_althold_lean_angle_max_rad());
    float target_roll_cd = rad_to_cd(target_roll_rad);
    float target_pitch_cd = rad_to_cd(target_pitch_rad);

    // get pilot's desired yaw rate
    float target_yaw_rate_cds = rad_to_cd(get_pilot_desired_yaw_rate_rads());

    // get pilot desired climb rate (for alt-hold mode and take-off)
    float target_climb_rate_cms = get_pilot_desired_climb_rate();
    target_climb_rate_cms = constrain_float(target_climb_rate_cms, -get_pilot_speed_dn(), g.pilot_speed_up);

    // relax loiter target if we might be landed
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Pos Hold State Machine Determination
    AltHoldModeState poshold_state = get_alt_hold_state(target_climb_rate_cms);

    // state machine
    switch (poshold_state) {

    case AltHoldModeState::MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate(false);
        pos_control->relax_U_controller(0.0f);   // forces throttle output to decay to zero
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();

        // set poshold state to pilot override
        roll_mode = RPMode::PILOT_OVERRIDE;
        pitch_mode = RPMode::PILOT_OVERRIDE;

        // initialise wind compensation estimate
        init_wind_comp_estimate();
        break;

    case AltHoldModeState::Landed_Ground_Idle:
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();
        attitude_control->reset_yaw_target_and_rate();
        init_wind_comp_estimate();
        FALLTHROUGH;

    case AltHoldModeState::Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_U_controller(0.0f);   // forces throttle output to decay to zero

        // set poshold state to pilot override
        roll_mode = RPMode::PILOT_OVERRIDE;
        pitch_mode = RPMode::PILOT_OVERRIDE;
        break;

    case AltHoldModeState::Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get avoidance adjusted climb rate
        target_climb_rate_cms = get_avoidance_adjusted_climbrate_cms(target_climb_rate_cms);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate_cms);

        // init and update loiter although pilot is controlling lean angles
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();

        // set poshold state to pilot override
        roll_mode = RPMode::PILOT_OVERRIDE;
        pitch_mode = RPMode::PILOT_OVERRIDE;
        break;

    case AltHoldModeState::Flying:
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // get avoidance adjusted climb rate
        target_climb_rate_cms = get_avoidance_adjusted_climbrate_cms(target_climb_rate_cms);

#if AP_RANGEFINDER_ENABLED
        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();
#endif

        // Send the commanded climb rate to the position controller
        pos_control->set_pos_target_U_from_climb_rate_cm(target_climb_rate_cms);
        break;
    }

    // poshold specific behaviour to calculate desired roll, pitch angles
    // Convert inertial navigation earth-frame velocities (NED) to body-frame velocities
    // This transformation is required because brake angles are calculated in body frame
    // (vehicle's perspective) while EKF provides velocities in earth frame (NED).
    // 
    // Transformation equations:
    //   vel_forward  = vel_north * cos(yaw) + vel_east * sin(yaw)
    //   vel_right    = -vel_north * sin(yaw) + vel_east * cos(yaw)
    // 
    // To-Do: move this to AP_Math (or perhaps we already have a function to do this)
    float vel_fw_cms = vel_neu_cms.x * ahrs.cos_yaw() + vel_neu_cms.y * ahrs.sin_yaw();
    float vel_right_cms = -vel_neu_cms.x * ahrs.sin_yaw() + vel_neu_cms.y * ahrs.cos_yaw();

    // If not in LOITER, retrieve latest wind compensation lean angles related to current yaw
    if (roll_mode != RPMode::LOITER || pitch_mode != RPMode::LOITER) {
        get_wind_comp_lean_angles(wind_comp_roll_cd, wind_comp_pitch_cd);
    }

    // Roll state machine
    // Each state (aka mode) is responsible for:
    //   1. Processing pilot input for this axis
    //   2. Calculating the final roll angle output to send to attitude controller
    //   3. Checking state transition conditions and initializing new state if needed
    // 
    // Roll and pitch state machines operate independently, allowing asymmetric control
    // (e.g., pilot controls roll while pitch remains in loiter). This provides intuitive
    // handling when pilot provides input on only one axis.
    switch (roll_mode) {

        case RPMode::PILOT_OVERRIDE:
            // Pilot has stick input - provide direct attitude control response
            // Update pilot desired roll angle using latest radio input.
            // The update function filters the input so that when pilot releases stick
            // (input returns toward center), the filtered angle decreases no faster
            // than the configured brake rate. This prevents abrupt attitude changes.
            update_pilot_lean_angle_cd(pilot_roll_cd, target_roll_cd);

            // Check if pilot has released stick and vehicle has nearly leveled off
            // Transition to BRAKE mode when:
            //   1. Pilot stick is at center (target_roll_cd == 0)
            //   2. Filtered pilot angle is small (< 2 * brake_rate_degs)
            // The 2x multiplier provides hysteresis to prevent rapid mode switching
            if (is_zero(target_roll_cd) && (fabsf(pilot_roll_cd) < 2 * g.poshold_brake_rate_degs)) {
                // Initialise BRAKE mode - begin dynamic braking to zero velocity
                roll_mode = RPMode::BRAKE;          // Set brake roll mode
                brake.roll_cd = 0.0f;               // initialise braking angle to zero
                brake.angle_max_roll_cd = 0.0f;     // reset brake_angle_max so we can detect when vehicle begins to flatten out during braking
                brake.start_time_roll_ms = now_ms;  // timestamp (ms) marking the start of roll-axis braking; updated during braking phase
                brake.time_updated_roll = false;    // flag the braking time can be re-estimated
            }

            // Final lean angle is pilot input plus wind compensation
            // Wind compensation maintains position drift correction even during pilot override
            roll_cd = pilot_roll_cd + wind_comp_roll_cd;
            break;

        case RPMode::BRAKE:
        case RPMode::BRAKE_READY_TO_LOITER:
            // BRAKE mode - dynamically calculate brake angle to decelerate vehicle to zero velocity
            // Brake angle is proportional to velocity with gain factor, opposing velocity direction
            // vel_right_cms is positive for rightward motion, so positive brake angle leans left to brake
            update_brake_angle_from_velocity(brake.roll_cd, vel_right_cms);

            // Dynamically estimate time required to complete braking
            // This adaptive timeout ensures we don't brake too long or transition to loiter prematurely
            if (!brake.time_updated_roll) {
                // Phase 1: Track maximum brake angle reached
                // The brake angle initially increases as the controller responds to velocity,
                // then decreases as velocity approaches zero. We track the peak to estimate brake duration.
                if (fabsf(brake.roll_cd) >= brake.angle_max_roll_cd) {
                    brake.angle_max_roll_cd = fabsf(brake.roll_cd);
                } else {
                    // Brake angle has peaked and started decreasing - vehicle is decelerating
                    // Re-start the brake timeout timer now that we know peak brake angle
                    brake.start_time_roll_ms = now_ms;
                    brake.time_updated_roll = true;
                }
            } else {
                // Phase 2: Calculate brake timeout based on observed maximum brake angle
                // Formula: timeout = 1.5 * (time to level aircraft at configured brake rate)
                //        = 1.5 * (angle_max / brake_rate)
                // The 1.5x factor provides margin for vehicle to fully decelerate and settle
                // 
                // Scaling factors:
                //   1.5 = safety margin multiplier
                //   1000 = convert seconds to milliseconds
                //   0.01 = convert centidegrees to degrees
                const uint32_t brake_timeout_roll_ms = MIN(POSHOLD_BRAKE_TIME_ESTIMATE_MAX_MS, (1.5 * 1000 * 0.01) * brake.angle_max_roll_cd / g.poshold_brake_rate_degs);

                // Velocity-based timeout reduction:
                // If velocity drops to near-zero (< 10 cm/s) and we've been braking for at least
                // 500ms, reduce remaining brake time to just 500ms to quickly transition to loiter.
                // This prevents unnecessarily long brake phase when vehicle has effectively stopped.
                if ((fabsf(vel_right_cms) <= POSHOLD_SPEED_0) && (now_ms - brake.start_time_roll_ms > 500) && (brake_timeout_roll_ms > 500)) {
                    brake.start_time_roll_ms = now_ms - brake_timeout_roll_ms + 500;
                }

                // Check if brake timeout has elapsed
                if (now_ms - brake.start_time_roll_ms > brake_timeout_roll_ms) {
                    // Braking complete for this axis - indicate ready to transition to loiter
                    // Note: Actual transition to LOITER only occurs when BOTH roll_mode and
                    // pitch_mode reach BRAKE_READY_TO_LOITER state. Logic for engaging loiter
                    // is handled below the roll and pitch mode switch statements.
                    roll_mode = RPMode::BRAKE_READY_TO_LOITER;
                }
            }

            // Final lean angle is braking angle plus wind compensation
            // Wind compensation prevents position drift during braking phase
            roll_cd = brake.roll_cd + wind_comp_roll_cd;

            // Check for pilot input during braking - allow pilot to interrupt brake
            if (!is_zero(target_roll_cd)) {
                // Pilot has moved stick - transition back to pilot control
                // Uses gradual blend to avoid sudden attitude change
                roll_controller_to_pilot_override();
            }
            break;

        case RPMode::BRAKE_TO_LOITER:
        case RPMode::LOITER:
            // these modes are combined roll-pitch modes and are handled below
            break;

        case RPMode::CONTROLLER_TO_PILOT_OVERRIDE:
            // update pilot desired roll angle using latest radio input
            //  this filters the input so that it returns to zero no faster than the brake-rate
            update_pilot_lean_angle_cd(pilot_roll_cd, target_roll_cd);

            // count-down loiter to pilot timer
            if (now_ms - controller_to_pilot_start_time_roll_ms > POSHOLD_CONTROLLER_TO_PILOT_MIX_TIME_MS) {
                // when timer runs out switch to full pilot override for next iteration
                roll_mode = RPMode::PILOT_OVERRIDE;
            }

            // calculate controller_to_pilot mix ratio
            controller_to_pilot_roll_mix = (float)(now_ms - controller_to_pilot_start_time_roll_ms) / (float)POSHOLD_CONTROLLER_TO_PILOT_MIX_TIME_MS;

            // mix final loiter lean angle and pilot desired lean angles
            roll_cd = mix_controls(controller_to_pilot_roll_mix, controller_final_roll_cd, pilot_roll_cd + wind_comp_roll_cd);
            break;
    }

    // Pitch state machine
    //  Each state (aka mode) is responsible for:
    //      1. dealing with pilot input
    //      2. calculating the final pitch output to the attitude contpitcher
    //      3. checking if the state (aka mode) should be changed and if 'yes' perform any required initialisation for the new state
    switch (pitch_mode) {

        case RPMode::PILOT_OVERRIDE:
            // update pilot desired pitch angle using latest radio input
            //  this filters the input so that it returns to zero no faster than the brake-rate
            update_pilot_lean_angle_cd(pilot_pitch_cd, target_pitch_cd);

            // switch to BRAKE mode for next iteration if no pilot input
            if (is_zero(target_pitch_cd) && (fabsf(pilot_pitch_cd) < 2 * g.poshold_brake_rate_degs)) {
                // initialise BRAKE mode
                pitch_mode = RPMode::BRAKE;         // set brake pitch mode
                brake.pitch_cd = 0.0f;              // initialise braking angle to zero
                brake.angle_max_pitch_cd = 0.0f;    // reset brake_angle_max so we can detect when vehicle begins to flatten out during braking
                brake.start_time_pitch_ms = now_ms; // timestamp (ms) marking the start of pitch-axis braking; updated during braking phase
                brake.time_updated_pitch = false;   // flag the braking time can be re-estimated
            }

            // final lean angle should be pilot input plus wind compensation
            pitch_cd = pilot_pitch_cd + wind_comp_pitch_cd;
            break;

        case RPMode::BRAKE:
        case RPMode::BRAKE_READY_TO_LOITER:
            // calculate brake_pitch angle to counter-act velocity
            update_brake_angle_from_velocity(brake.pitch_cd, -vel_fw_cms);

            // update braking time estimate
            if (!brake.time_updated_pitch) {
                // check if brake angle is increasing
                if (fabsf(brake.pitch_cd) >= brake.angle_max_pitch_cd) {
                    brake.angle_max_pitch_cd = fabsf(brake.pitch_cd);
                } else {
                    // braking angle has started decreasing so re-estimate braking time
                    brake.start_time_pitch_ms = now_ms;
                    brake.time_updated_pitch = true;
                }
            } else {
                // scaling factors:
                // 1.5 times the time taken to level the aircraft in ms
                // 1000 to convert from seconds to ms
                // 0.01 to convert angle_max_pitch_cd to degrees
                const uint32_t brake_timeout_pitch_ms = MIN(POSHOLD_BRAKE_TIME_ESTIMATE_MAX_MS, (1.5 * 1000 * 0.01) * brake.angle_max_pitch_cd / g.poshold_brake_rate_degs);

                // if velocity is very low reduce braking time to 0.5seconds
                if ((fabsf(vel_fw_cms) <= POSHOLD_SPEED_0) && (now_ms - brake.start_time_pitch_ms > 500) && (brake_timeout_pitch_ms > 500)) {
                    brake.start_time_pitch_ms = now_ms - brake_timeout_pitch_ms + 500;
                }

                if (now_ms - brake.start_time_pitch_ms > brake_timeout_pitch_ms) {
                    // indicate that we are ready to move to Loiter.
                    // Loiter will only actually be engaged once both pitch_mode and pitch_mode are changed to RPMode::BRAKE_READY_TO_LOITER
                    //  logic for engaging loiter is handled below the pitch and pitch mode switch statements
                    pitch_mode = RPMode::BRAKE_READY_TO_LOITER;
                }
            }

            // final lean angle is braking angle + wind compensation angle
            pitch_cd = brake.pitch_cd + wind_comp_pitch_cd;

            // check for pilot input
            if (!is_zero(target_pitch_cd)) {
                // init transition to pilot override
                pitch_controller_to_pilot_override();
            }
            break;

        case RPMode::BRAKE_TO_LOITER:
        case RPMode::LOITER:
            // these modes are combined pitch-pitch modes and are handled below
            break;

        case RPMode::CONTROLLER_TO_PILOT_OVERRIDE:
            // update pilot desired pitch angle using latest radio input
            //  this filters the input so that it returns to zero no faster than the brake-rate
            update_pilot_lean_angle_cd(pilot_pitch_cd, target_pitch_cd);

            // count-down loiter to pilot timer
            if (now_ms - controller_to_pilot_start_time_pitch_ms > POSHOLD_CONTROLLER_TO_PILOT_MIX_TIME_MS) {
                // when timer runs out switch to full pilot override for next iteration
                pitch_mode = RPMode::PILOT_OVERRIDE;
            }

            // calculate controller_to_pilot mix ratio
            controller_to_pilot_pitch_mix = (float)(now_ms - controller_to_pilot_start_time_pitch_ms) / (float)POSHOLD_CONTROLLER_TO_PILOT_MIX_TIME_MS;

            // mix final loiter lean angle and pilot desired lean angles
            pitch_cd = mix_controls(controller_to_pilot_pitch_mix, controller_final_pitch_cd, pilot_pitch_cd + wind_comp_pitch_cd);
            break;
    }

    //
    // Shared roll & pitch states (RPMode::BRAKE_TO_LOITER and RPMode::LOITER)
    //
    // These states operate on both roll and pitch axes simultaneously rather than
    // independently. This synchronization is necessary because the loiter controller
    // calculates combined roll/pitch outputs for position hold.
    //

    // Synchronization point: Switch into LOITER mode when both axes complete braking
    // This ensures smooth transition to position hold only after vehicle has decelerated
    // on both axes. Transitioning with residual velocity on either axis would cause
    // the loiter controller to fight the velocity, creating an uncomfortable jerk.
    if (roll_mode == RPMode::BRAKE_READY_TO_LOITER && pitch_mode == RPMode::BRAKE_READY_TO_LOITER) {
        roll_mode = RPMode::BRAKE_TO_LOITER;
        pitch_mode = RPMode::BRAKE_TO_LOITER;
        brake.loiter_transition_start_time_ms = now_ms;
        // Initialize loiter controller with current position as target
        // Account for position offset (e.g., from terrain following)
        loiter_nav->init_target_cm((pos_control->get_pos_estimate_NEU_cm().xy() - pos_control->get_pos_offset_NEU_cm().xy()).tofloat());
        // Start wind compensation estimation timer
        // Delayed start allows loiter to stabilize before estimating wind compensation
        wind_comp_start_time_ms = now_ms;
    }

    // roll-mode is used as the combined roll+pitch mode when in BRAKE_TO_LOITER or LOITER modes
    if (roll_mode == RPMode::BRAKE_TO_LOITER || roll_mode == RPMode::LOITER) {

        // force pitch mode to be same as roll_mode just to keep it consistent (it's not actually used in these states)
        pitch_mode = roll_mode;

        // handle combined roll+pitch mode
        switch (roll_mode) {
            case RPMode::BRAKE_TO_LOITER: {
                // reduce brake_to_loiter timer
                if (now_ms - brake.loiter_transition_start_time_ms > POSHOLD_BRAKE_TO_LOITER_TIME_MS) {
                    // progress to full loiter on next iteration
                    roll_mode = RPMode::LOITER;
                    pitch_mode = RPMode::LOITER;
                }

                // mix of brake and loiter controls.  0 = fully brake
                // controls, 1 = fully loiter controls
                const float brake_to_loiter_mix = (float)(now_ms - brake.loiter_transition_start_time_ms) / (float)POSHOLD_BRAKE_TO_LOITER_TIME_MS;

                // calculate brake.roll and pitch angles to counter-act velocity
                update_brake_angle_from_velocity(brake.roll_cd, vel_right_cms);
                update_brake_angle_from_velocity(brake.pitch_cd, -vel_fw_cms);

                // run loiter controller
                loiter_nav->update(false);

                // calculate final roll and pitch output by mixing loiter and brake controls
                roll_cd = mix_controls(brake_to_loiter_mix, brake.roll_cd + wind_comp_roll_cd, loiter_nav->get_roll_cd());
                pitch_cd = mix_controls(brake_to_loiter_mix, brake.pitch_cd + wind_comp_pitch_cd, loiter_nav->get_pitch_cd());

                // check for pilot input
                if (!is_zero(target_roll_cd) || !is_zero(target_pitch_cd)) {
                    // if roll input switch to pilot override for roll
                    if (!is_zero(target_roll_cd)) {
                        // init transition to pilot override
                        roll_controller_to_pilot_override();
                        // switch pitch-mode to brake (but ready to go back to loiter anytime)
                        // no need to reset brake.pitch here as wind comp has not been updated since last brake.pitch computation
                        pitch_mode = RPMode::BRAKE_READY_TO_LOITER;
                    }
                    // if pitch input switch to pilot override for pitch
                    if (!is_zero(target_pitch_cd)) {
                        // init transition to pilot override
                        pitch_controller_to_pilot_override();
                        if (is_zero(target_roll_cd)) {
                            // switch roll-mode to brake (but ready to go back to loiter anytime)
                            // no need to reset brake.roll here as wind comp has not been updated since last brake.roll computation
                            roll_mode = RPMode::BRAKE_READY_TO_LOITER;
                        }
                    }
                }
                break;
            }
            case RPMode::LOITER:
                // run loiter controller
                loiter_nav->update(false);

                // set roll angle based on loiter controller outputs
                roll_cd = loiter_nav->get_roll_cd();
                pitch_cd = loiter_nav->get_pitch_cd();

                // update wind compensation estimate
                update_wind_comp_estimate();

                // check for pilot input
                if (!is_zero(target_roll_cd) || !is_zero(target_pitch_cd)) {
                    // if roll input switch to pilot override for roll
                    if (!is_zero(target_roll_cd)) {
                        // init transition to pilot override
                        roll_controller_to_pilot_override();
                        // switch pitch-mode to brake (but ready to go back to loiter anytime)
                        pitch_mode = RPMode::BRAKE_READY_TO_LOITER;
                        // reset brake.pitch because wind_comp is now different and should give the compensation of the whole previous loiter angle
                        brake.pitch_cd = 0.0f;
                    }
                    // if pitch input switch to pilot override for pitch
                    if (!is_zero(target_pitch_cd)) {
                        // init transition to pilot override
                        pitch_controller_to_pilot_override();
                        // if roll not overridden switch roll-mode to brake (but be ready to go back to loiter any time)
                        if (is_zero(target_roll_cd)) {
                            roll_mode = RPMode::BRAKE_READY_TO_LOITER;
                            brake.roll_cd = 0.0f;
                        }
                            // if roll not overridden switch roll-mode to brake (but be ready to go back to loiter any time)
                    }
                }
                break;

            default:
                // do nothing for uncombined roll and pitch modes
                break;
        }
    }

    // constrain target pitch/roll angles
    float angle_max_cd = copter.aparm.angle_max;
    roll_cd = constrain_float(roll_cd, -angle_max_cd, angle_max_cd);
    pitch_cd = constrain_float(pitch_cd, -angle_max_cd, angle_max_cd);

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(roll_cd, pitch_cd, target_yaw_rate_cds);

    // run the vertical position controller and set output throttle
    pos_control->update_U_controller();
}

/**
 * @brief Update pilot's filtered lean angle with smooth stick release behavior
 * 
 * @details Implements intelligent filtering of pilot stick input to provide smooth
 *          transitions when pilot releases the stick toward center. The filtering
 *          behavior adapts based on stick position and direction:
 *          
 *          **Immediate Response Cases** (no filtering):
 *          - Stick input reverses direction (positive to negative or vice versa)
 *          - Stick input magnitude exceeds POSHOLD_STICK_RELEASE_SMOOTH_ANGLE (18 degrees)
 *          
 *          In these cases, filtered angle immediately snaps to raw input for
 *          responsive control during aggressive maneuvering.
 *          
 *          **Smooth Release** (filtered):
 *          When stick is returning toward center and below 18 degree threshold,
 *          the filtered angle is rate-limited to decrease no faster than the
 *          greater of:
 *          - POSHOLD_SMOOTH_RATE_FACTOR * current_angle (1.25% per iteration)
 *          - configured brake_rate (typically 8 deg/s)
 *          
 *          This prevents abrupt attitude changes when pilot releases stick,
 *          providing smooth deceleration into the brake phase.
 * 
 * @param[in,out] lean_angle_filtered_cd Filtered lean angle in centidegrees (updated)
 * @param[in]     lean_angle_raw_cd      Raw pilot stick input in centidegrees
 * 
 * @note Called once per iteration for each axis (roll and pitch independently)
 * @note All angles in centidegrees (0.01 degrees)
 * @note G_Dt is the delta time since last iteration (typically 0.0025s at 400Hz)
 * 
 * @see POSHOLD_SMOOTH_RATE_FACTOR
 * @see POSHOLD_STICK_RELEASE_SMOOTH_ANGLE
 */
void ModePosHold::update_pilot_lean_angle_cd(float &lean_angle_filtered_cd, float &lean_angle_raw_cd)
{
    // if raw input is large or reversing the vehicle's lean angle immediately set the filtered angle to the new raw angle
    if ((lean_angle_filtered_cd > 0 && lean_angle_raw_cd < 0) || (lean_angle_filtered_cd < 0 && lean_angle_raw_cd > 0) || (fabsf(lean_angle_raw_cd) > POSHOLD_STICK_RELEASE_SMOOTH_ANGLE)) {
        lean_angle_filtered_cd = lean_angle_raw_cd;
    } else {
        // lean_angle_raw_cd must be pulling lean_angle_filtered_cd towards zero, smooth the decrease
        if (lean_angle_filtered_cd > 0) {
            // reduce the filtered lean angle at 1.25% per step or the brake rate (whichever is faster).
            // poshold_brake_rate_degs is in degrees/s; multiply by 100 to convert to centidegrees/s
            lean_angle_filtered_cd -= MAX(lean_angle_filtered_cd * POSHOLD_SMOOTH_RATE_FACTOR, 100.0 * g.poshold_brake_rate_degs * G_Dt);
            // do not let the filtered angle fall below the pilot's input lean angle.
            // the above line pulls the filtered angle down and the below line acts as a catch
            lean_angle_filtered_cd = MAX(lean_angle_filtered_cd, lean_angle_raw_cd);
        }else{
            lean_angle_filtered_cd += MAX(-lean_angle_filtered_cd * POSHOLD_SMOOTH_RATE_FACTOR, 100.0 * g.poshold_brake_rate_degs * G_Dt);
            lean_angle_filtered_cd = MIN(lean_angle_filtered_cd, lean_angle_raw_cd);
        }
    }
}

/**
 * @brief Blend two control outputs based on mix ratio
 * 
 * @details Performs linear interpolation between two control values to provide
 *          smooth transitions between different control modes. Used extensively
 *          in PosHold for:
 *          - Blending brake angles with loiter controller output (BRAKE_TO_LOITER)
 *          - Blending controller output with pilot input (CONTROLLER_TO_PILOT_OVERRIDE)
 *          
 *          Formula: output = mix_ratio * first_control + (1 - mix_ratio) * second_control
 *          
 *          This linear blend prevents sudden attitude changes that would be
 *          uncomfortable and could destabilize the vehicle.
 * 
 * @param[in] mix_ratio      Blend ratio: 0.0 = all second_control, 1.0 = all first_control
 * @param[in] first_control  First control value (typically autopilot output)
 * @param[in] second_control Second control value (typically pilot input or next mode output)
 * 
 * @return Blended control output
 * 
 * @note mix_ratio is constrained to [0.0, 1.0] to prevent extrapolation
 * @note Typically called with mix_ratio = elapsed_time / blend_duration for time-based transitions
 * 
 * @see POSHOLD_BRAKE_TO_LOITER_TIME_MS
 * @see POSHOLD_CONTROLLER_TO_PILOT_MIX_TIME_MS
 */
float ModePosHold::mix_controls(float mix_ratio, float first_control, float second_control)
{
    mix_ratio = constrain_float(mix_ratio, 0.0f, 1.0f);
    return mix_ratio * first_control + (1.0f - mix_ratio) * second_control;
}

/**
 * @brief Calculate brake angle from vehicle velocity using velocity-dependent gain
 * 
 * @details Implements the core braking algorithm that calculates the lean angle required
 *          to decelerate the vehicle to zero velocity. The brake angle is calculated using
 *          a velocity-dependent gain formula that provides:
 *          - Stronger braking (higher gain) at higher velocities
 *          - Proportional response at all velocities
 *          - Smooth approach to zero as velocity decreases
 *          
 *          **Brake Angle Formula:**
 *          lean_angle = -brake_gain * velocity * (1 + 500 / (|velocity| + 60))
 *          
 *          The velocity-dependent term (1 + 500/(|vel|+60)) increases gain at higher
 *          velocities, providing more aggressive braking when needed. The negative sign
 *          ensures the lean angle opposes velocity direction (positive velocity right
 *          requires negative/left lean angle to brake).
 *          
 *          **Rate Limiting:**
 *          The brake angle is slewed (rate-limited) at the configured poshold_brake_rate_degs
 *          to prevent abrupt attitude changes. The maximum change per iteration is:
 *          brake_delta = 100 * brake_rate_degs * G_Dt (centidegrees)
 *          
 *          **Angle Limiting:**
 *          Final brake angle is constrained to ±poshold_brake_angle_max to prevent
 *          excessive lean angles that could destabilize the vehicle or violate angle limits.
 * 
 * @param[in,out] brake_angle_cd Current brake angle in centidegrees (updated with new value)
 * @param[in]     velocity_cms   Body-frame velocity in cm/s (positive = motion in lean direction)
 * 
 * @note For pitch axis, velocity should be negated (-vel_forward) because positive pitch
 *       leans backward but forward velocity is positive
 * @note For roll axis, velocity is used directly (positive = rightward motion and rightward lean)
 * @note brake_gain is calculated during init() based on poshold_brake_rate_degs parameter
 * @note G_Dt is delta time since last iteration (typically 0.0025s at 400Hz)
 * 
 * @warning Brake angle is constrained by poshold_brake_angle_max parameter (default 30 degrees)
 * 
 * @see ModePosHold::init() - where brake_gain is calculated
 * @see g.poshold_brake_rate_degs - configurable brake rate parameter
 * @see g.poshold_brake_angle_max - configurable maximum brake angle parameter
 */
void ModePosHold::update_brake_angle_from_velocity(float &brake_angle_cd, float velocity_cms)
{
    float lean_angle;
    float brake_delta_cd = 100.0f * g.poshold_brake_rate_degs * G_Dt;

    // calculate velocity-only based lean angle
    lean_angle = -brake.gain * velocity_cms * (1.0f + 500.0f / (fabsf(velocity_cms) + 60.0f));

    // do not let lean_angle be too far from brake_angle
    brake_angle_cd = constrain_float(lean_angle, brake_angle_cd - brake_delta_cd, brake_angle_cd + brake_delta_cd);

    // constrain final brake_angle
    brake_angle_cd = constrain_float(brake_angle_cd, -(float)g.poshold_brake_angle_max, (float)g.poshold_brake_angle_max);
}

/**
 * @brief Reset wind compensation estimates to zero
 * 
 * @details Initializes or resets all wind compensation state variables. Called when:
 *          - Entering PosHold mode (init)
 *          - Vehicle is landed or motors stopped
 *          - Any time wind estimates should be discarded and relearned
 *          
 *          Wind compensation estimates are learned over time during loiter mode by
 *          observing the position controller's acceleration commands needed to hold
 *          position. These estimates are then applied in other modes (pilot override,
 *          brake) to maintain position despite wind disturbances.
 * 
 * @note After reset, wind compensation will be gradually learned again during next loiter phase
 * @see ModePosHold::update_wind_comp_estimate()
 */
void ModePosHold::init_wind_comp_estimate()
{
    wind_comp_ef.zero();
    wind_comp_roll_cd = 0.0f;
    wind_comp_pitch_cd = 0.0f;
}

/**
 * @brief Update wind compensation estimates by observing position controller output
 * 
 * @details During loiter mode, the position controller applies lean angles to counteract
 *          wind and maintain position. These lean angles represent the wind disturbance
 *          affecting the vehicle. This function estimates wind compensation by low-pass
 *          filtering the position controller's acceleration commands in earth frame (NED).
 *          
 *          **Wind Estimation Algorithm:**
 *          1. Wait POSHOLD_WIND_COMP_START_TIME_MS (1500ms) after entering loiter to stabilize
 *          2. Only update when vehicle velocity < POSHOLD_WIND_COMP_ESTIMATE_SPEED_MAX (10 cm/s)
 *             to avoid incorporating maneuvering accelerations into wind estimate
 *          3. Low-pass filter position controller acceleration commands:
 *             wind_comp_ef = (1 - TC_WIND_COMP) * wind_comp_ef + TC_WIND_COMP * accel_target
 *          4. Limit total wind compensation to POSHOLD_WIND_COMP_LEAN_PCT_MAX (66.6%) of
 *             maximum lean angle to ensure pilot always retains control authority
 *          
 *          The estimated wind compensation is stored in earth frame (NED) and later
 *          transformed to body frame lean angles for application to control outputs.
 *          
 *          **Why This Works:**
 *          In steady wind, the position controller must command a constant acceleration
 *          to counteract wind drift. By filtering this acceleration over time, we extract
 *          the persistent wind component from transient positioning corrections.
 * 
 * @note Called at main loop rate (typically 400Hz) but only updates during loiter mode
 * @note Updates are delayed 1.5 seconds after entering loiter for stabilization
 * @note Updates only occur when velocity < 10 cm/s to get clean wind estimate
 * @note Wind compensation limited to 2/3 of angle_max for pilot override capability
 * 
 * @warning If this function is called when vehicle is moving fast, it will incorporate
 *          maneuvering accelerations into the wind estimate, corrupting the estimate
 * 
 * @see ModePosHold::get_wind_comp_lean_angles() - converts earth frame to body frame
 * @see TC_WIND_COMP - time constant for low-pass filter (0.0025)
 * @see POSHOLD_WIND_COMP_LEAN_PCT_MAX - maximum wind compensation fraction (0.6666)
 */
void ModePosHold::update_wind_comp_estimate()
{
    const uint32_t now_ms = AP_HAL::millis();
    // check wind estimate start has not been delayed
    if (now_ms - wind_comp_start_time_ms < POSHOLD_WIND_COMP_START_TIME_MS) {
        return;
    }

    // check horizontal velocity is low
    if (pos_control->get_vel_estimate_NEU_cms().xy().length() > POSHOLD_WIND_COMP_ESTIMATE_SPEED_MAX) {
        return;
    }

    // get position controller accel target
    const Vector3f& accel_target_neu_cmss = pos_control->get_accel_target_NEU_cmss();

    // update wind compensation in earth-frame lean angles
    if (is_zero(wind_comp_ef.x)) {
        // if wind compensation has not been initialised set it immediately to the pos controller's desired accel in north direction
        wind_comp_ef.x = accel_target_neu_cmss.x;
    } else {
        // low pass filter the position controller's lean angle output
        wind_comp_ef.x = (1.0f - TC_WIND_COMP) * wind_comp_ef.x + TC_WIND_COMP * accel_target_neu_cmss.x;
    }
    if (is_zero(wind_comp_ef.y)) {
        // if wind compensation has not been initialised set it immediately to the pos controller's desired accel in north direction
        wind_comp_ef.y = accel_target_neu_cmss.y;
    } else {
        // low pass filter the position controller's lean angle output
        wind_comp_ef.y = (1.0f - TC_WIND_COMP) * wind_comp_ef.y + TC_WIND_COMP * accel_target_neu_cmss.y;
    }

    // limit acceleration
    const float accel_lim_cmss = tanf(cd_to_rad(POSHOLD_WIND_COMP_LEAN_PCT_MAX * copter.aparm.angle_max)) * (GRAVITY_MSS * 100);
    const float wind_comp_ef_len = wind_comp_ef.length();
    if (!is_zero(accel_lim_cmss) && (wind_comp_ef_len > accel_lim_cmss)) {
        wind_comp_ef *= accel_lim_cmss / wind_comp_ef_len;
    }
}

/**
 * @brief Transform wind compensation from earth frame to body frame lean angles
 * 
 * @details Converts the wind compensation acceleration estimates (stored in earth frame NED)
 *          into body frame roll and pitch lean angles that can be added to control outputs.
 *          The transformation accounts for current vehicle yaw heading.
 *          
 *          **Transformation Process:**
 *          1. Earth frame wind compensation is in NED (North-East-Down) coordinates
 *          2. Body frame requires right/forward coordinates relative to vehicle heading
 *          3. Rotation by yaw angle transforms between frames:
 *             accel_right   = -wind_comp_north * sin(yaw) + wind_comp_east * cos(yaw)
 *             accel_forward = -wind_comp_north * cos(yaw) - wind_comp_east * sin(yaw)
 *          4. Convert accelerations to lean angles using small angle approximation:
 *             roll_angle  = atan(accel_right / gravity)
 *             pitch_angle = atan(accel_forward / gravity)
 *          5. Convert radians to centidegrees: angle_cd = angle_rad * (18000 / π)
 *          
 *          The negative signs in the transformation ensure that:
 *          - Positive north wind requires negative pitch (lean forward into wind)
 *          - Positive east wind requires positive roll (lean right into wind)
 * 
 * @param[out] roll_angle_cd  Body frame roll lean angle in centidegrees
 * @param[out] pitch_angle_cd Body frame pitch lean angle in centidegrees
 * 
 * @note Called at main loop rate (typically 400Hz) whenever wind compensation is needed
 * @note Uses current AHRS yaw angle for frame transformation
 * @note GRAVITY_MSS is in m/s², wind_comp_ef is in cm/s², hence the factor of 100
 * @note Result in centidegrees: 18000 centidegrees = 180 degrees = π radians
 * 
 * @see ModePosHold::update_wind_comp_estimate() - where wind_comp_ef is calculated
 */
void ModePosHold::get_wind_comp_lean_angles(float &roll_angle_cd, float &pitch_angle_cd)
{
    // convert earth frame desired accelerations to body frame roll and pitch lean angles
    roll_angle_cd = atanf((-wind_comp_ef.x * ahrs.sin_yaw() + wind_comp_ef.y * ahrs.cos_yaw()) / (GRAVITY_MSS * 100)) * (18000.0f / M_PI);
    pitch_angle_cd = atanf(-(wind_comp_ef.x * ahrs.cos_yaw() + wind_comp_ef.y * ahrs.sin_yaw()) / (GRAVITY_MSS * 100)) * (18000.0f / M_PI);
}

/**
 * @brief Initialize smooth transition from autopilot to pilot control on roll axis
 * 
 * @details When pilot moves roll stick while in brake or loiter mode, this function
 *          initiates a gradual transition to give control back to the pilot. Rather than
 *          instantly switching to pilot input (which could cause a sudden attitude change),
 *          the output is blended over POSHOLD_CONTROLLER_TO_PILOT_MIX_TIME_MS (500ms).
 *          
 *          **Transition Process:**
 *          1. Change state to CONTROLLER_TO_PILOT_OVERRIDE
 *          2. Record current time as transition start
 *          3. Reset pilot_roll_cd to 0 (pilot input tracking starts fresh)
 *          4. Store current controller output (controller_final_roll_cd) for blending
 *          5. Over next 500ms, gradually increase pilot control from 0% to 100%
 *          
 *          Starting pilot_roll_cd at 0 rather than current attitude allows the
 *          update_pilot_lean_angle_cd function to smoothly ramp up pilot authority
 *          without an initial step change. Wind compensation will adjust to compensate
 *          during the transition.
 * 
 * @note Called when pilot moves roll stick while in brake or loiter mode
 * @note Transition duration is 500ms (POSHOLD_CONTROLLER_TO_PILOT_MIX_TIME_MS)
 * @note Roll and pitch axes transition independently
 * 
 * @see POSHOLD_CONTROLLER_TO_PILOT_MIX_TIME_MS
 * @see ModePosHold::mix_controls() - performs the blending calculation
 */
void ModePosHold::roll_controller_to_pilot_override()
{
    const uint32_t now_ms = AP_HAL::millis();
    roll_mode = RPMode::CONTROLLER_TO_PILOT_OVERRIDE;
    controller_to_pilot_start_time_roll_ms = now_ms;
    // initialise pilot_roll_cd to 0, wind_comp will be updated to compensate and poshold_update_pilot_lean_angle function shall not smooth this transition at next iteration. so 0 is the right value
    pilot_roll_cd = 0.0f;
    // store final controller output for mixing with pilot input
    controller_final_roll_cd = roll_cd;
}

/**
 * @brief Initialize smooth transition from autopilot to pilot control on pitch axis
 * 
 * @details When pilot moves pitch stick while in brake or loiter mode, this function
 *          initiates a gradual transition to give control back to the pilot. Rather than
 *          instantly switching to pilot input (which could cause a sudden attitude change),
 *          the output is blended over POSHOLD_CONTROLLER_TO_PILOT_MIX_TIME_MS (500ms).
 *          
 *          **Transition Process:**
 *          1. Change state to CONTROLLER_TO_PILOT_OVERRIDE
 *          2. Record current time as transition start
 *          3. Reset pilot_pitch_cd to 0 (pilot input tracking starts fresh)
 *          4. Store current controller output (controller_final_pitch_cd) for blending
 *          5. Over next 500ms, gradually increase pilot control from 0% to 100%
 *          
 *          Starting pilot_pitch_cd at 0 rather than current attitude allows the
 *          update_pilot_lean_angle_cd function to smoothly ramp up pilot authority
 *          without an initial step change. Wind compensation will adjust to compensate
 *          during the transition.
 * 
 * @note Called when pilot moves pitch stick while in brake or loiter mode
 * @note Transition duration is 500ms (POSHOLD_CONTROLLER_TO_PILOT_MIX_TIME_MS)
 * @note Roll and pitch axes transition independently
 * 
 * @see POSHOLD_CONTROLLER_TO_PILOT_MIX_TIME_MS
 * @see ModePosHold::mix_controls() - performs the blending calculation
 */
void ModePosHold::pitch_controller_to_pilot_override()
{
    const uint32_t now_ms = AP_HAL::millis();
    pitch_mode = RPMode::CONTROLLER_TO_PILOT_OVERRIDE;
    controller_to_pilot_start_time_pitch_ms = now_ms;
    // initialise pilot_pitch_cd to 0, wind_comp will be updated to compensate and update_pilot_lean_angle_cd function shall not smooth this transition at next iteration. so 0 is the right value
    pilot_pitch_cd = 0.0f;
    // store final loiter outputs for mixing with pilot input
    controller_final_pitch_cd = pitch_cd;
}

#endif
