/**
 * @file mode.cpp
 * @brief ArduSub Mode base class implementation and mode management framework
 * 
 * @details This file implements the Mode abstract base class which serves as the
 *          foundation for all ArduSub flight modes. The Mode class provides:
 *          - Common subsystem accessor pattern (attitude_control, position_control, motors, etc.)
 *          - Base implementations of init() and run() lifecycle methods
 *          - Pilot input transformation utilities
 *          - Integration with the Sub singleton for vehicle-wide state access
 *          
 *          Mode Framework Architecture:
 *          - Each flight mode (Manual, Stabilize, AltHold, etc.) inherits from Mode base class
 *          - Sub vehicle class maintains pointers to mode instances (mode_manual, mode_stabilize, etc.)
 *          - Mode transitions handled by Sub::set_mode() with pre-flight checks and initialization
 *          - Active mode's run() method called at main loop rate via Sub::update_flight_mode()
 *          
 *          Underwater Vehicle Considerations:
 *          - Depth control replaces altitude control from multicopter heritage
 *          - 6DOF control supports ROV operation with lateral and forward translation
 *          - Mode requirements adapted for underwater operation (depth sensor vs barometer)
 *          
 * @note This implementation pattern is consistent across ArduPilot vehicle types
 *       (ArduCopter, ArduPlane, ArduRover) with vehicle-specific mode implementations
 * 
 * @see Mode.h for Mode base class definition
 * @see mode_*.cpp for individual mode implementations
 * 
 * Source: ArduSub/mode.cpp:1-300
 */

#include "Sub.h"

/**
 * @brief Constructor for Mode base class object
 * 
 * @details Initializes Mode base class with references to Sub vehicle subsystems.
 *          This constructor establishes the subsystem accessor pattern that allows
 *          all derived mode classes to access vehicle state and control interfaces.
 *          
 *          Subsystems accessed via reference:
 *          - g, g2: Parameter groups for configuration access
 *          - inertial_nav: Position and velocity estimation
 *          - ahrs: Attitude and heading reference
 *          - motors: Motor output control
 *          - channel_*: RC input channels for pilot control
 *          - position_control: Position controller instance
 *          - attitude_control: Attitude controller instance
 *          - G_Dt: Main loop delta time
 *          
 * @note Constructor called once per mode instance during Sub vehicle initialization
 * @note All references point to Sub singleton subsystems ensuring single source of truth
 */
Mode::Mode(void) :
    g(sub.g),
    g2(sub.g2),
    inertial_nav(sub.inertial_nav),
    ahrs(sub.ahrs),
    motors(sub.motors),
    channel_roll(sub.channel_roll),
    channel_pitch(sub.channel_pitch),
    channel_throttle(sub.channel_throttle),
    channel_yaw(sub.channel_yaw),
    channel_forward(sub.channel_forward),
    channel_lateral(sub.channel_lateral),
    position_control(&sub.pos_control),
    attitude_control(&sub.attitude_control),
    G_Dt(sub.G_Dt)
{ };

/**
 * @brief Return the static mode controller object corresponding to supplied mode number
 * 
 * @details Maps Mode::Number enumeration values to concrete mode instance pointers.
 *          Each mode instance is a static member of the Sub vehicle class, constructed
 *          once during initialization. This function provides the lookup mechanism for
 *          mode transitions and mode queries.
 *          
 *          Supported ArduSub modes:
 *          - MANUAL: Direct pilot control without stabilization
 *          - STABILIZE: Attitude stabilization with pilot control
 *          - ACRO: Acrobatic mode with rate control
 *          - ALT_HOLD: Depth hold with attitude stabilization
 *          - SURFTRAK: Surface tracking mode for shallow operation
 *          - POSHOLD: Position and depth hold
 *          - AUTO: Autonomous mission execution
 *          - GUIDED: External control (MAVLink commanded navigation)
 *          - CIRCLE: Circular navigation pattern
 *          - SURFACE: Automatic surface ascent
 *          - MOTOR_DETECT: Motor direction and function testing
 * 
 * @param[in] mode Mode number from Mode::Number enumeration
 * 
 * @return Mode* Pointer to corresponding mode controller instance, nullptr if invalid mode
 * 
 * @note Returns nullptr for invalid mode numbers - caller must check before use
 * @note Mode instances are static members of Sub class with lifetime of program execution
 * 
 * @see Sub::set_mode() for mode transition logic using this lookup
 */
Mode *Sub::mode_from_mode_num(const Mode::Number mode)
{
    Mode *ret = nullptr;

    switch (mode) {
    case Mode::Number::MANUAL:
        ret = &mode_manual;
        break;
    case Mode::Number::STABILIZE:
        ret = &mode_stabilize;
        break;
    case Mode::Number::ACRO:
        ret = &mode_acro;
        break;
    case Mode::Number::ALT_HOLD:
        ret = &mode_althold;
        break;
    case Mode::Number::SURFTRAK:
        ret = &mode_surftrak;
        break;
    case Mode::Number::POSHOLD:
        ret = &mode_poshold;
        break;
    case Mode::Number::AUTO:
        ret = &mode_auto;
        break;
    case Mode::Number::GUIDED:
        ret = &mode_guided;
        break;
    case Mode::Number::CIRCLE:
        ret = &mode_circle;
        break;
    case Mode::Number::SURFACE:
        ret = &mode_surface;
        break;
    case Mode::Number::MOTOR_DETECT:
        ret = &mode_motordetect;
        break;
    default:
        break;
    }

    return ret;
}

/**
 * @brief Change flight mode and perform necessary initialization and validation
 * 
 * @details Handles mode transition with comprehensive pre-flight checks and initialization:
 *          
 *          Mode Change Sequence:
 *          1. Check if already in requested mode (early return)
 *          2. Validate mode number and retrieve mode instance pointer
 *          3. Check position requirements if mode requires GPS
 *          4. Check depth/altitude sensor requirements if mode needs altitude control
 *          5. Call new mode's init() method
 *          6. Clean up previous mode via exit_mode()
 *          7. Update active mode pointers and control_mode
 *          8. Log mode change and notify GCS/LED indicators
 *          
 *          Failure Conditions:
 *          - Invalid mode number (nullptr from mode_from_mode_num)
 *          - Position sensor failure when mode requires_GPS()
 *          - Depth sensor failure when mode requires_altitude()
 *          - Mode init() returns false (mode-specific initialization failure)
 *          
 *          Underwater Vehicle Considerations:
 *          - Position check validates EKF position estimate quality
 *          - Altitude check validates barometer/depth sensor for depth hold modes
 *          - Some modes (MANUAL, STABILIZE) can always be entered as fallback
 * 
 * @param[in] mode Mode::Number enumeration value for desired mode
 * @param[in] reason ModeReason enumeration documenting why mode change requested
 *                   (pilot command, failsafe, mission, etc.)
 * 
 * @return true if mode was successfully changed and initialized
 * @return false if mode change failed due to validation or initialization failure
 * 
 * @note Always check return value - some mode changes can fail
 * @note Mode change failure logged to dataflash and sent to GCS as warning
 * @note Previous mode stored in prev_control_mode before transition
 * 
 * @warning Caller must handle false return appropriately - vehicle may remain in previous mode
 * 
 * @see Mode::init() for mode-specific initialization requirements
 * @see Mode::requires_GPS() for position-dependent mode requirements
 * @see Mode::requires_altitude() for depth-sensor-dependent mode requirements
 */
bool Sub::set_mode(Mode::Number mode, ModeReason reason)
{

    // return immediately if we are already in the desired mode
    if (mode == control_mode) {
        control_mode_reason = reason;
        return true;
    }

    Mode *new_flightmode = mode_from_mode_num((Mode::Number)mode);
    if (new_flightmode == nullptr) {
        notify_no_such_mode((uint8_t)mode);
        return false;
    }

    if (new_flightmode->requires_GPS() &&
        !sub.position_ok()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode change failed: %s requires position", new_flightmode->name());
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode));
        return false;
    }

    // check for valid altitude if old mode did not require it but new one does
    // we only want to stop changing modes if it could make things worse
    if (!flightmode->requires_altitude() &&
        new_flightmode->requires_altitude() &&
        !sub.control_check_barometer()) { // maybe use ekf_alt_ok() instead?
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode change failed: %s need alt estimate", new_flightmode->name());
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode));
        return false;
    }

    if (!new_flightmode->init(false)) {
        gcs().send_text(MAV_SEVERITY_WARNING,"Flight mode change failed %s", new_flightmode->name());
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode));
        return false;
    }

    // perform any cleanup required by previous flight mode
    exit_mode(flightmode, new_flightmode);

    // store previous flight mode
    prev_control_mode = control_mode;

    // update flight mode
    flightmode = new_flightmode;
    control_mode = mode;
    control_mode_reason = reason;
#if HAL_LOGGING_ENABLED
    logger.Write_Mode((uint8_t)control_mode, reason);
#endif
    gcs().send_message(MSG_HEARTBEAT);

    // update notify object
    notify_flight_mode();

    // return success
    return true;
}

/**
 * @brief High-level cleanup when exiting a flight mode
 * 
 * @details Performs mode-specific cleanup operations when transitioning away from certain modes.
 *          This function is called by set_mode() before initializing the new mode to ensure
 *          clean state transitions.
 *          
 *          Cleanup Operations by Mode:
 *          - AUTO mode: Stop mission execution if running, reset camera mount to default
 *          - All modes: Reset maximum throttle to 100% (removes any limits from previous mode)
 *          
 *          Underwater Vehicle Considerations:
 *          - Mission stop prevents continued waypoint execution after leaving AUTO
 *          - Camera mount reset ensures gimbal returns to pilot control
 *          - Throttle limit reset ensures full motor authority in new mode
 * 
 * @param[in] old_control_mode Mode::Number of mode being exited
 * @param[in] new_control_mode Mode::Number of mode being entered
 * 
 * @note Called automatically by set_mode() - should not be called directly
 * @note Cleanup must not depend on new mode initialization having occurred yet
 * 
 * @see Sub::set_mode() for mode transition sequence
 */
void Sub::exit_mode(Mode::Number old_control_mode, Mode::Number new_control_mode)
{
    // stop mission when we leave auto mode
    if (old_control_mode == Mode::Number::AUTO) {
        if (mission.state() == AP_Mission::MISSION_RUNNING) {
            mission.stop();
        }
#if HAL_MOUNT_ENABLED
        camera_mount.set_mode_to_default();
#endif  // HAL_MOUNT_ENABLED
    }
    motors.set_max_throttle(1.0f);
}

/**
 * @brief Set mode using uint8_t mode number (MAVLink compatibility wrapper)
 * 
 * @details Wrapper function that converts uint8_t mode number to Mode::Number enum type.
 *          This overload exists for compatibility with MAVLink protocol which transmits
 *          mode numbers as uint8_t values in COMMAND_LONG messages.
 *          
 *          Static assertion ensures safe casting between uint8_t and Mode::Number enum.
 * 
 * @param[in] new_mode Mode number as uint8_t (typically from MAVLink command)
 * @param[in] reason ModeReason documenting source of mode change request
 * 
 * @return true if mode change succeeded
 * @return false if mode change failed (see set_mode(Mode::Number, ModeReason) for details)
 * 
 * @note This is a thin wrapper - see primary set_mode() for complete documentation
 * 
 * @see Sub::set_mode(Mode::Number, ModeReason) for detailed mode change logic
 */
{
    static_assert(sizeof(Mode::Number) == sizeof(new_mode), "The new mode can't be mapped to the vehicles mode number");
    return sub.set_mode(static_cast<Mode::Number>(new_mode), reason);
}

/**
 * @brief Execute active flight mode's control logic at main loop rate
 * 
 * @details Calls the currently active mode's run() method which implements mode-specific
 *          control algorithms and pilot input processing. This function is the core of
 *          the mode execution framework.
 *          
 *          Execution Context:
 *          - Called from Sub::fast_loop() at main loop rate (typically 100Hz for ArduSub)
 *          - Each mode's run() implements its control loop iteration
 *          - Mode has access to current vehicle state via subsystem accessors
 *          - Mode outputs commands to attitude/position controllers and motors
 *          
 *          Mode Responsibilities in run():
 *          - Read and process pilot inputs from RC channels
 *          - Read sensor data (position, attitude, depth)
 *          - Execute mode-specific control algorithms
 *          - Generate attitude/position/velocity targets
 *          - Handle mode-specific failsafe conditions
 * 
 * @note Called at 100Hz or higher - mode implementations must be efficient
 * @note flightmode pointer always valid after successful set_mode()
 * 
 * @warning Mode run() methods must complete within loop period to maintain timing
 * 
 * @see Mode::run() for mode-specific implementation requirements
 * @see Sub::fast_loop() for main loop timing
 */
void Sub::update_flight_mode()
{
    flightmode->run();
}

/**
 * @brief Cleanup when exiting flight mode (pointer-based variant)
 * 
 * @details Performs cleanup operations when transitioning between modes using mode
 *          instance pointers. This variant is called directly by set_mode() during
 *          mode transitions.
 *          
 *          Cleanup Operations:
 *          - Reset camera mount to default mode (releases any mode-specific control)
 *          - Reset maximum throttle to 100% (removes mode-imposed limits)
 *          
 *          This variant is simpler than the Mode::Number overload as it doesn't need
 *          to check specific modes - it performs universal cleanup operations.
 * 
 * @param[in,out] old_flightmode Reference to pointer of mode being exited
 * @param[in,out] new_flightmode Reference to pointer of mode being entered
 * 
 * @note Pointers passed by reference but not modified by this function
 * @note Camera mount reset conditional on HAL_MOUNT_ENABLED feature flag
 * 
 * @see Sub::set_mode() for mode transition sequence
 */
void Sub::exit_mode(Mode *&old_flightmode, Mode *&new_flightmode){
#if HAL_MOUNT_ENABLED
        camera_mount.set_mode_to_default();
#endif  // HAL_MOUNT_ENABLED
    motors.set_max_throttle(1.0f);
}

/**
 * @brief Update notification system with current flight mode information
 * 
 * @details Updates the AP_Notify system flags and mode string to reflect the current
 *          flight mode. This information is used by:
 *          - LED indicators (OreoLED, NeoPixel, etc.) to display mode via colors/patterns
 *          - Buzzers for audio feedback on mode changes
 *          - Display devices showing mode name
 *          
 *          Updated Notify Information:
 *          - autopilot_mode flag: true if mode is autonomous (AUTO, GUIDED, CIRCLE)
 *          - flight_mode number: numeric mode identifier for LED pattern selection
 *          - flight_mode_str: 4-character mode name string for display
 *          
 *          Typical Usage:
 *          - Called by set_mode() after successful mode transition
 *          - Updates occur immediately so pilot sees mode change confirmation
 * 
 * @note Mode name limited to 4 characters for display compatibility (name4())
 * @note Notify system handles actual LED/buzzer/display hardware abstraction
 * 
 * @see AP_Notify for notification system architecture
 * @see Mode::is_autopilot() for autonomous mode detection
 * @see Mode::name4() for 4-character mode name generation
 */
void Sub::notify_flight_mode()
{
    AP_Notify::flags.autopilot_mode = flightmode->is_autopilot();
    AP_Notify::flags.flight_mode = (uint8_t)control_mode;
    notify.set_flight_mode_str(flightmode->name4());
}

/**
 * @brief Transform pilot RC inputs into desired body-frame angular rates
 * 
 * @details Converts pilot stick inputs to desired roll, pitch, and yaw rates with:
 *          - Expo curves for smooth control feel
 *          - Circular input limiting to prevent oversaturation
 *          - Optional ACRO trainer mode for self-leveling assistance
 *          - Rate limiting to prevent uncommanded flips through inverted
 *          
 *          Input Processing Pipeline:
 *          1. Apply circular limit to roll/pitch inputs (prevent diagonal oversaturation)
 *          2. Apply expo curves if configured (g.acro_expo) for smoother control near center
 *          3. Scale by ACRO gain parameters (g.acro_rp_p for roll/pitch, g.acro_yaw_p for yaw)
 *          4. If ACRO trainer enabled, add self-leveling rate correction in earth frame
 *          5. Convert earth-frame corrections to body-frame angular rates
 *          6. Apply rate limiting to prevent dangerous rate changes through inverted attitudes
 *          
 *          ACRO Trainer Modes:
 *          - DISABLED: No automatic leveling, pure rate control
 *          - LEVELING: Self-levels when sticks centered, proportional to current angle
 *          - LIMITED: LEVELING + hard angle limits to prevent excessive bank/pitch
 *          
 *          Expo Curve Math:
 *          - output = (expo * input³) + ((1 - expo) * input)
 *          - expo=0: Linear response
 *          - expo=1: Cubic response (very gentle near center, aggressive at extremes)
 *          - Range: 0.0 to 1.0
 *          
 *          Underwater Vehicle Considerations:
 *          - ROV operation benefits from expo for precise maneuvering
 *          - ACRO trainer helps maintain level attitude for observation tasks
 *          - Rate limits prevent loss of camera orientation during aggressive maneuvers
 * 
 * @param[in]  roll_in   Pilot roll input in normalized units (-4500 to 4500)
 * @param[in]  pitch_in  Pilot pitch input in normalized units (-4500 to 4500)
 * @param[in]  yaw_in    Pilot yaw input in normalized units (-4500 to 4500)
 * @param[out] roll_out  Desired roll rate in centi-degrees/second
 * @param[out] pitch_out Desired pitch rate in centi-degrees/second
 * @param[out] yaw_out   Desired yaw rate in centi-degrees/second
 * 
 * @note Called by ACRO and other rate-control modes at main loop rate
 * @note Input values typically from RC channels scaled to ±4500 range
 * @note Output rates in centi-degrees/second (deg/s * 100) for precision
 * 
 * @warning ACRO trainer angle limiting prevents inverted flight beyond configured limits
 * 
 * @see ACRO_TRAINER_DISABLED, ACRO_TRAINER_LEVELING, ACRO_TRAINER_LIMITED constants
 * @see g.acro_expo, g.acro_rp_p, g.acro_yaw_p parameters for tuning
 * @see AC_AttitudeControl::euler_rate_to_ang_vel() for frame transformation
 */
void Mode::get_pilot_desired_angle_rates(int16_t roll_in, int16_t pitch_in, int16_t yaw_in, float &roll_out, float &pitch_out, float &yaw_out)
{
    float rate_limit;
    Vector3f rate_ef_level, rate_bf_level, rate_bf_request;

    // apply circular limit to pitch and roll inputs
    float total_in = norm(pitch_in, roll_in);

    if (total_in > ROLL_PITCH_INPUT_MAX) {
        float ratio = (float)ROLL_PITCH_INPUT_MAX / total_in;
        roll_in *= ratio;
        pitch_in *= ratio;
    }

    // calculate roll, pitch rate requests
    if (g.acro_expo <= 0) {
        rate_bf_request.x = roll_in * g.acro_rp_p;
        rate_bf_request.y = pitch_in * g.acro_rp_p;
    } else {
        // expo variables
        float rp_in, rp_in3, rp_out;

        // range check expo
        if (g.acro_expo > 1.0f) {
            g.acro_expo.set(1.0f);
        }

        // roll expo
        rp_in = float(roll_in)/ROLL_PITCH_INPUT_MAX;
        rp_in3 = rp_in*rp_in*rp_in;
        rp_out = (g.acro_expo * rp_in3) + ((1 - g.acro_expo) * rp_in);
        rate_bf_request.x = ROLL_PITCH_INPUT_MAX * rp_out * g.acro_rp_p;

        // pitch expo
        rp_in = float(pitch_in)/ROLL_PITCH_INPUT_MAX;
        rp_in3 = rp_in*rp_in*rp_in;
        rp_out = (g.acro_expo * rp_in3) + ((1 - g.acro_expo) * rp_in);
        rate_bf_request.y = ROLL_PITCH_INPUT_MAX * rp_out * g.acro_rp_p;
    }

    // calculate yaw rate request
    rate_bf_request.z = yaw_in * g.acro_yaw_p;

    // calculate earth frame rate corrections to pull the vehicle back to level while in ACRO mode

    if (g.acro_trainer != ACRO_TRAINER_DISABLED) {
        // Calculate trainer mode earth frame rate command for roll
        int32_t roll_angle = wrap_180_cd(ahrs.roll_sensor);
        rate_ef_level.x = -constrain_int32(roll_angle, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE) * g.acro_balance_roll;

        // Calculate trainer mode earth frame rate command for pitch
        int32_t pitch_angle = wrap_180_cd(ahrs.pitch_sensor);
        rate_ef_level.y = -constrain_int32(pitch_angle, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE) * g.acro_balance_pitch;

        // Calculate trainer mode earth frame rate command for yaw
        rate_ef_level.z = 0;

        // Calculate angle limiting earth frame rate commands
        if (g.acro_trainer == ACRO_TRAINER_LIMITED) {
            if (roll_angle > sub.aparm.angle_max) {
                rate_ef_level.x -=  g.acro_balance_roll*(roll_angle-sub.aparm.angle_max);
            } else if (roll_angle < -sub.aparm.angle_max) {
                rate_ef_level.x -=  g.acro_balance_roll*(roll_angle+sub.aparm.angle_max);
            }

            if (pitch_angle > sub.aparm.angle_max) {
                rate_ef_level.y -=  g.acro_balance_pitch*(pitch_angle-sub.aparm.angle_max);
            } else if (pitch_angle < -sub.aparm.angle_max) {
                rate_ef_level.y -=  g.acro_balance_pitch*(pitch_angle+sub.aparm.angle_max);
            }
        }

        // convert earth-frame level rates to body-frame level rates
        attitude_control->euler_rate_to_ang_vel(attitude_control->get_attitude_target_quat(), rate_ef_level, rate_bf_level);

        // combine earth frame rate corrections with rate requests
        if (g.acro_trainer == ACRO_TRAINER_LIMITED) {
            rate_bf_request.x += rate_bf_level.x;
            rate_bf_request.y += rate_bf_level.y;
            rate_bf_request.z += rate_bf_level.z;
        } else {
            float acro_level_mix = constrain_float(1-MAX(MAX(abs(roll_in), abs(pitch_in)), abs(yaw_in))/4500.0, 0, 1)*ahrs.cos_pitch();

            // Scale leveling rates by stick input
            rate_bf_level = rate_bf_level*acro_level_mix;

            // Calculate rate limit to prevent change of rate through inverted
            rate_limit = fabsf(fabsf(rate_bf_request.x)-fabsf(rate_bf_level.x));
            rate_bf_request.x += rate_bf_level.x;
            rate_bf_request.x = constrain_float(rate_bf_request.x, -rate_limit, rate_limit);

            // Calculate rate limit to prevent change of rate through inverted
            rate_limit = fabsf(fabsf(rate_bf_request.y)-fabsf(rate_bf_level.y));
            rate_bf_request.y += rate_bf_level.y;
            rate_bf_request.y = constrain_float(rate_bf_request.y, -rate_limit, rate_limit);

            // Calculate rate limit to prevent change of rate through inverted
            rate_limit = fabsf(fabsf(rate_bf_request.z)-fabsf(rate_bf_level.z));
            rate_bf_request.z += rate_bf_level.z;
            rate_bf_request.z = constrain_float(rate_bf_request.z, -rate_limit, rate_limit);
        }
    }

    // hand back rate request
    roll_out = rate_bf_request.x;
    pitch_out = rate_bf_request.y;
    yaw_out = rate_bf_request.z;
}

/**
 * @brief Mode base class wrapper to request mode change via Sub vehicle
 * 
 * @details Allows mode instances to request mode changes through the Sub vehicle's
 *          mode management system. This provides a clean interface for modes to
 *          trigger mode transitions (e.g., failsafe-initiated mode changes).
 *          
 *          This wrapper maintains the single-point-of-control pattern where all
 *          mode changes go through Sub::set_mode() for proper validation and logging.
 * 
 * @param[in] mode Mode::Number to transition to
 * @param[in] reason ModeReason documenting why mode change requested
 * 
 * @return true if mode change succeeded
 * @return false if mode change failed validation or initialization
 * 
 * @note This is a thin wrapper delegating to Sub::set_mode()
 * 
 * @see Sub::set_mode() for complete mode transition logic
 */
{
    return sub.set_mode(mode, reason);
}

/**
 * @brief Access Ground Control Station communication interface from mode
 * 
 * @details Provides mode classes access to the GCS_Sub singleton for sending
 *          telemetry messages, status text, and handling MAVLink communications.
 *          
 *          Common usage in modes:
 *          - gcs().send_text() for status/warning messages to ground station
 *          - gcs().send_message() for MAVLink protocol messages
 *          - Error reporting during mode operation
 *          
 *          Accessor Pattern:
 *          This function provides clean access to the Sub vehicle's GCS instance
 *          without modes needing direct coupling to Sub singleton.
 * 
 * @return GCS_Sub& Reference to the Ground Control Station interface singleton
 * 
 * @note GCS instance lifetime managed by Sub vehicle class
 * @note All mode instances share single GCS_Sub instance
 * 
 * @see GCS_Sub for MAVLink communication interface
 * @see GCS::send_text() for sending text messages to ground station
 */
{
    return sub.gcs();
}
