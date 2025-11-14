/**
 * @file mode.cpp
 * @brief Blimp flight mode base class implementation and mode management
 * 
 * @details This file implements the Mode base class for lighter-than-air (blimp) vehicles
 *          and provides core mode management functionality including:
 *          - Mode base class construction binding Mode instances to global blimp singleton
 *          - mode_from_mode_num() conversion from mode number to Mode* pointer
 *          - set_mode() transitions with comprehensive safety checks, logging, and notify updates
 *          - update_flight_mode() dispatch to current mode's run() method
 *          - Pilot input extraction and scaling for multi-axis control
 *          
 *          Mode state machine transitions are guarded by checks including GPS availability,
 *          altitude estimation validity, and arming state. Mode-specific control logic
 *          is implemented in separate files (mode_land.cpp, mode_manual.cpp, etc.).
 *          
 *          Coordinate system: Blimp uses NED (North-East-Down) frame for position control
 *          with body frame inputs from pilot (front/right/up/yaw channels).
 * 
 * @note This file is safety-critical - mode transitions directly affect vehicle behavior
 * @warning Modifications to mode transition logic must be validated in SITL before flight
 * 
 * Source: Blimp/mode.cpp
 */

#include "Blimp.h"

/*
 * High level calls to set and update flight modes logic for individual
 * flight modes is in control_acro.cpp, control_stabilize.cpp, etc
 */

#include <AP_Vehicle/AP_MultiCopter.h>

/**
 * @brief Constructor for Mode base class object
 * 
 * @details Constructs a Mode object by binding references to the global blimp singleton's
 *          subsystems. This establishes the connection between each mode instance and the
 *          vehicle's core components including:
 *          - g, g2: Parameter groups for configuration access
 *          - inertial_nav: Inertial navigation system for position/velocity
 *          - ahrs: Attitude and Heading Reference System
 *          - motors: Fin motor control for thrust vectoring
 *          - loiter: Loiter position controller
 *          - channel_right, channel_front, channel_up, channel_yaw: RC input channels
 *          - G_Dt: Main loop delta time in seconds
 *          
 *          All mode subclasses (ModeLand, ModeManual, etc.) inherit these references,
 *          allowing mode-specific code to access vehicle state and control outputs.
 * 
 * @note This constructor is called once per mode at startup during static initialization
 * @note All modes share the same blimp singleton - changes affect global vehicle state
 * 
 * Source: Blimp/mode.cpp:13-25
 */
Mode::Mode(void) :
    g(blimp.g),
    g2(blimp.g2),
    inertial_nav(blimp.inertial_nav),
    ahrs(blimp.ahrs),
    motors(blimp.motors),
    loiter(blimp.loiter),
    channel_right(blimp.channel_right),
    channel_front(blimp.channel_front),
    channel_up(blimp.channel_up),
    channel_yaw(blimp.channel_yaw),
    G_Dt(blimp.G_Dt)
{ };

/**
 * @brief Convert mode number to static Mode controller object pointer
 * 
 * @details Returns a pointer to the static Mode object corresponding to the supplied
 *          mode number. Each flight mode is implemented as a static instance in the
 *          Blimp class (mode_land, mode_manual, mode_velocity, mode_loiter, mode_rtl).
 *          This function provides the mapping from Mode::Number enum to the actual
 *          Mode object pointer.
 *          
 *          Supported modes:
 *          - LAND: Autonomous landing mode
 *          - MANUAL: Direct pilot control without stabilization
 *          - VELOCITY: Velocity-controlled flight
 *          - LOITER: Position hold mode
 *          - RTL: Return to launch mode
 * 
 * @param[in] mode Mode number from Mode::Number enum
 * 
 * @return Mode* Pointer to static Mode object, or nullptr if mode number is invalid
 * 
 * @note Returns nullptr for invalid mode numbers - caller must check for null
 * @see set_mode() for mode transition logic that uses this function
 * 
 * Source: Blimp/mode.cpp:28-53
 */
Mode *Blimp::mode_from_mode_num(const Mode::Number mode)
{
    Mode *ret = nullptr;

    switch (mode) {
    case Mode::Number::LAND:
        ret = &mode_land;
        break;
    case Mode::Number::MANUAL:
        ret = &mode_manual;
        break;
    case Mode::Number::VELOCITY:
        ret = &mode_velocity;
        break;
    case Mode::Number::LOITER:
        ret = &mode_loiter;
        break;
    case Mode::Number::RTL:
        ret = &mode_rtl;
        break;
    default:
        break;
    }

    return ret;
}


/**
 * @brief Change flight mode with comprehensive safety checks and initialization
 * 
 * @details Performs a guarded transition to a new flight mode with the following sequence:
 *          
 *          1. Early return if already in requested mode (updates reason only)
 *          2. Convert mode number to Mode* pointer via mode_from_mode_num()
 *          3. Safety validation checks (unless disarmed):
 *             - GPS position availability for modes requiring GPS
 *             - Altitude estimation validity when transitioning from manual to auto throttle
 *          4. Mode-specific initialization via new_flightmode->init()
 *          5. Exit previous mode via exit_mode()
 *          6. Update global state: flightmode, control_mode, control_mode_reason
 *          7. Log mode change and send notifications (GCS heartbeat, notify LEDs)
 *          
 *          Mode Transition Rules:
 *          - Disarmed: All mode changes allowed (ignore_checks = true)
 *          - Armed: Safety checks enforced based on mode requirements
 *          - Invalid modes: Rejected with notify_no_such_mode()
 *          - Failed init: Rejected with GCS warning and error log
 *          
 *          The ignore_checks flag allows mode changes when disarmed, relying on
 *          pre-arm checks to validate mode safety before flight.
 * 
 * @param[in] mode Mode::Number enum specifying the desired flight mode
 * @param[in] reason ModeReason enum indicating why mode change was requested
 *                   (GCS command, RC switch, failsafe, etc.)
 * 
 * @return true if mode was successfully changed, false if change rejected
 * 
 * @note Some modes can always be set successfully, but return state of modes with
 *       requirements (GPS, altitude) should be checked by caller
 * @warning Mode transitions directly affect vehicle control behavior and stability
 * @warning Failed mode transitions during flight could compromise vehicle safety
 * 
 * @see mode_from_mode_num() for mode number to Mode* conversion
 * @see Mode::init() for mode-specific initialization requirements
 * 
 * Source: Blimp/mode.cpp:60-119
 */
bool Blimp::set_mode(Mode::Number mode, ModeReason reason)
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

    bool ignore_checks = !motors->armed();   // allow switching to any mode if disarmed.  We rely on the arming check to perform

    if (!ignore_checks &&
        new_flightmode->requires_GPS() &&
        !blimp.position_ok()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode change failed: %s requires position", new_flightmode->name());
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode));
        return false;
    }

    // check for valid altitude if old mode did not require it but new one does
    // we only want to stop changing modes if it could make things worse
    if (!ignore_checks &&
        !blimp.ekf_alt_ok() &&
        flightmode->has_manual_throttle() &&
        !new_flightmode->has_manual_throttle()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode change failed: %s need alt estimate", new_flightmode->name());
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode));
        return false;
    }

    if (!new_flightmode->init(ignore_checks)) {
        gcs().send_text(MAV_SEVERITY_WARNING,"Flight mode change failed %s", new_flightmode->name());
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode));
        return false;
    }

    // perform any cleanup required by previous flight mode
    exit_mode(flightmode, new_flightmode);

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
 * @brief Overloaded set_mode accepting uint8_t mode number
 * 
 * @details Wrapper function that casts uint8_t mode number to Mode::Number enum
 *          and calls the main set_mode() implementation. Provides compatibility
 *          with external interfaces (MAVLink, parameter system) that use numeric
 *          mode identifiers.
 *          
 *          Additional safety check: If DISALLOW_GCS_MODE_CHANGE_DURING_RC_FAILSAFE
 *          is defined, rejects GCS-commanded mode changes during radio failsafe to
 *          prevent unsafe mode transitions when pilot has lost RC control.
 * 
 * @param[in] new_mode Numeric mode identifier (0-255)
 * @param[in] reason ModeReason enum indicating source of mode change request
 * 
 * @return true if mode was successfully changed, false if rejected
 * 
 * @note static_assert validates Mode::Number and uint8_t are same size
 * @warning GCS mode changes blocked during radio failsafe if compile flag set
 * 
 * @see set_mode(Mode::Number, ModeReason) for main implementation
 * 
 * Source: Blimp/mode.cpp:121-131
 */
bool Blimp::set_mode(const uint8_t new_mode, const ModeReason reason)
{
    static_assert(sizeof(Mode::Number) == sizeof(new_mode), "The new mode can't be mapped to the vehicles mode number");
#ifdef DISALLOW_GCS_MODE_CHANGE_DURING_RC_FAILSAFE
    if (reason == ModeReason::GCS_COMMAND && blimp.failsafe.radio) {
        // don't allow mode changes while in radio failsafe
        return false;
    }
#endif
    return blimp.set_mode(static_cast<Mode::Number>(new_mode), reason);
}

/**
 * @brief Dispatch to current flight mode's run() method
 * 
 * @details Executes the active flight mode's control logic by calling its run() method.
 *          This is the main mode dispatch point called from the scheduler's fast loop.
 *          Each mode implements its own run() method with mode-specific control algorithms
 *          (attitude control, position control, velocity control, etc.).
 *          
 *          The flightmode pointer is updated by set_mode() when mode transitions occur.
 *          This function simply delegates execution to whichever Mode object is currently
 *          active (mode_manual, mode_loiter, mode_land, etc.).
 * 
 * @note Called at 100Hz or higher from main scheduler loop
 * @note This is a safety-critical function - executes every control loop iteration
 * @warning Modifications affecting timing or execution frequency could compromise stability
 * 
 * @see Mode::run() for mode-specific implementations
 * @see set_mode() for mode transition logic
 * 
 * Source: Blimp/mode.cpp:135-138
 */
void Blimp::update_flight_mode()
{
    flightmode->run();
}

/**
 * @brief High-level cleanup when exiting a flight mode
 * 
 * @details Organizes any cleanup required when transitioning from one flight mode to another.
 *          Currently implemented as an empty function - mode-specific cleanup is handled
 *          within each mode's exit logic if needed. This function provides a hook for
 *          future global cleanup operations during mode transitions.
 * 
 * @param[in,out] old_flightmode Pointer to mode being exited
 * @param[in,out] new_flightmode Pointer to mode being entered
 * 
 * @note Currently no global cleanup needed for blimp mode transitions
 * @see set_mode() which calls this function during mode transitions
 * 
 * Source: Blimp/mode.cpp:141-142
 */
void Blimp::exit_mode(Mode *&old_flightmode,
                      Mode *&new_flightmode) {}

/**
 * @brief Update notification system with current flight mode information
 * 
 * @details Updates the AP_Notify system (LEDs, buzzers, displays) with current flight
 *          mode state. Sets three notification flags:
 *          - autopilot_mode: true if mode is autonomous (not manual)
 *          - flight_mode: numeric mode identifier for LED patterns
 *          - flight mode string: 4-character mode name for displays
 *          
 *          This function is called after successful mode transitions to update
 *          external indicators. Primarily used for OreoLED notify devices but
 *          also affects other notification peripherals (NeoPixel, buzzer, etc.).
 * 
 * @note Called from set_mode() after successful mode transition
 * @see set_mode() for mode transition logic
 * @see Mode::is_autopilot() for autonomous mode determination
 * @see Mode::name4() for 4-character mode name
 * 
 * Source: Blimp/mode.cpp:145-150
 */
void Blimp::notify_flight_mode()
{
    AP_Notify::flags.autopilot_mode = flightmode->is_autopilot();
    AP_Notify::flags.flight_mode = (uint8_t)control_mode;
    notify.set_flight_mode_str(flightmode->name4());
}

/**
 * @brief Update navigation and autopilot decision logic
 * 
 * @details Executes high-level autopilot decision-making by calling run_autopilot().
 *          This function is called by autonomous modes to update navigation state,
 *          process waypoint progression, and make mode-level control decisions.
 *          
 *          Mode-specific implementations of run_autopilot() handle:
 *          - Waypoint navigation and mission progression
 *          - Loiter position updates
 *          - RTL path planning
 *          - Land descent control
 * 
 * @note Called by autonomous modes (not manual modes)
 * @see Mode::run_autopilot() for mode-specific navigation implementations
 * 
 * Source: Blimp/mode.cpp:152-156
 */
void Mode::update_navigation()
{
    // run autopilot to make high level decisions about control modes
    run_autopilot();
}

/**
 * @brief Extract and scale pilot RC input for multi-axis control
 * 
 * @details Reads pilot stick inputs from RC channels and scales them to normalized
 *          values in range [-1.0, +1.0] for use in control algorithms. Handles four
 *          control axes for lighter-than-air vehicle control:
 *          
 *          Control Mapping (body frame):
 *          - pilot.x: Front/back (channel_front) - forward positive
 *          - pilot.y: Right/left (channel_right) - right positive  
 *          - pilot.z: Up/down (channel_up) - up positive (note: channel is negated)
 *          - yaw: Rotation rate (channel_yaw) - clockwise positive
 *          
 *          Failsafe Handling: If radio failsafe is active or no RC input has ever been
 *          seen, all outputs are zeroed to ensure safe failsafe behavior.
 *          
 *          Scaling: Raw RC input (typically -4500 to +4500) divided by RC_SCALE to
 *          produce normalized output (-1.0 to +1.0).
 * 
 * @param[out] pilot Vector3f containing scaled X/Y/Z body frame inputs (-1.0 to +1.0)
 * @param[out] yaw Scaled yaw rate input (-1.0 to +1.0)
 * 
 * @note Coordinate system: Body frame with X forward, Y right, Z up
 * @note Channel_up is negated because RC throttle convention is higher stick = up
 * @warning Returns zero inputs during radio failsafe - modes must handle gracefully
 * 
 * @todo Verify channel_up negation is correct for blimp up/down control
 * 
 * @see RC_Channel::get_control_in() for raw RC input values
 * 
 * Source: Blimp/mode.cpp:159-175
 */
void Mode::get_pilot_input(Vector3f &pilot, float &yaw)
{
    // throttle failsafe check
    if (blimp.failsafe.radio || !rc().has_ever_seen_rc_input()) {
        pilot.y = 0;
        pilot.x = 0;
        pilot.z = 0;
        yaw = 0;
        return;
    }
    // fetch pilot inputs
    pilot.y = channel_right->get_control_in() / float(RC_SCALE);
    pilot.x = channel_front->get_control_in() / float(RC_SCALE);
    //TODO: need to make this channel_up instead, and then have it .negative. before being sent to pilot.z -> this is "throttle" channel, so higher = up.
    pilot.z = -channel_up->get_control_in() / float(RC_SCALE);
    yaw = channel_yaw->get_control_in() / float(RC_SCALE);
}

/**
 * @brief Check if vehicle is disarmed or has completed landing
 * 
 * @details Returns true if the vehicle is in a safe ground state, checking three conditions:
 *          - motors->armed(): false if vehicle is disarmed
 *          - blimp.ap.auto_armed: false if automatic arming criteria not met
 *          - blimp.ap.land_complete: true if landing sequence has completed
 *          
 *          Used by modes to determine if certain control behaviors should be disabled
 *          or if it's safe to perform ground operations. Any one of the three conditions
 *          being true indicates the vehicle should be treated as landed/disarmed.
 * 
 * @return true if vehicle is disarmed, not auto-armed, or has completed landing
 * @return false if vehicle is armed, auto-armed, and not landed
 * 
 * @note This is used for safety checks in various mode control paths
 * @see Mode::run() implementations for usage in mode-specific logic
 * 
 * Source: Blimp/mode.cpp:177-183
 */
bool Mode::is_disarmed_or_landed() const
{
    if (!motors->armed() || !blimp.ap.auto_armed || blimp.ap.land_complete) {
        return true;
    }
    return false;
}

/**
 * @brief Mode member function wrapper for set_mode
 * 
 * @details Convenience wrapper allowing modes to request mode changes via member function
 *          call rather than accessing blimp singleton directly. Simply delegates to the
 *          global Blimp::set_mode() with the same parameters.
 *          
 *          This allows mode implementations to write:
 *            set_mode(Mode::Number::RTL, ModeReason::MISSION_END)
 *          instead of:
 *            blimp.set_mode(Mode::Number::RTL, ModeReason::MISSION_END)
 * 
 * @param[in] mode Mode::Number enum specifying desired flight mode
 * @param[in] reason ModeReason enum indicating why mode change requested
 * 
 * @return true if mode change succeeded, false if rejected
 * 
 * @see Blimp::set_mode() for full implementation details
 * 
 * Source: Blimp/mode.cpp:185-188
 */
bool Mode::set_mode(Mode::Number mode, ModeReason reason)
{
    return blimp.set_mode(mode, reason);
}

/**
 * @brief Get reference to Ground Control Station interface
 * 
 * @details Convenience accessor providing modes with access to GCS communication
 *          functions without directly accessing the blimp singleton. Returns
 *          reference to the GCS_Blimp object for sending telemetry, text messages,
 *          and handling MAVLink communication.
 *          
 *          Typical usage in mode code:
 *            gcs().send_text(MAV_SEVERITY_WARNING, "Mode transition failed");
 * 
 * @return GCS_Blimp& Reference to the GCS interface object
 * 
 * @note Returns reference (not pointer) - always valid, never null
 * @see GCS_Blimp for available communication methods
 * 
 * Source: Blimp/mode.cpp:190-193
 */
GCS_Blimp &Mode::gcs()
{
    return blimp.gcs();
}
