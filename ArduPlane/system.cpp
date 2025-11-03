/**
 * @file system.cpp
 * @brief System initialization, startup sequence, and core vehicle lifecycle management for ArduPlane
 * 
 * @details This file implements the fundamental initialization and system management functions
 *          for the ArduPlane fixed-wing autopilot. It handles the complete vehicle startup
 *          sequence from hardware initialization through to ready-for-flight state, as well
 *          as critical runtime functions like flight mode changes and failsafe monitoring.
 *          
 *          Key responsibilities:
 *          - Complete vehicle initialization sequence (init_ardupilot)
 *          - Flight mode management and transitions (set_mode)
 *          - Inertial navigation system startup (startup_INS)
 *          - Failsafe state monitoring and management
 *          - System health checks and status reporting
 *          
 *          The initialization sequence follows a strict ordering to ensure proper dependency
 *          management between subsystems (e.g., sensors before navigation, navigation before
 *          control systems).
 * 
 * @note This is a safety-critical file. Changes to initialization order or mode change logic
 *       must be carefully reviewed and tested.
 * 
 * @warning Improper initialization sequence can prevent vehicle arming or cause flight instability
 * 
 * Source: ArduPlane/system.cpp
 */

#include "Plane.h"

#include "qautotune.h"

/**
 * @brief Static wrapper for scheduler-based failsafe checking
 * 
 * @details This function serves as a timer callback for the HAL scheduler's
 *          failsafe monitoring system. It provides a static entry point that
 *          delegates to the Plane instance's failsafe_check() method.
 * 
 * @note Registered with hal.scheduler at 1Hz (called every 1000ms) during initialization
 * @see Plane::failsafe_check()
 */
static void failsafe_check_static()
{
    plane.failsafe_check();
}

/**
 * @brief Main initialization function called at vehicle startup
 * 
 * @details Implements the complete ArduPlane initialization sequence from power-on through
 *          ready-for-flight state. This function is called once during vehicle boot from
 *          the HAL main() function and must complete successfully for the vehicle to arm.
 *          
 *          The initialization follows a strict multi-phase sequence to ensure proper
 *          dependency ordering between subsystems:
 *          
 *          **Phase 1: Hardware and Core Subsystem Initialization**
 *          - Configure IMU logging and convert legacy PID parameters
 *          - Initialize RC channels with arming safety options
 *          - Initialize relay outputs for auxiliary equipment
 *          - Initialize notification system (LEDs, buzzers, displays)
 *          - Initialize RC output channels (main servo rail)
 *          - Initialize barometer for altitude sensing
 *          - Initialize rangefinder for terrain sensing (if enabled)
 *          - Initialize battery monitoring and fuel gauges
 *          - Initialize RSSI and RPM sensors (if enabled)
 *          - Setup GCS telemetry serial ports
 *          - Initialize OSD (On-Screen Display) if enabled
 *          
 *          **Phase 2: Sensor Initialization**
 *          - Initialize compass with logging configuration
 *          - Initialize airspeed sensors with fixed-wing parameters
 *          - Initialize GPS with logging configuration
 *          - Initialize RC input channels from receiver
 *          - Initialize camera mount control (if enabled)
 *          - Initialize camera trigger system (if enabled)
 *          - Initialize landing gear control (if enabled)
 *          
 *          **Phase 3: Safety System Setup**
 *          - Configure fence trigger GPIO pin (if hardware-defined)
 *          - Register main loop health monitor (1Hz failsafe callback)
 *          
 *          **Phase 4: QuadPlane Initialization** (if HAL_QUADPLANE_ENABLED)
 *          - Initialize QuadPlane VTOL systems
 *          - Configure multicopter motors and control allocation
 *          - Setup VTOL-specific modes and transitions
 *          
 *          **Phase 5: Parameter and Mode Setup**
 *          - Reload parameter defaults from board configuration
 *          - Set initial flight mode to INITIALIZING state
 *          - Apply ground start delay if configured
 *          
 *          **Phase 6: Navigation System Initialization**
 *          - Perform INS (Inertial Navigation System) ground start
 *          - Initialize AHRS (Attitude and Heading Reference System)
 *          - Configure EKF (Extended Kalman Filter) for fixed-wing operation
 *          - Calibrate barometer ground pressure reference
 *          
 *          **Phase 7: Mission and Logging Setup**
 *          - Initialize mission library for AUTO mode waypoint handling
 *          - Initialize AP_Logger for flight data recording
 *          - Configure vehicle startup message logging
 *          - Reset GCS heartbeat timer to prevent false failsafe triggers
 *          
 *          **Phase 8: Output and Mode Finalization**
 *          - Initialize auxiliary RC outputs (after QuadPlane setup)
 *          - Configure OneShot ESC protocol if enabled
 *          - Configure DShot ESC protocol if enabled
 *          - Set configured initial flight mode from parameters
 *          - Reset mode switch state machine
 *          - Initialize optical flow sensor (if enabled)
 *          - Initialize precision landing system (if enabled)
 *          - Initialize internal combustion engine control (if enabled)
 * 
 * @note Called once at boot from HAL::run() → vehicle setup() → init_ardupilot()
 * @note Initialization order is critical - sensors must initialize before navigation systems,
 *       navigation before control systems, and control systems before mode initialization
 * @note Total initialization time varies by board and sensor count (typically 5-15 seconds)
 * 
 * @warning Failure in critical initialization steps (IMU, barometer, EKF) may prevent
 *          vehicle arming. Pre-arm checks will indicate which systems failed to initialize.
 * @warning Ground start must be performed on a stable, level surface for proper IMU
 *          and barometer calibration
 * @warning Do not move the vehicle during INS calibration phase
 * 
 * @see startup_INS() for inertial navigation initialization details
 * @see set_mode() for flight mode management
 * @see AP_Arming for pre-arm check requirements
 * 
 * Source: ArduPlane/system.cpp:10-178
 */
void Plane::init_ardupilot()
{

    ins.set_log_raw_bit(MASK_LOG_IMU_RAW);

    rollController.convert_pid();
    pitchController.convert_pid();

    // initialise rc channels including setting mode
    // CONVERSION: Added for upgrade to ArduPlane 4.2, Sep 2021
#if HAL_QUADPLANE_ENABLED
    rc().convert_options(RC_Channel::AUX_FUNC::ARMDISARM_UNUSED, (quadplane.enabled() && quadplane.option_is_set(QuadPlane::OPTION::AIRMODE_UNUSED) && (rc().find_channel_for_option(RC_Channel::AUX_FUNC::AIRMODE) == nullptr)) ? RC_Channel::AUX_FUNC::ARMDISARM_AIRMODE : RC_Channel::AUX_FUNC::ARMDISARM);
#else
    rc().convert_options(RC_Channel::AUX_FUNC::ARMDISARM_UNUSED, RC_Channel::AUX_FUNC::ARMDISARM);
#endif
    rc().init();

#if AP_RELAY_ENABLED
    relay.init();
#endif

    // initialise notify system
    notify.init();
    notify_mode(*control_mode);

    init_rc_out_main();

    // init baro
    barometer.init();

#if AP_RANGEFINDER_ENABLED
    // initialise rangefinder
    rangefinder.set_log_rfnd_bit(MASK_LOG_SONAR);
    rangefinder.init(ROTATION_PITCH_270);
#endif

    // initialise battery monitoring
    battery.init();

#if AP_RSSI_ENABLED
    rssi.init();
#endif

#if AP_RPM_ENABLED
    rpm_sensor.init();
#endif

    // setup telem slots with serial ports
    gcs().setup_uarts();


#if OSD_ENABLED
    osd.init();
#endif

    AP::compass().set_log_bit(MASK_LOG_COMPASS);
    AP::compass().init();

#if AP_AIRSPEED_ENABLED
    airspeed.set_fixedwing_parameters(&aparm);
    airspeed.set_log_bit(MASK_LOG_IMU);
#endif

    // GPS Initialization
    gps.set_log_gps_bit(MASK_LOG_GPS);
    gps.init();

    init_rc_in();               // sets up rc channels from radio

#if HAL_MOUNT_ENABLED
    // initialise camera mount
    camera_mount.init();
#endif

#if AP_CAMERA_ENABLED
    // initialise camera
    camera.init();
#endif

#if AP_LANDINGGEAR_ENABLED
    // initialise landing gear position
    g2.landing_gear.init();
#endif

#if FENCE_TRIGGERED_PIN > 0
    hal.gpio->pinMode(FENCE_TRIGGERED_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->write(FENCE_TRIGGERED_PIN, 0);
#endif

    /*
     *  setup the 'main loop is dead' check. Note that this relies on
     *  the RC library being initialised.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

#if HAL_QUADPLANE_ENABLED
    quadplane.setup();
#endif

    AP_Param::reload_defaults_file(true);

    set_mode(mode_initializing, ModeReason::INITIALISED);

#if (GROUND_START_DELAY > 0)
    gcs().send_text(MAV_SEVERITY_NOTICE,"Ground start with delay");
    delay(GROUND_START_DELAY * 1000);
#else
    gcs().send_text(MAV_SEVERITY_INFO,"Ground start");
#endif

    //INS ground start
    //------------------------
    //
    startup_INS();

    // Save the settings for in-air restart
    // ------------------------------------
    //save_EEPROM_groundstart();

    // initialise mission library
    mission.init();
#if HAL_LOGGING_ENABLED
    mission.set_log_start_mission_item_bit(MASK_LOG_CMD);
#endif

    // initialise AP_Logger library
#if HAL_LOGGING_ENABLED
    logger.setVehicle_Startup_Writer(
        FUNCTOR_BIND(&plane, &Plane::Log_Write_Vehicle_Startup_Messages, void)
        );
#endif

    // reset last heartbeat time, so we don't trigger failsafe on slow
    // startup
    gcs().sysid_mygcs_seen(AP_HAL::millis());

    // don't initialise aux rc output until after quadplane is setup as
    // that can change initial values of channels
    init_rc_out_aux();

    if (g2.oneshot_mask != 0) {
        hal.rcout->set_output_mode(g2.oneshot_mask, AP_HAL::RCOutput::MODE_PWM_ONESHOT);
    }
    hal.rcout->set_dshot_esc_type(SRV_Channels::get_dshot_esc_type());

    set_mode_by_number((enum Mode::Number)g.initial_mode.get(), ModeReason::INITIALISED);

    // set the correct flight mode
    // ---------------------------
    rc().reset_mode_switch();

    // initialise sensor
#if AP_OPTICALFLOW_ENABLED
    if (optflow.enabled()) {
        optflow.init(-1);
    }
#endif

#if AC_PRECLAND_ENABLED
    // scheduler table specifies 400Hz, but we can call it no faster
    // than the scheduler loop rate:
    g2.precland.init(MIN(400, scheduler.get_loop_rate_hz()));
#endif

#if AP_ICENGINE_ENABLED
    g2.ice_control.init();
#endif

}

#if AP_FENCE_ENABLED
/**
 * @brief Check if mode change reason indicates an automatic landing sequence transition
 * 
 * @details Determines whether a given ModeReason represents an automatic mode change
 *          that is part of a landing sequence. These transitions are exempt from certain
 *          safety restrictions (like fence breach recovery mode locks) because they are
 *          part of controlled descent and landing procedures.
 *          
 *          Landing sequence reasons include:
 *          - RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND: Transitioning from RTL to
 *            fixed-wing approach and landing
 *          - RTL_COMPLETE_SWITCHING_TO_VTOL_LAND_RTL: QuadPlane transitioning from
 *            fixed-wing RTL to VTOL landing
 *          - QRTL_INSTEAD_OF_RTL: Using QuadPlane RTL instead of fixed-wing RTL
 *          - QLAND_INSTEAD_OF_RTL: Directly entering VTOL land mode instead of RTL
 * 
 * @param[in] reason ModeReason enum to check
 * 
 * @return true if reason indicates landing sequence transition, false otherwise
 * 
 * @note Used by fence breach recovery logic to allow landing sequence mode changes
 *       even when mode changes are otherwise blocked
 * @note QuadPlane landing sequences involve multiple automatic mode transitions
 * 
 * @see set_mode() where this is used for fence recovery mode change validation
 * @see ModeReason enum for complete list of mode change triggers
 * 
 * Source: ArduPlane/system.cpp:185-197
 */
static bool mode_reason_is_landing_sequence(const ModeReason reason)
{
    switch (reason) {
    case ModeReason::RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND:
    case ModeReason::RTL_COMPLETE_SWITCHING_TO_VTOL_LAND_RTL:
    case ModeReason::QRTL_INSTEAD_OF_RTL:
    case ModeReason::QLAND_INSTEAD_OF_RTL:
        return true;
    default:
        break;
    }
    return false;
}
#endif // AP_FENCE_ENABLED

/**
 * @brief Check if a specific flight mode can be entered from the Ground Control Station
 * 
 * @details Validates whether a given flight mode is permitted to be selected via GCS
 *          MAVLink commands based on the FLTMODE_GCSBLOCK parameter bitmask. This allows
 *          operators to prevent inadvertent mode changes from ground stations while still
 *          allowing mode changes via RC transmitter or autonomous events.
 *          
 *          The function checks the requested mode against a list of blockable modes and
 *          tests if the corresponding bit is set in the FLTMODE_GCSBLOCK parameter.
 * 
 * @param[in] mode_num Flight mode number to check for GCS access
 * 
 * @return true if mode can be entered from GCS, false if blocked by parameter
 * 
 * @note Mode changes from RC transmitter, failsafe actions, and autonomous transitions
 *       are not affected by this check - only GCS MAVLink mode change commands
 * @note QuadPlane modes (QSTABILIZE, QHOVER, QLOITER, QACRO, QAUTOTUNE) are included
 *       in the blockable mode list when HAL_QUADPLANE_ENABLED is defined
 * 
 * @see set_mode() for the complete mode change implementation
 * @see FLTMODE_GCSBLOCK parameter for configuration
 * 
 * Source: ArduPlane/system.cpp:200-232
 */
bool Plane::gcs_mode_enabled(const Mode::Number mode_num) const
{
    // List of modes that can be blocked, index is bit number in parameter bitmask
    static const uint8_t mode_list [] {
        (uint8_t)Mode::Number::MANUAL,
        (uint8_t)Mode::Number::CIRCLE,
        (uint8_t)Mode::Number::STABILIZE,
        (uint8_t)Mode::Number::TRAINING,
        (uint8_t)Mode::Number::ACRO,
        (uint8_t)Mode::Number::FLY_BY_WIRE_A,
        (uint8_t)Mode::Number::FLY_BY_WIRE_B,
        (uint8_t)Mode::Number::CRUISE,
        (uint8_t)Mode::Number::AUTOTUNE,
        (uint8_t)Mode::Number::AUTO,
        (uint8_t)Mode::Number::LOITER,
        (uint8_t)Mode::Number::TAKEOFF,
        (uint8_t)Mode::Number::AVOID_ADSB,
        (uint8_t)Mode::Number::GUIDED,
        (uint8_t)Mode::Number::THERMAL,
#if HAL_QUADPLANE_ENABLED
        (uint8_t)Mode::Number::QSTABILIZE,
        (uint8_t)Mode::Number::QHOVER,
        (uint8_t)Mode::Number::QLOITER,
        (uint8_t)Mode::Number::QACRO,
#if QAUTOTUNE_ENABLED
        (uint8_t)Mode::Number::QAUTOTUNE
#endif
#endif
    };

    return !block_GCS_mode_change((uint8_t)mode_num, mode_list, ARRAY_SIZE(mode_list));
}

/**
 * @brief Change vehicle flight mode with safety checks and state management
 * 
 * @details Implements the complete flight mode transition sequence for ArduPlane,
 *          including validation, state backup, mode enter/exit calls, and logging.
 *          This is the primary mode change function called by all mode change paths
 *          (GCS commands, RC switch, failsafe actions, autonomous transitions).
 *          
 *          Mode change sequence:
 *          1. **Check if already in target mode** - Skip transition if already active,
 *             but still trigger notification if reason differs (prevents repeated beeping)
 *          
 *          2. **QuadPlane availability check** (HAL_QUADPLANE_ENABLED) - Validate that
 *             VTOL modes are only entered when QuadPlane is properly enabled and initialized
 *          
 *          3. **Fence breach recovery check** (AP_FENCE_ENABLED) - When armed, fenced,
 *             and recovering from breach, mode changes are blocked except for landing
 *             sequence transitions to prevent circumventing fence recovery
 *          
 *          4. **GCS mode block check** - Apply FLTMODE_GCSBLOCK parameter if mode change
 *             originated from ground control station
 *          
 *          5. **Backup current state** - Save pointers to current and previous modes
 *             and their transition reasons for rollback if new mode entry fails
 *          
 *          6. **Update mode pointers** - Optimistically update control_mode, previous_mode,
 *             and reason tracking (required for mode enter() compatibility)
 *          
 *          7. **Enter new mode** - Call new_mode.enter() which performs mode-specific
 *             initialization and validation. Returns false if mode cannot be entered
 *             due to prerequisites not being met (e.g., no GPS for AUTO mode)
 *          
 *          8. **Rollback on failure** - If enter() fails, restore all backed-up state
 *             pointers and trigger failure notification
 *          
 *          9. **Exit previous mode** - Call old_mode.exit() to perform cleanup and
 *             reset mode-specific state (only after successful new mode entry)
 *          
 *          10. **Log and notify** - Record mode change in dataflash log, update notify
 *              system (LEDs/OSD), send MAVLink heartbeat, and trigger success notification
 * 
 * @param[in] new_mode Reference to target Mode object to transition into
 * @param[in] reason ModeReason enum indicating why mode change was requested
 *                   (GCS_COMMAND, RC_COMMAND, RADIO_FAILSAFE, GCS_FAILSAFE, etc.)
 * 
 * @return true if mode change successful and new mode is now active
 * @return false if mode change denied or new mode entry failed
 * 
 * @note Some mode transitions may be denied based on arming state, fence status,
 *       QuadPlane availability, or GCS block configuration
 * @note Mode enter() can fail if prerequisites are not met (e.g., RTL without home set)
 * @note The mode transition is atomic - either completes fully or rolls back completely
 * @note control_mode pointer is updated before enter() call for legacy compatibility
 *       with mission command callbacks that check control_mode
 * 
 * @warning Mode changes during critical flight phases (fence recovery, landing) may be
 *          restricted by safety logic to prevent unsafe vehicle behavior
 * @warning Failed mode transitions leave vehicle in previous mode, but still log attempt
 *          and notify GCS with failure message
 * @warning Do not call directly from interrupt context - mode transitions involve
 *          potentially blocking operations and complex state changes
 * 
 * @see Mode::enter() for mode-specific initialization requirements
 * @see Mode::exit() for mode-specific cleanup behavior  
 * @see ModeReason enum for complete list of transition triggers
 * @see gcs_mode_enabled() for GCS mode blocking logic
 * 
 * Source: ArduPlane/system.cpp:234-334
 */
bool Plane::set_mode(Mode &new_mode, const ModeReason reason)
{

    if (control_mode == &new_mode) {
        // don't switch modes if we are already in the correct mode.
        // only make happy noise if using a different method to switch, this stops beeping for repeated change mode requests from GCS
        if ((reason != control_mode_reason) && (reason != ModeReason::INITIALISED)) {
            AP_Notify::events.user_mode_change = 1;
        }
        return true;
    }

#if HAL_QUADPLANE_ENABLED
    if (new_mode.is_vtol_mode() && !plane.quadplane.available()) {
        // dont try and switch to a Q mode if quadplane is not enabled and initialized
        gcs().send_text(MAV_SEVERITY_INFO,"Q_ENABLE 0");
        // make sad noise
        if (reason != ModeReason::INITIALISED) {
            AP_Notify::events.user_mode_change_failed = 1;
        }
        return false;
    }

#else
    if (new_mode.is_vtol_mode()) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        gcs().send_text(MAV_SEVERITY_INFO,"HAL_QUADPLANE_ENABLED=0");
        // make sad noise
        if (reason != ModeReason::INITIALISED) {
            AP_Notify::events.user_mode_change_failed = 1;
        }
        return false;
    }
#endif  // HAL_QUADPLANE_ENABLED

#if AP_FENCE_ENABLED
    // may not be allowed to change mode if recovering from fence breach
    if (hal.util->get_soft_armed() &&
        fence.enabled() &&
        fence.option_enabled(AC_Fence::OPTIONS::DISABLE_MODE_CHANGE) &&
        fence.get_breaches() &&
        in_fence_recovery() &&
        !mode_reason_is_landing_sequence(reason)) {
        gcs().send_text(MAV_SEVERITY_NOTICE,"Mode change to %s denied, in fence recovery", new_mode.name());
        AP_Notify::events.user_mode_change_failed = 1;
        return false;
    }
#endif

    // Check if GCS mode change is disabled via parameter
    if ((reason == ModeReason::GCS_COMMAND) && !gcs_mode_enabled(new_mode.mode_number())) {
        gcs().send_text(MAV_SEVERITY_NOTICE,"Mode change to %s denied, GCS entry disabled (FLTMODE_GCSBLOCK)", new_mode.name());
        return false;
    }

    // backup current control_mode and previous_mode
    Mode &old_previous_mode = *previous_mode;
    Mode &old_mode = *control_mode;

    // update control_mode assuming success
    // TODO: move these to be after enter() once start_command_callback() no longer checks control_mode
    previous_mode = control_mode;
    control_mode = &new_mode;
    const ModeReason  old_previous_mode_reason = previous_mode_reason;
    previous_mode_reason = control_mode_reason;
    control_mode_reason = reason;

    // attempt to enter new mode
    if (!new_mode.enter()) {
        // Log error that we failed to enter desired flight mode
        gcs().send_text(MAV_SEVERITY_WARNING, "Flight mode change failed");

        // we failed entering new mode, roll back to old
        previous_mode = &old_previous_mode;
        control_mode = &old_mode;
        control_mode_reason = previous_mode_reason;
        previous_mode_reason = old_previous_mode_reason;

        // make sad noise
        if (reason != ModeReason::INITIALISED) {
            AP_Notify::events.user_mode_change_failed = 1;
        }
        return false;
    }

    // exit previous mode
    old_mode.exit();

    // log and notify mode change
#if HAL_LOGGING_ENABLED
    logger.Write_Mode(control_mode->mode_number(), control_mode_reason);
#endif
    notify_mode(*control_mode);
    gcs().send_message(MSG_HEARTBEAT);

    // make happy noise
    if (reason != ModeReason::INITIALISED) {
        AP_Notify::events.user_mode_change = 1;
    }
    return true;
}

/**
 * @brief Change flight mode using uint8_t mode number (convenience overload)
 * 
 * @details Provides backward compatibility and convenience wrapper for mode changes
 *          when mode is specified as a uint8_t instead of Mode::Number enum.
 *          Performs compile-time size assertion to ensure safe casting, then delegates
 *          to set_mode_by_number().
 * 
 * @param[in] new_mode Mode number as uint8_t (typically from MAVLink or parameters)
 * @param[in] reason Why the mode change is being requested
 * 
 * @return true if mode change successful, false otherwise
 * 
 * @note This is a thin wrapper - see set_mode(Mode&, ModeReason) for complete behavior
 * @see set_mode_by_number() for the mode lookup implementation
 * 
 * Source: ArduPlane/system.cpp:336-341
 */
bool Plane::set_mode(const uint8_t new_mode, const ModeReason reason)
{
    static_assert(sizeof(Mode::Number) == sizeof(new_mode), "The new mode can't be mapped to the vehicles mode number");

    return set_mode_by_number(static_cast<Mode::Number>(new_mode), reason);
}

/**
 * @brief Change flight mode by looking up Mode object from mode number
 * 
 * @details Converts a Mode::Number enum to the corresponding Mode object pointer
 *          using the vehicle's mode lookup table, then delegates to the primary
 *          set_mode(Mode&, ModeReason) function. This handles the indirection
 *          needed when mode is specified by number rather than by object reference.
 * 
 * @param[in] new_mode_number Mode::Number enum value identifying target mode
 * @param[in] reason Why the mode change is being requested
 * 
 * @return true if mode exists and change successful, false if mode invalid or change failed
 * 
 * @note Returns false immediately if new_mode_number does not map to a valid mode
 *       (e.g., unsupported mode number or disabled compile-time feature)
 * @note Triggers "No such mode" notification if mode number is invalid
 * 
 * @see set_mode(Mode&, ModeReason) for complete mode change behavior
 * @see mode_from_mode_num() for mode number to Mode object mapping
 * 
 * Source: ArduPlane/system.cpp:343-351
 */
bool Plane::set_mode_by_number(const Mode::Number new_mode_number, const ModeReason reason)
{
    Mode *new_mode = plane.mode_from_mode_num(new_mode_number);
    if (new_mode == nullptr) {
        notify_no_such_mode(new_mode_number);
        return false;
    }
    return set_mode(*new_mode, reason);
}

/**
 * @brief Monitor and trigger long failsafe conditions (radio and GCS telemetry loss)
 * 
 * @details Continuously monitors radio control link and ground control station telemetry
 *          health, triggering appropriate failsafe actions when communication is lost for
 *          longer than the configured FS_TIMEOUT_LONG threshold. Long failsafe typically
 *          initiates RTL (Return to Launch) or other configured recovery actions.
 *          
 *          Monitored failsafe conditions:
 *          
 *          **Radio Failsafe** - Triggered when:
 *          - RC failsafe flag is set (no valid RC input)
 *          - Time since last valid RC exceeds FS_TIMEOUT_LONG parameter
 *          - Not currently landing
 *          - Not already in long failsafe state
 *          
 *          **GCS Heartbeat Failsafe** - Triggered based on GCS_FAILSAFE_ENABLED parameter:
 *          - GCS_FAILSAFE_HB_AUTO mode: Only in AUTO flight mode
 *          - GCS_FAILSAFE_HEARTBEAT mode: In any flight mode
 *          - GCS_FAILSAFE_HB_RSSI mode: Based on telemetry radio RSSI packets
 *          - Time since last GCS heartbeat exceeds FS_TIMEOUT_LONG
 *          
 *          **Recovery from Failsafe** - Long failsafe clears when:
 *          - Radio control is restored (for radio failsafe)
 *          - GCS heartbeat is restored (for GCS failsafe)
 *          - Timeout drops below FS_TIMEOUT_SHORT to prevent rapid oscillation
 *          
 *          Failsafe is inhibited during landing phase to prevent mode changes that could
 *          disrupt final approach and touchdown.
 * 
 * @note Called periodically from main loop (typically at 10Hz)
 * @note Different from short failsafe (FS_TIMEOUT_SHORT) which may have different actions
 * @note Failsafe actions are configured via FS_LONG_ACTION and FS_SHORT_ACTION parameters
 * @note GCS failsafe behavior depends on GCS_FAILSAFE_ENABLED parameter setting
 * @note Does not trigger during LAND mode to avoid interrupting landing sequence
 * 
 * @warning Radio and GCS failsafe use different timeout references - radio failsafe
 *          uses last valid RC time, GCS failsafe uses last seen GCS heartbeat time
 * @warning Failsafe timing is critical for safe operation - too short causes false triggers,
 *          too long delays recovery action
 * @warning RSSI-based GCS failsafe requires telemetry radio to send RADIO_STATUS messages
 * 
 * @see check_short_failsafe() for short-term failsafe monitoring
 * @see failsafe_long_on_event() for failsafe trigger actions
 * @see failsafe_long_off_event() for failsafe recovery actions
 * @see FS_TIMEOUT_LONG parameter for long failsafe timeout configuration
 * @see GCS_FAILSAFE_ENABLED parameter for GCS failsafe mode selection
 * 
 * Source: ArduPlane/system.cpp:353-398
 */
void Plane::check_long_failsafe()
{
    const uint32_t gcs_last_seen_ms = gcs().sysid_mygcs_last_seen_time_ms();
    const uint32_t tnow = millis();
    // only act on changes
    // -------------------
    if (failsafe.state != FAILSAFE_LONG && failsafe.state != FAILSAFE_GCS && flight_stage != AP_FixedWing::FlightStage::LAND) {
        uint32_t radio_timeout_ms = failsafe.last_valid_rc_ms;
        if (failsafe.state == FAILSAFE_SHORT) {
            // time is relative to when short failsafe enabled
            radio_timeout_ms = failsafe.short_timer_ms;
        }
        if (failsafe.rc_failsafe &&
            (tnow - radio_timeout_ms) > g.fs_timeout_long*1000) {
            failsafe_long_on_event(FAILSAFE_LONG, ModeReason::RADIO_FAILSAFE);
        } else if (g.gcs_heartbeat_fs_enabled == GCS_FAILSAFE_HB_AUTO && control_mode == &mode_auto &&
                   gcs_last_seen_ms != 0 &&
                   (tnow - gcs_last_seen_ms) > g.fs_timeout_long*1000) {
            failsafe_long_on_event(FAILSAFE_GCS, ModeReason::GCS_FAILSAFE);
        } else if ((g.gcs_heartbeat_fs_enabled == GCS_FAILSAFE_HEARTBEAT ||
                    g.gcs_heartbeat_fs_enabled == GCS_FAILSAFE_HB_RSSI) &&
                   gcs_last_seen_ms != 0 &&
                   (tnow - gcs_last_seen_ms) > g.fs_timeout_long*1000) {
            failsafe_long_on_event(FAILSAFE_GCS, ModeReason::GCS_FAILSAFE);
        } else if (g.gcs_heartbeat_fs_enabled == GCS_FAILSAFE_HB_RSSI && 
                   gcs().chan(0) != nullptr &&
                   gcs().chan(0)->last_radio_status_remrssi_ms() != 0 &&
                   (tnow - gcs().chan(0)->last_radio_status_remrssi_ms()) > g.fs_timeout_long*1000) {
            failsafe_long_on_event(FAILSAFE_GCS, ModeReason::GCS_FAILSAFE);
        }
    } else {
        float timeout_seconds = g.fs_timeout_long;
        if (g.fs_action_short != FS_ACTION_SHORT_DISABLED) {
            // avoid dropping back into short timeout
            timeout_seconds = g.fs_timeout_short;
        }
        // We do not change state but allow for user to change mode
        if (failsafe.state == FAILSAFE_GCS && 
            (tnow - gcs_last_seen_ms) < timeout_seconds*1000) {
            failsafe_long_off_event(ModeReason::GCS_FAILSAFE);
        } else if (failsafe.state == FAILSAFE_LONG && 
                   !failsafe.rc_failsafe) {
            failsafe_long_off_event(ModeReason::RADIO_FAILSAFE);
        }
    }
}

/**
 * @brief Monitor and trigger short failsafe condition (initial radio loss detection)
 * 
 * @details Monitors for the initial loss of radio control signal and triggers short
 *          failsafe actions when RC input becomes invalid. Short failsafe represents
 *          the first stage of radio failure response, typically occurring after 1-2
 *          seconds of signal loss (configurable via FS_TIMEOUT_SHORT parameter).
 *          
 *          Short failsafe behavior:
 *          
 *          **Failsafe Activation** - Triggered when all conditions met:
 *          - FS_SHORT_ACTION parameter is not disabled
 *          - Currently in FAILSAFE_NONE state (not already in failsafe)
 *          - Not in LAND mode (landing sequence has priority)
 *          - RC failsafe flag is set (RC signal lost or invalid)
 *          
 *          **Failsafe Recovery** - Clears when:
 *          - RC signal is restored (rc_failsafe flag clears)
 *          - FS_SHORT_ACTION is disabled via parameter
 *          
 *          Short failsafe typically triggers:
 *          - Circle mode at current location (FS_SHORT_ACTION = 0)
 *          - RTL (Return to Launch) (FS_SHORT_ACTION = 1)
 *          - QRTL for QuadPlane (FS_SHORT_ACTION = 4)
 *          
 *          If short failsafe persists beyond FS_TIMEOUT_LONG, long failsafe will
 *          activate with potentially different recovery action.
 * 
 * @note Called periodically from main loop (typically at 10Hz)
 * @note RC failsafe flag is set by RC input processing when signal is lost or invalid
 * @note Short failsafe is the first stage - long failsafe follows if signal not restored
 * @note Does not trigger during LAND mode to avoid disrupting landing sequence
 * @note Recovery is immediate when RC signal restored (no hysteresis delay)
 * 
 * @warning The rc_failsafe flag must be properly managed by RC input processing for
 *          this function to work correctly
 * @warning Disabling short failsafe (FS_SHORT_ACTION = -1) removes this safety layer
 * 
 * @see check_long_failsafe() for extended failsafe monitoring
 * @see failsafe_short_on_event() for short failsafe trigger actions
 * @see failsafe_short_off_event() for short failsafe recovery actions
 * @see FS_TIMEOUT_SHORT parameter for short failsafe timeout configuration
 * @see FS_SHORT_ACTION parameter for failsafe action selection
 * 
 * Source: ArduPlane/system.cpp:400-418
 */
void Plane::check_short_failsafe()
{
    // only act on changes
    // -------------------
    if (g.fs_action_short != FS_ACTION_SHORT_DISABLED &&
       failsafe.state == FAILSAFE_NONE &&
       flight_stage != AP_FixedWing::FlightStage::LAND) {
        // The condition is checked and the flag rc_failsafe is set in radio.cpp
        if(failsafe.rc_failsafe) {
            failsafe_short_on_event(FAILSAFE_SHORT, ModeReason::RADIO_FAILSAFE);
        }
    }

    if(failsafe.state == FAILSAFE_SHORT) {
        if(!failsafe.rc_failsafe || g.fs_action_short == FS_ACTION_SHORT_DISABLED) {
            failsafe_short_off_event(ModeReason::RADIO_FAILSAFE);
        }
    }
}

/**
 * @brief Initialize Inertial Navigation System (INS) and perform ground calibration
 * 
 * @details Performs the critical INS ground start sequence that establishes the inertial
 *          reference frame and calibrates sensors for flight. This function must complete
 *          successfully with the vehicle stationary and level for proper attitude estimation
 *          and navigation performance.
 *          
 *          Initialization sequence:
 *          1. **Gyroscope calibration** - If gyro calibration is enabled (not GYRO_CAL_NEVER),
 *             performs gyro bias estimation while vehicle is stationary. User must not move
 *             the vehicle during this phase (typically 30 seconds).
 *          
 *          2. **AHRS initialization** - Initialize Attitude and Heading Reference System:
 *             - Configure for fixed-wing flight dynamics (fly_forward = true)
 *             - Set vehicle class to FIXED_WING for appropriate EKF tuning
 *             - Enable wind estimation for airspeed/groundspeed comparison
 *          
 *          3. **INS initialization** - Initialize inertial sensor system at configured
 *             loop rate (typically 400Hz for Plane). This starts periodic sampling of
 *             accelerometers and gyroscopes.
 *          
 *          4. **AHRS reset** - Reset AHRS state after INS initialization to clear any
 *             transient data from sensor startup
 *          
 *          5. **Barometer calibration** - Establish ground pressure reference:
 *             - Configure barometer logging
 *             - Sample barometer multiple times to establish stable ground pressure
 *             - Set this as altitude zero reference for relative altitude calculations
 * 
 * @note Called from init_ardupilot() during vehicle boot sequence
 * @note Vehicle MUST be stationary and level during this procedure
 * @note Gyro calibration can be disabled with INS_GYR_CAL parameter for quick reboots,
 *       but initial calibration should always be performed after installation
 * @note Ground pressure reference is critical for altitude hold and terrain following
 * 
 * @warning Do not move vehicle during INS calibration - movement will corrupt gyro bias
 *          estimates and lead to attitude estimation errors in flight
 * @warning Calibration must be performed on level ground - tilt during calibration will
 *          introduce constant attitude error
 * @warning Calibration environment should be vibration-free - engine vibration or wind
 *          buffeting will degrade sensor calibration quality
 * 
 * @see AP_InertialSensor::init() for sensor sampling initialization
 * @see AP_AHRS::init() for attitude estimation setup
 * @see AP_Baro::calibrate() for pressure reference establishment
 * @see INS_GYR_CAL parameter for gyro calibration configuration
 * 
 * Source: ArduPlane/system.cpp:421-441
 */
void Plane::startup_INS(void)
{
    if (ins.gyro_calibration_timing() != AP_InertialSensor::GYRO_CAL_NEVER) {
        gcs().send_text(MAV_SEVERITY_ALERT, "Beginning INS calibration. Do not move plane");
    } else {
        gcs().send_text(MAV_SEVERITY_ALERT, "Skipping INS calibration");
    }

    ahrs.init();
    ahrs.set_fly_forward(true);
    ahrs.set_vehicle_class(AP_AHRS::VehicleClass::FIXED_WING);
    ahrs.set_wind_estimation_enabled(true);

    ins.init(scheduler.get_loop_rate_hz());
    ahrs.reset();

    // read Baro pressure at ground
    //-----------------------------
    barometer.set_log_baro_bit(MASK_LOG_IMU);
    barometer.calibrate();
}

/**
 * @brief Update notification system with current flight mode information
 * 
 * @details Updates the AP_Notify system (LEDs, buzzers, OSD, displays) with the current
 *          flight mode number and name. This ensures that external indicators properly
 *          reflect the active flight mode for pilot awareness.
 *          
 *          The notification system uses this information to:
 *          - Drive RGB LED color patterns specific to each mode
 *          - Display mode name on OSD and external displays
 *          - Select appropriate buzzer tones for mode changes
 *          - Update ground control station displays
 * 
 * @param[in] mode Reference to Mode object whose information should be displayed
 * 
 * @note Called automatically by set_mode() after successful mode transition
 * @note Mode number is used for LED patterns, 4-character name for text displays
 * 
 * @see AP_Notify for notification system architecture
 * @see set_mode() where this is called after mode changes
 * 
 * Source: ArduPlane/system.cpp:443-448
 */
void Plane::notify_mode(const Mode& mode)
{
    notify.flags.flight_mode = mode.mode_number();
    notify.set_flight_mode_str(mode.name4());
}

#if HAL_LOGGING_ENABLED
/**
 * @brief Check if a specific log message type should be written now
 * 
 * @details Determines whether a given log message type is currently enabled for
 *          logging based on the LOG_BITMASK parameter and current vehicle state
 *          (armed, flying, etc.). This is used throughout the codebase to conditionally
 *          write log messages only when their corresponding log category is enabled.
 *          
 *          Common log mask bits:
 *          - MASK_LOG_ATTITUDE_FAST - High-rate attitude data
 *          - MASK_LOG_IMU - IMU sensor data
 *          - MASK_LOG_GPS - GPS position and velocity
 *          - MASK_LOG_PM - Performance monitoring
 *          - MASK_LOG_CTUN - Control tuning data
 *          - MASK_LOG_NTUN - Navigation tuning data
 *          - MASK_LOG_MODE - Mode changes
 *          - MASK_LOG_CMD - Mission commands
 * 
 * @param[in] mask Bitmask of log type(s) to check (e.g., MASK_LOG_ATTITUDE_FAST)
 * 
 * @return true if logging is enabled for this message type, false otherwise
 * 
 * @note This is a thin wrapper around AP_Logger::should_log()
 * @note LOG_BITMASK parameter controls which message types are enabled
 * @note Some high-rate messages may be throttled even if enabled to manage storage bandwidth
 * 
 * @see AP_Logger::should_log() for detailed logging policy implementation
 * @see LOG_BITMASK parameter for log category configuration
 * 
 * Source: ArduPlane/system.cpp:454-457
 */
bool Plane::should_log(uint32_t mask)
{
    return logger.should_log(mask);
}
#endif

/**
 * @brief Calculate current throttle output as percentage for display and logging
 * 
 * @details Converts the current throttle servo output to a percentage value suitable
 *          for display on OSD, ground control stations, and log analysis. The return
 *          value range depends on whether reverse thrust is configured:
 *          - Normal operation: 0 to 100% (0% = idle, 100% = full throttle)
 *          - Reverse thrust enabled: -100% to +100% (-100% = full reverse, 0% = idle,
 *            +100% = full forward)
 *          
 *          For QuadPlane vehicles in VTOL modes (except during tailsitter transition),
 *          returns the multicopter motor throttle percentage instead of fixed-wing
 *          throttle, providing appropriate feedback for the active flight mode.
 * 
 * @return Throttle percentage as int8_t:
 *         - 0 to 100 when reverse thrust not configured
 *         - -100 to 100 when reverse thrust enabled
 *         - Multicopter throttle (0-100) in QuadPlane VTOL modes
 * 
 * @note Return value is constrained to specified ranges to prevent overflow
 * @note QuadPlane transition modes use fixed-wing throttle (not VTOL motors)
 * @note Reverse thrust availability is determined by THR_MIN parameter
 * @note This is display/logging data only - does not affect actual motor control
 * 
 * @see SRV_Channel::k_throttle for throttle servo channel
 * @see SRV_Channels::get_output_scaled() for servo output reading
 * @see have_reverse_thrust() for reverse thrust configuration check
 * @see QuadPlane::motors for multicopter motor control
 * @see THR_MIN parameter for reverse thrust configuration
 * 
 * Source: ArduPlane/system.cpp:463-475
 */
int8_t Plane::throttle_percentage(void)
{
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode() && !quadplane.tailsitter.in_vtol_transition()) {
        return quadplane.motors->get_throttle_out() * 100.0;
    }
#endif
    float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    if (!have_reverse_thrust()) {
        return constrain_int16(throttle, 0, 100);
    }
    return constrain_int16(throttle, -100, 100);
}
