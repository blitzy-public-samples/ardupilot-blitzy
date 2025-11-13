/**
 * @file system.cpp
 * @brief Implementation of rover system initialization, scheduler setup, and mode switching
 * 
 * @details This file contains the core system initialization sequence for ArduPilot Rover,
 *          including init_ardupilot() which performs the complete vehicle startup sequence,
 *          mode switching logic via set_mode(), scheduler task registration, and vehicle
 *          type-specific configuration for rovers, boats, balance bots, and sailboats.
 * 
 *          Key functions:
 *          - init_ardupilot(): Main initialization sequence called at vehicle startup
 *          - set_mode(): Mode switching with safety validation
 *          - startup_INS(): Inertial navigation system calibration
 *          - update_ahrs_flyforward(): Vehicle movement-based heading estimation control
 * 
 * Source: Rover/system.cpp
 */

#include "Rover.h"

/**
 * @brief Static callback wrapper for scheduler failsafe check timer
 * 
 * @details This static function is registered with the HAL scheduler as a timer failsafe
 *          callback. It runs at 1Hz (1000ms interval) to monitor main loop health and
 *          trigger failsafe actions if the main loop becomes unresponsive.
 * 
 * @note Called at 1Hz by scheduler timer failsafe mechanism
 * @warning Critical for detecting main loop failures and triggering safety responses
 * 
 * @see Rover::failsafe_check()
 * @see hal.scheduler->register_timer_failsafe() in init_ardupilot()
 */
static void failsafe_check_static()
{
    rover.failsafe_check();
}

/**
 * @brief Main ArduPilot initialization sequence for Rover
 * 
 * @details This function performs the complete vehicle startup and initialization sequence,
 *          called once during boot after HAL (Hardware Abstraction Layer) initialization.
 *          
 *          Initialization sequence:
 *          1. Notification system (LEDs, buzzers)
 *          2. Battery monitoring
 *          3. RPM and RSSI sensors
 *          4. Barometer (before GCS for CLI support)
 *          5. Serial ports and GCS communication
 *          6. On-screen display (OSD)
 *          7. Compass (magnetometer)
 *          8. Airspeed sensor (if equipped)
 *          9. Rangefinder (distance sensors)
 *          10. Proximity sensors (obstacle detection)
 *          11. Beacon positioning system
 *          12. GPS receivers
 *          13. Inertial sensors (IMU - gyro/accelerometer)
 *          14. RC input channels
 *          15. Motors and servo outputs
 *          16. Wheel encoders (for dead reckoning)
 *          17. Torqeedo motor driver (electric propulsion)
 *          18. Optical flow sensor
 *          19. Relay outputs
 *          20. Camera mount (gimbal)
 *          21. Camera trigger
 *          22. Precision landing
 *          23. Scheduler failsafe timer registration
 *          24. SmartRTL path recording
 *          25. Object avoidance
 *          26. Initial mode setting
 *          27. INS (Inertial Navigation System) startup and calibration
 *          28. Mission library
 *          29. Logging system
 *          30. RC channel option conversion
 *          31. Sailboat-specific initialization
 *          32. Vehicle type-specific configuration (rover/boat behavior)
 * 
 * @note This function must complete successfully before vehicle can arm
 * @warning Initialization order is critical - dependencies must be initialized before dependent systems
 * @warning INS calibration requires vehicle to be stationary - movement during calibration will cause errors
 * 
 * @see startup_INS() for inertial navigation system initialization
 * @see set_mode() for mode switching after initialization
 */
void Rover::init_ardupilot()
{
    // Step 1: Initialize notification system (LEDs, buzzers, displays)
    // Must be first so errors during initialization can be signaled to user
    notify.init();
    notify_mode(control_mode);

    // Step 2: Initialize battery monitoring for voltage/current sensing and capacity tracking
    // Critical for low battery failsafe detection
    battery.init();

#if AP_RPM_ENABLED
    // Step 3: Initialize RPM sensor for engine/motor speed monitoring
    rpm_sensor.init();
#endif

#if AP_RSSI_ENABLED
    // Step 4: Initialize RSSI (Received Signal Strength Indicator) for RC signal monitoring
    rssi.init();
#endif

    // Step 5: Initialize wind vane for sailboat wind direction sensing
    // Requires serial_manager for NMEA wind sensor communication
    g2.windvane.init(serial_manager);

    // Step 6: Initialize barometer (pressure sensor) before GCS initialization
    // Done early so CLI barometer test commands work during setup
    barometer.init();

    // Step 7: Setup Ground Control Station (GCS) telemetry communication
    // Configures MAVLink on all available serial ports based on SERIAL parameters
    gcs().setup_uarts();

#if OSD_ENABLED
    // Step 8: Initialize On-Screen Display (OSD) for video overlay telemetry
    osd.init();
#endif

    // Step 9: Initialize compass (magnetometer) for heading sensing
    // Configure logging and start compass driver detection/initialization
    AP::compass().set_log_bit(MASK_LOG_COMPASS);
    AP::compass().init();

#if AP_AIRSPEED_ENABLED
    // Step 10: Configure airspeed sensor logging (used on some rover applications)
    airspeed.set_log_bit(MASK_LOG_IMU);
#endif

#if AP_RANGEFINDER_ENABLED
    // Step 11: Initialize rangefinder (distance sensors: lidar, sonar, radar)
    // Used for obstacle avoidance, terrain following, and object detection
    // ROTATION_NONE indicates sensor is mounted facing forward
    rangefinder.set_log_rfnd_bit(MASK_LOG_RANGEFINDER);
    rangefinder.init(ROTATION_NONE);
#endif

#if HAL_PROXIMITY_ENABLED
    // Step 12: Initialize proximity sensors for 360-degree obstacle detection
    // Supports multiple sensor types for collision avoidance
    g2.proximity.init();
#endif

#if AP_BEACON_ENABLED
    // Step 13: Initialize beacon positioning system for GPS-denied navigation
    // Beacons provide triangulation-based position estimation indoors or in GPS-denied areas
    g2.beacon.init();
#endif

    // Step 14: Configure barometer for EKF (Extended Kalman Filter) altitude estimation
    // Calibration corrects for current pressure/temperature conditions
    // @warning Vehicle must be stationary during barometer calibration
    barometer.set_log_baro_bit(MASK_LOG_IMU);
    barometer.calibrate();

    // Step 15: Initialize GPS receivers for position/velocity estimation
    // Supports multiple GPS units and protocols (UBLOX, NMEA, SBF, etc.)
    gps.set_log_gps_bit(MASK_LOG_GPS);
    gps.init();

    // Step 16: Configure inertial sensor (IMU) raw data logging
    // Raw gyro/accelerometer data logged for analysis and replay
    ins.set_log_raw_bit(MASK_LOG_IMU_RAW);

    // Step 17: Initialize RC (Radio Control) input channels
    // Sets up stick deadzone, channel mapping, and input filtering
    init_rc_in();

    // Step 18: Initialize motor and servo output system
    // Frame type (rover, boat, balance bot, sailboat) determines output configuration
    // Sets PWM ranges, output functions, and motor mixing for vehicle type
    g2.motors.init(get_frame_type());
    AP::srv().enable_aux_servos();

    // Step 19: Initialize wheel encoders for dead reckoning and wheel odometry
    // Provides velocity estimation independent of GPS for improved navigation
    g2.wheel_encoder.init();

#if HAL_TORQEEDO_ENABLED
    // Step 20: Initialize Torqeedo electric motor driver
    // Provides advanced control and telemetry for Torqeedo electric propulsion systems
    g2.torqeedo.init();
#endif

#if AP_OPTICALFLOW_ENABLED
    // Step 21: Initialize optical flow sensor for ground-relative velocity estimation
    // Complements GPS by providing velocity when stationary or at low speeds
    optflow.init(MASK_LOG_OPTFLOW);
#endif      // AP_OPTICALFLOW_ENABLED

#if AP_RELAY_ENABLED
    // Step 22: Initialize relay outputs for on/off control of external devices
    relay.init();
#endif

#if HAL_MOUNT_ENABLED
    // Step 23: Initialize camera mount (gimbal) control system
    // Supports stabilized camera pointing and tracking
    camera_mount.init();
#endif

#if AP_CAMERA_ENABLED
    // Step 24: Initialize camera trigger and control
    // Supports mission-based camera triggering and geotagging
    camera.init();
#endif

#if AC_PRECLAND_ENABLED
    // Step 25: Initialize precision landing system
    // Uses IR beacons or visual markers for accurate landing positioning
    init_precland();
#endif

    /*
     * Step 26: Register scheduler failsafe timer for main loop health monitoring
     * 
     * This critical safety mechanism registers a 1Hz (1000ms interval) timer callback
     * that monitors main loop responsiveness. If the main loop hangs or becomes too slow,
     * the failsafe_check_static() callback will detect the failure and trigger appropriate
     * failsafe actions to maintain vehicle safety.
     * 
     * @note Relies on RC library being initialized first to detect RC failsafe conditions
     * @warning Critical safety system - must not be disabled or modified without careful analysis
     * @warning If this timer doesn't fire, main loop has failed and vehicle safety is compromised
     */
    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

    // Step 27: Initialize SmartRTL (Smart Return-To-Launch) path recording
    // Records path taken for intelligent return route that avoids obstacles encountered during flight
    g2.smart_rtl.init();

#if AP_OAPATHPLANNER_ENABLED
    // Step 28: Initialize object avoidance path planner
    // Provides dynamic path planning around detected obstacles using proximity/rangefinder data
    g2.oa.init();
#endif

    // Step 29: Set initial mode to INITIALIZING during startup
    // Vehicle will transition to configured initial mode once startup completes
    set_mode(mode_initializing, ModeReason::INITIALISED);

    // Step 30: Start Inertial Navigation System (INS) and perform calibration
    // @warning Critical initialization step - vehicle MUST be stationary during INS calibration
    // @warning Movement during calibration will result in attitude/position estimation errors
    startup_INS();

#if AP_MISSION_ENABLED
    // Step 31: Initialize mission library for autonomous waypoint navigation
    // Loads mission from storage and prepares mission execution system
    mode_auto.mission.init();
#if HAL_LOGGING_ENABLED
    // Configure logging of mission item execution for mission debugging and analysis
    mode_auto.mission.set_log_start_mission_item_bit(MASK_LOG_CMD);
#endif
#endif

    // Step 32: Initialize AP_Logger (DataFlash logging) system
    // Register callback to write vehicle-specific startup messages to log
#if HAL_LOGGING_ENABLED
    logger.setVehicle_Startup_Writer(
        FUNCTOR_BIND(&rover, &Rover::Log_Write_Vehicle_Startup_Messages, void)
        );
#endif

    // Step 33: Determine and set the user-configured initial flight mode
    // Reads INITIAL_MODE parameter and validates it exists for this vehicle configuration
    // Falls back to INITIALIZING mode if configured mode is invalid
    Mode *initial_mode = mode_from_mode_num((enum Mode::Number)g.initial_mode.get());
    if (initial_mode == nullptr) {
        initial_mode = &mode_initializing;
    }
    set_mode(*initial_mode, ModeReason::INITIALISED);

    // Step 34: Initialize RC channel auxiliary function mappings
    // Convert legacy option assignments to current function assignments
    // Sets up arm/disarm switch and servo trim save functions
    rc().convert_options(RC_Channel::AUX_FUNC::ARMDISARM_UNUSED, RC_Channel::AUX_FUNC::ARMDISARM);
    rc().convert_options(RC_Channel::AUX_FUNC::SAVE_TRIM, RC_Channel::AUX_FUNC::TRIM_TO_CURRENT_SERVO_RC);
    rc().init();

    // Step 35: Initialize sailboat-specific systems (wind vane, sail control, tacking logic)
    rover.g2.sailboat.init();

    // Step 36: Configure vehicle type-specific behavior
    // Boats should LOITER after mission completion to maintain position and avoid drifting
    // Land vehicles can HOLD without drifting due to static friction
    // This sets appropriate default behavior based on frame class
    if (is_boat()) {
        rover.g2.mis_done_behave.set_default(uint8_t(ModeAuto::DoneBehaviour::LOITER));
    }

    // Step 37: Set initialization complete flag
    // Vehicle is now ready for arming checks and operation
    // @note Arming will still require additional pre-arm safety checks to pass
    initialised = true;
}

/**
 * @brief Update AHRS flyforward setting to enable movement-based heading estimation
 * 
 * @details The AHRS (Attitude Heading Reference System) can use vehicle movement to improve
 *          heading estimation when the vehicle is moving forward. This is particularly useful
 *          for ground vehicles without a compass or when compass is unreliable.
 *          
 *          Flyforward mode is enabled when:
 *          - Vehicle is NOT a boat (boats can drift sideways, making movement unreliable)
 *          - Throttle exceeds threshold (15% or half cruise throttle, whichever is lower)
 *          - OR desired speed control is active and target speed > 0.5 m/s
 *          - Above conditions maintained for 2 seconds (hysteresis to avoid oscillation)
 *          
 *          When flyforward is active, AHRS uses GPS velocity vector to estimate heading,
 *          which can be more accurate than compass in some environments (magnetic interference).
 * 
 * @note Called regularly from main loop to update AHRS state based on vehicle movement
 * @note Boats always have flyforward disabled because they can drift laterally
 * @note 2-second delay prevents rapid switching during throttle transients
 * 
 * @see Rover::is_boat() for vehicle type detection
 * @see ahrs.set_fly_forward() for AHRS flyforward control
 */
void Rover::update_ahrs_flyforward()
{
    bool flyforward = false;

    // Boats never use movement to estimate heading because they can drift sideways
    // due to currents and wind, making velocity vector unreliable for heading
    if (!is_boat()) {
        // Throttle threshold is 15% or 1/2 cruise throttle (whichever is lower)
        // This ensures flyforward activates at reasonable movement speeds
        bool throttle_over_thresh = g2.motors.get_throttle() > MIN(g.throttle_cruise * 0.50f, 15.0f);
        
        // Alternative activation: speed controller active with desired speed > 0.5 m/s
        // This handles autonomous modes where throttle might vary but speed target is set
        bool desired_speed_over_thresh = g2.attitude_control.speed_control_active() && (g2.attitude_control.get_desired_speed() > 0.5f);
        
        if (throttle_over_thresh || (is_positive(g2.motors.get_throttle()) && desired_speed_over_thresh)) {
            uint32_t now = AP_HAL::millis();
            
            // Start timer on first detection of forward movement conditions
            if (flyforward_start_ms == 0) {
                flyforward_start_ms = now;
            }
            
            // Enable flyforward only after 2 seconds of continuous forward movement
            // This hysteresis prevents rapid switching during throttle transients
            flyforward = (now - flyforward_start_ms > 2000);
        } else {
            // Reset timer when not moving forward - flyforward will disable immediately
            flyforward_start_ms = 0;
        }
    }

    // Update AHRS with current flyforward state
    ahrs.set_fly_forward(flyforward);
}

/**
 * @brief Check if a flight mode can be entered via Ground Control Station (GCS) command
 * 
 * @details This function checks the FLTMODE_GCSBLOCK parameter bitmask to determine if
 *          a specific flight mode is blocked from GCS selection. This safety feature allows
 *          operators to prevent certain modes from being accidentally selected via telemetry,
 *          while still allowing mode changes via RC transmitter or other mechanisms.
 *          
 *          Use cases:
 *          - Prevent untrained GCS operators from entering advanced modes (AUTO, GUIDED)
 *          - Block manual modes when autonomous-only operation is required
 *          - Enhance safety by limiting GCS mode selection to approved modes
 * 
 * @param[in] mode_num The flight mode number to check
 * 
 * @return true if mode can be entered from GCS, false if blocked by FLTMODE_GCSBLOCK parameter
 * 
 * @note Mode changes via RC transmitter are not affected by this check
 * @note Only the modes listed in mode_list can be blocked via parameter
 * @note Blocking a mode via GCS doesn't prevent automatic mode changes (e.g., RTL failsafe)
 * 
 * @see set_mode() for mode switching logic that calls this function
 */
bool Rover::gcs_mode_enabled(const Mode::Number mode_num) const
{
    // List of modes that can be blocked via FLTMODE_GCSBLOCK parameter
    // Each mode's position in this array corresponds to a bit in the parameter bitmask
    // Index is bit number in parameter bitmask
    static const uint8_t mode_list [] {
        (uint8_t)Mode::Number::MANUAL,
        (uint8_t)Mode::Number::ACRO,
        (uint8_t)Mode::Number::STEERING,
        (uint8_t)Mode::Number::LOITER,
        (uint8_t)Mode::Number::FOLLOW,
        (uint8_t)Mode::Number::SIMPLE,
        (uint8_t)Mode::Number::CIRCLE,
        (uint8_t)Mode::Number::AUTO,
        (uint8_t)Mode::Number::RTL,
        (uint8_t)Mode::Number::SMART_RTL,
        (uint8_t)Mode::Number::GUIDED,
#if MODE_DOCK_ENABLED
        (uint8_t)Mode::Number::DOCK
#endif
    };

    // Return true if mode is NOT blocked (inverse logic)
    return !block_GCS_mode_change((uint8_t)mode_num, mode_list, ARRAY_SIZE(mode_list));
}

/**
 * @brief Switch vehicle to a new flight mode with validation and safety checks
 * 
 * @details This is the primary mode switching function that handles all flight mode changes.
 *          It performs comprehensive validation before switching and handles all necessary
 *          cleanup and initialization during the transition.
 *          
 *          Mode switching sequence:
 *          1. Check if already in requested mode (no-op if true)
 *          2. Validate GCS mode blocking if change requested via GCS
 *          3. Call new mode's enter() method to initialize mode-specific state
 *          4. Update control_mode pointer to new mode
 *          5. Start fence manual recovery if fence breach is active
 *          6. Update camera auto-trigger state for AUTO mode
 *          7. Call old mode's exit() method to clean up previous mode
 *          8. Log mode change event
 *          9. Send MAVLink heartbeat with new mode
 *          10. Update notification system (LEDs, OSD) with new mode
 *          
 *          Mode change can be triggered by:
 *          - RC transmitter mode switch
 *          - GCS MAVLink command
 *          - Failsafe conditions (battery, GPS loss, geofence breach, etc.)
 *          - Mission commands
 *          - Autonomous transitions (e.g., mission complete → RTL)
 * 
 * @param[in] new_mode Reference to the mode object to switch to
 * @param[in] reason Reason for mode change (GCS command, RC switch, failsafe, etc.)
 * 
 * @return true if mode change successful, false if mode change denied or failed
 * 
 * @note Mode enter() method may fail if mode-specific preconditions not met
 * @warning Failed mode changes leave vehicle in previous mode - ensure fallback logic exists
 * @warning GCS-commanded mode changes can be blocked via FLTMODE_GCSBLOCK parameter
 * @warning Mode changes during fence breach automatically start manual recovery timer
 * 
 * @see Mode::enter() for mode-specific initialization
 * @see Mode::exit() for mode-specific cleanup
 * @see gcs_mode_enabled() for GCS mode blocking check
 */
bool Rover::set_mode(Mode &new_mode, ModeReason reason)
{
    // Early return if already in requested mode - no action needed
    if (control_mode == &new_mode) {
        return true;
    }

    // Safety check: Validate GCS mode changes against FLTMODE_GCSBLOCK parameter
    // This prevents unauthorized or accidental mode changes via telemetry
    if ((reason == ModeReason::GCS_COMMAND) && !gcs_mode_enabled((Mode::Number)new_mode.mode_number())) {
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE,"Mode change to %s denied, GCS entry disabled (FLTMODE_GCSBLOCK)", new_mode.name4());
        return false;
    }

    Mode &old_mode = *control_mode;
    
    // Attempt to enter new mode - mode may reject entry if preconditions not met
    // Each mode's enter() method performs mode-specific initialization and validation
    if (!new_mode.enter()) {
        // Log mode change failure for post-flight analysis
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FLIGHT_MODE,
                           LogErrorCode(new_mode.mode_number()));
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Flight mode change failed");
        return false;
    }

    // Mode change successful - update global mode pointer
    control_mode = &new_mode;

#if AP_FENCE_ENABLED
    // If pilot changes mode during fence breach, assume they are attempting manual recovery
    // Temporarily disable fence to allow pilot to maneuver vehicle back to safe area
    // This applies to any mode change (pilot, automatic, failsafe) during active breach
    fence.manual_recovery_start();
#endif

#if AP_CAMERA_ENABLED
    // Update camera auto-trigger state - only active in AUTO mode for mission-based triggering
    camera.set_is_auto_mode(control_mode->mode_number() == Mode::Number::AUTO);
#endif

    // Clean up old mode - release resources, stop mode-specific tasks
    old_mode.exit();

    // Record reason for mode change for telemetry and logging
    control_mode_reason = reason;
    
#if HAL_LOGGING_ENABLED
    // Log mode change event with timestamp, mode number, and reason
    logger.Write_Mode((uint8_t)control_mode->mode_number(), control_mode_reason);
#endif

    // Send MAVLink heartbeat immediately to inform GCS of mode change
    gcs().send_message(MSG_HEARTBEAT);

    // Update notification system (LEDs, buzzers, OSD) to display new mode
    notify_mode(control_mode);
    
    return true;
}

/**
 * @brief Mode switching wrapper for uint8_t mode number (MAVLink compatibility)
 * 
 * @details Convenience overload that accepts mode number as uint8_t for MAVLink compatibility.
 *          Casts to Mode::Number enum and calls primary set_mode() implementation.
 * 
 * @param[in] new_mode Mode number as uint8_t (MAVLink mode number)
 * @param[in] reason Reason for mode change
 * 
 * @return true if mode change successful, false otherwise
 * 
 * @see set_mode(Mode::Number, ModeReason) for implementation
 */
bool Rover::set_mode(const uint8_t new_mode, ModeReason reason)
{
    static_assert(sizeof(Mode::Number) == sizeof(new_mode), "The new mode can't be mapped to the vehicles mode number");
    return rover.set_mode(static_cast<Mode::Number>(new_mode), reason);
}

/**
 * @brief Mode switching wrapper that looks up mode object from mode number
 * 
 * @details This overload accepts a Mode::Number enum and looks up the corresponding
 *          mode object before calling the primary set_mode() implementation.
 *          Handles invalid mode numbers gracefully by notifying user.
 * 
 * @param[in] new_mode Mode number as Mode::Number enum
 * @param[in] reason Reason for mode change
 * 
 * @return true if mode change successful, false if mode invalid or change failed
 * 
 * @note Invalid mode numbers result in notification and return false
 * 
 * @see set_mode(Mode&, ModeReason) for primary implementation
 * @see mode_from_mode_num() for mode object lookup
 */
bool Rover::set_mode(Mode::Number new_mode, ModeReason reason)
{
    // Look up mode object from mode number
    Mode *mode = rover.mode_from_mode_num(new_mode);
    
    // Handle invalid mode number - mode doesn't exist for this vehicle configuration
    if (mode == nullptr) {
        notify_no_such_mode((uint8_t)new_mode);
        return false;
    }
    
    // Call primary set_mode implementation with mode object reference
    return rover.set_mode(*mode, reason);
}

/**
 * @brief Initialize and calibrate the Inertial Navigation System (INS)
 * 
 * @details Performs critical startup calibration of the AHRS (Attitude and Heading Reference System)
 *          and INS (Inertial Navigation System). This function calibrates the gyroscopes and
 *          accelerometers to establish accurate attitude and position estimation.
 *          
 *          Initialization sequence:
 *          1. Display warning message to GCS that vehicle must remain stationary
 *          2. Delay 100ms to allow message transmission
 *          3. Initialize AHRS subsystem
 *          4. Configure AHRS for ground vehicle characteristics (forward movement assumption)
 *          5. Set vehicle class to GROUND for appropriate EKF tuning
 *          6. Initialize INS at configured loop rate
 *          7. Reset AHRS to clear any previous state
 *          
 *          AHRS configuration:
 *          - set_fly_forward(true): Enables use of GPS velocity for heading estimation
 *          - VehicleClass::GROUND: Optimizes EKF for ground vehicle dynamics (no vertical acceleration except gravity)
 * 
 * @warning CRITICAL: Vehicle MUST be completely stationary during this calibration
 * @warning Movement during INS calibration will result in incorrect attitude/position estimation
 * @warning Incorrect INS calibration can lead to vehicle instability and navigation errors
 * 
 * @note Called from init_ardupilot() during vehicle startup
 * @note Gyro bias calibration occurs during this initialization
 * @note Takes approximately 3-5 seconds to complete
 * 
 * @see init_ardupilot() for full initialization sequence
 * @see update_ahrs_flyforward() for dynamic flyforward adjustment
 */
void Rover::startup_INS(void)
{
    // Warn operator via GCS that vehicle must be stationary
    // Movement during calibration will corrupt gyro/accel bias estimates
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Beginning INS calibration. Do not move vehicle");
    hal.scheduler->delay(100);

    // Initialize AHRS (Attitude and Heading Reference System)
    ahrs.init();
    
    // Configure AHRS for ground vehicle operation
    // Rovers typically only move forward, which helps GPS-based heading estimation
    ahrs.set_fly_forward(true);
    
    // Set vehicle class to GROUND for appropriate EKF parameter tuning
    // Ground vehicles have different dynamics than aircraft (no aerodynamic forces)
    ahrs.set_vehicle_class(AP_AHRS::VehicleClass::GROUND);

    // Initialize INS at the configured scheduler loop rate (typically 50Hz for Rover)
    ins.init(scheduler.get_loop_rate_hz());
    
    // Reset AHRS to clear any residual state from previous power cycle
    ahrs.reset();
}

/**
 * @brief Update notification system with current flight mode
 * 
 * @details Updates the AP_Notify system (LEDs, buzzers, OSD) to display the current
 *          flight mode. This provides visual and audible feedback to the operator about
 *          the vehicle's current state.
 *          
 *          Updates three notification elements:
 *          1. autopilot_mode flag - indicates if mode is autonomous or manual
 *          2. flight_mode number - numeric mode identifier for display
 *          3. flight_mode string - 4-character mode name for OSD/display
 * 
 * @param[in] mode Pointer to current mode object
 * 
 * @note Called after every successful mode change
 * @note LED patterns and buzzer tones vary by mode and autopilot_mode flag
 * 
 * @see set_mode() for mode switching that calls this function
 */
void Rover::notify_mode(const Mode *mode)
{
    // Set autopilot_mode flag to distinguish autonomous vs manual modes
    // Affects LED patterns and notification behavior
    AP_Notify::flags.autopilot_mode = mode->is_autopilot_mode();
    
    // Update numeric mode identifier for display systems
    notify.flags.flight_mode = (uint8_t)mode->mode_number();
    
    // Update 4-character mode name string for OSD and text displays
    notify.set_flight_mode_str(mode->name4());
}

/**
 * @brief Read digital input pin state with pullup enabled
 * 
 * @details Configures a GPIO pin as input with internal pullup resistor enabled,
 *          then reads and returns the pin state. Pullup ensures defined logic level
 *          when pin is not driven externally.
 *          
 *          Use cases:
 *          - Reading switch states
 *          - Detecting jumper configurations
 *          - Hardware configuration detection
 * 
 * @param[in] pin GPIO pin number to read
 * 
 * @return 1 if pin is high (pullup or externally driven high), 0 if driven low
 * 
 * @note Pin is configured as input with pullup - external drivers must sink current to pull low
 * @note Returns high (1) by default due to pullup resistor
 */
uint8_t Rover::check_digital_pin(uint8_t pin)
{
    // Configure pin as input to read external state
    hal.gpio->pinMode(pin, HAL_GPIO_INPUT);

    // Enable internal pullup resistor to provide default high state
    // Pin will read high unless externally pulled low
    hal.gpio->write(pin, 1);

    // Read and return current pin state (0 = low, 1 = high)
    return hal.gpio->read(pin);
}

#if HAL_LOGGING_ENABLED
/**
 * @brief Check if a specific log message type should be logged now
 * 
 * @details Wrapper function that queries the logger to determine if a message type
 *          matching the provided bitmask should be logged based on current LOG_BITMASK
 *          parameter settings and logging system state.
 *          
 *          Common log mask bits:
 *          - MASK_LOG_ATTITUDE_FAST - High-rate attitude logging
 *          - MASK_LOG_GPS - GPS position/velocity
 *          - MASK_LOG_IMU - Inertial sensor data
 *          - MASK_LOG_RCIN - RC input channels
 *          - MASK_LOG_RCOUT - Servo/motor outputs
 *          - MASK_LOG_COMPASS - Magnetometer data
 * 
 * @param[in] mask Bitmask of log message type(s) to check
 * 
 * @return true if message type should be logged, false if disabled or logging inactive
 * 
 * @note Only available when HAL_LOGGING_ENABLED is defined
 * @note Used throughout codebase to conditionally log data based on user configuration
 * 
 * @see AP_Logger::should_log() for implementation
 */
bool Rover::should_log(uint32_t mask)
{
    return logger.should_log(mask);
}
#endif

/**
 * @brief Determine if vehicle is configured as a boat
 * 
 * @details Checks the FRAME_CLASS parameter to identify if vehicle is configured as a boat.
 *          This affects multiple vehicle behaviors:
 *          
 *          Boat-specific behaviors:
 *          - Disables flyforward in AHRS (boats can drift laterally, making GPS velocity unreliable for heading)
 *          - Changes mission completion behavior to LOITER (boats drift if stopped, need active position hold)
 *          - Affects control tuning and navigation algorithms
 *          - May enable sailboat-specific features
 *          
 *          Frame classes:
 *          - FRAME_ROVER - Standard ground vehicle
 *          - FRAME_BOAT - Surface watercraft
 *          - FRAME_BALANCEBOT - Self-balancing robot
 *          - FRAME_SAILBOAT - Wind-powered sailing vessel
 * 
 * @return true if FRAME_CLASS parameter is set to FRAME_BOAT, false otherwise
 * 
 * @note Boat configuration disables movement-based heading estimation due to lateral drift
 * @note Affects default mission completion behavior (LOITER vs HOLD)
 * 
 * @see update_ahrs_flyforward() which uses this to disable flyforward for boats
 * @see init_ardupilot() which sets boat-specific mission completion behavior
 * @see FRAME_CLASS parameter for vehicle type configuration
 */
bool Rover::is_boat() const
{
    // Check if FRAME_CLASS parameter is set to FRAME_BOAT
    return ((enum frame_class)g2.frame_class.get() == FRAME_BOAT);
}

#include <AP_Avoidance/AP_Avoidance.h>
#if AP_ADSB_AVOIDANCE_ENABLED
// dummy method to avoid linking AP_Avoidance
AP_Avoidance *AP::ap_avoidance() { return nullptr; }
#endif  // AP_ADSB_AVOIDANCE_ENABLED
