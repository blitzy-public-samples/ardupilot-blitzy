/**
 * @file system.cpp
 * @brief Blimp system initialization and state management functions
 * 
 * @details This file contains the core system initialization and state management
 *          functions for the ArduPilot Blimp vehicle. It handles:
 *          - Vehicle initialization sequence (init_ardupilot)
 *          - Sensor and subsystem setup in proper dependency order
 *          - INS/AHRS ground startup calibration
 *          - Position and altitude validity checking via EKF state
 *          - Auto-armed state management for flight modes
 *          - Motor/fin allocation for blimp control
 *          - MAVLink vehicle type identification
 * 
 *          The initialization sequence is critical for proper vehicle operation
 *          and follows a specific order to satisfy hardware and software dependencies.
 *          This file supports both ground starts and in-air restarts.
 * 
 * @note Blimp is a lighter-than-air vehicle using fin control similar to underwater vehicles
 * @see Blimp.h for main vehicle class definition
 * 
 * Source: Blimp/system.cpp
 */

#include "Blimp.h"

/**
 * @brief Static wrapper function for scheduler-based failsafe checking
 * 
 * @details This function provides a static entry point for the HAL scheduler's
 *          timer-based failsafe callback mechanism. It wraps the Blimp instance
 *          method failsafe_check() to enable registration with the scheduler.
 *          
 *          The failsafe check verifies that the main loop is executing properly
 *          and triggers safety actions if the main loop appears to be stalled.
 * 
 * @note Called at 1Hz by the scheduler failsafe timer (registered in init_ardupilot)
 * @warning This must execute quickly as it runs in interrupt context
 * 
 * @see Blimp::init_ardupilot() where this callback is registered
 * @see Blimp::failsafe_check() for the actual failsafe logic
 */
static void failsafe_check_static()
{
    blimp.failsafe_check();
}

/**
 * @brief Initialize all ArduPilot subsystems for blimp vehicle operation
 * 
 * @details This is the main initialization function called during vehicle startup.
 *          It initializes all vehicle subsystems in the correct dependency order:
 *          
 *          Initialization Sequence:
 *          1. Notification system (LEDs, buzzers)
 *          2. Battery monitoring
 *          3. RSSI (signal strength monitoring)
 *          4. Barometer
 *          5. Ground Control Station (GCS) telemetry
 *          6. RC input channels
 *          7. Motor/fin allocation and initialization
 *          8. Loiter controller
 *          9. RC output configuration
 *          10. Relay outputs (if enabled)
 *          11. Failsafe timer registration
 *          12. GPS initialization
 *          13. Compass initialization
 *          14. Barometer calibration
 *          15. Logger initialization
 *          16. INS/AHRS ground startup and calibration
 *          17. Fin output setup
 *          18. Motor output enable (if RC calibrated)
 *          19. Velocity filter initialization (XY, Z, yaw)
 *          20. Initial flight mode selection
 *          
 *          The function supports both ground starts and in-air restarts. Ground vs
 *          in-air determination happens later in the initialization sequence.
 * 
 * @note This function is called once during vehicle boot
 * @note Dependencies between subsystems require this specific initialization order
 * @warning Changing the initialization order may cause failures or unsafe operation
 * 
 * @see startup_INS_ground() for INS/AHRS calibration details
 * @see allocate_motors() for motor/fin object creation
 */
void Blimp::init_ardupilot()
{
    // initialise notify system (LEDs, buzzers, displays)
    notify.init();
    notify_flight_mode();

    // initialise battery voltage and current monitoring
    battery.init();

#if AP_RSSI_ENABLED
    // Init RSSI (Received Signal Strength Indicator) for RC link quality monitoring
    rssi.init();
#endif

    // Initialize barometer for altitude sensing
    barometer.init();

    // setup telemetry serial port connections to ground control stations
    gcs().setup_uarts();

    // sets up RC input channels from radio receiver
    init_rc_in();

    // allocate the motor/fin controller object based on frame class
    allocate_motors();
    // allocate loiter position controller with scheduler loop rate
    loiter = NEW_NOTHROW Loiter(blimp.scheduler.get_loop_rate_hz());

    // initialise RC channels including arming/disarming function mapping
    rc().convert_options(RC_Channel::AUX_FUNC::ARMDISARM_UNUSED, RC_Channel::AUX_FUNC::ARMDISARM);
    rc().init();

    // configure motor/fin outputs to ESCs and servos
    init_rc_out();

    // motors initialised so parameters can now be sent to ground station
    ap.initialised_params = true;

#if AP_RELAY_ENABLED
    // Initialize relay outputs for controlling external devices
    relay.init();
#endif

    /*
     * Setup the 'main loop is dead' watchdog check. This 1Hz timer callback
     * verifies the main loop is executing and triggers failsafe if stalled.
     * Note that this relies on the RC library being initialised first.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

    // Initialize GPS receiver and configure logging
    gps.set_log_gps_bit(MASK_LOG_GPS);
    gps.init();

    // Initialize compass (magnetometer) and configure logging
    AP::compass().set_log_bit(MASK_LOG_COMPASS);
    AP::compass().init();

    // Calibrate barometer using ground pressure as reference
    // This establishes the zero altitude datum for relative altitude
    barometer.set_log_baro_bit(MASK_LOG_IMU);
    barometer.calibrate();

#if HAL_LOGGING_ENABLED
    // Initialize dataflash logger and register vehicle-specific startup messages
    logger.setVehicle_Startup_Writer(FUNCTOR_BIND(&blimp, &Blimp::Log_Write_Vehicle_Startup_Messages, void));
#endif

    // Perform INS/AHRS ground initialization and gyro calibration
    startup_INS_ground();

    // Configure IMU raw data logging
    ins.set_log_raw_bit(MASK_LOG_IMU_RAW);

    // Configure fin control outputs (blimp uses fins similar to underwater vehicles)
    motors->setup_fins();

    // Enable motor/fin output if RC radio has been calibrated
    // This safety check prevents outputting to uncalibrated servos/ESCs
    if (arming.rc_calibration_checks(true)) {
        enable_motor_output();
    }

    // Initialize velocity command filters for smooth fin control
    // Filters prevent sudden velocity changes that could destabilize the blimp
    // Parameters: scheduler_rate_hz, motor_rate_hz, cutoff_freq_hz, max_freq_hz
    vel_xy_filter.init(scheduler.get_loop_rate_hz(), motors->freq_hz, 0.5f, 15.0f);  // XY plane velocity
    vel_z_filter.init(scheduler.get_loop_rate_hz(), motors->freq_hz, 1.0f, 15.0f);   // Vertical velocity
    vel_yaw_filter.init(scheduler.get_loop_rate_hz(),motors->freq_hz, 5.0f, 15.0f);  // Yaw rate

    // Attempt to enter the user-configured initial flight mode
    // If initial mode is unavailable, fall back to MANUAL mode for safety
    if (!set_mode((enum Mode::Number)g.initial_mode.get(), ModeReason::INITIALISED)) {
        // MANUAL mode ensures pilot has direct control if preferred mode unavailable
        set_mode(Mode::Number::MANUAL, ModeReason::UNAVAILABLE);
    } else {
        // Notify pilot of successful mode entry via LEDs/buzzer
        AP_Notify::events.failsafe_mode_change = 1;
    }

    // Set initialization complete flag - vehicle is ready for operation
    ap.initialised = true;
}


/**
 * @brief Perform INS/AHRS ground initialization and sensor calibration
 * 
 * @details This function performs all sensor calibrations and initialization
 *          required during a ground start. The initialization sequence:
 *          
 *          1. Initialize AHRS (Attitude Heading Reference System)
 *             - May push IMU calibration into hardware (e.g., MPU6000 DMP)
 *             - Configure AHRS for copter-class vehicle dynamics
 *          2. Initialize and calibrate INS (Inertial Navigation System)
 *             - Warm up gyroscopes
 *             - Calibrate gyro bias offsets
 *             - Configure for scheduler loop rate
 *          3. Reset AHRS state with calibrated gyro bias
 * 
 * @note This function assumes the vehicle is stationary on the ground
 * @note Gyro calibration requires vehicle to remain still during process
 * @warning Moving the vehicle during this function will corrupt gyro calibration
 * 
 * @see init_ardupilot() which calls this function during initialization
 */
void Blimp::startup_INS_ground()
{
    // Initialize AHRS (may push IMU calibration into the MPU6000 if using that device)
    ahrs.init();
    // Configure AHRS to use copter-class vehicle dynamics model
    // Blimp uses copter dynamics despite being lighter-than-air
    ahrs.set_vehicle_class(AP_AHRS::VehicleClass::COPTER);

    // Warm up gyroscopes and calibrate gyro bias offsets
    // Vehicle must remain stationary during this process
    ins.init(scheduler.get_loop_rate_hz());

    // Reset AHRS state including the newly calibrated gyro bias
    ahrs.reset();
}

/**
 * @brief Check if horizontal position estimate is valid for navigation
 * 
 * @details Determines whether the vehicle has a valid horizontal position estimate
 *          that can be used for position-based flight modes. The position is
 *          considered valid if:
 *          - EKF failsafe has not triggered, AND
 *          - Either absolute position (GPS) or relative position (optical flow/visual odometry) is available
 * 
 * @return true if horizontal position is valid for navigation
 * @return false if position invalid or EKF failsafe active
 * 
 * @note This check is used to determine if position-based modes (Loiter, Auto) can be used
 * @see ekf_has_absolute_position() for GPS-based position check
 * @see ekf_has_relative_position() for relative position check
 */
bool Blimp::position_ok() const
{
    // Return false immediately if EKF failsafe has triggered
    // EKF failsafe indicates critical navigation failure
    if (failsafe.ekf) {
        return false;
    }

    // Position is valid if we have either absolute (GPS) or relative position
    return (ekf_has_absolute_position() || ekf_has_relative_position());
}

/**
 * @brief Check if EKF has valid absolute WGS-84 position estimate
 * 
 * @details Determines if the Extended Kalman Filter can provide an absolute
 *          global position estimate (typically from GPS). The requirements differ
 *          based on armed state:
 *          
 *          When disarmed:
 *          - Accepts actual absolute position (HORIZ_POS_ABS)
 *          - Accepts predicted absolute position (PRED_HORIZ_POS_ABS)
 *          
 *          When armed:
 *          - Requires actual absolute position (HORIZ_POS_ABS)
 *          - Rejects constant position mode (CONST_POS_MODE indicates dead reckoning)
 * 
 * @return true if valid absolute position available
 * @return false if no absolute position or DCM fallback active
 * 
 * @note More strict requirements when armed for flight safety
 * @note DCM (Direct Cosine Matrix) fallback does not provide position estimation
 * 
 * @see position_ok() which uses this to validate navigation capability
 */
bool Blimp::ekf_has_absolute_position() const
{
    // EKF must be active; DCM fallback does not provide position estimation
    if (!ahrs.have_inertial_nav()) {
        return false;
    }

    // When disarmed, accept either actual or predicted horizontal absolute position
    // Less strict requirements since vehicle is not in flight
    if (!motors->armed()) {
        if (ahrs.has_status(AP_AHRS::Status::HORIZ_POS_ABS)) {
            return true;
        }
        if (ahrs.has_status(AP_AHRS::Status::PRED_HORIZ_POS_ABS)) {
            return true;
        }
        return false;
    }
    
    // When armed, require high-quality position estimate
    // Reject constant position mode (dead reckoning without updates)
    if (ahrs.has_status(AP_AHRS::Status::CONST_POS_MODE)) {
        return false;
     }
    return ahrs.has_status(AP_AHRS::Status::HORIZ_POS_ABS);
}

/**
 * @brief Check if EKF has valid relative position estimate
 * 
 * @details Determines if the Extended Kalman Filter can provide a position
 *          estimate relative to its starting location. This is typically used
 *          when GPS is unavailable but optical flow or visual odometry provides
 *          relative motion sensing.
 *          
 *          Requirements:
 *          - EKF must be active (not DCM fallback)
 *          - Optical flow or visual odometry must be enabled
 *          
 *          When disarmed:
 *          - Accepts predicted relative position (PRED_HORIZ_POS_REL)
 *          
 *          When armed:
 *          - Requires actual relative position (HORIZ_POS_REL)
 *          - Rejects constant position mode (dead reckoning)
 * 
 * @return true if valid relative position available
 * @return false if no relative position sensors or estimate unavailable
 * 
 * @note Currently returns false as optical flow/visual odometry not enabled in blimp
 * @todo Implement optical flow or visual odometry support for GPS-denied operation
 * 
 * @see position_ok() which checks both absolute and relative position
 */
bool Blimp::ekf_has_relative_position() const
{
    // Return immediately if EKF not active (DCM does not provide position)
    if (!ahrs.have_inertial_nav()) {
        return false;
    }

    // Check if optical flow or visual odometry is enabled
    // Currently hardcoded to false - no relative position sensors configured
    bool enabled = false;
    if (!enabled) {
        return false;
    }

    // When disarmed, accept predicted horizontal relative position
    if (!motors->armed()) {
        return ahrs.has_status(AP_AHRS::Status::PRED_HORIZ_POS_REL);
    }
    
    // When armed, reject constant position mode (dead reckoning)
    if (ahrs.has_status(AP_AHRS::Status::CONST_POS_MODE)) {
        return false;
    }
    return ahrs.has_status(AP_AHRS::Status::HORIZ_POS_REL);
}

/**
 * @brief Check if EKF has valid altitude estimate for altitude control
 * 
 * @details Determines if the Extended Kalman Filter can provide a reliable
 *          altitude estimate suitable for altitude hold flight modes. This
 *          requires both vertical position and velocity estimates to be valid.
 *          
 *          Requirements for valid altitude:
 *          - EKF must be active (not DCM fallback)
 *          - Vertical velocity estimate available (VERT_VEL)
 *          - Vertical position estimate available (VERT_POS)
 * 
 * @return true if altitude estimate valid for altitude hold modes
 * @return false if altitude estimate unavailable or unreliable
 * 
 * @note Required for any flight mode that performs altitude hold
 * @note Altitude typically derived from barometer and GPS
 * @warning Altitude control should not be attempted if this returns false
 * 
 * @see Mode::requires_GPS() for modes requiring altitude control
 */
bool Blimp::ekf_alt_ok() const
{
    // EKF must be active; DCM does not provide altitude estimation
    if (!ahrs.have_inertial_nav()) {
        return false;
    }

    // Require vertical velocity estimate for altitude rate control
    if (!ahrs.has_status(AP_AHRS::Status::VERT_VEL)) {
        return false;
    }
    
    // Require vertical position estimate for altitude hold
    if (!ahrs.has_status(AP_AHRS::Status::VERT_POS)) {
        return false;
    }

    return true;
}

/**
 * @brief Update auto-armed state based on current flight conditions
 * 
 * @details The auto-armed flag indicates that the vehicle was armed automatically
 *          (e.g., by a mission or auto mode) rather than by explicit pilot command.
 *          This function clears the auto-armed flag when:
 *          
 *          1. Motors become disarmed - auto-armed state is no longer relevant
 *          2. In manual mode with zero throttle and no radio failsafe - indicates
 *             pilot has taken control and reduced throttle
 * 
 * @note Called periodically from main loop to track arming state changes
 * @note Auto-armed flag is used to determine if automatic disarm is appropriate
 * 
 * @see set_auto_armed() to update the auto-armed state
 */
void Blimp::update_auto_armed()
{
    // Only process if currently in auto-armed state
    if (ap.auto_armed) {
        // Clear auto-armed if motors are disarmed
        // Once disarmed, auto-armed state is no longer meaningful
        if (!motors->armed()) {
            set_auto_armed(false);
            return;
        }
        
        // Clear auto-armed in manual mode when pilot reduces throttle to zero
        // Indicates pilot has taken manual control and landed/reduced thrust
        // Don't clear during radio failsafe to maintain auto-armed behavior
        if (flightmode->has_manual_throttle() && ap.throttle_zero && !failsafe.radio) {
            set_auto_armed(false);
        }
    }
}

#if HAL_LOGGING_ENABLED
/**
 * @brief Determine if a specific log message type should be logged
 * 
 * @details Checks whether a log message with the given type bitmask should
 *          be written to the dataflash log. This respects the LOG_BITMASK
 *          parameter configuration and logging system state.
 *          
 *          The function also updates the logging_started flag to track
 *          whether logging has been initiated.
 * 
 * @param[in] mask Log message type bitmask (e.g., MASK_LOG_IMU, MASK_LOG_GPS)
 * 
 * @return true if message type should be logged
 * @return false if message type disabled or logging not active
 * 
 * @note Message type masks defined in LogStructure.h
 * @see AP_Logger::should_log() for logging decision logic
 */
bool Blimp::should_log(uint32_t mask)
{
    // Update logging started status
    ap.logging_started = logger.logging_started();
    // Delegate to logger to check if this message type is enabled
    return logger.should_log(mask);
}
#endif

/**
 * @brief Get MAVLink vehicle type identifier for this frame
 * 
 * @details Returns the MAVLink MAV_TYPE enum value that identifies this
 *          vehicle type to ground control stations. Blimp vehicles are
 *          classified as MAV_TYPE_AIRSHIP in the MAVLink protocol.
 * 
 * @return MAV_TYPE_AIRSHIP indicating lighter-than-air vehicle
 * 
 * @note Used in MAVLink HEARTBEAT messages to identify vehicle type
 * @see MAVLink common.xml for MAV_TYPE definitions
 */
MAV_TYPE Blimp::get_frame_mav_type()
{
    return MAV_TYPE_AIRSHIP;
}

/**
 * @brief Get human-readable frame type string
 * 
 * @details Returns a string identifier for the blimp frame configuration.
 *          Currently returns "AIRFISH" for the default fin-based control frame.
 * 
 * @return Pointer to frame type string "AIRFISH"
 * 
 * @note Currently hardcoded to AIRFISH frame type
 * @todo Implement frame class selection to support multiple blimp configurations
 * @see allocate_motors() which creates frame-specific motor/fin objects
 */
const char* Blimp::get_frame_string()
{
    return "AIRFISH";  // Currently only AIRFISH frame supported
}

/**
 * @brief Allocate and initialize the motor/fin control object
 * 
 * @details Creates the appropriate Fins motor control object based on the
 *          configured frame class parameter. The process:
 *          
 *          1. Allocate Fins object based on FRAME_CLASS parameter
 *          2. Verify allocation succeeded (fail with error if out of memory)
 *          3. Load motor/fin parameters from EEPROM
 *          4. Reload parameter defaults (may be frame-specific)
 *          5. Invalidate parameter count (changed by new object)
 *          
 *          Currently only AIRFISH frame class is implemented.
 * 
 * @note Called during initialization before motors are used
 * @note Uses NEW_NOTHROW to handle out-of-memory gracefully
 * @warning Allocation failure is fatal and will prevent vehicle from flying
 * 
 * @see Fins class for motor/fin control implementation
 * @see init_ardupilot() which calls this during initialization
 */
void Blimp::allocate_motors(void)
{
    // Create motor/fin controller based on configured frame class
    switch ((Fins::motor_frame_class)g2.frame_class.get()) {
    case Fins::MOTOR_FRAME_AIRFISH:
    default:
        // Allocate Fins object with scheduler loop rate for control frequency
        motors = NEW_NOTHROW Fins(blimp.scheduler.get_loop_rate_hz());
        break;
    }
    
    // Verify allocation succeeded - fail with descriptive error if out of memory
    if (motors == nullptr) {
        AP_BoardConfig::allocation_error("FRAME_CLASS=%u", (unsigned)g2.frame_class.get());
    }
    
    // Load motor/fin parameters from EEPROM storage
    AP_Param::load_object_from_eeprom(motors, Fins::var_info);

    // Reload parameter default values that may now be accessible
    // Some defaults may be frame-class specific
    AP_Param::reload_defaults_file(true);

    // Parameter count has changed with new object - invalidate cached count
    AP_Param::invalidate_count();
}
