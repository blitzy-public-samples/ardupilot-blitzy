/**
 * @file system.cpp
 * @brief Copter system initialization and state management
 * 
 * @details This file contains the core system initialization sequences for ArduCopter,
 *          including hardware initialization, sensor setup, control allocation, and
 *          system health monitoring functions. The initialization process handles both
 *          ground starts and in-air restarts, configuring all subsystems required for
 *          safe flight operation.
 * 
 *          Key responsibilities:
 *          - Hardware subsystem initialization (sensors, motors, peripherals)
 *          - Control object allocation (motors, attitude control, position control)
 *          - Ground calibration and sensor fusion initialization
 *          - System state validation (position, altitude, navigation readiness)
 *          - Auto-armed state management for throttle safety
 * 
 * @note This code is safety-critical and executes during vehicle startup
 * @warning Modifications to initialization sequence can affect flight safety
 * 
 * @see Copter.h for main vehicle class definition
 * @see Copter::setup() in Copter.cpp for overall startup sequence
 */

#include "Copter.h"
#include <AP_ESC_Telem/AP_ESC_Telem.h>

/**
 * @brief Static wrapper for scheduler-based failsafe checking
 * 
 * @details This static function is registered with the HAL scheduler as a timer
 *          failsafe callback. It provides the required static interface while
 *          calling the non-static Copter::failsafe_check() method.
 * 
 *          The scheduler calls this function at 1Hz (1000ms interval) to monitor
 *          for main loop failures and trigger failsafe actions if the main loop
 *          becomes unresponsive.
 * 
 * @note Called at 1Hz by HAL scheduler timer failsafe mechanism
 * @warning Critical for detecting main loop hang conditions
 * 
 * @see Copter::init_ardupilot() where this callback is registered
 * @see Copter::failsafe_check() for actual failsafe logic
 */
static void failsafe_check_static()
{
    copter.failsafe_check();
}

/**
 * @brief Initialize all ArduCopter subsystems for flight operation
 * 
 * @details This is the main initialization function that sets up all vehicle subsystems
 *          required for flight. It handles both ground starts and in-air restarts,
 *          configuring hardware peripherals, sensors, control systems, and safety
 *          mechanisms in the correct sequence.
 * 
 *          Initialization sequence overview:
 *          1. Peripheral systems (winch, notify, battery, RSSI, barometer, telemetry)
 *          2. RC input and motor allocation
 *          3. Navigation sensors (GPS, compass, airspeed, optical flow)
 *          4. Peripherals (mount, camera, landing gear, rangefinder, proximity)
 *          5. Advanced features (mission, SmartRTL, logging, custom control)
 *          6. INS ground startup and calibration
 *          7. Initial flight mode selection
 *          8. Failsafe enablement
 * 
 *          This function is called from Copter::setup() during vehicle startup.
 *          After completion, the main loop (Copter::loop()) begins execution.
 * 
 * @note This function takes several seconds to complete due to sensor calibration
 * @note Barometer calibration requires the vehicle to be stationary
 * @warning Interrupting this sequence can leave systems in partially initialized state
 * @warning Motor outputs are set to minimum before mode initialization for safety
 * 
 * @see Copter::setup() for pre-initialization tasks
 * @see Copter::startup_INS_ground() for INS/AHRS initialization
 * @see Copter::allocate_motors() for motor and control object creation
 */
void Copter::init_ardupilot()
{
    // Initialize winch subsystem for payload deployment/recovery
#if AP_WINCH_ENABLED
    g2.winch.init();
#endif

    // Initialize notification system (LEDs, buzzers, external displays)
    // Must be early in sequence to provide visual/audio feedback during initialization
    notify.init();
    notify_flight_mode();

    // Initialize battery monitoring system for voltage/current sensing
    // Critical for low battery failsafe and flight time estimation
    battery.init();

#if AP_RSSI_ENABLED
    // Initialize RSSI (Received Signal Strength Indicator) monitoring
    // Used for RC link quality monitoring and failsafe detection
    rssi.init();
#endif

    // Initialize barometer for altitude sensing
    // Required for altitude hold and position control modes
    barometer.init();

    // Setup ground control station telemetry on configured serial ports
    // Establishes MAVLink communication channels for mission planning and monitoring
    gcs().setup_uarts();

#if OSD_ENABLED
    // Initialize on-screen display for first-person view systems
    osd.init();
#endif

    // Update motor interlock state based on configuration
    // Motor interlock provides additional safety layer for traditional helicopters
    update_using_interlock();

#if FRAME_CONFIG == HELI_FRAME
    // Traditional helicopter specific initialization
    // Configures swashplate mixing and rotor speed control
    heli_init();
#endif
#if FRAME_CONFIG == HELI_FRAME
    // Set input manager loop rate for traditional helicopter
    // Must match scheduler rate for proper cyclic/collective control
    input_manager.set_loop_rate(scheduler.get_loop_rate_hz());
#endif

    // Initialize RC input channels from receiver
    // Configures PPM, SBUS, DSM, or other RC protocols
    init_rc_in();

#if AP_RANGEFINDER_ENABLED
    // Initialize surface tracking mode for terrain following
    // Must be before RC init to preserve switch position from parameters
    surface_tracking.init((SurfaceTracking::Surface)copter.g2.surftrak_mode.get());
#endif

    // Allocate motor mixer and control objects based on frame type
    // Creates AP_Motors, attitude_control, pos_control, wp_nav objects
    // This is a critical allocation step - see allocate_motors() for details
    allocate_motors();

    // Initialize RC channels and convert legacy aux function options
    // Converts old ARMDISARM_UNUSED to ARMDISARM_AIRMODE for safety improvement
    rc().convert_options(RC_Channel::AUX_FUNC::ARMDISARM_UNUSED, RC_Channel::AUX_FUNC::ARMDISARM_AIRMODE);
    rc().init();

    // Configure motor outputs and ESC communication
    // Sets up PWM/DShot/OneShot protocols and output pin assignments
    init_rc_out();

    // Check if ESC calibration mode should be entered
    // Allows pass-through of throttle to ESCs for manual calibration
    esc_calibration_startup_check();

    // Mark that parameters are initialized and can be sent via MAVLink
    // Ground stations can now query and modify parameters
    ap.initialised_params = true;

#if AP_RELAY_ENABLED
    // Initialize relay outputs for controlling external devices
    // Can be used for parachute, sprayer, gripper, or custom payloads
    relay.init();
#endif

    /*
     * Setup the 'main loop is dead' watchdog check
     * This registers a 1Hz timer callback that monitors main loop health
     * If the main loop hangs, failsafe actions are triggered
     * Note: This relies on RC library being initialized first
     */
    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

    // Initialize GPS subsystem for position estimation
    // Configures GPS drivers (uBlox, NMEA, etc.) and enables logging
    gps.set_log_gps_bit(MASK_LOG_GPS);
    gps.init();

    // Initialize compass/magnetometer for heading estimation
    // Critical for navigation modes and yaw control
    AP::compass().set_log_bit(MASK_LOG_COMPASS);
    AP::compass().init();

#if AP_AIRSPEED_ENABLED
    // Initialize airspeed sensor (primarily for transitioning QuadPlanes)
    // Enable logging for flight performance analysis
    airspeed.set_log_bit(MASK_LOG_IMU);
#endif

#if AP_OAPATHPLANNER_ENABLED
    // Initialize object avoidance path planner
    // Uses proximity sensors and rangefinders to avoid obstacles
    g2.oa.init();
#endif

    // Validate attitude control parameters are within safe ranges
    // Prevents unstable configurations from causing loss of control
    attitude_control->parameter_sanity_check();

#if AP_OPTICALFLOW_ENABLED
    // Initialize optical flow sensor for velocity estimation
    // Enables position hold without GPS (requires downward-facing camera)
    optflow.init(MASK_LOG_OPTFLOW);
#endif      // AP_OPTICALFLOW_ENABLED

#if HAL_MOUNT_ENABLED
    // Initialize camera gimbal mount control
    // Supports stabilized camera pointing and region-of-interest tracking
    camera_mount.init();
#endif

#if AP_CAMERA_ENABLED
    // Initialize camera trigger control for aerial photography
    // Supports intervalometer and mission-based triggering
    camera.init();
#endif

#if AC_PRECLAND_ENABLED
    // Initialize precision landing system
    // Uses IR-LOCK or other beacon systems for accurate landing
    init_precland();
#endif

#if AP_LANDINGGEAR_ENABLED
    // Initialize retractable landing gear control
    // Manages deployment/retraction during flight phases
    landinggear.init();
#endif

#ifdef USERHOOK_INIT
    // User-defined custom initialization hook
    // Allows custom code insertion without modifying core firmware
    USERHOOK_INIT
#endif

    // Calibrate barometer at ground level
    // This establishes the zero altitude reference for flight
    // Vehicle must be stationary during this calibration
    barometer.set_log_baro_bit(MASK_LOG_IMU);
    barometer.calibrate();

#if AP_RANGEFINDER_ENABLED
    // Initialize rangefinder/lidar sensors for terrain sensing
    // Used for precision landing, terrain following, and object detection
    init_rangefinder();
#endif

#if HAL_PROXIMITY_ENABLED
    // Initialize 360-degree proximity sensors for obstacle detection
    // Provides situational awareness for collision avoidance
    g2.proximity.init();
#endif

#if AP_BEACON_ENABLED
    // Initialize positioning beacons for non-GPS navigation
    // Enables indoor flight using external positioning systems
    g2.beacon.init();
#endif

#if AP_RPM_ENABLED
    // Initialize RPM sensor library for motor/rotor speed monitoring
    // Used for rotor speed governor on traditional helicopters
    rpm_sensor.init();
#endif

#if MODE_AUTO_ENABLED
    // Initialize mission management library for autonomous flight
    // Loads waypoints, commands, and mission execution logic
    mode_auto.mission.init();
#if HAL_LOGGING_ENABLED
    // Enable mission command logging for post-flight analysis
    mode_auto.mission.set_log_start_mission_item_bit(MASK_LOG_CMD);
#endif
#endif

#if MODE_SMARTRTL_ENABLED
    // Initialize SmartRTL path recording system
    // Records flight path for intelligent return-to-launch
    g2.smart_rtl.init();
#endif

#if HAL_LOGGING_ENABLED
    // Initialize binary logging system (dataflash/SD card)
    // Register callback to write vehicle-specific startup messages
    logger.setVehicle_Startup_Writer(FUNCTOR_BIND(&copter, &Copter::Log_Write_Vehicle_Startup_Messages, void));
#endif

    // Perform INS (Inertial Navigation System) ground startup
    // Calibrates gyros, initializes AHRS, and warms up IMU sensors
    // This is a critical step that requires the vehicle to be stationary
    startup_INS_ground();

#if AC_CUSTOMCONTROL_MULTI_ENABLED
    // Initialize custom control backend for advanced control algorithms
    // Allows scripting or external control law implementation
    custom_control.init();
#endif

    // Set initial landed state flags
    // Assumes vehicle is on ground at startup for safety
    set_land_complete(true);
    set_land_complete_maybe(true);

    // Enable CPU load monitoring and failsafe
    // Triggers failsafe if scheduler cannot maintain required loop rates
    failsafe_enable();

    // Enable raw IMU logging for detailed sensor analysis
    ins.set_log_raw_bit(MASK_LOG_IMU_RAW);

    // Set motor outputs to minimum for safety before arming
    // Ensures motors are not spinning during initialization
    motors->output_min();

    // Attempt to enter the configured initial flight mode
    // Falls back to STABILIZE if initial mode is unavailable
    if (!set_mode((enum Mode::Number)g.initial_mode.get(), ModeReason::INITIALISED)) {
        // STABILIZE mode is always available and safe for startup
        // Triggers mode change notification to pilot via telemetry
        set_mode(Mode::Number::STABILIZE, ModeReason::UNAVAILABLE);
    }

    // Configure EKF variance filters for failsafe detection
    // Low-pass filters prevent transient EKF errors from triggering failsafe
    pos_variance_filt.set_cutoff_frequency(g2.fs_ekf_filt_hz);
    vel_variance_filt.set_cutoff_frequency(g2.fs_ekf_filt_hz);

    // Mark initialization complete - main loop can begin normal operation
    // This flag is checked by various subsystems before allowing full functionality
    ap.initialised = true;
}


/**
 * @brief Perform ground startup calibration for Inertial Navigation System
 * 
 * @details This function performs all ground-based calibrations required for the
 *          INS (Inertial Navigation System) and AHRS (Attitude and Heading Reference
 *          System) before flight. The vehicle MUST be stationary during this process.
 * 
 *          Sequence:
 *          1. Initialize AHRS and set vehicle class to COPTER for appropriate filtering
 *          2. Initialize and warm up IMU sensors at configured loop rate
 *          3. Calibrate gyroscope zero offsets over several seconds
 *          4. Reset AHRS state estimator with calibrated gyro biases
 * 
 *          This calibration establishes the initial attitude reference and removes
 *          gyroscope bias, which is critical for stable flight. Temperature-related
 *          drift is minimized by allowing IMU warm-up time.
 * 
 * @note Vehicle must be completely stationary during this function
 * @note This function blocks for several seconds during gyro calibration
 * @note IMU temperature affects gyro bias - warm-up time improves accuracy
 * 
 * @warning Moving the vehicle during calibration will result in incorrect bias values
 * @warning Incorrect gyro calibration will cause attitude estimation drift
 * 
 * @see Copter::init_ardupilot() which calls this function during initialization
 * @see AP_AHRS::init() for AHRS initialization details
 * @see AP_InertialSensor::init() for IMU calibration algorithm
 */
void Copter::startup_INS_ground()
{
    // Initialize AHRS (Attitude and Heading Reference System)
    // May push IMU calibration parameters to sensor if supported (e.g., MPU6000)
    ahrs.init();
    // Set vehicle class to COPTER for appropriate EKF tuning and filtering
    ahrs.set_vehicle_class(AP_AHRS::VehicleClass::COPTER);

    // Warm up IMU sensors and calibrate gyroscope zero offsets
    // This blocks for several seconds while collecting gyro samples
    // Loop rate determines IMU sample rate (typically 400Hz for copters)
    ins.init(scheduler.get_loop_rate_hz());

    // Reset AHRS state estimator with newly calibrated gyro biases
    // Clears any pre-existing state and initializes attitude to level
    ahrs.reset();
}

/**
 * @brief Check if horizontal position estimate is valid for navigation
 * 
 * @details This function determines if the EKF (Extended Kalman Filter) position
 *          estimate is sufficiently accurate to enable position-dependent flight modes
 *          such as Loiter, PosHold, Auto, and Guided. It checks for EKF failsafe
 *          conditions and validates that either absolute (GPS-based) or relative
 *          (optical flow/visual odometry) position is available.
 * 
 *          Position is considered valid if:
 *          - EKF failsafe has not triggered
 *          - AND either absolute GPS position OR relative position is available
 * 
 * @return true if position estimate is valid for navigation modes
 * @return false if position is unreliable or EKF failsafe is active
 * 
 * @note This is checked by flight modes before allowing position control
 * @note Absolute position requires GPS fix with sufficient accuracy
 * @note Relative position requires optical flow or visual odometry
 * 
 * @see Copter::ekf_has_absolute_position() for GPS-based position check
 * @see Copter::ekf_has_relative_position() for relative position check
 * @see Mode::requires_GPS() for mode-specific position requirements
 */
bool Copter::position_ok() const
{
    // Return false immediately if EKF failsafe has triggered
    // EKF failsafe indicates position estimate has diverged beyond safe limits
    if (failsafe.ekf) {
        return false;
    }

    // Check if EKF can provide either absolute or relative position
    // Absolute = GPS-based WGS-84 position
    // Relative = Position relative to initialization point using optical flow or visual odometry
    return (ekf_has_absolute_position() || ekf_has_relative_position());
}

/**
 * @brief Check if EKF can provide absolute GPS-based position estimate
 * 
 * @details Determines if the EKF has a valid absolute position estimate in the
 *          WGS-84 (World Geodetic System) coordinate frame, typically from GPS.
 *          Requirements differ based on whether the vehicle is armed:
 * 
 *          When DISARMED:
 *          - Accepts current absolute position estimate
 *          - Also accepts predicted absolute position (short-term GPS loss)
 * 
 *          When ARMED:
 *          - Requires current (not predicted) absolute position
 *          - Must not be in constant position mode (EKF fallback when no position updates)
 * 
 * @return true if valid absolute position is available
 * @return false if position unavailable or accuracy insufficient
 * 
 * @note Absolute position requires GPS with sufficient satellite count and HDOP
 * @note DCM-based position is rejected (requires EKF for navigation)
 * @note Constant position mode indicates EKF is coasting without position updates
 * 
 * @warning More stringent checks when armed to prevent navigation with stale data
 * 
 * @see Copter::position_ok() which calls this function
 * @see AP_AHRS::Status for EKF status flags
 */
bool Copter::ekf_has_absolute_position() const
{
    // Reject if using DCM-based attitude estimation without EKF
    // DCM cannot provide reliable position estimates for navigation
    if (!ahrs.have_inertial_nav()) {
        return false;
    }

    // When disarmed, accept predicted position estimates
    // Allows brief GPS loss during pre-arm checks
    if (!motors->armed()) {
        // Accept current absolute position from GPS
        if (ahrs.has_status(AP_AHRS::Status::HORIZ_POS_ABS)) {
            return true;
        }
        // Accept predicted position (short-term dead reckoning)
        if (ahrs.has_status(AP_AHRS::Status::PRED_HORIZ_POS_ABS)) {
            return true;
        }
        return false;
    }

    // When armed, require actual (not predicted) absolute position
    // Constant position mode = EKF coasting without position updates
    if (ahrs.has_status(AP_AHRS::Status::CONST_POS_MODE)) {
        return false;
    }
    // Require current absolute position estimate from GPS
    return ahrs.has_status(AP_AHRS::Status::HORIZ_POS_ABS);
}

/**
 * @brief Check if EKF can provide position estimate relative to starting point
 * 
 * @details Determines if the EKF has a valid relative position estimate (position
 *          relative to the initialization point) using non-GPS sensors such as
 *          optical flow, visual odometry, or wheel encoders. This enables position
 *          control in GPS-denied environments.
 * 
 *          Requirements:
 *          - EKF must be active (not DCM-based estimation)
 *          - At least one relative positioning sensor must be enabled and active:
 *            * Optical flow sensor
 *            * Visual odometry system
 *            * Dead reckoning (wheel encoders, etc.)
 *          - Different standards when armed vs disarmed
 * 
 *          When DISARMED:
 *          - Accepts predicted relative position
 * 
 *          When ARMED:
 *          - Requires current (not predicted) relative position
 *          - Must not be in constant position mode
 * 
 * @return true if valid relative position is available
 * @return false if no relative positioning sensors or position invalid
 * 
 * @note Enables GPS-free flight modes like Loiter and PosHold
 * @note Relative position accuracy degrades over time without absolute reference
 * @note Dead reckoning must be active and not timed out
 * 
 * @see Copter::position_ok() which calls this function
 * @see AP_AHRS::Status for EKF status flags
 */
bool Copter::ekf_has_relative_position() const
{
    // Return immediately if EKF is not being used
    // DCM-based estimation cannot provide reliable relative position
    if (!ahrs.have_inertial_nav()) {
        return false;
    }

    // Check if any relative positioning sensor is enabled
    // Relative position requires at least one of: optical flow, visual odometry, or dead reckoning
    bool enabled = false;
#if AP_OPTICALFLOW_ENABLED
    // Optical flow sensor provides velocity/position from downward-facing camera
    if (optflow.enabled()) {
        enabled = true;
    }
#endif
#if HAL_VISUALODOM_ENABLED
    // Visual odometry (e.g., Intel T265) provides position from SLAM
    if (visual_odom.enabled()) {
        enabled = true;
    }
#endif
    // Dead reckoning from wheel encoders or similar sensors
    // Must be actively running and not timed out
    if (dead_reckoning.active && !dead_reckoning.timeout) {
        enabled = true;
    }
    // If no relative positioning sensors are enabled, return false
    if (!enabled) {
        return false;
    }

    // When disarmed, accept predicted relative position
    // Allows position control during pre-arm without strict requirements
    if (!motors->armed()) {
        return ahrs.has_status(AP_AHRS::Status::PRED_HORIZ_POS_REL);
    }

    // When armed, require actual (not predicted) relative position
    // Constant position mode indicates EKF is coasting without updates
    if (ahrs.has_status(AP_AHRS::Status::CONST_POS_MODE)) {
        return false;
    }
    // Require current relative position estimate from sensors
    if (!ahrs.has_status(AP_AHRS::Status::HORIZ_POS_REL)) {
        return false;
    }
    return true;
}

/**
 * @brief Check if EKF has valid altitude estimate for altitude control modes
 * 
 * @details Determines if the EKF vertical position and velocity estimates are
 *          sufficiently accurate to enable altitude-holding flight modes such as
 *          AltHold, Loiter, PosHold, Auto, and Guided.
 * 
 *          Requirements for valid altitude:
 *          - EKF must be active (not DCM-based estimation)
 *          - Vertical position estimate must be available
 *          - Vertical velocity estimate must be available
 * 
 *          Altitude is primarily derived from barometric pressure, but can be
 *          augmented by GPS altitude, rangefinder, and IMU vertical acceleration.
 * 
 * @return true if altitude estimate is valid for altitude control
 * @return false if altitude unreliable or unavailable
 * 
 * @note Barometer provides primary altitude reference
 * @note Both position and velocity are required for stable altitude hold
 * @note DCM-based attitude estimation cannot provide reliable altitude control
 * 
 * @see Mode::requires_GPS() for mode-specific altitude requirements
 * @see Copter::position_ok() for horizontal position validation
 */
bool Copter::ekf_alt_ok() const
{
    // Reject if using DCM-based attitude estimation without EKF
    // DCM alone cannot provide reliable altitude control
    if (!ahrs.have_inertial_nav()) {
        return false;
    }

    // Require valid vertical position estimate
    // Position needed for altitude error calculation
    if (!ahrs.has_status(AP_AHRS::Status::VERT_POS)) {
        return false;
    }
    // Require valid vertical velocity estimate
    // Velocity needed for altitude rate control and damping
    if (!ahrs.has_status(AP_AHRS::Status::VERT_VEL)) {
        return false;
    }
    return true;
}

/**
 * @brief Update auto-armed state for throttle safety monitoring
 * 
 * @details The auto_armed flag indicates whether the vehicle has received sufficient
 *          throttle input to be considered actively flying. This is used to:
 *          - Enable/disable certain safety features
 *          - Detect when vehicle is ready for autonomous control
 *          - Transition between ground and flight states
 * 
 *          AUTO-DISARM Logic (when currently auto-armed):
 *          - Motors disarmed → auto_armed becomes false
 *          - Manual throttle mode (Stabilize/Acro) + zero throttle + valid RC → auto_armed becomes false
 * 
 *          AUTO-ARM Logic (when not currently auto-armed):
 *          - Traditional Heli: Armed + throttle > 0 + motor spool state THROTTLE_UNLIMITED → auto_armed becomes true
 *          - Non-Heli: Armed + (throttle > 0 OR in THROW mode) → auto_armed becomes true
 * 
 *          The distinction between armed and auto_armed allows detection of:
 *          - Vehicle armed but sitting on ground with zero throttle (not auto_armed)
 *          - Vehicle armed and actively flying or ready to fly (auto_armed)
 * 
 * @note Called at main loop rate to continuously monitor throttle state
 * @note Traditional helicopters require motor spool-up before being considered auto_armed
 * @note THROW mode is auto_armed when armed even with zero throttle
 * 
 * @see Copter::set_auto_armed() to update the flag value
 * @see Mode::has_manual_throttle() to check if mode has manual throttle control
 */
void Copter::update_auto_armed()
{
    // AUTO-DISARM CHECKS
    // Check if we should transition from auto_armed to not auto_armed
    if(ap.auto_armed){
        // If motors become disarmed, auto_armed should also become false
        // Ensures auto_armed state is cleared on disarm
        if(!motors->armed()) {
            set_auto_armed(false);
            return;
        }
        // In manual throttle modes (Stabilize, Acro), if throttle is zero and RC valid,
        // assume pilot has landed and clear auto_armed
        // This allows safety features that depend on "not flying" state
        if (flightmode->has_manual_throttle() && ap.throttle_zero && rc().has_valid_input()) {
            set_auto_armed(false);
        }

    }else{
        // AUTO-ARM CHECKS
        // Check if we should transition from not auto_armed to auto_armed
        
        // Traditional helicopter requires motor interlock and full spool-up
        // Interlock safety prevents rotor engagement until explicitly enabled
        if(motors->armed() && ap.using_interlock) {
            // Auto-arm when throttle is non-zero and motor has reached full RPM
            // THROTTLE_UNLIMITED = rotor at flight speed and ready for load
            if(!ap.throttle_zero && motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
                set_auto_armed(true);
            }
        // Non-helicopter (multirotor) auto-arm logic
        } else if (motors->armed() && !ap.using_interlock) {
            // Auto-arm when throttle is raised above zero
            // OR when in THROW mode (which arms motors in the air)
            if(!ap.throttle_zero || flightmode->mode_number() == Mode::Number::THROW) {
                set_auto_armed(true);
            }
        }
    }
}

#if HAL_LOGGING_ENABLED
/**
 * @brief Determine if a specific log message type should be logged now
 * 
 * @details This function checks if a particular log message type is currently
 *          enabled based on the LOG_BITMASK parameter and logging system state.
 *          It provides a vehicle-specific wrapper around the AP_Logger decision logic.
 * 
 * @param[in] mask Log bitmask for the message type to check (e.g., MASK_LOG_GPS, MASK_LOG_IMU)
 * 
 * @return true if this message type should be logged
 * @return false if message type is disabled or logging unavailable
 * 
 * @note Used throughout codebase to conditionally log messages
 * @note Logging can be disabled by parameter or if storage is full
 * 
 * @see AP_Logger::should_log() for underlying implementation
 * @see LOG_BITMASK parameter for message type selection
 */
bool Copter::should_log(uint32_t mask)
{
    return logger.should_log(mask);
}
#endif

/**
 * @brief Allocate motor mixer and control objects based on frame configuration
 * 
 * @details This critical function creates and initializes the core control objects
 *          required for flight based on the configured frame type (quad, hex, heli,
 *          etc.). It performs the following allocations:
 * 
 *          1. Motor mixer object (AP_Motors variant for frame type)
 *          2. AHRS view for control system coordinate transforms
 *          3. Attitude controller (rate and angle control)
 *          4. Position controller (velocity and position control)
 *          5. Waypoint navigation controller
 *          6. Loiter navigation controller
 *          7. Circle navigation controller (if enabled)
 * 
 *          Frame-specific configurations:
 *          - Multirotor: AP_MotorsMatrix with frame-specific mixing
 *          - Tricopter: AP_MotorsTri with yaw servo
 *          - Traditional Heli: AP_MotorsHeli_Single/Dual/Quad with swashplate
 *          - Coaxial: AP_MotorsCoax
 *          - Tailsitter: AP_MotorsTailsitter
 *          - Scripting: AP_MotorsMatrix_6DoF_Scripting or _Scripting_Dynamic
 * 
 *          After allocation, this function:
 *          - Loads parameters from EEPROM for all objects
 *          - Sets frame-specific PID defaults
 *          - Handles PWM output rate for brushed motors
 *          - Converts legacy parameters to new format
 * 
 * @note Called from Copter::init_ardupilot() during initialization
 * @note Allocation failure triggers AP_BoardConfig::allocation_error() with reboot
 * @note Frame type determined by FRAME_CLASS parameter
 * @note Loop rate from scheduler is passed to all control objects (typically 400Hz)
 * 
 * @warning Allocation errors are unrecoverable and will reboot the vehicle
 * @warning Frame type must match physical vehicle configuration for safe flight
 * 
 * @see Copter::init_ardupilot() which calls this function
 * @see FRAME_CLASS parameter for frame type selection
 * @see AP_Motors for motor mixing implementation
 */
void Copter::allocate_motors(void)
{
    // Allocate appropriate motor mixer class based on frame configuration
    // Switch on FRAME_CLASS parameter to determine vehicle frame type
    switch ((AP_Motors::motor_frame_class)g2.frame_class.get()) {
#if FRAME_CONFIG != HELI_FRAME
        // Standard multirotor frames using matrix-based motor mixing
        // Matrix mixer supports arbitrary motor arrangements with configurable thrust factors
        case AP_Motors::MOTOR_FRAME_QUAD:      // Quadcopter (4 motors)
        case AP_Motors::MOTOR_FRAME_HEXA:      // Hexacopter (6 motors)
        case AP_Motors::MOTOR_FRAME_Y6:        // Y6 (6 motors, 3 arms)
        case AP_Motors::MOTOR_FRAME_OCTA:      // Octocopter (8 motors)
        case AP_Motors::MOTOR_FRAME_OCTAQUAD:  // OctoQuad (8 motors, 4 arms)
        case AP_Motors::MOTOR_FRAME_DODECAHEXA:// DodecaHexa (12 motors)
        case AP_Motors::MOTOR_FRAME_DECA:      // Decacopter (10 motors)
        case AP_Motors::MOTOR_FRAME_SCRIPTING_MATRIX: // Custom scripting-defined matrix
        default:
            // Allocate matrix mixer with scheduler loop rate (typically 400Hz)
            motors = NEW_NOTHROW AP_MotorsMatrix(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsMatrix::var_info;
            break;
#if AP_MOTORS_TRI_ENABLED
        // Tricopter frame with 3 motors and yaw servo
        // Uses dedicated class for servo-based yaw control
        case AP_Motors::MOTOR_FRAME_TRI:
            motors = NEW_NOTHROW AP_MotorsTri(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsTri::var_info;
            // Set frame type flag for tricopter-specific parameters
            AP_Param::set_frame_type_flags(AP_PARAM_FRAME_TRICOPTER);
            break;
#endif  // AP_MOTORS_TRI_ENABLED
        // Single rotor (helicopter-like) with tilting mechanism
        case AP_Motors::MOTOR_FRAME_SINGLE:
            motors = NEW_NOTHROW AP_MotorsSingle(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsSingle::var_info;
            break;
        // Coaxial rotor configuration (2 counter-rotating rotors on same axis)
        case AP_Motors::MOTOR_FRAME_COAX:
            motors = NEW_NOTHROW AP_MotorsCoax(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsCoax::var_info;
            break;
        // Tailsitter VTOL configuration
        case AP_Motors::MOTOR_FRAME_TAILSITTER:
            motors = NEW_NOTHROW AP_MotorsTailsitter(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsTailsitter::var_info;
            break;
        // 6-DOF (6 degrees of freedom) scripting-controlled frame
        // Allows full translational and rotational control via Lua scripts
        case AP_Motors::MOTOR_FRAME_6DOF_SCRIPTING:
#if AP_SCRIPTING_ENABLED
            motors = NEW_NOTHROW AP_MotorsMatrix_6DoF_Scripting(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsMatrix_6DoF_Scripting::var_info;
#endif // AP_SCRIPTING_ENABLED
            break;
        // Dynamic matrix defined entirely by Lua scripting
        // Enables custom motor arrangements without firmware changes
        case AP_Motors::MOTOR_FRAME_DYNAMIC_SCRIPTING_MATRIX:
#if AP_SCRIPTING_ENABLED
            motors = NEW_NOTHROW AP_MotorsMatrix_Scripting_Dynamic(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsMatrix_Scripting_Dynamic::var_info;
#endif // AP_SCRIPTING_ENABLED
            break;
#else // FRAME_CONFIG == HELI_FRAME
        // Traditional helicopter frame types
        // Helicopters use cyclic/collective pitch control instead of variable motor speed
        
        // Dual rotor helicopter (tandem, intermeshing, or coaxial with separate swashplates)
        case AP_Motors::MOTOR_FRAME_HELI_DUAL:
            motors = NEW_NOTHROW AP_MotorsHeli_Dual(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsHeli_Dual::var_info;
            AP_Param::set_frame_type_flags(AP_PARAM_FRAME_HELI);
            break;

        // Quad helicopter (4 independent rotor heads with collective pitch)
        case AP_Motors::MOTOR_FRAME_HELI_QUAD:
            motors = NEW_NOTHROW AP_MotorsHeli_Quad(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsHeli_Quad::var_info;
            AP_Param::set_frame_type_flags(AP_PARAM_FRAME_HELI);
            break;
            
        // Single rotor traditional helicopter with tail rotor
        // Most common helicopter configuration
        case AP_Motors::MOTOR_FRAME_HELI:
        default:
            motors = NEW_NOTHROW AP_MotorsHeli_Single(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsHeli_Single::var_info;
            AP_Param::set_frame_type_flags(AP_PARAM_FRAME_HELI);
            break;
#endif
    }
    
    // Check for allocation failure - this is unrecoverable
    // NEW_NOTHROW returns nullptr on allocation failure instead of throwing exception
    if (motors == nullptr) {
        AP_BoardConfig::allocation_error("FRAME_CLASS=%u", (unsigned)g2.frame_class.get());
    }
    // Load motor-specific parameters from EEPROM
    AP_Param::load_object_from_eeprom(motors, motors_var_info);

    // Create AHRS view for control system coordinate transformations
    // View handles rotations between vehicle body frame and control frame
    ahrs_view = ahrs.create_view(ROTATION_NONE);
    if (ahrs_view == nullptr) {
        AP_BoardConfig::allocation_error("AP_AHRS_View");
    }

#if FRAME_CONFIG != HELI_FRAME
    // Allocate attitude controller based on frame capabilities
    // 6-DOF frames can control translation independently from rotation
    if ((AP_Motors::motor_frame_class)g2.frame_class.get() == AP_Motors::MOTOR_FRAME_6DOF_SCRIPTING) {
#if AP_SCRIPTING_ENABLED
        // 6-DOF attitude control allows independent control of all 6 degrees of freedom
        // Enables advanced maneuvers like lateral translation without tilting
        attitude_control = NEW_NOTHROW AC_AttitudeControl_Multi_6DoF(*ahrs_view, aparm, *motors);
        attitude_control_var_info = AC_AttitudeControl_Multi_6DoF::var_info;
#endif // AP_SCRIPTING_ENABLED
    } else {
        // Standard multirotor attitude control
        // Controls roll/pitch/yaw rates and angles using motor thrust vectoring
        attitude_control = NEW_NOTHROW AC_AttitudeControl_Multi(*ahrs_view, aparm, *motors);
        attitude_control_var_info = AC_AttitudeControl_Multi::var_info;
    }
#else
    // Helicopter-specific attitude control
    // Handles cyclic pitch for roll/pitch and collective pitch for throttle
    attitude_control = NEW_NOTHROW AC_AttitudeControl_Heli(*ahrs_view, aparm, *motors);
    attitude_control_var_info = AC_AttitudeControl_Heli::var_info;
#endif
    if (attitude_control == nullptr) {
        AP_BoardConfig::allocation_error("AttitudeControl");
    }
    // Load attitude controller PID gains and configuration from EEPROM
    AP_Param::load_object_from_eeprom(attitude_control, attitude_control_var_info);
        
    // Allocate position controller for velocity and position hold modes
    // Uses attitude controller to convert desired accelerations to attitude targets
    pos_control = NEW_NOTHROW AC_PosControl(*ahrs_view, *motors, *attitude_control);
    if (pos_control == nullptr) {
        AP_BoardConfig::allocation_error("PosControl");
    }
    // Load position controller configuration from EEPROM
    AP_Param::load_object_from_eeprom(pos_control, pos_control->var_info);

#if AP_OAPATHPLANNER_ENABLED
    // Allocate waypoint navigation controller with object avoidance
    // OA version uses path planner to navigate around detected obstacles
    wp_nav = NEW_NOTHROW AC_WPNav_OA(*ahrs_view, *pos_control, *attitude_control);
#else
    // Standard waypoint navigation controller
    // Generates velocity commands to follow waypoint paths
    wp_nav = NEW_NOTHROW AC_WPNav(*ahrs_view, *pos_control, *attitude_control);
#endif
    if (wp_nav == nullptr) {
        AP_BoardConfig::allocation_error("WPNav");
    }
    // Load waypoint navigation parameters (speeds, accelerations, corner behavior)
    AP_Param::load_object_from_eeprom(wp_nav, wp_nav->var_info);

    // Allocate loiter navigation controller for position hold
    // Maintains position over a fixed ground point
    loiter_nav = NEW_NOTHROW AC_Loiter(*ahrs_view, *pos_control, *attitude_control);
    if (loiter_nav == nullptr) {
        AP_BoardConfig::allocation_error("LoiterNav");
    }
    // Load loiter navigation parameters
    AP_Param::load_object_from_eeprom(loiter_nav, loiter_nav->var_info);

#if MODE_CIRCLE_ENABLED
    // Allocate circle navigation controller for orbiting a point
    // Used in Circle flight mode for aerial photography and inspection
    circle_nav = NEW_NOTHROW AC_Circle(*ahrs_view, *pos_control);
    if (circle_nav == nullptr) {
        AP_BoardConfig::allocation_error("CircleNav");
    }
    // Load circle navigation parameters (radius, rate)
    AP_Param::load_object_from_eeprom(circle_nav, circle_nav->var_info);
#endif

    // Reload default parameters that may depend on frame type
    // Some default values are only available after motor object creation
    AP_Param::reload_defaults_file(true);
    
    // Set frame-specific PID defaults for optimal performance
    // Different frames have different moment of inertia and require different tuning
    switch ((AP_Motors::motor_frame_class)g2.frame_class.get()) {
    // Y6 frame has different inertia characteristics than quad
    // Requires lower P and D gains for roll/pitch, higher yaw gains
    case AP_Motors::MOTOR_FRAME_Y6:
        attitude_control->get_rate_roll_pid().kP().set_default(0.1);
        attitude_control->get_rate_roll_pid().kD().set_default(0.006);
        attitude_control->get_rate_pitch_pid().kP().set_default(0.1);
        attitude_control->get_rate_pitch_pid().kD().set_default(0.006);
        attitude_control->get_rate_yaw_pid().kP().set_default(0.15);
        attitude_control->get_rate_yaw_pid().kI().set_default(0.015);
        break;
    // Tricopter yaw servo requires higher D-term filtering
    case AP_Motors::MOTOR_FRAME_TRI:
        attitude_control->get_rate_yaw_pid().filt_D_hz().set_default(100);
        break;
    default:
        break;
    }

    // Brushed motors (coreless) require high PWM frequency (16kHz)
    // Standard ESCs use 50-490Hz, brushed motors need 8-32kHz
    if (motors->is_brushed_pwm_type()) {
        g.rc_speed.set_default(16000);
    }
    
    // Convert legacy parameter format to new format
    // This must be done after allocating objects so new parameters exist
    convert_pid_parameters();
#if FRAME_CONFIG == HELI_FRAME
    // Convert helicopter-specific legacy parameters
    motors->heli_motors_param_conversions();
#endif

#if HAL_PROXIMITY_ENABLED
    // Convert old PRX_ parameters to new PRX1_ format for multiple sensors
    convert_prx_parameters();
#endif

    // Parameter count may have changed during conversions
    // Invalidate cache to force recount on next parameter list request
    AP_Param::invalidate_count();
}

/**
 * @brief Check if vehicle is configured as traditional helicopter
 * 
 * @details This function returns whether the firmware was compiled with traditional
 *          helicopter configuration (FRAME_CONFIG == HELI_FRAME). Traditional helicopters
 *          have fundamentally different control characteristics compared to multirotors:
 *          - Collective pitch control instead of throttle
 *          - Cyclic controls for roll/pitch via swashplate
 *          - Tail rotor or NOTAR for yaw control
 *          - Motor interlock safety mechanism
 *          - Different PID tuning and control laws
 * 
 *          This is a compile-time check, not a runtime configuration. The frame type
 *          is determined during firmware build, not by parameter settings.
 * 
 * @return true if compiled as traditional helicopter (HELI_FRAME)
 * @return false if compiled as multirotor or other frame type
 * 
 * @note This is a compile-time constant, not affected by parameters
 * @note Used to conditionally execute helicopter-specific code paths
 * 
 * @see FRAME_CONFIG in config.h for frame type selection
 * @see allocate_motors() where helicopter-specific motors are instantiated
 */
bool Copter::is_tradheli() const
{
#if FRAME_CONFIG == HELI_FRAME
    return true;
#else
    return false;
#endif
}
