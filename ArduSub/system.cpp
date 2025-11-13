/**
 * @file system.cpp
 * @brief ArduSub system initialization and boot sequence implementation
 * 
 * @details This file implements the core system initialization sequence for ArduSub,
 *          including hardware detection, sensor initialization, communication setup,
 *          and subsystem configuration. The boot sequence ensures all critical systems
 *          are properly initialized before allowing vehicle operation.
 * 
 *          Key responsibilities:
 *          - Hardware abstraction layer initialization
 *          - Sensor detection and calibration (barometer/depth, GPS, compass, IMU)
 *          - Communication protocol setup (GCS MAVLink, RC input/joystick)
 *          - Safety system initialization (failsafes, watchdogs)
 *          - Navigation system startup (AHRS, EKF)
 *          - Peripheral initialization (camera, mount, OSD)
 *          - Position estimate validation
 * 
 *          The initialization follows a specific sequence to ensure dependencies
 *          are met (e.g., barometer must be initialized before temperature sensor
 *          to determine the correct I2C bus).
 * 
 *          Safety-Critical Functions:
 *          - init_ardupilot(): Complete vehicle initialization sequence
 *          - failsafe_check_static(): Mainloop watchdog for detecting software hangs
 *          - position_ok(): Position estimate validation for navigation modes
 * 
 * @note This file is specific to ArduSub (underwater vehicle) and includes
 *       specialized handling for depth sensors and underwater operation.
 * 
 * @warning Modifications to initialization order may cause system instability
 *          or sensor detection failures. Test thoroughly in SITL before hardware.
 * 
 * Source: ArduSub/system.cpp
 */

#include "Sub.h"

/*****************************************************************************
*   The init_ardupilot function processes everything we need for an in - air restart
*        We will determine later if we are actually on the ground and process a
*        ground start in that case.
*
*****************************************************************************/

/**
 * @brief Mainloop watchdog callback for detecting software hangs
 * 
 * @details This static function is registered with the HAL scheduler as a timer
 *          failsafe callback. It is called at 1 Hz (1000ms interval) to verify
 *          that the main control loop is executing properly. If the main loop
 *          hangs or becomes unresponsive, this watchdog can detect the condition
 *          and trigger appropriate failsafe actions.
 * 
 *          The function delegates to the Sub instance's mainloop_failsafe_check()
 *          method to perform the actual health monitoring.
 * 
 * @note This function is called from timer interrupt context at 1 Hz
 * @warning This is a safety-critical function - it must execute quickly and
 *          reliably to detect mainloop failures
 * 
 * @see Sub::mainloop_failsafe_check()
 * @see Sub::init_ardupilot() where this callback is registered
 */
static void failsafe_check_static()
{
    sub.mainloop_failsafe_check();
}

/**
 * @brief Complete ArduSub initialization sequence
 * 
 * @details This function performs the complete initialization of the ArduSub vehicle,
 *          establishing all subsystems required for operation. The initialization
 *          sequence is carefully ordered to ensure dependencies are met.
 * 
 *          Initialization Sequence:
 * 
 *          1. **User Notification**: Initialize LED and buzzer notification system
 *             to provide visual/audio feedback during boot and operation.
 * 
 *          2. **Power Management**: Initialize battery monitor for voltage/current
 *             sensing and capacity tracking.
 * 
 *          3. **Depth Sensor Setup**: Initialize barometer subsystem (includes both
 *             atmospheric pressure sensors and underwater depth sensors).
 *             - Detect board type and configure external barometer I2C bus
 *             - For Pixhawk2: Use internal bus (BARO_EXT_BUS=0)
 *             - For Pixhawk and others: Use external bus (BARO_EXT_BUS=1)
 *             - Detect water pressure sensors (BARO_TYPE_WATER) for depth measurement
 * 
 *          4. **Temperature Monitoring**: Configure temperature sensor I2C bus to
 *             match the barometer's external bus (ensures proper sensor detection).
 * 
 *          5. **Communication Setup**: Initialize GCS telemetry UART ports for
 *             MAVLink communication with ground control stations.
 * 
 *          6. **RC and Joystick Input**: Initialize RC receiver channels and convert
 *             options, set up RC input processing, configure motor outputs to ESCs,
 *             and initialize joystick interface for manual control.
 * 
 *          7. **Relay Control**: Initialize relay outputs for auxiliary equipment
 *             (lights, cameras, grippers, etc.).
 * 
 *          8. **On-Screen Display**: Initialize OSD for real-time telemetry overlay
 *             on FPV video feed.
 * 
 *          9. **Watchdog Registration**: Register mainloop watchdog with HAL scheduler
 *             at 1 Hz to detect software hangs and trigger failsafes.
 * 
 *          10. **GPS Initialization**: Initialize GPS receiver with logging enabled.
 * 
 *          11. **Compass Initialization**: Initialize magnetometer with logging enabled
 *              for heading reference.
 * 
 *          12. **Airspeed Sensor**: Initialize airspeed sensor (typically not used on
 *              submarines but available if configured).
 * 
 *          13. **Optical Flow**: Initialize optical flow sensor for velocity estimation
 *              over seafloor or in confined spaces.
 * 
 *          14. **Camera Mount**: Initialize gimbal/camera mount system with RC targeting
 *              mode and set initial angle to level position.
 * 
 *          15. **Camera Trigger**: Initialize camera trigger interface for photo/video
 *              capture control.
 * 
 *          16. **Barometer Calibration**: Calibrate barometer, detect water pressure
 *              sensors, and configure as primary depth sensor. Set AHRS altitude
 *              measurement noise based on sensor type (0.1m for depth sensor, 10m for
 *              atmospheric barometer).
 * 
 *          17. **Leak Detection**: Initialize leak detector sensors for hull breach
 *              detection (critical for underwater operation).
 * 
 *          18. **Rangefinder**: Initialize rangefinder for obstacle detection and
 *              altitude-above-bottom measurement.
 * 
 *          19. **RPM Sensors**: Initialize RPM sensors for thruster feedback.
 * 
 *          20. **Mission Planning**: Initialize mission library for autonomous
 *              waypoint navigation.
 * 
 *          21. **Data Logging**: Initialize AP_Logger with vehicle-specific startup
 *              message writer.
 * 
 *          22. **Inertial Navigation**: Start up AHRS and INS subsystems, calibrate
 *              gyroscopes, and set vehicle class to SUBMARINE for appropriate
 *              algorithm tuning.
 * 
 *          23. **CPU Failsafe**: Enable CPU load monitoring failsafe to detect
 *              overload conditions.
 * 
 *          24. **IMU Logging**: Enable raw IMU data logging for post-flight analysis.
 * 
 *          25. **Actuator Initialization**: Initialize servo/motor actuator outputs.
 * 
 *          26. **Completion Flag**: Set ap.initialised flag to indicate successful
 *              initialization.
 * 
 * @note Initialization order is critical - do not reorder without understanding
 *       dependencies (e.g., temperature sensor needs barometer bus information).
 * 
 * @warning This is a safety-critical function. Initialization failures can prevent
 *          proper vehicle operation. All failures should be reported via GCS and
 *          notification system.
 * 
 * @see Sub::startup_INS_ground() for AHRS/INS initialization details
 * @see failsafe_check_static() for watchdog callback implementation
 * 
 * Source: ArduSub/system.cpp:15-166
 */
void Sub::init_ardupilot()
{
    // initialise notify system
    notify.init();

    // initialise battery monitor
    battery.init();

    barometer.init();

#if AP_FEATURE_BOARD_DETECT
    // Board-specific barometer I2C bus configuration
    // Different Pixhawk variants have external barometers on different I2C buses
    // This configuration ensures proper detection of external depth sensors
    // Detection won't work until after BoardConfig.init()
    switch (AP_BoardConfig::get_board_type()) {
    case AP_BoardConfig::PX4_BOARD_PIXHAWK2:
        // Pixhawk2/Cube: External barometer on internal I2C bus 0
        AP_Param::set_default_by_name("BARO_EXT_BUS", 0);
        break;
    case AP_BoardConfig::PX4_BOARD_PIXHAWK:
        // Pixhawk v1: External barometer on I2C bus 1
        // Use set_by_name (not set_default_by_name) to force this value
        AP_Param::set_by_name("BARO_EXT_BUS", 1);
        break;
    default:
        // Other boards: Default to external I2C bus 1
        AP_Param::set_default_by_name("BARO_EXT_BUS", 1);
        break;
    }
#elif CONFIG_HAL_BOARD != HAL_BOARD_LINUX
    // Non-Linux platforms without board detect: Default to external bus 1
    AP_Param::set_default_by_name("BARO_EXT_BUS", 1);
#endif
    // Note: Linux boards handle I2C bus enumeration differently and don't need this configuration

#if AP_TEMPERATURE_SENSOR_ENABLED
    // Temperature sensor I2C bus configuration
    // Configure temperature sensor to use the same I2C bus as the external barometer
    // This ensures the temperature sensor is detected on the correct bus where
    // underwater sensors are typically connected
    // In order to preserve Sub's previous AP_TemperatureSensor Behavior we set the Default I2C Bus Here
    AP_Param::set_default_by_name("TEMP1_BUS", barometer.external_bus());
#endif

    // setup telem slots with serial ports
    gcs().setup_uarts();

    // initialise rc channels including setting mode
    rc().convert_options(RC_Channel::AUX_FUNC::ARMDISARM_UNUSED, RC_Channel::AUX_FUNC::ARMDISARM);
    rc().init();


    init_rc_in();               // sets up rc channels from radio
    init_rc_out();              // sets up motors and output to escs
    init_joystick();            // joystick initialization

#if AP_RELAY_ENABLED
    relay.init();
#endif

#if OSD_ENABLED
    osd.init();
#endif

    /*
     * Mainloop watchdog registration (SAFETY-CRITICAL)
     * Register a timer-based failsafe callback that executes at 1 Hz (1000ms interval)
     * to detect if the main control loop becomes unresponsive or hangs.
     * 
     * Note: This relies on the RC library being initialized first, as the watchdog
     * checks RC input timeouts as part of its health monitoring.
     * 
     * The failsafe_check_static callback will trigger appropriate failsafe actions
     * if the mainloop is not executing within expected timing constraints.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

    // Do GPS init
    gps.set_log_gps_bit(MASK_LOG_GPS);
    gps.init();

    AP::compass().set_log_bit(MASK_LOG_COMPASS);
    AP::compass().init();

#if AP_AIRSPEED_ENABLED
    airspeed.set_log_bit(MASK_LOG_IMU);
#endif

#if AP_OPTICALFLOW_ENABLED
    // initialise optical flow sensor
    optflow.init(MASK_LOG_OPTFLOW);
#endif

#if HAL_MOUNT_ENABLED
    // initialise camera mount
    camera_mount.init();
    // This step is necessary so that the servo is properly initialized
    camera_mount.set_angle_target(0, 0, 0, false);
    // for some reason the call to set_angle_targets changes the mode to mavlink targeting!
    camera_mount.set_mode(MAV_MOUNT_MODE_RC_TARGETING);
#endif

#if AP_CAMERA_ENABLED
    // initialise camera
    camera.init();
#endif

#ifdef USERHOOK_INIT
    USERHOOK_INIT
#endif

    // Depth sensor detection and configuration (CRITICAL FOR UNDERWATER OPERATION)
    // Enable barometer logging and perform initial calibration and update
    barometer.set_log_baro_bit(MASK_LOG_IMU);
    barometer.calibrate(false);  // Calibrate without saving parameters
    barometer.update();

    // Scan all detected barometer instances to find water pressure sensor (depth sensor)
    // Water pressure sensors (e.g., MS5837, Bar30) are identified by BARO_TYPE_WATER
    // and provide accurate depth measurement underwater
    for (uint8_t i = 0; i < barometer.num_instances(); i++) {
        if (barometer.get_type(i) == AP_Baro::BARO_TYPE_WATER) {
            // Found a water pressure sensor - configure it as primary for depth measurement
            barometer.set_primary_baro(i);
            depth_sensor_idx = i;
            ap.depth_sensor_present = true;
            sensor_health.depth = barometer.healthy(depth_sensor_idx); // initialize health flag
            break; // Go with the first one we find
        }
    }

    if (!ap.depth_sensor_present) {
        // No external underwater depth sensor detected
        // Fallback to onboard atmospheric barometer (not ideal for depth measurement)
        // We only have onboard baro
        barometer.set_primary_baro(0);
        // Set high altitude measurement noise (10.0m) because atmospheric pressure
        // readings won't correlate well with INS vertical velocity underwater
        ahrs.set_alt_measurement_noise(10.0f);  // Readings won't correspond with rest of INS
    } else {
        // Water pressure sensor detected - set low measurement noise (0.1m)
        // for accurate depth tracking and integration with INS
        ahrs.set_alt_measurement_noise(0.1f);
    }

    leak_detector.init();

    last_pilot_heading = ahrs.yaw_sensor;

    // initialise rangefinder
#if AP_RANGEFINDER_ENABLED
    init_rangefinder();
#endif

    // initialise AP_RPM library
#if AP_RPM_ENABLED
    rpm_sensor.init();
#endif

    // initialise mission library
    mission.init();
#if HAL_LOGGING_ENABLED
    mission.set_log_start_mission_item_bit(MASK_LOG_CMD);
#endif

    // initialise AP_Logger library
#if HAL_LOGGING_ENABLED
    logger.setVehicle_Startup_Writer(FUNCTOR_BIND(&sub, &Sub::Log_Write_Vehicle_Startup_Messages, void));
#endif

    startup_INS_ground();

    // enable CPU failsafe
    mainloop_failsafe_enable();

    ins.set_log_raw_bit(MASK_LOG_IMU_RAW);
    g2.actuators.initialize_actuators();

    // flag that initialisation has completed
    ap.initialised = true;
}


/**
 * @brief Initialize and calibrate Inertial Navigation System (INS) and AHRS
 * 
 * @details This function performs ground-level initialization and calibration of the
 *          inertial measurement unit (IMU) and attitude/heading reference system (AHRS).
 *          Despite the name "ground start", this is used for ArduSub initialization
 *          in water or on surface.
 * 
 *          Initialization sequence:
 *          1. Initialize AHRS backends (may include IMU calibration for certain sensors)
 *          2. Set vehicle class to SUBMARINE for appropriate algorithm tuning
 *          3. Disable fly_forward flag (submarines don't have forward flight aerodynamics)
 *          4. Initialize INS with main loop rate for proper sensor fusion timing
 *          5. Reset AHRS state including gyro bias estimates
 * 
 *          The vehicle class setting affects how AHRS algorithms handle:
 *          - Centripetal force compensation
 *          - Airspeed integration
 *          - Attitude estimation constraints
 * 
 * @note This function performs gyroscope warm-up and offset calibration which
 *       requires the vehicle to be stationary during initialization
 * 
 * @warning Vehicle must be stationary during this initialization. Movement during
 *          gyro calibration will result in incorrect bias estimates and poor
 *          attitude estimation.
 * 
 * @see AP_AHRS::init()
 * @see AP_InertialSensor::init()
 * 
 * Source: ArduSub/system.cpp:172-184
 */
//******************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//******************************************************************************
void Sub::startup_INS_ground()
{
    // initialise ahrs (may push imu calibration into the mpu6000 if using that device).
    ahrs.init();
    ahrs.set_vehicle_class(AP_AHRS::VehicleClass::SUBMARINE);  // Configures AHRS for underwater vehicle dynamics
    ahrs.set_fly_forward(false);  // Submarines don't have forward-flight aerodynamics

    // Warm up and calibrate gyro offsets
    ins.init(scheduler.get_loop_rate_hz());

    // reset ahrs including gyro bias
    ahrs.reset();
}

/**
 * @brief Validate that horizontal position estimate is acceptable for navigation
 * 
 * @details This function checks whether the vehicle has a valid horizontal position
 *          estimate suitable for position-controlled flight modes. It combines checks
 *          from multiple position sources (EKF and optical flow) to determine if
 *          navigation can proceed safely.
 * 
 *          The function first checks for EKF failsafe conditions, then validates
 *          position estimates from available sources. Position can be valid if either:
 *          - EKF has a good absolute or relative position estimate, OR
 *          - Optical flow has a good relative position estimate
 * 
 * @return true if horizontal position estimate is valid for navigation
 * @return false if position estimate is unavailable, unreliable, or EKF failsafe active
 * 
 * @note This is used by flight modes that require position hold or waypoint navigation
 * @warning Navigation modes will refuse to engage or may trigger failsafe if this
 *          returns false. Critical for preventing position drift during missions.
 * 
 * @see Sub::ekf_position_ok() for EKF-based position validation
 * @see Sub::optflow_position_ok() for optical flow position validation
 * 
 * Source: ArduSub/system.cpp:188-197
 */
// calibrate gyros - returns true if successfully calibrated
// position_ok - returns true if the horizontal absolute position is ok and home position is set
bool Sub::position_ok()
{
    // return false if ekf failsafe has triggered
    if (failsafe.ekf) {
        return false;
    }

    // check ekf position estimate
    return (ekf_position_ok() || optflow_position_ok());
}

/**
 * @brief Validate EKF horizontal position estimate quality
 * 
 * @details This function checks whether the Extended Kalman Filter (EKF) is providing
 *          a reliable horizontal position estimate. The validation criteria differ
 *          based on whether the vehicle is armed or disarmed:
 * 
 *          **When Disarmed:**
 *          - Accepts absolute position (HORIZ_POS_ABS) from GPS or other absolute source
 *          - Also accepts predicted position (PRED_HORIZ_POS_ABS) based on last known position
 * 
 *          **When Armed:**
 *          - Requires absolute position estimate (HORIZ_POS_ABS)
 *          - Rejects constant position mode (CONST_POS_MODE) where EKF is dead-reckoning
 *            without position updates
 *          - More stringent requirements to ensure safe navigation during operation
 * 
 *          The function first checks that inertial navigation is available (EKF is
 *          running, not DCM-only mode). DCM does not provide position estimates.
 * 
 * @return true if EKF has valid horizontal absolute position estimate
 * @return false if inertial nav unavailable, position quality insufficient, or in const_pos_mode
 * 
 * @note Stricter validation when armed to ensure safe autonomous operation
 * @warning Const position mode indicates EKF has lost position reference and is
 *          dead-reckoning - unsafe for navigation
 * 
 * @see Sub::position_ok() for combined position validation
 * @see Sub::optflow_position_ok() for optical flow alternative
 * 
 * Source: ArduSub/system.cpp:200-223
 */
// ekf_position_ok - returns true if the ekf claims it's horizontal absolute position estimate is ok and home position is set
bool Sub::ekf_position_ok()
{
    if (!ahrs.have_inertial_nav()) {
        // do not allow navigation with dcm position
        return false;
    }

    // if disarmed we accept a predicted horizontal position
    if (!motors.armed()) {
        if (ahrs.has_status(AP_AHRS::Status::HORIZ_POS_ABS)) {
            return true;
        }
        if (ahrs.has_status(AP_AHRS::Status::PRED_HORIZ_POS_ABS)) {
            return true;
        }
        return false;
    }

    // once armed we require a good absolute position and EKF must not be in const_pos_mode
    if (ahrs.has_status(AP_AHRS::Status::CONST_POS_MODE)) {
        return false;  // Reject constant position mode - EKF has lost position reference
    }
    return ahrs.has_status(AP_AHRS::Status::HORIZ_POS_ABS);
}

/**
 * @brief Validate optical flow or visual odometry position estimate quality
 * 
 * @details This function checks whether optical flow or visual odometry sensors are
 *          providing reliable relative position estimates. This is particularly useful
 *          for underwater operation where GPS is unavailable and the vehicle navigates
 *          relative to the seafloor or visual features.
 * 
 *          Validation criteria:
 *          1. Inertial navigation (EKF) must be active
 *          2. Either optical flow OR visual odometry must be enabled
 *          3. Position estimate quality based on armed state:
 * 
 *          **When Disarmed:**
 *          - Accepts predicted horizontal relative position (PRED_HORIZ_POS_REL)
 * 
 *          **When Armed:**
 *          - Requires horizontal relative position (HORIZ_POS_REL)
 *          - Rejects constant position mode (CONST_POS_MODE)
 * 
 *          Optical flow provides velocity measurements over terrain/seafloor which
 *          the EKF integrates into relative position estimates. Visual odometry
 *          (e.g., Intel T265) provides more advanced camera-based position tracking.
 * 
 * @return true if optical flow or visual odometry position estimate is valid
 * @return false if sensors disabled, EKF unavailable, or position quality insufficient
 * 
 * @note Relative position is useful even without GPS - allows position hold and
 *       waypoint navigation relative to starting point or visual features
 * @warning Optical flow requires adequate lighting and visual texture on seafloor.
 *          Position may drift over featureless or very deep terrain.
 * 
 * @see Sub::position_ok() for combined position validation
 * @see Sub::ekf_position_ok() for GPS-based position validation
 * 
 * Source: ArduSub/system.cpp:226-259
 */
// optflow_position_ok - returns true if optical flow based position estimate is ok
bool Sub::optflow_position_ok()
{
    // return immediately if EKF not used
    if (!ahrs.have_inertial_nav()) {
        return false;
    }

    // return immediately if neither optflow nor visual odometry is enabled
    bool enabled = false;
#if AP_OPTICALFLOW_ENABLED
    if (optflow.enabled()) {
        enabled = true;
    }
#endif
#if HAL_VISUALODOM_ENABLED
    if (visual_odom.enabled()) {
        enabled = true;
    }
#endif
    if (!enabled) {
        return false;
    }

    // if disarmed we accept a predicted horizontal relative position
    if (!motors.armed()) {
        return ahrs.has_status(AP_AHRS::Status::PRED_HORIZ_POS_REL);
    }

    // Reject constant position mode when armed - indicates loss of visual reference
    if (ahrs.has_status(AP_AHRS::Status::CONST_POS_MODE)) {
        return false;
    }

    return ahrs.has_status(AP_AHRS::Status::HORIZ_POS_REL);
}

#if HAL_LOGGING_ENABLED
/**
 * @brief Determine if a message type should be logged based on mask
 * 
 * @details This function checks whether a specific log message type should be written
 *          to the data log based on the configured logging bitmask. It updates the
 *          logging_started status flag and delegates the decision to the logger subsystem.
 * 
 *          Log masks control which message types are recorded:
 *          - MASK_LOG_ATTITUDE_FAST: High-rate attitude data
 *          - MASK_LOG_IMU: IMU sensor data
 *          - MASK_LOG_GPS: GPS position and velocity
 *          - MASK_LOG_CTUN: Control tuning data
 *          - And many others...
 * 
 * @param[in] mask Log message bitmask to check (e.g., MASK_LOG_IMU)
 * 
 * @return true if the message type should be logged
 * @return false if logging is disabled for this message type or logging hasn't started
 * 
 * @note This is called frequently during main loop - must be efficient
 * @see AP_Logger::should_log()
 * 
 * Source: ArduSub/system.cpp:265-269
 */
/*
  should we log a message type now?
 */
bool Sub::should_log(uint32_t mask)
{
    ap.logging_started = logger.logging_started();
    return logger.should_log(mask);
}
#endif

#include <AP_AdvancedFailsafe/AP_AdvancedFailsafe.h>
#include <AP_Avoidance/AP_Avoidance.h>
#include <AP_ADSB/AP_ADSB.h>

/**
 * Linker stub functions for unused features
 * 
 * These dummy implementations prevent linker errors for features that are compiled
 * in ArduPilot libraries but not used by ArduSub:
 * 
 * - Advanced Failsafe (AFS): Termination system for aircraft, not applicable underwater
 * - ADSB Avoidance: ADS-B aircraft collision avoidance, not relevant for submarines
 * 
 * The stubs return nullptr or false to indicate the features are unavailable, allowing
 * the code to compile without pulling in unnecessary dependencies for ArduSub builds.
 */

// dummy method to avoid linking AFS
#if AP_ADVANCEDFAILSAFE_ENABLED
bool AP_AdvancedFailsafe::gcs_terminate(bool should_terminate, const char *reason) { return false; }
AP_AdvancedFailsafe *AP::advancedfailsafe() { return nullptr; }
#endif

#if AP_ADSB_AVOIDANCE_ENABLED
// dummy method to avoid linking AP_Avoidance
AP_Avoidance *AP::ap_avoidance() { return nullptr; }
#endif  // AP_ADSB_AVOIDANCE_ENABLED
