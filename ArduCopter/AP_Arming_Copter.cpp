/**
 * @file AP_Arming_Copter.cpp
 * @brief Copter-specific arming and pre-arm safety check implementations
 * 
 * @details This file implements multicopter-specific pre-arm and arming checks
 *          that verify vehicle readiness for flight. These checks are critical
 *          for flight safety and prevent arming with dangerous configurations.
 *          
 *          The arming system performs two types of checks:
 *          - Pre-arm checks: Comprehensive safety validation before arming is allowed
 *          - Arm checks: Final validation immediately before motors are armed
 *          
 *          Pre-arm checks validate:
 *          - GPS lock quality and position estimate accuracy
 *          - EKF health and state estimation validity
 *          - Compass calibration and consistency
 *          - Barometer sanity and altitude estimate
 *          - RC calibration and failsafe configuration
 *          - Flight mode validity and parameter sanity
 *          - Battery voltage and capacity
 *          - Motor configuration and ESC readiness
 *          - Sensor health (rangefinder, proximity, etc.)
 *          - Geofence and object avoidance configuration
 *          
 *          Each check can be individually enabled/disabled via ARMING_CHECK bitmask,
 *          though disabling safety checks is strongly discouraged.
 *          
 * @warning This is safety-critical code. All checks exist to prevent vehicle
 *          crashes, flyaways, or injury. Modifications must be thoroughly tested
 *          and reviewed for safety implications.
 * 
 * @note Arming checks are performed at approximately 1Hz when disarmed
 * @note Some checks have different thresholds for pre-arm vs arm to prevent
 *       race conditions where passing pre-arm doesn't guarantee passing arm
 * 
 * Source: ArduCopter/AP_Arming_Copter.cpp
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include "Copter.h"

#pragma GCC diagnostic push
#if defined(__clang_major__) && __clang_major__ >= 14
#pragma GCC diagnostic ignored "-Wbitwise-instead-of-logical"
#endif

/**
 * @brief Main entry point for copter pre-arm checks
 * 
 * @details Executes comprehensive pre-arm safety validation and updates
 *          the pre-arm status flags. This is the public interface called
 *          by the main vehicle code to determine if arming should be allowed.
 * 
 * @param[in] display_failure Whether to display failure messages to GCS/pilot
 * 
 * @return true if all pre-arm checks pass, false if any check fails
 * 
 * @note Called at approximately 1Hz when vehicle is disarmed
 * @see run_pre_arm_checks() for the actual check implementation
 */
bool AP_Arming_Copter::pre_arm_checks(bool display_failure)
{
    const bool passed = run_pre_arm_checks(display_failure);
    set_pre_arm_check(passed);
    return passed;
}

/**
 * @brief Execute all pre-arm safety checks for multicopter
 * 
 * @details Performs comprehensive validation of vehicle state before arming.
 *          Checks are organized in priority order with mandatory checks first,
 *          followed by user-configurable checks based on ARMING_CHECK parameter.
 *          
 *          Check sequence:
 *          1. System initialization status
 *          2. Motor interlock and emergency stop switch conflict detection
 *          3. Motor interlock switch position validation
 *          4. Disarm switch validation
 *          5. Motor subsystem arming checks
 *          6. Helicopter autorotation config (if applicable)
 *          7. Parameter validation
 *          8. Object avoidance configuration
 *          9. GCS failsafe state
 *          10. Winch configuration
 *          11. RC throttle failsafe configuration
 *          12. Altitude estimate availability
 *          13. Airspeed sensor (if enabled)
 *          14. Parent class checks (GPS, EKF, compass, barometer, etc.)
 *          
 *          Uses bitwise & operator to ensure ALL checks execute even after
 *          a failure, allowing pilot to see all failure reasons at once.
 * 
 * @param[in] display_failure Whether to display failure messages to GCS/pilot
 *                            via text messages and notification LEDs
 * 
 * @return true if all enabled pre-arm checks pass, false if any check fails
 * 
 * @warning Returns immediately if already armed to prevent check overhead
 *          during flight
 * @warning Motor interlock and emergency stop switches cannot be used together
 *          as this creates conflicting safety logic
 * @warning If checks_to_perform is 0, only mandatory checks are performed
 * 
 * @note Bitwise & instead of logical && ensures all checks run and report
 *       failures even if an earlier check failed
 * @note This function is called at approximately 1Hz when disarmed
 * 
 * @see mandatory_checks() for checks that always run regardless of ARMING_CHECK
 * @see parameter_checks() for parameter validation details
 */
bool AP_Arming_Copter::run_pre_arm_checks(bool display_failure)
{
    // Exit immediately if already armed - no need to perform checks during flight
    // This prevents unnecessary CPU overhead from check execution while flying
    if (copter.motors->armed()) {
        return true;
    }

    // Verify system initialization is complete before allowing arming
    // Ensures all critical subsystems (scheduler, HAL, sensors) are ready
    if (!hal.scheduler->is_system_initialized()) {
        check_failed(display_failure, "System not initialised");
        return false;
    }

    // SAFETY CHECK: Motor interlock and emergency stop switches create conflicting safety logic
    // Motor interlock (MOTOR_INTERLOCK) enables/disables motor output independently of arming
    // Emergency stop (MOTOR_ESTOP, ARM_EMERGENCY_STOP) provides immediate motor shutdown
    // Using both simultaneously creates ambiguous safety behavior - which takes priority?
    // This check ensures only one motor safety mechanism is configured at a time
    bool passed = true;
    if (rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOTOR_INTERLOCK) &&
        (rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOTOR_ESTOP) || 
        rc().find_channel_for_option(RC_Channel::AUX_FUNC::ARM_EMERGENCY_STOP))){
        check_failed(display_failure, "Interlock/E-Stop Conflict");
        passed = false;
    }

    // SAFETY CHECK: Motor interlock switch position validation
    // If motor interlock is enabled, the switch must be in the DISABLED position to arm
    // This prevents accidental motor startup if the interlock switch is already enabled
    // Pilot must explicitly disable interlock, arm, then enable interlock for motor start
    // This two-step process prevents unintended motor activation
    if (copter.ap.using_interlock && copter.ap.motor_interlock_switch) {
        check_failed(display_failure, "Motor Interlock Enabled");
        passed = false;
    }

    // Verify disarm switch is not in the disarm position
    // Prevents arming if pilot is actively commanding disarm via switch
    if (!disarm_switch_checks(display_failure)) {
        passed = false;
    }

    // MANDATORY: Motor subsystem pre-arm checks
    // Validates motor configuration, ESC connectivity, and motor ordering
    // Checks for motor/ESC communication failures, incorrect frame configuration,
    // and ESC calibration issues that could cause loss of control
    char failure_msg[100] {};
    if (!copter.motors->arming_checks(ARRAY_SIZE(failure_msg), failure_msg)) {
        check_failed(display_failure, "Motors: %s", failure_msg);
        passed = false;
    }

#if FRAME_CONFIG == HELI_FRAME && MODE_AUTOROTATE_ENABLED
    // HELICOPTER ONLY: Validate autorotation configuration
    // Autorotation is the emergency procedure for helicopters when engine fails
    // Checks RSC (Rotor Speed Controller) configuration and autorotation entry/exit parameters
    // Ensures helicopter can safely enter autorotation if power is lost
    if (!copter.g2.arot.arming_checks(ARRAY_SIZE(failure_msg), failure_msg)) {
        check_failed(display_failure, "AROT: %s", failure_msg);
        passed = false;
    }
#endif

    // If any mandatory checks failed, exit immediately before running optional checks
    // This ensures critical safety issues are addressed first
    if (!passed) {
        return false;
    }

    // ARMING_CHECK bypass: If all checks disabled (ARMING_CHECK = 0), run only mandatory checks
    // This allows emergency arming in field conditions but is NOT recommended for normal operation
    // Mandatory checks include GPS position estimate, altitude estimate, and AHRS health
    if (checks_to_perform == 0) {
        return mandatory_checks(display_failure);
    }

    // Execute all enabled pre-arm checks using bitwise & operator
    // Bitwise & (instead of logical &&) ensures ALL checks execute even after failures,
    // allowing pilot to see complete list of issues rather than just the first failure
    // This significantly improves debugging efficiency and reduces arming attempt cycles
    return parameter_checks(display_failure)
        & oa_checks(display_failure)              // Object avoidance configuration
        & gcs_failsafe_check(display_failure)     // GCS connection status
        & winch_checks(display_failure)           // Winch configuration if equipped
        & rc_throttle_failsafe_checks(display_failure)  // RC failsafe threshold
        & alt_checks(display_failure)             // Altitude estimate validity
#if AP_AIRSPEED_ENABLED
        & AP_Arming::airspeed_checks(display_failure)   // Airspeed sensor (if enabled)
#endif
        & AP_Arming::pre_arm_checks(display_failure);   // Parent class checks (GPS, EKF, compass, etc.)
}

/**
 * @brief Validate RC throttle failsafe configuration before arming
 * 
 * @details Verifies that RC throttle failsafe is properly configured to detect
 *          RC signal loss. This check prevents arming with failsafe disabled
 *          or with throttle position that would trigger immediate failsafe.
 *          
 *          For traditional helicopters, validates collective pitch instead of throttle.
 *          
 *          Failure modes prevented:
 *          - Flying with no RC failsafe detection (FS_THR_ENABLE = 0)
 *          - Arming with throttle already in failsafe zone
 *          - Arming without ever receiving RC signals
 *          
 *          The check uses FS_THR_VALUE parameter as the threshold below which
 *          RC signal is considered lost. Typical value is 975 (PWM microseconds).
 * 
 * @param[in] display_failure Whether to display failure messages to GCS/pilot
 * 
 * @return true if RC failsafe is properly configured, false otherwise
 * 
 * @warning Returns true if FS_THR_ENABLE = 0 (disabled), allowing arming without
 *          RC failsafe protection. This is dangerous for real flights.
 * @warning Does not detect ongoing RC failsafe condition - that is checked
 *          separately during the arming state machine
 * 
 * @note For helicopters, checks collective pitch channel instead of throttle
 * @note Requires either RC receiver or MAVLink RC override to have been active
 * @note Throttle must be above FS_THR_VALUE to pass this check
 * 
 * Source: ArduCopter/AP_Arming_Copter.cpp:89-125
 */
bool AP_Arming_Copter::rc_throttle_failsafe_checks(bool display_failure) const
{
    // Skip check if RC validation is disabled via ARMING_CHECK parameter
    // Not recommended - RC failsafe is critical for safe operation
    if (!check_enabled(Check::RC)) {
        return true;
    }

    // RC FAILSAFE CONFIGURATION CHECK
    // FS_THR_ENABLE parameter controls throttle failsafe behavior:
    // - FS_THR_DISABLED (0): No RC failsafe - dangerous for real flights
    // - FS_THR_ENABLED_ALWAYS_RTL (1): Always RTL on RC loss
    // - FS_THR_ENABLED_CONTINUE_MISSION (2): Continue mission on RC loss (removed)
    // - FS_THR_ENABLED_ALWAYS_LAND (3): Always land on RC loss
    // 
    // This check validates that failsafe is enabled AND properly configured
    // If disabled, allow arming (though this is unsafe for actual flights)
    if (copter.g.failsafe_throttle == FS_THR_DISABLED) {
        return true;
    }

    // Determine which control channel to check based on frame type
    // Traditional helicopters use collective pitch instead of throttle
#if FRAME_CONFIG == HELI_FRAME
    const char *rc_item = "Collective";
#else
    const char *rc_item = "Throttle";
#endif

    // CRITICAL: Verify we have received RC signals at some point
    // This prevents arming if:
    // - No RC receiver is connected (no pulses ever received)
    // - RC receiver is connected but transmitter is off
    // - RC override via MAVLink has never been used
    // 
    // Note: This check does NOT detect ongoing RC failsafe - that is handled
    // separately. This only validates that RC has worked at least once.
    if (!rc().has_had_rc_receiver() && !rc().has_had_rc_override()) {
        check_failed(Check::RC, display_failure, "RC not found");
        return false;
    }

    // THROTTLE POSITION VALIDATION
    // Verify throttle/collective is ABOVE the failsafe threshold (FS_THR_VALUE)
    // 
    // Why this matters:
    // - If throttle is already below failsafe threshold, arming would
    //   immediately trigger RC failsafe action (RTL/Land)
    // - Prevents arming with transmitter in failsafe state
    // - Typical FS_THR_VALUE is 975 microseconds (PWM)
    // 
    // PWM context:
    // - Normal RC range: 1000-2000 microseconds
    // - Failsafe threshold: typically 975 microseconds
    // - No signal: 0 microseconds or last known value depending on receiver
    if (copter.channel_throttle->get_radio_in() < copter.g.failsafe_throttle_value) {
        check_failed(Check::RC, display_failure, "%s below failsafe", rc_item);
        return false;
    }

    return true;
}

/**
 * @brief Validate barometer health and altitude estimate accuracy
 * 
 * @details Extends parent class barometer checks with copter-specific validation
 *          of altitude estimate consistency between barometer and EKF.
 *          
 *          Checks performed:
 *          1. Basic barometer health (parent class): sensor communication,
 *             recent updates, reasonable pressure values
 *          2. Baro/EKF altitude disparity: Ensures barometer and EKF altitude
 *             estimates agree within PREARM_MAX_ALT_DISPARITY_CM (typically 200cm/2m)
 *          
 *          The disparity check only applies when EKF is using absolute positioning
 *          (GPS-based), not ground-relative positioning. This is because in
 *          ground-relative mode, the EKF may intentionally offset from baro altitude
 *          due to baro drift compensation.
 * 
 * @param[in] display_failure Whether to display failure messages to GCS/pilot
 * 
 * @return true if barometer is healthy and altitude estimates are consistent,
 *         false otherwise
 * 
 * @warning Altitude disparity can indicate barometer sensor failure, EKF
 *          initialization issues, or rapid altitude changes during checks
 * @warning Disparity check skipped in ground-relative EKF modes to avoid
 *          false failures from intentional baro drift compensation
 * 
 * @note PREARM_MAX_ALT_DISPARITY_CM is typically 200cm (2 meters)
 * @note Barometer altitude can drift due to temperature changes and weather
 * @note EKF fuses barometer with GPS and other sensors for altitude estimate
 * 
 * Source: ArduCopter/AP_Arming_Copter.cpp:127-151
 */
bool AP_Arming_Copter::barometer_checks(bool display_failure)
{
    // Execute parent class barometer health checks first
    // Parent checks: sensor communication, recent updates, reasonable pressure values
    if (!AP_Arming::barometer_checks(display_failure)) {
        return false;
    }

    bool ret = true;
    
    // COPTER-SPECIFIC: Barometer/EKF altitude consistency check
    if (check_enabled(Check::BARO)) {
        // ALTITUDE DISPARITY VALIDATION
        // Compare barometer-derived altitude with EKF position estimate
        // This detects barometer sensor failures or EKF initialization problems
        // 
        // Check only applies when EKF is using absolute positioning (GPS-based)
        // because in ground-relative mode, EKF may intentionally differ from
        // baro altitude due to drift compensation
        const auto &ahrs = AP::ahrs();
        
        // Determine if EKF is using absolute (GPS) or relative (non-GPS) positioning
        // using_baro_ref = true means EKF is GPS-based and we should validate disparity
        // using_baro_ref = false means EKF is ground-relative and disparity is expected
        const bool using_baro_ref = !ahrs.has_status(AP_AHRS::Status::PRED_HORIZ_POS_REL) && 
                                     ahrs.has_status(AP_AHRS::Status::PRED_HORIZ_POS_ABS);
        
        // Get EKF altitude estimate relative to origin (typically home or arming location)
        // pos_d_m is DOWN component in NED frame (negative = above origin)
        float pos_d_m = 0;
        UNUSED_RESULT(AP::ahrs().get_relative_position_D_origin_float(pos_d_m));
        
        if (using_baro_ref) {
            // Convert EKF altitude from meters to centimeters and flip sign
            // (-pos_d_m * 100.0) converts DOWN to UP altitude in centimeters
            // copter.baro_alt is barometer altitude in centimeters
            // 
            // PREARM_MAX_ALT_DISPARITY_CM threshold is typically 200cm (2 meters)
            // Disparity > 2m indicates sensor failure or serious configuration problem
            if (fabsf(-pos_d_m * 100.0 - copter.baro_alt) > PREARM_MAX_ALT_DISPARITY_CM) {
                check_failed(Check::BARO, display_failure, "Altitude disparity");
                ret = false;
            }
        }
    }
    return ret;
}

/**
 * @brief Validate Inertial Navigation System (INS) health and EKF attitude
 * 
 * @details Extends parent class INS checks with copter-specific validation
 *          of EKF attitude estimate quality. The attitude estimate is critical
 *          for attitude control and must be valid before arming.
 *          
 *          Checks performed:
 *          1. Parent class INS checks: IMU health, gyro/accel calibration,
 *             sensor consistency, and bias estimation
 *          2. EKF attitude validity: Ensures EKF has converged to a valid
 *             attitude estimate with acceptable uncertainty
 *          
 *          EKF attitude problems are often caused by:
 *          - Excessive gyro biases (calibration needed)
 *          - IMU mounting issues or frame vibration
 *          - Compass interference affecting heading initialization
 *          - Insufficient EKF initialization time after power-on
 * 
 * @param[in] display_failure Whether to display failure messages to GCS/pilot
 * 
 * @return true if INS is healthy and EKF attitude is valid, false otherwise
 * 
 * @warning EKF attitude failures usually indicate gyro bias problems
 * @warning Flying with bad EKF attitude will cause immediate loss of control
 * @warning Attitude validity is more critical than position validity for
 *          attitude-based flight modes (Stabilize, AltHold)
 * 
 * @note EKF needs several seconds after power-on to initialize attitude
 * @note Vehicle should be stationary during EKF initialization
 * @note Gyro biases are automatically estimated by EKF during initialization
 * 
 * @see pre_arm_ekf_attitude_check() for attitude validity criteria
 * 
 * Source: ArduCopter/AP_Arming_Copter.cpp:153-167
 */
bool AP_Arming_Copter::ins_checks(bool display_failure)
{
    // Execute parent class INS health checks
    // Parent checks: IMU communication, gyro/accel calibration, sensor consistency
    bool ret = AP_Arming::ins_checks(display_failure);

    if (check_enabled(Check::INS)) {
        // COPTER-SPECIFIC: EKF attitude estimate validation
        // 
        // The EKF attitude estimate must be valid before arming because:
        // - Attitude control requires accurate roll/pitch/yaw knowledge
        // - Invalid attitude causes immediate loss of control after takeoff
        // - Bad attitude usually indicates gyro bias estimation problems
        // 
        // Common causes of attitude check failure:
        // - Excessive gyro biases requiring calibration
        // - IMU not mounted securely or experiencing vibration
        // - Compass interference affecting heading initialization
        // - Insufficient warm-up time after power-on (EKF needs ~10 seconds)
        // - Vehicle moved during EKF initialization
        if (!pre_arm_ekf_attitude_check()) {
            check_failed(Check::INS, display_failure, "EKF attitude is bad");
            ret = false;
        }
    }

    return ret;
}

/**
 * @brief Validate board power supply and battery health before arming
 * 
 * @details Extends parent class voltage checks with copter-specific validation
 *          of battery failsafe state. Prevents arming if battery is already in
 *          failsafe condition due to low voltage, low capacity, or critical state.
 *          
 *          Checks performed:
 *          1. Parent class board voltage checks: servo rail voltage, VCC voltage,
 *             and minimum operating voltage for flight computer
 *          2. Battery failsafe state: Ensures battery monitor is not reporting
 *             low voltage, low capacity, or critical battery conditions
 *          
 *          Battery failsafe triggers based on:
 *          - BATT_LOW_VOLT: Voltage threshold for low battery warning
 *          - BATT_CRT_VOLT: Critical voltage triggering immediate landing
 *          - BATT_LOW_MAH: Capacity threshold for low battery warning
 *          - BATT_CRT_MAH: Critical capacity triggering immediate landing
 * 
 * @param[in] display_failure Whether to display failure messages to GCS/pilot
 * 
 * @return true if all voltage checks pass and battery is healthy, false otherwise
 * 
 * @warning Flying with battery already in failsafe will trigger immediate
 *          failsafe action (RTL or Land) upon arming
 * @warning Low board voltage can cause brownouts during motor spin-up,
 *          leading to flight computer resets and crashes
 * @warning Battery voltage can drop significantly under load - check voltage
 *          with motors at flight idle, not just with system powered on
 * 
 * @note Battery monitor must be calibrated for accurate voltage/capacity readings
 * @note Voltage sag during arming is normal - this check uses current voltage
 * @note Multiple battery monitors supported - all must pass failsafe check
 * 
 * Source: ArduCopter/AP_Arming_Copter.cpp:169-184
 */
bool AP_Arming_Copter::board_voltage_checks(bool display_failure)
{
    // Execute parent class voltage checks
    // Parent checks: servo rail voltage, VCC voltage, minimum operating voltage
    if (!AP_Arming::board_voltage_checks(display_failure)) {
        return false;
    }

    // COPTER-SPECIFIC: Battery failsafe state validation
    if (check_enabled(Check::VOLTAGE)) {
        // BATTERY HEALTH CHECK
        // Verify battery is not already in failsafe condition before arming
        // 
        // Battery failsafe triggers when:
        // - Voltage drops below BATT_LOW_VOLT (warning) or BATT_CRT_VOLT (critical)
        // - Remaining capacity drops below BATT_LOW_MAH or BATT_CRT_MAH
        // - Battery reports critical state via smart battery protocol
        // 
        // Arming with battery already in failsafe would immediately trigger
        // failsafe action (RTL or Land), preventing takeoff
        // 
        // Common causes:
        // - Battery not fully charged before flight
        // - Battery voltage sag due to high current draw or damaged cells
        // - Incorrect battery monitor calibration (BATT_VOLT_MULT, BATT_AMP_PERVLT)
        // - Wrong battery capacity configured (BATT_CAPACITY)
        if (copter.battery.has_failsafed()) {
            check_failed(Check::VOLTAGE, display_failure, "Battery failsafe");
            return false;
        }
    }

    return true;
}

/**
 * @brief Determine if terrain database must be loaded before arming
 * 
 * @details Evaluates whether the current configuration requires terrain data
 *          to be available before allowing arming. This is necessary when using
 *          terrain-relative flight modes or altitude references.
 *          
 *          Terrain database required when:
 *          - RTL_ALT_TYPE = 1 (TERRAIN) and terrain source is TERRAINDATABASE
 *          
 *          Terrain database NOT required when:
 *          - Terrain source is RANGEFINDER (uses real-time rangefinder instead)
 *          - RTL using barometric altitude reference
 *          - Mission/Auto mode not using terrain-relative waypoints
 *          
 *          Terrain data provides ground elevation information for:
 *          - Terrain-following in Auto missions
 *          - RTL with terrain-relative altitude
 *          - Obstacle clearance over varying terrain
 * 
 * @return true if terrain database must be fully loaded before arming,
 *         false if arming allowed without terrain data
 * 
 * @warning Flying terrain-relative missions without terrain data will cause
 *          mission failures and potential crashes in mountainous areas
 * @warning Terrain database requires GPS fix and valid home position
 * @warning Terrain data may not be available in all geographic regions
 * 
 * @note Terrain database is loaded from SD card (SRTM data) or
 *       downloaded via MAVLink from ground station
 * @note Rangefinder can be used as primary terrain source for
 *       real-time terrain following without database
 * @note Parent class provides additional terrain requirement logic
 * 
 * Source: ArduCopter/AP_Arming_Copter.cpp:188-201
 */
bool AP_Arming_Copter::terrain_database_required() const
{
    // TERRAIN SOURCE EVALUATION
    // Determine if terrain database is needed based on configured terrain source
    
    // CASE 1: Rangefinder is primary terrain source
    // Real-time rangefinder data provides terrain clearance without database
    // This is useful for flights over flat terrain or when terrain data unavailable
    if (copter.wp_nav->get_terrain_source() == AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER) {
        return false;  // Database not required - using live sensor data
    }

    // CASE 2: Terrain database is primary source AND RTL uses terrain-relative altitude
    // This combination requires terrain database to be loaded for safe RTL
    // RTL with terrain-relative altitude needs ground elevation data to maintain
    // safe clearance above varying terrain during return flight
    if (copter.wp_nav->get_terrain_source() == AC_WPNav::TerrainSource::TERRAIN_FROM_TERRAINDATABASE &&
        copter.mode_rtl.get_alt_type() == ModeRTL::RTLAltType::TERRAIN) {
        return true;  // Database required for terrain-relative RTL
    }
    
    // CASE 3: Other configurations - defer to parent class logic
    // Parent class checks mission waypoints and other terrain usage
    return AP_Arming::terrain_database_required();
}

/**
 * @brief Comprehensive parameter sanity and configuration validation
 * 
 * @details Validates critical flight parameters and configuration settings that
 *          affect vehicle safety and flight behavior. This is one of the most
 *          comprehensive pre-arm checks, covering failsafe configuration, control
 *          limits, frame configuration, and subsystem parameters.
 *          
 *          Parameter categories validated:
 *          1. Failsafe configuration (FS_THR_VALUE, FS_GCS_ENABLE)
 *          2. Control angle limits (ANGLE_MAX)
 *          3. Acro mode balance parameters (ACRO_BAL_ROLL/PITCH)
 *          4. Vertical velocity limits (PILOT_SPEED_UP)
 *          5. Helicopter-specific configuration (motor interlock, autorotation)
 *          6. Frame class validation (prevents invalid frame types)
 *          7. RTL terrain configuration with rangefinder
 *          8. ADSB avoidance threat detection
 *          9. Position and attitude controller parameters
 *          
 *          Many parameter checks have specific thresholds and valid ranges
 *          that prevent dangerous configurations.
 * 
 * @param[in] display_failure Whether to display failure messages to GCS/pilot
 * 
 * @return true if all parameter checks pass, false if any parameter invalid
 * 
 * @warning Parameter misconfigurations are a common cause of crashes
 * @warning Some parameter combinations create subtle stability issues
 * @warning Invalid parameters may pass ground testing but fail in flight
 * @warning ADSB threat detection prevents arming near other aircraft
 * 
 * @note This function checks 15+ different parameter categories
 * @note Position controller (PSC) and attitude controller (ATC) have
 *       their own internal parameter validation
 * @note Helicopter frame requires motor interlock to be configured
 * 
 * Source: ArduCopter/AP_Arming_Copter.cpp:203-321
 */
bool AP_Arming_Copter::parameter_checks(bool display_failure)
{
    // Execute parameter validation if enabled
    // Skip if ARMING_CHECK parameter disables parameter checking (not recommended)
    if (check_enabled(Check::PARAMETERS)) {

        // THROTTLE FAILSAFE CONFIGURATION VALIDATION
        if (copter.g.failsafe_throttle) {
            // CRITICAL SAFETY CHECK: Throttle failsafe threshold configuration
            // 
            // Requirements:
            // 1. RC_MIN (throttle minimum) must be at least 10us above FS_THR_VALUE
            // 2. FS_THR_VALUE must be at least 910us
            // 
            // Why these requirements:
            // - PPM encoders typically output 900us when signal is lost
            // - FS_THR_VALUE must be above 900us to reliably detect signal loss
            // - RC_MIN must be above failsafe threshold to prevent false triggers
            // - 10us margin accounts for radio jitter and encoder inaccuracies
            // 
            // Typical values:
            // - RC_MIN: 1000-1100us (normal minimum throttle)
            // - FS_THR_VALUE: 975us (failsafe detection threshold)
            // - Signal loss: 0us or 900us depending on receiver
            if (copter.channel_throttle->get_radio_min() <= copter.g.failsafe_throttle_value+10 || 
                copter.g.failsafe_throttle_value < 910) {
                check_failed(Check::PARAMETERS, display_failure, "Check FS_THR_VALUE");
                return false;
            }
        }
        
        // GCS FAILSAFE CONFIGURATION VALIDATION
        // FS_GCS_ENABLE = 2 (continue mission on GCS loss) was removed due to safety concerns
        // Continuing mission without GCS prevents pilot intervention during emergencies
        // Use FS_OPTIONS to configure GCS failsafe behavior instead
        if (copter.g.failsafe_gcs == FS_GCS_ENABLED_CONTINUE_MISSION) {
            check_failed(Check::PARAMETERS, display_failure, "FS_GCS_ENABLE=2 removed, see FS_OPTIONS");
        }

        // MAXIMUM LEAN ANGLE VALIDATION (ANGLE_MAX parameter)
        // Controls maximum allowed roll/pitch angles for pilot input
        // 
        // Valid range: 1000-8000 centidegrees (10-80 degrees)
        // Recommended: 2000-4500 centidegrees (20-45 degrees)
        // 
        // Why limits matter:
        // - Below 10 degrees: Insufficient control authority, unable to reject disturbances
        // - Above 80 degrees: Extreme angles risk loss of altitude and control
        // - Competition pilots: 45-60 degrees typical
        // - Aerial photography: 20-30 degrees for stable footage
        // 
        // This parameter is used by:
        // - Stabilize mode for max lean angle limit
        // - AltHold, Loiter, PosHold for lean angle limits
        // - Altitude controller for tilt compensation
        if (copter.aparm.angle_max < 1000 || copter.aparm.angle_max > 8000) {
            check_failed(Check::PARAMETERS, display_failure, "Check ANGLE_MAX");
            return false;
        }

        // ACRO MODE BALANCE PARAMETER VALIDATION
#if MODE_ACRO_ENABLED || MODE_SPORT_ENABLED
        // ACRO_BAL_ROLL and ACRO_BAL_PITCH control automatic leveling in Acro mode
        // These parameters must not exceed angle controller P gains to prevent instability
        // 
        // Why this matters:
        // - Acro balance provides self-leveling when sticks are centered
        // - Balance gain higher than angle P gain causes oscillations
        // - Creates feedback loop conflict between acro balance and attitude controller
        // 
        // Typical values:
        // - ACRO_BAL_ROLL/PITCH: 0-1.0 (0 = no leveling, 1 = full leveling)
        // - Angle P gains: typically 4.5-7.0
        // - Balance should be fraction of angle P gain
        if ((copter.g.acro_balance_roll > copter.attitude_control->get_angle_roll_p().kP()) || 
            (copter.g.acro_balance_pitch > copter.attitude_control->get_angle_pitch_p().kP())) {
            check_failed(Check::PARAMETERS, display_failure, "Check ACRO_BAL_ROLL/PITCH");
            return false;
        }
#endif

        // VERTICAL CLIMB RATE PARAMETER VALIDATION (PILOT_SPEED_UP)
        // Maximum pilot-commanded climb rate in cm/s
        // 
        // Why must be positive:
        // - Zero or negative value prevents climbing
        // - Used to scale throttle stick input to desired vertical velocity
        // - Negative value would invert throttle control (up=down, down=up)
        // 
        // Typical values:
        // - Conservative: 250 cm/s (2.5 m/s)
        // - Standard: 500 cm/s (5 m/s)
        // - Aggressive: 1000+ cm/s (10+ m/s)
        // 
        // Affects flight modes: AltHold, Loiter, PosHold, Guided, Auto
        if (copter.g.pilot_speed_up <= 0) {
            check_failed(Check::PARAMETERS, display_failure, "Check PILOT_SPEED_UP");
            return false;
        }

        #if FRAME_CONFIG == HELI_FRAME
        // HELICOPTER-SPECIFIC PARAMETER VALIDATION
        char fail_msg[100]{};
        
        // HELICOPTER INPUT MANAGER VALIDATION
        // Input manager handles collective pitch, cyclic, and tail rotor mixing
        // Parameters control swashplate geometry and servo travel limits
        // Invalid parameters cause incorrect rotor control and crashes
        if (!copter.input_manager.parameter_check(fail_msg, ARRAY_SIZE(fail_msg))) {
            check_failed(Check::PARAMETERS, display_failure, "%s", fail_msg);
            return false;
        }

        // HELICOPTER MOTOR INTERLOCK REQUIREMENT
        // Traditional helicopters MUST have motor interlock configured
        // 
        // Why required:
        // - Separates rotor engagement from arming for safety
        // - Allows ground checks with rotor stopped
        // - Prevents accidental rotor engagement during startup
        // - Standard helicopter safety practice
        // 
        // Workflow:
        // 1. Arm vehicle (rotor still stopped)
        // 2. Perform final checks
        // 3. Enable motor interlock to engage rotor
        // 4. Increase collective to takeoff
        if (rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOTOR_INTERLOCK) == nullptr) {
            check_failed(Check::PARAMETERS, display_failure, "Motor Interlock not configured");
            return false;
        }

        #else
        // MULTICOPTER FRAME CLASS VALIDATION
        // Prevent helicopter frame classes from being used with multicopter firmware
        // 
        // Why this check exists:
        // - Helicopter frames require different motor mixing algorithms
        // - Swashplate servo mixing incompatible with multicopter ESC control
        // - Using wrong frame class causes unpredictable motor output
        // - Results in immediate loss of control on takeoff
        // 
        // Use dedicated ArduHeli firmware for helicopter frames
        switch (copter.g2.frame_class.get()) {
        case AP_Motors::MOTOR_FRAME_HELI_QUAD:      // Quad helicopter (4 rotors with swashplates)
        case AP_Motors::MOTOR_FRAME_HELI_DUAL:      // Dual helicopter (tandem rotors)
        case AP_Motors::MOTOR_FRAME_HELI:           // Single-rotor helicopter
            check_failed(Check::PARAMETERS, display_failure, "Invalid MultiCopter FRAME_CLASS");
            return false;

        default:
            break;
        }
        #endif // HELI_FRAME

        // RTL TERRAIN-RELATIVE ALTITUDE CONFIGURATION VALIDATION
#if MODE_RTL_ENABLED
        // When RTL_ALT_TYPE = TERRAIN, RTL maintains altitude above ground instead of above home
        // This requires terrain data source to be available and properly configured
        if (copter.mode_rtl.get_alt_type() == ModeRTL::RTLAltType::TERRAIN) {
            const char *failure_template = "RTL_ALT_TYPE is above-terrain but %s";
            
            // Evaluate terrain data source configuration
            switch (copter.wp_nav->get_terrain_source()) {
            case AC_WPNav::TerrainSource::TERRAIN_UNAVAILABLE:
                // NO TERRAIN SOURCE CONFIGURED
                // RTL requires terrain data but none is available
                // Could be missing terrain database, rangefinder, or GPS
                check_failed(Check::PARAMETERS, display_failure, failure_template, "no terrain data");
                return false;
                break;
                
            case AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER:
                // RANGEFINDER-BASED TERRAIN FOLLOWING
#if AP_RANGEFINDER_ENABLED
                // Verify rangefinder is enabled and has downward-facing sensor
                // ROTATION_PITCH_270 = pointing down (-Z axis in body frame)
                if (!copter.rangefinder_state.enabled || 
                    !copter.rangefinder.has_orientation(ROTATION_PITCH_270)) {
                    check_failed(Check::PARAMETERS, display_failure, failure_template, "no rangefinder");
                    return false;
                }
                
                // CRITICAL: RTL altitude must be within rangefinder maximum range
                // If RTL_ALT exceeds rangefinder max range, terrain following will fail
                // during RTL when vehicle climbs above sensor range
                // 
                // Units:
                // - RTL_ALT (copter.g.rtl_altitude): centimeters
                // - RNGFND_MAX_CM: centimeters  
                // - max_distance_orient() returns: meters (multiply by 100 for cm)
                // 
                // Example:
                // - RTL_ALT = 1500cm (15m)
                // - RNGFND_MAX = 50m
                // - Check: 1500cm < 5000cm ✓ Pass
                if (copter.g.rtl_altitude > copter.rangefinder.max_distance_orient(ROTATION_PITCH_270) * 100) {
                    check_failed(Check::PARAMETERS, display_failure, failure_template, 
                                "RTL_ALT (in cm) above RNGFND_MAX (in metres)");
                    return false;
                }
#else
                // Firmware compiled without rangefinder support
                // Cannot use rangefinder terrain source
                check_failed(Check::PARAMETERS, display_failure, failure_template, "rangefinder not in firmware");
#endif
                break;
                
            case AC_WPNav::TerrainSource::TERRAIN_FROM_TERRAINDATABASE:
                // TERRAIN DATABASE-BASED ALTITUDE
                // Terrain database checks performed in parent AP_Arming class
                // Validates terrain data is loaded for current location
                break;
            }
        }
#endif

        // ADSB COLLISION AVOIDANCE THREAT DETECTION
#if HAL_ADSB_ENABLED
        // SAFETY CHECK: Detect nearby aircraft via ADS-B before arming
        // 
        // ADS-B (Automatic Dependent Surveillance-Broadcast) provides real-time
        // aircraft position data for collision avoidance. This check prevents
        // arming if another aircraft is detected within configured threat radius.
        // 
        // Threat detection criteria:
        // - ADSB_ENABLE: ADS-B system enabled
        // - Aircraft within ADSB_RADIUS (horizontal distance)
        // - Altitude separation less than ADSB_VERT_SEP
        // - Projected collision within warning time
        // 
        // Why this matters:
        // - Prevents takeoff into path of manned aircraft
        // - Required for BVLOS (Beyond Visual Line of Sight) operations
        // - Regulatory compliance for commercial operations
        // - Automatic deconfliction in controlled airspace
        if (copter.failsafe.adsb) {
            check_failed(Check::PARAMETERS, display_failure, "ADSB threat detected");
            return false;
        }
#endif

        // POSITION CONTROLLER PARAMETER VALIDATION (PSC_* parameters)
        // Position controller manages horizontal position, velocity, and acceleration
        // Invalid parameters cause oscillations, position drift, or loss of position hold
        // 
        // Parameters validated:
        // - PSC_POSXY_P: Position error to velocity gain
        // - PSC_VELXY_P/I/D/IMAX/FILT: Velocity controller tuning
        // - PSC_ACCXY_P/I/D/IMAX/FILT: Acceleration controller tuning
        // - PSC_JERK_XY: Maximum horizontal jerk limit
        // 
        // Common failures:
        // - P gains too high: oscillations and instability
        // - I gains too high: overshoot and slow convergence
        // - Filter frequencies incompatible: noise amplification
        char failure_msg[100] = {};
        if (!copter.pos_control->pre_arm_checks("PSC", failure_msg, ARRAY_SIZE(failure_msg))) {
            check_failed(Check::PARAMETERS, display_failure, "Bad parameter: %s", failure_msg);
            return false;
        }
        
        // ATTITUDE CONTROLLER PARAMETER VALIDATION (ATC_* parameters)
        // Attitude controller manages roll, pitch, yaw angles and rates
        // Most critical controller for stability - invalid parameters cause immediate crashes
        // 
        // Parameters validated:
        // - ATC_RAT_RLL_P/I/D/IMAX/FILT: Roll rate controller
        // - ATC_RAT_PIT_P/I/D/IMAX/FILT: Pitch rate controller
        // - ATC_RAT_YAW_P/I/D/IMAX/FILT: Yaw rate controller
        // - ATC_ANG_RLL_P: Roll angle to rate gain
        // - ATC_ANG_PIT_P: Pitch angle to rate gain
        // - ATC_ANG_YAW_P: Yaw angle to rate gain
        // - ATC_SLEW_YAW: Yaw acceleration limit
        // 
        // Why critical:
        // - Runs at fast loop rate (400Hz typical)
        // - Direct control of motor outputs
        // - No backup if attitude control fails
        // - Incorrect tuning causes immediate loss of control
        if (!copter.attitude_control->pre_arm_checks("ATC", failure_msg, ARRAY_SIZE(failure_msg))) {
            check_failed(Check::PARAMETERS, display_failure, "Bad parameter: %s", failure_msg);
            return false;
        }
    }

    return true;
}

/**
 * @brief Validate object avoidance system configuration
 * 
 * @details Verifies that the object avoidance path planner is properly configured
 *          if enabled. Object avoidance uses proximity sensors and path planning
 *          algorithms to navigate around obstacles during autonomous missions.
 * 
 *          Configuration requirements:
 *          - At least one proximity sensor properly configured
 *          - Path planning margin parameters within valid ranges
 *          - Avoidance behavior parameters compatible with vehicle type
 *          - Sufficient CPU resources for path planning computation
 * 
 *          Common failures:
 *          - OA_TYPE enabled but no proximity sensors configured
 *          - OA_MARGIN_MAX less than OA_LOOKAHEAD (impossible geometry)
 *          - Invalid avoidance behavior for current flight mode
 * 
 * @param[in] display_failure If true, send error messages to GCS
 * 
 * @return true if object avoidance checks pass or disabled, false if validation fails
 * 
 * @note Only checks configuration, not real-time sensor data availability
 */
bool AP_Arming_Copter::oa_checks(bool display_failure)
{
#if AP_OAPATHPLANNER_ENABLED
    // OBJECT AVOIDANCE PATH PLANNER VALIDATION
    // Bendy ruler or Dijkstra path planning with proximity sensors
    char failure_msg[100] = {};
    if (copter.g2.oa.pre_arm_check(failure_msg, ARRAY_SIZE(failure_msg))) {
        return true;
    }
    
    // Object avoidance validation failed - report specific error
    if (strlen(failure_msg) == 0) {
        // No specific error message from OA system, use generic message
        check_failed(display_failure, "%s", "Check Object Avoidance");
    } else {
        // Display specific error from OA pre-arm check
        check_failed(display_failure, "%s", failure_msg);
    }
    return false;
#else
    // Object avoidance not compiled into firmware - always pass
    return true;
#endif
}

/**
 * @brief Validate RC (Radio Control) calibration and channel configuration
 * 
 * @details Verifies that all primary control channels are properly calibrated and
 *          configured with appropriate min/max/trim values. Poor RC calibration is
 *          a common cause of flyaways, crashes, and unpredictable vehicle behavior.
 * 
 *          Validation performed:
 *          - All primary channels (roll, pitch, throttle, yaw) have valid PWM ranges
 *          - Channel min/max separation sufficient (typically >200μs)
 *          - Trim values within expected ranges
 *          - No reversed channels on control inputs
 *          - PWM values within RC protocol bounds (800-2200μs typical)
 *          - Failsafe values properly configured
 * 
 *          Primary control channels:
 *          - Roll: Lateral cyclic control (channel 1 typical)
 *          - Pitch: Longitudinal cyclic control (channel 2 typical)
 *          - Throttle: Collective/throttle control (channel 3 typical)
 *          - Yaw: Tail rotor/rudder control (channel 4 typical)
 * 
 * @param[in] display_failure If true, send error messages to GCS
 * 
 * @return true if all RC channels properly calibrated, false if any channel fails
 * 
 * @warning Flying with poorly calibrated RC is extremely dangerous - can result in
 *          unintended full-throttle or aggressive attitude commands on arming
 * 
 * @note Sets copter.ap.pre_arm_rc_check flag for status reporting
 */
bool AP_Arming_Copter::rc_calibration_checks(bool display_failure)
{
    // Define primary control channels to validate
    // These channels must be properly calibrated for safe flight
    const RC_Channel *channels[] = {
        copter.channel_roll,      // Roll (lateral) control
        copter.channel_pitch,     // Pitch (longitudinal) control
        copter.channel_throttle,  // Throttle/collective control
        copter.channel_yaw        // Yaw (heading) control
    };

    // Perform comprehensive RC calibration checks
    // bitwise & ensures ALL checks are run even if one fails
    // (logical && would short-circuit and skip later checks)
    copter.ap.pre_arm_rc_check = rc_checks_copter_sub(display_failure, channels)
        & AP_Arming::rc_calibration_checks(display_failure);

    return copter.ap.pre_arm_rc_check;
}

/**
 * @brief Validate GPS lock quality and configuration for safe flight
 * 
 * @details Comprehensive GPS validation ensuring sufficient accuracy for the intended
 *          flight mode. GPS quality directly affects position hold, waypoint navigation,
 *          RTL (Return to Launch), and geofencing reliability.
 * 
 *          GPS required for:
 *          - Position-based flight modes (Loiter, PosHold, Auto, Guided, RTL, etc.)
 *          - Geofencing (circular or polygon fences)
 *          - Super Simple mode (uses GPS heading to home)
 *          
 *          Validation criteria:
 *          - GPS fix type: 3D fix minimum (GPS_TYPE)
 *          - Satellite count: Sufficient satellites visible (GPS_MIN_SAT)
 *          - HDOP (Horizontal Dilution of Precision): Below threshold for accuracy
 *          - EKF variance: Position estimate uncertainty within limits
 *          - No GPS glitching reported by EKF
 * 
 *          HDOP interpretation:
 *          - HDOP < 1.5: Excellent accuracy
 *          - HDOP 1.5-2.5: Good accuracy (typical threshold)
 *          - HDOP 2.5-5.0: Moderate accuracy
 *          - HDOP > 5.0: Poor accuracy - position hold unreliable
 * 
 * @param[in] display_failure If true, send error messages to GCS
 * 
 * @return true if GPS meets quality requirements or not needed, false if inadequate
 * 
 * @warning Poor GPS can cause flyaways - vehicle may drift significantly from intended
 *          position even in position-hold modes. In Auto mode with poor GPS, vehicle
 *          may miss waypoints by large margins or fail to return home accurately.
 * 
 * @note Updates AP_Notify::flags.pre_arm_gps_check for LED/buzzer status indication
 */
bool AP_Arming_Copter::gps_checks(bool display_failure)
{
    // DETERMINE IF GPS IS REQUIRED FOR CURRENT CONFIGURATION
    
    // Check if geofencing requires GPS
    bool fence_requires_gps = false;
#if AP_FENCE_ENABLED
    // Circular and polygon fences depend on GPS position
    // Altitude-only fences can work with barometer alone
    fence_requires_gps = (copter.fence.get_enabled_fences() & (AC_FENCE_TYPE_CIRCLE | AC_FENCE_TYPE_POLYGON)) > 0;
#endif

    // Determine if GPS is needed based on:
    // 1. Current flight mode requirements (Loiter, Auto, Guided, RTL, etc.)
    // 2. Geofence configuration (circular or polygon fences active)
    // 3. Super Simple mode enabled (needs GPS heading to home)
    bool mode_requires_gps = copter.flightmode->requires_GPS() || 
                             fence_requires_gps || 
                             (copter.simple_mode == Copter::SimpleMode::SUPERSIMPLE);

    // COMPREHENSIVE GPS QUALITY VALIDATION (if GPS required)
    if (mode_requires_gps) {
        // Call parent class GPS checks:
        // - Minimum satellite count (GPS_MIN_SAT parameter)
        // - 3D fix achieved
        // - Fresh GPS data being received
        // - GPS horizontal accuracy within limits
        if (!AP_Arming::gps_checks(display_failure)) {
            AP_Notify::flags.pre_arm_gps_check = false;
            return false;
        }
    }

    // MANDATORY GPS CHECKS (always run regardless of mode)
    // Validates EKF health, position estimate availability, and GPS glitching
    if (!mandatory_gps_checks(display_failure)) {
        AP_Notify::flags.pre_arm_gps_check = false;
        return false;
    }

    // GPS NOT REQUIRED - allow arming
    // Flight modes like Stabilize, AltHold, Acro don't need GPS
    if (!mode_requires_gps) {
        AP_Notify::flags.pre_arm_gps_check = true;
        return true;
    }

    // GPS CHECK DISABLED via ARMING_CHECK parameter - allow arming
    // User has explicitly disabled GPS quality verification
    if (!check_enabled(Check::GPS)) {
        AP_Notify::flags.pre_arm_gps_check = true;
        return true;
    }

    // HDOP (Horizontal Dilution of Precision) QUALITY CHECK
    // HDOP indicates GPS geometry quality - lower is better
    // 
    // Why check HDOP separately from GPS lock:
    // - Vehicle can have 3D fix but poor geometry (satellites clustered)
    // - Poor geometry = large position errors even with lock
    // - Users often confused by "GPS locked but can't arm" message
    // - Separate HDOP message clarifies the issue
    // 
    // Threshold: GPS_HDOP_GOOD parameter (default 2.5)
    if ((copter.gps.num_sensors() > 0) && 
        (copter.gps.get_hdop() > copter.g.gps_hdop_good)) {
        check_failed(Check::GPS, display_failure, "High GPS HDOP");
        AP_Notify::flags.pre_arm_gps_check = false;
        return false;
    }

    // ALL GPS CHECKS PASSED
    AP_Notify::flags.pre_arm_gps_check = true;
    return true;
}

/**
 * @brief Verify EKF attitude estimate is valid and converged
 * 
 * @details Checks that the Extended Kalman Filter has achieved a valid attitude
 *          solution before allowing arming. Poor EKF attitude is commonly caused by:
 *          
 *          - Uncalibrated gyroscope biases (needs time to settle after power-up)
 *          - Excessive vibration affecting IMU sensors
 *          - Gyroscope temperature not stabilized
 *          - Accelerometer calibration errors
 *          - Magnetic interference affecting compass-based heading
 * 
 *          EKF attitude convergence process:
 *          1. Power on: Large gyro bias uncertainty
 *          2. First 10-30 seconds: EKF estimates gyro biases from accelerometer
 *          3. Attitude covariance reduces as biases converge
 *          4. Attitude marked VALID when uncertainty below threshold
 * 
 *          Why attitude validity is critical:
 *          - All flight modes depend on accurate attitude
 *          - Bad attitude = incorrect motor mixing = immediate crash
 *          - Uncalibrated gyro biases cause slow attitude drift
 *          - Drift accumulates during flight, causes loss of position hold
 * 
 *          Typical failure scenarios:
 *          - Arming immediately after power-up (wait 30+ seconds)
 *          - High vibration environment (loose props, damaged motors)
 *          - IMU not calibrated or calibration corrupted
 * 
 * @return true if EKF attitude is valid and converged, false if still converging or invalid
 * 
 * @warning Do not disable this check - flying with invalid attitude will crash
 * 
 * @note This check is called from ins_checks() during pre-arm validation
 */
bool AP_Arming_Copter::pre_arm_ekf_attitude_check()
{
    // Query AHRS for attitude validity status
    // ATTITUDE_VALID flag set when:
    // - Gyro biases converged
    // - Attitude covariance below threshold
    // - IMU data quality sufficient
    return AP::ahrs().has_status(AP_AHRS::Status::ATTITUDE_VALID);
}

#if HAL_PROXIMITY_ENABLED
/**
 * @brief Verify no obstacles detected too close to vehicle before arming
 * 
 * @details Uses proximity sensors (lidar, ultrasonic, radar) to detect nearby obstacles
 *          and prevent arming if objects are dangerously close. This safety check prevents
 *          immediate collision on takeoff if vehicle is too close to walls, people, or
 *          other obstacles.
 * 
 *          Proximity sensor types supported:
 *          - 360° lidar (LightWare SF40, RPLidar, etc.)
 *          - Multi-directional rangefinders
 *          - Ultrasonic sensor arrays
 *          - Radar sensors
 * 
 *          Detection zones:
 *          - 360° coverage typical (sensor dependent)
 *          - Reports closest object in each direction sector
 *          - Distance and angle of closest approach
 * 
 *          Safety threshold:
 *          - Minimum clearance: 0.6 meters (60cm) default
 *          - Provides safety margin for propeller wash effects
 *          - Accounts for vehicle movement during arming/takeoff
 *          - Prevents damage to nearby objects from prop wash
 * 
 *          Common scenarios triggering this check:
 *          - Vehicle parked too close to wall or obstacle
 *          - People standing near vehicle
 *          - Indoor flight in confined space
 *          - Vehicle positioned under low ceiling/overhang
 * 
 * @param[in] display_failure If true, send error messages to GCS
 * 
 * @return true if no obstacles within minimum clearance or proximity disabled, false if obstacle too close
 * 
 * @warning Disabling this check risks collision on takeoff - props spinning near obstacles
 *          can cause damage to vehicle, obstacles, or people
 * 
 * @note Only checked when object avoidance system is enabled (OA_TYPE > 0)
 */
bool AP_Arming_Copter::proximity_checks(bool display_failure) const
{
    // Call parent class proximity sensor health checks
    // Validates sensors are functioning and providing data
    if (!AP_Arming::proximity_checks(display_failure)) {
        return false;
    }

    // Proximity clearance check can be disabled via ARMING_CHECK parameter
    if (!check_enabled(Check::PARAMETERS)) {
        return true;
    }

    // MINIMUM CLEARANCE VALIDATION
    // Check for obstacles within safety margin if avoidance system active
#if AP_AVOIDANCE_ENABLED
    float angle_deg, distance;
    
    // Only check clearance if object avoidance is enabled
    // No point checking proximity if we're not using it for avoidance
    if (copter.avoid.proximity_avoidance_enabled() && 
        copter.g2.proximity.get_closest_object(angle_deg, distance)) {
        
        // SAFETY MARGIN: 60cm minimum clearance
        // 
        // Why 60cm threshold:
        // - Propeller diameter typically 25-50cm
        // - Prop wash extends beyond prop radius
        // - Vehicle may shift position during arming sequence
        // - Provides margin for GPS drift in position modes
        // - Prevents accidental contact during takeoff climb
        const float tolerance = 0.6f;  // 0.6 meters = 60 centimeters
        
        if (distance <= tolerance) {
            // Obstacle too close - report direction and distance
            check_failed(Check::PARAMETERS, display_failure, 
                        "Proximity %d deg, %4.2fm (want > %0.1fm)", 
                        (int)angle_deg, (double)distance, (double)tolerance);
            return false;
        }
    }
#endif

    return true;
}
#endif  // HAL_PROXIMITY_ENABLED

/**
 * @brief Perform mandatory navigation and EKF health checks
 * 
 * @details These checks are ALWAYS run regardless of ARMING_CHECK parameter settings.
 *          They validate the core navigation system health required for safe flight.
 *          Failure of these checks indicates fundamental problems that make flight unsafe.
 * 
 *          Mandatory checks performed:
 *          1. AHRS/EKF initialization and health
 *          2. Position estimate availability (if required by mode or fence)
 *          3. GPS glitch detection
 *          4. EKF innovation variance threshold checking
 * 
 *          Why these checks are mandatory:
 *          - Cannot maintain altitude without height estimate
 *          - Cannot hold position without position estimate
 *          - High EKF variance indicates sensor fusion problems
 *          - GPS glitching causes position jumps and flyaways
 *          - AHRS failure means no attitude/heading reference
 * 
 *          Position estimate sources:
 *          - GPS + EKF (outdoor flight)
 *          - Optical flow + rangefinder (indoor flight)
 *          - Visual odometry (T265, etc.)
 *          - External positioning system (Vicon, etc.)
 * 
 * @param[in] display_failure If true, send error messages to GCS
 * 
 * @return true if all mandatory checks pass, false if any critical issue detected
 * 
 * @warning These checks cannot be disabled - they protect against catastrophic failures
 * 
 * @note Called from both normal pre-arm checks and mandatory_checks() when ARMING_CHECK=0
 */
bool AP_Arming_Copter::mandatory_gps_checks(bool display_failure)
{
    // Determine if current flight mode requires position estimate
    bool mode_requires_gps = copter.flightmode->requires_GPS();

    // AHRS/EKF HEALTH AND INITIALIZATION CHECK
    // Validates core navigation system is ready
    // 
    // Checks performed by ahrs.pre_arm_check():
    // - EKF initialized and running
    // - Attitude estimate valid
    // - Height estimate available (if not manual throttle mode)
    // - Position estimate available (if GPS mode)
    // - Velocity estimate available (if GPS mode)
    // - EKF healthy and not in coast mode
    const auto &ahrs = AP::ahrs();
    char failure_msg[100] = {};
    if (!ahrs.pre_arm_check(mode_requires_gps, failure_msg, sizeof(failure_msg))) {
        check_failed(display_failure, "AHRS: %s", failure_msg);
        return false;
    }

    // Check if geofencing requires position estimate
    bool fence_requires_gps = false;
#if AP_FENCE_ENABLED
    // Circular and polygon fences need GPS position
    // Altitude-only fence works with barometer
    fence_requires_gps = (copter.fence.get_enabled_fences() & (AC_FENCE_TYPE_CIRCLE | AC_FENCE_TYPE_POLYGON)) > 0;
#endif

    // POSITION ESTIMATE AVAILABILITY CHECK
    // Verify position estimate available when needed
    if (mode_requires_gps || require_location == RequireLocation::YES) {
        if (!copter.position_ok()) {
            // Vehicle-level position estimate check failed
            // Could be: No GPS lock, poor GPS quality, EKF not converged,
            // optical flow not working, external positioning system offline
            check_failed(display_failure, "Need Position Estimate");
            return false;
        }
    } else if (fence_requires_gps) {
        if (!copter.position_ok()) {
            // Clarify GPS needed for geofencing even in non-GPS flight mode
            // Example: User in Stabilize mode but has polygon fence enabled
            check_failed(display_failure, "Fence enabled, need position estimate");
            return false;
        }
    } else {
        // Position estimate not required - allow arming
        // Flight modes like Stabilize, AltHold, Acro don't need position
        return true;
    }

    // GPS GLITCH DETECTION
    // EKF monitors GPS for sudden position jumps indicating problems
    // 
    // GPS glitching causes:
    // - Multipath interference (reflections from buildings)
    // - Satellite constellation changes
    // - Ionospheric disturbances
    // - GPS receiver malfunction
    // - EMI from motors/ESCs
    // 
    // Why prevent arming during glitch:
    // - Glitch during flight causes sudden position jump
    // - Vehicle will try to correct perceived position error
    // - Results in aggressive unwanted movement (flyaway)
    nav_filter_status filt_status;
    if (ahrs.get_filter_status(filt_status)) {
        if (filt_status.flags.gps_glitching) {
            check_failed(display_failure, "GPS glitching");
            return false;
        }
    }

    // EKF INNOVATION VARIANCE THRESHOLD CHECK
    // Innovation = difference between EKF prediction and sensor measurement
    // High variance indicates sensor disagreement or EKF convergence problems
    // 
    // Variance threshold (FS_EKF_THRESH parameter):
    // - Default: 0.8 (conservative - trigger failsafe early)
    // - Range: 0.6 to 1.0 typically
    // - Set to 0.0 to disable variance checking
    // 
    // What each variance indicates:
    // - Compass variance: Magnetometer vs GPS/gyro heading disagreement
    // - Position variance: GPS position vs dead-reckoning uncertainty  
    // - Velocity variance: GPS velocity vs accelerometer integration error
    // - Height variance: Barometer vs GPS altitude disagreement
    if (copter.g.fs_ekf_thresh > 0.0f) {
        float vel_variance, pos_variance, hgt_variance, tas_variance;
        Vector3f mag_variance;
        ahrs.get_variances(vel_variance, pos_variance, hgt_variance, mag_variance, tas_variance);
        
        // Define variance checks with descriptive names
        const struct {
            const char *name;
            float value;
        } variances[] {
            { "compass", mag_variance.length() },  // Magnetometer heading uncertainty
            { "position", pos_variance },          // Horizontal position uncertainty
            { "velocity", vel_variance },          // Velocity estimate uncertainty
            { "height", hgt_variance },            // Altitude estimate uncertainty
        };
        
        // Check each variance against threshold
        for (auto &variance : variances) {
            if (variance.value < copter.g.fs_ekf_thresh) {
                continue;  // This variance OK, check next
            }
            // Variance exceeds threshold - EKF not converged or sensor problem
            check_failed(display_failure, "EKF %s variance", variance.name);
            return false;
        }
    }

    // ALL MANDATORY CHECKS PASSED
    return true;
}

/**
 * @brief Verify GCS (Ground Control Station) connection is active
 * 
 * @details Prevents arming if GCS failsafe is active, indicating loss of telemetry link
 *          to ground station. GCS failsafe triggers when no MAVLink heartbeat received
 *          from ground station for the timeout period (FS_GCS_TIMEOUT).
 * 
 *          GCS failsafe conditions:
 *          - No MAVLink heartbeat from GCS for FS_GCS_TIMEOUT seconds (default: 5s)
 *          - Telemetry radio link lost or degraded
 *          - Ground station software crashed or disconnected
 *          - Serial port or USB cable issue
 * 
 *          Why prevent arming during GCS failsafe:
 *          - May indicate communication system failure
 *          - User should investigate why telemetry lost before flight
 *          - Prevents arm/takeoff without ability to monitor vehicle
 *          - Ensures pilot can see vehicle status and warnings
 * 
 *          GCS failsafe behavior (FS_GCS_ENABLE):
 *          - 0 = Disabled (no action on GCS loss)
 *          - 1 = RTL (return to launch)
 *          - 3 = SmartRTL or RTL
 *          - 4 = SmartRTL or Land
 *          - 5 = Auto (continue mission)
 *          - 6 = Auto DO_LAND_START (land mission segment)
 *          - 7 = Brake (hold position)
 * 
 * @param[in] display_failure If true, send error messages to GCS
 * 
 * @return true if GCS connection active, false if GCS failsafe triggered
 * 
 * @note This check ensures telemetry available during pre-flight even if failsafe
 *       action is configured for in-flight GCS loss
 */
bool AP_Arming_Copter::gcs_failsafe_check(bool display_failure)
{
    // Check GCS failsafe flag - set when no heartbeat received
    if (copter.failsafe.gcs) {
        check_failed(display_failure, "GCS failsafe on");
        return false;
    }
    return true;
}

/**
 * @brief Validate winch system configuration and health
 * 
 * @details Verifies that the winch system (for payload delivery, cargo lifting, etc.)
 *          is properly configured and safe to operate. Winch systems control cables
 *          that can become entangled or cause vehicle instability if misconfigured.
 * 
 *          Winch system components:
 *          - Motor driver (servo output or PWM)
 *          - Cable length monitoring (encoder or time-based)
 *          - Tension/load sensing (optional)
 *          - Emergency release mechanism
 * 
 *          Winch configuration parameters (WINCH_*):
 *          - WINCH_TYPE: Backend type (servo, PWM, DSHOT, etc.)
 *          - WINCH_RATE_MAX: Maximum deployment/retraction rate
 *          - WINCH_POS_MIN/MAX: Cable length limits
 *          - WINCH_RATE_DN/UP: Descent and ascent rates
 * 
 *          Validation checks performed:
 *          - Winch backend properly initialized
 *          - Control output channel configured and valid
 *          - Cable length limits within reasonable ranges
 *          - Rate limits configured safely
 *          - No hardware faults detected
 * 
 *          Common failure modes:
 *          - No servo output channel assigned
 *          - Position limits inverted (min > max)
 *          - Rate limits too high for motor capability
 *          - Backend initialization failed
 * 
 * @param[in] display_failure If true, send error messages to GCS
 * 
 * @return true if winch configured correctly or not used, false if validation fails
 * 
 * @note Only validates configuration - does not check real-time winch state
 */
bool AP_Arming_Copter::winch_checks(bool display_failure) const
{
#if AP_WINCH_ENABLED
    // Winch check can be disabled via ARMING_CHECK parameter
    if (!check_enabled(Check::PARAMETERS)) {
        return true;
    }

    // Get winch singleton instance
    const AP_Winch *winch = AP::winch();
    if (winch == nullptr) {
        // Winch not instantiated - no winch configured
        return true;
    }
    
    // Perform winch-specific pre-arm validation
    // Checks backend configuration, output channels, and parameter sanity
    char failure_msg[100] = {};
    if (!winch->pre_arm_check(failure_msg, sizeof(failure_msg))) {
        check_failed(display_failure, "%s", failure_msg);
        return false;
    }
#endif
    return true;
}

/**
 * @brief Validate altitude estimate availability for non-manual throttle modes
 * 
 * @details Verifies that a reliable altitude estimate is available from the EKF before
 *          allowing arming in modes that control throttle automatically. Without a valid
 *          altitude estimate, altitude-hold and position-hold modes cannot maintain height.
 * 
 *          Altitude estimate sources:
 *          - Barometer (primary for altitude hold)
 *          - GPS altitude (less accurate, used for consistency checking)
 *          - Rangefinder (terrain-relative altitude)
 *          - External navigation system
 * 
 *          Modes requiring altitude estimate:
 *          - AltHold: Maintains specific altitude
 *          - Loiter: Holds position (includes altitude hold)
 *          - PosHold: Similar to Loiter
 *          - Auto: Waypoint navigation with altitude control
 *          - Guided: External position control
 *          - RTL: Return with altitude control
 *          - All GPS-based autonomous modes
 * 
 *          Modes NOT requiring altitude estimate (manual throttle):
 *          - Stabilize: Pilot directly controls throttle
 *          - Acro: Full manual control
 *          - Sport: Manual throttle with attitude limits
 * 
 *          Why altitude estimate is critical:
 *          - Without valid altitude, altitude-hold modes will drift up or down
 *          - Vehicle may descend into ground or obstacles
 *          - Or ascend uncontrollably until battery depleted
 *          - EKF uses barometer + GPS + rangefinder fusion for accuracy
 * 
 *          EKF altitude validity criteria:
 *          - Barometer providing consistent readings
 *          - EKF height variance below threshold
 *          - Vertical velocity estimate available
 *          - No recent EKF resets or divergence
 * 
 * @param[in] display_failure If true, send error messages to GCS
 * 
 * @return true if altitude estimate valid or not required, false if needed but unavailable
 * 
 * @warning Flying altitude-hold modes without valid altitude estimate causes
 *          uncontrolled altitude drift - often results in crash
 * 
 * @note Manual throttle modes (Stabilize, Acro) bypass this check since pilot
 *       directly controls altitude
 */
bool AP_Arming_Copter::alt_checks(bool display_failure)
{
    // ALTITUDE ESTIMATE REQUIREMENT CHECK
    // Only required for modes with automatic throttle control
    // 
    // has_manual_throttle() returns true for:
    // - Stabilize, Acro, Sport modes
    // 
    // has_manual_throttle() returns false for:
    // - AltHold, Loiter, PosHold, Auto, Guided, RTL, etc.
    if (!copter.flightmode->has_manual_throttle() && !copter.ekf_alt_ok()) {
        check_failed(display_failure, "Need Alt Estimate");
        return false;
    }

    return true;
}

/**
 * @brief Perform final realtime safety checks immediately before arming
 * 
 * @details Last-second validation performed right before motors arm. These checks verify
 *          current vehicle state is safe for arming, unlike pre-arm checks which validate
 *          configuration. Arm checks focus on transient conditions: vehicle attitude,
 *          current stick positions, safety switch state.
 * 
 *          Key differences from pre-arm checks:
 *          - Pre-arm: Configuration validation (parameters, calibration, GPS quality)
 *          - Arm: Current state validation (attitude, stick position, safety switch)
 *          - Pre-arm: Run continuously, cached result
 *          - Arm: Run once immediately before arming
 * 
 *          Arming methods:
 *          - RUDDER: Stick command (yaw right + throttle down)
 *          - MAVLINK: GCS arm command
 *          - SWITCH: RC auxiliary switch
 *          - SCRIPTING: Lua script arm command
 * 
 *          Safety checks performed:
 *          1. AHRS healthy and providing attitude
 *          2. Compass healthy (if used for heading)
 *          3. Flight mode allows arming
 *          4. Vehicle not leaning beyond ANGLE_MAX
 *          5. No ADS-B collision threat
 *          6. Throttle stick in safe position
 *          7. Hardware safety switch enabled (if present)
 * 
 * @param[in] method Arming method (RUDDER, MAVLINK, SWITCH, SCRIPTING)
 * 
 * @return true if safe to arm, false if any check fails
 * 
 * @warning This function has side effects - starts logging if arming succeeds
 * 
 * @note Always displays failures to user (display_failure=true) since this is
 *       the final gate before arming
 */
bool AP_Arming_Copter::arm_checks(AP_Arming::Method method)
{
    const auto &ahrs = AP::ahrs();

    // AHRS HEALTH CHECK
    // Verify navigation system providing valid attitude estimate
    // More stringent than pre-arm check - needs to be healthy RIGHT NOW
    if (!ahrs.healthy()) {
        check_failed(true, "AHRS not healthy");
        return false;
    }

#ifndef ALLOW_ARM_NO_COMPASS
    // COMPASS HEALTH CHECK (if compass used for heading)
    // 
    // Compass health requirements:
    // - Compass initialized and calibrated
    // - Recent valid readings within timeout
    // - No magnetic field anomalies detected
    // - Compass consistent with other sensors
    // 
    // Skip if using GPS or external heading source instead of compass
    if (!ahrs.using_noncompass_for_yaw()) {
        const Compass &_compass = AP::compass();
        if (!_compass.healthy()) {
            check_failed(true, "Compass not healthy");
            return false;
        }
    }
#endif

    // FLIGHT MODE ARMING PERMISSION CHECK
    // Each mode decides if it allows arming based on:
    // - Mode's inherent armability (some modes can't be armed directly)
    // - Current mode state readiness
    // - Arming method compatibility (some modes only arm via specific methods)
    // 
    // Modes that typically don't allow arming:
    // - RTL (use Auto Disarmed → RTL transition instead)
    // - Brake (emergency mode, enter from other modes)
    // - Avoid_ADSB (collision avoidance mode)
    if (!copter.flightmode->allows_arming(method)) {
        check_failed(true, "%s mode not armable", copter.flightmode->name());
        return false;
    }

    // BYPASS REMAINING CHECKS IF ARMING_CHECK=0
    // User has explicitly disabled arming safety checks
    if (checks_to_perform == 0) {
        return true;
    }

    // VEHICLE LEAN ANGLE CHECK
    // Prevent arming if vehicle tilted beyond maximum configured angle
    // 
    // Why this check matters:
    // - Vehicle tilted = motors immediately produce unbalanced thrust when armed
    // - Can cause sudden movement on arming (vehicle jumps/tips)
    // - Indicates vehicle on unstable surface or improper placement
    // - May indicate mechanical problem (bent frame, damaged motor mount)
    // 
    // Calculation:
    // - cos_roll * cos_pitch = cosine of total lean angle from vertical
    // - acos() recovers lean angle
    // - Compare to ANGLE_MAX parameter (degrees)
    // 
    // Typical ANGLE_MAX:
    // - Copter: 30-45° for flight control
    // - But arming lean check should be much stricter (< 5-10°)
    if (check_enabled(Check::INS)) {
        if (degrees(acosf(ahrs.cos_roll()*ahrs.cos_pitch()))*100.0f > copter.aparm.angle_max) {
            check_failed(Check::INS, true, "Leaning");
            return false;
        }
    }

    // ADS-B COLLISION THREAT CHECK (final verification)
#if HAL_ADSB_ENABLED
    if (check_enabled(Check::PARAMETERS)) {
        // Re-check ADS-B threat immediately before arming
        // Aircraft could have moved closer since pre-arm check
        if (copter.failsafe.adsb) {
            check_failed(Check::PARAMETERS, true, "ADSB threat detected");
            return false;
        }
    }
#endif

    // THROTTLE/COLLECTIVE STICK POSITION CHECK
    if (check_enabled(Check::RC)) {
#if FRAME_CONFIG == HELI_FRAME
        const char *rc_item = "Collective";
#else
        const char *rc_item = "Throttle";
#endif
        // CRITICAL SAFETY: Prevent arming with high throttle/collective
        // 
        // Exception: Allow GCS/scripting arm in Guided or Auto with high throttle
        // This supports automated takeoff sequences where vehicle arms at
        // non-zero throttle then immediately begins takeoff climb.
        // 
        // Normal arming: Throttle must be low
        // Automated arming (Guided/Auto): Throttle can be high
        if (!((AP_Arming::method_is_GCS(method) || method == AP_Arming::Method::SCRIPTING) && 
              copter.flightmode->allows_GCS_or_SCR_arming_with_throttle_high())) {
            
            // Check 1: Throttle above deadband top = always too high
            // get_pilot_desired_climb_rate() returns positive when throttle above mid-stick
            if (copter.get_pilot_desired_climb_rate() > 0.0f) {
                check_failed(Check::RC, true, "%s too high", rc_item);
                return false;
            }
            
            // Check 2: Manual throttle modes require throttle at absolute zero
            // In Stabilize, Acro, Sport, Drift - throttle maps directly to motors
            // Any throttle = immediate motor spin = vehicle jumps on arming
#if FRAME_CONFIG != HELI_FRAME
            if ((copter.flightmode->has_manual_throttle() || 
                 copter.flightmode->mode_number() == Mode::Number::DRIFT) && 
                copter.channel_throttle->get_control_in() > 0) {
                check_failed(Check::RC, true, "%s too high", rc_item);
                return false;
            }
#endif
        }
    }

    // HARDWARE SAFETY SWITCH CHECK
    // Physical switch on flight controller or GPS module that must be pushed
    // before arming is allowed. Common on Pixhawk-based boards.
    // 
    // Safety switch states:
    // - SAFETY_DISARMED: Switch not pressed - arming blocked
    // - SAFETY_ARMED: Switch pressed - arming allowed
    // - SAFETY_NONE: No safety switch present
    // 
    // Purpose:
    // - Prevents accidental arming from software/GCS commands
    // - Requires physical presence at vehicle to arm
    // - Compliance with some regulatory requirements
    // - Additional protection against software bugs
    if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        check_failed(true, "Safety Switch");
        return false;
    }

    // CALL PARENT CLASS ARM CHECKS
    // Performs additional arming validation in AP_Arming base class
    // 
    // MUST be called last because it has side effects:
    // - Starts logging
    // - Sets armed state flags
    // - Initializes arming timestamp
    // 
    // If any check above fails, these side effects must not occur
    return AP_Arming::arm_checks(method);
}

/**
 * @brief Execute minimum safety checks when user has disabled standard arming checks
 * 
 * @details These checks are enforced even when ARMING_CHECK parameter is set to 0 or
 *          arming is forced. They represent the absolute minimum validation required
 *          to prevent immediate catastrophic failure on arming.
 * 
 *          Use case for disabling arming checks:
 *          - Advanced users in controlled test environments
 *          - Recovery from sensor failures in emergency situations
 *          - Testing new hardware without full sensor suite
 *          - Competition flying where rapid arming needed
 * 
 *          Mandatory checks that cannot be bypassed:
 *          - AHRS/EKF health and convergence
 *          - Position estimate availability (if GPS mode or fence enabled)
 *          - GPS glitch detection
 *          - EKF innovation variance thresholds
 *          - Altitude estimate availability (if non-manual throttle mode)
 * 
 *          Checks bypassed when ARMING_CHECK=0:
 *          - GPS HDOP quality
 *          - Compass calibration and health
 *          - Barometer consistency
 *          - RC calibration
 *          - Parameter sanity checks
 *          - Battery voltage warnings
 *          - Rangefinder health
 * 
 * @param[in] display_failure If true, send error messages to GCS
 * 
 * @return true if minimum safety requirements met, false if critical failure detected
 * 
 * @warning Setting ARMING_CHECK=0 is extremely dangerous - only use in controlled
 *          test environments. Vehicle may arm with miscalibrated sensors, poor GPS,
 *          or parameter errors that will cause crashes.
 * 
 * @note Updates notification flags for LED/buzzer feedback
 */
bool AP_Arming_Copter::mandatory_checks(bool display_failure)
{
    // MANDATORY GPS/NAVIGATION CHECKS
    // Always validates EKF health, position estimate availability, and variance
    // These checks prevent arming with navigation system failures
    bool result = mandatory_gps_checks(display_failure);
    
    // Update notification system for LED/buzzer status indication
    // Even with ARMING_CHECK=0, user needs GPS status feedback
    AP_Notify::flags.pre_arm_gps_check = result;

    // MANDATORY ALTITUDE ESTIMATE CHECK
    // Prevents arming in altitude-hold modes without altitude estimate
    // Critical because altitude-hold modes are uncontrollable without this
    if (!alt_checks(display_failure)) {
        result = false;
    }

    // CALL PARENT CLASS MANDATORY CHECKS
    // Validates additional critical requirements in AP_Arming base class
    // Uses bitwise & to ensure all checks run (not short-circuit evaluation)
    return result & AP_Arming::mandatory_checks(display_failure);
}

/**
 * @brief Update pre-arm check status flags for internal state and external notifications
 * 
 * @details Sets pre-arm check status in both the vehicle state structure and the
 *          notification system. This dual update ensures consistency between:
 *          - Internal logic (copter.ap.pre_arm_check) - used by arming logic
 *          - External feedback (AP_Notify::flags) - drives LEDs, buzzers, GCS
 * 
 *          Pre-arm check status affects:
 *          - Vehicle LED patterns (solid green = passed, flashing = failed)
 *          - Buzzer tones (different patterns for pass/fail)
 *          - GCS pre-arm display indicators
 *          - Logging of pre-arm check status
 * 
 *          This function called from:
 *          - pre_arm_checks() after running all validation
 *          - Flight mode changes (re-evaluate arming eligibility)
 *          - Configuration changes affecting armability
 * 
 * @param[in] b Pre-arm check status: true if all checks passed, false if any failed
 * 
 * @note This is a state setter with side effects (notification system updates)
 * @note Synchronizes internal and external status to prevent inconsistencies
 */
void AP_Arming_Copter::set_pre_arm_check(bool b)
{
    // Update internal vehicle state flag
    // Used by arming logic to determine if arming attempts should be allowed
    copter.ap.pre_arm_check = b;
    
    // Update notification system flag
    // Drives external feedback (LEDs, buzzers, GCS displays)
    AP_Notify::flags.pre_arm_check = b;
}

/**
 * @brief Arm the vehicle motors and initialize flight-ready state
 * 
 * @details Complete arming sequence that transitions vehicle from disarmed to armed state.
 *          This is the final step after all pre-arm and arm checks pass. Performs extensive
 *          initialization and state setup required for safe flight.
 * 
 *          Arming sequence overview:
 *          1. Validate not already in arming process (prevent reentrancy)
 *          2. Return success if already armed
 *          3. Run arm checks if enabled (AHRS health, compass, mode, lean, throttle)
 *          4. Enable logging and arming notifications
 *          5. Temporarily disable CPU failsafe during initialization
 *          6. Initialize navigation references (home, bearing, altitude datum)
 *          7. Initialize SmartRTL path recording
 *          8. Output motor minimum values
 *          9. Set motors armed flag
 *          10. Re-enable failsafe monitoring
 *          11. Start arming delay timer
 * 
 *          Home position initialization logic:
 *          - If home not set: Reset EKF altitude datum, set arming altitude to zero
 *          - If home set but not locked: Update home to current position
 *          - If home locked: Preserve existing home, record arming altitude offset
 * 
 *          Arming delay period:
 *          - Brief delay after arming before full flight control active
 *          - Allows motors to spin up smoothly
 *          - Prevents sudden movements on arming
 *          - Configurable via ARMING_DELAY parameter
 * 
 * @param[in] method How arming was requested (RUDDER, MAVLINK, SWITCH, SCRIPTING)
 * @param[in] do_arming_checks If true, perform arm_checks() validation; if false, skip checks
 *                             (force arming - dangerous, only used in specific recovery scenarios)
 * 
 * @return true if armed successfully, false if checks failed or already arming
 * 
 * @warning This function has extensive side effects:
 *          - Starts logging
 *          - Initializes EKF altitude datum
 *          - Sets home position
 *          - Configures motor outputs
 *          - Updates notification flags
 *          - Records arming time
 * 
 * @note Uses static reentrancy guard to prevent nested calls during initialization
 * @note CPU failsafe temporarily disabled because initialization takes significant time
 * @note Notification update called 10 times to ensure message propagates to all systems
 */
bool AP_Arming_Copter::arm(const AP_Arming::Method method, const bool do_arming_checks)
{
    // REENTRANCY GUARD
    // Prevent nested calls to arm() during the initialization sequence
    // Arming initialization can take 100-200ms - during this time additional
    // arm requests must be rejected to prevent state corruption
    static bool in_arm_motors = false;

    if (in_arm_motors) {
        return false;
    }
    in_arm_motors = true;

    // ALREADY ARMED CHECK
    // Return success if motors already armed
    // Common case: User holds rudder arm stick, function called repeatedly
    if (copter.motors->armed()) {
        in_arm_motors = false;
        return true;
    }

    // PARENT CLASS ARM CHECKS AND VALIDATION
    // Performs arm_checks() and additional validation in AP_Arming base class
    // This is where the actual safety checks happen
    // If checks fail, abort arming process and notify user
    if (!AP_Arming::arm(method, do_arming_checks)) {
        AP_Notify::events.arming_failed = true;
        in_arm_motors = false;
        return false;
    }

#if HAL_LOGGING_ENABLED
    // ENABLE LOGGING SUBSYSTEM
    // Notify logger that vehicle is armed - may open log files, start recording
    // Critical for post-flight analysis and debugging
    AP::logger().set_vehicle_armed(true);
#endif

    // TEMPORARILY DISABLE CPU FAILSAFE
    // Arming initialization takes 100-200ms which can trigger CPU failsafe
    // Safe to disable because:
    // - Vehicle still on ground
    // - Motors not yet spinning
    // - Re-enabled at end of function
    copter.failsafe_disable();

    // EARLY ARMED NOTIFICATION
    // Set armed flag early and update notification system multiple times
    // Ensures LEDs, buzzers, and GCS receive armed status before motors spin
    // Gives pilot clear indication that arming sequence has started
    AP_Notify::flags.armed = true;
    
    // Call notify update 10 times to propagate message through all systems
    // Multiple calls ensure message not lost in notification buffers
    for (uint8_t i=0; i<=10; i++) {
        AP::notify().update();
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // SITL-specific status text (helpful for automated testing)
    send_arm_disarm_statustext("Arming motors");
#endif

    // INITIALIZE SIMPLE/SUPER SIMPLE MODE BEARING
    // Simple mode uses heading at arming as reference for pilot input translation
    // Must be recorded before first flight
    copter.init_simple_bearing();

    auto &ahrs = AP::ahrs();

    // Record absolute heading at arming (radians)
    // Used by simple/super-simple modes for pilot input frame transformation
    copter.initial_armed_bearing_rad = ahrs.get_yaw_rad();

    // HOME POSITION AND ALTITUDE DATUM INITIALIZATION
    // Three cases handled differently based on home status:
    
    if (!ahrs.home_is_set()) {
        // CASE 1: HOME NOT SET - First arming or home lost
        // 
        // Reset EKF altitude datum to current position
        // After reset, EKF altitude will be zero at arming location
        // This provides relative altitude reference for initial flights
        // 
        // Why reset altitude datum:
        // - Provides known altitude reference for modes using altitude
        // - Alt-hold mode needs altitude reference to function
        // - Position modes (Loiter, Auto) need vertical position reference
        // - GPS altitude can have significant bias/error
        ahrs.resetHeightDatum();
        LOGGER_WRITE_EVENT(LogEvent::EKF_ALT_RESET);

        // Arming altitude is zero because we just reset datum to current position
        copter.arming_altitude_m = 0;
        
    } else if (!ahrs.home_is_locked()) {
        // CASE 2: HOME SET BUT NOT LOCKED
        // 
        // Home was set (from GPS or manual) but not locked
        // Unlocked home can be updated as better position estimate becomes available
        // Update home to current position since this is confirmed arm location
        // 
        // Home locking:
        // - Unlocked: Home updates as vehicle moves (pre-arm)
        // - Locked: Home position fixed (post-arm or manual lock)
        if (!copter.set_home_to_current_location(false)) {
            // Ignore failure - home update is not critical for arming
            // Vehicle can still fly with previous home position
        }

        // Record altitude offset between arming position and home altitude
        // Used for:
        // - Return to launch altitude calculations
        // - Land detector (detect if vehicle has descended to arming altitude)
        // - Altitude displays showing altitude above arming point
        float pos_d_m = 0;
        UNUSED_RESULT(AP::ahrs().get_relative_position_D_origin_float(pos_d_m));
        copter.arming_altitude_m = -pos_d_m;  // Negative because D is down
        
    }
    // CASE 3: HOME LOCKED (implicit else)
    // Home position is locked - don't update it
    // Record arming altitude offset for relative altitude calculations
    // (arming_altitude_m already updated from else if branch)
    
    // UPDATE SUPER SIMPLE MODE BEARING
    // Super simple mode needs current position relative to home
    // Call with force=false to only update if home is set
    copter.update_super_simple_bearing(false);

    // INITIALIZE SMARTRTL PATH RECORDING
    // SmartRTL mode records path during flight for intelligent return
    // Set home/starting point for path recording at arming location
#if MODE_SMARTRTL_ENABLED
    // Pass position_ok() to indicate if we have valid position estimate
    // SmartRTL requires position estimate to function properly
    copter.g2.smart_rtl.set_home(copter.position_ok());
#endif

    // SET SOFT ARMED FLAG IN HAL
    // Notify hardware abstraction layer that vehicle is armed
    // Used by HAL for safety-critical hardware management:
    // - LED patterns
    // - Safety switch state
    // - Hardware watchdog configuration
    hal.util->set_soft_armed(true);

#if HAL_SPRAYER_ENABLED
    // DISABLE SPRAYER TEST MODE IF ACTIVE
    // Agricultural sprayer may be running test mode (for pre-flight check)
    // Must be disabled before flight to prevent unwanted spraying
    copter.sprayer.test_pump(false);
#endif

    // OUTPUT MINIMUM PWM VALUES TO MOTORS
    // Initialize motor outputs to minimum before arming
    // Ensures smooth motor spin-up from known state
    // Prevents sudden jumps in motor speed
    copter.motors->output_min();

    // *** ACTUALLY ARM THE MOTORS ***
    // This is the critical step - motors now spinning or ready to spin
    // After this call, throttle/collective input will control motor speed
    // Vehicle is now in flight-ready state
    copter.motors->armed(true);

#if HAL_LOGGING_ENABLED
    // LOG CURRENT FLIGHT MODE
    // Record mode at arming time in case it was changed while disarmed
    // Critical for flight log analysis and accident investigation
    AP::logger().Write_Mode((uint8_t)copter.flightmode->mode_number(), copter.control_mode_reason);
#endif

    // RE-ENABLE FAILSAFE MONITORING
    // Failsafe was disabled during initialization to prevent false triggers
    // Now that initialization complete, re-enable safety monitoring
    // Monitors: Battery, RC loss, GPS loss, EKF errors, geofence, etc.
    copter.failsafe_enable();

    // TELL PERFORMANCE MONITOR TO IGNORE THIS LOOP
    // Arming initialization causes main loop delay
    // Don't count this delay in performance statistics
    // Prevents false "slow loop" warnings after arming
    AP::scheduler().perf_info.ignore_this_loop();

    // CLEAR REENTRANCY GUARD
    // Allow future arm() calls now that initialization complete
    in_arm_motors = false;

    // RECORD ARMING TIMESTAMP
    // Used for:
    // - Flight time calculation
    // - Time-based failsafes (e.g., max flight time)
    // - Log analysis
    // - Battery capacity calculations
    copter.arm_time_ms = millis();

    // START ARMING DELAY PERIOD
    // Brief delay period after arming before full flight control active
    // During delay:
    // - Prevents immediate throttle response
    // - Allows motors to stabilize
    // - Gives pilot time to verify motor spin direction
    // - Smooth transition to flight-ready state
    copter.ap.in_arming_delay = true;

    // INITIALIZE AIRMODE SWITCH FLAG
    // Assume not armed with airmode switch active
    // Will be overridden in switches.cpp if airmode switch was used
    // Affects whether airmode automatically disables on disarm
    copter.ap.armed_with_airmode_switch = false;

    // ARMING COMPLETE - SUCCESS
    return true;
}

/**
 * @brief Disarm vehicle motors and perform post-flight shutdown sequence
 * 
 * @details Complete disarming sequence that transitions vehicle from armed to disarmed state.
 *          Performs safety checks, saves learned parameters, and cleanly shuts down flight systems.
 * 
 *          Disarming safety checks:
 *          - Vehicle must be on ground (land_complete flag set)
 *          - Cannot disarm via MAVLink while flying
 *          - Rudder disarm requires manual throttle mode OR landed
 *          - Switch/button disarm allowed anytime (emergency disarm)
 * 
 *          Post-flight shutdown sequence:
 *          1. Verify disarm safety conditions met
 *          2. Call parent class disarm validation
 *          3. Save compass offsets learned by EKF
 *          4. Set land complete flags
 *          5. Send disarm command to motors (immediate shutdown)
 *          6. Reset mission state
 *          7. Notify logging system
 *          8. Clear arming delay flag
 *          9. Save autotune parameters if applicable
 * 
 *          Learned parameter saving:
 *          - Compass offsets: EKF continuously improves compass calibration during flight
 *          - Autotune results: PID gains optimized during autotune saved on disarm
 * 
 *          Emergency disarm (switch method):
 *          - Bypasses most safety checks
 *          - Immediately cuts motors
 *          - Used when vehicle must be stopped immediately
 *          - Can be triggered in air (dangerous - vehicle will fall)
 * 
 * @param[in] method How disarm was requested (RUDDER, MAVLINK, SWITCH, SCRIPTING)
 * @param[in] do_disarm_checks If true, enforce safety checks; if false, force disarm
 * 
 * @return true if disarmed successfully, false if safety checks prevent disarming
 * 
 * @warning Emergency disarm (switch method) will cut motors in flight causing crash
 * @warning Disarming while flying with MAVLink is blocked for safety
 * 
 * @note Updates compass calibration with EKF-learned offsets if enabled
 * @note Resets mission to beginning on disarm
 */
bool AP_Arming_Copter::disarm(const AP_Arming::Method method, bool do_disarm_checks)
{
    // ALREADY DISARMED CHECK
    // Return success immediately if motors already disarmed
    // Common case: User holds rudder disarm stick, function called repeatedly
    if (!copter.motors->armed()) {
        return true;
    }

    // MAVLINK DISARM IN-FLIGHT SAFETY CHECK
    // Block GCS disarm commands while flying to prevent accidental motor cutoff
    // 
    // Why this matters:
    // - Accidental GCS button press could disarm flying vehicle
    // - Network errors could send spurious disarm commands
    // - Malicious actor could attempt to disarm via telemetry
    // 
    // Land complete detection:
    // - Weight on landing gear
    // - Low vibration levels
    // - Throttle at minimum
    // - Vertical velocity near zero
    if (do_disarm_checks &&
        AP_Arming::method_is_GCS(method) &&
        !copter.ap.land_complete) {
        return false;
    }

    // RUDDER DISARM IN-FLIGHT SAFETY CHECK
    // Allow rudder disarm (yaw left + throttle down) only if:
    // - In manual throttle mode (Stabilize, Acro, Sport), OR
    // - Vehicle is landed
    // 
    // Rationale:
    // - Manual modes: Pilot has direct control, should be able to disarm
    // - Auto modes in flight: Prevent accidental stick disarm during auto flight
    // - Landed: Always allow disarm regardless of mode
    if (method == AP_Arming::Method::RUDDER) {
        if (!copter.flightmode->has_manual_throttle() && !copter.ap.land_complete) {
            return false;
        }
    }

    // PARENT CLASS DISARM VALIDATION
    // Additional disarm checks in AP_Arming base class
    // Handles disarm logging and state management
    if (!AP_Arming::disarm(method, do_disarm_checks)) {
        return false;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // SITL-specific status text (helpful for automated testing)
    send_arm_disarm_statustext("Disarming motors");
#endif

    auto &ahrs = AP::ahrs();

    // SAVE EKF-LEARNED COMPASS OFFSETS
    // During flight, EKF improves compass calibration by comparing compass heading
    // with GPS velocity vector. If enabled, save these improved offsets for future flights.
    // 
    // Compass learning process:
    // - EKF estimates magnetic field offsets during flight
    // - Compares compass readings with GPS ground track
    // - Iteratively refines offset estimates
    // - Saves back to compass parameters on disarm
    // 
    // Learn type options:
    // - NONE: Don't learn or save offsets
    // - COPY_FROM_EKF: Save EKF-learned offsets to parameters
    // - INFLIGHT: Learn continuously but don't save
    Compass &compass = AP::compass();
    if (ahrs.use_compass() && compass.get_learn_type() == Compass::LearnType::COPY_FROM_EKF) {
        // Iterate through all compass instances (primary, secondary, tertiary)
        for(uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
            Vector3f magOffsets;
            // Get learned offsets from EKF for this compass instance
            if (ahrs.getMagOffsets(i, magOffsets)) {
                // Save to parameters and EEPROM for persistence across reboots
                compass.set_and_save_offsets(i, magOffsets);
            }
        }
    }

    // SET LANDED FLAGS
    // Mark vehicle definitively on ground and not in flight
    // These flags affect:
    // - Land detector state machine
    // - Altitude controller behavior
    // - Vibration-based health monitoring
    // - Crash detection sensitivity
    copter.set_land_complete(true);        // Definitely landed
    copter.set_land_complete_maybe(true);  // Not possibly in air

    // *** DISARM MOTORS - IMMEDIATE SHUTDOWN ***
    // This is the critical step - motors stop spinning immediately
    // After this call, throttle/collective input has no effect
    // Vehicle is now in disarmed state
    copter.motors->armed(false);

#if MODE_AUTO_ENABLED
    // RESET AUTO MODE MISSION
    // Return mission to beginning for next flight
    // Prevents continuing mid-mission on next arm
    // Pilot can resume mission manually if desired
    copter.mode_auto.mission.reset();
#endif

#if HAL_LOGGING_ENABLED
    // NOTIFY LOGGER OF DISARM
    // May close log files, stop recording
    // Important for log file integrity
    AP::logger().set_vehicle_armed(false);
#endif

    // CLEAR HAL SOFT ARMED FLAG
    // Notify hardware abstraction layer that vehicle is disarmed
    // Updates hardware state: LEDs, safety switches, watchdogs
    hal.util->set_soft_armed(false);

    // CLEAR ARMING DELAY FLAG
    // No longer in arming delay period (we're disarmed)
    copter.ap.in_arming_delay = false;

#if AUTOTUNE_ENABLED
    // SAVE AUTOTUNE RESULTS IF APPLICABLE
    // If we just completed autotune, save the optimized PID gains
    // Only saves if currently in autotune mode
    // 
    // Autotune process:
    // - Performs frequency sweeps during flight
    // - Measures vehicle response
    // - Calculates optimal PID gains
    // - On disarm: Prompts to save or revert gains
    copter.mode_autotune.autotune.disarmed(copter.flightmode == &copter.mode_autotune);
#endif

    // DISARM COMPLETE - SUCCESS
    return true;
}

#pragma GCC diagnostic pop
