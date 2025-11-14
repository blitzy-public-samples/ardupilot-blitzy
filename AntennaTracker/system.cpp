/**
 * @file system.cpp
 * @brief System initialization, mode management, and EEPROM home storage for antenna tracker
 * 
 * @details This file implements the core system functionality for the AntennaTracker vehicle:
 * - Complete initialization sequence (init_ardupilot) for hardware and software subsystems
 * - Mode management and switching between tracking modes (AUTO, MANUAL, SCAN, etc.)
 * - Home location persistence using EEPROM storage
 * - Servo arming/disarming and initialization
 * 
 * The initialization process follows a specific sequence to ensure proper hardware setup
 * and sensor calibration. Home location is stored persistently to provide altitude reference
 * for tracking operations even when GPS signal is temporarily unavailable.
 * 
 * Source: AntennaTracker/system.cpp
 */

#include "Tracker.h"

// mission storage
static const StorageAccess wp_storage(StorageManager::StorageMission);

/**
 * @brief Main initialization function called on tracker boot
 * 
 * @details Implements the complete startup sequence for the antenna tracker system.
 * This function is called once during boot and initializes all hardware and software
 * subsystems in the correct order to ensure proper sensor calibration and system readiness.
 * 
 * Initialization sequence:
 * 1. Initialize notification system (LEDs, buzzer) for user feedback
 * 2. Initialize battery monitoring subsystem
 * 3. Initialize and calibrate barometer (ground pressure reference)
 * 4. Setup GCS/MAVLink communication on all configured UARTs
 * 5. Initialize compass with automatic declination calculation
 * 6. Initialize GPS receiver and configure logging
 * 7. Initialize AHRS (Attitude Heading Reference System) for orientation estimation
 * 8. Initialize IMU sensors (gyroscope and accelerometer) at configured loop rate
 * 9. Perform barometer calibration to establish ground pressure reference
 * 10. Setup data logging system with vehicle-specific startup messages
 * 11. Initialize RC input channels and configure mode switching
 * 12. Initialize servo output channels for antenna control
 * 13. Load start position from parameters or EEPROM home location
 * 14. Set initial tracking mode (from INITIAL_MODE parameter, defaults to MANUAL)
 * 15. Optionally arm servos with trim values if startup delay configured
 * 
 * @note Blocking delays during initialization prevent the scheduler from running.
 * The system is not fully operational until this function completes.
 * 
 * @warning Barometer and IMU calibration require the tracker to be stationary.
 * Movement during initialization can result in incorrect altitude reference and
 * attitude estimation errors.
 * 
 * @see Tracker::init_servos() for servo channel setup
 * @see Tracker::set_mode() for mode initialization
 * @see Tracker::get_home_eeprom() for home location loading
 * 
 * Source: AntennaTracker/system.cpp:6-83
 */
void Tracker::init_ardupilot()
{
    // Step 1: Initialize notification system (LEDs, buzzer) to provide visual/audio feedback
    // during initialization and operation
    notify.init();
    AP_Notify::flags.pre_arm_check = true;
    AP_Notify::flags.pre_arm_gps_check = true;

    // Step 2: Initialize battery monitoring to track power levels and trigger low-battery warnings
    battery.init();

    // Step 3: Initialize barometer before GCS to enable CLI baro test functionality
    // Set logging bit to include barometer data in IMU log messages
    barometer.set_log_baro_bit(MASK_LOG_IMU);
    barometer.init();

    // Step 4: Setup GCS/MAVLink communication on all configured serial ports
    gcs().setup_uarts();
    // Call update_send() early to initialize stream rates, preventing internal errors
    // if the first received packet happens to be an arm command before main loop starts
    gcs().update_send();

    // Step 5: Initialize compass (magnetometer) for heading determination
    // Configure logging and perform auto-declination calculation
    AP::compass().set_log_bit(MASK_LOG_COMPASS);
    AP::compass().init();

    // Step 6: Initialize GPS receiver for position information
    // GPS data used to track moving targets and establish home location
    gps.set_log_gps_bit(MASK_LOG_GPS);
    gps.init();

    // Step 7: Initialize AHRS (Attitude Heading Reference System)
    // Tracker doesn't use fly-forward model since it's stationary
    ahrs.init();
    ahrs.set_fly_forward(false);

    // Step 8: Initialize IMU (Inertial Measurement Unit - gyros and accelerometers)
    // Uses configured scheduler loop rate for sensor sampling
    ins.init(scheduler.get_loop_rate_hz());
    // Reset AHRS after IMU initialization to clear any transient startup values
    ahrs.reset();

    // Step 9: Calibrate barometer to establish ground pressure reference
    // This provides the altitude baseline for all subsequent altitude measurements
    barometer.calibrate();

#if HAL_LOGGING_ENABLED
    // Step 10: Initialize data logging system with vehicle-specific startup message writer
    logger.setVehicle_Startup_Writer(FUNCTOR_BIND(&tracker, &Tracker::Log_Write_Vehicle_Startup_Messages, void));
#endif

    // Step 11: Initialize RC (Radio Control) input channels
    // Convert deprecated ARMDISARM_UNUSED to current ARMDISARM function
    rc().convert_options(RC_Channel::AUX_FUNC::ARMDISARM_UNUSED, RC_Channel::AUX_FUNC::ARMDISARM);
    rc().init();

    // Step 12: Initialize servo output channels for yaw and pitch antenna control
    init_servos();

    // Step 13: Load tracker home location from parameters or EEPROM
    // Use START_LATITUDE/START_LONGITUDE parameters if valid (useful for indoor testing
    // or when GPS lock is not available). Coordinates in decimal degrees.
    if (fabsf(g.start_latitude) <= 90.0f && fabsf(g.start_longitude) <= 180.0f) {
        // Convert decimal degrees to 1e7 integer format (ArduPilot standard)
        current_loc.lat = g.start_latitude * 1.0e7f;
        current_loc.lng = g.start_longitude * 1.0e7f;
    } else {
        gcs().send_text(MAV_SEVERITY_NOTICE, "Ignoring invalid START_LATITUDE or START_LONGITUDE parameter");
    }

    // If no valid start position from parameters, attempt to load from EEPROM
    // EEPROM stores the last known home location for persistence across reboots
    if (current_loc.lat == 0 && current_loc.lng == 0) {
        get_home_eeprom(current_loc);
    }

    // Delay for hardware stabilization (legacy delay, purpose unclear - preserved for compatibility)
    hal.scheduler->delay(1000); // Why????

    // Step 14: Set initial tracking mode from INITIAL_MODE parameter
    // Defaults to MANUAL mode if configured mode is invalid or unavailable
    Mode *newmode = mode_from_mode_num((Mode::Number)g.initial_mode.get());
    if (newmode == nullptr) {
        newmode = &mode_manual;
    }
    set_mode(*newmode, ModeReason::STARTUP);

    // Step 15: If startup delay configured, arm servos with trim values
    // Some servos require power-on initialization time before accepting commands
    if (g.startup_delay > 0) {
        // arm servos with trim value to allow them to start up (required
        // for some servos)
        prepare_servos();
    }
}

/**
 * @brief Reads home location from EEPROM persistent storage
 * 
 * @details Retrieves the stored home location from EEPROM using the mission storage
 * area. The home location is stored at fixed offsets within the waypoint storage space:
 * - Offset 1: Altitude (4 bytes, int32_t in centimeters)
 * - Offset 5: Latitude (4 bytes, int32_t in degrees * 1e7)
 * - Offset 9: Longitude (4 bytes, int32_t in degrees * 1e7)
 * 
 * The home location serves as the altitude reference and starting position for tracking
 * operations. It persists across power cycles and is especially useful when GPS signal
 * is temporarily unavailable during startup.
 * 
 * @param[out] loc Location structure to populate with home coordinates
 * 
 * @return true if valid home location was loaded from EEPROM
 * @return false if EEPROM is empty (command_total == 0) or home not previously saved
 * 
 * @note Home location is used as altitude reference for all tracking calculations.
 * Without a valid home location, altitude-based tracking may be inaccurate.
 * 
 * @see Tracker::set_home_eeprom() for writing home location to EEPROM
 * @see StorageAccess for EEPROM storage abstraction
 * 
 * Source: AntennaTracker/system.cpp:88-105
 */
bool Tracker::get_home_eeprom(Location &loc) const
{
    // Check if any locations have been saved to EEPROM
    // command_total indicates number of stored waypoints (0 or 1 for tracker)
    if (g.command_total.get() == 0) {
        return false;
    }

    // Read home location from fixed EEPROM offsets in mission storage area
    // Note: Offsets are hardcoded based on waypoint storage format
    // - Byte 0: Waypoint ID (set to 0 for home)
    // - Bytes 1-4: Altitude in centimeters (int32_t)
    // - Bytes 5-8: Latitude in degrees * 1e7 (int32_t)
    // - Bytes 9-12: Longitude in degrees * 1e7 (int32_t)
    loc = {
        int32_t(wp_storage.read_uint32(5)),  // Latitude
        int32_t(wp_storage.read_uint32(9)),  // Longitude
        int32_t(wp_storage.read_uint32(1)),  // Altitude
        Location::AltFrame::ABSOLUTE         // Altitude frame (absolute MSL)
    };

    return true;
}

/**
 * @brief Writes home location to EEPROM persistent storage
 * 
 * @details Stores the provided home location to EEPROM using fixed offsets in the
 * mission storage area. The location is saved in ArduPilot's standard format with
 * coordinates in 1e7 degrees and altitude in centimeters. Once saved, this location
 * persists across power cycles and system resets.
 * 
 * Storage layout in EEPROM:
 * - Byte 0: Waypoint ID (0x00 for home location)
 * - Bytes 1-4: Altitude in centimeters (int32_t, MSL)
 * - Bytes 5-8: Latitude in degrees * 1e7 (int32_t)
 * - Bytes 9-12: Longitude in degrees * 1e7 (int32_t)
 * 
 * @param[in] temp Location structure containing home coordinates to save
 * 
 * @return true if write successful (always returns true in current implementation)
 * 
 * @warning EEPROM has limited write cycles (typically 100,000). Do not call this
 * function frequently or in loops. Only update home when location genuinely changes.
 * 
 * @note Sets command_total parameter to 1, indicating home location is stored.
 * AntennaTracker only stores a single location (home) in mission storage.
 * 
 * @see Tracker::get_home_eeprom() for reading home location from EEPROM
 * @see Tracker::set_home() for high-level home location setting with AHRS update
 * 
 * Source: AntennaTracker/system.cpp:107-117
 */
bool Tracker::set_home_eeprom(const Location &temp)
{
    // Write waypoint ID byte (0 = home location)
    wp_storage.write_byte(0, 0);
    
    // Write altitude in centimeters (int32_t) at offset 1
    wp_storage.write_uint32(1, temp.alt);
    
    // Write latitude in degrees * 1e7 (int32_t) at offset 5
    wp_storage.write_uint32(5, temp.lat);
    
    // Write longitude in degrees * 1e7 (int32_t) at offset 9
    wp_storage.write_uint32(9, temp.lng);

    // Update parameter indicating home location is now stored in EEPROM
    // AntennaTracker stores at most 1 entry (HOME) in mission storage
    g.command_total.set_and_save(1); // At most 1 entry for HOME
    return true;
}

/**
 * @brief Sets home location to current GPS position
 * 
 * @details Convenience function that sets the home location to the tracker's current
 * GPS position. This is typically called when the user wants to establish home at
 * the tracker's physical location, usually after achieving good GPS lock.
 * 
 * @param[in] lock Unused parameter (retained for API compatibility)
 * 
 * @return true if home location was successfully set and saved to EEPROM
 * @return false if GPS location invalid or EEPROM write failed
 * 
 * @note Requires valid GPS fix with sufficient accuracy
 * 
 * @see Tracker::set_home() for the underlying home setting implementation
 * @see AP_GPS::location() for current GPS position
 * 
 * Source: AntennaTracker/system.cpp:119-122
 */
bool Tracker::set_home_to_current_location(bool lock)
{
    return set_home(AP::gps().location(), lock);
}

/**
 * @brief Sets tracker home location with AHRS and EEPROM update
 * 
 * @details High-level function to set the tracker's home location. This function:
 * 1. Updates AHRS home location if EKF origin has been established
 * 2. Saves home location to EEPROM for persistence across reboots
 * 3. Updates current_loc to the new home position
 * 
 * The home location serves as:
 * - Altitude reference for all tracking calculations
 * - Starting position if GPS signal is lost
 * - Origin for relative position calculations
 * 
 * @param[in] temp Location structure containing the new home coordinates
 * @param[in] lock Unused parameter (retained for API compatibility)
 * 
 * @return true if home location was successfully set in AHRS and saved to EEPROM
 * @return false if AHRS home update failed or EEPROM write failed
 * 
 * @note If EKF origin is not yet set, this function will still save to EEPROM
 * and update current_loc, but will skip AHRS update
 * 
 * @see Tracker::set_home_eeprom() for EEPROM persistence
 * @see AP_AHRS::set_home() for AHRS home location update
 * 
 * Source: AntennaTracker/system.cpp:124-141
 */
bool Tracker::set_home(const Location &temp, bool lock)
{
    // Check if EKF origin has been established (required for AHRS home update)
    Location ekf_origin;
    if (ahrs.get_origin(ekf_origin)) {
        // EKF origin exists, update AHRS home location
        if (!ahrs.set_home(temp)) {
            return false;
        }
    }

    // Save home location to EEPROM for persistence across power cycles
    if (!set_home_eeprom(temp)) {
        return false;
    }

    // Update current location to new home position
    current_loc = temp;

    return true;
}

/**
 * @brief Arms tracker servos for active antenna control
 * 
 * @details Enables servo outputs by setting the soft-armed state. When armed, the
 * yaw and pitch servos actively control antenna pointing. The armed state is also
 * logged for analysis and safety monitoring.
 * 
 * Unlike multicopters/planes, tracker servo arming is not safety-critical since
 * antenna movement poses no flight risk. Arming primarily enables active tracking
 * versus passive/stopped state.
 * 
 * @note Armed state required for AUTO, GUIDED, and SCAN modes to actively control antenna
 * 
 * @see Tracker::disarm_servos() for disabling servo control
 * @see Tracker::set_mode() for mode-based arming decisions
 * 
 * Source: AntennaTracker/system.cpp:143-149
 */
void Tracker::arm_servos()
{
    // Enable servo control by setting soft-armed state
    hal.util->set_soft_armed(true);
#if HAL_LOGGING_ENABLED
    // Log armed state change for analysis
    logger.set_vehicle_armed(true);
#endif
}

/**
 * @brief Disarms tracker servos to stop antenna control
 * 
 * @details Disables servo outputs by clearing the soft-armed state. When disarmed,
 * servos are not actively controlled and antenna may drift or hold last position
 * depending on servo type (digital servos typically hold, analog may drift).
 * 
 * Disarming is appropriate for STOP and MANUAL modes where active tracking is not
 * required or user may manually position the antenna.
 * 
 * @note Disarming stops active antenna control but doesn't cut servo power
 * 
 * @see Tracker::arm_servos() for enabling servo control
 * @see Tracker::set_mode() for mode-based arming decisions
 * 
 * Source: AntennaTracker/system.cpp:151-157
 */
void Tracker::disarm_servos()
{
    // Disable servo control by clearing soft-armed state
    hal.util->set_soft_armed(false);
#if HAL_LOGGING_ENABLED
    // Log disarmed state change for analysis
    logger.set_vehicle_armed(false);
#endif
}

/**
 * @brief Initializes servos to trim position for startup
 * 
 * @details Sets both yaw and pitch servos to their configured trim (center) positions
 * and outputs the PWM signals. This initialization is important for some servo types
 * that require power-on positioning before accepting normal commands.
 * 
 * The function:
 * 1. Records current time as startup timestamp
 * 2. Limits yaw servo output to TRIM value
 * 3. Limits pitch servo output to TRIM value  
 * 4. Calculates PWM values for all servo channels
 * 5. Outputs PWM signals to physical servo pins
 * 
 * Trim positions are configured via SERVOx_TRIM parameters and represent the
 * mechanical center position of each antenna axis. Starting at trim prevents
 * sudden servo movements during initialization.
 * 
 * @note Called during initialization if STARTUP_DELAY parameter is non-zero.
 * The delay gives servos time to move to trim position before normal operation.
 * 
 * @warning Some digital servos require initial positioning after power-on to
 * establish their control range. Skipping this step may result in erratic initial
 * servo behavior.
 * 
 * @see Tracker::init_ardupilot() for initialization sequence
 * @see SRV_Channel for servo output management
 * 
 * Source: AntennaTracker/system.cpp:162-169
 */
void Tracker::prepare_servos()
{
    // Record initialization start time for startup delay calculations
    start_time_ms = AP_HAL::millis();
    
    // Set yaw servo to trim (center) position
    SRV_Channels::set_output_limit(SRV_Channel::k_tracker_yaw, SRV_Channel::Limit::TRIM);
    
    // Set pitch servo to trim (center) position
    SRV_Channels::set_output_limit(SRV_Channel::k_tracker_pitch, SRV_Channel::Limit::TRIM);
    
    // Calculate PWM values for all configured servo channels based on output limits
    SRV_Channels::calc_pwm();
    
    // Output PWM signals to all servo channels (physical pin outputs)
    SRV_Channels::output_ch_all();
}

/**
 * @brief Changes tracker mode to specified mode object
 * 
 * @details Switches the tracker to a new operational mode and performs all necessary
 * state transitions. The function handles:
 * 1. Mode change validation (skips if already in target mode)
 * 2. Mode pointer update to new mode object
 * 3. Servo arming/disarming based on mode requirements
 * 4. Mode change logging for analysis
 * 5. GCS notification via heartbeat message
 * 6. Navigation status update with current bearing
 * 
 * Available modes:
 * - MANUAL: User directly controls servos via RC input
 * - STOP: Servos disarmed, antenna stationary
 * - SCAN: Automated scanning pattern for target acquisition  
 * - SERVOTEST: Servo range and trim testing
 * - AUTO: Automated target tracking using vehicle position
 * - GUIDED: Tracks specific lat/lon coordinates from GCS
 * 
 * Modes that track moving targets (AUTO, GUIDED) require armed servos for active
 * antenna control. Modes without tracking (STOP, MANUAL) typically disarm servos.
 * 
 * @param[in] newmode Reference to new mode object (mode_auto, mode_manual, etc.)
 * @param[in] reason ModeReason enum indicating why mode changed (STARTUP, GCS_COMMAND, etc.)
 * 
 * @note Mode changes are logged with reason for post-flight analysis.
 * Frequent mode changes may indicate configuration or operational issues.
 * 
 * @see Tracker::arm_servos() for servo arming on modes requiring active tracking
 * @see Tracker::disarm_servos() for servo disarming on passive modes
 * @see Mode::requires_armed_servos() for mode-specific arming requirements
 * 
 * Source: AntennaTracker/system.cpp:171-194
 */
void Tracker::set_mode(Mode &newmode, const ModeReason reason)
{
    // Record the reason for this mode change (for logging and analysis)
    control_mode_reason = reason;

    // Skip mode change if already in requested mode (avoid redundant state transitions)
    if (mode == &newmode) {
        // don't switch modes if we are already in the correct mode.
        return;
    }
    
    // Update mode pointer to new mode object
    mode = &newmode;

    // Arm or disarm servos based on whether new mode requires active antenna control
    // Tracking modes (AUTO, GUIDED, SCAN) arm servos; passive modes (STOP, MANUAL) disarm
    if (mode->requires_armed_servos()) {
        arm_servos();
    } else {
        disarm_servos();
    }

#if HAL_LOGGING_ENABLED
    // Log mode change with reason for post-flight analysis
    logger.Write_Mode((uint8_t)mode->number(), reason);
#endif
    
    // Send heartbeat to GCS to notify of mode change (reflected in system status)
    gcs().send_message(MSG_HEARTBEAT);

    // Update navigation status with current heading for tracking reference
    nav_status.bearing = ahrs.get_yaw_deg();
}

/**
 * @brief Changes tracker mode using numeric mode identifier
 * 
 * @details Convenience overload that accepts a numeric mode identifier (typically from
 * MAVLink commands or RC channel) and converts it to the appropriate mode object before
 * calling the main set_mode() function.
 * 
 * Mode number mapping:
 * - 0: INITIALISING (rejected - not a valid operational mode)
 * - 1: AUTO - Automated target tracking using vehicle telemetry
 * - 2: MANUAL - Direct RC control of servos
 * - 3: SCAN - Automated scanning pattern for target acquisition
 * - 4: SERVOTEST - Servo diagnostic and calibration mode
 * - 5: STOP - Servos disarmed, antenna stationary
 * - 6: GUIDED - Track specific coordinates from GCS
 * 
 * This function validates the requested mode number and rejects invalid values
 * (including INITIALISING which is only for boot-time state).
 * 
 * @param[in] new_mode Numeric mode identifier (0-6, from Mode::Number enum)
 * @param[in] reason ModeReason enum indicating why mode change was requested
 * 
 * @return true if mode number was valid and mode change completed successfully
 * @return false if mode number invalid (including INITIALISING) or mode object not found
 * 
 * @note This overload is commonly used by MAVLink command handlers and RC channel mode switching.
 * Invalid mode requests are rejected without changing current mode.
 * 
 * @see Tracker::set_mode(Mode&, ModeReason) for underlying mode change implementation
 * @see Mode::Number for complete mode enumeration
 * 
 * Source: AntennaTracker/system.cpp:196-226
 */
bool Tracker::set_mode(const uint8_t new_mode, const ModeReason reason)
{
    Mode *fred = nullptr;
    
    // Convert numeric mode to corresponding mode object pointer
    switch ((Mode::Number)new_mode) {
    case Mode::Number::INITIALISING:
        // INITIALISING is not a valid operational mode, reject request
        return false;
        
    case Mode::Number::AUTO:
        // Automated tracking using vehicle position telemetry
        fred = &mode_auto;
        break;
        
    case Mode::Number::MANUAL:
        // Direct RC control of antenna servos
        fred = &mode_manual;
        break;
        
    case Mode::Number::SCAN:
        // Automated scanning pattern to locate target
        fred = &mode_scan;
        break;
        
    case Mode::Number::SERVOTEST:
        // Servo testing and calibration mode
        fred = &mode_servotest;
        break;
        
    case Mode::Number::STOP:
        // Disarm servos, antenna stationary
        fred = &mode_stop;
        break;
        
    case Mode::Number::GUIDED:
        // Track specific lat/lon coordinates from GCS
        fred = &mode_guided;
        break;
    }
    
    // Validate that mode number mapped to a valid mode object
    if (fred == nullptr) {
        return false;
    }
    
    // Perform actual mode change using mode object
    set_mode(*fred, reason);
    return true;
}

#if HAL_LOGGING_ENABLED
/**
 * @brief Determines if a specific log message type should be logged
 * 
 * @details Checks if logging is enabled for the specified message type based on
 * the LOG_BITMASK parameter configuration. This function provides a vehicle-specific
 * hook for logging decisions, though currently it simply delegates to the logger.
 * 
 * The LOG_BITMASK parameter controls which message types are recorded to dataflash/SD:
 * - Each bit represents a different message category (ATTITUDE, GPS, RCIN, etc.)
 * - Setting a bit enables logging for that message type
 * - Selective logging reduces storage usage and improves logging performance
 * 
 * @param[in] mask Bit mask indicating the message type to check (e.g., MASK_LOG_ATTITUDE_FAST)
 * 
 * @return true if logging is enabled for this message type and logging system is active
 * @return false if logging disabled for this type or logging system not ready
 * 
 * @note Used throughout codebase to gate expensive log message preparation.
 * Always check should_log() before constructing complex log messages.
 * 
 * @see AP_Logger::should_log() for underlying logging decision logic
 * @see LOG_BITMASK parameter for message type configuration
 * 
 * Source: AntennaTracker/system.cpp:232-238
 */
bool Tracker::should_log(uint32_t mask)
{
    // Delegate to logger's should_log() - provides hook for vehicle-specific logic if needed
    if (!logger.should_log(mask)) {
        return false;
    }
    return true;
}
#endif

/*
 * Dummy method stubs to satisfy linker without including full library implementations
 * 
 * AntennaTracker doesn't use several AP libraries that are referenced by common headers
 * (shared singleton accessors, etc.). To avoid linking unused library code and bloating
 * the binary, we provide stub implementations that return null/false.
 * 
 * These stubs should never be called during normal tracker operation. If called, they
 * return safe default values (nullptr or false) to prevent crashes.
 */

#include <AP_AdvancedFailsafe/AP_AdvancedFailsafe.h>
#include <AP_Avoidance/AP_Avoidance.h>
#include <AP_ADSB/AP_ADSB.h>

#if AP_ADVANCEDFAILSAFE_ENABLED
/**
 * @brief Dummy implementation to avoid linking AP_AdvancedFailsafe library
 * 
 * @details AntennaTracker doesn't use Advanced Failsafe (AFS) system since it's
 * stationary and has no flight safety implications. This stub satisfies the linker
 * when AFS references exist in common code paths.
 * 
 * @return false Always returns false (termination not supported)
 * 
 * @note Should never be called during normal tracker operation
 */
bool AP_AdvancedFailsafe::gcs_terminate(bool should_terminate, const char *reason) {return false;}

/**
 * @brief Dummy accessor for AdvancedFailsafe singleton
 * 
 * @return nullptr Always returns null (AdvancedFailsafe not used by tracker)
 * 
 * @note Should never be called during normal tracker operation
 */
AP_AdvancedFailsafe *AP::advancedfailsafe() { return nullptr; }
#endif  // AP_ADVANCEDFAILSAFE_ENABLED

#if AP_ADSB_AVOIDANCE_ENABLED
/**
 * @brief Dummy accessor for AP_Avoidance singleton
 * 
 * @details AntennaTracker doesn't use avoidance system (no collision risk for
 * stationary antenna). This stub satisfies linker when avoidance references exist
 * in common code paths.
 * 
 * @return nullptr Always returns null (AP_Avoidance not used by tracker)
 * 
 * @note Should never be called during normal tracker operation
 */
AP_Avoidance *AP::ap_avoidance() { return nullptr; }
#endif
