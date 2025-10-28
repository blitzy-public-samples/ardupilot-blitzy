/**
 * @file AP_Logger.h
 * @brief ArduPilot main logging subsystem - binary flight data recorder
 * 
 * @details This is the primary logging interface for ArduPilot autopilot firmware.
 * AP_Logger provides a singleton interface for recording vehicle state, sensor data,
 * control outputs, and events to persistent storage or MAVLink streaming.
 * 
 * Architecture:
 * - Singleton pattern accessed via AP::logger() namespace function
 * - Pluggable backend architecture supporting multiple logging targets simultaneously:
 *   * AP_Logger_File: Filesystem-based logging to SD card (FAT/LittleFS)
 *   * AP_Logger_Block: Direct block flash device logging (DataFlash chips)
 *   * AP_Logger_MAVLink: Real-time streaming over MAVLink to ground station
 * - Non-blocking I/O model with dedicated io_thread for filesystem operations
 * - Dynamic message format registration for flexible logging schemas
 * - Rate limiting and priority-based message scheduling
 * 
 * Typical Usage:
 * @code
 * // Singleton access
 * AP_Logger &logger = AP::logger();
 * 
 * // Initialize with bitmask and format structures
 * logger.init(log_bitmask, vehicle_log_structures, num_structures);
 * 
 * // Write structured messages
 * logger.Write("CTUN", "TimeUS,ThI,ThO,DAlt,Alt,DSAlt,SAlt,DCRt,CRt", "Qfffffff",
 *              AP_HAL::micros64(),
 *              throttle_in, throttle_out, 
 *              desired_alt, current_alt,
 *              desired_climb_rate, climb_rate);
 * 
 * // Write events
 * logger.Write_Event(LogEvent::ARMED);
 * 
 * // Write errors
 * logger.Write_Error(LogErrorSubsystem::COMPASS, LogErrorCode::UNHEALTHY);
 * @endcode
 * 
 * Thread Safety:
 * - Write operations are thread-safe via internal semaphores
 * - Backend I/O isolated to dedicated thread to prevent main loop blocking
 * - Format registration protected by log_write_fmts_sem
 * 
 * Performance Characteristics:
 * - Typical logging rates: 100-400 Hz for critical messages, 1-50 Hz for telemetry
 * - Write latency: <100μs for buffer writes, actual I/O asynchronous
 * - Buffer sizes: 8-32 KB configurable per backend
 * - I/O thread priority: lower than main loop to prevent interference
 * 
 * @note This logging system is critical for post-flight analysis, in-flight
 * monitoring, and incident investigation. Log data is essential for debugging
 * flight issues and tuning vehicle performance.
 * 
 * @warning Logging operations must never block the main control loop. Always use
 * asynchronous write methods for time-critical code paths.
 * 
 * @see AP_Logger_Backend for backend implementation interface
 * @see LogStructure.h for binary message format definitions
 * @see AP_Logger_File, AP_Logger_Block, AP_Logger_MAVLink for backend implementations
 */
#pragma once

#include "AP_Logger_config.h"

#if HAL_LOGGING_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Logger/LogStructure.h>
#include <AP_Vehicle/ModeReason.h>

#include <stdint.h>

#include "LoggerMessageWriter.h"

class AP_Logger_Backend;

/**
 * @enum LogEvent
 * @brief Vehicle state change events logged to binary dataflash logs
 * 
 * @details LogEvent defines significant vehicle state changes and discrete events
 * that are logged with timestamps for post-flight analysis. These events mark
 * important transitions in vehicle operation, mode changes, safety system
 * activations, and configuration changes.
 * 
 * Events are logged via AP_Logger::Write_Event(LogEvent id) and appear as "EV"
 * messages in binary logs with the event ID and timestamp.
 * 
 * Usage Example:
 * @code
 * // Log arming event
 * AP::logger().Write_Event(LogEvent::ARMED);
 * 
 * // Log fence breach
 * AP::logger().Write_Event(LogEvent::FENCE_ENABLE);
 * 
 * // Log EKF yaw reset
 * AP::logger().Write_Event(LogEvent::EKF_YAW_RESET);
 * @endcode
 * 
 * Event Categories:
 * - Arming/Disarming: ARMED, DISARMED, AUTO_ARMED
 * - Landing State: LAND_COMPLETE, LAND_COMPLETE_MAYBE, NOT_LANDED
 * - Safety Systems: Fence events (FENCE_ENABLE, etc.), parachute, landing gear
 * - Motor Control: Emergency stop, interlock, rotor runup (heli-specific)
 * - Navigation: EKF resets, GPS primary changes, landing repositioning
 * - Flight Modes: Autotune states, ACRO trainer, simple/supersimple mode
 * - Autonomous Operations: ZIGZAG waypoint storage, SRTL path recording
 * - Underwater (Sub): SURFACED, NOT_SURFACED, BOTTOMED, NOT_BOTTOMED
 * 
 * @note Event IDs are NEVER reused or removed to maintain log compatibility.
 * Adding new events is safe, but modifying existing IDs breaks log analysis tools.
 * Gaps in numbering (e.g., 68-70 removed winch events) are intentional.
 * 
 * @warning Do not modify or reuse event IDs - this would break historical log
 * analysis and mission log review tools used by users and developers.
 */
// do not do anything here apart from add stuff; maintaining older
// entries means log analysis is easier
enum class LogEvent : uint8_t {
    ARMED = 10,
    DISARMED = 11,
    AUTO_ARMED = 15,
    LAND_COMPLETE_MAYBE = 17,
    LAND_COMPLETE = 18,
    LOST_GPS = 19,
    FLIP_START = 21,
    FLIP_END = 22,
    SET_HOME = 25,
    SET_SIMPLE_ON = 26,
    SET_SIMPLE_OFF = 27,
    NOT_LANDED = 28,
    SET_SUPERSIMPLE_ON = 29,
    AUTOTUNE_INITIALISED = 30,
    AUTOTUNE_OFF = 31,
    AUTOTUNE_RESTART = 32,
    AUTOTUNE_SUCCESS = 33,
    AUTOTUNE_FAILED = 34,
    AUTOTUNE_REACHED_LIMIT = 35,
    AUTOTUNE_PILOT_TESTING = 36,
    AUTOTUNE_SAVEDGAINS = 37,
    SAVE_TRIM = 38,
    SAVEWP_ADD_WP = 39,
    FENCE_ENABLE = 41,
    FENCE_DISABLE = 42,
    ACRO_TRAINER_OFF = 43,
    ACRO_TRAINER_LEVELING = 44,
    ACRO_TRAINER_LIMITED = 45,
    GRIPPER_GRAB = 46,
    GRIPPER_RELEASE = 47,
    PARACHUTE_DISABLED = 49,
    PARACHUTE_ENABLED = 50,
    PARACHUTE_RELEASED = 51,
    LANDING_GEAR_DEPLOYED = 52,
    LANDING_GEAR_RETRACTED = 53,
    MOTORS_EMERGENCY_STOPPED = 54,
    MOTORS_EMERGENCY_STOP_CLEARED = 55,
    MOTORS_INTERLOCK_DISABLED = 56,
    MOTORS_INTERLOCK_ENABLED = 57,
    ROTOR_RUNUP_COMPLETE = 58, // Heli only
    ROTOR_SPEED_BELOW_CRITICAL = 59, // Heli only
    EKF_ALT_RESET = 60,
    LAND_CANCELLED_BY_PILOT = 61,
    EKF_YAW_RESET = 62,
    AVOIDANCE_ADSB_ENABLE = 63,
    AVOIDANCE_ADSB_DISABLE = 64,
    AVOIDANCE_PROXIMITY_ENABLE = 65,
    AVOIDANCE_PROXIMITY_DISABLE = 66,
    GPS_PRIMARY_CHANGED = 67,
    // 68, 69, 70 were winch events
    ZIGZAG_STORE_A = 71,
    ZIGZAG_STORE_B = 72,
    LAND_REPO_ACTIVE = 73,
    STANDBY_ENABLE = 74,
    STANDBY_DISABLE = 75,

    // Fence events
    FENCE_ALT_MAX_ENABLE = 76,
    FENCE_ALT_MAX_DISABLE = 77,
    FENCE_CIRCLE_ENABLE = 78,
    FENCE_CIRCLE_DISABLE = 79,
    FENCE_ALT_MIN_ENABLE = 80,
    FENCE_ALT_MIN_DISABLE = 81,
    FENCE_POLYGON_ENABLE = 82,
    FENCE_POLYGON_DISABLE = 83,

    // if the EKF's source input set is changed (e.g. via a switch or
    // a script), we log an event:
    EK3_SOURCES_SET_TO_PRIMARY = 85,
    EK3_SOURCES_SET_TO_SECONDARY = 86,
    EK3_SOURCES_SET_TO_TERTIARY = 87,

    AIRSPEED_PRIMARY_CHANGED = 90,

    SURFACED = 163,
    NOT_SURFACED = 164,
    BOTTOMED = 165,
    NOT_BOTTOMED = 166,
};

/**
 * @enum LogDataID
 * @brief Identifiers for specific data items logged via Write() with type tagging
 * 
 * @details LogDataID provides type identifiers for logging specific data items
 * that need explicit type tagging in the log stream. These are used primarily
 * for internal state logging that doesn't fit standard message structures.
 * 
 * Current Usage:
 * - AP_STATE (7): ArduPilot subsystem state information
 * - INIT_SIMPLE_BEARING (9): Simple mode initial bearing in degrees
 * 
 * Historical:
 * - SYSTEM_TIME_SET (8): No longer used, kept as comment for ID reservation
 * 
 * @note This enum is sparsely populated. Most logging uses structured messages
 * via LogStructure definitions rather than these tagged data items.
 */
enum class LogDataID : uint8_t {
    AP_STATE = 7,
// SYSTEM_TIME_SET = 8,
    INIT_SIMPLE_BEARING = 9,
};

/**
 * @enum LogErrorSubsystem
 * @brief Subsystem identifiers for error logging via Write_Error()
 * 
 * @details LogErrorSubsystem categorizes error conditions by the vehicle subsystem
 * where the error occurred. Used with LogErrorCode to create hierarchical error
 * reporting in binary logs as "ERR" messages.
 * 
 * Errors are logged via:
 * @code
 * AP::logger().Write_Error(LogErrorSubsystem::COMPASS, LogErrorCode::UNHEALTHY);
 * AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_BATT, LogErrorCode::FAILSAFE_OCCURRED);
 * AP::logger().Write_Error(LogErrorSubsystem::GPS, LogErrorCode::GPS_GLITCH);
 * @endcode
 * 
 * Subsystem Categories:
 * - Core Systems: MAIN, CPU, INTERNAL_ERROR
 * - Sensors: COMPASS, GPS, BARO, OPTFLOW (deprecated)
 * - Communication: RADIO, PILOT_INPUT
 * - Navigation: NAVIGATION, EKF_PRIMARY, EKFCHECK, TERRAIN
 * - Flight Control: FLIGHT_MODE, CRASH_CHECK, FLIP, THRUST_LOSS_CHECK
 * - Failsafe Systems: FAILSAFE_RADIO, FAILSAFE_BATT, FAILSAFE_GPS (deprecated),
 *   FAILSAFE_GCS, FAILSAFE_FENCE, FAILSAFE_EKFINAV, FAILSAFE_ADSB,
 *   FAILSAFE_TERRAIN, FAILSAFE_SENSORS, FAILSAFE_LEAK (Sub-specific),
 *   FAILSAFE_VIBE, FAILSAFE_DEADRECKON
 * - Special Features: AUTOTUNE (deprecated), PARACHUTES
 * 
 * @note Some subsystems marked "not used" are retained for log compatibility.
 * Error subsystem IDs should not be reused to maintain log analysis consistency.
 * 
 * @warning Error logging is critical for diagnosing flight issues. Always log
 * errors when subsystem health degrades or failsafes trigger.
 * 
 * @see LogErrorCode for specific error codes within each subsystem
 * @see AP_Logger::Write_Error() for error logging method
 */
enum class LogErrorSubsystem : uint8_t {
    MAIN = 1,
    RADIO = 2,
    COMPASS = 3,
    OPTFLOW = 4,   // not used
    FAILSAFE_RADIO = 5,
    FAILSAFE_BATT = 6,
    FAILSAFE_GPS = 7,   // not used
    FAILSAFE_GCS = 8,
    FAILSAFE_FENCE = 9,
    FLIGHT_MODE = 10,
    GPS = 11,
    CRASH_CHECK = 12,
    FLIP = 13,
    AUTOTUNE = 14,  // not used
    PARACHUTES = 15,
    EKFCHECK = 16,
    FAILSAFE_EKFINAV = 17,
    BARO = 18,
    CPU = 19,
    FAILSAFE_ADSB = 20,
    TERRAIN = 21,
    NAVIGATION = 22,
    FAILSAFE_TERRAIN = 23,
    EKF_PRIMARY = 24,
    THRUST_LOSS_CHECK = 25,
    FAILSAFE_SENSORS = 26,
    FAILSAFE_LEAK = 27,
    PILOT_INPUT = 28,
    FAILSAFE_VIBE = 29,
    INTERNAL_ERROR = 30,
    FAILSAFE_DEADRECKON = 31
};

/**
 * @enum LogErrorCode
 * @brief Specific error codes within subsystems for error logging
 * 
 * @details LogErrorCode provides specific error conditions paired with
 * LogErrorSubsystem to create detailed error reports. Used via Write_Error()
 * to log subsystem-specific error conditions.
 * 
 * @warning This enumeration intentionally contains duplicate values across
 * different contexts, providing minimal type safety. Error codes are interpreted
 * based on their associated LogErrorSubsystem. For example, error code 1 means
 * "FAILED_TO_INITIALISE" for general errors, "FAILSAFE_OCCURRED" for failsafe
 * subsystems, and "MAIN_INS_DELAY" for MAIN subsystem.
 * 
 * Common Error Codes (subsystem-independent):
 * - ERROR_RESOLVED (0): Error condition cleared
 * - FAILED_TO_INITIALISE (1): Subsystem initialization failed
 * - UNHEALTHY (4): Subsystem unhealthy but functional
 * 
 * Failsafe-Specific Codes (FAILSAFE_* subsystems):
 * - FAILSAFE_RESOLVED (0): Failsafe condition cleared
 * - FAILSAFE_OCCURRED (1): Failsafe triggered
 * 
 * Radio Subsystem Codes:
 * - RADIO_LATE_FRAME (2): RC frame received late
 * 
 * Main Subsystem Codes:
 * - MAIN_INS_DELAY (1): Inertial sensor data delayed
 * 
 * Crash Check Subsystem Codes:
 * - CRASH_CHECK_CRASH (1): Vehicle crash detected
 * - CRASH_CHECK_LOSS_OF_CONTROL (2): Loss of control detected
 * 
 * Flip Mode Subsystem Codes:
 * - FLIP_ABANDONED (2): Flip maneuver abandoned
 * 
 * Terrain Subsystem Codes:
 * - MISSING_TERRAIN_DATA (2): Terrain data unavailable
 * 
 * Navigation Subsystem Codes:
 * - FAILED_TO_SET_DESTINATION (2): Navigation destination invalid
 * - RESTARTED_RTL (3): Return to launch restarted
 * - FAILED_CIRCLE_INIT (4): Circle mode initialization failed
 * - DEST_OUTSIDE_FENCE (5): Destination outside geofence
 * - RTL_MISSING_RNGFND (6): RTL requires rangefinder but none available
 * 
 * Internal Error Subsystem Codes:
 * - INTERNAL_ERRORS_DETECTED (1): Internal consistency check failed
 * 
 * Parachute Subsystem Codes:
 * - PARACHUTE_TOO_LOW (2): Altitude too low for parachute deployment
 * - PARACHUTE_LANDED (3): Vehicle already landed, parachute not deployed
 * 
 * EKF Check Subsystem Codes:
 * - EKFCHECK_BAD_VARIANCE (2): EKF variance exceeds threshold
 * - EKFCHECK_VARIANCE_CLEARED (0): EKF variance returned to normal
 * 
 * Barometer Subsystem Codes:
 * - BARO_GLITCH (2): Barometer reading glitch detected
 * - BAD_DEPTH (3): Invalid depth reading (Sub-specific)
 * 
 * GPS Subsystem Codes:
 * - GPS_GLITCH (2): GPS position glitch detected
 * 
 * Usage Example:
 * @code
 * // Log compass unhealthy
 * AP::logger().Write_Error(LogErrorSubsystem::COMPASS, LogErrorCode::UNHEALTHY);
 * 
 * // Log battery failsafe trigger
 * AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_BATT, LogErrorCode::FAILSAFE_OCCURRED);
 * 
 * // Log battery failsafe cleared
 * AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_BATT, LogErrorCode::FAILSAFE_RESOLVED);
 * 
 * // Log navigation error with specific code
 * AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
 * @endcode
 * 
 * @note Error codes are context-dependent based on subsystem. Always pair
 * LogErrorCode with appropriate LogErrorSubsystem for meaningful error reporting.
 * 
 * @see LogErrorSubsystem for subsystem categories
 * @see AP_Logger::Write_Error() for error logging method
 */
// bizarrely this enumeration has lots of duplicate values, offering
// very little in the way of typesafety
enum class LogErrorCode : uint8_t {
// general error codes
    ERROR_RESOLVED  = 0,
    FAILED_TO_INITIALISE = 1,
    UNHEALTHY = 4,
// subsystem specific error codes -- radio
    RADIO_LATE_FRAME = 2,
// subsystem specific error codes -- failsafe_thr, batt, gps
    FAILSAFE_RESOLVED = 0,
    FAILSAFE_OCCURRED = 1,
// subsystem specific error codes -- main
    MAIN_INS_DELAY = 1,
// subsystem specific error codes -- crash checker
    CRASH_CHECK_CRASH = 1,
    CRASH_CHECK_LOSS_OF_CONTROL = 2,
// subsystem specific error codes -- flip
    FLIP_ABANDONED = 2,
// subsystem specific error codes -- terrain
    MISSING_TERRAIN_DATA = 2,
// subsystem specific error codes -- navigation
    FAILED_TO_SET_DESTINATION = 2,
    RESTARTED_RTL = 3,
    FAILED_CIRCLE_INIT = 4,
    DEST_OUTSIDE_FENCE = 5,
    RTL_MISSING_RNGFND = 6,
// subsystem specific error codes -- internal_error
    INTERNAL_ERRORS_DETECTED = 1,

// parachute failed to deploy because of low altitude or landed
    PARACHUTE_TOO_LOW = 2,
    PARACHUTE_LANDED = 3,
// EKF check definitions
    EKFCHECK_BAD_VARIANCE = 2,
    EKFCHECK_VARIANCE_CLEARED = 0,
// Baro specific error codes
    BARO_GLITCH = 2,
    BAD_DEPTH = 3, // sub-only
// GPS specific error codes
    GPS_GLITCH = 2,
};

/**
 * @class AP_Logger
 * @brief Main ArduPilot logging subsystem singleton for binary flight data recording
 * 
 * @details AP_Logger is the central logging facility for ArduPilot autopilot software,
 * providing structured binary logging of vehicle state, sensor readings, control outputs,
 * and events. It implements a pluggable backend architecture supporting multiple
 * simultaneous logging targets.
 * 
 * Architecture Overview:
 * 
 * Singleton Pattern:
 * - Accessed globally via AP::logger() namespace function
 * - Single instance created at startup, initialized during vehicle init
 * - Thread-safe access via internal semaphores
 * 
 * Backend Architecture:
 * - Supports up to LOGGER_MAX_BACKENDS (2) simultaneous backends
 * - Backends probed and initialized during init() based on _params.backend_types
 * - Common backends:
 *   * AP_Logger_File: SD card logging via AP_Filesystem (FAT/LittleFS)
 *   * AP_Logger_Block: Direct DataFlash chip logging (SPI/QSPI flash)
 *   * AP_Logger_MAVLink: Real-time log streaming to ground station
 * - FOR_EACH_BACKEND macro iterates all active backends for operations
 * 
 * I/O Thread Model:
 * - Dedicated io_thread created via hal.scheduler->thread_create() for non-blocking I/O
 * - Main loop write calls copy data to memory buffers (fast, <100μs)
 * - io_thread handles actual filesystem/flash writes asynchronously
 * - Prevents main control loop blocking on slow storage operations
 * - io_timer provides periodic servicing and heartbeat monitoring
 * 
 * Dynamic Message Format System:
 * - Supports runtime registration of message formats via Write() methods
 * - log_write_fmts linked list maintains format definitions
 * - msg_fmt_for_name() finds or allocates format structures
 * - find_free_msg_type() allocates unique message type IDs (0-255)
 * - Automatic FMT message emission on first use of each message type
 * - Format strings define field types: Q=uint64_t, I=uint32_t, f=float, etc.
 * 
 * Initialization Sequence:
 * 1. Constructor: AP_Logger() creates singleton, registers parameters
 * 2. init(): Probes and initializes backends, registers message structures
 * 3. start_io_thread(): Launches dedicated I/O thread (if not already started)
 * 4. PrepForArming(): Pre-flight preparation, ensures logging ready
 * 5. Vehicle writes messages via Write(), WriteBlock(), etc.
 * 6. StopLogging(): Graceful shutdown, flushes buffers
 * 
 * Message Writing Methods:
 * - Write(): Variable-argument structured messages with format strings
 * - WriteV(): va_list variant for internal use
 * - WriteStreaming(): Lower-priority telemetry messages
 * - WriteCritical(): High-priority critical messages
 * - WriteBlock(): Raw binary block writes
 * - WriteCriticalBlock(): High-priority binary block writes
 * - WritePrioritisedBlock(): Priority-aware binary writes
 * 
 * Priority and Rate Limiting:
 * - Critical messages bypass rate limits
 * - Streaming messages subject to rate limiting
 * - Per-backend rate limits: _params.file_ratemax, _params.mav_ratemax
 * - should_log() checks bitmask to filter messages by category
 * 
 * Concurrency Control:
 * - log_write_fmts_sem: Protects format registration
 * - _log_send_sem: Protects MAVLink log transfer state
 * - WITH_SEMAPHORE used throughout for critical sections
 * - Thread-safe singleton access
 * 
 * Parameter System Integration:
 * - var_info[] defines configurable parameters via AP_Param
 * - _BACKEND_TYPE: Backend selection bitmask (file/MAVLink/block)
 * - _FILE_BUFSIZE: Filesystem backend buffer size (KB)
 * - _DISARMED: Disarmed logging behavior (none/continuous/discard)
 * - _REPLAY: Log replay mode for algorithm development
 * - _FILE_TIMEOUT: Maximum time without logging before closing file (seconds)
 * - _FILE_MB_FREE: Minimum free space before logging stops (MB)
 * - _FILE_RATEMAX: Maximum file logging rate (Hz)
 * - _MAV_RATEMAX: Maximum MAVLink streaming rate (Hz)
 * 
 * Log Enumeration API:
 * - find_last_log(): Returns most recent log number
 * - get_log_boundaries(): Returns start/end pages for log number
 * - get_num_logs(): Counts available logs
 * - get_log_info(): Retrieves log size and timestamp
 * - Log numbering: Sequential integers, may have gaps from erased logs
 * 
 * Vehicle Integration:
 * - _vehicle_messages: Callback for vehicle-specific startup logging
 * - setVehicle_Startup_Writer(): Registers vehicle startup writer
 * - Vehicle calls during startup to log initial configuration
 * 
 * Performance Characteristics:
 * - Typical logging rates: 100-400 Hz for flight control messages
 * - Telemetry rates: 1-50 Hz for lower-priority data
 * - Write latency: <100μs for memory buffer writes
 * - Buffer sizes: 8-32 KB per backend (configurable)
 * - I/O thread runs at lower priority than main control loop
 * 
 * Special Features:
 * - Crash dump persistence: Saves system state on panic
 * - File content snapshot: Logs configuration files at arming
 * - Log-while-disarmed: Continuous logging for debugging
 * - MAVLink log download: Remote log retrieval protocol
 * - Log replay: Data Abstraction Layer (DAL) for algorithm development
 * 
 * Typical Usage Pattern:
 * @code
 * // Access singleton
 * AP_Logger &logger = AP::logger();
 * 
 * // Check if logging enabled for category
 * if (logger.should_log(MASK_LOG_ATTITUDE)) {
 *     // Write structured message
 *     logger.Write("ATT", "TimeUS,Roll,Pitch,Yaw", "Qfff",
 *                  AP_HAL::micros64(),
 *                  ahrs.roll, ahrs.pitch, ahrs.yaw);
 * }
 * 
 * // Write event
 * logger.Write_Event(LogEvent::ARMED);
 * 
 * // Write error
 * logger.Write_Error(LogErrorSubsystem::GPS, LogErrorCode::GPS_GLITCH);
 * @endcode
 * 
 * @note Logging is essential for post-flight analysis, debugging, and performance
 * tuning. All safety-critical state changes and anomalies should be logged.
 * 
 * @warning Write operations must complete quickly (<100μs) to avoid blocking
 * the main control loop. Heavy I/O is delegated to io_thread automatically.
 * 
 * @see AP_Logger_Backend for backend implementation interface
 * @see AP_Logger_File for filesystem logging implementation
 * @see AP_Logger_Block for DataFlash logging implementation
 * @see AP_Logger_MAVLink for MAVLink streaming implementation
 * @see LogStructure.h for message format definitions
 */
class AP_Logger
{
    friend class AP_Logger_Backend; // for _num_types
    friend class AP_Logger_RateLimiter;

public:
    /**
     * @typedef vehicle_startup_message_Writer
     * @brief Function pointer type for vehicle-specific startup message callback
     * 
     * @details Allows vehicle code to register custom startup message writer
     * that executes during log file initialization. Used to log vehicle-specific
     * state that must be captured at log start (mode, configuration, versions).
     * 
     * @see setVehicle_Startup_Writer()
     */
    FUNCTOR_TYPEDEF(vehicle_startup_message_Writer, void);

    /**
     * @brief Constructor - creates AP_Logger singleton instance
     * 
     * @details Private constructor called once during static initialization.
     * Initializes member variables but does not start logging - that requires
     * explicit init() call from vehicle code.
     * 
     * Initialization happens in two phases:
     * 1. Constructor (automatic): Creates singleton, zeros state
     * 2. init() (explicit): Probes backends, starts I/O thread, begins logging
     * 
     * @note This is called automatically before main() - do not call directly
     */
    AP_Logger();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Logger);

    /**
     * @brief Get singleton instance of AP_Logger
     * 
     * @return Pointer to singleton AP_Logger instance
     * 
     * @note Prefer using AP::logger() namespace function for cleaner syntax
     * @see AP::logger()
     */
    static AP_Logger *get_singleton(void) {
        return _singleton;
    }

    /**
     * @brief Initialize logging subsystem with message structures and bitmask
     * 
     * @details Performs complete logging subsystem initialization:
     * 1. Registers message format structures from vehicle
     * 2. Probes and initializes backends based on _params.backend_types
     * 3. Starts I/O thread for asynchronous writes
     * 4. Emits startup messages (FMT, PARM, vehicle-specific)
     * 
     * Backend Initialization:
     * - AP_Logger_File::probe() checks for SD card/filesystem
     * - HAL_LOGGING_DATAFLASH_DRIVER::probe() checks for DataFlash chip
     * - AP_Logger_MAVLink always available if enabled
     * 
     * Format Registration:
     * - structure array defines binary message layouts
     * - Each LogStructure specifies: msg_type, msg_len, name, format, labels, units, mults
     * - FMT messages automatically emitted on first use of each message type
     * 
     * Startup Logging:
     * - Emits FMT messages for all registered structures
     * - Logs all parameters via Write_Parameter()
     * - Calls _vehicle_messages() for vehicle-specific startup
     * 
     * @param[in] log_bitmask Reference to parameter controlling which message categories to log
     *                        (bitmask of LOG_* flags from vehicle)
     * @param[in] structure   Array of LogStructure definitions for this vehicle
     * @param[in] num_types   Number of entries in structure array
     * 
     * @note Must be called during vehicle setup before logging begins
     * @note Backends may fail to initialize (e.g., no SD card) - check logging_enabled()
     * @warning Call only once during vehicle initialization
     * 
     * @see LogStructure for message format definition
     * @see AP_Logger_Backend::probe() for backend initialization
     */
    void init(const AP_Int32 &log_bitmask, const struct LogStructure *structure, uint8_t num_types);
    
    /**
     * @brief Set number of message types (for dynamic structure updates)
     * 
     * @param[in] num_types Number of message types registered
     * 
     * @note Used internally when adding dynamic message formats
     */
    void set_num_types(uint8_t num_types) { _num_types = num_types; }

    bool CardInserted(void);
    bool _log_pause;

    // pause logging if aux switch is active and log rate limit enabled
    void log_pause(bool value) {
        _log_pause = value;
    }

    /**
     * @brief Erase all logs on all backends
     * 
     * @details Deletes all log files/blocks on all active backends.
     * Used for log management and storage reclamation.
     * 
     * @warning This is a destructive operation - all flight logs will be lost.
     * Only call when explicitly commanded by user.
     */
    void EraseAll();

    /**
     * @brief Write a block of binary data to all backends
     * 
     * @details Writes raw binary data block to all active logging backends.
     * Data is copied to backend buffers and written asynchronously by io_thread.
     * This is a normal-priority write that may be rate-limited or dropped under
     * high logging load.
     * 
     * @param[in] pBuffer Pointer to data buffer to write
     * @param[in] size    Number of bytes to write
     * 
     * @note Typical execution time: 10-500μs depending on backend buffer state
     * @note Data includes message header - caller must format complete message
     * 
     * @warning Do not call from interrupt context - uses semaphores
     * @warning Performance impact: Typical 10-100μs in fast path, may spike to
     * 500μs if buffers full and blocking write required
     * 
     * @see WriteCriticalBlock() for high-priority writes
     * @see WritePrioritisedBlock() for explicit priority control
     */
    void WriteBlock(const void *pBuffer, uint16_t size);

    /**
     * @brief Write block to backends, return true if first backend succeeds
     * 
     * @details Similar to WriteBlock() but provides success feedback from first
     * backend. Used when caller needs to know if write was buffered successfully.
     * 
     * @param[in] pBuffer Pointer to data buffer to write
     * @param[in] size    Number of bytes to write
     * 
     * @return true if first backend buffered data successfully, false otherwise
     * 
     * @note Only checks first backend - others may still fail
     */
    bool WriteBlock_first_succeed(const void *pBuffer, uint16_t size);

    /**
     * @brief Write an important block of data with high priority
     * 
     * @details Writes critical binary data block that should not be dropped
     * under high logging load. Critical writes bypass rate limiting and have
     * higher priority in backend queues.
     * 
     * Use for:
     * - Safety-critical events (arming, failsafes, mode changes)
     * - Error conditions
     * - Flight control state at critical moments
     * 
     * @param[in] pBuffer Pointer to data buffer to write  
     * @param[in] size    Number of bytes to write
     * 
     * @note Critical writes may block briefly if all buffers full
     * @warning Use sparingly - too many critical writes defeat priority system
     * 
     * @see WriteBlock() for normal-priority writes
     */
    void WriteCriticalBlock(const void *pBuffer, uint16_t size);

    /**
     * @brief Write a block of log replay data
     * 
     * @details Writes replay data during log replay mode (used for algorithm
     * development with Data Abstraction Layer). Only active when log_replay
     * parameter is set.
     * 
     * @param[in] msg_id  Message type ID
     * @param[in] pBuffer Pointer to message data
     * @param[in] size    Number of bytes to write
     * 
     * @return true if replay write successful, false if replay not active
     * 
     * @note Only used during log replay for algorithm testing
     * @see log_replay() parameter
     */
    bool WriteReplayBlock(uint8_t msg_id, const void *pBuffer, uint16_t size);

    /**
     * @brief Find the most recent log number
     * 
     * @details Searches all backends to find highest log number. Log numbers
     * are sequential integers that may have gaps from erased logs.
     * 
     * @return Most recent log number, or 0 if no logs exist
     * 
     * @note Log numbering is persistent across reboots
     * @note Gaps in numbering indicate erased logs
     * 
     * @see get_num_logs() for total count
     */
    uint16_t find_last_log() const;
    
    /**
     * @brief Get start and end storage pages for a log number
     * 
     * @details Retrieves backend-specific page boundaries for a log file.
     * Used for log download and analysis. Page size and meaning depend on
     * backend (filesystem: byte offsets; DataFlash: physical pages).
     * 
     * @param[in]  log_num    Log number to query
     * @param[out] start_page First page/offset of log
     * @param[out] end_page   Last page/offset of log
     * 
     * @note Page meaning backend-specific (AP_Logger_File uses byte offsets)
     * @see get_log_data() for reading log content
     */
    void get_log_boundaries(uint16_t log_num, uint32_t & start_page, uint32_t & end_page);
    
    /**
     * @brief Get total number of logs available
     * 
     * @details Counts all logs across all backends. May be slow on first call
     * as it enumerates storage.
     * 
     * @return Total number of logs available for download
     * 
     * @see find_last_log() for most recent log number
     */
    uint16_t get_num_logs(void);
    
    /**
     * @brief Get maximum number of logs that can be stored
     * 
     * @details Returns backend's maximum log capacity. For filesystem backends,
     * limited by _params.max_log_files. For DataFlash, limited by chip size.
     * 
     * @return Maximum number of logs before oldest are overwritten
     * 
     * @note When max reached, oldest logs automatically erased
     */
    uint16_t get_max_num_logs();

    /**
     * @brief Register vehicle-specific startup message writer callback
     * 
     * @details Sets callback function that vehicle code provides to log
     * vehicle-specific startup information. Called during init() after
     * FMT and parameter logging to add vehicle configuration details.
     * 
     * Typical startup messages include:
     * - Vehicle firmware version
     * - Frame configuration
     * - Initial parameter snapshot
     * - Hardware detection results
     * 
     * @param[in] writer Function pointer to vehicle startup writer
     * 
     * Example:
     * @code
     * void Copter::log_startup_messages() {
     *     logger.Write_MessageF("Frame: %s", frame_type_string());
     *     // ... more vehicle-specific logging
     * }
     * 
     * // In vehicle init:
     * logger.setVehicle_Startup_Writer(
     *     FUNCTOR_BIND(&copter, &Copter::log_startup_messages, void));
     * @endcode
     * 
     * @note Vehicle must call this before init() for startup logging
     * @see _vehicle_messages member
     */
    void setVehicle_Startup_Writer(vehicle_startup_message_Writer writer);

    /**
     * @brief Prepare logging system for arming
     * 
     * @details Called by vehicle just before arming checks complete.
     * Ensures logging backends are ready and triggers pre-arming operations:
     * - Starts new log file if needed
     * - Verifies sufficient storage space
     * - Logs file content snapshots (configuration files)
     * - Resets rate limiters
     * 
     * @note Must be called during arming sequence
     * @warning Arming may be blocked if logging required but unavailable
     * 
     * @see set_vehicle_armed() for post-arming notification
     */
    void PrepForArming();

    void EnableWrites(bool enable) { _writes_enabled = enable; }
    bool WritesEnabled() const { return _writes_enabled; }

    void StopLogging();

    void Write_Parameter(const char *name, float value);
    void Write_Event(LogEvent id);
    void Write_Error(LogErrorSubsystem sub_system,
                     LogErrorCode error_code);
    void Write_RCIN(void);
    void Write_RCOUT(void);
    void Write_RSSI();
    void Write_Rally();
#if HAL_LOGGER_FENCE_ENABLED
    void Write_Fence();
#endif
    void Write_NamedValueFloat(const char *name, float value);
    void Write_Power(void);
    void Write_Radio(const mavlink_radio_t &packet);
    void Write_Message(const char *message);
    void Write_MessageF(const char *fmt, ...);
    void Write_Compass();
    void Write_Mode(uint8_t mode, const ModeReason reason);

    void Write_EntireMission();
    void Write_Command(const mavlink_command_int_t &packet,
                       uint8_t source_system,
                       uint8_t source_component,
                       MAV_RESULT result,
                       bool was_command_long=false);
    void Write_MISE(const AP_Mission &mission, const AP_Mission::Mission_Command &cmd) {
        Write_Mission_Cmd(mission, cmd, LOG_MISE_MSG);
    }
    void Write_CMD(const AP_Mission &mission, const AP_Mission::Mission_Command &cmd) {
        Write_Mission_Cmd(mission, cmd, LOG_CMD_MSG);
    }
    void Write_Mission_Cmd(const AP_Mission &mission,
                           const AP_Mission::Mission_Command &cmd,
                           LogMessages id);
    void Write_RallyPoint(uint8_t total,
                          uint8_t sequence,
                          const class RallyLocation &rally_point);
    void Write_SRTL(bool active, uint16_t num_points, uint16_t max_points, uint8_t action, const Vector3f& point);
    void Write_Winch(bool healthy, bool thread_end, bool moving, bool clutch, uint8_t mode, float desired_length, float length, float desired_rate, uint16_t tension, float voltage, int8_t temp);

    /**
     * @brief Write variable-argument structured message (normal priority, no units)
     * 
     * @details High-level interface for writing structured log messages with
     * automatic format registration and FMT emission. On first call with a
     * given name, allocates message type ID and emits FMT message defining structure.
     * 
     * Format String Syntax:
     * - 'B': uint8_t
     * - 'b': int8_t
     * - 'H': uint16_t
     * - 'h': int16_t
     * - 'I': uint32_t
     * - 'i': int32_t
     * - 'Q': uint64_t
     * - 'q': int64_t
     * - 'f': float
     * - 'd': double
     * - 'n': char[4] (4-character string)
     * - 'N': char[16] (16-character string)
     * - 'Z': char[64] (64-character string)
     * 
     * @param[in] name   Message name (up to 4 characters, e.g., "ATT", "CTUN")
     * @param[in] labels Comma-separated field labels (e.g., "TimeUS,Roll,Pitch,Yaw")
     * @param[in] fmt    Format string defining field types (e.g., "Qfff")
     * @param[in] ...    Variable arguments matching format string
     * 
     * Example:
     * @code
     * logger.Write("ATT", "TimeUS,Roll,Pitch,Yaw", "Qfff",
     *              AP_HAL::micros64(),
     *              ahrs.roll, ahrs.pitch, ahrs.yaw);
     * @endcode
     * 
     * @note First call allocates message type and emits FMT message
     * @warning Message name limited to 4 characters, labels to 64 characters
     * 
     * @see WriteV() for va_list variant
     */
    void Write(const char *name, const char *labels, const char *fmt, ...);
    
    /**
     * @brief Write structured message with units and multipliers (normal priority)
     * 
     * @details Extended Write() that includes unit and multiplier metadata for
     * better ground station display and log analysis. Units and multipliers are
     * logged in FMT message for automatic scaling and unit display.
     * 
     * Unit Codes: (from log_Units[] in LogStructure.h)
     * - '-': unitless
     * - 'd': degrees
     * - 's': seconds
     * - 'm': meters
     * - 'n': meters/second
     * - 'G': G (gravity)
     * - 'A': Amperes
     * - 'V': Volts
     * - Many more defined in log_Units[]
     * 
     * Multiplier Codes: (from log_Multipliers[] in LogStructure.h)
     * - '-': 1 (no multiplier)
     * - '2': 100 (centidegrees, centimeters, etc.)
     * - '?': 1e-2
     * - '0': 1e0
     * - Many more defined in log_Multipliers[]
     * 
     * @param[in] name   Message name (up to 4 characters)
     * @param[in] labels Comma-separated field labels
     * @param[in] units  Unit codes matching fields (e.g., "sdddddd" for time + 6 angles)
     * @param[in] mults  Multiplier codes matching fields (e.g., "-000000" for no mults)
     * @param[in] fmt    Format string defining field types
     * @param[in] ...    Variable arguments matching format string
     * 
     * Example:
     * @code
     * logger.Write("CTUN", "TimeUS,ThI,ThO,DAlt,Alt", "Qffff", "s----", "-0000",
     *              AP_HAL::micros64(),
     *              throttle_in, throttle_out, 
     *              desired_alt, current_alt);
     * @endcode
     * 
     * @see Write() for version without units
     */
    void Write(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, ...);
    
    /**
     * @brief Write lower-priority streaming message (rate-limited)
     * 
     * @details Similar to Write() but marks message as streaming/telemetry.
     * Streaming messages are subject to rate limiting and may be dropped under
     * high logging load to preserve bandwidth for critical data.
     * 
     * Use for:
     * - Telemetry data (non-critical)
     * - Slow-rate sensor data
     * - Diagnostic information
     * - Status updates
     * 
     * @param[in] name   Message name
     * @param[in] labels Field labels
     * @param[in] fmt    Format string
     * @param[in] ...    Variable arguments
     * 
     * @note May be dropped if logging rate exceeds _params.file_ratemax
     * @see Write() for normal priority
     */
    void WriteStreaming(const char *name, const char *labels, const char *fmt, ...);
    
    /**
     * @brief Write lower-priority streaming message with units
     * 
     * @param[in] name   Message name
     * @param[in] labels Field labels
     * @param[in] units  Unit codes
     * @param[in] mults  Multiplier codes
     * @param[in] fmt    Format string
     * @param[in] ...    Variable arguments
     * 
     * @see WriteStreaming() without units
     */
    void WriteStreaming(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, ...);
    
    /**
     * @brief Write high-priority critical message (bypasses rate limits)
     * 
     * @details Similar to Write() but marks message as critical. Critical
     * messages bypass rate limiting and have higher priority in backend queues.
     * Should not be dropped unless buffers completely full.
     * 
     * Use for:
     * - Safety-critical state changes
     * - Error conditions
     * - Arming/disarming events
     * - Failsafe activations
     * - Mode changes
     * 
     * @param[in] name   Message name
     * @param[in] labels Field labels
     * @param[in] fmt    Format string
     * @param[in] ...    Variable arguments
     * 
     * @warning Use sparingly - excessive critical messages defeat priority system
     * @see Write() for normal priority
     */
    void WriteCritical(const char *name, const char *labels, const char *fmt, ...);
    
    /**
     * @brief Write high-priority critical message with units
     * 
     * @param[in] name   Message name
     * @param[in] labels Field labels
     * @param[in] units  Unit codes
     * @param[in] mults  Multiplier codes
     * @param[in] fmt    Format string
     * @param[in] ...    Variable arguments
     * 
     * @see WriteCritical() without units
     */
    void WriteCritical(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, ...);
    
    /**
     * @brief Internal Write variant using va_list for variable arguments
     * 
     * @details Core implementation of Write functions. All Write/WriteStreaming/
     * WriteCritical variants call this with appropriate flags. Handles:
     * - Format structure lookup or allocation via msg_fmt_for_name()
     * - FMT message emission on first use via Safe_Write_Emit_FMT()
     * - Message packing according to format string
     * - Backend distribution via FOR_EACH_BACKEND
     * 
     * @param[in] name         Message name
     * @param[in] labels       Field labels
     * @param[in] units        Unit codes (or nullptr)
     * @param[in] mults        Multiplier codes (or nullptr)
     * @param[in] fmt          Format string
     * @param[in] arg_list     Variable argument list
     * @param[in] is_critical  True for critical priority (default: false)
     * @param[in] is_streaming True for streaming/rate-limited (default: false)
     * 
     * @note Used internally - prefer public Write/WriteStreaming/WriteCritical
     * 
     * @see Write(), WriteStreaming(), WriteCritical()
     */
    void WriteV(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, va_list arg_list, bool is_critical=false, bool is_streaming=false);

    void Write_PID(uint8_t msg_type, const class AP_PIDInfo &info);

    /**
     * @brief Check if message category should be logged based on bitmask
     * 
     * @details Tests if given log category bitmask is enabled in current
     * logging configuration. Used to filter messages before formatting to
     * avoid unnecessary CPU overhead when category disabled.
     * 
     * Common Log Masks (from vehicle):
     * - MASK_LOG_ATTITUDE_FAST: High-rate attitude data
     * - MASK_LOG_ATTITUDE_MED: Medium-rate attitude data
     * - MASK_LOG_GPS: GPS position data
     * - MASK_LOG_IMU: Raw IMU data
     * - MASK_LOG_RCIN: RC input
     * - MASK_LOG_RCOUT: Servo/motor outputs
     * - MASK_LOG_NTUN: Navigation tuning
     * - MASK_LOG_CTUN: Control tuning
     * - Many more vehicle-specific masks
     * 
     * @param[in] mask Bitmask to test against log_bitmask parameter
     * 
     * @return true if this message category should be logged
     * 
     * Example:
     * @code
     * if (logger.should_log(MASK_LOG_ATTITUDE_FAST)) {
     *     logger.Write("ATT", "TimeUS,Roll,Pitch,Yaw", "Qfff",
     *                  AP_HAL::micros64(), roll, pitch, yaw);
     * }
     * @endcode
     * 
     * @note Check before formatting expensive log messages to save CPU
     * @see _log_bitmask parameter
     */
    bool should_log(uint32_t mask) const;

    /**
     * @brief Check if logging has started (log file open and active)
     * 
     * @details Returns true if at least one backend has started logging
     * (file open, ready to record). Used to detect if logging system is
     * operational.
     * 
     * @return true if logging active, false if no backends logging
     * 
     * @note May be false if disabled, no storage, or insufficient free space
     * @see logging_enabled() for configuration check
     * @see logging_failed() for error detection
     */
    bool logging_started(void) const;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    // currently only AP_Logger_File support this:
    void flush(void);
#endif

    void handle_mavlink_msg(class GCS_MAVLINK &, const mavlink_message_t &msg);

    void periodic_tasks(); // may want to split this into GCS/non-GCS duties

    // We may need to make sure data is loggable before starting the
    // EKF; when allow_start_ekf we should be able to log that data
    bool allow_start_ekf() const;

    // number of blocks that have been dropped
    uint32_t num_dropped(void) const;

    // access to public parameters
    void set_force_log_disarmed(bool force_logging) { _force_log_disarmed = force_logging; }
    void set_long_log_persist(bool b) { _force_long_log_persist = b; }
    bool log_while_disarmed(void) const;
    bool in_log_persistance(void) const;
    uint8_t log_replay(void) const { return _params.log_replay; }

    vehicle_startup_message_Writer _vehicle_messages;

    enum class LogDisarmed : uint8_t {
        NONE = 0,
        LOG_WHILE_DISARMED = 1,
        LOG_WHILE_DISARMED_NOT_USB = 2,
        LOG_WHILE_DISARMED_DISCARD = 3,
    };

    /**
     * @brief AP_Param metadata for parameter system integration
     * 
     * @details Defines parameters exposed to ground control station.
     * Parameters stored in EEPROM and configurable via MAVLink.
     */
    static const struct AP_Param::GroupInfo        var_info[];
    
    /**
     * @brief Logger configuration parameters
     * 
     * @details Configurable parameters controlling logging behavior, performance,
     * and backend selection. All parameters prefixed with LOG_ in ground station.
     * 
     * Parameter Descriptions:
     * 
     * backend_types (LOG_BACKEND_TYPE):
     * - Bitmask selecting active backends
     * - 0x01: Filesystem (SD card)
     * - 0x02: MAVLink streaming
     * - 0x04: Block device (DataFlash)
     * - Default: typically 0x01 (file) or 0x03 (file+MAVLink)
     * - Units: bitmask
     * 
     * file_bufsize (LOG_FILE_BUFSIZE):
     * - Filesystem backend buffer size in kilobytes
     * - Range: 8-64 KB typically
     * - Default: 16 KB
     * - Larger = better write performance, more RAM usage
     * - Units: kilobytes
     * 
     * file_disarm_rot (LOG_FILE_DSRMROT):
     * - Number of logs to keep when disarmed
     * - Older logs auto-deleted to free space
     * - Default: 10
     * - Units: count
     * 
     * log_disarmed (LOG_DISARMED):
     * - Logging behavior when disarmed
     * - 0: No logging when disarmed
     * - 1: Log while disarmed
     * - 2: Log while disarmed except when on USB power
     * - 3: Log while disarmed, discard on arming
     * - Default: 0 (no disarmed logging)
     * - Units: enum
     * 
     * log_replay (LOG_REPLAY):
     * - Enable log replay mode for algorithm development
     * - 0: Disabled
     * - 1: Enabled
     * - Default: 0
     * - Units: boolean
     * 
     * mav_bufsize (LOG_MAV_BUFSIZE):
     * - MAVLink backend buffer size in kilobytes
     * - Range: 8-32 KB typically
     * - Default: 8 KB
     * - Units: kilobytes
     * 
     * file_timeout (LOG_FILE_TIMEOUT):
     * - Close log file after this many seconds without writes
     * - Prevents keeping files open indefinitely
     * - Default: 5 seconds
     * - Units: seconds
     * 
     * min_MB_free (LOG_FILE_MB_FREE):
     * - Stop logging when free space below this threshold
     * - Prevents filling storage completely
     * - Default: 500 MB
     * - Units: megabytes
     * 
     * file_ratemax (LOG_FILE_RATEMAX):
     * - Maximum logging rate for filesystem backend
     * - 0: Unlimited
     * - >0: Maximum Hz for non-critical messages
     * - Default: 0 (unlimited)
     * - Typical: 50-400 Hz when enabled
     * - Units: Hz
     * 
     * mav_ratemax (LOG_MAV_RATEMAX):
     * - Maximum logging rate for MAVLink streaming backend
     * - Critical for limited telemetry bandwidth
     * - Default: 10 Hz
     * - Typical: 5-50 Hz depending on link speed
     * - Units: Hz
     * 
     * blk_ratemax (LOG_BLK_RATEMAX):
     * - Maximum logging rate for block device backend
     * - 0: Unlimited
     * - Default: 0
     * - Units: Hz
     * 
     * disarm_ratemax (LOG_DARM_RATEMAX):
     * - Maximum rate when disarmed (if LOG_DISARMED enabled)
     * - Reduces storage usage during ground operations
     * - Default: 0 (unlimited)
     * - Typical: 1-10 Hz when enabled
     * - Units: Hz
     * 
     * max_log_files (LOG_MAX_FILES):
     * - Maximum number of log files to keep
     * - Oldest deleted when exceeded
     * - 0: Unlimited (limited by storage)
     * - Default: 0
     * - Typical: 10-500 depending on storage size
     * - Units: count
     * 
     * @note Parameter changes take effect on next logging start
     * @note Rate limits apply to streaming messages, not critical messages
     */
    struct {
        AP_Int8 backend_types;
        AP_Int16 file_bufsize; // in kilobytes
        AP_Int8 file_disarm_rot;
        AP_Enum<LogDisarmed> log_disarmed;
        AP_Int8 log_replay;
        AP_Int8 mav_bufsize; // in kilobytes
        AP_Int16 file_timeout; // in seconds
        AP_Int16 min_MB_free;
        AP_Float file_ratemax;
        AP_Float mav_ratemax;
        AP_Float blk_ratemax;
        AP_Float disarm_ratemax;
        AP_Int16 max_log_files;
    } _params;

    const struct LogStructure *structure(uint16_t num) const;
    const struct UnitStructure *unit(uint16_t num) const;
    const struct MultiplierStructure *multiplier(uint16_t num) const;

    // methods for mavlink SYS_STATUS message (send_sys_status)
    // these methods cover only the first logging backend used -
    // typically AP_Logger_File.
    bool logging_present() const;
    bool logging_enabled() const;
    bool logging_failed() const;

    // notify logging subsystem of an arming failure. This triggers
    // logging for HAL_LOGGER_ARM_PERSIST seconds
    void arming_failure() {
        _last_arming_failure_ms = AP_HAL::millis();
#if HAL_LOGGER_FILE_CONTENTS_ENABLED
        file_content_prepare_for_arming = true;
#endif
    }

    void set_vehicle_armed(bool armed_state);
    bool vehicle_is_armed() const { return _armed; }

    void handle_log_send();
    bool in_log_download() const;

    float quiet_nanf() const { return NaNf; } // "AR"
    double quiet_nan() const { return nan("0x4152445550490a"); } // "ARDUPI"

    // returns true if msg_type is associated with a message
    bool msg_type_in_use(uint8_t msg_type) const;

    // calculate the length of a message using fields specified in
    // fmt; includes the message header
    int16_t Write_calc_msg_len(const char *fmt) const;

    /**
     * @brief Dynamic message format structure for runtime-registered messages
     * 
     * @details Linked list node containing format definition for messages
     * registered via Write() calls. Similar to LogStructure but maintains
     * pointer linkage for efficient lookup by name.
     * 
     * Used for dynamic message registration when message types are not known
     * at compile time. Write() calls with new message names allocate these
     * structures and link them into log_write_fmts list.
     * 
     * Structure Members:
     * - next: Pointer to next format in linked list
     * - msg_type: Allocated message type ID (0-255)
     * - msg_len: Total message length in bytes including header
     * - name: Message name (e.g., "ATT", "CTUN"), up to 4 chars
     * - fmt: Format string defining field types (e.g., "Qfff")
     * - labels: Comma-separated field names (e.g., "TimeUS,Roll,Pitch,Yaw")
     * - units: Unit codes for each field (e.g., "sddd")
     * - mults: Multiplier codes for each field (e.g., "-000")
     * 
     * @note Protected by log_write_fmts_sem for thread-safe access
     * @see msg_fmt_for_name() for format lookup/allocation
     * @see LogStructure.h for compile-time format definitions
     */
    // this structure looks much like struct LogStructure in
    // LogStructure.h, however we need to remember a pointer value for
    // efficiency of finding message types
    struct log_write_fmt {
        struct log_write_fmt *next;
        uint8_t msg_type;
        uint8_t msg_len;
        const char *name;
        const char *fmt;
        const char *labels;
        const char *units;
        const char *mults;
    } *log_write_fmts;

    /**
     * @brief Find or allocate log_write_fmt for message name
     * 
     * @details Searches log_write_fmts linked list for existing format with
     * matching name. If not found and space available, allocates new format
     * structure and assigns free message type ID via find_free_msg_type().
     * 
     * First call with a name:
     * 1. Allocates log_write_fmt structure
     * 2. Assigns unique msg_type ID
     * 3. Calculates msg_len from format string
     * 4. Links into log_write_fmts list
     * 5. Returns format for use by Write() methods
     * 
     * Subsequent calls:
     * - Returns cached format structure
     * - Fast O(n) lookup where n = number of dynamic formats
     * 
     * @param[in] name         Message name (up to 4 characters)
     * @param[in] labels       Comma-separated field labels
     * @param[in] units        Unit codes (or nullptr)
     * @param[in] mults        Multiplier codes (or nullptr)
     * @param[in] fmt          Format string
     * @param[in] direct_comp  Direct comparison mode (default: false)
     * @param[in] copy_strings Copy string pointers vs reference (default: false)
     * 
     * @return Pointer to log_write_fmt structure, or nullptr if no msg_type available
     * 
     * @note Thread-safe via log_write_fmts_sem
     * @warning Returns nullptr if all 256 message type IDs exhausted
     * 
     * @see find_free_msg_type() for ID allocation
     * @see Safe_Write_Emit_FMT() for FMT message emission
     */
    struct log_write_fmt *msg_fmt_for_name(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, const bool direct_comp = false, const bool copy_strings = false);

    /**
     * @brief Emit FMT message for format structure to all backends
     * 
     * @details Writes FMT message defining message structure to all backends.
     * FMT messages must be logged before first data message of that type so
     * log analysis tools can parse message structure.
     * 
     * FMT Message Contents:
     * - Message type ID
     * - Message length in bytes
     * - Message name
     * - Format string
     * - Field labels
     * - Unit codes
     * - Multiplier codes
     * 
     * Safe in that it only emits FMT once per backend per format, tracked
     * by backend-specific state.
     * 
     * @param[in] f Pointer to log_write_fmt structure to emit
     * 
     * @note Called automatically by Write() on first use of message type
     * @note Idempotent - safe to call multiple times
     * 
     * @see msg_fmt_for_name() for format allocation
     */
    void Safe_Write_Emit_FMT(log_write_fmt *f);

    // get count of number of times we have started logging
    uint8_t get_log_start_count(void) const {
        return _log_start_count;
    }

    // add a filename to list of files to log. The name is copied internally so
    // the pointer passed can be freed after return.
    void log_file_content(const char *name);

protected:

    const struct LogStructure *_structures;
    uint8_t _num_types;
    const struct UnitStructure *_units = log_Units;
    const struct MultiplierStructure *_multipliers = log_Multipliers;
    const uint8_t _num_units = (sizeof(log_Units) / sizeof(log_Units[0]));
    const uint8_t _num_multipliers = (sizeof(log_Multipliers) / sizeof(log_Multipliers[0]));

    /**
     * @brief Write binary block with explicit priority control
     * 
     * @details Protected method for writing binary blocks with explicit
     * priority flag. Used internally by WriteBlock() and WriteCriticalBlock()
     * to implement priority-based message queuing.
     * 
     * Priority Behavior:
     * - is_critical=true: Bypasses rate limits, higher queue priority
     * - is_critical=false: Subject to rate limits, normal priority
     * 
     * @param[in] pBuffer    Pointer to binary data
     * @param[in] size       Number of bytes to write
     * @param[in] is_critical True for high priority, false for normal
     * 
     * @note Used internally - external code should use WriteBlock() or
     * WriteCriticalBlock() for clearer intent
     * 
     * @see WriteBlock() for normal priority writes
     * @see WriteCriticalBlock() for critical priority writes
     */
    /* Write a block with specified importance */
    /* might be useful if you have a boolean indicating a message is
     * important... */
    void WritePrioritisedBlock(const void *pBuffer, uint16_t size,
                               bool is_critical);

private:
    /**
     * @brief Maximum number of simultaneous logging backends
     * 
     * @details Supports up to 2 backends running concurrently, typically:
     * - Backend 0: AP_Logger_File (SD card logging)
     * - Backend 1: AP_Logger_MAVLink (telemetry streaming)
     * 
     * Example configuration: Log to SD card while streaming critical
     * messages to ground station over MAVLink link.
     */
    #define LOGGER_MAX_BACKENDS 2
    
    /**
     * @brief Index for next backend allocation during initialization
     * 
     * @details Incremented during init() as backends are probed and added.
     * Used to track how many backends are currently active.
     */
    uint8_t _next_backend;
    
    /**
     * @brief Array of active logging backend instances
     * 
     * @details Populated during init() by probing available backends:
     * - AP_Logger_File::probe() - Checks for SD card/filesystem
     * - AP_Logger_Block::probe() - Checks for DataFlash chip
     * - AP_Logger_MAVLink backend added if MAVLink enabled
     * 
     * FOR_EACH_BACKEND macro iterates this array for broadcast operations.
     */
    AP_Logger_Backend *backends[LOGGER_MAX_BACKENDS];
    
    /**
     * @brief Reference to log bitmask parameter for should_log() checks
     * 
     * @details Points to vehicle-specific LOG_BITMASK parameter used to
     * filter which message types are logged based on user configuration.
     * Each bit enables a category of messages (e.g., attitude, GPS, RC input).
     */
    const AP_Int32 *_log_bitmask;

    /**
     * @enum Backend_Type
     * @brief Bit flags identifying available logging backend types
     * 
     * @details Used in _params.backend_types to configure which backends
     * to initialize. Multiple backends can be OR'd together:
     * 
     * Example: backend_types = FILESYSTEM | MAVLINK enables SD card
     * logging and MAVLink streaming simultaneously.
     */
    enum class Backend_Type : uint8_t {
        NONE       = 0,         ///< No backends enabled
        FILESYSTEM = (1<<0),    ///< SD card/filesystem logging (AP_Logger_File)
        MAVLINK    = (1<<1),    ///< MAVLink streaming (AP_Logger_MAVLink)
        BLOCK      = (1<<2),    ///< Block flash device (AP_Logger_Block/DataFlash)
    };

    enum class RCLoggingFlags : uint8_t {
        HAS_VALID_INPUT = 1U<<0,  // true if the system is receiving good RC values
        IN_RC_FAILSAFE =  1U<<1,  // true if the system is current in RC failsafe
    };

    /**
     * @brief Support for dynamic Write; user-supplies name, format,
     * labels and values in a single function call.
     * 
     * @details Semaphore protecting log_write_fmts linked list during
     * concurrent format registration and lookup operations.
     * 
     * Thread Safety:
     * - Acquired during msg_fmt_for_name() format allocation
     * - Acquired during log_write_fmt_for_msg_type() lookups
     * - Prevents race conditions when multiple threads register formats
     * 
     * @note Use WITH_SEMAPHORE(log_write_fmts_sem) for RAII lock management
     */
    HAL_Semaphore log_write_fmts_sem;

    /**
     * @brief Lookup registered format structure by message type ID
     * 
     * @param[in] msg_type  Numeric message type (0-255)
     * @return Pointer to log_write_fmt if found, nullptr otherwise
     * 
     * @note Thread-safe via log_write_fmts_sem
     */
    const struct log_write_fmt *log_write_fmt_for_msg_type(uint8_t msg_type) const;

    /**
     * @brief Lookup predefined LogStructure by message type ID
     * 
     * @details Searches _structures array registered during init() for
     * static message definitions from vehicle LogStructure tables.
     * 
     * @param[in] msg_type  Numeric message type (0-255)
     * @return Pointer to LogStructure if found, nullptr otherwise
     */
    const struct LogStructure *structure_for_msg_type(uint8_t msg_type) const;

    /**
     * @brief Find unused message type ID for dynamic format allocation
     * 
     * @details Scans 0-255 range for msg_type not currently used by:
     * - Predefined structures (_structures array)
     * - Dynamically registered formats (log_write_fmts list)
     * 
     * @return Available msg_type (0-255), or -1 if all IDs exhausted
     * 
     * @warning Returns -1 when 256 message types exceeded - caller must check
     */
    int16_t find_free_msg_type() const;

    /**
     * @brief Populate LogStructure from registered message type
     * 
     * @details Fills logstruct with format information by searching both
     * predefined structures and dynamically registered formats.
     * 
     * @param[out] logstruct  Structure to populate with format info
     * @param[in]  msg_type   Message type to query
     * @return true if msg_type found and logstruct populated, false otherwise
     */
    bool fill_logstructure(struct LogStructure &logstruct, const uint8_t msg_type) const;

    bool _armed;

    // state to help us not log unnecessary RCIN values:
    bool should_log_rcin2;

    void Write_Compass_instance(uint64_t time_us, uint8_t mag_instance);

    void backend_starting_new_log(const AP_Logger_Backend *backend);

    static AP_Logger *_singleton;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    bool validate_structure(const struct LogStructure *logstructure, int16_t offset);
    void validate_structures(const struct LogStructure *logstructures, const uint8_t num_types);
    void dump_structure_field(const struct LogStructure *logstructure, const char *label, const uint8_t fieldnum);
    void dump_structures(const struct LogStructure *logstructures, const uint8_t num_types);
    bool assert_same_fmt_for_name(const log_write_fmt *f,
                                  const char *name,
                                  const char *labels,
                                  const char *units,
                                  const char *mults,
                                  const char *fmt) const;
    const char* unit_name(const uint8_t unit_id);
    double multiplier_name(const uint8_t multiplier_id);
    bool seen_ids[256] = { };
    bool labels_string_is_good(const char *labels) const;
#endif

    bool _writes_enabled:1;
    bool _force_log_disarmed:1;
    bool _force_long_log_persist:1;

    struct log_write_fmt_strings {
        char name[LS_NAME_SIZE];
        char format[LS_FORMAT_SIZE];
        char labels[LS_LABELS_SIZE];
        char units[LS_UNITS_SIZE];
        char multipliers[LS_MULTIPLIERS_SIZE];
    };

    /**
     * @brief Save message format for log replay functionality
     * 
     * @details Stores format information needed by AP_DAL (Data Abstraction Layer)
     * for log replay and EKF development. Captures FMT messages during logging
     * for later reconstruction of log structure.
     * 
     * @param[in] pBuffer  Pointer to FMT message buffer
     * 
     * @note Only used when LOG_REPLAY parameter is non-zero
     */
    void save_format_Replay(const void *pBuffer);

    /**
     * @brief I/O thread initialization flag
     * 
     * @details Set to true after hal.scheduler->thread_create() successfully
     * launches io_thread(). Prevents duplicate thread creation.
     */
    bool _io_thread_started;

    /**
     * @brief Initialize dedicated I/O thread for non-blocking filesystem operations
     * 
     * @details Creates separate thread via hal.scheduler->thread_create() with
     * priority lower than main loop. I/O thread handles:
     * - Filesystem writes to SD card
     * - Block flash device operations
     * - Buffer flushing and synchronization
     * 
     * Thread Model:
     * - Stack size: Typically 2KB
     * - Priority: Lower than main control loop
     * - Runs continuously, sleeping when no I/O pending
     * 
     * @note Called automatically during init() - manual invocation not required
     */
    void start_io_thread(void);
    
    /**
     * @brief I/O thread main loop - handles blocking filesystem operations
     * 
     * @details Executes in dedicated thread context, processing:
     * - Backend periodic_tasks() for I/O operations
     * - Crash dump persistence checks
     * - File content logging (if enabled)
     * - Heartbeat monitoring
     * 
     * Isolation Strategy:
     * - All blocking I/O isolated from main control loop
     * - Semaphores coordinate buffer access with main thread
     * - Sleep/yield when no work to prevent CPU hogging
     * 
     * @warning Never call directly - invoked by HAL scheduler thread system
     */
    void io_thread();
    
    /**
     * @brief Check for crash dump data and persist to storage
     * 
     * @details Queries HAL for crash dump information from previous boot
     * (e.g., hard fault handler captured registers, stack traces) and
     * writes to persistent storage for post-mortem analysis.
     * 
     * Crash Dump Sources:
     * - ARM Cortex-M hard fault handler preserved data
     * - Watchdog reset information
     * - Stack overflow detection results
     * 
     * @return true if crash dump found and saved, false otherwise
     * 
     * @note Called early in io_thread() startup before normal logging begins
     */
    bool check_crash_dump_save(void);

#if HAL_LOGGER_FILE_CONTENTS_ENABLED
    /**
     * @brief Support for logging file content - embed system files in dataflash logs
     * 
     * @details Allows logging complete contents of configuration files, parameter
     * files, or firmware metadata into the flight log for comprehensive state
     * capture and debugging.
     * 
     * Use Cases:
     * - Log current parameter file at arming for exact configuration capture
     * - Log hwdef.dat to record hardware configuration
     * - Log mission file for complete mission reconstruction
     * - Log scripts for Lua scripting diagnostics
     */
    
    /**
     * @struct file_list
     * @brief Linked list node for files queued for content logging
     * 
     * @details Each node represents one file to be embedded in the log.
     * Files are processed sequentially by io_thread to avoid blocking.
     */
    struct file_list {
        struct file_list *next;      ///< Next file in queue
        const char *filename;         ///< Source filesystem path (heap-allocated copy)
        char log_filename[16];        ///< Short name for FILA message (filename embedded in log)
    };
    
    /**
     * @struct FileContent
     * @brief State machine for incremental file content logging
     * 
     * @details Manages streaming file contents into log messages without
     * blocking. Files are read and logged in chunks over multiple io_thread
     * iterations to prevent stalls.
     */
    struct FileContent {
        void reset();                                    ///< Clear queue and close open file
        void remove_and_free(file_list *victim);        ///< Remove node from queue and free memory
        struct file_list *head, *tail;                   ///< Queue of files to log
        int fd{-1};                                      ///< Currently open file descriptor (-1 = none)
        uint32_t offset;                                 ///< Current read position in file
        bool fast;                                       ///< Fast mode (at arming) vs slow mode (normal)
        uint8_t counter;                                 ///< Unique ID counter for FILA messages
        HAL_Semaphore sem;                               ///< Protects queue from concurrent access
    };
    
    /**
     * @brief Normal priority file content queue
     * 
     * @details Files added via log_file_content() API during normal operation.
     * Processed slowly in background to minimize I/O impact.
     */
    FileContent normal_file_content;
    
    /**
     * @brief High priority file content queue for arming-time snapshot
     * 
     * @details Files logged rapidly at arming time to capture exact vehicle
     * configuration state. Processed with higher I/O priority.
     */
    FileContent at_arm_file_content;

    /**
     * @brief Flag indicating arming preparation is pending
     * 
     * @details Set true when PrepForArming() called, triggers
     * prepare_at_arming_sys_file_logging() to populate at_arm_file_content
     * queue with critical system files.
     */
    bool file_content_prepare_for_arming;

    /**
     * @brief Process file content logging state machine
     * 
     * @details Called from io_thread() to incrementally read and log file
     * chunks. Processes both normal and at_arm queues with appropriate
     * priorities.
     */
    void file_content_update(void);

    /**
     * @brief Queue system files for logging at arming time
     * 
     * @details Automatically adds critical files to at_arm_file_content queue:
     * - Current parameter file (PARM.pck or @SYS/parameters.pck)
     * - Hardware definition (hwdef.dat)
     * - Other vehicle-specific configuration files
     * 
     * @note Called by PrepForArming() when file_content_prepare_for_arming is true
     */
    void prepare_at_arming_sys_file_logging();

#endif
    
    /**
     * @brief Support for retrieving logs via MAVLink protocol
     * 
     * @details Implements MAVLink log download protocol allowing ground control
     * stations to retrieve stored logs over telemetry link. State machine
     * handles LOG_REQUEST_LIST, LOG_REQUEST_DATA, LOG_ERASE, LOG_REQUEST_END
     * MAVLink messages.
     * 
     * Protocol Flow:
     * 1. GCS sends LOG_REQUEST_LIST → Enter LISTING state
     * 2. AP sends LOG_ENTRY packets for each log → Return to IDLE
     * 3. GCS sends LOG_REQUEST_DATA → Enter SENDING state  
     * 4. AP sends LOG_DATA packets until complete → Return to IDLE
     * 
     * Performance:
     * - Rate limited to prevent telemetry link saturation
     * - Configurable via _params.mav_ratemax (kB/s)
     * - Typical download time: 10-60 minutes depending on log size and link speed
     * 
     * @note Only one log transfer can be active at a time
     * @warning Log downloads can saturate low-bandwidth telemetry links
     */

    /**
     * @enum TransferActivity
     * @brief Current state of MAVLink log download state machine
     */
    enum class TransferActivity {
        IDLE,    ///< No active transfer, all file descriptors closed
        LISTING, ///< Actively sending LOG_ENTRY packets to enumerate logs
        SENDING, ///< Actively sending LOG_DATA packets for specific log
    } transfer_activity = TransferActivity::IDLE;

    /**
     * @brief Timestamp of last MAVLink log transfer message handled (milliseconds)
     * 
     * @details Used for transfer timeout detection. If too much time passes
     * without activity, transfer is aborted and state returns to IDLE.
     */
    uint32_t _last_mavlink_log_transfer_message_handled_ms;
    
    /**
     * @brief Flag indicating disarm warning message has been sent
     * 
     * @details When user attempts operations requiring vehicle disarm
     * (e.g., log erase), this tracks if warning message was already sent
     * to prevent spam.
     */
    bool _warned_log_disarm;

    /**
     * @brief Next log number to send in LISTING state
     * 
     * @details Incremented as LOG_ENTRY packets sent during log enumeration.
     * Range: 1 to _log_num_logs
     */
    uint16_t _log_next_list_entry;

    /**
     * @brief Last log number to send in LISTING state
     * 
     * @details Ending boundary for log enumeration. When _log_next_list_entry
     * exceeds this value, LISTING is complete.
     */
    uint16_t _log_last_list_entry;

    /**
     * @brief Total number of logs available for download
     * 
     * @details Cached count of log files on storage backend. Updated during
     * LOG_REQUEST_LIST handling.
     */
    uint16_t _log_num_logs;

    /**
     * @brief Log number currently being sent in SENDING state
     * 
     * @details Identifies which log file is being transferred via LOG_DATA packets.
     * Range: 1 to _log_num_logs
     */
    uint16_t _log_num_data;

    /**
     * @brief Current byte offset in log being sent
     * 
     * @details Position in log file for next LOG_DATA packet. Incremented
     * by packet payload size (typically 90 bytes) after each send.
     */
    uint32_t _log_data_offset;

    /**
     * @brief Total size of log being sent (bytes)
     * 
     * @details Retrieved from backend->get_log_info() when SENDING state entered.
     * Used to calculate transfer completion.
     */
    uint32_t _log_data_size;

    /**
     * @brief Number of bytes remaining to send in current log
     * 
     * @details Decremented as LOG_DATA packets sent. When reaches zero,
     * transfer is complete and state returns to IDLE.
     */
    uint32_t _log_data_remaining;

    /**
     * @brief Start page of log data (backend-specific)
     * 
     * @details For block-based backends (DataFlash), page number where log
     * starts. For filesystem backends, may be unused.
     */
    uint32_t _log_data_page;

    /**
     * @brief MAVLink channel currently handling log transfer
     * 
     * @details Pointer to GCS_MAVLINK instance that initiated transfer.
     * LOG_DATA packets sent on this specific channel to ensure correct routing.
     * nullptr when no transfer active.
     */
    GCS_MAVLINK *_log_sending_link;
    
    /**
     * @brief Semaphore protecting log transfer state variables
     * 
     * @details Prevents race conditions between:
     * - handle_log_send() (main thread) - sends LOG_DATA packets
     * - handle_log_message() (MAVLink thread) - processes requests
     * 
     * @note Use WITH_SEMAPHORE(_log_send_sem) for critical sections
     */
    HAL_Semaphore _log_send_sem;

    /**
     * @brief Timestamp of last arming failure (milliseconds)
     * 
     * @details Set by arming_failure() callback. Triggers short-term logging
     * persistence (HAL_LOGGER_ARM_PERSIST seconds, typically 20s) to capture
     * data around arming failures for diagnosis.
     * 
     * Use Case: User attempts arm → pre-arm check fails → logging continues
     * briefly to record sensor state and error conditions.
     */
    uint32_t _last_arming_failure_ms;

    /**
     * @brief Counter incremented each time logging starts
     * 
     * @details Used by other subsystems to detect new log session and trigger
     * one-time startup logging. When subsystem sees this value change, it
     * knows a new log file has been created.
     * 
     * Example: EKF logs initial state when _log_start_count increments
     * 
     * @note Wraps at 255 - subsystems should detect != rather than >
     */
    uint8_t _log_start_count;

    /**
     * @brief MAVLink message dispatcher for log transfer protocol
     * 
     * @details Routes incoming MAVLink messages to appropriate handlers:
     * - LOG_REQUEST_LIST → handle_log_request_list()
     * - LOG_REQUEST_DATA → handle_log_request_data()
     * - LOG_ERASE → handle_log_request_erase()
     * - LOG_REQUEST_END → handle_log_request_end()
     * 
     * @param msg  MAVLink message to process
     */
    void handle_log_message(class GCS_MAVLINK &, const mavlink_message_t &msg);

    /**
     * @brief Handle LOG_REQUEST_LIST - enumerate available logs
     * 
     * @details Initiates LISTING state, populates log count, and begins
     * sending LOG_ENTRY packets for each available log file.
     */
    void handle_log_request_list(class GCS_MAVLINK &, const mavlink_message_t &msg);
    
    /**
     * @brief Handle LOG_REQUEST_DATA - initiate log download
     * 
     * @details Initiates SENDING state for specified log number and begins
     * streaming LOG_DATA packets from requested offset.
     */
    void handle_log_request_data(class GCS_MAVLINK &, const mavlink_message_t &msg);
    
    /**
     * @brief Handle LOG_ERASE - delete all logs
     * 
     * @details Erases all logs on storage backend if vehicle is disarmed.
     * Sends warning message if attempted while armed.
     * 
     * @warning Irreversible operation - all flight data lost
     */
    void handle_log_request_erase(class GCS_MAVLINK &, const mavlink_message_t &msg);
    
    /**
     * @brief Handle LOG_REQUEST_END - abort current transfer
     * 
     * @details Cleanly terminates active LISTING or SENDING operation,
     * closes file descriptors, and returns to IDLE state.
     */
    void handle_log_request_end(class GCS_MAVLINK &, const mavlink_message_t &msg);
    
    /**
     * @brief Terminate log transfer and clean up resources
     * 
     * @details Common cleanup path called by handle_log_request_end() and
     * timeout handlers. Closes files, releases locks, resets state to IDLE.
     */
    void end_log_transfer();
    
    /**
     * @brief Process LISTING state - send LOG_ENTRY packets
     * 
     * @details Called from handle_log_send() to transmit log enumeration.
     * Sends one LOG_ENTRY packet per call, rate-limited by MAVLink bandwidth.
     */
    void handle_log_send_listing();
    
    /**
     * @brief Process SENDING state - send LOG_DATA packets
     * 
     * @details Called from handle_log_send() to transmit log contents.
     * Sends multiple LOG_DATA packets per call up to rate limit.
     */
    void handle_log_sending();
    
    /**
     * @brief Send single LOG_DATA packet chunk
     * 
     * @details Reads up to 90 bytes from backend, constructs LOG_DATA packet,
     * sends via _log_sending_link, updates offset and remaining counters.
     * 
     * @return true if packet sent successfully, false on read error
     */
    bool handle_log_send_data();

    /**
     * @brief Retrieve log metadata from backend
     * 
     * @details Queries backend for log file information.
     * 
     * @param[in]  log_num   Log number (1 to num_logs)
     * @param[out] size      Log file size in bytes
     * @param[out] time_utc  Log start time (Unix timestamp)
     */
    void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc);

    /**
     * @brief Read log data chunk from backend
     * 
     * @details Retrieves specified byte range from log file for transmission
     * or analysis.
     * 
     * @param[in]  log_num  Log number (1 to num_logs)
     * @param[in]  page     Page number (backend-specific, may be unused)
     * @param[in]  offset   Byte offset in log file
     * @param[in]  len      Number of bytes to read (max 90 for MAVLink packet)
     * @param[out] data     Buffer to receive log data
     * @return Number of bytes actually read, or -1 on error
     */
    int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data);

    /* end support for retrieving logs via mavlink: */

#if HAL_LOGGER_FILE_CONTENTS_ENABLED
    void log_file_content(FileContent &file_content, const char *filename);
    void file_content_update(FileContent &file_content);
#endif
};

/**
 * @namespace AP
 * @brief ArduPilot library namespace providing singleton accessors
 * 
 * @details Namespace accessor pattern for core ArduPilot singletons.
 * Provides cleaner syntax than static getInstance() methods and
 * centralizes singleton management.
 */
namespace AP {
    /**
     * @brief Access AP_Logger singleton instance
     * 
     * @details Primary access point for logging subsystem throughout ArduPilot.
     * Returns reference to global AP_Logger instance.
     * 
     * Usage Pattern:
     * @code
     * AP::logger().Write_Event(LogEvent::ARMED);
     * AP::logger().Write("CTUN", "TimeUS,Thr", "Qf", AP_HAL::micros64(), throttle);
     * @endcode
     * 
     * @return Reference to global AP_Logger singleton
     * 
     * @note Preferred over AP_Logger::get_singleton() for consistency
     * @see AP_Logger::get_singleton() for alternative access method
     */
    AP_Logger &logger();
};

/**
 * @def LOGGER_WRITE_ERROR
 * @brief Convenience macro for logging error events
 * 
 * @details Shorthand for AP::logger().Write_Error() to reduce verbosity
 * in error handling code. Used throughout ArduPilot for subsystem errors.
 * 
 * @param subsys  LogErrorSubsystem enum value (e.g., LogErrorSubsystem::GPS)
 * @param err     LogErrorCode enum value (e.g., LogErrorCode::UNHEALTHY)
 * 
 * Example:
 * @code
 * if (!compass.healthy()) {
 *     LOGGER_WRITE_ERROR(LogErrorSubsystem::COMPASS, LogErrorCode::UNHEALTHY);
 * }
 * @endcode
 */
#define LOGGER_WRITE_ERROR(subsys, err) AP::logger().Write_Error(subsys, err)

/**
 * @def LOGGER_WRITE_EVENT
 * @brief Convenience macro for logging vehicle state events
 * 
 * @details Shorthand for AP::logger().Write_Event() to reduce verbosity
 * in event logging code. Used throughout ArduPilot for state changes.
 * 
 * @param evt  LogEvent enum value (e.g., LogEvent::ARMED, LogEvent::FENCE_ENABLE)
 * 
 * Example:
 * @code
 * if (arm_checks_passed) {
 *     LOGGER_WRITE_EVENT(LogEvent::ARMED);
 *     motors->armed(true);
 * }
 * @endcode
 */
#define LOGGER_WRITE_EVENT(evt) AP::logger().Write_Event(evt)

#else

#define LOGGER_WRITE_ERROR(subsys, err)
#define LOGGER_WRITE_EVENT(evt)

#endif  // HAL_LOGGING_ENABLED
