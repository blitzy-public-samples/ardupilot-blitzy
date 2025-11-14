/**
 * @file Parameters.h
 * @brief Parameter declarations and Parameters class for AntennaTracker configuration
 * 
 * @details Defines all configurable parameters for the AntennaTracker vehicle, organized
 *          in the Parameters class with AP_Param integration for persistent storage.
 *          Parameters include servo control PID tuning, mechanical limits, servo types,
 *          tracking behavior, MAVLink configuration, and logging options.
 *          
 *          All parameters are stored in EEPROM for persistence across reboots and are
 *          accessible via MAVLink for ground station configuration. Default values are
 *          defined in Parameters.cpp var_info table.
 *          
 * @note Parameter name format follows ArduPilot conventions with namespace prefixes.
 *       Changes to k_param_* enum values or parameter meanings require k_format_version update.
 * 
 * @see Parameters.cpp for var_info table with parameter metadata and defaults
 * @see AP_Param library for parameter storage and access mechanisms
 */

#pragma once

#define AP_PARAM_VEHICLE_NAME tracker

#include <AC_PID/AC_PID.h>
#include <AP_Param/AP_Param.h>

/**
 * @class Parameters
 * @brief Container for all AntennaTracker configuration parameters
 * 
 * @details This class holds all configurable parameter objects (AP_Int8, AP_Int16, AP_Float,
 *          AC_PID, etc.) that define the tracker's behavior. Parameters are integrated with
 *          the AP_Param system for:
 *          - Persistent storage in EEPROM with format versioning
 *          - MAVLink parameter protocol support for GCS configuration
 *          - Default value initialization from var_info table
 *          - Type-safe parameter access throughout the codebase
 *          
 *          The k_param_* enum defines unique keys for each parameter group in EEPROM storage.
 *          The k_format_version must be incremented when parameter meanings or enum values change
 *          to invalidate old EEPROM data and force parameter reset.
 *          
 * @note All parameter objects are public for direct access by vehicle code.
 *       The constructor initializes PID controllers with default tuning values.
 */
class Parameters {
public:

    /**
     * @brief EEPROM parameter format version for AntennaTracker
     * 
     * @details The value of k_format_version determines whether existing EEPROM parameter
     *          data is considered valid. When the stored format version doesn't match this
     *          value, all parameters are reset to defaults, forcing users to reconfigure.
     * 
     *          You should ONLY change this value under these circumstances:
     *          1) The meaning of an existing EEPROM parameter changes
     *          2) The value of an existing k_param_* enum value changes
     * 
     *          Adding a new parameter should NOT require a format version change except
     *          under special circumstances. Changing this value forces ALL AntennaTracker
     *          users to reload all their parameters, which should be extremely rare.
     * 
     * @warning STOP!!! DO NOT CHANGE THIS VALUE UNTIL YOU FULLY UNDERSTAND THE IMPLICATIONS.
     *          IF UNSURE, ASK ANOTHER DEVELOPER!!!
     * 
     * @note To verify if a k_param_* value has changed, use C++ enum rules to calculate
     *       the implicit integer values of neighboring enum entries.
     * 
     * @see AP_Param::check_var_info() for format version validation
     */
    static const uint16_t k_format_version = 1;

    /**
     * @brief Parameter storage key enumeration for EEPROM addressing
     * 
     * @details Defines unique keys for each parameter or parameter group stored in EEPROM.
     *          These keys determine the storage location and are used by AP_Param for
     *          serialization and deserialization. The enum values are implicitly assigned
     *          sequential integers by the compiler unless explicitly specified.
     * 
     *          Key ranges are organized by functional groups:
     *          - 0-99: Core system parameters
     *          - 100-199: System configuration (GCS, serial, sensors)
     *          - 200-219: Radio/servo control parameters
     *          - 220+: Mission, logging, and vehicle-specific parameters
     * 
     * @warning Changing enum values breaks EEPROM compatibility and requires k_format_version increment.
     *          Deprecated entries must remain in the enum to preserve numbering for remaining parameters.
     * 
     * @note Entries marked "unused" or "deprecated" are retained for EEPROM compatibility.
     *       Adding new parameters should use unused key slots or append to the end.
     */
    enum {
        // Layout version number, always key zero.
        //
        k_param_format_version = 0,
        k_param_software_type,      // deprecated

        k_param_gcs0_unused = 100,  // unused in ArduPilot-4.7
        k_param_gcs1_unused,        // unused in ArduPilot-4.7
        k_param_sysid_this_mav_old,
        k_param_sysid_my_gcs_old,
        k_param_serial0_baud,       // deprecated
        k_param_serial1_baud,       // deprecated
        k_param_imu,
        k_param_compass_enabled_deprecated,
        k_param_compass,
        k_param_ahrs,  // AHRS group
        k_param_barometer,
        k_param_scheduler,
        k_param_ins,
        k_param_sitl,
        k_param_pidPitch_old,   // deprecated
        k_param_pidYaw_old,     // deprecated
        k_param_gcs2_unused,        // unused in ArduPilot-4.7
        k_param_serial2_baud,       // deprecated

        k_param_yaw_slew_time,
        k_param_pitch_slew_time,
        k_param_min_reverse_time,

        k_param_start_latitude,
        k_param_start_longitude,
        k_param_startup_delay,
        k_param_BoardConfig,
        k_param_gps,
        k_param_scan_speed_unused, // deprecated
        k_param_proxy_mode_unused, // deprecated
        k_param_servo_pitch_type,
        k_param_onoff_yaw_rate,
        k_param_onoff_pitch_rate,
        k_param_onoff_yaw_mintime,
        k_param_onoff_pitch_mintime,
        k_param_yaw_trim,
        k_param_pitch_trim,
        k_param_yaw_range,
        k_param_pitch_range,	//deprecated
        k_param_distance_min,
        k_param_sysid_target,       // 138
        k_param_gcs3_unused,        // unused in ArduPilot-4.7
        k_param_log_bitmask,        // 140
        k_param_notify,
        k_param_can_mgr,
        k_param_battery,

        k_param_serial_manager_old = 144,     // serial manager library
        k_param_servo_yaw_type,
        k_param_alt_source,
        k_param_mavlink_update_rate,
        k_param_pitch_min,
        k_param_pitch_max,
        k_param_gcs4_unused,        // unused in ArduPilot-4.7
        k_param_gcs5_unused,        // unused in ArduPilot-4.7
        k_param_gcs6_unused,        // unused in ArduPilot-4.7

        //
        // 200 : Radio settings
        //
        k_param_channel_yaw_old = 200,
        k_param_channel_pitch_old,
        k_param_pidPitch2Srv,
        k_param_pidYaw2Srv,
        k_param_rc_channels,
        k_param_servo_channels,

        k_param_stats_old = 218,
        k_param_scripting_old = 219,

        //
        // 220: Waypoint data
        //
        k_param_command_total = 220,

        // 254,255: reserved
        k_param_gcs_pid_mask = 225,
        k_param_scan_speed_yaw,
        k_param_scan_speed_pitch,
        k_param_initial_mode,
        k_param_disarm_pwm,

        k_param_auto_opts,
        k_param_NavEKF2,
        k_param_NavEKF3,

        k_param_logger = 253, // 253 - Logging Group

        k_param_vehicle = 257, // vehicle common block of parameters
        k_param__gcs = 258,
    };

    /// @brief Parameter format version stored in EEPROM for compatibility checking
    AP_Int16 format_version;

    // ========================================================================
    // MAVLink System Configuration
    // ========================================================================
    
    /// @brief Target vehicle MAVLink system ID to track (identifies which vehicle to follow)
    AP_Int16 sysid_target;

    // ========================================================================
    // Servo Slew Rate and Timing Parameters
    // ========================================================================
    
    /// @brief Maximum yaw axis slew rate in degrees/second (limits servo speed for smooth tracking)
    AP_Float yaw_slew_time;
    
    /// @brief Maximum pitch axis slew rate in degrees/second (limits servo speed for smooth tracking)
    AP_Float pitch_slew_time;
    
    /// @brief Minimum time in seconds before reversing servo direction (prevents rapid oscillation)
    AP_Float min_reverse_time;
    
    /// @brief Yaw axis scan speed in degrees/second when in SCAN mode
    AP_Int16 scan_speed_yaw;
    
    /// @brief Pitch axis scan speed in degrees/second when in SCAN mode
    AP_Int16 scan_speed_pitch;

    // ========================================================================
    // Initial Position Configuration
    // ========================================================================
    
    /// @brief Starting latitude in degrees (tracker home position if GPS not available)
    AP_Float start_latitude;
    
    /// @brief Starting longitude in degrees (tracker home position if GPS not available)
    AP_Float start_longitude;

    // ========================================================================
    // Servo Type and Configuration
    // ========================================================================
    
    /// @brief Startup delay in seconds before enabling servo movement (allows mechanical settling)
    AP_Float startup_delay;
    
    /// @brief Pitch servo type: 0=Position, 1=OnOff, 2=ContinuousRotation
    AP_Int8  servo_pitch_type;
    
    /// @brief Yaw servo type: 0=Position, 1=OnOff, 2=ContinuousRotation
    AP_Int8  servo_yaw_type;
    
    /// @brief Altitude source for pitch calculation: 0=GPS, 1=Barometer
    AP_Int8  alt_source;
    
    /// @brief MAVLink telemetry update rate in Hz (how often to send position to GCS)
    AP_Int8  mavlink_update_rate;

    // ========================================================================
    // On/Off Servo Control Parameters (for bang-bang control mode)
    // ========================================================================
    
    /// @brief Yaw rate threshold in degrees/second for OnOff servo mode (dead zone for discrete control)
    AP_Float onoff_yaw_rate;
    
    /// @brief Pitch rate threshold in degrees/second for OnOff servo mode (dead zone for discrete control)
    AP_Float onoff_pitch_rate;
    
    /// @brief Minimum on-time in seconds for yaw OnOff servo (prevents rapid switching)
    AP_Float onoff_yaw_mintime;
    
    /// @brief Minimum on-time in seconds for pitch OnOff servo (prevents rapid switching)
    AP_Float onoff_pitch_mintime;

    // ========================================================================
    // Servo Trim and Mechanical Limits
    // ========================================================================
    
    /// @brief Yaw servo trim offset in degrees (calibration for zero-degree pointing)
    AP_Float yaw_trim;
    
    /// @brief Pitch servo trim offset in degrees (calibration for zero-degree pointing)
    AP_Float pitch_trim;
    
    /// @brief Yaw axis total range of motion in degrees (mechanical limit, ±range/2 from center)
    AP_Int16 yaw_range;
    
    /// @brief Minimum target distance in meters (targets closer than this are not tracked)
    AP_Int16 distance_min;
    
    /// @brief Minimum pitch angle in degrees (lower mechanical limit for pitch axis)
    AP_Int16 pitch_min;
    
    /// @brief Maximum pitch angle in degrees (upper mechanical limit for pitch axis)
    AP_Int16 pitch_max;

    // ========================================================================
    // GCS and Mode Configuration
    // ========================================================================
    
    /// @brief Bitmask for which PID values to send to GCS (for tuning visualization)
    AP_Int16 gcs_pid_mask;
    
    /// @brief Initial flight mode on boot (0=Manual, 1=Stop, 2=Scan, 3=Servo_Test, 4=Auto)
    AP_Int8  initial_mode;
    
    /// @brief Disarm servo behavior: 0=TRIM (center position), 1=ZERO (disable PWM output)
    AP_Int8 disarm_pwm;
    
    /// @brief AUTO mode option bitmask (bit flags for AUTO mode behavior customization)
    AP_Int8 auto_opts;

    // ========================================================================
    // Waypoint/Mission Configuration
    // ========================================================================
    
    /// @brief Number of stored waypoints (1 if HOME is set, 0 otherwise)
    AP_Int8 command_total;

    // ========================================================================
    // Logging Configuration
    // ========================================================================
    
    /// @brief Log message selection bitmask (enables/disables specific log message types)
    AP_Int32 log_bitmask;

    // ========================================================================
    // PID Controllers for Servo Control Loops
    // ========================================================================
    
    /**
     * @brief Pitch axis servo control PID controller
     * 
     * @details Converts pitch angle error (desired - actual) to servo output command.
     *          Default tuning: P=0.2, I=0.0, D=0.05, FF=0.02, IMAX=4000.0
     *          Runs in closed-loop to maintain accurate pitch pointing.
     * 
     * @note Tuning affects tracking accuracy and servo smoothness. Too aggressive causes oscillation.
     */
    AC_PID         pidPitch2Srv;
    
    /**
     * @brief Yaw axis servo control PID controller
     * 
     * @details Converts yaw angle error (desired - actual) to servo output command.
     *          Default tuning: P=0.2, I=0.0, D=0.05, FF=0.02, IMAX=4000.0
     *          Runs in closed-loop to maintain accurate yaw pointing.
     * 
     * @note Tuning affects tracking accuracy and servo smoothness. Too aggressive causes oscillation.
     */
    AC_PID         pidYaw2Srv;

    /**
     * @brief Constructor initializes PID controllers with default tuning values
     * 
     * @details Initializes both pitch and yaw PID controllers with:
     *          - P gain: 0.2 (proportional response to angle error)
     *          - I gain: 0.0 (no integral term by default)
     *          - D gain: 0.05 (damping term to reduce oscillation)
     *          - FF gain: 0.02 (feedforward term)
     *          - IMAX: 4000.0 (integral windup limit)
     *          - Filter: 0.1 Hz (D-term filtering)
     * 
     * @note These are starting values; actual tuning is loaded from EEPROM parameters.
     *       PID values are accessible via MAVLink for runtime tuning.
     */
    Parameters() :
        pidPitch2Srv(0.2, 0.0f, 0.05f, 0.02f, 4000.0f, 0.0f, 0.0f, 0.0f, 0.1f),
        pidYaw2Srv  (0.2, 0.0f, 0.05f, 0.02f, 4000.0f, 0.0f, 0.0f, 0.0f, 0.1f)
        {}
};

/**
 * @brief AP_Param metadata table for parameter storage and GCS access
 * 
 * @details Defined in Parameters.cpp, this table provides metadata for each parameter:
 *          - Parameter name (as displayed in ground station)
 *          - Storage key (k_param_* enum value)
 *          - Data type (AP_Int8, AP_Float, etc.)
 *          - Memory offset within Parameters class
 *          - Default value, units, min/max ranges
 *          - Help text for ground station display
 * 
 *          The AP_Param system uses this table to:
 *          - Serialize/deserialize parameters to/from EEPROM
 *          - Handle MAVLink parameter protocol requests
 *          - Validate parameter values against defined ranges
 *          - Provide parameter documentation to ground stations
 * 
 * @note This table must be kept synchronized with the Parameters class member variables
 *       and the k_param_* enumeration.
 * 
 * @see Parameters.cpp for the complete var_info table definition
 * @see AP_Param library for parameter system implementation details
 */
extern const AP_Param::Info var_info[];
