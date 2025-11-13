/**
 * @file Parameters.h
 * @brief Rover vehicle parameter definitions and AP_Param persistent storage declarations
 * 
 * @details This file defines all configurable parameters for the Rover vehicle type in ArduPilot.
 *          Parameters are stored persistently using the AP_Param system and are accessible via
 *          ground control stations for configuration and tuning.
 *          
 *          The parameter system is organized into two main classes:
 *          - Parameters: Legacy parameter group (keys 0-255)
 *          - ParametersG2: Second generation parameter group (key 109) for newer parameters
 *          
 *          Parameters control all aspects of rover behavior including:
 *          - Navigation (waypoint following, speed control, turn behavior)
 *          - Steering control (Ackermann vs skid steering)
 *          - Throttle control (cruise speed, limits)
 *          - Flight modes (manual, auto, guided, RTL, etc.)
 *          - Failsafe behavior (RC loss, GCS loss, EKF failure)
 *          - Sensor configuration (GPS, compass, rangefinder)
 *          - Safety features (arming checks, geofencing)
 *          
 * @note Parameters are persisted to EEPROM/flash storage via AP_Param and survive reboots.
 *       Changing parameter definitions requires incrementing k_format_version to trigger
 *       parameter migration and prevent incompatibilities.
 *       
 * @warning Modifying parameter keys, types, or removing parameters can break ground station
 *          compatibility and cause user configurations to be lost. Follow parameter deprecation
 *          guidelines when making changes.
 * 
 * @see Rover/Parameters.cpp for var_info[] tables that define parameter metadata
 * @see libraries/AP_Param/AP_Param.h for parameter storage system documentation
 * 
 * Source: Rover/Parameters.h
 */
#pragma once

#include <AP_Common/AP_Common.h>

#include "RC_Channel_Rover.h"
#include <AC_Avoidance/AC_Avoid.h>
#include "AC_Sprayer/AC_Sprayer.h"
#include <AP_AIS/AP_AIS.h>
#include <AP_Beacon/AP_Beacon.h>
#include <AP_Follow/AP_Follow.h>
#include <AP_Proximity/AP_Proximity.h>
#include "AP_Rally.h"
#include <AP_SmartRTL/AP_SmartRTL.h>
#include <AP_Stats/AP_Stats.h>
#include "AP_Torqeedo/AP_Torqeedo.h"
#include <AP_WindVane/AP_WindVane.h>

#define AP_PARAM_VEHICLE_NAME rover

/**
 * @class Parameters
 * @brief Main rover parameter container class with persistent storage via AP_Param
 * 
 * @details This class contains all primary rover parameters stored in the first parameter
 *          block (keys 0-255). The AP_Param system provides persistent storage across reboots
 *          and allows configuration via MAVLink from ground control stations.
 *          
 *          The class serves two main purposes:
 *          1. Defines parameter keys (enum) for organizing parameters into logical groups
 *          2. Declares parameter member variables as AP_Param types for persistent storage
 *          
 *          Parameter Organization:
 *          - Keys 0-9: Format version and core system settings
 *          - Keys 10-99: Miscellaneous settings (logging, modes, arming)
 *          - Keys 100-139: Telemetry and communication settings
 *          - Keys 140-149: Battery monitoring (mostly deprecated)
 *          - Keys 150-159: Navigation parameters (speed, waypoint behavior)
 *          - Keys 160-169: Radio settings (RC channels, mostly deprecated)
 *          - Keys 170-179: Throttle control parameters
 *          - Keys 180-189: Failsafe control parameters
 *          - Keys 190-209: Obstacle avoidance (rangefinder, mostly deprecated)
 *          - Keys 210-219: Driving mode configuration
 *          - Keys 220-229: Waypoint data (mostly unused)
 *          - Keys 230-239: PID controllers
 *          - Keys 240-252: Library objects (AHRS, INS, compass, etc.)
 *          
 *          Historical note: As the parameter space (0-255) filled up, newer parameters
 *          were moved to ParametersG2 (second generation) to avoid exhausting key space.
 *          
 * @note Parameter format version (k_format_version) must be incremented when parameter
 *       definitions change in incompatible ways. This triggers parameter conversion logic
 *       in Rover/Parameters.cpp to migrate old configurations to new format.
 *       
 * @warning Many parameter keys are marked "unused" or "deprecated" but must remain in the
 *          enum to maintain key numbering compatibility. Do not reuse deprecated keys
 *          without careful consideration of upgrade paths.
 * 
 * @see ParametersG2 for second-generation parameters (newer additions)
 * @see Rover/Parameters.cpp for var_info[] table with parameter metadata and defaults
 */
class Parameters {
public:
    /**
     * @brief Parameter format version number
     * 
     * @details When changing the parameter enum in an incompatible fashion, this value
     *          must be incremented by one. The increment prevents old parameters from being
     *          used incorrectly by newer code and triggers parameter conversion/migration
     *          logic in Parameters.cpp.
     *          
     *          Current version: 16
     *          
     * @note Incompatible changes include: removing parameters, changing parameter types,
     *       reordering parameters, or changing parameter key numbers.
     */
    static const uint16_t k_format_version = 16;

    /**
     * @enum Parameter key enumeration
     * @brief Defines parameter storage keys for AP_Param persistent storage system
     * 
     * @details Each parameter is assigned a unique key number that determines its storage
     *          location in EEPROM/flash. Keys are organized into logical groups by number
     *          range. Many keys are marked "unused" or "deprecated" but must remain in the
     *          enum to maintain key compatibility across firmware versions.
     */
    enum {
        // ========================================
        // Core System Settings (0-9)
        // ========================================
        
        /**
         * @brief Layout version number, always key zero
         * 
         * This key stores k_format_version and is checked on boot to detect parameter
         * format changes that require migration.
         */
        k_param_format_version = 0,
        k_param_software_type,      ///< Unused - reserved for future use
        k_param_can_mgr,            ///< CAN bus manager library parameters

        // ========================================
        // Miscellaneous Settings (10-29)
        // ========================================
        
        k_param_log_bitmask_old = 10,   ///< Unused - replaced by k_param_log_bitmask at key 40
        k_param_num_resets_old,         ///< Unused - deprecated
        k_param_reset_switch_chan,      ///< RC channel for parameter reset function
        k_param_initial_mode,           ///< Mode to enter on boot (Manual, Hold, Auto, etc.)
        k_param_scheduler,              ///< Scheduler library parameters
        k_param_relay,                  ///< Relay control library parameters
        k_param_BoardConfig,            ///< Board configuration library parameters
        k_param_pivot_turn_angle_old,   ///< Unused - pivot turn configuration moved to G2
        k_param_rc_13_old,              ///< Unused - RC channel 13 moved to RC_Channels library
        k_param_rc_14_old,              ///< Unused - RC channel 14 moved to RC_Channels library

        // ========================================
        // IO Pins (20-23)
        // ========================================
        
        k_param_rssi_pin = 20,          ///< Unused - replaced by RSSI library parameters
        k_param_battery_volt_pin,       ///< Battery voltage sense pin (deprecated)
        k_param_battery_curr_pin,       ///< Battery current sense pin (deprecated)

        k_param_precland = 24,          ///< Precision landing library parameters

        // ========================================
        // Braking Parameters (30-31) - Deprecated
        // ========================================
        
        k_param_braking_percent_old = 30,   ///< Unused - braking moved to attitude controller
        k_param_braking_speederr_old,       ///< Unused - braking speed error threshold

        // ========================================
        // Logging and Serial (40-70)
        // ========================================
        
        k_param_log_bitmask = 40,       ///< Bitmask controlling which log messages are written
        k_param_gps,                    ///< GPS library parameters
        k_param_serial0_baud,           ///< Deprecated - serial baud rates moved to SerialManager
        k_param_serial1_baud,           ///< Deprecated - serial baud rates moved to SerialManager
        k_param_serial2_baud,           ///< Deprecated - serial baud rates moved to SerialManager

        // ========================================
        // RSSI and RPM Sensors (97-98)
        // ========================================
        
        k_param_rssi = 97,              ///< RSSI (Received Signal Strength Indicator) library
        k_param_rpm_sensor,             ///< RPM sensor library (key 98)
        
        // ========================================
        // Arming Parameters (100-109)
        // ========================================
        
        k_param_arming = 100,           ///< Arming check library parameters

        // ========================================
        // Telemetry and GCS Control (110-128)
        // ========================================
        
        k_param_gcs0_unused = 110,      ///< Unused in ArduPilot 4.7+ (GCS telemetry moved)
        k_param_gcs1_unused,            ///< Unused in ArduPilot 4.7+ (GCS telemetry moved)
        k_param_sysid_this_mav_old,     ///< Deprecated - MAVLink system ID moved to GCS library
        k_param_sysid_my_gcs_old,       ///< Deprecated - MAVLink GCS ID moved to GCS library
        k_param_serial0_baud_old,       ///< Unused - serial configuration moved to SerialManager
        k_param_serial1_baud_old,       ///< Unused - serial configuration moved to SerialManager
        k_param_telem_delay_old,        ///< Deprecated - telemetry delay setting
        k_param_skip_gyro_cal,          ///< Unused - gyro calibration skip flag
        k_param_gcs2_unused,            ///< Unused in ArduPilot 4.7+ (GCS telemetry moved)
        k_param_serial2_baud_old,       ///< Unused - serial configuration moved to SerialManager
        k_param_serial2_protocol,       ///< Deprecated - serial protocol moved to SerialManager
        k_param_serial_manager_old,     ///< Serial manager library parameters (deprecated location)
        k_param_cli_enabled_old,        ///< Unused - CLI interface removed
        k_param_gcs3_unused,            ///< Unused in ArduPilot 4.7+ (GCS telemetry moved)
        k_param_gcs_pid_mask,           ///< Bitmask for streaming PID tuning data to GCS
        k_param_gcs4_unused,            ///< Unused in ArduPilot 4.7+ (GCS telemetry moved)
        k_param_gcs5_unused,            ///< Unused in ArduPilot 4.7+ (GCS telemetry moved)
        k_param_gcs6_unused,            ///< Unused in ArduPilot 4.7+ (GCS telemetry moved)

        // ========================================
        // Sensor and Navigation Libraries (130-139)
        // ========================================
        
        k_param_compass_enabled_deprecated = 130,   ///< Deprecated - compass enable moved to compass library
        k_param_steering_learn,         ///< Unused - steering learning feature removed
        k_param_NavEKF,                 ///< Deprecated - EKF1 library (use NavEKF3)
        k_param_mission,                ///< Mission library parameters
        k_param_NavEKF2_old,            ///< Deprecated - old NavEKF2 location
        k_param_NavEKF2,                ///< NavEKF2 library parameters (legacy, use EKF3)
        k_param_g2,                     ///< Second generation parameter block (ParametersG2)
        k_param_NavEKF3,                ///< NavEKF3 library parameters (current EKF implementation)

        // ========================================
        // Battery Controls (140-145)
        // ========================================
        
        k_param_battery_monitoring = 140,   ///< Deprecated - battery monitoring moved to battery library
        k_param_volt_div_ratio,             ///< Deprecated - voltage divider ratio moved to battery library
        k_param_curr_amp_per_volt,          ///< Deprecated - current sensor scaling moved to battery library
        k_param_input_voltage,              ///< Deprecated - input voltage moved to battery library
        k_param_pack_capacity,              ///< Deprecated - battery capacity moved to battery library
        k_param_battery,                    ///< Battery monitor library parameters

        // ========================================
        // Navigation Parameters (150-159)
        // ========================================
        
        k_param_crosstrack_gain = 150,      ///< Unused - crosstrack correction moved to L1 controller
        k_param_crosstrack_entry_angle,     ///< Unused - entry angle moved to L1 controller
        k_param_speed_cruise,               ///< Default cruise speed in m/s for Auto, Guided modes
        k_param_speed_turn_gain,            ///< Unused - turn speed scaling removed
        k_param_speed_turn_dist,            ///< Unused - turn distance lookahead removed
        k_param_ch7_option,                 ///< Unused - auxiliary channel options moved to RC library
        k_param_auto_trigger_pin,           ///< GPIO pin for triggering Auto mode start
        k_param_auto_kickstart,             ///< Throttle threshold (0-1) to trigger Auto mode start
        k_param_turn_circle,                ///< Unused - turn radius moved to G2 parameters
        k_param_turn_max_g_old,             ///< Unused - maximum lateral acceleration limit removed

        // ========================================
        // Radio Settings (160-167)
        // ========================================
        
        k_param_rc_1_old = 160,             ///< Unused - RC channel 1 moved to RC_Channels library
        k_param_rc_2_old,                   ///< Unused - RC channel 2 moved to RC_Channels library
        k_param_rc_3_old,                   ///< Unused - RC channel 3 moved to RC_Channels library
        k_param_rc_4_old,                   ///< Unused - RC channel 4 moved to RC_Channels library
        k_param_rc_5_old,                   ///< Unused - RC channel 5 moved to RC_Channels library
        k_param_rc_6_old,                   ///< Unused - RC channel 6 moved to RC_Channels library
        k_param_rc_7_old,                   ///< Unused - RC channel 7 moved to RC_Channels library
        k_param_rc_8_old,                   ///< Unused - RC channel 8 moved to RC_Channels library

        // ========================================
        // Throttle Control (170-176)
        // ========================================
        
        k_param_throttle_min_old = 170,     ///< Unused - throttle min moved to motor library
        k_param_throttle_max_old,           ///< Unused - throttle max moved to motor library
        k_param_throttle_cruise,            ///< Base throttle percentage (0-100) for cruising
        k_param_throttle_slewrate_old,      ///< Unused - throttle slew rate moved to motor library
        k_param_throttle_reduction,         ///< Unused - throttle reduction feature removed
        k_param_pilot_steer_type,           ///< Steering input style: default=0, two-paddle=1, direction-only=2
        k_param_skid_steer_out_old,         ///< Unused - skid steering moved to motor library

        // ========================================
        // Failsafe Control (180-187)
        // ========================================
        
        k_param_fs_action = 180,            ///< Action to take on RC failsafe: 0=Nothing, 1=RTL, 2=Hold, 3=SmartRTL, 4=SmartRTL_Hold, 5=Terminate
        k_param_fs_timeout,                 ///< Time in seconds with no RC before triggering failsafe (typically 1.5s)
        k_param_fs_throttle_enabled,        ///< Enable RC throttle failsafe: 0=Disabled, 1=Enabled, 2=Enabled (continue mission in Auto)
        k_param_fs_throttle_value,          ///< PWM value below which throttle failsafe triggers (typically 910)
        k_param_fs_gcs_enabled,             ///< Enable GCS (ground control station) failsafe: 0=Disabled, 1=Enabled, 2=Enabled (continue mission in Auto)
        k_param_fs_crash_check,             ///< Enable crash detection: 0=Disabled, 1=Hold, 2=Hold and disarm
        k_param_fs_ekf_action,              ///< Action on EKF failsafe: 0=Disabled, 1=Report only, 2=Hold
        k_param_fs_ekf_thresh,              ///< EKF variance threshold for triggering EKF failsafe

        // ========================================
        // Obstacle Control / Rangefinder (190-197)
        // ========================================
        
        k_param_sonar_enabled = 190,        ///< Deprecated - sonar enable moved to rangefinder library
        k_param_sonar_old,                  ///< Unused - sonar configuration removed
        k_param_rangefinder_trigger_cm,     ///< Unused - rangefinder trigger distance removed
        k_param_rangefinder_turn_angle,     ///< Unused - rangefinder-triggered turn angle removed
        k_param_rangefinder_turn_time,      ///< Unused - rangefinder-triggered turn time removed
        k_param_sonar2_old,                 ///< Unused - second sonar removed
        k_param_rangefinder_debounce,       ///< Unused - rangefinder debounce removed
        k_param_rangefinder,                ///< Rangefinder library parameters

        // ========================================
        // Driving Modes (210-217)
        // ========================================
        
        k_param_mode_channel = 210,         ///< RC channel for mode selection (typically channel 5)
        k_param_mode1,                      ///< Flight mode assigned to mode switch position 1
        k_param_mode2,                      ///< Flight mode assigned to mode switch position 2
        k_param_mode3,                      ///< Flight mode assigned to mode switch position 3
        k_param_mode4,                      ///< Flight mode assigned to mode switch position 4
        k_param_mode5,                      ///< Flight mode assigned to mode switch position 5
        k_param_mode6,                      ///< Flight mode assigned to mode switch position 6
        k_param_aux_channel_old,            ///< Unused - auxiliary mode channel removed

        // ========================================
        // Waypoint Data (220-223) - Mostly Unused
        // ========================================
        
        k_param_command_total = 220,        ///< Unused - mission command count stored in mission library
        k_param_command_index,              ///< Unused - current mission index stored in mission library
        k_param_waypoint_radius_old,        ///< Unused - waypoint radius moved to waypoint navigation library
        k_param_waypoint_overshoot_old,     ///< Unused - waypoint overshoot moved to waypoint navigation library

        // ========================================
        // Camera Control (224-226)
        // ========================================
        
        k_param_camera,                     ///< Camera library parameters
        k_param_camera_mount,               ///< Camera mount/gimbal library parameters
        k_param_camera_mount2,              ///< Unused - second camera mount removed

        // ========================================
        // PID Controllers (230-234)
        // ========================================
        
        k_param_pidNavSteer = 230,          ///< Navigation steering PID controller (lateral control)
        k_param_pidServoSteer,              ///< Unused - servo steering PID removed
        k_param_pidSpeedThrottle_old,       ///< Unused - speed throttle PID moved to attitude controller

        // ========================================
        // High RC Channels (235-238)
        // ========================================
        
        k_param_rc_9_old = 235,             ///< Unused - RC channel 9 moved to RC_Channels library
        k_param_rc_10_old,                  ///< Unused - RC channel 10 moved to RC_Channels library
        k_param_rc_11_old,                  ///< Unused - RC channel 11 moved to RC_Channels library
        k_param_rc_12_old,                  ///< Unused - RC channel 12 moved to RC_Channels library

        // ========================================
        // Library Objects (240-252)
        // ========================================
        
        k_param_sitl = 240,                 ///< SITL (Software In The Loop) simulation parameters
        k_param_ahrs,                       ///< AHRS (Attitude Heading Reference System) library
        k_param_ins,                        ///< INS (Inertial Navigation System) library
        k_param_compass,                    ///< Compass/magnetometer library
        k_param_rcmap,                      ///< RC channel mapping library
        k_param_L1_controller,              ///< Unused - L1 navigation controller (for aircraft, not rovers)
        k_param_steerController_old,        ///< Unused - old steering controller removed
        k_param_barometer,                  ///< Barometer library
        k_param_notify,                     ///< Notification library (LEDs, buzzers, etc.)
        k_param_button,                     ///< Button input library
        k_param_osd,                        ///< On-Screen Display library
        k_param_optflow,                    ///< Optical flow sensor library

        k_param_logger = 253,               ///< Logger library parameters (dataflash logging)

        // ========================================
        // Reserved and Common Blocks (254-258)
        // ========================================
        
        // 254,255: reserved for future use

        k_param_vehicle = 257,              ///< Vehicle common parameters shared across all vehicle types
        k_param__gcs = 258,                 ///< GCS (Ground Control Station) library parameters
        };

    // ========================================
    // Parameter Member Variables
    // ========================================
    // These AP_Param-wrapped variables provide persistent storage for rover configuration.
    // Values are stored in EEPROM/flash and survive reboots. Modified via MAVLink from GCS.
    
    AP_Int16    format_version;         ///< Parameter layout version (currently 16)

    // ----------------------------------------
    // Miscellaneous Settings
    // ----------------------------------------
    
    AP_Int32    log_bitmask;            ///< Bitmask controlling which log messages are written to dataflash (0=none, 0xFFFF=all)
    AP_Int8     reset_switch_chan;      ///< RC channel used for parameter reset function (0=disabled)
    AP_Int8     initial_mode;           ///< Mode to enter on boot: 0=Manual, 4=Hold, 10=Auto, 15=Guided, etc.

    // ----------------------------------------
    // Navigation Parameters
    // ----------------------------------------
    
    AP_Float    speed_cruise;           ///< Default cruise speed in m/s for Auto and Guided modes (typical: 2-5 m/s)
    AP_Int8     ch7_option;             ///< Unused - auxiliary channel function (moved to RC library)
    AP_Int8     auto_trigger_pin;       ///< GPIO pin number for auto mode trigger (-1=disabled, 0-54=GPIO pin)
    AP_Float    auto_kickstart;         ///< Throttle threshold (0.0-1.0) required to start Auto mode when using trigger
    AP_Int16    gcs_pid_mask;           ///< Bitmask selecting which PID controllers stream tuning data to GCS

    // ----------------------------------------
    // Throttle Control
    // ----------------------------------------
    
    AP_Int8     throttle_cruise;        ///< Base throttle percentage (0-100) for cruising at speed_cruise
    AP_Enum<PilotSteerType> pilot_steer_type; ///< Manual steering input style: 0=Default, 1=Two-paddle, 2=Direction-only

    // ----------------------------------------
    // Failsafe Control
    // ----------------------------------------
    
    AP_Int8     fs_action;              ///< RC failsafe action: 0=Nothing, 1=RTL, 2=Hold, 3=SmartRTL, 4=SmartRTL or Hold, 5=Terminate
    AP_Float    fs_timeout;             ///< Time in seconds without RC before triggering failsafe (typical: 1.5s)
    AP_Int8     fs_throttle_enabled;    ///< Enable RC throttle failsafe: 0=Disabled, 1=Enabled, 2=Enabled (continue if in Auto)
    AP_Int16    fs_throttle_value;      ///< PWM value below which throttle failsafe triggers (typical: 910 μs)
    AP_Int8     fs_gcs_enabled;         ///< Enable GCS failsafe: 0=Disabled, 1=Enabled, 2=Enabled (continue if in Auto)
    AP_Int8     fs_crash_check;         ///< Crash detection: 0=Disabled, 1=Hold on crash, 2=Hold and disarm on crash
    AP_Int8     fs_ekf_action;          ///< EKF failsafe action: 0=Disabled, 1=Report only, 2=Hold
    AP_Float    fs_ekf_thresh;          ///< EKF variance threshold for triggering EKF failsafe (typical: 0.8)

    // ----------------------------------------
    // Driving Mode Configuration
    // ----------------------------------------
    
    AP_Int8     mode_channel;           ///< RC channel for flight mode selection (typically channel 5, range: 1-16)
    AP_Int8     mode1;                  ///< Flight mode for switch position 1 (typically Manual=0)
    AP_Int8     mode2;                  ///< Flight mode for switch position 2
    AP_Int8     mode3;                  ///< Flight mode for switch position 3
    AP_Int8     mode4;                  ///< Flight mode for switch position 4
    AP_Int8     mode5;                  ///< Flight mode for switch position 5
    AP_Int8     mode6;                  ///< Flight mode for switch position 6 (typically RTL=11)

    /**
     * @brief Default constructor
     * 
     * Parameter values are loaded from persistent storage by AP_Param system during init.
     */
    Parameters() {}
};

/**
 * @class ParametersG2
 * @brief Second generation parameter block for rover parameters added after key space exhaustion
 * 
 * @details As the original Parameters class filled up the available key space (0-255), newer
 *          parameters were added to this ParametersG2 class which is stored at key 109 in the
 *          main parameter table. This allows effectively unlimited parameter expansion while
 *          maintaining backwards compatibility.
 *          
 *          ParametersG2 contains both simple parameters (AP_Param types) and complex subsystem
 *          objects that have their own parameter tables (e.g., motors, attitude_control, wp_nav).
 *          
 *          Key additions in G2:
 *          - Modern motor control (AP_MotorsUGV)
 *          - Attitude control system (AR_AttitudeControl)
 *          - Waypoint navigation with object avoidance (AR_WPNav_OA)
 *          - Advanced features: wheel encoders, sailboat, follow mode, smart RTL
 *          - Frame configuration (frame_class, frame_type for custom vehicles)
 *          - Modern peripheral libraries (proximity, avoidance, sprayer)
 *          
 * @note All parameters in G2 have been added in recent ArduPilot versions and may not be
 *       present in very old firmware. When adding new rover parameters, add them to G2
 *       rather than trying to squeeze them into the original Parameters class.
 *       
 * @warning Some members are conditionally compiled based on HAL features (e.g., AP_BEACON_ENABLED,
 *          HAL_PROXIMITY_ENABLED). The var_info[] table in Parameters.cpp must match the
 *          conditional compilation to avoid parameter numbering mismatches.
 * 
 * @see Parameters for legacy parameter block
 * @see Rover/Parameters.cpp for G2 var_info[] table with parameter metadata
 */
class ParametersG2 {
public:
    /**
     * @brief ParametersG2 constructor
     * 
     * Initializes the second-generation parameter block. Member objects are constructed
     * and their parameters registered with AP_Param for persistent storage.
     */
    ParametersG2(void);

    /**
     * @brief AP_Param group info table for G2 parameters
     * 
     * Defines parameter metadata, storage keys, and default values for all G2 parameters.
     * Must be kept in sync with member variable declarations and conditional compilation.
     */
    static const struct AP_Param::GroupInfo var_info[];

    // ========================================
    // Core Control Systems
    // ========================================
    
    RC_Channels_Rover rc_channels;      ///< RC input channel library for rover (handles RC failsafe, mode switching)
    SRV_Channels servo_channels;        ///< Servo/motor output channel management (PWM output ranges, reversing, trim)

#if AP_ROVER_ADVANCED_FAILSAFE_ENABLED
    AP_AdvancedFailsafe_Rover afs;      ///< Advanced failsafe with termination capability (requires AFS hardware)
#endif

#if AP_BEACON_ENABLED
    AP_Beacon beacon;                   ///< Beacon positioning system for non-GPS navigation
#endif

    // ========================================
    // Wheel Sensors and Odometry
    // ========================================
    
    AP_WheelEncoder wheel_encoder;      ///< Wheel encoder library for odometry (measures wheel rotation)
    AP_WheelRateControl wheel_rate_control; ///< Wheel rate controller for precise speed control using encoders

    // ========================================
    // Motor and Attitude Control
    // ========================================
    
    AP_MotorsUGV motors;                ///< Unmanned Ground Vehicle motor library (handles steering/throttle mixing)
    AR_AttitudeControl attitude_control; ///< Attitude and speed controller for rovers (steering and throttle PID)

    // ========================================
    // Steering Configuration
    // ========================================
    
    AP_Float turn_radius;               ///< Vehicle turn radius in meters (for steering mode, typical: 0.5-2.0m)
    AP_Float acro_turn_rate;            ///< Maximum turn rate in deg/s for Acro mode (typical: 90-360 deg/s)

    // ========================================
    // Return to Launch Configuration
    // ========================================
    
    AP_SmartRTL smart_rtl;              ///< Smart RTL path recording and return (records path, returns along safe route)
    AP_Float rtl_speed;                 ///< Default speed in m/s for RTL mode (0=use cruise speed)

    // ========================================
    // Vehicle Frame Configuration
    // ========================================
    
    AP_Int8 frame_class;                ///< Frame class: 0=Undefined, 1=Rover, 2=Boat, 3=BalanceBot, 4=Omni

#if HAL_PROXIMITY_ENABLED
    AP_Proximity proximity;             ///< Proximity sensor library for obstacle detection (360° sensing)
#endif

#if MODE_DOCK_ENABLED
    class ModeDock *mode_dock_ptr;      ///< Pointer to Dock mode for parameter table registration
#endif

#if AP_AVOIDANCE_ENABLED
    AC_Avoid avoid;                     ///< Object avoidance library (stop/slide behavior near obstacles)
#endif

    // ========================================
    // Balance Bot Configuration
    // ========================================
    
    AP_Float bal_pitch_max;             ///< Maximum pitch angle in degrees at 100% throttle for balance bots (typical: 10-25°)
    AP_Float bal_pitch_trim;            ///< Balance bot pitch trim adjustment in degrees for equilibrium

    // ========================================
    // Safety and Crash Detection
    // ========================================
    
    AP_Int8 crash_angle;                ///< Pitch/roll angle in degrees for crash detection (typical: 30°, 0=disabled)

#if AP_FOLLOW_ENABLED
    AP_Follow follow;                   ///< Follow mode library for following another vehicle via MAVLink
#endif

    // ========================================
    // Frame Type and Motor Configuration
    // ========================================
    
    AP_Int8 frame_type;                 ///< Frame type for custom motor configs: 0=Undefined, 1=Omni3, 2=OmniX, 3=OmniPlus

    // ========================================
    // Loiter Mode Configuration
    // ========================================
    
    AP_Int8 loit_type;                  ///< Loiter type: 0=Circle around point, 1=Face direction
    AP_Float loit_radius;               ///< Loiter circle radius in meters (typical: 5-20m, negative for counter-clockwise)

#if HAL_SPRAYER_ENABLED
    AC_Sprayer sprayer;                 ///< Agricultural sprayer control library (pump and spinner control)
#endif

#if HAL_RALLY_ENABLED
    AP_Rally_Rover rally;               ///< Rally point library for alternate RTL destinations
#endif

    // ========================================
    // Simplified Control Modes
    // ========================================
    
    AP_Int8 simple_type;                ///< Simple mode heading reference: 0=Disabled, 1=InitHeading, 2=CardinalDirections

    // ========================================
    // Wind Sensing (Sailboat)
    // ========================================
    
    AP_WindVane windvane;               ///< Wind vane library for wind direction/speed sensing (primarily for sailboats)

#if AP_MISSION_ENABLED
    AP_Enum<ModeAuto::DoneBehaviour> mis_done_behave; ///< Mission completion behavior: 0=Hold, 1=Loiter, 2=Acro, 3=Manual
#endif

    // ========================================
    // Manual Control Configuration
    // ========================================
    
    AP_Int8 stick_mixing;               ///< Stick mixing in auto modes: 0=Disabled, 1=Enabled (pilot override)
    AP_Float manual_steering_expo;      ///< Manual mode steering expo (0=linear, 0.5=moderate expo, 0.8=strong expo)

    // ========================================
    // Waypoint Navigation and Object Avoidance
    // ========================================
    
    AR_WPNav_OA wp_nav;                 ///< Waypoint navigation library with object avoidance integration
    AR_PosControl pos_control;          ///< Position controller for precision waypoint tracking

#if AP_OAPATHPLANNER_ENABLED
    AP_OAPathPlanner oa;                ///< Object avoidance path planner (Dijkstra/BendyRuler algorithms)
#endif

    // ========================================
    // Sailboat-Specific Control
    // ========================================
    
    Sailboat sailboat;                  ///< Sailboat control library (tacking, sail control, wind navigation)

    // ========================================
    // Speed Configuration
    // ========================================
    
    AP_Float speed_max;                 ///< Maximum speed in m/s (0=no limit, limits commanded speed in all modes)
    AP_Float loiter_speed_gain;         ///< Speed correction gain for loiter mode (typical: 0.5-2.0)

    // ========================================
    // Extended Failsafe Options
    // ========================================
    
    AP_Int32 fs_options;                ///< Failsafe options bitmask (bit 0: RC continue if in Auto, etc.)
    AP_Float fs_gcs_timeout;            ///< GCS failsafe timeout in seconds (0=use default FS_TIMEOUT)

#if HAL_TORQEEDO_ENABLED
    AP_Torqeedo torqeedo;               ///< Torqeedo electric motor driver library (battery monitor, motor control)
#endif

    // ========================================
    // Mode-Specific Options
    // ========================================
    
    AP_Int32 guided_options;            ///< Guided mode options bitmask (bit 0: allow arming in guided)
    AP_Int32 manual_options;            ///< Manual mode options bitmask (bit 0: enable throttle arming check)

    // ========================================
    // Mode Objects
    // ========================================
    
    class ModeCircle mode_circle;       ///< Circle mode implementation and parameters
};

extern const AP_Param::Info var_info[];
