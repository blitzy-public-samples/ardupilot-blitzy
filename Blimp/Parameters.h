/**
 * @file Parameters.h
 * @brief Parameter structure definitions for Blimp vehicle
 * 
 * @details This file defines the Parameters and ParametersG2 classes that contain
 *          all configurable parameters for the ArduPilot Blimp (lighter-than-air vehicle).
 *          
 *          The Parameters class contains the primary parameter set with unique identifiers
 *          in the k_param_* enumeration. These IDs are used by AP_Param to store and locate
 *          parameters in EEPROM/flash storage.
 *          
 *          The ParametersG2 class contains a secondary block of parameters to avoid
 *          exceeding the 256 top-level key limit of the AP_Param system.
 *          
 *          Parameter categories include:
 *          - Flight control (PID gains, velocity/position limits)
 *          - Navigation (RTL altitude, waypoint settings)
 *          - Failsafe configuration (EKF, GCS, throttle, vibration)
 *          - Hardware configuration (motors, servos, sensors)
 *          - Telemetry and logging
 *          - Flight modes
 *          
 * @note This file defines parameter structure only. Default values and metadata
 *       are defined in Parameters.cpp using AP_Param::GroupInfo tables.
 * 
 * @warning Modifying the k_param_* enum values will break parameter compatibility
 *          with existing EEPROM data. Always increment k_format_version when making
 *          incompatible changes.
 */

#pragma once

#define AP_PARAM_VEHICLE_NAME blimp

#include <AP_Common/AP_Common.h>
#include "RC_Channel_Blimp.h"
#include <SRV_Channel/SRV_Channel.h>

/**
 * @class Parameters
 * @brief Primary parameter storage class for Blimp vehicle
 * 
 * @details This class contains all primary configurable parameters for the Blimp vehicle.
 *          Each parameter is stored as an AP_Param type (AP_Int8, AP_Int16, AP_Int32, AP_Float)
 *          which provides automatic EEPROM/flash persistence and ground station access.
 *          
 *          The class uses an enumeration (k_param_*) to assign unique identifiers to each
 *          parameter or parameter group. These identifiers must remain stable across firmware
 *          versions to maintain parameter storage compatibility.
 *          
 *          Parameter Organization:
 *          - k_format_version (0): Layout version for compatibility checking
 *          - Library groups (1-9): INS, EKF, CAN, OSD, etc.
 *          - Core settings (10-89): Sensors, failsafes, misc
 *          - PID controllers (32-49): Velocity and position control PIDs
 *          - Limits (50-89): Maximum velocity/position values
 *          - Motors/hardware (90-109): Motor configuration, disarm delay
 *          - Telemetry (110-134): GCS communication settings
 *          - Navigation (135-164): RTL, waypoint settings
 *          - Camera (165-169): Camera and gimbal mount
 *          - Radio (170-199): RC input and failsafe
 *          - Flight modes (200-219): Mode selection and initial mode
 *          - Safety (220-256): Arming checks, EKF failsafe action
 *          - Logging (253): DataFlash logging configuration
 *          
 * @note The k_param_* enumeration space is 9-bits (0-511), but values above 256
 *       require using ParametersG2 due to AP_Param top-level key limitations.
 */
class Parameters
{
public:
    /**
     * @brief Parameter layout format version
     * 
     * @details This version number identifies the parameter layout structure.
     *          When the parameter enumeration is changed in an incompatible way
     *          (reordering, removing, or changing parameter IDs), this version
     *          must be incremented to prevent old EEPROM data from being loaded
     *          incorrectly by newer firmware.
     *          
     * @note Current version: 1
     * @warning Always increment when making incompatible parameter changes
     */
    static const uint16_t        k_format_version = 1;

    /**
     * @enum Parameter identifier enumeration
     * @brief Unique identifiers for all parameters and parameter groups
     * 
     * @details This enumeration assigns a unique ID to each parameter or parameter group.
     *          AP_Param uses these IDs to store and retrieve parameters from EEPROM/flash.
     *          
     *          Key principles:
     *          - Layout version is always key zero (k_param_format_version)
     *          - Entries without explicit values get next sequential number
     *          - Related parameters are grouped in numerical ranges
     *          - Parameter IDs must never be reused or changed (breaks EEPROM compatibility)
     *          - 9-bit ID space allows values 0-511
     *          - Values above 256 typically use ParametersG2 for top-level storage
     *          
     *          ID Ranges:
     *          - 0-9: Format version and library groups (INS, EKF, SITL)
     *          - 10-31: Core sensors and subsystems
     *          - 32-49: PID controllers for attitude and position
     *          - 50-89: Control limits and motor configuration
     *          - 90-109: Hardware and safety settings
     *          - 110-143: Telemetry and GCS communication
     *          - 135-163: Navigation parameters (RTL, waypoints)
     *          - 165-169: Camera and mount
     *          - 170-199: Radio input and failsafe
     *          - 200-219: Flight mode configuration
     *          - 220-256: Safety and arming
     *          - 253: Logging configuration
     *          - 257-258: Vehicle common parameters and GCS
     *          
     * @warning Modifying these values breaks parameter storage compatibility.
     *          Never change, remove, or reuse parameter IDs.
     */
    enum {
        /** @brief Layout version number - always key zero for compatibility checking */
        k_param_format_version = 0,
        
        /** @brief AP_InertialSensor library parameter group - IMU configuration */
        k_param_ins,
        
        /** @brief NavEKF2 library parameter group - Extended Kalman Filter version 2 */
        k_param_NavEKF2,
        
        /** @brief Secondary parameter block (ParametersG2) - extends beyond 256 key limit */
        k_param_g2,
        
        /** @brief NavEKF3 library parameter group - Extended Kalman Filter version 3 (current) */
        k_param_NavEKF3,
        
        /** @brief AP_CANManager library parameter group - CAN bus configuration */
        k_param_can_mgr,
        
        /** @brief AP_OSD library parameter group - On-screen display configuration */
        k_param_osd,

        /** @brief SITL simulation parameter group - software-in-the-loop testing configuration */
        k_param_sitl = 10,

        /** @brief AP_Baro library parameter group - barometer sensor configuration */
        k_param_barometer,

        /** @brief AP_Scheduler library parameter group - task scheduling debug parameters */
        k_param_scheduler,

        /** @brief AP_BoardConfig library parameter group - board-specific hardware configuration */
        k_param_BoardConfig,

        /** @brief AP_GPS library parameter group - GPS receiver configuration */
        k_param_gps,

        /** @brief AP_Parachute library parameter group - emergency parachute deployment */
        k_param_parachute,

        /** @brief AP_LandingGear library parameter group - landing gear retraction control */
        k_param_landinggear,

        /** @brief Input manager parameter group - pilot input processing configuration */
        k_param_input_manager,

        /** @brief GPS HDOP threshold for good position fix quality (unitless, lower is better) */
        k_param_gps_hdop_good,
        
        /** @brief AP_BattMonitor library parameter group - battery monitoring configuration */
        k_param_battery,
        
        /** @brief Position hold brake rate - deceleration rate when stopping (cm/s/s) */
        k_param_poshold_brake_rate,
        
        /** @brief Position hold maximum brake angle - attitude limit during braking (centidegrees) */
        k_param_poshold_brake_angle_max,
        
        /** @brief Pilot vertical acceleration control sensitivity (cm/s/s) */
        k_param_pilot_accel_z,
        
        /** @brief EKF failsafe innovation threshold - triggers failsafe when exceeded */
        k_param_fs_ekf_thresh,
        
        /** @brief AP_Terrain library parameter group - terrain following configuration */
        k_param_terrain,
        
        /** @brief Throttle input deadzone - prevents unintended drift (0-1000) */
        k_param_throttle_deadzone,
        
        /** @brief DataFlash logging bitmask - selects which message types to log */
        k_param_log_bitmask,
        
        /** @brief Throttle input filter cutoff frequency (Hz) */
        k_param_throttle_filt,
        
        /** @brief Throttle stick behavior configuration (spring-loaded vs retained position) */
        k_param_throttle_behavior,
        
        /** @brief Pilot takeoff altitude target (unused parameter, reserved for future use) */
        k_param_pilot_takeoff_alt,

        /** @brief AP_ADSB library parameter group - ADS-B receiver configuration for traffic avoidance */
        k_param_adsb,
        
        /** @brief AP_Notify library parameter group - LED and buzzer notification configuration */
        k_param_notify,

        /** @brief PID controller for horizontal velocity (XY plane) - stabilizes velocity in NED frame */
        k_param_pid_vel_xy = 32,
        
        /** @brief PID controller for vertical velocity (Z axis down) - climb/descent rate control */
        k_param_pid_vel_z,
        
        /** @brief PID controller for yaw rate - rotation rate around Z axis */
        k_param_pid_vel_yaw,
        
        /** @brief PID controller for horizontal position (XY plane) - waypoint position holding */
        k_param_pid_pos_xy,
        
        /** @brief PID controller for vertical position (altitude) - altitude hold */
        k_param_pid_pos_z,
        
        /** @brief PID controller for yaw position (heading) - heading hold */
        k_param_pid_pos_yaw,

        /** @brief Maximum horizontal velocity limit (m/s) - safety limit for XY movement */
        k_param_max_vel_xy = 50,
        
        /** @brief Maximum vertical velocity limit (m/s) - safety limit for climb/descent */
        k_param_max_vel_z,
        
        /** @brief Maximum yaw rate limit (deg/s) - safety limit for rotation */
        k_param_max_vel_yaw,
        
        /** @brief Maximum horizontal position error (m) - position controller limit */
        k_param_max_pos_xy,
        
        /** @brief Maximum vertical position error (m) - altitude controller limit */
        k_param_max_pos_z,
        
        /** @brief Maximum yaw position error (degrees) - heading controller limit */
        k_param_max_pos_yaw,
        
        /** @brief Simple mode enable flag - simplifies control for novice pilots */
        k_param_simple_mode,
        
        /** @brief Disturbance mask - enables/disables wind compensation on specific axes */
        k_param_dis_mask,
        
        /** @brief PID deadzone - minimum error before controller activates (prevents oscillation) */
        k_param_pid_dz,

        /**
         * @name Motor and Hardware Configuration (90-96)
         * @{
         */
        /** @brief AP_Motors library parameter group - motor mixing and output configuration */
        k_param_motors = 90,
        
        /** @brief Disarm delay - time after landing before automatic disarm (seconds) */
        k_param_disarm_delay,
        
        /** @brief Crash check failsafe enable - detects vehicle crash and disarms */
        k_param_fs_crash_check,
        
        /** @brief Throw mode motor start enable - arms motors when vehicle is thrown */
        k_param_throw_motor_start,
        
        /** @brief RTL altitude type - determines reference for RTL altitude (terrain vs home) */
        k_param_rtl_alt_type,
        
        /** @brief AP_Avoidance library parameter group - object avoidance configuration */
        k_param_avoid,
        
        /** @brief ADS-B avoidance enable - uses traffic data for collision avoidance */
        k_param_avoidance_adsb,
        /** @} */

        /** @brief AP_RSSI library parameter group - received signal strength indication (ID 97) */
        k_param_rssi = 97,

        /**
         * @name Telemetry Control (110-142)
         * @brief Ground control station communication parameters
         * @note Many entries marked unused as they were migrated to serial manager in ArduPilot-4.7
         * @{
         */
        /** @brief GCS port 0 - unused in ArduPilot-4.7, kept for EEPROM compatibility */
        k_param_gcs0_unused = 110,
        
        /** @brief GCS port 1 - unused in ArduPilot-4.7, kept for EEPROM compatibility */
        k_param_gcs1_unused,
        
        /** @brief Legacy system ID for this MAV - migrated to AP_Vehicle */
        k_param_sysid_this_mav_old,
        
        /** @brief Legacy ground control station system ID - migrated to AP_Vehicle */
        k_param_sysid_my_gcs_old,
        
        /** @brief Legacy telemetry delay - used for parameter conversion in ArduPilot-4.7 */
        k_param_telem_delay_old,
        
        /** @brief GCS port 2 - unused in ArduPilot-4.7, kept for EEPROM compatibility */
        k_param_gcs2_unused,
        
        /** @brief Legacy serial manager - migrated to AP_SerialManager library */
        k_param_serial_manager_old,
        
        /** @brief GCS port 3 - unused in ArduPilot-4.7, kept for EEPROM compatibility */
        k_param_gcs3_unused,
        
        /** @brief GCS PID mask - selects which PID values to stream to ground station */
        k_param_gcs_pid_mask,
        
        /** @brief GCS port 4 - unused in ArduPilot-4.7, kept for EEPROM compatibility */
        k_param_gcs4_unused,
        
        /** @brief GCS port 5 - unused in ArduPilot-4.7, kept for EEPROM compatibility */
        k_param_gcs5_unused,
        
        /** @brief GCS port 6 - unused in ArduPilot-4.7, kept for EEPROM compatibility */
        k_param_gcs6_unused,
        /** @} */

        /**
         * @name RTL Parameters (135-137)
         * @brief Return-to-launch configuration (ID range reserved for Solo compatibility)
         * @{
         */
        /** @brief RTL horizontal speed (cm/s) - speed during return to launch */
        k_param_rtl_speed_cms = 135,
        
        /** @brief Battery failsafe RTL current threshold - triggers RTL when battery current drops */
        k_param_fs_batt_curr_rtl,
        
        /** @brief RTL cone slope - angle of descent cone for obstacle avoidance during RTL (ID 137) */
        k_param_rtl_cone_slope,
        /** @} */

        /**
         * @name Sensor Parameters (140-159)
         * @brief Sensor library configuration
         * @{
         */
        /** @brief AP_Compass library parameter group - magnetometer/compass configuration */
        k_param_compass,
        
        /** @brief Frame type configuration - unused in current Blimp implementation */
        k_param_frame_type,
        
        /** @brief AP_AHRS library parameter group - attitude and heading reference system (ID 159) */
        k_param_ahrs,
        /** @} */

        /**
         * @name Navigation Parameters (160-163)
         * @brief Autonomous navigation and RTL configuration
         * @{
         */
        /** @brief RTL altitude target (cm) - altitude to climb/descend to during return to launch */
        k_param_rtl_altitude = 160,
        
        /** @brief RTL loiter time (ms) - time to loiter at home before landing */
        k_param_rtl_loiter_time,
        
        /** @brief RTL final altitude (cm) - altitude for final descent phase before landing */
        k_param_rtl_alt_final,
        /** @} */

        /**
         * @name Camera and Mount Parameters (165-166)
         * @brief Camera trigger and gimbal mount configuration
         * @{
         */
        /** @brief AP_Camera library parameter group - camera trigger and control */
        k_param_camera = 165,
        
        /** @brief AP_Mount library parameter group - gimbal mount stabilization and pointing */
        k_param_camera_mount,
        /** @} */

        /**
         * @name Radio Settings (170-199)
         * @brief RC input and failsafe configuration
         * @{
         */
        /** @brief Throttle failsafe action - behavior when throttle signal lost (0=disabled, 1=land, 2=RTL) */
        k_param_failsafe_throttle = 170,
        
        /** @brief Throttle failsafe trigger value (PWM) - threshold below which failsafe triggers */
        k_param_failsafe_throttle_value,
        
        /** @brief Radio tuning channel - unused parameter reserved for future use */
        k_param_radio_tuning,
        
        /** @brief RC refresh rate (Hz) - output update frequency for servo channels (ID 192) */
        k_param_rc_speed = 192,
        
        /** @brief GCS failsafe behavior - action when ground station telemetry lost */
        k_param_failsafe_gcs,
        
        /** @brief RCMap library parameter group - RC channel mapping configuration (ID 199) */
        k_param_rcmap,
        /** @} */

        /**
         * @name Flight Modes (200-207)
         * @brief Flight mode selection and configuration
         * @{
         */
        /** @brief Flight mode 1 - mode assigned to first switch position */
        k_param_flight_mode1 = 200,
        
        /** @brief Flight mode 2 - mode assigned to second switch position */
        k_param_flight_mode2,
        
        /** @brief Flight mode 3 - mode assigned to third switch position */
        k_param_flight_mode3,
        
        /** @brief Flight mode 4 - mode assigned to fourth switch position */
        k_param_flight_mode4,
        
        /** @brief Flight mode 5 - mode assigned to fifth switch position */
        k_param_flight_mode5,
        
        /** @brief Flight mode 6 - mode assigned to sixth switch position */
        k_param_flight_mode6,
        
        /** @brief Flight mode channel - RC channel used for mode selection */
        k_param_flight_mode_chan,
        
        /** @brief Initial flight mode - mode to enter on successful arming */
        k_param_initial_mode,
        /** @} */

        /**
         * @name Safety and Arming (220-221)
         * @brief Safety system configuration
         * @{
         */
        /** @brief EKF failsafe action - behavior when EKF variance exceeds threshold (ID 220) */
        k_param_fs_ekf_action = 220,
        
        /** @brief AP_Arming library parameter group - pre-arm and arming check configuration */
        k_param_arming,
        /** @} */

        /** @brief AP_Logger library parameter group - DataFlash logging configuration (ID 253) */
        k_param_logger = 253,

        /** @brief AP_Vehicle common parameter block - shared vehicle parameters (ID 257) */
        k_param_vehicle = 257,
        
        /** @brief GCS_MAVLINK library parameter group - MAVLink communication (ID 258) */
        k_param__gcs = 258,

        /** @note The k_param_* enumeration space is 9-bits in size, allowing IDs 0-511 */
    };

    /**
     * @brief Parameter layout format version number
     * @details Used by AP_Param to verify parameter storage compatibility
     * @note Must match k_format_version constant
     */
    AP_Int16        format_version;

    /**
     * @name Throttle Control Parameters
     * @{
     */
    /** @brief Throttle input low-pass filter cutoff frequency (Hz) */
    AP_Float        throttle_filt;
    
    /** @brief Throttle stick behavior (0=spring-loaded center, 1=center hover, 2=retain position) */
    AP_Int16        throttle_behavior;
    
    /** @brief Throttle failsafe action (0=disabled, 1=land, 2=RTL, 3=SmartRTL or Land) */
    AP_Int8         failsafe_throttle;
    
    /** @brief Throttle PWM value below which failsafe triggers (typically <975 for lost signal) */
    AP_Int16        failsafe_throttle_value;
    
    /** @brief Throttle input deadzone in PWM units (prevents unintended drift near center) */
    AP_Int16        throttle_deadzone;
    /** @} */

    /**
     * @name Ground Control Station Parameters
     * @{
     */
    /** @brief GCS failsafe behavior (0=disabled, 1=land, 2=RTL, 3=SmartRTL or Land, 5=continue mission) */
    AP_Int8         failsafe_gcs;
    
    /** @brief GCS PID mask - bitmask selecting which PID values to stream to ground station */
    AP_Int16        gcs_pid_mask;
    /** @} */

    /**
     * @name GPS Parameters
     * @{
     */
    /** @brief GPS HDOP threshold for good position fix (unitless, typical value 200 = 2.0 HDOP) */
    AP_Int16        gps_hdop_good;
    /** @} */

    /**
     * @name Flight Mode Configuration
     * @{
     */
    /** @brief Flight mode for switch position 1 (0=Land, 1=Manual, 2=Velocity, 3=Loiter) */
    AP_Int8         flight_mode1;
    
    /** @brief Flight mode for switch position 2 (0=Land, 1=Manual, 2=Velocity, 3=Loiter) */
    AP_Int8         flight_mode2;
    
    /** @brief Flight mode for switch position 3 (0=Land, 1=Manual, 2=Velocity, 3=Loiter) */
    AP_Int8         flight_mode3;
    
    /** @brief Flight mode for switch position 4 (0=Land, 1=Manual, 2=Velocity, 3=Loiter) */
    AP_Int8         flight_mode4;
    
    /** @brief Flight mode for switch position 5 (0=Land, 1=Manual, 2=Velocity, 3=Loiter) */
    AP_Int8         flight_mode5,
    
    /** @brief Flight mode for switch position 6 (0=Land, 1=Manual, 2=Velocity, 3=Loiter) */
    AP_Int8         flight_mode6;
    
    /** @brief RC input channel for flight mode selection (typically channel 5, value 1-18) */
    AP_Int8         flight_mode_chan;
    
    /** @brief Flight mode to enter immediately after successful arming */
    AP_Int8         initial_mode;
    /** @} */

    /**
     * @name Logging Parameters
     * @{
     */
    /** @brief DataFlash logging bitmask - each bit enables a message group for logging */
    AP_Int32        log_bitmask;
    /** @} */

    /**
     * @name Safety and Failsafe Parameters
     * @{
     */
    /** @brief Automatic disarm delay after landing (seconds, 0=never disarm) */
    AP_Int8         disarm_delay;
    
    /** @brief EKF failsafe action (1=land, 2=RTL, 3=land even in stabilize) */
    AP_Int8         fs_ekf_action;
    
    /** @brief Crash check enable (0=disabled, 1=disarm on crash, 2=disarm or parachute) */
    AP_Int8         fs_crash_check;
    
    /** @brief EKF variance threshold for triggering failsafe (unitless, typical 0.8) */
    AP_Float        fs_ekf_thresh;
    /** @} */

    /**
     * @name Control Limit Parameters
     * @brief Maximum velocity and position error limits for flight controller
     * @{
     */
    /** @brief Maximum horizontal velocity (m/s) - safety limit in XY plane */
    AP_Float        max_vel_xy;
    
    /** @brief Maximum vertical velocity (m/s) - safety limit for climb/descent */
    AP_Float        max_vel_z;
    
    /** @brief Maximum yaw rate (deg/s) - safety limit for rotation rate */
    AP_Float        max_vel_yaw;
    
    /** @brief Maximum horizontal position error (m) - position controller limit */
    AP_Float        max_pos_xy;
    
    /** @brief Maximum vertical position error (m) - altitude controller limit */
    AP_Float        max_pos_z;
    
    /** @brief Maximum yaw position error (degrees) - heading controller limit */
    AP_Float        max_pos_yaw;
    /** @} */

    /**
     * @name Control Mode Parameters
     * @{
     */
    /** @brief Simple mode enable (0=disabled, 1=enabled) - simplifies orientation for novice pilots */
    AP_Int8         simple_mode;
    
    /** @brief Disturbance compensation mask - enables wind rejection on specific axes (bitmask) */
    AP_Int16        dis_mask;
    
    /** @brief PID controller deadzone - minimum error before activation (prevents oscillation) */
    AP_Float        pid_dz;
    /** @} */

    /**
     * @name RTL Parameters
     * @{
     */
    /** @brief RTL altitude type (0=relative to home, 1=relative to terrain if available) */
    AP_Int8         rtl_alt_type;
    /** @} */

    /**
     * @name RC Output Parameters
     * @{
     */
    /** @brief RC output update rate for fast channels (Hz, typical 50-490) */
    AP_Int16        rc_speed;
    /** @} */

    /**
     * @brief Default constructor for Parameters class
     * 
     * @details Parameter initialization is handled by AP_Param system using
     *          var_info[] table defined in Parameters.cpp. Default values are
     *          specified in that table, not in this constructor.
     *          
     * @note Keep parameter declarations above in the same order as they appear
     *       in the var_info[] table for maintainability
     */
    Parameters()
    {
    }
};

/**
 * @class ParametersG2
 * @brief Secondary parameter storage class for Blimp vehicle
 * 
 * @details ParametersG2 provides a second block of parameters to extend beyond the
 *          256 top-level key limitation of the AP_Param system. This class is
 *          registered as k_param_g2 in the main Parameters class.
 *          
 *          This secondary parameter block allows the Blimp vehicle to have more than
 *          256 parameter groups without restructuring the entire parameter system.
 *          Parameters in this class can be accessed via the "g2." prefix in parameter
 *          names (e.g., g2.dev_options).
 *          
 *          Parameters in G2 include:
 *          - Advanced control features (acro expo, frame class)
 *          - Hardware interfaces (RC channels, servo channels)
 *          - Pilot input configuration (speed limits)
 *          - Additional failsafe options (vibration, GCS timeout)
 *          - Landing configuration (final altitude threshold)
 *          - Developer/debug options
 *          
 * @note Default values are defined in Parameters.cpp var_info[] table
 * @see Parameters class for primary parameter block
 */
class ParametersG2
{
public:
    /**
     * @brief Constructor for ParametersG2 class
     * @details Initialization handled by AP_Param system via var_info[] table
     */
    ParametersG2(void);

    /**
     * @brief AP_Param metadata table for G2 parameters
     * @details Defines parameter structure, names, defaults, and storage layout
     * @note Defined in Parameters.cpp
     */
    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @name Waypoint Navigation Parameters
     * @{
     */
    /** @brief Minimum altitude for waypoint navigation control during takeoff (cm above home) */
    AP_Float wp_navalt_min;
    /** @} */

    /**
     * @name Developer Options
     * @{
     */
    /** @brief Developer options bitmask - enables experimental or debug features */
    AP_Int32 dev_options;
    /** @} */

    /**
     * @name Acro Mode Parameters
     * @{
     */
    /** @brief Acrobatic mode yaw expo (0-1, 0=linear, 1=exponential response curve) */
    AP_Float acro_y_expo;
    /** @} */

    /**
     * @name Frame Configuration
     * @{
     */
    /** @brief Frame class identifier - defines blimp physical configuration and control mapping */
    AP_Int8 frame_class;
    /** @} */

    /**
     * @name RC and Servo Configuration
     * @{
     */
    /** @brief RC input channel configuration and processing - handles pilot stick inputs */
    RC_Channels_Blimp rc_channels;
    
    /** @brief Servo output channel configuration - manages fin/control surface outputs */
    SRV_Channels servo_channels;
    /** @} */

    /**
     * @name Pilot Input Parameters
     * @{
     */
    /** @brief Pilot maximum descent speed (cm/s) - limits commanded downward velocity */
    AP_Int16    pilot_speed_dn;
    /** @} */

    /**
     * @name Landing Parameters
     * @{
     */
    /** @brief Landing final stage altitude threshold (cm) - switches to final descent mode */
    AP_Int16 land_alt_low;
    /** @} */

    /**
     * @name Vibration Failsafe Parameters
     * @{
     */
    /** @brief Vibration failsafe enable (0=disabled, 1=enabled) - monitors IMU vibration levels */
    AP_Int8 fs_vibe_enabled;
    /** @} */

    /**
     * @name Failsafe Options
     * @{
     */
    /** @brief Failsafe options bitmask - modifies failsafe behaviors (see FS_Options in documentation) */
    AP_Int32 fs_options;
    
    /** @brief GCS failsafe timeout (seconds) - time without heartbeat before triggering GCS failsafe */
    AP_Float fs_gcs_timeout;
    /** @} */
};

/**
 * @brief AP_Param metadata table for primary Parameters class
 * @details Defines the complete parameter structure including names, types, defaults,
 *          ranges, and EEPROM storage layout for all Blimp parameters.
 * @note Implemented in Parameters.cpp
 * @see Parameters class for parameter declarations
 */
extern const AP_Param::Info        var_info[];
