/**
 * @file Parameters.h
 * @brief ArduSub parameter structure and registration system
 * 
 * @details This file defines the Parameters class which contains all user-configurable
 *          settings for ArduSub underwater vehicles. Parameters are stored persistently
 *          in EEPROM/Flash using the AP_Param system and are accessible via MAVLink
 *          protocol from ground control stations.
 * 
 *          The file includes:
 *          - Parameter ID enumeration for EEPROM addressing
 *          - Parameter variable declarations organized by functional groups
 *          - ParametersG2 secondary parameter block for parameter space expansion
 *          - Default parameter values specific to ArduSub
 * 
 * @note ArduSub parameters follow the naming convention with prefixes like:
 *       - PILOT_* for pilot input related parameters
 *       - FAILSAFE_* for failsafe threshold and behavior parameters
 *       - ACRO_* for acrobatic flight mode parameters
 *       - MOT_* for motor configuration parameters
 * 
 * @warning Changing parameter IDs or reordering the enum will break EEPROM compatibility
 *          with existing vehicles, causing parameter values to be lost or misinterpreted.
 * 
 * @see AP_Param for the underlying parameter storage and management system
 * @see Parameters.cpp for parameter group definitions and var_info tables
 * 
 * Source: ArduSub/Parameters.h:1-468
 */

#pragma once

#define AP_PARAM_VEHICLE_NAME sub

#include <AP_Common/AP_Common.h>

#include <AP_Arming/AP_Arming.h>
#include "actuators.h"

/**
 * @class Parameters
 * @brief Global parameter class containing all ArduSub user-configurable settings
 * 
 * @details The Parameters class serves as the central repository for all tunable
 *          settings in ArduSub. It uses the AP_Param system for persistent storage
 *          in EEPROM or flash memory, allowing parameters to survive power cycles
 *          and be modified via ground control stations.
 * 
 *          **Parameter Storage System:**
 *          - Each parameter has a unique ID (k_param_*) used for EEPROM addressing
 *          - Parameters are organized into logical groups for maintainability
 *          - The k_format_version tracks parameter layout changes
 *          - ParametersG2 provides additional parameter space beyond 256 IDs
 * 
 *          **Parameter Organization:**
 *          The parameter ID enumeration is organized into ranges:
 *          - 0-9: Core system parameters (version, G2 block, simulation)
 *          - 10-19: Telemetry parameters (GCS channels)
 *          - 20-29: Hardware/software configuration (board, scheduler, arming)
 *          - 30-49: Sensor objects (IMU, compass, barometer, GPS, etc.)
 *          - 50-64: Navigation libraries (AHRS, EKF, attitude/position control)
 *          - 65-74: Motor and peripheral hardware
 *          - 75-94: RC channels (deprecated) and joystick gains
 *          - 95-165: Joystick button mappings (32 buttons)
 *          - 126-139: PID controllers (deprecated)
 *          - 140-191: Failsafe thresholds and behaviors
 *          - 165-213: Miscellaneous Sub settings
 *          - 200-225: Flight modes and RSSI
 *          - 220-234: Acro mode parameters
 *          - 237-258: Additional settings (lights, pilot speeds, vehicle common)
 * 
 *          **EEPROM Compatibility:**
 *          Parameter IDs must remain stable across firmware versions to maintain
 *          EEPROM compatibility. When a parameter becomes obsolete, its ID should
 *          be marked as "unused" or "deprecated" but never reassigned to prevent
 *          conflicts with vehicles running older firmware versions.
 * 
 * @note Parameter values are loaded at startup from EEPROM and cached in RAM
 * @note Modified parameters are saved back to EEPROM after validation
 * @note Ground stations access parameters via MAVLink PARAM_REQUEST_READ/WRITE
 * 
 * @warning Never reorder or renumber parameter IDs - this breaks compatibility
 * @warning Never delete parameter IDs - mark as unused/deprecated instead
 * @warning Incrementing k_format_version forces parameter reset on vehicles
 * 
 * @see ParametersG2 for the secondary parameter block
 * @see AP_Param::setup() for parameter system initialization
 */
class Parameters {
public:
    /**
     * @brief Parameter layout format version number
     * 
     * @details The format version tracks the parameter enumeration layout and must be
     *          incremented whenever the parameter ID assignments change in an incompatible
     *          way. This prevents older parameter values stored in EEPROM from being
     *          incorrectly interpreted by newer firmware.
     * 
     *          When the firmware detects a format version mismatch between the stored
     *          parameters and the current code, it will reset all parameters to defaults
     *          rather than risk using incorrect values that could affect vehicle safety.
     * 
     * @note Current version is 1 for ArduSub
     * @warning Only increment this when making breaking changes to parameter IDs
     * @warning Incrementing this will reset parameters to defaults on all vehicles
     */
    static const uint16_t        k_format_version = 1;

    /**
     * @brief Parameter ID enumeration for EEPROM addressing
     * 
     * @details This enumeration assigns a unique ID number to every parameter and
     *          parameter group in ArduSub. These IDs are used by the AP_Param system
     *          to store and retrieve parameter values from EEPROM or flash memory.
     * 
     *          **ID Assignment Rules:**
     *          - Explicit IDs are assigned to start new parameter groups
     *          - Implicit IDs automatically increment from the previous value
     *          - Related parameters are grouped in contiguous ranges
     *          - Groups are arranged numerically for maintainability
     * 
     *          **Adding New Parameters:**
     *          When adding new parameters, find an appropriate unused range or add
     *          at the end of the enumeration. Never reuse IDs from deprecated parameters
     *          within the same format version, as vehicles with older firmware may still
     *          have values stored under those IDs.
     * 
     *          **Parameter Groups:**
     *          The enumeration is organized into functional groups:
     *          - Core: Version, G2 block, simulation (0-9)
     *          - Telemetry: GCS channels, MAVLink system IDs (10-19)
     *          - Hardware config: Board, scheduler, logger, arming (20-29)
     *          - Sensors: IMU, compass, barometer, GPS, rangefinder (30-49)
     *          - Navigation: AHRS, EKF variants, missions, terrain (50-64)
     *          - Motors: Motor control, relays, cameras, mounts (65-74)
     *          - RC/Joystick: RC channels (deprecated), joystick buttons (75-165)
     *          - Controllers: PID tuning parameters (mostly deprecated) (126-139)
     *          - Failsafes: Leak, pressure, temperature, pilot input, GCS (140-191)
     *          - Flight modes: Mode selection and channel mapping (200-225)
     *          - Acro mode: Acrobatic flight tuning (220-234)
     *          - Misc: Logging, speeds, lights, vehicle common (165-258)
     * 
     * @note Entries without explicit assignment get the next sequential ID
     * @note Some ID ranges have gaps to allow for future expansion
     * 
     * @warning NEVER change parameter IDs - breaks EEPROM compatibility with existing vehicles
     * @warning NEVER reorder parameters - AP_Param depends on these exact ID values
     * @warning NEVER reuse deprecated parameter IDs within the same format version
     * @warning Changing IDs without incrementing k_format_version causes data corruption
     */
    enum {
        // ========================================================================
        // Core System Parameters (0-9)
        // ========================================================================
        
        /**
         * Parameter layout format version - always ID 0
         * Used to detect parameter structure changes and trigger reset if needed
         */
        k_param_format_version = 0,
        
        /** Software type identifier (unused - reserved for future use) */
        k_param_software_type,

        /** ParametersG2 secondary parameter block (extends beyond 256 ID limit) */
        k_param_g2,

        /** SITL (Software In The Loop) simulation parameters */
        k_param_sitl,
        
        /** OSD (On-Screen Display) configuration parameters */
        k_param_osd,

        // ========================================================================
        // Telemetry Parameters (10-19)
        // Ground Control Station communication and MAVLink system identification
        // ========================================================================
        
        /** GCS channel 0 - deprecated, unused in ArduPilot-4.7+ */
        k_param_gcs0_unused = 10,
        
        /** GCS channel 1 - deprecated, unused in ArduPilot-4.7+ */
        k_param_gcs1_unused,
        
        /** GCS channel 2 - deprecated, unused in ArduPilot-4.7+ */
        k_param_gcs2_unused,
        
        /** GCS channel 3 - deprecated, unused in ArduPilot-4.7+ */
        k_param_gcs3_unused,
        
        /** MAVLink system ID for this vehicle (old location, moved to common) */
        k_param_sysid_this_mav_old,
        
        /** MAVLink system ID for ground control station (old location, moved) */
        k_param_sysid_my_gcs_old,

        // ========================================================================
        // Hardware/Software Configuration (20-29)
        // Board-specific settings, schedulers, logging, arming, and system config
        // ========================================================================
        
        /** Board configuration (Pixhawk, Navigator, Linux, etc.) - hardware variant settings */
        k_param_BoardConfig = 20,
        
        /** Task scheduler configuration (loop rates, performance monitoring) */
        k_param_scheduler,
        
        /** AP_Logger binary logging system (dataflash logging configuration) */
        k_param_logger,
        
        /** Serial port manager (old location, moved to common parameters) */
        k_param_serial_manager_old,
        
        /** AP_Notify library (LED patterns, buzzer tones, visual/audio feedback) */
        k_param_notify,
        
        /** AP_Arming pre-arm and arming safety checks configuration */
        k_param_arming = 26,
        
        /** CAN bus manager (DroneCAN/UAVCAN peripheral communication) */
        k_param_can_mgr,
        
        /** Throttle arming position (deprecated - throttle position required for arming) */
        k_param_thr_arming_position,

        // ========================================================================
        // Sensor Objects (30-49)
        // Physical sensor drivers and sensor fusion subsystems
        // ========================================================================
        
        /** AP_InertialSensor - IMU (gyroscope and accelerometer) configuration and calibration */
        k_param_ins = 30,
        
        /** AP_Compass - magnetometer configuration, calibration, and motor interference compensation */
        k_param_compass,
        
        /** AP_Baro - barometer/depth sensor for underwater pressure-based altitude */
        k_param_barometer,
        
        /** AP_BattMonitor - battery voltage, current monitoring, and capacity estimation */
        k_param_battery,
        
        /** Leak detector - water ingress detection system for watertight enclosures */
        k_param_leak_detector,
        
        /** AP_RangeFinder - distance sensors for collision avoidance and terrain following */
        k_param_rangefinder,
        
        /** AP_GPS - GPS receiver configuration (limited underwater use, surface positioning) */
        k_param_gps,
        
        /** AP_OpticalFlow - optical flow sensor for velocity estimation without GPS */
        k_param_optflow,


        // ========================================================================
        // Navigation Libraries (50-64)
        // Attitude/heading estimation, Kalman filters, position/attitude control, missions
        // ========================================================================
        
        /** AP_AHRS - Attitude and Heading Reference System (sensor fusion, DCM/EKF backend) */
        k_param_ahrs = 50,
        
        /** NavEKF (legacy) - Extended Kalman Filter v1 (deprecated, scheduled for removal) */
        k_param_NavEKF,
        
        /** AP_NavEKF2 - Extended Kalman Filter v2 for inertial navigation and sensor fusion */
        k_param_NavEKF2,
        
        /** AC_AttitudeControl - Attitude (roll/pitch/yaw) PID controllers and rate control */
        k_param_attitude_control,
        
        /** AC_PosControl - Position and velocity controllers (XY horizontal, Z vertical) */
        k_param_pos_control,
        
        /** AC_WPNav - Waypoint navigation with trajectory generation and path following */
        k_param_wp_nav,
        
        /** AP_Mission - Mission command storage, execution, and scripting */
        k_param_mission,
        
        /** AC_Fence - Geofencing (old location, kept for parameter conversion only) */
        k_param_fence_old,
        
        /** AP_Terrain - Terrain database for altitude-above-terrain awareness */
        k_param_terrain,
        
        /** AP_Rally - Rally point system (disabled in ArduSub) */
        k_param_rally,
        
        /** AC_Circle - Circle navigation mode (disabled in ArduSub) */
        k_param_circle_nav,
        
        /** AC_Avoid - Object avoidance using proximity sensors and fence boundaries */
        k_param_avoid,
        
        /** AP_NavEKF3 - Extended Kalman Filter v3 (current recommended EKF version) */
        k_param_NavEKF3,
        
        /** AC_Loiter - Loiter navigation mode for station-keeping */
        k_param_loiter_nav,


        // ========================================================================
        // Motor and Peripheral Hardware (65-74)
        // Thruster motors, relays, cameras, and gimbal mounts
        // ========================================================================
        
        /** AP_Motors - Thruster motor configuration, mixing, and output allocation */
        k_param_motors = 65,
        
        /** AP_Relay - Digital relay outputs for lights, switches, and other accessories */
        k_param_relay,
        
        /** AP_Camera - Camera trigger and control interface */
        k_param_camera,
        
        /** AP_Mount - Camera gimbal mount control (tilt/pan stabilization) */
        k_param_camera_mount,


        // ========================================================================
        // RC Channel Settings (75-88) - DEPRECATED
        // Legacy individual RC channel parameters, now managed via RC_Channels
        // ========================================================================
        
        /** RC channel 1 parameters (deprecated - use RC1_* parameters instead) */
        k_param_rc_1_old = 75,
        k_param_rc_2_old,   ///< RC channel 2 (deprecated)
        k_param_rc_3_old,   ///< RC channel 3 (deprecated)
        k_param_rc_4_old,   ///< RC channel 4 (deprecated)
        k_param_rc_5_old,   ///< RC channel 5 (deprecated)
        k_param_rc_6_old,   ///< RC channel 6 (deprecated)
        k_param_rc_7_old,   ///< RC channel 7 (deprecated)
        k_param_rc_8_old,   ///< RC channel 8 (deprecated)
        k_param_rc_9_old,   ///< RC channel 9 (deprecated)
        k_param_rc_10_old,  ///< RC channel 10 (deprecated)
        k_param_rc_11_old,  ///< RC channel 11 (deprecated)
        k_param_rc_12_old,  ///< RC channel 12 (deprecated)
        k_param_rc_13_old,  ///< RC channel 13 (deprecated)
        k_param_rc_14_old,  ///< RC channel 14 (deprecated)

        // ========================================================================
        // Joystick Gain Parameters (89-94)
        // Adjustable gain/sensitivity settings for joystick control
        // ========================================================================
        
        /** Default joystick input gain/sensitivity (0.0-1.0 scaling factor) */
        k_param_gain_default,
        
        /** Maximum joystick gain limit (prevents excessive sensitivity) */
        k_param_maxGain,
        
        /** Minimum joystick gain limit (prevents insufficient control authority) */
        k_param_minGain,
        
        /** Number of discrete gain settings available for in-flight adjustment */
        k_param_numGainSettings,
        
        /** Camera tilt step size (deprecated - use direct servo control) */
        k_param_cam_tilt_step,
        
        /** Lights brightness step size (deprecated - use PWM servo functions) */
        k_param_lights_step,

        // ========================================================================
        // Joystick Button Mapping (95-165)
        // Function assignments for 32 joystick buttons (0-31)
        // Allows customizable button functions like arm/disarm, mode changes, camera control
        // ========================================================================
        
        /** Joystick button 0 function mapping (JSButton parameter group) */
        k_param_jbtn_0 = 95,
        k_param_jbtn_1,    ///< Joystick button 1 function
        k_param_jbtn_2,    ///< Joystick button 2 function
        k_param_jbtn_3,    ///< Joystick button 3 function
        k_param_jbtn_4,    ///< Joystick button 4 function
        k_param_jbtn_5,    ///< Joystick button 5 function
        k_param_jbtn_6,    ///< Joystick button 6 function
        k_param_jbtn_7,    ///< Joystick button 7 function
        k_param_jbtn_8,    ///< Joystick button 8 function
        k_param_jbtn_9,    ///< Joystick button 9 function
        k_param_jbtn_10,   ///< Joystick button 10 function
        k_param_jbtn_11,   ///< Joystick button 11 function
        k_param_jbtn_12,   ///< Joystick button 12 function
        k_param_jbtn_13,   ///< Joystick button 13 function
        k_param_jbtn_14,   ///< Joystick button 14 function
        k_param_jbtn_15,   ///< Joystick button 15 function

        /**
         * Extended button mappings (16-31) for MANUAL_CONTROL MAVLink message
         * Supports advanced joysticks with more than 16 buttons
         */
        k_param_jbtn_16,   ///< Joystick button 16 function (extended)
        k_param_jbtn_17,   ///< Joystick button 17 function (extended)
        k_param_jbtn_18,   ///< Joystick button 18 function (extended)
        k_param_jbtn_19,   ///< Joystick button 19 function (extended)
        k_param_jbtn_20,   ///< Joystick button 20 function (extended)
        k_param_jbtn_21,   ///< Joystick button 21 function (extended)
        k_param_jbtn_22,   ///< Joystick button 22 function (extended)
        k_param_jbtn_23,   ///< Joystick button 23 function (extended)
        k_param_jbtn_24,   ///< Joystick button 24 function (extended)
        k_param_jbtn_25,   ///< Joystick button 25 function (extended)
        k_param_jbtn_26,   ///< Joystick button 26 function (extended)
        k_param_jbtn_27,   ///< Joystick button 27 function (extended)
        k_param_jbtn_28,   ///< Joystick button 28 function (extended)
        k_param_jbtn_29,   ///< Joystick button 29 function (extended)
        k_param_jbtn_30,   ///< Joystick button 30 function (extended)
        k_param_jbtn_31,   ///< Joystick button 31 function (extended)

        // ========================================================================
        // PID Controllers (126-131) - DEPRECATED
        // Legacy position and velocity controller gains, moved to AC_PosControl
        // ========================================================================
        
        /** XY position controller P gain (deprecated - now in PSC_POSXY_P) */
        k_param_p_pos_xy = 126,
        
        /** Altitude hold P gain (deprecated - now in PSC_POSZ_P) */
        k_param_p_alt_hold,
        
        /** XY velocity PI controller (deprecated - now in PSC_VELXY_*) */
        k_param_pi_vel_xy,
        
        /** Z velocity P gain (deprecated - now in PSC_VELZ_P) */
        k_param_p_vel_z,
        
        /** Z acceleration PID controller (deprecated - now in PSC_ACCZ_*) */
        k_param_pid_accel_z,


        // ========================================================================
        // Failsafe Thresholds and Behaviors (140-191)
        // Safety mechanisms for handling sensor failures, communication loss, and
        // environmental hazards specific to underwater operation
        // ========================================================================
        
        /**
         * Ground Control Station failsafe action
         * Behavior when GCS telemetry link is lost (surface, hold position, disarm)
         */
        k_param_failsafe_gcs = 140,
        /**
         * Leak detector failsafe action
         * Behavior when water ingress detected in electronics enclosure (surface, hold, disarm)
         */
        k_param_failsafe_leak,
        
        /**
         * Internal pressure failsafe action
         * Response when enclosure pressure exceeds safe limits (compromised seal)
         */
        k_param_failsafe_pressure,
        
        /**
         * Maximum internal enclosure pressure threshold
         * Pressure limit in Pascals before triggering failsafe (detects seal failure)
         */
        k_param_failsafe_pressure_max,
        
        /**
         * Internal temperature failsafe action
         * Response when electronics temperature exceeds safe operating range
         */
        k_param_failsafe_temperature,
        
        /**
         * Maximum internal temperature threshold
         * Temperature limit in degrees Celsius before triggering failsafe
         */
        k_param_failsafe_temperature_max,
        
        /**
         * Terrain failsafe behavior
         * Action when terrain data unavailable during terrain-following navigation
         */
        k_param_failsafe_terrain,
        
        /** EKF failsafe variance threshold (navigation solution quality monitoring) */
        k_param_fs_ekf_thresh,
        
        /** EKF failsafe action (behavior when navigation solution degrades) */
        k_param_fs_ekf_action,
        
        /** Crash detection check enable/disable (unused in ArduSub currently) */
        k_param_fs_crash_check,
        
        /** Battery failsafe enable (deprecated - moved to AP_BattMonitor parameters) */
        k_param_failsafe_battery_enabled,
        
        /** Battery mAh failsafe threshold (deprecated - moved to BATT_LOW_MAH) */
        k_param_fs_batt_mah,
        
        /** Battery voltage failsafe threshold (deprecated - moved to BATT_LOW_VOLT) */
        k_param_fs_batt_voltage,
        
        /**
         * Pilot input failsafe action
         * Behavior when joystick input lost (hold position, surface, disarm)
         */
        k_param_failsafe_pilot_input,
        
        /** Pilot input timeout duration (seconds before triggering pilot input failsafe) */
        k_param_failsafe_pilot_input_timeout,
        
        /** GCS communication timeout (seconds before triggering GCS failsafe) */
        k_param_failsafe_gcs_timeout,


        // ========================================================================
        // Miscellaneous Sub Settings (165-213)
        // Logging, control limits, speeds, camera, lights, and other settings
        // ========================================================================
        
        /**
         * Binary logging bitmask
         * Selects which message types to log to dataflash (each bit enables a category)
         */
        k_param_log_bitmask = 165,
        /**
         * Maximum attitude angle limit (degrees)
         * Limits maximum roll/pitch angle in stabilized modes for safety
         */
        k_param_angle_max = 167,
        
        /** Rangefinder gain (deprecated - altitude hold from rangefinder) */
        k_param_rangefinder_gain,
        
        /**
         * Waypoint yaw behavior
         * Controls how vehicle orients during autonomous missions (face next WP, hold heading, etc.)
         */
        k_param_wp_yaw_behavior = 170,
        
        /**
         * Crosstrack angle limit (degrees)
         * Maximum correction angle for getting back on course during waypoint navigation
         */
        k_param_xtrack_angle_limit,
        
        /**
         * Pilot vertical ascending speed (cm/s)
         * Maximum upward velocity pilot can command (formerly k_param_pilot_velocity_z_max)
         */
        k_param_pilot_speed_up,
        
        /** Pilot vertical acceleration (cm/s²) - maximum Z-axis acceleration */
        k_param_pilot_accel_z,
        
        /** Compass enable flag (deprecated - compass handling changed) */
        k_param_compass_enabled_deprecated,
        
        /** Surface depth threshold (meters) - depth considered "at surface" for mode transitions */
        k_param_surface_depth,
        
        /** Main output PWM frequency (Hz) - servo update rate for thrusters */
        k_param_rc_speed,
        
        /**
         * GCS PID tuning mask
         * Selects which PID controllers send tuning messages to GCS (bitmask)
         */
        k_param_gcs_pid_mask = 178,
        
        /** Throttle input filter (low-pass filter time constant for smoothing) */
        k_param_throttle_filt,
        
        /** Throttle deadzone (used in auto-throttle modes to prevent drift) */
        k_param_throttle_deadzone,
        
        /** Terrain following enable (deprecated - terrain-relative altitude hold) */
        k_param_terrain_follow = 182,
        
        /** RC feel roll/pitch (0=soft, 100=crisp - adjusts response to pilot input) */
        k_param_rc_feel_rp,
        
        /** Throttle gain multiplier (scales vertical velocity commands) */
        k_param_throttle_gain,
        
        /** Camera tilt center position (deprecated - use servo trim instead) */
        k_param_cam_tilt_center,
        
        /** Frame configuration (vectored/non-vectored, thruster layout selection) */
        k_param_frame_configuration,
        
        /** Maximum throttle at surface (limits thruster output when near surface) */
        k_param_surface_max_throttle,
        
        /** Surface thrust without barometer (default vertical thrust when depth sensor unavailable) */
        k_param_surface_nobaro_thrust,
        
        // ========================================================================
        // Flight Mode Configuration (200-225)
        // Flight mode assignments, mode channel selection, and RSSI monitoring
        // ========================================================================
        
        /** Flight mode 1 - mode assigned to first mode switch position */
        k_param_flight_mode1 = 200,
        k_param_flight_mode2,  ///< Flight mode 2
        k_param_flight_mode3,  ///< Flight mode 3
        k_param_flight_mode4,  ///< Flight mode 4
        k_param_flight_mode5,  ///< Flight mode 5
        k_param_flight_mode6,  ///< Flight mode 6
        
        /** Simple modes bitmask (deprecated in ArduSub) */
        k_param_simple_modes,
        
        /** Flight mode selection channel (RC/joystick channel for mode switching) */
        k_param_flight_mode_chan,
#if AP_RSSI_ENABLED
        /** AP_RSSI - Received Signal Strength Indicator monitoring */
        k_param_rssi,
#endif 
        
        // ========================================================================
        // Acro Mode Parameters (220-234)
        // Acrobatic flight mode tuning for direct rate control
        // ========================================================================
        
        /**
         * Acro mode yaw P gain
         * Converts pilot yaw stick input to desired yaw rate (used in all manual modes)
         */
        k_param_acro_yaw_p = 220,
        
        /** Acro trainer enable (provides automatic leveling assistance in acro mode) */
        k_param_acro_trainer,
        
        /** Acro expo (exponential curve on stick inputs for finer center control) */
        k_param_acro_expo,
        
        /** Acro roll/pitch P gain (converts stick input to desired roll/pitch rate) */
        k_param_acro_rp_p,
        
        /** Acro roll balance (trims roll axis for vehicle balance) */
        k_param_acro_balance_roll,
        
        /** Acro pitch balance (trims pitch axis for vehicle balance) */
        k_param_acro_balance_pitch,

        // ========================================================================
        // Peripheral Sensors and Libraries (232-236)
        // ========================================================================
        
        /** RPM sensor configuration (disabled in ArduSub - not commonly used underwater) */
        k_param_rpm_sensor = 232,

        /** RC_Mapper library (disabled - RC input mapping handled differently) */
        k_param_rcmap,

        /** GCS channel 4 (deprecated, unused in ArduPilot-4.7+) */
        k_param_gcs4_unused,
        
        /** GCS channel 5 (deprecated, unused in ArduPilot-4.7+) */
        k_param_gcs5_unused,
        
        /** GCS channel 6 (deprecated, unused in ArduPilot-4.7+) */
        k_param_gcs6_unused,

        // ========================================================================
        // Additional Settings (237-258)
        // Camera, lights, pilot speeds, and vehicle common parameters
        // ========================================================================
        
        /** Camera slew rate limit (deprecated - maximum gimbal movement rate) */
        k_param_cam_slew_limit = 237,
        
        /** Lights brightness step count (number of discrete brightness levels) */
        k_param_lights_steps,
        
        /** Pilot vertical descending speed (cm/s) - maximum downward velocity */
        k_param_pilot_speed_dn,
        
        /** Rangefinder minimum signal quality (threshold for valid readings) */
        k_param_rangefinder_signal_min,
        
        /** Surface tracking depth target (meters below surface for surftrak mode) */
        k_param_surftrak_depth,
        
        /** Pilot horizontal speed (cm/s) - maximum XY velocity pilot can command */
        k_param_pilot_speed,
        
        /** Throttle failsafe behavior (deprecated failsafe trigger method) */
        k_param_failsafe_throttle,
        
        /** Throttle failsafe PWM value (deprecated threshold for detecting RC loss) */
        k_param_failsafe_throttle_value,
        
        /** Vehicle common parameter block (shared parameters across all vehicle types) */
        k_param_vehicle = 257,
        
        /** GCS parameter group (ground station communication settings) */
        k_param__gcs = 258,
    };

    /**
     * @brief Parameter format version storage
     * @details Stores k_format_version value in EEPROM for compatibility checking
     */
    AP_Int16        format_version;

    // ========================================================================
    // Parameter Variable Declarations
    // Organized by functional groups matching the parameter ID enumeration
    // ========================================================================

    // Telemetry and Control Filtering
    // ========================================================================
    
    /** Throttle input low-pass filter time constant (seconds) for smooth control response */
    AP_Float        throttle_filt;

    // Rangefinder Configuration (conditional)
    // ========================================================================
#if AP_RANGEFINDER_ENABLED
    /** Minimum rangefinder signal quality (0-100%) required for valid altitude readings */
    AP_Int8         rangefinder_signal_min;
    
    /** Surface tracking target depth (meters) - surftrak mode maintains this depth below surface */
    AP_Float        surftrak_depth;
#endif

    // Failsafe Thresholds and Behaviors
    // ========================================================================
    
    /** Leak detector failsafe action (0=disabled, 1=warn, 2=surface, 3=hold) */
    AP_Int8         failsafe_leak;
    
    /** Ground Control Station failsafe action (0=disabled, 1=warn, 2=surface, 3=hold, 4=disarm) */
    AP_Int8         failsafe_gcs;
    
    /** Internal pressure failsafe action (enclosure seal monitoring) */
    AP_Int8         failsafe_pressure;
    
    /** Internal temperature failsafe action (electronics overheating protection) */
    AP_Int8         failsafe_temperature;
    
    /** Maximum internal pressure threshold (Pascals) before triggering pressure failsafe */
    AP_Int32        failsafe_pressure_max;
    
    /** Maximum internal temperature threshold (°C) before triggering temperature failsafe */
    AP_Int8         failsafe_temperature_max;
    
    /** Terrain data failsafe action (behavior when terrain database unavailable) */
    AP_Int8         failsafe_terrain;
    
    /** Pilot input loss failsafe action (joystick communication failure response) */
    AP_Int8         failsafe_pilot_input;
    
    /** Pilot input timeout (seconds) before triggering pilot input failsafe */
    AP_Float        failsafe_pilot_input_timeout;
    
    /** GCS communication timeout (seconds) before triggering GCS failsafe */
    AP_Float        failsafe_gcs_timeout;

    // Waypoint Navigation Parameters
    // ========================================================================
    
    /** Crosstrack correction angle limit (degrees) - max angle for course correction */
    AP_Int8         xtrack_angle_limit;

    /**
     * Waypoint yaw behavior during autonomous missions
     * (0=never change yaw, 1=face next waypoint, 2=face next waypoint except RTL, 3=face along GPS course)
     */
    AP_Int8         wp_yaw_behavior;
    
    /**
     * RC feel roll/pitch responsiveness (0-100)
     * Controls vehicle response to user input: 0=extremely soft, 100=extremely crisp
     */
    AP_Int8         rc_feel_rp;

    // Pilot Speed and Acceleration Limits
    // ========================================================================
    
    /** Maximum vertical ascending velocity pilot can command (cm/s) */
    AP_Int16        pilot_speed_up;
    
    /** Maximum vertical descending velocity pilot can command (cm/s) */
    AP_Int16        pilot_speed_dn;
    
    /** Maximum horizontal (XY) velocity pilot can command (cm/s) */
    AP_Int16        pilot_speed;
    
    /** Maximum vertical acceleration pilot can command (cm/s²) */
    AP_Int16        pilot_accel_z;

    // Throttle Configuration
    // ========================================================================
    
    /** Throttle deadzone size (PWM units) for center stick position */
    AP_Int16        throttle_deadzone;
    
    /** Throttle failsafe enable (deprecated method of RC loss detection) */
    AP_Int8         failsafe_throttle;
    
    /** Throttle failsafe PWM value (deprecated threshold for detecting RC loss) */
    AP_Int16        failsafe_throttle_value;
    
    /** Throttle arming position (deprecated - required PWM position for arming) */
    AP_Int16        thr_arming_position;
    

    // Miscellaneous System Configuration
    // ========================================================================
    
    /**
     * Binary logging bitmask - selects message types to log
     * Each bit enables a category (ATTITUDE, GPS, RCIN, RCOUT, etc.)
     */
    AP_Int32        log_bitmask;

    /** EKF failsafe action (0=disabled, 1=warn, 2=surface, 3=hold) */
    AP_Int8         fs_ekf_action;
    
    /** Crash detection check enable (0=disabled, 1=enabled - unused in ArduSub currently) */
    AP_Int8         fs_crash_check;
    
    /** EKF variance threshold for triggering navigation failsafe */
    AP_Float        fs_ekf_thresh;
    
    /** GCS PID tuning bitmask - selects which PID controllers send telemetry */
    AP_Int16        gcs_pid_mask;

    /** Main RC/servo output update rate (Hz) - PWM frequency for thruster outputs */
    AP_Int16        rc_speed;

    // Joystick Gain Configuration
    // ========================================================================
    
    /** Default joystick input gain (0.0-1.0 scaling factor for all axes) */
    AP_Float        gain_default;
    
    /** Maximum joystick gain setting (upper limit for adjustable gain) */
    AP_Float        maxGain;
    
    /** Minimum joystick gain setting (lower limit for adjustable gain) */
    AP_Float        minGain;
    
    /** Number of discrete gain settings for in-flight adjustment */
    AP_Int8         numGainSettings;
    
    /** Throttle gain multiplier (scales vertical thrust commands) */
    AP_Float        throttle_gain;

    /** Lights brightness step count (discrete brightness levels 0-N) */
    AP_Int16        lights_steps;

    // Joystick Button Function Assignments (32 buttons)
    // Each JSButton parameter defines the function mapped to that button
    // ========================================================================
    
    /** Joystick button 0 function assignment */
    JSButton        jbtn_0;
    /** Joystick button 1 function assignment */
    JSButton        jbtn_1;
    /** Joystick button 2 function assignment */
    JSButton        jbtn_2;
    /** Joystick button 3 function assignment */
    JSButton        jbtn_3;
    /** Joystick button 4 function assignment */
    JSButton        jbtn_4;
    /** Joystick button 5 function assignment */
    JSButton        jbtn_5;
    /** Joystick button 6 function assignment */
    JSButton        jbtn_6;
    /** Joystick button 7 function assignment */
    JSButton        jbtn_7;
    /** Joystick button 8 function assignment */
    JSButton        jbtn_8;
    /** Joystick button 9 function assignment */
    JSButton        jbtn_9;
    /** Joystick button 10 function assignment */
    JSButton        jbtn_10;
    /** Joystick button 11 function assignment */
    JSButton        jbtn_11;
    /** Joystick button 12 function assignment */
    JSButton        jbtn_12;
    /** Joystick button 13 function assignment */
    JSButton        jbtn_13;
    /** Joystick button 14 function assignment */
    JSButton        jbtn_14;
    /** Joystick button 15 function assignment */
    JSButton        jbtn_15;
    
    // Buttons 16-31 from MAVLink manual_control message extension
    /** Joystick button 16 function assignment */
    JSButton        jbtn_16;
    /** Joystick button 17 function assignment */
    JSButton        jbtn_17;
    /** Joystick button 18 function assignment */
    JSButton        jbtn_18;
    /** Joystick button 19 function assignment */
    JSButton        jbtn_19;
    /** Joystick button 20 function assignment */
    JSButton        jbtn_20;
    /** Joystick button 21 function assignment */
    JSButton        jbtn_21;
    /** Joystick button 22 function assignment */
    JSButton        jbtn_22;
    /** Joystick button 23 function assignment */
    JSButton        jbtn_23;
    /** Joystick button 24 function assignment */
    JSButton        jbtn_24;
    /** Joystick button 25 function assignment */
    JSButton        jbtn_25;
    /** Joystick button 26 function assignment */
    JSButton        jbtn_26;
    /** Joystick button 27 function assignment */
    JSButton        jbtn_27;
    /** Joystick button 28 function assignment */
    JSButton        jbtn_28;
    /** Joystick button 29 function assignment */
    JSButton        jbtn_29;
    /** Joystick button 30 function assignment */
    JSButton        jbtn_30;
    /** Joystick button 31 function assignment */
    JSButton        jbtn_31;

    // Acro Flight Mode Parameters
    // ========================================================================
    
    /** Acro mode roll/pitch P gain (rate response proportional gain) */
    AP_Float        acro_rp_p;
    
    /** Acro mode yaw P gain (yaw rate response proportional gain) */
    AP_Float        acro_yaw_p;
    
    /** Acro mode roll auto-leveling strength (0.0=none, 1.0=full leveling) */
    AP_Float        acro_balance_roll;
    
    /** Acro mode pitch auto-leveling strength (0.0=none, 1.0=full leveling) */
    AP_Float        acro_balance_pitch;
    
    /** Acro trainer mode (0=disabled, 1=leveling, 2=leveling and limiting) */
    AP_Int8         acro_trainer;
    
    /** Acro mode expo (0.0=linear, 1.0=exponential stick response curve) */
    AP_Float        acro_expo;
    
#if AP_SUB_RC_ENABLED

    // Flight Mode Channel Assignments (RC input to mode mapping)
    // ========================================================================
    
    /** Flight mode for RC mode switch position 1 */
    AP_Int8         flight_mode1;
    
    /** Flight mode for RC mode switch position 2 */
    AP_Int8         flight_mode2;
    
    /** Flight mode for RC mode switch position 3 */
    AP_Int8         flight_mode3;
    
    /** Flight mode for RC mode switch position 4 */
    AP_Int8         flight_mode4;
    
    /** Flight mode for RC mode switch position 5 */
    AP_Int8         flight_mode5;
    
    /** Flight mode for RC mode switch position 6 */
    AP_Int8         flight_mode6;
    
    /** Simple mode bitmask (deprecated - not used in ArduSub) */
    AP_Int8         simple_modes;
    
    /** RC channel number for flight mode switch (1-16) */
    AP_Int8         flight_mode_chan;
#endif 

    // Surface Mode Configuration
    // ========================================================================
    
    /** Surface mode target depth (meters) - depth to maintain in surface mode */
    AP_Float                surface_depth;
    
    /** Frame configuration type (BlueROV1, BlueROV2, vectored, etc.) */
    AP_Int8                 frame_configuration;

    /** Maximum throttle output in surface mode (0.0-1.0) - prevents diving in surface mode */
    AP_Float surface_max_throttle;

    // Note: keep initializers here in the same order as they are declared
    // above.
    Parameters()
    {
    }
};

/**
 * @class ParametersG2
 * @brief Secondary parameter block providing additional parameter space beyond 256 IDs
 * 
 * @details The AP_Param system originally had a limit of 256 top-level parameter IDs
 *          (k_param_* enumeration values 0-255 in the Parameters class). As ArduSub
 *          evolved, more parameters were needed than could fit in this space.
 * 
 *          ParametersG2 was introduced as a "second generation" parameter block that
 *          acts as a single entry (k_param_g2) in the main Parameters enum but
 *          internally contains its own set of parameters. This allows virtually
 *          unlimited parameter expansion without breaking EEPROM compatibility with
 *          existing vehicles.
 * 
 *          **Architecture:**
 *          - Main Parameters class: Limited to 256 top-level IDs (k_param_0 to k_param_255)
 *          - ParametersG2: Occupies k_param_g2 (value 2) in main enum
 *          - ParametersG2 internally: Contains its own var_info[] table with sub-parameters
 * 
 *          **Parameter Access:**
 *          From ground control stations, G2 parameters appear with the same names
 *          but are stored in a different EEPROM region accessed via the G2 group.
 * 
 *          **Current G2 Parameters:**
 *          - proximity: Object avoidance library parameters (conditional on HAL_PROXIMITY_ENABLED)
 *          - rc_channels: RC input channel configuration (Sub-specific subclass)
 *          - servo_channels: Servo output range configuration
 *          - backup_origin_lat/lon/alt: Emergency origin location for navigation fallback
 *          - surface_nobaro_thrust: Thrust output when surfaced without barometer feedback
 *          - actuators: Motor and servo actuator configuration
 * 
 * @note Adding new parameters to G2 does not break EEPROM compatibility
 * @note G2 parameters use AP_Param::GroupInfo like main Parameters class
 * @note Future parameter additions should go in G2 to preserve compatibility
 * 
 * @see Parameters for the main parameter block
 * @see AP_Param::GroupInfo for parameter registration mechanism
 */
class ParametersG2 {
public:
    /**
     * @brief Constructor for ParametersG2
     * @details Initializes the secondary parameter block
     */
    ParametersG2(void);

    /**
     * @brief AP_Param group information table for ParametersG2 parameters
     * @details Defines the parameter registration, storage, and metadata for all
     *          parameters in this secondary block. Used by AP_Param to manage
     *          EEPROM storage and MAVLink parameter protocol access.
     */
    static const struct AP_Param::GroupInfo var_info[];

#if HAL_PROXIMITY_ENABLED
    /**
     * Object avoidance/proximity sensor library
     * Manages rangefinder data for obstacle detection and avoidance
     * Only available when HAL_PROXIMITY_ENABLED is defined
     */
    AP_Proximity proximity;
#endif

    /**
     * RC input channel configuration (ArduSub-specific subclass)
     * Manages RC receiver input channels, failsafe behavior, and input mapping
     */
    RC_Channels_Sub rc_channels;

    /**
     * Servo output channel configuration
     * Controls servo/motor output ranges, functions, trim values, and reversing
     */
    SRV_Channels servo_channels;

    /** Backup origin latitude (degrees) for emergency navigation fallback */
    AP_Float backup_origin_lat;
    
    /** Backup origin longitude (degrees) for emergency navigation fallback */
    AP_Float backup_origin_lon;
    
    /** Backup origin altitude (meters above sea level) for emergency navigation fallback */
    AP_Float backup_origin_alt;
    
    /** Surface mode thrust output when barometer unavailable (0.0-1.0) */
    AP_Float surface_nobaro_thrust;
    
    /**
     * Actuator configuration object
     * Manages motor and servo actuator parameters specific to ArduSub
     */
    Actuators actuators;

};

/**
 * @brief Main parameter group information table (extern declaration)
 * @details References the var_info table defined in Parameters.cpp that contains
 *          the complete parameter registration for the main Parameters class.
 *          This table maps parameter IDs to their storage locations, types,
 *          names, and metadata used by the AP_Param system.
 * 
 * @note Actual definition is in Parameters.cpp
 * @see Parameters.cpp for the complete var_info table implementation
 */
extern const AP_Param::Info        var_info[];

/**
 * @brief ArduSub-specific default parameter values
 * @details This table defines default values for parameters that differ from
 *          the library defaults or need specific initialization for underwater
 *          vehicle operation. These defaults are applied when:
 *          - Parameters are loaded for the first time (new vehicle)
 *          - Parameters are reset via GCS or MAVLink command
 *          - k_format_version changes, forcing parameter reset
 * 
 *          **Key ArduSub Default Customizations:**
 *          - BRD_SAFETY_DEFLT: Disabled (0) - safety switch not used underwater
 *          - ARMING_CHECK: Reduced checks (RC, voltage, battery only)
 *          - CIRCLE_RATE: Slower rate (2.0 deg/s) for underwater navigation
 *          - ATC_ACCEL_Y_MAX: Higher yaw accel (110000 deg/s²) for responsive turning
 *          - ATC_RATE_Y_MAX: Standard yaw rate (180 deg/s)
 *          - RC3_TRIM: Throttle center (1500) for bidirectional thrusters
 *          - COMPASS_OFFS_MAX: Higher offset tolerance (1000) for underwater interference
 *          - INS_GYR_CAL: Disabled (0) - gyro calibration at boot not needed
 *          - RCMAP_ROLL/PITCH/FORWARD/LATERAL: Sub-specific channel mapping
 *          - RC7_OPTION/RC8_OPTION: Mount control (yaw/pitch) assignments
 *          - MOT_PWM_MIN/MAX: Standard ESC range (1100-1900μs)
 *          - PSC_JERK_Z: Lower jerk (50.0) for smooth depth changes
 *          - WPNAV_SPEED: Conservative navigation speed (100 cm/s)
 *          - PILOT_SPEED_UP: Moderate ascent rate (100 cm/s)
 *          - PSC_VELXY_P: Position control gain (6.0)
 *          - EK3_SRC1_VELZ: Disable GPS vertical velocity (0) - use baro only
 * 
 *          **Platform-Specific Defaults:**
 *          - Navigator board: Battery monitor, leak detector, servo assignments
 *          - Standard boards: Barometer probing, lights, mount assignments
 * 
 * @note Default values are applied only when parameter storage is initialized
 * @note Changes to defaults do not affect existing vehicles with saved parameters
 * @note Platform-specific defaults use conditional compilation (#if CONFIG_HAL_BOARD_SUBTYPE)
 * 
 * @warning Changing defaults affects only new vehicles or after parameter reset
 * 
 * Source: ArduSub/Parameters.h:1089-1137
 */
static const struct AP_Param::defaults_table_struct defaults_table[] = {
    { "BRD_SAFETY_DEFLT",    0 },
    { "ARMING_CHECK",        uint32_t(AP_Arming::Check::RC) |
                             uint32_t(AP_Arming::Check::VOLTAGE) |
                             uint32_t(AP_Arming::Check::BATTERY)},
    { "CIRCLE_RATE",         2.0f},
    { "ATC_ACCEL_Y_MAX",     110000.0f},
    { "ATC_RATE_Y_MAX",      180.0f},
    { "RC3_TRIM",            1500},
    { "COMPASS_OFFS_MAX",    1000},
    { "INS_GYR_CAL",         0},
    { "RCMAP_ROLL",          2},
    { "RCMAP_PITCH",         1},
    { "RCMAP_FORWARD",       5},
    { "RCMAP_LATERAL",       6},
#if HAL_MOUNT_ENABLED
    { "MNT1_TYPE",           1},
    { "MNT1_DEFLT_MODE",     MAV_MOUNT_MODE_RC_TARGETING},
    { "MNT1_RC_RATE",        30},
#endif
    { "RC7_OPTION",          214},   // MOUNT1_YAW
    { "RC8_OPTION",          213},   // MOUNT1_PITCH
    { "MOT_PWM_MIN",         1100},
    { "MOT_PWM_MAX",         1900},
    { "PSC_JERK_Z",          50.0f},
    { "WPNAV_SPEED",         100.0f},
    { "PILOT_SPEED_UP",      100.0f},
    { "PSC_VELXY_P",         6.0f},
    { "EK3_SRC1_VELZ",       0},
#if AP_SUB_RC_ENABLED
    { "RC_PROTOCOLS",        0},
#endif
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIGATOR
    { "BATT_MONITOR",        4},
    { "BATT_CAPACITY",       0},
    { "LEAK1_PIN",           27},
    { "SCHED_LOOP_RATE",     200},
    { "SERVO13_FUNCTION",    181},   // k_lights1
    { "SERVO14_FUNCTION",    182},   // k_lights2
    { "SERVO16_FUNCTION",    7},     // k_mount_tilt
    { "SERVO16_REVERSED",    1},
#else
#if AP_BARO_PROBE_EXT_PARAMETER_ENABLED
    { "BARO_PROBE_EXT",      768},
#endif
    { "SERVO9_FUNCTION",     59},    // k_rcin9, lights 1
    { "SERVO10_FUNCTION",    7},     // k_mount_tilt
#endif
};
