/**
 * @file Parameters.h
 * @brief ArduCopter parameter system definition and configuration
 * 
 * @details This file defines the complete parameter system for ArduCopter, managing all
 *          runtime-configurable settings for multicopter vehicles. The parameter system
 *          uses the AP_Param library to provide persistent storage in EEPROM/Flash with
 *          MAVLink parameter protocol support for ground station configuration.
 * 
 *          The parameter system is organized into two main classes:
 *          - Parameters: Primary parameter block (keys 0-255)
 *          - ParametersG2: Secondary parameter block (key 256+) for overflow
 * 
 *          **Parameter Architecture:**
 *          
 *          Each parameter is stored with a unique key derived from its position in the
 *          parameter enumeration. Keys must remain stable across firmware versions to
 *          maintain EEPROM compatibility when users upgrade/downgrade firmware.
 * 
 *          **Parameter Groups:**
 *          - Flight Modes (k_param_flight_mode1-6): Configurable flight mode assignments
 *          - Navigation (k_param_rtl_*, k_param_wp_*): Waypoint and RTL behavior
 *          - Safety/Failsafe (k_param_fs_*): Failsafe triggers and actions
 *          - Sensor Configuration (k_param_compass, k_param_gps): Sensor settings
 *          - Control Tuning (attitude_control, pos_control): PID and control parameters
 *          - Hardware (k_param_motors, servo_channels): Motor and servo configuration
 *          - Telemetry (k_param_gcs*, k_param_log_*): Communication and logging
 * 
 *          **Parameter Storage Lifecycle:**
 *          1. Compile-time defaults defined in Parameters.cpp var_info tables
 *          2. Runtime defaults loaded from EEPROM at boot (if present)
 *          3. User modifications via MAVLink parameter protocol
 *          4. Persistence to EEPROM when parameter modified
 *          5. Factory reset available to restore compile-time defaults
 * 
 *          **Adding New Parameters:**
 *          To add a new parameter:
 *          1. Add enum entry in appropriate section (mind key collisions)
 *          2. Add AP_Int*/AP_Float member variable to Parameters or ParametersG2
 *          3. Add var_info entry in Parameters.cpp with metadata (@DisplayName, @Description, etc.)
 *          4. Consider EEPROM compatibility - deprecated parameters stay as comments
 * 
 *          **Parameter Naming Conventions:**
 *          - Prefix indicates subsystem (RTL_, FS_, PILOT_, etc.)
 *          - Suffix indicates type when relevant (_MIN, _MAX, _ENABLE, etc.)
 *          - Use consistent units (altitude in cm, speed in cm/s, angles in centidegrees)
 * 
 *          **Thread Safety:**
 *          Parameters are typically read at high frequency (400Hz main loop) but modified
 *          infrequently (user configuration changes). The AP_Param system handles concurrent
 *          access through atomic load/store operations for basic types.
 * 
 *          **Safety Considerations:**
 *          Many parameters directly affect flight safety. Invalid parameter values can cause
 *          crashes. The parameter system includes:
 *          - Range validation in var_info (min/max values)
 *          - Pre-arm checks for critical parameters (AP_Arming)
 *          - Failsafe behaviors triggered by parameter thresholds
 * 
 * @note Parameter keys are 9-bit values (0-511) managed by AP_Param. The first 256 keys
 *       are in Parameters class, keys 256+ are in ParametersG2 (accessed via k_param_g2).
 * 
 * @warning NEVER change parameter key assignments - this breaks EEPROM compatibility.
 *          Deprecated parameters must remain in enum as comments to reserve their keys.
 * 
 * @see Parameters.cpp for var_info tables with parameter metadata
 * @see libraries/AP_Param/AP_Param.h for parameter system implementation
 * @see Copter.h for parameter access via copter.g and copter.g2 singletons
 * 
 * Source: ArduCopter/Parameters.h
 */

#pragma once

#define AP_PARAM_VEHICLE_NAME copter

#include <AP_Common/AP_Common.h>
#include "RC_Channel_Copter.h"
#include <AP_Proximity/AP_Proximity.h>

#if MODE_FOLLOW_ENABLED
 # include <AP_Follow/AP_Follow.h>
#endif
#if WEATHERVANE_ENABLED
 #include <AC_AttitudeControl/AC_WeatherVane.h>
#endif

/**
 * @class Parameters
 * @brief Primary parameter storage class for ArduCopter configuration
 * 
 * @details This class defines the main parameter block for ArduCopter, containing all
 *          runtime-configurable settings for multicopter flight control. Parameters are
 *          stored persistently in EEPROM/Flash and accessed via the AP_Param system.
 * 
 *          **Parameter Organization:**
 *          The class contains approximately 100 direct parameter members organized by
 *          functional area, plus references to library parameter groups (GPS, compass,
 *          INS, battery monitor, etc.) which contain their own parameters.
 * 
 *          **Key Assignment:**
 *          Each parameter or parameter group has a unique key from the k_param_* enumeration.
 *          Keys 0-255 are available in this primary Parameters class. Additional parameters
 *          use the ParametersG2 class accessed via k_param_g2 (key 58).
 * 
 *          **Parameter Types:**
 *          - AP_Int8, AP_Int16, AP_Int32: Integer parameters (8/16/32 bit)
 *          - AP_Float: Floating-point parameters (32-bit float)
 *          - AP_Enum<T>: Enumerated parameters with type safety
 * 
 *          **Format Version:**
 *          The k_format_version constant (currently 120) tracks incompatible parameter
 *          layout changes. When the parameter structure changes incompatibly, incrementing
 *          this version triggers EEPROM reset, preventing corruption from misinterpreted
 *          parameter values.
 * 
 *          **Deprecated Parameters:**
 *          Many k_param_* entries are marked deprecated or with "remove" comments. These
 *          are obsolete parameters that must remain in the enumeration to preserve key
 *          assignments for active parameters. Removing them would cause key shifts that
 *          break EEPROM compatibility.
 * 
 *          **Usage Pattern:**
 *          Parameters are accessed via the global copter.g singleton:
 *          @code
 *          // Read parameter value
 *          int16_t rtl_alt = copter.g.rtl_altitude;
 *          
 *          // Modify parameter value (triggers EEPROM save)
 *          copter.g.rtl_altitude.set_and_save(400); // 400cm = 4m
 *          @endcode
 * 
 *          **Parameter Metadata:**
 *          Additional parameter information (display names, descriptions, units, ranges)
 *          is defined in the var_info table in Parameters.cpp. This metadata is used by
 *          ground control stations for user-friendly parameter editing.
 * 
 * @note This class uses default constructor to avoid initialization order issues.
 *       Parameters are loaded from EEPROM after construction by AP_Param::load_all().
 * 
 * @warning Modifying the enumeration order or removing entries breaks EEPROM compatibility.
 *          Always add new parameters at logical group boundaries, never remove old entries.
 * 
 * @see ParametersG2 for secondary parameter block (keys 256+)
 * @see Parameters.cpp var_info table for parameter metadata definitions
 * @see AP_Param::load_all() for parameter loading from EEPROM at startup
 * 
 * Source: ArduCopter/Parameters.h:18-486
 */
class Parameters {
public:
    /**
     * @brief Parameter layout format version number
     * 
     * @details This version number tracks incompatible changes to the parameter structure.
     *          When the parameter enumeration is modified in a way that would cause parameter
     *          key collisions or misinterpretation of stored values, this version must be
     *          incremented.
     * 
     *          At boot, if the stored format version in EEPROM doesn't match this value,
     *          ArduPilot will reset all parameters to factory defaults, preventing corruption
     *          from misinterpreted parameter data.
     * 
     *          **When to Increment:**
     *          - Changing parameter type (e.g., Int16 to Float)
     *          - Changing parameter units or scaling
     *          - Removing active (non-deprecated) parameters
     *          - Reordering parameter keys
     * 
     *          **When NOT to Increment:**
     *          - Adding new parameters at unused key locations
     *          - Modifying parameter metadata (DisplayName, Description, etc.)
     *          - Changing default values
     *          - Deprecating parameters (leaving enum entry as comment)
     * 
     * @note Current version: 120
     * @warning Incrementing this forces EEPROM reset on all vehicles - use with caution
     */
    static const uint16_t        k_format_version = 120;

    /**
     * @enum Parameter key enumeration
     * @brief Unique identifier for each parameter and parameter group
     * 
     * @details This enumeration assigns a unique 9-bit key (0-511) to each parameter or
     *          parameter group. AP_Param uses these keys to identify parameters in EEPROM
     *          storage, ensuring values are correctly loaded/saved across firmware versions.
     * 
     *          **Key Assignment Rules:**
     *          - Keys 0-255: Available in this Parameters class
     *          - Keys 256-511: Available in ParametersG2 class (accessed via k_param_g2)
     *          - Key 0: Always reserved for k_param_format_version
     *          - Unspecified keys: Auto-assigned as (previous_key + 1)
     *          - Explicit keys: Used to reserve ranges for related parameters
     * 
     *          **Parameter Groups:**
     *          The enumeration is organized into functional groups with reserved key ranges:
     *          - 0-9: Core system (format version, INS, EKF, etc.)
     *          - 10-19: Hardware (SITL, barometer, GPS, sensors)
     *          - 20-64: Vehicle configuration (modes, tuning, limits)
     *          - 65-89: Legacy/deprecated (Limits, Heli, Singlecopter)
     *          - 90-99: Motor and avoidance systems
     *          - 100-109: Navigation controllers (WP_Nav, Attitude, Position)
     *          - 110-139: Telemetry and GCS communication
     *          - 140-169: Sensor configuration (compass, rangefinder, cameras)
     *          - 170-199: RC input and failsafe configuration
     *          - 200-209: Flight mode assignments
     *          - 210-219: Waypoint and landing parameters
     *          - 220-256: PID controllers and advanced features
     * 
     *          **Deprecated Parameters:**
     *          Entries marked "deprecated", "unused", or "remove" must remain in the
     *          enumeration as comments to preserve key assignments. Removing them would
     *          cause subsequent keys to shift, breaking EEPROM compatibility.
     * 
     *          **Library Parameter Groups:**
     *          Many entries (k_param_gps, k_param_compass, k_param_ins, etc.) are parameter
     *          groups that reference library objects with their own sub-parameters. These
     *          library parameters are defined in their respective var_info tables.
     * 
     * @note Parameter keys must NEVER be reused after deprecation. Once assigned, a key
     *       remains reserved forever to prevent accidental misinterpretation of old EEPROM data.
     * 
     * @warning Changing key values breaks EEPROM parameter storage. Users would lose all
     *          parameter settings, potentially causing dangerous flight behavior with default
     *          values that haven't been tuned for their vehicle.
     * 
     * @see ParametersG2 for keys 256-511 via k_param_g2 indirection
     * @see Parameters.cpp var_info[] for parameter metadata and sub-parameter definitions
     */
    enum {
        /**
         * @name Core System Parameters (keys 0-9)
         * @brief Fundamental system configuration and sensor fusion
         * @{
         */
        
        /// Layout version number - always key zero for AP_Param system identification
        k_param_format_version = 0,
        k_param_software_type, // deprecated
        k_param_ins_old,                        // *** Deprecated, remove with next eeprom number change
        k_param_ins,                            // libraries/AP_InertialSensor variables
        k_param_NavEKF2_old, // deprecated - remove
        k_param_NavEKF2,
        k_param_g2, // 2nd block of parameters (ParametersG2 - keys 256+)
        k_param_NavEKF3, // Extended Kalman Filter version 3 (primary state estimator)
        k_param_can_mgr, // CAN bus manager for DroneCAN/UAVCAN peripherals
        k_param_osd, // On-screen display configuration
        
        /** @} */ // End of Core System Parameters group

        /**
         * @name Hardware and Simulation Parameters (keys 10-19)
         * @brief Hardware abstraction and simulation configuration
         * @{
         */
        
        // Simulation parameters for Software-In-The-Loop (SITL) testing
        k_param_sitl = 10,

        // barometer object (needed for SITL)
        k_param_barometer,

        // scheduler object (for debugging)
        k_param_scheduler,

        // relay object
        k_param_relay,

        // (old) EPM object
        k_param_epm_unused,

        // BoardConfig object
        k_param_BoardConfig,

        // GPS object
        k_param_gps,

        // Parachute object
        k_param_parachute,

        // Landing gear object
        k_param_landinggear,    // 18

        // Input Management object - pilot input processing and RC failsafe
        k_param_input_manager,  // 19
        
        /** @} */ // End of Hardware and Simulation Parameters group

        /**
         * @name Vehicle Configuration Parameters (keys 20-89)
         * @brief Flight modes, tuning, limits, and vehicle-specific settings
         * @details This section contains general vehicle configuration including deprecated
         *          parameters from legacy features (Limits library, Heli, Singlecopter).
         *          Many entries are marked deprecated but must remain to preserve key assignments.
         * @{
         */
        
        // Miscellaneous vehicle configuration
        //
        k_param_log_bitmask_old = 20,           // Deprecated
        k_param_log_last_filenumber,            // *** Deprecated - remove
                                                // with next eeprom number
                                                // change
        k_param_toy_yaw_rate,                   // deprecated - remove
        k_param_crosstrack_min_distance,    // deprecated - remove with next eeprom number change
        k_param_rssi_pin,                   // unused, replaced by rssi_ library parameters
        k_param_throttle_accel_enabled,     // deprecated - remove
        k_param_wp_yaw_behavior,
        k_param_acro_trainer,
        k_param_pilot_speed_up,         // renamed from k_param_pilot_velocity_z_max
        k_param_circle_rate,            // deprecated - remove
        k_param_rangefinder_gain,       // deprecated - remove
        k_param_ch8_option_old,         // deprecated
        k_param_arming_check_old,       // deprecated - remove
        k_param_sprayer,
        k_param_angle_max,
        k_param_gps_hdop_good,
        k_param_battery,
        k_param_fs_batt_mah,            // unused - moved to AP_BattMonitor
        k_param_angle_rate_max,         // remove
        k_param_rssi_range,             // unused, replaced by rssi_ library parameters
        k_param_rc_feel_rp,             // deprecated
        k_param_NavEKF,                 // deprecated - remove
        k_param_mission,                // mission library
        k_param_rc_13_old,
        k_param_rc_14_old,
        k_param_rally,
        k_param_poshold_brake_rate_degs,
        k_param_poshold_brake_angle_max,
        k_param_pilot_accel_z,
        k_param_serial0_baud,           // deprecated - remove
        k_param_serial1_baud,           // deprecated - remove
        k_param_serial2_baud,           // deprecated - remove
        k_param_land_repositioning,
        k_param_rangefinder, // rangefinder object
        k_param_fs_ekf_thresh,
        k_param_terrain,
        k_param_acro_rp_expo,           // deprecated - remove
        k_param_throttle_deadzone,
        k_param_optflow,
        k_param_dcmcheck_thresh,        // deprecated - remove
        k_param_log_bitmask,
        k_param_cli_enabled_old,        // deprecated - remove
        k_param_throttle_filt,
        k_param_throttle_behavior,
        k_param_pilot_takeoff_alt, // 64

        // 65: AP_Limits Library
        k_param_limits = 65,            // deprecated - remove
        k_param_gpslock_limit,          // deprecated - remove
        k_param_geofence_limit,         // deprecated - remove
        k_param_altitude_limit,         // deprecated - remove
        k_param_fence_old,              // only used for conversion
        k_param_gps_glitch,             // deprecated
        k_param_baro_glitch,            // 71 - deprecated

        // AP_ADSB Library
        k_param_adsb,                   // 72
        k_param_notify,                 // 73

        // 74: precision landing object
        k_param_precland = 74,

        //
        // 75: Singlecopter, CoaxCopter
        //
        k_param_single_servo_1 = 75,    // remove
        k_param_single_servo_2,         // remove
        k_param_single_servo_3,         // remove
        k_param_single_servo_4,         // 78 - remove

        //
        // 80: Heli
        //
        k_param_heli_servo_1 = 80,  // remove
        k_param_heli_servo_2,       // remove
        k_param_heli_servo_3,       // remove
        k_param_heli_servo_4,       // remove
        k_param_heli_pitch_ff,      // remove
        k_param_heli_roll_ff,       // remove
        k_param_heli_yaw_ff,        // remove
        k_param_heli_stab_col_min,  // remove
        k_param_heli_stab_col_max,  // remove
        k_param_heli_servo_rsc,     // 89 = full! - remove
        
        /** @} */ // End of Vehicle Configuration Parameters group

        /**
         * @name Motor and Safety Systems (keys 90-99)
         * @brief Motor control, disarm, failsafe, and collision avoidance
         * @{
         */
        
        // Motor and avoidance configuration
        k_param_motors = 90, // Motor mixing and output configuration
        k_param_disarm_delay,
        k_param_fs_crash_check,
        k_param_throw_motor_start,
        k_param_rtl_alt_type,
        k_param_avoid,
        k_param_avoidance_adsb,

        // Received Signal Strength Indicator configuration
        k_param_rssi = 97,
        
        /** @} */ // End of Motor and Safety Systems group
        
        /**
         * @name Navigation Controllers (keys 100-109)
         * @brief Position, attitude, and waypoint navigation controllers
         * @details Controller objects managing vehicle position/attitude with PID loops and
         *          trajectory generation. These library parameters contain extensive sub-parameters
         *          for PID tuning, limits, and navigation behavior.
         * @{
         */
        
        // Navigation controller configuration
        k_param_inertial_nav = 100, // deprecated - functionality merged into EKF
        k_param_wp_nav,
        k_param_attitude_control,
        k_param_pos_control,
        k_param_circle_nav,
        k_param_loiter_nav,     // 105 - Loiter mode position holding
        k_param_custom_control, // Custom controller interface for external control
        
        /** @} */ // End of Navigation Controllers group

        /**
         * @name Telemetry and GCS Parameters (keys 110-139)
         * @brief Ground Control Station communication and telemetry configuration
         * @details Serial port assignments, MAVLink system IDs, telemetry rates, and RC channel
         *          auxiliary function mappings. Many GCS parameters moved to serial manager.
         * @{
         */
        
        // Telemetry control
        k_param_gcs0_unused = 110,   // unused in ArduPilot-4.7 (moved to serial manager)
        k_param_gcs1_unused,         // unused in ArduPilot-4.7
        k_param_sysid_this_mav_old,
        k_param_sysid_my_gcs_old,
        k_param_serial1_baud_old, // deprecated
        k_param_telem_delay_old,     // used for conversion in ArduPilot-4.7
        k_param_gcs2_unused,         // unused in ArduPilot-4.7
        k_param_serial2_baud_old, // deprecated
        k_param_serial2_protocol, // deprecated
        k_param_serial_manager_old,
        k_param_ch9_option_old,
        k_param_ch10_option_old,
        k_param_ch11_option_old,
        k_param_ch12_option_old,
        k_param_takeoff_trigger_dz_old,
        k_param_gcs3_unused,         // unused in ArduPilot-4.7
        k_param_gcs_pid_mask,    // 126
        k_param_gcs4_unused,         // unused in ArduPilot-4.7
        k_param_gcs5_unused,         // unused in ArduPilot-4.7
        k_param_gcs6_unused,         // unused in ArduPilot-4.7

        //
        // 135 : reserved for Solo until features merged with master
        //
        k_param_rtl_speed_cms = 135,
        k_param_fs_batt_curr_rtl,
        k_param_rtl_cone_slope, // 137 - RTL descent cone angle
        
        /** @} */ // End of Telemetry and GCS Parameters group

        /**
         * @name Sensor Configuration Parameters (keys 140-169)
         * @brief Sensor hardware configuration and calibration
         * @details IMU, compass, rangefinder, optical flow, battery monitoring, and camera
         *          configuration. Includes sensor enable/disable, calibration values, and
         *          orientation settings.
         * @{
         */
        
        // Sensor parameters
        k_param_imu = 140, // deprecated - can be deleted (moved to AP_InertialSensor)
        k_param_battery_monitoring = 141,   // deprecated - can be deleted
        k_param_volt_div_ratio, // deprecated - can be deleted
        k_param_curr_amp_per_volt,  // deprecated - can be deleted
        k_param_input_voltage,  // deprecated - can be deleted
        k_param_pack_capacity,  // deprecated - can be deleted
        k_param_compass_enabled_deprecated,
        k_param_compass,
        k_param_rangefinder_enabled_old, // deprecated
        k_param_frame_type,
        k_param_optflow_enabled,    // deprecated
        k_param_fs_batt_voltage,    // unused - moved to AP_BattMonitor
        k_param_ch7_option_old,
        k_param_auto_slew_rate,     // deprecated - can be deleted
        k_param_rangefinder_type_old,     // deprecated
        k_param_super_simple = 155,
        k_param_axis_enabled = 157, // deprecated - remove with next eeprom number change
        k_param_copter_leds_mode,   // deprecated - remove with next eeprom number change
        k_param_ahrs, // AHRS group // 159

        //
        // 160: Navigation parameters
        //
        k_param_rtl_altitude = 160,
        k_param_crosstrack_gain,    // deprecated - remove with next eeprom number change
        k_param_rtl_loiter_time,
        k_param_rtl_alt_final,
        k_param_tilt_comp, // 164 deprecated - remove with next eeprom number change


        //
        // Camera and mount parameters
        //
        k_param_camera = 165,
        k_param_camera_mount,
        k_param_camera_mount2,      // deprecated

        //
        /**
         * @name Battery Monitoring Parameters (keys 168-169)
         * @brief Legacy battery monitoring (moved to AP_BattMonitor library)
         * @deprecated These parameters moved to AP_BattMonitor - kept for key preservation
         * @{
         */
        k_param_battery_volt_pin = 168, // deprecated - can be deleted
        k_param_battery_curr_pin,   // 169 deprecated - can be deleted
        /** @} */
        
        /** @} */ // End of Sensor Configuration Parameters group

        /**
         * @name RC Input and Failsafe Parameters (keys 170-199)
         * @brief Radio control input configuration and failsafe behavior
         * @details RC channel mapping, calibration (min/max/trim), failsafe thresholds,
         *          and failsafe actions. Many RC parameters moved to RC_Channels library.
         * @{
         */
        
        // Radio settings
        k_param_rc_1_old = 170, // deprecated - moved to RC_Channels
        k_param_rc_2_old,
        k_param_rc_3_old,
        k_param_rc_4_old,
        k_param_rc_5_old,
        k_param_rc_6_old,
        k_param_rc_7_old,
        k_param_rc_8_old,
        k_param_rc_10_old,
        k_param_rc_11_old,
        k_param_throttle_min,           // remove
        k_param_throttle_max,           // remove
        k_param_failsafe_throttle,
        k_param_throttle_fs_action,     // remove
        k_param_failsafe_throttle_value,
        k_param_throttle_trim,          // remove
        k_param_esc_calibrate,
        k_param_rc_tuning_param,
        k_param_rc_tuning_param_high_old,   // unused
        k_param_rc_tuning_param_low_old,    // unused
        k_param_rc_speed = 192,
        k_param_failsafe_battery_enabled, // unused - moved to AP_BattMonitor
        k_param_throttle_mid,           // remove
        k_param_failsafe_gps_enabled,   // remove
        k_param_rc_9_old,
        k_param_rc_12_old,
        k_param_failsafe_gcs, // Ground Control Station failsafe enable/disable
        k_param_rcmap, // 199 - RC channel mapping (AETR configuration)
        
        /** @} */ // End of RC Input and Failsafe Parameters group

        /**
         * @name Flight Mode Parameters (keys 200-209)
         * @brief Flight mode slot assignments and configuration
         * @details Six flight mode slots (typically selected via RC switch) plus simple mode
         *          configuration. Flight modes define vehicle behavior (Stabilize, Loiter, Auto,
         *          RTL, etc.). Initial mode sets power-on default.
         * @{
         */
        
        // Flight mode assignments
        k_param_flight_mode1 = 200, // Flight mode for position 1 (typically RC switch low)
        k_param_flight_mode2,
        k_param_flight_mode3,
        k_param_flight_mode4,
        k_param_flight_mode5,
        k_param_flight_mode6,
        k_param_simple_modes, // Bitmask of flight modes using simple/super-simple
        k_param_flight_mode_chan, // RC channel for flight mode selection
        k_param_initial_mode, // Flight mode to use at startup
        
        /** @} */ // End of Flight Mode Parameters group

        /**
         * @name Waypoint and Landing Parameters (keys 210-219)
         * @brief Autonomous navigation and landing configuration
         * @details Waypoint navigation speeds, circle radius, landing speeds, and altitude
         *          control for autonomous missions. Many parameters deprecated - functionality
         *          moved to WP_Nav and Mission libraries.
         * @{
         */
        
        // Waypoint data
        k_param_waypoint_mode = 210, // remove - deprecated
        k_param_command_total,       // remove
        k_param_command_index,       // remove
        k_param_command_nav_index,   // remove
        k_param_waypoint_radius,     // remove
        k_param_circle_radius,       // remove
        k_param_waypoint_speed_max,  // remove
        k_param_land_speed, // Normal landing descent speed (cm/s)
        k_param_auto_velocity_z_min, // remove - deprecated
        k_param_auto_velocity_z_max, // remove - 219 - deprecated
        k_param_land_speed_high, // Fast landing descent speed for high altitude (cm/s)
        
        /** @} */ // End of Waypoint and Landing Parameters group

        /**
         * @name PID Controllers and Advanced Features (keys 220-256)
         * @brief PID tuning and advanced flight features
         * @details Many PID parameters deprecated - functionality moved to AC_AttitudeControl,
         *          AC_PosControl libraries. Modern parameters include arming checks, EKF failsafe,
         *          autotune, and logging configuration.
         * @{
         */
        
        // PID Controllers (mostly deprecated - moved to controller libraries)
        k_param_acro_rp_p = 221,    // remove - deprecated
        k_param_axis_lock_p,        // remove
        k_param_pid_rate_roll,      // remove
        k_param_pid_rate_pitch,     // remove
        k_param_pid_rate_yaw,       // remove
        k_param_p_stabilize_roll,   // remove
        k_param_p_stabilize_pitch,  // remove
        k_param_p_stabilize_yaw,    // remove
        k_param_p_pos_xy,           // remove
        k_param_p_loiter_lon,       // remove
        k_param_pid_loiter_rate_lat,    // remove
        k_param_pid_loiter_rate_lon,    // remove
        k_param_pid_nav_lat,        // remove
        k_param_pid_nav_lon,        // remove
        k_param_p_alt_hold,             // remove
        k_param_p_vel_z,                // remove
        k_param_pid_optflow_roll,       // remove
        k_param_pid_optflow_pitch,      // remove
        k_param_acro_balance_roll_old,  // remove
        k_param_acro_balance_pitch_old, // remove
        k_param_pid_accel_z,            // remove
        k_param_acro_balance_roll,
        k_param_acro_balance_pitch,
        k_param_acro_yaw_p,             // remove
        k_param_autotune_axis_bitmask, // remove
        k_param_autotune_aggressiveness, // remove
        k_param_pi_vel_xy,              // remove
        k_param_fs_ekf_action,
        k_param_rtl_climb_min,
        k_param_rpm_sensor,
        k_param_autotune_min_d, // remove
        k_param_arming, // 252  - AP_Arming
        k_param_logger = 253, // 253 - Logging Group

        // 254,255: reserved

        k_param_vehicle = 257, // vehicle common block of parameters
        k_param_throw_altitude_min,
        k_param_throw_altitude_max,
        k_param__gcs,
        k_param_throw_altitude_descend,
        k_param_throw_altitude_ascend, // Throw mode ascend altitude target (meters)
        
        /** @} */ // End of PID Controllers and Advanced Features group

        // The k_param_* enumeration space is 9-bits (0-511)
        // Keys 256-511 accessed via k_param_g2 indirection (ParametersG2 class)
        // Key 511: reserved for future use
    };

    /**
     * @name Parameter Member Variables
     * @brief Actual parameter storage with type-safe AP_Param wrappers
     * @details Each member variable below corresponds to an enum entry above and provides
     *          type-safe storage with automatic EEPROM persistence via the AP_Param system.
     * 
     *          Variable naming convention:
     *          - Uses full descriptive names (not abbreviated k_param_ enum names)
     *          - Grouped by functional area matching enum organization
     *          - Conditional compilation (#if) for optional features
     * 
     *          Parameter types:
     *          - AP_Int8: 8-bit integers (-128 to 127) for flags and small values
     *          - AP_Int16: 16-bit integers (-32768 to 32767) for speeds, altitudes
     *          - AP_Int32: 32-bit integers for large values (bitmasks, milliseconds)
     *          - AP_Float: 32-bit floating point for precise values (rates, ratios)
     *          - AP_Enum<T>: Type-safe enumerated values
     * 
     * @note Parameters are loaded from EEPROM at startup by AP_Param::load_all().
     *       Accessing a parameter reads its current value; modifying triggers EEPROM save.
     * 
     * @see Parameters.cpp var_info table for parameter metadata (units, ranges, defaults)
     * @{
     */
    
    /// Format version for EEPROM layout compatibility checking
    AP_Int16        format_version;

    // Throttle and pilot control parameters
    AP_Float        throttle_filt;          ///< Throttle filter cutoff frequency (Hz) for smoothing pilot input
    AP_Int16        throttle_behavior;      ///< Throttle stick behavior (0=feedback from mid, 1=no deadzone)
    AP_Float        pilot_takeoff_alt;      ///< Pilot-initiated takeoff altitude target (cm)

#if MODE_RTL_ENABLED
    /**
     * @name RTL (Return To Launch) Mode Parameters
     * @brief Configuration for automatic return to launch behavior
     * @details RTL mode navigates vehicle back to launch point (or rally point) and lands.
     *          These parameters control climb behavior, navigation speed, final altitude,
     *          and loiter time before landing.
     * @{
     */
    AP_Int32        rtl_altitude;           ///< RTL altitude (cm) - climb to this altitude before returning
    AP_Int16        rtl_speed_cms;              ///< RTL horizontal speed (cm/s) during return flight
    AP_Float        rtl_cone_slope;             ///< RTL cone slope for descent path (0=no cone, 1=45deg)
    AP_Int16        rtl_alt_final;              ///< RTL final altitude (cm) before landing begins
    AP_Int16        rtl_climb_min;              ///< RTL minimum climb (cm) before navigating home
    AP_Int32        rtl_loiter_time;            ///< RTL loiter time (ms) at final altitude before landing
    AP_Enum<ModeRTL::RTLAltType> rtl_alt_type; ///< RTL altitude type (relative/terrain/current)
    /** @} */ // End of RTL Mode Parameters
#endif

    /**
     * @name Failsafe Parameters
     * @brief Safety triggers and actions for loss of control or critical failures
     * @{
     */
    AP_Int8         failsafe_gcs;               ///< Ground station failsafe behavior (0=disabled, 1=enabled)
    AP_Int16        gps_hdop_good;              ///< GPS Hdop threshold - values at/below indicate good position
    /** @} */ // End of Failsafe Parameters

    /**
     * @name Flight Mode Behavior Parameters
     * @brief Options modifying flight mode behavior
     * @{
     */
    AP_Int8         super_simple;               ///< Super Simple mode bitmask (bit per flight mode)

    AP_Int8         wp_yaw_behavior;            ///< Yaw behavior during waypoint missions (0=never change, 1=face next WP, etc.)
    /** @} */ // End of Flight Mode Behavior Parameters

#if MODE_POSHOLD_ENABLED
    /**
     * @name PosHold Mode Parameters
     * @brief Position hold mode braking behavior configuration
     * @{
     */
    AP_Int16        poshold_brake_rate_degs;    ///< PosHold rotation rate during braking (deg/s)
    AP_Int16        poshold_brake_angle_max;    ///< PosHold max lean angle during braking (centidegrees)
    /** @} */ // End of PosHold Mode Parameters
#endif

    /**
     * @name Landing and Vertical Speed Parameters
     * @brief Altitude change rates and landing descent speeds
     * @{
     */
    AP_Int16        land_speed;                 ///< Normal landing descent speed (cm/s)
    AP_Int16        land_speed_high;            ///< High altitude landing descent speed (cm/s)
    AP_Int16        pilot_speed_up;             ///< Maximum pilot-commanded ascent rate (cm/s)
    AP_Int16        pilot_accel_z;              ///< Maximum pilot-commanded vertical acceleration (cm/s/s)
    /** @} */ // End of Landing and Vertical Speed Parameters

    /**
     * @name Throttle Failsafe Parameters
     * @brief RC throttle failsafe detection and response
     * @{
     */
    AP_Int8         failsafe_throttle;          ///< Throttle failsafe action (0=disabled, 1=Land, 2=RTL, etc.)
    AP_Int16        failsafe_throttle_value;    ///< Throttle PWM value indicating failsafe (typically <975)
    AP_Int16        throttle_deadzone;          ///< Throttle stick deadzone (PWM) around mid-stick
    /** @} */ // End of Throttle Failsafe Parameters

    /**
     * @name Flight Mode Configuration
     * @brief Flight mode slot assignments for RC channel selection
     * @details Six configurable flight mode slots selected by RC switch position.
     *          Each slot contains a flight mode number (0=Stabilize, 1=Acro, 2=AltHold, etc.)
     * @{
     */
    AP_Int8         flight_mode1;               ///< Flight mode for switch position 1
    AP_Int8         flight_mode2;               ///< Flight mode for switch position 2
    AP_Int8         flight_mode3;               ///< Flight mode for switch position 3
    AP_Int8         flight_mode4;               ///< Flight mode for switch position 4
    AP_Int8         flight_mode5;               ///< Flight mode for switch position 5
    AP_Int8         flight_mode6;               ///< Flight mode for switch position 6
    AP_Int8         simple_modes;               ///< Bitmask: flight modes using simple/super-simple
    AP_Int8         flight_mode_chan;           ///< RC channel for flight mode selection (typically channel 5)
    AP_Int8         initial_mode;               ///< Flight mode to use at power-on
    /** @} */ // End of Flight Mode Configuration

    /**
     * @name Logging and Diagnostics Parameters
     * @brief Data logging, calibration, and debugging configuration
     * @{
     */
    AP_Int32        log_bitmask;                ///< Bitmask of log message types to record
    AP_Int8         esc_calibrate;              ///< ESC calibration mode enable (1=enter calibration)
    AP_Int8         rc_tuning_param;            ///< RC tuning parameter selector (deprecated)
    AP_Int8         frame_type;                 ///< Frame type (X, +, V, H, etc.) for motor mixing
    AP_Int8         disarm_delay;               ///< Delay (seconds) before automatic disarm after landing
    /** @} */ // End of Logging and Diagnostics Parameters

    /**
     * @name EKF and Safety Monitoring Parameters
     * @brief EKF health monitoring, crash detection, and failsafe triggers
     * @{
     */
    AP_Int8         land_repositioning;         ///< Enable precision landing repositioning
    AP_Int8         fs_ekf_action;              ///< EKF failsafe action (1=Land, 2=AltHold, 3=Land even in manual modes)
    AP_Int8         fs_crash_check;             ///< Crash detection enable/disable
    AP_Float        fs_ekf_thresh;              ///< EKF variance threshold for failsafe trigger
    AP_Int16        gcs_pid_mask;               ///< Bitmask of PIDs to stream to ground station
    /** @} */ // End of EKF and Safety Monitoring Parameters

#if MODE_THROW_ENABLED
    /**
     * @name Throw Mode Parameters
     * @brief Configuration for throw-to-start flight mode
     * @details Throw mode allows hand-launching by detecting freefall and automatically
     *          starting motors. These parameters control detection thresholds and post-throw behavior.
     * @{
     */
    AP_Enum<ModeThrow::PreThrowMotorState> throw_motor_start; ///< Motor start behavior before throw
    AP_Int16         throw_altitude_min;    ///< Minimum altitude (m) for throw detection
    AP_Int16         throw_altitude_max;    ///< Maximum altitude (m) for throw detection
    AP_Float         throw_altitude_descend; ///< Target descent (m) during drop (positive value)
    AP_Float         throw_altitude_ascend; ///< Target ascent (m) during throw up (positive value)
    /** @} */ // End of Throw Mode Parameters
#endif

    /**
     * @name RC Input Parameters
     * @brief RC receiver and channel configuration
     * @{
     */
    AP_Int16        rc_speed;                   ///< RC output update rate (Hz) for fast channels
    /** @} */ // End of RC Input Parameters

#if MODE_ACRO_ENABLED || MODE_SPORT_ENABLED
    /**
     * @name Acro Mode Parameters
     * @brief Rate mode control configuration for acrobatic flight
     * @details Acro/Sport modes provide direct rate control for aerobatic maneuvers.
     *          Balance parameters control automatic leveling when sticks centered.
     * @{
     */
    AP_Float        acro_balance_roll;          ///< Acro roll balance (0=no leveling, 1=full leveling)
    AP_Float        acro_balance_pitch;         ///< Acro pitch balance (0=no leveling, 1=full leveling)
#endif

#if MODE_ACRO_ENABLED
    AP_Int8         acro_trainer;               ///< Acro trainer mode (0=disabled, 1=leveling, 2=leveling+limit)
    /** @} */ // End of Acro Mode Parameters
#endif

    /** @} */ // End of Parameter Member Variables

    /**
     * @brief Default constructor for Parameters class
     * 
     * @details Constructor intentionally left empty. Parameter initialization occurs through
     *          the AP_Param system which loads values from EEPROM at startup via
     *          AP_Param::load_all(), using default values defined in var_info tables if
     *          no stored values exist.
     * 
     * @note Keep parameter declarations above in same order as var_info table for clarity
     * @see Parameters.cpp var_info table for parameter metadata and default values
     */
    Parameters()
    {
    }
};

/**
 * @class ParametersG2
 * @brief Secondary parameter block for ArduCopter (keys 256-511)
 * 
 * @details This class provides additional parameter storage beyond the 256-key limit of the
 *          primary Parameters class. ParametersG2 is accessed via the k_param_g2 key in the
 *          main parameter enumeration and provides parameter keys 256-511.
 * 
 *          **Why ParametersG2 Exists:**
 *          The AP_Param system uses 9-bit keys (0-511) for parameter identification. The
 *          original Parameters class can only directly hold 256 parameters (keys 0-255).
 *          As ArduCopter added features, more parameters were needed. ParametersG2 provides
 *          an additional 256 parameter slots accessed indirectly through k_param_g2.
 * 
 *          **Parameter Organization:**
 *          ParametersG2 contains:
 *          - Recently added features (mode options, advanced failsafes, command models)
 *          - Library objects with sub-parameters (RC_Channels, SRV_Channels, SmartRTL, etc.)
 *          - Feature-specific parameters (weather vane, object avoidance, autorotation)
 *          - Mode-specific options (AUTO_OPTIONS, GUIDED_OPTIONS, RTL_OPTIONS, etc.)
 * 
 *          **Conditional Compilation:**
 *          Many ParametersG2 members use #if directives to conditionally include features
 *          based on vehicle configuration (frame type, enabled modes, hardware capabilities).
 *          This reduces memory usage on resource-constrained boards by excluding unused features.
 * 
 *          **Pointer Members:**
 *          Some members are void pointers (mode_flowhold_ptr, autotune_ptr, etc.) rather
 *          than direct objects. This allows the var_info table to reference modes without
 *          including their full definitions in this header, reducing compilation dependencies.
 * 
 *          **Library Parameter Groups:**
 *          Several members are library objects that contain their own sub-parameters:
 *          - rc_channels: RC_Channels_Copter - RC input channel configuration
 *          - servo_channels: SRV_Channels - Servo output channel configuration
 *          - smart_rtl: AP_SmartRTL - Smart Return To Launch path storage
 *          - beacon: AP_Beacon - Non-GPS positioning system
 *          - proximity: AP_Proximity - Object avoidance sensors
 *          - weathervane: AC_WeatherVane - Automatic heading into wind
 *          - oa: AP_OAPathPlanner - Object avoidance path planning
 * 
 *          **Usage Pattern:**
 *          ParametersG2 accessed via copter.g2 singleton:
 *          @code
 *          // Access G2 parameter
 *          float timeout = copter.g2.fs_gcs_timeout;
 *          
 *          // Modify G2 parameter (triggers EEPROM save)
 *          copter.g2.fs_gcs_timeout.set_and_save(30.0f);
 *          
 *          // Access library sub-parameter
 *          copter.g2.rc_channels.channel(2)->set_range(1000);
 *          @endcode
 * 
 *          **Var_Info Tables:**
 *          ParametersG2 has TWO var_info tables:
 *          - var_info: Primary table with most G2 parameters
 *          - var_info2: Overflow table for additional parameters
 *          This allows organizing related parameters even when G2 capacity is exceeded.
 * 
 * @note The unused_integer member exists solely to enable constructor syntax that works
 *       cleanly with multiple conditional compilation blocks.
 * 
 * @warning Like Parameters class, ParametersG2 keys must never change. Deprecated members
 *          must remain as comments to preserve key assignments.
 * 
 * @see Parameters class for primary parameter block (keys 0-255)
 * @see Parameters.cpp var_info_g2[] and var_info2_g2[] for G2 parameter metadata
 * @see Copter.h copter.g2 for global access to ParametersG2 instance
 * 
 * Source: ArduCopter/Parameters.h:491-693
 */
class ParametersG2 {
public:
    /**
     * @brief ParametersG2 constructor
     * 
     * @details Initializes the secondary parameter block. Constructor implementation
     *          in Parameters.cpp handles var_info table registration with AP_Param.
     * 
     * @see Parameters.cpp ParametersG2::ParametersG2() constructor implementation
     */
    ParametersG2(void);

    /**
     * @name Parameter Metadata Tables
     * @brief AP_Param metadata for ParametersG2 members
     * @details These static tables define parameter metadata (names, types, defaults, units,
     *          ranges) for ground station display and parameter validation. Two tables exist
     *          due to ParametersG2 containing more parameters than fit in a single var_info.
     * @{
     */
    static const struct AP_Param::GroupInfo var_info[];  ///< Primary G2 parameter metadata
    static const struct AP_Param::GroupInfo var_info2[]; ///< Secondary G2 parameter metadata (overflow)
    /** @} */

    /**
     * @name Navigation Parameters
     * @brief Waypoint navigation and takeoff configuration
     * @{
     */
    AP_Float wp_navalt_min;             ///< Minimum altitude (cm) for navigation control in takeoff
    /** @} */

    /**
     * @brief Constructor syntax helper
     * 
     * @details This member exists solely to enable clean constructor syntax in Parameters.cpp
     *          when many members are conditionally compiled with #if directives. It has no
     *          functional purpose and is not a parameter.
     */
    uint8_t unused_integer;

    /**
     * @name Button Input
     * @brief Physical button support for mode changes
     * @{
     */
#if HAL_BUTTON_ENABLED
    AP_Button *button_ptr;              ///< Pointer to button handler object
#endif
    /** @} */

#if MODE_THROW_ENABLED
    /**
     * @name Throw Mode G2 Parameters
     * @brief Additional throw mode configuration in G2 block
     * @{
     */
    AP_Int8 throw_nextmode;             ///< Flight mode to switch to after throw completes
    AP_Enum<ModeThrow::ThrowType> throw_type; ///< Throw type (upward/drop) detection
    /** @} */
#endif

    /**
     * @name Ground Effect Compensation
     * @brief Automatic thrust adjustment near ground
     * @{
     */
    AP_Int8 gndeffect_comp_enabled;     ///< Ground effect compensation enable (0=disabled, 1=enabled)
    /** @} */

#if AP_TEMPCALIBRATION_ENABLED
    /**
     * @name Temperature Calibration
     * @brief IMU temperature calibration for thermal drift compensation
     * @{
     */
    AP_TempCalibration temp_calibration; ///< Temperature calibration library
    /** @} */
#endif

#if AP_BEACON_ENABLED
    /**
     * @name Beacon Positioning
     * @brief Non-GPS positioning using beacon systems (Marvelmind, Pozyx, etc.)
     * @{
     */
    AP_Beacon beacon;                   ///< Beacon positioning library
    /** @} */
#endif

#if HAL_PROXIMITY_ENABLED
    /**
     * @name Proximity Sensors
     * @brief Object detection and collision avoidance sensors
     * @{
     */
    AP_Proximity proximity;             ///< Proximity sensor library (lidar, sonar arrays)
    /** @} */
#endif

#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
    /**
     * @name Advanced Failsafe
     * @brief Hardware-based termination for critical failures
     * @{
     */
    AP_AdvancedFailsafe_Copter afs;     ///< Advanced failsafe with hardware termination
    /** @} */
#endif

    /**
     * @name Developer Options
     * @brief Bitmask of developer/debug features
     * @{
     */
    AP_Int32 dev_options;               ///< Developer options bitmask (debugging features)
    /** @} */

#if MODE_ACRO_ENABLED
    /**
     * @name Acro Mode G2 Parameters
     * @brief Additional acro mode configuration
     * @{
     */
    AP_Float acro_thr_mid;              ///< Acro mode throttle mid-point for altitude hold assist
    /** @} */
#endif

    /**
     * @name Frame Configuration
     * @brief Frame type and motor configuration
     * @{
     */
    AP_Int8 frame_class;                ///< Frame class (quad/hexa/octa/Y6/etc.)
    /** @} */

    /**
     * @name RC and Servo Channels
     * @brief RC input and servo output channel management
     * @details These library objects handle RC receiver input processing and servo/motor
     *          output channel management with extensive sub-parameters for each channel.
     * @{
     */
    RC_Channels_Copter rc_channels;     ///< RC input channels (library with per-channel parameters)
    SRV_Channels servo_channels;        ///< Servo output channels (library with per-channel parameters)
    /** @} */

#if MODE_SMARTRTL_ENABLED
    /**
     * @name Smart RTL
     * @brief Intelligent return path using recorded waypoints
     * @{
     */
    AP_SmartRTL smart_rtl;              ///< Smart RTL path recording and management
    /** @} */
#endif

#if AP_WINCH_ENABLED
    /**
     * @name Winch Control
     * @brief Winch/hoist control for cargo delivery
     * @{
     */
    AP_Winch winch;                     ///< Winch control library
    /** @} */
#endif

    /**
     * @name Pilot Vertical Speed
     * @brief Additional vertical speed control parameters
     * @{
     */
    AP_Int16    pilot_speed_dn;         ///< Maximum pilot-commanded descent rate (cm/s, positive value)
    /** @} */

    /**
     * @name Landing Parameters
     * @brief Landing altitude thresholds and behavior
     * @{
     */
    AP_Int16 land_alt_low;              ///< Low altitude threshold (cm) for final landing stage
    /** @} */

#if TOY_MODE_ENABLED
    /**
     * @name Toy Mode
     * @brief Simplified flight mode for consumer applications
     * @{
     */
    ToyMode toy_mode;                   ///< Toy mode configuration
    /** @} */
#endif

#if MODE_FLOWHOLD_ENABLED
    /**
     * @name FlowHold Mode
     * @brief Optical flow-based position hold mode
     * @{
     */
    void *mode_flowhold_ptr;            ///< Pointer to FlowHold mode object (avoids header dependency)
    /** @} */
#endif

#if MODE_FOLLOW_ENABLED
    /**
     * @name Follow Mode
     * @brief Follow another vehicle using position telemetry
     * @{
     */
    AP_Follow follow;                   ///< Follow mode library
    /** @} */
#endif

#if USER_PARAMS_ENABLED
    /**
     * @name User Parameters
     * @brief Custom user-defined parameters
     * @{
     */
    UserParameters user_parameters;     ///< User-defined custom parameters
    /** @} */
#endif

#if AUTOTUNE_ENABLED
    /**
     * @name AutoTune
     * @brief Automatic PID tuning mode
     * @{
     */
    void *autotune_ptr;                 ///< Pointer to AutoTune object (avoids header dependency)
    /** @} */
#endif

    /**
     * @name RC Tuning Range
     * @brief RC channel tuning knob min/max values
     * @{
     */
    AP_Float tuning_min;                ///< Minimum value for RC tuning knob
    AP_Float tuning_max;                ///< Maximum value for RC tuning knob
    /** @} */

#if AP_OAPATHPLANNER_ENABLED
    /**
     * @name Object Avoidance Path Planner
     * @brief Dynamic path planning to avoid obstacles
     * @{
     */
    AP_OAPathPlanner oa;                ///< Object avoidance path planner library
    /** @} */
#endif

#if MODE_SYSTEMID_ENABLED
    /**
     * @name SystemID Mode
     * @brief System identification mode for frequency response analysis
     * @{
     */
    void *mode_systemid_ptr;            ///< Pointer to SystemID mode object (avoids header dependency)
    /** @} */
#endif

    /**
     * @name Vibration Failsafe
     * @brief IMU vibration level failsafe trigger
     * @{
     */
    AP_Int8 fs_vibe_enabled;            ///< Enable vibration failsafe: 0=Disabled, 1=Enabled (triggers when IMU vibration exceeds thresholds)
    /** @} */

    /**
     * @name Failsafe Options
     * @brief Bitmask of failsafe behavior options
     * @details Bit flags controlling failsafe behavior:
     *          - Bit 0: Continue mission on RC failsafe in AUTO mode
     *          - Bit 1: Continue mission on GCS failsafe in AUTO mode
     *          - Bit 2: Continue landing on RC failsafe in LAND mode
     *          - Bit 3: Continue landing on GCS failsafe in LAND mode
     *          - Additional bits for future failsafe options
     * @{
     */
    AP_Int32 fs_options;                ///< Failsafe options bitmask (parameter FS_OPTIONS)
    /** @} */

#if MODE_AUTOROTATE_ENABLED
    /**
     * @name Autorotation Mode
     * @brief Autonomous autorotation for helicopter emergency landing
     * @{
     */
    AC_Autorotation arot;               ///< Autonomous autorotation controller for helicopters
    /** @} */
#endif

#if MODE_ZIGZAG_ENABLED
    /**
     * @name ZigZag Mode
     * @brief Automated zigzag pattern for area coverage
     * @{
     */
    void *mode_zigzag_ptr;              ///< Pointer to ZigZag mode object (avoids header dependency)
    /** @} */
#endif

    /**
     * @name Command Models
     * @brief Input shaping and rate control for pilot commands
     * @details Command models provide configurable input response characteristics
     *          for pilot stick inputs, including expo curves, rate limits, and
     *          filtering for smooth and predictable control feel.
     * @{
     */
#if MODE_ACRO_ENABLED || MODE_SPORT_ENABLED
    AC_CommandModel command_model_acro_rp; ///< Acro/Sport mode roll/pitch command model
#endif

#if MODE_ACRO_ENABLED || MODE_DRIFT_ENABLED
    AC_CommandModel command_model_acro_y;  ///< Acro/Drift mode yaw command model
#endif

    AC_CommandModel command_model_pilot_y; ///< Pilot yaw input command model for stabilized modes
    /** @} */

    /**
     * @name Mode Options
     * @brief Bitmask options for flight mode behaviors
     * @details Each mode can have multiple configurable behaviors controlled by
     *          option bitmasks. These allow customizing mode behavior without
     *          adding separate parameters for each option.
     * @{
     */
#if MODE_ACRO_ENABLED
    AP_Int8 acro_options;               ///< Acro mode options bitmask (rate limiting, etc.)
#endif

#if MODE_AUTO_ENABLED
    AP_Int32 auto_options;              ///< Auto mode options bitmask (mission behavior customization)
#endif

#if MODE_GUIDED_ENABLED
    AP_Int32 guided_options;            ///< Guided mode options bitmask (control behavior options)
#endif

    AP_Float fs_gcs_timeout;            ///< GCS failsafe timeout in seconds (0=disabled)

#if MODE_RTL_ENABLED
    AP_Int32 rtl_options;               ///< RTL mode options bitmask (climb behavior, path planning, etc.)
#endif

    AP_Int32 flight_options;            ///< General flight options bitmask (applies across multiple modes)
    /** @} */

    /**
     * @name Rangefinder Filtering
     * @brief Low-pass filter for rangefinder altitude measurements
     * @{
     */
#if AP_RANGEFINDER_ENABLED
    AP_Float rangefinder_filt;          ///< Rangefinder low-pass filter frequency in Hz
#endif
    /** @} */

    /**
     * @name Guided Mode Timeout
     * @brief Automatic mode change when guided commands stop
     * @{
     */
#if MODE_GUIDED_ENABLED
    AP_Float guided_timeout;            ///< Guided mode timeout in seconds (0=disabled, switches to RTL/Land if no new commands)
#endif
    /** @} */

    /**
     * @name Surface Tracking and Dead Reckoning Failsafe
     * @brief Terrain following and position estimation during GPS loss
     * @{
     */
    AP_Int8                 surftrak_mode;      ///< Surface tracking mode: 0=Disabled, 1=Manual, 2=Auto
    AP_Int8                 failsafe_dr_enable; ///< Dead reckoning failsafe enable: 0=Disabled, 1=Enabled
    AP_Int16                failsafe_dr_timeout;///< Dead reckoning timeout in seconds
    AP_Float                surftrak_tc;        ///< Surface tracking time constant for altitude smoothing
    /** @} */

    /**
     * @name Takeoff Parameters
     * @brief Throttle and motor speed limits during takeoff
     * @details Controls throttle ramp-up during takeoff for smooth motor start
     *          and validates motor RPM to ensure safe launch conditions.
     * @{
     */
    AP_Float takeoff_throttle_slew_time;        ///< Throttle slew time during takeoff in seconds
    AP_Float takeoff_throttle_max;              ///< Maximum throttle percentage during takeoff (0-100)
#if HAL_WITH_ESC_TELEM && FRAME_CONFIG != HELI_FRAME
    AP_Int16 takeoff_rpm_min;                   ///< Minimum motor RPM required for takeoff
    AP_Int16 takeoff_rpm_max;                   ///< Maximum expected motor RPM during takeoff
#endif
    /** @} */

    /**
     * @name EKF Failsafe Filter
     * @brief Low-pass filter for EKF variance failsafe triggering
     * @{
     */
    AP_Float fs_ekf_filt_hz;            ///< EKF failsafe variance filter cutoff frequency in Hz
    /** @} */

#if WEATHERVANE_ENABLED
    /**
     * @name Weather Vane
     * @brief Automatic heading adjustment to face into wind
     * @{
     */
    AC_WeatherVane weathervane;         ///< Weather vane controller for automatic wind alignment
    /** @} */
#endif

    /**
     * @name Payload Place Parameters
     * @brief Precision payload placement configuration
     * @details Controls autonomous payload delivery behavior including thrust
     *          detection for contact, descent rate, and safety delays.
     * @{
     */
    AP_Float pldp_thrust_placed_fraction;       ///< Thrust fraction indicating payload is placed (0.0-1.0)
    AP_Float pldp_range_finder_maximum_m;       ///< Maximum rangefinder distance for payload place in meters
    AP_Float pldp_delay_s;                      ///< Delay after placement before releasing in seconds
    AP_Float pldp_descent_speed_ms;             ///< Descent speed during payload place in m/s
    /** @} */

    /**
     * @name Attitude Logging
     * @brief Quaternion attitude logging configuration
     * @details Controls high-rate attitude logging for detailed flight analysis.
     * @{
     */
    AP_Int8 att_enable;                 ///< Enable attitude quaternion logging: 0=Disabled, 1=Enabled
    AP_Int8 att_decimation;             ///< Attitude log decimation factor (log every Nth sample)
    /** @} */
};

extern const AP_Param::Info        var_info[];
