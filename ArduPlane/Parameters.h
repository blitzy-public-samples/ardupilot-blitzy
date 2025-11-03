/**
 * @file Parameters.h
 * @brief Complete parameter structure for ArduPlane vehicle configuration
 * 
 * @details This file defines the comprehensive parameter system for ArduPlane including:
 * - EEPROM format versioning and layout management
 * - Parameter identifier enumeration (k_param_*) for persistent storage
 * - Individual parameter declarations using AP_Param types (AP_Int8, AP_Float, etc.)
 * - Parameter groups (g2, landing, NavEKF2, NavEKF3, etc.)
 * - Secondary parameter block (ParametersG2) to avoid 256 key limit
 * 
 * The parameter system uses the AP_Param library to provide:
 * - Persistent storage in EEPROM/flash with wear leveling
 * - Ground station discovery and modification via MAVLink
 * - Default values and validation ranges
 * - Type safety and conversion handling
 * 
 * @warning This file defines the parameter EEPROM layout. Changes to k_format_version
 *          or k_param_* enum values force all users to reload parameters.
 * 
 * @note Parameter metadata (descriptions, units, ranges) defined in Parameters.cpp var_info arrays
 * @note AP_PARAM_VEHICLE_NAME macro identifies this as "plane" vehicle type
 * 
 * Source: ArduPlane/Parameters.h
 */

#pragma once

#define AP_PARAM_VEHICLE_NAME plane

#include <AP_Common/AP_Common.h>

/**
 * @class Parameters
 * @brief Global parameter storage class for ArduPlane vehicle configuration
 * 
 * @details This class defines all persistent configuration parameters stored in EEPROM/flash
 * for the ArduPlane vehicle. The parameter system provides:
 * 
 * **Structure Overview**:
 * - k_format_version: EEPROM format version identifier (CRITICAL - changes force parameter reset)
 * - k_param_* enum: Unique identifiers for each parameter's EEPROM storage location
 * - Individual parameter declarations: Typed AP_Param variables (AP_Int8, AP_Float, AP_Int16, etc.)
 * - Parameter groups: Nested parameter structures (g2, landing, NavEKF2, NavEKF3)
 * 
 * **Parameter Categories**:
 * - Flight modes and channel assignments
 * - Navigation limits and waypoint behavior
 * - Throttle and servo control
 * - Failsafe configuration
 * - Sensor calibration and fusion
 * - Tuning gains (feed-forward, PID controllers)
 * - Hardware configuration (RC, telemetry, peripherals)
 * 
 * **EEPROM Layout Management**:
 * The k_param_* enum values determine EEPROM storage addresses and MUST remain stable
 * across firmware versions for parameter persistence. Changing enum values breaks
 * compatibility and requires incrementing k_format_version.
 * 
 * **Adding New Parameters**:
 * 1. Use an "unused" enum slot or add to end of appropriate section
 * 2. Declare parameter variable with appropriate AP_Param type
 * 3. Add metadata to var_info array in Parameters.cpp
 * 4. DO NOT increment k_format_version unless enum values change
 * 
 * **Parameter Types**:
 * - AP_Int8: 8-bit signed integer (-128 to 127)
 * - AP_Int16: 16-bit signed integer (-32768 to 32767)
 * - AP_Int32: 32-bit signed integer
 * - AP_Float: 32-bit floating point
 * - AP_Enum: Enumerated type with validation
 * 
 * @warning k_format_version changes are EXTREMELY RARE and force parameter reset for all users
 * @warning Changing k_param_* enum values requires k_format_version increment
 * @warning Parameter names exposed to ground stations - changes affect mission planner compatibility
 * 
 * @note Maximum 256 top-level parameters - use ParametersG2 for additional parameters
 * @note Parameter persistence managed by AP_Param library with automatic EEPROM wear leveling
 * @note Ground stations discover parameters via MAVLink PARAM_REQUEST_LIST/PARAM_VALUE messages
 * 
 * @see ParametersG2 for secondary parameter block (avoids 256 key limit)
 * @see Parameters.cpp for parameter metadata definitions (var_info arrays)
 * @see libraries/AP_Param/AP_Param.h for parameter storage system
 */
class Parameters {
public:

    /**
     * @brief EEPROM format version identifier for parameter layout validation
     * 
     * @details Current version: 13
     * 
     * This version number determines whether existing EEPROM parameter data is considered
     * valid and compatible with the current firmware. When the firmware boots, it compares
     * the stored format version against this constant. If they don't match, all parameters
     * are reset to defaults and users must reconfigure their vehicle.
     * 
     * **When to increment k_format_version** (ONLY these circumstances):
     * 
     * 1. **Semantic Change**: The meaning of an existing EEPROM parameter changes
     *    Example: A parameter that was in degrees is now in radians
     * 
     * 2. **Enum Value Change**: The value of an existing k_param_* enum identifier changes
     *    Example: Reordering enum values or inserting new values in middle of enum
     *    This breaks EEPROM storage addresses and causes parameter corruption
     * 
     * **When NOT to increment k_format_version**:
     * 
     * - Adding a new parameter using an "unused" enum slot (SAFE - no version change needed)
     * - Adding a new parameter at the end of an enum section (SAFE if following C++ enum rules)
     * - Changing a parameter's default value (SAFE - only affects new installations)
     * - Renaming a parameter (use parameter conversion instead)
     * - Adding new parameters to ParametersG2 (separate parameter space)
     * 
     * **Impact of incrementing k_format_version**:
     * 
     * - ALL ArduPlane users worldwide will have parameters reset to defaults on next boot
     * - Users must recalibrate sensors, reload tuning, reconfigure all settings
     * - Mission planner will show parameter mismatch warnings
     * - Historical parameter files become incompatible
     * 
     * This should be an EXTREMELY RARE occurrence. If you are considering changing this
     * value "just in case" or "to be safe", you are likely making a mistake.
     * 
     * **C++ Enum Rules for k_param_* values**:
     * 
     * - First enum value defaults to 0 unless explicitly assigned
     * - Subsequent values increment by 1 unless explicitly assigned
     * - Explicit assignment: k_param_foo = 150, sets value to 150
     * - After explicit assignment, next value is 151, 152, etc.
     * - Comments do not affect enum values (/* unused */ doesn't skip numbers)
     * 
     * @warning DO NOT CHANGE THIS VALUE WITHOUT FULL UNDERSTANDING OF CONSEQUENCES
     * @warning Changing this forces parameter reset for ALL ArduPlane users worldwide
     * @warning Consult experienced ArduPilot developers before modifying
     * @warning Document the reason for any change in git commit message
     * 
     * @note Version history: Incremented sparingly over 15+ years of ArduPlane development
     * @note Parameter conversion system can handle some parameter changes without version increment
     * 
     * @see AP_Param::check_var_info() for parameter validation
     * @see AP_Param::invalidate_all() for parameter reset behavior
     */

    //////////////////////////////////////////////////////////////////
    // STOP!!! DO NOT CHANGE THIS VALUE UNTIL YOU FULLY UNDERSTAND THE
    // COMMENTS ABOVE. IF UNSURE, ASK ANOTHER DEVELOPER!!!
    static const uint16_t k_format_version = 13;
    //////////////////////////////////////////////////////////////////

    /**
     * @enum ThrFailsafe
     * @brief Throttle failsafe behavior modes
     * 
     * @details Defines how the vehicle responds when throttle input is lost or falls below
     * the configured failsafe threshold (throttle_fs_value). This is a critical safety
     * feature for detecting RC link loss or transmitter power failure.
     * 
     * **Failsafe Trigger Conditions**:
     * - Throttle PWM value drops below throttle_fs_value (typically ~950us)
     * - Sustained for fs_timeout_short duration (default 1.5 seconds)
     * - Only active when vehicle is armed and in certain flight modes
     * 
     * **Mode Behaviors**:
     * 
     * - **Disabled (0)**: No throttle failsafe monitoring
     *   - Throttle value is used directly without failsafe checks
     *   - WARNING: Not recommended - vehicle may crash on RC link loss
     *   - Use case: Ground testing, very short range operations only
     * 
     * - **Enabled (1)**: Full throttle failsafe with immediate failsafe action
     *   - Triggers configured failsafe action (fs_action_short or fs_action_long)
     *   - Typical actions: RTL (Return to Launch), QLAND, QRTL for quadplanes
     *   - Recommended for most operations
     * 
     * - **EnabledNoFS (2)**: Throttle failsafe detection without triggering failsafe actions
     *   - Monitors throttle and sets internal failsafe flags
     *   - Does NOT trigger automatic failsafe mode changes
     *   - Useful for: Testing failsafe detection, custom failsafe scripting
     *   - WARNING: Vehicle continues in current mode - may be unsafe
     * 
     * @note Throttle failsafe works in conjunction with GCS failsafe (gcs_heartbeat_fs_enabled)
     * @note Failsafe behavior varies by current flight mode (some modes ignore failsafe)
     * @note MANUAL mode typically ignores failsafe to maintain control if reachable
     * 
     * @warning Setting to Disabled removes important safety protection
     * @warning EnabledNoFS requires external handling of failsafe condition
     * 
     * @see throttle_fs_enabled parameter for mode selection
     * @see throttle_fs_value parameter for trigger threshold (PWM microseconds)
     * @see fs_action_short and fs_action_long for failsafe actions
     * @see fs_timeout_short and fs_timeout_long for timing configuration
     */
    enum class ThrFailsafe {
        Disabled    = 0,  ///< No throttle failsafe monitoring (not recommended)
        Enabled     = 1,  ///< Full failsafe with automatic mode changes (recommended)
        EnabledNoFS = 2   ///< Detection only, no automatic failsafe action
    };

    /**
     * @enum k_param
     * @brief Parameter identifier enumeration for EEPROM storage addressing
     * 
     * @details This enum defines unique identifiers for each parameter stored in EEPROM/flash.
     * Each enum value corresponds to a specific storage location and MUST remain stable across
     * firmware versions to maintain parameter persistence and compatibility.
     * 
     * **Critical Stability Requirements**:
     * 
     * - Enum values must NEVER change once deployed in a release
     * - Changing values breaks EEPROM parameter storage for all users
     * - Reordering enum entries changes their numeric values (C++ enum behavior)
     * - Inserting new values in the middle shifts subsequent values (BREAKS COMPATIBILITY)
     * 
     * **C++ Enum Value Rules**:
     * 
     * - Explicit assignment: k_param_foo = 150, fixes value at 150
     * - Auto-increment: After explicit assignment, next values are 151, 152, 153...
     * - Unassigned first value: Defaults to 0
     * - Comments don't affect values: /* unused */ annotations are documentation only
     * 
     * **Adding New Parameters** (SAFE procedures):
     * 
     * 1. **Use "unused" slot**: Replace a parameter marked "// unused" with new parameter
     *    - Maintains enum value stability (no version increment needed)
     *    - Preferred method for adding parameters
     * 
     * 2. **Add to end of section**: Add after last explicitly assigned value in a section
     *    - Requires careful C++ enum math to avoid conflicts
     *    - Must verify no collision with next section's explicit assignments
     * 
     * 3. **Use ParametersG2**: Add to g2 parameter group instead
     *    - Avoids 256 top-level parameter limit
     *    - Separate parameter space, no collision risk
     * 
     * **Parameter Organization** (by functional group):
     * 
     * - 0-9: Core system (format_version, EKF, g2, landing, etc.)
     * - 10-109: Miscellaneous vehicle configuration
     * - 110-119: Telemetry and GCS communication (many deprecated)
     * - 120-129: Fly-by-wire control modes
     * - 130-149: Sensor configuration (compass, baro, GPS, etc.)
     * - 150-159: Navigation parameters
     * - 160-169: Camera/mount and battery monitoring
     * - 170-209: Radio settings (mostly deprecated, moved to RC_Channels)
     * - 210-219: Flight mode assignments
     * - 220-229: Waypoint and fence configuration
     * - 230-239: Controller objects (L1, TECS, roll/pitch/yaw controllers)
     * - 240-252: PID controllers (mostly unused, moved to controller objects)
     * - 253-374: Extended parameters (logging, vehicle common, autotune, etc.)
     * 
     * **"unused" Annotations**:
     * 
     * Parameters marked "// unused" are deprecated but retained for EEPROM compatibility.
     * These slots can be reused for new parameters without incrementing k_format_version.
     * Common reasons for deprecation:
     * - Feature removed from codebase
     * - Parameter moved to different library (e.g., RC_Channels, AP_BattMonitor)
     * - Parameter renamed (old name kept for conversion)
     * - Functionality replaced by improved implementation
     * 
     * **Special Cases**:
     * 
     * - k_param_ins_old vs k_param_ins: Parameter migration/conversion
     * - k_param_serial*_baud_old vs current: Serial system refactoring
     * - k_param_rc_*_old: RC input system modernization
     * 
     * @warning Changing enum values or order requires incrementing k_format_version
     * @warning Reusing "unused" slots is SAFE, but document reason in commit message
     * @warning Verify no enum value collisions when adding parameters
     * 
     * @note Maximum 256 top-level parameters (0-255) - use ParametersG2 for more
     * @note Some values explicitly reserved (254, 255) for future use
     * @note Parameter metadata (name, type, description) in Parameters.cpp var_info arrays
     * 
     * @see k_format_version for EEPROM layout version
     * @see ParametersG2 for secondary parameter block
     * @see Parameters.cpp for AP_Param::GroupInfo var_info[] definitions
     */
    enum {
        ///////////////////////////////////////////////////////////////////////////////
        // Core System Parameters (0-9)
        ///////////////////////////////////////////////////////////////////////////////
        
        /// Layout version number, always key zero - DO NOT CHANGE POSITION
        k_param_format_version = 0,
        k_param_software_type, // unused
        k_param_num_resets, // unused
        k_param_NavEKF2,       ///< Extended Kalman Filter v2 parameter group
        k_param_g2,            ///< Secondary parameter block (ParametersG2)
        k_param_avoidance_adsb, ///< ADSB avoidance system configuration
        k_param_landing,       ///< AP_Landing parameter group (approach, flare, touchdown)
        k_param_NavEKF3,       ///< Extended Kalman Filter v3 parameter group (current)
        k_param_can_mgr,       ///< CAN bus manager configuration
        k_param_osd,           ///< On-Screen Display configuration

        ///////////////////////////////////////////////////////////////////////////////
        // Miscellaneous Vehicle Configuration (10-109)
        ///////////////////////////////////////////////////////////////////////////////
        // This section contains diverse vehicle configuration parameters including:
        // - Control surface configuration (elevons, flaps, mixing)
        // - Takeoff and landing sequences
        // - Stick mixing and trim
        // - Sensors (INS, sonar - many moved to dedicated libraries)
        // - Board configuration and logging
        // - Rally points and terrain following
        // Note: Many parameters marked "unused" have been migrated to specialized libraries
        //
        k_param_auto_trim      = 10, // unused
        k_param_log_bitmask_old,  // unused
        k_param_pitch_trim,
        k_param_mix_mode,
        k_param_reverse_elevons, // unused
        k_param_reverse_ch1_elevon, // unused
        k_param_reverse_ch2_elevon, // unused
        k_param_flap_1_percent,
        k_param_flap_1_speed,
        k_param_flap_2_percent,
        k_param_flap_2_speed,
        k_param_reset_switch_chan, // unused - moved to RC option
        k_param_manual_level, // unused
        k_param_land_pitch_cd,  // unused - moved to AP_Landing
        k_param_ins_old,            // *** Deprecated, remove with next eeprom number change
        k_param_stick_mixing,
        k_param_reset_mission_chan, // unused - moved to RC option
        k_param_land_flare_alt, // unused - moved to AP_Landing
        k_param_land_flare_sec, // unused - moved to AP_Landing
        k_param_crosstrack_min_distance, // unused
        k_param_rudder_steer, // unused
        k_param_throttle_nudge,
        k_param_alt_offset,
        k_param_ins,                // libraries/AP_InertialSensor variables
        k_param_takeoff_throttle_min_speed,
        k_param_takeoff_throttle_min_accel,
        k_param_takeoff_heading_hold, // unused
        k_param_level_roll_limit,
        k_param_hil_servos_unused,  // unused
        k_param_vtail_output, // unused
        k_param_nav_controller, // unused
        k_param_elevon_output, // unused
        k_param_att_controller, // unused
        k_param_mixing_gain,
        k_param_scheduler,
        k_param_relay,
        k_param_takeoff_throttle_delay,
        k_param_mode_takeoff, // was skip_gyro_cal
        k_param_auto_fbw_steer, // unused
        k_param_waypoint_max_radius,
        k_param_ground_steer_alt,        
        k_param_ground_steer_dps,
        k_param_rally_limit_km_old, //unused anymore -- just holding this index
        k_param_hil_err_limit_unused,  // unused
        k_param_sonar_old, // unused
        k_param_log_bitmask,
        k_param_BoardConfig,
        k_param_rssi_range,     // unused, replaced by rssi_ library parameters
        k_param_flapin_channel_old,  // unused, moved to RC_OPTION
        k_param_flaperon_output, // unused
        k_param_gps,
        k_param_autotune_level,
        k_param_rally,
        k_param_serial0_baud,           // deprecated
        k_param_serial1_baud,           // deprecated
        k_param_serial2_baud,           // deprecated
        k_param_takeoff_tdrag_elevator,
        k_param_takeoff_tdrag_speed1,
        k_param_takeoff_rotate_speed,
        k_param_takeoff_throttle_slewrate,
        k_param_takeoff_throttle_max,
        k_param_rangefinder,
        k_param_terrain,
        k_param_terrain_follow,
        k_param_stab_pitch_down_cd_old, // deprecated
        k_param_alt_slope_min,
        k_param_stab_pitch_down,
        k_param_terrain_lookahead,
        k_param_fbwa_tdrag_chan, // unused - moved to RC option
        k_param_rangefinder_landing,
        k_param_land_flap_percent,  // unused - moved to AP_Landing
        k_param_takeoff_flap_percent,
        k_param_flap_slewrate,
        k_param_rtl_autoland,
        k_param_override_channel,
        k_param_stall_prevention,
        k_param_optflow,
        k_param_cli_enabled_old, // unused - CLI removed
        k_param_trim_rc_at_start, // unused
        k_param_hil_mode_unused,  // unused
        k_param_land_disarm_delay,  // unused - moved to AP_Landing
        k_param_alt_slope_max_height,
        k_param_rudder_only,
        k_param_gcs3_unused,               // unused in ArduPilot-4.7
        k_param_gcs_pid_mask,
        k_param_crash_detection_enable,
        k_param_land_abort_throttle_enable, // unused - moved to AP_Landing
        k_param_rssi = 97,
        k_param_rpm_sensor,
        k_param_parachute,
        k_param_arming = 100,
        k_param_parachute_channel, // unused - moved to RC option
        k_param_crash_accel_threshold,
        k_param_override_safety, // unused
        k_param_land_throttle_slewrate, // 104 unused - moved to AP_Landing

        ///////////////////////////////////////////////////////////////////////////////
        // Extended Parameters (105-109)
        ///////////////////////////////////////////////////////////////////////////////
        // Additional parameters that didn't fit in earlier sections
        k_param_fence_retalt = 105,  ///< Fence breach return altitude
        k_param_fence_autoenable,
        k_param_fence_ret_rally,
        k_param_q_attitude_control,
        k_param_takeoff_pitch_limit_reduction_sec,

        ///////////////////////////////////////////////////////////////////////////////
        // Telemetry and GCS Communication (110-119)
        ///////////////////////////////////////////////////////////////////////////////
        // Ground Control Station and telemetry configuration
        // Most GCS parameters moved to GCS library in ArduPilot 4.7
        // Serial baud rate parameters deprecated - use SERIAL*_BAUD parameters instead
        k_param_gcs0_unused = 110,         // unused in ArduPilot-4.7 - moved to GCS library
        k_param_gcs1_unused,               // unused in ArduPilot-4.7
        k_param_sysid_this_mav_old,
        k_param_sysid_my_gcs_old,
        k_param_serial1_baud_old,   // deprecated
        k_param_telem_delay_old,
        k_param_serial0_baud_old,   // deprecated
        k_param_gcs2_unused,               // unused in ArduPilot-4.7
        k_param_serial2_baud_old,   // deprecated
        k_param_serial2_protocol,   // deprecated

        ///////////////////////////////////////////////////////////////////////////////
        // Fly-by-Wire Control Parameters (120-129)
        ///////////////////////////////////////////////////////////////////////////////
        // Configuration for FBWA (Fly-By-Wire A) and FBWB modes
        // - Airspeed limits and cruise settings
        // - Altitude control parameters
        // - ACRO mode rates and locking
        // - Reverse thrust configuration
        k_param_airspeed_min = 120,  ///< Minimum target airspeed (m/s) in auto-throttle modes
        k_param_airspeed_max,
        k_param_cruise_alt_floor,
        k_param_flybywire_elev_reverse,
        k_param_alt_control_algorithm, // unused
        k_param_flybywire_climb_rate,
        k_param_acro_roll_rate,
        k_param_acro_pitch_rate,
        k_param_acro_locking,
        k_param_use_reverse_thrust = 129,

        ///////////////////////////////////////////////////////////////////////////////
        // Sensor Configuration (130-149)
        ///////////////////////////////////////////////////////////////////////////////
        // Sensor hardware configuration and calibration parameters
        // Includes: Compass, barometer, GPS, airspeed, battery monitoring
        // Many battery parameters moved to AP_BattMonitor library
        // IMU and INS parameters moved to AP_InertialSensor library
        // Note: Several parameters marked "deprecated" or "unused" due to library migrations
        k_param_imu = 130,  // unused - IMU configuration moved to AP_InertialSensor
        k_param_altitude_mix, // deprecated

        k_param_compass_enabled_deprecated,
        k_param_compass,
        k_param_battery_monitoring, // unused
        k_param_volt_div_ratio,     // unused
        k_param_curr_amp_per_volt,  // unused
        k_param_input_voltage, // deprecated, can be deleted
        k_param_pack_capacity,      // unused
        k_param_sonar_enabled_old,  // unused
        k_param_ahrs,  // AHRS group
        k_param_barometer,   // barometer ground calibration
        k_param_airspeed,           // only used for parameter conversion; AP_Airspeed parameters moved to AP_Vehicle
        k_param_curr_amp_offset,
        k_param_NavEKF,  // deprecated - remove
        k_param_mission, // mission library
        k_param_serial_manager_old, // serial manager library
        k_param_NavEKF2_old,  // deprecated - remove
        k_param_land_pre_flare_alt, // unused - moved to AP_Landing
        k_param_land_pre_flare_airspeed = 149,  // unused - moved to AP_Landing

        ///////////////////////////////////////////////////////////////////////////////
        // Navigation Parameters (150-159)
        ///////////////////////////////////////////////////////////////////////////////
        // Waypoint navigation, altitude control, and flight envelope limits
        // - Roll and pitch angle limits
        // - Cruise airspeed and RTL altitude
        // - Minimum groundspeed for GPS navigation
        // Note: Crosstrack parameters unused - L1 controller handles lateral navigation
        k_param_crosstrack_gain = 150, // unused - L1 controller replaced crosstrack
        k_param_crosstrack_entry_angle, // unused
        k_param_roll_limit,
        k_param_pitch_limit_max,
        k_param_pitch_limit_min,
        k_param_airspeed_cruise,
        k_param_RTL_altitude,
        k_param_inverted_flight_ch_unused, // unused
        k_param_min_groundspeed,
        k_param_crosstrack_use_wind, // unused


        ///////////////////////////////////////////////////////////////////////////////
        // Camera, Mount, and Peripheral Devices (160-169)
        ///////////////////////////////////////////////////////////////////////////////
        // Configuration for camera triggering, gimbal mounts, and related peripherals
        // - Camera trigger and control
        // - Gimbal mount stabilization and pointing
        // - ADSB (Automatic Dependent Surveillance-Broadcast) for traffic avoidance
        // - Notification devices (LEDs, buzzers)
        k_param_camera = 160,  ///< Camera trigger and control configuration
        k_param_camera_mount,
        k_param_camera_mount2,      // unused
        k_param_adsb,
        k_param_notify,
        k_param_land_pre_flare_sec = 165,   // unused - moved to AP_Landing

        ///////////////////////////////////////////////////////////////////////////////
        // Battery Monitoring (166-169)
        ///////////////////////////////////////////////////////////////////////////////
        // Battery voltage and current monitoring configuration
        // Note: Most battery parameters migrated to AP_BattMonitor library
        // Legacy pin configuration parameters unused in modern firmware
        k_param_battery = 166,  ///< Battery monitoring configuration (AP_BattMonitor library)
        k_param_rssi_pin,               // unused, replaced by rssi_ library parameters - 167
        k_param_battery_volt_pin,       // unused - 168
        k_param_battery_curr_pin,       // unused - 169

        ///////////////////////////////////////////////////////////////////////////////
        // Radio Configuration (170-209)
        ///////////////////////////////////////////////////////////////////////////////
        // RC input, throttle, and failsafe configuration
        // - RC channel parameters (mostly deprecated - moved to RC_Channels library)
        // - Throttle limits, cruise, and slew rate
        // - Failsafe configuration (throttle, GCS, battery)
        // - Tuning channel assignment
        // Note: k_param_rc_*_old parameters unused - RC system modernized in ArduPilot 4.0+
        k_param_rc_1_old = 170,  // unused - RC channel configuration moved to RC_Channels library
        k_param_rc_2_old,
        k_param_rc_3_old,
        k_param_rc_4_old,
        k_param_rc_5_old,
        k_param_rc_6_old,
        k_param_rc_7_old,
        k_param_rc_8_old,
        k_param_rc_9_old,
        k_param_rc_10_old,
        k_param_rc_11_old,

        k_param_throttle_min,
        k_param_throttle_max,
        k_param_throttle_fs_enabled,
        k_param_throttle_fs_value,
        k_param_throttle_cruise,

        k_param_fs_action_short,
        k_param_fs_action_long,
        k_param_gcs_heartbeat_fs_enabled,
        k_param_throttle_slewrate,
        k_param_throttle_suppress_manual,
        k_param_throttle_passthru_stabilize,
        k_param_rc_12_old,
        k_param_fs_batt_voltage, // unused - moved to AP_BattMonitor
        k_param_fs_batt_mah,     // unused - moved to AP_BattMonitor
        k_param_fs_timeout_short,
        k_param_fs_timeout_long,
        k_param_rc_13_old,
        k_param_rc_14_old,
        k_param_tuning,

        ///////////////////////////////////////////////////////////////////////////////
        // Feed-Forward Gains and Advanced Control (200-209)
        ///////////////////////////////////////////////////////////////////////////////
        // Feed-forward control gains for improved control response
        // - Rudder mixing for coordinated turns
        // - Throttle-to-pitch and pitch-to-throttle coupling
        // - Speed scaling for control surface effectiveness
        // - Quadplane configuration
        // - RTL and landing behavior
        k_param_kff_pitch_compensation = 200, // unused - pitch compensation removed
        k_param_kff_rudder_mix,
        k_param_kff_pitch_to_throttle, // unused
        k_param_kff_throttle_to_pitch,
        k_param_scaling_speed,
        k_param_quadplane,
        k_param_rtl_radius,
        k_param_land_then_servos_neutral,   // unused - moved to AP_Landing
        k_param_rc_15_old,
        k_param_rc_16_old,

        ///////////////////////////////////////////////////////////////////////////////
        // Flight Mode Configuration (210-219)
        ///////////////////////////////////////////////////////////////////////////////
        // Flight mode selection and configuration
        // - Flight mode channel assignment (which RC input controls mode)
        // - Six flight mode slots (FLTMODE1 through FLTMODE6)
        // - Initial/startup mode selection
        // - Landing slope recalculation thresholds (moved to AP_Landing)
        k_param_flight_mode_channel = 210,  ///< RC channel for flight mode selection (typically channel 5 or 8)
        k_param_flight_mode1,
        k_param_flight_mode2,
        k_param_flight_mode3,
        k_param_flight_mode4,
        k_param_flight_mode5,
        k_param_flight_mode6,
        k_param_initial_mode,
        k_param_land_slope_recalc_shallow_threshold,    // unused - moved to AP_Landing
        k_param_land_slope_recalc_steep_threshold_to_abort, // unused - moved to AP_Landing

        ///////////////////////////////////////////////////////////////////////////////
        // Waypoint and Geofence Configuration (220-229)
        ///////////////////////////////////////////////////////////////////////////////
        // Mission waypoint behavior and geofence parameters
        // - Waypoint radius and loiter radius
        // - Geofence configuration (action, limits, enable channel)
        // Note: Many parameters moved to dedicated AP_Mission and AC_Fence libraries
        k_param_waypoint_mode = 220, // unused - waypoint mode removed
        k_param_command_total,  // unused
        k_param_command_index,  // unused
        k_param_waypoint_radius,
        k_param_loiter_radius,
        k_param_fence_action,
        k_param_fence_total,
        k_param_fence_channel, // unused - moved to RC option
        k_param_fence_minalt,
        k_param_fence_maxalt,

        ///////////////////////////////////////////////////////////////////////////////
        // Controller Objects and Libraries (230-239)
        ///////////////////////////////////////////////////////////////////////////////
        // Parameter groups for major control and simulation libraries
        // - SITL (Software In The Loop) simulation configuration
        // - Advanced Failsafe System (AFS)
        // - Roll, pitch, yaw, and steer controller tuning
        // - L1 lateral navigation controller
        // - TECS (Total Energy Control System) for altitude/airspeed
        // - RC input mapping
        k_param_sitl = 230,  ///< SITL simulation parameters
        k_param_afs,
        k_param_rollController,
        k_param_pitchController,
        k_param_yawController,
        k_param_L1_controller,
        k_param_rcmap,
        k_param_TECS_controller,
        k_param_rally_total_old,  //unused
        k_param_steerController,

        ///////////////////////////////////////////////////////////////////////////////
        // Legacy PID Controllers (240-252)
        ///////////////////////////////////////////////////////////////////////////////
        // Legacy PID controller parameters - mostly unused
        // Modern ArduPlane uses object-oriented controllers (rollController, pitchController, etc.)
        // PID tuning now embedded in controller objects defined above (230-239)
        k_param_pidNavRoll = 240, // unused - navigation moved to L1 controller
        k_param_pidServoRoll, // unused
        k_param_pidServoPitch, // unused
        k_param_pidNavPitchAirspeed, // unused
        k_param_pidServoRudder, // unused
        k_param_pidTeThrottle, // unused
        k_param_pidNavPitchAltitude, // unused
        k_param_pidWheelSteer, // unused

        k_param_mixing_offset,
        k_param_dspoiler_rud_rate,
        k_param_airspeed_stall,

        k_param_logger = 253, ///< AP_Logger parameter group (logging configuration)

        ///////////////////////////////////////////////////////////////////////////////
        // Reserved Slots (254-256)
        ///////////////////////////////////////////////////////////////////////////////
        // k_param values 254 and 255 reserved for future use
        // k_param_vehicle starts at 257 to avoid conflicts

        ///////////////////////////////////////////////////////////////////////////////
        // Vehicle Common and Extended Parameters (257-374)
        ///////////////////////////////////////////////////////////////////////////////
        // Shared parameters and late additions to parameter system
        // - Vehicle common library parameters
        // - GCS instances 4-6 (deprecated in 4.7)
        // - ACRO mode yaw rate
        // - Takeoff configuration options
        // - Autotune options
        // - Pullup maneuver configuration
        // - Quicktune system
        // - Mode-specific options (autoland)
        k_param_vehicle = 257, ///< Vehicle common parameter block (shared across vehicle types)
        k_param_gcs4_unused,               // unused in ArduPilot-4.7
        k_param_gcs5_unused,               // unused in ArduPilot-4.7
        k_param_gcs6_unused,               // unused in ArduPilot-4.7
        k_param_fence,         // vehicle fence - unused
        k_param_acro_yaw_rate,
        k_param_takeoff_throttle_max_t,
        k_param_autotune_options,
        k_param_takeoff_throttle_min,
        k_param_takeoff_options,
        k_param_takeoff_throttle_idle,

        k_param_pullup = 270,
        k_param_quicktune,
        k_param_mode_autoland,
        k_param__gcs,

    };

    AP_Int16 format_version;

    AP_Enum<RtlAutoland> rtl_autoland;

    AP_Int8  crash_accel_threshold;

    // Feed-forward gains
    //
    AP_Float kff_rudder_mix;
    AP_Float kff_pitch_to_throttle;
    AP_Float kff_throttle_to_pitch;
    AP_Float ground_steer_alt;
    AP_Int16 ground_steer_dps;
    AP_Float stab_pitch_down;

    // speed used for speed scaling
    AP_Float scaling_speed;

    // Waypoints
    //
    AP_Int16 waypoint_radius;
    AP_Int16 waypoint_max_radius;
    AP_Int16 rtl_radius;

    // Fly-by-wire
    //
    AP_Int8 flybywire_elev_reverse;
    AP_Int8 flybywire_climb_rate;

    // Throttle
    //
    AP_Int8 throttle_suppress_manual;
    AP_Int8 throttle_passthru_stabilize;
    AP_Enum<ThrFailsafe> throttle_fs_enabled;
    AP_Int16 throttle_fs_value;
    AP_Int8 throttle_nudge;
    AP_Int32 use_reverse_thrust;

    // Failsafe
    AP_Int8 fs_action_short;
    AP_Int8 fs_action_long;
    AP_Float fs_timeout_short;
    AP_Float fs_timeout_long;
    AP_Int8 gcs_heartbeat_fs_enabled;

    // Flight modes
    //
    AP_Int8 flight_mode_channel;
    AP_Int8 flight_mode1;
    AP_Int8 flight_mode2;
    AP_Int8 flight_mode3;
    AP_Int8 flight_mode4;
    AP_Int8 flight_mode5;
    AP_Int8 flight_mode6;
    AP_Int8 initial_mode;

    // Navigational manoeuvring limits
    //
    AP_Int16 alt_offset;
    AP_Int16 acro_roll_rate;
    AP_Int16 acro_pitch_rate;
    AP_Int16 acro_yaw_rate;
    AP_Int8  acro_locking;

    // Misc
    //
    AP_Int8 rudder_only;
    AP_Float mixing_gain;
    AP_Int16 mixing_offset;
    AP_Int16 dspoiler_rud_rate;
    AP_Int32 log_bitmask;
    AP_Float RTL_altitude;
    AP_Float pitch_trim;
    AP_Float cruise_alt_floor;

    AP_Int8 flap_1_percent;
    AP_Int8 flap_1_speed;
    AP_Int8 flap_2_percent;
    AP_Int8 flap_2_speed;
    AP_Int8 takeoff_flap_percent;  
    AP_Enum<StickMixing> stick_mixing;
    AP_Float takeoff_throttle_min_speed;
    AP_Float takeoff_throttle_min_accel;
    AP_Int8 takeoff_throttle_delay;
    AP_Int8 takeoff_tdrag_elevator;
    AP_Float takeoff_tdrag_speed1;
    AP_Float takeoff_rotate_speed;
    AP_Int8 takeoff_throttle_slewrate;
    AP_Float takeoff_pitch_limit_reduction_sec;
    AP_Int8 level_roll_limit;
#if AP_TERRAIN_AVAILABLE
    AP_Int32 terrain_follow;
    AP_Int16 terrain_lookahead;
#endif
    AP_Int16 alt_slope_min;
    AP_Float alt_slope_max_height;
    AP_Int8 rangefinder_landing;
    AP_Int8 flap_slewrate;
#if HAL_WITH_IO_MCU
    AP_Int8 override_channel;
#endif
    AP_Int16 gcs_pid_mask;
};

/**
 * @class ParametersG2
 * @brief Secondary parameter block to avoid 256 top-level key limit
 * 
 * @details ArduPilot's parameter system (AP_Param) has a limitation of 256 top-level
 * parameter keys (k_param enum values 0-255). As ArduPlane evolved and added features,
 * the parameter space filled up. ParametersG2 provides a second parameter namespace
 * accessed through the single top-level key k_param_g2.
 * 
 * **Design Pattern**:
 * - Parameters class contains: k_param_g2 (single top-level key)
 * - ParametersG2 contains: All additional parameters as a nested group
 * - From ground station perspective: Appears as parameters with "g2." prefix
 * 
 * **Parameter Organization in G2**:
 * - RC input configuration (rc_channels, manual_rc_mask)
 * - Servo output configuration (servo_channels)
 * - Advanced features: ICEngine, Soaring, Precision Landing
 * - Button input handling
 * - Crow flaps configuration
 * - Battery voltage compensation
 * - Guided mode heading control
 * - Follow mode
 * - EKF failsafe threshold
 * - RTL minimum climb
 * - Manual mode expo curves
 * - Rangefinder orientation
 * - System identification (autotune analysis)
 * 
 * **Adding Parameters to G2**:
 * 1. Declare parameter variable in ParametersG2 class (this file)
 * 2. Add to var_info array in Parameters.cpp
 * 3. No concerns about 256-key limit or k_param enum collisions
 * 4. No need to increment k_format_version (separate parameter space)
 * 
 * **Feature Flags**:
 * Many G2 parameters conditionally compiled based on:
 * - HAL_BUTTON_ENABLED: Button support
 * - AP_ICENGINE_ENABLED: Internal combustion engine
 * - HAL_SOARING_ENABLED: Thermal soaring
 * - AP_LANDINGGEAR_ENABLED: Retractable landing gear
 * - AC_PRECLAND_ENABLED: Precision landing
 * - AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED: Guided mode yaw control
 * - AP_SCRIPTING_ENABLED && AP_FOLLOW_ENABLED: Follow mode
 * - AP_RANGEFINDER_ENABLED: Rangefinder configuration
 * - AP_PLANE_SYSTEMID_ENABLED: System identification
 * 
 * @note G2 parameters appear with "g2." prefix in ground stations (e.g., "g2.manual_rc_mask")
 * @note No k_param enum needed - uses AP_Param::GroupInfo for nested parameters
 * @note Preferred location for new parameters to avoid main parameter space exhaustion
 * 
 * @see Parameters class for main parameter block (0-255 keys)
 * @see Parameters.cpp for var_info[] definition and parameter metadata
 * @see libraries/AP_Param/AP_Param.h for nested parameter group documentation
 */
class ParametersG2 {
public:
    ParametersG2(void);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    // just to make compilation easier when all things are compiled out...
    uint8_t unused_integer;

    // button reporting library
#if HAL_BUTTON_ENABLED
    AP_Button *button_ptr;
#endif

#if AP_ICENGINE_ENABLED
    // internal combustion engine control
    AP_ICEngine ice_control;
#endif

    // RC input channels
    RC_Channels_Plane rc_channels;
    
    // control over servo output ranges
    SRV_Channels servo_channels;

#if HAL_SOARING_ENABLED
    // ArduSoar parameters
    SoaringController soaring_controller;
#endif

    // dual motor tailsitter rudder to differential thrust scaling: 0-100%
    AP_Int8 rudd_dt_gain;

    // mask of channels to do manual pass-thru for
    AP_Int32 manual_rc_mask;

    // home reset altitude threshold
    AP_Int8 home_reset_threshold;

    AP_Int32 flight_options;

    AP_Int8 takeoff_throttle_accel_count;
    AP_Int8 takeoff_timeout;

#if AP_LANDINGGEAR_ENABLED
    AP_LandingGear landing_gear;
#endif

#if AC_PRECLAND_ENABLED
    AC_PrecLand precland;
#endif

    // crow flaps weighting
    AP_Int8 crow_flap_weight_outer;
    AP_Int8 crow_flap_weight_inner;
    AP_Int8 crow_flap_options;
    AP_Int8 crow_flap_aileron_matching;

    // Forward throttle battery voltage compensation
    class FWD_BATT_CMP {
    public:
        // Calculate the throttle scale to compensate for battery voltage drop
        void update();

        // Apply throttle scale to min and max limits
        void apply_min_max(int8_t &min_throttle, int8_t &max_throttle) const;

        // Apply throttle scale to throttle demand
        float apply_throttle(float throttle) const;

        AP_Float batt_voltage_max;
        AP_Float batt_voltage_min;
        AP_Float batt_voltage_throttle_cutoff;
        AP_Int8  batt_idx;

    private:
        bool enabled;
        float ratio;
    } fwd_batt_cmp;


#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    // guided yaw heading PID
    AC_PID guidedHeading{5000.0,  0.0,   0.0, 0 ,  10.0,   5.0,  5.0 ,  5.0  , 0.0};
#endif

#if AP_SCRIPTING_ENABLED && AP_FOLLOW_ENABLED
    AP_Follow follow;
#endif

    AP_Float        fs_ekf_thresh;

    // min initial climb in RTL
    AP_Int16        rtl_climb_min;

    AP_Int8         man_expo_roll;
    AP_Int8         man_expo_pitch;
    AP_Int8         man_expo_rudder;

    AP_Int32        oneshot_mask;
    
    AP_Int8         axis_bitmask; // axes to be autotuned

#if AP_RANGEFINDER_ENABLED
    // orientation of rangefinder to use for landing
    AP_Int8 rangefinder_land_orient;
#endif

#if AP_PLANE_SYSTEMID_ENABLED
    AP_SystemID systemid;
#endif
};

extern const AP_Param::Info var_info[];
