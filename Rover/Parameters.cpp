/**
 * @file Parameters.cpp
 * @brief Implementation of Rover parameter definitions and management
 * 
 * @details This file contains the complete parameter definition tables for the Rover vehicle,
 *          including both the primary parameter group (var_info) and the secondary parameter
 *          group (ParametersG2). It also implements parameter loading, conversion from legacy
 *          parameter names, and default value initialization.
 * 
 *          The parameter system uses AP_Param for persistent storage in EEPROM with wear leveling.
 *          Parameters are organized hierarchically with top-level scalar values (GSCALAR) and
 *          grouped object parameters (GOBJECT, AP_SUBGROUPINFO) that encapsulate related settings.
 * 
 *          Parameter Groups:
 *          - System: FORMAT_VERSION, LOG_BITMASK, INITIAL_MODE, GCS_PID_MASK
 *          - Navigation: CRUISE_SPEED, CRUISE_THROTTLE, WP_* parameters in g2.wp_nav
 *          - Control: Steering (ATC_* in g2.attitude_control), Motors (MOT_* in g2.motors)
 *          - Safety: Failsafe (FS_*), Crash detection (CRASH_*), EKF monitoring (FS_EKF_*)
 *          - Modes: MODE1-MODE6, MODE_CH, PILOT_STEER_TYPE
 *          - Sensors: COMPASS_, BARO, INS, GPS, AHRS_, EK2_, EK3_
 *          - Peripherals: SERVO, RC, CAM, MNT, NTF_
 * 
 * @note Parameter naming conventions:
 *       - Top-level scalars use descriptive names (CRUISE_SPEED, FS_ACTION)
 *       - Object groups use prefixes (COMPASS_, BARO, MOT_, ATC_, WP_)
 *       - Group prefixes end with underscore for sub-parameters
 *       - Numeric suffixes indicate indexed items (MODE1-MODE6)
 * 
 * @warning Changing parameter names requires entries in conversion_table to preserve
 *          user settings across firmware updates. Never reuse parameter indices.
 * 
 * @see Rover.h for parameter structure definitions (Parameters and ParametersG2)
 * @see AP_Param.h for parameter storage and conversion system
 * 
 * Source: Rover/Parameters.cpp
 */

#include "Rover.h"

#include <AP_Gripper/AP_Gripper.h>

/**
 * @brief Primary parameter information table for Rover vehicle
 * 
 * This table defines all parameters accessible through the ground control station and stored
 * persistently in EEPROM. Each entry uses one of the following macros:
 * 
 * - GSCALAR(variable, "NAME", default): Top-level scalar parameter stored in Parameters.g
 * - GOBJECT(object, "PREFIX_", Type): Object with its own var_info table (separate library)
 * - GOBJECTN(object, name, "PREFIX_", Type): Named object reference (for nested objects)
 * - AP_SUBGROUPINFO(object, "PREFIX_", key, class, Type): Sub-group within ParametersG2
 * 
 * Parameter documentation uses @Param tags processed by the ArduPilot build system to generate
 * ground station metadata files. Required tags: @DisplayName, @Description, @User.
 * Optional tags: @Units, @Range, @Increment, @Values, @Bitmask, @RebootRequired.
 * 
 * Storage: Parameters are stored using a key-based system where each top-level parameter gets
 * a unique key from the k_param enum in Parameters.h. Changes to this table must maintain
 * backward compatibility through conversion_table entries below.
 * 
 * @note This table is parsed at compile time to generate parameter metadata.
 * @note Parameter order affects EEPROM layout - only append new parameters at the end.
 * @note Removed parameters must leave placeholder comments to avoid key reuse.
 */
const AP_Param::Info Rover::var_info[] = {
    // ========================================
    // System Configuration Parameters
    // ========================================
    // Core system parameters for version tracking, logging, and system identification

    // @Param: FORMAT_VERSION
    // @DisplayName: Eeprom format version number
    // @Description: This value is incremented when changes are made to the eeprom format
    // @User: Advanced
    GSCALAR(format_version,         "FORMAT_VERSION",   1),

    // @Param: LOG_BITMASK
    // @DisplayName: Log bitmask
    // @Description: Bitmap of what log types to enable in on-board logger. This value is made up of the sum of each of the log types you want to be saved. On boards supporting microSD cards or other large block-storage devices it is usually best just to enable all basic log types by setting this to 65535.
    // @Bitmask: 0:Fast Attitude,1:Medium Attitude,2:GPS,3:System Performance,4:Throttle,5:Navigation Tuning,7:IMU,8:Mission Commands,9:Battery Monitor,10:Rangefinder,11:Compass,12:Camera,13:Steering,14:RC Input-Output,19:Raw IMU,20:Video Stabilization,21:Optical Flow
    // @User: Advanced
    GSCALAR(log_bitmask,            "LOG_BITMASK",      DEFAULT_LOG_BITMASK),

    // @Param: RST_SWITCH_CH
    // @DisplayName: Reset Switch Channel
    // @Description: RC channel to use to reset to last flight mode after geofence takeover.
    // @User: Advanced
    GSCALAR(reset_switch_chan,      "RST_SWITCH_CH",    0),

    // @Param: INITIAL_MODE
    // @DisplayName: Initial driving mode
    // @Description: This selects the mode to start in on boot. This is useful for when you want to start in AUTO mode on boot without a receiver. Usually used in combination with when AUTO_TRIGGER_PIN or AUTO_KICKSTART.
    // @CopyValuesFrom: MODE1
    // @User: Advanced
    GSCALAR(initial_mode,        "INITIAL_MODE",     (int8_t)Mode::Number::MANUAL),

    // SYSID_THISMAV was here

    // SYSID_MYGCS was here

    // TELEM_DELAY was here

    // @Param: GCS_PID_MASK
    // @DisplayName: GCS PID tuning mask
    // @Description: bitmask of PIDs to send MAVLink PID_TUNING messages for
    // @User: Advanced
    // @Bitmask: 0:Steering,1:Throttle,2:Pitch,3:Left Wheel,4:Right Wheel,5:Sailboat Heel,6:Velocity North,7:Velocity East
    GSCALAR(gcs_pid_mask,           "GCS_PID_MASK",     0),

    // @Param: AUTO_TRIGGER_PIN
    // @DisplayName: Auto mode trigger pin
    // @Description: pin number to use to enable the throttle in auto mode. If set to -1 then don't use a trigger, otherwise this is a pin number which if held low in auto mode will enable the motor to run. If the switch is released while in AUTO then the motor will stop again. This can be used in combination with INITIAL_MODE to give a 'press button to start' rover with no receiver.
    // @Values: -1:Disabled,0:APM TriggerPin0,1:APM TriggerPin1,2:APM TriggerPin2,3:APM TriggerPin3,4:APM TriggerPin4,5:APM TriggerPin5,6:APM TriggerPin6,7:APM TriggerPin7,8:APM TriggerPin8,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Standard
    GSCALAR(auto_trigger_pin,        "AUTO_TRIGGER_PIN", -1),

    // @Param: AUTO_KICKSTART
    // @DisplayName: Auto mode trigger kickstart acceleration
    // @Description: X acceleration in meters/second/second to use to trigger the motor start in auto mode. If set to zero then auto throttle starts immediately when the mode switch happens, otherwise the rover waits for the X acceleration to go above this value before it will start the motor
    // @Units: m/s/s
    // @Range: 0 20
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(auto_kickstart,          "AUTO_KICKSTART", 0.0f),

    // ========================================
    // Navigation and Speed Control Parameters
    // ========================================
    // Target speeds, throttle settings, and navigation behavior for autonomous modes.
    // CRUISE_SPEED defines target velocity, CRUISE_THROTTLE provides initial throttle estimate.
    // The speed controller adjusts throttle dynamically to maintain CRUISE_SPEED.

    // @Param: CRUISE_SPEED
    // @DisplayName: Target cruise speed in auto modes
    // @Description: The target speed in auto missions.
    // @Units: m/s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(speed_cruise,        "CRUISE_SPEED",    CRUISE_SPEED),


    // @Param: CRUISE_THROTTLE
    // @DisplayName: Base throttle percentage in auto
    // @Description: The base throttle percentage to use in auto mode. The CRUISE_SPEED parameter controls the target speed, but the rover starts with the CRUISE_THROTTLE setting as the initial estimate for how much throttle is needed to achieve that speed. It then adjusts the throttle based on how fast the rover is actually going.
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    GSCALAR(throttle_cruise,        "CRUISE_THROTTLE",    50),

    // @Param: PILOT_STEER_TYPE
    // @DisplayName: Pilot input steering type
    // @Description: Pilot RC input interpretation
    // @Values: 0:Default,1:Two Paddles Input,2:Direction reversed when backing up,3:Direction unchanged when backing up
    // @User: Standard
    GSCALAR(pilot_steer_type, "PILOT_STEER_TYPE", 0),

    // ========================================
    // Failsafe Configuration Parameters
    // ========================================
    // Safety parameters that define rover behavior when communication is lost or sensors fail.
    // FS_ACTION determines what the rover does (Hold, RTL, SmartRTL, Terminate).
    // FS_TIMEOUT sets the delay before triggering failsafe to avoid false triggers.
    // FS_THR_ENABLE/FS_THR_VALUE: Throttle failsafe when RC signal lost
    // FS_GCS_ENABLE: Ground control station heartbeat failsafe
    // FS_CRASH_CHECK: Crash detection and automatic stop
    // FS_EKF_*: Extended Kalman Filter health monitoring and failsafe

    // @Param: FS_ACTION
    // @DisplayName: Failsafe Action
    // @Description: What to do on a failsafe event
    // @Values: 0:Nothing,1:RTL,2:Hold,3:SmartRTL or RTL,4:SmartRTL or Hold,5:Terminate,6:Loiter or Hold
    // @User: Standard
    GSCALAR(fs_action,    "FS_ACTION",     (int8_t)FailsafeAction::Hold),

    // @Param: FS_TIMEOUT
    // @DisplayName: Failsafe timeout
    // @Description: The time in seconds that a failsafe condition must persist before the failsafe action is triggered
    // @Units: s
    // @Range: 1 100
    // @Increment: 0.5
    // @User: Standard
    GSCALAR(fs_timeout,    "FS_TIMEOUT",     1.5),

    // @Param: FS_THR_ENABLE
    // @DisplayName: Throttle Failsafe Enable
    // @Description: The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel to a low value. This can be used to detect the RC transmitter going out of range. Failsafe will be triggered when the throttle channel goes below the FS_THR_VALUE for FS_TIMEOUT seconds.
    // @Values: 0:Disabled,1:Enabled,2:Enabled Continue with Mission in Auto
    // @User: Standard
    GSCALAR(fs_throttle_enabled,    "FS_THR_ENABLE",     FS_THR_ENABLED),

    // @Param: FS_THR_VALUE
    // @DisplayName: Throttle Failsafe Value
    // @Description: The PWM level on the throttle channel below which throttle failsafe triggers.
    // @Range: 910 1100
    // @Increment: 1
    // @User: Standard
    GSCALAR(fs_throttle_value,      "FS_THR_VALUE",     910),

    // @Param: FS_GCS_ENABLE
    // @DisplayName: GCS failsafe enable
    // @Description: Enable ground control station telemetry failsafe. When enabled the Rover will execute the FS_ACTION when it fails to receive MAVLink heartbeat packets for FS_TIMEOUT seconds.
    // @Values: 0:Disabled,1:Enabled,2:Enabled Continue with Mission in Auto
    // @User: Standard
    GSCALAR(fs_gcs_enabled, "FS_GCS_ENABLE",   FS_GCS_DISABLED),

    // @Param: FS_CRASH_CHECK
    // @DisplayName: Crash check action
    // @Description: What to do on a crash event. When enabled the rover will go to hold if a crash is detected.
    // @Values: 0:Disabled,1:Hold,2:HoldAndDisarm
    // @User: Standard
    GSCALAR(fs_crash_check, "FS_CRASH_CHECK",    FS_CRASH_DISABLE),

    // @Param: FS_EKF_ACTION
    // @DisplayName: EKF Failsafe Action
    // @Description: Controls the action that will be taken when an EKF failsafe is invoked
    // @Values: 0:Disabled,1:Hold,2:ReportOnly
    // @User: Advanced
    GSCALAR(fs_ekf_action, "FS_EKF_ACTION", FS_EKF_HOLD),

    // @Param: FS_EKF_THRESH
    // @DisplayName: EKF failsafe variance threshold
    // @Description: Allows setting the maximum acceptable compass and velocity variance
    // @Values: 0.6:Strict, 0.8:Default, 1.0:Relaxed
    // @User: Advanced
    GSCALAR(fs_ekf_thresh, "FS_EKF_THRESH", 0.8f),

    // ========================================
    // Flight Mode Configuration Parameters
    // ========================================
    // MODE_CH selects which RC channel controls mode switching (typically channel 8).
    // MODE1-MODE6 define which driving mode is selected for each position of the mode switch:
    //   - MODE1: Switch low (910-1230 PWM, and >2049 PWM)
    //   - MODE2: Switch position 2 (1231-1360 PWM)
    //   - MODE3: Switch position 3 (1361-1490 PWM)
    //   - MODE4: Switch position 4 (1491-1620 PWM)
    //   - MODE5: Switch position 5 (1621-1749 PWM)
    //   - MODE6: Switch high (1750-2049 PWM)
    // Available modes: Manual, Acro, Steering, Hold, Loiter, Follow, Simple, Dock, Circle, Auto, RTL, SmartRTL, Guided

    // @Param: MODE_CH
    // @DisplayName: Mode channel
    // @Description: RC Channel to use for driving mode control
    // @User: Advanced
    GSCALAR(mode_channel,    "MODE_CH",       MODE_CHANNEL),

    // @Param: MODE1
    // @DisplayName: Mode1
    // @Values: 0:Manual,1:Acro,3:Steering,4:Hold,5:Loiter,6:Follow,7:Simple,8:Dock,9:Circle,10:Auto,11:RTL,12:SmartRTL,15:Guided
    // @User: Standard
    // @Description: Driving mode for switch position 1 (910 to 1230 and above 2049)
    GSCALAR(mode1,           "MODE1",         (int8_t)Mode::Number::MANUAL),

    // @Param: MODE2
    // @DisplayName: Mode2
    // @Description: Driving mode for switch position 2 (1231 to 1360)
    // @CopyValuesFrom: MODE1
    // @User: Standard
    GSCALAR(mode2,           "MODE2",         (int8_t)Mode::Number::MANUAL),

    // @Param: MODE3
    // @CopyFieldsFrom: MODE1
    // @DisplayName: Mode3
    // @Description: Driving mode for switch position 3 (1361 to 1490)
    GSCALAR(mode3,           "MODE3",         (int8_t)Mode::Number::MANUAL),

    // @Param: MODE4
    // @CopyFieldsFrom: MODE1
    // @DisplayName: Mode4
    // @Description: Driving mode for switch position 4 (1491 to 1620)
    GSCALAR(mode4,           "MODE4",         (int8_t)Mode::Number::MANUAL),

    // @Param: MODE5
    // @CopyFieldsFrom: MODE1
    // @DisplayName: Mode5
    // @Description: Driving mode for switch position 5 (1621 to 1749)
    GSCALAR(mode5,           "MODE5",         (int8_t)Mode::Number::MANUAL),

    // @Param: MODE6
    // @CopyFieldsFrom: MODE1
    // @DisplayName: Mode6
    // @Description: Driving mode for switch position 6 (1750 to 2049)
    GSCALAR(mode6,           "MODE6",         (int8_t)Mode::Number::MANUAL),

    // ========================================
    // Sensor and Hardware Object Groups
    // ========================================
    // These GOBJECT/GOBJECTN entries reference parameter groups defined in their respective
    // library implementations. Each has its own var_info table and parameter prefix.
    // Ground control stations query these groups to discover all available sub-parameters.

    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/AP_Compass.cpp
    GOBJECT(compass,                "COMPASS_", Compass),

    // @Group: SCHED_
    // @Path: ../libraries/AP_Scheduler/AP_Scheduler.cpp
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),

    // @Group: BARO
    // @Path: ../libraries/AP_Baro/AP_Baro.cpp
    GOBJECT(barometer, "BARO", AP_Baro),

#if AP_RELAY_ENABLED
    // @Group: RELAY
    // @Path: ../libraries/AP_Relay/AP_Relay.cpp
    GOBJECT(relay,                  "RELAY", AP_Relay),
#endif

    // @Group: RCMAP_
    // @Path: ../libraries/AP_RCMapper/AP_RCMapper.cpp
    GOBJECT(rcmap,                 "RCMAP_",         RCMapper),

    // SR0 through SR6 were here

    // AP_SerialManager was here

#if AP_RANGEFINDER_ENABLED
    // @Group: RNGFND
    // @Path: ../libraries/AP_RangeFinder/AP_RangeFinder.cpp
    GOBJECT(rangefinder,                 "RNGFND", RangeFinder),
#endif

    // @Group: INS
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    GOBJECT(ins,                            "INS", AP_InertialSensor),

#if AP_SIM_ENABLED
    // @Group: SIM_
    // @Path: ../libraries/SITL/SITL.cpp
    GOBJECT(sitl, "SIM_", SITL::SIM),
#endif

    // @Group: AHRS_
    // @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs,                   "AHRS_",    AP_AHRS),

#if AP_CAMERA_ENABLED
    // @Group: CAM
    // @Path: ../libraries/AP_Camera/AP_Camera.cpp
    GOBJECT(camera, "CAM", AP_Camera),
#endif

#if AC_PRECLAND_ENABLED
    // @Group: PLND_
    // @Path: ../libraries/AC_PrecLand/AC_PrecLand.cpp
    GOBJECT(precland,                "PLND_", AC_PrecLand),
#endif

#if HAL_MOUNT_ENABLED
    // @Group: MNT
    // @Path: ../libraries/AP_Mount/AP_Mount.cpp
    GOBJECT(camera_mount,           "MNT",  AP_Mount),
#endif

    // @Group: ARMING_
    // @Path: ../libraries/AP_Arming/AP_Arming.cpp
    GOBJECT(arming,                 "ARMING_", AP_Arming),

    // @Group: BATT
    // @Path: ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    GOBJECT(battery,                "BATT", AP_BattMonitor),

    // @Group: BRD_
    // @Path: ../libraries/AP_BoardConfig/AP_BoardConfig.cpp
    GOBJECT(BoardConfig,            "BRD_",       AP_BoardConfig),

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    // @Group: CAN_
    // @Path: ../libraries/AP_CANManager/AP_CANManager.cpp
    GOBJECT(can_mgr,        "CAN_",       AP_CANManager),
#endif

    // GPS driver
    // @Group: GPS
    // @Path: ../libraries/AP_GPS/AP_GPS.cpp
    GOBJECT(gps, "GPS", AP_GPS),

#if HAL_NAVEKF2_AVAILABLE
    // @Group: EK2_
    // @Path: ../libraries/AP_NavEKF2/AP_NavEKF2.cpp
    GOBJECTN(ahrs.EKF2, NavEKF2, "EK2_", NavEKF2),
#endif

#if HAL_NAVEKF3_AVAILABLE
    // @Group: EK3_
    // @Path: ../libraries/AP_NavEKF3/AP_NavEKF3.cpp
    GOBJECTN(ahrs.EKF3, NavEKF3, "EK3_", NavEKF3),
#endif

#if AP_RPM_ENABLED
    // @Group: RPM
    // @Path: ../libraries/AP_RPM/AP_RPM.cpp
    GOBJECT(rpm_sensor, "RPM", AP_RPM),
#endif

    // @Group: MIS_
    // @Path: ../libraries/AP_Mission/AP_Mission.cpp
    GOBJECTN(mode_auto.mission, mission, "MIS_", AP_Mission),

#if AP_RSSI_ENABLED
    // @Group: RSSI_
    // @Path: ../libraries/AP_RSSI/AP_RSSI.cpp
    GOBJECT(rssi, "RSSI_",  AP_RSSI),
#endif

    // @Group: NTF_
    // @Path: ../libraries/AP_Notify/AP_Notify.cpp
    GOBJECT(notify, "NTF_",  AP_Notify),

#if HAL_BUTTON_ENABLED
    // @Group: BTN_
    // @Path: ../libraries/AP_Button/AP_Button.cpp
    GOBJECT(button, "BTN_",  AP_Button),
#endif

    // @Group:
    // @Path: Parameters.cpp
    GOBJECT(g2, "",  ParametersG2),

#if OSD_ENABLED || OSD_PARAM_ENABLED
    // @Group: OSD
    // @Path: ../libraries/AP_OSD/AP_OSD.cpp
    GOBJECT(osd, "OSD", AP_OSD),
#endif

#if AP_OPTICALFLOW_ENABLED
    // @Group: FLOW
    // @Path: ../libraries/AP_OpticalFlow/AP_OpticalFlow.cpp
    GOBJECT(optflow,   "FLOW", AP_OpticalFlow),
#endif

    // @Group:
    // @Path: ../libraries/AP_Vehicle/AP_Vehicle.cpp
    PARAM_VEHICLE_INFO,

#if HAL_GCS_ENABLED
    // @Group: MAV
    // @Path: ../libraries/GCS_MAVLink/GCS.cpp
    GOBJECT(_gcs,           "MAV",  GCS),
#endif

    AP_VAREND
};

/**
 * @brief Secondary parameter group (ParametersG2) for Rover
 * 
 * @details This second-level parameter group was introduced to overcome the 256 parameter limit
 *          in the original var_info table. ParametersG2 is itself a GOBJECT entry in the primary
 *          var_info table, allowing it to contain additional scalars and sub-groups.
 * 
 *          ParametersG2 contains:
 *          - Vehicle-specific objects: servo_channels, rc_channels, motors, attitude_control
 *          - Navigation: wp_nav (AR_WPNav), smart_rtl, pos_control
 *          - Safety: Advanced failsafe (afs), avoidance (avoid)
 *          - Sensors: beacon, wheel_encoder, proximity, windvane
 *          - Peripherals: Camera/mount integration via mode_dock
 *          - Vehicle configuration: FRAME_CLASS, FRAME_TYPE, TURN_RADIUS
 *          - Mode behavior: LOIT_TYPE, LOIT_RADIUS, STICK_MIXING, MIS_DONE_BEHAVE
 *          - Advanced features: Balance bot (BAL_*), sailboat (SAIL_*), omni steering
 * 
 *          Each AP_SUBGROUPINFO entry has a unique numeric key that must never be reused, even
 *          if the parameter is removed. Placeholder comments mark removed entries.
 * 
 * @note Key numbering in this table is non-sequential due to removed parameters - this is normal.
 * @warning Never reuse key numbers - always use the next available number for new parameters.
 * 
 * Source: Rover/Parameters.cpp:353-643
 */
const AP_Param::GroupInfo ParametersG2::var_info[] = {
    // 1 was AP_Stats

    // 2 was SYSID_ENFORCE

    // ========================================
    // Output and Control Object Groups
    // ========================================

    // @Group: SERVO
    // @Path: ../libraries/SRV_Channel/SRV_Channels.cpp
    AP_SUBGROUPINFO(servo_channels, "SERVO", 3, ParametersG2, SRV_Channels),

    // @Group: RC
    // @Path: ../libraries/RC_Channel/RC_Channels_VarInfo.h
    AP_SUBGROUPINFO(rc_channels, "RC", 4, ParametersG2, RC_Channels_Rover),

#if AP_ROVER_ADVANCED_FAILSAFE_ENABLED
    // @Group: AFS_
    // @Path: ../libraries/AP_AdvancedFailsafe/AP_AdvancedFailsafe.cpp
    AP_SUBGROUPINFO(afs, "AFS_", 5, ParametersG2, AP_AdvancedFailsafe),
#endif

#if AP_BEACON_ENABLED
    // @Group: BCN
    // @Path: ../libraries/AP_Beacon/AP_Beacon.cpp
    AP_SUBGROUPINFO(beacon, "BCN", 6, ParametersG2, AP_Beacon),
#endif

    // 7 was used by AP_VisualOdometry

    // @Group: MOT_
    // @Path: ../libraries/AR_Motors/AP_MotorsUGV.cpp
    AP_SUBGROUPINFO(motors, "MOT_", 8, ParametersG2, AP_MotorsUGV),

    // @Group: WENC
    // @Path: ../libraries/AP_WheelEncoder/AP_WheelEncoder.cpp
    AP_SUBGROUPINFO(wheel_encoder, "WENC", 9, ParametersG2, AP_WheelEncoder),

    // @Group: ATC
    // @Path: ../libraries/APM_Control/AR_AttitudeControl.cpp
    AP_SUBGROUPINFO(attitude_control, "ATC", 10, ParametersG2, AR_AttitudeControl),

    // ========================================
    // Vehicle Configuration Parameters
    // ========================================
    // Physical characteristics and frame-specific settings

    // @Param: TURN_RADIUS
    // @DisplayName: Turn radius of vehicle
    // @Description: Turn radius of vehicle in meters while at low speeds.  Lower values produce tighter turns in steering mode
    // @Units: m
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("TURN_RADIUS", 11, ParametersG2, turn_radius, 0.9),

    // @Param: ACRO_TURN_RATE
    // @DisplayName: Acro mode turn rate maximum
    // @Description: Acro mode turn rate maximum
    // @Units: deg/s
    // @Range: 0 360
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ACRO_TURN_RATE", 12, ParametersG2, acro_turn_rate, 180.0f),

    // ========================================
    // Navigation and Autonomous Mode Parameters
    // ========================================

    // @Group: SRTL_
    // @Path: ../libraries/AP_SmartRTL/AP_SmartRTL.cpp
    AP_SUBGROUPINFO(smart_rtl, "SRTL_", 13, ParametersG2, AP_SmartRTL),

    // 14 was WP_SPEED and should not be re-used

    // @Param: RTL_SPEED
    // @DisplayName: Return-to-Launch speed default
    // @Description: Return-to-Launch speed default.  If zero use WP_SPEED or CRUISE_SPEED.
    // @Units: m/s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("RTL_SPEED", 15, ParametersG2, rtl_speed, 0.0f),

    // @Param: FRAME_CLASS
    // @DisplayName: Frame Class
    // @Description: Frame Class
    // @Values: 0:Undefined,1:Rover,2:Boat,3:BalanceBot
    // @User: Standard
    AP_GROUPINFO("FRAME_CLASS", 16, ParametersG2, frame_class, 1),

#if HAL_PROXIMITY_ENABLED
    // @Group: PRX
    // @Path: ../libraries/AP_Proximity/AP_Proximity.cpp
    AP_SUBGROUPINFO(proximity, "PRX", 18, ParametersG2, AP_Proximity),
#endif

#if AP_AVOIDANCE_ENABLED
    // @Group: AVOID_
    // @Path: ../libraries/AC_Avoidance/AC_Avoid.cpp
    AP_SUBGROUPINFO(avoid, "AVOID_", 19, ParametersG2, AC_Avoid),
#endif

    // 20 was PIVOT_TURN_RATE and should not be re-used

    // ========================================
    // Safety and Crash Detection Parameters
    // ========================================
    // Balance bot specific parameters and crash angle detection

    // @Param: BAL_PITCH_MAX
    // @DisplayName: BalanceBot Maximum Pitch
    // @Description: Pitch angle in degrees at 100% throttle
    // @Units: deg
    // @Range: 0 15
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("BAL_PITCH_MAX", 21, ParametersG2, bal_pitch_max, 10),

    // @Param: CRASH_ANGLE
    // @DisplayName: Crash Angle
    // @Description: Pitch/Roll angle limit in degrees for crash check. Zero disables check
    // @Units: deg
    // @Range: 0 60
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("CRASH_ANGLE", 22, ParametersG2, crash_angle, 0),

#if AP_FOLLOW_ENABLED
    // @Group: FOLL
    // @Path: ../libraries/AP_Follow/AP_Follow.cpp
    AP_SUBGROUPINFO(follow, "FOLL", 23, ParametersG2, AP_Follow),
#endif

    // @Param: FRAME_TYPE
    // @DisplayName: Frame Type
    // @Description: Frame Type
    // @Values: 0:Undefined,1:Omni3,2:OmniX,3:OmniPlus,4:Omni3Mecanum
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("FRAME_TYPE", 24, ParametersG2, frame_type, 0),

    // @Param: LOIT_TYPE
    // @DisplayName: Loiter type
    // @Description: Loiter behaviour when moving to the target point
    // @Values: 0:Forward or reverse to target point,1:Always face bow towards target point,2:Always face stern towards target point
    // @User: Standard
    AP_GROUPINFO("LOIT_TYPE", 25, ParametersG2, loit_type, 0),

#if HAL_SPRAYER_ENABLED
    // @Group: SPRAY_
    // @Path: ../libraries/AC_Sprayer/AC_Sprayer.cpp
    AP_SUBGROUPINFO(sprayer, "SPRAY_", 26, ParametersG2, AC_Sprayer),
#endif

    // @Group: WRC
    // @Path: ../libraries/AP_WheelEncoder/AP_WheelRateControl.cpp
    AP_SUBGROUPINFO(wheel_rate_control, "WRC", 27, ParametersG2, AP_WheelRateControl),

#if HAL_RALLY_ENABLED
    // @Group: RALLY_
    // @Path: AP_Rally.cpp,../libraries/AP_Rally/AP_Rally.cpp
    AP_SUBGROUPINFO(rally, "RALLY_", 28, ParametersG2, AP_Rally_Rover),
#endif

    // @Param: SIMPLE_TYPE
    // @DisplayName: Simple_Type
    // @Description: Simple mode types
    // @Values: 0:InitialHeading,1:CardinalDirections
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("SIMPLE_TYPE", 29, ParametersG2, simple_type, 0),

    // @Param: LOIT_RADIUS
    // @DisplayName: Loiter radius
    // @Description: Vehicle will drift when within this distance of the target position
    // @Units: m
    // @Range: 0 20
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("LOIT_RADIUS", 30, ParametersG2, loit_radius, 2),

    // @Group: WNDVN_
    // @Path: ../libraries/AP_WindVane/AP_WindVane.cpp
    AP_SUBGROUPINFO(windvane, "WNDVN_", 31, ParametersG2, AP_WindVane),

    // 32 to 36 were old sailboat params

    // 37 was airspeed

    // @Param: MIS_DONE_BEHAVE
    // @DisplayName: Mission done behave
    // @Description: Behaviour after mission completes
    // @Values: 0:Hold in Auto Mode,1:Loiter in Auto Mode,2:Acro Mode,3:Manual Mode
    // @User: Standard
    AP_GROUPINFO("MIS_DONE_BEHAVE", 38, ParametersG2, mis_done_behave, 0),

    // 39 was AP_Gripper

    // @Param: BAL_PITCH_TRIM
    // @DisplayName: Balance Bot pitch trim angle
    // @Description: Balance Bot pitch trim for balancing. This offsets the tilt of the center of mass.
    // @Units: deg
    // @Range: -2 2
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("BAL_PITCH_TRIM", 40, ParametersG2, bal_pitch_trim, 0),

    // 41 was Scripting

    // @Param: STICK_MIXING
    // @DisplayName: Stick Mixing
    // @Description: When enabled, this adds steering user stick input in auto modes, allowing the user to have some degree of control without changing modes.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("STICK_MIXING", 42, ParametersG2, stick_mixing, 0),

    // @Group: WP_
    // @Path: ../libraries/AR_WPNav/AR_WPNav.cpp
    AP_SUBGROUPINFO(wp_nav, "WP_", 43, ParametersG2, AR_WPNav_OA),

    // @Group: SAIL_
    // @Path: sailboat.cpp
    AP_SUBGROUPINFO(sailboat, "SAIL_", 44, ParametersG2, Sailboat),

#if AP_OAPATHPLANNER_ENABLED
    // @Group: OA_
    // @Path: ../libraries/AC_Avoidance/AP_OAPathPlanner.cpp
    AP_SUBGROUPINFO(oa, "OA_", 45, ParametersG2, AP_OAPathPlanner),
#endif

    // @Param: SPEED_MAX
    // @DisplayName: Speed maximum
    // @Description: Maximum speed vehicle can obtain at full throttle. If 0, it will be estimated based on CRUISE_SPEED and CRUISE_THROTTLE.
    // @Units: m/s
    // @Range: 0 30
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("SPEED_MAX", 46, ParametersG2, speed_max, 0.0f),

    // @Param: LOIT_SPEED_GAIN
    // @DisplayName: Loiter speed gain
    // @Description: Determines how aggressively LOITER tries to correct for drift from loiter point. Higher is faster but default should be acceptable.
    // @Range: 0 5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("LOIT_SPEED_GAIN", 47, ParametersG2, loiter_speed_gain, 0.5f),

    // @Param: FS_OPTIONS
    // @DisplayName: Failsafe Options
    // @Description: Bitmask to enable failsafe options
    // @Bitmask: 0:Failsafe enabled in Hold mode
    // @User: Advanced
    AP_GROUPINFO("FS_OPTIONS", 48, ParametersG2, fs_options, 0),

#if HAL_TORQEEDO_ENABLED
    // @Group: TRQ
    // @Path: ../libraries/AP_Torqeedo/AP_Torqeedo.cpp
    AP_SUBGROUPINFO(torqeedo, "TRQ", 49, ParametersG2, AP_Torqeedo),
#endif

    // @Group: PSC
    // @Path: ../libraries/APM_Control/AR_PosControl.cpp
    AP_SUBGROUPINFO(pos_control, "PSC", 51, ParametersG2, AR_PosControl),

    // @Param: GUID_OPTIONS
    // @DisplayName: Guided mode options
    // @Description: Options that can be applied to change guided mode behaviour
    // @Bitmask: 6:SCurves used for navigation
    // @User: Advanced
    AP_GROUPINFO("GUID_OPTIONS", 52, ParametersG2, guided_options, 0),

    // @Param: MANUAL_OPTIONS
    // @DisplayName: Manual mode options
    // @Description: Manual mode specific options
    // @Bitmask: 0:Enable steering speed scaling
    // @User: Advanced
    AP_GROUPINFO("MANUAL_OPTIONS", 53, ParametersG2, manual_options, 0),

#if MODE_DOCK_ENABLED
    // @Group: DOCK
    // @Path: mode_dock.cpp
    AP_SUBGROUPPTR(mode_dock_ptr, "DOCK", 54, ParametersG2, ModeDock),
#endif

    // @Param: MANUAL_STR_EXPO
    // @DisplayName: Manual Steering Expo
    // @Description: Manual steering expo to allow faster steering when stick at edges
    // @Values: 0:Disabled,0.1:Very Low,0.2:Low,0.3:Medium,0.4:High,0.5:Very High
    // @Range: -0.5 0.95
    // @User: Advanced
    AP_GROUPINFO("MANUAL_STR_EXPO", 55, ParametersG2, manual_steering_expo, 0),

    // @Param: FS_GCS_TIMEOUT
    // @DisplayName: GCS failsafe timeout
    // @Description: Timeout before triggering the GCS failsafe
    // @Units: s
    // @Range: 2 120
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("FS_GCS_TIMEOUT", 56, ParametersG2, fs_gcs_timeout, 5),

    // @Group: CIRC
    // @Path: mode_circle.cpp
    AP_SUBGROUPINFO(mode_circle, "CIRC", 57, ParametersG2, ModeCircle),

    AP_GROUPEND
};

// These auxiliary channel param descriptions are here so that users of beta Mission Planner (which uses the master branch as its source of descriptions)
// can get them.  These lines can be removed once Rover-3.6-beta testing begins or we improve the source of descriptions for GCSs.
//
// @Param: CH7_OPTION
// @DisplayName: Channel 7 option
// @Description: What to do use channel 7 for
// @Values: 0:Nothing,1:SaveWaypoint,2:LearnCruiseSpeed,3:ArmDisarm,4:Manual,5:Acro,6:Steering,7:Hold,8:Auto,9:RTL,10:SmartRTL,11:Guided,12:Loiter
// @User: Standard

// @Param: AUX_CH
// @DisplayName: Auxiliary switch channel
// @Description: RC Channel to use for auxiliary functions including saving waypoints
// @User: Advanced

// @Param: PIVOT_TURN_ANGLE
// @DisplayName: Pivot turn angle
// @Description: Navigation angle threshold in degrees to switch to pivot steering. This allows you to setup a skid steering rover to turn on the spot in auto mode when the angle it needs to turn it greater than this angle. An angle of zero means to disable pivot turning. Note that you will probably also want to set a low value for WP_RADIUS to get neat turns.
// @Units: deg
// @Range: 0 360
// @Increment: 1
// @User: Standard

// @Param: PIVOT_TURN_RATE
// @DisplayName: Pivot turn rate
// @Description: Desired pivot turn rate in deg/s.
// @Units: deg/s
// @Range: 0 360
// @Increment: 1
// @User: Standard

ParametersG2::ParametersG2(void)
    :
#if AP_ROVER_ADVANCED_FAILSAFE_ENABLED
    afs(),
#endif
#if AP_BEACON_ENABLED
    beacon(),
#endif
    wheel_rate_control(wheel_encoder),
    motors(wheel_rate_control),
    attitude_control(),
    smart_rtl(),
#if HAL_PROXIMITY_ENABLED
    proximity(),
#endif
#if MODE_DOCK_ENABLED
    mode_dock_ptr(&rover.mode_dock),
#endif
#if AP_AVOIDANCE_ENABLED
    avoid(),
#endif
#if AP_FOLLOW_ENABLED
    follow(),
#endif
    windvane(),
    wp_nav(attitude_control, pos_control),
    sailboat(),
    pos_control(attitude_control)
{
    AP_Param::setup_object_defaults(this, var_info);
}


/**
 * @brief Parameter name conversion table for backward compatibility
 * 
 * @details This table enables automatic migration of parameter values when parameter names
 *          change between firmware versions. During startup in load_parameters(), the
 *          AP_Param::convert_old_parameters() function scans EEPROM for old parameter names
 *          and copies their values to the new parameter names if the new parameter has not
 *          been set by the user.
 * 
 *          Conversion Process:
 *          1. load_parameters() calls AP_Param::convert_old_parameters() with this table
 *          2. For each entry, the system checks if the old parameter exists in EEPROM
 *          3. If found, and new parameter is not set, the value is copied to new parameter
 *          4. The new parameter is marked as set and saved to EEPROM
 *          5. User settings are preserved across firmware updates automatically
 * 
 *          Table Entry Format:
 *          { old_key, old_group_element, old_type, "NEW_NAME" }
 *          - old_key: k_param enum value from Parameters.h for the old parameter location
 *          - old_group_element: Index within the old var_info[] table (0 for top-level)
 *          - old_type: AP_PARAM_* type (INT8, INT16, INT32, FLOAT)
 *          - "NEW_NAME": New parameter name as a string
 * 
 *          This works even if the old parameter has been completely removed from var_info[],
 *          because the k_param enum keys are never reused. The old_key provides the EEPROM
 *          location where the old parameter was stored.
 * 
 * @note When renaming a parameter, always add an entry here to preserve user settings.
 * @note The old k_param enum value must remain in Parameters.h with a comment marking it as unused.
 * @note For parameters moved into sub-groups, old_group_element indicates the specific sub-parameter.
 * 
 * @warning Never remove entries from this table - old firmware may need them indefinitely.
 * @warning Always test parameter conversion with SITL before releasing firmware updates.
 * 
 * Examples:
 * - Simple rename: { k_param_battery_monitoring, 0, AP_PARAM_INT8, "BATT_MONITOR" }
 * - Move to group: { k_param_g2, 299, AP_PARAM_INT16, "WP_PIVOT_ANGLE" }
 *   (old_group_element 299 refers to position within old g2 structure)
 * 
 * Source: Rover/Parameters.cpp:721-765
 */
const AP_Param::ConversionInfo conversion_table[] = {
    // Battery parameter conversions - migrated to AP_BattMonitor library with BATT_ prefix
    { Parameters::k_param_battery_monitoring, 0,      AP_PARAM_INT8,  "BATT_MONITOR" },
    { Parameters::k_param_battery_volt_pin,   0,      AP_PARAM_INT8,  "BATT_VOLT_PIN" },
    { Parameters::k_param_battery_curr_pin,   0,      AP_PARAM_INT8,  "BATT_CURR_PIN" },
    { Parameters::k_param_volt_div_ratio,     0,      AP_PARAM_FLOAT, "BATT_VOLT_MULT" },
    { Parameters::k_param_curr_amp_per_volt,  0,      AP_PARAM_FLOAT, "BATT_AMP_PERVOLT" },
    { Parameters::k_param_pack_capacity,      0,      AP_PARAM_INT32, "BATT_CAPACITY" },
    
    // Serial port baud rate conversions - migrated to AP_SerialManager library
    { Parameters::k_param_serial0_baud,       0,      AP_PARAM_INT16, "SERIAL0_BAUD" },
    { Parameters::k_param_serial1_baud,       0,      AP_PARAM_INT16, "SERIAL1_BAUD" },
    { Parameters::k_param_serial2_baud,       0,      AP_PARAM_INT16, "SERIAL2_BAUD" },
    
    // Motor/throttle parameter conversions - migrated to MOT_ prefix in g2.motors
    { Parameters::k_param_throttle_min_old,   0,      AP_PARAM_INT8,  "MOT_THR_MIN" },
    { Parameters::k_param_throttle_max_old,   0,      AP_PARAM_INT8,  "MOT_THR_MAX" },
    // Sensor parameter conversions
    { Parameters::k_param_compass_enabled_deprecated,       0,      AP_PARAM_INT8, "COMPASS_ENABLE" },
    
    // Navigation parameter conversions - waypoint and pivot steering moved to WP_ namespace
    { Parameters::k_param_waypoint_radius_old,    0,  AP_PARAM_FLOAT,  "WP_RADIUS" },
    { Parameters::k_param_g2,               299,      AP_PARAM_INT16,  "WP_PIVOT_ANGLE" },
    { Parameters::k_param_g2,               363,      AP_PARAM_INT16,  "WP_PIVOT_RATE" },
    { Parameters::k_param_g2,               491,      AP_PARAM_FLOAT,  "WP_PIVOT_DELAY" },
    
    // Sailboat parameter conversions - consolidated into SAIL_ namespace in g2.sailboat
    { Parameters::k_param_g2,                32,      AP_PARAM_FLOAT,  "SAIL_ANGLE_MIN" },
    { Parameters::k_param_g2,                33,      AP_PARAM_FLOAT,  "SAIL_ANGLE_MAX" },
    { Parameters::k_param_g2,                34,      AP_PARAM_FLOAT,  "SAIL_ANGLE_IDEAL" },
    { Parameters::k_param_g2,                35,      AP_PARAM_FLOAT,  "SAIL_HEEL_MAX" },
    { Parameters::k_param_g2,                36,      AP_PARAM_FLOAT,  "SAIL_NO_GO_ANGLE" },
    
    // Arming and attitude control conversions
    { Parameters::k_param_arming,             2,     AP_PARAM_INT16,  "ARMING_CHECK" },
    { Parameters::k_param_turn_max_g_old,     0,     AP_PARAM_FLOAT,  "ATC_TURN_MAX_G" },
    
    // Proximity sensor conversions - old g2 sub-parameters moved to PRX1_ namespace
    { Parameters::k_param_g2,                82,     AP_PARAM_INT8 , "PRX1_TYPE" },
    { Parameters::k_param_g2,               146,     AP_PARAM_INT8 , "PRX1_ORIENT" },
    { Parameters::k_param_g2,               210,     AP_PARAM_INT16, "PRX1_YAW_CORR" },
    { Parameters::k_param_g2,               274,     AP_PARAM_INT16, "PRX1_IGN_ANG1" },
    { Parameters::k_param_g2,               338,     AP_PARAM_INT8,  "PRX1_IGN_WID1" },
    { Parameters::k_param_g2,               402,     AP_PARAM_INT16, "PRX1_IGN_ANG2" },
    { Parameters::k_param_g2,               466,     AP_PARAM_INT8,  "PRX1_IGN_WID2" },
    { Parameters::k_param_g2,               530,     AP_PARAM_INT16, "PRX1_IGN_ANG3" },
    { Parameters::k_param_g2,               594,     AP_PARAM_INT8,  "PRX1_IGN_WID3" },
    { Parameters::k_param_g2,               658,     AP_PARAM_INT16, "PRX1_IGN_ANG4" },
    { Parameters::k_param_g2,               722,     AP_PARAM_INT8,  "PRX1_IGN_WID4" },
    { Parameters::k_param_g2,               1234,    AP_PARAM_FLOAT, "PRX1_MIN" },
    { Parameters::k_param_g2,               1298,    AP_PARAM_FLOAT, "PRX1_MAX" },
    
    // Torqeedo motor controller conversions - consolidated into TRQ1_ namespace in g2.torqeedo
    { Parameters::k_param_g2,               113,     AP_PARAM_INT8, "TRQ1_TYPE" },
    { Parameters::k_param_g2,               177,     AP_PARAM_INT8, "TRQ1_ONOFF_PIN" },
    { Parameters::k_param_g2,               241,     AP_PARAM_INT8, "TRQ1_DE_PIN" },
    { Parameters::k_param_g2,               305,     AP_PARAM_INT16, "TRQ1_OPTIONS" },
    { Parameters::k_param_g2,               369,     AP_PARAM_INT8, "TRQ1_POWER" },
    { Parameters::k_param_g2,               433,     AP_PARAM_FLOAT, "TRQ1_SLEW_TIME" },
    { Parameters::k_param_g2,               497,     AP_PARAM_FLOAT, "TRQ1_DIR_DELAY" },
};


/**
 * @brief Load and initialize all Rover parameters from persistent storage
 * 
 * @details This function is called once during Rover initialization and performs the complete
 *          parameter loading sequence including:
 *          1. Loading base parameters from EEPROM via AP_Vehicle::load_parameters()
 *          2. Converting old parameter names to new names for backward compatibility
 *          3. Setting vehicle-specific frame type flags
 *          4. Initializing default servo channel functions
 *          5. Applying vehicle-specific parameter adjustments (e.g., balance bot crash angle)
 *          6. Handling complex parameter migrations that require special logic
 *          7. Converting legacy G2 objects to their new locations
 * 
 *          Parameter Loading Sequence:
 *          - Base parameters loaded and validated against format version
 *          - conversion_table applied to migrate renamed parameters
 *          - Frame type set to ROVER for parameter database
 *          - Default servo functions assigned (CH1=steering, CH3=throttle)
 *          - Vehicle-specific defaults applied based on frame configuration
 *          - Special case migrations handled (CH7_OPTION, WP_SPEED, etc.)
 *          - G2 object migrations for feature-specific parameters
 * 
 *          Thread Safety: This function is only called during initialization before the
 *          scheduler starts, so no locking is required.
 * 
 * @note This function must complete successfully before the vehicle can arm or operate.
 * @warning Parameter conversion errors are logged but do not prevent boot - verify logs
 *          after firmware updates to ensure settings migrated correctly.
 * 
 * @see conversion_table for parameter name migrations
 * @see AP_Vehicle::load_parameters() for base parameter loading
 * @see AP_Param::convert_old_parameters() for conversion mechanism
 * 
 * Source: Rover/Parameters.cpp:949-1074
 */
void Rover::load_parameters(void)
{
    // Load base parameters from EEPROM, checking format version for compatibility
    AP_Vehicle::load_parameters(g.format_version, Parameters::k_format_version);

    // Apply parameter name conversions for backward compatibility with older firmware versions
    // This preserves user settings when parameter names change across releases
    AP_Param::convert_old_parameters(&conversion_table[0], ARRAY_SIZE(conversion_table));

    // Set vehicle frame type to ROVER for the parameter system - this affects which
    // parameters are visible in ground control stations and parameter documentation
    AP_Param::set_frame_type_flags(AP_PARAM_FRAME_ROVER);

    // Set default servo channel assignments for typical rover configuration
    // CH_1 = steering (left/right control), CH_3 = throttle (forward/backward)
    // These can be overridden by user-configured SERVOx_FUNCTION parameters
    SRV_Channels::set_default_function(CH_1, SRV_Channel::k_steering);
    SRV_Channels::set_default_function(CH_3, SRV_Channel::k_throttle);

    // Apply vehicle-specific parameter defaults based on frame configuration
    // Balance bots need a higher crash detection angle due to their unstable equilibrium
    if (is_balancebot()) {
        g2.crash_angle.set_default(30);
    }

    // Upgrade legacy servo parameters to current format (handles pre-3.x parameter format)
    SRV_Channels::upgrade_parameters();

    // Special case: Convert CH7_OPTION to RC7_OPTION for Rover-3.4 to 3.5 upgrade
    // The parameter was renamed AND the option values were renumbered, so a direct
    // conversion is insufficient. This mapping translates old option values to new ones.
    // Old values: 0=None, 1=SaveWP, 2=LearnCruiseSpeed, 3-12 various aux functions
    // New values follow the standardized RCx_OPTION scheme shared across vehicle types
    const AP_Param::ConversionInfo ch7_option_info = { Parameters::k_param_ch7_option, 0, AP_PARAM_INT8, "RC7_OPTION" };
    AP_Int8 ch7_opt_old;
    if (AP_Param::find_old_parameter(&ch7_option_info, &ch7_opt_old)) {
        // Mapping table from old CH7_OPTION values to new RC7_OPTION values
        const uint8_t ch7_opt_map[] = {0,7,50,41,51,52,53,54,16,4,42,55,56};
        const uint8_t ch7_opt_old_val = (uint8_t)ch7_opt_old.get();
        if (ch7_opt_old_val < ARRAY_SIZE(ch7_opt_map)) {
            AP_Param::set_default_by_name(ch7_option_info.new_name, ch7_opt_map[ch7_opt_old_val]);
        }
    }

    // Special case: WP_SPEED parameter migration with fallback logic
    // WP_SPEED was moved into the AR_WPNav sub-group. This conversion handles two scenarios:
    // 1. If old WP_SPEED was explicitly set by user, migrate that value
    // 2. If old WP_SPEED was never set, use CRUISE_SPEED as the default for WP_SPEED
    // This ensures waypoint navigation speed is initialized sensibly for all users
    const AP_Param::ConversionInfo wp_speed_old_info = { Parameters::k_param_g2, 14, AP_PARAM_FLOAT, "WP_SPEED" };
    const AP_Param::ConversionInfo cruise_speed_info = { Parameters::k_param_speed_cruise, 0, AP_PARAM_FLOAT, "WP_SPEED" };
    AP_Float wp_speed_old;
    if (AP_Param::find_old_parameter(&wp_speed_old_info, &wp_speed_old)) {
        // Old WP_SPEED parameter value was explicitly set by user, so copy to new location
        AP_Param::convert_old_parameter(&wp_speed_old_info, 1.0f);
    } else {
        // Old WP_SPEED was never set, so use CRUISE_SPEED as default for new WP_SPEED
        // This maintains expected behavior where waypoint speed matches general cruise speed
        AP_Param::convert_old_parameter(&cruise_speed_info, 1.0f);
    }

    // Attitude control feedforward (FF) and filter (FILT) parameter conversions for Rover-3.6
    // These parameters were reorganized in the attitude control library with new naming
    // and moved locations. The conversions map old embedded indices to new parameter names:
    // - STR_RAT = Steering rate controller parameters
    // - SPEED = Speed controller parameters
    // - BAL = Balance bot controller parameters
    // - SAIL = Sailboat controller parameters
    // Each controller has FLTE (error filter) and FF (feedforward gain) parameters
    const AP_Param::ConversionInfo ff_and_filt_conversion_info[] = {
        { Parameters::k_param_g2, 24650, AP_PARAM_FLOAT, "ATC_STR_RAT_FLTE" },
        { Parameters::k_param_g2, 28746, AP_PARAM_FLOAT, "ATC_STR_RAT_FF" },
        { Parameters::k_param_g2, 24714, AP_PARAM_FLOAT, "ATC_SPEED_FLTE" },
        { Parameters::k_param_g2, 28810, AP_PARAM_FLOAT, "ATC_SPEED_FF" },
        { Parameters::k_param_g2, 25226, AP_PARAM_FLOAT, "ATC_BAL_FLTE" },
        { Parameters::k_param_g2, 29322, AP_PARAM_FLOAT, "ATC_BAL_FF" },
        { Parameters::k_param_g2, 25354, AP_PARAM_FLOAT, "ATC_SAIL_FLTE" },
        { Parameters::k_param_g2, 29450, AP_PARAM_FLOAT, "ATC_SAIL_FF" },
    };
    AP_Param::convert_old_parameters(&ff_and_filt_conversion_info[0], ARRAY_SIZE(ff_and_filt_conversion_info));

    // Configure hardware safety switch behavior for Rover (if board has one)
    // Rover allows motor stop/start via safety button while armed, unlike Copter which
    // disables this for flight safety. Ground vehicles can safely stop motors when armed.
    // BRD_SAFETYOPTION bits: BUTTON_ACTIVE_SAFETY_OFF | BUTTON_ACTIVE_SAFETY_ON | BUTTON_ACTIVE_ARMED
#if HAL_HAVE_SAFETY_SWITCH
    AP_Param::set_default_by_name("BRD_SAFETYOPTION", AP_BoardConfig::BOARD_SAFETY_OPTION_BUTTON_ACTIVE_SAFETY_OFF|
                                                      AP_BoardConfig::BOARD_SAFETY_OPTION_BUTTON_ACTIVE_SAFETY_ON|
                                                      AP_BoardConfig::BOARD_SAFETY_OPTION_BUTTON_ACTIVE_ARMED);
#endif

    // G2 object conversions: Migrate feature-specific parameter groups that were previously
    // embedded in ParametersG2 to their own top-level objects. This improves parameter
    // organization and allows libraries to manage their own parameters independently.
    // Required for conditional features that may be enabled/disabled at compile time.
#if AP_AIRSPEED_ENABLED | AP_AIS_ENABLED | AP_FENCE_ENABLED
    // Find G2's top-level parameter key needed for G2 sub-object conversions
    AP_Param::ConversionInfo info;
    if (!AP_Param::find_top_level_key_by_pointer(&g2, info.old_key)) {
        return;  // Conversion cannot proceed without G2 key - should not happen in normal operation
    }
#endif

    // G2 sub-object conversion table: Each entry specifies an object that was moved from
    // being a G2 sub-parameter to being a top-level parameter group. The number indicates
    // the old index within ParametersG2 where this object was located.
    static const AP_Param::G2ObjectConversion g2_conversions[] {
#if AP_AIRSPEED_ENABLED
        // PARAMETER_CONVERSION - Added: JAN-2022
        // Airspeed sensor parameters (ARSPD_*) moved from g2 to top-level
        { &airspeed, airspeed.var_info, 37 },
#endif
#if AP_AIS_ENABLED
        // PARAMETER_CONVERSION - Added: MAR-2022
        // AIS (Automatic Identification System) parameters moved from g2 to top-level
        { &ais, ais.var_info, 50 },
#endif
#if AP_FENCE_ENABLED
        // PARAMETER_CONVERSION - Added: Mar-2022
        // Geofencing parameters (FENCE_*) moved from g2 to top-level
        { &fence, fence.var_info, 17 },
#endif
#if AP_STATS_ENABLED
        // PARAMETER_CONVERSION - Added: Jan-2024 for Rover-4.6
        // Flight statistics parameters (STAT_*) moved from g2 to top-level
        { &stats, stats.var_info, 1 },
#endif
#if AP_SCRIPTING_ENABLED
        // PARAMETER_CONVERSION - Added: Jan-2024 for Rover-4.6
        // Lua scripting parameters (SCR_*) moved from g2 to top-level
        { &scripting, scripting.var_info, 41 },
#endif
#if AP_GRIPPER_ENABLED
        // PARAMETER_CONVERSION - Added: Feb-2024 for Copter-4.6
        // Gripper control parameters (GRIP_*) moved from g2 to top-level
        { &gripper, gripper.var_info, 39 },
#endif
    };

    // Execute all G2 object conversions defined above
    AP_Param::convert_g2_objects(&g2, g2_conversions, ARRAY_SIZE(g2_conversions));

    // PARAMETER_CONVERSION - Added: Feb-2024 for Rover-4.6
    // Logger parameters (LOG_*) conversion: Move from g.logger to top-level logger object
    // This allows the logging library to manage its own parameters independently
#if HAL_LOGGING_ENABLED
    AP_Param::convert_class(g.k_param_logger, &logger, logger.var_info, 0, true);
#endif

    // Top-level object conversions: Migrate entire parameter groups that were previously
    // embedded in the main Parameters structure to standalone top-level objects
    static const AP_Param::TopLevelObjectConversion toplevel_conversions[] {
#if AP_SERIALMANAGER_ENABLED
        // PARAMETER_CONVERSION - Added: Feb-2024 for Rover-4.6
        // Serial port parameters (SERIALx_*) moved from Parameters to SerialManager object
        { &serial_manager, serial_manager.var_info, Parameters::k_param_serial_manager_old },
#endif
    };

    // Execute all top-level object conversions
    AP_Param::convert_toplevel_objects(toplevel_conversions, ARRAY_SIZE(toplevel_conversions));

#if HAL_GCS_ENABLED
    // PARAMETER_CONVERSION - Added: Mar-2025 for ArduPilot-4.7
    // MAVLink/GCS parameter namespace consolidation: Move all MAVLink-related parameters
    // into a unified MAV_ namespace for better organization and discoverability
    // Old names: SYSID_THISMAV, SYSID_MYGCS, TELEM_DELAY
    // New names: MAV_SYSID, MAV_GCS_SYSID, MAV_TELEM_DELAY, MAV_OPTIONS
    {
        static const AP_Param::ConversionInfo gcs_conversion_info[] {
            { Parameters::k_param_sysid_this_mav_old, 0, AP_PARAM_INT16,  "MAV_SYSID" },
            { Parameters::k_param_sysid_my_gcs_old, 0, AP_PARAM_INT16, "MAV_GCS_SYSID" },
            { Parameters::k_param_g2,  2, AP_PARAM_INT8, "MAV_OPTIONS" },
            { Parameters::k_param_telem_delay_old,  0, AP_PARAM_INT8, "MAV_TELEM_DELAY" },
        };
        AP_Param::convert_old_parameters(&gcs_conversion_info[0], ARRAY_SIZE(gcs_conversion_info));
    }
#endif  // HAL_GCS_ENABLED

}
