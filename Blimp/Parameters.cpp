/**
 * @file Parameters.cpp
 * @brief Blimp vehicle parameter definitions and management
 * 
 * @details This file defines all configurable parameters for the Blimp (lighter-than-air) vehicle.
 *          Parameters are organized using the AP_Param system which provides:
 *          - Persistent storage in EEPROM/flash memory
 *          - Ground station access via MAVLink parameter protocol
 *          - Type-safe parameter definitions with metadata
 *          - Parameter migration/conversion between firmware versions
 *          
 *          The parameter system uses two main structures:
 *          1. var_info[] - Primary parameter table for the Blimp class
 *          2. ParametersG2::var_info[] - Secondary parameter group for newer parameters
 *          
 *          Parameter Organization:
 *          - GSCALAR: Single scalar parameters stored in Parameters.g
 *          - GOBJECT: Sub-object parameters (sensors, libraries)
 *          - GOBJECTN: Named sub-objects with custom names
 *          - GOBJECTPTR: Pointer to sub-objects
 *          
 *          Parameter Groups Include:
 *          - Flight control parameters (velocity/position limits, control modes)
 *          - Fin control parameters (motor/servo outputs for blimp fins)
 *          - PID controllers (velocity and position control for X, Y, Z, Yaw axes)
 *          - Failsafe parameters (GCS, throttle, EKF, crash detection)
 *          - Sensor configuration (GPS, compass, barometer, IMU)
 *          - Flight modes and RC input mapping
 *          - Logging and diagnostics configuration
 *          
 *          Blimp-Specific Parameters:
 *          - MAX_VEL_XY/Z/YAW: Maximum velocity limits for lighter-than-air dynamics
 *          - MAX_POS_XY/Z/YAW: Maximum position change rates
 *          - SIMPLE_MODE: Simplified position control for operator ease
 *          - DIS_MASK: Axis disable mask for testing/troubleshooting
 *          - PILOT_SPEED_DN: Descent rate limits for buoyant vehicles
 *          
 *          Parameter Persistence:
 *          - Parameters stored in non-volatile memory using AP_Param backend
 *          - EEPROM layout managed automatically with format versioning
 *          - FORMAT_VERSION tracks parameter table structure changes
 *          - load_parameters() handles migration from older firmware versions
 *          
 *          Parameter Naming Conventions:
 *          - Prefix indicates subsystem (e.g., VELXY_, POSZ_, FS_)
 *          - Suffixes indicate PID components (_P, _I, _D, _IMAX, _FF)
 *          - Filter parameters use _FLT* suffix (_FLTE, _FLTD, _FLTT)
 *          
 * @note This file should only contain parameter definitions and loading logic.
 *       No flight control logic should be implemented here.
 *       
 * @warning Modifying parameter names, types, or storage locations can break
 *          compatibility with saved configurations. Use parameter conversion
 *          logic in load_parameters() when making structural changes.
 * 
 * Source: Blimp/Parameters.cpp
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include "Blimp.h"

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define DEFAULT_FRAME_CLASS 0

/**
 * @brief Primary parameter table for Blimp vehicle
 * 
 * @details This var_info[] array defines all parameters for the Blimp class using the AP_Param system.
 *          Each parameter entry specifies:
 *          - Storage location (GSCALAR, GOBJECT, GOBJECTN, GOBJECTPTR)
 *          - Parameter name (string used in GCS and MAVLink)
 *          - Default value
 *          - Metadata (@Param, @DisplayName, @Description, @Units, @Range, etc.)
 *          
 *          Parameter Table Organization:
 *          
 *          1. System Configuration (lines 26-30):
 *             - FORMAT_VERSION: EEPROM format version for parameter migration
 *          
 *          2. Pilot Input Configuration (lines 36-98):
 *             - PILOT_THR_FILT: Throttle input filtering for smoother control
 *             - PILOT_THR_BHV: Throttle stick behavior options
 *             - THR_DZ: Throttle deadzone for altitude hold modes
 *          
 *          3. Failsafe Parameters (lines 54-189):
 *             - FS_GCS_ENABLE: Ground station communication loss handling
 *             - FS_THR_ENABLE/VALUE: RC transmitter failsafe configuration
 *             - FS_EKF_ACTION/THRESH: Navigation system failure handling
 *             - FS_CRASH_CHECK: Automatic crash detection and disarm
 *          
 *          4. Flight Mode Configuration (lines 100-149):
 *             - FLTMODE1-6: Flight modes assigned to RC channel positions
 *             - FLTMODE_CH: RC channel for flight mode selection (default: Channel 5)
 *             - INITIAL_MODE: Mode to start in on boot
 *          
 *          5. Vehicle Dynamics Limits (lines 191-231):
 *             - MAX_VEL_XY/Z/YAW: Maximum velocity limits (m/s, rad/s)
 *             - MAX_POS_XY/Z/YAW: Maximum position change rates (m/s, rad/s)
 *             - These limits are critical for lighter-than-air vehicle safety
 *          
 *          6. Control Configuration (lines 233-253):
 *             - SIMPLE_MODE: Simplified world-frame position control
 *             - DIS_MASK: Axis disable mask for testing
 *             - PID_DZ: Position controller deadzone
 *          
 *          7. Hardware Configuration (lines 255-262):
 *             - RC_SPEED: ESC/servo update rate (Hz)
 *          
 *          8. Sensor Objects (lines 266-309):
 *             - COMPASS_: Magnetometer configuration
 *             - INS: Inertial measurement unit configuration
 *             - AHRS_: Attitude and heading reference system
 *             - BARO: Barometer configuration
 *             - GPS: GPS receiver configuration
 *          
 *          9. Navigation Objects (lines 315-325):
 *             - EK2_: Extended Kalman Filter 2 (if available)
 *             - EK3_: Extended Kalman Filter 3 (if available)
 *          
 *          10. System Objects (lines 280-343):
 *              - BATT: Battery monitoring
 *              - BRD_: Board-specific configuration
 *              - CAN_: CAN bus manager
 *              - SCHED_: Task scheduler
 *              - NTF_: Notification system (LEDs, buzzers)
 *          
 *          11. Fin Control Object (lines 341-343):
 *              - FINS_: Motor and fin control for blimp propulsion
 *          
 *          12. Velocity PID Controllers (lines 345-489):
 *              - VELXY_: Horizontal (XY) velocity controller (AC_PID_2D)
 *              - VELZ_: Vertical (Z) velocity controller (AC_PID_Basic)
 *              - VELYAW_: Yaw velocity controller (AC_PID_Basic)
 *              Each controller includes: P, I, D, IMAX, FF, FLTE, FLTD parameters
 *          
 *          13. Position PID Controllers (lines 491-687):
 *              - POSXY_: Horizontal (XY) position controller (AC_PID_2D)
 *              - POSZ_: Vertical (Z) position controller (AC_PID_Basic)
 *              - POSYAW_: Yaw position controller (AC_PID with advanced features)
 *              Each controller converts position error to target velocity
 *          
 *          14. Communication and Logging (lines 151-156, 694-696):
 *              - LOG_BITMASK: Selects which data to log
 *              - GCS_PID_MASK: PIDs to send to ground station for tuning
 *              - MAV: Ground control station MAVLink configuration
 *          
 *          15. Secondary Parameter Group (lines 337-339):
 *              - g2: ParametersG2 object for newer parameters (see below)
 *          
 *          Parameter Storage and Access:
 *          - GSCALAR parameters stored directly in Parameters.g structure
 *          - GOBJECT parameters are separate objects with their own var_info tables
 *          - Parameters accessed via g.parameter_name or AP::library().parameter_name()
 *          - Nested parameter groups use dot notation in GCS (e.g., "COMPASS_DEV_ID")
 *          
 *          Adding New Parameters:
 *          1. Add to Parameters.h structure definition
 *          2. Add entry to this var_info[] table with metadata
 *          3. Use unique parameter name (check all vehicles for conflicts)
 *          4. Choose appropriate storage location (usually ParametersG2 for new params)
 *          5. Test parameter save/load and GCS access
 *          
 * @note The order of parameters in this table is critical for EEPROM compatibility.
 *       Inserting parameters in the middle breaks saved configurations. Add new
 *       parameters to ParametersG2::var_info[] instead.
 *       
 * @warning Never remove or reorder existing parameters without providing conversion
 *          logic in load_parameters(). Users will lose their configurations.
 *          
 * @see ParametersG2::var_info[] for secondary parameter group
 * @see Blimp::load_parameters() for parameter loading and migration logic
 * @see libraries/AP_Param/AP_Param.h for parameter system documentation
 */
const AP_Param::Info Blimp::var_info[] = {
    // @Param: FORMAT_VERSION
    // @DisplayName: Eeprom format version number
    // @Description: This value is incremented when changes are made to the eeprom format
    // @User: Advanced
    GSCALAR(format_version, "FORMAT_VERSION",   0),

    // SYSID_THISMAV was here

    // SYSID_MYGCS was here

    // @Param: PILOT_THR_FILT
    // @DisplayName: Throttle filter cutoff
    // @Description: Throttle filter cutoff (Hz) - active whenever altitude control is inactive - 0 to disable
    // @User: Advanced
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 0.5
    GSCALAR(throttle_filt,  "PILOT_THR_FILT",     0),

    // @Param: PILOT_THR_BHV
    // @DisplayName: Throttle stick behavior
    // @Description: Bitmask containing various throttle stick options. TX with sprung throttle can set PILOT_THR_BHV to "1" so motor feedback when landed starts from mid-stick instead of bottom of stick.
    // @User: Standard
    // @Bitmask: 0:Feedback from mid stick,1:High throttle cancels landing,2:Disarm on land detection
    GSCALAR(throttle_behavior, "PILOT_THR_BHV", 0),

    // SerialManager was here

    // @Param: GCS_PID_MASK
    // @DisplayName: GCS PID tuning mask
    // @Description: bitmask of PIDs to send MAVLink PID_TUNING messages for
    // @User: Advanced
    // @Bitmask: 0:VELX,1:VELY,2:VELZ,3:VELYAW,4:POSX,5:POSY,6:POZ,7:POSYAW
    GSCALAR(gcs_pid_mask, "GCS_PID_MASK",     0),

    // @Param: FS_GCS_ENABLE
    // @DisplayName: Ground Station Failsafe Enable
    // @Description: Controls whether failsafe will be invoked (and what action to take) when connection with Ground station is lost for at least 5 seconds. See FS_OPTIONS param for additional actions, or for cases allowing Mission continuation, when GCS failsafe is enabled.
    // @Values: 0:Disabled/NoAction,5:Land
    // @User: Standard
    GSCALAR(failsafe_gcs, "FS_GCS_ENABLE", FS_GCS_DISABLED),

    // @Param: GPS_HDOP_GOOD
    // @DisplayName: GPS Hdop Good
    // @Description: GPS Hdop value at or below this value represent a good position.  Used for pre-arm checks
    // @Range: 100 900
    // @User: Advanced
    GSCALAR(gps_hdop_good, "GPS_HDOP_GOOD", GPS_HDOP_GOOD_DEFAULT),

    // @Param: FS_THR_ENABLE
    // @DisplayName: Throttle Failsafe Enable
    // @Description: The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel
    // @Values:  0:Disabled,3:Enabled always Land
    // @User: Standard
    GSCALAR(failsafe_throttle,  "FS_THR_ENABLE",   FS_THR_ENABLED_ALWAYS_RTL),

    // @Param: FS_THR_VALUE
    // @DisplayName: Throttle Failsafe Value
    // @Description: The PWM level in microseconds on channel 3 below which throttle failsafe triggers
    // @Range: 910 1100
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    GSCALAR(failsafe_throttle_value, "FS_THR_VALUE",      FS_THR_VALUE_DEFAULT),

    // @Param: THR_DZ
    // @DisplayName: Throttle deadzone
    // @Description: The deadzone above and below mid throttle in PWM microseconds. Used in AltHold, Loiter, PosHold flight modes
    // @User: Standard
    // @Range: 0 300
    // @Units: PWM
    // @Increment: 1
    GSCALAR(throttle_deadzone,  "THR_DZ",    THR_DZ_DEFAULT),

    // @Param: FLTMODE1
    // @DisplayName: Flight Mode 1
    // @Description: Flight mode when Channel 5 pwm is <= 1230
    // @Values: 0:LAND,1:MANUAL,2:VELOCITY,3:LOITER
    // @User: Standard
    GSCALAR(flight_mode1, "FLTMODE1",               (uint8_t)FLIGHT_MODE_1),

    // @Param: FLTMODE2
    // @CopyFieldsFrom: FLTMODE1
    // @DisplayName: Flight Mode 2
    // @Description: Flight mode when Channel 5 pwm is >1230, <= 1360
    GSCALAR(flight_mode2, "FLTMODE2",               (uint8_t)FLIGHT_MODE_2),

    // @Param: FLTMODE3
    // @CopyFieldsFrom: FLTMODE1
    // @DisplayName: Flight Mode 3
    // @Description: Flight mode when Channel 5 pwm is >1360, <= 1490
    GSCALAR(flight_mode3, "FLTMODE3",               (uint8_t)FLIGHT_MODE_3),

    // @Param: FLTMODE4
    // @CopyFieldsFrom: FLTMODE1
    // @DisplayName: Flight Mode 4
    // @Description: Flight mode when Channel 5 pwm is >1490, <= 1620
    GSCALAR(flight_mode4, "FLTMODE4",               (uint8_t)FLIGHT_MODE_4),

    // @Param: FLTMODE5
    // @CopyFieldsFrom: FLTMODE1
    // @DisplayName: Flight Mode 5
    // @Description: Flight mode when Channel 5 pwm is >1620, <= 1749
    GSCALAR(flight_mode5, "FLTMODE5",               (uint8_t)FLIGHT_MODE_5),

    // @Param: FLTMODE6
    // @CopyFieldsFrom: FLTMODE1
    // @DisplayName: Flight Mode 6
    // @Description: Flight mode when Channel 5 pwm is >=1750
    GSCALAR(flight_mode6, "FLTMODE6",               (uint8_t)FLIGHT_MODE_6),

    // @Param: FLTMODE_CH
    // @DisplayName: Flightmode channel
    // @Description: RC Channel to use for flight mode control
    // @Values: 0:Disabled,5:Channel5,6:Channel6,7:Channel7,8:Channel8
    // @User: Advanced
    GSCALAR(flight_mode_chan, "FLTMODE_CH",         CH_MODE_DEFAULT),

    // @Param: INITIAL_MODE
    // @DisplayName: Initial flight mode
    // @Description: This selects the mode to start in on boot.
    // @CopyValuesFrom: FLTMODE1
    // @User: Advanced
    GSCALAR(initial_mode,        "INITIAL_MODE",     (uint8_t)Mode::Number::MANUAL),

    // @Param: LOG_BITMASK
    // @DisplayName: Log bitmask
    // @Description: Bitmap of what log types to enable in on-board logger. This value is made up of the sum of each of the log types you want to be saved. On boards supporting microSD cards or other large block-storage devices it is usually best just to enable all basic log types by setting this to 65535.
    // @Bitmask: 0:Fast Attitude,1:Medium Attitude,2:GPS,3:System Performance,4:Control Tuning,6:RC Input,7:IMU,9:Battery Monitor,10:RC Output,12:PID,13:Compass
    // @User: Standard
    GSCALAR(log_bitmask,    "LOG_BITMASK",          DEFAULT_LOG_BITMASK),

    // @Group: ARMING_
    // @Path: ../libraries/AP_Arming/AP_Arming.cpp
    GOBJECT(arming,                 "ARMING_", AP_Arming_Blimp),

    // @Param: DISARM_DELAY
    // @DisplayName: Disarm delay
    // @Description: Delay before automatic disarm in seconds. A value of zero disables auto disarm.
    // @Units: s
    // @Range: 0 127
    // @User: Advanced
    GSCALAR(disarm_delay, "DISARM_DELAY",           AUTO_DISARMING_DELAY),

    // @Param: FS_EKF_ACTION
    // @DisplayName: EKF Failsafe Action
    // @Description: Controls the action that will be taken when an EKF failsafe is invoked
    // @Values: 1:Land, 3:Land even in MANUAL
    // @User: Advanced
    GSCALAR(fs_ekf_action, "FS_EKF_ACTION",    FS_EKF_ACTION_DEFAULT),

    // @Param: FS_EKF_THRESH
    // @DisplayName: EKF failsafe variance threshold
    // @Description: Allows setting the maximum acceptable compass and velocity variance
    // @Values: 0.6:Strict, 0.8:Default, 1.0:Relaxed
    // @User: Advanced
    GSCALAR(fs_ekf_thresh, "FS_EKF_THRESH",    FS_EKF_THRESHOLD_DEFAULT),

    // @Param: FS_CRASH_CHECK
    // @DisplayName: Crash check enable
    // @Description: This enables automatic crash checking. When enabled the motors will disarm if a crash is detected.
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    GSCALAR(fs_crash_check, "FS_CRASH_CHECK",    1),

    // @Param: MAX_VEL_XY
    // @DisplayName: Max XY Velocity
    // @Description: Sets the maximum XY velocity, in m/s
    // @Range: 0.2 5
    // @User: Standard
    GSCALAR(max_vel_xy, "MAX_VEL_XY", 0.5),

    // @Param: MAX_VEL_Z
    // @DisplayName: Max Z Velocity
    // @Description: Sets the maximum Z velocity, in m/s
    // @Range: 0.2 5
    // @User: Standard
    GSCALAR(max_vel_z, "MAX_VEL_Z", 0.4),

    // @Param: MAX_VEL_YAW
    // @DisplayName: Max yaw Velocity
    // @Description: Sets the maximum yaw velocity, in rad/s
    // @Range: 0.2 5
    // @User: Standard
    GSCALAR(max_vel_yaw, "MAX_VEL_YAW", 0.5),

    // @Param: MAX_POS_XY
    // @DisplayName: Max XY Position change
    // @Description: Sets the maximum XY position change, in m/s
    // @Range: 0.1 5
    // @User: Standard
    GSCALAR(max_pos_xy, "MAX_POS_XY", 0.2),

    // @Param: MAX_POS_Z
    // @DisplayName: Max Z Position change
    // @Description: Sets the maximum Z position change, in m/s
    // @Range: 0.1 5
    // @User: Standard
    GSCALAR(max_pos_z, "MAX_POS_Z", 0.15),

    // @Param: MAX_POS_YAW
    // @DisplayName: Max Yaw Position change
    // @Description: Sets the maximum Yaw position change, in rad/s
    // @Range: 0.1 5
    // @User: Standard
    GSCALAR(max_pos_yaw, "MAX_POS_YAW", 0.3),

    // @Param: SIMPLE_MODE
    // @DisplayName: Simple mode
    // @Description: Simple mode for Position control - "forward" moves blimp in +ve X direction world-frame
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    GSCALAR(simple_mode, "SIMPLE_MODE", 0),

    // @Param: DIS_MASK
    // @DisplayName: Disable output mask
    // @Description: Mask for disabling (setting to zero) one or more of the 4 output axis in mode Velocity or Loiter
    // @Bitmask: 0:Right,1:Front,2:Down,3:Yaw
    // @User: Standard
    GSCALAR(dis_mask, "DIS_MASK", 0),

    // @Param: PID_DZ
    // @DisplayName: Deadzone for the position PIDs
    // @Description: Output 0 thrust signal when blimp is within this distance (in meters) of the target position. Warning: If this param is greater than MAX_POS_XY param then the blimp won't move at all in the XY plane in Loiter mode as it does not allow more than a second's lag. Same for the other axes.
    // @Units: m
    // @Range: 0.1 1
    // @User: Standard
    GSCALAR(pid_dz, "PID_DZ", 0),

    // @Param: RC_SPEED
    // @DisplayName: ESC Update Speed
    // @Description: This is the speed in Hertz that your ESCs will receive updates
    // @Units: Hz
    // @Range: 50 490
    // @Increment: 1
    // @User: Advanced
    GSCALAR(rc_speed, "RC_SPEED",              RC_FAST_SPEED),

    // variables not in the g class which contain EEPROM saved variables

    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/AP_Compass.cpp
    GOBJECT(compass,        "COMPASS_", Compass),

    // @Group: INS
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    GOBJECT(ins,            "INS", AP_InertialSensor),

    // SR0 through SR6 was here

    // @Group: AHRS_
    // @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs,                   "AHRS_",    AP_AHRS),

    // @Group: BATT
    // @Path: ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    GOBJECT(battery,                "BATT",         AP_BattMonitor),

    // @Group: BRD_
    // @Path: ../libraries/AP_BoardConfig/AP_BoardConfig.cpp
    GOBJECT(BoardConfig,            "BRD_",       AP_BoardConfig),

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    // @Group: CAN_
    // @Path: ../libraries/AP_CANManager/AP_CANManager.cpp
    GOBJECT(can_mgr,        "CAN_",       AP_CANManager),
#endif

#if AP_SIM_ENABLED
    GOBJECT(sitl, "SIM_", SITL::SIM),
#endif

    // @Group: BARO
    // @Path: ../libraries/AP_Baro/AP_Baro.cpp
    GOBJECT(barometer, "BARO", AP_Baro),

    // GPS driver
    // @Group: GPS
    // @Path: ../libraries/AP_GPS/AP_GPS.cpp
    GOBJECT(gps, "GPS", AP_GPS),

    // @Group: SCHED_
    // @Path: ../libraries/AP_Scheduler/AP_Scheduler.cpp
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),

    // @Group: RCMAP_
    // @Path: ../libraries/AP_RCMapper/AP_RCMapper.cpp
    GOBJECT(rcmap, "RCMAP_",        RCMapper),

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

#if AP_RSSI_ENABLED
    // @Group: RSSI_
    // @Path: ../libraries/AP_RSSI/AP_RSSI.cpp
    GOBJECT(rssi, "RSSI_",  AP_RSSI),
#endif

    // @Group: NTF_
    // @Path: ../libraries/AP_Notify/AP_Notify.cpp
    GOBJECT(notify, "NTF_",  AP_Notify),

    // @Group:
    // @Path: Parameters.cpp
    GOBJECT(g2, "",  ParametersG2),

    // @Group: FINS_
    // @Path: Fins.cpp
    GOBJECTPTR(motors, "FINS_", Fins),

    // @Param: VELXY_P
    // @DisplayName: Velocity (horizontal) P gain
    // @Description: Velocity (horizontal) P gain.  Converts the difference between desired and actual velocity to a target acceleration
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: VELXY_I
    // @DisplayName: Velocity (horizontal) I gain
    // @Description: Velocity (horizontal) I gain.  Corrects long-term difference between desired and actual velocity to a target acceleration
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: VELXY_D
    // @DisplayName: Velocity (horizontal) D gain
    // @Description: Velocity (horizontal) D gain.  Corrects short-term changes in velocity
    // @Range: 0.00 1.00
    // @Increment: 0.001
    // @User: Advanced

    // @Param: VELXY_IMAX
    // @DisplayName: Velocity (horizontal) integrator maximum
    // @Description: Velocity (horizontal) integrator maximum.  Constrains the target acceleration that the I gain will output
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cm/s/s
    // @User: Advanced

    // @Param: VELXY_FLTE
    // @DisplayName: Velocity (horizontal) input filter
    // @Description: Velocity (horizontal) input filter.  This filter (in Hz) is applied to the input for P and I terms
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: VELXY_FLTD
    // @DisplayName: Velocity (horizontal) input filter
    // @Description: Velocity (horizontal) input filter.  This filter (in Hz) is applied to the input for D term
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: VELXY_FF
    // @DisplayName: Velocity (horizontal) feed forward gain
    // @Description: Velocity (horizontal) feed forward gain.  Converts the difference between desired velocity to a target acceleration
    // @Range: 0 6
    // @Increment: 0.01
    // @User: Advanced
    GOBJECT(pid_vel_xy, "VELXY_", AC_PID_2D),

    // @Param: VELZ_P
    // @DisplayName: Velocity (vertical) P gain
    // @Description: Velocity (vertical) P gain.  Converts the difference between desired and actual velocity to a target acceleration
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: VELZ_I
    // @DisplayName: Velocity (vertical) I gain
    // @Description: Velocity (vertical) I gain.  Corrects long-term difference between desired and actual velocity to a target acceleration
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: VELZ_D
    // @DisplayName: Velocity (vertical) D gain
    // @Description: Velocity (vertical) D gain.  Corrects short-term changes in velocity
    // @Range: 0.00 1.00
    // @Increment: 0.001
    // @User: Advanced

    // @Param: VELZ_IMAX
    // @DisplayName: Velocity (vertical) integrator maximum
    // @Description: Velocity (vertical) integrator maximum.  Constrains the target acceleration that the I gain will output
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cm/s/s
    // @User: Advanced

    // @Param: VELZ_FLTE
    // @DisplayName: Velocity (vertical) input filter
    // @Description: Velocity (vertical) input filter.  This filter (in Hz) is applied to the input for P and I terms
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: VELZ_FLTD
    // @DisplayName: Velocity (vertical) input filter
    // @Description: Velocity (vertical) input filter.  This filter (in Hz) is applied to the input for D term
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: VELZ_FF
    // @DisplayName: Velocity (vertical) feed forward gain
    // @Description: Velocity (vertical) feed forward gain.  Converts the difference between desired velocity to a target acceleration
    // @Range: 0 6
    // @Increment: 0.01
    // @User: Advanced
    GOBJECT(pid_vel_z, "VELZ_", AC_PID_Basic),

    // @Param: VELYAW_P
    // @DisplayName: Velocity (yaw) P gain
    // @Description: Velocity (yaw) P gain.  Converts the difference between desired and actual velocity to a target acceleration
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: VELYAW_I
    // @DisplayName: Velocity (yaw) I gain
    // @Description: Velocity (yaw) I gain.  Corrects long-term difference between desired and actual velocity to a target acceleration
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: VELYAW_D
    // @DisplayName: Velocity (yaw) D gain
    // @Description: Velocity (yaw) D gain.  Corrects short-term changes in velocity
    // @Range: 0.00 1.00
    // @Increment: 0.001
    // @User: Advanced

    // @Param: VELYAW_IMAX
    // @DisplayName: Velocity (yaw) integrator maximum
    // @Description: Velocity (yaw) integrator maximum.  Constrains the target acceleration that the I gain will output
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cm/s/s
    // @User: Advanced

    // @Param: VELYAW_FLTE
    // @DisplayName: Velocity (yaw) input filter
    // @Description: Velocity (yaw) input filter.  This filter (in Hz) is applied to the input for P and I terms
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: VELYAW_FF
    // @DisplayName: Velocity (yaw) feed forward gain
    // @Description: Velocity (yaw) feed forward gain.  Converts the difference between desired velocity to a target acceleration
    // @Range: 0 6
    // @Increment: 0.01
    // @User: Advanced
    GOBJECT(pid_vel_yaw, "VELYAW_", AC_PID_Basic),

    // @Param: POSXY_P
    // @DisplayName: Position (horizontal) P gain
    // @Description: Position (horizontal) P gain.  Converts the difference between desired and actual position to a target velocity
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: POSXY_I
    // @DisplayName: Position (horizontal) I gain
    // @Description: Position (horizontal) I gain.  Corrects long-term difference between desired and actual position to a target velocity
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: POSXY_D
    // @DisplayName: Position (horizontal) D gain
    // @Description: Position (horizontal) D gain.  Corrects short-term changes in position
    // @Range: 0.00 1.00
    // @Increment: 0.001
    // @User: Advanced

    // @Param: POSXY_IMAX
    // @DisplayName: Position (horizontal) integrator maximum
    // @Description: Position (horizontal) integrator maximum.  Constrains the target acceleration that the I gain will output
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cm/s/s
    // @User: Advanced

    // @Param: POSXY_FLTE
    // @DisplayName: Position (horizontal) input filter
    // @Description: Position (horizontal) input filter.  This filter (in Hz) is applied to the input for P and I terms
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: POSXY_FLTD
    // @DisplayName: Position (horizontal) input filter
    // @Description: Position (horizontal) input filter.  This filter (in Hz) is applied to the input for D term
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: POSXY_FF
    // @DisplayName: Position (horizontal) feed forward gain
    // @Description: Position (horizontal) feed forward gain.  Converts the difference between desired position to a target velocity
    // @Range: 0 6
    // @Increment: 0.01
    // @User: Advanced
    GOBJECT(pid_pos_xy, "POSXY_", AC_PID_2D),

    // @Param: POSZ_P
    // @DisplayName: Position (vertical) P gain
    // @Description: Position (vertical) P gain.  Converts the difference between desired and actual position to a target velocity
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: POSZ_I
    // @DisplayName: Position (vertical) I gain
    // @Description: Position (vertical) I gain.  Corrects long-term difference between desired and actual position to a target velocity
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: POSZ_D
    // @DisplayName: Position (vertical) D gain
    // @Description: Position (vertical) D gain.  Corrects short-term changes in position
    // @Range: 0.00 1.00
    // @Increment: 0.001
    // @User: Advanced

    // @Param: POSZ_IMAX
    // @DisplayName: Position (vertical) integrator maximum
    // @Description: Position (vertical) integrator maximum.  Constrains the target acceleration that the I gain will output
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cm/s/s
    // @User: Advanced

    // @Param: POSZ_FLTE
    // @DisplayName: Position (vertical) input filter
    // @Description: Position (vertical) input filter.  This filter (in Hz) is applied to the input for P and I terms
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: POSZ_FLTD
    // @DisplayName: Position (vertical) input filter
    // @Description: Position (vertical) input filter.  This filter (in Hz) is applied to the input for D term
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: POSZ_FF
    // @DisplayName: Position (vertical) feed forward gain
    // @Description: Position (vertical) feed forward gain.  Converts the difference between desired position to a target velocity
    // @Range: 0 6
    // @Increment: 0.01
    // @User: Advanced
    GOBJECT(pid_pos_z, "POSZ_", AC_PID_Basic),

    // @Param: POSYAW_P
    // @DisplayName: Position (yaw) axis controller P gain
    // @Description: Position (yaw) axis controller P gain.
    // @Range: 0.0 3.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: POSYAW_I
    // @DisplayName: Position (yaw) axis controller I gain
    // @Description: Position (yaw) axis controller I gain.
    // @Range: 0.0 3.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: POSYAW_IMAX
    // @DisplayName: Position (yaw) axis controller I gain maximum
    // @Description: Position (yaw) axis controller I gain maximum.
    // @Range: 0 4000
    // @Increment: 10
    // @Units: d%
    // @User: Standard

    // @Param: POSYAW_D
    // @DisplayName: Position (yaw) axis controller D gain
    // @Description: Position (yaw) axis controller D gain.
    // @Range: 0.001 0.1
    // @Increment: 0.001
    // @User: Standard

    // @Param: POSYAW_FF
    // @DisplayName: Position (yaw) axis controller feed forward
    // @Description: Position (yaw) axis controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: POSYAW_FLTT
    // @DisplayName: Position (yaw) target frequency filter in Hz
    // @Description: Position (yaw) target frequency filter in Hz
    // @Range: 1 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: POSYAW_FLTE
    // @DisplayName: Position (yaw) error frequency filter in Hz
    // @Description: Position (yaw) error frequency filter in Hz
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: POSYAW_FLTD
    // @DisplayName: Position (yaw) derivative input filter in Hz
    // @Description: Position (yaw) derivative input filter in Hz
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: POSYAW_SMAX
    // @DisplayName: Yaw slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: POSYAW_PDMX
    // @DisplayName: Position (yaw) axis controller PD sum maximum
    // @Description: Position (yaw) axis controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 4000
    // @Increment: 10
    // @Units: d%
    // @User: Advanced

    // @Param: POSYAW_D_FF
    // @DisplayName: Position (yaw) Derivative FeedForward Gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.1
    // @Increment: 0.001
    // @User: Advanced

    // @Param: POSYAW_NTF
    // @DisplayName: Position (yaw) Target notch filter index
    // @Description: Position (yaw) Target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: POSYAW_NEF
    // @DisplayName: Position (yaw) Error notch filter index
    // @Description: Position (yaw) Error notch filter index
    // @Range: 1 8
    // @User: Advanced

    GOBJECT(pid_pos_yaw, "POSYAW_", AC_PID),

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
 * @brief Secondary parameter group for Blimp vehicle
 * 
 * @details ParametersG2 provides an extension mechanism for adding new parameters
 *          without breaking EEPROM compatibility with existing saved configurations.
 *          This "g2" parameter group was introduced to allow parameter additions
 *          after the primary var_info[] table was established.
 *          
 *          Purpose of ParametersG2:
 *          - Add new parameters without altering primary parameter table order
 *          - Maintain backward compatibility with saved configurations
 *          - Group related newer features together
 *          - Allow future expansion without EEPROM layout conflicts
 *          
 *          Parameter Slots in G2:
 *          Some slot numbers are used, others reserved or repurposed:
 *          - Slot 7: DEV_OPTIONS - Developer/debugging options bitmask
 *          - Slot 11: (was SYSID_ENFORCE - migrated to MAV_OPTIONS)
 *          - Slot 12: (was AP_Stats - moved to separate object)
 *          - Slot 15: FRAME_CLASS - Blimp frame configuration
 *          - Slot 16: SERVO - Servo channel configuration (SRV_Channels)
 *          - Slot 17: RC - RC input channel configuration (RC_Channels_Blimp)
 *          - Slot 24: PILOT_SPEED_DN - Descent rate limit
 *          - Slot 30: (was AP_Scripting - moved to separate object)
 *          - Slot 35: FS_VIBE_ENABLE - Vibration failsafe
 *          - Slot 36: FS_OPTIONS - Additional failsafe option flags
 *          - Slot 42: FS_GCS_TIMEOUT - GCS communication timeout
 *          
 *          G2 Parameters Defined Here:
 *          
 *          1. Development Options (DEV_OPTIONS):
 *             - Bitmask for experimental features and debugging
 *             - Meanings may change between versions
 *             - For developer use only
 *          
 *          2. Frame Configuration (FRAME_CLASS):
 *             - Defines blimp frame type (currently only Finnedblimp supported)
 *             - Requires reboot after change
 *             - Affects motor/fin mixing and control allocation
 *          
 *          3. Servo Configuration (SERVO):
 *             - Sub-group for SRV_Channel library parameters
 *             - Maps servo outputs to functions
 *             - Configures servo min/max/trim/reverse
 *          
 *          4. RC Channel Configuration (RC):
 *             - Sub-group for RC_Channels library parameters
 *             - Configures RC input channels and their functions
 *             - Includes expo, deadzone, and channel mapping
 *          
 *          5. Pilot Speed Controls (PILOT_SPEED_DN):
 *             - Descent rate limit for pilot control
 *             - Important for lighter-than-air vehicles due to buoyancy
 *             - Prevents over-rapid descents
 *          
 *          6. Failsafe Extensions (FS_VIBE_ENABLE, FS_OPTIONS, FS_GCS_TIMEOUT):
 *             - FS_VIBE_ENABLE: Vibration-based altitude estimation fallback
 *             - FS_OPTIONS: Bitmask for additional failsafe behaviors
 *             - FS_GCS_TIMEOUT: Customizable GCS communication timeout
 *          
 *          Parameter Slot Management:
 *          - Empty slots (commented out) reserve space for future use
 *          - Never reuse slots for different purposes (breaks compatibility)
 *          - Commented slots show migration history
 *          
 *          Adding Parameters to G2:
 *          1. Choose unused slot number
 *          2. Add to ParametersG2 class definition in Parameters.h
 *          3. Use AP_GROUPINFO or AP_SUBGROUPINFO macro
 *          4. Document slot number and parameter purpose
 *          5. If migrating from g, add conversion in load_parameters()
 *          
 * @note ParametersG2 is accessed via g2.parameter_name in code
 * @note When g2 slots are exhausted, create ParametersG3 following same pattern
 * 
 * @warning Never change slot numbers - this breaks saved parameter configurations
 * @warning Slot number comments (e.g., "11 was SYSID_ENFORCE") must be preserved
 *          to prevent accidental reuse
 * 
 * @see Blimp::var_info[] for primary parameter table
 * @see ParametersG2 class definition in Parameters.h
 * @see libraries/AP_Param/AP_Param.h for AP_GROUPINFO macro documentation
 */
const AP_Param::GroupInfo ParametersG2::var_info[] = {

    // @Param: DEV_OPTIONS
    // @DisplayName: Development options
    // @Description: Bitmask of developer options. The meanings of the bit fields in this parameter may vary at any time. Developers should check the source code for current meaning
    // @Bitmask: 0:Unknown
    // @User: Advanced
    AP_GROUPINFO("DEV_OPTIONS", 7, ParametersG2, dev_options, 0),

    // 11 was SYSID_ENFORCE

    // 12 was AP_Stats

    // @Param: FRAME_CLASS
    // @DisplayName: Frame Class
    // @Description: Controls major frame class for blimp.
    // @Values: 0:Finnedblimp
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("FRAME_CLASS", 15, ParametersG2, frame_class, DEFAULT_FRAME_CLASS),

    // @Group: SERVO
    // @Path: ../libraries/SRV_Channel/SRV_Channels.cpp
    AP_SUBGROUPINFO(servo_channels, "SERVO", 16, ParametersG2, SRV_Channels),

    // @Group: RC
    // @Path: ../libraries/RC_Channel/RC_Channels_VarInfo.h
    AP_SUBGROUPINFO(rc_channels, "RC", 17, ParametersG2, RC_Channels_Blimp),

    // @Param: PILOT_SPEED_DN
    // @DisplayName: Pilot maximum vertical speed descending
    // @Description: The maximum vertical descending velocity the pilot may request in cm/s
    // @Units: cm/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("PILOT_SPEED_DN", 24, ParametersG2, pilot_speed_dn, 0),

    // 30 was AP_Scripting

    // @Param: FS_VIBE_ENABLE
    // @DisplayName: Vibration Failsafe enable
    // @Description: This enables the vibration failsafe which will use modified altitude estimation and control during high vibrations
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    AP_GROUPINFO("FS_VIBE_ENABLE", 35, ParametersG2, fs_vibe_enabled, 1),

    // @Param: FS_OPTIONS
    // @DisplayName: Failsafe options bitmask
    // @Description: Bitmask of additional options for battery, radio, & GCS failsafes. 0 (default) disables all options.
    // @Bitmask: 4:Continue if in pilot controlled modes on GCS failsafe
    // @User: Advanced
    AP_GROUPINFO("FS_OPTIONS", 36, ParametersG2, fs_options, (float)Blimp::FailsafeOption::GCS_CONTINUE_IF_PILOT_CONTROL),

    // @Param: FS_GCS_TIMEOUT
    // @DisplayName: GCS failsafe timeout
    // @Description: Timeout before triggering the GCS failsafe
    // @Units: s
    // @Range: 2 120
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("FS_GCS_TIMEOUT", 42, ParametersG2, fs_gcs_timeout, 5),

    AP_GROUPEND
};

/**
 * @brief Constructor for ParametersG2 secondary parameter group
 * 
 * @details Initializes the ParametersG2 object by setting up default values
 *          for all parameters defined in the var_info[] table. This is called
 *          during Blimp object construction before parameters are loaded from
 *          persistent storage.
 *          
 *          Constructor Responsibilities:
 *          1. Calls AP_Param::setup_object_defaults() to initialize defaults
 *          2. Ensures all parameters have valid values before load
 *          3. Provides fallback values if EEPROM/flash is uninitialized
 *          
 *          Initialization Sequence:
 *          - Constructor sets defaults from var_info[] table
 *          - load_parameters() loads saved values from storage
 *          - Saved values override defaults if valid
 *          - Invalid/corrupted parameters revert to defaults
 *          
 * @note This constructor is called once during system initialization
 * @note Default values are specified in var_info[] parameter definitions
 * 
 * @see ParametersG2::var_info[] for parameter definitions and defaults
 * @see AP_Param::setup_object_defaults() for default value initialization
 * @see Blimp::load_parameters() for parameter loading from storage
 */
ParametersG2::ParametersG2(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/**
 * @brief Load parameters from persistent storage and perform version migration
 * 
 * @details This function loads all vehicle parameters from non-volatile storage
 *          (EEPROM or flash) and handles migration of parameters between firmware
 *          versions. It is called once during vehicle initialization.
 *          
 *          Loading Process:
 *          1. Load parameters using base class AP_Vehicle::load_parameters()
 *          2. Check format version (g.format_version vs k_format_version)
 *          3. Perform parameter conversions if format version changed
 *          4. Migrate parameters to new storage locations
 *          5. Set frame type flags for parameter system
 *          
 *          Parameter Conversion Types:
 *          
 *          1. G2 Object Conversions (g2_conversions[]):
 *             - Migrates parameters from g2 slots to separate top-level objects
 *             - Example: stats (slot 12) and scripting (slot 30) moved out of g2
 *             - Maintains parameter values during structural reorganization
 *             - Conditional compilation handles optional features
 *          
 *          2. Top-Level Object Conversions (toplevel_conversions[]):
 *             - Migrates entire parameter objects to new storage locations
 *             - Example: logger and serial_manager moved to top-level
 *             - Uses old parameter slot numbers (k_param_*) for lookup
 *             - Preserves all sub-parameters during migration
 *          
 *          3. Individual Parameter Conversions (gcs_conversion_info[]):
 *             - Renames individual parameters with new naming scheme
 *             - Example: SYSID_THIS_MAV → MAV_SYSID (Mar-2025, v4.7)
 *             - Maps old parameter names to new MAV_ namespace
 *             - Maintains parameter values across naming changes
 *          
 *          Parameter Migration Examples:
 *          
 *          Stats Library Migration (Jan-2024, Copter-4.6):
 *          - Moved from g2 slot 12 to separate top-level object
 *          - All STAT_* parameters preserved
 *          - Only executed if AP_STATS_ENABLED is defined
 *          
 *          Scripting Library Migration (Jan-2024, Copter-4.6):
 *          - Moved from g2 slot 30 to separate top-level object
 *          - All SCR_* parameters preserved
 *          - Only executed if AP_SCRIPTING_ENABLED is defined
 *          
 *          Logger Migration (Feb-2024):
 *          - Moved from Parameters::k_param_logger to top-level
 *          - All LOG_* parameters preserved
 *          - Only executed if HAL_LOGGING_ENABLED is defined
 *          
 *          Serial Manager Migration (Feb-2024):
 *          - Moved from k_param_serial_manager_old to top-level
 *          - All SERIAL* parameters preserved
 *          - Only executed if AP_SERIALMANAGER_ENABLED is defined
 *          
 *          MAVLink Parameter Namespace Migration (Mar-2025, v4.7):
 *          - SYSID_THIS_MAV → MAV_SYSID
 *          - SYSID_MY_GCS → MAV_GCS_SYSID
 *          - (unnamed g2.11) → MAV_OPTIONS
 *          - TELEM_DELAY → MAV_TELEM_DELAY
 *          - Consolidates GCS-related parameters under MAV_ prefix
 *          
 *          Frame Type Flags:
 *          - Sets AP_PARAM_FRAME_BLIMP flag for parameter system
 *          - Allows parameters to be vehicle-specific
 *          - Enables proper parameter filtering in ground stations
 *          
 *          Version Management:
 *          - g.format_version: Saved format version from EEPROM
 *          - k_format_version: Current firmware format version (Parameters.h)
 *          - If versions differ, conversions are applied
 *          - Format version updated after successful migration
 *          
 *          Error Handling:
 *          - Invalid parameters revert to defaults
 *          - Corrupted EEPROM triggers full reinitialization
 *          - Failed conversions logged but don't prevent boot
 *          - Missing parameters use default values from var_info[]
 *          
 * @note This function must be called after all subsystems are constructed
 *       but before they are initialized, as they depend on parameter values
 *       
 * @note Adding new parameter conversions requires:
 *       1. Increment k_format_version in Parameters.h
 *       2. Add conversion entry to appropriate array
 *       3. Document conversion with date and version comment
 *       4. Test migration with saved parameters from previous version
 *       
 * @warning Never remove parameter conversions - users may upgrade from old versions
 *          Keep conversion code indefinitely to support multi-version upgrades
 *          
 * @warning Incorrect conversion logic can cause loss of user configurations
 *          Always test parameter migration thoroughly before release
 * 
 * @see AP_Vehicle::load_parameters() for base loading functionality
 * @see AP_Param::convert_g2_objects() for g2 slot migration
 * @see AP_Param::convert_class() for library parameter migration  
 * @see AP_Param::convert_toplevel_objects() for top-level object migration
 * @see AP_Param::convert_old_parameters() for individual parameter renaming
 * @see Parameters.h for k_format_version and parameter structure definitions
 */
void Blimp::load_parameters(void)
{
    AP_Vehicle::load_parameters(g.format_version, Parameters::k_format_version);

    // Convert parameters that were moved from g2 secondary parameter group to separate objects
    // This maintains parameter values when reorganizing the parameter structure
    static const AP_Param::G2ObjectConversion g2_conversions[] {
#if AP_STATS_ENABLED
    // PARAMETER_CONVERSION - Added: Jan-2024 for Copter-4.6
    // Migrates stats parameters from g2 slot 12 to top-level stats object
    // Converts: g2.stats.* → stats.* (preserves all STAT_* parameter values)
        { &stats, stats.var_info, 12 },
#endif
#if AP_SCRIPTING_ENABLED
    // PARAMETER_CONVERSION - Added: Jan-2024 for Copter-4.6
    // Migrates scripting parameters from g2 slot 30 to top-level scripting object
    // Converts: g2.scripting.* → scripting.* (preserves all SCR_* parameter values)
        { &scripting, scripting.var_info, 30 },
#endif
    };
    AP_Param::convert_g2_objects(&g2, g2_conversions, ARRAY_SIZE(g2_conversions));

    // PARAMETER_CONVERSION - Added: Feb-2024
    // Migrates logger parameters from old storage location to top-level object
    // Converts: g.k_param_logger.* → logger.* (preserves all LOG_* parameters)
#if HAL_LOGGING_ENABLED
    AP_Param::convert_class(g.k_param_logger, &logger, logger.var_info, 0, true);
#endif

    // Convert entire parameter objects that were moved to top-level storage
    // This handles library parameters that were reorganized in the parameter table
    static const AP_Param::TopLevelObjectConversion toplevel_conversions[] {
#if AP_SERIALMANAGER_ENABLED
        // PARAMETER_CONVERSION - Added: Feb-2024
        // Migrates serial manager from old parameter slot to top-level object
        // Converts: k_param_serial_manager_old.* → serial_manager.* (preserves all SERIAL* parameters)
        { &serial_manager, serial_manager.var_info, Parameters::k_param_serial_manager_old },
#endif
    };

    AP_Param::convert_toplevel_objects(toplevel_conversions, ARRAY_SIZE(toplevel_conversions));

    // Setup AP_Param frame type flags to identify this as a Blimp vehicle
    // This allows parameters to be vehicle-specific and enables proper filtering
    // in ground control stations when displaying available parameters
    AP_Param::set_frame_type_flags(AP_PARAM_FRAME_BLIMP);

#if HAL_GCS_ENABLED
    // Move parameters into new MAV_ parameter namespace
    // PARAMETER_CONVERSION - Added: Mar-2025 for ArduPilot-4.7
    // This consolidates ground station and MAVLink-related parameters under a consistent prefix
    {
        static const AP_Param::ConversionInfo gcs_conversion_info[] {
            // Convert: SYSID_THIS_MAV → MAV_SYSID (vehicle's MAVLink system ID)
            { Parameters::k_param_sysid_this_mav_old, 0, AP_PARAM_INT16,  "MAV_SYSID" },
            // Convert: SYSID_MY_GCS → MAV_GCS_SYSID (ground station's MAVLink system ID)
            { Parameters::k_param_sysid_my_gcs_old, 0, AP_PARAM_INT16, "MAV_GCS_SYSID" },
            // Convert: g2 slot 11 → MAV_OPTIONS (MAVLink behavior options bitmask)
            { Parameters::k_param_g2,  11, AP_PARAM_INT8, "MAV_OPTIONS" },
            // Convert: TELEM_DELAY → MAV_TELEM_DELAY (telemetry startup delay)
            { Parameters::k_param_telem_delay_old,  0, AP_PARAM_INT8, "MAV_TELEM_DELAY" },
        };
        AP_Param::convert_old_parameters(&gcs_conversion_info[0], ARRAY_SIZE(gcs_conversion_info));
    }
#endif  // HAL_GCS_ENABLED

}
