#include "Copter.h"

#include <AP_Gripper/AP_Gripper.h>
#include <AP_InertialSensor/AP_InertialSensor_rate_config.h>

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

/**
 * @file Parameters.cpp
 * @brief ArduCopter parameter definitions and management
 * 
 * @details This file contains the complete parameter table for ArduCopter, defining all
 *          user-configurable parameters that control vehicle behavior. Parameters are
 *          stored in EEPROM for persistence across reboots and accessed through the
 *          AP_Param library system.
 * 
 * Parameter System Architecture:
 * - Parameters are organized into groups (var_info, var_info2) for EEPROM space management
 * - Each parameter has metadata defined via @Param tags for ground station display
 * - GSCALAR: Simple scalar parameters stored in main parameter structure
 * - GOBJECT/GOBJECTPTR: Nested parameter groups from library objects
 * - AP_Param library handles EEPROM persistence, default values, and parameter conversion
 * 
 * Parameter Groups:
 * - Copter::var_info: Primary parameter table with vehicle configuration
 * - ParametersG2::var_info: Secondary parameter table for additional parameters
 * - ParametersG2::var_info2: Extension table for overflow parameters
 * 
 * Parameter Documentation Tags:
 * - @Param: Parameter name as it appears in ground stations
 * - @DisplayName: Human-readable short name shown in GCS
 * - @Description: Detailed explanation of parameter function and behavior
 * - @Units: Physical units (Hz, cm, m/s, deg, etc.)
 * - @Range: Minimum and maximum valid values
 * - @Increment: Suggested step size for adjustments
 * - @Values: Enumerated valid values with descriptions
 * - @Bitmask: Bit field definitions for bitmask parameters
 * - @User: Access level (Standard, Advanced)
 * - @RebootRequired: Whether changing this parameter requires a reboot
 * 
 * Parameter Persistence:
 * - Parameters are automatically saved to EEPROM when changed via MAVLink
 * - FORMAT_VERSION parameter tracks EEPROM format for migration
 * - Parameter conversion functions handle format upgrades between versions
 * - Default values are applied on first boot or after EEPROM format changes
 * 
 * @note All parameter names must be unique and <= 16 characters
 * @warning Changing parameter definitions requires FORMAT_VERSION increment
 * @warning Parameter storage locations should not be reused to avoid conflicts
 * 
 * @see AP_Param library for parameter storage implementation
 * @see Copter::load_parameters() for parameter initialization
 * @see Parameters.h for parameter structure definitions
 */

// Frame class default selection based on vehicle configuration
#if FRAME_CONFIG == HELI_FRAME
// 6 here is AP_Motors::MOTOR_FRAME_HELI
#define DEFAULT_FRAME_CLASS 6
#else
#define DEFAULT_FRAME_CLASS 0
#endif

/**
 * Primary parameter table for ArduCopter
 * 
 * This table defines all vehicle parameters using the AP_Param system. Each entry
 * uses GSCALAR for simple parameters or GOBJECT/GOBJECTPTR for nested parameter groups.
 * 
 * Parameter Table Organization:
 * - Format and system parameters
 * - Pilot input parameters (throttle, takeoff, behavior)
 * - RTL (Return to Launch) parameters
 * - Failsafe configuration parameters
 * - Flight mode selection parameters
 * - Attitude and position control limits
 * - Subsystem objects (compass, GPS, motors, etc.)
 * 
 * Each parameter entry includes @Param tags that provide metadata for ground control stations.
 * These tags define display names, descriptions, valid ranges, units, and user access levels.
 * 
 * EEPROM Storage: Parameters are automatically persisted to EEPROM and restored on boot.
 * The AP_Param library manages storage allocation and parameter lookups by name.
 */
const AP_Param::Info Copter::var_info[] = {
    
    // ========================================================================
    // Format Version Parameter
    // ========================================================================
    // This parameter tracks the EEPROM format version. When parameter definitions
    // change significantly, this version is incremented to trigger parameter migration.
    // Users should not modify this value manually.
    
    // @Param: FORMAT_VERSION
    // @DisplayName: Eeprom format version number
    // @Description: This value is incremented when changes are made to the eeprom format
    // @User: Advanced
    GSCALAR(format_version, "FORMAT_VERSION",   0),

    // SYSID_THISMAV was here - moved to MAV_SYSID (see load_parameters for conversion)

    // SYSID_MYGCS was here - moved to MAV_GCS_SYSID (see load_parameters for conversion)

    // ========================================================================
    // Pilot Input Parameters
    // ========================================================================
    // These parameters control how pilot stick inputs are processed and filtered.
    // They affect throttle response, takeoff behavior, and pilot control characteristics.
    
    // @Param: PILOT_THR_FILT
    // @DisplayName: Throttle filter cutoff
    // @Description: Throttle filter cutoff (Hz) - active whenever altitude control is inactive - 0 to disable
    // @User: Advanced
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 0.5
    GSCALAR(throttle_filt,  "PILOT_THR_FILT",     0),

    // @Param: PILOT_TKOFF_ALT
    // @DisplayName: Pilot takeoff altitude
    // @Description: Altitude that altitude control modes will climb to when a takeoff is triggered with the throttle stick.
    // @User: Standard
    // @Units: cm
    // @Range: 0.0 1000.0
    // @Increment: 10
    GSCALAR(pilot_takeoff_alt,  "PILOT_TKOFF_ALT",  PILOT_TKOFF_ALT_DEFAULT),

    // @Param: PILOT_THR_BHV
    // @DisplayName: Throttle stick behavior
    // @Description: Bitmask containing various throttle stick options. TX with sprung throttle can set PILOT_THR_BHV to "1" so motor feedback when landed starts from mid-stick instead of bottom of stick.
    // @User: Standard
    // @Bitmask: 0:Feedback from mid stick,1:High throttle cancels landing,2:Disarm on land detection
    GSCALAR(throttle_behavior, "PILOT_THR_BHV", 0),

    // AP_SerialManager was here - moved to serial_manager object (see load_parameters for conversion)

    // ========================================================================
    // Ground Control Station (GCS) Parameters
    // ========================================================================
    // Parameters controlling telemetry and communication with ground control stations.
    
    // @Param: GCS_PID_MASK
    // @DisplayName: GCS PID tuning mask
    // @Description: bitmask of PIDs to send MAVLink PID_TUNING messages for
    // @User: Advanced
    // @Bitmask: 0:Roll,1:Pitch,2:Yaw,3:AccelZ
    GSCALAR(gcs_pid_mask,           "GCS_PID_MASK",     0),

#if MODE_RTL_ENABLED
    // ========================================================================
    // RTL (Return to Launch) Parameters
    // ========================================================================
    // These parameters control the vehicle's behavior when returning to the launch point.
    // RTL mode navigates back to home, climbs to a safe altitude, and performs landing.
    // The cone slope feature allows dynamic altitude adjustment based on distance from home.
    
    // @Param: RTL_ALT
    // @DisplayName: RTL Altitude
    // @Description: The minimum alt above home the vehicle will climb to before returning.  If the vehicle is flying higher than this value it will return at its current altitude.
    // @Units: cm
    // @Range: 30 300000
    // @Increment: 1
    // @User: Standard
    GSCALAR(rtl_altitude,   "RTL_ALT",     RTL_ALT),

    // @Param: RTL_CONE_SLOPE
    // @DisplayName: RTL cone slope
    // @Description: Defines a cone above home which determines maximum climb
    // @Range: 0.5 10.0
    // @Increment: 0.1
    // @Values: 0:Disabled,1:Shallow,3:Steep
    // @User: Standard
    GSCALAR(rtl_cone_slope,   "RTL_CONE_SLOPE",     RTL_CONE_SLOPE_DEFAULT),

    // @Param: RTL_SPEED
    // @DisplayName: RTL speed
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain horizontally while flying home. If this is set to zero, WPNAV_SPEED will be used instead.
    // @Units: cm/s
    // @Range: 0 2000
    // @Increment: 50
    // @User: Standard
    GSCALAR(rtl_speed_cms,   "RTL_SPEED",     0),

    // @Param: RTL_ALT_FINAL
    // @DisplayName: RTL Final Altitude
    // @Description: This is the altitude the vehicle will move to as the final stage of Returning to Launch or after completing a mission.  Set to zero to land.
    // @Units: cm
    // @Range: 0 1000
    // @Increment: 1
    // @User: Standard
    GSCALAR(rtl_alt_final,  "RTL_ALT_FINAL", RTL_ALT_FINAL),

    // @Param: RTL_CLIMB_MIN
    // @DisplayName: RTL minimum climb
    // @Description: The vehicle will climb this many cm during the initial climb portion of the RTL
    // @Units: cm
    // @Range: 0 3000
    // @Increment: 10
    // @User: Standard
    GSCALAR(rtl_climb_min,  "RTL_CLIMB_MIN",    RTL_CLIMB_MIN_DEFAULT),

    // @Param: RTL_LOIT_TIME
    // @DisplayName: RTL loiter time
    // @Description: Time (in milliseconds) to loiter above home before beginning final descent
    // @Units: ms
    // @Range: 0 60000
    // @Increment: 1000
    // @User: Standard
    GSCALAR(rtl_loiter_time,      "RTL_LOIT_TIME",    RTL_LOITER_TIME),

    // @Param: RTL_ALT_TYPE
    // @DisplayName: RTL mode altitude type
    // @Description: RTL altitude type.  Set to 1 for Terrain following during RTL and then set WPNAV_RFND_USE=1 to use rangefinder or WPNAV_RFND_USE=0 to use Terrain database
    // @Values: 0:Relative to Home, 1:Terrain
    // @User: Standard
    GSCALAR(rtl_alt_type, "RTL_ALT_TYPE", 0),
#endif

    // ========================================================================
    // Failsafe Parameters
    // ========================================================================
    // Failsafe parameters define vehicle responses to various failure conditions including
    // loss of RC link, GCS connection, GPS, battery depletion, and sensor failures.
    // These safety mechanisms help prevent crashes and loss of vehicle control.
    
    // @Param: FS_GCS_ENABLE
    // @DisplayName: Ground Station Failsafe Enable
    // @Description: Controls whether failsafe will be invoked (and what action to take) when connection with Ground station is lost for at least 5 seconds. See FS_OPTIONS param for additional actions, or for cases allowing Mission continuation, when GCS failsafe is enabled.
    // @Values: 0:Disabled/NoAction,1:RTL,2:RTL or Continue with Mission in Auto Mode (Removed in 4.0+-see FS_OPTIONS),3:SmartRTL or RTL,4:SmartRTL or Land,5:Land,6:Auto DO_LAND_START/DO_RETURN_PATH_START or RTL,7:Brake or Land
    // @User: Standard
    GSCALAR(failsafe_gcs, "FS_GCS_ENABLE", FS_GCS_DISABLED),

    // ========================================================================
    // GPS and Navigation Quality Parameters
    // ========================================================================
    // Parameters that define acceptable GPS and navigation quality metrics for flight.
    
    // @Param: GPS_HDOP_GOOD
    // @DisplayName: GPS Hdop Good
    // @Description: GPS Hdop value at or below this value represent a good position.  Used for pre-arm checks
    // @Range: 100 900
    // @User: Advanced
    GSCALAR(gps_hdop_good, "GPS_HDOP_GOOD", GPS_HDOP_GOOD_DEFAULT),

    // ========================================================================
    // Flight Assistance Parameters
    // ========================================================================
    // Parameters for pilot assist features like Super Simple mode that simplify flight control.
    
    // @Param: SUPER_SIMPLE
    // @DisplayName: Super Simple Mode
    // @Description: Bitmask to enable Super Simple mode for some flight modes. Setting this to Disabled(0) will disable Super Simple Mode. The bitmask is for flight mode switch positions
    // @Bitmask: 0:SwitchPos1, 1:SwitchPos2, 2:SwitchPos3, 3:SwitchPos4, 4:SwitchPos5, 5:SwitchPos6
    // @User: Standard
    GSCALAR(super_simple,   "SUPER_SIMPLE",     0),

    // ========================================================================
    // Waypoint Navigation Parameters
    // ========================================================================
    // Parameters controlling vehicle behavior during autonomous waypoint navigation.
    
    // @Param: WP_YAW_BEHAVIOR
    // @DisplayName: Yaw behaviour during missions
    // @Description: Determines how the autopilot controls the yaw during missions and RTL
    // @Values: 0:Never change yaw, 1:Face next waypoint, 2:Face next waypoint except RTL, 3:Face along GPS course
    // @User: Standard
    GSCALAR(wp_yaw_behavior,  "WP_YAW_BEHAVIOR",    WP_YAW_BEHAVIOR_DEFAULT),

    // ========================================================================
    // Landing Speed Parameters
    // ========================================================================
    // These parameters control descent rates during landing sequences.
    // Two-stage landing uses LAND_SPEED_HIGH initially, then LAND_SPEED for final touchdown.
    
    // @Param: LAND_SPEED
    // @DisplayName: Land speed
    // @Description: The descent speed for the final stage of landing in cm/s
    // @Units: cm/s
    // @Range: 30 200
    // @Increment: 10
    // @User: Standard
    GSCALAR(land_speed,             "LAND_SPEED",   LAND_SPEED),

    // @Param: LAND_SPEED_HIGH
    // @DisplayName: Land speed high
    // @Description: The descent speed for the first stage of landing in cm/s. If this is zero then WPNAV_SPEED_DN is used
    // @Units: cm/s
    // @Range: 0 500
    // @Increment: 10
    // @User: Standard
    GSCALAR(land_speed_high,        "LAND_SPEED_HIGH",   0),
    
    // ========================================================================
    // Pilot Vertical Velocity Parameters
    // ========================================================================
    // Maximum pilot-commanded vertical velocities and accelerations.
    // These limits apply in altitude-holding modes (AltHold, Loiter, PosHold, etc.).
    
    // @Param: PILOT_SPEED_UP
    // @DisplayName: Pilot maximum vertical speed ascending
    // @Description: The maximum vertical ascending velocity the pilot may request in cm/s
    // @Units: cm/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    GSCALAR(pilot_speed_up,     "PILOT_SPEED_UP",   PILOT_VELZ_MAX),

    // @Param: PILOT_ACCEL_Z
    // @DisplayName: Pilot vertical acceleration
    // @Description: The vertical acceleration used when pilot is controlling the altitude
    // @Units: cm/s/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    GSCALAR(pilot_accel_z,  "PILOT_ACCEL_Z",    PILOT_ACCEL_Z_DEFAULT),

    // ========================================================================
    // Throttle Failsafe Parameters
    // ========================================================================
    // RC throttle failsafe triggers when throttle PWM drops below FS_THR_VALUE.
    // This typically indicates RC link loss or transmitter power failure.
    // Failsafe action is configurable (RTL, Land, SmartRTL, etc.).
    
    // @Param: FS_THR_ENABLE
    // @DisplayName: Throttle Failsafe Enable
    // @Description: The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel
    // @Values:  0:Disabled,1:Enabled always RTL,2:Enabled Continue with Mission in Auto Mode (Removed in 4.0+),3:Enabled always Land,4:Enabled always SmartRTL or RTL,5:Enabled always SmartRTL or Land,6:Enabled Auto DO_LAND_START/DO_RETURN_PATH_START or RTL,7:Enabled always Brake or Land
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

    // ========================================================================
    // RC Input Parameters
    // ========================================================================
    // Parameters controlling RC (radio control) input processing and deadzones.
    
    // @Param: THR_DZ
    // @DisplayName: Throttle deadzone
    // @Description: The deadzone above and below mid throttle in PWM microseconds. Used in AltHold, Loiter, PosHold flight modes
    // @User: Standard
    // @Range: 0 300
    // @Units: PWM
    // @Increment: 1
    GSCALAR(throttle_deadzone,  "THR_DZ",    THR_DZ_DEFAULT),

    // ========================================================================
    // Flight Mode Selection Parameters
    // ========================================================================
    // These parameters map RC channel PWM ranges to specific flight modes.
    // FLTMODE_CH selects which RC channel is used for mode selection.
    // Six flight modes (FLTMODE1-6) are assigned to different PWM ranges on that channel.
    // PWM ranges: <=1230, 1231-1360, 1361-1490, 1491-1620, 1621-1749, >=1750
    
    // @Param: FLTMODE1
    // @DisplayName: Flight Mode 1
    // @Description: Flight mode when pwm of Flightmode channel(FLTMODE_CH) is <= 1230
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,9:Land,11:Drift,13:Sport,14:Flip,15:AutoTune,16:PosHold,17:Brake,18:Throw,19:Avoid_ADSB,20:Guided_NoGPS,21:Smart_RTL,22:FlowHold,23:Follow,24:ZigZag,25:SystemID,26:Heli_Autorotate,27:Auto RTL,28:Turtle
    // @User: Standard
    GSCALAR(flight_mode1, "FLTMODE1",               (uint8_t)FLIGHT_MODE_1),

    // @Param: FLTMODE2
    // @CopyFieldsFrom: FLTMODE1
    // @DisplayName: Flight Mode 2
    // @Description: Flight mode when pwm of Flightmode channel(FLTMODE_CH) is >1230, <= 1360
    GSCALAR(flight_mode2, "FLTMODE2",               (uint8_t)FLIGHT_MODE_2),

    // @Param: FLTMODE3
    // @CopyFieldsFrom: FLTMODE1
    // @DisplayName: Flight Mode 3
    // @Description: Flight mode when pwm of Flightmode channel(FLTMODE_CH) is >1360, <= 1490
    GSCALAR(flight_mode3, "FLTMODE3",               (uint8_t)FLIGHT_MODE_3),

    // @Param: FLTMODE4
    // @CopyFieldsFrom: FLTMODE1
    // @DisplayName: Flight Mode 4
    // @Description: Flight mode when pwm of Flightmode channel(FLTMODE_CH) is >1490, <= 1620
    GSCALAR(flight_mode4, "FLTMODE4",               (uint8_t)FLIGHT_MODE_4),

    // @Param: FLTMODE5
    // @CopyFieldsFrom: FLTMODE1
    // @DisplayName: Flight Mode 5
    // @Description: Flight mode when pwm of Flightmode channel(FLTMODE_CH) is >1620, <= 1749
    GSCALAR(flight_mode5, "FLTMODE5",               (uint8_t)FLIGHT_MODE_5),

    // @Param: FLTMODE6
    // @CopyFieldsFrom: FLTMODE1
    // @DisplayName: Flight Mode 6
    // @Description: Flight mode when pwm of Flightmode channel(FLTMODE_CH) is >=1750
    GSCALAR(flight_mode6, "FLTMODE6",               (uint8_t)FLIGHT_MODE_6),

    // @Param: FLTMODE_CH
    // @DisplayName: Flightmode channel
    // @Description: RC Channel to use for flight mode control
    // @Values: 0:Disabled,5:Channel5,6:Channel6,7:Channel7,8:Channel8,9:Channel9,10:Channel 10,11:Channel 11,12:Channel 12,13:Channel 13,14:Channel 14,15:Channel 15
    // @User: Advanced
    GSCALAR(flight_mode_chan, "FLTMODE_CH",         CH_MODE_DEFAULT),

    // INITIAL_MODE allows specifying startup mode when receiver is not available
    // or for autonomous applications that don't require pilot mode selection.
    
    // @Param: INITIAL_MODE
    // @DisplayName: Initial flight mode
    // @Description: This selects the mode to start in on boot. This is useful for when you want to start in AUTO mode on boot without a receiver.
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,9:Land,11:Drift,13:Sport,14:Flip,15:AutoTune,16:PosHold,17:Brake,18:Throw,19:Avoid_ADSB,20:Guided_NoGPS,21:Smart_RTL,22:FlowHold,23:Follow,24:ZigZag,25:SystemID,26:Heli_Autorotate
    // @User: Advanced
    GSCALAR(initial_mode,        "INITIAL_MODE",     (uint8_t)Mode::Number::STABILIZE),

    // SIMPLE mode bitmask enables simple/super-simple mode per flight mode switch position.
    // Simple mode transforms pilot inputs relative to vehicle's initial heading rather than current heading.
    
    // @Param: SIMPLE
    // @DisplayName: Simple mode bitmask
    // @Description: Bitmask which holds which flight modes use simple heading mode (eg bit 0 = 1 means Flight Mode 0 uses simple mode). The bitmask is for flightmode switch positions.
    // @Bitmask: 0:SwitchPos1, 1:SwitchPos2, 2:SwitchPos3, 3:SwitchPos4, 4:SwitchPos5, 5:SwitchPos6
    // @User: Advanced
    GSCALAR(simple_modes, "SIMPLE",                 0),

    // ========================================================================
    // Logging Parameters
    // ========================================================================
    // Control which data types are logged to onboard storage.
    // LOG_BITMASK is a bitmask where each bit enables a specific log type.
    
    // @Param: LOG_BITMASK
    // @DisplayName: Log bitmask
    // @Description: Bitmap of what on-board log types to enable. This value is made up of the sum of each of the log types you want to be saved. It is usually best just to enable all basiclog types by setting this to 65535. Note that if you want to reduce log sizes you should consider using LOG_FILE_RATEMAX instead of disabling logging items with this parameter.
    // @Bitmask: 0:Fast Attitude,1:Medium Attitude,2:GPS,3:System Performance,4:Control Tuning,5:Navigation Tuning,6:RC input,7:IMU,8:Mission Commands,9:Battery Monitor,10:RC output,11:Optical Flow,12:PID,13:Compass,15:Camera,17:Motors,18:Fast IMU,19:Raw IMU,20:Video Stabilization,21:Fast harmonic notch logging
    // @User: Standard
    GSCALAR(log_bitmask,    "LOG_BITMASK",          DEFAULT_LOG_BITMASK),

    // ========================================================================
    // ESC Calibration Parameter
    // ========================================================================
    // ESC calibration mode teaches ESCs the throttle range.
    // Normally set through GUI, not manually adjusted.
    
    // @Param: ESC_CALIBRATION
    // @DisplayName: ESC Calibration
    // @Description: Controls whether ArduCopter will enter ESC calibration on the next restart.  Do not adjust this parameter manually.
    // @User: Advanced
    // @Values: 0:Normal Start-up, 1:Start-up in ESC Calibration mode if throttle high, 2:Start-up in ESC Calibration mode regardless of throttle, 3:Start-up and automatically calibrate ESCs, 9:Disabled
    GSCALAR(esc_calibrate, "ESC_CALIBRATION",       0),

    // ========================================================================
    // In-Flight Tuning Parameter
    // ========================================================================
    // Allows real-time tuning of control parameters via RC transmitter knob.
    // Enables field tuning without ground station connection.
    
    // @Param: TUNE
    // @DisplayName: Tuning Parameter
    // @Description: Selects parameter (normally a PID gain) that is being tuned with an RC transmitter's knob. The RC input channel used is assigned by setting RCx_OPTION to 219.
    // @User: Standard
    // @Values: 0:None,1:Stab Roll/Pitch kP,4:Rate Roll/Pitch kP,5:Rate Roll/Pitch kI,21:Rate Roll/Pitch kD,3:Stab Yaw kP,6:Rate Yaw kP,26:Rate Yaw kD,56:Rate Yaw Filter,55:Motor Yaw Headroom,14:AltHold kP,7:Throttle Rate kP,34:Throttle Accel kP,35:Throttle Accel kI,36:Throttle Accel kD,12:Loiter Pos kP,22:Velocity XY kP,28:Velocity XY kI,10:WP Speed,25:Acro Roll/Pitch deg/s,40:Acro Yaw deg/s,45:RC Feel,13:Heli Ext Gyro,38:Declination,39:Circle Rate,46:Rate Pitch kP,47:Rate Pitch kI,48:Rate Pitch kD,49:Rate Roll kP,50:Rate Roll kI,51:Rate Roll kD,52:Rate Pitch FF,53:Rate Roll FF,54:Rate Yaw FF,58:SysID Magnitude,59:PSC Angle Max,60:Loiter Speed
    GSCALAR(rc_tuning_param, "TUNE",                   0),

    // ========================================================================
    // Frame Configuration Parameters
    // ========================================================================
    // FRAME_TYPE defines motor mixing geometry (X, +, etc.).
    // FRAME_CLASS (in g2) defines vehicle type (Quad, Hexa, Octa, etc.).
    
    // @Param: FRAME_TYPE
    // @DisplayName: Frame Type (+, X, V, etc)
    // @Description: Controls motor mixing for multicopters.  Not used for Tri or Traditional Helicopters.
    // @Values: 0:Plus, 1:X, 2:V, 3:H, 4:V-Tail, 5:A-Tail, 10:Y6B, 11:Y6F, 12:BetaFlightX, 13:DJIX, 14:ClockwiseX, 15: I, 18: BetaFlightXReversed, 19:Y4
    // @User: Standard
    // @RebootRequired: True
    GSCALAR(frame_type, "FRAME_TYPE", HAL_FRAME_TYPE_DEFAULT),

    // ========================================================================
    // Arming and Safety Parameters
    // ========================================================================
    // These parameters control pre-arm checks and automatic disarming.
    // Critical for ensuring vehicle is safe to fly before arming.
    
    // @Group: ARMING_
    // @Path: ../libraries/AP_Arming/AP_Arming.cpp
    GOBJECT(arming,                 "ARMING_", AP_Arming_Copter),

    // DISARM_DELAY prevents accidental motor shutdown during brief touchdowns.
    // After landing detection, motors remain armed for this duration.
    
    // @Param: DISARM_DELAY
    // @DisplayName: Disarm delay
    // @Description: Delay before automatic disarm in seconds after landing touchdown detection. A value of zero disables auto disarm. If Emergency Motor stop active, delay time is half this value.
    // @Units: s
    // @Range: 0 127
    // @User: Advanced
    GSCALAR(disarm_delay, "DISARM_DELAY",           AUTO_DISARMING_DELAY),
    
    // ========================================================================
    // Attitude Control Limits
    // ========================================================================
    // ANGLE_MAX limits maximum vehicle lean angle in all flight modes.
    // Critical safety parameter affecting stability and maneuverability.
    
    // @Param: ANGLE_MAX
    // @DisplayName: Angle Max
    // @Description: Maximum lean angle in all flight modes
    // @Units: cdeg
    // @Increment: 10
    // @Range: 1000 8000
    // @User: Advanced
    ASCALAR(angle_max, "ANGLE_MAX",                 DEFAULT_ANGLE_MAX),

    // ========================================================================
    // PosHold Mode Parameters
    // ========================================================================
    // PosHold mode-specific braking and rotation behavior.
    
#if MODE_POSHOLD_ENABLED
    // @Param: PHLD_BRAKE_RATE
    // @DisplayName: PosHold braking rate
    // @Description: PosHold flight mode's rotation rate during braking in deg/sec
    // @Units: deg/s
    // @Range: 4 12
    // @User: Advanced
    GSCALAR(poshold_brake_rate_degs, "PHLD_BRAKE_RATE",  POSHOLD_BRAKE_RATE_DEFAULT),

    // @Param: PHLD_BRAKE_ANGLE
    // @DisplayName: PosHold braking angle max
    // @Description: PosHold flight mode's max lean angle during braking in centi-degrees
    // @Units: cdeg
    // @Increment: 10
    // @Range: 2000 4500
    // @User: Advanced
    GSCALAR(poshold_brake_angle_max, "PHLD_BRAKE_ANGLE",  POSHOLD_BRAKE_ANGLE_DEFAULT),
#endif

    // ========================================================================
    // Landing Behavior Parameters
    // ========================================================================
    // Controls pilot input during autonomous landing sequences.
    
    // @Param: LAND_REPOSITION
    // @DisplayName: Land repositioning
    // @Description: Enables user input during LAND mode, the landing phase of RTL, and auto mode landings.
    // @Values: 0:No repositioning, 1:Repositioning
    // @User: Advanced
    GSCALAR(land_repositioning, "LAND_REPOSITION",     LAND_REPOSITION_DEFAULT),

    // ========================================================================
    // EKF Failsafe Parameters
    // ========================================================================
    // Navigation filter health monitoring and failsafe actions.
    // EKF monitors position/velocity estimation quality.
    
    // @Param: FS_EKF_ACTION
    // @DisplayName: EKF Failsafe Action
    // @Description: Controls the action that will be taken when an EKF failsafe is invoked
    // @Values: 0:Report only, 1:Switch to Land mode if current mode requires postion, 2:Switch to AltHold mode if current mode requires postion, 3:Switch to Land mode from all modes
    // @User: Advanced
    GSCALAR(fs_ekf_action, "FS_EKF_ACTION",    FS_EKF_ACTION_DEFAULT),

    // FS_EKF_THRESH sets the variance threshold for triggering EKF failsafe.
    // Lower values are stricter, higher values more permissive.
    
    // @Param: FS_EKF_THRESH
    // @DisplayName: EKF failsafe variance threshold
    // @Description: Allows setting the maximum acceptable compass, velocity, position and height variances. Used in arming check and EKF failsafe.
    // @Values: 0.6:Strict, 0.8:Default, 1.0:Relaxed
    // @User: Advanced
    GSCALAR(fs_ekf_thresh, "FS_EKF_THRESH",    FS_EKF_THRESHOLD_DEFAULT),

    // ========================================================================
    // Crash Detection Parameters
    // ========================================================================
    // Automatic crash detection monitors accelerations and attitude errors.
    
    // @Param: FS_CRASH_CHECK
    // @DisplayName: Crash check enable
    // @Description: This enables automatic crash checking. When enabled the motors will disarm if a crash is detected.
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    GSCALAR(fs_crash_check, "FS_CRASH_CHECK",    1),

    // ========================================================================
    // ESC Update Rate Parameters
    // ========================================================================
    // RC_SPEED controls PWM/digital protocol update rate to ESCs/servos.
    
    // @Param: RC_SPEED
    // @DisplayName: ESC Update Speed
    // @Description: This is the speed in Hertz that your ESCs will receive updates
    // @Units: Hz
    // @Range: 50 490
    // @Increment: 1
    // @User: Advanced
    GSCALAR(rc_speed, "RC_SPEED",              RC_FAST_SPEED),

    // ========================================================================
    // Acro and Sport Mode Parameters
    // ========================================================================
    // Manual flight mode settings for advanced users.
    // ACRO mode provides rate-based stabilization only.
    
#if MODE_ACRO_ENABLED || MODE_SPORT_ENABLED
    // ACRO_BAL parameters control the automatic leveling behavior in Acro/Sport.
    // Higher values cause faster return to level attitude when sticks centered.
    
    // @Param: ACRO_BAL_ROLL
    // @DisplayName: Acro Balance Roll
    // @Description: rate at which roll angle returns to level in acro and sport mode.  A higher value causes the vehicle to return to level faster. For helicopter sets the decay rate of the virtual flybar in the roll axis. A higher value causes faster decay of desired to actual attitude.
    // @Range: 0 3
    // @Increment: 0.1
    // @User: Advanced
    GSCALAR(acro_balance_roll,      "ACRO_BAL_ROLL",    ACRO_BALANCE_ROLL),

    // @Param: ACRO_BAL_PITCH
    // @DisplayName: Acro Balance Pitch
    // @Description: rate at which pitch angle returns to level in acro and sport mode.  A higher value causes the vehicle to return to level faster. For helicopter sets the decay rate of the virtual flybar in the pitch axis. A higher value causes faster decay of desired to actual attitude.
    // @Range: 0 3
    // @Increment: 0.1
    // @User: Advanced
    GSCALAR(acro_balance_pitch,     "ACRO_BAL_PITCH",   ACRO_BALANCE_PITCH),
#endif

    // ACRO_RP_EXPO moved to Command Model class (see ParametersG2)

#if MODE_ACRO_ENABLED
    // ACRO_TRAINER assists new pilots by providing optional leveling assistance.
    
    // @Param: ACRO_TRAINER
    // @DisplayName: Acro Trainer
    // @Description: Type of trainer used in acro mode
    // @Values: 0:Disabled,1:Leveling,2:Leveling and Limited
    // @User: Advanced
    GSCALAR(acro_trainer,   "ACRO_TRAINER",     (uint8_t)ModeAcro::Trainer::LIMITED),
#endif

    // ========================================================================
    // Library Parameter Groups (GOBJECT entries)
    // ========================================================================
    // The following section contains parameter groups for subsystems
    // implemented as library objects. Each GOBJECT links a library's
    // parameter table into the main vehicle parameter namespace.
    // Parameters are stored in EEPROM using the AP_Param system.
    
    // ========================================================================
    // Peripheral Device Parameters
    // ========================================================================
    
#if AP_CAMERA_ENABLED
    // Camera control and triggering system parameters.
    
    // @Group: CAM
    // @Path: ../libraries/AP_Camera/AP_Camera.cpp
    GOBJECT(camera, "CAM", AP_Camera),
#endif

#if AP_RELAY_ENABLED
    // Relay output control for various accessories (landing gear, camera, etc).
    
    // @Group: RELAY
    // @Path: ../libraries/AP_Relay/AP_Relay.cpp
    GOBJECT(relay,                  "RELAY", AP_Relay),
#endif

#if HAL_PARACHUTE_ENABLED
    // Parachute deployment system for emergency recovery.
    
    // @Group: CHUTE_
    // @Path: ../libraries/AP_Parachute/AP_Parachute.cpp
    GOBJECT(parachute, "CHUTE_", AP_Parachute),
#endif

#if AP_LANDINGGEAR_ENABLED
    // Retractable landing gear control system.
    
    // @Group: LGR_
    // @Path: ../libraries/AP_LandingGear/AP_LandingGear.cpp
    GOBJECT(landinggear,    "LGR_", AP_LandingGear),
#endif

#if FRAME_CONFIG == HELI_FRAME
    // Helicopter-specific input manager with swashplate mixing.
    
    // @Group: IM_
    // @Path: ../libraries/AC_InputManager/AC_InputManager_Heli.cpp
    GOBJECT(input_manager, "IM_", AC_InputManager_Heli),
#endif

    // ========================================================================
    // Core Sensor System Parameters
    // ========================================================================
    
    // Magnetometer calibration, declination, and motor interference compensation.
    
    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/AP_Compass.cpp
    GOBJECT(compass,        "COMPASS_", Compass),

    // Inertial sensor (IMU) configuration: gyro/accel calibration, filtering, and notch filters.
    
    // @Group: INS
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    GOBJECT(ins,            "INS", AP_InertialSensor),

    // ========================================================================
    // Navigation and Control System Parameters
    // ========================================================================
    
    // Waypoint navigation parameters: speed, acceleration, corner radius.
    
    // @Group: WPNAV_
    // @Path: ../libraries/AC_WPNav/AC_WPNav.cpp
    GOBJECTPTR(wp_nav, "WPNAV_",       AC_WPNav),

    // Loiter mode position hold parameters: brake rates, speed limits.
    
    // @Group: LOIT_
    // @Path: ../libraries/AC_WPNav/AC_Loiter.cpp
    GOBJECTPTR(loiter_nav, "LOIT_", AC_Loiter),

#if MODE_CIRCLE_ENABLED
    // Circle mode parameters: radius and rotation rate.
    
    // @Group: CIRCLE_
    // @Path: ../libraries/AC_WPNav/AC_Circle.cpp
    GOBJECTPTR(circle_nav, "CIRCLE_",  AC_Circle),
#endif

    // Attitude controller: PID gains for roll/pitch/yaw rate and angle control.
    // CRITICAL: These parameters directly affect flight stability and response.
    
    // @Group: ATC_
    // @Path: ../libraries/AC_AttitudeControl/AC_AttitudeControl.cpp,../libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp,../libraries/AC_AttitudeControl/AC_AttitudeControl_Heli.cpp
    GOBJECTVARPTR(attitude_control, "ATC_", &copter.attitude_control_var_info),

    // Position controller: velocity and acceleration control PID gains.
    // PSC (Position, Speed, Control) manages horizontal and vertical motion.
    
    // @Group: PSC
    // @Path: ../libraries/AC_AttitudeControl/AC_PosControl.cpp
    GOBJECTPTR(pos_control, "PSC", AC_PosControl),

    // SR0 through SR6 (telemetry stream rates) were here - moved to MAVLink library

    // AHRS (Attitude and Heading Reference System): EKF backend selection,
    // GPS/compass fusion, coordinate frame transformations.
    
    // @Group: AHRS_
    // @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs,                   "AHRS_",    AP_AHRS),

#if HAL_MOUNT_ENABLED
    // Gimbal mount control: pointing modes, stabilization, RC control.
    
    // @Group: MNT
    // @Path: ../libraries/AP_Mount/AP_Mount.cpp
    GOBJECT(camera_mount,           "MNT",  AP_Mount),
#endif

    // ========================================================================
    // Power and Hardware Configuration Parameters
    // ========================================================================
    
    // Battery monitoring: voltage/current sensing, capacity estimation, failsafe thresholds.
    
    // @Group: BATT
    // @Path: ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    GOBJECT(battery,                "BATT",         AP_BattMonitor),

    // Board-specific hardware configuration: safety switch, power brick settings,
    // IMU temperature calibration, board voltage monitoring.
    
    // @Group: BRD_
    // @Path: ../libraries/AP_BoardConfig/AP_BoardConfig.cpp
    GOBJECT(BoardConfig,            "BRD_",       AP_BoardConfig),

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    // CAN bus manager: DroneCAN/UAVCAN protocol configuration for CAN peripherals.
    
    // @Group: CAN_
    // @Path: ../libraries/AP_CANManager/AP_CANManager.cpp
    GOBJECT(can_mgr,        "CAN_",       AP_CANManager),
#endif

#if HAL_SPRAYER_ENABLED
    // Agricultural sprayer control system for crop spraying applications.
    
    // @Group: SPRAY_
    // @Path: ../libraries/AC_Sprayer/AC_Sprayer.cpp
    GOBJECT(sprayer,                "SPRAY_",       AC_Sprayer),
#endif

#if AP_SIM_ENABLED
    // SITL (Software In The Loop) simulation parameters: simulated sensors,
    // vehicle physics, wind, and failure injection for testing.
    
    // @Group: SIM_
    // @Path: ../libraries/SITL/SITL.cpp
    GOBJECT(sitl, "SIM_", SITL::SIM),
#endif

    // Barometer configuration: ground pressure calibration, temperature compensation.
    
    // @Group: BARO
    // @Path: ../libraries/AP_Baro/AP_Baro.cpp
    GOBJECT(barometer, "BARO", AP_Baro),

    // GPS receiver configuration: protocol selection, RTK settings, blending,
    // accuracy thresholds for navigation and arming checks.
    
    // @Group: GPS
    // @Path: ../libraries/AP_GPS/AP_GPS.cpp
    GOBJECT(gps, "GPS", AP_GPS),

    // Task scheduler configuration: loop rate control, timing budgets,
    // performance monitoring for real-time execution.
    
    // @Group: SCHED_
    // @Path: ../libraries/AP_Scheduler/AP_Scheduler.cpp
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),

    // ========================================================================
    // Safety and Avoidance System Parameters
    // ========================================================================
    
#if AP_AVOIDANCE_ENABLED
    // Obstacle avoidance using proximity sensors and ADS-B traffic data.
    
    // @Group: AVOID_
    // @Path: ../libraries/AC_Avoidance/AC_Avoid.cpp
    GOBJECT(avoid,      "AVOID_",   AC_Avoid),
#endif

#if HAL_RALLY_ENABLED
    // Rally point configuration: alternate RTL destinations for improved safety.
    
    // @Group: RALLY_
    // @Path: AP_Rally.cpp,../libraries/AP_Rally/AP_Rally.cpp
    GOBJECT(rally,      "RALLY_",   AP_Rally_Copter),
#endif

    // ========================================================================
    // Motor and Propulsion System Parameters
    // ========================================================================
    // Motor mixing, thrust curves, spin-when-armed, motor ordering.
    // CRITICAL: These parameters affect flight stability and safety.
    
#if FRAME_CONFIG == HELI_FRAME
    // Helicopter-specific parameters: swashplate servo positions, rotor speed governor,
    // tail rotor gain, collective pitch curves.
    
    // @Group: H_
    // @Path: ../libraries/AP_Motors/AP_MotorsHeli_Single.cpp,../libraries/AP_Motors/AP_MotorsHeli_Dual.cpp,../libraries/AP_Motors/AP_MotorsHeli.cpp
    GOBJECTVARPTR(motors, "H_",        &copter.motors_var_info),
#else
    // Multicopter motor parameters: frame type, motor ordering, thrust curve,
    // PWM output ranges, motor interlock, battery compensation.
    
    // @Group: MOT_
    // @Path: ../libraries/AP_Motors/AP_MotorsMulticopter.cpp
    GOBJECTVARPTR(motors, "MOT_",      &copter.motors_var_info),
#endif

    // ========================================================================
    // RC Input Configuration Parameters
    // ========================================================================
    
    // RC channel mapping: assigns physical RC channels to control functions
    // (roll, pitch, yaw, throttle). Allows non-standard TX configurations.
    
    // @Group: RCMAP_
    // @Path: ../libraries/AP_RCMapper/AP_RCMapper.cpp
    GOBJECT(rcmap, "RCMAP_",        RCMapper),

    // ========================================================================
    // Extended Kalman Filter (EKF) Parameters
    // ========================================================================
    // State estimation for position, velocity, and attitude using sensor fusion.
    // CRITICAL: These parameters directly affect navigation accuracy.
    
#if HAL_NAVEKF2_AVAILABLE
    // EKF2 (legacy): Extended Kalman Filter for navigation state estimation.
    // Uses GPS, IMU, barometer, compass, and optional visual odometry/beacon.
    
    // @Group: EK2_
    // @Path: ../libraries/AP_NavEKF2/AP_NavEKF2.cpp
    GOBJECTN(ahrs.EKF2, NavEKF2, "EK2_", NavEKF2),
#endif

#if HAL_NAVEKF3_AVAILABLE
    // EKF3 (current): Enhanced EKF with improved multi-IMU support, GSF yaw estimator,
    // better handling of magnetic interference and GPS glitches.
    
    // @Group: EK3_
    // @Path: ../libraries/AP_NavEKF3/AP_NavEKF3.cpp
    GOBJECTN(ahrs.EKF3, NavEKF3, "EK3_", NavEKF3),
#endif

    // ========================================================================
    // Mission and Autonomous Flight Parameters
    // ========================================================================
    
#if MODE_AUTO_ENABLED
    // Mission management: waypoint storage, command execution, resume behavior,
    // mission scripting, conditional commands, terrain following in missions.
    
    // @Group: MIS_
    // @Path: ../libraries/AP_Mission/AP_Mission.cpp
    GOBJECTN(mode_auto.mission, mission, "MIS_", AP_Mission),
#endif

    // ========================================================================
    // Sensor and Telemetry Systems
    // ========================================================================
    
#if AP_RSSI_ENABLED
    // RC signal strength monitoring: RSSI (Received Signal Strength Indicator)
    // from receiver or telemetry radio for link quality monitoring.
    
    // @Group: RSSI_
    // @Path: ../libraries/AP_RSSI/AP_RSSI.cpp
    GOBJECT(rssi, "RSSI_",  AP_RSSI),
#endif

#if AP_RANGEFINDER_ENABLED
    // Distance sensors: lidar, sonar, radar for terrain following, precision landing,
    // object detection. Supports multiple rangefinders with orientation settings.
    
    // @Group: RNGFND
    // @Path: ../libraries/AP_RangeFinder/AP_RangeFinder.cpp
    GOBJECT(rangefinder,   "RNGFND", RangeFinder),
#endif

#if AP_TERRAIN_AVAILABLE
    // Terrain database: stores elevation data for terrain following and
    // terrain-relative altitude control in missions.
    
    // @Group: TERRAIN_
    // @Path: ../libraries/AP_Terrain/AP_Terrain.cpp
    GOBJECT(terrain,                "TERRAIN_", AP_Terrain),
#endif

#if AP_OPTICALFLOW_ENABLED
    // Optical flow sensors: velocity estimation from downward-facing camera
    // for GPS-denied navigation and position hold.
    
    // @Group: FLOW
    // @Path: ../libraries/AP_OpticalFlow/AP_OpticalFlow.cpp
    GOBJECT(optflow,   "FLOW", AP_OpticalFlow),
#endif

#if AC_PRECLAND_ENABLED
    // Precision landing: IR beacon or visual marker tracking for accurate
    // landing on moving platforms or designated landing zones.
    
    // @Group: PLND_
    // @Path: ../libraries/AC_PrecLand/AC_PrecLand.cpp
    GOBJECT(precland, "PLND_", AC_PrecLand),
#endif

#if AP_RPM_ENABLED
    // RPM sensors: motor/rotor speed monitoring for health checks
    // and autorotation detection (helicopters).
    
    // @Group: RPM
    // @Path: ../libraries/AP_RPM/AP_RPM.cpp
    GOBJECT(rpm_sensor, "RPM", AP_RPM),
#endif

    // ========================================================================
    // Traffic and Collision Avoidance
    // ========================================================================
    
#if HAL_ADSB_ENABLED
    // ADS-B receiver: automatic detection of nearby aircraft for collision avoidance
    // and traffic awareness display on ground stations.
    
    // @Group: ADSB_
    // @Path: ../libraries/AP_ADSB/AP_ADSB.cpp
    GOBJECT(adsb,                "ADSB_", AP_ADSB),
#endif  // HAL_ADSB_ENABLED

#if AP_ADSB_AVOIDANCE_ENABLED
    // ADS-B avoidance: active collision avoidance maneuvers based on
    // detected aircraft traffic from ADS-B receiver.
    
    // @Group: AVD_
    // @Path: ../libraries/AP_Avoidance/AP_Avoidance.cpp
    GOBJECT(avoidance_adsb, "AVD_", AP_Avoidance_Copter),
#endif  // AP_ADSB_AVOIDANCE_ENABLED

    // ========================================================================
    // User Feedback and Notification Systems
    // ========================================================================
    
    // LED indicators, buzzers, and display notifications for arming status,
    // GPS lock, flight mode changes, and warnings/errors.
    
    // @Group: NTF_
    // @Path: ../libraries/AP_Notify/AP_Notify.cpp
    GOBJECT(notify, "NTF_",  AP_Notify),

    // ========================================================================
    // Specialized Flight Mode Parameters
    // ========================================================================
    
#if MODE_THROW_ENABLED
    // Throw mode: allows hand-launching by throwing the copter upward.
    // Motors arm automatically and stabilize after detecting the throw.
    
    // @Param: THROW_MOT_START
    // @DisplayName: Start motors before throwing is detected
    // @Description: Used by Throw mode. Controls whether motors will run at the speed set by MOT_SPIN_MIN or will be stopped when armed and waiting for the throw.
    // @Values: 0:Stopped,1:Running
    // @User: Standard
    GSCALAR(throw_motor_start, "THROW_MOT_START", (float)ModeThrow::PreThrowMotorState::STOPPED),

    // @Param: THROW_ALT_MIN
    // @DisplayName: Throw mode minimum altitude
    // @Description: Minimum altitude above which Throw mode will detect a throw or a drop - 0 to disable the check
    // @Units: m
    // @User: Advanced
    GSCALAR(throw_altitude_min, "THROW_ALT_MIN", 0),

    // @Param: THROW_ALT_MAX
    // @DisplayName: Throw mode maximum altitude
    // @Description: Maximum altitude under which Throw mode will detect a throw or a drop - 0 to disable the check
    // @Units: m
    // @User: Advanced
    GSCALAR(throw_altitude_max, "THROW_ALT_MAX", 0),

    // @Param: THROW_ALT_DCSND
    // @DisplayName: Throw mode target altitude to descend
    // @Description: Target altitude to descend during a drop, (must be positive). This allows for rapidly clearing surrounding obstacles.
    // @Units: m
    // @User: Advanced
    GSCALAR(throw_altitude_descend, "THROW_ALT_DCSND", 1.0),

    // @Param: THROW_ALT_ACSND
    // @DisplayName: Throw mode target altitude to ascsend
    // @Description: Target altitude to ascend during a throw upwards (must be positive). This allows for rapidly clearing surrounding obstacles.
    // @Units: m
    // @User: Advanced
    GSCALAR(throw_altitude_ascend, "THROW_ALT_ACSND", 3.0),
#endif

    // ========================================================================
    // Display and Interface Systems
    // ========================================================================
    
#if OSD_ENABLED || OSD_PARAM_ENABLED
    // On-screen display: configurable HUD overlay for FPV video feed with
    // flight data (altitude, speed, battery, GPS, warnings, etc.).
    
    // @Group: OSD
    // @Path: ../libraries/AP_OSD/AP_OSD.cpp
    GOBJECT(osd, "OSD", AP_OSD),
#endif

    // ========================================================================
    // Advanced Control Systems
    // ========================================================================
    
#if AC_CUSTOMCONTROL_MULTI_ENABLED
    // Custom control: user-defined control laws via scripting or external
    // backend for research and advanced flight control experiments.
    
    // @Group: CC
    // @Path: ../libraries/AC_CustomControl/AC_CustomControl.cpp
    GOBJECT(custom_control, "CC", AC_CustomControl),
#endif

    // ========================================================================
    // Extended Parameter Groups
    // ========================================================================
    
    // ParametersG2: Second-generation parameter group containing newer parameters
    // and features added after the original var_info table was established.
    // Allows parameter additions without breaking EEPROM compatibility.
    
    // @Group:
    // @Path: Parameters.cpp
    GOBJECT(g2, "",  ParametersG2),

    // Core vehicle-level parameters: firmware version info, scheduled tasks,
    // vehicle-specific capabilities, and common vehicle infrastructure.
    
    // @Group:
    // @Path: ../libraries/AP_Vehicle/AP_Vehicle.cpp
    PARAM_VEHICLE_INFO,

    // ========================================================================
    // Ground Control Station Communication
    // ========================================================================
    
#if HAL_GCS_ENABLED
    // MAVLink GCS interface: telemetry streaming, command handling, parameter
    // protocol, mission upload/download, and ground station communication.
    
    // @Group: MAV
    // @Path: ../libraries/GCS_MAVLink/GCS.cpp
    GOBJECT(_gcs,           "MAV",  GCS),
#endif

    AP_VAREND
};

/**
 * @brief Second-generation parameter table (ParametersG2)
 * 
 * @details This parameter group was created to allow parameter additions without breaking
 *          EEPROM compatibility with earlier firmware versions. New parameters are added
 *          here rather than the main var_info table to preserve parameter storage layout.
 *          
 *          The ParametersG2 structure allows:
 *          - Adding new parameters without re-numbering existing ones
 *          - Maintaining backward compatibility with saved parameter sets
 *          - Organizing newer features and modes added post-initial release
 *          - Extending to var_info2 for additional capacity
 *          
 *          Parameter numbering in this group starts from 1 and must not conflict
 *          with existing entries. Deleted parameters leave gaps that should not be reused
 *          to prevent conflicts with user's saved configurations.
 *          
 *          @note This is registered as a GOBJECT in the main var_info table
 *          @see Copter::var_info for primary parameter table
 *          @see ParametersG2::var_info2 for extended parameters
 */
const AP_Param::GroupInfo ParametersG2::var_info[] = {

    // ========================================================================
    // Navigation and Waypoint Parameters
    // ========================================================================
    
    // @Param: WP_NAVALT_MIN
    // @DisplayName: Minimum navigation altitude
    // @Description: This is the altitude in meters above which for navigation can begin. This applies in auto takeoff and auto landing.
    // @Range: 0 5
    // @User: Standard
    AP_GROUPINFO("WP_NAVALT_MIN", 1, ParametersG2, wp_navalt_min, 0),

    // ========================================================================
    // Hardware Input Configuration
    // ========================================================================
    
#if HAL_BUTTON_ENABLED
    // Button configuration: maps physical buttons on the flight controller
    // to functions like arm/disarm, mode changes, or custom actions.
    
    // @Group: BTN_
    // @Path: ../libraries/AP_Button/AP_Button.cpp
    AP_SUBGROUPPTR(button_ptr, "BTN_", 2, ParametersG2, AP_Button),
#endif

    // ========================================================================
    // Throw Mode Extended Parameters
    // ========================================================================
    
#if MODE_THROW_ENABLED
    // Throw mode follow-up behavior and detection configuration.
    // These extend the base THROW_ parameters from the main var_info table.
    
    // @Param: THROW_NEXTMODE
    // @DisplayName: Throw mode's follow up mode
    // @Description: Vehicle will switch to this mode after the throw is successfully completed.  Default is to stay in throw mode (18)
    // @Values: 3:Auto,4:Guided,5:LOITER,6:RTL,9:Land,17:Brake,18:Throw
    // @User: Standard
    AP_GROUPINFO("THROW_NEXTMODE", 3, ParametersG2, throw_nextmode, 18),

    // @Param: THROW_TYPE
    // @DisplayName: Type of Type
    // @Description: Used by Throw mode. Specifies whether Copter is thrown upward or dropped.
    // @Values: 0:Upward Throw,1:Drop
    // @User: Standard
    AP_GROUPINFO("THROW_TYPE", 4, ParametersG2, throw_type, (float)ModeThrow::ThrowType::Upward),
#endif

    // ========================================================================
    // Altitude Control Enhancements
    // ========================================================================
    
    // Ground effect compensation: adjusts altitude control when near the ground
    // to account for increased lift from rotor downwash reflection.
    
    // @Param: GND_EFFECT_COMP
    // @DisplayName: Ground Effect Compensation Enable/Disable
    // @Description: Ground Effect Compensation Enable/Disable
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("GND_EFFECT_COMP", 5, ParametersG2, gndeffect_comp_enabled, 1),

    // ========================================================================
    // Safety and Failsafe Systems
    // ========================================================================
    
#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
    // Advanced Failsafe System: hardware-triggered termination and emergency
    // actions based on external autonomous monitoring for flight termination systems.
    
    // @Group: AFS_
    // @Path: ../libraries/AP_AdvancedFailsafe/AP_AdvancedFailsafe.cpp
    AP_SUBGROUPINFO(afs, "AFS_", 6, ParametersG2, AP_AdvancedFailsafe),
#endif

    // ========================================================================
    // Developer and Debug Options
    // ========================================================================
    
    // Developer options bitmask: experimental features and debug functionality.
    // CAUTION: These options are for development/testing only and may change
    // behavior in unexpected ways. Not for production use.
    
    // @Param: DEV_OPTIONS
    // @DisplayName: Development options
    // @Description: Bitmask of developer options. The meanings of the bit fields in this parameter may vary at any time. Developers should check the source code for current meaning
    // @Bitmask: 0:ADSBMavlinkProcessing,1:DevOptionVFR_HUDRelativeAlt
    // @User: Advanced
    AP_GROUPINFO("DEV_OPTIONS", 7, ParametersG2, dev_options, 0),

    // ========================================================================
    // Alternative Positioning Systems
    // ========================================================================
    
#if AP_BEACON_ENABLED
    // Beacon positioning: triangulation-based position estimation using
    // fixed ground-based beacons as alternative to GPS.
    
    // @Group: BCN
    // @Path: ../libraries/AP_Beacon/AP_Beacon.cpp
    AP_SUBGROUPINFO(beacon, "BCN", 14, ParametersG2, AP_Beacon),
#endif

#if HAL_PROXIMITY_ENABLED
    // Proximity sensors: 360-degree obstacle detection using lidar, sonar,
    // or radar for collision avoidance and safe navigation in confined spaces.
    
    // @Group: PRX
    // @Path: ../libraries/AP_Proximity/AP_Proximity.cpp
    AP_SUBGROUPINFO(proximity, "PRX", 8, ParametersG2, AP_Proximity),
#endif

    // ========================================================================
    // Acro Mode Configuration
    // ========================================================================
    
    // ACRO_Y_EXPO (9) moved to Command Model Class

#if MODE_ACRO_ENABLED
    // Acro mode throttle center point: defines mid-stick throttle position
    // for spring-centered throttle transmitters.
    
    // @Param: ACRO_THR_MID
    // @DisplayName: Acro Thr Mid
    // @Description: Acro Throttle Mid
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("ACRO_THR_MID", 10, ParametersG2, acro_thr_mid, ACRO_THR_MID_DEFAULT),
#endif

    // ========================================================================
    // Parameter Slot History (Reserved/Moved)
    // ========================================================================
    
    // 11 was SYSID_ENFORCE (moved to MAV_OPTIONS)

    // 12 was AP_Stats (moved to vehicle-level parameters)

    // 13 was AP_Gripper (moved to vehicle-level parameters)

    // ========================================================================
    // Frame Configuration
    // ========================================================================
    
    // Frame class: fundamental airframe type (quad, hexa, octa, heli, etc.).
    // This determines motor mixing, control allocation, and vehicle-specific behaviors.
    // CRITICAL: Requires reboot and proper motor ordering for safety.
    
    // @Param: FRAME_CLASS
    // @DisplayName: Frame Class
    // @Description: Controls major frame class for multicopter component
    // @Values: 0:Undefined, 1:Quad, 2:Hexa, 3:Octa, 4:OctaQuad, 5:Y6, 6:Heli, 7:Tri, 8:SingleCopter, 9:CoaxCopter, 10:BiCopter, 11:Heli_Dual, 12:DodecaHexa, 13:HeliQuad, 14:Deca, 15:Scripting Matrix, 16:6DoF Scripting, 17:Dynamic Scripting Matrix
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("FRAME_CLASS", 15, ParametersG2, frame_class, DEFAULT_FRAME_CLASS),

    // ========================================================================
    // Servo and RC Channel Configuration
    // ========================================================================
    
    // Servo channel configuration: output function assignment, min/max/trim values,
    // reverse settings for all servo outputs beyond basic motor outputs.
    
    // @Group: SERVO
    // @Path: ../libraries/SRV_Channel/SRV_Channels.cpp
    AP_SUBGROUPINFO(servo_channels, "SERVO", 16, ParametersG2, SRV_Channels),

    // RC channel configuration: input mapping, trim, deadzone, override behavior,
    // and auxiliary switch functions for all RC input channels.
    
    // @Group: RC
    // @Path: ../libraries/RC_Channel/RC_Channels_VarInfo.h
    AP_SUBGROUPINFO(rc_channels, "RC", 17, ParametersG2, RC_Channels_Copter),

    // 18 was used by AP_VisualOdom (moved to vehicle-level parameters)

    // ========================================================================
    // Sensor Calibration
    // ========================================================================
    
#if AP_TEMPCALIBRATION_ENABLED
    // Temperature calibration: IMU bias compensation based on board temperature
    // to improve accuracy across varying environmental conditions.
    
    // @Group: TCAL
    // @Path: ../libraries/AP_TempCalibration/AP_TempCalibration.cpp
    AP_SUBGROUPINFO(temp_calibration, "TCAL", 19, ParametersG2, AP_TempCalibration),
#endif

    // ========================================================================
    // Advanced Flight Modes
    // ========================================================================
    
#if TOY_MODE_ENABLED
    // Toy mode: simplified flight mode with gesture-based controls for
    // beginner pilots or demonstration purposes.
    
    // @Group: TMODE
    // @Path: toy_mode.cpp
    AP_SUBGROUPINFO(toy_mode, "TMODE", 20, ParametersG2, ToyMode),
#endif

#if MODE_SMARTRTL_ENABLED
    // Smart RTL: intelligent return to launch that follows the safest path
    // by retracing the outbound flight path and avoiding obstacles encountered.
    
    // @Group: SRTL_
    // @Path: ../libraries/AP_SmartRTL/AP_SmartRTL.cpp
    AP_SUBGROUPINFO(smart_rtl, "SRTL_", 21, ParametersG2, AP_SmartRTL),
#endif

    // ========================================================================
    // Payload and Winch Systems
    // ========================================================================
    
#if AP_WINCH_ENABLED
    // 22 was AP_WheelEncoder (deprecated)

    // Winch control: automated payload delivery, cable length management,
    // and controlled descent/ascent operations for delivery missions.
    
    // @Group: WINCH
    // @Path: ../libraries/AP_Winch/AP_Winch.cpp
    AP_SUBGROUPINFO(winch, "WINCH", 23, ParametersG2, AP_Winch),
#endif

    // ========================================================================
    // Pilot Control Limits
    // ========================================================================
    
    // Pilot descent speed: maximum rate pilot can command downward movement.
    // If set to 0, uses PILOT_SPEED_UP value for symmetrical control.
    // Controls maximum descent rate in altitude-holding modes.
    
    // @Param: PILOT_SPEED_DN
    // @DisplayName: Pilot maximum vertical speed descending
    // @Description: The maximum vertical descending velocity the pilot may request in cm/s.  If 0 PILOT_SPEED_UP value is used.
    // @Units: cm/s
    // @Range: 0 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("PILOT_SPEED_DN", 24, ParametersG2, pilot_speed_dn, 0),

    // ========================================================================
    // Landing Configuration
    // ========================================================================
    
    // Landing altitude transition: altitude at which vehicle slows from fast
    // descent (LAND_SPEED_HIGH) to final slow descent (LAND_SPEED).
    // Provides smooth two-stage landing approach.
    
    // @Param: LAND_ALT_LOW
    // @DisplayName: Land alt low
    // @Description: Altitude during Landing at which vehicle slows to LAND_SPEED
    // @Units: cm
    // @Range: 100 10000
    // @Increment: 10
    // @User: Advanced
    AP_GROUPINFO("LAND_ALT_LOW", 25, ParametersG2, land_alt_low, 1000),

    // ========================================================================
    // Alternative Positioning Modes
    // ========================================================================
    
#if MODE_FLOWHOLD_ENABLED
    // FlowHold mode: position hold using optical flow sensor when GPS unavailable.
    // Enables stable hover indoors or in GPS-denied environments using downward
    // camera for velocity estimation.
    
    // @Group: FHLD
    // @Path: mode_flowhold.cpp
    AP_SUBGROUPPTR(mode_flowhold_ptr, "FHLD", 26, ParametersG2, ModeFlowHold),
#endif

#if MODE_FOLLOW_ENABLED
    // Follow mode: autonomous following of another vehicle using position telemetry.
    // Maintains configurable distance and angle offset from lead vehicle with
    // MAVLink position updates.
    
    // @Group: FOLL
    // @Path: ../libraries/AP_Follow/AP_Follow.cpp
    AP_SUBGROUPINFO(follow, "FOLL", 27, ParametersG2, AP_Follow),
#endif

    // ========================================================================
    // User-Defined Parameters
    // ========================================================================
    
#if USER_PARAMS_ENABLED
    // User parameters: custom parameters for scripting or external integrations.
    // Provides persistent storage for user-defined values without requiring
    // firmware modification.
    
    AP_SUBGROUPINFO(user_parameters, "USR", 28, ParametersG2, UserParameters),
#endif

    // ========================================================================
    // AutoTune System
    // ========================================================================
    
#if AUTOTUNE_ENABLED
    // AutoTune: automated PID gain tuning through controlled oscillation tests.
    // Systematically tests vehicle response and calculates optimal attitude
    // control gains. Multi-copter and helicopter implementations included.
    
    // @Group: AUTOTUNE_
    // @Path: ../libraries/AC_AutoTune/AC_AutoTune_Multi.cpp,../libraries/AC_AutoTune/AC_AutoTune_Heli.cpp
    AP_SUBGROUPPTR(autotune_ptr, "AUTOTUNE_",  29, ParametersG2, AutoTune),
#endif

    // 30 was AP_Scripting (moved to vehicle-level parameters)

    // ========================================================================
    // Transmitter Tuning Knob
    // ========================================================================
    
    // Transmitter tuning system: allows in-flight parameter adjustment via
    // RC channel knob. Enables real-time PID tuning without requiring ground
    // station connection. Range defined by TUNE_MIN and TUNE_MAX.
    
    // @Param: TUNE_MIN
    // @DisplayName: Tuning minimum
    // @Description: Transmitter Tuning minum value. The parameter being tuned will have its value set to this minimum value when the tuning knob is at its lowest position
    // @User: Standard
    AP_GROUPINFO("TUNE_MIN", 31, ParametersG2, tuning_min, 0),

    // @Param: TUNE_MAX
    // @DisplayName: Tuning maximum
    // @Description: Transmitter Tuning maximum value. The parameter being tuned will have its value set to this maximum value when the tuning knob is at its highest position
    // @User: Standard
    AP_GROUPINFO("TUNE_MAX", 32, ParametersG2, tuning_max, 0),

    // ========================================================================
    // Obstacle Avoidance
    // ========================================================================
    
#if AP_OAPATHPLANNER_ENABLED
    // Object avoidance path planner: dynamic path planning around obstacles
    // detected by proximity sensors. Adjusts waypoint navigation to avoid
    // collisions while maintaining mission progress.
    
    // @Group: OA_
    // @Path: ../libraries/AC_Avoidance/AP_OAPathPlanner.cpp
    AP_SUBGROUPINFO(oa, "OA_", 33, ParametersG2, AP_OAPathPlanner),
#endif

    // ========================================================================
    // System Identification Mode
    // ========================================================================
    
#if MODE_SYSTEMID_ENABLED
    // SystemID mode: automated frequency response testing for control system
    // analysis. Applies controlled frequency sweeps to measure vehicle dynamics
    // for advanced tuning and analysis.
    
    // @Group: SID
    // @Path: mode_systemid.cpp
    AP_SUBGROUPPTR(mode_systemid_ptr, "SID", 34, ParametersG2, ModeSystemId),
#endif

    // ========================================================================
    // Extended Failsafe Configuration
    // ========================================================================
    
    // Vibration failsafe: detects excessive vibrations and switches to modified
    // altitude estimation to maintain control during high-vibration conditions.
    // Uses accelerometer clipping detection and EKF variance monitoring.
    
    // @Param: FS_VIBE_ENABLE
    // @DisplayName: Vibration Failsafe enable
    // @Description: This enables the vibration failsafe which will use modified altitude estimation and control during high vibrations
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    AP_GROUPINFO("FS_VIBE_ENABLE", 35, ParametersG2, fs_vibe_enabled, 1),

    // Failsafe behavior options: bitmask controlling failsafe actions under
    // various conditions. Allows mission continuation, gripper release, and
    // mode-specific behaviors during RC/GCS/battery failsafe events.
    
    // @Param: FS_OPTIONS
    // @DisplayName: Failsafe options bitmask
    // @Description: Bitmask of additional options for battery, radio, & GCS failsafes. 0 (default) disables all options.
    // @Bitmask: 0:Continue if in Auto on RC failsafe, 1:Continue if in Auto on GCS failsafe, 2:Continue if in Guided on RC failsafe, 3:Continue if landing on any failsafe, 4:Continue if in pilot controlled modes on GCS failsafe, 5:Release Gripper
    // @User: Advanced
    AP_GROUPINFO("FS_OPTIONS", 36, ParametersG2, fs_options, (float)Copter::FailsafeOption::GCS_CONTINUE_IF_PILOT_CONTROL),

    // ========================================================================
    // Helicopter-Specific Modes
    // ========================================================================
    
#if MODE_AUTOROTATE_ENABLED
    // Autorotation mode: automated engine-off landing for helicopters.
    // Manages rotor speed, collective pitch, and flare timing for safe
    // unpowered descent and landing following engine failure.
    
    // @Group: AROT_
    // @Path: ../libraries/AC_Autorotation/AC_Autorotation.cpp
    AP_SUBGROUPINFO(arot, "AROT_", 37, ParametersG2, AC_Autorotation),
#endif

    // ========================================================================
    // Specialized Flight Modes
    // ========================================================================
    
#if MODE_ZIGZAG_ENABLED
    // ZigZag mode: automated survey pattern flight for aerial mapping.
    // Pilot defines survey boundaries, then vehicle automatically flies
    // back-and-forth pattern with configurable line spacing.
    
    // @Group: ZIGZ_
    // @Path: mode_zigzag.cpp
    AP_SUBGROUPPTR(mode_zigzag_ptr, "ZIGZ_", 38, ParametersG2, ModeZigZag),
#endif

    // ========================================================================
    // Mode-Specific Behavior Options
    // ========================================================================
    
#if MODE_ACRO_ENABLED
    // Acro mode behavior options: configures advanced Acro mode features.
    // Air-mode maintains stabilization authority at zero throttle.
    // Rate Loop Only bypasses angle stabilization for pure rate control.
    
    // @Param: ACRO_OPTIONS
    // @DisplayName: Acro mode options
    // @Description: A range of options that can be applied to change acro mode behaviour. Air-mode enables ATC_THR_MIX_MAN at all times (air-mode has no effect on helicopters). Rate Loop Only disables the use of angle stabilization and uses angular rate stabilization only.
    // @Bitmask: 0:Air-mode,1:Rate Loop Only
    // @User: Advanced
    AP_GROUPINFO("ACRO_OPTIONS", 39, ParametersG2, acro_options, 0),
#endif

#if MODE_AUTO_ENABLED
    // Auto mode behavior options: controls Auto mode arming, takeoff, and
    // pilot override behavior. Allows mission start without raising throttle
    // and configures yaw control authority during autonomous flight.
    
    // @Param: AUTO_OPTIONS
    // @DisplayName: Auto mode options
    // @Description: A range of options that can be applied to change auto mode behaviour. Allow Arming allows the copter to be armed in Auto. Allow Takeoff Without Raising Throttle allows takeoff without the pilot having to raise the throttle. Ignore pilot yaw overrides the pilot's yaw stick being used while in auto.
    // @Bitmask: 0:Allow Arming,1:Allow Takeoff Without Raising Throttle,2:Ignore pilot yaw,7:Allow weathervaning
    // @User: Advanced
    AP_GROUPINFO("AUTO_OPTIONS", 40, ParametersG2, auto_options, 0),
#endif

#if MODE_GUIDED_ENABLED
    // Guided mode behavior options: controls Guided mode arming, pilot overrides,
    // and attitude command interpretation. SetAttitudeTarget thrust interpretation
    // affects companion computer control behavior.
    
    // @Param: GUID_OPTIONS
    // @DisplayName: Guided mode options
    // @Description: Options that can be applied to change guided mode behaviour
    // @Bitmask: 0:Allow Arming from Transmitter,2:Ignore pilot yaw,3:SetAttitudeTarget interprets Thrust As Thrust,4:Do not stabilize PositionXY,5:Do not stabilize VelocityXY,6:Waypoint navigation used for position targets,7:Allow weathervaning
    // @User: Advanced
    AP_GROUPINFO("GUID_OPTIONS", 41, ParametersG2, guided_options, 0),
#endif

    // GCS failsafe timeout: delay before triggering GCS failsafe after loss
    // of ground station heartbeat messages. Prevents false triggers from
    // brief telemetry dropouts. Must balance responsiveness vs stability.
    
    // @Param: FS_GCS_TIMEOUT
    // @DisplayName: GCS failsafe timeout
    // @Description: Timeout before triggering the GCS failsafe
    // @Units: s
    // @Range: 2 120
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("FS_GCS_TIMEOUT", 42, ParametersG2, fs_gcs_timeout, 5),

#if MODE_RTL_ENABLED
    // RTL mode behavior options: controls pilot override capabilities during
    // Return to Launch. Ignore pilot yaw prevents yaw stick from affecting
    // vehicle heading during autonomous return.
    
    // @Param: RTL_OPTIONS
    // @DisplayName: RTL mode options
    // @Description: Options that can be applied to change RTL mode behaviour
    // @Bitmask: 2:Ignore pilot yaw
    // @User: Advanced
    AP_GROUPINFO("RTL_OPTIONS", 43, ParametersG2, rtl_options, 0),
#endif

    // General flight safety options: controls cross-mode safety features.
    // Thrust loss check detects motor failures. Yaw imbalance warning monitors
    // for asymmetric motor performance. Gripper release provides payload drop
    // capability during emergencies.
    
    // @Param: FLIGHT_OPTIONS
    // @DisplayName: Flight mode options
    // @Description: Flight mode specific options
    // @Bitmask: 0:Disable thrust loss check, 1:Disable yaw imbalance warning, 2:Release gripper on thrust loss
    // @User: Advanced
    AP_GROUPINFO("FLIGHT_OPTIONS", 44, ParametersG2, flight_options, 0),

    // ========================================================================
    // Rangefinder and Surface Tracking Configuration
    // ========================================================================
    
#if AP_RANGEFINDER_ENABLED
    // Rangefinder low-pass filter: smooths rangefinder readings to reduce
    // noise. Lower frequencies provide more smoothing but increase latency.
    // Zero disables filtering. Used for terrain following and precision landing.
    
    // @Param: RNGFND_FILT
    // @DisplayName: Rangefinder filter
    // @Description: Rangefinder filter to smooth distance.  Set to zero to disable filtering
    // @Units: Hz
    // @Range: 0 5
    // @Increment: 0.05
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("RNGFND_FILT", 45, ParametersG2, rangefinder_filt, RANGEFINDER_FILT_DEFAULT),
#endif

#if MODE_GUIDED_ENABLED
    // Guided mode command timeout: maximum time between position/velocity
    // updates from external controller. Vehicle stops or levels if timeout
    // expires, preventing flyaways from companion computer failures.
    
    // @Param: GUID_TIMEOUT
    // @DisplayName: Guided mode timeout
    // @Description: Guided mode timeout after which vehicle will stop or return to level if no updates are received from caller. Only applicable during any combination of velocity, acceleration, angle control, and/or angular rate control
    // @Units: s
    // @Range: 0.1 5
    // @User: Advanced
    AP_GROUPINFO("GUID_TIMEOUT", 46, ParametersG2, guided_timeout, 3.0),
#endif

    // NOTE: ACRO_PR_RATE (47), ACRO_Y_RATE (48), PILOT_Y_RATE (49) and PILOT_Y_EXPO (50)
    // have been migrated to AC_CommandModel class for improved rate control architecture.
    // See command_model_acro_rp, command_model_acro_y, and command_model_pilot_y below.

#if AP_RANGEFINDER_ENABLED
    // Surface tracking mode: selects which surface to maintain constant distance
    // from. Ground tracking for terrain following, ceiling tracking for indoor
    // navigation. Requires working rangefinder in selected direction.
    
    // @Param: SURFTRAK_MODE
    // @DisplayName: Surface Tracking Mode
    // @Description: set which surface to track in surface tracking
    // @Values: 0:Do not track, 1:Ground, 2:Ceiling
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("SURFTRAK_MODE", 51, ParametersG2, surftrak_mode, (uint8_t)Copter::SurfaceTracking::Surface::GROUND),
#endif

    // ========================================================================
    // Dead Reckoning Failsafe Configuration
    // ========================================================================
    
    // Dead reckoning failsafe action: triggered when EKF loses all position
    // and velocity sources (GPS, optical flow, etc.) and relies only on
    // inertial sensors and wind estimates. Immediate action prevents
    // uncontrolled drift during sensor failures.
    
    // @Param: FS_DR_ENABLE
    // @DisplayName: DeadReckon Failsafe Action
    // @Description: Failsafe action taken immediately as deadreckoning starts. Deadreckoning starts when EKF loses position and velocity source and relies only on wind estimates
    // @Values: 0:Disabled/NoAction,1:Land, 2:RTL, 3:SmartRTL or RTL, 4:SmartRTL or Land, 6:Auto DO_LAND_START/DO_RETURN_PATH_START or RTL
    // @User: Standard
    AP_GROUPINFO("FS_DR_ENABLE", 52, ParametersG2, failsafe_dr_enable, (uint8_t)Copter::FailsafeAction::RTL),

    // Dead reckoning timeout: duration the EKF can operate in dead reckoning
    // mode before triggering full EKF failsafe. Allows brief sensor outages
    // while preventing prolonged operation without position reference.
    
    // @Param: FS_DR_TIMEOUT
    // @DisplayName: DeadReckon Failsafe Timeout
    // @Description: DeadReckoning is available for this many seconds after losing position and/or velocity source.  After this timeout elapses the EKF failsafe will trigger in modes requiring a position estimate
    // @Range: 0 120
    // @User: Standard
    AP_GROUPINFO("FS_DR_TIMEOUT", 53, ParametersG2, failsafe_dr_timeout, 30),

    // ========================================================================
    // Advanced Rate Control - Command Model Architecture
    // ========================================================================
    
    // The AC_CommandModel system provides sophisticated pilot input processing
    // with configurable rate limits, exponential curves, and time constants.
    // This architecture replaces simple P gain rate control with a more
    // responsive and tunable input shaping system.
    
#if MODE_ACRO_ENABLED || MODE_SPORT_ENABLED
    // Acro mode roll/pitch rate control: defines maximum rotation rates and
    // input shaping for roll and pitch axes in Acro and Sport modes.
    // RATE sets maximum deg/s, EXPO shapes stick response curve, RATE_TC
    // controls input responsiveness (time constant for rate changes).
    
    // @Param: ACRO_RP_RATE
    // @DisplayName: Acro Roll and Pitch Rate
    // @Description: Acro mode maximum roll and pitch rate.  Higher values mean faster rate of rotation
    // @Units: deg/s
    // @Range: 1 1080
    // @User: Standard

    // @Param: ACRO_RP_EXPO
    // @DisplayName: Acro Roll/Pitch Expo
    // @Description: Acro roll/pitch Expo to allow faster rotation when stick at edges
    // @Values: 0:Disabled,0.1:Very Low,0.2:Low,0.3:Medium,0.4:High,0.5:Very High
    // @Range: -0.5 0.95
    // @User: Advanced

    // @Param: ACRO_RP_RATE_TC
    // @DisplayName: Acro roll/pitch rate control input time constant
    // @Description: Acro roll and pitch rate control input time constant.  Low numbers lead to sharper response, higher numbers to softer response
    // @Units: s
    // @Range: 0 1
    // @Increment: 0.01
    // @Values: 0.5:Very Soft, 0.2:Soft, 0.15:Medium, 0.1:Crisp, 0.05:Very Crisp
    // @User: Standard
    AP_SUBGROUPINFO(command_model_acro_rp, "ACRO_RP_", 54, ParametersG2, AC_CommandModel),
#endif

#if MODE_ACRO_ENABLED || MODE_DRIFT_ENABLED
    // Acro mode yaw rate control: defines maximum yaw rotation rate and
    // input shaping for Acro and Drift modes. Independent from roll/pitch
    // to allow different aggressiveness on yaw axis.
    
    // @Param: ACRO_Y_RATE
    // @DisplayName: Acro Yaw Rate
    // @Description: Acro mode maximum yaw rate.  Higher value means faster rate of rotation
    // @Units: deg/s
    // @Range: 1 360
    // @User: Standard

    // @Param: ACRO_Y_EXPO
    // @DisplayName: Acro Yaw Expo
    // @Description: Acro yaw expo to allow faster rotation when stick at edges
    // @Values: 0:Disabled,0.1:Very Low,0.2:Low,0.3:Medium,0.4:High,0.5:Very High
    // @Range: -1.0 0.95
    // @User: Advanced

    // @Param: ACRO_Y_RATE_TC
    // @DisplayName: Acro yaw rate control input time constant
    // @Description: Acro yaw rate control input time constant.  Low numbers lead to sharper response, higher numbers to softer response
    // @Units: s
    // @Range: 0 1
    // @Increment: 0.01
    // @Values: 0.5:Very Soft, 0.2:Soft, 0.15:Medium, 0.1:Crisp, 0.05:Very Crisp
    // @User: Standard
    AP_SUBGROUPINFO(command_model_acro_y, "ACRO_Y_", 55, ParametersG2, AC_CommandModel),
#endif

    // Pilot yaw rate control: defines maximum yaw rate and input shaping for
    // all stabilized modes (excluding Acro). Used in Stabilize, AltHold,
    // Loiter, PosHold, and other modes where pilot directly controls yaw rate.
    
    // @Param: PILOT_Y_RATE
    // @DisplayName: Pilot controlled yaw rate
    // @Description: Pilot controlled yaw rate max.  Used in all pilot controlled modes except Acro
    // @Units: deg/s
    // @Range: 1 360
    // @User: Standard

    // @Param: PILOT_Y_EXPO
    // @DisplayName: Pilot controlled yaw expo
    // @Description: Pilot controlled yaw expo to allow faster rotation when stick at edges
    // @Values: 0:Disabled,0.1:Very Low,0.2:Low,0.3:Medium,0.4:High,0.5:Very High
    // @Range: -0.5 1.0
    // @User: Advanced

    // @Param: PILOT_Y_RATE_TC
    // @DisplayName: Pilot yaw rate control input time constant
    // @Description: Pilot yaw rate control input time constant.  Low numbers lead to sharper response, higher numbers to softer response
    // @Units: s
    // @Range: 0 1
    // @Increment: 0.01
    // @Values: 0.5:Very Soft, 0.2:Soft, 0.15:Medium, 0.1:Crisp, 0.05:Very Crisp
    // @User: Standard
    AP_SUBGROUPINFO(command_model_pilot_y, "PILOT_Y_", 56, ParametersG2, AC_CommandModel),

    // ========================================================================
    // Takeoff Safety and Detection Parameters
    // ========================================================================
    
    // Takeoff throttle slew rate: controls how quickly throttle increases
    // during automatic takeoff detection. Slower slew rates provide more time
    // to detect successful motor spin-up and liftoff. Faster rates reduce
    // takeoff time but may miss detection failures.
    
    // @Param: TKOFF_SLEW_TIME
    // @DisplayName: Slew time of throttle during take-off
    // @Description: Time to slew the throttle from minimum to maximum while checking for a successful takeoff.
    // @Units: s
    // @Range: 0.25 5.0
    // @User: Standard
    AP_GROUPINFO("TKOFF_SLEW_TIME", 57, ParametersG2, takeoff_throttle_slew_time, 2.0),

#if HAL_WITH_ESC_TELEM && FRAME_CONFIG != HELI_FRAME
    // Takeoff minimum RPM check: requires ESC telemetry to report minimum
    // motor speed before allowing takeoff. Prevents takeoff with failed or
    // disconnected motors. Only available with ESC telemetry support.
    
    // @Param: TKOFF_RPM_MIN
    // @DisplayName: Takeoff Check RPM minimum
    // @Description: Takeoff is not permitted until motors report at least this RPM.  Set to zero to disable check
    // @Range: 0 10000
    // @User: Standard
    AP_GROUPINFO("TKOFF_RPM_MIN", 58, ParametersG2, takeoff_rpm_min, 0),
#endif

    // ========================================================================
    // Weathervaning - Automatic Heading Adjustment
    // ========================================================================
    
#if WEATHERVANE_ENABLED
    // Weathervaning: automatically adjusts vehicle heading to minimize wind
    // resistance during position-controlled flight. Reduces position holding
    // effort by pointing into the wind. Useful for long loiters and when
    // heading is not critical to mission success.
    
    // @Group: WVANE_
    // @Path: ../libraries/AC_AttitudeControl/AC_WeatherVane.cpp
    AP_SUBGROUPINFO(weathervane, "WVANE_", 59, ParametersG2, AC_WeatherVane),
#endif

    // ========================================================================
    // Parameter Table Extension
    // ========================================================================
    
    // NOTE: ID 60 is reserved for SHIP_OPS parameters (autonomous ship landing
    // operations from external forks). Not used in mainline ArduPilot.

    // Link to extended parameter table (var_info2) for additional parameters.
    // This extension mechanism allows parameter table expansion without
    // breaking backward compatibility with stored parameter values.
    
    AP_SUBGROUPEXTENSION("", 61, ParametersG2, var_info2),

    // NOTE: ID 62 is reserved for SHOW_... parameters from the Skybrush fork
    // (https://github.com/skybrush-io/ardupilot) used for drone light shows.
    // Not used in mainline ArduPilot.

    AP_GROUPEND
};

/*
  ============================================================================
  EXTENSION TO G2 PARAMETERS (var_info2)
  ============================================================================
  
  This is the extended parameter table for ParametersG2. It provides additional
  parameter storage space linked via AP_SUBGROUPEXTENSION in the main var_info
  table. This extension mechanism preserves parameter addressing compatibility
  when expanding the parameter set.
  
  Parameters in this table include specialized features:
  - Payload placement (PLDP_*) for precision cargo delivery
  - Surface tracking filtering (SURFTRAK_TC)
  - Extended takeoff safety checks (TKOFF_THR_MAX, TKOFF_RPM_MAX)
  - EKF failsafe filtering (FS_EKF_FILT)
  - Fast rate attitude thread configuration (FSTRATE_*)
 */
const AP_Param::GroupInfo ParametersG2::var_info2[] = {

    // ========================================================================
    // Payload Placement Parameters (PLDP_*)
    // ========================================================================
    
    // Payload placement: specialized mode for precision cargo delivery.
    // Vehicle descends slowly until payload touches down (detected by thrust
    // reduction or rangefinder), releases payload, waits specified delay,
    // then climbs back to starting altitude. Used for delivering packages
    // or placing sensors without landing the copter itself.
    
    // @Param: PLDP_THRESH
    // @DisplayName: Payload Place thrust ratio threshold
    // @Description: Ratio of vertical thrust during decent below which payload touchdown will trigger.
    // @Range: 0.5 0.9
    // @User: Standard
    AP_GROUPINFO("PLDP_THRESH", 1, ParametersG2, pldp_thrust_placed_fraction, 0.9),

    // @Param: PLDP_RNG_MAX
    // @DisplayName: Payload Place maximum range finder altitude
    // @Description: Maximum range finder altitude in m to trigger payload touchdown, set to zero to disable.
    // @Units: m
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("PLDP_RNG_MAX", 2, ParametersG2, pldp_range_finder_maximum_m, 0.0),

    // @Param: PLDP_DELAY
    // @DisplayName: Payload Place climb delay
    // @Description: Delay after release, in seconds, before aircraft starts to climb back to starting altitude.
    // @Units: s
    // @Range: 0 120
    // @User: Standard
    AP_GROUPINFO("PLDP_DELAY", 3, ParametersG2, pldp_delay_s, 0.0),

    // @Param: PLDP_SPEED_DN
    // @DisplayName: Payload Place decent speed
    // @Description: The maximum vertical decent velocity in m/s. If 0 LAND_SPEED value is used.
    // @Units: m/s
    // @Range: 0 5
    // @User: Standard
    AP_GROUPINFO("PLDP_SPEED_DN", 4, ParametersG2, pldp_descent_speed_ms, 0.0),

    // ========================================================================
    // Surface Tracking Filter Configuration
    // ========================================================================
    
    // Surface tracking filter time constant: controls responsiveness of
    // altitude tracking when following terrain or ceiling surfaces. Lower
    // values provide faster response to surface changes but may oscillate.
    // Higher values smooth out transients but respond more slowly to real
    // surface changes. Set to 0 to disable filtering (immediate response).
    
    // @Param: SURFTRAK_TC
    // @DisplayName: Surface Tracking Filter Time Constant
    // @Description: Time to achieve 63.2% of the surface altitude measurement change.  If 0 filtering is disabled
    // @Units: s
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("SURFTRAK_TC", 5, ParametersG2, surftrak_tc, 1.0),

    // ========================================================================
    // Extended Takeoff Safety Parameters
    // ========================================================================
    
    // Takeoff maximum throttle: sets upper limit for throttle during takeoff
    // detection phase. Controllers use this threshold to determine when
    // aircraft is attempting to become airborne. Value must be below 1.0 to
    // ensure detection margin. Too high may delay liftoff detection.
    
    // @Param: TKOFF_THR_MAX
    // @DisplayName: Takeoff maximum throttle during take-off ramp up
    // @Description: Takeoff maximum throttle allowed before controllers assume the aircraft is airborne during the takeoff process.
    // @Range: 0.0 0.9
    // @User: Advanced
    AP_GROUPINFO("TKOFF_THR_MAX", 6, ParametersG2, takeoff_throttle_max, 0.9),

#if HAL_WITH_ESC_TELEM && FRAME_CONFIG != HELI_FRAME
    // Takeoff maximum RPM check: ensures motors are not spinning too fast
    // before allowing takeoff. Detects stuck throttle or ESC configuration
    // errors. Complements TKOFF_RPM_MIN check. Requires ESC telemetry.
    
    // @Param: TKOFF_RPM_MAX
    // @DisplayName: Takeoff Check RPM maximum
    // @Description: Takeoff is not permitted until motors report no more than this RPM.  Set to zero to disable check
    // @Range: 0 10000
    // @User: Standard
    AP_GROUPINFO("TKOFF_RPM_MAX", 7, ParametersG2, takeoff_rpm_max, 0),
#endif

    // ========================================================================
    // EKF Failsafe Filter Configuration
    // ========================================================================
    
    // EKF failsafe filter: low-pass filters EKF variance values to prevent
    // spurious failsafe triggers from transient variance spikes. Lower cutoff
    // frequency (Hz) provides more filtering but delays detection of real
    // EKF failures. Higher frequency triggers failsafe more quickly but may
    // false-trigger on momentary variance increases.
    
    // @Param: FS_EKF_FILT
    // @DisplayName: EKF Failsafe filter cutoff
    // @Description: EKF Failsafe filter cutoff frequency. EKF variances are filtered using this value to avoid spurious failsafes from transient high variances. A higher value means the failsafe is more likely to trigger.
    // @Range: 0 10
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("FS_EKF_FILT", 8, ParametersG2, fs_ekf_filt_hz, FS_EKF_FILT_DEFAULT),

    // ========================================================================
    // Fast Rate Attitude Thread Configuration
    // ========================================================================
    
#if AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED
    // Fast rate thread: runs attitude control at higher frequency than main
    // loop (typically 1kHz+ vs 400Hz). Improves attitude response and allows
    // better use of high-speed gyro data. Dynamic mode adjusts divisor to
    // prevent CPU overload. Fixed modes run at constant rate for consistent
    // performance. Only available on capable hardware platforms.
    
    // @Param: FSTRATE_ENABLE
    // @DisplayName: Enable the fast Rate thread
    // @Description: Enable the fast Rate thread. In the default case the fast rate divisor, which controls the update frequency of the thread, is dynamically scaled from FSTRATE_DIV to avoid overrun in the gyro sample buffer and main loop slow-downs. Other values can be selected to fix the divisor to FSTRATE_DIV on arming or always.
    // @User: Advanced
    // @Values: 0:Disabled,1:Enabled-Dynamic,2:Enabled-FixedWhenArmed,3:Enabled-Fixed
    AP_GROUPINFO("FSTRATE_ENABLE", 9, ParametersG2, att_enable, 0),

    // Fast rate divisor: divides gyro sampling rate to determine fast rate
    // thread frequency. Example: 8kHz gyro / divisor 8 = 1kHz fast rate.
    // Lower divisor = higher rate (more CPU load, better performance).
    // Higher divisor = lower rate (less CPU load, adequate for most uses).
    
    // @Param: FSTRATE_DIV
    // @DisplayName: Fast rate thread divisor
    // @Description: Fast rate thread divisor used to control the maximum fast rate update rate. The actual rate is the gyro rate in Hz divided by this value. This value is scaled depending on the configuration of FSTRATE_ENABLE.
    // @User: Advanced
    // @Range: 1 10
    AP_GROUPINFO("FSTRATE_DIV", 10, ParametersG2, att_decimation, 1),
#endif

    // NOTE: ID 62 is reserved for AP_SUBGROUPEXTENSION (future expansion point
    // if var_info2 needs to be extended to var_info3, preserving parameter
    // addressing compatibility)

    AP_GROUPEND
};

/**
 * @brief Constructor for ParametersG2 object
 * 
 * @details Initializes the secondary parameter group (g2) which extends the
 *          main parameter set. This constructor initializes all conditional
 *          parameter pointers and sub-objects based on build-time feature
 *          flags (HAL_*, MODE_*_ENABLED, etc.).
 * 
 *          The constructor sets up links to:
 *          - Mode-specific objects (pointers to mode instances)
 *          - Optional feature objects (buttons, proximity, beacon, etc.)
 *          - Advanced control objects (autotune, custom control, etc.)
 *          - Command model objects for rate/expo control
 * 
 *          All sub-objects are initialized with their respective default
 *          values defined in Parameters.h or library defaults.
 * 
 * @note This constructor runs once during Copter object initialization.
 *       It does NOT load parameter values from EEPROM - that happens later
 *       in load_parameters() after AP_Param system is initialized.
 * 
 * @see Copter::Copter() - Calls this constructor during vehicle initialization
 * @see Copter::load_parameters() - Loads stored parameter values from EEPROM
 * @see AP_Param::setup_object_defaults() - Sets up default parameter values
 */
ParametersG2::ParametersG2(void) :
    unused_integer{17}
#if HAL_BUTTON_ENABLED
    ,button_ptr(&copter.button)
#endif
#if AP_TEMPCALIBRATION_ENABLED
    , temp_calibration()
#endif
#if AP_BEACON_ENABLED
    , beacon()
#endif
#if HAL_PROXIMITY_ENABLED
    , proximity()
#endif
#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
    ,afs()
#endif
#if MODE_SMARTRTL_ENABLED
    ,smart_rtl()
#endif
#if USER_PARAMS_ENABLED
    ,user_parameters()
#endif
#if MODE_FLOWHOLD_ENABLED
    ,mode_flowhold_ptr(&copter.mode_flowhold)
#endif
#if MODE_FOLLOW_ENABLED
    ,follow()
#endif
#if AUTOTUNE_ENABLED
    ,autotune_ptr(&copter.mode_autotune.autotune)
#endif
#if MODE_SYSTEMID_ENABLED
    ,mode_systemid_ptr(&copter.mode_systemid)
#endif
#if MODE_AUTOROTATE_ENABLED
    ,arot(copter.motors, copter.attitude_control)
#endif
#if MODE_ZIGZAG_ENABLED
    ,mode_zigzag_ptr(&copter.mode_zigzag)
#endif
#if MODE_ACRO_ENABLED || MODE_SPORT_ENABLED
    ,command_model_acro_rp(ACRO_RP_RATE_DEFAULT, ACRO_RP_EXPO_DEFAULT, 0.0f)
#endif

#if MODE_ACRO_ENABLED || MODE_DRIFT_ENABLED
    ,command_model_acro_y(ACRO_Y_RATE_DEFAULT, ACRO_Y_EXPO_DEFAULT, 0.0f)
#endif

    ,command_model_pilot_y(PILOT_Y_RATE_DEFAULT, PILOT_Y_EXPO_DEFAULT, 0.0f)

#if WEATHERVANE_ENABLED
    ,weathervane()
#endif
{
    AP_Param::setup_object_defaults(this, var_info);
    AP_Param::setup_object_defaults(this, var_info2);
}

/**
 * @brief Load and convert parameters from EEPROM
 * 
 * @details This function performs the critical task of loading all vehicle
 *          parameters from persistent storage (EEPROM/Flash) and applying
 *          parameter format conversions for backward compatibility. Called
 *          once during vehicle initialization after AP_Param system is ready.
 * 
 *          The function handles:
 *          1. Base parameter loading via AP_Vehicle parent class
 *          2. Parameter data type conversions (e.g., INT8 -> INT16 expansion)
 *          3. Parameter relocations between parameter tables (g vs g2)
 *          4. Parameter namespace changes (old names -> new names)
 *          5. Frame type flag setup for parameter filtering
 * 
 *          Parameter conversions preserve user settings when code changes
 *          parameter structure. Each conversion is dated and versioned to
 *          track which ArduPilot release introduced the change.
 * 
 * @note PARAMETER_CONVERSION comments mark backward compatibility conversions.
 *       These conversions remain in code permanently to support upgrading from
 *       any previous ArduPilot version. Removing conversions breaks upgrades.
 * 
 * @warning This function MUST complete successfully before flight systems
 *          initialize. Failed parameter loading can cause unpredictable
 *          vehicle behavior. Check boot logs for parameter load errors.
 * 
 * @see AP_Vehicle::load_parameters() - Base class parameter loader
 * @see AP_Param::convert_parameter_width() - Type conversion for parameter storage
 * @see AP_Param::convert_class() - Move parameters between classes
 * @see AP_Param::convert_g2_objects() - Move parameters from g to g2 table
 * @see AP_Param::convert_toplevel_objects() - Move top-level parameter objects
 * @see AP_Param::convert_old_parameters() - Rename parameters with new names
 */
void Copter::load_parameters(void)
{
    // Load base parameters and check format version compatibility.
    // If format version mismatch detected, AP_Vehicle will handle upgrade
    // or flag error if downgrade attempted.
    AP_Vehicle::load_parameters(g.format_version, Parameters::k_format_version);

    // ========================================================================
    // Parameter Data Type Conversions
    // ========================================================================

#if MODE_RTL_ENABLED
    // RTL_ALT parameter expanded from 8-bit to 16-bit to support higher
    // altitudes (was limited to 255 cm = 2.55 m, now supports 32767 cm).
    // PARAMETER_CONVERSION - Added: Sep-2021
    g.rtl_altitude.convert_parameter_width(AP_PARAM_INT16);
#endif

    // ========================================================================
    // Parameter Class Relocations (Old Class -> New Class)
    // ========================================================================

    // Fence parameters moved from global 'g' to dedicated 'fence' object
    // for better organization and to support multiple fence types.
    // PARAMETER_CONVERSION - Added: Mar-2022
#if AP_FENCE_ENABLED
    AP_Param::convert_class(g.k_param_fence_old, &fence, fence.var_info, 0, true);
#endif

    // ========================================================================
    // G2 Object Conversions (Moved from G2 table positions to dedicated objects)
    // ========================================================================
    
    // These objects were originally in g2 parameter table but moved to
    // top-level objects for better maintainability. Conversion preserves
    // user settings across the reorganization.
    
    static const AP_Param::G2ObjectConversion g2_conversions[] {
#if AP_STATS_ENABLED
        // Stats moved from g2 slot 12 to dedicated stats object
        // PARAMETER_CONVERSION - Added: Jan-2024 for Copter-4.6
        { &stats, stats.var_info, 12 },
#endif
#if AP_SCRIPTING_ENABLED
        // Scripting moved from g2 slot 30 to dedicated scripting object
        // PARAMETER_CONVERSION - Added: Jan-2024 for Copter-4.6
        { &scripting, scripting.var_info, 30 },
#endif
#if AP_GRIPPER_ENABLED
        // Gripper moved from g2 slot 13 to dedicated gripper object
        // PARAMETER_CONVERSION - Added: Feb-2024 for Copter-4.6
        { &gripper, gripper.var_info, 13 },
#endif
    };

    AP_Param::convert_g2_objects(&g2, g2_conversions, ARRAY_SIZE(g2_conversions));

    // ========================================================================
    // Top-Level Object Relocations (G -> Top-Level)
    // ========================================================================

    // Logger parameters moved from 'g' table to dedicated logger object
    // for consistency with other vehicles (Plane, Rover).
    // PARAMETER_CONVERSION - Added: Feb-2024 for Copter-4.6
#if HAL_LOGGING_ENABLED
    AP_Param::convert_class(g.k_param_logger, &logger, logger.var_info, 0, true);
#endif

    // Serial manager moved from 'g' table to top-level for better organization
    static const AP_Param::TopLevelObjectConversion toplevel_conversions[] {
#if AP_SERIALMANAGER_ENABLED
        // PARAMETER_CONVERSION - Added: Feb-2024 for Copter-4.6
        { &serial_manager, serial_manager.var_info, Parameters::k_param_serial_manager_old },
#endif
    };

    AP_Param::convert_toplevel_objects(toplevel_conversions, ARRAY_SIZE(toplevel_conversions));

    // ========================================================================
    // Parameter Namespace Changes (Old Name -> New Name)
    // ========================================================================

#if HAL_GCS_ENABLED
    // MAVLink parameters moved to dedicated MAV_ namespace for clarity and
    // consistency across all ArduPilot vehicles. Old parameter names mapped
    // to new names transparently.
    // PARAMETER_CONVERSION - Added: Mar-2025 for ArduPilot-4.7
    {
        const AP_Param::ConversionInfo gcs_conversion_info[] {
            { Parameters::k_param_sysid_this_mav_old, 0, AP_PARAM_INT16,  "MAV_SYSID" },
            { Parameters::k_param_sysid_my_gcs_old, 0, AP_PARAM_INT16, "MAV_GCS_SYSID" },
            { Parameters::k_param_g2,  11, AP_PARAM_INT8, "MAV_OPTIONS" },
            { Parameters::k_param_telem_delay_old,  0, AP_PARAM_INT8, "MAV_TELEM_DELAY" },
        };
        AP_Param::convert_old_parameters(&gcs_conversion_info[0], ARRAY_SIZE(gcs_conversion_info));
    }
#endif  // HAL_GCS_ENABLED

    // ========================================================================
    // Frame Type Configuration
    // ========================================================================
    
    // Set frame type flag for parameter filtering. This allows parameters
    // to be marked as Copter-specific vs Plane/Rover-specific in ground
    // station displays, helping users avoid configuring irrelevant parameters.
    AP_Param::set_frame_type_flags(AP_PARAM_FRAME_COPTER);
}

/**
 * @brief Convert PID gains and control parameters for backward compatibility
 * 
 * @details Handles conversion of control system parameters when parameter
 *          structure or naming changes between ArduPilot versions. This
 *          function specifically manages:
 * 
 *          1. Position control filter parameter relocations
 *          2. Traditional Helicopter-specific default values
 *          3. Harmonic notch filter parameter migrations
 *          4. Acro mode rate parameter conversions (P gain -> deg/s rate)
 *          5. Command model parameter relocations (rate/expo)
 *          6. Servo channel parameter upgrades
 * 
 *          Many conversions apply scaling factors to preserve user-configured
 *          behavior. For example, ACRO_RP_P (P gain) converts to ACRO_RP_RATE
 *          (deg/s) with 45.0 scaling factor to maintain similar responsiveness.
 * 
 * @note Called during initialization after load_parameters() completes.
 *       Conversion only occurs if old parameters exist in EEPROM - if user
 *       has new parameters already configured, conversions are skipped.
 * 
 * @warning Removing parameter conversions breaks upgrade path from older
 *          ArduPilot versions. All conversions must remain in code permanently
 *          to support upgrades from any previous release.
 * 
 * @see Copter::load_parameters() - Calls this after base parameter loading
 * @see AP_Param::convert_old_parameter() - Converts single parameter with scaling
 * @see AP_Param::set_defaults_from_table() - Applies frame-specific defaults
 * @see SRV_Channels::upgrade_parameters() - Servo channel parameter upgrades
 */
void Copter::convert_pid_parameters(void)
{
    // ========================================================================
    // Position Control Filter Parameter Conversions
    // ========================================================================
    
    // Velocity XY filter cutoff moved from PI_VEL_XY controller to position
    // controller (PSC) for better organization. No scaling needed (1:1 mapping).
    const AP_Param::ConversionInfo angle_and_filt_conversion_info[] = {
        // PARAMETER_CONVERSION - Added: Aug-2021
        { Parameters::k_param_pi_vel_xy, 3, AP_PARAM_FLOAT, "PSC_VELXY_FLTE" },
    };

    // Convert without scaling - filter frequency values remain unchanged
    for (const auto &info : angle_and_filt_conversion_info) {
        AP_Param::convert_old_parameter(&info, 1.0f);
    }

    // ========================================================================
    // Traditional Helicopter Default Parameters
    // ========================================================================
    
#if FRAME_CONFIG == HELI_FRAME
    // Traditional helicopters require different default tuning than multi-rotors
    // due to mechanical differences (collective pitch, flybar, tail rotor).
    // These defaults provide stable flight for typical helicopter configurations.
    // Users with specific helicopter models should use posted parameter files
    // from ardupilot.org/copter for their exact airframe.
    
    static const struct AP_Param::defaults_table_struct heli_defaults_table[] = {
        { "LOIT_ACC_MAX", 500.0f },          // Lower max acceleration for heli
        { "LOIT_BRK_ACCEL", 125.0f },        // Gentler braking for heli
        { "LOIT_BRK_DELAY", 1.0f },          // Longer brake delay
        { "LOIT_BRK_JERK", 250.0f },         // Lower jerk limit
        { "LOIT_SPEED", 3000.0f },           // Conservative loiter speed
        { "PHLD_BRAKE_ANGLE", 800.0f },      // Reduced brake angle (8 degrees)
        { "PHLD_BRAKE_RATE", 4.0f },         // Slower brake rotation rate
        { "PSC_ACCZ_P", 0.28f },             // Lower Z accel gain for collective pitch
        { "PSC_VELXY_D", 0.0f },             // Zero D term for heli (not needed)
        { "PSC_VELXY_I", 0.5f },             // Moderate I term for position hold
        { "PSC_VELXY_P", 1.0f },             // Conservative P term
        { "RC8_OPTION", 32 },                // RC8 default: Motor Interlock (critical for heli)
        { "RC_OPTIONS", 0 },                 // Standard RC options
        { "ATC_RAT_RLL_ILMI", 0.05},         // Lower I-term leak minimum for roll
        { "ATC_RAT_PIT_ILMI", 0.05},         // Lower I-term leak minimum for pitch
    };
    AP_Param::set_defaults_from_table(heli_defaults_table, ARRAY_SIZE(heli_defaults_table));
#endif  // FRAME_CONFIG == HELI_FRAME

    // ========================================================================
    // Harmonic Notch Filter Parameter Conversions
    // ========================================================================
    
#if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
#if HAL_INS_NUM_HARMONIC_NOTCH_FILTERS > 1
    // Second harmonic notch filter support added in 4.2.x. If second notch
    // not configured, convert old fixed notch filter parameters to second
    // notch for backward compatibility with users who had configured the
    // old fixed notch system.
    if (!ins.harmonic_notches[1].params.enabled()) {
        // Convert fixed notch parameters to INS_HNTC2_ (second harmonic notch)
        const AP_Param::ConversionInfo notchfilt_conversion_info[] {
            // PARAMETER_CONVERSION - Added: Apr 2022
            { Parameters::k_param_ins, 101, AP_PARAM_INT8,  "INS_HNTC2_ENABLE" },
            { Parameters::k_param_ins, 293, AP_PARAM_FLOAT, "INS_HNTC2_ATT" },
            { Parameters::k_param_ins, 357, AP_PARAM_FLOAT, "INS_HNTC2_FREQ" },
            { Parameters::k_param_ins, 421, AP_PARAM_FLOAT, "INS_HNTC2_BW" },
        };
        AP_Param::convert_old_parameters(&notchfilt_conversion_info[0], ARRAY_SIZE(notchfilt_conversion_info));
        
        // Set sensible defaults for converted fixed notch parameters
        AP_Param::set_default_by_name("INS_HNTC2_MODE", 0);    // Fixed frequency mode
        AP_Param::set_default_by_name("INS_HNTC2_HMNCS", 1);   // Track 1st harmonic only
    }
#endif
#endif  // AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED

    // ========================================================================
    // Acro Mode Rate Parameter Conversions
    // ========================================================================
    
    // Acro mode parameter naming changed from P gain (ACRO_RP_P, ACRO_Y_P)
    // to rate in deg/s (ACRO_RP_RATE, ACRO_Y_RATE) for Copter-4.2. This makes
    // the parameter meaning clearer to users (direct deg/s instead of gain).
    // Scaling factor 45.0 converts old P gain to equivalent deg/s rate.
    
    // PARAMETER_CONVERSION - Added: Sep-2021
    const AP_Param::ConversionInfo acro_rpy_conversion_info[] = {
        { Parameters::k_param_acro_rp_p, 0, AP_PARAM_FLOAT, "ACRO_RP_RATE" },
        { Parameters::k_param_acro_yaw_p,  0, AP_PARAM_FLOAT, "ACRO_Y_RATE" }
    };
    for (const auto &info : acro_rpy_conversion_info) {
        // Scale by 45.0: old P gain of 4.5 becomes 202.5 deg/s
        AP_Param::convert_old_parameter(&info, 45.0);
    }

    // ========================================================================
    // Command Model Parameter Relocations
    // ========================================================================
    
    // Command model parameters (rate, expo, time constant) moved to dedicated
    // AC_CommandModel sub-objects in Copter-4.3 for better organization and
    // to support independent tuning of different control axes.
    // No scaling needed - values map 1:1 to new locations.
    
    // PARAMETER_CONVERSION - Added: June-2022
    const AP_Param::ConversionInfo cmd_mdl_conversion_info[] = {
        { Parameters::k_param_g2, 47, AP_PARAM_FLOAT, "ACRO_RP_RATE" },
        { Parameters::k_param_acro_rp_expo,  0, AP_PARAM_FLOAT, "ACRO_RP_EXPO" },
        { Parameters::k_param_g2,  48, AP_PARAM_FLOAT, "ACRO_Y_RATE" },
        { Parameters::k_param_g2,  9, AP_PARAM_FLOAT, "ACRO_Y_EXPO" },
        { Parameters::k_param_g2,  49, AP_PARAM_FLOAT, "PILOT_Y_RATE" },
        { Parameters::k_param_g2,  50, AP_PARAM_FLOAT, "PILOT_Y_EXPO" },
    };
    for (const auto &info : cmd_mdl_conversion_info) {
        // No scaling - 1:1 conversion
        AP_Param::convert_old_parameter(&info, 1.0);
    }

    // ========================================================================
    // Servo Channel Parameter Upgrades
    // ========================================================================
    
    // SRV_Channels library handles its own parameter upgrades for servo
    // function assignments, reversing, trim, min/max values, etc.
    // This call must occur after all other conversions complete.
    SRV_Channels::upgrade_parameters();
}

/**
 * @brief Convert proximity sensor parameters for multi-sensor support
 * 
 * @details Converts single-sensor proximity parameters (PRX_*) to multi-sensor
 *          format (PRX1_*, PRX2_*, etc.) to support multiple proximity sensors
 *          on a single vehicle. This conversion was added in Copter-4.3 when
 *          proximity sensor library gained support for multiple simultaneous
 *          sensors (e.g., multiple lidars covering different directions).
 * 
 *          The conversion handles all proximity sensor configuration:
 *          - Sensor type and orientation
 *          - Yaw correction angle
 *          - Ignore zones (up to 4 zones per sensor)
 *          - Minimum and maximum detection distances
 * 
 *          Old parameter format:
 *            PRX_TYPE, PRX_ORIENT, PRX_YAW_CORR, PRX_IGN_ANG1, etc.
 * 
 *          New parameter format:
 *            PRX1_TYPE, PRX1_ORIENT, PRX1_YAW_CORR, PRX1_IGN_ANG1, etc.
 *            PRX2_TYPE, PRX2_ORIENT, PRX2_YAW_CORR, PRX2_IGN_ANG1, etc.
 * 
 *          Existing single-sensor configurations automatically become PRX1_*
 *          (first sensor) after conversion, preserving all user settings.
 * 
 * @note Only compiled when HAL_PROXIMITY_ENABLED is true (boards with
 *       sufficient memory and CPU for proximity sensor support).
 * 
 * @note Called during initialization after convert_pid_parameters() if
 *       proximity sensor support is enabled in the build.
 * 
 * @see AP_Proximity - Proximity sensor library documentation
 * @see AP_Param::convert_old_parameter() - Parameter conversion implementation
 * 
 * PARAMETER_CONVERSION - Added: Aug-2022 for Copter-4.3
 */
#if HAL_PROXIMITY_ENABLED
void Copter::convert_prx_parameters()
{
    // Convert all proximity sensor parameters from old single-sensor format
    // (PRX_*) to new multi-sensor format (PRX1_*). No scaling needed - all
    // parameter values transfer directly (1.0 scaling factor).
    
    static const AP_Param::ConversionInfo prx_conversion_info[] = {
        // Sensor type (lidar, sonar, etc.) and physical orientation
        { Parameters::k_param_g2, 72, AP_PARAM_INT8, "PRX1_TYPE" },
        { Parameters::k_param_g2, 136, AP_PARAM_INT8, "PRX1_ORIENT" },
        
        // Yaw correction angle - compensates for sensor mounting misalignment
        { Parameters::k_param_g2, 200, AP_PARAM_INT16, "PRX1_YAW_CORR" },
        
        // Ignore zone 1: angle and width for obstacles to ignore (vehicle arms, landing gear, etc.)
        { Parameters::k_param_g2, 264, AP_PARAM_INT16, "PRX1_IGN_ANG1" },
        { Parameters::k_param_g2, 328, AP_PARAM_INT8, "PRX1_IGN_WID1" },
        
        // Ignore zone 2
        { Parameters::k_param_g2, 392, AP_PARAM_INT16, "PRX1_IGN_ANG2" },
        { Parameters::k_param_g2, 456, AP_PARAM_INT8, "PRX1_IGN_WID2" },
        
        // Ignore zone 3
        { Parameters::k_param_g2, 520, AP_PARAM_INT16, "PRX1_IGN_ANG3" },
        { Parameters::k_param_g2, 584, AP_PARAM_INT8, "PRX1_IGN_WID3" },
        
        // Ignore zone 4
        { Parameters::k_param_g2, 648, AP_PARAM_INT16, "PRX1_IGN_ANG4" },
        { Parameters::k_param_g2, 712, AP_PARAM_INT8, "PRX1_IGN_WID4" },
        
        // Detection distance limits (meters)
        { Parameters::k_param_g2, 1224, AP_PARAM_FLOAT, "PRX1_MIN" },
        { Parameters::k_param_g2, 1288, AP_PARAM_FLOAT, "PRX1_MAX" },
    };
    
    // Convert all parameters with 1:1 mapping (no scaling)
    for (const auto &info : prx_conversion_info) {
        AP_Param::convert_old_parameter(&info, 1.0);
    }
}
#endif  // HAL_PROXIMITY_ENABLED
