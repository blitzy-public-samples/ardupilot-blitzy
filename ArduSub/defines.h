/**
 * @file defines.h
 * @brief ArduSub constant definitions and enumerations
 * 
 * @details This file contains all compile-time constants, enumerations, and 
 *          preprocessor definitions specific to the ArduSub underwater vehicle.
 *          Includes underwater-specific definitions such as:
 *          - Surface and bottom detection thresholds
 *          - Leak detection failsafe parameters
 *          - Internal pressure and temperature monitoring
 *          - Depth hold and buoyancy control modes
 *          - Underwater-specific failsafe actions
 *          
 *          Many definitions are shared with other ArduPilot vehicles but adapted
 *          for underwater operation where appropriate.
 * 
 * @note ArduSub operates in NED (North-East-Down) coordinate frame where Down is positive
 * @warning Modifying failsafe thresholds can affect vehicle safety in underwater environments
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot Development Team
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

//////////////////////////////////////////////////////////////////////////////
/// @name Underwater Surface Detection Constants
/// @{

/// @brief Time threshold for bottom contact detection
/// @details Continuous bottom contact for this duration triggers bottom detector
/// @units seconds
#define BOTTOM_DETECTOR_TRIGGER_SEC 1.0

/// @brief Time threshold for surface breach detection
/// @details Continuous surface contact for this duration triggers surface detector
/// @units seconds
#define SURFACE_DETECTOR_TRIGGER_SEC 1.0

/// @}

/**
 * @enum AutoSurfaceState
 * @brief State machine for automatic surfacing operations
 * 
 * @details Defines the stages of the auto-surface sequence for ArduSub.
 *          The vehicle first navigates horizontally to a safe surface location,
 *          then performs a vertical ascent to break the surface.
 * 
 * @note Used during emergency surface operations and controlled ascents
 */
enum AutoSurfaceState {
    AUTO_SURFACE_STATE_GO_TO_LOCATION,  ///< Navigate horizontally to target surface location
    AUTO_SURFACE_STATE_ASCEND           ///< Perform vertical ascent to surface
};

/**
 * @enum autopilot_yaw_mode
 * @brief Yaw control mode selection for autonomous operations
 * 
 * @details Defines how the autopilot controls vehicle heading (yaw) during
 *          autonomous flight modes. Different modes allow for mission-based
 *          heading control, pilot override, or camera/ROI pointing.
 *          
 *          These modes are used in AUTO, GUIDED, and RTL flight modes to
 *          determine yaw behavior during waypoint navigation and other
 *          autonomous operations.
 * 
 * @note In underwater operations, yaw control helps maintain camera orientation
 *       and reduces drag during forward motion
 */
enum autopilot_yaw_mode {
    AUTO_YAW_HOLD =              0,  ///< Pilot controls the heading (pilot input accepted)
    AUTO_YAW_LOOK_AT_NEXT_WP =   1,  ///< Point towards next waypoint (no pilot input accepted)
    AUTO_YAW_ROI =               2,  ///< Point towards Region of Interest location held in roi_WP (no pilot input accepted)
    AUTO_YAW_LOOK_AT_HEADING =   3,  ///< Point towards a particular angle in degrees (no pilot input accepted)
    AUTO_YAW_LOOK_AHEAD =        4,  ///< Point in the direction the vehicle is moving (velocity-based heading)
    AUTO_YAW_RESETTOARMEDYAW =   5,  ///< Point towards heading at time motors were armed
    AUTO_YAW_CORRECT_XTRACK =    6,  ///< Steer the sub to correct for crosstrack error during line following
    AUTO_YAW_RATE =              7   ///< Steer the sub with the desired yaw rate (deg/s control)
};

//////////////////////////////////////////////////////////////////////////////
/// @name Acro Trainer Mode Definitions
/// @brief Training assistance levels for acrobatic flight mode
/// @{

/// @brief Acro trainer disabled - full manual control
/// @details Pilot has complete control with no stabilization assistance
#define ACRO_TRAINER_DISABLED   0

/// @brief Acro trainer with automatic leveling
/// @details System will automatically level the vehicle when sticks are centered
#define ACRO_TRAINER_LEVELING   1

/// @brief Acro trainer with angle limits
/// @details Limits maximum roll and pitch angles to prevent excessive attitudes
#define ACRO_TRAINER_LIMITED    2

/// @}

//////////////////////////////////////////////////////////////////////////////
/// @name Waypoint Yaw Behavior Definitions
/// @brief Yaw control behavior during mission execution
/// @details Possible values for WP_YAW_BEHAVIOR parameter
/// @{

/// @brief No automatic yaw control during missions
/// @details Autopilot will never control yaw during missions or RTL
///          (except for explicit DO_CONDITIONAL_YAW commands)
#define WP_YAW_BEHAVIOR_NONE                          0

/// @brief Face next waypoint or home
/// @details Autopilot will point towards next waypoint during missions
///          or towards home during RTL
#define WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP               1

/// @brief Face next waypoint except during RTL
/// @details Autopilot will face next waypoint during missions but
///          maintains last heading during RTL
#define WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL    2

/// @brief Look ahead in direction of travel
/// @details Autopilot will point in direction of velocity vector
///          (primarily meant for traditional helicopters)
#define WP_YAW_BEHAVIOR_LOOK_AHEAD                    3

/// @brief Correct for crosstrack error
/// @details Point towards intermediate position target to minimize
///          perpendicular deviation from desired track
#define WP_YAW_BEHAVIOR_CORRECT_XTRACK                4

/// @}



/**
 * @enum LoggingParameters
 * @brief Vehicle-specific logging message type identifiers
 * 
 * @details Defines custom log message types specific to ArduSub.
 *          Only 32 message slots are available for vehicle-specific messages.
 *          These supplement the common logging messages defined in AP_Logger.
 *          
 *          Used for control tuning data, debug values, and guided mode targets.
 * 
 * @note Message IDs are mapped to binary log structures in Log.cpp
 * @see AP_Logger for common message types shared across all vehicles
 */
enum LoggingParameters {
    LOG_CONTROL_TUNING_MSG,     ///< Control loop tuning data (PID values, desired vs actual)
    LOG_DATA_INT16_MSG,         ///< Generic 16-bit signed integer debug data
    LOG_DATA_UINT16_MSG,        ///< Generic 16-bit unsigned integer debug data
    LOG_DATA_INT32_MSG,         ///< Generic 32-bit signed integer debug data
    LOG_DATA_UINT32_MSG,        ///< Generic 32-bit unsigned integer debug data
    LOG_DATA_FLOAT_MSG,         ///< Generic floating-point debug data
    LOG_GUIDEDTARGET_MSG        ///< Guided mode target position/velocity data
};

//////////////////////////////////////////////////////////////////////////////
/// @name Log Message Bitmask Definitions
/// @brief Bitmask flags to enable/disable specific log message types
/// @details Used with LOG_BITMASK parameter to selectively enable logging.
///          Multiple masks can be OR'd together to enable multiple message types.
/// @{

#define MASK_LOG_ATTITUDE_FAST          (1<<0)      ///< High-rate attitude logging (quaternion, rates)
#define MASK_LOG_ATTITUDE_MED           (1<<1)      ///< Medium-rate attitude logging
#define MASK_LOG_GPS                    (1<<2)      ///< GPS position, velocity, and status
#define MASK_LOG_PM                     (1<<3)      ///< Performance monitoring (CPU load, memory)
#define MASK_LOG_CTUN                   (1<<4)      ///< Control tuning (desired vs achieved control)
#define MASK_LOG_NTUN                   (1<<5)      ///< Navigation tuning (position/velocity control)
#define MASK_LOG_RCIN                   (1<<6)      ///< RC input channel values
#define MASK_LOG_IMU                    (1<<7)      ///< IMU sensor data (accel, gyro)
#define MASK_LOG_CMD                    (1<<8)      ///< Mission command execution
#define MASK_LOG_CURRENT                (1<<9)      ///< Battery current measurement
#define MASK_LOG_RCOUT                  (1<<10)     ///< Servo/motor output values
#define MASK_LOG_OPTFLOW                (1<<11)     ///< Optical flow sensor data
#define MASK_LOG_PID                    (1<<12)     ///< PID controller internal values
#define MASK_LOG_COMPASS                (1<<13)     ///< Compass/magnetometer readings
#define MASK_LOG_CAMERA                 (1<<15)     ///< Camera trigger events
#define MASK_LOG_MOTBATT                (1UL<<17)   ///< Motor battery voltage/current
#define MASK_LOG_IMU_FAST               (1UL<<18)   ///< High-rate IMU logging for vibration analysis
#define MASK_LOG_IMU_RAW                (1UL<<19)   ///< Raw IMU sensor data (pre-filtering)
#define MASK_LOG_ANY                    0xFFFF      ///< Enable all logging (use with caution - high CPU/storage load)

/// @}

//////////////////////////////////////////////////////////////////////////////
/// @name GCS Failsafe Configuration
/// @brief Ground Control Station communication loss failsafe
/// @{

/// @brief Default GCS failsafe enable state
/// @details Set to 0 to disable GCS failsafe by default
#ifndef FS_GCS
# define FS_GCS                        0
#endif

/// @brief GCS heartbeat timeout threshold
/// @details Failsafe triggers after this duration without receiving GCS heartbeat
/// @units seconds
/// @warning Critical for maintaining communication link with surface control station
#ifndef FS_GCS_TIMEOUT_S
# define FS_GCS_TIMEOUT_S             5.0
#endif

/// @}

//////////////////////////////////////////////////////////////////////////////
/// @name Terrain Data Failsafe Configuration  
/// @brief Missing terrain/altitude data failsafe
/// @{

/// @brief Timeout for terrain data validity
/// @details Failsafe triggers after this duration with unhealthy rangefinder
///          and/or missing terrain database information
/// @units milliseconds
/// @note Particularly important for terrain-following missions underwater
#ifndef FS_TERRAIN_TIMEOUT_MS
#define FS_TERRAIN_TIMEOUT_MS          1000
#endif

/// @}

//////////////////////////////////////////////////////////////////////////////
/// @name EKF Failsafe Configuration
/// @brief Extended Kalman Filter health monitoring and failsafe actions
/// @details Monitors EKF variance and innovation consistency to detect
///          navigation filter degradation
/// @{

/// @name EKF Failsafe Action Definitions
/// @brief Possible actions when EKF variance exceeds threshold (FS_EKF_ENABLE parameter)
/// @{

/// @brief EKF failsafe disabled
/// @details No action taken when EKF variance increases
#define FS_EKF_ACTION_DISABLED                0

/// @brief Warn only on EKF issues
/// @details Send warning message to ground control station but continue operation
#define FS_EKF_ACTION_WARN_ONLY               1

/// @brief Disarm on EKF failure
/// @details Immediately disarm vehicle when EKF variance exceeds threshold
/// @warning Not recommended for underwater operation - vehicle will sink
#define FS_EKF_ACTION_DISARM                  2

/// @}

/// @brief Default EKF failsafe action
/// @details Disabled by default for underwater vehicles to prevent unexpected disarm
#ifndef FS_EKF_ACTION_DEFAULT
# define FS_EKF_ACTION_DEFAULT         FS_EKF_ACTION_DISABLED
#endif

/// @brief Default EKF variance threshold
/// @details Compass and velocity variance threshold above which failsafe triggers
/// @units dimensionless (normalized variance)
/// @note Lower values are more sensitive to EKF degradation
#ifndef FS_EKF_THRESHOLD_DEFAULT
# define FS_EKF_THRESHOLD_DEFAULT      0.8f
#endif

/// @}

//////////////////////////////////////////////////////////////////////////////
/// @name GCS Failsafe Action Definitions
/// @brief Ground Control Station failsafe response actions (FS_GCS_ENABLE parameter)
/// @{

/// @brief GCS failsafe disabled
#define FS_GCS_DISABLED     0

/// @brief Warn only on GCS communication loss
/// @details Send warning to all connected GCS links (useful with multiple telemetry links)
#define FS_GCS_WARN_ONLY    1

/// @brief Disarm on GCS communication loss
/// @warning Not recommended underwater - vehicle will sink when disarmed
#define FS_GCS_DISARM       2

/// @brief Hold position on GCS communication loss
/// @details Switch to depth hold mode or position hold mode if available
/// @note Safe option for maintaining depth while communication is restored
#define FS_GCS_HOLD         3

/// @brief Surface on GCS communication loss
/// @details Execute automatic surfacing sequence to restore communication
/// @note Recommended failsafe action for underwater operations
#define FS_GCS_SURFACE      4

/// @}

//////////////////////////////////////////////////////////////////////////////
/// @name Leak Detection Failsafe Definitions (UNDERWATER-SPECIFIC)
/// @brief Water ingress detection and response actions (FS_LEAK_ENABLE parameter)
/// @details Triggered by leak detector sensors inside watertight enclosure
/// @warning Critical safety feature for underwater vehicles
/// @{

/// @brief Leak failsafe disabled
/// @details No action taken when leak is detected (not recommended)
#define FS_LEAK_DISABLED    0

/// @brief Warn only on leak detection
/// @details Send warning to GCS but continue operation
/// @note Allows pilot to assess situation before taking action
#define FS_LEAK_WARN_ONLY   1

/// @brief Surface immediately on leak detection
/// @details Execute automatic surfacing sequence to minimize water damage
/// @note Recommended setting - brings vehicle to surface for recovery
#define FS_LEAK_SURFACE     2

/// @}

//////////////////////////////////////////////////////////////////////////////
/// @name Internal Pressure Failsafe (UNDERWATER-SPECIFIC)
/// @brief Enclosure internal pressure monitoring and limits
/// @details Monitors pressure inside electronics enclosure to detect seal failure
/// @warning Excessive internal pressure indicates potential seal failure
/// @{

/// @brief Default maximum internal pressure threshold (FS_PRESS_MAX parameter)
/// @details Failsafe triggers when internal pressure exceeds this value
/// @units Pascal
/// @note Default is approximately 1.05 atmospheres (slightly above sea level pressure)
#define FS_PRESS_MAX_DEFAULT    105000

/// @name Internal Pressure Failsafe Actions (FS_PRESS_ENABLE parameter)
/// @{

/// @brief Internal pressure failsafe disabled
#define FS_PRESS_DISABLED       0

/// @brief Warn only on high internal pressure
/// @details Send warning to GCS when internal pressure exceeds threshold
#define FS_PRESS_WARN_ONLY      1

/// @}
/// @}

//////////////////////////////////////////////////////////////////////////////
/// @name Internal Temperature Failsafe (UNDERWATER-SPECIFIC)
/// @brief Enclosure internal temperature monitoring and limits
/// @details Monitors temperature inside electronics enclosure to prevent overheating
/// @warning Excessive temperature can damage electronics and batteries
/// @{

/// @brief Default maximum internal temperature threshold (FS_TEMP_MAX parameter)
/// @details Failsafe triggers when internal temperature exceeds this value
/// @units degrees Celsius
/// @note 62°C is below typical component thermal limits but provides safety margin
#define FS_TEMP_MAX_DEFAULT     62

/// @name Internal Temperature Failsafe Actions (FS_TEMP_ENABLE parameter)
/// @{

/// @brief Internal temperature failsafe disabled
#define FS_TEMP_DISABLED        0

/// @brief Warn only on high internal temperature
/// @details Send warning to GCS when internal temperature exceeds threshold
#define FS_TEMP_WARN_ONLY       1

/// @}
/// @}

//////////////////////////////////////////////////////////////////////////////
/// @name Crash Detection Failsafe
/// @brief Vehicle collision/impact detection and response
/// @details Detects sudden deceleration or attitude changes indicating collision
/// @{

/// @brief Crash detection disabled
#define FS_CRASH_DISABLED  0

/// @brief Warn only on crash detection
/// @details Send warning to GCS but continue operation
#define FS_CRASH_WARN_ONLY 1

/// @brief Disarm on crash detection
/// @details Immediately disarm vehicle to prevent propeller damage
/// @warning Underwater disarm will cause vehicle to sink - use with caution
#define FS_CRASH_DISARM    2

/// @}

//////////////////////////////////////////////////////////////////////////////
/// @name Terrain Failsafe Actions
/// @brief Response actions when terrain data is unavailable during AUTO mode
/// @details Triggered when rangefinder fails or terrain database is missing
/// @{

/// @brief Disarm on terrain data loss
/// @warning Not recommended underwater - vehicle will sink
#define FS_TERRAIN_DISARM       0

/// @brief Hold position on terrain data loss
/// @details Maintain current depth and position until data is restored
#define FS_TERRAIN_HOLD         1

/// @brief Surface on terrain data loss
/// @details Execute surfacing sequence to avoid terrain collision
/// @note Recommended for terrain-following missions
#define FS_TERRAIN_SURFACE      2

/// @}

//////////////////////////////////////////////////////////////////////////////
/// @name Pilot Input Failsafe
/// @brief Detection and response to loss of pilot input
/// @details Monitors RC input for loss of control signal
/// @{

/// @brief Pilot input failsafe disabled
#define FS_PILOT_INPUT_DISABLED    0

/// @brief Warn only on pilot input loss
/// @details Send warning to GCS but continue operation
#define FS_PILOT_INPUT_WARN_ONLY   1

/// @brief Disarm on pilot input loss
/// @details Immediately disarm vehicle when RC signal is lost
/// @warning Underwater disarm will cause vehicle to sink
#define FS_PILOT_INPUT_DISARM      2

/// @}

//////////////////////////////////////////////////////////////////////////////
/// @name Throttle Arming Position Configuration
/// @brief Throttle position requirement for arming (THR_ARM_POS parameter)
/// @{

/// @brief Throttle must be within trim range
/// @details Throttle stick must be near center position to arm
/// @note Prevents accidental arming with throttle applied
#define WITHIN_THR_TRIM 1

/// @}

//////////////////////////////////////////////////////////////////////////////
/// @name Radio Failsafe Definitions
/// @brief RC receiver signal loss detection and response (FS_THR parameter)
/// @details Triggered when RC receiver loses connection to transmitter
/// @{

/// @brief Radio failsafe disabled
#define FS_THR_DISABLED                            0

/// @brief Warn on radio signal loss
/// @details Send warning to GCS but continue operation
#define FS_THR_WARN                                1

/// @brief Surface on radio signal loss
/// @details Execute automatic surfacing sequence
/// @note Recommended setting for underwater operations
#define FS_THR_SURFACE                             2

/// @}

//////////////////////////////////////////////////////////////////////////////
/// @name Terrain Failsafe Recovery
/// @brief Terrain data recovery timeout configuration
/// @{

/// @brief Terrain data recovery timeout
/// @details Duration to attempt recovery of valid rangefinder data before
///          initiating terrain failsafe action
/// @units milliseconds
/// @note Provides time for temporary sensor glitches to clear
#define FS_TERRAIN_RECOVER_TIMEOUT_MS 10000

/// @}

//////////////////////////////////////////////////////////////////////////////
/// @name MAVLink Position Target Type Masks
/// @brief Bitmask definitions for MAVLink SET_POSITION_TARGET messages
/// @details Used in GUIDED mode to specify which fields of position/velocity
///          commands should be ignored. Allows selective control of position,
///          velocity, acceleration, and yaw components.
/// @see MAVLink SET_POSITION_TARGET_LOCAL_NED message definition
/// @{

/// @brief Ignore Z-axis (depth) component
/// @details When set, vertical position/velocity is not controlled
#define MAVLINK_SET_POS_TYPE_MASK_Z_IGNORE        (1<<2)

/// @brief Ignore all position components (X, Y, Z)
/// @details Mask bits 0-2 to disable position control
#define MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE      ((1<<0) | (1<<1) | (1<<2))

/// @brief Ignore all velocity components (VX, VY, VZ)
/// @details Mask bits 3-5 to disable velocity control
#define MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE      ((1<<3) | (1<<4) | (1<<5))

/// @brief Ignore all acceleration components (AX, AY, AZ)
/// @details Mask bits 6-8 to disable acceleration feedforward
#define MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE      ((1<<6) | (1<<7) | (1<<8))

/// @brief Interpret as force commands
/// @details When set, position/velocity values are treated as force setpoints
#define MAVLINK_SET_POS_TYPE_MASK_FORCE           (1<<9)

/// @brief Ignore yaw angle component
/// @details When set, yaw setpoint is not controlled
#define MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE      (1<<10)

/// @brief Ignore yaw rate component
/// @details When set, yaw rate setpoint is not controlled
#define MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE (1<<11)

/// @}

