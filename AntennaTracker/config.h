/**
 * @file config.h
 * @brief Compile-time configuration defaults for antenna tracker
 * 
 * @details This file defines default values for antenna tracker configuration parameters
 *          including MAVLink settings, servo ranges, tracking timeouts, and logging options.
 *          These defaults are used unless overridden by a custom APM_Config.h file.
 * 
 * @note Values defined here can be overridden by creating a custom APM_Config.h file
 *       in the AntennaTracker directory with your desired settings.
 * @note These defaults are suitable for most antenna tracker installations with
 *       standard 360-degree yaw rotation and ±90-degree pitch tilt.
 * @note Adjust YAW_RANGE_DEFAULT and PITCH_MIN/MAX_DEFAULT to match your specific
 *       hardware servo limits and mechanical constraints.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2013-2025 ArduPilot.org
 * 
 * Source: AntennaTracker/config.h
 */
#pragma once

#include "defines.h"

//////////////////////////////////////////////////////////////////////////////
// MAVLink Configuration
//
/**
 * @brief Default MAVLink system ID for antenna tracker
 * 
 * @details Set to 2 by default to avoid conflicts with typical vehicle system IDs.
 *          Most vehicles use system ID 1, so antenna tracker uses 2 to maintain
 *          separate communication streams on the same MAVLink network.
 * 
 * @note Can be overridden at runtime via MAV_SYSID parameter or at compile-time
 *       by defining MAV_SYSTEM_ID in APM_Config.h
 */
#ifndef MAV_SYSTEM_ID
 // use 2 for antenna tracker by default
 # define MAV_SYSTEM_ID          2
#endif


//////////////////////////////////////////////////////////////////////////////
// RC Channel definitions
//
/**
 * @brief Default RC channel assignments for antenna tracker control
 * 
 * @details Defines which RC channels control yaw (pan) and pitch (tilt) movements.
 *          These are used both for RC input (manual control) and servo output.
 */

/**
 * @brief RC channel for yaw (pan) control
 * 
 * @details Channel 1 is used by default for yaw axis control. This channel receives
 *          RC input for manual yaw control and outputs servo commands for yaw movement.
 */
#ifndef CH_YAW
 # define CH_YAW        CH_1    // RC input/output for yaw on channel 1
#endif

/**
 * @brief RC channel for pitch (tilt) control
 * 
 * @details Channel 2 is used by default for pitch axis control. This channel receives
 *          RC input for manual pitch control and outputs servo commands for pitch movement.
 */
#ifndef CH_PITCH
 # define CH_PITCH      CH_2    // RC input/output for pitch on channel 2
#endif


//////////////////////////////////////////////////////////////////////////////
// Servo Defaults - Yaw and Pitch Axis Angle Range
//
/**
 * @brief Default yaw servo range in degrees
 * 
 * @details Defines the total rotation range for the yaw (pan) axis. Default is 360 degrees
 *          for continuous rotation trackers. Some installations may use 180 degrees for
 *          limited rotation hardware.
 * 
 * @note Adjust this value to match your antenna tracker's mechanical yaw range.
 *       Common values: 360 for continuous rotation, 180 for limited rotation.
 * @warning Setting this beyond your hardware's physical limits may cause mechanical damage.
 * 
 * @units degrees
 */
#ifndef YAW_RANGE_DEFAULT
 # define YAW_RANGE_DEFAULT 360
#endif

/**
 * @brief Default minimum pitch angle in degrees
 * 
 * @details Defines the lower limit for pitch (tilt) angle. Default is -90 degrees
 *          (pointing straight down). This prevents the antenna from tilting beyond
 *          mechanical limits.
 * 
 * @note Adjust this value to match your antenna tracker's minimum safe pitch angle.
 *       Typical range: -90 to 0 degrees depending on mechanical constraints.
 * @warning Setting this beyond your hardware's physical limits may cause mechanical damage.
 * 
 * @units degrees
 */
#ifndef PITCH_MIN_DEFAULT
 # define PITCH_MIN_DEFAULT -90
#endif

/**
 * @brief Default maximum pitch angle in degrees
 * 
 * @details Defines the upper limit for pitch (tilt) angle. Default is 90 degrees
 *          (pointing straight up). This prevents the antenna from tilting beyond
 *          mechanical limits.
 * 
 * @note Adjust this value to match your antenna tracker's maximum safe pitch angle.
 *       Typical range: 0 to 90 degrees depending on mechanical constraints.
 * @warning Setting this beyond your hardware's physical limits may cause mechanical damage.
 * 
 * @units degrees
 */
#ifndef PITCH_MAX_DEFAULT
 # define PITCH_MAX_DEFAULT 90
#endif

//////////////////////////////////////////////////////////////////////////////
// Tracking Definitions - Timing and Distance Defaults
//
/**
 * @brief Vehicle tracking timeout in milliseconds
 * 
 * @details Time period after which the tracker considers it has lost track of the vehicle
 *          if no position update is received. Used primarily for updating armed/disarmed
 *          status LEDs and user interface indicators.
 * 
 * @note This timeout helps indicate communication loss to the operator through visual feedback.
 * @units milliseconds
 */
#ifndef TRACKING_TIMEOUT_MS
 # define TRACKING_TIMEOUT_MS               5000    // consider we've lost track of vehicle after 5 seconds with no position update.  Used to update armed/disarmed status leds
#endif

/**
 * @brief Vehicle tracking timeout in seconds (floating point)
 * 
 * @details Time period after which the tracker considers it has lost track of the vehicle
 *          if no position update is received. This is the floating-point version used
 *          in calculations requiring sub-second precision.
 * 
 * @note Should be kept synchronized with TRACKING_TIMEOUT_MS (same duration, different units).
 * @units seconds
 */
#ifndef TRACKING_TIMEOUT_SEC
 # define TRACKING_TIMEOUT_SEC              5.0f    // consider we've lost track of vehicle after 5 seconds with no position update.
#endif

/**
 * @brief Minimum tracking distance threshold
 * 
 * @details Defines the minimum distance at which the tracker will track a target.
 *          Targets closer than this distance are ignored to prevent erratic servo
 *          movements when the vehicle is very close or directly overhead.
 * 
 * @note This prevents servo jitter and excessive wear when vehicle is at close range.
 * @warning Setting this too low may cause rapid servo oscillations when vehicle is nearby.
 * @units meters
 */
#ifndef DISTANCE_MIN_DEFAULT
 # define DISTANCE_MIN_DEFAULT              5.0f    // do not track targets within 5 meters
#endif

//////////////////////////////////////////////////////////////////////////////
// Logging Control
//
/**
 * @brief Default set of log messages to record
 * 
 * @details Defines which log message types are enabled by default for antenna tracker.
 *          This bitmask enables logging of:
 *          - MASK_LOG_ATTITUDE: Vehicle attitude (roll, pitch, yaw)
 *          - MASK_LOG_GPS: GPS position and status
 *          - MASK_LOG_RCIN: RC input channels
 *          - MASK_LOG_IMU: Inertial Measurement Unit data (accelerometer, gyroscope)
 *          - MASK_LOG_RCOUT: Servo output commands
 *          - MASK_LOG_COMPASS: Magnetometer/compass data
 *          - MASK_LOG_CURRENT: Battery voltage and current measurements
 * 
 * @note These log messages provide essential data for debugging tracker behavior,
 *       analyzing servo performance, and reviewing tracking accuracy.
 * @note Can be modified at runtime via the LOG_BITMASK parameter or at compile-time
 *       by defining DEFAULT_LOG_BITMASK in APM_Config.h
 * @note Additional log types can be enabled through parameter settings.
 */
#ifndef DEFAULT_LOG_BITMASK
 # define DEFAULT_LOG_BITMASK \
    MASK_LOG_ATTITUDE | \
    MASK_LOG_GPS | \
    MASK_LOG_RCIN | \
    MASK_LOG_IMU | \
    MASK_LOG_RCOUT | \
    MASK_LOG_COMPASS | \
    MASK_LOG_CURRENT
#endif

//////////////////////////////////////////////////////////////////////////////
// Feature Configuration
//
/**
 * @brief Enable setting tracker home position via mission upload
 * 
 * @details When enabled (1), allows the antenna tracker's home position to be set
 *          through MAVLink mission upload commands. When disabled (0), home position
 *          must be set through other means (e.g., parameter setting or on-site setup).
 * 
 * @note Enabling this feature is convenient for remote configuration but should be used
 *       with caution to ensure the correct physical location is set.
 * @note Default is enabled (1) for flexibility in deployment and configuration.
 */
#ifndef AP_TRACKER_SET_HOME_VIA_MISSION_UPLOAD_ENABLED
#define AP_TRACKER_SET_HOME_VIA_MISSION_UPLOAD_ENABLED 1
#endif
