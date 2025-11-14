/**
 * @file config.h
 * @brief Build-time configuration constants and default parameter values for ArduPlane
 * 
 * @details This file defines compile-time configuration constants for the ArduPlane
 *          fixed-wing autopilot. These values establish default parameter values,
 *          feature enables, and hardware-specific settings that are used when
 *          parameters have not been explicitly set by the user.
 *          
 *          Configuration categories include:
 *          - Hardware configuration and MAVLink system ID
 *          - Radio channel mapping and flight mode assignments
 *          - Flight control limits (roll, pitch, throttle)
 *          - Navigation parameters (waypoint radius, loiter radius)
 *          - Airspeed control limits
 *          - Servo mapping and control surface defaults
 *          - Failsafe thresholds
 *          - Logging configuration
 *          - Developer and debugging options
 *          
 *          All configuration macros use conditional compilation (#ifndef) to allow
 *          override at compile time via build system flags or hwdef files.
 *          
 * @note Values defined here are defaults only - most can be changed via parameters
 *       at runtime. These defaults are used for initial parameter initialization.
 *       
 * @warning Changing values in this file requires recompilation. Modify carefully
 *          as these affect default flight behavior for all ArduPlane installations
 *          unless overridden by user parameters.
 *          
 * @see defines.h for feature enable flags and conditional compilation
 * @see Parameters.cpp for runtime parameter definitions
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "defines.h"

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// HARDWARE CONFIGURATION AND CONNECTIONS
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

/**
 * @defgroup hardware_config Hardware Configuration
 * @brief Hardware-specific configuration and MAVLink system identification
 * @{
 */

/**
 * @brief Deprecated hardware configuration flag check
 * 
 * @details CONFIG_APM_HARDWARE is no longer supported. Modern ArduPilot uses
 *          CONFIG_HAL_BOARD for hardware abstraction layer selection.
 */
#ifdef CONFIG_APM_HARDWARE
#error CONFIG_APM_HARDWARE option is deprecated! use CONFIG_HAL_BOARD instead.
#endif

/**
 * @brief MAVLink system ID for this vehicle
 * 
 * @details Default MAVLink system ID used for identifying this vehicle in
 *          multi-vehicle networks. Must be unique among vehicles communicating
 *          on the same MAVLink network.
 *          
 * @note Can be overridden by SYSID_THISMAV parameter at runtime
 * @note Valid range: 1-255
 */
#ifndef MAV_SYSTEM_ID
 # define MAV_SYSTEM_ID          1
#endif

/** @} */ // end of hardware_config group

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// RADIO CONFIGURATION
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

/**
 * @defgroup radio_config Radio and Control Surface Configuration
 * @brief RC channel mapping and control surface defaults
 * @{
 */

/**
 * @brief Flap position 1 deflection percentage
 * @details Default deflection percentage for first flap position
 * @note 0 = no flaps deployed, 100 = full flap deflection
 * @note Can be overridden by FLAP_1_PERCNT parameter
 */
#ifndef FLAP_1_PERCENT
 # define FLAP_1_PERCENT 0
#endif

/**
 * @brief Airspeed threshold for automatic flap position 1
 * @details Airspeed (m/s) below which flaps automatically deploy to position 1
 * @note 0 = automatic flap deployment disabled
 * @note Can be overridden by FLAP_1_SPEED parameter
 */
#ifndef FLAP_1_SPEED
 # define FLAP_1_SPEED 0
#endif

/**
 * @brief Flap position 2 deflection percentage
 * @details Default deflection percentage for second flap position (typically full flaps)
 * @note 0 = no flaps deployed, 100 = full flap deflection
 * @note Can be overridden by FLAP_2_PERCNT parameter
 */
#ifndef FLAP_2_PERCENT
 # define FLAP_2_PERCENT 0
#endif

/**
 * @brief Airspeed threshold for automatic flap position 2
 * @details Airspeed (m/s) below which flaps automatically deploy to position 2
 * @note 0 = automatic flap deployment disabled
 * @note Can be overridden by FLAP_2_SPEED parameter
 */
#ifndef FLAP_2_SPEED
 # define FLAP_2_SPEED 0
#endif
/**
 * @brief RC channel used for flight mode selection
 * 
 * @details Specifies which RC input channel (5-8) is used for flight mode switching.
 *          The PWM value on this channel is mapped to one of six configurable flight
 *          modes based on predefined PWM ranges.
 *          
 * @note Default is channel 8 (typically a 3-position or 6-position switch)
 * @note Can be overridden by FLTMODE_CH parameter at runtime
 * @note Valid values: 5, 6, 7, or 8 only (enforced by compile-time check)
 * 
 * @warning Compilation will fail if FLIGHT_MODE_CHANNEL is not 5, 6, 7, or 8
 */
#ifndef FLIGHT_MODE_CHANNEL
 # define FLIGHT_MODE_CHANNEL    8
#endif

/**
 * @brief Compile-time validation of flight mode channel
 * @details Ensures FLIGHT_MODE_CHANNEL is set to a valid channel number (5-8)
 */
#if (FLIGHT_MODE_CHANNEL != 5) && (FLIGHT_MODE_CHANNEL != 6) && (FLIGHT_MODE_CHANNEL != 7) && (FLIGHT_MODE_CHANNEL != 8)
 # error XXX
 # error XXX You must set FLIGHT_MODE_CHANNEL to 5, 6, 7 or 8
 # error XXX
#endif

/**
 * @brief Default flight mode for position 1 (lowest PWM on mode channel)
 * @details Sets the flight mode activated when the mode channel PWM is in range 1 (< 1230μs)
 * @note Default: RTL (Return To Launch) for safety
 * @note Can be overridden by FLTMODE1 parameter at runtime
 */
#if !defined(FLIGHT_MODE_1)
 # define FLIGHT_MODE_1                  Mode::Number::RTL
#endif

/**
 * @brief Default flight mode for position 2
 * @details Sets the flight mode activated when the mode channel PWM is in range 2 (1230-1360μs)
 * @note Default: RTL (Return To Launch) for safety
 * @note Can be overridden by FLTMODE2 parameter at runtime
 */
#if !defined(FLIGHT_MODE_2)
 # define FLIGHT_MODE_2                  Mode::Number::RTL
#endif

/**
 * @brief Default flight mode for position 3
 * @details Sets the flight mode activated when the mode channel PWM is in range 3 (1360-1490μs)
 * @note Default: FLY_BY_WIRE_A (stabilized flight with pilot roll/pitch control)
 * @note Can be overridden by FLTMODE3 parameter at runtime
 */
#if !defined(FLIGHT_MODE_3)
 # define FLIGHT_MODE_3                  Mode::Number::FLY_BY_WIRE_A
#endif

/**
 * @brief Default flight mode for position 4
 * @details Sets the flight mode activated when the mode channel PWM is in range 4 (1490-1620μs)
 * @note Default: FLY_BY_WIRE_A (stabilized flight with pilot roll/pitch control)
 * @note Can be overridden by FLTMODE4 parameter at runtime
 */
#if !defined(FLIGHT_MODE_4)
 # define FLIGHT_MODE_4                  Mode::Number::FLY_BY_WIRE_A
#endif

/**
 * @brief Default flight mode for position 5
 * @details Sets the flight mode activated when the mode channel PWM is in range 5 (1620-1749μs)
 * @note Default: MANUAL (direct pilot control with no stabilization)
 * @note Can be overridden by FLTMODE5 parameter at runtime
 */
#if !defined(FLIGHT_MODE_5)
 # define FLIGHT_MODE_5                  Mode::Number::MANUAL
#endif

/**
 * @brief Default flight mode for position 6 (highest PWM on mode channel)
 * @details Sets the flight mode activated when the mode channel PWM is in range 6 (> 1749μs)
 * @note Default: MANUAL (direct pilot control with no stabilization)
 * @note Can be overridden by FLTMODE6 parameter at runtime
 */
#if !defined(FLIGHT_MODE_6)
 # define FLIGHT_MODE_6                  Mode::Number::MANUAL
#endif


/**
 * @brief Automatic trim adjustment during flight
 * 
 * @details Enables or disables automatic adjustment of control surface trim based
 *          on required control inputs during stabilized flight. When enabled, the
 *          autopilot learns the trim offsets needed to fly straight and level.
 *          
 * @note Default: DISABLED for safety (manual trim preferred)
 * @note Can be overridden by TRIM_AUTO parameter at runtime
 * 
 * @warning Enable only after initial trim is roughly correct to avoid
 *          dangerous control surface positions during learning
 */
#ifndef AUTO_TRIM
 # define AUTO_TRIM                              DISABLED
#endif

/** @} */ // end of radio_config group

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// STARTUP BEHAVIOUR
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

/**
 * @defgroup startup_config Startup Behavior Configuration
 * @brief Configuration constants affecting vehicle initialization and startup
 * @{
 */


/**
 * @brief Delay before ground start initialization begins
 * 
 * @details Time delay (in seconds) before the vehicle performs ground start
 *          initialization sequence. Used to allow sensors to stabilize after
 *          power-on before gyro and accelerometer calibration begins.
 *          
 * @note Default: 0 seconds (immediate ground start)
 * @note Increase if sensors need additional warmup time
 */
#ifndef GROUND_START_DELAY
 # define GROUND_START_DELAY             0
#endif

/**
 * @brief Default rudder mixing rate for differential spoilers
 * 
 * @details Percentage of rudder input to mix into differential spoiler control.
 *          Differential spoilers can be used for roll control and yaw damping
 *          by deflecting asymmetrically. This value sets how much rudder input
 *          is blended into the spoiler commands.
 *          
 * @note Default: 100% (full rudder mixing)
 * @note Valid range: 0-100 percent
 * @note Can be overridden by DSPOILR_RUD_RATE parameter
 */
#ifndef DSPOILR_RUD_RATE_DEFAULT
 #define DSPOILR_RUD_RATE_DEFAULT 100
#endif

/** @} */ // end of startup_config group

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// FLIGHT AND NAVIGATION CONTROL
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

/**
 * @defgroup flight_nav_config Flight and Navigation Control Limits
 * @brief Default values for flight control limits, speeds, and navigation parameters
 * @{
 */

/**
 * @brief Default cruise airspeed
 * 
 * @details Target airspeed for cruise flight in autonomous modes and altitude/airspeed
 *          control modes. This is the speed the aircraft will attempt to maintain
 *          during normal flight operations.
 *          
 * @note Default: 12 m/s (approximately 43 km/h or 27 mph)
 * @note Units: meters per second
 * @note Can be overridden by ARSPD_CRUISE parameter at runtime
 * @note Should be set based on aircraft's optimal cruise speed for efficiency
 */
#ifndef AIRSPEED_CRUISE
 # define AIRSPEED_CRUISE                12 // 12 m/s
#endif

/**
 * @brief Minimum groundspeed target
 * 
 * @details Minimum desired groundspeed in autonomous modes. When groundspeed falls
 *          below this value, the aircraft will increase throttle or reduce altitude
 *          to maintain forward progress. Useful for preventing stalls in headwinds.
 *          
 * @note Default: 0 m/s (feature disabled)
 * @note Units: meters per second
 * @note Set to 0 to disable groundspeed floor
 * @note Can be overridden by MIN_GNDSPD_CM parameter at runtime (in cm/s)
 * 
 * @warning Setting too high can cause excessive throttle in strong headwinds
 */
#ifndef MIN_GROUNDSPEED
 # define MIN_GROUNDSPEED                   0 // m/s (0 disables)
#endif

/**
 * @brief Minimum airspeed for FLY_BY_WIRE_B mode
 * 
 * @details Lower limit for pilot-commanded airspeed in FBWB mode. Prevents pilot
 *          from commanding dangerously low airspeeds that could result in stall.
 *          
 * @note Default: 9 m/s (approximately 32 km/h or 20 mph)
 * @note Units: meters per second
 * @note Can be overridden by ARSPD_FBW_MIN parameter
 * 
 * @warning Should be set above aircraft stall speed with adequate margin
 */
#ifndef AIRSPEED_FBW_MIN
 # define AIRSPEED_FBW_MIN               9
#endif

/**
 * @brief Maximum airspeed for FLY_BY_WIRE_B mode
 * 
 * @details Upper limit for pilot-commanded airspeed in FBWB mode. Prevents pilot
 *          from commanding excessive airspeeds that could damage the airframe.
 *          
 * @note Default: 22 m/s (approximately 79 km/h or 49 mph)
 * @note Units: meters per second
 * @note Can be overridden by ARSPD_FBW_MAX parameter
 * 
 * @warning Should be set below aircraft never-exceed speed (Vne)
 */
#ifndef AIRSPEED_FBW_MAX
 # define AIRSPEED_FBW_MAX               22
#endif

/**
 * @brief Minimum altitude floor for cruise
 * 
 * @details Minimum altitude (meters above home) for cruise operations. Aircraft will
 *          not descend below this altitude in cruise modes regardless of throttle.
 *          
 * @note Default: 0 meters (no altitude floor)
 * @note Units: meters above home
 * @note Can be overridden by ALT_CTRL_ALG parameter
 */
#ifndef CRUISE_ALT_FLOOR
 # define CRUISE_ALT_FLOOR 0
#endif


/**
 * @brief Minimum throttle output percentage
 * 
 * @details Lowest throttle output in autonomous modes. Prevents throttle from
 *          going completely to zero, which can cause engine flameout or loss
 *          of control authority.
 *          
 * @note Default: 0% (throttle can go to zero)
 * @note Units: percent (0-100)
 * @note Can be overridden by THR_MIN parameter
 * @note For combustion engines, typically set to 5-10% to maintain idle
 */
#ifndef THROTTLE_MIN
 # define THROTTLE_MIN                   0 // percent
#endif

/**
 * @brief Cruise throttle percentage
 * 
 * @details Default throttle output for maintaining cruise airspeed in level flight.
 *          This value is used as the starting point for throttle control and should
 *          be set to the throttle percentage that produces cruise speed.
 *          
 * @note Default: 45%
 * @note Units: percent (0-100)
 * @note Can be overridden by TRIM_THROTTLE parameter
 * @note Should be calibrated for each aircraft based on weight and power
 * 
 * @warning Incorrect value will cause TECS to work harder and may affect
 *          altitude and airspeed tracking performance
 */
#ifndef THROTTLE_CRUISE
 # define THROTTLE_CRUISE                45
#endif

/**
 * @brief Maximum throttle output percentage
 * 
 * @details Upper limit for throttle output in all flight modes. Used to limit
 *          maximum power and protect propulsion system.
 *          
 * @note Default: 100% (full throttle available)
 * @note Units: percent (0-100)
 * @note Can be overridden by THR_MAX parameter
 * @note May be reduced for over-powered aircraft or to limit speed
 */
#ifndef THROTTLE_MAX
 # define THROTTLE_MAX                   100
#endif

/**
 * @brief Maximum roll angle limit
 * 
 * @details Maximum bank angle the autopilot will command in stabilized and
 *          autonomous flight modes. Limits how aggressively the aircraft
 *          can turn and provides safety margin from stall.
 *          
 * @note Default: 45 degrees
 * @note Units: degrees
 * @note Can be overridden by ROLL_LIMIT_DEG parameter
 * @note Typical range: 30-60 degrees depending on aircraft capability
 * 
 * @warning Excessive roll limits can lead to high-G turns and stall
 */
#ifndef ROLL_LIMIT_DEG
 # define ROLL_LIMIT_DEG                         45
#endif

/**
 * @brief Maximum pitch up angle limit
 * 
 * @details Maximum nose-up pitch angle the autopilot will command. Prevents
 *          excessive climb angles that could lead to stall.
 *          
 * @note Default: 20 degrees
 * @note Units: degrees
 * @note Can be overridden by PTCH_LIM_MAX_DEG parameter
 * @note Positive values indicate nose up
 * 
 * @warning Setting too high increases stall risk during climbs
 */
#ifndef PITCH_MAX
 # define PITCH_MAX                              20
#endif

/**
 * @brief Maximum pitch down angle limit
 * 
 * @details Maximum nose-down pitch angle the autopilot will command. Prevents
 *          excessive descent angles and airspeed buildup.
 *          
 * @note Default: -25 degrees (25 degrees nose down)
 * @note Units: degrees (negative values indicate nose down)
 * @note Can be overridden by PTCH_LIM_MIN_DEG parameter
 * 
 * @warning Setting too negative can cause excessive airspeed and structural loads
 */
#ifndef PITCH_MIN
 # define PITCH_MIN                              -25
#endif

/**
 * @brief Rudder to aileron mixing ratio
 * 
 * @details Amount of rudder deflection to mix with aileron input for coordinated
 *          turns. Helps counteract adverse yaw during rolling maneuvers.
 *          
 * @note Default: 0.5 (50% rudder mixing)
 * @note Valid range: 0.0 (no mixing) to 1.0 (100% mixing)
 * @note Can be overridden by RUD_MIX parameter
 * @note Optimal value depends on aircraft characteristics (dihedral, fin area)
 */
#ifndef RUDDER_MIX
 # define RUDDER_MIX           0.5f
#endif

/** @} */ // end of flight_nav_config group


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// DEBUGGING
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

/**
 * @defgroup debug_config Debugging and Logging Configuration
 * @brief Configuration constants for logging and debugging features
 * @{
 */

/**
 * @brief Default logging bitmask
 * 
 * @details Bitmask specifying which data types to log to dataflash/SD card by default.
 *          Each bit enables logging of a specific message type (ATT, GPS, MODE, etc.).
 *          
 * @note Default: 0xffff (all basic log types enabled, bits 0-15)
 * @note Can be overridden by LOG_BITMASK parameter at runtime
 * @note Setting to 0xffff provides comprehensive logging for post-flight analysis
 * 
 * @see AP_Logger.h for bit definitions and message types
 */
#define DEFAULT_LOG_BITMASK   0xffff

/** @} */ // end of debug_config group

/**
 * @defgroup navigation_defaults Navigation Default Values
 * @brief Default navigation parameters for waypoints, loitering, and altitude hold
 * @{
 */

/**
 * @brief Default waypoint radius
 * 
 * @details Distance from waypoint at which the waypoint is considered reached
 *          and the aircraft proceeds to the next waypoint in the mission.
 *          
 * @note Default: 90 meters
 * @note Units: meters
 * @note Can be overridden by WP_RADIUS parameter
 * @note Larger values allow smoother turns but less precise navigation
 * @note Smaller values require tighter turns and more precise control
 */
#ifndef WP_RADIUS_DEFAULT
 # define WP_RADIUS_DEFAULT              90
#endif

/**
 * @brief Default loiter circle radius
 * 
 * @details Radius of the circular loiter pattern in LOITER mode and when
 *          loitering at waypoints. Determines the size of the holding pattern.
 *          
 * @note Default: 60 meters
 * @note Units: meters
 * @note Can be overridden by WP_LOITER_RAD parameter
 * @note Positive values = clockwise circles, negative = counter-clockwise
 * @note Minimum recommended: 1.5x turn radius of aircraft
 */
#ifndef LOITER_RADIUS_DEFAULT
 # define LOITER_RADIUS_DEFAULT 60
#endif

/**
 * @brief Default altitude for ALT_HOLD mode
 * 
 * @details Target altitude above home when ALT_HOLD mode is first engaged
 *          without a previous altitude target.
 *          
 * @note Default: 100 meters above home
 * @note Units: meters above home
 * @note Can be overridden by ALT_HOLD_RTL parameter
 * @note Should be set high enough to clear obstacles in flying area
 */
#ifndef ALT_HOLD_HOME
 # define ALT_HOLD_HOME 100
#endif

/** @} */ // end of navigation_defaults group

/**
 * @defgroup developer_items Developer and Advanced Configuration
 * @brief Advanced configuration options for developers and special features
 * @{
 */

/**
 * @brief Airspeed for control surface scaling
 * 
 * @details Reference airspeed used for scaling control surface deflections.
 *          At this airspeed, full control deflection produces the configured
 *          response. At higher speeds, deflections are reduced to maintain
 *          consistent control authority.
 *          
 * @note Default: 15.0 m/s
 * @note Units: meters per second
 * @note Can be overridden by SCALING_SPEED parameter
 * @note Typically set to cruise or maneuvering speed
 */
#ifndef SCALING_SPEED
 # define SCALING_SPEED          15.0
#endif

/**
 * @brief GPIO pin to activate when geofence is triggered
 * 
 * @details Digital output pin that is set HIGH when a geofence breach occurs.
 *          Can be used to trigger external systems (e.g., parachute, LED indicator).
 *          
 * @note Default: -1 (disabled, no pin activation)
 * @note Set to valid GPIO number to enable fence trigger output
 * @note Can be overridden by FENCE_PIN parameter
 */
#ifndef FENCE_TRIGGERED_PIN
 # define FENCE_TRIGGERED_PIN -1
#endif

/**
 * @brief Enable slew rate limiting for offboard guided commands
 * 
 * @details When enabled, applies slew rate limiting to position/velocity targets
 *          received from offboard guidance systems (e.g., companion computers).
 *          Prevents dangerous sudden changes in commanded trajectory.
 *          
 * @note Default: 1 (enabled)
 * @note Set to 0 to disable slew limiting (not recommended)
 * @note Improves safety when using companion computer guidance
 */
#ifndef AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
 #define AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED 1
#endif

/** @} */ // end of developer_items group

/**
 * @defgroup failsafe_config Failsafe Configuration
 * @brief Failsafe thresholds and safety parameters
 * @{
 */

/**
 * @brief EKF failsafe variance threshold
 * 
 * @details EKF (Extended Kalman Filter) innovation variance threshold above which
 *          the EKF failsafe is triggered. When compass or velocity variance exceeds
 *          this value, it indicates poor state estimation quality and triggers
 *          failsafe action.
 *          
 * @note Default: 0.8
 * @note Units: dimensionless variance ratio
 * @note Can be overridden by FS_EKF_THRESH parameter
 * @note Lower values = more sensitive failsafe, higher = more tolerance
 * 
 * @warning Setting too low causes false failsafe triggers in normal flight
 * @warning Setting too high may delay failsafe in actual navigation failures
 */
#ifndef FS_EKF_THRESHOLD_DEFAULT
 # define FS_EKF_THRESHOLD_DEFAULT      0.8f    // EKF failsafe's default compass and velocity variance threshold above which the EKF failsafe will be triggered
#endif

/**
 * @brief Landing throttle control activation threshold
 * 
 * @details Altitude as a fraction of landing flare height below which throttle
 *          control transitions to landing-specific logic. Below this threshold,
 *          throttle is gradually reduced for touchdown.
 *          
 * @note Default: 0.7 (activate at 70% of flare altitude)
 * @note Units: fraction (0.0-1.0)
 * @note Can be overridden at runtime
 * @note Lower values = later throttle reduction, higher = earlier reduction
 * 
 * @warning Incorrect value can cause hard landings or bouncing
 */
#ifndef THR_CTRL_LAND_THRESH
 #define THR_CTRL_LAND_THRESH 0.7
#endif

/** @} */ // end of failsafe_config group
