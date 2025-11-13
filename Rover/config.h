/**
 * @file config.h
 * @brief Default configuration values and feature flags for ArduRover
 * 
 * @details This file provides compile-time configuration options that can be
 *          overridden by defining values before this file is included. It sets
 *          default values for MAVLink system ID, mode channel, cruise speed,
 *          and conditional feature compilation flags.
 *          
 *          Configuration values in this file establish baseline behavior for
 *          the ArduRover vehicle type. Hardware-specific or user-specific
 *          configurations can override these defaults by defining the
 *          corresponding symbols before including this file.
 * 
 * @note All configuration values can be overridden by defining them before
 *       this file is included, allowing for board-specific or user-specific
 *       customization without modifying this file.
 * 
 * @note Feature flags (MODE_DOCK_ENABLED, MODE_FOLLOW_ENABLED) depend on
 *       build-time library availability and are automatically configured
 *       based on enabled features.
 */

#pragma once

#include "defines.h"

/**
 * @brief MAVLink system identifier for this vehicle
 * 
 * @details Defines the unique MAVLink system ID used to identify this vehicle
 *          in a multi-vehicle network. Each vehicle in a network should have
 *          a unique system ID to enable proper message routing and command
 *          targeting by ground control stations.
 * 
 * @note Default: 1 (single vehicle configuration)
 * @note Valid range: 1-255
 * @note Must be unique across all vehicles in the same MAVLink network
 */
#ifndef MAV_SYSTEM_ID
  #define MAV_SYSTEM_ID    1
#endif

//////////////////////////////////////////////////////////////////////////////
// Auxiliary Channel Configuration
//

/**
 * @brief Default function for auxiliary channel 7
 * 
 * @details Configures the default behavior for auxiliary RC channel 7.
 *          CH7_SAVE_WP enables waypoint saving functionality, allowing
 *          the operator to record the current vehicle position as a
 *          waypoint during operation.
 * 
 * @note Default: CH7_SAVE_WP (save waypoint function)
 * @note Can be overridden to assign different auxiliary functions to channel 7
 * @note The actual channel function can also be configured via parameters
 *       at runtime without recompiling
 */
#ifndef CH7_OPTION
  #define CH7_OPTION CH7_SAVE_WP
#endif

//////////////////////////////////////////////////////////////////////////////
// Flight Mode Selection
//

/**
 * @brief RC channel number used for flight mode switching
 * 
 * @details Defines which RC input channel is used to select the vehicle's
 *          flight mode. The mode channel allows the operator to switch
 *          between different autonomous and manual control modes using
 *          a transmitter switch or knob.
 *          
 *          The mode channel reading is mapped to different flight modes
 *          based on PWM ranges configured in the mode parameters.
 * 
 * @note Default: 8 (typically a 3-position switch on channel 8)
 * @note Valid values: 5, 6, 7, or 8 only
 * @note Hardware constraint: Only channels 5-8 are supported for mode selection
 * 
 * @warning Changing this value requires recompilation. The compile-time
 *          validation below ensures only supported channels are used.
 */
#ifndef MODE_CHANNEL
  #define MODE_CHANNEL    8
#endif

/**
 * @brief Compile-time validation for MODE_CHANNEL
 * 
 * @details Ensures that MODE_CHANNEL is set to one of the supported values
 *          (5, 6, 7, or 8). This validation prevents build configurations
 *          that would result in runtime errors or undefined behavior.
 *          
 *          The restriction to channels 5-8 is due to implementation
 *          constraints in the mode switching logic.
 */
#if (MODE_CHANNEL != 5) && (MODE_CHANNEL != 6) && (MODE_CHANNEL != 7) && (MODE_CHANNEL != 8)
  #error XXX
  #error XXX You must set MODE_CHANNEL to 5, 6, 7 or 8
  #error XXX
#endif

//////////////////////////////////////////////////////////////////////////////
// Navigation Controller Configuration
//

/**
 * @brief L1 navigation controller period
 * 
 * @details Defines the period (in seconds) for the L1 navigation controller,
 *          which is used for waypoint navigation and path following in
 *          autonomous modes. The L1 controller is a guidance law that
 *          computes lateral acceleration commands to follow a desired path.
 *          
 *          A larger period results in wider, gentler turns, while a smaller
 *          period produces tighter turns. The optimal value depends on
 *          vehicle dynamics, speed, and desired path-following precision.
 * 
 * @note Default: 8 seconds
 * @note Units: seconds
 * @note Typical range: 5-20 seconds depending on vehicle characteristics
 * @note Can be tuned via NAVL1_PERIOD parameter without recompilation
 */
#ifndef NAVL1
  #define NAVL1_PERIOD    8
#endif

//////////////////////////////////////////////////////////////////////////////
// Speed Configuration
//

/**
 * @brief Default cruise speed for autonomous navigation
 * 
 * @details Sets the default target speed for autonomous modes such as Auto
 *          and Guided. The vehicle will attempt to maintain this speed
 *          during waypoint navigation and path following, subject to
 *          throttle limits and terrain constraints.
 *          
 *          This value provides a reasonable default for initial testing
 *          and can be adjusted via parameters for specific vehicle and
 *          mission requirements.
 * 
 * @note Default: 2 m/s (approximately 7.2 km/h or 4.5 mph)
 * @note Units: meters per second (m/s)
 * @note Can be configured via CRUISE_SPEED parameter without recompilation
 * @note Actual speed may vary based on terrain, throttle limits, and motor capabilities
 */
#ifndef CRUISE_SPEED
  #define CRUISE_SPEED    2  // in m/s
#endif

/**
 * @brief Default logging bitmask for data recording
 * 
 * @details Configures which data types are logged to onboard storage by default.
 *          Each bit in the bitmask enables logging for a specific category of
 *          data (attitude, GPS, motors, etc.). The value 0xffff enables all
 *          available logging categories, providing comprehensive data for
 *          analysis and debugging.
 *          
 *          Comprehensive logging is useful for development, testing, and
 *          post-flight analysis but consumes more storage space.
 * 
 * @note Default: 0xffff (all logging categories enabled)
 * @note Value is a bitmask where each bit represents a logging category
 * @note Can be configured via LOG_BITMASK parameter to disable specific
 *       categories and reduce storage consumption
 * @note Full logging may fill storage quickly on long missions
 */
#define DEFAULT_LOG_BITMASK    0xffff

//////////////////////////////////////////////////////////////////////////////
// Optional Flight Mode Features
//

/**
 * @brief Enable Dock mode for autonomous docking
 * 
 * @details Enables the Dock flight mode, which allows the vehicle to
 *          autonomously approach and dock with a docking target using
 *          precision landing sensors. Dock mode is only available when
 *          the precision landing library (AC_PrecLand) is compiled in.
 *          
 *          This mode is useful for autonomous charging stations or
 *          precise parking locations where the vehicle must align with
 *          a specific target.
 * 
 * @note Default: Enabled if AC_PRECLAND_ENABLED is true
 * @note Requires: Precision landing library and compatible sensors
 * @note Dependency: AC_PRECLAND_ENABLED build flag
 * @note Hardware: Requires precision landing sensor (IR-LOCK, optical flow, etc.)
 */
#ifndef MODE_DOCK_ENABLED
# define MODE_DOCK_ENABLED AC_PRECLAND_ENABLED
#endif

/**
 * @brief Enable Follow mode for tracking moving targets
 * 
 * @details Enables the Follow flight mode, which allows the vehicle to
 *          autonomously follow a moving target. The target position can
 *          be provided via MAVLink messages from another vehicle or a
 *          ground control station. Follow mode is only available when
 *          the AP_Follow library is compiled in.
 *          
 *          This mode enables cooperative vehicle operations, such as
 *          one rover following another or following a person with a
 *          telemetry device.
 * 
 * @note Default: Enabled if AP_FOLLOW_ENABLED is true
 * @note Requires: AP_Follow library
 * @note Dependency: AP_FOLLOW_ENABLED build flag
 * @note Target position typically provided via MAVLink GLOBAL_POSITION_INT messages
 */
#ifndef MODE_FOLLOW_ENABLED
# define MODE_FOLLOW_ENABLED AP_FOLLOW_ENABLED
#endif


//////////////////////////////////////////////////////////////////////////////
// Advanced Configuration Options
//

/**
 * @brief PWM threshold for mode reset after failsafe or fence breach
 * 
 * @details Defines the PWM value threshold on the reset switch channel
 *          (if configured) that triggers a reset of the control mode to
 *          match the current mode switch position. This allows the operator
 *          to return to normal switched mode control after an automatic
 *          failsafe action or geofence breach has forced the vehicle into
 *          a safety mode.
 *          
 *          When the reset switch channel exceeds this PWM value, the
 *          vehicle will exit failsafe/breach mode and return to the mode
 *          selected by the mode switch, restoring normal operator control.
 * 
 * @note Default: 1750 microseconds
 * @note Units: microseconds (μs) of PWM pulse width
 * @note Typical PWM range: 1000-2000 μs (1750 is approximately high position)
 * @note Only active if RESET_SWITCH_CH parameter is configured to a valid channel
 * @note Allows recovery from failsafe/fence breach without mode switch cycling
 */
#ifndef RESET_SWITCH_CHAN_PWM
  #define RESET_SWITCH_CHAN_PWM    1750
#endif

/**
 * @brief Enable Advanced Failsafe system
 * 
 * @details Enables the Advanced Failsafe (AFS) system, which provides
 *          enhanced safety features including GPS jamming detection,
 *          datalink loss handling, and automatic termination capabilities.
 *          Advanced Failsafe is designed for critical applications where
 *          additional safety measures beyond standard failsafe are required.
 *          
 *          When enabled, AFS monitors multiple system health indicators
 *          and can trigger automatic vehicle termination if critical
 *          failures are detected.
 * 
 * @note Default: 0 (disabled)
 * @note Valid values: 0 (disabled), 1 (enabled)
 * @note Requires: AP_AdvancedFailsafe library
 * 
 * @warning Advanced Failsafe can trigger automatic vehicle termination
 *          (motor shutdown) under certain failure conditions. Only enable
 *          if you understand the termination criteria and have appropriate
 *          hardware termination provisions.
 * 
 * @warning Enabling AFS requires careful configuration and testing to avoid
 *          unintended terminations during normal operations.
 */
#ifndef AP_ROVER_ADVANCED_FAILSAFE_ENABLED
  #define AP_ROVER_ADVANCED_FAILSAFE_ENABLED 0
#endif

/**
 * @brief Enable auto-arm once per boot functionality
 * 
 * @details When enabled, allows the vehicle to automatically arm once per
 *          boot cycle when in Auto mode without requiring explicit arming
 *          by the operator. After the first auto-arm, subsequent arming
 *          requires manual action. This feature facilitates autonomous
 *          mission start while maintaining safety by preventing repeated
 *          automatic arming.
 *          
 *          This is useful for autonomous operations where the vehicle
 *          should start a mission automatically on power-up, but requires
 *          operator intervention if disarmed during the mission.
 * 
 * @note Default: 1 (enabled)
 * @note Valid values: 0 (disabled), 1 (enabled)
 * @note Auto-arm only occurs once per boot, even if the vehicle is later disarmed
 * @note Standard pre-arm safety checks still apply before auto-arming
 * @note Manual arming via RC transmitter or GCS is always possible regardless of this setting
 */
#ifndef AP_ROVER_AUTO_ARM_ONCE_ENABLED
#define AP_ROVER_AUTO_ARM_ONCE_ENABLED 1
#endif
