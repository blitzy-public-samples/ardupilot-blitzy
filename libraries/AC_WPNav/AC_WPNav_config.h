/**
 * @file AC_WPNav_config.h
 * @brief Configuration header for AC_WPNav waypoint navigation library
 * 
 * @details This configuration file controls compile-time feature selection for the
 *          AC_WPNav waypoint navigation library, specifically enabling or disabling
 *          object avoidance (OA) integration.
 * 
 *          The primary purpose is to define AC_WPNAV_OA_ENABLED, which determines
 *          whether the AC_WPNav_OA (Object Avoidance) class is compiled into the
 *          firmware. This allows for conditional compilation to reduce binary size
 *          on platforms where object avoidance functionality is not needed.
 * 
 *          This file depends on AC_Avoidance_config.h which provides the underlying
 *          AP_OAPATHPLANNER_ENABLED definition that controls the object avoidance
 *          path planner subsystem availability.
 */

#pragma once

#include <AC_Avoidance/AC_Avoidance_config.h>

/**
 * @def AC_WPNAV_OA_ENABLED
 * @brief Enable/disable object avoidance integration in AC_WPNav waypoint navigation
 * 
 * @details This macro controls whether the AC_WPNav_OA (Object Avoidance) class is
 *          compiled and available for use in waypoint navigation. When enabled, the
 *          waypoint navigation system can integrate with the object avoidance path
 *          planner to dynamically adjust paths around detected obstacles.
 * 
 *          By default, this is set to AP_OAPATHPLANNER_ENABLED, which is defined in
 *          AC_Avoidance/AC_Avoidance_config.h. This creates a dependency chain:
 *          - AP_OAPATHPLANNER_ENABLED controls the core object avoidance path planner
 *          - AC_WPNAV_OA_ENABLED controls whether AC_WPNav uses that path planner
 * 
 *          If the object avoidance path planner is disabled (AP_OAPATHPLANNER_ENABLED = 0),
 *          then AC_WPNav_OA will also be disabled by default, preventing compilation of
 *          unnecessary code.
 * 
 *          This can be overridden by defining AC_WPNAV_OA_ENABLED before including this
 *          header, allowing platform-specific or build-specific control.
 * 
 * @note When disabled, the AC_WPNav_OA class is not compiled, reducing binary size.
 *       This is useful for memory-constrained platforms or applications that don't
 *       require object avoidance during waypoint navigation.
 * 
 * @note Disabling object avoidance means the vehicle will follow waypoint paths
 *       directly without dynamic obstacle avoidance, relying instead on pre-planned
 *       safe routes or other collision avoidance mechanisms.
 * 
 * @see AP_OAPATHPLANNER_ENABLED in AC_Avoidance/AC_Avoidance_config.h
 * @see AC_WPNav_OA class for object avoidance waypoint navigation implementation
 */
#ifndef AC_WPNAV_OA_ENABLED
#define AC_WPNAV_OA_ENABLED AP_OAPATHPLANNER_ENABLED
#endif
