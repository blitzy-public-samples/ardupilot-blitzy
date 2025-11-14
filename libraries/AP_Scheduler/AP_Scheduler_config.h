/**
 * @file AP_Scheduler_config.h
 * @brief Configuration header for AP_Scheduler task scheduling subsystem
 * 
 * @details This file defines compile-time configuration options for the ArduPilot
 *          task scheduler. The scheduler manages periodic execution of vehicle tasks,
 *          including sensor updates, control loops, and telemetry processing.
 *          
 *          Configuration flags in this file can be overridden at build time using
 *          -D compiler flags or in board-specific hwdef files to customize scheduler
 *          behavior for different platforms and use cases.
 * 
 * @note All configuration flags can be overridden at build time using compiler flags
 *       (e.g., -DAP_SCHEDULER_ENABLED=0) or in board hwdef files
 * 
 * @see AP_Scheduler.h for the main scheduler interface
 * @see libraries/AP_Scheduler/README.md for scheduler architecture documentation
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

/**
 * @brief Enable/disable compilation of the AP_Scheduler subsystem
 * 
 * @details This flag controls whether the task scheduler is compiled into the firmware.
 *          The AP_Scheduler provides cooperative multitasking for ArduPilot, managing
 *          the periodic execution of vehicle tasks at different frequencies (e.g., fast
 *          loops for attitude control, slower loops for telemetry).
 *          
 *          When disabled (0), the scheduler code is excluded from compilation, reducing
 *          flash memory usage. This is typically only disabled on extremely memory-
 *          constrained platforms or for specialized builds.
 *          
 *          Default: 1 (enabled)
 * 
 * @note Disabling the scheduler will prevent normal vehicle operation unless an
 *       alternative task management approach is implemented
 */
#ifndef AP_SCHEDULER_ENABLED
#define AP_SCHEDULER_ENABLED 1
#endif

/**
 * @brief Enable extended task information with class name prefixes
 * 
 * @details This flag controls whether task names include class prefixes for improved
 *          debugging and performance profiling. When enabled, scheduler task names are
 *          prefixed with their class name (e.g., "Copter::update_batt_compass" instead
 *          of just "update_batt_compass").
 *          
 *          Extended task information is useful for:
 *          - Performance profiling and timing analysis
 *          - Debugging scheduler task execution
 *          - Identifying which vehicle/library a task belongs to
 *          - Analysis of CPU time usage per component
 *          
 *          Disabling this flag reduces RAM usage slightly by storing shorter task name
 *          strings, at the cost of less detailed task identification in debug output.
 *          
 *          Default: 1 (enabled)
 * 
 * @note Task names are used in scheduler performance logging and debugging output
 */
#ifndef AP_SCHEDULER_EXTENDED_TASKINFO_ENABLED
#define AP_SCHEDULER_EXTENDED_TASKINFO_ENABLED 1
#endif
