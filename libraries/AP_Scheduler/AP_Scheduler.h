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
 * @file AP_Scheduler.h
 * @brief Main loop scheduler for real-time task scheduling in ArduPilot
 * 
 * @details The AP_Scheduler implements a priority-based cooperative multitasking scheduler
 *          for ArduPilot vehicle control. It manages the execution of periodic tasks at
 *          specified rates while maintaining real-time performance requirements. The scheduler
 *          synchronizes to IMU samples and provides timing budget management, performance
 *          monitoring, and adaptive loop rate adjustment for CPU overload handling.
 * 
 * @author Andrew Tridgell, January 2013
 * @copyright Copyright (c) 2013-2025 ArduPilot.org
 */
#pragma once

#include "AP_Scheduler_config.h"

#if AP_SCHEDULER_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Util.h>
#include <AP_Math/AP_Math.h>
#include "PerfInfo.h"       // loop perf monitoring

/**
 * @brief Task name initializer macro with extended class information
 * @details Creates task name string including class name and method name when
 *          AP_SCHEDULER_EXTENDED_TASKINFO_ENABLED is defined, otherwise just method name
 */
#if AP_SCHEDULER_EXTENDED_TASKINFO_ENABLED
#define AP_SCHEDULER_NAME_INITIALIZER(_clazz,_name) .name = #_clazz "::" #_name,
#define AP_FAST_NAME_INITIALIZER(_clazz,_name) .name = #_clazz "::" #_name "*",
#else
#define AP_SCHEDULER_NAME_INITIALIZER(_clazz,_name) .name = #_name,
#define AP_FAST_NAME_INITIALIZER(_clazz,_name) .name = #_name "*",
#endif

/**
 * @brief Constant indicating task should run at main loop rate
 */
#define LOOP_RATE 0

/**
 * @brief Macro to create scheduler task table entries
 * 
 * @details This macro simplifies the creation of Task structures for the scheduler task table.
 *          It binds a class method to a functor and initializes all task parameters.
 * 
 * @param classname The class type containing the task method
 * @param classptr Pointer to an instance of the class
 * @param func The method name to call (without parentheses)
 * @param _rate_hz Execution rate in Hz (0 = main loop rate, defined by LOOP_RATE)
 * @param _max_time_micros Maximum allowed execution time in microseconds (time budget)
 * @param _priority Task priority (lower number = higher priority, must be monotonically increasing in task table)
 * 
 * @note Task tables must be sorted by priority in ascending order
 * @warning max_time_micros affects real-time performance if task execution exceeds this limit
 */
#define SCHED_TASK_CLASS(classname, classptr, func, _rate_hz, _max_time_micros, _priority) { \
    .function = FUNCTOR_BIND(classptr, &classname::func, void),\
    AP_SCHEDULER_NAME_INITIALIZER(classname, func)\
    .rate_hz = _rate_hz,\
    .max_time_micros = _max_time_micros,        \
    .priority = _priority \
}

/**
 * @brief Macro for fast-loop tasks running at main loop rate
 * 
 * @details Creates a Task entry for high-priority tasks that run every loop iteration.
 *          These tasks are executed with FAST_TASK_PRI0 priority and no explicit time limit.
 * 
 * @param classname The class type containing the task method
 * @param classptr Pointer to an instance of the class
 * @param func The method name to call (without parentheses)
 * 
 * @note Fast tasks run at loop rate (0 Hz rate means every loop)
 */
#define FAST_TASK_CLASS(classname, classptr, func) { \
    .function = FUNCTOR_BIND(classptr, &classname::func, void),\
    AP_FAST_NAME_INITIALIZER(classname, func)\
    .rate_hz = 0,\
    .max_time_micros = 0,\
    .priority = AP_Scheduler::FAST_TASK_PRI0 \
}

/**
 * @class AP_Scheduler
 * @brief Priority-based cooperative multitasking scheduler for real-time vehicle control
 * 
 * @details The AP_Scheduler manages execution of periodic tasks in ArduPilot's main loop,
 *          providing deterministic real-time scheduling for flight-critical operations.
 *          
 *          Key Features:
 *          - Priority-based task execution with rate limiting
 *          - Timing budget management per task
 *          - Performance monitoring and CPU load tracking
 *          - Fast vs slow task separation for time-critical operations
 *          - Adaptive extra_loop_us for CPU overload handling
 *          
 *          Typical Loop Rates:
 *          - ArduCopter: 400Hz main loop (2500 microseconds per loop)
 *          - ArduPlane: 50Hz main loop (20000 microseconds per loop)
 *          - Other vehicles: Configurable via _loop_rate_hz parameter
 *          
 *          Scheduling Algorithm:
 *          1. Synchronize to IMU sample arrival
 *          2. Increment tick counter
 *          3. Calculate available time (loop period + extra_loop_us)
 *          4. Execute tasks in priority order if rate interval elapsed
 *          5. Monitor execution time against max_time_micros budget
 *          6. Detect slips (late execution) and overruns (budget exceeded)
 *          7. Adaptively adjust extra_loop_us if tasks consistently miss budgets
 *          
 *          Task Table Requirements:
 *          - Tasks must be sorted by priority (ascending, lowest number = highest priority)
 *          - Priority values must be monotonically increasing
 *          - Use SCHED_TASK_CLASS macro for standard tasks
 *          - Use FAST_TASK_CLASS macro for main-loop-rate tasks
 *          
 * @note Units: Hz for rates, microseconds for timing budgets, seconds for loop periods
 * @warning The loop() method should be the ONLY function called from vehicle's main loop
 */
class AP_Scheduler
{
public:
    AP_Scheduler();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Scheduler);

    /**
     * @brief Retrieve scheduler singleton instance
     * @return Pointer to the global AP_Scheduler instance
     */
    static AP_Scheduler *get_singleton();
    static AP_Scheduler *_singleton;

    FUNCTOR_TYPEDEF(task_fn_t, void);

    /**
     * @struct Task
     * @brief Task definition structure for scheduler task table
     * 
     * @details Defines a single schedulable task including its execution function,
     *          timing parameters, and priority. Tasks are organized in tables and
     *          executed by the scheduler based on their rate and priority.
     * 
     * @note Task tables must be sorted by priority in ascending order (lower number = higher priority)
     * @warning max_time_micros affects real-time performance if exceeded - scheduler logs overruns
     */
    struct Task {
        task_fn_t function;          ///< @brief Bound functor to execute (method bound to object instance)
        const char *name;            ///< @brief Task identifier for debugging and logging
        float rate_hz;               ///< @brief Execution rate in Hz (0 = main loop rate)
        uint16_t max_time_micros;    ///< @brief Time budget in microseconds (scheduler monitors overruns)
        uint8_t priority;            ///< @brief Execution priority (lower number = higher priority, must be monotonically increasing in task table)
    };

    /**
     * @enum Options
     * @brief Bitmask options for scheduler behavior configuration
     */
    enum class Options : uint8_t {
        RECORD_TASK_INFO = 1 << 0    ///< @brief Enable per-task performance tracking and recording
    };

    /**
     * @enum FastTaskPriorities
     * @brief Priority levels for fast-loop tasks running at main loop rate
     * 
     * @details Fast tasks are high-priority operations that execute every loop iteration.
     *          These priority levels separate time-critical tasks from lower priority operations.
     */
    enum FastTaskPriorities {
        FAST_TASK_PRI0 = 0,          ///< @brief Highest priority fast task (executed first)
        FAST_TASK_PRI1 = 1,          ///< @brief Medium priority fast task
        FAST_TASK_PRI2 = 2,          ///< @brief Lower priority fast task
        MAX_FAST_TASK_PRIORITIES = 3 ///< @brief Total number of fast task priority levels
    };

    /**
     * @brief Initialize scheduler with task table
     * 
     * @details Merges vehicle-specific task table with common tasks from AP_Vehicle,
     *          validates priority ordering, allocates _last_run array for tracking
     *          task execution history, and configures performance monitoring.
     * 
     * @param[in] tasks Array of Task structures defining vehicle-specific tasks
     * @param[in] num_tasks Number of tasks in the array
     * @param[in] log_performance_bit Bitmask for performance logging (enables PERF message output)
     * 
     * @warning Task table must have monotonically increasing priority values
     * @note Call once during vehicle initialization before entering main loop
     */
    void init(const Task *tasks, uint8_t num_tasks, uint32_t log_performance_bit);

    /**
     * @brief Main scheduler loop - synchronizes to IMU samples and dispatches tasks
     * 
     * @details This is the heart of ArduPilot's real-time scheduling. The loop:
     *          1. Waits for IMU sample arrival
     *          2. Calls tick() to increment scheduling cycle counter
     *          3. Calculates time_available = loop period + extra_loop_us
     *          4. Calls run() to execute scheduled tasks
     *          
     *          The loop synchronizes all vehicle operations to the IMU sample rate,
     *          ensuring consistent timing for state estimation and control.
     * 
     * @warning This should be the ONLY function called from vehicle's main loop
     * @note Holds scheduler semaphore except while waiting for IMU samples
     */
    void loop();

    /**
     * @brief Update scheduler logging output
     * @note Call at 1Hz to update logging statistics
     */
    void update_logging();

    /**
     * @brief Write PERF message to dataflash logger
     * @details Logs scheduler performance metrics including loop time, CPU load, and timing statistics
     */
    void Log_Write_Performance();

    /**
     * @brief Increment tick counter
     * @details Called once per loop iteration to track scheduling cycles for rate limiting
     */
    void tick(void);

    /**
     * @brief Return 16-bit tick counter
     * @return Tick count (wraps at 65535)
     * @note Used for rate limiting calculations in run()
     */
    uint16_t ticks() const { return _tick_counter; }
    
    /**
     * @brief Return 32-bit tick counter
     * @return Tick count (extended range, wraps at 4294967295)
     */
    uint32_t ticks32() const { return _tick_counter32; }

    /**
     * @brief Execute scheduled tasks within time budget
     * 
     * @details Iterates through the task table in priority order, executing tasks
     *          when their rate interval has elapsed. For each task:
     *          1. Checks if rate_hz interval has passed since last execution
     *          2. Measures execution time against max_time_micros budget
     *          3. Detects slips (late execution) and overruns (exceeded budget)
     *          4. Updates perf_info for performance monitoring
     *          5. Tracks spare time for CPU load calculation
     *          
     *          If tasks consistently miss budgets (task_not_achieved > max_task_slowdown),
     *          extra_loop_us is increased to provide more time per loop.
     * 
     * @param[in] time_available Microseconds available for task execution this loop
     * 
     * @warning Task execution times directly affect real-time performance and vehicle stability
     * @note Call once per tick with calculated time budget
     */
    void run(uint32_t time_available);

    /**
     * @brief Return microseconds remaining for current task
     * @return Microseconds available for the currently executing task
     * @note Useful for tasks to self-limit execution time
     */
    uint16_t time_available_usec(void) const;

    /**
     * @brief Return debug parameter value
     * @return Debug flags (0=disabled, 2=show slips, 3=show overruns)
     * @note Used for troubleshooting scheduler timing issues
     */
    uint8_t debug_flags(void) { return _debug; }

    /**
     * @brief Calculate CPU load
     * 
     * @details Computes load average as ratio of used time to available time.
     *          Calculated from spare time accumulated at end of run() calls.
     *          
     * @return Load as 0.0 to 1.0 (1.0 = 100% CPU load, 0.0 = idle)
     * 
     * @note Load > 0.9 indicates CPU is near saturation
     * @note Load consistently at 1.0 indicates scheduler cannot keep up with task load
     */
    float load_average();

    /**
     * @brief Return active main loop rate
     * 
     * @details Returns the configured loop rate set at startup. Lazy-initializes
     *          _active_loop_rate_hz from parameter _loop_rate_hz on first call.
     *          
     * @return Rate in Hz (typically 400 for copters, 50 for planes)
     * @note This is the target rate; actual rate may vary under CPU load
     */
    uint16_t get_loop_rate_hz(void) {
        if (_active_loop_rate_hz == 0) {
            _active_loop_rate_hz = _loop_rate_hz;
        }
        return _active_loop_rate_hz;
    }
    
    /**
     * @brief Return loop period in microseconds
     * 
     * @details Calculates period from loop rate: period = 1,000,000 / rate_hz
     *          Lazy-initializes on first call.
     *          
     * @return Period in microseconds (e.g., 2500µs for 400Hz, 20000µs for 50Hz)
     * @note This is the nominal time budget per loop
     */
    uint32_t get_loop_period_us() {
        if (_loop_period_us == 0) {
            _loop_period_us = 1000000UL / _loop_rate_hz;
        }
        return _loop_period_us;
    }
    
    /**
     * @brief Return loop period in seconds
     * 
     * @details Calculates period from loop rate: period = 1.0 / rate_hz
     *          Lazy-initializes on first call.
     *          
     * @return Period in seconds (e.g., 0.0025s for 400Hz, 0.02s for 50Hz)
     */
    float get_loop_period_s() {
        if (is_zero(_loop_period_s)) {
            _loop_period_s = 1.0f / _loop_rate_hz;
        }
        return _loop_period_s;
    }

    /**
     * @brief Return filtered loop execution time
     * 
     * @details Returns low-pass filtered measurement of actual loop duration.
     *          Useful for monitoring scheduler performance over time.
     *          
     * @return Time in seconds
     * @note Filtering reduces noise from individual loop variations
     */
    float get_filtered_loop_time(void) const {
        return perf_info.get_filtered_time();
    }

    /**
     * @brief Return filtered actual loop rate
     * 
     * @details Returns low-pass filtered measurement of achieved loop rate.
     *          May differ from nominal rate under CPU load or timing variations.
     *          
     * @return Rate in Hz
     */
    float get_filtered_loop_rate_hz() {
        return perf_info.get_filtered_loop_rate_hz();
    }

    /**
     * @brief Return last loop duration
     * 
     * @details Provides instantaneous measurement of previous loop execution time.
     *          
     * @return Time in seconds
     * @note Instantaneous value may be noisy; use get_filtered_loop_time() for trends
     */
    float get_last_loop_time_s(void) const {
        return _last_loop_time_s;
    }

    /**
     * @brief Return timestamp of current loop start
     * 
     * @details Provides microsecond timestamp when current scheduling loop began.
     *          Synchronized to IMU sample arrival time.
     *          
     * @return Time in microseconds since boot
     * @note Useful for timestamping events within the current loop
     */
    uint64_t get_loop_start_time_us(void) const {
        return _loop_sample_time_us;
    }

    /**
     * @brief Return adaptive extra time per loop
     * 
     * @details Returns additional microseconds added to each loop's time budget.
     *          Dynamically adjusted when tasks consistently miss their time budgets.
     *          Increases when task_not_achieved exceeds threshold, decreases when
     *          all tasks consistently meet budgets.
     *          
     * @return Microseconds of extra time per loop
     * @note Non-zero value indicates scheduler adapting to CPU load
     */
    uint32_t get_extra_loop_us(void) const {
        return extra_loop_us;
    }

    /**
     * @brief Return scheduler semaphore for synchronization
     * 
     * @details Provides access to semaphore held during task execution.
     *          Released only while waiting for IMU samples.
     *          
     * @return Reference to HAL_Semaphore
     * @note Other subsystems can use this for synchronization with scheduler
     */
    HAL_Semaphore &get_semaphore(void) { return _rsem; }

    /**
     * @brief Format task performance info to string
     * 
     * @details Generates human-readable summary of per-task performance metrics
     *          including execution times, overruns, and slips.
     *          
     * @param[out] str ExpandingString for formatted output
     * @note Requires Options::RECORD_TASK_INFO to be enabled
     */
    void task_info(ExpandingString &str);

    static const struct AP_Param::GroupInfo var_info[];

    // loop performance monitoring:
    AP::PerfInfo perf_info;

private:
    AP_Int8 _debug;                      ///< Debug level parameter (0=disabled, 2=show slips, 3=show overruns)

    AP_Int16 _loop_rate_hz;              ///< Main loop rate parameter in Hz (requires reboot to change)

    AP_Int16 _active_loop_rate_hz;       ///< Loop rate set at startup (cached from _loop_rate_hz)

    AP_Int8 _options;                    ///< Scheduler options bitmask (see Options enum)
    
    uint16_t _loop_period_us;            ///< Calculated loop period in microseconds (1000000 / _loop_rate_hz)

    float _loop_period_s;                ///< Calculated loop period in seconds (1.0 / _loop_rate_hz)
    
    const struct Task *_vehicle_tasks;   ///< Pointer to vehicle-specific task table
    uint8_t _num_vehicle_tasks;          ///< Number of vehicle-specific tasks

    const struct Task *_common_tasks;    ///< Pointer to common task table from AP_Vehicle
    uint8_t _num_common_tasks;           ///< Number of common tasks

    uint8_t _num_tasks;                  ///< Total task count (_num_vehicle_tasks + _num_common_tasks)

    uint16_t _tick_counter;              ///< 16-bit scheduling cycle counter (wraps at 65535)
    uint32_t _tick_counter32;            ///< 32-bit scheduling cycle counter (extended range)

    uint16_t *_last_run;                 ///< Array tracking last execution tick for each task (for rate limiting)

    uint32_t _task_time_allowed;         ///< Microseconds allowed for currently executing task (from max_time_micros)

    uint32_t _task_time_started;         ///< Microsecond timestamp when current task started execution

    uint32_t _spare_micros;              ///< Accumulated spare microseconds for load calculation

    uint8_t _spare_ticks;                ///< Number of ticks over which _spare_micros is accumulated

    uint32_t _loop_timer_start_us;       ///< Start timestamp of previous loop in microseconds

    float _last_loop_time_s;             ///< Duration of last loop in seconds

    uint64_t _loop_sample_time_us;       ///< Start timestamp of current loop in microseconds (64-bit for long uptimes)

    uint32_t _log_performance_bit;       ///< Logger bitmask for enabling PERF message output

    const uint8_t max_task_slowdown = 4; ///< Threshold: tasks running this many times slower than desired rate triggers extra_loop_us increase

    uint32_t task_not_achieved;          ///< Counter: number of times tasks missed time budgets (triggers extra_loop_us increase)
    uint32_t task_all_achieved;          ///< Counter: number of times all tasks met budgets (triggers extra_loop_us decrease)
    
    uint32_t extra_loop_us;              ///< Adaptive extra microseconds per loop (increased when tasks consistently overrun, decreased when stable)

    HAL_Semaphore _rsem;                 ///< Semaphore held during task execution, released while waiting for IMU samples
};

namespace AP {
    AP_Scheduler &scheduler();
};

#endif  // AP_SCHEDULER_ENABLED
