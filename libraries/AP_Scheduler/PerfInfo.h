/**
 * @file PerfInfo.h
 * @brief Loop performance monitoring and task profiling for the ArduPilot scheduler
 * 
 * This file provides the PerfInfo class which tracks main loop timing statistics
 * and per-task execution metrics for the AP_Scheduler system. It enables real-time
 * performance analysis and identification of long-running tasks or loops.
 */

#pragma once

#include "AP_Scheduler_config.h"

#if AP_SCHEDULER_ENABLED

#include <stdint.h>
#include <AP_Common/ExpandingString.h>

namespace AP {

/**
 * @class PerfInfo
 * @brief Provides loop performance monitoring and per-task profiling for the scheduler
 * 
 * @details The PerfInfo class tracks comprehensive performance statistics for the main
 *          scheduler loop and individual tasks. It maintains:
 *          - Statistical tracking: minimum, maximum, average, and standard deviation of loop times
 *          - Filtered loop time: low-pass filtered timing for stable rate estimation
 *          - Long-running loop detection: counts loops exceeding the overtime threshold
 *          - Per-task timing metrics: execution time, tick count, overruns, and slips
 * 
 *          Performance data is used for system health monitoring, telemetry reporting via
 *          MAVLink, and diagnostics. Task-level statistics enable identification of CPU-bound
 *          operations and scheduling bottlenecks.
 * 
 * @note All timing values use explicit units: microseconds for time measurements, Hz for rates
 */
class PerfInfo {
public:
    PerfInfo() {}

    /**
     * @struct TaskInfo
     * @brief Per-task performance metrics tracked by the scheduler
     * 
     * @details This structure maintains execution statistics for individual scheduler tasks,
     *          enabling identification of CPU-intensive operations, timing violations, and
     *          scheduling delays.
     */
    struct TaskInfo {
        uint16_t min_time_us;       ///< @brief Minimum task execution time in microseconds
        uint16_t max_time_us;       ///< @brief Maximum task execution time in microseconds
        uint32_t elapsed_time_us;   ///< @brief Cumulative execution time in microseconds
        uint32_t tick_count;        ///< @brief Number of times task has executed
        uint16_t slip_count;        ///< @brief Number of times task was delayed due to CPU load
        uint16_t overrun_count;     ///< @brief Number of times task exceeded its max_time_micros budget

        /**
         * @brief Update performance counters for this task
         * 
         * @param[in] task_time_us Task execution time in microseconds
         * @param[in] overrun True if task exceeded its max_time_micros budget
         */
        void update(uint16_t task_time_us, bool overrun);

        /**
         * @brief Print formatted task statistics
         * 
         * @param[in] task_name Name of the task for display
         * @param[in] total_time Total elapsed time for percentage calculation
         * @param[out] str Expanding string to append formatted output
         */
        void print(const char* task_name, uint32_t total_time, ExpandingString& str) const;
    };

    /* Do not allow copies */
    CLASS_NO_COPY(PerfInfo);

    /**
     * @brief Reset all performance counters to initial state
     * 
     * @details Clears all accumulated statistics including min/max/avg times,
     *          loop counts, and filtered values. Does not affect task-level statistics.
     */
    void reset();

    /**
     * @brief Exclude current loop from statistics
     * 
     * @details Marks the current loop to be ignored in performance calculations.
     *          Useful for rejecting outliers caused by initialization or other
     *          non-representative events.
     */
    void ignore_this_loop();

    /**
     * @brief Record loop timing and update performance statistics
     * 
     * @param[in] time_in_micros Loop duration in microseconds
     * 
     * @warning Loop timing directly impacts real-time guarantees. Consistently
     *          high loop times may indicate CPU overload or scheduler configuration issues.
     * 
     * @note This is typically called at the end of each main scheduler loop to track
     *       timing and detect long-running loops that exceed the overtime threshold.
     */
    void check_loop_time(uint32_t time_in_micros);

    /**
     * @brief Return number of loops recorded in statistics
     * 
     * @return Loop count
     */
    uint16_t get_num_loops() const;

    /**
     * @brief Return maximum loop time observed
     * 
     * @return Maximum loop time in microseconds
     */
    uint32_t get_max_time() const;

    /**
     * @brief Return minimum loop time observed
     * 
     * @return Minimum loop time in microseconds
     */
    uint32_t get_min_time() const;

    /**
     * @brief Count loops exceeding overtime threshold
     * 
     * @return Number of long-running loops
     * 
     * @note Long-running loops are those that exceed the overtime threshold,
     *       typically indicating performance problems or CPU contention.
     */
    uint16_t get_num_long_running() const;

    /**
     * @brief Return average loop time
     * 
     * @return Average loop time in microseconds
     */
    uint32_t get_avg_time() const;

    /**
     * @brief Return standard deviation of loop time
     * 
     * @return Standard deviation in microseconds
     * 
     * @note High standard deviation indicates inconsistent loop timing,
     *       which may affect control loop stability.
     */
    uint32_t get_stddev_time() const;

    /**
     * @brief Return low-pass filtered loop time
     * 
     * @return Filtered loop time in seconds
     * 
     * @note The filtered value provides a stable estimate of loop timing
     *       for rate calculations and reduces sensitivity to transient spikes.
     */
    float    get_filtered_time() const;

    /**
     * @brief Return filtered loop rate
     * 
     * @return Loop rate in Hz
     * 
     * @note Calculated from the filtered loop time to provide a stable
     *       estimate of the actual scheduler loop rate.
     */
    float get_filtered_loop_rate_hz() const;

    /**
     * @brief Configure expected loop rate for overtime detection
     * 
     * @param[in] rate_hz Target loop rate in Hz
     * 
     * @note Sets the expected loop rate which is used to calculate the
     *       overtime threshold for detecting long-running loops.
     */
    void set_loop_rate(uint16_t rate_hz);

    /**
     * @brief Send performance telemetry via MAVLink/GCS
     * 
     * @note Called at 1Hz to report loop performance statistics to ground control
     *       station for monitoring and diagnostics.
     */
    void update_logging() const;

    /**
     * @brief Allocate per-task statistics array
     * 
     * @param[in] num_tasks Number of tasks to track
     * 
     * @details Allocates memory for task-level performance tracking. This array
     *          is used by @SYS/tasks.txt to provide detailed per-task statistics
     *          for performance analysis and debugging.
     */
    void allocate_task_info(uint8_t num_tasks);

    /**
     * @brief Deallocate task statistics array
     * 
     * @details Frees memory allocated by allocate_task_info(). Should be called
     *          during cleanup or when task configuration changes.
     */
    void free_task_info();

    /**
     * @brief Check if task info is allocated
     * 
     * @return true if task statistics array is allocated, false otherwise
     */
    bool has_task_info() { return _task_info != nullptr; }

    /**
     * @brief Retrieve task statistics for a specific task
     * 
     * @param[in] task_index Zero-based task index
     * 
     * @return Pointer to TaskInfo structure, or nullptr if invalid task_index
     *         or task info not allocated
     */
    const TaskInfo* get_task_info(uint8_t task_index) const {
        return (_task_info && task_index < _num_tasks) ? &_task_info[task_index] : nullptr;
    }

    /**
     * @brief Update task performance metrics after task execution
     * 
     * @param[in] task_index Zero-based task index
     * @param[in] task_time_us Task execution time in microseconds
     * @param[in] overrun true if task exceeded its max_time_micros budget
     * 
     * @details Called by the scheduler after each task execution to update
     *          min/max/average times and track budget overruns.
     */
    void update_task_info(uint8_t task_index, uint16_t task_time_us, bool overrun);

    /**
     * @brief Record task slip (delayed execution)
     * 
     * @param[in] task_index Zero-based task index
     * 
     * @details Increments the slip count for a task when it cannot be executed
     *          at its scheduled time due to CPU load or other higher-priority tasks.
     */
    void task_slipped(uint8_t task_index) {
        if (_task_info && task_index < _num_tasks) {
            _task_info[task_index].overrun_count++;
        }
    }

private:
    uint16_t loop_rate_hz;
    uint16_t overtime_threshold_micros;
    uint16_t loop_count;
    uint32_t max_time; // in microseconds
    uint32_t min_time; // in microseconds
    uint64_t sigma_time;
    uint64_t sigmasquared_time;
    uint16_t long_running;
    uint32_t last_check_us;
    float filtered_loop_time;
    bool ignore_loop;
    // performance monitoring
    uint8_t _num_tasks;
    TaskInfo* _task_info;
};

};

#endif  // AP_SCHEDULER_ENABLED
