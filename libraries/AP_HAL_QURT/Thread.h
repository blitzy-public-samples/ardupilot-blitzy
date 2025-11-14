/*
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file Thread.h
 * @brief QURT thread wrapper classes built on pthreads with QURT timer integration
 * 
 * @details This file provides Thread and PeriodicThread classes for the Qualcomm
 *          Hexagon DSP platform. These classes wrap pthread functionality while
 *          handling Hexagon-specific requirements such as stack alignment and
 *          integrating with QURT hardware timers for accurate periodic execution.
 *          
 *          The QURT HAL runs on Qualcomm Hexagon DSP processors, which have
 *          limited memory and require careful thread stack sizing. These classes
 *          simplify thread creation while enforcing proper alignment and resource
 *          management for the DSP environment.
 * 
 * @note QURT (Qualcomm Real-Time) is the RTOS running on Hexagon DSP cores
 * 
 * @see libraries/AP_HAL_QURT/Scheduler.h for thread priority constants
 * @see libraries/AP_HAL_QURT/system.cpp for qurt_timer_sleep implementation
 */
#pragma once

#include <pthread.h>
#include <inttypes.h>
#include <stdlib.h>

#include <AP_HAL/utility/functor.h>

namespace QURT
{

/**
 * @class QURT::Thread
 * @brief pthread wrapper for creating and managing threads on Hexagon DSP
 * 
 * @details Provides a simplified interface for pthread creation with automatic
 *          stack alignment for Hexagon DSP requirements. This class abstracts
 *          the complexity of pthread management and ensures proper resource
 *          handling in the DSP environment where memory is constrained.
 *          
 *          Thread Lifecycle:
 *          1. Constructor - Create Thread object with task callback
 *          2. set_stack_size() - Configure stack size (optional, before start)
 *          3. start() - Creates pthread and begins execution
 *          4. Thread executes task callback
 *          5. join() or detach - Cleanup and resource release
 *          
 *          Stack Alignment:
 *          Hexagon DSP requires 8-byte stack alignment for proper operation.
 *          This class handles alignment automatically when creating pthreads,
 *          ensuring all stacks meet hardware requirements without manual
 *          intervention by the caller.
 *          
 *          Usage Pattern - Device Polling Thread:
 *          @code
 *          void sensor_poll_task() {
 *              while (!should_exit) {
 *                  read_sensor_data();
 *                  process_data();
 *              }
 *          }
 *          
 *          Thread poll_thread(FUNCTOR_BIND(&sensor_poll_task));
 *          poll_thread.set_stack_size(16384);  // 16KB stack
 *          poll_thread.start("SensorPoll", SCHED_FIFO, 10);
 *          @endcode
 *          
 *          Usage Pattern - Background Task Thread:
 *          @code
 *          void background_task() {
 *              perform_non_critical_work();
 *          }
 *          
 *          Thread bg_thread(FUNCTOR_BIND(&background_task));
 *          bg_thread.start("Background", SCHED_OTHER, 5);
 *          bg_thread.join();  // Wait for completion
 *          @endcode
 * 
 * @note Stack Size Considerations:
 *       DSP memory is limited (typically 8-32MB total). Default stack sizes
 *       should be minimal but sufficient for worst-case usage. Typical range:
 *       - Simple tasks: 8KB-16KB
 *       - Moderate complexity: 16KB-32KB
 *       - Complex with deep call stacks: 32KB-64KB
 *       
 * @note Thread Priorities:
 *       Thread priorities should generally be lower than APM_MAIN_PRIORITY
 *       to avoid starving the main vehicle control loop. See Scheduler.h
 *       for thread priority constants and scheduling policy definitions.
 * 
 * @warning Stack Overflow Risks:
 *          Stack overflow on DSP can corrupt memory without detection or
 *          immediate crash. Carefully size stacks for worst-case usage
 *          including all function calls, local variables, and interrupt
 *          handler overhead. Use get_stack_usage() to monitor actual usage.
 * 
 * @warning DSP CPU Budget:
 *          Each thread consumes CPU time. The sum of all thread execution
 *          times must leave sufficient budget for the main control loop.
 *          Monitor CPU usage and adjust thread priorities/periods accordingly.
 * 
 * @see Scheduler.h for thread priority constants and scheduling policy
 * @see PeriodicThread for threads that need fixed-rate execution
 */
class Thread
{
public:
    FUNCTOR_TYPEDEF(task_t, void);

    Thread(task_t t) : _task(t) { }

    virtual ~Thread() { }

    /**
     * @brief Create and start pthread execution
     * 
     * @details Creates a new pthread with configured stack size and starts
     *          executing the task callback. Stack is automatically aligned
     *          to 8-byte boundary as required by Hexagon DSP hardware.
     *          
     *          This method allocates pthread resources and begins concurrent
     *          execution. If stack size was not set via set_stack_size(),
     *          the system default stack size will be used.
     * 
     * @param[in] name       Thread name for debugging (shown in logs and debugger)
     * @param[in] policy     Scheduling policy (SCHED_FIFO, SCHED_RR, SCHED_OTHER)
     * @param[in] prio       Thread priority value (higher = more priority)
     * 
     * @return true if pthread created and started successfully, false on failure
     * 
     * @note Can only be called once per Thread object. Subsequent calls will fail.
     * @note Thread begins execution immediately after successful start.
     * 
     * @warning Start must be called from a different thread than the one being created.
     * @warning Failed start leaves object in invalid state; create new Thread object.
     */
    bool start(const char *name, int policy, int prio);

    /**
     * @brief Check if currently executing in this thread context
     * 
     * @details Compares pthread_self() handle with this thread's pthread handle
     *          to determine if the calling code is executing within this thread.
     *          Useful for assertions and debugging to verify execution context.
     * 
     * @return true if pthread_self() matches this thread's handle, false otherwise
     * 
     * @note Returns false if thread has not been started yet
     * @note Thread-safe, can be called from any thread
     */
    bool is_current_thread();

    bool is_started() const
    {
        return _started;
    }

    size_t get_stack_usage();

    /**
     * @brief Configure thread stack size before starting
     * 
     * @details Sets the stack size for the pthread that will be created when
     *          start() is called. Stack will be automatically aligned to 8-byte
     *          boundary as required by Hexagon DSP architecture. This method
     *          must be called before start() to have any effect.
     *          
     *          Stack Size Guidelines for Hexagon DSP:
     *          - Minimum practical: 4KB (very simple tasks only)
     *          - Typical range: 8KB-32KB for most ArduPilot tasks
     *          - Maximum recommended: 64KB (only for complex operations)
     *          
     *          Consider worst-case stack usage including:
     *          - All local variables in call stack
     *          - Function call overhead
     *          - Interrupt handler stack usage
     *          - Library function requirements
     * 
     * @param[in] stack_size Size in bytes (will be 8-byte aligned automatically)
     * 
     * @return true if stack size set successfully, false if already started
     * 
     * @note Must call before start() - has no effect after thread is started
     * @note Actual allocated size may be slightly larger due to alignment
     * @note Use get_stack_usage() after running to verify size is adequate
     * 
     * @warning Undersized stacks cause memory corruption without immediate detection
     * @warning Oversized stacks waste scarce DSP memory resources
     */
    bool set_stack_size(size_t stack_size);

    void set_auto_free(bool auto_free)
    {
        _auto_free = auto_free;
    }

    virtual bool stop()
    {
        return false;
    }

    /**
     * @brief Wait for thread completion and cleanup resources
     * 
     * @details Blocks the calling thread until this thread completes execution
     *          and exits. Once join() returns, the pthread resources are cleaned
     *          up and the Thread object can be safely destroyed. This is the
     *          standard synchronization mechanism for waiting on thread completion.
     *          
     *          Typical usage: Create thread, start it, perform other work, then
     *          join() to wait for thread to finish before proceeding or exiting.
     * 
     * @return true if join completed successfully, false on error
     * 
     * @note Blocking call - calling thread will wait until this thread exits
     * @note Use for thread synchronization and ensuring work is complete
     * @note Cannot join a thread from itself (will deadlock)
     * @note Can only join once per thread
     * 
     * @warning Calling join() from within the same thread will deadlock
     * @warning Not calling join() on non-detached threads leaks pthread resources
     */
    bool join();

protected:
    static void *_run_trampoline(void *arg);

    /*
     * Run the task assigned in the constructor. May be overriden in case it's
     * preferred to use Thread as an interface or when user wants to aggregate
     * some initialization or teardown for the thread.
     */
    virtual bool _run();

    void _poison_stack();

    task_t _task;
    bool _started = false;
    bool _should_exit = false;
    bool _auto_free = false;
    pthread_t _ctx = 0;

    struct stack_debug {
        uint32_t *start;
        uint32_t *end;
    } _stack_debug;

    size_t _stack_size = 0;
};

/**
 * @class QURT::PeriodicThread
 * @brief Thread that executes callback at fixed periodic intervals using QURT timer_sleep
 * 
 * @details Extends Thread to provide periodic execution pattern commonly needed
 *          in ArduPilot for tasks such as sensor polling, telemetry transmission,
 *          and background data processing. Uses QURT hardware timers to achieve
 *          microsecond-accurate periodic execution on the Hexagon DSP.
 *          
 *          Periodic Execution Loop:
 *          @code
 *          while (running) {
 *              callback();                  // Execute user task
 *              qurt_timer_sleep(period);    // Sleep until next period
 *          }
 *          @endcode
 *          
 *          Timing Accuracy:
 *          QURT timer_sleep() provides microsecond-accurate delays using Hexagon
 *          hardware timers. Typical jitter is <10us for sleep periods >100us.
 *          Actual period accuracy depends on callback execution time and DSP
 *          scheduling load from other threads.
 *          
 *          Usage Pattern - Sensor Polling at Fixed Rate:
 *          @code
 *          void poll_sensor() {
 *              uint8_t data[16];
 *              spi_device->transfer(data, sizeof(data));
 *              process_sensor_data(data);
 *          }
 *          
 *          PeriodicThread sensor_thread(FUNCTOR_BIND(&poll_sensor));
 *          sensor_thread.set_stack_size(16384);     // 16KB stack
 *          sensor_thread.set_rate(1000);            // 1kHz = 1000us period
 *          sensor_thread.start("IMU", SCHED_FIFO, 15);
 *          @endcode
 *          
 *          Usage Pattern - Background Telemetry at 10Hz:
 *          @code
 *          void send_telemetry() {
 *              mavlink_msg_heartbeat_send();
 *          }
 *          
 *          PeriodicThread telem_thread(FUNCTOR_BIND(&send_telemetry));
 *          telem_thread.set_rate(10);               // 10Hz = 100000us period
 *          telem_thread.start("Telem", SCHED_OTHER, 5);
 *          @endcode
 * 
 * @note Period Jitter:
 *       Period jitter depends on callback execution time and DSP scheduling load.
 *       Callback execution should be significantly faster than the period to
 *       maintain consistent timing. For 1kHz operation, callback should complete
 *       in <500us to allow margin for scheduling and timer overhead.
 *       
 * @note Typical Period Range:
 *       - High frequency: 1000us (1kHz) for fast sensor polling
 *       - Medium frequency: 10000us (100Hz) for control loops
 *       - Low frequency: 1000000us (1Hz) for slow background tasks
 *       
 * @note QURT Timer Resolution:
 *       Hexagon hardware timers provide <10us jitter for sleep periods >100us.
 *       Very short periods (<100us) may experience increased jitter due to
 *       scheduling overhead approaching timer resolution.
 * 
 * @warning Long Callback Execution:
 *          If callback execution time exceeds the configured period, the next
 *          execution will be delayed until the callback completes. This causes
 *          period skipping and timing irregularity. Monitor callback execution
 *          time to ensure it remains well below the period.
 *          
 * @warning DSP CPU Budget:
 *          Each periodic thread consumes CPU time = callback_duration × frequency.
 *          The sum of all periodic threads must leave sufficient CPU budget for
 *          the main vehicle control loop. Example: 10 threads at 1kHz with 100us
 *          callbacks = 10 × 0.0001s × 1000Hz = 100% CPU (unsustainable).
 *          Keep total periodic thread CPU usage below 50% to ensure main loop
 *          has adequate CPU time.
 * 
 * @see Thread base class for stack sizing and priority configuration
 * @see Scheduler.h for thread priority constants (APM_MAIN_PRIORITY, etc.)
 */
class PeriodicThread : public Thread
{
public:
    PeriodicThread(Thread::task_t t)
        : Thread(t)
    { }

    /**
     * @brief Configure execution rate in Hertz
     * 
     * @details Sets the periodic execution frequency by converting Hz to
     *          microsecond period for internal timer. Callback will execute
     *          at this rate (1000Hz = every 1000 microseconds).
     *          
     *          Common rates:
     *          - 1000 Hz = 1ms period (fast sensor polling)
     *          - 400 Hz = 2.5ms period (main loop rate for copters)
     *          - 100 Hz = 10ms period (medium rate tasks)
     *          - 50 Hz = 20ms period (telemetry, logging)
     *          - 10 Hz = 100ms period (slow background tasks)
     *          - 1 Hz = 1s period (very slow monitoring)
     * 
     * @param[in] rate_hz Period in Hertz (executions per second)
     * 
     * @return true if rate set successfully
     * 
     * @note Typical range: 1Hz to 1000Hz
     * @note Must call before start() for rate to take effect
     * @note Higher rates consume more CPU - ensure callback is fast enough
     * 
     * @warning Rates >1kHz may not be achievable depending on callback duration
     */
    bool set_rate(uint32_t rate_hz);

    /**
     * @brief Signal thread to stop execution
     * 
     * @details Sets flag to exit periodic loop. Thread will stop after the
     *          current callback completes, then exit cleanly. This is a
     *          non-blocking call - use join() afterward to wait for actual
     *          thread termination and resource cleanup.
     *          
     *          Shutdown sequence:
     *          1. stop() - Signal thread to exit (returns immediately)
     *          2. Current callback completes
     *          3. Thread checks exit flag and breaks loop
     *          4. Thread exits
     *          5. join() - Wait for cleanup (blocking)
     * 
     * @return true if stop signal sent successfully
     * 
     * @note Non-blocking - thread stops after current callback completes
     * @note Use join() after stop() to wait for actual thread exit
     * @note Thread exits cleanly, releasing all resources
     */
    bool stop() override;

protected:
    bool _run() override;

    uint64_t _period_usec = 0;
};

}
