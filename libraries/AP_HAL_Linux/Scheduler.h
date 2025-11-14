/**
 * @file Scheduler.h
 * @brief Linux thread-based scheduler with real-time POSIX scheduling
 * 
 * Implements task scheduling using POSIX threads with SCHED_FIFO real-time
 * scheduling for deterministic timing. Manages timer callbacks, I/O operations,
 * RC input processing, and UART communication in separate threads with
 * configurable priorities.
 */

#pragma once

#include <pthread.h>

#include "AP_HAL_Linux.h"

#include "Semaphores.h"
#include "Thread.h"

#define LINUX_SCHEDULER_MAX_TIMER_PROCS 10
#define LINUX_SCHEDULER_MAX_TIMESLICED_PROCS 10
#define LINUX_SCHEDULER_MAX_IO_PROCS 10

#define AP_LINUX_SENSORS_STACK_SIZE  256 * 1024
#define AP_LINUX_SENSORS_SCHED_POLICY  SCHED_FIFO
#define AP_LINUX_SENSORS_SCHED_PRIO 12

namespace Linux {

/**
 * @class Linux::Scheduler
 * @brief Multi-threaded task scheduler with real-time priorities
 * 
 * @details Implements ArduPilot scheduler using POSIX threads for Linux platforms.
 *          Unlike bare-metal HALs that use hardware timers, this implementation
 *          leverages Linux kernel scheduler with real-time priorities for
 *          deterministic execution.
 *          
 *          Thread architecture:
 *          - Main thread: Runs vehicle loop at configured rate (typically 400Hz)
 *          - Timer thread: Executes registered timer callbacks at high priority
 *          - I/O thread: Handles storage writes and non-timing-critical I/O
 *          - RCInput thread: Processes RC receiver input
 *          - UART thread: Manages serial port read/write operations
 *          
 *          Real-time scheduling (when running as root):
 *          - SCHED_FIFO policy ensures threads run to completion without preemption
 *          - Priority hierarchy: Main > Timer > Sensors > UART > RCInput > I/O
 *          - Typical priorities: Main=15, Timer=14, Sensors=12, UART=11, RCIn=10, I/O=1
 *          - mlockall() prevents memory paging for consistent latency
 *          
 *          Without real-time scheduling (non-root):
 *          - Falls back to SCHED_OTHER (normal Linux scheduling)
 *          - Higher timing jitter and less deterministic behavior
 *          - Acceptable for SITL/development but not flight hardware
 *          
 *          CPU affinity:
 *          - Threads can be pinned to specific CPU cores via set_cpu_affinity()
 *          - Reduces cache thrashing and improves cache locality
 *          - Useful on multi-core systems to isolate real-time workload
 *          
 *          Callback registration:
 *          - register_timer_process(): High-frequency callbacks (main loop rate)
 *          - register_io_process(): Low-priority I/O operations (background)
 *          - thread_create(): Custom threads for drivers/peripherals
 *          
 *          Stack sizes:
 *          - Sensor thread: 256KB (AP_LINUX_SENSORS_STACK_SIZE)
 *          - Custom threads: Configurable per thread_create() call
 *          - Larger than embedded HALs due to deeper call stacks on Linux
 *          
 *          Performance characteristics:
 *          - Timer jitter: 50-500μs (depends on kernel: PREEMPT_RT < PREEMPT < GENERIC)
 *          - Context switch latency: 5-50μs
 *          - Adequate for most flight control (800Hz+ IMU, 400Hz loop rate)
 *          - Not suitable for extremely tight timing (<10μs requirements)
 * 
 * @note Requires root privileges for SCHED_FIFO real-time scheduling
 * @note PREEMPT_RT kernel patch recommended for best performance
 * @warning Non-root operation has degraded timing performance
 * @warning Busy-waiting in callbacks blocks other threads at same priority
 * 
 * @see Thread for thread creation and management
 * @see PeriodicThread for periodic task execution
 */
class Scheduler : public AP_HAL::Scheduler {
public:
    /**
     * @brief Constructor initializes scheduler data structures
     * 
     * @note Does not create threads yet - deferred to init()
     */
    Scheduler();

    /**
     * @brief Downcast helper from generic scheduler pointer
     * 
     * @param[in] scheduler Pointer to AP_HAL::Scheduler base class
     * @return Scheduler* Downcasted pointer to Linux implementation
     */
    static Scheduler *from(AP_HAL::Scheduler *scheduler) {
        return static_cast<Scheduler*>(scheduler);
    }

    /**
     * @brief Initialize scheduler and start all threads
     * 
     * @details Initialization sequence:
     *          1. Configure real-time scheduling (if root)
     *          2. Set CPU affinity masks
     *          3. Create pthread_barrier for synchronized startup
     *          4. Start timer, I/O, RCInput, UART threads
     *          5. Wait for all threads to reach barrier (synchronized start)
     *          
     *          Thread startup synchronization ensures all threads are ready
     *          before callbacks start executing, preventing race conditions.
     * 
     * @note Called once during HAL initialization
     * @note Blocks until all threads are created and synchronized
     * @warning Prints warning if real-time scheduling unavailable (non-root)
     */
    void     init() override;

    /**
     * @brief Block execution for specified milliseconds
     * 
     * @param[in] ms Delay duration in milliseconds
     * 
     * @note Uses nanosleep() for delays - thread sleeps and yields CPU
     * @warning Do not call from timer callbacks - blocks entire thread
     * @warning Main loop delays break loop rate timing
     */
    void     delay(uint16_t ms) override;

    /**
     * @brief Block execution for specified microseconds
     * 
     * @param[in] us Delay duration in microseconds
     * 
     * @details Implementation depends on duration:
     *          - Very short (<100μs): May use busy-wait for accuracy
     *          - Longer delays: Uses nanosleep() to yield CPU
     * 
     * @note Minimum achievable delay ~1-10μs (kernel scheduler granularity)
     * @warning Do not call from timer callbacks - blocks entire thread
     */
    void     delay_microseconds(uint16_t us) override;

    /**
     * @brief Register callback for timer thread execution
     * 
     * @param[in] proc Member function callback to execute at main loop rate
     * 
     * @details Timer thread callbacks executed at high priority (SCHED_FIFO)
     *          in sequential order at main loop rate (typically 400Hz).
     *          
     *          Execution constraints:
     *          - Maximum LINUX_SCHEDULER_MAX_TIMER_PROCS (10) callbacks
     *          - All callbacks execute sequentially in timer thread
     *          - Combined execution time must be < loop period (2.5ms at 400Hz)
     * 
     * @note Callbacks run in timer thread context, not main thread
     * @warning Callback overruns cause loop rate violations and timing jitter
     * @warning No dynamic memory allocation in callbacks (not real-time safe)
     */
    void     register_timer_process(AP_HAL::MemberProc) override;

    /**
     * @brief Register callback for I/O thread execution
     * 
     * @param[in] proc Member function callback to execute in I/O thread
     * 
     * @details I/O thread callbacks run at low priority for non-critical operations:
     *          - Storage writes (parameter saves, log writes)
     *          - File operations
     *          - Network I/O (non-timing-critical)
     *          
     *          Execution characteristics:
     *          - Low priority (doesn't interfere with control loops)
     *          - No timing guarantees (may be preempted)
     *          - Maximum LINUX_SCHEDULER_MAX_IO_PROCS (10) callbacks
     * 
     * @note Callbacks run in I/O thread context
     * @note Suitable for operations that can tolerate variable latency
     */
    void     register_io_process(AP_HAL::MemberProc) override;

    /**
     * @brief Check if currently executing in main thread
     * 
     * @return bool True if executing in main thread, false otherwise
     * 
     * @note Compares pthread_self() with saved main thread ID
     * @note Useful for thread-safety assertions
     */
    bool     in_main_thread() const override;

    /**
     * @brief Register failsafe callback with periodic execution
     * 
     * @param[in] failsafe Callback function to execute periodically
     * @param[in] period_us Execution period in microseconds
     * 
     * @note Failsafe callback currently uses timer thread infrastructure
     * @warning Period must be achievable within timer thread budget
     */
    void     register_timer_failsafe(AP_HAL::Proc, uint32_t period_us) override;

    /**
     * @brief Mark system initialization complete
     * 
     * @note Signals that all HAL drivers are initialized and ready
     * @note Unblocks threads waiting on initialization barrier
     */
    void     set_system_initialized() override;

    /**
     * @brief Check if system initialization is complete
     * 
     * @return bool True if initialized, false if still initializing
     */
    bool     is_system_initialized() override { return _initialized; };

    /**
     * @brief Reboot the system
     * 
     * @param[in] hold_in_bootloader If true, reboot into bootloader mode
     * 
     * @details On Linux, this typically means:
     *          - Save parameters and close files
     *          - Call system() to execute reboot command (requires root)
     *          - Or simply exit() if reboot not possible
     * 
     * @note Linux reboot requires root privileges
     * @note hold_in_bootloader parameter not applicable to Linux systems
     */
    void     reboot(bool hold_in_bootloader) override;

    /**
     * @brief Stop clock at specified time (for replay/debugging)
     * 
     * @param[in] time_usec Timestamp to stop clock at in microseconds
     * 
     * @note Used by log replay system to freeze time
     * @note Allows deterministic replay of logged data
     */
    void     stop_clock(uint64_t time_usec) override;

    /**
     * @brief Get stopped clock timestamp
     * 
     * @return uint64_t Stopped clock time in microseconds (0 if not stopped)
     */
    uint64_t stopped_clock_usec() const { return _stopped_clock_usec; }

    /**
     * @brief Sleep for specified microseconds using nanosleep
     * 
     * @param[in] usec Sleep duration in microseconds
     * 
     * @note Wrapper around nanosleep() for internal use
     * @note More accurate than usleep() (which is deprecated)
     */
    void microsleep(uint32_t usec);

    /**
     * @brief Clean up scheduler and stop all threads
     * 
     * @note Called during HAL shutdown
     * @note Joins all threads to ensure clean exit
     */
    void teardown();

    /**
     * @brief Create new thread with specified priority
     * 
     * @param[in] proc Member function to execute in new thread
     * @param[in] name Thread name (for debugging/profiling)
     * @param[in] stack_size Stack size in bytes
     * @param[in] base Priority base (PRIORITY_BOOST, PRIORITY_MAIN, etc.)
     * @param[in] priority Priority offset from base (-10 to +10)
     * 
     * @return bool True if thread created successfully
     * 
     * @details Priority calculation:
     *          - Final priority = calculate_thread_priority(base, priority)
     *          - PRIORITY_BOOST: Highest priority (for critical drivers)
     *          - PRIORITY_MAIN: Main loop priority
     *          - PRIORITY_TIMER: Timer callback priority
     *          - PRIORITY_IO: Background I/O priority
     * 
     * @note Stack size should be generous on Linux (128KB+ typical)
     * @warning Too many high-priority threads cause priority inversion
     */
    bool thread_create(AP_HAL::MemberProc, const char *name, uint32_t stack_size, priority_base base, int8_t priority) override;
    
    /**
     * @brief Set CPU affinity mask for all threads
     * 
     * @param[in] cpu_affinity CPU set mask (which cores threads can run on)
     * 
     * @note Must be called before init() - affects thread creation only
     * @note Useful for isolating real-time workload to specific cores
     * @note On dual-core: Pin RT threads to core 1, leave core 0 for kernel
     */
    void set_cpu_affinity(const cpu_set_t &cpu_affinity) { _cpu_affinity = cpu_affinity; }

private:
    /**
     * @class Linux::Scheduler::SchedulerThread
     * @brief Periodic thread wrapper for scheduler tasks
     * 
     * @details Base class for timer, I/O, RCInput, and UART threads.
     *          Inherits from PeriodicThread to provide periodic execution
     *          with synchronized startup via pthread_barrier.
     * 
     * @note Internal scheduler implementation detail
     */
    class SchedulerThread : public PeriodicThread {
    public:
        SchedulerThread(Thread::task_t t, Scheduler &sched)
            : PeriodicThread(t)
            , _sched(sched)
        { }

    protected:
        bool _run() override;

        Scheduler &_sched;
    };

    /**
     * @brief Configure SCHED_FIFO real-time scheduling for all threads
     * 
     * @note Requires root privileges (CAP_SYS_NICE capability)
     * @note Enables mlockall() to prevent memory paging
     * @note Prints warning if real-time scheduling unavailable
     */
    void     init_realtime();

    /**
     * @brief Apply CPU affinity masks to all scheduler threads
     * 
     * @note Uses affinity set by set_cpu_affinity()
     * @note Applied during thread creation only
     */
    void     init_cpu_affinity();

    /**
     * @brief Wait for all threads to reach initialization barrier
     * 
     * @note Synchronizes thread startup to prevent race conditions
     * @note Blocks until all threads signal ready
     */
    void _wait_all_threads();

    /**
     * @brief Debug helper to print thread stack usage
     * 
     * @note Periodically logs stack high water marks
     * @note Useful for tuning stack sizes
     */
    void     _debug_stack();

    /**
     * Failsafe callback function pointer
     */
    AP_HAL::Proc _failsafe;

    /**
     * System initialization complete flag
     */
    bool _initialized;

    /**
     * Barrier for synchronized thread startup
     */
    pthread_barrier_t _initialized_barrier;

    /**
     * Array of registered timer callback functions
     */
    AP_HAL::MemberProc _timer_proc[LINUX_SCHEDULER_MAX_TIMER_PROCS];

    /**
     * Number of registered timer callbacks
     */
    uint8_t _num_timer_procs;

    /**
     * Flag indicating timer callback execution in progress
     */
    volatile bool _in_timer_proc;

    /**
     * Array of registered I/O callback functions
     */
    AP_HAL::MemberProc _io_proc[LINUX_SCHEDULER_MAX_IO_PROCS];

    /**
     * Number of registered I/O callbacks
     */
    uint8_t _num_io_procs;

    /**
     * @brief Calculate absolute thread priority from base and offset
     * 
     * @param[in] base Priority base category
     * @param[in] priority Relative priority offset
     * @return uint8_t Absolute priority value for SCHED_FIFO
     * 
     * @note Maps priority_base enum to Linux real-time priorities
     */
    uint8_t calculate_thread_priority(priority_base base, int8_t priority) const;

    /**
     * Timer thread instance (high priority)
     */
    SchedulerThread _timer_thread{FUNCTOR_BIND_MEMBER(&Scheduler::_timer_task, void), *this};

    /**
     * I/O thread instance (low priority, background tasks)
     */
    SchedulerThread _io_thread{FUNCTOR_BIND_MEMBER(&Scheduler::_io_task, void), *this};

    /**
     * RC input thread instance (medium priority)
     */
    SchedulerThread _rcin_thread{FUNCTOR_BIND_MEMBER(&Scheduler::_rcin_task, void), *this};

    /**
     * UART thread instance (medium priority)
     */
    SchedulerThread _uart_thread{FUNCTOR_BIND_MEMBER(&Scheduler::_uart_task, void), *this};

    /**
     * @brief Timer thread task - executes timer callbacks at high priority
     * 
     * @note Runs registered timer callbacks sequentially
     * @note Executes at main loop rate (typically 400Hz)
     */
    void _timer_task();

    /**
     * @brief I/O thread task - handles background I/O operations
     * 
     * @note Runs registered I/O callbacks at low priority
     * @note Used for storage writes, logging, non-critical file I/O
     */
    void _io_task();

    /**
     * @brief RC input thread task - processes RC receiver data
     * 
     * @note Polls RC input drivers for new data
     * @note Medium priority - important but not critical
     */
    void _rcin_task();

    /**
     * @brief UART thread task - manages serial port operations
     * 
     * @note Handles serial read/write for telemetry and peripherals
     * @note Medium priority to ensure responsive communication
     */
    void _uart_task();

    /**
     * @brief Execute all registered I/O callbacks
     */
    void _run_io();

    /**
     * @brief Execute UART driver read/write operations
     */
    void _run_uarts();

    /**
     * Stopped clock timestamp for replay (0 if clock running)
     */
    uint64_t _stopped_clock_usec;

    /**
     * Last time stack debug information was printed
     */
    uint64_t _last_stack_debug_msec;

    /**
     * Main thread pthread ID for in_main_thread() checks
     */
    pthread_t _main_ctx;

    /**
     * Semaphore for I/O thread synchronization
     */
    Semaphore _io_semaphore;

    /**
     * CPU affinity mask applied to all scheduler threads
     */
    cpu_set_t _cpu_affinity;
};

}
