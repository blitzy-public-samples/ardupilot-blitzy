/**
 * @file Scheduler.h
 * @brief QURT HAL Scheduler implementation using pthreads with QURT timer integration
 * 
 * @details This file implements the AP_HAL::Scheduler interface for Qualcomm Hexagon DSP
 *          running the QURT (Qualcomm User-space Real-Time) operating system. The scheduler
 *          uses POSIX pthreads for multithreading combined with qurt_timer_sleep for
 *          microsecond-accurate timing delays leveraging Hexagon hardware timers.
 *          
 *          The implementation spawns multiple worker threads with different priorities:
 *          - Main thread: Vehicle code execution
 *          - Timer thread: High-frequency callbacks (main loop, fast loop)
 *          - IO thread: Lower-priority I/O callbacks
 *          - Storage thread: Non-volatile storage operations
 *          - UART thread: Serial communication processing
 *          
 *          Thread synchronization uses pthread mutexes and priority-based scheduling
 *          to ensure real-time timer callbacks meet their deadlines (typically 400-1000Hz
 *          main loop rates).
 * 
 * @note QURT runs on the SLPI (Sensor Low Power Island) DSP on Snapdragon platforms,
 *       providing sensor processing and low-level flight control with minimal latency.
 * 
 * @warning DSP compute resources are limited compared to the application processor.
 *          All callback execution times must be carefully profiled and optimized.
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QURT
#include "AP_HAL_QURT_Namespace.h"
#include <sys/time.h>
#include <signal.h>
#include <pthread.h>

#define QURT_SCHEDULER_MAX_TIMER_PROCS 8

/**
 * @def APM_MAIN_PRIORITY
 * @brief Main vehicle thread priority on QURT RTOS
 * 
 * Priority value for the main thread executing vehicle control code. This thread
 * runs the vehicle's main loop and handles mode-specific logic.
 * 
 * @note QURT priority scale: higher number = higher priority (opposite of Linux nice values)
 * @note Value: 180 - High priority to ensure responsive vehicle control
 */
#define APM_MAIN_PRIORITY       180

/**
 * @def APM_TIMER_PRIORITY
 * @brief Timer callback thread priority (higher than main for real-time constraints)
 * 
 * Priority value for the timer thread executing high-frequency callbacks including
 * the main loop scheduler and fast loop callbacks. Must be higher priority than
 * the main thread to ensure timer callbacks execute at precise intervals.
 * 
 * @note Value: 181 - Highest priority to meet real-time deadlines for attitude estimation
 * @warning This is the highest priority thread in the system. Callbacks registered here
 *          must complete quickly (typically <1ms) to avoid starving other threads.
 */
#define APM_TIMER_PRIORITY      181

/**
 * @def APM_UART_PRIORITY
 * @brief UART I/O thread priority
 * 
 * Priority value for the UART processing thread handling serial communication with
 * peripherals, telemetry radios, and GPS receivers. Lower priority than timer and
 * main threads but higher than general I/O.
 * 
 * @note Value: 60 - Medium priority balancing latency and CPU availability
 */
#define APM_UART_PRIORITY        60

/**
 * @def APM_IO_PRIORITY
 * @brief General I/O thread priority
 * 
 * Priority value for the I/O callback thread handling lower-frequency, non-time-critical
 * operations such as logging, telemetry updates, and peripheral communication.
 * 
 * @note Value: 58 - Lower priority allows timer and UART threads to preempt when needed
 * @warning Priority tuning affects DSP real-time performance and timing guarantees.
 *          Improper priorities can cause timer callback jitter or missed deadlines.
 */
#define APM_IO_PRIORITY          58

/**
 * @class QURT::Scheduler
 * @brief pthread-based task scheduler with QURT timer_sleep integration for Hexagon DSP
 * 
 * @details Implements AP_HAL::Scheduler interface using POSIX pthreads for concurrent
 *          execution combined with qurt_timer_sleep() for microsecond-accurate delays.
 *          
 *          Threading Architecture:
 *          This scheduler spawns five separate pthread contexts, each with distinct
 *          priorities and responsibilities:
 *          
 *          1. Main Thread (APM_MAIN_PRIORITY=180):
 *             - Executes vehicle-specific control code
 *             - Handles mode transitions and mission logic
 *             - Processes pilot inputs and generates control outputs
 *          
 *          2. Timer Thread (APM_TIMER_PRIORITY=181):
 *             - Highest priority for real-time deadlines
 *             - Executes registered timer callbacks at precise intervals
 *             - Typically runs main loop (400-1000Hz) and fast loop callbacks
 *             - Critical for attitude estimation and control loop timing
 *          
 *          3. IO Thread (APM_IO_PRIORITY=58):
 *             - Lower priority for non-time-critical operations
 *             - Handles logging, telemetry, and sensor polling
 *             - Yields CPU to timer and main threads when needed
 *          
 *          4. Storage Thread:
 *             - Manages non-volatile storage operations (parameters, waypoints)
 *             - Handles EEPROM/flash writes without blocking flight-critical threads
 *          
 *          5. UART Thread (APM_UART_PRIORITY=60):
 *             - Serial communication processing
 *             - GPS, telemetry radio, and peripheral communication
 *          
 *          Scheduler Lifecycle:
 *          1. init() - Initialize scheduler data structures and thread contexts
 *          2. register_timer_process() - Register high-frequency callbacks
 *          3. register_io_process() - Register lower-priority callbacks
 *          4. hal_initialized() - Spawn all worker threads and begin execution
 *          5. Continuous execution until reboot()
 *          
 *          Thread Synchronization:
 *          Uses pthread mutexes to protect shared state and priority scheduling to ensure
 *          timer callbacks meet real-time deadlines. The _timer_suspended flag allows
 *          temporary disabling of timer callbacks during critical sections.
 *          
 *          Timing Implementation:
 *          - delay() and delay_microseconds() use qurt_timer_sleep() which provides
 *            hardware timer-based delays with microsecond accuracy
 *          - Timer callbacks use qurt_timer for precise periodic execution
 *          - All timing leverages Hexagon DSP hardware timer peripherals
 * 
 * @note Uses pthread mutexes and priority scheduling to ensure timer callbacks meet
 *       real-time deadlines (typically 1kHz main loop, 400Hz fast loop)
 * 
 * @warning DSP CPU constraints: Hexagon DSP has limited compute compared to application
 *          processor. Tune callback execution time carefully - timer callbacks should
 *          typically complete in <1ms to avoid jitter and missed deadlines.
 * 
 * @warning Memory constraints: Each pthread consumes DSP memory for stack (typically
 *          8KB-32KB per thread). Stack sizes must be tuned based on callback complexity
 *          to avoid overflow while minimizing memory usage on the resource-constrained DSP.
 * 
 * @see Thread.h for Thread and PeriodicThread helper classes used by vehicle code
 * @see AP_HAL::Scheduler for the abstract interface this class implements
 */
class QURT::Scheduler : public AP_HAL::Scheduler
{
public:
    Scheduler();
    /* AP_HAL::Scheduler methods */

    /**
     * @brief Initialize scheduler data structures and prepare thread contexts
     * 
     * @details Called during HAL initialization before any callbacks are registered.
     *          Sets up internal data structures, initializes mutexes, and prepares
     *          thread contexts. Does not spawn worker threads yet - that occurs in
     *          hal_initialized() after all subsystems have completed initialization.
     * 
     * @note Must be called before registering any timer or IO callbacks
     * @note Thread spawning deferred until hal_initialized() to allow proper initialization order
     */
    void     init() override;
    
    /**
     * @brief Blocking delay for specified milliseconds using qurt_timer_sleep
     * 
     * @details Suspends the calling thread for the specified number of milliseconds,
     *          yielding CPU to other threads. Uses qurt_timer_sleep() which provides
     *          hardware timer-based delays on the Hexagon DSP.
     *          
     *          During the delay, other threads (timer, IO, UART) continue executing
     *          based on their priority. This is a cooperative multitasking primitive.
     * 
     * @param[in] ms Delay duration in milliseconds (0-65535)
     * 
     * @note This yields CPU to other threads - not a busy-wait
     * @note For delays <1ms, consider delay_microseconds() for better precision
     * @warning Calling from timer thread may cause jitter in subsequent timer callbacks
     * @warning Long delays (>100ms) in time-critical threads can affect vehicle stability
     */
    void     delay(uint16_t ms) override;
    
    /**
     * @brief Microsecond-precision blocking delay using qurt_timer_sleep
     * 
     * @details Suspends the calling thread for the specified number of microseconds.
     *          Uses qurt_timer_sleep() with microsecond resolution for precise timing.
     *          
     *          For very short delays (<10us), this may consume significant CPU as the
     *          overhead of thread scheduling approaches the delay duration. For longer
     *          delays, it efficiently yields the CPU.
     * 
     * @param[in] us Delay duration in microseconds (0-65535)
     * 
     * @note Provides microsecond-accurate delays using Hexagon hardware timers
     * @warning CPU-intensive for delays <10us due to context switch overhead
     * @warning Avoid calling from timer callbacks as it blocks other timer processes
     */
    void     delay_microseconds(uint16_t us) override;
    
    /**
     * @brief Register high-frequency callback for timer thread execution
     * 
     * @details Registers a callback to be executed by the timer thread at high frequency
     *          (typically 400-1000Hz for main loop callbacks). Timer callbacks run at
     *          APM_TIMER_PRIORITY=181, the highest priority in the system, ensuring
     *          precise periodic execution for time-critical operations like attitude
     *          estimation and control loop calculations.
     *          
     *          Multiple callbacks can be registered (up to QURT_SCHEDULER_MAX_TIMER_PROCS).
     *          All registered callbacks execute sequentially in registration order on
     *          each timer tick.
     * 
     * @param[in] proc Member function pointer to callback (typically vehicle main loop)
     * 
     * @note Called from timer thread at APM_TIMER_PRIORITY (highest priority)
     * @note Typically used for main vehicle loop running at 400-1000Hz
     * @note All timer callbacks must complete quickly (target <1ms) to avoid jitter
     * @warning Timer callbacks block each other - long execution in one callback
     *          delays all subsequent callbacks and can cause missed deadlines
     * @warning Maximum of QURT_SCHEDULER_MAX_TIMER_PROCS (8) timer callbacks supported
     */
    void     register_timer_process(AP_HAL::MemberProc) override;
    
    /**
     * @brief Register lower-priority callback for IO thread execution
     * 
     * @details Registers a callback to be executed by the IO thread at lower priority
     *          (APM_IO_PRIORITY=58) than timer callbacks. IO callbacks are suitable for
     *          non-time-critical operations like logging, telemetry updates, sensor
     *          polling, and peripheral communication that can tolerate variable timing.
     *          
     *          Multiple callbacks can be registered (up to QURT_SCHEDULER_MAX_TIMER_PROCS).
     *          The IO thread yields CPU to timer and main threads when they need to execute.
     * 
     * @param[in] proc Member function pointer to callback
     * 
     * @note Called from IO thread at APM_IO_PRIORITY (lower priority than timer)
     * @note Suitable for logging, telemetry, and non-critical sensor updates
     * @note IO callbacks can be preempted by timer callbacks, causing variable timing
     * @warning Do not use for time-critical operations requiring precise periodic execution
     */
    void     register_io_process(AP_HAL::MemberProc) override;
    
    /**
     * @brief Register failsafe callback with specific execution period
     * 
     * @details Registers a special failsafe callback that executes periodically at the
     *          specified period. Failsafe callbacks monitor critical systems (RC input,
     *          battery voltage, GPS health) and trigger protective actions if failures
     *          are detected. Executes in the timer thread context at high priority.
     * 
     * @param[in] proc Function pointer to failsafe check callback
     * @param[in] period_us Execution period in microseconds (e.g., 1000 for 1kHz)
     * 
     * @note Executes in timer thread context at APM_TIMER_PRIORITY
     * @warning Must execute quickly to not block other timer callbacks - typically
     *          should complete in <100us to avoid affecting main loop timing
     * @warning Failsafe triggers can change vehicle mode or override pilot control
     */
    void     register_timer_failsafe(AP_HAL::Proc, uint32_t period_us) override;
    
    /**
     * @brief Temporarily disable timer callback execution
     * 
     * @details Sets the _timer_suspended flag to prevent timer callbacks from executing.
     *          Used during critical sections where timer callback execution would cause
     *          race conditions or data corruption (e.g., during parameter saves, sensor
     *          calibration, or when modifying shared data structures).
     *          
     *          While suspended, the timer thread continues running but skips callback
     *          execution. Must be paired with resume_timer_procs() to re-enable callbacks.
     * 
     * @note Must always be paired with resume_timer_procs() - never leave suspended
     * @warning Use sparingly and for minimal duration (<50ms if possible)
     * @warning Can cause attitude estimation drift if suspended too long (>100ms)
     * @warning May cause missed sensor updates and increased estimator lag
     * 
     * @see resume_timer_procs()
     */
    void     suspend_timer_procs();
    
    /**
     * @brief Re-enable timer callback execution after suspension
     * 
     * @details Clears the _timer_suspended flag to allow timer callbacks to resume
     *          execution. Must be called after suspend_timer_procs() to restore normal
     *          operation. Timer callbacks will resume on the next timer tick.
     * 
     * @note Always call after suspend_timer_procs() to restore normal operation
     * @warning Failure to resume will permanently disable timer callbacks, causing
     *          vehicle instability and potential crash
     * 
     * @see suspend_timer_procs()
     */
    void     resume_timer_procs();
    
    /**
     * @brief Trigger DSP reboot (SLPI restart)
     * 
     * @details Initiates a reboot of the Hexagon DSP. On Snapdragon platforms, this
     *          restarts the SLPI (Sensor Low Power Island) subsystem. The application
     *          processor may need to reinitialize communication with the DSP after reboot.
     *          
     *          This is typically used for critical error recovery or firmware updates.
     * 
     * @param[in] hold_in_bootloader Unused on QURT platform (bootloader hold not supported)
     * 
     * @warning Will restart entire SLPI subsystem, disrupting all sensor processing
     * @warning May require application processor intervention to restore communication
     * @warning Vehicle will lose control during reboot - only use when vehicle is disarmed
     *          or in emergency situations
     */
    void     reboot(bool hold_in_bootloader) override;

    /**
     * @brief Check if currently executing in main thread context
     * 
     * @details Compares the current pthread context (pthread_self()) with the stored
     *          main thread context (_main_thread_ctx) to determine if the calling code
     *          is executing in the main vehicle thread versus a worker thread (timer,
     *          IO, storage, UART).
     *          
     *          Useful for assertions and conditional logic that should only execute
     *          in specific thread contexts.
     * 
     * @return true if executing in main thread, false if in worker thread
     * 
     * @note Thread-safe, uses pthread_self() for context comparison
     * @note Returns false when called from timer, IO, storage, or UART threads
     */
    bool     in_main_thread() const override;
    
    /**
     * @brief Signal that HAL initialization is complete and spawn worker threads
     * 
     * @details Called after all HAL subsystems have initialized and callbacks have been
     *          registered. Spawns the timer, IO, storage, and UART worker threads which
     *          begin executing their respective callbacks. After this call, the scheduler
     *          is fully operational with all threads running concurrently.
     *          
     *          This two-phase initialization (init() then hal_initialized()) allows proper
     *          initialization ordering: HAL subsystems initialize, register callbacks, then
     *          threads spawn and begin execution.
     * 
     * @note Called once during system initialization after all callbacks registered
     * @note After this call, timer and IO callbacks begin executing concurrently
     */
    void     hal_initialized();

    /**
     * @brief Mark the complete system as initialized (vehicle code ready)
     * 
     * @details Sets the _initialized flag indicating that not only the HAL but also
     *          all vehicle-specific initialization has completed. This signals that the
     *          vehicle is ready for full operation including arming and flight.
     * 
     * @note Called by vehicle code after completing vehicle-specific initialization
     * @note Checked by is_system_initialized() to gate certain operations
     */
    void     set_system_initialized() override;
    
    /**
     * @brief Check if complete system initialization has finished
     * 
     * @details Returns the _initialized flag indicating whether both HAL and vehicle
     *          initialization have completed. Used to gate operations that should only
     *          occur after full system initialization.
     * 
     * @return true if system fully initialized, false if still initializing
     * 
     * @note Returns false during initialization, true after set_system_initialized()
     */
    bool     is_system_initialized() override
    {
        return _initialized;
    }

    /**
     * @brief Create a new pthread with specified priority and stack size
     * 
     * @details Creates a new POSIX thread (pthread) to execute the provided callback
     *          function. Calculates the thread's QURT priority based on the priority_base
     *          and relative priority offset. Configures the thread with the specified
     *          stack size (important on memory-constrained DSP).
     *          
     *          Used by vehicle code and libraries to create custom threads for specific
     *          tasks (e.g., sensor processing, communication handling).
     * 
     * @param[in] proc Member function pointer to execute in new thread
     * @param[in] name Thread name for debugging (currently unused on QURT)
     * @param[in] stack_size Stack size in bytes (typically 8KB-32KB on DSP)
     * @param[in] base Priority base type (PRIORITY_BOOST, PRIORITY_MAIN, etc.)
     * @param[in] priority Relative priority offset from base
     * 
     * @return true if thread created successfully, false on failure
     * 
     * @note QURT priority calculated using calculate_thread_priority(base, priority)
     * @note Stack size critical on DSP - too large wastes memory, too small causes overflow
     * @warning Each thread consumes DSP memory - limit thread creation on resource-constrained platform
     * @warning Thread creation failure is silent - check return value for error handling
     * 
     * @see calculate_thread_priority() for priority calculation algorithm
     */
    bool thread_create(AP_HAL::MemberProc proc, const char *name,
                       uint32_t stack_size, priority_base base, int8_t priority) override;

private:
    bool _initialized;                  ///< System fully initialized (HAL + vehicle code ready)
    volatile bool _hal_initialized;     ///< HAL initialization complete, worker threads spawned
    AP_HAL::Proc _delay_cb;             ///< Optional callback to execute during delay() calls
    uint16_t _min_delay_cb_ms;          ///< Minimum delay in ms before calling _delay_cb
    AP_HAL::Proc _failsafe;             ///< Registered failsafe callback function

    /**
     * @brief Flag indicating timer callbacks are temporarily disabled
     * 
     * When true, timer thread skips callback execution to protect critical sections.
     * Set by suspend_timer_procs(), cleared by resume_timer_procs().
     * 
     * @warning Must not remain true for extended periods (>100ms) to avoid estimation drift
     */
    volatile bool _timer_suspended;

    AP_HAL::MemberProc _timer_proc[QURT_SCHEDULER_MAX_TIMER_PROCS];  ///< Array of registered timer callbacks (max 8)
    uint8_t _num_timer_procs;           ///< Number of registered timer callbacks (0-8)
    volatile bool _in_timer_proc;       ///< True when timer thread is executing callbacks

    AP_HAL::MemberProc _io_proc[QURT_SCHEDULER_MAX_TIMER_PROCS];     ///< Array of registered IO callbacks (max 8)
    uint8_t _num_io_procs;              ///< Number of registered IO callbacks (0-8)
    volatile bool _in_io_proc;          ///< True when IO thread is executing callbacks

    volatile bool _timer_event_missed;  ///< True if timer callback execution overran deadline

    /**
     * @brief pthread context handles for all scheduler threads
     * 
     * Stores pthread_t handles for context comparison and thread management:
     * - _main_thread_ctx: Main vehicle control thread (APM_MAIN_PRIORITY=180)
     * - _timer_thread_ctx: High-frequency timer callbacks (APM_TIMER_PRIORITY=181)
     * - _io_thread_ctx: Lower-priority IO callbacks (APM_IO_PRIORITY=58)
     * - _storage_thread_ctx: Non-volatile storage operations
     * - _uart_thread_ctx: Serial communication processing (APM_UART_PRIORITY=60)
     * 
     * @note Used by in_main_thread() for thread context detection
     */
    pthread_t _main_thread_ctx;         ///< Main vehicle thread context
    pthread_t _timer_thread_ctx;        ///< Timer callback thread context (highest priority)
    pthread_t _io_thread_ctx;           ///< IO callback thread context (low priority)
    pthread_t _storage_thread_ctx;      ///< Storage operations thread context
    pthread_t _uart_thread_ctx;         ///< UART processing thread context

    /**
     * @brief Calculate QURT thread priority from base and offset
     * 
     * Converts abstract priority_base enum and relative priority offset into
     * concrete QURT priority value (0-255 scale, higher = more priority).
     * 
     * @param[in] base Priority base type (PRIORITY_BOOST, PRIORITY_MAIN, etc.)
     * @param[in] priority Relative priority offset from base
     * @return Calculated QURT priority value (0-255)
     */
    uint8_t calculate_thread_priority(priority_base base, int8_t priority) const;

    /**
     * @brief Timer thread entry point (static pthread callback)
     * 
     * Executes timer callbacks at high frequency with precise timing.
     * Runs at APM_TIMER_PRIORITY=181 (highest priority).
     * 
     * @param[in] arg Pointer to Scheduler instance (this)
     * @return nullptr (pthread return value, unused)
     */
    static void *_timer_thread(void *arg);
    
    /**
     * @brief IO thread entry point (static pthread callback)
     * 
     * Executes IO callbacks at lower priority for non-time-critical operations.
     * Runs at APM_IO_PRIORITY=58.
     * 
     * @param[in] arg Pointer to Scheduler instance (this)
     * @return nullptr (pthread return value, unused)
     */
    static void *_io_thread(void *arg);
    
    /**
     * @brief Storage thread entry point (static pthread callback)
     * 
     * Handles non-volatile storage operations (parameter saves, waypoint storage).
     * 
     * @param[in] arg Pointer to Scheduler instance (this)
     * @return nullptr (pthread return value, unused)
     */
    static void *_storage_thread(void *arg);
    
    /**
     * @brief UART thread entry point (static pthread callback)
     * 
     * Processes serial communication with peripherals and telemetry radios.
     * Runs at APM_UART_PRIORITY=60.
     * 
     * @param[in] arg Pointer to Scheduler instance (this)
     * @return nullptr (pthread return value, unused)
     */
    static void *_uart_thread(void *arg);

    /**
     * @brief Execute all registered timer callbacks sequentially
     * 
     * Iterates through _timer_proc[] array and executes each registered callback.
     * Checks _timer_suspended flag before execution.
     * 
     * @param[in] called_from_timer_thread True if called from timer thread context
     * 
     * @note Callbacks execute sequentially - long callbacks delay subsequent ones
     */
    void _run_timers(bool called_from_timer_thread);
    
    /**
     * @brief Execute all registered IO callbacks sequentially
     * 
     * Iterates through _io_proc[] array and executes each registered IO callback.
     * 
     * @note IO callbacks can be preempted by higher-priority timer thread
     */
    void _run_io(void);
};
#endif



