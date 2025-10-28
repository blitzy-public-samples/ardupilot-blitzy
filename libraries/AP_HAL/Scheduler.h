/**
 * @file Scheduler.h
 * @brief Task scheduler interface for cooperative multitasking and timing
 * 
 * Defines the HAL interface for task scheduling, delays, timing, and thread management.
 * Supports both cooperative scheduling (delay/yield) and preemptive threading (thread_create).
 * Provides microsecond-resolution timing for control loops and sensor sampling.
 * 
 * Source: libraries/AP_HAL/Scheduler.h
 */

#pragma once

#include <stdint.h>

#include <AP_Common/AP_Common.h>

#include "AP_HAL_Boards.h"
#include "AP_HAL_Namespace.h"


/**
 * @class AP_HAL::Scheduler
 * @brief Abstract interface for task scheduling and timing services
 * 
 * @details Manages execution of vehicle tasks, background threads, and timing control.
 *          ArduPilot uses a hybrid scheduling model:
 *          - Main thread: Runs scheduled tasks (vehicle loop, sensors, control, telemetry)
 *          - IO threads: Background I/O operations (sensor polling, CAN, logging)
 *          - Timer callbacks: High-frequency interrupt-driven tasks
 *          
 *          Main loop execution:
 *          1. Scheduler sorts tasks by priority
 *          2. Tasks execute in priority order consuming time budget
 *          3. Delay until next loop time (typically 400Hz for copters, 50Hz for planes)
 *          4. Repeat
 *          
 *          Task scheduling requirements:
 *          - Fast tasks (<100μs): Run every loop at main thread rate
 *          - Medium tasks (100μs-1ms): Run at divided rate (e.g., 100Hz, 50Hz)
 *          - Slow tasks (>1ms): Run at low rate (e.g., 10Hz, 1Hz) or background thread
 *          
 *          Thread safety:
 *          - delay/delay_microseconds: Yield current thread, allow other tasks
 *          - register_timer_process/register_io_process: Background execution
 *          - register_delay_callback: Deferred execution in main thread
 * 
 * @note Timing functions return microseconds in 32-bit or 64-bit precision
 * @warning Never delay in interrupt handlers - corrupts timing and causes crashes
 * @warning Long delays (>10ms) in main thread starve flight control - use background threads
 * 
 * Source: libraries/AP_HAL/Scheduler.h
 */
class AP_HAL::Scheduler {
public:
    Scheduler() {}
    
    /**
     * @brief Initialize the scheduler subsystem
     * 
     * @details Called during HAL initialization to set up scheduling infrastructure.
     *          Platform-specific implementations initialize thread contexts, timers,
     *          and priority configurations.
     * 
     * @note Called once during boot before any other scheduler methods
     */
    virtual void     init() = 0;
    
    /**
     * @brief Delay execution for specified milliseconds, yielding to other tasks
     * 
     * @param[in] ms Delay duration in milliseconds (0-65535)
     * 
     * @details Suspends execution of the current thread for approximately ms milliseconds.
     *          During the delay, the scheduler may execute other tasks, making this a
     *          cooperative yield point. The actual delay may be slightly longer due to
     *          scheduler quantum and task execution time.
     *          
     *          Internally may call registered delay callbacks if delay exceeds the
     *          minimum callback threshold (see register_delay_callback).
     * 
     * @note This is NOT a busy-wait - other tasks execute during delay
     * @note Actual delay may be slightly longer than requested
     * @warning Do not call from interrupt handlers or timer callbacks
     * @warning Delays >10ms in main thread impact flight control performance
     * @warning Maximum delay is 65535ms (~65 seconds) due to uint16_t parameter
     * 
     * @see delay_microseconds() for microsecond-precision delays
     * @see register_delay_callback() for executing code during long delays
     */
    virtual void     delay(uint16_t ms) = 0;

    /**
     * @brief Delay execution for specified microseconds with high accuracy
     * 
     * @param[in] us Delay duration in microseconds (0-65535)
     * 
     * @details Suspends execution for approximately us microseconds. This needs to be as
     *          accurate as possible - preferably within 100 microseconds. Implementation
     *          may use busy-waiting for very short delays and yielding for longer delays.
     *          
     *          Typical implementation strategy:
     *          - us < 100: Busy-wait loop (no context switch overhead)
     *          - us >= 100: Yield to scheduler with high-precision timer
     * 
     * @note Accuracy depends on platform timer resolution and interrupt latency
     * @note Very short delays (<10μs) may be less accurate due to overhead
     * @warning Do not call from interrupt handlers or timer callbacks
     * @warning Maximum delay is 65535μs (~65ms) due to uint16_t parameter
     * 
     * @see delay() for millisecond-precision delays
     * @see delay_microseconds_boost() for priority-boosted delays
     */
    virtual void     delay_microseconds(uint16_t us) = 0;

    /**
     * @brief Delay with temporary priority boost for main thread
     * 
     * @param[in] us Delay duration in microseconds (0-65535)
     * 
     * @details Delays for the specified microseconds and on supported platforms boosts
     *          the priority of the main thread for a short time when the delay completes.
     *          The aim is to ensure the main thread runs at a higher priority than
     *          drivers for the start of each loop, reducing jitter in control loop timing.
     *          
     *          Priority boost ensures that when the main loop wakes up, it gets CPU time
     *          before background I/O threads, improving control loop timing consistency.
     *          
     *          Default implementation: Falls back to delay_microseconds() on platforms
     *          without priority boost support.
     * 
     * @note Priority boost is temporary and automatically expires
     * @note Not all platforms support priority boosting
     * @warning Call boost_end() if exiting main loop early to release priority boost
     * 
     * @see delay_microseconds() for standard delays
     * @see boost_end() to manually end priority boost
     */
    virtual void     delay_microseconds_boost(uint16_t us) { delay_microseconds(us); }

    /**
     * @brief Inform scheduler that an expected long delay is starting
     * 
     * @param[in] ms Expected delay duration in milliseconds (0 to cancel)
     * 
     * @details Notifies the scheduler that the main thread is about to perform an
     *          operation that may take an extended amount of time. This can be used
     *          to prevent watchdog reset during expected long delays such as:
     *          - Flash chip erasing
     *          - SD card operations
     *          - Compass calibration
     *          - Parameter loading
     *          
     *          A value of zero cancels the previous expected delay notification.
     *          
     *          Use the EXPECT_DELAY_MS() macro for automatic scope-based management.
     * 
     * @note Default implementation does nothing (no-op)
     * @note Pair each non-zero call with a zero call to cancel
     * 
     * @see in_expected_delay() to check if currently in expected delay
     * @see ExpectDelay RAII helper class
     */
    virtual void     expect_delay_ms(uint32_t ms) { }

    /**
     * @brief Check if currently in a period of expected delay
     * 
     * @return true if an expected delay is active, false otherwise
     * 
     * @details Returns true if we are in a period of expected delay set by
     *          expect_delay_ms(). This can be used to suppress error messages,
     *          disable watchdogs, or adjust timeout behavior during known long
     *          operations.
     * 
     * @note Default implementation always returns false
     * 
     * @see expect_delay_ms() to set expected delay
     */
    virtual bool     in_expected_delay(void) const { return false; }

    /**
     * @brief End the priority boost from delay_microseconds_boost()
     * 
     * @details Manually ends the priority boost granted by delay_microseconds_boost().
     *          Normally the boost expires automatically, but this can be called to
     *          release the boost early if the main loop is exiting prematurely.
     * 
     * @note Default implementation does nothing (no-op)
     * @note Only needed on platforms with priority boost support
     * 
     * @see delay_microseconds_boost() for priority-boosted delays
     */
    virtual void     boost_end(void) {}

    /**
     * @brief Register callback to be called during long delays
     * 
     * @param[in] proc Function pointer to callback (AP_HAL::Proc signature: void (*)(void))
     * @param[in] min_time_ms Minimum delay duration in milliseconds to trigger callback
     * 
     * @details Registers a function to be called by the scheduler when it needs to sleep
     *          for more than min_time_ms milliseconds. This allows background tasks to
     *          execute during long delays, such as:
     *          - Sensor calibration progress updates
     *          - Telemetry transmission during initialization
     *          - Watchdog feeding during long operations
     *          
     *          The callback is invoked from the delay() implementation when the delay
     *          duration exceeds the threshold. Only one callback can be registered at a time.
     * 
     * @note Callback executes in the context of the thread that called delay()
     * @note Callback should be short and non-blocking
     * @warning Callback must not call delay() or delay_microseconds() (would recurse)
     * 
     * @see in_delay_callback() to check if currently executing callback
     * @see call_delay_cb() internal method that invokes the callback
     */
    virtual void     register_delay_callback(AP_HAL::Proc,
                                             uint16_t min_time_ms);
    
    /**
     * @brief Check if currently executing delay callback
     * 
     * @return true if the delay callback is currently executing, false otherwise
     * 
     * @details Returns true if the Scheduler has called the delay callback function.
     *          If you are on the main thread and this returns true, that means your
     *          call stack has the scheduler delay mechanism somewhere in the stack.
     *          
     *          Useful for:
     *          - Detecting recursive delay attempts
     *          - Adjusting behavior during calibration or initialization
     *          - Debugging unexpected call contexts
     * 
     * @see register_delay_callback() to register the callback
     */
    virtual bool     in_delay_callback() const { return _in_delay_callback; }

    /**
     * @brief Register high-priority timer task for periodic execution
     * 
     * @param[in] proc Member function pointer to execute periodically
     * 
     * @details Registers a task to run at high frequency, typically 1kHz. Timer tasks
     *          execute in a dedicated timer thread or from timer interrupt context with
     *          higher priority than main thread and I/O tasks.
     *          
     *          Typical use cases:
     *          - High-frequency sensor sampling (IMU at 1kHz+)
     *          - PWM output generation
     *          - Time-critical state updates
     *          
     *          Timer process context:
     *          - Executes at ~1kHz rate (platform-dependent)
     *          - Higher priority than main loop and I/O tasks
     *          - Should complete in <100μs to avoid overruns
     * 
     * @note Multiple timer processes can be registered (implementation-specific limits)
     * @warning Keep timer tasks short (<100μs) to avoid blocking other timer tasks
     * @warning Do not call delay functions from timer tasks
     * @warning Be careful with shared data - use semaphores for thread safety
     * 
     * @see register_io_process() for low-priority background tasks
     */
    virtual void     register_timer_process(AP_HAL::MemberProc) = 0;

    /**
     * @brief Register low-priority I/O task for background execution
     * 
     * @param[in] proc Member function pointer to execute periodically
     * 
     * @details Registers a task to run at lower priority than the main thread, typically
     *          for background I/O operations that should not block flight control.
     *          
     *          Typical use cases:
     *          - Sensor polling and data collection
     *          - CAN bus message processing
     *          - Log writing to SD card
     *          - Telemetry transmission
     *          
     *          I/O process context:
     *          - Executes at platform-specific rate (often 50-400Hz)
     *          - Lower priority than main loop
     *          - Can take longer than timer tasks (up to several milliseconds)
     * 
     * @note Multiple I/O processes can be registered (implementation-specific limits)
     * @note I/O tasks can call delay functions if needed
     * @warning Use semaphores when accessing data shared with main thread
     * 
     * @see register_timer_process() for high-priority timer tasks
     */
    virtual void     register_io_process(AP_HAL::MemberProc) = 0;

    /**
     * @brief Register periodic failsafe callback
     * 
     * @param[in] proc Function pointer to failsafe callback
     * @param[in] period_us Execution period in microseconds
     * 
     * @details Registers a failsafe function that executes periodically at the specified
     *          interval. Failsafe callbacks are typically used for critical safety checks
     *          that must execute reliably even if the main loop hangs.
     *          
     *          Typical use cases:
     *          - Watchdog feeding
     *          - Critical sensor health monitoring
     *          - Communication link timeout detection
     * 
     * @note Executes in timer/interrupt context - keep very short
     * @warning Failsafe callbacks must be thread-safe and re-entrant
     * @warning Do not call delay functions from failsafe callbacks
     */
    virtual void     register_timer_failsafe(AP_HAL::Proc,
                                             uint32_t period_us) = 0;

    /**
     * @brief Mark the system as fully initialized
     * 
     * @details Called after all subsystems have completed initialization to signal
     *          that the system is ready for normal operation. This allows various
     *          components to defer certain operations until the system is fully up,
     *          such as:
     *          - Enabling arming checks
     *          - Starting mission execution
     *          - Activating full telemetry
     *          
     *          Boot sequence coordination:
     *          1. HAL initialization
     *          2. Library initialization
     *          3. Vehicle initialization
     *          4. set_system_initialized() called
     *          5. Normal operation begins
     * 
     * @note Should be called exactly once during boot sequence
     * 
     * @see is_system_initialized() to check initialization status
     */
    virtual void     set_system_initialized() = 0;
    
    /**
     * @brief Check if system initialization is complete
     * 
     * @return true if set_system_initialized() has been called, false otherwise
     * 
     * @details Returns whether the system has completed its initialization sequence.
     *          Components can check this to determine if they should defer operations
     *          that require a fully initialized system.
     * 
     * @see set_system_initialized() to mark initialization complete
     */
    virtual bool     is_system_initialized() = 0;

    /**
     * @brief Reboot the system
     * 
     * @param[in] hold_in_bootloader If true, reboot into bootloader for firmware update;
     *                               if false, normal reboot into application
     * 
     * @details Performs a clean system shutdown and reboot. Attempts to flush logs,
     *          close files, and safely shutdown hardware before triggering reset.
     *          
     *          Clean shutdown sequence:
     *          1. Stop scheduled tasks
     *          2. Flush logs and close files
     *          3. Disarm motors and disable outputs
     *          4. Trigger hardware reset
     *          
     *          If hold_in_bootloader is true, the system reboots into the bootloader
     *          instead of the main application, allowing firmware updates via serial
     *          or USB connection.
     * 
     * @note This function does not return - system resets
     * @warning Ensure motors are disarmed before calling in flight-capable systems
     * 
     * @see AP_HAL::Util::reboot_device() for alternative reboot methods
     */
    virtual void     reboot(bool hold_in_bootloader = false) = 0;

    /**
     * @brief Stop system clock at specified time (for log replay)
     * 
     * @param[in] time_usec Timestamp in microseconds to freeze clock
     * 
     * @details Optional function to stop the system clock at a given time, used by
     *          the log replay system (Tools/Replay) for deterministic replay of
     *          recorded logs. When the system time reaches time_usec, the clock
     *          stops advancing, allowing inspection of system state at that moment.
     *          
     *          Used for:
     *          - Debugging EKF behavior from logs
     *          - Replaying specific scenarios
     *          - Algorithm development and testing
     * 
     * @note Default implementation does nothing (no-op)
     * @note Only functional in SITL and replay builds
     * 
     * @see Tools/Replay for log replay system documentation
     */
    virtual void     stop_clock(uint64_t time_usec) {}

    /**
     * @brief Check if currently executing in the main thread
     * 
     * @return true if called from main thread, false if from background thread
     * 
     * @details Returns whether the current execution context is the main thread.
     *          Used for:
     *          - Thread-safety assertions (ensuring main-thread-only access)
     *          - Debugging unexpected thread contexts
     *          - Conditional locking (skip lock if already in main thread)
     *          
     *          Main thread:
     *          - Runs the vehicle scheduler and main loop
     *          - Executes flight control and sensor fusion
     *          - Typically 50-400Hz depending on vehicle type
     * 
     * @note Essential for WITH_SEMAPHORE macro thread-safety checks
     * 
     * @see in_delay_callback() to detect delay callback context
     */
    virtual bool     in_main_thread() const = 0;

    /**
     * @brief Disable interrupts and save interrupt state for critical section
     * 
     * @return Opaque pointer to saved interrupt state context, or nullptr if not supported
     * 
     * @details Disables interrupts and returns a context that can be used to restore
     *          the interrupt state later with restore_interrupts(). This can be used
     *          to protect critical regions that must execute atomically without
     *          interrupt preemption.
     *          
     *          Typical usage pattern:
     *          ```cpp
     *          void* state = hal.scheduler->disable_interrupts_save();
     *          // Critical section - interrupts disabled
     *          critical_operation();
     *          hal.scheduler->restore_interrupts(state);
     *          ```
     *          
     *          Critical sections should be:
     *          - Very short (< 10μs preferred)
     *          - Non-blocking (no waiting or delays)
     *          - Used sparingly (impacts real-time performance)
     * 
     * @note Default implementation returns nullptr (no interrupt control)
     * @note May not be implemented on all HAL platforms
     * @warning Keep critical sections as short as possible
     * @warning Do not call delay functions with interrupts disabled
     * @warning Always pair with restore_interrupts() - use RAII if possible
     * 
     * @see restore_interrupts() to re-enable interrupts
     */
    virtual void *disable_interrupts_save(void) { return nullptr; }

    /**
     * @brief Restore interrupt state from disable_interrupts_save()
     * 
     * @param[in] state Opaque pointer returned by disable_interrupts_save()
     * 
     * @details Restores the interrupt state previously saved by disable_interrupts_save().
     *          This re-enables interrupts (if they were enabled before) and ends the
     *          critical section.
     * 
     * @note Default implementation does nothing (no-op)
     * @note Must be paired with disable_interrupts_save()
     * 
     * @see disable_interrupts_save() to disable interrupts and get state
     */
    virtual void restore_interrupts(void *) {}

    /**
     * @brief Internal method called by subclasses during delays
     * 
     * @details Called by platform-specific implementations when they need to execute
     *          the registered delay callback during a delay operation. This method
     *          checks if a callback is registered and if the delay duration exceeds
     *          the minimum threshold, then invokes the callback.
     *          
     *          Sets _in_delay_callback flag during callback execution to allow
     *          detection of callback context by in_delay_callback().
     * 
     * @note For internal use by HAL implementations only
     * 
     * @see register_delay_callback() for callback registration
     * @see in_delay_callback() to check if callback is executing
     */
    virtual void call_delay_cb();
    
    /**
     * @brief Minimum delay duration in milliseconds to trigger delay callback
     * 
     * @details Threshold set by register_delay_callback(). The delay callback is
     *          only invoked if the delay duration meets or exceeds this value.
     */
    uint16_t _min_delay_cb_ms;

    /**
     * @enum priority_base
     * @brief Base priority levels for thread creation
     * 
     * @details Defines priority reference points for creating new threads. The actual
     *          thread priority is calculated relative to these base priorities using
     *          the priority offset parameter in thread_create().
     *          
     *          Priority hierarchy (highest to lowest):
     *          - PRIORITY_BOOST: Temporary main thread boost for loop start
     *          - PRIORITY_MAIN: Main control loop thread
     *          - PRIORITY_TIMER: Timer-driven high-frequency tasks
     *          - PRIORITY_SPI: SPI bus I/O operations
     *          - PRIORITY_I2C: I2C bus I/O operations  
     *          - PRIORITY_CAN: CAN bus communication
     *          - PRIORITY_RCOUT: RC servo/motor output generation
     *          - PRIORITY_RCIN: RC receiver input processing
     *          - PRIORITY_UART: Serial port I/O
     *          - PRIORITY_IO: General background I/O tasks
     *          - PRIORITY_STORAGE: SD card and flash storage operations
     *          - PRIORITY_LED: LED and notification updates
     *          - PRIORITY_SCRIPTING: Lua scripting execution
     *          - PRIORITY_NET: Network communication (WiFi, Ethernet)
     *          
     *          Platform-specific mappings:
     *          - ChibiOS: Maps to ChibiOS thread priorities
     *          - Linux: Maps to nice values or real-time priorities
     *          - ESP32: Maps to FreeRTOS task priorities
     * 
     * @note Exact priority values are platform-specific
     * @warning Incorrect priority assignment can cause starvation or timing issues
     */
    enum priority_base {
        PRIORITY_BOOST,      ///< Temporary boost for main thread (highest)
        PRIORITY_MAIN,       ///< Main control loop thread
        PRIORITY_SPI,        ///< SPI bus I/O operations
        PRIORITY_I2C,        ///< I2C bus I/O operations
        PRIORITY_CAN,        ///< CAN bus communication
        PRIORITY_TIMER,      ///< Timer-driven tasks
        PRIORITY_RCOUT,      ///< RC servo/motor output
        PRIORITY_LED,        ///< LED and notifications
        PRIORITY_RCIN,       ///< RC receiver input
        PRIORITY_IO,         ///< General I/O operations
        PRIORITY_UART,       ///< Serial port I/O
        PRIORITY_STORAGE,    ///< Storage operations (SD, flash)
        PRIORITY_SCRIPTING,  ///< Lua scripting execution
        PRIORITY_NET,        ///< Network communication (lowest)
    };
    
    /**
     * @brief Create a new thread for background task execution
     * 
     * @param[in] proc        Member function pointer to execute in thread
     * @param[in] name        Thread name for debugging (null-terminated string)
     * @param[in] stack_size  Stack size in bytes for thread
     * @param[in] base        Base priority level (from priority_base enum)
     * @param[in] priority    Priority offset relative to base (-128 to +127)
     * 
     * @return true if thread created successfully, false on failure
     * 
     * @details Creates a new background thread that executes the specified member
     *          function continuously. The thread runs independently of the main loop
     *          and scheduler, managed by the underlying RTOS or threading library.
     *          
     *          Thread creation workflow:
     *          1. Allocate stack of specified size
     *          2. Calculate actual priority from base + offset
     *          3. Initialize thread context
     *          4. Start thread execution of proc()
     *          
     *          Priority calculation:
     *          - final_priority = platform_priority(base) + priority_offset
     *          - Positive offset increases priority (higher priority)
     *          - Negative offset decreases priority (lower priority)
     *          
     *          Common stack sizes:
     *          - Simple I/O threads: 2048-4096 bytes
     *          - Protocol handlers: 4096-8192 bytes
     *          - Complex processing: 8192-16384 bytes
     *          
     *          Thread function pattern:
     *          ```cpp
     *          void MyClass::thread_func() {
     *              while (true) {
     *                  // Do work
     *                  hal.scheduler->delay(10); // Yield periodically
     *              }
     *          }
     *          ```
     * 
     * @note Default implementation returns false (thread creation not supported)
     * @note Thread function should loop forever or exit cleanly
     * @note Thread must yield periodically with delay() to avoid starving other threads
     * @warning Insufficient stack_size causes stack overflow crashes
     * @warning Too many threads can exhaust memory and CPU resources
     * @warning Use WITH_SEMAPHORE for thread-safe access to shared data
     * 
     * @see priority_base enum for priority level definitions
     */
    virtual bool thread_create(AP_HAL::MemberProc proc, const char *name,
                               uint32_t stack_size, priority_base base, int8_t priority) {
        return false;
    }

private:

    AP_HAL::Proc _delay_cb;           ///< Registered delay callback function pointer
    bool _in_delay_callback : 1;      ///< Flag indicating if delay callback is currently executing

};

/**
 * @class ExpectDelay
 * @brief RAII helper for automatic expect_delay_ms() management
 * 
 * @details Provides scope-based management of expected delay notifications to the
 *          scheduler. Constructor calls expect_delay_ms() with the specified duration,
 *          and destructor automatically cancels the expected delay when the object
 *          goes out of scope.
 *          
 *          This ensures that expected delays are properly cancelled even if the
 *          function exits early via return or exception.
 *          
 *          Usage pattern:
 *          ```cpp
 *          void long_operation() {
 *              ExpectDelay delay(5000);  // Expect 5 second delay
 *              // Perform long operation
 *              flash_chip_erase();
 *              // Delay automatically cancelled when delay object destroyed
 *          }
 *          ```
 *          
 *          Or use the EXPECT_DELAY_MS() macro:
 *          ```cpp
 *          void long_operation() {
 *              EXPECT_DELAY_MS(5000);
 *              flash_chip_erase();
 *          }
 *          ```
 * 
 * @note Preferred over manual expect_delay_ms()/expect_delay_ms(0) pairs
 * 
 * @see EXPECT_DELAY_MS() macro for convenient usage
 * @see AP_HAL::Scheduler::expect_delay_ms() for underlying functionality
 */
class ExpectDelay {
public:
    /**
     * @brief Constructor - notify scheduler of expected delay
     * 
     * @param[in] ms Expected delay duration in milliseconds
     * 
     * @details Calls hal.scheduler->expect_delay_ms(ms) to notify the scheduler
     *          that a long operation is starting.
     */
    ExpectDelay(uint32_t ms);
    
    /**
     * @brief Destructor - cancel expected delay notification
     * 
     * @details Calls hal.scheduler->expect_delay_ms(0) to cancel the expected
     *          delay notification, indicating the operation completed.
     */
    ~ExpectDelay();
};

/**
 * @def EXPECT_DELAY_MS(ms)
 * @brief Macro to create scope-based expected delay notification
 * 
 * @param ms Expected delay duration in milliseconds
 * 
 * @details Creates an ExpectDelay RAII object with a unique name that automatically
 *          manages expected delay notifications. The delay is expected for the
 *          current scope and automatically cancelled when scope exits.
 *          
 *          Example usage:
 *          ```cpp
 *          void calibrate_compass() {
 *              EXPECT_DELAY_MS(30000);  // Expect 30 second calibration
 *              perform_compass_calibration();
 *              // Delay notification automatically cancelled at end of scope
 *          }
 *          ```
 * 
 * @note Uses __COUNTER__ to generate unique variable names
 * 
 * @see ExpectDelay class for implementation details
 */
#define EXPECT_DELAY_MS(ms) DELAY_JOIN( ms, __COUNTER__ )
#define DELAY_JOIN( ms, counter) _DO_DELAY_JOIN( ms, counter )
#define _DO_DELAY_JOIN( ms, counter ) ExpectDelay _getdelay ## counter(ms)


/**
 * @class TimeCheck
 * @brief RAII helper to detect and log unexpectedly long delays
 * 
 * @details Measures execution time of a code scope and prints a warning if execution
 *          exceeds the specified limit. Useful for detecting performance regressions
 *          and unexpected delays in time-critical code paths.
 *          
 *          Constructor records start time, destructor measures elapsed time and
 *          logs warning if limit exceeded, including file and line number.
 *          
 *          Usage pattern:
 *          ```cpp
 *          void critical_function() {
 *              TIME_CHECK(10);  // Warn if takes >10ms
 *              perform_critical_operation();
 *          }
 *          ```
 *          
 *          If the operation takes longer than 10ms, prints:
 *          "TimeCheck: critical_function.cpp:123 took XXms (limit 10ms)"
 * 
 * @note Overhead is minimal (~2 micros) - safe to use in performance-critical code
 * @warning Does not prevent long delays, only detects and reports them
 * 
 * @see TIME_CHECK() macro for convenient usage
 */
class TimeCheck {
public:
    /**
     * @brief Constructor - record start time
     * 
     * @param[in] limit_ms Maximum acceptable execution time in milliseconds
     * @param[in] file     Source file name from __FILE__
     * @param[in] line     Source line number from __LINE__
     * 
     * @details Records the current time as the start of the measured scope.
     */
    TimeCheck(uint32_t limit_ms, const char *file, uint32_t line);
    
    /**
     * @brief Destructor - check elapsed time and log if limit exceeded
     * 
     * @details Measures elapsed time since construction. If elapsed time exceeds
     *          limit_ms, logs a warning message with file, line, actual time, and limit.
     */
    ~TimeCheck();
    
private:
    const uint32_t limit_ms;   ///< Maximum acceptable duration in milliseconds
    const uint32_t line;       ///< Source code line number
    const char *file;          ///< Source code file name
    uint32_t start_ms;         ///< Recorded start time in milliseconds
};

/**
 * @def TIME_CHECK(limit_ms)
 * @brief Macro to detect unexpected long delays in code scope
 * 
 * @param limit_ms Maximum acceptable execution time in milliseconds
 * 
 * @details Creates a TimeCheck RAII object that measures execution time of the
 *          current scope. If execution time exceeds limit_ms, logs a warning with
 *          file and line information.
 *          
 *          Scatter TIME_CHECK() macros in performance-critical code to detect
 *          unexpected delays and performance regressions during development and testing.
 *          
 *          Example usage:
 *          ```cpp
 *          void fast_loop() {
 *              TIME_CHECK(5);  // Warn if loop iteration >5ms
 *              
 *              read_sensors();
 *              update_attitude();
 *              calculate_outputs();
 *          }
 *          ```
 * 
 * @note Uses __FILE__, __LINE__, and __COUNTER__ for automatic context capture
 * @note Minimal overhead - suitable for high-frequency loops
 * 
 * @see TimeCheck class for implementation details
 */
#define TIME_CHECK(limit_ms) JOIN_TC(limit_ms, __FILE__, __LINE__, __COUNTER__ )
#define JOIN_TC(limit_ms, file, line, counter ) _DO_JOIN_TC( limit_ms, file, line, counter )
#define _DO_JOIN_TC(limit_ms, file, line, counter ) TimeCheck _gettc ## counter(limit_ms, file, line)
