/**
 * @file Semaphores.h
 * @brief Synchronization primitives for thread-safe data access in ArduPilot
 * 
 * @details This file defines mutex and binary semaphore interfaces for protecting shared 
 *          data structures across multiple threads in the ArduPilot autopilot system. 
 *          ArduPilot uses semaphores extensively to coordinate:
 *          - Main loop execution with I/O threads
 *          - Interrupt handlers with background processing
 *          - Multiple threads accessing sensors, buses, and navigation state
 *          
 *          Two synchronization primitives are provided:
 *          - Semaphore: Recursive mutex for mutual exclusion of shared resources
 *          - BinarySemaphore: Signaling mechanism for thread coordination
 *          
 *          Common protected resources in ArduPilot:
 *          - SPI/I2C buses: Prevent simultaneous hardware access
 *          - Sensor data: Coordinate producer/consumer threads
 *          - Navigation state: Protect EKF updates from concurrent reads
 *          - Parameter storage: Serialize parameter modifications
 *          - Logging buffers: Coordinate log writers
 * 
 * @note All HAL implementations must provide concrete implementations of these interfaces
 * @warning Improper semaphore usage can cause deadlocks or race conditions in flight code
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_HAL_Namespace.h"

#include <AP_Common/AP_Common.h>

/**
 * @def HAL_SEMAPHORE_BLOCK_FOREVER
 * @brief Timeout value for infinite blocking on semaphore acquisition
 * 
 * @details Pass this value to take() or wait() to block indefinitely until the
 *          semaphore becomes available. Use with caution as it can cause deadlocks
 *          if the semaphore is never released.
 */
#define HAL_SEMAPHORE_BLOCK_FOREVER 0

/**
 * @class AP_HAL::Semaphore
 * @brief Recursive mutex for mutual exclusion of shared resources
 * 
 * @details Provides lock/unlock operations to protect critical sections from concurrent access
 *          by multiple threads. Semaphore implements a recursive mutex, meaning the same thread
 *          can acquire it multiple times and must release it the same number of times.
 *          
 *          **Typical usage pattern:**
 *          ```cpp
 *          if (semaphore->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
 *              // Critical section - exclusive access guaranteed
 *              modify_shared_data();
 *              semaphore->give();
 *          }
 *          ```
 *          
 *          **Preferred RAII pattern using WITH_SEMAPHORE macro:**
 *          ```cpp
 *          WITH_SEMAPHORE(semaphore);
 *          // Automatic unlock on scope exit - prevents forget-to-unlock bugs
 *          modify_shared_data();
 *          ```
 *          
 *          **Locking semantics:**
 *          - Recursive: Same thread can take semaphore multiple times (must give same count)
 *          - Priority inheritance: Implemented to prevent priority inversion on RTOS platforms
 *          - Timeout support: Configurable wait time prevents indefinite blocking
 *          - Thread ownership: Tracks which thread holds the semaphore for debugging
 *          
 *          **Common use cases in ArduPilot:**
 *          - SPI/I2C bus access: Prevent simultaneous hardware transactions from multiple drivers
 *          - Sensor data structures: Coordinate sensor driver updates with consumer reads
 *          - Navigation state: Protect EKF state updates from concurrent GCS telemetry reads
 *          - Parameter modifications: Serialize parameter storage operations
 *          - Logger access: Coordinate multiple threads writing to logging buffers
 *          
 *          **Performance characteristics:**
 *          - Lock/unlock overhead: Typically 1-10 microseconds depending on platform
 *          - Called at: Main loop rate (100-1000Hz), I/O thread rate, timer callbacks
 *          - Blocking behavior: May cause thread to yield CPU while waiting
 * 
 * @note Semaphore operations may block the calling thread - never call from interrupt handlers
 * @note All ArduPilot semaphores are recursive to simplify nested function calls
 * @warning Always unlock semaphore in same thread that locked it
 * @warning Forgetting to unlock causes deadlock - prefer WITH_SEMAPHORE macro for RAII
 * @warning Taking multiple semaphores in different orders can cause deadlock
 * 
 * @see WithSemaphore
 * @see WITH_SEMAPHORE
 * @see BinarySemaphore
 */
class AP_HAL::Semaphore {
public:

    /**
     * @brief Default constructor
     * 
     * @details Creates semaphore in unlocked state, ready for use.
     *          Platform-specific implementation performs actual initialization.
     */
    Semaphore() {}

    // do not allow copying
    CLASS_NO_COPY(Semaphore);

    /**
     * @brief Attempt to acquire semaphore with optional timeout
     * 
     * @details Tries to lock the semaphore, blocking the calling thread if already locked
     *          by another thread. If the calling thread already holds the semaphore (recursive
     *          lock), the internal count is incremented and the call succeeds immediately.
     *          
     *          Timeout behavior:
     *          - HAL_SEMAPHORE_BLOCK_FOREVER (0): Wait indefinitely until available
     *          - Non-zero value: Wait up to timeout_ms milliseconds
     *          - Returns false if timeout expires before semaphore acquired
     *          
     *          This method is the primary locking mechanism for most use cases. However,
     *          prefer the WITH_SEMAPHORE macro to ensure automatic unlock on scope exit.
     * 
     * @param[in] timeout_ms Maximum wait time in milliseconds. Use HAL_SEMAPHORE_BLOCK_FOREVER
     *                       for infinite wait, or 0 for non-blocking try-lock.
     * 
     * @return true if semaphore successfully acquired, false if timeout expired
     * 
     * @note Blocks calling thread until semaphore available or timeout expires
     * @note Recursive: Same thread can call take() multiple times (must call give() same count)
     * @note Thread may yield CPU while waiting, allowing other threads to run
     * @warning Never call from interrupt context - use BinarySemaphore for ISR signaling
     * @warning Result must be checked - WARN_IF_UNUSED attribute enforces this
     * 
     * @see take_nonblocking()
     * @see take_blocking()
     * @see give()
     */
    virtual bool take(uint32_t timeout_ms) WARN_IF_UNUSED = 0 ;
    
    /**
     * @brief Try to acquire semaphore without blocking
     * 
     * @details Attempts to lock the semaphore immediately, returning false if already locked
     *          by another thread. If the calling thread already holds the semaphore, increments
     *          the recursive count and returns true immediately.
     *          
     *          This method is useful in time-critical code paths that cannot afford to block,
     *          such as high-rate control loops or timer callbacks with tight deadlines.
     * 
     * @return true if semaphore immediately acquired, false if already locked by another thread
     * 
     * @note Equivalent to take(0) but clearer intent
     * @note Never blocks - returns immediately
     * @note Useful in control loops that must maintain consistent timing
     * @warning Result must be checked - data access without lock causes race conditions
     * 
     * @see take()
     */
    virtual bool take_nonblocking() WARN_IF_UNUSED = 0;

    /**
     * @brief Acquire semaphore, blocking indefinitely until available
     * 
     * @details Convenience wrapper for take(HAL_SEMAPHORE_BLOCK_FOREVER) that blocks forever
     *          waiting for the semaphore. This method does not return a value, so it should
     *          only be used when you are certain the semaphore will eventually be released.
     *          
     *          The compiler warning about unused results is suppressed for this method since
     *          it always succeeds (eventually).
     *          
     *          Prefer WITH_SEMAPHORE macro over this method for automatic unlock on scope exit.
     * 
     * @note Blocks indefinitely - use with caution as it can cause deadlock
     * @note Used primarily by WITH_SEMAPHORE macro implementation
     * @warning Can deadlock if semaphore is never released by holder
     * @warning No timeout protection - may block forever if deadlock occurs
     * 
     * @see take()
     * @see WITH_SEMAPHORE
     */
    // a variant that blocks forever
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-result"
    virtual void take_blocking() { take(HAL_SEMAPHORE_BLOCK_FOREVER); };
    #pragma GCC diagnostic pop
    
    /**
     * @brief Release semaphore, allowing other threads to acquire it
     * 
     * @details Decrements the recursive lock count. If count reaches zero, the semaphore
     *          is released and another waiting thread may acquire it. If multiple threads
     *          are waiting, priority inheritance rules determine which thread acquires next.
     *          
     *          Must be called the same number of times as take() was successfully called
     *          by this thread to fully release the semaphore.
     * 
     * @return true if successfully released (always true in current implementations)
     * 
     * @note Must be called from same thread that acquired semaphore
     * @note For recursive locks, must call give() once for each successful take()
     * @note Releases highest-priority waiting thread (if any) when fully unlocked
     * @warning Calling give() without prior successful take() is undefined behavior
     * @warning May cause immediate context switch if higher-priority thread was waiting
     * 
     * @see take()
     */
    virtual bool give() = 0;
    
    /**
     * @brief Virtual destructor for polymorphic deletion
     * 
     * @details Allows semaphore objects to be deleted through base class pointer.
     *          Platform implementations should ensure proper cleanup of OS primitives.
     */
    virtual ~Semaphore(void) {}
};

/**
 * @class WithSemaphore
 * @brief RAII helper class for automatic semaphore unlock on scope exit
 * 
 * @details Implements Resource Acquisition Is Initialization (RAII) pattern for semaphore
 *          management. Constructor acquires the semaphore (blocking forever), destructor
 *          automatically releases it when object goes out of scope. This prevents common
 *          bugs where semaphores are forgotten or not released on error paths.
 *          
 *          **Typical usage via WITH_SEMAPHORE macro (preferred):**
 *          ```cpp
 *          void update_sensor_data() {
 *              WITH_SEMAPHORE(sensor_semaphore);
 *              // Semaphore automatically acquired here
 *              
 *              process_sensor_reading();
 *              
 *              if (error_detected) {
 *                  return;  // Semaphore automatically released here
 *              }
 *              
 *              store_result();
 *              // Semaphore automatically released at end of scope
 *          }
 *          ```
 *          
 *          **Benefits over manual lock/unlock:**
 *          - Exception safe: Automatic unlock even if exception thrown
 *          - Early return safe: Automatic unlock on all exit paths
 *          - No forgot-to-unlock bugs: Compiler guarantees cleanup
 *          - Scope-based locking: Clear critical section boundaries
 *          
 *          **Implementation details:**
 *          - Constructor calls semaphore->take_blocking() (infinite wait)
 *          - Destructor calls semaphore->give()
 *          - Non-copyable to prevent double-unlock bugs
 *          - Line number tracking for debugging deadlocks
 *          
 *          **Limitations:**
 *          - Always blocks forever (no timeout support)
 *          - Cannot be used with binary semaphores (use signal/wait instead)
 *          - Must be assigned to a variable (macro generates unique name)
 * 
 * @note All ArduPilot semaphores are recursive - safe to nest WITH_SEMAPHORE in same thread
 * @note Line number parameter is automatically provided by WITH_SEMAPHORE macro for debugging
 * @warning Only works with Semaphore, not BinarySemaphore
 * @warning Always blocks forever - use manual take() if timeout needed
 * 
 * @see WITH_SEMAPHORE
 * @see Semaphore
 */
class WithSemaphore {
public:
    /**
     * @brief Construct and acquire semaphore from pointer
     * 
     * @param[in] mtx Pointer to semaphore to acquire (blocks forever if locked)
     * @param[in] line Source line number for debugging (provided by macro)
     * 
     * @note Blocks until semaphore is acquired
     */
    WithSemaphore(AP_HAL::Semaphore *mtx, uint32_t line);
    
    /**
     * @brief Construct and acquire semaphore from reference
     * 
     * @param[in] mtx Reference to semaphore to acquire (blocks forever if locked)
     * @param[in] line Source line number for debugging (provided by macro)
     * 
     * @note Blocks until semaphore is acquired
     */
    WithSemaphore(AP_HAL::Semaphore &mtx, uint32_t line);

    /**
     * @brief Destructor automatically releases semaphore
     * 
     * @details Called when WithSemaphore object goes out of scope, ensuring semaphore
     *          is always released even on early returns or exceptions.
     */
    ~WithSemaphore();
    
private:
    AP_HAL::Semaphore &_mtx;  ///< Reference to semaphore being managed
};

/**
 * @def WITH_SEMAPHORE(sem)
 * @brief Macro for RAII-style automatic semaphore management
 * 
 * @details Creates a WithSemaphore object with a unique name that acquires the specified
 *          semaphore and automatically releases it when the current scope exits. This is
 *          the preferred method for semaphore locking in ArduPilot as it prevents common
 *          bugs where semaphores are not released on error paths or early returns.
 *          
 *          **Usage:**
 *          ```cpp
 *          void update_navigation() {
 *              WITH_SEMAPHORE(ahrs_semaphore);
 *              // Semaphore acquired here, released at end of scope
 *              
 *              read_ahrs_state();
 *              compute_navigation();
 *              update_targets();
 *              
 *              // Automatic unlock here on scope exit
 *          }
 *          ```
 *          
 *          **Multiple semaphores:**
 *          ```cpp
 *          void update_with_multiple_locks() {
 *              WITH_SEMAPHORE(sensor_sem);  // Acquire first semaphore
 *              read_sensor_data();
 *              
 *              {
 *                  WITH_SEMAPHORE(logger_sem);  // Acquire second semaphore
 *                  log_sensor_data();
 *                  // logger_sem released here
 *              }
 *              
 *              process_data();
 *              // sensor_sem released here
 *          }
 *          ```
 *          
 *          **Implementation notes:**
 *          - Expands to WithSemaphore object with unique name (uses __COUNTER__)
 *          - Captures line number for deadlock debugging
 *          - Multiple levels of macro indirection ensure proper token expansion
 *          - Name uniqueness allows multiple WITH_SEMAPHORE in same scope
 * 
 * @param[in] sem Pointer or reference to AP_HAL::Semaphore to lock
 * 
 * @note Blocks forever waiting for semaphore - no timeout support
 * @note Safe to nest in same thread (semaphores are recursive)
 * @warning Cannot be used with BinarySemaphore - use signal/wait directly
 * @warning Always blocks indefinitely - if timeout needed, use manual take()
 * 
 * @see WithSemaphore
 * @see Semaphore::take()
 * 
 * Source: Macro pattern from https://stackoverflow.com/questions/19666142/why-is-a-level-of-indirection-needed-for-this-concatenation-macro
 */
#define WITH_SEMAPHORE( sem ) JOIN( sem, __AP_LINE__, __COUNTER__ )

/**
 * @def JOIN(sem, line, counter)
 * @brief Helper macro for WITH_SEMAPHORE - first level of token expansion
 * 
 * @details Intermediate macro that passes tokens to _DO_JOIN for final expansion.
 *          Level of indirection is necessary for proper preprocessor token concatenation.
 */
#define JOIN( sem, line, counter ) _DO_JOIN( sem, line, counter )

/**
 * @def _DO_JOIN(sem, line, counter)
 * @brief Helper macro for WITH_SEMAPHORE - performs actual token concatenation
 * 
 * @details Generates unique WithSemaphore variable name by concatenating _getsem with
 *          counter value. This allows multiple WITH_SEMAPHORE instances in same scope.
 */
#define _DO_JOIN( sem, line, counter ) WithSemaphore _getsem ## counter(sem, line)

/**
 * @class AP_HAL::BinarySemaphore
 * @brief Binary semaphore for event signaling between threads
 * 
 * @details Binary semaphores are used for producer-consumer signaling patterns, not mutual
 *          exclusion. Unlike Semaphore (mutex), BinarySemaphore has no ownership concept - any
 *          thread can signal it and any thread can wait for it. The semaphore has only two
 *          states: signaled (event occurred) or not signaled (no event).
 *          
 *          **Key differences from Semaphore:**
 *          - Purpose: Signaling between threads, not resource protection
 *          - Ownership: No ownership tracking - any thread can signal/wait
 *          - ISR-safe: signal_ISR() can be called from interrupt handlers
 *          - Non-recursive: No recursive locking concept
 *          - No WITH_SEMAPHORE: Cannot use WITH_SEMAPHORE macro (signals only)
 *          
 *          **Typical producer-consumer pattern:**
 *          ```cpp
 *          // Producer thread (or interrupt handler):
 *          void sensor_interrupt_handler() {
 *              read_sensor_data_to_buffer();
 *              data_ready_semaphore.signal_ISR();  // Wake consumer
 *          }
 *          
 *          // Consumer thread:
 *          void sensor_processing_thread() {
 *              while (true) {
 *                  if (data_ready_semaphore.wait(1000000)) {  // Wait up to 1 second
 *                      process_sensor_buffer();
 *                  } else {
 *                      handle_timeout();
 *                  }
 *              }
 *          }
 *          ```
 *          
 *          **Common use cases in ArduPilot:**
 *          - ISR to thread signaling: Interrupt signals data ready, thread processes it
 *          - Thread synchronization: One thread signals completion to another
 *          - Event notification: State change notifications between components
 *          - Periodic wakeup: Timer signals thread to perform periodic processing
 *          
 *          **State semantics:**
 *          - Signaled state persists until consumed by wait()
 *          - Multiple signals before wait() are coalesced (still one wait() needed)
 *          - wait() consumes the signal and resets to non-signaled state
 * 
 * @note Can be signaled from interrupt context using signal_ISR()
 * @note No ownership tracking - any thread can signal or wait
 * @note Cannot use WITH_SEMAPHORE macro - use explicit wait() calls
 * @warning Do not use for mutual exclusion - use Semaphore instead
 * @warning wait() timeout is in microseconds, not milliseconds
 * 
 * @see Semaphore
 */
class AP_HAL::BinarySemaphore {
public:
    /**
     * @brief Construct binary semaphore with specified initial state
     * 
     * @details Creates a binary semaphore in either signaled or non-signaled state.
     *          The initial state determines whether the first wait() call will block.
     *          
     *          Initial state semantics:
     *          - true: Semaphore starts signaled - first wait() will not block
     *          - false: Semaphore starts non-signaled - first wait() will block
     * 
     * @param[in] initial_state If true, semaphore starts signaled (wait() won't block).
     *                          If false, semaphore starts non-signaled (wait() will block).
     *                          Default is false (non-signaled).
     * 
     * @note Most common usage is default false (wait until first signal)
     */
    BinarySemaphore(bool initial_state=false) {}

    // do not allow copying
    CLASS_NO_COPY(BinarySemaphore);

    /**
     * @brief Wait for semaphore to be signaled with optional timeout
     * 
     * @details Blocks the calling thread until the semaphore is signaled by another thread
     *          or interrupt handler, or until the timeout expires. If the semaphore is already
     *          signaled when wait() is called, returns immediately and consumes the signal.
     *          
     *          Multiple signals before wait() are coalesced - only one wait() is needed to
     *          consume any number of prior signals. After wait() returns true, the semaphore
     *          is reset to non-signaled state.
     *          
     *          **Timeout behavior:**
     *          - 0: Non-blocking check (returns immediately)
     *          - Non-zero: Wait up to timeout_us microseconds
     *          - Returns false if timeout expires before signal received
     * 
     * @param[in] timeout_us Maximum wait time in microseconds. Use 0 for non-blocking check.
     *                       Common values: 1000 (1ms), 10000 (10ms), 1000000 (1 second).
     * 
     * @return true if semaphore was signaled, false if timeout expired
     * 
     * @note Timeout is in MICROSECONDS, not milliseconds (unlike Semaphore::take())
     * @note Consumes signal - resets semaphore to non-signaled state on success
     * @note Thread may yield CPU while waiting, allowing other threads to run
     * @warning Result must be checked - WARN_IF_UNUSED enforces this
     * @warning Do not call from interrupt context - only signal() is ISR-safe
     * 
     * @see signal()
     * @see wait_nonblocking()
     * @see wait_blocking()
     */
    virtual bool wait(uint32_t timeout_us) WARN_IF_UNUSED = 0 ;
    
    /**
     * @brief Wait for semaphore indefinitely until signaled
     * 
     * @details Blocks the calling thread forever until the semaphore is signaled.
     *          Use this method when you are certain a signal will eventually arrive,
     *          or when you want a thread to sleep indefinitely until woken by event.
     *          
     *          Typical usage: Worker thread waiting for tasks from producer.
     * 
     * @return true when semaphore is signaled (always succeeds eventually)
     * 
     * @note Blocks indefinitely - no timeout protection
     * @note Useful for worker threads waiting for events
     * @warning Can block forever if signal never arrives
     * @warning May cause deadlock if producer thread fails
     * 
     * @see wait()
     */
    virtual bool wait_blocking() = 0;
    
    /**
     * @brief Check if semaphore is signaled without blocking
     * 
     * @details Non-blocking check of semaphore state. Returns immediately with true if
     *          signaled, false if not signaled. Useful in tight loops or time-critical
     *          code that cannot afford to block.
     *          
     *          Equivalent to wait(0).
     * 
     * @return true if semaphore was signaled, false if not signaled
     * 
     * @note Never blocks - returns immediately
     * @note Consumes signal if available
     * @note Useful in polling loops with other work to do
     * 
     * @see wait()
     */
    virtual bool wait_nonblocking() { return wait(0); }

    /**
     * @brief Signal the semaphore, waking one waiting thread
     * 
     * @details Sets the semaphore to signaled state. If one or more threads are blocked
     *          in wait(), the highest-priority waiting thread is awakened. If no threads
     *          are waiting, the signal persists until consumed by a future wait().
     *          
     *          Multiple signal() calls before a wait() are coalesced - the semaphore remains
     *          in signaled state regardless of how many times signal() is called.
     * 
     * @note Can be called from any thread context
     * @note If multiple threads waiting, highest-priority thread wakes first
     * @note Signal persists until consumed by wait()
     * @note Multiple signals coalesce to single signaled state
     * @warning May cause immediate context switch if higher-priority thread waiting
     * 
     * @see signal_ISR()
     * @see wait()
     */
    virtual void signal() = 0;
    
    /**
     * @brief Signal the semaphore from interrupt context (ISR-safe)
     * 
     * @details ISR-safe version of signal() that can be called from interrupt handlers.
     *          Platform implementations must ensure this does not block or call non-ISR-safe
     *          OS primitives. Typically uses a lock-free mechanism or deferred signal.
     *          
     *          **Typical usage in interrupt handler:**
     *          ```cpp
     *          void __interrupt sensor_data_ready_isr() {
     *              read_sensor_register();
     *              data_ready_semaphore.signal_ISR();  // Wake processing thread
     *          }
     *          ```
     *          
     *          Default implementation simply calls signal(), but platform HALs may
     *          override with optimized ISR-safe implementations.
     * 
     * @note Safe to call from interrupt handlers
     * @note Must not block or call non-ISR-safe functions
     * @note Default implementation calls signal() - may not be optimal for all platforms
     * @warning Interrupt latency - keep ISR code minimal
     * @warning May cause context switch when interrupt returns
     * 
     * @see signal()
     */
    virtual void signal_ISR() { signal(); }
    
    /**
     * @brief Virtual destructor for polymorphic deletion
     * 
     * @details Allows binary semaphore objects to be deleted through base class pointer.
     *          Platform implementations should ensure proper cleanup of OS primitives.
     */
    virtual ~BinarySemaphore(void) {}
};
