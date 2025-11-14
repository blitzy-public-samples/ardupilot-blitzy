/**
 * @file Semaphores.h
 * @brief SITL (Software-In-The-Loop) semaphore implementation using POSIX pthread primitives
 * 
 * @details This file provides semaphore synchronization primitives for the SITL HAL platform.
 *          The implementation uses POSIX pthread mutexes and condition variables to provide
 *          cross-platform synchronization on Linux, macOS, and other POSIX-compatible systems
 *          where SITL simulations run.
 *          
 *          Two semaphore types are implemented:
 *          - Semaphore: Recursive mutex with owner tracking for thread-safe resource protection
 *          - BinarySemaphore: Condition variable-based signaling for thread coordination
 *          
 *          These implementations support the ArduPilot scheduler's multi-threaded task execution
 *          in simulation environments, providing equivalent synchronization behavior to the
 *          hardware RTOS implementations (ChibiOS, Linux, etc.).
 * 
 * @warning Timing behavior in SITL may differ from real-time embedded RTOS implementations.
 *          The SITL scheduler can advance simulation time faster than wall-clock time,
 *          which affects timeout calculations and scheduling behavior. Always test
 *          time-critical code on actual hardware.
 * 
 * @note For general semaphore usage patterns, see the WITH_SEMAPHORE macro in AP_HAL_Macros.h
 *       which provides RAII-style automatic acquisition and release of semaphores.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <stdint.h>
#include <AP_HAL/AP_HAL_Macros.h>
#include <AP_HAL/Semaphores.h>
#include "AP_HAL_SITL_Namespace.h"
#include <pthread.h>

/**
 * @class HALSITL::Semaphore
 * @brief Recursive mutex-based semaphore implementation for SITL using POSIX pthread_mutex_t
 * 
 * @details This class implements a recursive semaphore (mutex) that allows the same thread
 *          to acquire the lock multiple times without deadlocking. The implementation tracks
 *          the owning thread and maintains a recursion counter (take_count) to ensure proper
 *          nested acquisition and release.
 *          
 *          **Implementation Details**:
 *          - Uses pthread_mutex_t with PTHREAD_MUTEX_RECURSIVE attribute
 *          - Tracks ownership with pthread_t thread ID
 *          - Maintains take_count for recursion depth tracking
 *          - Supports both blocking and non-blocking acquisition
 *          - Integrates with SITL scheduler for timeout handling
 *          
 *          **Thread Safety**:
 *          - Safe for concurrent access from multiple threads
 *          - Recursive: Same thread can call take() multiple times
 *          - Must call give() same number of times as take() to fully release
 *          - Owner tracking enables deadlock detection via check_owner()
 *          
 *          **Usage Pattern with WITH_SEMAPHORE Macro**:
 *          @code
 *          HALSITL::Semaphore my_semaphore;
 *          
 *          void protected_function() {
 *              WITH_SEMAPHORE(my_semaphore);  // RAII-style acquisition
 *              // Critical section - semaphore held
 *              // Automatically released when scope exits
 *          }
 *          @endcode
 *          
 *          **Manual Usage Pattern**:
 *          @code
 *          if (my_semaphore.take(100)) {  // Wait up to 100ms
 *              // Critical section
 *              my_semaphore.give();
 *          }
 *          @endcode
 * 
 * @warning In SITL simulation, timeout values interact with simulated time, not wall-clock
 *          time. The scheduler may advance simulation time faster than real-time, causing
 *          timeouts to expire differently than on hardware.
 * 
 * @note This semaphore can be acquired recursively by the same thread. Each take() must
 *       be matched with a corresponding give().
 * 
 * @see AP_HAL::Semaphore for the base class interface
 * @see WITH_SEMAPHORE macro in AP_HAL_Macros.h for RAII-style usage
 */
class HALSITL::Semaphore : public AP_HAL::Semaphore {
public:
    friend class HALSITL::BinarySemaphore;
    
    /**
     * @brief Construct a new Semaphore object
     * 
     * @details Initializes a recursive pthread mutex for synchronization.
     *          The semaphore starts in the unlocked state (no owner, take_count = 0).
     *          
     *          Implementation: Creates a pthread_mutex_t with PTHREAD_MUTEX_RECURSIVE
     *          attribute to allow the same thread to acquire multiple times.
     */
    Semaphore();
    
    /**
     * @brief Release (unlock) the semaphore
     * 
     * @details Decrements the recursion counter and releases ownership when count reaches zero.
     *          If the semaphore was acquired multiple times recursively, give() must be called
     *          the same number of times to fully release the lock.
     *          
     *          Thread Safety: Must be called by the thread that currently owns the semaphore.
     *          Calling give() without owning the semaphore results in undefined behavior.
     *          
     *          Implementation: Decrements take_count. When take_count reaches zero, clears
     *          the owner thread ID and unlocks the underlying pthread_mutex_t.
     * 
     * @return true Successfully released the semaphore
     * @return false Failed to release (should not occur in normal operation)
     * 
     * @note Must be called once for each successful take() call to fully release
     * @warning Calling give() from a thread that doesn't own the semaphore is undefined behavior
     * 
     * @see take()
     * @see take_nonblocking()
     */
    bool give() override;
    
    /**
     * @brief Acquire (lock) the semaphore with timeout
     * 
     * @details Attempts to acquire the semaphore, blocking the calling thread for up to
     *          timeout_ms milliseconds if the semaphore is currently held by another thread.
     *          If the same thread already owns the semaphore, immediately succeeds and
     *          increments the recursion counter (take_count).
     *          
     *          **Timeout Handling in SITL**:
     *          The timeout interacts with the SITL scheduler's stop_clock mechanism. When
     *          blocking on semaphore acquisition, the scheduler may advance simulation time,
     *          affecting the effective wait duration. Timeout of 0 attempts non-blocking
     *          acquisition. Timeout of HAL_SEMAPHORE_BLOCK_FOREVER (UINT32_MAX) blocks
     *          indefinitely until acquisition succeeds.
     *          
     *          **Blocking Behavior**:
     *          - If semaphore is available: Acquires immediately, sets owner, increments take_count
     *          - If owned by same thread: Succeeds immediately, increments take_count (recursive)
     *          - If owned by other thread: Blocks up to timeout_ms waiting for release
     *          - After timeout: Returns false if acquisition failed
     * 
     * @param[in] timeout_ms Maximum time to wait in milliseconds
     *                       - 0 = non-blocking (equivalent to take_nonblocking())
     *                       - UINT32_MAX (HAL_SEMAPHORE_BLOCK_FOREVER) = block indefinitely
     *                       - Other values = block for specified milliseconds
     * 
     * @return true Successfully acquired the semaphore
     * @return false Timeout expired before acquisition, or semaphore in error state
     * 
     * @note Recursive: Same thread can call take() multiple times successfully
     * @warning Simulated time in SITL may advance faster than wall-clock time, affecting timeouts
     * @warning Long blocking waits can interfere with scheduler timing; prefer shorter timeouts
     * 
     * @see give()
     * @see take_nonblocking()
     * @see check_owner()
     */
    bool take(uint32_t timeout_ms) override;
    
    /**
     * @brief Attempt to acquire the semaphore without blocking
     * 
     * @details Attempts to acquire the semaphore immediately without waiting. Returns
     *          immediately whether acquisition succeeded or failed. Equivalent to take(0).
     *          If the same thread already owns the semaphore, succeeds immediately and
     *          increments the recursion counter.
     *          
     *          Use this method when you need to conditionally protect a resource but
     *          cannot afford to block the calling thread. Common in high-frequency
     *          control loops where timing is critical.
     *          
     *          Implementation: Calls pthread_mutex_trylock() which returns immediately.
     * 
     * @return true Successfully acquired the semaphore
     * @return false Semaphore is currently held by another thread, or in error state
     * 
     * @note Never blocks - returns immediately
     * @note Recursive: Same thread can call multiple times successfully
     * @note Useful in fast control loops where blocking is unacceptable
     * 
     * @see take() for blocking acquisition with timeout
     * @see give()
     */
    bool take_nonblocking() override;

    /**
     * @brief Assert that the current thread owns the semaphore
     * 
     * @details Debugging utility that verifies the calling thread is the current owner
     *          of the semaphore. Triggers an assertion failure if called by a thread
     *          that doesn't hold the semaphore. Useful for detecting lock ownership
     *          violations and deadlock conditions during development.
     *          
     *          Common usage: Call at the entry of functions that require the semaphore
     *          to already be held, to catch programming errors where the function is
     *          called without proper synchronization.
     * 
     * @note Only enabled in debug builds; may be compiled out in release builds
     * @warning Assertion failure (program abort) if current thread is not the owner
     * 
     * @see take()
     * @see give()
     */
    void check_owner() const;

protected:
    /**
     * @brief POSIX pthread mutex providing the underlying synchronization primitive
     * 
     * @details Initialized as PTHREAD_MUTEX_RECURSIVE to allow same thread to lock multiple
     *          times. Protected visibility allows BinarySemaphore friend class access.
     */
    pthread_mutex_t _lock;
    
    /**
     * @brief Thread ID of the current semaphore owner
     * 
     * @details Stores pthread_t ID of the thread that currently holds the semaphore.
     *          Set to the calling thread's ID on successful take(), cleared on final give().
     *          Used for recursion support and owner verification in check_owner().
     *          Value is undefined when take_count == 0 (semaphore not held).
     */
    pthread_t owner;

    /**
     * @brief Recursion depth counter tracking how many times the owner has acquired the semaphore
     * 
     * @details Incremented on each successful take() call by the owner thread. Decremented
     *          on each give() call. The semaphore is fully released only when take_count
     *          reaches zero. Enables recursive locking where the same thread can acquire
     *          the semaphore multiple times without deadlocking.
     *          
     *          Range: 0 (unlocked) to 255 (maximum recursion depth)
     * 
     * @note Must be zero when semaphore is not held
     * @note Maximum recursion depth is 255 (uint8_t limit)
     */
    uint8_t take_count;
};


/**
 * @class HALSITL::BinarySemaphore
 * @brief Binary semaphore (condition variable) implementation for thread signaling in SITL
 * 
 * @details This class implements a binary semaphore using POSIX condition variables
 *          (pthread_cond_t) for efficient thread signaling and synchronization. Unlike
 *          the recursive Semaphore class, BinarySemaphore is designed for producer-consumer
 *          patterns and event notification where one thread signals and another waits.
 *          
 *          **Implementation Architecture**:
 *          - Uses pthread_cond_t for blocking waits with efficient wakeup
 *          - Protected by HALSITL::Semaphore (mutex) for thread-safe pending flag access
 *          - Maintains 'pending' flag to handle signal-before-wait race conditions
 *          - Supports timed waits with microsecond precision
 *          
 *          **Signaling Semantics**:
 *          - signal() sets pending=true and wakes one waiting thread
 *          - wait() blocks until pending=true, then clears pending and returns
 *          - If signal() called before wait(), the signal is "remembered" via pending flag
 *          - Multiple signal() calls before wait() collapse to single wakeup (binary behavior)
 *          
 *          **Common Usage Patterns**:
 *          
 *          **Producer-Consumer Pattern**:
 *          @code
 *          HALSITL::BinarySemaphore data_ready;
 *          
 *          // Producer thread
 *          void producer() {
 *              prepare_data();
 *              data_ready.signal();  // Wake consumer
 *          }
 *          
 *          // Consumer thread
 *          void consumer() {
 *              if (data_ready.wait(1000000)) {  // Wait up to 1 second (1,000,000 us)
 *                  process_data();
 *              }
 *          }
 *          @endcode
 *          
 *          **Event Notification**:
 *          @code
 *          HALSITL::BinarySemaphore event_occurred;
 *          
 *          void wait_for_event() {
 *              event_occurred.wait_blocking();  // Wait indefinitely
 *              handle_event();
 *          }
 *          @endcode
 *          
 *          **Difference from Semaphore**:
 *          - Semaphore: Mutual exclusion, resource locking, can be recursive
 *          - BinarySemaphore: Event signaling, thread coordination, binary state (signaled/not signaled)
 * 
 * @warning Timing behavior in SITL simulation uses simulated time, not wall-clock time.
 *          Timeout values interact with scheduler stop_clock advance, potentially causing
 *          waits to complete faster or slower than expected in real-time.
 * 
 * @note Not recursive - multiple wait() calls from same thread will block
 * @note Not copyable - enforced by CLASS_NO_COPY macro
 * @note signal() is safe to call from any thread, including interrupt context (in real HAL)
 * 
 * @see AP_HAL::BinarySemaphore for the base class interface
 * @see HALSITL::Semaphore for mutex-based resource locking
 */
class HALSITL::BinarySemaphore : public AP_HAL::BinarySemaphore {
public:
    /**
     * @brief Construct a new Binary Semaphore object
     * 
     * @details Initializes the binary semaphore with the specified initial state.
     *          If initial_state is true, the semaphore starts in the "signaled" state,
     *          meaning the first wait() call will return immediately without blocking.
     *          If false, the semaphore starts "unsignaled" and wait() will block until
     *          signal() is called.
     *          
     *          Implementation: Initializes pthread_cond_t, creates internal mutex (mtx),
     *          and sets pending flag to initial_state.
     * 
     * @param[in] initial_state Initial signaled state
     *                          - true: Semaphore starts signaled (wait() returns immediately)
     *                          - false: Semaphore starts unsignaled (wait() blocks)
     *                          Default: false (unsignaled)
     * 
     * @note Usually initialized as false (unsignaled) for typical producer-consumer patterns
     */
    BinarySemaphore(bool initial_state=false);

    CLASS_NO_COPY(BinarySemaphore);

    /**
     * @brief Wait for semaphore signal with microsecond timeout
     * 
     * @details Blocks the calling thread until the semaphore is signaled via signal() or
     *          the timeout expires. Uses pthread_cond_timedwait() for efficient blocking
     *          with automatic wakeup on signal or timeout.
     *          
     *          **Blocking Behavior**:
     *          - If pending flag is already true: Clears pending, returns immediately with true
     *          - If pending is false: Blocks until signal() sets pending, or timeout expires
     *          - On wakeup: Clears pending flag and returns true
     *          - On timeout: Returns false, pending flag unchanged
     *          
     *          **Timeout Handling in SITL**:
     *          Timeout specified in microseconds interacts with simulated time. The SITL
     *          scheduler may advance simulation time faster than wall-clock, causing the
     *          effective wait duration to differ from real-time execution. Timeout of 0
     *          checks the pending flag without blocking.
     *          
     *          **Race Condition Handling**:
     *          The pending flag prevents signal-before-wait race conditions. If signal()
     *          is called before wait(), the signal is "remembered" and wait() returns
     *          immediately.
     *          
     *          Thread Safety: Protected by internal mutex (mtx). Multiple threads can
     *          safely call wait() and signal() concurrently.
     * 
     * @param[in] timeout_us Maximum wait time in microseconds
     *                       - 0 = non-blocking check (returns immediately)
     *                       - >0 = block for specified microseconds
     *                       Note: No "infinite wait" option; use wait_blocking() instead
     * 
     * @return true Semaphore was signaled (pending flag was/became true)
     * @return false Timeout expired before semaphore was signaled
     * 
     * @note Timeout precision depends on SITL scheduler timing resolution
     * @warning Simulated time in SITL may advance non-linearly relative to wall-clock time
     * @warning Multiple waiting threads: only one is woken per signal() call
     * 
     * @see signal()
     * @see wait_blocking()
     */
    bool wait(uint32_t timeout_us) override;
    
    /**
     * @brief Wait for semaphore signal indefinitely (blocking)
     * 
     * @details Blocks the calling thread indefinitely until the semaphore is signaled
     *          via signal(). No timeout - waits forever until explicitly signaled.
     *          Uses pthread_cond_wait() which blocks efficiently without busy-waiting.
     *          
     *          **Blocking Behavior**:
     *          - If pending flag is already true: Clears pending, returns immediately
     *          - If pending is false: Blocks indefinitely until signal() sets pending
     *          - On wakeup: Clears pending flag and returns
     *          
     *          Use this when you need guaranteed synchronization and can tolerate
     *          indefinite blocking. Common in initialization sequences or shutdown
     *          coordination where timeout would indicate unrecoverable error.
     *          
     *          Thread Safety: Protected by internal mutex (mtx). Safe for concurrent
     *          access from multiple threads.
     * 
     * @return true Always returns true when semaphore is signaled (no timeout possible)
     * 
     * @note Blocks forever until signal() is called - no timeout
     * @note Prefer wait(timeout_us) in most cases to prevent indefinite hangs
     * @warning Can cause deadlock if signal() is never called
     * @warning Thread will never return if signal() is not called by another thread
     * 
     * @see signal()
     * @see wait()
     */
    bool wait_blocking(void) override;
    
    /**
     * @brief Signal the semaphore, waking one waiting thread
     * 
     * @details Sets the pending flag and wakes up one thread blocked in wait() or
     *          wait_blocking(). Uses pthread_cond_signal() to wake exactly one waiter.
     *          If no threads are waiting, the signal is "remembered" via the pending flag,
     *          and the next wait() call will return immediately.
     *          
     *          **Signaling Semantics**:
     *          - Sets pending=true atomically under mutex protection
     *          - Wakes one waiting thread via pthread_cond_signal()
     *          - If no threads waiting: pending remains true for next wait() call
     *          - Multiple signal() calls collapse to single wakeup (binary semaphore)
     *          
     *          **Thread Safety**:
     *          Safe to call from any thread, including:
     *          - Interrupt handlers (on hardware HAL implementations)
     *          - Scheduler callbacks
     *          - Normal application threads
     *          - Concurrent signal() calls from multiple threads
     *          
     *          **Producer-Consumer Pattern**:
     *          Typically called by producer thread after preparing data or event,
     *          to notify consumer thread(s) that work is ready.
     * 
     * @note Wakes at most one waiting thread (pthread_cond_signal, not pthread_cond_broadcast)
     * @note Safe to call multiple times; multiple signals collapse to single wakeup
     * @note Safe to call even if no threads are waiting (signal is remembered)
     * @note Does not block - returns immediately after setting pending and signaling
     * 
     * @see wait()
     * @see wait_blocking()
     */
    void signal(void) override;

private:
    /**
     * @brief Internal mutex protecting access to pending flag and condition variable
     * 
     * @details Provides mutual exclusion for the pending flag and coordinates with
     *          pthread_cond_t operations. Must be held when checking or modifying
     *          pending flag, and when calling pthread_cond_wait/signal.
     *          
     *          Uses HALSITL::Semaphore for consistency with other SITL synchronization.
     */
    HALSITL::Semaphore mtx;
    
    /**
     * @brief POSIX condition variable for efficient thread blocking and wakeup
     * 
     * @details Provides the underlying blocking primitive. Threads call pthread_cond_wait()
     *          to block until pthread_cond_signal() is called. Works in conjunction with
     *          mtx to provide atomic wait operations.
     */
    pthread_cond_t cond;
    
    /**
     * @brief Pending signal flag indicating semaphore is in signaled state
     * 
     * @details Boolean flag protected by mtx mutex:
     *          - true: Semaphore has been signaled, next wait() returns immediately
     *          - false: Semaphore is unsignaled, wait() will block
     *          
     *          Set to true by signal(), cleared by wait()/wait_blocking() after wakeup.
     *          Prevents race condition where signal() is called before wait(), ensuring
     *          the signal is not lost (binary semaphore "memory" behavior).
     * 
     * @note Always accessed under mtx protection to ensure thread safety
     */
    bool pending;
};
