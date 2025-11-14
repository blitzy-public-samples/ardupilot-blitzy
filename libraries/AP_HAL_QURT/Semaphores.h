/**
 * @file Semaphores.h
 * @brief QURT (Qualcomm Hexagon DSP) semaphore implementations using pthread synchronization primitives
 * 
 * @details This file provides thread synchronization primitives for the QURT HAL,
 *          implemented using POSIX pthread mutexes and condition variables optimized
 *          for the Hexagon DSP architecture. These semaphores are used throughout
 *          the QURT HAL to protect shared resources and coordinate between threads.
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <stdint.h>
#include <AP_HAL/Semaphores.h>
#include <pthread.h>

namespace QURT
{

/**
 * @class Semaphore
 * @brief Recursive mutex-based semaphore for thread synchronization on Hexagon DSP
 * 
 * @details Implements AP_HAL::Semaphore using pthread_mutex with PTHREAD_MUTEX_RECURSIVE
 *          attribute. This provides a reentrant locking mechanism where the same thread
 *          can acquire the semaphore multiple times without causing deadlock (provided
 *          it releases the same number of times).
 * 
 *          The recursive property is critical for ArduPilot's architecture where callback
 *          chains and nested function calls frequently need to access the same protected
 *          resource. Without recursion support, these patterns would deadlock.
 * 
 *          Typical usage pattern employs the WITH_SEMAPHORE macro for RAII-style
 *          automatic lock/unlock:
 *          @code
 *          void update_shared_data() {
 *              WITH_SEMAPHORE(semaphore);
 *              // Protected code - semaphore automatically released on scope exit
 *              shared_variable = new_value;
 *          }
 *          @endcode
 * 
 * @note Used throughout QURT HAL for protecting shared state including:
 *       - RCInput channel data
 *       - UARTDriver transmit/receive buffers
 *       - SPI/I2C device bus access
 *       - Shared sensor data structures
 * 
 * @warning Recursive mutexes have performance overhead compared to normal mutexes
 *          (typically 1-2x slower on Hexagon DSP due to additional owner tracking),
 *          but prevent common deadlock scenarios in ArduPilot's callback-heavy architecture.
 * 
 * @warning DSP timing considerations: pthread mutex operations add 1-10μs overhead per
 *          acquire/release on Hexagon DSP. Minimize critical section duration to avoid
 *          impacting real-time performance.
 * 
 * @see DeviceBus.h for example usage in SPI/I2C bus synchronization
 */
class Semaphore : public AP_HAL::Semaphore
{
public:
    friend class BinarySemaphore;
    Semaphore();
    
    /**
     * @brief Release the semaphore (unlock the mutex)
     * 
     * @details Calls pthread_mutex_unlock() to release ownership of the mutex.
     *          If the same thread has acquired the semaphore multiple times
     *          (recursive locking), it must call give() the same number of times
     *          before other threads can acquire the semaphore.
     * 
     * @return true if successfully released, false on error
     * 
     * @warning Must be called by the thread that acquired the semaphore.
     *          Calling give() from a different thread results in undefined
     *          behavior on QURT/pthread implementations.
     */
    bool give() override;
    
    /**
     * @brief Acquire the semaphore with timeout
     * 
     * @details Attempts to acquire the mutex with a specified timeout. If the
     *          semaphore is already held by another thread, this call blocks
     *          until either the semaphore becomes available or the timeout expires.
     *          The calling thread yields DSP CPU cycles to other threads while waiting.
     * 
     * @param timeout_ms Maximum wait time in milliseconds (0 = try once, UINT32_MAX = wait indefinitely)
     * 
     * @return true if semaphore was acquired successfully, false if timeout elapsed
     * 
     * @note This is a blocking call that yields to other threads. Avoid calling
     *       from interrupt context or with very short timeouts in time-critical paths.
     * 
     * @note If the calling thread already owns the semaphore (recursive case),
     *       this call succeeds immediately and increments the lock count.
     */
    bool take(uint32_t timeout_ms) override;
    
    /**
     * @brief Try to acquire semaphore without blocking
     * 
     * @details Attempts to acquire the mutex immediately without blocking.
     *          Returns immediately whether successful or not. Useful for
     *          lock-free polling patterns.
     * 
     * @return true if semaphore was acquired immediately, false if already held by another thread
     * 
     * @note Safe to call from any context including time-critical loops.
     *       Use for polling-based synchronization where blocking is unacceptable.
     * 
     * @note If the calling thread already owns the semaphore (recursive case),
     *       this call succeeds immediately and increments the lock count.
     */
    bool take_nonblocking() override;
    
    bool check_owner(void);
protected:
    // qurt_mutex_t _lock;
    pthread_mutex_t _lock;   ///< Underlying pthread mutex (configured as recursive)
    pthread_t owner;         ///< Thread ID of current owner (for recursive lock tracking)
};


/**
 * @class BinarySemaphore
 * @brief Condition variable-based binary semaphore for event signaling
 * 
 * @details Implements AP_HAL::BinarySemaphore using a combination of:
 *          - pthread_cond (condition variable for thread wake-up)
 *          - pthread_mutex (protects the pending flag)
 *          - bool pending (tracks signaled state)
 * 
 *          A binary semaphore has only two states: signaled or not signaled.
 *          Multiple signal() calls without a corresponding wait() are collapsed
 *          into a single signal. This differs from counting semaphores where
 *          each signal increments a counter.
 * 
 *          Algorithm:
 *          - signal(): Acquires mutex, sets pending=true, signals condition variable, releases mutex
 *          - wait(): Acquires mutex, waits on condition variable until pending=true, clears pending, releases mutex
 * 
 *          Typical usage is for event notification and thread synchronization:
 *          @code
 *          // Producer thread:
 *          data_ready = true;
 *          semaphore.signal();  // Wake up consumer
 * 
 *          // Consumer thread:
 *          semaphore.wait_blocking();  // Sleep until signaled
 *          process_data();
 *          @endcode
 * 
 * @note More efficient than recursive Semaphore for simple event notification
 *       (no recursive locking overhead, optimized for signal/wait patterns).
 * 
 * @note The pending flag prevents race conditions and ensures that signals
 *       are not lost even if signal() is called before wait().
 * 
 * @warning Condition variables can experience spurious wakeups (pthread_cond_wait
 *          may return without signal() being called, per POSIX spec). This
 *          implementation handles spurious wakeups correctly by checking the
 *          pending flag in a loop.
 * 
 * @warning DSP timing: Condition variable operations involve thread scheduling
 *          and context switches. Expect 5-50μs latency from signal() to the
 *          waiting thread actually resuming execution on Hexagon DSP.
 */
class BinarySemaphore : public AP_HAL::BinarySemaphore
{
public:
    BinarySemaphore(bool initial_state=false);

    /**
     * @brief Wait for signal with microsecond timeout
     * 
     * @details Blocks the calling thread until either:
     *          1. The semaphore is signaled via signal(), or
     *          2. The timeout expires
     * 
     *          Uses pthread_cond_timedwait() with CLOCK_MONOTONIC for timeout handling.
     *          The pending flag is checked in a loop to handle spurious wakeups correctly.
     *          If signaled, the pending flag is automatically cleared before returning.
     * 
     * @param timeout_us Maximum wait time in microseconds
     * 
     * @return true if semaphore was signaled, false if timeout elapsed
     * 
     * @note Timeout resolution depends on QURT scheduler tick rate (typically 1ms).
     *       Actual wait time may be slightly longer than requested due to scheduling granularity.
     * 
     * @note Uses CLOCK_MONOTONIC to avoid issues with system time adjustments.
     *       Immune to NTP corrections or manual time changes.
     */
    bool wait(uint32_t timeout_us) override;
    
    /**
     * @brief Wait indefinitely for signal
     * 
     * @details Blocks the calling thread until the semaphore is signaled via signal().
     *          No timeout - waits forever until signaled. Uses pthread_cond_wait()
     *          with proper handling of spurious wakeups via pending flag check.
     * 
     * @return Always returns true (after being signaled)
     * 
     * @warning Can cause watchdog timeouts if signal() is never called.
     *          Use with caution in safety-critical code. Prefer wait() with
     *          a reasonable timeout for production code.
     * 
     * @note The calling thread yields all CPU time while waiting, allowing
     *       other threads and the DSP to enter low-power states if no work is available.
     */
    bool wait_blocking(void) override;
    
    /**
     * @brief Signal the semaphore to wake waiting thread(s)
     * 
     * @details Sets the pending flag and wakes up one thread waiting on the
     *          condition variable (if any). If no threads are waiting, the
     *          pending flag remains set so the next wait() call returns immediately.
     * 
     *          Multiple signal() calls without intervening wait() are collapsed
     *          into a single signal (binary semaphore property).
     * 
     * @note Safe to call from any thread context. Uses mutex protection to
     *       ensure thread-safe access to the pending flag.
     * 
     * @note If multiple threads are waiting, only one is awakened (POSIX
     *       pthread_cond_signal behavior). Use condition variable broadcast
     *       semantics if multiple threads need to be awakened simultaneously.
     */
    void signal(void) override;

protected:
    Semaphore mtx;           ///< Mutex protecting the pending flag and condition variable
    pthread_cond_t cond;     ///< Condition variable for thread wake-up notifications
    bool pending;            ///< Flag indicating semaphore has been signaled (true = signaled, false = not signaled)
};

/**
 * @note QURT Implementation Details:
 *       QURT provides an optimized pthread implementation specifically tuned for
 *       the Hexagon DSP architecture. The implementation is aware of Hexagon's
 *       hardware threading capabilities, cache hierarchy (L1/L2), and memory
 *       subsystem characteristics. Synchronization primitives use hardware-assisted
 *       atomic operations where available and are optimized for the DSP's memory
 *       consistency model.
 * 
 * @see AP_HAL_QURT/DeviceBus.h for examples of semaphore usage in SPI/I2C device drivers
 * @see AP_HAL_QURT/UARTDriver.h for examples protecting serial port buffers
 * @see AP_HAL_QURT/RCInput.h for examples protecting RC channel data
 */

}
