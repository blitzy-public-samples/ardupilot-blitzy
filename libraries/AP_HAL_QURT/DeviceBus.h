/*
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
 * @file DeviceBus.h
 * @brief Device bus abstraction for scheduling periodic device callbacks on Qualcomm Hexagon DSP (QURT platform)
 * 
 * @details This file provides the base infrastructure for managing per-bus callback scheduling
 *          with dedicated threads on the QURT (Qualcomm Real-Time) platform. The DeviceBus class
 *          enables SPI and I2C device drivers to schedule periodic callbacks at configurable rates,
 *          with each bus running its own dedicated thread for servicing device operations.
 * 
 *          Architecture: Each DeviceBus instance spawns a single bus thread that services all
 *          device callbacks registered on that bus sequentially. This design prevents concurrent
 *          access to the bus hardware while allowing multiple devices to share the same bus.
 * 
 * @note This is part of the QURT Hardware Abstraction Layer (HAL) implementation for ArduPilot
 *       on Qualcomm Hexagon DSP platforms (e.g., Snapdragon Flight).
 * 
 * @see SPIDevice.h for SPI bus implementation using this infrastructure
 * @see I2CDevice.h for I2C bus implementation using this infrastructure
 * @see Semaphores.h for HAL_Semaphore used for bus synchronization
 */

#pragma once

#include <inttypes.h>
#include <AP_HAL/HAL.h>
#include "Semaphores.h"
#include "AP_HAL_QURT.h"
#include "Scheduler.h"

#define HAL_QURT_DEVICE_STACK_SIZE 8192

namespace QURT
{

/**
 * @class DeviceBus
 * @brief Base class for managing per-bus callback scheduling with dedicated thread
 * 
 * @details DeviceBus provides infrastructure for SPI/I2C buses to schedule device-specific
 *          callbacks at configurable rates on the QURT platform (Qualcomm Hexagon DSP).
 *          Each DeviceBus instance spawns a single bus thread that services all device
 *          callbacks registered on that bus sequentially, preventing concurrent bus access
 *          while allowing multiple devices to share the same physical bus.
 * 
 * Architecture:
 * - One DeviceBus instance per physical bus (e.g., SPI1, I2C0)
 * - Single bus thread per DeviceBus instance
 * - Multiple devices register callbacks on the same bus
 * - Callbacks execute sequentially (not concurrently) to avoid bus conflicts
 * - Bus semaphore held during callback execution
 * 
 * Scheduling Algorithm:
 * - Maintains linked list of callback_info entries, each with period_usec and next_usec
 * - Bus thread calculates next wakeup time (earliest next_usec across all callbacks)
 * - Thread sleeps using qurt_timer_sleep() for microsecond-accurate timing
 * - Upon waking, executes all callbacks where current_time >= next_usec
 * - Updates each callback's next_usec by adding period_usec for consistent rate
 * 
 * Thread Execution Flow:
 * 1. Calculate next wakeup time from all registered callbacks
 * 2. Sleep until next callback is due (qurt_timer_sleep)
 * 3. Acquire bus semaphore
 * 4. Execute all due callbacks sequentially
 * 5. Release bus semaphore
 * 6. Repeat from step 1
 * 
 * @note Uses qurt_timer_sleep() for microsecond-accurate timing between callbacks.
 *       QURT timer provides <10us jitter for periods >100us, suitable for sensor polling rates.
 * 
 * @warning Bus thread stack size is HAL_QURT_DEVICE_STACK_SIZE (8192 bytes). This must
 *          accommodate the deepest callback call stack. If callbacks are deeply nested or
 *          allocate large stack variables, stack overflow may occur.
 * 
 * @warning Very short periods (<100us) may exceed timer resolution, causing irregular
 *          callback intervals and increased jitter.
 * 
 * @warning If a callback blocks indefinitely (e.g., waiting on another semaphore), the
 *          entire bus is blocked, affecting all devices on that bus. Keep callbacks fast
 *          and non-blocking.
 * 
 * @warning DSP CPU budget: The sum of (callback_duration × frequency) across all callbacks
 *          must leave sufficient CPU budget for the main vehicle loop and other threads.
 *          Oversubscription leads to scheduler starvation and timing violations.
 * 
 * @note Period jitter: If callback execution time approaches period_usec, jitter increases.
 *       Keep callbacks fast (typically <50% of period) to maintain consistent timing.
 * 
 * @note Thread priority: Bus thread priority should be lower than APM_MAIN_PRIORITY but
 *       higher than background tasks to ensure sensor data is serviced reliably without
 *       starving the main flight control loop.
 * 
 * @note During callback execution, the bus semaphore is held to prevent other threads
 *       from accessing the bus simultaneously. This ensures thread-safe bus access but
 *       means the main thread will block if it tries to access the bus during a callback.
 * 
 * @warning Callback synchronization: Callbacks should not block on other semaphores that
 *          might be held by the main thread, as this creates potential for deadlock
 *          (main thread waits on bus semaphore, bus thread waits on other semaphore held
 *          by main thread).
 * 
 * Example Usage Pattern:
 * @code
 * // SPIBus creates DeviceBus during initialization
 * SPIBus spi_bus = new SPIBus(...);
 * 
 * // IMU device registers 1000Hz (1000us period) callback
 * AP_HAL::Device::PeriodicHandle handle = 
 *     spi_bus->register_periodic_callback(1000, callback_func, imu_device);
 * 
 * // Bus thread wakes every 1000us → acquires semaphore → executes IMU callback
 * // → IMU reads sensor via SPI transfer → releases semaphore → sleeps until next period
 * @endcode
 * 
 * @note Used by SPIBus and I2CBus classes to provide device-specific polling infrastructure
 *       for sensor drivers on the QURT platform.
 * 
 * @see register_periodic_callback() for adding callbacks to the bus schedule
 * @see bus_thread() for the main scheduling loop implementation
 * @see SPIDevice.h and I2CDevice.h for concrete bus implementations
 */
class DeviceBus
{
public:
    /**
     * @brief Construct a new DeviceBus with specified thread priority
     * 
     * @param[in] _thread_priority Priority for the bus thread (should be lower than
     *                             APM_MAIN_PRIORITY but higher than background tasks)
     * 
     * @note The bus thread is not started in the constructor. It will be started
     *       when the first callback is registered via register_periodic_callback().
     */
    DeviceBus(AP_HAL::Scheduler::priority_base _thread_priority);

    /**
     * @brief Pointer to next DeviceBus in global bus list
     * @note Used for maintaining a linked list of all active buses in the system
     */
    struct DeviceBus *next;
    
    /**
     * @brief Semaphore for synchronizing bus access between threads
     * 
     * @details This semaphore is held during callback execution to prevent concurrent
     *          access to the bus hardware. The main thread must also acquire this
     *          semaphore before performing bus transactions outside of callbacks.
     * 
     * @note The bus thread holds this semaphore while executing device callbacks.
     * @warning Do not hold this semaphore for extended periods as it blocks all
     *          device operations on this bus.
     */
    HAL_Semaphore semaphore;

    /**
     * @brief Register a periodic callback to be executed on the bus thread
     * 
     * @details Adds a new callback to the bus schedule. The callback will be executed
     *          approximately every period_usec microseconds by the bus thread. If this
     *          is the first callback registered on this bus, the bus thread will be
     *          started automatically.
     * 
     *          The callback executes with the bus semaphore held, ensuring exclusive
     *          access to the bus hardware. Multiple callbacks on the same bus execute
     *          sequentially (not concurrently) in order of their scheduled execution times.
     * 
     * @param[in] period_usec Callback period in microseconds (e.g., 1000 for 1kHz)
     * @param[in] cb Callback function to execute (AP_HAL::Device::PeriodicCb type)
     * @param[in] hal_device Pointer to the device object (stored but may not be used)
     * 
     * @return Handle to the registered callback, or nullptr if registration failed
     * 
     * @note Thread-safe: Uses semaphore protection for callback list modifications
     * @note The actual callback rate may have jitter depending on other callbacks on
     *       the same bus and the execution time of each callback.
     * @note Callbacks should be fast (typically <50% of period_usec) to minimize jitter
     * 
     * @warning If the sum of callback execution times exceeds available CPU budget,
     *          timing violations will occur across the system.
     * @warning Callbacks must not block indefinitely or the entire bus will stall.
     * @warning Callbacks should not acquire semaphores held by the main thread to
     *          avoid deadlock scenarios.
     * 
     * @see adjust_timer() to modify the period of an existing callback
     * @see bus_thread() for the scheduling loop that executes callbacks
     */
    AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb, AP_HAL::Device *hal_device);
    
    /**
     * @brief Adjust the period of an existing periodic callback
     * 
     * @details Modifies the period_usec of a previously registered callback without
     *          unregistering and re-registering it. The next_usec execution time is
     *          recalculated based on the new period.
     * 
     * @param[in] h Handle to the callback (returned by register_periodic_callback)
     * @param[in] period_usec New callback period in microseconds
     * 
     * @return true if the timer was successfully adjusted, false if handle is invalid
     * 
     * @note Thread-safe: Uses semaphore protection for callback list modifications
     * @note The new period takes effect at the next scheduled callback execution
     */
    bool adjust_timer(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec);
    
    /**
     * @brief Main bus thread function that executes the callback scheduling loop
     * 
     * @details This function implements the infinite scheduling loop for the bus thread:
     * 
     * 1. Calculate next wakeup time (earliest next_usec across all callbacks)
     * 2. Sleep until next callback is due using qurt_timer_sleep()
     * 3. Acquire bus semaphore
     * 4. Execute all callbacks where current_time >= next_usec
     * 5. Update each executed callback's next_usec by adding period_usec
     * 6. Release bus semaphore
     * 7. Repeat from step 1
     * 
     * Callbacks execute sequentially (not concurrently) to avoid bus conflicts.
     * The bus semaphore is held during callback execution to prevent other threads
     * from accessing the bus simultaneously.
     * 
     * @note This function never returns - it runs for the lifetime of the system.
     * @note Called automatically when the bus thread is started by the first
     *       register_periodic_callback() call.
     * 
     * @note Timing accuracy: Uses qurt_timer_sleep() which provides <10us jitter
     *       for periods >100us, suitable for sensor polling rates.
     * 
     * @warning If a callback blocks indefinitely, the entire bus is blocked and
     *          all devices on that bus will stop functioning.
     * @warning Stack size is HAL_QURT_DEVICE_STACK_SIZE (8192 bytes) - callbacks
     *          must not exceed this stack depth.
     * 
     * @see register_periodic_callback() for adding callbacks to the schedule
     */
    void bus_thread(void);

private:
    /**
     * @struct callback_info
     * @brief Callback entry for periodic device operation scheduling
     * 
     * @details Internal structure that maintains the state for each registered periodic
     *          callback. The bus thread maintains a linked list of these structures and
     *          uses them to determine when to execute each callback.
     * 
     *          Each callback_info contains:
     *          - Callback function pointer
     *          - Requested period in microseconds
     *          - Next scheduled execution time in microseconds (monotonic)
     * 
     * Timing Mechanism:
     * - next_usec is initialized to current_time + period_usec when callback is registered
     * - After each callback execution, next_usec is updated: next_usec += period_usec
     * - This maintains a consistent rate even if callback execution is delayed
     * - The bus thread wakes when current_time >= next_usec for any callback
     * 
     * @note Period jitter: If callback execution time approaches period_usec, jitter
     *       increases because the next execution time is delayed. Keep callbacks fast
     *       (typically <50% of period) to maintain consistent timing.
     * 
     * @note The next pointer forms a singly-linked list of all callbacks on this bus.
     *       The list is traversed during each scheduling cycle to find due callbacks.
     * 
     * @warning All time values use monotonic microsecond timestamps from the QURT
     *          platform timer. Do not use wall-clock time or assume time starts at zero.
     */
    struct callback_info {
        /**
         * @brief Pointer to next callback in the linked list
         * @note nullptr indicates end of list
         */
        struct callback_info *next;
        
        /**
         * @brief Callback function to execute
         * @note Type is AP_HAL::Device::PeriodicCb (member function pointer)
         */
        AP_HAL::Device::PeriodicCb cb;
        
        /**
         * @brief Requested callback period in microseconds
         * @note This is the target period; actual period may have jitter depending
         *       on callback execution time and other bus activity
         */
        uint32_t period_usec;
        
        /**
         * @brief Next scheduled execution time in microseconds (monotonic timestamp)
         * @details Updated after each callback execution by adding period_usec.
         *          The bus thread compares current_time against this value to
         *          determine when the callback is due for execution.
         * @note Uses monotonic time from QURT platform timer
         */
        uint64_t next_usec;
    } *callbacks; ///< Head of linked list of registered callbacks
    
    /**
     * @brief Priority level for the bus thread
     * @note Should be lower than APM_MAIN_PRIORITY but higher than background tasks
     */
    AP_HAL::Scheduler::priority_base thread_priority;
    
    /**
     * @brief Flag indicating whether bus thread has been started
     * @note Set to true when first callback is registered and thread is spawned
     */
    bool thread_started;
    
    /**
     * @brief Pointer to HAL device object
     * @note Stored during callback registration but usage depends on implementation
     */
    AP_HAL::Device *hal_device;
};

}


