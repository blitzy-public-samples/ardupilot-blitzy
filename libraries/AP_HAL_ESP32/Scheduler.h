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
 * @file Scheduler.h
 * @brief ESP32 FreeRTOS-based scheduler implementation for ArduPilot HAL
 * 
 * @details This file implements the ArduPilot Hardware Abstraction Layer (HAL) scheduler
 *          interface specifically for the ESP32 platform using FreeRTOS as the underlying
 *          real-time operating system. The implementation creates a multi-threaded task
 *          architecture where different subsystems (main loop, timer callbacks, I/O operations,
 *          RC input/output, UART, storage) run as separate FreeRTOS tasks with carefully
 *          tuned priorities and stack sizes.
 * 
 *          Key architectural features:
 *          - FreeRTOS task-based threading model with priority inheritance
 *          - Dual-core CPU support with task affinity configuration
 *          - Task Watchdog Timer (TWDT) integration for deadlock detection
 *          - Semaphore-based synchronization for timer and I/O callbacks
 *          - Configurable task priorities optimized for IMU sensor types
 *          - Memory-constrained stack sizing for ESP32 SRAM limitations
 * 
 * @note This scheduler is designed for ESP32 dual-core architecture (APP_CPU and PRO_CPU)
 *       and leverages FreeRTOS configMAX_PRIORITIES=25 configuration.
 * 
 * @warning ESP32 has limited IRAM (instruction RAM) and timing constraints. High-priority
 *          ISR-safe operations must execute from IRAM with minimal latency.
 * 
 * @see AP_HAL::Scheduler for the base class interface definition
 * @see libraries/AP_HAL/Scheduler.h for platform-independent scheduler interface
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once
#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>
#include "HAL_ESP32_Namespace.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/** @brief Maximum number of timer callback procedures that can be registered */
#define ESP32_SCHEDULER_MAX_TIMER_PROCS 10

/** @brief Maximum number of I/O callback procedures that can be registered */
#define ESP32_SCHEDULER_MAX_IO_PROCS 10

/**
 * @brief Task Watchdog Timer (TWDT) timeout in milliseconds
 * 
 * @details This timeout defines how long a task can run without resetting the watchdog
 *          before the TWDT triggers a system reset. Set to 3000ms (3 seconds) to allow
 *          for long-running operations while still detecting genuine deadlocks.
 * 
 * @note Lower values provide faster deadlock detection but may cause false positives
 *       during legitimate long operations (e.g., filesystem operations, initialization).
 */
#define TWDT_TIMEOUT_MS 3000

/**
 * @class ESP32::Scheduler
 * @brief FreeRTOS-based multi-threaded scheduler implementation for ESP32 platform
 * 
 * @details This class implements the ArduPilot scheduler interface using FreeRTOS tasks
 *          to provide concurrent execution of multiple subsystems on the ESP32's dual-core
 *          architecture. The scheduler creates dedicated FreeRTOS tasks for:
 *          - Main loop execution (runs vehicle-specific code at configured loop rate)
 *          - Timer callbacks (high-priority periodic tasks like sensor reads)
 *          - I/O operations (lower-priority asynchronous operations)
 *          - RC input processing (radio control input decoding)
 *          - RC output processing (servo/motor output generation)
 *          - UART communication (serial port handling for telemetry/GPS)
 *          - Storage operations (parameter and log file system access)
 * 
 *          Thread Safety:
 *          - Uses FreeRTOS semaphores for callback synchronization
 *          - Provides in_main_thread() check for thread-safe API usage
 *          - Timer and I/O callbacks protected by semaphores
 * 
 *          Priority Architecture:
 *          - Priorities tuned for SPI-based IMU as primary sensor (SPI_PRIORITY=24)
 *          - Main loop at highest priority (MAIN_PRIO=24) for deterministic execution
 *          - Timer callbacks at near-highest priority (TIMER_PRIO=23) for sensor reads
 *          - I2C and peripheral tasks at lower priorities to avoid blocking critical paths
 * 
 *          CPU Core Affinity:
 *          - Main loop and scheduler tasks typically pinned to PRO_CPU (core 0)
 *          - WiFi and network tasks typically pinned to APP_CPU (core 1)
 *          - UART tasks on APP_CPU to avoid interfering with main loop
 * 
 *          Watchdog Integration:
 *          - Task Watchdog Timer (TWDT) monitors all long-running tasks
 *          - 3-second timeout prevents system hangs from deadlocks
 *          - Core mask configured to monitor specific CPU cores
 * 
 * @note ESP32 has limited RAM (520KB SRAM total, split between DRAM and IRAM).
 *       Stack sizes are carefully tuned to balance functionality with memory constraints.
 * 
 * @warning Modifying task priorities can severely affect system stability and timing.
 *          Higher priority tasks can starve lower priority tasks. The main loop must
 *          remain at high priority to ensure deterministic vehicle control.
 * 
 * @warning ESP32 IRAM is limited (~128KB) and used for interrupt handlers and time-critical
 *          code. Functions marked IRAM_ATTR must be small and execute quickly.
 * 
 * @see AP_HAL::Scheduler for base class interface and method contracts
 * @see libraries/AP_HAL_ChibiOS/Scheduler.h for ARM ChibiOS scheduler comparison
 * @see FreeRTOS documentation for task management and priority details
 */
class ESP32::Scheduler : public AP_HAL::Scheduler
{

public:
    /**
     * @brief Construct a new ESP32 Scheduler object
     * 
     * @details Initializes the scheduler state, sets callback counters to zero,
     *          and prepares for FreeRTOS task creation during init().
     */
    Scheduler();
    
    /**
     * @brief Destroy the Scheduler object
     * 
     * @details Cleans up scheduler resources. Note that FreeRTOS tasks are not
     *          explicitly deleted as the scheduler is typically never destroyed
     *          during normal operation (scheduler lives for entire system lifetime).
     */
    ~Scheduler();
    
    /* AP_HAL::Scheduler methods */
    
    /**
     * @brief Initialize the scheduler and create all FreeRTOS tasks
     * 
     * @details This method performs the following initialization sequence:
     *          1. Configures the Task Watchdog Timer (TWDT) with timeout and core mask
     *          2. Creates main loop task (_main_thread) at MAIN_PRIO priority
     *          3. Creates timer callback task (_timer_thread) at TIMER_PRIO priority
     *          4. Creates I/O callback task (_io_thread) at IO_PRIO priority
     *          5. Creates RC input task (_rcin_thread) at RCIN_PRIO priority
     *          6. Creates RC output task (_rcout_thread) at RCOUT_PRIO priority
     *          7. Creates UART task (_uart_thread) at UART_PRIO priority
     *          8. Creates storage task (_storage_thread) at STORAGE_PRIO priority
     *          9. Stores task handles for watchdog monitoring and debugging
     * 
     *          Each task is created with its corresponding stack size constant (e.g.,
     *          MAIN_SS, TIMER_SS) to fit within ESP32 memory constraints.
     * 
     * @note This method must be called before any other scheduler operations.
     * @note Task creation uses xTaskCreatePinnedToCore() for CPU core affinity control.
     * 
     * @warning This method assumes FreeRTOS is already initialized by ESP-IDF startup code.
     * 
     * @see wdt_init() for watchdog timer configuration
     */
    void     init() override;
    
    /**
     * @brief Set the HAL callback pointers for setup() and loop()
     * 
     * @details Stores the callback structure containing the vehicle's setup() and loop()
     *          functions. These callbacks are invoked by the main thread after initialization.
     * 
     * @param[in] cb Pointer to HAL::Callbacks structure with setup and loop function pointers
     * 
     * @note This must be called before init() to ensure callbacks are available when
     *       the main thread starts executing.
     */
    void     set_callbacks(AP_HAL::HAL::Callbacks *cb)
    {
        callbacks = cb;
    };
    
    /**
     * @brief Block execution for specified milliseconds
     * 
     * @details Implements a blocking delay using FreeRTOS vTaskDelay(). This yields the
     *          CPU to other tasks during the delay period, making it more efficient than
     *          busy-waiting. The actual delay may be slightly longer due to task scheduling.
     * 
     * @param[in] ms Delay duration in milliseconds
     * 
     * @note Internally converts milliseconds to FreeRTOS ticks using pdMS_TO_TICKS().
     * @note Minimum delay resolution depends on FreeRTOS tick rate (typically 1000 Hz = 1ms).
     * 
     * @warning Do not use for precise microsecond-level timing. Use delay_microseconds()
     *          or hardware timers for sub-millisecond accuracy.
     * 
     * @see delay_microseconds() for microsecond-precision delays
     */
    void     delay(uint16_t ms) override;
    
    /**
     * @brief Block execution for specified microseconds
     * 
     * @details Implements a microsecond-precision delay. For very short delays (<1000us),
     *          uses busy-waiting via ets_delay_us() for accuracy. For longer delays,
     *          may delegate to FreeRTOS task delay to avoid blocking other tasks.
     * 
     * @param[in] us Delay duration in microseconds
     * 
     * @note For delays <1000us, this busy-waits and prevents other tasks from running.
     * @note For delays >=1000us, may use vTaskDelay() to yield CPU (implementation dependent).
     * 
     * @warning Short microsecond delays consume CPU cycles without yielding. Use sparingly
     *          in high-priority tasks to avoid starving lower-priority tasks.
     * 
     * @see delay() for millisecond-precision delays that yield CPU
     */
    void     delay_microseconds(uint16_t us) override;
    
    /**
     * @brief Register a timer callback to be called at fast loop rate
     * 
     * @details Adds a member function callback to the timer callback array. Timer callbacks
     *          are executed by the dedicated timer thread (_timer_thread) at high priority
     *          (TIMER_PRIO=23). These callbacks are intended for time-critical operations
     *          like sensor reads that must occur at regular, frequent intervals.
     * 
     * @param[in] proc Member function pointer (AP_HAL::MemberProc) to register as timer callback
     * 
     * @note Maximum of ESP32_SCHEDULER_MAX_TIMER_PROCS (10) timer callbacks can be registered.
     * @note Timer callbacks execute sequentially within the timer thread in registration order.
     * @note If maximum callbacks exceeded, the registration is silently ignored (no error).
     * 
     * @warning Timer callbacks must execute quickly (<1ms typical) to maintain loop rate.
     *          Long-running operations will delay subsequent timer callbacks and degrade
     *          system timing performance.
     * 
     * @see register_io_process() for lower-priority asynchronous callbacks
     * @see _run_timers() for timer callback execution implementation
     */
    void     register_timer_process(AP_HAL::MemberProc) override;
    
    /**
     * @brief Register an I/O callback to be called at slower rate
     * 
     * @details Adds a member function callback to the I/O callback array. I/O callbacks
     *          are executed by the dedicated I/O thread (_io_thread) at lower priority
     *          (IO_PRIO=5). These callbacks are intended for less time-critical operations
     *          like logging, LED updates, or periodic housekeeping tasks.
     * 
     * @param[in] proc Member function pointer (AP_HAL::MemberProc) to register as I/O callback
     * 
     * @note Maximum of ESP32_SCHEDULER_MAX_IO_PROCS (10) I/O callbacks can be registered.
     * @note I/O callbacks execute sequentially within the I/O thread in registration order.
     * @note If maximum callbacks exceeded, the registration is silently ignored (no error).
     * 
     * @warning I/O callbacks run at low priority and may be delayed by higher priority tasks.
     *          Do not use for time-critical operations or operations requiring deterministic timing.
     * 
     * @see register_timer_process() for high-priority time-critical callbacks
     * @see _run_io() for I/O callback execution implementation
     */
    void     register_io_process(AP_HAL::MemberProc) override;
    
    /**
     * @brief Register a failsafe callback with specified period
     * 
     * @details Registers a function to be called periodically as a failsafe check. The failsafe
     *          callback is typically used to detect loss of control input or other critical
     *          failures that require immediate safety action.
     * 
     * @param[in] proc Function pointer (AP_HAL::Proc) to call as failsafe callback
     * @param[in] period_us Period in microseconds between failsafe callback invocations
     * 
     * @note Only one failsafe callback can be registered (subsequent calls override previous).
     * @note The period_us parameter may be used to configure failsafe timing or may be advisory.
     * 
     * @warning Failsafe callbacks must execute extremely quickly as they may be called from
     *          high-priority contexts. Blocking or slow operations can cause system instability.
     */
    void     register_timer_failsafe(AP_HAL::Proc, uint32_t period_us) override;
    
    /**
     * @brief Reboot the ESP32 system
     * 
     * @details Performs a software reset of the ESP32. The hold_in_bootloader parameter
     *          controls whether the system should enter bootloader mode after reset for
     *          firmware update purposes.
     * 
     * @param[in] hold_in_bootloader If true, reboot into bootloader mode for firmware update;
     *                                if false, perform normal reboot into application
     * 
     * @note This method does not return - execution halts and system resets.
     * @note Uses ESP-IDF esp_restart() or bootloader entry functions.
     * 
     * @warning All unsaved data will be lost. Ensure critical data is flushed before calling.
     */
    void     reboot(bool hold_in_bootloader) override;
    
    /**
     * @brief Check if code is executing in the main thread context
     * 
     * @details Returns true if the calling code is executing within the main loop task
     *          (_main_thread), false otherwise. This is used to enforce thread-safety
     *          requirements where certain APIs must only be called from the main thread.
     * 
     * @return true if executing in main thread (_main_thread task), false otherwise
     * 
     * @note Comparison is performed by checking current FreeRTOS task handle against
     *       the stored _main_task_handle.
     * 
     * @warning Many ArduPilot APIs assume main thread context and will assert or fail
     *          if called from timer/IO callbacks. Always check before calling such APIs.
     * 
     * @see xTaskGetCurrentTaskHandle() for FreeRTOS task handle retrieval
     */
    bool     in_main_thread() const override;
    
    /**
     * @brief Mark the system as fully initialized
     * 
     * @details Sets the internal _initialized flag to true, indicating that all subsystems
     *          have completed initialization and the system is ready for normal operation.
     *          This is typically called after setup() completes successfully.
     * 
     * @note This affects behavior of is_system_initialized() checks throughout the codebase.
     */
    void     set_system_initialized() override;
    
    /**
     * @brief Check if system initialization is complete
     * 
     * @details Returns the status of the _initialized flag. Many operations are deferred
     *          until system initialization is complete to avoid accessing uninitialized
     *          subsystems or hardware.
     * 
     * @return true if set_system_initialized() has been called, false otherwise
     * 
     * @note During startup, this returns false until the HAL setup() callback completes.
     */
    bool     is_system_initialized() override;

    /**
     * @brief Print scheduler performance statistics to console
     * 
     * @details Outputs diagnostic information about scheduler operation including task
     *          execution times, loop rates, and timing performance. Useful for debugging
     *          scheduler behavior and identifying timing issues or task starvation.
     * 
     * @note Output format and destination depend on console configuration.
     * @note This is typically called via MAVLink command or debug console.
     */
    void     print_stats(void) ;
    
    /**
     * @brief Print the current main loop execution rate to console
     * 
     * @details Displays the actual measured rate at which the main loop is executing,
     *          which may differ from the configured _loop_rate_hz if the system cannot
     *          maintain the target rate due to CPU load or blocking operations.
     * 
     * @note Compares _active_loop_rate_hz (measured) against _loop_rate_hz (configured).
     */
    void     print_main_loop_rate(void);

    /**
     * @brief Get the configured main loop rate in Hz
     * 
     * @details Returns the target frequency at which the main vehicle loop should execute.
     *          This is used for scheduling and timing calculations throughout the system.
     * 
     * @return uint16_t Loop rate in Hz (cycles per second)
     * 
     * @note Typical values: 400 Hz for multicopters, 50-100 Hz for planes/rovers.
     * @note Actual loop rate may be lower if system cannot maintain target rate.
     * 
     * @see _loop_rate_hz parameter for configured value
     * @see _active_loop_rate_hz for measured actual rate
     */
    uint16_t get_loop_rate_hz(void);
    
    /**
     * @brief Currently measured main loop execution rate in Hz
     * 
     * @details This parameter reflects the actual measured frequency of main loop execution.
     *          If this value is significantly lower than _loop_rate_hz, it indicates the
     *          system is overloaded and cannot maintain the target loop rate.
     * 
     * @note Updated dynamically based on loop timing measurements.
     * @note Persistent parameter stored in AP_Param system.
     */
    AP_Int16 _active_loop_rate_hz;
    
    /**
     * @brief Configured target main loop rate in Hz
     * 
     * @details This parameter sets the desired frequency for main loop execution. The
     *          scheduler attempts to call the vehicle loop() callback at this rate.
     *          Higher rates provide better control response but increase CPU load.
     * 
     * @note Configurable via parameter system (e.g., SCHED_LOOP_RATE).
     * @note Persistent parameter stored in AP_Param system.
     * @note Common values: 400 Hz (Copter), 50 Hz (Plane), 50 Hz (Rover).
     */
    AP_Int16 _loop_rate_hz;

    /**
     * @brief Trampoline function for thread creation
     * 
     * @details This static function serves as the entry point for dynamically created
     *          FreeRTOS tasks. It receives a context pointer containing the actual
     *          member function (AP_HAL::MemberProc) to execute, extracts it, and
     *          invokes the member function in the new task context.
     * 
     * @param[in] ctx Context pointer containing the AP_HAL::MemberProc to execute
     * 
     * @note Required because FreeRTOS task functions must be static or free functions,
     *       but ArduPilot uses member function pointers for callbacks.
     * @note The context is typically allocated on heap and freed after extraction.
     * 
     * @warning The ctx pointer must remain valid until the task starts and extracts the proc.
     * 
     * @see thread_create() for task creation with this trampoline
     */
    static void thread_create_trampoline(void *ctx);
    
    /**
     * @brief Create a new FreeRTOS task to execute a member function
     * 
     * @details Creates a new FreeRTOS task that will execute the specified member function
     *          callback. The task is created with the specified stack size and priority,
     *          with priority adjusted based on the priority_base and offset. This allows
     *          dynamic task creation for vehicle-specific threads or library operations.
     * 
     * @param[in] proc        Member function pointer to execute in the new task
     * @param[in] name        Human-readable name for the task (for debugging)
     * @param[in] stack_size  Stack size in bytes for the task
     * @param[in] base        Priority base (DRIVER, STORAGE, etc.) for priority calculation
     * @param[in] priority    Priority offset to add to base priority
     * 
     * @return true if task created successfully, false if creation failed
     * 
     * @note Uses thread_create_trampoline() as the FreeRTOS task entry point.
     * @note Task handle is not stored, so created tasks cannot be monitored via this class.
     * @note Priority calculation: final_priority = base_priority + priority_offset
     * 
     * @warning Stack size must be sufficient for the task's operations. ESP32 stack overflow
     *          causes crashes that are difficult to debug. Add margin for interrupt context.
     * 
     * @warning Excessive task creation can exhaust RAM. ESP32 has limited memory (~300KB
     *          usable after heap/stacks/buffers). Each task consumes its stack_size + overhead.
     * 
     * @see thread_create_trampoline() for task entry point implementation
     */
    bool thread_create(AP_HAL::MemberProc, const char *name, uint32_t stack_size, priority_base base, int8_t priority) override;

    /*
     * Legacy priority configuration (commented out for reference):
     * Previous priority values were lower and less optimized for dual-core architecture.
     */
    /*static const int SPI_PRIORITY = 40; // if your primary imu is spi, this should be above the i2c value, spi is better.
    static const int MAIN_PRIO = 15;
    static const int I2C_PRIORITY = 8; // if your primary imu is i2c, this should be above the spi value, i2c is not preferred.
    static const int TIMER_PRIO = 15;
    static const int RCIN_PRIO = 15;
    static const int RCOUT_PRIO = 15;
    static const int WIFI_PRIO = 10;
    static const int UART_PRIO = 8;
    static const int IO_PRIO = 6;
    static const int STORAGE_PRIO = 6; */

    /**
     * @brief FreeRTOS Priority Configuration for ESP32 Scheduler
     * 
     * @details Priority constants for FreeRTOS tasks. ESP32 FreeRTOS is configured with
     *          configMAX_PRIORITIES=25, meaning valid priority values are 0-24, where
     *          higher numbers indicate higher priority. Priority 24 is the highest
     *          non-ISR priority, and priority 0 is the idle task priority.
     * 
     *          Priority Architecture Rationale:
     *          - SPI_PRIORITY (24) and MAIN_PRIO (24) are highest to ensure deterministic
     *            sensor reads and main loop execution for flight control
     *          - TIMER_PRIO (23) is slightly lower to prevent starvation of WiFi/networking
     *          - UART_PRIO (23) is high to prevent telemetry packet loss but not max priority
     *          - WIFI_PRIO1 (20) provides good network throughput without blocking control
     *          - Mid-range priorities (10-12) for secondary communication and RC output
     *          - Low priorities (4-5) for non-critical I/O, storage, and I2C peripherals
     * 
     *          CPU Core Affinity Considerations:
     *          - PRO_CPU (CPU0): Main loop, timer callbacks, SPI sensors (deterministic control)
     *          - APP_CPU (CPU1): WiFi, UART, networking (offload communications from control)
     * 
     * @note FreeRTOS uses priority inheritance to prevent priority inversion when tasks
     *       wait on mutexes held by lower-priority tasks.
     * 
     * @warning Modifying these priorities can severely affect system stability. Higher priority
     *          tasks can completely starve lower priority tasks. The WiFi stack requires
     *          sufficient CPU time or it will fail/reset. Balance control loop determinism
     *          with communication throughput when tuning priorities.
     */
      // configMAX_PRIORITIES=25

    /**
     * @brief SPI bus task priority (24 - highest application priority)
     * 
     * @note SPI is the preferred interface for IMU sensors due to lower latency and higher
     *       reliability compared to I2C. If the primary IMU is SPI-based, this ensures
     *       sensor reads occur with minimal latency and jitter.
     * @note Typically runs on PRO_CPU (core 0) for deterministic timing.
     */
    static const int SPI_PRIORITY = 24;
    
    /**
     * @brief Main loop task priority (24 - highest application priority)
     * 
     * @note Set to maximum priority to ensure the vehicle control loop runs deterministically
     *       without preemption by other tasks (except ISRs). This is critical for stable
     *       flight control.
     * @note Runs on PRO_CPU (core 0) for deterministic execution.
     */
    static const int MAIN_PRIO    = 24;
    
    /**
     * @brief I2C bus task priority (5 - low priority)
     * 
     * @note I2C is slower and less reliable than SPI for IMU sensors. Set to low priority
     *       to avoid blocking higher-priority SPI-based sensor reads.
     * @note If the primary IMU is I2C-based (not recommended), consider increasing this
     *       priority above SPI_PRIORITY, though SPI remains the preferred interface.
     */
    static const int I2C_PRIORITY = 5;
    
    /**
     * @brief Timer callback task priority (23 - near-highest priority)
     * 
     * @note Set to 23 (not 24) to remain slightly below main loop priority. This ensures
     *       the main loop can preempt timer callbacks if needed, and prevents timer tasks
     *       from completely starving network/WiFi processing.
     * @note Runs on PRO_CPU (core 0) alongside main loop for timing consistency.
     */
    static const int TIMER_PRIO   = 23;
    
    /**
     * @brief RC input task priority (5 - low priority)
     * 
     * @note Radio control input processing is relatively infrequent (50-100 Hz typical)
     *       and can tolerate some latency without affecting control quality. Set to low
     *       priority to avoid interfering with sensor reads and main loop execution.
     */
    static const int RCIN_PRIO    = 5;
    
    /**
     * @brief RC output task priority (10 - medium priority)
     * 
     * @note Servo/motor output generation requires moderate priority to ensure outputs
     *       update regularly, but not so high as to interfere with sensor reads or
     *       main loop execution. Medium priority provides balance.
     */
    static const int RCOUT_PRIO   = 10;
    
    /**
     * @brief WiFi task 1 priority (20 - high priority for network)
     * 
     * @note WiFi requires relatively high priority to maintain network stack responsiveness
     *       and prevent buffer overflows/underflows in the WiFi driver. However, priority
     *       remains below control loop priorities to avoid disrupting flight control.
     * @note Runs on APP_CPU (core 1) to isolate network processing from control loops.
     */
    static const int WIFI_PRIO1   = 20;
    
    /**
     * @brief WiFi task 2 priority (12 - medium-high priority)
     * 
     * @note Secondary WiFi task for additional network processing. Lower priority than
     *       WIFI_PRIO1 but still elevated to ensure good network throughput.
     * @note Runs on APP_CPU (core 1).
     */
    static const int WIFI_PRIO2   = 12;
    
    /**
     * @brief UART communication task priority (23 - near-highest priority)
     * 
     * @note UART priority is high (23, not 24) to prevent telemetry packet loss and ensure
     *       GPS/radio data is processed promptly. However, set below 24 to avoid completely
     *       blocking the main loop/scheduler. UART feeds data to WiFi subsystem in
     *       _writebuf/_readbuf, so adequate UART priority helps maintain WiFi throughput.
     * @note Runs on APP_CPU (core 1) to offload communication from control CPU.
     */
    static const int UART_PRIO    = 23;
    
    /**
     * @brief I/O callback task priority (5 - low priority)
     * 
     * @note I/O callbacks are for non-time-critical operations like LED updates, logging,
     *       and periodic housekeeping. Low priority ensures they don't interfere with
     *       control loops or high-priority sensor/communication tasks.
     */
    static const int IO_PRIO      = 5;
    
    /**
     * @brief Storage task priority (4 - lowest application priority)
     * 
     * @note Storage operations (parameter saves, log writes to SD card/flash) are the
     *       lowest priority tasks. File system operations can be slow and blocking, so
     *       keeping them at lowest priority prevents them from disrupting more critical
     *       operations.
     * @note Storage operations may be significantly delayed if system is under heavy load.
     */
    static const int STORAGE_PRIO = 4;

    /**
     * @brief FreeRTOS Task Stack Size Configuration for ESP32
     * 
     * @details Stack size constants in bytes for each FreeRTOS task. ESP32 has limited
     *          SRAM (~520KB total, with ~300KB typically available for application after
     *          system reserves, heap, and buffers). Stack sizes are carefully tuned to:
     *          1. Provide sufficient stack for task operations and interrupt context
     *          2. Minimize total RAM consumption to fit within ESP32 constraints
     *          3. Include safety margin to prevent stack overflow crashes
     * 
     *          Stack Sizing Methodology:
     *          - Measured actual stack usage during typical operation
     *          - Added 30-50% margin for interrupt nesting and edge cases
     *          - Adjusted based on empirical testing and crash analysis
     *          - Total stack allocation across all tasks: ~30KB
     * 
     * @warning Stack overflow causes immediate crash or memory corruption that is extremely
     *          difficult to debug. If experiencing random crashes, enable FreeRTOS stack
     *          overflow checking (CONFIG_FREERTOS_CHECK_STACKOVERFLOW) and increase stack
     *          sizes if overflows detected.
     * 
     * @warning ESP32 stack must accommodate both task context AND interrupt context. ISRs
     *          execute on the stack of the interrupted task, so high-priority tasks with
     *          frequent interrupts need larger stacks.
     * 
     * @note Stack sizes are specified in bytes. FreeRTOS internally may use words (4 bytes
     *       on ESP32), but these constants are in bytes for clarity.
     */

    /**
     * @brief Timer callback task stack size (3 KB)
     * 
     * @note Timer callbacks execute sensor reads and time-critical processing. 3KB provides
     *       sufficient space for sensor driver calls, data processing, and interrupt context.
     */
    static const int TIMER_SS     = 1024*3;
    
    /**
     * @brief Main loop task stack size (5 KB)
     * 
     * @note Main loop executes vehicle-specific control code and calls numerous subsystems.
     *       5KB is the largest stack allocation, reflecting the complexity of operations
     *       performed in the main loop. This stack must accommodate the entire call chain
     *       from loop() through all control and navigation functions.
     * 
     * @warning Insufficient main loop stack can cause crashes during complex operations
     *          like mission execution, parameter loads, or failsafe activations. Monitor
     *          stack high water mark during testing.
     */
    static const int MAIN_SS      = 1024*5;
    
    /**
     * @brief RC input task stack size (3 KB)
     * 
     * @note RC input decoding involves protocol parsing and signal validation. 3KB provides
     *       adequate space for protocol state machines and data buffers.
     */
    static const int RCIN_SS      = 1024*3;
    
    /**
     * @brief RC output task stack size (1.5 KB)
     * 
     * @note RC output is relatively simple - converts servo/motor values to PWM signals.
     *       1.5KB is sufficient for this straightforward task.
     */
    static const int RCOUT_SS     = 1024*1.5;
    
    /**
     * @brief WiFi task 1 stack size (2.25 KB)
     * 
     * @note WiFi tasks handle network stack operations. 2.25KB accommodates network buffer
     *       handling and TCP/IP stack calls without excessive memory consumption.
     * @note WiFi stack is provided by ESP-IDF and has its own memory allocation; these
     *       tasks primarily move data between WiFi buffers and application.
     */
    static const int WIFI_SS1     = 1024*2.25;
    
    /**
     * @brief WiFi task 2 stack size (2.25 KB)
     * 
     * @note Secondary WiFi task with same stack sizing as WIFI_SS1.
     */
    static const int WIFI_SS2     = 1024*2.25;
    
    /**
     * @brief UART communication task stack size (2.25 KB)
     * 
     * @note UART tasks handle serial I/O for telemetry, GPS, and peripherals. 2.25KB provides
     *       space for protocol parsing (MAVLink, NMEA, etc.) and buffer management.
     */
    static const int UART_SS      = 1024*2.25;
    
    /**
     * @brief Device bus (SPI/I2C) task stack size (4 KB)
     * 
     * @note Device bus tasks manage sensor communication. 4KB accommodates SPI/I2C driver
     *       calls, sensor initialization sequences, and data processing. This is the second
     *       largest stack after main loop due to complexity of device driver operations.
     */
    static const int DEVICE_SS    = 1024*4;     // DEVICEBUS/s
    
    /**
     * @brief I/O callback task stack size (3.5 KB)
     * 
     * @note I/O callbacks perform diverse operations (logging, LED control, housekeeping).
     *       3.5KB provides flexibility for various callback types without excessive allocation.
     */
    static const int IO_SS        = 1024*3.5;   // APM_IO
    
    /**
     * @brief Storage task stack size (2 KB)
     * 
     * @note Storage operations access file system for parameters and logs. 2KB is sufficient
     *       for file system API calls (FAT, LittleFS) and small buffers. Large data transfers
     *       use DMA or separate heap allocations rather than stack buffers.
     */
    static const int STORAGE_SS   = 1024*2;     // APM_STORAGE

private:
    /**
     * @brief Pointer to HAL callback structure containing setup() and loop() functions
     * 
     * @details Set via set_callbacks() and invoked by _main_thread() after initialization.
     */
    AP_HAL::HAL::Callbacks *callbacks;
    
    /**
     * @brief Failsafe callback function pointer
     * 
     * @details Registered via register_timer_failsafe() and called periodically to check
     *          for critical failures requiring safety action.
     */
    AP_HAL::Proc _failsafe;

    /**
     * @brief Array of registered timer callback functions
     * 
     * @details Stores member function pointers registered via register_timer_process().
     *          Maximum of ESP32_SCHEDULER_MAX_TIMER_PROCS (10) callbacks supported.
     *          Callbacks executed sequentially by _timer_thread() at high priority.
     * 
     * @see register_timer_process() for registration
     * @see _run_timers() for execution
     */
    AP_HAL::MemberProc _timer_proc[ESP32_SCHEDULER_MAX_TIMER_PROCS];
    
    /**
     * @brief Number of registered timer callbacks
     * 
     * @details Count of callbacks in _timer_proc array. Used to iterate callbacks in _run_timers().
     */
    uint8_t _num_timer_procs;

    /**
     * @brief Array of registered I/O callback functions
     * 
     * @details Stores member function pointers registered via register_io_process().
     *          Maximum of ESP32_SCHEDULER_MAX_IO_PROCS (10) callbacks supported.
     *          Callbacks executed sequentially by _io_thread() at low priority.
     * 
     * @see register_io_process() for registration
     * @see _run_io() for execution
     */
    AP_HAL::MemberProc _io_proc[ESP32_SCHEDULER_MAX_IO_PROCS];
    
    /**
     * @brief Number of registered I/O callbacks
     * 
     * @details Count of callbacks in _io_proc array. Used to iterate callbacks in _run_io().
     */
    uint8_t _num_io_procs;

    /**
     * @brief System initialization status flag
     * 
     * @details Set to true by set_system_initialized() after setup() completes.
     *          Checked by is_system_initialized() throughout system to defer operations
     *          until initialization is complete.
     * 
     * @note Static to allow checking initialization status from static methods.
     */
    static bool _initialized;

    /**
     * @brief FreeRTOS task handle for main loop task
     * 
     * @details Handle for the task running _main_thread(). Used for task identification
     *          in in_main_thread() and for watchdog registration.
     * 
     * @see _main_thread() for task implementation
     */
    tskTaskControlBlock* _main_task_handle;
    
    /**
     * @brief FreeRTOS task handle for timer callback task
     * 
     * @details Handle for the task running _timer_thread(). Used for watchdog registration
     *          and debugging.
     * 
     * @see _timer_thread() for task implementation
     */
    tskTaskControlBlock* _timer_task_handle;
    
    /**
     * @brief FreeRTOS task handle for RC input task
     * 
     * @details Handle for the task running _rcin_thread(). Used for watchdog registration.
     * 
     * @see _rcin_thread() for task implementation
     */
    tskTaskControlBlock* _rcin_task_handle;
    
    /**
     * @brief FreeRTOS task handle for RC output task
     * 
     * @details Handle for the task running _rcout_thread(). Used for watchdog registration.
     * 
     * @see _rcout_thread() for task implementation
     */
    tskTaskControlBlock* _rcout_task_handle;
    
    /**
     * @brief FreeRTOS task handle for UART communication task
     * 
     * @details Handle for the task running _uart_thread(). Used for watchdog registration.
     * 
     * @see _uart_thread() for task implementation
     */
    tskTaskControlBlock* _uart_task_handle;
    
    /**
     * @brief FreeRTOS task handle for I/O callback task
     * 
     * @details Handle for the task running _io_thread(). Used for watchdog registration.
     * 
     * @see _io_thread() for task implementation
     */
    tskTaskControlBlock* _io_task_handle;
    
    /**
     * @brief FreeRTOS task handle for test/debug task
     * 
     * @details Handle for optional test task. May be used for development/debugging purposes.
     * 
     * @note Usage depends on build configuration and may not be active in production builds.
     */
    tskTaskControlBlock* test_task_handle;
    
    /**
     * @brief FreeRTOS task handle for storage task
     * 
     * @details Handle for the task running _storage_thread(). Used for watchdog registration.
     * 
     * @see _storage_thread() for task implementation
     */
    tskTaskControlBlock* _storage_task_handle;

    /**
     * @brief Main loop thread entry point
     * 
     * @details Static function that serves as the FreeRTOS task entry point for the main
     *          vehicle loop. Calls callbacks->setup() for initialization, then repeatedly
     *          calls callbacks->loop() at the configured loop rate (_loop_rate_hz).
     *          Implements loop rate control and timing measurement.
     * 
     * @param[in] arg Context pointer (typically points to Scheduler instance)
     * 
     * @note Runs at MAIN_PRIO (24) priority on PRO_CPU (core 0).
     * @note This task never exits - runs for entire system lifetime.
     * 
     * @see MAIN_PRIO for priority configuration
     * @see MAIN_SS for stack size configuration
     */
    static void _main_thread(void *arg);
    
    /**
     * @brief Timer callback thread entry point
     * 
     * @details Static function that executes registered timer callbacks at high priority.
     *          Waits on _timer_sem semaphore, then calls _run_timers() to execute all
     *          registered timer callbacks sequentially. Intended for time-critical operations
     *          like sensor reads that must occur at precise intervals.
     * 
     * @param[in] arg Context pointer (typically points to Scheduler instance)
     * 
     * @note Runs at TIMER_PRIO (23) priority on PRO_CPU (core 0).
     * @note This task never exits - runs for entire system lifetime.
     * 
     * @see TIMER_PRIO for priority configuration
     * @see TIMER_SS for stack size configuration
     * @see _run_timers() for callback execution
     */
    static void _timer_thread(void *arg);
    
    /**
     * @brief RC output thread entry point
     * 
     * @details Static function that generates RC output signals (PWM) for servos and motors.
     *          Calls into RCOutput HAL to generate output waveforms at configured frequencies.
     * 
     * @param[in] arg Context pointer (typically points to Scheduler instance)
     * 
     * @note Runs at RCOUT_PRIO (10) priority.
     * @note This task never exits - runs for entire system lifetime.
     * 
     * @see RCOUT_PRIO for priority configuration
     * @see RCOUT_SS for stack size configuration
     */
    static void _rcout_thread(void *arg);
    
    /**
     * @brief RC input thread entry point
     * 
     * @details Static function that processes incoming RC signals from radio receiver.
     *          Decodes RC protocols (SBUS, PPM, DSM, etc.) and updates RC channel values.
     * 
     * @param[in] arg Context pointer (typically points to Scheduler instance)
     * 
     * @note Runs at RCIN_PRIO (5) priority.
     * @note This task never exits - runs for entire system lifetime.
     * 
     * @see RCIN_PRIO for priority configuration
     * @see RCIN_SS for stack size configuration
     */
    static void _rcin_thread(void *arg);
    
    /**
     * @brief UART communication thread entry point
     * 
     * @details Static function that handles serial port communication for telemetry, GPS,
     *          and peripheral devices. Manages UART buffers and protocol processing.
     * 
     * @param[in] arg Context pointer (typically points to Scheduler instance)
     * 
     * @note Runs at UART_PRIO (23) priority on APP_CPU (core 1).
     * @note High priority to prevent telemetry packet loss and GPS data delays.
     * 
     * @see UART_PRIO for priority configuration
     * @see UART_SS for stack size configuration
     */
    static void _uart_thread(void *arg);
    
    /**
     * @brief I/O callback thread entry point
     * 
     * @details Static function that executes registered I/O callbacks at low priority.
     *          Waits on _io_sem semaphore, then calls _run_io() to execute all registered
     *          I/O callbacks sequentially. Intended for non-time-critical operations like
     *          logging, LED updates, and housekeeping tasks.
     * 
     * @param[in] arg Context pointer (typically points to Scheduler instance)
     * 
     * @note Runs at IO_PRIO (5) priority - lowest among active tasks.
     * @note This task never exits - runs for entire system lifetime.
     * 
     * @see IO_PRIO for priority configuration
     * @see IO_SS for stack size configuration
     * @see _run_io() for callback execution
     */
    static void _io_thread(void *arg);
    
    /**
     * @brief Storage thread entry point
     * 
     * @details Static function that handles file system operations for parameter storage
     *          and logging. Accesses FAT/LittleFS file systems on SD card or flash.
     * 
     * @param[in] arg Context pointer (typically points to Scheduler instance)
     * 
     * @note Runs at STORAGE_PRIO (4) priority - lowest priority due to potentially
     *       slow/blocking file system operations.
     * @note This task never exits - runs for entire system lifetime.
     * 
     * @see STORAGE_PRIO for priority configuration
     * @see STORAGE_SS for stack size configuration
     */
    static void _storage_thread(void *arg);

    /**
     * @brief Set position helper function (implementation specific)
     * 
     * @details Static utility function for setting position-related state. Usage depends
     *          on specific implementation requirements.
     * 
     * @param[in] arg Context pointer
     * 
     * @note Purpose and usage depend on ESP32-specific implementation details.
     */
    static void set_position(void* arg);

    /**
     * @brief Print profiling information helper function
     * 
     * @details Static function to output performance profiling data. Used for debugging
     *          and optimization to identify timing bottlenecks and task execution patterns.
     * 
     * @param[in] arg Context pointer
     * 
     * @note Output format and destination depend on implementation and debug configuration.
     */
    static void _print_profile(void* arg);

    /**
     * @brief ESC testing helper function
     * 
     * @details Static function for testing Electronic Speed Controllers (ESCs). Used during
     *          hardware validation and motor/ESC configuration.
     * 
     * @param[in] arg Context pointer
     * 
     * @warning ESC test functions can cause motors to spin. Use with extreme caution and
     *          only with propellers removed.
     * 
     * @note Usage typically limited to development/testing, not production operation.
     */
    static void test_esc(void* arg);

    /**
     * @brief Initialize the Task Watchdog Timer (TWDT)
     * 
     * @details Configures the ESP32 Task Watchdog Timer to monitor specified tasks and
     *          detect deadlocks. Tasks must periodically reset the watchdog to prove they
     *          are still executing. If a task fails to reset within the timeout period,
     *          the TWDT triggers a system reset to recover from the deadlock.
     * 
     * @param[in] timeout    Watchdog timeout in milliseconds (TWDT_TIMEOUT_MS = 3000)
     * @param[in] core_mask  Bitmask specifying which CPU cores to monitor (bit 0 = core 0, bit 1 = core 1)
     * 
     * @note Called during init() to enable watchdog monitoring.
     * @note Tasks are subscribed to the watchdog individually after creation.
     * 
     * @warning Watchdog timeout must be long enough to accommodate legitimate long operations
     *          (e.g., flash writes, file system operations) or false resets will occur.
     * 
     * @see TWDT_TIMEOUT_MS for timeout configuration
     */
    static void wdt_init(uint32_t timeout, uint32_t core_mask);

    /**
     * @brief Flag indicating timer callbacks are currently executing
     * 
     * @details Set to true when _run_timers() is executing, false otherwise. Used for
     *          detecting reentrant calls and debugging timing issues.
     */
    bool _in_timer_proc;
    
    /**
     * @brief Execute all registered timer callbacks
     * 
     * @details Iterates through the _timer_proc array and invokes each registered callback
     *          sequentially. Called by _timer_thread() at high priority (TIMER_PRIO=23).
     *          Sets _in_timer_proc flag during execution to detect reentrancy.
     * 
     * @note Callbacks execute sequentially in registration order, not concurrently.
     * @note Total execution time of all callbacks affects timer thread loop rate.
     * 
     * @warning Long-running callbacks delay subsequent callbacks and degrade timing performance.
     *          Keep timer callbacks short (<1ms typical) for best timing behavior.
     * 
     * @see register_timer_process() for callback registration
     * @see _timer_thread() for calling context
     */
    void _run_timers();
    
    /**
     * @brief Semaphore for timer callback synchronization
     * 
     * @details Used to signal _timer_thread() that timer callbacks should execute.
     *          The thread waits on this semaphore, and it is signaled when callbacks
     *          need to run (typically from main loop or interrupt context).
     * 
     * @see _timer_thread() for semaphore wait
     * @see _run_timers() for callback execution
     */
    Semaphore _timer_sem;

    /**
     * @brief Flag indicating I/O callbacks are currently executing
     * 
     * @details Set to true when _run_io() is executing, false otherwise. Used for
     *          detecting reentrant calls and debugging.
     */
    bool _in_io_proc;
    
    /**
     * @brief Execute all registered I/O callbacks
     * 
     * @details Iterates through the _io_proc array and invokes each registered callback
     *          sequentially. Called by _io_thread() at low priority (IO_PRIO=5).
     *          Sets _in_io_proc flag during execution to detect reentrancy.
     * 
     * @note Callbacks execute sequentially in registration order, not concurrently.
     * @note I/O callbacks run at low priority and may be preempted by higher priority tasks.
     * 
     * @see register_io_process() for callback registration
     * @see _io_thread() for calling context
     */
    void _run_io();
    
    /**
     * @brief Semaphore for I/O callback synchronization
     * 
     * @details Used to signal _io_thread() that I/O callbacks should execute.
     *          The thread waits on this semaphore, and it is signaled when callbacks
     *          need to run (typically from main loop at a slower rate than timer callbacks).
     * 
     * @see _io_thread() for semaphore wait
     * @see _run_io() for callback execution
     */
    Semaphore _io_sem;
};
