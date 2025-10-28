/**
 * @file HAL_Linux_Class.h
 * @brief Main Linux HAL class implementation aggregating all Linux-specific drivers
 * 
 * Defines HAL_Linux class that instantiates and manages all hardware abstraction
 * drivers for Linux-based platforms. This is the central point where Linux-specific
 * implementations of UART, GPIO, I2C, SPI, and other interfaces are created and
 * wired to the HAL interface.
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

#include <signal.h>

/**
 * @class HAL_Linux
 * @brief Linux platform implementation of ArduPilot HAL
 * 
 * @details Main HAL class for Linux-based flight controllers. Instantiates platform-specific
 *          drivers and manages the main execution loop. Unlike embedded HAL (ChibiOS), this
 *          implementation uses Linux kernel interfaces (sysfs, /dev files, POSIX threads)
 *          rather than direct hardware register access.
 *          
 *          Initialization sequence:
 *          1. Constructor creates driver instances (UART, GPIO, I2C, SPI, etc.)
 *          2. Drivers probe hardware during init phase
 *          3. run() method processes command line, sets up signals, calls callbacks
 *          4. Main loop executes via Linux scheduler with real-time priorities
 *          
 *          Driver instantiation:
 *          - UARTDriver: Linux serial devices (/dev/ttyS*, /dev/ttyUSB*)
 *          - GPIO: Board-specific implementation (sysfs, memory-mapped, or custom)
 *          - I2CDevice: Linux i2c-dev interface (/dev/i2c-N)
 *          - SPIDevice: Linux spidev interface (/dev/spidevX.Y)
 *          - Scheduler: POSIX thread-based with SCHED_FIFO real-time scheduling
 *          - Storage: File-based parameter storage
 *          - RCInput/Output: Board-specific implementations (sysfs PWM, PRU, PCA9685)
 *          
 *          Signal handling:
 *          - SIGTERM/SIGINT: Trigger clean shutdown via exit_signal_handler()
 *          - Sets _should_exit flag checked by main loop
 *          - Allows parameter saves and clean driver shutdown before exit
 *          
 *          Command-line arguments (Linux/SITL only):
 *          - -A <connection>: Primary telemetry (e.g., udp:192.168.1.100:14550)
 *          - -B <connection>: Secondary telemetry
 *          - -C <connection>: Tertiary telemetry
 *          - -D <connection>: Quaternary telemetry
 *          - --daemon: Run as background daemon
 *          - --log-directory <path>: Custom log directory
 *          
 *          Real-time performance:
 *          - Main loop thread runs with SCHED_FIFO priority (typically 15)
 *          - mlockall() locks all pages in RAM to prevent paging delays
 *          - CPU affinity can pin main loop to specific core
 *          - Typical jitter: 50-500μs depending on kernel configuration
 * 
 * @note Requires Linux kernel 3.10+ with appropriate modules (i2c-dev, spidev, gpio)
 * @note Real-time performance requires PREEMPT or PREEMPT_RT kernel patches
 * @warning Running without real-time scheduling may cause timing violations in flight
 * 
 * @see AP_HAL::HAL base class interface
 * @see Scheduler for thread and timing management
 * @see GPIO backends for board-specific pin access
 */
class HAL_Linux : public AP_HAL::HAL {
public:
    /**
     * @brief Constructor instantiates all Linux HAL drivers
     * 
     * @note Called once before main() via static initialization
     * @note Allocates driver objects but does not initialize hardware yet
     */
    HAL_Linux();
    
    /**
     * @brief Main execution entry point - never returns
     * 
     * @param[in] argc Command-line argument count
     * @param[in] argv Command-line argument array
     * @param[in] callbacks Firmware callbacks object (vehicle setup/loop)
     * 
     * @details Execution sequence:
     *          1. Parse command line arguments
     *          2. Set up signal handlers (SIGTERM, SIGINT)
     *          3. Initialize all HAL drivers (scheduler, GPIO, buses, etc.)
     *          4. Call callbacks->setup() for firmware initialization
     *          5. Enter infinite loop calling callbacks->loop()
     *          6. On signal, cleanup and exit
     *          
     *          Threading model:
     *          - Main thread runs firmware loop at configured rate (e.g., 400Hz)
     *          - Scheduler creates additional threads for I/O and timers
     *          - All threads use SCHED_FIFO real-time scheduling if running as root
     * 
     * @note Never returns except on fatal error or SIGTERM/SIGINT
     * @note Blocks until firmware loop completes or signal received
     * @warning Must run as root for real-time scheduling (SCHED_FIFO)
     */
    void run(int argc, char* const* argv, Callbacks* callbacks) const override;

    /**
     * @brief Configure POSIX signal handlers for graceful shutdown
     * 
     * @details Installs signal handlers for:
     *          - SIGTERM: System termination request
     *          - SIGINT: User interrupt (Ctrl-C)
     *          - SIGHUP: Hangup signal (terminal disconnect)
     *          
     *          Handler sets _should_exit flag checked by main loop,
     *          allowing clean shutdown with parameter saves.
     * 
     * @note Called during run() initialization
     * @note Signal-safe: Handler only sets atomic flag
     */
    void setup_signal_handlers() const;

    /**
     * @brief Static signal handler for clean shutdown
     * 
     * @param[in] signum Signal number (SIGTERM, SIGINT, etc.)
     * 
     * @details Sets global _should_exit flag causing main loop to terminate
     *          and perform cleanup (save parameters, close files, shutdown drivers).
     *          Must be static to use with signal() API.
     * 
     * @note Signal-safe: Only modifies sig_atomic_t variable
     * @note Actual cleanup happens in main loop, not in handler
     */
    static void exit_signal_handler(int);

protected:
    /**
     * Flag indicating shutdown request from signal handler
     * volatile sig_atomic_t ensures atomic access from signal context
     */
    volatile sig_atomic_t _should_exit = false;
};

#if HAL_NUM_CAN_IFACES
namespace Linux {
    class CANIface;
}
typedef Linux::CANIface HAL_CANIface;
#endif
