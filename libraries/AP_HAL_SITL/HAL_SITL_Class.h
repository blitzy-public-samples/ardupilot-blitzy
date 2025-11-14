/**
 * @file HAL_SITL_Class.h
 * @brief SITL (Software In The Loop) Hardware Abstraction Layer implementation
 * 
 * This file defines the HAL_SITL class, which implements the AP_HAL::HAL interface
 * for the SITL simulation platform. This is the top-level entry point for running
 * ArduPilot vehicle code in the SITL simulation environment.
 * 
 * The HAL_SITL class:
 * - Provides the main run() loop entry point for SITL execution
 * - Initializes all HAL components (UART, I2C, SPI, scheduler, storage, etc.)
 * - Integrates with SITL_State for physics simulation and sensor emulation
 * - Manages storage backend configuration (POSIX, flash, FRAM emulation)
 * - Handles signal handlers for clean simulation termination
 * 
 * Vehicle code accesses HAL functionality through the global 'hal' object,
 * which is an instance of this class instantiated in AP_HAL_SITL_Class.cpp.
 * 
 * @note This implementation is only compiled when CONFIG_HAL_BOARD == HAL_BOARD_SITL
 * @warning SITL HAL behavior differs from real hardware HAL implementations:
 *          - Timing is not real-time (can run faster or slower than wall clock)
 *          - Sensors are simulated rather than reading actual hardware
 *          - Storage backends emulate flash/FRAM without real hardware constraints
 *          - No actual hardware interrupts or DMA operations
 * 
 * @see AP_HAL::HAL for the abstract HAL interface definition
 * @see SITL_State for physics simulation and sensor emulation
 * @see libraries/AP_HAL_SITL/README.md for SITL architecture documentation
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "SITL_State.h"

/**
 * @class HAL_SITL
 * @brief SITL platform Hardware Abstraction Layer implementation
 * 
 * @details HAL_SITL implements the AP_HAL::HAL interface for the Software In The Loop
 *          simulation environment. This class serves as the top-level HAL object that
 *          vehicle code uses to access all hardware functionality in SITL.
 * 
 * Key Responsibilities:
 * - Initialize all HAL device drivers (UARTs, SPI, I2C, GPIO, etc.)
 * - Provide the main run() entry point that executes vehicle setup() and loop()
 * - Manage the SITL_State object for physics and sensor simulation
 * - Configure storage backend emulation (POSIX files, flash, FRAM)
 * - Handle command-line arguments for SITL configuration
 * - Provide signal handlers for graceful shutdown (Ctrl+C)
 * - Support simulation-specific features like storage wiping for testing
 * 
 * Architecture:
 * The HAL_SITL class owns a SITL_State object that:
 * - Runs the physics simulation in a separate thread
 * - Emulates sensor data (IMU, GPS, barometer, compass, etc.)
 * - Interfaces with external simulators (JSBSim, Gazebo, RealFlight, etc.)
 * - Manages simulation timing and speed
 * 
 * Instantiation and Usage:
 * - A global instance is created in AP_HAL_SITL_Class.cpp as 'const HAL_SITL hal;'
 * - Vehicle code accesses this via the global 'hal' reference
 * - The run() method is called from main() to start vehicle execution
 * - Vehicle setup() and loop() callbacks are invoked from run()
 * 
 * Storage Backend Configuration:
 * SITL supports multiple storage backend emulations that can be enabled/disabled:
 * - POSIX: File-based storage using the host filesystem (default enabled)
 * - Flash: Emulates flash memory with wear leveling
 * - FRAM: Emulates FRAM (Ferroelectric RAM) with faster write cycles
 * 
 * This allows testing different storage configurations without hardware changes.
 * 
 * @note Vehicle code should use the global 'hal' object, not create HAL_SITL instances
 * @warning Do not instantiate HAL_SITL directly - use the global 'hal' object
 * 
 * @see AP_HAL::HAL for inherited interface methods
 * @see SITL_State for simulation state management
 */
class HAL_SITL : public AP_HAL::HAL {
public:
    /**
     * @brief Construct the SITL Hardware Abstraction Layer
     * 
     * @details Initializes the HAL_SITL instance by:
     *          - Setting up all HAL device pointers (UART, SPI, I2C, GPIO, etc.)
     *          - Creating the SITL_State object for physics simulation
     *          - Initializing storage backend configuration
     *          - Preparing signal handlers for clean shutdown
     * 
     * The constructor does not start simulation - that happens in run().
     * 
     * @note This is called once during static initialization to create the global 'hal' object
     * @see run() for the main execution entry point
     */
    HAL_SITL();
    
    /**
     * @brief Main execution entry point for SITL simulation
     * 
     * @details This is the top-level entry point called from main() to start the
     *          ArduPilot vehicle in SITL simulation mode. The method:
     *          
     *          1. Parses command-line arguments for SITL configuration
     *          2. Initializes the SITL_State physics simulation
     *          3. Sets up all HAL device drivers
     *          4. Installs signal handlers (SIGINT, SIGTERM) for clean shutdown
     *          5. Calls the vehicle's setup() callback function once
     *          6. Enters the main loop, repeatedly calling the vehicle's loop() callback
     *          
     *          The method runs until the simulation is terminated (Ctrl+C, exit command,
     *          or programmatic shutdown).
     *          
     *          Command-line arguments control:
     *          - Vehicle instance number (for multi-vehicle simulation)
     *          - Home location (latitude, longitude, altitude, heading)
     *          - Simulation model and parameters
     *          - Network configuration for external simulator connections
     *          - Storage backend selection and configuration
     *          
     *          Main Loop Execution:
     *          The main loop executes vehicle loop() callbacks at the scheduler's configured
     *          rate, synchronized with the SITL_State simulation timing.
     * 
     * @param[in] argc       Argument count from command line
     * @param[in] argv       Argument vector from command line
     * @param[in] callbacks  Vehicle callbacks containing setup() and loop() functions
     * 
     * @note This method does not return under normal operation (runs until shutdown)
     * @warning Any exceptions or fatal errors in this method will terminate the simulation
     * 
     * @see SITL_State for simulation timing and physics integration
     * @see setup_signal_handlers() for signal handling configuration
     */
    void run(int argc, char * const argv[], Callbacks* callbacks) const override;
    
    /**
     * @brief Perform a simulated reboot of the autopilot
     * 
     * @details Executes a clean shutdown and restart of the SITL simulation,
     *          mimicking a real hardware reboot. This:
     *          - Stops all running threads
     *          - Closes file descriptors and network connections
     *          - Exits the process (allowing external restart scripts to relaunch)
     *          
     *          In SITL, a "reboot" typically means exiting the process with a special
     *          exit code that instructs the sim_vehicle.py wrapper script to restart.
     * 
     * @note This is a static method callable without a HAL_SITL instance
     * @warning This immediately terminates the simulation - no cleanup callbacks are called
     * 
     * @see exit_signal_handler() for signal-based termination
     */
    static void actually_reboot();

    /**
     * @brief Enable or disable POSIX file-based storage emulation
     * 
     * @details Controls whether SITL uses POSIX filesystem-based storage for
     *          parameter and configuration persistence. When enabled, parameters
     *          are stored in files on the host filesystem (typically eeprom.bin).
     *          
     *          POSIX storage is the default and most commonly used storage backend
     *          in SITL as it provides easy inspection and modification of stored data.
     * 
     * @param[in] _enabled  true to enable POSIX storage, false to disable
     * 
     * @note POSIX storage is enabled by default in SITL
     * @see StorageManager for storage backend abstraction
     */
    void set_storage_posix_enabled(bool _enabled) {
        storage_posix_enabled = _enabled;
    }
    
    /**
     * @brief Query whether POSIX file-based storage is enabled
     * 
     * @return true if POSIX storage backend is enabled, false otherwise
     */
    bool get_storage_posix_enabled() const { return storage_posix_enabled; }
    
    /**
     * @brief Enable or disable flash memory storage emulation
     * 
     * @details Controls whether SITL emulates flash memory storage behavior,
     *          including wear leveling and flash-specific write characteristics.
     *          This allows testing storage code paths that would execute on
     *          real hardware with flash storage.
     * 
     * @param[in] _enabled  true to enable flash emulation, false to disable
     * 
     * @note Flash emulation simulates limitations like block erase requirements
     * @see AP_FlashStorage for flash storage implementation
     */
    void set_storage_flash_enabled(bool _enabled) {
        storage_flash_enabled = _enabled;
    }
    
    /**
     * @brief Query whether flash memory storage emulation is enabled
     * 
     * @return true if flash storage emulation is enabled, false otherwise
     */
    bool get_storage_flash_enabled() const { return storage_flash_enabled; }
    
    /**
     * @brief Enable or disable FRAM (Ferroelectric RAM) storage emulation
     * 
     * @details Controls whether SITL emulates FRAM storage behavior. FRAM has
     *          different characteristics than flash (no erase cycles, faster writes,
     *          limited endurance). This allows testing storage behavior on boards
     *          that use FRAM for parameter storage.
     * 
     * @param[in] _enabled  true to enable FRAM emulation, false to disable
     * 
     * @note FRAM has higher write endurance than flash but different access patterns
     */
    void set_storage_fram_enabled(bool _enabled) {
        storage_fram_enabled = _enabled;
    }
    
    /**
     * @brief Query whether FRAM storage emulation is enabled
     * 
     * @return true if FRAM storage emulation is enabled, false otherwise
     */
    bool get_storage_fram_enabled() const { return storage_fram_enabled; }

    /**
     * @brief Instruct simulation to wipe storage on initialization
     * 
     * @details When enabled, all storage backends will be erased/reset when
     *          opened during SITL initialization. This is useful for:
     *          - Testing first-boot behavior
     *          - Ensuring clean parameter state for automated testing
     *          - Debugging parameter loading issues
     *          - Simulating factory reset conditions
     *          
     *          Storage wipe occurs once at startup before vehicle setup() is called.
     * 
     * @param[in] _wipe_storage  true to wipe storage at startup, false to preserve
     * 
     * @note This affects all enabled storage backends (POSIX, flash, FRAM)
     * @warning Wiping storage will reset all parameters to defaults
     * 
     * @see StorageManager for storage backend management
     */
    void set_wipe_storage(bool _wipe_storage) {
        wipe_storage = _wipe_storage;
    }
    
    /**
     * @brief Query whether storage will be wiped at startup
     * 
     * @return true if storage will be wiped on initialization, false otherwise
     */
    bool get_wipe_storage() const { return wipe_storage; }

    /**
     * @brief Get the SITL instance number for multi-vehicle simulation
     * 
     * @details Returns the instance number of this SITL simulation, used when
     *          running multiple vehicle instances simultaneously. Instance numbers
     *          are typically set via command-line arguments and determine:
     *          - Network port offsets for MAVLink and other connections
     *          - Separate storage file locations (eeprom0.bin, eeprom1.bin, etc.)
     *          - Different home locations for each vehicle
     *          - Unique simulated sensor IDs
     *          
     *          Instance 0 is the default for single-vehicle simulation.
     * 
     * @return Instance number (0-based), typically 0-15 for multi-vehicle setups
     * 
     * @note Instance numbers are configured via the -I command-line argument
     * @see SITL_State::instance for the underlying instance value
     */
    uint8_t get_instance() const;

#if defined(HAL_BUILD_AP_PERIPH)
    /**
     * @brief Check if running in maintenance mode (AP_Periph only)
     * 
     * @details For AP_Periph builds, determines if the peripheral is running in
     *          maintenance mode for firmware updates or diagnostics. Not applicable
     *          to full vehicle builds.
     * 
     * @return true if in maintenance mode, false for normal operation
     * 
     * @note Only compiled for HAL_BUILD_AP_PERIPH builds (CAN peripherals)
     */
    bool run_in_maintenance_mode() const;
#endif

    /**
     * @brief Get count of UART output buffer full conditions
     * 
     * @details Returns the total number of times UART output buffers became full
     *          and had to drop data or block. This metric helps identify:
     *          - MAVLink telemetry congestion
     *          - GPS or other serial device communication issues
     *          - Excessive logging or debug output
     *          
     *          High values may indicate the simulation is running too fast for
     *          network/serial output to keep up, or that output data rate is too high.
     * 
     * @return Count of UART buffer full events since simulation start
     * 
     * @note This is a diagnostic metric useful for performance tuning
     * @see UARTDriver for individual UART statistics
     */
    uint32_t get_uart_output_full_queue_count() const;

    /**
     * @brief Get the SITL_State object managing physics simulation
     * 
     * @details Returns a pointer to the SITL_State object that manages:
     *          - Physics simulation integration with external simulators
     *          - Sensor data emulation (IMU, GPS, barometer, compass, etc.)
     *          - Simulation timing and rate control
     *          - Home location and vehicle initial conditions
     *          
     *          The SITL_State object is the core of the SITL simulation,
     *          bridging the gap between ArduPilot vehicle code and physics engines.
     * 
     * @return Pointer to SITL_State object (never null after HAL initialization)
     * 
     * @note This is used internally by HAL drivers to access simulated sensor data
     * @see SITL_State for simulation state management
     */
    HALSITL::SITL_State * get_sitl_state() { return _sitl_state; }

private:
    /**
     * @brief Pointer to SITL_State object managing physics and sensor simulation
     * 
     * @details The SITL_State object is created in the HAL_SITL constructor and
     *          owns the physics simulation thread, sensor emulation, and external
     *          simulator communication. All simulated sensor drivers access this
     *          object to retrieve emulated sensor data.
     */
    HALSITL::SITL_State *_sitl_state;

    /**
     * @brief Install signal handlers for clean simulation shutdown
     * 
     * @details Installs handlers for SIGINT (Ctrl+C) and SIGTERM signals to
     *          enable graceful shutdown of the SITL simulation. The handlers:
     *          - Close log files and flush data
     *          - Terminate physics simulation threads
     *          - Clean up network connections
     *          - Exit with appropriate status code
     *          
     *          Without signal handlers, Ctrl+C would cause abrupt termination
     *          and potential data loss or corruption.
     * 
     * @note Called during run() initialization before entering main loop
     * @see exit_signal_handler() for the actual signal handler function
     */
    void setup_signal_handlers() const;
    
    /**
     * @brief Signal handler for SIGINT and SIGTERM
     * 
     * @details Static function called by the OS when SIGINT (Ctrl+C) or SIGTERM
     *          is received. Performs minimal cleanup and exits the process cleanly.
     *          
     *          As a signal handler, this function must be async-signal-safe and
     *          cannot perform complex operations like memory allocation or logging.
     * 
     * @param[in] signum  Signal number (SIGINT, SIGTERM, etc.)
     * 
     * @note This is a static method as it's registered as a C-style signal handler
     * @warning Only async-signal-safe operations allowed in signal handlers
     */
    static void exit_signal_handler(int signum);

    /**
     * @brief Flag to enable POSIX file-based storage backend
     * 
     * Default: true (POSIX storage is the standard SITL storage mechanism)
     */
    bool storage_posix_enabled = true;
    
    /**
     * @brief Flag to enable flash memory storage emulation
     * 
     * Default: false (disabled unless explicitly enabled for testing)
     */
    bool storage_flash_enabled;
    
    /**
     * @brief Flag to enable FRAM storage emulation
     * 
     * Default: false (disabled unless explicitly enabled for testing)
     */
    bool storage_fram_enabled;

    /**
     * @brief Flag to wipe all storage on initialization
     * 
     * @details When true, all storage backends are erased when opened,
     *          simulating first-boot or factory reset conditions.
     *          Useful for automated testing requiring clean state.
     * 
     * Default: false (preserve storage across simulation runs)
     */
    bool wipe_storage;
};

/**
 * @brief Define HAL_CANIface as the SITL CAN interface implementation
 * 
 * @details When CAN interfaces are enabled (HAL_NUM_CAN_IFACES > 0), this typedef
 *          aliases HAL_CANIface to the SITL-specific CAN interface implementation.
 *          This allows generic HAL code to use HAL_CANIface while the actual
 *          implementation is platform-specific.
 *          
 *          SITL CAN interface emulates CAN bus behavior for testing DroneCAN/UAVCAN
 *          peripherals in simulation without physical CAN hardware.
 * 
 * @note Only defined when CAN support is compiled in (HAL_NUM_CAN_IFACES > 0)
 * @see HALSITL::CANIface for the SITL CAN implementation
 * @see AP_CANManager for CAN bus management
 */
#if HAL_NUM_CAN_IFACES
typedef HALSITL::CANIface HAL_CANIface;
#endif

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_SITL
