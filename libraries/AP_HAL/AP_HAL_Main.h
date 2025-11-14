/*
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
 *
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
 * @file AP_HAL_Main.h
 * @brief Main entry point macros for synthesizing platform-specific main() functions
 * 
 * @details This file provides macros that generate the C main() function and wire it to
 * HAL::run() with firmware callbacks. These macros handle platform-specific initialization
 * and startup code while presenting a uniform interface to vehicle code.
 * 
 * The macros expand to different code based on CONFIG_HAL_BOARD, allowing each platform
 * (ChibiOS, Linux, SITL, ESP32, etc.) to implement its required initialization sequence
 * while maintaining consistent vehicle code across all platforms.
 * 
 * Key responsibilities handled by these macros:
 * - Creating the main() entry point function
 * - Platform-specific early initialization (clocks, memory, watchdog)
 * - Command-line argument parsing (where applicable)
 * - HAL singleton initialization
 * - Connecting vehicle callbacks to the HAL scheduler
 * - Entering the main HAL event loop via hal.run()
 * 
 * @note Used by all vehicle sketches (ArduCopter, ArduPlane, Rover, ArduSub, etc.)
 * @note The generated main() function never returns - it enters an infinite loop
 * @note Platform-specific macro expansions are defined in each HAL implementation
 * 
 * @see AP_HAL::HAL::run() for the main event loop implementation
 * @see AP_HAL::HAL::Callbacks for the vehicle callback interface
 */
#pragma once

#include "HAL.h"

/**
 * @def AP_MAIN
 * @brief Defines the name of the main entry point function
 * 
 * @details This macro allows platforms to override the standard "main" function name
 * if needed for their specific requirements. Most platforms use the default "main",
 * but some embedded systems or test frameworks may require a different entry point name.
 * 
 * The macro can be defined before including this header to customize the entry point:
 * ```cpp
 * #define AP_MAIN custom_main
 * #include <AP_HAL/AP_HAL_Main.h>
 * ```
 * 
 * @note Defaults to "main" if not defined
 * @note Must be defined before including AP_HAL_Main.h to take effect
 */
#ifndef AP_MAIN
#define AP_MAIN main
#endif

/**
 * @def AP_HAL_MAIN()
 * @brief Legacy main entry point macro using function-based callbacks
 * 
 * @details Generates a main() function that creates HAL::FunCallbacks from setup() and
 * loop() functions, then calls hal.run() with those callbacks. This macro was the original
 * entry point mechanism but has been deprecated in favor of AP_HAL_MAIN_CALLBACKS().
 * 
 * The macro expects two functions to be defined:
 * - void setup() - Called once during initialization
 * - void loop() - Called repeatedly in the main loop
 * 
 * Legacy usage pattern:
 * ```cpp
 * void setup() {
 *     // Initialization code
 * }
 * 
 * void loop() {
 *     // Main loop code
 * }
 * 
 * AP_HAL_MAIN();  // Generate main() that calls setup() once, then loop() repeatedly
 * ```
 * 
 * The macro internally creates:
 * 1. An AP_HAL::HAL::FunCallbacks object wrapping setup() and loop()
 * 2. A main() function declaration and definition
 * 3. A call to hal.run(argc, argv, &callbacks) which never returns
 * 
 * @note This macro is preserved for backward compatibility only
 * @deprecated Use AP_HAL_MAIN_CALLBACKS() with a full callbacks object for modern vehicle code
 * @warning The setup()/loop() pattern is less flexible than object-oriented callbacks
 * 
 * @see AP_HAL_MAIN_CALLBACKS() for the modern entry point macro
 * @see AP_HAL::HAL::FunCallbacks for the function callback wrapper
 * @see AP_HAL::HAL::run() for the main event loop
 */
#define AP_HAL_MAIN() \
    AP_HAL::HAL::FunCallbacks callbacks(setup, loop); \
    extern "C" {                               \
    int AP_MAIN(int argc, char* const argv[]); \
    int AP_MAIN(int argc, char* const argv[]) { \
        hal.run(argc, argv, &callbacks); \
        return 0; \
    } \
    }

/**
 * @def AP_HAL_MAIN_CALLBACKS(CALLBACKS)
 * @brief Primary main entry point macro for vehicle firmware
 * 
 * @param CALLBACKS Pointer to AP_HAL::HAL::Callbacks implementation (typically the vehicle class)
 * 
 * @details This macro generates the complete main() function for ArduPilot vehicle firmware,
 * handling all platform-specific initialization and connecting the vehicle callbacks to the
 * HAL scheduler. This is the standard entry point used by all modern vehicle implementations
 * (ArduCopter, ArduPlane, Rover, ArduSub, Blimp, AntennaTracker).
 * 
 * The macro creates a main() function that:
 * 1. Declares the main() function signature with standard argc/argv parameters
 * 2. Calls hal.run(argc, argv, callbacks) which never returns
 * 3. Includes unreachable return 0 for compiler satisfaction
 * 
 * The actual platform-specific initialization happens inside hal.run(), which is implemented
 * differently for each HAL platform. Common initialization tasks include:
 * - Early hardware initialization (clocks, memory controllers, watchdogs)
 * - Device driver initialization (UART, SPI, I2C, GPIO)
 * - Scheduler initialization
 * - Command-line argument parsing (on platforms that support it)
 * - Callback registration with the scheduler
 * - Entering the main event loop
 * 
 * Typical usage in vehicle code:
 * ```cpp
 * class Copter : public AP_HAL::HAL::Callbacks {
 * public:
 *     void setup() override;
 *     void loop() override;
 * };
 * 
 * Copter copter;
 * AP_HAL_MAIN_CALLBACKS(&copter);
 * ```
 * 
 * Platform-Specific Behavior:
 * 
 * **ChibiOS (ARM embedded targets):**
 * - Sets up ChibiOS RTOS kernel
 * - Configures system clocks and PLLs
 * - Initializes USB stack (with enumeration delay for bootloader detection)
 * - Configures watchdog timers
 * - Sets up DMA controllers
 * - Creates main thread and enters RTOS scheduler
 * 
 * **Linux (including Raspberry Pi, BeagleBone, etc.):**
 * - Parses command-line arguments (--help, --daemon, --instance, etc.)
 * - Sets up signal handlers for graceful shutdown
 * - Configures real-time scheduling if available (SCHED_FIFO)
 * - Daemonizes process if requested
 * - Initializes sysfs/GPIO access
 * - Sets up memory-mapped hardware access where applicable
 * 
 * **SITL (Software In The Loop simulation):**
 * - Parses extensive command-line options (--home, --model, --speedup, etc.)
 * - Connects to simulator via TCP/UDP
 * - Sets up multiple simulated serial ports
 * - Enables networking for GCS connections
 * - Configures simulated sensor inputs
 * - Enables debug features (stack checking, assert handling)
 * 
 * **ESP32 (Wi-Fi enabled targets):**
 * - Configures Wi-Fi stack
 * - Sets up FreeRTOS tasks
 * - Initializes flash filesystem
 * - Configures partition table
 * - Sets up network interfaces
 * 
 * Command-Line Arguments:
 * The argc and argv parameters allow platforms that support command-line arguments
 * (Linux, SITL) to configure runtime behavior. Embedded platforms (ChibiOS, ESP32)
 * typically ignore these parameters. Common arguments include:
 * - --help: Display usage information
 * - --daemon: Run as background process (Linux)
 * - --instance N: Multi-vehicle simulation (SITL)
 * - --home LAT,LON,ALT,HDG: Set home location (SITL)
 * - --model NAME: Select vehicle model (SITL)
 * 
 * HAL Singleton:
 * The macro uses the global 'hal' singleton object, which is defined by each platform's
 * HAL implementation (e.g., AP_HAL_ChibiOS::HAL_ChibiOS hal). The singleton provides
 * access to all hardware abstraction interfaces (UART, SPI, I2C, GPIO, Scheduler, etc.).
 * 
 * Callbacks Object Lifecycle:
 * The CALLBACKS pointer must remain valid for the entire program lifetime, as hal.run()
 * stores this pointer and calls setup() once during initialization, then calls loop()
 * repeatedly forever. The callbacks object is typically a global vehicle instance.
 * 
 * Return Behavior:
 * The main() function generated by this macro never returns under normal operation.
 * The hal.run() call enters an infinite loop managed by the platform's scheduler.
 * The "return 0;" statement is unreachable but required by C standard for main().
 * 
 * @note Macro expansion varies dramatically by platform - see platform HAL documentation
 * @note The main() function never returns - enters infinite loop via hal.run()
 * @note The callbacks object must remain valid for the entire program lifetime
 * @note On embedded platforms, argc/argv may be zero/null as no command line exists
 * 
 * @warning Do not attempt to return from or exit the main() function
 * @warning The callbacks object must not be stack-allocated or temporary
 * @warning Ensure callbacks->setup() and callbacks->loop() are thread-safe if required
 * 
 * @see AP_HAL::HAL::run() for the main event loop implementation
 * @see AP_HAL::HAL::Callbacks for the callback interface definition
 * @see AP_HAL_ChibiOS for ChibiOS-specific initialization
 * @see AP_HAL_Linux for Linux-specific initialization
 * @see AP_HAL_SITL for simulation-specific initialization
 * 
 * Source: libraries/AP_HAL/AP_HAL_Main.h:35-41
 */
#define AP_HAL_MAIN_CALLBACKS(CALLBACKS) extern "C" { \
    int AP_MAIN(int argc, char* const argv[]); \
    int AP_MAIN(int argc, char* const argv[]) { \
        hal.run(argc, argv, CALLBACKS); \
        return 0; \
    } \
    }
