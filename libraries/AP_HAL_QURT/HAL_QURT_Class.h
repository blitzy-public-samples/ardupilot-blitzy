/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file HAL_QURT_Class.h
 * @brief HAL_QURT class definition - concrete HAL implementation for Qualcomm Hexagon DSP platform
 * 
 * This file defines HAL_QURT, the Hardware Abstraction Layer implementation for
 * the Qualcomm Hexagon DSP (QDSP6) running the QURT RTOS. This HAL is used on
 * platforms such as Snapdragon Flight and VOXL where ArduPilot runs on the
 * Sensors Low Power Island (SLPI) DSP processor.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2015-2025 ArduPilot.org
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_HAL_QURT_Namespace.h"
#include "interface.h"

/**
 * @class HAL_QURT
 * @brief Main HAL class for QURT/Hexagon DSP platform (Snapdragon Flight, VOXL)
 * 
 * @details HAL_QURT extends AP_HAL::HAL to provide a complete Hardware Abstraction Layer
 *          implementation for Qualcomm Hexagon DSP processors running the QURT real-time
 *          operating system. This HAL enables ArduPilot to execute on the DSP subsystem
 *          of Snapdragon platforms, providing access to sensors and peripherals through
 *          the SLPI (Sensors Low Power Island) execution environment.
 * 
 *          Architecture:
 *          - Implements HAL singleton pattern accessed via get_HAL()
 *          - Wires up QURT-specific driver implementations for all HAL interfaces
 *          - Provides DSP-specific initialization and main thread management
 *          - Integrates with QURT RTOS pthread and timer APIs
 *          - Communicates with application processor via RPC (sl_client_* API)
 * 
 *          Driver Initialization:
 *          The HAL_QURT constructor initializes the following QURT-specific drivers:
 *          - Scheduler: QURT::Scheduler for task scheduling with pthread
 *          - Storage: QURT::Storage for parameter/mission persistence
 *          - UARTDriver: QURT::UARTDriver for serial communication
 *          - I2CDeviceManager: QURT I2C bus interface
 *          - SPIDeviceManager: QURT SPI bus interface
 *          - RCInput: QURT::RCInput for RC receiver input
 *          - RCOutput: QURT::RCOutput for PWM/motor output
 *          - AnalogIn: QURT::AnalogIn for ADC channels
 * 
 *          DSP Execution Model:
 *          - Main vehicle code runs on a dedicated pthread with APM_MAIN_PRIORITY
 *          - Scheduler callbacks run in separate timer threads
 *          - I/O operations may invoke RPC to applications processor
 *          - Uses QURT timer_sleep API for timing operations
 * 
 * @note HAL singleton pattern: Only one HAL_QURT instance exists, returned by get_HAL()
 * @note Entry point: qurt_ardupilot_main() in AP_HAL_QURT_Main.h calls run()
 * @note Inter-processor communication: Many HAL operations invoke RPC calls to the
 *       applications processor via sl_client_* API defined in interface.h
 * 
 * @warning DSP memory constraints: Hexagon DSP has significantly limited memory compared
 *          to the applications processor. Code size and heap allocations must be minimized.
 * @warning Thread-safety: All HAL operations must use proper synchronization primitives
 *          (HAL semaphores) when accessing shared state across timer/IO/main threads.
 * @warning SLPI restrictions: The SLPI execution environment has restricted system call
 *          access and requires RPC for many peripheral operations.
 * 
 * @see AP_HAL::HAL for the abstract HAL interface definition
 * @see AP_HAL_QURT_Main.h for the qurt_ardupilot_main entry point
 * @see interface.h for SLPI link ABI and sl_client_* RPC function declarations
 * 
 * Source: libraries/AP_HAL_QURT/HAL_QURT_Class.cpp for implementation details
 */
class HAL_QURT : public AP_HAL::HAL
{
public:
    /**
     * @brief Constructs the HAL_QURT singleton instance
     * 
     * @details Initializes the HAL singleton with QURT-specific driver implementations
     *          for all hardware abstraction interfaces. The constructor wires up:
     *          - Scheduler for task scheduling and timing
     *          - Storage backend for parameter/mission persistence
     *          - UART drivers for serial communication
     *          - I2C and SPI device managers for sensor bus access
     *          - RC input/output drivers for receiver and PWM
     *          - Analog input for ADC channels
     * 
     *          This constructor is called once during static initialization to create
     *          the single HAL instance for the Hexagon DSP platform.
     * 
     * @note Only one HAL_QURT instance should exist (singleton pattern)
     * @note Constructor runs during C++ static initialization before main()
     */
    HAL_QURT();
    
    /**
     * @brief Main HAL entry point - initializes HAL and starts vehicle execution
     * 
     * @param[in] argc Command line argument count (typically 0 on embedded platforms)
     * @param[in] argv Command line argument vector (typically NULL on embedded platforms)
     * @param[in] callbacks Vehicle setup/loop callbacks containing vehicle-specific
     *                      setup() and loop() functions to execute
     * 
     * @details Implements the HAL initialization sequence for QURT platform:
     *          1. Spawns main_thread as a pthread for vehicle execution
     *          2. Registers fatal error hooks for crash handling
     *          3. Starts the HAL scheduler for timer callbacks and periodic tasks
     *          4. Blocks until vehicle execution completes (never returns normally)
     * 
     *          The run() method is the bridge between the platform entry point
     *          (qurt_ardupilot_main) and the vehicle code. It sets up the execution
     *          environment and transfers control to the vehicle callbacks.
     * 
     * @note Called by qurt_ardupilot_main() entry point (see AP_HAL_QURT_Main.h)
     * @note This method does not return under normal operation
     * @note argc/argv are preserved for potential future command-line configuration
     * 
     * @warning Must be called only once during vehicle startup on DSP
     * @warning Calling run() multiple times will result in undefined behavior
     * @warning This method blocks indefinitely - it does not return to caller
     */
    void run(int argc, char* const* argv, Callbacks* callbacks) const override;
    
    /**
     * @brief Creates and starts the main vehicle execution thread
     * 
     * @param[in] callbacks Vehicle setup/loop callbacks to execute on main thread
     * 
     * @details Creates a pthread for main vehicle execution with APM_MAIN_PRIORITY
     *          scheduling priority. The thread executes main_thread() which runs
     *          the vehicle setup() and loop() functions.
     * 
     *          Thread configuration:
     *          - Priority: APM_MAIN_PRIORITY (defined in AP_HAL_QURT)
     *          - Affinity: Configured for optimal DSP core assignment
     *          - Stack: Sized for vehicle execution requirements
     *          - Detached: Thread resources cleaned up automatically
     * 
     *          DSP Scheduling Considerations:
     *          The main thread priority is balanced against scheduler timer threads
     *          and I/O threads to ensure proper system timing while allowing vehicle
     *          control loops to execute with appropriate latency.
     * 
     * @note Called internally by run() during HAL initialization
     * @note Thread creation uses QURT pthread API (POSIX-like interface)
     * @note Thread starts immediately upon creation
     * 
     * @warning Thread priority must be carefully chosen to balance vehicle control
     *          loop timing with scheduler and I/O thread execution
     */
    void start_main_thread(Callbacks* callbacks);
    
    /**
     * @brief Main vehicle execution thread function
     * 
     * @details This is the main vehicle execution thread that runs the vehicle-specific
     *          setup() and loop() functions via the callbacks registered in run().
     * 
     *          Execution sequence:
     *          1. Calls vehicle setup() once for initialization
     *          2. Enters infinite loop calling vehicle loop() repeatedly
     *          3. Handles any fatal errors or exceptions
     *          4. Never returns under normal operation
     * 
     *          Execution context:
     *          - Runs as a pthread on Hexagon DSP with QURT RTOS thread context
     *          - Has APM_MAIN_PRIORITY scheduling priority
     *          - Executes vehicle control loops, navigation, and mission logic
     *          - Interacts with scheduler callbacks for sensor updates and I/O
     * 
     * @note Executes on Hexagon DSP with QURT RTOS thread context
     * @note This function does not return - runs for vehicle lifetime
     * @note Called internally by pthread created in start_main_thread()
     * 
     * @warning Thread-safety: Uses HAL semaphores for synchronization with
     *          timer/IO threads when accessing shared hardware or state
     * @warning Memory constraints: Must operate within DSP heap limitations
     * @warning This thread runs the core vehicle logic - errors here are fatal
     */
    void main_thread(void);
};
