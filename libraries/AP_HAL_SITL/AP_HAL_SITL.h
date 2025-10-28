/**
 * @file AP_HAL_SITL.h
 * @brief Main public include header for the AP_HAL_SITL platform
 * 
 * @details This header serves as the entry point for the SITL (Software In The Loop)
 *          Hardware Abstraction Layer implementation. The SITL HAL provides a complete
 *          hardware abstraction that runs on host operating systems (Linux, MacOS, Windows)
 *          to enable rapid testing and development without requiring physical hardware.
 * 
 *          SITL HAL simulates all hardware interfaces including:
 *          - Serial ports (UART) via TCP/UDP sockets
 *          - I2C and SPI device interfaces
 *          - GPIO and interrupt handling
 *          - Scheduler and timing using host OS time
 *          - Storage using host filesystem
 *          - RC input/output via MAVLink or other protocols
 * 
 *          Vehicle code should NOT include this header directly. Instead, vehicle code
 *          includes AP_HAL/AP_HAL.h which automatically selects the appropriate platform
 *          HAL based on the CONFIG_HAL_BOARD build configuration setting.
 * 
 *          The conditional compilation guard (CONFIG_HAL_BOARD == HAL_BOARD_SITL) ensures
 *          this HAL is only compiled when building for SITL simulation targets. This
 *          prevents SITL-specific code from being included in embedded firmware builds.
 * 
 *          This header includes HAL_SITL_Class.h which defines the HAL_SITL_Class and
 *          contains the global HAL instance (hal) that provides access to all hardware
 *          abstraction interfaces during SITL simulation.
 * 
 * @note SITL enables rapid testing, continuous integration, and algorithm development
 *       without the risks and time overhead of hardware-in-the-loop testing.
 * 
 * @see HAL_SITL_Class.h for the SITL HAL implementation class
 * @see README.md in libraries/AP_HAL_SITL/ for comprehensive SITL documentation,
 *      architecture details, and usage examples
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

/**
 * Conditional compilation guard for SITL platform
 * 
 * This block is only compiled when CONFIG_HAL_BOARD is set to HAL_BOARD_SITL
 * during the build configuration, ensuring SITL-specific code is isolated from
 * embedded hardware builds.
 */
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

/**
 * Include the SITL HAL implementation class
 * 
 * HAL_SITL_Class.h defines the HAL_SITL_Class which implements all AP_HAL
 * interfaces for simulation on host operating systems. It also contains the
 * global 'hal' instance that vehicle code uses to access hardware abstractions.
 */
#include "HAL_SITL_Class.h"

#endif  // CONFIG_HAL_BOARD
