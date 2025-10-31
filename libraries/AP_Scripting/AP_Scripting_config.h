/**
 * @file AP_Scripting_config.h
 * @brief Configuration header for ArduPilot Lua scripting subsystem
 * 
 * @details This file defines the configuration macros that control the availability
 *          and features of the AP_Scripting library. The scripting subsystem provides
 *          Lua scripting capabilities to ArduPilot, allowing users to extend vehicle
 *          behavior, implement custom algorithms, and integrate with peripherals without
 *          modifying core firmware.
 * 
 *          The configuration system uses conditional compilation to:
 *          - Enable/disable scripting based on available flash memory
 *          - Validate required filesystem backend availability
 *          - Control optional features like virtual serial device support
 * 
 *          Platform Requirements:
 *          - Minimum 1MB flash memory (HAL_PROGRAM_SIZE_LIMIT_KB > 1024)
 *          - At least one supported filesystem backend
 *          - POSIX-compliant filesystem interface or equivalent
 * 
 * @note This is a configuration-only header. See libraries/AP_Scripting/README.md
 *       for comprehensive scripting subsystem documentation including architecture,
 *       Lua API reference, and usage examples.
 * 
 * @see AP_Scripting.h for the main scripting subsystem interface
 * @see libraries/AP_Scripting/docs/ for Lua binding documentation
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_SerialManager/AP_SerialManager_config.h>

/**
 * @def AP_SCRIPTING_ENABLED
 * @brief Master enable/disable for Lua scripting subsystem
 * 
 * @details Controls compilation of the entire AP_Scripting library. When enabled,
 *          provides Lua 5.3 scripting capabilities allowing users to:
 *          - Extend vehicle behavior with custom scripts
 *          - Implement custom control algorithms
 *          - Interface with custom sensors and peripherals
 *          - Create mission logic and automation
 *          - Access vehicle state and actuator control
 * 
 *          Default Policy:
 *          - Automatically enabled on boards with >1MB flash (HAL_PROGRAM_SIZE_LIMIT_KB > 1024)
 *          - Can be explicitly disabled by defining AP_SCRIPTING_ENABLED 0 in hwdef
 *          - Can be explicitly enabled on smaller boards if space available
 * 
 *          Memory Requirements:
 *          - Flash: ~150-200KB for Lua interpreter and bindings
 *          - RAM: Configurable heap allocation (default 32-64KB per script)
 *          - Stack: ~8KB for Lua execution context
 * 
 *          Dependencies:
 *          - Filesystem backend (validated below)
 *          - AP_Scheduler for periodic script execution
 *          - Lua 5.3 interpreter (bundled in modules/lua)
 * 
 *          Platform Constraints:
 *          - Disabled on boards with ≤1MB flash by default (insufficient space)
 *          - Requires sufficient RAM for script heap allocation
 *          - Performance impact: ~100-500μs per script execution cycle
 * 
 * @note Scripts are loaded from /scripts/ directory on filesystem at boot
 * @note See SCR_* parameters for runtime configuration (heap size, VM count, etc.)
 * 
 * @warning Lua scripts execute in main vehicle loop context - poorly written scripts
 *          can impact flight performance. Use SCR_HEAP_SIZE to limit memory usage.
 * 
 * @see libraries/AP_Scripting/README.md for comprehensive scripting documentation
 * @see libraries/AP_Scripting/examples/ for example scripts
 */
#ifndef AP_SCRIPTING_ENABLED
#define AP_SCRIPTING_ENABLED (HAL_PROGRAM_SIZE_LIMIT_KB > 1024)
#endif

/*
 * Filesystem Backend Validation
 * 
 * The scripting subsystem requires a functional filesystem to load Lua scripts
 * from persistent storage. Scripts are typically stored in /scripts/ directory
 * and loaded at vehicle startup or when triggered by SCR_ENABLE parameter.
 * 
 * Supported Filesystem Backends:
 * - AP_FILESYSTEM_POSIX_ENABLED: POSIX filesystem (Linux, SITL)
 * - AP_FILESYSTEM_FATFS_ENABLED: FAT filesystem on SD card (most flight controllers)
 * - AP_FILESYSTEM_ESP32_ENABLED: ESP32 SPIFFS/LittleFS (ESP32 platforms)
 * - AP_FILESYSTEM_ROMFS_ENABLED: Read-only filesystem (embedded scripts in flash)
 * - AP_FILESYSTEM_LITTLEFS_ENABLED: LittleFS for flash storage (wear-leveling)
 * 
 * At least one backend must be enabled for scripting to function. The validation
 * below produces a compile-time error if scripting is enabled but no filesystem
 * backend is available.
 * 
 * Script Loading Process:
 * 1. At boot: AP_Scripting scans /scripts/ directory
 * 2. Loads all .lua files found
 * 3. Creates separate Lua VM for each script (up to SCR_VM_I_COUNT)
 * 4. Executes scripts periodically via scheduler
 * 
 * Typical Usage:
 * - Flight controllers: FAT filesystem on SD card (AP_FILESYSTEM_FATFS_ENABLED)
 * - Linux boards: POSIX filesystem (AP_FILESYSTEM_POSIX_ENABLED)
 * - ESP32 boards: ESP32 filesystem (AP_FILESYSTEM_ESP32_ENABLED)
 * - SITL simulation: POSIX filesystem (AP_FILESYSTEM_POSIX_ENABLED)
 * - Factory scripts: ROMFS embedded in flash (AP_FILESYSTEM_ROMFS_ENABLED)
 * 
 * Performance Considerations:
 * - SD card read latency: 10-50ms per script file at startup
 * - ROMFS read latency: <1ms (flash memory access)
 * - Filesystem errors prevent script loading but don't block vehicle operation
 * 
 * @note ROMFS is typically used for factory-default scripts that cannot be modified
 * @note Multiple backends can be enabled simultaneously (e.g., FATFS + ROMFS)
 * 
 * Source: libraries/AP_Scripting/AP_Scripting.cpp (script loading logic)
 */
#if AP_SCRIPTING_ENABLED
    #include <AP_Filesystem/AP_Filesystem_config.h>
    // Enumerate all of the possible filesystem backends we can read scripts from.
    // At least one must be enabled for scripting to function.
    #if !AP_FILESYSTEM_POSIX_ENABLED && !AP_FILESYSTEM_FATFS_ENABLED && !AP_FILESYSTEM_ESP32_ENABLED && !AP_FILESYSTEM_ROMFS_ENABLED && !AP_FILESYSTEM_LITTLEFS_ENABLED
        #error "Scripting requires a filesystem backend: enable AP_FILESYSTEM_POSIX, FATFS, ESP32, ROMFS, or LITTLEFS"
    #endif
#endif

/**
 * @def AP_SCRIPTING_SERIALDEVICE_ENABLED
 * @brief Enable virtual serial device support for Lua scripts
 * 
 * @details Controls compilation of the Lua SerialDevice binding, which allows scripts
 *          to create and manage virtual serial ports. Virtual serial devices enable
 *          scripts to:
 *          - Implement custom communication protocols
 *          - Interface with custom sensors via UART
 *          - Create telemetry bridges between protocols
 *          - Emulate serial devices for testing
 *          - Parse and generate custom serial data streams
 * 
 *          The SerialDevice binding provides bidirectional serial I/O with:
 *          - Read/write byte operations
 *          - Read/write string operations  
 *          - Buffer management
 *          - Baud rate configuration
 *          - Flow control (if supported by HAL)
 * 
 *          Default Policy:
 *          - Enabled when AP_SERIALMANAGER_REGISTER_ENABLED and >1MB flash
 *          - Requires dynamic serial port registration support in SerialManager
 *          - Automatically disabled on memory-constrained platforms
 * 
 *          Memory Requirements:
 *          - Flash: ~8-12KB additional code for SerialDevice binding
 *          - RAM: Minimal overhead + buffer allocation per virtual device
 * 
 *          Dependencies:
 *          - AP_SERIALMANAGER_REGISTER_ENABLED: Dynamic serial port registration
 *          - HAL_PROGRAM_SIZE_LIMIT_KB > 1024: Sufficient flash space
 *          - AP_SCRIPTING_ENABLED: Base scripting subsystem
 * 
 *          Platform Constraints:
 *          - Requires HAL support for dynamic device registration
 *          - Limited by maximum number of serial ports in SerialManager
 *          - Performance depends on UART hardware and buffer sizes
 * 
 *          Typical Usage:
 *          - Custom sensor integration (GPS, rangefinder, compass)
 *          - Protocol translation (e.g., MAVLink to custom format)
 *          - Serial device emulation for testing
 *          - Data logging to external devices
 * 
 * @note Scripts access via serial:serialdevice() Lua binding function
 * @note Virtual devices are registered dynamically at script runtime
 * @note See scripting_serial_device.lua example for usage patterns
 * 
 * @warning Virtual serial devices operate at script execution rate (typically 50-200Hz),
 *          not true interrupt-driven serial. High-bandwidth protocols may experience
 *          data loss. Use hardware UARTs for time-critical communication.
 * 
 * @see libraries/AP_SerialManager/AP_SerialManager.h for serial port management
 * @see libraries/AP_Scripting/lua_bindings.cpp for SerialDevice binding implementation
 * @see libraries/AP_Scripting/examples/scripting_serial_device.lua for usage example
 */
#ifndef AP_SCRIPTING_SERIALDEVICE_ENABLED
#define AP_SCRIPTING_SERIALDEVICE_ENABLED AP_SERIALMANAGER_REGISTER_ENABLED && (HAL_PROGRAM_SIZE_LIMIT_KB>1024)
#endif
