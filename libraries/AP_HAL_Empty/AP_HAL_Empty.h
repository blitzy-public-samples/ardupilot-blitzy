#pragma once

/**
 * @file AP_HAL_Empty.h
 * @brief Public umbrella header for AP_HAL_Empty stub implementation module
 * 
 * @details Single-include entry point for the Empty HAL module providing compile-time
 * and link-time substitutes for all AP_HAL interfaces. Enables hardwareless
 * builds for CI, unit testing, and host-based tools without requiring actual
 * platform-specific hardware drivers.
 * 
 * @note This is the ONLY header external code should include from AP_HAL_Empty
 * @note Provides access to HAL_Empty singleton via global AP_HAL_Empty object
 * @note Internal implementation details hidden in Empty namespace
 * @note Layer exports should depend on AP_HAL.h ONLY - no platform-specific includes
 */

/* Your layer exports should depend on AP_HAL.h ONLY. */
#include <AP_HAL/AP_HAL.h>

/**
 * @brief Module design pattern and naming conventions
 * 
 * @details AP_HAL_Empty follows the standard ArduPilot HAL module pattern:
 * 
 * **Namespace organization**:
 * - `Empty` namespace: Contains all stub implementation classes
 * - Global namespace: Contains HAL_Empty class and AP_HAL_Empty singleton
 * 
 * **Class naming**:
 * - Main HAL class: `HAL_Empty` (inherits from AP_HAL::HAL)
 * - Singleton instance: `const HAL_Empty AP_HAL_Empty` (defined in HAL_Empty_Class.cpp)
 * - Component implementations: `Empty::ComponentName` (e.g., Empty::UARTDriver)
 * 
 * **Header organization**:
 * - AP_HAL_Empty.h (this file): Public umbrella, includes HAL_Empty_Class.h
 * - AP_HAL_Empty_Namespace.h: Forward declarations of Empty:: classes
 * - AP_HAL_Empty_Private.h: Internal umbrella including all implementation headers
 * - HAL_Empty_Class.h/cpp: Main HAL singleton class definition and instantiation
 * - Individual component headers: AnalogIn.h, GPIO.h, UARTDriver.h, etc.
 * 
 * **Dependency constraints**:
 * - Public exports depend ONLY on AP_HAL.h (AP_HAL interfaces)
 * - No platform-specific includes (no <linux/...>, no <ChibiOS/...>)
 * - Minimal standard library usage (prefer AP_HAL facilities)
 * 
 * This pattern enables:
 * - Clean separation between interface (AP_HAL) and implementation (Empty)
 * - Namespace isolation prevents naming conflicts
 * - Single header include for convenience (`#include <AP_HAL_Empty/AP_HAL_Empty.h>`)
 * - Implementation details hidden from external compilation units
 */

/**
 * @brief Singleton instance and accessor pattern
 * 
 * @details The Empty HAL singleton follows ArduPilot's standard pattern:
 * 
 * Declaration (in HAL_Empty_Class.h):
 * ```cpp
 * extern const HAL_Empty AP_HAL_Empty;
 * ```
 * 
 * Definition (in HAL_Empty_Class.cpp):
 * ```cpp
 * const HAL_Empty AP_HAL_Empty;
 * ```
 * 
 * Access via AP_HAL namespace functions (implemented in HAL_Empty_Class.cpp):
 * ```cpp
 * const AP_HAL::HAL& AP_HAL::get_HAL() {
 *     return AP_HAL_Empty;
 * }
 * 
 * AP_HAL::HAL& AP_HAL::get_HAL_mutable() {
 *     return const_cast<HAL_Empty&>(AP_HAL_Empty);
 * }
 * ```
 * 
 * **Usage in application code**:
 * ```cpp
 * #include <AP_HAL/AP_HAL.h>
 * extern const AP_HAL::HAL& hal;
 * 
 * void some_function() {
 *     hal.console->printf("Empty HAL test\n");
 *     hal.scheduler->delay(1000);  // No-op in Empty HAL
 * }
 * ```
 * 
 * The `extern const AP_HAL::HAL& hal;` declaration is typically provided by
 * vehicle code or test harness, referencing the platform-specific singleton
 * (AP_HAL_Empty in this case).
 * 
 * @note Module header exports singleton instances conforming to AP_HAL::HAL interface
 * @note Implementation details (class names, headers) exposed only via Empty namespace
 * @see HAL_Empty_Class.h for singleton extern declaration
 */

/**
 * @brief Compilation guards and board selection
 * 
 * @details The Empty HAL is selected at compile time via CONFIG_HAL_BOARD macro:
 * 
 * Board selection (in AP_HAL_Boards.h):
 * ```cpp
 * #define HAL_BOARD_EMPTY 0
 * 
 * #if CONFIG_HAL_BOARD == HAL_BOARD_EMPTY
 * #define HAL_BOARD_NAME "Empty"
 * // Define Empty capabilities...
 * #endif
 * ```
 * 
 * Build command:
 * ```bash
 * ./waf configure --board=empty
 * ./waf copter
 * ```
 * 
 * All HAL_Empty compilation should be guarded:
 * ```cpp
 * #if CONFIG_HAL_BOARD == HAL_BOARD_EMPTY
 * // Empty HAL implementation
 * #endif
 * ```
 * 
 * This ensures Empty HAL code only compiles when explicitly selected,
 * preventing accidental inclusion in production builds for real hardware.
 * 
 * @note All declaration and compilation guarded by CONFIG_HAL_BOARD == HAL_BOARD_EMPTY
 * @note When creating a new HAL, declare a new HAL_BOARD_ in AP_HAL/AP_HAL_Boards.h
 */

/**
 * @brief Creating a new HAL based on Empty template
 * 
 * @details To port ArduPilot to a new platform using Empty as template:
 * 
 * 1. **Copy Empty directory**:
 *    ```bash
 *    cp -r libraries/AP_HAL_Empty libraries/AP_HAL_NewPlatform
 *    ```
 * 
 * 2. **Rename classes and files**:
 *    - Replace "Empty" with "NewPlatform" in all class names
 *    - Update namespace: `namespace Empty` → `namespace NewPlatform`
 *    - Rename files: AP_HAL_Empty* → AP_HAL_NewPlatform*
 * 
 * 3. **Add board definition** (AP_HAL_Boards.h):
 *    ```cpp
 *    #define HAL_BOARD_NEWPLATFORM 10  // Choose unused number
 *    
 *    #if CONFIG_HAL_BOARD == HAL_BOARD_NEWPLATFORM
 *    #include <AP_HAL_NewPlatform/AP_HAL_NewPlatform.h>
 *    #define HAL_BOARD_NAME "NewPlatform"
 *    #endif
 *    ```
 * 
 * 4. **Replace stub implementations**:
 *    - Start with Scheduler (timing) and UARTDriver (console output)
 *    - Add GPIO for LED blink test
 *    - Incrementally implement remaining interfaces
 * 
 * 5. **Maintain naming conventions**:
 *    - Main class: HAL_NewPlatform
 *    - Singleton: const HAL_NewPlatform AP_HAL_NewPlatform
 *    - Namespace: NewPlatform
 *    - Components: NewPlatform::UARTDriver, etc.
 * 
 * @see libraries/AP_HAL_Empty/README.md for comprehensive porting guide
 * @see libraries/AP_HAL_ChibiOS for complete production HAL example
 * @see libraries/AP_HAL_Linux for Linux-based HAL example
 */

/**
 * @brief Stub implementation characteristics
 * 
 * @details Empty HAL provides minimal implementations with these properties:
 * 
 * **No-op methods** - Return immediately without action:
 * - GPIO write: No actual pin control
 * - Scheduler delay: Returns immediately (time not advanced)
 * - Storage write: Does not persist data
 * 
 * **Deterministic return values** - Fixed values for predictable testing:
 * - AnalogIn: Always returns 1.11 (voltage), 5.0V (board voltage)
 * - RCInput: Returns 1500μs (centered) for most channels, 900μs for channel 3 (low throttle)
 * - GPIO read: Always returns 0
 * - UARTDriver read: Always returns 0 (no data available)
 * 
 * **Minimal state storage** - In-memory buffers for consistency:
 * - RCOutput: Stores PWM values in uint16_t array for read-back
 * - Semaphore: Simple boolean flag (not thread-safe)
 * - DigitalSource: uint8_t value storage for toggle operations
 * 
 * **Always-success operations** - Never fail:
 * - Semaphore take: Always succeeds immediately
 * - UARTDriver write: Reports all bytes written successfully
 * - SPI/I2C transfers: Return true without actual bus communication
 * 
 * These characteristics enable:
 * - Predictable unit test behavior
 * - CI builds without hardware dependencies
 * - Documentation of required interface surface
 * - Template for real implementations
 * 
 * @warning Empty HAL is NOT suitable for:
 * - Actual flight hardware (no hardware control)
 * - Multi-threaded applications (no thread safety)
 * - Performance testing (no realistic timing)
 * - Hardware-in-the-loop testing (no actual I/O)
 */

/**
 * @brief Include dependency structure
 * 
 * @details This file includes only:
 * - AP_HAL/AP_HAL.h: Provides all AP_HAL interface definitions
 * - HAL_Empty_Class.h: Provides HAL_Empty class and AP_HAL_Empty singleton
 * 
 * HAL_Empty_Class.h includes:
 * - AP_HAL/AP_HAL.h: For AP_HAL::HAL base class
 * - AP_HAL_Empty_Namespace.h: For Empty:: forward declarations
 * 
 * This minimal dependency chain ensures fast compilation and prevents
 * circular dependencies. Implementation headers (UARTDriver.h, GPIO.h, etc.)
 * are NOT included here - they are only included in .cpp files that need them.
 */

#include "HAL_Empty_Class.h"
