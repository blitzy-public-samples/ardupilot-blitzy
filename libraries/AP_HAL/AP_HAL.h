/**
 * @file AP_HAL.h
 * @brief Master Hardware Abstraction Layer facade header
 * 
 * Single-include convenience header that brings in complete HAL interface.
 * Included by virtually every ArduPilot source file to access hardware
 * services. Provides deterministic include order to prevent circular dependencies.
 * 
 * @note This is the ONLY HAL header most application code needs to include
 * @note Include order carefully chosen to resolve dependencies
 */

/**
 * @mainpage ArduPilot Hardware Abstraction Layer (AP_HAL)
 * 
 * @section intro_sec Introduction
 * 
 * The ArduPilot Hardware Abstraction Layer (AP_HAL) provides a platform-independent
 * interface to hardware services, enabling the same vehicle code to run on diverse
 * hardware platforms without modification.
 * 
 * @section platforms_sec Supported Platforms
 * 
 * - **ChibiOS (HAL_BOARD_CHIBIOS)**: ARM Cortex-M microcontrollers
 *   - Pixhawk family: Pixhawk 1/2/4/5/6, CubeOrange/Black/Yellow
 *   - Holybro Kakute, Matek flight controllers
 *   - 150+ board variants defined in AP_HAL_ChibiOS/hwdef/
 * 
 * - **Linux (HAL_BOARD_LINUX)**: Linux-based flight controllers
 *   - Navio/Navio2 (Raspberry Pi HAT)
 *   - BeagleBone Blue
 *   - Raspberry Pi with custom hardware
 *   - Intel Edison, Parrot Bebop
 * 
 * - **SITL (HAL_BOARD_SITL)**: Software-In-The-Loop simulation
 *   - Desktop simulation on Linux/macOS/Windows
 *   - Integrates with JSBSim, X-Plane, Gazebo, RealFlight
 *   - Primary development and testing environment
 * 
 * - **ESP32 (HAL_BOARD_ESP32)**: ESP32-based boards
 *   - Low-cost WiFi/Bluetooth enabled boards
 *   - Experimental platform for IoT applications
 * 
 * @section architecture_sec HAL Architecture
 * 
 * The HAL uses object-oriented design with pure virtual interfaces:
 * 
 * 1. **Interface Layer** (libraries/AP_HAL/*.h):
 *    - Abstract base classes defining hardware contracts
 *    - Pure virtual methods that platforms must implement
 *    - Platform-independent data structures
 * 
 * 2. **Platform Layer** (libraries/AP_HAL_*):
 *    - Concrete implementations for each platform
 *    - Driver code for specific hardware peripherals
 *    - RTOS integration (ChibiOS, FreeRTOS, Linux)
 * 
 * 3. **Application Layer** (ArduCopter, ArduPlane, libraries):
 *    - Vehicle code and libraries
 *    - Uses HAL interfaces via global 'hal' object
 *    - Platform-agnostic (compiles for all platforms)
 * 
 * @section usage_sec Basic Usage
 * 
 * Every ArduPilot source file includes AP_HAL.h and declares extern hal:
 * @code{.cpp}
 * #include <AP_HAL/AP_HAL.h>
 * extern const AP_HAL::HAL& hal;
 * 
 * void example_function() {
 *     // Serial output
 *     hal.console->printf("System initialized\n");
 *     
 *     // Delays
 *     hal.scheduler->delay(1000);  // 1 second delay
 *     
 *     // Digital I/O
 *     hal.gpio->pinMode(LED_PIN, HAL_GPIO_OUTPUT);
 *     hal.gpio->write(LED_PIN, 1);
 *     
 *     // Timing
 *     uint32_t now = AP_HAL::millis();
 * }
 * @endcode
 * 
 * @section interfaces_sec Key Interfaces
 * 
 * - **UARTDriver**: Serial ports for telemetry, GPS, peripherals
 * - **I2CDevice, SPIDevice**: Bus protocols for sensors
 * - **GPIO**: Digital I/O pins for LEDs, relays
 * - **AnalogIn**: ADC for voltage/current monitoring
 * - **RCInput, RCOutput**: RC receiver input, motor/servo output
 * - **Scheduler**: Task scheduling, delays, timing
 * - **Storage**: Parameter persistence across reboots
 * - **Semaphore**: Thread synchronization
 * - **Util**: System info, memory allocation, safety state
 * 
 * @section porting_sec Porting to New Platform
 * 
 * To port ArduPilot to a new platform:
 * 
 * 1. Create libraries/AP_HAL_NewPlatform directory
 * 2. Implement all pure virtual HAL interfaces
 * 3. Define HAL_BOARD_NEWPLATFORM in AP_HAL_Boards.h
 * 4. Create platform-specific HAL object in HAL_NewPlatform_Class.cpp
 * 5. Implement get_HAL() and get_HAL_mutable() functions
 * 6. Add build system support in wscript
 * 7. Test with autotest suite
 * 
 * See libraries/AP_HAL_Empty for stub implementation template.
 * 
 * @section design_principles Design Principles
 * 
 * - **Zero-cost abstraction**: Virtual function overhead acceptable for infrequent ops
 * - **Compile-time selection**: Platform selected at build time, no runtime overhead
 * - **Minimal dependencies**: HAL depends only on standard C/C++ libraries
 * - **Thread-safe**: Interfaces designed for multi-threaded access
 * - **Single-precision FP**: Double-precision prohibited on embedded targets
 * 
 * @section performance Performance Considerations
 * 
 * - Virtual function calls: ~10ns overhead on modern ARM Cortex-M7
 * - Inlining: HAL calls in hot paths should be profiled and optimized
 * - DMA: Use for bulk transfers (SPI sensor reads, UART telemetry)
 * - Interrupts: Minimize work in ISRs, defer to scheduler tasks
 * - Memory: Static allocation preferred, dynamic allocation for config-time only
 * 
 * @see AP_HAL::HAL for main aggregator class
 * @see AP_HAL_Boards.h for platform identification
 * @see AP_HAL_Namespace.h for forward declarations
 */

#pragma once

#include <stdint.h>

/**
 * @brief Include order and dependencies
 * 
 * Include order is critical for resolving dependencies:
 * 1. AP_HAL_Namespace.h - Forward declarations and namespace definition
 * 2. AP_HAL_Boards.h - Board identification and capabilities
 * 3. AP_HAL_Macros.h - Compile policies (double-precision restrictions)
 * 4. AP_HAL_Main.h - Main entry point macros
 * 5. Individual interface headers - UARTDriver, GPIO, Scheduler, etc.
 * 6. HAL.h - Main HAL aggregator class
 * 7. system.h - Timing and diagnostic functions
 * 
 * @note Do not change include order - build will break
 * @note Applications should include only AP_HAL.h, not individual headers
 */

#include "AP_HAL_Namespace.h"
#include "AP_HAL_Boards.h"
#include "AP_HAL_Macros.h"
#include "AP_HAL_Main.h"

/* HAL Module Classes (all pure virtual) */
#include "UARTDriver.h"
#include "AnalogIn.h"
#include "Storage.h"
#include "GPIO.h"
#include "RCInput.h"
#include "RCOutput.h"
#include "Scheduler.h"
#include "Semaphores.h"
#include "Util.h"
#include "OpticalFlow.h"
#include "Flash.h"
#include "DSP.h"

#include "CANIface.h"

#include "utility/BetterStream.h"

/* HAL Class definition */
#include "HAL.h"

#include "system.h"

/**
 * @brief Global HAL Object Usage Pattern
 * 
 * After including this header, application code must declare an extern reference
 * to the global HAL singleton object:
 * 
 * @code{.cpp}
 * extern const AP_HAL::HAL& hal;
 * @endcode
 * 
 * The concrete HAL object is defined by each platform implementation:
 * - ChibiOS: Defined in libraries/AP_HAL_ChibiOS/HAL_ChibiOS_Class.cpp
 * - Linux: Defined in libraries/AP_HAL_Linux/HAL_Linux_Class.cpp
 * - SITL: Defined in libraries/AP_HAL_SITL/HAL_SITL_Class.cpp
 * - ESP32: Defined in libraries/AP_HAL_ESP32/HAL_ESP32_Class.cpp
 * 
 * The hal object is accessed throughout ArduPilot to interact with hardware:
 * @code{.cpp}
 * // Serial I/O
 * hal.console->printf("Message\n");
 * hal.serial(0)->write("data", 4);
 * 
 * // Timing and delays
 * uint32_t now = AP_HAL::millis();
 * hal.scheduler->delay(100);
 * 
 * // Digital I/O
 * hal.gpio->pinMode(pin, HAL_GPIO_OUTPUT);
 * hal.gpio->write(pin, value);
 * 
 * // SPI/I2C devices
 * AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev = hal.i2c_mgr->get_device(bus, addr);
 * AP_HAL::OwnPtr<AP_HAL::SPIDevice> spi = hal.spi->get_device("sensor");
 * 
 * // Storage
 * hal.storage->read_block(buffer, offset, length);
 * 
 * // Utilities
 * void* mem = hal.util->malloc_type(size, AP_HAL::Util::MEM_DMA_SAFE);
 * @endcode
 * 
 * @note The hal object is const to prevent accidental modification of hardware interfaces
 * @note Platform implementations populate hal with concrete driver instances during init
 * @note Never create your own HAL object - always use the global hal singleton
 * 
 * @see AP_HAL::get_HAL() for accessor function returning the hal singleton
 * @see AP_HAL_Namespace.h for forward declaration of get_HAL() and get_HAL_mutable()
 */
