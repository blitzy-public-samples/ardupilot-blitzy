#pragma once

/**
 * @file AP_HAL_Empty_Private.h
 * @brief Internal umbrella header aggregating all Empty HAL implementation headers
 * 
 * Convenience header that imports all private implementation headers for the
 * AP_HAL_Empty module. Used only within AP_HAL_Empty implementation files
 * (e.g., HAL_Empty_Class.cpp) to access all driver class definitions with
 * a single include.
 * 
 * @note Only import this header from inside AP_HAL_Empty module
 * @note External code should include AP_HAL_Empty.h (public umbrella) instead
 * @note Include order carefully chosen to resolve forward dependencies
 * 
 * @warning Do not include this from application code - breaks encapsulation
 * @warning Adding headers here triggers recompilation of all Empty HAL sources
 * 
 * @details Header inclusion strategy
 * 
 * This umbrella includes all component implementation headers in dependency order:
 * 
 * 1. **Device/Peripheral Drivers** (AnalogIn, GPIO, I2CDevice, SPIDevice, WSPIDevice):
 *    - Low-level hardware interface stubs
 *    - No dependencies on other HAL components
 *    - Can be included in any order
 * 
 * 2. **Input/Output Systems** (RCInput, RCOutput, OpticalFlow):
 *    - RC receiver and motor output stubs
 *    - Flow sensor stub
 *    - Minimal dependencies
 * 
 * 3. **System Services** (Scheduler, Semaphores, Storage, Util):
 *    - Core platform services stubs
 *    - Scheduler provides timing primitives used by others
 *    - Semaphores used for synchronization (though stub is not thread-safe)
 * 
 * 4. **Communication** (UARTDriver):
 *    - Serial port stub for console and telemetry
 *    - May depend on Scheduler for timing
 * 
 * 5. **Advanced Features** (Flash, DSP):
 *    - Flash programming stub (bootloader support)
 *    - DSP/FFT stub (dynamic notch filtering)
 *    - Optional features controlled by HAL_WITH_DSP, etc.
 * 
 * Component headers are self-contained and include only:
 * - AP_HAL_Empty.h (gains access to AP_HAL interfaces)
 * - Standard C++ headers (algorithm, cstring, etc.)
 * 
 * This design minimizes compilation coupling and keeps build times fast.
 * 
 * @details Included component implementation headers
 * 
 * Each included header provides:
 * - Class definition: `class Empty::ComponentName : public AP_HAL::ComponentName`
 * - Method implementations: Either inline in header or in corresponding .cpp file
 * - Stub behavior: No-op, deterministic return values, or minimal state storage
 * 
 * Changing any of these headers triggers recompilation of:
 * - HAL_Empty_Class.cpp (instantiates all driver singletons)
 * - Any other Empty HAL source files including this umbrella
 * 
 * To minimize rebuild impact when modifying a single component:
 * - Include only the specific component header (.cpp files)
 * - Reserve this umbrella for HAL_Empty_Class.cpp where all components needed
 * 
 * @par Usage Example
 * @code
 * // In HAL_Empty_Class.cpp:
 * #include "AP_HAL_Empty_Private.h"
 * 
 * // Now have access to all Empty:: class definitions
 * Empty::UARTDriver console;
 * Empty::GPIO gpio;
 * Empty::Scheduler scheduler;
 * // etc.
 * @endcode
 */

// Device/Peripheral Drivers - Low-level hardware interface stubs
#include "AnalogIn.h"      /**< ADC interface stub - fixed voltage values for testing */
#include "GPIO.h"          /**< Digital I/O stub - no actual hardware pin control */
#include "I2CDevice.h"     /**< I2C bus stub - device manager returns nullptr */
#include "SPIDevice.h"     /**< SPI bus stub - transfers succeed without bus communication */
#include "WSPIDevice.h"    /**< Wide SPI stub - Quad/Octo SPI device allocation returns nullptr */

// Input/Output Systems - RC and sensor interface stubs
#include "OpticalFlow.h"   /**< Optical flow sensor stub - no flow data */
#include "RCInput.h"       /**< RC receiver stub - default channel values (1500µs centered, 900µs ch3) */
#include "RCOutput.h"      /**< Motor/servo output stub - in-memory value storage */

// System Services - Core platform services stubs
#include "Scheduler.h"     /**< Task scheduling stub - delays are no-ops */
#include "Semaphores.h"    /**< Synchronization stub - boolean flag, not thread-safe */
#include "Storage.h"       /**< Parameter storage stub - read zeros, write no-op */
#include "Util.h"          /**< Utility functions stub - safe default implementations */

// Communication - Serial interface stub
#include "UARTDriver.h"    /**< Serial port stub - write succeeds, read returns 0 */

// Advanced Features - Optional capability stubs
#include "Flash.h"         /**< Internal flash stub - page operations succeed without actual flash access */
#include "DSP.h"           /**< DSP/FFT stub - FFT initialization returns nullptr (no actual computation) */
