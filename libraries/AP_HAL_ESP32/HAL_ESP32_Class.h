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
 * @file HAL_ESP32_Class.h
 * @brief HAL_ESP32 class declaration and central HAL wiring for ESP32 platform
 * 
 * @details This header declares the concrete HAL implementation for the ESP32
 *          platform, which provides the hardware abstraction layer between
 *          ArduPilot vehicle code and ESP32-specific drivers and hardware interfaces.
 *          
 *          The corresponding implementation file (HAL_ESP32_Class.cpp) instantiates
 *          all static driver objects that form the complete HAL system for ESP32.
 *          
 *          This HAL implementation is built on top of the ESP-IDF (Espressif IoT
 *          Development Framework) and FreeRTOS operating system.
 * 
 * @note ESP-IDF Integration: The esp_idf module must be properly initialized
 *       before any HAL functionality is used. This includes FreeRTOS scheduler
 *       initialization and ESP32 peripheral setup.
 * 
 * @warning FreeRTOS Requirements: ESP32 has limited RAM compared to ARM-based
 *          flight controllers. Memory constraints must be considered in all
 *          ArduPilot configurations. The FreeRTOS scheduler and task stack
 *          allocations directly impact available memory for ArduPilot operations.
 * 
 * @see AP_HAL::HAL Base HAL interface
 * @see HAL_ESP32_Namespace.h Driver forward declarations
 * @see HAL_ESP32_Class.cpp Driver instantiation and initialization
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_Empty/AP_HAL_Empty_Namespace.h>
#include <AP_HAL_ESP32/HAL_ESP32_Namespace.h>

/**
 * @class HAL_ESP32
 * @brief Concrete ESP32 HAL implementation
 * 
 * @details HAL_ESP32 is the main Hardware Abstraction Layer class for the ESP32
 *          platform. It inherits from AP_HAL::HAL and provides the complete set
 *          of driver objects required for ArduPilot operation on ESP32-based boards.
 *          
 *          The implementation file (HAL_ESP32_Class.cpp) instantiates all static
 *          driver objects including:
 *          - Console UART driver for debugging output
 *          - Serial port drivers for telemetry and GPS
 *          - I2C manager for I2C peripheral devices (sensors, compass, etc.)
 *          - SPI manager for SPI peripheral devices (IMU, baro, etc.)
 *          - GPIO driver for digital I/O pin control
 *          - Analog input driver for voltage/current sensing
 *          - Storage driver for parameter and configuration persistence
 *          - Scheduler for task management and timing
 *          - Utility functions (timing, system info, etc.)
 *          - RC input processing for receiver input
 *          - RC output (PWM/servo) driver for motor and servo control
 *          
 *          A global singleton instance (hal_esp32) is defined in system.cpp and
 *          provides system-wide access to HAL functionality via the AP_HAL::get_HAL()
 *          accessor.
 *          
 * @note Conditional Driver Selection: Some drivers are conditionally compiled
 *       based on feature macros:
 *       - HAL_ESP32_WIFI: Enables WiFi networking support
 *       - HAL_WITH_DSP: Enables digital signal processing features
 *       - HAL_DISABLE_ADC_DRIVER: Disables analog-to-digital converter
 *       - AP_SIM_ENABLED: Enables simulation mode adaptations
 *       
 * @note Platform Specifics: ESP32 uses the Xtensa or RISC-V architecture
 *       (depending on chip variant) and runs FreeRTOS as the underlying RTOS.
 *       All timing, interrupts, and concurrency are managed through FreeRTOS
 *       primitives.
 * 
 * @warning Memory Constraints: ESP32 typically has 520KB of on-chip SRAM
 *          compared to megabytes available on ARM flight controllers. Driver
 *          implementations must be memory-efficient. Large buffers and data
 *          structures should be avoided or carefully managed.
 * 
 * @see AP_HAL::HAL Base class interface definition
 * @see HAL_ESP32_Namespace.h Forward declarations for all ESP32 driver types
 * @see system.cpp Global hal_esp32 instance definition
 */
class HAL_ESP32 : public AP_HAL::HAL
{
public:
    /**
     * @brief Construct the HAL_ESP32 instance
     * 
     * @details The constructor initializes the HAL object by passing pointers to
     *          all static ESP32 driver instances to the base AP_HAL::HAL class.
     *          These driver objects are defined in HAL_ESP32_Class.cpp as file-scope
     *          static instances and include all hardware interface drivers (UARTs,
     *          I2C, SPI, GPIO, ADC, storage, scheduler, RC I/O, etc.).
     *          
     *          The constructor does not perform hardware initialization; that occurs
     *          later during the boot sequence when init() is called on individual
     *          drivers.
     *          
     * @note This constructor is called during static initialization before main()
     *       to create the global hal_esp32 singleton instance.
     * 
     * @see HAL_ESP32_Class.cpp Static driver object definitions
     * @see AP_HAL::HAL::HAL() Base class constructor
     */
    HAL_ESP32();
    
    /**
     * @brief Start the main scheduler loop
     * 
     * @details This method initializes the ArduPilot scheduler and enters the
     *          main execution loop. It performs the following sequence:
     *          1. Initializes all HAL drivers (calls init() on each driver)
     *          2. Invokes the setup() callback to run vehicle-specific initialization
     *          3. Starts the main scheduler loop by calling loop() callback repeatedly
     *          
     *          This method NEVER returns under normal operation. The scheduler runs
     *          continuously, executing tasks at their configured rates until the
     *          system is powered off or reset.
     *          
     *          The scheduler uses FreeRTOS primitives for task management and timing.
     *          High-priority tasks (e.g., IMU sampling at 1kHz, attitude control at
     *          400Hz) are executed with precise timing, while lower-priority tasks
     *          (e.g., telemetry, logging) run at slower rates.
     * 
     * @param[in] argc Command-line argument count (typically unused on embedded ESP32)
     * @param[in] argv Command-line argument values (typically unused on embedded ESP32)
     * @param[in] callbacks Pointer to vehicle-specific callbacks structure containing
     *                      setup() and loop() function pointers
     * 
     * @note This method is called from main() in system.cpp after ESP-IDF and
     *       FreeRTOS initialization is complete.
     * 
     * @note Command-line arguments: While argc/argv are part of the HAL interface
     *       for compatibility with Linux/SITL implementations, they are typically
     *       not used on embedded ESP32 platforms.
     * 
     * @warning This method never returns. Any cleanup or shutdown logic must be
     *          implemented within the scheduler callbacks or as registered shutdown
     *          handlers.
     * 
     * @see AP_HAL::HAL::run() Base class virtual method
     * @see AP_Scheduler Main task scheduler implementation
     * @see system.cpp ESP32 main() entry point
     */
    void run(int argc, char* const* argv, Callbacks* callbacks) const override;
};
