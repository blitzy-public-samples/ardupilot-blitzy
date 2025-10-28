/**
 * @file AP_HAL_Namespace.h
 * @brief Forward declarations for HAL namespace classes
 * 
 * @details Provides lightweight forward declarations of all HAL interface classes within
 *          the AP_HAL namespace. Enables header files to reference HAL types without
 *          circular dependencies. Mandates global HAL accessor functions that platforms
 *          must implement.
 * 
 * @note Included by nearly every header - keep minimal and fast to compile
 * @note Changes here trigger near-complete rebuild
 * 
 * Source: libraries/AP_HAL/AP_HAL_Namespace.h
 */

#pragma once

#include "string.h"
#include "utility/functor.h"

/**
 * @namespace AP_HAL
 * @brief Hardware Abstraction Layer namespace containing all HAL interfaces
 * 
 * @details The AP_HAL namespace encapsulates all hardware abstraction classes,
 *          preventing naming conflicts with platform-specific implementations.
 *          
 *          Organization:
 *          - Interface classes: UARTDriver, SPIDevice, I2CDevice, GPIO, etc.
 *          - Data structures: CANFrame, AnalogSource, SPIDeviceType, etc.
 *          - Manager classes: SPIDeviceManager, I2CDeviceManager
 *          - Utilities: Semaphore, Scheduler, Util
 *          
 *          Platform implementations (e.g., AP_HAL::ChibiOS, AP_HAL::Linux) provide
 *          concrete classes inheriting from these abstract interfaces.
 * 
 * @note All HAL interfaces are pure virtual - platforms must implement
 * @note Forward-declared here, defined in individual header files
 */
namespace AP_HAL {

    /**
     * @class HAL
     * @brief Toplevel pure virtual class aggregating all HAL subsystems
     * 
     * @details Main HAL interface that provides access to all hardware subsystems.
     *          Defined in AP_HAL.h. Each platform provides a concrete implementation.
     */
    class HAL;

    /**
     * @brief Core driver interface forward declarations
     * 
     * @details Forward-declares primary HAL interface classes to enable circular references
     *          without pulling in full headers. Keeps compilation fast and dependencies minimal.
     *          
     *          Each class represents a hardware abstraction interface:
     *          - UARTDriver: Serial port interface
     *          - I2CDevice, I2CDeviceManager: I2C bus interfaces
     *          - Device: Base device class for sensors
     */
    class UARTDriver;
    class I2CDevice;
    class I2CDeviceManager;
    class Device;

    /**
     * @brief SPI (Serial Peripheral Interface) device abstractions
     * 
     * @details SPIDevice: Individual SPI device interface
     *          SPIDeviceDriver: Legacy SPI driver interface (deprecated)
     *          SPIDeviceManager: Factory for creating SPI devices
     */
    class SPIDevice;
    class SPIDeviceDriver;
    class SPIDeviceManager;

    /**
     * @brief Analog and digital I/O interfaces
     * 
     * @details AnalogSource: Single ADC channel interface
     *          AnalogIn: Multi-channel ADC manager
     *          Storage: Persistent parameter storage interface
     *          DigitalSource: Digital input interface
     *          PWMSource: PWM input capture interface
     *          GPIO: General purpose I/O interface
     */
    class AnalogSource;
    class AnalogIn;
    class Storage;
    class DigitalSource;
    class PWMSource;
    class GPIO;
    
    /**
     * @brief RC (Radio Control) input and output interfaces
     * 
     * @details RCInput: RC receiver input interface
     *          RCOutput: PWM output interface for servos/ESCs
     */
    class RCInput;
    class RCOutput;
    
    /**
     * @brief System services and utilities
     * 
     * @details Scheduler: Task scheduling and timing interface
     *          Semaphore: Counting semaphore for thread synchronization
     *          BinarySemaphore: Binary semaphore for mutual exclusion
     *          OpticalFlow: Optical flow sensor interface
     *          DSP: Digital signal processing interface
     */
    class Scheduler;
    class Semaphore;
    class BinarySemaphore;
    class OpticalFlow;
    class DSP;

    /**
     * @brief Wide SPI (WSPI/QuadSPI) device abstractions
     * 
     * @details Extended SPI interface supporting quad/octal SPI modes
     *          for high-speed flash memory and sensors.
     */
    class WSPIDevice;
    class WSPIDeviceDriver;
    class WSPIDeviceManager;

    /**
     * @brief CAN bus interface
     * 
     * @details CANIface: CAN interface controller
     *          CANFrame: CAN message frame structure
     */
    class CANIface;
    class CANFrame;

    /**
     * @brief Utility and flash programming interfaces
     * 
     * @details Util: Platform utility functions (safety state, timing, etc.)
     *          Flash: Internal flash memory programming interface
     */
    class Util;
    class Flash;

    /**
     * @brief I/O stream utility classes
     * 
     * @details Print: Basic output interface (print, println)
     *          Stream: Bidirectional stream with read/write
     *          BetterStream: Enhanced stream with additional features
     *          
     *          These provide Arduino-compatible I/O abstractions
     *          used primarily by console and telemetry interfaces.
     */
    class Print;
    class Stream;
    class BetterStream;

    /**
     * @brief Function pointer typedefs for callbacks
     * 
     * @details Proc: Simple function pointer for void(void) callbacks
     *          MemberProc: Functor encapsulating member function callbacks
     *          
     *          For member functions we use the FastDelegate pattern via FUNCTOR_TYPEDEF
     *          which allows us to encapsulate a member function as a callable type.
     *          This enables scheduler callbacks and event handlers to invoke both
     *          static functions and member functions uniformly.
     * 
     * @see AP_HAL::Scheduler::register_timer_process()
     * @see utility/functor.h for FUNCTOR_TYPEDEF implementation
     */
    typedef void(*Proc)(void);
    FUNCTOR_TYPEDEF(MemberProc, void);

    /**
     * @enum SPIDeviceType
     * @brief Legacy SPI device type enumeration
     * 
     * @details Global enumeration for SPI device types on all platforms.
     *          Modern code uses AP_HAL::SPIDevice abstraction instead.
     *          
     *          SPIDevice_Type = -1: Indicates use of AP_HAL::SPIDevice interface
     * 
     * @note This enum is largely deprecated in favor of device manager approach
     * @deprecated Prefer AP_HAL::SPIDeviceManager::get_device()
     */
    enum SPIDeviceType {
        // Devices using AP_HAL::SPIDevice abstraction
        SPIDevice_Type              = -1,
    };

    /**
     * @class SIMState
     * @brief Simulation state access interface for SITL
     * 
     * @details Provides access to SITL (Software In The Loop) simulation state
     *          including simulated sensors, vehicle dynamics, and environment.
     *          Only available in SITL builds.
     */
    class SIMState;

    /**
     * @brief Global HAL accessor function (const)
     * 
     * @details Returns reference to platform-specific HAL singleton instance.
     *          Must be implemented by each concrete HAL implementation
     *          (ChibiOS, Linux, ESP32, SITL, etc.).
     *          
     *          The HAL object provides access to all hardware subsystems:
     *          - hal.console: Primary serial console
     *          - hal.scheduler: Task scheduler
     *          - hal.storage: Persistent parameter storage
     *          - hal.uartA/B/C/etc: Serial ports
     *          - hal.i2c_mgr: I2C device manager
     *          - hal.spi: SPI device manager
     *          - hal.gpio: Digital I/O
     *          - hal.rcin: RC input
     *          - hal.rcout: PWM output
     *          - hal.analogin: ADC input
     *          - hal.util: Platform utilities
     * 
     * @return const AP_HAL::HAL& Reference to global HAL object
     * 
     * @note Available after early initialization, before main()
     * @note Most code uses extern hal declaration instead of calling get_HAL()
     * @note Platform-specific implementation in AP_HAL_XXX/HAL_XXX_Class.cpp
     * 
     * @see AP_HAL::HAL class definition in AP_HAL.h
     */
    const HAL& get_HAL();
    
    /**
     * @brief Global HAL accessor function (mutable)
     * 
     * @details Returns mutable reference to platform-specific HAL singleton.
     *          Used rarely, primarily during HAL initialization and testing.
     *          Application code should use const get_HAL() or extern hal instead.
     * 
     * @return AP_HAL::HAL& Mutable reference to global HAL object
     * 
     * @note Use sparingly - prefer const get_HAL() for read-only access
     * @note Mainly used by HAL implementation code, not application code
     * @warning Modifying HAL state after initialization may cause undefined behavior
     */
    HAL& get_HAL_mutable();

} // namespace AP_HAL

/**
 * @example Typical HAL usage pattern
 * 
 * Most ArduPilot code uses an extern declaration to access the HAL singleton:
 * 
 * @code{.cpp}
 * #include <AP_HAL/AP_HAL.h>
 * 
 * // Extern declaration in source file
 * extern const AP_HAL::HAL& hal;
 * 
 * void some_function() {
 *     // Access serial console
 *     hal.console->printf("Hello from ArduPilot\n");
 *     
 *     // Delay using scheduler
 *     hal.scheduler->delay(1000);  // 1000 ms delay
 *     
 *     // Register periodic callback
 *     hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&MyClass::update, void));
 *     
 *     // Access I2C device
 *     auto dev = hal.i2c_mgr->get_device(0, 0x68);  // Bus 0, address 0x68
 *     
 *     // Read analog input
 *     float voltage = hal.analogin->channel(0)->voltage_average();
 * }
 * @endcode
 * 
 * @note The 'hal' extern is defined by each platform's HAL implementation
 * @note In HAL implementation code, use get_HAL() or get_HAL_mutable() instead
 */
