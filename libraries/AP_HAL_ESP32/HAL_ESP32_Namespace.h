#pragma once

/**
 * @file HAL_ESP32_Namespace.h
 * @brief ESP32 Hardware Abstraction Layer namespace forward declarations
 * 
 * @details This file provides forward declarations for all classes in the ESP32 namespace,
 *          which implements the ArduPilot Hardware Abstraction Layer (AP_HAL) for ESP32-based
 *          boards. Forward declarations reduce compilation dependencies by allowing header
 *          files to reference these classes without including their full definitions.
 * 
 *          The ESP32 HAL implementation integrates with:
 *          - ESP-IDF (Espressif IoT Development Framework) for hardware access
 *          - FreeRTOS for task scheduling and synchronization primitives
 *          - ESP32 peripheral drivers (UART, SPI, I2C, GPIO, PWM, ADC)
 *          - WiFi and networking capabilities specific to ESP32
 * 
 *          This namespace isolation allows ESP32-specific implementations to coexist
 *          with other platform HALs (ChibiOS, Linux, SITL) in the same codebase.
 * 
 * @note This is a forward declaration header only. See individual class headers in
 *       libraries/AP_HAL_ESP32/ for complete interface definitions.
 * 
 * @see AP_HAL for the abstract HAL interface definitions
 * @see libraries/AP_HAL_ESP32/README.md for ESP32 HAL architecture documentation
 */

/**
 * @namespace ESP32
 * @brief ESP32 Hardware Abstraction Layer implementation namespace
 * 
 * @details The ESP32 namespace contains all platform-specific implementations of the
 *          ArduPilot Hardware Abstraction Layer interfaces for ESP32-based flight
 *          controllers and peripherals. This implementation leverages ESP-IDF and
 *          FreeRTOS to provide:
 * 
 *          - Serial communication (UART, WiFi UDP)
 *          - Real-time task scheduling with FreeRTOS integration
 *          - Persistent storage via ESP32 NVS (Non-Volatile Storage)
 *          - Analog input via ESP32 ADC
 *          - RC input/output using RMT (Remote Control) peripheral
 *          - GPIO and digital I/O
 *          - Thread synchronization primitives (semaphores)
 * 
 *          The ESP32 platform provides unique capabilities:
 *          - Integrated WiFi for telemetry and configuration
 *          - Dual-core Xtensa processors for parallel processing
 *          - Low power consumption for battery-powered applications
 *          - Cost-effective hardware platform
 * 
 * @note All classes in this namespace inherit from corresponding AP_HAL interfaces
 *       and implement ESP32-specific behavior using ESP-IDF APIs.
 */
namespace ESP32
{
    /// @class UARTDriver
    /// @brief ESP32 UART serial port driver implementation using ESP-IDF UART peripheral
    class UARTDriver;
    
    /// @class WiFiDriver
    /// @brief ESP32 WiFi-based serial communication driver for telemetry and ground station connectivity
    class WiFiDriver;
    
    /// @class WiFiUdpDriver
    /// @brief ESP32 WiFi UDP socket driver for MAVLink and telemetry streaming over wireless networks
    class WiFiUdpDriver;
    
    /// @class Scheduler
    /// @brief ESP32 real-time task scheduler using FreeRTOS for periodic callbacks and priority management
    class Scheduler;
    
    /// @class EEPROMStorage
    /// @brief ESP32 EEPROM emulation using NVS (Non-Volatile Storage) flash memory for parameter persistence
    class EEPROMStorage;
    
    /// @class AnalogIn
    /// @brief ESP32 analog input subsystem managing multiple ADC channels for sensor reading
    class AnalogIn;
    
    /// @class RCInput
    /// @brief ESP32 RC receiver input processing using RMT peripheral for PWM/PPM signal decoding
    class RCInput;
    
    /// @class RCOutput
    /// @brief ESP32 RC servo and motor output using PWM via LEDC or MCPWM peripherals
    class RCOutput;
    
    /// @class ADCSource
    /// @brief ESP32 individual ADC channel source for voltage/current sensing
    class ADCSource;
    
    /// @class Util
    /// @brief ESP32 utility functions for system info, safety state, and platform-specific operations
    class Util;
    
    /// @class Semaphore
    /// @brief ESP32 mutual exclusion semaphore using FreeRTOS primitives for thread synchronization
    class Semaphore;
    
    /// @class Semaphore_Recursive
    /// @brief ESP32 recursive mutex allowing same task to acquire lock multiple times
    class Semaphore_Recursive;
    
    /// @class BinarySemaphore
    /// @brief ESP32 binary semaphore for signaling between tasks using FreeRTOS primitives
    class BinarySemaphore;
    
    /// @class GPIO
    /// @brief ESP32 general-purpose I/O control using ESP-IDF GPIO driver
    class GPIO;
    
    /// @class DigitalSource
    /// @brief ESP32 individual digital I/O pin abstraction for reading/writing GPIO states
    class DigitalSource;
    
    /// @class Storage
    /// @brief ESP32 persistent storage backend for parameters and configuration using NVS flash
    class Storage;
    
    /// @class RmtSigReader
    /// @brief ESP32 RMT (Remote Control) peripheral signal reader for precise timing capture of RC signals
    class RmtSigReader;
}  // namespace ESP32
