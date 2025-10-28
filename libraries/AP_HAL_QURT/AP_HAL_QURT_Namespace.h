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
 * @file AP_HAL_QURT_Namespace.h
 * @brief Forward declarations for all QURT HAL classes
 * 
 * This file provides forward declarations for all Qualcomm Hexagon DSP HAL implementation classes.
 * The QURT namespace encapsulates platform-specific implementations of the AP_HAL abstract interfaces
 * for the Qualcomm Hexagon DSP running the QURT real-time operating system.
 * 
 * The QURT HAL implementation enables ArduPilot to run on Qualcomm Snapdragon-based platforms,
 * providing hardware abstraction for DSP-accelerated flight control applications.
 * 
 * @note All classes implement corresponding AP_HAL abstract interfaces defined in libraries/AP_HAL/
 * @see libraries/AP_HAL/ for abstract interface definitions
 * @see interface.h for SL client RPC API definitions used by these classes
 */

#pragma once

/**
 * @namespace QURT
 * @brief Encapsulates all Qualcomm Hexagon DSP HAL implementations
 * 
 * The QURT namespace contains platform-specific implementations of AP_HAL abstract interfaces
 * for the Qualcomm Hexagon DSP platform running the QURT real-time operating system.
 * This namespace follows the ArduPilot HAL pattern, similar to Linux:: namespace for AP_HAL_Linux
 * and ESP32:: for AP_HAL_ESP32.
 * 
 * Key characteristics of the QURT HAL implementation:
 * - Utilizes sl_client RPC API for hardware communication
 * - Integrates with QURT RTOS threading and synchronization primitives
 * - Supports DSP-accelerated sensor processing
 * - Implements framed UART protocol for ESC/servo control and telemetry
 * 
 * @note This is a forward declaration header. Actual implementations are in corresponding .cpp files.
 */
namespace QURT
{
/**
 * @class UARTDriver
 * @brief UART communication driver using sl_client_uart API
 * 
 * Implements UART serial communication for the Hexagon DSP platform using the sl_client_uart API
 * with qurt_rpc framing protocol. Supports hardware flow control and configurable baud rates.
 * 
 * @note Implements AP_HAL::UARTDriver interface
 */
class UARTDriver;

/**
 * @class UARTDriver_Console
 * @brief Console UART driver for debugging output
 * 
 * Specialized UART driver for console/debug output on the Hexagon DSP platform.
 * Typically used for system diagnostics and development logging.
 * 
 * @note Derives from UARTDriver with console-specific configuration
 */
class UARTDriver_Console;

/**
 * @class UARTDriver_MAVLinkUDP
 * @brief UDP-based MAVLink transport driver
 * 
 * Provides UDP socket-based MAVLink communication as an alternative to serial UART.
 * Enables network-based ground control station connectivity.
 * 
 * @note Implements UART interface over UDP transport
 */
class UARTDriver_MAVLinkUDP;

/**
 * @class UARTDriver_Local
 * @brief Local inter-processor UART communication
 * 
 * Specialized UART driver for communication between DSP and application processor
 * on the same Snapdragon SoC.
 * 
 * @note Optimized for low-latency inter-processor messaging
 */
class UARTDriver_Local;

/**
 * @class UDPDriver
 * @brief UDP network socket driver
 * 
 * Provides UDP socket communication capabilities for network-based telemetry,
 * MAVLink, and other protocols on platforms with networking support.
 * 
 * @note Implements AP_HAL::UARTDriver interface over UDP sockets
 */
class UDPDriver;

/**
 * @class Util
 * @brief Platform utility functions and system information
 * 
 * Provides platform-specific utility functions including system time, memory allocation,
 * safety state management, and hardware identification for the Hexagon DSP platform.
 * 
 * @note Implements AP_HAL::Util interface
 */
class Util;

/**
 * @class Scheduler
 * @brief pthread-based task scheduling with QURT timer integration
 * 
 * Implements task scheduling using POSIX threads (pthread) with QURT-specific timer sleep
 * integration. Manages periodic callbacks, timing budgets, and task priorities for
 * flight control loops and background tasks.
 * 
 * @note Implements AP_HAL::Scheduler interface
 */
class Scheduler;

/**
 * @class Storage
 * @brief Emulated EEPROM in RAM with periodic flash persistence
 * 
 * Provides parameter storage using RAM-based EEPROM emulation with periodic flush to
 * flash partition. Implements wear leveling and ensures parameter persistence across reboots.
 * 
 * @note Implements AP_HAL::Storage interface
 */
class Storage;

/**
 * @class Semaphore
 * @brief pthread mutex/condvar synchronization primitives
 * 
 * Provides thread synchronization using pthread mutex and condition variables.
 * Supports priority inheritance and deadlock detection for multi-threaded flight control.
 * 
 * @note Implements AP_HAL::Semaphore interface
 */
class Semaphore;

/**
 * @class RCInput
 * @brief RC channel pulse buffering with periodic polling
 * 
 * Implements RC receiver input handling with channel pulse buffering and periodic polling.
 * Supports multiple RC protocols and provides failsafe detection.
 * 
 * @note Implements AP_HAL::RCInput interface
 */
class RCInput;

/**
 * @class RCOutput
 * @brief ESC/servo control via framed UART protocol with telemetry
 * 
 * Controls electronic speed controllers (ESC) and servos using framed UART protocol.
 * Supports bidirectional communication for ESC telemetry including voltage, current,
 * RPM, and temperature feedback.
 * 
 * @note Implements AP_HAL::RCOutput interface
 */
class RCOutput;

/**
 * @class AnalogSource
 * @brief Single analog input channel abstraction
 * 
 * Represents a single analog input source for voltage or current measurements.
 * Provides filtered readings and scaling conversion.
 * 
 * @note Implements AP_HAL::AnalogSource interface
 */
class AnalogSource;

/**
 * @class AnalogIn
 * @brief Analog input channels delegating to RCOutput for readings
 * 
 * Manages analog input channels for voltage and current measurements.
 * Delegates to RCOutput for actual ADC readings received via ESC telemetry feedback.
 * 
 * @note Implements AP_HAL::AnalogIn interface
 */
class AnalogIn;
}

