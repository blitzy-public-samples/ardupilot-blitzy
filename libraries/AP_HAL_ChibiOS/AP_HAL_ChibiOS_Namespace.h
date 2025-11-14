/**
 * @file AP_HAL_ChibiOS_Namespace.h
 * @brief Forward declarations for ChibiOS HAL implementation classes
 * 
 * Provides lightweight forward declarations of all ChibiOS-specific HAL implementation
 * classes within the ChibiOS namespace. Enables header files to reference ChibiOS types
 * without circular dependencies while keeping compilation fast.
 * 
 * @note Included by AP_HAL_ChibiOS.h and platform-specific headers
 * @note Changes here trigger rebuild of ChibiOS HAL components
 */

#pragma once

/**
 * @namespace ChibiOS
 * @brief ChibiOS/ARM platform implementation namespace
 * 
 * @details Encapsulates all ChibiOS-specific implementations of AP_HAL interfaces for
 *          ARM Cortex-M microcontrollers using the ChibiOS RTOS. This namespace prevents
 *          naming conflicts with other platform implementations (Linux, SITL, ESP32).
 *          
 *          ChibiOS platform characteristics:
 *          - Target MCUs: STM32F4/F7/H7/G4/L4, IMXRT, and other ARM Cortex-M
 *          - RTOS: ChibiOS kernel with threads, semaphores, message queues
 *          - Memory: Carefully managed RAM regions (DTCM, CCM, AXI RAM, backup SRAM)
 *          - Peripherals: Native STM32 HAL and ChibiOS drivers (SPI, I2C, UART, CAN, USB, ADC)
 *          - DMA: Shared DMA resource manager with contention resolution
 *          - Interrupts: ChibiOS interrupt priority system
 *          - Board definitions: hwdef system defines pin mappings and features per board
 *          
 *          Key implementation classes (all inherit from AP_HAL base interfaces):
 *          - Scheduler: ChibiOS thread-based task scheduling with priority management
 *          - UARTDriver: DMA-accelerated serial ports with USB CDC support
 *          - SPIDevice/I2CDevice: Bus managers with background I/O threads
 *          - GPIO: STM32 pin control with alternate function mapping
 *          - RCOutput: PWM/DShot motor control with DMA waveform generation
 *          - RCInput: PPM/SBUS/DSM/CRSF receiver decoding
 *          - Storage: Flash-backed parameter storage with wear leveling
 *          - CANIface: bxCAN and FDCAN controller support for DroneCAN
 *          
 *          150+ board variants supported via hwdef files in hwdef/ directory.
 *          Each board has unique pin mappings, sensor configurations, and features.
 * 
 * @note Forward-declared here, fully defined in respective implementation headers
 * @see libraries/AP_HAL_ChibiOS/README.md for architecture overview
 * @see libraries/AP_HAL_ChibiOS/hwdef/README.md for board definition system
 */
namespace ChibiOS {
    class AnalogIn;          ///< ADC (Analog-to-Digital Converter) manager for voltage/current sensing
    class AnalogSource;      ///< Individual ADC channel wrapper for analog sensors
    class DigitalSource;     ///< Digital input pin reader for GPIO
#if HAL_WITH_IO_MCU
    class IOMCU_DigitalSource;  ///< Digital input via I/O coprocessor (Pixhawk IOMCU)
#endif
    class DSP;               ///< Hardware-accelerated FFT for gyro notch filtering
    class GPIO;              ///< Digital I/O pin control with STM32 alternate function mapping
    class I2CBus;            ///< I2C bus manager with background thread for sensor polling
    class I2CDevice;         ///< I2C device instance with DMA and error recovery
    class I2CDeviceManager;  ///< I2C device registry and bus arbitration
    class OpticalFlow;       ///< Optical flow sensor interface (legacy, rarely used)
    class RCInput;           ///< RC receiver input decoder (PPM/SBUS/DSM/CRSF/IBUS/etc.)
    class RCOutput;          ///< PWM/DShot motor and servo output with DMA waveform generation
    class Scheduler;         ///< ChibiOS thread-based task scheduler with priority management
    class Semaphore;         ///< Counting semaphore for resource protection and synchronization
    class BinarySemaphore;   ///< Binary semaphore for mutual exclusion (mutex)
    class SPIBus;            ///< SPI bus manager with DMA and background transfer thread
    class SPIDesc;           ///< SPI device descriptor (board-specific pin/bus configuration)
    class SPIDevice;         ///< SPI device instance for sensor communication
    class SPIDeviceDriver;   ///< Low-level SPI transaction driver with chip select management
    class SPIDeviceManager;  ///< SPI device registry and bus arbitration
    class WSPIBus;           ///< Quad/Octo SPI bus manager for high-speed external flash
    class WSPIDesc;          ///< WSPI device descriptor (board-specific QSPI/OSPI configuration)
    class WSPIDevice;        ///< WSPI device instance for external flash memory
    class WSPIDeviceManager; ///< WSPI device registry for external flash support
    class Storage;           ///< Persistent parameter storage using internal flash with wear leveling
    class UARTDriver;        ///< DMA-accelerated serial port (UART/USART/USB CDC)
    class Util;              ///< Platform utilities (memory allocation, safety state, system info)
    class Shared_DMA;        ///< DMA resource arbitration and contention management
    class SoftSigReader;     ///< Software-based RC signal decoder (PPM/SBUS/etc.)
    class SoftSigReaderInt;  ///< Interrupt-driven software signal decoder variant
    class CANIface;          ///< CAN bus interface (bxCAN/FDCAN) for DroneCAN/UAVCAN peripherals
    class Flash;             ///< Internal flash memory programming interface for bootloader and storage
}
