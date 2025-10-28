#pragma once

/* Your layer exports should depend on AP_HAL.h ONLY. */
#include <AP_HAL/AP_HAL.h>

/**
 * @file AP_HAL_ChibiOS.h
 * @brief Main public header for ChibiOS/ARM HAL implementation
 * 
 * @details Umbrella header providing access to the ChibiOS platform implementation
 *          of ArduPilot's Hardware Abstraction Layer. This is the primary entry point
 *          for accessing ChibiOS-specific HAL classes and the global HAL singleton.
 *          
 *          The ChibiOS HAL implementation supports 150+ ARM-based flight controller boards
 *          including the Pixhawk family (Pixhawk 1/2/4/5/6), CubeOrange/Black/Yellow,
 *          Holybro Kakute series, Matek flight controllers, and many others based on
 *          STM32F4/F7/H7/G4/L4 and IMXRT microcontrollers.
 *          
 *          Platform characteristics:
 *          - RTOS: ChibiOS kernel v20+ with threads, semaphores, and message queues
 *          - MCUs: ARM Cortex-M4/M7 with hardware FPU (single-precision)
 *          - Memory: 256KB-2MB RAM, 1-2MB flash, specialized regions (DTCM, CCM, backup SRAM)
 *          - Peripherals: STM32 native and ChibiOS drivers
 *          - DMA: Shared resource manager prevents channel conflicts
 *          - USB: CDC serial for console, parameter upload, firmware updates
 *          - CAN: bxCAN and FDCAN support for DroneCAN/UAVCAN
 *          - Real-time: Deterministic interrupt priorities, microsecond timing
 *          
 *          Board selection and configuration:
 *          - Boards defined in libraries/AP_HAL_ChibiOS/hwdef/<boardname>/ directories
 *          - Each board has hwdef.dat file specifying MCU, pins, peripherals, features
 *          - Build system (waf) selects board via --board=<boardname> argument
 *          - Conditional compilation via HAL_BOARD_CHIBIOS and board-specific feature flags
 *          
 *          Key implementation classes (inherit from AP_HAL interfaces):
 *          - HAL_ChibiOS: Main HAL singleton aggregating all subsystem drivers
 *          - Scheduler: ChibiOS thread-based cooperative scheduler
 *          - UARTDriver: DMA-accelerated serial with USB CDC support
 *          - SPIDevice/I2CDevice: Bus managers with background I/O threads
 *          - RCOutput: PWM/DShot motor control with DMA waveform generation
 *          - Storage: Flash-backed parameter persistence with wear leveling
 *          - GPIO: Pin control with alternate function routing
 *          - CANIface: CAN bus controller for DroneCAN peripherals
 *          - Shared_DMA: DMA channel arbitration and contention resolution
 *          
 *          Global HAL access pattern:
 *          ```cpp
 *          #include <AP_HAL/AP_HAL.h>
 *          extern const AP_HAL::HAL& hal;  // Defined as HAL_ChibiOS singleton
 *          
 *          void example() {
 *              hal.console->printf("ChibiOS HAL active\\n");
 *              hal.scheduler->delay(1000);
 *          }
 *          ```
 *          
 *          Architecture documentation: See libraries/AP_HAL_ChibiOS/README.md
 *          Board definitions: See libraries/AP_HAL_ChibiOS/hwdef/README.md
 * 
 * @note Application code should include <AP_HAL/AP_HAL.h>, not this header directly
 * @note This header included automatically when CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
 * @warning Modifying HAL interface contract breaks all platform implementations
 * 
 * @see libraries/AP_HAL_ChibiOS/README.md for ChibiOS HAL architecture
 * @see libraries/AP_HAL_ChibiOS/hwdef/README.md for board definition system
 * @see libraries/AP_HAL/README.md for general HAL architecture
 */

#include "HAL_ChibiOS_Class.h"
