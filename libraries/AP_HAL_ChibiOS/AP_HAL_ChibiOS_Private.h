#pragma once

/**
 * @file AP_HAL_ChibiOS_Private.h
 * @brief Umbrella header aggregating all private ChibiOS HAL implementation headers
 * 
 * @details Convenience header that includes all ChibiOS-specific implementation headers
 *          for use within the AP_HAL_ChibiOS module. This provides single-include access
 *          to all platform drivers and internal implementation details.
 *          
 *          Included implementation headers:
 *          - AnalogIn.h: ADC sampling and voltage monitoring
 *          - GPIO.h: Digital I/O pin control with STM32 alternate functions
 *          - Scheduler.h: ChibiOS thread-based task scheduler
 *          - Util.h: Platform utilities (memory, safety, system info)
 *          - UARTDriver.h: DMA-accelerated serial ports with USB CDC
 *          - SPIDevice.h: SPI bus management with background I/O threads
 *          - Storage.h: Flash-backed parameter storage with wear leveling
 *          - RCInput.h: RC receiver protocol decoding (PPM/SBUS/DSM/CRSF)
 *          - RCOutput.h: PWM/DShot motor control with DMA waveform generation
 *          - I2CDevice.h: I2C bus management with retry logic
 *          - Flash.h: Internal flash programming for bootloader and storage
 *          - DSP.h: Hardware-accelerated FFT operations
 *          - WSPIDevice.h: Quad/Octo SPI for external flash memory
 *          
 *          Usage restrictions:
 *          - INTERNAL ONLY: Include ONLY from within AP_HAL_ChibiOS implementation files
 *          - NOT for application code: Vehicle and library code should include <AP_HAL/AP_HAL.h>
 *          - NOT for other HAL platforms: Linux/SITL/ESP32 have their own Private headers
 *          
 *          Purpose:
 *          - Simplifies implementation file headers (single include instead of many)
 *          - Ensures consistent include order across implementation files
 *          - Reduces risk of circular dependencies through controlled aggregation
 * 
 * @note This header exposes internal implementation details - use sparingly
 * @warning Including from outside AP_HAL_ChibiOS breaks platform abstraction
 * @warning Do not add platform-specific code paths in vehicle/library code using these headers
 * 
 * @see libraries/AP_HAL_ChibiOS/README.md for architecture overview
 */

// NOTE: Include order carefully chosen to resolve internal dependencies
// Do not reorder without understanding dependency graph

#include "AnalogIn.h"
#include "GPIO.h"
#include "Scheduler.h"
#include "Util.h"
#include "UARTDriver.h"
#include "SPIDevice.h"
#include "Storage.h"
#include "RCInput.h"
#include "RCOutput.h"
#include "I2CDevice.h"
#include "Flash.h"
#include "DSP.h"
#include "WSPIDevice.h"
