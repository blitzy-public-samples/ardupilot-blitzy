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
#pragma once

/**
 * @file AP_HAL_QURT_Private.h
 * @brief Private umbrella header for AP_HAL_QURT module internal implementation
 * 
 * @details This header provides internal-only aggregation of QURT-specific HAL
 *          implementation headers for use within the AP_HAL_QURT module. It serves
 *          as a centralized include point for QURT platform implementation files,
 *          keeping Qualcomm Hexagon DSP-specific implementation details encapsulated
 *          and hidden from other ArduPilot modules.
 *          
 *          The header currently includes:
 *          - UARTDriver.h: QURT UART device implementation
 *          - Util.h: QURT utility functions and platform-specific helpers
 *          
 *          These headers contain QURT RTOS API dependencies including pthreads,
 *          sl_client_* RPC interfaces, and other Qualcomm Hexagon DSP platform-specific
 *          APIs that should not be exposed outside this module.
 * 
 * @warning THIS IS A PRIVATE HEADER - Only include from within AP_HAL_QURT module!
 *          External code must NOT include this header. All external access to QURT
 *          HAL functionality must go through the abstract AP_HAL interfaces defined
 *          in libraries/AP_HAL/*.h. This encapsulation ensures platform independence
 *          and prevents tight coupling to QURT-specific implementation details.
 * 
 * Rationale for Encapsulation:
 * The HAL (Hardware Abstraction Layer) architecture isolates platform-specific
 * implementations behind abstract interfaces. External ArduPilot code depends only
 * on AP_HAL abstract interfaces (UARTDriver, SPIDevice, Scheduler, etc.), allowing
 * the same vehicle and library code to run on multiple platforms (ChibiOS, Linux,
 * ESP32, QURT, SITL) without modification. Including platform-specific private
 * headers would break this abstraction and create unmaintainable platform dependencies.
 * 
 * @note Other HAL platform modules follow the same encapsulation pattern:
 *       - AP_HAL_ChibiOS_Private.h (ARM/ChibiOS RTOS platforms)
 *       - AP_HAL_Linux_Private.h (Linux-based platforms)
 *       - AP_HAL_ESP32_Private.h (ESP32 WiFi platforms)
 *       - AP_HAL_SITL_Private.h (Software-In-The-Loop simulation)
 * 
 * Platform Context:
 * QURT is the Qualcomm Hexagon RTOS running on Qualcomm Snapdragon Flight and
 * similar DSP-based autopilot platforms. This HAL implementation provides ArduPilot
 * support for Hexagon DSP architecture with its unique RTOS and RPC communication model.
 */

#include "UARTDriver.h"
#include "Util.h"
