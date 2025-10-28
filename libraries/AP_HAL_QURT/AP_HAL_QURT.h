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
 * @file AP_HAL_QURT.h
 * @brief Main QURT HAL entry point for Qualcomm Hexagon DSP platform
 * 
 * @details This file serves as the primary include header for the QURT (Qualcomm User Real-Time)
 *          Hardware Abstraction Layer implementation in ArduPilot. It provides the bootstrapping
 *          includes for CONFIG_HAL_BOARD == HAL_BOARD_QURT builds targeting Qualcomm Snapdragon
 *          platforms with Hexagon DSP processors.
 * 
 * Platform Architecture:
 * ---------------------
 * The QURT HAL enables ArduPilot to run on Qualcomm Snapdragon platforms featuring Hexagon DSP
 * coprocessors (ADSP - Application DSP, or cDSP - Compute DSP). This implementation integrates
 * with the Qualcomm QURT RTOS (QuRT - Qualcomm User Real-Time Operating System) running on the
 * Hexagon DSP alongside the main ARM application processor running Linux.
 * 
 * Target Platforms:
 * ----------------
 * - Snapdragon Flight: Original development platform with Hexagon 801 DSP
 * - VOXL (Vehicle Open eXperimental Linux): ModalAI platforms with Snapdragon 821
 * - VOXL 2: Next-generation platform with Snapdragon QRB5165 and Hexagon 698 DSP
 * - Custom Snapdragon-based autopilots using Hexagon DSP offload
 * 
 * Compilation:
 * -----------
 * This header is only active when CONFIG_HAL_BOARD is set to HAL_BOARD_QURT during
 * the build configuration. The conditional compilation ensures QURT-specific code
 * is only included for Hexagon DSP target builds.
 * 
 * HAL Integration:
 * ---------------
 * Depends on the AP_HAL abstraction layer (AP_HAL/AP_HAL.h) which defines the
 * platform-independent hardware interface contracts. This header includes:
 * - HAL_QURT_Class.h: Main HAL implementation class for QURT platform
 * - AP_HAL_QURT_Main.h: Platform bootstrapping and main entry point logic
 * 
 * @note QURT Platform Specifics:
 *       - Uses POSIX pthreads for threading abstraction on QuRT RTOS
 *       - Utilizes QURT timer API for high-precision timing on Hexagon DSP
 *       - Employs SL (Sensor Low-latency) client RPC mechanism for hardware access
 *       - Communicates with ARM Linux host via FastRPC for I/O operations
 *       - DSP execution provides deterministic real-time performance for flight control
 * 
 * @see HAL_QURT_Class
 * @see AP_HAL::HAL
 * @see libraries/AP_HAL_QURT/README.md for platform setup and porting guide
 */
#pragma once

/* Your layer exports should depend on AP_HAL.h ONLY. */
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_QURT

#include "HAL_QURT_Class.h"
#include "AP_HAL_QURT_Main.h"

#endif // CONFIG_HAL_BOARD
