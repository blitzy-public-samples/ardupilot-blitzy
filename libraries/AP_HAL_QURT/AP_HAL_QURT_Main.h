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
 * @file AP_HAL_QURT_Main.h
 * @brief QURT platform-specific main entry point definition for ArduPilot
 * 
 * @details This file defines the AP_MAIN entry point mapping for the Qualcomm
 *          QURT (QuRT) Real-Time Operating System running on Hexagon DSP processors.
 *          
 *          The QURT platform requires a specific entry point naming convention for
 *          applications running on the SLPI (Sensor Low Power Island) DSP subsystem.
 *          This header remaps the standard ArduPilot AP_MAIN macro to the QURT-specific
 *          qurt_ardupilot_main function name.
 *          
 *          Platform Context:
 *          - Target: Qualcomm Snapdragon platforms (VOXL, Snapdragon Flight)
 *          - Processor: Hexagon DSP (QDSP6)
 *          - OS: QURT Real-Time Operating System
 *          - Execution: SLPI (Sensor Low Power Island) for sensor processing
 *          
 *          Execution Flow:
 *          1. Hexagon DSP loads application
 *          2. QURT scheduler initializes DSP subsystem
 *          3. qurt_ardupilot_main() is called as entry point
 *          4. Vehicle-specific initialization proceeds from there
 *          
 *          This follows the QURT/Hexagon application lifecycle where the standard
 *          main() function is replaced by DSP-specific entry points that integrate
 *          with the QURT scheduler and power management system.
 * 
 * @note The actual implementation of qurt_ardupilot_main is in HAL_QURT_Class.cpp
 * 
 * @warning Modifying or removing the AP_MAIN macro will break vehicle startup on
 *          all Snapdragon-based platforms (VOXL, Snapdragon Flight). The QURT loader
 *          expects the specific qurt_ardupilot_main entry point name.
 * 
 * @see libraries/AP_HAL_QURT/HAL_QURT_Class.cpp for qurt_ardupilot_main implementation
 */

#pragma once

/**
 * @def AP_MAIN
 * @brief Remaps standard ArduPilot main entry point to QURT-specific function name
 * 
 * @details This macro remaps the platform-independent AP_MAIN identifier to
 *          qurt_ardupilot_main, which is the required entry point name for
 *          applications running on the Qualcomm Hexagon DSP under QURT.
 *          
 *          The QURT DSP execution model does not use a standard main() function.
 *          Instead, applications must provide a specifically-named entry point that
 *          the QURT scheduler can locate and invoke after DSP initialization.
 *          
 *          Usage in Vehicle Code:
 *          Vehicle-specific main files (e.g., ArduCopter/Copter.cpp) use AP_MAIN
 *          to define their entry point in a platform-independent way:
 *          
 *          @code
 *          AP_MAIN {
 *              // Vehicle initialization code
 *              hal.run(argc, argv, &copter);
 *          }
 *          @endcode
 *          
 *          On QURT platforms, this expands to:
 *          @code
 *          qurt_ardupilot_main {
 *              // Vehicle initialization code
 *              hal.run(argc, argv, &copter);
 *          }
 *          @endcode
 *          
 *          Execution Context:
 *          - Called by: QURT scheduler after SLPI DSP initialization
 *          - Thread: Main application thread on Hexagon DSP
 *          - Priority: Application-defined, managed by QURT scheduler
 *          - Memory: DSP local memory and shared memory regions
 *          
 *          Platform-Specific Requirements:
 *          - Function signature must match QURT expectations (int argc, char *argv[])
 *          - Must be visible to QURT dynamic loader (non-static linkage)
 *          - Must coordinate with QURT power management for SLPI low-power modes
 *          - Must use QURT synchronization primitives for inter-DSP communication
 * 
 * @warning Changing this macro name or removing it will prevent ArduPilot from
 *          starting on Snapdragon platforms. The QURT loader specifically searches
 *          for the qurt_ardupilot_main symbol during application loading.
 * 
 * @note This is part of the HAL (Hardware Abstraction Layer) that allows the same
 *       vehicle code to run on different hardware platforms with different operating
 *       systems and startup mechanisms.
 * 
 * @see AP_HAL_QURT_Namespace.h for QURT HAL implementation details
 * @see libraries/AP_HAL_QURT/HAL_QURT_Class.cpp for the actual qurt_ardupilot_main implementation
 */
#define AP_MAIN qurt_ardupilot_main

