/**
 * @file AP_HAL_SITL_Private.h
 * @brief Private/internal interfaces for the SITL (Software In The Loop) Hardware Abstraction Layer
 * 
 * @details This header provides internal implementation interfaces for the SITL HAL,
 *          which enables ArduPilot to run in simulation mode on desktop operating systems
 *          without physical hardware. It includes internal classes and helper functions
 *          used exclusively within the SITL HAL implementation.
 *          
 *          **Purpose of Private vs Public Headers:**
 *          - **Public Header (AP_HAL_SITL.h)**: Defines the public API for accessing
 *            the SITL HAL instance and initialization. Used by code outside the HAL.
 *          - **Private Header (this file)**: Declares internal implementation details
 *            including scheduler, storage, UART drivers, semaphores, CAN interfaces,
 *            and DSP implementations specific to SITL simulation.
 *          
 *          **When to Include This Header:**
 *          This header should ONLY be included by source files within the
 *          libraries/AP_HAL_SITL/ directory that implement the SITL HAL internals.
 *          
 *          **Simulation-Specific Internal APIs:**
 *          - Scheduler: Task scheduling and timing simulation
 *          - Storage: Simulated EEPROM/flash storage backed by filesystem
 *          - UARTDriver: Serial port simulation via TCP/UDP sockets or files
 *          - SITL_State: Core simulation state management and physics integration
 *          - Semaphores: Thread synchronization primitives for simulation
 *          - CANSocketIface: CAN bus simulation via SocketCAN
 *          - DSP: Digital Signal Processor simulation for FFT operations
 * 
 * @warning DO NOT include this header from code outside libraries/AP_HAL_SITL/.
 *          External code should only use the public AP_HAL interface defined in
 *          libraries/AP_HAL/ and the SITL HAL accessor via hal.h. Including this
 *          private header from vehicle code or libraries creates tight coupling
 *          to SITL-specific implementation details and breaks HAL abstraction.
 * 
 * @note The SITL HAL allows developers to test ArduPilot on desktop computers
 *       (Linux, macOS, Windows via WSL/Cygwin) with simulated sensors, actuators,
 *       and physics, dramatically accelerating development and testing cycles.
 * 
 * @see libraries/AP_HAL_SITL/AP_HAL_SITL.h for public SITL HAL interface
 * @see libraries/AP_HAL/ for generic HAL interface definitions
 * @see libraries/SITL/ for physics simulation and sensor models
 * @see Tools/autotest/ for SITL testing framework
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_HAL_SITL_Namespace.h"
#include "Scheduler.h"
#include "Storage.h"
#include "UARTDriver.h"
#include "SITL_State.h"
#include "Semaphores.h"
#include "CANSocketIface.h"
#include "DSP.h"
