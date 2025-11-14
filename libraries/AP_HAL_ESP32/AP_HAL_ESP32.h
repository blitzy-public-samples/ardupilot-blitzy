/**
 * @file AP_HAL_ESP32.h
 * @brief Main entry point header for the ESP32 Hardware Abstraction Layer (HAL) implementation
 * 
 * @details This file serves as the primary include header for users of the ESP32 HAL
 *          implementation. It acts as a barrel header that re-exports the base AP_HAL
 *          interfaces and the concrete HAL_ESP32_Class implementation for the ESP32
 *          platform.
 *          
 *          The ESP32 HAL provides hardware abstraction for ESP32-based autopilot boards,
 *          enabling ArduPilot to run on Espressif's ESP32 microcontroller platform with
 *          FreeRTOS integration.
 *          
 *          This header provides the ESP32 HAL entry point without exposing implementation
 *          details. Client code should include this file to access ESP32 HAL functionality.
 * 
 * @note The actual HAL implementation details are contained in HAL_ESP32_Class.h/cpp.
 *       This separation keeps the public API clean while allowing the implementation
 *       to use ESP32-specific features and ESP-IDF APIs.
 * 
 * @warning ESP32-specific requirements:
 *          - Requires ESP-IDF (Espressif IoT Development Framework)
 *          - Depends on FreeRTOS integration provided by ESP-IDF
 *          - Requires ESP32 or ESP32-S3 hardware platform
 *          - Build configuration must enable ESP32 HAL support
 * 
 * @see HAL_ESP32_Class for the concrete HAL implementation
 * @see AP_HAL::HAL for the base hardware abstraction layer interface
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

/* Your layer exports should depend on AP_HAL.h ONLY. */
#include <AP_HAL/AP_HAL.h>


#include "HAL_ESP32_Class.h"
