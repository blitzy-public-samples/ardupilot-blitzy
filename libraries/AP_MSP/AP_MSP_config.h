/**
 * @file AP_MSP_config.h
 * @brief Compile-time configuration and feature flags for the AP_MSP library
 * 
 * @details This header provides configuration macros that control MSP (MultiWii Serial Protocol)
 *          functionality in ArduPilot. MSP is a lightweight binary protocol used for:
 *          - Telemetry data transmission to ground stations and OSD devices
 *          - External sensor integration (GPS, rangefinders, optical flow, etc.)
 *          - Character-based OSD rendering via MSP DisplayPort protocol
 *          - Compatibility with Betaflight, iNav, and DJI FPV ecosystem devices
 * 
 * @note These flags are typically overridden in board-specific hwdef files for boards
 *       with limited flash memory or RAM to disable unused MSP features.
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

/**
 * @brief Master enable flag for MSP (MultiWii Serial Protocol) support
 * 
 * @details Controls whether MSP protocol functionality is compiled into the firmware.
 *          When enabled, ArduPilot can communicate using the MSP binary protocol for:
 *          - Telemetry streaming to MSP-compatible ground stations
 *          - Status and configuration data exchange
 *          - Compatibility with Betaflight Configurator and similar tools
 *          - Foundation for MSP sensor drivers and DisplayPort features
 * 
 * @note Defaults to 1 (enabled) on all platforms unless overridden in board hwdef
 * @note This is a prerequisite for HAL_MSP_SENSORS_ENABLED and HAL_WITH_MSP_DISPLAYPORT
 * 
 * @warning Disabling this removes all MSP functionality including:
 *          - MSP telemetry output on serial ports
 *          - MSP sensor input support (GPS, rangefinder, optical flow, etc.)
 *          - MSP OSD/DisplayPort support for character-based displays
 *          - Compatibility with MSP-based ground control stations
 */
#ifndef HAL_MSP_ENABLED
#define HAL_MSP_ENABLED 1
#endif

/**
 * @brief Enable MSP v2 sensor message drivers for external sensor integration
 * 
 * @details When enabled, allows external sensors to send measurement data to ArduPilot
 *          via MSP v2 protocol messages. Supported sensor types include:
 *          - Rangefinder/distance sensors (MSP_RANGEFINDER message)
 *          - Optical flow sensors (MSP_OPTICAL_FLOW message)
 *          - GPS receivers (MSP_GPS message)
 *          - Barometric pressure sensors (MSP_BARO message)
 *          - Magnetometers/compass (MSP_COMPASS message)
 *          - Airspeed sensors (MSP_AIRSPEED message)
 * 
 *          This feature enables integration of MSP-compatible external sensors from
 *          companion computers, sensor hubs, or other flight controllers acting as
 *          sensor sources.
 * 
 * @note Defaults to HAL_MSP_ENABLED value (enabled if MSP protocol is enabled)
 * @note Requires HAL_MSP_ENABLED=1 to function
 * @note Can be disabled independently to save flash space if only telemetry output is needed
 * 
 * @see libraries/AP_MSP/msp_sensors.cpp for sensor driver implementations
 */
#ifndef HAL_MSP_SENSORS_ENABLED
#define HAL_MSP_SENSORS_ENABLED HAL_MSP_ENABLED
#endif

/**
 * @brief Enable MSP DisplayPort protocol for character-based OSD rendering
 * 
 * @details DisplayPort is an MSP sub-protocol that allows ArduPilot to drive MSP-compatible
 *          On-Screen Display (OSD) devices using character-based rendering. Supported devices:
 *          - Betaflight OSD hardware (e.g., FCHUB-VTX, OMNIBUS F4)
 *          - DJI FPV goggles in MSP DisplayPort mode
 *          - MWOSD and other MSP OSD implementations
 *          - Generic MSP-compatible character overlay devices
 * 
 *          DisplayPort provides a grid-based character display interface where ArduPilot
 *          renders flight data (altitude, speed, battery, GPS, warnings) as text overlays
 *          on the video feed.
 * 
 * @note Defaults to HAL_MSP_ENABLED value (enabled if MSP protocol is enabled)
 * @note Requires HAL_MSP_ENABLED=1 to function
 * @note Can be disabled independently to save flash space on boards without OSD hardware
 * @note DisplayPort uses character grid rendering (typically 30x16 or similar resolution)
 * 
 * @see libraries/AP_MSP/msp_displayport.cpp for DisplayPort implementation
 * @see libraries/AP_OSD for the main OSD rendering system that can use MSP DisplayPort as a backend
 */
#ifndef HAL_WITH_MSP_DISPLAYPORT
#define HAL_WITH_MSP_DISPLAYPORT HAL_MSP_ENABLED
#endif
