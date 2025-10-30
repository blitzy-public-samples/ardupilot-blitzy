/**
 * @file msp_version.h
 * @brief MSP protocol version constants for flight controller identification
 * 
 * @details This file defines version constants used by the MSP (MultiWii Serial Protocol)
 *          implementation to identify the flight controller software version to MSP clients
 *          such as ground control stations, OSD devices, and configuration tools.
 *          
 *          ArduPilot emulates Betaflight 4.2.0 version identification to maintain
 *          compatibility with MSP clients that expect Betaflight/iNav/Cleanflight
 *          version reporting format and behavior. This allows ArduPilot to work
 *          seamlessly with existing MSP-based tools and OSD hardware.
 * 
 * @note ArduPilot emulates Betaflight 4.2.0 for MSP protocol compatibility.
 *       This does NOT mean ArduPilot uses Betaflight code or algorithms - it only
 *       reports a Betaflight-compatible version to MSP clients for protocol
 *       compatibility and feature negotiation.
 * 
 * @see libraries/AP_MSP/AP_MSP.cpp for MSP protocol implementation
 */

#pragma once

#if HAL_MSP_ENABLED

/**
 * @brief Maximum length of git short revision string
 * 
 * Defines the buffer size for storing the abbreviated git commit hash
 * used in MSP_API_VERSION responses. Standard git short hash is 7 characters
 * plus null terminator.
 */
#define GIT_SHORT_REVISION_LENGTH   8  // 7 character git short hash + null terminator

/**
 * @brief Maximum length of build date string
 * 
 * Defines the buffer size for storing the build date string in format "MMM DD YYYY"
 * (e.g., "Jan 15 2024"). Total of 11 characters plus null terminator.
 */
#define BUILD_DATE_LENGTH           11  // "MMM DD YYYY" format + null terminator

/**
 * @brief Maximum length of build time string
 * 
 * Defines the buffer size for storing the build time string in format "HH:MM:SS"
 * (e.g., "14:30:45"). Total of 8 characters plus null terminator.
 */
#define BUILD_TIME_LENGTH           8  // "HH:MM:SS" format + null terminator

/**
 * @brief Flight controller major version number (Betaflight compatibility)
 * 
 * Major version component of the emulated Betaflight version (4.2.0).
 * Reported to MSP clients via MSP_API_VERSION and MSP_FC_VERSION messages.
 * This value determines high-level protocol compatibility and feature set
 * expectations for MSP clients.
 */
#define FC_VERSION_MAJOR            4

/**
 * @brief Flight controller minor version number (Betaflight compatibility)
 * 
 * Minor version component of the emulated Betaflight version (4.2.0).
 * Reported to MSP clients via MSP_API_VERSION and MSP_FC_VERSION messages.
 * This value indicates incremental feature additions within the major version.
 */
#define FC_VERSION_MINOR            2

/**
 * @brief Flight controller patch level (Betaflight compatibility)
 * 
 * Patch level component of the emulated Betaflight version (4.2.0).
 * Reported to MSP clients via MSP_API_VERSION and MSP_FC_VERSION messages.
 * This value typically indicates bug fixes within a minor version release.
 */
#define FC_VERSION_PATCH_LEVEL      0

#endif //HAL_MSP_ENABLED