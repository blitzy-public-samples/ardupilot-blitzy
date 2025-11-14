/**
 * @file version.h
 * @brief ArduSub firmware version identification
 * 
 * @details This header file defines the version information for the ArduSub underwater
 *          vehicle firmware. It provides version macros used throughout the codebase
 *          for version checking, telemetry reporting, and firmware identification.
 * 
 *          The version information follows semantic versioning (MAJOR.MINOR.PATCH)
 *          with an additional firmware type indicator (DEV, ALPHA, BETA, RC, OFFICIAL).
 * 
 * @note This file should NEVER be included directly by user code. Always include
 *       AP_Common/AP_FWVersion.h instead, which provides a unified interface for
 *       accessing firmware version information across all vehicle types.
 * 
 * @note Version values in this file are maintained by the development team and
 *       updated during the release process. The FIRMWARE_VERSION macro is
 *       specifically formatted for parsing by automated testing scripts.
 * 
 * @warning Modifying version information incorrectly can cause issues with:
 *          - Ground Control Station compatibility checks
 *          - Parameter migration between firmware versions
 *          - Log file analysis and replay tools
 *          - Automatic update mechanisms
 * 
 * @see AP_Common/AP_FWVersion.h for the public firmware version API
 * @see ap_version.h for build-specific version information (git hash, build date)
 * 
 * Source: ArduSub/version.h:1-21
 */

#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

/**
 * @brief Human-readable firmware identification string
 * 
 * @details This string is displayed in ground control stations, log files, and
 *          telemetry messages to identify the firmware name and version.
 *          Format: "ArduSub V<MAJOR>.<MINOR>.<PATCH>-<TYPE>"
 * 
 * @note This string is reported via MAVLink AUTOPILOT_VERSION message and
 *       appears in dataflash logs for firmware identification.
 */
#define THISFIRMWARE "ArduSub V4.7.0-dev"

/**
 * @brief Comma-separated firmware version tuple for automated parsing
 * 
 * @details This macro provides version information in a format suitable for
 *          automated testing scripts and version comparison macros. The format is:
 *          MAJOR, MINOR, PATCH, TYPE
 * 
 * @note This line is specifically parsed by autotest scripts in Tools/autotest/
 *       for version validation and compatibility testing. Do not modify the
 *       format or comment structure.
 * 
 * @see FIRMWARE_VERSION_TYPE_DEV, FIRMWARE_VERSION_TYPE_ALPHA, etc. in AP_Common
 */
// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,7,0,FIRMWARE_VERSION_TYPE_DEV

/**
 * @brief Major version number (semantic versioning)
 * 
 * @details Incremented for major releases with significant new features or
 *          breaking changes. Used for compatibility checking and parameter migration.
 */
#define FW_MAJOR 4

/**
 * @brief Minor version number (semantic versioning)
 * 
 * @details Incremented for minor releases with new features and enhancements
 *          that maintain backward compatibility. Reset to 0 when FW_MAJOR increments.
 */
#define FW_MINOR 7

/**
 * @brief Patch version number (semantic versioning)
 * 
 * @details Incremented for patch releases containing bug fixes and minor
 *          improvements. Reset to 0 when FW_MINOR increments.
 */
#define FW_PATCH 0

/**
 * @brief Firmware type/release status indicator
 * 
 * @details Indicates the release status of this firmware build:
 *          - FIRMWARE_VERSION_TYPE_DEV: Development/unstable build
 *          - FIRMWARE_VERSION_TYPE_ALPHA: Alpha testing build
 *          - FIRMWARE_VERSION_TYPE_BETA: Beta testing build
 *          - FIRMWARE_VERSION_TYPE_RC: Release candidate
 *          - FIRMWARE_VERSION_TYPE_OFFICIAL: Official stable release
 * 
 * @note Development builds may contain experimental features and have not
 *       undergone full testing. Use official releases for production vehicles.
 * 
 * @see AP_Common/AP_FWVersion.h for firmware type constant definitions
 */
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV

/**
 * @brief Include firmware version structure definition
 * 
 * @details This header uses the version macros defined above to construct the
 *          fw_version_var structure that is accessible throughout the codebase
 *          via AP_FWVersion.h. It must be included after all version macros are defined.
 */
#include <AP_Common/AP_FWVersionDefine.h>

/**
 * @brief Include firmware compatibility checking definitions
 * 
 * @details This header provides compile-time checks to ensure firmware compatibility
 *          and configuration validity. It verifies that required features are enabled
 *          and that the build configuration is consistent.
 */
#include <AP_CheckFirmware/AP_CheckFirmwareDefine.h>
