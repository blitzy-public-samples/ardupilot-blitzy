/**
 * @file version.h
 * @brief Firmware version macros for AntennaTracker
 * 
 * @details This file defines the firmware version identification strings and
 *          numeric version codes for the AntennaTracker vehicle type. These
 *          version identifiers are reported in MAVLink AUTOPILOT_VERSION messages,
 *          displayed in ground control stations, and logged for diagnostics.
 *          
 *          The version information follows semantic versioning: MAJOR.MINOR.PATCH
 *          with an additional type indicator (DEV, ALPHA, BETA, RC, OFFICIAL).
 * 
 * @note This file should never be included directly. Include AP_Common/AP_FWVersion.h instead.
 * @warning Version macros must match across all definition forms to ensure consistency
 *          in version reporting to ground control stations and logging systems.
 */

#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

/**
 * @brief Firmware identification string for AntennaTracker
 * 
 * @details Human-readable firmware name and version string displayed in ground
 *          control stations and used for firmware identification. Format follows
 *          the pattern: "AntennaTracker V<major>.<minor>.<patch>-<type>"
 *          where type is one of: dev, alpha, beta, rc, or omitted for official releases.
 * 
 * @note This string is reported in MAVLink AUTOPILOT_VERSION messages and displayed
 *       prominently in GCS software like Mission Planner, QGroundControl, and MAVProxy.
 * @note Version string is also written to dataflash logs for flight analysis and debugging.
 */
#define THISFIRMWARE "AntennaTracker V4.7.0-dev"

/**
 * @brief Numeric firmware version code for compatibility checking
 * 
 * @details Comma-separated version components used for programmatic version
 *          comparison and compatibility checking. Format is: major,minor,patch,type
 *          where type is defined in AP_Common/AP_FWVersion.h (e.g., FIRMWARE_VERSION_TYPE_DEV,
 *          FIRMWARE_VERSION_TYPE_OFFICIAL, FIRMWARE_VERSION_TYPE_BETA).
 * 
 * @note This line is parsed by autotest scripts to verify version consistency.
 * @note Used by ground control stations to determine feature compatibility and
 *       display appropriate UI elements for the firmware version.
 * @note The numeric version enables backward compatibility checks and ensures
 *       parameter compatibility between firmware versions.
 */
// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,7,0,FIRMWARE_VERSION_TYPE_DEV

/**
 * @brief Major version number component
 * @details Indicates significant releases with major features or breaking changes.
 *          Incremented for major architectural changes or significant feature additions.
 */
#define FW_MAJOR 4

/**
 * @brief Minor version number component
 * @details Indicates feature releases within a major version. Incremented for
 *          new features, improvements, or significant bug fixes that maintain
 *          backward compatibility.
 */
#define FW_MINOR 7

/**
 * @brief Patch version number component
 * @details Indicates bug fix releases within a minor version. Incremented for
 *          bug fixes, minor improvements, and maintenance updates.
 */
#define FW_PATCH 0

/**
 * @brief Firmware release type identifier
 * @details Indicates the stability and release status of the firmware.
 *          Values defined in AP_Common/AP_FWVersion.h include:
 *          - FIRMWARE_VERSION_TYPE_DEV: Development/unstable version
 *          - FIRMWARE_VERSION_TYPE_ALPHA: Early testing release
 *          - FIRMWARE_VERSION_TYPE_BETA: Feature-complete testing release
 *          - FIRMWARE_VERSION_TYPE_RC: Release candidate
 *          - FIRMWARE_VERSION_TYPE_OFFICIAL: Stable production release
 */
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV

#include <AP_Common/AP_FWVersionDefine.h>
#include <AP_CheckFirmware/AP_CheckFirmwareDefine.h>
