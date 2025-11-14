/**
 * @file version.h
 * @brief Firmware version identification for ArduPilot Blimp vehicle variant
 * 
 * @details This file defines version constants for the Blimp (airship/lighter-than-air)
 *          vehicle firmware variant. The version information is used throughout the system
 *          for:
 *          - MAVLink AUTOPILOT_VERSION messages sent to ground control stations
 *          - Firmware identification in binary logs for post-flight analysis
 *          - Version checking and compatibility validation
 *          - Autotest script parsing and validation
 * 
 *          The version follows semantic versioning: MAJOR.MINOR.PATCH-TYPE
 *          where TYPE can be DEV (development), ALPHA, BETA, RC (release candidate),
 *          or OFFICIAL (stable release).
 * 
 * @warning This file should never be included directly. Include AP_Common/AP_FWVersion.h
 *          instead, which provides the proper abstraction layer for firmware version access.
 * 
 * @note The version constants defined here are automatically extracted by build scripts
 *       and autotest validation tools.
 * 
 * Source: Blimp/version.h
 */

#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

/**
 * @brief Human-readable firmware identification string for Blimp variant
 * 
 * @details This string identifier is used in:
 *          - MAVLink AUTOPILOT_VERSION.flight_sw_version field
 *          - Startup banner messages on console/telemetry
 *          - Log file headers for firmware identification
 *          - Ground control station display (Mission Planner, QGroundControl, etc.)
 * 
 * Format: "Blimp V{MAJOR}.{MINOR}.{PATCH}-{TYPE}"
 * Example: "Blimp V4.7.0-dev" indicates development version 4.7.0
 * 
 * @note This string is also used by users to identify which firmware variant
 *       and version is running on their vehicle.
 */
#define THISFIRMWARE "Blimp V4.7.0-dev"

/**
 * @brief Numeric firmware version tuple for programmatic version checks
 * 
 * @details Comma-separated version components used for numeric version comparison
 *          and compatibility checking. Format: MAJOR,MINOR,PATCH,TYPE
 * 
 *          Version type constants (defined in AP_Common/AP_FWVersion.h):
 *          - FIRMWARE_VERSION_TYPE_DEV (255): Development/unstable builds
 *          - FIRMWARE_VERSION_TYPE_ALPHA (192): Alpha testing builds
 *          - FIRMWARE_VERSION_TYPE_BETA (128): Beta testing builds
 *          - FIRMWARE_VERSION_TYPE_RC (64): Release candidate builds
 *          - FIRMWARE_VERSION_TYPE_OFFICIAL (0): Stable release builds
 * 
 * @warning This line is parsed by autotest scripts - maintain exact format.
 *          Any changes to format may break automated testing infrastructure.
 * 
 * @note Used for:
 *       - Minimum version requirements checking
 *       - Feature compatibility detection
 *       - Automatic update notifications
 *       - Log analysis tools for version-specific behavior
 */
// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,7,0,FIRMWARE_VERSION_TYPE_DEV

/**
 * @brief Major version number component
 * 
 * @details Incremented for incompatible API/behavior changes or major feature releases.
 *          Breaking changes to parameter names, MAVLink messages, or flight behavior
 *          typically warrant a major version increment.
 */
#define FW_MAJOR 4

/**
 * @brief Minor version number component
 * 
 * @details Incremented for backward-compatible new features and enhancements.
 *          New flight modes, sensor drivers, or capabilities are typically
 *          added in minor version releases.
 */
#define FW_MINOR 7

/**
 * @brief Patch version number component
 * 
 * @details Incremented for backward-compatible bug fixes and minor improvements.
 *          Security patches, stability fixes, and minor corrections increment
 *          the patch version.
 */
#define FW_PATCH 0

/**
 * @brief Firmware build type identifier
 * 
 * @details Indicates the stability/release status of this build:
 *          - FIRMWARE_VERSION_TYPE_DEV: Unstable development build from master branch
 *          - FIRMWARE_VERSION_TYPE_ALPHA: Early testing build, may have known issues
 *          - FIRMWARE_VERSION_TYPE_BETA: Feature-complete testing build
 *          - FIRMWARE_VERSION_TYPE_RC: Release candidate, pending final validation
 *          - FIRMWARE_VERSION_TYPE_OFFICIAL: Stable release for production use
 * 
 * @warning Development builds (TYPE_DEV) may contain experimental features and
 *          should only be used by developers or for testing purposes. Always use
 *          OFFICIAL releases for operational vehicles.
 */
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV

#include <AP_Common/AP_FWVersionDefine.h>
#include <AP_CheckFirmware/AP_CheckFirmwareDefine.h>
