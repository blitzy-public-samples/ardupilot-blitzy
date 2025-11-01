/**
 * @file version.h
 * @brief ArduPlane firmware version identification and numbering
 * 
 * @details This file defines the ArduPlane firmware version macros used throughout
 *          the codebase for version identification, compatibility checks, and
 *          reporting to ground control stations via MAVLink.
 *          
 *          The version numbering follows semantic versioning principles:
 *          - MAJOR version: Significant architectural changes or API breaks
 *          - MINOR version: New features, backward compatible
 *          - PATCH version: Bug fixes, backward compatible
 *          - TYPE: Development status (DEV, ALPHA, BETA, RC, OFFICIAL)
 *          
 *          Version information is used for:
 *          - Ground station display and compatibility checking
 *          - Log file identification for debugging
 *          - Feature availability detection
 *          - Firmware update verification
 *          
 * @note This file should NEVER be included directly. Always include
 *       AP_Common/AP_FWVersion.h instead, which provides the proper
 *       framework for version management across all vehicle types.
 * 
 * @warning Modifying version numbers incorrectly can cause compatibility
 *          issues with ground control stations and parameter files.
 * 
 * @see AP_Common/AP_FWVersion.h
 * @see AP_Common/AP_FWVersionDefine.h
 * 
 * Source: ArduPlane/version.h
 */

#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

/**
 * @brief ArduPlane firmware version string for display and identification
 * 
 * @details Human-readable firmware version string displayed in ground control
 *          stations, boot messages, and log files. Format follows the pattern:
 *          "ArduPlane V<MAJOR>.<MINOR>.<PATCH>-<TYPE>"
 *          
 *          This string is transmitted via MAVLink AUTOPILOT_VERSION message
 *          and appears in DataFlash logs for flight reconstruction and debugging.
 *          
 *          Version type suffixes:
 *          - "dev": Development/bleeding-edge builds from master branch
 *          - "alpha": Early testing releases, expect bugs
 *          - "beta": Feature-complete, undergoing testing
 *          - "rc": Release candidate, final testing before stable
 *          - (no suffix): Official stable release
 * 
 * @note Autotest scripts parse log files looking for this string to identify
 *       the firmware version under test.
 */
#define THISFIRMWARE "ArduPlane V4.7.0-dev"

/**
 * @brief Comma-separated firmware version components for programmatic access
 * 
 * @details Machine-readable version tuple used by the firmware version framework
 *          to construct version comparison and compatibility checking logic.
 *          Format: MAJOR,MINOR,PATCH,TYPE
 *          
 *          This macro is parsed by:
 *          - Autotest scripts to verify firmware version in SITL testing
 *          - AP_FWVersion system to populate version structures
 *          - Ground control station compatibility checks
 *          
 *          The comma-separated format allows expansion into function arguments
 *          or array initializers throughout the version management system.
 * 
 * @note Autotest scripts parse this line directly - maintain exact format
 * @see AP_Common/AP_FWVersion.h for version comparison functions
 */
// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,7,0,FIRMWARE_VERSION_TYPE_DEV

/**
 * @brief Major version number component
 * 
 * @details Incremented for significant architectural changes, major new features,
 *          or breaking changes to APIs, parameter formats, or log structures.
 *          Major version changes typically require user intervention for migration.
 *          
 *          Major version 4 represents the current ArduPlane architecture with:
 *          - Quadplane support
 *          - EKF3 state estimation
 *          - Lua scripting capability
 *          - Modern HAL abstraction
 */
#define FW_MAJOR 4

/**
 * @brief Minor version number component
 * 
 * @details Incremented for new features and enhancements that maintain backward
 *          compatibility with the same major version. Minor releases add:
 *          - New flight modes
 *          - Additional sensor drivers
 *          - Enhanced tuning capabilities
 *          - New peripheral support
 *          
 *          Parameter files and mission plans remain compatible within the same
 *          major version across minor version changes.
 */
#define FW_MINOR 7

/**
 * @brief Patch version number component
 * 
 * @details Incremented for bug fixes, performance improvements, and minor
 *          enhancements that don't add new features. Patch releases address:
 *          - Critical bug fixes
 *          - Safety improvements
 *          - Performance optimizations
 *          - Documentation corrections
 *          
 *          Patch version 0 indicates a development or pre-release version where
 *          the patch number hasn't been finalized for release.
 */
#define FW_PATCH 0

/**
 * @brief Firmware release type/status indicator
 * 
 * @details Identifies the release maturity and stability level. Valid types:
 *          - FIRMWARE_VERSION_TYPE_DEV: Development/unstable, master branch
 *          - FIRMWARE_VERSION_TYPE_ALPHA: Early testing, expect significant bugs
 *          - FIRMWARE_VERSION_TYPE_BETA: Feature complete, testing in progress
 *          - FIRMWARE_VERSION_TYPE_RC: Release candidate, final validation
 *          - FIRMWARE_VERSION_TYPE_OFFICIAL: Stable production release
 *          
 *          The type affects:
 *          - Ground station warnings about stability
 *          - Feature availability (some features restricted to official builds)
 *          - Telemetry logging verbosity
 *          - Support expectations from development team
 * 
 * @warning Development builds may contain experimental features that could
 *          affect flight safety. Always test thoroughly before flying.
 * 
 * @see ap_version.h for FIRMWARE_VERSION_TYPE_* constant definitions
 */
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV

/**
 * @note AP_FWVersionDefine.h uses the macros defined above to construct the
 *       complete firmware version framework, including version comparison
 *       functions and MAVLink version reporting structures.
 */
#include <AP_Common/AP_FWVersionDefine.h>

/**
 * @note AP_CheckFirmwareDefine.h provides compile-time firmware compatibility
 *       validation to ensure the build configuration matches the vehicle type.
 */
#include <AP_CheckFirmware/AP_CheckFirmwareDefine.h>
