/**
 * @file version.h
 * @brief ArduCopter firmware version identification and build tracking
 * 
 * @details This file defines the version information for ArduCopter firmware builds,
 *          including major, minor, and patch version numbers, as well as the firmware
 *          development type (dev, beta, official release, etc.).
 * 
 *          The version system integrates with the ArduPilot firmware versioning framework
 *          (AP_Common/AP_FWVersion.h) to provide consistent version identification across
 *          all vehicle types. Version information is used for:
 *          - Build identification and release tracking
 *          - Compatibility checking with ground control stations
 *          - Automated testing and continuous integration
 *          - User-visible firmware version display
 *          - Parameter and log file version tagging
 * 
 *          IMPORTANT: This file should NEVER be included directly in application code.
 *          Instead, include AP_Common/AP_FWVersion.h which provides the proper version
 *          access interface. Direct inclusion is prevented by the FORCE_VERSION_H_INCLUDE
 *          guard to maintain version system integrity.
 * 
 * @note Version numbers follow semantic versioning: MAJOR.MINOR.PATCH
 * @note The FIRMWARE_VERSION macro format is parsed by autotest scripts - do not modify format
 * @warning Changing version numbers affects compatibility checks and should only be done
 *          as part of the official release process
 * 
 * @see AP_Common/AP_FWVersion.h for the public firmware version interface
 * @see ap_version.h for vehicle-specific version metadata
 * 
 * Source: ArduCopter/version.h
 */

#pragma once

/**
 * @brief Include guard preventing direct inclusion of version.h
 * 
 * @details This preprocessor check ensures that version.h is only included through
 *          the proper version system headers (AP_Common/AP_FWVersion.h). Direct
 *          inclusion is prohibited because:
 *          - Version information should be accessed through the unified FWVersion API
 *          - Direct inclusion bypasses version system initialization
 *          - It prevents proper version metadata propagation
 * 
 *          The FORCE_VERSION_H_INCLUDE macro is defined only by AP_FWVersionDefine.h
 *          during proper version system initialization.
 * 
 * @note If you see this error, include AP_Common/AP_FWVersion.h instead
 */
#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

/**
 * @brief Include vehicle-specific version metadata
 * 
 * @details Includes ap_version.h which is generated during the build process and
 *          contains build-specific metadata such as:
 *          - Git commit hash
 *          - Build date and time
 *          - Board target information
 *          - Build environment details
 */
#include "ap_version.h"

/**
 * @brief Human-readable firmware version string
 * 
 * @details This macro defines the complete firmware identification string displayed
 *          to users in ground control stations, boot messages, and version queries.
 *          Format: "ArduCopter V<MAJOR>.<MINOR>.<PATCH>-<TYPE>"
 * 
 *          Version type suffixes:
 *          - "-dev" : Development/unstable build from master branch
 *          - "-beta" : Beta testing release candidate
 *          - "" (none) : Official stable release
 *          - "-rc" : Release candidate
 * 
 * @note This string is sent to ground stations via MAVLink AUTOPILOT_VERSION message
 * @note Displayed during boot sequence and in version status messages
 */
#define THISFIRMWARE "ArduCopter V4.7.0-dev"

/**
 * @brief Machine-readable firmware version tuple for automated processing
 * 
 * @details This macro defines the firmware version as a comma-separated tuple used by:
 *          - Autotest scripts for version verification and compatibility testing
 *          - Build system for version-based conditional compilation
 *          - Ground control stations for compatibility checks
 *          - Automated CI/CD pipelines for release validation
 * 
 *          Format: MAJOR, MINOR, PATCH, TYPE
 *          Where TYPE is one of:
 *          - FIRMWARE_VERSION_TYPE_DEV : Development build
 *          - FIRMWARE_VERSION_TYPE_ALPHA : Alpha testing build
 *          - FIRMWARE_VERSION_TYPE_BETA : Beta testing build
 *          - FIRMWARE_VERSION_TYPE_RC : Release candidate
 *          - FIRMWARE_VERSION_TYPE_OFFICIAL : Official stable release
 * 
 * @warning The format of this line is parsed by autotest scripts - do NOT modify
 *          the syntax or spacing without updating the corresponding parser code
 * @warning This macro is automatically parsed - keep it on a single line
 * 
 * @note Types defined in AP_Common/AP_FWVersion.h
 * @see Tools/autotest/ for scripts that parse this version information
 */
// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,7,0,FIRMWARE_VERSION_TYPE_DEV

/**
 * @brief Major version number (semantic versioning)
 * 
 * @details The major version number is incremented for:
 *          - Incompatible API changes
 *          - Major architectural changes
 *          - Breaking changes to parameter formats or log structures
 *          - Significant flight behavior changes requiring retuning
 * 
 * @note Major version changes may require ground station updates
 * @note Users should review release notes carefully for major version updates
 */
#define FW_MAJOR 4

/**
 * @brief Minor version number (semantic versioning)
 * 
 * @details The minor version number is incremented for:
 *          - New features and capabilities added in a backward-compatible manner
 *          - New flight modes or significant enhancements
 *          - New sensor or hardware support
 *          - Performance improvements
 * 
 * @note Minor version updates are typically backward compatible
 * @note Parameter and log formats remain compatible within same major version
 */
#define FW_MINOR 7

/**
 * @brief Patch version number (semantic versioning)
 * 
 * @details The patch version number is incremented for:
 *          - Bug fixes and stability improvements
 *          - Security patches
 *          - Minor performance optimizations
 *          - Documentation updates
 * 
 * @note Patch releases should be fully compatible and safe to install
 * @note Patch updates typically do not require parameter changes or retuning
 */
#define FW_PATCH 0

/**
 * @brief Firmware release type identifier
 * 
 * @details Indicates the development stage and stability level of this firmware build:
 *          - FIRMWARE_VERSION_TYPE_DEV : Unstable development build from master branch,
 *            features under active development, not recommended for production use
 *          - FIRMWARE_VERSION_TYPE_ALPHA : Early testing build, known issues expected
 *          - FIRMWARE_VERSION_TYPE_BETA : Feature-complete testing build, community testing
 *          - FIRMWARE_VERSION_TYPE_RC : Release candidate, final testing before release
 *          - FIRMWARE_VERSION_TYPE_OFFICIAL : Stable release, recommended for production
 * 
 * @warning Development and alpha builds may have incomplete features or known bugs
 * @warning Only OFFICIAL releases are recommended for operational vehicles
 * 
 * @note Ground stations may display warnings for non-official firmware types
 * @note Type affects parameter defaults and safety check strictness in some cases
 */
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV

/**
 * @brief Include firmware version system definition and initialization
 * 
 * @details This header processes the version macros defined above and integrates them
 *          into the ArduPilot firmware version framework. It:
 *          - Constructs the complete FWVersion structure
 *          - Defines the FORCE_VERSION_H_INCLUDE guard (allowing this inclusion)
 *          - Sets up version accessors for runtime queries
 *          - Integrates git commit hash and build metadata
 * 
 * @note This must be included after all version macros are defined
 * @see AP_Common/AP_FWVersion.h for the public version query interface
 */
#include <AP_Common/AP_FWVersionDefine.h>

/**
 * @brief Include firmware compatibility checking system
 * 
 * @details This header provides compile-time and runtime firmware compatibility
 *          verification mechanisms. It:
 *          - Validates firmware configuration against hardware capabilities
 *          - Checks for required feature availability
 *          - Verifies board-specific firmware compatibility
 *          - Enables build-time safety checks for unsupported configurations
 * 
 * @note Compatibility checks help prevent loading incompatible firmware on hardware
 * @see AP_CheckFirmware/ library for firmware validation implementation
 */
#include <AP_CheckFirmware/AP_CheckFirmwareDefine.h>
