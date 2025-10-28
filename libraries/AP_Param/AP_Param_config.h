/**
 * @file AP_Param_config.h
 * @brief Configuration header for AP_Param feature toggles and platform settings
 * 
 * @details Defines compile-time configuration for the parameter system including:
 * - AP_PARAM_DEFAULTS_FILE_PARSING_ENABLED: Controls external defaults file parsing
 * - FORCE_APJ_DEFAULT_PARAMETERS: Forces embedded defaults at build time
 * 
 * This configuration header allows platform-specific and build-specific customization
 * of the parameter system's behavior without modifying core AP_Param implementation.
 * Settings here affect parameter loading, storage, and defaults management across
 * the entire ArduPilot system.
 * 
 * @note Includes AP_Filesystem_config.h to inherit filesystem capability settings
 * @note Modifying these settings affects parameter loading behavior system-wide
 * 
 * @see AP_Param.h for the main parameter system implementation
 * @see AP_Param::load_defaults_file() for defaults file parsing implementation
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#include <AP_Filesystem/AP_Filesystem_config.h>

/**
 * @def AP_PARAM_DEFAULTS_FILE_PARSING_ENABLED
 * @brief Enable parsing of external parameter defaults files from filesystem
 * 
 * @details When enabled, AP_Param can load default parameter values from text files
 *          in the filesystem at boot time. This allows board-specific or application-specific
 *          defaults without recompiling firmware. The defaults file uses a simple text format
 *          with parameter names and values.
 *          
 *          Defaults to AP_FILESYSTEM_FILE_READING_ENABLED (inherits filesystem read capability).
 *          Set to 0 to disable defaults file parsing and save code space on resource-constrained
 *          platforms.
 *          
 *          Typical use cases:
 *          - Loading board-specific parameter defaults at boot
 *          - OEM customization with external configuration files
 *          - Development/testing with rapidly changing defaults
 *          - Multi-vehicle configurations from single firmware build
 * 
 * @note Requires functional filesystem (SD card or internal storage)
 * @note Defaults file typically located at @/APM/defaults.parm on SD card
 * @note See AP_Param::load_defaults_file() for file format specification
 * @note Disabling this feature reduces flash usage by ~2KB
 * 
 * @see AP_Param::load_defaults_file() for implementation details
 * @see AP_FILESYSTEM_FILE_READING_ENABLED for filesystem capability configuration
 */
#ifndef AP_PARAM_DEFAULTS_FILE_PARSING_ENABLED
#define AP_PARAM_DEFAULTS_FILE_PARSING_ENABLED AP_FILESYSTEM_FILE_READING_ENABLED
#endif

/**
 * @def FORCE_APJ_DEFAULT_PARAMETERS
 * @brief Force inclusion of embedded default parameters in APJ firmware file
 * 
 * @details When set to 1, embeds parameter defaults directly in firmware binary
 *          for modification by apj_tool.py without recompiling. This creates a reserved
 *          space in the firmware image that can be overwritten post-compilation to
 *          inject custom default parameters. Increases firmware size by 
 *          AP_PARAM_MAX_EMBEDDED_PARAM bytes.
 *          
 *          Defaults to 0 (no embedded defaults, saves flash space).
 *          
 *          Use cases:
 *          - OEM firmware customization without source code access
 *          - Factory default parameter sets per vehicle variant
 *          - Rapid default changes during development/testing
 *          - Distribution of pre-configured firmware binaries
 *          - Support organizations providing custom firmware builds
 *          
 *          Workflow:
 *          1. Build firmware with FORCE_APJ_DEFAULT_PARAMETERS=1
 *          2. Use apj_tool.py --set-defaults to inject parameters into .apj file
 *          3. Flash modified .apj to vehicle
 *          4. Parameters apply as defaults on first boot or after parameter reset
 * 
 * @note Embedded defaults space defined by AP_PARAM_MAX_EMBEDDED_PARAM in AP_Param.h
 * @note Modified via Tools/scripts/apj_tool.py --set-defaults after firmware compilation
 * @note Parameters injected post-build do not require firmware recompilation
 * @note Embedded space is allocated even if not used - only enable if needed
 * 
 * @warning Increases firmware size - only use if default modification capability needed
 * @warning Embedded defaults consume flash space that cannot be reclaimed at runtime
 * @warning Ensure parameter names in defaults file match firmware version
 * 
 * @see AP_Param.h for AP_PARAM_MAX_EMBEDDED_PARAM definition
 * @see Tools/scripts/apj_tool.py for parameter injection tool
 */
#ifndef FORCE_APJ_DEFAULT_PARAMETERS
#define FORCE_APJ_DEFAULT_PARAMETERS 0
#endif
