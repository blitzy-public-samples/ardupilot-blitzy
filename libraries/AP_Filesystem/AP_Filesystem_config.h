/**
 * @file AP_Filesystem_config.h
 * @brief Central compile-time configuration for AP_Filesystem backends and capabilities
 * 
 * @details This header provides HAL-aware feature flags that control which filesystem
 *          backends are compiled into the ArduPilot firmware. Backend selection is
 *          typically automatic based on the target board's HAL configuration, but can
 *          be customized for specific hardware configurations.
 * 
 *          Configuration Approach:
 *          - Each backend has an AP_FILESYSTEM_*_ENABLED flag
 *          - Flags default based on HAL board type and capabilities
 *          - Override flags in hwdef files or board-specific headers for custom builds
 *          - Aggregate flags (FILE_WRITING_ENABLED, FILE_READING_ENABLED) combine
 *            backend capabilities for compile-time feature detection
 * 
 *          Filesystem Backend Types:
 *          - Physical Storage: ESP32, FATFS (SD card), LittleFS (SPI flash), POSIX
 *          - Virtual Filesystems: @PARAM (parameter access), @SYS (diagnostics), @MISSION
 *          - Read-Only: ROMFS (embedded files)
 * 
 *          Customization Example for Custom Boards:
 *          @code
 *          // In custom_board_config.h, disable LittleFS and enable only FATFS:
 *          #define AP_FILESYSTEM_LITTLEFS_ENABLED 0
 *          #define AP_FILESYSTEM_FATFS_ENABLED 1
 *          @endcode
 * 
 * @note Backend availability affects binary size and RAM usage - disable unused
 *       backends for resource-constrained boards
 * 
 * @see AP_Filesystem.cpp for backend registration and initialization
 * @see libraries/AP_Filesystem/README.md for filesystem architecture documentation
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

/**
 * @defgroup filesystem_flash_types LittleFS Flash Type Constants
 * @brief Flash hardware type identifiers for LittleFS backend configuration
 * @{
 */

/**
 * @def AP_FILESYSTEM_FLASH_JEDEC_NOR
 * @brief JEDEC-compliant NOR flash type identifier
 * 
 * @details Standard SPI NOR flash chips following JEDEC specifications.
 *          Most common flash type for embedded systems.
 *          Characteristics: Random access, XIP-capable, erase-before-write.
 * 
 * @note This is the default flash type for LittleFS if not overridden
 */
#define AP_FILESYSTEM_FLASH_JEDEC_NOR 1

/**
 * @def AP_FILESYSTEM_FLASH_W25NXX
 * @brief Winbond W25N series NAND flash type identifier
 * 
 * @details Winbond W25N NAND flash chips (e.g., W25N01GV, W25N02JW).
 *          Characteristics: Block-based access, higher density, requires ECC.
 * 
 * @note Only use if your board has W25N-series NAND flash hardware
 * @warning Flash type must exactly match hardware - incorrect type causes filesystem corruption
 */
#define AP_FILESYSTEM_FLASH_W25NXX 2

/** @} */ // end of filesystem_flash_types

/**
 * @defgroup filesystem_backends Filesystem Backend Enable Flags
 * @brief Compile-time flags controlling which filesystem backends are included
 * 
 * @details Each backend implements a specific storage interface or virtual filesystem.
 *          Backends are automatically enabled based on HAL board capabilities, but can
 *          be overridden in board configuration files for custom builds.
 * 
 *          Backend Dependencies:
 *          - ESP32: Requires ESP-IDF and CONFIG_HAL_BOARD == HAL_BOARD_ESP32
 *          - FATFS: Requires HAL_OS_FATFS_IO (SD card hardware and ChibiOS)
 *          - LittleFS: Requires HAL_OS_LITTLEFS_IO (SPI flash hardware)
 *          - POSIX: Requires POSIX-compliant OS (SITL, Linux, QURT)
 *          - ROMFS: Requires embedded ROMFS data (HAL_HAVE_AP_ROMFS_EMBEDDED_H)
 *          - @PARAM, @SYS: Always enabled (core virtual filesystems)
 *          - @MISSION: Requires AP_MISSION_ENABLED
 * 
 * @{
 */

/**
 * @def AP_FILESYSTEM_ESP32_ENABLED
 * @brief Enable ESP32 filesystem backend (ESP-IDF FatFs wrapper)
 * 
 * @details Provides filesystem access on ESP32 boards using ESP-IDF's VFS layer.
 *          Wraps ESP-IDF's FatFs implementation for SD card and SPIFFS support.
 *          Automatically enabled when building for ESP32 hardware.
 * 
 * @note Only available on CONFIG_HAL_BOARD == HAL_BOARD_ESP32
 * @see libraries/AP_HAL_ESP32/ for ESP32 HAL implementation
 */
#ifndef AP_FILESYSTEM_ESP32_ENABLED
#define AP_FILESYSTEM_ESP32_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_ESP32)
#endif

/**
 * @def AP_FILESYSTEM_FATFS_ENABLED
 * @brief Enable FATFS filesystem backend (SD card support)
 * 
 * @details Provides FAT filesystem support for SD cards and eMMC storage.
 *          Primary storage backend for ChibiOS-based flight controllers with SD cards.
 *          Implements standard FAT12/FAT16/FAT32 filesystem with long filename support.
 * 
 *          Storage Backend: SD card via SDIO or SPI interface
 *          Use Cases: DataFlash logging, parameter storage, terrain data, mission files
 * 
 * @note Automatically enabled when HAL_OS_FATFS_IO is defined (ChibiOS boards with SD)
 * @note Binary size impact: ~40KB code, varies by features enabled
 * @warning Disabling removes logging capability on SD-card-based boards
 * 
 * @see libraries/AP_Logger/ for DataFlash logging implementation
 */
#ifndef AP_FILESYSTEM_FATFS_ENABLED
#define AP_FILESYSTEM_FATFS_ENABLED HAL_OS_FATFS_IO
#endif

/**
 * @def AP_FILESYSTEM_LITTLEFS_ENABLED
 * @brief Enable LittleFS filesystem backend (internal SPI flash)
 * 
 * @details Provides power-loss-resilient filesystem for internal SPI flash memory.
 *          Designed for embedded systems with NOR/NAND flash, features wear leveling
 *          and dynamic bad block management. Alternative to FATFS for boards without SD.
 * 
 *          Storage Backend: Internal SPI flash (NOR or NAND)
 *          Characteristics: Power-loss resilient, wear leveling, small RAM footprint
 *          Use Cases: Parameter storage, small log files, configuration on flash-only boards
 * 
 * @note Automatically enabled when HAL_OS_LITTLEFS_IO is defined (boards with SPI flash)
 * @note Requires AP_FILESYSTEM_LITTLEFS_FLASH_TYPE to match hardware
 * @note Binary size impact: ~25KB code
 * 
 * @see AP_FILESYSTEM_LITTLEFS_FLASH_TYPE for flash hardware configuration
 */
#ifndef AP_FILESYSTEM_LITTLEFS_ENABLED
#define AP_FILESYSTEM_LITTLEFS_ENABLED HAL_OS_LITTLEFS_IO
#endif

/**
 * @def AP_FILESYSTEM_LITTLEFS_FLASH_TYPE
 * @brief LittleFS flash chip type selection
 * 
 * @details Configures LittleFS driver for specific flash hardware characteristics.
 *          Must match actual flash chip type for correct operation.
 * 
 *          Supported Flash Types:
 *          - AP_FILESYSTEM_FLASH_JEDEC_NOR: Standard SPI NOR flash (default)
 *          - AP_FILESYSTEM_FLASH_W25NXX: Winbond W25N NAND flash series
 * 
 *          Override Example:
 *          @code
 *          // In board hwdef for W25N NAND flash:
 *          define AP_FILESYSTEM_LITTLEFS_FLASH_TYPE AP_FILESYSTEM_FLASH_W25NXX
 *          @endcode
 * 
 * @warning Flash type must exactly match hardware - mismatch causes data corruption
 * @warning Changing flash type on existing filesystem erases all data
 * 
 * @see AP_FILESYSTEM_FLASH_JEDEC_NOR, AP_FILESYSTEM_FLASH_W25NXX
 */
#ifndef AP_FILESYSTEM_LITTLEFS_FLASH_TYPE
#define AP_FILESYSTEM_LITTLEFS_FLASH_TYPE AP_FILESYSTEM_FLASH_JEDEC_NOR
#endif

/**
 * @def AP_FILESYSTEM_PARAM_ENABLED
 * @brief Enable virtual @PARAM filesystem for parameter pack/unpack
 * 
 * @details Provides magic "@PARAM" filesystem prefix for parameter operations.
 *          Allows parameter backup/restore via standard file operations.
 *          Writing to @PARAM/filename.parm saves all parameters to file.
 *          Reading from @PARAM/filename.parm loads parameters from file.
 * 
 *          Virtual Filesystem: No physical storage, translates file ops to parameter ops
 *          Use Cases: Parameter backup, bulk parameter loading, GCS parameter transfer
 * 
 * @note Always enabled - core functionality required by parameter system
 * @note This is a virtual filesystem - no actual files exist
 * 
 * @see libraries/AP_Param/ for parameter storage system
 */
#ifndef AP_FILESYSTEM_PARAM_ENABLED
#define AP_FILESYSTEM_PARAM_ENABLED 1
#endif

/**
 * @def AP_FILESYSTEM_POSIX_ENABLED
 * @brief Enable POSIX filesystem backend (native OS filesystem)
 * 
 * @details Provides direct access to host operating system filesystem via POSIX API.
 *          Used for SITL simulation and Linux-based flight controllers for development
 *          and testing. Allows file operations on host filesystem (reads/writes to disk).
 * 
 *          Platforms: SITL (simulation), Linux HAL, QURT (Qualcomm Hexagon DSP)
 *          Use Cases: SITL development, log replay, Linux-based autopilots
 * 
 * @note Automatically enabled on POSIX-compliant platforms (SITL, Linux, QURT)
 * @note Not available on embedded RTOS platforms (ChibiOS, ESP32 native)
 * 
 * @see AP_FILESYSTEM_POSIX_MAP_FILENAME_ALLOC for base directory mapping
 * @see libraries/AP_HAL_SITL/ for SITL implementation
 */
#ifndef AP_FILESYSTEM_POSIX_ENABLED
#define AP_FILESYSTEM_POSIX_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX || CONFIG_HAL_BOARD == HAL_BOARD_QURT)
#endif

/**
 * @def AP_FILESYSTEM_ROMFS_ENABLED
 * @brief Enable ROMFS backend for embedded read-only files
 * 
 * @details Provides read-only access to files embedded in firmware at compile time.
 *          Files are stored in flash memory and accessed via "@ROMFS" prefix.
 *          Used for storing default configurations, Lua scripts, web UI assets.
 * 
 *          Storage Backend: Flash memory (program storage)
 *          Characteristics: Read-only, zero RAM overhead, compile-time embedded
 *          Use Cases: Default parameter files, embedded scripts, web interface files
 * 
 * @note Automatically enabled if HAL_HAVE_AP_ROMFS_EMBEDDED_H is defined
 * @note Files must be embedded at build time using build system tools
 * @note Binary size impact: Size of embedded files
 * 
 * @see libraries/AP_ROMFS/ for ROMFS implementation
 * @see Tools/scripts/ROMFS for file embedding tools
 */
#ifndef AP_FILESYSTEM_ROMFS_ENABLED
#define AP_FILESYSTEM_ROMFS_ENABLED defined(HAL_HAVE_AP_ROMFS_EMBEDDED_H)
#endif

/**
 * @def AP_FILESYSTEM_SYS_ENABLED
 * @brief Enable virtual @SYS filesystem for system diagnostics
 * 
 * @details Provides magic "@SYS" filesystem prefix for runtime system information.
 *          Allows reading system state via standard file operations.
 *          Virtual files expose internal state: threads, memory, timers, versions.
 * 
 *          Virtual Filesystem: No physical storage, generates data on read
 *          Available Files: @SYS/threads.txt, @SYS/version.txt, etc.
 *          Use Cases: System diagnostics, health monitoring, debugging
 * 
 * @note Always enabled - core functionality for system diagnostics
 * @note This is a virtual filesystem - files are generated on demand
 * @note Reading @SYS files may briefly block for data collection
 * 
 * @see AP_FILESYSTEM_SYS_FLASH_ENABLED for flash memory inspection feature
 */
#ifndef AP_FILESYSTEM_SYS_ENABLED
#define AP_FILESYSTEM_SYS_ENABLED 1
#endif

/** @} */ // end of filesystem_backends

/**
 * @defgroup filesystem_posix_flags POSIX Backend Configuration
 * @brief POSIX-specific filesystem configuration options
 * @{
 */

/**
 * @def AP_FILESYSTEM_POSIX_MAP_FILENAME_ALLOC
 * @brief Enable dynamic base directory mapping for POSIX backend
 * 
 * @details When enabled, allows runtime configuration of base directory path
 *          for file operations. Requires AP_FILESYSTEM_POSIX_MAP_FILENAME_BASEDIR
 *          to be defined with the base directory path.
 * 
 *          Use Case: Map ArduPilot filesystem root to specific host directory
 *          Example: Map "/" to "/tmp/ardupilot/" for SITL isolation
 * 
 * @note Disabled by default to reduce memory overhead
 * @note Only applicable when AP_FILESYSTEM_POSIX_ENABLED is true
 */
#ifndef AP_FILESYSTEM_POSIX_MAP_FILENAME_ALLOC
// this requires AP_FILESYSTEM_POSIX_MAP_FILENAME_BASEDIR
#define AP_FILESYSTEM_POSIX_MAP_FILENAME_ALLOC 0
#endif

/** @} */ // end of filesystem_posix_flags

/**
 * @defgroup filesystem_capability_flags System-Wide Filesystem Capability Flags
 * @brief Aggregate flags indicating overall filesystem read/write capabilities
 * 
 * @details These flags combine individual backend capabilities to provide
 *          system-wide feature detection at compile time. Code can conditionally
 *          compile file I/O features based on these aggregate flags.
 * 
 *          Usage Pattern:
 *          @code
 *          #if AP_FILESYSTEM_FILE_WRITING_ENABLED
 *              // Code that requires file write capability (e.g., logging)
 *          #endif
 *          @endcode
 * 
 * @{
 */

/**
 * @def AP_FILESYSTEM_FILE_WRITING_ENABLED
 * @brief System-wide file writing capability flag
 * 
 * @details True if any physical storage backend is enabled that supports writing.
 *          Indicates that the system can open and write non-virtual files.
 * 
 *          Includes: ESP32, FATFS, LittleFS, POSIX (physical storage backends)
 *          Excludes: @SYS, @PARAM, @MISSION (virtual filesystems)
 *          Excludes: ROMFS (read-only filesystem)
 * 
 *          Dependent Features:
 *          - AP_Logger (DataFlash logging) requires this for log writes
 *          - Parameter backup to file requires this
 *          - Terrain data caching requires this
 *          - Mission storage to file requires this
 * 
 * @note Used by code to conditionally compile write-dependent features
 * @note This only indicates physical storage capability, not specific filesystem type
 * @warning Disabling all physical backends (this flag false) removes logging capability
 * 
 * @see AP_FILESYSTEM_FILE_READING_ENABLED for read capability
 * @see libraries/AP_Logger/ for logging implementation
 */
#ifndef AP_FILESYSTEM_FILE_WRITING_ENABLED
#define AP_FILESYSTEM_FILE_WRITING_ENABLED (AP_FILESYSTEM_ESP32_ENABLED || AP_FILESYSTEM_FATFS_ENABLED || AP_FILESYSTEM_LITTLEFS_ENABLED || AP_FILESYSTEM_POSIX_ENABLED)
#endif

/**
 * @def AP_FILESYSTEM_FILE_READING_ENABLED
 * @brief System-wide file reading capability flag
 * 
 * @details True if any backend supports reading files (physical or virtual).
 *          Broader than FILE_WRITING_ENABLED - includes read-only and virtual backends.
 * 
 *          Includes: All backends that support FILE_WRITING_ENABLED
 *          Plus: ROMFS (read-only embedded files)
 *          Plus: @SYS (virtual diagnostic files)
 *          Plus: @PARAM (virtual parameter files)
 *          Excludes: @MISSION when AP_MISSION_ENABLED is false
 * 
 *          Dependent Features:
 *          - Parameter loading from file requires this
 *          - ROMFS script/config loading requires this
 *          - System diagnostics (@SYS) requires this
 *          - Log replay requires this
 * 
 * @note Nearly always true - at minimum @PARAM and @SYS are enabled
 * @note This does not guarantee writable storage - check FILE_WRITING_ENABLED for that
 * 
 * @see AP_FILESYSTEM_FILE_WRITING_ENABLED for write capability
 */
#ifndef AP_FILESYSTEM_FILE_READING_ENABLED
#define AP_FILESYSTEM_FILE_READING_ENABLED (AP_FILESYSTEM_FILE_WRITING_ENABLED || AP_FILESYSTEM_ROMFS_ENABLED || AP_FILESYSTEM_SYS_ENABLED || AP_FILESYSTEM_PARAM_ENABLED)
#endif

/** @} */ // end of filesystem_capability_flags

/**
 * @defgroup filesystem_platform_flags Platform-Specific Filesystem Features
 * @brief Platform-dependent filesystem capability flags
 * @{
 */

/**
 * @def AP_FILESYSTEM_SYS_FLASH_ENABLED
 * @brief Enable @SYS flash memory access feature
 * 
 * @details Allows reading raw flash memory content via @SYS filesystem for debugging.
 *          ChibiOS-only feature that exposes flash memory regions as virtual files.
 * 
 *          Virtual Files: @SYS/flash/* (platform-specific memory regions)
 *          Use Cases: Flash inspection, bootloader debugging, memory corruption analysis
 * 
 * @note Only available on ChibiOS-based flight controllers
 * @note Requires understanding of board memory layout
 * @warning Reading arbitrary flash addresses can expose sensitive data
 * @warning Use only for development and debugging purposes
 */
#ifndef AP_FILESYSTEM_SYS_FLASH_ENABLED
#define AP_FILESYSTEM_SYS_FLASH_ENABLED CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#endif

/**
 * @def AP_FILESYSTEM_HAVE_DIRENT_DTYPE
 * @brief Platform supports dirent d_type field for directory entry type detection
 * 
 * @details Indicates that the platform's dirent structure includes the d_type field,
 *          which provides file type information (regular file, directory, symlink, etc.)
 *          without requiring a separate stat() call. This optimization improves
 *          directory listing performance.
 * 
 *          When true: File type available from readdir(), no stat() needed
 *          When false: Must call stat() to determine file type (performance penalty)
 * 
 * @note Default true - most modern platforms support d_type
 * @note Disable for legacy POSIX systems without d_type support
 */
#ifndef AP_FILESYSTEM_HAVE_DIRENT_DTYPE
#define AP_FILESYSTEM_HAVE_DIRENT_DTYPE 1
#endif

/** @} */ // end of filesystem_platform_flags

/**
 * @defgroup filesystem_virtual_mission Mission Virtual Filesystem
 * @brief Virtual filesystem for mission upload/download
 * @{
 */

/**
 * @def AP_FILESYSTEM_MISSION_ENABLED
 * @brief Enable virtual @MISSION filesystem for mission operations
 * 
 * @details Provides magic "@MISSION" filesystem prefix for mission upload/download.
 *          Allows mission transfer via standard file operations instead of MAVLink protocol.
 *          Writing to @MISSION uploads mission items, reading downloads current mission.
 * 
 *          Virtual Filesystem: No physical storage, translates file ops to mission commands
 *          Use Cases: Bulk mission upload, mission backup, non-MAVLink mission transfer
 * 
 * @note Automatically enabled when AP_MISSION_ENABLED is true
 * @note This is a virtual filesystem - no actual mission files exist
 * @note Mission data ultimately stored via AP_Mission in parameter storage
 * 
 * @see libraries/AP_Mission/ for mission management system
 */
#ifndef AP_FILESYSTEM_MISSION_ENABLED
#include <AP_Mission/AP_Mission_config.h>
#define AP_FILESYSTEM_MISSION_ENABLED AP_MISSION_ENABLED
#endif

/** @} */ // end of filesystem_virtual_mission
