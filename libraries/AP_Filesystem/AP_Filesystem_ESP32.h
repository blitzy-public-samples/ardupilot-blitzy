/**
 * @file AP_Filesystem_ESP32.h
 * @brief ESP32 filesystem backend implementation
 * 
 * @details This file provides the ESP32-specific filesystem backend for ArduPilot,
 *          implementing a thin wrapper around POSIX/FatFs calls for ESP-IDF builds.
 *          It enables SD card and flash filesystem access on ESP32 hardware through
 *          native ESP-IDF VFS (Virtual File System) integration with FatFs backend.
 * 
 *          Architecture: Native ESP-IDF VFS integration with FatFs backend
 *          Platform: ESP32 only (enabled via AP_FILESYSTEM_ESP32_ENABLED)
 * 
 * @note This backend leverages ESP-IDF's built-in VFS layer which provides POSIX
 *       compatibility for filesystem operations across different storage backends.
 */

#pragma once

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <dirent.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"

#include "AP_Filesystem_backend.h"

/**
 * @class AP_Filesystem_ESP32
 * @brief ESP32-specific filesystem backend using ESP-IDF VFS layer
 * 
 * @details This backend provides filesystem access on ESP32 platforms by wrapping
 *          native POSIX calls and ESP-IDF FatFs utilities. It supports SD card access
 *          via SDMMC/SPI interfaces and internal flash partitions. Unlike other backends,
 *          this leverages ESP-IDF's built-in VFS (Virtual File System) which provides
 *          POSIX compatibility across different storage types.
 * 
 *          Key capabilities:
 *          - SD card access via SDMMC (4-bit) or SPI interfaces through ESP-IDF
 *          - Flash filesystem partitions (must be mounted by ESP-IDF before use)
 *          - Standard POSIX-like file operations (open, read, write, seek, etc.)
 *          - Directory operations (opendir, readdir, closedir)
 *          - File metadata operations (stat, rename, unlink)
 *          - Storage space queries (disk_free, disk_space)
 * 
 *          Integration:
 *          - Uses ESP-IDF VFS mount points for path resolution
 *          - Thread safety provided by ESP-IDF VFS layer
 *          - Error codes follow POSIX errno conventions
 *          - No internal caching - each operation goes directly to hardware
 * 
 * @note This backend is automatically enabled on CONFIG_HAL_BOARD == HAL_BOARD_ESP32
 * @note Integrates with ESP-IDF FatFs component for SD card access
 * @note Path handling depends on ESP-IDF VFS mount points configured at startup
 * 
 * @warning File operations use blocking ESP-IDF calls - avoid in time-critical paths
 * @warning SD card operations can be slow (10-100ms) - avoid in main thread when armed
 * @warning Some ESP-IDF errors may not map cleanly to POSIX errno values
 */
class AP_Filesystem_ESP32 : public AP_Filesystem_Backend
{
public:
    /**
     * @brief Open or create file using ESP-IDF VFS
     * 
     * @details Opens a file through ESP-IDF's VFS layer using the underlying POSIX ::open()
     *          system call. Supports standard POSIX flags for file access modes and creation.
     *          Forces O_TRUNC and O_CLOEXEC flags for safety in the ArduPilot environment.
     * 
     * @param[in] fname File path (relative or absolute depending on allow_absolute_paths)
     * @param[in] flags POSIX open flags: O_RDONLY, O_WRONLY, O_RDWR, O_CREAT, O_TRUNC, O_APPEND
     * @param[in] allow_absolute_paths If true, allow absolute paths without prefix stripping
     * 
     * @return File descriptor (>=0) on success, -1 on error (check errno for details)
     * 
     * @note File descriptors are native ESP-IDF descriptors, not AP_Filesystem encoded
     * @note O_TRUNC and O_CLOEXEC flags are automatically applied for safety
     * @note Check errno after failure for detailed error information
     * 
     * @see close(), read(), write()
     */
    int open(const char *fname, int flags, bool allow_absolute_paths = false) override;
    
    /**
     * @brief Close file descriptor
     * 
     * @details Closes an open file descriptor using the ESP-IDF VFS ::close() system call.
     *          Flushes any buffered data and releases resources associated with the file.
     * 
     * @param[in] fd File descriptor from open()
     * 
     * @return 0 on success, -1 on error (check errno for details)
     * 
     * @note Always close file descriptors when done to prevent resource leaks
     * 
     * @see open()
     */
    int close(int fd) override;
    
    /**
     * @brief Read data from file
     * 
     * @details Reads up to count bytes from the file into the buffer using the ESP-IDF
     *          VFS ::read() system call. May return fewer bytes than requested if end of
     *          file is reached or if interrupted.
     * 
     * @param[in]  fd    File descriptor from open()
     * @param[out] buf   Destination buffer for read data
     * @param[in]  count Maximum number of bytes to read
     * 
     * @return Number of bytes actually read (may be less than count), 0 on EOF, -1 on error
     * 
     * @note May return partial reads even when data is available
     * @note Returns 0 when attempting to read at end of file
     * @note SD card reads can take 10-100ms depending on data size
     * 
     * @warning Buffer must be at least count bytes in size
     * 
     * @see write(), lseek()
     */
    int32_t read(int fd, void *buf, uint32_t count) override;
    
    /**
     * @brief Write data to file
     * 
     * @details Writes up to count bytes from buffer to the file using the ESP-IDF VFS
     *          ::write() system call. May return fewer bytes than requested if storage
     *          is full or if interrupted.
     * 
     * @param[in] fd    File descriptor from open()
     * @param[in] buf   Source buffer containing data to write
     * @param[in] count Number of bytes to write
     * 
     * @return Number of bytes actually written (may be less than count), -1 on error
     * 
     * @note May return partial writes - check return value and retry if needed
     * @note Data may be buffered - call fsync() to ensure it reaches storage
     * @note SD card writes can take 10-100ms depending on data size
     * 
     * @warning Ensure sufficient free space before writing large amounts of data
     * 
     * @see read(), fsync()
     */
    int32_t write(int fd, const void *buf, uint32_t count) override;
    
    /**
     * @brief Synchronize file data to storage
     * 
     * @details Flushes all buffered data for the file to the underlying storage device
     *          using the ESP-IDF VFS ::fsync() system call. Ensures data persistence in
     *          case of power loss or system crash.
     * 
     * @param[in] fd File descriptor from open()
     * 
     * @return 0 on success, -1 on error
     * 
     * @note Call periodically when writing critical data (e.g., logs, parameters)
     * @note Can take significant time (10-100ms) on SD cards
     * @note See bytes_until_fsync() for advisory sync frequency
     * 
     * @warning Avoid calling too frequently as it impacts write performance
     * 
     * @see write(), bytes_until_fsync()
     */
    int fsync(int fd) override;
    
    /**
     * @brief Seek to file position
     * 
     * @details Changes the file offset for subsequent read/write operations using the
     *          ESP-IDF VFS ::lseek() system call. Supports seeking relative to start,
     *          current position, or end of file.
     * 
     * @param[in] fd     File descriptor from open()
     * @param[in] offset Byte offset for new position
     * @param[in] whence Origin for offset: SEEK_SET (0), SEEK_CUR (1), or SEEK_END (2)
     * 
     * @return New absolute file position in bytes from start, or -1 on error
     * 
     * @note SEEK_SET: offset from beginning of file
     * @note SEEK_CUR: offset from current position
     * @note SEEK_END: offset from end of file
     * @note Cannot seek beyond end of file for reading, but can for writing (creates hole)
     * 
     * @see read(), write()
     */
    int32_t lseek(int fd, int32_t offset, int whence) override;
    
    /**
     * @brief Get file status information
     * 
     * @details Retrieves file metadata including size, modification time, and permissions
     *          using the ESP-IDF VFS ::stat() system call. Does not require file to be open.
     * 
     * @param[in]  pathname Path to file or directory
     * @param[out] stbuf    Pointer to stat structure to populate with file information
     * 
     * @return 0 on success, -1 on error (file not found or permission denied)
     * 
     * @note Populates st_size (file size in bytes), st_mtime (modification time), st_mode (type and permissions)
     * @note Use S_ISREG() macro to check if regular file, S_ISDIR() for directory
     * 
     * @see unlink(), rename()
     */
    int stat(const char *pathname, struct stat *stbuf) override;
    
    /**
     * @brief Delete file
     * 
     * @details Removes a file from the filesystem using the ESP-IDF VFS ::unlink() system call.
     *          File must not be open. Frees storage space occupied by the file.
     * 
     * @param[in] pathname Path to file to delete
     * 
     * @return 0 on success, -1 on error (file not found, in use, or permission denied)
     * 
     * @note Cannot delete directories - use rmdir() instead (not currently exposed)
     * @note Cannot delete open files - close all file descriptors first
     * @note Deletion is immediate and cannot be undone
     * 
     * @warning Ensure file is closed before unlinking to avoid filesystem corruption
     * 
     * @see rename(), stat()
     */
    int unlink(const char *pathname) override;
    
    /**
     * @brief Create directory
     * 
     * @details Creates a new directory with default permissions using the ESP-IDF VFS
     *          ::mkdir() system call. Creates only a single directory level - parent
     *          directories must already exist.
     * 
     * @param[in] pathname Path for new directory
     * 
     * @return 0 on success, -1 on error (already exists, parent missing, or no space)
     * 
     * @note Does not create intermediate directories - use multiple mkdir() calls
     * @note Directory names follow same restrictions as file names
     * @note On FAT filesystems, directory entries consume space like files
     * 
     * @see opendir(), closedir()
     */
    int mkdir(const char *pathname) override;
    
    /**
     * @brief Open directory for reading
     * 
     * @details Opens a directory stream for reading entries using the ESP-IDF VFS
     *          ::opendir() system call. Returns a handle for subsequent readdir() calls.
     * 
     * @param[in] pathname Path to directory to open
     * 
     * @return Directory handle (cast DIR* to void*) on success, NULL on error
     * 
     * @note Must call closedir() when done to release resources
     * @note Directory handle is opaque - only pass to readdir() and closedir()
     * @note Entries returned in filesystem-dependent order (not alphabetical)
     * 
     * @see readdir(), closedir()
     */
    void *opendir(const char *pathname) override;
    
    /**
     * @brief Read next directory entry
     * 
     * @details Retrieves the next file or subdirectory entry from an open directory
     *          using the ESP-IDF VFS ::readdir() system call. Returns NULL when all
     *          entries have been read.
     * 
     * @param[in] dirp Directory handle from opendir()
     * 
     * @return Pointer to dirent structure with d_name and d_type, or NULL at end/error
     * 
     * @note Returned pointer is internal and invalidated by next readdir() or closedir()
     * @note d_name contains entry name (not full path)
     * @note d_type indicates DT_REG (file), DT_DIR (directory), or DT_UNKNOWN
     * @note Special entries "." and ".." are included on some filesystems
     * 
     * @see opendir(), closedir()
     */
    struct dirent *readdir(void *dirp) override;
    
    /**
     * @brief Close directory handle
     * 
     * @details Closes an open directory stream and releases associated resources using
     *          the ESP-IDF VFS ::closedir() system call.
     * 
     * @param[in] dirp Directory handle from opendir()
     * 
     * @return 0 on success, -1 on error
     * 
     * @note Always close directory handles to prevent resource leaks
     * @note Handle becomes invalid after successful close
     * 
     * @see opendir(), readdir()
     */
    int closedir(void *dirp) override;
    
    /**
     * @brief Rename or move file
     * 
     * @details Renames a file or moves it to a different directory using the ESP-IDF VFS
     *          ::rename() system call. Can operate on files and directories. Atomic on
     *          same filesystem.
     * 
     * @param[in] oldpath Current file path
     * @param[in] newpath New file path
     * 
     * @return 0 on success, -1 on error (source missing, destination exists, cross-device)
     * 
     * @note Can move files across directories on same filesystem
     * @note Fails if newpath already exists
     * @note Atomic operation on single filesystem (either completes fully or not at all)
     * @note May fail when moving across different filesystem types
     * 
     * @see unlink(), stat()
     */
    int rename(const char *oldpath, const char *newpath) override;

    /**
     * @brief Get advisory fsync threshold
     * 
     * @details Returns a suggested number of bytes to write before calling fsync() for
     *          optimal performance. This is a fixed advisory value rather than dynamically
     *          calculated based on actual buffer state.
     * 
     * @param[in] fd File descriptor (unused, included for API compatibility)
     * 
     * @return 4096 bytes (fixed advisory threshold)
     * 
     * @note Provides consistent 4KB threshold for periodic fsync calls
     * @note Not dynamically calculated - same value returned regardless of fd state
     * @note Balances data safety (frequent syncs) vs performance (batched writes)
     * @note Actual optimal value depends on workload and storage device characteristics
     * 
     * @see fsync()
     */
    uint32_t bytes_until_fsync(int fd) override;

    /**
     * @brief Get free disk space
     * 
     * @details Queries available storage space on the volume containing the specified path.
     *          Uses FatFs f_getfree() for "/SDCARD/" volume. Returns -1 for other paths
     *          as ESP-IDF flash partitions may not report accurate free space.
     * 
     * @param[in] path Path on target volume (typically "/SDCARD/")
     * 
     * @return Free space in bytes on success, -1 on error or unsupported filesystem
     * 
     * @note Only accurate for "/SDCARD/" path on ESP32 (uses FatFs query)
     * @note ESP-IDF flash partitions (SPIFFS, LittleFS) may return -1
     * @note Can be slow on large SD cards (100-500ms)
     * @note Free space calculation includes filesystem overhead
     * 
     * @warning Do not call frequently - cache result if needed
     * 
     * @see disk_space()
     */
    int64_t disk_free(const char *path) override;

    /**
     * @brief Get total disk capacity
     * 
     * @details Queries total storage capacity of the volume containing the specified path.
     *          Calculates from FatFs volume size for SD card. Returns -1 for unsupported
     *          filesystems.
     * 
     * @param[in] path Path on target volume
     * 
     * @return Total space in bytes on success, -1 on error or unsupported filesystem
     * 
     * @note Returns raw capacity, not accounting for filesystem overhead
     * @note Same performance and path restrictions as disk_free()
     * 
     * @see disk_free()
     */
    int64_t disk_space(const char *path) override;

    /**
     * @brief Set file modification time
     * 
     * @details Updates the modification timestamp of a file using the utime() system call
     *          if available in ESP-IDF. Useful for preserving timestamps when copying files
     *          or restoring from backups.
     * 
     * @param[in] filename  Path to file to modify
     * @param[in] mtime_sec Modification time as Unix timestamp (seconds since 1970-01-01)
     * 
     * @return true on success, false on error (file not found or utime not supported)
     * 
     * @note Requires utime() support in ESP-IDF VFS implementation
     * @note Some filesystems may not support setting mtime (returns false)
     * @note Does not modify file contents or other metadata
     * 
     * @see stat()
     */
    bool set_mtime(const char *filename, const uint32_t mtime_sec) override;
};

