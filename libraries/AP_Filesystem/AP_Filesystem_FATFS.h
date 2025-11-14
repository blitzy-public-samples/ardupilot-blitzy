/**
 * @file AP_Filesystem_FATFS.h
 * @brief FATFS filesystem backend for SD card support
 * 
 * @details Provides FAT32 filesystem access on SD cards using the FatFs library (ff.h).
 *          Primary storage backend for data logging, mission files, and parameters on
 *          embedded flight controllers.
 * 
 *          Platform: ChibiOS builds with HAL_OS_FATFS_IO enabled
 *          Architecture: Manages bounded file table, serializes operations with semaphore,
 *                       implements DMA-aware chunking
 * 
 * Source: libraries/AP_Filesystem/AP_Filesystem_FATFS.h
 */

#pragma once

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <stddef.h>
#include "AP_Filesystem_backend.h"

#if AP_FILESYSTEM_FATFS_ENABLED

// Seek offset macros
#define SEEK_SET 0
#define SEEK_CUR 1
#define SEEK_END 2

/**
 * @class AP_Filesystem_FATFS
 * @brief FAT filesystem backend using FatFs library for SD card access
 * 
 * @details This backend wraps the FatFs library (http://elm-chan.org/fsw/ff/) to provide
 *          reliable SD card filesystem access on embedded platforms. Features include:
 *          - Bounded file table (MAX_FILES concurrent open files)
 *          - Semaphore-serialized operations for thread safety
 *          - DMA-aware I/O chunking (MAX_IO_SIZE 4096 bytes per operation)
 *          - Automatic remount on transient errors
 *          - Asynchronous format via scheduler
 *          - FRESULT to errno error code mapping
 *          - Disk free space queries via f_getfree()
 * 
 * @note Enabled via AP_FILESYSTEM_FATFS_ENABLED (HAL_OS_FATFS_IO)
 * @warning SD card operations can block for 10-100ms - gated during armed flight
 * 
 * Primary use case: Data logging (AP_Logger), parameter storage, mission files, terrain data
 * 
 * Thread Safety: All operations protected by HAL_Semaphore (WITH_SEMAPHORE)
 * Performance: DMA-aware chunking ensures memory safety on ChibiOS
 * Error Handling: FRESULT errors mapped to errno (FR_NO_FILE→ENOENT, FR_DENIED→EACCES, FR_DISK_ERR→EIO)
 * Configuration: Enabled via AP_FILESYSTEM_FATFS_ENABLED, file table size (MAX_FILES) and I/O chunk size (MAX_IO_SIZE=4096) defined in implementation
 * 
 * Source: libraries/AP_Filesystem/AP_Filesystem_FATFS.h, AP_Filesystem_FATFS.cpp
 */
class AP_Filesystem_FATFS : public AP_Filesystem_Backend
{
public:
    // File operation methods - closely match POSIX calls
    
    /**
     * @brief Open or create file on SD card
     * 
     * @param[in] fname File path (relative to SD card root)
     * @param[in] flags POSIX flags: O_RDONLY, O_WRONLY, O_RDWR, O_CREAT, O_TRUNC, O_APPEND
     * @param[in] allow_absolute_paths Allow absolute paths without prefix stripping
     * @return File descriptor (index into file_table) or -1 on error
     * 
     * @details Translates POSIX flags to FatFs FA_* flags, allocates file_table slot, calls f_open()
     * 
     * @warning Limited to MAX_FILES concurrent opens (typically 16-32)
     * @note Sets errno on failure (EMFILE if table full, EIO for FatFs errors)
     */
    int open(const char *fname, int flags, bool allow_absolute_paths = false) override;
    
    /**
     * @brief Close file and release table slot
     * 
     * @param[in] fd File descriptor from open()
     * @return 0 on success, -1 on error
     * 
     * @details Calls f_close(), frees file_table entry
     * @note File table slot immediately available for reuse
     */
    int close(int fd) override;
    
    /**
     * @brief Read data from file with DMA-safe chunking
     * 
     * @param[in]  fd    File descriptor
     * @param[out] buf   Destination buffer (must be DMA-safe if chunking)
     * @param[in]  count Bytes to read
     * @return Bytes read, 0 on EOF, -1 on error
     * 
     * @details Calls f_read() with automatic retry on transient errors, chunks large reads to MAX_IO_SIZE (4096) for DMA compatibility
     * @note May return partial read on EOF or error
     */
    int32_t read(int fd, void *buf, uint32_t count) override;
    
    /**
     * @brief Write data to file with DMA-safe chunking
     * 
     * @param[in] fd    File descriptor
     * @param[in] buf   Source buffer (must be DMA-safe if chunking)
     * @param[in] count Bytes to write
     * @return Bytes written, -1 on error
     * 
     * @details Calls f_write() with chunking, may retry mount on certain errors
     * @warning May return partial write on disk full or error
     */
    int32_t write(int fd, const void *buf, uint32_t count) override;
    
    /**
     * @brief Synchronize file data to SD card
     * 
     * @param[in] fd File descriptor
     * @return 0 on success, -1 on error
     * 
     * @details Calls f_sync() to flush FatFs buffers and commit to SD card
     * @note Critical for data integrity (logging, parameters)
     * @warning Can block for 10-50ms on SD card write
     */
    int fsync(int fd) override;
    
    /**
     * @brief Seek to file position
     * 
     * @param[in] fd     File descriptor
     * @param[in] offset Byte offset
     * @param[in] whence Origin: SEEK_SET (0), SEEK_CUR (1), SEEK_END (2)
     * @return New position or -1 on error
     * 
     * @details Calls f_lseek(), handles SEEK_END by getting file size first
     */
    int32_t lseek(int fd, int32_t offset, int whence) override;
    
    /**
     * @brief Get file/directory information
     * 
     * @param[in]  pathname Path to file or directory
     * @param[out] stbuf    Output stat structure (size, mtime, mode)
     * @return 0 on success, -1 on error
     * 
     * @details Uses f_stat() to populate struct stat
     */
    int stat(const char *pathname, struct stat *stbuf) override;
    
    /**
     * @brief Delete file
     * 
     * @param[in] pathname File path
     * @return 0 on success, -1 on error
     * 
     * @details Calls f_unlink()
     * @warning Cannot delete open files
     */
    int unlink(const char *pathname) override;
    
    /**
     * @brief Create directory
     * 
     * @param[in] pathname Directory path
     * @return 0 on success, -1 on error
     * 
     * @details Calls f_mkdir(), creates single level
     */
    int mkdir(const char *pathname) override;
    
    /**
     * @brief Open directory for reading
     * 
     * @param[in] pathname Directory path
     * @return DIR* handle (cast to void*) or NULL
     * 
     * @details Allocates DIR structure, calls f_opendir()
     */
    void *opendir(const char *pathname) override;
    
    /**
     * @brief Rename or move file/directory
     * 
     * @param[in] oldpath Source path
     * @param[in] newpath Destination path
     * @return 0 on success, -1 on error
     * 
     * @details Calls f_rename()
     */
    int rename(const char *oldpath, const char *newpath) override;
    
    /**
     * @brief Read next directory entry
     * 
     * @param[in] dirp Directory handle
     * @return dirent* with d_name and d_type, NULL at end
     * 
     * @details Calls f_readdir(), translates FILINFO to dirent
     * @note d_type populated from AM_DIR attribute
     */
    struct dirent *readdir(void *dirp) override;
    
    /**
     * @brief Close directory
     * 
     * @param[in] dirp Directory handle
     * @return 0 on success, -1 on error
     * 
     * @details Calls f_closedir(), frees DIR structure
     */
    int closedir(void *dirp) override;

    // Storage management methods
    
    /**
     * @brief Get recommended bytes before next fsync
     * 
     * @param[in] fd File descriptor
     * @return Bytes (typically 4096-8192)
     * 
     * @details Provides adaptive threshold to balance write performance and data integrity
     * @note Used by AP_Logger to schedule log flushes
     */
    uint32_t bytes_until_fsync(int fd) override;

    /**
     * @brief Get free space on SD card
     * 
     * @param[in] path Path on SD card
     * @return Free bytes or -1 on error
     * 
     * @details Calls f_getfree() to query FAT filesystem
     * @note Can be slow (100ms+) on large cards
     */
    int64_t disk_free(const char *path) override;

    /**
     * @brief Get total SD card capacity
     * 
     * @param[in] path Path on SD card
     * @return Total bytes or -1 on error
     * 
     * @details Calculates from f_getfree() volume size
     */
    int64_t disk_space(const char *path) override;

    /**
     * @brief Set file modification timestamp
     * 
     * @param[in] filename  File path
     * @param[in] mtime_sec Unix timestamp (seconds since 1970)
     * @return true on success
     * 
     * @details Converts Unix time to FAT timestamp, calls f_utime()
     */
    bool set_mtime(const char *filename, const uint32_t mtime_sec) override;

    // Mount management methods
    
    /**
     * @brief Attempt to remount SD card after error
     * 
     * @return true if mount successful
     * 
     * @details Called on transient errors (card removed/reinserted, corruption recovery)
     * @note Automatic retry logic for resilience
     * @warning Closes all open files on remount
     */
    bool retry_mount(void) override;

    /**
     * @brief Unmount filesystem for safe reboot
     * 
     * @details Calls f_unmount(), flushes all pending writes
     * @note Called during shutdown sequence
     * @warning File operations fail after unmount
     */
    void unmount(void) override;

    // Format methods
    
    /**
     * @brief Start asynchronous SD card format
     * 
     * @return true if format initiated
     * 
     * @details Schedules format_handler() on task scheduler, uses f_mkfs()
     * @warning Erases all data on SD card
     * @note Non-blocking - monitor with get_format_status()
     */
    bool format(void) override;
    
    /**
     * @brief Query format operation progress
     * 
     * @return FormatStatus: NOT_STARTED, IN_PROGRESS, SUCCESS, or FAILED
     * 
     * @details Thread-safe status query for async format
     */
    AP_Filesystem_Backend::FormatStatus get_format_status() const override;

private:
    /**
     * @brief Internal async format handler
     * 
     * @details Runs on scheduler, calls f_mkfs(), updates format_status
     * @note Private implementation detail, called by scheduler
     */
    void format_handler(void);
    
    /**
     * @brief Current format operation status
     * 
     * @details Thread-safe status variable for async format tracking
     */
    FormatStatus format_status;
};

#endif  // #if AP_FILESYSTEM_FATFS_ENABLED
