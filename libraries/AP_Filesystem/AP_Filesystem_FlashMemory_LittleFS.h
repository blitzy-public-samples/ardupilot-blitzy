/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file AP_Filesystem_FlashMemory_LittleFS.h
 * @brief LittleFS filesystem backend for internal SPI flash storage
 * 
 * @details Provides wear-leveling, power-safe filesystem on SPI NOR/NAND flash chips 
 *          using LittleFS library. Primary use: parameter storage, small configuration 
 *          files, terrain cache on flash-equipped boards.
 * 
 * Platform: Boards with SPI flash chips (enabled via HAL_OS_LITTLEFS_IO)
 * 
 * Architecture: JEDEC flash detection, lazy mount, SPI device abstraction, 
 *               semaphore-protected operations
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 * 
 * Source: libraries/AP_Filesystem/AP_Filesystem_FlashMemory_LittleFS.h
 */

#pragma once

#include "AP_Filesystem_backend.h"

#if AP_FILESYSTEM_LITTLEFS_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include "lfs.h"

/**
 * @class AP_Filesystem_FlashMemory_LittleFS
 * @brief LittleFS backend for power-safe flash filesystem with wear leveling
 * 
 * @details This backend wraps the LittleFS library (https://github.com/littlefs-project/littlefs) 
 *          to provide reliable, power-safe file storage on internal SPI flash chips.
 * 
 * Key features:
 * - **Power-loss resilience**: Copy-on-write ensures filesystem consistency even during sudden power loss
 * - **Wear leveling**: Distributes writes across flash blocks to extend chip lifetime
 * - **Dynamic bad block management**: Detects and avoids failing flash blocks
 * - **JEDEC flash detection**: Auto-detects common NOR flash chips (W25Q, MX25, etc.) and NAND flash
 * - **Lazy mounting**: Filesystem mounted on first access, retried on errors
 * - **Compaction awareness**: bytes_until_fsync() heuristic to avoid triggering expensive compaction
 * - **SITL support**: File-backed block device for simulation testing
 * 
 * Primary use cases:
 * - Parameter storage (persistent across power cycles)
 * - Terrain data cache
 * - Small configuration files
 * 
 * Thread Safety:
 * - All operations protected by fs_sem (HAL_Semaphore)
 * - SPI device access protected by dev_sem
 * - File descriptors are thread-local (not shared across threads)
 * 
 * Performance Characteristics:
 * - Read latency: 0.1-1ms (depends on SPI speed, typically 10-40MHz)
 * - Write latency: 1-10ms (page program time)
 * - Erase latency: 10-200ms (sector/block erase)
 * - Compaction can cause 1-5 second stalls if triggered
 * 
 * Flash Chip Support:
 * - JEDEC-compliant NOR flash (W25Q series, MX25 series, etc.)
 * - Winbond W25N NAND flash series
 * - Auto-detection via JEDEC ID read
 * - Configurable via AP_FILESYSTEM_LITTLEFS_FLASH_TYPE
 * 
 * Error Handling:
 * - LittleFS errors mapped to errno codes
 * - Common errors: ENOSPC (flash full), EIO (flash failure), ENOENT (file not found)
 * - Repeated EIO errors trigger mark_dead() to prevent corruption
 * - Mount failures automatically retry or trigger format
 * 
 * @note Enabled via AP_FILESYSTEM_LITTLEFS_ENABLED (HAL_OS_LITTLEFS_IO)
 * @warning Flash has limited write endurance (10K-100K erase cycles) - avoid frequent small writes
 * @warning Flash operations can block 1-200ms (erase time)
 * @warning Compaction can cause 1-5 second stalls - use bytes_until_fsync() to avoid
 * 
 * Source: libraries/AP_Filesystem/AP_Filesystem_FlashMemory_LittleFS.h
 */
class AP_Filesystem_FlashMemory_LittleFS : public AP_Filesystem_Backend
{
public:
    // POSIX-like file operations
    
    /**
     * @brief Open file on flash filesystem
     * 
     * @details Translates to lfs_file_open(), allocates FileDescriptor, lazy-mounts 
     *          filesystem on first access.
     * 
     * @param[in] fname File path (relative to flash root)
     * @param[in] flags POSIX flags: O_RDONLY, O_WRONLY, O_RDWR, O_CREAT, O_TRUNC, O_APPEND
     * @param[in] allow_absolute_paths Allow absolute paths (default false)
     * 
     * @return File descriptor (>= 0) on success, -1 on error (errno set)
     * 
     * @note Limited to MAX_OPEN_FILES (16) concurrent opens
     * @note All operations trigger lazy mount if not already mounted
     * @warning First open after boot may trigger mount (100-500ms delay)
     */
    int open(const char *fname, int flags, bool allow_absolute_paths = false) override;
    
    /**
     * @brief Close file descriptor
     * 
     * @details Calls lfs_file_close(), flushes pending writes, releases FileDescriptor.
     * 
     * @param[in] fd File descriptor from open()
     * 
     * @return 0 on success, -1 on error
     * 
     * @note Implicitly syncs any buffered data
     */
    int close(int fd) override;
    
    /**
     * @brief Read data from file
     * 
     * @details Calls lfs_file_read(), reads from current file position.
     * 
     * @param[in] fd File descriptor
     * @param[out] buf Destination buffer
     * @param[in] count Bytes to read
     * 
     * @return Bytes read on success (may be less than count at EOF), -1 on error
     * 
     * @note Read latency: 0.1-1ms depending on SPI speed
     */
    int32_t read(int fd, void *buf, uint32_t count) override;
    
    /**
     * @brief Write data to file
     * 
     * @details Calls lfs_file_write(), writes at current file position. Operations are 
     *          power-safe (copy-on-write).
     * 
     * @param[in] fd File descriptor
     * @param[in] buf Source buffer
     * @param[in] count Bytes to write
     * 
     * @return Bytes written on success, -1 on error (errno: ENOSPC if full)
     * 
     * @note Write latency: 1-10ms (page program time)
     * @warning May trigger compaction if free space low (1-5 second stall)
     * @note Use bytes_until_fsync() to avoid unexpected compaction
     */
    int32_t write(int fd, const void *buf, uint32_t count) override;
    
    /**
     * @brief Sync file data to flash
     * 
     * @details Calls lfs_file_sync(), ensures all buffered data written to flash.
     * 
     * @param[in] fd File descriptor
     * 
     * @return 0 on success, -1 on error
     * 
     * @note Recommended to call periodically based on bytes_until_fsync()
     * @warning Can trigger compaction (1-5 second delay)
     */
    int fsync(int fd) override;
    
    /**
     * @brief Seek to file position
     * 
     * @details Calls lfs_file_seek(), changes current file position.
     * 
     * @param[in] fd File descriptor
     * @param[in] offset Offset in bytes
     * @param[in] whence SEEK_SET, SEEK_CUR, or SEEK_END
     * 
     * @return New file position on success, -1 on error
     */
    int32_t lseek(int fd, int32_t offset, int whence) override;
    
    /**
     * @brief Get file or directory status
     * 
     * @details Calls lfs_stat(), populates stat structure with file metadata.
     * 
     * @param[in] pathname File or directory path
     * @param[out] stbuf Stat structure to populate
     * 
     * @return 0 on success, -1 on error (errno: ENOENT if not found)
     * 
     * @note st_size contains file size, st_mtime contains modification time
     */
    int stat(const char *pathname, struct stat *stbuf) override;

    /**
     * @brief Delete file
     * 
     * @details Calls lfs_remove(), deletes file from filesystem.
     * 
     * @param[in] pathname File path to delete
     * 
     * @return 0 on success, -1 on error (errno: ENOENT if not found)
     * 
     * @note Power-safe deletion (atomic operation)
     */
    int unlink(const char *pathname) override;
    
    /**
     * @brief Create directory
     * 
     * @details Calls lfs_mkdir(), creates new directory.
     * 
     * @param[in] pathname Directory path to create
     * 
     * @return 0 on success, -1 on error (errno: EEXIST if already exists)
     * 
     * @note Parent directory must exist
     */
    int mkdir(const char *pathname) override;

    /**
     * @brief Open directory for reading
     * 
     * @details Allocates directory handle for iteration.
     * 
     * @param[in] pathname Directory path
     * 
     * @return Directory handle on success, NULL on error
     * 
     * @note Call closedir() when done to release resources
     */
    void *opendir(const char *pathname) override;
    
    /**
     * @brief Read next directory entry
     * 
     * @details Iterates through directory entries.
     * 
     * @param[in] dirp Directory handle from opendir()
     * 
     * @return Pointer to dirent structure, NULL at end or on error
     * 
     * @note Returns entries in arbitrary order
     */
    struct dirent *readdir(void *dirp) override;
    
    /**
     * @brief Close directory handle
     * 
     * @details Releases directory resources.
     * 
     * @param[in] dirp Directory handle from opendir()
     * 
     * @return 0 on success, -1 on error
     */
    int closedir(void *dirp) override;

    // Storage management methods
    
    /**
     * @brief Get recommended bytes before next fsync to avoid compaction
     * 
     * @details LittleFS performs expensive garbage collection/compaction when free space 
     *          is low. This heuristic suggests fsync() timing to avoid triggering 
     *          compaction mid-write.
     * 
     * @param[in] fd File descriptor
     * 
     * @return Adaptive threshold based on flash wear state (bytes)
     * 
     * @note Returns smaller values as flash fills up (triggers more frequent syncs)
     * @warning Ignoring this can cause 1-5 second pauses during compaction
     */
    uint32_t bytes_until_fsync(int fd) override;

    /**
     * @brief Query free space on flash
     * 
     * @details Calls lfs_fs_size() and calculates from block configuration.
     * 
     * @param[in] path Filesystem path (ignored, uses mounted filesystem)
     * 
     * @return Free bytes available, -1 on error
     * 
     * @note Free space calculation includes overhead for wear leveling
     */
    int64_t disk_free(const char *path) override;
    
    /**
     * @brief Query total flash capacity
     * 
     * @details Returns total filesystem size from configuration.
     * 
     * @param[in] path Filesystem path (ignored, uses mounted filesystem)
     * 
     * @return Total capacity in bytes, -1 on error
     */
    int64_t disk_space(const char *path) override;

    /**
     * @brief Set file modification time
     * 
     * @details Uses LittleFS custom attributes to store Unix timestamp.
     * 
     * @param[in] filename File path
     * @param[in] mtime_sec Modification time (Unix timestamp, seconds since epoch)
     * 
     * @return true on success, false on error
     * 
     * @note Mtime stored as 32-bit attribute, survives power cycles
     */
    bool set_mtime(const char *filename, const uint32_t mtime_sec) override;

    // Mount and format methods
    
    /**
     * @brief Attempt to remount flash filesystem after error
     * 
     * @details Tries lfs_mount(), falls back to format if corrupted.
     * 
     * @return true if mount successful
     * 
     * @note Multiple retry attempts with increasing delays
     * @warning Closes all open files on remount
     */
    bool retry_mount(void) override;
    
    /**
     * @brief Unmount filesystem for safe shutdown
     * 
     * @details Calls lfs_unmount(), flushes all pending writes.
     * 
     * @note Required before board power-down
     */
    void unmount(void) override;
    
    /**
     * @brief Start asynchronous flash format
     * 
     * @details Schedules format_handler() on task scheduler, erases and reinitializes 
     *          filesystem.
     * 
     * @return true if format initiated, false if already in progress
     * 
     * @warning Erases all data on flash chip
     * @note Non-blocking - monitor with get_format_status()
     */
    bool format(void) override;
    
    /**
     * @brief Query async format progress
     * 
     * @return FormatStatus: NOT_STARTED, IN_PROGRESS, SUCCESS, FAILED
     * 
     * @note Thread-safe status query
     */
    AP_Filesystem_Backend::FormatStatus get_format_status() const override;

    // LittleFS callbacks - Flash hardware interface (public for LittleFS)
    
    /**
     * @brief LittleFS read callback - read flash block
     * 
     * @details Called by LittleFS, executes SPI flash read command.
     * 
     * @param[in] block Block number
     * @param[in] off Offset within block (bytes)
     * @param[out] buffer Destination buffer
     * @param[in] size Bytes to read
     * 
     * @return 0 on success, negative on error
     * 
     * @note Must be fast - called frequently during filesystem operations
     * @note Read latency: 0.1-1ms depending on SPI speed
     */
    int _flashmem_read(lfs_block_t block, lfs_off_t off, void* buffer, lfs_size_t size);
    
    /**
     * @brief LittleFS program callback - write to flash
     * 
     * @details Executes SPI flash page program command.
     * 
     * @param[in] block Block number
     * @param[in] off Offset within block (bytes)
     * @param[in] buffer Source buffer
     * @param[in] size Bytes to program
     * 
     * @return 0 on success, negative on error
     * 
     * @warning Flash must be erased before programming
     * @note Write latency: 1-10ms (page program time)
     */
    int _flashmem_prog(lfs_block_t block, lfs_off_t off, const void* buffer, lfs_size_t size);
    
    /**
     * @brief LittleFS erase callback - erase flash block
     * 
     * @details Executes SPI flash block/sector erase command.
     * 
     * @param[in] block Block number
     * 
     * @return 0 on success, negative on error
     * 
     * @note Erase can take 10-200ms depending on flash chip
     * @warning Blocks scheduler during erase operation
     */
    int _flashmem_erase(lfs_block_t block);
    
    /**
     * @brief LittleFS sync callback - ensure writes committed
     * 
     * @return 0 (always succeeds)
     * 
     * @note No-op for NOR flash, waits for completion on NAND
     */
    int _flashmem_sync();

private:
    // Private members
    
    /**
     * @brief Semaphore protecting filesystem operations
     * @details All LittleFS calls must hold this semaphore (WITH_SEMAPHORE)
     */
    HAL_Semaphore fs_sem;

    /**
     * @brief LittleFS filesystem context structure
     * @details Opaque handle for mounted filesystem
     */
    lfs_t fs;

    /**
     * @brief LittleFS configuration (block size, counts, callbacks)
     * @details Populated by find_block_size_and_count() based on JEDEC detection
     */
    struct lfs_config fs_cfg;

    /**
     * @brief Maximum concurrent open files (16)
     * @details Limits memory usage for file descriptors
     */
    static constexpr int MAX_OPEN_FILES = 16;

    /**
     * @brief Filesystem mount status flag
     * @details true if lfs_mount() successful, false requires mount attempt
     */
    bool mounted;

    /**
     * @brief Filesystem marked as failed/corrupted
     * @details true if unrecoverable error detected, operations return -EIO
     */
    bool dead;

    /**
     * @brief Per-file state for open files
     * @details Contains:
     *          - file: LittleFS file handle
     *          - cfg: File configuration (attributes)
     *          - attrs[1]: Custom attribute array (mtime storage)
     *          - mtime: Cached modification time
     *          - filename: Dynamically allocated path string
     */
    struct FileDescriptor {
        lfs_file_t file;
        lfs_file_config cfg;
        lfs_attr attrs[1];
        uint32_t mtime;
        char* filename;
    };

    /**
     * @brief Array of open file descriptor pointers
     * @details Indexed by file descriptor, NULL if slot free
     */
    FileDescriptor* open_files[MAX_OPEN_FILES];

    /**
     * @brief SPI device handle for flash chip
     * @details Owns SPI bus access for flash operations
     */
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev;

    /**
     * @brief Semaphore for SPI device access
     * @details Prevents concurrent SPI transactions to flash
     */
    AP_HAL::Semaphore *dev_sem;

    /**
     * @brief Flash chip addressing mode flag
     * @details true for >16MB flash chips requiring 32-bit addresses
     */
    bool use_32bit_address;
    
    /**
     * @brief Current format operation status
     * @details Thread-safe atomic for async format tracking
     */
    FormatStatus format_status;

    // Private helper methods
    
    /**
     * @brief Find free file descriptor slot
     * @return File descriptor index (0-15) or -1 if table full
     */
    int allocate_fd();
    
    /**
     * @brief Release file descriptor slot
     * @param[in] fd File descriptor to free
     * @return 0 on success, -1 if invalid fd
     */
    int free_fd(int fd);
    
    /**
     * @brief Close all open files (used on unmount)
     * @note Forcibly closes all files without syncing
     */
    void free_all_fds();
    
    /**
     * @brief Convert fd to FileDescriptor pointer
     * @param[in] fd File descriptor
     * @return FileDescriptor* or NULL if invalid
     */
    FileDescriptor* lfs_file_from_fd(int fd) const;

    /**
     * @brief JEDEC flash detection and configuration
     * @details Reads JEDEC ID, determines chip capacity, sets block sizes
     * @return Total blocks detected
     */
    uint32_t find_block_size_and_count();
    
    /**
     * @brief Initialize SPI flash chip
     * @details Sends initialization commands, detects addressing mode
     * @return true on success
     * @note WARN_IF_UNUSED - caller must check return value
     */
    bool init_flash() WARN_IF_UNUSED;
    
    /**
     * @brief Enable flash write/erase operations
     * @details Sends WREN command, required before program/erase
     * @return true if enabled successfully
     * @note WARN_IF_UNUSED - caller must check return value
     */
    bool write_enable() WARN_IF_UNUSED;
    
    /**
     * @brief Check if flash chip is busy (write/erase in progress)
     * @details Reads status register
     * @return true if busy
     */
    bool is_busy();
    
    /**
     * @brief Attempt to mount LittleFS
     * @details Tries lfs_mount(), formats if necessary, retries on errors
     * @return true if mounted successfully
     */
    bool mount_filesystem();
    
    /**
     * @brief Send SPI command with address
     * @param[in] command JEDEC command byte
     * @param[in] addr 24 or 32-bit address
     * @note Handles address byte order and length
     */
    void send_command_addr(uint8_t command, uint32_t addr);
    
    /**
     * @brief Send SPI command with page number
     * @param[in] command JEDEC command
     * @param[in] page Page index
     * @note Helper for NAND flash page operations
     */
    void send_command_page(uint8_t command, uint32_t page);
    
    /**
     * @brief Poll flash status until ready
     * @details Waits for busy flag to clear
     * @return true if ready, false on timeout
     * @note WARN_IF_UNUSED - must check return value
     */
    bool wait_until_device_is_ready() WARN_IF_UNUSED;
    
    /**
     * @brief Modify flash status register bits
     * @param[in] reg Register number
     * @param[in] bits Bits to set
     * @note Used for write protection configuration
     */
    void write_status_register(uint8_t reg, uint8_t bits);
    
    /**
     * @brief Async format operation handler
     * @details Runs on scheduler, calls lfs_format(), updates status
     * @note Non-blocking background operation
     */
    void format_handler(void);
    
    /**
     * @brief Mark filesystem as failed
     * @details Sets dead flag, prevents further operations
     * @note Called on repeated fatal errors to prevent corruption
     */
    void mark_dead();
};

#endif  // #if AP_FILESYSTEM_LITTLEFS_ENABLED
