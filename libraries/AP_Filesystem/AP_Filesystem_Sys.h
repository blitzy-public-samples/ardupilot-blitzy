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
 * @file AP_Filesystem_Sys.h
 * @brief @SYS virtual filesystem exposing system diagnostics and internal state
 * 
 * This file implements the @SYS virtual filesystem backend that provides
 * read-only access to ArduPilot system diagnostics, crash dumps, and internal
 * state through a file-based interface. All files are generated on-demand at
 * open() time and stored in memory as snapshots.
 * 
 * The @SYS filesystem is accessible via MAVLink FTP protocol, enabling remote
 * diagnostics and debugging without requiring shell access to the flight controller.
 * 
 * @note All @SYS files are read-only. Write operations are not supported.
 * @warning Reading large files (flash.bin) generates significant memory allocation
 * 
 * @see libraries/AP_Filesystem/README.md for MAVLink FTP protocol details
 * 
 * Source: libraries/AP_Filesystem/AP_Filesystem_Sys.h
 */

#pragma once

#include "AP_Filesystem_backend.h"

#include "AP_Filesystem_config.h"

#if AP_FILESYSTEM_SYS_ENABLED

class ExpandingString;

/**
 * @class AP_Filesystem_Sys
 * @brief Virtual filesystem backend providing system diagnostics as readable files
 * 
 * @details This backend implements the @SYS virtual filesystem, exposing ArduPilot
 *          internal state and diagnostics through standard filesystem operations.
 *          Content is generated dynamically at open() time and stored in memory
 *          as a snapshot for subsequent read() operations.
 * 
 * **Available Virtual Files:**
 * 
 * - **@SYS/threads.txt**: Thread stack usage and health information
 *   - Thread name, total stack size, free stack, used percentage
 *   - Platform: ChibiOS only (RTOS with thread tracking)
 *   - Use case: Identify stack overflow risks, tune thread stack sizes
 * 
 * - **@SYS/tasks.txt**: AP_Scheduler task information and timing statistics
 *   - Task names, execution rates, timing overruns, budget usage
 *   - Platform: All platforms
 *   - Use case: Performance profiling, scheduler tuning
 * 
 * - **@SYS/dma.txt**: DMA controller allocation and usage information
 *   - Platform: ChibiOS on STM32
 *   - Use case: DMA conflict debugging, resource optimization
 * 
 * - **@SYS/memory.txt**: Memory allocation statistics and heap status
 *   - Platform: All platforms
 *   - Use case: Memory leak detection, heap fragmentation analysis
 * 
 * - **@SYS/uarts.txt**: UART/serial port configuration and status
 *   - Platform: All platforms
 *   - Use case: Serial port debugging, baud rate verification
 * 
 * - **@SYS/timers.txt**: Hardware timer allocation and configuration
 *   - Platform: ChibiOS on STM32
 *   - Use case: PWM conflict debugging, timer resource planning
 * 
 * - **@SYS/can_log.txt**: CAN bus protocol driver logs (if HAL_MAX_CAN_PROTOCOL_DRIVERS)
 *   - Platform: Boards with CAN support
 *   - Use case: DroneCAN/UAVCAN debugging
 * 
 * - **@SYS/can0_stats.txt, @SYS/can1_stats.txt**: CAN interface statistics
 *   - Platform: Boards with CAN hardware (HAL_NUM_CAN_IFACES > 0)
 *   - Use case: CAN bus health monitoring, error rate analysis
 * 
 * - **@SYS/persistent.parm**: Persistent parameters from crash-safe storage
 *   - Platform: STM32F7/H7 non-bootloader builds
 *   - Use case: Recover parameters after crash or corruption
 * 
 * - **@SYS/crash_dump.bin**: Binary crash dump from last fault
 *   - Crash context: registers, backtrace, fault address, exception type
 *   - Platform: Platforms with crash detection support
 *   - Use case: Post-crash analysis, fault diagnosis
 * 
 * - **@SYS/storage.bin**: Raw byte dump of parameter storage area
 *   - Complete parameter EEPROM/flash contents
 *   - Platform: All platforms
 *   - Use case: Parameter corruption analysis, storage debugging
 * 
 * - **@SYS/flash.bin**: Complete flash memory dump (if AP_FILESYSTEM_SYS_FLASH_ENABLED)
 *   - WARNING: Very large file (multiple megabytes)
 *   - Platform: ChibiOS only
 *   - Use case: Firmware verification, low-level debugging
 * 
 * **Key Characteristics:**
 * - Read-only filesystem (write/create operations return EROFS error)
 * - Content generated at open() time and cached in ExpandingString buffer
 * - Maximum 4 simultaneously open files (max_open_file limit)
 * - File content is snapshot - not updated during read operations
 * - Accessible via MAVLink FTP protocol for remote diagnostics
 * 
 * **Memory Considerations:**
 * - open() allocates ExpandingString buffer for entire file content
 * - Large files (flash.bin, storage.bin) cause memory spikes
 * - close() releases buffer to prevent memory leaks
 * - Multiple open files consume memory proportional to content size
 * 
 * **Thread Safety:**
 * - File operations are not thread-safe
 * - Should be accessed from GCS MAVLink thread context
 * - No locking on file[] array access
 * 
 * @note File list can be enumerated using opendir()/readdir() operations
 * @warning Reading flash.bin is slow and memory-intensive (multi-MB file)
 * @warning open() can cause memory allocation failures on systems with limited RAM
 * 
 * @see AP_Filesystem_Backend for base filesystem interface
 * @see ExpandingString for dynamic string buffer implementation
 * @see libraries/AP_Filesystem/README.md for complete @SYS documentation
 * 
 * Source: libraries/AP_Filesystem/AP_Filesystem_Sys.h, AP_Filesystem_Sys.cpp
 */
class AP_Filesystem_Sys : public AP_Filesystem_Backend
{
public:
    /**
     * @brief Open a virtual @SYS file and generate its diagnostic content
     * 
     * @details Opens a virtual diagnostic file from the @SYS filesystem. The file
     *          content is generated immediately at open time by calling the appropriate
     *          diagnostic function and storing the output in an ExpandingString buffer.
     *          The generated content is a snapshot and remains static for the lifetime
     *          of the file handle.
     * 
     *          This operation allocates memory proportional to the file size. For large
     *          files like flash.bin (multi-MB), this can cause significant memory pressure.
     * 
     *          Only read-only access is supported (O_RDONLY flag). Write attempts will
     *          fail with EROFS error.
     * 
     * @param[in] fname File path within @SYS filesystem (e.g., "threads.txt", "crash_dump.bin")
     * @param[in] flags File open flags (only O_RDONLY supported, O_WRONLY/O_RDWR return EROFS)
     * @param[in] allow_absolute_paths Ignored for @SYS filesystem (not used)
     * 
     * @return File descriptor (0-3) on success, -1 on error with errno set
     * 
     * @retval 0-3 Valid file descriptor for opened file
     * @retval -1 Error occurred:
     *         - errno = EROFS: Write access requested (only read allowed)
     *         - errno = ENFILE: All file slots in use (max_open_file=4 limit reached)
     *         - errno = ENOMEM: Failed to allocate ExpandingString buffer
     *         - errno = ENOENT: File not found in @SYS filesystem
     * 
     * @note Maximum 4 files can be open simultaneously (max_open_file limit)
     * @note Content is generated at open time, not lazily during read
     * @note Large files (flash.bin) may take several seconds to generate
     * 
     * @warning Memory spike on open - ensure sufficient free heap before calling
     * @warning flash.bin generation is very slow (multi-megabyte read from flash memory)
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Sys.cpp:69-180
     */
    int open(const char *fname, int flags, bool allow_absolute_paths = false) override;
    
    /**
     * @brief Close virtual file and release memory buffer
     * 
     * @details Closes a previously opened @SYS file and deallocates the ExpandingString
     *          buffer holding the generated diagnostic content. This releases memory
     *          allocated during open().
     * 
     * @param[in] fd File descriptor returned by open() (0-3)
     * 
     * @return 0 on success, -1 on error with errno set
     * 
     * @retval 0 File closed successfully
     * @retval -1 Invalid file descriptor (errno = EBADF)
     * 
     * @note Always call close() to prevent memory leaks from ExpandingString buffers
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Sys.cpp:182-195
     */
    int close(int fd) override;
    
    /**
     * @brief Read data from virtual file's pre-generated content buffer
     * 
     * @details Reads data from the ExpandingString buffer populated during open().
     *          Content is static (snapshot from open time) and does not update during
     *          sequential reads. Supports partial reads and respects current file offset.
     * 
     * @param[in] fd File descriptor (0-3)
     * @param[out] buf Buffer to receive read data
     * @param[in] count Maximum number of bytes to read
     * 
     * @return Number of bytes read (0-count), or -1 on error with errno set
     * 
     * @retval >0 Number of bytes successfully read into buf
     * @retval 0 End of file reached (file_ofs >= content length)
     * @retval -1 Invalid file descriptor (errno = EBADF)
     * 
     * @note Advances internal file offset (file_ofs) by bytes read
     * @note Subsequent reads continue from current offset
     * @note Reading past EOF returns 0 bytes (standard POSIX behavior)
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Sys.cpp:197-218
     */
    int32_t read(int fd, void *buf, uint32_t count) override;
    
    /**
     * @brief Seek to position within virtual file content
     * 
     * @details Changes the current read offset within the pre-generated file content.
     *          Supports standard POSIX seek modes (SEEK_SET, SEEK_CUR, SEEK_END).
     * 
     * @param[in] fd File descriptor (0-3)
     * @param[in] offset Byte offset for seek operation
     * @param[in] whence Seek mode:
     *                   - SEEK_SET: Absolute offset from beginning
     *                   - SEEK_CUR: Relative offset from current position
     *                   - SEEK_END: Offset relative to end of file
     * 
     * @return New file offset from beginning, or -1 on error with errno set
     * 
     * @retval >=0 New absolute file offset in bytes
     * @retval -1 Error:
     *         - errno = EBADF: Invalid file descriptor
     *         - errno = EINVAL: Invalid whence value or resulting offset out of bounds
     * 
     * @note Does not regenerate content - only changes read position in existing buffer
     * @note Seeking past EOF is allowed but read() will return 0 bytes
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Sys.cpp:220-240
     */
    int32_t lseek(int fd, int32_t offset, int whence) override;
    
    /**
     * @brief Get file status information for virtual @SYS file
     * 
     * @details Returns metadata for a virtual @SYS file. Generates the file content
     *          internally to determine size, then discards it. This is necessary
     *          because file sizes are dynamic (threads.txt size depends on number
     *          of threads, etc.).
     * 
     * @param[in] pathname @SYS file path (e.g., "threads.txt", "crash_dump.bin")
     * @param[out] stbuf Stat structure to populate with file information
     * 
     * @return 0 on success, -1 on error with errno set
     * 
     * @retval 0 Success, stbuf populated with:
     *         - st_size: File size in bytes
     *         - st_mode: S_IFREG | 0444 (regular file, read-only)
     * @retval -1 Error:
     *         - errno = ENOENT: File not found in @SYS filesystem
     *         - errno = ENOMEM: Failed to generate content for size calculation
     * 
     * @note Temporarily generates entire file content to determine size
     * @note May be slow for large files (flash.bin)
     * @note Size reflects content at stat() call time, may differ at open() time
     * 
     * @warning Can cause memory pressure due to temporary content generation
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Sys.cpp:242-268
     */
    int stat(const char *pathname, struct stat *stbuf) override;
    
    /**
     * @brief Open directory for enumerating @SYS virtual files
     * 
     * @details Opens the @SYS directory to enable listing all available virtual files
     *          using readdir(). Currently only the root @SYS directory is supported.
     * 
     * @param[in] pathname Directory path (typically "" or "/" for @SYS root)
     * 
     * @return Directory handle pointer on success, nullptr on error with errno set
     * 
     * @retval non-null Opaque directory handle for readdir() operations
     * @retval nullptr Error:
     *         - errno = ENOMEM: Failed to allocate DirReadTracker structure
     * 
     * @note Directory listing shows all files defined in sysfs_file_list[] array
     * @note Platform-specific files (threads.txt, flash.bin) appear even if unavailable
     * @note Must call closedir() to free allocated DirReadTracker
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Sys.cpp:270-282
     */
    void *opendir(const char *pathname) override;
    
    /**
     * @brief Read next directory entry from @SYS virtual filesystem
     * 
     * @details Returns the next file entry from the @SYS directory listing.
     *          Iterates through the sysfs_file_list[] array.
     * 
     * @param[in] dirp Directory handle from opendir()
     * 
     * @return Pointer to dirent structure, or nullptr when no more entries
     * 
     * @retval non-null Pointer to static dirent structure in DirReadTracker with:
     *         - d_name: Virtual file name (e.g., "threads.txt")
     *         - d_type: DT_REG (regular file)
     * @retval nullptr End of directory reached, or invalid dirp
     * 
     * @note Returned pointer valid until next readdir() call or closedir()
     * @note Directory entries are not sorted
     * @note Platform-conditional files always listed (may not be accessible on all boards)
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Sys.cpp:284-302
     */
    struct dirent *readdir(void *dirp) override;
    
    /**
     * @brief Close directory handle and free resources
     * 
     * @details Closes a directory opened with opendir() and frees the DirReadTracker.
     * 
     * @param[in] dirp Directory handle from opendir()
     * 
     * @return 0 on success, -1 on error with errno set
     * 
     * @retval 0 Directory closed successfully
     * @retval -1 Invalid directory handle (errno = EBADF)
     * 
     * @note Always call closedir() to prevent memory leak of DirReadTracker
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Sys.cpp:304-315
     */
    int closedir(void *dirp) override;

private:
    /**
     * @brief Maximum number of simultaneously open @SYS files
     * 
     * @details Limits concurrent file opens to conserve memory and file descriptor slots.
     *          Each open file maintains an ExpandingString buffer with full file content,
     *          so this limit prevents excessive memory consumption.
     */
    static constexpr uint8_t max_open_file = 4;
    
    /**
     * @brief Check if filename exists in @SYS virtual filesystem
     * 
     * @details Searches the sysfs_file_list[] array for the specified filename.
     *          Used during open() to validate file existence before generating content.
     * 
     * @param[in] fname Filename to search for (without @SYS/ prefix)
     * 
     * @return Array index (0-based) if found, -1 if not found
     * 
     * @retval >=0 Index in sysfs_file_list[] array
     * @retval -1 File not in @SYS filesystem
     * 
     * @note Case-sensitive string comparison
     * @note Platform-conditional files (flash.bin) are in list even if not available
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Sys.cpp:60-67
     */
    int8_t file_in_sysfs(const char *fname);

    /**
     * @struct DirReadTracker
     * @brief Directory iteration state for readdir() operations
     * 
     * @details Tracks position within sysfs_file_list[] array during directory
     *          enumeration. Allocated by opendir() and freed by closedir().
     */
    struct DirReadTracker {
        size_t file_offset;       ///< Current index in sysfs_file_list[] array
        struct dirent curr_file;  ///< Static dirent buffer returned by readdir()
    };

    /**
     * @struct rfile
     * @brief Open file state for @SYS virtual file handle
     * 
     * @details Maintains state for each open @SYS file. The ExpandingString buffer
     *          holds the complete generated file content as a snapshot from open() time.
     */
    struct rfile {
        bool open;              ///< True if this slot contains an open file
        uint32_t file_ofs;      ///< Current read position within content buffer (bytes)
        ExpandingString *str;   ///< Dynamically allocated buffer containing generated file content
    } file[max_open_file];      ///< Array of open file slots (max 4 concurrent opens)
};

#endif  // AP_FILESYSTEM_SYS_ENABLED
