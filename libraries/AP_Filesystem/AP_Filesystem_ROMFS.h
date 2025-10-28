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
 * @file AP_Filesystem_ROMFS.h
 * @brief Read-only filesystem backend for firmware-embedded resources using AP_ROMFS
 * 
 * @details This filesystem backend provides access to files compiled directly into
 *          the firmware binary at build time. Resources are embedded using the
 *          ROMFS_EMBEDDED directive in board hwdef files and are accessible through
 *          standard filesystem operations (read-only).
 *          
 *          Common use cases include:
 *          - OSD fonts for on-screen display (AP_OSD)
 *          - Web UI assets (HTML, CSS, JavaScript) for AP_Networking
 *          - Default configuration files and parameter sets
 *          - Terrain elevation lookup tables
 *          - Calibration data and reference tables
 *          
 *          The embedded files consume code flash space but zero data RAM until
 *          explicitly loaded. Optional compression support allows efficient storage
 *          with decompression performed on-demand during file access.
 *          
 *          Build Integration:
 *          - ROMFS generation: Tools/scripts/apj_tool.py processes embedded resources
 *          - Detection flag: HAL_HAVE_AP_ROMFS_EMBEDDED_H indicates ROMFS availability
 *          - Board configuration: hwdef ROMFS_EMBEDDED directive specifies source files
 * 
 * Source: libraries/AP_Filesystem/AP_Filesystem_ROMFS.h
 * Source: libraries/AP_ROMFS/AP_ROMFS.h
 */

#pragma once

#include "AP_Filesystem_config.h"

#if AP_FILESYSTEM_ROMFS_ENABLED

#include <AP_HAL/Semaphores.h>

#include "AP_Filesystem_backend.h"

/**
 * @class AP_Filesystem_ROMFS
 * @brief Read-only filesystem backend for firmware-embedded resources
 * 
 * @details Provides POSIX-like filesystem interface to resources compiled into
 *          the firmware binary at build time. This backend wraps the AP_ROMFS
 *          system to expose embedded files through standard open/read/stat operations.
 *          
 *          Architecture:
 *          - Resources embedded at build time via ROMFS_EMBEDDED in hwdef files
 *          - Files stored in code flash section (zero data RAM cost when not loaded)
 *          - Optional compression with automatic decompression on access
 *          - Fixed pool of file handles (max_open_file = 4)
 *          - Fixed pool of directory handles (max_open_dir = 4)
 *          - Thread-safe access via HAL_Semaphore protection
 *          
 *          Resource Management:
 *          - Files remain in flash; small memory footprint (pointers into code section)
 *          - No dynamic allocation for file handles (uses fixed pool)
 *          - load_file() allocates heap memory for bulk reads (caller must free)
 *          - Instant "mount" with no initialization overhead
 *          
 *          Performance Characteristics:
 *          - Fast random access (direct code flash access)
 *          - Efficient for small, frequent reads
 *          - Optional compression reduces firmware size
 *          - Decompression overhead on first access to compressed files
 *          
 *          Limitations:
 *          - Strictly read-only (all write operations return -EROFS)
 *          - Limited concurrent open files (4 files + 4 directories)
 *          - Files cannot be modified at runtime (compile-time only)
 *          - File size limited by available code flash space
 * 
 * @note This filesystem is ideal for static resources that never change at runtime
 * @warning Limited open file slots - applications should close files promptly
 * @warning All write, unlink, and mkdir operations will fail with -EROFS
 * 
 * Source: libraries/AP_Filesystem/AP_Filesystem_ROMFS.h:26-79
 * Source: libraries/AP_Filesystem/AP_Filesystem_ROMFS.cpp
 */
class AP_Filesystem_ROMFS : public AP_Filesystem_Backend
{
public:
    /**
     * @brief Open an embedded ROMFS file for reading
     * 
     * @details Searches the compiled-in ROMFS directory for the named file and
     *          allocates a file handle from the fixed pool (max 4 concurrent files).
     *          Only read-only access is supported; write flags will cause failure.
     *          
     *          The file data remains in code flash and is not copied to RAM.
     *          Multiple opens of the same file are allowed (each gets separate handle).
     * 
     * @param[in] fname         Path to file within ROMFS (e.g., "@ROMFS/fonts/font.bin")
     * @param[in] flags         Open flags (O_RDONLY supported; O_WRONLY/O_RDWR return -EROFS)
     * @param[in] allow_absolute_paths Unused for ROMFS (embedded paths are always "absolute")
     * 
     * @return File descriptor (0-3) on success, -errno on failure
     * @retval -EROFS  Attempt to open for writing (O_WRONLY or O_RDWR)
     * @retval -ENOENT File not found in embedded ROMFS directory
     * @retval -EMFILE No free file handles available (max 4 concurrent opens)
     * 
     * @note This operation is very fast (directory lookup only, no I/O)
     * @warning Limited to 4 concurrent open files - close files when done
     * 
     * @see close()
     * @see AP_ROMFS::find_file()
     */
    int open(const char *fname, int flags, bool allow_absolute_paths = false) override;
    
    /**
     * @brief Close an open ROMFS file handle
     * 
     * @details Releases the file handle back to the fixed pool, making it available
     *          for subsequent open() calls. The underlying embedded file data remains
     *          in code flash unchanged.
     * 
     * @param[in] fd File descriptor returned from open() (0-3)
     * 
     * @return 0 on success, -errno on failure
     * @retval 0      Successfully closed file
     * @retval -EBADF Invalid file descriptor (not in range 0-3 or not open)
     * 
     * @note Always close files promptly due to limited handle pool
     * 
     * @see open()
     */
    int close(int fd) override;
    
    /**
     * @brief Read data from an open ROMFS file
     * 
     * @details Reads data from the embedded file starting at the current file offset.
     *          Data is read directly from code flash with no buffering. The file
     *          offset is automatically advanced by the number of bytes read.
     *          
     *          For compressed files, decompression is handled transparently by
     *          AP_ROMFS on the first access.
     * 
     * @param[in]  fd    File descriptor returned from open() (0-3)
     * @param[out] buf   Buffer to receive read data
     * @param[in]  count Maximum number of bytes to read
     * 
     * @return Number of bytes actually read (may be less than count at EOF), -errno on failure
     * @retval >=0    Number of bytes successfully read into buffer
     * @retval 0      End of file reached
     * @retval -EBADF Invalid file descriptor
     * 
     * @note Efficient for small reads; use load_file() for bulk loading entire file
     * @note Reading from code flash is fast (no SD card or SPI overhead)
     * 
     * @see lseek()
     * @see load_file()
     */
    int32_t read(int fd, void *buf, uint32_t count) override;
    
    /**
     * @brief Write operation - always fails (read-only filesystem)
     * 
     * @details ROMFS is strictly read-only. All write attempts return error.
     *          Files are embedded at compile time and cannot be modified at runtime.
     * 
     * @param[in] fd    File descriptor (ignored)
     * @param[in] buf   Data buffer (ignored)
     * @param[in] count Byte count (ignored)
     * 
     * @return Always -EROFS (read-only filesystem error)
     * 
     * @warning ROMFS files cannot be modified at runtime - embed changes at build time
     */
    int32_t write(int fd, const void *buf, uint32_t count) override;
    
    /**
     * @brief Sync operation - no-op for ROMFS (always in sync)
     * 
     * @details ROMFS files are in read-only code flash and never modified,
     *          so synchronization is not applicable. Always succeeds.
     * 
     * @param[in] fd File descriptor (ignored)
     * 
     * @return Always 0 (success)
     * 
     * @note No actual operation performed - embedded files are always consistent
     */
    int fsync(int fd) override;
    
    /**
     * @brief Seek to position within an open ROMFS file
     * 
     * @details Changes the current file offset for subsequent read() operations.
     *          Supports standard POSIX seek modes (SEEK_SET, SEEK_CUR, SEEK_END).
     *          Seeking beyond end of file is allowed but reads will return 0 bytes.
     * 
     * @param[in] fd     File descriptor returned from open() (0-3)
     * @param[in] offset Offset in bytes (interpretation depends on whence)
     * @param[in] whence Seek mode: SEEK_SET (absolute), SEEK_CUR (relative), SEEK_END (from end)
     * 
     * @return New absolute file offset in bytes, -errno on failure
     * @retval >=0    New file position from start of file
     * @retval -EBADF Invalid file descriptor
     * @retval -EINVAL Invalid whence parameter
     * 
     * @note Fast operation (pointer arithmetic only, no I/O)
     * @note Seeking to arbitrary positions is efficient for ROMFS (direct flash access)
     * 
     * @see read()
     */
    int32_t lseek(int fd, int32_t offset, int whence) override;
    
    /**
     * @brief Get file status information for an embedded ROMFS file
     * 
     * @details Returns file metadata from the ROMFS directory entries.
     *          For ROMFS, only the file size (st_size) and type (st_mode) fields
     *          are meaningful. Timestamps and permissions are not applicable.
     * 
     * @param[in]  pathname Path to file within ROMFS (e.g., "@ROMFS/fonts/font.bin")
     * @param[out] stbuf    Pointer to stat structure to fill with file information
     * 
     * @return 0 on success, -errno on failure
     * @retval 0      Successfully filled stat structure
     * @retval -ENOENT File not found in embedded ROMFS directory
     * 
     * @note st_size contains decompressed file size for compressed files
     * @note st_mode indicates regular file (S_IFREG) or directory (S_IFDIR)
     * @note Timestamps (st_mtime) are not meaningful for embedded files
     * 
     * @see AP_ROMFS::find_size()
     */
    int stat(const char *pathname, struct stat *stbuf) override;
    
    /**
     * @brief Delete operation - always fails (read-only filesystem)
     * 
     * @details ROMFS files are compiled into firmware and cannot be deleted
     *          at runtime. This operation always returns error.
     * 
     * @param[in] pathname File path (ignored)
     * 
     * @return Always -EROFS (read-only filesystem error)
     * 
     * @warning Files can only be added/removed by rebuilding firmware
     */
    int unlink(const char *pathname) override;
    
    /**
     * @brief Create directory operation - always fails (read-only filesystem)
     * 
     * @details ROMFS directory structure is defined at build time and cannot
     *          be modified at runtime. This operation always returns error.
     * 
     * @param[in] pathname Directory path (ignored)
     * 
     * @return Always -EROFS (read-only filesystem error)
     * 
     * @warning Directory structure is fixed at compile time via ROMFS_EMBEDDED
     */
    int mkdir(const char *pathname) override;
    
    /**
     * @brief Open a directory for reading entries
     * 
     * @details Opens an embedded ROMFS directory for enumeration via readdir().
     *          Allocates a directory handle from the fixed pool (max 4 concurrent).
     *          The directory structure is determined by files embedded at build time.
     * 
     * @param[in] pathname Path to directory within ROMFS (e.g., "@ROMFS/fonts/")
     * 
     * @return Opaque directory handle on success, nullptr on failure
     * @retval non-null Directory handle for use with readdir() and closedir()
     * @retval nullptr  Directory not found or no free directory handles (max 4)
     * 
     * @note Directory listings enumerate all files with matching pathname prefix
     * @warning Limited to 4 concurrent open directories - close when done
     * 
     * @see readdir()
     * @see closedir()
     * @see AP_ROMFS::dir_list()
     */
    void *opendir(const char *pathname) override;
    
    /**
     * @brief Read next directory entry
     * 
     * @details Returns the next file or subdirectory entry from an open directory.
     *          Call repeatedly until nullptr to enumerate all entries. Each call
     *          advances the directory position automatically.
     * 
     * @param[in] dirp Directory handle returned from opendir()
     * 
     * @return Pointer to dirent structure on success, nullptr when no more entries or error
     * @retval non-null Pointer to dirent with d_name containing entry name
     * @retval nullptr  End of directory or invalid directory handle
     * 
     * @note The returned dirent pointer is valid only until next readdir() or closedir()
     * @note Entry names are relative to the opened directory path
     * 
     * @see opendir()
     * @see closedir()
     */
    struct dirent *readdir(void *dirp) override;
    
    /**
     * @brief Close an open directory handle
     * 
     * @details Releases the directory handle back to the fixed pool, making it
     *          available for subsequent opendir() calls.
     * 
     * @param[in] dirp Directory handle returned from opendir()
     * 
     * @return 0 on success, -errno on failure
     * @retval 0      Successfully closed directory
     * @retval -EBADF Invalid directory handle
     * 
     * @note Always close directories promptly due to limited handle pool (4 max)
     * 
     * @see opendir()
     */
    int closedir(void *dirp) override;

    /**
     * @brief Get free space - not applicable for ROMFS
     * 
     * @details ROMFS uses code flash space which is fixed at compile time.
     *          Free space concept is not meaningful for read-only embedded filesystem.
     * 
     * @param[in] path Path (ignored)
     * 
     * @return Always -1 (not applicable)
     * 
     * @note ROMFS size is fixed at firmware build time
     */
    int64_t disk_free(const char *path) override;

    /**
     * @brief Get total space - not applicable for ROMFS
     * 
     * @details ROMFS uses code flash space which is fixed at compile time.
     *          Total space depends on firmware image size, not queryable at runtime.
     * 
     * @param[in] path Path (ignored)
     * 
     * @return Always -1 (not applicable)
     * 
     * @note ROMFS size is determined by embedded files at firmware build
     */
    int64_t disk_space(const char *path) override;

    /**
     * @brief Set modification time - not supported (read-only filesystem)
     * 
     * @details ROMFS files are in read-only code flash with no concept of
     *          modification time. Files are embedded at build time.
     * 
     * @param[in] filename  File path (ignored)
     * @param[in] mtime_sec Modification time (ignored)
     * 
     * @return Always false (operation not supported)
     * 
     * @warning Cannot modify metadata for embedded files
     */
    bool set_mtime(const char *filename, const uint32_t mtime_sec) override;

    /**
     * @brief Load entire embedded file into heap-allocated buffer
     * 
     * @details Efficiently loads a complete ROMFS file into a dynamically allocated
     *          buffer in one operation. For compressed files, performs automatic
     *          decompression. This is more efficient than repeated read() calls
     *          for bulk file loading.
     *          
     *          The returned FileData object contains:
     *          - data: Pointer to file contents (null-terminated)
     *          - length: Decompressed file size in bytes
     *          
     *          Caller must call unload_file() or delete the FileData object to
     *          free the allocated memory.
     * 
     * @param[in] filename Path to file within ROMFS (e.g., "@ROMFS/fonts/font.bin")
     * 
     * @return Pointer to FileData object on success, nullptr on failure
     * @retval non-null FileData with heap-allocated buffer containing file contents
     * @retval nullptr  File not found or memory allocation failed
     * 
     * @note Allocates heap memory - caller MUST call unload_file() to free
     * @note For compressed files, returns decompressed data
     * @note Data is guaranteed null-terminated for text file convenience
     * @warning Memory allocation can fail on low-memory systems - check for nullptr
     * @warning Caller responsible for freeing memory via unload_file() or delete
     * 
     * @see unload_file()
     * @see AP_ROMFS::find_decompress()
     */
    FileData *load_file(const char *filename) override;

    /**
     * @brief Free memory allocated by load_file()
     * 
     * @details Releases the heap-allocated buffer created by load_file().
     *          After this call, the FileData pointer and its contents are invalid.
     * 
     * @param[in] fd FileData pointer returned from load_file()
     * 
     * @note Safe to call with nullptr (no operation performed)
     * @note Equivalent to `delete fd` for FileData objects
     * 
     * @see load_file()
     * @see AP_ROMFS::free()
     */
    void unload_file(FileData *fd) override;
    
private:
    /**
     * @brief Semaphore protecting file and directory handle allocation
     * 
     * @details Ensures thread-safe access to file[] and dir[] arrays when searching
     *          for free handles during open()/opendir() and releasing handles during
     *          close()/closedir(). Prevents race conditions in multi-threaded environments.
     */
    HAL_Semaphore record_sem;

    /**
     * @brief Maximum number of concurrent open files
     * 
     * @details Fixed pool size for file handles. Limits concurrent open() calls to 4.
     *          This conservative limit reduces memory overhead while supporting typical
     *          usage patterns (e.g., loading OSD fonts, web UI resources).
     */
    static constexpr uint8_t max_open_file = 4;
    
    /**
     * @brief Maximum number of concurrent open directories
     * 
     * @details Fixed pool size for directory handles. Limits concurrent opendir() calls to 4.
     *          Sufficient for directory tree traversal and enumeration operations.
     */
    static constexpr uint8_t max_open_dir = 4;
    
    /**
     * @brief File handle structure for open ROMFS files
     * 
     * @details Tracks state for each open file:
     *          - data: Pointer into code flash where file contents begin
     *          - size: Total file size in bytes (decompressed size for compressed files)
     *          - ofs:  Current read offset within file (updated by read() and lseek())
     *          
     *          Handles are allocated from fixed pool on open(), released on close().
     *          A null data pointer indicates the slot is free.
     */
    struct rfile {
        const uint8_t *data;  ///< Pointer to file data in code flash (nullptr = free slot)
        uint32_t size;        ///< Total file size in bytes
        uint32_t ofs;         ///< Current file offset for read operations
    } file[max_open_file];

    /**
     * @brief Directory handle structure for open ROMFS directories
     * 
     * @details Tracks state for each open directory:
     *          - path: Directory path prefix for filtering entries (heap-allocated)
     *          - ofs:  Current position in directory enumeration sequence
     *          - de:   dirent structure returned to caller (reused for each readdir())
     *          
     *          Handles are allocated from fixed pool on opendir(), released on closedir().
     *          A null path pointer indicates the slot is free.
     */
    struct rdir {
        char *path;           ///< Directory path prefix (nullptr = free slot)
        uint16_t ofs;         ///< Current enumeration offset for readdir()
        struct dirent de;     ///< Reusable dirent structure for returning entries
    } dir[max_open_dir];
};

#endif  // AP_FILESYSTEM_ROMFS_ENABLED
