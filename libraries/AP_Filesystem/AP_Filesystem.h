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
 * @file AP_Filesystem.h
 * @brief Main filesystem router and unified interface for ArduPilot
 * 
 * @details Provides single AP::FS() entry point that multiplexes across multiple 
 *          backend implementations (FATFS, LittleFS, POSIX, ROMFS, virtual backends). 
 *          Routes operations based on path prefixes (@PARAM/, @SYS/, @MISSION/, @ROMFS/) 
 *          or file descriptor encoding. Normalizes paths, encodes backend indices into 
 *          file descriptors, and forwards POSIX-like operations to selected backends.
 * 
 *          Architecture: Singleton accessible via AP::FS(), backend registration array, 
 *          path-based routing
 * 
 *          Path Routing Examples:
 *          - "@PARAM/param.pck" → AP_Filesystem_Param backend
 *          - "@SYS/threads.txt" → AP_Filesystem_Sys backend
 *          - "@MISSION/mission.dat" → AP_Filesystem_Mission backend
 *          - "@ROMFS/font.bin" → AP_Filesystem_ROMFS backend
 *          - "/SDCARD/log.bin" → FATFS or POSIX backend (platform-specific)
 *          - "params.txt" → Default backend (FATFS/POSIX/LittleFS based on platform)
 * 
 *          File Descriptor Encoding Scheme:
 *          System-wide file descriptors encode backend index in the fd value:
 *          - Format: fd = backend_index * MAX_FD_PER_BACKEND + backend_local_fd
 *          - MAX_FD_PER_BACKEND typically 1000 (allows 1000 files per backend)
 *          - Decoding: backend_index = fd / MAX_FD_PER_BACKEND, local_fd = fd % MAX_FD_PER_BACKEND
 * 
 * @note Thread-safe (backends handle their own synchronization)
 * @note File descriptors are unique system-wide (encoded with backend index)
 * @warning File descriptors from one backend cannot be used on another
 * 
 * Source: libraries/AP_Filesystem/AP_Filesystem.h, AP_Filesystem.cpp (routing implementation)
 */
#pragma once

#include <stdint.h>
#include <AP_HAL/AP_HAL_Boards.h>

#include "AP_Filesystem_config.h"

/**
 * @brief Maximum filename length in bytes
 * @details POSIX PATH_MAX compatibility, used for dirent.d_name buffer sizing
 */
#ifndef MAX_NAME_LEN
#define MAX_NAME_LEN 255
#endif

#if (CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS) || (CONFIG_HAL_BOARD == HAL_BOARD_ESP32)
/**
 * @brief Directory entry type constant for regular files
 * @details Used in dirent.d_type field to indicate file type, compatible with POSIX DT_REG
 */
#define DT_REG 0

/**
 * @brief Directory entry type constant for directories
 * @details Used in dirent.d_type field to indicate directory type, compatible with POSIX DT_DIR
 */
#define DT_DIR 1

/**
 * @brief Directory entry type constant for symbolic links
 * @details Used in dirent.d_type field to indicate symlink type, compatible with POSIX DT_LNK
 */
#define DT_LNK 10
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#if AP_FILESYSTEM_FATFS_ENABLED
#include "AP_Filesystem_FATFS.h"
#endif
#if AP_FILESYSTEM_LITTLEFS_ENABLED
#include "AP_Filesystem_FlashMemory_LittleFS.h"
#endif

/**
 * @brief ChibiOS directory entry structure for readdir()
 * @details Provides directory entry information for ChibiOS platforms (POSIX systems use system dirent)
 */
struct dirent {
   char    d_name[MAX_NAME_LEN]; ///< @brief Filename (null-terminated)
   uint8_t d_type;                ///< @brief Entry type (DT_REG, DT_DIR, DT_LNK)
};

#endif // HAL_BOARD_CHIBIOS

#include <fcntl.h>
#include <errno.h>
#include <unistd.h>

/**
 * @brief Compile-time flag to enable/disable filesystem format capability
 * @details When enabled, format() and get_format_status() methods are functional.
 *          Disable to reduce code size on platforms where formatting is not needed.
 */
#ifndef AP_FILESYSTEM_FORMAT_ENABLED
#define AP_FILESYSTEM_FORMAT_ENABLED 1
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX || CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_QURT
#include "AP_Filesystem_posix.h"
#if AP_FILESYSTEM_LITTLEFS_ENABLED
#include "AP_Filesystem_FlashMemory_LittleFS.h"
#endif
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
#include "AP_Filesystem_ESP32.h"
#endif

#include "AP_Filesystem_backend.h"

/**
 * @class AP_Filesystem
 * @brief Unified filesystem router multiplexing multiple backend implementations
 * 
 * @details This singleton class provides the primary filesystem interface for all ArduPilot code. It:
 *          - Routes operations to appropriate backends based on path prefixes or file descriptor encoding
 *          - Normalizes and strips path prefixes before passing to backends
 *          - Encodes backend indices into file descriptors (per-backend FD ranges)
 *          - Provides POSIX-like interface familiar to developers
 *          - Supports virtual filesystems (@PARAM, @SYS, @MISSION) alongside physical storage
 * 
 *          Access: Via AP::FS() singleton getter in AP namespace
 * 
 *          Thread Safety:
 *          - AP_Filesystem itself is stateless (except virtual_dirent)
 *          - Thread safety provided by backend implementations
 *          - File descriptors should not be shared across threads
 *          - Multiple threads can safely call AP::FS() methods
 * 
 *          Error Handling:
 *          - Returns -1 or nullptr on errors (POSIX convention)
 *          - errno set by backends for detailed error information
 *          - Backend selection never fails (falls back to default)
 *          - Invalid file descriptors return -1
 * 
 *          Performance Considerations:
 *          - Backend selection is fast (linear search, typically <10 backends)
 *          - No buffering at router level (backends handle buffering)
 *          - Virtual backend overhead: content generation on open()
 *          - Physical backend overhead: hardware I/O timing (ms-scale for SD/flash)
 * 
 * @note File descriptors are unique system-wide (encoded with backend index)
 * @warning File descriptors from one backend cannot be used on another
 */
class AP_Filesystem {
private:
    /**
     * @brief Opaque directory handle with backend routing information
     * @details Internal structure passed to readdir() and closedir() to route operations
     *          to the correct backend. Created by opendir(), destroyed by closedir().
     */
    struct DirHandle {
        uint8_t fs_index; ///< @brief Backend index for routing readdir/closedir operations
        void *dir;        ///< @brief Backend-specific directory handle (void* to backend's DIR/directory object)
    };

public:
    AP_Filesystem() {}

    // Core File Operations

    /**
     * @brief Open file on any backend, selecting backend by path prefix
     * 
     * @param[in] fname         File path (with optional prefix like "@PARAM/", "@SYS/", etc.)
     * @param[in] flags         POSIX open flags (O_RDONLY, O_WRONLY, O_RDWR, O_CREAT, O_TRUNC, O_APPEND)
     * @param[in] allow_absolute_paths  Skip prefix stripping for absolute paths (default false)
     * 
     * @return System-wide file descriptor (encoded with backend index) on success, -1 on error
     * 
     * @details Selects backend via backend_by_path(), normalizes path, encodes backend 
     *          index into returned fd (fd_offset + backend_fd). Backend selection based on 
     *          path prefix matching (e.g., "@PARAM/" routes to param backend).
     * 
     * @note Prefix examples: "@PARAM/param.pck", "@SYS/threads.txt", "/SDCARD/log.bin"
     * @note File descriptor encoding: fd = backend_index * MAX_FD_PER_BACKEND + backend_local_fd
     */
    int open(const char *fname, int flags, bool allow_absolute_paths = false);

    /**
     * @brief Close file descriptor
     * 
     * @param[in] fd  System-wide file descriptor from open()
     * 
     * @return 0 on success, -1 on error
     * 
     * @details Decodes backend from fd, routes to backend->close()
     */
    int close(int fd);

    /**
     * @brief Read from file
     * 
     * @param[in]  fd     File descriptor from open()
     * @param[out] buf    Destination buffer for read data
     * @param[in]  count  Maximum number of bytes to read
     * 
     * @return Number of bytes read, 0 on EOF, -1 on error
     * 
     * @details Routes to backend->read() based on fd encoding
     */
    int32_t read(int fd, void *buf, uint32_t count);

    /**
     * @brief Write to file
     * 
     * @param[in] fd     File descriptor from open()
     * @param[in] buf    Source buffer containing data to write
     * @param[in] count  Number of bytes to write
     * 
     * @return Number of bytes written, -1 on error
     * 
     * @details Routes to backend->write(), may return EROFS for read-only backends
     */
    int32_t write(int fd, const void *buf, uint32_t count);

    /**
     * @brief Synchronize file to storage (flush pending writes)
     * 
     * @param[in] fd  File descriptor from open()
     * 
     * @return 0 on success, -1 on error
     * 
     * @details Routes to backend->fsync(), critical for data integrity after writes
     * 
     * @note Essential for ensuring log data is persisted to SD card/flash
     */
    int fsync(int fd);

    /**
     * @brief Seek to position in file
     * 
     * @param[in] fd      File descriptor from open()
     * @param[in] offset  Byte offset for new position
     * @param[in] whence  Reference point (SEEK_SET=beginning, SEEK_CUR=current, SEEK_END=end)
     * 
     * @return New file position in bytes from beginning, or -1 on error
     * 
     * @details Routes to backend->lseek()
     */
    int32_t lseek(int fd, int32_t offset, int whence);

    /**
     * @brief Get file information (POSIX stat structure)
     * 
     * @param[in]  pathname  File path (with optional prefix)
     * @param[out] stbuf     Output stat structure (st_size, st_mode, st_mtime, etc.)
     * 
     * @return 0 on success, -1 on error
     * 
     * @details Selects backend by path, calls backend->stat()
     */
    int stat(const char *pathname, struct stat *stbuf);

    /**
     * @brief Simplified stat structure for scripting (Lua-friendly)
     * @details Provides essential file information in a format easily accessible from scripts
     */
    typedef struct Stat {
        uint32_t size;  ///< @brief File size in bytes
        int32_t mode;   ///< @brief File mode/permissions (POSIX st_mode format with S_IFMT type bits)
        uint32_t mtime; ///< @brief Modification time (Unix timestamp seconds)
        uint32_t atime; ///< @brief Access time (Unix timestamp seconds)
        uint32_t ctime; ///< @brief Creation/change time (Unix timestamp seconds)
        
        /**
         * @brief Check if entry is a directory
         * @return true if S_IFDIR mode bit is set, false otherwise
         */
        bool is_directory(void) const {
            return (mode & S_IFMT) == S_IFDIR;
        }
    } stat_t;

    /**
     * @brief Get file information (scripting-friendly variant)
     * 
     * @param[in]  pathname  File path (with optional prefix)
     * @param[out] stbuf     Output simplified stat structure
     * 
     * @return true on success, false on error
     * 
     * @details Wrapper for Lua scripting, converts struct stat to stat_t
     */
    bool stat(const char *pathname, stat_t &stbuf);

    // File System Modification Operations

    /**
     * @brief Delete file
     * 
     * @param[in] pathname  File path (with optional prefix)
     * 
     * @return 0 on success, -1 on error
     * 
     * @details Routes to backend->unlink()
     */
    int unlink(const char *pathname);

    /**
     * @brief Create directory
     * 
     * @param[in] pathname  Directory path (with optional prefix)
     * 
     * @return 0 on success, -1 on error (including EEXIST if already exists)
     * 
     * @details Routes to backend->mkdir()
     */
    int mkdir(const char *pathname);

    /**
     * @brief Rename or move file
     * 
     * @param[in] oldpath  Source file path
     * @param[in] newpath  Destination file path
     * 
     * @return 0 on success, -1 on error
     * 
     * @details Routes to backend->rename(), both paths must be on same backend
     * 
     * @warning Fails if oldpath and newpath are on different backends
     */
    int rename(const char *oldpath, const char *newpath);

    // Directory Operations

    /**
     * @brief Open directory for reading entries
     * 
     * @param[in] pathname  Directory path (with optional prefix)
     * 
     * @return DirHandle pointer on success, nullptr on error
     * 
     * @details Selects backend, creates DirHandle with backend index and backend DIR*.
     *          Caller must call closedir() to free the handle.
     * 
     * @note Returned handle must be freed with closedir()
     */
    DirHandle *opendir(const char *pathname);

    /**
     * @brief Read next directory entry
     * 
     * @param[in] dirp  DirHandle from opendir()
     * 
     * @return Pointer to dirent structure, or nullptr at end of directory
     * 
     * @details Uses dirp->fs_index to route to backend->readdir()
     * 
     * @note Returns pointer to internal buffer (not thread-safe across calls)
     * @note Call repeatedly until nullptr to enumerate all entries
     */
    struct dirent *readdir(DirHandle *dirp);

    /**
     * @brief Close directory handle
     * 
     * @param[in] dirp  DirHandle from opendir()
     * 
     * @return 0 on success, -1 on error
     * 
     * @details Routes to backend->closedir(), frees DirHandle memory
     */
    int closedir(DirHandle *dirp);

    // Storage Management

    /**
     * @brief Advisory fsync threshold for optimal write batching
     * 
     * @param[in] fd  File descriptor from open()
     * 
     * @return Recommended number of bytes to write before calling fsync(), or 0 if any number is acceptable
     * 
     * @details Routes to backend->bytes_until_fsync(), used by AP_Logger to batch writes
     *          for performance while maintaining data integrity
     */
    uint32_t bytes_until_fsync(int fd);

    /**
     * @brief Get free storage space
     * 
     * @param[in] path  Path on target backend (determines which backend to query)
     * 
     * @return Free space in bytes, or -1 on error
     * 
     * @details Selects backend by path, calls backend->disk_free()
     */
    int64_t disk_free(const char *path);

    /**
     * @brief Get total storage capacity
     * 
     * @param[in] path  Path on target backend (determines which backend to query)
     * 
     * @return Total capacity in bytes, or -1 on error
     * 
     * @details Selects backend by path, calls backend->disk_space()
     */
    int64_t disk_space(const char *path);

    /**
     * @brief Set file modification time
     * 
     * @param[in] filename   File path (with optional prefix)
     * @param[in] mtime_sec  Unix timestamp in seconds
     * 
     * @return true on success, false on error
     * 
     * @details Routes to backend->set_mtime()
     */
    bool set_mtime(const char *filename, const uint32_t mtime_sec);

    // Mount and Format Operations

    /**
     * @brief Attempt to remount filesystems after errors
     * 
     * @return true if any backend successfully mounted, false otherwise
     * 
     * @details Calls retry_mount() on all backends, used for SD card hot-swap recovery
     *          and error recovery after filesystem failures
     */
    bool retry_mount(void);

    /**
     * @brief Unmount all filesystems for safe shutdown
     * 
     * @details Calls unmount() on all backends, used during safe shutdown sequence
     *          to ensure pending writes are flushed
     * 
     * @note Always call before power-off to prevent data corruption
     */
    void unmount(void);

    /**
     * @brief Format primary writable backend (typically FATFS)
     * 
     * @return true if format operation initiated successfully, false on error
     * 
     * @details Asynchronous operation, monitor progress with get_format_status()
     * 
     * @warning Erases all data on the formatted backend (typically SD card)
     */
    bool format(void);

    /**
     * @brief Query format operation progress
     * 
     * @return FormatStatus enum value indicating current state
     * 
     * @details Queries primary backend format status (FS_FORMAT_SUCCESS, FS_FORMAT_IN_PROGRESS, etc.)
     */
    AP_Filesystem_Backend::FormatStatus get_format_status() const;

    // Utility Methods

    /**
     * @brief Read line from file (terminated by CR, LF, or CRLF)
     * 
     * @param[out] buf     Destination buffer for line text
     * @param[in]  buflen  Buffer size in bytes (including null terminator)
     * @param[in]  fd      File descriptor from open()
     * 
     * @return true on success, false on EOF or error
     * 
     * @details Reads until newline (CR, LF, or CRLF), null-terminates result.
     *          Newline character(s) not included in output.
     */
    bool fgets(char *buf, uint8_t buflen, int fd);

    /**
     * @brief Calculate CRC32 checksum of entire file
     * 
     * @param[in]  fname     File path (with optional prefix)
     * @param[out] checksum  Output CRC32 value
     * 
     * @return true on success, false on error (file not found, read error)
     * 
     * @details Opens file, reads in chunks, calculates CRC using standard CRC32 algorithm
     * 
     * @note WARN_IF_UNUSED - must check return value to detect errors
     */
    bool crc32(const char *fname, uint32_t& checksum) WARN_IF_UNUSED;

    /**
     * @brief Load entire file into memory
     * 
     * @param[in] filename  File path (with optional prefix)
     * 
     * @return FileData pointer containing data and length, or nullptr on error
     * 
     * @details Routes to backend->load_file(), may use optimized implementation 
     *          (e.g., ROMFS decompression). The data is guaranteed to be null-terminated
     *          such that it can be treated as a string.
     * 
     * @warning Caller must `delete` returned FileData* to free memory
     * @note Some backends provide optimized implementations (e.g., ROMFS decompression)
     */
    FileData *load_file(const char *filename);

    /**
     * @brief Get AP_Filesystem singleton for scripting access
     * 
     * @return Pointer to singleton instance
     * 
     * @details Provides scripting (Lua) access to filesystem singleton
     */
    static AP_Filesystem *get_singleton(void);

private:
    /**
     * @brief Backend registration entry
     * @details Associates a path prefix with a backend implementation
     */
    struct Backend {
        const char *prefix;            ///< @brief Path prefix for routing (e.g., "@PARAM/"), empty string ("") for default backend
        AP_Filesystem_Backend &fs;     ///< @brief Reference to backend instance (concrete backend object like FATFS, LittleFS, etc.)
    };

    /**
     * @brief Static array of registered backends
     * @details Initialized in AP_Filesystem.cpp, searched in order for path matching.
     *          First match wins. Empty prefix ("") acts as default/fallback backend.
     */
    static const struct Backend backends[];

    /**
     * @brief Find backend matching file path and strip prefix
     * 
     * @param[in,out] path  File path (modified to remove prefix if matched)
     * 
     * @return Reference to matching backend
     * 
     * @details Searches backends[] array for prefix match, strips prefix from path,
     *          returns default backend if no match found.
     * 
     * @note Modifies path parameter by removing matched prefix
     */
    const Backend &backend_by_path(const char *&path) const;

    /**
     * @brief Find backend owning file descriptor and convert to backend-local fd
     * 
     * @param[in,out] fd  System-wide file descriptor (modified to backend-local value)
     * 
     * @return Reference to backend owning this file descriptor
     * 
     * @details Decodes backend index from fd (using MAX_FD_PER_BACKEND ranges),
     *          adjusts fd to backend-local value.
     * 
     * @note Modifies fd parameter to convert from system-wide to backend-local
     * @note FD encoding: backend_index = fd / MAX_FD_PER_BACKEND, local_fd = fd % MAX_FD_PER_BACKEND
     */
    const Backend &backend_by_fd(int &fd) const;

    /**
     * @brief State for virtual directory enumeration
     * @details Used when opening root "/" to list virtual prefixes like @SYS, @PARAM, etc.
     *          Tracks iteration position through backends array.
     */
    struct {
        uint8_t backend_ofs;    ///< @brief Current backend index in iteration
        struct dirent de;       ///< @brief Cached dirent structure for current entry
        uint8_t d_off;          ///< @brief Character offset in backend prefix string
    } virtual_dirent;
};

/**
 * @brief AP namespace for ArduPilot singleton accessors
 */
namespace AP {
    /**
     * @brief Singleton accessor for AP_Filesystem
     * 
     * @return Reference to AP_Filesystem singleton instance
     * 
     * @details Primary access point for all filesystem operations in ArduPilot.
     *          All filesystem operations should use AP::FS().method() pattern.
     * 
     * Example usage:
     * @code
     * int fd = AP::FS().open("@PARAM/params.pck", O_RDONLY);
     * if (fd >= 0) {
     *     char buf[256];
     *     int32_t bytes = AP::FS().read(fd, buf, sizeof(buf));
     *     AP::FS().close(fd);
     * }
     * @endcode
     */
    AP_Filesystem &FS();
};

