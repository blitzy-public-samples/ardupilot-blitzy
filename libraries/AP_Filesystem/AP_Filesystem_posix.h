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
 * @file AP_Filesystem_posix.h
 * @brief Native POSIX filesystem backend for development and Linux platforms
 * 
 * @details This filesystem backend provides direct access to the host operating
 *          system's native POSIX filesystem. It wraps standard POSIX system calls
 *          (open, read, write, etc.) to provide filesystem access for:
 *          - SITL (Software In The Loop) simulation with host filesystem access
 *          - Linux boards (native storage access)
 *          - QURT (Qualcomm Hexagon DSP) platform
 * 
 *          The backend supports optional base directory mapping for sandboxing
 *          via map_filename(), which can restrict access to a specific directory
 *          tree for security. On SITL, this defaults to preventing access above
 *          the current directory.
 * 
 *          Platform availability is controlled by AP_FILESYSTEM_POSIX_ENABLED.
 * 
 * @note All file operations are gated by file_op_allowed() checks to prevent
 *       file I/O in the main thread when the vehicle is armed (safety feature).
 * 
 * @note This backend enforces O_CLOEXEC on all opened files to ensure file
 *       descriptors are not inherited by child processes.
 * 
 * Source: libraries/AP_Filesystem/AP_Filesystem_posix.h
 * Source: libraries/AP_Filesystem/AP_Filesystem_posix.cpp
 */

#pragma once

#include "AP_Filesystem_config.h"

#if AP_FILESYSTEM_POSIX_ENABLED

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stddef.h>
#include <stdio.h>
#include <dirent.h>
#include <unistd.h>
#include <errno.h>
#include "AP_Filesystem_backend.h"

#ifndef AP_FILESYSTEM_POSIX_HAVE_UTIME
#define AP_FILESYSTEM_POSIX_HAVE_UTIME 1
#endif

#ifndef AP_FILESYSTEM_POSIX_HAVE_FSYNC
#define AP_FILESYSTEM_POSIX_HAVE_FSYNC 1
#endif

#ifndef AP_FILESYSTEM_POSIX_HAVE_STATFS
#define AP_FILESYSTEM_POSIX_HAVE_STATFS 1
#endif

#ifndef O_CLOEXEC
#define O_CLOEXEC 0
#endif

/**
 * @class AP_Filesystem_Posix
 * @brief POSIX filesystem backend providing native host filesystem access
 * 
 * @details This backend implements the AP_Filesystem_Backend interface by wrapping
 *          native POSIX system calls, providing direct access to the host operating
 *          system's filesystem. It is the primary filesystem backend for:
 * 
 *          **Supported Platforms:**
 *          - **SITL**: Maps to host filesystem for log storage and analysis during
 *            simulation. Defaults to sandboxing within the working directory.
 *          - **Linux boards**: Direct native storage access (SD cards, eMMC, etc.)
 *          - **QURT**: Qualcomm Hexagon DSP filesystem integration
 * 
 *          **Key Features:**
 *          - Direct kernel system calls for minimal overhead
 *          - Optional base directory sandboxing via map_filename()
 *          - Automatic O_CLOEXEC enforcement for security
 *          - Full POSIX compatibility (directories, symbolic links, timestamps)
 * 
 *          **Path Mapping:**
 *          When AP_FILESYSTEM_POSIX_MAP_FILENAME_ALLOC is enabled (QURT), paths
 *          are prefixed with a base directory since QURT lacks chdir(). On SITL,
 *          absolute paths are converted to relative paths to prevent access above
 *          the working directory, unless allow_absolute_paths is specified.
 * 
 *          **Safety Mechanisms:**
 *          - FS_CHECK_ALLOWED() macro prevents file operations in main thread when armed
 *          - Only regular files and symbolic links can be opened (blocks on devices)
 *          - O_CLOEXEC automatically added to prevent fd leaks to child processes
 * 
 *          **Performance:**
 *          Direct system calls provide native filesystem performance with no caching
 *          or buffering beyond what the kernel provides. All operations are synchronous.
 * 
 * @note This backend is only available when AP_FILESYSTEM_POSIX_ENABLED is defined.
 * 
 * @warning Host filesystem errors (permission denied, disk full, etc.) propagate
 *          directly as errno values and must be handled by calling code.
 * 
 * Source: libraries/AP_Filesystem/AP_Filesystem_posix.cpp:42-65 (map_filename)
 * Source: libraries/AP_Filesystem/AP_Filesystem_posix.cpp:74-96 (open implementation)
 */
class AP_Filesystem_Posix : public AP_Filesystem_Backend
{
public:
    // POSIX filesystem operations - direct wrappers around native system calls
    
    /**
     * @brief Open a file using native POSIX open() system call
     * 
     * @details Opens a file with the specified flags, automatically adding O_CLOEXEC
     *          to prevent file descriptor inheritance by child processes. Only regular
     *          files and symbolic links can be opened (device files are blocked).
     * 
     *          Unless allow_absolute_paths is true, the path is processed through
     *          map_filename() for sandboxing. On SITL, absolute paths are converted
     *          to relative paths. On QURT, a base directory prefix is added.
     * 
     * @param[in] fname File path to open (relative or absolute)
     * @param[in] flags POSIX open flags (O_RDONLY, O_WRONLY, O_RDWR, O_CREAT, O_TRUNC, etc.)
     * @param[in] allow_absolute_paths If true, skip path mapping and use fname as-is
     * 
     * @return File descriptor on success (>=0), -1 on error with errno set
     * 
     * @note O_CLOEXEC is automatically added to flags for safety
     * @note File permissions for newly created files are set to 0644
     * @note Blocked by FS_CHECK_ALLOWED() when armed in main thread
     * 
     * @warning Attempting to open device files (not regular files or symlinks) will fail
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_posix.cpp:74-96
     */
    int open(const char *fname, int flags, bool allow_absolute_paths = false) override;
    
    /**
     * @brief Close a file descriptor using native POSIX close() system call
     * 
     * @param[in] fd File descriptor to close
     * 
     * @return 0 on success, -1 on error with errno set
     * 
     * @note Blocked by FS_CHECK_ALLOWED() when armed in main thread
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_posix.cpp:98-102
     */
    int close(int fd) override;
    
    /**
     * @brief Read data from file using native POSIX read() system call
     * 
     * @param[in]  fd    File descriptor to read from
     * @param[out] buf   Buffer to store read data
     * @param[in]  count Maximum number of bytes to read
     * 
     * @return Number of bytes actually read on success, -1 on error with errno set
     * 
     * @note May return fewer bytes than requested (not an error)
     * @note Blocked by FS_CHECK_ALLOWED() when armed in main thread
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_posix.cpp:104-108
     */
    int32_t read(int fd, void *buf, uint32_t count) override;
    
    /**
     * @brief Write data to file using native POSIX write() system call
     * 
     * @param[in] fd    File descriptor to write to
     * @param[in] buf   Buffer containing data to write
     * @param[in] count Number of bytes to write
     * 
     * @return Number of bytes actually written on success, -1 on error with errno set
     * 
     * @note May write fewer bytes than requested (not an error)
     * @note Blocked by FS_CHECK_ALLOWED() when armed in main thread
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_posix.cpp:110-114
     */
    int32_t write(int fd, const void *buf, uint32_t count) override;
    
    /**
     * @brief Synchronize file data to storage using native POSIX fsync() system call
     * 
     * @details Forces all buffered data for the file to be written to physical storage.
     *          If AP_FILESYSTEM_POSIX_HAVE_FSYNC is not available, returns success
     *          without performing synchronization (caller will not treat as I/O failure).
     * 
     * @param[in] fd File descriptor to synchronize
     * 
     * @return 0 on success, -1 on error with errno set
     * 
     * @note Returns 0 (success) on platforms without fsync() support to avoid caller errors
     * @note Blocked by FS_CHECK_ALLOWED() when armed in main thread
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_posix.cpp:116-126
     */
    int fsync(int fd) override;
    
    /**
     * @brief Seek to position in file using native POSIX lseek() system call
     * 
     * @param[in] fd     File descriptor to seek
     * @param[in] offset Offset in bytes (interpretation depends on whence)
     * @param[in] whence Seek origin: SEEK_SET (start), SEEK_CUR (current), SEEK_END (end)
     * 
     * @return New file position in bytes from start of file, -1 on error with errno set
     * 
     * @note Blocked by FS_CHECK_ALLOWED() when armed in main thread
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_posix.cpp:128-132
     */
    int32_t lseek(int fd, int32_t offset, int whence) override;
    
    /**
     * @brief Get file status using native POSIX stat() system call
     * 
     * @details Retrieves file metadata including size, permissions, timestamps, and type.
     *          The pathname is processed through map_filename() for sandboxing.
     * 
     * @param[in]  pathname Path to file or directory to query
     * @param[out] stbuf    Pointer to stat structure to fill with file information
     * 
     * @return 0 on success, -1 on error with errno set
     * 
     * @note Blocked by FS_CHECK_ALLOWED() when armed in main thread
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_posix.cpp:134-141
     */
    int stat(const char *pathname, struct stat *stbuf) override;
    
    /**
     * @brief Remove file or directory using native POSIX unlink()/rmdir() system calls
     * 
     * @details Matches FATFS interface by supporting removal of both files and directories.
     *          First attempts rmdir() for directories, then falls back to unlink() for files.
     *          The pathname is processed through map_filename() for sandboxing.
     * 
     * @param[in] pathname Path to file or directory to remove
     * 
     * @return 0 on success, -1 on error with errno set
     * 
     * @note Directories must be empty to be removed
     * @note Blocked by FS_CHECK_ALLOWED() when armed in main thread
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_posix.cpp:143-155
     */
    int unlink(const char *pathname) override;
    
    /**
     * @brief Create directory using native POSIX mkdir() system call
     * 
     * @details Creates a new directory with permissions 0775 (rwxrwxr-x).
     *          The pathname is processed through map_filename() for sandboxing.
     * 
     * @param[in] pathname Path of directory to create
     * 
     * @return 0 on success, -1 on error with errno set (EEXIST if already exists)
     * 
     * @note Parent directory must exist (does not create intermediate directories)
     * @note Directory permissions are set to 0775
     * @note Blocked by FS_CHECK_ALLOWED() when armed in main thread
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_posix.cpp:157-164
     */
    int mkdir(const char *pathname) override;
    
    /**
     * @brief Open directory for reading using native POSIX opendir() system call
     * 
     * @details Opens a directory stream for iterating through directory entries.
     *          The pathname is processed through map_filename() for sandboxing.
     *          Returns an opaque handle (actually DIR*) for use with readdir()/closedir().
     * 
     * @param[in] pathname Path to directory to open
     * 
     * @return Directory handle on success, nullptr on error with errno set
     * 
     * @note Must be closed with closedir() to avoid resource leaks
     * @note Blocked by FS_CHECK_ALLOWED() when armed in main thread
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_posix.cpp:166-173
     */
    void *opendir(const char *pathname) override;
    
    /**
     * @brief Read next directory entry using native POSIX readdir() system call
     * 
     * @param[in] dirp Directory handle from opendir()
     * 
     * @return Pointer to dirent structure on success, nullptr at end of directory or on error
     * 
     * @note Returned pointer is valid until next readdir() call or closedir()
     * @note Blocked by FS_CHECK_ALLOWED() when armed in main thread
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_posix.cpp:175-179
     */
    struct dirent *readdir(void *dirp) override;
    
    /**
     * @brief Close directory handle using native POSIX closedir() system call
     * 
     * @param[in] dirp Directory handle from opendir()
     * 
     * @return 0 on success, -1 on error with errno set
     * 
     * @note Blocked by FS_CHECK_ALLOWED() when armed in main thread
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_posix.cpp:181-185
     */
    int closedir(void *dirp) override;
    
    /**
     * @brief Rename or move file/directory using native POSIX rename() system call
     * 
     * @details Atomically renames a file or directory. Can move files between directories
     *          on the same filesystem. Both paths are processed through map_filename()
     *          for sandboxing.
     * 
     * @param[in] oldpath Current path of file or directory
     * @param[in] newpath New path for file or directory
     * 
     * @return 0 on success, -1 on error with errno set
     * 
     * @note Cannot move files across filesystem boundaries (returns EXDEV)
     * @note If newpath exists, it will be atomically replaced (for files)
     * @note Blocked by FS_CHECK_ALLOWED() when armed in main thread
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_posix.cpp:187-196
     */
    int rename(const char *oldpath, const char *newpath) override;

    /**
     * @brief Get free disk space using native POSIX statfs() system call
     * 
     * @details Queries the filesystem containing the specified path to determine
     *          available disk space. Returns available blocks * block size in bytes.
     *          Only available when AP_FILESYSTEM_POSIX_HAVE_STATFS is defined.
     * 
     * @param[in] path Path to any file or directory on the filesystem to query
     * 
     * @return Free disk space in bytes, -1 on error or if statfs() not available
     * 
     * @note Reports space available to unprivileged users (bavail, not bfree)
     * @note Path is processed through map_filename() for sandboxing
     * @note Blocked by FS_CHECK_ALLOWED() when armed in main thread
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_posix.cpp:199-214
     */
    int64_t disk_free(const char *path) override;

    /**
     * @brief Get total disk space using native POSIX statfs() system call
     * 
     * @details Queries the filesystem containing the specified path to determine
     *          total disk capacity. Returns total blocks * block size in bytes.
     *          Only available when AP_FILESYSTEM_POSIX_HAVE_STATFS is defined.
     * 
     * @param[in] path Path to any file or directory on the filesystem to query
     * 
     * @return Total disk space in bytes, -1 on error or if statfs() not available
     * 
     * @note Reports total filesystem size (f_blocks)
     * @note Path is processed through map_filename() for sandboxing
     * @note Blocked by FS_CHECK_ALLOWED() when armed in main thread
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_posix.cpp:217-232
     */
    int64_t disk_space(const char *path) override;

    /**
     * @brief Set file modification time using native POSIX utime() system call
     * 
     * @details Updates both access time and modification time to the specified value.
     *          Only available when AP_FILESYSTEM_POSIX_HAVE_UTIME is defined.
     * 
     * @param[in] filename  Path to file to modify
     * @param[in] mtime_sec Modification time in seconds since Unix epoch (1970-01-01)
     * 
     * @return true on success, false on error or if utime() not available
     * 
     * @note Sets both access time and modification time to the same value
     * @note Path is processed through map_filename() for sandboxing
     * @note Blocked by FS_CHECK_ALLOWED() when armed in main thread
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_posix.cpp:238-250
     */
    bool set_mtime(const char *filename, const uint32_t mtime_sec) override;
};

#endif  // AP_FILESYSTEM_POSIX_ENABLED
