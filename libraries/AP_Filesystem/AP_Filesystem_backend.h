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
 * @file AP_Filesystem_backend.h
 * @brief Abstract base class defining filesystem backend interface contract
 * 
 * @details All concrete filesystem backends (FATFS, LittleFS, POSIX, ROMFS, virtual) 
 *          inherit from this class and implement the virtual methods. This provides
 *          a unified interface for file operations across different storage media
 *          and filesystem types.
 *          
 *          Architecture: Plugin pattern allowing AP_Filesystem.cpp router to multiplex
 *          across multiple backend implementations based on file path prefixes or
 *          file descriptor encoding.
 *          
 *          Backends may return -1 or nullptr for unsupported operations 
 *          (e.g., ROMFS returns -EROFS for write operations).
 * 
 * Source: libraries/AP_Filesystem/AP_Filesystem_backend.h
 */
#pragma once

#include <stdint.h>
#include <AP_HAL/AP_HAL_Boards.h>

#include "AP_Filesystem_config.h"

#include <AP_InternalError/AP_InternalError.h>

/**
 * @class FileData
 * @brief RAII container for load_file() bulk reads with automatic cleanup
 * 
 * @details Holds file contents loaded into memory with guaranteed null termination
 *          for string compatibility. The destructor automatically calls 
 *          backend->unload_file() to free memory, ensuring proper resource management.
 *          
 *          This class implements the Resource Acquisition Is Initialization (RAII)
 *          pattern, making memory management automatic and exception-safe.
 * 
 * @note Destructor automatically calls backend->unload_file() to free memory
 * @warning Must delete returned FileData* to avoid memory leak
 * 
 * Source: libraries/AP_Filesystem/AP_Filesystem_backend.h:28-40
 */
class FileData {
public:
    /**
     * @brief Size of loaded data in bytes
     * @details Does not include the null terminator. This is the actual file size.
     */
    uint32_t length;
    
    /**
     * @brief Pointer to loaded file contents
     * @details Null-terminated for string compatibility. The data is const to
     *          prevent modification of the loaded buffer.
     */
    const uint8_t *data;

    /**
     * @brief Initialize FileData with backend reference
     * @param _backend Backend pointer for cleanup routing (stored as opaque pointer)
     * @details The backend pointer is used by the destructor to route unload calls
     */
    FileData(void *_backend) :
        backend(_backend) {}
    
    /**
     * @brief Automatic cleanup via backend->unload_file()
     * @details Frees allocated memory when FileData goes out of scope or is deleted.
     *          This ensures proper resource cleanup even in error paths.
     */
    ~FileData();
    
private:
    /**
     * @brief Opaque backend pointer for cleanup
     * @details Used by destructor to route unload call to the correct backend.
     *          Stored as void* to avoid circular dependencies.
     */
    const void *backend;
};

/**
 * @class AP_Filesystem_Backend
 * @brief Abstract base class defining filesystem backend interface
 * 
 * @details This pure virtual interface defines the contract that all filesystem 
 *          backends must implement. Backends may return -1 or nullptr for unsupported
 *          operations (e.g., ROMFS returns -EROFS for write operations). 
 *          
 *          The router (AP_Filesystem.cpp) selects the appropriate backend based on 
 *          file path prefixes (e.g., @PARAM/, @SYS/, @MISSIONS/) or file descriptor 
 *          encoding for physical storage (SD card, internal flash).
 *          
 *          All methods have default implementations returning failure (-1/nullptr),
 *          allowing concrete backends to override only the methods they support.
 * 
 * @note All methods have default implementations returning failure (-1/nullptr)
 * @note Concrete backends override only methods they support
 * @note Backends are responsible for their own thread safety (typically WITH_SEMAPHORE)
 * @note File descriptors should not be shared between threads
 * @note Multiple threads can safely access different files on same backend
 * @note Methods return -1 or nullptr on error
 * @note Backends should set errno for detailed error information
 * @note Common errno values: ENOENT (not found), EACCES (permission), EIO (I/O error), 
 *       ENOSPC (disk full), EROFS (read-only)
 * 
 * Source: libraries/AP_Filesystem/AP_Filesystem_backend.h:42-106
 */
class AP_Filesystem_Backend {

public:
    /**
     * @brief Open or create file
     * 
     * @param[in] fname File path (may include backend prefix like @PARAM/)
     * @param[in] flags POSIX flags (O_RDONLY, O_WRONLY, O_RDWR, O_CREAT, O_TRUNC, O_APPEND)
     * @param[in] allow_absolute_paths Allow unmodified absolute paths (default: false)
     * 
     * @return File descriptor (>=0) on success, -1 on error
     * 
     * @details Default implementation returns -1 (unsupported). Concrete backends
     *          implement actual open logic including hardware access, path validation,
     *          and file descriptor allocation.
     *          
     *          The file descriptor encoding may include backend identification bits
     *          to route subsequent operations to the correct backend.
     * 
     * @note Sets errno on error (ENOENT, EACCES, EIO, etc.)
     */
    virtual int open(const char *fname, int flags, bool allow_absolute_paths = false) {
        return -1;
    }
    
    /**
     * @brief Close file descriptor
     * 
     * @param[in] fd File descriptor returned from open()
     * 
     * @return 0 on success, -1 on error
     * 
     * @details Default implementation returns -1 (unsupported). Concrete backends
     *          implement resource cleanup including flushing buffers, releasing
     *          hardware resources, and deallocating file descriptor.
     * 
     * @note Sets errno on error
     */
    virtual int close(int fd) { return -1; }
    
    /**
     * @brief Read from file
     * 
     * @param[in] fd File descriptor
     * @param[out] buf Destination buffer for read data
     * @param[in] count Maximum number of bytes to read
     * 
     * @return Number of bytes actually read (may be less than count for partial reads),
     *         0 on end-of-file, -1 on error
     * 
     * @details Default implementation returns -1 (unsupported). Concrete backends
     *          implement actual read logic from storage hardware. Partial reads are
     *          allowed and common.
     * 
     * @note Sets errno on error
     * @note May return fewer bytes than requested (partial read)
     */
    virtual int32_t read(int fd, void *buf, uint32_t count) { return -1; }
    
    /**
     * @brief Write to file
     * 
     * @param[in] fd File descriptor
     * @param[in] buf Source buffer containing data to write
     * @param[in] count Number of bytes to write
     * 
     * @return Number of bytes actually written (may be less than count for partial writes),
     *         -1 on error
     * 
     * @details Default implementation returns -1 (unsupported). Concrete backends
     *          implement actual write logic to storage hardware. Read-only backends
     *          (ROMFS) return -EROFS. Partial writes are allowed.
     * 
     * @note Sets errno on error (EROFS for read-only, ENOSPC for disk full)
     * @note May return fewer bytes than requested (partial write)
     * @warning Data not guaranteed persistent until fsync() called
     */
    virtual int32_t write(int fd, const void *buf, uint32_t count) { return -1; }
    
    /**
     * @brief Flush writes to storage
     * 
     * @param[in] fd File descriptor
     * 
     * @return 0 on success, -1 on error
     * 
     * @details Default implementation returns 0 (no-op). Concrete backends implement
     *          actual sync logic to ensure data is committed to persistent storage.
     *          This is critical for data integrity.
     * 
     * @note Essential for data integrity after writes
     * @note Default no-op assumes synchronous writes
     */
    virtual int fsync(int fd) { return 0; }
    
    /**
     * @brief Seek to position in file
     * 
     * @param[in] fd File descriptor
     * @param[in] offset Byte offset for seek operation
     * @param[in] whence Reference point (SEEK_SET=absolute, SEEK_CUR=relative, SEEK_END=from end)
     * 
     * @return New absolute position in bytes from start of file, -1 on error
     * 
     * @details Default implementation returns -1 (unsupported). Concrete backends
     *          implement position tracking and validation.
     *          
     *          SEEK_SET: offset is absolute position from start
     *          SEEK_CUR: offset is relative to current position
     *          SEEK_END: offset is relative to end of file
     * 
     * @note Sets errno on error
     */
    virtual int32_t lseek(int fd, int32_t offset, int whence) { return -1; }
    
    /**
     * @brief Get file information
     * 
     * @param[in] pathname File path
     * @param[out] stbuf Output stat structure to populate
     * 
     * @return 0 on success, -1 on error
     * 
     * @details Default implementation returns -1 (unsupported). Concrete backends
     *          populate stbuf with file metadata including size, modification time,
     *          and mode (file vs directory).
     *          
     *          Typical fields populated: st_size, st_mtime, st_mode
     * 
     * @note Sets errno on error (ENOENT if file not found)
     */
    virtual int stat(const char *pathname, struct stat *stbuf) { return -1; }
    
    /**
     * @brief Delete file
     * 
     * @param[in] pathname File path to delete
     * 
     * @return 0 on success, -1 on error
     * 
     * @details Default implementation returns -1 (unsupported). Concrete backends
     *          implement file deletion from storage hardware. Read-only backends
     *          return -EROFS.
     * 
     * @note Sets errno on error (ENOENT, EACCES, EROFS)
     * @warning Deletion is typically permanent and unrecoverable
     */
    virtual int unlink(const char *pathname) { return -1; }
    
    /**
     * @brief Create directory
     * 
     * @param[in] pathname Directory path to create
     * 
     * @return 0 on success, -1 on error
     * 
     * @details Default implementation returns -1 (unsupported). Concrete backends
     *          implement directory creation. Some backends automatically create
     *          parent directories (recursive mkdir).
     * 
     * @note Sets errno on error (EEXIST if already exists, EROFS for read-only)
     */
    virtual int mkdir(const char *pathname) { return -1; }
    
    /**
     * @brief Open directory for reading
     * 
     * @param[in] pathname Directory path to open
     * 
     * @return Directory handle (opaque pointer) on success, nullptr on error
     * 
     * @details Default implementation returns nullptr (unsupported). Concrete backends
     *          return opaque handle for use with readdir() and closedir().
     *          
     *          The handle type is backend-specific and should not be accessed directly.
     * 
     * @note Sets errno on error (ENOENT, ENOTDIR)
     */
    virtual void *opendir(const char *pathname) { return nullptr; }
    
    /**
     * @brief Read next directory entry
     * 
     * @param[in] dirp Directory handle from opendir()
     * 
     * @return Pointer to dirent structure, or nullptr at end of directory
     * 
     * @details Default implementation returns nullptr (unsupported). Concrete backends
     *          return pointer to next entry with d_name field populated. Returns nullptr
     *          when no more entries (not an error condition).
     *          
     *          The returned pointer may be invalidated by next readdir() call or closedir().
     * 
     * @note nullptr at end is normal, not an error
     * @note Returned pointer may be temporary/reused
     */
    virtual struct dirent *readdir(void *dirp) { return nullptr; }
    
    /**
     * @brief Close directory handle
     * 
     * @param[in] dirp Directory handle from opendir()
     * 
     * @return 0 on success, -1 on error
     * 
     * @details Default implementation returns -1 (unsupported). Concrete backends
     *          implement resource cleanup for directory handle.
     * 
     * @note Sets errno on error
     */
    virtual int closedir(void *dirp) { return -1; }
    
    /**
     * @brief Rename or move file
     * 
     * @param[in] oldpath Source file path
     * @param[in] newpath Destination file path
     * 
     * @return 0 on success, -1 on error
     * 
     * @details Default implementation returns -1 (unsupported). Concrete backends
     *          implement file rename/move operation. May fail if moving across
     *          filesystems or if destination already exists.
     * 
     * @note Sets errno on error (ENOENT, EEXIST, EXDEV for cross-device)
     * @note Atomic operation on same filesystem (backend-dependent)
     */
    virtual int rename(const char *oldpath, const char *newpath) { return -1; }

    /**
     * @brief Advisory threshold for fsync calls
     * 
     * @param[in] fd File descriptor
     * 
     * @return Recommended number of bytes to write before next fsync, 
     *         0 = any number of bytes acceptable
     * 
     * @details Returns number of bytes that should be written before fsync for optimal
     *          streaming performance and robustness. Used by loggers to optimize write
     *          batching and avoid storage stalls.
     *          
     *          Default returns 0 (any size acceptable). Backends like LittleFS return
     *          dynamic values to avoid compaction stalls during logging.
     *          
     *          This is advisory only - callers may ignore and fsync at any time.
     * 
     * @note Return 0 if backend has no preference
     * @note Used for optimizing high-rate logging performance
     */
    virtual uint32_t bytes_until_fsync(int fd) { return 0; }

    /**
     * @brief Get free storage space
     * 
     * @param[in] path Path on target volume (backend uses to identify which volume)
     * 
     * @return Free space in bytes, or -1 on error
     * 
     * @details Default implementation returns 0 (no information available). Concrete
     *          backends query storage hardware for available free space.
     *          
     *          Used for pre-flight checks and logging space warnings.
     * 
     * @note Returns 0 if backend cannot determine (not an error)
     */
    virtual int64_t disk_free(const char *path) { return 0; }

    /**
     * @brief Get total storage capacity
     * 
     * @param[in] path Path on target volume (backend uses to identify which volume)
     * 
     * @return Total capacity in bytes, or -1 on error
     * 
     * @details Default implementation returns 0 (no information available). Concrete
     *          backends query storage hardware for total capacity.
     * 
     * @note Returns 0 if backend cannot determine (not an error)
     */
    virtual int64_t disk_space(const char *path) { return 0; }

    /**
     * @brief Set file modification timestamp
     * 
     * @param[in] filename File path
     * @param[in] mtime_sec Modification time as Unix timestamp (seconds since epoch)
     * 
     * @return true on success, false on failure or unsupported
     * 
     * @details Default implementation returns false (unsupported). Concrete backends
     *          that support timestamps update the file's mtime metadata.
     *          
     *          Used for preserving timestamps during file transfers.
     * 
     * @note Returns false if backend does not support timestamps (not an error)
     */
    virtual bool set_mtime(const char *filename, const uint32_t mtime_sec) { return false; }

    /**
     * @brief Attempt remount after error
     * 
     * @return true if mounted successfully, false if mount failed
     * 
     * @details Default implementation returns true (assume success). Implemented by
     *          FATFS and LittleFS for error recovery after I/O failures or card removal.
     *          
     *          Called by upper layers when filesystem operations fail with I/O errors.
     *          Backends attempt to reinitialize hardware and remount filesystem.
     * 
     * @note Default assumes always mounted (virtual backends)
     * @note May cause brief I/O stall during remount attempt
     */
    virtual bool retry_mount(void) { return true; }

    /**
     * @brief Unmount filesystem for safe shutdown
     * 
     * @details Default implementation is no-op. Concrete backends implement safe
     *          shutdown including flushing all pending writes, updating filesystem
     *          metadata, and releasing hardware resources.
     *          
     *          Called before reboot or hardware power-down.
     * 
     * @note Always call before system reboot
     * @warning Failure to unmount may cause filesystem corruption
     */
    virtual void unmount(void) {}

    /**
     * @enum FormatStatus
     * @brief Format operation status for async format monitoring
     * 
     * @details Format operations are asynchronous to avoid blocking flight-critical
     *          code. Callers poll get_format_status() to monitor progress.
     */
    enum class FormatStatus {
        NOT_STARTED,   ///< @brief Format not initiated
        PENDING,       ///< @brief Format queued but not started
        IN_PROGRESS,   ///< @brief Format operation actively running
        SUCCESS,       ///< @brief Format completed successfully
        FAILURE,       ///< @brief Format failed (I/O error, hardware issue, etc.)
    };

    /**
     * @brief Start asynchronous format operation
     * 
     * @return true if format started successfully, false if failed to start
     * 
     * @details Default implementation returns false (unsupported). Concrete backends
     *          initiate asynchronous format (erase and reinitialize filesystem).
     *          
     *          Format runs in background. Poll get_format_status() for completion.
     *          Format may take several seconds to minutes depending on storage size.
     * 
     * @note Asynchronous operation - monitor with get_format_status()
     * @warning Erases ALL data on the filesystem - unrecoverable
     * @warning Do not perform while armed or in flight
     */
    virtual bool format(void) { return false; }
    
    /**
     * @brief Query format operation progress
     * 
     * @return Current FormatStatus
     * 
     * @details Default implementation returns NOT_STARTED. Concrete backends return
     *          current status of async format operation initiated by format().
     *          
     *          Poll this method to determine when format completes or fails.
     * 
     * @note Safe to call repeatedly for polling
     */
    virtual AP_Filesystem_Backend::FormatStatus get_format_status() const { return FormatStatus::NOT_STARTED; }

    /**
     * @brief Load entire file into memory with null termination
     * 
     * @param[in] filename File path to load
     * 
     * @return FileData* containing data and length, or nullptr on error
     * 
     * @details Loads complete file contents into memory with guaranteed null termination
     *          for string compatibility. Returned FileData* must be deleted by caller
     *          to free memory (RAII destructor handles cleanup).
     *          
     *          Default implementation in AP_Filesystem_backend.cpp uses open/read/close
     *          and can be overridden for optimization. For example, ROMFS returns
     *          decompressed buffer directly without file I/O.
     *          
     *          Useful for loading configuration files, mission data, and scripts.
     * 
     * @return FileData with data and length fields, or nullptr on error
     * 
     * @warning Caller must delete returned FileData* to avoid memory leak
     * @warning May allocate large amounts of memory for large files
     * 
     * @note Data is guaranteed null-terminated for string use
     * @note FileData destructor automatically calls unload_file()
     * 
     * Source: Implementation in libraries/AP_Filesystem/AP_Filesystem_backend.cpp
     */
    virtual FileData *load_file(const char *filename);

    /**
     * @brief Free data from load_file()
     * 
     * @param[in] fd FileData* to free (from load_file())
     * 
     * @details Default implementation frees data buffer. Called automatically by
     *          FileData destructor. Backends can override for custom memory management.
     *          
     *          Typically called via FileData destructor, not directly by users.
     * 
     * @note Usually called automatically by FileData destructor
     * 
     * Source: Implementation in libraries/AP_Filesystem/AP_Filesystem_backend.cpp
     */
    virtual void unload_file(FileData *fd);

protected:
    /**
     * @brief Check if file operations are permitted in current state
     * 
     * @return true if file operations allowed, false if blocked
     * 
     * @details Implemented in AP_Filesystem_backend.cpp. Returns false if system is
     *          armed and caller is main thread (HAL safety gating). This prevents
     *          SD card or flash access from stalling flight-critical code.
     *          
     *          Always returns true for virtual backends (@PARAM, @SYS, @MISSION)
     *          which don't access physical storage hardware.
     *          
     *          Used by FS_CHECK_ALLOWED macro at entry of all file operation methods.
     * 
     * @note Returns false when armed and in main thread (flight safety)
     * @note Always returns true for virtual/memory-based backends
     * 
     * Source: Implementation in libraries/AP_Filesystem/AP_Filesystem_backend.cpp
     */
    bool file_op_allowed(void) const;
};


/**
 * @def FS_CHECK_ALLOWED
 * @brief Safety gate macro for file operations
 * 
 * @param retfail Value to return if operation not allowed
 * 
 * @details Checks file_op_allowed() and returns retfail if false. On SITL builds,
 *          also logs internal error for debugging. Used at entry of all file operation
 *          methods in concrete backends to enforce flight safety rules.
 *          
 *          Prevents main thread file I/O when armed, which could stall the flight
 *          control loop and cause crashes or loss of control.
 *          
 *          Example usage in backend methods:
 *          @code
 *          int Backend::open(const char *fname, int flags) {
 *              FS_CHECK_ALLOWED(-1);  // Return -1 if blocked
 *              // ... actual implementation
 *          }
 *          @endcode
 * 
 * @note Used at entry of all file operation methods in concrete backends
 * @warning Prevents main thread file I/O when armed (could stall flight loop)
 * @warning SITL version logs internal error to catch incorrect usage during testing
 */
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#define FS_CHECK_ALLOWED(retfail) do { if (!file_op_allowed()) { INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control); return retfail; } } while(0)
#else
#define FS_CHECK_ALLOWED(retfail) do { if (!file_op_allowed()) { return retfail; } } while(0)
#endif
