/**
 * @file AP_Logger_File.h
 * @brief Filesystem-based logging backend for AP_Logger
 * 
 * @details This backend implements persistent logging to filesystem storage via
 *          AP_Filesystem abstraction. Supports SD cards and internal flash with
 *          multiple filesystem types (FAT, LittleFS).
 * 
 *          Architecture:
 *          - Creates log files named logNN.dat in configured directory
 *          - Tracks current log number via LASTLOG.TXT file
 *          - Implements write buffering with async I/O thread
 *          - Provides log enumeration and download for GCS
 * 
 *          Filesystem Support:
 *          - FAT filesystem on SD cards (ChibiOS, Linux)
 *          - LittleFS on internal flash (ChibiOS)
 *          - Native filesystem on SITL/Linux
 * 
 *          Thread Safety:
 *          - Separate I/O thread handles filesystem operations
 *          - Semaphores protect concurrent access to file descriptors and buffers
 *          - Main thread writes to ring buffer, I/O thread flushes to filesystem
 * 
 * @note This uses POSIX-style file I/O through AP_Filesystem abstraction
 * @warning SD card write performance varies widely (1-50 MB/s) affecting buffer sizing
 * 
 * @see AP_Logger_Backend for base logging interface
 * @see AP_Filesystem for filesystem abstraction
 */
#pragma once

#include <AP_Filesystem/AP_Filesystem.h>

#include <AP_HAL/utility/RingBuffer.h>
#include "AP_Logger_Backend.h"

#if HAL_LOGGING_FILESYSTEM_ENABLED

/**
 * @brief Write buffer chunk size for filesystem I/O operations
 * 
 * @details Defines the size of data written to filesystem in each I/O operation.
 *          Smaller chunks for LittleFS reduce write latency and flash wear.
 *          Larger chunks for FAT/SD cards improve throughput.
 * 
 *          LittleFS: 2048 bytes - matches typical flash page size
 *          Other FS: 4096 bytes - optimized for SD card block writes
 * 
 * @note Override by defining HAL_LOGGER_WRITE_CHUNK_SIZE in hwdef
 */
#ifndef HAL_LOGGER_WRITE_CHUNK_SIZE
#if AP_FILESYSTEM_LITTLEFS_ENABLED
#define HAL_LOGGER_WRITE_CHUNK_SIZE 2048
#else
#define HAL_LOGGER_WRITE_CHUNK_SIZE 4096
#endif
#endif

/**
 * @class AP_Logger_File
 * @brief Filesystem-based logging backend with buffered writes and async I/O
 * 
 * @details Implements persistent logging to filesystem storage with the following features:
 * 
 *          Log File Management:
 *          - Creates log files: logNN.dat (e.g., log001.dat, log002.dat)
 *          - Tracks current log number in LASTLOG.TXT file
 *          - Automatic log rotation and oldest log deletion when space limited
 *          - Supports up to 65535 log files
 * 
 *          Write Buffering Architecture:
 *          - Ring buffer accumulates log data from main thread
 *          - Separate I/O thread flushes buffer to filesystem asynchronously
 *          - Buffer size: HAL_LOGGER_WRITE_CHUNK_SIZE (2048 or 4096 bytes)
 *          - Prevents real-time code blocking on slow filesystem operations
 * 
 *          Thread Safety:
 *          - semaphore: Protects ring buffer access
 *          - write_fd_semaphore: Protects file descriptor access
 *          - io_timer() runs in separate thread for blocking I/O
 *          - Main thread only writes to ring buffer (non-blocking)
 * 
 *          Platform-Specific Behavior:
 *          - SITL: Immediate writes for debugging (no buffering)
 *          - ChibiOS: Buffered writes with I/O thread for performance
 *          - Linux: Buffered writes with native filesystem
 * 
 *          Free Space Management:
 *          - Monitors available filesystem space (1 Hz)
 *          - Enforces minimum free space to prevent corruption
 *          - Automatically stops logging when space exhausted
 *          - Reserved space: 8 MB (FAT), 256 KB (LittleFS), 1 MB (W25NXX flash)
 * 
 *          Log Download Protocol:
 *          - Enumerates logs via get_num_logs() and find_last_log()
 *          - Provides log metadata via get_log_info()
 *          - Streams log data via get_log_data() for MAVLink download
 *          - Separate read file descriptor independent of write operations
 * 
 * @warning SD card performance varies significantly (1-50 MB/s) based on card quality
 * @warning Buffer overrun will cause log data loss if I/O thread cannot keep up
 * @note Filesystem must be mounted before Init() is called
 * @note LASTLOG.TXT corruption will reset log numbering to 1
 * 
 * @see AP_Logger_Backend for base logging interface
 * @see AP_Filesystem for filesystem operations
 */
class AP_Logger_File : public AP_Logger_Backend
{
public:
    /**
     * @brief Construct filesystem-based logger backend
     * 
     * @param[in] front Reference to AP_Logger frontend
     * @param[in] ls Pointer to DFLogStart message writer for log headers
     * 
     * @note Constructor only initializes member variables, filesystem access in Init()
     */
    AP_Logger_File(AP_Logger &front,
                   LoggerMessageWriter_DFLogStart *);

    /**
     * @brief Factory method to create AP_Logger_File backend instance
     * 
     * @param[in] front Reference to AP_Logger frontend
     * @param[in] ls Pointer to DFLogStart message writer
     * 
     * @return Pointer to new AP_Logger_File instance, or nullptr if allocation fails
     * 
     * @note Uses NEW_NOTHROW for safe memory allocation without exceptions
     */
    static AP_Logger_Backend  *probe(AP_Logger &front,
                                     LoggerMessageWriter_DFLogStart *ls) {
        return NEW_NOTHROW AP_Logger_File(front, ls);
    }

    /**
     * @brief Initialize filesystem-based logging backend
     * 
     * @details Performs the following initialization sequence:
     *          1. Ensures log directory exists (creates if missing)
     *          2. Reads LASTLOG.TXT to determine next log number
     *          3. Validates filesystem is writable
     *          4. Allocates write buffer (HAL_LOGGER_WRITE_CHUNK_SIZE)
     *          5. Starts I/O thread for async writes
     * 
     *          LASTLOG.TXT Format:
     *          - Single line containing decimal log number (e.g., "42")
     *          - Updated each time a new log is started
     *          - If missing or corrupt, logging starts at log 1
     * 
     *          Log File Rotation:
     *          - Creates log files: logNN.dat (e.g., log001.dat)
     *          - When starting new log, increments number from LASTLOG.TXT
     *          - Deletes oldest logs if filesystem space insufficient
     * 
     * @note Must be called after filesystem is mounted and available
     * @note If LASTLOG.TXT is corrupt, log numbering resets to 1
     * @warning Failure to initialize disables logging for this backend
     * 
     * @see start_new_log() for log creation
     * @see Prep_MinSpace() for free space enforcement
     */
    void Init() override;
    
    /**
     * @brief Check if storage media (SD card) is inserted and accessible
     * 
     * @return true if filesystem is mounted and accessible, false otherwise
     * 
     * @note Used by frontend to determine if logging is possible
     * @note On platforms without removable media, always returns true if FS mounted
     */
    bool CardInserted(void) const override;

    /**
     * @brief Erase all log files asynchronously
     * 
     * @details Implements non-blocking log deletion:
     *          1. Stops current logging if active
     *          2. Closes all open file descriptors
     *          3. Marks erase operation as pending
     *          4. Actual deletion occurs incrementally in io_timer()
     *          5. Deletes one log file per io_timer() call to avoid blocking
     * 
     *          Asynchronous Implementation:
     *          - Returns immediately without blocking
     *          - Logs deleted one-by-one in background I/O thread
     *          - Prevents long delays that would affect real-time performance
     *          - Erase progress tracked in erase struct
     * 
     * @note Does not delete LASTLOG.TXT (preserves log numbering)
     * @note Logging automatically resumes after erase completes
     * @warning May take several seconds to complete for many large log files
     * 
     * @see erase_next() for incremental deletion logic
     */
    void EraseAll() override;

    /**
     * @brief Write a prioritized block of data to the log buffer
     * 
     * @param[in] pBuffer Pointer to data to write
     * @param[in] size Number of bytes to write
     * @param[in] is_critical true if data is critical and should not be dropped
     * 
     * @return true if data successfully buffered, false if buffer full
     * 
     * @details Writes data to ring buffer for async flush to filesystem:
     *          - Non-blocking operation (only writes to RAM buffer)
     *          - Critical data may overwrite older non-critical data if buffer full
     *          - I/O thread (io_timer) flushes buffer to filesystem
     *          - Buffer size defined by HAL_LOGGER_WRITE_CHUNK_SIZE
     * 
     *          Priority Handling:
     *          - Critical data: Mode changes, errors, arming events
     *          - Non-critical data: High-rate sensor data
     *          - Critical data prevents loss of important flight events
     * 
     * @note Called from main thread at high rate (400 Hz typical)
     * @note Does not guarantee immediate write to filesystem (buffered)
     * @warning If I/O thread cannot keep up, buffer overruns cause data loss
     * 
     * @see io_timer() for buffer flush implementation
     * @see bufferspace_available() to check available buffer space
     */
    bool _WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical) override;
    
    /**
     * @brief Get available space in write buffer
     * 
     * @return Number of bytes available in ring buffer
     * 
     * @details Reports free space in write buffer:
     *          - Used by frontend to determine if logging can continue
     *          - Decreases as data is written to buffer
     *          - Increases as I/O thread flushes data to filesystem
     *          - Buffer size: HAL_LOGGER_WRITE_CHUNK_SIZE bytes
     * 
     * @note Does not reflect filesystem free space, only RAM buffer space
     * @note Returns 0 if logging stopped due to filesystem errors
     * 
     * @see _WritePrioritisedBlock() for buffer write operation
     */
    uint32_t bufferspace_available() override;

    /**
     * @brief Find the most recent log number
     * 
     * @return Log number of most recent log file, or 0 if no logs exist
     * 
     * @details Reads LASTLOG.TXT to determine last written log number:
     *          - LASTLOG.TXT contains single decimal number
     *          - Updated each time start_new_log() is called
     *          - Returns 0 if LASTLOG.TXT missing or unreadable
     * 
     * @note Does not scan directory, relies on LASTLOG.TXT accuracy
     * @see start_new_log() for log number increment
     */
    uint16_t find_last_log() override;
    
    /**
     * @brief Get log boundaries for download protocol
     * 
     * @param[in]  log_num Log number to query
     * @param[out] start_page Starting page (always 1 for file-based logs)
     * @param[out] end_page Ending page (file size / page size)
     * 
     * @details Provides log boundaries for MAVLink download protocol:
     *          - Page size convention: 1 page = 1 byte for file backend
     *          - start_page always returns 1
     *          - end_page returns file size in bytes
     *          - Used by GCS to determine download progress
     * 
     * @note Page numbering is legacy from dataflash backend
     * @see get_log_data() for actual data retrieval
     */
    void get_log_boundaries(uint16_t log_num, uint32_t & start_page, uint32_t & end_page) override;
    
    /**
     * @brief Get log file metadata
     * 
     * @param[in]  log_num Log number to query
     * @param[out] size File size in bytes
     * @param[out] time_utc File creation time as Unix timestamp (UTC)
     * 
     * @details Retrieves log file information for GCS display:
     *          - size: File size from filesystem stat()
     *          - time_utc: File modification time from filesystem
     *          - Returns 0 for both if log file doesn't exist
     * 
     * @note time_utc accuracy depends on RTC availability at log creation
     * @note If RTC not set, time_utc may be incorrect (1970 epoch)
     * 
     * @see _get_log_size() for size retrieval
     * @see _get_log_time() for time retrieval
     */
    void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc) override;
    
    /**
     * @brief Read data from log file for download
     * 
     * @param[in]  log_num Log number to read
     * @param[in]  page Page number (unused for file backend)
     * @param[in]  offset Byte offset in file to start reading
     * @param[in]  len Number of bytes to read
     * @param[out] data Buffer to receive log data
     * 
     * @return Number of bytes actually read, or -1 on error
     * 
     * @details Implements log download for MAVLink protocol:
     *          - Opens log file on first call for given log_num
     *          - Seeks to requested offset
     *          - Reads up to len bytes into data buffer
     *          - Maintains separate read file descriptor (_read_fd)
     *          - Reading does not interfere with active logging
     * 
     * @note page parameter ignored (legacy from dataflash)
     * @note Caller must call end_log_transfer() to close file
     * @warning Reading from currently active log may return partial data
     * 
     * @see end_log_transfer() to close read file descriptor
     * @see get_log_boundaries() for valid offset range
     */
    int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) override;
    
    /**
     * @brief End log transfer and close read file descriptor
     * 
     * @details Closes log file opened by get_log_data():
     *          - Closes _read_fd if open
     *          - Resets _read_fd_log_num
     *          - Called by GCS when download complete or cancelled
     * 
     * @note Safe to call multiple times
     * @see get_log_data() which opens the file
     */
    void end_log_transfer() override;
    
    /**
     * @brief Count total number of log files
     * 
     * @return Number of log files in log directory
     * 
     * @details Scans log directory and counts logNN.dat files:
     *          - Iterates through directory entries
     *          - Matches logNN.dat naming pattern
     *          - Returns count for GCS log list display
     * 
     * @note May be slow for large numbers of logs (directory scan)
     * @note Does not count LASTLOG.TXT or other non-log files
     */
    uint16_t get_num_logs() override;
    
    /**
     * @brief Start a new log file
     * 
     * @details Creates new log file and updates log tracking:
     *          1. Close current log file if open
     *          2. Increment log number from LASTLOG.TXT
     *          3. Ensure sufficient free space (Prep_MinSpace)
     *          4. Create new logNN.dat file
     *          5. Write LASTLOG.TXT with new log number
     *          6. Write log header (DFLogStart message)
     * 
     *          Log Number Management:
     *          - Reads current number from LASTLOG.TXT
     *          - Increments by 1
     *          - Wraps at 65535 back to 1
     *          - Writes new number to LASTLOG.TXT
     * 
     * @note Called automatically when arming if logging enabled
     * @note Can be called manually via MAVLink command
     * @warning Fails silently if insufficient free space
     * 
     * @see Init() for initial log number determination
     * @see Prep_MinSpace() for free space enforcement
     */
    void start_new_log(void) override;
    
    /**
     * @brief Find oldest log file number
     * 
     * @return Log number of oldest log file, or 0 if no logs exist
     * 
     * @details Scans log directory to find log with lowest number:
     *          - Iterates through all logNN.dat files
     *          - Finds minimum log number
     *          - Used for automatic log deletion when space limited
     * 
     * @note May be slow for large numbers of logs (full directory scan)
     * @see Prep_MinSpace() which uses this for log deletion
     */
    uint16_t find_oldest_log() override;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    /**
     * @brief Force immediate flush of write buffer to filesystem
     * 
     * @details SITL/Linux only: Immediately writes buffered data to disk
     *          - Bypasses normal buffering for immediate persistence
     *          - Used during debugging to ensure logs captured before crash
     *          - Calls fsync() to ensure data reaches physical storage
     * 
     * @note Only available on SITL and Linux platforms
     * @note On other platforms, flushing happens asynchronously in io_timer()
     * @warning Reduces performance, only use for debugging
     */
    void flush(void) override;
#endif
    
    /**
     * @brief Periodic 1 Hz maintenance tasks
     * 
     * @details Performs low-frequency maintenance:
     *          - Checks filesystem free space
     *          - Updates logging health status
     *          - Monitors I/O thread health
     *          - Updates MAVLink status messages
     * 
     * @note Called at 1 Hz from scheduler
     * @see periodic_fullrate() for high-rate tasks
     */
    void periodic_1Hz() override;
    
    /**
     * @brief Periodic full-rate maintenance tasks
     * 
     * @details Performs high-frequency checks:
     *          - Monitors buffer utilization
     *          - Checks for I/O errors
     *          - Updates internal timing statistics
     * 
     * @note Called at main loop rate (typically 400 Hz)
     * @note Must be very fast to avoid impacting real-time performance
     */
    void periodic_fullrate() override;

    /**
     * @brief Check if logging has encountered errors
     * 
     * @return true if logging failed due to filesystem errors, false otherwise
     * 
     * @details Reports logging health for MAVLink system status:
     *          - Returns true if write operations failing
     *          - Returns true if filesystem full
     *          - Returns true if I/O thread not responding
     *          - Used to set SYS_STATUS.onboard_control_sensors_health
     * 
     * @note Does not indicate buffer overruns (only filesystem errors)
     * @see logging_started() to check if logging active
     */
    bool logging_failed() const override;

    /**
     * @brief Check if logging is currently active
     * 
     * @return true if log file is open for writing, false otherwise
     * 
     * @note Simple check of write file descriptor validity
     * @see start_new_log() to start logging
     * @see stop_logging() to stop logging
     */
    bool logging_started(void) const override { return _write_fd != -1; }
    
    /**
     * @brief I/O thread timer callback for async filesystem operations
     * 
     * @details Runs in separate thread to perform blocking I/O operations:
     *          1. Check if ring buffer has data to write
     *          2. Copy data from ring buffer (with semaphore protection)
     *          3. Write data to filesystem via AP_Filesystem
     *          4. Update write statistics and error tracking
     *          5. Handle asynchronous erase operations
     * 
     *          Thread Safety:
     *          - Acquires semaphore before accessing ring buffer
     *          - Acquires write_fd_semaphore before filesystem operations
     *          - Main thread only writes to buffer, never blocks on I/O
     * 
     *          Platform-Specific Behavior:
     *          - ChibiOS: Runs in separate RTOS thread with lower priority
     *          - Linux: Runs in separate pthread
     *          - SITL: May run synchronously for deterministic simulation
     * 
     *          Write Strategy:
     *          - Writes chunks of HAL_LOGGER_WRITE_CHUNK_SIZE bytes
     *          - Multiple chunks written per call if buffer contains data
     *          - Returns when buffer empty or write error occurs
     *          - Tracks last write time for FILE_TIMEOUT monitoring
     * 
     * @warning This function performs BLOCKING I/O operations
     * @warning SD card write latency (1-100ms) can stall this thread
     * @warning Must run in separate thread to avoid blocking real-time code
     * 
     * @note Called periodically by scheduler in background thread
     * @note Write performance depends on SD card quality and filesystem
     * @note Buffer overruns occur if this cannot keep up with data rate
     * 
     * @see _WritePrioritisedBlock() which writes to ring buffer
     * @see HAL_LOGGER_WRITE_CHUNK_SIZE for write chunk sizing
     */
    void io_timer(void) override;

protected:

    /**
     * @brief Check if write operations are currently functional
     * 
     * @return true if writes succeeding, false if errors detected
     * 
     * @details Validates logging health:
     *          - Checks if write file descriptor is valid
     *          - Checks if recent write operations succeeded
     *          - Checks if I/O thread is responding
     *          - Used by frontend to determine logging availability
     * 
     * @note Returns false if filesystem full or I/O errors
     */
    bool WritesOK() const override;
    
    /**
     * @brief Check if conditions allow starting a new log
     * 
     * @return true if new log can be started, false otherwise
     * 
     * @details Validates preconditions for log creation:
     *          - Filesystem is accessible
     *          - Sufficient free space available
     *          - No recent open errors
     *          - Write buffer allocated successfully
     * 
     * @see start_new_log() which creates the log file
     */
    bool StartNewLogOK() const override;
    
    /**
     * @brief Prepare for arming by starting logging
     * 
     * @details Called by frontend when vehicle arms:
     *          - Starts new log file if not already logging
     *          - Ensures log captures all armed flight data
     *          - Called before motors spin up
     * 
     * @note May fail silently if filesystem unavailable
     * @see start_new_log() for actual log creation
     */
    void PrepForArming_start_logging() override;

private:
    /// @brief File descriptor for current write log file, or -1 if not logging
    int _write_fd = -1;
    
    /// @brief Dynamically allocated filename string for current log file
    char *_write_filename;
    
    /// @brief Flag indicating last log should be discarded (incomplete/corrupt)
    bool last_log_is_marked_discard;
    
    /// @brief Timestamp (milliseconds) of last successful write operation
    uint32_t _last_write_ms;
    
#if AP_RTC_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    /// @brief Flag indicating file timestamp needs RTC update (ChibiOS only)
    bool _need_rtc_update;
#endif
    
    /// @brief File descriptor for log file being read for download, or -1 if none
    int _read_fd = -1;
    
    /// @brief Log number currently open for reading via _read_fd
    uint16_t _read_fd_log_num;
    
    /// @brief Current read position offset in bytes for get_log_data()
    uint32_t _read_offset;
    
    /// @brief Current write position offset in bytes in active log file
    uint32_t _write_offset;
    
    /// @brief Timestamp (milliseconds) of last file open error, marked volatile for thread access
    volatile uint32_t _open_error_ms;
    
    /// @brief Directory path for log files (e.g., "/APM/LOGS")
    const char *_log_directory;
    
    /// @brief Flag indicating last write operation failed due to filesystem error
    bool _last_write_failed;

    /// @brief Heartbeat counter incremented by io_timer() to prove thread liveness
    uint32_t _io_timer_heartbeat;
    
    /**
     * @brief Check if I/O thread is responding
     * 
     * @return true if io_timer() called recently, false if thread hung
     * 
     * @details Monitors I/O thread health by checking heartbeat updates
     * 
     * @note Used to detect hung I/O thread due to filesystem issues
     */
    bool io_thread_alive() const;
    
    /// @brief Counter for rate-limiting I/O thread warning messages
    uint8_t io_thread_warning_decimation_counter;

    /**
     * @brief Check if recent file open error occurred
     * 
     * @return true if open error within last few seconds, false otherwise
     * 
     * @details Checks _open_error_ms timestamp to detect persistent errors
     * 
     * @note Used to temporarily disable logging after errors
     */
    bool recent_open_error(void) const;

    /**
     * @brief Ensure minimum free space by deleting old logs if necessary
     * 
     * @details Free space management to prevent filesystem corruption:
     *          1. Check available filesystem space
     *          2. If below minimum threshold, delete oldest log
     *          3. Repeat until sufficient space or no logs remain
     * 
     *          Minimum Free Space Thresholds:
     *          - FAT/SD card: 8 MB (_free_space_min_avail)
     *          - LittleFS W25NXX: 1 MB
     *          - LittleFS other: 256 KB
     * 
     * @warning May block for extended period if many logs must be deleted
     * @note Called before starting new log and periodically during logging
     * 
     * @see disk_space_avail() for space checking
     * @see find_oldest_log() for log deletion order
     */
    void Prep_MinSpace();
    
    /**
     * @brief Get available free space on filesystem
     * 
     * @return Available space in bytes, or -1 on error
     * 
     * @details Queries filesystem via AP_Filesystem::disk_free()
     * 
     * @note Cached for 1 second to avoid excessive filesystem queries
     */
    int64_t disk_space_avail();
    
    /**
     * @brief Get total filesystem capacity
     * 
     * @return Total space in bytes, or -1 on error
     * 
     * @details Queries filesystem via AP_Filesystem::disk_space()
     */
    int64_t disk_space();

    /**
     * @brief Create log directory if it doesn't exist
     * 
     * @details Creates directory path specified by _log_directory
     * 
     * @note Called during Init() to ensure directory exists
     * @note Fails silently if directory creation not supported
     */
    void ensure_log_directory_exists();

    /**
     * @brief Check if file exists in filesystem
     * 
     * @param[in] filename Full path to file
     * 
     * @return true if file exists and is accessible, false otherwise
     */
    bool file_exists(const char *filename) const;
    
    /**
     * @brief Check if log file exists for given log number
     * 
     * @param[in] lognum Log number to check
     * 
     * @return true if logNN.dat exists, false otherwise
     * 
     * @note Constructs filename from log number and checks existence
     */
    bool log_exists(const uint16_t lognum) const;

    /**
     * @brief Parse log number from directory entry name
     * 
     * @param[in]  de Directory entry from readdir()
     * @param[out] log_num Extracted log number if successful
     * 
     * @return true if entry matches logNN.dat pattern, false otherwise
     * 
     * @details Parses directory entry names matching "logNNN.dat" format
     * 
     * @note Used during directory scanning for log enumeration
     */
    bool dirent_to_log_num(const dirent *de, uint16_t &log_num) const;
    
    /**
     * @brief Write log number to LASTLOG.TXT file
     * 
     * @param[in] log_num Log number to write
     * 
     * @return true if successfully written, false on error
     * 
     * @details Updates LASTLOG.TXT with current log number:
     *          - Opens LASTLOG.TXT for writing
     *          - Writes decimal log number as ASCII string
     *          - Closes file
     * 
     * @note Called each time a new log is started
     * @warning If this fails, log numbering may restart at 1 on next boot
     */
    bool write_lastlog_file(uint16_t log_num);

    /**
     * @brief Ring buffer for write data accumulation
     * 
     * @details Circular buffer holding log data pending write to filesystem:
     *          - Main thread writes log data to this buffer (non-blocking)
     *          - I/O thread reads from buffer and writes to filesystem
     *          - Size: HAL_LOGGER_WRITE_CHUNK_SIZE bytes
     *          - Protected by semaphore for thread-safe access
     * 
     * @note Buffer size critical for performance vs memory tradeoff
     * @see HAL_LOGGER_WRITE_CHUNK_SIZE for size definition
     */
    ByteBuffer _writebuf{0};
    
    /// @brief Size of chunks written to filesystem in each io_timer() call
    const uint16_t _writebuf_chunk = HAL_LOGGER_WRITE_CHUNK_SIZE;
    
    /// @brief Timestamp (microseconds) of last write operation for timeout detection
    uint32_t _last_write_time;

    /**
     * @brief Construct log filename from log number
     * 
     * @param[in] log_num Log number (1-65535)
     * 
     * @return Dynamically allocated filename string (e.g., "/APM/LOGS/log042.dat")
     * 
     * @warning Caller must free returned string with free()
     * 
     * @note Returns nullptr if allocation fails
     */
    char *_log_file_name(const uint16_t log_num) const;
    
    /**
     * @brief Construct LASTLOG.TXT filename
     * 
     * @return Dynamically allocated filename string (e.g., "/APM/LOGS/LASTLOG.TXT")
     * 
     * @warning Caller must free returned string with free()
     * 
     * @note Returns nullptr if allocation fails
     */
    char *_lastlog_file_name() const;
    
    /**
     * @brief Get size of log file
     * 
     * @param[in] log_num Log number to query
     * 
     * @return File size in bytes, or 0 if file doesn't exist or error
     * 
     * @details Uses filesystem stat() to determine file size
     */
    uint32_t _get_log_size(const uint16_t log_num);
    
    /**
     * @brief Get creation time of log file
     * 
     * @param[in] log_num Log number to query
     * 
     * @return File modification time as Unix timestamp (UTC), or 0 if unavailable
     * 
     * @details Uses filesystem stat() to get file modification time
     * 
     * @note Accuracy depends on RTC being set when log was created
     */
    uint32_t _get_log_time(const uint16_t log_num);

    /**
     * @brief Stop logging and close log file
     * 
     * @details Stops active logging:
     *          1. Flush remaining buffer data to filesystem
     *          2. Close write file descriptor
     *          3. Clear write filename
     *          4. Reset write offset
     * 
     * @note Safe to call multiple times or when not logging
     * @see start_new_log() to resume logging
     */
    void stop_logging(void) override;

    /// @brief Timestamp (milliseconds) of last DFMessageWrite status message sent
    uint32_t last_messagewrite_message_sent;

    /**
     * @brief Free space monitoring and enforcement
     * 
     * @details Prevents filesystem corruption from running out of space:
     *          - Checks free space every _free_space_check_interval (1 second)
     *          - Stops logging if space drops below _free_space_min_avail
     *          - Deletes oldest logs to maintain minimum free space
     * 
     *          Historical Context:
     *          Filling SD cards under NuttX led to corrupt filesystems causing:
     *          - Loss of existing log data
     *          - Failure to gather new data
     *          - Failures-to-boot requiring card reformat
     * 
     * @see Prep_MinSpace() for space enforcement logic
     */
    
    /// @brief Timestamp (milliseconds) of last free space check
    uint32_t _free_space_last_check_time;
    
    /// @brief Interval between free space checks (1000 ms = 1 Hz)
    const uint32_t _free_space_check_interval = 1000UL;
    
#if AP_FILESYSTEM_LITTLEFS_ENABLED
#if AP_FILESYSTEM_LITTLEFS_FLASH_TYPE == AP_FILESYSTEM_FLASH_W25NXX
    /// @brief Minimum free space required: 1 MB for W25NXX NAND flash
    const uint32_t _free_space_min_avail = 1024 * 1024;
#else
    /// @brief Minimum free space required: 256 KB for other LittleFS flash
    const uint32_t _free_space_min_avail = 1024 * 256;
#endif
#else
    /// @brief Minimum free space required: 8 MB for FAT filesystem on SD card
    const uint32_t _free_space_min_avail = 8388608;
#endif

    /**
     * @brief Semaphore protecting ring buffer access
     * 
     * @details Synchronizes access to _writebuf between threads:
     *          - Main thread: Acquires when writing log data to buffer
     *          - I/O thread: Acquires when reading buffer to write filesystem
     *          - Prevents data corruption from concurrent access
     * 
     * @note Critical for thread-safe buffer operations
     */
    HAL_Semaphore semaphore;
    
    /**
     * @brief Semaphore protecting write file descriptor
     * 
     * @details Synchronizes write_fd access between frontend and backend:
     *          - Frontend: Acquires when opening/closing log files
     *          - I/O thread: Acquires before writing to _write_fd
     *          - Prevents writing to closed/invalid file descriptor
     * 
     * @note Essential to prevent crashes from writing to invalid fd
     */
    HAL_Semaphore write_fd_semaphore;

    /**
     * @brief Asynchronous erase state tracking
     * 
     * @details Tracks state for incremental log deletion:
     *          - was_logging: true if logging was active when erase started
     *          - log_num: Current log being deleted in erase sequence
     * 
     * @note EraseAll() sets this up, erase_next() processes incrementally
     * @see EraseAll() to initiate erase
     * @see erase_next() for incremental deletion logic
     */
    struct {
        /// @brief Flag: was actively logging when EraseAll() called
        bool was_logging;
        /// @brief Current log number being processed in erase operation
        uint16_t log_num;
    } erase;
    
    /**
     * @brief Delete next log file in asynchronous erase sequence
     * 
     * @details Called from io_timer() to delete one log per iteration:
     *          - Deletes log number from erase.log_num
     *          - Increments erase.log_num
     *          - Continues until all logs deleted
     *          - Resumes logging if erase.was_logging was true
     * 
     * @note Non-blocking: Only deletes one log per call
     * @see EraseAll() which initiates the erase sequence
     */
    void erase_next(void);

    /// @brief Debug string tracking last I/O operation for error reporting
    const char *last_io_operation = "";

    /// @brief Flag indicating start_new_log() should be called from io_timer()
    bool start_new_log_pending;
};

#endif // HAL_LOGGING_FILESYSTEM_ENABLED
