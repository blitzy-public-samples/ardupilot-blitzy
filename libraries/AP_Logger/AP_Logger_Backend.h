/**
 * @file AP_Logger_Backend.h
 * @brief Abstract backend interface for AP_Logger storage implementations
 * 
 * @details This file defines the abstract base class AP_Logger_Backend that all
 *          storage backend implementations must inherit from and implement.
 *          
 *          Backend implementations include:
 *          - AP_Logger_File: Stores logs to filesystem (SD card, onboard flash)
 *          - AP_Logger_Block: Stores logs to block-oriented storage (DataFlash)
 *          - AP_Logger_MAVLink: Streams logs over MAVLink telemetry
 *          
 *          The backend architecture provides a pluggable storage layer,
 *          allowing different platforms to use appropriate storage mechanisms
 *          while maintaining a consistent logging API for vehicle code.
 *          
 *          Rate limiting functionality (AP_Logger_RateLimiter) manages per-message
 *          logging rates for armed and disarmed states to balance data resolution
 *          with storage capacity and telemetry bandwidth.
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_Logger_config.h"

#if HAL_LOGGING_ENABLED

#include <AP_Common/Bitmask.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Vehicle/ModeReason.h>
#include "LogStructure.h"

class LoggerMessageWriter_DFLogStart;

/**
 * @class AP_Logger_RateLimiter
 * @brief Rate limiter for log messages to control logging frequency per message type
 * 
 * @details Implements per-message-type rate limiting with separate limits for
 *          armed and disarmed states. Uses timing cache to efficiently track
 *          when each message type was last logged and implements special handling
 *          for multi-instance messages (same message ID logged multiple times
 *          in a single scheduler loop iteration).
 *          
 *          Rate limiting prevents log storage exhaustion during high-rate events
 *          while ensuring critical data is captured. Armed rate limits are typically
 *          higher than disarmed limits since flight data has higher priority.
 *          
 *          The class maintains three caches:
 *          - last_send_ms: Timestamp of last successful log for each message type
 *          - last_sched_count: Scheduler counter to detect multi-instance messages
 *          - not_streaming: Bitmask cache of non-streaming message types
 * 
 * @note Rate limits are specified in Hz via AP_Float parameters
 * @note Multi-instance detection ensures all instances of a message pass or fail together
 */
class AP_Logger_RateLimiter
{
public:
    /**
     * @brief Construct a rate limiter with armed and disarmed rate limit parameters
     * 
     * @param[in] _front Reference to the AP_Logger frontend for configuration access
     * @param[in] _limit_hz Armed rate limit in Hz (higher rate for flight data)
     * @param[in] _disarm_limit_hz Disarmed rate limit in Hz (lower rate to conserve storage)
     */
    AP_Logger_RateLimiter(const class AP_Logger &_front, const AP_Float &_limit_hz, const AP_Float &_disarm_limit_hz);

    /**
     * @brief Determine if a message should be logged based on rate limiting
     * 
     * @details Implements rate limiting with multi-instance message handling.
     *          When the same message ID is logged multiple times in one scheduler
     *          iteration (multi-instance messages like IMU1, IMU2), all instances
     *          pass or fail together to maintain temporal correlation.
     *          
     *          Uses scheduler counter to detect multi-instance scenarios and caches
     *          the last decision to ensure consistency across instances.
     * 
     * @param[in] msgid Message type ID to check (0-255)
     * @param[in] writev_streaming True if message is being written via streaming API
     * 
     * @return true if message passes rate limit test and should be logged
     * @return false if message should be dropped due to rate limiting
     * 
     * @note Streaming messages bypass standard rate limiting and use should_log_streaming()
     */
    bool should_log(uint8_t msgid, bool writev_streaming);
    
    /**
     * @brief Determine if a streaming message should be logged at specified rate
     * 
     * @details Streaming messages (high-frequency sensor data) use explicit rate
     *          specification rather than global armed/disarmed rates. This allows
     *          fine-grained control over high-bandwidth data sources like raw IMU.
     * 
     * @param[in] msgid Message type ID to check (0-255)
     * @param[in] rate_hz Desired logging rate in Hz for this message type
     * 
     * @return true if sufficient time has elapsed since last log
     * @return false if message should be dropped to maintain specified rate
     */
    bool should_log_streaming(uint8_t msgid, float rate_hz);

private:
    const AP_Logger &front;  ///< Reference to frontend for configuration and scheduler access
    const AP_Float &rate_limit_hz;  ///< Armed rate limit in Hz (applied during flight)
    const AP_Float &disarm_rate_limit_hz;  ///< Disarmed rate limit in Hz (conserves storage on ground)

    /// Time in milliseconds (16-bit wrap-safe) when each message type was last logged
    /// Indexed by message ID (0-255), stores AP_HAL::millis16() timestamp
    uint16_t last_send_ms[256];

    /// Scheduler counter when each message type was last logged
    /// Used to detect multi-instance messages within same scheduler iteration
    /// If sched_count matches, it's a multi-instance message requiring consistent decision
    uint16_t last_sched_count[256];

    /// Bitmask cache of non-streaming message types (true = not streaming)
    /// Avoids repeated expensive structure_for_msg_type() lookups
    /// Populated on first access for each message type
    Bitmask<256> not_streaming;

    /// Cached result of last rate limit decision for each message type
    /// Used for multi-instance handling to ensure all instances get same decision
    /// If last_sched_count matches current, return cached result
    Bitmask<256> last_return;
};

/**
 * @class AP_Logger_Backend
 * @brief Abstract base class for AP_Logger storage backend implementations
 * 
 * @details Defines the interface that all logging storage backends must implement.
 *          This provides a pluggable architecture allowing different platforms to
 *          use appropriate storage mechanisms:
 *          
 *          - **File backend** (AP_Logger_File): Uses filesystem API for SD cards
 *            or onboard flash storage. Provides high capacity and easy log retrieval.
 *          
 *          - **Block backend** (AP_Logger_Block): Uses block-oriented DataFlash chips
 *            via SPI. Provides reliable storage on boards without filesystem support.
 *          
 *          - **MAVLink backend** (AP_Logger_MAVLink): Streams logs over MAVLink
 *            telemetry to ground station. Enables real-time log collection.
 *          
 *          The backend handles low-level storage operations while the frontend
 *          (AP_Logger) provides the high-level logging API. Backends implement
 *          write operations, storage management, log enumeration, and retrieval.
 *          
 *          **Thread Safety**: Backends must handle concurrent access from:
 *          - Main thread writing log messages
 *          - IO thread performing asynchronous writes (file backends)
 *          - MAVLink threads requesting log download
 *          
 *          Use semaphores (WITH_SEMAPHORE) to protect shared state.
 *          
 *          **Performance Considerations**: Write methods are called at high rate
 *          (400Hz+ for critical messages). Backends should buffer writes and
 *          perform actual I/O asynchronously to avoid blocking vehicle code.
 * 
 * @note Backends inherit from this class and implement all pure virtual methods
 * @warning Write operations must complete quickly (<1ms) to avoid impacting flight performance
 */
class AP_Logger_Backend
{

public:
    FUNCTOR_TYPEDEF(vehicle_startup_message_Writer, void);

    /**
     * @brief Construct a logger backend instance
     * 
     * @param[in] front Reference to AP_Logger frontend
     * @param[in] writer Pointer to startup message writer for emitting initial log entries
     */
    AP_Logger_Backend(AP_Logger &front,
                      class LoggerMessageWriter_DFLogStart *writer);

    /**
     * @brief Get the vehicle startup message writer functor
     * 
     * @return vehicle_startup_message_Writer Functor for writing vehicle-specific startup messages
     */
    vehicle_startup_message_Writer vehicle_message_writer() const;

    /**
     * @brief Check if storage media is inserted and available
     * 
     * @details Backends implement this to report storage availability:
     *          - File backend: Check if SD card is mounted
     *          - Block backend: Check if DataFlash chip is detected
     *          - MAVLink backend: Check if client is connected
     * 
     * @return true if storage media is present and accessible
     * @return false if storage media is missing or inaccessible
     * 
     * @note Pure virtual - must be implemented by all backends
     */
    virtual bool CardInserted(void) const = 0;

    /**
     * @brief Erase all logs from storage
     * 
     * @details Performs storage-specific erase operation:
     *          - File backend: Delete all log files from filesystem
     *          - Block backend: Erase all DataFlash blocks and reset log numbers
     *          - MAVLink backend: Clear any buffered logs
     *          
     *          This is a destructive operation that cannot be undone.
     *          Typically called via ground station command or CLI.
     * 
     * @warning All log data will be permanently lost
     * @note Pure virtual - must be implemented by all backends
     */
    virtual void EraseAll() = 0;

    /**
     * @brief Write a normal priority block of data to the log
     * 
     * @details Writes log message data with standard priority. May be dropped
     *          if buffer is full or rate limiting is active. Use for non-critical
     *          sensor data, status messages, and informational logs.
     * 
     * @param[in] pBuffer Pointer to data buffer containing formatted log message
     * @param[in] size Size of data buffer in bytes
     * 
     * @return true if data was successfully queued for writing
     * @return false if data was dropped (buffer full, rate limited, or backend busy)
     * 
     * @note Delegates to WritePrioritisedBlock() with is_critical=false
     * @see WritePrioritisedBlock()
     */
    bool WriteBlock(const void *pBuffer, uint16_t size) {
        return WritePrioritisedBlock(pBuffer, size, false);
    }

    /**
     * @brief Write a critical priority block of data to the log
     * 
     * @details Writes log message data with high priority and reserved buffer space.
     *          Critical messages are guaranteed buffer allocation and bypass rate
     *          limiting. Use for safety-critical events: mode changes, arming state,
     *          failsafe triggers, errors, and flight-critical parameter changes.
     * 
     * @param[in] pBuffer Pointer to data buffer containing formatted log message
     * @param[in] size Size of data buffer in bytes
     * 
     * @return true if data was successfully queued for writing
     * @return false if data was dropped (should be rare for critical messages)
     * 
     * @note Delegates to WritePrioritisedBlock() with is_critical=true
     * @warning Reserve critical writes for truly important events to avoid buffer exhaustion
     * @see WritePrioritisedBlock()
     */
    bool WriteCriticalBlock(const void *pBuffer, uint16_t size) {
        return WritePrioritisedBlock(pBuffer, size, true);
    }

    /**
     * @brief Write a block with specified priority and streaming mode
     * 
     * @details Public interface for writing log data with priority control.
     *          Performs validation, rate limiting, format emission, and delegates
     *          to backend-specific _WritePrioritisedBlock() for actual storage.
     *          
     *          **Buffer Management**:
     *          - Critical messages have reserved buffer space (1KB typical)
     *          - Non-critical messages compete for remaining buffer space
     *          - Rate limiting applied to non-critical messages
     *          
     *          **Streaming Mode**:
     *          - writev_streaming=true for high-rate sensor data (e.g., raw IMU)
     *          - Uses different rate limiting algorithm based on desired Hz
     *          - Allows burst writes of multi-part messages
     * 
     * @param[in] pBuffer Pointer to data buffer containing formatted log message
     * @param[in] size Size of data buffer in bytes (typically sizeof(log_Message))
     * @param[in] is_critical true for critical messages with guaranteed buffer space
     * @param[in] writev_streaming true for streaming messages with rate-based limiting
     * 
     * @return true if data was accepted and queued for writing
     * @return false if data was dropped due to buffer full, rate limiting, or errors
     * 
     * @note Called at high rate (400Hz+) - must complete quickly
     * @warning Must not block or perform slow I/O operations
     * @see _WritePrioritisedBlock()
     */
    bool WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical, bool writev_streaming=false);

    /**
     * @brief Find the log number of the most recent log
     * 
     * @details Returns the highest log number currently stored. Used to determine
     *          the next log number when starting a new log.
     *          
     *          **List Entry vs Log Number**: This returns a log number, not a list_entry.
     *          List entries are 0-based indices into the list of available logs,
     *          while log numbers are sequential identifiers that may have gaps.
     * 
     * @return uint16_t Log number of most recent log, or 0 if no logs exist
     * 
     * @note Pure virtual - must be implemented by all backends
     */
    virtual uint16_t find_last_log() = 0;
    
    /**
     * @brief Get storage boundaries for a specific log
     * 
     * @details Returns the start and end page numbers for a log identified by
     *          its position in the log list (list_entry, not log number).
     *          Used for log download and analysis to know how to retrieve log data.
     *          
     *          **List Entry Indexing**: list_entry is 0-based index into the
     *          list of available logs, ordered by log number. Entry 0 is the
     *          oldest log, entry (get_num_logs()-1) is the newest log.
     *          
     *          Page size and interpretation is backend-specific:
     *          - File backend: Pages may represent file chunks
     *          - Block backend: Pages are DataFlash block numbers
     * 
     * @param[in] list_entry 0-based index into list of logs (0 = oldest)
     * @param[out] start_page Starting page/block number of this log
     * @param[out] end_page Ending page/block number of this log (inclusive)
     * 
     * @note Pure virtual - must be implemented by all backends
     * @warning list_entry must be < get_num_logs() or behavior is undefined
     */
    virtual void get_log_boundaries(uint16_t list_entry, uint32_t & start_page, uint32_t & end_page) = 0;
    
    /**
     * @brief Get metadata for a specific log
     * 
     * @details Returns size and timestamp for a log identified by list_entry.
     *          Used by ground stations to display log list and determine which
     *          logs to download.
     *          
     *          **List Entry Indexing**: See get_log_boundaries() for explanation.
     * 
     * @param[in] list_entry 0-based index into list of logs (0 = oldest)
     * @param[out] size Log size in bytes
     * @param[out] time_utc Log timestamp in Unix epoch seconds (UTC)
     * 
     * @note Pure virtual - must be implemented by all backends
     * @warning list_entry must be < get_num_logs() or behavior is undefined
     */
    virtual void get_log_info(uint16_t list_entry, uint32_t &size, uint32_t &time_utc) = 0;
    
    /**
     * @brief Read data from a specific log
     * 
     * @details Retrieves a chunk of log data for download to ground station or
     *          analysis tool. Supports random access within a log using page and
     *          offset addressing.
     *          
     *          **List Entry Indexing**: See get_log_boundaries() for explanation.
     *          
     *          **Addressing**: Combines page number (from get_log_boundaries())
     *          with byte offset within that page. Backend translates to actual
     *          storage location.
     * 
     * @param[in] list_entry 0-based index into list of logs (0 = oldest)
     * @param[in] page Page/block number within the log
     * @param[in] offset Byte offset within the specified page
     * @param[in] len Number of bytes to read (typically 90 bytes for MAVLink packet)
     * @param[out] data Buffer to receive log data
     * 
     * @return int16_t Number of bytes actually read, or -1 on error
     * 
     * @note Pure virtual - must be implemented by all backends
     * @warning Caller must ensure data buffer is at least len bytes
     */
    virtual int16_t get_log_data(uint16_t list_entry, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) = 0;
    
    /**
     * @brief Signal that log transfer is complete
     * 
     * @details Called when ground station finishes downloading a log.
     *          Backends can release resources (close files, free buffers).
     * 
     * @note Pure virtual - must be implemented by all backends
     */
    virtual void end_log_transfer() = 0;
    
    /**
     * @brief Get total number of logs available
     * 
     * @details Returns count of logs that can be accessed via list_entry indexing.
     *          Valid list_entry values are 0 to (get_num_logs()-1).
     * 
     * @return uint16_t Number of logs available, or 0 if no logs exist
     * 
     * @note Pure virtual - must be implemented by all backends
     */
    virtual uint16_t get_num_logs() = 0;
    
    /**
     * @brief Find the log number of the oldest log
     * 
     * @details Returns the lowest log number currently stored. Default implementation
     *          uses get_log_boundaries() to find oldest. Backends can override for
     *          more efficient implementation.
     * 
     * @return uint16_t Log number of oldest log, or 0 if no logs exist
     * 
     * @note Virtual - backends can override default implementation
     */
    virtual uint16_t find_oldest_log();

    /**
     * @brief Check if logging is currently active
     * 
     * @details Reports whether the backend is currently accepting and writing
     *          log messages. Used to determine if vehicle is ready to arm
     *          (logging must be active before arming).
     * 
     * @return true if logging is active and accepting messages
     * @return false if logging is stopped or not initialized
     * 
     * @note Pure virtual - must be implemented by all backends
     */
    virtual bool logging_started(void) const = 0;

    /**
     * @brief Initialize the logging backend
     * 
     * @details Performs backend-specific initialization:
     *          - File backend: Mount filesystem, create log directory
     *          - Block backend: Detect DataFlash chip, read log directory
     *          - MAVLink backend: Initialize streaming buffers
     *          
     *          Called once during system startup. Backends should probe hardware,
     *          allocate resources, and prepare for logging.
     * 
     * @note Pure virtual - must be implemented by all backends
     * @warning Must be called before any write operations
     */
    virtual void Init() = 0;

    /**
     * @brief Get available buffer space for new log messages
     * 
     * @details Returns bytes available in write buffer. Used to determine if
     *          there is space for new messages and to calculate drop statistics.
     *          
     *          The buffer absorbs burst writes and is drained by asynchronous I/O.
     *          Low buffer space indicates I/O can't keep up with logging rate.
     * 
     * @return uint32_t Available buffer space in bytes
     * 
     * @note Pure virtual - must be implemented by all backends
     * @warning If buffer fills completely, messages will be dropped
     */
    virtual uint32_t bufferspace_available() = 0;

    /**
     * @brief Prepare backend for vehicle arming
     * 
     * @details Called when vehicle is about to arm. Backends should ensure logging
     *          is active and ready to capture flight data. Default implementation
     *          calls PrepForArming_start_logging() to start a new log if needed.
     *          
     *          This ensures log data is available from the moment of arming, which
     *          is critical for post-flight analysis and incident investigation.
     * 
     * @note Virtual - backends can override to add specific preparation steps
     */
    virtual void PrepForArming();

    /**
     * @brief Start a new log file/sequence
     * 
     * @details Closes current log (if any) and starts a new log with incremented
     *          log number. Called at:
     *          - System startup (if configured)
     *          - Vehicle arming (if configured)
     *          - Explicit user command
     *          
     *          Default implementation is empty; backends override to implement
     *          storage-specific log rotation.
     * 
     * @note Virtual - backends override to implement log creation
     */
    virtual void start_new_log() { }

    /**
     * @brief Stop logging and close storage resources
     * 
     * @details Stops accepting new log messages and performs backend-specific
     *          shutdown operations:
     *          - File backend: Flush buffers, close log file
     *          - Block backend: Update directory, flush pending writes
     *          - MAVLink backend: Signal end of stream to client
     *          
     *          **Important**: This does NOT prevent logging from restarting.
     *          For example, AP_Logger_MAVLink may receive another start packet
     *          from a ground station and resume logging immediately.
     * 
     * @note Pure virtual - must be implemented by all backends
     * @warning Any buffered data must be flushed to prevent data loss
     */
    virtual void stop_logging(void) = 0;
    
    /**
     * @brief Asynchronously stop logging
     * 
     * @details Initiates asynchronous shutdown of logging. Useful for backends
     *          with slow I/O operations (e.g., SD card flush) to avoid blocking
     *          vehicle code. Status can be checked via logging_started().
     *          
     *          Default implementation calls synchronous stop_logging().
     * 
     * @note Virtual - backends can override for asynchronous shutdown
     * @see logging_started()
     */
    virtual void stop_logging_async(void) { stop_logging(); }

    /**
     * @brief Fill a log_Format packet from LogStructure definition
     * 
     * @details Converts internal LogStructure to wire-format log_Format packet
     *          for writing to log. Format messages define message structure
     *          and are written at log start to enable decoding.
     * 
     * @param[in] structure Pointer to LogStructure definition
     * @param[out] pkt log_Format packet to populate
     */
    void Fill_Format(const struct LogStructure *structure, struct log_Format &pkt);
    
    /**
     * @brief Fill a log_Format_Units packet from LogStructure definition
     * 
     * @details Converts internal LogStructure unit information to wire-format
     *          log_Format_Units packet. Unit messages define the units and
     *          multipliers for each field in a message type.
     * 
     * @param[in] s Pointer to LogStructure definition
     * @param[out] pkt log_Format_Units packet to populate
     */
    void Fill_Format_Units(const struct LogStructure *s, struct log_Format_Units &pkt);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    /**
     * @brief Flush all buffered data to storage
     * 
     * @details Forces immediate write of all pending log data. Currently only
     *          supported by AP_Logger_File backend. Used in simulation and
     *          testing to ensure data is on disk before analysis.
     *          
     *          Default implementation does nothing.
     * 
     * @note Virtual - only File backend implements this
     * @warning May cause significant delay if large buffers need flushing
     */
    virtual void flush(void) { }
#endif

    /**
     * @brief Handle MAVLink remote log block status message
     * 
     * @details Processes REMOTE_LOG_BLOCK_STATUS messages from ground station
     *          during log download over MAVLink. Only relevant for MAVLink backend.
     *          
     *          Default implementation does nothing (non-MAVLink backends ignore).
     * 
     * @param[in] link Reference to GCS_MAVLINK instance that received message
     * @param[in] msg Received MAVLink message containing block status
     * 
     * @note Virtual - only MAVLink backend implements this
     */
    virtual void remote_log_block_status_msg(const class GCS_MAVLINK &link,
                                             const mavlink_message_t &msg) { }

    /**
     * @brief Perform periodic maintenance tasks
     * 
     * @details Called regularly from main loop to perform backend housekeeping:
     *          - Check for log rotation conditions
     *          - Update statistics
     *          - Dispatch rate-specific tasks (1Hz, 10Hz, fullrate)
     *          
     *          Calls periodic_1Hz(), periodic_10Hz(), and periodic_fullrate()
     *          at appropriate intervals based on elapsed time.
     * 
     * @note Virtual - backends can override to add tasks, but should call base implementation
     * @see periodic_1Hz(), periodic_10Hz(), periodic_fullrate()
     */
   virtual void periodic_tasks();

    /**
     * @brief Get number of registered log message types
     * 
     * @return uint8_t Count of LogStructure entries
     */
    uint8_t num_types() const;
    
    /**
     * @brief Get LogStructure definition for a message type
     * 
     * @param[in] structure Index of LogStructure (not message ID)
     * @return const struct LogStructure* Pointer to structure definition, or nullptr if invalid index
     */
    const struct LogStructure *structure(uint8_t structure) const;

    /**
     * @brief Get number of registered unit types
     * 
     * @return uint8_t Count of UnitStructure entries
     */
    uint8_t num_units() const;
    
    /**
     * @brief Get UnitStructure definition for a unit
     * 
     * @param[in] unit Index of UnitStructure
     * @return const struct UnitStructure* Pointer to unit definition, or nullptr if invalid index
     */
    const struct UnitStructure *unit(uint8_t unit) const;

    /**
     * @brief Get number of registered multiplier types
     * 
     * @return uint8_t Count of MultiplierStructure entries
     */
    uint8_t num_multipliers() const;
    
    /**
     * @brief Get MultiplierStructure definition for a multiplier
     * 
     * @param[in] multiplier Index of MultiplierStructure
     * @return const struct MultiplierStructure* Pointer to multiplier definition, or nullptr if invalid index
     */
    const struct MultiplierStructure *multiplier(uint8_t multiplier) const;

    /**
     * @brief Write entire mission plan to log
     * 
     * @return true if mission logged successfully
     * @return false if logging failed
     */
    bool Write_EntireMission();
    
    /**
     * @brief Write a single rally point to log
     * 
     * @param[in] total Total number of rally points
     * @param[in] sequence Sequence number of this rally point (0-based)
     * @param[in] rally_point Rally point location and configuration
     * @return true if rally point logged successfully
     */
    bool Write_RallyPoint(uint8_t total,
                          uint8_t sequence,
                          const class RallyLocation &rally_point);
    
    /**
     * @brief Write all rally points to log
     * 
     * @return true if all rally points logged successfully
     */
    bool Write_Rally();
    
#if HAL_LOGGER_FENCE_ENABLED
    /**
     * @brief Write a single fence point to log
     * 
     * @param[in] total Total number of fence points
     * @param[in] sequence Sequence number of this fence point (0-based)
     * @param[in] fence_point Fence point location and type
     * @return true if fence point logged successfully
     */
    bool Write_FencePoint(uint8_t total, uint8_t sequence, const class AC_PolyFenceItem &fence_point);
    
    /**
     * @brief Write all fence points to log
     * 
     * @return true if all fence points logged successfully
     */
    bool Write_Fence();
#endif
    
    /**
     * @brief Write a format definition message to log
     * 
     * @details Writes FMT message defining structure of a message type.
     *          Must be written before first instance of that message type.
     * 
     * @param[in] structure Pointer to LogStructure to define
     * @return true if format message logged successfully
     */
    bool Write_Format(const struct LogStructure *structure);
    
    /**
     * @brief Check if format has been emitted for a message type
     * 
     * @param[in] a_type Message type to check
     * @return true if FMT message has been written for this type
     * @return false if FMT message not yet written
     */
    bool have_emitted_format_for_type(LogMessages a_type) const {
        return _formats_written.get(uint8_t(a_type));
    }
    
    /**
     * @brief Write a text message to log
     * 
     * @param[in] message Null-terminated string message
     * @return true if message logged successfully
     */
    bool Write_Message(const char *message);
    
    /**
     * @brief Write a formatted text message to log
     * 
     * @param[in] fmt printf-style format string
     * @param[in] ... Variable arguments for format string
     * @return true if message logged successfully
     */
    bool Write_MessageF(const char *fmt, ...);
    
    /**
     * @brief Write a mission command to log
     * 
     * @param[in] mission Reference to mission object
     * @param[in] cmd Mission command to log
     * @param[in] id Log message type ID (CMD or MSCL)
     * @return true if command logged successfully
     */
    bool Write_Mission_Cmd(const AP_Mission &mission,
                           const AP_Mission::Mission_Command &cmd,
                           LogMessages id);
    
    /**
     * @brief Write mode change to log
     * 
     * @param[in] mode New mode number
     * @param[in] reason Reason for mode change
     * @return true if mode change logged successfully
     */
    bool Write_Mode(uint8_t mode, const ModeReason reason);
    
    /**
     * @brief Write parameter value to log
     * 
     * @param[in] name Parameter name string
     * @param[in] value Current parameter value
     * @param[in] default_val Default value of parameter
     * @return true if parameter logged successfully
     */
    bool Write_Parameter(const char *name, float value, float default_val);
    
    /**
     * @brief Write AP_Param parameter to log
     * 
     * @param[in] ap Pointer to AP_Param object
     * @param[in] token Parameter token from enumeration
     * @param[in] type Variable type of parameter
     * @param[in] default_val Default value of parameter
     * @return true if parameter logged successfully
     */
    bool Write_Parameter(const AP_Param *ap,
                             const AP_Param::ParamToken &token,
                             enum ap_var_type type,
                             float default_val);
    
    /**
     * @brief Write version information to log
     * 
     * @details Logs firmware version, board type, and build information
     * 
     * @return true if version info logged successfully
     */
    bool Write_VER();

    /**
     * @brief Get count of dropped messages
     * 
     * @details Returns cumulative count of log messages that were dropped
     *          due to buffer full, rate limiting, or backend busy conditions.
     *          Used for diagnostics and logging statistics.
     * 
     * @return uint32_t Number of messages dropped since logging started
     */
    uint32_t num_dropped(void) const {
        return _dropped;
    }

    /**
     * @brief Write format definition if not already written
     * 
     * @details Ensures FMT message for specified type has been written to log.
     *          Uses _formats_written bitmask to track which formats have been
     *          emitted to avoid duplicates.
     * 
     * @param[in] msg_type Message type ID to emit format for
     * 
     * @return true if format has been written (either previously or just now)
     * @return false if format write failed
     * 
     * @note Must be called before first instance of any message type
     */
    bool Write_Emit_FMT(uint8_t msg_type);

    /**
     * @brief Safely emit format definition
     * 
     * @details Wrapper around Write_Emit_FMT that doesn't return status.
     *          Used when format emission is best-effort but failure is not critical.
     * 
     * @param[in] msg_type Message type ID to emit format for
     */
    void Safe_Write_Emit_FMT(uint8_t msg_type);

    /**
     * @brief Write a log message with variable arguments
     * 
     * @details Low-level write interface that formats and writes a log message.
     *          Takes va_list of arguments matching the LogStructure format string
     *          for the specified message type.
     *          
     *          **Message Formatting**: Uses LogStructure format string to pack
     *          arguments into binary log format. Format string defines field
     *          types and packing order.
     *          
     *          **Priority Handling**: is_critical bypasses rate limiting and
     *          reserves buffer space for important events.
     *          
     *          **Streaming Mode**: is_streaming uses rate-based limiting for
     *          high-frequency sensor data.
     * 
     * @param[in] msg_type Message type ID from LogMessages enum
     * @param[in] arg_list va_list of arguments matching LogStructure format
     * @param[in] is_critical true for critical messages with guaranteed buffer
     * @param[in] is_streaming true for streaming messages with rate-based limiting
     * 
     * @return true if message was queued for writing
     * @return false if message was dropped
     * 
     * @note Emits format message automatically if not already done
     * @warning arg_list must match format string or corruption will occur
     */
    bool Write(uint8_t msg_type, va_list arg_list, bool is_critical=false, bool is_streaming=false);

    /**
     * @brief Check if logging is enabled
     * 
     * @details Reports whether logging backend is enabled in configuration.
     *          Used for MAVLink status reporting.
     * 
     * @return true if logging backend is enabled
     * @return false if logging backend is disabled
     * 
     * @note Virtual - backends can override status reporting
     */
    virtual bool logging_enabled() const;
    
    /**
     * @brief Check if logging has failed
     * 
     * @details Reports whether backend is in a failed state (e.g., SD card
     *          removed, DataFlash chip error). Used for MAVLink status reporting
     *          and arming checks.
     * 
     * @return true if logging backend has failed
     * @return false if logging backend is operational
     * 
     * @note Pure virtual - must be implemented by all backends
     */
    virtual bool logging_failed() const = 0;

    /**
     * @brief Check if backend is ready for EKF startup
     * 
     * @details EKF initialization generates important log data that must not be
     *          lost. This method verifies that logging is active and ready before
     *          allowing EKF to start, ensuring EKF init messages are captured.
     * 
     * @return true if EKF can start (logging ready to capture EKF init data)
     * @return false if EKF should wait for logging to be ready
     * 
     * @note Critical for ensuring EKF initialization data is logged
     */
    bool allow_start_ekf() const;

    /**
     * @brief Notification that vehicle has disarmed
     * 
     * @details Called when vehicle disarms. Backends can use this to:
     *          - Rotate logs after flight
     *          - Flush pending data
     *          - Reduce logging rate
     *          - Update statistics
     *          
     *          Default implementation handles rate limiter updates.
     * 
     * @note Virtual - backends can override to add disarm handling
     */
    virtual void vehicle_was_disarmed();

    /**
     * @brief Write unit definition to log
     * 
     * @details Writes UNIT message defining a unit type. Called during log
     *          initialization to provide unit metadata for log analysis.
     * 
     * @param[in] s Pointer to UnitStructure definition
     * @return true if unit definition logged successfully
     */
    bool Write_Unit(const struct UnitStructure *s);
    
    /**
     * @brief Write multiplier definition to log
     * 
     * @details Writes MULT message defining a multiplier. Called during log
     *          initialization to provide multiplier metadata for log analysis.
     * 
     * @param[in] s Pointer to MultiplierStructure definition
     * @return true if multiplier definition logged successfully
     */
    bool Write_Multiplier(const struct MultiplierStructure *s);
    
    /**
     * @brief Write format units metadata to log
     * 
     * @details Writes FMTU message defining units and multipliers for each
     *          field in a message type. Enables proper unit display in log
     *          analysis tools.
     * 
     * @param[in] structure Pointer to LogStructure to write units for
     * @return true if units metadata logged successfully
     */
    bool Write_Format_Units(const struct LogStructure *structure);

    /**
     * @brief IO timer callback for asynchronous operations
     * 
     * @details Called from dedicated IO thread for backends that perform
     *          asynchronous I/O operations. File backends use this to:
     *          - Flush buffers to disk
     *          - Perform background writes
     *          - Update directory structures
     *          
     *          Default implementation does nothing (synchronous backends).
     * 
     * @note Virtual - only backends with IO threads implement this
     * @warning Must be thread-safe and use semaphores to protect shared state
     */
    virtual void io_timer(void) {}

protected:

    AP_Logger &_front;  ///< Reference to AP_Logger frontend for configuration and vehicle state

    /**
     * @brief Periodic tasks called at 10Hz
     * 
     * @details Performs maintenance operations that need 10Hz update rate:
     *          - Statistics updates
     *          - Buffer monitoring
     *          - Medium-frequency checks
     *          
     *          Called automatically by periodic_tasks() at approximately 10Hz.
     * 
     * @param[in] now Current time in milliseconds
     * 
     * @note Virtual - backends can override to add 10Hz tasks
     */
    virtual void periodic_10Hz(const uint32_t now);
    
    /**
     * @brief Periodic tasks called at 1Hz
     * 
     * @details Performs maintenance operations that need 1Hz update rate:
     *          - Log statistics reporting
     *          - Storage health checks
     *          - Low-frequency diagnostics
     *          
     *          Called automatically by periodic_tasks() at approximately 1Hz.
     * 
     * @note Virtual - backends can override to add 1Hz tasks
     */
    virtual void periodic_1Hz();
    
    /**
     * @brief Periodic tasks called at full main loop rate
     * 
     * @details Performs maintenance operations that need high-rate updates:
     *          - Push buffered data to backend
     *          - Handle urgent operations
     *          - High-frequency state updates
     *          
     *          Called automatically by periodic_tasks() on every invocation
     *          (typically 400Hz or main loop rate).
     * 
     * @note Virtual - backends can override to add fullrate tasks
     * @warning Must complete quickly (<100us) to avoid impacting main loop timing
     */
    virtual void periodic_fullrate();

    /**
     * @brief Determine if message should be logged based on vehicle state
     * 
     * @details Checks vehicle logging state and message priority to decide
     *          if message should be written. Critical messages bypass some
     *          checks to ensure they are always logged.
     * 
     * @param[in] is_critical true if message is critical priority
     * 
     * @return true if message should be logged based on current vehicle state
     * @return false if logging is disabled or conditions don't permit logging
     */
    bool ShouldLog(bool is_critical);
    
    /**
     * @brief Check if backend is ready for write operations
     * 
     * @details Backend-specific check that write operations can proceed:
     *          - File backend: Filesystem mounted and file open
     *          - Block backend: DataFlash chip responding
     *          - MAVLink backend: Client connected and ready
     * 
     * @return true if backend can accept write operations
     * @return false if backend is not ready
     * 
     * @note Pure virtual - must be implemented by all backends
     */
    virtual bool WritesOK() const = 0;
    
    /**
     * @brief Check if it's appropriate to start a new log
     * 
     * @details Verifies conditions for log rotation:
     *          - Backend is initialized
     *          - Previous log properly closed
     *          - Storage has sufficient space
     *          
     *          Default checks basic conditions; backends can override for
     *          additional checks.
     * 
     * @return true if new log can be started
     * @return false if conditions not met for new log
     * 
     * @note Virtual - backends can override to add specific checks
     */
    virtual bool StartNewLogOK() const;

    /**
     * @brief Start logging when preparing for arming
     * 
     * @details Called by PrepForArming() to actually start logging.
     *          Default implementation calls start_new_log().
     *          Backends can override to implement custom pre-arm logging setup.
     * 
     * @note Virtual - backends can override pre-arm logging behavior
     */
    virtual void PrepForArming_start_logging(void) {
        start_new_log();
    }

    /**
     * @brief Write additional startup messages to log
     * 
     * @details Continues writing startup messages (formats, parameters, etc.)
     *          that are queued by the startup message writer. Called repeatedly
     *          until all startup messages are written.
     *          
     *          Startup messages are written incrementally to avoid blocking
     *          vehicle initialization with large parameter dumps.
     * 
     * @note Virtual - backends can override startup message handling
     */
    virtual void WriteMoreStartupMessages();
    
    /**
     * @brief Push buffered log blocks to backend storage
     * 
     * @details Called periodically to transfer buffered log data to backend
     *          storage. Allows backends to batch writes for efficiency.
     * 
     * @note Virtual - backends override to implement buffer draining
     */
    virtual void push_log_blocks();

    LoggerMessageWriter_DFLogStart *_startup_messagewriter;  ///< Writer for startup messages (formats, params)
    bool _writing_startup_messages;  ///< True while startup messages are being written

    uint16_t _cached_oldest_log;  ///< Cached log number of oldest log to avoid repeated searches

    uint32_t _dropped;  ///< Count of dropped messages due to buffer full or rate limiting
    bool _rotate_pending;  ///< True if log rotation should occur when logging next stops

    /**
     * @brief Reset variables when starting a new log
     * 
     * @details Called at start of new log to reset per-log state:
     *          - Clear dropped message counter
     *          - Reset format emission tracking
     *          - Initialize statistics
     *          
     *          Backends should call this from their start_new_log() implementation.
     * 
     * @note Virtual - backends can override to reset additional state
     */
    virtual void start_new_log_reset_variables();
    
    /**
     * @brief Convert list entry index to log number
     * 
     * @details Translates 0-based list_entry (position in log list) to actual
     *          log number stored on backend. Accounts for backend-specific
     *          numbering schemes and gaps in log sequences.
     * 
     * @param[in] list_entry 0-based index into list of logs (0 = oldest)
     * @return uint16_t Actual log number in storage
     */
    uint16_t log_num_from_list_entry(const uint16_t list_entry);

    /**
     * @brief Calculate reserved buffer space for critical messages
     * 
     * @details Determines how much buffer space to reserve for critical messages
     *          to ensure important events are always logged even when buffer is
     *          near full. Currently reserves 1KB or entire buffer if smaller.
     * 
     * @param[in] bufsize Total buffer size in bytes
     * @return uint32_t Reserved space for critical messages in bytes
     * 
     * @note If bufsize < 1KB, entire buffer is reserved (only critical messages)
     */
    uint32_t critical_message_reserved_space(uint32_t bufsize) const {
        // possibly make this a proportional to buffer size?
        uint32_t ret = 1024;
        if (ret > bufsize) {
            // in this case you will only get critical messages
            ret = bufsize;
        }
        return ret;
    };
    
    /**
     * @brief Calculate reserved buffer space for non-messagewriter messages
     * 
     * @details Determines buffer space to reserve for regular log messages
     *          while startup messagewriter is active. Ensures messagewriter
     *          can complete while allowing some regular logging. Currently
     *          reserves 1KB or 0 if buffer very small.
     * 
     * @param[in] bufsize Total buffer size in bytes
     * @return uint32_t Reserved space for non-messagewriter messages in bytes
     * 
     * @note If bufsize <= 1KB, returns 0 (messagewriter uses entire buffer)
     */
    uint32_t non_messagewriter_message_reserved_space(uint32_t bufsize) const {
        // possibly make this a proportional to buffer size?
        uint32_t ret = 1024;
        if (ret >= bufsize) {
            // need to allow messages out from the messagewriters.  In
            // this case while you have a messagewriter you won't get
            // any other messages.  This should be a corner case!
            ret = 0;
        }
        return ret;
    };

    /**
     * @brief Backend-specific implementation of prioritised write
     * 
     * @details Pure virtual method that backends must implement to perform
     *          actual storage I/O. This is where backend-specific write logic
     *          lives:
     *          
     *          - File backend: Append to log file, manage buffers
     *          - Block backend: Write to DataFlash blocks, update directory
     *          - MAVLink backend: Queue for transmission to ground station
     *          
     *          **Critical vs Non-Critical**:
     *          - Critical messages get guaranteed buffer space
     *          - Non-critical messages may be dropped if buffer pressure high
     *          
     *          **Thread Safety**: Must be thread-safe if backend uses IO thread.
     *          Use semaphores to protect shared state.
     *          
     *          **Performance**: Called at high rate (400Hz+). Must complete
     *          quickly, ideally just copying to buffer for asynchronous I/O.
     * 
     * @param[in] pBuffer Pointer to formatted log message data
     * @param[in] size Size of message data in bytes
     * @param[in] is_critical true if message has critical priority
     * 
     * @return true if message was accepted by backend (queued or written)
     * @return false if message was rejected (buffer full, backend error)
     * 
     * @note Pure virtual - MUST be implemented by all backends
     * @warning Must complete in <1ms to avoid impacting flight performance
     * @warning Must be thread-safe if backend uses asynchronous I/O
     */
    virtual bool _WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical) = 0;

    bool _initialised;  ///< True if Init() has been called successfully

    /**
     * @brief Gather statistics about write operation
     * 
     * @details Records write statistics for performance monitoring:
     *          - Bytes written
     *          - Buffer space remaining
     *          - Min/max/mean buffer utilization
     * 
     * @param[in] bytes_written Number of bytes written in this operation
     * @param[in] space_remaining Buffer space remaining after write
     */
    void df_stats_gather(uint16_t bytes_written, uint32_t space_remaining);
    
    /**
     * @brief Write accumulated statistics to log
     * 
     * @details Writes DSF (DataFlash Statistics) message containing
     *          write performance metrics. Called periodically to log
     *          buffer utilization and write patterns.
     */
    void df_stats_log();
    
    /**
     * @brief Clear accumulated statistics
     * 
     * @details Resets statistics counters. Called after logging statistics
     *          to start fresh accumulation period.
     */
    void df_stats_clear();

    AP_Logger_RateLimiter *rate_limiter;  ///< Pointer to rate limiter instance for this backend

private:
    /**
     * @struct df_stats
     * @brief Statistics structure for DataFlash write performance
     * 
     * @details Accumulates write statistics over a period for performance
     *          monitoring and diagnostics. Written to log as DSF message.
     */
    struct df_stats {
        uint16_t blocks;           ///< Number of blocks written in period
        uint32_t bytes;            ///< Total bytes written in period
        uint32_t buf_space_min;    ///< Minimum buffer space remaining (tightest point)
        uint32_t buf_space_max;    ///< Maximum buffer space remaining (most free)
        uint32_t buf_space_sigma;  ///< Sum of buffer space samples (for mean calculation)
    };
    struct df_stats stats;  ///< Current statistics accumulator

    uint32_t _last_periodic_1Hz;   ///< Timestamp of last 1Hz periodic call
    uint32_t _last_periodic_10Hz;  ///< Timestamp of last 10Hz periodic call
    bool have_logged_armed;        ///< True if we've logged an armed state message

    /**
     * @brief Write statistics to log file
     * 
     * @details Writes DSF (DataFlash Statistics) message with accumulated
     *          performance metrics. Internal implementation of df_stats_log().
     * 
     * @param[in] _stats Statistics structure to write
     */
    void Write_AP_Logger_Stats_File(const struct df_stats &_stats);
    
    /**
     * @brief Validate write buffer before processing
     * 
     * @details Performs sanity checks on write buffer:
     *          - Size is reasonable
     *          - Buffer pointer is valid
     *          - Message type is recognized
     *          
     *          Called in debug builds to catch corruption early.
     * 
     * @param[in] pBuffer Pointer to write buffer
     * @param[in] size Size of buffer
     */
    void validate_WritePrioritisedBlock(const void *pBuffer, uint16_t size);

    /**
     * @brief Extract message type from write buffer
     * 
     * @details Parses message header to determine message type ID.
     *          Used for format emission and validation.
     * 
     * @param[in] pBuffer Pointer to message buffer
     * @param[in] size Size of message buffer
     * @param[out] type Extracted message type
     * 
     * @return true if message type extracted successfully
     * @return false if buffer invalid or type couldn't be determined
     */
    bool message_type_from_block(const void *pBuffer, uint16_t size, LogMessages &type) const;
    
    /**
     * @brief Ensure format message has been emitted for message buffer
     * 
     * @details Checks if format has been written for the message type in
     *          pBuffer, and emits format if needed. Ensures log can be decoded.
     * 
     * @param[in] pBuffer Pointer to message buffer
     * @param[in] size Size of message buffer
     * 
     * @return true if format already emitted or successfully emitted now
     * @return false if format emission failed
     */
    bool ensure_format_emitted(const void *pBuffer, uint16_t size);
    
    /**
     * @brief Emit format message for specified type
     * 
     * @details Writes FMT message for given message type if not already written.
     *          Updates _formats_written bitmask to track emission.
     * 
     * @param[in] a_type Message type to emit format for
     * 
     * @return true if format emitted successfully or already emitted
     * @return false if format emission failed
     */
    bool emit_format_for_type(LogMessages a_type);
    
    /// Bitmask tracking which message types have had FMT messages written
    /// Indexed by message type ID (0-255), prevents duplicate format messages
    Bitmask<256> _formats_written;

};

#endif  // HAL_LOGGING_ENABLED
