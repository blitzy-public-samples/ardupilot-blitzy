/**
 * @file AP_Logger_MAVLink.h
 * @brief MAVLink-based real-time log streaming backend
 * 
 * @details This AP_Logger backend implements real-time log streaming to ground
 *          control stations via MAVLink REMOTE_LOG_DATA_BLOCK messages. Unlike
 *          storage-based backends (SD card, flash), this backend prioritizes
 *          real-time visibility over completeness, streaming log data as it's
 *          generated within MAVLink bandwidth constraints.
 * 
 *          Key characteristics:
 *          - Block-based transmission: 200-byte fixed-size blocks (MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN)
 *          - Sequence-numbered blocks for reassembly at ground station
 *          - ACK/NACK protocol with automatic retransmission
 *          - Queue-based buffer management (pending → sent → retry)
 *          - Bandwidth-constrained: typical 5-50 kbit/s vs on-board logging 1-10 Mbit/s
 * 
 * @note This backend transfers blocks of the open log file to a MAVLink client
 *       using the REMOTE_LOG_DATA_BLOCK message protocol.
 * 
 * @warning MAVLink bandwidth limitations mean not all log data may be streamed
 *          in high-data-rate scenarios. This is intended for real-time monitoring,
 *          not as a replacement for on-board logging.
 * 
 * @see AP_Logger_Backend for base logging interface
 * @see AP_Logger_MAVLinkLogTransfer for LOG_REQUEST_DATA protocol (historical log download)
 */
#pragma once

#include "AP_Logger_Backend.h"

#if HAL_LOGGING_MAVLINK_ENABLED

#include <AP_HAL/Semaphores.h>

#define DF_MAVLINK_DISABLE_INTERRUPTS 0

/**
 * @class AP_Logger_MAVLink
 * @brief Real-time log streaming backend using MAVLink protocol
 * 
 * @details This logger backend streams log data in real-time to a ground control
 *          station using MAVLink REMOTE_LOG_DATA_BLOCK messages. It manages a
 *          fixed-size pool of 200-byte data blocks through multiple queues to
 *          handle transmission, acknowledgment, and retransmission.
 * 
 *          Block Buffer Pool Management:
 *          - Fixed pool of dm_block structures (typically 8-50 blocks)
 *          - Each block: 200 bytes (MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN)
 *          - Blocks transition: free → pending → sent → [retry] → free
 *          - Sequence numbers track blocks for GCS reassembly
 * 
 *          Queue Architecture:
 *          - _blocks_free: Stack of available blocks
 *          - _blocks_pending: Queue of blocks ready to transmit
 *          - _blocks_sent: Queue of blocks awaiting acknowledgment
 *          - _blocks_retry: Queue of blocks needing retransmission
 * 
 *          Flow Control and Reliability:
 *          - ACK/NACK protocol with ground station
 *          - Automatic retransmission with exponential backoff
 *          - Configurable max blocks per periodic call (_max_blocks_per_send_blocks)
 *          - Typical throughput: 5-20 Hz × 200 bytes = 1-4 KB/s
 * 
 *          Thread Safety:
 *          - Queue operations protected by semaphore
 *          - Safe interaction with MAVLink send thread
 *          - Non-blocking writes with overflow handling
 * 
 *          Performance Characteristics:
 *          - Latency: 20-200ms typical (depends on link quality and rate)
 *          - Bandwidth: 5-50 kbit/s (MAVLink limited) vs 1-10 Mbit/s on-board
 *          - Packet loss handling: automatic retries with timeout
 * 
 *          DMS (Data Management System) Integration:
 *          - Statistics tracking: free/pending/sent/retry queue depths
 *          - Resend counters and min/max queue sizes
 *          - Logged periodically for streaming health monitoring
 * 
 * @note This backend does not provide historical log download functionality.
 *       For downloading stored logs, see AP_Logger_MAVLinkLogTransfer which
 *       implements the LOG_REQUEST_DATA protocol.
 * 
 * @warning This is a real-time streaming solution with trade-offs:
 *          - Real-time visibility: See logs as they happen
 *          - Incompleteness: May drop data under bandwidth constraints
 *          - Bandwidth efficiency: Uses precious telemetry bandwidth
 *          - Not for certification: Use on-board logging for flight records
 */
class AP_Logger_MAVLink : public AP_Logger_Backend
{
public:
    /**
     * @brief Construct MAVLink streaming logger backend
     * 
     * @param[in] front   Reference to main AP_Logger frontend
     * @param[in] writer  Pointer to DFLogStart message writer for log headers
     */
    AP_Logger_MAVLink(class AP_Logger &front, LoggerMessageWriter_DFLogStart *writer);

    /**
     * @brief Factory method to create MAVLink logger backend
     * 
     * @details Allocates and constructs an AP_Logger_MAVLink instance using
     *          NEW_NOTHROW for safe dynamic allocation in embedded environment.
     * 
     * @param[in] front  Reference to main AP_Logger frontend
     * @param[in] ls     Pointer to DFLogStart message writer
     * 
     * @return Pointer to newly created backend, or nullptr if allocation failed
     */
    static AP_Logger_Backend  *probe(AP_Logger &front,
                                     LoggerMessageWriter_DFLogStart *ls) {
        return NEW_NOTHROW AP_Logger_MAVLink(front, ls);
    }

    /**
     * @brief Initialize MAVLink logging backend
     * 
     * @details Sets up block buffer pool, initializes queues, and prepares
     *          the backend for streaming. Called during AP_Logger initialization.
     */
    void Init() override;

    /**
     * @brief Check if logging has started
     * 
     * @details Returns initialization status. This backend is always "logging"
     *          once initialized, but in practice it discards data until a
     *          MAVLink client connects and begins receiving log blocks.
     * 
     * @return true if backend is initialized and ready
     * 
     * @note This prevents the vehicle from calling start_new_log() repeatedly.
     *       Data is buffered/discarded until a client connects.
     */
    bool logging_started() const override { return _initialised; }

    /**
     * @brief Stop log streaming
     * 
     * @details Stops streaming log data to the connected MAVLink client.
     *          Flushes pending blocks and releases resources.
     */
    void stop_logging() override;

    /**
     * @brief Write a block of data to the streaming log
     * 
     * @details Writes log data into the current dm_block buffer. When a block
     *          fills (200 bytes), it's enqueued to _blocks_pending for transmission.
     *          This method handles buffer allocation and queue management.
     * 
     * @param[in] pBuffer      Pointer to data to write
     * @param[in] size         Number of bytes to write
     * @param[in] is_critical  Priority flag (currently unused for streaming)
     * 
     * @return true if data was written successfully, false if buffers are full
     * 
     * @note This is called at high frequency (100-400Hz) from logging tasks.
     *       If no free blocks are available, data may be discarded.
     * 
     * @warning Under high data rates or poor links, buffer exhaustion can occur,
     *          resulting in log data loss. This is expected behavior for streaming.
     */
    bool _WritePrioritisedBlock(const void *pBuffer, uint16_t size,
                               bool is_critical) override;

    /**
     * @brief Check if "card" (virtual storage) is available
     * 
     * @details For MAVLink streaming, we always report storage available since
     *          we're not writing to physical media. This allows logging to proceed.
     * 
     * @return Always true for streaming backend
     */
    bool CardInserted(void) const override { return true; }

    /**
     * @brief Erase all logs (no-op for streaming backend)
     * 
     * @details MAVLink streaming doesn't store logs, so erase is not applicable.
     *          This method exists to satisfy the AP_Logger_Backend interface.
     */
    void EraseAll() override {}

    /**
     * @brief Prepare for vehicle arming (no-op for streaming backend)
     * 
     * @details Streaming backend requires no special preparation for arming.
     *          Logging continues normally through armed/disarmed transitions.
     */
    void PrepForArming() override {}

    /**
     * @brief Find last log number (not applicable for streaming)
     * 
     * @return Always 0 (streaming backend doesn't maintain log history)
     */
    uint16_t find_last_log(void) override { return 0; }
    
    /**
     * @brief Get log boundaries (not applicable for streaming)
     * 
     * @param[in]  log_num    Log number (unused)
     * @param[out] start_page Starting page (unused)
     * @param[out] end_page   Ending page (unused)
     * 
     * @note Streaming backend doesn't store logs, so boundaries are meaningless.
     */
    void get_log_boundaries(uint16_t log_num, uint32_t & start_page, uint32_t & end_page) override {}
    
    /**
     * @brief Get log information (not applicable for streaming)
     * 
     * @param[in]  log_num  Log number (unused)
     * @param[out] size     Log size in bytes (unused)
     * @param[out] time_utc Log timestamp UTC (unused)
     */
    void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc) override {}
    
    /**
     * @brief Get log data (not applicable for streaming)
     * 
     * @param[in]  log_num Log number (unused)
     * @param[in]  page    Page number (unused)
     * @param[in]  offset  Offset within page (unused)
     * @param[in]  len     Length to read (unused)
     * @param[out] data    Output buffer (unused)
     * 
     * @return Always 0 (no data available)
     * 
     * @note For historical log download, see AP_Logger_MAVLinkLogTransfer
     */
    int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) override { return 0; }
    
    /**
     * @brief End log transfer (no-op for streaming)
     */
    void end_log_transfer() override { };
    
    /**
     * @brief Get number of stored logs
     * 
     * @return Always 0 (streaming backend doesn't store logs)
     */
    uint16_t get_num_logs(void) override { return 0; }

    /**
     * @brief Handle REMOTE_LOG_BLOCK_STATUS message from ground station
     * 
     * @details Processes ACK/NACK responses from the ground station for transmitted
     *          log blocks. ACK messages cause blocks to be moved from _blocks_sent
     *          to _blocks_free. NACK messages trigger retransmission by moving
     *          blocks to _blocks_retry queue.
     * 
     *          ACK/NACK Protocol:
     *          - GCS sends REMOTE_LOG_BLOCK_STATUS with sequence numbers
     *          - ACK: Block received successfully, free the buffer
     *          - NACK: Block lost or corrupted, schedule for retransmission
     *          - Updates _last_response_time for timeout monitoring
     * 
     * @param[in] link  Reference to MAVLink connection that received the message
     * @param[in] msg   MAVLink message containing block status
     * 
     * @see handle_ack() for ACK processing logic
     * @see handle_retry() for retransmission scheduling
     * @see do_resends() for periodic retransmission execution
     */
    void remote_log_block_status_msg(const GCS_MAVLINK &link, const mavlink_message_t& msg) override;
    
    /**
     * @brief Handle vehicle disarm event (no-op for streaming)
     * 
     * @details Streaming backend continues logging through disarm events.
     *          No special cleanup or state changes required.
     */
    void vehicle_was_disarmed() override {}

protected:

    /**
     * @brief Push pending log blocks to MAVLink send queue
     * 
     * @details Called periodically (typically at 10Hz) to transmit pending blocks
     *          and handle retransmissions. Respects _max_blocks_per_send_blocks
     *          limit to avoid monopolizing MAVLink bandwidth in a single call.
     * 
     *          Transmission priority:
     *          1. Blocks in _blocks_retry (retransmissions)
     *          2. Blocks in _blocks_pending (new data)
     * 
     * @note Rate-limited to prevent excessive MAVLink traffic. Typical config
     *       allows 2-4 blocks per call, yielding 20-80 blocks/second max.
     * 
     * @see do_resends() for retry timeout handling
     */
    void push_log_blocks() override;
    
    /**
     * @brief Check if backend can accept writes
     * 
     * @details Returns true if there are free blocks available in the buffer pool.
     *          When false, _WritePrioritisedBlock() will discard data.
     * 
     * @return true if free blocks are available, false if buffer pool exhausted
     * 
     * @warning Returning false indicates buffer exhaustion due to transmission
     *          backlog or poor link quality. Data loss will occur.
     */
    bool WritesOK() const override;

private:

    /**
     * @struct dm_block
     * @brief Data block structure for MAVLink log streaming
     * 
     * @details Represents a single 200-byte block of log data with metadata for
     *          transmission tracking and queue management. Blocks are managed in
     *          a fixed-size pool and transition through multiple queues during
     *          their lifecycle.
     * 
     *          Block Lifecycle:
     *          1. Allocated from _blocks_free stack
     *          2. Filled with log data in _WritePrioritisedBlock()
     *          3. Moved to _blocks_pending when full
     *          4. Transmitted and moved to _blocks_sent (awaiting ACK)
     *          5a. ACK received → moved back to _blocks_free
     *          5b. NACK/timeout → moved to _blocks_retry → retransmit → back to _blocks_sent
     * 
     * @note MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN is fixed at 200 bytes
     *       by MAVLink protocol definition.
     */
    struct dm_block {
        uint32_t seqno;     ///< Sequence number for block ordering and ACK/NACK matching
        uint8_t buf[MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN]; ///< 200-byte data buffer
        uint32_t last_sent; ///< Timestamp (milliseconds) of last transmission attempt
        struct dm_block *next; ///< Pointer to next block in queue/stack (linked list)
    };
    /**
     * @brief Transmit a single log block via MAVLink
     * 
     * @param[in] block  Reference to block to transmit
     * 
     * @return true if block was queued for transmission, false otherwise
     * 
     * @note Sets block.last_sent timestamp for retry timeout tracking
     */
    bool send_log_block(struct dm_block &block);
    
    /**
     * @brief Handle ACK for successfully received block
     * 
     * @details Removes acknowledged block from _blocks_sent queue and returns
     *          it to _blocks_free stack for reuse.
     * 
     * @param[in] link  MAVLink connection that sent the ACK
     * @param[in] msg   Original status message
     * @param[in] seqno Sequence number being acknowledged
     */
    void handle_ack(const GCS_MAVLINK &link, const mavlink_message_t &msg, uint32_t seqno);
    
    /**
     * @brief Schedule block for retransmission
     * 
     * @details Moves block from _blocks_sent to _blocks_retry queue for
     *          retransmission. Called when NACK received or timeout occurs.
     * 
     *          Retry Strategy:
     *          - Exponential backoff: retry delays increase with attempt count
     *          - Retry limit enforcement: blocks dropped after max attempts
     *          - Priority handling: retries sent before new blocks
     * 
     * @param[in] block_num  Sequence number of block to retry
     * 
     * @warning Excessive retries indicate poor link quality. After retry limit,
     *          blocks are dropped to prevent queue stagnation.
     */
    void handle_retry(uint32_t block_num);
    
    /**
     * @brief Process retransmission timeouts and send retry blocks
     * 
     * @details Called periodically (typically 10Hz) to check for blocks that
     *          haven't been acknowledged within timeout period. Moves timed-out
     *          blocks from _blocks_sent to _blocks_retry.
     * 
     *          Timeout Logic:
     *          - Checks last_sent timestamp against current time
     *          - Timeout threshold typically 1-5 seconds (link-dependent)
     *          - Implements exponential backoff for repeated failures
     * 
     * @param[in] now  Current timestamp in milliseconds
     * 
     * @note This implements the retransmission logic critical for reliable
     *       delivery over lossy MAVLink connections.
     * 
     * @see handle_retry() for retry queuing
     */
    void do_resends(uint32_t now);
    
    /**
     * @brief Free all allocated blocks and reset queues
     * 
     * @details Returns all blocks from all queues back to free stack.
     *          Called during cleanup or reset operations.
     */
    void free_all_blocks();

    /**
     * @struct dm_block_queue
     * @brief FIFO queue structure for managing dm_block transmission lifecycle
     * 
     * @details Implements a linked-list queue for tracking blocks through various
     *          stages of transmission. Multiple queues manage different states:
     *          - _blocks_pending: Blocks filled with data, awaiting transmission
     *          - _blocks_sent: Blocks transmitted, awaiting ACK from GCS
     *          - _blocks_retry: Blocks needing retransmission due to NACK/timeout
     * 
     *          Queue Operations:
     *          - enqueue_block(): Add block to youngest end (FIFO)
     *          - dequeue_seqno(): Remove specific block by sequence number
     *          - Maintains sent_count for statistics
     * 
     * @note _blocks_free uses a simple stack (LIFO) since order doesn't matter
     *       for free blocks, but transmission queues use FIFO for fairness.
     */
    struct dm_block_queue {
        uint32_t sent_count;        ///< Total blocks sent from this queue (statistics)
        struct dm_block *oldest;    ///< Head of queue (next to dequeue)
        struct dm_block *youngest;  ///< Tail of queue (where new blocks enqueue)
    };
    typedef struct dm_block_queue dm_block_queue_t ;
    /**
     * @brief Add block to end of queue (FIFO)
     * 
     * @param[in,out] queue  Queue to enqueue into
     * @param[in]     block  Block to add to queue
     * 
     * @note Updates youngest pointer and links block into queue
     */
    void enqueue_block(dm_block_queue_t &queue, struct dm_block *block);
    
    /**
     * @brief Check if queue contains specific block
     * 
     * @param[in] queue  Queue to search
     * @param[in] block  Block to search for
     * 
     * @return true if block is in queue, false otherwise
     */
    bool queue_has_block(dm_block_queue_t &queue, struct dm_block *block);
    
    /**
     * @brief Remove and return block with specific sequence number
     * 
     * @details Searches queue for block matching seqno, removes it from queue,
     *          and returns pointer to the block. Used for ACK/NACK processing.
     * 
     * @param[in,out] queue  Queue to search and modify
     * @param[in]     seqno  Sequence number to find
     * 
     * @return Pointer to removed block, or nullptr if not found
     */
    struct dm_block *dequeue_seqno(dm_block_queue_t &queue, uint32_t seqno);
    
    /**
     * @brief Remove block with sequence number from queue and free it
     * 
     * @param[in]     seqno  Sequence number to free
     * @param[in,out] queue  Queue to search
     * 
     * @return true if block was found and freed, false otherwise
     */
    bool free_seqno_from_queue(uint32_t seqno, dm_block_queue_t &queue);
    
    /**
     * @brief Transmit multiple blocks from specified queue
     * 
     * @details Sends up to _max_blocks_per_send_blocks from the queue.
     *          Used to process both _blocks_retry and _blocks_pending.
     * 
     * @param[in,out] queue  Queue to transmit blocks from
     * 
     * @return true if any blocks were sent, false otherwise
     */
    bool send_log_blocks_from_queue(dm_block_queue_t &queue);
    
    /**
     * @brief Count blocks in free stack
     * 
     * @param[in] stack  Head of free block stack
     * 
     * @return Number of blocks in stack
     */
    uint8_t stack_size(struct dm_block *stack);
    
    /**
     * @brief Count blocks in queue
     * 
     * @param[in] queue  Queue to count
     * 
     * @return Number of blocks in queue
     */
    uint8_t queue_size(dm_block_queue_t queue);
    
    struct dm_block *_blocks_free;      ///< Stack of available blocks (LIFO)
    dm_block_queue_t _blocks_sent;      ///< Queue of blocks awaiting ACK from GCS
    dm_block_queue_t _blocks_pending;   ///< Queue of blocks ready to transmit
    dm_block_queue_t _blocks_retry;     ///< Queue of blocks needing retransmission

    /**
     * @struct _stats
     * @brief DMS (Data Management System) statistics for streaming health monitoring
     * 
     * @details Tracks queue depths and retransmission counts to monitor MAVLink
     *          log streaming performance. Statistics are collected periodically
     *          and logged via DMS messages for ground station analysis.
     * 
     *          Monitoring Strategy:
     *          - Collect samples at 10Hz (stats_collect called from periodic_10Hz)
     *          - Track min/max/cumulative queue depths
     *          - Log aggregated statistics every 10 seconds
     *          - Reset counters after logging (stats_reset)
     * 
     *          Health Indicators:
     *          - High state_retry: Poor link quality or congestion
     *          - Low state_free: Buffer exhaustion, likely data loss
     *          - High state_sent: Slow ACKs, possible GCS processing delays
     *          - High resends: Packet loss or timeouts
     * 
     * @note Statistics are reset after logging to provide per-period metrics.
     *       Cumulative values allow averaging over collection period.
     */
    struct _stats {
        uint32_t resends;           ///< Total retransmission attempts this period
        uint8_t collection_count;   ///< Number of samples collected this period
        uint16_t state_free;        ///< Cumulative free block count across period
        uint8_t state_free_min;     ///< Minimum free blocks observed this period
        uint8_t state_free_max;     ///< Maximum free blocks observed this period
        uint16_t state_pending;     ///< Cumulative pending block count across period
        uint8_t state_pending_min;  ///< Minimum pending blocks this period
        uint8_t state_pending_max;  ///< Maximum pending blocks this period
        uint16_t state_retry;       ///< Cumulative retry block count across period
        uint8_t state_retry_min;    ///< Minimum retry blocks this period
        uint8_t state_retry_max;    ///< Maximum retry blocks this period
        uint16_t state_sent;        ///< Cumulative sent (awaiting ACK) count across period
        uint8_t state_sent_min;     ///< Minimum sent blocks this period
        uint8_t state_sent_max;     ///< Maximum sent blocks this period
    } stats;

    /**
     * @brief Check if logging is enabled
     * 
     * @return Always true (streaming backend is always enabled when active)
     * 
     * @note Used for MAVLink SYS_STATUS reporting
     */
    bool logging_enabled() const override { return true; }
    
    /**
     * @brief Check if logging has failed
     * 
     * @details Returns true if streaming has encountered unrecoverable errors.
     *          Typically indicates MAVLink connection loss or persistent failures.
     * 
     * @return true if logging has failed, false if operational
     */
    bool logging_failed() const override;

    const GCS_MAVLINK *_link;   ///< MAVLink connection to ground station client

    uint8_t _target_system_id;      ///< MAVLink system ID of ground station
    uint8_t _target_component_id;   ///< MAVLink component ID of ground station

    /**
     * @brief Maximum blocks to transmit per push_log_blocks() call
     * 
     * @details Rate-limits block transmission to prevent monopolizing MAVLink
     *          bandwidth and CPU time in a single periodic call. This balances
     *          throughput with system responsiveness.
     * 
     *          Throughput Calculation:
     *          - Each block: 200 bytes (MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN)
     *          - Call frequency: typically 10-50Hz (depends on vehicle scheduler)
     *          - Example: 2 blocks/call × 50Hz × 200 bytes = 20 KB/s maximum
     * 
     *          Tuning Considerations:
     *          - Higher values: Better throughput, more CPU per call
     *          - Lower values: Better real-time performance, lower throughput
     *          - Must be sufficient to drain log generation rate under normal conditions
     * 
     * @note push_log_blocks() is called from periodic_tasks at scheduler rate.
     *       Setting this too high can cause scheduler overruns.
     */
    const uint8_t _max_blocks_per_send_blocks;
    
    /**
     * @brief Next sequence number to assign to a block
     * 
     * @details Monotonically increasing counter used to uniquely identify each
     *          transmitted block. Ground station uses sequence numbers to:
     *          - Detect missing blocks
     *          - Reassemble blocks in correct order
     *          - Send ACK/NACK for specific blocks
     * 
     * @note Sequence numbers are never reused during a session. Wraps at UINT32_MAX.
     */
    uint32_t _next_seq_num;
    
    uint16_t _latest_block_len;         ///< Length of data in current block being filled
    uint32_t _last_response_time;       ///< Timestamp of last ACK/NACK from GCS (milliseconds)
    uint32_t _last_send_time;           ///< Timestamp of last block transmission (milliseconds)
    uint8_t _next_block_number_to_resend; ///< Index for round-robin retry processing
    bool _sending_to_client;            ///< True if actively streaming to connected client

    /**
     * @brief Write DMS (Data Management System) statistics to log
     * 
     * @details Logs aggregated statistics about MAVLink streaming health,
     *          including queue depths, retry counts, and buffer utilization.
     *          Called periodically (typically every 10 seconds) by periodic_1Hz.
     * 
     * @param[in] logger  Reference to this logger instance
     * 
     * @note DMS messages provide visibility into streaming performance for
     *       post-flight analysis and link quality assessment.
     * 
     * @see stats structure for detailed statistics fields
     */
    void Write_DMS(AP_Logger_MAVLink &logger);

    /**
     * @brief Calculate available buffer space
     * 
     * @details Returns total bytes available across all free blocks plus
     *          remaining space in current block being filled.
     * 
     * @return Available buffer space in bytes
     * 
     * @note Used by frontend to determine if writes will succeed
     */
    uint32_t bufferspace_available() override;
    
    /**
     * @brief Get remaining space in current block
     * 
     * @details Returns number of bytes available in the block currently
     *          being filled before it needs to be enqueued for transmission.
     * 
     * @return Bytes remaining in current block (0-200)
     */
    uint8_t remaining_space_in_current_block() const;
    /**
     * @name Write Buffer Management
     * @{
     */
    
    uint8_t _blockcount_free;       ///< Current number of blocks in free stack
    uint8_t _blockcount;            ///< Total number of blocks in pool (typically 8-50)
    struct dm_block *_blocks;       ///< Pointer to block pool array (allocated at Init)
    struct dm_block *_current_block; ///< Block currently being filled with log data
    
    /**
     * @brief Allocate next block for writing
     * 
     * @details Pops a block from _blocks_free stack, enqueues current block to
     *          _blocks_pending if it contains data, and returns the new block.
     * 
     * @return Pointer to next available block, or nullptr if pool exhausted
     * 
     * @warning Returns nullptr when buffers are exhausted, causing data loss
     */
    struct dm_block *next_block();
    
    /** @} */

    /**
     * @brief Periodic processing at 10Hz
     * 
     * @details Handles time-critical streaming operations:
     *          - Calls do_resends() to check for transmission timeouts
     *          - Calls push_log_blocks() to transmit pending/retry blocks
     *          - Updates statistics (stats_collect)
     * 
     * @param[in] now  Current timestamp in milliseconds
     */
    void periodic_10Hz(uint32_t now) override;
    
    /**
     * @brief Periodic processing at 1Hz
     * 
     * @details Handles lower-frequency operations:
     *          - Logs DMS statistics (stats_log every 10 seconds)
     *          - Connection health monitoring
     */
    void periodic_1Hz() override;
    
    /**
     * @brief Initialize statistics structure
     * 
     * @details Zeros all statistics counters. Called during Init().
     */
    void stats_init();
    
    /**
     * @brief Reset statistics for new collection period
     * 
     * @details Clears cumulative counters and min/max values after logging.
     *          Called after stats_log() to start fresh period.
     */
    void stats_reset();
    
    /**
     * @brief Collect statistics sample
     * 
     * @details Samples current queue depths and updates min/max/cumulative values.
     *          Called from periodic_10Hz() for 10Hz sampling rate.
     */
    void stats_collect();
    
    /**
     * @brief Log aggregated statistics via DMS message
     * 
     * @details Writes collected statistics to log and resets counters.
     *          Called every 10 seconds from periodic_1Hz().
     */
    void stats_log();
    
    uint32_t _stats_last_collected_time;  ///< Timestamp of last stats_collect() call
    uint32_t _stats_last_logged_time;     ///< Timestamp of last stats_log() call
    uint8_t mavlink_seq;                  ///< MAVLink message sequence number

    /**
     * @brief Start new log (no-op for streaming backend)
     * 
     * @details Streaming backend ignores requests to start a new log since
     *          it continuously streams data rather than creating discrete log files.
     * 
     * @note Notionally could close current session and wait for client to
     *       reconnect, but current implementation just ignores the request.
     */
    void start_new_log(void) override {
        return;
    }

    /**
     * @brief Semaphore for thread-safe queue operations
     * 
     * @details Protects queue manipulation and block transitions from race
     *          conditions between:
     *          - Main logging thread (writes, enqueues)
     *          - MAVLink send thread (transmissions)
     *          - MAVLink receive thread (ACK/NACK handling)
     * 
     * @note Critical sections should be kept short to avoid blocking MAVLink
     *       or logging operations.
     */
    HAL_Semaphore semaphore;
};

#endif // HAL_LOGGING_MAVLINK_ENABLED
