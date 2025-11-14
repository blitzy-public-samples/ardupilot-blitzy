/**
 * @file AP_Logger_Block.h
 * @brief Page-oriented block-device logging backend for SPI flash chips
 * 
 * @details This file implements the AP_Logger_Block backend for logging to 
 *          block-oriented flash storage devices (SPI/QSPI flash chips). It provides:
 *          - Page-based storage model (typically 256-2048 bytes per page)
 *          - Block/sector erase management (64KB blocks, 4KB sectors)
 *          - DMA-safe ring buffer for write operations
 *          - Wear leveling through circular log file arrangement
 *          - Log recovery and validation on startup
 * 
 *          Storage Model:
 *          - Pages: Smallest read/write unit (256-2048 bytes depending on chip)
 *          - Sectors: 4KB erase units (typically 16 pages)
 *          - Blocks: 64KB erase units (typically 16 sectors)
 *          - Each page has a PageHeader with file number and page index
 *          - Each log file starts with a FileHeader containing timestamp
 * 
 *          Thread Safety:
 *          - Main thread: start_new_log(), stop_logging(), public query methods
 *          - IO thread: write_log_page(), BufferToPage(), erase operations
 *          - Ring buffer protected by write_sem semaphore
 *          - Chip access protected by sem semaphore
 * 
 *          Performance Characteristics:
 *          - Write latency: ~1-3ms per page (chip dependent)
 *          - Sector erase: ~100-300ms
 *          - Block erase: ~500-2000ms
 *          - DMA transfers used where available for efficiency
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */
#pragma once

#include "AP_Logger_Backend.h"

#if HAL_LOGGING_BLOCK_ENABLED

#define BLOCK_LOG_VALIDATE 0

/**
 * @class AP_Logger_Block
 * @brief Block-device logging backend for SPI flash storage
 * 
 * @details Implements logging to page-oriented flash storage devices with the following architecture:
 * 
 *          Storage Organization:
 *          - Flash chip organized as: Pages → Sectors → Blocks
 *          - Pages are smallest addressable unit (256-2048 bytes)
 *          - Sectors are 4KB erase units (df_PagePerSector pages)
 *          - Blocks are 64KB erase units (df_PagePerBlock pages)
 *          - Total capacity: df_NumPages × df_PageSize
 * 
 *          Page Header Layout (struct PageHeader):
 *          - FilePage (uint32_t): Page number within current log file (starts at 1)
 *          - FileNumber (uint16_t): Log file number (1-65535)
 *          - crc (uint32_t): Optional CRC if BLOCK_LOG_VALIDATE enabled
 * 
 *          File Header Layout (struct FileHeader):
 *          - utc_secs (uint32_t): UTC timestamp when log file created
 * 
 *          Ring Buffer Semantics:
 *          - Logs written in circular fashion across flash chip
 *          - When chip fills, oldest logs overwritten automatically
 *          - df_PageAdr tracks current write position
 *          - df_FilePage tracks position within current log file
 *          - Wrap-around handled transparently via is_wrapped()
 * 
 *          DMA-Safe Buffer Design:
 *          - writebuf is ByteBuffer ring buffer for incoming data
 *          - buffer is page-aligned DMA-safe staging area
 *          - Data flows: writebuf → buffer → BufferToPage() → flash chip
 *          - Alignment ensures DMA controllers can directly access memory
 * 
 *          Wear Leveling Strategy:
 *          - Circular write pattern distributes wear across chip
 *          - No fixed log file locations - they float across chip
 *          - Erase operations performed on sector/block boundaries
 *          - Typical flash endurance: 100K erase cycles per sector
 * 
 *          Log Recovery Process:
 *          1. Init() probes flash chip and reads page/sector/block geometry
 *          2. validate_log_structure() scans chip for valid log files
 *          3. find_last_log() identifies most recent log by scanning PageHeaders
 *          4. find_last_page() determines where to resume writing
 *          5. NeedErase() checks if erase required before writing
 * 
 *          Error Handling:
 *          - chip_full flag set when no writable space available
 *          - erase_started flag indicates erase in progress
 *          - logging_failed() returns true if critical errors detected
 *          - validate_log_structure() marks corrupt regions for erase
 * 
 * @note Flash Wear Consideration: Each sector can typically handle 100,000 erase cycles.
 *       At 4KB per sector with continuous logging at 100KB/s, each sector is erased
 *       approximately once per hour. This yields ~11 years of continuous operation.
 * 
 * @warning Some methods (BufferToPage, SectorErase) block for milliseconds during flash operations.
 *          Erase operations can block for 100ms-2s depending on erase size. Use InErase() to
 *          monitor erase progress. Never call blocking operations from interrupt context.
 * 
 * @see AP_Logger_Backend - Base class interface
 * @see HAL_Semaphore - Semaphore implementation for thread safety
 * @see ByteBuffer - Ring buffer implementation
 */
class AP_Logger_Block : public AP_Logger_Backend {
public:
    /**
     * @brief Construct a block-device logger backend
     * 
     * @param[in] front Reference to main AP_Logger frontend
     * @param[in] writer Pointer to DFLogStart message writer for log file headers
     */
    AP_Logger_Block(AP_Logger &front, LoggerMessageWriter_DFLogStart *writer);

    /**
     * @brief Initialize the block-device logger backend
     * 
     * @details Performs complete initialization of the flash storage backend:
     *          1. Probes flash chip to detect presence and read JEDEC ID
     *          2. Detects page size (256, 512, 1024, or 2048 bytes)
     *          3. Detects chip capacity and calculates df_NumPages
     *          4. Calculates df_PagePerSector (typically 16 for 4KB sectors)
     *          5. Calculates df_PagePerBlock (typically 256 for 64KB blocks)
     *          6. Allocates DMA-safe buffer aligned to page size
     *          7. Allocates writebuf ring buffer for incoming data
     *          8. Calls validate_log_structure() to scan for existing logs
     *          9. Calls find_last_page() to determine write start position
     * 
     *          The initialization process is robust to:
     *          - Missing or unresponsive flash chips
     *          - Partially erased chips
     *          - Chips with corrupted log data
     *          - Power-loss during previous write operations
     * 
     *          After successful initialization:
     *          - df_PageSize, df_NumPages, df_PagePerSector, df_PagePerBlock are set
     *          - buffer points to DMA-safe page-aligned memory
     *          - writebuf is ready to accept log data
     *          - df_PageAdr points to next writable page
     * 
     * @note Called once during AP_Logger initialization on system startup
     * @note Blocks for 100-500ms during chip probing and structure validation
     * @warning Must be called before any logging operations
     * 
     * @see validate_log_structure() - Log recovery process
     * @see find_last_page() - Determines write start position
     */
    virtual void Init(void) override;
    
    /**
     * @brief Check if flash storage device is present
     * 
     * @details Platform-specific implementation checks if the flash chip
     *          is physically present and responsive. This may involve:
     *          - Checking SPI chip select line
     *          - Reading JEDEC ID
     *          - Verifying expected device ID
     * 
     * @return true if flash chip detected and responsive, false otherwise
     * 
     * @note Pure virtual - implemented by platform-specific subclasses
     * @note Called frequently - should be fast (typically < 1ms)
     */
    virtual bool CardInserted(void) const override = 0;

    /**
     * @brief Erase all log data from flash chip
     * 
     * @details Initiates complete chip erase sequence:
     *          1. Sets erase_started flag to prevent new writes
     *          2. Calls StartErase() to begin chip erase operation
     *          3. Erase proceeds asynchronously in background
     *          4. InErase() returns true while erase is in progress
     *          5. Once complete, chip is ready for new log files
     * 
     *          Erase Timing (chip dependent):
     *          - 4KB sector erase: ~100-300ms
     *          - 64KB block erase: ~500-2000ms
     *          - Full chip erase: ~10-60 seconds
     * 
     *          During Erase:
     *          - logging_failed() may return true
     *          - New log writes are deferred until erase completes
     *          - status_msg set to ERASE_COMPLETE when finished
     * 
     * @note Erase operation is asynchronous - use InErase() to check progress
     * @warning Full chip erase can take 10-60 seconds. Monitor progress via InErase()
     *          and provide user feedback. Do not power off during erase.
     * @warning All existing log data is permanently destroyed
     * 
     * @see StartErase() - Platform-specific erase initiation
     * @see InErase() - Check if erase operation is still in progress
     * @see NeedErase() - Determine if erase is required before writing
     */
    void EraseAll() override;

    // high level interface
    /**
     * @brief Find the most recent log file number
     * 
     * @details Scans flash chip PageHeaders to identify the log file with the
     *          highest FileNumber. Used during initialization and when querying
     *          available logs.
     * 
     *          Algorithm:
     *          1. Scan all pages in circular fashion
     *          2. Read PageHeader.FileNumber from each page
     *          3. Track maximum FileNumber encountered
     *          4. Handle wrap-around in circular buffer
     * 
     * @return Log file number (1-65535), or 0 if no logs found
     * 
     * @note Called during initialization and when listing logs
     * @note Scan time proportional to chip size (typically 100-1000ms)
     */
    uint16_t find_last_log() override;
    
    /**
     * @brief Get page boundaries for a specific log file
     * 
     * @param[in]  list_entry Log list index (0-based, ordered by FileNumber)
     * @param[out] start_page Physical page address where log file begins
     * @param[out] end_page   Physical page address where log file ends
     * 
     * @details Determines the start and end page addresses for the specified log file.
     *          List entries are ordered by FileNumber, with list_entry 0 being the oldest
     *          log file. The start_page and end_page define the range of physical pages
     *          occupied by the log file in flash memory.
     * 
     * @note list_entry must be < get_num_logs(), otherwise behavior is undefined
     */
    void get_log_boundaries(uint16_t list_entry, uint32_t & start_page, uint32_t & end_page) override;
    
    /**
     * @brief Get size and timestamp for a specific log file
     * 
     * @param[in]  list_entry Log list index (0-based, ordered by FileNumber)
     * @param[out] size       Log file size in bytes
     * @param[out] time_utc   UTC timestamp from FileHeader (seconds since Unix epoch)
     * 
     * @details Retrieves metadata for the specified log file:
     *          - size: Calculated from number of pages × page size
     *          - time_utc: Read from FileHeader at start of log file
     * 
     * @note list_entry must be < get_num_logs()
     * @note time_utc is 0 if log file has no valid FileHeader
     */
    void get_log_info(uint16_t list_entry, uint32_t &size, uint32_t &time_utc) override;
    
    /**
     * @brief Read log data from a specific log file
     * 
     * @param[in]  list_entry Log list index (0-based)
     * @param[in]  page       Page number within log file (0-based)
     * @param[in]  offset     Byte offset within page (0 to df_PageSize-1)
     * @param[in]  len        Number of bytes to read (must fit within page)
     * @param[out] data       Buffer to receive log data (must be ≥ len bytes)
     * 
     * @return Number of bytes read, or -1 on error
     * 
     * @details Reads raw log data from the specified location. The combination of
     *          page and offset specifies the exact byte position within the log file.
     *          Reads cannot cross page boundaries - if offset+len exceeds page size,
     *          only data up to page boundary is returned.
     * 
     *          Algorithm:
     *          1. Convert list_entry to physical page addresses
     *          2. Calculate physical page = start_page + page
     *          3. Call BlockRead() to read from flash
     *          4. Copy data from page buffer to output buffer
     * 
     * @note Reads are synchronous and block until complete (typically 1-3ms)
     * @note offset+len must not exceed df_PageSize
     * @warning Return value must be checked - WARN_IF_UNUSED attribute enforced
     * 
     * @see BlockRead() - Low-level page read operation
     */
    int16_t get_log_data(uint16_t list_entry, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) override WARN_IF_UNUSED;
    
    /**
     * @brief Complete a log download operation
     * 
     * @details Called when ground station completes log download. Currently a no-op
     *          for block-device backend as no state cleanup is required.
     */
    void end_log_transfer() override { }
    
    /**
     * @brief Get total number of log files on flash chip
     * 
     * @return Number of log files (0 if chip empty or no valid logs)
     * 
     * @details Scans flash chip to count distinct log files. Each unique FileNumber
     *          encountered in PageHeaders represents one log file.
     * 
     * @note Result cached after validate_log_structure() - fast to call repeatedly
     */
    uint16_t get_num_logs() override;
    
    /**
     * @brief Start a new log file
     * 
     * @details Initiates a new log file on the flash chip:
     *          1. Increments df_FileNumber
     *          2. Checks if erase needed via NeedErase()
     *          3. If erase needed, initiates async erase and sets new_log_pending
     *          4. Otherwise calls StartWrite() to begin new log
     *          5. Sets log_write_started flag
     *          6. Writes FileHeader with current UTC timestamp
     * 
     *          If chip is full and no space available:
     *          - Oldest log files are overwritten (circular buffer behavior)
     *          - Erase operation clears sectors as needed
     * 
     * @note Called from main thread - thread-safe via semaphores
     * @note May trigger async erase if insufficient free space
     * @note New log starts at next available page after current write position
     * 
     * @see StartWrite() - Begin writing to new log file
     * @see NeedErase() - Check if erase required
     */
    void start_new_log(void) override;
    
    /**
     * @brief Get available buffer space for new log data
     * 
     * @return Number of bytes available in writebuf ring buffer
     * 
     * @details Returns free space in the DMA-safe ring buffer (writebuf).
     *          This indicates how much log data can be queued before the
     *          buffer fills and writes must block or be dropped.
     * 
     *          Ring Buffer Full Condition:
     *          - When writebuf.available() returns 0
     *          - IO thread is not draining buffer fast enough
     *          - May indicate slow flash chip or erase in progress
     * 
     * @note Updated by IO thread as write_log_page() drains buffer
     * @note Critical for flow control - caller should throttle writes if low
     */
    uint32_t bufferspace_available() override;
    
    /**
     * @brief Stop logging immediately
     * 
     * @details Stops logging operations synchronously:
     *          1. Clears log_write_started flag
     *          2. Calls FinishWrite() to flush any pending data
     *          3. Writes final page to flash
     *          4. Updates df_PageAdr to next write position
     * 
     * @note Blocks until final page written (typically 1-3ms)
     * @note Safe to call from main thread
     * 
     * @see stop_logging_async() - Asynchronous variant
     * @see FinishWrite() - Flush and finalize current log file
     */
    void stop_logging(void) override;
    
    /**
     * @brief Request asynchronous stop of logging
     * 
     * @details Sets stop_log_pending flag to request logging stop. Actual stop
     *          is performed by IO thread on next io_timer() call. This prevents
     *          blocking the caller during final page writes.
     * 
     * @note Non-blocking - returns immediately
     * @note IO thread completes the stop on next cycle
     * @note Use logging_started() to verify logging has stopped
     */
    void stop_logging_async(void) override;
    
    /**
     * @brief Check if logging has encountered critical failures
     * 
     * @return true if logging cannot proceed due to errors, false if operational
     * 
     * @details Returns true if any of these conditions are true:
     *          - chip_full flag set (no writable space available)
     *          - erase_started but not yet complete
     *          - Flash chip not responding (CardInserted() returns false)
     *          - IO thread not running (io_timer_heartbeat not updating)
     * 
     * @note Checked by frontend to report logging status
     * @note Transient failures (erase in progress) resolve automatically
     */
    bool logging_failed() const override;
    
    /**
     * @brief Check if logging has been started
     * 
     * @return true if log file is open and accepting data, false otherwise
     * 
     * @note Set by start_new_log(), cleared by stop_logging()
     */
    bool logging_started(void) const override { return log_write_started; }
    
    /**
     * @brief IO thread callback for writing log pages
     * 
     * @details Called periodically by IO thread (typically 10-50Hz) to:
     *          1. Check if writebuf has enough data for a page write
     *          2. Copy data from writebuf to DMA-safe buffer
     *          3. Call BufferToPage() to write to flash
     *          4. Update df_PageAdr and df_FilePage
     *          5. Write PageHeader with FileNumber and FilePage
     *          6. Handle erase completion and pending operations
     * 
     *          Thread Safety:
     *          - Acquires write_sem before accessing writebuf
     *          - Acquires sem before flash chip operations
     *          - Updates io_timer_heartbeat for health monitoring
     * 
     * @note Called from dedicated IO thread - not from interrupt context
     * @note Critical for logging performance - must complete quickly
     * @note Typical execution time: 0.1-3ms depending on write operations
     * 
     * @see write_log_page() - Actual page write implementation
     */
    void io_timer(void) override;

protected:
    /**
     * @brief Write a block of data to the ring buffer
     * 
     * @param[in] pBuffer     Pointer to data to write
     * @param[in] size        Number of bytes to write
     * @param[in] is_critical true if data must be written (may block), false if can be dropped
     * 
     * @return true if data written successfully, false if buffer full and data dropped
     * 
     * @details Writes data to the writebuf ring buffer. The IO thread drains this buffer
     *          by writing pages to flash. Behavior depends on is_critical flag:
     *          
     *          If is_critical = true:
     *          - Blocks until space available in writebuf
     *          - Ensures critical messages (FMT, PARM) are never lost
     *          - May cause latency spike if buffer full
     *          
     *          If is_critical = false:
     *          - Returns immediately if insufficient space
     *          - Non-critical data may be dropped to prevent blocking
     *          - Allows system to continue operating under buffer pressure
     * 
     * @note Thread-safe via write_sem semaphore
     * @note Data is not immediately written to flash - buffered first
     * @warning Critical writes may block for extended periods if IO thread stalled
     */
    bool _WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical) override;
    
    /**
     * @brief Periodic 1Hz maintenance callback
     * 
     * @details Performs low-frequency maintenance tasks:
     *          - Checks IO thread health via io_timer_heartbeat
     *          - Reports status messages (ERASE_COMPLETE, RECOVERY_COMPLETE)
     *          - Updates warning messages if chip full or logging failed
     * 
     * @note Called from main scheduler at 1Hz rate
     */
    void periodic_1Hz() override;
    
    /**
     * @brief Periodic 10Hz maintenance callback
     * 
     * @param[in] now Current timestamp in milliseconds
     * 
     * @details Performs medium-frequency maintenance tasks:
     *          - Checks for pending erase completion
     *          - Initiates new log if new_log_pending flag set
     *          - Monitors buffer usage and health
     * 
     * @note Called from main scheduler at 10Hz rate
     */
    void periodic_10Hz(const uint32_t now) override;
    
    /**
     * @brief Check if writes can proceed
     * 
     * @return true if safe to write log data, false otherwise
     * 
     * @details Returns true if all conditions met for successful writes:
     *          - log_write_started flag is set
     *          - !chip_full (writable space available)
     *          - !erase_started (no erase in progress)
     *          - CardInserted() returns true
     * 
     * @note Used internally to gate write operations
     */
    bool WritesOK() const override;

    /**
     * @brief Get sector number from page address
     * 
     * @param[in] current_page Physical page address (1-based)
     * 
     * @return Sector number (0-based)
     * 
     * @details Converts page address to sector number for erase operations.
     *          Sectors are typically 4KB (16 pages of 256 bytes).
     *          Formula: sector = (page - 1) / df_PagePerSector
     * 
     * @note Pages are 1-based, sectors are 0-based
     */
    uint32_t get_sector(uint32_t current_page) const {
        return ((current_page - 1) / df_PagePerSector);
    }

    /**
     * @brief Get block number from page address
     * 
     * @param[in] current_page Physical page address (1-based)
     * 
     * @return Block number (0-based)
     * 
     * @details Converts page address to block number for erase operations.
     *          Blocks are typically 64KB (256 pages of 256 bytes).
     *          Formula: block = (page - 1) / df_PagePerBlock
     * 
     * @note Pages are 1-based, blocks are 0-based
     */
    uint32_t get_block(uint32_t current_page) const {
        return ((current_page - 1) / df_PagePerBlock);
    }

    // Flash chip geometry (initialized by Init())
    
    /** @brief Number of bytes per page (typically 256, 512, 1024, or 2048) */
    uint32_t df_PageSize;
    
    /** @brief Number of pages per 64KB block (typically 256 for 256-byte pages) */
    uint16_t df_PagePerBlock;
    
    /** @brief Number of pages per 4KB sector (typically 16 for 256-byte pages) */
    uint16_t df_PagePerSector;
    
    /** @brief Total number of pages on flash chip */
    uint32_t df_NumPages;
    
    /** @brief Flag indicating log file is open and accepting writes */
    volatile bool log_write_started;

    /**
     * @brief DMA-safe page-aligned buffer for flash write operations
     * 
     * @details This buffer is allocated with proper alignment to allow DMA controllers
     *          to directly access the memory during SPI/QSPI transfers. The alignment
     *          requirements vary by platform:
     *          - ARM platforms: 32-byte alignment for cache line efficiency
     *          - DMA requirements: Must not cross certain address boundaries
     *          - Size: Equals df_PageSize (typically 256-2048 bytes)
     * 
     *          Data Flow:
     *          writebuf (ring buffer) → buffer (DMA-safe) → BufferToPage() → flash chip
     * 
     * @note Allocated by Init() using hal.util->malloc_type(..., DMA_REGION)
     * @warning Must not be accessed during DMA transfers to prevent data corruption
     */
    uint8_t *buffer;
    
    /** @brief Timestamp of last MESSAGE_WRITE_INTERVAL_MS notification (milliseconds) */
    uint32_t last_messagewrite_message_sent;
    
    /** @brief Current page address for read operations (1-based page number) */
    uint32_t df_Read_PageAdr;

private:
    /**
     * @name Platform-Specific Flash Operations
     * @brief Pure virtual functions implemented by hardware-specific backends
     * 
     * @details These methods provide the hardware abstraction layer for different
     *          flash chip interfaces (SPI, QSPI, memory-mapped). Each platform
     *          (ChibiOS, Linux, SITL) provides its own implementation.
     * 
     * @{
     */
    
    /**
     * @brief Write the staging buffer to a flash page
     * 
     * @param[in] PageAdr Physical page address to write (1-based)
     * 
     * @details Writes the contents of the buffer to the specified page on the flash chip.
     *          This operation:
     *          1. Sends page program command to flash chip
     *          2. Transfers df_PageSize bytes from buffer via SPI/QSPI
     *          3. Waits for write to complete (typically 1-3ms)
     *          4. Verifies write success (platform dependent)
     * 
     *          Timing:
     *          - Typical page write: 1-3ms
     *          - Must be erased before writing (flash bits can only be changed 1→0)
     * 
     * @note Pure virtual - implemented by platform-specific subclass
     * @note Blocks until write completes
     * @warning Must not be called from interrupt context (blocks for milliseconds)
     * @warning Page must be erased before writing, or data will be corrupted
     */
    virtual void BufferToPage(uint32_t PageAdr) = 0;
    
    /**
     * @brief Read a flash page into the staging buffer
     * 
     * @param[in] PageAdr Physical page address to read (1-based)
     * 
     * @details Reads the specified page from flash chip into the buffer.
     *          This operation:
     *          1. Sends read command to flash chip
     *          2. Transfers df_PageSize bytes into buffer via SPI/QSPI
     *          3. Completes quickly (typically < 1ms for 256-byte page)
     * 
     *          Timing:
     *          - Typical read: 0.5-1ms for 256 bytes
     *          - Faster than writes (no physical erase/program cycle)
     * 
     * @note Pure virtual - implemented by platform-specific subclass
     * @note Blocks until read completes
     */
    virtual void PageToBuffer(uint32_t PageAdr) = 0;
    
    /**
     * @brief Erase a 64KB block
     * 
     * @param[in] SectorAdr Block address to erase (block number, 0-based)
     * 
     * @details Erases a 64KB block (typically 256 pages). Block erase sets all
     *          bits to 1 (0xFF), preparing the region for new writes.
     * 
     *          Timing:
     *          - Typical 64KB block erase: 500-2000ms (chip dependent)
     * 
     * @note Pure virtual - implemented by platform-specific subclass
     * @warning Blocks for 0.5-2 seconds during erase
     * @warning All data in block is permanently destroyed
     */
    virtual void SectorErase(uint32_t SectorAdr) = 0;
    
    /**
     * @brief Erase a 4KB sector
     * 
     * @param[in] SectorAdr Sector address to erase (sector number, 0-based)
     * 
     * @details Erases a 4KB sector (typically 16 pages). Sector erase is faster
     *          than block erase but still relatively slow.
     * 
     *          Timing:
     *          - Typical 4KB sector erase: 100-300ms (chip dependent)
     * 
     * @note Pure virtual - implemented by platform-specific subclass
     * @warning Blocks for 100-300ms during erase
     * @warning All data in sector is permanently destroyed
     */
    virtual void Sector4kErase(uint32_t SectorAdr) = 0;
    
    /**
     * @brief Initiate asynchronous chip erase
     * 
     * @details Begins a full chip erase operation that proceeds in the background.
     *          The erase may take 10-60 seconds depending on chip size.
     *          Use InErase() to monitor progress.
     * 
     *          Typical sequence:
     *          1. StartErase() initiates erase
     *          2. InErase() returns true while erase proceeds
     *          3. InErase() returns false when complete
     *          4. Chip is fully erased and ready for writing
     * 
     * @note Pure virtual - implemented by platform-specific subclass
     * @note Non-blocking - returns immediately
     * @warning Do not power off device during erase operation
     * @warning All data on chip is permanently destroyed
     * 
     * @see InErase() - Check erase progress
     */
    virtual void StartErase() = 0;
    
    /**
     * @brief Check if chip erase is in progress
     * 
     * @return true if erase operation still running, false if complete
     * 
     * @details Polls the flash chip status register to determine if a background
     *          erase operation (initiated by StartErase()) is still in progress.
     * 
     * @note Pure virtual - implemented by platform-specific subclass
     * @note Should be fast (< 1ms) - just reads status register
     * 
     * @see StartErase() - Initiate chip erase
     */
    virtual bool InErase() = 0;
    
    /** @} */ // end of Platform-Specific Flash Operations
    
    /**
     * @brief Perform flash chip diagnostic tests
     * 
     * @details Internal test function for verifying flash chip operation during
     *          development and debugging. Tests basic read/write/erase operations.
     * 
     * @note Not used in production code
     */
    void         flash_test(void);

    /**
     * @struct PageHeader
     * @brief Header structure stored at the beginning of each flash page
     * 
     * @details Every page on the flash chip begins with a PageHeader that identifies
     *          which log file the page belongs to and the page's position within that file.
     *          This allows the logger to:
     *          - Identify log file boundaries during recovery
     *          - Detect wrap-around in circular buffer
     *          - Reconstruct log file structure after power loss
     *          - Validate data integrity (optional CRC)
     * 
     *          Memory Layout (with BLOCK_LOG_VALIDATE=0):
     *          Offset 0-3: FilePage (uint32_t) - Page number within log file (1-based)
     *          Offset 4-5: FileNumber (uint16_t) - Log file number (1-65535)
     *          Total: 6 bytes
     * 
     *          Memory Layout (with BLOCK_LOG_VALIDATE=1):
     *          Offset 0-3: FilePage (uint32_t)
     *          Offset 4-5: FileNumber (uint16_t)
     *          Offset 6-9: crc (uint32_t) - CRC32 of page data
     *          Total: 10 bytes
     * 
     *          Log data immediately follows PageHeader in each page.
     *          Usable page data = df_PageSize - sizeof(PageHeader)
     * 
     * @note PACKED attribute ensures no padding bytes (critical for flash storage)
     * @note FilePage starts at 1, not 0 (0 indicates invalid/erased page)
     * @note FileNumber 0 indicates erased page, valid logs use 1-65535
     */
    struct PACKED PageHeader {
        /** @brief Page number within current log file (1-based, 0=invalid) */
        uint32_t FilePage;
        
        /** @brief Log file number (1-65535, 0=erased) */
        uint16_t FileNumber;
        
#if BLOCK_LOG_VALIDATE
        /** @brief CRC32 of page data for integrity validation (optional) */
        uint32_t crc;
#endif
    };

    /**
     * @struct FileHeader
     * @brief Header structure at the start of each log file
     * 
     * @details The first page of each log file (FilePage=1) contains a FileHeader
     *          immediately after the PageHeader. This provides metadata about when
     *          the log file was created.
     * 
     *          Memory Layout in First Page:
     *          Offset 0-5 (or 0-9):  PageHeader
     *          Offset 6-9 (or 10-13): FileHeader (utc_secs)
     *          Offset 10+:           Log data (FMT messages, PARM, etc.)
     * 
     *          The UTC timestamp allows ground stations to:
     *          - Sort logs by creation time
     *          - Display human-readable log dates
     *          - Correlate logs with external events
     * 
     * @note PACKED attribute ensures no padding bytes
     * @note utc_secs is 0 if GPS time not available when log started
     * @note FileHeader only present in first page (FilePage=1) of each log
     */
    struct PACKED FileHeader {
        /** @brief UTC timestamp when log file created (seconds since Unix epoch, 0=invalid) */
        uint32_t utc_secs;
    };

    /**
     * @brief Semaphore protecting flash chip access
     * 
     * @details Mediates access to the flash chip hardware between multiple contexts.
     *          Must be acquired before calling any BufferToPage, PageToBuffer, or
     *          erase operations to prevent concurrent access conflicts.
     * 
     *          Lock Order (to prevent deadlocks):
     *          1. Acquire write_sem first (if needed)
     *          2. Then acquire sem
     *          3. Release sem first
     *          4. Then release write_sem
     * 
     * @note Used by: io_timer(), Init(), erase operations, read operations
     */
    HAL_Semaphore sem;
    
    /**
     * @brief Semaphore protecting ring buffer (writebuf) access
     * 
     * @details Mediates access to the writebuf ring buffer between:
     *          - Main thread calling _WritePrioritisedBlock() (producer)
     *          - IO thread calling write_log_page() (consumer)
     * 
     *          Ring Buffer Algorithm:
     *          - Producer: Writes data to writebuf.write()
     *          - Consumer: Reads data from writebuf.read()
     *          - Full condition: writebuf.available() returns 0
     *          - Empty condition: writebuf.get_size() returns 0
     * 
     *          Thread Safety:
     *          - Always acquire write_sem before accessing writebuf
     *          - Keep critical section minimal to avoid blocking
     * 
     * @note Lock order: write_sem before sem (never reversed)
     */
    HAL_Semaphore write_sem;
    
    /**
     * @brief Ring buffer for incoming log data
     * 
     * @details ByteBuffer ring buffer that queues log data from main thread to IO thread.
     *          This decouples log data production from flash write operations, preventing
     *          the main thread from blocking during slow flash writes.
     * 
     *          Ring Buffer Mechanics:
     *          - Head pointer: Where new data is written (producer)
     *          - Tail pointer: Where data is read from (consumer)
     *          - Wrap-around: Pointers wrap to start when reaching end
     *          - Full condition: Head catches up to tail (no space)
     *          - Empty condition: Head equals tail (no data)
     * 
     *          Typical Size: 8KB - 32KB (configurable per platform)
     *          
     *          Data Flow:
     *          1. _WritePrioritisedBlock() writes to writebuf
     *          2. writebuf accumulates data until page-sized chunk available
     *          3. write_log_page() drains writebuf in page-sized chunks
     *          4. Data copied to DMA-safe buffer
     *          5. BufferToPage() writes to flash
     * 
     *          Performance Impact:
     *          - Larger buffer: More tolerance for flash write latency spikes
     *          - Smaller buffer: Less RAM usage but more likely to drop data
     *          - Optimal size: ~4-8 pages worth of data
     * 
     * @note Allocated by Init() - size platform dependent
     * @note Protected by write_sem semaphore
     * @note Must be sized to handle burst writes (parameter downloads, mission uploads)
     */
    ByteBuffer writebuf;

    /**
     * @name Read/Write State Variables
     * @brief Internal state tracking for log file operations
     * @{
     */
    
    /** @brief Current byte offset within buffer during read operations */
    uint16_t df_Read_BufferIdx;
    
    /**
     * @brief Current physical page address for write operations (1-based)
     * 
     * @details Tracks the next page to be written during logging. Increments after
     *          each page write. Wraps back to 1 when reaching df_NumPages+1.
     *          
     *          Circular Buffer Behavior:
     *          - Writing proceeds sequentially: page 1, 2, 3, ..., df_NumPages
     *          - When reaching end, wraps to page 1 (oldest data overwritten)
     *          - df_PageAdr always points to next writable page
     */
    uint32_t df_PageAdr;
    
    /** @brief Current log file number being read (1-65535) */
    uint16_t df_FileNumber;
    
    /** @brief Current log file number being written (1-65535) */
    uint16_t df_Write_FileNumber;
    
    /** @brief UTC timestamp of current log file (seconds since Unix epoch) */
    uint32_t df_FileTime;
    
    /**
     * @brief Current page number within log file being read (1-based)
     * 
     * @details Relative page index within the current log file. First page
     *          of each log has FilePage=1. Used to track position during
     *          log downloads and recovery.
     */
    uint32_t df_FilePage;
    
    /**
     * @brief Current page number within log file being written (1-based)
     * 
     * @details Increments with each page written to current log file.
     *          Reset to 1 when start_new_log() creates a new log file.
     *          Stored in PageHeader.FilePage for each page written.
     */
    uint32_t df_Write_FilePage;
    
    /**
     * @brief Physical page address to begin erasing from if corruption detected
     * 
     * @details Set by validate_log_structure() when log corruption is detected.
     *          Indicates the start of the corrupted region that should be erased
     *          before continuing logging.
     */
    uint32_t df_EraseFrom;
    
    /** @} */ // end of Read/Write State Variables
    
    /**
     * @brief Flag indicating FMT headers are being added to log
     * 
     * @details When starting a new log, format definition (FMT) messages must be
     *          written first to define the structure of subsequent log messages.
     *          This flag is true during the initial FMT message writes.
     * 
     * @note FMT messages define message structure for log parsing tools
     */
    bool adding_fmt_headers;

    /**
     * @name Asynchronous Operation Flags
     * @brief Volatile flags for coordinating async operations between threads
     * @{
     */
    
    /**
     * @brief Flag indicating chip erase operation in progress
     * 
     * @details Set by EraseAll() or when auto-erase needed. Cleared when
     *          InErase() returns false. While set, logging operations are
     *          deferred and logging_failed() may return true.
     * 
     * @note Volatile: Accessed from both main thread and IO thread
     */
    volatile bool erase_started;
    
    /**
     * @brief Flag indicating new log should start after erase completes
     * 
     * @details Set when start_new_log() called but erase is needed first.
     *          IO thread monitors this flag and calls StartWrite() when
     *          erase completes and flag is true.
     * 
     * @note Volatile: Accessed from both main thread and IO thread
     */
    volatile bool new_log_pending;
    
    /**
     * @brief Flag requesting asynchronous logging stop
     * 
     * @details Set by stop_logging_async(). IO thread checks this flag
     *          and calls FinishWrite() on next cycle. Allows non-blocking
     *          stop requests from main thread.
     * 
     * @note Volatile: Accessed from both main thread and IO thread
     */
    volatile bool stop_log_pending;
    
    /**
     * @brief Latch indicating flash chip has no writable space
     * 
     * @details Set when chip is full and no space available for new pages.
     *          Prevents repeated "chip full" warnings. Cleared when space
     *          becomes available (after erase or initialization).
     * 
     * @note Volatile: Accessed from both main thread and IO thread
     * @note Once set, logging cannot proceed until cleared
     */
    volatile bool chip_full;
    
    /**
     * @brief IO thread heartbeat timestamp for health monitoring
     * 
     * @details Updated by io_timer() on each call. Main thread checks this
     *          to detect if IO thread has stalled. If not updated for >5 seconds,
     *          IO thread is considered dead and logging_failed() returns true.
     * 
     *          Typical update rate: 10-50Hz (io_timer call frequency)
     * 
     * @note Volatile: Read by main thread, written by IO thread
     * @note Timestamp in milliseconds (system uptime)
     */
    volatile uint32_t io_timer_heartbeat;
    
    /** @brief Counter for decimating warning messages (prevents log spam) */
    uint8_t warning_decimation_counter;

    /**
     * @enum StatusMessage
     * @brief Status messages to be reported to user
     * 
     * @details Used to coordinate status reporting between IO thread (which
     *          detects events) and main thread (which reports to user).
     *          IO thread sets status_msg, periodic_1Hz() reads and reports it.
     */
    volatile enum class StatusMessage {
        NONE,              /**< No status message pending */
        ERASE_COMPLETE,    /**< Chip erase operation completed successfully */
        RECOVERY_COMPLETE, /**< Log structure recovery completed on initialization */
    } status_msg;
    
    /** @} */ // end of Asynchronous Operation Flags

    /**
     * @name Low-Level Read Operations
     * @brief Internal methods for reading data from flash pages
     * @{
     */
    
    /**
     * @brief Read data from within a flash page
     * 
     * @param[in]  IntPageAdr Offset within current page buffer (0 to df_PageSize-1)
     * @param[out] pBuffer    Destination buffer for data
     * @param[in]  size       Number of bytes to read
     * 
     * @return true if read successful, false on error
     * 
     * @details Reads data from the buffer (which holds current page data loaded
     *          by PageToBuffer). The IntPageAdr is an offset within the page buffer,
     *          not a physical page address.
     * 
     * @warning Caller must ensure IntPageAdr + size ≤ df_PageSize. If size exceeds
     *          page boundary, data wraps to start of page (modulo behavior).
     * @warning Must call PageToBuffer() first to load page into buffer
     * 
     * @note IntPageAdr is page-internal offset, not physical flash address
     */
    bool BlockRead(uint16_t IntPageAdr, void *pBuffer, uint16_t size);

    /**
     * @brief Read a block of data from current position in buffer
     * 
     * @param[out] pBuffer Destination buffer for data
     * @param[in]  size    Number of bytes to read
     * 
     * @return true if read successful, false if insufficient data
     * 
     * @details Reads sequential data from buffer starting at df_Read_BufferIdx.
     *          Advances df_Read_BufferIdx by size bytes. Used for streaming
     *          reads during log downloads.
     * 
     * @note Uses df_Read_BufferIdx to track position
     * @note Does not handle page boundaries - caller must manage
     */
    bool ReadBlock(void *pBuffer, uint16_t size);
    
    /** @} */ // end of Low-Level Read Operations
    
    /**
     * @name Erase Management
     * @brief Methods for determining when erases are needed
     * @{
     */
    
    /**
     * @brief Determine if erase is required before writing
     * 
     * @return true if erase needed, false if safe to write
     * 
     * @details Checks if the next page to be written (df_PageAdr) requires
     *          erasing before use. A page needs erase if:
     *          - It contains data from a previous log that would be overwritten
     *          - The page is marked corrupt by validate_log_structure()
     *          - We've wrapped around in circular buffer to old data
     * 
     *          Algorithm:
     *          1. Check if df_PageAdr > df_EraseFrom (in corrupted region)
     *          2. Check if we've wrapped around to old log data
     *          3. Return true if erase needed, triggering async erase
     * 
     * @note Called by start_new_log() before beginning new log file
     * @note Triggers async erase via StartErase() if true
     */
    bool NeedErase(void);
    
    /**
     * @brief Validate and recover log file structure on flash chip
     * 
     * @details Scans entire flash chip during initialization to:
     *          1. Identify all valid log files by scanning PageHeaders
     *          2. Detect corruption (invalid FileNumber, bad FilePage sequence)
     *          3. Mark corrupted regions for erase (sets df_EraseFrom)
     *          4. Build internal map of log file locations
     *          5. Determine where to resume writing (find_last_page)
     * 
     *          Recovery Process:
     *          - Scans all pages sequentially
     *          - Validates PageHeader.FileNumber (must be 1-65535)
     *          - Validates PageHeader.FilePage sequence within each log
     *          - Checks for CRC if BLOCK_LOG_VALIDATE enabled
     *          - Identifies oldest and newest logs
     *          - Determines if chip has wrapped around
     * 
     *          Corruption Handling:
     *          - Invalid FileNumber (0 or >65535): Page marked for erase
     *          - FilePage sequence gap: Log truncated at gap
     *          - CRC mismatch: Page and remainder of log marked invalid
     *          - Power-loss recovery: Incomplete page writes detected and handled
     * 
     *          Performance:
     *          - Scan time: ~1-5 seconds depending on chip size
     *          - Scans all df_NumPages pages
     *          - Can be slow on large chips (>128MB)
     * 
     * @note Called once during Init() - blocks until complete
     * @note Sets df_EraseFrom if corruption detected
     * @note Sets status_msg to RECOVERY_COMPLETE when done
     * 
     * @warning Blocks for 1-5 seconds during chip scan
     * @see find_last_page() - Determines write start position after validation
     */
    void validate_log_structure();
    
    /** @} */ // end of Erase Management
    
    /**
     * @name Internal High-Level Operations
     * @brief Private methods for log file management
     * @{
     */
    
    /**
     * @brief Read raw log data without offset adjustment
     * 
     * @param[in]  log_num Log file number (1-65535)
     * @param[in]  page    Page number within log (0-based)
     * @param[in]  offset  Byte offset within page
     * @param[in]  len     Number of bytes to read
     * @param[out] data    Buffer to receive data
     * 
     * @return Number of bytes read, or -1 on error
     * 
     * @details Internal method used by get_log_data(). Reads raw data without
     *          accounting for FMT message headers that may have been prepended.
     * 
     * @warning Return value must be checked - WARN_IF_UNUSED enforced
     */
    int16_t get_log_data_raw(uint16_t log_num, uint32_t page, uint32_t offset, uint16_t len, uint8_t *data) WARN_IF_UNUSED;
    
    /**
     * @brief Begin reading from specified page
     * 
     * @param[in] PageAdr Physical page address to start reading from
     * 
     * @return FileNumber at that page, or 0 if invalid
     * 
     * @details Positions read state to the specified page:
     *          1. Sets df_Read_PageAdr = PageAdr
     *          2. Calls PageToBuffer(PageAdr) to load page
     *          3. Reads and validates PageHeader
     *          4. Sets df_Read_BufferIdx to start of log data
     *          5. Returns FileNumber from PageHeader
     */
    uint16_t StartRead(uint32_t PageAdr);
    
    /**
     * @brief Read PageHeader and FileHeader at current position
     * 
     * @return FileNumber from PageHeader, or 0 if invalid
     * 
     * @details Reads headers from buffer at current df_Read_BufferIdx:
     *          1. Reads PageHeader (6 or 10 bytes depending on BLOCK_LOG_VALIDATE)
     *          2. If FilePage==1, also reads FileHeader (4 bytes)
     *          3. Advances df_Read_BufferIdx past headers
     *          4. Updates df_FilePage and df_FileNumber
     */
    uint16_t ReadHeaders();
    
    /**
     * @brief Find the last written page on the chip
     * 
     * @return Physical page address of last written page, or 1 if chip empty
     * 
     * @details Scans chip to find where logging should resume. Uses the following
     *          algorithm:
     *          1. Scan for highest FileNumber
     *          2. Within that file, find highest FilePage
     *          3. Return physical page address of that page + 1
     *          4. Handle wrap-around at end of chip
     * 
     *          Called during initialization to position df_PageAdr for new writes.
     * 
     * @note Depends on validate_log_structure() being called first
     * @note Returns page+1 (next writable page), not last written page
     */
    uint32_t find_last_page(void);
    
    /**
     * @brief Find the last page of a specific log file
     * 
     * @param[in] log_number Log file number to search for
     * 
     * @return Physical page address of last page in that log file
     * 
     * @details Scans chip for all pages with PageHeader.FileNumber == log_number,
     *          and returns the address of the page with highest FilePage value.
     */
    uint32_t find_last_page_of_log(uint16_t log_number);
    
    /**
     * @brief Check if chip has wrapped around in circular buffer
     * 
     * @return true if writes have wrapped to overwrite old data, false otherwise
     * 
     * @details Determines if logging has proceeded long enough to wrap around
     *          from end of chip back to beginning, overwriting oldest logs.
     *          
     *          Detection:
     *          - Looks for FileNumber discontinuity (newer file at lower address)
     *          - Checks if oldest and newest logs indicate wrap
     */
    bool is_wrapped(void);
    
    /**
     * @brief Begin writing to a new location on flash chip
     * 
     * @param[in] PageAdr Physical page address to begin writing at
     * 
     * @details Initializes write state for a new log file or resume:
     *          1. Sets df_PageAdr = PageAdr (current write position)
     *          2. Sets df_Write_FilePage = 1 (first page of new log)
     *          3. Increments df_Write_FileNumber for new log
     *          4. Sets df_FileTime = current UTC time
     *          5. Clears any error flags
     * 
     * @note Called by start_new_log() after erase check
     * @see FinishWrite() - Complete current log file
     */
    void StartWrite(uint32_t PageAdr);
    
    /**
     * @brief Finish writing current log file
     * 
     * @details Completes the current log file write:
     *          1. Flushes any partial page data from writebuf
     *          2. Writes final PageHeader
     *          3. Calls BufferToPage() for last page
     *          4. Clears log_write_started flag
     *          5. Updates df_PageAdr to next page
     * 
     * @note Blocks until final page written (1-3ms)
     * @note Called by stop_logging() and when starting new log
     * @see StartWrite() - Begin new log file
     */
    void FinishWrite(void);
    
    /**
     * @brief Initialize a new log file with headers
     * 
     * @param[in] FileNumber Log file number for new log (1-65535)
     * 
     * @details Prepares a new log file by:
     *          1. Writing FileHeader with current UTC time
     *          2. Queuing FMT messages (format definitions)
     *          3. Queuing PARM messages (parameter values)
     *          4. Setting up PageHeader for first page
     * 
     * @note Called by StartWrite() when beginning new log
     */
    void StartLogFile(uint16_t FileNumber);
    
    /**
     * @brief Get current log file number being read
     * 
     * @return Current FileNumber from read state
     */
    uint16_t GetFileNumber() const;

    /**
     * @brief Print log format definitions to a stream
     * 
     * @param[in] port Output stream for format definitions
     * 
     * @details Outputs all FMT message definitions for log parsing tools.
     *          Used when downloading logs via MAVLink or serial.
     */
    void _print_log_formats(AP_HAL::BetterStream *port);
    
    /** @} */ // end of Internal High-Level Operations

    /**
     * @name IO Thread Operations
     * @brief Methods called from IO thread context
     * @{
     */
    
    /**
     * @brief Check if IO thread is healthy
     * 
     * @return true if IO thread is running, false if stalled
     * 
     * @details Checks io_timer_heartbeat timestamp to determine if IO thread
     *          is still calling io_timer() regularly. If heartbeat hasn't
     *          updated in >5 seconds, thread is considered dead.
     * 
     * @note Called from main thread to monitor IO thread health
     */
    bool io_thread_alive() const;
    
    /**
     * @brief Write one page of log data from ring buffer to flash
     * 
     * @details Core IO thread function that drains writebuf ring buffer:
     *          1. Check if writebuf has ≥ df_PageSize bytes available
     *          2. Acquire write_sem to access writebuf
     *          3. Copy df_PageSize bytes from writebuf to DMA-safe buffer
     *          4. Release write_sem
     *          5. Write PageHeader at start of buffer
     *          6. Acquire sem for chip access
     *          7. Call BufferToPage(df_PageAdr) to write to flash
     *          8. Release sem
     *          9. Increment df_PageAdr and df_Write_FilePage
     *          10. Handle wrap-around at end of chip
     * 
     *          Timing:
     *          - Typical execution: 1-3ms (dominated by BufferToPage)
     *          - Called repeatedly from io_timer() while data available
     *          - Must complete quickly to maintain logging throughput
     * 
     *          Flow Control:
     *          - If writebuf empty, returns immediately
     *          - If flash chip busy (InErase()), defers write
     *          - If chip full, sets chip_full flag and stops
     * 
     * @note Called from IO thread only - not thread-safe for main thread
     * @note Blocks for 1-3ms during flash write
     * @see io_timer() - Periodic callback that invokes this method
     */
    void write_log_page();
    
    /** @} */ // end of IO Thread Operations
};

#endif  // HAL_LOGGING_BLOCK_ENABLED
