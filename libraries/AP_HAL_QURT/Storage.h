/**
 * @file Storage.h
 * @brief QURT platform storage implementation providing emulated EEPROM functionality
 * 
 * @details This file implements persistent parameter storage for the Qualcomm Hexagon
 *          DSP platform (QURT RTOS) using an in-memory RAM buffer with periodic write-back
 *          to flash storage. The implementation emulates traditional EEPROM behavior while
 *          optimizing for flash wear leveling and DSP memory constraints.
 *          
 *          Architecture: RAM buffer maintains current storage state, writes update buffer
 *          immediately and mark corresponding storage lines as dirty. A periodic timer
 *          callback incrementally flushes dirty lines to the flash partition, spreading
 *          flash write operations over time to prevent blocking and reduce wear.
 * 
 * @note Platform: Qualcomm Hexagon DSP running QURT RTOS (e.g., VOXL flight controller)
 * @warning DSP memory constraints: Storage buffer consumes precious DSP RAM
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

/**
 * @def QURT_STORAGE_SIZE
 * @brief Total storage size in bytes for parameter and configuration persistence
 * 
 * @details Defines the size of the in-memory storage buffer that emulates EEPROM behavior.
 *          This buffer holds all persistent vehicle configuration including:
 *          - Vehicle parameters (PID gains, sensor calibration, flight modes)
 *          - Fence points (geofence polygon vertices)
 *          - Rally points (safe return locations)
 *          - Learned values (compass offsets, accelerometer calibration)
 *          
 *          The buffer is divided into storage lines for efficient dirty tracking and
 *          incremental flash write-back. Typical size ranges from 4KB to 16KB depending
 *          on parameter count and configuration complexity.
 * 
 * @note Resolves to HAL_STORAGE_SIZE which is platform-configurable
 * @note Storage format is ArduPilot-standard, compatible across all platforms
 * @warning DSP memory constraints: Storage buffer consumes precious DSP RAM, size carefully
 * @warning Flash partition must be defined in platform configuration (e.g., VOXL board definition)
 * 
 * @see AP_Param for parameter storage format details
 * @see HAL_STORAGE_SIZE in AP_HAL for platform-specific size configuration
 */
#define QURT_STORAGE_SIZE HAL_STORAGE_SIZE

namespace QURT
{

/**
 * @class Storage
 * @brief Emulated EEPROM storage using RAM buffer with write-back to flash partition
 * 
 * @details This class implements the AP_HAL::Storage interface to provide persistent
 *          parameter and configuration storage on the Qualcomm Hexagon DSP platform.
 *          Unlike traditional EEPROM or direct flash access, this implementation uses
 *          a write-back caching strategy to optimize for DSP platform constraints:
 *          
 *          **Architecture:**
 *          - In-memory buffer (QURT_STORAGE_SIZE bytes) maintains current storage state
 *          - Storage logically divided into lines (typically 512 bytes each)
 *          - Dirty line tracking via _dirty_mask bitmask (one bit per line)
 *          - Writes update RAM immediately and mark corresponding lines dirty
 *          - Periodic _timer_tick() callback incrementally flushes dirty lines to flash
 *          
 *          **Write-Back Mechanism:**
 *          Read operations access RAM buffer directly (fast, ~50ns latency).
 *          Write operations update RAM buffer immediately and set corresponding bit(s)
 *          in _dirty_mask to indicate the line(s) need flash write-back. The _timer_tick()
 *          method, called periodically by the scheduler (typically 10-100Hz), scans
 *          _dirty_mask and flushes one dirty line per invocation. This spreads flash
 *          writes over time, preventing long blocking periods while still persisting
 *          data within reasonable time (~seconds for full storage).
 *          
 *          **Wear Leveling:**
 *          Batching writes to storage lines reduces flash erase cycles compared to
 *          writing on every parameter change. Flash memory typically supports 10K-100K
 *          write cycles; incremental flushing extends flash lifetime significantly.
 *          
 *          **Platform Considerations:**
 *          On the Hexagon DSP platform, the flash controller resides on the applications
 *          processor. All flash writes require RPC (Remote Procedure Call) to the apps
 *          processor, adding significant latency (1-10ms per line). The write-back
 *          architecture minimizes RPC overhead by batching and deferring flash writes.
 * 
 * @note Storage lines: Typically 512 bytes to match flash page size for efficient writing
 * @note Storage contents: All data stored in ArduPilot-standard format (compatible across platforms)
 * @note Performance: Reads fast (RAM ~50ns), writes add dirty tracking (~1us), flush slow (flash ~1-10ms/line)
 * 
 * @warning Power loss risk: Recent writes exist only in RAM until flushed to flash.
 *          Power failure before flush loses uncommitted changes. Critical data should
 *          call flush() explicitly to force immediate persistence.
 * @warning Thread safety: Not inherently thread-safe. Caller should use WITH_SEMAPHORE
 *          if concurrent access from multiple threads is possible.
 * @warning Flash wear: Flash memory has limited write cycles (typically 10K-100K).
 *          Frequent writes degrade flash. Write-back mechanism mitigates but doesn't eliminate wear.
 * @warning DSP memory: Storage buffer consumes DSP RAM (QURT_STORAGE_SIZE bytes).
 *          DSP memory is constrained resource; balance storage size with available RAM.
 * 
 * @see AP_HAL::Storage for the interface contract this class implements
 * @see AP_Param for parameter storage format and access patterns
 * @see AP_Scheduler for _timer_tick() callback registration
 */
class Storage : public AP_HAL::Storage
{
public:
    /**
     * @brief Static cast helper to convert AP_HAL::Storage pointer to QURT::Storage
     * 
     * @param[in] storage Pointer to AP_HAL::Storage interface
     * @return QURT::Storage* Pointer cast to QURT-specific implementation
     * 
     * @note Convenience method for accessing platform-specific functionality
     * @warning Caller must ensure storage pointer is actually QURT::Storage instance
     */
    static Storage *from(AP_HAL::Storage *storage)
    {
        return static_cast<Storage*>(storage);
    }


    /**
     * @brief Initialize storage system and load existing data from flash
     * 
     * @details Performs storage subsystem initialization:
     *          1. Opens flash partition for storage persistence
     *          2. Reads existing storage data from flash into RAM buffer
     *          3. Initializes dirty mask to zero (all lines clean)
     *          4. Registers _timer_tick() callback with scheduler for periodic flushing
     *          
     *          If flash partition doesn't exist or read fails, initializes buffer to
     *          default values (typically all 0xFF to simulate erased EEPROM).
     * 
     * @note Called once during HAL initialization before parameter system loads
     * @note Flash partition must be defined in platform configuration (board definition)
     * @warning Blocking operation: Flash read may take 10-100ms depending on storage size
     * @warning Initialization failure may result in loss of stored parameters (reverts to defaults)
     * 
     * @see _storage_create() for flash partition creation
     * @see AP_Param::load_all() for parameter loading after storage initialization
     */
    void init() override;

    /**
     * @brief Read single byte from storage
     * 
     * @param[in] loc Offset in storage (0 to QURT_STORAGE_SIZE-1)
     * @return uint8_t Byte value at specified location
     * 
     * @note Reads directly from RAM buffer (fast operation, ~50ns)
     * @note No bounds checking in release builds for performance
     * @warning Reading uninitialized storage returns unpredictable values
     */
    uint8_t  read_byte(uint16_t loc);
    
    /**
     * @brief Read 16-bit word from storage
     * 
     * @param[in] loc Offset in storage (0 to QURT_STORAGE_SIZE-2)
     * @return uint16_t 16-bit value at specified location (little-endian)
     * 
     * @note Reads directly from RAM buffer, combines two bytes
     * @warning Ensure loc+1 < QURT_STORAGE_SIZE to avoid buffer overflow
     */
    uint16_t read_word(uint16_t loc);
    
    /**
     * @brief Read 32-bit double-word from storage
     * 
     * @param[in] loc Offset in storage (0 to QURT_STORAGE_SIZE-4)
     * @return uint32_t 32-bit value at specified location (little-endian)
     * 
     * @note Reads directly from RAM buffer, combines four bytes
     * @warning Ensure loc+3 < QURT_STORAGE_SIZE to avoid buffer overflow
     */
    uint32_t read_dword(uint16_t loc);
    
    /**
     * @brief Read block of data from storage
     * 
     * @param[out] dst Destination buffer to receive data
     * @param[in]  src Offset in storage to read from
     * @param[in]  n   Number of bytes to read
     * 
     * @details Reads arbitrary-length block from RAM buffer into destination.
     *          Always reads from RAM buffer (no flash access), providing fast
     *          read performance regardless of block size.
     * 
     * @note Fast operation: memcpy from RAM buffer (~50ns + n*2ns)
     * @warning Caller must ensure dst buffer is at least n bytes
     * @warning Reading beyond storage bounds (src+n > QURT_STORAGE_SIZE) results in undefined behavior
     * 
     * @see write_block() for corresponding write operation
     */
    void     read_block(void *dst, uint16_t src, size_t n) override;

    /**
     * @brief Write single byte to storage
     * 
     * @param[in] loc   Offset in storage (0 to QURT_STORAGE_SIZE-1)
     * @param[in] value Byte value to write
     * 
     * @details Updates RAM buffer immediately at specified location and marks
     *          corresponding storage line as dirty for later flush to flash.
     *          Write is non-blocking; actual flash persistence occurs during
     *          periodic _timer_tick() calls.
     * 
     * @note RAM update is immediate (~1us including dirty tracking)
     * @note Flash persistence is deferred (typically within seconds)
     * @warning Data not persisted to flash immediately; power loss before flush loses change
     * 
     * @see flush() to force immediate flash write-back
     * @see _mark_dirty() for dirty tracking implementation
     */
    void write_byte(uint16_t loc, uint8_t value);
    
    /**
     * @brief Write 16-bit word to storage
     * 
     * @param[in] loc   Offset in storage (0 to QURT_STORAGE_SIZE-2)
     * @param[in] value 16-bit value to write (stored little-endian)
     * 
     * @details Updates RAM buffer immediately and marks corresponding storage
     *          line(s) as dirty. If word spans two storage lines, marks both dirty.
     * 
     * @note RAM update is immediate
     * @warning Ensure loc+1 < QURT_STORAGE_SIZE to avoid buffer overflow
     * @warning Data not persisted to flash immediately
     */
    void write_word(uint16_t loc, uint16_t value);
    
    /**
     * @brief Write 32-bit double-word to storage
     * 
     * @param[in] loc   Offset in storage (0 to QURT_STORAGE_SIZE-4)
     * @param[in] value 32-bit value to write (stored little-endian)
     * 
     * @details Updates RAM buffer immediately and marks corresponding storage
     *          line(s) as dirty. If dword spans multiple storage lines, marks all dirty.
     * 
     * @note RAM update is immediate
     * @warning Ensure loc+3 < QURT_STORAGE_SIZE to avoid buffer overflow
     * @warning Data not persisted to flash immediately
     */
    void write_dword(uint16_t loc, uint32_t value);
    
    /**
     * @brief Write block of data to storage
     * 
     * @param[in] dst Offset in storage to write to
     * @param[in] src Source buffer containing data to write
     * @param[in] n   Number of bytes to write
     * 
     * @details Writes arbitrary-length block from source buffer to RAM buffer and
     *          marks all affected storage lines as dirty for later flush to flash.
     *          Write is non-blocking; RAM update is immediate but flash persistence
     *          is deferred to periodic _timer_tick() callbacks.
     *          
     *          Algorithm:
     *          1. Copy n bytes from src to _buffer[dst]
     *          2. Call _mark_dirty(dst, n) to set dirty bits for affected lines
     *          3. Return immediately (no flash write)
     * 
     * @note RAM update performance: memcpy (~1us + n*2ns) + dirty tracking (~1us)
     * @note Flash persistence deferred (typically within seconds depending on flush rate)
     * 
     * @warning Caller must ensure src buffer contains at least n bytes
     * @warning Writing beyond storage bounds (dst+n > QURT_STORAGE_SIZE) results in undefined behavior
     * @warning Data not persisted to flash immediately; power loss before flush loses changes
     * @warning For critical data, call flush() after write_block() to force immediate persistence
     * 
     * @see read_block() for corresponding read operation
     * @see _mark_dirty() for dirty line tracking
     * @see flush() to force immediate flash write-back
     */
    void write_block(uint16_t dst, const void* src, size_t n) override;

    /**
     * @brief Get direct pointer to storage buffer for bulk access
     * 
     * @param[out] ptr  Reference to pointer, set to _buffer address on success
     * @param[out] size Reference to size_t, set to QURT_STORAGE_SIZE on success
     * @return bool true if storage initialized and pointer provided, false otherwise
     * 
     * @details Provides direct access to the internal RAM buffer for bulk operations.
     *          Used by AP_Param system for efficient parameter loading/saving.
     *          
     * @note Direct pointer access bypasses dirty tracking; caller responsible for
     *       calling flush() after modifications
     * @note Only succeeds if _initialised is true (after successful init())
     * 
     * @warning Direct buffer modification bypasses write tracking; must manually
     *          set _dirty_mask or call flush() to persist changes
     * @warning Concurrent access requires external synchronization (WITH_SEMAPHORE)
     * 
     * @see AP_Param for parameter system bulk access patterns
     */
    bool get_storage_ptr(void *&ptr, size_t &size) override;

    /**
     * @brief Periodic callback to incrementally flush dirty storage lines to flash
     * 
     * @details Called by scheduler at regular intervals (typically 10-100Hz) to
     *          incrementally persist dirty storage lines to flash. Each invocation
     *          flushes at most one dirty line, spreading flash writes over time to
     *          prevent long blocking periods while still ensuring data persistence
     *          within reasonable time (~seconds for full storage).
     *          
     *          Algorithm:
     *          1. Scan _dirty_mask for set bits (dirty lines)
     *          2. If dirty line found:
     *             a. Calculate line offset in storage (line_num * line_size)
     *             b. Write line data to flash partition via RPC
     *             c. Clear corresponding bit in _dirty_mask
     *             d. Advance to next line for next invocation
     *          3. If no dirty lines, do nothing (fast return)
     *          
     *          **Flush Rate Example:**
     *          Storage: 16KB, Line: 512 bytes = 32 lines total
     *          Tick rate: 50Hz, Flush: 1 line/tick
     *          Full storage flush time: 32 lines / 50Hz = 0.64 seconds
     *          
     *          **Platform Consideration:**
     *          On Hexagon DSP, flash controller resides on apps processor. Each line
     *          flush requires RPC to apps processor, adding 1-10ms latency per flush.
     *          Incremental flushing prevents blocking main flight loops.
     * 
     * @note Registered with AP_Scheduler during init() for periodic execution
     * @note Flush rate: 1 line per invocation regardless of dirty line count
     * @note Flash write latency: 1-10ms per line due to RPC to apps processor
     * 
     * @warning Blocking operation: Each line flush may block for 1-10ms
     * @warning Recent writes may remain in RAM for seconds before flush completes
     * @warning Power failure during flush may result in partially persisted state
     * 
     * @see AP_Scheduler for timer callback registration
     * @see flush() for synchronous forced flush of all dirty lines
     * @see _mark_dirty() for dirty line tracking mechanism
     */
    virtual void _timer_tick(void) override;

protected:
    /**
     * @brief Mark storage lines as dirty after write operations
     * 
     * @param[in] loc    Starting offset in storage of modified data
     * @param[in] length Number of bytes modified
     * 
     * @details Calculates which storage line(s) are affected by a write operation
     *          and sets corresponding bits in _dirty_mask. Storage is logically
     *          divided into fixed-size lines (typically 512 bytes); any write
     *          affecting a line marks that entire line for flush.
     *          
     *          Algorithm:
     *          1. Calculate first affected line: start_line = loc / line_size
     *          2. Calculate last affected line: end_line = (loc + length - 1) / line_size
     *          3. Set bits [start_line..end_line] in _dirty_mask
     *          
     *          Example: Write 100 bytes at offset 600, line size 512:
     *          - start_line = 600 / 512 = 1
     *          - end_line = (600 + 100 - 1) / 512 = 1
     *          - Set bit 1 in _dirty_mask
     * 
     * @note Called internally by write_byte(), write_word(), write_dword(), write_block()
     * @note Multiple writes to same line only set dirty bit once (idempotent)
     * @note Dirty bit remains set until line flushed by _timer_tick()
     * 
     * @see _timer_tick() for dirty line flushing mechanism
     * @see _dirty_mask for dirty tracking bitmask structure
     */
    void _mark_dirty(uint16_t loc, uint16_t length);
    
    /**
     * @brief Create or open flash partition for storage persistence
     * 
     * @return bool true if flash partition successfully opened, false on failure
     * 
     * @details Opens the dedicated flash partition used for storage persistence.
     *          If partition doesn't exist, attempts to create it with appropriate
     *          size (typically slightly larger than QURT_STORAGE_SIZE for wear
     *          leveling metadata). On success, sets _fd to valid file descriptor
     *          for subsequent flash read/write operations.
     *          
     *          Flash partition is separate from firmware/bootloader partitions,
     *          preserving parameters across firmware updates.
     * 
     * @note Called during init() to establish flash access
     * @note Flash partition name/location defined in platform configuration
     * @note Partition size typically QURT_STORAGE_SIZE + metadata overhead
     * 
     * @warning Flash partition must be defined in board configuration (e.g., VOXL hwdef)
     * @warning Creation failure results in volatile storage (parameters lost on reboot)
     * 
     * @see init() for initialization sequence
     */
    bool _storage_create(void);

    /**
     * @brief File descriptor for flash partition access
     * 
     * @details Open file descriptor referencing the flash partition where storage
     *          is persisted. Set to valid descriptor (>=0) after successful
     *          _storage_create() call during init(). Used for all flash read/write
     *          operations. Value -1 indicates partition not opened (storage non-functional).
     * 
     * @note Initialized to -1 (invalid)
     * @note Set to valid descriptor by _storage_create() during init()
     * @note Used by _timer_tick() for incremental line flushing
     */
    int _fd = -1;
    
    /**
     * @brief Storage initialization complete flag
     * 
     * @details Set to true after successful init() completion (flash partition opened,
     *          existing data loaded into RAM buffer, scheduler callback registered).
     *          Used to guard operations requiring initialized storage. Volatile to
     *          ensure visibility across threads/interrupts.
     * 
     * @note Volatile: Ensures memory ordering in multi-threaded access
     * @note Checked by get_storage_ptr() before providing buffer access
     * @warning Operations on uninitialized storage result in undefined behavior
     */
    volatile bool _initialised;
    
    /**
     * @brief Dirty line tracking bitmask for incremental flash write-back
     * 
     * @details Bitmask where each bit corresponds to one storage line. Bit set (1)
     *          indicates line has been modified since last flush and requires write-back
     *          to flash. Bit clear (0) indicates line matches flash (clean).
     *          
     *          Structure: For storage size S and line size L:
     *          - Number of lines = S / L
     *          - Bitmask size = ceil((S / L) / 8) bytes
     *          - Each bit tracks one line
     *          
     *          Example: 16KB storage, 512-byte lines:
     *          - Lines = 16384 / 512 = 32 lines
     *          - Bitmask = 32 bits (uint32_t)
     *          - Bit 0 tracks line 0 (bytes 0-511)
     *          - Bit 31 tracks line 31 (bytes 15872-16383)
     *          
     *          Updated by _mark_dirty() after writes, scanned and cleared by _timer_tick()
     *          during incremental flushing. Volatile to ensure atomic access visibility.
     * 
     * @note Volatile: Ensures memory ordering between write operations and flush
     * @note Provides efficient dirty line identification without scanning entire buffer
     * @note Cleared bit-by-bit as _timer_tick() flushes lines to flash
     * 
     * @see _mark_dirty() for dirty bit setting
     * @see _timer_tick() for dirty bit scanning and clearing
     */
    volatile uint32_t _dirty_mask;
    
    /**
     * @brief In-memory RAM buffer maintaining current storage state
     * 
     * @details Primary storage buffer holding all persistent vehicle configuration.
     *          Provides fast read/write access (RAM speed) while deferring flash
     *          writes for wear leveling. All read operations access this buffer
     *          directly; all write operations update this buffer and mark corresponding
     *          lines dirty for later flush.
     *          
     *          Contents organized by AP_Param system:
     *          - Header: Parameter table metadata
     *          - Body: Parameter values, fence points, rally points, learned values
     *          - Format: ArduPilot-standard binary encoding (platform-independent)
     *          
     *          Size: QURT_STORAGE_SIZE bytes (typically 4KB-16KB)
     * 
     * @note All reads served from this buffer (no flash access on reads)
     * @note All writes update this buffer immediately (flash writes deferred)
     * @note Consumes DSP RAM continuously (not paged or swappable)
     * 
     * @warning DSP memory constraint: Buffer consumes precious DSP RAM continuously
     * @warning Power loss before flush loses dirty buffer contents not yet written to flash
     * 
     * @see QURT_STORAGE_SIZE for buffer size definition
     * @see AP_Param for storage format and organization
     */
    uint8_t _buffer[QURT_STORAGE_SIZE];
};

}
