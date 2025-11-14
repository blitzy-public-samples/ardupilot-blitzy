/**
 * @file Storage.h
 * @brief Nonvolatile storage emulation for Software In The Loop (SITL) simulation
 * 
 * @details This file implements the AP_HAL::Storage interface for SITL, providing
 *          emulated nonvolatile storage that mimics real hardware EEPROM/Flash behavior.
 *          The implementation maintains parameter persistence across SITL runs by
 *          storing data in a file (typically eeprom.bin) on the host filesystem.
 * 
 *          SITL storage supports multiple backend implementations:
 *          - POSIX file-based storage (default): Direct file I/O for simplicity
 *          - Flash storage simulation: Emulates flash memory with sector erase
 *          - FRAM simulation: Emulates Ferroelectric RAM with unlimited write cycles
 * 
 *          The storage system uses an in-memory buffer with dirty-line tracking to
 *          optimize write operations. Modified data is marked dirty and periodically
 *          flushed to the persistent backend during timer ticks.
 * 
 *          This storage backend is used by AP_Param for parameter persistence,
 *          allowing parameters to retain their values across SITL simulator restarts.
 * 
 * @note SITL storage writes are synchronous and complete immediately, unlike real
 *       hardware which may have write latency or require erase cycles.
 * 
 * @warning SITL storage does NOT simulate:
 *          - Flash wear leveling or write cycle limitations
 *          - EEPROM write time delays
 *          - Power-loss interruption of writes
 *          - Bit errors or corruption
 *          Use with awareness that timing and reliability differ from real hardware.
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/Bitmask.h>
#include "AP_HAL_SITL_Namespace.h"
#include <AP_FlashStorage/AP_FlashStorage.h>
#include <AP_RAMTRON/AP_RAMTRON.h>

/**
 * @brief Enable flash storage backend simulation
 * @details When set to 1, enables AP_FlashStorage-based storage emulation that
 *          simulates flash memory behavior with sector erase operations.
 */
#ifndef STORAGE_USE_FLASH
#define STORAGE_USE_FLASH 1
#endif

/**
 * @brief Enable POSIX file-based storage backend
 * @details When set to 1, enables direct file I/O storage using standard POSIX
 *          operations. This is the simplest backend and default for SITL.
 */
#ifndef STORAGE_USE_POSIX
#define STORAGE_USE_POSIX 1
#endif

/**
 * @brief Enable FRAM (Ferroelectric RAM) storage backend simulation
 * @details When set to 1, enables FRAM emulation using AP_RAMTRON. FRAM provides
 *          unlimited write cycles and instant writes without erase requirements.
 */
#ifndef STORAGE_USE_FRAM
#define STORAGE_USE_FRAM HAL_WITH_RAMTRON
#endif

/**
 * @brief Storage line size bit shift value (3 = 8 bytes per line)
 * @details Defines the granularity of dirty-line tracking. Each storage "line"
 *          is 2^STORAGE_LINE_SHIFT bytes (8 bytes with shift=3). When data is
 *          modified, entire lines are marked dirty for efficient batch writes.
 */
#define STORAGE_LINE_SHIFT 3

/**
 * @brief Size of each storage line in bytes (8 bytes)
 * @details Calculated as 2^STORAGE_LINE_SHIFT. This determines the minimum
 *          granularity for write operations and dirty tracking.
 */
#define STORAGE_LINE_SIZE (1<<STORAGE_LINE_SHIFT)

/**
 * @brief Total number of storage lines in the buffer
 * @details Divides total storage size by line size to determine how many lines
 *          are tracked in the dirty mask. With HAL_STORAGE_SIZE typically 16KB
 *          and 8-byte lines, this equals 2048 lines.
 */
#define STORAGE_NUM_LINES (HAL_STORAGE_SIZE/STORAGE_LINE_SIZE)

/**
 * @class Storage
 * @brief SITL implementation of nonvolatile storage interface
 * 
 * @details This class provides emulated nonvolatile storage for SITL simulation,
 *          implementing the AP_HAL::Storage interface. It maintains an in-memory
 *          buffer that mirrors the storage contents and persists changes to disk.
 * 
 *          Architecture:
 *          - In-memory buffer (_buffer): Fast access cache of all storage data
 *          - Dirty-line tracking (_dirty_mask): Bitmap tracking modified lines
 *          - Backend selection: POSIX file, Flash simulation, or FRAM simulation
 *          - Write-through cache: Writes update buffer immediately, flush to disk later
 * 
 *          Storage Backends:
 *          1. POSIX (StorageBackend::SDCard): Direct file I/O to eeprom.bin
 *             - Simplest implementation, no erase cycles
 *             - Instant writes with standard file operations
 *          2. Flash (StorageBackend::Flash): AP_FlashStorage simulation
 *             - Emulates sector-based flash with erase operations
 *             - Uses callback functors for flash operations
 *          3. FRAM (StorageBackend::FRAM): AP_RAMTRON simulation
 *             - Emulates ferroelectric RAM with unlimited writes
 *             - No erase required, byte-level access
 * 
 *          Usage Pattern:
 *          - AP_Param system reads/writes parameters through this interface
 *          - Writes are buffered in memory and marked dirty
 *          - Timer tick (_timer_tick) periodically flushes dirty data to backend
 *          - Storage persists across SITL runs via filesystem file
 * 
 *          Storage Layout:
 *          - Total size: HAL_STORAGE_SIZE (typically 16KB)
 *          - Addressing: 16-bit offset (0 to HAL_STORAGE_SIZE-1)
 *          - Line granularity: 8 bytes (STORAGE_LINE_SIZE)
 *          - Dirty tracking: Per-line bitmap (STORAGE_NUM_LINES bits)
 * 
 *          Thread Safety:
 *          - All operations execute in main thread context
 *          - No explicit locking required for SITL single-threaded model
 *          - Writes are atomic at line granularity
 * 
 * @note Storage file location: Typically ./eeprom.bin in SITL working directory
 * 
 * @warning SITL storage differences from real hardware:
 *          - Writes complete immediately (no delays)
 *          - No wear leveling simulation
 *          - No power-loss corruption
 *          - No write cycle limitations
 *          - Host filesystem performance affects timing
 */
class HALSITL::Storage : public AP_HAL::Storage {
public:
    /**
     * @brief Initialize storage system
     * 
     * @details Initialization is handled lazily on first access rather than
     *          in this method. The storage backend is opened and created if
     *          necessary during the first read or write operation.
     * 
     * @note This override is intentionally empty as initialization occurs
     *       in _storage_open() called from read_block() or write_block()
     */
    void init() override {}
    
    /**
     * @brief Read a block of data from storage
     * 
     * @details Reads data from the in-memory buffer cache, which mirrors the
     *          persistent storage contents. If storage has not been initialized,
     *          this triggers opening of the storage backend and loading data.
     *          All reads are served from the buffer for maximum performance.
     * 
     * @param[out] dst    Destination buffer to receive data
     * @param[in]  src    Source offset in storage (0 to HAL_STORAGE_SIZE-1)
     * @param[in]  n      Number of bytes to read
     * 
     * @note Reads beyond storage bounds are undefined behavior
     * @note This operation is always synchronous and completes immediately
     */
    void read_block(void *dst, uint16_t src, size_t n) override;
    
    /**
     * @brief Write a block of data to storage
     * 
     * @details Writes data to the in-memory buffer and marks affected storage
     *          lines as dirty. The dirty lines are flushed to persistent backend
     *          during timer ticks (_timer_tick). This provides write-through cache
     *          behavior where writes are visible immediately but persisted later.
     * 
     *          Write Process:
     *          1. Copy data to _buffer at specified offset
     *          2. Mark affected lines dirty in _dirty_mask
     *          3. Return immediately (write completes synchronously to buffer)
     *          4. Timer tick later flushes dirty lines to backend
     * 
     * @param[in]  dst    Destination offset in storage (0 to HAL_STORAGE_SIZE-1)
     * @param[in]  src    Source buffer containing data to write
     * @param[in]  n      Number of bytes to write
     * 
     * @note Writes beyond storage bounds are undefined behavior
     * @note Line-aligned writes are most efficient (8-byte boundaries)
     */
    void write_block(uint16_t dst, const void* src, size_t n) override;
    
    /**
     * @brief Get direct pointer to storage buffer
     * 
     * @details Provides direct memory access to the internal storage buffer for
     *          efficient bulk operations. Callers can read/write the buffer
     *          directly but must be aware this bypasses dirty tracking.
     * 
     * @param[out] ptr   Reference to pointer, set to _buffer address on success
     * @param[out] size  Reference to size, set to HAL_STORAGE_SIZE on success
     * 
     * @return true if storage is available and pointer is valid
     * @return false if storage initialization failed
     * 
     * @warning Direct buffer access bypasses dirty-line tracking. Modifications
     *          through this pointer may not be persisted unless explicitly marked
     *          dirty or flushed.
     * @warning Pointer remains valid only while Storage object exists
     */
    bool get_storage_ptr(void *&ptr, size_t &size) override;

    /**
     * @brief Periodic timer tick for background storage operations
     * 
     * @details Called periodically by the scheduler to perform background storage
     *          maintenance. This method flushes dirty lines to the persistent backend,
     *          ensuring modified data is written to disk. The flush rate is controlled
     *          to balance responsiveness with write efficiency.
     * 
     *          Operations performed:
     *          - Check for dirty lines in _dirty_mask
     *          - Write dirty lines to backend (POSIX/Flash/FRAM)
     *          - Clear dirty flags for successfully written lines
     *          - Monitor time since last write for backup operations
     * 
     * @note Called at scheduler rate (typically every few milliseconds)
     * @note Implements rate limiting to avoid excessive disk I/O
     */
    void _timer_tick(void) override;
    
    /**
     * @brief Check if storage backend is healthy and operational
     * 
     * @details Returns the health status of the storage system. For POSIX backend,
     *          this is always true once initialized. For Flash backend, this reflects
     *          whether flash operations are succeeding without errors.
     * 
     * @return true if storage is operational and accepting reads/writes
     * @return false if storage backend has failed or is unavailable
     * 
     * @note For Flash backend, repeated failures set _flash_failed flag
     * @note POSIX and FRAM backends typically always return true
     */
    bool healthy(void) override;

private:
    /**
     * @enum StorageBackend
     * @brief Identifies which storage backend implementation is in use
     * 
     * @details SITL can emulate different nonvolatile storage types depending on
     *          compile-time configuration. This enum tracks which backend was
     *          successfully initialized and is actively providing storage.
     */
    enum class StorageBackend: uint8_t {
        None,     ///< No backend initialized yet or initialization failed
        FRAM,     ///< FRAM (Ferroelectric RAM) simulation via AP_RAMTRON
        Flash,    ///< Flash memory simulation via AP_FlashStorage with sector erase
        SDCard,   ///< POSIX file-based storage (name reflects embedded SD card usage)
    };
    
    /** @brief Currently active storage backend type */
    StorageBackend _initialisedType = StorageBackend::None;

    /**
     * @brief Create storage file if it doesn't exist
     * @details Initializes new storage file with default/empty content
     */
    void _storage_create(void);
    
    /**
     * @brief Open existing storage file or create if necessary
     * @details Attempts to open backend storage, creates if missing, and loads
     *          initial data into _buffer. Selects backend based on compile options.
     */
    void _storage_open(void);
    
    /**
     * @brief Save backup copy of storage contents
     * @details Creates backup file for storage recovery or debugging
     */
    void _save_backup(void);
    
    /**
     * @brief Mark storage lines as dirty for later flush
     * 
     * @param[in] loc    Starting byte offset of modified region
     * @param[in] length Number of bytes modified
     * 
     * @details Calculates which storage lines are affected by a write operation
     *          and sets corresponding bits in _dirty_mask. Dirty lines are later
     *          flushed to persistent storage during timer ticks.
     */
    void _mark_dirty(uint16_t loc, uint16_t length);
    
    /**
     * @brief In-memory buffer mirroring storage contents
     * @details All reads and writes operate on this buffer. Buffer is loaded from
     *          persistent storage on initialization and dirty lines are flushed
     *          back periodically. Aligned to 4-byte boundary for efficient access.
     */
    uint8_t _buffer[HAL_STORAGE_SIZE] __attribute__((aligned(4)));
    
    /**
     * @brief Bitmap tracking which storage lines have been modified
     * @details Each bit represents one STORAGE_LINE_SIZE (8-byte) line. Set bits
     *          indicate lines needing flush to persistent backend. Cleared after
     *          successful write to backend.
     */
    Bitmask<STORAGE_NUM_LINES> _dirty_mask;

    /**
     * @brief Timestamp of last time all dirty lines were flushed (milliseconds)
     * @details Used to trigger backup operations when storage has been clean for
     *          extended periods, ensuring data persistence.
     */
    uint32_t _last_empty_ms;

#if STORAGE_USE_FLASH
    /**
     * @brief Flash backend: Write data to simulated flash sector
     * 
     * @param[in] sector Sector number to write
     * @param[in] offset Byte offset within sector
     * @param[in] data   Data buffer to write
     * @param[in] length Number of bytes to write
     * 
     * @return true if write succeeded, false on failure
     * 
     * @details Callback for AP_FlashStorage. Writes data to simulated flash file,
     *          emulating flash program operation. Called by AP_FlashStorage when
     *          flushing data to persistent storage.
     */
    bool _flash_write_data(uint8_t sector, uint32_t offset, const uint8_t *data, uint16_t length);
    
    /**
     * @brief Flash backend: Read data from simulated flash sector
     * 
     * @param[in]  sector Sector number to read
     * @param[in]  offset Byte offset within sector
     * @param[out] data   Buffer to receive read data
     * @param[in]  length Number of bytes to read
     * 
     * @return true if read succeeded, false on failure
     * 
     * @details Callback for AP_FlashStorage. Reads data from simulated flash file,
     *          emulating flash read operation.
     */
    bool _flash_read_data(uint8_t sector, uint32_t offset, uint8_t *data, uint16_t length);
    
    /**
     * @brief Flash backend: Erase simulated flash sector
     * 
     * @param[in] sector Sector number to erase
     * 
     * @return true if erase succeeded, false on failure
     * 
     * @details Callback for AP_FlashStorage. Erases (fills with 0xFF) an entire
     *          flash sector, emulating flash erase operation required before writes.
     */
    bool _flash_erase_sector(uint8_t sector);
    
    /**
     * @brief Flash backend: Check if erase operation is permitted
     * 
     * @return true if erase is allowed, false if erase should be deferred
     * 
     * @details Callback for AP_FlashStorage. In SITL, erases are always permitted
     *          as there are no real-time constraints.
     */
    bool _flash_erase_ok(void);

    /**
     * @brief Flag indicating flash backend has experienced failures
     * @details Set to true if flash operations fail. Used by healthy() to report
     *          storage status. Reset on successful re-initialization.
     */
    bool _flash_failed;
    
    /**
     * @brief Timestamp of last flash re-initialization attempt (milliseconds)
     * @details Used to rate-limit re-initialization attempts after flash failures
     */
    uint32_t _last_re_init_ms;

    /**
     * @brief AP_FlashStorage instance providing flash simulation layer
     * 
     * @details Initialized with functors binding to this Storage class methods,
     *          allowing AP_FlashStorage to call back for actual I/O operations.
     *          Manages wear leveling, sector allocation, and flash-specific logic.
     * 
     *          Constructor parameters:
     *          - _buffer: Storage buffer to manage
     *          - HAL_FLASH_SECTOR_SIZE: Simulated flash sector size
     *          - Functors: Callbacks for write, read, erase, and erase_ok operations
     * 
     * @note AP_FlashStorage provides algorithm, this class provides I/O backend
     */
    AP_FlashStorage _flash{_buffer,
            HAL_FLASH_SECTOR_SIZE,
            FUNCTOR_BIND_MEMBER(&Storage::_flash_write_data, bool, uint8_t, uint32_t, const uint8_t *, uint16_t),
            FUNCTOR_BIND_MEMBER(&Storage::_flash_read_data, bool, uint8_t, uint32_t, uint8_t *, uint16_t),
            FUNCTOR_BIND_MEMBER(&Storage::_flash_erase_sector, bool, uint8_t),
            FUNCTOR_BIND_MEMBER(&Storage::_flash_erase_ok, bool)};

    /**
     * @brief Load initial data from flash backend into buffer
     * @details Called during initialization to populate _buffer with data from
     *          simulated flash file. Invokes AP_FlashStorage load sequence.
     */
    void _flash_load(void);
    
    /**
     * @brief Write single dirty line to flash backend
     * 
     * @param[in] line Line number (0 to STORAGE_NUM_LINES-1) to write
     * 
     * @details Writes one 8-byte storage line from _buffer to flash via AP_FlashStorage.
     *          Called during timer tick for each dirty line needing persistence.
     */
    void _flash_write(uint16_t line);
#endif

#if STORAGE_USE_POSIX
    /**
     * @brief POSIX backend: File descriptor for storage file
     * 
     * @details File descriptor for the storage file (typically eeprom.bin) used
     *          by the POSIX/SDCard backend. Opened during _storage_open() and used
     *          for direct read/write operations to persist storage data.
     * 
     * @note File persists in SITL working directory across simulator runs
     * @note -1 indicates file not opened, >=0 is valid file descriptor
     */
    int log_fd;
#endif

#if STORAGE_USE_FRAM
    /**
     * @brief FRAM backend: AP_RAMTRON instance providing FRAM simulation
     * 
     * @details Provides Ferroelectric RAM (FRAM) emulation if STORAGE_USE_FRAM
     *          is enabled. FRAM offers unlimited write cycles and byte-level
     *          access without erase requirements, simulating RAMTRON FRAM chips.
     * 
     * @note FRAM backend used when HAL_WITH_RAMTRON is defined
     * @note FRAM provides simpler interface than flash (no sector erase)
     */
    AP_RAMTRON fram;
#endif
};
