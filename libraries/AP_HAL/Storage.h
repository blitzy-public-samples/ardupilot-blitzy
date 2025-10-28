/**
 * @file Storage.h
 * @brief Persistent storage interface for parameters and configuration
 * 
 * Defines abstract interface for non-volatile storage used primarily for parameter
 * persistence across reboots. Implementations use internal flash, FRAM, or EEPROM
 * depending on board capabilities.
 */

#pragma once

#include <stdint.h>
#include "AP_HAL_Namespace.h"

/**
 * @class AP_HAL::Storage
 * @brief Abstract interface for persistent parameter storage
 * 
 * @details Provides block read/write operations for parameter data persistence.
 *          Storage characteristics:
 *          - Size: Typically 4-32KB depending on board
 *          - Layout: Managed by AP_Param system, not application code
 *          - Persistence: Survives power cycles and reboots
 *          - Write durability: Implementation-specific (flash has limited erase cycles)
 *          
 *          Storage types by platform:
 *          - ChibiOS: Internal flash with wear leveling (FlashStorage)
 *          - SITL: File-based storage in /tmp or user directory
 *          - Linux: EEPROM device or file storage
 *          
 *          Write patterns:
 *          - Parameters typically written infrequently (user changes, autotune)
 *          - Cached in RAM for read performance
 *          - Background thread handles physical writes to reduce latency
 *          
 *          Storage regions (defined by StorageManager):
 *          - Parameters: Main AP_Param storage area
 *          - Fence/Rally: Mission planning data
 *          - Custom: Application-specific persistent data
 * 
 * @note Storage writes may take milliseconds - use deferred write for time-critical code
 * @note Storage size fixed at compile time - cannot be extended at runtime
 * @warning Exceeding storage size corrupts parameters - respect StorageManager limits
 * @warning Write-heavy patterns exhaust flash erase cycles (typically 10K-100K cycles)
 * 
 * @see StorageManager for storage region allocation
 * @see AP_Param for parameter persistence implementation
 */
class AP_HAL::Storage {
public:
    /**
     * @brief Initialize storage subsystem and load cached data
     * 
     * @details Called once during HAL initialization before parameter system loads.
     *          Performs platform-specific storage initialization:
     *          - Opens storage device (flash, FRAM, file)
     *          - Loads storage contents into RAM cache for fast access
     *          - Validates storage integrity (checksums, magic numbers)
     *          - Initializes wear-leveling structures if applicable
     *          
     *          Initialization time varies by platform:
     *          - Flash-based: 10-50ms to read and cache
     *          - File-based (SITL): <1ms for file open
     *          - FRAM/EEPROM: 5-20ms depending on size
     * 
     * @note Must be called before any read_block() or write_block() operations
     * @note Called from HAL::init() during system startup
     * @note Blocking operation - may take several milliseconds
     */
    virtual void init() = 0;
    
    /**
     * @brief Erase entire storage to default values
     * 
     * @details Resets all storage to erased state (typically 0xFF for flash).
     *          Used during factory reset or storage format operations.
     *          Default implementation returns false (not supported).
     *          Platform-specific implementations may:
     *          - Erase physical storage media
     *          - Clear RAM cache
     *          - Reset wear-leveling structures
     *          - Mark storage as uninitialized
     *          
     *          After erase, parameter system must reload defaults.
     * 
     * @return true if erase successful, false if not supported or failed
     * 
     * @note Destructive operation - all parameters and configuration lost
     * @note May take significant time (seconds) on flash-based storage
     * @warning Cannot be undone - ensure backup if data recovery needed
     * @warning Vehicle will revert to default parameters after erase
     */
    virtual bool erase();
    
    /**
     * @brief Read data block from persistent storage
     * 
     * @details Copies n bytes from storage offset src to destination buffer dst.
     *          Reads from RAM cache maintained by storage implementation - fast operation.
     *          No I/O blocking or physical media access during read.
     *          
     *          Performance characteristics:
     *          - Typical execution time: <1 microsecond (RAM copy)
     *          - No mutex locking in most implementations
     *          - Safe to call from any thread context
     *          
     *          Common usage:
     *          - AP_Param system reads parameter values
     *          - Mission/fence data retrieval
     *          - Configuration data loading
     * 
     * @param[out] dst Destination buffer for read data (must be at least n bytes)
     * @param[in]  src Byte offset in storage space (0 to storage_size-1)
     * @param[in]  n   Number of bytes to read
     * 
     * @note Reads from RAM cache - fast operation (microseconds)
     * @note No bounds checking - caller must ensure src+n <= storage_size
     * @note Destination buffer must be pre-allocated with sufficient size
     * @warning Reading beyond storage size has undefined behavior
     * @warning Buffer overrun if dst smaller than n bytes
     */
    virtual void read_block(void *dst, uint16_t src, size_t n) = 0;
    
    /**
     * @brief Write data block to persistent storage
     * 
     * @details Copies n bytes from source buffer src to storage offset dst.
     *          Updates RAM cache immediately for read consistency.
     *          Physical write to storage media may be deferred to background thread
     *          to avoid blocking caller (especially important for flash storage).
     *          
     *          Write behavior by platform:
     *          - Flash-based: Deferred write with dirty tracking, coalesced writes
     *          - FRAM/EEPROM: May write immediately (fast write times)
     *          - File-based (SITL): Typically immediate write or buffered I/O
     *          
     *          Write optimization:
     *          - Multiple writes to same region coalesced before physical write
     *          - Dirty page tracking reduces unnecessary writes
     *          - Background thread batches writes for flash efficiency
     *          
     *          Performance characteristics:
     *          - RAM cache update: <1 microsecond
     *          - Physical write scheduling: <10 microseconds
     *          - Actual storage write: Deferred (milliseconds to seconds)
     * 
     * @param[in] dst Byte offset in storage space (0 to storage_size-1)
     * @param[in] src Source buffer containing data to write (at least n bytes)
     * @param[in] n   Number of bytes to write
     * 
     * @note Updates RAM cache immediately - subsequent reads see new data
     * @note Physical write may be deferred to background thread (_timer_tick)
     * @note Multiple writes within short time may be coalesced for flash efficiency
     * @note No bounds checking - caller must ensure dst+n <= storage_size
     * @warning Writing beyond storage size corrupts memory or parameters
     * @warning Frequent writes to flash exhaust erase cycles (wear leveling helps)
     */
    virtual void write_block(uint16_t dst, const void* src, size_t n) = 0;
    
    /**
     * @brief Background timer callback for deferred storage operations
     * 
     * @details Called periodically (typically 1kHz) by scheduler to perform
     *          background storage maintenance operations:
     *          - Flush pending writes from RAM cache to physical storage
     *          - Process dirty page tracking and write coalescing
     *          - Manage wear-leveling updates for flash storage
     *          - Update storage health metrics
     *          
     *          Default implementation is empty (no background processing).
     *          Flash-based implementations use this for deferred write processing
     *          to avoid blocking write_block() callers with slow flash operations.
     *          
     *          Execution time budget:
     *          - Target: <100 microseconds per call
     *          - Flash write operations spread across multiple ticks
     *          - Rate limiting prevents excessive CPU usage
     * 
     * @note Called from 1kHz timer interrupt or high-priority scheduler task
     * @note Must execute quickly to avoid impacting real-time performance
     * @note Default implementation does nothing (empty function body)
     * 
     * @see AP_HAL::Scheduler for timer callback registration
     */
    virtual void _timer_tick(void) {};
    
    /**
     * @brief Check storage subsystem health status
     * 
     * @details Returns health status of storage system based on:
     *          - Physical media accessibility
     *          - Write operation success rate
     *          - Storage integrity (checksums, magic numbers)
     *          - Wear-leveling status for flash
     *          - Pending write queue depth
     *          
     *          Default implementation returns true (always healthy).
     *          Platform implementations may return false if:
     *          - Storage device not responding
     *          - Write failures detected
     *          - Flash wear level critical
     *          - Corruption detected
     *          
     *          Health status used by:
     *          - Pre-arm checks (warn if storage unhealthy)
     *          - Parameter save operations (retry logic)
     *          - System health monitoring
     * 
     * @return true if storage healthy, false if degraded or failed
     * 
     * @note Default implementation always returns true
     * @note Called periodically by system health monitoring
     * @note Does not affect read operations (reads from cache)
     */
    virtual bool healthy(void) { return true; }
    
    /**
     * @brief Get direct pointer to storage RAM cache (if available)
     * 
     * @details Provides direct read/write access to storage RAM cache for
     *          performance-critical operations that need to avoid copy overhead.
     *          Default implementation returns false (direct access not supported).
     *          
     *          When supported (returns true):
     *          - ptr is set to cache base address
     *          - size is set to total storage size in bytes
     *          - Caller can read/write cache directly
     *          - Caller must call write_block() to trigger persistence
     *          
     *          When not supported (returns false):
     *          - Use read_block() and write_block() instead
     *          - Typical for storage without RAM cache
     *          - Typical for memory-constrained platforms
     *          
     *          Direct access advantages:
     *          - Zero-copy parameter access
     *          - Reduced CPU overhead for bulk operations
     *          - Simplified serialization for complex data structures
     *          
     *          Direct access risks:
     *          - Cache coherency responsibility on caller
     *          - Must call write_block() for persistence
     *          - Pointer invalid after storage reinitialization
     * 
     * @param[out] ptr  Reference to pointer, set to cache address if available
     * @param[out] size Reference to size_t, set to storage size if available
     * 
     * @return true if direct access supported and ptr/size valid, false otherwise
     * 
     * @note Default implementation returns false (not supported)
     * @note Caller must handle both supported and unsupported cases
     * @note Changes via pointer not automatically persisted - call write_block()
     * @warning Direct writes bypass dirty tracking - must call write_block() manually
     * @warning Pointer may become invalid if storage reinitialized
     */
    virtual bool get_storage_ptr(void *&ptr, size_t &size) { return false; }
};
