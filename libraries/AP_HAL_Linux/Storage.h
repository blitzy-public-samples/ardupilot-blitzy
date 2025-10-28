/**
 * @file Storage.h
 * @brief File-based persistent storage emulation for parameters and settings
 * 
 * Implements EEPROM-like storage using a regular file on Linux filesystem.
 * Provides atomic write operations with dirty line tracking and periodic
 * flush to disk for parameter persistence across reboots.
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

/**
 * Total storage size in bytes (matches HAL_STORAGE_SIZE)
 */
#define LINUX_STORAGE_SIZE HAL_STORAGE_SIZE

/**
 * Maximum bytes to write in single operation
 */
#define LINUX_STORAGE_MAX_WRITE 512

/**
 * Shift value for storage line size calculation (2^9 = 512 bytes)
 */
#define LINUX_STORAGE_LINE_SHIFT 9

/**
 * Size of one storage line in bytes (512 bytes)
 * Storage organized in lines for dirty tracking granularity
 */
#define LINUX_STORAGE_LINE_SIZE (1<<LINUX_STORAGE_LINE_SHIFT)

/**
 * Number of storage lines (total size / line size)
 */
#define LINUX_STORAGE_NUM_LINES (LINUX_STORAGE_SIZE/LINUX_STORAGE_LINE_SIZE)

namespace Linux {

/**
 * @class Linux::Storage
 * @brief File-based persistent storage for parameters and EEPROM emulation
 * 
 * @details Emulates EEPROM/flash storage using a regular file on Linux filesystem.
 *          Parameters, fence points, and other persistent settings stored here.
 *          
 *          Storage architecture:
 *          - Fixed-size memory buffer (typically 4-16KB)
 *          - Backed by file: "storage" in data directory
 *          - Dirty line tracking: Bitmask tracks which 512-byte lines modified
 *          - Periodic flush: _timer_tick() writes dirty lines to file
 *          - Atomic writes: pwrite() ensures partial-write safety
 *          
 *          Line-based dirty tracking:
 *          - Storage divided into 512-byte lines (LINUX_STORAGE_LINE_SIZE)
 *          - Dirty mask: One bit per line (32 lines max with 32-bit mask)
 *          - Write operation sets dirty bit for affected lines
 *          - Flush writes only dirty lines, reducing I/O
 *          
 *          File persistence:
 *          - Storage file created on first init if not exists
 *          - Reads entire file into buffer at init
 *          - Writes back only modified lines periodically
 *          - Typical flush interval: 1 second (via _timer_tick)
 *          
 *          Thread safety:
 *          - Buffer reads are direct (fast, no locking)
 *          - Writes set dirty bits and return (deferred flush)
 *          - Flush happens in timer thread (_timer_tick)
 *          - volatile dirty_mask for thread-safe flag updates
 *          
 *          Data integrity:
 *          - pwrite() used for atomic line writes
 *          - Writes at 512-byte boundaries
 *          - File descriptor kept open to avoid repeated open/close
 *          - No explicit sync() - relies on OS page cache
 *          
 *          Storage layout (managed by AP_Param):
 *          - Offset 0: Parameter data
 *          - Various offsets: Fence, rally points, mission items
 *          - Layout defined in AP_Param system, not here
 *          
 *          Typical usage (via AP_Param):
 *          ```cpp
 *          // Read parameter from storage
 *          AP_Float param;
 *          param.load();  // Uses hal.storage->read_block()
 *          
 *          // Write parameter to storage
 *          param.set_and_save(1.23f);  // Uses hal.storage->write_block()
 *          // Actual file write happens asynchronously in _timer_tick()
 *          ```
 * 
 * @note File location: <datadir>/storage (e.g., /tmp/storage for SITL)
 * @note File created if not exists, zero-filled
 * @warning Data loss possible on power failure (no fsync by default)
 * @warning Limited to 32 lines (16KB) due to 32-bit dirty mask
 * 
 * @see AP_Param for parameter management using this storage
 */
class Storage : public AP_HAL::Storage
{
public:
    /**
     * @brief Constructor initializes storage with closed file descriptor
     * 
     * @note File not opened until init() called
     */
    Storage() : _fd(-1),_dirty_mask(0) { }

    /**
     * @brief Downcast helper from generic Storage pointer
     * 
     * @param[in] storage Pointer to AP_HAL::Storage base class
     * @return Storage* Downcasted pointer to Linux implementation
     */
    static Storage *from(AP_HAL::Storage *storage) {
        return static_cast<Storage*>(storage);
    }


    /**
     * @brief Initialize storage system and open storage file
     * 
     * @details Initialization sequence:
     *          1. Determine storage file path (based on data directory)
     *          2. Open or create storage file
     *          3. Read entire file into memory buffer
     *          4. Zero-fill if file is smaller than LINUX_STORAGE_SIZE
     *          5. Mark as initialized
     *          
     *          File creation:
     *          - If file doesn't exist, create and zero-fill
     *          - If file exists but too small, extend and zero-fill
     *          - If file exists and correct size, read contents
     * 
     * @note Called once during HAL initialization
     * @note File kept open for lifetime of program
     * @warning Prints error and continues if file open fails (storage disabled)
     */
    void init() override;

    /**
     * @brief Read single byte from storage
     * 
     * @param[in] loc Offset in bytes (0 to LINUX_STORAGE_SIZE-1)
     * @return uint8_t Byte value at specified location
     * 
     * @note Direct buffer read (no file I/O)
     * @note Fast operation (no locking required)
     */
    uint8_t  read_byte(uint16_t loc);
    
    /**
     * @brief Read 16-bit word from storage (little-endian)
     * 
     * @param[in] loc Offset in bytes (0 to LINUX_STORAGE_SIZE-2)
     * @return uint16_t Word value at specified location
     * 
     * @note Direct buffer read, little-endian byte order
     */
    uint16_t read_word(uint16_t loc);
    
    /**
     * @brief Read 32-bit doubleword from storage (little-endian)
     * 
     * @param[in] loc Offset in bytes (0 to LINUX_STORAGE_SIZE-4)
     * @return uint32_t Doubleword value at specified location
     * 
     * @note Direct buffer read, little-endian byte order
     */
    uint32_t read_dword(uint16_t loc);
    
    /**
     * @brief Read block of data from storage
     * 
     * @param[out] dst Destination buffer
     * @param[in] src Source offset in storage
     * @param[in] n Number of bytes to read
     * 
     * @note Direct buffer memcpy (no file I/O)
     * @note No bounds checking - caller must ensure valid range
     */
    void     read_block(void *dst, uint16_t src, size_t n) override;

    /**
     * @brief Write single byte to storage
     * 
     * @param[in] loc Offset in bytes
     * @param[in] value Byte value to write
     * 
     * @note Updates buffer and marks line dirty
     * @note Actual file write deferred to _timer_tick()
     */
    void write_byte(uint16_t loc, uint8_t value);

    /**
     * @brief Write 16-bit word to storage (little-endian)
     * 
     * @param[in] loc Offset in bytes
     * @param[in] value Word value to write
     * 
     * @note Updates buffer and marks line(s) dirty
     * @note Word may span two lines if not aligned
     */
    void write_word(uint16_t loc, uint16_t value);

    /**
     * @brief Write 32-bit doubleword to storage (little-endian)
     * 
     * @param[in] loc Offset in bytes
     * @param[in] value Doubleword value to write
     * 
     * @note Updates buffer and marks line(s) dirty
     * @note May span multiple lines if not aligned
     */
    void write_dword(uint16_t loc, uint32_t value);

    /**
     * @brief Write block of data to storage
     * 
     * @param[in] dst Destination offset in storage
     * @param[in] src Source buffer
     * @param[in] n Number of bytes to write
     * 
     * @details Write process:
     *          1. Copy data to buffer
     *          2. Mark all affected lines as dirty
     *          3. Return immediately (deferred flush)
     *          
     *          Dirty line calculation:
     *          - Start line = dst / LINUX_STORAGE_LINE_SIZE
     *          - End line = (dst + n - 1) / LINUX_STORAGE_LINE_SIZE
     *          - Set all bits in dirty mask for affected lines
     * 
     * @note Actual file write deferred to _timer_tick()
     * @note Maximum LINUX_STORAGE_MAX_WRITE bytes per call
     */
    void write_block(uint16_t dst, const void* src, size_t n) override;

    /**
     * @brief Get direct pointer to storage buffer (not recommended)
     * 
     * @param[out] ptr Pointer to buffer (if supported)
     * @param[out] size Size of buffer
     * @return bool True if direct access supported
     * 
     * @note Bypasses dirty tracking - use read/write methods instead
     * @note Provided for legacy compatibility only
     */
    bool get_storage_ptr(void *&ptr, size_t &size) override;

    /**
     * @brief Timer tick callback to flush dirty lines to file
     * 
     * @details Flush process:
     *          1. Check dirty mask for modified lines
     *          2. For each dirty line:
     *             a. Calculate file offset (line_num * LINE_SIZE)
     *             b. Write line to file using pwrite()
     *             c. Clear dirty bit
     *          3. Return when all dirty lines written
     *          
     *          Rate limiting:
     *          - Called at 100Hz from scheduler timer thread
     *          - Writes max LINUX_STORAGE_MAX_WRITE bytes per tick
     *          - Spreads large writes across multiple ticks
     * 
     * @note Runs in timer thread context, not main thread
     * @note pwrite() used for atomic writes at line boundaries
     * @note No explicit fsync() - relies on OS buffering and periodic sync
     */
    virtual void _timer_tick(void) override;

protected:
    /**
     * @brief Mark storage lines as dirty for deferred flush
     * 
     * @param[in] loc Starting offset of modified region
     * @param[in] length Length of modified region in bytes
     * 
     * @details Dirty mask updates:
     *          - Calculate affected line range: start_line to end_line
     *          - Set corresponding bits in 32-bit _dirty_mask
     *          - Bits set atomically using bitwise OR
     *          
     *          Example: Writing 200 bytes at offset 600
     *          - Affects lines 1 and 2 (600/512=1, 800/512=1)
     *          - Sets bits 1 and 2 in dirty mask
     * 
     * @note Thread-safe via volatile dirty_mask
     */
    void _mark_dirty(uint16_t loc, uint16_t length);

    /**
     * @brief Create storage file if it doesn't exist
     * 
     * @param[in] dpath Path to storage file
     * @return int File descriptor, or -1 on error
     * 
     * @details File creation:
     *          - Open with O_RDWR | O_CREAT (create if not exists)
     *          - Truncate to LINUX_STORAGE_SIZE if needed
     *          - Zero-fill entire file for initial state
     * 
     * @note File permissions: 0666 (rw-rw-rw-)
     */
    int _storage_create(const char *dpath);

    /**
     * File descriptor for storage file (-1 if not open)
     */
    int _fd;

    /**
     * Initialization complete flag
     */
    volatile bool _initialised;

    /**
     * Dirty line bitmask (bit N = line N needs flush)
     * Maximum 32 lines (32-bit mask)
     */
    volatile uint32_t _dirty_mask;

    /**
     * In-memory storage buffer (entire storage contents)
     * Reads/writes operate on this buffer, periodic flush writes to file
     */
    uint8_t _buffer[LINUX_STORAGE_SIZE];
};

}
