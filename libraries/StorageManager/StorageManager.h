/*
   Please contribute your ideas! See https://ardupilot.org/dev for details

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
 * @file StorageManager.h
 * @brief Non-volatile storage management and abstraction layer
 * 
 * @details This file provides the StorageManager and StorageAccess classes that
 *          manage non-volatile storage areas for ArduPilot. The storage system
 *          provides backwards-compatible mapping of logical storage offsets to
 *          physical storage, supporting multiple storage types with automatic
 *          layout management.
 * 
 *          Architecture:
 *          - StorageManager: Holds the layout of non-volatile storage areas
 *          - StorageAccess: Provides typed access to specific storage areas
 *          - Storage areas are allocated based on available HAL_STORAGE_SIZE
 *          - Supports storage backends: internal flash, SD card (optional)
 * 
 *          Storage Types:
 *          - Parameters: Vehicle configuration parameters (AP_Param)
 *          - Fence: Geofence boundary definitions
 *          - Rally: Rally point locations for return-to-launch
 *          - Mission: Waypoint mission data
 *          - Keys: Cryptographic keys for signing
 *          - BindInfo: RC receiver binding information
 *          - CANDNA: DroneCAN dynamic node allocation data
 *          - ParamBak: Parameter backup storage
 * 
 *          Wear Leveling:
 *          The underlying HAL storage implementation provides wear leveling
 *          to extend flash memory lifetime. StorageManager organizes logical
 *          storage areas that are mapped to physical storage by the HAL.
 * 
 *          Storage Backends:
 *          - Internal Flash: Primary storage via HAL (FlashStorage)
 *          - SD Card: Optional storage for large areas like missions
 * 
 * @note Storage layout is determined at compile time based on HAL_STORAGE_SIZE
 * @warning Changing storage layout can invalidate existing stored data
 * 
 * @see AP_HAL::Storage
 * @see AP_Param
 * 
 * Source: libraries/StorageManager/StorageManager.h
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig_config.h>

/**
 * @def STORAGE_NUM_AREAS
 * @brief Number of storage areas allocated based on available storage size
 * 
 * @details The number of storage areas is determined at compile time based on
 *          HAL_STORAGE_SIZE. Boards with limited storage (4k) use fewer areas
 *          with combined functionality, while boards with larger storage can
 *          allocate separate areas for each storage type.
 * 
 *          Storage Area Allocation by Size:
 *          - >= 32KB: 18 areas (full feature set with multiple backups)
 *          - >= 16KB: 15 areas (full feature set)
 *          - >= 15KB with CAN: 12 areas (includes CAN DNA storage)
 *          - >= 15KB: 11 areas (standard configuration)
 *          - >= 8KB:  10 areas (reduced backup areas)
 *          - >= 4KB:   4 areas (minimal: param, fence, rally, mission)
 *          - > 0:      1 area (emergency: parameters only)
 * 
 * @note Actual area definitions are in StorageManager.cpp layout table
 * @warning Boards with < 4KB storage may have limited functionality
 */
#if HAL_STORAGE_SIZE >= 32768
#define STORAGE_NUM_AREAS 18
#elif HAL_STORAGE_SIZE >= 16384
#define STORAGE_NUM_AREAS 15
#elif HAL_STORAGE_SIZE >= 15360 && defined(HAL_NUM_CAN_IFACES)
#define STORAGE_NUM_AREAS 12
#elif HAL_STORAGE_SIZE >= 15360
#define STORAGE_NUM_AREAS 11
#elif HAL_STORAGE_SIZE >= 8192
#define STORAGE_NUM_AREAS 10
#elif HAL_STORAGE_SIZE >= 4096
#define STORAGE_NUM_AREAS 4
#elif HAL_STORAGE_SIZE > 0
#define STORAGE_NUM_AREAS 1
#else
#error "Unsupported storage size"
#endif

/**
 * @class StorageManager
 * @brief Manages the layout of non-volatile storage areas
 * 
 * @details StorageManager maintains the static layout table that maps logical
 *          storage types to physical storage offsets and sizes. The layout is
 *          determined at compile time based on available HAL_STORAGE_SIZE and
 *          provides backwards compatibility when accessing stored data.
 * 
 *          The storage manager is used by StorageAccess objects to locate and
 *          access specific storage areas. Each storage area has a type, offset,
 *          and length defined in the static layout table.
 * 
 *          Storage Organization:
 *          - Layout table maps types to physical offsets
 *          - Multiple areas can share the same type
 *          - Offsets are into HAL storage (typically flash)
 *          - Sizes are fixed at compile time
 * 
 *          Thread Safety:
 *          StorageManager itself is stateless (static layout table). Thread
 *          safety for actual storage I/O is provided by the HAL storage layer.
 * 
 * @note Layout table is defined in StorageManager.cpp
 * @warning Modifying storage layout requires careful migration planning
 * 
 * @see StorageAccess
 * @see AP_HAL::Storage
 */
class StorageManager {
    friend class StorageAccess;
public:
    /**
     * @enum StorageType
     * @brief Defines the different types of storage areas
     * 
     * @details Each storage type represents a different category of data that
     *          needs to be persisted across reboots. The storage manager can
     *          allocate multiple areas for the same type if space permits.
     */
    enum StorageType {
        StorageParam   = 0,  ///< Vehicle parameters (AP_Param system)
        StorageFence   = 1,  ///< Geofence boundary points
        StorageRally   = 2,  ///< Rally point locations for RTL
        StorageMission = 3,  ///< Mission waypoints and commands
        StorageKeys    = 4,  ///< Cryptographic keys for signing
        StorageBindInfo= 5,  ///< RC receiver binding information
        StorageCANDNA  = 6,  ///< DroneCAN dynamic node allocation data
        StorageParamBak = 7  ///< Parameter backup area
    };

    /**
     * @brief Erase entire storage area
     * 
     * @details Erases all storage areas by writing zeros to the entire
     *          HAL storage space. This operation is typically performed
     *          during factory reset or when recovering from corrupted storage.
     * 
     *          The erase operation:
     *          - Writes zeros to all storage locations
     *          - Invalidates all stored data (parameters, missions, etc.)
     *          - Requires reboot to take full effect
     *          - Cannot be undone
     * 
     * @note This is a destructive operation that clears ALL stored data
     * @warning Vehicle must be rearmed and reconfigured after erase
     * @warning Do not call during flight
     * 
     * @see AP_HAL::Storage::write_block()
     */
    static void erase(void);

    /**
     * @brief Check if the last storage I/O operation failed
     * 
     * @details Returns the status of the most recent storage I/O operation.
     *          This flag is set by StorageAccess when read or write operations
     *          fail, typically due to hardware issues or corrupted storage.
     * 
     *          Failure conditions include:
     *          - Flash read/write errors
     *          - SD card I/O failures (if using SD storage)
     *          - Timeout during storage operations
     * 
     * @return true if last storage operation failed
     * @return false if last storage operation succeeded
     * 
     * @note This is a global flag shared by all storage operations
     * @warning Persistent failures indicate hardware problems
     * 
     * @see StorageAccess::read_block()
     * @see StorageAccess::write_block()
     */
    static bool storage_failed(void) {
        return last_io_failed;
    }

private:
    /// Global flag indicating if last storage I/O operation failed
    static bool last_io_failed;

    /**
     * @struct StorageArea
     * @brief Defines a single storage area in the layout table
     * 
     * @details Each StorageArea entry maps a storage type to a physical
     *          location (offset) and size (length) in HAL storage. Multiple
     *          areas can have the same type for redundancy or expansion.
     */
    struct StorageArea {
        StorageType type;    ///< Type of data stored in this area
        uint16_t    offset;  ///< Byte offset into HAL storage
        uint16_t    length;  ///< Size of area in bytes
    };

    /**
     * @brief Static storage layout table
     * 
     * @details Defines the complete layout of storage areas for this board.
     *          The layout is determined at compile time based on STORAGE_NUM_AREAS
     *          and provides backwards-compatible access to stored data.
     * 
     *          Layout Table Structure:
     *          - Each entry defines type, offset, and length
     *          - Areas are non-overlapping
     *          - Multiple areas can have same type
     *          - Total size must not exceed HAL_STORAGE_SIZE
     * 
     * @note Defined in StorageManager.cpp with board-specific values
     * @see StorageArea
     */
    static const StorageArea layout[STORAGE_NUM_AREAS];
};

/**
 * @class StorageAccess
 * @brief Provides typed access to a specific storage area
 * 
 * @details StorageAccess is a lightweight accessor object that provides
 *          read/write access to a specific storage type. It finds all storage
 *          areas of the requested type in the StorageManager layout and provides
 *          a contiguous view of the combined storage space.
 * 
 *          Key Features:
 *          - Type-safe access to storage areas
 *          - Combines multiple areas of same type
 *          - Provides byte, word, and block access methods
 *          - Optional SD card storage support for large areas
 *          - Automatic endianness handling for multi-byte values
 * 
 *          Lifecycle:
 *          1. Construct with desired StorageType
 *          2. Constructor calculates total size from layout
 *          3. Use read/write methods to access storage
 *          4. Object can be stack-allocated (no dynamic memory)
 * 
 *          Storage Access Pattern:
 *          - Offsets are relative to the storage type (0-based)
 *          - Automatically mapped to physical HAL storage offsets
 *          - Spans multiple areas if type appears multiple times in layout
 * 
 *          Thread Safety:
 *          - Not thread-safe by itself
 *          - HAL storage layer provides synchronization
 *          - SD card storage uses per-accessor semaphore
 * 
 * @note Objects may be stack-allocated and are NOT zero-initialized
 * @note Const methods can be called on const objects (storage is external)
 * @warning Always check return values for I/O operations
 * 
 * @see StorageManager
 * @see AP_HAL::Storage
 */
class StorageAccess {
public:
    /**
     * @brief Construct a storage accessor for a specific type
     * 
     * @details Creates a StorageAccess object for the specified storage type.
     *          The constructor scans the StorageManager layout table to find
     *          all areas matching the requested type and calculates the total
     *          available storage size.
     * 
     *          Initialization Process:
     *          - Searches layout table for matching type
     *          - Sums lengths of all matching areas
     *          - Stores total size for bounds checking
     *          - No actual storage I/O performed
     * 
     * @param[in] _type Type of storage to access
     * 
     * @note Constructor does not allocate memory or perform I/O
     * @note Object may be created on stack for temporary access
     * 
     * @see StorageManager::StorageType
     */
    StorageAccess(StorageManager::StorageType _type);

    /**
     * @brief Get total size of this storage area
     * 
     * @details Returns the total size in bytes of all storage areas that
     *          match this accessor's type. This is the sum of all matching
     *          area lengths from the StorageManager layout table.
     * 
     * @return Total size in bytes available for this storage type
     * 
     * @note Size is calculated once during construction
     * @note Return value is valid immediately after construction
     */
    uint16_t size(void) const { return total_size; }

    /**
     * @brief Read a block of data from storage
     * 
     * @details Reads n bytes from storage starting at the specified source
     *          offset (relative to this storage type) into the destination
     *          buffer. Automatically maps logical offsets to physical storage
     *          locations, handling areas that span multiple layout entries.
     * 
     *          Read Process:
     *          - Maps src offset to physical storage location(s)
     *          - May read from multiple storage areas if crossing boundary
     *          - Handles SD card storage if configured
     *          - Sets StorageManager::last_io_failed on error
     * 
     * @param[out] dst Destination buffer for read data
     * @param[in]  src Source offset within this storage type (bytes)
     * @param[in]  n   Number of bytes to read
     * 
     * @return true if read completed successfully
     * @return false if read failed (bounds check, I/O error, SD card error)
     * 
     * @note Offsets are relative to storage type, not physical storage
     * @note Reads beyond allocated size will fail
     * @warning Ensure dst buffer has at least n bytes available
     * @warning Check return value before using read data
     * 
     * @see write_block()
     * @see AP_HAL::Storage::read_block()
     */
    bool read_block(void *dst, uint16_t src, size_t n) const;

    /**
     * @brief Write a block of data to storage
     * 
     * @details Writes n bytes from the source buffer to storage starting at
     *          the specified destination offset (relative to this storage type).
     *          Automatically maps logical offsets to physical storage locations
     *          and handles wear-leveling through the HAL storage layer.
     * 
     *          Write Process:
     *          - Maps dst offset to physical storage location(s)
     *          - May write to multiple storage areas if crossing boundary
     *          - Handles SD card storage with dirty tracking
     *          - Sets StorageManager::last_io_failed on error
     *          - HAL layer handles wear leveling
     * 
     * @param[in] dst Destination offset within this storage type (bytes)
     * @param[in] src Source buffer containing data to write
     * @param[in] n   Number of bytes to write
     * 
     * @return true if write completed successfully
     * @return false if write failed (bounds check, I/O error, SD card error)
     * 
     * @note Offsets are relative to storage type, not physical storage
     * @note Writes beyond allocated size will fail
     * @note SD card writes are buffered and flushed periodically
     * @warning Ensure src buffer contains at least n valid bytes
     * @warning Write failures can indicate flash wear or hardware failure
     * 
     * @see read_block()
     * @see AP_HAL::Storage::write_block()
     */
    bool write_block(uint16_t dst, const void* src, size_t n) const;    

    /**
     * @brief Read a single byte from storage
     * 
     * @details Reads one byte from the specified location within this storage
     *          type. This is a convenience wrapper around read_block() for
     *          single-byte access.
     * 
     * @param[in] loc Offset within this storage type (bytes)
     * 
     * @return Byte value read from storage, or 0 if read fails
     * 
     * @note Returns 0 on failure (no error indication)
     * @note For error checking, use read_block() directly
     * 
     * @see read_uint8()
     * @see read_block()
     */
    uint8_t  read_byte(uint16_t loc) const;

    /**
     * @brief Read an 8-bit unsigned integer from storage
     * 
     * @details Alias for read_byte() providing type-specific naming.
     *          Reads a single unsigned byte from storage.
     * 
     * @param[in] loc Offset within this storage type (bytes)
     * 
     * @return 8-bit unsigned integer value, or 0 if read fails
     * 
     * @see read_byte()
     */
    uint8_t  read_uint8(uint16_t loc) const { return read_byte(loc); }

    /**
     * @brief Read a 16-bit unsigned integer from storage
     * 
     * @details Reads a 16-bit unsigned integer stored in little-endian format.
     *          Automatically handles endianness conversion on big-endian systems.
     *          Reads 2 bytes starting at the specified location.
     * 
     * @param[in] loc Offset within this storage type (bytes)
     * 
     * @return 16-bit unsigned integer value, or 0 if read fails
     * 
     * @note Storage format is little-endian regardless of platform
     * @note Returns 0 on failure (no error indication)
     * 
     * @see write_uint16()
     */
    uint16_t read_uint16(uint16_t loc) const;

    /**
     * @brief Read a 32-bit unsigned integer from storage
     * 
     * @details Reads a 32-bit unsigned integer stored in little-endian format.
     *          Automatically handles endianness conversion on big-endian systems.
     *          Reads 4 bytes starting at the specified location.
     * 
     * @param[in] loc Offset within this storage type (bytes)
     * 
     * @return 32-bit unsigned integer value, or 0 if read fails
     * 
     * @note Storage format is little-endian regardless of platform
     * @note Returns 0 on failure (no error indication)
     * 
     * @see write_uint32()
     */
    uint32_t read_uint32(uint16_t loc) const;

    /**
     * @brief Read a floating-point value from storage
     * 
     * @details Reads a 32-bit IEEE-754 floating-point value from storage.
     *          The value is stored in little-endian byte order and automatically
     *          converted on big-endian systems. Reads 4 bytes.
     * 
     * @param[in] loc Offset within this storage type (bytes)
     * 
     * @return Float value, or 0.0 if read fails
     * 
     * @note Storage format is IEEE-754 single precision
     * @note Byte order is little-endian
     * @note Returns 0.0 on failure (no error indication)
     * @warning NaN and infinity values are preserved if originally stored
     * 
     * @see write_float()
     */
    float read_float(uint16_t loc) const;

    /**
     * @brief Write a single byte to storage
     * 
     * @details Writes one byte to the specified location within this storage
     *          type. This is a convenience wrapper around write_block() for
     *          single-byte writes.
     * 
     * @param[in] loc   Offset within this storage type (bytes)
     * @param[in] value Byte value to write
     * 
     * @note No return value - failures are silent
     * @note For error checking, use write_block() directly
     * 
     * @see write_uint8()
     * @see write_block()
     */
    void write_byte(uint16_t loc, uint8_t value) const;

    /**
     * @brief Write an 8-bit unsigned integer to storage
     * 
     * @details Alias for write_byte() providing type-specific naming.
     *          Writes a single unsigned byte to storage.
     * 
     * @param[in] loc   Offset within this storage type (bytes)
     * @param[in] value 8-bit unsigned integer to write
     * 
     * @see write_byte()
     */
    void write_uint8(uint16_t loc, uint8_t value) const { return write_byte(loc, value); }

    /**
     * @brief Write a 16-bit unsigned integer to storage
     * 
     * @details Writes a 16-bit unsigned integer in little-endian format.
     *          Automatically handles endianness conversion on big-endian systems.
     *          Writes 2 bytes starting at the specified location.
     * 
     * @param[in] loc   Offset within this storage type (bytes)
     * @param[in] value 16-bit unsigned integer to write
     * 
     * @note Storage format is little-endian regardless of platform
     * @note No return value - failures are silent
     * 
     * @see read_uint16()
     */
    void write_uint16(uint16_t loc, uint16_t value) const;

    /**
     * @brief Write a 32-bit unsigned integer to storage
     * 
     * @details Writes a 32-bit unsigned integer in little-endian format.
     *          Automatically handles endianness conversion on big-endian systems.
     *          Writes 4 bytes starting at the specified location.
     * 
     * @param[in] loc   Offset within this storage type (bytes)
     * @param[in] value 32-bit unsigned integer to write
     * 
     * @note Storage format is little-endian regardless of platform
     * @note No return value - failures are silent
     * 
     * @see read_uint32()
     */
    void write_uint32(uint16_t loc, uint32_t value) const;

    /**
     * @brief Write a floating-point value to storage
     * 
     * @details Writes a 32-bit IEEE-754 floating-point value to storage in
     *          little-endian byte order. Automatically handles endianness
     *          conversion on big-endian systems. Writes 4 bytes.
     * 
     * @param[in] loc   Offset within this storage type (bytes)
     * @param[in] value Float value to write
     * 
     * @note Storage format is IEEE-754 single precision
     * @note Byte order is little-endian
     * @note No return value - failures are silent
     * @note NaN and infinity values can be stored and retrieved
     * 
     * @see read_float()
     */
    void write_float(uint16_t loc, float value) const;

    /**
     * @brief Copy entire contents from one storage area to another
     * 
     * @details Copies all data from the source storage area to this storage
     *          area. The areas do not need to be the same type, but this area
     *          must be at least as large as the source area. Used for backup
     *          operations and storage migration.
     * 
     *          Copy Process:
     *          - Reads entire source area
     *          - Writes to entire destination area
     *          - Truncates if destination is smaller (returns false)
     *          - Pads with zeros if destination is larger
     * 
     * @param[in] source Source storage area to copy from
     * 
     * @return true if copy completed successfully
     * @return false if copy failed (size mismatch or I/O error)
     * 
     * @note Source and destination can be different storage types
     * @note Destination must be at least as large as source
     * @warning This overwrites all data in destination area
     * 
     * @see read_block()
     * @see write_block()
     */
    bool copy_area(const StorageAccess &source) const;

    /**
     * @brief Attach a file on microSD card as backing storage
     * 
     * @details Configures this storage area to use a file on the microSD card
     *          as the backing store instead of internal flash. This is useful
     *          for large storage areas like missions that exceed available
     *          flash space. The file is created if it doesn't exist.
     * 
     *          SD Card Storage Features:
     *          - File-backed storage for large areas
     *          - Buffered I/O with dirty tracking
     *          - Periodic flush to SD card
     *          - Automatic file creation and sizing
     *          - Wear leveling provided by SD card controller
     * 
     *          Configuration:
     *          - Call once during initialization
     *          - File is created with specified size
     *          - Buffer allocated for I/O operations
     *          - Subsequent reads/writes use SD file
     * 
     * @param[in] fname       Path to storage file on SD card
     * @param[in] size_kbyte  Size of storage area in kilobytes
     * 
     * @return true if file attached successfully
     * @return false if attachment failed (SD not available, allocation failed)
     * 
     * @note Only available if AP_SDCARD_STORAGE_ENABLED is defined
     * @note Requires functional SD card mount
     * @note Allocates buffer for I/O operations
     * @warning File path must be valid and writable
     * @warning SD card failures will cause storage access failures
     * 
     * @see flush_file()
     */
    bool attach_file(const char *fname, uint16_t size_kbyte);

private:
    /// Storage type this accessor provides access to
    const StorageManager::StorageType type;
    
    /// Total size in bytes of all areas matching this type
    uint16_t total_size;

#if AP_SDCARD_STORAGE_ENABLED
    /**
     * @struct FileStorage
     * @brief SD card file backing store for large storage areas
     * 
     * @details When storage is backed by an SD card file, this structure
     *          maintains the file descriptor, I/O buffer, and dirty tracking
     *          bitmap. Provides buffered I/O to reduce SD card wear and
     *          improve performance.
     * 
     *          Buffer Management:
     *          - buffer: Memory buffer for file contents
     *          - bufsize: Size of buffer in bytes
     *          - dirty_mask: Bitmap tracking modified 1KB blocks (64 bits = 64KB max)
     * 
     *          Flushing Strategy:
     *          - Dirty blocks flushed periodically
     *          - last_clean_ms: Timestamp of last flush operation
     *          - Automatic flush on buffer overflow or timeout
     * 
     *          Error Tracking:
     *          - last_io_fail_ms: Timestamp of last I/O failure
     *          - Used to detect persistent SD card problems
     * 
     * @note Only used when AP_SDCARD_STORAGE_ENABLED is defined
     * @note Currently used primarily for StorageMission
     * @see attach_file()
     */
    struct FileStorage {
        HAL_Semaphore sem;           ///< Semaphore for thread-safe file access
        int fd;                      ///< File descriptor for open storage file
        uint8_t *buffer;             ///< Memory buffer for file contents
        uint32_t bufsize;            ///< Size of buffer in bytes
        uint32_t last_clean_ms;      ///< Timestamp of last buffer flush (milliseconds)
        uint32_t last_io_fail_ms;    ///< Timestamp of last I/O failure (milliseconds)
        uint64_t dirty_mask;         ///< Bitmap of dirty 1KB blocks (bit per KB)
    } *file;  ///< Pointer to file storage structure (nullptr if using flash)

    /**
     * @brief Flush dirty buffers to SD card file
     * 
     * @details Writes all dirty blocks from the memory buffer to the SD card
     *          file. Uses the dirty_mask bitmap to identify which 1KB blocks
     *          need to be written, minimizing unnecessary SD card writes.
     * 
     *          Flush Process:
     *          - Iterate through dirty_mask bits
     *          - Write each dirty 1KB block to file
     *          - Clear dirty bits on successful write
     *          - Update last_clean_ms timestamp
     *          - Set last_io_fail_ms on error
     * 
     * @note Only called when file storage is attached
     * @note Automatically called periodically during normal operation
     * @note Thread-safe via FileStorage::sem
     * 
     * @see attach_file()
     */
    void flush_file(void);
#endif
};
