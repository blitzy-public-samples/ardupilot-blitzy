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
 * @file AP_Filesystem_Mission.h
 * @brief Virtual filesystem backend providing mission, fence, and rally data as files
 * 
 * @details This file implements a virtual filesystem mounted at @MISSION that exposes
 *          mission waypoints, geofence polygons, and rally points as binary files
 *          accessible through MAVLink FTP. This provides an efficient alternative to
 *          the traditional MAVLink mission protocol for uploading/downloading large
 *          mission data sets.
 * 
 *          Virtual Files Exposed:
 *          - @MISSION/mission.dat: Mission waypoints from AP_Mission
 *          - @MISSION/fence.dat: Geofence polygon points from AC_Fence
 *          - @MISSION/rally.dat: Rally points from AP_Rally
 * 
 *          The binary format consists of a header (magic, type, options, start, count)
 *          followed by packed MAVLINK_MISSION_ITEM_INT structures. On write, data is
 *          buffered in memory and validated/committed atomically on close().
 * 
 *          Thread Safety: Mission storage access is protected with WITH_SEMAPHORE
 *          when committing data to AP_Mission/AC_Fence/AP_Rally.
 * 
 *          Performance: Reads are streamed directly from mission storage; writes use
 *          ExpandingString buffering to handle variable-size uploads efficiently.
 * 
 * Source: libraries/AP_Filesystem/AP_Filesystem_Mission.h
 * Source: libraries/AP_Filesystem/AP_Filesystem_Mission.cpp
 */

#pragma once

#include "AP_Filesystem_config.h"

#if AP_FILESYSTEM_MISSION_ENABLED

#include "AP_Filesystem_backend.h"
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Common/ExpandingString.h>

/**
 * @class AP_Filesystem_Mission
 * @brief Virtual filesystem backend exposing mission, fence, and rally data as files
 * 
 * @details AP_Filesystem_Mission provides a POSIX-like file interface to ArduPilot's
 *          mission storage systems, enabling efficient bulk upload/download via MAVLink FTP.
 * 
 *          Architecture:
 *          - Implements AP_Filesystem_Backend interface with open/close/read/write/lseek/stat
 *          - Maintains up to 4 simultaneous open file handles (max_open_file)
 *          - Read operations stream directly from AP_Mission/AC_Fence/AP_Rally
 *          - Write operations buffer to ExpandingString, validate and commit on close()
 * 
 *          Binary File Format:
 *          - Header (10 bytes):
 *            * uint16_t magic = 0x763d
 *            * uint16_t data_type (MAV_MISSION_TYPE_MISSION/FENCE/RALLY)
 *            * uint16_t options (e.g., NO_CLEAR flag)
 *            * uint16_t start (first waypoint number, 0 for full upload)
 *            * uint16_t num_items (total item count)
 *          - Body: Packed array of MAVLINK_MISSION_ITEM_INT structures (48 bytes each)
 * 
 *          Validation on Write Close:
 *          - Verifies magic number matches 0x763d
 *          - Checks num_items matches actual data length
 *          - Rejects any all-zero items (incomplete uploads)
 *          - Validates MAVLink mission item conversion
 *          - Checks DO_JUMP targets are within bounds
 *          - Atomically commits to storage or rolls back on error
 * 
 *          Supported Operations:
 *          - READ: Download current mission/fence/rally as binary file
 *          - WRITE: Upload new mission/fence/rally data (one write operation at a time)
 *          - STAT: Returns conservative size estimate (1MB) without scanning storage
 * 
 *          Timeout Behavior:
 *          - Open files auto-close after 30 seconds of inactivity (IDLE_TIMEOUT_MS)
 *          - last_op_ms updated on each read/write operation
 * 
 *          Concurrency:
 *          - Only one write operation allowed at a time across all file types
 *          - Multiple simultaneous reads supported
 *          - Mission storage access protected with semaphores during commit
 * 
 *          Error Handling:
 *          - Sets errno to POSIX error codes (ENOENT, EBADF, ENFILE, ENOSPC, EINVAL)
 *          - Returns -1 on error per POSIX convention
 *          - Write failures on close() prevent partial mission uploads
 * 
 * @note Mission uploads are NOT persistent until close() completes successfully
 * @warning Uploading invalid mission data will fail close() and leave original data intact
 * @warning Only one write operation permitted at a time to prevent race conditions
 * 
 * Source: libraries/AP_Filesystem/AP_Filesystem_Mission.cpp:41-535
 */
class AP_Filesystem_Mission : public AP_Filesystem_Backend
{
public:
    /**
     * @brief Open a virtual mission file for reading or writing
     * 
     * @details Opens one of the three virtual files (mission.dat, fence.dat, rally.dat)
     *          for read or write access. For writes, allocates an ExpandingString buffer
     *          to accumulate data until close(). For reads, prepares to stream data from
     *          the corresponding mission storage subsystem.
     * 
     *          File name mapping:
     *          - "mission.dat" → MAV_MISSION_TYPE_MISSION (AP_Mission)
     *          - "fence.dat" → MAV_MISSION_TYPE_FENCE (AC_Fence)
     *          - "rally.dat" → MAV_MISSION_TYPE_RALLY (AP_Rally)
     * 
     *          Concurrency: Only one write file can be open at a time across all types.
     *          Stale file handles (inactive >30s) are automatically closed.
     * 
     * @param[in] fname Filename: "mission.dat", "fence.dat", or "rally.dat"
     * @param[in] flags Open flags (O_RDONLY for read, O_WRONLY/O_RDWR for write)
     * @param[in] allow_absolute_paths Ignored for this virtual filesystem
     * 
     * @return File descriptor (0-3) on success, -1 on error
     * 
     * @note errno set to ENOENT (invalid filename), ENFILE (too many open files), or -1 (write conflict)
     * @note Write mode allocates memory for ExpandingString buffer
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Mission.cpp:41-84
     */
    int open(const char *fname, int flags, bool allow_absolute_paths = false) override;
    
    /**
     * @brief Close a mission file and commit write buffer if applicable
     * 
     * @details For read operations, simply marks the file descriptor as closed. For write
     *          operations, validates the complete buffered data and atomically commits to
     *          mission storage (AP_Mission/AC_Fence/AP_Rally).
     * 
     *          Write Validation Steps:
     *          1. Verify magic number (0x763d)
     *          2. Check num_items matches data length
     *          3. Reject any all-zero items (incomplete upload)
     *          4. Convert and validate each MAVLink mission item
     *          5. Check DO_JUMP command targets are within bounds
     *          6. Acquire storage semaphore and commit atomically
     * 
     *          On validation failure, buffered data is discarded and original storage
     *          remains unchanged. This ensures atomic commit semantics.
     * 
     * @param[in] fd File descriptor returned by open()
     * 
     * @return 0 on success, -1 on error
     * 
     * @note errno set to EBADF (invalid fd) or EINVAL (validation failure)
     * @note Write buffer is freed regardless of commit success/failure
     * @warning Validation failure on write close leaves original mission data intact
     * @warning DO_JUMP commands with invalid targets will cause close() to fail
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Mission.cpp:86-104
     * Source: libraries/AP_Filesystem/AP_Filesystem_Mission.cpp:363-410 (finish_upload)
     */
    int close(int fd) override;
    
    /**
     * @brief Read mission data from virtual file
     * 
     * @details Streams binary mission data from the underlying storage system. First
     *          reads the file header (if at offset 0), then streams packed
     *          MAVLINK_MISSION_ITEM_INT structures retrieved from mission storage.
     * 
     *          Data is generated on-demand by calling get_item() for each mission item.
     *          No pre-buffering is performed, making reads memory-efficient even for
     *          large missions.
     * 
     *          Binary Format Returned:
     *          - Bytes 0-9: Header (magic, data_type, options, start, num_items)
     *          - Bytes 10+: MAVLINK_MISSION_ITEM_INT structures (48 bytes each)
     * 
     * @param[in]  fd    File descriptor returned by open()
     * @param[out] buf   Buffer to receive data
     * @param[in]  count Maximum bytes to read
     * 
     * @return Number of bytes read on success, -1 on error
     * 
     * @note errno set to EBADF (invalid fd or fd opened for write)
     * @note Updates last_op_ms to prevent idle timeout
     * @note Returns 0 when all data has been read (EOF)
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Mission.cpp:119-178
     */
    int32_t read(int fd, void *buf, uint32_t count) override;
    
    /**
     * @brief Reposition file offset for read operations
     * 
     * @details Changes the file offset for subsequent read operations. Supports
     *          SEEK_SET (absolute) and SEEK_CUR (relative) positioning. SEEK_END
     *          is not supported as file size is not pre-calculated.
     * 
     * @param[in] fd     File descriptor returned by open()
     * @param[in] offset New offset (SEEK_SET) or offset delta (SEEK_CUR)
     * @param[in] whence SEEK_SET (0), SEEK_CUR (1), or SEEK_END (2 - not supported)
     * 
     * @return New file offset on success, -1 on error
     * 
     * @note errno set to EBADF (invalid fd) or EINVAL (SEEK_END not supported)
     * @note Seeking beyond file size is allowed but subsequent reads will return 0
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Mission.cpp:180-199
     */
    int32_t lseek(int fd, int32_t offset, int whence) override;
    
    /**
     * @brief Write mission data to buffer for later commit
     * 
     * @details Appends data to the ExpandingString write buffer. Data is not validated
     *          or committed until close(). The buffer automatically expands to accommodate
     *          the full upload based on the num_items field in the header.
     * 
     *          On receiving the first write containing the header, pre-expands the buffer
     *          to the full expected size to minimize reallocations during streaming upload.
     * 
     * @param[in] fd    File descriptor returned by open() in write mode
     * @param[in] buf   Data to write
     * @param[in] count Bytes to write
     * 
     * @return Number of bytes written on success, -1 on error
     * 
     * @note errno set to EBADF (invalid fd or read-only), ENOSPC (out of memory)
     * @note Updates last_op_ms to prevent idle timeout
     * @note Buffer expansion based on header num_items field for efficiency
     * @warning Data is not validated until close() - invalid data will cause close() to fail
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Mission.cpp:310-347
     */
    int32_t write(int fd, const void *buf, uint32_t count) override;
    
    /**
     * @brief Get file status for virtual mission file
     * 
     * @details Returns stat information for one of the three virtual files. Since
     *          calculating exact file size would require scanning entire mission storage,
     *          returns a conservative 1MB size estimate to satisfy FTP clients.
     * 
     * @param[in]  pathname Filename: "mission.dat", "fence.dat", or "rally.dat"
     * @param[out] stbuf    Stat buffer to fill
     * 
     * @return 0 on success, -1 on error
     * 
     * @note errno set to ENOENT (invalid filename)
     * @note st_size is set to 1MB (1024*1024) as conservative estimate
     * @note Actual file size depends on current number of mission items
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Mission.cpp:201-212
     */
    int stat(const char *pathname, struct stat *stbuf) override;

private:
    /**
     * @brief Maximum number of simultaneously open file handles
     * 
     * @details Limits resource usage to 4 concurrent file operations. Each open file
     *          consumes memory for the rfile structure and potentially an ExpandingString
     *          buffer for writes. Idle files (>30s inactive) are auto-closed to reclaim slots.
     */
    static constexpr uint8_t max_open_file = 4;

    /**
     * @brief Magic number identifying valid mission file format
     * 
     * @details Value 0x763d is checked at the start of every uploaded file to ensure
     *          data integrity and detect protocol mismatches. Upload will fail if magic
     *          number does not match.
     */
    static constexpr uint16_t mission_magic = 0x763d;

    /**
     * @enum Options
     * @brief Option flags for mission upload behavior
     * 
     * @details Bit flags encoded in the header options field to control upload behavior.
     *          Multiple flags can be OR'ed together.
     */
    enum class Options {
        /**
         * @brief Don't clear existing mission before uploading
         * 
         * @details When set, new mission items are merged with or replace existing items.
         *          When clear, existing mission is truncated before upload.
         *          
         * @note Currently only supported for MAV_MISSION_TYPE_MISSION
         * @note Fence and rally uploads require NO_CLEAR to be 0 (full replacement)
         */
        NO_CLEAR = (1U<<0),
    };

    /**
     * @struct header
     * @brief File header structure at start of binary mission file
     * 
     * @details 10-byte header preceding the packed mission item array. All fields are
     *          little-endian uint16_t values.
     */
    struct header {
        uint16_t magic = mission_magic;  ///< Magic number 0x763d for format validation
        uint16_t data_type;              ///< MAV_MISSION_TYPE_MISSION, _FENCE, or _RALLY
        uint16_t options;                ///< Bit flags from Options enum (e.g., NO_CLEAR)
        uint16_t start;                  ///< First waypoint number (0 for full upload)
        uint16_t num_items;              ///< Total number of mission items following header
    };

    /**
     * @struct rfile
     * @brief Per-file-descriptor state for open mission file
     * 
     * @details Tracks state for each of the max_open_file (4) file handles. For writes,
     *          allocates an ExpandingString buffer that grows as data is written. For reads,
     *          tracks current offset for streaming data from mission storage.
     */
    struct rfile {
        bool open;                       ///< True if this file descriptor slot is in use
        ExpandingString *writebuf;       ///< Write buffer (nullptr for read-only files)
        uint32_t file_ofs;               ///< Current file offset in bytes
        uint32_t num_items;              ///< Total items in mission (cached at open time)
        enum MAV_MISSION_TYPE mtype;     ///< Mission type (MISSION/FENCE/RALLY)
        uint32_t last_op_ms;             ///< Timestamp of last operation (for idle timeout)
    } file[max_open_file];

    /**
     * @brief Validate filename and extract mission type
     * 
     * @details Maps virtual filenames to MAV_MISSION_TYPE enum values. Only three
     *          filenames are valid, each conditionally compiled based on feature flags.
     * 
     * @param[in]  fname Filename to check
     * @param[out] mtype Mission type enum if filename is valid
     * 
     * @return true if filename is valid, false otherwise
     * 
     * @note Supported filenames: "mission.dat", "fence.dat", "rally.dat"
     * @note Availability depends on AP_MISSION_ENABLED, AP_FENCE_ENABLED, HAL_RALLY_ENABLED
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Mission.cpp:217-238
     */
    bool check_file_name(const char *fname, enum MAV_MISSION_TYPE &mtype);

    /**
     * @brief Retrieve a single mission item by index
     * 
     * @details Fetches mission item from the appropriate storage subsystem and converts
     *          to MAVLINK_MISSION_ITEM_INT format for serialization.
     * 
     * @param[in]  idx   Mission item index (0-based)
     * @param[in]  mtype Mission type (MISSION/FENCE/RALLY)
     * @param[out] item  Populated mission item structure
     * 
     * @return true if item retrieved successfully, false if index out of range
     * 
     * @note Calls AP_Mission::get_item(), MissionItemProtocol_Fence::get_item_as_mission_item(),
     *       or MissionItemProtocol_Rally::get_item_as_mission_item() depending on mtype
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Mission.cpp:243-265
     */
    bool get_item(uint32_t idx, enum MAV_MISSION_TYPE mtype, mavlink_mission_item_int_t &item) const;

    /**
     * @brief Get total number of items in mission storage
     * 
     * @details Queries the appropriate storage subsystem for current item count.
     *          Result is cached in rfile::num_items at open() time.
     * 
     * @param[in] mtype Mission type (MISSION/FENCE/RALLY)
     * 
     * @return Number of items in storage, 0 if subsystem unavailable
     * 
     * @note Calls AP_Mission::num_commands(), AC_Fence::polyfence().num_stored_items(),
     *       or AP_Rally::get_rally_total() depending on mtype
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Mission.cpp:268-305
     */
    uint32_t get_num_items(enum MAV_MISSION_TYPE mtype) const;

    /**
     * @brief Validate and commit buffered write data to mission storage
     * 
     * @details Called by close() to process write buffer. Validates magic number,
     *          item count, rejects all-zero items, then delegates to type-specific
     *          upload functions for final validation and commit.
     * 
     * @param[in] r File descriptor state with write buffer
     * 
     * @return true if validation and commit successful, false on any error
     * 
     * @note Validation failures leave original mission data intact (atomic commit)
     * @warning Any all-zero mission items will cause validation to fail
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Mission.cpp:363-410
     */
    bool finish_upload(const rfile &r);
    
    /**
     * @brief Commit mission waypoint data to AP_Mission
     * 
     * @details Validates and uploads mission items to AP_Mission storage under semaphore
     *          protection. Optionally clears existing mission first based on NO_CLEAR flag.
     *          Validates DO_JUMP targets and MAVLink command conversion.
     * 
     * @param[in] hdr File header with options and item count
     * @param[in] r   File descriptor state
     * @param[in] b   Raw buffer containing header + packed mission items
     * 
     * @return true if all items uploaded successfully, false on validation error
     * 
     * @note Protected by WITH_SEMAPHORE(mission->get_semaphore())
     * @warning DO_JUMP commands with targets >= num_items will fail validation
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Mission.cpp:413-448
     */
    bool finish_upload_mission(const struct header &hdr, const rfile &r, const uint8_t *b);
    
    /**
     * @brief Commit fence polygon data to AC_Fence
     * 
     * @details Converts MISSION_ITEM_INT to AC_PolyFenceItem and uploads to fence storage.
     *          Currently requires NO_CLEAR=0 (full fence replacement).
     * 
     * @param[in] hdr File header with options and item count
     * @param[in] r   File descriptor state
     * @param[in] b   Raw buffer containing header + packed mission items
     * 
     * @return true if fence uploaded successfully, false on error or out of memory
     * 
     * @note Allocates temporary AC_PolyFenceItem array, freed before return
     * @note NO_CLEAR option not supported - only full fence uploads allowed
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Mission.cpp:452-496
     */
    bool finish_upload_fence(const struct header &hdr, const rfile &r, const uint8_t *b);
    
    /**
     * @brief Commit rally point data to AP_Rally
     * 
     * @details Converts MISSION_ITEM_INT to RallyLocation and uploads to rally storage.
     *          Currently requires NO_CLEAR=0 (full rally point replacement).
     * 
     * @param[in] hdr File header with options and item count
     * @param[in] r   File descriptor state
     * @param[in] b   Raw buffer containing header + packed mission items
     * 
     * @return true if rally points uploaded successfully, false on error
     * 
     * @note Truncates existing rally points before upload (rally->truncate(0))
     * @note NO_CLEAR option not supported - only full rally uploads allowed
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Mission.cpp:500-534
     */
    bool finish_upload_rally(const struct header &hdr, const rfile &r, const uint8_t *b);

    /**
     * @brief Check if a memory block contains only zeros
     * 
     * @details Used during upload validation to detect incomplete mission items.
     *          Client is required to fill entire file; any all-zero items indicate
     *          incomplete upload and will cause validation to fail.
     * 
     * @param[in] b    Pointer to memory block
     * @param[in] size Size of block in bytes
     * 
     * @return true if all bytes are zero, false otherwise
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Mission.cpp:350-358
     */
    bool all_zero(const uint8_t *b, uint8_t size) const;
};

#endif  // AP_FILESYSTEM_MISSION_ENABLED
