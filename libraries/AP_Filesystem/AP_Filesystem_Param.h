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
 * @file AP_Filesystem_Param.h
 * @brief @PARAM virtual filesystem backend for compressed parameter file access
 * 
 * @details This file implements the @PARAM virtual filesystem interface that provides
 *          efficient parameter download via MAVLink FTP. The primary interface is the
 *          @PARAM/param.pck virtual file which contains a compressed binary representation
 *          of the flight controller's parameter list.
 *          
 *          The param.pck format uses prefix compression to minimize bandwidth usage,
 *          making it significantly faster than traditional MAVLink parameter messages
 *          for downloading full parameter lists. The format is optimized for MAVLink
 *          FTP with SiK radios (recommended burst size: 110 bytes).
 *          
 *          Key Features:
 *          - Prefix compression of parameter names (avoids sending common prefixes)
 *          - Consistent block sizes to prevent value corruption during retransmission
 *          - Query string support for partial parameter lists (?start=N&count=M)
 *          - Optional default values (?withdefaults=1)
 *          - Read-only virtual file (no write support to param.pck)
 *          - Zero padding to ensure values don't split across MAVLink blocks
 *          
 *          File Format (little-endian):
 *          - Header (6 bytes): magic(0x671b), num_params, total_params
 *          - Variable-length parameter blocks with prefix compression
 *          - Optional default values if requested and different from set value
 *          
 * @note Complete format specification documented in libraries/AP_Filesystem/README.md
 * 
 * @see AP_Filesystem_Backend
 * @see AP_Param
 * @see libraries/AP_Filesystem/README.md for param.pck format details
 */

#pragma once

#include "AP_Filesystem_config.h"

#if AP_FILESYSTEM_PARAM_ENABLED

#include "AP_Filesystem_backend.h"
#include <AP_Common/ExpandingString.h>

#include <AP_Param/AP_Param.h>

/**
 * @class AP_Filesystem_Param
 * @brief Virtual filesystem backend providing compressed parameter file access via @PARAM interface
 * 
 * @details AP_Filesystem_Param implements a read-only virtual filesystem that exposes the
 *          flight controller's parameter list as @PARAM/param.pck. This provides efficient
 *          parameter download via MAVLink FTP using a compressed binary format.
 *          
 *          Compression Algorithm:
 *          Parameter names are prefix-compressed by storing only the non-common suffix
 *          relative to the previous parameter name. For example, if previous parameter
 *          was "SERVO1_MIN" and current is "SERVO1_MAX", only "_MAX" needs to be sent
 *          (common_len=7, name_len=3).
 *          
 *          Block Consistency:
 *          All reads on a single file handle must use the same block size. The implementation
 *          adds zero-padding before parameter blocks to ensure parameter values never split
 *          across MAVLink FTP block boundaries. This prevents corruption when retransmitting
 *          lost blocks.
 *          
 *          Query String Support:
 *          - ?start=N&count=M: Download M parameters starting from index N
 *          - ?withdefaults=1: Include default values (only if different from set value)
 *          
 *          MAVLink FTP Optimization:
 *          Designed for efficient streaming over SiK radios. Recommended burst-read size
 *          is 110 bytes for optimal performance with SiK telemetry radios.
 *          
 *          Lifecycle:
 *          1. open() - Parse query strings, initialize parameter iteration cursor
 *          2. read() - Stream compressed parameters with consistent block sizes
 *          3. close() - Release cursor and buffer resources
 *          
 *          Upload Support (write mode):
 *          Supports parameter upload via write operations. Written data is buffered in
 *          an ExpandingString until close(), then parsed and applied atomically.
 *          
 * @note Thread Safety: File descriptor indices are per-instance, not globally synchronized
 * @note Performance: Cursor array minimizes seeking when filling gaps from retransmissions
 * @note Maximum 4 simultaneous open file handles (max_open_file)
 * 
 * @warning Read-only for param.pck downloads. Write operations are for parameter upload.
 * @warning All reads must use consistent block size after first read to prevent corruption
 * 
 * Source: libraries/AP_Filesystem/AP_Filesystem_Param.cpp
 * Source: libraries/AP_Filesystem/README.md (format specification)
 */
class AP_Filesystem_Param : public AP_Filesystem_Backend
{
public:
    // functions that closely match the equivalent posix calls
    
    /**
     * @brief Open @PARAM/param.pck virtual file with optional query string parameters
     * 
     * @details Opens the compressed parameter file for reading or writing. For read operations,
     *          initializes parameter iteration cursors and parses query string arguments.
     *          For write operations, allocates an ExpandingString buffer for parameter upload.
     *          
     *          Supported Query Strings:
     *          - ?start=N: Begin parameter list at index N (0-based)
     *          - ?count=M: Include M parameters (default: all from start)
     *          - ?withdefaults=1: Include default values where different from set value
     *          
     *          Query strings can be combined: @PARAM/param.pck?start=50&count=10&withdefaults=1
     *          
     *          Read Mode (O_RDONLY):
     *          - Allocates cursor array for efficient parameter iteration
     *          - Parses query string to set start/count/withdefaults flags
     *          - Initializes file_ofs to 0 for streaming reads
     *          
     *          Write Mode (O_WRONLY, O_RDWR):
     *          - Allocates ExpandingString buffer for incoming parameter data
     *          - Data written via write() calls until close()
     *          - Parameters parsed and applied atomically on close()
     * 
     * @param[in] fname     File name, must be "@PARAM/param.pck" optionally with query string
     * @param[in] flags     Open flags (O_RDONLY for download, O_WRONLY/O_RDWR for upload)
     * @param[in] allow_absolute_paths  Ignored for virtual filesystem (always false)
     * 
     * @return File descriptor index (0 to max_open_file-1) on success, -1 on failure
     * 
     * @note Only 4 simultaneous open files supported (max_open_file=4)
     * @note Query string parsing: invalid values (>=UINT16_MAX) result in EINVAL
     * 
     * @warning File name must exactly match "@PARAM/param.pck" (case-sensitive)
     * 
     * Error Codes (errno):
     * - ENOENT: Invalid file name (not @PARAM/param.pck)
     * - ENFILE: Too many open files (all 4 slots in use)
     * - ENOMEM: Failed to allocate cursors or write buffer
     * - EINVAL: Invalid query string parameter value
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Param.cpp:38-130
     */
    int open(const char *fname, int flags, bool allow_absolute_paths = false) override;
    
    /**
     * @brief Close file descriptor and release associated resources
     * 
     * @details Releases cursor array (for read mode) or processes parameter upload buffer
     *          (for write mode). For write operations, triggers parsing of buffered parameter
     *          data and applies changes to AP_Param storage.
     *          
     *          Write Mode Behavior:
     *          - Calls finish_upload() to parse and apply buffered parameters
     *          - May trigger multiple parse attempts if retry needed
     *          - Validates parameter format and types before applying
     *          
     *          Read Mode Behavior:
     *          - Frees cursor array allocated in open()
     *          - Resets file structure to closed state
     * 
     * @param[in] fd  File descriptor from successful open() call (0 to max_open_file-1)
     * 
     * @return 0 on success, -1 on failure
     * 
     * @note Always succeeds for read mode
     * @note For write mode, failure indicates parameter parse/apply error
     * 
     * Error Codes (errno):
     * - EBADF: Invalid file descriptor (out of range or not open)
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Param.cpp:132-147
     */
    int close(int fd) override;
    
    /**
     * @brief Read compressed parameter data with consistent block sizing
     * 
     * @details Streams compressed parameter blocks from the virtual param.pck file.
     *          Implements critical block consistency requirement: all reads on a single
     *          file descriptor must use the same block size. This prevents parameter
     *          values from being split across MAVLink FTP block boundaries during
     *          retransmission of lost packets.
     *          
     *          Block Consistency Algorithm:
     *          1. First read establishes block size for this file descriptor
     *          2. Subsequent reads must match initial block size (returns -1 if different)
     *          3. Zero padding added before parameters to prevent value splits
     *          
     *          Parameter Streaming:
     *          - Maintains multiple cursors for efficient gap-filling
     *          - Advances through AP_Param tokens to generate compressed blocks
     *          - Respects start/count query string limits
     *          
     *          Compressed Block Format (per parameter):
     *          - Optional zero padding (ensures value doesn't cross block boundary)
     *          - type:4, flags:4 (AP_Param type and flags byte)
     *          - common_len:4, name_len:4 (prefix compression fields)
     *          - name[name_len+1] (non-common parameter name suffix)
     *          - data[type_size] (parameter value, size depends on type)
     *          - default[type_size] (optional, only if withdefaults=1 and differs)
     *          
     *          Optimizations:
     *          - Uses two cursors to minimize seeking when filling gaps
     *          - Caches trailer bytes for efficient retransmission
     *          - Prefix compression reduces bandwidth by ~30-50%
     * 
     * @param[in]  fd     File descriptor from open()
     * @param[out] buf    Buffer to receive compressed parameter data
     * @param[in]  count  Number of bytes to read (must match first read size for this fd)
     * 
     * @return Number of bytes read (0 at EOF), -1 on error
     * 
     * @note First read establishes block size - all subsequent reads must match
     * @note Recommended block size: 110 bytes for SiK radio optimization
     * @note Returns 0 when all requested parameters have been streamed
     * 
     * @warning Varying block size after first read causes EINVAL and returns -1
     * @warning Block size must be large enough for max_pack_len (AP_MAX_NAME_SIZE+13)
     * 
     * Error Codes (errno):
     * - EBADF: Invalid file descriptor
     * - EINVAL: Block size mismatch with previous read, or write-mode file
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Param.cpp:149-299
     */
    int32_t read(int fd, void *buf, uint32_t count) override;
    
    /**
     * @brief Seek to specified offset in virtual parameter file
     * 
     * @details Changes the file position for subsequent read() operations. Seeking is
     *          necessary for MAVLink FTP protocol when retransmitting lost blocks.
     *          The implementation efficiently repositions parameter iteration cursors
     *          using token-based seeking rather than restarting from the beginning.
     *          
     *          Supported Seek Modes:
     *          - SEEK_SET: Absolute offset from file start
     *          - SEEK_CUR: Relative offset from current position
     *          - SEEK_END: Offset from file end (typically negative)
     *          
     *          Performance:
     *          Uses token_seek() to efficiently position cursors without full iteration.
     *          Maintains multiple cursors to minimize seeking when alternating between
     *          different file regions during gap-filling.
     * 
     * @param[in] fd      File descriptor from open()
     * @param[in] offset  Byte offset for seek operation
     * @param[in] whence  Seek mode (SEEK_SET, SEEK_CUR, or SEEK_END)
     * 
     * @return New file position on success, -1 on error
     * 
     * @note Seeking beyond file end positions at EOF (subsequent reads return 0)
     * @note Seeking backwards requires cursor repositioning via token_seek()
     * 
     * Error Codes (errno):
     * - EBADF: Invalid file descriptor
     * - EINVAL: Invalid whence value or write-mode file
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Param.cpp:301-328
     */
    int32_t lseek(int fd, int32_t offset, int whence) override;
    
    /**
     * @brief Get file statistics for @PARAM/param.pck virtual file
     * 
     * @details Returns estimated file size based on parameter count and average parameter
     *          size. The actual file size varies due to prefix compression and optional
     *          default values, so this provides an upper-bound estimate for buffer allocation.
     *          
     *          Size Calculation:
     *          file_size = header(6 bytes) + num_params * avg_param_size
     *          
     *          The average parameter size accounts for:
     *          - Compression fields (2 bytes)
     *          - Average name length after prefix compression
     *          - Parameter value (1-4 bytes depending on type)
     *          - Optional default value (if withdefaults requested)
     *          - Padding overhead for block alignment
     * 
     * @param[in]  pathname  File path, must be "@PARAM/param.pck" with optional query string
     * @param[out] stbuf     stat structure to populate with file information
     * 
     * @return 0 on success, -1 on error
     * 
     * @note Returned size is estimate - actual size depends on compression effectiveness
     * @note Sets st_size field only; other stat fields not applicable to virtual file
     * 
     * Error Codes (errno):
     * - ENOENT: Invalid file name (not @PARAM/param.pck)
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Param.cpp:568-587
     */
    int stat(const char *pathname, struct stat *stbuf) override;
    
    /**
     * @brief Write parameter data for upload (buffer until close)
     * 
     * @details Buffers incoming parameter data in ExpandingString for later parsing.
     *          Data is accumulated across multiple write() calls and processed atomically
     *          when close() is called. This allows parameter upload via MAVLink FTP.
     *          
     *          Upload Format:
     *          Expects same param.pck compressed format as read operations:
     *          - 6-byte header with magic, num_params, total_params
     *          - Variable-length compressed parameter blocks
     *          - Parameters parsed and applied via AP_Param::set() on close()
     *          
     *          Buffering:
     *          - Uses ExpandingString to grow buffer as needed
     *          - No size limit (constrained by available RAM)
     *          - All parsing deferred until close() for atomicity
     * 
     * @param[in] fd     File descriptor from open() in write mode
     * @param[in] buf    Parameter data to buffer
     * @param[in] count  Number of bytes to write
     * 
     * @return Number of bytes buffered on success, -1 on error
     * 
     * @note Data not applied until close() - upload is atomic
     * @note Write calls can be any size; accumulated in ExpandingString
     * 
     * @warning Ensure sufficient RAM for parameter upload buffer
     * 
     * Error Codes (errno):
     * - EBADF: Invalid file descriptor
     * - EINVAL: File opened in read-only mode
     * - ENOMEM: Failed to expand buffer (out of memory)
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Param.cpp:330-347
     */
    int32_t write(int fd, const void *buf, uint32_t count) override;

private:
    /**
     * @brief Number of parameter iteration cursors maintained per open file
     * 
     * @details Multiple cursors minimize seeking overhead when filling gaps during
     *          MAVLink FTP retransmission of lost blocks. When a read request targets
     *          a different file offset, the implementation selects the nearest cursor
     *          rather than always seeking from the beginning.
     *          
     *          Two cursors provide good balance between memory usage and seek efficiency
     *          for typical FTP gap-filling patterns.
     */
    static constexpr uint8_t num_cursors = 2;

    /**
     * @brief Maximum number of simultaneously open @PARAM files
     * 
     * @details Limits concurrent file handles to conserve memory. Each open file
     *          requires cursor arrays and potentially write buffers. Four simultaneous
     *          handles support multiple concurrent GCS connections or overlapping
     *          parameter operations.
     */
    static constexpr uint8_t max_open_file = 4;

    /**
     * @brief Maximum size of one packed parameter including name, value, and default
     * 
     * @details Calculated as:
     *          - AP_MAX_NAME_SIZE: Parameter name (typically 16 bytes)
     *          - 2 bytes: type/flags and common_len/name_len compression fields
     *          - 4 bytes: Parameter value (worst case: FLOAT or INT32)
     *          - 4 bytes: Optional default value (if withdefaults=1)
     *          - 3 bytes: Margin for padding and field overhead
     *          
     *          Used to validate block sizes and allocate cursor trailer buffers.
     * 
     * @note Block size for read() must be >= max_pack_len to avoid truncation
     */
    static constexpr uint8_t max_pack_len = AP_MAX_NAME_SIZE + 2 + 4 + 4 + 3;

    /**
     * @brief Magic number for standard param.pck format (0x671b)
     * 
     * @details Identifies param.pck file format version in the 6-byte header.
     *          This magic value indicates standard format without default values.
     *          Little-endian encoding.
     * 
     * @note Magic value distinguishes param.pck format from other binary formats
     */
    static constexpr uint16_t pmagic = 0x671b;
    
    /**
     * @brief Magic number for param.pck format with default values (0x671c)
     * 
     * @details Extended format magic number used when withdefaults=1 query string
     *          is specified. Indicates that default values may be included in
     *          parameter blocks where they differ from set values.
     *          
     *          Default values help GCS identify which parameters have been customized
     *          versus using factory defaults.
     * 
     * @note Currently 0x671b used for both modes; 0x671c reserved for future use
     */
    static constexpr uint16_t pmagic_with_default = 0x671c;

    /**
     * @struct header
     * @brief 6-byte little-endian header at start of param.pck file
     * 
     * @details The header provides version identification and parameter count information.
     *          All fields are little-endian uint16_t for efficient parsing on embedded systems.
     *          
     *          Format:
     *          - Bytes 0-1: magic (0x671b for standard, 0x671c for withdefaults)
     *          - Bytes 2-3: num_params (count of parameters in this file)
     *          - Bytes 4-5: total_params (total parameters available on flight controller)
     *          
     *          For Partial Downloads:
     *          If ?start=N&count=M specified, num_params=M but total_params remains the
     *          full parameter count. This allows GCS to know total parameter space.
     *          
     *          For Upload:
     *          The total_params field may contain the total upload file length instead
     *          of parameter count to facilitate progress tracking.
     * 
     * @note Header is always exactly 6 bytes (3 × uint16_t)
     * @note Little-endian encoding matches MAVLink protocol endianness
     */
    struct header {
        uint16_t magic = pmagic;            ///< Format version identifier (0x671b or 0x671c)
        uint16_t num_params;                ///< Number of parameters in this file/stream
        uint16_t total_params;              ///< Total parameters available (or file length for upload)
    };

    /**
     * @struct cursor
     * @brief Parameter iteration cursor for efficient seeking and streaming
     * 
     * @details Each cursor maintains state for iterating through the AP_Param parameter
     *          tree. Multiple cursors per file allow efficient gap-filling when MAVLink FTP
     *          retransmits lost blocks without restarting iteration from the beginning.
     *          
     *          Cursor State Components:
     *          - token: AP_Param iteration token (tracks position in parameter tree)
     *          - token_ofs: Byte offset in file corresponding to current token position
     *          - last_name: Previous parameter name (for prefix compression)
     *          - trailer: Cached packed parameter bytes (for efficient retransmission)
     *          - idx: Current parameter index (0-based)
     *          
     *          Prefix Compression Context:
     *          The last_name field is critical for prefix compression. Each parameter block
     *          encodes only the non-common suffix relative to the previous parameter. For
     *          example, if last_name="SERVO1_MIN" and current="SERVO1_MAX", only "_MAX"
     *          is encoded (common_len=7).
     *          
     *          Trailer Caching:
     *          When a parameter is packed, the compressed bytes are cached in the trailer
     *          buffer. If the same data is requested again (retransmission), it can be
     *          returned immediately without recomputing compression.
     * 
     * @note Multiple cursors minimize seeking overhead during gap-filling
     * @note Trailer caching assumes parameters don't change during file read
     */
    struct cursor {
        AP_Param::ParamToken token;            ///< Current position in parameter tree iteration
        uint32_t token_ofs;                    ///< File byte offset corresponding to this token
        char last_name[AP_MAX_NAME_SIZE+1];    ///< Previous parameter name (for prefix compression context)
        uint8_t trailer_len;                   ///< Length of cached packed parameter in trailer[]
        uint8_t trailer[max_pack_len];         ///< Cached packed parameter bytes (for retransmission)
        uint16_t idx;                          ///< Current parameter index (0-based, respects start offset)
    };

    /**
     * @struct rfile
     * @brief Open file descriptor state for @PARAM/param.pck access
     * 
     * @details Maintains all state associated with an open file handle including query
     *          string parameters, read position, and iteration cursors. Separate instances
     *          support concurrent access from multiple GCS connections.
     *          
     *          Read Mode State (O_RDONLY):
     *          - cursors: Array of cursor structs for parameter iteration
     *          - read_size: Established block size (must be consistent across all reads)
     *          - start/count: Query string range parameters (?start=N&count=M)
     *          - with_defaults: Include default values flag (?withdefaults=1)
     *          - file_ofs: Current read position in virtual file
     *          
     *          Write Mode State (O_WRONLY, O_RDWR):
     *          - writebuf: ExpandingString accumulating parameter upload data
     *          - Data buffered until close(), then parsed and applied atomically
     *          
     *          Block Size Consistency:
     *          The read_size field enforces critical consistency requirement. After first
     *          read establishes block size, all subsequent reads must match. This prevents
     *          parameter values from splitting across MAVLink FTP blocks during retransmission.
     *          
     *          Query String Semantics:
     *          - start=0, count=0: All parameters (default)
     *          - start=N, count=0: All parameters from index N onward
     *          - start=N, count=M: Exactly M parameters starting at index N
     *          - with_defaults=1: Include default values where different from set value
     * 
     * @note read_size=0 until first read() establishes block size
     * @note cursors is nullptr for write-mode files
     * @note writebuf is nullptr for read-mode files
     */
    struct rfile {
        bool open;                    ///< True if this file descriptor slot is in use
        bool with_defaults;           ///< True if ?withdefaults=1 query string specified
        uint16_t read_size;           ///< Established block size (must be consistent), 0 before first read
        uint16_t start;               ///< Starting parameter index from ?start=N query string
        uint16_t count;               ///< Parameter count limit from ?count=M query string (0=unlimited)
        uint32_t file_ofs;            ///< Current file position for read/seek operations
        uint32_t file_size;           ///< Estimated total file size (for stat and EOF detection)
        struct cursor *cursors;       ///< Array of num_cursors iteration cursors (read mode only)
        ExpandingString *writebuf;    ///< Parameter upload buffer (write mode only)
    } file[max_open_file];            ///< Array of file descriptor state (max 4 concurrent files)

    /**
     * @brief Seek parameter iteration cursor to specified file offset
     * 
     * @details Efficiently positions a cursor to the parameter corresponding to the given
     *          file byte offset. Uses AP_Param token iteration to advance through parameters
     *          without repacking each one until the target offset is reached.
     *          
     *          Algorithm:
     *          1. If target offset < current cursor position, restart from beginning
     *          2. Iterate through parameters, accumulating packed size estimates
     *          3. Stop when accumulated size reaches target offset
     *          4. Update cursor token, token_ofs, idx, and last_name
     *          
     *          Performance:
     *          Maintains token_ofs to enable efficient forward seeking without repacking.
     *          Multiple cursors reduce seeking overhead when alternating between file regions.
     * 
     * @param[in]     r        Open file descriptor state
     * @param[in]     data_ofs Target byte offset in file (relative to end of header)
     * @param[in,out] c        Cursor to reposition
     * 
     * @return true if seek successful, false if offset beyond file end
     * 
     * @note Backwards seeks require restarting from beginning (no reverse iteration)
     * @note Offset is relative to data (excludes 6-byte header)
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Param.cpp:349-428
     */
    bool token_seek(const struct rfile &r, const uint32_t data_ofs, struct cursor &c);
    
    /**
     * @brief Pack parameter at current cursor position into compressed format
     * 
     * @details Generates compressed parameter block with prefix compression, zero padding
     *          to prevent value splits across MAVLink blocks, and optional default value.
     *          
     *          Compression Process:
     *          1. Compare current parameter name with cursor.last_name
     *          2. Calculate common_len (shared prefix bytes)
     *          3. Encode: type|flags, common_len|name_len, name_suffix, value, [default]
     *          4. Add zero padding if necessary to prevent value split
     *          5. Cache result in cursor.trailer for retransmission
     *          
     *          Padding Logic:
     *          If parameter value would cross a read_size block boundary, prepend zero
     *          padding bytes to push the entire parameter into the next block. This
     *          ensures retransmission of a lost block cannot cause value corruption.
     *          
     *          Type Encoding (4 bits):
     *          - 0: NONE (invalid)
     *          - 1: INT8 (1 byte)
     *          - 2: INT16 (2 bytes)
     *          - 3: INT32 (4 bytes)
     *          - 4: FLOAT (4 bytes)
     *          
     *          Flags Encoding (4 bits):
     *          - Bit 0: Default value included (if withdefaults=1 and differs from set value)
     *          - Bits 1-3: Reserved for future use
     * 
     * @param[in]     r    Open file descriptor state (for read_size and with_defaults)
     * @param[in,out] c    Cursor at parameter to pack (updated with packed bytes)
     * @param[out]    buf  Buffer to receive packed parameter bytes
     * 
     * @return Number of bytes written to buf (including padding)
     * 
     * @note Updates cursor.last_name for next parameter's prefix compression
     * @note Caches result in cursor.trailer and cursor.trailer_len
     * @note Returns 0 if parameter should be skipped (out of range)
     * 
     * @warning Buffer must be at least max_pack_len bytes
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Param.cpp:430-540
     */
    uint8_t pack_param(const struct rfile &r, struct cursor &c, uint8_t *buf);
    
    /**
     * @brief Validate that file name matches @PARAM/param.pck pattern
     * 
     * @details Checks that the provided file name is exactly "@PARAM/param.pck"
     *          (case-sensitive) optionally followed by query string parameters.
     *          Query strings begin with '?' and are not validated here (parsed in open()).
     *          
     *          Valid Examples:
     *          - @PARAM/param.pck
     *          - @PARAM/param.pck?start=10
     *          - @PARAM/param.pck?start=10&count=5&withdefaults=1
     *          
     *          Invalid Examples:
     *          - @param/param.pck (wrong case)
     *          - @PARAM/params.pck (wrong filename)
     *          - PARAM/param.pck (missing @)
     * 
     * @param[in] fname  File name to validate
     * 
     * @return true if valid @PARAM/param.pck path, false otherwise
     * 
     * @note Case-sensitive comparison
     * @note Query strings not validated by this function
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Param.cpp:589-598
     */
    bool check_file_name(const char *fname);

    /**
     * @brief Complete parameter upload by parsing and applying buffered data
     * 
     * @details Called by close() for write-mode files. Parses accumulated parameter
     *          data from writebuf and applies changes to AP_Param storage. Upload is
     *          atomic - all parameters applied together or none on parse failure.
     *          
     *          Upload Process:
     *          1. Validate header (magic, num_params, total_params)
     *          2. Parse each parameter block (decompression, type validation)
     *          3. Apply parameters via AP_Param::set()
     *          4. May retry parse if transient errors occur
     *          
     *          Error Handling:
     *          Returns false on parse errors (invalid format, unknown parameters, etc.).
     *          Some errors trigger retry via param_upload_parse need_retry flag.
     * 
     * @param[in] r  Open file descriptor with buffered upload data in writebuf
     * 
     * @return true if upload successful and parameters applied, false on error
     * 
     * @note Upload is atomic - parameters not applied on parse failure
     * @note May modify parameter values on flight controller if successful
     * 
     * @warning Parameter changes become effective immediately on successful upload
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Param.cpp:542-555
     */
    bool finish_upload(const rfile &r);
    
    /**
     * @brief Parse parameter upload data and apply to AP_Param storage
     * 
     * @details Implements the actual parsing logic for parameter upload. Decompresses
     *          parameter names using prefix compression, validates types and values,
     *          and applies changes via AP_Param::set().
     *          
     *          Decompression Algorithm:
     *          1. Read common_len from current parameter block
     *          2. Copy common_len bytes from previous parameter name
     *          3. Append name_len+1 new bytes from parameter block
     *          4. Look up parameter in AP_Param by reconstructed name
     *          5. Validate type matches and value is in range
     *          6. Apply via AP_Param::set()
     *          
     *          Retry Logic:
     *          Sets need_retry flag for transient errors that may succeed on retry
     *          (e.g., temporary resource contention). Permanent errors (invalid format,
     *          unknown parameters) do not set retry flag.
     * 
     * @param[in]  r           Open file descriptor with upload data
     * @param[out] need_retry  Set to true if parse should be retried, false for permanent errors
     * 
     * @return true if parse and apply successful, false on error
     * 
     * @note Modifies parameter values on flight controller if successful
     * @note May be called multiple times if need_retry=true
     * 
     * @warning Parameter changes take effect immediately - ensure upload data is valid
     * 
     * Source: libraries/AP_Filesystem/AP_Filesystem_Param.cpp:557-566
     */
    bool param_upload_parse(const rfile &r, bool &need_retry);
};

#endif  // AP_FILESYSTEM_PARAM_ENABLED
