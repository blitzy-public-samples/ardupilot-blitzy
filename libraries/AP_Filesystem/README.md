# AP_Filesystem: The ArduPilot virtual filesystem layer

AP_Filesystem is a filesystem abstraction for ArduPilot that provides
ArduPilot access to both local filesystems on the flight controller
and a set of virtual filesystem abstractions. It offers a unified
POSIX-like API across different storage backends (SD cards, internal
flash, read-only embedded resources, virtual parameter/diagnostic
interfaces) while handling platform-specific implementations transparently.

This document covers the AP_Filesystem architecture, backend implementations,
API usage patterns, and virtual filesystem interfaces available to both
ArduPilot code and ground station authors via the MAVLink FTP transport.

## Architecture Overview

AP_Filesystem implements a router/multiplexer pattern that directs file operations to
the appropriate backend based on path prefixes. The system provides a single unified
interface accessible via the `AP::FS()` singleton.

### Core Architecture Components

```mermaid
graph LR
    A[Application Code] -->|AP::FS| B[AP_Filesystem Router]
    B -->|path routing| C{Backend Selection}
    C -->|default path| D[Local FS<br/>FATFS/LittleFS/POSIX/ESP32]
    C -->|@ROMFS/| E[ROMFS Backend]
    C -->|@PARAM/| F[Param Backend]
    C -->|@SYS/| G[Sys Backend]
    C -->|@MISSION/| H[Mission Backend]
    D --> I[SD Card<br/>Internal Flash<br/>Native OS]
    E --> J[Embedded Resources]
    F --> K[Parameter System]
    G --> L[System Diagnostics]
    H --> M[Mission Storage]
```

### File Descriptor Encoding

AP_Filesystem uses an encoding scheme to multiplex file descriptors across multiple backends:

- **File descriptor range**: `0` to `NUM_BACKENDS * MAX_FD_PER_BACKEND - 1`
- **MAX_FD_PER_BACKEND**: `256` file descriptors per backend
- **Encoding**: `encoded_fd = backend_index * 256 + backend_fd`
- **Decoding**: `backend_index = fd / 256`, `backend_fd = fd % 256`

This allows the router to identify which backend owns each file descriptor and
route operations correctly without maintaining additional state.

### Backend Registration

Backends are registered in a static array in `AP_Filesystem.cpp`:

```cpp
const AP_Filesystem::Backend AP_Filesystem::backends[] = {
    { nullptr, fs_local },        // Default: FATFS/LittleFS/POSIX/ESP32
    { "@ROMFS", fs_romfs },       // Read-only embedded resources
    { "@PARAM", fs_param },       // Virtual parameter file interface
    { "@SYS", fs_sys },           // Virtual system diagnostics
    { "@MISSION", fs_mission },   // Virtual mission/fence/rally files
};
```

**Path Routing Logic**:
1. Strip leading slashes from path
2. Check for `@` prefix indicating virtual filesystem
3. Match against registered backend prefixes
4. Default to local filesystem backend if no match
5. Strip matched prefix from path before passing to backend

### Singleton Access Pattern

All filesystem operations use the `AP::FS()` singleton accessor:

```cpp
#include <AP_Filesystem/AP_Filesystem.h>

// Open a file
int fd = AP::FS().open("/APM/logs/latest.bin", O_RDONLY);
if (fd >= 0) {
    uint8_t buffer[512];
    int32_t bytes_read = AP::FS().read(fd, buffer, sizeof(buffer));
    AP::FS().close(fd);
}
```

## Backend Implementations

AP_Filesystem supports multiple storage backends, each optimized for different use cases.
Only one "local" filesystem backend (FATFS, LittleFS, POSIX, or ESP32) is compiled in per platform,
while virtual backends (@ROMFS, @PARAM, @SYS, @MISSION) can coexist.

### FATFS Backend (AP_Filesystem_FATFS)

**Purpose**: SD card filesystem support for ARM platforms using ChibiOS

**Source**: `libraries/AP_Filesystem/AP_Filesystem_FATFS.cpp`

**Key Features**:
- FAT32 filesystem on SD cards via SDIO or SPI interface
- Automatic mount on system startup with retry logic
- DMA-aware I/O chunking (breaks large transfers into smaller chunks for DMA compatibility)
- Disk space queries (`disk_free()`, `disk_space()`)
- Async formatting support with progress monitoring
- Error recovery through remount attempts
- Write-through caching with `fsync()` support

**Configuration**:
- Enabled via: `AP_FILESYSTEM_FATFS_ENABLED` (set by `HAL_OS_FATFS_IO`)
- Default mount point: `/` (root)
- SD card detection: Automatic via HAL layer

**Implementation Details**:
- Uses ChibiOS FATFS library (FatFs by ChaN)
- File descriptors: Wraps FATFS `FIL` structures
- Thread safety: Semaphore-protected operations
- Mount retry: Attempts remount on first operation after failure
- Format operation: Async format with `get_format_status()` monitoring

**Common Use Cases**:
- DataFlash logging to SD card
- Mission/rally/fence storage
- Parameter file backups
- Terrain data caching

### LittleFS Backend (AP_Filesystem_FlashMemory_LittleFS)

**Purpose**: Internal SPI flash storage with wear-leveling and power-safe writes

**Source**: `libraries/AP_Filesystem/AP_Filesystem_FlashMemory_LittleFS.cpp`

**Key Features**:
- Wear-leveling filesystem for SPI NOR/NAND flash
- Power-safe writes (crash-safe, no corruption on power loss)
- JEDEC flash chip auto-detection
- Automatic mount with configurable retry attempts
- Background compaction for space recovery
- `bytes_until_fsync()` optimization for streaming writes

**Flash Types Supported**:
- **JEDEC NOR flash** (`AP_FILESYSTEM_FLASH_JEDEC_NOR`): Standard SPI NOR flash (default)
- **W25N NAND flash** (`AP_FILESYSTEM_FLASH_W25NXX`): Winbond W25N series NAND

**Configuration**:
- Enabled via: `AP_FILESYSTEM_LITTLEFS_ENABLED` (set by `HAL_OS_LITTLEFS_IO`)
- Flash type: `AP_FILESYSTEM_LITTLEFS_FLASH_TYPE`
- Default mount point: `/` (root)

**Implementation Details**:
- Uses LittleFS library (ARM Mbed project)
- Block device: Custom SPI flash driver
- Mount behavior: Retry on failure, auto-format if unformatted
- Thread safety: Semaphore-protected
- Write optimization: `bytes_until_fsync()` returns optimal sync interval

**Common Use Cases**:
- Parameter storage on boards without SD card
- Logging on flash-only boards
- Configuration file storage
- Small data persistence

**Trade-offs**:
- **Pros**: Wear-leveling, power-safe, no SD card needed
- **Cons**: Limited capacity (typically 2-16MB), slower than SD card, finite write endurance

### ROMFS Backend (AP_Filesystem_ROMFS)

**Purpose**: Read-only access to firmware-embedded resources

**Source**: `libraries/AP_Filesystem/AP_Filesystem_ROMFS.cpp`

**Key Features**:
- Zero runtime storage cost (compiled into firmware)
- Decompression support for compressed resources
- Fast access (memory-mapped)
- Immutable content (cannot be modified at runtime)

**Embedded Resources**:
- OSD fonts (character sets for on-screen display)
- Lookup tables for algorithms
- Default configuration files
- Calibration data

**Configuration**:
- Enabled via: `AP_FILESYSTEM_ROMFS_ENABLED` (automatic if `HAL_HAVE_AP_ROMFS_EMBEDDED_H` defined)
- Access path: `@ROMFS/` prefix

**Implementation Details**:
- Resources embedded via `ap_romfs_embedded.h` header
- Files stored as byte arrays in flash memory
- Optional decompression for space-constrained platforms
- Read-only operations only (open with O_RDONLY, write/unlink unsupported)

**Common Use Cases**:
- Accessing embedded OSD fonts
- Reading default parameter files
- Loading firmware-bundled data

### POSIX Backend (AP_Filesystem_Posix)

**Purpose**: Native Linux/SITL filesystem access for development and testing

**Source**: `libraries/AP_Filesystem/AP_Filesystem_posix.cpp`

**Key Features**:
- Direct passthrough to native OS filesystem
- Base directory mapping for sandboxing
- `statfs` support for disk space queries
- Full POSIX compatibility

**Configuration**:
- Enabled via: `AP_FILESYSTEM_POSIX_ENABLED` (automatic on Linux/SITL/QURT)
- Base directory: Configurable via `AP_FILESYSTEM_POSIX_MAP_FILENAME_BASEDIR`
- Path mapping: Optional filename rewriting

**Implementation Details**:
- Thin wrapper around POSIX calls (open/read/write/lseek/close)
- File descriptors: Direct POSIX file descriptors
- Thread safety: Relies on OS thread safety
- Paths: Can be mapped to base directory for sandboxing

**Common Use Cases**:
- SITL simulation (Software-In-The-Loop)
- Linux autopilot boards (BBBMini, Navio, etc.)
- Development and testing on desktop Linux
- Log replay tools

### ESP32 Backend (AP_Filesystem_ESP32)

**Purpose**: ESP-IDF filesystem integration for ESP32 platforms

**Source**: `libraries/AP_Filesystem/AP_Filesystem_ESP32.cpp`

**Key Features**:
- ESP-IDF VFS integration
- FatFS wrapper for SD card support
- ESP32-specific volume handling

**Configuration**:
- Enabled via: `AP_FILESYSTEM_ESP32_ENABLED` (automatic on ESP32)
- Mount point: `/SDCARD` or other ESP-IDF configured volumes

**Implementation Details**:
- Wraps ESP-IDF VFS layer
- Uses ESP-IDF FatFS implementation
- Thread safety: ESP-IDF semaphore protection

**Common Use Cases**:
- SD card access on ESP32-based autopilots
- File logging on ESP32 platforms

### PARAM Backend (AP_Filesystem_Param)

**Purpose**: Virtual filesystem for efficient parameter downloads

**Source**: `libraries/AP_Filesystem/AP_Filesystem_Param.cpp`

**Key Features**:
- Packed binary parameter format for fast transfer
- Read-only virtual file: `@PARAM/param.pck`
- Block-aligned padding prevents corruption on retransmit
- Query string support for partial downloads and defaults

**Configuration**:
- Enabled via: `AP_FILESYSTEM_PARAM_ENABLED` (default: 1)
- Access path: `@PARAM/param.pck[?query]`

**Implementation Details**:
- Generates packed parameter list on-demand
- Caches packing for consistent read sizes
- Thread safety: Prevents parameter changes during read
- See **The @PARAM VFS** section below for detailed file format

### SYS Backend (AP_Filesystem_Sys)

**Purpose**: Virtual filesystem for system diagnostics and introspection

**Source**: `libraries/AP_Filesystem/AP_Filesystem_Sys.cpp`

**Key Features**:
- Virtual files exposing system internals
- Read-only access to diagnostic information
- No storage overhead (generated on-demand)

**Available Virtual Files**:
- `@SYS/threads.txt`: Thread stack usage (ChibiOS only)
- `@SYS/tasks.txt`: Scheduler task timing and load
- `@SYS/crash_dump.bin`: Crash dump from last reset (if available)
- `@SYS/flash*.bin`: Internal flash inspection

**Configuration**:
- Enabled via: `AP_FILESYSTEM_SYS_ENABLED` (default: 1)
- Flash inspection: `AP_FILESYSTEM_SYS_FLASH_ENABLED` (ChibiOS only)
- Access path: `@SYS/` prefix

**Implementation Details**:
- Files generated dynamically on open
- Content cached until file closed
- Thread safety: Captures snapshot on open
- See **The @SYS VFS** section below for detailed content descriptions

### MISSION Backend (AP_Filesystem_Mission)

**Purpose**: Virtual filesystem for mission/fence/rally point binary storage

**Source**: `libraries/AP_Filesystem/AP_Filesystem_Mission.cpp`

**Key Features**:
- Binary mission file format for efficient transfer
- Write support for mission upload
- Validation during streaming writes
- Separate files for mission, fence, and rally points

**Available Virtual Files**:
- `@MISSION/mission.dat`: Waypoint mission
- `@MISSION/fence.dat`: Geofence definition
- `@MISSION/rally.dat`: Rally points

**Configuration**:
- Enabled via: `AP_FILESYSTEM_MISSION_ENABLED` (linked to `AP_MISSION_ENABLED`)
- Access path: `@MISSION/` prefix

**Implementation Details**:
- Wraps AP_Mission storage APIs
- Binary format matches internal mission storage
- Write validation prevents corrupted missions
- Thread safety: Mission library semaphores

## Backend Selection and Usage Patterns

### When to Use Each Backend

| Backend | Storage Type | Use Case | Capacity | Speed | Removable | Wear Limit |
|---------|-------------|----------|----------|-------|-----------|------------|
| **FATFS** | SD Card | Logs, large files | 2GB-512GB | Medium | Yes | ~100K writes |
| **LittleFS** | SPI Flash | Parameters, small logs | 2-16MB | Fast | No | ~100K writes |
| **ROMFS** | Firmware ROM | Static resources | ~100KB | Very Fast | No | Infinite (read-only) |
| **POSIX** | OS Filesystem | SITL/Linux dev | Unlimited | Fast | Varies | OS-dependent |
| **ESP32** | SD Card | ESP32 logs/storage | 2GB-512GB | Medium | Yes | ~100K writes |
| **PARAM** | Virtual | Parameter transfer | N/A | N/A | N/A | N/A (virtual) |
| **SYS** | Virtual | Diagnostics | N/A | N/A | N/A | N/A (virtual) |
| **MISSION** | Virtual | Mission upload | N/A | N/A | N/A | N/A (virtual) |

### Path Prefix Routing

AP_Filesystem routes operations based on path prefixes:

- **No prefix or `/`**: Local filesystem (FATFS/LittleFS/POSIX/ESP32)
  - Example: `/APM/logs/latest.bin`
  - Example: `parameters.parm`
- **`@ROMFS/`**: Embedded read-only resources
  - Example: `@ROMFS/font0.bin`
- **`@PARAM/`**: Virtual parameter interface
  - Example: `@PARAM/param.pck?start=0&count=100`
- **`@SYS/`**: System diagnostics
  - Example: `@SYS/threads.txt`
- **`@MISSION/`**: Mission/fence/rally data
  - Example: `@MISSION/mission.dat`

### Storage Trade-offs

**SD Card (FATFS)**:
- **Pros**: Large capacity, removable for offline analysis, widely compatible
- **Cons**: Slower than flash, mechanical/connection failure risk, may not be present on all boards
- **Best for**: High-volume logging, terrain data, large file storage

**Internal Flash (LittleFS)**:
- **Pros**: Always available, fast, integrated, wear-leveling, power-safe
- **Cons**: Limited capacity, finite write endurance, not removable
- **Best for**: Parameter storage, system config, small logs on boards without SD

**Embedded ROM (ROMFS)**:
- **Pros**: No runtime storage cost, very fast, cannot be corrupted
- **Cons**: Read-only, fixed at compile time, increases firmware size
- **Best for**: Fonts, lookup tables, default configs, immutable resources

## File Operations API

AP_Filesystem provides a POSIX-like API accessible through the `AP::FS()` singleton.

### Core Operations

#### open()
```cpp
int AP::FS().open(const char *fname, int flags, bool allow_absolute_paths = false)
```
Opens a file and returns a file descriptor.

- **Parameters**:
  - `fname`: Path to file (may include `@` prefix for virtual filesystems)
  - `flags`: POSIX flags (`O_RDONLY`, `O_WRONLY`, `O_RDWR`, `O_CREAT`, `O_TRUNC`, `O_APPEND`)
  - `allow_absolute_paths`: Allow absolute paths (security control)
- **Returns**: File descriptor >= 0 on success, -1 on error (errno set)
- **Thread safety**: Backend-dependent, typically semaphore-protected

**Example**:
```cpp
int fd = AP::FS().open("/APM/logs/log001.bin", O_RDONLY);
if (fd < 0) {
    // Error: errno contains error code
    return;
}
```

#### close()
```cpp
int AP::FS().close(int fd)
```
Closes an open file descriptor.

- **Returns**: 0 on success, -1 on error
- **Note**: Always close file descriptors to prevent leaks (MAX_FILES limit)

#### read()
```cpp
int32_t AP::FS().read(int fd, void *buf, uint32_t count)
```
Reads data from file.

- **Returns**: Bytes read (0-count), 0 on EOF, -1 on error

#### write()
```cpp
int32_t AP::FS().write(int fd, const void *buf, uint32_t count)
```
Writes data to file.

- **Returns**: Bytes written, -1 on error
- **Note**: May return short write; caller should retry

#### lseek()
```cpp
int32_t AP::FS().lseek(int fd, int32_t offset, int whence)
```
Repositions file pointer.

- **whence**: `SEEK_SET` (absolute), `SEEK_CUR` (relative), `SEEK_END` (from end)
- **Returns**: New file position, -1 on error

#### fsync()
```cpp
int AP::FS().fsync(int fd)
```
Flushes buffered writes to storage.

- **Returns**: 0 on success, -1 on error
- **Note**: Important for crash safety, but slow; use `bytes_until_fsync()` for optimization

### Extended Operations

#### stat()
```cpp
int AP::FS().stat(const char *pathname, struct stat *stbuf)
bool AP::FS().stat(const char *pathname, stat_t &stbuf)  // Scripting variant
```
Gets file/directory information.

- **Fields**: `size`, `mode`, `mtime`, `atime`, `ctime`
- **Returns**: 0 on success, -1 on error

#### unlink()
```cpp
int AP::FS().unlink(const char *pathname)
```
Deletes a file.

- **Returns**: 0 on success, -1 on error

#### mkdir()
```cpp
int AP::FS().mkdir(const char *pathname)
```
Creates a directory.

- **Returns**: 0 on success, -1 on error

#### rename()
```cpp
int AP::FS().rename(const char *oldpath, const char *newpath)
```
Renames/moves a file.

- **Returns**: 0 on success, -1 on error

### Directory Operations

#### opendir()
```cpp
DirHandle *AP::FS().opendir(const char *pathname)
```
Opens a directory for listing.

- **Returns**: Directory handle, nullptr on error

#### readdir()
```cpp
struct dirent *AP::FS().readdir(DirHandle *dirp)
```
Reads next directory entry.

- **Returns**: Pointer to dirent (valid until next readdir/closedir), nullptr on end/error
- **dirent fields**: `d_name` (filename), `d_type` (`DT_REG`=file, `DT_DIR`=directory)

#### closedir()
```cpp
int AP::FS().closedir(DirHandle *dirp)
```
Closes directory handle.

**Directory Listing Example**:
```cpp
auto *dir = AP::FS().opendir("/APM/logs");
if (dir != nullptr) {
    struct dirent *de;
    while ((de = AP::FS().readdir(dir)) != nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "File: %s", de->d_name);
    }
    AP::FS().closedir(dir);
}
```

### Utility Functions

#### load_file()
```cpp
FileData *AP::FS().load_file(const char *filename)
```
Loads entire file into memory.

- **Returns**: `FileData` object (must `delete` when done), nullptr on error
- **Note**: Data is null-terminated for string use

#### bytes_until_fsync()
```cpp
uint32_t AP::FS().bytes_until_fsync(int fd)
```
Returns optimal write size before calling fsync().

- **Returns**: Bytes before fsync recommended, 0 if no specific requirement
- **Use case**: Streaming writes (logging) to optimize performance vs crash safety

#### disk_free()
```cpp
int64_t AP::FS().disk_free(const char *path)
```
Returns free space in bytes.

- **Returns**: Free bytes, -1 on error

#### disk_space()
```cpp
int64_t AP::FS().disk_space(const char *path)
```
Returns total space in bytes.

- **Returns**: Total bytes, -1 on error

#### set_mtime()
```cpp
bool AP::FS().set_mtime(const char *filename, const uint32_t mtime_sec)
```
Sets file modification time.

- **Returns**: true on success

#### crc32()
```cpp
bool AP::FS().crc32(const char *fname, uint32_t& checksum)
```
Calculates CRC32 checksum of file.

- **Returns**: true on success, checksum in output parameter

#### format()
```cpp
bool AP::FS().format(void)
```
Formats the local filesystem (async operation).

- **Returns**: true if format started
- **Monitor**: Use `get_format_status()` to check progress

#### get_format_status()
```cpp
AP_Filesystem_Backend::FormatStatus AP::FS().get_format_status() const
```
Returns format operation status.

- **Values**: `FormatStatus::NotStarted`, `FormatStatus::InProgress`, `FormatStatus::Succeeded`, `FormatStatus::Failed`

#### fgets()
```cpp
bool AP::FS().fgets(char *buf, uint8_t buflen, int fd)
```
Reads a line from file (null or CR/LF terminated).

- **Returns**: true on success

### Error Handling

File operations set `errno` on failure following POSIX conventions:

- **ENOENT**: File not found
- **EACCES**: Permission denied
- **ENOSPC**: No space left on device
- **EINVAL**: Invalid argument
- **EIO**: I/O error
- **ENOMEM**: Out of memory
- **ERANGE**: File descriptor out of range

**Error Handling Pattern**:
```cpp
int fd = AP::FS().open("/APM/config.txt", O_RDONLY);
if (fd < 0) {
    if (errno == ENOENT) {
        // File doesn't exist
    } else {
        // Other error
    }
    return;
}
```

## MAVLink FTP

ArduPilot implements the FILE_TRANSFER_PROTOCOL MAVLink message to
allow for remote file operations. This protocol allows a GCS to
transfer files to and from a flight controller. It also allows the GCS
to access some special purpose VFS interfaces for efficient access to
flight controller data using a file API.

The VFS interfaces that don't represent local filesystem objects on
the flight controller are prefixed with an '@' symbol. Currently there
are two interfaces, the @PARAM interface and the @SYS interface.

### FTP Protocol Extension

To facilitate more efficient file transfer over commonly used SiK
radios I have added an extension to the ftp burst protocol where the
'size' field in the burst read request sets the block size of the
burst replies. This helps as SiK radios do badly with very large
packets. I have found that the best results with SiK radios is
achieved with a burst read size of 110. If the size field is set to
zero then the default of the max size (239) is used.

## The @PARAM VFS

The @PARAM VFS allows a GCS to very efficiently download full or
partial parameter list from the flight controller. Currently the
@PARAM filesystem offers only a single file, called @PARAM/param.pck,
which is a packed representation of the full parameter
list. Downloading the full parameter list via this interface is a lot
faster than using the traditional mavlink parameter messages.

The @PARAM/param.pck file has a special restriction that all reads
from the file on a single file handle must be of the same size. This
allows the server to ensure that filling in of lost transfers cannot
cause a parameter value to be split across a network block, which
prevents corruption. Attempts to vary the read size after the first
read will return a failed read.

The file format of the @PARAM/param.pck file is as follows

### File header

There is a 6 byte header, consisting of 3 uint16_t values
```
  uint16_t magic # 0x671b
  uint16_t num_params
  uint16_t total_params
```
The magic value is used to give the version of the packing format. It
should have a value of 0x671b. The num_params is how many parameters
will be sent (may be less than total if client requests a subset, see
query strings below). The total_params is the total number of
parameters the flight controller has.

The header is little-endian.

### Parameter Block

After the header comes a series of variable length parameter blocks, one per
parameter. The format is:

```
    uint8_t type:4;         // AP_Param type NONE=0, INT8=1, INT16=2, INT32=3, FLOAT=4
    uint8_t flags:4;        // bit 0: default value included, bits 1-3: for future use
    uint8_t common_len:4;   // number of name bytes in common with previous entry, 0..15
    uint8_t name_len:4;     // non-common length of param name -1 (0..15)
    uint8_t name[name_len]; // name
    uint8_t data[];         // value, length given by variable type
    uint8_t default[];      // optional default value, included if flags bit 0 is set
```

There may be any number of leading zero pad bytes before the start of
the parameter block. The pad bytes are added to ensure that a
parameter value does not cross a MAVLink FTP block boundary. This
padding prevents a re-fetch of a missing block from potentially
leading to a corrupt value.

The packing format avoids sending common leading characters of
parameter names from the previous parameter. It does this by sending a
common_len field which says how many characters should be copied from
the previous parameter. This is always zero in the first block in the
file.

The name_len field is the number of non-common characters, minus one.

The default value is included only if requested by the withdefaults
query string and if different from the set value. Otherwise, flags
bit 0 will not be set, and default will be of zero length.

### Query Strings

The file name @PARAM/param.pck can optionally be extended with query
string elements to change the result. For example:

 - @PARAM/param.pck?start=50&count=10

that means to download 10 parameters starting with parameter number
50.

 - @PARAM/param.pck?withdefaults=1

that means to include the default values in the returned data, where
it is different from the parameter's set value.

### Parameter Client Examples

The script Tools/scripts/param_unpack.py can be used to unpack a
param.pck file. Additionally the MAVProxy mavproxy_param.py module
implements parameter download via ftp.

## The @SYS VFS

The @SYS VFS gives access to flight controller internals through virtual files that
expose system diagnostics and runtime information. These files are generated on-demand
and do not consume storage space.

### Available Files

#### @SYS/threads.txt

**Platform**: ChibiOS only

**Content**: Thread stack usage information for all running threads.

**Format**: Text file with one line per thread:
```
THREAD_NAME: total_stack_bytes, used_stack_bytes, free_stack_bytes
```

**Example Content**:
```
IDLE: 128, 32, 96
main: 8192, 2048, 6144
AP_Vehicle: 4096, 1024, 3072
```

**Use Cases**:
- Detecting stack overflow risks
- Optimizing stack allocations
- Runtime monitoring of thread memory usage

**Implementation**: Queries ChibiOS thread registry and calculates stack usage
from stack watermarking (Source: `libraries/AP_Filesystem/AP_Filesystem_Sys.cpp`)

#### @SYS/tasks.txt

**Platform**: All platforms

**Content**: Scheduler task timing and load information.

**Format**: Text file listing all scheduler tasks with:
- Task name
- Average execution time
- Maximum execution time
- Schedule interval
- Loop rate

**Use Cases**:
- Performance profiling
- Identifying slow tasks
- Load balancing analysis

**Implementation**: Reads scheduler performance counters maintained by AP_Scheduler
(Source: `libraries/AP_Filesystem/AP_Filesystem_Sys.cpp`)

#### @SYS/crash_dump.bin

**Platform**: ChibiOS (with crash dump support)

**Content**: Binary crash dump from last reset if a crash was captured.

**Format**: Platform-specific binary format containing:
- Register state at crash
- Stack trace
- Thread information
- Fault status registers

**Use Cases**:
- Post-crash debugging
- Firmware issue diagnosis
- Flight controller failure analysis

**Implementation**: Reads persistent crash dump from watchdog-protected memory region
(Source: `libraries/AP_Filesystem/AP_Filesystem_Sys.cpp`)

#### @SYS/flash*.bin

**Platform**: ChibiOS only (`AP_FILESYSTEM_SYS_FLASH_ENABLED`)

**Content**: Raw internal flash memory inspection.

**Files**:
- `@SYS/flash0.bin`: First flash bank
- `@SYS/flash1.bin`: Second flash bank (if present)

**Use Cases**:
- Flash memory diagnostics
- Bootloader inspection
- Low-level debugging

**Safety**: Read-only access, no write support

**Implementation**: Memory-mapped read of internal flash regions
(Source: `libraries/AP_Filesystem/AP_Filesystem_Sys.cpp`)

### Virtual File Implementation Details

**Generation**: Files generated dynamically when opened; content snapshot captured
and cached until file closed.

**Thread Safety**: Captures system snapshot at open time with appropriate locking.

**Error Handling**: Returns -ENOENT if requested file doesn't exist or feature not
supported on platform.

**GCS Access**: All @SYS files accessible via MAVLink FTP protocol for remote diagnostics.

## Initialization and Mount Procedures

### Backend Initialization

Filesystem backends initialize at boot according to platform and configuration:

1. **Static Allocation**: Backend instances created as static globals in `AP_Filesystem.cpp`
2. **HAL Initialization**: Hardware drivers (SD card, SPI flash) initialized by HAL layer
3. **Backend Registration**: Backends registered in `backends[]` array at compile time
4. **Mount Attempts**: Backends attempt mount during first operation or explicitly

### Automatic Mount Behavior

**FATFS (SD Card)**:
- Mount attempted on first file operation
- Retry logic: Attempts remount on operation failure
- Detection: SD card presence detected by HAL
- Failure mode: Operations return -ENOENT if SD card not present

**LittleFS (SPI Flash)**:
- Mount attempted on initialization
- Retry count: Configurable mount retries
- Auto-format: Formats flash if unformatted or corrupted
- Failure mode: Falls back to read-only or disables filesystem

**POSIX/ESP32**:
- Direct passthrough to OS, no explicit mount
- Relies on OS mount state

**ROMFS**:
- No mount needed (memory-mapped firmware data)

### Manual Mount Control

```cpp
// Retry mount if filesystem not running
if (!AP::FS().retry_mount()) {
    // Mount failed
}

// Unmount filesystem for safe shutdown
AP::FS().unmount();
```

### Format Procedures

**Format API**:
```cpp
// Start asynchronous format
if (AP::FS().format()) {
    // Format started, monitor status
    while (true) {
        auto status = AP::FS().get_format_status();
        if (status == AP_Filesystem_Backend::FormatStatus::Succeeded) {
            // Format complete
            break;
        } else if (status == AP_Filesystem_Backend::FormatStatus::Failed) {
            // Format failed
            break;
        }
        // Still in progress...
        hal.scheduler->delay(100);
    }
}
```

**Format Behavior**:
- **FATFS**: Formats SD card to FAT32
- **LittleFS**: Formats internal flash (erases all data)
- **Other backends**: Format not supported

**Warning**: Formatting erases all data. Ensure critical data is backed up.

### Safe Unmount

Before reboot or power-off, ensure filesystem sync:

```cpp
// Sync all open file descriptors
for (int fd : open_fds) {
    AP::FS().fsync(fd);
    AP::FS().close(fd);
}

// Unmount filesystem
AP::FS().unmount();
```

## Thread Safety and Concurrency

### Semaphore Protection

All backend implementations use semaphore protection for thread-safe operations:

```cpp
// Internal backend pattern (example from LittleFS)
int AP_Filesystem_Backend::open(const char *fname, int flags) {
    WITH_SEMAPHORE(sem);  // Automatically locks/unlocks
    // ... operation ...
}
```

### File Operation Safety During Armed State

File operations are gated during armed state to prevent timing interference with
flight-critical tasks:

```cpp
// Internal check (in backend base class)
bool AP_Filesystem_Backend::file_op_allowed() const {
    // Check if vehicle is armed
    if (hal.util->get_soft_armed()) {
        // Some operations restricted during flight
        return false;
    }
    return true;
}
```

**Restrictions**:
- **During flight (armed)**: Some file operations may be restricted or throttled
- **Logging**: Continues during flight (optimized with buffering)
- **Mission upload**: Typically blocked during armed state

### Scheduler Integration

Filesystem operations execute in the caller's thread context:

- **Main thread**: Avoid long file operations in fast scheduler tasks
- **IO thread**: File operations typically called from IO thread for logging
- **Slow tasks**: File management operations run in slow scheduler tasks

**Best Practices**:
- Perform file operations in IO thread or slow tasks
- Avoid blocking main loop with synchronous file I/O
- Use async patterns where possible (e.g., logger buffering)

### Concurrent Access Limitations

**File Descriptors**:
- **Per-backend limit**: MAX_FD_PER_BACKEND (256)
- **System-wide**: NUM_BACKENDS * 256
- **Leak prevention**: Always close files; leaked FDs exhaust available handles

**Directory Listing**:
- Non-reentrant per directory handle
- Multiple threads can open different directories safely

**Write Conflicts**:
- No multi-writer protection; application must coordinate writes to same file
- Simultaneous writes to different files: Safe

## Troubleshooting Guide

### SD Card Detection Failures

**Symptoms**:
- Log file writes fail
- SD card not detected at boot
- File operations return -ENOENT

**Diagnosis**:
```cpp
// Check disk space
int64_t free = AP::FS().disk_free("/");
if (free < 0) {
    // SD card not mounted
}
```

**Solutions**:
1. **Check physical connection**: Ensure SD card seated properly
2. **Verify card compatibility**: Use Class 10 or better, FAT32 formatted
3. **Retry mount**: `AP::FS().retry_mount()`
4. **Check logs**: Look for FATFS errors in system logs
5. **Reformat**: Use `AP::FS().format()` to reformat card
6. **Hardware test**: Try card in computer to rule out card failure

### Filesystem Corruption

**Symptoms**:
- Mount failures after power loss
- File read/write errors
- Directory listing failures

**FATFS Error Codes**:
- `FR_DISK_ERR`: Hardware I/O error
- `FR_NOT_READY`: Drive not ready (card removed?)
- `FR_NO_FILESYSTEM`: Partition table or boot sector corrupted

**LittleFS Errors**:
- `LFS_ERR_CORRUPT`: Filesystem metadata corrupted
- `LFS_ERR_IO`: Flash I/O error

**Recovery Procedures**:

1. **Attempt remount**:
```cpp
AP::FS().unmount();
hal.scheduler->delay(100);
if (AP::FS().retry_mount()) {
    // Remount successful
}
```

2. **Reformat filesystem** (erases data):
```cpp
AP::FS().format();
```

3. **Check hardware**:
   - SD card: Try different card
   - SPI flash: Check SPI bus integrity

### Common Error Messages and Solutions

| Error Code | Meaning | Common Causes | Solution |
|------------|---------|---------------|----------|
| ENOENT | No such file or directory | File doesn't exist, SD card not mounted | Check path, verify mount |
| ENOSPC | No space left on device | Disk full | Delete files, use larger SD card |
| EIO | I/O error | Hardware failure, bad sectors | Check hardware, reformat |
| EINVAL | Invalid argument | Bad flags, null pointer | Check API usage |
| ENOMEM | Out of memory | RAM exhausted | Close files, reduce memory usage |
| EACCES | Permission denied | Read-only filesystem, file locked | Check backend type, permissions |

### Log Analysis for Filesystem Issues

**Enable filesystem debug logging**:
```
LOG_FILE_DSRMASK 0xFFFF  // Enable all DataFlash message types
```

**Check for**:
- Mount failure messages
- I/O error reports
- SD card detection events
- Filesystem format progress

**GCS Messages**:
- Watch for "SD card" messages in GCS console
- "Logging started" indicates successful mount
- "Logging stopped" may indicate SD card removal

### Hardware Troubleshooting

**SD Card Issues**:
- **Symptom**: Intermittent errors, slow writes
- **Test**: Try card in PC, check for errors
- **Solution**: Use high-quality card (SanDisk, Samsung), format FAT32

**SPI Flash Problems**:
- **Symptom**: LittleFS mount failures
- **Diagnosis**: Check `@SYS/flash0.bin` readability
- **Solution**: Verify SPI bus signals, check flash chip soldering

**SPI Bus Conflicts**:
- **Symptom**: Flash operations fail when other SPI devices active
- **Solution**: Check SPI CS (chip select) signals, ensure proper pullups

## Testing

### SITL Filesystem Testing

**Setup SITL with filesystem**:
```bash
# Start SITL with file-backed storage
sim_vehicle.py -v ArduCopter --console --map
```

**Test filesystem operations** in MAVProxy:
```
# List files
module load ftp
ftp list /
ftp list @SYS/

# Download files
ftp get @PARAM/param.pck param_backup.pck
ftp get @SYS/threads.txt threads.txt

# Upload test file
ftp put test.txt /test.txt
```

**Native POSIX paths** in SITL:
- Files written to: `$HOME/tmp/ArduCopter.whatever/` by default
- Accessible from host OS for inspection

### Hardware Testing Workflows

**SD Card Benchmark**:
```cpp
// Measure write performance
uint32_t start = AP_HAL::millis();
int fd = AP::FS().open("/APM/test.bin", O_WRONLY | O_CREAT | O_TRUNC);
for (int i = 0; i < 1000; i++) {
    uint8_t buf[512];
    AP::FS().write(fd, buf, sizeof(buf));
}
AP::FS().fsync(fd);
AP::FS().close(fd);
uint32_t elapsed = AP_HAL::millis() - start;
// elapsed = time to write 512KB
```

**Flash Endurance Testing**:
- Write/erase cycles to test wear-leveling
- Monitor `disk_free()` after many writes
- Check for bad blocks or failures

**Mount/Remount Stress Test**:
```cpp
for (int i = 0; i < 100; i++) {
    AP::FS().unmount();
    hal.scheduler->delay(10);
    if (!AP::FS().retry_mount()) {
        // Mount failed
    }
}
```

### Example Test Scenarios

**File I/O Example** (from `libraries/AP_Filesystem/examples/File_IO`):
```cpp
#include <AP_Filesystem/AP_Filesystem.h>

void test_file_operations() {
    // Write test
    int fd = AP::FS().open("/test.txt", O_WRONLY | O_CREAT | O_TRUNC);
    const char *msg = "Hello ArduPilot\n";
    AP::FS().write(fd, msg, strlen(msg));
    AP::FS().close(fd);
    
    // Read test
    fd = AP::FS().open("/test.txt", O_RDONLY);
    char buf[32];
    int32_t bytes = AP::FS().read(fd, buf, sizeof(buf)-1);
    buf[bytes] = '\0';
    AP::FS().close(fd);
    
    // Verify
    if (strcmp(buf, msg) == 0) {
        // Success
    }
}
```

### Validation Procedures for New Backends

When implementing a new backend:

1. **Basic operations**: Verify open/close/read/write/lseek
2. **Error handling**: Test behavior on file not found, disk full, etc.
3. **Directory operations**: Test mkdir/opendir/readdir/closedir
4. **Edge cases**: Zero-length reads/writes, seek beyond EOF
5. **Concurrency**: Multiple file handles, thread safety
6. **Power-loss**: Test filesystem integrity after simulated power loss
7. **Performance**: Measure latency and throughput

## Configuration

### Feature Flags (AP_Filesystem_config.h)

**Backend Enable/Disable**:
```cpp
AP_FILESYSTEM_ESP32_ENABLED      // ESP32 filesystem backend
AP_FILESYSTEM_FATFS_ENABLED      // FATFS (SD card) backend
AP_FILESYSTEM_LITTLEFS_ENABLED   // LittleFS (SPI flash) backend
AP_FILESYSTEM_PARAM_ENABLED      // @PARAM virtual filesystem
AP_FILESYSTEM_POSIX_ENABLED      // POSIX (Linux/SITL) backend
AP_FILESYSTEM_ROMFS_ENABLED      // ROMFS embedded resources
AP_FILESYSTEM_SYS_ENABLED        // @SYS diagnostics
AP_FILESYSTEM_MISSION_ENABLED    // @MISSION virtual filesystem
```

**Flash Configuration**:
```cpp
AP_FILESYSTEM_LITTLEFS_FLASH_TYPE  // Flash chip type:
    AP_FILESYSTEM_FLASH_JEDEC_NOR  // Standard SPI NOR (default)
    AP_FILESYSTEM_FLASH_W25NXX     // Winbond W25N NAND
```

**Aggregate Capability Flags**:
```cpp
AP_FILESYSTEM_FILE_WRITING_ENABLED  // True if any writable backend enabled
AP_FILESYSTEM_FILE_READING_ENABLED  // True if any readable backend enabled
```

**Platform-Specific**:
```cpp
AP_FILESYSTEM_SYS_FLASH_ENABLED     // Enable @SYS/flash*.bin (ChibiOS only)
AP_FILESYSTEM_FORMAT_ENABLED        // Enable format() API
AP_FILESYSTEM_HAVE_DIRENT_DTYPE     // Platform supports dirent.d_type
```

### Board-Specific Defaults

**HAL Dependencies**:
- `HAL_OS_FATFS_IO`: Enables FATFS on ChibiOS
- `HAL_OS_LITTLEFS_IO`: Enables LittleFS
- `HAL_HAVE_AP_ROMFS_EMBEDDED_H`: Enables ROMFS if embedded resources present

**Example Board Configuration**:
```cpp
// Board with SD card (hwdef)
define HAL_OS_FATFS_IO 1

// Board with SPI flash (hwdef)
define HAL_OS_LITTLEFS_IO 1
define AP_FILESYSTEM_LITTLEFS_FLASH_TYPE AP_FILESYSTEM_FLASH_JEDEC_NOR
```

### Compile-Time Selection

Only **one** local filesystem backend is compiled per platform:
- Priority: FATFS > ESP32 > LittleFS > POSIX
- Virtual backends (@PARAM, @SYS, @MISSION, @ROMFS) can coexist with local backend

## POSIX Compatibility Layer

### Unbuffered FILE* API (posix_compat.h/cpp)

AP_Filesystem provides unbuffered POSIX-compatible `FILE*` functions for Lua scripting
and legacy code compatibility:

```cpp
FILE *apfs_fopen(const char *pathname, const char *mode);
int apfs_fclose(FILE *stream);
size_t apfs_fread(void *ptr, size_t size, size_t nmemb, FILE *stream);
size_t apfs_fwrite(const void *ptr, size_t size, size_t nmemb, FILE *stream);
int apfs_fseek(FILE *stream, long offset, int whence);
long apfs_ftell(FILE *stream);
int apfs_feof(FILE *stream);
int apfs_ferror(FILE *stream);
```

**Implementation**: Thin wrappers around `AP::FS()` file descriptor API

**Differences from standard FILE***:
- **Unbuffered**: No internal buffering (performance implications)
- **Limited**: Subset of full stdio.h functionality
- **errno**: Error handling via errno, consistent with POSIX

**Use Cases**:
- Lua scripting file I/O
- Cross-platform code requiring FILE* interface
- Legacy code migration

### Cross-Platform Consistency

AP_Filesystem provides consistent behavior across platforms:
- **Error codes**: Standard errno values
- **Path separators**: Forward slash `/` on all platforms
- **File modes**: POSIX flags (O_RDONLY, O_WRONLY, O_RDWR, etc.)
- **Stat structure**: Consistent fields across backends

## Safety Considerations

### Flight-Critical Restrictions

**File Operations During Armed State**:

Some file operations are restricted or throttled when vehicle is armed to prevent
interference with flight-critical tasks:

```cpp
// Check in backend base class
bool AP_Filesystem_Backend::file_op_allowed() const {
    if (hal.util->get_soft_armed()) {
        // Operations may be restricted
        return false;
    }
    return true;
}
```

**Allowed During Flight**:
- **Logging writes**: Buffered, non-blocking, optimized for flight
- **Reading mission/params**: Cached, minimal overhead

**Restricted During Flight**:
- **Formatting**: Never allowed while armed
- **File deletion**: May be blocked
- **Large file operations**: May be throttled

### Timing Constraints

**Scheduler Considerations**:
- File operations are **blocking** calls
- Avoid file I/O in fast scheduler loops (>100Hz)
- Use IO thread or slow tasks for file operations
- Logger uses buffering to avoid blocking main loop

**Latency Characteristics**:
- **SD card write**: 1-10ms per fsync()
- **Flash write**: <1ms typical
- **Read operations**: <1ms typical
- **Directory listing**: Variable (proportional to file count)

### Resource Limits

**File Descriptors**:
- **Maximum**: `NUM_BACKENDS * MAX_FD_PER_BACKEND` (typically 5 * 256 = 1280)
- **Per-backend**: 256 file descriptors
- **Leak impact**: Exhausted FDs prevent new file opens

**Memory Allocation**:
- **File handles**: Allocated from heap (stack used during operations)
- **Directory handles**: Allocated dynamically
- **Load_file()**: Allocates memory for entire file (caller must delete)

**Best Practice**: Always close files and free resources:
```cpp
int fd = AP::FS().open(path, O_RDONLY);
if (fd >= 0) {
    // ... use file ...
    AP::FS().close(fd);  // Always close, even on error paths
}
```

### Failsafe Behavior on Storage Failures

**SD Card Removal**:
- **Detection**: Operations return -EIO or -ENOENT
- **Logging**: Logger detects failure, attempts remount
- **Flight impact**: Vehicle continues flying (logging lost)
- **Recovery**: Logger resumes when SD card reinserted

**Flash Failure**:
- **Detection**: Mount failures, I/O errors
- **Parameter storage**: Falls back to defaults if flash corrupted
- **Flight impact**: May prevent arming if parameter load fails
- **Recovery**: Reformat or replace hardware

**Error Recovery Pattern**:
```cpp
int fd = AP::FS().open(path, O_RDONLY);
if (fd < 0) {
    if (errno == ENOENT || errno == EIO) {
        // Storage failure - try remount
        if (AP::FS().retry_mount()) {
            // Retry operation
            fd = AP::FS().open(path, O_RDONLY);
        }
    }
}
```

### Data Integrity

**Power-Loss Protection**:
- **LittleFS**: Power-safe writes (crash-safe, no corruption)
- **FATFS**: Not power-safe; corruption possible on power loss during write
- **Mitigation**: Frequent `fsync()` calls reduce window of vulnerability

**Corruption Prevention**:
- **@PARAM files**: Block-aligned padding prevents partial parameter corruption
- **Logger**: Write-ahead buffering minimizes corruption risk
- **Format on corruption**: Auto-format on mount failure (LittleFS)

**Validation**:
- **CRC checks**: Use `crc32()` to verify file integrity
- **Read-back verification**: Verify critical writes

## References

### Source Code

Primary implementation files:
- `libraries/AP_Filesystem/AP_Filesystem.cpp` - Router and core API
- `libraries/AP_Filesystem/AP_Filesystem.h` - Public API definitions
- `libraries/AP_Filesystem/AP_Filesystem_backend.h` - Backend base class
- `libraries/AP_Filesystem/AP_Filesystem_FATFS.cpp` - SD card backend (ChibiOS)
- `libraries/AP_Filesystem/AP_Filesystem_FlashMemory_LittleFS.cpp` - SPI flash backend
- `libraries/AP_Filesystem/AP_Filesystem_ROMFS.cpp` - Embedded resources backend
- `libraries/AP_Filesystem/AP_Filesystem_posix.cpp` - Linux/SITL backend
- `libraries/AP_Filesystem/AP_Filesystem_ESP32.cpp` - ESP32 backend
- `libraries/AP_Filesystem/AP_Filesystem_Param.cpp` - Virtual parameter interface
- `libraries/AP_Filesystem/AP_Filesystem_Sys.cpp` - Virtual system diagnostics
- `libraries/AP_Filesystem/AP_Filesystem_Mission.cpp` - Virtual mission interface
- `libraries/AP_Filesystem/posix_compat.cpp` - FILE* compatibility layer
- `libraries/AP_Filesystem/AP_Filesystem_config.h` - Feature configuration

### Related Modules

- `libraries/AP_Logger/` - DataFlash logging system (major filesystem user)
- `libraries/AP_Param/` - Parameter storage system
- `libraries/AP_Mission/` - Mission/fence/rally storage
- `GCS_MAVLink/` - MAVLink FTP protocol implementation

### External Documentation

- FatFs Documentation: http://elm-chan.org/fsw/ff/00index_e.html
- LittleFS: https://github.com/littlefs-project/littlefs
- MAVLink File Transfer Protocol: https://mavlink.io/en/services/ftp.html
- ArduPilot Wiki: https://ardupilot.org/dev/
