/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Andy Piper and Siddharth Bharat Purohit
 */

/**
 * @file WSPIDevice.h
 * @brief Wide SPI (Quad/Octo SPI) device interface for high-speed external flash and peripherals
 * 
 * @details Defines abstract interface for Wide SPI devices supporting Quad SPI (4-bit) and
 *          Octo SPI (8-bit) parallel data transfer modes. Wide SPI extends standard SPI by
 *          allowing multiple data lines to operate in parallel, significantly increasing
 *          throughput for memory-intensive operations.
 *          
 *          Wide SPI is primarily used for:
 *          - External flash memory (logging, parameter storage, terrain data)
 *          - High-bandwidth sensor interfaces
 *          - Fast firmware updates
 *          
 *          Typical transfer phases:
 *          1. Command phase: 8-bit command code (standard or quad/octo mode)
 *          2. Address phase: 8/16/24/32-bit address (standard or quad/octo mode)
 *          3. Alternate byte phase: Optional mode/dummy bytes
 *          4. Data phase: Bulk transfer (standard, dual, quad, or octo mode)
 *          
 *          Hardware support: STM32H7, STM32F7 (Quad only), and similar advanced MCUs
 * 
 * @see Device.h for base device interface
 * @see AP_HAL::Device::CommandHeader for transfer configuration structure
 * 
 * Source: libraries/AP_HAL/WSPIDevice.h
 */
#pragma once

#include <inttypes.h>

#include "AP_HAL_Namespace.h"
#include "Device.h"
#include "utility/OwnPtr.h"

#ifndef HAL_USE_WSPI_DEFAULT_CFG
#define HAL_USE_WSPI_DEFAULT_CFG 1
#endif

namespace AP_HAL
{

/**
 * @namespace AP_HAL::WSPI
 * @brief Configuration constants for Wide SPI transfer modes
 * 
 * @details Defines bit masks and values for configuring WSPI command headers.
 *          These constants control the number of data lines used for each
 *          transfer phase (command, address, alternate bytes, data).
 *          
 *          Transfer mode options:
 *          - NONE: Phase not used
 *          - ONE_LINE: Standard SPI (1-bit MOSI/MISO)
 *          - TWO_LINES: Dual SPI (2× throughput)
 *          - FOUR_LINES: Quad SPI (4× throughput)
 *          - EIGHT_LINES: Octo SPI (8× throughput, OCTOSPI only)
 *          
 *          Size options control address and alternate byte lengths.
 *          DDR flags enable Double Data Rate (sample on both clock edges).
 *          SIOO flag enables Send Instruction Only Once optimization.
 *          
 *          Platform-specific: QuadSPI vs OctoSPI have different bit layouts.
 * 
 * @note Underlying HAL implementation can override these defaults
 * @warning Incorrect configuration causes device communication failure
 */
// Underlying HAL implementation can override these
#if HAL_USE_WSPI_DEFAULT_CFG
namespace WSPI
{
// QuadSPI configuration bit masks and values (STM32F7, STM32H7 Quad mode)
#if HAL_USE_QUADSPI
// Command phase configuration (bits 8-9): Controls how command byte is sent
constexpr uint32_t CFG_CMD_MODE_MASK        =   (3LU << 8LU);  ///< Mask for command mode bits
constexpr uint32_t CFG_CMD_MODE_NONE        =   (0LU << 8LU);  ///< No command phase
constexpr uint32_t CFG_CMD_MODE_ONE_LINE    =   (1LU << 8LU);  ///< Command on single line (standard SPI)
constexpr uint32_t CFG_CMD_MODE_TWO_LINES   =   (2LU << 8LU);  ///< Command on two lines (dual mode)
constexpr uint32_t CFG_CMD_MODE_FOUR_LINES  =   (3LU << 8LU);  ///< Command on four lines (quad mode)

// Command size configuration: QuadSPI supports only 8-bit commands
constexpr uint32_t CFG_CMD_SIZE_MASK     =  0LU;  ///< Command size mask (unused, always 8-bit)
constexpr uint32_t CFG_CMD_SIZE_8        =  0LU;  ///< 8-bit command size (default and only option)

// Address phase configuration (bits 10-11): Controls how address is sent
constexpr uint32_t CFG_ADDR_MODE_MASK        =  (3LU << 10LU);  ///< Mask for address mode bits
constexpr uint32_t CFG_ADDR_MODE_NONE        =  (0LU << 10LU);  ///< No address phase
constexpr uint32_t CFG_ADDR_MODE_ONE_LINE    =  (1LU << 10LU);  ///< Address on single line
constexpr uint32_t CFG_ADDR_MODE_TWO_LINES   =  (2LU << 10LU);  ///< Address on two lines
constexpr uint32_t CFG_ADDR_MODE_FOUR_LINES  =  (3LU << 10LU);  ///< Address on four lines

// Address size configuration (bits 12-13): Number of address bits
constexpr uint32_t CFG_ADDR_SIZE_MASK   =  (3LU << 12LU);  ///< Mask for address size bits
constexpr uint32_t CFG_ADDR_SIZE_8      =  (0LU << 12LU);  ///< 8-bit address
constexpr uint32_t CFG_ADDR_SIZE_16     =  (1LU << 12LU);  ///< 16-bit address
constexpr uint32_t CFG_ADDR_SIZE_24     =  (2LU << 12LU);  ///< 24-bit address (common for flash)
constexpr uint32_t CFG_ADDR_SIZE_32     =  (3LU << 12LU);  ///< 32-bit address (large flash devices)

// Alternate byte phase configuration (bits 14-15): Optional mode/dummy bytes
constexpr uint32_t CFG_ALT_MODE_MASK        =  (3LU << 14LU);  ///< Mask for alternate byte mode bits
constexpr uint32_t CFG_ALT_MODE_NONE        =  (0LU << 14LU);  ///< No alternate byte phase
constexpr uint32_t CFG_ALT_MODE_ONE_LINE    =  (1LU << 14LU);  ///< Alternate bytes on single line
constexpr uint32_t CFG_ALT_MODE_TWO_LINES   =  (2LU << 14LU);  ///< Alternate bytes on two lines
constexpr uint32_t CFG_ALT_MODE_FOUR_LINES  =  (3LU << 14LU);  ///< Alternate bytes on four lines

constexpr uint32_t CFG_ALT_DDR              =  (1LU << 31LU);  ///< Alternate bytes in DDR mode (both clock edges)

// Alternate byte size configuration (bits 16-17)
constexpr uint32_t CFG_ALT_SIZE_MASK        =  (3LU << 16LU);  ///< Mask for alternate byte size bits
constexpr uint32_t CFG_ALT_SIZE_8           =  (0LU << 16LU);  ///< 8-bit alternate bytes
constexpr uint32_t CFG_ALT_SIZE_16          =  (1LU << 16LU);  ///< 16-bit alternate bytes
constexpr uint32_t CFG_ALT_SIZE_24          =  (2LU << 16LU);  ///< 24-bit alternate bytes
constexpr uint32_t CFG_ALT_SIZE_32          =  (3LU << 16LU);  ///< 32-bit alternate bytes

// Data phase configuration (bits 24-25): Controls bulk data transfer mode
constexpr uint32_t CFG_DATA_MODE_MASK       =  (3LU << 24LU);  ///< Mask for data mode bits
constexpr uint32_t CFG_DATA_MODE_NONE       =  (0LU << 24LU);  ///< No data phase
constexpr uint32_t CFG_DATA_MODE_ONE_LINE   =  (1LU << 24LU);  ///< Data on single line (standard SPI)
constexpr uint32_t CFG_DATA_MODE_TWO_LINES  =  (2LU << 24LU);  ///< Data on two lines (dual SPI)
constexpr uint32_t CFG_DATA_MODE_FOUR_LINES =  (3LU << 24LU);  ///< Data on four lines (quad SPI)

constexpr uint32_t CFG_DATA_DDR             =  (1LU << 31LU);  ///< Data phase in DDR mode (both clock edges)

constexpr uint32_t CFG_SIOO                 =  (1LU << 28LU);  ///< Send Instruction Only Once (performance optimization)

// OctoSPI configuration bit masks and values (STM32H7 Octo mode, STM32L4+)
#else   // OCTOSPI
// Command phase configuration (bits 0-2): OctoSPI supports up to 8 data lines
constexpr uint32_t CFG_CMD_MODE_MASK        =   (7LU << 0LU);  ///< Mask for command mode bits
constexpr uint32_t CFG_CMD_MODE_NONE        =   (0LU << 0LU);  ///< No command phase
constexpr uint32_t CFG_CMD_MODE_ONE_LINE    =   (1LU << 0LU);  ///< Command on single line (standard SPI)
constexpr uint32_t CFG_CMD_MODE_TWO_LINES   =   (2LU << 0LU);  ///< Command on two lines (dual mode)
constexpr uint32_t CFG_CMD_MODE_FOUR_LINES  =   (3LU << 0LU);  ///< Command on four lines (quad mode)
constexpr uint32_t CFG_CMD_MODE_EIGHT_LINES  =  (4LU << 0LU);  ///< Command on eight lines (octo mode)

// Command size configuration (bits 4-5): OctoSPI supports configurable command width
constexpr uint32_t CFG_CMD_SIZE_MASK     =  (3LU << 4LU);  ///< Mask for command size bits
constexpr uint32_t CFG_CMD_SIZE_8        =  (0LU << 4LU);  ///< 8-bit command size (most common)

// Address phase configuration (bits 8-10): OctoSPI supports up to 8 address lines
constexpr uint32_t CFG_ADDR_MODE_MASK        =  (7LU << 8LU);  ///< Mask for address mode bits
constexpr uint32_t CFG_ADDR_MODE_NONE        =  (0LU << 8LU);  ///< No address phase
constexpr uint32_t CFG_ADDR_MODE_ONE_LINE    =  (1LU << 8LU);  ///< Address on single line
constexpr uint32_t CFG_ADDR_MODE_TWO_LINES   =  (2LU << 8LU);  ///< Address on two lines
constexpr uint32_t CFG_ADDR_MODE_FOUR_LINES  =  (3LU << 8LU);  ///< Address on four lines
constexpr uint32_t CFG_ADDR_MODE_EIGHT_LINES  = (4LU << 8LU);  ///< Address on eight lines (octo mode)


// Address size configuration (bits 12-13): Number of address bits
constexpr uint32_t CFG_ADDR_SIZE_MASK   =  (3LU << 12LU);  ///< Mask for address size bits
constexpr uint32_t CFG_ADDR_SIZE_8      =  (0LU << 12LU);  ///< 8-bit address
constexpr uint32_t CFG_ADDR_SIZE_16     =  (1LU << 12LU);  ///< 16-bit address
constexpr uint32_t CFG_ADDR_SIZE_24     =  (2LU << 12LU);  ///< 24-bit address (common for flash)
constexpr uint32_t CFG_ADDR_SIZE_32     =  (3LU << 12LU);  ///< 32-bit address (large flash devices)

// Alternate byte phase configuration (bits 16-18): Optional mode/dummy bytes for OctoSPI
constexpr uint32_t CFG_ALT_MODE_MASK        =  (7LU << 16LU);  ///< Mask for alternate byte mode bits
constexpr uint32_t CFG_ALT_MODE_NONE        =  (0LU << 16LU);  ///< No alternate byte phase
constexpr uint32_t CFG_ALT_MODE_ONE_LINE    =  (1LU << 16LU);  ///< Alternate bytes on single line
constexpr uint32_t CFG_ALT_MODE_TWO_LINES   =  (2LU << 16LU);  ///< Alternate bytes on two lines
constexpr uint32_t CFG_ALT_MODE_FOUR_LINES  =  (3LU << 16LU);  ///< Alternate bytes on four lines
constexpr uint32_t CFG_ALT_MODE_EIGHT_LINES  = (4LU << 16LU);  ///< Alternate bytes on eight lines

constexpr uint32_t CFG_ALT_DDR              =  (1LU << 19LU);  ///< Alternate bytes in DDR mode (both clock edges)

// Alternate byte size configuration (bits 20-21)
constexpr uint32_t CFG_ALT_SIZE_MASK        =  (3LU << 20LU);  ///< Mask for alternate byte size bits
constexpr uint32_t CFG_ALT_SIZE_8           =  (0LU << 20LU);  ///< 8-bit alternate bytes
constexpr uint32_t CFG_ALT_SIZE_16          =  (1LU << 20LU);  ///< 16-bit alternate bytes
constexpr uint32_t CFG_ALT_SIZE_24          =  (2LU << 20LU);  ///< 24-bit alternate bytes
constexpr uint32_t CFG_ALT_SIZE_32          =  (3LU << 20LU);  ///< 32-bit alternate bytes

// Data phase configuration (bits 24-26): OctoSPI supports up to 8 data lines
constexpr uint32_t CFG_DATA_MODE_MASK       =  (7LU << 24LU);  ///< Mask for data mode bits
constexpr uint32_t CFG_DATA_MODE_NONE       =  (0LU << 24LU);  ///< No data phase
constexpr uint32_t CFG_DATA_MODE_ONE_LINE   =  (1LU << 24LU);  ///< Data on single line (standard SPI)
constexpr uint32_t CFG_DATA_MODE_TWO_LINES  =  (2LU << 24LU);  ///< Data on two lines (dual SPI)
constexpr uint32_t CFG_DATA_MODE_FOUR_LINES =  (3LU << 24LU);  ///< Data on four lines (quad SPI)
constexpr uint32_t CFG_DATA_MODE_EIGHT_LINES=  (4LU << 24LU);  ///< Data on eight lines (octo SPI)

constexpr uint32_t CFG_DATA_DDR             =  (1LU << 27LU);  ///< Data phase in DDR mode (both clock edges)

constexpr uint32_t CFG_SIOO                 =  (1LU << 31LU);  ///< Send Instruction Only Once (performance optimization)
#endif // HAL_USE_QUADSPI
}
#endif //#if HAL_USE_WSPI_DEFAULT_CFG

/**
 * @class AP_HAL::WSPIDevice
 * @brief Abstract interface for Quad/Octo SPI device access
 * 
 * @details Extends the standard Device interface to support Wide SPI protocols with
 *          parallel data lanes for high-throughput communication. Wide SPI is primarily
 *          used for external flash memory devices but can support any peripheral requiring
 *          high bandwidth.
 *          
 *          **Transfer Modes Explained:**
 *          - Standard (1-bit): Traditional MOSI/MISO, ~6.25 MB/s at 50MHz
 *          - Dual (2-bit): 2× throughput, ~12.5 MB/s at 50MHz
 *          - Quad (4-bit): 4× throughput, ~25 MB/s at 50MHz
 *          - Octo (8-bit): 8× throughput, ~50 MB/s at 50MHz
 *          
 *          **Typical Transfer Sequence:**
 *          1. Acquire device semaphore with WITH_SEMAPHORE()
 *          2. Configure transfer using set_cmd_header()
 *          3. Execute transfer() with send/receive buffers
 *          4. Check is_busy() if needed for status polling
 *          5. Release semaphore (automatic with WITH_SEMAPHORE)
 *          
 *          **Common Use Cases:**
 *          - External flash read/write for logging (AP_Logger)
 *          - Parameter storage backup (AP_Param)
 *          - Terrain data caching (AP_Terrain)
 *          - Firmware update storage
 *          
 *          **Hardware Requirements:**
 *          - MCU with QUADSPI or OCTOSPI peripheral (STM32H7, STM32F7, etc.)
 *          - External flash with quad/octo support (W25Q, MT25Q series)
 *          - 6 pins for quad mode: CLK, CS, IO0-IO3
 *          - 10 pins for octo mode: CLK, CS, IO0-IO7
 *          
 *          **Performance Considerations:**
 *          - Use quad/octo mode for bulk transfers >256 bytes
 *          - Use standard mode for command-only operations
 *          - DDR mode doubles throughput but increases signal integrity requirements
 *          - SIOO flag reduces command overhead for consecutive reads
 * 
 * @note Hardware support limited to advanced MCUs with QSPI/OSPI peripherals
 * @note CommandHeader configuration is device-specific - consult flash datasheet
 * @warning Incorrect mode/timing configuration causes data corruption
 * @warning Some flash devices require specific dummy cycle counts per datasheet
 * 
 * @see Device.h for base device interface contract
 * @see AP_HAL::Device::CommandHeader for transfer configuration structure
 * @see AP_Logger for example usage in logging subsystem
 * 
 * Source: libraries/AP_HAL/WSPIDevice.h:135-154
 */
class WSPIDevice : public Device
{
public:

    /**
     * @brief Construct a WSPI device interface
     * 
     * @details Initializes base Device with BUS_TYPE_WSPI identifier.
     *          Actual hardware initialization occurs in platform-specific HAL implementation.
     */
    WSPIDevice() : Device(BUS_TYPE_WSPI) { }

    /**
     * @brief Execute a WSPI transfer with configured command header
     * 
     * @details Performs data transfer using the command header previously set via
     *          set_cmd_header(). The transfer executes according to the configured
     *          mode (standard/dual/quad/octo) for each phase.
     *          
     *          Transfer phases (as configured in CommandHeader):
     *          1. Command phase: Send command byte(s)
     *          2. Address phase: Send address bytes (if configured)
     *          3. Alternate byte phase: Send mode/dummy bytes (if configured)
     *          4. Data phase: Send/receive data buffer
     *          
     *          For read operations: send=nullptr or send_len=0, recv buffer receives data
     *          For write operations: send buffer contains data, recv=nullptr or recv_len=0
     * 
     * @param[in]  send      Pointer to data to send (can be nullptr for read-only)
     * @param[in]  send_len  Number of bytes to send (0 for read-only)
     * @param[out] recv      Pointer to receive buffer (can be nullptr for write-only)
     * @param[in]  recv_len  Number of bytes to receive (0 for write-only)
     * 
     * @return true if transfer completed successfully, false on error
     * 
     * @note Must call set_cmd_header() before transfer() to configure operation
     * @note Caller must hold device semaphore during transfer
     * @note For flash devices, typical read uses quad data mode, write uses standard mode initially
     * @warning Ensure recv buffer is large enough for recv_len bytes
     * @warning Do not modify buffers during transfer execution
     * 
     * @see AP_HAL::Device::transfer() for base interface
     * @see set_cmd_header() for configuring transfer parameters
     */
    virtual bool transfer(const uint8_t *send, uint32_t send_len,
                          uint8_t *recv, uint32_t recv_len) override = 0;

    /**
     * @brief Configure command header for upcoming WSPI transfer(s)
     * 
     * @details Sets the command structure that defines how the next transfer() call(s)
     *          will execute. The CommandHeader specifies:
     *          - cmd: Command byte (e.g., 0x03=read, 0x02=write for flash)
     *          - cfg: Configuration bitmask using WSPI::CFG_* constants
     *          - addr: Memory address or register address
     *          - alt: Alternate bytes (mode bits, dummy cycles)
     *          - dummy: Number of dummy clock cycles
     *          
     *          Configuration typically combines multiple CFG_* flags:
     *          - Command mode (NONE/ONE_LINE/FOUR_LINES)
     *          - Address mode and size
     *          - Alternate byte mode and size
     *          - Data mode (determines transfer throughput)
     *          - DDR flags (if supported)
     *          - SIOO optimization flag
     *          
     *          Example for quad SPI flash read:
     *          cmd = 0x6B (fast read quad)
     *          cfg = CFG_CMD_MODE_ONE_LINE | CFG_ADDR_MODE_ONE_LINE | 
     *                CFG_ADDR_SIZE_24 | CFG_DATA_MODE_FOUR_LINES
     *          addr = 0x000000 (flash address)
     *          alt = 0x00 (mode bits if required)
     *          dummy = 8 (dummy cycles per datasheet)
     * 
     * @param[in] cmd_hdr  Command header structure with transfer configuration
     * 
     * @note Command header persists until next set_cmd_header() call
     * @note Configuration is device-specific - consult peripheral datasheet
     * @note Some HAL implementations may support batching multiple transfers with same header
     * @warning Incorrect configuration causes device communication failure or data corruption
     * @warning Dummy cycle requirements are device-specific and frequency-dependent
     * 
     * @see AP_HAL::Device::CommandHeader for structure definition
     * @see WSPI namespace constants for cfg bitmask values
     */
    virtual void set_cmd_header(const CommandHeader& cmd_hdr) override = 0;

    /**
     * @brief Check if WSPI device is currently busy with a transfer
     * 
     * @details Queries hardware status to determine if a transfer is in progress.
     *          Useful for non-blocking operations or status polling during long
     *          flash erase operations.
     *          
     *          Typical usage: Poll during flash erase/program operations which can
     *          take milliseconds to complete.
     * 
     * @return true if device is busy with ongoing transfer, false if idle
     * 
     * @note Blocking transfers (default) will not return until complete
     * @note Most use cases don't require is_busy() check with semaphore-protected blocking transfers
     */
    virtual bool is_busy() = 0;

    /**
     * @brief Get the semaphore protecting this WSPI device
     * 
     * @details Returns the semaphore that must be acquired before accessing this device.
     *          Use WITH_SEMAPHORE() macro for automatic RAII-style locking.
     *          
     *          Usage pattern:
     *          ```cpp
     *          auto dev = wspi_mgr->get_device("flash");
     *          WITH_SEMAPHORE(*dev->get_semaphore());
     *          dev->set_cmd_header(cmd);
     *          dev->transfer(nullptr, 0, buffer, len);
     *          ```
     * 
     * @return Pointer to device semaphore, never nullptr
     * 
     * @note Semaphore prevents concurrent access from multiple threads/tasks
     * @note Always use WITH_SEMAPHORE() macro to ensure proper release on early return
     * @warning Never access device without holding its semaphore - causes race conditions
     * 
     * @see AP_HAL::Semaphore for semaphore interface
     * @see WITH_SEMAPHORE macro for usage pattern
     */
    virtual AP_HAL::Semaphore* get_semaphore() override = 0;

protected:
    uint32_t _trx_flags;  ///< Platform-specific transfer flags (timing, mode options)
};

/**
 * @class AP_HAL::WSPIDeviceManager
 * @brief Factory interface for creating and managing WSPI device instances
 * 
 * @details Provides platform-independent interface for accessing WSPI devices.
 *          The manager is responsible for:
 *          - Device initialization and configuration
 *          - Device handle creation and ownership management
 *          - Device enumeration and name resolution
 *          
 *          Device names are typically defined in board hwdef files:
 *          - "flash": External SPI flash chip
 *          - "fram": External FRAM storage
 *          - Custom names per board configuration
 *          
 *          Devices are typically accessed as singletons through AP::hal object:
 *          ```cpp
 *          auto &wspi_mgr = AP::hal.wspi;
 *          auto dev = wspi_mgr->get_device("flash");
 *          if (dev) {
 *              // Use device
 *          }
 *          ```
 *          
 *          Platform-specific implementation handles:
 *          - MCU QSPI/OSPI peripheral initialization
 *          - GPIO pin configuration per hwdef
 *          - Clock speed and timing configuration
 *          - Memory-mapped mode setup (if supported)
 * 
 * @note Device availability depends on board hardware definition
 * @note Device names are case-sensitive and board-specific
 * @warning Device pointers become invalid if manager is destroyed
 * 
 * @see WSPIDevice for device interface
 * @see AP_HAL_ChibiOS::WSPIDeviceManager for ChibiOS/STM32 implementation
 * @see hwdef files in AP_HAL_ChibiOS/hwdef/ for device name definitions
 * 
 * Source: libraries/AP_HAL/WSPIDevice.h:156-175
 */
class WSPIDeviceManager
{
public:
    /**
     * @brief Get a WSPI device by name
     * 
     * @details Creates or retrieves a device handle for the specified WSPI device.
     *          Device names are defined in board hwdef files and typically include:
     *          - "flash": Main external flash storage
     *          - "fram": External FRAM if present
     *          - Board-specific custom device names
     *          
     *          Returns nullptr if device name not found or initialization fails.
     *          
     *          Typical usage:
     *          ```cpp
     *          auto flash = hal.wspi->get_device("flash");
     *          if (!flash) {
     *              // Handle missing flash device
     *              return;
     *          }
     *          // Use flash device
     *          ```
     * 
     * @param[in] name  Device name string (null-terminated, case-sensitive)
     * 
     * @return OwnPtr to WSPIDevice if found and initialized, nullptr otherwise
     * 
     * @note Default implementation returns nullptr (no WSPI support)
     * @note Device initialization occurs on first access
     * @note Returned OwnPtr manages device lifetime automatically
     * @warning Verify return value is not nullptr before use
     * @warning Device names are board-specific - check hwdef files
     * 
     * @see OwnPtr for smart pointer semantics
     * @see get_device_name() to enumerate available devices
     */
    virtual OwnPtr<WSPIDevice> get_device(const char *name)
    {
        return nullptr;
    }

    /**
     * @brief Get the number of WSPI devices available on this platform
     * 
     * @details Returns the count of WSPI devices defined in board configuration.
     *          Useful for device enumeration and capability detection.
     *          
     *          Typical usage:
     *          ```cpp
     *          uint8_t count = hal.wspi->get_count();
     *          for (uint8_t i = 0; i < count; i++) {
     *              const char* name = hal.wspi->get_device_name(i);
     *              // Process device
     *          }
     *          ```
     * 
     * @return Number of WSPI devices available (0 if none or not supported)
     * 
     * @note Default implementation returns 0 (no WSPI support)
     * @note Count is determined by board hwdef file configuration
     */
    virtual uint8_t get_count() const
    {
        return 0;
    }

    /**
     * @brief Get device name by index
     * 
     * @details Returns the name string for the WSPI device at the specified index.
     *          Used for device enumeration and discovery.
     *          
     *          Example enumeration:
     *          ```cpp
     *          for (uint8_t i = 0; i < hal.wspi->get_count(); i++) {
     *              const char* name = hal.wspi->get_device_name(i);
     *              if (name) {
     *                  printf("WSPI device %d: %s\n", i, name);
     *              }
     *          }
     *          ```
     * 
     * @param[in] idx  Device index (0 to get_count()-1)
     * 
     * @return Device name string if index valid, nullptr otherwise
     * 
     * @note Default implementation returns nullptr (no WSPI support)
     * @note Returned string pointer lifetime matches WSPIDeviceManager lifetime
     * @warning Index out of range returns nullptr
     * @warning Do not free or modify returned string
     * 
     * @see get_count() to determine valid index range
     * @see get_device() to obtain device handle by name
     */
    virtual const char *get_device_name(uint8_t idx) const
    {
        return nullptr;
    }
};

}
