/**
 * @file AP_Logger_Flash_JEDEC.h
 * @brief AP_Logger backend for JEDEC-compliant SPI NOR flash memory chips
 * 
 * @details This file implements the AP_Logger_Block interface for JEDEC-standard
 *          SPI NOR flash chips commonly used for flight data logging. Supports
 *          popular flash families including:
 *          - Winbond W25Q series (W25Q16, W25Q32, W25Q64, W25Q128, etc.)
 *          - Macronix MX25 series (MX25L3206, MX25L6406, MX25L12845, etc.)
 *          - GigaDevice GD25 series (GD25Q16, GD25Q32, GD25Q64, etc.)
 *          - Micron/Numonyx N25Q series
 *          - ISSI IS25 series
 * 
 *          Implementation uses JEDEC standard commands (0x9F ID read, 0x06 write enable,
 *          0x20/0xD8 sector erase, 0x02 page program, etc.) and acquires the SPIDevice
 *          named "dataflash" from the HAL.
 * 
 * @note Flash chips must support standard JEDEC commands and provide valid manufacturer
 *       and device ID response to the Read JEDEC ID (0x9F) command.
 * 
 * @warning This driver assumes exclusive access to the SPI flash chip. Do not share
 *          the same flash device with other subsystems.
 * 
 * Source: libraries/AP_Logger/AP_Logger_Flash_JEDEC.h
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_Logger_Block.h"

#if HAL_LOGGING_FLASH_JEDEC_ENABLED

/**
 * @class AP_Logger_Flash_JEDEC
 * @brief Block-based dataflash logging backend for JEDEC-compliant SPI NOR flash chips
 * 
 * @details This class provides flight data logging to external SPI NOR flash memory
 *          devices that comply with JEDEC standards. The implementation translates
 *          AP_Logger block operations into appropriate JEDEC SPI commands and manages
 *          flash-specific timing constraints.
 * 
 *          **Architecture:**
 *          - Inherits from AP_Logger_Block for standardized logging interface
 *          - Acquires SPIDevice named "dataflash" from HAL during initialization
 *          - Implements page-based buffering (typically 256-byte pages)
 *          - Manages sector erase operations (4KB and 64KB sectors)
 *          - Handles asynchronous chip erase with progress monitoring
 * 
 *          **Flash Memory Organization:**
 *          - Page size: 256 bytes (standard for JEDEC NOR flash)
 *          - Sector size: 4KB (minimum erase unit)
 *          - Block size: 64KB (optional larger erase unit)
 *          - Total capacity: Detected via JEDEC ID and configuration registers
 * 
 *          **Address Modes:**
 *          - 3-byte addressing: For chips ≤16MB (128Mbit)
 *          - 4-byte addressing: Automatically enabled for chips >16MB
 *          - Address mode detection based on chip capacity
 * 
 *          **Performance Characteristics:**
 *          - Page program time: 0.5-5ms per 256-byte page (typical 1-3ms)
 *          - 4KB sector erase: 20-400ms (typical 50-100ms)
 *          - 64KB block erase: 200-1000ms (typical 500ms)
 *          - Full chip erase: Multiple seconds (depends on capacity)
 *          - Read bandwidth: Limited by SPI clock (typically 20-50 MHz)
 *          - SPI modes supported: Mode 0 and Mode 3 (CPOL=0/1, CPHA=0/1)
 * 
 *          **JEDEC Command Set Used:**
 *          - 0x9F: Read JEDEC ID (manufacturer + device identification)
 *          - 0x06: Write Enable (sets WEL bit in status register)
 *          - 0x04: Write Disable (clears WEL bit)
 *          - 0x05: Read Status Register (check WIP, WEL bits)
 *          - 0x02: Page Program (write up to 256 bytes)
 *          - 0x03: Read Data (standard read)
 *          - 0x20: 4KB Sector Erase
 *          - 0xD8: 64KB Block Erase
 *          - 0xC7/0x60: Chip Erase (entire flash)
 *          - 0xB7: Enter 4-Byte Address Mode (for >16MB chips)
 * 
 *          **Write Protection Management:**
 *          - Write Enable Latch (WEL) must be set before program/erase
 *          - WEL automatically clears after operation completion
 *          - Status register polling used to verify WEL state
 *          - Hardware write-protect pin handling (if board-specific)
 * 
 *          **Thread Safety:**
 *          - SPI bus access protected via device semaphore (dev_sem)
 *          - Blocking operations hold semaphore for entire duration
 *          - WaitReady() polls status without releasing semaphore
 *          - Caller must not access flash from multiple threads simultaneously
 * 
 *          **Failure Detection:**
 *          - flash_died flag set if chip becomes unresponsive
 *          - CardInserted() returns false if flash has failed
 *          - Automatic detection of SPI communication failures
 *          - No recovery attempted once flash_died is set
 * 
 * @note Flash chip must be connected to SPI bus with chip-select defined in
 *       board hardware definition (hwdef) as "dataflash" device.
 * 
 * @warning Erase operations take significant time (20-400ms for 4KB sector).
 *          Ensure erase calls are made from appropriate thread context to avoid
 *          blocking critical flight control tasks.
 * 
 * @warning Flash memory has limited write endurance (typically 100K erase cycles
 *          per sector). AP_Logger_Block handles wear leveling, but avoid excessive
 *          erase operations during testing.
 * 
 * @see AP_Logger_Block for base logging interface
 * @see AP_HAL::SPIDevice for SPI communication abstraction
 */
class AP_Logger_Flash_JEDEC : public AP_Logger_Block {
public:
    /**
     * @brief Construct JEDEC flash logger backend
     * 
     * @param[in] front Reference to main AP_Logger instance
     * @param[in] writer Pointer to DFLogStart message writer for log headers
     * 
     * @note Constructor only initializes base class. Actual flash hardware
     *       initialization occurs in Init() method.
     */
    AP_Logger_Flash_JEDEC(AP_Logger &front, LoggerMessageWriter_DFLogStart *writer) :
        AP_Logger_Block(front, writer) {}
    
    /**
     * @brief Probe for JEDEC flash chip and create backend instance
     * 
     * @details Factory method that attempts to create an AP_Logger_Flash_JEDEC
     *          instance. The actual chip detection occurs during Init() call.
     *          This probe method always succeeds if memory allocation succeeds;
     *          flash chip presence is verified during initialization.
     * 
     * @param[in] front Reference to main AP_Logger instance
     * @param[in] ls Pointer to DFLogStart message writer
     * 
     * @return Pointer to new AP_Logger_Flash_JEDEC backend, or nullptr if allocation fails
     * 
     * @note Uses NEW_NOTHROW to handle allocation failure gracefully
     * @see Init() for actual hardware detection and configuration
     */
    static AP_Logger_Backend  *probe(AP_Logger &front,
                                     LoggerMessageWriter_DFLogStart *ls) {
        return NEW_NOTHROW AP_Logger_Flash_JEDEC(front, ls);
    }
    
    /**
     * @brief Initialize JEDEC flash chip and detect configuration
     * 
     * @details Performs complete flash chip initialization sequence:
     *          1. Acquire SPIDevice named "dataflash" from HAL
     *          2. Issue Read JEDEC ID command (0x9F) to identify chip
     *          3. Parse manufacturer ID and device ID to determine capacity
     *          4. Enable 4-byte address mode if chip capacity >16MB
     *          5. Calculate number of pages based on chip size
     *          6. Verify chip is responsive via status register reads
     * 
     *          If any step fails, sets flash_died flag and CardInserted() returns false.
     * 
     * @note Called once during AP_Logger initialization
     * @note Failure during Init() is not fatal to flight controller - logging is disabled
     * 
     * @warning Flash chip must be defined in board hwdef as SPIDevice "dataflash"
     * @see getSectorCount() for capacity detection logic
     */
    void              Init(void) override;
    
    /**
     * @brief Check if flash chip is present and operational
     * 
     * @return true if flash chip initialized successfully and has not failed, false otherwise
     * 
     * @note Returns false if flash_died flag is set (communication failure detected)
     * @note Returns false if df_NumPages is 0 (chip not detected or has zero capacity)
     * @note This is called by AP_Logger to determine if logging backend is available
     */
    bool              CardInserted() const override { return !flash_died && df_NumPages > 0; }

private:
    /**
     * @brief Write buffer contents to flash memory page
     * 
     * @details Translates AP_Logger page write operation into JEDEC SPI flash
     *          Page Program command sequence:
     *          1. Assert Write Enable (0x06 command, sets WEL bit)
     *          2. Send Page Program command (0x02) with page address
     *          3. Transfer 256 bytes of data from internal buffer
     *          4. Wait for program operation to complete (poll WIP bit)
     * 
     *          Page address is converted to byte address (PageAdr * 256) and
     *          sent as either 3-byte or 4-byte address depending on chip size.
     * 
     * @param[in] PageAdr Page number to write (0-based index into flash page array)
     * 
     * @note Typical program time: 1-5ms per page (256 bytes)
     * @note Page must be within previously erased sector (0xFF state required)
     * @note Write operation will fail if WEL bit is not set (WriteEnable() handles this)
     * @note Blocks until program operation completes
     * 
     * @warning Programming a page that has not been erased will result in data corruption
     *          due to NAND flash write characteristics (can only change 1→0, not 0→1)
     * @warning This is a blocking operation taking 1-5ms - should not be called from
     *          time-critical threads
     * 
     * @see WriteEnable() for WEL bit management
     * @see WaitReady() for completion polling
     */
    void              BufferToPage(uint32_t PageAdr) override;
    
    /**
     * @brief Read flash memory page into internal buffer
     * 
     * @details Translates AP_Logger page read operation into JEDEC SPI flash
     *          Read Data command:
     *          1. Send Read Data command (0x03) with page address
     *          2. Transfer 256 bytes from flash into internal buffer
     *          3. Set read_cache_valid flag to indicate buffer contains valid data
     * 
     *          Page address is converted to byte address (PageAdr * 256) and
     *          sent as either 3-byte or 4-byte address depending on chip size.
     * 
     * @param[in] PageAdr Page number to read (0-based index into flash page array)
     * 
     * @note Read operations are relatively fast (limited by SPI clock frequency)
     * @note Typical read bandwidth: 5-20 MB/s depending on SPI clock configuration
     * @note Read does not require prior Write Enable - can read anytime chip is ready
     * @note Does not block waiting for chip - assumes chip is ready (not programming/erasing)
     * 
     * @see send_command_addr() for address transmission
     */
    void              PageToBuffer(uint32_t PageAdr) override;
    /**
     * @brief Erase 64KB block (sector) of flash memory
     * 
     * @details Erases a 64KB block using JEDEC Block Erase command (0xD8):
     *          1. Assert Write Enable (0x06 command, sets WEL bit)
     *          2. Send Block Erase command (0xD8) with sector address
     *          3. Wait for erase operation to complete (poll WIP bit in status register)
     * 
     *          Sector address is aligned to 64KB boundaries. All bytes within the
     *          64KB block are set to 0xFF (erased state).
     * 
     * @param[in] SectorAdr Sector address (byte address, must be 64KB-aligned)
     * 
     * @note Erase time: 200-1000ms typical (500ms average) for 64KB block
     * @note All 64KB (65536 bytes) within the block are erased to 0xFF
     * @note Sector address should be aligned to 64KB boundary (address & 0xFFFF0000)
     * 
     * @warning This is a BLOCKING operation taking 200-1000ms
     * @warning Do NOT call from time-critical flight control threads
     * @warning Erasing active log data will cause data loss - AP_Logger_Block manages this
     * @warning Flash has limited erase endurance (~100K cycles per block)
     * 
     * @see Sector4kErase() for smaller 4KB erase operations
     * @see WaitReady() for completion polling implementation
     */
    void              SectorErase(uint32_t SectorAdr) override;
    
    /**
     * @brief Erase 4KB sector of flash memory
     * 
     * @details Erases a 4KB sector using JEDEC Sector Erase command (0x20):
     *          1. Assert Write Enable (0x06 command, sets WEL bit)
     *          2. Send Sector Erase command (0x20) with sector address
     *          3. Wait for erase operation to complete (poll WIP bit in status register)
     * 
     *          Sector address is aligned to 4KB boundaries. All bytes within the
     *          4KB sector are set to 0xFF (erased state). This is the minimum
     *          erase unit for JEDEC flash chips.
     * 
     * @param[in] SectorAdr Sector address (byte address, must be 4KB-aligned)
     * 
     * @note Erase time: 20-400ms typical (50-100ms average) for 4KB sector
     * @note All 4KB (4096 bytes) within the sector are erased to 0xFF
     * @note Sector address should be aligned to 4KB boundary (address & 0xFFFFF000)
     * @note Faster than 64KB block erase when only small region needs erasing
     * 
     * @warning This is a BLOCKING operation taking 20-400ms
     * @warning Do NOT call from time-critical flight control threads
     * @warning Erasing active log data will cause data loss - AP_Logger_Block manages this
     * @warning Flash has limited erase endurance (~100K cycles per sector)
     * 
     * @see SectorErase() for larger 64KB block erase operations
     * @see WaitReady() for completion polling implementation
     */
    void              Sector4kErase(uint32_t SectorAdr) override;
    
    /**
     * @brief Initiate asynchronous full chip erase operation
     * 
     * @details Starts a full chip erase using JEDEC Chip Erase command (0xC7 or 0x60):
     *          1. Assert Write Enable (0x06 command, sets WEL bit)
     *          2. Send Chip Erase command (0xC7)
     *          3. Store start timestamp for progress monitoring
     *          4. Return immediately without waiting for completion
     * 
     *          This operation erases ALL flash memory contents to 0xFF state.
     *          Unlike sector erase, this method returns immediately and does not
     *          block waiting for completion. Use InErase() to poll progress.
     * 
     * @note Chip erase time: Multiple seconds (5-60s depending on capacity)
     *       - 16Mbit (2MB): ~5-10 seconds
     *       - 64Mbit (8MB): ~10-20 seconds
     *       - 128Mbit (16MB): ~20-40 seconds
     *       - 256Mbit (32MB): ~40-60 seconds
     * @note Erase time stored in erase_start_ms for progress calculation
     * @note Non-blocking: Returns immediately, use InErase() to check completion
     * 
     * @warning Erases ENTIRE flash chip - all log data will be permanently lost
     * @warning Chip is unusable until erase completes (seconds to minutes)
     * @warning Do NOT power cycle during chip erase - may corrupt flash
     * 
     * @see InErase() for checking erase completion status
     */
    void              StartErase() override;
    
    /**
     * @brief Check if asynchronous chip erase operation is still in progress
     * 
     * @details Polls flash chip status register to determine if chip erase
     *          initiated by StartErase() has completed:
     *          1. Read Status Register (0x05 command)
     *          2. Check WIP (Write In Progress) bit (bit 0)
     *          3. Return true if WIP=1 (erase in progress), false if WIP=0 (complete)
     * 
     *          This method is called periodically to monitor long-running chip
     *          erase operations without blocking.
     * 
     * @return true if chip erase is still in progress, false if completed
     * 
     * @note Typical polling interval: 100-500ms during erase
     * @note Can be called safely during erase - does not interfere with operation
     * @note Returns false if no erase operation was started
     * 
     * @see StartErase() for initiating chip erase
     * @see Busy() for low-level status register polling
     */
    bool              InErase() override;
    /**
     * @brief Send SPI command with address bytes
     * 
     * @details Transmits a JEDEC command followed by address bytes in either
     *          3-byte or 4-byte format depending on use_32bit_address flag:
     *          - 3-byte mode: cmd + addr[23:16] + addr[15:8] + addr[7:0]
     *          - 4-byte mode: cmd + addr[31:24] + addr[23:16] + addr[15:8] + addr[7:0]
     * 
     *          Address bytes are transmitted MSB-first (big-endian) as required
     *          by JEDEC specification.
     * 
     * @param[in] cmd JEDEC command byte (e.g., 0x02 for Page Program, 0x03 for Read)
     * @param[in] address Target byte address in flash memory (0 to flash_size-1)
     * 
     * @note Automatically selects 3-byte or 4-byte addressing based on chip configuration
     * @note Address is NOT validated - caller must ensure address is within chip capacity
     * @note CS (chip select) is asserted by SPIDevice for duration of transaction
     * 
     * @see Enter4ByteAddressMode() for 4-byte address mode configuration
     */
    void              send_command_addr(uint8_t cmd, uint32_t address);
    
    /**
     * @brief Block until flash chip completes current operation
     * 
     * @details Polls status register WIP (Write In Progress) bit until it clears,
     *          indicating the flash chip has completed the current program or erase
     *          operation. Implements busy-wait polling with small delays to avoid
     *          excessive SPI traffic.
     * 
     *          Polling sequence:
     *          1. Read Status Register (0x05)
     *          2. Check bit 0 (WIP - Write In Progress)
     *          3. If WIP=1, delay briefly and repeat
     *          4. If WIP=0, return (operation complete)
     * 
     * @note Typical wait times:
     *       - Page program: 1-10ms
     *       - 4KB sector erase: 20-400ms
     *       - 64KB block erase: 200-1000ms
     *       - Chip erase: Multiple seconds (not typically used with WaitReady)
     * 
     * @warning This is a BLOCKING busy-wait loop
     * @warning Wait time varies with operation type and flash chip manufacturer
     * @warning Do NOT call WaitReady() after chip erase - use InErase() instead
     * 
     * @see Busy() for non-blocking status check
     * @see ReadStatusReg() for raw status register access
     */
    void              WaitReady();
    
    /**
     * @brief Check if flash chip is currently busy with an operation
     * 
     * @details Reads status register and checks WIP (Write In Progress) bit
     *          without blocking. Returns immediately with current busy state.
     * 
     * @return true if WIP bit is set (operation in progress), false if idle
     * 
     * @note Non-blocking single status register read
     * @note Called by InErase() to monitor chip erase progress
     * @note Can be called at any time without side effects
     * 
     * @see ReadStatusReg() for raw status register value
     * @see WaitReady() for blocking wait until idle
     */
    bool              Busy();
    
    /**
     * @brief Read flash chip status register
     * 
     * @details Issues Read Status Register command (0x05) and returns the
     *          status byte. Status register bits (JEDEC standard):
     *          - Bit 0: WIP (Write In Progress) - 1=busy, 0=ready
     *          - Bit 1: WEL (Write Enable Latch) - 1=write enabled, 0=write disabled
     *          - Bit 2-4: BP0-BP2 (Block Protect bits) - write protection configuration
     *          - Bit 5-6: Manufacturer specific or reserved
     *          - Bit 7: SRWD (Status Register Write Disable)
     * 
     * @return Status register byte value (8 bits)
     * 
     * @note Most commonly checked bits are WIP (bit 0) and WEL (bit 1)
     * @note Block protect bits vary by manufacturer - consult specific datasheet
     * @note Some chips have extended status registers (0x35, 0x15 commands)
     * 
     * @see Busy() for convenient WIP bit checking
     * @see WriteEnable() which checks WEL bit
     */
    uint8_t           ReadStatusReg();
    
    /**
     * @brief Enable 4-byte address mode for large flash chips
     * 
     * @details For flash chips with capacity >16MB (128Mbit), JEDEC standard
     *          requires 4-byte addressing to access full address space. This
     *          method sends the Enter 4-Byte Address Mode command (0xB7).
     * 
     *          After this command:
     *          - All address-based commands require 4 address bytes
     *          - use_32bit_address flag is set to true
     *          - send_command_addr() automatically uses 4-byte format
     * 
     *          Standard 3-byte addressing limits:
     *          - Maximum address: 0xFFFFFF (16MB)
     *          - Chips ≤16MB use 3-byte mode
     *          - Chips >16MB require 4-byte mode for full capacity
     * 
     * @note Called automatically during Init() for chips >16MB
     * @note Mode is volatile - resets on power cycle (re-enabled on each init)
     * @note Some chips support non-volatile 4-byte mode via configuration register
     * @note Not all JEDEC chips support 0xB7 command - older chips may use different methods
     * 
     * @warning Once enabled, all subsequent commands must use 4-byte addresses
     * @warning Sending 3-byte addresses in 4-byte mode will access wrong addresses
     * 
     * @see getSectorCount() which detects chip size and enables 4-byte mode if needed
     */
    void              Enter4ByteAddressMode(void);

    /**
     * @brief Enable write and erase operations by setting WEL bit
     * 
     * @details JEDEC flash chips require the Write Enable Latch (WEL) bit to be
     *          set before any program or erase operation. This method sends the
     *          Write Enable command (0x06) which sets WEL in status register.
     * 
     *          WEL behavior:
     *          - Set by Write Enable command (0x06)
     *          - Automatically cleared after Page Program completes
     *          - Automatically cleared after Erase completes
     *          - Must be re-asserted for each program/erase operation
     * 
     *          Sequence for program/erase:
     *          1. WriteEnable() - sets WEL bit
     *          2. Issue program/erase command
     *          3. WEL automatically clears on completion
     * 
     * @note WEL is a safety mechanism preventing accidental writes
     * @note Write Enable must be issued immediately before program/erase command
     * @note WEL can be cleared manually with Write Disable command (0x04) if needed
     * 
     * @warning Program/erase commands will be IGNORED if WEL is not set
     * @warning WEL automatically clears after operation - must re-enable for next write
     * 
     * @see BufferToPage() which calls WriteEnable() before Page Program
     * @see SectorErase(), Sector4kErase() which call WriteEnable() before erase
     */
    void              WriteEnable();
    
    /**
     * @brief Detect flash chip capacity and calculate sector count
     * 
     * @details Reads JEDEC ID (command 0x9F) to identify flash chip and determine
     *          its capacity. JEDEC ID returns:
     *          - Byte 0: Manufacturer ID (e.g., 0xEF=Winbond, 0xC2=Macronix, 0x20=Micron)
     *          - Byte 1: Memory type (e.g., 0x40=SPI NOR flash)
     *          - Byte 2: Capacity code (e.g., 0x15=16Mbit, 0x16=32Mbit, 0x17=64Mbit, 0x18=128Mbit)
     * 
     *          Capacity decoding (byte 2):
     *          - 0x14: 8Mbit (1MB)
     *          - 0x15: 16Mbit (2MB)
     *          - 0x16: 32Mbit (4MB)
     *          - 0x17: 64Mbit (8MB)
     *          - 0x18: 128Mbit (16MB)
     *          - 0x19: 256Mbit (32MB)
     *          - 0x1A: 512Mbit (64MB)
     * 
     *          Calculates df_NumPages based on detected capacity (pages = capacity / 256).
     *          For chips >16MB, automatically enables 4-byte address mode.
     * 
     * @return true if valid JEDEC ID detected and capacity determined, false otherwise
     * 
     * @note Sets df_NumPages member variable with calculated page count
     * @note Sets use_32bit_address flag for chips >16MB
     * @note Calls Enter4ByteAddressMode() for large chips
     * @note Returns false if JEDEC ID is invalid (0xFF or 0x00)
     * 
     * @warning If this returns false, flash chip is not usable (sets flash_died)
     * 
     * @see Enter4ByteAddressMode() for 4-byte address enabling
     */
    bool              getSectorCount(void);

    /**
     * @brief SPI device handle for flash chip communication
     * 
     * @details Owned pointer to SPIDevice acquired by name "dataflash" from HAL.
     *          Provides exclusive access to SPI bus transactions with the flash chip.
     *          Automatically manages chip select (CS) assertion/deassertion.
     * 
     * @note Initialized in Init() via AP_HAL::get_HAL().spi->get_device("dataflash")
     * @note nullptr if "dataflash" device not defined in board hardware definition
     * @see AP_HAL::SPIDevice for SPI communication interface
     */
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev;
    
    /**
     * @brief Semaphore for thread-safe SPI device access
     * 
     * @details Pointer to semaphore protecting concurrent access to flash chip
     *          via SPI bus. Must be acquired before any SPI transaction.
     *          Prevents multiple threads from simultaneously accessing flash.
     * 
     * @note Obtained from dev->get_semaphore() during Init()
     * @note All public methods acquire this semaphore via WITH_SEMAPHORE macro
     * @note Blocking operations hold semaphore for entire duration
     * 
     * @warning Do not hold semaphore across long delays or erase operations
     *          if other subsystems need SPI access
     */
    AP_HAL::Semaphore *dev_sem;

    /**
     * @brief Flag indicating flash chip has failed and is unusable
     * 
     * @details Set to true if flash chip becomes unresponsive or communication
     *          fails during initialization or operation. Once set, this flag
     *          is never cleared (flash considered permanently failed).
     * 
     *          Reasons for flash_died=true:
     *          - JEDEC ID read failure (invalid or no response)
     *          - SPI communication timeout or error
     *          - Flash chip not defined in hardware definition
     *          - Status register reads returning invalid data
     * 
     * @note CardInserted() returns false when flash_died is true
     * @note No recovery attempted once flash fails
     * @see Init() for detection logic
     */
    bool flash_died;
    
    /**
     * @brief Timestamp (milliseconds) when chip erase operation started
     * 
     * @details Stores AP_HAL::millis() timestamp when StartErase() is called.
     *          Used to track erase duration and timeout detection. Set to 0
     *          when no erase operation is in progress.
     * 
     * @note Used by InErase() to calculate elapsed time
     * @note Typical chip erase duration: 5-60 seconds depending on capacity
     * @see StartErase() for initialization
     * @see InErase() for usage
     */
    uint32_t erase_start_ms;
    
    /**
     * @brief JEDEC command used for current erase operation
     * 
     * @details Stores the erase command byte sent to flash chip:
     *          - 0xC7: Chip Erase (erase entire flash)
     *          - 0xD8: 64KB Block Erase
     *          - 0x20: 4KB Sector Erase
     * 
     *          Used for debugging and operation tracking.
     * 
     * @note Set by StartErase(), SectorErase(), Sector4kErase()
     */
    uint8_t erase_cmd;
    
    /**
     * @brief Flag indicating whether 4-byte address mode is enabled
     * 
     * @details JEDEC flash chips with capacity >16MB (128Mbit) require 4-byte
     *          addressing to access full address space. This flag determines
     *          address byte count in SPI commands:
     *          - false: 3-byte addresses (24-bit, max 16MB)
     *          - true: 4-byte addresses (32-bit, max 4GB)
     * 
     * @note Automatically set by getSectorCount() based on detected chip size
     * @note Affects send_command_addr() address transmission
     * @note Mode must be re-enabled after power cycle (volatile setting)
     * 
     * @see Enter4ByteAddressMode() for enabling 4-byte mode
     * @see getSectorCount() for automatic detection
     */
    bool use_32bit_address;
    
    /**
     * @brief Flag indicating internal buffer contains valid page data
     * 
     * @details Set to true by PageToBuffer() after successfully reading a page
     *          from flash into internal buffer. Cleared when buffer is modified
     *          or invalidated. Used by AP_Logger_Block to optimize read operations
     *          (avoid re-reading same page multiple times).
     * 
     * @note Optimization flag for caching - not critical for functionality
     * @note Set true after PageToBuffer() completes
     * @note Should be cleared when buffer is written or log position changes
     */
    bool read_cache_valid;
};

#endif // HAL_LOGGING_FLASH_JEDEC_ENABLED
