/**
 * @file AP_Logger_W25NXX.h
 * @brief AP_Logger backend for Winbond W25N series SPI NAND flash memory chips
 * 
 * @details This file implements AP_Logger_Block for Winbond W25N series NAND flash devices
 *          (W25N01G, W25N02G, etc.). These chips differ significantly from traditional NOR flash:
 * 
 *          Key NAND Flash Characteristics:
 *          - Page size: 2048 bytes (vs 256 bytes typical in NOR flash)
 *          - Block organization: 64 pages per block (128KB per block)
 *          - Built-in ECC (Error Correction Code) for data integrity
 *          - Internal buffer architecture: data staged through chip buffers before SPI transfer
 *          - Block-level erase only (cannot erase individual pages)
 *          - Bad block management required (factory-marked and runtime-detected bad blocks)
 * 
 *          Write Operation Sequence:
 *          1. Load data into chip's internal buffer (via PROGRAM LOAD command)
 *          2. Execute page program (transfers buffer to NAND array)
 *          3. Poll status register until program complete (~700μs typical)
 *          4. Verify ECC status for write success
 * 
 *          Read Operation Sequence:
 *          1. Issue PAGE READ command with page address
 *          2. Wait for data transfer to chip's internal buffer
 *          3. Read data from buffer via SPI
 * 
 *          Performance Characteristics:
 *          - Page program time: ~700μs typical
 *          - Block erase time: ~3ms typical
 *          - Page read to buffer: ~60μs
 *          - Endurance: 100,000 program/erase cycles per block
 * 
 *          ECC Features:
 *          - 1-bit ECC per 512 bytes (corrects 1-bit errors, detects 2-bit errors)
 *          - Automatic ECC during page program and read
 *          - Status register reports ECC results: pass, corrected, or uncorrectable error
 * 
 * @note Thread safety: All SPI operations must be protected by dev_sem semaphore
 * @warning Write ordering critical: Must load buffer before programming page
 * @warning ECC errors: Distinguish between correctable (warning) and uncorrectable (data loss)
 * 
 * Source: libraries/AP_Logger/AP_Logger_W25NXX.h
 */
#pragma once

#include "AP_Logger_config.h"

#if HAL_LOGGING_FLASH_W25NXX_ENABLED

#include <AP_HAL/AP_HAL.h>

#include "AP_Logger_Block.h"

/**
 * @class AP_Logger_W25NXX
 * @brief Dataflash logging backend for Winbond W25N series SPI NAND flash chips
 * 
 * @details Implements block-based logging interface for NAND flash devices with fundamentally
 *          different characteristics from NOR flash used in other AP_Logger backends:
 * 
 *          NAND vs NOR Flash Differences:
 *          - Page size: 2048 bytes (NAND) vs 256 bytes (NOR)
 *          - Erase granularity: 64-page blocks (128KB) vs 4KB or 64KB sectors
 *          - Built-in ECC: Hardware ECC in NAND vs none in NOR
 *          - Bad blocks: NAND has factory-marked and runtime bad blocks
 *          - Write latency: Buffer-staged writes add ~700μs program time
 *          - Architecture: Separate read/program buffers vs direct array access
 * 
 *          Register Interface (W25N-specific):
 *          - Feature Register (0xB0): ECC enable, buffer read mode, reset
 *          - Status Register (0xC0): Busy, write enable, ECC status (2 bits)
 *          - Protection Register (0xA0): Block protection configuration
 * 
 *          ECC Status Decoding (Status Register bits 4-5):
 *          - 00: No ECC errors detected
 *          - 01: 1-bit error corrected successfully (data valid, but block wearing)
 *          - 10: Multiple bit errors detected (data may be corrupted)
 *          - 11: ECC error (data uncorrectable, page should be marked bad)
 * 
 *          Bad Block Management:
 *          - Factory bad blocks: Marked in spare area by manufacturer
 *          - Runtime bad blocks: Detected via repeated ECC failures
 *          - Strategy: Skip bad blocks during address mapping
 *          - No wear leveling: Relies on sequential write pattern of logger
 * 
 *          Buffer-Read Modes:
 *          - Standard mode: PAGE READ loads buffer, then read via SPI
 *          - Continuous read: Sequential page reads optimized
 *          - Random read: Read any location within current buffer
 * 
 *          Thread Safety:
 *          - All methods assume caller holds dev_sem semaphore
 *          - WaitReady() polls device status - may block for ms-level times
 *          - Do not call from interrupt context
 * 
 *          Performance Optimization:
 *          - read_cache_valid: Avoids redundant PAGE READ for same page
 *          - Sequential writes: Optimized for logging use case
 *          - Block erase: Deferred until necessary (InErase() state machine)
 * 
 * @note Inherits from AP_Logger_Block which provides high-level block device abstraction
 * @warning Device failure (flash_died=true) is permanent - requires power cycle
 * @warning ECC correction indicates block wear - monitor for preventive replacement
 * 
 * Source: libraries/AP_Logger/AP_Logger_W25NXX.h
 */
class AP_Logger_W25NXX : public AP_Logger_Block {
public:
    /**
     * @brief Constructor for W25NXX logger backend
     * 
     * @param[in] front Reference to main AP_Logger instance
     * @param[in] writer Pointer to log message writer for start messages
     */
    AP_Logger_W25NXX(AP_Logger &front, LoggerMessageWriter_DFLogStart *writer) :
        AP_Logger_Block(front, writer) {}
    
    /**
     * @brief Factory method to probe and instantiate W25NXX logger backend
     * 
     * @details Allocates AP_Logger_W25NXX instance using NEW_NOTHROW for safe
     *          memory allocation in resource-constrained environment
     * 
     * @param[in] front Reference to main AP_Logger instance
     * @param[in] ls Pointer to log start message writer
     * 
     * @return Pointer to new AP_Logger_W25NXX backend, or nullptr if allocation fails
     * 
     * @note Called during logger initialization to detect and configure backend
     */
    static AP_Logger_Backend  *probe(AP_Logger &front,
                                     LoggerMessageWriter_DFLogStart *ls) {
        return NEW_NOTHROW AP_Logger_W25NXX(front, ls);
    }
    
    /**
     * @brief Initialize W25NXX flash device and configure for logging
     * 
     * @details Initialization sequence:
     *          1. Acquire SPI device via HAL
     *          2. Reset device to known state
     *          3. Configure feature register (enable ECC, set buffer mode)
     *          4. Read device ID and determine capacity
     *          5. Calculate available pages (accounting for bad blocks)
     *          6. Verify device is operational (sets flash_died on failure)
     * 
     * @note Must be called before any logging operations
     * @warning If initialization fails, CardInserted() will return false
     */
    void              Init(void) override;
    
    /**
     * @brief Check if flash memory is present and operational
     * 
     * @details Returns true if:
     *          - Device initialized successfully (!flash_died)
     *          - Device has non-zero capacity (df_NumPages > 0)
     * 
     * @return true if flash is available for logging, false if failed or absent
     * 
     * @note Called by AP_Logger to determine if backend is usable
     */
    bool              CardInserted() const override { return !flash_died && df_NumPages > 0; }

private:
    /**
     * @brief Write data from RAM buffer to NAND flash page
     * 
     * @details Two-stage write process for NAND flash:
     *          1. Load data into chip's internal buffer (PROGRAM LOAD command)
     *          2. Execute page program (PROGRAM EXECUTE command with page address)
     *          3. Poll status register until program completes (~700μs)
     *          4. Check ECC status bits for write verification
     * 
     * @param[in] PageAdr Physical page address to program (0 to df_NumPages-1)
     * 
     * @note Assumes buffer has been populated via AP_Logger_Block interface
     * @warning Must call WriteEnable() before programming
     * @warning Page must be in erased state (0xFF) before programming
     * @warning If ECC reports error, page may be bad and should be skipped
     * 
     * Source: libraries/AP_Logger/AP_Logger_W25NXX.cpp (implementation)
     */
    void              BufferToPage(uint32_t PageAdr) override;
    
    /**
     * @brief Read NAND flash page into RAM buffer
     * 
     * @details Two-stage read process:
     *          1. Issue PAGE READ to transfer page to chip's internal buffer (~60μs)
     *          2. Read data from buffer to RAM via SPI READ command
     *          3. Check ECC status to verify data integrity
     * 
     * @param[in] PageAdr Physical page address to read (0 to df_NumPages-1)
     * 
     * @note Uses read_cache_valid to avoid redundant reads of same page
     * @warning If ECC status indicates uncorrectable error, data is invalid
     * @warning ECC correction (1-bit) is normal but indicates block wear
     * 
     * Source: libraries/AP_Logger/AP_Logger_W25NXX.cpp (implementation)
     */
    void              PageToBuffer(uint32_t PageAdr) override;
    
    /**
     * @brief Erase a flash sector (block)
     * 
     * @details W25N NAND flash uses block erase (not sector erase):
     *          - Erase unit: 64 pages (128KB per block)
     *          - Erase time: ~3ms typical
     *          - Address: Block address (PageAdr / 64)
     *          - Result: All pages in block set to 0xFF (erased state)
     * 
     * @param[in] SectorAdr Block address to erase (actually block, not sector)
     * 
     * @note Despite name, this erases a 128KB block (NAND terminology)
     * @warning Must call WriteEnable() before erase
     * @warning Blocks have limited erase cycles (100,000 typical)
     * @warning If erase fails repeatedly, block should be marked bad
     * 
     * Source: libraries/AP_Logger/AP_Logger_W25NXX.cpp (implementation)
     */
    void              SectorErase(uint32_t SectorAdr) override;
    
    /**
     * @brief Erase a 4KB sector (not supported on W25N NAND flash)
     * 
     * @details W25N NAND devices do not support 4KB erase - minimum erase
     *          is full 128KB block. This method likely delegates to SectorErase()
     *          or is unused for NAND flash.
     * 
     * @param[in] SectorAdr Sector address (parameter ignored or converted to block)
     * 
     * @note NAND flash has block-level erase only (64 pages = 128KB)
     * @warning Using this may erase more data than expected (full 128KB block)
     * 
     * Source: libraries/AP_Logger/AP_Logger_W25NXX.cpp (implementation)
     */
    void              Sector4kErase(uint32_t SectorAdr) override;
    
    /**
     * @brief Begin asynchronous mass erase operation
     * 
     * @details Starts erasing flash chip block-by-block for chip erase or
     *          log clearing operation. Uses erase_block to track progress.
     * 
     * @note Erase continues across multiple calls to InErase()
     * @note Use InErase() to check completion status
     * 
     * Source: libraries/AP_Logger/AP_Logger_W25NXX.cpp (implementation)
     */
    void              StartErase() override;
    
    /**
     * @brief Check if asynchronous erase operation is in progress
     * 
     * @details Performs incremental block erase:
     *          - Erases one block per call (non-blocking state machine)
     *          - Tracks progress in erase_block counter
     *          - Returns false when all blocks erased
     * 
     * @return true if erase still in progress, false when complete
     * 
     * @note Called repeatedly by AP_Logger to poll erase completion
     * @note Each block erase takes ~3ms
     * 
     * Source: libraries/AP_Logger/AP_Logger_W25NXX.cpp (implementation)
     */
    bool              InErase() override;
    
    /**
     * @brief Send SPI command with 24-bit address to W25N device
     * 
     * @details W25N command format:
     *          - Byte 0: Command opcode
     *          - Byte 1: Address bits [23:16] (typically 0x00 for page addressing)
     *          - Byte 2: Address bits [15:8] (page address high byte)
     *          - Byte 3: Address bits [7:0] (page address low byte)
     * 
     * @param[in] cmd Command opcode (e.g., PAGE READ=0x13, PROGRAM EXECUTE=0x10)
     * @param[in] address 24-bit address (page number for most commands)
     * 
     * @note Assumes caller holds dev_sem semaphore
     * @note Some commands ignore address parameter
     * 
     * Source: libraries/AP_Logger/AP_Logger_W25NXX.cpp (implementation)
     */
    void              send_command_addr(uint8_t cmd, uint32_t address);
    
    /**
     * @brief Wait for W25N device to complete current operation
     * 
     * @details Polls status register BUSY bit (bit 0) until clear:
     *          - Page program: ~700μs
     *          - Block erase: ~3ms
     *          - Page read to buffer: ~60μs
     * 
     * @warning Blocks calling thread - may wait milliseconds
     * @warning Do not call from interrupt context
     * @note Yields CPU during polling to allow other tasks to run
     * 
     * Source: libraries/AP_Logger/AP_Logger_W25NXX.cpp (implementation)
     */
    void              WaitReady();
    
    /**
     * @brief Check if W25N device is busy with current operation
     * 
     * @details Reads status register bit 0 (BUSY):
     *          - 0: Device ready for new command
     *          - 1: Device busy (program/erase/read in progress)
     * 
     * @return true if device is busy, false if ready
     * 
     * @note Non-blocking status check
     * @note Use WaitReady() to block until operation completes
     * 
     * Source: libraries/AP_Logger/AP_Logger_W25NXX.cpp (implementation)
     */
    bool              Busy();
    
    /**
     * @brief Read specific bits from W25N status register
     * 
     * @details W25N Status Register (0xC0) bits:
     *          - Bit 0: BUSY (1=busy, 0=ready)
     *          - Bit 1: WEL (Write Enable Latch)
     *          - Bit 2: E_FAIL (Erase failure)
     *          - Bit 3: P_FAIL (Program failure)
     *          - Bits 4-5: ECC status (00=pass, 01=corrected, 10=multiple, 11=uncorrectable)
     * 
     * @param[in] bits Bitmask specifying which bits to read
     * 
     * @return Status register value masked with specified bits
     * 
     * @note Most commonly used to check ECC status after read/program
     * @warning ECC bits 4-5 = 11 indicates uncorrectable error (data invalid)
     * 
     * Source: libraries/AP_Logger/AP_Logger_W25NXX.cpp (implementation)
     */
    uint8_t           ReadStatusRegBits(uint8_t bits);
    
    /**
     * @brief Write to W25N configuration register
     * 
     * @details W25N has multiple registers:
     *          - 0xB0: Feature Register (ECC enable, buffer mode, reset)
     *          - 0xC0: Status Register (read-only except for clearing error flags)
     *          - 0xA0: Protection Register (block write protection)
     * 
     * @param[in] reg Register address (0xA0, 0xB0, or 0xC0)
     * @param[in] bits Data byte to write to register
     * 
     * @note Used during Init() to configure ECC and buffer modes
     * @warning Incorrect configuration can disable ECC or prevent writes
     * 
     * Source: libraries/AP_Logger/AP_Logger_W25NXX.cpp (implementation)
     */
    void              WriteStatusReg(uint8_t reg, uint8_t bits);

    /**
     * @brief Enable writes to W25N flash device
     * 
     * @details Sets Write Enable Latch (WEL) in status register:
     *          - Required before any program or erase operation
     *          - Automatically cleared after program/erase completes
     *          - Must be set again for each write/erase command
     * 
     * @note Check WEL bit (status register bit 1) to verify
     * @warning Attempting program/erase without write enable will fail silently
     * 
     * Source: libraries/AP_Logger/AP_Logger_W25NXX.cpp (implementation)
     */
    void              WriteEnable();
    
    /**
     * @brief Detect flash chip capacity and calculate available sectors
     * 
     * @details Initialization helper that:
     *          1. Reads device ID to determine chip model
     *          2. Calculates total pages based on device capacity
     *          3. Determines block count (pages / 64)
     *          4. Sets df_NumPages for AP_Logger_Block base class
     * 
     * @return true if device detected and capacity determined, false on failure
     * 
     * @note Supported devices: W25N01G (1Gbit), W25N02G (2Gbit), etc.
     * @note Sets flash_died=true if detection fails
     * 
     * Source: libraries/AP_Logger/AP_Logger_W25NXX.cpp (implementation)
     */
    bool              getSectorCount(void);

    // Hardware interface members
    
    /**
     * @brief SPI device handle for W25N flash chip
     * 
     * @details Owned pointer to HAL SPI device interface providing:
     *          - SPI bus access with chip select control
     *          - Transfer methods for command/data exchange
     *          - Bus speed configuration
     * 
     * @note Acquired during Init() via HAL::SPIDeviceManager
     * @note Automatically released on backend destruction
     */
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev;
    
    /**
     * @brief Semaphore protecting SPI device access
     * 
     * @details Thread-safety mechanism ensuring:
     *          - Only one thread accesses SPI bus at a time
     *          - Atomic command sequences (e.g., write enable + program)
     *          - Protection during multi-byte transfers
     * 
     * @note Must be held during all device operations
     * @warning Not holding semaphore can cause bus contention or data corruption
     */
    AP_HAL::Semaphore *dev_sem;

    // Flash device state members
    
    /**
     * @brief Total number of erasable blocks in flash device
     * 
     * @details Block organization:
     *          - W25N NAND flash: 1 block = 64 pages = 128KB
     *          - Total capacity = flash_blockNum * 128KB
     *          - Example: W25N01G has 1024 blocks (128MB total)
     * 
     * @note Calculated during Init() via getSectorCount()
     * @note Used for mass erase operations
     */
    uint32_t flash_blockNum;

    /**
     * @brief Flag indicating permanent flash device failure
     * 
     * @details Set to true when:
     *          - Device fails to respond during Init()
     *          - Device ID read fails
     *          - Repeated ECC uncorrectable errors
     *          - Command interface becomes unresponsive
     * 
     * @note Once set, CardInserted() returns false permanently
     * @warning Device failure requires power cycle or hardware replacement
     * @note Used to prevent continued attempts to access failed device
     */
    bool flash_died;
    
    /**
     * @brief Timestamp (milliseconds) when mass erase operation started
     * 
     * @details Used to track erase operation timing:
     *          - Set by StartErase()
     *          - Used by InErase() to pace block erasing
     *          - Allows timeout detection for stuck erase operations
     * 
     * @note Erase timing: ~3ms per block * flash_blockNum blocks
     */
    uint32_t erase_start_ms;
    
    /**
     * @brief Current block number being erased during mass erase operation
     * 
     * @details State machine counter for asynchronous erase:
     *          - Initialized to 0 by StartErase()
     *          - Incremented by InErase() after each block erase
     *          - Erase complete when erase_block >= flash_blockNum
     * 
     * @note Allows non-blocking erase operation spread across multiple calls
     * @note Each block erase takes ~3ms, so incremental erase prevents watchdog timeouts
     */
    uint16_t erase_block;
    
    /**
     * @brief Cache validity flag for last page read into buffer
     * 
     * @details Optimization to avoid redundant PAGE READ operations:
     *          - Set to true after successful PageToBuffer()
     *          - Cleared when buffer modified or different page requested
     *          - Allows multiple reads from same page without re-reading
     * 
     * @note W25N PAGE READ takes ~60μs, so cache provides measurable benefit
     * @note Invalidated by any write operation or page address change
     */
    bool read_cache_valid;
};

#endif // HAL_LOGGING_FLASH_W25NXX_ENABLED
