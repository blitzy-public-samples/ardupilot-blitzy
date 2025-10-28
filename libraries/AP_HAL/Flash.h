/**
 * @file Flash.h
 * @brief Internal flash memory interface for direct flash operations
 * 
 * Provides low-level access to microcontroller internal flash memory for bootloader,
 * firmware storage, and parameter persistence. Handles page-based erase and word/byte
 * write operations with proper unlock/lock sequencing.
 * 
 * @warning Direct flash operations can brick the board if misused - primarily for bootloader
 */
#pragma once

#include <stdint.h>
#include "AP_HAL_Namespace.h"

/**
 * @class AP_HAL::Flash
 * @brief Abstract interface for platform-specific flash memory operations
 * 
 * @details Provides page-based erase and arbitrary-address write operations for internal
 *          flash memory. Flash memory characteristics:
 *          - Must be erased (set to 0xFF) before writing
 *          - Erase operates on fixed-size pages (typically 1-4KB)
 *          - Write can modify individual bytes/words but only 1→0 transitions
 *          - Limited erase/write cycles (typically 10,000-100,000)
 *          
 *          Common use cases:
 *          - Bootloader: Writing new firmware during updates
 *          - Storage: Parameter persistence across reboots
 *          - Logs: On-flash circular log buffers
 *          
 *          Page layout is platform-specific and may vary within a single device.
 *          Always query page address and size before erase/write operations.
 * 
 * @note Flash writes are blocking and may take milliseconds - do not call from interrupts
 * @note Flash memory is shared with executing firmware - writing executable regions crashes
 * @warning Erasing or writing wrong pages can brick the board requiring JTAG recovery
 * @warning Keep flash unlocked only during write operations to prevent accidental corruption
 */
class AP_HAL::Flash {
public:
    /**
     * @brief Get start address of specified flash page
     * 
     * @param[in] page Page number (0 to getnumpages()-1)
     * 
     * @return uint32_t Absolute address of page start
     * 
     * @note Page numbering is zero-based and contiguous
     * @note Some MCUs have non-uniform page sizes - page N+1 address ≠ page N address + size
     */
    virtual uint32_t getpageaddr(uint32_t page) = 0;
    
    /**
     * @brief Get size in bytes of specified flash page
     * 
     * @param[in] page Page number (0 to getnumpages()-1)
     * 
     * @return uint32_t Page size in bytes (typically 1024, 2048, or 4096)
     * 
     * @note Page sizes may vary within same device (e.g., STM32: 16KB then 128KB pages)
     * @note Always query size for each page - do not assume uniform sizing
     */
    virtual uint32_t getpagesize(uint32_t page) = 0;
    
    /**
     * @brief Get total number of flash pages
     * 
     * @return uint32_t Total page count
     * 
     * @note Valid page numbers: 0 to getnumpages()-1
     */
    virtual uint32_t getnumpages(void) = 0;
    
    /**
     * @brief Erase specified flash page (set all bytes to 0xFF)
     * 
     * @param[in] page Page number to erase
     * 
     * @return true if erase successful, false on error
     * 
     * @note Erasing page sets all bytes to 0xFF (blank state)
     * @note Erase time: typically 10-100ms depending on page size and flash technology
     * @note Flash must be unlocked via keep_unlocked(true) before calling
     * @warning Blocks for entire erase duration - do not call from time-critical contexts
     * @warning Erasing firmware pages crashes system - verify page before erasing
     */
    virtual bool erasepage(uint32_t page) = 0;
    
    /**
     * @brief Write data to flash at specified address
     * 
     * @param[in] addr Absolute flash address (must be within valid flash range)
     * @param[in] buf Pointer to data to write
     * @param[in] count Number of bytes to write
     * 
     * @return true if write successful, false on error
     * 
     * @note Flash region must be erased (0xFF) before writing
     * @note Write time: typically 50-100μs per word/byte depending on flash technology
     * @note Flash must be unlocked via keep_unlocked(true) before calling
     * @note Some platforms require word-aligned address and word-multiple count
     * @warning Writing to currently executing flash region crashes system
     * @warning Writing unerased bytes may silently fail (bits can only transition 1→0)
     */
    virtual bool write(uint32_t addr, const void *buf, uint32_t count) = 0;
    
    /**
     * @brief Control flash unlock state for write/erase operations
     * 
     * @param[in] set true to unlock flash, false to lock
     * 
     * @note Flash must be unlocked before erase or write operations
     * @note Keep flash locked when not actively writing to prevent accidental corruption
     * @warning Never leave flash unlocked during interrupt-driven or error-prone code
     */
    virtual void keep_unlocked(bool set) = 0;
    
    /**
     * @brief Check if specified flash page is erased (all bytes 0xFF)
     * 
     * @param[in] page Page number to check
     * 
     * @return true if all bytes in page are 0xFF, false if any byte is not 0xFF
     * 
     * @note Useful before write to verify erase succeeded
     * @note May be faster than reading and comparing all bytes on some platforms
     */
    virtual bool ispageerased(uint32_t page) = 0;
};
