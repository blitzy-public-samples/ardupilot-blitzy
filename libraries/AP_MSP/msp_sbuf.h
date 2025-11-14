/**
 * @file msp_sbuf.h
 * @brief Safe buffer (sbuf) utilities for MSP packet serialization and deserialization
 * 
 * @details This file provides bounds-checked buffer operations for marshaling and
 *          unmarshaling MSP (MultiWii Serial Protocol) packets. The sbuf implementation
 *          is adapted from Betaflight's streambuf for safe buffer access.
 * 
 *          Key features:
 *          - Bounds checking on all read/write operations
 *          - Read/write pointer management
 *          - Buffer conversion for bidirectional use
 *          - Protection against buffer overflows
 * 
 * Source: Code ported and adapted from betaflight/src/main/common/streambuf.h
 */
#pragma once
#include "msp.h"
#include <stdint.h>

#if HAL_MSP_ENABLED

/**
 * @namespace MSP
 * @brief MultiWii Serial Protocol implementation namespace
 * 
 * @details Contains MSP packet handling, serialization utilities, and safe buffer
 *          operations for communicating with MSP-compatible devices and OSDs.
 */
namespace MSP
{

/**
 * @struct sbuf_s
 * @brief Safe buffer structure for bounds-checked read/write operations
 * 
 * @details This structure maintains pointers for safe buffer access with overflow protection.
 *          It can be used for both writing (serialization) and reading (deserialization) by
 *          switching between modes with sbuf_switch_to_reader().
 * 
 *          The structure layout is carefully designed to allow casting sbuf_t* to uint8_t**
 *          for compatibility with certain MSP operations.
 * 
 * @note The ptr field MUST be first to allow sbuf_t* to be cast to uint8_t**
 */
typedef struct sbuf_s {
    uint8_t *ptr;          ///< Current read/write position in buffer (must be first for uint8_t** compatibility)
    uint8_t *end;          ///< Pointer to end of buffer for bounds checking
} sbuf_t;

// Helper functions for safe buffer operations

/**
 * @brief Get current buffer pointer position
 * 
 * @param[in] buf The sbuf instance
 * @return Pointer to current position in buffer for read/write operations
 */
uint8_t* sbuf_ptr(sbuf_t *buf);

/**
 * @brief Calculate remaining bytes from current position to end of buffer
 * 
 * @param[in] buf The sbuf instance (const)
 * @return Number of bytes remaining in buffer from current position
 * 
 * @note Used to determine if sufficient space exists for pending operations
 */
uint16_t sbuf_bytes_remaining(const sbuf_t *buf);

/**
 * @brief Check if len bytes can be read/written without buffer overflow
 * 
 * @param[in] buf The sbuf instance (const)
 * @param[in] len Number of bytes to check for availability
 * @return true if len bytes are available, false if operation would overflow
 * 
 * @warning Calls INTERNAL_ERROR on bounds violation for debugging
 * 
 * @note This function provides safety checks for all buffer operations
 */
bool sbuf_check_bounds(const sbuf_t *buf, const uint8_t len);

/**
 * @brief Convert write buffer to read buffer by resetting pointers
 * 
 * @param[in,out] buf The sbuf instance to convert from write to read mode
 * @param[in] base Base pointer to start of data to be read
 * 
 * @details After writing data to a buffer, this function prepares it for reading by:
 *          - Setting ptr to base (start of data)
 *          - Setting end to current ptr position (end of written data)
 *          This allows the same buffer structure to be reused for reading what was written.
 * 
 * @note Sets ptr=base and end=current ptr position for reading written data
 */
void sbuf_switch_to_reader(sbuf_t *buf, uint8_t *base);

/**
 * @brief Write raw data to sbuf with bounds checking
 * 
 * @param[in,out] dst Destination sbuf to write data into
 * @param[in] data Source data pointer to copy from
 * @param[in] len Number of bytes to write from source to destination
 * 
 * @details Copies len bytes from data to the current position in dst buffer,
 *          then advances the dst pointer. Performs bounds checking to prevent
 *          buffer overflow.
 * 
 * @warning Performs bounds checking via sbuf_check_bounds before writing
 */
void sbuf_write_data(sbuf_t *dst, const void *data, int len);

/**
 * @note The sbuf implementation provides safe bounds-checked buffer operations for MSP
 *       packet marshaling. All operations check buffer boundaries to prevent overflows,
 *       making it suitable for processing untrusted MSP data from external devices.
 */
}

#endif //HAL_MSP_ENABLED