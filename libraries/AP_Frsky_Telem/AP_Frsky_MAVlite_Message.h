#pragma once

/**
 * @file AP_Frsky_MAVlite_Message.h
 * @brief MAVlite message structure with typed accessor wrapper for binary payload data
 * 
 * @details This file defines the AP_Frsky_MAVlite_Message class, which provides
 *          type-safe access to MAVlite message payload bytes. MAVlite is a lightweight
 *          bidirectional communication protocol used by FrSky telemetry systems.
 *          
 *          The message structure consists of:
 *          - msgid: 1 byte message identifier
 *          - len: 1 byte payload length (0-31 bytes)
 *          - payload: up to 31 bytes of data
 *          
 *          Multi-byte values are stored in little-endian byte order.
 *          
 *          This class is only compiled when HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL is enabled.
 */

#include "AP_Frsky_MAVlite.h"

#include <AP_Common/AP_Common.h>

#include <stdint.h>

#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
/**
 * @class AP_Frsky_MAVlite_Message
 * @brief Type-safe accessor wrapper for MAVlite message payload data
 * 
 * @details This class provides typed accessor methods for reading and writing
 *          data to MAVlite message payloads. MAVlite messages have a fixed structure:
 *          
 *          Memory layout:
 *          - msgid: 1 byte - Message type identifier
 *          - len: 1 byte - Payload length in bytes (0-31)
 *          - payload: Up to 31 bytes of message-specific data
 *          
 *          Endianness:
 *          All multi-byte types (uint16_t, float) are stored in little-endian byte order.
 *          
 *          Bounds checking:
 *          All accessor methods perform bounds checking and return false if the requested
 *          offset + size would exceed MAVLITE_MAX_PAYLOAD_LEN (31 bytes).
 *          
 *          Alignment:
 *          4-byte alignment is NOT required for float access. Values are accessed
 *          byte-by-byte to support unaligned access on any platform.
 *          
 * @note All accessor methods are marked WARN_IF_UNUSED, requiring return value checking
 * @warning Bounds checking is critical - verify all accessors return true before using data
 * @see AP_Frsky_MAVlite.h for MAVLITE_MAX_PAYLOAD_LEN constant definition
 */
class AP_Frsky_MAVlite_Message {
public:
    // Float accessors
    /**
     * @brief Read a 32-bit IEEE 754 float from the payload
     * 
     * @param[out] value      The float value read from payload (little-endian)
     * @param[in]  offset     Byte offset in payload (0-27 for float)
     * 
     * @return true if offset+4 is within bounds (offset <= 27), false otherwise
     * 
     * @note 4-byte alignment is NOT required - values accessed byte-by-byte
     * @note Offset+size must not exceed MAVLITE_MAX_PAYLOAD_LEN (31 bytes)
     * @warning Return value MUST be checked due to WARN_IF_UNUSED attribute
     */
    bool get_float(float &value, const uint8_t offset) const WARN_IF_UNUSED {
        return get_bytes((uint8_t*)&value, offset, 4);
    }
    
    /**
     * @brief Write a 32-bit IEEE 754 float to the payload
     * 
     * @param[in]  value      The float value to write (stored as little-endian)
     * @param[in]  offset     Byte offset in payload (0-27 for float)
     * 
     * @return true if offset+4 is within bounds (offset <= 27), false otherwise
     * 
     * @note 4-byte alignment is NOT required - values accessed byte-by-byte
     * @note Offset+size must not exceed MAVLITE_MAX_PAYLOAD_LEN (31 bytes)
     * @warning Return value MUST be checked due to WARN_IF_UNUSED attribute
     */
    bool set_float(const float value, const uint8_t offset) WARN_IF_UNUSED {
        return set_bytes((uint8_t*)&value, offset, 4);
    }

    // String accessors
    /**
     * @brief Read a null-terminated string from the payload
     * 
     * @param[out] value      Buffer to store the null-terminated string (max 16 bytes including null)
     * @param[in]  offset     Byte offset in payload where string starts (0-30)
     * 
     * @return true if string fits within payload bounds, false otherwise
     * 
     * @note Maximum string length is 16 bytes including null terminator
     * @note String must fit between offset and MAVLITE_MAX_PAYLOAD_LEN
     * @warning Return value MUST be checked due to WARN_IF_UNUSED attribute
     * @warning Caller must ensure value buffer is large enough (at least 16 bytes)
     */
    bool get_string(char* value, const uint8_t offset) const WARN_IF_UNUSED;
    
    /**
     * @brief Write a null-terminated string to the payload
     * 
     * @param[in]  value      Null-terminated string to write (max 16 bytes including null)
     * @param[in]  offset     Byte offset in payload where string starts (0-30)
     * 
     * @return true if string fits within payload bounds, false otherwise
     * 
     * @note Maximum string length is 16 bytes including null terminator
     * @note String must fit between offset and MAVLITE_MAX_PAYLOAD_LEN
     * @warning Return value MUST be checked due to WARN_IF_UNUSED attribute
     */
    bool set_string(const char* value, const uint8_t offset) WARN_IF_UNUSED;

    // uint16_t accessors
    /**
     * @brief Read a 16-bit unsigned integer from the payload
     * 
     * @param[out] value      The uint16_t value read from payload (little-endian)
     * @param[in]  offset     Byte offset in payload (0-29 for uint16_t)
     * 
     * @return true if offset+2 is within bounds (offset <= 29), false otherwise
     * 
     * @note Values stored in little-endian byte order
     * @note Offset+size must not exceed MAVLITE_MAX_PAYLOAD_LEN (31 bytes)
     * @warning Return value MUST be checked due to WARN_IF_UNUSED attribute
     */
    bool get_uint16(uint16_t &value, const uint8_t offset) const WARN_IF_UNUSED {
        return get_bytes((uint8_t*)&value, offset, 2);
    }
    
    /**
     * @brief Write a 16-bit unsigned integer to the payload
     * 
     * @param[in]  value      The uint16_t value to write (stored as little-endian)
     * @param[in]  offset     Byte offset in payload (0-29 for uint16_t)
     * 
     * @return true if offset+2 is within bounds (offset <= 29), false otherwise
     * 
     * @note Values stored in little-endian byte order
     * @note Offset+size must not exceed MAVLITE_MAX_PAYLOAD_LEN (31 bytes)
     * @warning Return value MUST be checked due to WARN_IF_UNUSED attribute
     */
    bool set_uint16(const uint16_t value, const uint8_t offset) WARN_IF_UNUSED {
        return set_bytes((uint8_t*)&value, offset, 2);
    }

    // uint8_t accessors
    /**
     * @brief Read an 8-bit unsigned integer from the payload
     * 
     * @param[out] value      The uint8_t value read from payload
     * @param[in]  offset     Byte offset in payload (0-30 for uint8_t)
     * 
     * @return true if offset is within bounds (offset <= 30), false otherwise
     * 
     * @note Offset must not exceed MAVLITE_MAX_PAYLOAD_LEN-1 (30)
     * @warning Return value MUST be checked due to WARN_IF_UNUSED attribute
     */
    bool get_uint8(uint8_t &value, const uint8_t offset) const WARN_IF_UNUSED {
        return get_bytes((uint8_t*)&value, offset, 1);
    }
    
    /**
     * @brief Write an 8-bit unsigned integer to the payload
     * 
     * @param[in]  value      The uint8_t value to write
     * @param[in]  offset     Byte offset in payload (0-30 for uint8_t)
     * 
     * @return true if offset is within bounds (offset <= 30), false otherwise
     * 
     * @note Offset must not exceed MAVLITE_MAX_PAYLOAD_LEN-1 (30)
     * @warning Return value MUST be checked due to WARN_IF_UNUSED attribute
     */
    bool set_uint8(const uint8_t value, const uint8_t offset) WARN_IF_UNUSED {
        return set_bytes((uint8_t*)&value, offset, 1);
    }

    // Public message structure members
    /**
     * @brief MAVlite message ID identifying the message type
     * 
     * @details The msgid field identifies the message type and determines how
     *          the payload should be interpreted. Message IDs are defined by
     *          the MAVlite protocol specification.
     */
    uint8_t msgid = 0;
    
    /**
     * @brief Payload length in bytes (0-31)
     * 
     * @details The len field specifies the number of valid data bytes in the
     *          payload array. Valid range is 0 to MAVLITE_MAX_PAYLOAD_LEN (31).
     * 
     * @warning This length does NOT include the msgid, len, or checksum bytes -
     *          it represents only the payload data length
     */
    uint8_t len = 0;
    
    /**
     * @brief Raw payload data buffer
     * 
     * @details Binary payload data with maximum size of MAVLITE_MAX_PAYLOAD_LEN
     *          (31 bytes). Actual valid data length is specified by the len field.
     *          Use the typed accessor methods (get_float, get_uint16, etc.) for
     *          safe, bounds-checked access to payload data.
     * 
     * @note Array size is MAVLITE_MAX_PAYLOAD_LEN (31 bytes)
     * @see get_float, get_uint16, get_uint8, get_string for type-safe access
     */
    uint8_t payload[MAVLITE_MAX_PAYLOAD_LEN];

    // Bit manipulation helpers
    /**
     * @brief Pack a bit field into a uint8_t value
     * 
     * @details This helper function inserts a bit field value into a uint8_t at
     *          a specified bit position. Useful for packing multiple small values
     *          into a single byte for efficient payload usage.
     * 
     * @param[in,out] value        The uint8_t value to modify with packed bits
     * @param[in]     bit_value    The bits to insert (only low bit_count bits used)
     * @param[in]     bit_count    Number of bits to insert (1-8)
     * @param[in]     bit_offset   Bit position to start insertion (0-7, where 0 is LSB)
     * 
     * @note bit_offset + bit_count must not exceed 8
     * @note Existing bits outside the specified range are preserved
     * 
     * Example: bit8_pack(value, 0b101, 3, 2) inserts 3 bits at position 2
     */
    static void bit8_pack(uint8_t &value, const uint8_t bit_value, const uint8_t bit_count, const uint8_t bit_offset);
    
    /**
     * @brief Extract a bit field from a uint8_t value
     * 
     * @details This helper function extracts a bit field from a uint8_t at a
     *          specified bit position. Useful for unpacking multiple small values
     *          that were packed into a single byte.
     * 
     * @param[in] value        The uint8_t value to extract bits from
     * @param[in] bit_count    Number of bits to extract (1-8)
     * @param[in] bit_offset   Bit position to start extraction (0-7, where 0 is LSB)
     * 
     * @return Extracted bits right-aligned (low bit_count bits contain the field)
     * 
     * @note bit_offset + bit_count must not exceed 8
     * @note Returned value has only the low bit_count bits valid
     * 
     * Example: bit8_unpack(0b00010100, 3, 2) extracts 3 bits starting at position 2, returns 0b101
     */
    static uint8_t bit8_unpack(const uint8_t value, const  uint8_t bit_count, const uint8_t bit_offset);

private:
    /**
     * @brief Read multiple bytes from the payload (internal helper)
     * 
     * @param[out] bytes       Buffer to store the read bytes
     * @param[in]  offset      Byte offset in payload where reading starts
     * @param[in]  count       Number of bytes to read
     * 
     * @return true if offset+count is within bounds, false otherwise
     * 
     * @note This is the internal implementation used by all typed get accessors
     * @warning Return value MUST be checked due to WARN_IF_UNUSED attribute
     */
    bool get_bytes(uint8_t *bytes, const uint8_t offset, const uint8_t count) const WARN_IF_UNUSED;
    
    /**
     * @brief Write multiple bytes to the payload (internal helper)
     * 
     * @param[in]  bytes       Buffer containing bytes to write
     * @param[in]  offset      Byte offset in payload where writing starts
     * @param[in]  count       Number of bytes to write
     * 
     * @return true if offset+count is within bounds, false otherwise
     * 
     * @note This is the internal implementation used by all typed set accessors
     * @warning Return value MUST be checked due to WARN_IF_UNUSED attribute
     */
    bool set_bytes(const uint8_t *bytes,  const uint8_t offset, const uint8_t count) WARN_IF_UNUSED;
};
#endif