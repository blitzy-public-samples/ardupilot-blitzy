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
 * @file crc.h
 * @brief Comprehensive CRC, checksum, and hash function suite for data integrity verification
 * 
 * @details This file provides a complete collection of cyclic redundancy checks (CRC),
 *          checksums, and hash functions used throughout ArduPilot for data integrity
 *          verification, error detection, and data validation. The suite includes:
 * 
 *          - CRC4: Simple 4-bit CRC for bootloader verification
 *          - CRC8 family: DVB-S2, DVB, Maxim, SAE, RDS02UF standards for telemetry
 *          - CRC16 family: XMODEM, CCITT, IBM, Modbus, GDL90 for serial protocols
 *          - CRC24: Used in specialized telemetry protocols
 *          - CRC32: High-strength verification for storage and firmware validation
 *          - CRC64: Extended precision for large data blocks
 *          - Fletcher checksum: Fast error detection for network protocols
 *          - FNV-1a hash: Fast non-cryptographic hash for data fingerprinting
 * 
 *          Implementations are optimized for embedded systems with both table-driven
 *          (fast, higher memory) and computation-based (slower, minimal memory) variants
 *          available for resource-constrained platforms.
 * 
 * @note    Use cases include:
 *          - Bootloader firmware verification (CRC4, CRC32)
 *          - Telemetry checksums (CRC8, CRC16)
 *          - Storage integrity verification (CRC32, CRC64)
 *          - MAVLink message validation (CRC16)
 *          - Parameter storage validation (CRC32)
 *          - Log file integrity (CRC32)
 * 
 * @warning Multi-byte CRC calculations may have endianness dependencies.
 *          Ensure consistent byte ordering when validating CRCs across
 *          different architectures or when comparing with external systems.
 * 
 * @author  ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */
#pragma once

#include <stdint.h>

/**
 * @brief Calculate 4-bit CRC for bootloader verification
 * 
 * @details Computes a simple 4-bit cyclic redundancy check commonly used in
 *          bootloader protocols for firmware validation. This is a lightweight
 *          integrity check suitable for small data blocks.
 * 
 * @param[in] data Pointer to 16-bit data words to calculate CRC over
 * 
 * @return 16-bit result containing the 4-bit CRC value
 * 
 * @note Primarily used in bootloader firmware verification sequences
 * @note The 4-bit CRC is packed into a 16-bit return value
 */
uint16_t crc_crc4(uint16_t *data);

/**
 * @brief Calculate standard 8-bit CRC
 * 
 * @details Computes a standard 8-bit CRC using lookup table for fast computation.
 *          This is a general-purpose CRC8 implementation suitable for basic
 *          data integrity checks.
 * 
 * @param[in] p   Pointer to data buffer
 * @param[in] len Length of data buffer in bytes
 * 
 * @return 8-bit CRC value
 * 
 * @note Uses table-driven implementation for performance
 */
uint8_t crc_crc8(const uint8_t *p, uint8_t len);

/**
 * @brief Calculate 8-bit CRC with custom polynomial (computation-based)
 * 
 * @details Computes an 8-bit CRC using a specified polynomial without using
 *          a lookup table. This is suitable for memory-constrained systems
 *          or when using non-standard polynomials. Slower than table-based
 *          methods but uses minimal memory.
 * 
 * @param[in] buf           Pointer to data buffer
 * @param[in] buf_len       Length of data buffer in bytes
 * @param[in] polynomial    CRC polynomial to use (e.g., 0x07, 0x31, 0xD5)
 * @param[in] initial_value Initial CRC value (seed), defaults to 0
 * 
 * @return 8-bit CRC value
 * 
 * @note Does not use lookup table - trades speed for memory savings
 * @note Suitable for custom or non-standard CRC polynomials
 */
uint8_t crc8_generic(const uint8_t *buf, const uint16_t buf_len, const uint8_t polynomial, uint8_t initial_value=0);

/**
 * @brief Calculate CRC8 using DVB-S2 standard for single byte
 * 
 * @details Implements the DVB-S2 (Digital Video Broadcasting - Satellite Second Generation)
 *          CRC8 algorithm for a single byte. DVB-S2 uses polynomial 0xD5 and is
 *          commonly used in satellite communication protocols.
 * 
 * @param[in] crc Current CRC value (use 0 for first byte)
 * @param[in] a   Byte to add to CRC calculation
 * 
 * @return Updated 8-bit CRC value
 * 
 * @note DVB-S2 polynomial: 0xD5 (x^8 + x^7 + x^6 + x^4 + x^2 + 1)
 * @note Used in satellite telemetry protocols
 */
uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a);

/**
 * @brief Calculate CRC8 using DVB standard for single byte
 * 
 * @details Implements the DVB (Digital Video Broadcasting) CRC8 algorithm
 *          for a single byte with configurable seed value.
 * 
 * @param[in] crc  Current CRC value
 * @param[in] a    Byte to add to CRC calculation
 * @param[in] seed Seed value for CRC initialization
 * 
 * @return Updated 8-bit CRC value
 * 
 * @note Standard DVB CRC8 used in digital broadcasting protocols
 */
uint8_t crc8_dvb(uint8_t crc, uint8_t a, uint8_t seed);

/**
 * @brief Calculate CRC8 DVB-S2 for data buffer
 * 
 * @details Implements DVB-S2 CRC8 algorithm for a complete data buffer.
 *          This is the incremental update version for processing multiple bytes.
 * 
 * @param[in] crc    Current CRC value (use 0 for initial call)
 * @param[in] data   Pointer to data buffer
 * @param[in] length Length of data buffer in bytes
 * 
 * @return Updated 8-bit CRC value
 * 
 * @note Can be called incrementally for streaming data
 * @note DVB-S2 standard used in satellite communication systems
 */
uint8_t crc8_dvb_s2_update(uint8_t crc, const void *data, uint32_t length);

/**
 * @brief Calculate CRC8 DVB for data buffer
 * 
 * @details Implements DVB CRC8 algorithm for a complete data buffer.
 * 
 * @param[in] crc     Current CRC value (use 0 for initial call)
 * @param[in] buf     Pointer to data buffer
 * @param[in] buf_len Length of data buffer in bytes
 * 
 * @return Updated 8-bit CRC value
 * 
 * @note Standard DVB protocol CRC8 variant
 */
uint8_t crc8_dvb_update(uint8_t crc, const uint8_t* buf, const uint16_t buf_len);

/**
 * @brief Calculate CRC8 using Maxim (Dallas) standard
 * 
 * @details Implements the Maxim/Dallas 1-Wire CRC8 algorithm using polynomial 0x31.
 *          Commonly used in 1-Wire devices and Dallas semiconductor sensors.
 * 
 * @param[in] data   Pointer to data buffer
 * @param[in] length Length of data buffer in bytes
 * 
 * @return 8-bit CRC value
 * 
 * @note Maxim polynomial: 0x31 (x^8 + x^5 + x^4 + 1)
 * @note Used in Dallas/Maxim 1-Wire protocol devices
 * @note Common in temperature sensors and other I2C/1-Wire peripherals
 */
uint8_t crc8_maxim(const uint8_t *data, uint16_t length);

/**
 * @brief Calculate CRC8 using SAE J1850 standard
 * 
 * @details Implements the SAE J1850 CRC8 algorithm used in automotive diagnostics.
 *          SAE J1850 is commonly used in vehicle communication protocols.
 * 
 * @param[in] data   Pointer to data buffer
 * @param[in] length Length of data buffer in bytes
 * 
 * @return 8-bit CRC value
 * 
 * @note SAE J1850 polynomial: 0x1D (x^8 + x^4 + x^3 + x^2 + 1)
 * @note Used in automotive diagnostic protocols (OBD-II)
 */
uint8_t crc8_sae(const uint8_t *data, uint16_t length);

/**
 * @brief Calculate CRC8 using RDS02UF standard
 * 
 * @details Implements the RDS02UF CRC8 algorithm used in specific telemetry protocols.
 * 
 * @param[in] data   Pointer to data buffer
 * @param[in] length Length of data buffer in bytes
 * 
 * @return 8-bit CRC value
 * 
 * @note Used in RDS02UF telemetry protocol
 */
uint8_t crc8_rds02uf(const uint8_t *data, uint16_t length);

/**
 * @brief Update CRC16 XMODEM calculation with single byte
 * 
 * @details Incrementally updates a CRC16 value using the XMODEM protocol standard.
 *          XMODEM uses polynomial 0x1021 with initial value 0x0000 and is widely
 *          used in file transfer protocols.
 * 
 * @param[in] crc  Current CRC value (use 0 for first byte)
 * @param[in] data Byte to add to CRC calculation
 * 
 * @return Updated 16-bit CRC value
 * 
 * @note XMODEM polynomial: 0x1021 (x^16 + x^12 + x^5 + 1)
 * @note Initial value: 0x0000
 * @note Used in XMODEM/YMODEM file transfer protocols
 */
uint16_t crc_xmodem_update(uint16_t crc, uint8_t data);

/**
 * @brief Calculate CRC16 XMODEM for data buffer
 * 
 * @details Computes a complete CRC16 using XMODEM standard for entire data buffer.
 * 
 * @param[in] data Pointer to data buffer
 * @param[in] len  Length of data buffer in bytes
 * 
 * @return 16-bit CRC value
 * 
 * @note XMODEM polynomial: 0x1021, initial value: 0x0000
 * @note Common in serial file transfer protocols
 */
uint16_t crc_xmodem(const uint8_t *data, uint16_t len);

/**
 * @brief Calculate CRC32 using table-driven algorithm
 * 
 * @details Computes a 32-bit CRC using a lookup table for fast calculation.
 *          This is the standard CRC32 algorithm (polynomial 0x04C11DB7) used
 *          extensively for firmware verification, file integrity checks, and
 *          storage validation in ArduPilot. Uses table-driven approach for
 *          high performance at the cost of ~1KB lookup table.
 * 
 * @param[in] crc  Current CRC value (use 0xFFFFFFFF for initial call, or ~0U)
 * @param[in] buf  Pointer to data buffer
 * @param[in] size Length of data buffer in bytes
 * 
 * @return 32-bit CRC value
 * 
 * @note CRC32 polynomial: 0x04C11DB7 (Ethernet/ZIP standard)
 * @note Uses 256-entry lookup table (1KB memory)
 * @note Primary use: firmware validation, parameter storage, log integrity
 * @note Can be called incrementally for large data blocks
 */
uint32_t crc_crc32(uint32_t crc, const uint8_t *buf, uint32_t size);

/**
 * @brief Calculate CRC32 using computation-based algorithm
 * 
 * @details Computes a 32-bit CRC without using a lookup table, trading
 *          performance for minimal memory usage. Suitable for memory-constrained
 *          systems or when the ~1KB table overhead is prohibitive.
 * 
 * @param[in] crc  Current CRC value (use 0xFFFFFFFF for initial call)
 * @param[in] buf  Pointer to data buffer
 * @param[in] size Length of data buffer in bytes
 * 
 * @return 32-bit CRC value
 * 
 * @note No lookup table - minimal memory footprint
 * @note Slower than table-driven crc_crc32()
 * @note Same polynomial and result as crc_crc32()
 * @note Use for systems with severe memory constraints
 */
uint32_t crc32_small(uint32_t crc, const uint8_t *buf, uint32_t size);

/**
 * @brief Calculate 24-bit CRC
 * 
 * @details Computes a 24-bit cyclic redundancy check providing intermediate
 *          strength error detection between CRC16 and CRC32. Used in specialized
 *          telemetry and communication protocols.
 * 
 * @param[in] bytes Pointer to data buffer
 * @param[in] len   Length of data buffer in bytes
 * 
 * @return 32-bit value containing 24-bit CRC
 * 
 * @note Provides better error detection than CRC16 with less overhead than CRC32
 * @note Used in specific telemetry protocols requiring 24-bit checksums
 */
uint32_t crc_crc24(const uint8_t *bytes, uint16_t len);

/**
 * @brief Calculate CRC16 using IBM/ANSI standard
 * 
 * @details Computes CRC16 using the IBM/ANSI polynomial (0x8005), also known as
 *          CRC-16-ANSI or CRC-16-IBM. This variant is used in various industrial
 *          protocols and legacy systems.
 * 
 * @param[in] crc_accum      Current CRC accumulator value (use 0 for initial)
 * @param[in] data_blk_ptr   Pointer to data block
 * @param[in] data_blk_size  Size of data block in bytes
 * 
 * @return 16-bit CRC value
 * 
 * @note IBM/ANSI polynomial: 0x8005 (x^16 + x^15 + x^2 + 1)
 * @note Different from CCITT/XMODEM variant
 * @note Used in Modbus RTU and other industrial protocols
 */
uint16_t crc_crc16_ibm(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);

/**
 * @brief Calculate checksum used by SPORT/FPort protocols
 * 
 * @details Implements the specific checksum algorithm used by FrSky SPORT and
 *          FPort telemetry protocols. For each byte, adds it to a 16-bit sum,
 *          then adds those two bytes together. Returns the complement of the
 *          final sum. This is not a true CRC but a simple additive checksum
 *          with carry handling.
 * 
 * @param[in] p   Pointer to data buffer
 * @param[in] len Length of data buffer in bytes
 * 
 * @return 8-bit checksum (complement of sum)
 * 
 * @note FrSky SPORT/FPort specific checksum algorithm
 * @note Not a cryptographically secure or standard CRC
 * @note Used in FrSky RC telemetry systems
 */
uint8_t crc_sum8_with_carry(const uint8_t *p, uint8_t len);

/**
 * @brief Calculate CRC16 using CCITT standard
 * 
 * @details Implements the CCITT CRC16 algorithm (polynomial 0x1021) widely used
 *          in communication protocols. This implementation is from Swift Navigation Inc.
 *          and is commonly used in GPS and navigation systems.
 * 
 * @param[in] buf Pointer to data buffer
 * @param[in] len Length of data buffer in bytes
 * @param[in] crc Initial CRC value (typically 0xFFFF for CCITT)
 * 
 * @return 16-bit CRC value
 * 
 * @note CCITT polynomial: 0x1021 (x^16 + x^12 + x^5 + 1)
 * @note Initial value typically 0xFFFF for standard CCITT
 * @note Used extensively in GPS protocols and MAVLink
 * @note Copyright (C) 2010 Swift Navigation Inc. (Fergus Noble <fergus@swift-nav.com>)
 */
uint16_t crc16_ccitt(const uint8_t *buf, uint32_t len, uint16_t crc);

/**
 * @brief Calculate CRC16 CCITT with output XOR
 * 
 * @details Extended CCITT CRC16 implementation with additional output XOR value
 *          for protocol-specific variants. Allows customization of the final
 *          CRC output transformation.
 * 
 * @param[in] buf Pointer to data buffer
 * @param[in] len Length of data buffer in bytes
 * @param[in] crc Initial CRC value
 * @param[in] out Value to XOR with final CRC (output transformation)
 * 
 * @return 16-bit CRC value (after output XOR)
 * 
 * @note Provides output XOR customization for protocol variants
 * @note Copyright (C) 2010 Swift Navigation Inc.
 */
uint16_t crc16_ccitt_r(const uint8_t *buf, uint32_t len, uint16_t crc, uint16_t out);

/**
 * @brief Calculate CRC16 CCITT using GDL90 non-standard variant
 * 
 * @details Implements the CRC16_CCITT algorithm using the GDL90 parser method,
 *          which is a non-standard variant of CCITT used in the FAA GDL90 protocol
 *          for ADS-B data link. The GDL90 protocol is used for aircraft traffic
 *          information and weather data.
 * 
 * @param[in] buf Pointer to data buffer
 * @param[in] len Length of data buffer in bytes
 * @param[in] crc Initial CRC value
 * 
 * @return 16-bit CRC value
 * 
 * @note Non-standard CCITT variant specific to GDL90 protocol
 * @note Used in FAA GDL90 ADS-B data link specification
 * @note Reference: https://www.faa.gov/nextgen/programs/adsb/archival/media/gdl90_public_icd_reva.pdf
 * @warning Not compatible with standard CCITT CRC16
 */
uint16_t crc16_ccitt_GDL90(const uint8_t *buf, uint32_t len, uint16_t crc);

/**
 * @brief Calculate CRC for Modbus protocol
 * 
 * @details Computes the CRC used in Modbus RTU (Remote Terminal Unit) protocol.
 *          Modbus uses a specific CRC16 variant with polynomial 0x8005 (IBM/ANSI)
 *          with initial value 0xFFFF and byte-reversed bit order.
 * 
 * @param[in] buf Pointer to data buffer
 * @param[in] len Length of data buffer in bytes
 * 
 * @return 16-bit Modbus CRC value
 * 
 * @note Modbus RTU polynomial: 0x8005 (reflected)
 * @note Initial value: 0xFFFF
 * @note Used in Modbus RTU industrial control protocol
 * @note Common in industrial sensors and actuators
 */
uint16_t calc_crc_modbus(const uint8_t *buf, uint16_t len);

/**
 * @brief Calculate Fletcher-16 checksum
 * 
 * @details Computes the Fletcher-16 checksum, a fast error-detection algorithm
 *          that provides better error detection than simple additive checksums
 *          while being faster to compute than CRCs. Fletcher checksums detect
 *          all single-bit errors, most double-bit errors, and many burst errors.
 *          
 *          The algorithm maintains two running sums: sum1 (sum of bytes) and
 *          sum2 (sum of sum1 values). This provides position-dependent error
 *          detection unlike simple additive checksums.
 * 
 * @param[in] buffer Pointer to data buffer
 * @param[in] len    Length of data buffer in bytes
 * 
 * @return 16-bit Fletcher checksum
 * 
 * @note Faster than CRC but provides weaker error detection
 * @note Detects all single-bit errors and most multi-bit errors
 * @note Used in network protocols where speed is critical
 * @note Position-dependent unlike simple additive checksums
 */
uint16_t crc_fletcher16(const uint8_t * buffer, uint32_t len);

/**
 * @brief FNV-1a offset basis constant for 64-bit hash
 * 
 * @details Initial hash value for FNV-1a 64-bit algorithm. This is a
 *          carefully chosen prime number that provides good hash distribution.
 */
#define FNV_1_OFFSET_BASIS_64 14695981039346656037UL

/**
 * @brief Generate 64-bit FNV-1a hash from buffer
 * 
 * @details Computes a 64-bit FNV-1a (Fowler-Noll-Vo) hash, a fast non-cryptographic
 *          hash function with good distribution properties. FNV-1a is designed to
 *          be fast while maintaining low collision rates for similar inputs.
 *          
 *          The algorithm XORs each byte with the hash value, then multiplies by
 *          the FNV prime (0x100000001b3). This provides good avalanche properties
 *          where small input changes result in large hash changes.
 * 
 * @param[in]  len  Length of data buffer in bytes
 * @param[in]  buf  Pointer to data buffer
 * @param[out] hash Pointer to 64-bit hash output (initialize to FNV_1_OFFSET_BASIS_64)
 * 
 * @note Non-cryptographic hash - NOT suitable for security applications
 * @note Fast computation with good distribution properties
 * @note Low collision rate for similar inputs
 * @note Used for data fingerprinting and hash tables
 * @note Initialize *hash to FNV_1_OFFSET_BASIS_64 before first call
 * @warning Not cryptographically secure - do not use for security validation
 */
void hash_fnv_1a(uint32_t len, const uint8_t* buf, uint64_t* hash);

/**
 * @brief Calculate CRC-64-WE (Weak-Error) using polynomial 0x42F0E1EBA9EA3693
 * 
 * @details Computes a 64-bit CRC using the CRC-64-WE polynomial, providing
 *          extended precision error detection for large data blocks. The 64-bit
 *          CRC provides extremely strong error detection suitable for large
 *          firmware images or extensive data storage validation.
 *          
 *          Polynomial: 0x42F0E1EBA9EA3693 (ECMA-182 standard)
 * 
 * @param[in] data      Pointer to array of 32-bit words
 * @param[in] num_words Number of 32-bit words in array
 * 
 * @return 64-bit CRC value
 * 
 * @note CRC-64-WE polynomial: 0x42F0E1EBA9EA3693 (ECMA-182)
 * @note Input is 32-bit words, not bytes
 * @note Used for large data block validation (firmware, logs)
 * @note Provides extremely strong error detection for big data
 * @warning Input buffer must be properly aligned for 32-bit word access
 */
uint64_t crc_crc64(const uint32_t *data, uint16_t num_words);

/**
 * @brief Calculate parity of a byte
 * 
 * @details Returns the parity bit for a byte: 1 if there is an odd number of
 *          bits set, 0 if there is an even number of bits set. Parity is a
 *          simple error detection mechanism used in serial communications.
 * 
 * @param[in] byte Input byte to calculate parity for
 * 
 * @return 1 if odd number of bits set, 0 if even number of bits set
 * 
 * @note Simple single-bit error detection
 * @note Used in serial communication parity checking
 * @note Can detect single-bit errors but not multiple-bit errors
 */
uint8_t parity(uint8_t byte);

/**
 * @brief Sum bytes in buffer modulo 256
 * 
 * @details Computes a simple additive checksum by summing all bytes in the
 *          buffer and returning the sum modulo 256 (i.e., wrapped to 8 bits).
 *          This is the simplest form of checksum, providing minimal error
 *          detection suitable only for detecting random errors.
 * 
 * @param[in] data  Pointer to data buffer
 * @param[in] count Number of bytes in buffer
 * 
 * @return 8-bit sum (sum mod 256)
 * 
 * @note Weakest error detection - detects only random errors
 * @note Does not detect byte reordering or complemented errors
 * @note Fast but provides minimal integrity checking
 * @note Use CRC for stronger error detection
 */
uint8_t crc_sum_of_bytes(const uint8_t *data, uint16_t count);

/**
 * @brief Sum bytes in buffer modulo 65536
 * 
 * @details Computes a simple additive checksum by summing all bytes in the
 *          buffer and returning the sum modulo 65536 (i.e., wrapped to 16 bits).
 *          Provides slightly better error detection than 8-bit sum but still
 *          relatively weak compared to CRC algorithms.
 * 
 * @param[in] data  Pointer to data buffer
 * @param[in] count Number of bytes in buffer
 * 
 * @return 16-bit sum (sum mod 65536)
 * 
 * @note Better than 8-bit sum but still weak error detection
 * @note Does not detect byte reordering or complemented errors
 * @note Use CRC for stronger error detection requirements
 */
uint16_t crc_sum_of_bytes_16(const uint8_t *data, uint16_t count);
