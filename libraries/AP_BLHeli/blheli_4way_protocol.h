/**
 * @file blheli_4way_protocol.h
 * @brief BLHeli 4-way protocol specification for ESC configuration and firmware flashing
 * 
 * @details This header defines the BLHeli 4-way protocol used for configuring and flashing
 *          BLHeli ESCs (Electronic Speed Controllers) via a 4-wire interface. The protocol
 *          enables reading/writing ESC flash memory, EEPROM access, and bootloader operations.
 *          
 *          **Wire Protocol Format:**
 *          
 *          Send Structure:
 *          - ESC + CMD + PARAM_LEN + [PARAMS] + CRC16_Hi + CRC16_Lo
 *          
 *          Return Structure:
 *          - ESC + CMD + PARAM_LEN + [PARAMS] + ACK + CRC16_Hi + CRC16_Lo
 *          
 *          **Protocol Features:**
 *          - Flash memory read/write operations
 *          - EEPROM read/write operations
 *          - Page erase and full chip erase
 *          - Multi-interface support (C2, SiLabs, Atmel, ARM bootloaders)
 *          - CRC16 error detection
 *          - ACK/NAK acknowledgment system
 *          
 * @note This is a header-only protocol specification file with no implementation code.
 * @warning During 4-way protocol operations, UART must have exclusive access to ESC.
 *          No other communication should occur on the interface during operations.
 * 
 * @see Based on serial_4way.c from betaflight project
 * @see https://github.com/betaflight/betaflight
 * 
 * Source: libraries/AP_BLHeli/blheli_4way_protocol.h
 */
/*
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 * Author: 4712
*/

/**
 * @defgroup BLHeli_Protocol_Commands BLHeli 4-Way Protocol Commands
 * @{
 */

/**
 * @brief Remote escape sequence character for protocol framing
 * @details Used to escape remote device commands in the protocol stream.
 *          ASCII character '.' (0x2E)
 */
#define cmd_Remote_Escape 0x2E // '.'

/**
 * @brief Local escape sequence character for protocol framing
 * @details Used to escape local interface commands in the protocol stream.
 *          ASCII character '/' (0x2F)
 */
#define cmd_Local_Escape  0x2F // '/'

/**
 * @brief Test if interface is still alive and responding
 * @details Sends a keepalive/ping command to verify the 4-way interface is operational.
 *          This should be used periodically during long operations to maintain connection.
 *          ASCII character '0' (0x30)
 * 
 * PARAMETERS: None
 * RETURN: ACK_OK on success
 * 
 * @note Used for connection health monitoring
 */
#define cmd_InterfaceTestAlive 0x30 // '0' alive

/**
 * @brief Get protocol version number
 * @details Retrieves the 4-way protocol version implemented by the interface.
 *          Protocol version is used to ensure compatibility between ground station
 *          and interface implementation.
 *          ASCII character '1' (0x31)
 * 
 * PARAMETERS: None
 * RETURN: uint8_t VersionNumber (1..255) + ACK_OK
 * 
 * @note Current protocol version is defined by SERIAL_4WAY_PROTOCOL_VER
 */
#define cmd_ProtocolGetVersion 0x31  // '1' version

/**
 * @brief Get interface name string
 * @details Retrieves a human-readable string identifying the 4-way interface implementation.
 *          Typically returns interface type and version information.
 *          ASCII character '2' (0x32)
 * 
 * PARAMETERS: None
 * RETURN: String (null-terminated) + ACK_OK
 * 
 * @note String length varies by implementation
 */
#define cmd_InterfaceGetName 0x32 // '2' name

/**
 * @brief Get interface version number
 * @details Retrieves the interface firmware version number (separate from protocol version).
 *          Used to identify specific interface implementation versions.
 *          ASCII character '3' (0x33)
 * 
 * PARAMETERS: None
 * RETURN: uint8_t VersionNumber (1..255) + ACK_OK
 * 
 * @note Version format defined by SERIAL_4WAY_VERSION
 */
#define cmd_InterfaceGetVersion 0x33  // '3' version

/**
 * @brief Exit 4-way interface mode
 * @details Terminates 4-way protocol session and returns interface to normal operating mode.
 *          Can be used to switch back to flight controller mode after ESC configuration.
 *          ASCII character '4' (0x34)
 * 
 * PARAMETERS: None
 * RETURN: ACK_OK on successful exit
 * 
 * @warning After this command, interface will no longer respond to 4-way protocol commands
 */
#define cmd_InterfaceExit 0x34       // '4' exit

/**
 * @brief Reset the connected ESC device
 * @details Issues a hardware reset to the ESC connected to the interface.
 *          ESC will restart and return to normal operating mode.
 *          ASCII character '5' (0x35)
 * 
 * PARAMETERS: None
 * RETURN: ACK_OK after reset completed
 * 
 * @note Device may take time to restart after reset
 */
#define cmd_DeviceReset 0x35        // '5' reset

/**
 * @brief Get Device ID (REMOVED - deprecated since version 06/106)
 * @details This command was used to retrieve the unique device identifier of connected ESC.
 *          Removed from protocol specification in version 06/106.
 *          ASCII character '6' (0x36)
 * 
 * @deprecated Removed since protocol version 06/106
 */
// #define cmd_DeviceGetID 0x36      //'6' device id removed since 06/106
// RETURN: uint8_t DeviceID + ACK

/**
 * @brief Initialize flash memory access on connected ESC
 * @details Prepares the ESC flash memory for read/write/erase operations.
 *          Must be called before any flash memory operations.
 *          ASCII character '7' (0x37)
 * 
 * PARAMETERS: None
 * RETURN: ACK_OK on successful initialization, ACK_D_GENERAL_ERROR on failure
 * 
 * @note Required before cmd_DeviceRead, cmd_DeviceWrite, cmd_DevicePageErase, or cmd_DeviceEraseAll
 * @warning Flash operations require stable power supply to prevent corruption
 */
#define cmd_DeviceInitFlash 0x37    // '7' init flash access

/**
 * @brief Erase entire flash memory of connected ESC
 * @details Performs a full chip erase of the ESC flash memory.
 *          This operation may take several seconds to complete.
 *          ASCII character '8' (0x38)
 * 
 * PARAMETERS: None
 * RETURN: ACK_OK after erase completes
 * 
 * @warning This will erase all firmware and configuration on the ESC. Device will be unusable until new firmware is flashed.
 * @note Requires cmd_DeviceInitFlash to be called first
 */
#define cmd_DeviceEraseAll 0x38     // '8' erase all

/**
 * @brief Erase single page of flash memory
 * @details Erases one page of flash memory. Page size is device-dependent (typically 512 or 1024 bytes).
 *          More efficient than full chip erase when updating partial firmware.
 *          ASCII character '9' (0x39)
 * 
 * PARAMETERS:
 * - uint8_t PageNumber: Flash page number to erase (0 to device max pages)
 * 
 * RETURN: ACK_OK after page erase completes, ACK_I_INVALID_PARAM if page number invalid
 * 
 * @note Page size varies by ESC chip type (SiLabs, Atmel, ARM)
 * @note Requires cmd_DeviceInitFlash to be called first
 */
#define cmd_DevicePageErase 0x39    // '9' page erase

/**
 * @brief Read flash memory from connected ESC
 * @details Reads a block of flash memory data from ESC into buffer.
 *          Maximum buffer length is 256 bytes per transaction.
 *          ASCII character ':' (0x3A)
 * 
 * PARAMETERS:
 * - uint8_t ADDRESS_Hi: High byte of 16-bit start address
 * - uint8_t ADDRESS_Lo: Low byte of 16-bit start address
 * - uint8_t BuffLen: Number of bytes to read (0 means 256 bytes)
 * 
 * RETURN:
 * - uint8_t ADDRESS_Hi: High byte of address (echo)
 * - uint8_t ADDRESS_Lo: Low byte of address (echo)
 * - uint8_t BuffLen: Number of bytes read
 * - uint8_t Buffer[0..255]: Read data
 * - ACK_OK on success
 * 
 * @note BuffLen=0 is special case meaning 256 bytes
 * @note Requires cmd_DeviceInitFlash to be called first
 */
#define cmd_DeviceRead 0x3A  // ':' read Device

/**
 * @brief Write flash memory on connected ESC
 * @details Writes a block of data to ESC flash memory.
 *          Maximum buffer length is 256 bytes per transaction.
 *          Flash page must be erased before writing.
 *          ASCII character ';' (0x3B)
 * 
 * PARAMETERS:
 * - uint8_t ADDRESS_Hi: High byte of 16-bit start address
 * - uint8_t ADDRESS_Lo: Low byte of 16-bit start address
 * - uint8_t BuffLen: Number of bytes to write (0 means 256 bytes)
 * - uint8_t Buffer[0..255]: Data to write
 * 
 * RETURN: ACK_OK on successful write, ACK_D_GENERAL_ERROR on write failure
 * 
 * @note BuffLen=0 is special case meaning 256 bytes
 * @note Flash must be erased (0xFF) before writing
 * @note Requires cmd_DeviceInitFlash to be called first
 * @warning Ensure stable power during write operations to prevent corruption
 */
#define cmd_DeviceWrite 0x3B    // ';' write

/**
 * @brief Set C2CK line low (permanent reset state)
 * @details Forces C2 clock line low, holding device in reset state indefinitely.
 *          Used for C2 interface debugging or forced reset scenarios.
 *          ASCII character '<' (0x3C)
 * 
 * PARAMETERS: None
 * RETURN: ACK_OK
 * 
 * @note Only applicable to C2 interface mode (imC2)
 * @warning Device remains in reset until C2CK released or power cycled
 */
#define cmd_DeviceC2CK_LOW 0x3C // '<'

/**
 * @brief Read EEPROM memory from connected ESC
 * @details Reads a block of EEPROM data from ESC configuration storage.
 *          Maximum buffer length is 256 bytes per transaction.
 *          ASCII character '=' (0x3D)
 * 
 * PARAMETERS:
 * - uint8_t ADDRESS_Hi: High byte of 16-bit start address
 * - uint8_t ADDRESS_Lo: Low byte of 16-bit start address
 * - uint8_t BuffLen: Number of bytes to read (0 means 256 bytes)
 * 
 * RETURN:
 * - uint8_t ADDRESS_Hi: High byte of address (echo)
 * - uint8_t ADDRESS_Lo: Low byte of address (echo)
 * - uint8_t BuffLen: Number of bytes read
 * - uint8_t Buffer[0..255]: Read data
 * - ACK_OK on success
 * 
 * @note BuffLen=0 is special case meaning 256 bytes
 * @note EEPROM typically stores ESC configuration parameters
 */
#define cmd_DeviceReadEEprom 0x3D  // '=' read Device

/**
 * @brief Write EEPROM memory on connected ESC
 * @details Writes a block of data to ESC EEPROM configuration storage.
 *          Maximum buffer length is 256 bytes per transaction.
 *          ASCII character '>' (0x3E)
 * 
 * PARAMETERS:
 * - uint8_t ADDRESS_Hi: High byte of 16-bit start address
 * - uint8_t ADDRESS_Lo: Low byte of 16-bit start address
 * - uint8_t BuffLen: Number of bytes to write (0 means 256 bytes)
 * - uint8_t Buffer[0..255]: Data to write
 * 
 * RETURN: ACK_OK on successful write, ACK_D_GENERAL_ERROR on write failure
 * 
 * @note BuffLen=0 is special case meaning 256 bytes
 * @note EEPROM stores ESC configuration parameters (motor timing, startup power, etc.)
 */
#define cmd_DeviceWriteEEprom 0x3E  // '>' write

/**
 * @brief Set interface communication mode
 * @details Configures the physical interface protocol used to communicate with ESC bootloader.
 *          Different ESC chips require different interface modes.
 *          ASCII character '?' (0x3F)
 * 
 * PARAMETERS:
 * - uint8_t Mode: Interface mode to use
 *   - imC2 (0): Silicon Labs C2 interface
 *   - imSIL_BLB (1): Silicon Labs bootloader
 *   - imATM_BLB (2): Atmel bootloader
 *   - imSK (3): SimonK bootloader
 *   - imARM_BLB (4): ARM bootloader
 * 
 * RETURN: ACK_OK on successful mode change, ACK_I_INVALID_CHANNEL if mode not supported
 * 
 * @note Interface mode must match ESC chip type
 * @warning Incorrect mode selection will prevent communication with ESC
 */
#define cmd_InterfaceSetMode 0x3F   // '?'

/**
 * @brief Verify flash memory contents
 * @details Compares buffer contents with flash memory to verify successful write.
 *          Used after flashing firmware to ensure data integrity.
 *          Maximum buffer length is 256 bytes per transaction.
 *          ASCII character '@' (0x40)
 * 
 * PARAMETERS:
 * - uint8_t ADDRESS_Hi: High byte of 16-bit start address
 * - uint8_t ADDRESS_Lo: Low byte of 16-bit start address
 * - uint8_t BuffLen: Number of bytes to verify (0 means 256 bytes)
 * - uint8_t Buffer[0..255]: Expected data for comparison
 * 
 * RETURN: ACK_OK if data matches, ACK_I_VERIFY_ERROR if mismatch detected
 * 
 * @note BuffLen=0 is special case meaning 256 bytes
 * @note Essential for ensuring firmware integrity after flashing
 */
#define cmd_DeviceVerify 0x40   //'@' write

/** @} */ // End of BLHeli_Protocol_Commands group

/**
 * @defgroup BLHeli_Version_Macros BLHeli 4-Way Interface Version Information
 * @{
 */

/**
 * @brief Major version number of 4-way interface implementation
 * @details Main version component (XX.yy.zz format)
 * @note Maximum value is 24 due to uint8_t limitation in sub-version calculation
 */
#define SERIAL_4WAY_VER_MAIN 20

/**
 * @brief First sub-version number (minor version)
 * @details Sub-version component (xx.YY.zz format)
 */
#define SERIAL_4WAY_VER_SUB_1 (uint8_t) 0

/**
 * @brief Second sub-version number (patch version)
 * @details Patch level component (xx.yy.ZZ format)
 */
#define SERIAL_4WAY_VER_SUB_2 (uint8_t) 05

/**
 * @brief Protocol version number
 * @details Version of the 4-way communication protocol specification.
 *          Used by cmd_ProtocolGetVersion to report protocol compatibility.
 *          Current version: 107
 */
#define SERIAL_4WAY_PROTOCOL_VER 107

#if (SERIAL_4WAY_VER_MAIN > 24)
#error "beware of SERIAL_4WAY_VER_SUB_1 is uint8_t"
#endif

/**
 * @brief Combined version number as 16-bit integer
 * @details Calculated as: (MAIN * 1000) + (SUB_1 * 100) + SUB_2
 *          Example: Version 20.0.5 = 20005
 */
#define SERIAL_4WAY_VERSION (uint16_t) ((SERIAL_4WAY_VER_MAIN * 1000) + (SERIAL_4WAY_VER_SUB_1 * 100) + SERIAL_4WAY_VER_SUB_2)

/**
 * @brief High byte of version number for transmission
 * @details Upper 8 bits of version number (SERIAL_4WAY_VERSION / 100)
 */
#define SERIAL_4WAY_VERSION_HI (uint8_t) (SERIAL_4WAY_VERSION / 100)

/**
 * @brief Low byte of version number for transmission
 * @details Lower 8 bits of version number (SERIAL_4WAY_VERSION % 100)
 */
#define SERIAL_4WAY_VERSION_LO (uint8_t) (SERIAL_4WAY_VERSION % 100)

/** @} */ // End of BLHeli_Version_Macros group

/**
 * @defgroup BLHeli_Bootloader_Results BLHeli Bootloader Operation Result Codes
 * @{
 */

/**
 * @brief Bootloader operation successful
 * @details Returned by ESC bootloader when operation completes successfully
 */
#define brSUCCESS           0x30

/**
 * @brief Bootloader verification error
 * @details Returned when flash verification fails (data mismatch after write)
 */
#define brERRORVERIFY       0xC0

/**
 * @brief Bootloader invalid command error
 * @details Returned when bootloader receives unrecognized or invalid command
 */
#define brERRORCOMMAND      0xC1

/**
 * @brief Bootloader CRC error
 * @details Returned when CRC16 check fails on received data packet
 */
#define brERRORCRC          0xC2

/**
 * @brief No bootloader result available
 * @details Initial/default state indicating no operation result yet
 */
#define brNONE              0xFF

/** @} */ // End of BLHeli_Bootloader_Results group

/**
 * @defgroup BLHeli_Bootloader_Subcommands BLHeli Bootloader Subcommands
 * @brief Low-level bootloader commands used during flash operations
 * @{
 */

/**
 * @brief Exit bootloader and run application
 * @details Commands ESC bootloader to exit and start running main application firmware
 */
#define CMD_RUN             0x00

/**
 * @brief Program (write) flash memory
 * @details Bootloader command to write data to flash memory
 */
#define CMD_PROG_FLASH      0x01

/**
 * @brief Erase flash memory
 * @details Bootloader command to erase flash memory region
 */
#define CMD_ERASE_FLASH     0x02

/**
 * @brief Read flash memory (Silicon Labs)
 * @details Bootloader command to read flash on SiLabs MCUs
 */
#define CMD_READ_FLASH_SIL  0x03

/**
 * @brief Verify flash memory contents
 * @details Bootloader command to verify written flash data
 * @note Same opcode as CMD_READ_FLASH_SIL (0x03) - context dependent
 */
#define CMD_VERIFY_FLASH    0x03

/**
 * @brief Verify flash memory (ARM)
 * @details Bootloader command to verify flash on ARM MCUs
 */
#define CMD_VERIFY_FLASH_ARM 0x04

/**
 * @brief Read EEPROM memory
 * @details Bootloader command to read EEPROM configuration storage
 * @note Same opcode as CMD_VERIFY_FLASH_ARM (0x04) - context dependent
 */
#define CMD_READ_EEPROM     0x04

/**
 * @brief Program (write) EEPROM memory
 * @details Bootloader command to write EEPROM configuration data
 */
#define CMD_PROG_EEPROM     0x05

/**
 * @brief Read SRAM memory
 * @details Bootloader command to read device SRAM (volatile memory)
 */
#define CMD_READ_SRAM       0x06

/**
 * @brief Read flash memory (Atmel)
 * @details Bootloader command to read flash on Atmel MCUs
 */
#define CMD_READ_FLASH_ATM  0x07

/**
 * @brief Keepalive command for bootloader
 * @details Prevents bootloader timeout during long operations
 */
#define CMD_KEEP_ALIVE      0xFD

/**
 * @brief Set buffer data for bootloader operations
 * @details Pre-loads data buffer before flash/EEPROM write operations
 */
#define CMD_SET_BUFFER      0xFE

/**
 * @brief Set memory address for bootloader operations
 * @details Sets target address pointer for subsequent read/write operations
 */
#define CMD_SET_ADDRESS     0xFF

/**
 * @brief Restart bootloader mode
 * @details Parameter value to restart bootloader (used with certain commands)
 */
#define RestartBootloader   0

/**
 * @brief Exit bootloader mode
 * @details Parameter value to exit bootloader and run application
 */
#define ExitBootloader      1

/** @} */ // End of BLHeli_Bootloader_Subcommands group

/**
 * @defgroup BLHeli_ACK_Codes BLHeli Protocol Acknowledgment Codes
 * @brief Response codes returned in protocol ACK field
 * @details Every protocol command returns an ACK byte indicating success or specific error type.
 *          ACK codes starting with ACK_I_ indicate interface errors.
 *          ACK codes starting with ACK_D_ indicate device errors.
 * @{
 */

/**
 * @brief Operation successful
 * @details Command completed successfully with no errors
 */
#define ACK_OK                  0x00

/**
 * @brief Invalid command error (interface)
 * @details Interface received unrecognized or unsupported command opcode
 */
#define ACK_I_INVALID_CMD       0x02

/**
 * @brief CRC error (interface)
 * @details CRC16 checksum validation failed on received packet
 * @note Indicates data corruption during transmission
 */
#define ACK_I_INVALID_CRC       0x03

/**
 * @brief Verification error (interface)
 * @details Flash verification failed - written data does not match expected
 * @warning Indicates potentially corrupted firmware flash
 */
#define ACK_I_VERIFY_ERROR      0x04

/**
 * @brief Invalid channel/mode error (interface)
 * @details Requested interface mode or ESC channel is not supported or available
 * @note Returned by cmd_InterfaceSetMode for unsupported modes
 */
#define ACK_I_INVALID_CHANNEL   0x08

/**
 * @brief Invalid parameter error (interface)
 * @details Command parameter value is out of valid range or incorrect
 * @note Examples: invalid page number, invalid address, invalid buffer length
 */
#define ACK_I_INVALID_PARAM     0x09

/**
 * @brief General device error
 * @details ESC device reported an error during operation
 * @note Covers various device-level failures (flash write fail, EEPROM error, etc.)
 */
#define ACK_D_GENERAL_ERROR     0x0F

/** @} */ // End of BLHeli_ACK_Codes group

/**
 * @defgroup BLHeli_Interface_Modes BLHeli 4-Way Interface Communication Modes
 * @brief Physical interface protocols for different ESC chip types
 * @details Different ESC microcontrollers use different bootloader protocols.
 *          The interface mode must be set to match the connected ESC chip type
 *          using the cmd_InterfaceSetMode command.
 * @{
 */

/**
 * @brief Silicon Labs C2 interface mode
 * @details Two-wire C2 protocol for Silicon Labs C8051 microcontrollers.
 *          Uses C2CK (clock) and C2D (data) lines for direct debug interface access.
 * @note Used for ESCs with SiLabs C8051Fxxx chips
 */
#define imC2 0

/**
 * @brief Silicon Labs bootloader mode
 * @details Communicates with built-in bootloader on Silicon Labs microcontrollers.
 *          Bootloader protocol for firmware flashing via UART.
 * @note Used for ESCs with SiLabs bootloader-enabled chips
 */
#define imSIL_BLB 1

/**
 * @brief Atmel bootloader mode
 * @details Communicates with built-in bootloader on Atmel AVR microcontrollers.
 *          Bootloader protocol for firmware flashing.
 * @note Used for ESCs with Atmel ATmega chips (common in older BLHeli versions)
 */
#define imATM_BLB 2

/**
 * @brief SimonK bootloader mode
 * @details Communicates with SimonK bootloader protocol (open-source ESC firmware).
 *          Alternative bootloader for Atmel chips.
 * @note Used for ESCs running SimonK firmware
 */
#define imSK 3

/**
 * @brief ARM bootloader mode
 * @details Communicates with bootloader on ARM Cortex-M microcontrollers.
 *          Used for newer 32-bit ESCs.
 * @note Used for ESCs with ARM chips (STM32, etc.)
 */
#define imARM_BLB 4

/** @} */ // End of BLHeli_Interface_Modes group
