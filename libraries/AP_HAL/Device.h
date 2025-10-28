/*
 * Copyright (C) 2015-2016  Intel Corporation. All rights reserved.
 *
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
 */

/**
 * @file Device.h
 * @brief Unified device abstraction for I2C, SPI, and other bus-connected peripherals
 * 
 * @details Defines the abstract base interface common to all bus-connected devices 
 *          (I2C, SPI, UAVCAN, MSP, Serial). Provides platform-independent device 
 *          operations including data transfer, register access, periodic callbacks,
 *          and automatic register integrity verification.
 *          
 *          This interface is implemented by platform-specific HAL drivers:
 *          - AP_HAL::I2CDevice for I2C peripherals
 *          - AP_HAL::SPIDevice for SPI peripherals
 *          - Device wrappers for UAVCAN, MSP, and other bus types
 *          
 *          Key abstractions:
 *          - Bus-agnostic transfer operations
 *          - Unique device identification (bus ID encoding)
 *          - Thread-safe access via semaphores
 *          - Background I/O via periodic callbacks
 *          - Register integrity checking for SEU/EMI protection
 * 
 * Source: libraries/AP_HAL/Device.h
 */
#pragma once

#include <inttypes.h>

#include "AP_HAL_Namespace.h"
#include "utility/functor.h"
#include "AP_HAL_Boards.h"

#if CONFIG_HAL_BOARD != HAL_BOARD_QURT
// we need utility for std::move, but not on QURT due to a include error in hexagon SDK
#include <utility>
#endif

/**
 * @class AP_HAL::Device
 * @brief Abstract base class for bus-connected peripheral devices
 * 
 * @details Provides common functionality for devices regardless of bus type:
 *          
 *          **Bus Identification:**
 *          - Query bus type (I2C, SPI, UAVCAN, etc.)
 *          - Unique 24-bit device ID encoding bus type, number, address, and device type
 *          - Device IDs can be serialized over MAVLink without loss of information
 *          
 *          **Periodic Callbacks:**
 *          - Register background polling/sampling tasks on dedicated I/O thread
 *          - Callbacks execute with bus lock already held
 *          - Typical usage: sensor polling at fixed rates (1kHz IMU, 50Hz baro, etc.)
 *          
 *          **Checked Register System:**
 *          - Automatic verification of critical configuration registers
 *          - Protection against Single Event Upsets (SEU) and EMI corruption
 *          - Amortized checking: verify one register per call to avoid bus saturation
 *          - Automatic repair of corrupted registers to configured values
 *          
 *          **Thread Safety:**
 *          - All operations should be protected by bus semaphore
 *          - Use get_semaphore() and WITH_SEMAPHORE() macro for safe access
 *          - Periodic callbacks run with semaphore already taken
 *          
 *          **Typical Usage Pattern:**
 *          ```cpp
 *          // Driver initialization
 *          bool MyDriver::init() {
 *              // 1. Detect device and configure
 *              if (!_dev->read_registers(REG_WHO_AM_I, &id, 1)) {
 *                  return false;
 *              }
 *              
 *              // 2. Write configuration registers
 *              _dev->write_register(REG_CONFIG, config_val);
 *              
 *              // 3. Setup register checking for critical config
 *              _dev->setup_checked_registers(3);  // Monitor 3 registers
 *              _dev->set_checked_register(REG_CONFIG, config_val);
 *              _dev->set_checked_register(REG_CTRL1, ctrl1_val);
 *              _dev->set_checked_register(REG_CTRL2, ctrl2_val);
 *              
 *              // 4. Register periodic callback for sampling
 *              _dev->register_periodic_callback(1000,  // 1ms = 1kHz
 *                  FUNCTOR_BIND_MEMBER(&MyDriver::_timer, void));
 *              
 *              return true;
 *          }
 *          
 *          // Periodic sampling (runs on I/O thread)
 *          void MyDriver::_timer() {
 *              // Check one register per callback
 *              _dev->check_next_register();
 *              
 *              // Read sensor data
 *              uint8_t data[6];
 *              if (_dev->read_registers(REG_DATA, data, sizeof(data))) {
 *                  // Process data...
 *              }
 *          }
 *          ```
 *          
 *          **Bus ID Encoding (24 bits):**
 *          - Bits [0-2]:   Bus type (BusType enum)
 *          - Bits [3-7]:   Bus number (0-31)
 *          - Bits [8-15]:  Device address (I2C addr or SPI CS pin)
 *          - Bits [16-23]: Device type (sensor-specific)
 * 
 * @note All device operations should check semaphore availability
 * @note Periodic callbacks run on I/O thread, not main thread
 * @note Callbacks must complete quickly to avoid delaying other devices
 * 
 * @warning Checked registers increase bus traffic - use only for critical config
 * @warning Long-running callbacks delay all devices on same I/O thread
 * @warning Forgetting semaphore causes bus conflicts and data corruption
 * 
 * @see AP_HAL::I2CDevice
 * @see AP_HAL::SPIDevice
 * 
 * Source: libraries/AP_HAL/Device.h
 */
class AP_HAL::Device {
public:
    /**
     * @enum BusType
     * @brief Bus type identification for device instances
     * 
     * @details Used in device ID encoding to identify which physical bus
     *          the device is connected to. This allows the system to route
     *          operations to the correct bus driver implementation.
     *          
     *          Bus type is encoded in the lower 3 bits of the device ID,
     *          limiting the maximum to 8 distinct bus types.
     */
    enum BusType {
        BUS_TYPE_UNKNOWN = 0,  ///< Uninitialized or invalid bus type
        BUS_TYPE_I2C     = 1,  ///< I2C/I²C/TWI serial bus
        BUS_TYPE_SPI     = 2,  ///< SPI (Serial Peripheral Interface) bus
        BUS_TYPE_UAVCAN  = 3,  ///< UAVCAN/DroneCAN network
        BUS_TYPE_SITL    = 4,  ///< Software-in-the-loop simulation
        BUS_TYPE_MSP     = 5,  ///< MultiWii Serial Protocol device
        BUS_TYPE_SERIAL  = 6,  ///< Generic UART/serial device
        BUS_TYPE_WSPI    = 7,  ///< Wide SPI (Quad/Octo SPI)
    };

    /**
     * @enum Speed
     * @brief Bus speed selection for device transfers
     * 
     * @details Platform and bus-specific speed settings. Actual frequencies
     *          depend on HAL implementation:
     *          - I2C: Typically 100kHz (low) or 400kHz (high)
     *          - SPI: Typically 1MHz (low) or 8-20MHz (high)
     *          
     *          Speed may be shared across all devices on the same bus
     *          depending on platform capabilities.
     */
    enum Speed {
        SPEED_HIGH,  ///< High-speed transfers (sensor data, high-rate polling)
        SPEED_LOW,   ///< Low-speed transfers (configuration, infrequent access)
    };

    /**
     * @struct CommandHeader
     * @brief Command structure for Wide SPI (Quad/Octo SPI) transfers
     * 
     * @details Used for devices that support enhanced SPI modes with multiple
     *          data lines (Quad SPI, Octo SPI). Allows specifying command,
     *          address, alternate bytes, and dummy cycles for complex protocols
     *          like QSPI flash memory or high-speed sensors.
     *          
     *          Standard SPI devices can ignore this structure.
     * 
     * @note Only relevant for BUS_TYPE_WSPI devices
     * @see set_cmd_header()
     */
    struct CommandHeader {
        uint32_t  cmd;    ///< Command phase data (instruction byte)
        uint32_t  cfg;    ///< Transfer configuration field (bus width, phases)
        uint32_t  addr;   ///< Address phase data (memory/register address)
        uint32_t  alt;    ///< Alternate phase data (mode bits)
        uint32_t  dummy;  ///< Number of dummy cycles to be inserted
    };

    /**
     * @typedef PeriodicCb
     * @brief Callback functor for periodic device operations
     * 
     * @details Function signature: void callback(void)
     *          
     *          Callback executes on dedicated I/O thread at specified interval.
     *          Must complete quickly to avoid delaying other devices.
     *          Bus semaphore is already held when callback executes.
     * 
     * @see register_periodic_callback()
     */
    FUNCTOR_TYPEDEF(PeriodicCb, void);
    
    /**
     * @typedef PeriodicHandle
     * @brief Opaque handle for registered periodic callback
     * 
     * @details Returned by register_periodic_callback() and used to adjust
     *          or unregister the callback later.
     */
    typedef void* PeriodicHandle;

    /**
     * @typedef RegisterRWCb
     * @brief Callback functor for register read/write notifications
     * 
     * @details Function signature: void callback(uint8_t reg_addr, 
     *                                           uint8_t* data, 
     *                                           uint32_t data_len, 
     *                                           bool is_write)
     *          
     *          Called before each register read or write operation.
     *          Allows monitoring or modifying register access patterns.
     *          
     * @param[in] reg_addr    Register address being accessed
     * @param[in,out] data    Pointer to data buffer
     * @param[in] data_len    Length of data in bytes
     * @param[in] is_write    true for write operation, false for read
     * 
     * @see set_register_rw_callback()
     */
    FUNCTOR_TYPEDEF(RegisterRWCb, void, uint8_t, uint8_t*, uint32_t, bool);
    
    /**
     * @typedef RegisterRWHandle
     * @brief Opaque handle for register read/write callback
     */
    typedef void* RegisterRWHandle;

    /**
     * @typedef BankSelectCb
     * @brief Callback functor for bank selection on multi-bank devices
     * 
     * @details Function signature: bool callback(uint8_t bank)
     *          
     *          Some sensors have multiple register banks. This callback
     *          switches to the specified bank before register access.
     *          
     * @param[in] bank  Bank number to select (0-255)
     * @return true if bank selection succeeded, false on error
     * 
     * @see setup_bankselect_callback()
     * @see transfer_bank()
     */
    FUNCTOR_TYPEDEF(BankSelectCb, bool, uint8_t);

    /**
     * @brief Construct a device instance with specified bus type
     * 
     * @param[in] type Bus type enum identifying physical bus connection
     * 
     * @note Subclasses (I2CDevice, SPIDevice) call this constructor
     * @note Bus number and device address must be set separately
     */
    Device(enum BusType type)
    {
        _bus_id.devid_s.bus_type = type;
    }

    /**
     * @brief Get the bus type for this device
     * 
     * @return BusType enum identifying the physical bus (I2C, SPI, etc.)
     * 
     * @note Useful for device drivers that need bus-specific behavior
     */
    enum BusType bus_type(void) const {
        return _bus_id.devid_s.bus_type;
    }

    /**
     * @brief Get the bus number/instance for this device
     * 
     * @return Bus number (0-31) identifying which instance of the bus type
     * 
     * @details For example, returns 1 for I2C1 or 2 for SPI2.
     *          Useful for diagnostics and logging.
     */
    uint8_t bus_num(void) const {
        return _bus_id.devid_s.bus;
    }

    /**
     * @brief Get the unique 24-bit bus identifier for this device
     * 
     * @return 32-bit value with device ID in lower 24 bits
     * 
     * @details Bus ID encoding (lower 24 bits):
     *          - Bits [0-2]:   Bus type (BusType enum)
     *          - Bits [3-7]:   Bus number (0-31)
     *          - Bits [8-15]:  Device address (I2C addr or SPI CS pin)
     *          - Bits [16-23]: Device type (sensor-specific)
     *          
     *          This ID uniquely identifies a device instance and can be
     *          serialized over MAVLink for device identification and logging.
     *          
     * @note Upper 8 bits are unused (always 0)
     * @note Two devices with same ID are considered identical instances
     * 
     * @see make_bus_id()
     * @see get_bus_id_devtype()
     */
    uint32_t get_bus_id(void) const {
        return _bus_id.devid;
    }

    /**
     * @brief Get the device address on the bus
     * 
     * @return Device address (I2C 7-bit address or SPI CS pin number)
     * 
     * @details For I2C: Returns 7-bit slave address (0-127)
     *          For SPI: Returns chip select pin number
     *          For other buses: Bus-specific addressing
     */
    uint8_t get_bus_address(void) const {
        return _bus_id.devid_s.address;
    }

    /**
     * @brief Set the device type identifier
     * 
     * @param[in] devtype Device type code from sensor-specific enumeration
     * 
     * @details Device type distinguishes between different sensor models on
     *          the same bus. For example, AP_COMPASS_TYPE_LSM303D vs
     *          AP_COMPASS_TYPE_HMC5843.
     *          
     *          Device type appears in:
     *          - Device bus ID for unique identification
     *          - Log messages for debugging
     *          - Health monitoring and diagnostics
     * 
     * @note Each sensor subsystem defines its own device type enum
     * @note Must be called during device initialization
     */
    void set_device_type(uint8_t devtype);

    /**
     * @brief Virtual destructor to ensure proper cleanup of derived classes
     * 
     * @details Frees checked register array if allocated
     */
    virtual ~Device() {
        delete[] _checked.regs;
    }

    /**
     * @brief Change the I2C device address
     * 
     * @param[in] address New 7-bit I2C slave address (0-127)
     * 
     * @details Note that this is the 7-bit address without the R/W bit.
     *          For example, an address of 0x68 should be passed as 0x68,
     *          not 0xD0 or 0xD1.
     *          
     * @note Only works on I2C devices - SPI and other buses ignore this
     * @note Default implementation does nothing - I2C subclass overrides
     */
    virtual void set_address(uint8_t address) {};
    
    /**
     * @brief Set the bus speed for future transfers
     * 
     * @param[in] speed Speed setting (SPEED_HIGH or SPEED_LOW)
     * 
     * @return true if speed was successfully set, false on failure
     * @return true if platform doesn't implement speed control (success)
     * 
     * @details Actual bus frequencies depend on HAL implementation:
     *          - I2C: Typically 100kHz (low) or 400kHz (high)
     *          - SPI: Typically 1MHz (low) or 8-20MHz (high)
     *          
     *          Speed setting may be shared across all devices on the same
     *          bus depending on hardware and HAL implementation. Last call
     *          to set_speed() may affect all devices on that bus.
     * 
     * @note Must be called before register_periodic_callback() for effect
     * @note Pure virtual - must be implemented by derived classes
     * 
     * @warning Changing speed after initialization may disrupt other devices
     */
    virtual bool set_speed(Speed speed)  = 0;

    /**
     * @brief Perform a half-duplex bus transfer (write then read)
     * 
     * @param[in]  send     Pointer to data to transmit (or nullptr for read-only)
     * @param[in]  send_len Number of bytes to transmit
     * @param[out] recv     Pointer to buffer for received data (or nullptr for write-only)
     * @param[in]  recv_len Number of bytes to receive
     * 
     * @return true on successful transfer, false on bus error or timeout
     * 
     * @details Performs a single bus transaction in two phases:
     *          1. Transmit send_len bytes from send buffer
     *          2. Receive recv_len bytes into recv buffer
     *          
     *          Operation is half-duplex regardless of bus type:
     *          - I2C: Natural half-duplex operation with repeated start
     *          - SPI: CS asserted, write phase, then read phase, CS deasserted
     *          
     *          Common patterns:
     *          - Write-only: transfer(data, len, nullptr, 0)
     *          - Read-only:  transfer(nullptr, 0, data, len)
     *          - Register read: transfer(&reg, 1, data, len)
     * 
     * @note Pure virtual - must be implemented by derived classes
     * @note Bus semaphore should be held before calling
     * @note For full-duplex SPI, use transfer_fullduplex() instead
     * 
     * @warning Do not call from interrupt context
     * @warning Ensure buffers remain valid for duration of transfer
     * 
     * @see transfer_fullduplex()
     * @see read_registers()
     */
    virtual bool transfer(const uint8_t *send, uint32_t send_len,
                          uint8_t *recv, uint32_t recv_len) = 0;

    /**
     * @brief Perform a full-duplex bus transfer (simultaneous send/receive)
     * 
     * @param[in,out] send_recv Pointer to buffer containing data to send;
     *                          receives incoming data simultaneously
     * @param[in]     len       Number of bytes to transfer in each direction
     * 
     * @return true on successful transfer, false on bus error or timeout
     * 
     * @details Performs simultaneous send and receive (where supported):
     *          - SPI: Full-duplex transfer with MOSI and MISO active simultaneously
     *          - I2C: Falls back to half-duplex (calls transfer())
     *          
     *          The same buffer is used for both send and receive data.
     *          Transmitted data is overwritten with received data.
     *          
     *          Some HAL implementations use DMA optimization for this method.
     * 
     * @note Default implementation calls transfer(send_recv, len, send_recv, len)
     * @note SPI implementations may override for true full-duplex operation
     * @note Bus semaphore should be held before calling
     * 
     * @warning Buffer contents are modified during transfer
     * @warning Do not call from interrupt context
     * 
     * @see transfer()
     */
    virtual bool transfer_fullduplex(uint8_t *send_recv, uint32_t len) {
        return transfer(send_recv, len, send_recv, len);
    }

    /**
     * @brief Configure Wide SPI command header for next transfer
     * 
     * @param[in] cmd_hdr Command header structure with WSPI parameters
     * 
     * @details Sets command, address, alternate bytes, and dummy cycles
     *          for Wide SPI protocols (Quad SPI, Octo SPI).
     *          
     *          Must be called before each transfer on WSPI devices.
     *          Standard SPI devices ignore this call.
     * 
     * @note Only relevant for BUS_TYPE_WSPI devices
     * @note Default implementation does nothing
     * 
     * @see CommandHeader
     */
    virtual void set_cmd_header(const CommandHeader& cmd_hdr) {}

    /**
     * @brief Enter execute-in-place (XIP) mode for memory-mapped access
     * 
     * @param[out] map_ptr Receives pointer to memory-mapped region
     * 
     * @return true if XIP mode enabled successfully, false if unsupported
     * 
     * @details XIP mode allows direct CPU access to external memory (like
     *          QSPI flash) without explicit transfer operations. The device
     *          is mapped into CPU address space.
     *          
     *          Only supported by Wide SPI flash memory devices on platforms
     *          with memory-mapped QSPI controllers.
     * 
     * @note Default implementation returns false (not supported)
     * @note Must call exit_xip_mode() before normal transfers resume
     * 
     * @see exit_xip_mode()
     */
    virtual bool enter_xip_mode(void** map_ptr) { return false; }
    
    /**
     * @brief Exit execute-in-place (XIP) mode
     * 
     * @return true if XIP mode exited successfully, false if unsupported
     * 
     * @details Returns device to normal SPI transfer mode after XIP.
     * 
     * @note Default implementation returns false (not supported)
     * 
     * @see enter_xip_mode()
     */
    virtual bool exit_xip_mode() { return false; }

    /**
     * @brief Read multiple consecutive registers from device
     * 
     * @param[in]  first_reg Starting register address
     * @param[out] recv      Buffer to receive register data
     * @param[in]  recv_len  Number of bytes (registers) to read
     * 
     * @return true on successful read, false on failure
     * 
     * @details Wrapper over transfer() that reads consecutive registers
     *          starting at first_reg. For devices that require a read flag,
     *          the value set by set_read_flag() is ORed with first_reg.
     *          
     *          Typical usage:
     *          ```cpp
     *          uint8_t data[6];
     *          if (dev->read_registers(REG_ACCEL_X, data, 6)) {
     *              // Process 6-byte accelerometer data
     *          }
     *          ```
     *          
     *          For multi-bank devices, use read_bank_registers() instead.
     * 
     * @note Automatically applies read flag if set via set_read_flag()
     * @note Bus semaphore should be held before calling
     * 
     * @see write_register()
     * @see set_read_flag()
     * @see read_bank_registers()
     * 
     * Source: libraries/AP_HAL/Device.h:163
     */
    bool read_registers(uint8_t first_reg, uint8_t *recv, uint32_t recv_len);

    /**
     * @brief Write a single byte to a device register
     * 
     * @param[in] reg     Register address to write
     * @param[in] val     Byte value to write
     * @param[in] checked If true, add to checked register list for verification
     * 
     * @return true on successful write, false on failure
     * 
     * @details Wrapper over transfer() that writes one byte to specified register.
     *          Transfer sequence: [reg, val]
     *          
     *          If checked=true, the register is added to the checked register
     *          list for automatic verification and corruption recovery.
     *          
     *          Typical usage:
     *          ```cpp
     *          // Write configuration register
     *          dev->write_register(REG_CONFIG, 0x38);
     *          
     *          // Write critical register with checking enabled
     *          dev->write_register(REG_CTRL1, 0x4F, true);
     *          ```
     * 
     * @note Bus semaphore should be held before calling
     * @note For multi-bank devices, use write_bank_register() instead
     * 
     * @see read_registers()
     * @see set_checked_register()
     * @see write_bank_register()
     * 
     * Source: libraries/AP_HAL/Device.h:171
     */
    bool write_register(uint8_t reg, uint8_t val, bool checked=false);
    
    /**
     * @brief Set callback to monitor register read/write operations
     * 
     * @param[in] register_rw_callback Functor to call on each register access
     * 
     * @details The callback is invoked before each register read or write
     *          operation with the register address, data buffer, length,
     *          and operation direction.
     *          
     *          Useful for:
     *          - Debugging register access patterns
     *          - Monitoring configuration changes
     *          - Implementing custom register access logging
     * 
     * @note Callback should execute quickly to avoid delaying transfers
     * @note Default implementation stores callback for derived class use
     * 
     * @see RegisterRWCb
     */
    virtual void set_register_rw_callback(RegisterRWCb register_rw_callback) {
        _register_rw_callback = register_rw_callback;
    }

    /**
     * @brief Perform bus transfer on multi-bank device with bank selection
     * 
     * @param[in]  bank     Register bank number to select
     * @param[in]  send     Pointer to data to transmit
     * @param[in]  send_len Number of bytes to transmit
     * @param[out] recv     Pointer to buffer for received data
     * @param[in]  recv_len Number of bytes to receive
     * 
     * @return true on successful transfer, false on failure
     * 
     * @details Some sensors (e.g., ICM-20948) have multiple register banks.
     *          This method:
     *          1. Calls bank selection callback to switch to specified bank
     *          2. Performs the transfer operation
     *          
     *          Bank selection callback must be registered first via
     *          setup_bankselect_callback().
     * 
     * @note Bank selection adds overhead - use regular transfer() if possible
     * @note Bus semaphore should be held before calling
     * 
     * @see setup_bankselect_callback()
     * @see read_bank_registers()
     * @see write_bank_register()
     * 
     * Source: libraries/AP_HAL/Device.h:186
     */
    bool transfer_bank(uint8_t bank, const uint8_t *send, uint32_t send_len,
                          uint8_t *recv, uint32_t recv_len);

    /**
     * @brief Read multiple consecutive registers from multi-bank device
     * 
     * @param[in]  bank      Register bank number to select
     * @param[in]  first_reg Starting register address within bank
     * @param[out] recv      Buffer to receive register data
     * @param[in]  recv_len  Number of bytes (registers) to read
     * 
     * @return true on successful read, false on failure
     * 
     * @details Wrapper over transfer_bank() that reads consecutive registers
     *          from specified bank. Automatically handles bank selection.
     *          
     *          Read flag set by set_read_flag() is ORed with first_reg.
     * 
     * @note Requires bank selection callback registered via setup_bankselect_callback()
     * @note Bus semaphore should be held before calling
     * 
     * @see read_registers()
     * @see write_bank_register()
     * 
     * Source: libraries/AP_HAL/Device.h:197
     */
    bool read_bank_registers(uint8_t bank, uint8_t first_reg, uint8_t *recv, uint32_t recv_len);

    /**
     * @brief Write a single byte to a register in multi-bank device
     * 
     * @param[in] bank    Register bank number to select
     * @param[in] reg     Register address within bank
     * @param[in] val     Byte value to write
     * @param[in] checked If true, add to checked register list
     * 
     * @return true on successful write, false on failure
     * 
     * @details Wrapper over transfer_bank() that writes one byte to specified
     *          register in specified bank. Automatically handles bank selection.
     *          
     *          If checked=true, register is added to checked register list
     *          with bank information for automatic verification.
     * 
     * @note Requires bank selection callback registered via setup_bankselect_callback()
     * @note Bus semaphore should be held before calling
     * 
     * @see write_register()
     * @see read_bank_registers()
     * 
     * Source: libraries/AP_HAL/Device.h:205
     */
    bool write_bank_register(uint8_t bank, uint8_t reg, uint8_t val, bool checked=false);

    /**
     * @brief Set expected value for a checked register in multi-bank device
     * 
     * @param[in] bank Bank number containing the register
     * @param[in] reg  Register address within bank
     * @param[in] val  Expected register value
     * 
     * @details Adds register to checked register list with bank information.
     *          The register will be periodically verified and automatically
     *          corrected if corrupted.
     *          
     *          Must call setup_checked_registers() first to allocate space.
     * 
     * @note For multi-bank devices like ICM-20948
     * @note Does not immediately write the value - use write_bank_register() first
     * 
     * @see setup_checked_registers()
     * @see check_next_register()
     * @see set_checked_register(uint8_t, uint8_t)
     * 
     * Source: libraries/AP_HAL/Device.h:210
     */
    void set_checked_register(uint8_t bank, uint8_t reg, uint8_t val);

    /**
     * @brief Set expected value for a checked register (single-bank device)
     * 
     * @param[in] reg Register address
     * @param[in] val Expected register value
     * 
     * @details Adds register to checked register list. The register will be
     *          periodically verified and automatically corrected if corrupted.
     *          
     *          Must call setup_checked_registers() first to allocate space.
     * 
     * @note For single-bank devices - multi-bank devices use other overload
     * @note Does not immediately write the value - use write_register() first
     * 
     * @see setup_checked_registers()
     * @see check_next_register()
     * @see set_checked_register(uint8_t, uint8_t, uint8_t)
     * 
     * Source: libraries/AP_HAL/Device.h:215
     */
    void set_checked_register(uint8_t reg, uint8_t val);

    /**
     * @brief Initialize automatic register checking system
     * 
     * @param[in] num_regs  Maximum number of registers to check
     * @param[in] frequency Check frequency divider (default 10)
     * 
     * @return true if setup successful, false on memory allocation failure
     * 
     * @details Sets up the checked register system for automatic verification
     *          of critical configuration registers. Protects against:
     *          - Single Event Upsets (SEU) from cosmic rays
     *          - EMI-induced register corruption
     *          - Bus glitches corrupting configuration
     *          
     *          Frequency parameter controls checking rate:
     *          - frequency=1:  Check one register every call
     *          - frequency=10: Check one register every 10th call (default)
     *          
     *          After setup, use set_checked_register() to add registers
     *          to the monitoring list, then call check_next_register()
     *          periodically (typically in sensor polling callback).
     *          
     *          Checking is amortized across calls to avoid bus saturation.
     *          With frequency=10 and 3 monitored registers, each register
     *          is verified every 30 calls.
     *          
     *          Example usage:
     *          ```cpp
     *          // In init():
     *          dev->setup_checked_registers(3, 10);  // Monitor 3 registers
     *          dev->write_register(REG_CONFIG, 0x38);
     *          dev->set_checked_register(REG_CONFIG, 0x38);
     *          
     *          // In periodic callback:
     *          dev->check_next_register();  // Verify one register
     *          ```
     * 
     * @note Must be called before set_checked_register()
     * @note Allocates memory for register list
     * @note Higher frequency = more bus overhead but faster corruption detection
     * 
     * @warning Each check adds bus transaction overhead
     * @warning Balance between safety and bus bandwidth
     * 
     * @see set_checked_register()
     * @see check_next_register()
     * 
     * Source: libraries/AP_HAL/Device.h:221
     */
    bool setup_checked_registers(uint8_t num_regs, uint8_t frequency=10);

    /**
     * @brief Verify next register in checked register list
     * 
     * @return true if register value correct or checking skipped (frequency divider)
     * @return false if register value incorrect or checking not setup
     * 
     * @details Checks one register from the monitored list and automatically
     *          repairs it if corrupted. Call this periodically (typically in
     *          sensor polling callback) to maintain register integrity.
     *          
     *          Checking sequence:
     *          1. Increments internal counter
     *          2. If counter reaches frequency threshold, reads next register
     *          3. Compares read value to expected value
     *          4. If mismatch, writes expected value to repair corruption
     *          5. Returns false if corruption detected
     *          
     *          The method cycles through all registered checks in round-robin
     *          fashion, so each register is verified periodically.
     *          
     *          Example usage:
     *          ```cpp
     *          void MyDriver::_timer() {
     *              // Check one register per callback (amortized overhead)
     *              if (!_dev->check_next_register()) {
     *                  // Register corruption detected and repaired
     *                  corruption_count++;
     *              }
     *              
     *              // Read sensor data
     *              _dev->read_registers(REG_DATA, data, sizeof(data));
     *          }
     *          ```
     * 
     * @note Must call setup_checked_registers() first
     * @note Automatically repairs corrupted registers
     * @note Checking is throttled by frequency parameter from setup
     * 
     * @see setup_checked_registers()
     * @see set_checked_register()
     * @see check_next_register(struct checkreg&)
     * 
     * Source: libraries/AP_HAL/Device.h:227
     */
    bool check_next_register(void);

    /**
     * @struct checkreg
     * @brief Checked register descriptor
     * 
     * @details Stores information about a single register being monitored
     *          for automatic corruption detection and repair.
     */
    struct checkreg {
        uint8_t bank;    ///< Register bank number (0 for single-bank devices)
        uint8_t regnum;  ///< Register address within bank
        uint8_t value;   ///< Expected register value
    };
    
    /**
     * @brief Verify next register with detailed failure information
     * 
     * @param[out] fail Receives information about failed register check
     * 
     * @return true if register value correct or checking skipped
     * @return false if register value incorrect, with details in fail parameter
     * 
     * @details Same as check_next_register() but returns information about
     *          which register failed and what value was read.
     *          
     *          Useful for debugging and logging register corruption events.
     *          
     *          Example usage:
     *          ```cpp
     *          struct AP_HAL::Device::checkreg failed;
     *          if (!_dev->check_next_register(failed)) {
     *              GCS_SEND_TEXT(MAV_SEVERITY_WARNING, 
     *                  "Sensor register 0x%02X corrupted", failed.regnum);
     *          }
     *          ```
     * 
     * @note Automatically repairs corrupted register before returning
     * @note fail parameter only valid when return value is false
     * 
     * @see check_next_register()
     * @see checkreg
     * 
     * Source: libraries/AP_HAL/Device.h:241
     */
    bool check_next_register(struct checkreg &fail);
    
    /**
     * @brief Read raw bytes from device without register addressing
     * 
     * @param[out] recv     Buffer to receive data
     * @param[in]  recv_len Number of bytes to read
     * 
     * @return true on successful read, false on failure
     * 
     * @details Wrapper over transfer() that performs a pure read operation
     *          without writing any address or command bytes first.
     *          
     *          Unlike read_registers(), this does NOT:
     *          - Send a register address
     *          - Apply the read flag from set_read_flag()
     *          
     *          Useful for:
     *          - Devices with auto-incrementing read pointers
     *          - Streaming data protocols
     *          - Devices where register addressing is handled elsewhere
     * 
     * @note Bus semaphore should be held before calling
     * 
     * @see transfer()
     * @see read_registers()
     * 
     * Source: libraries/AP_HAL/Device.h:248
     */
    bool read(uint8_t *recv, uint32_t recv_len)
    {
        return transfer(nullptr, 0, recv, recv_len);
    }

    /**
     * @brief Get the bus semaphore for thread-safe access
     * 
     * @return Pointer to bus semaphore, or nullptr if not available
     * 
     * @details Returns the semaphore that protects this device's bus from
     *          concurrent access by multiple threads. The semaphore must be
     *          taken before any bus operations to prevent race conditions.
     *          
     *          Typical usage patterns:
     *          
     *          **Initialization (manual locking):**
     *          ```cpp
     *          bool init() {
     *              AP_HAL::Semaphore *sem = _dev->get_semaphore();
     *              if (!sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
     *                  return false;
     *              }
     *              
     *              // Perform initialization transfers
     *              _dev->write_register(REG_CONFIG, value);
     *              
     *              sem->give();
     *              return true;
     *          }
     *          ```
     *          
     *          **Periodic callback (automatic locking):**
     *          ```cpp
     *          // Periodic callbacks run with semaphore already held
     *          void _timer() {
     *              // No need to take semaphore - already locked
     *              _dev->read_registers(REG_DATA, data, len);
     *          }
     *          ```
     *          
     *          **Using WITH_SEMAPHORE macro (recommended):**
     *          ```cpp
     *          void update() {
     *              WITH_SEMAPHORE(_dev->get_semaphore());
     *              // Semaphore automatically released on scope exit
     *              _dev->transfer(send, send_len, recv, recv_len);
     *          }
     *          ```
     * 
     * @note Pure virtual - must be implemented by derived classes
     * @note Periodic callbacks registered via register_periodic_callback()
     *       execute with semaphore already taken
     * @note Intended primarily for initialization phase; prefer periodic
     *       callbacks for regular device polling
     * 
     * @warning Never nest semaphore locks - causes deadlock
     * @warning Must release semaphore promptly to avoid blocking other devices
     * @warning Forgetting semaphore causes bus conflicts and data corruption
     * 
     * @see register_periodic_callback()
     * @see WITH_SEMAPHORE()
     * 
     * Source: libraries/AP_HAL/Device.h:257
     */
    virtual AP_HAL::Semaphore *get_semaphore() = 0;

    /**
     * @brief Register a function to be called periodically on I/O thread
     * 
     * @param[in] period_usec Period between calls in microseconds
     * @param[in] cb          Callback functor (use FUNCTOR_BIND_MEMBER)
     * 
     * @return Handle for this callback, or nullptr on failure
     * 
     * @details Registers a function to be called at fixed intervals on a
     *          dedicated I/O thread. This is the preferred method for regular
     *          device polling and sensor sampling.
     *          
     *          **Callback Execution Context:**
     *          - Runs on dedicated per-bus I/O thread (NOT main thread)
     *          - Bus semaphore is already taken when callback executes
     *          - All callbacks on same bus run sequentially on same thread
     *          - Callback should complete within ~50% of period to avoid jitter
     *          
     *          **Typical Periods:**
     *          - Fast IMU: 1000μs (1kHz)
     *          - Medium rate (baro, compass): 10000-20000μs (50-100Hz)
     *          - Slow devices (GPS): 100000μs (10Hz)
     *          
     *          **Usage Pattern:**
     *          ```cpp
     *          class MyDriver {
     *              AP_HAL::Device *_dev;
     *              
     *              bool init() {
     *                  // Register 1kHz polling callback
     *                  _dev->register_periodic_callback(1000, 
     *                      FUNCTOR_BIND_MEMBER(&MyDriver::_timer, void));
     *                  return true;
     *              }
     *              
     *              // Runs on I/O thread at 1kHz
     *              void _timer() {
     *                  // Semaphore already held
     *                  _dev->check_next_register();
     *                  
     *                  uint8_t data[6];
     *                  if (_dev->read_registers(REG_DATA, data, 6)) {
     *                      // Process sensor data
     *                      _publish_data(data);
     *                  }
     *              }
     *          };
     *          ```
     *          
     *          **After Registration:**
     *          After registering the callback, avoid calling device methods
     *          from other threads unless you take the semaphore first.
     *          The periodic callback becomes the primary device interaction.
     * 
     * @note Pure virtual - must be implemented by derived classes
     * @note Callback can return false to self-cancel
     * @note Multiple callbacks per device are allowed (rare)
     * @note Bus semaphore is already held - do NOT take it again
     * 
     * @warning Callback must complete quickly (within ~50% of period)
     * @warning Long callbacks delay ALL devices on the same bus
     * @warning Do NOT block or sleep in callback
     * @warning Do NOT call functions that might take a long time
     * 
     * @see adjust_periodic_callback()
     * @see unregister_callback()
     * @see PeriodicCb
     * @see PeriodicHandle
     * 
     * Source: libraries/AP_HAL/Device.h:273
     */
    virtual PeriodicHandle register_periodic_callback(uint32_t period_usec, PeriodicCb) = 0;

    /**
     * @brief Adjust the period of a registered periodic callback
     * 
     * @param[in] h           Handle returned by register_periodic_callback()
     * @param[in] period_usec New period in microseconds
     * 
     * @return true if period successfully adjusted, false on failure
     * 
     * @details Changes the callback period without unregistering and
     *          re-registering. The new period takes effect immediately,
     *          resetting the timer from the current moment.
     *          
     *          Useful for:
     *          - Adaptive sampling rates based on motion detection
     *          - Power saving by reducing polling during idle periods
     *          - Temporarily increasing rate during calibration
     *          
     *          Example:
     *          ```cpp
     *          // Slow down polling when vehicle is disarmed
     *          if (!hal.util->get_soft_armed()) {
     *              _dev->adjust_periodic_callback(_handle, 10000);  // 100Hz
     *          } else {
     *              _dev->adjust_periodic_callback(_handle, 1000);   // 1kHz
     *          }
     *          ```
     * 
     * @note Pure virtual - must be implemented by derived classes
     * @note Timer resets from moment of call, not from last callback
     * 
     * @see register_periodic_callback()
     * 
     * Source: libraries/AP_HAL/Device.h:282
     */
    virtual bool adjust_periodic_callback(PeriodicHandle h, uint32_t period_usec) = 0;

    /**
     * @brief Unregister and cancel a periodic callback
     * 
     * @param[in] h Handle returned by register_periodic_callback()
     * 
     * @return true if successfully unregistered, false on failure
     * 
     * @details Stops the periodic callback from being called. After this
     *          call, the handle is no longer valid.
     *          
     *          Alternative cancellation: Callback can return false to
     *          self-cancel without needing to call this method.
     *          
     *          Typical usage:
     *          ```cpp
     *          // During driver shutdown
     *          if (_periodic_handle != nullptr) {
     *              _dev->unregister_callback(_periodic_handle);
     *              _periodic_handle = nullptr;
     *          }
     *          ```
     * 
     * @note Default implementation returns false (not supported)
     * @note Callback may execute one more time during unregistration
     * 
     * @see register_periodic_callback()
     * 
     * Source: libraries/AP_HAL/Device.h:290
     */
    virtual bool unregister_callback(PeriodicHandle h) { return false; }

    /**
     * @brief Register callback for multi-bank device register access
     * 
     * @param[in] bank_select Functor to call for bank switching
     * 
     * @details Some devices (e.g., ICM-20948, ICM-42688) have multiple
     *          register banks. This callback is invoked to switch between
     *          banks before register access.
     *          
     *          The callback should:
     *          1. Write the bank select register
     *          2. Return true on success, false on failure
     *          
     *          Example implementation:
     *          ```cpp
     *          bool select_bank(uint8_t bank) {
     *              return _dev->write_register(REG_BANK_SEL, bank << 4);
     *          }
     *          
     *          // During init:
     *          _dev->setup_bankselect_callback(
     *              FUNCTOR_BIND_MEMBER(&MyDriver::select_bank, bool, uint8_t));
     *          ```
     *          
     *          Once registered, use:
     *          - transfer_bank() instead of transfer()
     *          - read_bank_registers() instead of read_registers()
     *          - write_bank_register() instead of write_register()
     * 
     * @note Required for checked registers on multi-bank devices
     * @note Default implementation stores callback for derived class use
     * 
     * @see BankSelectCb
     * @see transfer_bank()
     * @see deregister_bankselect_callback()
     * 
     * Source: libraries/AP_HAL/Device.h:295
     */
    virtual void setup_bankselect_callback(BankSelectCb bank_select) {
        _bank_select = bank_select;
    }

    /**
     * @brief Remove the bank selection callback
     * 
     * @details Clears the bank selection callback, reverting to single-bank
     *          operation mode.
     * 
     * @see setup_bankselect_callback()
     * 
     * Source: libraries/AP_HAL/Device.h:302
     */
    virtual void deregister_bankselect_callback() {
        _bank_select = nullptr;
    }


    /**
     * @brief Register callback for DMA transfer completion notification
     * 
     * @param[in] proc Member function to call when DMA completes
     * 
     * @details Enables asynchronous DMA transfers on platforms that support
     *          it. When a completion callback is registered:
     *          
     *          1. Transfer methods return immediately after starting DMA
     *          2. Callback is invoked from interrupt context when DMA completes
     *          3. Bus semaphore must remain held until callback executes
     *          4. Callback must call register_completion_callback(nullptr) to
     *             indicate it's safe to release the semaphore
     *          
     *          This is an advanced feature for high-performance sensor drivers
     *          that need to overlap computation with bus transfers.
     *          
     *          Example:
     *          ```cpp
     *          void _dma_complete() {
     *              // Process DMA data
     *              process_sensor_data(_dma_buffer);
     *              
     *              // Clear completion callback
     *              _dev->register_completion_callback((AP_HAL::MemberProc)nullptr);
     *              
     *              // Now safe to release semaphore
     *          }
     *          
     *          void _timer() {
     *              // Register completion handler
     *              _dev->register_completion_callback(
     *                  MEMBERPROC_BIND(&MyDriver::_dma_complete));
     *              
     *              // Start async transfer (returns immediately if DMA available)
     *              _dev->transfer(nullptr, 0, _dma_buffer, len);
     *              
     *              // Do other work while DMA in progress...
     *              // Semaphore still held, will be released after callback
     *          }
     *          ```
     * 
     * @note Default implementation does nothing (DMA not supported)
     * @note Only some HAL implementations support DMA transfers
     * @note Callback executes in interrupt context - keep it short
     * @note Must clear callback to indicate completion
     * 
     * @warning Experimental feature - not all platforms support this
     * @warning Bus semaphore timing is critical with async transfers
     * 
     * Source: libraries/AP_HAL/Device.h:312
     */
    virtual void register_completion_callback(AP_HAL::MemberProc proc) {}
    
    /**
     * @brief Register callback for DMA transfer completion (function pointer)
     * 
     * @param[in] proc Function pointer to call when DMA completes
     * 
     * @details Same as register_completion_callback(AP_HAL::MemberProc) but
     *          accepts a plain function pointer instead of member function.
     * 
     * @see register_completion_callback(AP_HAL::MemberProc)
     * 
     * Source: libraries/AP_HAL/Device.h:313
     */
    virtual void register_completion_callback(AP_HAL::Proc proc) {}
    
    /**
     * @brief Manually control SPI chip select signal
     * 
     * @param[in] set true to assert CS (active low), false to deassert
     * 
     * @return true if CS control successful, false if not supported
     * 
     * @details Provides direct control over SPI chip select for devices with
     *          unusual timing requirements, such as:
     *          - Specific delays between CS assertion and first clock
     *          - Holding CS across multiple logical transfers
     *          - Custom CS pulsing patterns
     *          
     *          Normal SPI transfers handle CS automatically - this is only
     *          needed for non-standard protocols.
     *          
     *          Example:
     *          ```cpp
     *          // Custom transfer with inter-byte delay
     *          _dev->set_chip_select(true);   // Assert CS
     *          _dev->transfer(&cmd, 1, nullptr, 0);
     *          hal.scheduler->delay_microseconds(50);  // Required delay
     *          _dev->transfer(nullptr, 0, &response, 1);
     *          _dev->set_chip_select(false);  // Deassert CS
     *          ```
     * 
     * @note Only works for SPI devices
     * @note Default implementation returns false (not supported)
     * @note Must manually manage CS for entire transaction
     * 
     * @warning Incorrect CS timing can damage devices
     * @warning Must ensure CS is eventually deasserted
     * 
     * Source: libraries/AP_HAL/Device.h:320
     */
    virtual bool set_chip_select(bool set) { return false; }

    /**
     * @brief Set read flag bit to be ORed with register addresses
     * 
     * @param[in] flag Bit(s) to OR with register address on reads
     * 
     * @details Some devices require a specific bit set in the register
     *          address byte to indicate a read operation (vs write).
     *          
     *          Common patterns:
     *          - Bit 7 set for read: flag = 0x80 (many SPI sensors)
     *          - Bit 6 for multi-byte: flag = 0x40 (some I2C sensors)
     *          
     *          The flag is automatically applied by read_registers() and
     *          read_bank_registers().
     *          
     *          Example:
     *          ```cpp
     *          // Device datasheet: "Set bit 7 of register address for reads"
     *          _dev->set_read_flag(0x80);
     *          
     *          // This actually sends 0x80 | 0x28 = 0xA8
     *          _dev->read_registers(0x28, data, 6);
     *          ```
     * 
     * @note Default value is 0x00 (no flag)
     * @note Only affects read_registers(), not raw transfer()
     * 
     * @see read_registers()
     * @see read_bank_registers()
     * 
     * Source: libraries/AP_HAL/Device.h:327
     */
    void set_read_flag(uint8_t flag);

    /**
     * @brief Construct a bus ID from individual components
     * 
     * @param[in] bus_type Bus type enum
     * @param[in] bus      Bus number (0-31)
     * @param[in] address  Device address on bus
     * @param[in] devtype  Device type code
     * 
     * @return 32-bit bus ID with components encoded in lower 24 bits
     * 
     * @details Creates a bus ID from individual fields. This is primarily
     *          used by devices that don't go through standard HAL device
     *          initialization, such as UAVCAN/DroneCAN devices.
     *          
     *          Encoding:
     *          - Bits [0-2]:   Bus type
     *          - Bits [3-7]:   Bus number
     *          - Bits [8-15]:  Address
     *          - Bits [16-23]: Device type
     *          
     *          Example:
     *          ```cpp
     *          // Create bus ID for UAVCAN compass
     *          uint32_t bus_id = AP_HAL::Device::make_bus_id(
     *              AP_HAL::Device::BUS_TYPE_UAVCAN,
     *              0,  // First UAVCAN bus
     *              node_id,
     *              AP_COMPASS_TYPE_UAVCAN);
     *          ```
     * 
     * @note Standard HAL devices (I2C, SPI) construct this automatically
     * 
     * @see get_bus_id()
     * @see change_bus_id()
     * 
     * Source: libraries/AP_HAL/Device.h:334
     */
    static uint32_t make_bus_id(enum BusType bus_type, uint8_t bus, uint8_t address, uint8_t devtype);

    /**
     * @brief Create new bus ID with different device type
     * 
     * @param[in] old_id  Existing bus ID
     * @param[in] devtype New device type code
     * 
     * @return New bus ID with same bus info but new device type
     * 
     * @details Useful for auxiliary devices on the same bus connection,
     *          such as a magnetometer integrated into an IMU chip.
     *          The new device shares the same bus/address but has a
     *          different device type for identification.
     *          
     *          Example:
     *          ```cpp
     *          // IMU has integrated magnetometer
     *          uint32_t imu_id = _imu_dev->get_bus_id();
     *          uint32_t mag_id = AP_HAL::Device::change_bus_id(
     *              imu_id, AP_COMPASS_TYPE_ICM20948);
     *          ```
     * 
     * @see make_bus_id()
     * @see get_bus_id_devtype()
     * 
     * Source: libraries/AP_HAL/Device.h:340
     */
    static uint32_t change_bus_id(uint32_t old_id, uint8_t devtype);

    /**
     * @brief Get bus ID with different device type
     * 
     * @param[in] devtype New device type code
     * 
     * @return New bus ID for this device with specified device type
     * 
     * @details Non-static version of change_bus_id() that operates on
     *          the current device's bus ID.
     * 
     * @see change_bus_id()
     * @see get_bus_id()
     * 
     * Source: libraries/AP_HAL/Device.h:345
     */
    uint32_t get_bus_id_devtype(uint8_t devtype) const;

    /**
     * @brief Extract bus type from bus ID
     * 
     * @param[in] dev_id Bus ID to decode
     * 
     * @return BusType enum from bits [0-2]
     * 
     * @details Utility function to decode the bus type field from a bus ID.
     * 
     * @see make_bus_id()
     * 
     * Source: libraries/AP_HAL/Device.h:350
     */
    static enum BusType devid_get_bus_type(uint32_t dev_id);

    /**
     * @brief Extract bus number from bus ID
     * 
     * @param[in] dev_id Bus ID to decode
     * 
     * @return Bus number (0-31) from bits [3-7]
     * 
     * @see make_bus_id()
     * 
     * Source: libraries/AP_HAL/Device.h:352
     */
    static uint8_t devid_get_bus(uint32_t dev_id);

    /**
     * @brief Extract device address from bus ID
     * 
     * @param[in] dev_id Bus ID to decode
     * 
     * @return Device address from bits [8-15]
     * 
     * @see make_bus_id()
     * 
     * Source: libraries/AP_HAL/Device.h:354
     */
    static uint8_t devid_get_address(uint32_t dev_id);

    /**
     * @brief Extract device type from bus ID
     * 
     * @param[in] dev_id Bus ID to decode
     * 
     * @return Device type code from bits [16-23]
     * 
     * @see make_bus_id()
     * 
     * Source: libraries/AP_HAL/Device.h:356
     */
    static uint8_t devid_get_devtype(uint32_t dev_id);


    /**
     * @brief Set number of automatic retries for failed transfers
     * 
     * @param[in] retries Number of times to retry on transfer failure (0-255)
     * 
     * @details Configures automatic retry behavior for bus transfers that
     *          fail due to transient errors (timeouts, NACK, etc.).
     *          
     *          Retries improve robustness against:
     *          - Electrical noise causing bus errors
     *          - Device not ready for communication
     *          - Temporary bus contention
     *          
     *          Each retry adds latency, so balance reliability with timing:
     *          - High-rate sensors: 1-2 retries
     *          - Slow devices: 3-5 retries
     *          - Critical operations: More retries acceptable
     * 
     * @note Default implementation does nothing (no retry support)
     * @note Actual retry behavior depends on HAL implementation
     * 
     * Source: libraries/AP_HAL/Device.h:360
     */
    virtual void set_retries(uint8_t retries) {};

protected:
    /**
     * @brief Read flag to OR with register address on read operations
     * 
     * @details Set via set_read_flag() and applied by read_registers().
     *          Default value is 0x00 (no flag).
     * 
     * @see set_read_flag()
     */
    uint8_t _read_flag = 0;

    /**
     * @struct DeviceStructure
     * @brief Bitfield structure for device ID components
     * 
     * @details Breaks out device ID into individual fields using bitfields.
     *          Bitfields keep the total size to 24 bits, which fits in a
     *          float without loss of precision. This allows device IDs to
     *          be transported over MAVLink parameter protocol.
     *          
     *          Total size: 24 bits (3 + 5 + 8 + 8)
     */
    struct DeviceStructure {
        enum BusType bus_type : 3;  ///< Bus type (0-7), 3 bits
        uint8_t bus: 5;              ///< Bus number (0-31), 5 bits
        uint8_t address;             ///< Device address (0-255), 8 bits
        uint8_t devtype;             ///< Device type code (0-255), 8 bits
    };

    /**
     * @union DeviceId
     * @brief Union for accessing device ID as bitfields or integer
     * 
     * @details Allows treating device ID as either:
     *          - Structured bitfields (devid_s) for field access
     *          - 32-bit integer (devid) for storage and comparison
     *          
     *          Only lower 24 bits are used; upper 8 bits are always 0.
     */
    union DeviceId {
        struct DeviceStructure devid_s;  ///< Bitfield structure view
        uint32_t devid;                   ///< Integer view (lower 24 bits valid)
    };

    /**
     * @brief Device identifier combining bus and device information
     * 
     * @details Stores the complete device identification in 24 bits.
     *          Initialized by constructor and set methods.
     */
    union DeviceId _bus_id;

    /**
     * @brief Set device address field in bus ID
     * 
     * @param[in] address Device address (I2C 7-bit addr or SPI CS pin)
     * 
     * @note Called by derived classes during initialization
     */
    void set_device_address(uint8_t address) {
        _bus_id.devid_s.address = address;
    }

    /**
     * @brief Set bus number field in bus ID
     * 
     * @param[in] bus Bus instance number (0-31)
     * 
     * @note Called by derived classes during initialization
     */
    void set_device_bus(uint8_t bus) {
        _bus_id.devid_s.bus = bus;
    }

private:
    /**
     * @brief Bank selection callback for multi-bank devices
     * 
     * @details Functor that switches register banks on devices with
     *          multiple register banks. Set via setup_bankselect_callback().
     *          nullptr if not configured.
     * 
     * @see setup_bankselect_callback()
     */
    BankSelectCb _bank_select;
    
    /**
     * @brief Register read/write monitoring callback
     * 
     * @details Functor called on each register access operation.
     *          Set via set_register_rw_callback(). nullptr if not configured.
     * 
     * @see set_register_rw_callback()
     */
    RegisterRWCb _register_rw_callback;
    
    /**
     * @brief Checked register system state
     * 
     * @details Manages automatic register verification and corruption recovery.
     *          Amortizes checking across multiple calls to avoid bus saturation.
     */
    struct {
        uint8_t n_allocated;           ///< Size of regs array (max registers to check)
        uint8_t n_set;                 ///< Number of registers currently monitored
        uint8_t next;                  ///< Index of next register to check (round-robin)
        uint8_t frequency;             ///< Check frequency divider (check every N calls)
        uint8_t counter;               ///< Current position in frequency countdown
        struct checkreg last_reg_fail; ///< Most recent register check failure details
        struct checkreg *regs;         ///< Array of checked register descriptors (allocated)
    } _checked;
};
