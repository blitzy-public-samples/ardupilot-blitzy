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
 * @file I2CDevice.h
 * @brief Linux i2c-dev interface implementation for I2C device access
 * 
 * Provides I2C bus access through Linux kernel i2c-dev interface (/dev/i2c-N).
 * Uses ioctl() calls for I2C transfers with automatic retry logic and
 * periodic callback support for sensor polling.
 */

#pragma once

#include <inttypes.h>
#include <vector>

#include <AP_HAL/HAL.h>
#include <AP_HAL/I2CDevice.h>

#include "Semaphores.h"

namespace Linux {

/**
 * Forward declaration of I2CBus - manages a single I2C bus
 */
class I2CBus;

/**
 * @class Linux::I2CDevice
 * @brief Linux kernel i2c-dev implementation of I2C device interface
 * 
 * @details Represents one I2C device (slave) on a Linux I2C bus. Uses Linux
 *          i2c-dev kernel interface accessed via /dev/i2c-N device files.
 *          
 *          Linux i2c-dev interface:
 *          - Open /dev/i2c-N where N is the bus number
 *          - Set slave address with I2C_SLAVE ioctl
 *          - Perform transfers with I2C_RDWR ioctl (i2c_msg structures)
 *          - Kernel driver handles bus arbitration, ACK/NACK, timing
 *          
 *          Transfer modes:
 *          - Combined transfers: write-then-read without stop condition
 *          - Split transfers: separate write and read with stop between
 *          - set_split_transfers(true) forces split mode for some devices
 *          
 *          Retry mechanism:
 *          - Configurable via set_retries() (default: 0)
 *          - Retries on NACK or bus busy conditions
 *          - Exponential backoff between retries
 *          - Protects against transient bus errors
 *          
 *          Periodic callbacks:
 *          - register_periodic_callback() for sensor polling
 *          - Callbacks execute in I2CBus poller thread
 *          - Each callback has independent period (typically 100Hz-1kHz)
 *          - Used by sensor drivers for continuous sampling
 *          
 *          Thread safety:
 *          - get_semaphore() returns per-bus semaphore
 *          - Multiple devices on same bus share semaphore
 *          - Prevents concurrent access to same bus
 *          - Use WITH_SEMAPHORE guard in driver code
 *          
 *          Typical usage pattern:
 *          ```cpp
 *          // Get device handle
 *          auto dev = hal.i2c_mgr->get_device(1, 0x68); // bus 1, address 0x68
 *          
 *          // Register periodic callback
 *          dev->register_periodic_callback(10000, // 10ms = 100Hz
 *              FUNCTOR_BIND_MEMBER(&Driver::_timer, void));
 *          
 *          // In callback, read sensor:
 *          uint8_t data[6];
 *          WITH_SEMAPHORE(dev->get_semaphore());
 *          if (!dev->read_registers(REG_ACCEL_X, data, 6)) {
 *              return; // Read failed
 *          }
 *          // Process data...
 *          ```
 * 
 * @note Requires i2c-dev kernel module (CONFIG_I2C_CHARDEV=y)
 * @note Bus speed not configurable via i2c-dev (set in device tree)
 * @warning Some I2C devices require split transfers - use set_split_transfers(true)
 * @warning Not all kernel I2C adapters support combined read/write
 * 
 * @see I2CBus for bus management and polling thread
 * @see I2CDeviceManager for device creation
 */
class I2CDevice : public AP_HAL::I2CDevice {
public:
    /**
     * @brief Downcast helper from generic I2CDevice pointer
     * 
     * @param[in] dev Pointer to AP_HAL::I2CDevice base class
     * @return I2CDevice* Downcasted pointer to Linux implementation
     */
    static I2CDevice *from(AP_HAL::I2CDevice *dev)
    {
        return static_cast<I2CDevice*>(dev);
    }

    /* AP_HAL::I2CDevice implementation */

    /**
     * @brief Constructor creates I2C device handle
     * 
     * @param[in] bus Reference to I2CBus managing this device's bus
     * @param[in] address 7-bit I2C slave address (0x00-0x7F)
     * 
     * @note Does not open device or access bus yet
     * @note Address can be changed later with set_address()
     */
    I2CDevice(I2CBus &bus, uint8_t address);

    /**
     * @brief Destructor unregisters device from bus
     * 
     * @note Automatically cancels periodic callbacks
     * @note Does not close bus (shared by multiple devices)
     */
    ~I2CDevice();

    /**
     * @brief Change I2C slave address for this device
     * 
     * @param[in] address New 7-bit I2C slave address
     * 
     * @note Useful for devices with programmable addresses
     * @note Takes effect on next transfer
     * @warning Do not change address while transfers in progress
     */
    void set_address(uint8_t address) override { _address = address; }

    /**
     * @brief Set number of automatic transfer retries on failure
     * 
     * @param[in] retries Number of retry attempts (0 = no retries)
     * 
     * @note Retries on NACK or bus busy conditions
     * @note Exponential backoff: 10μs, 100μs, 1ms delays
     * @note Useful for devices with clock stretching or slow response
     */
    void set_retries(uint8_t retries) override { _retries = retries; }

    /* AP_HAL::Device implementation */

    /**
     * @brief Set bus speed (not implemented for i2c-dev)
     * 
     * @param[in] speed Desired bus speed (ignored)
     * @return bool Always returns true (no-op)
     * 
     * @note i2c-dev does not support runtime speed changes
     * @note Bus speed configured in device tree or kernel parameters
     * @note Typical speeds: 100kHz (standard), 400kHz (fast), 1MHz (fast-plus)
     */
    bool set_speed(enum Device::Speed speed) override { return true; }

    /**
     * @brief Perform I2C read/write transfer
     * 
     * @param[in] send Pointer to data to write (or nullptr for read-only)
     * @param[in] send_len Number of bytes to write
     * @param[out] recv Pointer to buffer for received data (or nullptr for write-only)
     * @param[in] recv_len Number of bytes to read
     * @return bool True if transfer successful, false on error
     * 
     * @details Transfer modes:
     *          - Write-only: send != nullptr, recv == nullptr
     *          - Read-only: send == nullptr, recv != nullptr
     *          - Write-then-read: both != nullptr (combined or split based on flag)
     *          
     *          Combined vs split transfers:
     *          - Combined (default): Write and read in single I2C transaction
     *          - Split (set_split_transfers): Write, STOP, START, read
     *          - Some devices require split mode for register reads
     *          
     *          Error conditions:
     *          - ENXIO: No device at address (NACK)
     *          - EREMOTEIO: I/O error on bus
     *          - EAGAIN/EBUSY: Bus busy, retried automatically
     * 
     * @note Automatically acquires bus semaphore during transfer
     * @note Retries on failure if set_retries() configured
     * @warning send and recv must not overlap (undefined behavior)
     */
    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override;

    /**
     * @brief Read multiple consecutive registers with repeated start
     * 
     * @param[in] first_reg Starting register address
     * @param[out] recv Buffer for received data
     * @param[in] recv_len Number of bytes to read per transaction
     * @param[in] times Number of times to repeat read
     * @return bool True if all reads successful
     * 
     * @details Performs multiple read transactions:
     *          1. Write first_reg address
     *          2. Read recv_len bytes
     *          3. Repeat 'times' times
     *          
     *          Used for high-rate sensor sampling (e.g., gyro FIFO reads).
     * 
     * @note Each read is a separate I2C transaction
     * @note Total bytes read = recv_len * times
     */
    bool read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                 uint32_t recv_len, uint8_t times) override;

    /**
     * @brief Get semaphore protecting this device's bus
     * 
     * @return AP_HAL::Semaphore* Pointer to bus semaphore (shared by all devices on bus)
     * 
     * @note Use WITH_SEMAPHORE(dev->get_semaphore()) in driver code
     * @note Semaphore is per-bus, not per-device
     * @note Prevents concurrent access to same I2C bus from multiple threads
     */
    AP_HAL::Semaphore *get_semaphore() override;

    /**
     * @brief Register periodic callback for sensor polling
     * 
     * @param[in] period_usec Callback period in microseconds
     * @param[in] cb Callback function to execute periodically
     * @return AP_HAL::Device::PeriodicHandle Handle for adjust/cancel operations
     * 
     * @details Callback execution:
     *          - Executes in I2CBus poller thread (not main thread)
     *          - Period accuracy: ±10-100μs depending on kernel scheduler
     *          - Multiple callbacks supported per device with independent periods
     *          - Automatically acquires bus semaphore before callback
     *          
     *          Typical usage:
     *          ```cpp
     *          // Poll sensor at 100Hz (10ms = 10000μs)
     *          _periodic_handle = _dev->register_periodic_callback(10000,
     *              FUNCTOR_BIND_MEMBER(&IMU_Driver::_timer, void));
     *          ```
     * 
     * @note Callback must complete in less than period to avoid overruns
     * @note Use for sensor drivers requiring continuous sampling
     * @warning Callback runs in I2C bus thread context, not main thread
     */
    AP_HAL::Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;

    /**
     * @brief Adjust period of existing periodic callback
     * 
     * @param[in] h Handle returned from register_periodic_callback()
     * @param[in] period_usec New period in microseconds
     * @return bool True if adjustment successful
     * 
     * @note Useful for changing sensor sample rates dynamically
     * @note Takes effect on next callback invocation
     */
    bool adjust_periodic_callback(
        AP_HAL::Device::PeriodicHandle h, uint32_t period_usec) override;

    /**
     * @brief Enable split transfer mode (separate write and read transactions)
     * 
     * @param[in] set True to enable split transfers, false for combined
     * 
     * @details Split vs combined transfers:
     *          - Combined (default): WRITE data, REPEAT-START, READ data (no STOP between)
     *          - Split (enabled): WRITE data, STOP, START, READ data
     *          
     *          Some I2C devices require split mode because they:
     *          - Don't support repeated start condition
     *          - Need processing time between write and read
     *          - Have strict protocol timing requirements
     * 
     * @note Try combined first (more efficient); use split if device doesn't respond
     */
    void set_split_transfers(bool set) override {
        _split_transfers = set;
    }
    
protected:
    /**
     * Reference to I2CBus managing this device's physical bus
     */
    I2CBus &_bus;
    
    /**
     * 7-bit I2C slave address (0x00-0x7F)
     */
    uint8_t _address;
    
    /**
     * Number of automatic retry attempts on transfer failure (default: 0)
     */
    uint8_t _retries = 0;
    
    /**
     * Split transfer mode flag: true = separate write/read, false = combined
     */
    bool _split_transfers = false;
};

/**
 * @class Linux::I2CDeviceManager
 * @brief Factory for creating and managing Linux I2C devices
 * 
 * @details Manages I2C buses and creates device handles. Each I2CBus has a
 *          dedicated poller thread for periodic callbacks and bus arbitration.
 *          
 *          Bus discovery:
 *          - Scans /dev/i2c-* for available buses
 *          - Creates I2CBus instance for each discovered bus
 *          - Buses created on-demand when first device requested
 *          
 *          Device lifecycle:
 *          1. get_device() creates I2CDevice handle
 *          2. Device registers periodic callbacks if needed
 *          3. Device accesses bus via transfer() methods
 *          4. Device destroyed when driver releases handle
 *          
 *          Bus masking:
 *          - get_bus_mask(): All available buses
 *          - get_bus_mask_external(): External I2C buses (exposed connectors)
 *          - get_bus_mask_internal(): Internal I2C buses (on-board sensors)
 *          - Allows selective bus probing (e.g., avoid probing internal buses)
 * 
 * @note Thread-safe: Multiple threads can create devices concurrently
 * @note Buses persist for lifetime of manager (not per-device)
 * 
 * @see I2CDevice for device operations
 * @see I2CBus for bus management (internal)
 */
class I2CDeviceManager : public AP_HAL::I2CDeviceManager {
public:
    friend class I2CDevice;

    /**
     * @brief Downcast helper from generic I2CDeviceManager pointer
     * 
     * @param[in] i2c_mgr Pointer to AP_HAL::I2CDeviceManager base class
     * @return I2CDeviceManager* Downcasted pointer to Linux implementation
     */
    static I2CDeviceManager *from(AP_HAL::I2CDeviceManager *i2c_mgr)
    {
        return static_cast<I2CDeviceManager*>(i2c_mgr);
    }

    /**
     * @brief Constructor initializes I2C device manager
     * 
     * @note Does not enumerate buses yet - deferred to first get_device() call
     */
    I2CDeviceManager();

    /* AP_HAL::I2CDeviceManager implementation */
    
    /**
     * @brief Create or get existing I2C device handle
     * 
     * @param[in] bus Bus number (maps to /dev/i2c-N)
     * @param[in] address 7-bit I2C slave address
     * @param[in] bus_clock Desired bus clock in Hz (ignored on i2c-dev)
     * @param[in] use_smbus Enable SMBus protocol (typically false)
     * @param[in] timeout_ms Transfer timeout in milliseconds (default: 4ms)
     * @return AP_HAL::I2CDevice* Pointer to device handle, or nullptr on error
     * 
     * @details Device creation:
     *          - If bus doesn't exist, creates I2CBus and starts poller thread
     *          - Creates I2CDevice associated with bus
     *          - Multiple get_device() calls return separate device handles
     *          - Each handle can have independent periodic callbacks
     * 
     * @note bus_clock parameter ignored (bus speed set in device tree)
     * @note Returns nullptr if /dev/i2c-N doesn't exist
     * @note Caller owns returned pointer (auto-deleted via shared_ptr internally)
     */
    AP_HAL::I2CDevice *get_device_ptr(uint8_t bus, uint8_t address,
                                     uint32_t bus_clock=400000,
                                     bool use_smbus = false,
                                     uint32_t timeout_ms=4) override;

    /**
     * @brief Stop all I2C polling threads and finalize
     * 
     * @note Called during HAL shutdown
     * @note Blocks until all poller threads complete
     * @note Existing device handles remain valid but callbacks stop
     */
    void teardown();

    /**
     * @brief Get bitmask of all available I2C buses
     * 
     * @return uint32_t Bitmask where bit N set means /dev/i2c-N exists
     * 
     * @note Bit 0 = bus 0, bit 1 = bus 1, etc.
     * @note Use for selective bus probing in drivers
     */
    uint32_t get_bus_mask(void) const override;

    /**
     * @brief Get bitmask of external I2C buses (exposed connectors)
     * 
     * @return uint32_t Bitmask of external buses
     * 
     * @note External buses have user-accessible connectors
     * @note Used for GPS, compass, and expansion peripherals
     * @note Board-specific configuration (hwdef)
     */
    uint32_t get_bus_mask_external(void) const override;

    /**
     * @brief Get bitmask of internal I2C buses (on-board sensors)
     * 
     * @return uint32_t Bitmask of internal buses
     * 
     * @note Internal buses connect to on-board IMU, barometer, etc.
     * @note Typically not probed for user peripherals
     * @note Board-specific configuration (hwdef)
     */
    uint32_t get_bus_mask_internal(void) const override;
    
protected:
    /**
     * @brief Unregister and cleanup I2CBus (internal use)
     * 
     * @param[in] b Reference to I2CBus to unregister
     * 
     * @note Called when last device on bus is destroyed
     */
    void _unregister(I2CBus &b);
    
    /**
     * @brief Create I2CDevice instance (internal factory method)
     * 
     * @param[in] b Reference to I2CBus for device
     * @param[in] address 7-bit I2C slave address
     * @return AP_HAL::I2CDevice* Pointer to new device
     */
    AP_HAL::I2CDevice *_create_device(I2CBus &b, uint8_t address) const;

    /**
     * Vector of I2CBus instances (one per discovered bus)
     * Buses created on-demand and persist for manager lifetime
     */
    std::vector<I2CBus*> _buses;
};

}
