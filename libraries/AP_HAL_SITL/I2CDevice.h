/*
 * Copyright (C) 2020  Peter Barker. All rights reserved.
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
 * @brief SITL I2C device simulation implementation
 * 
 * @details This file implements simulated I2C device communication for ArduPilot's
 *          Software In The Loop (SITL) simulation environment. It provides a HAL
 *          implementation that allows simulated sensors and peripherals to communicate
 *          over virtual I2C buses without requiring physical hardware.
 * 
 *          The simulation works by marshaling I2C transactions (i2c_msg and 
 *          i2c_rdwr_ioctl_data structures) to the SITL_State I2C ioctl handler
 *          via AP::sitl()->i2c_ioctl(). This allows simulated sensors (barometers,
 *          magnetometers, etc.) to respond to I2C read/write operations as if they
 *          were real hardware devices.
 * 
 *          Key architectural components:
 *          - I2CDevice: Simulates an individual I2C peripheral device
 *          - I2CDeviceManager: Creates and manages I2C device instances
 *          - I2CBus: Provides per-bus semaphores for thread-safe access
 * 
 *          Simulated I2C devices must implement handlers in SITL_State to respond
 *          to transactions. The simulation uses the same AP_HAL::I2CDevice interface
 *          as real hardware, enabling transparent testing of sensor drivers in SITL.
 * 
 * @warning The SITL I2C simulation does not model real-world I2C timing constraints,
 *          clock stretching, or electrical characteristics. Code that depends on
 *          precise I2C timing may behave differently on real hardware.
 * 
 * @note This implementation is only compiled when CONFIG_HAL_BOARD == HAL_BOARD_SITL
 * 
 * @see SITL_State::i2c_ioctl() for I2C transaction handling
 * @see AP_HAL::I2CDevice for the abstract I2C device interface
 */
#pragma once

#include <inttypes.h>

#include <AP_HAL/HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <AP_HAL/I2CDevice.h>

#include "AP_HAL_SITL_Namespace.h"
#include "Semaphores.h"

class I2CBus;

/**
 * @class HALSITL::I2CDevice
 * @brief Simulated I2C device for SITL environment
 * 
 * @details This class simulates an I2C peripheral device in ArduPilot's Software
 *          In The Loop (SITL) simulation. It implements the AP_HAL::I2CDevice
 *          interface to provide transparent I2C communication for simulated sensors
 *          and peripherals.
 * 
 *          The simulation mechanism works as follows:
 *          1. Driver code calls transfer() or read_registers_multiple()
 *          2. I2CDevice marshals the request into i2c_msg structures
 *          3. The i2c_rdwr_ioctl_data structure containing messages is passed to
 *             AP::sitl()->i2c_ioctl()
 *          4. SITL_State routes the transaction to the appropriate simulated device
 *          5. Simulated device handler processes the transaction and returns data
 * 
 *          Each I2CDevice is associated with a specific bus number and device address.
 *          Multiple devices can exist on the same bus, with the I2CBus object providing
 *          thread-safe access via per-bus semaphores.
 * 
 *          Periodic callbacks registered via register_periodic_callback() are executed
 *          by the I2CDeviceManager::_timer_tick() method, which is called from the
 *          main SITL scheduler loop.
 * 
 * @note Thread Safety: All I2C operations should be protected by acquiring the
 *       bus semaphore via get_semaphore() to prevent race conditions with other
 *       devices on the same bus.
 * 
 * @warning Simulation does not model I2C clock stretching, arbitration, or electrical
 *          timing. Code depending on these hardware-specific behaviors may not work
 *          identically in SITL.
 * 
 * @see SITL_State::i2c_ioctl()
 * @see I2CDeviceManager
 * @see I2CBus
 */
class HALSITL::I2CDevice : public AP_HAL::I2CDevice {
public:
    /**
     * @brief Cast AP_HAL::I2CDevice pointer to SITL-specific I2CDevice
     * 
     * @param[in] dev Pointer to generic I2C device interface
     * @return Pointer to SITL I2CDevice implementation
     * 
     * @note This is a convenience method for accessing SITL-specific functionality
     */
    static I2CDevice *from(AP_HAL::I2CDevice *dev) {
        return static_cast<I2CDevice*>(dev);
    }

    /* AP_HAL::I2CDevice implementation */

    /**
     * @brief Construct a simulated I2C device
     * 
     * @param[in] bus     Reference to the I2C bus this device is attached to
     * @param[in] address 7-bit I2C device address (0x00-0x7F)
     * 
     * @details Creates a simulated I2C device on the specified bus with the given
     *          address. The device will be accessible via SITL's I2C simulation
     *          system if a corresponding handler is registered in SITL_State.
     */
    I2CDevice(I2CBus &bus, uint8_t address);

    /**
     * @brief Destructor for simulated I2C device
     */
    ~I2CDevice() {}

    /**
     * @brief Change the I2C address of this device
     * 
     * @param[in] address New 7-bit I2C device address (0x00-0x7F)
     * 
     * @details Updates the device address used for subsequent I2C transactions.
     *          This is useful for devices that support multiple addresses or
     *          for probing devices at different addresses during initialization.
     * 
     * @note The address change takes effect immediately for the next transfer
     * 
     * @see AP_HAL::I2CDevice::set_address()
     */
    void set_address(uint8_t address) override { _address = address; }

    /**
     * @brief Set the number of retries for failed I2C transactions
     * 
     * @param[in] retries Number of times to retry a failed transaction (0-255)
     * 
     * @details Configures how many times the device will retry a failed I2C
     *          transfer before reporting failure. In SITL, retries are simulated
     *          but may not reflect real hardware behavior where bus errors,
     *          NACKs, or arbitration losses can occur.
     * 
     * @note Default retry count is typically 0
     * 
     * @see AP_HAL::I2CDevice::set_retries()
     */
    void set_retries(uint8_t retries) override { _retries = retries; }

    /* AP_HAL::Device implementation */

    /**
     * @brief Set I2C bus speed (not implemented in SITL)
     * 
     * @param[in] speed Desired bus speed (ignored in SITL)
     * @return Always returns true
     * 
     * @details Empty implementation as SITL does not simulate I2C clock speeds.
     *          Real hardware may support Device::Speed::SPEED_LOW (100kHz),
     *          Device::Speed::SPEED_HIGH (400kHz), etc., but SITL transactions
     *          complete instantaneously.
     * 
     * @warning Code depending on I2C timing or clock speed behavior will not
     *          work correctly in SITL
     * 
     * @see AP_HAL::Device::set_speed()
     */
    bool set_speed(enum Device::Speed speed) override { return true; }

    /**
     * @brief Perform bidirectional I2C data transfer
     * 
     * @param[in]  send     Buffer containing data to send (can be nullptr if send_len is 0)
     * @param[in]  send_len Number of bytes to send
     * @param[out] recv     Buffer to receive data (can be nullptr if recv_len is 0)
     * @param[in]  recv_len Number of bytes to receive
     * @return true if transfer succeeded, false on simulated I2C error
     * 
     * @details Performs a combined write-then-read I2C transaction. This is the
     *          primary method for I2C communication in ArduPilot drivers.
     * 
     *          The transfer is simulated by:
     *          1. Creating i2c_msg structures for write and/or read operations
     *          2. Packaging messages into i2c_rdwr_ioctl_data structure
     *          3. Calling AP::sitl()->i2c_ioctl(I2C_RDWR, ...) to execute
     *          4. SITL_State routes to appropriate simulated device handler
     * 
     *          If _split_transfers is true, write and read are sent as separate
     *          messages. Otherwise, they are combined into a single transaction.
     * 
     * @note This method may be called at high frequency (e.g., 1kHz for IMU sensors),
     *       so simulated device handlers should be efficient
     * 
     * @warning Always acquire the bus semaphore before calling this method to
     *          ensure thread-safe access
     * 
     * @see AP_HAL::Device::transfer()
     * @see SITL_State::i2c_ioctl()
     */
    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override;

    /**
     * @brief Read the same register multiple times
     * 
     * @param[in]  first_reg Starting register address
     * @param[out] recv      Buffer to receive data
     * @param[in]  recv_len  Number of bytes to read per transaction
     * @param[in]  times     Number of times to repeat the read
     * @return true if all reads succeeded, false on any failure
     * 
     * @details Performs multiple consecutive reads of the same register address.
     *          This is used by some sensor drivers to sample data at high rates
     *          or to read FIFO buffers.
     * 
     *          Each read transaction sends the register address then reads recv_len
     *          bytes. The received data is concatenated in the recv buffer.
     * 
     * @note Total bytes read = recv_len * times
     * 
     * @see AP_HAL::I2CDevice::read_registers_multiple()
     */
    bool read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                 uint32_t recv_len, uint8_t times) override;

    /**
     * @brief Get the semaphore for thread-safe bus access
     * 
     * @return Pointer to the semaphore for this device's I2C bus
     * 
     * @details Returns the per-bus semaphore that must be acquired before
     *          performing any I2C operations. This prevents multiple devices
     *          on the same bus from interfering with each other's transactions.
     * 
     *          Typical usage pattern:
     *          @code
     *          AP_HAL::Semaphore *sem = dev->get_semaphore();
     *          if (!sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
     *              return false;
     *          }
     *          bool success = dev->transfer(send, send_len, recv, recv_len);
     *          sem->give();
     *          @endcode
     * 
     *          Or using WITH_SEMAPHORE macro:
     *          @code
     *          WITH_SEMAPHORE(dev->get_semaphore());
     *          dev->transfer(send, send_len, recv, recv_len);
     *          @endcode
     * 
     * @note Each I2C bus has its own semaphore, so devices on different buses
     *       can operate concurrently
     * 
     * @see AP_HAL::Device::get_semaphore()
     * @see I2CBus
     */
    AP_HAL::Semaphore *get_semaphore() override;

    /**
     * @brief Register a callback to be executed periodically
     * 
     * @param[in] period_usec Period in microseconds between callback invocations
     * @param[in] cb          Callback function to execute
     * @return Handle to the registered callback, or nullptr on failure
     * 
     * @details Registers a periodic callback that will be invoked at the specified
     *          interval by the I2CDeviceManager timer tick. This is commonly used
     *          by sensor drivers to poll hardware at regular intervals.
     * 
     *          The callback is executed in the context of the I2CDeviceManager's
     *          timer tick, which runs in the SITL scheduler. Callbacks should:
     *          - Complete quickly (< 1ms execution time preferred)
     *          - Acquire the bus semaphore before I2C operations
     *          - Handle failures gracefully
     * 
     *          Example usage (IMU driver reading at 1kHz):
     *          @code
     *          auto handle = dev->register_periodic_callback(1000, 
     *              FUNCTOR_BIND_MEMBER(&AP_InertialSensor::update, void));
     *          @endcode
     * 
     * @note Periodic callbacks continue until the device is destroyed or the
     *       callback is adjusted/removed
     * 
     * @see AP_HAL::Device::register_periodic_callback()
     * @see I2CDeviceManager::_timer_tick()
     * @see adjust_periodic_callback()
     */
    AP_HAL::Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;

    /**
     * @brief Adjust the period of an existing periodic callback
     * 
     * @param[in] h           Handle to the callback (returned by register_periodic_callback)
     * @param[in] period_usec New period in microseconds
     * @return true if adjustment succeeded, false if handle is invalid
     * 
     * @details Changes the execution interval of a previously registered periodic
     *          callback. This is useful for dynamically adjusting sensor sample
     *          rates based on flight mode or system load.
     * 
     * @note The new period takes effect after the next callback invocation
     * 
     * @see AP_HAL::Device::adjust_periodic_callback()
     * @see register_periodic_callback()
     */
    bool adjust_periodic_callback(
        AP_HAL::Device::PeriodicHandle h, uint32_t period_usec) override;

    /**
     * @brief Enable or disable split transfer mode
     * 
     * @param[in] set true to enable split transfers, false for combined transfers
     * 
     * @details When split transfers are enabled, write and read operations in a
     *          transfer() call are sent as separate I2C messages. When disabled,
     *          they are combined into a single transaction with a repeated START
     *          condition.
     * 
     *          Some I2C devices require split transfers, while others expect
     *          combined transfers. This setting allows drivers to match device
     *          requirements.
     * 
     * @note Default is false (combined transfers)
     * 
     * @see transfer()
     */
    void set_split_transfers(bool set) override {
        _split_transfers = set;
    }

    /**
     * @brief Update simulated I2C devices (static update function)
     * 
     * @details This static method is called periodically to update the state of
     *          all simulated I2C devices. Currently a placeholder for potential
     *          future simulation features.
     * 
     * @note This is separate from periodic callbacks, which are per-device
     */
    static void sitl_update();

    /**
     * @name I2C Message Structures
     * @brief Data structures for marshaling I2C transactions to SITL simulation
     * 
     * @details These structures must be identical to those defined in SITL/SIM_I2C.h
     *          to ensure proper communication between the HAL and the simulation backend.
     *          
     *          The structures model the Linux i2c-dev ioctl interface, allowing SITL
     *          to simulate I2C communication using familiar Linux I2C semantics.
     * 
     * @warning Do not modify these structures without updating SITL/SIM_I2C.h to match
     * @{
     */
    
    /** @brief Flag indicating this message is a read operation */
#define I2C_M_RD 1
    
    /** @brief ioctl command code for read/write operations */
#define I2C_RDWR 0
    
    /**
     * @struct i2c_msg
     * @brief Represents a single I2C read or write message
     */
    struct i2c_msg {
        uint8_t bus;      ///< I2C bus number (0 to NUM_SITL_I2C_BUSES-1)
        uint8_t addr;     ///< 7-bit I2C device address
        uint8_t flags;    ///< Message flags (I2C_M_RD for read, 0 for write)
        uint8_t *buf;     ///< Data buffer for send or receive
        uint16_t len;     ///< Number of bytes to transfer
    };
    
    /**
     * @struct i2c_rdwr_ioctl_data
     * @brief Container for one or more I2C messages in a transaction
     */
    struct i2c_rdwr_ioctl_data {
        i2c_msg *msgs;    ///< Array of I2C messages
        uint8_t nmsgs;    ///< Number of messages in the array
    };
    /** @} */

protected:
    I2CBus &_bus;                     ///< Reference to the I2C bus this device is on
    uint8_t _address;                 ///< 7-bit I2C device address
    uint8_t _retries;                 ///< Number of retries for failed transactions
    bool _split_transfers = false;    ///< true = send write/read as separate messages

    /**
     * @brief Internal transfer implementation
     * 
     * @param[in]  send     Buffer containing data to send
     * @param[in]  send_len Number of bytes to send
     * @param[out] recv     Buffer to receive data
     * @param[in]  recv_len Number of bytes to receive
     * @return true if transfer succeeded, false on error
     * 
     * @details Protected implementation of the transfer operation that constructs
     *          i2c_msg and i2c_rdwr_ioctl_data structures and passes them to
     *          AP::sitl()->i2c_ioctl() for simulation.
     * 
     * @note Called by the public transfer() method with retry logic
     */
    bool _transfer(const uint8_t *send, uint32_t send_len,
                   uint8_t *recv, uint32_t recv_len);
};

/**
 * @class HALSITL::I2CDeviceManager
 * @brief Manager for simulated I2C devices in SITL
 * 
 * @details The I2CDeviceManager creates and manages I2CDevice instances for the
 *          SITL simulation environment. It maintains an array of I2CBus objects,
 *          each representing a simulated I2C bus with its own semaphore for
 *          thread-safe access.
 * 
 *          Key responsibilities:
 *          - Create I2CDevice instances on requested buses and addresses
 *          - Maintain per-bus resources (semaphores, device lists)
 *          - Execute periodic callbacks registered by devices via _timer_tick()
 *          - Provide bus enumeration (up to NUM_SITL_I2C_BUSES buses)
 * 
 *          In SITL, there is no actual I2C hardware or per-bus threads. Instead,
 *          the _timer_tick() method is called periodically from the main SITL
 *          scheduler to process all device callbacks.
 * 
 *          SITL supports up to 4 I2C buses (NUM_SITL_I2C_BUSES), allowing testing
 *          of multi-bus configurations common in flight controllers.
 * 
 * @note Unlike real hardware HAL implementations, SITL does not support SMBus
 *       protocol variations, bus clock speed settings, or hardware timeouts.
 *       These parameters are accepted but ignored in get_device_ptr().
 * 
 * @see I2CDevice
 * @see I2CBus
 */
class HALSITL::I2CDeviceManager : public AP_HAL::I2CDeviceManager {
public:
    friend class I2CDevice;

    /**
     * @brief Cast AP_HAL::I2CDeviceManager pointer to SITL-specific implementation
     * 
     * @param[in] i2c_mgr Pointer to generic I2C device manager interface
     * @return Pointer to SITL I2CDeviceManager implementation
     * 
     * @note Convenience method for accessing SITL-specific functionality
     */
    static I2CDeviceManager *from(AP_HAL::I2CDeviceManager *i2c_mgr)
    {
        return static_cast<I2CDeviceManager*>(i2c_mgr);
    }

    /**
     * @brief Construct the I2C device manager
     * 
     * @details Initializes the I2C bus array and prepares the manager for
     *          device creation. Called once during HAL initialization.
     */
    I2CDeviceManager();
    
    /**
     * @brief Destructor for I2C device manager
     */
    ~I2CDeviceManager() {};

    /**
     * @brief Process periodic callbacks for all I2C devices
     * 
     * @details Called periodically from the SITL scheduler to execute registered
     *          device callbacks. This replaces the per-bus threads found in real
     *          hardware HAL implementations.
     * 
     *          The method iterates through all buses and invokes due callbacks
     *          for devices on each bus. Callbacks are executed in the order they
     *          were registered.
     * 
     * @note This method is called from the main SITL loop at a fixed rate
     *       (typically 1kHz), so callback timing may have jitter.
     */
    void _timer_tick();

    /**
     * @brief Get or create an I2C device instance
     * 
     * @param[in] bus        I2C bus number (0 to NUM_SITL_I2C_BUSES-1)
     * @param[in] address    7-bit I2C device address (0x00-0x7F)
     * @param[in] bus_clock  Requested bus clock frequency in Hz (ignored in SITL)
     * @param[in] use_smbus  true to use SMBus protocol (ignored in SITL)
     * @param[in] timeout_ms Transaction timeout in milliseconds (ignored in SITL)
     * @return Pointer to I2CDevice instance, or nullptr if bus number is invalid
     * 
     * @details Creates a new I2CDevice instance for the specified bus and address.
     *          Multiple calls with the same bus and address will create separate
     *          device instances (they are not cached or shared).
     * 
     *          The bus_clock, use_smbus, and timeout_ms parameters are accepted for
     *          API compatibility with real hardware but are ignored in SITL:
     *          - SITL does not simulate I2C clock speeds
     *          - SITL does not implement SMBus protocol features (PEC, block reads)
     *          - SITL transactions complete instantaneously (no timeouts needed)
     * 
     * @note Always check return value before using - invalid bus numbers return nullptr
     * 
     * @warning The caller is responsible for the lifecycle of the returned device.
     *          Devices are typically stored as class members and destroyed with
     *          their containing driver object.
     * 
     * @see AP_HAL::I2CDeviceManager::get_device_ptr()
     */
    AP_HAL::I2CDevice *get_device_ptr(uint8_t bus, uint8_t address,
                                      uint32_t bus_clock=400000,
                                      bool use_smbus = false,
                                      uint32_t timeout_ms=4) override;

protected:

    /** @brief Number of simulated I2C buses available in SITL */
    #define NUM_SITL_I2C_BUSES 4
    
    /**
     * @brief Array of I2C bus objects
     * 
     * @details Static array containing all simulated I2C buses. Each bus
     *          maintains its own semaphore and device callback list.
     */
    static I2CBus buses[];
};
#endif //#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
