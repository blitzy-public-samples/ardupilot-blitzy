/*
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
 * @brief QURT I2C device implementation for Qualcomm Hexagon DSP platform
 * 
 * @details This file implements I2C device drivers for the QURT (Qualcomm 
 *          User Real-Time) platform running on Hexagon DSP. All I2C operations 
 *          use Remote Procedure Calls (RPC) to the applications processor where 
 *          the physical I2C hardware controllers are located.
 *          
 *          The implementation uses the sl_client_i2c_transfer() RPC API from 
 *          interface.h to communicate with I2C peripherals (compass, barometers, 
 *          external sensors, etc.) connected to the apps processor's I2C buses.
 *          
 *          Architecture:
 *          - I2CBus: Represents a single I2C bus with device management
 *          - I2CDevice: Individual I2C peripheral driver (compass, baro, etc.)
 *          - I2CDeviceManager: Factory for creating I2CDevice instances
 *          
 * @note RPC latency overhead: Each I2C transfer incurs 50-500μs inter-processor 
 *       communication latency in addition to actual I2C transaction time
 *       
 * @warning Physical I2C controllers are on apps processor, not DSP. All access 
 *          goes through RPC layer which adds latency and potential failure modes
 *          
 * @see DeviceBus.h for base bus scheduling and callback management
 * @see interface.h for sl_client_i2c_transfer RPC function declarations
 */
#pragma once

#include <inttypes.h>

#include <AP_HAL/HAL.h>
#include <AP_HAL/I2CDevice.h>

#include "Semaphores.h"
#include "Scheduler.h"
#include "DeviceBus.h"

namespace QURT
{

/**
 * @class I2CBus
 * @brief Represents a single I2C bus on the Qualcomm Hexagon DSP platform
 * 
 * @details This class extends DeviceBus to provide I2C-specific bus handling 
 *          with device management and address caching for performance optimization.
 *          Each I2CBus instance corresponds to one physical I2C bus on the apps 
 *          processor (e.g., I2C0, I2C1).
 *          
 *          The bus manages multiple I2CDevice instances that share the same 
 *          physical I2C bus. Device callbacks are executed sequentially by the 
 *          bus thread to prevent bus conflicts since I2C is a shared medium.
 *          
 *          Key Features:
 *          - Device callback scheduling at I2C priority level
 *          - Bus clock configuration (typically 100kHz or 400kHz)
 *          - Address caching to optimize consecutive transfers to same device
 *          - Semaphore protection for atomic multi-transfer operations
 *          
 * @note Bus scheduling: Inherits from DeviceBus with PRIORITY_I2C to ensure 
 *       I2C operations run at appropriate priority relative to other buses
 *       
 * @note Address caching: last_address optimization skips redundant address set 
 *       operations when the same device is accessed consecutively, reducing RPC calls
 *       
 * @warning I2C is a shared bus: Only one device can transfer at a time. Bus 
 *          semaphore enforces mutual exclusion across all devices on the bus.
 *          
 * @see DeviceBus for base scheduling and callback management implementation
 */
class I2CBus : public DeviceBus
{
public:
    I2CBus():DeviceBus(AP_HAL::Scheduler::PRIORITY_I2C) {};
    
    /**
     * @brief Configurable I2C clock speed for this bus
     * 
     * @details Specifies the I2C bus clock frequency in Hz. Set via RPC when 
     *          device is initialized. Common values:
     *          - 100000 (100kHz): Standard mode, most compatible
     *          - 400000 (400kHz): Fast mode, higher throughput
     *          
     * @note Must match device capabilities. Some sensors only support 100kHz.
     *       Misconfiguration causes communication failures.
     */
    uint32_t bus_clock;
    
    /**
     * @brief File descriptor for I2C bus obtained via RPC to apps processor
     * 
     * @details This fd represents the open I2C bus device on the apps processor.
     *          It is obtained through the RPC layer during bus initialization 
     *          and used in all subsequent sl_client_i2c_transfer() calls.
     *          
     *          Value of -2 indicates uninitialized state.
     *          Valid fd is >= 0 after successful initialization.
     *          
     * @note This is NOT a DSP-local file descriptor. It represents a resource 
     *       on the apps processor, accessed through RPC.
     */
    int fd = -2;
    
    /**
     * @brief Cached I2C address of last accessed device
     * 
     * @details Performance optimization to skip redundant address set operations 
     *          when accessing the same device consecutively. Since I2C requires 
     *          addressing each transaction, caching the last address allows the 
     *          RPC layer to avoid unnecessary address configuration overhead.
     *          
     *          The address is a 7-bit I2C device address (0x00-0x7F).
     *          
     * @note This optimization is particularly effective when a single sensor 
     *       (e.g., IMU on I2C) is polled frequently at high rates.
     */
    uint8_t last_address;
};

/**
 * @class I2CDevice
 * @brief I2C device driver for peripherals connected to I2C buses on QURT platform
 * 
 * @details This class implements the AP_HAL::I2CDevice interface for I2C peripherals
 *          (compass, barometer, external sensors, etc.) on the Qualcomm Hexagon DSP 
 *          platform. All hardware access is performed via RPC to the apps processor 
 *          using the sl_client_i2c_transfer() API.
 *          
 *          Transfer Model:
 *          All I2C operations involve RPC to the apps processor where the physical 
 *          I2C controller performs the actual transaction. This adds latency but 
 *          allows the DSP to access peripherals connected to the apps processor's 
 *          I2C buses.
 *          
 *          Typical Usage Pattern:
 *          1. Create device: i2c_mgr->get_device(bus_num, i2c_address)
 *          2. Configure: set_retries(), optionally set_address()
 *          3. Register callback: register_periodic_callback(period, callback_func)
 *          4. In callback: Use transfer() or read_registers() to communicate
 *          
 *          Example - Compass Driver:
 *          @code
 *          // Create device for HMC5843 compass at address 0x1E on bus 0
 *          AP_HAL::I2CDevice *dev = i2c_mgr->get_device(0, 0x1E);
 *          
 *          // Register 100Hz callback (10000μs period)
 *          dev->register_periodic_callback(10000, FUNCTOR_BIND_MEMBER(&Compass::update, void));
 *          
 *          // In update callback, read magnetometer registers
 *          uint8_t reg = 0x03; // Data register
 *          uint8_t data[6];
 *          if (dev->transfer(&reg, 1, data, 6)) {
 *              // Process magnetometer data
 *          }
 *          @endcode
 *          
 *          Example - Barometer Combined Write/Read:
 *          @code
 *          // Write register address, read pressure data in single transfer
 *          uint8_t cmd = 0xF6; // Pressure MSB register
 *          uint8_t data[3];
 *          if (dev->transfer(&cmd, 1, data, 3)) {
 *              uint32_t pressure = (data[0] << 16) | (data[1] << 8) | data[2];
 *          }
 *          @endcode
 *          
 * @note RPC latency: Each I2C transfer has 50-500μs overhead from inter-processor 
 *       communication, in addition to the actual I2C transaction time. Design 
 *       sensor polling rates accordingly.
 *       
 * @note Addressing: 7-bit I2C device address is specified in constructor and 
 *       automatically handled in all transfers. Change with set_address() if needed.
 *       
 * @warning Clock stretching: Some I2C devices use clock stretching which adds 
 *          unpredictable delays. Test carefully on DSP platform as RPC timeout 
 *          may interact with stretched transactions.
 *          
 * @warning I2C bus speed limitations: 400kHz is standard maximum. Some devices 
 *          only support 100kHz. Misconfiguration causes communication failures 
 *          (NACK, timeout). Verify device datasheet specifications.
 *          
 * @see AP_HAL::I2CDevice for interface contract and method descriptions
 * @see DeviceBus.h for callback scheduling and bus thread management
 */
class I2CDevice : public AP_HAL::I2CDevice
{
public:
    /**
     * @brief Cast AP_HAL::I2CDevice pointer to QURT::I2CDevice
     * 
     * @param[in] dev Pointer to base I2CDevice interface
     * @return QURT::I2CDevice pointer for platform-specific access
     * 
     * @note Used internally when platform-specific operations are needed
     */
    static I2CDevice *from(AP_HAL::I2CDevice *dev)
    {
        return static_cast<I2CDevice*>(dev);
    }

    /**
     * @brief Construct I2C device driver instance
     * 
     * @param[in] bus I2C bus number (0, 1, etc.)
     * @param[in] address 7-bit I2C device address (0x00-0x7F)
     * @param[in] bus_clock I2C bus clock speed in Hz (typically 100000 or 400000)
     * @param[in] use_smbus Enable SMBus protocol extensions (typically false for sensors)
     * @param[in] timeout_ms RPC timeout in milliseconds (default: 4ms)
     * 
     * @note Initializes device but does not perform I2C communication. First 
     *       transfer will initialize the bus if needed.
     */
    I2CDevice(uint8_t bus, uint8_t address, uint32_t bus_clock, bool use_smbus, uint32_t timeout_ms);
    
    /**
     * @brief Destructor - cleanup device resources
     */
    ~I2CDevice();

    /**
     * @brief Change the I2C device address
     * 
     * @param[in] address New 7-bit I2C device address (0x00-0x7F)
     * 
     * @details Updates the device address used for subsequent I2C transfers.
     *          Rarely needed as address is typically fixed in constructor, but 
     *          useful for devices with programmable addresses or multi-device 
     *          configurations.
     *          
     * @note Takes effect on next transfer operation
     * @note Does not validate address or check if device responds at new address
     * 
     * @see AP_HAL::I2CDevice::set_address()
     */
    void set_address(uint8_t address) override
    {
        _address = address;
    }

    /**
     * @brief Configure transfer retry count for error recovery
     * 
     * @param[in] retries Number of retry attempts on NACK or transfer error
     * 
     * @details Sets how many times to retry failed I2C transfers before giving up.
     *          Default is typically 2 retries. Increase for unreliable buses or 
     *          devices that occasionally NACK. Decrease for faster failure detection.
     *          
     * @note Retries apply to all subsequent transfer operations
     * @note Each retry adds latency (RPC + I2C transaction time)
     * 
     * @see AP_HAL::I2CDevice::set_retries()
     */
    void set_retries(uint8_t retries) override
    {
        _retries = retries;
    }

    /**
     * @brief Set I2C bus speed (not supported on QURT platform)
     * 
     * @param[in] speed Desired bus speed (ignored)
     * @return Always returns true (success)
     * 
     * @details Empty implementation. Bus speed is configured at device creation 
     *          time via bus_clock parameter in constructor. Cannot be changed 
     *          dynamically on QURT platform as speed is set during RPC bus init.
     *          
     * @note To change speed, create new device instance with different bus_clock
     * 
     * @see AP_HAL::Device::set_speed()
     */
    bool set_speed(enum Device::Speed speed) override
    {
        return true;
    }

    /**
     * @brief Perform combined write/read I2C transfer
     * 
     * @param[in]  send     Buffer containing data to write to device
     * @param[in]  send_len Number of bytes to write
     * @param[out] recv     Buffer to receive data read from device
     * @param[in]  recv_len Number of bytes to read
     * 
     * @return true if transfer successful, false on NACK, timeout, or RPC error
     * 
     * @details Executes a combined I2C write followed by read operation via RPC 
     *          to apps processor. This is the fundamental I2C operation, commonly 
     *          used as:
     *          - Write-only: transfer(data, len, nullptr, 0)
     *          - Read-only: transfer(nullptr, 0, data, len)
     *          - Write-then-read: transfer(reg_addr, 1, data, len) for register reads
     *          
     *          The operation is atomic from the I2C bus perspective (no other 
     *          device can interrupt between write and read phases).
     *          
     *          Common Patterns:
     *          - Register read: Write register address, read data bytes
     *          - Register write: Write register address followed by data
     *          - Burst read: Write starting address, read multiple registers
     *          
     * @note Each transfer incurs RPC latency (50-500μs) plus I2C transaction time
     * @note Retries automatically on failure up to configured retry count
     * @note Honors bus semaphore for thread-safe operation
     * 
     * @warning Ensure buffers are valid for entire operation. RPC may access 
     *          buffers asynchronously.
     *          
     * @see AP_HAL::Device::transfer()
     * @see set_retries() to configure retry behavior
     */
    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override;

    /**
     * @brief Read consecutive registers multiple times (not implemented)
     * 
     * @param[in]  first_reg Starting register address
     * @param[out] recv      Buffer to receive data
     * @param[in]  recv_len  Number of bytes to read per iteration
     * @param[in]  times     Number of times to repeat the read
     * 
     * @return Always returns false (not implemented)
     * 
     * @details Not implemented on QURT platform. Use transfer() in a loop instead.
     * 
     * @see AP_HAL::I2CDevice::read_registers_multiple()
     */
    bool read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                 uint32_t recv_len, uint8_t times) override
    {
        return false;
    };

    /**
     * @brief Register callback function for periodic device polling
     * 
     * @param[in] period_usec Callback period in microseconds
     * @param[in] cb          Callback function to execute periodically
     * 
     * @return Handle to registered callback (for later adjustment/removal)
     * 
     * @details Registers a callback function to be executed periodically by the 
     *          bus thread at the specified interval. This is the primary mechanism 
     *          for polling I2C sensors at regular rates.
     *          
     *          The callback runs in the I2C bus thread context at PRIORITY_I2C.
     *          Multiple devices on the same bus share the thread, so callbacks 
     *          execute sequentially (not concurrently).
     *          
     *          Typical usage: Sensor drivers register callbacks at 50Hz-400Hz to 
     *          read sensor data regularly (e.g., 100Hz for compass, 400Hz for IMU).
     *          
     * @note Actual callback rate may vary slightly based on RPC latency and other 
     *       device callbacks on the same bus
     *       
     * @note Callback should complete quickly (< 1ms) to avoid delaying other devices
     * 
     * @warning Do not block in callback. I2C transfers are acceptable, but long 
     *          delays affect all devices on the bus.
     *          
     * @see AP_HAL::Device::register_periodic_callback()
     * @see adjust_periodic_callback() to change period dynamically
     */
    AP_HAL::Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;

    /**
     * @brief Adjust period of previously registered callback
     * 
     * @param[in] h          Handle from register_periodic_callback()
     * @param[in] period_usec New callback period in microseconds
     * 
     * @return true if adjustment successful, false if handle invalid
     * 
     * @details Changes the execution period of a previously registered callback.
     *          Useful for dynamic rate adjustment based on vehicle state or 
     *          sensor requirements.
     *          
     * @note Period change takes effect on next callback invocation
     * 
     * @see AP_HAL::Device::adjust_periodic_callback()
     * @see register_periodic_callback()
     */
    bool adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec) override;

    /**
     * @brief Get bus semaphore for atomic multi-transfer operations
     * 
     * @return Pointer to bus semaphore
     * 
     * @details Returns the semaphore protecting this I2C bus. Acquire the semaphore 
     *          when performing multiple related transfers that must be atomic 
     *          (e.g., sensor configuration sequence).
     *          
     *          Usage pattern:
     *          @code
     *          AP_HAL::Semaphore *sem = dev->get_semaphore();
     *          if (sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
     *              // Perform multiple transfers atomically
     *              dev->transfer(...);
     *              dev->transfer(...);
     *              sem->give();
     *          }
     *          @endcode
     *          
     * @note Single transfer() calls automatically acquire/release semaphore
     * @note Manual semaphore use only needed for multi-transfer atomic sequences
     * 
     * @warning MUST call give() after take() to avoid deadlocking the bus
     * @warning All devices on same bus share this semaphore
     * 
     * @see AP_HAL::Device::get_semaphore()
     */
    AP_HAL::Semaphore* get_semaphore() override //TODO check all
    {
        // if asking for invalid bus number use bus 0 semaphore
        return &bus.semaphore;
    }

protected:
    /**
     * @brief Reference to I2C bus this device is attached to
     * 
     * @details Provides access to bus-level resources including semaphore, 
     *          file descriptor, and scheduling. All devices on the same bus 
     *          number share the same I2CBus instance.
     */
    I2CBus &bus;
    
    /**
     * @brief Number of retry attempts on transfer failure
     * 
     * @details Configurable via set_retries(). Default is typically 2 retries.
     *          Applied to all transfer operations.
     */
    uint8_t _retries;
    
    /**
     * @brief 7-bit I2C device address
     * 
     * @details Set in constructor, modifiable via set_address(). Used in all 
     *          I2C transfers to address this specific device on the bus.
     */
    uint8_t _address;
    
    /**
     * @brief Device name string for debugging/logging
     * 
     * @details Used in error messages and debug output to identify device.
     *          Typically contains bus number and address (e.g., "I2C0:0x1E").
     */
    char *pname;

};

/**
 * @class I2CDeviceManager
 * @brief Factory for creating and managing I2C device instances on QURT platform
 * 
 * @details This class implements the AP_HAL::I2CDeviceManager interface to provide 
 *          I2C device creation and bus management for the Qualcomm Hexagon DSP 
 *          platform. It maintains the array of I2CBus instances and creates 
 *          I2CDevice objects for specific bus/address combinations.
 *          
 *          Device Creation Pattern:
 *          Sensor drivers request I2C devices from the manager, which either 
 *          creates a new device or returns an existing one. The manager ensures 
 *          bus instances are properly initialized and shared across devices.
 *          
 *          Typical Usage:
 *          @code
 *          // Get I2C device manager (singleton)
 *          AP_HAL::I2CDeviceManager *i2c_mgr = hal.i2c_mgr;
 *          
 *          // Create device for compass at address 0x1E on bus 0, 400kHz
 *          AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev = i2c_mgr->get_device(0, 0x1E, 400000);
 *          
 *          // Check bus availability
 *          uint32_t available_buses = i2c_mgr->get_bus_mask();
 *          if (available_buses & (1 << 0)) {
 *              // Bus 0 is available
 *          }
 *          @endcode
 *          
 * @note Bus Mask: Bitmask indicates which I2C buses are available on the platform.
 *       For example, mask 0x03 means buses 0 and 1 are available (bits 0 and 1 set).
 *       
 * @note External vs Internal: Platform may distinguish between internal buses 
 *       (onboard sensors) and external buses (expansion ports). Query with 
 *       get_bus_mask_external() and get_bus_mask_internal().
 *       
 * @see AP_HAL::I2CDeviceManager for interface contract
 * @see I2CDevice for created device instances
 */
class I2CDeviceManager : public AP_HAL::I2CDeviceManager
{
public:
    friend class I2CDevice;

    /**
     * @brief Static array of I2C bus instances
     * 
     * @details Contains one I2CBus instance per available I2C bus on the platform.
     *          Shared across all I2CDevice instances on the same bus number.
     *          Initialized at startup based on platform configuration.
     */
    static I2CBus businfo[];

    /**
     * @brief Construct I2C device manager
     * 
     * @details Initializes the I2C device manager and prepares bus infrastructure.
     *          Called once at HAL initialization.
     */
    I2CDeviceManager();

    /**
     * @brief Cast AP_HAL::I2CDeviceManager pointer to QURT::I2CDeviceManager
     * 
     * @param[in] i2c_mgr Pointer to base I2CDeviceManager interface
     * @return QURT::I2CDeviceManager pointer for platform-specific access
     * 
     * @note Used internally when platform-specific operations are needed
     */
    static I2CDeviceManager *from(AP_HAL::I2CDeviceManager *i2c_mgr)
    {
        return static_cast<I2CDeviceManager*>(i2c_mgr);
    }

    /**
     * @brief Create or retrieve I2C device for specified bus and address
     * 
     * @param[in] bus        I2C bus number (0, 1, 2, etc.)
     * @param[in] address    7-bit I2C device address (0x00-0x7F)
     * @param[in] bus_clock  I2C bus clock speed in Hz (default: 100000 = 100kHz)
     * @param[in] use_smbus  Enable SMBus protocol extensions (default: false)
     * @param[in] timeout_ms RPC timeout in milliseconds (default: 4ms)
     * 
     * @return Pointer to I2CDevice instance, or nullptr if bus invalid
     * 
     * @details Creates a new I2CDevice instance for the specified bus/address 
     *          combination. This is the primary method for sensor drivers to 
     *          obtain I2C device handles.
     *          
     *          Common bus_clock values:
     *          - 100000 (100kHz): Standard mode, maximum compatibility
     *          - 400000 (400kHz): Fast mode, higher throughput for capable devices
     *          
     *          The device is not initialized until first transfer operation.
     *          
     * @note Multiple calls with same bus/address create independent device objects.
     *       Sharing devices between drivers is typically not needed.
     *       
     * @note Verify bus is available with get_bus_mask() before creating device
     * 
     * @warning Invalid bus number returns nullptr. Check return value before use.
     * @warning Ensure device address matches hardware. Incorrect address causes 
     *          NACK on all transfers.
     *          
     * @see AP_HAL::I2CDeviceManager::get_device()
     * @see get_bus_mask() to query available buses
     */
    AP_HAL::I2CDevice *get_device_ptr(uint8_t bus, uint8_t address,
            uint32_t bus_clock=100000,
            bool use_smbus = false,
            uint32_t timeout_ms=4) override;

    /**
     * @brief Get bitmask of all configured I2C buses
     * 
     * @return Bitmask where bit N set indicates bus N is available
     * 
     * @details Returns a bitmask indicating which I2C bus numbers are available 
     *          on this platform. Bit position corresponds to bus number.
     *          
     *          Example interpretations:
     *          - 0x01 (binary 0001): Only bus 0 available
     *          - 0x03 (binary 0011): Buses 0 and 1 available
     *          - 0x05 (binary 0101): Buses 0 and 2 available
     *          
     *          Use to check bus availability before creating devices or to 
     *          enumerate all available buses for device probing.
     *          
     * @note Includes both internal and external buses
     * 
     * @see AP_HAL::I2CDeviceManager::get_bus_mask()
     * @see get_bus_mask_external() for external-only buses
     * @see get_bus_mask_internal() for internal-only buses
     */
    uint32_t get_bus_mask(void) const override;

    /**
     * @brief Get bitmask of external I2C buses (expansion ports)
     * 
     * @return Bitmask where bit N set indicates bus N is external
     * 
     * @details Returns bitmask of I2C buses exposed on external connectors for 
     *          connecting peripheral sensors, displays, or expansion modules.
     *          
     *          External buses are typically used for:
     *          - User-added sensors (rangefinders, GPS+compass combos)
     *          - Expansion modules
     *          - External displays or indicators
     *          
     * @note Subset of get_bus_mask() - only includes externally accessible buses
     * 
     * @see AP_HAL::I2CDeviceManager::get_bus_mask_external()
     * @see get_bus_mask() for all buses
     */
    uint32_t get_bus_mask_external(void) const override;

    /**
     * @brief Get bitmask of internal I2C buses (onboard devices)
     * 
     * @return Bitmask where bit N set indicates bus N is internal
     * 
     * @details Returns bitmask of I2C buses connected to onboard sensors that are 
     *          integral to the flight controller hardware.
     *          
     *          Internal buses are typically used for:
     *          - Onboard IMU sensors
     *          - Integrated barometers
     *          - Internal compass (if present)
     *          
     * @note Subset of get_bus_mask() - only includes internal buses
     * @note Internal buses may run at higher speeds or have different electrical 
     *       characteristics than external buses
     *       
     * @see AP_HAL::I2CDeviceManager::get_bus_mask_internal()
     * @see get_bus_mask() for all buses
     */
    uint32_t get_bus_mask_internal(void) const override;
};
}
