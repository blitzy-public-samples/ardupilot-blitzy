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
 * @brief I2C bus interface for sensors and peripherals
 * 
 * @details Defines abstract interface for I2C device communication supporting multi-master,
 *          multi-device bus topologies. Handles 7-bit and 10-bit addressing, clock stretching,
 *          and bus error recovery. Platform implementations provide hardware I2C controller drivers.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2015-2016 Intel Corporation. Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <inttypes.h>

#include "AP_HAL_Namespace.h"
#include "Device.h"
#include "utility/OwnPtr.h"

namespace AP_HAL {

/**
 * @class AP_HAL::I2CDevice
 * @brief Abstract interface for I2C peripheral device access
 * 
 * @details Provides register read/write and block transfer operations for I2C devices.
 *          Inherits from Device base class for common functionality (checked registers,
 *          periodic callbacks, bus locking).
 *          
 *          I2C characteristics:
 *          - Multi-device bus: Multiple devices share SCL/SDA lines
 *          - 7-bit addressing: Device addresses 0x00-0x7F (address 0 is broadcast)
 *          - Clock stretching: Slow devices can hold SCL low to delay master
 *          - Bus recovery: Handles stuck-low SDA condition with clock pulses
 *          
 *          Typical workflow:
 *          1. get_device() from I2CDeviceManager with bus/address
 *          2. set_speed() for device-specific timing (Standard 100kHz, Fast 400kHz)
 *          3. get_semaphore() and lock for exclusive bus access
 *          4. transfer() or read_registers() for data exchange
 *          5. Unlock semaphore
 *          6. Optional: register_periodic_callback() for background polling
 * 
 * @note All operations assume 7-bit addressing unless platform explicitly supports 10-bit
 * @note Bus speed affects all devices on shared bus - use slowest device's max speed
 * @warning Must hold bus semaphore during transfers to prevent conflicts with other devices
 * @warning I2C buses may lock up if devices hold SDA low - requires bus recovery
 * 
 * @see Device
 * @see I2CDeviceManager
 */
class I2CDevice : public Device {
public:
    I2CDevice() : Device(BUS_TYPE_I2C) { }

    virtual ~I2CDevice() { }

    /* Device implementation */

    /**
     * @brief Set I2C bus speed for this device
     * 
     * @details Configures I2C clock frequency for device-specific timing requirements.
     *          Common speeds:
     *          - SPEED_LOW: 100 kHz (Standard Mode)
     *          - SPEED_HIGH: 400 kHz (Fast Mode)
     *          - SPEED_VERY_HIGH: 1 MHz (Fast Mode Plus)
     *          - SPEED_ULTRA_HIGH: 3.4 MHz (High Speed Mode, rarely used)
     *          
     *          Bus speed affects all devices on the shared I2C bus during this device's
     *          transactions. Set to slowest speed required by any device on the bus.
     * 
     * @param[in] speed Desired bus speed from Device::Speed enumeration
     * 
     * @return bool true if speed successfully configured, false if unsupported
     * 
     * @note Must be called before transfer operations
     * @note Some platforms may ignore speed setting and use board-configured default
     * @warning Excessive speed may cause communication errors or device malfunction
     * 
     * @see Device::Speed
     * @see Device::set_speed()
     */
    virtual bool set_speed(Device::Speed speed) override = 0;

    /**
     * @brief Perform I2C read, write, or combined transaction
     * 
     * @details Executes I2C data transfer with device. Supports three transaction types:
     *          - Write-only: send != nullptr, recv == nullptr
     *          - Read-only: send == nullptr, recv != nullptr  
     *          - Combined write-then-read: both != nullptr (uses repeated start)
     *          
     *          Combined transactions are typical for register reads: write register address,
     *          then read data without releasing bus (repeated start prevents other devices
     *          from interfering).
     *          
     *          Transaction format:
     *          START → [ADDRESS+W] → [send data] → REPEATED_START → [ADDRESS+R] → [recv data] → STOP
     * 
     * @param[in] send Pointer to data to transmit (or nullptr for read-only)
     * @param[in] send_len Number of bytes to transmit (0 if send is nullptr)
     * @param[out] recv Pointer to buffer for received data (or nullptr for write-only)
     * @param[in] recv_len Number of bytes to receive (0 if recv is nullptr)
     * 
     * @return bool true if transaction successful, false on NAK, timeout, or bus error
     * 
     * @note Typical timing: 10-100μs per byte at 400kHz, longer at 100kHz
     * @note Combined transactions use repeated start (no stop between write and read)
     * @warning Must hold device semaphore during call to prevent bus conflicts
     * @warning Blocks until completion - do not call from time-critical interrupt handlers
     * @warning Bus errors may require power cycle or bus recovery procedure
     * 
     * @see Device::transfer()
     * @see get_semaphore()
     * @see set_split_transfers()
     */
    virtual bool transfer(const uint8_t *send, uint32_t send_len,
                          uint8_t *recv, uint32_t recv_len) override = 0;

    /**
     * @brief Read from device register multiple times with sequential storage
     * 
     * @details Performs repeated reads from the same starting register address, storing
     *          each read result sequentially in the buffer. Useful for reading FIFO buffers
     *          or burst-mode sensor data where each read automatically advances the device's
     *          internal address pointer.
     *          
     *          Example: Reading 6 bytes of IMU data 3 times:
     *          - Read 1: bytes 0-5 contain first sample
     *          - Read 2: bytes 6-11 contain second sample  
     *          - Read 3: bytes 12-17 contain third sample
     *          Total buffer size needed: recv_len × times bytes
     *          
     *          Common use case: Emptying sensor FIFO buffers that auto-increment on read
     * 
     * @param[in] first_reg Starting register address (8-bit)
     * @param[out] recv Pointer to buffer for received data (must be recv_len × times bytes)
     * @param[in] recv_len Number of bytes to read in each transaction
     * @param[in] times Number of consecutive read transactions to perform
     * 
     * @return bool true if all transactions successful, false on any failure
     * 
     * @note Buffer offset advances by recv_len after each read
     * @note Useful for FIFO registers that auto-increment on each read
     * @warning Must hold device semaphore during entire operation
     * @warning Total operation time = times × (transaction overhead + recv_len × byte_time)
     * 
     * @see transfer()
     */
    virtual bool read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                         uint32_t recv_len, uint8_t times) = 0;

    /**
     * @brief Get semaphore for exclusive bus access
     * 
     * @details Returns semaphore protecting I2C bus access. Must be locked before any
     *          transfer operations to prevent conflicts with other threads accessing
     *          devices on the same bus.
     *          
     *          I2C bus is shared resource - multiple devices on same physical bus lines.
     *          Semaphore prevents concurrent access that would corrupt transactions.
     *          
     *          Typical usage pattern:
     *          @code{.cpp}
     *          Semaphore *sem = device->get_semaphore();
     *          if (sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
     *              device->transfer(send, send_len, recv, recv_len);
     *              sem->give();
     *          }
     *          @endcode
     *          
     *          Or use convenience macro:
     *          @code{.cpp}
     *          WITH_SEMAPHORE(device->get_semaphore());
     *          device->transfer(send, send_len, recv, recv_len);
     *          @endcode
     * 
     * @return Semaphore* Pointer to bus semaphore (never nullptr)
     * 
     * @note Semaphore is per-bus, not per-device - all devices on same bus share semaphore
     * @note Periodic callbacks registered via register_periodic_callback() automatically
     *       acquire semaphore before callback execution
     * @warning Failing to hold semaphore during transfers causes bus conflicts and corruption
     * @warning Do not hold semaphore for extended periods - blocks other device access
     * 
     * @see Device::get_semaphore()
     * @see register_periodic_callback()
     */
    virtual Semaphore *get_semaphore() override = 0;

    /**
     * @brief Register periodic callback for background sensor polling
     * 
     * @details Registers callback function to be invoked periodically by I/O thread at
     *          specified interval. Callback automatically acquires bus semaphore before
     *          execution and releases after completion.
     *          
     *          Typical use: Polling sensors at fixed rate (e.g., 1kHz IMU sampling).
     *          
     *          Callback execution context:
     *          - Runs in I/O thread, not main thread
     *          - Bus semaphore held automatically
     *          - Should complete quickly (< 1ms typical)
     *          - No blocking operations (sleep, long I2C transfers)
     *          
     *          Example:
     *          @code{.cpp}
     *          auto handle = device->register_periodic_callback(1000, // 1ms = 1000μs
     *              FUNCTOR_BIND_MEMBER(&MyClass::read_sensor, void));
     *          @endcode
     * 
     * @param[in] period_usec Callback period in microseconds (1000 = 1ms, 1000000 = 1s)
     * @param[in] cb Callback function pointer or functor (Device::PeriodicCb type)
     * 
     * @return Device::PeriodicHandle Handle for adjusting or unregistering callback
     * 
     * @note Callback frequency limited by scheduler resolution (typically 1kHz max)
     * @note Bus semaphore acquired automatically before callback execution
     * @note Multiple callbacks can be registered on same device
     * @warning Callback must not block or perform long operations - blocks other device access
     * @warning Callback runs at interrupt priority level on some platforms
     * 
     * @see Device::register_periodic_callback()
     * @see adjust_periodic_callback()
     * @see Device::PeriodicCb
     */
    virtual Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, Device::PeriodicCb) override = 0;

    /**
     * @brief Adjust period of existing periodic callback
     * 
     * @details Modifies the callback period for an already-registered periodic callback
     *          without unregistering and re-registering. Useful for dynamically adjusting
     *          sensor sampling rates based on flight mode or performance requirements.
     *          
     *          Example use cases:
     *          - Reduce IMU rate during low-activity modes to save CPU
     *          - Increase rate during aggressive maneuvers for better control
     *          - Adapt to changing sensor requirements
     * 
     * @param[in] h Handle returned by register_periodic_callback()
     * @param[in] period_usec New callback period in microseconds
     * 
     * @return bool true if period successfully adjusted, false if handle invalid
     * 
     * @note New period takes effect on next scheduled callback invocation
     * @note Does not reset callback phase - next call still at originally scheduled time
     * @warning Invalid handle returns false - callback was never registered or already removed
     * 
     * @see Device::adjust_periodic_callback()
     * @see register_periodic_callback()
     */
    virtual bool adjust_periodic_callback(
        Device::PeriodicHandle h, uint32_t period_usec) override = 0;

    /**
     * @brief Force I2C transfers to split send and receive with stop condition
     * 
     * @details Forces combined write-then-read transfers to use separate transactions with
     *          full stop condition between write and read, instead of repeated start.
     *          
     *          Normal combined transfer (repeated start):
     *          START → [ADDR+W] → [write data] → REPEATED_START → [ADDR+R] → [read data] → STOP
     *          
     *          Split transfer (with stop):
     *          START → [ADDR+W] → [write data] → STOP
     *          START → [ADDR+R] → [read data] → STOP
     *          
     *          Required for devices that don't support repeated start condition or have
     *          unusual timing requirements. Most modern I2C devices support repeated start.
     *          
     *          Some platforms always use split transfers due to hardware limitations.
     * 
     * @param[in] set true to force split transfers, false for combined (repeated start)
     * 
     * @note Default implementation empty - platforms with split-only hardware ignore this
     * @note Allows using standard read_registers() methods with incompatible devices
     * @warning Split transfers release bus between write and read - other devices can interfere
     * @warning Increases transaction time and bus overhead compared to repeated start
     * 
     * @see transfer()
     */
    virtual void set_split_transfers(bool set) {};
};

/**
 * @class AP_HAL::I2CDeviceManager
 * @brief Factory for creating I2C device handles
 * 
 * @details Singleton manager accessed via hal.i2c_mgr provides I2C device instance creation
 *          and bus configuration queries. Manages I2C bus initialization, clock configuration,
 *          and device handle lifecycle.
 *          
 *          Responsibilities:
 *          - Create device handles for specific bus/address combinations
 *          - Configure bus clock speeds and timeouts
 *          - Provide bus availability masks (internal vs external buses)
 *          - Initialize hardware I2C controllers during HAL startup
 *          
 *          Bus numbering is platform-specific and defined in board hwdef files:
 *          - Bus 0: Typically internal I2C (on-board sensors)
 *          - Bus 1-3: Often external I2C (expansion connectors)
 *          - Actual availability depends on board hardware
 *          
 *          External buses (exposed on connectors) typically use higher numbers
 *          to allow expansion without conflicting with internal sensors.
 *          
 *          Access via global HAL singleton:
 *          @code{.cpp}
 *          auto dev = hal.i2c_mgr->get_device(1, 0x68); // Bus 1, address 0x68
 *          @endcode
 * 
 * @note Device handles are owned by caller - keep reference as long as device is needed
 * @note Bus mask methods allow driver probe loops across available buses
 * @warning Do not delete I2CDeviceManager instance - singleton managed by HAL
 * 
 * @see I2CDevice
 * @see hal
 */
class I2CDeviceManager {
public:
    /**
     * @brief Get raw pointer to newly-allocated I2C device handle
     * 
     * @details Allocates and returns raw pointer to I2CDevice for specified bus and address.
     *          Caller is responsible for lifetime management (deletion).
     *          
     *          Prefer get_device() which uses OwnPtr for automatic lifetime management.
     *          This method exists for compatibility with code requiring raw pointers.
     *          
     *          Creates device handle even if no device present at address - probe separately
     *          to verify device presence.
     * 
     * @param[in] bus I2C bus number (0-based, platform-specific numbering)
     * @param[in] address 7-bit I2C device address (0x00-0x7F)
     * @param[in] bus_clock Bus clock frequency in Hz (default 400000 = 400kHz Fast Mode)
     * @param[in] use_smbus Enable SMBus protocol extensions (PEC, block transfers) (default false)
     * @param[in] timeout_ms Transaction timeout in milliseconds (default 4ms)
     * 
     * @return I2CDevice* Pointer to device handle, or nullptr if bus invalid or allocation failed
     * 
     * @note Caller must delete returned pointer when finished with device
     * @note Bus clock affects all devices during this device's transactions
     * @note SMBus mode adds packet error checking and extended command set
     * @warning Raw pointer - manual lifetime management required
     * @warning Returns non-null even if device not physically present - check separately
     * 
     * @see get_device()
     */
    virtual AP_HAL::I2CDevice *get_device_ptr(uint8_t bus, uint8_t address,
                                              uint32_t bus_clock=400000,
                                              bool use_smbus = false,
                                              uint32_t timeout_ms=4) = 0;
    /**
     * @brief Get I2C device handle with automatic lifetime management
     * 
     * @details Returns OwnPtr-wrapped device handle for specified bus and address.
     *          OwnPtr provides automatic deletion when handle goes out of scope,
     *          eliminating manual lifetime management and memory leaks.
     *          
     *          Preferred method for device handle creation - automatically manages memory.
     *          
     *          Typical usage pattern:
     *          @code{.cpp}
     *          class MySensor {
     *              AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;
     *              
     *              bool init() {
     *                  dev = hal.i2c_mgr->get_device(1, 0x68, 400000);
     *                  if (!dev) {
     *                      return false;
     *                  }
     *                  return dev->transfer(init_sequence, sizeof(init_sequence), nullptr, 0);
     *              }
     *          };
     *          // Device automatically deleted when MySensor destroyed
     *          @endcode
     * 
     * @param[in] bus I2C bus number (0-based, platform-specific numbering from hwdef)
     * @param[in] address 7-bit I2C device address (0x00-0x7F)
     * @param[in] bus_clock Bus clock frequency in Hz (100000=100kHz, 400000=400kHz, default 400kHz)
     * @param[in] use_smbus Enable SMBus protocol extensions if supported (default false)
     * @param[in] timeout_ms Transaction timeout in milliseconds (default 4ms)
     * 
     * @return OwnPtr<I2CDevice> Smart pointer to device handle (nullptr if bus invalid)
     * 
     * @note OwnPtr automatically deletes device when handle destroyed or reassigned
     * @note Standard I2C speeds: 100kHz (Standard), 400kHz (Fast), 1MHz (Fast Plus)
     * @note Check if OwnPtr is null before use: if (!dev) { handle_error(); }
     * @warning Returns non-null even if device not present - verify with probe transfer
     * 
     * @see get_device_ptr()
     * @see AP_HAL::OwnPtr
     */
    OwnPtr<AP_HAL::I2CDevice> get_device(uint8_t bus, uint8_t address,
                                         uint32_t bus_clock=400000,
                                         bool use_smbus = false,
                                         uint32_t timeout_ms=4) {
        return AP_HAL::OwnPtr<AP_HAL::I2CDevice>(get_device_ptr(bus, address, bus_clock, use_smbus, timeout_ms));
    }
;
    /**
     * @brief Get bitmask of all configured I2C buses
     * 
     * @details Returns bitmask where each set bit represents an available I2C bus.
     *          Bit N set means bus N is available (bit 0 = bus 0, bit 1 = bus 1, etc.).
     *          
     *          Used for driver probe loops to enumerate all available buses:
     *          @code{.cpp}
     *          FOREACH_I2C(bus) {
     *              auto dev = hal.i2c_mgr->get_device(bus, MY_DEVICE_ADDR);
     *              if (probe_device(dev)) {
     *                  // Found device on this bus
     *              }
     *          }
     *          @endcode
     *          
     *          Bus availability determined by board hwdef configuration and hardware capabilities.
     * 
     * @return uint32_t Bitmask of available I2C buses (bit N = bus N available)
     * 
     * @note Default implementation returns 0x0F (buses 0-3), override in platform HAL
     * @note Maximum 32 buses supported (bits 0-31)
     * 
     * @see get_bus_mask_external()
     * @see get_bus_mask_internal()
     * @see FOREACH_I2C()
     */
    virtual uint32_t get_bus_mask(void) const { return 0x0F; }

    /**
     * @brief Get bitmask of external I2C buses (exposed on board connectors)
     * 
     * @details Returns bitmask of I2C buses available on external connectors for user-added
     *          peripherals (GPS, compass, rangefinder, etc.).
     *          
     *          External buses distinguished from internal buses to allow:
     *          - User parameter configuration (I2C_EXT_MASK parameter)
     *          - Selective probing of user-facing buses
     *          - Protection of internal sensor buses from interference
     *          
     *          Typical external bus usage:
     *          @code{.cpp}
     *          FOREACH_I2C_EXTERNAL(bus) {
     *              // Probe only external buses for user-connected devices
     *              probe_external_device(bus);
     *          }
     *          @endcode
     *          
     *          Bus classification defined in board hwdef file (external vs internal).
     * 
     * @return uint32_t Bitmask of external I2C buses (bit N = bus N is external)
     * 
     * @note Default implementation returns 0x0F (buses 0-3 external), override in platform HAL
     * @note External buses typically have connector pinouts and user documentation
     * @note Some boards have no external buses (all internal sensors)
     * 
     * @see get_bus_mask()
     * @see get_bus_mask_internal()
     * @see FOREACH_I2C_EXTERNAL()
     */
    virtual uint32_t get_bus_mask_external(void) const { return 0x0F; }

    /**
     * @brief Get bitmask of internal I2C buses (on-board sensors only)
     * 
     * @details Returns bitmask of I2C buses connected only to on-board sensors (IMU, compass,
     *          barometer, etc.) not exposed to user on external connectors.
     *          
     *          Internal buses used for:
     *          - Soldered on-board sensors
     *          - Pre-configured hardware with known sensor configurations
     *          - Protected buses isolated from user wiring errors
     *          
     *          Drivers can selectively probe internal buses for factory-configured sensors:
     *          @code{.cpp}
     *          FOREACH_I2C_INTERNAL(bus) {
     *              // Probe only internal buses for on-board sensors
     *              probe_onboard_sensor(bus);
     *          }
     *          @endcode
     *          
     *          Bus classification defined in board hwdef file during board porting.
     * 
     * @return uint32_t Bitmask of internal I2C buses (bit N = bus N is internal)
     * 
     * @note Default implementation returns 0x01 (bus 0 internal), override in platform HAL
     * @note Internal buses typically have lower numbers (bus 0 common)
     * @note Some boards are all-internal (no external I2C) or all-external (no on-board sensors)
     * 
     * @see get_bus_mask()
     * @see get_bus_mask_external()
     * @see FOREACH_I2C_INTERNAL()
     */
    virtual uint32_t get_bus_mask_internal(void) const { return 0x01; }
};

/**
 * @defgroup i2c_iteration_macros I2C Bus Iteration Macros
 * @brief Convenience macros for iterating over available I2C buses
 * 
 * @details Provide compact syntax for driver probe loops across available buses.
 *          Macros expand to for-loops that iterate only over bus numbers with
 *          corresponding bit set in the bus mask.
 *          
 *          Example usage:
 *          @code{.cpp}
 *          FOREACH_I2C_EXTERNAL(bus) {
 *              auto dev = hal.i2c_mgr->get_device(bus, DEVICE_ADDR);
 *              if (probe_device(dev)) {
 *                  printf("Found device on external bus %u\n", bus);
 *              }
 *          }
 *          @endcode
 * 
 * @{
 */

/**
 * @def FOREACH_I2C_MASK(i, mask)
 * @brief Iterate over I2C buses specified by bitmask
 * 
 * @param i Loop variable (uint32_t) receiving bus number (0-31)
 * @param mask Bitmask of buses to iterate (bit N = include bus N)
 * 
 * @note Base macro used by other FOREACH_I2C_* macros
 * @note Loop variable i declared by macro - do not declare separately
 */
#define FOREACH_I2C_MASK(i,mask) for (uint32_t _bmask=mask, i=0; i<32; i++) if ((1U<<i)&_bmask)

/**
 * @def FOREACH_I2C_EXTERNAL(i)
 * @brief Iterate over external I2C buses (user-facing connectors)
 * 
 * @param i Loop variable receiving bus number
 * 
 * @note Expands to FOREACH_I2C_MASK with external bus mask
 * @see I2CDeviceManager::get_bus_mask_external()
 */
#define FOREACH_I2C_EXTERNAL(i) FOREACH_I2C_MASK(i,hal.i2c_mgr->get_bus_mask_external())

/**
 * @def FOREACH_I2C_INTERNAL(i)
 * @brief Iterate over internal I2C buses (on-board sensors)
 * 
 * @param i Loop variable receiving bus number
 * 
 * @note Expands to FOREACH_I2C_MASK with internal bus mask
 * @see I2CDeviceManager::get_bus_mask_internal()
 */
#define FOREACH_I2C_INTERNAL(i) FOREACH_I2C_MASK(i,hal.i2c_mgr->get_bus_mask_internal())

/**
 * @def FOREACH_I2C(i)
 * @brief Iterate over all configured I2C buses (internal and external)
 * 
 * @param i Loop variable receiving bus number
 * 
 * @note Expands to FOREACH_I2C_MASK with all available buses
 * @see I2CDeviceManager::get_bus_mask()
 */
#define FOREACH_I2C(i) FOREACH_I2C_MASK(i,hal.i2c_mgr->get_bus_mask())

/** @} */ // end of i2c_iteration_macros group

}
