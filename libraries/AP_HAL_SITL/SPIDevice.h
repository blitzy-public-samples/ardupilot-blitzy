/*
 * Copyright (C) 2021 Peter Barker. All rights reserved.
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
 * @file SPIDevice.h
 * @brief SITL (Software In The Loop) SPI device simulation implementation
 * 
 * @details This file implements simulated SPI (Serial Peripheral Interface) devices for
 *          ArduPilot's SITL environment. Unlike real hardware implementations, these classes
 *          provide software simulation of SPI bus communication for testing and development.
 *          
 *          The simulation implements the spi_ioc_transfer semantics by routing SPI transactions
 *          through the AP::sitl()->spi_ioctl interface, which dispatches to sensor-specific
 *          simulation handlers registered in SITL_State.
 *          
 *          Key simulation characteristics:
 *          - Simulates multi-device SPI buses with chip select handling
 *          - Supports SPI_TRANSACTION_1LONG and SPI_TRANSACTION_2LONG transaction modes
 *          - Provides per-bus semaphores for thread-safe access
 *          - Routes transactions to simulated sensor implementations
 *          - Device registration via static SPIDesc tables
 *          
 * @warning This is a simulation and differs from real SPI hardware in several ways:
 *          - No actual timing constraints or clock speed enforcement
 *          - No real signal characteristics (voltage levels, rise/fall times)
 *          - Transfer speeds are not enforced (set_speed is a no-op)
 *          - Full-duplex transfers may have different behavior than real hardware
 *          
 * @note Simulated SPI devices must implement corresponding handlers in SITL_State to
 *       respond to SPI transactions. See libraries/SITL/SIM_*.cpp for examples.
 * 
 * @see AP_HAL::SPIDevice for the abstract interface this implements
 * @see SITL_State for the simulation backend that handles SPI transactions
 */
#pragma once

#include <inttypes.h>

#include <AP_HAL/HAL.h>
#include <AP_HAL/SPIDevice.h>

#include "Semaphores.h"

class SPIBus;

namespace HALSITL {

/**
 * @struct SPIDesc
 * @brief SPI device descriptor for SITL device registration
 * 
 * @details Describes a simulated SPI device for registration with the SPIDeviceManager.
 *          These descriptors are typically defined in static device_table arrays and
 *          specify the device name, bus number, and chip select pin used for device
 *          identification and selection during SPI transactions.
 *          
 *          The device name is matched against driver probe requests (e.g., "bmp280",
 *          "mpu9250"), the bus number identifies which simulated SPI bus the device
 *          is attached to, and the cs_pin distinguishes multiple devices on the same bus.
 * 
 * @note Device names must match those used in sensor driver probe functions
 * @see SPIDeviceManager::get_device_ptr() for device lookup by name
 */
struct SPIDesc {
    /**
     * @brief Construct an SPI device descriptor
     * 
     * @param[in] _name     Device name string for identification (e.g., "bmp280", "mpu9250")
     * @param[in] _bus      SPI bus number (0-based, typically 0-3 depending on board simulation)
     * @param[in] _cs_pin   Chip select pin number for device selection
     */
    SPIDesc(const char *_name, uint8_t _bus, uint8_t _cs_pin)
        : name(_name), bus(_bus), cs_pin(_cs_pin)
    { }

    const char *name;   ///< Device identification name
    uint8_t bus;        ///< SPI bus number
    uint8_t cs_pin;     ///< Chip select pin number
};

/**
 * @class SPIDevice
 * @brief Simulated SPI device implementation for SITL environment
 * 
 * @details Implements the AP_HAL::SPIDevice interface for software-in-the-loop simulation.
 *          This class provides a software simulation of SPI device communication, routing
 *          SPI transactions through the SITL backend (AP::sitl()->spi_ioctl) to simulated
 *          sensor implementations.
 *          
 *          Key features:
 *          - Per-device semaphore for thread-safe multi-tasking access
 *          - Chip select handling for bus sharing with multiple devices
 *          - Transaction routing to simulated sensors via SITL_State
 *          - Support for standard half-duplex SPI transfers
 *          - Periodic callback registration for sensor polling
 *          
 *          Transaction flow:
 *          1. Driver calls transfer() with send/receive buffers
 *          2. SPIDevice acquires bus semaphore for exclusive access
 *          3. Chip select is asserted (simulated via cs_pin in device_desc)
 *          4. Transaction forwarded to AP::sitl()->spi_ioctl() with bus/cs_pin info
 *          5. SITL backend dispatches to appropriate sensor simulation handler
 *          6. Simulated sensor populates receive buffer with response data
 *          7. Chip select deasserted, semaphore released
 *          
 * @warning This simulation does not enforce real SPI timing constraints:
 *          - Clock speeds are not simulated (set_speed is a no-op)
 *          - No inter-byte delays or setup/hold times
 *          - Instantaneous transaction completion
 *          - No signal integrity simulation
 *          
 * @warning Full-duplex transfers (transfer_fullduplex) are not implemented and will
 *          abort if called. Use standard half-duplex transfer() method instead.
 * 
 * @note Each SPIDevice instance maintains its own semaphore, but devices on the same
 *       bus share bus-level synchronization through the SPIBus object.
 * 
 * @see AP_HAL::SPIDevice for the abstract interface definition
 * @see SITL_State::spi_ioctl() for the backend transaction handler
 */
class SPIDevice : public AP_HAL::SPIDevice {
public:
    /**
     * @brief Construct a simulated SPI device
     * 
     * @param[in] _bus          Reference to the SPIBus this device is attached to
     * @param[in] _device_desc  Device descriptor containing name, bus number, and chip select pin
     */
    SPIDevice(SPIBus &_bus, SPIDesc &_device_desc);

    /**
     * @brief Set the SPI bus speed (no-op in SITL simulation)
     * 
     * @details In SITL, there is no concept of actual bus speed or timing constraints.
     *          This method always succeeds but does not affect transaction timing.
     *          Real hardware implementations would configure clock dividers here.
     * 
     * @param[in] speed  Requested bus speed (AP_HAL::Device::SPEED_LOW or SPEED_HIGH)
     * 
     * @return Always returns true (speed change always "succeeds")
     * 
     * @warning No actual speed enforcement occurs. All transactions complete instantaneously
     *          regardless of the requested speed setting.
     * 
     * @note This is intentionally a no-op to simplify SITL simulation while maintaining
     *       API compatibility with real hardware HAL implementations.
     */
    // no concept of speed in SITL yet
    bool set_speed(AP_HAL::Device::Speed speed) override { return true; }

    /**
     * @brief Perform a half-duplex SPI transaction with separate send and receive buffers
     * 
     * @details Simulates an SPI transaction by routing data through the SITL backend.
     *          The transaction supports separate send and receive phases (half-duplex):
     *          1. If send buffer provided: transmit send_len bytes to device
     *          2. If recv buffer provided: receive recv_len bytes from device
     *          
     *          Transaction types supported:
     *          - SPI_TRANSACTION_1LONG: send_len == 1, recv_len > 0 (register read)
     *          - SPI_TRANSACTION_2LONG: send_len == 2, recv_len > 0 (register read with 16-bit address)
     *          - Write transactions: send_len > 0, recv_len == 0
     *          
     *          The transaction is forwarded to AP::sitl()->spi_ioctl() which dispatches
     *          to the appropriate simulated sensor handler based on bus number and chip select pin.
     * 
     * @param[in]  send      Pointer to transmit buffer (can be nullptr if send_len == 0)
     * @param[in]  send_len  Number of bytes to transmit
     * @param[out] recv      Pointer to receive buffer (can be nullptr if recv_len == 0)
     * @param[in]  recv_len  Number of bytes to receive
     * 
     * @return true if transaction completed successfully, false on error
     * 
     * @note This is the primary SPI transfer method used by sensor drivers
     * @note Actual timing is instantaneous - no delays between bytes
     * 
     * @see do_transfer() for the internal implementation
     * @see SITL_State::spi_ioctl() for backend transaction handling
     */
    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override;

    /**
     * @brief Full-duplex SPI transfer (NOT IMPLEMENTED in SITL)
     * 
     * @details Full-duplex SPI transfers (simultaneous transmit and receive) are not
     *          currently implemented in the SITL simulation. Calling this method will
     *          abort the program.
     * 
     * @param[in]  send  Transmit buffer
     * @param[out] recv  Receive buffer
     * @param[in]  len   Number of bytes to transfer
     * 
     * @return Never returns (aborts)
     * 
     * @warning DO NOT CALL - Will abort program execution
     * @warning Use half-duplex transfer() method instead
     */
    bool transfer_fullduplex(const uint8_t *send, uint8_t *recv,
                             uint32_t len) override {
        abort();
    }

    /**
     * @brief Full-duplex SPI transfer with single buffer (NOT IMPLEMENTED in SITL)
     * 
     * @details Full-duplex SPI transfers (simultaneous transmit and receive) are not
     *          currently implemented in the SITL simulation. Calling this method will
     *          abort the program.
     * 
     * @param[in,out] send_recv  Buffer for both transmit and receive data
     * @param[in]     len        Number of bytes to transfer
     * 
     * @return Never returns (aborts)
     * 
     * @warning DO NOT CALL - Will abort program execution
     * @warning Use half-duplex transfer() method instead
     */
    bool transfer_fullduplex(uint8_t *send_recv, uint32_t len) override {
        abort();
    }

    /**
     * @brief Get the semaphore for this SPI device
     * 
     * @details Returns a pointer to the device's semaphore for thread-safe access.
     *          Drivers should acquire this semaphore (WITH_SEMAPHORE macro) before
     *          performing SPI transactions to ensure exclusive device access during
     *          multi-step operations.
     *          
     *          Each SPIDevice has its own semaphore instance, allowing concurrent access
     *          to different devices on different buses while serializing access to the
     *          same device.
     * 
     * @return Pointer to the device's AP_HAL::Semaphore object
     * 
     * @note Drivers typically use WITH_SEMAPHORE(_dev->get_semaphore()) for automatic
     *       semaphore acquisition and release
     * 
     * @see AP_HAL::Semaphore for semaphore usage patterns
     */
    AP_HAL::Semaphore *get_semaphore() override;

    /**
     * @brief Register a periodic callback function for this device
     * 
     * @details Registers a function to be called periodically at the specified rate.
     *          This is commonly used by sensor drivers to poll sensors at regular intervals.
     *          The callback is executed by the SITL scheduler.
     *          
     *          Multiple callbacks can be registered for the same device at different rates.
     *          Callbacks should be short and non-blocking to avoid affecting simulation timing.
     * 
     * @param[in] period_usec  Period between callback invocations in microseconds
     * @param[in] cb           Callback function to invoke periodically
     * 
     * @return Handle to the registered callback (can be used to unregister if needed)
     * 
     * @note Callbacks run in the SITL scheduler context - avoid long operations
     * @note Typical sensor polling rates: 100Hz (10000 usec), 400Hz (2500 usec), 1000Hz (1000 usec)
     * 
     * @see AP_HAL::Device::PeriodicCb for callback function signature
     */
    AP_HAL::Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;

private:
    SPIBus &bus;               ///< Reference to the SPI bus this device is attached to
    SPIDesc &device_desc;      ///< Device descriptor with name, bus number, and chip select pin

    Semaphore _semaphore;      ///< Per-device semaphore for thread-safe access

    /**
     * @brief Internal method to perform the actual SPI transfer
     * 
     * @details Handles the low-level SPI transaction by routing to the SITL backend.
     *          This is called by the public transfer() method after buffer validation.
     * 
     * @param[in]  send  Pointer to transmit buffer
     * @param[out] recv  Pointer to receive buffer
     * @param[in]  len   Number of bytes to transfer
     * 
     * @return true if transfer completed successfully, false on error
     */
    bool do_transfer(const uint8_t *send, uint8_t *recv, uint32_t len);
};

/**
 * @class SPIDeviceManager
 * @brief Manager for simulated SPI devices in SITL environment
 * 
 * @details Implements the AP_HAL::SPIDeviceManager interface for SITL, providing device
 *          lookup and instantiation services. The manager maintains a static device_table
 *          that defines all available simulated SPI devices and their bus/chip-select
 *          assignments.
 *          
 *          Device registration and lookup:
 *          1. Static device_table[] contains SPIDesc entries for all simulated devices
 *          2. Sensor drivers call get_device_ptr("device_name") during probe
 *          3. Manager searches device_table for matching name
 *          4. If found, creates SPIDevice instance attached to appropriate bus
 *          5. Returns device pointer to driver for subsequent operations
 *          
 *          The manager also maintains static SPIBus objects for each simulated SPI bus,
 *          which handle bus-level semaphores and device coordination.
 * 
 * @note The device_table must be populated with all devices that should be visible
 *       to sensor drivers during their probe/detection phase. Devices not in this
 *       table will not be discoverable.
 * 
 * @note Each simulated board configuration (e.g., SITL for Copter, Plane) may have
 *       different device tables reflecting different sensor configurations.
 * 
 * @see AP_HAL::SPIDeviceManager for the abstract interface definition
 * @see SPIDesc for device descriptor format
 * @see SPIDevice for individual device implementation
 */
class SPIDeviceManager : public AP_HAL::SPIDeviceManager {
public:

    /**
     * @brief Construct the SPI device manager
     * 
     * @details Initializes the SITL SPI device manager and prepares the device table
     *          and bus structures for device lookup operations.
     */
    SPIDeviceManager();

    /**
     * @brief Get a pointer to an SPI device by name
     * 
     * @details Searches the static device_table for a device with the specified name.
     *          If found, creates a new SPIDevice instance attached to the appropriate
     *          bus and returns a pointer to it. If not found, returns nullptr.
     *          
     *          This is the primary method used by sensor drivers during their probe/detect
     *          phase to obtain device handles. Typical usage:
     *          
     *              AP_HAL::SPIDevice *dev = hal.spi->get_device_ptr("bmp280");
     *              if (dev) { /* device found, proceed with initialization */ }
     * 
     * @param[in] name  Device name string to search for (e.g., "bmp280", "mpu9250", "icm20948")
     * 
     * @return Pointer to SPIDevice instance if device found in device_table, nullptr if not found
     * 
     * @note The caller typically does not need to delete the returned device - it is managed
     *       by the HAL lifecycle
     * 
     * @note Device names are case-sensitive and must exactly match entries in device_table
     * 
     * @see SPIDesc::name for device naming conventions
     */
    AP_HAL::SPIDevice *get_device_ptr(const char *name) override;

    static SPIDesc device_table[];  ///< Static table of all available simulated SPI devices
    static SPIBus *buses;           ///< Array of SPIBus objects for bus-level management
};

}
