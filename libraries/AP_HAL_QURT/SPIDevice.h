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
 * @file SPIDevice.h
 * @brief QURT (Qualcomm Hexagon DSP) SPI device driver implementation
 * 
 * @details This file implements the SPI device interface for the Qualcomm Hexagon DSP platform.
 *          The QURT HAL uses an RPC (Remote Procedure Call) model where the DSP communicates
 *          with the applications processor to access physical SPI hardware via the sl_client_spi_transfer
 *          API. This is necessary because SPI controllers are located on the apps processor, not the DSP.
 *          
 *          Architecture:
 *          - SPIBus: Represents a single SPI bus with multiple devices
 *          - SPIDevice: Individual SPI device (sensor) on the bus
 *          - SPIDeviceManager: Factory for creating SPI device instances
 *          
 *          All SPI operations involve inter-processor communication which adds 50-500us latency
 *          per transfer compared to direct hardware access on other platforms.
 * 
 * @note Physical SPI controllers are on the applications processor, not the Hexagon DSP
 * @warning RPC overhead makes this unsuitable for very high-rate polling (>1kHz)
 * 
 * @see DeviceBus.h for base bus scheduling and callback management
 * @see interface.h for sl_client_spi_transfer RPC function declaration
 */

#pragma once

#include <inttypes.h>
#include <AP_HAL/HAL.h>
#include <AP_HAL/SPIDevice.h>
#include "AP_HAL_QURT.h"

#include "Semaphores.h"
#include "Scheduler.h"
#include "DeviceBus.h"

namespace QURT
{

/**
 * @class SPIBus
 * @brief Represents a single SPI bus on the Hexagon DSP platform with device management
 * 
 * @details SPIBus extends DeviceBus to provide SPI-specific bus handling with device list
 *          management and periodic callback scheduling. Each SPIBus instance manages multiple
 *          SPIDevice instances that share the same physical SPI bus.
 *          
 *          Bus Architecture:
 *          - Each SPIBus manages a list of SPIDevice instances
 *          - Devices register periodic callbacks for sensor polling
 *          - Bus thread invokes device callbacks sequentially based on requested update rates
 *          - All devices on the same bus share access through semaphore protection
 *          
 *          Scheduling Model:
 *          - Bus runs its own thread at HAL_QURT_DEVICE_STACK_SIZE (8KB stack)
 *          - Thread priority is configurable per platform
 *          - Callbacks are invoked in order based on their registered periods
 *          - If a callback takes too long, subsequent callbacks may be delayed
 *          
 *          Thread Safety:
 *          - All bus operations protected by semaphore (inherited from DeviceBus)
 *          - Multiple devices can safely share the same bus
 *          - Semaphore must be held for atomic multi-transfer operations
 * 
 * @note Bus thread runs at HAL_QURT_DEVICE_STACK_SIZE (8KB stack) with configurable priority
 * @note All device callbacks execute in the context of the bus thread
 * @warning Long-running callbacks can impact other devices on the same bus
 * 
 * @see DeviceBus for base class implementation and scheduling details
 */
class SPIBus : public DeviceBus
{
public:
    SPIBus();
    int fd = -1;  ///< File descriptor for SPI bus, -1 if not opened
};

/**
 * @class SPIDevice
 * @brief SPI device driver for sensors on SPI bus using RPC to applications processor
 * 
 * @details SPIDevice implements the AP_HAL::SPIDevice interface for the QURT (Hexagon DSP)
 *          platform. All SPI operations use sl_client_spi_transfer RPC calls to the apps
 *          processor where the physical SPI hardware controllers are located.
 *          
 *          Transfer Model:
 *          - All SPI operations (read/write/transfer) involve RPC to apps processor
 *          - Each transfer includes: RPC request → apps processor SPI access → RPC response
 *          - Typical RPC overhead: 50-500us depending on system load
 *          - Transfers are blocking calls that wait for RPC completion
 *          
 *          Chip Select Handling:
 *          - CS pin controlled via RPC to apps processor GPIO
 *          - CS asserted before transfer, deasserted after transfer complete
 *          - CS timing follows standard SPI protocol
 *          
 *          Typical Usage Pattern:
 *          @code
 *          // IMU driver initialization
 *          AP_HAL::SPIDevice *dev = hal.spi->get_device("mpu9250");
 *          dev->set_speed(AP_HAL::Device::SPEED_HIGH);
 *          dev->register_periodic_callback(1000, FUNCTOR_BIND_MEMBER(&IMU::update, void));
 *          
 *          // In periodic callback
 *          void IMU::update() {
 *              uint8_t cmd = READ_ACCEL;
 *              uint8_t data[6];
 *              dev->transfer(&cmd, 1, data, sizeof(data));
 *              // Process accelerometer data...
 *          }
 *          @endcode
 *          
 *          Performance Considerations:
 *          - RPC latency adds 50-500us per transfer
 *          - Not suitable for very high-rate polling (>1kHz)
 *          - For high-bandwidth sensors, consider DMA if available to reduce CPU usage
 *          - Batch multiple register reads into single transfer when possible
 * 
 * @note RPC latency: Each SPI transfer has 50-500us overhead from inter-processor communication
 * @note SPI hardware: Physical SPI controllers on apps processor, all DSP access is via sl_client_* RPC layer
 * @note Thread safety: All SPI operations use bus semaphore, safe to call from multiple threads
 * @warning Performance: Not suitable for very high-rate polling (>1kHz), DSP memory access is faster via DMA when available
 * 
 * @see AP_HAL::SPIDevice for the interface definition
 * @see DeviceBus.h for callback scheduling implementation
 */
class SPIDevice : public AP_HAL::SPIDevice
{
public:
    /**
     * @brief Construct SPI device for named sensor on specified bus
     * 
     * @param name Device name string (e.g., "mpu9250", "bmi088_g", "icm42688")
     * @param bus Reference to SPIBus instance this device is attached to
     * 
     * @note Device name must match board configuration for device lookup to succeed
     */
    SPIDevice(const char *name, SPIBus &bus);
    
    /**
     * @brief Destructor - cleans up device resources and unregisters callbacks
     */
    virtual ~SPIDevice();

    /**
     * @brief Set SPI bus clock speed for this device
     * 
     * @param[in] speed Speed setting (SPEED_LOW or SPEED_HIGH)
     * @return true if speed was set successfully, false on RPC error
     * 
     * @note Issues RPC to configure hardware SPI controller on apps processor
     * @note Actual clock frequency depends on apps processor SPI controller configuration
     */
    bool set_speed(AP_HAL::Device::Speed speed) override;
    
    /**
     * @brief Perform half-duplex SPI transfer (separate send and receive buffers)
     * 
     * @details Performs a split SPI transaction: first transmits all send bytes,
     *          then receives all receive bytes. Uses sl_client_spi_transfer RPC
     *          to access physical SPI hardware on apps processor.
     * 
     * @param[in]  send     Buffer containing data to transmit (can be NULL if send_len is 0)
     * @param[in]  send_len Number of bytes to send
     * @param[out] recv     Buffer to receive data (can be NULL if recv_len is 0)
     * @param[in]  recv_len Number of bytes to receive
     * @return true if transfer completed successfully, false on error
     * 
     * @note Blocking call - includes RPC roundtrip time (50-500us)
     * @note CS pin asserted before transfer, deasserted after completion
     * @warning Total transfer time = RPC overhead + (send_len + recv_len) / SPI_clock_rate
     */
    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override;
    
    /**
     * @brief Perform full-duplex SPI transfer (simultaneous send and receive)
     * 
     * @details Performs simultaneous SPI send and receive using sl_client_spi_transfer
     *          with bidirectional data. Each clock cycle transmits one bit and receives
     *          one bit, typical for SPI sensor register reads (send register address,
     *          receive register value).
     * 
     * @param[in]  send Buffer containing data to transmit
     * @param[out] recv Buffer to receive data (same length as send)
     * @param[in]  len  Number of bytes to transfer in both directions
     * @return true if transfer completed successfully, false on error
     * 
     * @note Blocking call - includes RPC roundtrip time (50-500us)
     * @note Uses sl_client_spi_transfer with bidirectional data
     * @note CS pin asserted before transfer, deasserted after completion
     */
    bool transfer_fullduplex(const uint8_t *send, uint8_t *recv, uint32_t len) override;
    
    /**
     * @brief Get bus semaphore for atomic multi-transfer operations
     * 
     * @details Returns pointer to the bus semaphore which must be acquired before
     *          performing multiple atomic transfers. The semaphore ensures exclusive
     *          bus access and prevents other devices from interleaving transfers.
     *          
     *          Usage pattern:
     *          @code
     *          AP_HAL::Semaphore *sem = dev->get_semaphore();
     *          if (sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
     *              dev->transfer(...);  // First transfer
     *              dev->transfer(...);  // Second transfer (atomic with first)
     *              sem->give();
     *          }
     *          @endcode
     * 
     * @return Pointer to HAL_Semaphore for this device's bus
     * 
     * @warning Must call give() on semaphore after use to prevent bus lockup
     * @note Semaphore is shared by all devices on the same bus
     * @note Individual transfers automatically acquire/release semaphore internally
     */
    AP_HAL::Semaphore *get_semaphore() override;
    
    /**
     * @brief Register callback for periodic execution at specified rate
     * 
     * @details Registers a callback function to be invoked periodically by the bus thread.
     *          The callback is typically used for polling sensor data at regular intervals.
     *          Callbacks execute sequentially in the bus thread context based on their
     *          requested periods.
     * 
     * @param[in] period_usec Period in microseconds between callback invocations
     * @param[in] cb          Callback function to invoke (typically a FUNCTOR_BIND_MEMBER)
     * @return Handle to the periodic callback, or nullptr on failure
     * 
     * @note Callback invoked from bus thread context
     * @note Typical sensor polling rates: IMU 1000us (1kHz), baro 20000us (50Hz)
     * @note Long-running callbacks can delay other devices on the same bus
     * @warning Do not perform blocking operations in callback (already in bus thread)
     * 
     * @see adjust_periodic_callback() to change callback rate after registration
     */
    AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;
    
    /**
     * @brief Adjust period of previously registered periodic callback
     * 
     * @param[in] h           Handle returned by register_periodic_callback()
     * @param[in] period_usec New period in microseconds
     * @return true if period was adjusted successfully, false if handle invalid
     * 
     * @note Useful for adaptive sampling rates based on vehicle state
     */
    bool adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec) override;

private:
    SPIBus &bus;            ///< Reference to parent SPI bus this device is attached to
    const char *pname;      ///< Device name string for identification
    
    /**
     * @brief Acquire or release bus for transfer operation
     * 
     * @param[in] accuire true to acquire bus (assert CS), false to release (deassert CS)
     * 
     * @note Internal method called by transfer functions to control chip select
     */
    void acquire_bus(bool accuire);
};

/**
 * @class SPIDeviceManager
 * @brief Factory for creating and managing SPI device instances
 * 
 * @details SPIDeviceManager provides the get_device() method (via get_device_ptr) to create
 *          SPIDevice instances for named devices. The manager maintains the mapping between
 *          device names and their bus/configuration details.
 *          
 *          Device Naming:
 *          - Devices identified by string names (e.g., "bmi088_g", "icm42688", "mpu9250")
 *          - Names must match board configuration in hwdef or device table
 *          - Case-sensitive matching
 *          
 *          Device Lookup Process:
 *          1. Search available SPI devices on platform
 *          2. Match requested name against device table
 *          3. Create SPIDevice instance if found
 *          4. Return nullptr if device not found or already in use
 *          
 *          Typical Usage:
 *          @code
 *          // Get SPI device manager from HAL
 *          AP_HAL::SPIDeviceManager *mgr = hal.spi;
 *          
 *          // Request device by name
 *          AP_HAL::SPIDevice *imu = mgr->get_device("mpu9250");
 *          if (imu != nullptr) {
 *              // Configure and use device
 *              imu->set_speed(AP_HAL::Device::SPEED_HIGH);
 *              imu->register_periodic_callback(1000, callback);
 *          }
 *          @endcode
 * 
 * @note Device names are platform-specific and defined in board configuration
 * @note Returns nullptr if device not found or unavailable
 * 
 * @see SPIDevice for individual device usage
 * @see AP_HAL::SPIDeviceManager for interface definition
 */
class SPIDeviceManager : public AP_HAL::SPIDeviceManager
{
public:
    friend class SPIDevice;

    /**
     * @brief Get SPI device by name
     * 
     * @details Creates or returns existing SPIDevice instance for the named device.
     *          The device name must match an entry in the board's SPI device configuration.
     * 
     * @param[in] name Device name string (e.g., "mpu9250", "bmi088_g", "icm42688")
     * @return Pointer to SPIDevice instance, or nullptr if device not found
     * 
     * @note Device names are board-specific and case-sensitive
     * @note Returns nullptr if device doesn't exist in board configuration
     * @warning Do not delete returned pointer - managed by SPIDeviceManager
     */
    AP_HAL::SPIDevice *get_device_ptr(const char *name) override;
};
}

