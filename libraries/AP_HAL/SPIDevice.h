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
 * @file SPIDevice.h
 * @brief Serial Peripheral Interface (SPI) bus interface for high-speed sensors
 * 
 * Defines abstract interface for SPI device communication supporting multi-device buses
 * with hardware chip select management. SPI provides faster data rates than I2C (MHz vs kHz)
 * and is preferred for high-bandwidth sensors (IMUs, barometers, magnetometers).
 * 
 * SPI is a synchronous, full-duplex master-slave protocol where the flight controller acts
 * as bus master and sensors act as slaves. Multiple devices share MOSI, MISO, and SCK lines,
 * with individual chip select (CS) pins for device selection.
 * 
 * @note Part of Hardware Abstraction Layer (HAL) - platform implementations in AP_HAL_ChibiOS, AP_HAL_Linux, etc.
 * @see Device.h for base device interface
 */

#pragma once

#include <inttypes.h>

#include "AP_HAL_Namespace.h"
#include "Device.h"
#include "utility/OwnPtr.h"

namespace AP_HAL {

/**
 * @class SPIDevice
 * @brief Abstract interface for SPI peripheral device access
 * 
 * @details Provides full-duplex transfer operations with hardware chip select control.
 *          Inherits from Device base class for common functionality (checked registers,
 *          periodic callbacks, bus locking).
 *          
 *          SPI characteristics:
 *          - Full-duplex: Simultaneous transmit and receive on MOSI/MISO lines
 *          - Master-slave: Flight controller is always master, sensors are slaves
 *          - Chip select (CS): One CS pin per device, active-low selects device
 *          - Clock polarity/phase: Four modes (CPOL/CPHA) - must match device datasheet
 *          - High speed: Typical 1-20MHz depending on board and sensor capabilities
 *          - Multi-device: Multiple slaves share bus, selected via CS pins
 *          
 *          Typical driver workflow:
 *          1. hal.spi->get_device(name) to obtain device handle
 *          2. set_speed() to switch between low-speed (init) and high-speed (runtime)
 *          3. get_semaphore()->take() for exclusive bus access
 *          4. transfer() or transfer_fullduplex() for data exchange
 *          5. get_semaphore()->give() - CS automatically deasserted
 *          6. Optional: register_periodic_callback() for high-rate sensor sampling
 *          
 *          Thread safety:
 *          - All transfers must hold device semaphore to prevent CS conflicts
 *          - Semaphore is per-device, not per-bus (multiple devices can queue)
 *          - CS assertion/deassertion is automatic and atomic with transfers
 * 
 * @note CS is automatically asserted at transfer start, deasserted at completion
 * @note SPI speed affects only selected device - each device can have different speed
 * @warning Must hold bus semaphore during transfers to prevent CS conflicts between devices
 * @warning SPI mode (CPOL/CPHA) must exactly match device datasheet or data corrupts
 * @warning High-speed transfers (>10MHz) may be affected by PCB trace length and capacitance
 * 
 * @see Device.h for base class interface
 * @see SPIDeviceManager for device creation
 */
class SPIDevice : public Device {
public:
    /**
     * @brief Construct SPI device with bus type identifier
     * 
     * @note Constructor is protected by SPIDeviceManager - drivers obtain devices via get_device()
     */
    SPIDevice() : Device(BUS_TYPE_SPI) { }

    /**
     * @brief Virtual destructor for proper cleanup in derived classes
     */
    virtual ~SPIDevice() { }
    /* Device implementation */

    /**
     * @brief Change SPI clock speed for subsequent transfers
     * 
     * @details Allows switching between low-speed and high-speed operation for the same device.
     *          Common pattern: SPEED_LOW during sensor initialization/configuration (more reliable),
     *          SPEED_HIGH during runtime sampling (maximum throughput).
     *          
     *          Typical speeds:
     *          - SPEED_LOW: 1MHz (safe for configuration reads)
     *          - SPEED_HIGH: 10-20MHz (maximum sensor sampling rate)
     *          
     *          Actual speeds are platform and board-specific. Check platform HAL implementation
     *          for exact frequencies. Speed persists until next set_speed() call.
     * 
     * @param[in] speed Speed enum (Device::SPEED_LOW or Device::SPEED_HIGH)
     * 
     * @return true if speed change successful, false on error
     * 
     * @note Speed setting is persistent - affects all subsequent transfers until changed
     * @note Actual speeds are platform-specific and may differ from nominal values
     * @note Does not require holding semaphore (speed is per-device property)
     * 
     * @see Device::Speed for available speed options
     */
    virtual bool set_speed(Device::Speed speed) override = 0;

    /**
     * @brief Perform SPI half-duplex transaction (write-only, read-only, or write-then-read)
     * 
     * @details Executes SPI transfer with automatic chip select (CS) control. CS is asserted
     *          before transfer begins and deasserted when complete. Supports three transfer modes:
     *          
     *          1. Write-only: send != nullptr, recv == nullptr
     *             - Transmits send_len bytes (e.g., sensor configuration commands)
     *          
     *          2. Read-only: send == nullptr, recv != nullptr
     *             - Receives recv_len bytes (clock generated automatically)
     *          
     *          3. Write-then-read: both non-null
     *             - Transmits send_len bytes, then receives recv_len bytes
     *             - Clock idles between send and receive phases (not full-duplex)
     *             - Common for register read: write register address, read data
     *          
     *          CS timing:
     *          - CS asserted (low) at transfer start
     *          - Remains asserted for entire send and/or receive operation
     *          - Deasserted (high) when transfer completes or on error
     * 
     * @param[in] send Pointer to data to transmit (nullptr for receive-only)
     * @param[in] send_len Number of bytes to send (ignored if send == nullptr)
     * @param[out] recv Pointer to receive buffer (nullptr for transmit-only)
     * @param[in] recv_len Number of bytes to receive (ignored if recv == nullptr)
     * 
     * @return true if transfer successful, false on timeout/error
     * 
     * @note If send and recv both non-null, send completes before receive starts (half-duplex)
     * @note CS automatically managed - asserted before, deasserted after
     * @note For simultaneous transmit/receive, use transfer_fullduplex() instead
     * 
     * @warning Must hold device semaphore during call (use get_semaphore()->take())
     * @warning Buffers must remain valid for entire transfer duration (DMA may be used)
     * @warning Do not call from interrupt context
     * 
     * @see transfer_fullduplex() for simultaneous transmit and receive
     * @see get_semaphore() for bus locking
     */
    virtual bool transfer(const uint8_t *send, uint32_t send_len,
                          uint8_t *recv, uint32_t recv_len) override = 0;

    /**
     * @brief Perform simultaneous SPI transmit and receive (full-duplex)
     * 
     * @details Executes true full-duplex SPI transfer where data is transmitted and received
     *          simultaneously. Each clock cycle sends one bit from @send while receiving one
     *          bit into @recv. This is the native SPI operation mode.
     *          
     *          Typical usage pattern for sensor reads:
     *          - send[0]: Register address with read bit set (e.g., 0x80 | reg_addr)
     *          - send[1..n]: Don't-care bytes (often 0x00) to generate clock for receive
     *          - recv[0]: Dummy byte (sensor processing time)
     *          - recv[1..n]: Sensor data bytes
     *          
     *          Full-duplex means:
     *          - send[i] transmitted while recv[i] received in same clock cycle
     *          - Both buffers must be exactly len bytes
     *          - More efficient than transfer() for bidirectional operations
     *          - Typical for bulk sensor data reads (IMU FIFO, multi-register reads)
     * 
     * @param[in] send Pointer to transmit data buffer
     * @param[out] recv Pointer to receive data buffer
     * @param[in] len Number of bytes to exchange simultaneously
     * 
     * @return true if transfer successful, false on timeout/error
     * 
     * @note Transmit and receive occur simultaneously (true full-duplex)
     * @note CS automatically asserted before transfer, deasserted after
     * @note More efficient than transfer() for reading sensor data
     * 
     * @warning send and recv buffers must BOTH be exactly len bytes
     * @warning Buffers must remain valid for entire transfer (DMA may be used)
     * @warning Must hold device semaphore during call
     * @warning Do not call from interrupt context
     * 
     * @see transfer() for half-duplex write-then-read operations
     */
    virtual bool transfer_fullduplex(const uint8_t *send, uint8_t *recv,
                                     uint32_t len) = 0;

    /**
     * @brief Perform in-place full-duplex transfer using single buffer
     * 
     * @details Convenience overload for full-duplex transfer using a single buffer for both
     *          transmit and receive. Data is transmitted from buffer while simultaneously
     *          receiving into the same buffer, overwriting the transmit data.
     *          
     *          This function is optimized for DMA-enabled transfers without buffer copying.
     *          Platform implementations may use more efficient DMA scatter-gather operations.
     *          
     *          Usage pattern:
     *          - Initialize send_recv[0] with register address and read command
     *          - Initialize send_recv[1..n] with don't-care values (or leave uninitialized)
     *          - After call, send_recv contains received data (send_recv[0] is dummy byte)
     * 
     * @param[in,out] send_recv Buffer containing transmit data, overwritten with received data
     * @param[in] len Number of bytes to exchange
     * 
     * @return true if transfer successful, false on timeout/error
     * 
     * @note Buffer contents are overwritten with received data
     * @note Implemented via transfer() base class call (may be overridden for optimization)
     * @note CS automatically asserted before transfer, deasserted after
     * 
     * @warning Buffer must remain valid for entire transfer duration (DMA may be used)
     * @warning Must hold device semaphore during call
     * @warning Original transmit data is lost after call
     * 
     * @see transfer_fullduplex(const uint8_t*, uint8_t*, uint32_t) for separate buffers
     */
    bool transfer_fullduplex(uint8_t *send_recv, uint32_t len) override {
        return transfer(send_recv, len, send_recv, len);
    }

    /**
     * @brief Send clock pulses without asserting chip select
     * 
     * @details Generates SPI clock cycles with CS deasserted (high). Used exclusively for
     *          initializing SD card and microSD interfaces over SPI, which require 74+ clock
     *          pulses with CS high to enter SPI mode from power-on state.
     *          
     *          Normal SPI transfers always assert CS, but SD card initialization requires
     *          clocking with CS inactive to reset the card's command interface.
     *          
     *          Operation:
     *          - CS remains deasserted (high) for entire operation
     *          - MOSI driven to logic high or don't-care
     *          - MISO ignored (no data received)
     *          - Clock toggles for len byte-times (len * 8 clock cycles)
     * 
     * @param[in] len Number of bytes worth of clock pulses to generate (len * 8 clock edges)
     * 
     * @return true if successful, false if not supported or error
     * 
     * @note Default implementation returns false (not supported)
     * @note Only needed for SD card SPI initialization - not used for standard sensors
     * @note Most platform implementations override this for SD card support
     * @note Does not require holding semaphore (CS is not asserted)
     * 
     * @warning Specialized operation - only use for SD card initialization
     * 
     * @see http://elm-chan.org/docs/mmc/mmc_e.html for SD card SPI initialization sequence
     */
    virtual bool clock_pulse(uint32_t len) { return false; }
    
    /**
     * @brief Get semaphore for exclusive bus access
     * 
     * @details Returns the semaphore protecting this SPI device. Must be taken before any
     *          transfer operations to prevent conflicts with other threads accessing the
     *          same or different devices on the same bus.
     *          
     *          Locking requirements:
     *          - Required for: transfer(), transfer_fullduplex(), register reads/writes
     *          - Not required for: set_speed(), get_device_name() (device properties)
     *          - Take semaphore before transfers, give after completion
     *          - Nested takes from same thread may deadlock - avoid
     *          
     *          The semaphore is per-device (not per-bus), but platform implementations
     *          typically map all devices on same physical bus to the same semaphore for
     *          hardware arbitration.
     * 
     * @return Pointer to semaphore for this device (never nullptr)
     * 
     * @note Semaphore must be taken before any transfer operations
     * @note Use Semaphore::take() to acquire, Semaphore::give() to release
     * @note Blocking take may delay scheduler - use WITH_SEMAPHORE() macro for safety
     * 
     * @warning Failing to take semaphore before transfer causes race conditions
     * @warning Do not take semaphore in interrupt context (will block)
     * @warning Avoid holding semaphore for extended periods (blocks other drivers)
     * 
     * @see Device::get_semaphore() for base class documentation
     * @see WITH_SEMAPHORE() macro for exception-safe locking
     */
    virtual Semaphore *get_semaphore() override = 0;

    /**
     * @brief Register callback for periodic sensor sampling
     * 
     * @details Schedules a function to be called at regular intervals for high-rate sensor
     *          sampling. The scheduler automatically handles semaphore locking and timing.
     *          
     *          Typical usage for IMU sampling:
     *          - Register callback at sensor update rate (e.g., 1000Hz = 1000μs period)
     *          - Callback performs SPI read of sensor data
     *          - Semaphore is automatically held during callback execution
     *          - Callback runs in timer interrupt context (keep short and fast)
     *          
     *          Callback execution context:
     *          - May run in timer interrupt or high-priority thread (platform-specific)
     *          - Semaphore is pre-acquired before callback invocation
     *          - Should complete quickly (typically <100μs for sensor reads)
     *          - Must not block or sleep
     *          - Must not call functions that may block
     * 
     * @param[in] period_usec Period in microseconds between callback invocations
     * @param[in] cb Callback function to invoke (Device::PeriodicCb type)
     * 
     * @return Handle for callback (used with adjust_periodic_callback()), or nullptr on failure
     * 
     * @note Callback runs with semaphore automatically held
     * @note Multiple callbacks can be registered on same device
     * @note Callback execution is not guaranteed to be exactly periodic (scheduler dependent)
     * 
     * @warning Callback runs in interrupt/high-priority context - keep very short
     * @warning Do not block, sleep, or call blocking functions in callback
     * @warning Overrunning callback time budget affects overall system timing
     * @warning Must not call get_semaphore()->take() in callback (already held, will deadlock)
     * 
     * @see Device::register_periodic_callback() for base class documentation
     * @see adjust_periodic_callback() for changing callback rate
     */
    virtual Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, Device::PeriodicCb) override = 0;

    /**
     * @brief Adjust period of existing periodic callback
     * 
     * @details Changes the invocation rate of a previously registered periodic callback
     *          without needing to unregister and re-register. Useful for dynamic sample
     *          rate adjustment based on flight mode or performance requirements.
     *          
     *          Common use cases:
     *          - Reduce IMU sample rate when in low-power mode
     *          - Increase rate during aggressive flight maneuvers
     *          - Adapt rate based on CPU load
     * 
     * @param[in] h Handle returned from register_periodic_callback()
     * @param[in] period_usec New period in microseconds
     * 
     * @return true if adjustment successful, false if not supported or invalid handle
     * 
     * @note Default implementation returns false (not supported by all platforms)
     * @note Platform support varies - check return value
     * @note Period change takes effect on next callback invocation
     * 
     * @see register_periodic_callback() for initial callback registration
     */
    virtual bool adjust_periodic_callback(
        PeriodicHandle h, uint32_t period_usec) override { return false; }

    /**
     * @brief Configure bus clock slowdown factor for debugging
     * 
     * @details Reduces SPI clock speed by an integer divisor for debugging signal integrity
     *          issues or timing-sensitive sensors. Primarily used during hardware bringup
     *          or when diagnosing communication problems.
     *          
     *          Operation:
     *          - Divides configured SPI clock by slowdown factor
     *          - slowdown=1: No effect (normal speed)
     *          - slowdown=2: Half speed
     *          - slowdown=4: Quarter speed
     *          
     *          Use cases:
     *          - Debugging signal integrity issues (long PCB traces, noise)
     *          - Testing sensors at multiple speeds
     *          - Workaround for marginal hardware timing
     * 
     * @param[in] slowdown Integer divisor for clock speed (1=normal, 2=half, 4=quarter, etc.)
     * 
     * @note Default implementation does nothing (optional interface)
     * @note Not all platforms support this feature
     * @note Setting applies to all subsequent transfers until changed
     * @note For production code, use set_speed() instead
     * 
     * @warning Reduces sensor sampling rate - affects control loop timing
     * @warning Not intended for production use (debugging/bringup only)
     * 
     * @see set_speed() for normal speed control (SPEED_LOW vs SPEED_HIGH)
     */
    virtual void set_slowdown(uint8_t slowdown) {}
};

/**
 * @class SPIDeviceManager
 * @brief Factory for creating SPI device handles with platform-specific configuration
 * 
 * @details Singleton manager accessed via hal.spi that creates SPIDevice instances for
 *          sensor drivers. Abstracts platform-specific SPI bus configuration and device
 *          naming conventions.
 *          
 *          Device naming is platform-specific:
 *          - ChibiOS: Uses hwdef names from board definition files (e.g., "mpu6000", "baro")
 *          - Linux: Uses /dev/spidevX.Y device nodes (e.g., "spidev0.0")
 *          - SITL: Simulated device names matching real hardware
 *          
 *          SPI bus configuration (frequency, mode, CS pins) is typically defined in:
 *          - ChibiOS: hwdef.dat board definition files
 *          - Linux: Device tree or platform-specific configuration
 *          - Other platforms: Platform HAL implementation
 *          
 *          Typical usage in sensor drivers:
 *          1. AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev = hal.spi->get_device("mpu6000");
 *          2. if (!dev) { initialization failed }
 *          3. dev->set_speed(AP_HAL::Device::SPEED_LOW);
 *          4. Configure sensor via SPI transfers
 *          5. dev->set_speed(AP_HAL::Device::SPEED_HIGH);
 *          6. dev->register_periodic_callback() for sampling
 * 
 * @note Accessed via hal.spi singleton (defined in AP_HAL class)
 * @note Device names are platform and board-specific
 * @note get_device() returns nullptr if device name not found or unavailable
 * @note Drivers should gracefully handle nullptr return (device not present on this board)
 * 
 * @warning Device names must match exactly (case-sensitive)
 * @warning Multiple get_device() calls for same name may fail (single-owner model)
 * 
 * @see AP_HAL::get_HAL() for accessing singleton via hal.spi
 */
class SPIDeviceManager {
public:
    /**
     * @brief Get raw pointer to SPI device (internal use)
     * 
     * @details Internal factory method returning raw pointer. Drivers should use
     *          get_device() wrapper instead, which returns an OwnPtr for automatic
     *          resource management.
     *          
     *          Platform implementations create SPIDevice instances based on device name,
     *          configuring bus number, CS pin, SPI mode (CPOL/CPHA), and default speed
     *          from board definition or device tree.
     * 
     * @param[in] name Platform-specific device name (e.g., "mpu6000", "baro", "spidev0.0")
     * 
     * @return Raw pointer to SPIDevice, or nullptr if device not found or unavailable
     * 
     * @note Prefer get_device() wrapper for automatic memory management
     * @note Caller is responsible for deleting returned pointer (use OwnPtr instead)
     * 
     * @warning Returned pointer must be deleted by caller or wrapped in OwnPtr
     * @warning Multiple calls for same device may fail (hardware resource conflict)
     * 
     * @see get_device() for recommended wrapper returning OwnPtr
     */
    virtual SPIDevice *get_device_ptr(const char *name) = 0;
    
    /**
     * @brief Get smart pointer to SPI device (recommended interface)
     * 
     * @details Wrapper around get_device_ptr() returning an OwnPtr for automatic resource
     *          management. This is the recommended way to obtain SPI devices in driver code.
     *          
     *          OwnPtr provides:
     *          - Automatic deletion when going out of scope
     *          - Move semantics (no copying)
     *          - Null-safety (can check boolean conversion)
     *          
     *          Example usage:
     *          @code
     *          AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev = hal.spi->get_device("mpu6000");
     *          if (!dev) {
     *              // Device not available on this board
     *              return false;
     *          }
     *          dev->set_speed(AP_HAL::Device::SPEED_HIGH);
     *          // dev automatically deleted when going out of scope
     *          @endcode
     * 
     * @param[in] name Platform-specific device name (e.g., "mpu6000", "baro", "ms5611")
     * 
     * @return OwnPtr to SPIDevice (evaluates to false if device not found)
     * 
     * @note Device ownership transferred to caller - OwnPtr handles deletion
     * @note Check boolean conversion before use: if (!dev) { device not available }
     * @note Device name must match board configuration exactly
     * 
     * @warning Returns empty OwnPtr (nullptr) if device not found - always check before use
     * @warning Multiple get_device() calls for same device may fail (resource already allocated)
     * 
     * @see get_device_ptr() for raw pointer version (not recommended)
     * @see OwnPtr for smart pointer documentation
     */
    OwnPtr<SPIDevice> get_device(const char *name) {
        return AP_HAL::OwnPtr<AP_HAL::SPIDevice>(get_device_ptr(name));
    }

    /**
     * @brief Get count of SPI devices available on this platform
     * 
     * @details Returns the number of SPI devices registered in the platform configuration.
     *          Useful for device enumeration and diagnostics.
     *          
     *          Count includes:
     *          - All devices defined in board configuration (hwdef, device tree)
     *          - Both internal and external SPI devices
     *          - Devices whether currently in use or available
     * 
     * @return Number of SPI devices (0 if not supported or no devices)
     * 
     * @note Default implementation returns 0 (not all platforms support enumeration)
     * @note Count is static (doesn't change after initialization)
     * 
     * @see get_device_name() for retrieving device names by index
     */
    virtual uint8_t get_count() { return 0; }

    /**
     * @brief Get device name by index for enumeration
     * 
     * @details Returns the name of the SPI device at given index, allowing drivers to
     *          enumerate available devices. Typically used for diagnostics, testing,
     *          or dynamic device discovery.
     *          
     *          Example enumeration:
     *          @code
     *          for (uint8_t i = 0; i < hal.spi->get_count(); i++) {
     *              const char *name = hal.spi->get_device_name(i);
     *              if (name) {
     *                  printf("SPI device %u: %s\n", i, name);
     *              }
     *          }
     *          @endcode
     * 
     * @param[in] idx Zero-based device index (0 to get_count()-1)
     * 
     * @return Device name string, or nullptr if idx out of range or not supported
     * 
     * @note Default implementation returns nullptr (not all platforms support enumeration)
     * @note Returned string pointer is valid for lifetime of SPIDeviceManager
     * @note Index order is platform-specific and may change between boots
     * 
     * @warning Do not modify or free returned string (points to internal storage)
     * @warning Index may not correspond to physical bus numbering
     * 
     * @see get_count() for number of available devices
     */
    virtual const char *get_device_name(uint8_t idx) { return nullptr; }

    /**
     * @brief Set callback for register read/write debugging (optional)
     * 
     * @details Installs a callback function that is invoked on every register read or write
     *          operation for the specified device. Used primarily for low-level debugging,
     *          protocol analysis, or implementing software protocol analyzers.
     *          
     *          Callback receives:
     *          - Register address
     *          - Read/write direction
     *          - Data value
     *          - Timestamp
     *          
     *          Use cases:
     *          - Debugging sensor communication issues
     *          - Verifying register configuration sequences
     *          - Analyzing protocol timing
     *          - Creating software logic analyzers
     * 
     * @param[in] name Device name to attach callback to
     * @param[in] cb Callback function (Device::RegisterRWCb type)
     * 
     * @note Default implementation does nothing (optional interface)
     * @note Not all platforms support this feature
     * @note Callback may be invoked at high rate (affects performance)
     * @note Only one callback per device supported
     * 
     * @warning Callback runs in transfer context - must be extremely fast
     * @warning Do not perform blocking operations in callback
     * @warning Enabling affects performance (intended for debugging only)
     * 
     * @see Device::RegisterRWCb for callback signature
     */
    virtual void set_register_rw_callback(const char* name, AP_HAL::Device::RegisterRWCb cb) {}
};

}
