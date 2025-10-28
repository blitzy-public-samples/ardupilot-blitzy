/*
 * Copyright (C) 2015  Intel Corporation. All rights reserved.
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
 * @brief Linux spidev interface implementation for SPI device access
 * 
 * Provides SPI bus access through Linux kernel spidev interface (/dev/spidevX.Y).
 * Supports both kernel-managed chip select and userspace GPIO chip select with
 * configurable speed, mode, and bits per word.
 */

#pragma once

#include <inttypes.h>
#include <vector>

#include <AP_HAL/HAL.h>
#include <AP_HAL/SPIDevice.h>

namespace Linux {

/**
 * Forward declaration of SPIBus - manages a single SPI bus with poller thread
 */
class SPIBus;

/**
 * Forward declaration of SPIDesc - SPI device descriptor structure
 */
class SPIDesc;

/**
 * @struct Linux::SPIDesc
 * @brief SPI device descriptor for board-specific SPI device configuration
 * 
 * @details Defines static configuration for one SPI device. Typically defined
 *          in board-specific tables (e.g., GPIO_Navio2.cpp) and selected via
 *          device name lookup.
 *          
 *          Board-specific SPI device table:
 *          - Array of SPIDesc entries defined per board
 *          - Indexed by device name (e.g., "bmi160", "mpu9250")
 *          - Maps logical device to physical SPI bus and CS pin
 *          
 *          SPI mode bits:
 *          - Mode 0 (0x00): CPOL=0, CPHA=0 (clock idles low, sample on rising edge)
 *          - Mode 1 (0x01): CPOL=0, CPHA=1 (clock idles low, sample on falling edge)
 *          - Mode 2 (0x02): CPOL=1, CPHA=0 (clock idles high, sample on falling edge)
 *          - Mode 3 (0x03): CPOL=1, CPHA=1 (clock idles high, sample on rising edge)
 *          
 *          Chip select handling:
 *          - cs_pin >= 0: Userspace GPIO CS (software controlled)
 *          - cs_pin < 0: Kernel CS (hardware controlled by spidev)
 *          
 *          Speed settings:
 *          - lowspeed: Used during device initialization/probe
 *          - highspeed: Used during normal operation
 *          - Driver switches via set_speed() calls
 * 
 * @note Chip select is active-low by convention
 * @note bits_per_word typically 8 (byte transfers)
 * 
 * @see SPIDevice for usage
 */
struct SPIDesc {
    /**
     * @brief Constructor initializes SPI device descriptor
     * 
     * @param[in] name_ Device name for lookup (e.g., "bmi160", "mpu9250")
     * @param[in] bus_ SPI bus number (maps to /dev/spidevX.Y, X=bus)
     * @param[in] subdev_ SPI subdevice number (maps to /dev/spidevX.Y, Y=subdev)
     * @param[in] mode_ SPI mode 0-3 (CPOL/CPHA combination)
     * @param[in] bits_per_word_ Transfer word size in bits (typically 8)
     * @param[in] cs_pin_ GPIO pin for chip select (-1 for kernel CS)
     * @param[in] lowspeed_ Low speed in Hz for initialization
     * @param[in] highspeed_ High speed in Hz for normal operation
     */
    SPIDesc(const char *name_, uint16_t bus_, uint16_t subdev_, uint8_t mode_,
            uint8_t bits_per_word_, int16_t cs_pin_, uint32_t lowspeed_,
            uint32_t highspeed_)
        : name(name_), bus(bus_), subdev(subdev_), mode(mode_)
        , bits_per_word(bits_per_word_), cs_pin(cs_pin_), lowspeed(lowspeed_)
        , highspeed(highspeed_)
    {
    }

    /**
     * Device name string (e.g., "bmi160", "mpu9250")
     */
    const char *name;
    
    /**
     * SPI bus number: /dev/spidevX.Y where X=bus
     */
    uint16_t bus;
    
    /**
     * SPI subdevice/CS number: /dev/spidevX.Y where Y=subdev
     * Only used if cs_pin < 0 (kernel chip select)
     */
    uint16_t subdev;
    
    /**
     * SPI mode (0-3): Clock polarity and phase configuration
     */
    uint8_t mode;
    
    /**
     * Transfer word size in bits (typically 8 for byte transfers)
     */
    uint8_t bits_per_word;
    
    /**
     * GPIO pin number for chip select, or -1 for kernel CS
     */
    int16_t cs_pin;
    
    /**
     * Low speed in Hz (used during device probe/initialization)
     */
    uint32_t lowspeed;
    
    /**
     * High speed in Hz (used during normal operation)
     */
    uint32_t highspeed;
};

/**
 * @class Linux::SPIDevice
 * @brief Linux kernel spidev implementation of SPI device interface
 * 
 * @details Represents one SPI device (slave) on a Linux SPI bus. Uses Linux
 *          spidev kernel interface accessed via /dev/spidevX.Y device files.
 *          
 *          Linux spidev interface:
 *          - Open /dev/spidevX.Y where X=bus, Y=chip_select
 *          - Configure mode, speed, bits with ioctl() (SPI_IOC_WR_MODE, etc.)
 *          - Perform transfers with SPI_IOC_MESSAGE ioctl (struct spi_ioc_transfer)
 *          - Kernel driver handles timing, clock generation, bus arbitration
 *          
 *          Chip select modes:
 *          - Kernel CS: cs_pin < 0, kernel controls /dev/spidevX.Y CS line
 *          - Userspace CS: cs_pin >= 0, software GPIO toggle for CS
 *          - Userspace CS advantages: More CS lines than hardware supports
 *          - Userspace CS disadvantages: Slower, timing-sensitive
 *          
 *          Transfer types:
 *          - Half-duplex: Send OR receive (transfer with send or recv nullptr)
 *          - Full-duplex: Simultaneous send and receive (transfer_fullduplex)
 *          - Most sensors use half-duplex (write register, then read data)
 *          
 *          Speed management:
 *          - set_speed(LOW): Use lowspeed from SPIDesc (probe/init)
 *          - set_speed(HIGH): Use highspeed from SPIDesc (normal operation)
 *          - Typical lowspeed: 500kHz-2MHz (conservative for detection)
 *          - Typical highspeed: 10-20MHz (maximum sensor supports)
 *          
 *          Periodic callbacks:
 *          - register_periodic_callback() for continuous sensor sampling
 *          - Callbacks execute in SPIBus poller thread
 *          - Each device can have independent callback period
 *          - Typical sensor polling: 100Hz-8kHz depending on sensor
 *          
 *          Thread safety:
 *          - get_semaphore() returns per-bus semaphore
 *          - Multiple devices on same bus share semaphore
 *          - Use WITH_SEMAPHORE(dev->get_semaphore()) in driver code
 *          
 *          Typical usage pattern:
 *          ```cpp
 *          // Get device handle by name
 *          auto dev = hal.spi->get_device("mpu9250");
 *          
 *          // Start with low speed for probing
 *          dev->set_speed(AP_HAL::Device::SPEED_LOW);
 *          uint8_t whoami;
 *          dev->read_registers(REG_WHO_AM_I, &whoami, 1);
 *          
 *          // Switch to high speed
 *          dev->set_speed(AP_HAL::Device::SPEED_HIGH);
 *          
 *          // Register 1kHz callback
 *          dev->register_periodic_callback(1000, // 1ms = 1kHz
 *              FUNCTOR_BIND_MEMBER(&IMU::_timer, void));
 *          ```
 * 
 * @note Requires spidev kernel module (CONFIG_SPI_SPIDEV=y)
 * @note Maximum speed limited by spidev, sensor, and board routing
 * @warning Userspace CS has timing uncertainty (kernel scheduler jitter)
 * @warning Some sensors require specific CS setup/hold times
 * 
 * @see SPIDesc for device configuration
 * @see SPIDeviceManager for device creation
 */
class SPIDevice : public AP_HAL::SPIDevice {
public:
    /**
     * @brief Constructor creates SPI device handle
     * 
     * @param[in] bus Reference to SPIBus managing this device's bus
     * @param[in] device_desc Reference to SPIDesc with device configuration
     * 
     * @note Opens spidev file descriptor
     * @note Initializes CS GPIO if userspace CS specified
     * @note Starts with low speed by default
     */
    SPIDevice(SPIBus &bus, SPIDesc &device_desc);

    /**
     * @brief Destructor closes device and releases CS GPIO
     * 
     * @note Automatically deasserts CS if userspace CS
     * @note Closes spidev file descriptor
     * @note Unregisters periodic callbacks
     */
    virtual ~SPIDevice();

    /* AP_HAL::SPIDevice implementation */

    /**
     * @brief Set SPI bus speed
     * 
     * @param[in] speed Speed setting (SPEED_LOW or SPEED_HIGH)
     * @return bool True if speed set successfully
     * 
     * @details Speed mapping:
     *          - SPEED_LOW: Use SPIDesc::lowspeed (probe/init)
     *          - SPEED_HIGH: Use SPIDesc::highspeed (normal operation)
     *          
     *          When to use low speed:
     *          - Device detection (WHO_AM_I register reads)
     *          - Initial configuration writes
     *          - First communication after power-on
     *          
     *          When to use high speed:
     *          - Continuous sensor data reads
     *          - High-rate gyro/accel sampling
     *          - After successful initialization
     * 
     * @note Speed change takes effect on next transfer
     * @note Actual speed may be lower (limited by SPI controller dividers)
     */
    bool set_speed(AP_HAL::Device::Speed speed) override;

    /**
     * @brief Perform SPI half-duplex transfer (send then receive)
     * 
     * @param[in] send Pointer to data to write (or nullptr for read-only)
     * @param[in] send_len Number of bytes to write
     * @param[out] recv Pointer to buffer for received data (or nullptr for write-only)
     * @param[in] recv_len Number of bytes to read
     * @return bool True if transfer successful
     * 
     * @details Transfer sequence:
     *          1. Assert CS (if userspace CS)
     *          2. Write send_len bytes from send buffer
     *          3. Read recv_len bytes into recv buffer
     *          4. Deassert CS (if userspace CS)
     *          
     *          Transfer modes:
     *          - Write-only: send != nullptr, recv == nullptr
     *          - Read-only: send == nullptr, recv != nullptr
     *          - Write-then-read: both != nullptr (sequential, not simultaneous)
     *          
     *          Typical sensor register read:
     *          ```cpp
     *          uint8_t reg = 0x3B;  // Register address
     *          uint8_t data[6];      // Read buffer
     *          dev->transfer(&reg, 1, data, 6); // Write reg, read 6 bytes
     *          ```
     * 
     * @note Uses SPI_IOC_MESSAGE ioctl with spi_ioc_transfer array
     * @note CS automatically handled (asserted during transfer)
     * @warning send and recv must not overlap
     */
    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override;

    /**
     * @brief Perform SPI full-duplex transfer (simultaneous send and receive)
     * 
     * @param[in] send Pointer to data to transmit
     * @param[out] recv Pointer to buffer for received data
     * @param[in] len Number of bytes to transfer (same for send and receive)
     * @return bool True if transfer successful
     * 
     * @details Full-duplex means:
     *          - Transmit and receive happen simultaneously
     *          - Every transmitted byte receives a response byte
     *          - Total transfer time = len bytes at configured speed
     *          
     *          Used for:
     *          - Devices with simultaneous bidirectional protocols
     *          - Some types of memory devices
     *          - Custom protocols requiring full-duplex
     * 
     * @note Most sensors use half-duplex (transfer), not full-duplex
     * @warning send and recv must not overlap (use in-place variant)
     */
    bool transfer_fullduplex(const uint8_t *send, uint8_t *recv,
                             uint32_t len) override;

    /**
     * @brief Perform SPI full-duplex transfer in-place
     * 
     * @param[in,out] send_recv Buffer for both transmit and receive data
     * @param[in] len Number of bytes to transfer
     * @return bool True if transfer successful
     * 
     * @details In-place full-duplex:
     *          - Transmit bytes from buffer
     *          - Received bytes overwrite same buffer
     *          - Saves memory for temporary buffers
     * 
     * @note Original transmit data lost (overwritten by received data)
     */
    bool transfer_fullduplex(uint8_t *send_recv, uint32_t len) override;

    /**
     * @brief Get semaphore protecting this device's SPI bus
     * 
     * @return AP_HAL::Semaphore* Pointer to bus semaphore (shared by all devices on bus)
     * 
     * @note Use WITH_SEMAPHORE(dev->get_semaphore()) in driver code
     * @note Semaphore is per-bus, not per-device
     * @note Prevents concurrent access to same SPI bus from multiple threads
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
     *          - Executes in SPIBus poller thread (not main thread)
     *          - Period accuracy: ±10-100μs depending on kernel scheduler
     *          - Multiple callbacks supported per device with independent periods
     *          - Automatically acquires bus semaphore before callback
     *          
     *          Typical IMU polling:
     *          ```cpp
     *          // Poll gyro at 1kHz (1000μs period)
     *          _periodic_handle = _dev->register_periodic_callback(1000,
     *              FUNCTOR_BIND_MEMBER(&Gyro::_timer, void));
     *          ```
     * 
     * @note Callback must complete in less than period to avoid overruns
     * @warning Callback runs in SPI bus thread context, not main thread
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

protected:
    /**
     * Reference to SPIBus managing this device's physical bus
     */
    SPIBus &_bus;
    
    /**
     * Reference to SPIDesc device descriptor with configuration
     */
    SPIDesc &_desc;
    
    /**
     * GPIO control for userspace chip select (nullptr if kernel CS)
     */
    AP_HAL::DigitalSource *_cs;
    
    /**
     * Current SPI clock speed in Hz
     */
    uint32_t _speed;

    /**
     * @brief Assert chip select (active low) if userspace CS
     * 
     * @note Only acts if SPIDesc::cs_pin >= 0 (userspace CS)
     * @note Writes GPIO LOW to select device
     * @note Called automatically by transfer methods
     */
    void _cs_assert();

    /**
     * @brief Deassert chip select (inactive high) if userspace CS
     * 
     * @note Only acts if SPIDesc::cs_pin >= 0 (userspace CS)
     * @note Writes GPIO HIGH to deselect device
     * @note Called automatically by transfer methods
     */
    void _cs_release();
};

/**
 * @class Linux::SPIDeviceManager
 * @brief Factory for creating and managing Linux SPI devices
 * 
 * @details Manages SPI buses and creates device handles based on name lookup.
 *          Each SPIBus has a dedicated poller thread for periodic callbacks.
 *          
 *          Device lookup process:
 *          1. Search static SPIDesc table for matching device name
 *          2. Extract bus number from SPIDesc
 *          3. Create or get existing SPIBus for that bus number
 *          4. Create SPIDevice linked to bus and descriptor
 *          
 *          Device naming convention:
 *          - Board-specific device names (e.g., "mpu9250", "bmi160")
 *          - Defined in board GPIO files (GPIO_Navio2.cpp, etc.)
 *          - Same device may have different names on different boards
 *          - Allows board-independent driver code
 *          
 *          Bus management:
 *          - Buses created on-demand when first device requested
 *          - Each bus has poller thread for periodic callbacks
 *          - Buses persist for lifetime of manager
 * 
 * @note Thread-safe device creation
 * @see SPIDevice for device operations
 * @see SPIDesc for device configuration
 */
class SPIDeviceManager : public AP_HAL::SPIDeviceManager {
public:
    friend class SPIDevice;

    /**
     * @brief Downcast helper from generic SPIDeviceManager pointer
     * 
     * @param[in] spi_mgr Pointer to AP_HAL::SPIDeviceManager base class
     * @return SPIDeviceManager* Downcasted pointer to Linux implementation
     */
    static SPIDeviceManager *from(AP_HAL::SPIDeviceManager *spi_mgr)
    {
        return static_cast<SPIDeviceManager*>(spi_mgr);
    }

    /**
     * @brief Constructor initializes SPI device manager
     * 
     * @note Reserves space for 3 buses (typical board configuration)
     * @note Does not enumerate buses yet - deferred to first get_device() call
     */
    SPIDeviceManager()
    {
        /* Reserve space up-front for 3 buses */
        _buses.reserve(3);
    }

    /* AP_HAL::SPIDeviceManager implementation */
    
    /**
     * @brief Create or get SPI device handle by name
     * 
     * @param[in] name Device name (e.g., "mpu9250", "bmi160")
     * @return AP_HAL::SPIDevice* Pointer to device handle, or nullptr if not found
     * 
     * @details Device lookup:
     *          - Searches static _device[] array for matching name
     *          - Returns nullptr if name not found in table
     *          - Creates SPIBus if not already created
     *          - Creates SPIDevice linked to bus and descriptor
     *          
     *          Example usage:
     *          ```cpp
     *          auto imu_dev = hal.spi->get_device("mpu9250");
     *          if (!imu_dev) {
     *              // Device not available on this board
     *              return false;
     *          }
     *          ```
     * 
     * @note Device name is board-specific (check board GPIO file)
     * @note Returns nullptr if device not defined in SPIDesc table
     * @note Caller owns returned pointer (auto-deleted via shared_ptr internally)
     */
    AP_HAL::SPIDevice *get_device_ptr(const char *name) override;

    /**
     * @brief Stop all SPI polling threads and finalize
     * 
     * @note Called during HAL shutdown
     * @note Blocks until all poller threads complete
     * @note Existing device handles remain valid but callbacks stop
     */
    void teardown();

    /**
     * @brief Get number of available SPI devices
     * 
     * @return uint8_t Count of devices in SPIDesc table
     * 
     * @note Count is board-specific (number of entries in _device[])
     */
    uint8_t get_count() override;

    /**
     * @brief Get device name by index
     * 
     * @param[in] idx Index into device table (0 to get_count()-1)
     * @return const char* Device name string, or nullptr if index invalid
     * 
     * @note Used for enumerating available SPI devices
     * @note Index order is board-specific
     */
    const char *get_device_name(uint8_t idx) override;

protected:
    /**
     * @brief Unregister and cleanup SPIBus (internal use)
     * 
     * @param[in] b Reference to SPIBus to unregister
     * 
     * @note Called when last device on bus is destroyed
     */
    void _unregister(SPIBus &b);
    
    /**
     * @brief Create SPIDevice instance (internal factory method)
     * 
     * @param[in] b Reference to SPIBus for device
     * @param[in] device_desc Reference to SPIDesc with configuration
     * @return AP_HAL::SPIDevice* Pointer to new device
     */
    AP_HAL::SPIDevice *_create_device(SPIBus &b, SPIDesc &device_desc) const;

    /**
     * Vector of SPIBus instances (one per discovered bus)
     * Buses created on-demand and persist for manager lifetime
     */
    std::vector<SPIBus*> _buses;

    /**
     * Number of device descriptors in _device[] array
     * Board-specific constant
     */
    static const uint8_t _n_device_desc;
    
    /**
     * Static array of SPI device descriptors
     * Defined per-board in GPIO_*.cpp files
     */
    static SPIDesc _device[];
};

}
