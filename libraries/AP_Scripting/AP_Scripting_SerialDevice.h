/**
 * @file AP_Scripting_SerialDevice.h
 * @brief Virtual serial device implementation for Lua scripting interface
 * 
 * @details This file implements virtual serial ports that allow Lua scripts to
 *          create protocol endpoints registered with AP_SerialManager. Scripts
 *          can instantiate virtual serial devices that appear as standard serial
 *          ports to other ArduPilot systems, enabling custom protocol implementations
 *          entirely in Lua without modifying C++ code.
 *          
 *          The virtual serial devices use buffered I/O with configurable RX/TX
 *          buffer sizes, and are protected by semaphores for thread-safe access
 *          between the scripting VM and AP_SerialManager consumers.
 *          
 *          Maximum number of virtual ports is defined by AP_SCRIPTING_SERIALDEVICE_NUM_PORTS
 *          (default 3), and individual ports can be enabled/disabled via parameters.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_Scripting_config.h"

#if AP_SCRIPTING_SERIALDEVICE_ENABLED

#include <AP_SerialManager/AP_SerialManager.h>

#ifndef AP_SCRIPTING_SERIALDEVICE_NUM_PORTS
#define AP_SCRIPTING_SERIALDEVICE_NUM_PORTS 3
#endif

class AP_Scripting;

/**
 * @class AP_Scripting_SerialDevice
 * @brief Container managing virtual serial ports for Lua scripting
 * 
 * @details This class provides a container that manages up to 
 *          AP_SCRIPTING_SERIALDEVICE_NUM_PORTS (default 3) virtual serial ports.
 *          Each port can be accessed by Lua scripts to implement custom serial
 *          protocols without requiring C++ code modifications.
 *          
 *          Virtual ports are registered with AP_SerialManager and appear as
 *          standard serial ports to other ArduPilot systems. This enables scripts
 *          to create protocol endpoints (e.g., custom telemetry formats, sensor
 *          interfaces, or ground station protocols) that integrate seamlessly
 *          with the existing serial port infrastructure.
 *          
 *          **Lifecycle:**
 *          - Constructor creates empty port array
 *          - init() initializes port infrastructure and parameter storage
 *          - clear() deallocates buffers and resets ports
 *          
 *          **Thread Safety:**
 *          Each port uses HAL_Semaphore for protecting buffer access between
 *          the scripting VM (which calls device_write/device_read) and
 *          AP_SerialManager consumers (which call _write/_read).
 *          
 *          **Parameter Control:**
 *          Ports are enabled/disabled via the enable parameter (AP_Int8),
 *          allowing runtime configuration without recompilation.
 * 
 * @note Maximum port count is compile-time constant AP_SCRIPTING_SERIALDEVICE_NUM_PORTS
 * @see AP_SerialManager for serial port registration and protocol routing
 */
class AP_Scripting_SerialDevice
{
public:
    /* Do not allow copies */
    CLASS_NO_COPY(AP_Scripting_SerialDevice);

    AP_Scripting_SerialDevice() {}

    /**
     * @brief Parameter controlling virtual serial device enable/disable state
     * 
     * @details This parameter allows runtime enable/disable of the virtual
     *          serial device system. When disabled, no virtual ports are
     *          registered with AP_SerialManager.
     *          
     *          Type: AP_Int8 (8-bit signed integer parameter)
     *          Storage: Persistent parameter stored in EEPROM/parameter file
     */
    AP_Int8 enable;

    /**
     * @brief Initialize virtual serial device infrastructure
     * 
     * @details Initializes the virtual serial device system by setting up
     *          parameter storage and preparing port management infrastructure.
     *          This method must be called during AP_Scripting initialization
     *          before any scripts attempt to use virtual serial devices.
     *          
     *          Initialization includes:
     *          - Parameter system registration for enable flag
     *          - Port array preparation
     *          - Integration with AP_SerialManager registration system
     *          
     *          After init() completes, individual ports can be configured
     *          and registered by Lua scripts via the scripting bindings.
     * 
     * @note Called once during scripting subsystem initialization
     * @see clear() for cleanup and deallocation
     */
    void init(void);

    /**
     * @brief Clear and deallocate all virtual serial ports
     * 
     * @details Clears all virtual serial ports by deallocating buffers,
     *          resetting state, and unregistering from AP_SerialManager.
     *          This method is called during scripting shutdown or when
     *          reconfiguring the virtual serial device system.
     *          
     *          Cleanup operations:
     *          - Deallocate RX/TX ByteBuffers for all ports
     *          - Reset buffer size tracking variables
     *          - Clear AP_SerialManager registrations
     *          - Reset port state to uninitialized
     *          
     *          After clear() completes, ports must be reinitialized via
     *          init() before use.
     * 
     * @note Safe to call multiple times; handles already-cleared state
     * @warning Any pending data in buffers will be lost
     * @see init() for reinitialization after clearing
     */
    void clear(void);

public:
    /**
     * @class Port
     * @brief Virtual serial port implementation for script-controlled serial devices
     * 
     * @details This nested class implements a single virtual serial port that appears
     *          as a standard serial port to AP_SerialManager consumers. It derives from
     *          AP_SerialManager::RegisteredPort to integrate with the serial port
     *          routing infrastructure.
     *          
     *          **Dual Interface Design:**
     *          The Port class provides two distinct interfaces:
     *          
     *          1. **Script-Side API (device_* methods):**
     *             - device_write(): Script writes data that consumers can read
     *             - device_read(): Script reads data written by consumers
     *             - device_available(): Check bytes available for script to read
     *             
     *          2. **AP_SerialManager Protocol (_* methods):**
     *             - _write(): Consumers write data that script can read
     *             - _read(): Consumers read data written by script
     *             - _available(): Check bytes available for consumer to read
     *             - _begin(): Initialize port with baud rate and buffer sizes
     *             - _flush(), _discard_input(): Buffer management
     *          
     *          **Data Flow:**
     *          - Consumer writes → _write() → readbuffer → device_read() → Script
     *          - Script writes → device_write() → writebuffer → _read() → Consumer
     *          
     *          **Buffer Management:**
     *          Uses ByteBuffer for both RX (readbuffer) and TX (writebuffer) with
     *          configurable sizes. Minimum buffer sizes are enforced in init_buffers().
     *          Buffer sizes can be changed dynamically; last_size_tx and last_size_rx
     *          track previous allocations to detect when reallocation is needed.
     *          
     *          **Thread Safety:**
     *          All buffer access is protected by HAL_Semaphore (sem) to ensure
     *          safe concurrent access from scripting VM thread and consumer threads.
     * 
     * @note Inherits from AP_SerialManager::RegisteredPort for protocol integration
     * @warning All buffer access must be protected with WITH_SEMAPHORE(sem)
     * @see AP_SerialManager::RegisteredPort for base protocol interface
     * @see ByteBuffer for buffer implementation details
     */
    class Port : public AP_SerialManager::RegisteredPort {
    public:
        friend class AP_Scripting_SerialDevice;

        /**
         * @brief Initialize virtual port infrastructure
         * 
         * @details Initializes this virtual port by preparing state and
         *          setting up for buffer allocation. Does not allocate buffers;
         *          buffer allocation occurs during _begin() call when consumer
         *          opens the port with specific buffer size requirements.
         * 
         * @note Called during AP_Scripting_SerialDevice::init()
         * @see _begin() for buffer allocation
         */
        void init(void);

        /**
         * @brief Clear and deallocate virtual port resources
         * 
         * @details Deallocates RX/TX ByteBuffers and resets port state to
         *          uninitialized. Safe to call on already-cleared ports.
         *          
         *          Cleanup operations:
         *          - Delete readbuffer (consumer → script data)
         *          - Delete writebuffer (script → consumer data)
         *          - Reset last_size_tx and last_size_rx to 0
         *          - Clear any pending state
         * 
         * @warning Any pending data in buffers will be lost
         * @note Protected by semaphore (sem) for thread safety
         */
        void clear(void);

        /**
         * @brief Write data from script to be read by serial port consumer
         * 
         * @details This is the script-side write API. Data written by the Lua script
         *          is placed into the writebuffer, where it can be read by the
         *          AP_SerialManager consumer via _read().
         *          
         *          Data flow: Script → device_write() → writebuffer → _read() → Consumer
         *          
         *          This method is called from Lua bindings when a script wants to
         *          send data to the serial port consumer (e.g., sending telemetry
         *          to a ground station or responding to a protocol request).
         * 
         * @param[in] buffer Pointer to data buffer containing bytes to write
         * @param[in] size   Number of bytes to write from buffer
         * 
         * @return Number of bytes actually written (may be less than size if buffer full)
         * 
         * @note Protected by semaphore for thread-safe buffer access
         * @warning Returns 0 if writebuffer is not allocated or full
         * @see device_read() for reading data from consumers
         * @see _read() for consumer-side read interface
         */
        size_t device_write(const uint8_t *buffer, size_t size);

        /**
         * @brief Read data written by serial port consumer for script processing
         * 
         * @details This is the script-side read API. Data written by the AP_SerialManager
         *          consumer via _write() is stored in the readbuffer, where it can be
         *          read by the Lua script via this method.
         *          
         *          Data flow: Consumer → _write() → readbuffer → device_read() → Script
         *          
         *          This method is called from Lua bindings when a script wants to
         *          receive data from the serial port consumer (e.g., receiving commands
         *          from a ground station or reading sensor data from a custom protocol).
         * 
         * @param[out] buffer Pointer to buffer where read bytes will be stored
         * @param[in]  count  Maximum number of bytes to read into buffer
         * 
         * @return Number of bytes actually read (may be less than count if fewer available),
         *         or -1 on error
         * 
         * @note Protected by semaphore for thread-safe buffer access
         * @note Returns 0 if readbuffer is empty or not allocated
         * @see device_available() to check available bytes before reading
         * @see device_write() for writing data to consumers
         * @see _write() for consumer-side write interface
         */
        ssize_t device_read(uint8_t *buffer, uint16_t count);

        /**
         * @brief Check number of bytes available for script to read
         * 
         * @details Returns the number of bytes currently in the readbuffer that
         *          were written by the AP_SerialManager consumer via _write() and
         *          are available for the script to read via device_read().
         *          
         *          This is the script-side availability check. Scripts typically
         *          call this method to determine how much data is waiting before
         *          calling device_read().
         * 
         * @return Number of bytes available in readbuffer for script to read,
         *         or 0 if readbuffer not allocated or empty
         * 
         * @note Protected by semaphore for thread-safe buffer access
         * @see device_read() for reading the available data
         * @see _available() for consumer-side availability check (writebuffer)
         */
        uint32_t device_available(void);

    private:
        /**
         * @brief Check if port is initialized (AP_SerialManager protocol implementation)
         * 
         * @details Override of AP_SerialManager::RegisteredPort::is_initialized().
         *          Virtual serial ports are always considered initialized once the
         *          Port object exists, as they don't depend on hardware initialization.
         * 
         * @return Always returns true for virtual serial ports
         * 
         * @note Part of AP_SerialManager protocol interface
         */
        bool is_initialized() override {
            return true;
        }

        /**
         * @brief Check if transmission is pending (AP_SerialManager protocol implementation)
         * 
         * @details Override of AP_SerialManager::RegisteredPort::tx_pending().
         *          Virtual serial ports use buffered I/O and don't have hardware
         *          transmission queues, so there's never "pending" transmission
         *          in the hardware sense. Data is immediately available in the
         *          buffer once written.
         * 
         * @return Always returns false for virtual serial ports
         * 
         * @note Part of AP_SerialManager protocol interface
         */
        bool tx_pending() override {
            return false;
        }

        /**
         * @brief Initialize or reallocate RX/TX buffers with specified sizes
         * 
         * @details Allocates ByteBuffer instances for readbuffer and writebuffer
         *          with the requested sizes. Enforces minimum buffer sizes to ensure
         *          usable capacity. If buffers already exist and sizes haven't changed,
         *          skips reallocation.
         *          
         *          Buffer size tracking (last_size_tx, last_size_rx) is used to
         *          detect when reallocation is needed, avoiding unnecessary
         *          deallocation/reallocation cycles.
         *          
         *          Minimum sizes are enforced to prevent unusably small buffers
         *          that could cause excessive overhead or protocol failures.
         * 
         * @param[in] size_rx Requested size in bytes for readbuffer (consumer→script)
         * @param[in] size_tx Requested size in bytes for writebuffer (script→consumer)
         * 
         * @return true if buffers successfully allocated, false on allocation failure
         * 
         * @note Protected by semaphore for thread-safe allocation
         * @warning Reallocation discards any pending data in existing buffers
         * @see _begin() which calls this method during port opening
         */
        bool init_buffers(const uint32_t size_rx, const uint32_t size_tx);

        /**
         * @brief Get available space in writebuffer (AP_SerialManager protocol)
         * 
         * @details Returns the number of bytes that can be written to writebuffer
         *          by the script via device_write(). This is called by AP_SerialManager
         *          consumers to implement flow control, though it's less commonly used
         *          than _available() for reading.
         * 
         * @return Number of bytes available in writebuffer, or 0 if not allocated
         * 
         * @note Protected by semaphore for thread-safe buffer access
         * @note Part of AP_SerialManager protocol interface
         */
        uint32_t txspace() override;

        /**
         * @brief Begin serial port operation with specified baud and buffer sizes
         * 
         * @details Override of AP_SerialManager::RegisteredPort::_begin(). Called when
         *          a consumer opens the virtual serial port. Initializes or reallocates
         *          buffers with the requested RX/TX sizes.
         *          
         *          For virtual serial ports, the baud rate parameter is ignored as
         *          there's no hardware UART. Buffer sizes are the critical configuration.
         * 
         * @param[in] b    Baud rate (ignored for virtual ports)
         * @param[in] rxS  RX buffer size in bytes (consumer→script data)
         * @param[in] txS  TX buffer size in bytes (script→consumer data)
         * 
         * @note Calls init_buffers() to allocate/reallocate ByteBuffers
         * @note Part of AP_SerialManager protocol interface
         * @see init_buffers() for buffer allocation details
         */
        void _begin(uint32_t b, uint16_t rxS, uint16_t txS) override;

        /**
         * @brief Write data from consumer to be read by script
         * 
         * @details Override of AP_SerialManager::RegisteredPort::_write(). This is
         *          the consumer-side write API. Data written by AP_SerialManager
         *          consumers is placed into the readbuffer, where it can be read
         *          by the Lua script via device_read().
         *          
         *          Data flow: Consumer → _write() → readbuffer → device_read() → Script
         *          
         *          This is called when other ArduPilot systems write to the virtual
         *          serial port (e.g., MAVLink router sending commands, or a sensor
         *          driver sending data).
         * 
         * @param[in] buffer Pointer to data buffer containing bytes to write
         * @param[in] size   Number of bytes to write from buffer
         * 
         * @return Number of bytes actually written (may be less than size if buffer full)
         * 
         * @note Protected by semaphore for thread-safe buffer access
         * @note Part of AP_SerialManager protocol interface
         * @see device_read() for script-side read interface
         */
        size_t _write(const uint8_t *buffer, size_t size) override;

        /**
         * @brief Read data written by script for consumer processing
         * 
         * @details Override of AP_SerialManager::RegisteredPort::_read(). This is
         *          the consumer-side read API. Data written by the Lua script via
         *          device_write() is stored in the writebuffer, where it can be
         *          read by AP_SerialManager consumers via this method.
         *          
         *          Data flow: Script → device_write() → writebuffer → _read() → Consumer
         *          
         *          This is called when other ArduPilot systems read from the virtual
         *          serial port (e.g., MAVLink parser reading telemetry, or protocol
         *          handler reading responses).
         * 
         * @param[out] buffer Pointer to buffer where read bytes will be stored
         * @param[in]  count  Maximum number of bytes to read into buffer
         * 
         * @return Number of bytes actually read (may be less than count if fewer available),
         *         or -1 on error
         * 
         * @note Protected by semaphore for thread-safe buffer access
         * @note Part of AP_SerialManager protocol interface
         * @see device_write() for script-side write interface
         */
        ssize_t _read(uint8_t *buffer, uint16_t count) override;

        /**
         * @brief Check number of bytes available for consumer to read
         * 
         * @details Override of AP_SerialManager::RegisteredPort::_available(). Returns
         *          the number of bytes currently in the writebuffer that were written
         *          by the Lua script via device_write() and are available for the
         *          consumer to read via _read().
         *          
         *          This is the consumer-side availability check. AP_SerialManager
         *          consumers call this to determine how much data is waiting before
         *          calling _read().
         * 
         * @return Number of bytes available in writebuffer for consumer to read,
         *         or 0 if writebuffer not allocated or empty
         * 
         * @note Protected by semaphore for thread-safe buffer access
         * @note Part of AP_SerialManager protocol interface
         * @see _read() for reading the available data
         */
        uint32_t _available() override;

        /**
         * @brief End serial port operation (no-op for virtual ports)
         * 
         * @details Override of AP_SerialManager::RegisteredPort::_end(). For virtual
         *          serial ports, there's no hardware to shut down, so this is a no-op.
         *          Buffer deallocation occurs via clear() method instead.
         * 
         * @note Part of AP_SerialManager protocol interface
         * @see clear() for actual cleanup and buffer deallocation
         */
        void _end() override {}

        /**
         * @brief Flush any pending writes (no-op for virtual ports)
         * 
         * @details Override of AP_SerialManager::RegisteredPort::_flush(). For virtual
         *          serial ports using buffered I/O, there's no hardware transmit queue
         *          to flush. Data written to buffers is immediately available for reading.
         * 
         * @note Part of AP_SerialManager protocol interface
         */
        void _flush() override {}

        /**
         * @brief Discard all data in readbuffer (consumer input discard)
         * 
         * @details Override of AP_SerialManager::RegisteredPort::_discard_input().
         *          Discards all pending data in the readbuffer (data written by
         *          consumers via _write() that hasn't been read by script yet).
         *          
         *          This is called when consumers want to clear pending input data,
         *          typically during protocol resets or error recovery.
         * 
         * @return true if input was successfully discarded, false if readbuffer not allocated
         * 
         * @note Protected by semaphore for thread-safe buffer access
         * @note Part of AP_SerialManager protocol interface
         * @warning Any data in readbuffer waiting for script will be lost
         */
        bool _discard_input() override;

        /**
         * @brief Buffer for consumer→script data (consumer writes, script reads)
         * 
         * @details ByteBuffer storing data written by AP_SerialManager consumers
         *          via _write() that will be read by Lua scripts via device_read().
         *          
         *          Data flow: Consumer → _write() → readbuffer → device_read() → Script
         *          
         *          Allocated dynamically in init_buffers() with size specified in
         *          _begin() call. Deallocated in clear().
         * 
         * @note Protected by sem semaphore for thread-safe access
         * @see writebuffer for the opposite direction (script→consumer)
         */
        ByteBuffer *readbuffer;

        /**
         * @brief Buffer for script→consumer data (script writes, consumer reads)
         * 
         * @details ByteBuffer storing data written by Lua scripts via device_write()
         *          that will be read by AP_SerialManager consumers via _read().
         *          
         *          Data flow: Script → device_write() → writebuffer → _read() → Consumer
         *          
         *          Allocated dynamically in init_buffers() with size specified in
         *          _begin() call. Deallocated in clear().
         * 
         * @note Protected by sem semaphore for thread-safe access
         * @see readbuffer for the opposite direction (consumer→script)
         */
        ByteBuffer *writebuffer;

        /**
         * @brief Last allocated TX buffer size for reallocation detection
         * 
         * @details Tracks the previously allocated writebuffer size in bytes.
         *          Used by init_buffers() to detect when buffer size changes
         *          require reallocation. If requested size matches last_size_tx,
         *          reallocation is skipped to avoid unnecessary overhead.
         *          
         *          Set to 0 when buffer is deallocated or not yet allocated.
         * 
         * @note Reduces reallocation overhead when _begin() called repeatedly
         *       with same buffer sizes
         */
        uint32_t last_size_tx;

        /**
         * @brief Last allocated RX buffer size for reallocation detection
         * 
         * @details Tracks the previously allocated readbuffer size in bytes.
         *          Used by init_buffers() to detect when buffer size changes
         *          require reallocation. If requested size matches last_size_rx,
         *          reallocation is skipped to avoid unnecessary overhead.
         *          
         *          Set to 0 when buffer is deallocated or not yet allocated.
         * 
         * @note Reduces reallocation overhead when _begin() called repeatedly
         *       with same buffer sizes
         */
        uint32_t last_size_rx;

        /**
         * @brief Semaphore protecting buffer access for thread safety
         * 
         * @details HAL_Semaphore used to protect concurrent access to readbuffer
         *          and writebuffer between the scripting VM thread (which calls
         *          device_write/device_read) and AP_SerialManager consumer threads
         *          (which call _write/_read).
         *          
         *          All buffer operations must be protected with WITH_SEMAPHORE(sem)
         *          to prevent race conditions and ensure data integrity.
         *          
         *          Critical sections protected:
         *          - ByteBuffer read/write operations
         *          - Buffer allocation/deallocation
         *          - Buffer availability checks
         *          - Input discard operations
         * 
         * @warning All buffer access MUST be protected by this semaphore
         * @note HAL_Semaphore is a mutex-style lock with WITH_SEMAPHORE macro support
         */
        HAL_Semaphore sem;
    };

    /**
     * @brief Array of virtual serial ports
     * 
     * @details Fixed-size array containing AP_SCRIPTING_SERIALDEVICE_NUM_PORTS
     *          (default 3) virtual Port instances. Each port can be independently
     *          configured and used by Lua scripts.
     *          
     *          Ports are indexed 0 to (AP_SCRIPTING_SERIALDEVICE_NUM_PORTS - 1).
     *          Scripts access ports via array index through Lua bindings.
     * 
     * @note Array size is compile-time constant AP_SCRIPTING_SERIALDEVICE_NUM_PORTS
     * @see AP_SCRIPTING_SERIALDEVICE_NUM_PORTS for maximum port count configuration
     */
    Port ports[AP_SCRIPTING_SERIALDEVICE_NUM_PORTS];
};

#endif  // AP_SCRIPTING_SERIALDEVICE_ENABLED
