#pragma once

/**
 * @file UARTDriver.h
 * @brief Linux serial port UART driver implementation
 * 
 * Provides UART/serial communication using Linux device files (/dev/ttyS*, /dev/ttyUSB*)
 * and network sockets (TCP/UDP). Implements buffered I/O with automatic read/write
 * in scheduler UART thread for low-latency communication.
 */

#include <AP_HAL/utility/OwnPtr.h>
#include <AP_HAL/utility/RingBuffer.h>

#include "AP_HAL_Linux.h"
#include "SerialDevice.h"
#include "Semaphores.h"

namespace Linux {

/**
 * @class Linux::UARTDriver
 * @brief Linux serial port driver with buffered I/O and network socket support
 * 
 * @details Implements serial communication for telemetry, GPS, and peripherals on Linux.
 *          Supports physical serial ports, USB serial adapters, and network transports.
 *          
 *          Supported device types:
 *          - Physical serial: /dev/ttyS0, /dev/ttyS1, etc.
 *          - USB serial adapters: /dev/ttyUSB0, /dev/ttyACM0, etc.
 *          - TCP server: tcp:port (listens for connection)
 *          - TCP client: tcp:host:port (connects to remote)
 *          - UDP: udp:host:port or udp:host:port:bcast (with broadcast)
 *          
 *          Buffering architecture:
 *          - Ring buffers for RX and TX data (in-task, not kernel buffers)
 *          - UART thread periodically calls _timer_tick() to service buffers
 *          - _timer_tick() reads from device to RX buffer, writes from TX buffer to device
 *          - Reduces syscall overhead (batched reads/writes vs byte-at-a-time)
 *          
 *          Flow control:
 *          - Hardware flow control (RTS/CTS) via SerialDevice
 *          - Software flow control (XON/XOFF) via SerialDevice
 *          - Configurable per-port for different peripherals
 *          
 *          Parity configuration:
 *          - None, Even, Odd parity settings
 *          - Applied during device initialization
 *          - Useful for specific peripheral protocols (e.g., SBUS)
 *          
 *          Timestamp estimation:
 *          - receive_time_constraint_us() estimates packet arrival time
 *          - Accounts for baud rate and buffering delays
 *          - Used for GPS time synchronization
 *          - Conservative estimate (packet arrived no later than this time)
 *          
 *          MAVLink packetization:
 *          - Optional packetization on message boundaries
 *          - Reduces fragmentation of MAVLink packets
 *          - Improves network transport efficiency
 *          
 *          Console mode:
 *          - First UART can be console (stdout/stderr redirect)
 *          - Used for debug output during early boot
 *          - Disabled after normal UART init
 *          
 *          Typical usage:
 *          ```cpp
 *          // Configure telemetry UART
 *          hal.uartA->begin(57600); // 57.6kbaud
 *          hal.uartA->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
 *          
 *          // Write MAVLink message
 *          hal.uartA->write(msg_buf, msg_len);
 *          
 *          // Read GPS data
 *          int16_t nbytes = hal.gps->read(buf, sizeof(buf));
 *          ```
 * 
 * @note TX/RX buffers allocated dynamically during begin()
 * @note Network sockets useful for SITL and network-based telemetry
 * @warning Console mode conflicts with normal UART usage
 * @warning Flow control must match peripheral expectations
 * 
 * @see SerialDevice for underlying device abstraction
 * @see UARTDevice for physical serial port implementation
 * @see TCPServerDevice, UDPDevice for network implementations
 */
class UARTDriver : public AP_HAL::UARTDriver {
public:
    /**
     * @brief Constructor initializes UART driver
     * 
     * @param[in] default_console True if this UART is system console
     * 
     * @note Console UART used for early debug output before normal init
     * @note Most UARTs created with default_console=false
     */
    UARTDriver(bool default_console);

    /**
     * @brief Downcast helper from generic UARTDriver pointer
     * 
     * @param[in] uart Pointer to AP_HAL::UARTDriver base class
     * @return UARTDriver* Downcasted pointer to Linux implementation
     */
    static UARTDriver *from(AP_HAL::UARTDriver *uart) {
        return static_cast<UARTDriver*>(uart);
    }

    /**
     * @brief Check if UART has been initialized
     * 
     * @return bool True if begin() called and device opened successfully
     * 
     * @note Returns false if device open failed or not yet initialized
     */
    bool is_initialized() override;
    
    /**
     * @brief Check if transmit data is pending in buffer
     * 
     * @return bool True if TX buffer contains unsent data
     * 
     * @note Useful for waiting until transmission complete before shutdown
     */
    bool tx_pending() override;

    /**
     * @brief Get available space in transmit buffer
     * 
     * @return uint32_t Number of bytes free in TX buffer
     * 
     * @note Returns 0 if buffer full (write would block)
     * @note Check before large writes to avoid blocking
     */
    uint32_t txspace() override;

    /**
     * @brief Set device path for UART
     * 
     * @param[in] path Device path string (e.g., "/dev/ttyS1", "tcp:14550", "udp:192.168.1.100:14550")
     * 
     * @details Path formats:
     *          - Physical serial: /dev/ttyS0, /dev/ttyUSB0, etc.
     *          - TCP server: tcp:PORT (listen on PORT)
     *          - TCP client: tcp:HOST:PORT (connect to HOST:PORT)
     *          - UDP: udp:HOST:PORT (send to HOST:PORT)
     *          - UDP broadcast: udp:HOST:PORT:bcast
     * 
     * @note Must be called before begin()
     * @note Path parsed by _parseDevicePath() to create appropriate SerialDevice
     */
    void set_device_path(const char *path);

    /**
     * @brief Write pending bytes from TX buffer to device
     * 
     * @return bool True if write successful or no data pending
     * 
     * @note Called from _timer_tick() in UART thread
     * @note Writes as much data as device can accept without blocking
     * @note Automatic retry on EAGAIN (non-blocking I/O)
     */
    bool _write_pending_bytes(void);
    
    /**
     * @brief Timer tick callback for UART servicing
     * 
     * @details Called periodically by UART thread to:
     *          1. Read available data from device into RX buffer
     *          2. Write pending data from TX buffer to device
     *          3. Update receive timestamp for time constraint estimates
     *          
     *          This batches I/O to reduce syscall overhead compared to
     *          byte-at-a-time reads/writes from application code.
     * 
     * @note Executes in UART thread context, not main thread
     * @note Frequency: 100-1000Hz depending on scheduler configuration
     */
    virtual void _timer_tick(void) override;

    /**
     * @brief Get current flow control setting
     * 
     * @return enum flow_control Flow control mode (DISABLE, ENABLE, AUTO)
     * 
     * @note Delegates to underlying SerialDevice
     */
    virtual enum flow_control get_flow_control(void) override
    {
        return _device->get_flow_control();
    }

    /**
     * @brief Configure parity bit setting
     * 
     * @param[in] v Parity setting (0=none, 1=odd, 2=even)
     * 
     * @note Applied during device initialization
     * @note Some protocols (e.g., SBUS) require even parity
     * @note Must be set before begin() to take effect
     */
    void configure_parity(uint8_t v) override;

    /**
     * @brief Set flow control mode
     * 
     * @param[in] flow_control_setting Flow control mode to enable
     * 
     * @details Flow control modes:
     *          - FLOW_CONTROL_DISABLE: No flow control
     *          - FLOW_CONTROL_ENABLE: Hardware RTS/CTS flow control
     *          - FLOW_CONTROL_AUTO: Automatic flow control detection
     * 
     * @note Hardware flow control requires RTS/CTS wiring
     * @note Software XON/XOFF not commonly used on ArduPilot
     */
    virtual void set_flow_control(enum flow_control flow_control_setting) override
   {
       _device->set_flow_control(flow_control_setting);
   }

    /**
     * @brief Estimate receive time for packet based on baud rate
     * 
     * @param[in] nbytes Packet size in bytes
     * @return uint64_t Estimated time in microseconds when packet started arriving
     * 
     * @details Time estimate calculation:
     *          - Accounts for baud rate (transmission time per byte)
     *          - Accounts for buffering delays (system and driver buffers)
     *          - Returns conservative estimate (packet arrived no later than this)
     *          - Used for GPS timestamp synchronization
     *          
     *          Example: 100 bytes at 115200 baud:
     *          - Transmission time = 100 * 10 bits / 115200 baud ≈ 8.7ms
     *          - Return current_time - 8.7ms
     * 
     * @note Returns 0 if baud rate unknown (e.g., USB, network sockets)
     * @note Useful for correlating received data with system time
     * @warning This should be treated as a time constraint, not an exact time
     */
    uint64_t receive_time_constraint_us(uint16_t nbytes) override;

    /**
     * @brief Get effective bandwidth in bytes per second
     * 
     * @return uint32_t Bandwidth in bytes/sec (baud_rate / 10)
     * 
     * @note Assumes 10 bits per byte (8 data + 1 start + 1 stop)
     * @note Used for buffer sizing and flow estimation
     */
    uint32_t bw_in_bytes_per_second() const override;

    /**
     * @brief Get configured baud rate
     * 
     * @return uint32_t Baud rate in bits per second
     * 
     * @note Returns 0 if not initialized or baud rate not applicable (network)
     */
    virtual uint32_t get_baud_rate() const override { return _baudrate; }

private:
    /** Underlying serial device (physical port, TCP, or UDP) */
    AP_HAL::OwnPtr<SerialDevice> _device;
    
    /** True if this UART is system console (for early debug output) */
    bool _console;
    
    /** Flag indicating _timer_tick() execution in progress (prevents reentrancy) */
    volatile bool _in_timer;
    
    /** Base port number for network sockets (TCP/UDP) */
    uint16_t _base_port;
    
    /** Configured baud rate in bits per second */
    uint32_t _baudrate;
    
    /** IP address string for network sockets (TCP/UDP) */
    char *_ip;
    
    /** Additional flags string for device configuration */
    char *_flag;
    
    /** True if TCP client has connected (for TCP server mode) */
    bool _connected;
    
    /** True to enable MAVLink packetization on message boundaries */
    bool _packetise;

    /**
     * @brief Allocate RX and TX ring buffers
     * 
     * @param[in] rxS Receive buffer size in bytes
     * @param[in] txS Transmit buffer size in bytes
     * 
     * @note Called during begin()
     * @note Buffers are heap-allocated
     */
    void _allocate_buffers(uint16_t rxS, uint16_t txS);
    
    /**
     * @brief Deallocate RX and TX ring buffers
     * 
     * @note Called during end() or on reinitialization
     */
    void _deallocate_buffers();

    /**
     * @brief Parse device path string and create appropriate SerialDevice
     * 
     * @param[in] arg Device path string
     * @return AP_HAL::OwnPtr<SerialDevice> Pointer to created device
     * 
     * @details Parsing logic:
     *          - Starts with "/dev/": Create UARTDevice (physical serial)
     *          - Starts with "tcp:": Create TCPServerDevice or TCP client
     *          - Starts with "udp:": Create UDPDevice
     *          - Otherwise: Assume physical serial path
     * 
     * @note Returns nullptr if path format invalid
     */
    AP_HAL::OwnPtr<SerialDevice> _parseDevicePath(const char *arg);

    /**
     * Double-buffered receive timestamp for time constraint estimation
     * Avoids locking overhead during timestamp updates
     */
    uint64_t _receive_timestamp[2];
    
    /** Index into _receive_timestamp array (0 or 1) */
    uint8_t _receive_timestamp_idx;

protected:
    /** Device path string (e.g., "/dev/ttyS1", "tcp:14550") */
    const char *device_path;
    
    /** Initialization complete flag */
    volatile bool _initialised;

    /**
     * Receive ring buffer for batched reads
     * We use in-task ring buffers to reduce the system call cost
     * of ::read() and ::write() in the main loop
     */
    ByteBuffer _readbuf{0};
    
    /**
     * Transmit ring buffer for batched writes
     * We use in-task ring buffers to reduce the system call cost
     * of ::read() and ::write() in the main loop
     */
    ByteBuffer _writebuf{0};

    /**
     * @brief Write data to underlying device file descriptor
     * 
     * @param[in] buf Data buffer to write
     * @param[in] n Number of bytes to write
     * @return int Number of bytes written, or -1 on error
     * 
     * @note Virtual to allow override in derived classes
     * @note Non-blocking: Returns immediately even if not all bytes written
     */
    virtual int _write_fd(const uint8_t *buf, uint16_t n);
    
    /**
     * @brief Read data from underlying device file descriptor
     * 
     * @param[out] buf Buffer to receive data
     * @param[in] n Maximum bytes to read
     * @return int Number of bytes read, or -1 on error
     * 
     * @note Virtual to allow override in derived classes
     * @note Non-blocking: Returns immediately with available data
     */
    virtual int _read_fd(uint8_t *buf, uint16_t n);

    /** Semaphore protecting write operations (thread-safe writes) */
    Linux::Semaphore _write_mutex;

    /**
     * @brief Discard all data in receive buffer
     * 
     * @return bool True if successful
     * 
     * @note Clears RX ring buffer
     * @note Does not affect data in kernel buffers
     */
    bool _discard_input() override;
    
    /**
     * @brief Initialize UART with specified baud rate and buffer sizes
     * 
     * @param[in] b Baud rate in bits per second
     * @param[in] rxS Receive buffer size in bytes
     * @param[in] txS Transmit buffer size in bytes
     * 
     * @note Opens device, allocates buffers, configures parameters
     * @note Must call set_device_path() first
     */
    void _begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    
    /**
     * @brief Close UART and release resources
     * 
     * @note Deallocates buffers, closes device
     */
    void _end() override;
    
    /**
     * @brief Flush transmit buffer (wait for all data to be sent)
     * 
     * @note Blocks until TX buffer empty and device reports transmission complete
     */
    void _flush() override;
    
    /**
     * @brief Get number of bytes available in receive buffer
     * 
     * @return uint32_t Number of bytes ready to read
     * 
     * @note Data in RX ring buffer, not kernel buffer
     */
    uint32_t _available() override;
    
    /**
     * @brief Write data to transmit buffer
     * 
     * @param[in] buffer Data to write
     * @param[in] size Number of bytes to write
     * @return size_t Number of bytes actually written (may be less if buffer full)
     * 
     * @note Non-blocking: Returns immediately with bytes written
     * @note Check txspace() before write to avoid partial writes
     */
    size_t _write(const uint8_t *buffer, size_t size) override;
    
    /**
     * @brief Read data from receive buffer
     * 
     * @param[out] buffer Buffer to receive data
     * @param[in] count Maximum bytes to read
     * @return ssize_t Number of bytes read, or -1 on error
     * 
     * @note Non-blocking: Returns immediately with available data
     * @note Returns 0 if no data available (not an error)
     */
    ssize_t _read(uint8_t *buffer, uint16_t count) override WARN_IF_UNUSED;
};

}
