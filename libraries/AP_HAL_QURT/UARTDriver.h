/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file UARTDriver.h
 * @brief UART driver implementations for Qualcomm Hexagon DSP platform (QURT)
 * 
 * @details This file provides UART driver implementations for the QURT HAL,
 *          which runs on Qualcomm Hexagon DSP processors (e.g., Snapdragon Flight, VOXL).
 *          
 *          The QURT UART drivers use Remote Procedure Call (RPC) mechanisms to access
 *          physical UART hardware located on the application processor. The DSP code
 *          communicates with the apps processor using:
 *          - sl_client_uart_* RPC API for standard UART operations
 *          - qurt_rpc framing protocol for MAVLink-over-UDP transport
 *          
 *          Multiple specialized driver classes are provided:
 *          - UARTDriver: Base implementation with circular buffers and RPC
 *          - UARTDriver_Console: Debug console output via HAP_PRINTF
 *          - UARTDriver_MAVLinkUDP: MAVLink protocol bridged over UDP network
 *          - UARTDriver_Local: Direct serial port access for GPS, telemetry, peripherals
 *          
 *          All UART access from the DSP incurs inter-processor communication overhead,
 *          typically adding 50-500 microseconds of latency per operation depending on
 *          system load.
 * 
 * @note Physical UART hardware is located on the application processor, not the DSP.
 *       All DSP UART operations involve RPC calls to the apps processor.
 * 
 * @see interface.h for sl_client_uart_* function declarations
 * @see replace.cpp for qurt_rpc message framing protocol implementation
 * @see AP_HAL::UARTDriver for the abstract interface being implemented
 */

#pragma once

#include "AP_HAL_QURT.h"
#include "Semaphores.h"
#include <AP_HAL/utility/RingBuffer.h>
#include "ap_host/src/protocol.h"

/**
 * @class QURT::UARTDriver
 * @brief Base UART driver implementation for QURT platform using RPC for hardware access
 * 
 * @details This class provides the base UART driver functionality for the Qualcomm Hexagon DSP
 *          platform, implementing the AP_HAL::UARTDriver interface using Remote Procedure Calls
 *          to access physical UART hardware on the application processor.
 *          
 *          **Architecture Overview:**
 *          - Separate circular buffers for RX and TX operations (_readbuf, _writebuf)
 *          - Semaphore-protected buffer access for thread-safe I/O (_read_mutex, _write_mutex)
 *          - Non-blocking buffer operations for vehicle code
 *          - Periodic _timer_tick() called by scheduler to flush TX and poll RX
 *          
 *          **RPC Communication Model:**
 *          DSP code calls sl_client_uart_write() and sl_client_uart_read() functions which
 *          perform RPC to the application processor where the actual UART hardware is located.
 *          The apps processor handles the hardware I/O and returns data/status to the DSP.
 *          
 *          **Latency Characteristics:**
 *          Each RPC operation incurs inter-processor communication overhead:
 *          - Typical roundtrip time: 50-500 microseconds
 *          - Varies with IPC load and system state
 *          - Not suitable for hard real-time protocols requiring deterministic timing
 *          
 *          **Buffer Management:**
 *          - RX buffer: Filled by _fill_read_buffer() during _timer_tick()
 *          - TX buffer: Flushed by _write_pending_bytes() during _timer_tick()
 *          - Buffer sizes configured in _begin() call (rxS, txS parameters)
 *          - Typical buffer sizes: 1KB-4KB to fit DSP memory constraints
 * 
 * @note The _timer_tick() method is called periodically by the HAL scheduler (typically
 *       100-1000Hz) to perform background I/O operations. This rate determines maximum
 *       throughput and latency for UART operations.
 * 
 * @warning Buffer Overflow Risk: If vehicle code doesn't read RX data fast enough, the
 *          circular buffer will wrap and old data will be lost. There is no hardware
 *          flow control to prevent this. Monitor available() and read regularly.
 * 
 * @warning DSP Memory Constraints: Buffer sizes are limited by DSP memory availability.
 *          Allocating buffers larger than 4KB may cause allocation failures on
 *          memory-constrained DSP configurations. Always check is_initialized() after begin().
 */
class QURT::UARTDriver : public AP_HAL::UARTDriver
{
public:
    /**
     * @brief Check if UART driver has been successfully initialized
     * @return true if begin() was called and buffers allocated successfully, false otherwise
     */
    bool is_initialized() override;
    
    /**
     * @brief Check if data is pending in the TX buffer waiting to be sent
     * @return true if TX buffer contains unsent data, false if buffer is empty
     */
    bool tx_pending() override;

    /**
     * @brief Get available space in TX buffer for writing
     * @return Number of bytes that can be written without blocking
     * @note Returns 0 if buffer is full - write() will not block but will discard data
     */
    uint32_t txspace() override;

    /**
     * @brief Flush TX buffer to hardware via RPC
     * @return true if data was sent, false if no data pending or send failed
     * @note Called by _timer_tick() to perform background transmission
     * @note Subclasses override this to implement specific RPC mechanisms
     */
    virtual bool _write_pending_bytes(void)
    {
        return false;
    }
    
    /**
     * @brief Periodic timer callback for background UART I/O operations
     * @details Called by HAL scheduler at regular intervals (typically 100-1000Hz) to:
     *          - Flush TX buffer by calling _write_pending_bytes()
     *          - Fill RX buffer by calling _fill_read_buffer()
     *          - Perform any required housekeeping
     * @note This is the mechanism that drives all asynchronous UART I/O on QURT platform
     */
    virtual void _timer_tick(void) override;
    
    /**
     * @brief Poll hardware and fill RX buffer with received data via RPC
     * @note Called by _timer_tick() to perform background reception
     * @note Subclasses override this to implement specific RPC mechanisms
     */
    virtual void _fill_read_buffer(void) {}

    /**
     * @brief Get estimated bandwidth capability in bytes per second
     * @return Bandwidth estimate in bytes/second (default 5760 bytes/sec = 57600 baud / 10 bits/byte)
     * @note Used by MAVLink for flow control and rate limiting
     */
    virtual uint32_t bw_in_bytes_per_second() const override
    {
        return 5760;
    }
    
    /**
     * @brief Get flow control configuration
     * @return Flow control mode (default: FLOW_CONTROL_DISABLE)
     * @note Base driver has no hardware flow control support - data loss occurs on overflow
     */
    virtual enum AP_HAL::UARTDriver::flow_control get_flow_control(void) override
    {
        return AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE;
    }

protected:
    /**
     * @brief Initialize UART with specified parameters
     * @param[in] b Baud rate (e.g., 57600, 115200)
     * @param[in] rxS RX buffer size in bytes (typically 1024-4096)
     * @param[in] txS TX buffer size in bytes (typically 1024-4096)
     * @note Allocates circular buffers and performs RPC to configure hardware UART
     * @warning May fail if buffer allocation fails due to DSP memory constraints
     */
    virtual void _begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    
    /**
     * @brief Write data to TX buffer
     * @param[in] buffer Pointer to data to write
     * @param[in] size Number of bytes to write
     * @return Number of bytes actually buffered (may be less than size if buffer full)
     * @note Non-blocking operation - returns immediately even if buffer is full
     * @warning If buffer is full, data is discarded and return value is 0 or partial count
     */
    size_t _write(const uint8_t *buffer, size_t size) override;
    
    /**
     * @brief Read data from RX buffer
     * @param[out] buffer Buffer to receive data
     * @param[in] size Maximum number of bytes to read
     * @return Number of bytes actually read, or -1 on error
     * @note Non-blocking operation - returns immediately with available data
     */
    ssize_t _read(uint8_t *buffer, uint16_t size) override WARN_IF_UNUSED;
    
    /**
     * @brief Close UART and release resources
     * @note Deallocates buffers and closes RPC connection to hardware
     */
    void _end() override;
    
    /**
     * @brief Flush TX buffer - wait until all pending data is sent
     * @note Blocks until _write_pending_bytes() returns false (all data transmitted)
     */
    void _flush() override;
    
    /**
     * @brief Get number of bytes available to read from RX buffer
     * @return Number of bytes in RX buffer ready to be read
     */
    uint32_t _available() override;
    
    /**
     * @brief Discard all data in RX buffer
     * @return true if buffer was cleared successfully, false otherwise
     * @note Used to synchronize protocol state after errors
     */
    bool _discard_input() override;
    
    volatile bool _initialised;  ///< Initialization flag - true if begin() succeeded

    ByteBuffer _readbuf{0};  ///< Circular buffer for received data (RX)
    ByteBuffer _writebuf{0}; ///< Circular buffer for data to transmit (TX)

    QURT::Semaphore _read_mutex;  ///< Protects _readbuf access for thread-safe reads
    QURT::Semaphore _write_mutex; ///< Protects _writebuf access for thread-safe writes
};

/**
 * @class QURT::UARTDriver_Console
 * @brief Console/debug UART driver for system boot messages and debugging output
 * 
 * @details This specialized UART driver provides console output functionality for the
 *          ArduPilot HAL console (hal.console->printf()). Console output is used for:
 *          - Boot sequence messages during initialization
 *          - Debug printf() statements
 *          - Error and warning messages
 *          - System status information
 *          
 *          **Output Routing:**
 *          Console output is routed through HAP_PRINTF, which is a Qualcomm-provided
 *          debug output mechanism. HAP_PRINTF sends output to the QURT debug log system,
 *          which can be viewed on the host system.
 *          
 *          **Viewing Console Output:**
 *          On VOXL/Snapdragon platforms, console output can be viewed using:
 *          - adb logcat (Android Debug Bridge)
 *          - QURT debug log utilities
 *          - Typically appears with "HAP" or "DSP" tags in logcat output
 *          
 *          **Typical Usage:**
 *          Console is automatically connected during HAL initialization and is available
 *          via hal.console global pointer. It is typically mapped to the first USB
 *          serial connection for easy debugging during development.
 * 
 * @note Console output goes to system debug log, not to a physical UART. It cannot
 *       be used for bidirectional communication with external devices.
 * 
 * @note Console is primarily for debugging and should not be used for high-rate
 *       logging in production as it may impact DSP performance.
 */
class QURT::UARTDriver_Console : public QURT::UARTDriver
{
public:
    using UARTDriver::UARTDriver;
    
    /**
     * @brief Formatted print to console output
     * @param[in] fmt Printf-style format string
     * @param[in] ... Variable arguments matching format string
     * @note Output is sent to HAP_PRINTF for display in system debug log (logcat)
     */
    virtual void printf(const char *fmt, ...) override;
};

/**
 * @class QURT::UARTDriver_MAVLinkUDP
 * @brief MAVLink-over-UDP transport driver using qurt_rpc framing protocol
 * 
 * @details This specialized UART driver bridges the MAVLink serial protocol to UDP
 *          networking for communication with ground control stations over WiFi/Ethernet.
 *          It provides a transparent serial interface to vehicle code while actually
 *          transmitting and receiving MAVLink packets over UDP network connections.
 *          
 *          **Protocol Architecture:**
 *          MAVLink packets from vehicle code → Serial-like API → qurt_rpc framing →
 *          UDP packets → Application processor → Network interface → Ground station
 *          
 *          **qurt_rpc Framing Protocol:**
 *          MAVLink data is encapsulated in qurt_rpc message frames for UDP transport:
 *          - Adds packet headers with sequence numbers for ordering
 *          - Includes message type identifiers
 *          - Provides basic packet validation
 *          - See replace.cpp for detailed protocol implementation
 *          
 *          **Network Addressing:**
 *          - Default listening port: UDP 14550 (standard MAVLink port)
 *          - Accepts incoming MAVLink from any ground control station
 *          - Automatically replies to the source address of received packets
 *          - UDP packets are sent/received via apps processor network stack
 *          
 *          **Bandwidth and Flow Control:**
 *          - Much higher bandwidth than physical serial ports (typically 250 kbytes/sec)
 *          - Flow control is enabled to prevent overwhelming network link
 *          - MAVLink stream rate limiting applied based on bandwidth estimate
 *          
 *          **Multiple Instances:**
 *          Supports multiple MAVLink UDP instances for simultaneous connections to
 *          multiple ground stations or companion computers (inst parameter).
 * 
 * @note UDP is an unreliable transport - packets may be lost, duplicated, or reordered.
 *       MAVLink protocol handles retransmission at the application layer, but some
 *       loss is expected especially on congested WiFi networks.
 * 
 * @warning Packet Loss: UDP provides no delivery guarantees. Mission-critical commands
 *          may be lost if network is congested. MAVLink handles retries for most critical
 *          commands, but telemetry streaming data may be permanently lost. Monitor
 *          RADIO_STATUS messages for packet loss statistics.
 * 
 * @see replace.cpp for qurt_rpc framing protocol implementation details
 * @see ap_host/src/protocol.h for qurt_rpc_msg structure definition
 */
class QURT::UARTDriver_MAVLinkUDP : public QURT::UARTDriver
{
public:
    /**
     * @brief Construct MAVLink UDP driver for specified instance
     * @param[in] instance Instance number (0, 1, 2...) for multiple simultaneous connections
     * @note Each instance uses a different UDP port: 14550 + instance
     */
    UARTDriver_MAVLinkUDP(uint8_t instance);

    /**
     * @brief Flush TX buffer to network via qurt_rpc framing and UDP
     * @return true if data was sent, false if no data pending or send failed
     * @note Encapsulates MAVLink data in qurt_rpc frames with sequence numbers
     */
    bool _write_pending_bytes(void) override;

    /**
     * @brief Check received packet sequence number for loss detection
     * @param[in] seq Sequence number from received qurt_rpc packet
     * @note Detects packet loss by checking for gaps in sequence numbers
     * @note Logs warning if packets are lost (sequence gap detected)
     */
    void check_rx_seq(uint32_t seq);

    /**
     * @brief Get network bandwidth estimate
     * @return Bandwidth in bytes/second (750 kbytes/sec = 250 kbytes/sec * 3 for safety margin)
     * @note Much higher than physical serial ports due to network transport
     */
    uint32_t bw_in_bytes_per_second() const override
    {
        return 250000 * 3;
    }
    
    /**
     * @brief Get flow control configuration for network transport
     * @return FLOW_CONTROL_ENABLE to enable MAVLink rate limiting
     * @note Flow control prevents overwhelming network link with telemetry
     */
    enum AP_HAL::UARTDriver::flow_control get_flow_control(void) override
    {
        return AP_HAL::UARTDriver::FLOW_CONTROL_ENABLE;
    }

private:
    /**
     * @brief Callback for received MAVLink data from network
     * @param[in] msg Pointer to qurt_rpc message containing MAVLink data
     * @param[in] p Pointer to UARTDriver_MAVLinkUDP instance (this)
     * @note Called by RPC layer when UDP packet arrives from apps processor
     */
    static void _mavlink_data_cb(const struct qurt_rpc_msg *msg, void *p);
    
    uint8_t inst;      ///< Instance number for this MAVLink connection (0, 1, 2...)
    uint32_t tx_seq;   ///< Transmit sequence number for outgoing packets
    uint32_t rx_seq;   ///< Expected receive sequence number for incoming packets
};

/**
 * @class QURT::UARTDriver_Local
 * @brief Local serial port driver for GPS, telemetry radios, and external peripherals
 * 
 * @details This specialized UART driver provides access to physical serial ports on the
 *          board for connecting external devices such as:
 *          - GPS receivers (UBLOX, NMEA, etc.)
 *          - Telemetry radios (SiK radio, RFD900, XBee)
 *          - Rangefinders and distance sensors
 *          - Companion computers
 *          - Other serial peripherals
 *          
 *          **Hardware Access Model:**
 *          Physical UART hardware is located on the application processor, not the DSP.
 *          This driver uses sl_client_uart_* RPC API to communicate with the apps
 *          processor UART driver, which performs actual hardware I/O operations.
 *          
 *          **Port Identification:**
 *          Each local UART is identified by a port_id number that corresponds to a
 *          specific hardware UART on the board. The port_id is used with
 *          sl_client_uart_open() to establish the RPC connection to the correct UART.
 *          
 *          **Baud Rate and Configuration:**
 *          - Configurable baud rate (typically 9600 to 921600)
 *          - 8N1 format (8 data bits, no parity, 1 stop bit) is standard
 *          - Hardware flow control (RTS/CTS) not currently supported
 *          
 *          **Receive Timestamps:**
 *          This driver tracks receive timestamps to support protocols that require
 *          accurate timing information (e.g., GPS time-of-week synchronization).
 *          The receive_time_constraint_us() method estimates when a packet arrived.
 *          
 *          **Performance Characteristics:**
 *          - Adds 50-500us latency due to RPC overhead
 *          - Maximum throughput limited by RPC bandwidth (~100-200 kbytes/sec typical)
 *          - Suitable for most sensor protocols but not hard real-time applications
 * 
 * @note Each physical UART on the board requires a separate UARTDriver_Local instance
 *       with unique port_id. The mapping of port_id to physical UART pins is
 *       board-specific and defined in the QURT board configuration.
 * 
 * @warning RPC Latency: All I/O operations involve inter-processor communication.
 *          High-rate data streams (>100 kHz) may experience buffer overflow if the
 *          RPC cannot keep up with the data rate. Monitor buffer usage for high-rate sensors.
 */
class QURT::UARTDriver_Local : public QURT::UARTDriver
{
public:
    /**
     * @brief Construct local UART driver for specified hardware port
     * @param[in] _port_id Hardware UART port identifier (board-specific)
     * @note port_id maps to physical UART instance on application processor
     */
    UARTDriver_Local(uint8_t _port_id) : port_id(_port_id) {}

    /**
     * @brief Get estimated bandwidth based on configured baud rate
     * @return Bandwidth in bytes/second (baud_rate / 10 bits per byte)
     * @note Accounts for start/stop bits: 10 bits per byte for 8N1 format
     */
    uint32_t bw_in_bytes_per_second() const override
    {
        return baudrate?baudrate/10:5760;
    }

    /**
     * @brief Flush TX buffer to hardware UART via sl_client_uart_write RPC
     * @return true if data was sent successfully, false otherwise
     * @note Calls RPC to apps processor to perform actual hardware write
     */
    bool _write_pending_bytes(void) override;
    
    /**
     * @brief Initialize UART with specified parameters and open hardware port
     * @param[in] b Baud rate (e.g., 9600, 57600, 115200)
     * @param[in] rxS RX buffer size in bytes
     * @param[in] txS TX buffer size in bytes
     * @note Uses sl_client_uart_open() to establish RPC connection to hardware UART
     * @warning May fail if port_id is invalid or UART is already in use
     */
    void _begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    
    /**
     * @brief Poll hardware UART and fill RX buffer with received data
     * @note Uses sl_client_uart_read() RPC to retrieve data from apps processor
     * @note Called periodically by _timer_tick() to perform background reception
     * @note Updates receive_timestamp_us for timing-sensitive protocols
     */
    void _fill_read_buffer(void) override;

    /**
     * @brief Get configured baud rate
     * @return Current baud rate in bits per second
     */
    uint32_t get_baud_rate() const override
    {
        return baudrate;
    }

    /**
     * @brief Estimate timestamp when packet started arriving
     * @param[in] nbytes Number of bytes in the packet
     * @return Estimated timestamp in microseconds when first byte of packet arrived
     * @details Calculates arrival time by subtracting transmission time from current
     *          timestamp: arrival_time = current_time - (nbytes * bits_per_byte / baud_rate)
     * @note Used by GPS and other timing-sensitive protocols to synchronize measurements
     */
    uint64_t receive_time_constraint_us(uint16_t nbytes) override;

private:
    const uint8_t port_id;         ///< Hardware UART port identifier (immutable)
    int fd = -1;                   ///< File descriptor for sl_client_uart connection (-1 if closed)
    uint32_t baudrate;             ///< Configured baud rate in bits per second
    uint64_t receive_timestamp_us; ///< Timestamp of most recent data reception (microseconds)
};
