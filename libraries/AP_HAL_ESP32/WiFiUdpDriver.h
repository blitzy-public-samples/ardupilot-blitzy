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
 * @file WiFiUdpDriver.h
 * @brief ESP32 WiFi UDP driver for MAVLink telemetry over WiFi
 * 
 * @details This file implements a UDP-based UART driver for ESP32 that enables
 *          wireless MAVLink communication over WiFi networks. The driver uses
 *          lwIP sockets for UDP packet transmission and reception, providing
 *          a connectionless transport layer suitable for telemetry data.
 * 
 *          UDP Protocol Advantages:
 *          - Connectionless operation (no handshake overhead)
 *          - Support for broadcast and multicast
 *          - Lower latency compared to TCP
 *          - Simpler protocol suitable for real-time telemetry
 * 
 *          The driver integrates with ESP32's WiFi stack and FreeRTOS,
 *          using a dedicated task for receiving UDP packets and managing
 *          thread-safe buffer access for concurrent read/write operations.
 * 
 * @note Default MAVLink UDP port is 14550
 * @see WiFiDriver for TCP-based alternative
 */

#pragma once

#include <AP_HAL/UARTDriver.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_HAL_ESP32/AP_HAL_ESP32.h>
#include <AP_HAL_ESP32/Semaphores.h>
#include "lwip/sockets.h"
#include "esp_event.h"

#ifndef WIFI_MAX_CONNECTION
#define WIFI_MAX_CONNECTION 5
#endif

/**
 * @class WiFiUdpDriver
 * @brief UDP-based UART driver for ESP32 WiFi telemetry communication
 * 
 * @details This class implements a UDP socket-based transport layer for MAVLink
 *          telemetry over ESP32 WiFi connections. It extends AP_HAL::UARTDriver
 *          to provide wireless communication using lwIP UDP sockets.
 * 
 *          Architecture:
 *          - Uses lwIP sockets API for UDP packet I/O
 *          - Implements ring buffers for transmit and receive data
 *          - Spawns dedicated FreeRTOS task for UDP receive loop
 *          - Thread-safe buffer access via semaphore protection
 * 
 *          UDP Transport Characteristics:
 *          - Connectionless protocol (no TCP handshake overhead)
 *          - Lower latency suitable for real-time telemetry
 *          - Broadcast support for multi-GCS scenarios
 *          - Packet-oriented (preserves message boundaries)
 * 
 *          Buffer Architecture:
 *          - TX ring buffer: 1024 bytes
 *          - RX ring buffer: 1024 bytes
 *          - Packet buffer: 255 bytes (sized for MAVLink packets)
 * 
 *          Thread Safety:
 *          - _read_mutex protects receive buffer access
 *          - _write_mutex protects transmit buffer access
 *          - FreeRTOS task handles asynchronous UDP reception
 * 
 * @note Default MAVLink UDP port is 14550
 * @warning Buffer sizes limit maximum packet size and throughput
 * @see WiFiDriver for TCP-based alternative with connection management
 */
class ESP32::WiFiUdpDriver : public AP_HAL::UARTDriver
{
public:
    /**
     * @brief Constructor for WiFiUdpDriver
     * 
     * @details Initializes the UDP driver in NOT_INITIALIZED state.
     *          Actual WiFi and socket initialization occurs in _begin().
     */
    WiFiUdpDriver();

    /**
     * @brief Check if the UDP driver has been initialized
     * 
     * @details Returns true once WiFi initialization and UDP socket creation
     *          are complete (state >= INITIALIZED). This indicates the driver
     *          is ready for communication operations.
     * 
     * @return true if driver is initialized and ready
     * @return false if driver is in NOT_INITIALIZED state
     * 
     * @note Does not indicate active WiFi connection, only initialization status
     */
    bool is_initialized() override;
    
    /**
     * @brief Check if transmit data is pending in the write buffer
     * 
     * @details Queries the write ring buffer to determine if data is waiting
     *          to be transmitted via UDP. Used by MAVLink layer to manage
     *          transmission flow.
     * 
     * @return true if data is available in transmit buffer
     * @return false if transmit buffer is empty
     */
    bool tx_pending() override;

    /**
     * @brief Get available space in transmit buffer
     * 
     * @details Returns the number of bytes that can be written to the transmit
     *          buffer without blocking. Used for flow control to prevent buffer
     *          overflow when queuing outgoing MAVLink messages.
     * 
     * @return Number of bytes available in TX buffer (max TX_BUF_SIZE=1024)
     * 
     * @note Thread-safe, protected by _write_mutex
     */
    uint32_t txspace() override;

    /**
     * @brief Report nominal bandwidth in bytes per second
     * 
     * @details Returns theoretical WiFi bandwidth for planning purposes.
     *          Actual throughput depends on WiFi link quality, network
     *          congestion, and UDP packet loss.
     * 
     * @return Nominal bandwidth in bytes/second (1000*1024 = 1,024,000 bytes/sec)
     * 
     * @note This is a theoretical maximum, not measured throughput
     */
    uint32_t bw_in_bytes_per_second() const override
    {
        return 1000*1024;
    }


private:
    /**
     * @enum ConnectionState
     * @brief UDP socket lifecycle states
     * 
     * @details Tracks the initialization and connection state of the UDP driver:
     *          - NOT_INITIALIZED: Initial state, WiFi not configured
     *          - INITIALIZED: WiFi initialized, UDP socket created
     *          - CONNECTED: UDP socket bound and ready for I/O
     * 
     *          State Transitions:
     *          NOT_INITIALIZED -> INITIALIZED: After initialize_wifi() completes
     *          INITIALIZED -> CONNECTED: After UDP socket bind succeeds
     * 
     * @note Unlike TCP, UDP is connectionless; CONNECTED indicates socket is bound
     */
    enum ConnectionState {
        NOT_INITIALIZED,    ///< WiFi and socket not initialized
        INITIALIZED,        ///< WiFi initialized, socket created
        CONNECTED           ///< UDP socket bound and operational
    };
    
    const size_t TX_BUF_SIZE = 1024;  ///< Transmit ring buffer size in bytes
    const size_t RX_BUF_SIZE = 1024;  ///< Receive ring buffer size in bytes
    
    /**
     * @brief Temporary packet buffer for UDP send/receive operations
     * 
     * @details Sized at 255 bytes to accommodate typical MAVLink packets.
     *          Original size of 32 bytes was too small, causing slow parameter
     *          reads as most MAVLink packets exceed this size. Size of 128 bytes
     *          was still insufficient due to packet overhead and fragmentation.
     * 
     *          MAVLink v2 maximum packet size is 280 bytes (header + payload + signature).
     *          This 255-byte buffer handles most common packets; larger packets
     *          may require fragmentation.
     * 
     * @warning Buffer size limits maximum single UDP packet size
     * @note Increased from 32 -> 128 -> 255 bytes based on MAVLink packet analysis
     */
    uint8_t _buffer[255];
    
    ByteBuffer _readbuf{0};   ///< Ring buffer for received data (RX_BUF_SIZE bytes)
    ByteBuffer _writebuf{0};  ///< Ring buffer for transmit data (TX_BUF_SIZE bytes)
    
    /**
     * @brief Semaphore protecting write buffer access
     * 
     * @details Ensures thread-safe access to _writebuf between the main thread
     *          (writing MAVLink messages) and the WiFi task (reading for transmission).
     *          Prevents data corruption during concurrent buffer operations.
     */
    Semaphore _write_mutex;
    
    /**
     * @brief Semaphore protecting read buffer access
     * 
     * @details Ensures thread-safe access to _readbuf between the WiFi task
     *          (receiving UDP packets) and the main thread (reading MAVLink messages).
     *          Prevents data corruption during concurrent buffer operations.
     */
    Semaphore _read_mutex;
    
    ConnectionState _state;   ///< Current driver state (NOT_INITIALIZED, INITIALIZED, CONNECTED)

    int accept_socket;        ///< UDP socket file descriptor (lwIP socket handle)

    tskTaskControlBlock* _wifi_task_handle;  ///< FreeRTOS task handle for UDP receive loop
    
    /**
     * @brief Initialize WiFi subsystem and create UDP socket
     * 
     * @details Performs the following initialization sequence:
     *          1. Initialize ESP32 WiFi subsystem
     *          2. Register event handlers with esp_event system
     *          3. Configure WiFi mode (AP, STA, or AP+STA)
     *          4. Create UDP socket using lwIP sockets API
     *          5. Bind socket to MAVLink UDP port (default 14550)
     *          6. Transition state to INITIALIZED
     * 
     *          Integration with ESP-IDF:
     *          - Uses esp_event for WiFi event notifications
     *          - Leverages lwIP for UDP/IP stack
     *          - Configures WiFi parameters from NVS (if available)
     * 
     * @note Called once during _begin() to set up WiFi infrastructure
     * @see esp_event.h for WiFi event handling
     */
    void initialize_wifi();
    
    /**
     * @brief Read all available data from UDP socket into receive buffer
     * 
     * @details Performs non-blocking read of UDP packets from socket and
     *          stores received data in the _readbuf ring buffer. Called
     *          repeatedly by the WiFi receive task.
     * 
     *          Operation:
     *          1. Check for available data on UDP socket (non-blocking)
     *          2. Read packet into _buffer (up to 255 bytes)
     *          3. Copy data to _readbuf ring buffer (with _read_mutex protection)
     * 
     * @return true if data was successfully read and buffered
     * @return false if no data available or buffer full
     * 
     * @note Thread-safe, uses _read_mutex for buffer protection
     * @warning Packets larger than 255 bytes will be truncated
     */
    bool read_all();
    
    /**
     * @brief Write pending data from transmit buffer to UDP socket
     * 
     * @details Reads data from _writebuf ring buffer and transmits it via
     *          UDP socket. Called periodically to flush outgoing MAVLink messages.
     * 
     *          Operation:
     *          1. Read data from _writebuf into _buffer (with _write_mutex)
     *          2. Send packet via UDP socket using sendto()
     *          3. Remove sent data from _writebuf
     * 
     * @return true if data was successfully transmitted
     * @return false if socket error or no data to send
     * 
     * @note Thread-safe, uses _write_mutex for buffer protection
     * @note UDP is unreliable; no delivery guarantee
     */
    bool write_data();
    
    bool start_listen();      ///< Start listening for UDP packets (legacy, may be unused for UDP)
    bool try_accept();        ///< Accept connection (legacy TCP method, unused for UDP)
    
    /**
     * @brief Static FreeRTOS task entry point for UDP receive loop
     * 
     * @details This static method serves as the FreeRTOS task function that
     *          continuously receives UDP packets in the background. It runs in
     *          a separate FreeRTOS task context to avoid blocking the main loop.
     * 
     *          Task Responsibilities:
     *          - Continuously call read_all() to receive UDP packets
     *          - Call write_data() to transmit pending data
     *          - Handle UDP socket I/O asynchronously
     *          - Yield to other tasks between iterations
     * 
     *          FreeRTOS Integration:
     *          - Created via xTaskCreate() in initialize_wifi()
     *          - Runs at configured priority on ESP32 core
     *          - Uses task stack allocated by FreeRTOS
     * 
     * @param[in] arg Pointer to WiFiUdpDriver instance (casted from void*)
     * 
     * @note Static method required for FreeRTOS task API compatibility
     * @note Task runs indefinitely until driver is destroyed
     * @warning Task stack size must accommodate lwIP socket operations
     */
    static void _wifi_thread2(void* arg);

protected:
    /**
     * @brief Initialize and start the WiFi UDP driver
     * 
     * @details Initializes WiFi subsystem, creates UDP socket, allocates ring buffers,
     *          and spawns the FreeRTOS receive task. This is the main initialization
     *          entry point called by the HAL.
     * 
     * @param[in] b        Baud rate (ignored for WiFi, retained for UART compatibility)
     * @param[in] rxS      Receive buffer size (uses RX_BUF_SIZE=1024 if 0)
     * @param[in] txS      Transmit buffer size (uses TX_BUF_SIZE=1024 if 0)
     * 
     * @note Baud rate parameter is not applicable to WiFi but maintained for API compatibility
     */
    void _begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    
    /**
     * @brief Write data to transmit buffer for UDP transmission
     * 
     * @details Copies data into the transmit ring buffer (_writebuf) for subsequent
     *          UDP transmission by the WiFi task. Thread-safe operation protected
     *          by _write_mutex.
     * 
     * @param[in] buffer   Pointer to data to transmit
     * @param[in] size     Number of bytes to write
     * 
     * @return Number of bytes actually written (may be less than size if buffer full)
     * 
     * @note Non-blocking; returns immediately if buffer full
     */
    size_t _write(const uint8_t *buffer, size_t size) override;
    
    /**
     * @brief Read received data from receive buffer
     * 
     * @details Reads data from the receive ring buffer (_readbuf) that was previously
     *          received from UDP socket by the WiFi task. Thread-safe operation
     *          protected by _read_mutex.
     * 
     * @param[out] buf     Buffer to store received data
     * @param[in]  count   Maximum number of bytes to read
     * 
     * @return Number of bytes actually read (0 if no data available)
     * 
     * @note Non-blocking; returns immediately with available data
     */
    ssize_t _read(uint8_t *buf, uint16_t count) override;
    
    /**
     * @brief Get number of bytes available in receive buffer
     * 
     * @details Returns the count of received bytes waiting in _readbuf ring buffer.
     *          Used by MAVLink layer to determine if messages are pending.
     * 
     * @return Number of bytes available for reading
     * 
     * @note Thread-safe, protected by _read_mutex
     */
    uint32_t _available() override;
    
    /**
     * @brief Discard all pending input data in receive buffer
     * 
     * @details Clears the receive ring buffer, discarding any received UDP packets
     *          that have not been read. Used for error recovery or protocol resets.
     * 
     * @return true if input was successfully discarded
     * @return false on error
     * 
     * @note Thread-safe, protected by _read_mutex
     */
    bool _discard_input() override;
    
    /**
     * @brief Shutdown the WiFi UDP driver
     * 
     * @details Closes the UDP socket, terminates the WiFi receive task,
     *          and releases allocated resources.
     * 
     * @note Transitions state back to NOT_INITIALIZED
     */
    void _end() override;
    
    /**
     * @brief Flush pending transmit data
     * 
     * @details Ensures all buffered transmit data is sent via UDP socket.
     *          For UDP this triggers immediate transmission of pending packets.
     * 
     * @note UDP is unreliable; flush does not guarantee delivery
     */
    void _flush() override;
};
