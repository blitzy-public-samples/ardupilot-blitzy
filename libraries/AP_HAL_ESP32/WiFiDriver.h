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
 * @file WiFiDriver.h
 * @brief ESP32 WiFi TCP driver for MAVLink telemetry over WiFi
 * 
 * @details This driver implements TCP-based MAVLink communication over WiFi
 *          using the ESP32's native esp_wifi and lwIP TCP/IP stack. It provides
 *          a UARTDriver-compatible interface for transparent WiFi telemetry.
 *          
 *          The driver supports two WiFi operational modes:
 *          - SoftAP (Access Point) mode: ESP32 creates its own WiFi network
 *          - Station mode: ESP32 connects to an existing WiFi network
 *          
 *          TCP connections are established on port 5760 (default MAVLink port)
 *          and support multiple concurrent client connections up to
 *          WIFI_MAX_CONNECTION limit.
 * 
 * @note Default TCP port is 5760 for MAVLink compatibility
 * @see WiFiUdpDriver for UDP-based alternative WiFi transport
 */

#pragma once

#include <AP_HAL/UARTDriver.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_HAL_ESP32/AP_HAL_ESP32.h>
#include <AP_HAL_ESP32/Semaphores.h>


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief Maximum number of simultaneous TCP client connections
 * 
 * @warning Increasing this value requires more memory for socket management
 *          and may impact performance on resource-constrained ESP32 devices.
 *          Each connection maintains separate socket state and buffers.
 */
#ifndef WIFI_MAX_CONNECTION
#define WIFI_MAX_CONNECTION 5
#endif

/**
 * @class WiFiDriver
 * @brief TCP-based MAVLink transport over WiFi using ESP32 esp_wifi and lwIP
 * 
 * @details This driver provides a UARTDriver-compatible interface for MAVLink
 *          telemetry over WiFi TCP connections. It integrates with the ESP32
 *          native WiFi stack (esp_wifi) and the lwIP TCP/IP stack for network
 *          communication.
 *          
 *          Key features:
 *          - TCP server on port 5760 (configurable)
 *          - Multiple concurrent client connections (up to WIFI_MAX_CONNECTION)
 *          - Buffered non-blocking I/O using ByteBuffer with Semaphore protection
 *          - FreeRTOS task for network operations
 *          - SoftAP (Access Point) or Station mode operation
 *          
 *          WiFi Configuration Modes:
 *          - SoftAP mode: ESP32 creates WiFi access point for direct connection
 *          - Station mode: ESP32 connects to existing WiFi network infrastructure
 *          
 *          Architecture:
 *          - WiFi initialization uses NVS (Non-Volatile Storage) partition for
 *            configuration persistence
 *          - Network interface (netif) setup via ESP-IDF netif abstraction
 *          - Event loop integration for WiFi connection state management
 *          - Socket management with accept_socket and socket_list array
 *          - Separate FreeRTOS task (_wifi_thread) handles network I/O
 *          
 *          Thread Safety:
 *          - Write operations protected by _write_mutex Semaphore
 *          - Read operations use ByteBuffer with internal synchronization
 * 
 * @note Inherits from AP_HAL::UARTDriver to provide transparent serial-like
 *       interface for MAVLink message routing
 * @see WiFiUdpDriver for UDP-based WiFi transport alternative
 */
class ESP32::WiFiDriver : public AP_HAL::UARTDriver
{
public:
    /**
     * @brief Construct WiFiDriver instance
     * 
     * @details Initializes connection state to NOT_INITIALIZED and prepares
     *          socket management structures. Actual WiFi initialization occurs
     *          in initialize_wifi() called from _begin().
     */
    WiFiDriver();

    /**
     * @brief Check if WiFi driver is initialized and ready
     * 
     * @return true if WiFi is initialized and operational, false otherwise
     * 
     * @note Returns true only after successful WiFi initialization and
     *       network interface configuration
     */
    bool is_initialized() override;
    
    /**
     * @brief Check if transmit data is pending in write buffer
     * 
     * @return true if data is waiting to be transmitted, false if buffer empty
     * 
     * @details Checks the write buffer (ByteBuffer _writebuf) for pending data
     *          that has not yet been sent to connected TCP clients.
     */
    bool tx_pending() override;

    /**
     * @brief Get available space in transmit buffer
     * 
     * @return Number of bytes available for writing in transmit buffer
     * 
     * @details Returns the free space in _writebuf (ByteBuffer). Applications
     *          should check this before large writes to avoid blocking or
     *          data loss.
     * 
     * @note Buffer size is TX_BUF_SIZE (default 1024 bytes)
     */
    uint32_t txspace() override;

    /**
     * @brief Get estimated bandwidth in bytes per second
     * 
     * @return Bandwidth estimate in bytes/second (1000 KB/s = 1 MB/s)
     * 
     * @details Returns a nominal bandwidth estimate for WiFi connections.
     *          Actual throughput depends on WiFi signal strength, network
     *          congestion, and TCP window size.
     * 
     * @note This is an estimate used for scheduling and buffer management,
     *       not a guaranteed throughput measurement
     */
    uint32_t bw_in_bytes_per_second() const override
    {
        return 1000*1024;
    }

    /**
     * @brief Flag indicating more data is available for reading
     * 
     * @details Used by the network thread to signal the main application
     *          that additional data has been received and buffered.
     */
    bool _more_data;

private:
    /**
     * @enum ConnectionState
     * @brief WiFi driver connection lifecycle states
     * 
     * @details Tracks the initialization and connection progression:
     *          NOT_INITIALIZED → INITIALIZED → CONNECTED
     *          
     *          - NOT_INITIALIZED: Driver constructed but WiFi not initialized
     *          - INITIALIZED: WiFi stack initialized, network interface ready,
     *                        listening for connections
     *          - CONNECTED: At least one TCP client connected and active
     */
    enum ConnectionState {
        NOT_INITIALIZED,  ///< WiFi stack not yet initialized
        INITIALIZED,      ///< WiFi initialized, waiting for connections
        CONNECTED         ///< One or more clients connected
    };
    
    /**
     * @brief Transmit buffer size in bytes
     * 
     * @warning Increasing TX_BUF_SIZE requires more heap memory. ESP32 has
     *          limited RAM (~520KB total, with significant portions used by
     *          WiFi stack and FreeRTOS). Monitor heap usage when adjusting.
     */
    const size_t TX_BUF_SIZE = 1024;
    
    /**
     * @brief Receive buffer size in bytes
     * 
     * @warning Increasing RX_BUF_SIZE requires more heap memory. Consider
     *          total memory usage including WiFi stack buffers, TCP buffers,
     *          and application needs when adjusting buffer sizes.
     */
    const size_t RX_BUF_SIZE = 1024;
    
    /**
     * @brief Temporary buffer for socket I/O operations
     * 
     * @details Used as staging buffer for recv() and send() socket operations
     *          to avoid frequent small allocations during network I/O.
     */
    uint8_t _buffer[32];
    
    /**
     * @brief Receive buffer for incoming TCP data
     * 
     * @details ByteBuffer provides thread-safe buffered storage for data
     *          received from TCP clients. Data is read from sockets in
     *          _wifi_thread and consumed by application via _read() method.
     *          
     *          Size: RX_BUF_SIZE (default 1024 bytes)
     */
    ByteBuffer _readbuf{0};
    
    /**
     * @brief Transmit buffer for outgoing TCP data
     * 
     * @details ByteBuffer provides thread-safe buffered storage for data
     *          to be transmitted to TCP clients. Data is written by application
     *          via _write() method and transmitted in _wifi_thread.
     *          
     *          Size: TX_BUF_SIZE (default 1024 bytes)
     *          
     * @note Protected by _write_mutex semaphore for thread-safe writes
     */
    ByteBuffer _writebuf{0};
    
    /**
     * @brief Semaphore protecting write buffer access
     * 
     * @details Ensures thread-safe access to _writebuf when multiple tasks
     *          attempt to write data. Prevents buffer corruption from
     *          concurrent writes between application task and network task.
     */
    Semaphore _write_mutex;
    
    /**
     * @brief Current connection state
     * 
     * @details Tracks driver lifecycle through ConnectionState enum values.
     *          State transitions managed by initialize_wifi(), start_listen(),
     *          and try_accept() methods.
     */
    ConnectionState _state;
    
    /**
     * @brief Server socket descriptor for accepting new connections
     * 
     * @details TCP listening socket bound to port 5760 (default MAVLink port).
     *          Used in try_accept() to accept incoming client connections which
     *          are then stored in socket_list array.
     * 
     * @note Set to -1 when not listening or on socket errors
     */
    short accept_socket;
    
    /**
     * @brief Array of active client socket descriptors
     * 
     * @details Manages up to WIFI_MAX_CONNECTION simultaneous TCP client
     *          connections. Each element stores a socket file descriptor for
     *          an active connection, or -1 for unused slots.
     *          
     *          Socket management:
     *          - New connections added via try_accept()
     *          - Data read/written in read_data()/write_data()
     *          - Failed sockets removed and marked as -1
     * 
     * @warning Array size limited by WIFI_MAX_CONNECTION (default 5).
     *          Additional connection attempts will be rejected when full.
     */
    short socket_list[WIFI_MAX_CONNECTION];
    
    /**
     * @brief FreeRTOS task handle for WiFi network thread
     * 
     * @details Handle to the FreeRTOS task running _wifi_thread static method.
     *          Task handles all network I/O operations (accept, read, write)
     *          to avoid blocking the main application task.
     */
    tskTaskControlBlock* _wifi_task_handle;
    
    /**
     * @brief Initialize WiFi hardware and network stack
     * 
     * @details Performs complete WiFi initialization sequence:
     *          1. Initialize NVS (Non-Volatile Storage) partition for WiFi
     *             configuration persistence
     *          2. Initialize TCP/IP adapter (ESP-NETIF)
     *          3. Create default WiFi station or AP network interface
     *          4. Initialize WiFi driver with default configuration
     *          5. Register event handlers for WiFi state changes
     *          6. Start WiFi in configured mode (SoftAP or Station)
     *          7. Configure TCP socket parameters
     *          
     *          Configuration is typically stored in NVS and includes:
     *          - SSID and password (for Station mode)
     *          - AP configuration (for SoftAP mode)
     *          - IP address settings (static or DHCP)
     *          
     * @note Called automatically from _begin() on first initialization
     * @note Transitions state from NOT_INITIALIZED to INITIALIZED on success
     */
    void initialize_wifi();
    
    /**
     * @brief Read available data from connected TCP clients
     * 
     * @return true if data was read successfully, false on errors
     * 
     * @details Iterates through socket_list array, calling recv() on each
     *          active socket to read available data into _readbuf. Handles
     *          socket errors and removes disconnected clients from socket_list.
     *          
     * @note Called from _wifi_thread network task
     * @note Non-blocking socket operations to avoid stalling network thread
     */
    bool read_data();
    
    /**
     * @brief Write buffered data to connected TCP clients
     * 
     * @return true if data was written successfully, false on errors
     * 
     * @details Reads data from _writebuf and sends to all connected clients
     *          in socket_list using send(). Handles partial writes and socket
     *          errors, removing failed sockets from active list.
     *          
     * @note Called from _wifi_thread network task
     * @note Uses non-blocking sockets to prevent network thread stalls
     */
    bool write_data();
    
    /**
     * @brief Start TCP server listening for client connections
     * 
     * @return true if listening started successfully, false on errors
     * 
     * @details Creates TCP server socket, binds to port 5760, and begins
     *          listening for incoming connections. Socket is stored in
     *          accept_socket for use by try_accept().
     *          
     * @note Called after successful initialize_wifi()
     * @note Default port 5760 is standard MAVLink telemetry port
     */
    bool start_listen();
    
    /**
     * @brief Accept pending TCP client connection
     * 
     * @return true if connection accepted, false if no pending connections
     * 
     * @details Checks accept_socket for pending connections and accepts the
     *          next client. New socket is added to first available slot in
     *          socket_list array. Transitions state to CONNECTED on first
     *          successful connection.
     *          
     * @note Called periodically from _wifi_thread
     * @note Rejects connections when socket_list is full (WIFI_MAX_CONNECTION)
     */
    bool try_accept();
    
    /**
     * @brief Static FreeRTOS task entry point for WiFi network operations
     * 
     * @param[in] arg Pointer to WiFiDriver instance (this pointer)
     * 
     * @details Network I/O task that runs continuously to handle:
     *          - Accepting new client connections (try_accept)
     *          - Reading data from clients (read_data)
     *          - Writing buffered data to clients (write_data)
     *          
     *          Task characteristics:
     *          - Runs as separate FreeRTOS task for non-blocking I/O
     *          - May be pinned to specific CPU core for performance
     *          - Yields periodically to prevent starvation of other tasks
     *          - Handles all socket operations asynchronously
     *          
     * @note FreeRTOS task pinning configuration depends on ESP32 core
     *       allocation strategy (APP_CPU vs PRO_CPU)
     * @note Task priority and stack size configured at task creation
     */
    static void _wifi_thread(void* arg);
    
    /**
     * @brief Find first available socket slot in socket_list
     * 
     * @return Index of first unused socket slot, or WIFI_MAX_CONNECTION if full
     * 
     * @details Searches socket_list array for first slot containing -1
     *          (indicating unused/closed socket). Used by try_accept() to
     *          find where to store newly accepted client connection.
     */
    unsigned short available_socket();

protected:
    /**
     * @brief Begin WiFi UART driver operation
     * 
     * @param[in] b Baud rate (ignored for WiFi - TCP has no baud rate concept)
     * @param[in] rxS Receive buffer size in bytes (used to allocate _readbuf)
     * @param[in] txS Transmit buffer size in bytes (used to allocate _writebuf)
     * 
     * @details Initializes WiFi driver and starts network operations:
     *          1. Allocates ByteBuffer for RX and TX (using rxS, txS parameters)
     *          2. Calls initialize_wifi() to configure ESP32 WiFi stack
     *          3. Calls start_listen() to begin accepting connections
     *          4. Creates FreeRTOS task for network I/O (_wifi_thread)
     *          
     * @note Baud rate parameter ignored as TCP connections are not serial
     * @note Buffer sizes default to TX_BUF_SIZE and RX_BUF_SIZE if parameters
     *       are zero or too small
     */
    void _begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    
    /**
     * @brief Write data to transmit buffer for sending to TCP clients
     * 
     * @param[in] buffer Pointer to data to write
     * @param[in] size Number of bytes to write from buffer
     * 
     * @return Number of bytes actually written to transmit buffer
     * 
     * @details Copies data from application buffer into _writebuf (ByteBuffer)
     *          for asynchronous transmission by _wifi_thread. Write operation
     *          is protected by _write_mutex semaphore for thread safety.
     *          
     *          Data is broadcast to all connected TCP clients in socket_list.
     *          
     * @note May write fewer bytes than requested if buffer is full
     * @note Non-blocking operation - returns immediately even if buffer full
     */
    size_t _write(const uint8_t *buffer, size_t size) override;
    
    /**
     * @brief Read received data from TCP clients
     * 
     * @param[out] buffer Pointer to buffer where data will be stored
     * @param[in] size Maximum number of bytes to read into buffer
     * 
     * @return Number of bytes actually read, or -1 on error
     * 
     * @details Reads available data from _readbuf (ByteBuffer) into application
     *          buffer. Data originates from all connected TCP clients and is
     *          buffered by _wifi_thread calling read_data().
     *          
     * @note Returns 0 if no data available (non-blocking)
     * @note Thread-safe due to ByteBuffer internal synchronization
     */
    ssize_t _read(uint8_t *buffer, uint16_t size) override;
    
    /**
     * @brief End WiFi UART driver operation and close connections
     * 
     * @details Shuts down WiFi driver:
     *          1. Closes all active client sockets in socket_list
     *          2. Closes accept_socket (server listening socket)
     *          3. Terminates _wifi_thread FreeRTOS task
     *          4. Deinitializes WiFi hardware (optional - may keep running)
     *          5. Frees allocated ByteBuffer resources
     *          
     * @note May not fully deinitialize WiFi if other components are using it
     */
    void _end() override;
    
    /**
     * @brief Flush transmit buffer, ensuring all data sent to clients
     * 
     * @details Blocks until _writebuf is empty and all buffered data has been
     *          transmitted to connected TCP clients via write_data().
     *          
     * @note Blocking operation - waits for network transmission to complete
     * @note Timeout may apply to prevent indefinite blocking on stalled sockets
     */
    void _flush() override;
    
    /**
     * @brief Discard all pending received data in read buffer
     * 
     * @return true if input discarded successfully, false on error
     * 
     * @details Clears _readbuf (ByteBuffer), discarding all received data that
     *          has not yet been read by the application. Useful for resetting
     *          communication state or skipping stale data.
     *          
     * @note Does not affect data currently in transit on TCP sockets
     */
    bool _discard_input() override;
    
    /**
     * @brief Get number of bytes available for reading
     * 
     * @return Number of bytes in receive buffer ready to be read
     * 
     * @details Returns the number of bytes currently stored in _readbuf that
     *          can be read via _read() without blocking.
     *          
     * @note Value may increase asynchronously as _wifi_thread receives more data
     */
    uint32_t _available() override;
};
