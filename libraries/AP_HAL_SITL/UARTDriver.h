/**
 * @file UARTDriver.h
 * @brief SITL UART/serial port simulation driver
 * 
 * @details This file implements serial port emulation for Software-In-The-Loop (SITL)
 *          simulation, providing virtual UART interfaces that can be connected to various
 *          transports including TCP, UDP, multicast, UNIX sockets, and real hardware serial
 *          devices. This enables MAVLink telemetry, GPS simulation, and other serial protocol
 *          testing without physical hardware.
 * 
 *          The driver supports multiple connection types specified via path strings:
 *          - tcp:port[:wait|nowait] - TCP server listening on port
 *          - tcpclient:host:port - TCP client connecting to remote host
 *          - udp:host:port - UDP connection for GCS/MAVProxy
 *          - mcast:address:port - Multicast UDP for multi-GCS simulation
 *          - uart:device:baudrate - Real hardware serial device passthrough
 *          - sim:protocol - Built-in protocol simulators (GPS, ADSB, etc.)
 *          - logic_async_csv:file - Logic analyzer CSV playback
 * 
 *          Baud rate emulation is enforced through DataRateLimit to accurately simulate
 *          serial bandwidth constraints and timing behavior.
 * 
 * @note Timing behavior approximates but may not exactly match real serial hardware due
 *       to simulation environment differences and host system scheduling.
 * 
 * @warning Connection paths and baud rates should match expected protocol requirements
 *          to avoid data loss or timing issues in simulated scenarios.
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <stdint.h>
#include <stdarg.h>
#include "AP_HAL_SITL_Namespace.h"
#include <AP_HAL/utility/Socket_native.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_CSVReader/AP_CSVReader.h>
#include <AP_HAL/utility/DataRateLimit.h>

#include <SITL/SIM_SerialDevice.h>

/**
 * @class HALSITL::UARTDriver
 * @brief Software-In-The-Loop UART driver implementation
 * 
 * @details Provides virtual serial port functionality for SITL simulation, enabling
 *          connections to various transports (TCP, UDP, multicast, real serial devices)
 *          for testing MAVLink telemetry, GPS protocols, and other serial communication
 *          without physical hardware.
 * 
 *          Architecture:
 *          - Non-blocking ring buffers for read/write operations
 *          - DataRateLimit enforces baud rate emulation for realistic timing
 *          - Supports multiple transport types via path string configuration
 *          - Integrates with SITL_State for simulation orchestration
 *          - Optional byte loss simulation for robustness testing
 *          - Statistics collection when HAL_UART_STATS_ENABLED
 * 
 *          Connection Types:
 *          - TCP Server: Listen for GCS connections (tcp:port:wait/nowait)
 *          - TCP Client: Connect to remote endpoint (tcpclient:host:port)
 *          - UDP: Connectionless transport (udp:host:port)
 *          - Multicast: Multiple GCS support (mcast:address:port)
 *          - Hardware Serial: Real UART device passthrough (uart:device:baudrate)
 *          - Simulator: Built-in device simulators (sim:gps, sim:adsb, etc.)
 *          - CSV Replay: Logic analyzer data playback (logic_async_csv:file)
 * 
 *          Baud Rate Emulation:
 *          Data rates are limited to configured baud rate using DataRateLimit,
 *          simulating realistic serial bandwidth constraints and introducing
 *          appropriate timing delays for protocol testing.
 * 
 *          Thread Safety:
 *          Write operations protected by write_mtx semaphore for multi-threaded access.
 * 
 * @note Timing approximates real hardware but may vary due to simulation scheduling.
 * 
 * @warning TCP server mode with "wait" blocks until client connects - use "nowait" for
 *          non-blocking startup. UDP mode does not guarantee packet delivery.
 * 
 * @see AP_HAL::UARTDriver for base interface contract
 * @see SITL_State for simulation state management
 */
class HALSITL::UARTDriver : public AP_HAL::UARTDriver {
public:
    friend class HALSITL::SITL_State;

    /**
     * @brief Construct SITL UART driver instance
     * 
     * @param[in] portNumber  Serial port number (0-based index)
     * @param[in] sitlState   Pointer to SITL simulation state manager
     */
    UARTDriver(const uint8_t portNumber, SITL_State *sitlState) {
        _portNumber = portNumber;
        _sitlState = sitlState;

        _fd = -1;
        _mc_fd = -1;
        _listen_fd = -1;
    }

    /**
     * @brief Check if UART is initialized
     * 
     * @return true - SITL UARTs are always considered initialized
     * 
     * @note SITL drivers are initialized on construction, no separate init phase required
     */
    bool is_initialized() override {
        return true;
    }

    /**
     * @brief Get system-level transmit queue length for flow control
     * 
     * @details Queries the underlying transport (socket, device) for buffered data
     *          awaiting transmission at the OS level, useful for flow control decisions.
     * 
     * @return Number of bytes queued in system buffers, or -1 if unavailable
     * 
     * @note Returns -1 for transports without system queue visibility (e.g., UDP)
     */
    ssize_t get_system_outqueue_length() const;

    /**
     * @brief Check if transmit operations are pending
     * 
     * @return false - SITL uses non-blocking writes with ring buffers
     * 
     * @note Unlike hardware UARTs with DMA, SITL immediately buffers all writes
     */
    bool tx_pending() override {
        return false;
    }

    /**
     * @brief Get available transmit buffer space
     * 
     * @details Returns number of bytes that can be written to the UART without blocking,
     *          accounting for ring buffer capacity and baud rate limiting.
     * 
     * @return Number of bytes available in transmit buffer
     * 
     * @note Space may be limited by DataRateLimit even if buffer has capacity
     */
    uint32_t txspace() override;

    /**
     * @brief Unbuffered write mode flag (public for SITL_State access)
     * 
     * When true, writes bypass ring buffer and go directly to transport
     */
    bool _unbuffered_writes;

    /**
     * @brief Get flow control configuration
     * 
     * @return FLOW_CONTROL_ENABLE - SITL always reports flow control enabled
     * 
     * @note Flow control in SITL is simulated through baud rate limiting rather
     *       than hardware RTS/CTS pins
     */
    enum flow_control get_flow_control(void) override { return FLOW_CONTROL_ENABLE; }

    /**
     * @brief Configure serial parity setting
     * 
     * @param[in] v  Parity configuration value (0=none, 1=odd, 2=even)
     * 
     * @note SITL stores this setting but does not enforce parity on simulated transport
     */
    void configure_parity(uint8_t v) override;

    /**
     * @brief Set number of stop bits
     * 
     * @param[in] n  Number of stop bits (1 or 2)
     * 
     * @note SITL stores this setting but does not enforce stop bits on simulated transport
     */
    void set_stop_bits(int n) override;

    /**
     * @brief Enable or disable unbuffered write mode
     * 
     * @details When enabled, write operations bypass the ring buffer and write directly
     *          to the underlying transport. This can reduce latency but may cause blocking
     *          if the transport cannot accept data immediately.
     * 
     * @param[in] on  true to enable unbuffered writes, false for buffered mode
     * 
     * @return true if mode was successfully set
     * 
     * @warning Unbuffered mode may block if transport buffer is full, potentially
     *          affecting real-time performance in simulation
     */
    bool set_unbuffered_writes(bool on) override;

    /**
     * @brief Get current bandwidth utilization in bytes per second
     * 
     * @details Calculates actual data rate based on recent transmission activity,
     *          useful for monitoring protocol efficiency and detecting bottlenecks.
     * 
     * @return Current throughput in bytes per second
     * 
     * @note Returns smoothed average over recent time window, not instantaneous rate
     */
    uint32_t bw_in_bytes_per_second() const override;

    /**
     * @brief Periodic timer callback for background processing
     * 
     * @details Called regularly by scheduler to handle data transfer between ring buffers
     *          and underlying transport, enforce baud rate limits, and update statistics.
     *          Processes both reading from device to read buffer and writing from write
     *          buffer to device.
     * 
     * @note Called at scheduler timer rate (typically 1kHz), not related to baud rate
     */
    void _timer_tick(void) override;

    /**
     * @brief Estimate timestamp when packet reception started
     * 
     * @details Returns timestamp constraint in microseconds for when the start of a packet
     *          arrived on the UART. This provides timing information for protocol analysis
     *          and time-sensitive operations. The estimate accounts for baud rate and
     *          transmission time for the specified number of bytes.
     * 
     *          This is a time constraint, not an exact timestamp:
     *          - Packet reception did NOT start after this time (upper bound)
     *          - Packet MAY have started earlier (could have been buffered)
     *          - Accounts for serial transmission time at configured baud rate
     * 
     *          Calculation includes:
     *          - Baud rate transmission delay for nbytes
     *          - System buffering delays where measurable
     *          - Transport-specific timing characteristics
     * 
     * @param[in] nbytes  Number of bytes in the packet
     * 
     * @return Timestamp in microseconds when packet reception likely started, or 0 if
     *         timing information is unavailable for this transport
     * 
     * @note For transports without inherent baud rate (UDP, TCP), estimate may be less
     *       accurate as transmission time is not well-defined
     * 
     * @note Useful for analyzing protocol timing requirements and detecting delays in
     *       time-critical communication paths
     * 
     * @see SITL::SIM_SerialDevice for built-in device simulator timing
     */
    uint64_t receive_time_constraint_us(uint16_t nbytes) override;

    /**
     * @brief Get configured baud rate
     * 
     * @return Baud rate in bits per second
     * 
     * @note SITL enforces this baud rate through DataRateLimit for realistic timing
     */
    uint32_t get_baud_rate() const override { return _uart_baudrate; }

#if HAL_UART_STATS_ENABLED
    /**
     * @brief Provide UART I/O statistics for diagnostics
     * 
     * @details Collects and reports statistics about UART activity including bytes
     *          transmitted, bytes received, bandwidth utilization, and timing information.
     *          Used for performance analysis and debugging communication issues.
     * 
     * @param[out] str     ExpandingString to append formatted statistics
     * @param[out] stats   StatsTracker for aggregating statistics across ports
     * @param[in]  dt_ms   Time delta in milliseconds for rate calculations
     * 
     * @note Only available when HAL_UART_STATS_ENABLED is defined
     * @see StatsTracker for statistics collection infrastructure
     */
    void uart_info(ExpandingString &str, StatsTracker &stats, const uint32_t dt_ms) override;
#endif

private:

    // File descriptor for primary connection (socket or device)
    int _fd;

    // File descriptor for reading multicast packets (separate from primary _fd)
    int _mc_fd;

    // Serial port number (0-based index: 0=SERIAL0, 1=SERIAL1, etc.)
    uint8_t _portNumber;

    // Connection state flag - true when TCP client has connected to server
    bool _connected = false;

    // Use send()/recv() instead of write()/read() for socket operations
    bool _use_send_recv = false;

    // Listening socket file descriptor for TCP server mode
    int _listen_fd;

    // Serial port number for real hardware UART passthrough
    int _serial_port;

    // Console output flag - first UART can output to console
    static bool _console;

    // Non-blocking ring buffer for received data (16KB capacity)
    ByteBuffer _readbuffer{16384};

    // Non-blocking ring buffer for transmit data (16KB capacity)
    ByteBuffer _writebuffer{16384};

    // Default multicast IP address for multi-GCS simulation (239.255.145.50)
    const char *mcast_ip_default = "239.255.145.50";

    // Default multicast port matching MAVLink/GCS convention (14550)
    const uint16_t mcast_port_default = 14550;

    // Connection path string (e.g., "tcp:5760:wait", "udp:127.0.0.1:14550")
    const char *_uart_path;

    // Configured baud rate in bits per second (enforced by DataRateLimit)
    uint32_t _uart_baudrate;

    /**
     * @brief Start TCP server listening on specified port
     * 
     * @details Creates TCP server socket and optionally waits for first client connection.
     *          Used for GCS connections where autopilot acts as server.
     * 
     * @param[in] port                  TCP port number to listen on
     * @param[in] wait_for_connection   true=block until client connects, false=non-blocking
     * 
     * @note Path format: "tcp:5760:wait" or "tcp:5760:nowait"
     */
    void _tcp_start_connection(uint16_t port, bool wait_for_connection);

    /**
     * @brief Start connection to real hardware UART device
     * 
     * @details Opens hardware serial device for passthrough testing, enabling SITL to
     *          communicate with actual hardware peripherals (GPS, telemetry radios, etc.).
     * 
     * @note Path format: "uart:/dev/ttyUSB0:57600"
     */
    void _uart_start_connection(void);

    /**
     * @brief Check for and handle reconnection after disconnect
     * 
     * @details Monitors connection state and attempts to re-establish TCP connections
     *          if client disconnects, enabling persistent simulation across GCS restarts.
     */
    void _check_reconnect();

    /**
     * @brief Start TCP client connection to remote host
     * 
     * @details Connects as TCP client to remote server, useful for forwarding data to
     *          external processes or connecting multiple SITL instances.
     * 
     * @param[in] address  Remote host address (IP or hostname)
     * @param[in] port     Remote TCP port number
     * 
     * @note Path format: "tcpclient:192.168.1.100:14550"
     */
    void _tcp_start_client(const char *address, uint16_t port);

    /**
     * @brief Start UDP client connection
     * 
     * @details Creates UDP socket for connectionless communication, commonly used with
     *          MAVProxy and ground control stations.
     * 
     * @param[in] address  Remote host address for UDP packets
     * @param[in] port     Remote UDP port number
     * 
     * @note Path format: "udp:127.0.0.1:14550"
     * @warning UDP provides no delivery guarantees - packets may be lost
     */
    void _udp_start_client(const char *address, uint16_t port);

    /**
     * @brief Start UDP multicast connection for multi-GCS support
     * 
     * @details Joins multicast group enabling multiple ground stations to simultaneously
     *          receive telemetry without explicit per-client configuration.
     * 
     * @param[in] address  Multicast group address (e.g., "239.255.145.50")
     * @param[in] port     Multicast port number (typically 14550)
     * 
     * @note Path format: "mcast:239.255.145.50:14550"
     * @see mcast_ip_default, mcast_port_default for standard values
     */
    void _udp_start_multicast(const char *address, uint16_t port);

    /**
     * @brief Check connection status and accept new TCP clients
     * 
     * @details Polls for new TCP client connections in server mode and updates connection
     *          state flags. Called periodically during timer tick.
     */
    void _check_connection(void);

    /**
     * @brief Check if file descriptor has data available using select()
     * 
     * @param[in] fd  File descriptor to check
     * @return true if data is available for reading
     * 
     * @note Non-blocking check with zero timeout
     */
    static bool _select_check(int fd);

    /**
     * @brief Set file descriptor to non-blocking mode
     * 
     * @param[in] fd  File descriptor to configure
     * 
     * @note Required for ring buffer operation without blocking on read/write
     */
    static void _set_nonblocking(int fd);

    /**
     * @brief Configure baud rate for hardware UART device
     * 
     * @param[in] speed  Baud rate in bits per second
     * @return true if baud rate was successfully set
     * 
     * @note Only applicable to real hardware UART connections (uart: paths)
     */
    bool set_speed(int speed) const;

    // Pointer to SITL simulation state manager
    SITL_State *_sitlState;

    // Timestamp of most recent data reception in microseconds
    uint64_t _receive_timestamp;

    // Flag indicating UDP transport (affects packetization behavior)
    bool _is_udp;

    // Enable packet-oriented behavior for UDP (vs stream-oriented for TCP)
    bool _packetise;

    // Local UDP port for multicast reception
    uint16_t _mc_myport;

    /**
     * @brief Baud rate limiting for realistic serial timing
     * 
     * @details Enforces configured baud rate on both transmit and receive paths,
     *          simulating bandwidth constraints and transmission delays of real serial
     *          hardware. Essential for accurate protocol timing analysis.
     */
    struct {
        DataRateLimit write;  ///< Transmit rate limiter
        DataRateLimit read;   ///< Receive rate limiter
    } baud_limits;

    // Semaphore protecting write operations for thread-safe access
    HAL_Semaphore write_mtx;

    // Pointer to built-in serial device simulator (GPS, ADSB, etc.)
    SITL::SerialDevice *_sim_serial_device;

    /**
     * @brief Logic analyzer CSV data replay state
     * 
     * @details Supports playback of logic analyzer captures in CSV format, enabling
     *          hardware-recorded protocol traces to be replayed into SITL for testing
     *          protocol parsers and timing-sensitive code.
     * 
     *          CSV format: timestamp_us,byte_value
     *          Example: 1000,0x55
     *                   1020,0xAA
     * 
     *          Timing is preserved relative to first timestamp, allowing accurate
     *          reproduction of captured serial data patterns.
     * 
     * @note Path format: "logic_async_csv:/path/to/capture.csv"
     */
    struct {
        bool active;                                   ///< CSV replay mode is active
        uint8_t term[20];                              ///< CSV term buffer
        AP_CSVReader csvreader{term, sizeof(term), ','}; ///< CSV parser instance
        struct {
            uint32_t timestamp_us;                     ///< Timestamp from CSV in microseconds
            uint8_t b;                                 ///< Byte value to emit
        } loaded_data;
        bool loaded;                                   ///< Valid data loaded from CSV
        bool done_first_line = false;                  ///< First line processed flag
        uint8_t terms_seen;                            ///< Number of CSV terms parsed
        uint32_t first_timestamp_us;                   ///< First timestamp for relative timing
        uint32_t first_emit_micros_us;                 ///< Real time when replay started
    } logic_async_csv;

    /**
     * @brief Read data from logic analyzer CSV replay
     * 
     * @details Reads bytes from CSV file according to recorded timestamps, preserving
     *          original timing relationships for accurate protocol replay.
     * 
     * @param[out] buffer  Buffer to write data into
     * @param[in]  space   Available space in buffer
     * 
     * @return Number of bytes written to buffer (may be 0 if timing not yet reached)
     */
    uint16_t read_from_async_csv(uint8_t *buffer, uint16_t space);

protected:
    /**
     * @brief Initialize UART with specified parameters
     * 
     * @details Parses connection path string, creates appropriate transport (TCP, UDP,
     *          serial device, etc.), configures baud rate limiting, and initializes
     *          ring buffers. Called by public begin() method.
     * 
     * @param[in] b    Baud rate in bits per second
     * @param[in] rxS  Receive buffer size (override default 16KB)
     * @param[in] txS  Transmit buffer size (override default 16KB)
     * 
     * @note Connection path determined by SITL parameters (e.g., SERIAL0_PROTOCOL)
     */
    void _begin(uint32_t b, uint16_t rxS, uint16_t txS) override;

    /**
     * @brief Write data to UART transmit buffer
     * 
     * @details Writes data to ring buffer or directly to transport if unbuffered mode
     *          is enabled. Respects baud rate limits when DataRateLimit is active.
     * 
     * @param[in] buffer  Data to transmit
     * @param[in] size    Number of bytes to write
     * 
     * @return Number of bytes actually written (may be less than size if buffer full)
     * 
     * @note Protected by write_mtx semaphore for thread safety
     */
    size_t _write(const uint8_t *buffer, size_t size) override;

    /**
     * @brief Read data from UART receive buffer
     * 
     * @details Reads available data from ring buffer, up to specified count. Data has
     *          already been transferred from transport to ring buffer by _timer_tick().
     * 
     * @param[out] buffer  Buffer to read data into
     * @param[in]  count   Maximum number of bytes to read
     * 
     * @return Number of bytes actually read, or -1 on error
     */
    ssize_t _read(uint8_t *buffer, uint16_t count) override;

    /**
     * @brief Get number of bytes available for reading
     * 
     * @return Number of bytes in receive ring buffer ready to read
     */
    uint32_t _available() override;

    /**
     * @brief Close UART connection
     * 
     * @details Closes underlying transport file descriptors and frees resources.
     */
    void _end() override;

    /**
     * @brief Flush transmit buffer
     * 
     * @details Attempts to push all buffered transmit data to the underlying transport.
     *          May not fully flush if transport cannot accept all data immediately.
     */
    void _flush() override;

    /**
     * @brief Discard all pending receive data
     * 
     * @details Clears receive ring buffer, discarding all unread data. Useful for
     *          resynchronizing protocol parsers after errors.
     * 
     * @return true if input was successfully discarded
     */
    bool _discard_input() override;

#if HAL_UART_STATS_ENABLED
    /**
     * @brief Get cumulative transmitted byte count
     * 
     * @return Total number of bytes transmitted since initialization
     * 
     * @note Only available when HAL_UART_STATS_ENABLED is defined
     */
    uint32_t get_total_tx_bytes() const override { return _tx_stats_bytes; }

    /**
     * @brief Get cumulative received byte count
     * 
     * @return Total number of bytes received since initialization
     * 
     * @note Only available when HAL_UART_STATS_ENABLED is defined
     */
    uint32_t get_total_rx_bytes() const override { return _rx_stats_bytes; }
#endif

private:
    /**
     * @brief Transfer data from write buffer to underlying transport
     * 
     * @details Called by _timer_tick() to move data from ring buffer to socket/device,
     *          respecting baud rate limits and transport readiness.
     */
    void handle_writing_from_writebuffer_to_device();

    /**
     * @brief Transfer data from underlying transport to read buffer
     * 
     * @details Called by _timer_tick() to read data from socket/device into ring buffer,
     *          respecting baud rate limits and buffer space availability.
     */
    void handle_reading_from_device_to_readbuffer();

    // Cumulative transmit byte counter for statistics
    uint32_t _tx_stats_bytes;

    // Cumulative receive byte counter for statistics
    uint32_t _rx_stats_bytes;

};

#endif
