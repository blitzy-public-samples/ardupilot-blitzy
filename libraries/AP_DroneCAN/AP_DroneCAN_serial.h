/**
 * @file AP_DroneCAN_serial.h
 * @brief DroneCAN serial tunneling implementation for ArduPilot
 * 
 * @details This file implements serial-over-CAN tunneling using the DroneCAN/UAVCAN
 *          protocol. It provides virtual serial ports that transparently tunnel serial
 *          data through the CAN bus to remote DroneCAN nodes, enabling features like:
 *          - Remote GPS modules connected via CAN
 *          - Telemetry links over CAN infrastructure
 *          - Debug consoles to CAN-attached devices
 *          - General-purpose serial communication over CAN
 * 
 *          The implementation uses the uavcan_tunnel_Targetted message for bidirectional
 *          serial data transfer, with each virtual port mapped to a specific target node
 *          and protocol ID. Data is buffered locally and transmitted in CAN frames,
 *          with flow control managed through buffer availability.
 * 
 * @note Maximum number of virtual serial ports is defined by AP_DRONECAN_SERIAL_NUM_PORTS
 * 
 * @warning This module requires proper semaphore locking for thread-safe operation
 *          between the main thread and CAN receive callbacks
 * 
 * @see AP_SerialManager for serial port registration and enumeration
 * @see AP_DroneCAN for the underlying DroneCAN protocol implementation
 * 
 * Source: libraries/AP_DroneCAN/AP_DroneCAN_serial.h
 */

#pragma once

#include <AP_SerialManager/AP_SerialManager.h>

#ifndef AP_DRONECAN_SERIAL_NUM_PORTS
#define AP_DRONECAN_SERIAL_NUM_PORTS 3
#endif

class AP_DroneCAN;

/**
 * @class AP_DroneCAN_Serial
 * @brief Serial-over-CAN tunneling manager for DroneCAN networks
 * 
 * @details This class manages virtual serial ports that tunnel serial data through
 *          DroneCAN (UAVCAN) CAN bus networks. It provides up to AP_DRONECAN_SERIAL_NUM_PORTS
 *          virtual serial ports that can be assigned to communicate with remote DroneCAN
 *          nodes using the uavcan_tunnel_Targetted message protocol.
 * 
 *          Each virtual port acts as a standard ArduPilot serial port registered with
 *          AP_SerialManager, allowing any serial protocol (GPS, telemetry, etc.) to
 *          transparently operate over the CAN bus without protocol-specific modifications.
 * 
 *          **Serial Protocol Mapping:**
 *          - SerialProtocol_GPS → UAVCAN protocol ID for GPS tunneling
 *          - SerialProtocol_MAVLink → UAVCAN protocol ID for MAVLink tunneling
 *          - SerialProtocol_Rangefinder → UAVCAN protocol ID for rangefinder tunneling
 *          - Other protocols mapped according to DroneCAN tunnel specification
 * 
 *          **Message Format:**
 *          The uavcan_tunnel_Targetted message contains:
 *          - target_node: Destination DroneCAN node ID (1-127)
 *          - protocol_id: Application protocol identifier (e.g., GPS, MAVLink)
 *          - buffer: Serial data payload (up to UAVCAN max frame size)
 *          - baudrate: Optional baud rate hint for remote serial port configuration
 * 
 *          **Routing:**
 *          - Outbound: Local serial writes → uavcan_tunnel_Targetted → CAN bus → remote node
 *          - Inbound: Remote node → uavcan_tunnel_Targetted → CAN bus → local serial reads
 * 
 *          **Buffer Management:**
 *          Each port maintains separate RX and TX buffers sized according to begin() call.
 *          Default buffer sizes balance latency and memory usage for typical protocols.
 * 
 *          **Performance Considerations:**
 *          - CAN bus bandwidth limits effective serial throughput (typically ~100kbps usable)
 *          - Latency increased by CAN frame transmission time and node processing
 *          - Multiple ports share CAN bandwidth, affecting total throughput
 *          - Buffer sizes should accommodate protocol burst patterns
 * 
 * @note Buffer sizes are configurable per port via begin() method parameters
 * @note Maximum of AP_DRONECAN_SERIAL_NUM_PORTS virtual ports supported (default 3)
 * @note Thread-safe operation requires semaphore protection (handled internally by Port class)
 * 
 * @warning Enabling too many ports or oversized buffers can exhaust available RAM
 * @warning CAN bus errors or node failures will stall serial communication on affected ports
 * @warning Bandwidth-intensive protocols (video, high-rate telemetry) may saturate CAN bus
 * 
 * @see Port class for individual virtual port implementation
 * @see uavcan_tunnel_Targetted message definition in DroneCAN specification
 */
class AP_DroneCAN_Serial
{
public:
    /* Do not allow copies */
    CLASS_NO_COPY(AP_DroneCAN_Serial);

    /**
     * @brief Default constructor for AP_DroneCAN_Serial
     * 
     * @details Initializes the serial tunneling manager with default state.
     *          Actual initialization occurs in init() after DroneCAN driver is ready.
     */
    AP_DroneCAN_Serial() {}

    /**
     * @brief Enable/disable parameter for serial tunneling
     * 
     * @details When non-zero, serial tunneling is enabled and ports can be configured.
     *          When zero, all tunneling is disabled to save resources.
     *          This parameter is typically set via ground station or parameter file.
     */
    AP_Int8 enable;

    /**
     * @brief Initialize serial tunneling with DroneCAN driver
     * 
     * @details Sets up the serial tunneling subsystem, registers message handlers,
     *          and initializes virtual serial ports. Must be called after DroneCAN
     *          driver initialization but before serial ports are used.
     * 
     *          This method:
     *          - Stores reference to parent DroneCAN driver
     *          - Registers uavcan_tunnel_Targetted message handler
     *          - Initializes publisher for outbound tunnel messages
     *          - Configures each enabled virtual port
     * 
     * @param[in] dronecan Pointer to parent AP_DroneCAN driver instance
     * 
     * @note Called once during system startup by AP_DroneCAN
     * @warning Must be called before any port operations
     */
    void init(AP_DroneCAN *dronecan);

    /**
     * @brief Periodic update for serial tunneling operations
     * 
     * @details Called regularly (typically at scheduler rate) to process pending
     *          serial data transfers. This method:
     *          - Checks each port for pending TX data and sends via CAN
     *          - Updates port statistics and timing information
     *          - Manages flow control and buffer states
     *          - Handles timeout conditions for inactive ports
     * 
     * @note Called by AP_DroneCAN::update() at main loop rate
     * @note Execution time depends on amount of pending serial data
     */
    void update(void);

public:
    /**
     * @class Port
     * @brief Virtual serial port implementation for DroneCAN serial tunneling
     * 
     * @details Implements a virtual serial port that tunnels data through DroneCAN CAN bus.
     *          This class inherits from AP_SerialManager::RegisteredPort, making it a
     *          standard ArduPilot serial port that can be assigned to any serial protocol
     *          (GPS, telemetry, rangefinder, etc.) through normal serial port configuration.
     * 
     *          **Implementation Details:**
     *          - Maintains separate RX and TX ByteBuffers for data queuing
     *          - Uses HAL_Semaphore for thread-safe access between main loop and CAN callbacks
     *          - Tracks statistics (bytes sent/received, drops) for monitoring
     *          - Implements flow control through buffer space reporting
     *          - Supports configurable buffer sizes via begin() method
     * 
     *          **Virtual Serial Port Interface:**
     *          Implements all required methods from AP_SerialManager::RegisteredPort:
     *          - begin(): Configure port (baud rate, buffer sizes)
     *          - write(): Send data over CAN (buffered, non-blocking)
     *          - read(): Receive data from CAN (buffered)
     *          - available(): Check RX buffer occupancy
     *          - txspace(): Check TX buffer free space
     *          - flush(), end(): Standard serial port lifecycle methods
     * 
     *          **Data Flow:**
     *          TX (outbound): write() → TX buffer → update() → uavcan_tunnel_Targetted → CAN bus
     *          RX (inbound): CAN bus → handle_tunnel_targetted() → RX buffer → read()
     * 
     *          **Configuration:**
     *          Each port is configured via parameters:
     *          - node: Target DroneCAN node ID (1-127) to communicate with
     *          - idx: Index/instance for multiple ports to same node (protocol discrimination)
     * 
     *          **Buffer Sizing:**
     *          Default buffer sizes vary by protocol:
     *          - GPS: Moderate buffers for NMEA/UBX sentences (e.g., 512/512 bytes)
     *          - Telemetry: Larger buffers for MAVLink message bursts (e.g., 1024/2048 bytes)
     *          - Debug: Small buffers for console output (e.g., 256/256 bytes)
     * 
     *          **Thread Safety:**
     *          All public methods acquire semaphore before buffer access to prevent
     *          race conditions between main thread operations and CAN receive callbacks.
     * 
     * @note Buffer allocation occurs in begin() and may fail if insufficient RAM
     * @note Statistics counters are cumulative and only reset on port re-initialization
     * @note Port remains registered with SerialManager even when not actively used
     * 
     * @warning Must call init() before using port for first time
     * @warning Large buffer sizes reduce available RAM for other subsystems
     * @warning Dropped bytes (rx_stats_dropped_bytes) indicate RX buffer overrun
     * 
     * @see AP_SerialManager::RegisteredPort for base serial port interface
     * @see ByteBuffer for circular buffer implementation
     */
    class Port : public AP_SerialManager::RegisteredPort {
    public:
        friend class AP_DroneCAN_Serial;

        /**
         * @brief Initialize the virtual serial port
         * 
         * @details Performs port-specific initialization after construction.
         *          Sets up initial state and registers the port with SerialManager
         *          for protocol assignment and discovery.
         * 
         * @note Called by AP_DroneCAN_Serial::init() for each configured port
         */
        void init(void);

        /**
         * @brief Target DroneCAN node ID parameter
         * 
         * @details Specifies which DroneCAN node this port communicates with.
         *          Valid range: 1-127 (0 disables port, 128+ reserved by UAVCAN).
         *          Set via parameter: e.g., CAN_D1_UC_SER1_NODE for first port on first driver.
         */
        AP_Int8 node;

        /**
         * @brief Port index/instance parameter
         * 
         * @details Discriminates multiple virtual ports to the same node.
         *          Allows routing different protocols to same physical node.
         *          Valid range: 0-255, mapped to protocol field in tunnel messages.
         */
        AP_Int8 idx;

    private:
        /**
         * @brief Check if port is initialized
         * 
         * @details Override from RegisteredPort interface. Returns true since
         *          DroneCAN serial ports are considered always initialized once
         *          construction completes (actual comm established dynamically).
         * 
         * @return true Always returns true for virtual CAN ports
         */
        bool is_initialized() override {
            return true;
        }

        /**
         * @brief Check if transmit data is pending
         * 
         * @details Override from RegisteredPort interface. Returns false since
         *          pending TX data is managed through txspace() reporting rather
         *          than explicit pending flag for CAN tunneling implementation.
         * 
         * @return false Always returns false (use txspace() for flow control)
         */
        bool tx_pending() override {
            return false;
        }

        /**
         * @brief Initialize RX and TX buffers
         * 
         * @details Allocates ByteBuffer instances for receive and transmit queuing.
         *          Buffers are sized according to begin() parameters. Allocation
         *          may fail on RAM-constrained systems with large buffer requests.
         * 
         * @param[in] size_rx Receive buffer size in bytes
         * @param[in] size_tx Transmit buffer size in bytes
         * 
         * @return true if both buffers allocated successfully, false on allocation failure
         * 
         * @note Called by _begin() method during port configuration
         * @warning Allocation failure leaves port non-functional
         */
        bool init_buffers(const uint32_t size_rx, const uint32_t size_tx);

        /**
         * @brief Get available transmit buffer space
         * 
         * @details Returns free space in TX buffer for flow control. Protocol
         *          implementations check this before writing to avoid blocking.
         *          Protected by semaphore for thread-safe access to buffer state.
         * 
         * @return Number of bytes available in TX buffer for writing
         * 
         * @note Returns 0 if buffers not initialized
         */
        uint32_t txspace() override;

        /**
         * @brief Configure and start the virtual serial port
         * 
         * @details Implements serial port begin() operation. Configures baud rate
         *          (stored for potential remote node configuration), allocates
         *          buffers if needed, and prepares port for data transfer.
         * 
         * @param[in] b Baud rate (informational, CAN bandwidth is actual limit)
         * @param[in] rxS Receive buffer size in bytes (0 uses default)
         * @param[in] txS Transmit buffer size in bytes (0 uses default)
         * 
         * @note Baud rate stored but CAN bus speed determines actual throughput
         * @note Buffer sizes rounded up to power of 2 by ByteBuffer implementation
         */
        void _begin(uint32_t b, uint16_t rxS, uint16_t txS) override;

        /**
         * @brief Write data to virtual serial port
         * 
         * @details Writes data to TX buffer for transmission over CAN. Non-blocking:
         *          writes up to available buffer space and returns actual count written.
         *          Caller should check txspace() first or handle partial writes.
         *          Protected by semaphore for thread safety.
         * 
         * @param[in] buffer Pointer to data to write
         * @param[in] size Number of bytes to write from buffer
         * 
         * @return Number of bytes actually written (may be less than requested if buffer full)
         * 
         * @note Returns 0 if buffers not initialized or buffer full
         * @note Partial writes require caller to retry remaining data
         */
        size_t _write(const uint8_t *buffer, size_t size) override;

        /**
         * @brief Read data from virtual serial port
         * 
         * @details Reads available data from RX buffer populated by incoming CAN
         *          tunnel messages. Non-blocking: returns immediately with available
         *          data up to requested count. Protected by semaphore for thread safety.
         * 
         * @param[out] buffer Pointer to buffer to receive data
         * @param[in] count Maximum number of bytes to read
         * 
         * @return Number of bytes actually read (may be less than requested, -1 on error)
         * 
         * @note Returns 0 if no data available
         * @note Returns -1 if buffers not initialized
         */
        ssize_t _read(uint8_t *buffer, uint16_t count) override;

        /**
         * @brief Get number of bytes available to read
         * 
         * @details Returns occupancy of RX buffer (data received via CAN and
         *          awaiting read by protocol handler). Protected by semaphore
         *          for thread-safe access to buffer state.
         * 
         * @return Number of bytes available in RX buffer for reading
         * 
         * @note Returns 0 if buffers not initialized or buffer empty
         */
        uint32_t _available() override;

        /**
         * @brief End serial port operation
         * 
         * @details Stub implementation for virtual port lifecycle. No action needed
         *          since CAN tunneling doesn't require explicit close operations.
         *          Port remains functional and registered after end() called.
         */
        void _end() override {}

        /**
         * @brief Flush transmit buffer
         * 
         * @details Stub implementation for virtual port. Flushing not applicable to
         *          CAN tunneling since data transmitted asynchronously in update()
         *          rather than on explicit flush command.
         */
        void _flush() override {}

        /**
         * @brief Discard all pending input data
         * 
         * @details Clears RX buffer, discarding any received data not yet read.
         *          Used when protocol needs to resynchronize or discard stale data.
         *          Protected by semaphore for thread safety.
         * 
         * @return true if input discarded successfully, false if buffers not initialized
         */
        bool _discard_input() override;

        /**
         * @brief Calculate time constraint for receiving specified bytes
         * 
         * @details Estimates maximum time in microseconds to receive nbytes based
         *          on configured baud rate. Used by protocol implementations for
         *          timeout calculations and receive timing validation.
         * 
         * @param[in] nbytes Number of bytes for timing calculation
         * 
         * @return Time in microseconds to receive nbytes at current baud rate
         * 
         * @note Returns value based on stored baudrate parameter
         * @note Actual CAN latency may differ from calculated serial timing
         */
        uint64_t receive_time_constraint_us(uint16_t nbytes) override;

        /**
         * @brief Receive buffer for incoming CAN tunnel data
         * 
         * @details Circular buffer holding data received via uavcan_tunnel_Targetted
         *          messages awaiting read() by protocol handler. Size configured in begin().
         *          Allocated dynamically in init_buffers().
         * 
         * @warning NULL if buffers not initialized or allocation failed
         */
        ByteBuffer *readbuffer;

        /**
         * @brief Transmit buffer for outgoing CAN tunnel data
         * 
         * @details Circular buffer holding data from write() awaiting transmission
         *          via uavcan_tunnel_Targetted messages. Size configured in begin().
         *          Allocated dynamically in init_buffers().
         * 
         * @warning NULL if buffers not initialized or allocation failed
         */
        ByteBuffer *writebuffer;

        /**
         * @brief Configured baud rate for port
         * 
         * @details Baud rate setting from begin() call, stored for informational purposes
         *          and timing calculations. Does not directly control CAN bus speed.
         *          May be communicated to remote node as configuration hint.
         */
        uint32_t baudrate;

        /**
         * @brief Timestamp of last successful CAN transmission
         * 
         * @details Millisecond timestamp (from AP_HAL::millis()) when data was last
         *          sent over CAN. Used for activity monitoring and timeout detection.
         */
        uint32_t last_send_ms;

        /**
         * @brief TX buffer size at last reconfiguration
         * 
         * @details Stores transmit buffer size for detecting buffer size changes
         *          requiring reallocation. Used in begin() for optimization.
         */
        uint32_t last_size_tx;

        /**
         * @brief RX buffer size at last reconfiguration
         * 
         * @details Stores receive buffer size for detecting buffer size changes
         *          requiring reallocation. Used in begin() for optimization.
         */
        uint32_t last_size_rx;

        /**
         * @brief Timestamp of last received data
         * 
         * @details Microsecond timestamp (from AP_HAL::micros64()) when data was last
         *          received via CAN tunnel message. Used for receive timing validation
         *          and protocol timeout calculations.
         */
        uint64_t last_recv_us;

        /**
         * @brief Cumulative transmitted bytes counter
         * 
         * @details Total count of bytes successfully written to TX buffer since
         *          port initialization. Used for statistics reporting and monitoring.
         *          Reported via get_total_tx_bytes() when HAL_UART_STATS_ENABLED.
         */
        uint32_t tx_stats_bytes;

        /**
         * @brief Cumulative received bytes counter
         * 
         * @details Total count of bytes successfully received via CAN and placed
         *          in RX buffer since port initialization. Used for statistics
         *          reporting and monitoring. Reported via get_total_rx_bytes().
         */
        uint32_t rx_stats_bytes;

        /**
         * @brief Cumulative dropped receive bytes counter
         * 
         * @details Total count of bytes dropped due to RX buffer overflow since
         *          port initialization. Non-zero value indicates buffer too small
         *          for data rate or protocol not reading fast enough. Reported via
         *          get_total_dropped_rx_bytes() when HAL_UART_STATS_ENABLED.
         * 
         * @warning Non-zero dropped bytes indicates data loss requiring investigation
         */
        uint32_t rx_stats_dropped_bytes;

        /**
         * @brief Semaphore for thread-safe buffer access
         * 
         * @details Protects read/write buffer access between main thread operations
         *          (read, write, available, txspace) and CAN receive callback thread
         *          (handle_tunnel_targetted). Must be acquired before any buffer operation.
         * 
         * @note Uses HAL_Semaphore (mutex-like) for mutual exclusion
         * @warning Failure to acquire semaphore can cause race conditions and data corruption
         */
        HAL_Semaphore sem;

    protected:

#if HAL_UART_STATS_ENABLED
        /**
         * @brief Get cumulative transmitted bytes count
         * 
         * @details Returns total bytes written to TX buffer since port initialization.
         *          Used by statistics reporting system for bandwidth monitoring.
         * 
         * @return Cumulative transmitted bytes (tx_stats_bytes)
         * 
         * @note Only available when HAL_UART_STATS_ENABLED defined
         */
        uint32_t get_total_tx_bytes() const override { return tx_stats_bytes; }

        /**
         * @brief Get cumulative received bytes count
         * 
         * @details Returns total bytes received via CAN and placed in RX buffer
         *          since port initialization. Used for statistics and health monitoring.
         * 
         * @return Cumulative received bytes (rx_stats_bytes)
         * 
         * @note Only available when HAL_UART_STATS_ENABLED defined
         */
        uint32_t get_total_rx_bytes() const override { return rx_stats_bytes; }

        /**
         * @brief Get cumulative dropped receive bytes count
         * 
         * @details Returns total bytes dropped due to RX buffer overflow since
         *          port initialization. Non-zero indicates buffer sizing or
         *          protocol read rate issues requiring attention.
         * 
         * @return Cumulative dropped bytes (rx_stats_dropped_bytes)
         * 
         * @note Only available when HAL_UART_STATS_ENABLED defined
         * @warning Non-zero value indicates data loss occurring on this port
         */
        uint32_t get_total_dropped_rx_bytes() const override { return rx_stats_dropped_bytes; }
#endif
    };

    /**
     * @brief Array of virtual serial ports
     * 
     * @details Fixed array of Port instances, one for each supported virtual serial
     *          port. Size determined by AP_DRONECAN_SERIAL_NUM_PORTS compile-time
     *          constant. Each port independently configured for different target
     *          nodes, protocols, and buffer sizes.
     * 
     * @note Maximum ports limited by AP_DRONECAN_SERIAL_NUM_PORTS (default 3)
     * @note All ports allocated regardless of enable state (uses minimal RAM when unused)
     */
    Port ports[AP_DRONECAN_SERIAL_NUM_PORTS];

private:
    /**
     * @brief Pointer to parent DroneCAN driver instance
     * 
     * @details Reference to owning AP_DroneCAN driver, used for accessing CAN
     *          interface, sending tunnel messages, and registering message handlers.
     *          Set during init() call.
     * 
     * @note NULL until init() called
     */
    AP_DroneCAN *dronecan;

    /**
     * @brief Publisher for outbound tunnel messages
     * 
     * @details Canard publisher instance for sending uavcan_tunnel_Targetted messages.
     *          Used by update() to transmit pending TX buffer data to remote nodes.
     *          Allocated during init() and manages message priority, transfer ID,
     *          and CAN frame generation.
     * 
     * @note NULL until init() called
     * @see uavcan_tunnel_Targetted message definition
     */
    Canard::Publisher<uavcan_tunnel_Targetted> *targetted;

    /**
     * @brief Static callback handler for incoming tunnel messages
     * 
     * @details Registered with DroneCAN driver to receive uavcan_tunnel_Targetted
     *          messages from remote nodes. Routes received data to appropriate Port
     *          instance based on target node and protocol ID matching. Executes in
     *          CAN receive interrupt/callback context.
     * 
     * @param[in] dronecan Pointer to DroneCAN driver instance receiving the message
     * @param[in] transfer CanardRxTransfer containing message metadata and timing
     * @param[in] msg Decoded uavcan_tunnel_Targetted message with serial data
     * 
     * @note Executes in interrupt/callback context - must be fast and thread-safe
     * @note Acquires port semaphore before writing to RX buffer
     * 
     * @warning Must not block or perform lengthy operations
     */
    static void handle_tunnel_targetted(AP_DroneCAN *dronecan,
                                        const CanardRxTransfer& transfer,
                                        const uavcan_tunnel_Targetted &msg);

    /**
     * @brief Static array of serial manager instances per CAN driver
     * 
     * @details One AP_DroneCAN_Serial instance per DroneCAN driver (up to
     *          HAL_MAX_CAN_PROTOCOL_DRIVERS). Allows static message handler to
     *          locate correct instance when receiving tunnel messages. Indexed
     *          by DroneCAN driver number.
     * 
     * @note Size limited by HAL_MAX_CAN_PROTOCOL_DRIVERS (typically 2-3)
     */
    static AP_DroneCAN_Serial *serial[HAL_MAX_CAN_PROTOCOL_DRIVERS];
};
