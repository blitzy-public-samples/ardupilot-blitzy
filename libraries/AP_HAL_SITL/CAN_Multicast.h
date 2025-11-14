/**
 * @file CAN_Multicast.h
 * @brief UDP multicast transport for CAN bus simulation in SITL
 * 
 * @details This file implements a UDP multicast-based CAN transport layer for
 *          Software In The Loop (SITL) simulation. It enables multiple SITL
 *          instances to communicate over a simulated CAN network using multicast
 *          UDP datagrams, allowing developers to test multi-vehicle CAN scenarios
 *          without physical hardware.
 *          
 *          The multicast transport uses IP multicast groups to simulate the
 *          broadcast nature of CAN bus communication. Each SITL CAN interface
 *          instance joins a unique multicast group, allowing multiple simulated
 *          vehicles on the same host or network to exchange CAN frames.
 *          
 *          Key features:
 *          - Multi-vehicle CAN network simulation
 *          - UDP multicast for broadcast semantics
 *          - CRC16 CCITT packet validation
 *          - CAN-FD frame support
 *          - Integration with DroneCAN/UAVCAN testing
 *          
 *          Packet format (network byte order):
 *          - Magic number: 0x2934 (2 bytes) - packet identification
 *          - CRC16: (2 bytes) - CCITT CRC over flags + message_id + data
 *          - Flags: (2 bytes) - bit 0: CAN-FD flag
 *          - Message ID: (4 bytes) - CAN message identifier
 *          - Data: (0-64 bytes) - CAN frame payload
 *          
 *          Network configuration:
 *          - Base multicast address: 239.65.82.0
 *          - Per-instance addressing: last octet = instance number
 *            (e.g., instance 0 → 239.65.82.0, instance 1 → 239.65.82.1)
 *          - UDP port: 57732
 *          
 * @note This transport is only available when HAL_NUM_CAN_IFACES > 0
 * @warning UDP multicast is inherently unreliable - packets may be lost,
 *          duplicated, or reordered. This simulates real-world CAN bus
 *          imperfections and tests error handling, but may not reflect
 *          the high reliability of physical CAN networks.
 * 
 * @see CAN_Transport Base class interface
 * @see CANSocketIface Integration point for SITL CAN interface
 * 
 * Source: libraries/AP_HAL_SITL/CAN_Multicast.cpp:20-32
 */
#pragma once

#include "CAN_Transport.h"

#if HAL_NUM_CAN_IFACES

/**
 * @class CAN_Multicast
 * @brief UDP multicast transport implementation for SITL CAN bus simulation
 * 
 * @details CAN_Multicast implements the CAN_Transport interface using UDP
 *          multicast to simulate CAN bus communication between multiple SITL
 *          instances. This enables testing of multi-vehicle scenarios involving
 *          DroneCAN/UAVCAN communication without requiring physical hardware.
 *          
 *          **Architecture:**
 *          - Inherits from CAN_Transport base class
 *          - Uses SocketAPM_native for UDP multicast operations
 *          - Each CAN interface instance joins a unique multicast group
 *          - Sends CAN frames as UDP multicast datagrams
 *          - Receives frames from all other instances on same multicast group
 *          
 *          **Use Cases:**
 *          1. Multi-vehicle DroneCAN testing (e.g., swarm communication)
 *          2. CAN peripheral simulation (GPS, compass, ESC telemetry)
 *          3. Protocol testing without physical CAN hardware
 *          4. Network latency and packet loss simulation
 *          5. Integration testing with multiple autopilots
 *          
 *          **Network Requirements:**
 *          - Multicast routing must be enabled on the host system
 *          - Firewall must allow UDP port 57732
 *          - For cross-host testing, network switches must support IGMP
 *          - Multicast TTL may need adjustment for routed scenarios
 *          
 *          **Packet Validation:**
 *          - Magic number 0x2934 identifies valid CAN multicast packets
 *          - CRC16 CCITT checksum validates packet integrity
 *          - Corrupted packets are silently discarded
 *          - Minimum packet size: 10 bytes (header only)
 *          - Maximum packet size: 74 bytes (header + 64 byte CAN-FD payload)
 *          
 *          **Integration:**
 *          - Used by CANSocketIface (libraries/AP_HAL_SITL/CANSocketIface.cpp)
 *          - Compatible with DroneCAN GUI tool for monitoring
 *          - Packet format compatible with other SITL CAN analysis tools
 *          
 * @note This class is only compiled when HAL_NUM_CAN_IFACES > 0
 * @warning UDP multicast provides best-effort delivery. Packets may be:
 *          - Lost (simulates CAN bus errors and congestion)
 *          - Duplicated (simulates retransmissions)
 *          - Reordered (simulates timing variations)
 *          This behavior is intentional for realistic testing but may cause
 *          intermittent test failures if protocols don't handle errors properly.
 * 
 * @warning Multicast groups are shared across all processes on the host.
 *          Running multiple unrelated SITL simulations simultaneously on the
 *          same instance number will cause CAN traffic cross-contamination.
 * 
 * Source: libraries/AP_HAL_SITL/CAN_Multicast.cpp:37-92
 */
class CAN_Multicast : public CAN_Transport {
public:

    /**
     * @brief Initialize the multicast CAN transport for a specific instance
     * 
     * @details Initializes the UDP multicast socket and joins the appropriate
     *          multicast group for the given CAN interface instance. The multicast
     *          address is derived by mutating the last octet of the base address
     *          (239.65.82.0) with the instance number.
     *          
     *          Address mutation examples:
     *          - Instance 0 → 239.65.82.0
     *          - Instance 1 → 239.65.82.1
     *          - Instance 2 → 239.65.82.2
     *          
     *          This allows multiple CAN interfaces to operate independently on the
     *          same host or network without interference. Each interface can
     *          simulate a separate physical CAN bus.
     *          
     *          The socket is configured for:
     *          - Multicast UDP on port 57732
     *          - Non-blocking I/O for polling in the scheduler
     *          - Multicast group membership for broadcast reception
     *          - Appropriate buffer sizes for CAN frame bursts
     * 
     * @param[in] instance CAN interface instance number (0-9 supported due to
     *                     single-digit address mutation in implementation)
     * 
     * @return true if socket initialization and multicast join succeeded
     * @return false if socket creation, bind, or multicast join failed
     * 
     * @note Must be called before send() or receive() operations
     * @note Instance numbers above 9 may result in invalid multicast addresses
     *       due to the single-character address mutation implementation
     * 
     * @warning Calling init() multiple times will fail as the socket cannot
     *          rebind to the same port
     * 
     * Source: libraries/AP_HAL_SITL/CAN_Multicast.cpp:37-44
     */
    bool init(uint8_t instance) override;
    
    /**
     * @brief Send a CAN frame via UDP multicast
     * 
     * @details Transmits a CAN frame by encapsulating it in a multicast UDP packet
     *          with the following structure:
     *          
     *          Packet format (total size: 10 + data_length bytes):
     *          - Bytes 0-1: Magic number (0x2934) for packet identification
     *          - Bytes 2-3: CRC16 CCITT checksum over flags + message_id + data
     *          - Bytes 4-5: Flags (bit 0: CAN-FD indicator)
     *          - Bytes 6-9: CAN message ID (29-bit extended or 11-bit standard)
     *          - Bytes 10+: CAN frame data (0-8 bytes for CAN 2.0, up to 64 for CAN-FD)
     *          
     *          The CRC16 CCITT checksum is calculated over the flags, message_id,
     *          and data fields using initial value 0xFFFF, providing integrity
     *          validation for the multicast transport.
     *          
     *          CAN-FD Support:
     *          If HAL_CANFD_SUPPORTED is enabled and the frame is a CAN-FD frame,
     *          the MCAST_FLAG_CANFD bit (0x0001) is set in the flags field. This
     *          allows receivers to properly reconstruct the frame type.
     *          
     *          The packet is sent via UDP multicast to all other SITL instances
     *          subscribed to the same multicast group, simulating the broadcast
     *          nature of a physical CAN bus.
     * 
     * @param[in] frame CAN frame to transmit, containing:
     *                  - id: CAN message identifier
     *                  - dlc: Data length code (0-8 for CAN 2.0, up to 15 for CAN-FD)
     *                  - data: Frame payload bytes
     *                  - canfd: CAN-FD frame indicator (if supported)
     * 
     * @return true if the complete packet was sent successfully
     * @return false if the UDP send operation failed or sent incomplete data
     * 
     * @note This function is typically called at high frequency (1kHz or more)
     *       from the CAN interface driver task
     * 
     * @warning UDP multicast is unreliable - the packet may not reach all
     *          receivers even if this function returns true. Network congestion,
     *          packet loss, or receiver buffer overflows can cause frame loss.
     * 
     * @warning The CRC provides data integrity checking but not authentication.
     *          Malicious or corrupted packets on the multicast group could be
     *          accepted if they match the expected format.
     * 
     * Source: libraries/AP_HAL_SITL/CAN_Multicast.cpp:49-65
     */
    bool send(const AP_HAL::CANFrame &frame) override;
    
    /**
     * @brief Receive a CAN frame from UDP multicast
     * 
     * @details Attempts to receive a CAN frame from the multicast socket. This
     *          function performs non-blocking reads and validates the received
     *          packet before returning a CAN frame to the caller.
     *          
     *          Reception and validation sequence:
     *          1. Non-blocking UDP receive on multicast socket
     *          2. Minimum size check (at least 10 bytes for header)
     *          3. Magic number validation (must be 0x2934)
     *          4. CRC16 CCITT validation over flags + message_id + data
     *          5. CAN frame reconstruction from validated packet
     *          6. Event semaphore signaling (if configured)
     *          
     *          The CRC validation protects against:
     *          - Corrupted packets due to network transmission errors
     *          - Malformed packets from non-CAN multicast traffic
     *          - Bit errors in UDP payload
     *          
     *          Invalid packets are silently discarded (returns false) to maintain
     *          the best-effort semantics of the CAN transport.
     *          
     *          **Frame Reconstruction:**
     *          The received packet is parsed to extract:
     *          - CAN message ID from bytes 6-9
     *          - Data payload from bytes 10+ (length = packet_size - 10)
     *          - CAN-FD flag from flags field bit 0
     *          
     *          The frame is reconstructed using placement new to ensure proper
     *          initialization of the CANFrame object.
     *          
     *          **Event Notification:**
     *          If a binary semaphore handle has been registered via
     *          set_event_handle(), it is signaled after successful frame
     *          reception. This wakes up waiting tasks in the CAN driver,
     *          enabling efficient event-driven processing instead of polling.
     * 
     * @param[out] frame CAN frame object to populate with received data.
     *                   Only modified if a valid frame is received.
     *                   Contains:
     *                   - id: CAN message identifier
     *                   - dlc: Data length code derived from payload size
     *                   - data: Received frame payload
     *                   - canfd: CAN-FD indicator from packet flags
     * 
     * @return true if a valid CAN frame was received and validated
     * @return false if no data available, packet too short, magic number invalid,
     *               or CRC validation failed
     * 
     * @note This function is called at high frequency (typically 1kHz) from the
     *       CAN interface polling loop. It must return quickly to avoid blocking
     *       the scheduler.
     * 
     * @note The function performs non-blocking I/O. Returns false immediately
     *       if no data is available on the socket.
     * 
     * @warning Packets failing validation are silently dropped. This is
     *          intentional behavior to filter out noise on the multicast group,
     *          but it means that debugging packet reception issues requires
     *          external network monitoring tools.
     * 
     * @warning This function receives packets from ALL senders on the multicast
     *          group, including the sending instance itself (loopback). The CAN
     *          driver layer is responsible for filtering out self-transmitted
     *          frames if needed.
     * 
     * Source: libraries/AP_HAL_SITL/CAN_Multicast.cpp:70-92
     */
    bool receive(AP_HAL::CANFrame &frame) override;
    
    /**
     * @brief Get the file descriptor for socket read operations
     * 
     * @details Returns the underlying file descriptor of the UDP multicast socket,
     *          allowing the CAN interface driver to integrate with select/poll/epoll
     *          mechanisms for efficient event-driven I/O.
     *          
     *          The file descriptor can be used for:
     *          - select() or poll() system calls to wait for incoming data
     *          - epoll monitoring in event-driven architectures  
     *          - Integration with SITL scheduler I/O monitoring
     *          - Non-blocking read readiness checking
     *          
     *          This enables the CAN driver to avoid busy-waiting on the receive()
     *          method and instead block until data is available, reducing CPU
     *          usage in SITL simulations.
     *          
     *          **Usage Pattern:**
     *          ```cpp
     *          int fd = transport->get_read_fd();
     *          fd_set read_fds;
     *          FD_ZERO(&read_fds);
     *          FD_SET(fd, &read_fds);
     *          select(fd + 1, &read_fds, nullptr, nullptr, &timeout);
     *          if (FD_ISSET(fd, &read_fds)) {
     *              transport->receive(frame);  // Data is ready
     *          }
     *          ```
     * 
     * @return File descriptor for the multicast UDP socket (positive integer)
     * @return -1 if the socket is not initialized or invalid
     * 
     * @note The returned file descriptor remains valid for the lifetime of the
     *       CAN_Multicast object or until the socket is closed
     * 
     * @note The file descriptor is configured for non-blocking I/O. Attempts to
     *       read directly from the FD (instead of using receive()) may return
     *       EAGAIN or EWOULDBLOCK if no data is available.
     * 
     * @warning Do not close() this file descriptor directly. Socket lifecycle
     *          is managed by the SocketAPM_native object and will be cleaned up
     *          when the CAN_Multicast object is destroyed.
     * 
     * @warning Do not perform write operations on this file descriptor. Use the
     *          send() method instead to ensure proper packet formatting and CRC
     *          calculation.
     */
    int get_read_fd(void) const override {
        return sock.get_read_fd();
    }

private:
    /**
     * @brief Native UDP multicast socket for CAN frame transport
     * 
     * @details SocketAPM_native wrapper around a UDP socket configured for:
     *          - Multicast group membership on 239.65.82.x addresses
     *          - Non-blocking I/O for scheduler integration
     *          - Datagram-based communication (connectionless)
     *          - Port 57732 for CAN multicast protocol
     *          
     *          The boolean parameter 'true' in initialization enables datagram
     *          mode (UDP) rather than stream mode (TCP).
     *          
     *          Socket lifecycle:
     *          - Created during object construction
     *          - Initialized and bound during init()
     *          - Used for send/receive operations
     *          - Automatically closed during object destruction
     * 
     * @note The socket is shared between send and receive operations on the
     *       same multicast group, allowing bidirectional communication.
     */
    SocketAPM_native sock{true};
};

#endif // HAL_NUM_CAN_IFACES
