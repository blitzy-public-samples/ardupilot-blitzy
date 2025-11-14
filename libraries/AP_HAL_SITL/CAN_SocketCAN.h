/**
 * @file CAN_SocketCAN.h
 * @brief Linux SocketCAN transport implementation for SITL CAN simulation
 * 
 * @details This file implements CAN communication for Software-In-The-Loop (SITL)
 *          simulation using Linux SocketCAN. SocketCAN provides a standardized
 *          interface for CAN hardware on Linux systems using the PF_CAN protocol
 *          family with raw sockets.
 * 
 *          The implementation uses virtual CAN (vcan) interfaces, which are kernel-level
 *          virtual CAN devices that allow CAN communication without physical hardware.
 *          This enables testing of CAN-based features in SITL simulation.
 * 
 *          Setup Requirements:
 *          - Linux kernel with SocketCAN support (CONFIG_CAN)
 *          - vcan kernel module loaded: sudo modprobe vcan
 *          - Virtual CAN interface created:
 *            sudo ip link add dev vcan0 type vcan
 *            sudo ip link set up vcan0
 * 
 *          Integration:
 *          - Works with standard Linux CAN tools (can-utils: cansend, candump, etc.)
 *          - Compatible with Wireshark for CAN traffic analysis
 *          - Allows testing with actual CAN peripherals in simulation environment
 *          - Maps ArduPilot CANFrame to Linux can_frame structure
 * 
 *          Limitations:
 *          - Linux-only implementation (requires SocketCAN kernel support)
 *          - CAN-FD (Flexible Data-Rate) is not currently supported
 *          - Only available when HAL_CAN_WITH_SOCKETCAN is defined at compile time
 * 
 * @note This transport is only compiled when both HAL_NUM_CAN_IFACES and
 *       HAL_CAN_WITH_SOCKETCAN are defined.
 * 
 * @warning Platform-specific: This implementation requires Linux with SocketCAN
 *          kernel support and will not compile on other platforms.
 * 
 * @see CAN_Transport for the base transport interface
 * @see AP_HAL::CANFrame for the frame structure being transported
 */
#pragma once

#include "CAN_Transport.h"

#if HAL_NUM_CAN_IFACES && HAL_CAN_WITH_SOCKETCAN

/**
 * @class CAN_SocketCAN
 * @brief Linux SocketCAN transport for SITL CAN communication
 * 
 * @details Implements the CAN_Transport interface using Linux SocketCAN, providing
 *          CAN communication capabilities for SITL simulation through virtual CAN
 *          interfaces (vcan). This allows ArduPilot to communicate with simulated
 *          or real CAN peripherals during software testing.
 * 
 *          The class opens a raw SocketCAN socket (PF_CAN/CAN_RAW) bound to a
 *          virtual CAN interface (vcan%u where %u is the instance number). All
 *          CAN frames are mapped to the Linux can_frame structure for transmission
 *          and reception.
 * 
 *          Typical lifecycle:
 *          1. Instantiate CAN_SocketCAN object
 *          2. Call init() with CAN instance number (maps to vcan%u interface)
 *          3. Use send() to transmit CAN frames
 *          4. Use receive() to read incoming CAN frames
 *          5. Socket automatically closed on destruction
 * 
 *          Thread Safety: Not inherently thread-safe. Caller must ensure proper
 *          synchronization if accessing from multiple threads.
 * 
 * @note Virtual CAN interface must be created and brought up before calling init():
 *       sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
 */
class CAN_SocketCAN : public CAN_Transport {
public:
    /**
     * @brief Initialize the SocketCAN transport for a specific CAN instance
     * 
     * @details Opens a raw SocketCAN socket (AF_CAN/CAN_RAW) and binds it to
     *          the virtual CAN interface vcan%u (where %u is the instance number).
     *          The socket is configured as non-blocking to prevent blocking the
     *          SITL scheduler.
     * 
     *          This method performs the following operations:
     *          1. Creates a raw CAN socket (socket(PF_CAN, SOCK_RAW, CAN_RAW))
     *          2. Resolves the vcan interface name (e.g., "vcan0" for instance 0)
     *          3. Binds the socket to the specified interface
     *          4. Sets the socket to non-blocking mode
     * 
     *          Prerequisites:
     *          - vcan kernel module must be loaded (modprobe vcan)
     *          - Virtual CAN interface must exist (ip link add dev vcan%u type vcan)
     *          - Interface must be up (ip link set up vcan%u)
     * 
     * @param[in] instance CAN instance number (0-based), maps to vcan%u interface name
     * 
     * @return true if socket created, bound, and configured successfully
     * @return false if socket creation, interface binding, or configuration failed
     * 
     * @note This method should be called once during SITL CAN initialization
     * @warning Failure typically indicates missing vcan interface or SocketCAN support
     * 
     * @see CAN_Transport::init()
     */
    bool init(uint8_t instance) override;

    /**
     * @brief Send a CAN frame via SocketCAN
     * 
     * @details Transmits a CAN frame through the SocketCAN interface by converting
     *          the ArduPilot CANFrame structure to a Linux can_frame structure and
     *          writing it to the socket file descriptor.
     * 
     *          The frame conversion includes:
     *          - CAN identifier (standard 11-bit or extended 29-bit)
     *          - Data length code (DLC, 0-8 bytes)
     *          - Data payload (up to 8 bytes for classic CAN)
     *          - Frame flags (extended frame, RTR, error frame)
     * 
     *          The socket is non-blocking, so this method returns immediately if
     *          the send buffer is full (returns false).
     * 
     * @param[in] frame CAN frame to transmit, contains ID, DLC, data, and flags
     * 
     * @return true if frame successfully written to SocketCAN interface
     * @return false if write failed (buffer full, socket error, or not initialized)
     * 
     * @note Non-blocking operation - failure may indicate temporary buffer full condition
     * @note CAN-FD frames are not currently supported
     * 
     * @see CAN_Transport::send()
     * @see AP_HAL::CANFrame
     */
    bool send(const AP_HAL::CANFrame &frame) override;

    /**
     * @brief Receive a CAN frame from SocketCAN
     * 
     * @details Reads a CAN frame from the SocketCAN interface by reading from the
     *          socket file descriptor and converting the Linux can_frame structure
     *          to an ArduPilot CANFrame.
     * 
     *          The frame conversion includes:
     *          - CAN identifier extraction (standard or extended)
     *          - Data length code (DLC)
     *          - Data payload (up to 8 bytes)
     *          - Frame flag interpretation (extended, RTR, error)
     * 
     *          The socket is non-blocking, so this method returns immediately with
     *          false if no frame is available (EAGAIN/EWOULDBLOCK).
     * 
     * @param[out] frame CAN frame structure to populate with received data
     * 
     * @return true if frame successfully read and converted from SocketCAN
     * @return false if no frame available, read error, or socket not initialized
     * 
     * @note Non-blocking operation - false return is normal when no data available
     * @note Caller should check get_read_fd() for readability before calling
     * @note CAN-FD frames are not currently supported
     * 
     * @see CAN_Transport::receive()
     * @see AP_HAL::CANFrame
     */
    bool receive(AP_HAL::CANFrame &frame) override;

    /**
     * @brief Get the file descriptor for the SocketCAN socket
     * 
     * @details Returns the raw file descriptor for the SocketCAN socket, allowing
     *          the caller to use select()/poll()/epoll() to monitor for readable
     *          data. This enables efficient event-driven CAN frame reception without
     *          busy polling.
     * 
     *          Typical usage:
     *          1. Call get_read_fd() to obtain the socket file descriptor
     *          2. Add fd to select/poll set monitoring for read events
     *          3. When fd becomes readable, call receive() to read frame(s)
     * 
     * @return File descriptor of the SocketCAN socket (>=0 if initialized)
     * @return -1 if socket not yet initialized via init()
     * 
     * @note The returned file descriptor should not be closed by the caller
     * @note File descriptor is managed by this class and closed on destruction
     * 
     * @see CAN_Transport::get_read_fd()
     */
    int get_read_fd(void) const override {
        return fd;
    }

private:
    /**
     * @brief SocketCAN socket file descriptor
     * 
     * @details File descriptor for the raw SocketCAN socket (AF_CAN/CAN_RAW) bound
     *          to the virtual CAN interface. Initialized to -1 (invalid) and set to
     *          a valid file descriptor (>=0) by init() when the socket is successfully
     *          created and bound.
     * 
     *          This file descriptor is used for:
     *          - Writing CAN frames via write() system call in send()
     *          - Reading CAN frames via read() system call in receive()
     *          - Monitoring socket readability via select()/poll() in get_read_fd()
     * 
     * @note Value of -1 indicates uninitialized/closed socket
     * @note Socket is configured as non-blocking (O_NONBLOCK)
     */
    int fd = -1;
};

#endif // HAL_NUM_CAN_IFACES
