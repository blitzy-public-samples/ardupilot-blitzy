/**
 * @file CAN_Transport.h
 * @brief Abstract base class for CAN transport interfaces in ArduPilot SITL
 * 
 * @details This file defines the CAN_Transport interface, which provides an
 *          abstraction layer for simulating CAN bus communication in the
 *          Software-In-The-Loop (SITL) environment. SITL supports two main
 *          transport implementations:
 *          
 *          - **CAN_Multicast**: Simulates CAN using UDP multicast, allowing
 *            multiple SITL instances to communicate over a network
 *          - **CAN_SocketCAN**: Uses Linux SocketCAN for integration with
 *            virtual or real CAN hardware interfaces (vcan, can0, etc.)
 *          
 *          This abstraction enables testing CAN-based features (DroneCAN/UAVCAN
 *          peripherals, CAN sensors, ESCs) without physical hardware.
 * 
 * @warning SITL CAN simulation differs from real hardware in timing accuracy,
 *          reliability guarantees, and arbitration behavior. Real CAN hardware
 *          provides deterministic bus arbitration and precise timing, while
 *          SITL transports may experience network delays and packet loss.
 *          Always validate CAN-dependent features on actual hardware.
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */
#pragma once

#include "AP_HAL_SITL.h"

#if HAL_NUM_CAN_IFACES

#include <AP_HAL/CANIface.h>

/**
 * @class CAN_Transport
 * @brief Abstract interface for CAN bus transport simulation in SITL
 * 
 * @details CAN_Transport defines the API for simulating CAN communication in
 *          ArduPilot's Software-In-The-Loop environment. This base class provides
 *          a unified interface for different transport mechanisms, allowing SITL
 *          to simulate CAN buses using various backend implementations.
 *          
 *          **Architecture**:
 *          - Pure virtual interface (abstract base class)
 *          - Implemented by CAN_Multicast (UDP-based) and CAN_SocketCAN (Linux SocketCAN)
 *          - Integrates with AP_HAL::CANIface for frame compatibility
 *          - Supports event-driven receive via file descriptor polling
 *          
 *          **Transport Implementations**:
 *          - **CAN_Multicast**: Network-based simulation using UDP multicast groups.
 *            Allows multiple SITL instances on different hosts to share a virtual
 *            CAN bus. Useful for multi-vehicle simulations and distributed testing.
 *          - **CAN_SocketCAN**: Linux SocketCAN integration using virtual (vcan)
 *            or physical CAN interfaces. Enables HITL testing and integration with
 *            CAN analysis tools like candump and cansend.
 *          
 *          **Typical Usage Pattern**:
 *          1. Create transport instance (CAN_Multicast or CAN_SocketCAN)
 *          2. Initialize with CAN instance number: init(instance)
 *          3. Set event semaphore for receive notifications: set_event_handle()
 *          4. Send frames: send(frame)
 *          5. Receive frames: receive(frame)
 *          6. Use get_read_fd() for poll/select integration in scheduler
 *          
 *          **Frame Format**:
 *          Uses AP_HAL::CANFrame which encapsulates:
 *          - CAN ID (11-bit standard or 29-bit extended)
 *          - Data payload (0-8 bytes for CAN 2.0, up to 64 for CAN FD)
 *          - RTR (Remote Transmission Request) flag
 *          - Error frame indication
 *          
 *          **Event Handling**:
 *          Transports support non-blocking receive through file descriptors.
 *          The scheduler can poll get_read_fd() to detect incoming frames,
 *          then call receive() to retrieve them. The optional BinarySemaphore
 *          provides additional event notification.
 *          
 *          **Queueing Behavior**:
 *          Implementations typically maintain internal queues:
 *          - Transmit queue: Buffers outgoing frames when network/interface busy
 *          - Receive queue: Buffers incoming frames until application retrieves them
 *          - Queue depths vary by implementation and available resources
 *          
 * @note This interface does not implement CAN bus arbitration or priority
 *       handling like real hardware. Frame ordering may differ from physical CAN.
 * 
 * @warning Timing behavior differs from real CAN controllers:
 *          - No deterministic latency guarantees
 *          - Network delays affect multicast transport
 *          - No cycle-accurate timing simulation
 *          - Bus errors and arbitration loss not fully simulated
 * 
 * @see CAN_Multicast for UDP multicast implementation
 * @see CAN_SocketCAN for Linux SocketCAN implementation
 * @see AP_HAL::CANIface for hardware CAN interface abstraction
 */
class CAN_Transport {
public:
    /**
     * @brief Virtual destructor for proper cleanup of derived classes
     * 
     * @details Ensures derived transport implementations (CAN_Multicast,
     *          CAN_SocketCAN) are properly destroyed when deleted through
     *          base class pointer. Derived classes should close network
     *          sockets, file descriptors, and release resources.
     */
    virtual ~CAN_Transport() {}
    
    /**
     * @brief Initialize the CAN transport for a specific CAN interface instance
     * 
     * @details Initializes the transport backend (UDP socket, SocketCAN interface)
     *          for the specified CAN instance number. For CAN_Multicast, this
     *          configures the multicast group based on the instance. For
     *          CAN_SocketCAN, this opens the corresponding Linux CAN interface
     *          (e.g., can0, vcan0).
     *          
     *          Must be called before send() or receive() operations.
     *          Implementation should set up all required resources including
     *          sockets, file descriptors, and internal queues.
     * 
     * @param[in] instance CAN interface instance number (0-based), corresponds
     *                     to HAL CAN interface index. Typically 0 or 1 for
     *                     dual-CAN systems.
     * 
     * @return true if initialization successful and transport ready for use
     * @return false if initialization failed (port unavailable, permission denied,
     *               invalid instance number, or resource allocation failure)
     * 
     * @note Initialization failure is typically non-recoverable. SITL should
     *       report the error and may need to be restarted with different parameters.
     */
    virtual bool init(uint8_t instance) = 0;
    
    /**
     * @brief Send a CAN frame through the transport
     * 
     * @details Transmits a CAN frame using the underlying transport mechanism.
     *          For CAN_Multicast, sends frame as UDP packet to multicast group.
     *          For CAN_SocketCAN, writes frame to Linux CAN socket.
     *          
     *          This method may queue frames internally if the transport is busy.
     *          Implementations should handle non-blocking transmission and return
     *          immediately even if frame is queued rather than sent.
     * 
     * @param[in] frame CAN frame to transmit, containing CAN ID, data length,
     *                  payload bytes, and flags (RTR, extended ID, etc.)
     *                  Frame must be properly formatted with valid CAN ID
     *                  and data length (0-8 bytes for CAN 2.0)
     * 
     * @return true if frame accepted for transmission (queued or sent successfully)
     * @return false if frame could not be sent (queue full, transport error,
     *               invalid frame format, or transport not initialized)
     * 
     * @note Unlike real CAN hardware, this does not implement bus arbitration.
     *       All frames that return true will eventually be transmitted in
     *       the order queued, regardless of CAN ID priority.
     * 
     * @warning Transmission timing is non-deterministic due to network latency
     *          and OS scheduling. Do not rely on precise transmission timing.
     */
    virtual bool send(const AP_HAL::CANFrame &frame) = 0;
    
    /**
     * @brief Receive a CAN frame from the transport
     * 
     * @details Retrieves the next available CAN frame from the receive queue.
     *          Should be called when get_read_fd() indicates data is available
     *          or after sem_handle is signaled.
     *          
     *          Non-blocking operation - returns immediately if no frames available.
     *          For event-driven reception, use get_read_fd() with poll/select
     *          to detect incoming frames before calling receive().
     * 
     * @param[out] frame Reference to CANFrame structure that will be filled with
     *                   received frame data including CAN ID, data length, payload,
     *                   and flags. Frame contents undefined if receive returns false.
     * 
     * @return true if a frame was successfully received and frame parameter contains
     *              valid data
     * @return false if no frames available in receive queue, or transport error
     * 
     * @note Multiple frames may be available. Keep calling receive() until it
     *       returns false to drain the receive queue.
     * 
     * @warning Frames may arrive out of order compared to physical CAN bus due to
     *          network reordering (multicast) or OS scheduling. Applications
     *          requiring strict ordering should implement timestamps or sequence numbers.
     */
    virtual bool receive(AP_HAL::CANFrame &frame) = 0;
    
    /**
     * @brief Get file descriptor for receive event polling
     * 
     * @details Returns a file descriptor that becomes readable when CAN frames
     *          are available to receive. Used by the SITL scheduler for efficient
     *          event-driven CAN frame reception through poll() or select().
     *          
     *          For CAN_Multicast, returns UDP socket file descriptor.
     *          For CAN_SocketCAN, returns SocketCAN interface file descriptor.
     *          
     *          The scheduler typically adds this FD to its poll set, allowing
     *          the CAN receive handler to be called only when data is available
     *          rather than busy-polling receive().
     * 
     * @return File descriptor number (>= 0) that can be used with poll/select
     *         to detect incoming CAN frames. FD becomes readable when receive()
     *         will return true.
     * @return -1 if transport not initialized or FD not available
     * 
     * @note The returned file descriptor is owned by the transport implementation.
     *       Do not close or modify it directly. It remains valid until the
     *       transport object is destroyed.
     * 
     * @warning Do not perform blocking reads directly on this FD. Always use
     *          receive() method to retrieve frames to ensure proper protocol
     *          handling and internal state management.
     */
    virtual int get_read_fd(void) const = 0;

    /**
     * @brief Set binary semaphore for receive event notification
     * 
     * @details Registers a binary semaphore that will be signaled when CAN frames
     *          arrive. This provides an alternative to file descriptor polling for
     *          receive event notification. The transport implementation should
     *          signal (give) the semaphore whenever new frames are queued.
     *          
     *          Optional mechanism - not required if using get_read_fd() with
     *          poll/select. Useful when integrating with thread-based architectures
     *          or when file descriptor polling is not available.
     *          
     *          The semaphore is not owned by CAN_Transport and must remain valid
     *          for the lifetime of the transport or until a new semaphore is set.
     * 
     * @param[in] handle Pointer to BinarySemaphore to be signaled on receive events.
     *                   Pass nullptr to disable semaphore notification.
     *                   Semaphore must be initialized before passing to this method.
     * 
     * @note The semaphore will be signaled (give()) by the transport when frames
     *       arrive, but the caller is responsible for waiting (take()) on it.
     *       Multiple frames may arrive before semaphore is consumed.
     * 
     * @note Not all transport implementations may support semaphore notification.
     *       get_read_fd() polling is the primary receive notification mechanism.
     */
    void set_event_handle(AP_HAL::BinarySemaphore *handle) {
        sem_handle = handle;
    }

protected:
    /**
     * @brief Binary semaphore for optional receive event notification
     * 
     * @details Pointer to semaphore that will be signaled when frames are received.
     *          Set via set_event_handle(). May be nullptr if semaphore notification
     *          not in use. Derived classes should signal (give) this semaphore
     *          when frames arrive if it is non-null.
     *          
     *          Provides alternative to FD-based polling for receive events.
     *          Not owned by this class - caller maintains lifetime.
     */
    AP_HAL::BinarySemaphore *sem_handle;
};

#endif // HAL_NUM_CAN_IFACES
