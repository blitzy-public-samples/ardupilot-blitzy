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
 * @file CANSocketIface.h
 * @brief CAN socket interface implementation for Software-In-The-Loop (SITL) simulation
 * 
 * @details This file implements the AP_HAL::CANIface abstract interface for SITL,
 *          providing a virtual CAN bus implementation that allows testing of CAN-based
 *          protocols (DroneCAN/UAVCAN) without physical hardware.
 *          
 *          The implementation uses either Unix domain sockets (Multicast) or Linux
 *          SocketCAN to create a virtual CAN bus environment. Multiple SITL instances
 *          can communicate over this virtual bus, enabling multi-vehicle CAN testing.
 *          
 *          Key features:
 *          - Configurable TX/RX queue depths (default 100 frames each)
 *          - Poll-based I/O with select() integration for efficient waiting
 *          - Statistics collection (TX/RX counts, errors, buffer overruns)
 *          - Support for both standard and extended CAN frame formats
 *          - Integration with CAN_Transport backends (Multicast or SocketCAN)
 *          
 *          Thread Safety: Access to TX/RX queues is protected by HAL_Semaphore.
 *          Multiple threads can safely call send/receive operations.
 * 
 * @warning This is a simulation implementation with limitations:
 *          - No bus arbitration (frames transmitted instantly without collision)
 *          - No realistic CAN timing or bit-level errors
 *          - Performance characteristics do not match real CAN hardware
 *          - Suitable for protocol testing but not timing-critical validation
 * 
 * @note This interface is used by DroneCAN/UAVCAN protocol stacks to communicate
 *       with CAN-based peripherals in SITL simulation environments.
 * 
 * @see AP_HAL::CANIface
 * @see CAN_Transport
 * @see libraries/AP_DroneCAN for protocol implementation
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_HAL_SITL.h"

#if HAL_NUM_CAN_IFACES

#include <AP_HAL/CANIface.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <string>
#include <memory>
#include <map>
#include <unordered_set>
#include <poll.h>
#include "CAN_Transport.h"

namespace HALSITL {

/**
 * @class CANIface
 * @brief Virtual CAN interface implementation for SITL simulation environment
 * 
 * @details Provides a complete implementation of the AP_HAL::CANIface abstract interface
 *          for Software-In-The-Loop simulation. This allows ArduPilot to communicate with
 *          simulated CAN peripherals (GPS, compass, ESCs, etc.) using DroneCAN/UAVCAN
 *          protocols without requiring physical CAN hardware.
 *          
 *          Architecture:
 *          - TX Queue: ObjectArray with configurable depth (default 100 frames)
 *          - RX Queue: ObjectArray with configurable depth (default 100 frames)
 *          - Transport Layer: Pluggable backend (Multicast or SocketCAN)
 *          - I/O Model: Poll-based with select() for efficient blocking
 *          
 *          Frame Flow:
 *          1. send() -> _tx_queue -> _pollWrite() -> transport->send()
 *          2. transport->receive() -> _pollRead() -> _rx_queue -> receive()
 *          
 *          The implementation tracks frames in the socket TX queue to prevent
 *          overwhelming the transport layer. When the queue is full, send()
 *          returns 0 (buffer full) rather than blocking.
 *          
 *          Statistics Collection:
 *          Maintains counters for TX/RX frames, errors, buffer overruns, and
 *          other diagnostic information accessible via get_stats() and get_statistics().
 *          
 *          Simulation Characteristics:
 *          - Instant transmission (no bus arbitration delay)
 *          - Perfect delivery (no bit errors or collisions)
 *          - Timing unrealistic compared to hardware
 *          - Suitable for protocol logic testing, not timing validation
 * 
 * @note Each instance represents one CAN interface (CAN0, CAN1, etc.).
 *       Multiple instances can coexist for multi-CAN simulations.
 * 
 * @warning No busoff condition exists in virtual CAN - is_busoff() always returns false
 * 
 * @see AP_HAL::CANIface for interface contract
 * @see CAN_Transport for transport layer abstraction
 */
class CANIface: public AP_HAL::CANIface {
public:
    /**
     * @brief Construct a CANIface with specified index
     * 
     * @param[in] index Interface number (0 for CAN0, 1 for CAN1, etc.)
     * 
     * @note Increments the global interface counter _num_interfaces
     */
    CANIface(int index)
      : _self_index(index)
      , _frames_in_socket_tx_queue(0)
    {
        _num_interfaces++;
    }
    
    /**
     * @brief Default constructor - uses next available interface index
     * 
     * @details Automatically assigns the next interface number based on
     *          the current value of _num_interfaces
     */
    CANIface() : CANIface(_num_interfaces) {}
    
    static uint8_t _num_interfaces; ///< Global counter of CAN interfaces created

    /**
     * @brief Destructor - cleans up CAN interface resources
     * 
     * @details Currently no explicit cleanup required as transport layer
     *          is managed externally
     */
    ~CANIface() { }

    /**
     * @brief Initialize CAN peripheral with CAN-FD support
     * 
     * @details Initializes the virtual CAN interface with specified bitrates and operating mode.
     *          For SITL, bitrate parameters are mostly informational as there is no real
     *          bus timing. The transport layer (Multicast or SocketCAN) is created and
     *          configured based on SITL parameters.
     *          
     *          Sets up:
     *          - Transport layer (socket communication)
     *          - Poll file descriptor for select operations
     *          - TX/RX queues (cleared to empty state)
     *          - Statistics structures (reset to zero)
     * 
     * @param[in] bitrate      CAN bitrate in bits/second (e.g., 1000000 for 1Mbps)
     * @param[in] fdbitrate    CAN-FD data phase bitrate in bits/second (not used in SITL)
     * @param[in] mode         Operating mode (Normal, Silent, AutomaticTxAbortOnError)
     * 
     * @return true if initialization successful, false on error
     * 
     * @note In SITL, bitrate parameters don't affect actual timing behavior
     * @warning Must be called before any send/receive operations
     * 
     * @see init(bitrate, mode) for non-FD initialization
     */
    bool init(const uint32_t bitrate, const uint32_t fdbitrate, const OperatingMode mode) override;
    
    /**
     * @brief Initialize CAN peripheral without CAN-FD support
     * 
     * @details Simplified initialization for standard CAN (non-FD) operation.
     *          Internally calls the FD version with fdbitrate=0.
     * 
     * @param[in] bitrate CAN bitrate in bits/second (e.g., 1000000 for 1Mbps)
     * @param[in] mode    Operating mode (Normal, Silent, AutomaticTxAbortOnError)
     * 
     * @return true if initialization successful, false on error
     * 
     * @see init(bitrate, fdbitrate, mode)
     */
    bool init(const uint32_t bitrate, const OperatingMode mode) override;

    /**
     * @brief Get the total number of initialized CAN interfaces
     * 
     * @return Number of CANIface instances created (0, 1, or 2 typically)
     * 
     * @note This is a global count across all instances
     */
    static uint8_t num_interfaces(void) {
        return _num_interfaces;
    }

    /**
     * @brief Transmit a CAN frame by placing it in the TX queue
     * 
     * @details Attempts to add a CAN frame to the internal TX queue for transmission.
     *          The frame will be sent asynchronously by _pollWrite() when the transport
     *          layer is ready. If the TX queue is full, the function returns 0 without
     *          blocking.
     *          
     *          Flow: send() -> _tx_queue.push() -> [later] _pollWrite() -> transport->send()
     *          
     *          Queue Management:
     *          - TX queue depth: 100 frames (configurable via ObjectArray template)
     *          - Tracks frames in socket TX queue via _frames_in_socket_tx_queue
     *          - Returns 0 (buffer full) if queue is at capacity
     *          
     *          Thread Safety: Protected by internal HAL_Semaphore (sem)
     * 
     * @param[in] frame         CAN frame to transmit (standard or extended format)
     * @param[in] tx_deadline   Transmission deadline in microseconds (not enforced in SITL)
     * @param[in] flags         CAN I/O flags (Loopback, AbortOnError, etc.)
     * 
     * @return  1 on successful enqueue
     * @return  0 if TX queue is full (try again later)
     * @return -1 on error (interface not initialized)
     * 
     * @note tx_deadline is not enforced in SITL - frames are sent as fast as possible
     * @warning In SITL, transmission is instant with no bus arbitration or timing
     * 
     * @see receive() for RX operation
     * @see _pollWrite() for actual transmission logic
     */
    int16_t send(const AP_HAL::CANFrame& frame, uint64_t tx_deadline,
                 CanIOFlags flags) override;

    /**
     * @brief Receive a CAN frame from the RX queue
     * 
     * @details Attempts to retrieve a CAN frame from the internal RX queue.
     *          Frames are placed in the queue by _pollRead() when received from
     *          the transport layer. If the RX queue is empty, returns 0 immediately
     *          without blocking.
     *          
     *          Flow: transport->receive() -> _pollRead() -> _rx_queue.push() -> receive()
     *          
     *          Queue Management:
     *          - RX queue depth: 100 frames (configurable via ObjectArray template)
     *          - Oldest frame is retrieved first (FIFO order)
     *          - Statistics updated on successful receive
     *          
     *          Thread Safety: Protected by internal HAL_Semaphore (sem)
     * 
     * @param[out] out_frame        Received CAN frame (ID, data, length)
     * @param[out] out_timestamp_us Reception timestamp in microseconds
     * @param[out] out_flags        CAN I/O flags (Loopback indication, etc.)
     * 
     * @return  1 on successful frame retrieval
     * @return  0 if RX queue is empty (no frames available)
     * @return -1 on error (interface not initialized)
     * 
     * @note Timestamps are simulated and may not reflect realistic CAN timing
     * @warning Returns 0 if no frames available - use select() to wait for frames
     * 
     * @see send() for TX operation
     * @see select() for efficient waiting on RX frames
     * @see _pollRead() for frame reception logic
     */
    int16_t receive(AP_HAL::CANFrame& out_frame, uint64_t& out_timestamp_us,
                    CanIOFlags& out_flags) override;

    /**
     * @brief Check if the CAN interface is in bus-off state
     * 
     * @details In real CAN hardware, bus-off occurs after excessive error conditions
     *          (128+ transmit errors). However, virtual CAN in SITL has no error
     *          conditions, collisions, or bit errors, so bus-off never occurs.
     * 
     * @return false always (virtual CAN cannot enter bus-off state)
     * 
     * @note This maintains compatibility with the CANIface interface contract
     * @warning Do not rely on bus-off detection for error handling in SITL tests
     */
    bool is_busoff() const override
    {
        return false;
    }

    /**
     * @brief Flush all pending frames from the TX queue
     * 
     * @details Clears the internal TX queue, discarding all frames that have not
     *          yet been transmitted. Useful for resetting communication state or
     *          responding to mode changes.
     *          
     *          Thread Safety: Protected by internal HAL_Semaphore (sem)
     * 
     * @note Does not affect frames already sent to the transport layer
     * @warning Discarded frames will not be transmitted - use with caution
     * 
     * @see clear_rx() to flush RX queue
     */
    void flush_tx() override;

    /**
     * @brief Clear all frames from the RX queue
     * 
     * @details Discards all received frames that are pending in the RX queue.
     *          Useful for resetting communication state or skipping old data
     *          after configuration changes.
     *          
     *          Thread Safety: Protected by internal HAL_Semaphore (sem)
     * 
     * @note Does not prevent new frames from being received
     * @warning Discarded frames are lost - ensure they're no longer needed
     * 
     * @see flush_tx() to flush TX queue
     */
    void clear_rx() override;

    /**
     * @brief Get the total count of errors detected on this CAN interface
     * 
     * @details Returns cumulative error count from the bus statistics structure.
     *          In SITL, errors are primarily buffer overruns (TX/RX queue full)
     *          rather than actual CAN bus errors (bit errors, CRC errors, etc.).
     *          
     *          Error Sources in SITL:
     *          - RX buffer overrun (frames dropped due to full RX queue)
     *          - TX buffer overrun (frames rejected due to full TX queue)
     *          - Transport layer errors (socket failures)
     * 
     * @return Total error count since initialization or last statistics reset
     * 
     * @note In real hardware, this would include CAN protocol errors
     * @see get_statistics() for detailed error breakdown
     */
    uint32_t getErrorCount() const override;

    /**
     * @brief Check if the CAN interface has been successfully initialized
     * 
     * @details Returns true if init() has been called successfully and the
     *          interface is ready for send/receive operations. Must be true
     *          before attempting any CAN communication.
     * 
     * @return true if init() succeeded, false otherwise
     * 
     * @note Calling send/receive on uninitialized interface returns error (-1)
     * @see init() for initialization
     */
    bool is_initialized() const override;

    /******************************************
     * Select Method                          *
     * ****************************************/
    /**
     * @brief Wait for CAN I/O readiness with timeout
     * 
     * @details Blocks until the interface is ready for reading or writing, or until
     *          the blocking deadline is reached. Uses poll() system call to efficiently
     *          wait for socket events without busy-waiting.
     *          
     *          This is the primary mechanism for efficient CAN I/O in ArduPilot.
     *          Protocol stacks call select() to wait for frames to arrive or for
     *          TX queue space to become available.
     *          
     *          Operation:
     *          1. Checks for pending TX frames (if write requested and pending_tx provided)
     *          2. Polls transport layer socket with calculated timeout
     *          3. Performs read/write operations if socket is ready
     *          4. Updates read/write flags to indicate readiness
     *          
     *          If sem_handle is set, signals the semaphore when data is available,
     *          allowing other threads to wake up and process CAN events.
     * 
     * @param[in,out] read             Input: request read readiness check
     *                                 Output: true if frames available to receive
     * @param[in,out] write            Input: request write readiness check
     *                                 Output: true if TX queue has space
     * @param[in]     pending_tx       Optional frame to send if write is ready (can be nullptr)
     * @param[in]     blocking_deadline Absolute time in microseconds to stop waiting
     * 
     * @return true if I/O is ready (read or write), false if timed out
     * 
     * @note This is called at high frequency by CAN protocol stacks
     * @warning Long timeouts can delay other system operations - keep reasonable
     * 
     * @see set_event_handle() for semaphore-based event notification
     * @see _poll() for internal polling implementation
     */
    bool select(bool &read, bool &write,
                const AP_HAL::CANFrame* const pending_tx,
                uint64_t blocking_deadline) override;
    
    /**
     * @brief Register a binary semaphore for CAN event notification
     * 
     * @details Allows a thread to be notified asynchronously when CAN events occur
     *          (frames received or TX queue space available). The semaphore is
     *          signaled by select() when relevant events happen, enabling efficient
     *          multi-threaded CAN processing.
     *          
     *          This is typically used by CAN protocol stack threads to sleep while
     *          waiting for CAN activity, rather than polling continuously.
     * 
     * @param[in] handle Pointer to BinarySemaphore to signal on events (or nullptr to disable)
     * 
     * @return true if handle was successfully registered
     * 
     * @note Only one semaphore can be registered at a time
     * @see select() for event waiting
     */
    bool set_event_handle(AP_HAL::BinarySemaphore *handle) override;

    /**
     * @brief Retrieve formatted CAN statistics as text
     * 
     * @details Generates a human-readable text report of CAN interface statistics
     *          including TX/RX frame counts, error counts, buffer overruns, and
     *          queue utilization. This is used by the @SYS filesystem to provide
     *          runtime diagnostics via @SYS/can0_stats.txt and @SYS/can1_stats.txt.
     *          
     *          Statistics Included:
     *          - Total frames transmitted/received
     *          - TX/RX buffer overrun counts
     *          - Total error count
     *          - Current queue depths
     *          - Frames lost due to full queues
     *          
     *          Useful for:
     *          - Debugging CAN communication issues
     *          - Monitoring bus utilization in SITL
     *          - Detecting buffer overflow conditions
     * 
     * @param[out] str ExpandingString to append statistics text to
     * 
     * @note Output format is plain text suitable for logging or display
     * @see get_statistics() for structured statistics data
     */
    void get_stats(ExpandingString &str) override;

    /**
     * @brief Get pointer to structured CAN bus statistics
     * 
     * @details Returns a pointer to the internal bus_stats_t structure containing
     *          detailed CAN interface statistics. This provides programmatic access
     *          to the same data available via get_stats() but in structured form.
     *          
     *          Statistics Structure (bus_stats_t):
     *          - tx_requests: Total send() calls
     *          - tx_confirmed: Frames successfully transmitted
     *          - tx_aborted: Frames aborted (not applicable in SITL)
     *          - tx_rejected: Frames rejected due to full queue
     *          - rx_received: Total frames received
     *          - rx_errors: RX errors (buffer overruns, etc.)
     *          - num_busoff_err: Bus-off events (always 0 in SITL)
     * 
     * @return Pointer to internal statistics structure (valid until interface destroyed)
     * 
     * @note Pointer remains valid for the lifetime of the CANIface object
     * @warning Do not modify the returned structure - it is for read-only access
     * @see get_stats() for formatted text output
     */
    const bus_stats_t *get_statistics(void) const override {
        return &stats;
    }
    
private:
    /**
     * @brief Write pending frames from TX queue to transport layer
     * 
     * @details Dequeues frames from _tx_queue and sends them via the transport
     *          layer (socket). Called by select() and _poll() when the socket
     *          is ready for writing. Continues until queue is empty or socket
     *          would block.
     * 
     * @note Updates _frames_in_socket_tx_queue and statistics counters
     */
    void _pollWrite();

    /**
     * @brief Read available frames from transport layer into RX queue
     * 
     * @details Receives frames from the transport layer (socket) and enqueues
     *          them in _rx_queue. Called by select() and _poll() when the socket
     *          has data available. Continues until no more frames available or
     *          RX queue is full.
     * 
     * @return true if at least one frame was received, false otherwise
     * 
     * @note Updates statistics counters and timestamps received frames
     */
    bool _pollRead();

    /**
     * @brief Confirm successful transmission of a frame
     * 
     * @details Decrements _frames_in_socket_tx_queue counter to track frames
     *          that have been fully processed by the transport layer. This
     *          allows send() to determine if more frames can be queued.
     */
    void _confirmSentFrame();

    /**
     * @brief Check if TX queue has frames ready to send
     * 
     * @return true if TX queue is not empty and transport can accept frames
     */
    bool _hasReadyTx();

    /**
     * @brief Check if RX queue has frames available
     * 
     * @return true if RX queue is not empty
     */
    bool _hasReadyRx();

    /**
     * @brief Perform I/O polling on transport socket
     * 
     * @details Internal polling implementation that calls _pollRead() and/or
     *          _pollWrite() based on requested operations. Used by select()
     *          to handle socket I/O after poll() returns ready.
     * 
     * @param[in] read  Perform read operations if true
     * @param[in] write Perform write operations if true
     */
    void _poll(bool read, bool write);

    /**
     * @brief Open socket for CAN communication (unused in current implementation)
     * 
     * @param[in] iface_name Name of CAN interface (e.g., "can0")
     * 
     * @return File descriptor or negative on error
     * 
     * @note Socket creation is now handled by CAN_Transport
     */
    int _openSocket(const std::string& iface_name);

    /**
     * @brief Update interface status based on poll results
     * 
     * @details Examines poll file descriptor flags (POLLERR, POLLHUP, POLLNVAL)
     *          to detect socket errors or disconnection. Updates error counters
     *          in statistics.
     * 
     * @param[in] pfd Poll file descriptor structure from poll() result
     */
    void _updateDownStatusFromPollResult(const pollfd& pfd);

    CAN_Transport *transport; ///< Transport layer (Multicast or SocketCAN backend)

    const uint8_t _self_index; ///< Interface index (0=CAN0, 1=CAN1, etc.)

    unsigned _frames_in_socket_tx_queue; ///< Count of frames queued in transport layer
    uint32_t _tx_frame_counter; ///< Total frames transmitted (for statistics)
    AP_HAL::BinarySemaphore *sem_handle; ///< Optional semaphore for event notification

    pollfd _pollfd; ///< Poll file descriptor for select() operations
    ObjectArray<CanTxItem> _tx_queue{100}; ///< Transmit queue (configurable depth)
    ObjectArray<CanRxItem> _rx_queue{100}; ///< Receive queue (configurable depth)

    /**
     * Bus statistics tracking TX/RX counts, errors, and overruns
     */
    AP_HAL::CANIface::bus_stats_t stats;

    HAL_Semaphore sem; ///< Mutex protecting TX/RX queue access

    /**
     * @brief Add a received frame to the RX queue (internal interface)
     * 
     * @param[in] rx_item Received frame with timestamp and flags
     * 
     * @return true always (queue push always succeeds or overwrites)
     * 
     * @note This is called by transport layer when frames arrive
     */
    bool add_to_rx_queue(const CanRxItem &rx_item) override {
        _rx_queue.push(rx_item);
        return true;
    }

    /**
     * @brief Get the interface number for this instance
     * 
     * @return Interface index (0=CAN0, 1=CAN1, etc.)
     */
    int8_t get_iface_num(void) const override {
        return _self_index;
    }
};

}

#endif //#if HAL_NUM_CAN_IFACES
