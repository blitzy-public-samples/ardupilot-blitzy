/**
 * @file AP_Canard_iface.h
 * @brief DroneCAN Canard Interface Adapter Layer
 * 
 * @details This file defines the CanardInterface class which provides an adapter
 *          layer between the libcanard library and ArduPilot's HAL CAN interface.
 *          It manages the low-level CAN frame transmission and reception, handles
 *          multiple CAN interface support, and provides thread-safe message queuing
 *          for DroneCAN protocol communication.
 * 
 *          The adapter implements the Canard::Interface abstract interface, allowing
 *          the DroneCAN stack to work with ArduPilot's hardware abstraction layer
 *          across different platforms (ChibiOS, Linux, SITL, etc.).
 * 
 *          Key features:
 *          - Multi-interface CAN support (up to HAL_NUM_CAN_IFACES)
 *          - Thread-safe transmission and reception with semaphore protection
 *          - Support for both 29-bit UAVCAN frames and 11-bit auxiliary frames
 *          - CAN-FD support (when CANARD_ENABLE_CANFD is defined)
 *          - Protocol statistics tracking for monitoring
 *          - Raw command prioritization for ESC control
 * 
 * @note This implementation requires HAL_ENABLE_DRONECAN_DRIVERS to be defined
 * 
 * Source: libraries/AP_DroneCAN/AP_Canard_iface.h
 */

#pragma once
#include <AP_HAL/AP_HAL.h>
#if HAL_ENABLE_DRONECAN_DRIVERS
#include <canard/interface.h>
#include <dronecan_msgs.h>

class AP_DroneCAN;
class CANSensor;

/**
 * @class CanardInterface
 * @brief Adapter interface between libcanard and ArduPilot HAL CAN drivers
 * 
 * @details CanardInterface provides the bridge between the libcanard DroneCAN/UAVCAN
 *          protocol stack and ArduPilot's hardware abstraction layer (HAL) CAN
 *          interfaces. It manages the lifecycle of the CanardInstance, handles
 *          frame transmission and reception, and provides thread-safe access to
 *          the CAN buses.
 * 
 *          The class implements the adapter pattern, converting between:
 *          - Canard::Transfer objects and raw CAN frames
 *          - Canard callbacks and ArduPilot HAL callbacks
 *          - libcanard memory management and ArduPilot memory arenas
 * 
 *          Lifecycle:
 *          1. Construction: Create instance with driver index
 *          2. init(): Initialize with memory arena and set node ID
 *          3. add_interface(): Register one or more HAL CAN interfaces
 *          4. Message handling: broadcast/request/respond for outgoing messages
 *          5. Processing: processTx/processRx/process for message pump
 * 
 *          Multi-Interface Support:
 *          The interface can manage up to HAL_NUM_CAN_IFACES CAN buses simultaneously,
 *          with automatic iface_mask management for selective transmission. This
 *          enables redundant CAN bus configurations for improved reliability.
 * 
 *          Thread Safety:
 *          All public methods are protected by internal semaphores (_sem_tx for
 *          transmission, _sem_rx for reception) to ensure thread-safe operation
 *          when called from multiple tasks (e.g., main loop, IO thread).
 * 
 *          Auxiliary Driver Support:
 *          In addition to standard 29-bit UAVCAN frames, the interface supports
 *          11-bit auxiliary frames via add_11bit_driver() for non-UAVCAN devices
 *          (e.g., proprietary ESCs, sensors) sharing the same CAN bus.
 * 
 * @note CAN-FD support is enabled when CANARD_ENABLE_CANFD is defined
 * @note Multi-interface mode requires CANARD_MULTI_IFACE to be defined
 * 
 * @warning This class is NOT thread-safe by default. Callers must ensure proper
 *          synchronization when accessing shared resources. Internal semaphores
 *          protect only the transmission and reception queues.
 * 
 * @see AP_DroneCAN for the high-level DroneCAN driver implementation
 * @see Canard::Interface for the abstract interface definition
 */
class CanardInterface : public Canard::Interface {
    friend class AP_DroneCAN;
public:

    /**
     * @brief Delete copy constructor - CanardInterface is not copyable
     * 
     * @details Copying a CanardInterface would duplicate HAL interface pointers
     *          and semaphore state, leading to undefined behavior. The interface
     *          manages hardware resources that must have a single owner.
     */
    CanardInterface(const CanardInterface&) = delete;
    
    /**
     * @brief Delete assignment operator - CanardInterface is not assignable
     * 
     * @details Assignment would duplicate HAL interface pointers and semaphore
     *          state, leading to undefined behavior.
     */
    CanardInterface& operator=(const CanardInterface&) = delete;

    /**
     * @brief Construct a new CanardInterface adapter
     * 
     * @details Creates an uninitialized CanardInterface. After construction,
     *          init() must be called to allocate the memory arena and set the
     *          node ID before the interface can be used.
     * 
     * @param[in] driver_index Index of this DroneCAN driver (0-based), used for
     *                         identifying the interface in multi-driver configurations
     * 
     * @note The interface is not functional until init() is called
     * @see init() for initialization
     */
    CanardInterface(uint8_t driver_index);

    /**
     * @brief Initialize the CanardInterface with memory arena and node ID
     * 
     * @details Initializes the underlying CanardInstance with the provided memory
     *          arena for dynamic frame allocation and sets the local node ID for
     *          this interface. This must be called before any message handling
     *          operations.
     * 
     *          The memory arena is used by libcanard for:
     *          - Allocating transmission queue entries
     *          - Buffering multi-frame transfers
     *          - Managing reception state
     * 
     *          Memory requirements depend on:
     *          - Number of concurrent transfers
     *          - Maximum transfer payload sizes
     *          - Number of subscribed message types
     *          
     *          Typical arena size: 8192-16384 bytes
     * 
     * @param[in] mem_arena      Pointer to pre-allocated memory arena for libcanard
     * @param[in] mem_arena_size Size of memory arena in bytes
     * @param[in] node_id        DroneCAN node ID for this interface (1-127, 0 for anonymous)
     * 
     * @note Must be called exactly once before message handling
     * @note The memory arena must remain valid for the lifetime of this object
     * @warning Setting node_id to 0 enables anonymous mode (limited functionality)
     * 
     * @see canardInit() in libcanard for memory arena management details
     */
    void init(void* mem_arena, size_t mem_arena_size, uint8_t node_id);

    /**
     * @brief Broadcast a message to all nodes on the CAN bus
     * 
     * @details Queues a broadcast transfer for transmission to all listening nodes.
     *          Broadcast messages use the DroneCAN broadcast addressing scheme with
     *          CAN ID encoding the message type and priority. The message will be
     *          transmitted on all registered CAN interfaces (multi-interface support).
     * 
     *          The transfer is queued internally and will be transmitted by the next
     *          call to processTx(). Memory for the transfer is allocated from the
     *          memory arena provided to init().
     * 
     *          Common broadcast messages:
     *          - Node status (uavcan.protocol.NodeStatus)
     *          - Sensor data (uavcan.equipment.*)
     *          - Actuator commands (uavcan.equipment.esc.RawCommand)
     * 
     * @param[in] bcast_transfer Transfer object containing message type, payload,
     *                           and transfer metadata (priority, payload length)
     * 
     * @return true if the message was successfully queued for transmission
     * @return false if queue is full or memory allocation failed
     * 
     * @note This method is thread-safe (protected by _sem_tx)
     * @warning High-frequency broadcasts can saturate the CAN bus - respect timing requirements
     * 
     * @see processTx() for actual frame transmission
     * @see Canard::Transfer for transfer structure details
     */
    bool broadcast(const Canard::Transfer &bcast_transfer) override;

    /**
     * @brief Send a service request to a specific node
     * 
     * @details Queues a service request transfer for transmission to a specific
     *          destination node. Service requests use the DroneCAN service addressing
     *          scheme and expect a response from the destination node.
     * 
     *          The transfer is queued internally with a unique transfer ID for
     *          response matching. The caller should wait for the response via
     *          the onTransferReception callback.
     * 
     *          Common service requests:
     *          - Parameter access (uavcan.protocol.param.GetSet)
     *          - File operations (uavcan.protocol.file.*)
     *          - Node info (uavcan.protocol.GetNodeInfo)
     * 
     * @param[in] destination_node_id Node ID of the target node (1-127)
     * @param[in] req_transfer        Transfer object containing service request data
     * 
     * @return true if the request was successfully queued for transmission
     * @return false if queue is full, memory allocation failed, or invalid node ID
     * 
     * @note This method is thread-safe (protected by _sem_tx)
     * @note Service requests timeout if no response is received (handled by caller)
     * @warning destination_node_id must be valid (1-127), 0 and 255 are reserved
     * 
     * @see respond() for sending service responses
     * @see onTransferReception() for receiving responses
     */
    bool request(uint8_t destination_node_id, const Canard::Transfer &req_transfer) override;

    /**
     * @brief Send a service response to a requesting node
     * 
     * @details Queues a service response transfer in reply to a previously received
     *          service request. The response is matched to the request using the
     *          transfer ID from the original request transfer.
     * 
     *          Responses must be sent within the DroneCAN service timeout window
     *          (typically 1 second) or the requesting node will consider the
     *          request failed.
     * 
     *          Common service responses:
     *          - Parameter values (uavcan.protocol.param.GetSet response)
     *          - Node information (uavcan.protocol.GetNodeInfo response)
     *          - File read data (uavcan.protocol.file.Read response)
     * 
     * @param[in] destination_node_id Node ID of the requesting node (1-127)
     * @param[in] res_transfer        Transfer object containing service response data,
     *                                including the transfer_id from the request
     * 
     * @return true if the response was successfully queued for transmission
     * @return false if queue is full, memory allocation failed, or invalid node ID
     * 
     * @note This method is thread-safe (protected by _sem_tx)
     * @note The transfer_id in res_transfer must match the original request
     * @warning Response must be sent before the service timeout expires
     * 
     * @see request() for sending service requests
     * @see onTransferReception() for receiving requests
     */
    bool respond(uint8_t destination_node_id, const Canard::Transfer &res_transfer) override;

    /**
     * @brief Process transmission queue and send pending CAN frames
     * 
     * @details Processes the libcanard transmission queue and sends queued frames
     *          to the HAL CAN interface(s). This method should be called periodically
     *          (typically at 1kHz) to ensure timely transmission of queued messages.
     * 
     *          The method:
     *          1. Retrieves frames from the libcanard transmission queue
     *          2. Applies iface_mask to select target interfaces
     *          3. Writes frames to HAL CAN interface(s)
     *          4. Removes successfully transmitted frames from queue
     * 
     *          Raw Commands Only Mode:
     *          When raw_commands_only=true, only high-priority ESC command frames
     *          (uavcan.equipment.esc.RawCommand) are transmitted. This mode is used
     *          for time-critical motor control in safety-critical flight phases,
     *          ensuring ESC commands are not delayed by lower-priority messages.
     * 
     * @param[in] raw_commands_only If true, transmit only ESC RawCommand messages;
     *                              if false, transmit all queued messages normally
     * 
     * @note This method is thread-safe (protected by _sem_tx)
     * @note Transmission failures are silently dropped (bus saturation handling)
     * @warning Calling with raw_commands_only=true will delay all other message types
     * 
     * @see broadcast(), request(), respond() for queueing messages
     * @see process() for combined TX/RX processing
     */
    void processTx(bool raw_commands_only);
    
    /**
     * @brief Process received CAN frames and dispatch to handlers
     * 
     * @details Reads incoming CAN frames from all registered HAL interfaces and
     *          passes them to libcanard for reassembly into complete transfers.
     *          When a complete transfer is received, the onTransferReception
     *          callback is invoked for message dispatching.
     * 
     *          The method:
     *          1. Polls all registered CAN interfaces for new frames
     *          2. Feeds frames to libcanard (canardHandleRxFrame)
     *          3. Updates reception statistics (protocol_stats)
     *          4. Triggers onTransferReception for complete transfers
     * 
     *          Multi-frame transfers are automatically reassembled by libcanard
     *          using the internal memory arena. CRC validation and transfer ID
     *          management are handled transparently.
     * 
     *          Error Handling:
     *          - Out-of-memory: Increments rx_error_oom counter, frame dropped
     *          - CRC errors: Increments rx_error_bad_crc counter, transfer dropped
     *          - Invalid frames: Silently ignored
     * 
     * @note This method is thread-safe (protected by _sem_rx)
     * @note Should be called frequently (1kHz recommended) to minimize latency
     * @note Reception errors are tracked in protocol_stats for monitoring
     * 
     * @see onTransferReception() for transfer dispatch callback
     * @see update_rx_protocol_stats() for statistics tracking
     * @see process() for combined TX/RX processing
     */
    void processRx();

    /**
     * @brief Combined processing loop for TX and RX with timeout
     * 
     * @details Convenience method that performs both transmission and reception
     *          processing in a single call, with a specified duration timeout.
     *          This method repeatedly calls processTx() and processRx() until
     *          the timeout expires.
     * 
     *          Typical usage in main loop:
     *          - Call with small duration (1-2ms) each loop iteration
     *          - Ensures regular processing without blocking
     * 
     *          The method processes:
     *          - All pending transmission frames
     *          - All available received frames
     *          - Updates protocol statistics
     * 
     * @param[in] duration Maximum processing duration in microseconds
     * 
     * @note This method is thread-safe (both _sem_tx and _sem_rx used internally)
     * @note Actual processing time may be less than duration if queues are empty
     * @note Does not use raw_commands_only filter (all messages transmitted)
     * 
     * @see processTx() for transmission-only processing
     * @see processRx() for reception-only processing
     */
    void process(uint32_t duration);

    /**
     * @brief Callback invoked when a complete transfer is received
     * 
     * @details Static callback function registered with libcanard that is invoked
     *          when a complete DroneCAN transfer has been received and validated.
     *          This function dispatches the transfer to the appropriate AP_DroneCAN
     *          instance for message handling.
     * 
     *          The callback:
     *          1. Locates the CanardInterface instance from CanardInstance pointer
     *          2. Validates transfer integrity (CRC, transfer ID)
     *          3. Dispatches to AP_DroneCAN message handler
     *          4. Updates reception statistics
     * 
     *          This callback is invoked from processRx() context and holds _sem_rx
     *          during execution. Message handlers must be efficient to avoid blocking
     *          the reception loop.
     * 
     * @param[in] ins      Pointer to the CanardInstance that received the transfer
     * @param[in] transfer Pointer to the complete received transfer containing:
     *                     - data_type_id: DroneCAN message type ID
     *                     - payload: Message payload bytes
     *                     - payload_len: Payload length in bytes
     *                     - source_node_id: Sender node ID
     *                     - transfer_id: Transfer sequence number
     * 
     * @note This is a static callback - must not access instance members without proper locking
     * @note Called from processRx() context with _sem_rx held
     * @warning Message handlers should execute quickly to avoid reception delays
     * 
     * @see processRx() for reception processing loop
     * @see shouldAcceptTransfer() for subscription filtering
     */
    static void onTransferReception(CanardInstance* ins, CanardRxTransfer* transfer);
    
    /**
     * @brief Callback to determine if a transfer should be accepted
     * 
     * @details Static callback function registered with libcanard that filters
     *          incoming transfers based on message type, transfer type, and source
     *          node. This callback allows selective subscription to DroneCAN
     *          messages, reducing CPU and memory usage by ignoring unwanted messages.
     * 
     *          The callback:
     *          1. Checks if the data_type_id is subscribed by AP_DroneCAN
     *          2. Validates the transfer_type (broadcast vs service)
     *          3. Optionally filters by source_node_id
     *          4. Returns expected data type signature for validation
     * 
     *          Filtering criteria:
     *          - Message type subscriptions (configured via AP_DroneCAN)
     *          - Transfer type (message, request, response)
     *          - Source node restrictions (if applicable)
     * 
     * @param[in]  ins                    Pointer to the CanardInstance
     * @param[out] out_data_type_signature Output parameter for expected message signature
     * @param[in]  data_type_id           DroneCAN data type ID of the transfer
     * @param[in]  transfer_type          Transfer type (broadcast, request, response)
     * @param[in]  source_node_id         Node ID of the sender
     * 
     * @return true if the transfer should be accepted and processed
     * @return false if the transfer should be ignored (not subscribed)
     * 
     * @note This is a static callback - must not access instance members
     * @note Called very frequently (for every CAN frame) - must be efficient
     * @note out_data_type_signature is used for DSDL message validation
     * 
     * @see onTransferReception() for accepted transfer handling
     * @see canardShouldAcceptTransfer() in libcanard
     */
    static bool shouldAcceptTransfer(const CanardInstance* ins,
                                     uint64_t* out_data_type_signature,
                                     uint16_t data_type_id,
                                     CanardTransferType transfer_type,
                                     uint8_t source_node_id);

    /**
     * @brief Register a HAL CAN interface for message transmission/reception
     * 
     * @details Adds a hardware CAN interface to this CanardInterface for message
     *          handling. Multiple interfaces can be added (up to HAL_NUM_CAN_IFACES)
     *          to support redundant CAN bus configurations. When multiple interfaces
     *          are registered, transmitted messages are sent on all interfaces
     *          (unless filtered by iface_mask).
     * 
     *          The interface must be properly initialized by the HAL before being
     *          added to the CanardInterface. After addition, the interface will
     *          be used for both transmission (processTx) and reception (processRx).
     * 
     *          Multi-Interface Configuration:
     *          - CAN1 + CAN2: Full redundancy with automatic failover
     *          - Separate message routing per interface (iface_mask control)
     *          - Independent error counters per interface
     * 
     * @param[in] can_drv Pointer to initialized HAL CAN interface (ChibiOS, Linux, etc.)
     * 
     * @return true if interface was successfully added
     * @return false if maximum interface count reached or can_drv is null
     * 
     * @note Must be called after init() and before message processing
     * @note The HAL interface must remain valid for the lifetime of CanardInterface
     * @warning Adding interfaces during active message processing may cause frame loss
     * 
     * @see HAL_NUM_CAN_IFACES for maximum interface count
     * @see AP_HAL::CANIface for HAL interface requirements
     */
    bool add_interface(AP_HAL::CANIface *can_drv);

    /**
     * @brief Register an auxiliary driver for 11-bit CAN frame handling
     * 
     * @details Adds support for non-UAVCAN devices that use 11-bit (standard)
     *          CAN identifiers instead of the 29-bit (extended) identifiers used
     *          by DroneCAN/UAVCAN. This allows proprietary sensors, ESCs, or other
     *          devices to share the same CAN bus as DroneCAN traffic.
     * 
     *          Auxiliary drivers:
     *          - Use 11-bit CAN IDs (0x000-0x7FF)
     *          - Bypass DroneCAN protocol processing
     *          - Receive raw CAN frames directly
     *          - Send frames via write_aux_frame()
     * 
     *          Common auxiliary devices:
     *          - Proprietary ESCs with custom protocols
     *          - Non-UAVCAN sensors (some magnetometers, GPS)
     *          - Legacy CAN devices
     * 
     *          Only one auxiliary driver is supported per CanardInterface to avoid
     *          frame routing ambiguity.
     * 
     * @param[in] sensor Pointer to CANSensor implementing 11-bit frame handling
     * 
     * @return true if auxiliary driver was successfully registered
     * @return false if an auxiliary driver is already registered or sensor is null
     * 
     * @note 11-bit frames do not conflict with 29-bit UAVCAN frames (different ID spaces)
     * @note Only one auxiliary driver per interface (single registration)
     * @warning Auxiliary frames are not protected by DroneCAN CRC or transfer IDs
     * 
     * @see write_aux_frame() for sending auxiliary frames
     * @see CANSensor for auxiliary driver interface requirements
     */
    bool add_11bit_driver(CANSensor *sensor);

    /**
     * @brief Transmit a CAN frame for auxiliary (non-UAVCAN) drivers
     * 
     * @details Sends a raw CAN frame on behalf of an auxiliary 11-bit driver.
     *          This bypasses the DroneCAN protocol stack and writes the frame
     *          directly to the HAL CAN interface(s). Used by auxiliary drivers
     *          for proprietary protocols that coexist with DroneCAN traffic.
     * 
     *          The frame:
     *          - Must use 11-bit CAN ID (standard identifier)
     *          - Bypasses libcanard transmission queue
     *          - Sent with specified timeout
     *          - Transmitted on all registered interfaces
     * 
     *          Typical usage:
     *          - Auxiliary driver calls this from its update loop
     *          - Frame contains device-specific protocol data
     *          - Timeout prevents blocking on bus saturation
     * 
     * @param[in,out] out_frame  CAN frame to transmit (11-bit ID, payload, DLC)
     * @param[in]     timeout_us Transmission timeout in microseconds
     * 
     * @return true if frame was successfully transmitted on at least one interface
     * @return false if transmission failed on all interfaces or timeout expired
     * 
     * @note This method is thread-safe (protected by _sem_tx)
     * @note Auxiliary frames do not participate in DroneCAN priority arbitration
     * @warning High-frequency auxiliary frames can interfere with DroneCAN traffic
     * @warning Timeout of 0 can block indefinitely on a saturated bus
     * 
     * @see add_11bit_driver() for registering auxiliary drivers
     * @see AP_HAL::CANFrame for frame structure
     */
    bool write_aux_frame(AP_HAL::CANFrame &out_frame, const uint32_t timeout_us);
    
#if AP_TEST_DRONECAN_DRIVERS
    /**
     * @brief Get test interface instance for unit testing
     * 
     * @details Returns a reference to the static test CanardInterface used for
     *          unit testing DroneCAN driver implementations. This interface is
     *          only available when AP_TEST_DRONECAN_DRIVERS is defined (test builds).
     * 
     * @return Reference to static test interface instance
     * 
     * @note Only available in test builds (AP_TEST_DRONECAN_DRIVERS defined)
     * @note Test interface uses simulated CAN buses, not real hardware
     * 
     * @see processTestRx() for test reception processing
     */
    static CanardInterface& get_test_iface() { return test_iface; }
    
    /**
     * @brief Process received frames for test interface
     * 
     * @details Processes reception for all test CanardInterface instances,
     *          used by unit tests to simulate CAN frame reception without
     *          real hardware. Only available in test builds.
     * 
     * @note Only available in test builds (AP_TEST_DRONECAN_DRIVERS defined)
     * @see get_test_iface() for accessing test interface
     */
    static void processTestRx();
#endif

    /**
     * @brief Update reception protocol statistics counters
     * 
     * @details Updates the DroneCAN protocol statistics structure with the result
     *          of a frame reception attempt. This tracks reception health metrics
     *          including successful frames, out-of-memory errors, CRC errors, and
     *          other protocol-level statistics.
     * 
     *          Statistics tracked:
     *          - rx_error_oom: Memory allocation failures during reception
     *          - rx_error_bad_crc: Transfers with invalid CRC
     *          - Successful reception count
     *          - Transfer type counters
     * 
     *          These statistics are accessible via MAVLink for monitoring CAN
     *          bus health and diagnosing communication issues.
     * 
     * @param[in] res Result code from libcanard frame processing:
     *                - Positive: Successful reception
     *                - Negative: Error code (see libcanard error definitions)
     * 
     * @note Called internally by processRx() for each received frame
     * @note Statistics are cumulative (not reset automatically)
     * 
     * @see protocol_stats for statistics structure
     * @see processRx() for reception processing loop
     */
    void update_rx_protocol_stats(int16_t res);

    /**
     * @brief Get the DroneCAN node ID for this interface
     * 
     * @details Returns the node ID that was set during init(). This is used
     *          for message addressing and identification on the CAN bus.
     * 
     * @return uint8_t Node ID (1-127), or 0 if in anonymous mode
     * 
     * @note Node ID is set during init() and cannot be changed afterwards
     * @note Override of Canard::Interface::get_node_id()
     */
    uint8_t get_node_id() const override { return canard.node_id; }

    /**
     * @brief Get semaphore held during message reception
     * 
     * @details Returns a reference to the reception semaphore (_sem_rx) that is
     *          held during processRx() and onTransferReception(). External code
     *          that needs to access reception state synchronously can use this
     *          semaphore to ensure mutual exclusion.
     * 
     *          Typical usage:
     *          - External handler needs to access received message data
     *          - Must take _sem_rx before accessing reception buffers
     *          - Prevents race conditions with ongoing reception
     * 
     * @return Reference to reception semaphore
     * 
     * @warning Holding this semaphore for extended periods blocks reception processing
     * @note Transmission uses a separate semaphore (_sem_tx)
     * 
     * @see HAL_Semaphore for semaphore usage patterns
     */
    HAL_Semaphore &get_sem_rx(void) { return _sem_rx; }

private:
    // ========================================================================
    // Private Members - Internal State
    // ========================================================================
    
    /**
     * @brief Core libcanard instance managing protocol state
     * 
     * @details The CanardInstance contains:
     *          - Memory arena for dynamic allocations
     *          - Transmission queue (linked list of pending frames)
     *          - Reception state machines (per transfer type and node)
     *          - Node ID configuration
     *          - Transfer ID counters
     * 
     * Lifecycle: Initialized by init(), used throughout message processing
     */
    CanardInstance canard;
    
    /**
     * @brief Array of registered HAL CAN interfaces
     * 
     * @details Supports up to HAL_NUM_CAN_IFACES interfaces for redundant
     *          CAN bus configurations. Each interface represents a physical
     *          CAN controller (e.g., CAN1, CAN2 on flight controller).
     * 
     * @note num_ifaces tracks the actual number of registered interfaces
     * @see add_interface() for interface registration
     */
    AP_HAL::CANIface* ifaces[HAL_NUM_CAN_IFACES];
    
#if AP_TEST_DRONECAN_DRIVERS
    /**
     * @brief Array of test interfaces for unit testing
     * @note Only available when AP_TEST_DRONECAN_DRIVERS is defined
     */
    static CanardInterface* canard_ifaces[3];
    
    /**
     * @brief Static test interface instance
     * @note Only available when AP_TEST_DRONECAN_DRIVERS is defined
     */
    static CanardInterface test_iface;
#endif
    
    /**
     * @brief Number of registered CAN interfaces
     * @details Valid range: 0 to HAL_NUM_CAN_IFACES
     * @see add_interface() for registration
     */
    uint8_t num_ifaces;
    
    /**
     * @brief Binary semaphore for transfer handling synchronization
     * @details Used internally by libcanard for multi-threaded transfer management
     */
    HAL_BinarySemaphore sem_handle;
    
    /**
     * @brief Initialization state flag
     * @details true after init() has been called, false otherwise
     * @see init() for initialization
     */
    bool initialized;
    
    /**
     * @brief Semaphore protecting transmission queue
     * 
     * @details This semaphore ensures thread-safe access to the transmission
     *          queue when multiple tasks call broadcast/request/respond/processTx.
     *          Must be held when:
     *          - Queueing new transfers (broadcast, request, respond)
     *          - Processing transmission queue (processTx)
     *          - Accessing tx_transfer state
     * 
     * @warning Holding this semaphore blocks all transmission operations
     * @see broadcast(), request(), respond(), processTx()
     */
    HAL_Semaphore _sem_tx;
    
    /**
     * @brief Semaphore protecting reception state
     * 
     * @details This semaphore ensures thread-safe access to reception state
     *          and callbacks when multiple tasks interact with received messages.
     *          Must be held when:
     *          - Processing received frames (processRx)
     *          - Dispatching transfers (onTransferReception)
     *          - Accessing reception statistics
     * 
     * @warning Holding this semaphore blocks all reception operations
     * @see processRx(), onTransferReception(), get_sem_rx()
     */
    HAL_Semaphore _sem_rx;
    
    /**
     * @brief Transfer object for transmission operations
     * @details Reusable transfer structure for building outgoing messages
     */
    CanardTxTransfer tx_transfer;
    
    /**
     * @brief DroneCAN protocol statistics counters
     * 
     * @details Tracks protocol-level statistics including:
     *          - rx_error_oom: Out-of-memory errors during reception
     *          - rx_error_bad_crc: CRC validation failures
     *          - tx_success: Successfully transmitted frames
     *          - rx_success: Successfully received transfers
     *          - Transfer type counters (broadcast, request, response)
     * 
     * These statistics are accessible via MAVLink for monitoring and diagnostics.
     * 
     * @see update_rx_protocol_stats() for statistics updates
     */
    dronecan_protocol_Stats protocol_stats;

    /**
     * @brief Auxiliary driver for 11-bit CAN frames
     * 
     * @details Pointer to optional CANSensor driver handling non-UAVCAN devices
     *          that use 11-bit (standard) CAN identifiers. Only one auxiliary
     *          driver is supported per interface to avoid routing ambiguity.
     * 
     * @note NULL if no auxiliary driver is registered
     * @see add_11bit_driver() for registration
     * @see write_aux_frame() for auxiliary frame transmission
     */
    CANSensor *aux_11bit_driver;
};
#endif // HAL_ENABLE_DRONECAN_DRIVERS
