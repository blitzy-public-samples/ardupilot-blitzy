/**
 * @file AP_DroneCAN_DNA_Server.h
 * @brief DroneCAN Dynamic Node Allocation (DNA) server implementation
 * 
 * @details This file implements the DroneCAN Dynamic Node Allocation protocol server,
 *          which manages the assignment of node IDs to devices on a DroneCAN network.
 *          The DNA server maintains a persistent database of node registrations that
 *          maps unique device IDs to assigned node IDs, enabling consistent node ID
 *          allocation across power cycles.
 * 
 *          Key responsibilities:
 *          - Process dynamic node allocation requests from devices without node IDs
 *          - Maintain persistent storage of node ID assignments using StorageManager
 *          - Verify registered nodes by requesting GetNodeInfo and validating unique IDs
 *          - Detect and report duplicate node IDs on the network
 *          - Monitor node health status and track active nodes
 * 
 *          The DNA protocol uses multi-part messages to transmit the full unique ID
 *          (up to 16 bytes) and allocates node IDs in the range 1-127. The server
 *          stores a hash of each unique ID for efficient storage and lookup.
 * 
 * @note The database uses StorageManager for persistent storage across reboots
 * @warning Database corruption or duplicate node detection will set unhealthy server state
 * 
 * @see https://dronecan.github.io/Specification/6._Application_level_functions/
 * @see AP_DroneCAN class for integration with ArduPilot CAN manager
 */
#pragma once
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>

#if HAL_ENABLE_DRONECAN_DRIVERS
#include <AP_Common/Bitmask.h>
#include <StorageManager/StorageManager.h>
#include <AP_CANManager/AP_CANManager.h>
#include <canard/publisher.h>
#include <canard/subscriber.h>
#include <canard/service_client.h>
#include "AP_Canard_iface.h"
#include <dronecan_msgs.h>

class AP_DroneCAN;
//Forward declaring classes

/**
 * @class AP_DroneCAN_DNA_Server
 * @brief DroneCAN Dynamic Node Allocation (DNA) protocol server
 * 
 * @details Implements the server side of the DroneCAN Dynamic Node Allocation protocol,
 *          which allows devices without pre-configured node IDs to request and receive
 *          unique node IDs from the server. The server maintains a persistent database
 *          of node registrations that survive power cycles.
 * 
 *          Server operation:
 *          1. Listens for allocation requests on uavcan.protocol.dynamic_node_id.Allocation
 *          2. Processes multi-part allocation messages to receive full unique IDs
 *          3. Checks database for existing registration or allocates new node ID
 *          4. Responds with allocated node ID in allocation message
 *          5. Monitors NodeStatus messages to track active nodes
 *          6. Periodically verifies registered nodes via GetNodeInfo requests
 *          7. Detects duplicate node IDs and unhealthy node conditions
 * 
 *          The unique ID hash is computed using FNV-1a hash algorithm and stored
 *          as 6 bytes with CRC protection. The allocation protocol follows the
 *          DroneCAN specification for multi-part message handling.
 * 
 *          Thread safety: The internal Database uses semaphores for thread-safe access
 *          allowing multiple DNA servers on different CAN interfaces.
 * 
 * @note One DNA server instance per DroneCAN interface (CAN bus)
 * @warning Duplicate node IDs set server_state to DUPLICATE_NODES and fail prearm
 * @warning Node verification failures set server_state to NODE_STATUS_UNHEALTHY
 */
class AP_DroneCAN_DNA_Server
{
    StorageAccess storage;

    /**
     * @struct NodeRecord
     * @brief Persistent storage record for a single node registration
     * 
     * @details Each node ID (1-127) has an associated NodeRecord stored in the database.
     *          The record contains a hash of the node's unique ID rather than the full
     *          16-byte unique ID to save storage space. The hash is computed using the
     *          FNV-1a (Fowler-Noll-Vo) hash algorithm which provides good distribution
     *          and low collision probability for unique IDs.
     * 
     *          Storage format:
     *          - uid_hash[6]: 48-bit hash of unique ID (FNV-1a algorithm)
     *          - crc: 8-bit CRC for detecting storage corruption
     * 
     *          A NodeRecord with all-zero uid_hash or invalid CRC indicates the node ID
     *          has no valid registration. CRC validation detects storage corruption from
     *          flash wear or power loss during writes.
     * 
     * @note The 6-byte hash provides 2^48 possible values, sufficient for collision-free
     *       hashing of the 127 possible node IDs in a DroneCAN network
     */
    struct NodeRecord {
        uint8_t uid_hash[6];  ///< FNV-1a hash of device unique ID (6 bytes)
        uint8_t crc;          ///< CRC8 for detecting record corruption
    };

    /**
     * @class Database
     * @brief Persistent storage and management of node ID registrations
     * 
     * @details The Database class provides thread-safe persistent storage of node
     *          registrations using the StorageManager. Each node ID (1 through MAX_NODE_ID)
     *          can have one registration consisting of a NodeRecord which contains the
     *          hash of the unique ID reported by that node ID.
     * 
     *          Physical storage layout:
     *          - Header: Magic number and format version
     *          - Records: Array of NodeRecords indexed by node ID (1-127)
     * 
     *          A NodeRecord with all-zero unique ID hash or invalid CRC indicates that
     *          node ID has no registration. The database validates CRC on read to detect
     *          corruption from flash wear or power loss during write operations.
     * 
     *          Thread safety: All public methods acquire a semaphore lock for their duration,
     *          allowing multiple DNA servers on different CAN interfaces to safely share
     *          the database.
     * 
     *          The database handles the server behavior for allocation and verification:
     *          - handle_allocation: Processes allocation requests and assigns node IDs
     *          - handle_node_info: Verifies node unique IDs match registrations
     *          - init_server: Registers the server's own node ID and unique ID
     * 
     * @note Uses StorageManager for persistent storage across reboots
     * @warning Database corruption detected via CRC will invalidate affected registrations
     */
    class Database {
    public:
        Database() {};

        /**
         * @brief Initialize the database with a storage accessor
         * 
         * @details Sets up the database to use the provided StorageManager accessor for
         *          persistent storage. The storage accessor is always replaced with the
         *          supplied one, allowing reinitialization if needed. This must be called
         *          before any other database operations.
         * 
         * @param[in] storage_ Pointer to StorageAccess for persistent storage operations
         * 
         * @note Must be called before any node registration operations
         */
        void init(StorageAccess *storage_);

        /**
         * @brief Remove all registrations from the database
         * 
         * @details Clears all node registrations from persistent storage by zeroing all
         *          NodeRecord entries. This operation is permanent and cannot be undone.
         *          Used for factory reset or when reinitializing the network.
         * 
         * @warning All registered node IDs will be forgotten and must re-register on next boot
         */
        void reset();

        /**
         * @brief Check if a node ID has a valid registration
         * 
         * @details Returns true if the specified node ID has a valid registration in the
         *          database (non-zero unique ID hash with valid CRC). This is a fast lookup
         *          using the node_registered bitmask without storage access.
         * 
         * @param[in] node_id Node ID to check (1-127)
         * 
         * @return true if node_id has a valid registration, false otherwise
         */
        bool is_registered(uint8_t node_id) {
            return node_registered.get(node_id);
        }

        /**
         * @brief Initialize the server's own node ID and unique ID registration
         * 
         * @details Registers the DNA server's own node ID and unique ID in the database
         *          to prevent allocating the server's node ID to other devices. This must
         *          be called during server initialization before processing any allocation
         *          requests.
         * 
         * @param[in] own_node_id The server's node ID (1-127)
         * @param[in] own_unique_id The server's unique device ID (up to 16 bytes)
         * @param[in] own_unique_id_len Length of own_unique_id in bytes (typically 16)
         * 
         * @note This prevents the server from allocating its own node ID to other nodes
         */
        void init_server(uint8_t own_node_id, const uint8_t own_unique_id[], uint8_t own_unique_id_len);

        /**
         * @brief Process a GetNodeInfo response and verify node registration
         * 
         * @details Handles the GetNodeInfo response during node verification. Checks if the
         *          unique ID in the response matches the stored registration for the source
         *          node ID. If a mismatch is detected, this indicates a duplicate node ID
         *          (two devices using the same node ID) which is a critical network error.
         * 
         * @param[in] source_node_id Node ID that sent the GetNodeInfo response (1-127)
         * @param[in] unique_id Unique device ID from GetNodeInfo response (16 bytes)
         * 
         * @return true if duplicate node detected (unique ID mismatch), false if verified OK
         * 
         * @warning A duplicate node condition indicates two devices are using the same node ID
         */
        bool handle_node_info(uint8_t source_node_id, const uint8_t unique_id[]);

        /**
         * @brief Process dynamic node allocation request and allocate node ID
         * 
         * @details Handles the complete allocation protocol sequence:
         *          1. Searches database for existing registration matching unique ID
         *          2. If found, returns the previously allocated node ID
         *          3. If not found, allocates next available node ID (1-127)
         *          4. Creates new registration with unique ID hash
         *          5. Returns 0 if allocation fails (no free node IDs)
         * 
         *          The allocation protocol uses multi-part messages to transmit the full
         *          unique ID (up to 16 bytes). This method is called after the complete
         *          unique ID has been received and assembled.
         * 
         * @param[in] unique_id Complete device unique ID (16 bytes) from allocation request
         * 
         * @return Allocated node ID (1-127) on success, 0 if allocation failed
         * 
         * @note Returns existing node ID if unique ID already registered
         * @note Thread-safe: acquires lock for duration of operation
         */
        uint8_t handle_allocation(const uint8_t unique_id[]);

    private:
        /**
         * @brief Find node ID with matching unique ID
         * 
         * @details Searches through all registered node IDs to find one with a unique ID
         *          hash matching the provided unique ID. Used during allocation to check
         *          if a device has been previously registered.
         * 
         * @param[in] unique_id Device unique ID to search for (up to 16 bytes)
         * @param[in] size Length of unique_id in bytes
         * 
         * @return Node ID (1-127) if found, 0 if no matching registration exists
         */
        uint8_t find_node_id(const uint8_t unique_id[], uint8_t size);

        /**
         * @brief Compute FNV-1a hash of unique ID
         * 
         * @details Fills the NodeRecord with a 6-byte hash of the unique ID computed using
         *          the FNV-1a (Fowler-Noll-Vo) hash algorithm. FNV-1a provides good hash
         *          distribution and low collision probability for device unique IDs.
         * 
         *          FNV-1a algorithm:
         *          hash = FNV_offset_basis
         *          for each byte:
         *              hash = hash XOR byte
         *              hash = hash * FNV_prime
         * 
         *          The 6-byte hash is then stored in the record along with a CRC for
         *          corruption detection.
         * 
         * @param[out] record NodeRecord to fill with computed hash
         * @param[in] unique_id Device unique ID to hash (up to 16 bytes)
         * @param[in] size Length of unique_id in bytes
         * 
         * @note FNV-1a chosen for simplicity, speed, and good distribution properties
         */
        void compute_uid_hash(NodeRecord &record, const uint8_t unique_id[], uint8_t size) const;

        /**
         * @brief Register unique ID to node ID, removing any conflicting registrations
         * 
         * @details Associates the given unique ID with the specified node ID. If the unique
         *          ID was previously registered to a different node ID, that old registration
         *          is deleted first to prevent conflicts.
         * 
         * @param[in] node_id Target node ID for registration (1-127)
         * @param[in] unique_id Device unique ID to register (up to 16 bytes)
         * @param[in] size Length of unique_id in bytes
         * 
         * @note Deletes any existing registration for the same unique ID at different node ID
         */
        void register_uid(uint8_t node_id, const uint8_t unique_id[], uint8_t size);

        /**
         * @brief Create new registration for node ID
         * 
         * @details Creates a new registration entry for the specified node ID with the given
         *          unique ID. Computes the hash, calculates CRC, and writes to persistent
         *          storage. Updates the node_registered bitmask.
         * 
         * @param[in] node_id Node ID to register (1-127)
         * @param[in] unique_id Device unique ID (up to 16 bytes)
         * @param[in] size Length of unique_id in bytes
         * 
         * @note Assumes node_id has no existing registration (caller should check)
         */
        void create_registration(uint8_t node_id, const uint8_t unique_id[], uint8_t size);

        /**
         * @brief Delete node ID registration
         * 
         * @details Removes the registration for the specified node ID by zeroing its
         *          NodeRecord in persistent storage and clearing the node_registered bitmask.
         *          This makes the node ID available for allocation to other devices.
         * 
         * @param[in] node_id Node ID to unregister (1-127)
         */
        void delete_registration(uint8_t node_id);

        /**
         * @brief Verify node ID has valid registration in storage
         * 
         * @details Reads the NodeRecord from persistent storage and validates that it has
         *          a non-zero unique ID hash and correct CRC. Returns true if the registration
         *          is valid and intact.
         * 
         * @param[in] node_id Node ID to check (1-127)
         * 
         * @return true if registration exists and CRC is valid, false otherwise
         * 
         * @note More thorough than is_registered() as it validates storage integrity
         */
        bool check_registration(uint8_t node_id);

        /**
         * @brief Read NodeRecord from persistent storage
         * 
         * @details Retrieves the NodeRecord for the specified node ID from StorageManager.
         *          Does not validate CRC - caller must check if needed.
         * 
         * @param[out] record NodeRecord structure to populate with stored data
         * @param[in] node_id Node ID whose record to read (1-127)
         */
        void read_record(NodeRecord &record, uint8_t node_id);

        /**
         * @brief Write NodeRecord to persistent storage
         * 
         * @details Writes the NodeRecord for the specified node ID to StorageManager.
         *          Assumes record already has valid uid_hash and CRC computed.
         * 
         * @param[in] record NodeRecord to write to storage
         * @param[in] node_id Node ID whose record to write (1-127)
         */
        void write_record(const NodeRecord &record, uint8_t node_id);

        /**
         * @brief Bitmask tracking which node IDs have valid registrations
         * 
         * @details Fast lookup bitmap where bit N indicates if node ID N has a valid
         *          registration in the database. Updated when registrations are created
         *          or deleted. Covers node IDs 1-127 (node ID 0 is reserved/invalid).
         */
        Bitmask<128> node_registered; // have a registration for this node ID

        StorageAccess *storage;  ///< Persistent storage accessor for node registrations
        HAL_Semaphore sem;       ///< Semaphore for thread-safe database access
    };

    /**
     * @brief Shared database instance for all DNA servers
     * 
     * @details Static database shared across all DNA server instances on different CAN
     *          interfaces. This allows consistent node ID allocation regardless of which
     *          CAN bus a device appears on. The database uses internal locking for
     *          thread-safe access from multiple servers.
     * 
     * @note Static to ensure single source of truth for node registrations across all CAN buses
     */
    static Database db;

    /**
     * @enum ServerState
     * @brief DNA server health and error states
     * 
     * @details Tracks the operational state of the DNA server for prearm checks and
     *          diagnostics. Negative values indicate error conditions that will fail
     *          prearm checks.
     */
    enum ServerState {
        NODE_STATUS_UNHEALTHY = -5,  ///< Node verification failed or node reports unhealthy status
        DUPLICATE_NODES = -2,        ///< Duplicate node IDs detected on network (critical error)
        HEALTHY = 0                  ///< All nodes verified and healthy
    };

    uint32_t last_verification_request;  ///< Timestamp (ms) of last GetNodeInfo request sent
    uint8_t curr_verifying_node;         ///< Node ID currently being verified (0 if none)
    uint8_t self_node_id;                ///< This server's own node ID
    bool nodeInfo_resp_rcvd;             ///< Flag indicating GetNodeInfo response received

    /**
     * @brief Bitmasks tracking node states for verification and monitoring
     * 
     * @details Each bitmask tracks a specific status for all possible node IDs (1-127).
     *          Bit N corresponds to node ID N. These bitmasks are used for:
     *          - node_verified: Node has been verified (GetNodeInfo matches registration)
     *          - node_seen: NodeStatus message received from this node ID
     *          - node_logged: Node information has been written to log file
     *          - node_healthy: Node reports healthy status in NodeStatus messages
     * 
     *          The verification process checks registered nodes against actual devices
     *          by requesting GetNodeInfo and comparing unique IDs. Mismatches indicate
     *          duplicate node IDs which is a critical network error.
     * 
     * @note Covers node IDs 1-127 (node ID 0 and >127 are invalid)
     */
    Bitmask<128> node_verified; // node seen and unique ID matches stored
    Bitmask<128> node_seen;     // received NodeStatus
    Bitmask<128> node_logged;   // written to log file
    Bitmask<128> node_healthy;  // reports healthy

    uint8_t last_logging_count;  ///< Count of logged nodes in last cycle

    /**
     * @brief Server error state and fault information
     * 
     * @details Tracks the current server state and information about any detected faults.
     *          When duplicate nodes or verification failures occur, the fault_node_id and
     *          fault_node_name are populated for diagnostic reporting.
     */
    enum ServerState server_state;  ///< Current server health state
    uint8_t fault_node_id;          ///< Node ID causing fault (if any)
    char fault_node_name[15];       ///< Name of faulting node from GetNodeInfo

    /**
     * @brief State variables for multi-part allocation message processing
     * 
     * @details The allocation protocol uses multi-part messages to transmit the full
     *          16-byte unique ID. These variables track the reassembly state:
     *          - rcvd_unique_id: Buffer accumulating unique ID bytes across messages
     *          - rcvd_unique_id_offset: Current position in rcvd_unique_id buffer
     *          - last_alloc_msg_ms: Timestamp of last allocation message (for timeout)
     * 
     *          If allocation messages stop arriving mid-sequence (timeout), the server
     *          resets and waits for a new allocation request sequence to begin.
     */
    uint8_t rcvd_unique_id[16];      ///< Buffer for assembling multi-part unique ID
    uint8_t rcvd_unique_id_offset;   ///< Current offset in unique ID buffer (0-16)
    uint32_t last_alloc_msg_ms;      ///< Timestamp (ms) of last allocation message

    AP_DroneCAN &_ap_dronecan;      ///< Reference to parent DroneCAN driver
    CanardInterface &_canard_iface; ///< Reference to Canard interface for this CAN bus

    /// Publisher for allocation responses
    Canard::Publisher<uavcan_protocol_dynamic_node_id_Allocation> allocation_pub{_canard_iface};

    /// Callback and subscriber for allocation requests
    Canard::ObjCallback<AP_DroneCAN_DNA_Server, uavcan_protocol_dynamic_node_id_Allocation> allocation_cb{this, &AP_DroneCAN_DNA_Server::handle_allocation};
    Canard::Subscriber<uavcan_protocol_dynamic_node_id_Allocation> allocation_sub;

    /// Callback and subscriber for node status monitoring
    Canard::ObjCallback<AP_DroneCAN_DNA_Server, uavcan_protocol_NodeStatus> node_status_cb{this, &AP_DroneCAN_DNA_Server::handleNodeStatus};
    Canard::Subscriber<uavcan_protocol_NodeStatus> node_status_sub;

    /// Callback and client for node verification via GetNodeInfo
    Canard::ObjCallback<AP_DroneCAN_DNA_Server, uavcan_protocol_GetNodeInfoResponse> node_info_cb{this, &AP_DroneCAN_DNA_Server::handleNodeInfo};
    Canard::Client<uavcan_protocol_GetNodeInfoResponse> node_info_client;

public:
    /**
     * @brief Constructor for DNA server
     * 
     * @details Creates a DNA server instance associated with a specific DroneCAN driver
     *          and CAN interface. Each CAN bus should have its own DNA server instance.
     * 
     * @param[in] ap_dronecan Reference to parent DroneCAN driver
     * @param[in] canard_iface Reference to Canard interface for this CAN bus
     * @param[in] driver_index Index of this DroneCAN driver (for logging/identification)
     */
    AP_DroneCAN_DNA_Server(AP_DroneCAN &ap_dronecan, CanardInterface &canard_iface, uint8_t driver_index);

    // Do not allow copies
    CLASS_NO_COPY(AP_DroneCAN_DNA_Server);

    /**
     * @brief Initialize DNA server with node ID and unique ID
     * 
     * @details Initializes the DNA server by:
     *          1. Setting up Canard publisher and subscribers for allocation protocol
     *          2. Registering server's own node ID and unique ID in database
     *          3. Preparing to handle allocation requests and monitor nodes
     * 
     *          Must be called before the server can process allocation requests.
     *          The server's own node ID is registered to prevent allocating it to
     *          other devices.
     * 
     * @param[in] own_unique_id Server's device unique ID (typically 16 bytes)
     * @param[in] own_unique_id_len Length of own_unique_id in bytes
     * @param[in] node_id Server's node ID on the CAN bus (1-127)
     * 
     * @return true if initialization successful, false on failure
     * 
     * @note Must be called after CAN interface is initialized
     */
    bool init(uint8_t own_unique_id[], uint8_t own_unique_id_len, uint8_t node_id);

    /**
     * @brief Perform prearm check of DNA server state
     * 
     * @details Checks the DNA server health state and returns whether the system is
     *          safe to arm. Fails prearm if:
     *          - DUPLICATE_NODES: Multiple devices using same node ID detected
     *          - NODE_STATUS_UNHEALTHY: Node verification failed or node unhealthy
     * 
     *          If prearm fails, fail_msg is populated with diagnostic information
     *          including the fault node ID and name.
     * 
     * @param[out] fail_msg Buffer for failure message (if any)
     * @param[in] fail_msg_len Size of fail_msg buffer
     * 
     * @return true if server healthy (safe to arm), false if errors detected
     * 
     * @warning Duplicate nodes indicate critical network misconfiguration
     * @note Called by ArduPilot prearm check system before arming
     */
    bool prearm_check(char* fail_msg, uint8_t fail_msg_len) const;

    /**
     * @brief Handle dynamic node allocation request message
     * 
     * @details Processes incoming allocation request messages from devices without node IDs.
     *          The allocation protocol uses multi-part messages to transmit the complete
     *          16-byte unique ID:
     * 
     *          1. First message: Contains first part of unique ID
     *          2. Subsequent messages: Continue transmitting unique ID
     *          3. Final message: Complete unique ID received, server responds
     * 
     *          Algorithm:
     *          - Accumulate unique ID bytes in rcvd_unique_id buffer
     *          - If message not marked as final, wait for more
     *          - If final message received, call db.handle_allocation()
     *          - Send allocation response with assigned node ID
     * 
     *          The database either returns existing node ID for known device or
     *          allocates new node ID from available pool (1-127).
     * 
     * @param[in] transfer Canard transfer information (timestamp, source, etc.)
     * @param[in] msg Allocation message containing unique ID fragment
     * 
     * @note Multi-part message sequence times out if messages stop arriving
     * @note Called by Canard when allocation messages received
     */
    void handle_allocation(const CanardRxTransfer& transfer, const uavcan_protocol_dynamic_node_id_Allocation& msg);

    /**
     * @brief Handle node status message
     * 
     * @details Processes NodeStatus messages broadcast periodically by all nodes.
     *          Tracks which nodes are active on the network and monitors their health.
     *          Updates node_seen, node_healthy, and node_logged bitmasks.
     * 
     *          For registered nodes not yet verified, adds them to verification queue.
     *          Nodes must be verified by requesting GetNodeInfo and confirming their
     *          unique ID matches the database registration.
     * 
     * @param[in] transfer Canard transfer information (timestamp, source node ID, etc.)
     * @param[in] msg NodeStatus message with health and uptime information
     * 
     * @note NodeStatus typically broadcast at 1 Hz by all nodes
     * @note Called by Canard when NodeStatus messages received
     */
    void handleNodeStatus(const CanardRxTransfer& transfer, const uavcan_protocol_NodeStatus& msg);

    /**
     * @brief Handle GetNodeInfo response for node verification
     * 
     * @details Processes GetNodeInfo responses during node verification. Compares the
     *          unique ID in the response against the database registration for that
     *          node ID. Three possible outcomes:
     * 
     *          1. Match: Node verified, sets node_verified bit
     *          2. Mismatch: Duplicate node detected, sets server_state to DUPLICATE_NODES
     *          3. Timeout: No response, node may be offline
     * 
     *          Duplicate node detection is critical - it means two devices are using
     *          the same node ID which will cause communication failures and data corruption.
     * 
     * @param[in] transfer Canard transfer information (timestamp, source node ID, etc.)
     * @param[in] rsp GetNodeInfo response with unique ID and node information
     * 
     * @warning Sets DUPLICATE_NODES state if unique ID mismatch detected
     * @note Called by Canard when GetNodeInfo responses received
     */
    void handleNodeInfo(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoResponse& rsp);

    /**
     * @brief Verify registered nodes by requesting GetNodeInfo
     * 
     * @details Periodically called to verify that registered nodes are present and have
     *          matching unique IDs. Cycles through all registered but unverified nodes,
     *          sending GetNodeInfo requests and waiting for responses.
     * 
     *          Verification process:
     *          1. Find next registered but unverified node
     *          2. Send GetNodeInfo request to that node
     *          3. Wait for response (handled by handleNodeInfo)
     *          4. Continue to next node after timeout or response
     * 
     *          This detects cases where:
     *          - Node is offline but was previously registered
     *          - Different device is using a registered node ID (duplicate)
     *          - Node ID collision from database corruption
     * 
     * @note Called periodically by main DNA server loop
     * @note Verification rate limited to avoid flooding network with requests
     */
    void verify_nodes();
};

#endif
