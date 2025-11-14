/**
 * @file AP_Scripting_helpers.h
 * @brief Helper classes and functions for Lua scripting integration
 * 
 * @details This file provides helper classes that enable efficient Lua script
 *          interaction with ArduPilot core functionality. The primary helpers are:
 *          
 *          - Parameter: Fast parameter access via cached pointers, avoiding repeated
 *            parameter tree lookups for improved script performance
 *          - DroneCAN_Handle: Interface for Lua scripts to send and receive DroneCAN
 *            (UAVCAN) messages for CAN bus communication with peripherals
 *          
 *          These helpers bridge the gap between Lua's dynamic typing and ArduPilot's
 *          strongly-typed C++ APIs, providing performance optimization through caching
 *          and simplified interfaces for complex subsystems.
 * 
 * @note These classes are specifically designed for use from Lua bindings and should
 *       not be used directly in C++ vehicle code
 * 
 * Source: libraries/AP_Scripting/AP_Scripting_helpers.h
 */

#pragma once

#include <AP_Param/AP_Param.h>
#include "lua/src/lua.hpp"
#include <AP_DroneCAN/AP_DroneCAN.h>

/**
 * @brief Lua binding helper to create new Parameter objects from Lua scripts
 * 
 * @param[in] L Lua state containing parameter name or initialization data
 * @return int Number of return values pushed to Lua stack (1 on success, 0 on failure)
 */
int lua_new_Parameter(lua_State *L);

/**
 * @class Parameter
 * @brief Fast parameter access via cached pointer for Lua scripts
 * 
 * @details This class provides high-performance parameter access for Lua scripts by
 *          caching the parameter pointer after initial lookup. This avoids repeated
 *          expensive parameter tree searches that would occur with direct parameter
 *          name lookups.
 *          
 *          Performance: Initial lookup via init() performs full parameter tree search
 *          (O(n) where n is parameter count). Subsequent get/set operations use cached
 *          pointer for O(1) access, providing 100-1000x speedup for frequently accessed
 *          parameters in high-rate scripts.
 *          
 *          Typical Usage Pattern:
 *          1. Create Parameter object in Lua script setup
 *          2. Call init() once with parameter name
 *          3. Use get()/set() in loop for fast repeated access
 *          
 *          Thread Safety: Parameter objects cache pointers to AP_Param entries. The
 *          underlying AP_Param system is thread-safe for reads and writes. However,
 *          Parameter objects should not be shared across multiple script instances.
 *          
 * @warning Parameter objects become invalid if the parameter system is reloaded
 *          (e.g., during parameter reset). Scripts should reinitialize Parameter
 *          objects after parameter system changes.
 * 
 * @note Alternative init_by_info() method supports accessing parameters by token,
 *       useful for reading old parameter values during parameter migrations
 * 
 * Source: libraries/AP_Scripting/AP_Scripting_helpers.h:10-30
 */
class Parameter
{
public:

    /**
     * @brief Initialize parameter by name for fast cached access
     * 
     * @details Performs initial parameter tree search to locate the parameter by name
     *          and caches the pointer for subsequent fast access. This is the primary
     *          initialization method for Lua scripts accessing parameters.
     *          
     *          Performance: This performs a full parameter tree search (expensive, O(n)).
     *          Call once during script initialization, not in loops.
     * 
     * @param[in] name Parameter name string (e.g., "WPNAV_SPEED", "BATT_CAPACITY")
     *                 Must match exact parameter name including case and underscores
     * 
     * @return true if parameter found and cached successfully, false if parameter not found
     * 
     * @note After successful init(), subsequent get() and set() calls use cached pointer
     * @warning Returns false if parameter name doesn't exist - scripts should check return value
     * 
     * @see get(), set()
     */
    bool init(const char *name);

    /**
     * @brief Initialize parameter by token for accessing old parameter values
     * 
     * @details Initializes parameter access using parameter token (key, group_element, type)
     *          instead of name. This is primarily used for parameter migration scenarios where
     *          scripts need to read old parameter values that may have been renamed or relocated.
     *          
     *          Token-based access bypasses name lookup and directly addresses parameter storage.
     * 
     * @param[in] key Parameter key identifying the parameter group
     * @param[in] group_element Element index within the parameter group
     * @param[in] type Parameter variable type (AP_PARAM_FLOAT, AP_PARAM_INT8, etc.)
     * 
     * @return true if parameter accessed successfully by token, false if invalid token
     * 
     * @note This is an advanced method - most scripts should use init() with parameter name
     * @warning Token values are internal to parameter system and may change between versions
     */
    bool init_by_info(uint16_t key, uint32_t group_element, enum ap_var_type type);

    /**
     * @brief Set parameter value (volatile, not saved to storage)
     * 
     * @details Updates parameter value in RAM only. Change takes effect immediately but
     *          will be lost on reboot. Use set_and_save() to persist changes.
     *          
     *          Thread Safety: Safe to call concurrently from multiple contexts.
     * 
     * @param[in] value New parameter value to set
     * 
     * @return true if value set successfully, false if parameter not initialized or value invalid
     * 
     * @note Value range checking is performed by underlying AP_Param system
     * @warning Must call init() successfully before calling set()
     * 
     * @see set_and_save(), get()
     */
    bool set(float value);

    /**
     * @brief Set parameter value and save to persistent storage
     * 
     * @details Updates parameter value in RAM and writes to EEPROM/Flash storage.
     *          Change persists across reboots. More expensive than set() due to storage write.
     *          
     *          Performance: Storage writes are slow (~10ms). Avoid calling in high-rate loops.
     *          
     *          Thread Safety: Safe to call concurrently - AP_Param handles storage locking.
     * 
     * @param[in] value New parameter value to set and save
     * 
     * @return true if value set and saved successfully, false if parameter not initialized or save failed
     * 
     * @note Excessive storage writes can wear out flash memory - use sparingly
     * @warning Must call init() successfully before calling set_and_save()
     * @warning Storage writes may take 10+ milliseconds - do not call in time-critical code
     * 
     * @see set(), get()
     */
    bool set_and_save(float value);

    /**
     * @brief Get current parameter value
     * 
     * @details Reads parameter value from cached pointer (O(1) operation).
     *          Returns value via reference parameter.
     *          
     *          Performance: Very fast - direct memory read from cached pointer.
     *          Safe to call in high-rate loops.
     * 
     * @param[out] value Reference to float variable that will receive parameter value
     * 
     * @return true if value retrieved successfully, false if parameter not initialized
     * 
     * @note Output parameter is only modified on successful return (true)
     * @warning Must call init() successfully before calling get()
     * 
     * @see set(), init()
     */
    bool get(float &value);

    /**
     * @brief Check if parameter is initialized and ready for use
     * 
     * @details Verifies that init() or init_by_info() was called successfully and
     *          the parameter pointer is valid. Scripts should call this to verify
     *          parameter availability before using get/set methods.
     * 
     * @return true if parameter is initialized and cached pointer is valid, false otherwise
     * 
     * @note Returns false if init() was never called or init() failed to find parameter
     * 
     * @see init(), init_by_info()
     */
    bool configured();

    /**
     * @brief Set default value for parameter (if not already set)
     * 
     * @details Sets the parameter to specified value only if the parameter has never
     *          been modified from its compiled default. Useful for script-specific
     *          parameter initialization without overwriting user-configured values.
     *          
     *          Use Case: Script can set preferred defaults for its parameters without
     *          forcing values on users who have already tuned them.
     * 
     * @param[in] value Default value to set if parameter is currently at compiled default
     * 
     * @return true if default set successfully or parameter already modified, false if parameter not initialized
     * 
     * @note Does NOT overwrite user-configured values - only affects factory defaults
     * @warning Must call init() successfully before calling set_default()
     * 
     * @see set(), set_and_save()
     */
    bool set_default(float value);

private:
    enum ap_var_type vtype;  ///< Cached parameter variable type (float, int8, int16, etc.)
    AP_Param *vp;            ///< Cached pointer to parameter object for fast access
};


#if HAL_ENABLE_DRONECAN_DRIVERS

#ifndef DRONECAN_HANDLE_MAX_PAYLOADS
/// Maximum number of received DroneCAN payloads buffered per subscription (configurable)
#define DRONECAN_HANDLE_MAX_PAYLOADS 8
#endif

/**
 * @class DroneCAN_Handle
 * @brief Interface for Lua scripts to send and receive DroneCAN (UAVCAN) messages
 * 
 * @details Provides Lua scripting access to DroneCAN/UAVCAN communication for custom
 *          CAN bus messaging with peripherals. Supports:
 *          - Broadcasting messages to all nodes
 *          - Sending service requests to specific nodes
 *          - Subscribing to and receiving messages from other nodes
 *          
 *          DroneCAN (formerly UAVCAN v0) is a CAN bus protocol for real-time communication
 *          with autopilot peripherals including GPS, compass, ESCs, airspeed sensors,
 *          and custom devices.
 *          
 *          Typical Usage:
 *          1. Create DroneCAN_Handle in Lua script
 *          2. Call subscribe() to receive messages of specific type
 *          3. Use broadcast() to send messages to all nodes
 *          4. Use request() to query specific node for data
 *          5. Call check_message() to retrieve received messages
 *          
 *          Thread Safety: DroneCAN operations use internal locking for thread-safe
 *          access to CAN bus hardware and message queues.
 * 
 * @warning DroneCAN_Handle objects allocate memory for message buffers using NEW_NOTHROW.
 *          Memory allocation failures will prevent subscription but are rare in normal operation.
 * 
 * @warning CAN bus bandwidth is limited. Excessive message transmission can saturate bus
 *          and affect critical peripheral communication (GPS, compass, etc.).
 * 
 * @note Requires HAL_ENABLE_DRONECAN_DRIVERS=1 and CAN_Dx_PROTOCOL=1 (DroneCAN) for operation
 * @note Maximum of DRONECAN_HANDLE_MAX_PAYLOADS messages buffered per subscription
 * 
 * Source: libraries/AP_Scripting/AP_Scripting_helpers.h:42-86
 */
class DroneCAN_Handle {
public:
    /**
     * @brief Lua binding to create new DroneCAN_Handle object
     * 
     * @param[in] L Lua state containing DroneCAN interface number and message signature
     * @return int Number of values pushed to Lua stack (1 on success, 0 on failure)
     * 
     * @note Called from Lua as dronecan:DroneCAN(interface)
     */
    static int new_handle(lua_State *L);

    /**
     * @brief Broadcast DroneCAN message to all nodes on the bus
     * 
     * @details Sends a message to all listening nodes without targeting specific node ID.
     *          Used for periodic telemetry broadcasts and status updates that multiple
     *          nodes may need to receive.
     *          
     * @param[in] L Lua state containing message data to broadcast
     * @return int Number of values pushed to Lua stack (1 = success boolean)
     * 
     * @warning Broadcasting large messages at high rates can saturate CAN bus bandwidth
     * 
     * @see request()
     */
    static int broadcast(lua_State *L);

    /**
     * @brief Lua garbage collection destructor
     * 
     * @details Called automatically by Lua garbage collector when handle is no longer
     *          referenced. Cleans up subscriptions and releases resources.
     * 
     * @param[in] L Lua state
     * @return int Number of values pushed to Lua stack (0)
     */
    static int __gc(lua_State *L);

    /**
     * @brief Close handle and release resources
     * 
     * @details Unsubscribes from message reception and frees allocated buffers.
     *          Handle becomes invalid after close() and should not be used further.
     * 
     * @note Automatically called by Lua garbage collector via __gc()
     */
    void close(void);

    /**
     * @brief Send DroneCAN service request to specific node
     * 
     * @details Sends a request message to a specific node ID and expects response.
     *          Used for querying node information or commanding specific peripherals.
     *          
     *          Unlike broadcast(), request() targets a single node and establishes
     *          request/response transaction.
     * 
     * @param[in] L Lua state containing node ID and request data
     * @return int Number of values pushed to Lua stack (1 = success boolean)
     * 
     * @note Response messages are received via check_message() with matching transfer ID
     * 
     * @see broadcast(), check_message()
     */
    static int request(lua_State *L);

    /**
     * @brief Check for received DroneCAN messages
     * 
     * @details Retrieves next buffered message from subscription queue. Returns nil if
     *          no messages available. Scripts should poll check_message() periodically
     *          to process received messages.
     *          
     *          Messages are buffered in FIFO queue up to DRONECAN_HANDLE_MAX_PAYLOADS depth.
     *          Older messages are dropped if queue fills.
     * 
     * @param[in] L Lua state
     * @return int Number of values pushed to Lua stack (1 = message table or nil)
     * 
     * @note Returns Lua table containing message data, timestamp, source node ID
     * @warning Messages are dropped if buffer fills - increase DRONECAN_HANDLE_MAX_PAYLOADS if needed
     * 
     * @see subscribe()
     */
    static int check_message(lua_State *L);

    /**
     * @brief Subscribe to DroneCAN messages of specific type
     * 
     * @details Registers message handler to receive messages matching the data_type
     *          and signature configured in this handle. Received messages are buffered
     *          for retrieval via check_message().
     *          
     *          Subscription persists until handle is closed or garbage collected.
     * 
     * @return true if subscription registered successfully, false if allocation failed
     * 
     * @warning Memory allocation uses NEW_NOTHROW - returns false on allocation failure
     * @warning Only one subscription per data_type per script - duplicate subscribe() calls replace previous subscription
     * 
     * @see check_message(), close()
     */
    bool subscribe(void);

    uint64_t signature;    ///< DroneCAN message type signature for filtering
    uint16_t data_type;    ///< DroneCAN data type ID for message routing
    uint8_t transfer_id;   ///< Transfer ID for matching requests with responses
    bool canfd;            ///< True if using CAN-FD extended frames, false for classic CAN

private:
    AP_DroneCAN *dc;       ///< Pointer to DroneCAN interface driver

    /**
     * @class Subscriber
     * @brief Message subscription handler for receiving DroneCAN messages in Lua scripts
     * 
     * @details Implements DroneCAN message reception by registering with the Canard handler
     *          system. Received messages are buffered in a circular queue for retrieval by
     *          Lua scripts via check_message().
     *          
     *          Inherits from Canard::HandlerList to integrate with libcanard message dispatch.
     *          The handle_message() callback is invoked by CAN interrupt handler when matching
     *          messages arrive.
     *          
     *          Message Buffering: Received messages are copied to Payload structures and
     *          stored in ObjectBuffer queue. If queue is full, oldest messages are dropped.
     *          
     *          Thread Safety: Message reception occurs in CAN interrupt context.
     *          ObjectBuffer provides thread-safe FIFO for passing messages to script context.
     * 
     * @warning Subscriber allocates memory for message payloads using NEW_NOTHROW in handle_message().
     *          Allocation failures during reception will cause message drops.
     * 
     * @note Each Subscriber handles one message type per DroneCAN_Handle
     * 
     * Source: libraries/AP_Scripting/AP_Scripting_helpers.h:61-83
     */
    class Subscriber : public Canard::HandlerList {
    public:
        /**
         * @brief Construct Subscriber for specific message type and transfer mode
         * 
         * @param[in] _handle Reference to parent DroneCAN_Handle containing message type configuration
         * @param[in] transfer_type Canard transfer type (broadcast, request, response)
         * 
         * @note Registers handler with Canard library for message type matching
         */
        Subscriber(DroneCAN_Handle &_handle, CanardTransferType transfer_type);

        /**
         * @brief Destructor - unregisters message handler and frees buffered messages
         * 
         * @details Safely removes handler from Canard dispatch and releases all
         *          queued message payloads.
         * 
         * @warning Must be called from non-interrupt context
         */
        virtual ~Subscriber(void);

        uint8_t node_id;  ///< Source node ID filter (0 = receive from all nodes)

        /**
         * @struct Payload
         * @brief Container for received DroneCAN message data
         * 
         * @details Stores complete received message including data buffer, metadata,
         *          and timing information for delivery to Lua script.
         */
        struct Payload {
            uint8_t *data;      ///< Dynamically allocated message data buffer (freed when retrieved)
            uint16_t length;    ///< Message data length in bytes
            uint8_t node_id;    ///< Source node ID that sent this message
#if CANARD_ENABLE_CANFD
            bool canfd;         ///< True if received via CAN-FD, false if classic CAN
#endif
            uint64_t timestamp; ///< Reception timestamp in microseconds (system time)
        };

        /**
         * @brief Thread-safe circular buffer for received message payloads
         * 
         * @details ObjectBuffer provides lock-free FIFO queue for passing messages from
         *          CAN interrupt context to Lua script context. Fixed size defined by
         *          DRONECAN_HANDLE_MAX_PAYLOADS.
         * 
         * @warning If buffer fills, oldest messages are dropped - scripts should call
         *          check_message() frequently enough to prevent overflow
         */
        ObjectBuffer<Payload> payloads{DRONECAN_HANDLE_MAX_PAYLOADS};

        CanardTransferType trans_type;  ///< Transfer type filter (broadcast/request/response)

    private:
        /**
         * @brief Message reception callback invoked by Canard library
         * 
         * @details Called from CAN interrupt context when matching message received.
         *          Allocates Payload structure, copies message data, and queues for
         *          script retrieval.
         *          
         *          Memory Management: Allocates message data buffer using NEW_NOTHROW.
         *          Memory is freed when script retrieves message or on Subscriber destruction.
         * 
         * @param[in] transfer Received CAN transfer containing message data and metadata
         * @return true if message handled successfully, false if buffer full or allocation failed
         * 
         * @warning Called in interrupt context - must be fast and non-blocking
         * @warning Memory allocation can fail - messages dropped on allocation failure
         */
        bool handle_message(const CanardRxTransfer& transfer) override;

        DroneCAN_Handle *handle;  ///< Back-pointer to parent handle for accessing configuration
    };

    Subscriber *subscriber;  ///< Active subscription handler (nullptr if not subscribed)
};

#endif // HAL_ENABLE_DRONECAN_DRIVERS
