/**
 * @file AP_RCProtocol_DroneCAN.h
 * @brief DroneCAN RC input protocol backend for receiving RC control data over CAN bus
 * 
 * @details This file implements the DroneCAN/UAVCAN RC input protocol backend, enabling
 *          ArduPilot to receive radio control (RC) commands from RC receivers connected
 *          via the CAN bus using the DroneCAN protocol. This allows for digital RC
 *          transmission with improved reliability and reduced wiring complexity compared
 *          to traditional PWM/PPM RC connections.
 * 
 *          Integration: This backend subscribes to dronecan.sensors.rc.RCInput messages
 *          published by DroneCAN nodes (typically RC receivers with CAN interface) and
 *          processes them through the AP_RCProtocol framework.
 * 
 *          Multi-Node Support: Supports multiple DroneCAN RC receivers on the same CAN
 *          bus, with separate RC data buffers maintained per node ID (up to 4 nodes).
 * 
 * @note Part of the AP_RCProtocol backend system for unified RC input handling
 * @see AP_RCProtocol_Backend
 * @see AP_DroneCAN
 */

#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_DRONECAN_ENABLED

#include "AP_RCProtocol_Backend.h"

#include <AP_DroneCAN/AP_DroneCAN.h>

#include <AP_Common/missing/endian.h>

/**
 * @class AP_RCProtocol_DroneCAN
 * @brief Backend for receiving RC input via DroneCAN protocol over CAN bus
 * 
 * @details This backend implements RC input reception from DroneCAN-enabled RC receivers
 *          connected to the vehicle's CAN bus. It subscribes to the DroneCAN
 *          dronecan.sensors.rc.RCInput message type and processes incoming RC channel
 *          data, including quality indicators and failsafe status.
 * 
 *          Key Features:
 *          - Receives up to MAX_RCIN_CHANNELS RC channels over DroneCAN
 *          - Channel values in standard PWM microseconds (typically 1000-2000μs)
 *          - Failsafe detection via message timeout and status flags
 *          - Quality indicators for signal strength monitoring
 *          - Multi-node support for redundant RC receivers
 *          - Thread-safe access to RC data via semaphore protection
 * 
 *          Message Format: Processes dronecan.sensors.rc.RCInput messages containing:
 *          - status: Quality valid bit and failsafe bit flags
 *          - quality: Signal quality indicator (0-255, higher is better)
 *          - id: RC system ID for multi-receiver setups
 *          - channels[]: Array of RC channel values in microseconds
 * 
 *          Lifecycle:
 *          1. Backend instantiated by AP_RCProtocol system
 *          2. subscribe_msgs() called to register DroneCAN message subscription
 *          3. handle_rcinput() callback invoked when messages arrive from CAN bus
 *          4. update() called periodically to provide RC data to frontend
 * 
 * @warning Thread Safety: RC data buffer access is protected by semaphore as messages
 *          arrive asynchronously from CAN interrupt context and are consumed in main thread
 * 
 * @note Supports multiple DroneCAN interfaces - each CAN bus can have RC receivers
 * @note Failsafe triggered if no valid messages received within timeout period
 */
class AP_RCProtocol_DroneCAN : public AP_RCProtocol_Backend {
public:

    /**
     * @brief Construct DroneCAN RC protocol backend
     * 
     * @param[in] _frontend Reference to the AP_RCProtocol frontend managing all RC backends
     * 
     * @note Sets singleton pointer for static callback access from DroneCAN subscription
     */
    AP_RCProtocol_DroneCAN(AP_RCProtocol &_frontend) :
        AP_RCProtocol_Backend(_frontend) {
        _singleton = this;
    }

    /**
     * @brief Subscribe to DroneCAN RC input messages on specified CAN interface
     * 
     * @details Registers a message handler callback with the AP_DroneCAN manager to receive
     *          dronecan.sensors.rc.RCInput messages published by RC receivers on the CAN bus.
     *          This function is called during initialization for each active DroneCAN interface
     *          to establish the subscription.
     * 
     * @param[in] ap_dronecan Pointer to the AP_DroneCAN instance managing the CAN interface
     * 
     * @return true if subscription successfully registered, false if subscription failed
     * 
     * @note Called during system initialization, typically once per CAN interface
     * @note Subscription remains active for lifetime of the DroneCAN interface
     * @see handle_rcinput() - Callback function invoked when messages arrive
     */
    static bool subscribe_msgs(AP_DroneCAN* ap_dronecan);

    /**
     * @brief Update RC input data from DroneCAN and provide to frontend
     * 
     * @details Called periodically by the AP_RCProtocol framework to process RC data received
     *          from DroneCAN messages. Retrieves RC channel values from the thread-safe buffer
     *          populated by handle_rcinput() callback and forwards them to the RC frontend.
     *          Handles failsafe detection based on message timeout.
     * 
     *          Processing Flow:
     *          1. Acquire semaphore to safely access RC data buffer
     *          2. Check for message timeout (failsafe condition)
     *          3. Extract channel values (in microseconds) and quality/failsafe flags
     *          4. Forward valid RC data to frontend via add_input()
     *          5. Release semaphore
     * 
     * @note Called at RC input processing rate (typically 50-100Hz)
     * @note Thread-safe: Protected by semaphore for concurrent access with CAN callbacks
     * @warning Message timeout triggers failsafe - no RC data provided to frontend
     * 
     * @see add_input() - Frontend method to deliver RC channel data
     */
    void update() override;

private:

    /**
     * @brief Singleton pointer for static callback access
     * 
     * @details Provides access to the backend instance from static DroneCAN message callbacks
     *          which cannot directly access instance methods.
     * 
     * @note Set during construction, used by handle_rcinput() callback
     */
    static class AP_RCProtocol_DroneCAN *_singleton;

    /**
     * @brief DroneCAN message callback for RC input messages
     * 
     * @details Static callback function invoked by AP_DroneCAN framework when a
     *          dronecan.sensors.rc.RCInput message is received on the CAN bus. Processes
     *          the message and stores RC channel data in the thread-safe buffer for
     *          consumption by update() in the main thread.
     * 
     *          Message Processing:
     *          - Extracts channel values from msg.channels[] array (PWM microseconds)
     *          - Stores quality indicator (0-255 signal strength)
     *          - Captures failsafe and quality valid status flags
     *          - Records timestamp for timeout detection
     *          - Supports multi-node operation via node_id differentiation
     * 
     *          DroneCAN Message Structure (dronecan.sensors.rc.RCInput):
     *          - status: uint8_t with bit 0 = QUALITY_VALID, bit 1 = FAILSAFE
     *          - quality: uint8_t signal quality (0=worst, 255=best)
     *          - id: uint8_t RC system identifier
     *          - channels[]: Array of uint16_t PWM values in microseconds (1000-2000μs typical)
     * 
     * @param[in] ap_dronecan Pointer to AP_DroneCAN instance that received the message
     * @param[in] transfer    CanardRxTransfer containing message metadata (timestamp, node_id, etc.)
     * @param[in] msg         Parsed dronecan.sensors.rc.RCInput message data
     * 
     * @note Called from CAN receive interrupt context - must be fast and thread-safe
     * @warning Semaphore protection required for RC buffer access due to async invocation
     * @see update() - Main thread consumer of RC data stored by this callback
     */
    static void handle_rcinput(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const dronecan_sensors_rc_RCInput &msg);

    /**
     * @brief Retrieve or create backend instance for specific DroneCAN node
     * 
     * @details Implements multi-node RC receiver support by maintaining separate backend
     *          instances for each CAN node ID. Searches the device registry for an existing
     *          backend handling the specified node, or creates a new registry entry if this
     *          is the first message from that node.
     * 
     *          Multi-Node Operation:
     *          - Allows multiple RC receivers on same CAN bus (redundancy or multiple vehicles)
     *          - Each node_id gets dedicated RC data buffer
     *          - Registry supports up to 4 simultaneous RC nodes
     *          - First message from a node auto-registers it
     * 
     * @param[in] ap_dronecan Pointer to AP_DroneCAN interface receiving messages
     * @param[in] node_id     DroneCAN node identifier (0-127) of the RC receiver
     * 
     * @return Pointer to backend instance for this node, or nullptr if registry full
     * 
     * @note Thread-safe: Registry access protected by semaphore
     * @warning Returns nullptr if maximum number of nodes already registered
     */
    static AP_RCProtocol_DroneCAN* get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id);

    /**
     * @brief RC input data buffer for thread-safe message storage
     * 
     * @details Contains the most recent RC input data received from DroneCAN messages.
     *          Populated asynchronously by handle_rcinput() callback (CAN interrupt context)
     *          and consumed by update() method (main thread). Semaphore protection ensures
     *          data consistency between producer and consumer threads.
     * 
     * @warning Thread Safety: All accesses must acquire rcin.sem semaphore to prevent
     *          race conditions between CAN callbacks and main thread RC processing
     */
    struct {
        uint8_t quality;              ///< Signal quality indicator (0-255, higher is better) from DroneCAN message
        
        /**
         * @brief Status flags from DroneCAN RC message
         * 
         * @details Union allowing access as complete uint16_t or individual bit flags.
         *          Provides both packed status word and convenient bit field access.
         */
        union {
            uint16_t status;          ///< Complete status word from DroneCAN message
            struct {
                uint8_t QUALITY_VALID : 1;  ///< Bit 0: Quality field contains valid data
                uint8_t FAILSAFE : 1;       ///< Bit 1: RC receiver is in failsafe state
            } bits;
        };
        
        uint8_t num_channels;         ///< Number of active RC channels received (0 to MAX_RCIN_CHANNELS)
        
        /**
         * @brief RC channel values in microseconds
         * 
         * @details Channel array containing PWM pulse width values for each RC channel.
         *          Standard RC servos expect 1000-2000μs (1ms-2ms) with 1500μs as center.
         *          ArduPilot typically uses 1000-2000μs range with extensions to 800-2200μs.
         * 
         * @note Units: Microseconds (μs), standard PWM pulse width encoding
         * @note Valid range: Typically 1000-2000μs, extended range 800-2200μs
         * @note Array size: MAX_RCIN_CHANNELS (system maximum RC channels)
         */
        uint16_t channels[MAX_RCIN_CHANNELS];

        uint32_t last_sample_time_ms; ///< Timestamp (milliseconds) of last received RC message for timeout detection
        
        HAL_Semaphore sem;            ///< Semaphore protecting concurrent access between CAN callbacks and main thread
    } rcin;

    /**
     * @brief Multi-node device detection and management registry
     * 
     * @details Maintains a registry of detected DroneCAN RC receiver nodes, enabling support
     *          for multiple RC receivers on the same CAN bus. Each detected node receives a
     *          dedicated backend instance to handle its RC data independently, enabling
     *          redundancy or multi-vehicle scenarios on shared CAN infrastructure.
     * 
     *          Use Cases:
     *          - Redundant RC receivers for safety-critical applications
     *          - Multiple vehicles sharing one CAN bus with separate RC links
     *          - Hot-swapping RC receivers without reboot
     * 
     * @note Static storage: Registry is shared across all backend instances
     * @warning Thread-safe: Registry access protected by registry.sem semaphore
     */
    static struct Registry {
        /**
         * @brief Information for one detected DroneCAN RC receiver node
         * 
         * @details Stores the association between a CAN node ID, its DroneCAN interface,
         *          and the backend driver instance handling its messages. Created when first
         *          message arrives from a new node_id.
         */
        struct DetectedDevice {
            AP_DroneCAN* ap_dronecan;           ///< Pointer to AP_DroneCAN interface this node is on
            uint8_t node_id;                    ///< DroneCAN node identifier (0-127) for this RC receiver
            AP_RCProtocol_DroneCAN *driver;     ///< Backend driver instance handling this node's RC data
        } detected_devices[1];                  ///< Array of detected devices (currently supports 1 node)
        
        HAL_Semaphore sem;                      ///< Semaphore protecting registry modifications from concurrent access
    } registry;

    uint32_t last_receive_ms;                   ///< Timestamp (milliseconds) of last message reception for failsafe timeout detection
};


#endif  // AP_RCPROTOCOL_DRONECAN_ENABLED
