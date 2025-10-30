/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Eugene Shamaev, Siddharth Bharat Purohit
 */

/**
 * @file AP_DroneCAN.h
 * @brief Main DroneCAN/UAVCAN driver implementation for ArduPilot
 * 
 * @details This file implements the primary interface to the DroneCAN (formerly UAVCAN v0)
 *          protocol stack, providing CAN bus communication for ArduPilot autopilots.
 *          DroneCAN enables communication with CAN-connected peripherals including:
 *          - Electronic Speed Controllers (ESCs) with bidirectional telemetry
 *          - Servo actuators with position feedback
 *          - GPS receivers with RTK support
 *          - Magnetometers, barometers, and other sensors
 *          - GNSS receivers, rangefinders, optical flow sensors
 *          - Safety switches, buttons, and indicators (LEDs, buzzers)
 *          - Camera gimbals and other payloads
 * 
 *          The driver runs in a dedicated thread (loop()) and handles:
 *          - Message transmission/reception via libcanard library
 *          - Servo/ESC output command streaming
 *          - Peripheral sensor data ingestion
 *          - Parameter get/set operations on remote nodes
 *          - Dynamic Node Allocation (DNA) for node ID assignment
 *          - Node health monitoring and statistics
 * 
 *          Protocol: UAVCAN v0 via DroneCAN specification
 *          Transport: CAN 2.0B (11-bit/29-bit identifiers) and CAN FD (optional)
 * 
 * @note This implementation uses the libcanard library for protocol handling
 * @see libraries/AP_DroneCAN/README.md for comprehensive architecture documentation
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#if HAL_ENABLE_DRONECAN_DRIVERS

#include "AP_Canard_iface.h"
#include <AP_CANManager/AP_CANManager.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>
#include <SRV_Channel/SRV_Channel_config.h>
#include <canard/publisher.h>
#include <canard/subscriber.h>
#include <canard/service_client.h>
#include <canard/service_server.h>
#include <stdio.h>
#include "AP_DroneCAN_DNA_Server.h"
#include <canard.h>
#include <dronecan_msgs.h>
#include <AP_SerialManager/AP_SerialManager_config.h>
#include <AP_Relay/AP_Relay_config.h>
#include <AP_Servo_Telem/AP_Servo_Telem_config.h>
#include <AP_Mount/AP_Mount_config.h>

#ifndef DRONECAN_SRV_NUMBER
#define DRONECAN_SRV_NUMBER NUM_SERVO_CHANNELS
#endif

#ifndef AP_DRONECAN_SEND_GPS
#define AP_DRONECAN_SEND_GPS (HAL_PROGRAM_SIZE_LIMIT_KB > 1024)
#endif

#define AP_DRONECAN_SW_VERS_MAJOR 1
#define AP_DRONECAN_SW_VERS_MINOR 0

#define AP_DRONECAN_HW_VERS_MAJOR 1
#define AP_DRONECAN_HW_VERS_MINOR 0


#ifndef AP_DRONECAN_HOBBYWING_ESC_SUPPORT
#define AP_DRONECAN_HOBBYWING_ESC_SUPPORT (HAL_PROGRAM_SIZE_LIMIT_KB>1024)
#endif

#ifndef AP_DRONECAN_HIMARK_SERVO_SUPPORT
#define AP_DRONECAN_HIMARK_SERVO_SUPPORT (HAL_PROGRAM_SIZE_LIMIT_KB>1024)
#endif

#ifndef AP_DRONECAN_SERIAL_ENABLED
#define AP_DRONECAN_SERIAL_ENABLED AP_SERIALMANAGER_REGISTER_ENABLED && (HAL_PROGRAM_SIZE_LIMIT_KB>1024)
#endif

#ifndef AP_DRONECAN_VOLZ_FEEDBACK_ENABLED
#define AP_DRONECAN_VOLZ_FEEDBACK_ENABLED 0
#endif

#if AP_DRONECAN_SERIAL_ENABLED
#include "AP_DroneCAN_serial.h"
#endif

// fwd-declare callback classes
class AP_DroneCAN_DNA_Server;
class CANSensor;

/**
 * @class AP_DroneCAN
 * @brief Main DroneCAN/UAVCAN protocol driver for CAN bus peripherals
 * 
 * @details AP_DroneCAN manages communication with DroneCAN-compatible devices over CAN bus.
 *          This class serves as the central hub for:
 *          - Publishing actuator commands (ESC/servo outputs)
 *          - Subscribing to sensor data and peripheral status messages
 *          - Managing parameter synchronization with remote nodes
 *          - Coordinating Dynamic Node Allocation (DNA) for plug-and-play devices
 *          - Broadcasting vehicle state and telemetry
 * 
 * **Thread Architecture:**
 * The driver runs in a dedicated thread created during init(), executing the loop() method
 * continuously. Message handling uses the Canard Publisher/Subscriber/Client pattern:
 * - Publishers: Send periodic or event-driven messages (actuator commands, telemetry)
 * - Subscribers: Receive messages from CAN devices (sensor data, status updates)
 * - Clients: Request/response transactions (parameter get/set, node info queries)
 * 
 * **Lifecycle:**
 * 1. Construction: AP_DroneCAN(driver_index) - Create instance for CAN interface
 * 2. Initialization: init(driver_index, enable_filters) - Set up memory pool, start thread
 * 3. Interface binding: add_interface(can_iface) - Attach HAL CAN interface
 * 4. Runtime: loop() runs continuously in dedicated thread
 * 5. Destruction: ~AP_DroneCAN() - Clean up resources and stop thread
 * 
 * **ESC and Servo Output:**
 * Supports multiple output protocols selectable via Options parameter:
 * - Standard ESC RawCommand (PWM-like values 0-8191)
 * - Actuator ArrayCommand (normalized -1.0 to +1.0 with metadata)
 * - Vendor-specific protocols (Hobbywing ESC, Himark servo)
 * Output rates configurable via _servo_rate_hz parameter (typically 50-400 Hz)
 * 
 * **Peripheral Support:**
 * Automatically discovers and integrates:
 * - GPS: Fix data, RTK corrections, heading/yaw information
 * - Compass: Magnetic field vectors with calibration offsets
 * - Barometer: Pressure altitude with temperature compensation
 * - Rangefinder: Distance measurements for terrain following
 * - ESC Telemetry: Voltage, current, RPM, temperature per ESC
 * - Servo Feedback: Position, velocity, effort from smart servos
 * - Safety Devices: Arming buttons, parachute triggers, hardpoints
 * 
 * **Message Priorities:**
 * DroneCAN uses 5-bit priority field (0=highest, 31=lowest):
 * - Critical (0-7): Safety, arming status, emergency commands
 * - High (8-15): Control outputs (ESC/servo commands)
 * - Normal (16-23): Telemetry, sensor data, periodic status
 * - Low (24-31): Diagnostics, parameter operations, logging
 * 
 * **Memory Management:**
 * Uses memory pool allocated at init() for CAN message buffers. Pool size configurable
 * via _pool_size parameter (default depends on features enabled). Insufficient pool size
 * will cause message transmission failures logged as _fail_send_count.
 * 
 * **CAN FD Support:**
 * When Options::CANFD_ENABLED is set and hardware supports it, enables CAN FD mode:
 * - Larger payloads (up to 64 bytes vs 8 bytes for CAN 2.0)
 * - Higher bit rates for improved bandwidth
 * - Backward compatible with CAN 2.0B devices on same bus
 * 
 * @note This driver implements UAVCAN v0 protocol via DroneCAN specification
 * @warning CAN bus timing is critical - misconfigured bit rates cause communication failure.
 *          Ensure all devices on bus use same bit rate (typically 1 Mbps).
 * @warning Message priorities must be carefully managed to prevent control output starvation
 *          by high-rate sensor data. Critical safety messages always use highest priority.
 * @warning Thread-safety: Most methods are called from the dedicated DroneCAN thread.
 *          External calls (e.g., SRV_push_servos) use semaphores for synchronization.
 * 
 * @see AP_CANDriver Base class providing HAL CAN interface abstraction
 * @see AP_ESC_Telem_Backend Base class for ESC telemetry data ingestion
 * @see AP_DroneCAN_DNA_Server Handles Dynamic Node Allocation protocol
 * @see CanardInterface Libcanard interface wrapper for protocol stack
 */
class AP_DroneCAN : public AP_CANDriver, public AP_ESC_Telem_Backend {
    friend class AP_DroneCAN_DNA_Server;
public:
    /**
     * @brief Construct a new AP_DroneCAN driver instance
     * 
     * @details Creates a DroneCAN driver for the specified CAN interface index.
     *          Initializes internal state but does not allocate memory pool or start
     *          the communication thread - that occurs in init().
     * 
     * @param[in] driver_index CAN driver index (0 for CAN1, 1 for CAN2, etc.)
     * 
     * @note Constructor is lightweight - heavy initialization deferred to init()
     */
    AP_DroneCAN(const int driver_index);
    
    /**
     * @brief Destructor - cleanup resources and stop thread
     * 
     * @details Stops the DroneCAN thread, frees memory pool, and releases CAN interface.
     * 
     * @warning Ensure no active CAN transactions when destroying instance
     */
    ~AP_DroneCAN();

    /**
     * @brief Parameter table definition for AP_Param system
     * 
     * @details Defines configuration parameters:
     *          - NODE: Local node ID (1-127, 0=disabled)
     *          - SRV_BM: Servo output bitmask
     *          - ESC_BM: ESC output bitmask  
     *          - SRV_RATE: Servo update rate in Hz
     *          - OPTIONS: Feature bitmask (see Options enum)
     *          - POOL_SIZE: Memory pool size for message buffers
     */
    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @brief Get DroneCAN driver instance by index
     * 
     * @details Returns pointer to initialized DroneCAN driver for specified interface.
     *          Used by peripheral drivers to access DroneCAN communication services.
     * 
     * @param[in] driver_index CAN driver index (0-based)
     * 
     * @return AP_DroneCAN* Pointer to driver instance, or nullptr if not initialized
     * 
     * @note Returns nullptr if driver not ready or index invalid
     */
    static AP_DroneCAN *get_dronecan(uint8_t driver_index);
    
    /**
     * @brief Pre-arm safety check for DroneCAN subsystem
     * 
     * @details Validates DroneCAN health before allowing vehicle arming:
     *          - Checks DNA server status (no duplicate node IDs)
     *          - Verifies critical peripherals are online (GPS, compass if configured)
     *          - Confirms ESC outputs are responding if configured
     *          - Validates CAN bus health (no excessive errors)
     * 
     * @param[out] fail_msg Buffer to receive failure description if check fails
     * @param[in]  fail_msg_len Size of fail_msg buffer in bytes
     * 
     * @return true if all checks pass and vehicle safe to arm
     * @return false if any check fails (fail_msg contains reason)
     * 
     * @note Called by AP_Arming during pre-arm check sequence
     */
    bool prearm_check(char* fail_msg, uint8_t fail_msg_len) const;

    /**
     * @brief Initialize DroneCAN driver and start communication thread
     * 
     * @details Performs heavy initialization:
     *          - Allocates memory pool for message buffers (size from _pool_size parameter)
     *          - Initializes libcanard interface with local node ID
     *          - Sets up publishers, subscribers, and service clients
     *          - Starts dedicated thread running loop() method
     *          - Initializes DNA server for node ID allocation
     * 
     * @param[in] driver_index CAN driver index (must match constructor value)
     * @param[in] enable_filters If true, configure CAN hardware filters for efficiency
     * 
     * @warning Must be called before any CAN communication
     * @warning Only call once per driver instance
     * 
     * @note Thread name format: "dronecan_N" where N is driver_index
     */
    __INITFUNC__ void init(uint8_t driver_index, bool enable_filters) override;
    
    /**
     * @brief Attach HAL CAN interface to this driver
     * 
     * @details Binds hardware CAN interface from HAL to this DroneCAN driver instance.
     *          Must be called after init() but before communication begins.
     * 
     * @param[in] can_iface Pointer to HAL CAN interface (AP_HAL::CANIface)
     * 
     * @return true if interface successfully attached
     * @return false if interface already attached or invalid
     * 
     * @note Typically called by AP_CANManager during system initialization
     */
    bool add_interface(AP_HAL::CANIface* can_iface) override;

    /**
     * @brief Register an 11-bit auxiliary CAN driver
     * 
     * @details Allows non-DroneCAN devices using 11-bit CAN identifiers to share the bus.
     *          Auxiliary drivers receive frames not matching DroneCAN protocol format.
     * 
     * @param[in] sensor Pointer to CANSensor auxiliary driver instance
     * 
     * @return true if driver successfully registered
     * @return false if registration failed (table full or invalid)
     * 
     * @note Used for proprietary CAN devices that don't follow DroneCAN protocol
     */
    bool add_11bit_driver(CANSensor *sensor) override;

    /**
     * @brief Send raw CAN frame from auxiliary driver
     * 
     * @details Transmits a frame on behalf of an auxiliary driver, bypassing DroneCAN
     *          protocol stack. Used for proprietary CAN devices.
     * 
     * @param[in,out] out_frame CAN frame to transmit (may be modified with timestamp)
     * @param[in]     timeout_us Maximum time to wait for TX buffer space (microseconds)
     * 
     * @return true if frame successfully queued for transmission
     * @return false if timeout or bus error
     * 
     * @warning Auxiliary frames compete with DroneCAN traffic for bus bandwidth
     * @note Called by auxiliary drivers, not directly by vehicle code
     */
    bool write_aux_frame(AP_HAL::CANFrame &out_frame, const uint32_t timeout_us) override;
    
    /**
     * @brief Get the driver index for this DroneCAN instance
     * 
     * @return uint8_t Driver index (0 for CAN1, 1 for CAN2, etc.)
     * 
     * @note Used to associate peripherals with specific CAN interface
     */
    uint8_t get_driver_index() const { return _driver_index; }

    /**
     * @brief String structure for DroneCAN string parameters
     * 
     * @details Length-prefixed string format matching UAVCAN protocol string representation.
     *          Used for parameter get/set operations with string values.
     */
    struct string { 
        uint8_t len;          ///< String length in bytes (0-128)
        uint8_t data[128];    ///< String data buffer
    };

    /**
     * @brief Callback function type for integer parameter get/set operations
     * 
     * @details Signature: bool callback(AP_DroneCAN* dronecan, uint8_t node_id, const char* name, int32_t& value)
     * 
     * @param dronecan Pointer to DroneCAN driver instance
     * @param node_id Remote node ID that responded
     * @param name Parameter name
     * @param value Parameter value (input for set, output for get)
     * @return true if operation succeeded, false on error
     */
    FUNCTOR_TYPEDEF(ParamGetSetIntCb, bool, AP_DroneCAN*, const uint8_t, const char*, int32_t &);
    
    /**
     * @brief Callback function type for floating-point parameter get/set operations
     * 
     * @details Signature: bool callback(AP_DroneCAN* dronecan, uint8_t node_id, const char* name, float& value)
     * 
     * @param dronecan Pointer to DroneCAN driver instance
     * @param node_id Remote node ID that responded
     * @param name Parameter name
     * @param value Parameter value (input for set, output for get)
     * @return true if operation succeeded, false on error
     */
    FUNCTOR_TYPEDEF(ParamGetSetFloatCb, bool, AP_DroneCAN*, const uint8_t, const char*, float &);
    
    /**
     * @brief Callback function type for string parameter get/set operations
     * 
     * @details Signature: bool callback(AP_DroneCAN* dronecan, uint8_t node_id, const char* name, string& value)
     * 
     * @param dronecan Pointer to DroneCAN driver instance
     * @param node_id Remote node ID that responded
     * @param name Parameter name
     * @param value Parameter string value
     * @return true if operation succeeded, false on error
     */
    FUNCTOR_TYPEDEF(ParamGetSetStringCb, bool, AP_DroneCAN*, const uint8_t, const char*, string &);
    
    /**
     * @brief Callback function type for parameter save operations
     * 
     * @details Signature: void callback(AP_DroneCAN* dronecan, uint8_t node_id, bool success)
     * 
     * @param dronecan Pointer to DroneCAN driver instance
     * @param node_id Remote node ID that responded
     * @param success true if parameters saved successfully, false on error
     */
    FUNCTOR_TYPEDEF(ParamSaveCb, void, AP_DroneCAN*,  const uint8_t, bool);

    /**
     * @brief Send node status message
     * 
     * @details Broadcasts uavcan.protocol.NodeStatus message containing:
     *          - Node health (OK, WARNING, ERROR, CRITICAL)
     *          - Operating mode (OPERATIONAL, INITIALIZATION, MAINTENANCE, etc.)
     *          - Uptime in seconds
     *          - Vendor-specific status code
     * 
     * @note Called periodically from loop() at 1 Hz by default
     * @note Other nodes use this for health monitoring and diagnostics
     */
    void send_node_status();

    /**
     * @brief Push servo outputs to DroneCAN actuators
     * 
     * @details Called by vehicle code (typically from SRV_Channels) to send updated
     *          servo/ESC commands over CAN bus. Converts RC_Channel PWM values to
     *          appropriate DroneCAN message format based on configuration:
     *          - Standard ESC RawCommand (default): PWM-like 0-8191 values
     *          - Actuator ArrayCommand (USE_ACTUATOR_PWM option): Normalized -1.0 to +1.0
     *          - Vendor-specific (Hobbywing, Himark): Protocol-specific formatting
     * 
     * @note Thread-safe: Uses SRV_sem semaphore for synchronization
     * @note Actual transmission occurs in loop() at configured rate (_servo_rate_hz)
     * @note Only outputs with bits set in _servo_bm or _esc_bm are transmitted
     */
    void SRV_push_servos(void);

    /**
     * @brief Control RGB LED on a DroneCAN device
     * 
     * @details Sends uavcan.equipment.indication.LightsCommand to control addressable
     *          RGB LED on remote node. Commonly used for status indication on
     *          DroneCAN-connected peripherals (GPS, flight controllers, etc.)
     * 
     * @param[in] led_index LED index on remote device (device-specific, typically 0)
     * @param[in] red Red channel intensity (0-255)
     * @param[in] green Green channel intensity (0-255)
     * @param[in] blue Blue channel intensity (0-255)
     * 
     * @return true if message successfully queued for transmission
     * @return false if transmission failed (bus error or queue full)
     * 
     * @note LED updates typically occur at low rate (1-10 Hz)
     * @note Not all DroneCAN devices support LED control
     */
    bool led_write(uint8_t led_index, uint8_t red, uint8_t green, uint8_t blue);

    /**
     * @brief Play tone on DroneCAN buzzer
     * 
     * @details Sends uavcan.equipment.indication.BeepCommand to activate buzzer
     *          on remote node. Used for audible feedback and warnings.
     * 
     * @param[in] frequency Tone frequency in Hz (0 = silence, typical range 200-4000 Hz)
     * @param[in] duration_s Duration in seconds (0 for continuous until next command)
     * 
     * @note Multiple DroneCAN nodes may have buzzers; all will respond to command
     * @note Frequency interpretation is device-specific; some buzzers have limited range
     */
    void set_buzzer_tone(float frequency, float duration_s);

    /**
     * @brief Send reboot request to remote DroneCAN node
     * 
     * @details Transmits uavcan.protocol.RestartNode command to reboot specified device.
     *          Used for applying configuration changes or recovering from errors.
     * 
     * @param[in] node_id Target node ID to reboot (1-127)
     * 
     * @warning NOT THREAD SAFE - must only be called from DroneCAN thread context
     * @warning Safe to call from DroneCAN callbacks/handlers, NOT from vehicle code
     * @warning Node will disconnect briefly during reboot; ensure system can tolerate loss
     * 
     * @note Some nodes may not support remote reboot
     * @note Reboot typically takes 1-5 seconds depending on node bootloader
     */
    void send_reboot_request(uint8_t node_id);

    /**
     * @brief Set floating-point parameter on remote DroneCAN node
     * 
     * @details Sends uavcan.protocol.param.GetSet request to modify parameter value
     *          on specified node. Operation is asynchronous; callback invoked when
     *          response received or timeout occurs.
     * 
     * @param[in] node_id Target node ID (1-127)
     * @param[in] name Parameter name (null-terminated string, max 92 chars)
     * @param[in] value New parameter value (float)
     * @param[in] cb Callback function invoked with result (may be nullptr)
     * 
     * @return true if request successfully queued
     * @return false if previous request still pending (only one request at a time)
     * 
     * @note Parameter not persisted until save_parameters_on_node() called
     * @note Timeout after 1000ms if no response received
     * @warning Only one parameter operation at a time; check return value
     */
    bool set_parameter_on_node(uint8_t node_id, const char *name, float value, ParamGetSetFloatCb *cb);
    
    /**
     * @brief Set integer parameter on remote DroneCAN node
     * 
     * @details Sends uavcan.protocol.param.GetSet request to modify parameter value
     *          on specified node. Operation is asynchronous; callback invoked when
     *          response received or timeout occurs.
     * 
     * @param[in] node_id Target node ID (1-127)
     * @param[in] name Parameter name (null-terminated string, max 92 chars)
     * @param[in] value New parameter value (int32_t)
     * @param[in] cb Callback function invoked with result (may be nullptr)
     * 
     * @return true if request successfully queued
     * @return false if previous request still pending (only one request at a time)
     * 
     * @note Parameter not persisted until save_parameters_on_node() called
     * @note Timeout after 1000ms if no response received
     * @warning Only one parameter operation at a time; check return value
     */
    bool set_parameter_on_node(uint8_t node_id, const char *name, int32_t value, ParamGetSetIntCb *cb);
    
    /**
     * @brief Set string parameter on remote DroneCAN node
     * 
     * @details Sends uavcan.protocol.param.GetSet request to modify parameter value
     *          on specified node. Operation is asynchronous; callback invoked when
     *          response received or timeout occurs.
     * 
     * @param[in] node_id Target node ID (1-127)
     * @param[in] name Parameter name (null-terminated string, max 92 chars)
     * @param[in] value New parameter value (string structure with length prefix)
     * @param[in] cb Callback function invoked with result (may be nullptr)
     * 
     * @return true if request successfully queued
     * @return false if previous request still pending (only one request at a time)
     * 
     * @note Parameter not persisted until save_parameters_on_node() called
     * @note Timeout after 1000ms if no response received
     * @warning Only one parameter operation at a time; check return value
     */
    bool set_parameter_on_node(uint8_t node_id, const char *name, const string &value, ParamGetSetStringCb *cb);
    
    /**
     * @brief Get floating-point parameter from remote DroneCAN node
     * 
     * @details Sends uavcan.protocol.param.GetSet request to read parameter value
     *          from specified node. Operation is asynchronous; callback invoked with
     *          value when response received or timeout occurs.
     * 
     * @param[in] node_id Target node ID (1-127)
     * @param[in] name Parameter name (null-terminated string, max 92 chars)
     * @param[in] cb Callback function invoked with result (required, not nullptr)
     * 
     * @return true if request successfully queued
     * @return false if previous request still pending (only one request at a time)
     * 
     * @note Timeout after 1000ms if no response received
     * @warning Only one parameter operation at a time; check return value
     */
    bool get_parameter_on_node(uint8_t node_id, const char *name, ParamGetSetFloatCb *cb);
    
    /**
     * @brief Get integer parameter from remote DroneCAN node
     * 
     * @details Sends uavcan.protocol.param.GetSet request to read parameter value
     *          from specified node. Operation is asynchronous; callback invoked with
     *          value when response received or timeout occurs.
     * 
     * @param[in] node_id Target node ID (1-127)
     * @param[in] name Parameter name (null-terminated string, max 92 chars)
     * @param[in] cb Callback function invoked with result (required, not nullptr)
     * 
     * @return true if request successfully queued
     * @return false if previous request still pending (only one request at a time)
     * 
     * @note Timeout after 1000ms if no response received
     * @warning Only one parameter operation at a time; check return value
     */
    bool get_parameter_on_node(uint8_t node_id, const char *name, ParamGetSetIntCb *cb);
    
    /**
     * @brief Get string parameter from remote DroneCAN node
     * 
     * @details Sends uavcan.protocol.param.GetSet request to read parameter value
     *          from specified node. Operation is asynchronous; callback invoked with
     *          value when response received or timeout occurs.
     * 
     * @param[in] node_id Target node ID (1-127)
     * @param[in] name Parameter name (null-terminated string, max 92 chars)
     * @param[in] cb Callback function invoked with result (required, not nullptr)
     * 
     * @return true if request successfully queued
     * @return false if previous request still pending (only one request at a time)
     * 
     * @note Timeout after 1000ms if no response received
     * @warning Only one parameter operation at a time; check return value
     */
    bool get_parameter_on_node(uint8_t node_id, const char *name, ParamGetSetStringCb *cb);

    /**
     * @brief Save parameters to persistent storage on remote node
     * 
     * @details Sends uavcan.protocol.param.ExecuteOpcode request with OPCODE_SAVE
     *          to persist current parameter values to non-volatile memory (EEPROM/flash).
     *          Parameters modified by set_parameter_on_node() are not permanent until saved.
     * 
     * @param[in] node_id Target node ID (1-127)
     * @param[in] cb Callback function invoked when save completes (may be nullptr)
     * 
     * @return true if request successfully queued
     * @return false if previous save request still pending
     * 
     * @note Save operation may take several seconds depending on flash write speed
     * @note Some nodes may have limited write cycles; avoid excessive saves
     * @warning Only one save operation at a time per driver instance
     */
    bool save_parameters_on_node(uint8_t node_id, ParamSaveCb *cb);

    /**
     * @brief DroneCAN driver options bitmask
     * 
     * @details Configuration flags controlling driver behavior and feature enablement.
     *          Set via CAN_Dx_OPTIONS parameter where x is driver index (1-based).
     *          Multiple options can be combined with bitwise OR.
     */
    enum class Options : uint16_t {
        /**
         * @brief Clear DNA (Dynamic Node Allocation) database on boot
         * 
         * Forces all nodes to re-request node IDs, clearing saved allocations.
         * Useful for testing or recovering from corrupted DNA database.
         */
        DNA_CLEAR_DATABASE        = (1U<<0),
        
        /**
         * @brief Ignore duplicate node ID conflicts
         * 
         * Allows multiple nodes with same ID (non-standard, debugging only).
         * Normally duplicate IDs cause DNA server to reject node.
         * @warning May cause unpredictable behavior; use only for debugging
         */
        DNA_IGNORE_DUPLICATE_NODE = (1U<<1),
        
        /**
         * @brief Enable CAN FD (Flexible Data-rate) mode
         * 
         * Enables CAN FD for higher bandwidth (64-byte payloads, faster bit rates).
         * Requires CAN FD-capable hardware on all bus devices.
         * Falls back to CAN 2.0B if not supported by hardware.
         */
        CANFD_ENABLED             = (1U<<2),
        
        /**
         * @brief Ignore unhealthy nodes during DNA
         * 
         * Allows node ID allocation to proceed even if some nodes report errors.
         * Prevents healthy nodes from being blocked by failing devices.
         */
        DNA_IGNORE_UNHEALTHY_NODE = (1U<<3),
        
        /**
         * @brief Use Actuator PWM command format
         * 
         * Sends actuator commands as normalized values (-1.0 to +1.0) with metadata
         * instead of raw ESC commands (0-8191). Preferred for smart actuators that
         * support closed-loop control and can interpret normalized commands.
         */
        USE_ACTUATOR_PWM          = (1U<<4),
        
        /**
         * @brief Send GNSS data over DroneCAN
         * 
         * Broadcasts GPS position/velocity/status from flight controller to CAN bus.
         * Used when flight controller is GNSS source and peripherals need position
         * (e.g., redundant flight controllers, external payloads).
         * Sends Fix2, Auxiliary, Heading, and Status messages.
         */
        SEND_GNSS                 = (1U<<5),
        
        /**
         * @brief Enable Himark servo protocol support
         * 
         * Uses vendor-specific Himark ServoCmd messages instead of standard commands.
         * Required for Himark smart servos; do not use with other servo types.
         */
        USE_HIMARK_SERVO          = (1U<<6),
        
        /**
         * @brief Enable Hobbywing ESC protocol support
         * 
         * Uses vendor-specific Hobbywing ESC messages with enhanced telemetry.
         * Required for Hobbywing Platinum/Flyfun ESCs; includes ESC ID discovery.
         */
        USE_HOBBYWING_ESC         = (1U<<7),
        
        /**
         * @brief Enable CAN statistics broadcasting
         * 
         * Periodically broadcasts dronecan.protocol.Stats and CanStats messages
         * containing error counters, TX/RX counts, and buffer usage.
         * Useful for bus health monitoring and debugging.
         */
        ENABLE_STATS              = (1U<<8),
        
        /**
         * @brief Enable FlexDebug message reception
         * 
         * Subscribe to dronecan.protocol.FlexDebug messages for custom debugging.
         * Messages accessible to Lua scripts via get_FlexDebug() method.
         * Increases memory usage; enable only when debugging with scripts.
         */
        ENABLE_FLEX_DEBUG         = (1U<<9),
    };

    /**
     * @brief Check if specific option flag is set
     * 
     * @param[in] option Option flag to test (from Options enum)
     * 
     * @return true if option bit is set in _options parameter
     * @return false if option bit is clear
     * 
     * @note Thread-safe: reads parameter atomically
     */
    bool option_is_set(Options option) const {
        return (uint16_t(_options.get()) & uint16_t(option)) != 0;
    }

    /**
     * @brief Check and clear option flag atomically
     * 
     * @details Tests if option is set, and if so, clears the bit and returns true.
     *          Used for one-shot options like DNA_CLEAR_DATABASE that should only
     *          trigger once after being set.
     * 
     * @param[in] option Option flag to test and clear
     * 
     * @return true if option was set (now cleared)
     * @return false if option was already clear
     * 
     * @warning Modifies _options parameter; ensure proper synchronization
     */
    bool check_and_reset_option(Options option);

    /**
     * @brief Get reference to libcanard interface
     * 
     * @details Provides access to underlying Canard interface for direct protocol
     *          operations. Used by peripheral drivers and advanced integrations.
     * 
     * @return CanardInterface& Reference to libcanard interface wrapper
     * 
     * @warning Direct use of Canard interface bypasses driver abstractions
     * @note Most users should use higher-level driver methods instead
     */
    CanardInterface& get_canard_iface() { return canard_iface; }

    /**
     * @brief Publisher for RGB LED control commands
     * 
     * @details Sends uavcan.equipment.indication.LightsCommand messages to control
     *          addressable RGB LEDs on DroneCAN peripherals. Used by led_write().
     * 
     * @note Public to allow advanced users to send custom LED patterns
     */
    Canard::Publisher<uavcan_equipment_indication_LightsCommand> rgb_led{canard_iface};
    
    /**
     * @brief Publisher for buzzer/beeper commands
     * 
     * @details Sends uavcan.equipment.indication.BeepCommand messages to control
     *          audible buzzers on DroneCAN peripherals. Used by set_buzzer_tone().
     * 
     * @note Public to allow advanced users to send custom beep patterns
     */
    Canard::Publisher<uavcan_equipment_indication_BeepCommand> buzzer{canard_iface};
    
    /**
     * @brief Publisher for RTCM correction data stream
     * 
     * @details Broadcasts uavcan.equipment.gnss.RTCMStream messages containing
     *          RTK correction data from base station. DroneCAN GPS receivers
     *          subscribe to this for centimeter-level positioning accuracy.
     * 
     * @note Corrections typically from NTRIP caster or local base station
     * @note RTCM3 messages fragmented across multiple CAN frames if needed
     */
    Canard::Publisher<uavcan_equipment_gnss_RTCMStream> rtcm_stream{canard_iface};

#if HAL_MOUNT_XACTI_ENABLED
    /**
     * @brief Publisher for Xacti gimbal copter attitude status
     * 
     * @details Sends com.xacti.CopterAttStatus messages to Xacti camera gimbal
     *          with vehicle attitude for stabilization and target tracking.
     * 
     * @note Xacti-specific protocol, only used with Xacti camera systems
     */
    Canard::Publisher<com_xacti_CopterAttStatus> xacti_copter_att_status{canard_iface};
    
    /**
     * @brief Publisher for Xacti gimbal control commands
     * 
     * @details Sends com.xacti.GimbalControlData messages to control Xacti gimbal
     *          angles, modes, and camera settings.
     * 
     * @note Xacti-specific protocol, only used with Xacti camera systems
     */
    Canard::Publisher<com_xacti_GimbalControlData> xacti_gimbal_control_data{canard_iface};
    
    /**
     * @brief Publisher for Xacti GNSS status
     * 
     * @details Sends com.xacti.GnssStatus messages to Xacti camera with GPS
     *          position/status for geotagging and navigation features.
     * 
     * @note Xacti-specific protocol, only used with Xacti camera systems
     */
    Canard::Publisher<com_xacti_GnssStatus> xacti_gnss_status{canard_iface};
#endif  // HAL_MOUNT_XACTI_ENABLED

#if AP_RELAY_DRONECAN_ENABLED
    /**
     * @brief Publisher for relay/hardpoint commands
     * 
     * @details Sends uavcan.equipment.hardpoint.Command messages to control
     *          relays (switches) on DroneCAN devices. Supports both edge-triggered
     *          and level-based relay control.
     * 
     * @note Public to allow relay drivers to send both streaming and edge-trigger commands
     * @note Command includes relay ID (0-255) and state (0=off, 1=on)
     */
    Canard::Publisher<uavcan_equipment_hardpoint_Command> relay_hardpoint{canard_iface};
#endif

#if AP_SCRIPTING_ENABLED
    /**
     * @brief Get FlexDebug message from specific node
     * 
     * @details Retrieves stored FlexDebug message from specified node for Lua script access.
     *          FlexDebug provides custom debugging data from DroneCAN devices that can be
     *          processed by scripts for advanced monitoring and diagnostics.
     * 
     * @param[in]  node_id Node ID to query (1-127)
     * @param[in]  msg_id Message ID filter (0=any message)
     * @param[out] timestamp_us Timestamp when message received (microseconds)
     * @param[out] msg FlexDebug message structure
     * 
     * @return true if matching message found and returned
     * @return false if no message from node_id or no match for msg_id
     * 
     * @note Only available when Options::ENABLE_FLEX_DEBUG is set
     * @note Message history limited by memory pool size
     */
    bool get_FlexDebug(uint8_t node_id, uint16_t msg_id, uint32_t &timestamp_us, dronecan_protocol_FlexDebug &msg) const;
#endif

private:
    void loop(void);

    ///// SRV output /////
    void SRV_send_actuator();
    void SRV_send_esc();
#if AP_DRONECAN_HIMARK_SERVO_SUPPORT
    void SRV_send_himark();
#endif

    //scale servo output appropriately before sending
    int16_t scale_esc_output(uint8_t idx);

    // SafetyState
    void safety_state_send();

    // send notify vehicle state
    void notify_state_send();

    // check for parameter get/set response timeout
    void check_parameter_callback_timeout();

    // send queued parameter get/set request. called from loop
    void send_parameter_request();
    
    // send queued parameter save request. called from loop
    void send_parameter_save_request();

    // periodic logging
    void logging();
    
    // get parameter on a node
    ParamGetSetIntCb *param_int_cb;         // latest get param request callback function (for integers)
    ParamGetSetFloatCb *param_float_cb;     // latest get param request callback function (for floats)
    ParamGetSetStringCb *param_string_cb;   // latest get param request callback function (for strings)
    bool param_request_sent = true;         // true after a param request has been sent, false when queued to be sent
    uint32_t param_request_sent_ms;         // system time that get param request was sent
    HAL_Semaphore _param_sem;               // semaphore protecting this block of variables
    uint8_t param_request_node_id;          // node id of most recent get param request

    // save parameters on a node
    ParamSaveCb *save_param_cb;             // latest save param request callback function
    bool param_save_request_sent = true;    // true after a save param request has been sent, false when queued to be sent
    HAL_Semaphore _param_save_sem;          // semaphore protecting this block of variables
    uint8_t param_save_request_node_id;     // node id of most recent save param request

    // UAVCAN parameters
    AP_Int8 _dronecan_node;
    AP_Int32 _servo_bm;
    AP_Int32 _esc_bm;
    AP_Int8 _esc_offset;
    AP_Int16 _servo_rate_hz;
    AP_Int16 _options;
    AP_Int16 _notify_state_hz;
    AP_Int16 _pool_size;
    AP_Int32 _esc_rv;

    uint32_t *mem_pool;

    uint8_t _driver_index;

    CanardInterface canard_iface;

    AP_DroneCAN_DNA_Server _dna_server;

    char _thread_name[13];
    bool _initialized;
    ///// SRV output /////
    struct {
        uint16_t pulse;
        bool esc_pending;
        bool servo_pending;
    } _SRV_conf[DRONECAN_SRV_NUMBER];

    uint32_t _esc_send_count;
    uint32_t _srv_send_count;
    uint32_t _fail_send_count;

    uint32_t _SRV_armed_mask; // mask of servo outputs that are active
    uint32_t _ESC_armed_mask; // mask of ESC outputs that are active
    uint32_t _SRV_last_send_us;
    HAL_Semaphore SRV_sem;

    // last log time
    uint32_t last_log_ms;

#if AP_DRONECAN_SEND_GPS
    // send GNSS Fix and yaw, same thing AP_GPS_DroneCAN would receive
    void gnss_send_fix();
    void gnss_send_yaw();
    
    // GNSS Fix and Status
    struct {
        uint32_t last_gps_lib_fix_ms;
        uint32_t last_send_status_ms;
        uint32_t last_lib_yaw_time_ms;
    } _gnss;
#endif

    // node status send
    uint32_t _node_status_last_send_ms;

    // safety status send state
    uint32_t _last_safety_state_ms;

    // notify vehicle state
    uint32_t _last_notify_state_ms;
    uavcan_protocol_NodeStatus node_status_msg;

#if AP_RELAY_DRONECAN_ENABLED
    void relay_hardpoint_send();
    struct {
        AP_Int16 rate_hz;
        uint32_t last_send_ms;
        uint8_t last_index;
    } _relay;
#endif

#if AP_DRONECAN_SERIAL_ENABLED
    AP_DroneCAN_Serial serial;
#endif

    Canard::Publisher<uavcan_protocol_NodeStatus> node_status{canard_iface};
    Canard::Publisher<dronecan_protocol_CanStats> can_stats{canard_iface};
    Canard::Publisher<dronecan_protocol_Stats> protocol_stats{canard_iface};
    Canard::Publisher<uavcan_equipment_actuator_ArrayCommand> act_out_array{canard_iface};
    Canard::Publisher<uavcan_equipment_esc_RawCommand> esc_raw{canard_iface};
    Canard::Publisher<ardupilot_indication_SafetyState> safety_state{canard_iface};
    Canard::Publisher<uavcan_equipment_safety_ArmingStatus> arming_status{canard_iface};
    Canard::Publisher<ardupilot_indication_NotifyState> notify_state{canard_iface};

#if AP_DRONECAN_HIMARK_SERVO_SUPPORT
    Canard::Publisher<com_himark_servo_ServoCmd> himark_out{canard_iface};
#endif

#if AP_DRONECAN_SEND_GPS
    Canard::Publisher<uavcan_equipment_gnss_Fix2> gnss_fix2{canard_iface};
    Canard::Publisher<uavcan_equipment_gnss_Auxiliary> gnss_auxiliary{canard_iface};
    Canard::Publisher<ardupilot_gnss_Heading> gnss_heading{canard_iface};
    Canard::Publisher<ardupilot_gnss_Status> gnss_status{canard_iface};
#endif
    // incoming messages
    Canard::ObjCallback<AP_DroneCAN, ardupilot_indication_Button> safety_button_cb{this, &AP_DroneCAN::handle_button};
    Canard::Subscriber<ardupilot_indication_Button> safety_button_listener{safety_button_cb, _driver_index};

    Canard::ObjCallback<AP_DroneCAN, ardupilot_equipment_trafficmonitor_TrafficReport> traffic_report_cb{this, &AP_DroneCAN::handle_traffic_report};
    Canard::Subscriber<ardupilot_equipment_trafficmonitor_TrafficReport> traffic_report_listener{traffic_report_cb, _driver_index};

#if AP_SERVO_TELEM_ENABLED
    Canard::ObjCallback<AP_DroneCAN, uavcan_equipment_actuator_Status> actuator_status_cb{this, &AP_DroneCAN::handle_actuator_status};
    Canard::Subscriber<uavcan_equipment_actuator_Status> actuator_status_listener{actuator_status_cb, _driver_index};
#endif

    Canard::ObjCallback<AP_DroneCAN, uavcan_equipment_esc_Status> esc_status_cb{this, &AP_DroneCAN::handle_ESC_status};
    Canard::Subscriber<uavcan_equipment_esc_Status> esc_status_listener{esc_status_cb, _driver_index};

#if AP_EXTENDED_ESC_TELEM_ENABLED
    Canard::ObjCallback<AP_DroneCAN, uavcan_equipment_esc_StatusExtended> esc_status_extended_cb{this, &AP_DroneCAN::handle_esc_ext_status};
    Canard::Subscriber<uavcan_equipment_esc_StatusExtended> esc_status_extended_listener{esc_status_extended_cb, _driver_index};
#endif

    Canard::ObjCallback<AP_DroneCAN, uavcan_protocol_debug_LogMessage> debug_cb{this, &AP_DroneCAN::handle_debug};
    Canard::Subscriber<uavcan_protocol_debug_LogMessage> debug_listener{debug_cb, _driver_index};

#if AP_DRONECAN_HIMARK_SERVO_SUPPORT && AP_SERVO_TELEM_ENABLED
    Canard::ObjCallback<AP_DroneCAN, com_himark_servo_ServoInfo> himark_servo_ServoInfo_cb{this, &AP_DroneCAN::handle_himark_servoinfo};
    Canard::Subscriber<com_himark_servo_ServoInfo> himark_servo_ServoInfo_cb_listener{himark_servo_ServoInfo_cb, _driver_index};
#endif
#if AP_DRONECAN_VOLZ_FEEDBACK_ENABLED
    Canard::ObjCallback<AP_DroneCAN, com_volz_servo_ActuatorStatus> volz_servo_ActuatorStatus_cb{this, &AP_DroneCAN::handle_actuator_status_Volz};
    Canard::Subscriber<com_volz_servo_ActuatorStatus> volz_servo_ActuatorStatus_listener{volz_servo_ActuatorStatus_cb, _driver_index};
#endif

    // param client
    Canard::ObjCallback<AP_DroneCAN, uavcan_protocol_param_GetSetResponse> param_get_set_res_cb{this, &AP_DroneCAN::handle_param_get_set_response};
    Canard::Client<uavcan_protocol_param_GetSetResponse> param_get_set_client{canard_iface, param_get_set_res_cb};

    Canard::ObjCallback<AP_DroneCAN, uavcan_protocol_param_ExecuteOpcodeResponse> param_save_res_cb{this, &AP_DroneCAN::handle_param_save_response};
    Canard::Client<uavcan_protocol_param_ExecuteOpcodeResponse> param_save_client{canard_iface, param_save_res_cb};

    // reboot client
    void handle_restart_node_response(const CanardRxTransfer& transfer, const uavcan_protocol_RestartNodeResponse& msg) {}
    Canard::ObjCallback<AP_DroneCAN, uavcan_protocol_RestartNodeResponse> restart_node_res_cb{this, &AP_DroneCAN::handle_restart_node_response};
    Canard::Client<uavcan_protocol_RestartNodeResponse> restart_node_client{canard_iface, restart_node_res_cb};

    uavcan_protocol_param_ExecuteOpcodeRequest param_save_req;
    uavcan_protocol_param_GetSetRequest param_getset_req;

    // Node Info Server
    Canard::ObjCallback<AP_DroneCAN, uavcan_protocol_GetNodeInfoRequest> node_info_req_cb{this, &AP_DroneCAN::handle_node_info_request};
    Canard::Server<uavcan_protocol_GetNodeInfoRequest> node_info_server{canard_iface, node_info_req_cb};
    uavcan_protocol_GetNodeInfoResponse node_info_rsp;

#if AP_SCRIPTING_ENABLED
    Canard::ObjCallback<AP_DroneCAN, dronecan_protocol_FlexDebug> FlexDebug_cb{this, &AP_DroneCAN::handle_FlexDebug};
    Canard::Subscriber<dronecan_protocol_FlexDebug> FlexDebug_listener{FlexDebug_cb, _driver_index};
#endif
    
#if AP_DRONECAN_HOBBYWING_ESC_SUPPORT
    /*
      Hobbywing ESC support. Note that we need additional meta-data as
      the status messages do not have an ESC ID in them, so we need a
      mapping from node ID
    */
    #define HOBBYWING_MAX_ESC 8
    struct {
        uint32_t last_GetId_send_ms;
        uint8_t thr_chan[HOBBYWING_MAX_ESC];
    } hobbywing;
    void hobbywing_ESC_update();

    void SRV_send_esc_hobbywing();
    Canard::Publisher<com_hobbywing_esc_RawCommand> esc_hobbywing_raw{canard_iface};
    Canard::Publisher<com_hobbywing_esc_GetEscID> esc_hobbywing_GetEscID{canard_iface};
    Canard::ObjCallback<AP_DroneCAN, com_hobbywing_esc_GetEscID> esc_hobbywing_GetEscID_cb{this, &AP_DroneCAN::handle_hobbywing_GetEscID};
    Canard::Subscriber<com_hobbywing_esc_GetEscID> esc_hobbywing_GetEscID_listener{esc_hobbywing_GetEscID_cb, _driver_index};
    Canard::ObjCallback<AP_DroneCAN, com_hobbywing_esc_StatusMsg1> esc_hobbywing_StatusMSG1_cb{this, &AP_DroneCAN::handle_hobbywing_StatusMsg1};
    Canard::Subscriber<com_hobbywing_esc_StatusMsg1> esc_hobbywing_StatusMSG1_listener{esc_hobbywing_StatusMSG1_cb, _driver_index};
    Canard::ObjCallback<AP_DroneCAN, com_hobbywing_esc_StatusMsg2> esc_hobbywing_StatusMSG2_cb{this, &AP_DroneCAN::handle_hobbywing_StatusMsg2};
    Canard::Subscriber<com_hobbywing_esc_StatusMsg2> esc_hobbywing_StatusMSG2_listener{esc_hobbywing_StatusMSG2_cb, _driver_index};
    bool hobbywing_find_esc_index(uint8_t node_id, uint8_t &esc_index) const;
    void handle_hobbywing_GetEscID(const CanardRxTransfer& transfer, const com_hobbywing_esc_GetEscID& msg);
    void handle_hobbywing_StatusMsg1(const CanardRxTransfer& transfer, const com_hobbywing_esc_StatusMsg1& msg);
    void handle_hobbywing_StatusMsg2(const CanardRxTransfer& transfer, const com_hobbywing_esc_StatusMsg2& msg);
#endif // AP_DRONECAN_HOBBYWING_ESC_SUPPORT

#if AP_DRONECAN_HIMARK_SERVO_SUPPORT && AP_SERVO_TELEM_ENABLED
    void handle_himark_servoinfo(const CanardRxTransfer& transfer, const com_himark_servo_ServoInfo &msg);
#endif

    // incoming button handling
    void handle_button(const CanardRxTransfer& transfer, const ardupilot_indication_Button& msg);
    void handle_traffic_report(const CanardRxTransfer& transfer, const ardupilot_equipment_trafficmonitor_TrafficReport& msg);
#if AP_SERVO_TELEM_ENABLED
    void handle_actuator_status(const CanardRxTransfer& transfer, const uavcan_equipment_actuator_Status& msg);
#endif
#if AP_DRONECAN_VOLZ_FEEDBACK_ENABLED
    void handle_actuator_status_Volz(const CanardRxTransfer& transfer, const com_volz_servo_ActuatorStatus& msg);
#endif
    void handle_ESC_status(const CanardRxTransfer& transfer, const uavcan_equipment_esc_Status& msg);
#if AP_EXTENDED_ESC_TELEM_ENABLED
    void handle_esc_ext_status(const CanardRxTransfer& transfer, const uavcan_equipment_esc_StatusExtended& msg);
#endif
    static bool is_esc_data_index_valid(const uint8_t index);
    void handle_debug(const CanardRxTransfer& transfer, const uavcan_protocol_debug_LogMessage& msg);
    void handle_param_get_set_response(const CanardRxTransfer& transfer, const uavcan_protocol_param_GetSetResponse& rsp);
    void handle_param_save_response(const CanardRxTransfer& transfer, const uavcan_protocol_param_ExecuteOpcodeResponse& rsp);
    void handle_node_info_request(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoRequest& req);

#if AP_SCRIPTING_ENABLED
    void handle_FlexDebug(const CanardRxTransfer& transfer, const dronecan_protocol_FlexDebug& msg);
    struct FlexDebug {
        struct FlexDebug *next;
        uint32_t timestamp_us;
        uint8_t node_id;
        dronecan_protocol_FlexDebug msg;
    } *flexDebug_list;
#endif
};

#endif // #if HAL_ENABLE_DRONECAN_DRIVERS
