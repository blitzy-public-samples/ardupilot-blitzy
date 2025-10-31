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
 * @file AP_Scripting.h
 * @brief Main AP_Scripting subsystem managing Lua VM thread lifecycle, parameter configuration, and hardware resource allocation
 * 
 * This file defines the AP_Scripting singleton class which manages the entire scripting subsystem
 * in ArduPilot. The scripting system allows users to extend vehicle behavior using Lua scripts
 * without recompiling the firmware.
 * 
 * Key responsibilities:
 * - Initialize and manage the Lua VM execution thread
 * - Allocate and track hardware resources (I2C devices, PWM sources, network sockets)
 * - Handle MAVLink message routing to scripts
 * - Process mission commands that scripts have registered for
 * - Perform pre-arm checks for script health
 * - Manage script loading from ROMFS and SD card directories
 * 
 * The subsystem uses a dedicated thread for script execution separate from the main flight
 * control thread to prevent scripts from blocking critical flight operations.
 * 
 * @warning Scripts run with significant vehicle control authority. Poorly written scripts
 *          can cause vehicle crashes or loss of control.
 * 
 * Source: libraries/AP_Scripting/AP_Scripting.h
 */

#pragma once

#include "AP_Scripting/AP_Scripting_config.h"

#if AP_SCRIPTING_ENABLED

#include <GCS_MAVLink/GCS_config.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_HAL/I2CDevice.h>
#include "AP_Scripting_CANSensor.h"
#include <AP_Networking/AP_Networking_Config.h>

#ifndef SCRIPTING_MAX_NUM_I2C_DEVICE
  #define SCRIPTING_MAX_NUM_I2C_DEVICE 4
#endif

#define SCRIPTING_MAX_NUM_PWM_SOURCE 4

#if AP_NETWORKING_ENABLED
#ifndef SCRIPTING_MAX_NUM_NET_SOCKET
#define SCRIPTING_MAX_NUM_NET_SOCKET 50
#endif
class SocketAPM;
#endif

#if AP_SCRIPTING_SERIALDEVICE_ENABLED
#include "AP_Scripting_SerialDevice.h"
#endif

/**
 * @class AP_Scripting
 * @brief Singleton managing the ArduPilot Lua scripting subsystem
 * 
 * @details This class manages the complete lifecycle of the Lua scripting environment:
 * - Subsystem initialization and thread creation
 * - Script loading from filesystem (ROMFS and SD card)
 * - Hardware resource allocation (I2C, PWM, CAN, network sockets)
 * - MAVLink message routing to scripts
 * - Mission command event notification
 * - Pre-arm validation checks
 * - Runtime configuration via parameters
 * 
 * Thread Architecture:
 * The scripting subsystem uses a dual-thread model:
 * - Main thread: Calls update(), handle_message(), handle_mission_command() and other
 *   integration methods at scheduler rate
 * - Scripting thread: Executes Lua VM bytecode in thread() method, running continuously
 *   with yielding to prevent blocking
 * 
 * Resource Management:
 * Hardware resources are allocated on-demand as scripts request them through bindings.
 * Arrays are fixed-size with compile-time maximums to prevent unbounded memory growth:
 * - I2C devices: SCRIPTING_MAX_NUM_I2C_DEVICE (default 4)
 * - PWM sources: SCRIPTING_MAX_NUM_PWM_SOURCE (4)
 * - Network sockets: SCRIPTING_MAX_NUM_NET_SOCKET (default 50)
 * 
 * Memory allocation uses NEW_NOTHROW throughout - scripts must check for nullptr returns.
 * 
 * MAVLink Integration:
 * Scripts can register to receive specific MAVLink message IDs and can block command
 * execution to handle commands within Lua code. Message buffering uses ObjectBuffer
 * with semaphore protection for thread-safe access.
 * 
 * @note This is a singleton class accessed via AP::scripting() or get_singleton()
 * 
 * @warning Script execution errors can affect vehicle behavior. The pre-arm check
 *          validates scripts loaded successfully unless disabled via debug options.
 * 
 * @warning All hardware resource arrays are allocated with NEW_NOTHROW. Scripts must
 *          handle allocation failures gracefully.
 * 
 * @see AP_Scripting::init() for initialization sequence
 * @see AP_Scripting::thread() for main script execution
 * 
 * Source: libraries/AP_Scripting/AP_Scripting.h:48-213
 */
class AP_Scripting
{
public:
    /**
     * @brief Construct the AP_Scripting singleton
     * 
     * @details Initializes member variables to default states and registers the singleton
     *          instance. Actual subsystem initialization occurs in init().
     * 
     * @note Constructor does not allocate resources or start threads
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.cpp
     */
    AP_Scripting();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Scripting);

    /**
     * @brief Initialize the scripting subsystem
     * 
     * @details Performs complete initialization sequence:
     * 1. Check if scripting is enabled via SCR_ENABLE parameter
     * 2. Allocate heap memory for Lua VM (size from SCR_HEAP_SIZE parameter)
     * 3. Initialize filesystem access for script loading
     * 4. Allocate MAVLink message buffer
     * 5. Allocate mission command buffer (if AP_MISSION_ENABLED)
     * 6. Create and start the dedicated scripting thread
     * 
     * If any allocation fails, sets _init_failed flag and scripting will not run.
     * Thread priority is configured via SCR_THD_PRIORITY parameter.
     * 
     * @note Called once during vehicle initialization, typically from AP_Vehicle::init_ardupilot()
     * @note Must be called before update() or any other subsystem methods
     * 
     * @warning Memory allocation failures will disable scripting but won't prevent vehicle operation
     * 
     * @see AP_Scripting::enabled() to check if initialization succeeded
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.cpp
     */
    void init(void);

#if AP_SCRIPTING_SERIALDEVICE_ENABLED
    /**
     * @brief Initialize serial device ports for scripting access
     * 
     * @details Sets up serial port bindings that scripts can access through the serial
     *          device API. Configures port parameters and registers ports with the
     *          scripting VM for script access.
     * 
     * @note Only available when AP_SCRIPTING_SERIALDEVICE_ENABLED is defined
     * @note Called after init() if serial device support is compiled in
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.cpp
     */
    void init_serialdevice_ports(void);
#endif

    /**
     * @brief Main thread periodic update hook
     * 
     * @details Called at scheduler rate from the main flight control thread. Handles:
     * - Checking for script restart requests (_restart flag)
     * - Processing debug option changes (checksum save requests)
     * - Monitoring scripting thread health
     * - Coordinating with the dedicated scripting thread
     * 
     * Does NOT execute scripts directly - script execution happens in the thread() method
     * running on the dedicated scripting thread.
     * 
     * @note Called at main loop rate (typically 50-400Hz depending on vehicle)
     * @note This is a lightweight call - heavy work is on the scripting thread
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.cpp
     */
    void update();

    /**
     * @brief Check if scripting subsystem is enabled
     * 
     * @details Returns true if the SCR_ENABLE parameter is non-zero, indicating that
     *          scripting should be active. Does not check for initialization failures.
     * 
     * @return true if SCR_ENABLE parameter is non-zero, false otherwise
     * 
     * @note This checks the parameter value only, not initialization success
     * @see should_run() for comprehensive runtime state check
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:64
     */
    bool enabled(void) const { return _enable != 0; };
    
    /**
     * @brief Check if scripts should be running
     * 
     * @details Returns true if scripting is enabled AND not stopped. This is the
     *          comprehensive check for whether the scripting VM should be executing.
     *          Accounts for both the enable parameter and runtime stop flag.
     * 
     * @return true if enabled and not stopped, false otherwise
     * 
     * @see enabled() to check just the parameter
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:65
     */
    bool should_run(void) const { return enabled() && !_stop; }

#if HAL_GCS_ENABLED
    /**
     * @brief Dispatch MAVLink message to scripts
     * 
     * @details Called by GCS_MAVLink when a message is received that scripts have
     *          registered interest in. Messages are queued in the rx_buffer for scripts
     *          to retrieve via the mavlink:receive_msg() binding.
     * 
     *          Only messages with IDs in the accept_msg_ids array are queued.
     *          Buffer is protected by semaphore for thread-safe access between
     *          main thread (this method) and scripting thread.
     * 
     * @param[in] msg MAVLink message structure containing message data
     * @param[in] chan MAVLink channel the message was received on
     * 
     * @note Called from main thread at message receipt time
     * @note Messages are timestamped with AP_HAL::millis() at queue time
     * 
     * @warning Buffer overflow will discard oldest messages
     * 
     * @see mavlink_data for message buffer structure
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.cpp
     */
    void handle_message(const mavlink_message_t &msg, const mavlink_channel_t chan);

    /**
     * @brief Check if a MAVLink command ID is blocked by scripts
     * 
     * @details Searches the mavlink_command_block_list to determine if a script has
     *          registered to handle this command ID. If blocked, the command will be
     *          routed to scripts via handle_command_int_packet() instead of normal
     *          command processing.
     * 
     * @param[in] cmd_id MAVLink command ID to check (e.g., MAV_CMD_DO_SET_MODE)
     * 
     * @return true if command is in the block list (scripts handling it), false otherwise
     * 
     * @note Thread-safe: Protected by mavlink_command_block_list_sem
     * @note Block list is managed by scripts via command_int_register() binding
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.cpp
     */
    bool is_handling_command(uint16_t cmd_id);
#endif

    /**
     * @brief Get the singleton instance of AP_Scripting
     * 
     * @return Pointer to the singleton instance, or nullptr if not initialized
     * 
     * @note Prefer using AP::scripting() accessor from AP namespace
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:74
     */
    static AP_Scripting * get_singleton(void) { return _singleton; }

    /**
     * @brief Parameter table for AP_Scripting configuration
     * 
     * @details Defines the following parameters:
     * - SCR_ENABLE: Enable/disable scripting (0=disabled, 1=enabled)
     * - SCR_VM_I_COUNT: VM instruction count per script execution slice
     * - SCR_HEAP_SIZE: Heap memory allocated for Lua VM in bytes
     * - SCR_DEBUG: Debug option flags (bitmask of DebugOption values)
     * - SCR_DIR_DISABLE: Disable script directories (bitmask of SCR_DIR values)
     * - SCR_LD_CHECKSUM: Required checksum for loaded scripts (validation)
     * - SCR_RUN_CHECKSUM: Required checksum for running scripts (validation)
     * - SCR_THD_PRIORITY: Scripting thread priority (ThreadPriority enum)
     * - SCR_USER1 through SCR_USER6: User-defined float parameters accessible to scripts
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.cpp
     */
    static const struct AP_Param::GroupInfo var_info[];

#if HAL_GCS_ENABLED
    /**
     * @brief Handle MAVLink COMMAND_INT packet from scripts
     * 
     * @details Called when a MAVLink COMMAND_INT is received and the command ID is
     *          in the script block list. Routes the command to scripts for processing
     *          via the Lua command_int callback.
     * 
     *          Scripts must return a MAV_RESULT value indicating acceptance, rejection,
     *          or in-progress status. Timeout handling is managed by the MAVLink layer.
     * 
     * @param[in] packet MAVLink COMMAND_INT packet structure with command parameters
     * 
     * @return MAV_RESULT MAV_RESULT_ACCEPTED, MAV_RESULT_DENIED, MAV_RESULT_IN_PROGRESS,
     *         or MAV_RESULT_UNSUPPORTED based on script handling
     * 
     * @note Only called if is_handling_command() returned true for this command ID
     * @note Runs on main thread - scripts should process quickly or defer work
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.cpp
     */
    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet);
#endif

    /**
     * @brief Notify scripts of mission command execution
     * 
     * @details Called by AP_Mission when a mission command is being executed that
     *          scripts have registered interest in via mission:cmd_id_register().
     *          Command data is queued in mission_data buffer for scripts to retrieve.
     * 
     *          Only certain mission command parameters are exposed to scripts:
     *          - p1 (command-specific parameter 1)
     *          - content.location (as separate lat/lon/alt fields)
     *          - Timestamp of command execution
     * 
     * @param[in] cmd Mission command structure from AP_Mission
     * 
     * @note Called from main thread during mission execution
     * @note Buffer overflow discards oldest unprocessed mission commands
     * 
     * @warning Available only if AP_MISSION_ENABLED is defined
     * 
     * @see scripting_mission_cmd for queued data structure
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.cpp
     */
    void handle_mission_command(const class AP_Mission::Mission_Command& cmd);

    /**
     * @brief Pre-arm check for scripting subsystem health
     * 
     * @details Validates scripting subsystem state before arming:
     * - Checks if required scripts loaded successfully (checksum validation)
     * - Verifies VM initialized without errors
     * - Confirms no script runtime failures occurred
     * 
     *          Can be disabled via DEBUG_OPTS DISABLE_PRE_ARM flag for development.
     * 
     * @param[in] buflen Maximum length of error message buffer in bytes
     * @param[out] buffer Character buffer to write failure reason if check fails
     * 
     * @return true if pre-arm check passes, false if scripting not ready to arm
     * 
     * @note Called by AP_Arming during pre-arm check sequence
     * @note Failure message written to buffer is displayed to pilot
     * 
     * @warning Disabling pre-arm checks via debug options can allow arming with broken scripts
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.cpp
     */
    bool arming_checks(size_t buflen, char *buffer) const;
    
    /**
     * @brief Restart all scripts without vehicle reboot
     * 
     * @details Sets the _restart flag which causes the scripting thread to:
     * 1. Stop all currently running scripts
     * 2. Reset the Lua VM state
     * 3. Reload all scripts from filesystem
     * 4. Resume script execution
     * 
     *          Useful for script development and testing. Can be triggered via
     *          MAVLink or other mechanisms.
     * 
     * @note Non-blocking: sets flag and returns immediately
     * @note Actual restart happens asynchronously on scripting thread
     * 
     * @warning All script state is lost - variables, timers, etc. are reset
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.cpp
     */
    void restart_all(void);

    /**
     * @brief User-defined parameters accessible to scripts
     * 
     * @details Array of 6 float parameters (SCR_USER1 through SCR_USER6) that can be
     *          set via ground station and read by scripts. Provides a simple way to
     *          pass configuration values or runtime inputs to scripts without modifying
     *          script code.
     * 
     *          Scripts access these via the param:get() and param:set() bindings using
     *          parameter names "SCR_USER1", "SCR_USER2", etc.
     * 
     * @note Public member to allow direct access from scripts
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:89
     */
    AP_Float _user[6];

    /**
     * @enum SCR_DIR
     * @brief Script directory source flags
     * 
     * @details Bitmask values for SCR_DIR_DISABLE parameter to selectively disable
     *          script loading from different filesystem locations:
     * - ROMFS: Scripts embedded in firmware ROMFS (read-only filesystem)
     * - SCRIPTS: Scripts on SD card in APM/scripts/ directory
     * 
     *          Disabling directories can be useful for debugging or to prevent
     *          loading scripts from untrusted SD cards.
     * 
     * @note Values are bit positions, can be combined with bitwise OR
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:91-94
     */
    enum class SCR_DIR {
        ROMFS = 1 << 0,      ///< Firmware ROMFS scripts directory
        SCRIPTS = 1 << 1,    ///< SD card APM/scripts/ directory
    };
    
    /**
     * @brief Get disabled script directories bitmask
     * 
     * @return Bitmask of SCR_DIR values indicating which directories are disabled
     * 
     * @note Used internally during script loading to skip disabled directories
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:95
     */
    uint16_t get_disabled_dir() { return uint16_t(_dir_disable.get());}

    /**
     * @brief Number of I2C devices currently allocated
     * 
     * @details Count of active I2C device handles in _i2c_dev array. Incremented when
     *          scripts call i2c:get_device() and allocate a new device.
     * 
     * @note Maximum value is SCRIPTING_MAX_NUM_I2C_DEVICE (default 4)
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:98
     */
    uint8_t num_i2c_devices;
    
    /**
     * @brief Storage array for I2C device pointers
     * 
     * @details Fixed-size array holding I2C device handles allocated by scripts via
     *          i2c:get_device() binding. Each device represents a connection to a
     *          specific I2C address on a specific bus.
     * 
     *          Allocation uses NEW_NOTHROW - scripts must check for nullptr.
     * 
     * @warning Array size is compile-time fixed at SCRIPTING_MAX_NUM_I2C_DEVICE (default 4)
     * @warning Scripts attempting to allocate more devices than the limit will receive nullptr
     * 
     * @note Devices are not automatically freed - persist for VM lifetime
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:99
     */
    AP_HAL::I2CDevice *_i2c_dev[SCRIPTING_MAX_NUM_I2C_DEVICE];

#if AP_SCRIPTING_CAN_SENSOR_ENABLED
    /**
     * @brief Scripting CAN sensor interface - primary device
     * 
     * @details Pointer to first ScriptingCANSensor instance for CAN bus communication.
     *          Allows scripts to send and receive CAN frames on DroneCAN/UAVCAN networks.
     * 
     * @note Only available when AP_SCRIPTING_CAN_SENSOR_ENABLED is defined
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:103
     */
    ScriptingCANSensor *_CAN_dev;
    
    /**
     * @brief Scripting CAN sensor interface - secondary device
     * 
     * @details Pointer to second ScriptingCANSensor instance for accessing a second
     *          CAN bus interface from scripts.
     * 
     * @note Only available when AP_SCRIPTING_CAN_SENSOR_ENABLED is defined
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:104
     */
    ScriptingCANSensor *_CAN_dev2;
#endif

#if AP_MISSION_ENABLED
    /**
     * @brief Maximum mission command queue size
     * 
     * @details Fixed size for mission_data ObjectBuffer. Limits the number of mission
     *          commands that can be queued for script processing before overflow.
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:109
     */
    static const int mission_cmd_queue_size = 5;
    
    /**
     * @struct scripting_mission_cmd
     * @brief Mission command data structure for script notification
     * 
     * @details Simplified mission command format passed to scripts when a registered
     *          mission command executes. Contains subset of full Mission_Command data:
     * 
     * @var scripting_mission_cmd::p1
     *      Command-specific parameter 1 (uint16_t from mission item)
     * 
     * @var scripting_mission_cmd::content_p1
     *      Command content parameter 1 (typically latitude or x-coordinate)
     * 
     * @var scripting_mission_cmd::content_p2
     *      Command content parameter 2 (typically longitude or y-coordinate)
     * 
     * @var scripting_mission_cmd::content_p3
     *      Command content parameter 3 (typically altitude or z-coordinate)
     * 
     * @var scripting_mission_cmd::time_ms
     *      Timestamp of command execution in milliseconds (AP_HAL::millis())
     * 
     * @note Simplified format exposes only commonly needed mission parameters
     * @note Scripts retrieve via mission:receive() binding
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:110-116
     */
    struct scripting_mission_cmd {
        uint16_t p1;
        float content_p1;
        float content_p2;
        float content_p3;
        uint32_t time_ms;
    };
    
    /**
     * @brief Mission command queue buffer
     * 
     * @details ObjectBuffer holding up to mission_cmd_queue_size mission commands
     *          waiting for script processing. Commands are pushed by handle_mission_command()
     *          and popped by scripts via mission:receive() binding.
     * 
     * @warning Buffer overflow discards oldest commands
     * @note Only available when AP_MISSION_ENABLED is defined
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:117
     */
    ObjectBuffer<struct scripting_mission_cmd> * mission_data;
#endif

    /**
     * @brief Number of PWM source devices currently allocated
     * 
     * @details Count of active PWM source handles in _pwm_source array. Incremented
     *          when scripts call PWMSource() constructor to monitor PWM inputs.
     * 
     * @note Maximum value is SCRIPTING_MAX_NUM_PWM_SOURCE (4)
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:121
     */
    uint8_t num_pwm_source;
    
    /**
     * @brief Storage array for PWM source pointers
     * 
     * @details Fixed-size array holding PWM source device handles allocated by scripts.
     *          Each PWMSource monitors PWM input signal on a GPIO pin to measure pulse
     *          width and frequency (e.g., for RC receivers, sensors, external triggers).
     * 
     *          Allocation uses NEW_NOTHROW - scripts must check for nullptr.
     * 
     * @warning Array size is compile-time fixed at SCRIPTING_MAX_NUM_PWM_SOURCE (4)
     * @warning Attempting to allocate more sources than the limit will return nullptr
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:122
     */
    AP_HAL::PWMSource *_pwm_source[SCRIPTING_MAX_NUM_PWM_SOURCE];
    
    /**
     * @brief Get current Lua environment reference
     * 
     * @return Lua registry reference to current environment table
     * 
     * @note Used internally by script bindings for environment isolation
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:123
     */
    int get_current_env_ref() { return current_env_ref; }
    
    /**
     * @brief Set current Lua environment reference
     * 
     * @param[in] ref Lua registry reference to set as current environment
     * 
     * @note Used internally during script loading and execution context switches
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:124
     */
    void set_current_env_ref(int ref) { current_env_ref = ref; }

#if AP_NETWORKING_ENABLED
    /**
     * @brief Storage array for network socket pointers
     * 
     * @details Fixed-size array holding SocketAPM handles allocated by scripts via
     *          Socket() constructor. Supports TCP and UDP network communication from
     *          scripts on platforms with networking support (e.g., Linux, SITL, ESP32).
     * 
     *          Allocation uses NEW_NOTHROW - scripts must check for nullptr.
     * 
     * @warning Array size is compile-time fixed at SCRIPTING_MAX_NUM_NET_SOCKET (default 50)
     * @warning Attempting to allocate more sockets than the limit will return nullptr
     * 
     * @note Only available when AP_NETWORKING_ENABLED is defined
     * @note Platform must support BSD-style sockets
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:128
     */
    SocketAPM *_net_sockets[SCRIPTING_MAX_NUM_NET_SOCKET];
#endif

    /**
     * @struct mavlink_msg
     * @brief Timestamped MAVLink message for script queue
     * 
     * @details Wraps a MAVLink message with channel and timestamp information for
     *          queuing to scripts. Scripts retrieve messages via mavlink:receive_msg()
     *          binding.
     * 
     * @var mavlink_msg::msg
     *      Complete MAVLink message structure (headers, payload, checksum)
     * 
     * @var mavlink_msg::chan
     *      MAVLink channel the message was received on (e.g., MAVLINK_COMM_0)
     * 
     * @var mavlink_msg::timestamp_ms
     *      System timestamp when message was received (AP_HAL::millis())
     * 
     * @note Timestamp allows scripts to detect stale messages or calculate message age
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:131-135
     */
    struct mavlink_msg {
        mavlink_message_t msg;
        mavlink_channel_t chan;
        uint32_t timestamp_ms;
    };

    /**
     * @struct mavlink
     * @brief MAVLink message queue and filtering configuration
     * 
     * @details Manages the message queue between GCS_MAVLink and scripts. Only messages
     *          with IDs in accept_msg_ids array are queued. Thread-safe access is
     *          protected by semaphore.
     * 
     * @var mavlink::rx_buffer
     *      ObjectBuffer queue holding received mavlink_msg structures. Scripts pop
     *      messages via mavlink:receive_msg() binding.
     * 
     * @var mavlink::accept_msg_ids
     *      Array of MAVLink message IDs that scripts have registered interest in via
     *      mavlink:register_rx_msgid(). Only matching messages are queued.
     * 
     * @var mavlink::accept_msg_ids_size
     *      Number of valid entries in accept_msg_ids array. Grows dynamically as
     *      scripts register for additional message IDs.
     * 
     * @var mavlink::sem
     *      Semaphore protecting concurrent access to rx_buffer and accept_msg_ids
     *      between main thread (message receipt) and scripting thread (message consumption).
     * 
     * @warning Buffer overflow discards oldest messages
     * @note Message filtering prevents unnecessary queuing of unwanted message types
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:137-142
     */
    struct mavlink {
        ObjectBuffer<struct mavlink_msg> *rx_buffer;
        uint32_t *accept_msg_ids;
        uint16_t accept_msg_ids_size;
        HAL_Semaphore sem;
    } mavlink_data;

    /**
     * @struct command_block_list
     * @brief Linked list node for blocked MAVLink command IDs
     * 
     * @details Simple singly-linked list storing command IDs that scripts have
     *          registered to handle via command_int_register() binding. When a
     *          COMMAND_INT packet arrives, is_handling_command() checks this list.
     * 
     * @var command_block_list::id
     *      MAVLink command ID (e.g., MAV_CMD_DO_SET_MODE = 176)
     * 
     * @var command_block_list::next
     *      Pointer to next node in list, or nullptr if end of list
     * 
     * @note Linked list allows dynamic growth without fixed-size array limits
     * @note List is not sorted - search is linear O(n)
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:144-147
     */
    struct command_block_list {
        uint16_t id;
        command_block_list *next;
    };
    
    /**
     * @brief Head of command block list linked list
     * 
     * @details Pointer to first node in the command_block_list, or nullptr if empty.
     *          Protected by mavlink_command_block_list_sem for thread-safe access.
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:148
     */
    command_block_list *mavlink_command_block_list;
    
    /**
     * @brief Semaphore protecting command block list
     * 
     * @details Ensures thread-safe access to mavlink_command_block_list between
     *          scripting thread (adding/removing blocks) and main thread (checking blocks).
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:149
     */
    HAL_Semaphore mavlink_command_block_list_sem;

    #if AP_SCRIPTING_SERIALDEVICE_ENABLED
        /**
         * @brief Serial device manager for script access
         * 
         * @details Manages serial port bindings that scripts can access for custom
         *          serial communication (e.g., sensors, displays, external devices).
         *          Initialized by init_serialdevice_ports().
         * 
         * @note Only available when AP_SCRIPTING_SERIALDEVICE_ENABLED is defined
         * 
         * Source: libraries/AP_Scripting/AP_Scripting.h:152
         */
        AP_Scripting_SerialDevice _serialdevice;
    #endif

    /**
     * @enum DebugOption
     * @brief Debug flags for scripting subsystem (SCR_DEBUG parameter)
     * 
     * @details Bitmask values for SCR_DEBUG parameter to enable debugging features:
     * 
     * NO_SCRIPTS_TO_RUN (bit 0):
     *   Log warning message if no scripts are found or loaded. Helps diagnose
     *   script loading issues.
     * 
     * RUNTIME_MSG (bit 1):
     *   Log runtime statistics messages including heap usage, instruction count,
     *   and execution time per script.
     * 
     * SUPPRESS_SCRIPT_LOG (bit 2):
     *   Suppress log messages generated by scripts via gcs:send_text(). Useful
     *   for reducing log spam during script development.
     * 
     * LOG_RUNTIME (bit 3):
     *   Write script runtime statistics to onboard log (SCRP log messages).
     *   Enables performance analysis from log files.
     * 
     * DISABLE_PRE_ARM (bit 4):
     *   Disable pre-arm checks for scripting subsystem. Allows arming even if
     *   scripts failed to load or encountered errors.
     * 
     * SAVE_CHECKSUM (bit 5):
     *   Trigger-once flag to save current script checksums to SCR_LD_CHECKSUM
     *   and SCR_RUN_CHECKSUM parameters. Used to record known-good checksums.
     * 
     * DISABLE_HEAP_EXPANSION (bit 6):
     *   Prevent Lua VM from expanding heap beyond initial SCR_HEAP_SIZE allocation.
     *   Enforces strict memory limits.
     * 
     * @warning Disabling pre-arm checks can allow arming with broken scripts
     * @note Multiple flags can be combined using bitwise OR
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:155-163
     */
    enum class DebugOption : uint8_t {
        NO_SCRIPTS_TO_RUN = 1U << 0,
        RUNTIME_MSG = 1U << 1,
        SUPPRESS_SCRIPT_LOG = 1U << 2,
        LOG_RUNTIME = 1U << 3,
        DISABLE_PRE_ARM = 1U << 4,
        SAVE_CHECKSUM = 1U << 5,
        DISABLE_HEAP_EXPANSION = 1U << 6,
    };

private:

    /**
     * @brief Main script execution thread entry point
     * 
     * @details Dedicated thread that runs the Lua VM continuously:
     * 1. Load scripts from filesystem (ROMFS and SD card)
     * 2. Initialize Lua VM with bindings and libraries
     * 3. Execute scripts in round-robin fashion with instruction count limits
     * 4. Handle script errors and restarts
     * 5. Process VM yield points to prevent blocking
     * 
     *          Thread priority is configured via SCR_THD_PRIORITY parameter.
     *          Execution is sliced with instruction count limits (SCR_VM_I_COUNT)
     *          to ensure scripts don't monopolize CPU.
     * 
     * @note Runs continuously until scripting is disabled
     * @note Separate from main thread to isolate script execution
     * 
     * @warning Thread failure sets _thread_failed flag and disables scripting
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.cpp
     */
    void thread(void); // main script execution thread

    /**
     * @brief Save current script checksums to parameters
     * 
     * @details When SAVE_CHECKSUM debug option is set, computes checksums of all
     *          loaded and running scripts, then saves to SCR_LD_CHECKSUM and
     *          SCR_RUN_CHECKSUM parameters. Used to record known-good script state.
     * 
     *          Automatically clears SAVE_CHECKSUM flag after saving.
     * 
     * @note Called from update() when debug option is detected
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.cpp
     */
    void save_checksum();

    /**
     * @brief Checksum parameter mask for float transport compatibility
     * 
     * @details Masks checksums to 23 bits (0x007FFFFF) to match float mantissa precision.
     *          Required because parameters are transported as floats over MAVLink, and
     *          floats cannot represent the full uint32_t range without precision loss.
     * 
     * @note Limits checksum to 23-bit values but maintains uniqueness for script validation
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:174
     */
    const uint32_t checksum_param_mask = 0x007FFFFF;

    /**
     * @enum ThreadPriority
     * @brief Scripting thread priority levels (SCR_THD_PRIORITY parameter)
     * 
     * @details Maps to AP_HAL scheduler priority levels to control CPU scheduling
     *          of the scripting thread relative to other ArduPilot tasks:
     * 
     * - NORMAL (0): Default priority - below flight control tasks
     * - IO (1): I/O priority - for scripts doing heavy peripheral access
     * - STORAGE (2): Storage priority - for logging-intensive scripts
     * - UART (3): UART priority - for serial communication intensive scripts
     * - I2C (4): I2C bus priority - for I2C sensor scripts
     * - SPI (5): SPI bus priority - for SPI device scripts
     * - TIMER (6): Timer priority - for time-critical scripts
     * - MAIN (7): Main loop priority - near flight control (use with caution)
     * - BOOST (8): Highest priority - boosts above main loop (dangerous)
     * 
     * @warning Priorities MAIN and BOOST can impact flight control performance
     * @warning Higher priorities increase risk of scripts blocking critical tasks
     * 
     * @note Most scripts should use NORMAL (default) priority
     * @note Higher priorities only needed for time-critical hardware interfacing
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:176-186
     */
    enum class ThreadPriority : uint8_t {
        NORMAL = 0,
        IO = 1,
        STORAGE = 2,
        UART = 3,
        I2C = 4,
        SPI = 5,
        TIMER = 6,
        MAIN = 7,
        BOOST = 8
    };

    // Configuration parameters
    
    /**
     * @brief SCR_ENABLE parameter - Enable/disable scripting subsystem
     * 
     * @details 0 = disabled, 1 = enabled. When disabled, scripting thread is not
     *          started and no resources are allocated.
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.cpp
     */
    AP_Int8 _enable;
    
    /**
     * @brief SCR_VM_I_COUNT parameter - VM instruction count per execution slice
     * 
     * @details Number of Lua VM instructions to execute per script before yielding
     *          to other scripts. Controls time-slicing granularity. Higher values
     *          allow longer script runs but increase latency. Typical: 10000-100000.
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.cpp
     */
    AP_Int32 _script_vm_exec_count;
    
    /**
     * @brief SCR_HEAP_SIZE parameter - Lua VM heap memory allocation in bytes
     * 
     * @details Amount of heap memory allocated for Lua VM. Must accommodate all
     *          loaded scripts, global variables, and runtime data. Typical range:
     *          32KB to 256KB depending on script complexity and platform memory.
     * 
     * @warning Insufficient heap causes script load failures
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.cpp
     */
    AP_Int32 _script_heap_size;
    
    /**
     * @brief SCR_DEBUG parameter - Debug option flags bitmask
     * 
     * @details Bitmask of DebugOption values to enable debugging features.
     *          See DebugOption enum for flag descriptions.
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.cpp
     */
    AP_Int8 _debug_options;
    
    /**
     * @brief SCR_DIR_DISABLE parameter - Disabled script directories bitmask
     * 
     * @details Bitmask of SCR_DIR values to disable script loading from specific
     *          directories (ROMFS and/or SD card scripts directory).
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.cpp
     */
    AP_Int16 _dir_disable;
    
    /**
     * @brief SCR_LD_CHECKSUM parameter - Required loaded scripts checksum
     * 
     * @details Expected checksum of all scripts that should be loaded. If non-zero,
     *          loading will fail if checksum doesn't match. Used to ensure correct
     *          scripts are present. Set via SAVE_CHECKSUM debug option.
     * 
     * @note Masked to 23 bits for float transport compatibility
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.cpp
     */
    AP_Int32 _required_loaded_checksum;
    
    /**
     * @brief SCR_RUN_CHECKSUM parameter - Required running scripts checksum
     * 
     * @details Expected checksum of scripts that successfully loaded and are running.
     *          If non-zero, validation fails if checksum doesn't match. Set via
     *          SAVE_CHECKSUM debug option.
     * 
     * @note Masked to 23 bits for float transport compatibility
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.cpp
     */
    AP_Int32 _required_running_checksum;

    /**
     * @brief SCR_THD_PRIORITY parameter - Scripting thread priority
     * 
     * @details ThreadPriority enum value controlling scheduler priority of the
     *          scripting execution thread. Default is NORMAL.
     * 
     * @warning Higher priorities can impact flight control performance
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.cpp
     */
    AP_Enum<ThreadPriority> _thd_priority;

    // Helper methods for debug options
    
    /**
     * @brief Check if a debug option bit is set
     * 
     * @param[in] option DebugOption flag to check
     * 
     * @return true if option bit is set in _debug_options parameter
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:198-200
     */
    bool option_is_set(DebugOption option) const {
        return (uint8_t(_debug_options.get()) & uint8_t(option)) != 0;
    }

    /**
     * @brief Clear a debug option bit and save to EEPROM
     * 
     * @param[in] option DebugOption flag to clear
     * 
     * @note Used for trigger-once options like SAVE_CHECKSUM
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:202-204
     */
    void option_clear(DebugOption option) {
        _debug_options.set_and_save(_debug_options.get() & ~uint8_t(option));
    }

    // Internal state flags
    
    /**
     * @brief Thread allocation failure flag
     * 
     * @details Set to true if HAL thread creation failed during init(). Prevents
     *          further scripting operations.
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:206
     */
    bool _thread_failed;
    
    /**
     * @brief Memory allocation failure flag
     * 
     * @details Set to true if any memory allocation failed during init() (heap,
     *          buffers, etc.). Prevents scripting from running.
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:207
     */
    bool _init_failed;
    
    /**
     * @brief Script restart request flag
     * 
     * @details Set by restart_all() to trigger script reload. Scripting thread
     *          checks this flag and performs restart sequence when true.
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:208
     */
    bool _restart;
    
    /**
     * @brief Script execution stop flag
     * 
     * @details When true, stops script execution. Used for emergency stops or
     *          during shutdown sequence.
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:209
     */
    bool _stop;

    /**
     * @brief Singleton instance pointer
     * 
     * @details Static pointer to the single AP_Scripting instance. Set during
     *          construction, accessed via get_singleton() or AP::scripting().
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:211
     */
    static AP_Scripting *_singleton;
    
    /**
     * @brief Current Lua environment registry reference
     * 
     * @details Lua registry reference to the current script's environment table.
     *          Used for environment isolation between scripts.
     * 
     * @note Managed via get_current_env_ref() and set_current_env_ref()
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.h:212
     */
    int current_env_ref;
};

/**
 * @namespace AP
 * @brief ArduPilot singleton accessor namespace
 */
namespace AP {
    /**
     * @brief Get the AP_Scripting singleton instance
     * 
     * @details Preferred method to access the scripting subsystem. Returns pointer
     *          to the singleton instance or nullptr if scripting is not compiled in
     *          or not initialized.
     * 
     * @return Pointer to AP_Scripting singleton, or nullptr if unavailable
     * 
     * @note Prefer this over AP_Scripting::get_singleton() for consistency
     * 
     * Source: libraries/AP_Scripting/AP_Scripting.cpp
     */
    AP_Scripting * scripting(void);
};

#endif // AP_SCRIPTING_ENABLED
