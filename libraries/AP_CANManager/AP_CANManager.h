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
 * Code by Siddharth Bharat Purohit
 */

/**
 * @file AP_CANManager.h
 * @brief CAN bus manager singleton coordinating interfaces and protocol drivers
 * 
 * This file defines the AP_CANManager class, which serves as the central coordinator
 * for all CAN bus operations in ArduPilot. The manager handles:
 * - Physical CAN interface allocation and configuration
 * - Protocol driver registration and lifecycle management
 * - CAN frame logging and debugging support
 * - MAVLink CAN frame forwarding and injection
 * - SLCAN serial-to-CAN bridge integration
 * 
 * Main components:
 * - AP_CANManager: Singleton manager class
 * - CANIface_Params: Parameter group for physical interface configuration (CAN_Pn_*)
 * - CANDriver_Params: Parameter group for protocol driver configuration (CAN_Dn_*)
 * 
 * Integration points:
 * - HAL CAN interfaces (AP_HAL::CANIface) for hardware abstraction
 * - Protocol drivers (AP_CANDriver subclasses: DroneCAN, PiccoloCAN, etc.)
 * - MAVLink bridge (AP_MAVLinkCAN) for ground station CAN frame access
 * - SLCAN adapter (SLCAN::CANIface) for serial CAN bus bridges
 * 
 * Thread safety:
 * - HAL_Semaphore (_sem) protects log buffer operations
 * - Driver registration protected during initialization phase only
 * - Frame logging callbacks use semaphore protection
 */

#pragma once

#include "AP_CANManager_config.h"

#if HAL_CANMANAGER_ENABLED

#include <AP_HAL/AP_HAL.h>

#include <AP_Param/AP_Param.h>
#include "AP_SLCANIface.h"
#include "AP_CANDriver.h"
#include <GCS_MAVLink/GCS_config.h>
#if HAL_GCS_ENABLED
#include "AP_MAVLinkCAN.h"
#endif

#include "AP_CAN.h"

class CANSensor;

/**
 * @class AP_CANManager
 * @brief Singleton manager coordinating CAN interfaces and protocol drivers across ArduPilot
 * 
 * @details AP_CANManager serves as the central coordination point for all CAN bus functionality
 * in ArduPilot. It manages the complete lifecycle of CAN communications:
 * 
 * Physical Interface Management:
 * - Manages up to HAL_NUM_CAN_IFACES physical CAN interfaces (typically 2-3 depending on board)
 * - Configures interface bitrates from CAN_Pn_BITRATE parameters
 * - Allocates HAL CAN interfaces during initialization
 * - Supports both CAN 2.0 (standard) and CAN FD (flexible data-rate) configurations
 * 
 * Protocol Driver Coordination:
 * - Instantiates and coordinates up to HAL_MAX_CAN_PROTOCOL_DRIVERS protocol drivers
 * - Registers protocol drivers (DroneCAN, PiccoloCAN) based on CAN_Dn_PROTOCOL parameters
 * - Supports dual protocol operation per interface (29-bit + 11-bit addressing)
 * - Routes frames to appropriate drivers based on addressing and protocol type
 * 
 * Debug and Diagnostics:
 * - Provides circular text logging buffer (LOG_BUFFER_SIZE=1024) for CAN subsystem debug messages
 * - Configurable log levels: NONE, ERROR, WARNING, INFO, DEBUG via CAN_LOGLEVEL parameter
 * - Thread-safe logging via HAL_Semaphore protection
 * - Log retrieval for ground station display via MAVLink
 * 
 * Advanced Features:
 * - SLCAN integration: Serial-to-CAN bridge when AP_CAN_SLCAN_ENABLED
 * - MAVLink forwarding: CAN frame injection/monitoring via GCS when HAL_GCS_ENABLED
 * - Frame logging: Complete CAN frame capture to dataflash when AP_CAN_LOGGING_ENABLED
 * - Filter modification: Dynamic CAN acceptance filter configuration
 * 
 * Initialization sequence:
 * 1. Constructor allocates singleton and initializes parameters
 * 2. init() enumerates HAL interfaces and instantiates protocol drivers
 * 3. Protocol drivers call register_driver() during their construction
 * 4. Interfaces configured and started based on parameters
 * 5. Frame routing and logging callbacks registered
 * 
 * @note Singleton accessed via AP::can() or get_singleton()
 * @warning Must call init() during vehicle startup before using CAN functionality
 * @warning Driver pointers passed to register_driver() must remain valid for manager lifetime
 * 
 * @see AP_CANDriver for protocol driver interface
 * @see AP_CAN for protocol type definitions
 * @see AP_CANSensor for auxiliary sensor driver helpers
 * @see AP_SLCANIface for SLCAN bridge implementation
 * @see AP_MAVLinkCAN for MAVLink CAN frame forwarding
 */
class AP_CANManager
{
public:
    AP_CANManager();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_CANManager);

    /**
     * @brief Return singleton instance of AP_CANManager
     * 
     * @return Pointer to singleton AP_CANManager instance
     * 
     * @warning Triggers AP_HAL::panic if called before singleton allocation
     * @note Prefer using AP::can() accessor for cleaner syntax
     */
    static AP_CANManager* get_singleton()
    {
        if (_singleton == nullptr) {
            AP_HAL::panic("CANManager used before allocation.");
        }
        return _singleton;
    }

    /**
     * @brief Logging verbosity levels for CAN subsystem debug messages
     * 
     * Controls which messages are captured in the internal circular log buffer
     * and made available for retrieval. Configured via CAN_LOGLEVEL parameter.
     */
    enum LogLevel : uint8_t {
        LOG_NONE,      ///< No logging - all messages suppressed
        LOG_ERROR,     ///< Critical errors only - hardware failures, initialization errors
        LOG_WARNING,   ///< Errors and warnings - recoverable issues, degraded operation
        LOG_INFO,      ///< Informational messages - normal operation events, status changes
        LOG_DEBUG,     ///< Verbose debug output - frame counts, timing, detailed state info
    };

    /**
     * @brief Initialize CAN manager, allocate interfaces, instantiate protocol drivers
     * 
     * @details Performs complete CAN subsystem initialization:
     * 1. Enumerates up to HAL_NUM_CAN_IFACES physical interfaces
     * 2. Configures interface bitrates from CAN_Pn_BITRATE and CAN_Pn_FDBITRATE parameters
     * 3. Allocates HAL CAN interface drivers for each enabled interface
     * 4. Instantiates protocol drivers based on CAN_Dn_PROTOCOL parameter values
     * 5. Registers interfaces with protocol drivers via add_interface() calls
     * 6. Enables CAN frame logging callbacks when AP_CAN_LOGGING_ENABLED
     * 7. Initializes SLCAN bridge if AP_CAN_SLCAN_ENABLED
     * 8. Sets up MAVLink CAN forwarding if HAL_GCS_ENABLED
     * 
     * Protocol driver instantiation creates drivers for:
     * - DroneCAN/UAVCAN when CAN_Dn_PROTOCOL = 1
     * - PiccoloCAN when CAN_Dn_PROTOCOL = 4
     * - Additional protocols as configured
     * 
     * @note Called once during vehicle startup after AP_Param::load_all()
     * @note Must complete before any CAN communication begins
     * @warning Requires valid AP_Param configuration to be loaded
     * @warning Requires HAL CAN interfaces to be available (board-dependent)
     */
    __INITFUNC__ void init(void);

    /**
     * @brief Register a protocol driver for a specific driver slot
     * 
     * @param[in] dtype Protocol type from AP_CAN::Protocol enum (DroneCAN, PiccoloCAN, etc.)
     * @param[in] driver Pointer to instantiated AP_CANDriver subclass (must remain valid)
     * 
     * @return true if registration successful, false if driver slot full or invalid
     * 
     * @details Registers a protocol driver instance with the manager. Each driver is assigned
     * to a slot (0 to HAL_MAX_CAN_PROTOCOL_DRIVERS-1) and associated with a protocol type.
     * The manager will route CAN frames to the driver and coordinate interface allocation.
     * 
     * Driver registration typically occurs during protocol driver constructor execution,
     * which happens within init() after parameter loading. The driver pointer must remain
     * valid for the lifetime of the manager as no ownership transfer occurs.
     * 
     * @note Typically called by protocol driver constructors during init()
     * @warning Driver pointer must remain valid for lifetime of manager (no ownership transfer)
     * @warning Returns false if all driver slots are full (HAL_MAX_CAN_PROTOCOL_DRIVERS exceeded)
     */
    bool register_driver(AP_CAN::Protocol dtype, AP_CANDriver *driver);

    /**
     * @brief Register auxiliary sensor driver using 11-bit CAN addressing
     * 
     * @param[in] dtype Protocol type from AP_CAN::Protocol enum
     * @param[in] sensor Pointer to CANSensor subclass implementing 11-bit protocol
     * @param[out] driver_index Assigned driver index for this sensor
     * 
     * @return true if registration successful, false if no slots available
     * 
     * @details Enables dual protocol operation on a single CAN interface by supporting
     * both 29-bit extended addressing (primary protocol) and 11-bit standard addressing
     * (auxiliary sensors). This allows CAN_Dn_PROTOCOL2 parameter usage for secondary
     * protocols alongside the primary CAN_Dn_PROTOCOL.
     * 
     * Common use case: DroneCAN (29-bit) as primary protocol with legacy 11-bit sensors
     * (GPS, compass, airspeed) operating simultaneously on the same bus.
     * 
     * @note Supports CAN_Dn_PROTOCOL2 parameter for secondary 11-bit protocols
     * @warning Sensor pointer must remain valid for manager lifetime
     */
    bool register_11bit_driver(AP_CAN::Protocol dtype, CANSensor *sensor, uint8_t &driver_index);

    /**
     * @brief Return maximum number of protocol driver slots
     * 
     * @return HAL_MAX_CAN_PROTOCOL_DRIVERS compile-time constant (typically 6)
     * 
     * @note Actual active drivers may be fewer depending on CAN_Dn_PROTOCOL configuration
     * @note Empty slots return nullptr from get_driver()
     */
    uint8_t get_num_drivers(void) const
    {
        return HAL_MAX_CAN_PROTOCOL_DRIVERS;
    }

    /**
     * @brief Retrieve protocol driver by index
     * 
     * @param[in] i Driver index (0 to HAL_MAX_CAN_PROTOCOL_DRIVERS-1)
     * 
     * @return Pointer to AP_CANDriver if slot allocated, nullptr if empty or index invalid
     * 
     * @note Use in conjunction with get_num_drivers() to iterate all driver slots
     * @note Check return value for nullptr before dereferencing
     */
    AP_CANDriver* get_driver(uint8_t i) const
    {
        if (i < ARRAY_SIZE(_drivers)) {
            return _drivers[i];
        }
        return nullptr;
    }

    /**
     * @brief Return current CAN subsystem logging verbosity
     * 
     * @return LogLevel enum value from CAN_LOGLEVEL parameter
     * 
     * @note Used by log_text() to filter messages before buffering
     * @note Changes to CAN_LOGLEVEL take effect immediately
     */
    LogLevel get_log_level(void) const
    {
        return LogLevel(_loglevel.get());
    }
    
    /**
     * @brief Log formatted debug message to internal circular buffer
     * 
     * @param[in] loglevel Message severity level (LOG_ERROR, LOG_WARNING, LOG_INFO, LOG_DEBUG)
     * @param[in] tag Component identifier string (e.g., "DroneCAN", "PiccoloCAN", "CANMgr")
     * @param[in] fmt printf-style format string
     * @param[in] ... Variable arguments for format string
     * 
     * @details Logs a formatted message to the internal circular buffer (LOG_BUFFER_SIZE=1024 bytes)
     * if the message level is enabled by the CAN_LOGLEVEL parameter. Messages are stored with
     * timestamps and can be retrieved via log_retrieve() for ground station display.
     * 
     * Thread safety is provided by HAL_Semaphore (_sem) protection. When the buffer fills,
     * oldest messages are overwritten (circular behavior).
     * 
     * Typical usage by protocol drivers:
     * - LOG_ERROR: Hardware communication failures, initialization errors
     * - LOG_WARNING: Frame drops, buffer overruns, degraded operation
     * - LOG_INFO: Protocol state changes, device discovery, configuration updates
     * - LOG_DEBUG: Frame counts, timing statistics, detailed protocol state
     * 
     * @note Thread-safe via HAL_Semaphore protection
     * @note Messages filtered by CAN_LOGLEVEL parameter before buffering
     * @warning High-frequency logging (especially LOG_DEBUG) can impact CAN bus performance
     * @warning Consider CPU overhead when enabling verbose logging on high-rate buses
     */
    void log_text(AP_CANManager::LogLevel loglevel, const char *tag, const char *fmt, ...) FMT_PRINTF(4,5);

    /**
     * @brief Retrieve accumulated log messages for display
     * 
     * @param[out] str ExpandingString to receive log buffer contents
     * 
     * @details Copies the current log buffer contents to the provided ExpandingString for
     * transmission to ground control station or other consumers. Used by GCS log retrieval
     * commands to provide CAN subsystem debug information without dataflash access.
     * 
     * @note Thread-safe via HAL_Semaphore protection
     * @note Does not clear buffer - messages remain until overwritten
     */
    void log_retrieve(ExpandingString &str) const;

    /**
     * @brief Return protocol type for driver slot
     * 
     * @param[in] i Driver index (0 to HAL_MAX_CAN_PROTOCOL_DRIVERS-1)
     * 
     * @return AP_CAN::Protocol enum value, Protocol::None if slot empty or index invalid
     * 
     * @details Returns the protocol type (DroneCAN, PiccoloCAN, etc.) assigned to the
     * specified driver slot. Uses cached value from _driver_type_cache array for fast
     * lookup without accessing driver object.
     * 
     * Protocol types include:
     * - Protocol::DroneCAN (1): DroneCAN/UAVCAN protocol
     * - Protocol::PiccoloCAN (4): Piccolo CAN protocol
     * - Protocol::None (0): Slot not allocated
     * 
     * @note Uses cached protocol type for fast lookup
     * @note Returns Protocol::None for empty slots
     */
    AP_CAN::Protocol get_driver_type(uint8_t i) const
    {
        if (i < ARRAY_SIZE(_driver_type_cache)) {
            return _driver_type_cache[i];
        }
        return AP_CAN::Protocol::None;
    }

    static const struct AP_Param::GroupInfo var_info[];

#if HAL_GCS_ENABLED
    /**
     * @brief Process MAVLink CAN_FORWARD command for frame injection
     * 
     * @param[in] chan MAVLink channel receiving command
     * @param[in] packet COMMAND_INT packet with CAN bus and frame configuration
     * @param[in] msg Complete MAVLink message
     * 
     * @return true if command handled successfully, false if invalid or failed
     * 
     * @details Handles MAV_CMD_CAN_FORWARD command which allows ground control stations
     * to inject CAN frames directly onto the bus. The command specifies target bus,
     * frame ID, DLC, and data bytes. Used for CAN bus diagnostics, device configuration,
     * and protocol development.
     * 
     * @note Conditionally compiled when HAL_GCS_ENABLED
     * @warning Allows arbitrary CAN frame injection - use with caution on flight-critical buses
     */
    inline bool handle_can_forward(mavlink_channel_t chan, const mavlink_command_int_t &packet, const mavlink_message_t &msg)
    {
        return mavlink_can.handle_can_forward(chan, packet, msg);
    }

    /**
     * @brief Inject MAVLink CAN_FRAME or CANFD_FRAME into CAN bus
     * 
     * @param[in] msg MAVLink message containing CAN_FRAME or CANFD_FRAME data
     * 
     * @details Processes CAN_FRAME (standard/extended) or CANFD_FRAME (CAN FD) messages
     * from ground control station and queues them for transmission on the specified
     * CAN bus interface. Supports both CAN 2.0 (up to 8 bytes) and CAN FD (up to 64 bytes).
     * 
     * Frame is validated and queued via can_frame_callback. If the queue is full or
     * interface is unavailable, frame may be dropped.
     * 
     * @note Conditionally compiled when HAL_GCS_ENABLED
     * @warning No authentication or authorization - assumes trusted MAVLink connection
     */
    inline void handle_can_frame(const mavlink_message_t &msg)
    {
        mavlink_can.handle_can_frame(msg);
    }

    /**
     * @brief Update CAN filter configuration via MAVLink
     * 
     * @param[in] msg CAN_FILTER_MODIFY MAVLink message with filter parameters
     * 
     * @details Processes CAN_FILTER_MODIFY message to dynamically adjust hardware acceptance
     * filters on CAN interfaces. Allows ground station to configure which CAN IDs are
     * received by the autopilot, reducing CPU load and buffer usage on high-traffic buses.
     * 
     * Filter configuration includes:
     * - Filter bank index
     * - Filter ID (11-bit or 29-bit)
     * - Filter mask
     * - Filter enable/disable
     * 
     * @note Conditionally compiled when HAL_GCS_ENABLED
     * @warning Incorrect filter configuration can block critical frames
     */
    inline void handle_can_filter_modify(const mavlink_message_t &msg)
    {
        mavlink_can.handle_can_filter_modify(msg);
    }
#endif

private:

    /**
     * @class CANIface_Params
     * @brief Parameter group for physical CAN interface configuration
     * 
     * @details Stores per-interface configuration parameters exposed as CAN_Pn_* where
     * n = 1 to HAL_NUM_CAN_IFACES. Instantiated as _interfaces[HAL_NUM_CAN_IFACES] array.
     * 
     * Parameters controlled:
     * - CAN_Pn_DRIVER: Protocol driver assignment (which driver uses this interface)
     * - CAN_Pn_BITRATE: CAN 2.0 bitrate in bits/second (125000, 250000, 500000, 1000000)
     * - CAN_Pn_FDBITRATE: CAN FD data phase bitrate for faster payload transfer
     * - CAN_Pn_OPTIONS: Bitmask options (frame logging, filtering, etc.)
     * 
     * Typical configurations:
     * - CAN_P1_DRIVER=1, CAN_P1_BITRATE=1000000: Interface 1 assigned to driver 1 at 1Mbps
     * - CAN_P2_DRIVER=0: Interface 2 disabled (no driver assigned)
     * 
     * @note Array index is 0-based, parameter numbering is 1-based (CAN_P1_* uses index 0)
     */
    class CANIface_Params
    {
        friend class AP_CANManager;

    public:
        CANIface_Params()
        {
            AP_Param::setup_object_defaults(this, var_info);
        }

        static const struct AP_Param::GroupInfo var_info[];

        /**
         * @brief Bitmask options for CAN interface behavior
         * 
         * Options configured via CAN_Pn_OPTIONS parameter. Multiple options can be
         * combined using bitwise OR.
         */
        enum class Options : uint32_t {
            LOG_ALL_FRAMES = (1U<<0),  ///< Enable logging of all CAN frames to dataflash (high CPU/storage cost)
        };

        /**
         * @brief Check if specific option bit is enabled
         * 
         * @param[in] option Options enum value to test
         * 
         * @return true if option bit set in CAN_Pn_OPTIONS parameter, false otherwise
         * 
         * @note Used by check_logging_enable() to conditionally register frame logging callbacks
         */
        bool option_is_set(Options option) const {
            return (_options & uint32_t(option)) != 0;
        }

    private:
        AP_Int8 _driver_number;
        AP_Int32 _bitrate;
        AP_Int32 _fdbitrate;
        AP_Int32 _options;

#if AP_CAN_LOGGING_ENABLED && HAL_LOGGING_ENABLED
        uint8_t logging_id;
#endif
    };

    /**
     * @class CANDriver_Params
     * @brief Parameter group for protocol driver configuration
     * 
     * @details Stores per-driver configuration parameters exposed as CAN_Dn_* where
     * n = 1 to HAL_MAX_CAN_PROTOCOL_DRIVERS. Instantiated as _drv_param[HAL_MAX_CAN_PROTOCOL_DRIVERS] array.
     * 
     * Parameters controlled:
     * - CAN_Dn_PROTOCOL: Primary protocol type (0=None, 1=DroneCAN, 4=PiccoloCAN, etc.)
     * - CAN_Dn_PROTOCOL2: Secondary 11-bit protocol type for auxiliary sensors
     * 
     * Internal driver pointers:
     * - _uavcan: Pointer to DroneCAN/UAVCAN driver instance if allocated
     * - _piccolocan: Pointer to PiccoloCAN driver instance if allocated
     * 
     * Typical configurations:
     * - CAN_D1_PROTOCOL=1: Driver slot 1 uses DroneCAN protocol
     * - CAN_D2_PROTOCOL=4: Driver slot 2 uses PiccoloCAN protocol
     * - CAN_D1_PROTOCOL2=10: Driver 1 also handles 11-bit sensor protocol
     * 
     * Driver instantiation occurs during init() based on CAN_Dn_PROTOCOL values.
     * The protocol drivers call register_driver() to associate themselves with the slot.
     * 
     * @note Array index is 0-based, parameter numbering is 1-based (CAN_D1_* uses index 0)
     */
    class CANDriver_Params
    {
        friend class AP_CANManager;

    public:
        CANDriver_Params()
        {
            AP_Param::setup_object_defaults(this, var_info);
        }
        static const struct AP_Param::GroupInfo var_info[];

    private:
        AP_Int8 _driver_type;
        AP_Int8 _driver_type_11bit;
        AP_CANDriver* _uavcan;
        AP_CANDriver* _piccolocan;
    };

    CANIface_Params _interfaces[HAL_NUM_CAN_IFACES];
    AP_CANDriver* _drivers[HAL_MAX_CAN_PROTOCOL_DRIVERS];
    CANDriver_Params _drv_param[HAL_MAX_CAN_PROTOCOL_DRIVERS];
    AP_CAN::Protocol _driver_type_cache[HAL_MAX_CAN_PROTOCOL_DRIVERS];

    AP_Int8 _loglevel;
    uint8_t _num_drivers;
#if AP_CAN_SLCAN_ENABLED
    SLCAN::CANIface _slcan_interface;
#endif

    static AP_CANManager *_singleton;

    char* _log_buf;
    uint32_t _log_pos;

    HAL_Semaphore _sem;

#if HAL_GCS_ENABLED
    // Class for handling MAVLink CAN frames
    AP_MAVLinkCAN mavlink_can;
#endif // HAL_GCS_ENABLED

#if AP_CAN_LOGGING_ENABLED && HAL_LOGGING_ENABLED
    /**
     * @brief Callback for logging CAN frames to dataflash (AP_Logger)
     * 
     * @param[in] bus CAN interface index (0-based, 0 to HAL_NUM_CAN_IFACES-1)
     * @param[in] frame CAN frame structure containing ID, DLC, and data bytes
     * @param[in] flags Transmit/receive direction and error status indicators
     * 
     * @details Registered via FUNCTOR_BIND_MEMBER when LOG_ALL_FRAMES option is enabled
     * for an interface. Called by HAL CAN driver for every frame transmitted or received.
     * 
     * Logs frames to dataflash using AP_Logger::Write() with message type determined by
     * logging_id assigned during initialization. Frame data includes:
     * - Timestamp (microseconds)
     * - Bus number
     * - Frame ID (11-bit or 29-bit)
     * - DLC (data length code)
     * - Data bytes (up to 8 for CAN 2.0, up to 64 for CAN FD)
     * - Flags (TX/RX, error status)
     * 
     * @note Registered per-interface via HAL CAN set_frame_logging_callback()
     * @warning High CPU and storage overhead - can log thousands of frames per second
     * @warning May impact real-time performance on high-traffic buses
     */
    void can_logging_callback(uint8_t bus, const AP_HAL::CANFrame &frame, AP_HAL::CANIface::CanIOFlags flags);
    
    /**
     * @brief Conditionally register/unregister logging callbacks based on OPTIONS
     * 
     * @details Called when CAN_Pn_OPTIONS parameter changes to enable or disable frame
     * logging for each interface. Checks LOG_ALL_FRAMES option bit and registers or
     * unregisters can_logging_callback() accordingly.
     * 
     * Registration uses HAL_CANIface::set_frame_logging_callback() with FUNCTOR_BIND_MEMBER
     * to create callback binding. Unregistration passes nullptr to disable logging.
     * 
     * @note Called during init() and when parameters are modified
     * @note Each interface can independently enable/disable frame logging
     */
    void check_logging_enable(void);
#endif
};

namespace AP
{
    /**
     * @brief AP namespace accessor function for CAN manager singleton
     * 
     * @return Reference to AP_CANManager singleton instance
     * 
     * @details Provides convenient access to the CAN manager singleton using AP::can() syntax.
     * Preferred over calling AP_CANManager::get_singleton() directly for cleaner code.
     * 
     * Typical usage:
     * ```cpp
     * AP::can().log_text(AP_CANManager::LOG_INFO, "MyDriver", "Device initialized");
     * AP_CANDriver* driver = AP::can().get_driver(0);
     * ```
     * 
     * @note Implemented in AP_CANManager.cpp
     * @see AP_CANManager::get_singleton()
     */
    AP_CANManager& can();
}



#endif  // HAL_CANMANAGER_ENABLED
