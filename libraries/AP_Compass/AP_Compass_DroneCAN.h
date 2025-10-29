/**
 * @file AP_Compass_DroneCAN.h
 * @brief DroneCAN/UAVCAN magnetometer backend driver
 * 
 * @details This driver implements the AP_Compass backend interface for magnetometers
 *          connected via DroneCAN (formerly UAVCAN) CAN bus protocol. It subscribes to
 *          standard UAVCAN equipment.ahrs.MagneticFieldStrength and
 *          equipment.ahrs.MagneticFieldStrength2 messages, as well as the high-resolution
 *          dronecan.sensors.magnetometer.MagneticFieldStrengthHiRes message when enabled.
 * 
 *          The driver supports multiple compass nodes on the same CAN bus, with automatic
 *          device detection and registration. Each detected compass is assigned a unique
 *          device ID combining CAN node ID and sensor ID for multi-compass configurations.
 * 
 *          Magnetic field data received from CAN messages undergoes coordinate frame
 *          transformations to match ArduPilot's NED (North-East-Down) convention before
 *          being passed to the compass library for sensor fusion.
 * 
 * @note This driver requires AP_COMPASS_DRONECAN_ENABLED to be defined
 * @note Supports up to COMPASS_MAX_BACKEND compass instances
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 * 
 * Source: libraries/AP_Compass/AP_Compass_DroneCAN.h
 */

#pragma once

#include "AP_Compass.h"

#if AP_COMPASS_DRONECAN_ENABLED

#include "AP_Compass_Backend.h"

#include <AP_DroneCAN/AP_DroneCAN.h>

/**
 * @class AP_Compass_DroneCAN
 * @brief DroneCAN/UAVCAN compass backend for CAN bus magnetometers
 * 
 * @details This backend connects to magnetometer sensors over DroneCAN (UAVCAN) CAN bus.
 *          It implements a subscription-based architecture where static message handlers
 *          receive magnetic field messages from any CAN node, then route them to the
 *          appropriate driver instance based on node ID and sensor ID.
 * 
 *          Architecture:
 *          - Static registration: subscribe_msgs() registers message callbacks with DroneCAN
 *          - Dynamic detection: When messages arrive, devices are auto-detected and registered
 *          - Instance routing: Messages routed to correct backend instance via DetectedModules
 *          - Thread safety: Registry protected by semaphore for multi-threaded access
 * 
 *          Supported UAVCAN Messages:
 *          - uavcan.equipment.ahrs.MagneticFieldStrength (standard 16-bit resolution)
 *          - uavcan.equipment.ahrs.MagneticFieldStrength2 (extended with covariance)
 *          - dronecan.sensors.magnetometer.MagneticFieldStrengthHiRes (24-bit resolution)
 * 
 *          Coordinate Frames:
 *          - Input: UAVCAN magnetic field in sensor body frame (varies by message type)
 *          - Output: ArduPilot NED frame after transformation via compass calibration
 * 
 *          Units:
 *          - CAN messages typically provide field strength in Gauss or Tesla
 *          - Internally converted to milligauss for ArduPilot compass subsystem
 *          - Unit conversions handled in message handlers based on protocol specification
 * 
 *          Multi-Compass Support:
 *          - Each CAN compass identified by (CAN node ID, sensor ID) tuple
 *          - Device ID constructed combining these identifiers for unique identification
 *          - Supports up to COMPASS_MAX_BACKEND compass instances across all CAN buses
 * 
 * @note Driver instances are created dynamically when CAN compass messages are received
 * @note Thread-safe: Registry access protected by _sem_registry semaphore
 * @warning Ensure CAN bus termination and bit rate configuration are correct
 * 
 * Source: libraries/AP_Compass/AP_Compass_DroneCAN.h
 */
class AP_Compass_DroneCAN : public AP_Compass_Backend {
public:
    /**
     * @brief Construct a DroneCAN compass backend instance
     * 
     * @details Creates a compass backend for a specific DroneCAN device identified by
     *          device ID. The device ID encodes both CAN node ID and sensor ID for
     *          unique identification in multi-compass configurations.
     * 
     * @param[in] ap_dronecan Pointer to the DroneCAN interface manager
     * @param[in] devid       Device identifier combining CAN node ID and sensor ID
     * 
     * @note Constructor should be called only after device detection via CAN messages
     * @note Instance initialization completed in init() method
     */
    AP_Compass_DroneCAN(AP_DroneCAN* ap_dronecan, uint32_t devid);

    /**
     * @brief Read latest magnetic field data from the compass
     * 
     * @details Implements the required backend interface read() method. For DroneCAN
     *          compasses, actual data reception happens asynchronously via CAN message
     *          callbacks. This method ensures the backend remains active in the scheduler
     *          but the heavy lifting is done in handle_mag_msg().
     * 
     * @note Called periodically by the compass library at configured sample rate
     * @note Data updates occur in handle_mag_msg() callback, not in read()
     */
    void        read(void) override;

    /**
     * @brief Subscribe to DroneCAN magnetic field messages
     * 
     * @details Registers static message handlers with the DroneCAN interface for all
     *          supported magnetic field message types. Must be called during system
     *          initialization to enable reception of compass data over CAN bus.
     * 
     *          Registered message types:
     *          - uavcan.equipment.ahrs.MagneticFieldStrength
     *          - uavcan.equipment.ahrs.MagneticFieldStrength2
     *          - dronecan.sensors.magnetometer.MagneticFieldStrengthHiRes (if enabled)
     * 
     * @param[in] ap_dronecan Pointer to DroneCAN interface to subscribe through
     * 
     * @return true if subscription successful, false on failure
     * 
     * @note Should be called once during AP_Compass initialization
     * @note Subscription is global for all CAN nodes on the bus
     * 
     * Source: libraries/AP_Compass/AP_Compass_DroneCAN.cpp
     */
    static bool subscribe_msgs(AP_DroneCAN* ap_dronecan);

    /**
     * @brief Probe for detected DroneCAN compass at given index
     * 
     * @details Creates and returns a backend instance for a previously detected
     *          DroneCAN compass device. Called by the compass library to instantiate
     *          drivers for compasses found via CAN message auto-detection.
     * 
     * @param[in] index Index into detected modules array (0 to COMPASS_MAX_BACKEND-1)
     * 
     * @return Pointer to newly created backend instance, or nullptr if invalid index
     * 
     * @note Device must have been detected via CAN messages before probing
     * @note Returns existing driver if already instantiated for this device
     * 
     * Source: libraries/AP_Compass/AP_Compass_DroneCAN.cpp
     */
    static AP_Compass_Backend* probe(uint8_t index);

    /**
     * @brief Get device ID for detected compass at given index
     * 
     * @details Retrieves the device identifier for a detected DroneCAN compass without
     *          instantiating the driver. Used by compass library to query available
     *          CAN compass devices before creating backend instances.
     * 
     * @param[in] index Index into detected modules array (0 to COMPASS_MAX_BACKEND-1)
     * 
     * @return Device ID combining CAN node ID and sensor ID, or 0 if not detected
     * 
     * @note Returns 0 for indices beyond detected compass count
     */
    static uint32_t get_detected_devid(uint8_t index) { return _detected_modules[index].devid; }

    /**
     * @brief Handle UAVCAN MagneticFieldStrength message (standard resolution)
     * 
     * @details Callback for uavcan.equipment.ahrs.MagneticFieldStrength messages received
     *          over CAN bus. This is the standard 16-bit resolution magnetic field message
     *          providing 3-axis field strength in sensor body frame.
     * 
     *          Message Processing:
     *          1. Extract CAN node ID and sensor ID from transfer
     *          2. Look up or create backend instance for this device
     *          3. Convert magnetic field from message units to milligauss
     *          4. Pass field vector to backend's handle_mag_msg() for integration
     * 
     *          Coordinate Frame:
     *          - Input: Sensor body frame (varies by sensor orientation)
     *          - Transformation: Applied via compass calibration in compass library
     * 
     *          Units:
     *          - Message: Typically Gauss as per UAVCAN specification
     *          - Converted to: Milligauss for ArduPilot compass subsystem
     * 
     * @param[in] ap_dronecan Pointer to DroneCAN interface that received the message
     * @param[in] transfer    CAN transfer information (node ID, timestamp, etc.)
     * @param[in] msg         Decoded magnetic field message payload
     * 
     * @note Static callback registered via subscribe_msgs()
     * @note Handles device auto-detection and driver instantiation
     * @note Thread-safe: Uses semaphore for registry access
     * 
     * Source: libraries/AP_Compass/AP_Compass_DroneCAN.cpp
     */
    static void handle_magnetic_field(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_ahrs_MagneticFieldStrength& msg);

    /**
     * @brief Handle UAVCAN MagneticFieldStrength2 message (with covariance)
     * 
     * @details Callback for uavcan.equipment.ahrs.MagneticFieldStrength2 messages.
     *          This extended message includes magnetic field covariance matrix for
     *          improved sensor fusion, in addition to 3-axis field measurements.
     * 
     *          Enhancements over MagneticFieldStrength:
     *          - Includes covariance matrix for uncertainty quantification
     *          - Supports better multi-sensor fusion in EKF
     *          - Maintains backward compatibility with standard message
     * 
     *          Processing mirrors handle_magnetic_field() with additional covariance
     *          extraction if used by compass library sensor fusion algorithms.
     * 
     * @param[in] ap_dronecan Pointer to DroneCAN interface that received the message
     * @param[in] transfer    CAN transfer information (node ID, timestamp, etc.)
     * @param[in] msg         Decoded magnetic field message with covariance
     * 
     * @note Static callback registered via subscribe_msgs()
     * @note Covariance data may be used by advanced filtering if implemented
     * @note Thread-safe: Uses semaphore for registry access
     * 
     * Source: libraries/AP_Compass/AP_Compass_DroneCAN.cpp
     */
    static void handle_magnetic_field_2(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_ahrs_MagneticFieldStrength2 &msg);

#if AP_COMPASS_DRONECAN_HIRES_ENABLED
    /**
     * @brief Handle DroneCAN high-resolution magnetic field message
     * 
     * @details Callback for dronecan.sensors.magnetometer.MagneticFieldStrengthHiRes
     *          messages providing 24-bit resolution magnetic field measurements for
     *          improved precision over standard 16-bit UAVCAN messages.
     * 
     *          High-Resolution Benefits:
     *          - 24-bit vs 16-bit field measurements (increased precision)
     *          - Reduced quantization noise for sensitive magnetometers
     *          - Better performance in magnetically clean environments
     * 
     *          This message is a DroneCAN extension beyond standard UAVCAN protocol.
     * 
     * @param[in] ap_dronecan Pointer to DroneCAN interface that received the message
     * @param[in] transfer    CAN transfer information (node ID, timestamp, etc.)
     * @param[in] msg         Decoded high-resolution magnetic field message
     * 
     * @note Only available when AP_COMPASS_DRONECAN_HIRES_ENABLED is defined
     * @note Requires DroneCAN-compatible sensors supporting this message type
     * @note Thread-safe: Uses semaphore for registry access
     * 
     * Source: libraries/AP_Compass/AP_Compass_DroneCAN.cpp
     */
    static void handle_magnetic_field_hires(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const dronecan_sensors_magnetometer_MagneticFieldStrengthHiRes &msg);
#endif

private:
    /**
     * @brief Initialize the compass backend instance
     * 
     * @details Performs backend initialization including registering the compass instance
     *          with the AP_Compass library, setting device ID, and configuring initial
     *          parameters. Called after constructor to complete driver setup.
     * 
     * @return true if initialization successful, false on failure
     * 
     * @note Called internally during driver instantiation
     * @note Failure typically indicates compass library is at capacity
     * 
     * Source: libraries/AP_Compass/AP_Compass_DroneCAN.cpp
     */
    bool init();

    /**
     * @brief Process received magnetic field data
     * 
     * @details Callback invoked by static message handlers to deliver magnetic field
     *          measurements to this specific backend instance. Performs final coordinate
     *          transformations and publishes data to the compass library for sensor fusion.
     * 
     *          Processing steps:
     *          1. Receive field vector in sensor frame (NED convention assumed from CAN)
     *          2. Apply any necessary coordinate transformations
     *          3. Publish to compass library via backend interface
     *          4. Update timestamp and sample count
     * 
     * @param[in] mag Magnetic field vector in milligauss (sensor body frame)
     * 
     * @note Called from static message handlers after CAN message reception
     * @note Runs in DroneCAN thread context - must be thread-safe
     * @note Data ultimately integrated into EKF for attitude/heading estimation
     * 
     * Source: libraries/AP_Compass/AP_Compass_DroneCAN.cpp
     */
    void handle_mag_msg(const Vector3f &mag);

    /**
     * @brief Find or create backend instance for CAN compass device
     * 
     * @details Searches the detected modules registry for an existing backend matching
     *          the given CAN node ID and sensor ID. If found, returns existing driver.
     *          If not found and space available, creates new registry entry and instantiates
     *          a new backend driver for the device.
     * 
     *          Device Identification:
     *          - CAN node ID: Identifies specific node on CAN bus (1-127)
     *          - Sensor ID: Distinguishes multiple sensors on same node (0-255)
     *          - Combined into unique device ID for ArduPilot compass library
     * 
     * @param[in] ap_dronecan Pointer to DroneCAN interface managing this device
     * @param[in] node_id     CAN node identifier from received message
     * @param[in] sensor_id   Sensor identifier from message payload
     * 
     * @return Pointer to backend instance for this device, or nullptr if registry full
     * 
     * @note Thread-safe: Protected by _sem_registry semaphore
     * @note Called from message handlers during device auto-detection
     * @note Registry limited to COMPASS_MAX_BACKEND entries
     * 
     * Source: libraries/AP_Compass/AP_Compass_DroneCAN.cpp
     */
    static AP_Compass_DroneCAN* get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id, uint8_t sensor_id);

    /**
     * @brief Compass instance identifier within AP_Compass library
     * 
     * @details Instance number assigned by compass library during init(). Used for
     *          publishing magnetic field data to the correct compass slot.
     */
    uint8_t  _instance;

    /**
     * @brief Device identifier combining CAN node ID and sensor ID
     * 
     * @details Unique device ID constructed from CAN node ID and sensor ID for
     *          multi-compass identification. Format allows compass library to
     *          distinguish between multiple CAN compasses and assign priorities.
     */
    uint32_t _devid;

    /**
     * @struct DetectedModules
     * @brief Registry entry for detected DroneCAN compass devices
     * 
     * @details Tracks all DroneCAN compass devices detected via CAN messages,
     *          mapping (CAN interface, node ID, sensor ID) tuples to driver instances
     *          and device IDs. Enables automatic device discovery and backend instantiation.
     */
    static struct DetectedModules {
        AP_DroneCAN* ap_dronecan;  ///< DroneCAN interface managing this device
        uint8_t node_id;            ///< CAN node identifier (1-127)
        uint8_t sensor_id;          ///< Sensor identifier on this node (0-255)
        AP_Compass_DroneCAN *driver;///< Backend driver instance, or nullptr if not instantiated
        uint32_t devid;             ///< Combined device identifier for compass library
    } _detected_modules[COMPASS_MAX_BACKEND];  ///< Array of detected compass devices

    /**
     * @brief Semaphore protecting detected modules registry
     * 
     * @details Provides thread-safe access to _detected_modules array, which may be
     *          accessed concurrently from DroneCAN thread (message reception) and
     *          main thread (probe/initialization).
     * 
     * @note Critical for multi-threaded safety in device detection and registration
     * @warning Always acquire semaphore before accessing _detected_modules
     */
    static HAL_Semaphore _sem_registry;
};

#endif  // AP_COMPASS_DRONECAN_ENABLED
