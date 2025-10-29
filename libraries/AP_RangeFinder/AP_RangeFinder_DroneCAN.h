/**
 * @file AP_RangeFinder_DroneCAN.h
 * @brief DroneCAN/UAVCAN rangefinder backend implementation
 * 
 * This file implements the DroneCAN (formerly UAVCAN) protocol backend for
 * rangefinder sensors. It handles UAVCAN equipment.range_sensor.Measurement
 * messages received over CAN bus to provide distance measurements to the
 * ArduPilot rangefinder subsystem.
 * 
 * The backend supports multiple CAN buses through the AP_DroneCAN MultiCAN
 * architecture, allowing rangefinder sensors to be connected to any configured
 * CAN interface. Multiple DroneCAN rangefinder instances can be registered
 * simultaneously, identified by their unique node_id and sensor address.
 * 
 * @note This backend requires AP_RANGEFINDER_DRONECAN_ENABLED to be defined
 * @see AP_RangeFinder_Backend for base class interface
 * @see AP_DroneCAN for DroneCAN protocol implementation
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_DRONECAN_ENABLED

#include "AP_RangeFinder_Backend.h"
#include <AP_DroneCAN/AP_DroneCAN.h>

class MeasurementCb;

/**
 * @class AP_RangeFinder_DroneCAN
 * @brief DroneCAN/UAVCAN protocol backend for rangefinder sensors
 * 
 * @details This backend receives distance measurements from rangefinder sensors
 *          using the UAVCAN equipment.range_sensor.Measurement message over CAN bus.
 *          It integrates with the AP_DroneCAN MultiCAN architecture to support
 *          sensors on multiple CAN buses.
 * 
 *          Key Features:
 *          - UAVCAN standard rangefinder message support
 *          - Multi-CAN bus support (CAN1, CAN2, etc.)
 *          - Multiple sensor instances per node
 *          - Automatic sensor type detection from UAVCAN messages
 *          - Node ID and sensor address identification
 * 
 *          Message Flow:
 *          1. DroneCAN node publishes equipment.range_sensor.Measurement
 *          2. AP_DroneCAN layer receives and validates CAN message
 *          3. handle_measurement() processes measurement data
 *          4. Backend updates rangefinder state and distance
 *          5. Main rangefinder driver reads via update() method
 * 
 *          Thread Safety: This backend is called from the CAN receive thread
 *          (handle_measurement) and the main scheduler thread (update).
 *          Access to shared state is protected by the underlying CAN driver.
 * 
 * @note Sensor identification uses combination of node_id and sensor address
 * @warning Distance measurements are in centimeters internally
 * @see UAVCAN equipment.range_sensor.Measurement specification
 */
class AP_RangeFinder_DroneCAN : public AP_RangeFinder_Backend {
public:
    /**
     * @brief Constructor - inherits from base RangeFinder backend
     * 
     * @details Registers this DroneCAN rangefinder instance with the top-level
     *          RangeFinder driver. Uses inherited constructor from
     *          AP_RangeFinder_Backend to initialize state and parameters.
     * 
     * @note This is called during backend detection/creation via detect()
     * @see AP_RangeFinder_Backend::AP_RangeFinder_Backend for base constructor
     */
    using AP_RangeFinder_Backend::AP_RangeFinder_Backend;

    /**
     * @brief Update rangefinder state for main scheduler loop
     * 
     * @details Called periodically by the main scheduler to update the rangefinder
     *          state and make distance measurements available to the vehicle.
     *          Checks for new data from handle_measurement() callback and updates
     *          the base class state accordingly.
     * 
     *          This method:
     *          - Checks for new measurement data flag
     *          - Updates state.distance_cm if new data available
     *          - Updates state.status (healthy, out of range, no data)
     *          - Checks for measurement timeout
     * 
     * @note This is called from the main vehicle thread at scheduler rate
     * @warning Does not perform actual CAN communication - only state update
     * @see handle_measurement() for where new measurements are received
     */
    void update() override;

    /**
     * @brief Subscribe to DroneCAN rangefinder messages on specified interface
     * 
     * @details Registers message subscription for UAVCAN equipment.range_sensor.Measurement
     *          messages on the specified DroneCAN interface. This enables the
     *          interface to receive and process rangefinder data from CAN nodes.
     * 
     *          Called during DroneCAN initialization to set up message routing
     *          from the CAN driver to the handle_measurement() callback.
     * 
     * @param[in] ap_dronecan Pointer to DroneCAN interface to subscribe on
     * 
     * @return true if subscription successful, false on failure
     * 
     * @note This must be called during initialization before messages are received
     * @note Supports MultiCAN - can be called for each CAN interface
     * @see AP_DroneCAN::subscribe_message() for subscription mechanism
     */
    static bool subscribe_msgs(AP_DroneCAN* ap_dronecan);

    /**
     * @brief Get or create DroneCAN rangefinder backend for specific sensor
     * 
     * @details Searches for an existing DroneCAN rangefinder backend matching
     *          the specified node_id and sensor address. If no match is found
     *          and create_new is true, allocates a new backend instance.
     * 
     *          This enables multiple rangefinder sensors to be supported on the
     *          same CAN bus, identified by their unique node_id and address
     *          combination. MultiCAN support allows sensors across different
     *          CAN interfaces.
     * 
     *          Sensor Identification:
     *          - node_id: UAVCAN node ID of the sensor (1-127)
     *          - address: Sensor-specific address/instance (0-255)
     *          - ap_dronecan: Which CAN interface the sensor is on
     * 
     * @param[in] ap_dronecan DroneCAN interface sensor is connected to
     * @param[in] node_id UAVCAN node ID of the rangefinder sensor
     * @param[in] address Sensor address/instance identifier
     * @param[in] create_new If true, create new backend if not found
     * 
     * @return Pointer to matching backend, or nullptr if not found/created
     * 
     * @note Called from handle_measurement() to route messages to correct backend
     * @warning Returns nullptr if no match and create_new is false
     */
    static AP_RangeFinder_DroneCAN* get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id, uint8_t address, bool create_new);

    /**
     * @brief Detect and initialize DroneCAN rangefinder backend
     * 
     * @details Factory method to create a DroneCAN rangefinder backend instance.
     *          Called by the rangefinder driver during initialization to set up
     *          DroneCAN protocol support for a rangefinder instance.
     * 
     *          This method allocates a new backend and associates it with the
     *          provided state and parameter references. Actual sensor detection
     *          occurs when UAVCAN messages are received.
     * 
     * @param[in,out] _state Reference to RangeFinder state structure
     * @param[in] _params Reference to RangeFinder parameters
     * 
     * @return Pointer to new backend instance, or nullptr on allocation failure
     * 
     * @note This is called during rangefinder initialization
     * @see AP_RangeFinder::detect_instance() for backend detection framework
     */
    static AP_RangeFinder_Backend* detect(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    /**
     * @brief Handle incoming UAVCAN rangefinder measurement message
     * 
     * @details Callback function invoked by the DroneCAN layer when a
     *          uavcan.equipment.range_sensor.Measurement message is received.
     *          This is the primary data path for DroneCAN rangefinder sensors.
     * 
     *          Message Processing:
     *          1. Extract node_id from CAN transfer metadata
     *          2. Retrieve or create backend for this sensor
     *          3. Validate measurement data (range, sensor_id, covariance)
     *          4. Convert range from meters (UAVCAN) to centimeters (ArduPilot)
     *          5. Determine sensor type from UAVCAN message
     *          6. Update backend state with new measurement
     *          7. Record timestamp for timeout detection
     * 
     *          UAVCAN Message Fields:
     *          - sensor_id: Sensor address/instance (uint8)
     *          - range: Distance measurement in meters (float)
     *          - sensor_type: UAVCAN sensor type enumeration
     *          - covariance: Measurement uncertainty
     *          - reading_type: Normal, too close, too far
     * 
     *          Thread Safety: Called from CAN receive thread context.
     *          Must be fast and non-blocking to avoid CAN RX overflow.
     * 
     * @param[in] ap_dronecan DroneCAN interface that received the message
     * @param[in] transfer CAN transfer metadata (node_id, timestamp, etc.)
     * @param[in] msg Decoded UAVCAN range_sensor.Measurement message
     * 
     * @note This is called from CAN RX interrupt/thread context
     * @warning Keep processing minimal to avoid blocking CAN reception
     * @warning Distance units: UAVCAN uses meters, converted to centimeters
     * @see uavcan.equipment.range_sensor.Measurement message specification
     */
    static void handle_measurement(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_range_sensor_Measurement &msg);

protected:
    /**
     * @brief Get MAVLink distance sensor type for telemetry reporting
     * 
     * @details Returns the MAVLink sensor type enumeration corresponding to
     *          the UAVCAN sensor type received in measurement messages. This
     *          enables proper sensor type reporting to ground control stations.
     * 
     *          Sensor types are mapped from UAVCAN equipment.range_sensor.Measurement
     *          sensor_type field to MAVLink DISTANCE_SENSOR enum values during
     *          message processing in handle_measurement().
     * 
     * @return MAVLink DISTANCE_SENSOR type enum value (ultrasonic, laser, radar, etc.)
     * 
     * @note Called by base class for MAVLink telemetry messages
     * @see MAV_DISTANCE_SENSOR enum in MAVLink common.xml
     */
    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return _sensor_type;
    }

private:
    /**
     * @brief Backend instance number in rangefinder driver
     * 
     * @details Instance index used to identify this backend within the main
     *          rangefinder driver. Used for parameter access and state management.
     */
    uint8_t _instance;

    /**
     * @brief Current rangefinder status
     * 
     * @details Tracks health and operational status of the sensor:
     *          - NotConnected: No messages received yet
     *          - NoData: Timeout - no recent measurements
     *          - OutOfRangeLow: Measurement below sensor minimum range
     *          - OutOfRangeHigh: Measurement above sensor maximum range
     *          - Good: Valid measurement within range
     * 
     * @note Updated by update() method based on measurement data and timeouts
     */
    RangeFinder::Status _status;

    /**
     * @brief Last measured distance in centimeters
     * 
     * @details Stores the most recent valid distance measurement. Converted
     *          from UAVCAN message format (meters) to ArduPilot standard (centimeters).
     * 
     * @note Units: centimeters (cm)
     * @warning UAVCAN messages provide meters - conversion performed in handle_measurement()
     */
    uint16_t _distance_cm;

    /**
     * @brief Timestamp of last valid measurement in milliseconds
     * 
     * @details System timestamp (AP_HAL::millis()) when the last measurement
     *          was received. Used by update() to detect sensor timeouts and
     *          set status to NoData if no recent measurements.
     * 
     * @note Units: milliseconds since system boot
     * @see AP_HAL::millis() for timestamp source
     */
    uint32_t _last_reading_ms;

    /**
     * @brief Pointer to DroneCAN interface this backend is bound to
     * 
     * @details Reference to the specific AP_DroneCAN interface (CAN bus) that
     *          this rangefinder sensor communicates through. Enables MultiCAN
     *          support where sensors can be on different CAN buses.
     * 
     * @note Set during backend creation in get_dronecan_backend()
     * @see AP_DroneCAN for CAN interface implementation
     */
    AP_DroneCAN* _ap_dronecan;

    /**
     * @brief UAVCAN node ID of the rangefinder sensor
     * 
     * @details The unique node identifier (1-127) of the DroneCAN node that
     *          publishes the rangefinder measurements. Combined with sensor
     *          address to uniquely identify sensors on the CAN bus.
     * 
     * @note Valid range: 1-127 per UAVCAN specification
     * @see UAVCAN node ID specification
     */
    uint8_t _node_id;

    /**
     * @brief Flag indicating new measurement data available
     * 
     * @details Set to true by handle_measurement() when a new measurement is
     *          received from CAN bus. Cleared by update() after processing
     *          the measurement data.
     * 
     *          Provides synchronization between CAN receive thread (where
     *          handle_measurement runs) and main scheduler thread (where
     *          update runs).
     * 
     * @note Acts as a simple flag for cross-thread communication
     * @warning No explicit locking - relies on atomic boolean operations
     */
    bool new_data;

    /**
     * @brief MAVLink distance sensor type for telemetry
     * 
     * @details Stores the MAVLink DISTANCE_SENSOR enum value corresponding
     *          to this sensor's type. Mapped from UAVCAN sensor_type field
     *          in measurement messages.
     * 
     *          Common Types:
     *          - MAV_DISTANCE_SENSOR_LASER: Laser rangefinder
     *          - MAV_DISTANCE_SENSOR_ULTRASOUND: Ultrasonic rangefinder
     *          - MAV_DISTANCE_SENSOR_RADAR: Radar altimeter
     *          - MAV_DISTANCE_SENSOR_INFRARED: IR rangefinder
     * 
     * @note Used for ground station telemetry reporting
     * @see MAV_DISTANCE_SENSOR enum in MAVLink specification
     */
    MAV_DISTANCE_SENSOR _sensor_type;
};
#endif  // AP_RANGEFINDER_DRONECAN_ENABLED
