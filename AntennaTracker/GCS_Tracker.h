/**
 * @file GCS_Tracker.h
 * @brief GCS interface class declarations for antenna tracker
 * 
 * @details This file declares the GCS_Tracker singleton class which manages
 *          Ground Control Station communication for the antenna tracker vehicle.
 *          It provides the MAVLink backend factory and vehicle-specific GCS
 *          interface implementations including frame type identification and
 *          custom mode reporting.
 * 
 * Source: AntennaTracker/GCS_Tracker.h
 */

#pragma once

#include <GCS_MAVLink/GCS.h>
#include "GCS_MAVLink_Tracker.h"

/**
 * @class GCS_Tracker
 * @brief GCS interface manager for antenna tracker vehicle
 * 
 * @details Extends the base GCS class to provide antenna tracker-specific
 *          Ground Control Station communication functionality. This class is
 *          responsible for:
 *          - Creating MAVLink backend instances for each communication channel
 *          - Providing vehicle type identification (MAV_TYPE_ANTENNA_TRACKER)
 *          - Reporting current flight mode via custom_mode accessor
 *          - Handling legacy datastream rate configuration requests
 *          - Managing sensor status flag updates for telemetry
 * 
 *          The class integrates with the Tracker singleton to access vehicle
 *          state and mode information for MAVLink reporting.
 * 
 * @note This is a singleton class accessed through the gcs() function
 * @see GCS base class in libraries/GCS_MAVLink/GCS.h
 * @see GCS_MAVLINK_Tracker backend class
 */
class GCS_Tracker : public GCS
{
    friend class Tracker; // for access to _chan in parameter declarations
    friend class GCS_MAVLINK_Tracker;

public:

    /**
     * @brief Channel accessor method definitions for tracker MAVLink backends
     * 
     * @details The following macro expands to a pair of methods to retrieve a
     *          pointer to an object of the correct subclass for the link at
     *          offset ofs. These are of the form:
     *          - GCS_MAVLINK_Tracker *chan(const uint8_t ofs) override;
     *          - const GCS_MAVLINK_Tracker *chan(const uint8_t ofs) override const;
     * 
     *          These methods provide type-safe access to the tracker-specific
     *          MAVLink backend instances for each communication channel.
     */
    GCS_MAVLINK_CHAN_METHOD_DEFINITIONS(GCS_MAVLINK_Tracker);

    /**
     * @brief Update vehicle sensor status flags for MAVLink reporting
     * 
     * @details Updates the internal sensor status flags that are reported
     *          in MAVLink SYS_STATUS messages. This method is called periodically
     *          to reflect the current health and availability of antenna tracker
     *          sensors and subsystems.
     * 
     * @note Called by the GCS update loop to maintain current telemetry
     * @see GCS::update_vehicle_sensor_status_flags() base implementation
     */
    void update_vehicle_sensor_status_flags() override;

    /**
     * @brief Get current custom flight mode number
     * 
     * @details Returns the current antenna tracker mode as a numeric value
     *          corresponding to the Mode::Number enum. This value is reported
     *          in MAVLink HEARTBEAT messages to inform ground control stations
     *          of the tracker's current operational mode.
     * 
     * @return uint32_t Current mode number from Mode::Number enum
     * 
     * @note Mode numbers are tracker-specific and defined in mode.h
     * @see Mode::Number enum in AntennaTracker/mode.h
     */
    uint32_t custom_mode() const override;

    /**
     * @brief Get MAVLink vehicle frame type
     * 
     * @details Returns the MAVLink vehicle type identifier for antenna tracker.
     *          This constant value identifies the vehicle as an antenna tracker
     *          to ground control stations and is included in HEARTBEAT messages.
     * 
     * @return MAV_TYPE Returns MAV_TYPE_ANTENNA_TRACKER
     * 
     * @note This is a constant value specific to antenna tracker vehicles
     * @see MAV_TYPE enum in MAVLink common message set
     */
    MAV_TYPE frame_type() const override;

protected:

    /**
     * @brief Factory method to create tracker-specific MAVLink backend
     * 
     * @details Creates a new GCS_MAVLINK_Tracker instance for handling MAVLink
     *          communication on a specific UART channel. This factory method is
     *          called by the base GCS class during initialization to instantiate
     *          the appropriate backend type for each configured communication link.
     * 
     * @param[in] uart Reference to UART driver for this communication channel
     * 
     * @return GCS_MAVLINK_Tracker* Pointer to newly created backend instance,
     *         or nullptr if allocation fails
     * 
     * @note Uses NEW_NOTHROW for safe allocation without exceptions
     * @see GCS_MAVLINK_Tracker backend implementation
     */
    GCS_MAVLINK_Tracker *new_gcs_mavlink_backend(AP_HAL::UARTDriver &uart) override {
        return NEW_NOTHROW GCS_MAVLINK_Tracker(uart);
    }

private:

    /**
     * @brief Request position datastream from remote system
     * 
     * @details Sends a MAVLink REQUEST_DATA_STREAM message to request position
     *          data from a remote MAVLink system. This is part of the legacy
     *          datastream configuration system (pre-MAVLink 2.0 message
     *          interval configuration).
     * 
     * @param[in] sysid   System ID of the target MAVLink system
     * @param[in] compid  Component ID of the target MAVLink component
     * 
     * @note Legacy method - newer systems should use MESSAGE_INTERVAL
     * @see MAVLink REQUEST_DATA_STREAM message definition
     */
    void request_datastream_position(uint8_t sysid, uint8_t compid);

    /**
     * @brief Request air pressure datastream from remote system
     * 
     * @details Sends a MAVLink REQUEST_DATA_STREAM message to request barometric
     *          pressure and altitude data from a remote MAVLink system. This is
     *          part of the legacy datastream configuration system (pre-MAVLink 2.0
     *          message interval configuration).
     * 
     * @param[in] sysid   System ID of the target MAVLink system
     * @param[in] compid  Component ID of the target MAVLink component
     * 
     * @note Legacy method - newer systems should use MESSAGE_INTERVAL
     * @see MAVLink REQUEST_DATA_STREAM message definition
     */
    void request_datastream_airpressure(uint8_t sysid, uint8_t compid);

};
