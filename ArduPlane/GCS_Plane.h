/**
 * @file GCS_Plane.h
 * @brief Ground Control Station (GCS) interface for ArduPlane
 * 
 * @details This file declares the GCS_Plane class, which provides the vehicle-specific
 *          implementation of the MAVLink Ground Control Station interface for fixed-wing
 *          aircraft. GCS_Plane extends the base GCS class with Plane-specific telemetry,
 *          status reporting, and MAVLink channel management.
 * 
 *          The class manages communication between the autopilot and ground control stations
 *          via the MAVLink protocol, handling message routing, telemetry streaming, and
 *          vehicle-specific status flags.
 * 
 * @see GCS
 * @see GCS_MAVLINK_Plane
 * @see libraries/GCS_MAVLink/GCS.h
 */

#pragma once

#include <GCS_MAVLink/GCS.h>
#include "GCS_MAVLink_Plane.h"

/**
 * @class GCS_Plane
 * @brief Fixed-wing vehicle Ground Control Station interface
 * 
 * @details GCS_Plane provides the vehicle-specific implementation of the MAVLink
 *          Ground Control Station communication interface for ArduPlane. This class
 *          inherits from the base GCS class and customizes behavior for fixed-wing
 *          aircraft requirements.
 * 
 *          Key responsibilities:
 *          - Managing multiple MAVLink channels (serial, USB, telemetry radios)
 *          - Providing Plane-specific sensor status flags and health monitoring
 *          - Reporting custom flight modes specific to fixed-wing aircraft
 *          - Creating GCS_MAVLINK_Plane backend instances for each communication channel
 *          - Defining GPS health requirements for fixed-wing operations
 * 
 *          The class works in conjunction with GCS_MAVLINK_Plane instances, where
 *          GCS_Plane manages the overall GCS system and GCS_MAVLINK_Plane handles
 *          individual MAVLink channel message processing.
 * 
 * @note This class is instantiated once per vehicle and manages all GCS communication
 * @warning Modifications to virtual method overrides must maintain compatibility with
 *          ground control station software expecting standard MAVLink behavior
 */
class GCS_Plane : public GCS
{
    /**
     * @brief Friend declaration allowing Plane class access to protected GCS members
     * 
     * @details The Plane vehicle class requires access to the _chan array in the base
     *          GCS class for parameter declarations and channel management. This friend
     *          declaration grants that access without exposing internal channel management
     *          to other classes.
     */
    friend class Plane;  // for access to _chan in parameter declarations

public:

    /**
     * @brief MAVLink channel accessor method definitions
     * 
     * @details This macro expands to a pair of methods that retrieve a pointer to the
     *          GCS_MAVLINK_Plane object for a specific MAVLink channel. The macro generates
     *          both const and non-const versions of the channel accessor:
     * 
     *          Generated methods:
     *          - GCS_MAVLINK_Plane *chan(const uint8_t ofs) override;
     *          - const GCS_MAVLINK_Plane *chan(const uint8_t ofs) override const;
     * 
     *          These methods provide type-safe access to Plane-specific MAVLink channel
     *          backends, allowing the GCS system to retrieve the correct subclass instance
     *          for message handling on each active communication link.
     * 
     * @param ofs Channel offset (0 to MAVLINK_COMM_NUM_BUFFERS-1)
     * @return Pointer to GCS_MAVLINK_Plane instance for the specified channel
     * 
     * @note Each channel typically corresponds to a different physical link (USB, telemetry radio, etc.)
     * @see GCS_MAVLINK_Plane
     * @see GCS_MAVLINK_CHAN_METHOD_DEFINITIONS macro in GCS.h
     */
    GCS_MAVLINK_CHAN_METHOD_DEFINITIONS(GCS_MAVLINK_Plane);

protected:

    /**
     * @brief Update vehicle-specific sensor status flags for MAVLink telemetry
     * 
     * @details This method updates the MAVLink SYS_STATUS message sensor health flags
     *          with fixed-wing aircraft specific sensor status. It sets bits in the
     *          onboard_control_sensors_present, onboard_control_sensors_enabled, and
     *          onboard_control_sensors_health fields based on the current state of
     *          Plane-specific sensors and subsystems.
     * 
     *          Fixed-wing specific sensors include:
     *          - Airspeed sensor (critical for fixed-wing flight)
     *          - Differential pressure sensors
     *          - Fixed-wing specific navigation sensors
     * 
     *          This override extends the base GCS sensor status reporting with
     *          ArduPlane-specific hardware and capabilities.
     * 
     * @note Called periodically during telemetry updates (typically at stream rate)
     * @see MAVLink SYS_STATUS message definition
     * @see GCS::update_vehicle_sensor_status_flags()
     */
    void update_vehicle_sensor_status_flags(void) override;
    
    /**
     * @brief Get the current custom flight mode for MAVLink reporting
     * 
     * @details Returns the ArduPlane-specific custom flight mode number that corresponds
     *          to the current flight mode (MANUAL, STABILIZE, FBWA, FBWB, AUTO, RTL, etc.).
     *          This value is transmitted in the HEARTBEAT message and used by ground control
     *          stations to display the current flight mode to operators.
     * 
     *          The custom mode encoding is vehicle-specific and differs between Copter,
     *          Plane, Rover, and other vehicle types. Ground stations must interpret the
     *          custom_mode field according to the MAV_TYPE to correctly decode the mode.
     * 
     * @return uint32_t Custom mode number representing current Plane flight mode
     * 
     * @note Mode numbers are defined in Plane mode enumeration
     * @see MAVLink HEARTBEAT message
     * @see plane.h for mode definitions
     */
    uint32_t custom_mode() const override;
    
    /**
     * @brief Get the MAVLink frame type for this vehicle
     * 
     * @details Returns the MAV_TYPE enumeration value that identifies this vehicle as
     *          a fixed-wing aircraft (MAV_TYPE_FIXED_WING). This value is included in
     *          the HEARTBEAT message and allows ground control stations to identify the
     *          vehicle type and adjust their interface accordingly.
     * 
     *          Fixed-wing aircraft have different control paradigms, available flight modes,
     *          and operational characteristics compared to multirotors or ground vehicles,
     *          so correct frame type reporting is essential for proper GCS operation.
     * 
     * @return MAV_TYPE Always returns MAV_TYPE_FIXED_WING for ArduPlane
     * 
     * @note This value determines how GCS software interprets custom modes and available commands
     * @see MAVLink MAV_TYPE enumeration
     * @see MAVLink HEARTBEAT message
     */
    MAV_TYPE frame_type() const override;

    /**
     * @brief Create a new MAVLink backend instance for a communication channel
     * 
     * @details Factory method that instantiates a new GCS_MAVLINK_Plane object to handle
     *          MAVLink communication on a specific UART (serial port). This method is called
     *          when setting up each MAVLink channel during system initialization.
     * 
     *          Each MAVLink channel operates independently with its own message buffers,
     *          stream rates, and protocol state. The GCS_MAVLINK_Plane backend handles all
     *          message encoding/decoding and protocol processing for that specific link.
     * 
     *          Memory allocation uses NEW_NOTHROW to safely handle potential allocation
     *          failures in resource-constrained embedded environments.
     * 
     * @param[in] uart Reference to the UART driver for this communication channel
     * 
     * @return Pointer to newly created GCS_MAVLINK_Plane instance, or nullptr if allocation fails
     * 
     * @note Called during GCS initialization for each configured MAVLink serial port
     * @warning Caller must check for nullptr return value
     * @see GCS_MAVLINK_Plane
     * @see AP_HAL::UARTDriver
     */
    GCS_MAVLINK_Plane *new_gcs_mavlink_backend(AP_HAL::UARTDriver &uart) override {
        return NEW_NOTHROW GCS_MAVLINK_Plane(uart);
    }

    /**
     * @brief Define minimum GPS status required for healthy GPS reporting
     * 
     * @details Returns the minimum GPS fix quality that ArduPlane considers acceptable
     *          for normal operation. Fixed-wing aircraft require a 3D GPS fix (GPS_OK_FIX_3D)
     *          for safe navigation, as they depend on GPS for waypoint navigation, return-to-launch,
     *          and position hold modes.
     * 
     *          This threshold affects:
     *          - GPS health reporting in MAVLink SYS_STATUS messages
     *          - Pre-arm GPS checks
     *          - Failsafe GPS quality monitoring
     *          - Mode availability (AUTO, GUIDED, RTL require healthy GPS)
     * 
     *          Fixed-wing aircraft are less forgiving of GPS quality issues than multirotors
     *          because they cannot hover in place - they must maintain forward airspeed and
     *          require accurate position information for safe navigation.
     * 
     * @return AP_GPS::GPS_Status Always returns GPS_OK_FIX_3D for ArduPlane
     * 
     * @note GPS_OK_FIX_3D excludes NO_GPS, NO_FIX, and 2D fixes
     * @warning Lower GPS quality requirements could compromise navigation safety
     * @see AP_GPS::GPS_Status enumeration
     * @see GPS health checks in arming_checks.cpp
     */
    AP_GPS::GPS_Status min_status_for_gps_healthy() const override {
        // NO_FIX simply excludes NO_GPS
        return AP_GPS::GPS_OK_FIX_3D;
    }
};
