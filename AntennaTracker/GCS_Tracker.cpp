/**
 * @file GCS_Tracker.cpp
 * @brief GCS interface implementation for AntennaTracker vehicle
 * 
 * @details This file implements the GCS_Tracker class which provides the MAVLink
 *          Ground Control Station interface specific to the AntennaTracker vehicle.
 *          It handles legacy REQUEST_DATA_STREAM messages by translating them to
 *          modern stream rate configurations, and reports vehicle sensor status flags
 *          appropriate for antenna tracking systems.
 *          
 *          The AntennaTracker GCS interface is simplified compared to other vehicles,
 *          focusing on position tracking and attitude control capabilities rather than
 *          full flight sensor suites.
 * 
 * @note Modern ground control stations should use MESSAGE_INTERVAL or STREAM_RATE
 *       parameters instead of REQUEST_DATA_STREAM messages for better control.
 * 
 * Source: AntennaTracker/GCS_Tracker.cpp
 */

#include "GCS_Tracker.h"
#include "Tracker.h"

/**
 * @brief Request position data stream from a remote MAVLink system
 * 
 * @details Sends REQUEST_DATA_STREAM message to a remote MAVLink system requesting
 *          position information (MAV_DATA_STREAM_POSITION). This is typically used
 *          to request the tracked vehicle to stream its position data to the tracker.
 *          
 *          The request is sent on all available GCS channels that have sufficient
 *          payload space, using the configured mavlink_update_rate parameter.
 *          
 *          Legacy Protocol: This implements the older REQUEST_DATA_STREAM protocol.
 *          Modern systems should use MESSAGE_INTERVAL commands for individual message
 *          control or STREAM_RATE parameters for rate group configuration.
 * 
 * @param[in] _sysid  System ID of the target MAVLink system to request data from
 * @param[in] compid  Component ID of the target MAVLink component
 * 
 * @note This method loops through all GCS channels to broadcast the request
 * @note Rate is determined by tracker.g.mavlink_update_rate parameter
 * @note Legacy compatibility - modern GCS use MESSAGE_INTERVAL or STREAM_RATE parameters
 * 
 * @see request_datastream_airpressure()
 * 
 * Source: AntennaTracker/GCS_Tracker.cpp:4-18
 */
void GCS_Tracker::request_datastream_position(const uint8_t _sysid, const uint8_t compid)
{
    for (uint8_t i=0; i < num_gcs(); i++) {
            // request position
            if (HAVE_PAYLOAD_SPACE((mavlink_channel_t)i, DATA_STREAM)) {
                mavlink_msg_request_data_stream_send(
                    (mavlink_channel_t)i,
                    _sysid,
                    compid,
                    MAV_DATA_STREAM_POSITION,
                    tracker.g.mavlink_update_rate,
                    1); // start streaming
            }
    }
}

/**
 * @brief Request air pressure and raw sensor data stream from a remote MAVLink system
 * 
 * @details Sends REQUEST_DATA_STREAM message to a remote MAVLink system requesting
 *          raw sensor data including air pressure (MAV_DATA_STREAM_RAW_SENSORS).
 *          This is used to receive barometric pressure and other raw sensor readings
 *          from the tracked vehicle.
 *          
 *          The request is sent on all available GCS channels that have sufficient
 *          payload space, using the configured mavlink_update_rate parameter.
 *          
 *          Legacy Protocol: This implements the older REQUEST_DATA_STREAM protocol.
 *          Modern systems should use MESSAGE_INTERVAL commands for individual message
 *          control or STREAM_RATE parameters for rate group configuration.
 * 
 * @param[in] _sysid  System ID of the target MAVLink system to request data from
 * @param[in] compid  Component ID of the target MAVLink component
 * 
 * @note This method loops through all GCS channels to broadcast the request
 * @note Rate is determined by tracker.g.mavlink_update_rate parameter
 * @note Legacy compatibility - modern GCS use MESSAGE_INTERVAL or STREAM_RATE parameters
 * @note MAV_DATA_STREAM_RAW_SENSORS includes barometer, IMU, and other raw sensor data
 * 
 * @see request_datastream_position()
 * 
 * Source: AntennaTracker/GCS_Tracker.cpp:20-34
 */
void GCS_Tracker::request_datastream_airpressure(const uint8_t _sysid, const uint8_t compid)
{
    for (uint8_t i=0; i < num_gcs(); i++) {
            // request air pressure
            if (HAVE_PAYLOAD_SPACE((mavlink_channel_t)i, DATA_STREAM)) {
                mavlink_msg_request_data_stream_send(
                    (mavlink_channel_t)i,
                    _sysid,
                    compid,
                    MAV_DATA_STREAM_RAW_SENSORS,
                    tracker.g.mavlink_update_rate,
                    1); // start streaming
            }
    }
}

/**
 * @brief Update vehicle sensor and subsystem status flags for MAVLink SYS_STATUS reporting
 * 
 * @details Updates the control_sensors_present, control_sensors_enabled, and 
 *          control_sensors_health bitmasks that are reported to the ground control station
 *          via MAVLink SYS_STATUS messages. These flags indicate which sensors and
 *          control capabilities are available, active, and functioning correctly.
 *          
 *          For AntennaTracker, this sets flags for the core tracking capabilities:
 *          - ANGULAR_RATE_CONTROL: Ability to control servo angular rates
 *          - ATTITUDE_STABILIZATION: Attitude-based pointing control
 *          - YAW_POSITION: Yaw angle positioning for azimuth tracking
 *          
 *          These flags are simplified compared to full flight vehicles since the
 *          tracker does not require flight sensors (GPS, accelerometer, etc.) for
 *          basic operation - only servo control for pointing.
 * 
 * @note This is called periodically by the GCS base class to refresh status flags
 * @note All three flags (present, enabled, health) are set to indicate fully
 *       operational tracking control capabilities
 * @note The base class handles GPS, battery, and other common sensors
 * 
 * @see GCS::update_sensor_status_flags() - Base class method that calls this
 * 
 * Source: AntennaTracker/GCS_Tracker.cpp:36-54
 */
void GCS_Tracker::update_vehicle_sensor_status_flags()
{
    // default sensors present
    control_sensors_present |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;

    control_sensors_enabled |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;

    control_sensors_health |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;
}

/**
 * @brief Stub implementation to prevent linking LTM telemetry library
 * 
 * @details Provides empty init() method for AP_LTM_Telem to satisfy linker
 *          requirements when AP_LTM_TELEM_ENABLED is defined but the full
 *          LTM telemetry implementation is not needed for AntennaTracker.
 *          
 *          This avoids including unnecessary telemetry protocol code in the
 *          tracker firmware, reducing flash memory usage.
 * 
 * @note This is a build optimization specific to AntennaTracker
 * @note LTM (Light Telemetry) is typically used for FPV OSD systems
 */
#if AP_LTM_TELEM_ENABLED
// avoid building/linking LTM:
void AP_LTM_Telem::init() {};
#endif

/**
 * @brief Stub implementation to prevent linking Devo telemetry library
 * 
 * @details Provides empty init() method for AP_DEVO_Telem to satisfy linker
 *          requirements when AP_DEVO_TELEM_ENABLED is defined but the full
 *          Devo telemetry implementation is not needed for AntennaTracker.
 *          
 *          This avoids including unnecessary telemetry protocol code in the
 *          tracker firmware, reducing flash memory usage.
 * 
 * @note This is a build optimization specific to AntennaTracker
 * @note Devo telemetry is used for Walkera Devo RC transmitters
 */
#if AP_DEVO_TELEM_ENABLED
// avoid building/linking Devo:
void AP_DEVO_Telem::init() {};
#endif
