/**
 * @file GCS_Blimp.cpp
 * @brief Ground Control Station (GCS) MAVLink interface implementation for Blimp vehicle
 * 
 * @details This file implements the GCS_Blimp class which provides the vehicle-specific
 *          Ground Control Station interface for lighter-than-air blimp vehicles. It extends
 *          the common GCS_MAVLink base class with blimp-specific functionality including:
 *          - Frame type identification for MAVLink reporting
 *          - Vehicle sensor capability flags for blimp control modes
 *          - MAVLink status reporting for altitude and position control
 *          
 *          The GCS_Blimp class acts as a wrapper that integrates the blimp vehicle with
 *          the ArduPilot MAVLink communication framework, enabling ground control station
 *          software to monitor and command blimp operations.
 *          
 *          Source: Blimp/GCS_Blimp.cpp:1-31
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include "GCS_Blimp.h"

#include "Blimp.h"

/**
 * @brief Get the blimp frame type string for MAVLink reporting
 * 
 * @details This method returns a human-readable string identifying the blimp frame
 *          configuration. It is used by ground control stations to display the
 *          vehicle frame type in their user interfaces. The frame string is obtained
 *          from the main blimp singleton instance.
 *          
 *          This method is called by the MAVLink telemetry system when reporting
 *          vehicle identification information to connected ground stations.
 * 
 * @return const char* Pointer to null-terminated string describing blimp frame type
 * 
 * @note This method accesses the global blimp singleton instance
 * @see Blimp::get_frame_string()
 */
const char* GCS_Blimp::frame_string() const
{
    return blimp.get_frame_string();
}

/**
 * @brief Update MAVLink sensor status flags for blimp vehicle capabilities
 * 
 * @details This method configures the MAVLink SYS_STATUS sensor capability flags to
 *          accurately report the blimp vehicle's control and sensor capabilities to
 *          ground control stations. It sets three categories of flags:
 *          
 *          **control_sensors_present**: Indicates which sensors/capabilities are physically
 *          present on the blimp hardware. This includes angular rate control, attitude
 *          stabilization, yaw position control, altitude control, and position control.
 *          
 *          **control_sensors_enabled**: Indicates which sensors/capabilities are currently
 *          enabled and active. For blimps, angular rate control, attitude stabilization,
 *          and yaw position control are enabled by default.
 *          
 *          **control_sensors_health**: Indicates the operational health status of each
 *          sensor/capability. Healthy flags for angular rate control, attitude stabilization,
 *          and yaw position control indicate these systems are functioning correctly.
 *          
 *          The method specifically reports:
 *          - MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL: Angular velocity control capability
 *          - MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION: Attitude stabilization capability
 *          - MAV_SYS_STATUS_SENSOR_YAW_POSITION: Yaw position hold capability
 *          - MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL: Vertical altitude control capability
 *          - MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL: Horizontal position control capability
 *          
 *          This information is transmitted via MAVLink HEARTBEAT and SYS_STATUS messages
 *          and allows ground stations to determine which flight modes and control options
 *          are available for the blimp vehicle.
 * 
 * @note This method is called periodically by the MAVLink telemetry update loop
 * @note The flags use bitwise OR operations to add capabilities to inherited base class flags
 * @note Altitude and position control are only marked as present, not necessarily enabled or healthy
 * 
 * @see GCS_MAVLINK::update_sensor_status_flags() for base implementation
 */
void GCS_Blimp::update_vehicle_sensor_status_flags(void)
{
    // mode-specific flags:
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

    control_sensors_present |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
}
