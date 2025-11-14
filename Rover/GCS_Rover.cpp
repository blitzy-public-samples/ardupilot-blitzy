/**
 * @file GCS_Rover.cpp
 * @brief Implementation of rover GCS (Ground Control Station) telemetry and status reporting
 * 
 * @details This file provides the rover-specific implementation of the GCS interface,
 *          handling telemetry data reporting and status flag management for communication
 *          with ground control stations via MAVLink protocol.
 *          
 *          Key responsibilities:
 *          - Report simple mode status for input handling
 *          - Update MAVLink sensor status flags based on rover control mode and hardware
 *          - Provide vehicle-specific telemetry customization
 *          
 *          Coordinate System: Uses NED (North-East-Down) earth frame for position data
 *          Communication Protocol: MAVLink for all ground station communication
 * 
 * @note This file implements methods declared in GCS_Rover.h
 * @see GCS_Rover.h for class definition
 * @see GCS_MAVLink_Rover.cpp for MAVLink message handling
 */

#include "GCS_Rover.h"

#include "Rover.h"

#include <AP_RangeFinder/AP_RangeFinder_Backend.h>

/**
 * @brief Check if simple mode with initial heading is currently active
 * 
 * @details Simple mode simplifies rover control by maintaining a fixed reference heading.
 *          This method reports whether the vehicle is in simple mode with the initial
 *          heading variant, which locks the control reference to the heading at mode entry.
 *          
 *          This status is used by the GCS to display appropriate control mode indicators
 *          and adjust telemetry displays for operator awareness.
 * 
 * @return true if rover is in simple mode with initial heading type
 * @return false if rover is not in simple mode or using a different simple type
 * 
 * @note Called during telemetry updates to report control mode status to GCS
 * @see supersimple_input_active() for cardinal direction variant
 */
bool GCS_Rover::simple_input_active() const
{
    if (rover.control_mode != &rover.mode_simple) {
        return false;
    }
    return (rover.g2.simple_type == ModeSimple::Simple_InitialHeading);
}

/**
 * @brief Check if simple mode with cardinal directions is currently active
 * 
 * @details Super-simple mode is a variant of simple mode that aligns control inputs
 *          to cardinal directions (North, South, East, West) rather than a fixed heading.
 *          This simplifies operator control by providing intuitive directional commands
 *          regardless of vehicle orientation.
 *          
 *          This status is reported to the GCS for display and to help operators understand
 *          the current input interpretation mode.
 *          
 *          Coordinate System: Cardinal directions referenced to North in NED frame
 * 
 * @return true if rover is in simple mode with cardinal directions type
 * @return false if rover is not in simple mode or using a different simple type
 * 
 * @note Called during telemetry updates to report control mode status to GCS
 * @see simple_input_active() for initial heading variant
 */
bool GCS_Rover::supersimple_input_active() const
{
    if (rover.control_mode != &rover.mode_simple) {
        return false;
    }
    return (rover.g2.simple_type == ModeSimple::Simple_CardinalDirections);
}

/**
 * @brief Update MAVLink sensor status flags for rover vehicle telemetry
 * 
 * @details This method updates the control_sensors_present, control_sensors_enabled,
 *          and control_sensors_health flags that are reported to the ground control
 *          station via MAVLink SYS_STATUS messages. These flags inform the GCS about
 *          which control capabilities and sensors are available, enabled, and healthy.
 *          
 *          The flags are updated based on:
 *          - Current control mode capabilities (attitude stabilization, position control)
 *          - Proximity sensor availability and health
 *          - Rangefinder sensor availability and data validity
 *          
 *          Status Flag Meanings:
 *          - control_sensors_present: Hardware/software capability exists
 *          - control_sensors_enabled: Capability is actively enabled for current mode
 *          - control_sensors_health: Capability is functioning correctly
 *          
 *          Coordinate System: Position control flags reference NED (North-East-Down) frame
 *          Units: Status flags are bitmasks defined by MAVLink protocol
 * 
 * @note Called periodically during telemetry updates to maintain current status
 * @note Flags are cumulative - this method sets bits without clearing existing ones
 * 
 * @see MAV_SYS_STATUS_SENSOR enum in MAVLink common.xml for flag definitions
 */
void GCS_Rover::update_vehicle_sensor_status_flags(void)
{
    // Mode-specific control capabilities that rover can support
    // Mark these as present (available) regardless of current mode state
    control_sensors_present |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |      // Yaw rate control capability
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |    // Heading stabilization capability
        MAV_SYS_STATUS_SENSOR_YAW_POSITION |              // Yaw position hold capability
        MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;        // Position control capability

    // Check if current mode provides attitude (heading) stabilization
    // Modes like Steering, Acro, and Auto provide active heading control
    if (rover.control_mode->attitude_stabilized()) {
        // Enable angular rate control flags - rover uses yaw rate control for steering
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL;
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL;
        
        // Enable attitude stabilization flags - rover maintains desired heading
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;
    }
    
    // Check if current mode is an autonomous navigation mode
    // Modes like Auto, Guided, RTL, SmartRTL provide position control
    if (rover.control_mode->is_autopilot_mode()) {
        // Enable yaw position control - autopilot modes actively control heading to target
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_YAW_POSITION;
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_YAW_POSITION;
        
        // Enable X/Y position control - autopilot modes navigate to waypoints in NED frame
        // Units: Position control works in meters in earth-fixed NED frame
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
    }

#if HAL_PROXIMITY_ENABLED
    // Proximity sensors provide 360-degree obstacle detection for collision avoidance
    // Common sensors: lightware SF40C, TeraRanger Tower, RPLidar
    // Coordinate System: Distances measured in body frame, multiple sectors around vehicle
    // Units: Distance typically in meters
    const AP_Proximity *proximity = AP_Proximity::get_singleton();
    
    // Check if proximity sensor is connected and detected
    if (proximity && proximity->get_status() > AP_Proximity::Status::NotConnected) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;  // Sensor hardware detected
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;  // Sensor active and enabled
    }
    
    // Check if proximity sensor is providing valid data
    if (proximity && proximity->get_status() != AP_Proximity::Status::NoData) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;   // Sensor healthy with valid data
    }
#endif

#if AP_RANGEFINDER_ENABLED
    // Rangefinders provide distance-to-obstacle measurements for object detection
    // Common sensors: Lidar (Lightware, Benewake), Sonar (MaxBotix), Radar
    // Typical mounting: Forward-facing for obstacle avoidance
    // Coordinate System: Distances measured along sensor axis in body frame
    // Units: Distance in meters, typically 0.1m to 40m range depending on sensor
    const RangeFinder *rangefinder = RangeFinder::get_singleton();
    
    // Check if at least one rangefinder is configured
    if (rangefinder && rangefinder->num_sensors() > 0) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;  // Rangefinder hardware configured
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;  // Rangefinder active
        
        // Check primary (first) rangefinder for valid data
        AP_RangeFinder_Backend *s = rangefinder->get_backend(0);
        if (s != nullptr && s->has_data()) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;  // Primary rangefinder healthy
        }
    }
#endif
}
