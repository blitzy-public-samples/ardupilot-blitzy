/**
 * @file GCS_Copter.cpp
 * @brief ArduCopter-specific Ground Control Station (GCS) implementation
 * 
 * @details This file implements the GCS_Copter class which provides vehicle-specific
 *          MAVLink telemetry and Ground Control Station communication functionality
 *          for multicopter vehicles. It handles:
 *          - Vehicle sensor status reporting for MAVLink SYS_STATUS messages
 *          - Frame type identification and reporting
 *          - Simple/SuperSimple mode status for ground station display
 *          - Multicopter-specific control capability flags
 * 
 *          The GCS_Copter class extends the base GCS class and provides copter-specific
 *          implementations of telemetry functions that are periodically sent to ground
 *          control stations via MAVLink protocol.
 * 
 * @note This file contains vehicle-specific GCS functionality. Core MAVLink
 *       message handling is in GCS_MAVLink_Copter.cpp
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include "GCS_Copter.h"

#include "Copter.h"

/**
 * @brief Get human-readable frame type string for telemetry display
 * 
 * @details Returns a descriptive string identifying the multicopter frame configuration
 *          for display in ground control stations and telemetry logs. Frame types include
 *          configurations like "Quad", "Hexa", "Octa", "Y6", etc., which are determined
 *          by the motor library based on the actual frame configuration loaded.
 * 
 *          This string is used in MAVLink AUTOPILOT_VERSION messages to inform ground
 *          stations about the vehicle's physical configuration. The frame type affects
 *          how ground stations display the vehicle icon and provide appropriate tuning
 *          interfaces.
 * 
 * @return Pointer to constant string describing frame type (e.g., "Quad", "X8", "Y6")
 *         Returns "MultiCopter" as a safe default if motor library is not initialized
 * 
 * @note This function is called during GCS connection establishment and may be
 *       rate-limited for periodic telemetry updates
 * 
 * @see AP_Motors::get_frame_string() for frame type determination logic
 * @see GCS_MAVLINK::send_autopilot_version() for usage in MAVLink messages
 * 
 * Source: ArduCopter/GCS_Copter.cpp:25-31
 */
const char* GCS_Copter::frame_string() const
{
    // Check if motors library is initialized - required for frame identification
    if (copter.motors == nullptr) {
        // Return generic frame type if motors not yet initialized (e.g., during early boot)
        return "MultiCopter";
    }
    // Delegate to motors library which knows the actual frame configuration
    return copter.motors->get_frame_string();
}

/**
 * @brief Check if Simple mode is currently active for pilot input processing
 * 
 * @details Simple mode is a flight assistance feature that maintains yaw orientation
 *          relative to the arming direction, making it easier for pilots to control
 *          the vehicle when it's far away or oriented in different directions. When
 *          active, forward stick input always moves the vehicle away from the pilot
 *          regardless of the vehicle's yaw orientation.
 * 
 *          This status is reported to ground control stations in MAVLink messages so
 *          the GCS can display the current simple mode state to the operator. The mode
 *          can be activated via RC channel options or as part of specific flight modes.
 * 
 * @return true if Simple mode is currently active
 * @return false if Simple mode is not active (normal or SuperSimple mode instead)
 * 
 * @note Simple mode affects roll/pitch input interpretation but does not affect
 *       vehicle stabilization or navigation algorithms
 * 
 * @warning Do not confuse with SuperSimple mode - they are mutually exclusive states
 * 
 * @see Copter::simple_mode for mode state management
 * @see supersimple_input_active() for SuperSimple mode status
 * @see Copter::update_simple_mode() for mode activation logic
 * 
 * Source: ArduCopter/GCS_Copter.cpp:46-49
 */
bool GCS_Copter::simple_input_active() const
{
    // Check if simple mode is set to SIMPLE (not NONE or SUPERSIMPLE)
    return copter.simple_mode == Copter::SimpleMode::SIMPLE;
}

/**
 * @brief Check if SuperSimple mode is currently active for pilot input processing
 * 
 * @details SuperSimple mode is an advanced flight assistance feature that extends
 *          Simple mode by automatically maintaining the vehicle's orientation relative
 *          to the home/arming position. The vehicle continuously adjusts its heading
 *          so that forward stick input always moves away from home, backward moves
 *          toward home, and left/right moves perpendicular to the home direction.
 * 
 *          This status is reported to ground control stations in MAVLink messages for
 *          operator situational awareness. SuperSimple mode is particularly useful for
 *          beginners or when flying at extended ranges where visual orientation is difficult.
 * 
 * @return true if SuperSimple mode is currently active
 * @return false if SuperSimple mode is not active (normal or Simple mode instead)
 * 
 * @note SuperSimple mode continuously updates yaw based on position relative to home,
 *       consuming more CPU than Simple mode but providing better orientation assistance
 * 
 * @warning SuperSimple and Simple modes are mutually exclusive - only one can be active
 *          at a time. SuperSimple takes precedence when both are configured.
 * 
 * @see Copter::simple_mode for mode state management
 * @see simple_input_active() for basic Simple mode status
 * @see Copter::update_simple_mode() for mode activation and heading adjustment logic
 * 
 * Source: ArduCopter/GCS_Copter.cpp:70-73
 */
bool GCS_Copter::supersimple_input_active() const
{
    // Check if simple mode is set to SUPERSIMPLE (not NONE or SIMPLE)
    return copter.simple_mode == Copter::SimpleMode::SUPERSIMPLE;
}

/**
 * @brief Update vehicle sensor and control capability status flags for MAVLink telemetry
 * 
 * @details This function is called periodically (typically at 1-5 Hz) to update the sensor
 *          and control system status flags that are transmitted to ground control stations
 *          in MAVLink SYS_STATUS messages. It populates three flag categories:
 * 
 *          - control_sensors_present: Hardware/software capabilities that exist in this build
 *          - control_sensors_enabled: Capabilities currently enabled by parameters/configuration
 *          - control_sensors_health: Capabilities currently functioning correctly
 * 
 *          Ground stations use these flags to:
 *          - Display sensor health status indicators to the operator
 *          - Determine if the vehicle is ready for arming
 *          - Provide appropriate warnings about degraded functionality
 *          - Show which control modes are available
 * 
 *          The function reports multicopter-specific control capabilities (attitude, rate,
 *          position controllers) as well as optional sensor systems (rangefinder, proximity,
 *          precision landing, terrain) based on their availability and health status.
 * 
 * @note This function modifies inherited status flag members (control_sensors_*)
 *       Rate-limited telemetry streaming prevents bandwidth saturation on low-speed links
 * 
 * @warning Incorrectly reporting sensor health can lead to inappropriate arming/flying
 *          decisions by pilots relying on ground station displays
 * 
 * @see MAVLink SYS_STATUS message definition (message ID 1)
 * @see GCS::update_sensor_status_flags() for base class implementation
 * @see MAV_SYS_STATUS_SENSOR enum for flag bit definitions
 * 
 * Source: ArduCopter/GCS_Copter.cpp:99-193
 */
void GCS_Copter::update_vehicle_sensor_status_flags(void)
{
    // =============================================================================
    // CORE MULTICOPTER CONTROL CAPABILITIES - Always present, enabled, and healthy
    // =============================================================================
    // These represent fundamental multicopter control modes that are always available
    // in the firmware. If these were not functional, the vehicle would not be flyable.
    
    // Report angular rate control capability (rate mode stabilization)
    // This is the innermost control loop that stabilizes gyro rates
    control_sensors_present |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;

    // These control modes are always enabled for multicopters
    control_sensors_enabled |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;

    // Mark as healthy - core stabilization always functions if firmware is running
    control_sensors_health |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;

    // =============================================================================
    // POSITION AND ALTITUDE CONTROL CAPABILITIES
    // =============================================================================
    // Position control is present if the controller is initialized, but only enabled/healthy
    // when actively being used by the current flight mode (e.g., Loiter, PosHold, Auto)
    
    if (copter.pos_control != nullptr) {
        // Position control capability exists in this firmware build
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;

        // Check if horizontal (North-East) position controller is actively running
        // This is true in modes like Loiter, PosHold, Auto, Guided that hold/navigate to positions
        if (copter.pos_control->is_active_NE()) {
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
            // Mark healthy - if controller is active, it's functioning (unhealthy states handled elsewhere)
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        }

        // Check if vertical (Up in NED frame, which is altitude) position controller is actively running
        // This is true in most flight modes that maintain altitude (AltHold, Loiter, Auto, etc.)
        if (copter.pos_control->is_active_U()) {
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
            // Mark healthy - active altitude control indicates proper barometer/rangefinder fusion
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        }
    }

    // =============================================================================
    // OPTIONAL SENSOR SYSTEMS
    // =============================================================================
    // These sensors may or may not be present depending on hardware configuration and
    // compile-time feature flags. Each sensor reports present/enabled/health separately.
    
#if HAL_PROXIMITY_ENABLED
    // Proximity sensors (360-degree obstacle detection via lidar, sonar, or radar)
    // Used for object avoidance in flight modes and pre-arm collision warnings
    if (copter.g2.proximity.sensor_present()) {
        // At least one proximity sensor is physically connected and detected
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_PROXIMITY;
    }
    if (copter.g2.proximity.sensor_enabled()) {
        // Proximity sensing is enabled via parameters (PRX_TYPE != 0)
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_PROXIMITY;
    }
    if (!copter.g2.proximity.sensor_failed()) {
        // Sensor is providing valid distance data (no timeouts or errors)
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_PROXIMITY;
    }
#endif

#if AP_RANGEFINDER_ENABLED
    // Downward-facing rangefinder (lidar/sonar for terrain following and precision landing)
    // ROTATION_PITCH_270 = pointing straight down in body frame
    const RangeFinder *rangefinder = RangeFinder::get_singleton();
    if (rangefinder && rangefinder->has_orientation(ROTATION_PITCH_270)) {
        // Downward-facing rangefinder is configured (hardware may or may not be connected)
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
    }
    if (copter.rangefinder_state.enabled) {
        // Rangefinder is enabled for use by flight modes (terrain following, surface tracking)
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        if (rangefinder && rangefinder->has_data_orient(ROTATION_PITCH_270)) {
            // Downward rangefinder is providing valid distance measurements (not timed out)
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        }
    }
#endif

#if AC_PRECLAND_ENABLED
    // Precision landing system (IR-LOCK beacon or visual target tracking)
    // Provides accurate position corrections for landing on specific targets
    if (copter.precland.enabled()) {
        // Precision landing is configured via parameters (PLND_ENABLED = 1)
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
    if (copter.precland.enabled() && copter.precland.healthy()) {
        // Precision landing sensor is tracking a valid target beacon
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
#endif

#if AP_TERRAIN_AVAILABLE
    // Terrain following system (uses downloaded terrain database or rangefinder)
    // Allows maintaining constant height above ground rather than sea level altitude
    switch (copter.terrain.status()) {
    case AP_Terrain::TerrainStatusDisabled:
        // Terrain following disabled via parameters (TERRAIN_ENABLE = 0)
        // Don't report any terrain capability to GCS
        break;
    case AP_Terrain::TerrainStatusUnhealthy:
        // Terrain enabled but data unavailable or outdated
        // To-Do: restore unhealthy terrain status reporting once terrain is used in copter
        // Currently commented out to avoid confusing GCS displays since terrain isn't
        // widely used in multicopter modes yet (mainly a Plane/Rover feature)
        //control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        //control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        //break;
    case AP_Terrain::TerrainStatusOK:
        // Terrain data available and current (downloaded from GCS or using rangefinder)
        control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_health  |= MAV_SYS_STATUS_TERRAIN;
        break;
    }
#endif

    // =============================================================================
    // PROPULSION SYSTEM STATUS
    // =============================================================================
    // Propulsion health indicates whether motors are producing expected thrust levels
    // Unhealthy propulsion is typically caused by thrust saturation (insufficient thrust
    // available to maintain desired attitude/altitude)
    
    // Propulsion is always present and enabled on multicopters (no fixed-wing config)
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_PROPULSION;
    control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_PROPULSION;
    
    // Mark propulsion healthy only if motors are not requesting thrust boost
    // Thrust boost indicates motor saturation - vehicle is demanding more thrust than
    // available (e.g., heavy payload, loss of motor, or aggressive maneuvers)
    if (!copter.motors->get_thrust_boost()) {
        // Motors operating within nominal thrust range - propulsion system healthy
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_PROPULSION;
    }
    // If thrust boost is active, propulsion health flag is NOT set, alerting GCS operator
    // that the vehicle may have degraded control authority or be overloaded
}
