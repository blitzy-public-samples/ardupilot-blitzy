/**
 * @file GCS_Sub.cpp
 * @brief ArduSub-specific Ground Control Station (GCS) configuration and MAVLink telemetry
 * 
 * @details This file implements the ArduSub-specific customization of the GCS_MAVLink
 *          telemetry system. It handles vehicle-specific sensor status reporting,
 *          MAVLink message streaming configuration, and underwater vehicle state
 *          communication with ground control stations.
 * 
 *          Key responsibilities:
 *          - Configure MAVLink sensor status flags for underwater vehicle sensors
 *          - Override parent class sensor reporting for underwater-specific sensors
 *          - Report depth control and position control capabilities based on flight mode
 *          - Handle water pressure sensors differently from air pressure sensors
 *          - Manage optional terrain and rangefinder status reporting
 * 
 *          Integration:
 *          - Extends GCS_MAVLink base class with ArduSub-specific behavior
 *          - Works with Sub vehicle singleton for state information
 *          - Communicates sensor health through MAVLink SYS_STATUS messages
 * 
 * @note This file is part of the ArduSub vehicle implementation and should only
 *       be compiled for submarine/ROV configurations
 * 
 * @see GCS_Sub.h for class declaration
 * @see GCS_MAVLink class in libraries/GCS_MAVLink/ for base implementation
 * 
 * Source: ArduSub/GCS_Sub.cpp
 */

#include "GCS_Sub.h"

#include "Sub.h"

/**
 * @brief Update MAVLink sensor status flags for ArduSub vehicle
 * 
 * @details This function updates the vehicle sensor status flags that are reported
 *          to ground control stations via MAVLink SYS_STATUS messages. It sets three
 *          categories of flags for each sensor type:
 *          - control_sensors_present: Sensors physically present/detected
 *          - control_sensors_enabled: Sensors currently enabled and in use
 *          - control_sensors_health: Sensors functioning correctly
 * 
 *          ArduSub-Specific Behavior:
 *          
 *          1. **Always-Present Control Sensors**:
 *             Reports angular rate control, attitude stabilization, and yaw position
 *             as always present, enabled, and healthy for underwater vehicles.
 * 
 *          2. **Mode-Dependent Position Control**:
 *             Altitude (depth) and XY position control are marked enabled/healthy
 *             only in modes that actively use position/depth control:
 *             - ALT_HOLD: Depth hold mode
 *             - AUTO: Autonomous mission mode
 *             - GUIDED: Guided positioning mode
 *             - CIRCLE: Circular pattern mode
 *             - SURFACE: Surface ascent mode
 *             - POSHOLD: Position and depth hold mode
 * 
 *          3. **Underwater Pressure Sensors**:
 *             CRITICAL: Overrides parent class absolute pressure sensor reporting
 *             to ONLY report water pressure (depth) sensors, not air pressure sensors.
 *             This is essential because submarines use water pressure for depth
 *             measurement, not barometric altitude.
 * 
 *          4. **Optional Sensor Systems**:
 *             - Terrain: Reports terrain database availability if compiled in
 *             - Rangefinder: Reports downward-facing rangefinder status
 * 
 * @note This function is called periodically by the GCS_MAVLink update loop
 *       to keep ground control stations informed of vehicle sensor health
 * 
 * @warning Do not remove the ABSOLUTE_PRESSURE override - it is critical for
 *          proper depth sensor reporting on underwater vehicles. Air pressure
 *          sensors are meaningless underwater and must not be reported.
 * 
 * @see MAV_SYS_STATUS_SENSOR enum in MAVLink common.xml for sensor flag definitions
 * @see GCS_MAVLink::update_sensor_status_flags() for parent implementation
 * 
 * Source: ArduSub/GCS_Sub.cpp:5-83
 */
void GCS_Sub::update_vehicle_sensor_status_flags()
{
    // Always report core stabilization sensors as present, enabled, and healthy
    // These are fundamental to underwater vehicle control
    control_sensors_present |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |      // Gyro-based rate control
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |    // Attitude (roll/pitch) control
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;               // Compass-based yaw control

    control_sensors_enabled |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;

    control_sensors_health |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;

    // Position and altitude (depth) control sensors are always present hardware-wise,
    // but only enabled/healthy in modes that actively use position control
    control_sensors_present |=
        MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |        // Depth hold capability
        MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;        // Horizontal position hold

    // Enable position control sensors based on current flight mode
    // Only modes that actively control position/depth mark these sensors as enabled and healthy
    switch (sub.control_mode) {
    case Mode::Number::ALT_HOLD:    // Depth hold mode - uses Z control
    case Mode::Number::AUTO:        // Autonomous mission - uses full 3D position control
    case Mode::Number::GUIDED:      // Guided mode - uses full 3D position control
    case Mode::Number::CIRCLE:      // Circle mode - uses XY position control
    case Mode::Number::SURFACE:     // Surface mode - uses Z control to ascend
    case Mode::Number::POSHOLD:     // Position hold - uses full 3D position control
        // Mark depth (Z) and horizontal position (XY) control as enabled and healthy
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        break;
    default:
        // Manual modes (MANUAL, ACRO, STABILIZE) do not use position control
        // Sensors remain present but not enabled
        break;
    }

    // CRITICAL UNDERWATER VEHICLE BEHAVIOR:
    // Override the parent class's ABSOLUTE_PRESSURE reporting to only honor
    // water pressure (depth) sensors, NOT barometric (air) pressure sensors.
    // 
    // Rationale: Underwater vehicles use water pressure to measure depth, which is
    // fundamentally different from aircraft using air pressure to measure altitude.
    // Air pressure sensors provide no useful information when submerged.
    // 
    // First, clear any air pressure sensor flags set by parent class
    control_sensors_present &= ~MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    control_sensors_enabled &= ~MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    
    // Then, set flags ONLY if we have a valid water pressure (depth) sensor
    if (sub.ap.depth_sensor_present) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
        // Mark sensor as healthy only if depth sensor is actually working
        if (sub.sensor_health.depth) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
        }
    }

    // Optional terrain database support for underwater bathymetry
    // Terrain data can be used for bottom-following or obstacle avoidance
#if AP_TERRAIN_AVAILABLE
    switch (sub.terrain.status()) {
    case AP_Terrain::TerrainStatusDisabled:
        // Terrain system is compiled in but disabled - don't report
        break;
    case AP_Terrain::TerrainStatusUnhealthy:
        // Terrain system is enabled but database is unavailable or corrupted
        // @todo Restore unhealthy terrain status reporting once terrain is actively used in Sub
        // Currently commented out because terrain features are not fully implemented for underwater use
        //control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        //control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        //break;
    case AP_Terrain::TerrainStatusOK:
        // Terrain database is loaded and healthy - report as available
        control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_health  |= MAV_SYS_STATUS_TERRAIN;
        break;
    }
#endif

    // Optional rangefinder (distance sensor) support for bottom distance measurement
    // Typically used for terrain following or precision landing on underwater structures
#if AP_RANGEFINDER_ENABLED
    const RangeFinder *rangefinder = RangeFinder::get_singleton();
    if (sub.rangefinder_state.enabled) {
        // Rangefinder is configured and enabled
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        // Mark as healthy only if downward-facing rangefinder (ROTATION_PITCH_270) has valid data
        // ROTATION_PITCH_270 = pointing straight down relative to vehicle body frame
        if (rangefinder && rangefinder->has_data_orient(ROTATION_PITCH_270)) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        }
    }
#endif
}

/**
 * @brief Stub implementation to disable LTM (Lightweight Telemetry) for ArduSub
 * 
 * @details LTM telemetry is designed for aircraft and is not applicable to underwater
 *          vehicles. This stub prevents the LTM telemetry module from being linked
 *          into ArduSub binaries, saving flash space and avoiding unnecessary overhead.
 * 
 * @note This stub is only compiled when AP_LTM_TELEM_ENABLED is defined but we want
 *       to explicitly disable it for submarine applications
 */
#if AP_LTM_TELEM_ENABLED
// Avoid building/linking LTM telemetry module for ArduSub
void AP_LTM_Telem::init() {};
#endif

/**
 * @brief Stub implementation to disable Devo telemetry for ArduSub
 * 
 * @details Devo telemetry is designed for surface vehicles and aircraft, and is not
 *          applicable to underwater ROV/submarine operations. This stub prevents the
 *          Devo telemetry module from being linked, saving flash space.
 * 
 * @note This stub is only compiled when AP_DEVO_TELEM_ENABLED is defined but we want
 *       to explicitly disable it for submarine applications
 */
#if AP_DEVO_TELEM_ENABLED
// Avoid building/linking Devo telemetry module for ArduSub
void AP_DEVO_Telem::init() {};
#endif
