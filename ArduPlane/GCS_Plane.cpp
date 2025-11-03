/**
 * @file GCS_Plane.cpp
 * @brief ArduPlane-specific Ground Control Station (GCS) interface implementation
 * 
 * @details This file implements the GCS_Plane class which provides the ArduPlane-specific
 *          interface for MAVLink communication with ground control stations. It handles:
 *          - Vehicle-specific sensor status reporting
 *          - Mode-dependent control capability flags
 *          - Reverse thrust status reporting
 *          - Terrain following status (when available)
 *          - Rangefinder status for landing operations
 *          - Quadplane-specific mode handling (when enabled)
 * 
 *          The GCS_Plane class inherits from GCS (in GCS_MAVLink library) and provides
 *          ArduPlane-specific implementations of virtual methods for telemetry and
 *          status reporting. This enables ground stations to understand vehicle
 *          capabilities and current operational status.
 * 
 * @note This file focuses on sensor status flag management. Other GCS functionality
 *       (message handling, command processing, telemetry streaming) is implemented
 *       in GCS_MAVLink_Plane.cpp and the base GCS_MAVLink library.
 * 
 * @see GCS_Plane.h for class definition
 * @see GCS_MAVLink_Plane.cpp for message handling implementation
 * @see libraries/GCS_MAVLink/ for base GCS implementation
 * 
 * Source: ArduPlane/GCS_Plane.cpp
 */

#include "GCS_Plane.h"
#include "Plane.h"

/**
 * @brief Update vehicle sensor status flags for MAVLink SYS_STATUS message
 * 
 * @details This method updates the control_sensors_present, control_sensors_enabled,
 *          and control_sensors_health flags that are reported to the ground station
 *          via the MAVLink SYS_STATUS message. These flags communicate which sensors
 *          and control capabilities are available, enabled, and healthy.
 * 
 *          The flags are mode-dependent and reflect the current vehicle capabilities:
 *          - MANUAL mode: No stabilization or control
 *          - ACRO modes: Rate control only
 *          - STABILIZE/FBWA/FBWB/CRUISE: Rate + attitude stabilization
 *          - AUTO/RTL/LOITER/GUIDED: Full 3D position control
 * 
 *          Additional status flags reported:
 *          - Reverse thrust capability and active state
 *          - Terrain following system status (if AP_TERRAIN_AVAILABLE)
 *          - Rangefinder status for landing operations (if AP_RANGEFINDER_ENABLED)
 *          - Quadplane mode capabilities (if HAL_QUADPLANE_ENABLED)
 * 
 *          Control Sensor Flags Explained:
 *          - control_sensors_present: Hardware/feature is present in vehicle
 *          - control_sensors_enabled: Feature is currently enabled/active
 *          - control_sensors_health: Feature is functioning correctly
 * 
 * @note This method is called periodically by the GCS telemetry update loop,
 *       typically at 1-10 Hz depending on stream rate configuration
 * 
 * @note The flags use MAV_SYS_STATUS_SENSOR_* bitmask values defined in
 *       the MAVLink protocol specification
 * 
 * @warning Incorrect flag reporting can cause ground station warnings or prevent
 *          arming. Ensure mode classification matches actual vehicle control behavior.
 * 
 * @see GCS::update() for the calling context
 * @see mavlink_msg_sys_status_send() for how these flags are transmitted
 * 
 * Source: ArduPlane/GCS_Plane.cpp:4-127
 */
void GCS_Plane::update_vehicle_sensor_status_flags(void)
{
    // Report reverse thrust capability and active state
    // Reverse thrust allows fixed-wing aircraft to apply negative thrust for braking
    // during landing or ground operations
    if (plane.have_reverse_thrust()) {
        // Vehicle has reverse thrust capability configured
        control_sensors_present |= MAV_SYS_STATUS_REVERSE_MOTOR;
    }
    if (plane.have_reverse_thrust() && is_negative(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle))) {
        // Reverse thrust is currently active (negative throttle output)
        control_sensors_enabled |= MAV_SYS_STATUS_REVERSE_MOTOR;
        control_sensors_health |= MAV_SYS_STATUS_REVERSE_MOTOR;
    }

    // Report flight mode-specific control capabilities
    // All fixed-wing modes have the potential for rate control, attitude stabilization,
    // and position control depending on the current mode
    control_sensors_present |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |      // 3D angular rate control (roll/pitch/yaw rates)
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |    // Attitude hold (level flight)
        MAV_SYS_STATUS_SENSOR_YAW_POSITION |              // Heading/yaw position control
        MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |        // Altitude hold
        MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;        // Horizontal position control (loiter, waypoint nav)

    // Determine current control loop status based on flight mode
    // These flags indicate which control loops are currently active
    bool rate_controlled = false;      // True if rate PIDs are active
    bool attitude_stabilized = false;  // True if attitude controller is stabilizing
    // Classify current flight mode by control capabilities
    // Mode categories (in order of increasing autonomy):
    // 1. MANUAL - Direct servo control, no stabilization
    // 2. ACRO - Rate control only (gyro stabilization)
    // 3. STABILIZE/FBWA/FBWB/CRUISE - Rate + attitude stabilization
    // 4. AUTO/RTL/LOITER/GUIDED - Full 3D position control
    switch (plane.control_mode->mode_number()) {
    case Mode::Number::MANUAL:
        // Manual mode: Direct pilot control with no stabilization
        // Rate and attitude controllers are completely disabled
        break;

    case Mode::Number::ACRO:
        // Acrobatic mode: Rate control only
        // Pilot controls angular rates directly, no attitude hold
#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QACRO:
        // Quadplane acrobatic mode: VTOL rate control
#endif
        rate_controlled = true;
        break;

    case Mode::Number::STABILIZE:
        // Stabilize mode: Attitude stabilization with pilot input
    case Mode::Number::FLY_BY_WIRE_A:
        // FBWA: Stabilized mode with airspeed/altitude hold
    case Mode::Number::AUTOTUNE:
        // AutoTune: Automated PID tuning in stabilized flight
#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QSTABILIZE:
        // Quadplane stabilize: VTOL attitude stabilization
    case Mode::Number::QHOVER:
        // Quadplane hover: VTOL position hold
    case Mode::Number::QLAND:
        // Quadplane land: VTOL automated landing
    case Mode::Number::QLOITER:
        // Quadplane loiter: VTOL position hold with GPS
#if QAUTOTUNE_ENABLED
    case Mode::Number::QAUTOTUNE:
        // Quadplane autotune: VTOL PID tuning
#endif
#endif  // HAL_QUADPLANE_ENABLED
    case Mode::Number::FLY_BY_WIRE_B:
        // FBWB: Stabilized mode with altitude and airspeed hold
    case Mode::Number::CRUISE:
        // Cruise: Level flight with ground track hold
        rate_controlled = true;
        attitude_stabilized = true;
        break;

    case Mode::Number::TRAINING:
        // Training mode: Configurable stabilization assist
        // Stabilization can be disabled per-axis for training purposes
        if (!plane.training_manual_roll || !plane.training_manual_pitch) {
            // At least one axis has stabilization enabled
            rate_controlled = true;
            attitude_stabilized = true;
        }
        break;

    case Mode::Number::AUTO:
        // Auto mode: Full autonomous mission execution
    case Mode::Number::RTL:
        // Return to Launch: Autonomous return and landing
    case Mode::Number::LOITER:
        // Loiter: Circle around a point at fixed altitude
    case Mode::Number::AVOID_ADSB:
        // ADSB Avoidance: Autonomous collision avoidance
    case Mode::Number::GUIDED:
        // Guided mode: GCS-commanded position targets
    case Mode::Number::CIRCLE:
        // Circle mode: Continuous circling around a point
    case Mode::Number::TAKEOFF:
        // Takeoff mode: Autonomous takeoff sequence
#if MODE_AUTOLAND_ENABLED
    case Mode::Number::AUTOLAND:
        // Autoland mode: Autonomous landing approach
#endif
#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QRTL:
        // Quadplane RTL: VTOL return and landing
    case Mode::Number::LOITER_ALT_QLAND:
        // Loiter with VTOL landing at altitude trigger
#endif
    case Mode::Number::THERMAL:
        // Thermal mode: Autonomous thermal soaring
        // All autonomous modes have full 3D control active
        rate_controlled = true;
        attitude_stabilized = true;
        // Report that yaw, altitude, and horizontal position control are active and healthy
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_YAW_POSITION;
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_YAW_POSITION;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        break;

    case Mode::Number::INITIALISING:
        // Initializing mode: System startup, no control active
        break;
    }

    // Update enabled and health flags based on active control loops
    if (rate_controlled) {
        // Rate controller is active - gyro-based rate stabilization is running
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL;
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL;
    }
    if (attitude_stabilized) {
        // Attitude controller is active - maintaining level flight or pilot-commanded attitude
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;
    }

#if AP_TERRAIN_AVAILABLE
    // Report terrain following system status
    // Terrain following allows the vehicle to maintain altitude above ground level (AGL)
    // rather than above mean sea level (MSL), useful for low-altitude flight over
    // varying terrain
    switch (plane.terrain.status()) {
    case AP_Terrain::TerrainStatusDisabled:
        // Terrain following is disabled or not configured
        break;
    case AP_Terrain::TerrainStatusUnhealthy:
        // Terrain system is present and enabled but not currently healthy
        // This can occur when terrain data is unavailable or position accuracy is insufficient
        control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        break;
    case AP_Terrain::TerrainStatusOK:
        // Terrain system is present, enabled, and providing valid data
        control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_health  |= MAV_SYS_STATUS_TERRAIN;
        break;
    }
#endif

#if AP_RANGEFINDER_ENABLED
    // Report rangefinder (distance sensor) status
    // Rangefinders are typically used for precision landing, terrain following,
    // and obstacle avoidance. Most commonly mounted pointing downward for
    // altitude-above-ground measurement during landing.
    const RangeFinder *rangefinder = RangeFinder::get_singleton();
    if (rangefinder && rangefinder->has_orientation(plane.rangefinder_orientation())) {
        // Rangefinder hardware is present with the configured orientation
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        
        // Check if rangefinder is enabled for landing operations
        // RNGFND_LANDING parameter determines if rangefinder is used for landing altitude
        if (uint16_t(plane.g.rangefinder_landing.get()) != 0) {
            // Rangefinder is configured for use in landing (non-zero = enabled)
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        }
        
        // Check if rangefinder is providing valid data
        if (rangefinder->has_data_orient(plane.rangefinder_orientation())) {
            // Rangefinder is returning valid distance measurements
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;            
        }
    }
#endif
}
