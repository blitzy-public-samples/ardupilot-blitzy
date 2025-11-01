/**
 * @file GCS_MAVLink_Copter.cpp
 * @brief Copter-specific MAVLink Ground Control Station communication implementation
 * 
 * @details This file implements the copter-specific MAVLink message handling and telemetry
 *          for communication with ground control stations (GCS). It extends the base GCS_MAVLINK
 *          class to provide multicopter-specific functionality including:
 *          
 *          - MAVLink command processing (MAV_CMD_* commands for copter operations)
 *          - Guided mode setpoint handling (position, velocity, acceleration, attitude targets)
 *          - Mission upload/download protocol handling
 *          - Parameter synchronization with ground stations
 *          - Telemetry streaming (attitude, position, status messages)
 *          - Mode change requests and validation
 *          - Arming/disarming command validation
 *          - Fence and rally point operations
 *          - Copter-specific message handlers for SET_POSITION_TARGET_*, SET_ATTITUDE_TARGET, etc.
 *          
 *          Key architectural patterns:
 *          - Message routing via handle_message() and handle_command_int_packet()
 *          - Coordinate frame transformations (NED/body frame/global coordinates)
 *          - Type mask parsing for selective field processing
 *          - Safety validation before accepting external control commands
 *          - Integration with flight mode system for guided operations
 *          
 * @note All MAVLink messages follow the MAVLink protocol specification
 * @warning Command handlers perform safety checks before executing - invalid commands are rejected
 * 
 * Source: ArduCopter/GCS_MAVLink_Copter.cpp
 */

#include "Copter.h"

#include "GCS_MAVLink_Copter.h"
#include <AP_RPM/AP_RPM_config.h>
#include <AP_EFI/AP_EFI_config.h>

/**
 * @brief Determine the MAVLink vehicle frame type for ground station display
 * 
 * @details Returns the appropriate MAV_TYPE enum value based on the configured frame type
 *          and motor configuration. This allows ground control stations to display the
 *          correct vehicle icon and provide appropriate UI controls for flight mode selection.
 *          
 *          Resolution order:
 *          1. If motors not allocated, return compile-time default (HELICOPTER or QUADROTOR)
 *          2. Query motor library for frame-specific MAV_TYPE
 *          3. If motor library returns MAV_TYPE_GENERIC, fall back to compile-time default
 *          
 *          Supports frame types: quadrotor, hexarotor, octorotor, helicopter, tri, Y6, etc.
 * 
 * @return MAV_TYPE enum value representing the vehicle frame type for MAVLink reporting
 *         - MAV_TYPE_HELICOPTER for traditional helicopters (FRAME_CONFIG == HELI_FRAME)
 *         - MAV_TYPE_QUADROTOR, MAV_TYPE_HEXAROTOR, etc. for multicopters
 *         - Never returns MAV_TYPE_GENERIC (falls back to default for GCS compatibility)
 * 
 * @note Called during telemetry streaming to populate HEARTBEAT message
 * @note GCS uses this value to determine UI layout and available flight modes
 */
MAV_TYPE GCS_Copter::frame_type() const
{
    // For GCS don't give MAV_TYPE_GENERIC as the GCS would have no
    // information and won't display UIs such as flight mode selection
#if FRAME_CONFIG == HELI_FRAME
    const MAV_TYPE mav_type_default = MAV_TYPE_HELICOPTER;
#else
    const MAV_TYPE mav_type_default = MAV_TYPE_QUADROTOR;
#endif
    // If motors not yet allocated (during early boot), return default type
    if (copter.motors == nullptr) {
        return mav_type_default;
    }
    // Query motor library for actual configured frame type
    MAV_TYPE mav_type = copter.motors->get_frame_mav_type();
    // Prevent returning GENERIC which provides no information to GCS
    if (mav_type == MAV_TYPE_GENERIC) {
        mav_type = mav_type_default;
    }
    return mav_type;
}

/**
 * @brief Calculate MAVLink base mode flags for current vehicle state
 * 
 * @details Constructs the base_mode field for MAVLink HEARTBEAT message using MAV_MODE_FLAG bits.
 *          This provides a standardized indication of vehicle state across different autopilot systems.
 *          
 *          Note: base_mode is not particularly useful for ArduPilot systems because the bit definitions
 *          are ambiguous and don't map well to ArduPilot's flight mode architecture. Ground stations
 *          should rely on custom_mode for accurate flight mode information. We populate base_mode
 *          as best we can for compatibility with generic MAVLink-enabled ground stations.
 *          
 *          Base mode flags set:
 *          - MAV_MODE_FLAG_STABILIZE_ENABLED: Always set (all modes provide stabilization)
 *          - MAV_MODE_FLAG_GUIDED_ENABLED: Set when position control is active (guided/auto modes)
 *          - MAV_MODE_FLAG_MANUAL_INPUT_ENABLED: Always set (stick mixing allows manual override)
 *          - MAV_MODE_FLAG_SAFETY_ARMED: Set when motors are armed
 *          - MAV_MODE_FLAG_CUSTOM_MODE_ENABLED: Always set (custom_mode field is valid)
 * 
 * @return uint8_t Bitmask of MAV_MODE_FLAG values representing vehicle state
 * 
 * @note MAV_MODE_FLAG_AUTO_ENABLED is intentionally NOT set as its definition
 *       ("system finds its own goal positions") doesn't match ArduPilot's behavior
 * @note Ground stations should use custom_mode() for accurate flight mode determination
 * 
 * @see custom_mode() for ArduPilot-specific flight mode number
 */
uint8_t GCS_MAVLINK_Copter::base_mode() const
{
    // Start with stabilize flag - all copter modes provide some form of stabilization
    uint8_t _base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
    
    // Work out the base_mode. This value is not very useful for APM, but we calculate
    // it as best we can so a generic MAVLink enabled ground station can work out something
    // about what the MAV is up to. The actual bit values are highly ambiguous for most of
    // the APM flight modes. In practice, you only get useful information from the custom_mode,
    // which maps to the APM flight mode and has a well defined meaning in the ArduCopter documentation
    
    // Set GUIDED flag when position control is active in North-East plane
    if ((copter.pos_control != nullptr) && copter.pos_control->is_active_NE()) {
        _base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
        // Note that MAV_MODE_FLAG_AUTO_ENABLED does not match what APM does in any mode,
        // as that is defined as "system finds its own goal positions", which APM does not currently do
    }

    // All modes except INITIALISING have some form of manual override if stick mixing is enabled
    _base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

    // Set ARMED flag if motors are armed (and motors object is allocated)
    if (copter.motors != nullptr && copter.motors->armed()) {
        _base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // Indicate we have set a custom mode (ArduPilot-specific flight mode number)
    _base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    return _base_mode;
}

/**
 * @brief Get ArduPilot-specific flight mode number for MAVLink reporting
 * 
 * @details Returns the current flight mode as a uint32 value for the custom_mode field
 *          in MAVLink HEARTBEAT message. This provides the precise ArduPilot flight mode
 *          (STABILIZE, ALT_HOLD, LOITER, AUTO, GUIDED, etc.) to ground control stations.
 *          
 *          Mode numbers are defined in Mode::Number enum and documented in ArduCopter
 *          flight mode documentation. Ground stations use this value to display the
 *          current mode name and provide mode-specific UI controls.
 * 
 * @return uint32_t Current flight mode number (cast from Mode::Number enum)
 * 
 * @note This is the primary method for GCS to determine exact flight mode
 * @note Mode numbers are vehicle-specific (Copter vs Plane vs Rover have different mappings)
 * 
 * @see base_mode() for generic MAVLink mode flags
 */
uint32_t GCS_Copter::custom_mode() const
{
    return (uint32_t)copter.flightmode->mode_number();
}

/**
 * @brief Determine vehicle system status for MAVLink HEARTBEAT message
 * 
 * @details Returns MAV_STATE enum representing the current vehicle operational state.
 *          This provides ground stations with high-level status information for display
 *          and alerting purposes.
 *          
 *          State determination priority (checked in this order):
 *          1. MAV_STATE_CRITICAL - Any failsafe triggered (radio, battery, GCS, EKF, etc.)
 *          2. MAV_STATE_STANDBY - Landed and disarmed (land_complete flag set)
 *          3. MAV_STATE_BOOT - System initializing (before initialization complete)
 *          4. MAV_STATE_ACTIVE - Normal operation (armed or ready to arm)
 * 
 * @return MAV_STATE enum value representing current system status
 *         - MAV_STATE_CRITICAL: Failsafe active (radio loss, low battery, EKF failure, etc.)
 *         - MAV_STATE_STANDBY: On ground, disarmed, ready for flight
 *         - MAV_STATE_BOOT: System initialization in progress
 *         - MAV_STATE_ACTIVE: Normal operational state
 * 
 * @note Ground stations typically use this for status indication (colors, alerts)
 * @note CRITICAL state indicates immediate attention required
 * 
 * @see any_failsafe_triggered() for failsafe condition detection
 */
MAV_STATE GCS_MAVLINK_Copter::vehicle_system_status() const
{
    // Set system as CRITICAL if any failsafe have triggered (highest priority status)
    if (copter.any_failsafe_triggered())  {
        return MAV_STATE_CRITICAL;
    }

    // STANDBY state when landed and disarmed
    if (copter.ap.land_complete) {
        return MAV_STATE_STANDBY;
    }

    // BOOT state during initialization sequence
    if (!copter.ap.initialised) {
    	return MAV_STATE_BOOT;
    }

    // ACTIVE state for normal operation (armed or ready to arm)
    return MAV_STATE_ACTIVE;
}


/**
 * @brief Send current attitude controller targets to ground station
 * 
 * @details Streams the ATTITUDE_TARGET MAVLink message containing the attitude controller's
 *          current target quaternion, angular velocity targets, and thrust output. This allows
 *          ground stations and companion computers to monitor what the attitude controller is
 *          commanding, useful for performance analysis and trajectory tracking validation.
 *          
 *          Message contents:
 *          - time_boot_ms: System uptime in milliseconds
 *          - typemask: Set to 0 (no fields ignored) to provide maximum information
 *          - q: Target attitude quaternion in [w, x, y, z] order (unit-length)
 *          - body_roll_rate: Target roll rate in rad/s (body frame)
 *          - body_pitch_rate: Target pitch rate in rad/s (body frame)
 *          - body_yaw_rate: Target yaw rate in rad/s (body frame)
 *          - thrust: Normalized collective thrust (0.0 to 1.0)
 * 
 * @note Typemask is always 0 - we send all available information for maximum visibility
 * @note Quaternion uses [w, x, y, z] ordering per MAVLink specification
 * @note Angular velocities are in body frame, not earth frame
 * @note Called periodically when ATTITUDE_TARGET streaming is enabled
 * 
 * @see handle_message_set_attitude_target() for receiving attitude commands from GCS
 */
void GCS_MAVLINK_Copter::send_attitude_target()
{
    // Get current targets from attitude controller
    const Quaternion quat  = copter.attitude_control->get_attitude_target_quat();
    const Vector3f ang_vel = copter.attitude_control->get_attitude_target_ang_vel();
    const float thrust = copter.attitude_control->get_throttle_in();

    // Convert quaternion to array format required by MAVLink [w, x, y, z] order
    const float quat_out[4] {quat.q1, quat.q2, quat.q3, quat.q4};

    // Note: When sending out the attitude_target info, we send out all of info no matter the mavlink typemask.
    // This way we send out the maximum information that can be used by the receiving control systems
    // to adapt their generated trajectories
    const uint16_t typemask = 0;    // Ignore nothing - send all fields

    mavlink_msg_attitude_target_send(
        chan,
        AP_HAL::millis(),       // time since boot (ms)
        typemask,               // Bitmask that tells the system what control dimensions should be ignored by the vehicle
        quat_out,               // Attitude quaternion [w, x, y, z] order, zero-rotation is [1, 0, 0, 0], unit-length
        ang_vel.x,              // roll rate (rad/s)
        ang_vel.y,              // pitch rate (rad/s)
        ang_vel.z,              // yaw rate (rad/s)
        thrust);                // Collective thrust, normalized to 0 .. 1
}

/**
 * @brief Send current waypoint target position in global coordinates
 * 
 * @details Streams the POSITION_TARGET_GLOBAL_INT MAVLink message containing the vehicle's
 *          current target waypoint in global latitude/longitude/altitude coordinates. This
 *          is primarily used in AUTO, GUIDED, and RTL modes to show the GCS where the
 *          vehicle is navigating toward.
 *          
 *          Message format:
 *          - coordinate_frame: MAV_FRAME_GLOBAL (AMSL altitude reference)
 *          - type_mask: Velocity, acceleration, yaw fields ignored (position-only target)
 *          - lat_int: Target latitude in degrees * 1e7
 *          - lon_int: Target longitude in degrees * 1e7
 *          - alt: Target altitude in meters AMSL (Above Mean Sea Level)
 *          
 *          Position target extraction:
 *          1. Query current flight mode for waypoint target via get_wp()
 *          2. Convert altitude to AMSL frame (may query terrain database)
 *          3. If conversion fails or no target available, do not send message
 * 
 * @note Only sends position fields - velocity/acceleration/yaw are marked as ignored
 * @note Altitude is converted to AMSL regardless of flight mode's internal reference
 * @note Returns early if flight mode has no active waypoint target
 * @note Altitude sent as float in meters, lat/lon sent as int32 in degrees * 1e7
 * 
 * @see send_position_target_local_ned() for NED frame position targets
 */
void GCS_MAVLINK_Copter::send_position_target_global_int()
{
    // Get current waypoint target from active flight mode
    Location target;
    if (!copter.flightmode->get_wp(target)) {
        return;  // No target available - mode doesn't provide waypoint info
    }

    // Convert altitude frame to AMSL (Above Mean Sea Level) for MAV_FRAME_GLOBAL
    // This may use the terrain database if target is terrain-relative
    if (!target.change_alt_frame(Location::AltFrame::ABSOLUTE)) {
        return;  // Altitude conversion failed - cannot send message
    }
    
    // Construct type mask to indicate which fields are valid (position only)
    static constexpr uint16_t POSITION_TARGET_TYPEMASK_LAST_BYTE = 0xF000;
    static constexpr uint16_t TYPE_MASK = POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                                          POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                                          POSITION_TARGET_TYPEMASK_YAW_IGNORE | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE | POSITION_TARGET_TYPEMASK_LAST_BYTE;
    
    mavlink_msg_position_target_global_int_send(
        chan,
        AP_HAL::millis(), // time_boot_ms
        MAV_FRAME_GLOBAL, // targets are always global altitude (AMSL)
        TYPE_MASK, // ignore everything except the x/y/z components
        target.lat, // latitude as 1e7 (degrees * 10^7)
        target.lng, // longitude as 1e7 (degrees * 10^7)
        target.alt * 0.01f, // altitude is sent as a float in meters (convert from cm)
        0.0f, // vx - ignored
        0.0f, // vy - ignored
        0.0f, // vz - ignored
        0.0f, // afx - ignored
        0.0f, // afy - ignored
        0.0f, // afz - ignored
        0.0f, // yaw - ignored
        0.0f); // yaw_rate - ignored
}

/**
 * @brief Send current guided mode targets in local NED coordinates
 * 
 * @details Streams the POSITION_TARGET_LOCAL_NED MAVLink message containing the guided mode's
 *          current position, velocity, and/or acceleration targets in the local NED frame.
 *          The message content varies based on the active guided submode, with type_mask
 *          indicating which fields are valid.
 *          
 *          Guided submodes and their outputs:
 *          - Angle: No local target available (message not sent)
 *          - TakeOff/WP/Pos: Position target only
 *          - PosVelAccel: Position, velocity, and acceleration targets
 *          - VelAccel: Velocity and acceleration targets (no position)
 *          - Accel: Acceleration target only
 *          
 *          Coordinate frame: MAV_FRAME_LOCAL_NED (North-East-Down from EKF origin)
 *          
 *          Unit conversions applied:
 *          - Position: cm (internal) → meters (MAVLink)
 *          - Velocity: cm/s (internal) → m/s (MAVLink)
 *          - Acceleration: cm/s² (internal) → m/s² (MAVLink)
 *          - Z-axis: Up (internal) → Down (NED frame) via negation
 * 
 * @note Only sent when in guided mode (returns early otherwise)
 * @note Type mask indicates which fields are valid for current guided submode
 * @note Z-axis sign is inverted to convert from "up" convention to NED "down" convention
 * @note Yaw and yaw_rate fields are always set to 0 and marked as ignored
 * @note Requires MODE_GUIDED_ENABLED to be compiled in
 * 
 * @see handle_message_set_position_target_local_ned() for receiving position commands
 */
void GCS_MAVLINK_Copter::send_position_target_local_ned()
{
#if MODE_GUIDED_ENABLED
    // Only send this message when in guided mode
    if (!copter.flightmode->in_guided_mode()) {
        return;
    }

    // Determine which targets are available based on guided submode
    const ModeGuided::SubMode guided_mode = copter.mode_guided.submode();
    Vector3f target_pos;
    Vector3f target_vel;
    Vector3f target_accel;
    uint16_t type_mask = 0;

    // Populate target vectors and type mask based on active guided submode
    switch (guided_mode) {
    case ModeGuided::SubMode::Angle:
        // Angle mode has no local position/velocity target - only attitude
        return;
        
    case ModeGuided::SubMode::TakeOff:
    case ModeGuided::SubMode::WP:
    case ModeGuided::SubMode::Pos:
        // Position-only target (velocity and acceleration not commanded)
        type_mask = POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                    POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                    POSITION_TARGET_TYPEMASK_YAW_IGNORE| POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE; // ignore everything except position
        target_pos = copter.mode_guided.get_target_pos().tofloat() * 0.01; // convert cm to metres
        break;
        
    case ModeGuided::SubMode::PosVelAccel:
        // Full trajectory target with position, velocity, and acceleration
        type_mask = POSITION_TARGET_TYPEMASK_YAW_IGNORE| POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE; // ignore everything except position, velocity & acceleration
        target_pos = copter.mode_guided.get_target_pos().tofloat() * 0.01; // convert cm to metres
        target_vel = copter.mode_guided.get_target_vel() * 0.01f; // convert cm/s to metres/s
        target_accel = copter.mode_guided.get_target_accel() * 0.01f; // convert cm/s/s to metres/s/s
        break;
        
    case ModeGuided::SubMode::VelAccel:
        // Velocity and acceleration target (no position setpoint)
        type_mask = POSITION_TARGET_TYPEMASK_X_IGNORE | POSITION_TARGET_TYPEMASK_Y_IGNORE | POSITION_TARGET_TYPEMASK_Z_IGNORE |
                    POSITION_TARGET_TYPEMASK_YAW_IGNORE| POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE; // ignore everything except velocity & acceleration
        target_vel = copter.mode_guided.get_target_vel() * 0.01f; // convert cm/s to metres/s
        target_accel = copter.mode_guided.get_target_accel() * 0.01f; // convert cm/s/s to metres/s/s
        break;
        
    case ModeGuided::SubMode::Accel:
        // Acceleration-only target (position and velocity not commanded)
        type_mask = POSITION_TARGET_TYPEMASK_X_IGNORE | POSITION_TARGET_TYPEMASK_Y_IGNORE | POSITION_TARGET_TYPEMASK_Z_IGNORE |
                    POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                    POSITION_TARGET_TYPEMASK_YAW_IGNORE| POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE; // ignore everything except acceleration
        target_accel = copter.mode_guided.get_target_accel() * 0.01f; // convert cm/s/s to metres/s/s
        break;
    }

    mavlink_msg_position_target_local_ned_send(
        chan,
        AP_HAL::millis(), // time boot ms
        MAV_FRAME_LOCAL_NED, 
        type_mask,
        target_pos.x,   // x in metres (North)
        target_pos.y,   // y in metres (East)
        -target_pos.z,  // z in metres (Down - NED frame requires negation)
        target_vel.x,   // vx in m/s (North)
        target_vel.y,   // vy in m/s (East)
        -target_vel.z,  // vz in m/s (Down - NED frame requires negation)
        target_accel.x, // afx in m/s/s (North)
        target_accel.y, // afy in m/s/s (East)
        -target_accel.z,// afz in m/s/s (Down - NED frame requires negation)
        0.0f, // yaw - not used
        0.0f); // yaw_rate - not used
#endif
}

/**
 * @brief Send navigation controller status and target information to GCS
 * 
 * @details Streams the NAV_CONTROLLER_OUTPUT MAVLink message containing navigation controller
 *          state, target attitude, waypoint information, and position errors. This message
 *          allows ground stations to display navigation performance and debug controller behavior.
 *          
 *          Message contents:
 *          - nav_roll: Target roll angle in degrees (from attitude controller)
 *          - nav_pitch: Target pitch angle in degrees (from attitude controller)
 *          - nav_bearing: Target yaw/heading in degrees (from attitude controller)
 *          - target_bearing: Bearing to target waypoint in degrees (from flight mode)
 *          - wp_dist: Distance to target waypoint in meters (from flight mode)
 *          - alt_error: Altitude error in meters (from position controller)
 *          - aspd_error: Airspeed error (always 0 for copters - field not used)
 *          - xtrack_error: Cross-track error in meters (from flight mode)
 *          
 *          Unit conversions:
 *          - Attitude targets: radians → degrees
 *          - Altitude error: cm → meters
 *          - Cross-track error: cm → meters
 * 
 * @note Returns early if vehicle not yet initialized
 * @note wp_dist is clamped to UINT16_MAX (65535 meters) for message encoding
 * @note Airspeed error always reported as 0 (copters don't have airspeed setpoints)
 * @note Cross-track error is only meaningful in AUTO/GUIDED modes with waypoint navigation
 * 
 * @see send_position_target_local_ned() for detailed guided mode targets
 */
void GCS_MAVLINK_Copter::send_nav_controller_output() const
{
    // Wait until initialization complete before sending navigation data
    if (!copter.ap.initialised) {
        return;
    }
    
    // Get target attitude from attitude controller (in radians)
    const Vector3f &targets_rad = copter.attitude_control->get_att_target_euler_rad();
    const Mode *flightmode = copter.flightmode;
    
    mavlink_msg_nav_controller_output_send(
        chan,
        degrees(targets_rad.x),  // nav_roll: target roll in degrees
        degrees(targets_rad.y),  // nav_pitch: target pitch in degrees
        degrees(targets_rad.z),  // nav_bearing: target yaw in degrees
        flightmode->wp_bearing_deg(),  // target_bearing: bearing to waypoint in degrees
        MIN(flightmode->wp_distance_m(), UINT16_MAX),  // wp_dist: distance to waypoint (clamped to uint16 range)
        copter.pos_control->get_pos_error_U_cm() * 1.0e-2f,  // alt_error: altitude error in meters (up axis)
        0,  // aspd_error: airspeed error - not applicable for copters
        flightmode->crosstrack_error() * 1.0e-2f);  // xtrack_error: cross-track error in meters
}

/**
 * @brief Calculate airspeed value for VFR_HUD message
 * 
 * @details Determines the best available airspeed estimate for display in ground station HUD.
 *          Prioritizes sources in order of accuracy:
 *          1. Physical airspeed sensor (if enabled and healthy)
 *          2. EKF3 synthetic airspeed from wind estimation
 *          3. GPS ground speed (fallback when no wind estimate available)
 *          
 *          For copters without airspeed sensors, EKF3 can estimate airspeed by comparing
 *          GPS velocity with IMU-based velocity to infer wind velocity, then calculate
 *          true airspeed in body frame.
 * 
 * @return float Airspeed in m/s (best available estimate)
 * 
 * @note Physical airspeed sensors provide most accurate reading but uncommon on copters
 * @note EKF3 airspeed estimate requires wind estimation to be running
 * @note GPS ground speed used as fallback - equals airspeed only in zero wind
 * @note Requires AP_AIRSPEED_ENABLED for physical sensor support
 * 
 * @see vfr_hud_throttle() for complementary throttle percentage
 */
float GCS_MAVLINK_Copter::vfr_hud_airspeed() const
{
#if AP_AIRSPEED_ENABLED
    // Airspeed sensors provide the most accurate reading when available
    // While AHRS airspeed_estimate may use a sensor, that value is constrained
    // by ground speed. For reporting, send true unconstrained sensor value.
    if (copter.airspeed.enabled() && copter.airspeed.healthy()) {
        return copter.airspeed.get_airspeed();
    }
#endif
    
    // Try to get EKF3 synthetic airspeed estimate from wind estimation
    Vector3f airspeed_vec_bf;
    if (AP::ahrs().airspeed_vector_true(airspeed_vec_bf)) {
        // EKF3 wind estimation is running and can provide airspeed estimate
        // by comparing GPS velocity to IMU-based velocity to infer wind
        return airspeed_vec_bf.length();
    }
    
    // Fallback: return GPS ground speed (equals airspeed only in zero wind)
    return AP::gps().ground_speed();
}

/**
 * @brief Calculate throttle percentage for VFR_HUD message
 * 
 * @details Returns current motor throttle output as a percentage (0-100) for display
 *          in ground station HUD. The throttle value represents the collective thrust
 *          commanded to the motors, normalized to percentage.
 *          
 *          For multirotors, this is the collective throttle that would be applied
 *          uniformly to all motors before mixing in roll/pitch/yaw corrections.
 * 
 * @return int16_t Throttle percentage (0-100), or 0 if motors not initialized
 * 
 * @note Returns 0 if motor library not yet allocated (early in boot)
 * @note Value scaled from normalized throttle (0.0-1.0) to percentage (0-100)
 * @note This is commanded throttle, not actual motor output after mixing
 * 
 * @see vfr_hud_airspeed() for complementary airspeed value
 * @see send_attitude_target() for detailed attitude and thrust targets
 */
int16_t GCS_MAVLINK_Copter::vfr_hud_throttle() const
{
    // Return 0 if motors not yet initialized (early boot sequence)
    if (copter.motors == nullptr) {
        return 0;
    }
    
    // Get normalized throttle (0.0-1.0) and convert to percentage (0-100)
    return (int16_t)(copter.motors->get_throttle() * 100);
}

/**
 * @brief Send PID controller tuning data to GCS for real-time monitoring
 * 
 * @details Streams PID_TUNING MAVLink messages for enabled controller axes, allowing
 *          ground stations to display real-time PID performance for tuning and debugging.
 *          Each message contains target value, actual value, and individual P/I/D/FF terms.
 *          
 *          Supported tuning axes:
 *          - PID_TUNING_ROLL: Roll rate controller (attitude_control)
 *          - PID_TUNING_PITCH: Pitch rate controller (attitude_control)
 *          - PID_TUNING_YAW: Yaw rate controller (attitude_control)
 *          - PID_TUNING_ACCZ: Vertical acceleration controller (pos_control)
 *          
 *          Message fields:
 *          - axis: Which controller axis (1=roll, 2=pitch, 3=yaw, 4=accZ)
 *          - target: Desired value (rate in rad/s for attitude, accel in cm/s/s for alt)
 *          - actual: Measured value (rate in rad/s or accel in cm/s/s)
 *          - FF: Feed-forward term contribution
 *          - P: Proportional term contribution
 *          - I: Integral term contribution
 *          - D: Derivative term contribution
 *          - slew_rate: Rate of change limit applied
 *          - Dmod: Derivative modification factor
 *          
 *          Streaming controlled by GCS_PID_MASK parameter bitmask:
 *          - Bit 0: Roll rate PID
 *          - Bit 1: Pitch rate PID
 *          - Bit 2: Yaw rate PID
 *          - Bit 3: Altitude acceleration PID
 * 
 * @note Only sends data for axes enabled in copter.g.gcs_pid_mask parameter
 * @note Returns early if insufficient bandwidth available for message
 * @note Primarily used during tuning to visualize PID response in real-time
 * @note High streaming rate can consume significant bandwidth
 * 
 * @warning Enabling multiple axes simultaneously can saturate telemetry link
 * 
 * @see AC_AttitudeControl for rate controller implementation
 * @see AC_PosControl for altitude controller implementation
 */
void GCS_MAVLINK_Copter::send_pid_tuning()
{
    // Define all available PID tuning axes
    static const PID_TUNING_AXIS axes[] = {
        PID_TUNING_ROLL,   // Roll rate controller
        PID_TUNING_PITCH,  // Pitch rate controller
        PID_TUNING_YAW,    // Yaw rate controller
        PID_TUNING_ACCZ    // Vertical acceleration controller
    };
    
    // Iterate through each axis and send if enabled in parameter mask
    for (uint8_t i=0; i<ARRAY_SIZE(axes); i++) {
        // Check if this axis is enabled in GCS_PID_MASK parameter
        // Bitmask uses (axis-1) because axis enums start at 1
        if (!(copter.g.gcs_pid_mask & (1<<(axes[i]-1)))) {
            continue;  // This axis not enabled for streaming
        }
        
        // Check if enough bandwidth available for PID_TUNING message
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;  // Insufficient space, try again next opportunity
        }
        
        // Get PID info structure for the requested axis
        const AP_PIDInfo *pid_info = nullptr;
        switch (axes[i]) {
        case PID_TUNING_ROLL:
            // Roll rate controller from attitude control
            pid_info = &copter.attitude_control->get_rate_roll_pid().get_pid_info();
            break;
        case PID_TUNING_PITCH:
            // Pitch rate controller from attitude control
            pid_info = &copter.attitude_control->get_rate_pitch_pid().get_pid_info();
            break;
        case PID_TUNING_YAW:
            // Yaw rate controller from attitude control
            pid_info = &copter.attitude_control->get_rate_yaw_pid().get_pid_info();
            break;
        case PID_TUNING_ACCZ:
            // Vertical acceleration controller from position control
            pid_info = &copter.pos_control->get_accel_U_pid().get_pid_info();
            break;
        default:
            continue;  // Unknown axis
        }
        
        // Send PID tuning message if valid PID info obtained
        if (pid_info != nullptr) {
            mavlink_msg_pid_tuning_send(chan,
                                        axes[i],              // Axis identifier (1-4)
                                        pid_info->target,     // Desired/target value
                                        pid_info->actual,     // Measured/actual value
                                        pid_info->FF,         // Feed-forward term
                                        pid_info->P,          // Proportional term
                                        pid_info->I,          // Integral term
                                        pid_info->D,          // Derivative term
                                        pid_info->slew_rate,  // Slew rate limit
                                        pid_info->Dmod);      // D-term modification
        }
    }
}

#if AP_WINCH_ENABLED
/**
 * @brief Send winch status telemetry to GCS
 * 
 * @details Transmits WINCH_STATUS MAVLink message containing current winch state,
 *          including line length, speed, tension, and operational status. This enables
 *          ground station monitoring of payload delivery or recovery operations.
 *          
 *          Winch status includes:
 *          - Line length extended (meters)
 *          - Line speed (m/s, positive=deploying, negative=retracting)
 *          - Line tension (kg-force if sensor available)
 *          - Clutch engaged state
 *          - Health and error flags
 * 
 * @note Only available when AP_WINCH_ENABLED is defined
 * @note Returns immediately if winch subsystem not initialized
 * @note Actual message formatting handled by AP_Winch library
 * 
 * @see handle_MAV_CMD_DO_WINCH() for winch control commands
 * @see AP_Winch for winch implementation details
 */
void GCS_MAVLINK_Copter::send_winch_status() const
{
    // Get singleton winch instance
    AP_Winch *winch = AP::winch();
    if (winch == nullptr) {
        return;  // Winch not initialized or not available
    }
    
    // Delegate to winch library to send status message
    winch->send_status(*this);
}
#endif

/**
 * @brief Check if vehicle initialization is complete
 * 
 * @details Returns whether the copter has completed its boot-up initialization sequence
 *          and is ready for normal operation. Used by GCS_MAVLink base class to determine
 *          when to enable certain MAVLink features and telemetry streaming.
 *          
 *          Initialization includes:
 *          - Hardware detection and configuration
 *          - Sensor calibration loading
 *          - Parameter loading from storage
 *          - Motor library allocation and setup
 *          - Flight mode initialization
 *          - Position and attitude controller setup
 * 
 * @return bool true if vehicle initialization complete, false during boot sequence
 * 
 * @note Used to gate certain GCS operations that require full vehicle initialization
 * @note Checked before processing some MAVLink commands and requests
 * @note Copter sets ap.initialised flag after completing setup() in main loop
 * 
 * @see params_ready() for parameter system initialization status
 * @see vehicle_system_status() for overall vehicle state reporting
 */
bool GCS_Copter::vehicle_initialised() const {
    return copter.ap.initialised;
}

/**
 * @brief Attempt to send a specific MAVLink message type to GCS
 * 
 * @details Called by telemetry scheduler to transmit queued messages when bandwidth available.
 *          Routes copter-specific message types to appropriate handlers, deferring to base
 *          class for common messages. Checks payload space before sending to avoid buffer overflow.
 *          
 *          Copter-specific messages handled:
 *          - MSG_TERRAIN_REQUEST: Request terrain altitude data for terrain following
 *          - MSG_TERRAIN_REPORT: Report current terrain altitude and status
 *          - MSG_WIND: Wind velocity estimate from airspeed/EKF (copter-specific calculation)
 *          - MSG_ADSB_VEHICLE: Traffic information from ADSB receiver and object avoidance database
 *          
 *          Message routing:
 *          1. Check if message is copter-specific type
 *          2. Verify sufficient bandwidth with CHECK_PAYLOAD_SIZE macro
 *          3. Call appropriate subsystem to format and send message
 *          4. Return to base class for non-copter messages
 * 
 * @param[in] id Message type enum from ap_message enumeration
 * 
 * @return bool true if message sent successfully or handled, false if not sent
 * 
 * @note CHECK_PAYLOAD_SIZE returns false if insufficient space, preventing transmission
 * @note Terrain messages only available when AP_TERRAIN_AVAILABLE defined
 * @note ADSB messages require HAL_ADSB_ENABLED or AP_OAPATHPLANNER_ENABLED
 * @note Called repeatedly by scheduler until message successfully transmitted
 * @note Does not block - returns immediately if bandwidth unavailable
 * 
 * @see GCS_MAVLINK::try_send_message() for base class message handling
 * @see send_wind() for wind estimation specific to copters
 */
bool GCS_MAVLINK_Copter::try_send_message(enum ap_message id)
{
    switch(id) {

#if AP_TERRAIN_AVAILABLE
    case MSG_TERRAIN_REQUEST:
        // Check bandwidth available for terrain data request
        CHECK_PAYLOAD_SIZE(TERRAIN_REQUEST);
        copter.terrain.send_request(chan);
        break;
    case MSG_TERRAIN_REPORT:
        // Send current terrain altitude and status
        CHECK_PAYLOAD_SIZE(TERRAIN_REPORT);
        copter.terrain.send_report(chan);
        break;
#endif

    case MSG_WIND:
        // Send wind velocity estimate (requires valid airspeed vector on copter)
        CHECK_PAYLOAD_SIZE(WIND);
        send_wind();
        break;

    case MSG_ADSB_VEHICLE: {
#if HAL_ADSB_ENABLED
        // Send traffic information from ADSB receiver
        CHECK_PAYLOAD_SIZE(ADSB_VEHICLE);
        copter.adsb.send_adsb_vehicle(chan);
#endif
#if AP_OAPATHPLANNER_ENABLED
        // Send object avoidance database entries as pseudo-ADSB targets
        AP_OADatabase *oadb = AP_OADatabase::get_singleton();
        if (oadb != nullptr) {
            CHECK_PAYLOAD_SIZE(ADSB_VEHICLE);
            uint16_t interval_ms = 0;
            if (get_ap_message_interval(id, interval_ms)) {
                oadb->send_adsb_vehicle(chan, interval_ms);
            }
        }
#endif
        break;
    }

    default:
        // Not a copter-specific message, defer to base class
        return GCS_MAVLINK::try_send_message(id);
    }
    return true;
}


/**
 * @brief Get current mission execution state for telemetry reporting
 * 
 * @details Returns the operational state of the mission for MISSION_CURRENT message,
 *          allowing GCS to display mission progress and paused status. Copter-specific
 *          override adds AUTO mode pause detection beyond base class states.
 *          
 *          Mission states returned:
 *          - MISSION_STATE_PAUSED: Auto mode explicitly paused (user requested hold)
 *          - MISSION_STATE_ACTIVE: Mission executing normally (from base class)
 *          - MISSION_STATE_COMPLETE: Mission finished (from base class)
 *          - MISSION_STATE_NO_MISSION: No mission loaded (from base class)
 *          
 *          Pause detection:
 *          Checks if AUTO mode is in paused state, typically triggered by:
 *          - MAV_CMD_DO_PAUSE_CONTINUE command with param1=0
 *          - Loss of GPS in AUTO mode requiring pause
 *          - Mission safeguards requiring temporary hold
 * 
 * @param[in] mission Reference to mission object being queried
 * 
 * @return MISSION_STATE Current state of mission execution
 * 
 * @note Only checks AUTO mode pause state, other states handled by base class
 * @note Pause state takes precedence over other base class states
 * @note Used in MISSION_CURRENT message to report mission progress
 * 
 * @see handle_command_pause_continue() for mission pause/resume commands
 * @see ModeAuto::paused() for AUTO mode pause detection
 */
MISSION_STATE GCS_MAVLINK_Copter::mission_state(const class AP_Mission &mission) const
{
    // Check if AUTO mode has been explicitly paused
    if (copter.mode_auto.paused()) {
        return MISSION_STATE_PAUSED;
    }
    
    // Use base class for all other mission states
    return GCS_MAVLINK::mission_state(mission);
}

/**
 * @brief Handle guided mode target request from mission command
 * 
 * @details Processes mission commands that request vehicle to enter guided mode with
 *          specific target. Called by base class when mission contains a guided mode
 *          command (MAV_CMD_DO_GUIDED_ENABLE with guided commands). Allows mission
 *          to temporarily hand control to external guidance commands while in AUTO mode.
 *          
 *          Supported guided mission commands:
 *          - MAV_CMD_NAV_GUIDED_ENABLE: Enable/disable guided mode during mission
 *          - Position/velocity targets in guided submode
 *          - Attitude targets in guided submode
 *          
 *          Processing:
 *          1. Receives mission command structure from mission executor
 *          2. Delegates to AUTO mode's guided handler (do_guided)
 *          3. AUTO mode transitions to guided submode if command valid
 *          4. Returns success/failure to mission state machine
 * 
 * @param[in,out] cmd Mission command structure containing guided target parameters
 * 
 * @return bool true if guided request accepted and applied, false if rejected
 * 
 * @note Only available when MODE_AUTO_ENABLED is defined
 * @note Returns false if AUTO mode not compiled into firmware
 * @note Guided targets set during mission execute until next mission command
 * @note Vehicle remains in AUTO mode, just with guided control active
 * 
 * @see ModeAuto::do_guided() for guided mission command implementation
 * @see MAV_CMD_DO_GUIDED_ENABLE in MAVLink protocol specification
 */
bool GCS_MAVLINK_Copter::handle_guided_request(AP_Mission::Mission_Command &cmd)
{
#if MODE_AUTO_ENABLED
    // Delegate to AUTO mode to handle guided mission command
    return copter.mode_auto.do_guided(cmd);
#else
    // AUTO mode not available, cannot handle guided requests
    return false;
#endif
}

/**
 * @brief Pre-process received MAVLink packet before normal routing
 * 
 * @details Called by MAVLink parser immediately after complete packet received and validated.
 *          Allows copter-specific early processing of messages before they reach normal
 *          routing and handling pipeline. Used for time-critical or special-case message
 *          interception that must bypass routing delays.
 *          
 *          Current early processing:
 *          - ADSB/Avoidance: GLOBAL_POSITION_INT messages can be treated as avoidance targets
 *            when DevOptionADSBMAVLink flag set, enabling MAVLink-based traffic avoidance
 *            from cooperative vehicles broadcasting position instead of ADSB transponders
 *          
 *          Processing flow:
 *          1. Packet received and CRC validated by parser
 *          2. This function called before routing decision
 *          3. Copter-specific interception performed if needed
 *          4. Base class packetReceived() handles normal routing and processing
 *          
 *          ADSB MAVLink mode:
 *          - Treats other vehicles' GLOBAL_POSITION_INT as traffic for avoidance
 *          - Enables cooperative avoidance without physical ADSB hardware
 *          - Useful for multi-vehicle operations and swarms
 *          - Controlled by DevOptionADSBMAVLink development option flag
 * 
 * @param[in] status MAVLink channel status (sequence numbers, packet drops)
 * @param[in] msg Complete decoded MAVLink message with msgid and payload
 * 
 * @note Called for every received packet on this MAVLink channel
 * @note Does not replace normal message handling, just pre-processes certain types
 * @note ADSB avoidance interception only active when explicitly enabled via dev_options
 * @note Base class handles routing, ACKs, and standard message dispatch
 * 
 * @see handle_message() for normal message processing after routing
 * @see AP_ADSB::handle_msg() for ADSB avoidance message processing
 */
void GCS_MAVLINK_Copter::packetReceived(const mavlink_status_t &status,
                                        const mavlink_message_t &msg)
{
    // Handle messages that need early interception before routing
    
#if AP_ADSB_AVOIDANCE_ENABLED
    // Optional MAVLink-based avoidance: treat GLOBAL_POSITION_INT as traffic
    if (copter.g2.dev_options.get() & DevOptionADSBMAVLink) {
        // Feed position messages to ADSB avoidance system as pseudo-traffic
        // Enables cooperative avoidance from other MAVLink-equipped vehicles
        copter.avoidance_adsb.handle_msg(msg);
    }
#endif
    
    // Continue with normal packet routing and handling
    GCS_MAVLINK::packetReceived(status, msg);
}

/**
 * @brief Check if parameters are ready for GCS access
 * 
 * @details Determines whether parameter system is sufficiently initialized to handle
 *          GCS parameter requests (PARAM_REQUEST_LIST, PARAM_REQUEST_READ, PARAM_SET).
 *          Prevents GCS from seeing incomplete or inconsistent parameter list during
 *          bootup when object allocation and initialization still in progress.
 *          
 *          Returns true when safe to expose parameters:
 *          - Board configuration error state: Parameters accessible even if incomplete
 *            (allows recovery by changing parameters that caused the error)
 *          - Normal operation: Only after copter.ap.initialised_params flag set
 *            (indicates motors allocated, all subsystems initialized, complete param list)
 *          
 *          Returns false during bootup:
 *          - Motors object not yet allocated (motors == nullptr)
 *          - Subsystems still initializing
 *          - Parameter groups still being registered
 *          
 *          Why blocking matters:
 *          - Parameter count changes as objects allocated (motors, sensors, etc.)
 *          - GCS caches parameter list during PARAM_REQUEST_LIST
 *          - Changing count after caching causes GCS confusion and sync errors
 *          - Some parameters depend on hardware detection (board type, sensor presence)
 *          
 *          Recovery mode:
 *          - Config errors bypass check to allow parameter-based recovery
 *          - User can modify parameters that caused error condition
 *          - Enables fixing misconfiguration without serial console access
 * 
 * @return bool true if GCS can safely access parameters, false to block requests
 * 
 * @note Returning false causes PARAM_REQUEST_* to be NAKed until initialization complete
 * @note Typical initialization takes 2-5 seconds from power-on
 * @note Ground stations may retry param requests during this blocking period
 * @note Motors allocation is key milestone - many params depend on motor object
 * 
 * @see AP_Param for parameter storage and registration system
 * @see Copter::init_ardupilot() for initialization sequence
 */
bool GCS_MAVLINK_Copter::params_ready() const
{
    if (AP_BoardConfig::in_config_error()) {
        // Board configuration error - allow parameter access for recovery
        // User needs to access parameters to fix misconfiguration
        return true;
    }
    
    // Block parameter access during bootup initialization
    // Prevents GCS from seeing incomplete parameter list and getting confused
    // when parameter count changes as objects are allocated
    return copter.ap.initialised_params;
}

/**
 * @brief Send vehicle identification banner to GCS on connection
 * 
 * @details Called when GCS first connects to provide human-readable vehicle identification.
 *          Sends text messages with firmware version, board type, and copter-specific
 *          configuration (frame type and motor configuration). Helps operators verify
 *          correct vehicle connection and configuration before flight.
 *          
 *          Banner information transmitted:
 *          - Base information from parent class:
 *            * ArduPilot version and git hash
 *            * Board type (e.g., "CubeOrange", "Pixhawk6C")
 *            * Build timestamp
 *          - Copter-specific information:
 *            * Frame type (e.g., "Quad", "Hexa", "Y6", "Heli")
 *            * Frame class (X, Plus, H, V for multicopters)
 *            * Motor configuration details
 *          
 *          Error handling:
 *          - If motors not allocated: Reports error instead of frame info
 *            (indicates initialization problem or early connection)
 *          - Continues even if motor info unavailable
 *          
 *          Example output:
 *          - "ArduCopter V4.5.0 (abcd1234)"
 *          - "CubeOrange"
 *          - "Quad X"
 * 
 * @note Called automatically by base class on initial GCS connection
 * @note Banner sent as STATUSTEXT messages with MAV_SEVERITY_INFO
 * @note Multiple message packets sent - one per line of banner text
 * @note Motor allocation failure message helps diagnose boot problems
 * 
 * @see GCS_MAVLINK::send_banner() for base firmware and board information
 * @see AP_Motors::get_frame_and_type_string() for frame description formatting
 */
void GCS_MAVLINK_Copter::send_banner()
{
    // Send base banner information (version, board type)
    GCS_MAVLINK::send_banner();
    
    // Add copter-specific frame and motor configuration info
    if (copter.motors == nullptr) {
        // Motors not yet allocated - likely early in boot sequence
        send_text(MAV_SEVERITY_INFO, "motors not allocated");
        return;
    }
    
    // Get human-readable frame type and configuration string
    char frame_and_type_string[30];
    copter.motors->get_frame_and_type_string(frame_and_type_string, ARRAY_SIZE(frame_and_type_string));
    send_text(MAV_SEVERITY_INFO, "%s", frame_and_type_string);
}

/**
 * @brief Handle received COMMAND_ACK message from GCS
 * 
 * @details Processes COMMAND_ACK messages received from ground control station in response
 *          to commands sent by vehicle. Updates vehicle statistics and delegates to base
 *          class for standard ACK processing. Used to track command completion and detect
 *          communication issues.
 *          
 *          COMMAND_ACK protocol flow:
 *          1. Vehicle sends COMMAND_INT or COMMAND_LONG to GCS
 *          2. GCS processes command and returns COMMAND_ACK with result
 *          3. This handler receives ACK message
 *          4. Increments ACK counter for statistics/debugging
 *          5. Base class processes ACK (matches to pending command, updates state)
 *          
 *          Copter-specific tracking:
 *          - Maintains command_ack_counter for debugging communication issues
 *          - Counter helps diagnose GCS responsiveness and command processing
 *          - Useful for detecting lost ACKs or GCS hangs
 *          
 *          ACK message contains:
 *          - command: Original command ID that was acknowledged
 *          - result: MAV_RESULT (ACCEPTED, FAILED, DENIED, etc.)
 *          - progress: Optional progress indicator for long-running commands
 *          - result_param2: Additional result-specific data
 * 
 * @param[in] msg Complete decoded COMMAND_ACK MAVLink message
 * 
 * @note Vehicle rarely sends commands to GCS, so ACKs are uncommon
 * @note Most command flow is GCS→Vehicle, not Vehicle→GCS
 * @note Counter useful for developer debugging, not typically exposed to users
 * @note Base class matches ACKs to pending outgoing command queue
 * 
 * @see GCS_MAVLINK::handle_command_ack() for base ACK processing
 * @see MAVLINK_MSG_ID_COMMAND_ACK in MAVLink protocol specification
 */
void GCS_MAVLINK_Copter::handle_command_ack(const mavlink_message_t &msg)
{
    // Increment copter-specific ACK counter for statistics
    copter.command_ack_counter++;
    
    // Delegate to base class for standard ACK processing
    GCS_MAVLINK::handle_command_ack(msg);
}

/**
 * @brief Handle LANDING_TARGET message for precision landing
 * 
 * @details Processes LANDING_TARGET MAVLink messages containing visual or IR beacon
 *          position measurements for precision landing system. Message provides angular
 *          position of landing target relative to vehicle, enabling automated landing
 *          on visually-marked landing pads with sub-meter accuracy.
 *          
 *          Landing target detection methods:
 *          - Onboard camera with vision processing (e.g., IR-LOCK sensor)
 *          - External vision system sending position via MAVLink
 *          - Companion computer performing target detection
 *          - Ground-based camera tracking vehicle and reversing perspective
 *          
 *          Message contents:
 *          - target_num: Which landing target (multiple targets supported)
 *          - frame: Coordinate frame (body, global, etc.)
 *          - angle_x, angle_y: Target angular position in radians
 *          - distance: Range to target in meters (if available)
 *          - size_x, size_y: Target size in radians (for distance estimation)
 *          
 *          Timestamp handling:
 *          - timestamp_ms parameter has been jitter-corrected by caller
 *          - Jitter correction accounts for network delays and processing time
 *          - Accurate timestamps critical for Kalman filter fusion
 *          - Precision landing uses EKF to fuse target measurements with IMU
 *          
 *          Precision landing operation:
 *          1. Target detected and angular position measured
 *          2. LANDING_TARGET message sent with measurement
 *          3. This handler receives and forwards to precision landing subsystem
 *          4. Precision landing EKF fuses measurement with vehicle state
 *          5. Landing controller uses filtered target position
 *          6. Vehicle autonomously corrects position to center on target
 * 
 * @param[in] packet Decoded LANDING_TARGET message with target measurements
 * @param[in] timestamp_ms Jitter-corrected timestamp when target was detected (milliseconds)
 * 
 * @note Only functional when AC_PRECLAND_ENABLED compiled into firmware
 * @note Requires precision landing hardware or companion computer
 * @note Timestamp jitter correction performed by base class before calling
 * @note Multiple landing targets supported via target_num field
 * @note Angular measurements more reliable than absolute position estimates
 * 
 * @warning Inaccurate timestamps degrade precision landing performance
 * @warning Message rate must be sufficient for control loop (typically 10+ Hz)
 * 
 * @see AC_PrecLand::handle_msg() for precision landing target processing
 * @see MAVLINK_MSG_ID_LANDING_TARGET in MAVLink protocol specification
 */
void GCS_MAVLINK_Copter::handle_landing_target(const mavlink_landing_target_t &packet, uint32_t timestamp_ms)
{
#if AC_PRECLAND_ENABLED
    // Forward landing target measurement to precision landing subsystem
    // Timestamp has already been jitter-corrected for accurate fusion
    copter.precland.handle_msg(packet, timestamp_ms);
#endif
}

/**
 * @brief Handle preflight calibration commands
 * 
 * @details Processes MAV_CMD_PREFLIGHT_CALIBRATION command for sensor calibration
 *          before flight. Handles copter-specific compassmot calibration in addition
 *          to standard calibrations (gyro, accel, compass, pressure, RC, etc.).
 *          
 *          Command parameters (MAV_CMD_PREFLIGHT_CALIBRATION):
 *          - param1: Gyro calibration (1=calibrate, 0=skip)
 *          - param2: Magnetometer calibration (1=calibrate, 0=skip)
 *          - param3: Ground pressure calibration (1=calibrate, 0=skip)
 *          - param4: Radio calibration (1=calibrate, 0=skip)
 *          - param5: Accelerometer calibration (1=calibrate, 2=level, 4=simple)
 *          - param6: Airspeed calibration (copter: not used)
 *          - param7: ESC calibration (1=calibrate, 2=listen to ESC, 3=passthrough)
 *          - y (param2): Compassmot calibration trigger (copter-specific extension)
 *          
 *          Compassmot calibration:
 *          - Triggered when packet.y == 1
 *          - Measures compass interference from motor magnetic fields
 *          - User spools up motors at various throttle levels
 *          - System correlates compass errors with throttle/current
 *          - Generates compensation parameters (MOT_COMPASS_xx)
 *          - Critical for accurate heading when motors running
 *          - Should be performed after compass offset calibration
 *          - Requires vehicle armed and props off for safety
 *          
 *          Standard calibrations (delegated to base class):
 *          - Gyro: Measures and removes gyro bias (vehicle must be still)
 *          - Accelerometer: 6-point or simple calibration for level reference
 *          - Barometer: Sets ground pressure reference for altitude
 *          - Compass: Offsets and scale factors (vehicle rotation required)
 *          - RC: Learns min/max/center for all channels
 *          
 *          Safety checks:
 *          - Most calibrations require vehicle disarmed
 *          - Compassmot requires armed but landed
 *          - Vehicle must be stable during calibration
 *          - Failed calibration returns MAV_RESULT_FAILED
 * 
 * @param[in] packet COMMAND_INT packet with calibration parameters
 * @param[in] msg Complete MAVLink message (for routing info if needed)
 * 
 * @return MAV_RESULT Calibration command result
 *         - MAV_RESULT_ACCEPTED: Calibration started or completed successfully
 *         - MAV_RESULT_FAILED: Calibration failed (see STATUSTEXT for reason)
 *         - MAV_RESULT_DENIED: Preconditions not met (armed when shouldn't be, etc.)
 *         - MAV_RESULT_TEMPORARILY_REJECTED: Try again (system busy)
 * 
 * @note Compassmot takes 30-60 seconds and requires user interaction
 * @note Standard calibrations may take several seconds to complete
 * @note Progress and instructions sent as STATUSTEXT messages
 * @note Calibration results stored to parameters automatically
 * 
 * @warning Do not calibrate gyros or accels with vehicle in motion
 * @warning Compassmot requires props removed for safety during motor spin-up
 * @warning Some calibrations overwrite existing calibration data
 * 
 * @see Copter::mavlink_compassmot() for compassmot procedure implementation
 * @see GCS_MAVLINK::_handle_command_preflight_calibration() for standard calibrations
 */
MAV_RESULT GCS_MAVLINK_Copter::_handle_command_preflight_calibration(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    // Check for copter-specific compassmot calibration request
    if (packet.y == 1) {
        // Compassmot: calibrate motor magnetic interference compensation
        return copter.mavlink_compassmot(*this);
    }

    // Delegate standard calibrations to base class
    // (gyro, accel, compass, baro, RC, ESC)
    return GCS_MAVLINK::_handle_command_preflight_calibration(packet, msg);
}


/**
 * @brief Set Region of Interest (ROI) for automatic yaw control
 * 
 * @details Configures vehicle to automatically yaw towards a geographic location.
 *          Vehicle continuously points nose (or camera gimbal) at the specified ROI
 *          while flying missions or during guided mode operation. Commonly used for:
 *          - Keeping camera pointed at ground target during orbit
 *          - Pointing at structure during inspection flights
 *          - Tracking moving ground vehicles or boats
 *          - Orienting towards landing zone during approach
 *          
 *          ROI behavior:
 *          - Vehicle calculates bearing from current position to ROI
 *          - Auto-yaw continuously updates to track ROI as vehicle moves
 *          - Works in AUTO, GUIDED, and other modes supporting auto-yaw
 *          - ROI can be cleared by setting to (0,0,0) location
 *          - If gimbal present, may point gimbal instead of yawing vehicle
 *          
 *          Coordinate frame:
 *          - roi_loc is absolute WGS84 lat/lon position
 *          - Altitude component may be used for 3D tracking
 *          - Location must pass validity checks (valid lat/lon range)
 *          
 *          Auto-yaw modes (overridden by ROI):
 *          - AUTO mode: Default yaw matches flight path direction
 *          - GUIDED mode: Default yaw set by position target messages
 *          - Setting ROI overrides default auto-yaw behavior
 *          - Manual yaw stick input may override ROI depending on mode
 *          
 *          Typical mission usage:
 *          1. Takeoff and navigate to orbit position
 *          2. Set ROI to structure/target location
 *          3. Execute orbit or inspection pattern
 *          4. Vehicle maintains pointing throughout maneuver
 *          5. Clear ROI before final approach/landing
 * 
 * @param[in] roi_loc Geographic location (WGS84 lat/lon/alt) to point towards
 * 
 * @return MAV_RESULT Command execution result
 *         - MAV_RESULT_ACCEPTED: ROI set successfully, auto-yaw now tracking
 *         - MAV_RESULT_FAILED: Invalid location (lat/lon out of range)
 * 
 * @note ROI persists across waypoints in AUTO mode until explicitly cleared
 * @note Some modes may not support ROI (returns accepted but has no effect)
 * @note Yaw rate limited by YAW_SLEW parameter for smooth tracking
 * @note Gimbal-equipped vehicles may point gimbal instead of yawing vehicle
 * 
 * @warning ROI location validity not checked beyond lat/lon range
 * @warning Setting ROI to vehicle's current position may cause erratic yaw
 * 
 * @see Mode::auto_yaw for automatic yaw control implementation
 * @see MAV_CMD_DO_SET_ROI, MAV_CMD_DO_SET_ROI_LOCATION in MAVLink commands
 */
MAV_RESULT GCS_MAVLINK_Copter::handle_command_do_set_roi(const Location &roi_loc)
{
    // Validate location has reasonable lat/lon values
    if (!roi_loc.check_latlng()) {
        return MAV_RESULT_FAILED;
    }
    
    // Configure auto-yaw system to continuously track ROI location
    copter.flightmode->auto_yaw.set_roi(roi_loc);
    return MAV_RESULT_ACCEPTED;
}

/**
 * @brief Handle preflight reboot command with copter-specific safety checks
 * 
 * @details Processes MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN command to reboot or shutdown
 *          flight controller. Adds copter-specific safety check to prevent conflicts
 *          with automatic ESC calibration mode that requires special boot sequence.
 *          
 *          Command parameters (MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN):
 *          - param1: Autopilot (1=reboot, 2=shutdown, 3=reboot and enter bootloader)
 *          - param2: Onboard computer (1=reboot, 2=shutdown)
 *          - param3: Reserved
 *          - param4: Reserved
 *          - param5: Reserved
 *          - param6: Reserved
 *          - param7: Reserved
 *          
 *          ESC calibration conflict:
 *          - Parameter ESC_CALIBRATE can be set to "Auto" (1)
 *          - Auto mode performs ESC calibration on next reboot
 *          - Calibration requires special startup sequence:
 *            1. Boot with throttle at maximum
 *            2. ESCs detect throttle high and enter calibration
 *            3. Throttle moved to minimum
 *            4. ESCs learn PWM range
 *          - Normal reboot command would skip this sequence
 *          - Conflict detected and reboot rejected to prevent confusion
 *          - User must complete ESC cal or cancel it before rebooting normally
 *          
 *          Reboot reasons (typical):
 *          - Apply parameter changes requiring restart
 *          - Recover from software error or stuck state
 *          - Enter bootloader for firmware update
 *          - Reset after configuration changes
 *          - Clear persistent error flags
 *          
 *          Safety considerations:
 *          - Reboot requires vehicle disarmed (checked by base class)
 *          - All flight operations cease immediately
 *          - Logged data may not be fully flushed
 *          - Reboot takes 5-10 seconds typically
 *          - Telemetry connection will drop and reconnect
 * 
 * @param[in] packet COMMAND_INT packet with reboot parameters
 * @param[in] msg Complete MAVLink message for context
 * 
 * @return MAV_RESULT Command execution result
 *         - MAV_RESULT_ACCEPTED: Reboot/shutdown initiated
 *         - MAV_RESULT_FAILED: ESC calibration mode active, reboot rejected
 *         - MAV_RESULT_DENIED: Preconditions not met (still armed, etc.)
 * 
 * @note ESC_CALIBRATE parameter checked to detect calibration mode
 * @note Base class performs actual reboot after copter-specific checks
 * @note Critical error message sent to GCS when reboot rejected
 * 
 * @warning Vehicle will lose control authority immediately on reboot
 * @warning Reboot during flight would cause crash - arming check prevents this
 * @warning Some parameter changes lost if not saved before reboot
 * 
 * @see GCS_MAVLINK::handle_preflight_reboot() for standard reboot handling
 * @see Copter::ESCCalibrationModes for ESC calibration mode enumeration
 */
MAV_RESULT GCS_MAVLINK_Copter::handle_preflight_reboot(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    // Reject reboot if user has also specified they want the "Auto" ESC calibration on next reboot
    // ESC calibration requires special boot sequence that normal reboot would skip
    if (copter.g.esc_calibrate == (uint8_t)Copter::ESCCalibrationModes::ESCCAL_AUTO) {
        send_text(MAV_SEVERITY_CRITICAL, "Reboot rejected, ESC cal on reboot");
        return MAV_RESULT_FAILED;
    }

    // Delegate to base class for standard reboot/shutdown handling
    return GCS_MAVLINK::handle_preflight_reboot(packet, msg);
}

/**
 * @brief Handle MAV_CMD_DO_REPOSITION to fly to new position in guided mode
 * 
 * @details Processes DO_REPOSITION command to immediately fly to a new geographic location.
 *          Command can optionally change vehicle to GUIDED mode automatically. Commonly
 *          used by ground control stations to interactively reposition vehicle during flight.
 *          
 *          Command parameters (MAV_CMD_DO_REPOSITION):
 *          - param1: Ground speed (m/s, -1=use default, 0=not specified)
 *          - param2: Bitmask flags (MAV_DO_REPOSITION_FLAGS)
 *            * Bit 0 (MAV_DO_REPOSITION_FLAGS_CHANGE_MODE): Switch to GUIDED if not already
 *          - param3: Reserved
 *          - param4: Yaw angle (deg, NaN=use default/current)
 *          - x (param5): Target latitude (degE7, INT32_MAX=no change)
 *          - y (param6): Target longitude (degE7, INT32_MAX=no change)
 *          - z (param7): Target altitude (meters, depending on frame)
 *          - frame: Coordinate frame for altitude (GLOBAL, GLOBAL_RELATIVE_ALT, etc.)
 *          
 *          Mode change behavior:
 *          - If MAV_DO_REPOSITION_FLAGS_CHANGE_MODE set: Auto-switch to GUIDED mode
 *          - If flag NOT set: Only works if already in GUIDED mode (or Auto-Guided)
 *          - Mode change happens atomically with position set
 *          - Position loaded twice if mode changed (once to validate, once after switch)
 *          
 *          Validation sequence:
 *          1. Check if already in guided mode OR change_mode flag set
 *          2. Validate latitude/longitude in valid WGS84 range
 *          3. Convert COMMAND_INT to Location structure
 *          4. Sanitize location (check altitude frame, terrain, etc.)
 *          5. Test if guided mode can accept destination
 *          6. If not in guided, attempt mode change
 *          7. Set destination in guided mode controller
 *          
 *          Coordinate frame handling:
 *          - MAV_FRAME_GLOBAL: Altitude relative to MSL
 *          - MAV_FRAME_GLOBAL_RELATIVE_ALT: Altitude relative to home
 *          - MAV_FRAME_GLOBAL_TERRAIN_ALT: Altitude above terrain
 *          - Location sanitized relative to current position
 *          
 *          Guided mode behavior:
 *          - Vehicle flies directly to target position
 *          - Speed controlled by param1 or default WPNAV_SPEED
 *          - Altitude approach may be separate from horizontal
 *          - Vehicle loiters at destination until next command
 *          
 *          Common use cases:
 *          - GCS map click to "fly here" during operation
 *          - Dynamic mission adjustment during flight
 *          - Manual override of auto mission temporarily
 *          - Emergency relocation away from hazard
 * 
 * @param[in] packet COMMAND_INT packet with reposition target and flags
 * 
 * @return MAV_RESULT Command execution result
 *         - MAV_RESULT_ACCEPTED: Reposition command accepted, vehicle flying to target
 *         - MAV_RESULT_DENIED: Not in guided mode and mode change not allowed
 *         - MAV_RESULT_DENIED: Invalid latitude/longitude values
 *         - MAV_RESULT_DENIED: Location failed sanitization checks
 *         - MAV_RESULT_FAILED: Guided mode couldn't accept destination
 *         - MAV_RESULT_FAILED: Mode change to GUIDED failed
 *         - MAV_RESULT_UNSUPPORTED: GUIDED mode not compiled in
 * 
 * @note Command only works if GUIDED mode enabled at compile time
 * @note Position validated before any mode change attempted
 * @note Speed parameter (param1) currently not implemented
 * @note Yaw parameter (param4) currently not implemented
 * 
 * @warning Reposition during low altitude flight may cause terrain collision
 * @warning No obstacle avoidance during straight-line flight to target
 * @warning Sudden large repositions may exceed velocity limits
 * 
 * @see ModeGuided::set_destination() for guided mode position control
 * @see Location::sanitize() for location validation
 */
MAV_RESULT GCS_MAVLINK_Copter::handle_command_int_do_reposition(const mavlink_command_int_t &packet)
{
#if MODE_GUIDED_ENABLED
    // Check if command includes flag to automatically change to GUIDED mode
    const bool change_modes = ((int32_t)packet.param2 & MAV_DO_REPOSITION_FLAGS_CHANGE_MODE) == MAV_DO_REPOSITION_FLAGS_CHANGE_MODE;
    
    // Reject if not in guided mode and mode change not allowed
    if (!copter.flightmode->in_guided_mode() && !change_modes) {
        return MAV_RESULT_DENIED;
    }

    // Sanity check location coordinates are in valid WGS84 range
    if (!check_latlng(packet.x, packet.y)) {
        return MAV_RESULT_DENIED;
    }

    // Convert COMMAND_INT format to Location structure
    Location request_location;
    if (!location_from_command_t(packet, request_location)) {
        return MAV_RESULT_DENIED;
    }

    // Sanitize location relative to current position
    // Checks altitude frame validity, terrain availability, reasonable distance, etc.
    if (request_location.sanitize(copter.current_loc)) {
        // Sanitize returns true if it had to modify location (indicates invalid input)
        return MAV_RESULT_DENIED; // failed as the location is not valid
    }

    // Test if guided mode can accept this destination before changing modes
    // We don't want to change flight mode unless we can also set the target
    if (!copter.mode_guided.set_destination(request_location, false, 0, false, 0)) {
        return MAV_RESULT_FAILED;
    }

    // If not already in guided mode, attempt mode change
    if (!copter.flightmode->in_guided_mode()) {
        if (!copter.set_mode(Mode::Number::GUIDED, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_FAILED;
        }
        // The position won't have been loaded if we had to change the flight mode,
        // so load it again now that we're in GUIDED
        if (!copter.mode_guided.set_destination(request_location, false, 0, false, 0)) {
            return MAV_RESULT_FAILED;
        }
    }

    return MAV_RESULT_ACCEPTED;
#else
    return MAV_RESULT_UNSUPPORTED;
#endif
}

/**
 * @brief Main dispatcher for COMMAND_INT MAVLink messages
 * 
 * @details Central routing function that processes COMMAND_INT messages containing
 *          copter-specific MAVLink commands. COMMAND_INT is the preferred command
 *          format providing integer coordinates with higher precision than COMMAND_LONG.
 *          Routes commands to specialized handlers and provides direct mode changes.
 *          
 *          MAVLink command processing flow:
 *          1. GCS sends COMMAND_INT with command ID and parameters
 *          2. Message decoded and validated by MAVLink library
 *          3. Routed to this handler by GCS_MAVLINK base class
 *          4. Switch statement dispatches to command-specific handler
 *          5. Handler validates parameters and safety conditions
 *          6. Command executed or rejected
 *          7. COMMAND_ACK sent with result code
 *          
 *          Commands handled (copter-specific):
 *          - MAV_CMD_CONDITION_YAW: Set target yaw angle
 *          - MAV_CMD_DO_CHANGE_SPEED: Modify flight speed
 *          - MAV_CMD_DO_REPOSITION: Fly to new position in guided mode
 *          - MAV_CMD_DO_PAUSE_CONTINUE: Pause/resume auto mission
 *          - MAV_CMD_DO_MOTOR_TEST: Test individual motors
 *          - MAV_CMD_NAV_TAKEOFF/VTOL_TAKEOFF: Initiate takeoff
 *          - MAV_CMD_DO_PARACHUTE: Deploy parachute
 *          - MAV_CMD_SOLO_BTN_*: 3DR Solo smart button commands
 *          - MAV_CMD_MISSION_START: Start AUTO mission
 *          - MAV_CMD_DO_WINCH: Control winch deployment
 *          - MAV_CMD_NAV_LOITER_UNLIM: Change to LOITER mode
 *          - MAV_CMD_NAV_RETURN_TO_LAUNCH: Change to RTL mode
 *          - MAV_CMD_NAV_LAND/VTOL_LAND: Change to LAND mode
 *          - MAV_CMD_DO_RETURN_PATH_START: Jump to return path in AUTO
 *          - MAV_CMD_DO_LAND_START: Jump to landing sequence in AUTO
 *          
 *          Commands delegated to base class:
 *          - Standard MAVLink commands (arm/disarm, set_mode, etc.)
 *          - Parameter operations
 *          - Mission upload/download
 *          - Fence operations
 *          - Rally point management
 *          - Camera/gimbal control
 *          - All other MAVLink commands not listed above
 *          
 *          COMMAND_INT vs COMMAND_LONG:
 *          - COMMAND_INT: Uses x,y as int32 lat/lon (degE7), z as float
 *          - COMMAND_LONG: All parameters as float (precision loss for coordinates)
 *          - COMMAND_INT preferred for geographic commands
 *          - Both formats supported, INT has priority if available
 *          
 *          Safety checks (performed by handlers):
 *          - Arming state verification
 *          - Flight mode compatibility
 *          - Parameter range validation
 *          - Position estimate availability
 *          - Hardware capability checks
 *          
 *          Result codes:
 *          - MAV_RESULT_ACCEPTED: Command executed successfully
 *          - MAV_RESULT_TEMPORARILY_REJECTED: Try again (system busy)
 *          - MAV_RESULT_DENIED: Preconditions not met (wrong mode, armed, etc.)
 *          - MAV_RESULT_UNSUPPORTED: Command not implemented or compiled out
 *          - MAV_RESULT_FAILED: Command failed during execution
 * 
 * @param[in] packet Decoded COMMAND_INT message with command and parameters
 * @param[in] msg Original MAVLink message (for routing and tracking)
 * 
 * @return MAV_RESULT Command execution result sent back in COMMAND_ACK
 * 
 * @note Some commands only available if specific features compiled in
 * @note Mode change commands may fail if mode not enabled in parameters
 * @note Commands delegated to base class if not handled here
 * @note COMMAND_ACK automatically sent by MAVLink routing layer
 * 
 * @warning Commands execute immediately without further confirmation
 * @warning Mode changes during critical flight phases could be dangerous
 * 
 * @see GCS_MAVLINK::handle_command_int_packet() for standard command handling
 * @see Individual handle_MAV_CMD_* functions for command-specific documentation
 */
MAV_RESULT GCS_MAVLINK_Copter::handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    // Route command to appropriate handler based on command ID
    switch(packet.command) {

    case MAV_CMD_CONDITION_YAW:
        // Set target yaw angle (heading) for auto/guided modes
        return handle_MAV_CMD_CONDITION_YAW(packet);

    case MAV_CMD_DO_CHANGE_SPEED:
        // Change flight speed (horizontal, climb, or descent)
        return handle_MAV_CMD_DO_CHANGE_SPEED(packet);

    case MAV_CMD_DO_REPOSITION:
        // Fly to new position, optionally changing to guided mode
        return handle_command_int_do_reposition(packet);

    // Pause or resume an auto mission
    case MAV_CMD_DO_PAUSE_CONTINUE:
        return handle_command_pause_continue(packet);

    case MAV_CMD_DO_MOTOR_TEST:
        // Test individual motors for diagnostics
        return handle_MAV_CMD_DO_MOTOR_TEST(packet);

    case MAV_CMD_NAV_TAKEOFF:
    case MAV_CMD_NAV_VTOL_TAKEOFF:
        // Initiate takeoff to specified altitude
        return handle_MAV_CMD_NAV_TAKEOFF(packet);

#if HAL_PARACHUTE_ENABLED
    case MAV_CMD_DO_PARACHUTE:
        // Enable/disable or deploy parachute
        return handle_MAV_CMD_DO_PARACHUTE(packet);
#endif

#if AC_MAVLINK_SOLO_BUTTON_COMMAND_HANDLING_ENABLED
    // 3DR Solo smart button commands for simplified user interface
    case MAV_CMD_SOLO_BTN_PAUSE_CLICK:
        // Solo user presses pause button
        return handle_MAV_CMD_SOLO_BTN_PAUSE_CLICK(packet);
    case MAV_CMD_SOLO_BTN_FLY_HOLD:
        // Solo user presses Fly button (arm/takeoff or land)
        return handle_MAV_CMD_SOLO_BTN_FLY_HOLD(packet);
    case MAV_CMD_SOLO_BTN_FLY_CLICK:
        // Solo user holds down Fly button for a couple of seconds
        return handle_MAV_CMD_SOLO_BTN_FLY_CLICK(packet);
#endif

#if MODE_AUTO_ENABLED
    case MAV_CMD_MISSION_START:
        // Start or resume AUTO mission from current waypoint
        return handle_MAV_CMD_MISSION_START(packet);
#endif

#if AP_WINCH_ENABLED
    case MAV_CMD_DO_WINCH:
        // Control winch deployment rate or position
        return handle_MAV_CMD_DO_WINCH(packet);
#endif

    case MAV_CMD_NAV_LOITER_UNLIM:
        // Change to LOITER mode (hold position indefinitely)
        if (!copter.set_mode(Mode::Number::LOITER, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        // Change to RTL mode (return to home and land)
        if (!copter.set_mode(Mode::Number::RTL, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_NAV_VTOL_LAND:
    case MAV_CMD_NAV_LAND:
        // Change to LAND mode (descend and land at current location)
        if (!copter.set_mode(Mode::Number::LAND, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;

#if MODE_AUTO_ENABLED
    case MAV_CMD_DO_RETURN_PATH_START:
        // Jump to return path (start of RTL sequence) in AUTO mode
        if (copter.mode_auto.return_path_start_auto_RTL(ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;

    case MAV_CMD_DO_LAND_START:
        // Jump to landing sequence (final approach) in AUTO mode
        if (copter.mode_auto.jump_to_landing_sequence_auto_RTL(ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;
#endif

    default:
        // Delegate unhandled commands to base class for standard MAVLink command processing
        return GCS_MAVLINK::handle_command_int_packet(packet, msg);
    }
}

#if HAL_MOUNT_ENABLED
/**
 * @brief Handle gimbal/camera mount control commands with vehicle yaw compensation
 * 
 * @details Processes mount control commands with special handling for copters where
 *          the gimbal lacks pan (yaw) control. When a gimbal cannot pan independently,
 *          the entire vehicle yaws to point the camera at the target. This intercepts
 *          mount commands before passing to base handler to add copter-specific logic.
 *          
 *          Mount control architecture:
 *          - AP_Mount library handles gimbal hardware communication
 *          - GCS_MAVLINK base class handles standard mount protocol
 *          - This override adds copter-specific yaw compensation
 *          - Auto_yaw controller manages vehicle heading for camera pointing
 *          
 *          MAV_CMD_DO_MOUNT_CONTROL parameters:
 *          - param1: Pitch angle (degrees, positive = up)
 *          - param2: Roll angle (degrees, positive = right)
 *          - param3: Yaw angle (degrees, body frame relative to vehicle heading)
 *          - param4-6: Reserved
 *          - param7 (z): Mount mode (MAV_MOUNT_MODE enum)
 *            * MAV_MOUNT_MODE_MAVLINK_TARGETING: GCS controls gimbal angles
 *            * MAV_MOUNT_MODE_RC_TARGETING: RC channels control gimbal
 *            * MAV_MOUNT_MODE_GPS_POINT: Point at GPS coordinate
 *            * MAV_MOUNT_MODE_RETRACT: Move to retracted position
 *            * MAV_MOUNT_MODE_NEUTRAL: Move to neutral position
 *          
 *          Yaw compensation logic:
 *          - Applies only in MAV_MOUNT_MODE_MAVLINK_TARGETING
 *          - Only if camera mount exists and lacks pan control
 *          - Yaw angle in body frame = offset from current heading
 *          - Vehicle rotates to align camera with requested bearing
 *          - Pitch and roll still controlled by gimbal actuators
 *          
 *          Gimbal capabilities:
 *          - 3-axis gimbal: Independent roll, pitch, yaw control
 *          - 2-axis gimbal: Roll and pitch only, vehicle yaws
 *          - 1-axis gimbal: Pitch only (rare on copters)
 *          - Capabilities queried from AP_Mount configuration
 *          
 *          Coordinate frames:
 *          - Mount angles in body frame (relative to vehicle)
 *          - Yaw offset relative to current vehicle heading
 *          - Auto_yaw converts offset to absolute NED heading
 *          - Position controller maintains heading for camera
 *          
 *          Flight mode interactions:
 *          - GUIDED: Yaw control overrides normal guided yaw
 *          - AUTO: Yaw control overrides waypoint-based yaw
 *          - LOITER: Maintains camera pointing while holding position
 *          - Other modes: May conflict with mode's yaw control
 *          
 *          Common use cases:
 *          - FPV piloting with fixed camera (vehicle follows view)
 *          - Tracking moving target without 3-axis gimbal
 *          - Aerial photography where vehicle orientation matters
 *          - Survey missions with 2-axis gimbal
 * 
 * @param[in] packet COMMAND_INT with mount control angles and mode
 * @param[in] msg Original MAVLink message for routing
 * 
 * @return MAV_RESULT Command result from base mount handler
 * 
 * @note Only intercepts DO_MOUNT_CONTROL, other commands pass through
 * @note Yaw compensation disabled if gimbal has pan control
 * @note Base handler still processes command for pitch/roll
 * @note Vehicle yaw changes may affect flight path in some modes
 * 
 * @warning Yaw commands can cause unexpected vehicle rotation
 * @warning May conflict with mode-specific yaw behavior
 * 
 * @see AP_Mount for gimbal hardware abstraction
 * @see Mode::auto_yaw for yaw control architecture
 */
MAV_RESULT GCS_MAVLINK_Copter::handle_command_mount(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    switch (packet.command) {
    case MAV_CMD_DO_MOUNT_CONTROL:
        // If vehicle has a camera mount but it doesn't do pan control, then yaw the entire vehicle instead
        // This allows 2-axis gimbals to point camera in any direction by rotating copter
        if (((MAV_MOUNT_MODE)packet.z == MAV_MOUNT_MODE_MAVLINK_TARGETING) &&
            (copter.camera_mount.get_mount_type() != AP_Mount::Type::None) &&
            !copter.camera_mount.has_pan_control()) {
            // Per the handler in AP_Mount, DO_MOUNT_CONTROL yaw angle is in body frame,
            // which is equivalent to an offset to the current yaw demand
            copter.flightmode->auto_yaw.set_yaw_angle_offset_deg(packet.param3);
        }
        break;
    default:
        break;
    }
    // Always call base handler to process mount control for pitch/roll
    return GCS_MAVLINK::handle_command_mount(packet, msg);
}
#endif

/**
 * @brief Handle MAV_CMD_NAV_TAKEOFF command to initiate takeoff to specified altitude
 * 
 * @details Processes takeoff command from GCS to climb to a specified altitude above
 *          current position. Compatible with both MAV_CMD_NAV_TAKEOFF (fixed-wing style)
 *          and MAV_CMD_NAV_VTOL_TAKEOFF (VTOL style) - copters treat them identically.
 *          Initiates vertical climb with optional pilot horizontal control override.
 *          
 *          Takeoff sequence:
 *          1. GCS sends NAV_TAKEOFF command with target altitude
 *          2. Command validated (frame check, altitude range)
 *          3. Flight mode's do_user_takeoff() called
 *          4. Mode checks arming state and pre-takeoff conditions
 *          5. Motors spin up and vehicle climbs vertically
 *          6. Vehicle reaches target altitude and holds position
 *          
 *          Coordinate frame requirements:
 *          - MUST be MAV_FRAME_GLOBAL_RELATIVE_ALT
 *          - Altitude relative to current ground position
 *          - Latitude/longitude parameters ignored (takeoff at current location)
 *          - Ensures consistent takeoff behavior across GCS implementations
 *          
 *          MAV_CMD_NAV_TAKEOFF parameters:
 *          - param1: Pitch angle (not used on copters, fixed-wing only)
 *          - param2: Empty
 *          - param3: Pilot horizontal navigation flag
 *            * 0 = autopilot controls horizontal position (GPS lock required)
 *            * Non-zero = pilot can override horizontal position with sticks
 *          - param4: Yaw angle (not supported - vehicle maintains current heading)
 *          - param5 (x): Latitude (not supported - takeoff at current position)
 *          - param6 (y): Longitude (not supported - takeoff at current position)
 *          - param7 (z): Altitude in meters (relative to current position)
 *          
 *          Altitude handling:
 *          - Input in meters, converted to centimeters (ArduPilot internal units)
 *          - Relative to current position at command receipt
 *          - Range typically limited by PILOT_TKOFF_ALT parameter
 *          - Too high: May fail if exceeds fence or battery limits
 *          - Too low: May fail minimum altitude checks
 *          
 *          Flight mode behavior:
 *          - GUIDED: Takeoff at current position, then await next command
 *          - AUTO: Executes as mission item, proceeds to next waypoint
 *          - Other modes: Takeoff may be rejected if mode doesn't support it
 *          - Mode must implement do_user_takeoff() for command to succeed
 *          
 *          Pilot control during takeoff:
 *          - param3 = 0: Autopilot maintains horizontal position (GPS hold)
 *          - param3 != 0: Pilot can move horizontally with RC sticks
 *          - Altitude control always automatic during takeoff
 *          - Yaw control follows mode-specific behavior
 *          
 *          Safety checks (performed by mode):
 *          - Vehicle must be armed or armable
 *          - GPS lock required for position hold (param3=0)
 *          - EKF health check for position estimate
 *          - Geofence altitude limits respected
 *          - Battery capacity check
 *          
 *          Failure conditions:
 *          - Wrong coordinate frame specified
 *          - Altitude out of valid range
 *          - Mode doesn't support takeoff
 *          - Vehicle not armed and cannot arm
 *          - Position estimate unavailable (GPS/EKF)
 *          - Safety interlocks not cleared
 * 
 * @param[in] packet COMMAND_INT with takeoff altitude and parameters
 * 
 * @return MAV_RESULT_ACCEPTED if takeoff initiated
 * @return MAV_RESULT_DENIED if frame invalid or parameters out of range
 * @return MAV_RESULT_FAILED if mode rejected takeoff or vehicle not ready
 * 
 * @note Copters ignore latitude/longitude - always takeoff at current position
 * @note Yaw parameter not supported - vehicle maintains current heading
 * @note Both MAV_CMD_NAV_TAKEOFF and MAV_CMD_NAV_VTOL_TAKEOFF handled identically
 * @note Altitude converted from meters to centimeters for internal use
 * 
 * @warning Ensure adequate clearance above current position before takeoff
 * @warning GPS lock required for autonomous position hold during climb
 * @warning Pilot horizontal control (param3!=0) requires RC connection
 * 
 * @see Mode::do_user_takeoff() for mode-specific takeoff implementation
 * @see PILOT_TKOFF_ALT parameter for default/maximum takeoff altitude
 */
MAV_RESULT GCS_MAVLINK_Copter::handle_MAV_CMD_NAV_TAKEOFF(const mavlink_command_int_t &packet)
{
    // Validate coordinate frame - must be relative altitude for consistent behavior
    if (packet.frame != MAV_FRAME_GLOBAL_RELATIVE_ALT) {
        return MAV_RESULT_DENIED;  // Wrong frame = bad parameters
    }

    // MAV_CMD_NAV_TAKEOFF parameter definitions:
    // param1: Pitch angle (fixed-wing only, not used on copters)
    // param2: Empty
    // param3: Horizontal navigation by pilot acceptable (0=no pilot control, 1=pilot can override)
    // param4: Yaw angle (not supported on copters)
    // param5 (x): Latitude (not supported - takeoff at current position)
    // param6 (y): Longitude (not supported - takeoff at current position)
    // param7 (z): Altitude in meters (relative to current position)

    float takeoff_alt = packet.z * 100;      // Convert meters to centimeters (ArduPilot internal units)

    // Request takeoff from current flight mode
    // is_zero(packet.param3) = true means no pilot horizontal control (position hold)
    // is_zero(packet.param3) = false means pilot can control horizontal position during takeoff
    if (!copter.flightmode->do_user_takeoff(takeoff_alt, is_zero(packet.param3))) {
        return MAV_RESULT_FAILED;  // Mode rejected takeoff (not armed, no GPS, etc.)
    }
    return MAV_RESULT_ACCEPTED;
}

#if AP_MAVLINK_COMMAND_LONG_ENABLED
/**
 * @brief Specify default coordinate frame for COMMAND_LONG to COMMAND_INT conversion
 * 
 * @details Provides default coordinate frame when converting legacy COMMAND_LONG messages
 *          to modern COMMAND_INT format. COMMAND_LONG lacks explicit frame parameter,
 *          so this function supplies the appropriate frame for each command type.
 *          Required for backwards compatibility with older GCS software.
 *          
 *          MAVLink command format evolution:
 *          - Legacy: COMMAND_LONG (all params as float, no frame specified)
 *          - Modern: COMMAND_INT (x,y as int32, z as float, explicit frame)
 *          - Conversion needed for GCS using old protocol format
 *          - Frame defaults ensure consistent interpretation of parameters
 *          
 *          Coordinate frame selection rationale:
 *          - Takeoff commands: GLOBAL_RELATIVE_ALT
 *            * Altitude relative to current ground position
 *            * Most intuitive for takeoff operations
 *            * Matches handle_MAV_CMD_NAV_TAKEOFF() requirements
 *            * Prevents accidental absolute altitude interpretation
 *          
 *          MAV_FRAME types commonly used:
 *          - MAV_FRAME_GLOBAL_RELATIVE_ALT: Relative to home position altitude
 *          - MAV_FRAME_GLOBAL: Absolute AMSL altitude
 *          - MAV_FRAME_LOCAL_NED: Meters from EKF origin
 *          - MAV_FRAME_BODY_NED: Meters relative to vehicle
 *          
 *          Command categories handled:
 *          - Navigation commands (NAV_*): Geographic waypoints, takeoff, landing
 *          - DO commands (DO_*): Actions at waypoints, immediate execution
 *          - Each category may have different frame requirements
 *          
 *          Copter-specific overrides:
 *          - MAV_CMD_NAV_TAKEOFF: Force GLOBAL_RELATIVE_ALT
 *          - MAV_CMD_NAV_VTOL_TAKEOFF: Force GLOBAL_RELATIVE_ALT
 *          - All other commands: Delegate to base class defaults
 *          
 *          Conversion process:
 *          1. GCS sends COMMAND_LONG without frame parameter
 *          2. MAVLink library detects legacy format
 *          3. Calls this function to get appropriate frame
 *          4. Converts to COMMAND_INT with specified frame
 *          5. Processes through normal COMMAND_INT handler
 *          
 *          Why COMMAND_INT is preferred:
 *          - Integer lat/lon in degE7 prevents floating-point precision loss
 *          - Explicit frame parameter avoids ambiguity
 *          - Supports geographic coordinates beyond float32 precision
 *          - Backward compatible through automatic conversion
 * 
 * @param[out] frame Coordinate frame to use for this command (set by function)
 * @param[in] packet_command MAVLink command ID to get frame for
 * 
 * @return true if frame specified (output valid), false to use default
 * 
 * @note Only active if AP_MAVLINK_COMMAND_LONG_ENABLED compiled in
 * @note COMMAND_INT messages don't need conversion (already have frame)
 * @note Base class provides frames for standard commands not listed here
 * 
 * @see handle_MAV_CMD_NAV_TAKEOFF() for frame validation
 * @see GCS_MAVLINK::mav_frame_for_command_long() for standard frame mappings
 */
bool GCS_MAVLINK_Copter::mav_frame_for_command_long(MAV_FRAME &frame, MAV_CMD packet_command) const
{
    // Provide frame for takeoff commands converted from COMMAND_LONG format
    if (packet_command == MAV_CMD_NAV_TAKEOFF ||
        packet_command == MAV_CMD_NAV_VTOL_TAKEOFF) {
        // Use relative altitude frame for takeoff commands
        // Matches requirement in handle_MAV_CMD_NAV_TAKEOFF()
        frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
        return true;  // Frame specified, use our value
    }
    // Delegate other commands to base class for standard frame mappings
    return GCS_MAVLINK::mav_frame_for_command_long(frame, packet_command);
}
#endif


/**
 * @brief Handle MAV_CMD_CONDITION_YAW to set target vehicle heading
 * 
 * @details Commands the vehicle to yaw (rotate horizontally) to a specified heading
 *          at a specified rate. Used in AUTO and GUIDED modes to control vehicle
 *          orientation independently of flight path. The command can specify absolute
 *          or relative heading and control rotation direction and speed.
 *          
 *          Yaw control modes:
 *          - Absolute: Yaw to compass heading (0-360°, 0=North, 90=East)
 *          - Relative: Yaw by offset from current heading (+/- degrees)
 *          - Direction control: Shortest path, clockwise, or counter-clockwise
 *          - Speed control: Deg/s rate limit during rotation
 *          
 *          MAV_CMD_CONDITION_YAW parameters:
 *          - param1: Target angle [0-360 degrees]
 *            * Absolute mode: Compass heading (0=North, 90=East, 180=South, 270=West)
 *            * Relative mode: Offset from current heading (positive=CW, negative=CCW)
 *          - param2: Rotation speed [deg/s]
 *            * 0 = use default yaw speed from parameters
 *            * >0 = rotate at specified rate
 *            * Rate limited by ATC_SLEW_YAW parameter
 *          - param3: Direction control
 *            * -1 = rotate counter-clockwise (CCW)
 *            * +1 = rotate clockwise (CW)
 *            * 0 = shortest path (automatic direction selection)
 *          - param4: Relative/absolute flag
 *            * 0 = absolute angle (compass heading)
 *            * 1 = relative offset (change by N degrees)
 *          
 *          Validation rules:
 *          - param1 must be in range [0, 360] degrees
 *          - param4 must be exactly 0 or 1 (not intermediate values)
 *          - Other parameters not validated (clamped by auto_yaw controller)
 *          
 *          AUTO mode behavior:
 *          - Yaw command typically in mission as condition command
 *          - Vehicle maintains yaw while executing subsequent NAV commands
 *          - Yaw override persists until next yaw command or ROI set
 *          - Does not delay mission progression (non-blocking)
 *          
 *          GUIDED mode behavior:
 *          - Yaw command affects current guided target orientation
 *          - Combined with position/velocity commands for coordinated control
 *          - Maintained during position hold or trajectory following
 *          - Overridden by SET_ATTITUDE_TARGET messages
 *          
 *          Coordinate frames:
 *          - Absolute angles in NED frame (North=0°, East=90°)
 *          - Body frame angles relative to current vehicle heading
 *          - Magnetic declination NOT applied (true North=0°)
 *          - Conversion to radians for internal use
 *          
 *          Rotation direction logic:
 *          - param3=-1: Always CCW even if longer rotation
 *          - param3=+1: Always CW even if longer rotation  
 *          - param3=0: Shortest path (auto-selects CW or CCW)
 *          - Direction combined with relative/absolute mode
 *          
 *          Common use cases:
 *          - Point camera/sensor in specific direction
 *          - Align vehicle for payload deployment
 *          - Orient vehicle for landing approach
 *          - Survey missions with heading control
 *          - FPV flight with predictable orientation
 *          
 *          Flight mode compatibility:
 *          - AUTO: Full support as mission condition command
 *          - GUIDED: Full support for GCS yaw control
 *          - LOITER: Supported (maintains yaw while holding position)
 *          - RTL/LAND: May be overridden by mode-specific yaw behavior
 *          - STABILIZE/ALTHOLD: Not applicable (manual yaw control)
 *          
 *          Rate limiting:
 *          - Yaw rate limited by ATC_SLEW_YAW parameter (deg/s)
 *          - Prevents mechanical stress on airframe
 *          - Ensures stable control during rotation
 *          - May take longer than param2 suggests if limited
 * 
 * @param[in] packet COMMAND_INT with yaw target and rotation parameters
 * 
 * @return MAV_RESULT_ACCEPTED if yaw command valid and applied
 * @return MAV_RESULT_FAILED if parameters out of range
 * 
 * @note Angles converted from degrees to radians for internal use
 * @note Yaw maintained until overridden by another yaw command
 * @note Does not block mission progression in AUTO mode
 * @note Zero yaw rate uses default from parameters
 * 
 * @warning Rapid yaw changes may affect stability in windy conditions
 * @warning Yaw commands override ROI (region of interest) yaw control
 * 
 * @see Mode::auto_yaw for yaw control architecture
 * @see ATC_SLEW_YAW parameter for rate limiting configuration
 */
MAV_RESULT GCS_MAVLINK_Copter::handle_MAV_CMD_CONDITION_YAW(const mavlink_command_int_t &packet)
{
    // MAV_CMD_CONDITION_YAW parameter definitions:
    // param1: Target angle [0-360 degrees]
    // param2: Rotation speed [deg/s]
    // param3: Direction (-1=CCW, +1=CW, 0=shortest path)
    // param4: Relative offset (1) or absolute angle (0)
    
    // Validate parameters before applying yaw command
    if ((packet.param1 >= 0.0f)   &&      // Angle must be non-negative
        (packet.param1 <= 360.0f) &&      // Angle must not exceed 360°
        (is_zero(packet.param4) || is_equal(packet.param4,1.0f))) {  // Relative flag must be exactly 0 or 1
        
        // Set fixed yaw target through auto_yaw controller
        // Converts degrees to radians and applies rotation direction
        copter.flightmode->auto_yaw.set_fixed_yaw_rad(
            radians(packet.param1),        // Target angle in radians
            radians(packet.param2),        // Yaw rate in rad/s (0=default)
            (int8_t)packet.param3,         // Direction: -1=CCW, +1=CW, 0=shortest
            is_positive(packet.param4));   // true=relative offset, false=absolute heading
        
        return MAV_RESULT_ACCEPTED;
    }
    // Parameter validation failed
    return MAV_RESULT_FAILED;
}

/**
 * @brief Handle MAV_CMD_DO_CHANGE_SPEED to modify flight speed
 * 
 * @details Changes the vehicle's horizontal, climb, or descent speed during flight.
 *          Allows GCS to dynamically adjust speeds in AUTO or GUIDED modes without
 *          modifying mission waypoints or parameters. Speed changes persist until
 *          another speed command or mode change. Supports independent control of
 *          horizontal movement, climbing, and descending speeds.
 *          
 *          Speed types supported:
 *          - GROUNDSPEED: Horizontal velocity over ground (XY plane)
 *          - AIRSPEED: Treated as groundspeed on copters (no airspeed sensor typically)
 *          - CLIMB_SPEED: Vertical ascent rate (positive Z, up)
 *          - DESCENT_SPEED: Vertical descent rate (negative Z, down)
 *          
 *          MAV_CMD_DO_CHANGE_SPEED parameters:
 *          - param1: Speed type (SPEED_TYPE enum)
 *            * 0 = AIRSPEED (m/s) - treated as groundspeed on copters
 *            * 1 = GROUNDSPEED (m/s) - horizontal velocity
 *            * 2 = CLIMB_SPEED (m/s) - vertical ascent rate
 *            * 3 = DESCENT_SPEED (m/s) - vertical descent rate
 *          - param2: Target speed [m/s]
 *            * Must be positive (>0)
 *            * Converted to cm/s internally (x100)
 *            * Limited by vehicle capabilities and parameters
 *          - param3: Throttle (not used on copters, fixed-wing only)
 *          - param4: Relative/absolute (not currently used)
 *          
 *          Validation:
 *          - Speed must be positive (param2 > 0)
 *          - Speed type must be valid enum value
 *          - Mode must support dynamic speed changes
 *          - Actual speed may be limited by safety parameters
 *          
 *          Flight mode interactions:
 *          - AUTO: Speed affects waypoint navigation and mission execution
 *          - GUIDED: Speed affects position/velocity commands
 *          - LOITER: Horizontal speed affects position corrections
 *          - RTL: Speed may affect return flight
 *          - LAND: Descent speed affects landing rate
 *          - Other modes: Command may be rejected or ignored
 *          
 *          Speed limiting:
 *          - WPNAV_SPEED (cm/s): Maximum horizontal waypoint speed
 *          - WPNAV_SPEED_UP (cm/s): Maximum climb rate
 *          - WPNAV_SPEED_DN (cm/s): Maximum descent rate
 *          - WPNAV_ACCEL (cm/s²): Horizontal acceleration limit
 *          - WPNAV_ACCEL_Z (cm/s²): Vertical acceleration limit
 *          - Commands cannot exceed these parameter limits
 *          
 *          Unit conversions:
 *          - Input: meters per second (m/s, MAVLink standard)
 *          - Internal: centimeters per second (cm/s, ArduPilot standard)
 *          - Conversion factor: x100 (1 m/s = 100 cm/s)
 *          
 *          Airspeed vs groundspeed:
 *          - Copters typically lack airspeed sensors
 *          - AIRSPEED treated identically to GROUNDSPEED
 *          - Maintains compatibility with fixed-wing mission files
 *          - Wind correction automatic through position controller
 *          
 *          Speed persistence:
 *          - Speed change persists until next speed command
 *          - Mode changes may reset to default speeds
 *          - Mission speed commands apply from that point forward
 *          - Parameter defaults restored on disarm
 *          
 *          Common use cases:
 *          - Slow down for precision maneuvers
 *          - Speed up transit segments
 *          - Gentle landing (reduce descent speed)
 *          - Conservative climb (reduce climb speed)
 *          - Energy-efficient cruise speeds
 *          
 *          Safety considerations:
 *          - Very slow speeds may reduce stability
 *          - Very fast speeds increase crash risk
 *          - Descent speed affects landing safety
 *          - Climb speed affects obstacle clearance time
 * 
 * @param[in] packet COMMAND_INT with speed type and target speed
 * 
 * @return MAV_RESULT_ACCEPTED if speed changed successfully
 * @return MAV_RESULT_DENIED if speed invalid (<=0 or bad type)
 * @return MAV_RESULT_FAILED if mode rejected speed change
 * 
 * @note Speed values converted from m/s to cm/s internally
 * @note Copters treat airspeed and groundspeed identically
 * @note Speed changes may be limited by parameter maximums
 * @note Mode must support dynamic speed changes
 * 
 * @warning Very low speeds may affect flight stability
 * @warning Speed changes affect battery consumption and range
 * 
 * @see WPNAV_SPEED, WPNAV_SPEED_UP, WPNAV_SPEED_DN parameters
 * @see Mode::set_speed_xy_cms() for horizontal speed control
 * @see Mode::set_speed_up_cms() for climb speed control
 * @see Mode::set_speed_down_cms() for descent speed control
 */
MAV_RESULT GCS_MAVLINK_Copter::handle_MAV_CMD_DO_CHANGE_SPEED(const mavlink_command_int_t &packet)
{
    // Validate speed is positive
    if (!is_positive(packet.param2)) {
        // Target speed must be greater than zero
        return MAV_RESULT_DENIED;
    }

    // Convert speed from m/s (MAVLink standard) to cm/s (ArduPilot internal units)
    const float speed_cms = packet.param2 * 100.0;

    // Apply speed change based on speed type
    bool success = false;
    switch (SPEED_TYPE(packet.param1)) {
        case SPEED_TYPE_ENUM_END:
            // Invalid speed type
            return MAV_RESULT_DENIED;

        case SPEED_TYPE_AIRSPEED: 
            // Airspeed is treated as ground speed for GCS compatibility
            // (copters typically don't have airspeed sensors)
        case SPEED_TYPE_GROUNDSPEED:
            // Set horizontal (XY plane) speed for waypoint navigation
            success = copter.flightmode->set_speed_xy_cms(speed_cms);
            break;

        case SPEED_TYPE_CLIMB_SPEED:
            // Set vertical climb (up) speed
            success = copter.flightmode->set_speed_up_cms(speed_cms);
            break;

        case SPEED_TYPE_DESCENT_SPEED:
            // Set vertical descent (down) speed
            success = copter.flightmode->set_speed_down_cms(speed_cms);
            break;
    }

    // Return success or failure based on mode's ability to change speed
    return success ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;
}

#if MODE_AUTO_ENABLED
/**
 * @brief Handle MAV_CMD_MISSION_START to begin or resume AUTO mission
 * 
 * @details Starts execution of the uploaded mission by switching to AUTO mode and
 *          initiating mission progression. If mission already running, ensures it
 *          continues from current waypoint. Automatically arms the vehicle if required.
 *          Primary method for GCS to begin autonomous mission execution.
 *          
 *          Mission start sequence:
 *          1. Validate command parameters (no partial mission support)
 *          2. Request mode change to AUTO
 *          3. Set auto_armed flag (allows AUTO to arm vehicle)
 *          4. Check mission state
 *          5. Start or resume mission if not already running
 *          6. Vehicle begins executing from current mission item
 *          
 *          MAV_CMD_MISSION_START parameters:
 *          - param1: First mission item (0=current, not supported otherwise)
 *          - param2: Last mission item (0=no limit, not supported otherwise)
 *          - param3-7: Reserved/unused
 *          
 *          Parameter restrictions:
 *          - param1 must be 0 (start from current/first item)
 *          - param2 must be 0 (run entire mission)
 *          - Partial mission execution not supported
 *          - Use DO_JUMP mission items for mission segmentation instead
 *          
 *          Mission state handling:
 *          - MISSION_RUNNING: Mission already executing, no action needed
 *          - MISSION_STOPPED: Resume from last active waypoint
 *          - MISSION_COMPLETE: Restart from beginning
 *          - No mission loaded: AUTO mode active but no mission execution
 *          
 *          AUTO mode requirements:
 *          - Valid mission uploaded via mission protocol
 *          - GPS lock with good position estimate (typical)
 *          - EKF healthy and initialized
 *          - Battery capacity sufficient for mission
 *          - Geofence checks passed (if enabled)
 *          - Arming checks satisfied (or auto_armed allows arming)
 *          
 *          Auto-arming behavior:
 *          - set_auto_armed(true) permits AUTO mode to arm vehicle
 *          - Vehicle arms automatically if pre-arm checks pass
 *          - Allows seamless mission start without separate arm command
 *          - Standard arming checks still apply
 *          - ARMING_CHECK parameter controls which checks enforced
 *          
 *          Mission execution:
 *          - Starts from first waypoint if mission complete/stopped
 *          - Resumes from current waypoint if mission paused
 *          - Progresses through DO_* commands and NAV_* waypoints
 *          - Continues until mission complete or interrupted
 *          - Mission completion typically triggers RTL or LAND
 *          
 *          Mode change failure reasons:
 *          - AUTO mode disabled in FLTMODE parameters
 *          - Pre-flight checks failed (no GPS, EKF unhealthy)
 *          - Geofence breach detected
 *          - Battery failsafe active
 *          - Mission not uploaded or invalid
 *          
 *          Mission upload protocol:
 *          - GCS uploads mission via MISSION_COUNT/MISSION_ITEM messages
 *          - Mission stored in EEPROM or RAM
 *          - Mission validated on upload
 *          - This command starts uploaded mission
 *          
 *          Comparison with other mission commands:
 *          - MISSION_START: Start/resume full mission execution
 *          - DO_PAUSE_CONTINUE: Pause/resume without mode change
 *          - DO_JUMP: Jump to specific mission item programmatically
 *          - MAV_CMD_DO_SET_MODE AUTO: Changes mode but doesn't ensure mission starts
 *          
 *          Safety considerations:
 *          - Vehicle may arm and takeoff automatically
 *          - Ensure safe clearance and GPS lock before command
 *          - Mission should start with safe takeoff sequence
 *          - Emergency stop via mode change or RC override
 *          
 *          Common use cases:
 *          - Automated survey missions
 *          - Delivery flight plans
 *          - Inspection routes
 *          - Search and rescue patterns
 *          - Agricultural operations
 * 
 * @param[in] packet COMMAND_INT with mission start parameters
 * 
 * @return MAV_RESULT_ACCEPTED if AUTO mode entered and mission started
 * @return MAV_RESULT_DENIED if parameters invalid (partial mission not supported)
 * @return MAV_RESULT_FAILED if mode change to AUTO failed
 * 
 * @note Only compiled if MODE_AUTO_ENABLED is set
 * @note Vehicle may auto-arm if pre-arm checks pass
 * @note Partial mission execution (param1, param2) not supported
 * @note Mission must be uploaded before this command
 * 
 * @warning Vehicle may takeoff automatically if mission starts with NAV_TAKEOFF
 * @warning Ensure safe clearance and GPS lock before issuing command
 * 
 * @see AP_Mission for mission management and execution
 * @see Mode_Auto for AUTO mode implementation
 * @see ARMING_CHECK parameter for pre-arm validation control
 */
MAV_RESULT GCS_MAVLINK_Copter::handle_MAV_CMD_MISSION_START(const mavlink_command_int_t &packet)
{
        // Validate mission start parameters
        if (!is_zero(packet.param1) || !is_zero(packet.param2)) {
            // Partial mission execution (first-item/last-item) not supported
            // param1 and param2 must both be zero
            return MAV_RESULT_DENIED;
        }
        
        // Attempt to switch to AUTO mode for mission execution
        if (copter.set_mode(Mode::Number::AUTO, ModeReason::GCS_COMMAND)) {
            // Mode change successful
            
            // Enable auto_armed to allow AUTO mode to arm the vehicle automatically
            // Vehicle will arm if pre-arm checks pass
            copter.set_auto_armed(true);
            
            // Check if mission already running
            if (copter.mode_auto.mission.state() != AP_Mission::MISSION_RUNNING) {
                // Mission not running, start or resume it
                // Starts from first item if complete, resumes from current if paused
                copter.mode_auto.mission.start_or_resume();
            }
            // else: Mission already running, no action needed
            
            return MAV_RESULT_ACCEPTED;
        }
        
        // Mode change to AUTO failed (pre-flight checks, no GPS, etc.)
        return MAV_RESULT_FAILED;
}
#endif



#if HAL_PARACHUTE_ENABLED
/**
 * @brief Handle MAV_CMD_DO_PARACHUTE to control parachute system
 * 
 * @details Enables, disables, or triggers emergency parachute deployment. Parachute
 *          provides last-resort recovery mechanism for catastrophic failures. Manual
 *          release includes altitude checks to prevent ground-level deployment. Enable/
 *          disable allows temporary parachute system control without parameter changes.
 *          
 *          Parachute system overview:
 *          - Emergency recovery device for multi-rotor failures
 *          - Typically pyrotechnic or spring-loaded deployment
 *          - Controlled via servo/relay output channel
 *          - Automatic deployment via failsafe logic
 *          - Manual deployment via GCS command (this function)
 *          
 *          MAV_CMD_DO_PARACHUTE actions (param1):
 *          - PARACHUTE_DISABLE (0): Temporarily disable parachute system
 *            * Prevents automatic or manual deployment
 *            * Used for ground testing or maintenance
 *            * Does not affect CHUTE_ENABLED parameter
 *          - PARACHUTE_ENABLE (1): Re-enable parachute system
 *            * Restores automatic deployment capability
 *            * Allows manual release commands
 *            * Returns to normal operational state
 *          - PARACHUTE_RELEASE (2): Immediately deploy parachute
 *            * Triggers manual parachute deployment
 *            * Includes altitude safety checks
 *            * Logs deployment event
 *            * Disarms motors after deployment
 *          
 *          Manual release safety checks:
 *          - Altitude check: Must be above minimum deployment altitude
 *          - CHUTE_ALT_MIN parameter: Minimum safe deployment altitude (meters)
 *          - Prevents ground-level deployment that could entangle
 *          - Automatic deployment bypasses checks (emergency situation)
 *          
 *          Parachute deployment sequence:
 *          1. Validate altitude (manual release only)
 *          2. Trigger servo/relay output
 *          3. Log deployment to dataflash
 *          4. Send notification to GCS
 *          5. Disarm motors (prevent prop strike)
 *          6. Vehicle descends under parachute
 *          
 *          Configuration parameters:
 *          - CHUTE_ENABLED: Enable/disable parachute system (1=enabled)
 *          - CHUTE_TYPE: Servo or relay control (0=disabled, 1=servo, 2=relay)
 *          - CHUTE_SERVO_ON: Servo PWM value for deployed state
 *          - CHUTE_SERVO_OFF: Servo PWM value for stowed state
 *          - CHUTE_ALT_MIN: Minimum deployment altitude (meters AGL)
 *          - CHUTE_CRT_SINK: Critical sink rate triggering auto-deploy (m/s)
 *          
 *          Automatic deployment triggers:
 *          - Exceeded critical sink rate (CHUTE_CRT_SINK)
 *          - EKF failsafe and high descent rate
 *          - Loss of control (attitude error exceeds limits)
 *          - Typically indicates motor failure or loss of control
 *          
 *          Hardware requirements:
 *          - Dedicated servo or relay output
 *          - Parachute deployment mechanism (pyro, spring, CO2)
 *          - Proper mechanical mounting and routing
 *          - Sufficient altitude for full deployment
 *          
 *          Enable/disable use cases:
 *          - DISABLE: Ground testing without deployment risk
 *          - DISABLE: Maintenance mode for parachute servicing
 *          - ENABLE: Restore to operational state after testing
 *          - Toggle during flight rarely needed
 *          
 *          Manual release use cases:
 *          - Testing parachute system in flight (risky!)
 *          - Emergency deployment by pilot command
 *          - Triggered by companion computer detecting failure
 *          - Last-resort recovery attempt
 *          
 *          Post-deployment behavior:
 *          - Motors disarmed immediately
 *          - Vehicle descends under parachute
 *          - No control authority after deployment
 *          - GPS track logged to assist recovery
 *          - Potentially flyable again if parachute replaceable
 *          
 *          Safety considerations:
 *          - Parachute deployment is irreversible
 *          - Vehicle may be damaged on landing
 *          - Deploy only in true emergency or safe test
 *          - Ensure sufficient altitude (above CHUTE_ALT_MIN)
 *          - Check for obstacles and people below
 *          - Manual release should be last resort
 *          
 *          Testing recommendations:
 *          - Test enable/disable on ground only
 *          - Test deployment at altitude with expendable vehicle
 *          - Use SITL simulation for logic testing
 *          - Verify servo/relay output before flight
 *          - Practice emergency procedures
 * 
 * @param[in] packet COMMAND_INT with parachute action in param1
 * 
 * @return MAV_RESULT_ACCEPTED if action executed successfully
 * @return MAV_RESULT_FAILED if action invalid or not recognized
 * 
 * @note Only compiled if HAL_PARACHUTE_ENABLED is set
 * @note Manual release includes altitude safety checks
 * @note Enable/disable does not modify CHUTE_ENABLED parameter
 * @note Parachute deployment disarms motors
 * 
 * @warning Parachute deployment is irreversible - use with extreme caution
 * @warning Manual release should only be used in genuine emergency
 * @warning Ensure sufficient altitude before deployment
 * 
 * @see AP_Parachute for parachute system implementation
 * @see Copter::parachute_manual_release() for manual release logic
 * @see CHUTE_* parameters for parachute configuration
 */
MAV_RESULT GCS_MAVLINK_Copter::handle_MAV_CMD_DO_PARACHUTE(const mavlink_command_int_t &packet)
{
        // Determine requested parachute action from param1
        switch ((uint16_t)packet.param1) {
        case PARACHUTE_DISABLE:
            // Temporarily disable parachute system (prevents deployment)
            copter.parachute.enabled(false);
            return MAV_RESULT_ACCEPTED;
            
        case PARACHUTE_ENABLE:
            // Re-enable parachute system (allows deployment)
            copter.parachute.enabled(true);
            return MAV_RESULT_ACCEPTED;
            
        case PARACHUTE_RELEASE:
            // Deploy parachute immediately
            // Treat as manual release which performs altitude safety checks
            // (automatic deployment bypasses checks in emergency)
            copter.parachute_manual_release();
            return MAV_RESULT_ACCEPTED;
        }
        
        // Unknown parachute action
        return MAV_RESULT_FAILED;
}
#endif

/**
 * @brief Handle MAV_CMD_DO_MOTOR_TEST to test individual motors
 * 
 * @details Tests individual motors or sequences of motors at specified throttle levels
 *          for diagnostic, verification, and setup purposes. Essential for confirming
 *          motor direction, output mapping, ESC calibration, and propeller installation.
 *          Performs safety checks before allowing motor test. Used during initial setup
 *          and troubleshooting to verify vehicle configuration without flight.
 *          
 *          Motor test capabilities:
 *          - Test single motor at a time
 *          - Test sequence of motors in order
 *          - Multiple throttle control modes
 *          - Configurable test duration
 *          - Different motor ordering schemes
 *          
 *          MAV_CMD_DO_MOTOR_TEST parameters:
 *          - param1: Motor sequence number
 *            * 1 to max_motors (typically 1-8 for octocopters, 1-4 for quadcopters)
 *            * Motor number in configured order
 *            * Frame-dependent numbering (see motor order diagram)
 *          - param2: Throttle type (MOTOR_TEST_THROTTLE_TYPE enum)
 *            * 0 = Throttle percentage (0-100%)
 *            * 1 = PWM value (typically 1000-2000 µs)
 *            * 2 = Pilot throttle channel pass-through
 *          - param3: Throttle value (interpretation depends on param2)
 *            * Percentage: 0-100 (5-10% typical for safe test)
 *            * PWM: 1000-2000 µs (1100-1200 for safe test)
 *            * Pass-through: ignored (uses actual throttle stick)
 *          - param4: Test duration timeout (seconds)
 *            * Recommended: 1-5 seconds
 *            * Motor stops automatically after timeout
 *            * Maximum enforced by safety limits
 *          - param5 (x in COMMAND_INT): Number of motors in sequence
 *            * 0 or 1: Test single motor (param1)
 *            * >1: Test sequence starting from param1
 *          - param6: Motor test order (not currently used)
 *          
 *          Safety pre-conditions:
 *          - Vehicle disarmed (motors not spinning)
 *          - Vehicle on ground (land_complete flag set)
 *          - No active failsafes
 *          - Throttle stick at zero (some modes)
 *          - Safety switch enabled (if installed)
 *          - Minimum/maximum throttle validated
 *          
 *          Throttle type details:
 *          
 *          THROTTLE_PERCENT (0):
 *          - Percentage of maximum throttle (0-100%)
 *          - Easiest and safest for operators
 *          - 5-10% sufficient to verify motor spin
 *          - Independent of ESC calibration
 *          - Recommended for most testing
 *          
 *          THROTTLE_PWM (1):
 *          - Direct PWM value to ESC (typically 1000-2000 µs)
 *          - Useful for ESC calibration verification
 *          - Requires knowledge of ESC range
 *          - 1100-1200 µs safe starting point
 *          - May not spin motor if below MOT_SPIN_MIN
 *          
 *          THROTTLE_PILOT (2):
 *          - Uses pilot's actual throttle stick input
 *          - Allows real-time throttle control during test
 *          - Operator controls test throttle
 *          - Useful for variable-speed testing
 *          
 *          Sequential motor test:
 *          - Set param5 to number of motors to test
 *          - Tests motors in order starting from param1
 *          - Each motor runs for full timeout duration
 *          - Total test time = timeout × num_motors
 *          - Helps verify motor numbering sequence
 *          
 *          Motor numbering schemes:
 *          - Frame-dependent (quadcopter: 1=front-right, 2=rear-left, etc.)
 *          - Configured in FRAME_CLASS and FRAME_TYPE parameters
 *          - Diagrams available in documentation for each frame
 *          - Motor order critical for stable flight
 *          - CCW vs CW rotation alternates
 *          
 *          Common use cases:
 *          - Verify motor direction (CW vs CCW)
 *          - Confirm motor-to-output mapping
 *          - Test after ESC calibration
 *          - Verify propeller installation
 *          - Diagnose motor/ESC problems
 *          - Check motor-to-motor consistency
 *          
 *          Test procedure recommendations:
 *          1. Remove propellers for initial tests
 *          2. Secure vehicle to prevent movement
 *          3. Start with low throttle (5-10%)
 *          4. Verify each motor spins correct direction
 *          5. Check motor numbers match diagram
 *          6. Install propellers (correct orientation)
 *          7. Retest with propellers at low throttle
 *          8. Verify thrust direction and magnitude
 *          
 *          Safety features:
 *          - Automatic timeout prevents runaway
 *          - Safety checks prevent armed testing
 *          - Throttle limits enforced
 *          - Emergency stop via mode change
 *          - Logs test events for review
 *          
 *          Failure reasons:
 *          - Vehicle armed (motors already controllable)
 *          - Vehicle not on ground (in flight)
 *          - Invalid motor number (exceeds configured motors)
 *          - Invalid throttle type
 *          - Failsafe active
 *          - Pre-arm checks failed
 *          
 *          ESC calibration workflow:
 *          - Often performed before motor test
 *          - Set MOT_PWM_MIN and MOT_PWM_MAX
 *          - Use PWM throttle mode to verify calibration
 *          - Ensure motors start at same PWM value
 *          - Adjust MOT_SPIN_MIN if needed
 * 
 * @param[in] packet COMMAND_INT with motor test parameters
 * 
 * @return MAV_RESULT_ACCEPTED if motor test started successfully
 * @return MAV_RESULT_DENIED if safety checks failed
 * @return MAV_RESULT_FAILED if invalid parameters or test failed to start
 * 
 * @note Vehicle must be disarmed and on ground
 * @note Propeller removal recommended for initial tests
 * @note Motor numbers are frame-specific (see documentation)
 * @note Test stops automatically after timeout
 * 
 * @warning Keep hands and objects clear of propellers
 * @warning Secure vehicle to prevent movement during test
 * @warning Use low throttle values (5-10%) for safety
 * @warning Verify motor direction before installing propellers
 * 
 * @see Copter::mavlink_motor_test_start() for test implementation
 * @see AP_Motors for motor control and mixing
 * @see MOT_SPIN_MIN, MOT_PWM_MIN, MOT_PWM_MAX parameters
 */
MAV_RESULT GCS_MAVLINK_Copter::handle_MAV_CMD_DO_MOTOR_TEST(const mavlink_command_int_t &packet)
{
        // Motor test parameters:
        // param1: Motor sequence number (1 to max motors)
        // param2: Throttle type (0=percentage, 1=PWM, 2=pilot pass-through)
        // param3: Throttle value (interpretation depends on throttle type)
        // param4: Timeout duration in seconds (motor stops after this time)
        // param5 (packet.x): Number of motors to test in sequence
        // param6: Motor test order (reserved for future use)
        
        // Delegate to motor test implementation which performs safety checks
        return copter.mavlink_motor_test_start(*this,
                                               (uint8_t)packet.param1,   // motor sequence number
                                               (uint8_t)packet.param2,   // throttle type
                                               packet.param3,            // throttle value
                                               packet.param4,            // timeout in seconds
                                               (uint8_t)packet.x);       // number of motors in sequence
}

#if AP_WINCH_ENABLED
/**
 * @brief Handle MAV_CMD_DO_WINCH to control winch system
 * 
 * @details Controls winch for payload delivery, cargo hook, or tether operations.
 *          Supports relax (free-spool), position control (release/retract specific
 *          length), and rate control (continuous reel in/out). Essential for delivery
 *          drones, search and rescue, and cargo operations. Provides both absolute
 *          positioning and velocity control modes for different operational needs.
 *          
 *          Winch system overview:
 *          - Electric motor-driven cable reel
 *          - Encoder for position feedback (optional)
 *          - Clutch mechanism for manual release
 *          - Load cell for tension measurement (optional)
 *          - Typically mounted underneath vehicle
 *          - Cable length up to 100+ meters
 *          
 *          MAV_CMD_DO_WINCH parameters:
 *          - param1: Winch instance number
 *            * Currently ignored (single winch support)
 *            * Reserved for future multi-winch vehicles
 *          - param2: Winch action (WINCH_ACTIONS enum)
 *            * 0 = WINCH_RELAXED: Disable motor, allow free-spool
 *            * 1 = WINCH_RELATIVE_LENGTH_CONTROL: Release/retract by distance
 *            * 2 = WINCH_RATE_CONTROL: Continuous reel at specified rate
 *          - param3: Length to release (meters) - used with action 1
 *            * Positive: Release cable (lower payload)
 *            * Negative: Retract cable (raise payload)
 *            * Relative to current position
 *          - param4: Release rate (m/s) - used with action 2
 *            * Positive: Release cable (reel out)
 *            * Negative: Retract cable (reel in)
 *            * Continues until next command
 *          
 *          WINCH_RELAXED (0) - Free-spool mode:
 *          - Disables winch motor
 *          - Allows manual cable extraction
 *          - Clutch releases tension
 *          - Used for ground setup
 *          - Used for emergency manual control
 *          - Cable may unspool under payload weight
 *          
 *          WINCH_RELATIVE_LENGTH_CONTROL (1) - Position mode:
 *          - Release or retract specific cable length
 *          - Relative to current position
 *          - Uses encoder feedback for accuracy
 *          - Stops automatically at target length
 *          - Ideal for precise payload placement
 *          - param3 specifies distance (positive=release)
 *          
 *          WINCH_RATE_CONTROL (2) - Velocity mode:
 *          - Continuous cable movement at constant rate
 *          - Continues indefinitely until stopped
 *          - Useful for gradual delivery
 *          - Operator monitors and stops manually
 *          - param4 specifies rate (positive=release)
 *          - Stop with rate=0 or different action
 *          
 *          Configuration parameters:
 *          - WINCH_ENABLE: Enable winch system (1=enabled)
 *          - WINCH_TYPE: Winch hardware type
 *          - WINCH_RATE_MAX: Maximum release/retract rate (m/s)
 *          - WINCH_POS_P: Position controller P gain
 *          - WINCH_OPTIONS: Feature flags
 *          
 *          Position control behavior:
 *          - Target position = current + param3
 *          - Rate limited by WINCH_RATE_MAX
 *          - Encoder tracks actual cable position
 *          - Stops when target reached
 *          - Holds position after reaching target
 *          - Multiple commands can be chained
 *          
 *          Rate control behavior:
 *          - Constant velocity until changed
 *          - No automatic stop
 *          - Rate clamped to ±WINCH_RATE_MAX
 *          - Rate=0 holds current position
 *          - Override with new rate or position command
 *          
 *          Typical delivery sequence:
 *          1. Fly to delivery location
 *          2. Hover at delivery altitude
 *          3. WINCH_RELAXED to prepare
 *          4. WINCH_RATE_CONTROL at moderate rate
 *          5. Monitor cable tension and length
 *          6. WINCH_RATE_CONTROL at 0 when payload landed
 *          7. Release payload (external mechanism)
 *          8. WINCH_RATE_CONTROL negative to retract
 *          9. Fly away when retracted
 *          
 *          Safety considerations:
 *          - Cable can snag on obstacles
 *          - Payload swing affects vehicle stability
 *          - Excessive cable length risks entanglement
 *          - Winch motor can stall under load
 *          - Free-spool risks uncontrolled release
 *          - Monitor tension to prevent damage
 *          
 *          Failure modes:
 *          - Winch not enabled (WINCH_ENABLE=0)
 *          - Invalid action type
 *          - Cable jam or snag
 *          - Motor overload
 *          - Encoder failure (position control only)
 *          - Power supply issues
 *          
 *          Integration with flight modes:
 *          - GUIDED: Winch control during guided flight
 *          - AUTO: Mission commands can include DO_WINCH
 *          - LOITER: Manual winch control while hovering
 *          - Position hold while winch operating
 *          - Attitude compensation for payload weight
 *          
 *          Encoder feedback (if equipped):
 *          - Tracks cable length precisely
 *          - Enables accurate position control
 *          - Detects cable slip or jam
 *          - Optional but recommended
 *          - Calibrate before first use
 *          
 *          Applications:
 *          - Package delivery
 *          - Search and rescue (lifting person)
 *          - Payload drop in difficult terrain
 *          - Sensor lowering (underwater, caves)
 *          - Cargo hook operations
 *          - Tethered operations (power, data)
 * 
 * @param[in] packet COMMAND_INT with winch action and parameters
 * 
 * @return MAV_RESULT_ACCEPTED if winch command executed successfully
 * @return MAV_RESULT_FAILED if winch disabled or invalid action
 * 
 * @note Only compiled if AP_WINCH_ENABLED is set
 * @note Winch must be enabled via WINCH_ENABLE parameter
 * @note param1 (instance) currently ignored (single winch only)
 * @note Rate and length limits enforced by winch system
 * 
 * @warning Cable entanglement can cause vehicle instability
 * @warning Monitor payload swing during winch operations
 * @warning Free-spool mode allows uncontrolled cable release
 * 
 * @see AP_Winch for winch control implementation
 * @see WINCH_* parameters for configuration
 */
MAV_RESULT GCS_MAVLINK_Copter::handle_MAV_CMD_DO_WINCH(const mavlink_command_int_t &packet)
{
        // Winch command parameters:
        // param1: Winch instance number (currently ignored, single winch only)
        // param2: Winch action (WINCH_ACTIONS enum)
        //         0=RELAXED, 1=RELATIVE_LENGTH_CONTROL, 2=RATE_CONTROL
        
        // Check if winch system is enabled
        if (!copter.g2.winch.enabled()) {
            return MAV_RESULT_FAILED;
        }
        
        // Execute requested winch action
        switch ((uint8_t)packet.param2) {
        case WINCH_RELAXED:
            // Disable winch motor, allow free-spool (manual cable extraction)
            copter.g2.winch.relax();
            return MAV_RESULT_ACCEPTED;
            
        case WINCH_RELATIVE_LENGTH_CONTROL: {
            // Release or retract cable by specified length
            // param3: Length in meters (positive=release, negative=retract)
            copter.g2.winch.release_length(packet.param3);
            return MAV_RESULT_ACCEPTED;
        }
        
        case WINCH_RATE_CONTROL:
            // Set continuous release/retract rate
            // param4: Rate in m/s (positive=release, negative=retract)
            copter.g2.winch.set_desired_rate(packet.param4);
            return MAV_RESULT_ACCEPTED;
            
        default:
            // Unknown winch action
            break;
        }
        
        // Invalid action type
        return MAV_RESULT_FAILED;
}
#endif  // AP_WINCH_ENABLED

#if AC_MAVLINK_SOLO_BUTTON_COMMAND_HANDLING_ENABLED
/**
 * @brief Handle MAV_CMD_SOLO_BTN_FLY_CLICK for 3DR Solo fly button click
 * 
 * @details Processes single click of the Fly button on 3DR Solo controller. Changes
 *          vehicle to position-hold mode (LOITER preferred, ALT_HOLD fallback) for
 *          stable hovering. Part of Solo's simplified button interface designed for
 *          consumer drone operation. Single click enters stable hover mode, allowing
 *          pilot to pause and reposition without full manual control.
 *          
 *          3DR Solo button interface:
 *          - Fly button: Primary flight control button
 *          - Single click: Enter hover mode (this function)
 *          - Long hold: Takeoff or land (separate handler)
 *          - Pause button: Stop/hold position (separate handler)
 *          - Simplified compared to traditional RC
 *          
 *          Button click behavior:
 *          - Single short press detected by Solo controller
 *          - Controller sends this MAVLink command
 *          - ArduPilot switches to position-hold mode
 *          - Vehicle holds current position automatically
 *          - User can reorient or prepare for next action
 *          
 *          Mode selection priority:
 *          1. LOITER mode (preferred)
 *             - GPS-based position hold
 *             - Maintains 3D position accurately
 *             - Compensates for wind drift
 *             - Requires GPS lock and EKF healthy
 *          2. ALT_HOLD mode (fallback)
 *             - Altitude hold without position lock
 *             - Used if LOITER unavailable (no GPS)
 *             - Maintains altitude but drifts horizontally
 *             - More basic but always available
 *          
 *          LOITER mode requirements:
 *          - Good GPS position estimate (3D fix)
 *          - EKF healthy and converged
 *          - Sufficient satellite count (6+)
 *          - HDOP acceptable (< 2.0 typically)
 *          - If any requirement fails, falls back to ALT_HOLD
 *          
 *          ALT_HOLD mode fallback:
 *          - Always available (no GPS needed)
 *          - Uses barometer for altitude hold
 *          - Pilot must manually correct horizontal drift
 *          - Suitable for indoor or GPS-denied operation
 *          - Less stable than LOITER in wind
 *          
 *          Failsafe interaction:
 *          - Command ignored if radio failsafe active
 *          - Returns ACCEPTED but performs no action
 *          - Prevents mode change during failsafe recovery
 *          - Safety measure to avoid mode confusion
 *          - Failsafe takes priority over button commands
 *          
 *          Solo user workflow:
 *          1. Arm and takeoff (Fly button hold)
 *          2. Fly manually or in Smart Shots
 *          3. Click Fly button to pause/hover
 *          4. Reposition controller or adjust camera
 *          5. Resume flight in current or new mode
 *          6. Land (Fly button hold when near ground)
 *          
 *          Smart Shots integration:
 *          - Solo's automated camera movements
 *          - Orbit, Cable Cam, Follow, etc.
 *          - Fly click pauses Smart Shot
 *          - Returns to manual hover control
 *          - Different handling than manual flight
 *          
 *          Comparison with other Solo commands:
 *          - FLY_CLICK: Enter hover mode (this function)
 *          - FLY_HOLD: Takeoff or land
 *          - PAUSE_CLICK: Emergency stop/hold
 *          - Each button press type has distinct behavior
 *          
 *          RC override capability:
 *          - Pilot can override with RC sticks anytime
 *          - Stick input takes precedence
 *          - Returns to LOITER when sticks centered
 *          - Provides manual recovery option
 *          
 *          Mode change logging:
 *          - Mode change logged with reason GCS_COMMAND
 *          - Telemetry reports new mode to controller
 *          - Solo app updates mode display
 *          - Log analysis shows button-triggered changes
 *          
 *          Safety considerations:
 *          - LOITER drift if GPS degrades
 *          - ALT_HOLD horizontal drift in wind
 *          - Failsafe overrides button commands
 *          - Always monitor vehicle after mode change
 *          - Maintain RC control readiness
 *          
 *          Solo controller context:
 *          - Consumer-friendly interface
 *          - Minimal buttons compared to RC
 *          - Tactile feedback on button press
 *          - Visual confirmation on controller screen
 *          - Designed for smartphone generation users
 *          
 *          Legacy compatibility:
 *          - 3DR Solo discontinued but still used
 *          - Commands maintained for existing fleet
 *          - Could be adapted for other button controllers
 *          - MAVLink standard allows other implementations
 * 
 * @param[in] packet COMMAND_INT from Solo controller (parameters unused)
 * 
 * @return MAV_RESULT_ACCEPTED in all cases (command processed)
 * 
 * @note Only compiled if AC_MAVLINK_SOLO_BUTTON_COMMAND_HANDLING_ENABLED
 * @note Command ignored if radio failsafe active
 * @note Always returns ACCEPTED even if mode change fails
 * @note LOITER attempted first, ALT_HOLD used as fallback
 * 
 * @see handle_MAV_CMD_SOLO_BTN_FLY_HOLD() for takeoff/land handling
 * @see handle_MAV_CMD_SOLO_BTN_PAUSE_CLICK() for pause button
 * @see Mode_Loiter for GPS position hold implementation
 * @see Mode_AltHold for altitude-only hold implementation
 */
MAV_RESULT GCS_MAVLINK_Copter::handle_MAV_CMD_SOLO_BTN_FLY_CLICK(const mavlink_command_int_t &packet)
{
        // Check if radio failsafe is active
        if (copter.failsafe.radio) {
            // During failsafe, ignore button commands to avoid mode confusion
            // Return accepted to acknowledge command receipt
            return MAV_RESULT_ACCEPTED;
        }

        // Attempt to set mode to LOITER (GPS-based position hold)
        if (!copter.set_mode(Mode::Number::LOITER, ModeReason::GCS_COMMAND)) {
            // LOITER failed (no GPS, EKF unhealthy, etc.)
            // Fall back to ALT_HOLD (altitude hold without position lock)
            copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::GCS_COMMAND);
        }
        // Note: Always returns ACCEPTED regardless of mode change success
        // Solo expects command acknowledgment even if fallback mode used
        return MAV_RESULT_ACCEPTED;
}

/**
 * @brief Handle MAV_CMD_SOLO_BTN_FLY_HOLD for 3DR Solo fly button hold
 * 
 * @details Processes long press (hold) of Fly button on 3DR Solo controller. Performs
 *          context-aware action based on vehicle state: arms if disarmed, takes off if
 *          armed and landed, or lands if flying. Provides intuitive single-button
 *          control of the complete flight cycle from arm through takeoff to landing.
 *          Simplifies operation for consumer users with state-machine behavior.
 *          
 *          State-dependent actions:
 *          1. Disarmed → Arm motors (prepare for flight)
 *          2. Armed + Landed → Takeoff (automated climb)
 *          3. Flying → Land (automated descent)
 *          
 *          Disarmed state handling:
 *          - Motors not spinning
 *          - Vehicle on ground and stable
 *          - Long press initiates arming sequence
 *          - Executes standard pre-arm checks
 *          - Arms via MAVLink method
 *          - No mode change or motion
 *          - User must press again to takeoff
 *          
 *          Armed + Landed state handling:
 *          - Motors spinning at idle
 *          - Vehicle on ground (land_complete flag set)
 *          - Long press initiates automated takeoff
 *          - Switches to LOITER mode first
 *          - Commands climb to altitude from param1
 *          - GPS-based position hold during climb
 *          - If LOITER unavailable, command ignored
 *          
 *          Flying state handling:
 *          - Vehicle airborne (land_complete flag clear)
 *          - Currently in any flight mode
 *          - Long press initiates automated landing
 *          - Switches to LAND mode
 *          - Descends vertically from current position
 *          - Detects landing and disarms automatically
 *          
 *          Takeoff sequence detail:
 *          1. Verify motors armed and landed
 *          2. Switch to LOITER mode (GPS position hold)
 *          3. Call do_user_takeoff() with target altitude
 *          4. Vehicle climbs at PILOT_SPEED_UP rate
 *          5. Maintains horizontal position via GPS
 *          6. Reaches target altitude and holds
 *          7. Ready for pilot input or mode change
 *          
 *          Takeoff altitude (param1):
 *          - Specified in meters (MAVLink standard)
 *          - Converted to centimeters internally (×100)
 *          - Typical Solo default: 2.5 meters
 *          - Altitude relative to takeoff location
 *          - Limited by geofence altitude if enabled
 *          - Reasonable range: 1-5 meters for safety
 *          
 *          Landing sequence:
 *          - Switch to LAND mode from any flight mode
 *          - Vehicle descends vertically
 *          - Uses current horizontal position
 *          - Rate controlled by LAND_SPEED parameter
 *          - Slows near ground (rangefinder if equipped)
 *          - Detects ground contact
 *          - Disarms automatically after landing
 *          
 *          Failsafe interaction:
 *          - Command ignored if radio failsafe active
 *          - Returns ACCEPTED but performs no action
 *          - Prevents arming/takeoff during failsafe
 *          - Safety measure for controlled recovery
 *          - Failsafe takes priority over button
 *          
 *          Arming checks (disarmed state):
 *          - Standard pre-arm validation
 *          - GPS lock (if required by parameters)
 *          - EKF healthy
 *          - Battery voltage adequate
 *          - Accelerometer calibrated
 *          - Compass calibrated and healthy
 *          - RC calibration valid
 *          - No active failsafes
 *          - Controlled by ARMING_CHECK parameter
 *          
 *          LOITER mode requirement for takeoff:
 *          - Takeoff only attempted if LOITER available
 *          - Requires GPS 3D fix and healthy EKF
 *          - If LOITER unavailable, command acknowledged but no takeoff
 *          - Prevents takeoff without position hold capability
 *          - Safety feature to avoid uncontrolled flight
 *          
 *          Solo user experience:
 *          1. Place vehicle on ground
 *          2. Hold Fly button → Arms
 *          3. Hold Fly button again → Takeoff
 *          4. Vehicle climbs to preset altitude
 *          5. Fly manually or use Smart Shots
 *          6. Hold Fly button → Lands
 *          7. Vehicle descends and disarms
 *          
 *          Comparison with other Solo commands:
 *          - FLY_HOLD: Arm/takeoff/land (this function)
 *          - FLY_CLICK: Enter hover mode
 *          - PAUSE_CLICK: Emergency stop
 *          - Each provides different control aspect
 *          
 *          Smart Shots compatibility:
 *          - Can land from any Smart Shot mode
 *          - Overrides automated camera movements
 *          - Provides emergency landing capability
 *          - Solo app may warn about interrupting shot
 *          
 *          Safety features:
 *          - State-aware (won't takeoff while flying)
 *          - Requires good GPS for takeoff
 *          - Standard arming checks enforced
 *          - Failsafe overrides button commands
 *          - Always returns ACCEPTED (no retry needed)
 *          
 *          Logging and telemetry:
 *          - Arming logged with Method::MAVLINK
 *          - Mode changes logged with ModeReason::GCS_COMMAND
 *          - Takeoff event logged
 *          - Landing detection logged
 *          - Solo app updated via telemetry
 *          
 *          Consumer design philosophy:
 *          - Single button for full flight cycle
 *          - Context-aware intelligent behavior
 *          - No need to select modes manually
 *          - Reduces operator workload
 *          - Lowers barrier to entry for new pilots
 *          
 *          Parameter interactions:
 *          - PILOT_SPEED_UP: Takeoff climb rate
 *          - LAND_SPEED: Landing descent rate
 *          - ARMING_CHECK: Pre-arm validation level
 *          - FENCE_ALT_MAX: Maximum takeoff altitude
 *          - All standard copter parameters apply
 * 
 * @param[in] packet COMMAND_INT with takeoff altitude in param1 (meters)
 * 
 * @return MAV_RESULT_ACCEPTED in all cases (command processed)
 * 
 * @note Only compiled if AC_MAVLINK_SOLO_BUTTON_COMMAND_HANDLING_ENABLED
 * @note Command ignored if radio failsafe active
 * @note Always returns ACCEPTED regardless of action success
 * @note Takeoff requires LOITER mode (GPS) availability
 * @note param1 specifies takeoff altitude in meters
 * 
 * @warning Vehicle may arm and takeoff automatically
 * @warning Ensure safe clearance before holding button
 * @warning GPS lock required for automated takeoff
 * 
 * @see handle_MAV_CMD_SOLO_BTN_FLY_CLICK() for hover command
 * @see Mode_Loiter for GPS position hold during takeoff
 * @see Mode_Land for automated landing
 * @see AP_Arming::arm() for arming implementation
 */
MAV_RESULT GCS_MAVLINK_Copter::handle_MAV_CMD_SOLO_BTN_FLY_HOLD(const mavlink_command_int_t &packet)
{
        // Check if radio failsafe is active
        if (copter.failsafe.radio) {
            // During failsafe, ignore button commands to prevent unsafe actions
            return MAV_RESULT_ACCEPTED;
        }

        // Determine action based on vehicle state
        if (!copter.motors->armed()) {
            // State 1: Disarmed → Arm motors
            // Executes standard pre-arm checks before allowing arming
            copter.arming.arm(AP_Arming::Method::MAVLINK);
            
        } else if (copter.ap.land_complete) {
            // State 2: Armed + Landed → Takeoff
            // Vehicle is armed but on ground, initiate automated takeoff
            
            // First switch to LOITER mode for GPS-based position hold during climb
            if (copter.set_mode(Mode::Number::LOITER, ModeReason::GCS_COMMAND)) {
                // LOITER mode active, execute takeoff
                // param1 contains target altitude in meters, convert to centimeters
                // Second parameter (true) indicates guided takeoff
                copter.flightmode->do_user_takeoff(packet.param1*100, true);
            }
            // If LOITER unavailable (no GPS), command acknowledged but no takeoff
            
        } else {
            // State 3: Flying → Land
            // Vehicle is airborne, initiate automated landing
            copter.set_mode(Mode::Number::LAND, ModeReason::GCS_COMMAND);
        }
        
        // Always return ACCEPTED to acknowledge command receipt
        // Solo expects acknowledgment regardless of action success
        return MAV_RESULT_ACCEPTED;
}

/**
 * @brief Handle MAV_CMD_SOLO_BTN_PAUSE_CLICK for 3DR Solo pause button
 * 
 * @details Processes click of Pause button on 3DR Solo controller. Performs emergency
 *          stop or pause action based on vehicle state: disarms if armed and landed,
 *          or immediately stops motion if flying. During Solo Smart Shots (automated
 *          camera movements), delegates pause handling to SoloLink companion computer.
 *          Provides quick-stop capability for safety and operational control.
 *          
 *          State-dependent pause actions:
 *          1. Disarmed → No action (motors already stopped)
 *          2. Armed + Landed → Disarm motors (safe shutdown)
 *          3. Flying (manual) → Emergency stop/hover (BRAKE or ALT_HOLD)
 *          4. Flying (Smart Shot) → Delegate to SoloLink (shot-specific pause)
 *          
 *          Armed + Landed state:
 *          - Motors spinning at idle on ground
 *          - land_complete flag indicates vehicle landed
 *          - Pause button triggers immediate disarm
 *          - Uses SOLOPAUSEWHENLANDED disarm method
 *          - Stops motor spin for safety
 *          - Appropriate when takeoff aborted or after landing
 *          
 *          Flying (manual flight) state:
 *          - Vehicle airborne in non-Smart-Shot mode
 *          - Pause button triggers emergency stop
 *          - Prefers BRAKE mode if available
 *          - Falls back to ALT_HOLD if BRAKE disabled
 *          - Stops horizontal motion immediately
 *          - Maintains altitude hold
 *          
 *          BRAKE mode (preferred):
 *          - Rapidly decelerates to zero velocity
 *          - Uses maximum deceleration rates
 *          - Holds position for 2.5 seconds
 *          - Automatically transitions to LOITER
 *          - Most responsive emergency stop
 *          - Requires GPS for position hold after stop
 *          
 *          BRAKE mode sequence:
 *          1. Switch to BRAKE mode
 *          2. Apply maximum deceleration
 *          3. Reach zero velocity quickly
 *          4. Hold position for 2500ms
 *          5. Transition to LOITER (GPS position hold)
 *          6. Resume normal control
 *          
 *          ALT_HOLD fallback mode:
 *          - Used if BRAKE mode not compiled/available
 *          - Holds altitude only (no position lock)
 *          - Pilot stick input stops horizontal motion
 *          - Less aggressive stop than BRAKE
 *          - No GPS required
 *          - Vehicle may drift horizontally
 *          
 *          Smart Shot mode detection:
 *          - param1 non-zero indicates Smart Shot active
 *          - Mode is GUIDED or GUIDED_NOGPS
 *          - Solo app controls automated camera movements
 *          - SoloLink companion computer manages shot
 *          - ArduPilot delegates pause to SoloLink
 *          - Shot-specific pause behavior preserved
 *          
 *          Smart Shots overview:
 *          - Orbit: Circular path around subject
 *          - Cable Cam: Point-to-point automated flight
 *          - Follow: Track moving subject
 *          - Selfie: Pull away and return
 *          - All use GUIDED mode with SoloLink control
 *          - Pause may stop motion or freeze frame
 *          
 *          SoloLink delegation (Smart Shots):
 *          - Command acknowledged but no mode change
 *          - SoloLink receives pause notification
 *          - Shot pauses per its own logic
 *          - May freeze position mid-shot
 *          - User can adjust camera or reframe
 *          - Resume continues shot from pause point
 *          
 *          Failsafe interaction:
 *          - Command ignored if radio failsafe active
 *          - Returns ACCEPTED but no action taken
 *          - Failsafe recovery takes priority
 *          - Prevents mode confusion during failsafe
 *          - Button press acknowledged to controller
 *          
 *          Emergency stop use cases:
 *          - Unexpected obstacle detected
 *          - Loss of visual contact with vehicle
 *          - Wind gust pushing vehicle off course
 *          - Need to quickly halt motion
 *          - Pre-landing position adjustment
 *          - Abort takeoff after arm
 *          
 *          Safety features:
 *          - Immediate response (high priority)
 *          - Aggressive deceleration (BRAKE mode)
 *          - Maintains altitude during stop
 *          - Transitions to stable hold mode
 *          - Works in GPS or non-GPS conditions
 *          
 *          Mode priority for emergency stop:
 *          1. BRAKE (if enabled): Fast stop → LOITER
 *          2. ALT_HOLD (fallback): Altitude hold only
 *          3. Smart Shot: Delegate to SoloLink
 *          
 *          Comparison with other Solo commands:
 *          - PAUSE_CLICK: Emergency stop (this function)
 *          - FLY_CLICK: Enter hover mode (less urgent)
 *          - FLY_HOLD: Takeoff/land (full automation)
 *          - Different urgency and behavior levels
 *          
 *          Disarm method significance:
 *          - Method::SOLOPAUSEWHENLANDED specifically used
 *          - Distinguishes from manual disarm
 *          - Logged for post-flight analysis
 *          - May have different safety checks
 *          - Appropriate for automated pause action
 *          
 *          BRAKE timeout behavior:
 *          - 2500ms (2.5 seconds) hold period
 *          - Prevents immediate drift after stop
 *          - Gives pilot time to assess situation
 *          - Automatic transition prevents indefinite brake
 *          - LOITER provides long-term position hold
 *          
 *          User experience workflow:
 *          1. Flying vehicle manually or in Smart Shot
 *          2. Need to stop immediately
 *          3. Press Pause button
 *          4. Vehicle stops motion rapidly (BRAKE)
 *          5. Holds position for 2.5 seconds
 *          6. Transitions to LOITER hold
 *          7. Pilot can reposition or land
 *          
 *          Solo controller feedback:
 *          - Tactile button press confirmation
 *          - App shows mode change
 *          - Telemetry updates status
 *          - Visual confirmation of stop
 *          
 *          Configuration parameters:
 *          - MODE_BRAKE_ENABLED: Compile-time brake mode
 *          - All standard copter brake/hold parameters
 *          - BRAKE_SPEED_DECAY affects stop rate
 *          
 * @param[in] packet COMMAND_INT with shot mode flag in param1
 * 
 * @return MAV_RESULT_ACCEPTED in all cases (command processed)
 * 
 * @note Only compiled if AC_MAVLINK_SOLO_BUTTON_COMMAND_HANDLING_ENABLED
 * @note Command ignored if radio failsafe active
 * @note Smart Shots (param1≠0) delegated to SoloLink
 * @note BRAKE mode transitions to LOITER after 2.5 seconds
 * @note Always returns ACCEPTED regardless of action taken
 * 
 * @warning Aggressive deceleration in BRAKE mode
 * @warning ALT_HOLD fallback allows horizontal drift
 * @warning Smart Shot pause behavior controlled by SoloLink
 * 
 * @see Mode_Brake for rapid deceleration implementation
 * @see Mode_AltHold for altitude-only hold
 * @see AP_Arming::disarm() for motor shutdown
 */
MAV_RESULT GCS_MAVLINK_Copter::handle_MAV_CMD_SOLO_BTN_PAUSE_CLICK(const mavlink_command_int_t &packet)
{
        // Check if radio failsafe is active
        if (copter.failsafe.radio) {
            // During failsafe, ignore button commands
            return MAV_RESULT_ACCEPTED;
        }

        // Only process if motors are armed
        if (copter.motors->armed()) {
            if (copter.ap.land_complete) {
                // State: Armed but landed on ground
                // Action: Disarm motors for safety (abort takeoff or post-landing)
                copter.arming.disarm(AP_Arming::Method::SOLOPAUSEWHENLANDED);
                
            } else {
                // State: Flying (airborne)
                // Determine if in Smart Shot mode or manual flight
                
                // Smart Shot detection: param1 non-zero AND in GUIDED modes
                // Assumes all Smart Shots use GUIDED or GUIDED_NOGPS mode
                bool shot_mode = (!is_zero(packet.param1) && 
                                 (copter.flightmode->mode_number() == Mode::Number::GUIDED || 
                                  copter.flightmode->mode_number() == Mode::Number::GUIDED_NOGPS));

                if (!shot_mode) {
                    // Manual flight mode: Execute emergency stop
                    
#if MODE_BRAKE_ENABLED
                    // Preferred: Use BRAKE mode for rapid deceleration
                    if (copter.set_mode(Mode::Number::BRAKE, ModeReason::GCS_COMMAND)) {
                        // BRAKE mode active: Set timeout to transition to LOITER
                        // After 2500ms (2.5 seconds), automatically switch to LOITER
                        // This prevents indefinite brake and provides stable position hold
                        copter.mode_brake.timeout_to_loiter_ms(2500);
                    } else {
                        // BRAKE mode failed (no GPS): Fall back to ALT_HOLD
                        copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::GCS_COMMAND);
                    }
#else
                    // BRAKE mode not compiled: Use ALT_HOLD fallback
                    copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::GCS_COMMAND);
#endif
                } else {
                    // Smart Shot mode active
                    // SoloLink (companion computer) is expected to handle pause
                    // No mode change here - SoloLink manages shot-specific pause behavior
                    // This allows different pause actions for different Smart Shots
                }
            }
        }
        // Note: If disarmed, command acknowledged but no action taken
        
        // Always return ACCEPTED to acknowledge command receipt
        return MAV_RESULT_ACCEPTED;
}
#endif  // AC_MAVLINK_SOLO_BUTTON_COMMAND_HANDLING_ENABLED

/**
 * @brief Handle MAV_CMD_DO_PAUSE_CONTINUE command for mission pause/resume
 * 
 * @details Processes generic MAVLink command to pause or resume the current flight mode.
 *          Unlike Solo-specific buttons, this is a standard MAVLink command supported
 *          by all ground control stations. Delegates pause/resume action to the active
 *          flight mode, allowing mode-specific behavior. AUTO mode pauses mission
 *          execution, GUIDED mode may pause trajectory following, other modes may
 *          not support pause. Provides GCS control over mission execution without
 *          changing flight modes.
 *          
 *          Command parameters:
 *          - param1 = 0: Request pause
 *          - param1 = 1: Request resume
 *          - param1 = other: Invalid (command denied)
 *          
 *          Pause action (param1 = 0):
 *          - Calls active mode's pause() method
 *          - Mode-specific pause behavior invoked
 *          - AUTO: Pauses mission at current waypoint
 *          - GUIDED: May pause trajectory following
 *          - RTL: May pause return sequence
 *          - Other modes: May not support pause
 *          
 *          AUTO mode pause behavior:
 *          - Mission execution halted
 *          - Vehicle holds current position
 *          - Mission index preserved
 *          - Waypoint progress saved
 *          - Do-commands not executed during pause
 *          - Vehicle maintains altitude and position
 *          - Ready to resume from pause point
 *          
 *          Resume action (param1 = 1):
 *          - Calls active mode's resume() method
 *          - Mode-specific resume behavior invoked
 *          - AUTO: Continues mission from pause point
 *          - GUIDED: Resumes trajectory following
 *          - Returns to pre-pause behavior
 *          
 *          AUTO mode resume behavior:
 *          - Mission execution continues
 *          - Resumes from saved mission index
 *          - Vehicle navigates to next waypoint
 *          - Do-commands execute as encountered
 *          - Normal mission progression restored
 *          
 *          Mode support for pause/resume:
 *          - AUTO: Full support (mission pause/resume)
 *          - GUIDED: Limited support (trajectory pause)
 *          - RTL: May support pause of return
 *          - LOITER: No pause needed (already holding)
 *          - LAND: Typically doesn't support pause (safety)
 *          - STABILIZE/ALT_HOLD: No pause concept
 *          
 *          Success vs failure handling:
 *          - Success: Mode supports and accepts pause/resume
 *          - Failure: Mode doesn't support operation
 *          - Denied: Invalid param1 value
 *          - Informative text sent to GCS on failure
 *          
 *          Error messages to GCS:
 *          - "Failed to pause": Mode rejected pause request
 *          - "Failed to resume": Mode rejected resume request
 *          - Displayed in ground station UI
 *          - Helps operator understand capability
 *          - Logged for post-flight analysis
 *          
 *          Return value meanings:
 *          - MAV_RESULT_ACCEPTED: Operation successful
 *          - MAV_RESULT_FAILED: Mode doesn't support operation
 *          - MAV_RESULT_DENIED: Invalid command parameters
 *          
 *          GCS use cases:
 *          - Pause mission to assess situation
 *          - Hold for obstacle clearance
 *          - Wait for better lighting (photography)
 *          - Coordinate with other vehicles
 *          - Adjust mission parameters mid-flight
 *          - Resume after resolving issue
 *          
 *          Difference from Solo pause button:
 *          - Generic MAVLink standard command
 *          - Supported by all GCS software
 *          - Delegates to mode (mode-specific)
 *          - Reports success/failure explicitly
 *          - Solo button: Hardware-specific, emergency stop
 *          - This command: Mission control, graceful pause
 *          
 *          Mission planning workflow:
 *          1. Upload mission with multiple waypoints
 *          2. Start mission execution (AUTO mode)
 *          3. Vehicle navigates waypoints autonomously
 *          4. Operator sees need to pause
 *          5. Send pause command (param1=0)
 *          6. Vehicle holds position at current location
 *          7. Operator adjusts plans or waits
 *          8. Send resume command (param1=1)
 *          9. Vehicle continues to next waypoint
 *          
 *          Pause state persistence:
 *          - Pause flag saved in mode state
 *          - Survives short telemetry interruptions
 *          - Cleared only by resume command or mode change
 *          - Mission index preserved during pause
 *          
 *          Mode change during pause:
 *          - Changing mode clears pause state
 *          - Returning to AUTO doesn't auto-resume
 *          - Mission restarts from beginning (default)
 *          - Or resumes from saved index (parameter dependent)
 *          
 *          Safety considerations:
 *          - Pause maintains altitude and position
 *          - Battery continues draining during pause
 *          - Monitor battery level during long pauses
 *          - Failsafes remain active during pause
 *          - Geofence still enforced
 *          
 *          Telemetry during pause:
 *          - Mission state reported as PAUSED
 *          - Current waypoint index maintained
 *          - Target waypoint shown in telemetry
 *          - Distance to next waypoint static
 *          
 *          Parameter validation:
 *          - Only param1 values 0 and 1 valid
 *          - Cast to uint8_t for comparison
 *          - Other values explicitly denied
 *          - Clear command semantics enforced
 *          
 *          Comparison with other pause mechanisms:
 *          - This command: GCS-initiated, mode-specific
 *          - Solo button: Hardware-initiated, emergency
 *          - RC switch: Pilot-initiated, mode change
 *          - Failsafe: Automatic, safety override
 *          
 * @param[in] packet COMMAND_INT with action in param1 (0=pause, 1=resume)
 * 
 * @return MAV_RESULT_ACCEPTED if mode supports and accepts action
 * @return MAV_RESULT_FAILED if mode doesn't support pause/resume
 * @return MAV_RESULT_DENIED if param1 invalid (not 0 or 1)
 * 
 * @note Pause/resume support varies by flight mode
 * @note AUTO mode fully supports mission pause/resume
 * @note GCS receives text message on failure explaining issue
 * @note Vehicle maintains position during pause
 * 
 * @warning Battery drains during extended pause
 * @warning Mode change clears pause state
 * @warning Not all modes support pause/resume
 * 
 * @see Mode::pause() for mode-specific pause implementation
 * @see Mode::resume() for mode-specific resume implementation
 * @see mission_state() for PAUSED telemetry reporting
 */
MAV_RESULT GCS_MAVLINK_Copter::handle_command_pause_continue(const mavlink_command_int_t &packet)
{
    // Requested pause action
    if ((uint8_t) packet.param1 == 0) {
        // Delegate pause to active flight mode
        if (copter.flightmode->pause()) {
            // Mode successfully paused
            return MAV_RESULT_ACCEPTED;
        }
        // Mode doesn't support pause or pause failed
        send_text(MAV_SEVERITY_INFO, "Failed to pause");
        return MAV_RESULT_FAILED;
    }

    // Requested resume action
    if ((uint8_t) packet.param1 == 1) {
        // Delegate resume to active flight mode
        if (copter.flightmode->resume()) {
            // Mode successfully resumed
            return MAV_RESULT_ACCEPTED;
        }
        // Mode doesn't support resume or resume failed
        send_text(MAV_SEVERITY_INFO, "Failed to resume");
        return MAV_RESULT_FAILED;
    }
    
    // Invalid param1 value (not 0 or 1)
    return MAV_RESULT_DENIED;
}

#if HAL_MOUNT_ENABLED
/**
 * @brief Handle MOUNT_CONTROL MAVLink message for gimbal/camera control
 * 
 * @details Processes MOUNT_CONTROL messages for camera gimbal control with special
 *          handling for mounts without pan (yaw) capability. When gimbal lacks pan
 *          control but yaw targeting is commanded, rotates entire vehicle instead
 *          to point camera at target. This provides functional yaw targeting even
 *          with 2-axis gimbals (roll/pitch only). Delegates full message processing
 *          to base class after handling copter-specific vehicle yaw compensation.
 *          
 *          Message: MAVLINK_MSG_ID_MOUNT_CONTROL
 *          - input_a: pitch angle (centidegrees)
 *          - input_b: roll angle (centidegrees)
 *          - input_c: yaw angle (centidegrees, body frame)
 *          - save_position: save as neutral position flag
 *          
 *          Gimbal types and pan capability:
 *          - 3-axis gimbal: Roll, pitch, yaw control
 *          - 2-axis gimbal: Roll and pitch only (no yaw/pan)
 *          - Fixed mount: No stabilization (vehicle attitude only)
 *          
 *          Vehicle yaw compensation logic:
 *          Conditions for vehicle yaw control:
 *          1. Mount exists (not Type::None)
 *          2. Mount in MAVLink targeting mode
 *          3. Mount lacks pan/yaw control capability
 *          
 *          When all conditions met:
 *          - Yaw command redirected to vehicle
 *          - Entire copter rotates for camera pointing
 *          - Gimbal handles roll/pitch stabilization
 *          - Combined system achieves 3-axis targeting
 *          
 *          Mount modes:
 *          - RETRACT: Gimbal in retracted/stowed position
 *          - NEUTRAL: Gimbal at neutral angles
 *          - MAVLINK_TARGETING: GCS controls gimbal angles (this mode)
 *          - RC_TARGETING: RC transmitter controls gimbal
 *          - GPS_POINT: Gimbal points at GPS coordinate
 *          
 *          MAVLink targeting mode:
 *          - GCS sends desired gimbal angles
 *          - Real-time angle commands from ground station
 *          - Typically used for manual camera control
 *          - Operator controls camera view from GCS
 *          - Gimbal tracks commanded angles
 *          
 *          Yaw angle interpretation:
 *          - Angles in body frame (vehicle-relative)
 *          - input_c in centidegrees (0.01 degree units)
 *          - Converted to degrees for yaw offset
 *          - Body frame: 0° = forward, 90° = right, -90° = left
 *          - Offset added to current yaw setpoint
 *          
 *          Yaw offset vs absolute yaw:
 *          - Command specifies offset from current yaw
 *          - Not absolute heading command
 *          - Allows incremental yaw adjustments
 *          - Preserves existing yaw behavior in mode
 *          - Compatible with AUTO mode yaw control
 *          
 *          Auto yaw system integration:
 *          - Each mode has auto_yaw controller
 *          - Manages yaw behavior for mode
 *          - AUTO: yaw toward waypoints
 *          - GUIDED: yaw as commanded
 *          - LOITER: yaw as pilot commands
 *          - Offset added to mode's yaw target
 *          
 *          2-axis gimbal workflow:
 *          1. GCS sends MOUNT_CONTROL with yaw command
 *          2. ArduPilot detects no pan capability
 *          3. Extracts yaw angle from input_c field
 *          4. Converts centidegrees to degrees
 *          5. Sets yaw offset in active mode's auto_yaw
 *          6. Vehicle rotates to point camera
 *          7. Gimbal stabilizes roll/pitch
 *          8. Camera achieves desired orientation
 *          
 *          3-axis gimbal behavior:
 *          - has_pan_control() returns true
 *          - Vehicle yaw compensation skipped
 *          - All control delegated to gimbal
 *          - Vehicle maintains heading from mode
 *          - Gimbal independently controls yaw
 *          - More efficient (less vehicle motion)
 *          
 *          Use case example (2-axis gimbal):
 *          - Aerial photography with 2-axis gimbal
 *          - Pilot flies in AUTO mode toward waypoint
 *          - Photographer wants camera pointed right
 *          - Sends MOUNT_CONTROL with +90° yaw
 *          - Vehicle rotates 90° right
 *          - Auto_yaw maintains waypoint heading + 90° offset
 *          - Camera now points perpendicular to flight path
 *          - Gimbal stabilizes roll/pitch for level horizon
 *          
 *          Mount type detection:
 *          - Type::None: No mount configured
 *          - Type::Servo: Servo-driven gimbal
 *          - Type::SoloGimbal: 3DR Solo gimbal
 *          - Type::Alexmos: Alexmos/STorM32 controller
 *          - Type::SToRM32: SToRM32 gimbal controller
 *          - Type::Gremsy: Gremsy gimbal
 *          
 *          Coordinate frame considerations:
 *          - MOUNT_CONTROL angles in body frame
 *          - Body frame moves with vehicle
 *          - Yaw offset in body frame
 *          - Auto_yaw converts to earth frame internally
 *          - Maintains consistency across modes
 *          
 *          Mode-specific yaw behavior:
 *          - AUTO: Offset from waypoint heading
 *          - GUIDED: Offset from guided target heading
 *          - LOITER: Offset from pilot yaw input
 *          - RTL: Offset from return heading
 *          - Offset preserves mode's base yaw control
 *          
 *          Safety considerations:
 *          - Vehicle rotation for camera pointing
 *          - May conflict with navigation requirements
 *          - Pilot maintains override capability
 *          - Geofence still enforced
 *          - Failsafes remain active
 *          
 *          Limitations of vehicle yaw compensation:
 *          - Slower response than gimbal yaw
 *          - Affects entire vehicle orientation
 *          - May interfere with flight path
 *          - Less precise than dedicated gimbal yaw
 *          - Suitable for slow, deliberate movements
 *          
 *          Alternative: DO_MOUNT_CONTROL command:
 *          - MAV_CMD_DO_MOUNT_CONTROL is command version
 *          - Handled by handle_command_mount()
 *          - Similar vehicle yaw compensation logic
 *          - Command vs message interface difference
 *          
 *          Message delegation to base class:
 *          - GCS_MAVLINK::handle_mount_message() called
 *          - Base class routes to mount library
 *          - AP_Mount handles gimbal-specific processing
 *          - Roll/pitch commands processed normally
 *          - Mount library manages gimbal drivers
 *          
 *          Configuration parameters:
 *          - MNT_TYPE: Mount type selection
 *          - MNT_DEFLT_MODE: Default mount mode
 *          - MNT_RC_IN_*: RC input channels for mount control
 *          - MNT_ANGMIN/MAX_*: Angle limits per axis
 *          
 * @param[in] msg MAVLink message (MOUNT_CONTROL or other mount messages)
 * 
 * @note Only compiled if HAL_MOUNT_ENABLED
 * @note Vehicle yaw compensation only for 2-axis gimbals in MAVLink targeting mode
 * @note Yaw angle in body frame as offset to current heading
 * @note All messages delegated to base class after copter-specific handling
 * 
 * @warning Vehicle yaw rotation affects flight path
 * @warning May conflict with autonomous navigation in some modes
 * 
 * @see AP_Mount for gimbal control library
 * @see auto_yaw.set_yaw_angle_offset_deg() for yaw offset implementation
 * @see handle_command_mount() for command-based mount control
 */
void GCS_MAVLINK_Copter::handle_mount_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_MOUNT_CONTROL:
        // Check if vehicle yaw compensation needed for 2-axis gimbal
        if ((copter.camera_mount.get_mount_type() != AP_Mount::Type::None) &&
            (copter.camera_mount.get_mode() == MAV_MOUNT_MODE_MAVLINK_TARGETING) &&
            !copter.camera_mount.has_pan_control()) {
            
            // Gimbal lacks pan/yaw control - rotate vehicle instead
            // Extract yaw angle from MOUNT_CONTROL message
            // Per AP_Mount convention, MOUNT_CONTROL yaw angle is in body frame
            // Body frame angle is equivalent to offset from current yaw demand
            const float yaw_offset_deg = mavlink_msg_mount_control_get_input_c(&msg) * 0.01f;
            
            // Apply yaw offset to active mode's auto yaw controller
            // This rotates the vehicle to point the camera
            copter.flightmode->auto_yaw.set_yaw_angle_offset_deg(yaw_offset_deg);
            break;
        }
    }
    
    // Delegate all mount messages to base class for standard processing
    // Base class routes to AP_Mount library for gimbal control
    GCS_MAVLINK::handle_mount_message(msg);
}
#endif

/**
 * @brief Process MANUAL_CONTROL MAVLink message for joystick/gamepad input
 * 
 * @details Handles joystick input from ground control station or companion computer,
 *          allowing direct manual control via MAVLink instead of RC transmitter.
 *          Maps joystick axes to copter control channels (roll, pitch, yaw, throttle)
 *          and overrides RC input with joystick values. Commonly used for ArduSub
 *          joystick control, but also supports copter control for applications
 *          requiring computer-based piloting (autonomous agents, research platforms,
 *          accessibility devices). Validates thrust is non-negative (copters can't
 *          pull downward). Provides full manual control authority via MAVLink.
 *          
 *          MANUAL_CONTROL message structure:
 *          - x: pitch axis (-1000 to +1000, normalized int16)
 *          - y: roll axis (-1000 to +1000, normalized int16)
 *          - z: thrust axis (0 to +1000, normalized uint16)
 *          - r: yaw axis (-1000 to +1000, normalized int16)
 *          - buttons: Button bitmask (not used here)
 *          
 *          Axis mapping to copter controls:
 *          - packet.x (pitch) → copter.channel_pitch
 *          - packet.y (roll) → copter.channel_roll
 *          - packet.z (thrust) → copter.channel_throttle
 *          - packet.r (yaw) → copter.channel_yaw
 *          
 *          Note: Roll and pitch swapped from intuitive mapping
 *          - MAVLink x = pitch (forward/back joystick)
 *          - MAVLink y = roll (left/right joystick)
 *          - Matches joystick physical layout conventions
 *          
 *          PWM range mapping:
 *          Roll channel:
 *          - Input: -1000 to +1000 (MAVLink normalized)
 *          - Output: 1000 to 2000 μs (PWM range)
 *          - Center: 0 → 1500 μs
 *          - Full left: -1000 → 1000 μs
 *          - Full right: +1000 → 2000 μs
 *          
 *          Pitch channel:
 *          - Input: -1000 to +1000 (MAVLink normalized)
 *          - Output: 1000 to 2000 μs (PWM range)
 *          - Center: 0 → 1500 μs
 *          - Full forward: +1000 → 2000 μs
 *          - Full back: -1000 → 1000 μs
 *          - Reversed flag: true (pitch typically reversed)
 *          
 *          Throttle channel:
 *          - Input: 0 to +1000 (MAVLink normalized, no negative)
 *          - Output: 0 to 1000 μs (non-standard PWM range)
 *          - Zero throttle: 0 → 0 μs (special case)
 *          - Full throttle: +1000 → 1000 μs
 *          - Note: Different range than roll/pitch/yaw
 *          
 *          Yaw channel:
 *          - Input: -1000 to +1000 (MAVLink normalized)
 *          - Output: 1000 to 2000 μs (PWM range)
 *          - Center: 0 → 1500 μs
 *          - Full left: -1000 → 1000 μs
 *          - Full right: +1000 → 2000 μs
 *          
 *          Negative thrust rejection:
 *          - Copters can only push downward (positive thrust)
 *          - Negative thrust physically impossible
 *          - packet.z < 0 indicates invalid input
 *          - Command silently ignored if negative
 *          - Prevents flight controller confusion
 *          - Safety feature for copter-specific physics
 *          
 *          Contrast with planes/rovers:
 *          - Planes: Negative thrust = reverse thrust (some models)
 *          - Rovers: Negative throttle = reverse drive
 *          - Subs: Negative thrust = pull down (buoyant vehicle)
 *          - Copters: Only positive thrust valid
 *          
 *          Manual override mechanism:
 *          - Temporarily replaces RC input with MAVLink values
 *          - Duration controlled by tnow timestamp
 *          - Override expires if no new messages received
 *          - RC input automatically restored after timeout
 *          - Prevents runaway if MAVLink connection lost
 *          
 *          Override timeout behavior:
 *          - Timestamp tnow marks override start time
 *          - Base class enforces timeout duration
 *          - Typically 1-2 seconds without new messages
 *          - RC failsafe triggers if both RC and MAVLink lost
 *          - Smooth transition back to RC if available
 *          
 *          Use cases for MAVLink manual control:
 *          1. Computer vision piloting (autonomous agents)
 *          2. Accessibility devices (adaptive controllers)
 *          3. Remote piloting over internet (low-bandwidth)
 *          4. Research platforms (algorithm testing)
 *          5. Backup control if RC transmitter fails
 *          6. Joystick control from GCS laptop
 *          7. Tablet/smartphone piloting applications
 *          
 *          Typical joystick control workflow:
 *          1. Operator connects joystick to GCS computer
 *          2. GCS software reads joystick axes
 *          3. GCS normalizes axes to -1000 to +1000 range
 *          4. GCS sends MANUAL_CONTROL messages at high rate (10-50 Hz)
 *          5. ArduPilot receives messages via MAVLink
 *          6. This function maps axes to control channels
 *          7. Manual override applied to channels
 *          8. Flight controller processes as normal RC input
 *          9. Vehicle responds to joystick commands
 *          
 *          Message rate requirements:
 *          - Minimum: 5 Hz (200ms period) for safety
 *          - Recommended: 10-20 Hz (50-100ms) for responsiveness
 *          - Higher rates: Smoother control but more bandwidth
 *          - Must exceed override timeout period
 *          
 *          Pitch reverse flag significance:
 *          - manual_override(channel_pitch, ..., true)
 *          - Final parameter 'true' indicates axis reversal
 *          - Compensates for RC transmitter conventions
 *          - MAVLink pitch: positive = forward
 *          - Some RC: positive = back (depends on config)
 *          - Ensures consistent control feel
 *          
 *          Channel object access:
 *          - copter.channel_roll: RC roll input channel
 *          - copter.channel_pitch: RC pitch input channel
 *          - copter.channel_throttle: RC throttle input channel
 *          - copter.channel_yaw: RC yaw input channel
 *          - Pointers to RC_Channel objects
 *          - Override writes to these channel values
 *          
 *          Flight mode compatibility:
 *          - STABILIZE: Full manual control via joystick
 *          - ALT_HOLD: Manual roll/pitch/yaw, altitude hold
 *          - LOITER: Manual control overrides position hold
 *          - GUIDED: Joystick may conflict with GCS commands
 *          - AUTO: Manual override depends on mode settings
 *          
 *          Safety features:
 *          - Negative thrust validation
 *          - Automatic timeout and RC restoration
 *          - RC failsafe still active if both lost
 *          - Geofence enforcement continues
 *          - Arming checks still required
 *          
 *          Configuration parameters:
 *          - PILOT_ACCEL_Z: Throttle responsiveness
 *          - PILOT_SPEED_UP/DN: Climb/descent rates
 *          - All standard pilot input parameters apply
 *          - Joystick treated same as RC input
 *          
 *          Comparison with RC transmitter:
 *          - RC: Direct radio link, low latency, high reliability
 *          - MAVLink: Telemetry link, higher latency, more flexible
 *          - RC: Hardware joysticks/gimbals
 *          - MAVLink: Any USB/Bluetooth joystick
 *          - RC: Dedicated frequency bands
 *          - MAVLink: Shared telemetry link
 *          
 *          Latency considerations:
 *          - MAVLink adds communication latency
 *          - Telemetry link may have delays
 *          - 50-200ms typical round-trip time
 *          - Higher than RC direct link (5-20ms)
 *          - Suitable for non-aggressive flying
 *          - Not recommended for acrobatic maneuvers
 *          
 * @param[in] packet Decoded MANUAL_CONTROL message with joystick axes
 * @param[in] tnow Current time in milliseconds for override timeout tracking
 * 
 * @note Negative thrust (packet.z < 0) silently ignored for copter safety
 * @note Override expires if messages stop (automatic RC restoration)
 * @note Pitch axis typically reversed to match RC conventions
 * @note High message rate (10-50 Hz) required for smooth control
 * 
 * @warning MAVLink control has higher latency than RC transmitter
 * @warning Both RC and MAVLink loss triggers failsafe
 * @warning Throttle range (0-1000) differs from other axes (1000-2000)
 * 
 * @see manual_override() for channel override implementation
 * @see RC_Channel for channel value management
 * @see handle_rc_channels_override() for alternative channel override method
 */
void GCS_MAVLINK_Copter::handle_manual_control_axes(const mavlink_manual_control_t &packet, const uint32_t tnow)
{
    // Validate thrust is non-negative (copters can only push down)
    if (packet.z < 0) {
        // Negative thrust invalid for copters - silently ignore command
        return;
    }

    // Map roll axis (MAVLink y) to roll channel
    // Range: -1000 to +1000 (MAVLink) → 1000 to 2000 μs (PWM)
    manual_override(copter.channel_roll, packet.y, 1000, 2000, tnow);
    
    // Map pitch axis (MAVLink x) to pitch channel
    // Range: -1000 to +1000 (MAVLink) → 1000 to 2000 μs (PWM)
    // Reversed flag (true) compensates for RC transmitter conventions
    manual_override(copter.channel_pitch, packet.x, 1000, 2000, tnow, true);
    
    // Map thrust axis (MAVLink z) to throttle channel
    // Range: 0 to +1000 (MAVLink) → 0 to 1000 μs (non-standard PWM)
    // Note: Different range than roll/pitch/yaw
    manual_override(copter.channel_throttle, packet.z, 0, 1000, tnow);
    
    // Map yaw axis (MAVLink r) to yaw channel
    // Range: -1000 to +1000 (MAVLink) → 1000 to 2000 μs (PWM)
    manual_override(copter.channel_yaw, packet.r, 1000, 2000, tnow);
}

/**
 * @brief Validate velocity or acceleration vector for sanity and physical limits
 * 
 * @details Performs input validation on velocity or acceleration vectors received
 *          from MAVLink SET_POSITION_TARGET messages. Checks each component for
 *          NaN (Not-a-Number) and enforces maximum magnitude of 1000 m/s (velocity)
 *          or 1000 m/s² (acceleration). Prevents invalid floating-point values and
 *          physically unrealistic commands from causing flight controller errors
 *          or dangerous vehicle behavior. Critical safety check before processing
 *          external setpoint commands.
 *          
 *          Validation checks performed:
 *          1. NaN detection: Rejects non-numeric values
 *          2. Magnitude limit: Rejects values exceeding ±1000
 *          3. Applied to all three components (X, Y, Z)
 *          
 *          NaN (Not-a-Number) detection:
 *          - NaN results from invalid math operations
 *          - Example: 0/0, ∞-∞, sqrt(-1)
 *          - Can propagate through calculations
 *          - Causes unpredictable behavior in controllers
 *          - Must be caught and rejected immediately
 *          - isnan() checks for NaN floating-point value
 *          
 *          Sources of NaN values:
 *          - Software bugs in GCS or companion computer
 *          - Uninitialized variables in command generation
 *          - Invalid coordinate transformations
 *          - Division by zero in motion planning
 *          - Corrupted telemetry data
 *          - Numerical instability in algorithms
 *          
 *          Velocity magnitude limit (1000 m/s):
 *          - Equivalent to 3600 km/h or 2237 mph
 *          - Far exceeds any copter capability
 *          - Realistic copter speeds: 5-30 m/s typical
 *          - Racing copters: up to 50 m/s
 *          - Limit catches obvious input errors
 *          - Protects against command corruption
 *          
 *          Acceleration magnitude limit (1000 m/s²):
 *          - Equivalent to ~100 g acceleration
 *          - Far exceeds copter structural limits
 *          - Realistic copter accel: 1-5 m/s² typical
 *          - Aggressive maneuvers: up to 20 m/s²
 *          - Limit catches corrupted acceleration commands
 *          - Prevents impossible control demands
 *          
 *          Component-by-component validation:
 *          - Each axis (X, Y, Z) validated independently
 *          - Single invalid component fails entire vector
 *          - Ensures all components usable
 *          - Prevents partial corruption issues
 *          
 *          Return value semantics:
 *          - true: Vector is valid and safe to use
 *          - false: Vector contains NaN or excessive values
 *          - Caller must reject command on false return
 *          
 *          Usage in SET_POSITION_TARGET handlers:
 *          1. Receive MAVLink position/velocity/acceleration command
 *          2. Extract velocity or acceleration vector
 *          3. Call sane_vel_or_acc_vector() for validation
 *          4. If false: Reject command, stop guided mode
 *          5. If true: Proceed with command processing
 *          
 *          Failure handling by caller:
 *          - Call copter.mode_guided.init(true) to stop
 *          - Holds current position
 *          - Prevents execution of invalid command
 *          - Protects vehicle from dangerous inputs
 *          - Command silently rejected (no ACK failure)
 *          
 *          Why 1000 m/s threshold chosen:
 *          - Conservative margin above realistic values
 *          - Catches obvious errors (corrupted data)
 *          - Doesn't restrict legitimate commands
 *          - Easy to detect telemetry issues
 *          - Round number for clarity
 *          
 *          Vector units context:
 *          - Input always in meters/second or meters/second²
 *          - MAVLink standard units (SI base units)
 *          - Internally converted to cm/s or cm/s² after validation
 *          - Validation on original units before conversion
 *          
 *          Coordinate frame independence:
 *          - Validation independent of coordinate frame
 *          - Checks magnitude, not direction
 *          - Applies to NED, ENU, body frame equally
 *          - Frame transformation happens after validation
 *          
 *          Example invalid vectors rejected:
 *          - Vector3f(NaN, 0, 0): Contains NaN
 *          - Vector3f(1500, 0, 0): Exceeds 1000 m/s limit
 *          - Vector3f(0, -2000, 0): Exceeds 1000 m/s limit
 *          - Vector3f(100, NaN, 50): Contains NaN in Y
 *          
 *          Example valid vectors accepted:
 *          - Vector3f(0, 0, 0): Zero velocity (hover)
 *          - Vector3f(20, 0, 0): 20 m/s forward (typical)
 *          - Vector3f(5, -3, 2): 3D velocity vector
 *          - Vector3f(999, 999, 999): Just under limit
 *          
 *          Float comparison considerations:
 *          - fabsf() used for absolute value (float version)
 *          - Comparison against exact 1000 value
 *          - No floating-point epsilon needed (magnitude check)
 *          - Simple greater-than comparison sufficient
 *          
 *          Performance considerations:
 *          - Very fast validation (3 comparisons + loop)
 *          - No expensive math operations
 *          - Early exit on first invalid component
 *          - Negligible CPU overhead
 *          - Called frequently in SET_POSITION_TARGET handlers
 *          
 *          Comparison with other validation:
 *          - This function: Physics/numerical validation
 *          - Frame validation: Coordinate system checks
 *          - GPS validation: Location sanity checks
 *          - Mode checks: Capability validation
 *          - Complementary validation layers
 *          
 *          Why not check for infinity:
 *          - Infinity (±∞) caught by magnitude limit
 *          - fabsf(∞) = ∞ > 1000, returns false
 *          - NaN check separate from infinity
 *          - Both invalid conditions detected
 *          
 *          Guided mode integration:
 *          - Used by set_velaccel() handlers
 *          - Used by set_destination_posvelaccel() handlers
 *          - Used by global position setpoint handlers
 *          - Critical for guided mode safety
 *          - Prevents invalid trajectory commands
 *          
 *          Testing recommendations:
 *          - SITL: Send commands with NaN values
 *          - SITL: Send commands with excessive velocities
 *          - Verify guided mode stops on invalid input
 *          - Check logs for command rejection
 *          
 * @param[in] vec Velocity or acceleration vector in meters/second or m/s²
 * 
 * @return true if all components are numbers (not NaN) and magnitude ≤1000
 * @return false if any component is NaN or magnitude >1000
 * 
 * @note Applied to velocity vectors (m/s) and acceleration vectors (m/s²)
 * @note Limit of 1000 applies to both velocity and acceleration
 * @note All three components (X, Y, Z) must pass validation
 * @note Caller should reject command and stop guided mode if false
 * 
 * @warning Single invalid component fails entire vector
 * @warning Infinity values fail magnitude check (treated as invalid)
 * 
 * @see handle_message_set_position_target_local_ned() for usage
 * @see handle_message_set_position_target_global_int() for usage
 */
bool GCS_MAVLINK_Copter::sane_vel_or_acc_vector(const Vector3f &vec) const
{
    // Check all three vector components (X, Y, Z)
    for (uint8_t i=0; i<3; i++) {
        // Reject if component is NaN (not a number)
        // NaN indicates invalid floating-point value from math error
        if (isnan(vec[i])) {
            return false;
        }
        
        // Reject if component magnitude exceeds 1000 m/s or m/s²
        // Limit catches corrupted data and physically impossible commands
        // fabsf() computes absolute value for magnitude check
        if (fabsf(vec[i]) > 1000) {
            return false;
        }
    }
    
    // All components valid - vector is sane
    return true;
}

#if MODE_GUIDED_ENABLED
    // for mavlink SET_POSITION_TARGET messages
    constexpr uint32_t MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE =
        POSITION_TARGET_TYPEMASK_X_IGNORE |
        POSITION_TARGET_TYPEMASK_Y_IGNORE |
        POSITION_TARGET_TYPEMASK_Z_IGNORE;

    constexpr uint32_t MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE =
        POSITION_TARGET_TYPEMASK_VX_IGNORE |
        POSITION_TARGET_TYPEMASK_VY_IGNORE |
        POSITION_TARGET_TYPEMASK_VZ_IGNORE;

    constexpr uint32_t MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE =
        POSITION_TARGET_TYPEMASK_AX_IGNORE |
        POSITION_TARGET_TYPEMASK_AY_IGNORE |
        POSITION_TARGET_TYPEMASK_AZ_IGNORE;

    constexpr uint32_t MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE =
        POSITION_TARGET_TYPEMASK_YAW_IGNORE;
    constexpr uint32_t MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE =
        POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
    constexpr uint32_t MAVLINK_SET_POS_TYPE_MASK_FORCE_SET =
        POSITION_TARGET_TYPEMASK_FORCE_SET;
#endif

#if MODE_GUIDED_ENABLED
/**
 * @brief Handle SET_ATTITUDE_TARGET MAVLink message for guided attitude control
 * 
 * @details Processes attitude and thrust/climb-rate commands sent via MAVLink for
 *          precision attitude control in Guided mode. Allows external controllers
 *          (companion computers, vision systems, research algorithms) to command
 *          vehicle orientation and vertical motion directly. Supports quaternion
 *          attitude targets with optional body angular rates and thrust/climb-rate
 *          control. Validates inputs for safety (unit quaternions, valid rates).
 *          Only active in Guided or Auto-Guided modes for operator safety.
 *          
 *          MAVLink message: SET_ATTITUDE_TARGET (message ID 82)
 *          Target system: This copter
 *          
 *          Message fields:
 *          - time_boot_ms: Timestamp from sender (reference only)
 *          - target_system: Target vehicle system ID
 *          - target_component: Target component ID
 *          - type_mask: Bitmask indicating which fields to ignore
 *          - q[4]: Attitude quaternion [w, x, y, z] (unit length required)
 *          - body_roll_rate: Roll rate in radians/second (body frame)
 *          - body_pitch_rate: Pitch rate in rad/s (body frame)
 *          - body_yaw_rate: Yaw rate in rad/s (body frame)
 *          - thrust: Collective thrust 0.0-1.0 or climb rate flag
 *          
 *          Type mask bit definitions:
 *          - ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE (0x01)
 *          - ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE (0x02)
 *          - ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE (0x04)
 *          - ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE (0x40)
 *          - ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE (0x80)
 *          
 *          Mode requirement enforcement:
 *          - Only processes in Guided mode
 *          - Only processes in Auto-Guided mode (AUTO with guided override)
 *          - Silently ignores in other modes
 *          - Prevents unintended attitude commands
 *          - Requires deliberate mode selection by operator
 *          
 *          Why Guided mode only:
 *          - Guided mode indicates external control expected
 *          - Operator has selected companion computer control
 *          - Other modes have their own attitude control
 *          - Prevents conflicts with autonomous navigation
 *          - Safety: Operator must explicitly enable
 *          
 *          Quaternion attitude representation:
 *          - q[0] = w (scalar component)
 *          - q[1] = x (i component)
 *          - q[2] = y (j component)
 *          - q[3] = z (k component)
 *          - Order: [w, x, y, z] in message
 *          - Must be unit length: |q| = 1.0 (validated)
 *          
 *          Quaternion validation:
 *          - Unit length check: sqrt(w²+x²+y²+z²) ≈ 1.0
 *          - Tolerance: ±0.001 (slightly > sqrt(FLT_EPSILON))
 *          - Invalid quaternion causes command rejection
 *          - Protects against corrupted attitude data
 *          - Prevents gimbal lock and numerical issues
 *          
 *          Attitude ignore flag handling:
 *          - If ATTITUDE_IGNORE set: Zero quaternion used
 *          - Indicates rate-only control mode
 *          - Attitude target not updated
 *          - Only body rates controlled
 *          - Allows pure rate control without attitude target
 *          
 *          Body rate control:
 *          - Rates in body frame (vehicle-relative)
 *          - Roll rate: Rotation about forward axis (rad/s)
 *          - Pitch rate: Rotation about right axis (rad/s)
 *          - Yaw rate: Rotation about down axis (rad/s)
 *          - All three must be specified or all ignored
 *          
 *          Rate ignore flags validation:
 *          - All three rate ignore flags must match
 *          - Can't mix ignored and used rates
 *          - Either provide all rates or none
 *          - Partial rate specification rejected
 *          - Prevents ambiguous control commands
 *          
 *          Invalid rate configuration:
 *          - Roll ignored, pitch/yaw specified: INVALID
 *          - Pitch ignored, roll/yaw specified: INVALID
 *          - Yaw ignored, roll/pitch specified: INVALID
 *          - Any partial specification stops guided mode
 *          
 *          Thrust field interpretation:
 *          - Two modes: Thrust mode or Climb-rate mode
 *          - Mode determined by set_attitude_target_provides_thrust()
 *          - GUID_OPTIONS parameter bit controls mode
 *          - Different scaling for each mode
 *          
 *          Thrust mode (use_thrust = true):
 *          - Direct thrust control (normalized)
 *          - Range: -1.0 to +1.0
 *          - 0.0 = Zero collective
 *          - +1.0 = Maximum upward thrust
 *          - -1.0 = Maximum downward thrust (copters limited)
 *          - Passed directly to attitude controller
 *          
 *          Climb-rate mode (use_thrust = false, default):
 *          - Thrust field interpreted as climb rate encoding
 *          - Range: 0.0 to 1.0
 *          - 0.5 = Zero climb rate (hover)
 *          - >0.5 = Climb (scaled to WPNAV_SPEED_UP)
 *          - <0.5 = Descend (scaled to WPNAV_SPEED_DN)
 *          - More intuitive for high-level controllers
 *          
 *          Climb-rate scaling:
 *          - thrust = 0.5: climb_rate = 0 cm/s
 *          - thrust = 1.0: climb_rate = +WPNAV_SPEED_UP cm/s
 *          - thrust = 0.0: climb_rate = -WPNAV_SPEED_DN cm/s
 *          - Linear interpolation between limits
 *          - Typical WPNAV_SPEED_UP: 250-500 cm/s
 *          - Typical WPNAV_SPEED_DN: 150-300 cm/s
 *          
 *          Thrust ignore flag handling:
 *          - If THROTTLE_IGNORE set: Command rejected
 *          - Thrust/climb-rate is mandatory
 *          - Can't control attitude without vertical control
 *          - Prevents incomplete control specification
 *          - Stops guided mode if thrust missing
 *          
 *          Command rejection scenarios:
 *          1. Not in Guided mode → Silent ignore
 *          2. Throttle ignored → Stop guided mode
 *          3. Invalid quaternion → Stop guided mode
 *          4. Partial rate specification → Stop guided mode
 *          
 *          Stop guided mode (init):
 *          - Calls copter.mode_guided.init(true)
 *          - Puts guided mode in position-hold state
 *          - Vehicle maintains current position
 *          - Prevents execution of invalid command
 *          - Safe default behavior on error
 *          
 *          Control handoff to guided mode:
 *          - Calls copter.mode_guided.set_angle()
 *          - Passes validated attitude quaternion
 *          - Passes body angular rates vector
 *          - Passes thrust or climb rate value
 *          - Guided mode implements low-level control
 *          
 *          Use cases for attitude control:
 *          1. Computer vision stabilization
 *          2. External IMU sensor fusion
 *          3. Research control algorithms
 *          4. Precision aerial manipulation
 *          5. Custom flight control laws
 *          6. Acrobatic maneuver automation
 *          
 *          Typical command sequence:
 *          1. Switch to Guided mode
 *          2. External controller computes desired attitude
 *          3. Convert attitude to quaternion
 *          4. Optionally compute body rates
 *          5. Encode climb rate or thrust
 *          6. Send SET_ATTITUDE_TARGET message
 *          7. Repeat at high rate (20-100 Hz)
 *          
 *          Message rate requirements:
 *          - Minimum: 10 Hz for stable control
 *          - Recommended: 50 Hz for responsive control
 *          - High rates: 100+ Hz for aggressive flight
 *          - Timeout if messages stop
 *          - Guided mode reverts to position hold
 *          
 *          Coordinate frames:
 *          - Quaternion: Earth frame to body frame rotation
 *          - Body rates: Body frame (vehicle-relative)
 *          - Thrust: Vehicle Z-axis (body down)
 *          - Consistent with MAVLink conventions
 *          
 *          Configuration parameters:
 *          - GUID_OPTIONS: Thrust interpretation bit
 *          - WPNAV_SPEED_UP: Maximum climb rate
 *          - WPNAV_SPEED_DN: Maximum descent rate
 *          - ATC_* attitude control gains
 *          
 * @param[in] msg MAVLink message containing SET_ATTITUDE_TARGET
 * 
 * @note Only compiled if MODE_GUIDED_ENABLED
 * @note Only processes message if in Guided or Auto-Guided mode
 * @note Thrust field is mandatory (THROTTLE_IGNORE flag causes rejection)
 * @note Quaternion must be unit length (tolerance ±0.001)
 * @note Body rates must all be specified or all ignored
 * @note Invalid inputs cause guided mode to stop (position hold)
 * 
 * @warning High message rate required for stable control (50+ Hz recommended)
 * @warning Message timeout causes reversion to position hold
 * @warning Quaternion validation critical for numerical stability
 * 
 * @see ModeGuided::set_angle() for attitude control implementation
 * @see ModeGuided::set_attitude_target_provides_thrust() for thrust mode check
 */
void GCS_MAVLINK_Copter::handle_message_set_attitude_target(const mavlink_message_t &msg)
{
    // Decode MAVLink message into packet structure
    mavlink_set_attitude_target_t packet;
    mavlink_msg_set_attitude_target_decode(&msg, &packet);

    // Enforce mode requirement: Only process in Guided or Auto-Guided mode
    // Prevents unintended attitude commands in other autonomous modes
    if (!copter.flightmode->in_guided_mode()) {
        // Silently ignore if not in appropriate mode
        return;
    }

    // Extract type mask flags to determine which fields to process
    // Bitmask indicates which fields should be ignored by vehicle
    const bool roll_rate_ignore   = packet.type_mask & ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE;
    const bool pitch_rate_ignore  = packet.type_mask & ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE;
    const bool yaw_rate_ignore    = packet.type_mask & ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE;
    const bool throttle_ignore    = packet.type_mask & ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE;
    const bool attitude_ignore    = packet.type_mask & ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE;

    // MANDATORY: Thrust/climb-rate must be specified
    // Attitude control without vertical control is incomplete
    if (throttle_ignore) {
        // Thrust field is required - cannot control attitude without vertical control
        // Stop guided mode and hold current position
        copter.mode_guided.init(true);
        return;
    }

    // Process attitude quaternion if provided
    Quaternion attitude_quat;
    if (attitude_ignore) {
        // Attitude target not specified - using rate-only control
        // Zero quaternion indicates no attitude target
        attitude_quat.zero();
    } else {
        // Extract quaternion from message
        // MAVLink order: q[0]=w, q[1]=x, q[2]=y, q[3]=z
        // Quaternion represents rotation from earth frame to body frame
        attitude_quat = Quaternion(packet.q[0],packet.q[1],packet.q[2],packet.q[3]);

        // CRITICAL VALIDATION: Quaternion must be unit length
        // Unit quaternion: sqrt(w² + x² + y² + z²) = 1.0
        // Tolerance: ±0.001 (slightly greater than sqrt(FLT_EPSILON))
        // Non-unit quaternions indicate corruption or invalid computation
        if (!attitude_quat.is_unit_length()) {
            // Quaternion magnitude invalid - reject command
            // Non-unit quaternions cause numerical instability and gimbal lock
            // Stop guided mode to prevent undefined behavior
            copter.mode_guided.init(true);
            return;
        }
    }

    // Process body angular rates if provided
    Vector3f ang_vel_body;
    if (!roll_rate_ignore && !pitch_rate_ignore && !yaw_rate_ignore) {
        // All three body rates specified - extract and use them
        // Body frame: Roll=forward axis, Pitch=right axis, Yaw=down axis
        // Units: radians per second
        ang_vel_body.x = packet.body_roll_rate;
        ang_vel_body.y = packet.body_pitch_rate;
        ang_vel_body.z = packet.body_yaw_rate;
    } else if (!(roll_rate_ignore && pitch_rate_ignore && yaw_rate_ignore)) {
        // INVALID: Partial rate specification detected
        // All three rates must be specified together or all ignored
        // Mixed ignore flags create ambiguous control command
        // Examples of invalid configurations:
        // - Roll ignored, pitch/yaw specified
        // - Pitch ignored, roll/yaw specified
        // Cannot safely process partial rates - stop guided mode
        copter.mode_guided.init(true);
        return;
    }
    // Note: If all rates ignored, ang_vel_body remains zero-initialized

    // Determine thrust field interpretation mode
    // Two modes: Direct thrust or Climb-rate encoding
    // Controlled by GUID_OPTIONS parameter bit
    const bool use_thrust = copter.mode_guided.set_attitude_target_provides_thrust();

    float climb_rate_or_thrust;
    if (use_thrust) {
        // THRUST MODE: Direct normalized thrust control
        // Thrust range: -1.0 (max down) to +1.0 (max up)
        // 0.0 = Zero collective thrust
        // Typically used for low-level control or research
        // Constrain to valid range to handle out-of-range inputs
        climb_rate_or_thrust = constrain_float(packet.thrust, -1.0f, 1.0f);
    } else {
        // CLIMB-RATE MODE: Thrust field encodes desired climb rate
        // This is the default mode (more intuitive for high-level control)
        // Range: 0.0 to 1.0
        // Mapping: 0.0 = max descent, 0.5 = hover, 1.0 = max climb
        
        // Constrain to valid range [0.0, 1.0]
        packet.thrust = constrain_float(packet.thrust, 0.0f, 1.0f);
        
        if (is_equal(packet.thrust, 0.5f)) {
            // 0.5 = Hover command (zero climb rate)
            climb_rate_or_thrust = 0.0f;
        } else if (packet.thrust > 0.5f) {
            // >0.5 = Climb command
            // Scale: (thrust - 0.5) * 2.0 maps [0.5, 1.0] to [0.0, 1.0]
            // Multiply by WPNAV_SPEED_UP to get climb rate in cm/s
            // thrust=1.0 → full WPNAV_SPEED_UP climb rate
            // thrust=0.75 → half WPNAV_SPEED_UP climb rate
            climb_rate_or_thrust = (packet.thrust - 0.5f) * 2.0f * copter.wp_nav->get_default_speed_up_cms();
        } else {
            // <0.5 = Descend command
            // Scale: (0.5 - thrust) * 2.0 maps [0.0, 0.5] to [1.0, 0.0]
            // Multiply by -WPNAV_SPEED_DN to get negative climb rate
            // thrust=0.0 → full WPNAV_SPEED_DN descent rate (negative)
            // thrust=0.25 → half WPNAV_SPEED_DN descent rate (negative)
            climb_rate_or_thrust = (0.5f - packet.thrust) * 2.0f * -copter.wp_nav->get_default_speed_down_cms();
        }
    }

    // Command validated - pass to guided mode attitude controller
    // set_angle() implements low-level attitude tracking
    // Parameters:
    // - attitude_quat: Target attitude quaternion (or zero if rate-only)
    // - ang_vel_body: Body angular rates in rad/s (or zero if not specified)
    // - climb_rate_or_thrust: Vertical control (cm/s or normalized thrust)
    // - use_thrust: Interpretation flag for vertical control
    copter.mode_guided.set_angle(attitude_quat, ang_vel_body,
            climb_rate_or_thrust, use_thrust);
}

/**
 * @brief Handle SET_POSITION_TARGET_LOCAL_NED MAVLink message for guided position/velocity/acceleration control
 * 
 * @details Processes position, velocity, and acceleration setpoints sent via MAVLink in
 *          local NED (North-East-Down) coordinates for precision trajectory control in
 *          Guided mode. Enables external path planners, vision systems, and companion
 *          computers to command complex trajectories with position+velocity+acceleration
 *          feedforward. Supports multiple coordinate frames (local, body, offset),
 *          flexible field masking, and yaw/yaw-rate control. Validates all inputs for
 *          safety (coordinate frames, velocity sanity, position estimates). Only active
 *          in Guided or Auto-Guided modes for operator safety.
 *          
 *          MAVLink message: SET_POSITION_TARGET_LOCAL_NED (message ID 84)
 *          
 *          Message fields:
 *          - time_boot_ms: Timestamp from sender (reference only)
 *          - target_system: Target vehicle system ID
 *          - target_component: Target component ID
 *          - coordinate_frame: Frame of reference for position/velocity
 *          - type_mask: Bitmask indicating which fields to ignore
 *          - x, y, z: Position in meters (NED frame)
 *          - vx, vy, vz: Velocity in m/s (NED frame)
 *          - afx, afy, afz: Acceleration in m/s² (NED frame)
 *          - yaw: Yaw angle in radians
 *          - yaw_rate: Yaw rate in radians/second
 *          
 *          Type mask bit definitions:
 *          - POSITION_TARGET_TYPEMASK_X_IGNORE (0x0001)
 *          - POSITION_TARGET_TYPEMASK_Y_IGNORE (0x0002)
 *          - POSITION_TARGET_TYPEMASK_Z_IGNORE (0x0004)
 *          - POSITION_TARGET_TYPEMASK_VX_IGNORE (0x0008)
 *          - POSITION_TARGET_TYPEMASK_VY_IGNORE (0x0010)
 *          - POSITION_TARGET_TYPEMASK_VZ_IGNORE (0x0020)
 *          - POSITION_TARGET_TYPEMASK_AX_IGNORE (0x0040)
 *          - POSITION_TARGET_TYPEMASK_AY_IGNORE (0x0080)
 *          - POSITION_TARGET_TYPEMASK_AZ_IGNORE (0x0100)
 *          - POSITION_TARGET_TYPEMASK_FORCE_SET (0x0200)
 *          - POSITION_TARGET_TYPEMASK_YAW_IGNORE (0x0400)
 *          - POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE (0x0800)
 *          
 *          Supported coordinate frames:
 *          - MAV_FRAME_LOCAL_NED: Absolute position relative to EKF origin
 *          - MAV_FRAME_LOCAL_OFFSET_NED: Offset from current position
 *          - MAV_FRAME_BODY_NED: Body-frame axes, absolute altitude
 *          - MAV_FRAME_BODY_OFFSET_NED: Body-frame offset from current position
 *          
 *          Frame transformation logic:
 *          - LOCAL_NED: Direct use (already in EKF origin frame)
 *          - LOCAL_OFFSET_NED: Add current position to offset
 *          - BODY_NED: Rotate to NE, add current position for offset
 *          - BODY_OFFSET_NED: Rotate to NE, add current position
 *          
 *          Mode requirement enforcement:
 *          - Only processes in Guided mode
 *          - Only processes in Auto-Guided mode
 *          - Silently ignores in other modes
 *          - Prevents unintended trajectory commands
 *          
 *          Valid command combinations (by type mask):
 *          1. Position only: Set waypoint destination
 *          2. Position + Velocity: Feedforward trajectory control
 *          3. Position + Velocity + Acceleration: Full feedforward
 *          4. Velocity only: Velocity-guided mode (no position target)
 *          5. Velocity + Acceleration: Velocity with accel feedforward
 *          6. Acceleration only: Pure acceleration control
 *          
 *          Invalid combinations (rejected):
 *          - Force set with acceleration (forces not supported)
 *          - Unsupported coordinate frame
 *          - Position offset without valid position estimate
 *          - Invalid velocity magnitude (>1000 m/s or NaN)
 *          - Ambiguous field combinations
 *          
 *          Position processing:
 *          - Message units: meters
 *          - Internal units: centimeters
 *          - Conversion: multiply by 100.0
 *          - Z-axis inversion: -packet.z (NED down → up)
 *          - Body frame rotation if needed
 *          - Offset addition if needed
 *          
 *          Velocity processing:
 *          - Message units: meters/second
 *          - Internal units: centimeters/second
 *          - Conversion: multiply by 100.0
 *          - Z-axis inversion: -packet.vz
 *          - Sanity check: |v| <= 1000 m/s, not NaN
 *          - Body frame rotation if needed
 *          
 *          Acceleration processing:
 *          - Message units: meters/second²
 *          - Internal units: centimeters/second²
 *          - Conversion: multiply by 100.0
 *          - Z-axis inversion: -packet.afz
 *          - Body frame rotation if needed
 *          - Feedforward to position controller
 *          
 *          Force handling:
 *          - Force inputs NOT supported by copter
 *          - FORCE_SET flag with acceleration → REJECT
 *          - Force would require direct motor control
 *          - Position/velocity/acceleration abstraction used instead
 *          
 *          Yaw control:
 *          - Yaw in radians
 *          - Can be absolute or relative to body
 *          - Body frame: yaw relative to current heading
 *          - Yaw rate: radians/second rotation rate
 *          - Yaw and yaw-rate can be used together or separately
 *          
 *          Command routing to guided mode:
 *          - set_destination(): Position only
 *          - set_destination_posvelaccel(): Pos + vel + accel
 *          - set_velaccel(): Velocity + accel (no position)
 *          - set_accel(): Acceleration only
 *          
 *          Rejection and safety:
 *          - Invalid inputs call copter.mode_guided.init(true)
 *          - Stops and holds current position
 *          - Prevents execution of unsafe commands
 *          - No ACK sent on silent rejection
 *          
 *          Typical usage scenarios:
 *          1. Vision-based precision positioning
 *          2. Trajectory tracking for cinematography
 *          3. Aggressive acrobatic maneuvers
 *          4. Research control algorithms
 *          5. Obstacle avoidance path following
 *          6. Formation flight coordination
 *          
 *          Message rate requirements:
 *          - Position only: 1-10 Hz sufficient
 *          - Velocity control: 20-50 Hz recommended
 *          - Acceleration control: 50-100 Hz required
 *          - Higher rates = smoother trajectories
 *          - Timeout causes position hold
 *          
 *          Body frame rotation:
 *          - Uses current vehicle yaw
 *          - Calls copter.rotate_body_frame_to_NE()
 *          - Rotates X,Y components only (Z unchanged)
 *          - Forward/right → North/East transformation
 *          
 *          Position estimate requirement:
 *          - Offset frames need current position
 *          - Obtained from AHRS relative position
 *          - If unavailable: Reject command
 *          - GPS, optical flow, or vision required
 *          
 * @param[in] msg MAVLink message containing SET_POSITION_TARGET_LOCAL_NED
 * 
 * @note Only compiled if MODE_GUIDED_ENABLED
 * @note Only processes message if in Guided or Auto-Guided mode
 * @note Position in meters, velocity in m/s, acceleration in m/s²
 * @note Z-axis inverted: NED down convention
 * @note Velocity must pass sanity check (|v| <= 1000 m/s, not NaN)
 * @note Force inputs not supported (FORCE_SET with accel rejected)
 * 
 * @warning High message rate required for velocity/accel control (50+ Hz)
 * @warning Offset frames require valid position estimate
 * @warning Invalid velocity causes immediate stop (safety check)
 * @warning Body frame commands rotate with vehicle heading
 * 
 * @see handle_message_set_position_target_global_int() for global coordinates
 * @see sane_vel_or_acc_vector() for velocity validation
 * @see ModeGuided::set_destination() for position-only control
 * @see ModeGuided::set_destination_posvelaccel() for full trajectory control
 * @see ModeGuided::set_velaccel() for velocity control
 */
void GCS_MAVLINK_Copter::handle_message_set_position_target_local_ned(const mavlink_message_t &msg)
{
    // Decode MAVLink message into packet structure
    mavlink_set_position_target_local_ned_t packet;
    mavlink_msg_set_position_target_local_ned_decode(&msg, &packet);

    // Enforce mode requirement: Only process in Guided or Auto-Guided mode
    // Prevents unintended trajectory commands in other autonomous modes
    if (!copter.flightmode->in_guided_mode()) {
        // Silently ignore if not in appropriate mode
        return;
    }

    // Validate coordinate frame - only specific frames supported
    // Supported frames allow transformation to local NED coordinates
    if (packet.coordinate_frame != MAV_FRAME_LOCAL_NED &&
        packet.coordinate_frame != MAV_FRAME_LOCAL_OFFSET_NED &&
        packet.coordinate_frame != MAV_FRAME_BODY_NED &&
        packet.coordinate_frame != MAV_FRAME_BODY_OFFSET_NED) {
        // Unsupported coordinate frame
        // Cannot safely transform coordinates - reject command
        // Stop guided mode and hold current position
        copter.mode_guided.init(true);
        return;
    }

    // Extract type mask flags to determine which fields to process
    // Bitmask indicates which fields should be ignored by vehicle
    // Each field (position, velocity, acceleration) can be independently enabled
    bool pos_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
    bool vel_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
    bool acc_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;
    bool yaw_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
    bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;
    bool force_set       = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_FORCE_SET;

    // Force inputs are NOT supported by copter position controller
    // Force control would require direct motor thrust commands
    // We use position/velocity/acceleration abstraction instead
    // FORCE_SET flag with acceleration is invalid combination
    if (force_set && !acc_ignore) {
        // Force requested with acceleration - unsupported combination
        // Reject command and stop guided mode
        copter.mode_guided.init(true);
        return;
    }

    // Process position target if specified
    Vector3f pos_vector;
    if (!pos_ignore) {
        // Extract position from message and convert units
        // Message units: meters
        // Internal units: centimeters (multiply by 100.0)
        // Z-axis: NED down convention, negate for internal up convention
        pos_vector = Vector3f(packet.x * 100.0f, packet.y * 100.0f, -packet.z * 100.0f);
        
        // Rotate from body frame to NE (North-East) if body frame specified
        // Body frame: Forward/Right/Down relative to vehicle
        // NE frame: North/East/Up relative to EKF origin
        if (packet.coordinate_frame == MAV_FRAME_BODY_NED ||
            packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
            // Rotate X,Y components using current vehicle yaw
            // Z component unchanged (down is down in both frames)
            copter.rotate_body_frame_to_NE(pos_vector.x, pos_vector.y);
        }
        
        // Add current position offset if offset frame specified
        // Offset frames: Position relative to current location
        // Absolute frames: Position relative to EKF origin
        if (packet.coordinate_frame == MAV_FRAME_LOCAL_OFFSET_NED ||
            packet.coordinate_frame == MAV_FRAME_BODY_NED ||
            packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
            // Get current position relative to EKF origin
            Vector3f pos_ned_m;
            if (!AP::ahrs().get_relative_position_NED_origin_float(pos_ned_m)) {
                // Position estimate not available
                // Cannot compute offset without knowing current position
                // May occur if GPS not locked or no vision/optical flow
                // Reject command and stop guided mode
                copter.mode_guided.init(true);
                return;
            }
            // Add current position to offset to get absolute position
            // XY: Simple addition (both in NED frame)
            // Z: Subtract because internal uses up convention vs NED down
            pos_vector.xy() += pos_ned_m.xy() * 100.0;  // Convert m to cm
            pos_vector.z -= pos_ned_m.z * 100.0;  // Invert sign for up convention
        }
    }

    // Process velocity target if specified
    Vector3f vel_vector;
    if (!vel_ignore) {
        // Extract velocity from message
        // Z-axis: Negate for internal up convention vs NED down
        vel_vector = Vector3f{packet.vx, packet.vy, -packet.vz};
        
        // CRITICAL SAFETY CHECK: Validate velocity sanity
        // Checks for NaN and magnitude > 1000 m/s
        // Protects against corrupted data and impossible commands
        if (!sane_vel_or_acc_vector(vel_vector)) {
            // Velocity contains NaN or exceeds physical limits
            // Reject command to prevent dangerous behavior
            // Stop guided mode and hold current position
            copter.mode_guided.init(true);
            return;
        }
        
        // Convert velocity units: meters/second -> centimeters/second
        vel_vector *= 100;  // m/s -> cm/s
        
        // Rotate from body frame to NE if body frame specified
        // Body frame velocities: Forward/Right/Down relative to vehicle
        // NE frame velocities: North/East/Up relative to earth
        if (packet.coordinate_frame == MAV_FRAME_BODY_NED || 
            packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
            // Rotate X,Y components using current vehicle yaw
            // Z component unchanged (up/down independent of yaw)
            copter.rotate_body_frame_to_NE(vel_vector.x, vel_vector.y);
        }
    }

    // Process acceleration target if specified (feedforward)
    Vector3f accel_vector;
    if (!acc_ignore) {
        // Extract acceleration from message and convert units
        // Message units: meters/second²
        // Internal units: centimeters/second² (multiply by 100.0)
        // Z-axis: Negate for internal up convention vs NED down
        accel_vector = Vector3f(packet.afx * 100.0f, packet.afy * 100.0f, -packet.afz * 100.0f);
        
        // Rotate from body frame to NE if body frame specified
        // Body frame acceleration: Forward/Right/Down relative to vehicle
        // NE frame acceleration: North/East/Up relative to earth
        if (packet.coordinate_frame == MAV_FRAME_BODY_NED || 
            packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
            // Rotate X,Y components using current vehicle yaw
            // Z component unchanged (up/down independent of yaw)
            copter.rotate_body_frame_to_NE(accel_vector.x, accel_vector.y);
        }
    }

    // Process yaw target if specified
    float yaw_rad = 0.0f;
    bool yaw_relative = false;
    float yaw_rate_rads = 0.0f;
    
    if (!yaw_ignore) {
        // Extract yaw angle from message
        // Units: radians (MAVLink standard)
        yaw_rad = packet.yaw;
        
        // Determine if yaw is absolute (earth frame) or relative (body frame)
        // Body frame: Yaw relative to current vehicle heading
        // Earth frame: Yaw absolute (north = 0)
        yaw_relative = packet.coordinate_frame == MAV_FRAME_BODY_NED || 
                       packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED;
    }
    
    if (!yaw_rate_ignore) {
        // Extract yaw rate from message
        // Units: radians/second (MAVLink standard)
        yaw_rate_rads = packet.yaw_rate;
    }

    // Dispatch to appropriate guided mode controller based on type mask
    // Type mask determines which fields are valid: position, velocity, acceleration
    // Controller selection based on combination of valid fields
    
    if (!pos_ignore && !vel_ignore) {
        // Position + Velocity + Acceleration specified
        // Use position-velocity-acceleration trajectory controller
        // Acceleration provides feedforward for smoother tracking
        // All three components for complete trajectory specification
        copter.mode_guided.set_destination_posvelaccel(pos_vector, vel_vector, accel_vector, 
                                                       !yaw_ignore, yaw_rad, 
                                                       !yaw_rate_ignore, yaw_rate_rads, 
                                                       yaw_relative);
    } else if (pos_ignore && !vel_ignore) {
        // Velocity + Acceleration specified (no position)
        // Use velocity-acceleration controller
        // Good for velocity hold or manual velocity commands
        // Acceleration provides feedforward
        copter.mode_guided.set_velaccel(vel_vector, accel_vector, 
                                        !yaw_ignore, yaw_rad, 
                                        !yaw_rate_ignore, yaw_rate_rads, 
                                        yaw_relative);
    } else if (pos_ignore && vel_ignore && !acc_ignore) {
        // Acceleration only specified
        // Use acceleration controller (rare, mostly for testing)
        // Direct acceleration control bypasses position/velocity loops
        copter.mode_guided.set_accel(accel_vector, 
                                     !yaw_ignore, yaw_rad, 
                                     !yaw_rate_ignore, yaw_rate_rads, 
                                     yaw_relative);
    } else if (!pos_ignore && vel_ignore && acc_ignore) {
        // Position only specified (most common case)
        // Use position controller
        // Vehicle plans trajectory to position target
        // Velocity and acceleration computed internally
        copter.mode_guided.set_destination(pos_vector, 
                                           !yaw_ignore, yaw_rad, 
                                           !yaw_rate_ignore, yaw_rate_rads, 
                                           yaw_relative, false);
    } else {
        // Invalid combination of type mask flags
        // No valid control target specified
        // All position/velocity/acceleration ignored, or invalid combination
        // Reject command and stop guided mode to hold position
        copter.mode_guided.init(true);
    }
}

/**
 * @brief Handle SET_POSITION_TARGET_GLOBAL_INT MAVLink message
 * 
 * @details Processes MAVLink SET_POSITION_TARGET_GLOBAL_INT message (ID 86) containing
 *          position/velocity/acceleration setpoints in global coordinates (latitude/longitude).
 *          Global version of SET_POSITION_TARGET_LOCAL_NED using GPS coordinates instead
 *          of local NED offsets. Validates coordinate frame, extracts type-masked fields,
 *          performs coordinate conversions, and dispatches to appropriate guided mode
 *          controller. Only processes message when vehicle in Guided or Auto-Guided mode.
 *          
 *          Message ID: MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT (86)
 *          
 *          MAVLink message structure:
 *          - time_boot_ms: Timestamp (milliseconds since system boot)
 *          - coordinate_frame: MAV_FRAME enum specifying coordinate system
 *          - type_mask: Bitmask indicating which fields to use/ignore
 *          - lat_int: Latitude in degrees * 1E7 (integer encoding)
 *          - lon_int: Longitude in degrees * 1E7 (integer encoding)
 *          - alt: Altitude in meters (float, frame-dependent)
 *          - vx: X velocity in m/s (North/East depending on frame)
 *          - vy: Y velocity in m/s (East/North depending on frame)
 *          - vz: Z velocity in m/s (Down in NED convention)
 *          - afx: X acceleration in m/s² (feedforward, North/East)
 *          - afy: Y acceleration in m/s² (feedforward, East/North)
 *          - afz: Z acceleration in m/s² (feedforward, Down in NED)
 *          - yaw: Yaw angle in radians (north=0, increasing clockwise)
 *          - yaw_rate: Yaw rate in rad/s
 *          
 *          Supported coordinate frames:
 *          - MAV_FRAME_GLOBAL: WGS84 coordinates, altitude MSL
 *          - MAV_FRAME_GLOBAL_INT: Same as GLOBAL (int encoding)
 *          - MAV_FRAME_GLOBAL_RELATIVE_ALT: Altitude relative to home
 *          - MAV_FRAME_GLOBAL_TERRAIN_ALT: Altitude above terrain
 *          
 *          Type mask bits (bitwise OR):
 *          - 0x0001 (bit 0): Ignore position X (latitude)
 *          - 0x0002 (bit 1): Ignore position Y (longitude)
 *          - 0x0004 (bit 2): Ignore position Z (altitude)
 *          - 0x0008 (bit 3): Ignore velocity X (vx)
 *          - 0x0010 (bit 4): Ignore velocity Y (vy)
 *          - 0x0020 (bit 5): Ignore velocity Z (vz)
 *          - 0x0040 (bit 6): Ignore acceleration X (afx)
 *          - 0x0080 (bit 7): Ignore acceleration Y (afy)
 *          - 0x0100 (bit 8): Ignore acceleration Z (afz)
 *          - 0x0200 (bit 9): Force set (not supported, rejected)
 *          - 0x0400 (bit 10): Ignore yaw
 *          - 0x0800 (bit 11): Ignore yaw rate
 *          
 *          Processing flow:
 *          1. Decode MAVLink message
 *          2. Check vehicle in Guided mode (exit if not)
 *          3. Extract type mask flags
 *          4. Validate force_set not used with acceleration
 *          5. Extract and convert position (lat/lon/alt)
 *          6. Validate GPS coordinates if position specified
 *          7. Convert altitude to appropriate frame
 *          8. Extract and validate velocity vector
 *          9. Extract acceleration vector
 *          10. Extract yaw and yaw rate
 *          11. Dispatch to guided mode controller
 *          
 *          Guided mode requirement:
 *          - Only processes when flightmode->in_guided_mode() true
 *          - Includes GUIDED and Auto-Guided (AUTO executing guided items)
 *          - Silently ignores message in other modes
 *          - Prevents unexpected position changes
 *          - GCS must set GUIDED mode first
 *          
 *          Position conversion:
 *          - lat_int/lon_int encoded as degrees * 1E7 (integer)
 *          - Converted to internal Location format
 *          - Altitude frame determined from coordinate_frame field
 *          - Altitude converted to centimeters (internal units)
 *          - Location sanity checking performed
 *          
 *          Coordinate frame conversions:
 *          - GLOBAL: Altitude MSL (mean sea level)
 *          - GLOBAL_RELATIVE_ALT: Altitude above home position
 *          - GLOBAL_TERRAIN_ALT: Altitude above terrain (requires terrain data)
 *          - Frame stored in Location object for processing
 *          
 *          Velocity/acceleration handling:
 *          - Same as local NED version
 *          - Units: m/s (velocity), m/s² (acceleration)
 *          - Converted to cm/s and cm/s² internally
 *          - Sanity checked for NaN and excessive magnitude
 *          - Always in earth frame (no body frame for global)
 *          
 *          Controller dispatch options:
 *          1. Position + Velocity: set_destination_posvel()
 *          2. Velocity + Acceleration: set_velaccel()
 *          3. Acceleration only: set_accel()
 *          4. Position only: set_destination(Location)
 *          5. Invalid combination: Stop guided mode
 *          
 *          Force set rejection:
 *          - type_mask FORCE_SET flag with acceleration not supported
 *          - Forces are not implemented in copter control
 *          - Would require different control allocation
 *          - Rejected to prevent misinterpretation
 *          
 *          Differences from local NED version:
 *          - Uses GPS coordinates instead of NED offsets
 *          - No body frame support (only earth frame)
 *          - Requires valid GPS position estimate
 *          - Altitude frame handling more complex
 *          - No coordinate frame rotation needed
 *          
 *          Position-velocity controller:
 *          - Cannot use with ABOVE_TERRAIN altitude frame
 *          - Requires EKF origin for coordinate conversion
 *          - Converts Location to NED vector from origin
 *          - Falls back to position-only if conversion fails
 *          
 *          Safety validations:
 *          - GPS coordinate sanity check (lat/lon valid range)
 *          - Velocity vector sanity check (NaN, magnitude)
 *          - EKF origin availability check
 *          - Guided mode active check
 *          - Force set + acceleration rejection
 *          
 *          Failure handling:
 *          - Invalid input: Stop guided mode (init(true))
 *          - Holds current position
 *          - No error message to GCS
 *          - Command silently rejected
 *          - Prevents dangerous behavior
 *          
 *          Typical usage scenarios:
 *          - Mission planning software sending waypoints
 *          - Offboard position control in GPS coordinates
 *          - Dynamic waypoint updates during flight
 *          - "Fly to" commands from GCS
 *          
 *          Integration with terrain following:
 *          - GLOBAL_TERRAIN_ALT frame supported
 *          - Requires terrain database availability
 *          - Altitude maintained above terrain
 *          - Good for survey missions
 *          
 *          Comparison to DO_REPOSITION command:
 *          - SET_POSITION_TARGET: Continuous streaming
 *          - DO_REPOSITION: Single waypoint command
 *          - SET_POSITION_TARGET: Can include velocity
 *          - DO_REPOSITION: Position only
 *          - SET_POSITION_TARGET: Real-time control
 *          - DO_REPOSITION: Mission command
 *          
 * @param[in] msg MAVLink message containing SET_POSITION_TARGET_GLOBAL_INT
 * 
 * @return void (no return value)
 * 
 * @note Only processes message when vehicle in Guided mode
 * @note Silently ignores message in other flight modes
 * @note Uses GPS coordinates (lat/lon) unlike local NED version
 * @note No body frame support (earth frame only)
 * @note Position-velocity mode doesn't support ABOVE_TERRAIN frame
 * 
 * @warning Force set flag with acceleration not supported and rejected
 * @warning Invalid coordinates stop guided mode for safety
 * @warning Requires valid GPS position for operation
 * @warning NaN or excessive velocity values rejected
 * 
 * @see handle_message_set_position_target_local_ned() for local version
 * @see ModeGuided::set_destination() for position-only control
 * @see ModeGuided::set_destination_posvel() for pos+vel control
 */
void GCS_MAVLINK_Copter::handle_message_set_position_target_global_int(const mavlink_message_t &msg)
{
    // Decode MAVLink message into packet structure
    // Message ID 86: SET_POSITION_TARGET_GLOBAL_INT
    mavlink_set_position_target_global_int_t packet;
    mavlink_msg_set_position_target_global_int_decode(&msg, &packet);

    // CRITICAL MODE CHECK: Only process in Guided mode
    // Guided mode includes:
    // - GUIDED: Direct GCS control mode
    // - Auto-Guided: AUTO mode executing guided mission items
    // This check prevents unexpected position changes in other modes
    // Silently ignores message if not in guided mode
    if (!copter.flightmode->in_guided_mode()) {
        return;
    }

    // TODO: Coordinate frame validation could be added here
    // Currently accepts all frames, validates during Location conversion
    // Could reject unsupported frames earlier for efficiency

    // Extract type mask flags indicating which fields to use
    // Type mask uses bitwise OR of ignore flags
    // Set bit = ignore field, clear bit = use field
    bool pos_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
    bool vel_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
    bool acc_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;
    bool yaw_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
    bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;
    bool force_set       = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_FORCE_SET;

    // FORCE SET REJECTION: Force inputs not supported
    // Force set flag indicates forces instead of accelerations
    // ArduPilot copter uses acceleration control, not force control
    // Force control would require motor-specific force allocation
    // Reject command if force_set true and acceleration specified
    if (force_set && !acc_ignore) {
        // Invalid combination: force flag with acceleration
        // Stop guided mode and hold current position
        copter.mode_guided.init(true);
        return;
    }

    // Extract GPS location from message if position specified
    Location loc;
    if (!pos_ignore) {
        // CRITICAL SAFETY CHECK: Validate GPS coordinates
        // Checks latitude and longitude are within valid ranges:
        // Latitude: -90° to +90° (encoded as ±900000000)
        // Longitude: -180° to +180° (encoded as ±1800000000)
        // Protects against corrupted data and invalid coordinates
        if (!check_latlng(packet.lat_int, packet.lon_int)) {
            // GPS coordinates invalid or out of range
            // Reject command to prevent flying to invalid location
            // Stop guided mode and hold current position
            copter.mode_guided.init(true);
            return;
        }
        
        // Convert MAVLink coordinate frame to ArduPilot altitude frame
        // MAV_FRAME enum specifies coordinate system:
        // - GLOBAL: Altitude MSL (mean sea level)
        // - GLOBAL_RELATIVE_ALT: Altitude above home
        // - GLOBAL_TERRAIN_ALT: Altitude above terrain
        Location::AltFrame frame;
        if (!mavlink_coordinate_frame_to_location_alt_frame((MAV_FRAME)packet.coordinate_frame, frame)) {
            // Unknown or unsupported coordinate frame
            // Cannot interpret altitude correctly
            // Reject command for safety
            copter.mode_guided.init(true);
            return;
        }
        
        // Construct Location object with validated coordinates
        // lat_int, lon_int: Degrees * 1E7 (integer encoding)
        // alt: Meters (float), converted to centimeters (* 100)
        // frame: Altitude frame from coordinate_frame field
        loc = {packet.lat_int, packet.lon_int, int32_t(packet.alt*100), frame};
    }

    // Process velocity target if specified
    Vector3f vel_vector;
    if (!vel_ignore) {
        // Extract velocity from message
        // vx, vy: North, East velocity in m/s (earth frame)
        // vz: Down velocity in m/s (NED convention)
        // Z-axis: Negate for internal up convention vs NED down
        vel_vector = Vector3f{packet.vx, packet.vy, -packet.vz};
        
        // CRITICAL SAFETY CHECK: Validate velocity sanity
        // Checks for NaN and magnitude > 1000 m/s
        // Protects against corrupted data and impossible commands
        if (!sane_vel_or_acc_vector(vel_vector)) {
            // Velocity contains NaN or exceeds physical limits
            // Reject command to prevent dangerous behavior
            // Stop guided mode and hold current position
            copter.mode_guided.init(true);
            return;
        }
        
        // Convert velocity units: meters/second -> centimeters/second
        // Internal ArduPilot units are cm/s for velocity
        vel_vector *= 100;  // m/s -> cm/s
    }

    // Process acceleration feedforward if specified
    Vector3f accel_vector;
    if (!acc_ignore) {
        // Extract acceleration from message
        // afx, afy: North, East acceleration in m/s² (earth frame)
        // afz: Down acceleration in m/s² (NED convention)
        // Z-axis: Negate for internal up convention vs NED down
        // Acceleration used as feedforward term in position control
        // Convert to centimeters/second² (internal units)
        accel_vector = Vector3f(packet.afx * 100.0f, packet.afy * 100.0f, -packet.afz * 100.0f);
    }

    // Process yaw targets if specified
    // Yaw control independent of position/velocity
    float yaw_rad = 0.0f;
    float yaw_rate_rads = 0.0f;
    if (!yaw_ignore) {
        // Yaw angle in radians
        // Zero = North, increasing clockwise (earth frame)
        yaw_rad = packet.yaw;
    }
    if (!yaw_rate_ignore) {
        // Yaw rate in radians/second
        // Positive = clockwise rotation
        yaw_rate_rads = packet.yaw_rate;
    }

    // CONTROLLER DISPATCH: Route to appropriate guided mode controller
    // Selection based on which fields specified in type_mask
    // Different controllers optimized for different control modes
    
    if (!pos_ignore && !vel_ignore) {
        // POSITION + VELOCITY MODE: Combined pos/vel control
        // Most capable mode: tracks position with velocity feedforward
        // Requires converting Location to NED vector from EKF origin
        
        // LIMITATION: Cannot use ABOVE_TERRAIN frame with posvel controller
        // Position-velocity controller works in NED frame from origin
        // Terrain following requires different approach
        if (loc.get_alt_frame() == Location::AltFrame::ABOVE_TERRAIN) {
            // Altitude frame ABOVE_TERRAIN not supported in this mode
            // Use position-only mode for terrain following
            // Reject command to prevent altitude confusion
            copter.mode_guided.init(true);
            return;
        }
        
        // Convert GPS Location to NED vector from EKF origin
        // Required for position-velocity controller
        // NEU format: North-East-Up (converted internally to NED)
        Vector3f pos_neu_cm;
        if (!loc.get_vector_from_origin_NEU_cm(pos_neu_cm)) {
            // Conversion failed: no valid EKF origin
            // Requires GPS position estimate to be available
            // Reject command if cannot convert coordinates
            copter.mode_guided.init(true);
            return;
        }
        
        // Set position + velocity target with optional yaw
        // pos_neu_cm: Position in centimeters from origin (NEU)
        // vel_vector: Velocity in cm/s (NED)
        // Yaw flags control whether yaw/yaw_rate used
        copter.mode_guided.set_destination_posvel(pos_neu_cm, vel_vector, !yaw_ignore, yaw_rad, !yaw_rate_ignore, yaw_rate_rads);
        
    } else if (pos_ignore && !vel_ignore) {
        // VELOCITY + ACCELERATION MODE: Velocity control with accel feedforward
        // No position target, tracks velocity setpoint
        // Acceleration used as feedforward term for better tracking
        // Vehicle drifts with velocity, no position hold
        copter.mode_guided.set_velaccel(vel_vector, accel_vector, !yaw_ignore, yaw_rad, !yaw_rate_ignore, yaw_rate_rads);
        
    } else if (pos_ignore && vel_ignore && !acc_ignore) {
        // ACCELERATION-ONLY MODE: Direct acceleration control
        // No position or velocity targets
        // Pilot-like control through acceleration
        // Requires active control to prevent drift
        copter.mode_guided.set_accel(accel_vector, !yaw_ignore, yaw_rad, !yaw_rate_ignore, yaw_rate_rads);
        
    } else if (!pos_ignore && vel_ignore && acc_ignore) {
        // POSITION-ONLY MODE: Standard waypoint navigation
        // Fly to GPS coordinate at default velocity
        // Most common mode for waypoint commands
        // Supports all altitude frames including ABOVE_TERRAIN
        copter.mode_guided.set_destination(loc, !yaw_ignore, yaw_rad, !yaw_rate_ignore, yaw_rate_rads);
        
    } else {
        // INVALID COMBINATION: No valid control mode specified
        // All control axes ignored or unsupported combination
        // Reject command and stop guided mode
        // Holds current position for safety
        copter.mode_guided.init(true);
    }
}
#endif  // MODE_GUIDED_ENABLED

/**
 * @brief Main MAVLink message handler dispatcher for Copter-specific messages
 * 
 * @details Primary entry point for processing incoming MAVLink messages specific to
 *          multicopter vehicle type. Routes messages to appropriate handler functions
 *          based on message ID. Handles copter-specific messages that require custom
 *          processing beyond generic GCS_MAVLINK base class handling.
 *          
 *          Message routing architecture:
 *          1. Check message ID in switch statement
 *          2. Call copter-specific handler if ID matches
 *          3. Fall through to base class for standard messages
 *          
 *          Copter-specific messages handled:
 *          - SET_ATTITUDE_TARGET (ID 82): Attitude/rate/thrust setpoints
 *          - SET_POSITION_TARGET_LOCAL_NED (ID 84): Local position/velocity targets
 *          - SET_POSITION_TARGET_GLOBAL_INT (ID 86): Global GPS position targets
 *          - TERRAIN_DATA (ID 134): Terrain elevation data
 *          - TERRAIN_CHECK (ID 135): Terrain elevation request
 *          - NAMED_VALUE_INT (ID 252): Toy mode control values
 *          
 *          Message processing requirements:
 *          - Only processes messages when in appropriate mode
 *          - Mode checks within individual handlers
 *          - Guided mode required for control messages
 *          - Terrain messages require terrain database enabled
 *          
 *          SET_ATTITUDE_TARGET handling:
 *          - Requires GUIDED mode active
 *          - Sets attitude quaternion or body rates
 *          - Includes thrust/climb rate control
 *          - High-rate control loop integration
 *          - See handle_message_set_attitude_target()
 *          
 *          SET_POSITION_TARGET_LOCAL_NED handling:
 *          - Requires GUIDED mode active
 *          - Position in NED frame from origin
 *          - Velocity and acceleration feedforward
 *          - Body frame and offset frame support
 *          - See handle_message_set_position_target_local_ned()
 *          
 *          SET_POSITION_TARGET_GLOBAL_INT handling:
 *          - Requires GUIDED mode active
 *          - GPS coordinates (lat/lon/alt)
 *          - Multiple altitude frame support
 *          - Velocity and acceleration feedforward
 *          - See handle_message_set_position_target_global_int()
 *          
 *          Terrain message handling:
 *          - TERRAIN_DATA: Elevation grid data from GCS
 *          - TERRAIN_CHECK: Request for terrain height
 *          - Requires AP_TERRAIN_AVAILABLE compile flag
 *          - Forwarded to terrain database object
 *          - Enables terrain following and avoidance
 *          
 *          Toy mode handling:
 *          - NAMED_VALUE_INT: Special control for toy mode
 *          - Requires TOY_MODE_ENABLED compile flag
 *          - Simplified control interface
 *          - Educational/demonstration feature
 *          
 *          Message ID reference:
 *          - 82: SET_ATTITUDE_TARGET
 *          - 84: SET_POSITION_TARGET_LOCAL_NED
 *          - 86: SET_POSITION_TARGET_GLOBAL_INT
 *          - 134: TERRAIN_DATA
 *          - 135: TERRAIN_CHECK
 *          - 252: NAMED_VALUE_INT
 *          
 *          Base class fallthrough:
 *          - All unhandled messages passed to GCS_MAVLINK::handle_message()
 *          - Processes common messages (HEARTBEAT, PARAM_REQUEST, etc.)
 *          - Mission protocol messages
 *          - Parameter get/set
 *          - Command messages (routed to handle_command_int)
 *          
 *          Handler function responsibilities:
 *          - Decode MAVLink message structure
 *          - Validate message contents
 *          - Check vehicle mode requirements
 *          - Perform coordinate conversions
 *          - Call appropriate vehicle control functions
 *          - No ACK required (streaming messages)
 *          
 *          Performance considerations:
 *          - Switch statement for O(1) dispatch
 *          - No processing if mode requirements not met
 *          - Handlers return quickly if not applicable
 *          - High-rate messages (50-100 Hz typical)
 *          
 *          Thread safety:
 *          - Called from MAVLink receive thread
 *          - Handlers must be thread-safe
 *          - Atomic operations or semaphores required
 *          - Vehicle state access must be protected
 *          
 *          Integration with command handling:
 *          - Commands (COMMAND_INT, COMMAND_LONG) routed separately
 *          - Commands return MAV_RESULT ACK/NAK
 *          - Messages processed without acknowledgment
 *          - Commands are one-time, messages are streaming
 *          
 *          Compile-time configuration:
 *          - MODE_GUIDED_ENABLED: Enable position/attitude setpoint handlers
 *          - AP_TERRAIN_AVAILABLE: Enable terrain message handlers
 *          - TOY_MODE_ENABLED: Enable toy mode handler
 *          - Reduces code size when features disabled
 *          
 *          Comparison to command handling:
 *          - handle_message(): Streaming control inputs
 *          - handle_command_int(): One-shot commands with ACK
 *          - Messages: Continuous setpoints
 *          - Commands: Mode changes, actions
 *          
 *          Typical message flow:
 *          1. GCS sends position setpoint message
 *          2. Message received by MAVLink thread
 *          3. handle_message() dispatches by ID
 *          4. Handler checks mode requirements
 *          5. Handler updates vehicle setpoints
 *          6. Main loop reads setpoints and controls vehicle
 *          
 *          Error handling:
 *          - Invalid messages silently ignored
 *          - No error responses sent
 *          - Mode checks prevent inappropriate control
 *          - Handlers validate message contents
 *          
 * @param[in] msg MAVLink message structure containing message ID and data
 * 
 * @return void (no return value)
 * 
 * @note Called from MAVLink receive thread for each message
 * @note Handlers must be thread-safe when accessing vehicle state
 * @note Unhandled messages forwarded to base class GCS_MAVLINK
 * @note No acknowledgment sent for streaming messages
 * 
 * @warning Handlers must check mode requirements before changing vehicle state
 * @warning High-rate messages require efficient processing
 * @warning Thread safety required for vehicle state access
 * 
 * @see handle_message_set_attitude_target() for attitude control
 * @see handle_message_set_position_target_local_ned() for local position
 * @see handle_message_set_position_target_global_int() for GPS position
 * @see handle_command_int_packet() for command (vs message) handling
 */
void GCS_MAVLINK_Copter::handle_message(const mavlink_message_t &msg)
{
    // Message dispatcher: Route to copter-specific handlers by message ID
    // Switch provides O(1) message routing
    // Unhandled IDs fall through to base class
    switch (msg.msgid) {
#if MODE_GUIDED_ENABLED
    // GUIDED MODE CONTROL MESSAGES
    // High-rate streaming setpoints for position/attitude control
    // Only processed when vehicle in Guided or Auto-Guided mode
    // Enable closed-loop offboard control from GCS or companion computer
    
    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
        // Message ID 82: Attitude quaternion, body rates, and thrust/climb rate
        // Used for direct attitude control in Guided mode
        // Typical rate: 10-100 Hz depending on application
        // Bypasses position controller for direct attitude control
        handle_message_set_attitude_target(msg);
        break;
        
    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
        // Message ID 84: Position/velocity/acceleration in local NED coordinates
        // Local frame: North-East-Down offsets from EKF origin
        // Supports body frame and offset modes
        // Most common for offboard position control
        handle_message_set_position_target_local_ned(msg);
        break;
        
    case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:
        // Message ID 86: Position/velocity/acceleration in global GPS coordinates
        // Global frame: Latitude, longitude, altitude (various frames)
        // Easier for mission planning software (no local frame)
        // Converted internally to local coordinates
        handle_message_set_position_target_global_int(msg);
        break;
#endif

#if AP_TERRAIN_AVAILABLE
    // TERRAIN DATABASE MESSAGES
    // Elevation data exchange for terrain following and avoidance
    // Requires terrain database compiled in and enabled
    
    case MAVLINK_MSG_ID_TERRAIN_DATA:
        // Message ID 134: Terrain elevation grid data from GCS
        // GCS provides elevation data for requested areas
        // Stored in terrain database for terrain following
        // Falls through to same handler as TERRAIN_CHECK
    case MAVLINK_MSG_ID_TERRAIN_CHECK:
        // Message ID 135: Request/response for terrain height at location
        // Vehicle requests terrain data for planned path
        // GCS responds with elevation or grid data
        // Both messages handled by terrain database object
        copter.terrain.handle_data(chan, msg);
        break;
#endif

#if TOY_MODE_ENABLED
    // TOY MODE CONTROL MESSAGES
    // Simplified control interface for educational/demonstration use
    
    case MAVLINK_MSG_ID_NAMED_VALUE_INT:
        // Message ID 252: Named integer values for toy mode control
        // Special control mode for simplified operation
        // Educational feature for learning/demos
        // Not used in normal operation
        copter.g2.toy_mode.handle_message(msg);
        break;
#endif

    default:
        // STANDARD MESSAGES: Forward to base class handler
        // Includes common messages like:
        // - HEARTBEAT (ID 0): System status
        // - PARAM_REQUEST_LIST (ID 21): Parameter list request
        // - PARAM_REQUEST_READ (ID 20): Single parameter request
        // - PARAM_SET (ID 23): Parameter update
        // - MISSION_REQUEST_LIST (ID 43): Mission waypoint list
        // - MISSION_ITEM (ID 39): Mission waypoint data
        // - MISSION_REQUEST (ID 40): Request specific waypoint
        // - COMMAND_INT (ID 75): Integer parameter commands
        // - COMMAND_LONG (ID 76): Float parameter commands
        // - Plus many other standard MAVLink messages
        GCS_MAVLINK::handle_message(msg);
        break;
    }
}

/**
 * @brief Handle MAV_CMD_DO_FLIGHTTERMINATION command for emergency vehicle shutdown
 * 
 * @details Processes flight termination command from GCS for emergency situations.
 *          Terminates flight by immediately disarming motors, causing controlled crash.
 *          This is an emergency safety feature used only in critical situations where
 *          continued flight poses greater risk than controlled crash/landing.
 *          
 *          Command processing sequence:
 *          1. Check if advanced failsafe enabled (compile-time option)
 *          2. If available, try advanced failsafe termination first
 *          3. If not available or not accepted, perform standard disarm
 *          4. Standard termination: Immediate motor disarm
 *          
 *          Advanced failsafe termination (if enabled):
 *          - May include additional hardware safety actions
 *          - Could trigger external termination devices
 *          - Parachute deployment if configured
 *          - Hardware-level motor shutoff
 *          - More comprehensive safety response
 *          
 *          Standard termination (always available):
 *          - Immediate motor disarm
 *          - Vehicle will fall/crash
 *          - No recovery possible after termination
 *          - Logged as TERMINATION disarm method
 *          - Black box data preserved
 *          
 *          Parameter interpretation:
 *          - param1 > 0.5: Activate termination
 *          - param1 <= 0.5: Do not terminate (command fails)
 *          - Binary threshold: 0.5 is clear ON/OFF boundary
 *          - Prevents accidental termination from noise
 *          
 *          Safety considerations:
 *          - This is a last-resort emergency measure
 *          - Vehicle will crash/fall after termination
 *          - Use only when continued flight is more dangerous
 *          - Examples: Flyaway, mid-air collision risk, fire
 *          - Not for normal landing or controlled descent
 *          
 *          Typical use cases:
 *          - Uncontrolled vehicle behavior (flyaway)
 *          - Loss of control with collision risk
 *          - Fire or smoke detected onboard
 *          - Flying into restricted airspace
 *          - Safety pilot emergency abort
 *          - Competition/test range safety termination
 *          
 *          Effects of termination:
 *          - Motors disarm immediately (within milliseconds)
 *          - Vehicle drops/falls with no motor control
 *          - Attitude control lost
 *          - May tumble or spin during fall
 *          - Logs continue recording until impact
 *          - Parameters preserved in storage
 *          
 *          Post-termination state:
 *          - Vehicle disarmed
 *          - Cannot be rearmed without power cycle
 *          - Requires inspection before next flight
 *          - Check for impact damage
 *          - Review logs to determine cause
 *          
 *          Comparison to other emergency responses:
 *          - Termination: Immediate disarm (crash)
 *          - Land mode: Controlled descent to ground
 *          - RTL: Return and land at home
 *          - Parachute: Deploy chute (if equipped)
 *          - Termination is most aggressive response
 *          
 *          Advanced failsafe option:
 *          - Compile flag: AP_COPTER_ADVANCED_FAILSAFE_ENABLED
 *          - May include hardware termination switch
 *          - External safety systems integration
 *          - Tries advanced method first
 *          - Falls back to standard if unavailable
 *          
 *          Command ID: MAV_CMD_DO_FLIGHTTERMINATION (ID 185)
 *          
 *          Return values:
 *          - MAV_RESULT_ACCEPTED: Termination activated
 *          - MAV_RESULT_FAILED: param1 <= 0.5 (no termination)
 *          
 *          MAVLink command format:
 *          - command: 185 (MAV_CMD_DO_FLIGHTTERMINATION)
 *          - param1: 0=Cancel, 1=Terminate
 *          - All other params unused
 *          
 *          Integration with safety systems:
 *          - Can be triggered by hardware switch
 *          - GCS emergency stop button
 *          - Geofence breach action
 *          - Failsafe cascade final stage
 *          - Competition range safety officer
 *          
 *          Testing considerations:
 *          - DO NOT test with propellers attached
 *          - Test on bench with motors removed
 *          - Verify disarm occurs
 *          - Check log for TERMINATION method
 *          - Have safety plan before any test
 *          
 *          Legal/regulatory considerations:
 *          - May be required for certain operations
 *          - Competition rules may mandate
 *          - Beyond visual line of sight (BVLOS)
 *          - Operations over populated areas
 *          - Test range requirements
 *          
 * @param[in] packet MAVLink COMMAND_INT packet with termination parameters
 *                   packet.param1 > 0.5 activates termination
 * 
 * @return MAV_RESULT_ACCEPTED if termination activated successfully
 * @return MAV_RESULT_FAILED if param1 <= 0.5 (termination not requested)
 * 
 * @note This is an EMERGENCY function - vehicle will crash
 * @note Use only as last resort when continued flight is more dangerous
 * @note Advanced failsafe tried first if compiled in
 * @note Standard termination always available as fallback
 * 
 * @warning VEHICLE WILL CRASH - motors disarm immediately
 * @warning No recovery possible after termination
 * @warning Use only in true emergency situations
 * @warning Ensure area below is clear before termination
 * 
 * @see AP_Arming::disarm() for disarm implementation
 * @see AP_Arming::Method::TERMINATION for disarm method enum
 */
MAV_RESULT GCS_MAVLINK_Copter::handle_flight_termination(const mavlink_command_int_t &packet) {
#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
    // Advanced failsafe: Try hardware-level termination first if available
    // May include external safety devices, parachute deployment, hardware motor cutoff
    if (GCS_MAVLINK::handle_flight_termination(packet) == MAV_RESULT_ACCEPTED) {
        return MAV_RESULT_ACCEPTED;
    }
#endif

    // Standard termination: param1 > 0.5 activates emergency disarm
    // Threshold at 0.5 prevents accidental termination from parameter noise
    if (packet.param1 > 0.5f) {
        // EMERGENCY TERMINATION: Disarm motors immediately
        // Vehicle will fall/crash - this is a last-resort safety measure
        // Logged as TERMINATION method for post-incident analysis
        copter.arming.disarm(AP_Arming::Method::TERMINATION);
        return MAV_RESULT_ACCEPTED;
    }

    // param1 <= 0.5: Termination not requested, fail command
    return MAV_RESULT_FAILED;
}

/**
 * @brief Get altitude value for VFR_HUD message with compatibility handling
 * 
 * @details Returns altitude for VFR_HUD (Visual Flight Rules Heads-Up Display) message
 *          with optional compatibility mode for older ground stations.
 *          
 *          VFR_HUD message (ID 74):
 *          - Primary flight display data for ground stations
 *          - Mimics traditional aircraft instruments
 *          - Shows: airspeed, groundspeed, heading, throttle, altitude, climb
 *          - Standard telemetry stream message
 *          
 *          Altitude frame behavior:
 *          - Standard mode (default): Returns MSL (Mean Sea Level) altitude
 *          - Compatibility mode: Returns relative altitude (above home)
 *          - Controlled by DevOptionVFR_HUDRelativeAlt developer option
 *          
 *          Standard mode (DevOptionVFR_HUDRelativeAlt = 0):
 *          - Returns absolute altitude MSL from base class
 *          - Consistent with MAVLink specification
 *          - Preferred for modern ground stations
 *          - GPS altitude reference
 *          
 *          Compatibility mode (DevOptionVFR_HUDRelativeAlt = 1):
 *          - Returns altitude relative to home/arming point
 *          - For older GCS expecting relative altitude
 *          - Legacy behavior from earlier ArduPilot versions
 *          - Current location altitude field (cm) converted to meters
 *          
 *          Units:
 *          - Internal: centimeters (current_loc.alt in cm)
 *          - Returned: meters (divided by 100)
 *          - MAVLink VFR_HUD expects meters
 *          
 *          Altitude reference frames:
 *          - MSL (standard): Height above mean sea level
 *          - Relative (compat): Height above home position
 *          - Home set at arming or EKF initialization
 *          - Relative useful for "how high am I flying"
 *          - MSL useful for terrain/obstacle avoidance
 *          
 *          Ground station compatibility:
 *          - Modern GCS: Use standard MSL mode
 *          - Mission Planner (recent): Handles both
 *          - QGroundControl: Expects MSL (standard)
 *          - Older/custom GCS: May need compatibility mode
 *          - Legacy tablet apps: Often expect relative
 *          
 *          Developer option control:
 *          - Parameter: BRD_OPTIONS bit DevOptionVFR_HUDRelativeAlt
 *          - Bitmask in developer options parameter
 *          - Runtime configurable (no recompile needed)
 *          - Check with bitwise AND operation
 *          
 *          Negative altitude handling:
 *          - Standard MSL: Can be negative (below sea level)
 *          - Relative: Can be negative (below home)
 *          - Both modes support full range
 *          
 *          Typical values:
 *          - Ground at sea level: 0m MSL, 0m relative
 *          - Flying 100m AGL at 500m MSL: 500m MSL, 100m relative
 *          - Ground at 1000m MSL: 1000m MSL, 0m relative
 *          
 *          Message streaming:
 *          - VFR_HUD typically sent at 4-10 Hz
 *          - Part of standard telemetry stream
 *          - Low bandwidth requirement
 *          - Essential for basic flight monitoring
 *          
 * @return Altitude in meters (MSL or relative depending on dev option)
 *         Standard mode: MSL altitude from base class
 *         Compatibility mode: Relative altitude (current_loc.alt * 0.01)
 * 
 * @note Default is standard MSL altitude (preferred)
 * @note Enable compatibility mode only for older ground stations
 * @note Altitude frame affects displayed height on GCS
 */
float GCS_MAVLINK_Copter::vfr_hud_alt() const
{
    // Check developer option for compatibility mode
    if (copter.g2.dev_options.get() & DevOptionVFR_HUDRelativeAlt) {
        // Compatibility option: Return relative altitude for older devices
        // Legacy behavior - older mavlink-aware devices expected relative altitude
        // current_loc.alt is in centimeters, convert to meters
        return copter.current_loc.alt * 0.01f;
    }
    // Standard mode: Return MSL altitude from base class (preferred)
    return GCS_MAVLINK::vfr_hud_alt();
}

/**
 * @brief Report vehicle's MAVLink protocol capabilities to ground station
 * 
 * @details Returns bitmask of supported MAVLink protocol features and message types.
 *          Used by ground stations to determine which commands/messages are supported,
 *          enabling adaptive UI and feature availability based on vehicle capabilities.
 *          
 *          Sent in AUTOPILOT_VERSION message (ID 148) during connection handshake.
 *          Ground stations query capabilities to enable/disable features in UI.
 *          
 *          Copter-specific capabilities:
 *          
 *          MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT:
 *          - Support for mission items with floating-point parameters
 *          - Standard mission commands (waypoints, splines, etc.)
 *          - Altitude in meters, coordinates in degrees
 *          
 *          MAV_PROTOCOL_CAPABILITY_MISSION_INT:
 *          - Support for integer-encoded mission items
 *          - Higher precision GPS coordinates (1e7 format)
 *          - Preferred format for waypoint missions
 *          - Latitude/longitude as integers for exact representation
 *          
 *          MAV_PROTOCOL_CAPABILITY_COMMAND_INT:
 *          - Support for COMMAND_INT message (ID 75)
 *          - Commands with integer position parameters
 *          - More precise than COMMAND_LONG for position commands
 *          - Used for DO_REPOSITION, NAV_TAKEOFF, etc.
 *          
 *          MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED:
 *          - Support for offboard position control in local frame
 *          - Message ID 84: SET_POSITION_TARGET_LOCAL_NED
 *          - Enables guided mode with local NED setpoints
 *          - Position/velocity/acceleration control
 *          - Used by companion computers for offboard control
 *          
 *          MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT:
 *          - Support for offboard position control in global frame
 *          - Message ID 86: SET_POSITION_TARGET_GLOBAL_INT
 *          - GPS coordinate-based guided control
 *          - Position/velocity in lat/lon/alt format
 *          - Easier for mission planning software
 *          
 *          MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION:
 *          - Support for emergency termination command
 *          - Command ID 185: MAV_CMD_DO_FLIGHTTERMINATION
 *          - Immediate motor shutoff for safety
 *          - Emergency abort capability
 *          - Required for some certification/competition
 *          
 *          MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET:
 *          - Support for direct attitude control
 *          - Message ID 82: SET_ATTITUDE_TARGET
 *          - Quaternion and body rate commands
 *          - Thrust/climb rate control
 *          - For advanced flight controllers and research
 *          
 *          MAV_PROTOCOL_CAPABILITY_TERRAIN (conditional):
 *          - Support for terrain following and database
 *          - Only included if terrain system compiled in AND enabled
 *          - Message IDs 134/135: TERRAIN_DATA/TERRAIN_CHECK
 *          - Elevation data exchange with GCS
 *          - Runtime check: terrain must be enabled by parameter
 *          
 *          Base class capabilities (inherited):
 *          - Parameter protocol (get/set/list)
 *          - Mission upload/download
 *          - Data streaming control
 *          - System status reporting
 *          - Fence operations
 *          - Rally point operations
 *          - File transfer (FTP)
 *          - Plus many standard MAVLink features
 *          
 *          Usage by ground stations:
 *          - Request AUTOPILOT_VERSION message on connect
 *          - Parse capabilities bitmask
 *          - Enable/disable UI elements based on capabilities
 *          - Example: Show offboard control button if position target supported
 *          - Example: Gray out terrain following if not available
 *          
 *          Capability negotiation:
 *          - GCS queries capabilities during connection
 *          - Vehicle responds with supported features
 *          - GCS adapts interface to available features
 *          - Prevents user confusion from unavailable features
 *          - Enables graceful degradation
 *          
 *          Compile-time variations:
 *          - Terrain capability: Only if AP_TERRAIN_AVAILABLE defined
 *          - Other capabilities: Always present in Copter
 *          - Feature flags affect compiled capabilities
 *          
 *          Runtime variations:
 *          - Terrain: Depends on parameter enable
 *          - All others: Static for given build
 *          - Capabilities don't change during flight
 *          
 * @return Bitmask of MAV_PROTOCOL_CAPABILITY values indicating supported features
 *         Includes copter-specific capabilities ORed with base class capabilities
 * 
 * @note Capabilities reported here must match actual vehicle behavior
 * @note Ground stations use this to adapt UI and available commands
 * @note Terrain capability conditional on compile flag and runtime enable
 * 
 * @see GCS_MAVLINK::capabilities() for base class capabilities
 * @see MAV_PROTOCOL_CAPABILITY enum in MAVLink for capability definitions
 */
uint64_t GCS_MAVLINK_Copter::capabilities() const
{
    return (MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |               // Floating-point mission items
            MAV_PROTOCOL_CAPABILITY_MISSION_INT |                 // Integer-encoded mission items (preferred)
            MAV_PROTOCOL_CAPABILITY_COMMAND_INT |                 // Integer parameter commands
            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED | // Local NED position control
            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT | // Global GPS position control
            MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |          // Emergency termination command
            MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |         // Direct attitude control
#if AP_TERRAIN_AVAILABLE
            // Terrain capability: Only if compiled in AND enabled at runtime
            (copter.terrain.enabled() ? MAV_PROTOCOL_CAPABILITY_TERRAIN : 0) |
#endif
            GCS_MAVLINK::capabilities());  // Add all base class capabilities
}

/**
 * @brief Report current landed state of vehicle for ground station display
 * 
 * @details Returns vehicle's landing/takeoff state for EXTENDED_SYS_STATE message.
 *          Provides clear indication of vehicle flight phase to ground station,
 *          enabling appropriate UI display and user alerts.
 *          
 *          Reported in EXTENDED_SYS_STATE message (ID 245):
 *          - Sent periodically with system status updates
 *          - Used by GCS for status display and warnings
 *          - Helps user understand current vehicle state
 *          - Important for safety awareness
 *          
 *          State determination priority (checked in order):
 *          
 *          1. ON_GROUND (highest priority):
 *             - Checked first: copter.ap.land_complete flag
 *             - Set when vehicle has landed and settled
 *             - Motors may still be armed
 *             - Throttle at zero or idle
 *             - No vertical or horizontal movement
 *             - Vehicle confirmed on surface
 *             
 *          2. LANDING:
 *             - Active landing sequence in progress
 *             - Flight mode reports is_landing() == true
 *             - Descending to ground
 *             - May be in Land mode or auto-land
 *             - Touchdown imminent
 *             
 *          3. TAKEOFF:
 *             - Active takeoff sequence in progress
 *             - Flight mode reports is_taking_off() == true
 *             - Climbing from ground
 *             - May be in TakeOff mode or auto-takeoff
 *             - Gaining altitude
 *             
 *          4. IN_AIR (default):
 *             - Normal flight operations
 *             - Not landing, taking off, or on ground
 *             - Could be hovering, cruising, loitering
 *             - All other flight states
 *          
 *          State transitions:
 *          - Ground → Takeoff: User takeoff command or mission start
 *          - Takeoff → In Air: Reached target takeoff altitude
 *          - In Air → Landing: Land mode or auto-land command
 *          - Landing → Ground: Touchdown and settle complete
 *          
 *          Ground station usage:
 *          - Display landing/takeoff status to user
 *          - Show appropriate icons/indicators
 *          - Enable/disable commands based on state
 *          - Example: Disable takeoff command when already in air
 *          - Example: Show landing progress during descent
 *          
 *          Safety implications:
 *          - ON_GROUND: Safe to approach (if disarmed)
 *          - LANDING: Stay clear, touchdown imminent
 *          - TAKEOFF: Stay clear, vehicle gaining altitude
 *          - IN_AIR: Maintain safe distance
 *          
 *          land_complete flag details:
 *          - Set when vehicle detected on ground
 *          - Based on: low throttle, no movement, rangefinder
 *          - May take several seconds to confirm
 *          - Cleared immediately on liftoff
 *          - Used throughout codebase for state logic
 *          
 *          Flight mode landing/takeoff indicators:
 *          - Each mode can report is_landing() or is_taking_off()
 *          - Land mode: Always reports is_landing()
 *          - Auto mode: Reports is_landing() during auto-land
 *          - Guided mode: Reports state based on commanded action
 *          - Loiter/AltHold: Neither landing nor taking off
 *          
 *          Typical state sequences:
 *          
 *          Normal flight:
 *          - ON_GROUND (armed) → TAKEOFF → IN_AIR → LANDING → ON_GROUND
 *          
 *          Auto mission:
 *          - ON_GROUND → TAKEOFF → IN_AIR (waypoints) → LANDING → ON_GROUND
 *          
 *          Manual flight:
 *          - ON_GROUND → IN_AIR (immediate) → IN_AIR → LANDING → ON_GROUND
 *          
 *          Edge cases:
 *          - Crash: May go directly from IN_AIR to ON_GROUND
 *          - Rejected takeoff: TAKEOFF → ON_GROUND
 *          - Aborted landing: LANDING → IN_AIR
 *          
 * @return Current landed state as MAV_LANDED_STATE enum value:
 *         MAV_LANDED_STATE_ON_GROUND: Vehicle on surface, settled
 *         MAV_LANDED_STATE_LANDING: Active landing in progress
 *         MAV_LANDED_STATE_TAKEOFF: Active takeoff in progress
 *         MAV_LANDED_STATE_IN_AIR: Normal flight operations (default)
 * 
 * @note Checked in priority order: ground, landing, takeoff, in-air
 * @note Used by EXTENDED_SYS_STATE message for GCS display
 * @note Important for user situational awareness and safety
 * 
 * @see copter.ap.land_complete for ground detection flag
 * @see Mode::is_landing() for landing state
 * @see Mode::is_taking_off() for takeoff state
 */
MAV_LANDED_STATE GCS_MAVLINK_Copter::landed_state() const
{
    // Priority 1: Check if vehicle is on ground and settled
    // land_complete flag set when touchdown detected and vehicle stable
    if (copter.ap.land_complete) {
        return MAV_LANDED_STATE_ON_GROUND;
    }
    
    // Priority 2: Check if actively landing
    // Flight mode indicates landing sequence in progress
    if (copter.flightmode->is_landing()) {
        return MAV_LANDED_STATE_LANDING;
    }
    
    // Priority 3: Check if actively taking off
    // Flight mode indicates takeoff sequence in progress
    if (copter.flightmode->is_taking_off()) {
        return MAV_LANDED_STATE_TAKEOFF;
    }
    
    // Default: Normal flight operations
    // Not on ground, not landing, not taking off
    return MAV_LANDED_STATE_IN_AIR;
}

/**
 * @brief Send wind speed and direction estimate to ground station
 * 
 * @details Transmits WIND message (ID 168) with estimated wind vector from EKF.
 *          Wind estimation requires airspeed data or EKF3 wind estimation active.
 *          Copters typically use EKF3 synthetic airspeed from velocity/attitude
 *          to estimate wind without a dedicated airspeed sensor.
 *          
 *          WIND message format (ID 168):
 *          - direction: Wind direction in degrees (0-360)
 *          - speed: Wind speed in m/s (horizontal component)
 *          - speed_z: Vertical wind component in m/s (positive up)
 *          
 *          Wind estimation sources (in priority order):
 *          
 *          1. EKF3 wind estimation (most common for copters):
 *             - Estimates wind from GPS velocity vs inertial velocity
 *             - Requires GPS lock and reasonable flight speed
 *             - No airspeed sensor needed
 *             - Enabled by EK3_DRAG_ENABLE parameter
 *             - Accuracy improves with flight speed and maneuvers
 *             - Estimates both horizontal and vertical components
 *             
 *          2. Physical airspeed sensor (if installed):
 *             - Direct measurement of air velocity
 *             - More common on planes than copters
 *             - Provides immediate wind data
 *             - Used by EKF for higher accuracy
 *             
 *          3. No estimation:
 *             - If neither source available, no message sent
 *             - Function returns early without sending
 *             - Prevents sending invalid/zero wind data
 *          
 *          Wind vector coordinate frames:
 *          
 *          Internal representation (NED frame):
 *          - North component: wind.x (positive = wind from south)
 *          - East component: wind.y (positive = wind from west)
 *          - Down component: wind.z (positive = downward wind)
 *          
 *          MAVLink message format (meteorological convention):
 *          - direction: Where wind is coming FROM (0° = north, 90° = east)
 *          - speed: Horizontal wind speed magnitude (always positive)
 *          - speed_z: Vertical component (positive = upward wind, opposite of NED)
 *          
 *          Direction calculation:
 *          - Uses atan2f(-wind.y, -wind.x) for wind FROM direction
 *          - Negatives convert velocity TO wind to wind FROM wind
 *          - Result in radians converted to degrees
 *          - 0° = north, 90° = east, 180° = south, 270° = west
 *          
 *          Ground station usage:
 *          - Display wind arrow on map
 *          - Show wind speed/direction numerically
 *          - Factor into flight planning (range, speed)
 *          - Alert user to high wind conditions
 *          - Log wind for post-flight analysis
 *          
 *          Copter-specific considerations:
 *          
 *          Wind estimation requirements:
 *          - Copters don't have airspeed sensors by default
 *          - EKF3 can estimate wind from GPS vs inertial velocity
 *          - Requires GPS lock and some forward flight
 *          - Hovering copter has poor wind estimation
 *          - Moving copter gets better wind estimates
 *          
 *          Validation before sending:
 *          - Check airspeed_vector_true() returns valid data
 *          - This indicates EKF has valid wind estimate
 *          - If false, no wind message sent (return early)
 *          - Prevents sending garbage wind data to GCS
 *          
 *          Accuracy factors:
 *          - Flight speed: Faster = better estimate
 *          - GPS accuracy: Better GPS = better wind estimate
 *          - Flight duration: Estimate improves over time
 *          - Maneuvers: Varied flight improves estimate
 *          - Altitude: Higher altitude may have different winds
 *          
 *          Typical update rate:
 *          - Sent on GCS request or at stream rate
 *          - Usually 1-10 Hz depending on telemetry config
 *          - Not sent if wind estimate unavailable
 *          
 *          Message size: 12 bytes
 *          
 *          Use cases:
 *          - Flight planning: Estimate range with headwind/tailwind
 *          - Safety: Warn about excessive wind
 *          - Performance: Optimize routes based on wind
 *          - Analysis: Post-flight wind condition review
 *          - Competition: Wind compensation strategies
 *          
 * @return None (void function)
 *         Sends WIND message if valid estimate available
 *         Returns early without sending if no valid wind data
 * 
 * @note Copters rely on EKF3 wind estimation without airspeed sensor
 * @note Requires GPS lock and flight movement for valid estimates
 * @note Wind direction uses meteorological convention (FROM direction)
 * @note Vertical wind component sign inverted from NED to MAVLink convention
 * 
 * @see AP::ahrs().airspeed_vector_true() for airspeed validation
 * @see AP::ahrs().wind_estimate() for wind vector
 * @see EK3_DRAG_ENABLE parameter for wind estimation enable
 */
void GCS_MAVLINK_Copter::send_wind() const
{
    // Validate that we have a valid airspeed estimate
    // For copters, this checks if EKF3 wind estimation is active
    // Returns true only if wind estimate is valid
    Vector3f airspeed_vec_bf;
    if (!AP::ahrs().airspeed_vector_true(airspeed_vec_bf)) {
        // No valid airspeed/wind estimate available
        // Copters need EKF3 wind estimation or physical airspeed sensor
        // Don't send WIND message with invalid data
        return;
    }
    
    // Get estimated wind vector from EKF
    // Returns 3D wind vector in NED frame (m/s)
    // wind.x = north component, wind.y = east component, wind.z = down component
    const Vector3f wind = AP::ahrs().wind_estimate();
    
    // Send WIND message (ID 168)
    mavlink_msg_wind_send(
        chan,
        degrees(atan2f(-wind.y, -wind.x)),  // Direction in degrees (meteorological: wind FROM)
                                             // Negatives convert velocity TO to wind FROM
                                             // 0° = north, 90° = east, 180° = south, 270° = west
        wind.length(),                       // Horizontal wind speed magnitude (m/s)
                                             // Always positive, computed from sqrt(x²+y²)
        wind.z);                             // Vertical wind component (m/s)
                                             // Positive = downward wind (NED convention)
                                             // Note: MAVLink convention is positive up
}

/**
 * @brief Calculate target altitude for HIGH_LATENCY2 message
 * 
 * @details Computes vehicle's target altitude for bandwidth-efficient telemetry.
 *          HIGH_LATENCY2 message (ID 235) designed for satellite/low-bandwidth links
 *          where full telemetry not feasible. Provides essential flight data in compact
 *          single message sent at low rate (typically 0.1-1 Hz).
 *          
 *          HIGH_LATENCY2 message purpose:
 *          - Extremely bandwidth-efficient status reporting
 *          - Single message contains all critical flight data
 *          - Designed for Iridium satellite modems (340 byte/min)
 *          - Also useful for LoRa long-range links
 *          - Enables basic monitoring over constrained links
 *          - Used in remote/oceanic operations
 *          
 *          Target altitude calculation:
 *          
 *          Component 1: Current altitude
 *          - global_position_current.alt from AHRS
 *          - Altitude in centimeters (internal format)
 *          - AMSL (Above Mean Sea Level) reference
 *          - From GPS + barometer fusion in EKF
 *          
 *          Component 2: Position error
 *          - copter.pos_control->get_pos_error_U_cm()
 *          - Vertical position error in cm
 *          - "U" = up direction in NED frame
 *          - Difference between target and actual altitude
 *          - Positive = need to climb to reach target
 *          - Negative = need to descend to reach target
 *          
 *          Target = Current + Error:
 *          - If error is +10m, target is 10m above current
 *          - If error is -5m, target is 5m below current
 *          - If error is 0, target equals current (at target)
 *          
 *          Unit conversion:
 *          - Internal: centimeters (cm)
 *          - Message: meters (m)
 *          - Multiply by 0.01 to convert cm to m
 *          
 *          Initialization check:
 *          - Only return valid data if copter.ap.initialised == true
 *          - Before initialization, position/control not valid
 *          - Return 0 if not initialized (safe default)
 *          
 *          Altitude reference frame:
 *          - AMSL (Above Mean Sea Level) from GPS/baro
 *          - Not relative to home or terrain
 *          - Consistent with standard altitude reporting
 *          - Ground station must convert if needed
 *          
 *          Typical use cases:
 *          
 *          Long-range expeditions:
 *          - Aircraft flying beyond normal radio range
 *          - Satellite link provides basic monitoring
 *          - Know target altitude for mission progress
 *          
 *          Oceanic operations:
 *          - Ships/boats with satellite link only
 *          - Monitor vehicle target altitude remotely
 *          - Verify mission execution
 *          
 *          Emergency monitoring:
 *          - Backup telemetry over satellite
 *          - If primary link fails, satellite provides status
 *          - Critical for high-value vehicles
 *          
 *          LoRa long-range:
 *          - 10-100km range but very low bandwidth
 *          - HIGH_LATENCY2 provides complete status
 *          - Target altitude key mission parameter
 *          
 *          Message size constraints:
 *          - HIGH_LATENCY2 total size: 42 bytes
 *          - Altitude: 2 bytes (int16)
 *          - Range: -32768 to +32767 meters
 *          - Sufficient for all aircraft operations
 *          
 *          Related HIGH_LATENCY2 fields:
 *          - latitude, longitude: Current position
 *          - target_distance: Distance to target waypoint
 *          - target_heading: Direction to target
 *          - airspeed: Current speed
 *          - groundspeed: Speed over ground
 *          - battery: Battery percentage
 *          - throttle: Current throttle
 *          - Plus many other compact status fields
 *          
 *          Bandwidth savings example:
 *          - Normal telemetry: ~1000+ bytes/sec
 *          - HIGH_LATENCY2: 42 bytes every 10 sec = 4.2 bytes/sec
 *          - 200x reduction enables satellite operation
 *          
 *          UNUSED_RESULT macro:
 *          - Suppresses compiler warning about ignoring return value
 *          - ahrs.get_location() returns bool (success/failure)
 *          - We use the location output parameter regardless
 *          - Safe because we check initialization flag
 *          
 * @return Target altitude in meters (AMSL reference frame)
 *         Current altitude + vertical position error
 *         0 if vehicle not initialized (safe default)
 *         Range: -32768 to +32767 meters (int16 limits)
 * 
 * @note Only valid when copter.ap.initialised is true
 * @note Returns 0 before initialization to avoid invalid data
 * @note Altitude in AMSL reference frame (not relative to home)
 * @note Part of HIGH_LATENCY2 message for low-bandwidth links
 * 
 * @see HIGH_LATENCY2 message (ID 235) specification
 * @see copter.pos_control->get_pos_error_U_cm() for vertical error
 */
#if HAL_HIGH_LATENCY2_ENABLED
int16_t GCS_MAVLINK_Copter::high_latency_target_altitude() const
{
    // Get AHRS reference for position queries
    AP_AHRS &ahrs = AP::ahrs();
    
    // Get current global position from AHRS/EKF
    // Altitude in centimeters, AMSL reference
    Location global_position_current;
    UNUSED_RESULT(ahrs.get_location(global_position_current));  // Ignore bool return, use output parameter

    // Return units are meters
    if (copter.ap.initialised) {
        // Calculate target = current + error
        // current: global_position_current.alt (cm)
        // error: pos_control->get_pos_error_U_cm() (cm, positive = need to climb)
        // Convert sum from cm to m by multiplying by 0.01
        return 0.01 * (global_position_current.alt + copter.pos_control->get_pos_error_U_cm());
    }
    
    // Not initialized yet, return safe default
    return 0;
    
}

/**
 * @brief Calculate target heading for HIGH_LATENCY2 message
 * 
 * @details Computes bearing to current waypoint for bandwidth-efficient telemetry.
 *          Returns heading in compact half-degree units to fit in uint8 (0-180 range).
 *          Provides ground station with vehicle's intended direction for mission monitoring
 *          over low-bandwidth links (satellite, LoRa).
 *          
 *          HIGH_LATENCY2 heading field:
 *          - Field name: target_heading
 *          - Type: uint8_t (1 byte)
 *          - Units: degrees / 2 (half-degrees)
 *          - Range: 0-180 representing 0-360 degrees
 *          - Resolution: 2 degrees
 *          - Example: Value 90 = 180 degrees
 *          - Example: Value 45 = 90 degrees (east)
 *          - Example: Value 0 = 0 degrees (north)
 *          - Example: Value 180 = 360 degrees (north)
 *          
 *          Calculation steps:
 *          
 *          Step 1: Get waypoint bearing from flight mode
 *          - flightmode->wp_bearing_deg() returns bearing to target
 *          - Range: -180 to +180 degrees (signed)
 *          - Positive = clockwise from north
 *          - Negative = counter-clockwise from north
 *          - Example: 90° = east, -90° = west
 *          
 *          Step 2: Wrap to 0-360 range
 *          - wrap_360() converts -180→180 to 0→360
 *          - Removes negative angles
 *          - Example: -90° becomes 270°
 *          - Example: -180° becomes 180°
 *          - Example: 90° stays 90°
 *          
 *          Step 3: Convert to half-degrees
 *          - Multiply by 0.5 to compress range
 *          - 0-360° becomes 0-180 value
 *          - Fits in uint8_t (max 255)
 *          - Loses 1 degree of precision (acceptable for LOW bandwidth)
 *          
 *          Waypoint bearing per flight mode:
 *          
 *          Guided/Auto modes:
 *          - Bearing to next waypoint in mission
 *          - Updated as vehicle flies mission
 *          - Points toward next mission item
 *          
 *          Loiter mode:
 *          - Bearing to loiter center point
 *          - May be 0 if already at center
 *          
 *          RTL mode:
 *          - Bearing to home location
 *          - Direct line to launch point
 *          
 *          Manual modes (Stabilize, AltHold):
 *          - May return 0 or current heading
 *          - No specific waypoint target
 *          
 *          Land mode:
 *          - Bearing to landing point
 *          - Usually small (already near target)
 *          
 *          Ground station usage:
 *          - Display arrow showing intended direction
 *          - Compare with actual heading to see if on track
 *          - Monitor mission progress
 *          - Detect course deviations
 *          - Useful for verifying mission execution
 *          
 *          Initialization check:
 *          - Only return valid data if copter.ap.initialised
 *          - Before initialization, flight mode not set up
 *          - Return 0 if not initialized (safe default)
 *          - Prevents garbage data on startup
 *          
 *          Accuracy and limitations:
 *          - 2-degree resolution sufficient for monitoring
 *          - Not precision navigation data
 *          - Good enough to verify mission progress
 *          - Bandwidth savings worth precision loss
 *          
 *          Comparison to normal telemetry:
 *          - Normal NAV_CONTROLLER_OUTPUT: 4 bytes for bearing (float)
 *          - HIGH_LATENCY2: 1 byte for bearing (uint8)
 *          - 75% size reduction
 *          - Acceptable precision loss for monitoring
 *          
 *          Typical values:
 *          - 0 (0°): Heading north
 *          - 45 (90°): Heading east  
 *          - 90 (180°): Heading south
 *          - 135 (270°): Heading west
 *          - 180 (360°): Heading north (wrapped)
 *          
 *          Use case example:
 *          - Vehicle on long-range mission over ocean
 *          - Satellite link provides periodic updates
 *          - Target heading shows mission is progressing
 *          - Sudden heading change might indicate issue
 *          - Operator can monitor without full telemetry
 *          
 *          Related telemetry fields:
 *          - target_distance: How far to waypoint
 *          - heading: Actual vehicle heading
 *          - Compare target vs actual to see tracking error
 *          
 * @return Target heading in half-degrees (0-180 representing 0-360°)
 *         Bearing to current waypoint from flight mode
 *         0 if vehicle not initialized
 *         Resolution: 2 degrees
 * 
 * @note Units are degrees/2 to fit in uint8 (0-180 = 0-360°)
 * @note Only valid when copter.ap.initialised is true
 * @note Waypoint bearing depends on current flight mode
 * @note 2-degree resolution acceptable for monitoring applications
 * 
 * @see Mode::wp_bearing_deg() for waypoint bearing
 * @see wrap_360() for angle wrapping
 * @see HIGH_LATENCY2 message (ID 235) specification
 */
uint8_t GCS_MAVLINK_Copter::high_latency_tgt_heading() const
{
    if (copter.ap.initialised) {
        // Return units are deg/2 (half-degrees)
        const Mode *flightmode = copter.flightmode;
        
        // Get bearing to waypoint from flight mode
        // Range: -180 to +180 degrees
        // Then wrap to 0-360 range (removes negatives)
        // Then divide by 2 to compress to 0-180 value
        // Example: 270° becomes 135, 90° becomes 45
        return wrap_360(flightmode->wp_bearing_deg()) * 0.5;
    }
    
    // Not initialized, return safe default
    return 0;     
}
    
/**
 * @brief Calculate distance to target for HIGH_LATENCY2 message
 * 
 * @details Computes distance to current waypoint in decimeters for compact telemetry.
 *          Returns distance in decimeter units to fit larger ranges in uint16 field.
 *          Enables monitoring of mission progress over low-bandwidth satellite/LoRa links.
 *          
 *          HIGH_LATENCY2 distance field:
 *          - Field name: target_distance
 *          - Type: uint16_t (2 bytes)
 *          - Units: decimeters (dm) = 0.1 meters
 *          - Range: 0-65535 dm = 0-6553.5 meters = 0-6.5 km
 *          - Resolution: 0.1 meters (10 cm)
 *          - Example: Value 100 = 10 meters
 *          - Example: Value 1000 = 100 meters
 *          - Example: Value 10000 = 1000 meters = 1 km
 *          
 *          Calculation steps:
 *          
 *          Step 1: Get waypoint distance from flight mode
 *          - flightmode->wp_distance_m() returns distance in meters
 *          - Horizontal distance to target waypoint
 *          - Float value with meter precision
 *          - May be very large for distant waypoints
 *          
 *          Step 2: Clamp to uint16 maximum
 *          - MIN(distance, UINT16_MAX) prevents overflow
 *          - UINT16_MAX = 65535
 *          - Distances beyond 6.5km clamped to maximum
 *          - Prevents wrapping for very distant waypoints
 *          - GCS sees "max distance" for far targets
 *          
 *          Step 3: Convert meters to decimeters
 *          - Divide by 10 to convert meters to decimeters
 *          - Example: 150 meters → 1500 decimeters
 *          - Provides finer resolution than meters
 *          - 10cm resolution adequate for monitoring
 *          
 *          Distance overflow handling:
 *          - If actual distance > 6553.5 meters:
 *            * Distance clamped to 65535 before division
 *            * Result will be 6553 dm = 6.5 km
 *            * GCS knows waypoint is "far away"
 *            * Acceptable for monitoring purposes
 *          - Most missions have waypoints < 6.5 km apart
 *          - Long missions may show clamped distance
 *          
 *          Waypoint distance per flight mode:
 *          
 *          Auto/Guided modes:
 *          - Distance to next waypoint in mission
 *          - Decreases as vehicle approaches
 *          - Updates to next waypoint when reached
 *          
 *          RTL mode:
 *          - Distance to home location
 *          - Straight-line horizontal distance
 *          - May be many kilometers
 *          
 *          Loiter mode:
 *          - Distance to loiter center
 *          - Usually small once established
 *          - Fluctuates with position hold
 *          
 *          Land mode:
 *          - Distance to landing point
 *          - Usually very small (already near target)
 *          - May be zero if landing at current location
 *          
 *          Manual modes (Stabilize, AltHold):
 *          - May return 0 (no specific target)
 *          - Depends on mode implementation
 *          
 *          Ground station usage:
 *          - Display numeric distance to waypoint
 *          - Show progress bar for mission segment
 *          - Estimate time to waypoint (with speed)
 *          - Alert if not making progress
 *          - Verify mission execution remotely
 *          
 *          Initialization check:
 *          - Only return valid data if copter.ap.initialised
 *          - Before initialization, flight mode not valid
 *          - Return 0 if not initialized
 *          - Prevents invalid data on startup
 *          
 *          Resolution analysis:
 *          - Decimeter = 10 cm resolution
 *          - Sufficient for mission monitoring
 *          - Not precision navigation data
 *          - Much finer than typical waypoint radii (2-10m)
 *          
 *          Bandwidth optimization:
 *          - Normal telemetry: 4 bytes (float meters)
 *          - HIGH_LATENCY2: 2 bytes (uint16 decimeters)
 *          - 50% size reduction
 *          - Acceptable range/precision trade-off
 *          
 *          Typical values:
 *          - 0: At waypoint or no target
 *          - 50 (5m): Close to waypoint
 *          - 500 (50m): Approaching waypoint
 *          - 1000 (100m): En route to waypoint
 *          - 10000 (1km): Distant waypoint
 *          - 6553 (6.5km max): Very far or clamped
 *          
 *          Use case example:
 *          - Long-range survey mission over ocean
 *          - Satellite link every 10 seconds
 *          - Distance shows progress toward next waypoint
 *          - Operator confirms mission proceeding normally
 *          - If distance not decreasing, may indicate problem
 *          
 *          Mission monitoring:
 *          - Combine with target_heading for full picture
 *          - Distance + heading = mission progress vector
 *          - Can estimate arrival time with speed
 *          - Detect mission deviations or holds
 *          
 *          Precision considerations:
 *          - 10cm resolution fine for monitoring
 *          - Not for precision navigation
 *          - Waypoint acceptance radius typically 2-5m
 *          - Resolution 20-50x finer than needed
 *          
 * @return Distance to target waypoint in decimeters (0.1 meter units)
 *         Range: 0-6553 dm (0-6.5 km)
 *         Clamped to maximum for very distant waypoints (>6.5km)
 *         0 if vehicle not initialized
 *         Resolution: 10 centimeters
 * 
 * @note Units are decimeters (dm) = 0.1 meters
 * @note Clamped to 6.5km maximum due to uint16 field size
 * @note Only valid when copter.ap.initialised is true
 * @note Distance depends on current flight mode and target
 * @note 10cm resolution sufficient for mission monitoring
 * 
 * @see Mode::wp_distance_m() for waypoint distance in meters
 * @see HIGH_LATENCY2 message (ID 235) specification
 */
uint16_t GCS_MAVLINK_Copter::high_latency_tgt_dist() const
{
    if (copter.ap.initialised) {
        // Return units are dm (decimeters = 0.1 meters)
        const Mode *flightmode = copter.flightmode;
        
        // Get distance to waypoint in meters from flight mode
        // Clamp to UINT16_MAX (65535) before converting to prevent overflow
        // Then divide by 10 to convert meters to decimeters
        // Result fits in uint16: 0-6553 dm (0-6.5 km range)
        return MIN(flightmode->wp_distance_m(), UINT16_MAX) / 10;
    }
    
    // Not initialized, return safe default
    return 0;
}

/**
 * @brief Calculate target airspeed for HIGH_LATENCY2 message
 * 
 * @details Computes target velocity magnitude in scaled units for bandwidth efficiency.
 *          For copters, "airspeed" is actually target velocity from position controller.
 *          Returns speed in m/s*5 units to fit wider range in uint8 with acceptable resolution.
 *          
 *          HIGH_LATENCY2 airspeed field:
 *          - Field name: airspeed_sp (airspeed setpoint)
 *          - Type: uint8_t (1 byte)
 *          - Units: m/s × 5 (scaled m/s)
 *          - Range: 0-255 representing 0-51 m/s
 *          - Resolution: 0.2 m/s
 *          - Example: Value 25 = 5 m/s
 *          - Example: Value 50 = 10 m/s
 *          - Example: Value 100 = 20 m/s
 *          
 *          Copter interpretation of "airspeed":
 *          
 *          Copters don't have traditional airspeed:
 *          - Fixed-wing planes have airspeed sensors
 *          - Copters measure velocity over ground via GPS
 *          - "Airspeed" for copters = target velocity magnitude
 *          - Represents desired movement speed
 *          - Not actual air-relative speed
 *          
 *          Target velocity source:
 *          - copter.pos_control->get_vel_target_NEU_cms()
 *          - NEU = North-East-Up coordinate frame
 *          - Target velocity vector from position controller
 *          - Units: cm/s (centimeters per second)
 *          - 3D vector: north, east, up components
 *          
 *          Calculation steps:
 *          
 *          Step 1: Get target velocity vector
 *          - get_vel_target_NEU_cms() returns 3D velocity target
 *          - NEU frame: north component, east component, up component
 *          - Each component in cm/s
 *          - Vector represents commanded velocity
 *          
 *          Step 2: Calculate magnitude
 *          - .length() computes 3D vector magnitude
 *          - sqrt(north² + east² + up²)
 *          - Total speed regardless of direction
 *          - Includes horizontal and vertical components
 *          
 *          Step 3: Convert units and scale
 *          - Multiply by 5.0e-2 = 0.05
 *          - First: cm/s to m/s (divide by 100 = 0.01)
 *          - Then: scale by 5 for compact storage
 *          - Combined: 0.01 * 5 = 0.05
 *          - Result: (cm/s) * 0.05 = (m/s * 5)
 *          
 *          Step 4: Clamp to uint8 range
 *          - MIN(value, UINT8_MAX) prevents overflow
 *          - UINT8_MAX = 255
 *          - Speeds > 51 m/s clamped to 255 (51 m/s)
 *          - Copters rarely exceed 25 m/s, so adequate
 *          
 *          Target velocity vs actual velocity:
 *          - This reports TARGET (desired) velocity
 *          - Not actual measured velocity
 *          - Shows what position controller is commanding
 *          - Actual velocity reported in separate field
 *          - Difference indicates tracking error
 *          
 *          Flight mode implications:
 *          
 *          Auto/Guided modes:
 *          - Target velocity based on waypoint navigation
 *          - May be configured cruise speed (WPNAV_SPEED)
 *          - Changes during acceleration/deceleration
 *          - Zero when holding position
 *          
 *          Loiter mode:
 *          - Target velocity usually zero (hold position)
 *          - Non-zero when correcting for wind drift
 *          - Small values for position maintenance
 *          
 *          Manual modes (Stabilize):
 *          - Target velocity based on pilot stick input
 *          - Can be zero to maximum configured speed
 *          - Varies continuously with pilot commands
 *          
 *          RTL mode:
 *          - Target velocity toward home
 *          - Based on RTL_SPEED parameter
 *          - Typically 5-15 m/s for copters
 *          
 *          Land mode:
 *          - Primarily vertical velocity (descent)
 *          - Horizontal component small (stabilization)
 *          - Magnitude includes vertical descent rate
 *          
 *          Resolution analysis:
 *          - 0.2 m/s resolution (1/5 m/s)
 *          - Adequate for monitoring purposes
 *          - Typical copter cruise: 5-20 m/s
 *          - 25-100 discrete values in normal range
 *          - Sufficient granularity for status monitoring
 *          
 *          Range analysis:
 *          - Maximum: 51 m/s (183 km/h)
 *          - Well above typical copter speeds
 *          - Racing copters: ~30 m/s max
 *          - Survey copters: ~15 m/s typical
 *          - Slow copters: ~5 m/s
 *          - Range adequate for all copters
 *          
 *          Bandwidth optimization:
 *          - Normal telemetry: 12 bytes (3D float vector)
 *          - HIGH_LATENCY2: 1 byte (scaled magnitude)
 *          - 92% size reduction
 *          - Loses directional information (see heading field)
 *          - Acceptable for monitoring applications
 *          
 *          Ground station usage:
 *          - Display target speed numeric value
 *          - Compare with actual speed to see tracking
 *          - Estimate time to waypoint
 *          - Verify mission profile execution
 *          - Detect speed anomalies
 *          
 *          Initialization check:
 *          - Only return valid data if copter.ap.initialised
 *          - Before initialization, pos_control not set up
 *          - Return 0 if not initialized
 *          - Prevents invalid/garbage data
 *          
 *          Typical values:
 *          - 0: Hovering or holding position
 *          - 25 (5 m/s): Slow cruise
 *          - 50 (10 m/s): Normal cruise
 *          - 75 (15 m/s): Fast cruise
 *          - 100 (20 m/s): Racing speed
 *          
 *          Use case example:
 *          - Survey mission over satellite link
 *          - Target airspeed shows intended survey speed
 *          - If actual speed differs, indicates wind/problem
 *          - Operator monitors mission execution remotely
 *          
 * @return Target velocity magnitude in scaled units (m/s × 5)
 *         Range: 0-255 representing 0-51 m/s
 *         3D velocity target magnitude from position controller
 *         Clamped to 255 (51 m/s) for very high speeds
 *         0 if vehicle not initialized
 *         Resolution: 0.2 m/s
 * 
 * @note Units are m/s × 5 (multiply by 0.2 to get m/s)
 * @note For copters, "airspeed" is target velocity magnitude
 * @note Includes horizontal and vertical velocity components
 * @note Clamped to 51 m/s maximum (adequate for all copters)
 * @note Only valid when copter.ap.initialised is true
 * @note 0.2 m/s resolution sufficient for monitoring
 * 
 * @see copter.pos_control->get_vel_target_NEU_cms() for target velocity vector
 * @see HIGH_LATENCY2 message (ID 235) specification
 */
uint8_t GCS_MAVLINK_Copter::high_latency_tgt_airspeed() const
{
    if (copter.ap.initialised) {
        // Return units are m/s*5 (scaled m/s)
        
        // Get 3D target velocity vector from position controller
        // NEU frame (North-East-Up), units are cm/s
        // Calculate magnitude: sqrt(north² + east² + up²)
        // Convert cm/s to m/s and scale by 5: multiply by 5.0e-2 (0.05)
        // Clamp to UINT8_MAX (255) to prevent overflow
        // Result: 0-255 representing 0-51 m/s in 0.2 m/s steps
        return MIN(copter.pos_control->get_vel_target_NEU_cms().length() * 5.0e-2, UINT8_MAX);
    }
    
    // Not initialized, return safe default
    return 0;  
}

/**
 * @brief Calculate wind speed for HIGH_LATENCY2 message
 * 
 * @details Computes estimated wind speed magnitude in scaled units for bandwidth efficiency.
 *          Wind estimation from EKF3 provides environmental data for mission monitoring
 *          over low-bandwidth satellite/LoRa links. Returns speed in m/s*5 units.
 *          
 *          HIGH_LATENCY2 wind speed field:
 *          - Field name: wind_speed
 *          - Type: uint8_t (1 byte)
 *          - Units: m/s × 5 (scaled m/s)
 *          - Range: 0-255 representing 0-51 m/s
 *          - Resolution: 0.2 m/s
 *          - Example: Value 25 = 5 m/s
 *          - Example: Value 50 = 10 m/s (typical)
 *          - Example: Value 75 = 15 m/s (strong wind)
 *          
 *          Wind estimation validation:
 *          
 *          Prerequisite check:
 *          - AP::ahrs().airspeed_vector_true(airspeed_vec_bf)
 *          - Returns true only if wind estimation valid
 *          - For copters, checks EKF3 wind estimation active
 *          - Requires GPS lock and flight movement
 *          - If false, no valid wind data available
 *          
 *          Wind estimation sources:
 *          - EKF3 wind estimation (most common for copters)
 *          - Physical airspeed sensor (rare on copters)
 *          - Estimates from GPS vs inertial velocity difference
 *          - Accuracy improves with flight speed and maneuvers
 *          
 *          Calculation steps:
 *          
 *          Step 1: Validate wind estimation available
 *          - airspeed_vector_true() checks EKF wind valid
 *          - Must return true to proceed
 *          - If false, return 0 (no valid wind data)
 *          - Prevents sending invalid wind information
 *          
 *          Step 2: Get wind vector from EKF
 *          - AP::ahrs().wind_estimate() returns 3D wind vector
 *          - NED frame: north, east, down components
 *          - Units: m/s
 *          - Includes horizontal and vertical wind
 *          
 *          Step 3: Calculate magnitude
 *          - wind.length() computes 3D magnitude
 *          - sqrt(north² + east² + down²)
 *          - Total wind speed regardless of direction
 *          - Typically dominated by horizontal components
 *          
 *          Step 4: Scale for transmission
 *          - Multiply by 5 to scale m/s to (m/s × 5)
 *          - Fits wider range in uint8
 *          - 0.2 m/s resolution
 *          - Implicit clamp to 255 max (51 m/s)
 *          
 *          Wind speed interpretation:
 *          
 *          Magnitude only:
 *          - This field reports speed magnitude only
 *          - Direction in separate field (high_latency_wind_direction)
 *          - Together provide complete 2D wind vector
 *          - Vertical component included in magnitude
 *          
 *          Typical wind speeds:
 *          - 0-5 m/s: Calm to light breeze
 *          - 5-10 m/s: Moderate breeze
 *          - 10-15 m/s: Strong breeze
 *          - 15-20 m/s: High wind warning
 *          - 20-25 m/s: Gale force
 *          - >25 m/s: Storm conditions
 *          
 *          Copter operations by wind speed:
 *          - 0-5 m/s: Ideal conditions
 *          - 5-10 m/s: Normal operations
 *          - 10-15 m/s: Challenging flight
 *          - 15-20 m/s: Expert pilots only
 *          - >20 m/s: Dangerous, avoid flight
 *          
 *          Resolution adequacy:
 *          - 0.2 m/s resolution
 *          - Sufficient for safety monitoring
 *          - Can distinguish wind categories
 *          - Not precision meteorology
 *          - Good enough for flight decisions
 *          
 *          Range analysis:
 *          - Maximum: 51 m/s (hurricane force)
 *          - Well above safe flight limits
 *          - Copters shouldn't fly above ~15 m/s
 *          - Range adequate for all conditions
 *          - Values >40 indicate extreme weather
 *          
 *          Ground station usage:
 *          
 *          Safety monitoring:
 *          - Display wind speed to operator
 *          - Alert if exceeding safe limits
 *          - Color-code based on severity
 *          - Compare with vehicle capabilities
 *          
 *          Mission planning:
 *          - Estimate range impact from headwind
 *          - Plan routes to minimize wind exposure
 *          - Decide if conditions acceptable
 *          - Abort if wind too high
 *          
 *          Performance analysis:
 *          - Explain reduced ground speed
 *          - Account for increased battery consumption
 *          - Understand position holding difficulty
 *          - Post-flight condition review
 *          
 *          Wind estimation limitations:
 *          
 *          Copter-specific:
 *          - No airspeed sensor typically
 *          - Relies on EKF3 estimation
 *          - Requires GPS and flight movement
 *          - Hovering copter: poor wind estimate
 *          - Moving copter: better wind estimate
 *          
 *          Accuracy factors:
 *          - Flight speed: Faster = better estimate
 *          - Maneuvers: Varied flight improves estimate
 *          - GPS quality: Better GPS = better wind
 *          - Time: Estimate converges over minutes
 *          
 *          Invalid conditions:
 *          - GPS lost: No wind estimation
 *          - Stationary: Wind estimate poor
 *          - Just powered on: Not converged yet
 *          - Function returns 0 if invalid
 *          
 *          Bandwidth efficiency:
 *          - Normal WIND message: 12 bytes (3D float)
 *          - HIGH_LATENCY2: 1 byte wind speed + 1 byte direction
 *          - 83% size reduction
 *          - Acceptable precision loss
 *          
 *          Related telemetry:
 *          - wind_direction: Where wind is from
 *          - airspeed: Vehicle speed through air
 *          - groundspeed: Vehicle speed over ground
 *          - Together explain flight performance
 *          
 *          Use case example:
 *          - Long-range flight over ocean
 *          - Satellite link provides periodic updates
 *          - Wind speed shows conditions
 *          - If too high, operator commands RTL
 *          - Safety decision based on wind data
 *          
 * @return Wind speed magnitude in scaled units (m/s × 5)
 *         Range: 0-255 representing 0-51 m/s
 *         0 if wind estimation not available
 *         Resolution: 0.2 m/s
 *         Includes horizontal and vertical components
 * 
 * @note Units are m/s × 5 (multiply by 0.2 to get m/s)
 * @note Returns 0 if wind estimation invalid
 * @note Copters use EKF3 wind estimation (no airspeed sensor)
 * @note Requires GPS lock and flight movement for valid estimate
 * @note Magnitude only; direction in separate field
 * @note 0.2 m/s resolution adequate for monitoring
 * 
 * @see AP::ahrs().airspeed_vector_true() for wind validity check
 * @see AP::ahrs().wind_estimate() for wind vector
 * @see high_latency_wind_direction() for wind direction
 * @see HIGH_LATENCY2 message (ID 235) specification
 */
uint8_t GCS_MAVLINK_Copter::high_latency_wind_speed() const
{
    Vector3f airspeed_vec_bf;
    Vector3f wind;
    
    // Return units are m/s*5 (scaled m/s)
    
    // Validate that wind estimation is available and valid
    // For copters, checks if EKF3 wind estimation active
    // Returns true only if wind estimate reliable
    if (AP::ahrs().airspeed_vector_true(airspeed_vec_bf)) {
        // Get 3D wind vector from EKF
        // NED frame (North-East-Down), units are m/s
        wind = AP::ahrs().wind_estimate();
        
        // Calculate magnitude: sqrt(north² + east² + down²)
        // Scale by 5 for compact transmission: m/s × 5
        // Result: 0-255 representing 0-51 m/s in 0.2 m/s steps
        // Implicit clamp to UINT8_MAX (255) if wind > 51 m/s
        return wind.length() * 5;
    }
    
    // Wind estimation not available or invalid
    // Return 0 to indicate no valid wind data
    // Happens when: GPS lost, hovering, just started, etc.
    return 0; 
}

/**
 * @brief Calculate wind direction for HIGH_LATENCY2 message
 * 
 * @details Computes estimated wind direction in compressed half-degree units.
 *          Provides wind FROM direction using meteorological convention for mission
 *          monitoring over low-bandwidth satellite/LoRa links. Complements wind speed
 *          to give complete 2D wind vector for flight performance analysis.
 *          
 *          HIGH_LATENCY2 wind direction field:
 *          - Field name: wind_heading
 *          - Type: uint8_t (1 byte)
 *          - Units: degrees / 2 (half-degrees)
 *          - Range: 0-180 representing 0-360 degrees
 *          - Resolution: 2 degrees
 *          - Meteorological convention: Wind FROM direction
 *          - Example: Value 0 = 0° (wind from north)
 *          - Example: Value 45 = 90° (wind from east)
 *          - Example: Value 90 = 180° (wind from south)
 *          - Example: Value 135 = 270° (wind from west)
 *          
 *          Wind estimation validation:
 *          
 *          Prerequisite check:
 *          - AP::ahrs().airspeed_vector_true(airspeed_vec_bf)
 *          - Same validation as wind speed
 *          - Returns true only if wind estimation valid
 *          - For copters, checks EKF3 wind estimation active
 *          - If false, return 0 (no valid wind data)
 *          
 *          Calculation steps:
 *          
 *          Step 1: Validate wind estimation
 *          - airspeed_vector_true() checks validity
 *          - Must return true to proceed
 *          - If false, return 0 (default north)
 *          - Prevents invalid direction data
 *          
 *          Step 2: Get wind vector from EKF
 *          - AP::ahrs().wind_estimate() returns 3D vector
 *          - NED frame: north (x), east (y), down (z)
 *          - Units: m/s
 *          - Focus on horizontal components (x, y)
 *          
 *          Step 3: Calculate direction angle
 *          - atan2f(-wind.y, -wind.x)
 *          - Computes angle from wind velocity TO
 *          - Negatives convert to wind FROM direction
 *          - Result in radians
 *          
 *          Step 4: Convert to degrees
 *          - degrees() converts radians to degrees
 *          - Range: -180 to +180 degrees
 *          
 *          Step 5: Wrap to 0-360
 *          - wrap_360() removes negative angles
 *          - Converts -180→180 to 0→360
 *          - Standard compass heading format
 *          
 *          Step 6: Compress to half-degrees
 *          - Divide by 2 for compact storage
 *          - 0-360° becomes 0-180 value
 *          - Fits in uint8_t
 *          - 2-degree resolution
 *          
 *          Wind direction conventions:
 *          
 *          Meteorological convention (used here):
 *          - Wind FROM direction (where wind originates)
 *          - 0° = wind from north (blowing south)
 *          - 90° = wind from east (blowing west)
 *          - 180° = wind from south (blowing north)
 *          - 270° = wind from west (blowing east)
 *          - Standard for weather reporting
 *          
 *          Mathematical convention (NOT used):
 *          - Wind TO direction (where wind is going)
 *          - Opposite of meteorological
 *          - Common in physics but confusing for pilots
 *          
 *          Coordinate frame details:
 *          
 *          NED frame wind vector:
 *          - wind.x = north component (positive = toward north)
 *          - wind.y = east component (positive = toward east)
 *          - wind.z = down component (not used for direction)
 *          
 *          Angle calculation with negatives:
 *          - atan2f(-wind.y, -wind.x)
 *          - Negatives convert velocity TO to wind FROM
 *          - Example: wind velocity toward north means wind from south
 *          - Example: wind velocity toward east means wind from west
 *          
 *          Direction examples:
 *          
 *          Northerly wind (from north):
 *          - wind.x negative, wind.y ≈ 0
 *          - atan2f(0, positive) = 0°
 *          - Result: 0° (north)
 *          
 *          Easterly wind (from east):
 *          - wind.x ≈ 0, wind.y negative
 *          - atan2f(positive, 0) = 90°
 *          - Result: 90° (east)
 *          
 *          Southerly wind (from south):
 *          - wind.x positive, wind.y ≈ 0
 *          - atan2f(0, negative) = 180°
 *          - Result: 180° (south)
 *          
 *          Westerly wind (from west):
 *          - wind.x ≈ 0, wind.y positive
 *          - atan2f(negative, 0) = 270°
 *          - Result: 270° (west)
 *          
 *          Resolution analysis:
 *          - 2-degree resolution
 *          - 180 discrete values
 *          - Adequate for monitoring
 *          - Not precision meteorology
 *          - Sufficient for flight planning
 *          
 *          Ground station usage:
 *          
 *          Wind visualization:
 *          - Display wind arrow on map
 *          - Arrow points FROM wind origin
 *          - Length proportional to wind speed
 *          - Standard weather map convention
 *          
 *          Flight performance analysis:
 *          - Headwind: vehicle heading opposite wind direction
 *          - Tailwind: vehicle heading same as wind direction
 *          - Crosswind: vehicle heading perpendicular
 *          - Affects ground speed and battery consumption
 *          
 *          Mission planning:
 *          - Plan routes with tailwind sections
 *          - Avoid prolonged headwind segments
 *          - Account for crosswind drift
 *          - Optimize for range/endurance
 *          
 *          Bandwidth optimization:
 *          - Normal WIND message: Direction as float (4 bytes)
 *          - HIGH_LATENCY2: Direction as uint8 (1 byte)
 *          - 75% size reduction
 *          - Acceptable precision loss
 *          
 *          Combined with wind speed:
 *          - Speed + direction = complete 2D wind vector
 *          - Reconstruct horizontal wind components
 *          - North component = speed × cos(direction)
 *          - East component = speed × sin(direction)
 *          
 *          Typical directions:
 *          - 0 (0°): From north (northerly)
 *          - 45 (90°): From east (easterly)
 *          - 90 (180°): From south (southerly)
 *          - 135 (270°): From west (westerly)
 *          - 22 (45°): From northeast
 *          - 67 (135°): From southeast
 *          
 *          Flight impact by direction:
 *          
 *          Mission heading 0° (north):
 *          - Wind 0° (north): Tailwind, increased ground speed
 *          - Wind 90° (east): Crosswind from right
 *          - Wind 180° (south): Headwind, reduced ground speed
 *          - Wind 270° (west): Crosswind from left
 *          
 *          Mission heading 90° (east):
 *          - Wind 0° (north): Crosswind from left
 *          - Wind 90° (east): Tailwind
 *          - Wind 180° (south): Crosswind from right
 *          - Wind 270° (west): Headwind
 *          
 *          Use case example:
 *          - Survey mission over satellite link
 *          - Wind from 270° (west) at 10 m/s
 *          - Mission legs running north-south
 *          - Expect crosswind drift on all legs
 *          - Operator monitors progress, sees expected behavior
 *          - If ground track drifting, wind data explains why
 *          
 *          Invalid data handling:
 *          - Returns 0 if wind estimation invalid
 *          - 0 = north, not "unknown"
 *          - GCS should check wind speed also
 *          - Speed = 0, Direction = 0 indicates no wind data
 *          - Speed > 0, Direction meaningful
 *          
 *          Accuracy considerations:
 *          - Same limitations as wind speed
 *          - Requires GPS and flight movement
 *          - Converges over time
 *          - Better with varied flight paths
 *          - 2° resolution adequate for monitoring
 *          
 * @return Wind direction in half-degrees (0-180 representing 0-360°)
 *         Meteorological convention: Wind FROM direction
 *         0 = north, 45 = east, 90 = south, 135 = west
 *         0 if wind estimation not available
 *         Resolution: 2 degrees
 * 
 * @note Units are degrees/2 (multiply by 2 to get degrees)
 * @note Meteorological convention: Wind FROM direction
 * @note Returns 0 if wind estimation invalid (default north)
 * @note Copters use EKF3 wind estimation (no airspeed sensor)
 * @note Requires GPS lock and flight movement for valid estimate
 * @note Combine with wind speed for complete 2D wind vector
 * @note 2-degree resolution adequate for flight planning
 * 
 * @see AP::ahrs().airspeed_vector_true() for wind validity check
 * @see AP::ahrs().wind_estimate() for wind vector
 * @see high_latency_wind_speed() for wind speed magnitude
 * @see wrap_360() for angle wrapping
 * @see HIGH_LATENCY2 message (ID 235) specification
 */
uint8_t GCS_MAVLINK_Copter::high_latency_wind_direction() const
{
    Vector3f airspeed_vec_bf;
    Vector3f wind;
    
    // Return units are deg/2 (half-degrees)
    
    // Validate that wind estimation is available and valid
    // Same check as wind speed - ensures consistent data
    if (AP::ahrs().airspeed_vector_true(airspeed_vec_bf)) {
        // Get 3D wind vector from EKF
        // NED frame: wind.x = north, wind.y = east, wind.z = down
        wind = AP::ahrs().wind_estimate();
        
        // Calculate wind FROM direction (meteorological convention)
        // atan2f(-wind.y, -wind.x): Angle of wind origin
        //   - Negatives convert velocity TO to wind FROM
        //   - Returns radians: -π to +π
        // degrees(): Convert radians to degrees: -180 to +180
        // wrap_360(): Convert -180→180 to 0→360 (remove negatives)
        // / 2: Compress to half-degrees for uint8 storage
        // Result: 0-180 representing 0-360° in 2° steps
        return wrap_360(degrees(atan2f(-wind.y, -wind.x))) / 2;
    }
    
    // Wind estimation not available or invalid
    // Return 0 (north as default)
    // GCS should check wind speed = 0 to detect invalid data
    return 0;
}
#endif // HAL_HIGH_LATENCY2_ENABLED

/**
 * @brief Send available flight mode for AVAILABLE_MODES message
 * 
 * @details Sends information about a specific flight mode that is available on this vehicle.
 *          Called iteratively by MAVLink protocol to enumerate all supported flight modes,
 *          enabling ground control stations to display only valid mode options to operators.
 *          Essential for mode selection UI and preventing commands for unsupported modes.
 *          
 *          AVAILABLE_MODES message purpose:
 *          - Reports which flight modes are compiled and available
 *          - Enables dynamic GCS UI based on vehicle capabilities
 *          - Prevents operator from selecting unavailable modes
 *          - Supports feature flags (compile-time mode exclusion)
 *          - Allows custom scripting modes to be enumerated
 *          
 *          Protocol iteration:
 *          - GCS requests modes sequentially by index
 *          - Index starts at 1 (not 0)
 *          - Each call returns one mode's information
 *          - Function returns total mode count
 *          - GCS iterates until all modes received
 *          - Index beyond count indicates completion
 *          
 *          Mode array structure:
 *          - Static array contains pointers to mode objects
 *          - Order defines iteration sequence
 *          - Includes only modes enabled by feature flags
 *          - Conditional compilation excludes disabled modes
 *          - AUTO appears twice (AUTO and AUTO_RTL variants)
 *          
 *          Index parameter:
 *          - 1-based indexing (1 = first mode)
 *          - NOT mode number (different concept)
 *          - Converted to 0-based internally (index_zero)
 *          - Out-of-range index returns count without sending
 *          - Sequential iteration: 1, 2, 3, ..., count
 *          
 *          Feature flag compilation:
 *          - Each mode has associated feature flag
 *          - Example: #if MODE_AUTO_ENABLED
 *          - Modes excluded at compile time if disabled
 *          - Reduces binary size for space-constrained boards
 *          - Mode count varies by configuration
 *          
 *          Dynamic mode count:
 *          - Base modes: Compiled-in standard modes
 *          - Custom scripting modes: Runtime-added guided modes
 *          - Total = base_mode_count + scripting modes
 *          - Scripting modes discovered at runtime
 *          - Count can vary between vehicles
 *          
 * @param[in] index Mode index to send (1-based, not mode number)
 *                  Valid range: 1 to total mode count
 *                  Index 1 = first available mode
 *                  Index beyond count = query total without sending
 * 
 * @return Total number of available modes on this vehicle
 *         Used by GCS to determine iteration endpoint
 *         Same value returned for all index values
 * 
 * @note Index starts at 1, not 0 (MAVLink protocol convention)
 * @note Index is NOT mode number (different numbering scheme)
 * @note Returns mode count even if index out of range
 * @note AUTO mode appears twice (AUTO and AUTO_RTL)
 * @note Scripting modes add to base mode count dynamically
 * 
 * @see Mode::Number enum for mode numbers
 * @see mavlink_msg_available_modes_send() for message transmission
 * @see copter.mode_guided_custom[] for scripting modes
 */
// Send the mode with the given index (not mode number!) return the total number of modes
// Index starts at 1
uint8_t GCS_MAVLINK_Copter::send_available_mode(uint8_t index) const
{
    // Static array of available mode pointers
    // Order determines iteration sequence for AVAILABLE_MODES message
    // Each entry conditionally compiled based on feature flags
    // Note: AUTO appears twice (AUTO_RTL and AUTO are distinct)
    const Mode* modes[] {
#if MODE_AUTO_ENABLED
        &copter.mode_auto, // Index 0: This auto is actually auto RTL! (Special handling below)
        &copter.mode_auto, // Index 1: This one is really is auto! (Standard AUTO mode)
#endif
#if MODE_ACRO_ENABLED
        &copter.mode_acro, // Rate-controlled manual flight (expert mode)
#endif
        &copter.mode_stabilize, // Always available: Self-leveling manual flight
        &copter.mode_althold,   // Always available: Altitude hold with manual horizontal
#if MODE_CIRCLE_ENABLED
        &copter.mode_circle, // Automated circular flight path
#endif
#if MODE_LOITER_ENABLED
        &copter.mode_loiter, // GPS position hold
#endif
#if MODE_GUIDED_ENABLED
        &copter.mode_guided, // GCS/companion computer control
#endif
        &copter.mode_land,   // Always available: Automated landing
#if MODE_RTL_ENABLED
        &copter.mode_rtl, // Return to launch
#endif
#if MODE_DRIFT_ENABLED
        &copter.mode_drift, // Car-like flight control
#endif
#if MODE_SPORT_ENABLED
        &copter.mode_sport, // Rate-limited manual without position hold
#endif
#if MODE_FLIP_ENABLED
        &copter.mode_flip, // Automated flip maneuver
#endif
#if AUTOTUNE_ENABLED
        &copter.mode_autotune, // Automated PID tuning
#endif
#if MODE_POSHOLD_ENABLED
        &copter.mode_poshold, // Position hold with brake on stick release
#endif
#if MODE_BRAKE_ENABLED
        &copter.mode_brake, // Rapid deceleration
#endif
#if MODE_THROW_ENABLED
        &copter.mode_throw, // Throw-to-arm startup
#endif
#if AP_ADSB_AVOIDANCE_ENABLED
        &copter.mode_avoid_adsb, // ADSB collision avoidance
#endif
#if MODE_GUIDED_NOGPS_ENABLED
        &copter.mode_guided_nogps, // Guided mode without GPS (optical flow, etc.)
#endif
#if MODE_SMARTRTL_ENABLED
        &copter.mode_smartrtl, // RTL following recorded path
#endif
#if MODE_FLOWHOLD_ENABLED
        (Mode*)copter.g2.mode_flowhold_ptr, // Optical flow position hold (no GPS)
#endif
#if MODE_FOLLOW_ENABLED
        &copter.mode_follow, // Follow another vehicle
#endif
#if MODE_ZIGZAG_ENABLED
        &copter.mode_zigzag, // Automated zigzag pattern (agriculture)
#endif
#if MODE_SYSTEMID_ENABLED
        (Mode *)copter.g2.mode_systemid_ptr, // System identification tuning
#endif
#if MODE_AUTOROTATE_ENABLED
        &copter.mode_autorotate, // Helicopter emergency autorotation
#endif
#if MODE_TURTLE_ENABLED
        &copter.mode_turtle, // Flip-over recovery
#endif
    };

    // Count compiled-in base modes from static array
    // Array size determined at compile time based on feature flags
    const uint8_t base_mode_count = ARRAY_SIZE(modes);
    uint8_t mode_count = base_mode_count;

#if AP_SCRIPTING_ENABLED
    // Add dynamically-created scripting modes to total count
    // Scripting allows custom flight modes defined in Lua scripts
    // These modes extend guided mode with script-defined behavior
    for (uint8_t i = 0; i < ARRAY_SIZE(copter.mode_guided_custom); i++) {
        // Check if scripting mode slot is occupied
        // nullptr indicates empty slot
        if (copter.mode_guided_custom[i] != nullptr) {
            mode_count += 1; // Increment total available mode count
        }
    }
#endif

    // Convert 1-based MAVLink index to 0-based array index
    // MAVLink protocol uses 1-based indexing (index 1 = first mode)
    // C arrays use 0-based indexing (array[0] = first element)
    const uint8_t index_zero = index - 1;
    
    // Validate index is within valid range
    // If index beyond mode count, just return count without sending
    // This allows GCS to query total count with any large index
    if (index_zero >= mode_count) {
        // Requested index out of range
        // Don't send message, just return total mode count
        // GCS uses this to know when iteration complete
        return mode_count;
    }

    // Extract mode name and number for the requested index
    // Mode number is the actual flight mode identifier used in commands
    // Mode name is human-readable string displayed in GCS
    const char* name;
    uint8_t mode_number;

    // Determine if requested mode is base mode or scripting mode
    if (index_zero < base_mode_count) {
        // Base mode from static modes[] array
        // Get mode name (e.g., "LOITER", "AUTO", "STABILIZE")
        name = modes[index_zero]->name();
        
        // Get mode number from Mode::Number enum
        // This is the actual mode identifier used in MAVLink commands
        // Example: Mode::Number::LOITER = 5, Mode::Number::AUTO = 3
        mode_number = (uint8_t)modes[index_zero]->mode_number();

    } else {
        // Scripting mode (index beyond base modes)
#if AP_SCRIPTING_ENABLED
        // Calculate scripting mode array index
        // Subtract base_mode_count to get scripting array index
        const uint8_t custom_index = index_zero - base_mode_count;
        
        // Validate scripting mode pointer (safety check)
        // Should always be valid if index calculation correct
        if (copter.mode_guided_custom[custom_index] == nullptr) {
            // Invalid index, should not happen
            // This indicates logic error in mode counting
            return mode_count;
        }
        
        // Get scripting mode name and number
        // Scripting modes have custom names defined in Lua scripts
        // Mode numbers allocated dynamically at runtime
        name = copter.mode_guided_custom[custom_index]->name();
        mode_number = (uint8_t)copter.mode_guided_custom[custom_index]->mode_number();
#else
        // Scripting disabled but index suggests scripting mode
        // Should not reach here if mode counting logic correct
        return mode_count;
#endif
    }

#if MODE_AUTO_ENABLED
    // AUTO and AUTO_RTL special handling
    // These two modes share the same mode object (copter.mode_auto)
    // but represent different operational modes with distinct numbers
    // AUTO_RTL: Mission RTL - returns home using mission's return path
    // AUTO: Standard mission execution
    // Must override name and number for first two array entries
    if (index_zero == 0) {
        // First entry: AUTO_RTL variant
        // Uses mission-defined return path if available
        // Falls back to standard RTL if no mission return sequence
        mode_number = (uint8_t)Mode::Number::AUTO_RTL;
        name = "AUTO RTL";

    } else if (index_zero == 1) {
        // Second entry: Standard AUTO mode
        // Executes uploaded mission waypoints
        // Standard autonomous mission flight
        mode_number = (uint8_t)Mode::Number::AUTO;
        name = "AUTO";

    }
#endif

    // Send AVAILABLE_MODES message with mode information
    // GCS uses this to populate mode selection UI
    mavlink_msg_available_modes_send(
        chan,                                           // MAVLink channel to send on
        mode_count,                                     // Total number of available modes
        index,                                          // Current mode index (1-based)
        MAV_STANDARD_MODE::MAV_STANDARD_MODE_NON_STANDARD, // Not standard MAVLink mode
        mode_number,                                    // ArduPilot mode number (from Mode::Number enum)
        0,                                             // MAV_MODE_PROPERTY bitmask (reserved, unused)
        name                                           // Human-readable mode name string
    );

    // Return total mode count
    // GCS uses this to know when all modes enumerated
    // Same value returned regardless of index parameter
    return mode_count;
}
