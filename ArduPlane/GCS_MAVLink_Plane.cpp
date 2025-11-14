/**
 * @file GCS_MAVLink_Plane.cpp
 * @brief MAVLink communication handling for ArduPlane
 * 
 * @details This file implements the plane-specific MAVLink message handling,
 *          extending the base GCS_MAVLINK class with fixed-wing and quadplane
 *          specific telemetry, commands, and protocol handling.
 *          
 *          Key Responsibilities:
 *          - MAVLink command processing (MAV_CMD_* commands via command_int)
 *          - Telemetry streaming (attitude, navigation, position targets, PID tuning)
 *          - Mission protocol handling (waypoint management, mission commands)
 *          - Parameter protocol support (inherited from base class)
 *          - Position/attitude control message handling for GUIDED mode
 *          - Plane-specific status reporting (VTOL state, landed state, etc.)
 *          - Mode switching and flight control commands
 *          - Quadplane-specific command extensions (VTOL transitions, motor test)
 *          
 *          MAVLink Protocol Support:
 *          - MAVLink 2.0 protocol with command_int support
 *          - Position target commands (local NED and global int)
 *          - Attitude target commands for external control
 *          - Mission protocol with float and int support
 *          - Terrain protocol integration
 *          - High latency telemetry optimization
 *          
 *          Coordinate Systems:
 *          - Telemetry uses NED (North-East-Down) frame convention
 *          - Attitude in radians (roll, pitch, yaw)
 *          - Position in WGS84 geodetic coordinates (lat/lon in 1e7 degrees, alt in meters)
 *          - Local positions in NED frame relative to origin
 *          
 *          Safety Considerations:
 *          - External control commands only accepted in GUIDED mode
 *          - Fence checks applied to commanded positions
 *          - Mode change validation with proper ModeReason tracking
 *          - Failsafe protection: commands rejected during failsafe conditions
 *          
 * @note This implementation supports both traditional fixed-wing and quadplane configurations
 * @warning External attitude/position control can affect flight stability if misconfigured
 * 
 * @see GCS_MAVLink for base MAVLink handling
 * @see GCS_Plane for plane-specific GCS interface
 * @see Plane for main vehicle control logic
 * 
 * Source: ArduPlane/GCS_MAVLink_Plane.cpp
 */

#include "GCS_MAVLink_Plane.h"

#include "Plane.h"
#include <AP_RPM/AP_RPM_config.h>
#include <AP_Airspeed/AP_Airspeed_config.h>
#include <AP_EFI/AP_EFI_config.h>

/**
 * @brief Returns the MAVLink frame type for the vehicle
 * 
 * @details Reports the vehicle type to ground control stations for proper
 *          UI rendering and command interpretation. For quadplane vehicles,
 *          the type may dynamically reflect the current flight mode (fixed-wing
 *          vs multicopter behavior).
 * 
 * @return MAV_TYPE Vehicle type (MAV_TYPE_FIXED_WING or quadplane-specific type)
 * 
 * @note Quadplane vehicles may report different types based on current state
 */
MAV_TYPE GCS_Plane::frame_type() const
{
#if HAL_QUADPLANE_ENABLED
    return plane.quadplane.get_mav_type();
#else
    return MAV_TYPE_FIXED_WING;
#endif
}

/**
 * @brief Calculates MAVLink base_mode flags for current vehicle state
 * 
 * @details Converts ArduPlane flight modes and states into standard MAVLink
 *          base_mode bit flags for generic ground station compatibility.
 *          The mapping is approximate since ArduPlane modes don't align perfectly
 *          with MAVLink's generic mode definitions. Ground stations should primarily
 *          use custom_mode for accurate mode identification.
 *          
 *          Base Mode Flag Mapping:
 *          - MAV_MODE_FLAG_MANUAL_INPUT_ENABLED: Manual control modes (MANUAL, TRAINING, ACRO, QACRO)
 *          - MAV_MODE_FLAG_STABILIZE_ENABLED: Stabilized modes (STABILIZE, FBWA, FBWB, CRUISE, Q modes)
 *          - MAV_MODE_FLAG_GUIDED_ENABLED: Autonomous modes (AUTO, RTL, LOITER, GUIDED, etc.)
 *          - MAV_MODE_FLAG_SAFETY_ARMED: Vehicle is armed and ready to fly
 *          - MAV_MODE_FLAG_CUSTOM_MODE_ENABLED: Always set (custom_mode contains actual mode)
 *          
 *          Additional Logic:
 *          - Stick mixing adds MANUAL_INPUT_ENABLED flag to most modes
 *          - Training mode conditions affect STABILIZE flag
 *          - INITIALISING mode sets no operational flags
 * 
 * @return uint8_t Bitmask of MAV_MODE_FLAG_* values representing current state
 * 
 * @note This is a best-effort mapping; custom_mode provides definitive mode information
 * @note MAV_MODE_FLAG_AUTO_ENABLED is not used as it implies autonomous goal-finding
 * 
 * @see custom_mode() for precise ArduPlane mode number
 * @see plane.control_mode for current flight mode pointer
 */
uint8_t GCS_MAVLINK_Plane::base_mode() const
{
    uint8_t _base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    // work out the base_mode. This value is not very useful
    // for APM, but we calculate it as best we can so a generic
    // MAVLink enabled ground station can work out something about
    // what the MAV is up to. The actual bit values are highly
    // ambiguous for most of the APM flight modes. In practice, you
    // only get useful information from the custom_mode, which maps to
    // the APM flight mode and has a well defined meaning in the
    // ArduPlane documentation
    switch (plane.control_mode->mode_number()) {
    case Mode::Number::MANUAL:
    case Mode::Number::TRAINING:
    case Mode::Number::ACRO:
#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QACRO:
        _base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        break;
#endif
    case Mode::Number::STABILIZE:
    case Mode::Number::FLY_BY_WIRE_A:
    case Mode::Number::AUTOTUNE:
    case Mode::Number::FLY_BY_WIRE_B:
#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QSTABILIZE:
    case Mode::Number::QHOVER:
    case Mode::Number::QLOITER:
    case Mode::Number::QLAND:
#if QAUTOTUNE_ENABLED
    case Mode::Number::QAUTOTUNE:
#endif
#endif  // HAL_QUADPLANE_ENABLED
    case Mode::Number::CRUISE:
        _base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
        break;
    case Mode::Number::AUTO:
    case Mode::Number::RTL:
    case Mode::Number::LOITER:
    case Mode::Number::THERMAL:
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:
    case Mode::Number::CIRCLE:
    case Mode::Number::TAKEOFF:
#if MODE_AUTOLAND_ENABLED
    case Mode::Number::AUTOLAND:
#endif
#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QRTL:
    case Mode::Number::LOITER_ALT_QLAND:
#endif
        _base_mode = MAV_MODE_FLAG_GUIDED_ENABLED |
                     MAV_MODE_FLAG_STABILIZE_ENABLED;
        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
        break;
    case Mode::Number::INITIALISING:
        break;
    }

    if (!plane.training_manual_pitch || !plane.training_manual_roll) {
        _base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;        
    }

    if (plane.control_mode != &plane.mode_manual && plane.control_mode != &plane.mode_initializing) {
        // stabiliser of some form is enabled
        _base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
    }

    if (plane.g.stick_mixing != StickMixing::NONE && plane.control_mode != &plane.mode_initializing) {
        if ((plane.g.stick_mixing != StickMixing::VTOL_YAW) || (plane.control_mode == &plane.mode_auto)) {
            // all modes except INITIALISING have some form of manual
            // override if stick mixing is enabled
            _base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        }
    }

    // we are armed if we are not initialising
    if (plane.control_mode != &plane.mode_initializing && plane.arming.is_armed()) {
        _base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // indicate we have set a custom mode
    _base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    return _base_mode;
}

/**
 * @brief Returns the ArduPlane-specific custom mode number
 * 
 * @details Provides the precise flight mode as an integer that maps directly
 *          to Mode::Number enum. Ground stations use this value to determine
 *          the exact flight mode and display appropriate controls.
 * 
 * @return uint32_t Mode number from Mode::Number enum (MANUAL=0, CIRCLE=1, etc.)
 * 
 * @see Mode::Number enum for complete mode list
 * @see base_mode() for generic MAVLink mode flags
 */
uint32_t GCS_Plane::custom_mode() const
{
    return plane.control_mode->mode_number();
}

/**
 * @brief Reports current vehicle system status for MAVLink HEARTBEAT message
 * 
 * @details Determines the vehicle's operational state based on flight condition,
 *          failsafe status, and crash detection. This status is reported in the
 *          MAVLink HEARTBEAT message to inform ground stations of system health.
 *          
 *          Status Priority (highest to lowest):
 *          1. MAV_STATE_CALIBRATING: During initialization/mode_initializing
 *          2. MAV_STATE_CRITICAL: Any failsafe condition triggered
 *          3. MAV_STATE_EMERGENCY: Crash detected by crash_check algorithm
 *          4. MAV_STATE_ACTIVE: Vehicle is flying (airborne operation)
 *          5. MAV_STATE_STANDBY: On ground, armed or disarmed, ready to fly
 * 
 * @return MAV_STATE Current system status code
 * 
 * @note Status is evaluated in strict priority order
 * @warning MAV_STATE_CRITICAL indicates reduced capability or active failsafe
 * 
 * @see plane.any_failsafe_triggered() for failsafe detection
 * @see plane.crash_state for crash detection state
 * @see plane.is_flying() for flight detection algorithm
 */
MAV_STATE GCS_MAVLINK_Plane::vehicle_system_status() const
{
    if (plane.control_mode == &plane.mode_initializing) {
        return MAV_STATE_CALIBRATING;
    }
    if (plane.any_failsafe_triggered()) {
        return MAV_STATE_CRITICAL;
    }
    if (plane.crash_state.is_crashed) {
        return MAV_STATE_EMERGENCY;
    }
    if (plane.is_flying()) {
        return MAV_STATE_ACTIVE;
    }

    return MAV_STATE_STANDBY;
}


/**
 * @brief Sends MAVLink ATTITUDE message with current vehicle attitude and rates
 * 
 * @details Transmits vehicle attitude (roll, pitch, yaw) and angular rates to ground
 *          control station. For quadplanes in VTOL modes, uses the VTOL-specific
 *          AHRS view for more relevant attitude feedback. Pitch trim is automatically
 *          removed from reported pitch unless disabled by flight option.
 *          
 *          Attitude Reporting:
 *          - Roll, Pitch, Yaw: Euler angles in radians (NED frame convention)
 *          - Angular rates: Body-frame gyro measurements in rad/s
 *          - Pitch trim compensation applied by default for fixed-wing display
 *          
 *          Quadplane Behavior:
 *          - VTOL modes use quadplane AHRS view for multicopter-relevant attitude
 *          - Fixed-wing modes use standard AHRS attitude
 *          
 *          Message Format: ATTITUDE (#30)
 *          - time_boot_ms: System uptime in milliseconds
 *          - roll, pitch, yaw: Attitude in radians
 *          - rollspeed, pitchspeed, yawspeed: Angular velocity in rad/s
 * 
 * @note Pitch trim (g.pitch_trim) subtracted from reported pitch unless GCS_REMOVE_TRIM_PITCH enabled
 * @note Attitude is in NED (North-East-Down) frame: positive roll=right wing down, positive pitch=nose up
 * 
 * @see AP_AHRS::get_roll_rad() for roll angle source
 * @see plane.g.pitch_trim for configured pitch trim value
 * @see FlightOptions::GCS_REMOVE_TRIM_PITCH to disable trim removal
 */
void GCS_MAVLINK_Plane::send_attitude() const
{
    const AP_AHRS &ahrs = AP::ahrs();

    float r = ahrs.get_roll_rad();
    float p = ahrs.get_pitch_rad();
    float y = ahrs.get_yaw_rad();

    if (!(plane.flight_option_enabled(FlightOptions::GCS_REMOVE_TRIM_PITCH))) {
        p -= radians(plane.g.pitch_trim);
    }

#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.show_vtol_view()) {
        r = plane.quadplane.ahrs_view->roll;
        p = plane.quadplane.ahrs_view->pitch;
        y = plane.quadplane.ahrs_view->yaw;
    }
#endif

    const Vector3f &omega = ahrs.get_gyro();
    mavlink_msg_attitude_send(
        chan,
        millis(),
        r,
        p,
        y,
        omega.x,
        omega.y,
        omega.z);
}

/**
 * @brief Sends MAVLink ATTITUDE_TARGET message with commanded attitude setpoints
 * 
 * @details Reports the target attitude commanded by the attitude controller to ground
 *          stations, allowing real-time monitoring of control setpoints vs actual attitude.
 *          Only applicable and sent for quadplane vehicles during VTOL flight.
 *          
 *          Validity Check:
 *          - Target only sent if attitude control active within last 100ms
 *          - Prevents stale target data from being reported
 *          
 *          Target Content:
 *          - Attitude: Quaternion representing desired orientation [w,x,y,z] format
 *          - Body rates: Desired angular velocities in body frame (rad/s)
 *          - Throttle: Normalized collective thrust (0.0 to 1.0)
 *          
 *          Message Format: ATTITUDE_TARGET (#83)
 *          - type_mask: Set to 0 (all fields valid)
 *          - Quaternion in [w,x,y,z] order, unit-length
 *          - Rates in body frame (roll, pitch, yaw rates)
 * 
 * @note Quadplane-only feature, no output for traditional fixed-wing
 * @note Returns early without sending if attitude control not recently active
 * 
 * @see AC_AttitudeControl::get_attitude_target_quat() for target attitude
 * @see plane.quadplane.last_att_control_ms for last control update time
 */
void GCS_MAVLINK_Plane::send_attitude_target() 
{
#if HAL_QUADPLANE_ENABLED
    // Check if the attitude target is valid for reporting
    const uint32_t now = AP_HAL::millis();
    if (now  - plane.quadplane.last_att_control_ms > 100) {
        return;
    }

    const Quaternion quat  = plane.quadplane.attitude_control->get_attitude_target_quat();
    const Vector3f& ang_vel = plane.quadplane.attitude_control->get_attitude_target_ang_vel();
    const float throttle = plane.quadplane.attitude_control->get_throttle_in();

    const float quat_out[4] {quat.q1, quat.q2, quat.q3, quat.q4};

    const uint16_t typemask = 0; 

    mavlink_msg_attitude_target_send(
        chan,
        now,                    // time since boot (ms)
        typemask,               // Bitmask that tells the system what control dimensions should be ignored by the vehicle
        quat_out,               // Target attitude quaternion [w, x, y, z] order, zero-rotation is [1, 0, 0, 0], unit-length
        ang_vel.x,              // bodyframe target roll rate (rad/s)
        ang_vel.y,              // bodyframe target pitch rate (rad/s)
        ang_vel.z,              // bodyframe yaw rate (rad/s)
        throttle);              // Collective thrust, normalized to 0 .. 1

#endif // HAL_QUADPLANE_ENABLED 
}

/**
 * @brief Sends MAVLink AOA_SSA message with angle of attack and sideslip angle
 * 
 * @details Transmits aerodynamic angle measurements for flight analysis and tuning.
 *          These angles are calculated by the AHRS from airspeed, ground speed, and
 *          attitude, or directly measured if AOA/SSA sensors are installed.
 *          
 *          Angle Definitions:
 *          - AOA (Angle of Attack): Angle between body x-axis and relative wind
 *          - SSA (Sideslip Angle): Angle of relative wind in body y-z plane
 *          
 *          Use Cases:
 *          - Stall margin monitoring during flight
 *          - Aerodynamic tuning and analysis
 *          - Flight envelope protection development
 *          - Validation of airspeed and wind estimates
 *          
 *          Message Format: AOA_SSA (#11020)
 *          - time_usec: Timestamp in microseconds
 *          - AOA: Angle of attack in radians
 *          - SSA: Sideslip angle in radians
 * 
 * @note Values are estimates unless dedicated AOA/SSA sensors installed
 * @note Accuracy depends on airspeed sensor quality and wind estimation
 * 
 * @see AP_AHRS::getAOA() for angle of attack calculation
 * @see AP_AHRS::getSSA() for sideslip angle calculation
 */

void GCS_MAVLINK_Plane::send_aoa_ssa()
{
    AP_AHRS &ahrs = AP::ahrs();

    mavlink_msg_aoa_ssa_send(
        chan,
        micros(),
        ahrs.getAOA(),
        ahrs.getSSA());
}

/**
 * @brief Sends MAVLink NAV_CONTROLLER_OUTPUT message with navigation guidance state
 * 
 * @details Reports the current navigation controller targets and errors for ground station
 *          display and analysis. Content varies between fixed-wing navigation (L1 controller)
 *          and quadplane VTOL navigation (position controller).
 *          
 *          Fixed-Wing Navigation Output:
 *          - nav_roll/pitch: Commanded roll/pitch in degrees for path following
 *          - nav_bearing: Direction to next waypoint in degrees
 *          - target_bearing: Desired heading for path following in degrees
 *          - wp_dist: Distance to waypoint in meters (capped at UINT16_MAX)
 *          - alt_error: Altitude error in meters (positive = above target)
 *          - aspd_error: Airspeed error in m/s * 100 (incorrect units, see PR#7933)
 *          - xtrack_error: Cross-track error in meters from desired path
 *          
 *          Quadplane VTOL Navigation Output:
 *          - Uses attitude target angles in centidegrees
 *          - Position errors from position controller
 *          - Cross-track error from waypoint navigation
 *          - Altitude error from vertical position control
 *          
 *          Message Format: NAV_CONTROLLER_OUTPUT (#62)
 * 
 * @note Not sent in MANUAL mode (no navigation active)
 * @note Airspeed error units are non-standard (*100 instead of m/s) - historical bug
 * @note Quadplane automatically switches output based on current flight mode
 * 
 * @see AP_Navigation for fixed-wing L1 controller
 * @see AC_PosControl for quadplane position controller
 * @see AC_WPNav for quadplane waypoint navigation
 */
void GCS_MAVLINK_Plane::send_nav_controller_output() const
{
    if (plane.control_mode == &plane.mode_manual) {
        return;
    }
#if HAL_QUADPLANE_ENABLED
    const QuadPlane &quadplane = plane.quadplane;
    if (quadplane.show_vtol_view() && quadplane.using_wp_nav()) {
        const Vector3f &targets = quadplane.attitude_control->get_att_target_euler_cd();

        const Vector2f& curr_pos = quadplane.inertial_nav.get_position_xy_cm();
        const Vector2f& target_pos = quadplane.pos_control->get_pos_target_NEU_cm().xy().tofloat();
        const Vector2f error = (target_pos - curr_pos) * 0.01;

        mavlink_msg_nav_controller_output_send(
            chan,
            targets.x * 0.01,
            targets.y * 0.01,
            targets.z * 0.01,
            degrees(error.angle()),
            MIN(error.length(), UINT16_MAX),
            (plane.control_mode != &plane.mode_qstabilize) ? quadplane.pos_control->get_pos_error_U_cm() * 0.01 : 0,
            plane.airspeed_error * 100,  // incorrect units; see PR#7933
            quadplane.wp_nav->crosstrack_error());
        return;
    }
#endif
    {
        const AP_Navigation *nav_controller = plane.nav_controller;
        mavlink_msg_nav_controller_output_send(
            chan,
            plane.nav_roll_cd * 0.01,
            plane.nav_pitch_cd * 0.01,
            nav_controller->nav_bearing_cd() * 0.01,
            nav_controller->target_bearing_cd() * 0.01,
            MIN(plane.auto_state.wp_distance, UINT16_MAX),
            plane.calc_altitude_error_cm() * 0.01,
            plane.airspeed_error * 100,  // incorrect units; see PR#7933
            nav_controller->crosstrack_error());
    }
}

/**
 * @brief Sends MAVLink POSITION_TARGET_GLOBAL_INT message with navigation target location
 * 
 * @details Transmits the current target waypoint position to ground stations for display
 *          on map interfaces. Reports the next waypoint location in global coordinates
 *          (WGS84 geodetic) that the vehicle is navigating toward.
 *          
 *          Message Content:
 *          - Latitude/Longitude: Next waypoint position in 1e7 degrees (WGS84)
 *          - Altitude: Target altitude in meters (absolute frame)
 *          - type_mask: Velocity and acceleration fields ignored (not computed)
 *          - coordinate_frame: Always MAV_FRAME_GLOBAL (absolute altitude)
 *          
 *          Behavior:
 *          - Not sent in MANUAL mode (no target waypoint)
 *          - Reports next_WP_loc which is updated by navigation modes
 *          - Altitude converted from internal cm to meters for MAVLink
 *          - Zero location not sent (waypoint not yet set)
 *          
 *          Message Format: POSITION_TARGET_GLOBAL_INT (#87)
 * 
 * @note Only position fields valid; velocity/acceleration/yaw fields zeroed
 * @note Altitude always reported in absolute frame regardless of internal frame
 * 
 * @see plane.next_WP_loc for current navigation target
 * @see Location::get_alt_cm() for altitude frame conversion
 */
void GCS_MAVLINK_Plane::send_position_target_global_int()
{
    if (plane.control_mode == &plane.mode_manual) {
        return;
    }
    Location &next_WP_loc = plane.next_WP_loc;
    static constexpr uint16_t POSITION_TARGET_TYPEMASK_LAST_BYTE = 0xF000;
    static constexpr uint16_t TYPE_MASK = POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                                          POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                                          POSITION_TARGET_TYPEMASK_YAW_IGNORE | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE | POSITION_TARGET_TYPEMASK_LAST_BYTE;
    int32_t alt = 0;
    if (!next_WP_loc.is_zero()) {
        UNUSED_RESULT(next_WP_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, alt));
    }

    mavlink_msg_position_target_global_int_send(
        chan,
        AP_HAL::millis(), // time_boot_ms
        MAV_FRAME_GLOBAL, // targets are always global altitude
        TYPE_MASK, // ignore everything except the x/y/z components
        next_WP_loc.lat, // latitude as 1e7
        next_WP_loc.lng, // longitude as 1e7
        alt * 0.01, // altitude is sent as a float
        0.0f, // vx
        0.0f, // vy
        0.0f, // vz
        0.0f, // afx
        0.0f, // afy
        0.0f, // afz
        0.0f, // yaw
        0.0f); // yaw_rate
}


/**
 * @brief Returns airspeed value for VFR_HUD MAVLink message display
 * 
 * @details Provides the best available airspeed measurement for ground station HUD display,
 *          with prioritized fallback: actual sensor → AHRS estimate → zero.
 *          Prefers true airspeed sensor readings over constrained estimates.
 *          
 *          Airspeed Source Priority:
 *          1. Direct airspeed sensor reading (if enabled and healthy)
 *          2. AHRS airspeed estimate (fused from ground speed and wind)
 *          3. Zero (no valid airspeed available)
 *          
 *          Rationale:
 *          - Direct sensor provides unconstrained true airspeed
 *          - AHRS estimate is constrained by ground speed and may lag
 *          - Reporting zero prevents displaying misleading values
 * 
 * @return float Current airspeed in m/s, or 0 if unavailable
 * 
 * @note Returns actual sensor reading when available, not constrained estimate
 * @note Zero return indicates airspeed not available, not zero airspeed
 * 
 * @see AP_Airspeed::get_airspeed() for sensor measurement
 * @see AP_AHRS::airspeed_estimate() for backup estimate
 */
float GCS_MAVLINK_Plane::vfr_hud_airspeed() const
{
    // airspeed sensors are best.  While the AHRS airspeed_estimate
    // will use an airspeed sensor, that value is constrained by the
    // ground speed.  When reporting we should send the true airspeed
    // value if possible:
#if AP_AIRSPEED_ENABLED
    if (plane.airspeed.enabled() && plane.airspeed.healthy()) {
        return plane.airspeed.get_airspeed();
    }
#endif

    // airspeed estimates are OK:
    float aspeed;
    if (AP::ahrs().airspeed_estimate(aspeed)) {
        return aspeed;
    }

    // lying is worst:
    return 0;
}

/**
 * @brief Returns throttle percentage for VFR_HUD MAVLink message display
 * 
 * @details Provides current throttle output as percentage for ground station HUD.
 *          Value represents actual throttle being applied, accounting for throttle
 *          limiting, failsafe conditions, and control mode.
 * 
 * @return int16_t Throttle percentage (0-100), or negative values for special states
 * 
 * @see Plane::throttle_percentage() for calculation including all limiting factors
 */
int16_t GCS_MAVLINK_Plane::vfr_hud_throttle() const
{
    return plane.throttle_percentage();
}

/**
 * @brief Returns climb rate for VFR_HUD MAVLink message display
 * 
 * @details Provides vertical velocity for ground station HUD, with special handling
 *          for soaring mode. When soaring is active, reports vario (air mass vertical
 *          velocity) instead of ground-relative climb rate to display thermal strength.
 *          
 *          Climb Rate Source:
 *          - Soaring active: Vario reading (air mass vertical velocity in m/s)
 *          - Normal flight: Ground-relative climb rate from base class
 *          
 *          Vario vs Climb Rate:
 *          - Vario: Vertical velocity of air mass (detects thermals)
 *          - Climb rate: Vehicle vertical velocity relative to ground
 * 
 * @return float Climb rate in m/s (positive = ascending)
 * 
 * @note Returns vario reading when soaring to help pilot find thermals
 * 
 * @see GCS_MAVLINK::vfr_hud_climbrate() for standard climb rate
 * @see SoaringController::get_vario_reading() for thermal air mass velocity
 */
float GCS_MAVLINK_Plane::vfr_hud_climbrate() const
{
#if HAL_SOARING_ENABLED
    if (plane.g2.soaring_controller.is_active()) {
        return plane.g2.soaring_controller.get_vario_reading();
    }
#endif
    return GCS_MAVLINK::vfr_hud_climbrate();
}

/**
 * @brief Sends MAVLink WIND message with wind velocity estimate
 * 
 * @details Transmits the AHRS wind estimate to ground stations for display
 *          and analysis. Wind is reported in meteorological convention
 *          (direction wind is coming FROM, not going TO).
 *          
 *          Message Content:
 *          - Wind direction: Degrees (0-360), direction wind is coming FROM
 *          - Wind speed: Horizontal wind magnitude in m/s
 *          - Wind vertical: Vertical wind component in m/s (positive = up)
 *          
 *          Coordinate Convention:
 *          - Uses negative of NED wind vector to convert to meteorological convention
 *          - North = 0°, East = 90°, South = 180°, West = 270°
 *          - Vertical: Positive = updraft, Negative = downdraft
 *          
 *          Wind Estimation:
 *          - Derived from AHRS sensor fusion (GPS ground speed vs airspeed)
 *          - Estimates may be poor when airspeed sensor unavailable
 * 
 * @note Wind direction uses meteorological convention (direction FROM)
 * @note Requires valid airspeed and ground speed for accurate estimation
 * 
 * @see AP_AHRS::wind_estimate() for wind vector computation
 */
void GCS_MAVLINK_Plane::send_wind() const
{
    const Vector3f wind = AP::ahrs().wind_estimate();
    mavlink_msg_wind_send(
        chan,
        degrees(atan2f(-wind.y, -wind.x)), // use negative, to give
                                          // direction wind is coming from
        wind.length(),
        wind.z);
}

/**
 * @brief Sends a single PID controller status via MAVLink PID_TUNING message
 * 
 * @details Transmits PID controller internal state for real-time tuning and analysis.
 *          Provides target, achieved value, and individual term contributions
 *          (feedforward, proportional, integral, derivative) for visualization
 *          in ground station tuning tools.
 *          
 *          Message Content:
 *          - Target: Desired setpoint for the controlled axis
 *          - Achieved: Actual measured value
 *          - FF: Feedforward term contribution
 *          - P: Proportional term contribution
 *          - I: Integral term contribution
 *          - D: Derivative term contribution
 *          - Slew rate: Rate limiting applied
 *          - Dmod: Derivative modification factor
 *          
 *          Usage:
 *          - Real-time PID tuning via ground station graphs
 *          - Controller performance analysis
 *          - Identifying oscillations or sluggish response
 * 
 * @param[in] pid_info Pointer to PID controller info structure (from PID::get_pid_info())
 * @param[in] axis PID axis identifier (PID_TUNING_ROLL, PID_TUNING_PITCH, etc.)
 * @param[in] achieved Current achieved value for this control axis
 * 
 * @note Returns early if pid_info is null or no payload space available
 * @note Message only sent if channel has space for PID_TUNING message
 * 
 * @see send_pid_tuning() for complete multi-axis PID telemetry
 * @see AP_PIDInfo for PID controller state structure
 */
void GCS_MAVLINK_Plane::send_pid_info(const AP_PIDInfo *pid_info,
                          const uint8_t axis, const float achieved)
{
    if (pid_info == nullptr) {
        return;
    }
    if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
        return;
    }
     mavlink_msg_pid_tuning_send(chan, axis,
                                 pid_info->target,
                                 achieved,
                                 pid_info->FF,
                                 pid_info->P,
                                 pid_info->I,
                                 pid_info->D,
                                 pid_info->slew_rate,
                                 pid_info->Dmod);
}

/**
 * @brief Sends PID tuning telemetry for enabled control axes
 * 
 * @details Streams PID controller status for all axes enabled in the GCS_PID_MASK
 *          parameter. Handles both fixed-wing and quadplane controllers, automatically
 *          switching to quadplane PIDs when in VTOL mode. Allows real-time tuning
 *          visualization and performance analysis via ground station.
 *          
 *          Supported Control Axes:
 *          - TUNING_BITS_ROLL: Roll rate controller (fixed-wing or quadplane)
 *          - TUNING_BITS_PITCH: Pitch rate controller (fixed-wing or quadplane)
 *          - TUNING_BITS_YAW: Yaw rate controller (fixed-wing or quadplane)
 *          - TUNING_BITS_STEER: Ground steering controller
 *          - TUNING_BITS_LAND: Landing flare controller (during landing only)
 *          - TUNING_BITS_ACCZ: Vertical acceleration controller (quadplane VTOL only)
 *          
 *          Controller Selection:
 *          - Fixed-wing mode: Uses rollController, pitchController, yawController
 *          - Quadplane VTOL mode: Uses quadplane.attitude_control rate PIDs
 *          - Landing phase: Sends landing flare controller with gyro feedback
 *          
 *          Behavior:
 *          - Not sent in MANUAL mode (no active PID control)
 *          - Controlled by GCS_PID_MASK parameter (bitmask of axes to send)
 *          - Each enabled axis sends separate PID_TUNING message
 *          - Quadplane automatically switches to multicopter controllers in VTOL mode
 * 
 * @note PID messages only sent when respective bit set in g.gcs_pid_mask
 * @note ACCZ tuning only available in quadplane VTOL modes
 * @note Landing PID only sent when in LAND flight stage
 * 
 * @see send_pid_info() for individual axis message transmission
 * @see Parameters::gcs_pid_mask for axis enable configuration
 */
void GCS_MAVLINK_Plane::send_pid_tuning()
{
    if (plane.control_mode == &plane.mode_manual) {
        // no PIDs should be used in manual
        return;
    }

    const Parameters &g = plane.g;

    const AP_PIDInfo *pid_info;
    if (g.gcs_pid_mask & TUNING_BITS_ROLL) {
        pid_info = &plane.rollController.get_pid_info();
#if HAL_QUADPLANE_ENABLED
        if (plane.quadplane.in_vtol_mode()) {
            pid_info = &plane.quadplane.attitude_control->get_rate_roll_pid().get_pid_info();
        }
#endif
        send_pid_info(pid_info, PID_TUNING_ROLL, pid_info->actual);
    }
    if (g.gcs_pid_mask & TUNING_BITS_PITCH) {
        pid_info = &plane.pitchController.get_pid_info();
#if HAL_QUADPLANE_ENABLED
        if (plane.quadplane.in_vtol_mode()) {
            pid_info = &plane.quadplane.attitude_control->get_rate_pitch_pid().get_pid_info();
        }
#endif
        send_pid_info(pid_info, PID_TUNING_PITCH, pid_info->actual);
    }
    if (g.gcs_pid_mask & TUNING_BITS_YAW) {
        pid_info = &plane.yawController.get_pid_info();
#if HAL_QUADPLANE_ENABLED
        if (plane.quadplane.in_vtol_mode()) {
            pid_info = &plane.quadplane.attitude_control->get_rate_yaw_pid().get_pid_info();
        }
#endif
        send_pid_info(pid_info, PID_TUNING_YAW, pid_info->actual);
    }
    if (g.gcs_pid_mask & TUNING_BITS_STEER) {
        pid_info = &plane.steerController.get_pid_info();
        send_pid_info(pid_info, PID_TUNING_STEER, pid_info->actual);
    }
    if ((g.gcs_pid_mask & TUNING_BITS_LAND) && (plane.flight_stage == AP_FixedWing::FlightStage::LAND)) {
        AP_AHRS &ahrs = AP::ahrs();
        const Vector3f &gyro = ahrs.get_gyro();
        send_pid_info(plane.landing.get_pid_info(), PID_TUNING_LANDING, degrees(gyro.z));
    }
#if HAL_QUADPLANE_ENABLED
    if (g.gcs_pid_mask & TUNING_BITS_ACCZ && plane.quadplane.in_vtol_mode()) {
        pid_info = &plane.quadplane.pos_control->get_accel_U_pid().get_pid_info();
        send_pid_info(pid_info, PID_TUNING_ACCZ, pid_info->actual);
    }
#endif
}

/**
 * @brief Attempts to send a plane-specific MAVLink message
 * 
 * @details Handles transmission of ArduPlane-specific messages within the streaming
 *          telemetry system. Checks payload buffer space before sending to prevent
 *          serial buffer overflow. Falls back to base class for common messages.
 *          
 *          Plane-Specific Messages Handled:
 *          - MSG_TERRAIN_REQUEST: Request terrain height data from GCS
 *          - MSG_TERRAIN_REPORT: Report terrain following status
 *          - MSG_WIND: Wind velocity estimate
 *          - MSG_ADSB_VEHICLE: ADS-B traffic aircraft positions
 *          - MSG_AOA_SSA: Angle of Attack and Sideslip Angle
 *          - MSG_LANDING: Landing controller status (deepstall)
 *          - MSG_HYGROMETER: Humidity sensor data from airspeed sensors
 *          
 *          Flow Control:
 *          - CHECK_PAYLOAD_SIZE() verifies buffer space before transmission
 *          - Returns false if message won't fit (retry later)
 *          - Returns true if message sent or not applicable
 *          
 *          Message Streaming:
 *          - Called by telemetry scheduler based on stream rates
 *          - Conditional compilation (#if) excludes unavailable features
 *          - Base class handles all common vehicle-agnostic messages
 * 
 * @param[in] id Message identifier from ap_message enum
 * 
 * @return bool true if message sent or not applicable, false if buffer full
 * 
 * @note Returns true even if message not sent due to feature disabled
 * @note Checks payload space to prevent serial buffer overflow
 * 
 * @see GCS_MAVLINK::try_send_message() for common message handling
 * @see ap_message enum for complete message list
 */
bool GCS_MAVLINK_Plane::try_send_message(enum ap_message id)
{
    switch (id) {

#if AP_TERRAIN_AVAILABLE
    case MSG_TERRAIN_REQUEST:
        CHECK_PAYLOAD_SIZE(TERRAIN_REQUEST);
        plane.terrain.send_request(chan);
        break;
    case MSG_TERRAIN_REPORT:
        CHECK_PAYLOAD_SIZE(TERRAIN_REPORT);
        plane.terrain.send_report(chan);
        break;
#endif

    case MSG_WIND:
        CHECK_PAYLOAD_SIZE(WIND);
        send_wind();
        break;

    case MSG_ADSB_VEHICLE:
#if HAL_ADSB_ENABLED
        CHECK_PAYLOAD_SIZE(ADSB_VEHICLE);
        plane.adsb.send_adsb_vehicle(chan);
#endif
        break;

    case MSG_AOA_SSA:
        CHECK_PAYLOAD_SIZE(AOA_SSA);
        send_aoa_ssa();
        break;

#if HAL_LANDING_DEEPSTALL_ENABLED
    case MSG_LANDING:
        plane.landing.send_landing_message(chan);
        break;
#endif  // HAL_LANDING_DEEPSTALL_ENABLED

    case MSG_HYGROMETER:
#if AP_AIRSPEED_HYGROMETER_ENABLE
        CHECK_PAYLOAD_SIZE(HYGROMETER_SENSOR);
        send_hygrometer();
#endif
        break;

    default:
        return GCS_MAVLINK::try_send_message(id);
    }
    return true;
}

#if AP_AIRSPEED_HYGROMETER_ENABLE
/**
 * @brief Sends humidity sensor data from airspeed sensors with hygrometer capability
 * 
 * @details Transmits temperature and humidity measurements from airspeed sensors
 *          equipped with hygrometer (humidity) capability. Iterates through all
 *          airspeed sensors, sending data from those with active hygrometer readings.
 *          Uses round-robin transmission to distribute multiple sensors over time.
 *          
 *          Message Content:
 *          - Sensor ID: Index of airspeed sensor (0-AIRSPEED_MAX_SENSORS)
 *          - Temperature: Degrees Celsius * 100 (int16_t)
 *          - Humidity: Relative humidity percentage * 100 (uint16_t)
 *          
 *          Transmission Logic:
 *          - Round-robin through sensors (last_hygrometer_send_idx tracks position)
 *          - Only sends sensors with recent data (within 2000ms)
 *          - Skips sensors without hygrometer or with stale data
 *          - Checks payload space before each message
 *          
 *          Data Freshness:
 *          - Requires hygrometer data updated within last 2 seconds
 *          - Stale sensors skipped to avoid outdated readings
 *          
 *          Use Cases:
 *          - Icing condition detection (temperature + humidity)
 *          - Environmental monitoring
 *          - Flight data logging
 * 
 * @note Only airspeed sensors with hygrometer capability provide data
 * @note Temperature in centidegrees Celsius, humidity in centi-percent
 * @note Multiple sensors sent one per call (round-robin scheduling)
 * 
 * @see AP_Airspeed::get_hygrometer() for sensor data retrieval
 */
void GCS_MAVLINK_Plane::send_hygrometer()
{
    if (!HAVE_PAYLOAD_SPACE(chan, HYGROMETER_SENSOR)) {
        return;
    }

    const auto *airspeed = AP::airspeed();
    if (airspeed == nullptr) {
        return;
    } 
    const uint32_t now = AP_HAL::millis();

    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        uint8_t idx = (i+last_hygrometer_send_idx+1) % AIRSPEED_MAX_SENSORS;
        float temperature, humidity;
        uint32_t last_sample_ms;
        if (!airspeed->get_hygrometer(idx, last_sample_ms, temperature, humidity)) {
            continue;
        }
        if (now - last_sample_ms > 2000) {
            // not updating, stop sending
            continue;
        }
        if (!HAVE_PAYLOAD_SPACE(chan, HYGROMETER_SENSOR)) {
            return;
        }

        mavlink_msg_hygrometer_sensor_send(
            chan,
            idx,
            int16_t(temperature*100),
            uint16_t(humidity*100));
        last_hygrometer_send_idx = idx;
    }
}
#endif // AP_AIRSPEED_HYGROMETER_ENABLE

/**
 * @brief Handles request to switch to GUIDED mode with target location
 * 
 * @details Callback invoked by mission protocol when receiving mission commands
 *          that require switching to GUIDED mode. Delegates to the current flight
 *          mode's guided request handler for mode-specific validation and setup.
 *          
 *          Usage Context:
 *          - Called during mission upload/execution
 *          - Mission item processing via handle_mission_item()
 *          - Mode-specific validation ensures safe transition
 *          
 *          Mode Behavior:
 *          - Some modes accept guided requests (AUTO, GUIDED)
 *          - Others may reject to maintain safety (RTL, LAND)
 *          - Each mode validates location feasibility
 * 
 * @param[in] cmd Mission command containing target location in content.location
 * 
 * @return bool true if mode accepts guided request, false if rejected
 * 
 * @note Actual mode change handled by control_mode's handler
 * @note Called as callback from mission protocol processing
 * 
 * @see Mode::handle_guided_request() for mode-specific handling
 * @see GCS_MAVLINK::handle_mission_item() for mission protocol
 */
bool GCS_MAVLINK_Plane::handle_guided_request(AP_Mission::Mission_Command &cmd)
{
    return plane.control_mode->handle_guided_request(cmd.content.location);
}

/**
 * @brief Handles request to change current waypoint altitude
 * 
 * @details Updates the target waypoint altitude when commanded via MAVLink
 *          (typically DO_CHANGE_ALTITUDE mission command or direct command_int).
 *          Properly handles terrain-relative and absolute altitude frames,
 *          ensuring terrain database is consulted for terrain altitudes.
 *          
 *          Altitude Frame Handling:
 *          - Terrain-relative: Sets next_WP_loc.alt in ABOVE_TERRAIN frame
 *          - Absolute: Converts to ABSOLUTE frame using location.get_alt_m()
 *          - Preserves original frame intent from command
 *          
 *          Terrain Processing:
 *          - Calls fix_terrain_WP() to load terrain height if needed
 *          - Ensures terrain database populated for terrain-relative waypoints
 *          - Handles terrain unavailability gracefully
 *          
 *          Side Effects:
 *          - Updates plane.next_WP_loc altitude
 *          - Resets offset altitude (pitch compensation altitude offset)
 *          - Navigation immediately targets new altitude
 * 
 * @param[in,out] location Commanded altitude location (frame and altitude value)
 *                         Modified by fix_terrain_WP() if terrain loading needed
 * 
 * @note Changes navigation target immediately (no trajectory planning)
 * @note Called as callback from mission protocol and command handlers
 * @note Resets offset altitude to prevent pitch compensation conflicts
 * 
 * @see Plane::fix_terrain_WP() for terrain height database loading
 * @see Location::set_alt_cm() for altitude frame setting
 * @see Plane::reset_offset_altitude() for pitch compensation reset
 */
void GCS_MAVLINK_Plane::handle_change_alt_request(Location &location)
{
    plane.fix_terrain_WP(location, __LINE__);

    if (location.terrain_alt) {
        plane.next_WP_loc.set_alt_cm(location.alt, Location::AltFrame::ABOVE_TERRAIN);
    } else {
        // convert to absolute alt
        float abs_alt_m;
        if (location.get_alt_m(Location::AltFrame::ABSOLUTE, abs_alt_m)) {
            plane.next_WP_loc.set_alt_m(abs_alt_m, Location::AltFrame::ABSOLUTE);
        }
    }
    plane.reset_offset_altitude();
}


/**
 * @brief Handles LANDING_TARGET MAVLink message for precision landing
 * 
 * @details Processes LANDING_TARGET messages from external vision systems or IR-LOCK
 *          sensors to enable precision landing on visual targets. Timestamp is
 *          pre-corrected for system clock jitter to ensure accurate target tracking.
 *          
 *          Message Usage:
 *          - Vision-based landing target detection (AprilTags, visual markers)
 *          - IR-LOCK beacon tracking for precision landing
 *          - External camera system target reports
 *          
 *          Timestamp Correction:
 *          - timestamp_ms already adjusted for clock jitter by base class
 *          - Allows accurate velocity estimation of moving targets
 *          - Compensates for MAVLink transmission delays
 *          
 *          Precision Landing Integration:
 *          - Delegates to precision landing controller (AC_PrecLand)
 *          - Used during LAND mode for final approach guidance
 *          - Target position fused with vehicle navigation
 * 
 * @param[in] packet LANDING_TARGET MAVLink message with target position/velocity
 * @param[in] timestamp_ms Jitter-corrected timestamp in milliseconds
 * 
 * @note Only processed when precision landing compiled in (AC_PRECLAND_ENABLED)
 * @note Timestamp pre-corrected by base class for accurate tracking
 * 
 * @see AC_PrecLand::handle_msg() for precision landing processing
 */
void GCS_MAVLINK_Plane::handle_landing_target(const mavlink_landing_target_t &packet, uint32_t timestamp_ms)
{
#if AC_PRECLAND_ENABLED
    plane.g2.precland.handle_msg(packet, timestamp_ms);
#endif
}

/**
 * @brief Handles preflight calibration commands (sensors, RC, level, etc.)
 * 
 * @details Processes MAV_CMD_PREFLIGHT_CALIBRATION commands to calibrate sensors
 *          before flight. Sets calibration flag to prevent normal flight operations
 *          during calibration process. Supports gyro, accelerometer, magnetometer,
 *          barometer, RC, and level calibration procedures.
 *          
 *          Calibration Types:
 *          - param1: Gyro calibration
 *          - param2: Magnetometer calibration
 *          - param3: Ground pressure (barometer) calibration
 *          - param4: RC calibration
 *          - param5: Accelerometer calibration
 *          - param6: Compass motor compensation
 *          - param7: Airspeed calibration
 *          
 *          Safety:
 *          - Sets plane.in_calibration flag during calibration
 *          - Prevents flight control during calibration
 *          - Ensures sensors stabilize before clearing flag
 *          
 *          Typical Sequence:
 *          1. Set in_calibration = true
 *          2. Base class performs calibration
 *          3. Clear in_calibration = false
 *          4. Return result to GCS
 * 
 * @param[in] packet COMMAND_INT message with calibration parameters
 * @param[in] msg Complete MAVLink message for context
 * 
 * @return MAV_RESULT Result code (ACCEPTED, DENIED, FAILED, etc.)
 * 
 * @warning Vehicle must be stationary during calibration
 * @note Calibration flag prevents arming and normal operation
 * 
 * @see GCS_MAVLINK::handle_command_preflight_calibration() for base implementation
 */
MAV_RESULT GCS_MAVLINK_Plane::handle_command_preflight_calibration(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    plane.in_calibration = true;
    MAV_RESULT ret = GCS_MAVLINK::handle_command_preflight_calibration(packet, msg);
    plane.in_calibration = false;

    return ret;
}

/**
 * @brief Callback invoked when MAVLink packet received on this channel
 * 
 * @details Called by MAVLink parser after successful packet reception and CRC
 *          validation. Allows plane-specific pre-processing before standard
 *          message handling. Currently used for ADS-B avoidance integration.
 *          
 *          Pre-Processing:
 *          - ADS-B avoidance system receives copy of all packets
 *          - Enables avoidance system to track ADSB_VEHICLE messages
 *          - Base class handles standard protocol processing
 *          
 *          Processing Order:
 *          1. Plane-specific pre-processing (ADS-B)
 *          2. Base class message routing (GCS_MAVLINK::packetReceived)
 *          3. Message-specific handlers (handle_message)
 * 
 * @param[in] status MAVLink channel status (sequence, packet drops, etc.)
 * @param[in] msg Received MAVLink message after CRC validation
 * 
 * @note Called for every successfully received packet
 * @note ADS-B avoidance examines all packets for relevant messages
 * 
 * @see GCS_MAVLINK::packetReceived() for base message routing
 * @see AP_Avoidance_ADSB::handle_msg() for ADS-B packet processing
 */
void GCS_MAVLINK_Plane::packetReceived(const mavlink_status_t &status,
                                       const mavlink_message_t &msg)
{
#if AP_ADSB_AVOIDANCE_ENABLED
    plane.avoidance_adsb.handle_msg(msg);
#endif
    GCS_MAVLINK::packetReceived(status, msg);
}


/**
 * @brief Sets home position to current GPS location
 * 
 * @details Updates the home position to the vehicle's current GPS coordinates
 *          and persists to storage. If already in RTL/QRTL mode, re-enters the
 *          mode to navigate to the new home location immediately.
 *          
 *          Home Position Usage:
 *          - Return point for RTL (Return to Launch) mode
 *          - Origin for mission coordinate calculations
 *          - Reference for geofence boundaries
 *          - Baseline for altitude calculations
 *          
 *          Locking:
 *          - If _lock=true, prevents further home updates
 *          - Protects against unwanted GPS drift updates
 *          - Typically locked after initial GPS fix
 *          
 *          RTL Behavior:
 *          - If currently in RTL or QRTL mode, re-enters mode
 *          - Causes immediate navigation to new home location
 *          - Ensures RTL flies to updated home, not old position
 * 
 * @param[in] _lock If true, locks home position preventing further updates
 * 
 * @return bool true if home set successfully, false if GPS invalid or set failed
 * 
 * @note Requires valid GPS position fix
 * @note Persists home position to storage (survives reboot)
 * @warning Updates RTL target immediately if in RTL mode
 * 
 * @see AP::gps().location() for current GPS position
 * @see set_home_persistently() for storage operation
 */
bool Plane::set_home_to_current_location(bool _lock)
{
    if (!set_home_persistently(AP::gps().location())) {
        return false;
    }
    if (_lock) {
        AP::ahrs().lock_home();
    }
    if ((control_mode == &mode_rtl)
#if HAL_QUADPLANE_ENABLED
            || (control_mode == &mode_qrtl)
#endif
                                                        ) {
        // if in RTL head to the updated home location
        control_mode->enter();
    }
    return true;
}

/**
 * @brief Sets home position to specified location
 * 
 * @details Updates home position to the provided location. If already in RTL/QRTL
 *          mode, re-enters the mode to navigate to the new home location immediately.
 *          Allows GCS or companion computer to set home to arbitrary position.
 *          
 *          Use Cases:
 *          - GCS commands home to specific coordinates
 *          - Mission planning sets home before takeoff
 *          - Recovery operations set home to safe location
 *          - Formation flight shares common home position
 *          
 *          Locking:
 *          - If _lock=true, prevents further home updates
 *          - Protects against unwanted GPS drift updates
 *          - Typically locked after initial GPS fix
 *          
 *          RTL Behavior:
 *          - If currently in RTL or QRTL mode, re-enters mode
 *          - Causes immediate navigation to new home location
 *          - Ensures RTL flies to updated home, not old position
 * 
 * @param[in] loc Location to set as new home position
 * @param[in] _lock If true, locks home position preventing further updates
 * 
 * @return bool true if home set successfully, false if location invalid
 * 
 * @note Does not persist to storage (lost on reboot)
 * @warning Updates RTL target immediately if in RTL mode
 * 
 * @see AP::ahrs().set_home() for AHRS home setting
 * @see AP::ahrs().lock_home() for locking mechanism
 */
bool Plane::set_home(const Location& loc, bool _lock)
{
    if (!AP::ahrs().set_home(loc)) {
        return false;
    }
    if (_lock) {
        AP::ahrs().lock_home();
    }
    if ((control_mode == &mode_rtl)
#if HAL_QUADPLANE_ENABLED
            || (control_mode == &mode_qrtl)
#endif
                                                        ) {
        // if in RTL head to the updated home location
        control_mode->enter();
    }
    return true;
}

/**
 * @brief Handles MAV_CMD_DO_REPOSITION command to fly to new location
 * 
 * @details Processes DO_REPOSITION command to navigate to a new position. Validates
 *          location sanity, checks geofence boundaries, and switches to GUIDED mode
 *          if requested or already in GUIDED. Supports loiter radius and direction.
 *          
 *          Command Parameters:
 *          - param1: Ground speed in m/s (not currently used)
 *          - param2: Flags (MAV_DO_REPOSITION_FLAGS_CHANGE_MODE to force GUIDED)
 *          - param3: Loiter radius in meters (positive value required)
 *          - param4: Yaw/direction (NaN or 0=clockwise, positive=counter-clockwise)
 *          - x (int32): Latitude in 1E7 degrees
 *          - y (int32): Longitude in 1E7 degrees
 *          - z (float): Altitude (frame specified by packet.frame)
 *          
 *          Validation Steps:
 *          1. Sanity check lat/lon values
 *          2. Parse location from command_int
 *          3. Load terrain height if terrain-relative
 *          4. Parse loiter direction from param4
 *          5. Sanitize location (check for NaN, invalid values)
 *          6. Check geofence boundaries
 *          
 *          Mode Behavior:
 *          - If CHANGE_MODE flag set or already in GUIDED: switches to GUIDED
 *          - Otherwise: command fails
 *          - Clears any guided heading commands
 *          
 *          Altitude Handling:
 *          - Terrain-relative: Preserved as ABOVE_TERRAIN
 *          - Non-terrain: Converted to ABSOLUTE frame
 *          
 *          Loiter Configuration:
 *          - param3 > 0: Sets loiter radius in meters
 *          - param4: Sets loiter direction (CW/CCW)
 * 
 * @param[in] packet COMMAND_INT message with reposition parameters
 * 
 * @return MAV_RESULT ACCEPTED if repositioned, DENIED if invalid, FAILED if mode issue
 * 
 * @note Requires valid GPS fix for position validation
 * @warning Rejected if destination outside geofence
 * @warning Only works in GUIDED mode or with CHANGE_MODE flag
 * 
 * @see Plane::set_guided_WP() for waypoint navigation setup
 * @see ModeGuided::set_radius_and_direction() for loiter configuration
 */
MAV_RESULT GCS_MAVLINK_Plane::handle_command_int_do_reposition(const mavlink_command_int_t &packet)
{
    // sanity check location
    if (!check_latlng(packet.x, packet.y)) {
        return MAV_RESULT_DENIED;
    }

    Location requested_position;
    if (!location_from_command_t(packet, requested_position)) {
        return MAV_RESULT_DENIED;
    }
    plane.fix_terrain_WP(requested_position, __LINE__);

    if (isnan(packet.param4) || is_zero(packet.param4)) {
        requested_position.loiter_ccw = 0;
    } else {
        requested_position.loiter_ccw = 1;
    }

    if (requested_position.sanitize(plane.current_loc)) {
        // if the location wasn't already sane don't load it
        return MAV_RESULT_DENIED;
    }

#if AP_FENCE_ENABLED
    // reject destination if outside the fence
    if (!plane.fence.check_destination_within_fence(requested_position)) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        return MAV_RESULT_DENIED;
    }
#endif

    // location is valid load and set
    if (((int32_t)packet.param2 & MAV_DO_REPOSITION_FLAGS_CHANGE_MODE) ||
        (plane.control_mode == &plane.mode_guided)) {
        plane.set_mode(plane.mode_guided, ModeReason::GCS_COMMAND);
#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
        plane.guided_state.target_heading_type = GUIDED_HEADING_NONE;
#endif

        // convert to absolute alt
        if (!requested_position.terrain_alt) {
            requested_position.change_alt_frame(Location::AltFrame::ABSOLUTE);
        }

        plane.set_guided_WP(requested_position);

        // Loiter radius for planes. Positive radius in meters, direction is controlled by Yaw (param4) value, parsed above
        if (!isnan(packet.param3) && packet.param3 > 0) {
            plane.mode_guided.set_radius_and_direction(packet.param3, requested_position.loiter_ccw);
        }

        return MAV_RESULT_ACCEPTED;
    }
    return MAV_RESULT_FAILED;
}

/**
 * @brief Handles MAV_CMD_DO_CHANGE_ALTITUDE command during flight
 * 
 * @details Changes the target altitude for the current waypoint or loiter. Works
 *          in AUTO, GUIDED, and other navigation modes. Converts altitude from
 *          MAVLink coordinate frame to ArduPilot Location altitude frame.
 *          
 *          Command Parameters:
 *          - param1: Target altitude in meters
 *          - param2: MAV_FRAME coordinate frame (GLOBAL, TERRAIN, etc.)
 *          
 *          Supported Frames:
 *          - MAV_FRAME_GLOBAL: Altitude relative to MSL
 *          - MAV_FRAME_GLOBAL_RELATIVE_ALT: Altitude relative to home
 *          - MAV_FRAME_GLOBAL_TERRAIN_ALT: Altitude above terrain
 *          
 *          Behavior:
 *          - Constructs temporary Location with new altitude
 *          - Delegates to handle_change_alt_request() for processing
 *          - Updates plane.next_WP_loc altitude immediately
 *          - Navigation tracks to new altitude starting next loop
 * 
 * @param[in] packet COMMAND_INT message with altitude parameters
 * 
 * @return MAV_RESULT ACCEPTED if altitude changed, DENIED if invalid frame
 * 
 * @note Altitude change is immediate (no trajectory planning)
 * @note Works in AUTO, GUIDED, LOITER, and other navigation modes
 * 
 * @see handle_change_alt_request() for altitude update processing
 * @see mavlink_coordinate_frame_to_location_alt_frame() for frame conversion
 */
MAV_RESULT GCS_MAVLINK_Plane::handle_command_int_DO_CHANGE_ALTITUDE(const mavlink_command_int_t &packet)
{
    const float alt = packet.param1;
    MAV_FRAME mav_frame = (MAV_FRAME)packet.param2;
    Location::AltFrame alt_frame;
    if (!mavlink_coordinate_frame_to_location_alt_frame(mav_frame, alt_frame)) {
        return MAV_RESULT_DENIED;
    }
    Location loc {
        0,
        0,
        int32_t(alt * 100),  // m -> cm
        alt_frame,
    };
    handle_change_alt_request(loc);
    return MAV_RESULT_ACCEPTED;
}

#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
/**
 * @brief Handles GUIDED mode commands with rate/slew limiting for smooth control
 * 
 * @details Processes advanced GUIDED mode commands that support rate-limited (slewed)
 *          changes to speed, altitude, and heading. Enables smoother external control
 *          from companion computers or ground stations compared to immediate changes.
 *          Only valid when vehicle is in GUIDED mode to prevent interference with
 *          autonomous failsafe operations.
 *          
 *          Supported Commands:
 *          - MAV_CMD_GUIDED_CHANGE_SPEED: Rate-limited airspeed changes
 *          - MAV_CMD_GUIDED_CHANGE_ALTITUDE: Rate-limited altitude changes
 *          - MAV_CMD_GUIDED_CHANGE_HEADING: Rate-limited heading changes
 *          
 *          Safety Philosophy:
 *          - Commands restricted to GUIDED mode only
 *          - Prevents external control during failsafe modes (RTL, LOITER)
 *          - Ensures autonomous safety mechanisms take precedence
 *          - Rate limiting prevents abrupt maneuvers
 * 
 * @param[in] packet COMMAND_INT message with slew command parameters
 * 
 * @return MAV_RESULT Command execution result
 *         - ACCEPTED: Command processed successfully
 *         - FAILED: Not in GUIDED mode
 *         - DENIED: Invalid parameters
 *         - UNSUPPORTED: Unknown command
 * 
 * @note Only processes commands when plane.control_mode == &plane.mode_guided
 * @warning Commands rejected if not in GUIDED mode for safety
 * 
 * @see ModeGuided::handle_change_airspeed() for airspeed slew implementation
 */
MAV_RESULT GCS_MAVLINK_Plane::handle_command_int_guided_slew_commands(const mavlink_command_int_t &packet)
{
  switch(packet.command) {
    case MAV_CMD_GUIDED_CHANGE_SPEED: {
        /**
         * Handles rate-limited airspeed changes in GUIDED mode
         * 
         * Command Parameters:
         * - param1: Speed type (only SPEED_TYPE_AIRSPEED supported)
         * - param2: Target airspeed in m/s
         * - param3: Acceleration limit in m/s/s (slew rate)
         * 
         * Behavior:
         * - Gradually changes airspeed at specified acceleration
         * - Smoother than immediate speed changes
         * - Useful for companion computer control
         */
        // command is only valid in guided mode
        if (plane.control_mode != &plane.mode_guided) {
            return MAV_RESULT_FAILED;
        }

        // only airspeed commands are supported right now...
        if (int(packet.param1) != SPEED_TYPE_AIRSPEED) {  // since SPEED_TYPE is int in range 0-1 and packet.param1 is a *float* this works.
            return MAV_RESULT_DENIED;
        }

        if (!plane.mode_guided.handle_change_airspeed(packet.param2, packet.param3)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;
    }

     case MAV_CMD_GUIDED_CHANGE_ALTITUDE: {
        /**
         * Handles rate-limited altitude changes in GUIDED mode
         * 
         * Command Parameters:
         * - frame: MAV_FRAME coordinate frame for altitude
         * - z: Target altitude in meters
         * - param3: Vertical velocity limit in m/s (climb/descent rate)
         * 
         * Behavior:
         * - Changes altitude at specified vertical velocity
         * - Prevents abrupt climbs/descents
         * - Records target for trajectory generation
         * 
         * Safety:
         * - Rejects altitude of -1 (default/uninitialized value)
         * - Rejects altitude of 0 (ground level, dangerous)
         * - param3=0 interpreted as maximum rate (1000 m/s)
         */
        // command is only valid in guided
        if (plane.control_mode != &plane.mode_guided) {
            return MAV_RESULT_FAILED;
        }

        // disallow default value of -1 and dangerous value of zero
        if (is_equal(packet.z, -1.0f) || is_equal(packet.z, 0.0f)){
            return MAV_RESULT_DENIED;
        }

        Location::AltFrame new_target_alt_frame;
        if (!mavlink_coordinate_frame_to_location_alt_frame((MAV_FRAME)packet.frame, new_target_alt_frame)) {
            return MAV_RESULT_DENIED;
        }
        // keep a copy of what came in via MAVLink - this is needed for logging, but not for anything else
        plane.guided_state.target_mav_frame = packet.frame;

        const int32_t new_target_alt_cm = packet.z * 100;
        plane.guided_state.target_location.set_alt_cm(new_target_alt_cm, new_target_alt_frame); 
        plane.guided_state.target_alt_time_ms = AP_HAL::millis();

        // param3 contains the desired vertical velocity (not acceleration)
        if (is_zero(packet.param3)) {
            // the user wanted /maximum altitude change rate, pick a large value as close enough
            plane.guided_state.target_alt_rate = 1000.0;
        } else {
            plane.guided_state.target_alt_rate = fabsf(packet.param3);
        }

        return MAV_RESULT_ACCEPTED;
    }

     case MAV_CMD_GUIDED_CHANGE_HEADING: {
        /**
         * Handles rate-limited heading changes in GUIDED mode
         * 
         * Command Parameters:
         * - param1: Heading type (HEADING_TYPE enum)
         *   * HEADING_TYPE_COURSE_OVER_GROUND (0): GPS course
         *   * HEADING_TYPE_HEADING (1): Vehicle heading
         *   * HEADING_TYPE_DEFAULT (4): Cancel heading command
         * - param2: Target heading in degrees [0-360)
         * - param3: Heading change acceleration in deg/s/s (minimum 0.05)
         * 
         * Behavior:
         * - Smoothly changes heading at limited acceleration
         * - COG mode: flies to point in direction, updates prev_WP
         * - Heading mode: maintains vehicle heading
         * - Resets heading controller integrator
         */
        // command is only valid in guided mode
        if (plane.control_mode != &plane.mode_guided) {
            return MAV_RESULT_FAILED;
        }

         // don't accept packets outside of [0-360] degree range
        if (packet.param2 < 0.0f || packet.param2 >= 360.0f) {
            return MAV_RESULT_DENIED;
        }

        float new_target_heading = radians(wrap_180(packet.param2));

        switch(HEADING_TYPE(packet.param1)) {
        case HEADING_TYPE_COURSE_OVER_GROUND:
            // course over ground
            plane.guided_state.target_heading_type = GUIDED_HEADING_COG;
            plane.prev_WP_loc = plane.current_loc;
            break;
        case HEADING_TYPE_HEADING:
            // normal vehicle heading
            plane.guided_state.target_heading_type = GUIDED_HEADING_HEADING;
            break;
        case HEADING_TYPE_DEFAULT:
            plane.guided_state.target_heading_type = GUIDED_HEADING_NONE;
            return MAV_RESULT_ACCEPTED;
        default:
            //  MAV_RESULT_DENIED  means Command is invalid (is supported but has invalid parameters).
            return MAV_RESULT_DENIED;
        }

        plane.g2.guidedHeading.reset_I();

        plane.guided_state.target_heading = new_target_heading;
        plane.guided_state.target_heading_accel_limit = MAX(packet.param3, 0.05f);
        plane.guided_state.target_heading_time_ms = AP_HAL::millis();
        return MAV_RESULT_ACCEPTED;
    }
  }
  // anything else ...
  return MAV_RESULT_UNSUPPORTED;
}
#endif // AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED

/**
 * @brief Main dispatcher for COMMAND_INT MAVLink messages (plane-specific commands)
 * 
 * @details Routes COMMAND_INT messages to appropriate plane-specific command handlers.
 *          COMMAND_INT uses integer lat/lon (1E7 format) and supports more precise
 *          position commands than COMMAND_LONG. This function handles fixed-wing
 *          specific commands; unrecognized commands forwarded to base class.
 *          
 *          Command Categories Handled:
 *          - Flight control: DO_REPOSITION, DO_CHANGE_ALTITUDE, DO_CHANGE_SPEED
 *          - GUIDED mode: GUIDED_CHANGE_SPEED/ALTITUDE/HEADING (slew-enabled)
 *          - Autotune: DO_AUTOTUNE_ENABLE
 *          - Engine: DO_ENGINE_CONTROL (internal combustion)
 *          - Safety: DO_PARACHUTE, DO_GO_AROUND
 *          - Quadplane: DO_MOTOR_TEST, DO_VTOL_TRANSITION, NAV_TAKEOFF
 *          - Mission: MISSION_START, DO_RETURN_PATH_START, DO_LAND_START
 *          - Mode changes: NAV_LOITER_UNLIM, NAV_RETURN_TO_LAUNCH
 *          - Follow: DO_FOLLOW (scripting+follow enabled)
 *          - Altitude: SET_HAGL (external height above ground)
 *          
 *          Base Class Forwarding:
 *          - Unrecognized commands: Forwarded to GCS_MAVLINK::handle_command_int_packet
 *          - Base handles: arm/disarm, RC override, geofence, parameters, etc.
 *          
 *          Result Codes:
 *          - MAV_RESULT_ACCEPTED: Command executed successfully
 *          - MAV_RESULT_DENIED: Command has invalid parameters
 *          - MAV_RESULT_FAILED: Command execution failed
 *          - MAV_RESULT_UNSUPPORTED: Command not recognized
 * 
 * @param[in] packet COMMAND_INT message structure with command and parameters
 * @param[in] msg Complete MAVLink message for extended context
 * 
 * @return MAV_RESULT Command execution result for acknowledgment to GCS
 * 
 * @note Integer coordinates (x=lat*1E7, y=lon*1E7) provide cm-level precision
 * @note Some commands mode-restricted (e.g., GUIDED commands only in GUIDED mode)
 * @warning Mission commands may fail if no valid GPS position
 * 
 * @see handle_command_long_packet() for floating-point coordinate commands
 * @see GCS_MAVLINK::handle_command_int_packet() for base command handling
 */
MAV_RESULT GCS_MAVLINK_Plane::handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    switch(packet.command) {

    case MAV_CMD_DO_AUTOTUNE_ENABLE:
        return handle_MAV_CMD_DO_AUTOTUNE_ENABLE(packet);

    case MAV_CMD_DO_REPOSITION:
        return handle_command_int_do_reposition(packet);

    case MAV_CMD_DO_CHANGE_ALTITUDE:
        return handle_command_int_DO_CHANGE_ALTITUDE(packet);

#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    // special 'slew-enabled' guided commands here... for speed,alt, and direction commands
    case MAV_CMD_GUIDED_CHANGE_SPEED:
    case MAV_CMD_GUIDED_CHANGE_ALTITUDE:
    case MAV_CMD_GUIDED_CHANGE_HEADING:
        return handle_command_int_guided_slew_commands(packet);
#endif

#if AP_SCRIPTING_ENABLED && AP_FOLLOW_ENABLED
    case MAV_CMD_DO_FOLLOW:
        // param1: sysid of target to follow
        if ((packet.param1 > 0) && (packet.param1 <= 255)) {
            plane.g2.follow.set_target_sysid((uint8_t)packet.param1);
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_DENIED;
#endif

#if AP_ICENGINE_ENABLED
    case MAV_CMD_DO_ENGINE_CONTROL:
        if (!plane.g2.ice_control.engine_control(packet.param1, packet.param2, packet.param3, (uint32_t)packet.param4)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;
#endif

    case MAV_CMD_DO_CHANGE_SPEED:
        return handle_command_DO_CHANGE_SPEED(packet);

#if HAL_PARACHUTE_ENABLED
    case MAV_CMD_DO_PARACHUTE:
        return handle_MAV_CMD_DO_PARACHUTE(packet);
#endif

#if HAL_QUADPLANE_ENABLED
    case MAV_CMD_DO_MOTOR_TEST:
        return handle_MAV_CMD_DO_MOTOR_TEST(packet);

    case MAV_CMD_DO_VTOL_TRANSITION:
        return handle_command_DO_VTOL_TRANSITION(packet);

    case MAV_CMD_NAV_TAKEOFF:
        return handle_command_MAV_CMD_NAV_TAKEOFF(packet);
#endif

    case MAV_CMD_DO_GO_AROUND:
        // param1: altitude in meters (0 = current altitude)
        return plane.trigger_land_abort(packet.param1) ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;

    case MAV_CMD_DO_RETURN_PATH_START:
        // attempt to rejoin after the next DO_RETURN_PATH_START command in the mission
        if (plane.have_position && plane.mission.jump_to_closest_mission_leg(plane.current_loc)) {
            plane.mission.set_force_resume(true);
            if (plane.set_mode(plane.mode_auto, ModeReason::GCS_COMMAND)) {
                return MAV_RESULT_ACCEPTED;
            }
            // mode change failed, revert force resume flag
            plane.mission.set_force_resume(false);
        }
        return MAV_RESULT_FAILED;

    case MAV_CMD_DO_LAND_START:
        // attempt to switch to next DO_LAND_START command in the mission
        if (plane.have_position && plane.mission.jump_to_landing_sequence(plane.current_loc)) {
            plane.mission.set_force_resume(true);
            if (plane.set_mode(plane.mode_auto, ModeReason::GCS_COMMAND)) {
                return MAV_RESULT_ACCEPTED;
            }
            // mode change failed, revert force resume flag
            plane.mission.set_force_resume(false);
        }
        return MAV_RESULT_FAILED;

    case MAV_CMD_MISSION_START:
        // param1/param2: first/last item (not supported, must be 0)
        if (!is_zero(packet.param1) || !is_zero(packet.param2)) {
            // first-item/last item not supported
            return MAV_RESULT_DENIED;
        }
        plane.set_mode(plane.mode_auto, ModeReason::GCS_COMMAND);
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_NAV_LOITER_UNLIM:
        plane.set_mode(plane.mode_loiter, ModeReason::GCS_COMMAND);
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        plane.set_mode(plane.mode_rtl, ModeReason::GCS_COMMAND);
        return MAV_RESULT_ACCEPTED;

#if AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED
    case MAV_CMD_SET_HAGL:
        // External height above ground for advanced landing
        plane.handle_external_hagl(packet);
        return MAV_RESULT_ACCEPTED;
#endif
        
    default:
        return GCS_MAVLINK::handle_command_int_packet(packet, msg);
    }
}

/**
 * @brief Handles DO_CHANGE_SPEED command to modify airspeed or groundspeed
 * 
 * @details Changes vehicle target speed during autonomous flight. Command accepted
 *          only in GUIDED or AUTO modes to prevent external interference with
 *          failsafe operations (RTL, LOITER) or pilot-controlled modes (MANUAL,
 *          TRAINING, etc.).
 *          
 *          Safety Philosophy:
 *          - Rejected in failsafe modes: prevents GCS/companion from overriding
 *            autonomous safety behaviors during critical situations
 *          - Rejected in manual modes: pilot has direct control, external commands
 *            inappropriate
 *          - Accepted in GUIDED/AUTO: external mission control expected
 *          
 *          Speed Control:
 *          - SPEED_TYPE_AIRSPEED: Control indicated airspeed (fixed-wing primary)
 *          - SPEED_TYPE_GROUNDSPEED: Control GPS groundspeed
 *          - Throttle percentage: Set throttle directly (percent)
 *          
 *          Use Cases:
 *          - Mission speed changes without modifying mission file
 *          - Companion computer adaptive speed control
 *          - Ground station mission oversight
 * 
 * @param[in] packet COMMAND_INT with speed change parameters
 *                   - param1: Speed type (SPEED_TYPE enum: 0=airspeed, 1=groundspeed, 2=climb, 3=descent)
 *                   - param2: Target speed in m/s (or throttle percentage if type=3)
 *                   - param3: Throttle percentage (0-100, -1=no change, fixed-wing only)
 * 
 * @return MAV_RESULT Command execution result
 *         - ACCEPTED: Speed change applied successfully
 *         - FAILED: Not in GUIDED/AUTO mode, or speed change rejected
 * 
 * @note Immediate speed change (not rate-limited like GUIDED_CHANGE_SPEED)
 * @warning Rejected in RTL/LOITER to maintain predictable failsafe behavior
 * @warning Rejected in MANUAL/TRAINING where pilot has direct control
 * 
 * @see Plane::do_change_speed() for speed change implementation
 * @see handle_command_int_guided_slew_commands() for rate-limited speed changes
 */
MAV_RESULT GCS_MAVLINK_Plane::handle_command_DO_CHANGE_SPEED(const mavlink_command_int_t &packet)
{
        // if we're in failsafe modes (e.g., RTL, LOITER) or in pilot
        // controlled modes (e.g., MANUAL, TRAINING)
        // this command should be ignored since it comes in from GCS
        // or a companion computer:
        if ((!plane.control_mode->is_guided_mode()) &&
            (plane.control_mode != &plane.mode_auto)) {
            // failed
            return MAV_RESULT_FAILED;
        }

        if (plane.do_change_speed((SPEED_TYPE)packet.param1, packet.param2, packet.param3)) {
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;
}

#if HAL_QUADPLANE_ENABLED
#if AP_MAVLINK_COMMAND_LONG_ENABLED
/**
 * @brief Converts MAV_CMD_NAV_TAKEOFF from COMMAND_LONG to COMMAND_INT format
 * 
 * @details Handles conversion of legacy floating-point COMMAND_LONG NAV_TAKEOFF
 *          to modern integer COMMAND_INT format for quadplane VTOL takeoffs.
 *          Uses MAV_FRAME_LOCAL_OFFSET_NED frame (NED local tangent frame with
 *          origin that travels with the vehicle).
 *          
 *          Parameter Mapping:
 *          - Only altitude (param7/z) is used for quadplane takeoffs
 *          - Position parameters (x/y) ignored: takeoff rises vertically
 *          - Coordinate conversion: param7 (up) → -z (down in NED)
 *          - Other params unused for quadplane vertical takeoff
 *          
 *          Coordinate Frame:
 *          - Output: MAV_FRAME_LOCAL_OFFSET_NED
 *          - Z-axis: Positive down (NED convention)
 *          - Input param7: Positive up (altitude above current position)
 *          - Conversion: z = -param7 (up becomes down)
 * 
 * @param[in]  in  COMMAND_LONG message with takeoff altitude in param7
 * @param[out] out COMMAND_INT message with altitude in z field (NED frame)
 * 
 * @note Quadplane takes off vertically; horizontal position ignored
 * @note Only z-axis (altitude) parameter used from input command
 * 
 * @see handle_command_MAV_CMD_NAV_TAKEOFF() for COMMAND_INT takeoff handler
 */
void GCS_MAVLINK_Plane::convert_MAV_CMD_NAV_TAKEOFF_to_COMMAND_INT(const mavlink_command_long_t &in, mavlink_command_int_t &out)
{
    // convert to MAV_FRAME_LOCAL_OFFSET_NED, "NED local tangent frame
    // with origin that travels with the vehicle"
    out = {};
    out.target_system = in.target_system;
    out.target_component = in.target_component;
    out.frame = MAV_FRAME_LOCAL_OFFSET_NED;
    out.command = in.command;
    // out.current = 0;
    // out.autocontinue = 0;
    // out.param1 = in.param1;  // we only use the "z" parameter in this command:
    // out.param2 = in.param2;
    // out.param3 = in.param3;
    // out.param4 = in.param4;
    // out.x = 0;  // we don't handle positioning when doing takeoffs
    // out.y = 0;
    out.z = -in.param7;  // up -> down
}

/**
 * @brief Converts plane-specific COMMAND_LONG messages to COMMAND_INT format
 * 
 * @details Overrides base class to handle plane-specific command conversions.
 *          Currently only handles MAV_CMD_NAV_TAKEOFF for quadplane; all other
 *          commands forwarded to base class implementation.
 *          
 *          Conversion Purpose:
 *          - Modern MAVLink prefers COMMAND_INT (integer coordinates, precise)
 *          - Legacy GCS may send COMMAND_LONG (float coordinates, less precise)
 *          - Conversion allows handling both message types uniformly
 *          
 *          Plane-Specific Conversions:
 *          - NAV_TAKEOFF: Quadplane vertical takeoff with altitude only
 *          
 *          Base Class Conversions:
 *          - All other commands: Standard lat/lon/alt conversion
 * 
 * @param[in]  in    COMMAND_LONG message to convert
 * @param[out] out   COMMAND_INT message (converted output)
 * @param[in]  frame MAVLink coordinate frame for conversion
 * 
 * @note COMMAND_INT provides cm-level precision (lat/lon as int32_t * 1E7)
 * @note COMMAND_LONG has meter-level precision (lat/lon as float)
 * 
 * @see GCS_MAVLINK::convert_COMMAND_LONG_to_COMMAND_INT() for base conversions
 */
void GCS_MAVLINK_Plane::convert_COMMAND_LONG_to_COMMAND_INT(const mavlink_command_long_t &in, mavlink_command_int_t &out, MAV_FRAME frame)
{
    switch (in.command) {
    case MAV_CMD_NAV_TAKEOFF:
        convert_MAV_CMD_NAV_TAKEOFF_to_COMMAND_INT(in, out);
        return;
    }
    return GCS_MAVLINK::convert_COMMAND_LONG_to_COMMAND_INT(in, out, frame);
}
#endif  // AP_MAVLINK_COMMAND_LONG_ENABLED

/**
 * @brief Handles MAV_CMD_NAV_TAKEOFF command for quadplane vertical takeoff
 * 
 * @details Initiates quadplane VTOL takeoff to specified altitude above current
 *          position. Only valid when quadplane hardware is available and enabled.
 *          
 *          Takeoff Behavior:
 *          - Vertical ascent using VTOL motors
 *          - No horizontal movement during ascent
 *          - Altitude relative to current position
 *          - Typically used from ground or low hover
 *          
 *          Coordinate Frame Support:
 *          - MAV_FRAME_LOCAL_OFFSET_NED: NED frame relative to current position
 *            * Z-axis positive down (NED convention)
 *            * Input z converted to altitude (down → up)
 *          - Other frames: Rejected (unsupported)
 *          
 *          Safety Checks:
 *          - Quadplane hardware available (return FAILED if fixed-wing only)
 *          - Altitude valid and achievable
 *          - Arming and pre-takeoff checks (within do_user_takeoff)
 * 
 * @param[in] packet COMMAND_INT with takeoff parameters
 *                   - frame: Must be MAV_FRAME_LOCAL_OFFSET_NED
 *                   - z: Target altitude in meters (NED: positive down, converted to up)
 * 
 * @return MAV_RESULT Command execution result
 *         - ACCEPTED: Takeoff initiated successfully
 *         - DENIED: Invalid coordinate frame
 *         - FAILED: Quadplane not available or takeoff preconditions not met
 * 
 * @note Typically transitions to QLOITER or QHOVER after reaching altitude
 * @warning Only accepts MAV_FRAME_LOCAL_OFFSET_NED coordinate frame
 * @warning Requires quadplane hardware (HAL_QUADPLANE_ENABLED)
 * 
 * @see QuadPlane::do_user_takeoff() for takeoff execution
 * @see convert_MAV_CMD_NAV_TAKEOFF_to_COMMAND_INT() for COMMAND_LONG conversion
 */
MAV_RESULT GCS_MAVLINK_Plane::handle_command_MAV_CMD_NAV_TAKEOFF(const mavlink_command_int_t &packet)
{
    float takeoff_alt = packet.z;
    switch (packet.frame) {
    case MAV_FRAME_LOCAL_OFFSET_NED:  // "NED local tangent frame with origin that travels with the vehicle"
        takeoff_alt = -takeoff_alt;  // down -> up
        break;
    default:
        return MAV_RESULT_DENIED; // "is supported but has invalid parameters"
    }
    if (!plane.quadplane.available()) {
        return MAV_RESULT_FAILED;
    }
    if (!plane.quadplane.do_user_takeoff(takeoff_alt)) {
        return MAV_RESULT_FAILED;
    }
    return MAV_RESULT_ACCEPTED;
}
#endif

/**
 * @brief Handles MAV_CMD_DO_AUTOTUNE_ENABLE command to enable/disable autotune
 * 
 * @details Enables or disables the autotune mode for automatic PID tuning.
 *          Can be invoked from any flight mode to toggle autotune state.
 *          
 *          Autotune Operation:
 *          - When enabled: Initiates automated PID tuning process
 *          - When disabled: Stops tuning, reverts to previous PID values
 *          - Tuning typically requires several minutes of flight
 *          - Tests aircraft response to control inputs
 *          - Adjusts roll, pitch, yaw PID gains automatically
 *          
 *          Fixed-Wing Autotune:
 *          - Requires sufficient altitude (safety margin)
 *          - Tests bank angle response
 *          - Measures pitch response
 *          - Adjusts gains based on aircraft dynamics
 *          
 *          Quadplane Autotune:
 *          - Can tune VTOL controllers independently
 *          - Tests hover stability
 *          - See QAUTOTUNE mode for dedicated quadplane tuning
 * 
 * @param[in] packet COMMAND_INT with autotune control
 *                   - param1: Enable flag (0=disable, non-zero=enable)
 * 
 * @return MAV_RESULT Always returns ACCEPTED (command cannot fail)
 * 
 * @note Enable/disable only; does not start tuning flight automatically
 * @note Actual tuning requires pilot to fly specific maneuvers
 * @warning Requires adequate altitude and safe flight conditions
 * @warning Keep manual override ready during autotune
 * 
 * @see Plane::autotune_enable() for autotune state management
 * @see Mode::autotune for dedicated autotune mode
 */
MAV_RESULT GCS_MAVLINK_Plane::handle_MAV_CMD_DO_AUTOTUNE_ENABLE(const mavlink_command_int_t &packet)
{
        // param1 : enable/disable
        plane.autotune_enable(!is_zero(packet.param1));
        return MAV_RESULT_ACCEPTED;
}

#if HAL_PARACHUTE_ENABLED
/**
 * @brief Handles MAV_CMD_DO_PARACHUTE command to control parachute deployment
 * 
 * @details Controls emergency parachute system for fixed-wing recovery. Supports
 *          enable/disable of parachute system and manual release trigger. Parachute
 *          provides last-resort recovery mechanism for catastrophic failures.
 *          
 *          Parachute Actions:
 *          - DISABLE: Deactivates parachute system (prevents automatic/manual release)
 *          - ENABLE: Activates parachute system (allows release)
 *          - RELEASE: Manually triggers parachute deployment
 *          
 *          Release Safety Checks:
 *          - Parachute must be enabled before release
 *          - Cannot release if already deployed
 *          - Altitude check performed (minimum altitude required)
 *          - Manual release requires explicit confirmation
 *          
 *          Typical Use Cases:
 *          - Pre-flight: Enable parachute system
 *          - Emergency: Manual release command from GCS
 *          - Post-flight: Disable to prevent accidental deployment
 *          - Testing: Enable/disable without release
 *          
 *          Hardware Integration:
 *          - Parachute deployed via servo or pyrotechnic actuator
 *          - One-time use (cannot be reset in-flight)
 *          - Deployment typically triggers immediate vehicle shutdown
 * 
 * @param[in] packet COMMAND_INT with parachute action
 *                   - param1: Action (0=DISABLE, 1=ENABLE, 2=RELEASE)
 * 
 * @return MAV_RESULT Command execution result
 *         - ACCEPTED: Command executed successfully
 *         - FAILED: Parachute already released, not enabled, altitude too low,
 *                   or invalid action
 * 
 * @note RELEASE performs altitude check via parachute_manual_release()
 * @warning Parachute deployment is irreversible and terminates flight
 * @warning Minimum altitude required for effective parachute deployment
 * @warning Typically causes vehicle loss; use only in emergencies
 * 
 * @see Plane::parachute_manual_release() for release safety checks
 * @see AP_Parachute for automatic deployment triggers
 */
MAV_RESULT GCS_MAVLINK_Plane::handle_MAV_CMD_DO_PARACHUTE(const mavlink_command_int_t &packet)
{
        // configure or release parachute
        switch ((uint16_t)packet.param1) {
        case PARACHUTE_DISABLE:
            plane.parachute.enabled(false);
            return MAV_RESULT_ACCEPTED;
        case PARACHUTE_ENABLE:
            plane.parachute.enabled(true);
            return MAV_RESULT_ACCEPTED;
        case PARACHUTE_RELEASE:
            // treat as a manual release which performs some additional check of altitude
            if (plane.parachute.released()) {
                gcs().send_text(MAV_SEVERITY_NOTICE, "Parachute already released");
                return MAV_RESULT_FAILED;
            }
            if (!plane.parachute.enabled()) {
                gcs().send_text(MAV_SEVERITY_NOTICE, "Parachute not enabled");
                return MAV_RESULT_FAILED;
            }
            if (!plane.parachute_manual_release()) {
                return MAV_RESULT_FAILED;
            }
            return MAV_RESULT_ACCEPTED;
        default:
            break;
        }
        return MAV_RESULT_FAILED;
}
#endif


#if HAL_QUADPLANE_ENABLED
/**
 * @brief Handles MAV_CMD_DO_MOTOR_TEST command for quadplane motor testing
 * 
 * @details Tests individual VTOL motors or sequences of motors for quadplane
 *          pre-flight verification. Essential for confirming motor operation,
 *          direction, and response before flight.
 *          
 *          Safety Requirements:
 *          - Vehicle must be disarmed or in safe test mode
 *          - Propellers must be removed or aircraft secured
 *          - Test duration limited by timeout parameter
 *          - Automatic test termination on timeout or failure
 *          
 *          Test Parameters:
 *          - Motor Selection: Individual motor number (1 to max motors)
 *          - Throttle Type:
 *            * 0: Throttle percentage (0-100%)
 *            * 1: PWM value (typically 1000-2000 μs)
 *            * 2: Pilot throttle channel pass-through
 *          - Motor Count: Number of motors to test in sequence
 *          - Timeout: Maximum test duration per motor (seconds)
 *          
 *          Test Sequence:
 *          - Tests motors in numerical order starting from param1
 *          - Each motor tested for specified duration
 *          - Automatic progression through sequence
 *          - Test stops on timeout or command cancellation
 *          
 *          Typical Usage:
 *          - Pre-flight verification of motor operation
 *          - Motor direction confirmation
 *          - ESC calibration verification
 *          - Troubleshooting motor/ESC issues
 * 
 * @param[in] packet COMMAND_INT with motor test parameters
 *                   - param1: Motor sequence number (1 to N)
 *                   - param2: Throttle type (0=percent, 1=PWM, 2=passthrough)
 *                   - param3: Throttle value (range depends on param2)
 *                   - param4: Test timeout in seconds
 *                   - x: Motor count (number of motors in test sequence)
 * 
 * @return MAV_RESULT Result from quadplane motor test initiation
 *         - ACCEPTED: Motor test started successfully
 *         - FAILED: Cannot start test (armed, in flight, invalid parameters)
 * 
 * @warning Remove propellers or secure vehicle before testing
 * @warning Never perform motor test on armed vehicle in flight
 * @warning Test at low throttle initially to verify motor direction
 * 
 * @see QuadPlane::mavlink_motor_test_start() for test execution
 */
MAV_RESULT GCS_MAVLINK_Plane::handle_MAV_CMD_DO_MOTOR_TEST(const mavlink_command_int_t &packet)
{
        // param1 : motor sequence number (a number from 1 to max number of motors on the vehicle)
        // param2 : throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)
        // param3 : throttle (range depends upon param2)
        // param4 : timeout (in seconds)
        // param5 : motor count (number of motors to test in sequence)
        return plane.quadplane.mavlink_motor_test_start(chan,
                                                        (uint8_t)packet.param1,
                                                        (uint8_t)packet.param2,
                                                        (uint16_t)packet.param3,
                                                        packet.param4,
                                                        (uint8_t)packet.x);
}

/**
 * @brief Handles MAV_CMD_DO_VTOL_TRANSITION command for fixed/VTOL mode changes
 * 
 * @details Commands quadplane to transition between fixed-wing and VTOL flight
 *          modes. Manages the transition state machine that coordinates motor
 *          activation, tilt servo movement, and flight controller changeover.
 *          
 *          VTOL State Transitions:
 *          - MAV_VTOL_STATE_MC: Transition to multicopter (VTOL motors active)
 *          - MAV_VTOL_STATE_FW: Transition to fixed-wing (forward flight)
 *          
 *          Transition Process:
 *          - Gradual motor throttle ramp up/down
 *          - Tilt servo movement (for tilt-rotor configurations)
 *          - Airspeed monitoring during transition
 *          - Altitude maintenance during changeover
 *          - Control authority handoff between controllers
 *          
 *          Safety Considerations:
 *          - Minimum altitude required for transition
 *          - Airspeed conditions must be met
 *          - Cannot transition during critical flight phases
 *          - Automatic abort if transition fails
 *          
 *          Typical Use Cases:
 *          - Takeoff: MC → FW after vertical climb
 *          - Landing approach: FW → MC for precision landing
 *          - Manual transition override during mission
 *          - Emergency transition to VTOL for controlled landing
 * 
 * @param[in] packet COMMAND_INT with transition command
 *                   - param1: Target VTOL state (MAV_VTOL_STATE enum)
 *                             * MAV_VTOL_STATE_MC (2): Transition to multicopter
 *                             * MAV_VTOL_STATE_FW (3): Transition to fixed-wing
 * 
 * @return MAV_RESULT Transition command result
 *         - ACCEPTED: Transition initiated successfully
 *         - FAILED: Cannot transition (altitude too low, airspeed inadequate,
 *                   invalid state, or transition already in progress)
 * 
 * @note Transition may take several seconds to complete
 * @warning Requires sufficient altitude for safe transition
 * @warning Airspeed must be adequate for fixed-wing transition
 * @warning Do not transition during critical flight phases (takeoff/landing)
 * 
 * @see QuadPlane::handle_do_vtol_transition() for transition state machine
 * @see QuadPlane::transition for transition controller
 */
MAV_RESULT GCS_MAVLINK_Plane::handle_command_DO_VTOL_TRANSITION(const mavlink_command_int_t &packet)
{
        if (!plane.quadplane.handle_do_vtol_transition((enum MAV_VTOL_STATE)packet.param1)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;
}
#endif

/**
 * @brief Handles MANUAL_CONTROL packet to override RC inputs with joystick/GCS
 * 
 * @details Processes MAVLink manual control inputs from ground station joystick
 *          or companion computer, allowing external control to override RC receiver
 *          inputs. Provides alternative control method when RC link unavailable or
 *          for companion computer flight control.
 *          
 *          Control Axis Mapping:
 *          - packet.y → Roll channel (1000-2000 PWM range)
 *          - packet.x → Pitch channel (1000-2000 PWM range, reversed)
 *          - packet.z → Throttle channel (0-1000 normalized range)
 *          - packet.r → Rudder/Yaw channel (1000-2000 PWM range)
 *          
 *          Input Scaling:
 *          - MAVLink values: Typically -1000 to +1000 (normalized)
 *          - PWM conversion: Mapped to standard RC ranges
 *          - Pitch reversed: MAVLink convention differs from RC
 *          
 *          Manual Override Behavior:
 *          - Overrides RC receiver inputs temporarily
 *          - Times out if no updates received
 *          - Allows smooth handoff between RC and external control
 *          - RC regains control on override timeout
 *          
 *          Use Cases:
 *          - Ground station joystick control (RC backup)
 *          - Companion computer direct control
 *          - Testing without RC transmitter
 *          - Remote operation via telemetry link
 * 
 * @param[in] packet MAVLink MANUAL_CONTROL message
 *                   - x: Pitch axis (-1000 to +1000)
 *                   - y: Roll axis (-1000 to +1000)
 *                   - z: Throttle axis (0 to +1000)
 *                   - r: Yaw/rudder axis (-1000 to +1000)
 * @param[in] tnow   Current time in milliseconds (for timeout tracking)
 * 
 * @note Pitch reversed (true parameter) to match ArduPilot convention
 * @note Throttle uses 0-1000 range (not -1000 to +1000)
 * @warning External control overrides RC; ensure failsafe configured
 * @warning Timeout protection reverts to RC if updates stop
 * 
 * @see manual_override() for override mechanism and timeout handling
 */
void GCS_MAVLINK_Plane::handle_manual_control_axes(const mavlink_manual_control_t &packet, const uint32_t tnow)
{
    manual_override(plane.channel_roll, packet.y, 1000, 2000, tnow);
    manual_override(plane.channel_pitch, packet.x, 1000, 2000, tnow, true);
    manual_override(plane.channel_throttle, packet.z, 0, 1000, tnow);
    manual_override(plane.channel_rudder, packet.r, 1000, 2000, tnow);
}

/**
 * @brief Main MAVLink message dispatcher for plane-specific message handling
 * 
 * @details Routes incoming MAVLink messages to appropriate plane-specific handlers.
 *          Handles messages unique to fixed-wing/quadplane operations that require
 *          vehicle-specific processing. Messages not handled here forwarded to
 *          base class for common MAVLink functionality.
 *          
 *          Plane-Specific Messages Handled:
 *          - TERRAIN_DATA/TERRAIN_CHECK: Terrain following data from GCS
 *          - SET_ATTITUDE_TARGET: External attitude control (GUIDED mode)
 *          - SET_POSITION_TARGET_LOCAL_NED: Local position control commands
 *          - SET_POSITION_TARGET_GLOBAL_INT: Global position control commands
 *          
 *          Message Processing Flow:
 *          1. Message received from MAVLink parser
 *          2. Message ID checked against plane-specific handlers
 *          3. If match: Route to specialized handler
 *          4. If no match: Forward to base class (GCS_MAVLINK::handle_message)
 *          5. Base class handles common messages (HEARTBEAT, PARAM, etc.)
 *          
 *          External Control Messages:
 *          - Attitude/position targets only accepted in GUIDED mode
 *          - Safety check: Prevents external control during failsafe
 *          - Timeout protection: Commands expire if not refreshed
 *          
 *          Terrain Following:
 *          - Receives terrain height data from ground station
 *          - Enables terrain-relative altitude control
 *          - Used for low-level flight over varying terrain
 * 
 * @param[in] msg MAVLink message to process
 *                - msgid: Message ID for routing
 *                - payload: Message-specific data
 *                - chan: Source MAVLink channel
 * 
 * @note Called by MAVLink parser for each received message
 * @note Base class handles all common MAVLink protocol messages
 * @warning Attitude/position control requires GUIDED mode for safety
 * 
 * @see handle_set_attitude_target() for external attitude control
 * @see handle_set_position_target_global_int() for position commands
 * @see GCS_MAVLINK::handle_message() for common message handling
 */
void GCS_MAVLINK_Plane::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {

    case MAVLINK_MSG_ID_TERRAIN_DATA:
    case MAVLINK_MSG_ID_TERRAIN_CHECK:
#if AP_TERRAIN_AVAILABLE
        plane.terrain.handle_data(chan, msg);
#endif
        break;

    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
        handle_set_attitude_target(msg);
        break;

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
        handle_set_position_target_local_ned(msg);
        break;

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:
        handle_set_position_target_global_int(msg);
        break;

    default:
        GCS_MAVLINK::handle_message(msg);
        break;
    } // end switch
} // end handle mavlink

/**
 * @brief Handles SET_ATTITUDE_TARGET message for external attitude control
 * 
 * @details Allows companion computer or ground station to directly command
 *          vehicle attitude (roll/pitch/yaw) and throttle in GUIDED mode.
 *          Provides low-level control interface for vision-based navigation,
 *          obstacle avoidance, or custom flight controllers.
 *          
 *          Safety Restriction:
 *          - ONLY accepted in GUIDED mode
 *          - Rejected in all other modes (RTL, CIRCLE, AUTO, etc.)
 *          - Prevents external control from interfering with autonomous
 *            failsafe behaviors or pilot manual control
 *          - FENCE_ACTION=4 (RTL) recommended with external control
 *          
 *          Attitude Representation:
 *          - Input: Quaternion [w, x, y, z] (attitude target)
 *          - Converted to: Euler angles (roll, pitch, yaw)
 *          - Units: Quaternion → degrees → centidegrees
 *          - Frame: Body frame (aircraft-relative)
 *          
 *          Type Mask (Inverted):
 *          - Bit set = Ignore corresponding field
 *          - Bit clear = Use corresponding field
 *          - Mask inverted (XOR 0xFF) for easier interpretation
 *          - Bits: [attitude(8), throttle(7), reserved(6), -, -, yaw_rate(3), pitch_rate(2), roll_rate(1)]
 *          
 *          Selective Control:
 *          - Can control individual axes (roll, pitch, yaw) independently
 *          - Throttle optional (bit 7)
 *          - Unused axes retain previous values
 *          - Allows partial attitude control
 *          
 *          Timeout Protection:
 *          - Each axis has independent timestamp
 *          - Commands expire if not refreshed (typically 500ms)
 *          - Automatic reversion to mode default behavior on timeout
 *          - Prevents runaway from stale commands
 *          
 *          Typical Applications:
 *          - Vision-based precision landing
 *          - Obstacle avoidance maneuvers
 *          - Formation flying
 *          - Custom autopilot overlay
 * 
 * @param[in] msg MAVLink SET_ATTITUDE_TARGET message
 *                - q[4]: Attitude quaternion [w, x, y, z]
 *                - type_mask: Fields to ignore (inverted bitmask)
 *                - thrust: Throttle 0.0-1.0 (if not masked)
 * 
 * @note Silently ignored if not in GUIDED mode (no error response)
 * @note Attitude updates at message rate (typically 10-50 Hz)
 * @warning Requires GUIDED mode; commands ignored in other modes for safety
 * @warning Ensure proper timeout handling on companion computer side
 * @warning Test thoroughly before using on actual aircraft
 * 
 * @see plane.guided_state for forced attitude/throttle storage
 */
void GCS_MAVLINK_Plane::handle_set_attitude_target(const mavlink_message_t &msg)
    {
        // Only allow companion computer (or other external controller) to
        // control attitude in GUIDED mode.  We DON'T want external control
        // in e.g., RTL, CICLE. Specifying a single mode for companion
        // computer control is more safe (even more so when using
        // FENCE_ACTION = 4 for geofence failures).
        if (plane.control_mode != &plane.mode_guided) { // don't screw up failsafes
            return;
        }

        mavlink_set_attitude_target_t att_target;
        mavlink_msg_set_attitude_target_decode(&msg, &att_target);

        // Mappings: If any of these bits are set, the corresponding input should be ignored.
        // NOTE, when parsing the bits we invert them for easier interpretation but transport has them inverted
        // bit 1: body roll rate
        // bit 2: body pitch rate
        // bit 3: body yaw rate
        // bit 4: unknown
        // bit 5: unknown
        // bit 6: reserved
        // bit 7: throttle
        // bit 8: attitude

        // if not setting all Quaternion values, use _rate flags to indicate which fields.

        // Extract the Euler roll angle from the Quaternion.
        Quaternion q(att_target.q[0], att_target.q[1],
                att_target.q[2], att_target.q[3]);

        // NOTE: att_target.type_mask is inverted for easier interpretation
        att_target.type_mask = att_target.type_mask ^ 0xFF;

        uint8_t attitude_mask = att_target.type_mask & 0b10000111; // q plus rpy

        uint32_t now = AP_HAL::millis();
        if ((attitude_mask & 0b10000001) ||    // partial, including roll
                (attitude_mask == 0b10000000)) { // all angles
            plane.guided_state.forced_rpy_cd.x = degrees(q.get_euler_roll()) * 100.0f;

            // Update timer for external roll to the nav control
            plane.guided_state.last_forced_rpy_ms.x = now;
        }

        if ((attitude_mask & 0b10000010) ||    // partial, including pitch
                (attitude_mask == 0b10000000)) { // all angles
            plane.guided_state.forced_rpy_cd.y = degrees(q.get_euler_pitch()) * 100.0f;

            // Update timer for external pitch to the nav control
            plane.guided_state.last_forced_rpy_ms.y = now;
        }

        if ((attitude_mask & 0b10000100) ||    // partial, including yaw
                (attitude_mask == 0b10000000)) { // all angles
            plane.guided_state.forced_rpy_cd.z = degrees(q.get_euler_yaw()) * 100.0f;

            // Update timer for external yaw to the nav control
            plane.guided_state.last_forced_rpy_ms.z = now;
        }
        if (att_target.type_mask & 0b01000000) { // throttle
            plane.guided_state.forced_throttle = att_target.thrust * 100.0f;

            // Update timer for external throttle
            plane.guided_state.last_forced_throttle_ms = now;
        }
    }

/**
 * @brief Handles SET_POSITION_TARGET_LOCAL_NED for local position offsets
 * 
 * @details Processes local position target commands in NED frame relative to
 *          current position. Currently implements altitude control only; full
 *          3D position control not yet implemented for fixed-wing.
 *          
 *          Current Implementation:
 *          - Altitude offset only (Z-axis in NED frame)
 *          - Position (X/Y) not implemented
 *          - Velocity/acceleration not implemented
 *          
 *          Coordinate Frame:
 *          - MAV_FRAME_LOCAL_OFFSET_NED only (required)
 *          - North-East-Down convention
 *          - Z-axis: Positive down (NED standard)
 *          - Offset relative to current position
 *          
 *          Safety Restrictions:
 *          - ONLY accepted in GUIDED mode
 *          - Rejected in other modes (RTL, AUTO, manual modes)
 *          - Prevents external interference with autonomous behaviors
 *          
 *          Altitude Adjustment:
 *          - packet.z in meters (NED: positive = down)
 *          - Converted to cm: multiply by 100
 *          - Negated: down (NED) → up (altitude increase)
 *          - Added to next_WP_loc.alt for incremental change
 *          
 *          Use Cases:
 *          - Companion computer altitude adjustments
 *          - Vision-based terrain following offsets
 *          - Obstacle clearance altitude increases
 *          - Dynamic altitude mission adjustments
 *          
 *          Future Enhancement:
 *          - Full 3D position control (X/Y/Z)
 *          - Velocity targets
 *          - Acceleration targets
 *          - Type mask field support
 * 
 * @param[in] msg MAVLink SET_POSITION_TARGET_LOCAL_NED message
 *                - coordinate_frame: Must be MAV_FRAME_LOCAL_OFFSET_NED
 *                - z: Altitude offset in meters (positive down)
 *                - x, y: Ignored (not implemented)
 *                - vx, vy, vz: Ignored (not implemented)
 * 
 * @note Only altitude (Z-axis) implemented; horizontal position ignored
 * @note Silently rejected if not in GUIDED mode or wrong frame
 * @warning Limited implementation: Only altitude offset supported
 * @warning No bounds checking on altitude change
 * 
 * @see handle_set_position_target_global_int() for global position commands
 * @see plane.next_WP_loc for target waypoint location
 */
void GCS_MAVLINK_Plane::handle_set_position_target_local_ned(const mavlink_message_t &msg)
    {
        // decode packet
        mavlink_set_position_target_local_ned_t packet;
        mavlink_msg_set_position_target_local_ned_decode(&msg, &packet);

        // exit if vehicle is not in Guided mode
        if (plane.control_mode != &plane.mode_guided) {
            return;
        }

        // only local moves for now
        if (packet.coordinate_frame != MAV_FRAME_LOCAL_OFFSET_NED) {
            return;
        }

        // just do altitude for now
        plane.next_WP_loc.alt += -packet.z*100.0;
        gcs().send_text(MAV_SEVERITY_INFO, "Change alt to %.1f",
                        (double)((plane.next_WP_loc.alt - plane.home.alt)*0.01));
    }

/**
 * @brief Handles SET_POSITION_TARGET_GLOBAL_INT for global position commands
 * 
 * @details Processes global position target commands with GPS coordinates and
 *          altitude. Currently implements altitude control only; lat/lon position
 *          targets not yet implemented for fixed-wing in this handler.
 *          
 *          Current Implementation:
 *          - Altitude target (Z-axis) with multiple frame support
 *          - Position (lat/lon) not implemented in this handler
 *          - Velocity/acceleration not implemented
 *          
 *          Coordinate Frames Supported:
 *          - MAV_FRAME_GLOBAL: WGS84 with altitude relative to sea level (MSL)
 *          - MAV_FRAME_GLOBAL_RELATIVE_ALT: Altitude relative to home
 *          - MAV_FRAME_GLOBAL_TERRAIN_ALT: Altitude above terrain
 *          - MAV_FRAME_GLOBAL_INT: Integer lat/lon (1e7 degrees)
 *          
 *          Safety Restrictions:
 *          - ONLY accepted in GUIDED mode
 *          - Rejected in all other modes (RTL, CIRCLE, AUTO, manual)
 *          - Prevents external control during failsafes
 *          - FENCE_ACTION=4 (RTL) recommended with external control
 *          
 *          Type Mask (Inverted Logic):
 *          - Bit set = IGNORE corresponding field
 *          - Bit clear = USE corresponding field
 *          - POSITION_TARGET_TYPEMASK_Z_IGNORE: Ignore altitude if set
 *          - Currently only altitude (Z) field processed
 *          
 *          Altitude Processing:
 *          - Input: meters (float)
 *          - Converted: meters → centimeters (×100)
 *          - Frame: Specified by coordinate_frame field
 *          - Validation: Frame must be valid Location::AltFrame
 *          
 *          Error Handling:
 *          - Invalid frame: Warning message sent, command rejected
 *          - Whole message rejected if any field invalid
 *          - No partial application of commands
 *          
 *          Use Cases:
 *          - Companion computer altitude commands
 *          - Ground station altitude overrides
 *          - Vision-based altitude adjustments
 *          - Terrain-relative altitude control
 *          
 *          Future Enhancement:
 *          - Lat/lon position targeting
 *          - Velocity/acceleration fields
 *          - Yaw angle targeting
 *          - Full type mask support
 * 
 * @param[in] msg MAVLink SET_POSITION_TARGET_GLOBAL_INT message
 *                - coordinate_frame: Altitude frame (GLOBAL, RELATIVE, TERRAIN)
 *                - type_mask: Fields to ignore (bit set = ignore)
 *                - lat_int: Latitude (ignored currently)
 *                - lon_int: Longitude (ignored currently)
 *                - alt: Altitude in meters (frame-dependent)
 *                - vx, vy, vz: Velocity (ignored currently)
 * 
 * @note Only altitude implemented; lat/lon position ignored
 * @note Type mask checked: altitude only processed if not masked
 * @note Silently rejected if not in GUIDED mode
 * @warning Invalid coordinate frame causes warning and rejection
 * @warning Partial implementation: Only altitude control available
 * 
 * @see handle_change_alt_request() for altitude change processing
 * @see handle_set_position_target_local_ned() for local frame commands
 */
void GCS_MAVLINK_Plane::handle_set_position_target_global_int(const mavlink_message_t &msg)
    {
        // Only want to allow companion computer position control when
        // in a certain mode to avoid inadvertently sending these
        // kinds of commands when the autopilot is responding to problems
        // in modes such as RTL, CIRCLE, etc.  Specifying ONLY one mode
        // for companion computer control is more safe (provided
        // one uses the FENCE_ACTION = 4 (RTL) for geofence failures).
        if (plane.control_mode != &plane.mode_guided) {
            //don't screw up failsafes
            return;
        }

        mavlink_set_position_target_global_int_t pos_target;
        mavlink_msg_set_position_target_global_int_decode(&msg, &pos_target);

        Location::AltFrame frame;
        if (!mavlink_coordinate_frame_to_location_alt_frame((MAV_FRAME)pos_target.coordinate_frame, frame)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Invalid coord frame in SET_POSTION_TARGET_GLOBAL_INT");
            // Even though other parts of the command may be valid, reject the whole thing.
            return;
        }

        // Unexpectedly, the mask is expecting "ones" for dimensions that should
        // be IGNORED rather than INCLUDED.  See mavlink documentation of the
        // SET_POSITION_TARGET_GLOBAL_INT message, type_mask field.
        const bool alt_ignore = (pos_target.type_mask & POSITION_TARGET_TYPEMASK_Z_IGNORE);
        if (!alt_ignore) {
            Location loc {
                0,  // lat
                0,  // lng
                int32_t(pos_target.alt * 100),  // m -> cm
                frame,
            };
            handle_change_alt_request(loc);
        }
    }

/**
 * @brief Handles MAV_CMD_DO_SET_MISSION_CURRENT command
 * 
 * @details Changes the current mission waypoint during AUTO mode flight.
 *          Allows ground station or companion computer to skip waypoints,
 *          jump to specific mission items, or restart mission segments.
 *          
 *          Base Processing:
 *          - Delegates to GCS_MAVLINK::handle_command_do_set_mission_current()
 *          - Validates mission item index
 *          - Updates mission state machine
 *          - Sends acknowledgment to GCS
 *          
 *          Plane-Specific Behavior:
 *          - Disables crosstrack correction for new waypoint
 *          - Prevents immediate lateral offset at waypoint change
 *          - Allows smooth transition to new path
 *          
 *          Mission Resume Logic:
 *          - If in AUTO mode and mission stopped: Resume mission
 *          - Enables seamless mission continuation after jump
 *          - Common after DO_LAND_START or DO_RETURN_PATH_START
 *          
 *          Crosstrack Handling:
 *          - next_wp_crosstrack set to false
 *          - Vehicle flies direct to new waypoint
 *          - No lateral correction to previous path
 *          - Prevents confusing behavior on waypoint skip
 *          
 *          Use Cases:
 *          - GCS manual waypoint skip
 *          - Emergency mission segment jump
 *          - Landing sequence initiation
 *          - Mission restart without full reboot
 *          
 *          Synchronization:
 *          - Must match handle_mission_set_current() behavior
 *          - Two entry points for same functionality
 *          - Keep implementations consistent
 * 
 * @param[in] packet MAV_CMD_DO_SET_MISSION_CURRENT command
 *                   - param1: Mission sequence number (waypoint index)
 * 
 * @return MAV_RESULT_ACCEPTED if successful
 * @return MAV_RESULT_DENIED if invalid mission index
 * @return MAV_RESULT_FAILED if mission system error
 * 
 * @note Crosstrack disabled to prevent lateral offset on waypoint change
 * @note Mission auto-resumes if in AUTO mode and stopped
 * @warning Keep synchronized with handle_mission_set_current()
 * 
 * @see handle_mission_set_current() for MISSION_SET_CURRENT message handler
 * @see plane.auto_state.next_wp_crosstrack for path tracking behavior
 */
MAV_RESULT GCS_MAVLINK_Plane::handle_command_do_set_mission_current(const mavlink_command_int_t &packet)
{
    const MAV_RESULT result = GCS_MAVLINK::handle_command_do_set_mission_current(packet);
    if (result != MAV_RESULT_ACCEPTED) {
        return result;
    }

    // if you change this you must change handle_mission_set_current
    plane.auto_state.next_wp_crosstrack = false;
    if (plane.control_mode == &plane.mode_auto && plane.mission.state() == AP_Mission::MISSION_STOPPED) {
        plane.mission.resume();
    }

    return result;
}

#if AP_MAVLINK_MISSION_SET_CURRENT_ENABLED
/**
 * @brief Handles MISSION_SET_CURRENT MAVLink message
 * 
 * @details Alternative entry point for changing current mission waypoint.
 *          Provides identical functionality to MAV_CMD_DO_SET_MISSION_CURRENT
 *          but via MISSION_SET_CURRENT message instead of command.
 *          
 *          Message vs Command:
 *          - MISSION_SET_CURRENT: Legacy MAVLink message format
 *          - MAV_CMD_DO_SET_MISSION_CURRENT: Modern command format
 *          - Both supported for backward compatibility
 *          - Identical behavior and side effects
 *          
 *          Base Processing:
 *          - Delegates to GCS_MAVLINK::handle_mission_set_current()
 *          - Updates mission item index
 *          - Validates mission bounds
 *          - Sends acknowledgment
 *          
 *          Plane-Specific Behavior:
 *          - Disables crosstrack correction (next_wp_crosstrack = false)
 *          - Resumes mission if in AUTO mode and stopped
 *          - Identical to handle_command_do_set_mission_current()
 *          
 *          Crosstrack Behavior:
 *          - Prevents lateral correction to old path
 *          - Vehicle flies direct to new waypoint
 *          - Avoids confusing offsets on waypoint skip
 *          
 *          Mission State Management:
 *          - AUTO mode + STOPPED → Resume mission
 *          - Enables smooth continuation after jumps
 *          - Required for landing sequence restarts
 *          
 *          Synchronization Requirement:
 *          - MUST match handle_command_do_set_mission_current() exactly
 *          - Two code paths for same logical operation
 *          - Any changes must be applied to both functions
 * 
 * @param[in,out] mission Reference to mission manager object
 * @param[in] msg MAVLink MISSION_SET_CURRENT message
 *                - seq: Mission item sequence number to set as current
 * 
 * @note Conditional compilation: Requires AP_MAVLINK_MISSION_SET_CURRENT_ENABLED
 * @note Keep synchronized with handle_command_do_set_mission_current()
 * @warning Must maintain identical behavior to command handler
 * 
 * @see handle_command_do_set_mission_current() for command-based equivalent
 * @see plane.auto_state.next_wp_crosstrack for path tracking control
 */
void GCS_MAVLINK_Plane::handle_mission_set_current(AP_Mission &mission, const mavlink_message_t &msg)
{
    // if you change this you must change handle_command_do_set_mission_current
    plane.auto_state.next_wp_crosstrack = false;
    GCS_MAVLINK::handle_mission_set_current(mission, msg);
    if (plane.control_mode == &plane.mode_auto && plane.mission.state() == AP_Mission::MISSION_STOPPED) {
        plane.mission.resume();
    }
}
#endif

/**
 * @brief Reports MAVLink protocol capabilities to ground station
 * 
 * @details Returns bitmask of supported MAVLink protocol features for ArduPlane.
 *          Ground stations use this to enable/disable features based on autopilot
 *          capabilities, ensuring compatibility and proper UI presentation.
 *          
 *          Capabilities Advertised:
 *          
 *          MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT:
 *          - Mission items with floating-point parameters supported
 *          - Precise waypoint coordinates and altitudes
 *          - Required for accurate navigation
 *          
 *          MAV_PROTOCOL_CAPABILITY_COMMAND_INT:
 *          - COMMAND_INT message format supported
 *          - Integer lat/lon coordinates (1e7 degrees)
 *          - Modern command protocol preferred over COMMAND_LONG
 *          
 *          MAV_PROTOCOL_CAPABILITY_MISSION_INT:
 *          - MISSION_ITEM_INT message format supported
 *          - Integer coordinates for mission waypoints
 *          - Reduces floating-point precision issues
 *          
 *          MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT:
 *          - External position commands supported in GUIDED mode
 *          - Enables companion computer control
 *          - Global coordinate frame position targeting
 *          
 *          MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET:
 *          - External attitude commands supported in GUIDED mode
 *          - Quaternion attitude control
 *          - Body frame angular rate control
 *          
 *          MAV_PROTOCOL_CAPABILITY_TERRAIN (conditional):
 *          - Terrain following and terrain database supported
 *          - Only advertised if terrain system enabled and available
 *          - Requires terrain data loaded on SD card
 *          
 *          Base Capabilities:
 *          - GCS_MAVLINK::capabilities() provides common features
 *          - Parameter protocol, log download, etc.
 *          - Vehicle-agnostic capabilities
 *          
 *          Ground Station Usage:
 *          - Query via REQUEST_AUTOPILOT_CAPABILITIES command
 *          - Enables appropriate UI elements
 *          - Disables unsupported features gracefully
 *          - Determines command format to use
 *          
 *          Capability Detection:
 *          - GCS can test individual bits to check support
 *          - Bitwise AND with specific capability flag
 *          - Zero = not supported, non-zero = supported
 * 
 * @return uint64_t Bitmask of MAV_PROTOCOL_CAPABILITY_* flags
 *         - Each bit represents a specific protocol capability
 *         - Multiple capabilities combined with bitwise OR
 *         - Terrain capability conditional on system state
 * 
 * @note Terrain capability dynamically enabled based on terrain.enabled()
 * @note Includes base GCS capabilities from parent class
 * 
 * @see MAV_PROTOCOL_CAPABILITY enum for capability bit definitions
 * @see GCS_MAVLINK::capabilities() for base vehicle-agnostic capabilities
 */
uint64_t GCS_MAVLINK_Plane::capabilities() const
{
    return (MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
            MAV_PROTOCOL_CAPABILITY_COMMAND_INT |
            MAV_PROTOCOL_CAPABILITY_MISSION_INT |
            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
            MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
#if AP_TERRAIN_AVAILABLE
            (plane.terrain.enabled() ? MAV_PROTOCOL_CAPABILITY_TERRAIN : 0) |
#endif
            GCS_MAVLINK::capabilities());
}

#if HAL_HIGH_LATENCY2_ENABLED
/**
 * @brief Calculates target altitude for HIGH_LATENCY2 message
 * 
 * @details Computes the desired altitude target for bandwidth-constrained telemetry.
 *          HIGH_LATENCY2 protocol designed for satellite/Iridium links with
 *          severe bandwidth limitations (few bytes per minute).
 *          
 *          Calculation Method:
 *          - Current altitude from AHRS position estimate
 *          - Plus altitude error (desired - actual)
 *          - Yields target altitude vehicle is trying to reach
 *          
 *          Fixed-Wing Mode:
 *          - current_alt + calc_altitude_error_cm()
 *          - Altitude error from navigation controller
 *          - Target from AUTO/GUIDED waypoint or mode-specific target
 *          
 *          QuadPlane VTOL Mode:
 *          - Uses pos_control target if in VTOL mode
 *          - Returns current + position controller U-axis error
 *          - QSTABILIZE exception: Returns 0 (no altitude target)
 *          - show_vtol_view() determines if VTOL display active
 *          
 *          QSTABILIZE Special Case:
 *          - Returns 0 when in QSTABILIZE mode
 *          - No altitude hold in this mode
 *          - Pilot has direct throttle control
 *          - Target altitude not meaningful
 *          
 *          Unit Conversion:
 *          - Internal: Centimeters
 *          - Output: Meters (multiply by 0.01)
 *          - HIGH_LATENCY2 requires meters
 *          
 *          Use Cases:
 *          - Satellite telemetry links (Iridium, RockBLOCK)
 *          - Long-range low-bandwidth scenarios
 *          - Emergency backup telemetry
 *          - Minimal data rate telemetry
 * 
 * @return int16_t Target altitude in meters relative to home
 *         - Range: -32768 to +32767 meters
 *         - 0 if in QSTABILIZE mode (no altitude target)
 *         - Current + error for other modes
 * 
 * @note Conditional compilation: Requires HAL_HIGH_LATENCY2_ENABLED
 * @note QuadPlane behavior differs from fixed-wing
 * @note QSTABILIZE returns 0 (no altitude hold)
 * 
 * @see plane.calc_altitude_error_cm() for fixed-wing altitude error
 * @see quadplane.pos_control->get_pos_error_U_cm() for VTOL altitude error
 */
int16_t GCS_MAVLINK_Plane::high_latency_target_altitude() const
{
    AP_AHRS &ahrs = AP::ahrs();
    Location global_position_current;
    UNUSED_RESULT(ahrs.get_location(global_position_current));

#if HAL_QUADPLANE_ENABLED
    const QuadPlane &quadplane = plane.quadplane;
    //return units are m
    if (quadplane.show_vtol_view()) {
        return (plane.control_mode != &plane.mode_qstabilize) ? 0.01 * (global_position_current.alt + quadplane.pos_control->get_pos_error_U_cm()) : 0;
    }
#endif
    return 0.01 * (global_position_current.alt + plane.calc_altitude_error_cm());
}

/**
 * @brief Calculates target heading for HIGH_LATENCY2 message
 * 
 * @details Computes the desired heading for bandwidth-constrained telemetry.
 *          Returns heading in compressed format (degrees/2) to save bandwidth
 *          in HIGH_LATENCY2 protocol for satellite/Iridium links.
 *          
 *          Fixed-Wing Mode:
 *          - Uses navigation controller target bearing
 *          - Bearing to next waypoint or loiter center
 *          - Wrapped to 0-360 degree range
 *          - Divided by 2 for transmission (uint8: 0-180 = 0-360°)
 *          
 *          QuadPlane VTOL Mode:
 *          - Uses attitude controller yaw target
 *          - Attitude target in centidegrees (Euler Z-axis)
 *          - Converted: centidegrees → degrees → degrees/2
 *          - Body frame yaw target
 *          
 *          Unit Conversions:
 *          Fixed-Wing:
 *          - Input: target_bearing_cd() in centidegrees (-18000 to 18000)
 *          - Wrap: -180° to +180° → 0° to 360°
 *          - Scale: centidegrees / 200 = degrees/2
 *          
 *          QuadPlane:
 *          - Input: targets.z in centidegrees (0 to 36000)
 *          - Convert: × 0.01 = degrees
 *          - Scale: / 2 = degrees/2
 *          
 *          Bandwidth Optimization:
 *          - uint8_t range: 0-255
 *          - Represents: 0-360° in 2° increments
 *          - Sufficient resolution for heading indication
 *          - Halves bandwidth vs full degree resolution
 *          
 *          Navigation Context:
 *          - Fixed-wing: Course to waypoint
 *          - VTOL: Body frame yaw angle
 *          - Different meanings but both valid headings
 *          
 *          Use Cases:
 *          - Satellite telemetry (Iridium, RockBLOCK)
 *          - Emergency low-bandwidth links
 *          - Long-range monitoring
 *          - Backup telemetry path
 * 
 * @return uint8_t Target heading in degrees/2
 *         - Range: 0-180 representing 0-360°
 *         - Resolution: 2° per increment
 *         - Fixed-wing: Course to waypoint
 *         - QuadPlane: Yaw attitude target
 * 
 * @note Conditional compilation: Requires HAL_HIGH_LATENCY2_ENABLED
 * @note Returns degrees/2 for bandwidth efficiency
 * @note Different source for fixed-wing vs QuadPlane VTOL
 * 
 * @see plane.nav_controller->target_bearing_cd() for fixed-wing bearing
 * @see quadplane.attitude_control->get_att_target_euler_cd() for VTOL yaw
 */
uint8_t GCS_MAVLINK_Plane::high_latency_tgt_heading() const
{
    // return units are deg/2
#if HAL_QUADPLANE_ENABLED
    const QuadPlane &quadplane = plane.quadplane;
    if (quadplane.show_vtol_view()) {
        const Vector3f &targets = quadplane.attitude_control->get_att_target_euler_cd();
        return ((uint16_t)(targets.z * 0.01)) / 2;
    }
#endif
        const AP_Navigation *nav_controller = plane.nav_controller;
        // need to convert -18000->18000 to 0->360/2
        return wrap_360_cd(nav_controller->target_bearing_cd() ) / 200;
}

/**
 * @brief Calculates distance to target for HIGH_LATENCY2 message
 * 
 * @details Computes distance to current navigation target in decimeters for
 *          bandwidth-constrained telemetry. HIGH_LATENCY2 uses decimeters to
 *          achieve reasonable range (0-6553.5m) with uint16_t data type.
 *          
 *          Fixed-Wing Mode:
 *          - Distance to current waypoint from auto_state
 *          - Straight-line distance in horizontal plane
 *          - Updated continuously during navigation
 *          
 *          QuadPlane VTOL Mode:
 *          - Uses waypoint navigation distance if active
 *          - wp_nav->get_wp_distance_to_destination_cm()
 *          - Returns 0 if waypoint navigation not active
 *          - Typical for VTOL position control modes
 *          
 *          Unit Conversion:
 *          - Internal: Centimeters (cm)
 *          - Output: Decimeters (dm) - divide by 10
 *          - uint16_t range: 0-65535 dm (0-6553.5 meters)
 *          
 *          Range Limitation:
 *          - Clamped to UINT16_MAX before division
 *          - Prevents overflow on long distances
 *          - Maximum representable: 6553.5 meters
 *          - Beyond max: Saturates at 65535 dm
 *          
 *          Waypoint Navigation Check:
 *          - QuadPlane: using_wp_nav() verifies navigation active
 *          - False if hovering without waypoint
 *          - False if in stabilize/attitude-only modes
 *          - Returns 0 when navigation inactive
 *          
 *          Use Cases:
 *          - Satellite telemetry progress monitoring
 *          - Low-bandwidth distance indication
 *          - Emergency telemetry
 *          - Mission progress tracking on limited links
 *          
 *          Resolution:
 *          - 0.1 meter (1 decimeter) resolution
 *          - Sufficient for navigation monitoring
 *          - Good balance: range vs precision
 * 
 * @return uint16_t Distance to target in decimeters
 *         - Range: 0-65535 dm (0 to 6553.5 meters)
 *         - Resolution: 0.1 meter
 *         - 0 if no active waypoint navigation (QuadPlane)
 *         - Saturates at UINT16_MAX for very long distances
 * 
 * @note Conditional compilation: Requires HAL_HIGH_LATENCY2_ENABLED
 * @note Returns decimeters (dm) not meters or centimeters
 * @note QuadPlane returns 0 if waypoint navigation inactive
 * @note Distance saturates at maximum representable value
 * 
 * @see plane.auto_state.wp_distance for fixed-wing distance
 * @see quadplane.wp_nav->get_wp_distance_to_destination_cm() for VTOL distance
 */
// return units are dm
uint16_t GCS_MAVLINK_Plane::high_latency_tgt_dist() const
{
#if HAL_QUADPLANE_ENABLED
    const QuadPlane &quadplane = plane.quadplane;
    if (quadplane.show_vtol_view()) {
        bool wp_nav_valid = quadplane.using_wp_nav();
        return (wp_nav_valid ? MIN(quadplane.wp_nav->get_wp_distance_to_destination_cm(), UINT16_MAX) : 0) / 10;
    }
    #endif

    return MIN(plane.auto_state.wp_distance, UINT16_MAX) / 10;
}

/**
 * @brief Calculates target airspeed for HIGH_LATENCY2 message
 * 
 * @details Computes desired airspeed in compressed format (m/s × 5) for
 *          bandwidth-constrained telemetry. Compression allows uint8_t to
 *          represent 0-51 m/s with 0.2 m/s resolution.
 *          
 *          Unit Conversion:
 *          - Input: target_airspeed_cm in centimeters/second
 *          - Conversion: cm/s × 0.05 = m/s × 5
 *          - Mathematical: cm/s / 20 = (m/s × 100) / 20 = m/s × 5
 *          
 *          Bandwidth Optimization:
 *          - uint8_t range: 0-255
 *          - Represents: 0-51 m/s (0-183 km/h)
 *          - Resolution: 0.2 m/s per increment
 *          - Sufficient for fixed-wing airspeeds
 *          
 *          Typical Airspeed Ranges:
 *          - Small UAV: 10-25 m/s (36-90 km/h)
 *          - Large UAV: 20-40 m/s (72-144 km/h)
 *          - Maximum representable: 51 m/s (183 km/h)
 *          - Coverage adequate for most ArduPlane vehicles
 *          
 *          Target Source:
 *          - plane.target_airspeed_cm from navigation controller
 *          - Mode-specific airspeed target
 *          - AUTO: Mission waypoint airspeed
 *          - CRUISE/FBWB: Cruise airspeed parameter
 *          - GUIDED: Commanded airspeed
 *          
 *          Use Cases:
 *          - Satellite telemetry monitoring
 *          - Low-bandwidth airspeed indication
 *          - Mission execution monitoring
 *          - Energy management tracking
 *          
 *          HIGH_LATENCY2 Context:
 *          - Designed for Iridium/satellite links
 *          - Few bytes per minute bandwidth
 *          - Every byte counts
 *          - Scaled encoding maximizes information density
 * 
 * @return uint8_t Target airspeed in (m/s × 5)
 *         - Range: 0-255 representing 0-51 m/s
 *         - Resolution: 0.2 m/s
 *         - Multiply by 0.2 to get m/s
 *         - Multiply by 0.72 to get km/h
 * 
 * @note Conditional compilation: Requires HAL_HIGH_LATENCY2_ENABLED
 * @note Returns m/s × 5 not m/s directly
 * @note Saturates at 255 (51 m/s) for very high speeds
 * 
 * @see plane.target_airspeed_cm for airspeed target source
 */
uint8_t GCS_MAVLINK_Plane::high_latency_tgt_airspeed() const
{
    // return units are m/s*5
    return plane.target_airspeed_cm * 0.05;
}

/**
 * @brief Calculates wind speed for HIGH_LATENCY2 message
 * 
 * @details Computes estimated wind speed magnitude in compressed format (m/s × 5)
 *          for bandwidth-constrained telemetry. Uses 3D wind vector from AHRS
 *          and calculates total magnitude including vertical component.
 *          
 *          Wind Estimation Source:
 *          - AP::ahrs().wind_estimate() - EKF-estimated wind vector
 *          - 3D vector: North, East, Down components (NED frame)
 *          - Derived from airspeed vs groundspeed differences
 *          - Continuously updated by navigation filter
 *          - Includes vertical wind (updrafts/downdrafts)
 *          
 *          Magnitude Calculation:
 *          - 3D vector length: sqrt(N² + E² + D²)
 *          - Includes horizontal AND vertical wind
 *          - Total wind magnitude not just horizontal
 *          - Important for sailplanes (thermal detection)
 *          
 *          Unit Conversion:
 *          - Input: m/s (wind estimate in meters/second)
 *          - Conversion: m/s × 5
 *          - Output range: 0-255 representing 0-51 m/s
 *          - Resolution: 0.2 m/s per increment
 *          
 *          Saturation Protection:
 *          - MIN(value, UINT8_MAX) prevents overflow
 *          - Clamps to 255 (51 m/s) maximum
 *          - 51 m/s = 183 km/h = 100 knots = hurricane-force winds
 *          - Saturation rare but prevents wrap-around
 *          
 *          Typical Wind Conditions:
 *          - Calm: 0-5 m/s (0-11 mph)
 *          - Moderate: 5-10 m/s (11-22 mph)
 *          - Strong: 10-20 m/s (22-45 mph)
 *          - Very strong: 20-30 m/s (45-67 mph)
 *          - Maximum representable: 51 m/s (114 mph)
 *          
 *          Wind Estimate Accuracy:
 *          - Requires airspeed sensor for best accuracy
 *          - GPS velocity vs airspeed comparison
 *          - Improves over time as filter converges
 *          - Less accurate without airspeed sensor
 *          - Vertical component from climb rate analysis
 *          
 *          Use Cases:
 *          - Flight performance assessment
 *          - Weather condition monitoring
 *          - Thermal soaring indication (vertical component)
 *          - Return-to-launch feasibility check
 *          - Energy management planning
 *          
 *          HIGH_LATENCY2 Context:
 *          - Critical for understanding flight conditions
 *          - Helps ground station predict behavior
 *          - Important for long-distance operations
 *          - Satellite link constraint: minimal bytes
 * 
 * @return uint8_t Wind speed in (m/s × 5)
 *         - Range: 0-255 representing 0-51 m/s
 *         - Resolution: 0.2 m/s
 *         - Saturates at 255 (51 m/s) for extreme winds
 *         - Includes horizontal and vertical components
 *         - Multiply by 0.2 to get m/s
 * 
 * @note Conditional compilation: Requires HAL_HIGH_LATENCY2_ENABLED
 * @note Returns m/s × 5 not m/s directly
 * @note Includes vertical wind component (3D magnitude)
 * @note Saturates at UINT8_MAX to prevent overflow
 * 
 * @see AP::ahrs().wind_estimate() for wind vector source
 */
uint8_t GCS_MAVLINK_Plane::high_latency_wind_speed() const
{
    Vector3f wind;
    wind = AP::ahrs().wind_estimate();

    // return units are m/s*5
    return MIN(wind.length() * 5, UINT8_MAX);
}

/**
 * @brief Calculates wind direction for HIGH_LATENCY2 message
 * 
 * @details Computes wind direction (where wind is coming FROM) in compressed
 *          format (deg ÷ 2) for bandwidth-constrained telemetry. Uses
 *          horizontal wind vector components and applies meteorological
 *          convention (direction wind is blowing FROM).
 *          
 *          Wind Estimation Source:
 *          - AP::ahrs().wind_estimate() - EKF wind vector
 *          - NED frame: wind.x = North, wind.y = East, wind.z = Down
 *          - Horizontal components only used for direction
 *          - Vertical component ignored for direction calculation
 *          
 *          Meteorological Convention:
 *          - Reports direction wind is COMING FROM
 *          - NOT direction wind is blowing TO
 *          - North wind (0°): Wind from north, blowing south
 *          - East wind (90°): Wind from east, blowing west
 *          - Negative signs in atan2f convert TO-vector to FROM-vector
 *          
 *          Direction Calculation:
 *          - atan2f(-wind.y, -wind.x) computes angle from North
 *          - Negative signs: Reverse vector to get FROM direction
 *          - atan2f returns radians in range [-π, +π]
 *          - degrees() converts to degrees [-180, +180]
 *          - wrap_360() converts to [0, 360) range
 *          - Division by 2 compresses to uint8_t range
 *          
 *          Unit Conversion:
 *          - Internal: Radians from atan2f
 *          - degrees(): Convert to degrees [0, 360)
 *          - Output: degrees ÷ 2
 *          - uint8_t range: 0-255 represents 0-510°
 *          - Effective range: 0-180 (0-360° ÷ 2)
 *          - Resolution: 2° per increment
 *          
 *          Angle Wrapping:
 *          - wrap_360() normalizes to [0, 360)
 *          - Handles atan2f output crossing ±180° boundary
 *          - Ensures positive angle output
 *          - Standard compass convention: 0° = North
 *          
 *          Compass Directions:
 *          - 0° (0): North
 *          - 45° (22): Northeast  
 *          - 90° (45): East
 *          - 135° (67): Southeast
 *          - 180° (90): South
 *          - 225° (112): Southwest
 *          - 270° (135): West
 *          - 315° (157): Northwest
 *          
 *          Resolution Analysis:
 *          - 2° resolution adequate for wind direction
 *          - Wind direction inherently variable
 *          - Higher precision not meaningful
 *          - Good balance: accuracy vs bandwidth
 *          
 *          Use Cases:
 *          - Crosswind assessment for landing planning
 *          - Headwind/tailwind identification
 *          - Weather pattern understanding
 *          - Flight path optimization decisions
 *          - Return-to-launch wind compensation
 *          
 *          Edge Cases:
 *          - Calm winds: Direction undefined but returns value
 *          - Rapidly changing winds: Instantaneous direction
 *          - Pure vertical wind: Returns arbitrary horizontal direction
 * 
 * @return uint8_t Wind direction in (degrees ÷ 2)
 *         - Range: 0-180 representing 0-360°
 *         - Resolution: 2° per increment
 *         - Multiply by 2 to get degrees
 *         - Meteorological convention: Direction FROM
 *         - 0 = North, 45 = East, 90 = South, 135 = West
 * 
 * @note Conditional compilation: Requires HAL_HIGH_LATENCY2_ENABLED
 * @note Returns degrees ÷ 2 not degrees directly
 * @note Meteorological convention: direction wind comes FROM
 * @note Uses horizontal wind components only (ignores vertical)
 * 
 * @see AP::ahrs().wind_estimate() for wind vector source
 */
uint8_t GCS_MAVLINK_Plane::high_latency_wind_direction() const
{
    const Vector3f wind = AP::ahrs().wind_estimate();

    // return units are deg/2
    // need to convert -180->180 to 0->360/2
    return wrap_360(degrees(atan2f(-wind.y, -wind.x))) / 2;
}
#endif // HAL_HIGH_LATENCY2_ENABLED

/**
 * @brief Reports current VTOL transition state for MAVLink
 * 
 * @details Returns the current VTOL (Vertical Takeoff and Landing) state for
 *          QuadPlane hybrid aircraft. This indicates whether the vehicle is
 *          operating in fixed-wing mode, multicopter mode, or transitioning
 *          between the two.
 *          
 *          MAV_VTOL_STATE Values:
 *          - MAV_VTOL_STATE_UNDEFINED: Not a VTOL aircraft or not available
 *          - MAV_VTOL_STATE_TRANSITION_TO_FW: Transitioning to fixed-wing
 *          - MAV_VTOL_STATE_TRANSITION_TO_MC: Transitioning to multicopter
 *          - MAV_VTOL_STATE_MC: Multicopter mode (VTOL motors active)
 *          - MAV_VTOL_STATE_FW: Fixed-wing mode (forward flight)
 *          
 *          QuadPlane Availability Check:
 *          - quadplane.available() verifies QuadPlane configured
 *          - Returns UNDEFINED if pure fixed-wing (no VTOL)
 *          - Returns UNDEFINED if QuadPlane disabled
 *          - Must have Q_ENABLE parameter set
 *          
 *          Transition State Management:
 *          - transition->get_mav_vtol_state() queries transition controller
 *          - Reflects current flight dynamics state
 *          - Updated continuously during flight
 *          - Critical for ground station awareness
 *          
 *          Pure Fixed-Wing Configuration:
 *          - HAL_QUADPLANE_ENABLED = 0: No VTOL capability
 *          - Always returns MAV_VTOL_STATE_UNDEFINED
 *          - Compile-time optimization for non-VTOL builds
 *          - Reduces code size and memory usage
 *          
 *          Ground Station Display:
 *          - GCS shows current flight mode type
 *          - Indicates transition progress
 *          - Important for pilot situational awareness
 *          - Critical during manual transitions
 *          
 *          Transition Scenarios:
 *          - Takeoff: MC → FW transition after altitude gained
 *          - Landing: FW → MC transition for vertical landing
 *          - Manual: Pilot commands transition via RC or GCS
 *          - Automatic: Mode-triggered transitions (AUTO, RTL, etc.)
 *          
 *          State Timing:
 *          - Transitions can take several seconds
 *          - State changes when transition logic determines ready
 *          - Airspeed, altitude, and stability criteria must be met
 *          - Ground station tracks transition progress
 *          
 *          Use Cases:
 *          - Ground station flight mode indication
 *          - Automated procedure verification
 *          - Telemetry log analysis
 *          - Transition behavior debugging
 *          - Pilot awareness during manual flight
 *          
 *          Safety Considerations:
 *          - Critical for understanding control authority
 *          - Different control modes in different states
 *          - Pilot input interpretation varies by state
 *          - Stall protection differs by state
 * 
 * @return MAV_VTOL_STATE Current VTOL transition state
 *         - MAV_VTOL_STATE_UNDEFINED: Pure fixed-wing or disabled QuadPlane
 *         - MAV_VTOL_STATE_MC: Multicopter mode active
 *         - MAV_VTOL_STATE_FW: Fixed-wing mode active
 *         - MAV_VTOL_STATE_TRANSITION_TO_FW: Transitioning to forward flight
 *         - MAV_VTOL_STATE_TRANSITION_TO_MC: Transitioning to VTOL flight
 * 
 * @note Conditional compilation: HAL_QUADPLANE_ENABLED determines availability
 * @note Returns UNDEFINED if QuadPlane not available or configured
 * @note State updated continuously by transition controller
 * 
 * @see plane.quadplane.transition->get_mav_vtol_state() for state source
 * @see plane.quadplane.available() for QuadPlane availability check
 */
MAV_VTOL_STATE GCS_MAVLINK_Plane::vtol_state() const
{
#if !HAL_QUADPLANE_ENABLED
    return MAV_VTOL_STATE_UNDEFINED;
#else
    if (!plane.quadplane.available()) {
        return MAV_VTOL_STATE_UNDEFINED;
    }

    return plane.quadplane.transition->get_mav_vtol_state();
#endif
};

/**
 * @brief Reports current landed state for MAVLink EXTENDED_SYS_STATE
 * 
 * @details Determines whether the aircraft is on the ground, in flight,
 *          taking off, or landing. This information is critical for ground
 *          control stations, automated systems, and logging.
 *          
 *          MAV_LANDED_STATE Values:
 *          - MAV_LANDED_STATE_ON_GROUND: Vehicle on ground, not flying
 *          - MAV_LANDED_STATE_IN_AIR: Normal flight
 *          - MAV_LANDED_STATE_TAKEOFF: Active takeoff sequence
 *          - MAV_LANDED_STATE_LANDING: Active landing sequence
 *          
 *          State Determination Logic:
 *          1. Check is_flying() first - primary flight state
 *          2. If flying, check if taking off - priority state
 *          3. If flying, check if landing - priority state
 *          4. If flying but not takeoff/landing - normal flight
 *          5. If not flying - on ground
 *          
 *          Flight Detection (is_flying()):
 *          - Airspeed above threshold (fixed-wing)
 *          - Altitude change rate (vertical velocity)
 *          - Time since motor commands (recent throttle)
 *          - QuadPlane: Almost always reports flying in Q-modes
 *          - Ground effect considerations
 *          
 *          Takeoff Detection (is_taking_off()):
 *          - TAKEOFF flight mode active
 *          - AUTO mode with takeoff waypoint in progress
 *          - Altitude below takeoff complete threshold
 *          - Positive climb rate
 *          - Mode-specific takeoff logic
 *          
 *          Landing Detection (is_landing()):
 *          - LAND mode active (QuadPlane)
 *          - AUTO mode with land waypoint in progress
 *          - Final approach phase of landing sequence
 *          - Below landing approach altitude
 *          - Flare active (fixed-wing)
 *          
 *          QuadPlane Considerations:
 *          - "Q-modes almost always consider themselves as flying"
 *          - VTOL motors running = flying
 *          - Hover on ground may still report IN_AIR
 *          - Conservative for safety (assume flying unless certain)
 *          - Ground detection more reliable after disarm
 *          
 *          Ground State:
 *          - !is_flying() returns true
 *          - Motors off or very low throttle
 *          - Minimal altitude change
 *          - Airspeed below flight threshold
 *          - Settled on ground for duration
 *          
 *          Use Cases:
 *          - Ground station flight phase indication
 *          - Automated mission sequencing
 *          - Data logging and analysis
 *          - Flight time tracking
 *          - Safety system state awareness
 *          - Geofence behavior (e.g., disable on ground)
 *          
 *          State Transitions:
 *          - ON_GROUND → TAKEOFF: Takeoff mode entered, throttle up
 *          - TAKEOFF → IN_AIR: Altitude threshold reached
 *          - IN_AIR → LANDING: Landing sequence initiated
 *          - LANDING → ON_GROUND: Touchdown detected
 *          
 *          Safety Implications:
 *          - Determines when to apply ground-only restrictions
 *          - Affects arming/disarming logic
 *          - Influences failsafe behavior
 *          - Controls throttle safety limits
 *          
 *          Logging and Analysis:
 *          - Flight time calculation
 *          - Takeoff/landing statistics
 *          - Phase-specific parameter analysis
 *          - Incident investigation
 * 
 * @return MAV_LANDED_STATE Current landed state
 *         - MAV_LANDED_STATE_ON_GROUND: Vehicle stationary on ground
 *         - MAV_LANDED_STATE_TAKEOFF: Active takeoff phase
 *         - MAV_LANDED_STATE_LANDING: Active landing phase
 *         - MAV_LANDED_STATE_IN_AIR: Normal flight (not takeoff/landing)
 * 
 * @note State based on is_flying(), is_taking_off(), is_landing()
 * @note QuadPlane Q-modes almost always report flying
 * @note Takeoff and landing states take priority over generic IN_AIR
 * 
 * @see plane.is_flying() for flight detection logic
 * @see plane.is_taking_off() for takeoff detection
 * @see plane.is_landing() for landing detection
 */
MAV_LANDED_STATE GCS_MAVLINK_Plane::landed_state() const
{
    if (plane.is_flying()) {
        if (plane.is_taking_off()) {
            return MAV_LANDED_STATE_TAKEOFF;
        }
        if (plane.is_landing()) {
            return MAV_LANDED_STATE_LANDING;
        }

        // note that Q-modes almost always consider themselves as flying
        return MAV_LANDED_STATE_IN_AIR;
    }

    return MAV_LANDED_STATE_ON_GROUND;
}

/**
 * @brief Sends available flight mode information via AVAILABLE_MODES message
 * 
 * @details Transmits details about a specific flight mode by index to enable
 *          ground control stations to enumerate all available modes. This
 *          supports dynamic mode discovery, allowing GCS to present only
 *          modes available on the current vehicle configuration.
 *          
 *          AVAILABLE_MODES Protocol:
 *          - MAVLink message ID 435 (AVAILABLE_MODES)
 *          - GCS requests mode list during connection
 *          - Autopilot responds with mode details one by one
 *          - Index parameter selects which mode to send
 *          - Returns total mode count for enumeration
 *          
 *          Index Convention:
 *          - Input index starts at 1 (not zero-based)
 *          - Internally converted to zero-based for array access
 *          - Index 1 = first mode in list
 *          - Index N = Nth mode in list
 *          - Returning mode_count enables GCS to iterate
 *          
 *          Mode Categories:
 *          1. Fixed-Wing Modes (always present):
 *             - MANUAL, CIRCLE, STABILIZE, TRAINING, ACRO
 *             - FLY_BY_WIRE_A, FLY_BY_WIRE_B, CRUISE, AUTOTUNE
 *             - AUTO, RTL, LOITER, GUIDED, INITIALIZING, TAKEOFF
 *             - AVOID_ADSB (if HAL_ADSB_ENABLED)
 *             - THERMAL (if HAL_SOARING_ENABLED)
 *             - AUTOLAND (if MODE_AUTOLAND_ENABLED)
 *          
 *          2. QuadPlane Modes (if quadplane.available()):
 *             - QSTABILIZE, QHOVER, QLOITER, QLAND, QRTL, QACRO
 *             - LOITER_ALT_QLAND
 *             - QAUTOTUNE (if QAUTOTUNE_ENABLED)
 *          
 *          Mode Array Organization:
 *          - fw_modes[] contains fixed-wing mode pointers
 *          - q_modes[] contains QuadPlane mode pointers
 *          - Fixed-wing modes listed first (indices 0 to fw_mode_count-1)
 *          - QuadPlane modes follow (indices fw_mode_count to mode_count-1)
 *          
 *          Mode Information Retrieved:
 *          - name: Human-readable mode name string
 *          - mode_number: Internal mode enumeration value
 *          - Both obtained via Mode class methods
 *          
 *          Message Fields Sent:
 *          - mode_count: Total number of available modes
 *          - mode_index: Current mode being reported (1-based)
 *          - standard_mode: MAV_STANDARD_MODE_NON_STANDARD (ArduPilot custom)
 *          - custom_mode: ArduPilot mode number
 *          - properties: 0 (no special properties bitmask)
 *          - mode_name: Human-readable name string
 *          
 *          Conditional Mode Inclusion:
 *          - QuadPlane modes only if plane.quadplane.available()
 *          - AVOID_ADSB only if HAL_ADSB_ENABLED
 *          - THERMAL only if HAL_SOARING_ENABLED
 *          - AUTOLAND only if MODE_AUTOLAND_ENABLED
 *          - QAUTOTUNE only if QAUTOTUNE_ENABLED
 *          
 *          Ground Station Usage:
 *          - GCS requests mode list on connection
 *          - Iterates from index 1 to mode_count
 *          - Builds dynamic mode selection UI
 *          - Only shows modes available for this vehicle
 *          - Avoids showing disabled/unavailable modes
 *          
 *          Example Enumeration:
 *          - GCS calls with index=1, receives MANUAL (returns total=18)
 *          - GCS calls with index=2, receives CIRCLE (returns total=18)
 *          - ... continues until index=18
 *          - GCS now has complete mode list
 *          
 *          Error Handling:
 *          - If index > mode_count, returns mode_count without sending
 *          - Should not happen with correct GCS implementation
 *          - Prevents array out-of-bounds access
 *          
 *          Use Cases:
 *          - Dynamic mode menu generation in GCS
 *          - Vehicle capability discovery
 *          - User interface adaptation to hardware
 *          - Mode availability logging
 *          - Configuration verification
 * 
 * @param[in] index Mode index to send (1-based, not zero-based)
 *                  - Range: 1 to mode_count
 *                  - Index 1 = first mode
 *                  - Out of range returns mode_count without sending
 * 
 * @return uint8_t Total number of available modes
 *         - Includes fixed-wing modes (always present)
 *         - Includes QuadPlane modes (if available)
 *         - Same value returned regardless of index
 *         - GCS uses to determine iteration count
 * 
 * @note Index parameter is 1-based not 0-based
 * @note Returns total mode count for all indices
 * @note QuadPlane modes only included if quadplane.available()
 * @note Sends AVAILABLE_MODES MAVLink message on chan
 * 
 * @see Mode::name() for mode name retrieval
 * @see Mode::mode_number() for mode number retrieval
 * @see plane.quadplane.available() for QuadPlane mode inclusion
 */
// Send the mode with the given index (not mode number!) return the total number of modes
// Index starts at 1
uint8_t GCS_MAVLINK_Plane::send_available_mode(uint8_t index) const
{
    // Fixed wing modes
    const Mode* fw_modes[] {
        &plane.mode_manual,
        &plane.mode_circle,
        &plane.mode_stabilize,
        &plane.mode_training,
        &plane.mode_acro,
        &plane.mode_fbwa,
        &plane.mode_fbwb,
        &plane.mode_cruise,
        &plane.mode_autotune,
        &plane.mode_auto,
        &plane.mode_rtl,
        &plane.mode_loiter,
#if HAL_ADSB_ENABLED
        &plane.mode_avoidADSB,
#endif
        &plane.mode_guided,
        &plane.mode_initializing,
        &plane.mode_takeoff,
#if HAL_SOARING_ENABLED
        &plane.mode_thermal,
#endif
#if MODE_AUTOLAND_ENABLED
        &plane.mode_autoland,
#endif
    };

    const uint8_t fw_mode_count = ARRAY_SIZE(fw_modes);

    // Fixedwing modes are always present
    uint8_t mode_count = fw_mode_count;

#if HAL_QUADPLANE_ENABLED
    // Quadplane modes
    const Mode* q_modes[] {
        &plane.mode_qstabilize,
        &plane.mode_qhover,
        &plane.mode_qloiter,
        &plane.mode_qland,
        &plane.mode_qrtl,
        &plane.mode_qacro,
        &plane.mode_loiter_qland,
#if QAUTOTUNE_ENABLED
        &plane.mode_qautotune,
#endif
    };

    // Quadplane modes must be enabled
    if (plane.quadplane.available()) {
        mode_count += ARRAY_SIZE(q_modes);
    }
#endif // HAL_QUADPLANE_ENABLED


    // Convert to zero indexed
    const uint8_t index_zero = index - 1;
    if (index_zero >= mode_count) {
        // Mode does not exist!?
        return mode_count;
    }

    // Ask the mode for its name and number
    const char* name;
    uint8_t mode_number;

    if (index_zero < fw_mode_count) {
        // A fixedwing mode
        name = fw_modes[index_zero]->name();
        mode_number = (uint8_t)fw_modes[index_zero]->mode_number();

    } else {
#if HAL_QUADPLANE_ENABLED
        // A Quadplane mode
        const uint8_t q_index = index_zero - fw_mode_count;
        name = q_modes[q_index]->name();
        mode_number = (uint8_t)q_modes[q_index]->mode_number();
#else
        // Should not endup here
        return mode_count;
#endif
    }

    mavlink_msg_available_modes_send(
        chan,
        mode_count,
        index,
        MAV_STANDARD_MODE::MAV_STANDARD_MODE_NON_STANDARD,
        mode_number,
        0, // MAV_MODE_PROPERTY bitmask
        name
    );

    return mode_count;
}
