/**
 * @file GCS_MAVLink_Sub.cpp
 * @brief ArduSub-specific MAVLink protocol implementation
 * 
 * @details This file implements the MAVLink Ground Control Station (GCS) communication
 *          protocol for ArduSub underwater vehicles. It handles bidirectional communication
 *          between the submarine and ground control stations, including:
 *          - Telemetry streaming (position, attitude, depth, sensors)
 *          - Command handling (guided mode, attitude control, mission commands)
 *          - Parameter and mission protocols
 *          - Submarine-specific messages (depth/pressure, lights, camera control)
 *          - Manual control and RC override for joystick input
 *          
 *          Key ArduSub-specific features:
 *          - MAV_TYPE_SUBMARINE identification for GCS compatibility
 *          - SCALED_PRESSURE3 message repurposed for temperature telemetry
 *          - Depth control via SET_POSITION_TARGET messages
 *          - Manual control message handling for joystick operations
 *          - Leak detector integration via SYS_STATUS messages
 *          - Named value floats for lights, camera tilt/pan, tether turns
 *          
 *          Message Flow:
 *          - Outgoing: Telemetry streamed at configurable rates via try_send_message()
 *          - Incoming: Commands processed in handle_message() and command handlers
 *          - Position control: SET_POSITION_TARGET_LOCAL_NED and GLOBAL_INT
 *          - Attitude control: SET_ATTITUDE_TARGET with thrust-to-climb-rate conversion
 *          
 * @note This implementation extends GCS_MAVLINK base class with submarine-specific handlers
 * @warning All position coordinates use NED (North-East-Down) frame convention
 * @warning Depth is represented as negative Z in NED frame (down is positive depth)
 * 
 * @see GCS_MAVLINK base class in libraries/GCS_MAVLink/
 * @see MAVLink protocol documentation at https://mavlink.io/
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:1-830
 */

#include "Sub.h"

#include "GCS_MAVLink_Sub.h"
#include <AP_RPM/AP_RPM_config.h>

/**
 * @brief Returns the MAVLink frame type for submarine vehicles
 * 
 * @details Identifies this vehicle as MAV_TYPE_SUBMARINE (28) to ground control stations.
 *          This allows GCS software to provide submarine-specific UI elements and
 *          control modes appropriate for underwater operation.
 * 
 * @return MAV_TYPE_SUBMARINE (MAVLink vehicle type 28)
 * 
 * @note This is called during heartbeat message assembly
 * @see MAV_TYPE enum in MAVLink common message set
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:6-9
 */
MAV_TYPE GCS_Sub::frame_type() const
{
    return MAV_TYPE_SUBMARINE;
}

/**
 * @brief Calculates the MAVLink base_mode bitmask for current vehicle state
 * 
 * @details Constructs the base_mode field for HEARTBEAT messages by setting appropriate
 *          MAV_MODE_FLAG bits based on current flight mode, arming state, and capabilities.
 *          The base_mode provides generic information that any MAVLink GCS can interpret,
 *          though the custom_mode field provides more precise ArduSub mode information.
 *          
 *          Flags set:
 *          - MAV_MODE_FLAG_STABILIZE_ENABLED: Always set (all Sub modes provide stabilization)
 *          - MAV_MODE_FLAG_GUIDED_ENABLED: Set for AUTO, GUIDED, CIRCLE, POSHOLD modes
 *          - MAV_MODE_FLAG_MANUAL_INPUT_ENABLED: Always set (pilot can override)
 *          - MAV_MODE_FLAG_SAFETY_ARMED: Set when motors are armed
 *          - MAV_MODE_FLAG_CUSTOM_MODE_ENABLED: Always set (custom_mode field is valid)
 * 
 * @return uint8_t Bitmask of MAV_MODE_FLAG values indicating current vehicle mode
 * 
 * @note This value is less useful than custom_mode for ArduPilot vehicles
 * @note Called during HEARTBEAT message assembly (typically at 1Hz)
 * @warning Do not use MAV_MODE_FLAG_AUTO_ENABLED as ArduPilot definition differs from MAVLink
 * 
 * @see custom_mode() for ArduSub-specific mode number
 * @see MAV_MODE_FLAG enum in MAVLink common.xml
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:11-49
 */
uint8_t GCS_MAVLINK_Sub::base_mode() const
{
    uint8_t _base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;

    // work out the base_mode. This value is not very useful
    // for APM, but we calculate it as best we can so a generic
    // MAVLink enabled ground station can work out something about
    // what the MAV is up to. The actual bit values are highly
    // ambiguous for most of the APM flight modes. In practice, you
    // only get useful information from the custom_mode, which maps to
    // the APM flight mode and has a well defined meaning in the
    // ArduPlane documentation
    switch (sub.control_mode) {
    case Mode::Number::AUTO:
    case Mode::Number::GUIDED:
    case Mode::Number::CIRCLE:
    case Mode::Number::POSHOLD:
        _base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
        break;
    default:
        break;
    }

    // all modes except INITIALISING have some form of manual
    // override if stick mixing is enabled
    _base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

    if (sub.motors.armed()) {
        _base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // indicate we have set a custom mode
    _base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    return _base_mode;
}

/**
 * @brief Returns the ArduSub-specific custom mode number
 * 
 * @details Provides the current flight mode as a uint32_t for inclusion in HEARTBEAT messages.
 *          The custom_mode field allows GCS software to precisely identify the active ArduSub
 *          mode (MANUAL, STABILIZE, ACRO, ALT_HOLD, AUTO, GUIDED, CIRCLE, SURFACE, POSHOLD, etc.).
 *          Ground stations use this value to display the current mode and enable appropriate controls.
 * 
 * @return uint32_t Current Mode::Number cast to uint32_t (0-10 for ArduSub modes)
 * 
 * @note This is the primary mode indicator for ArduSub (more precise than base_mode)
 * @note Called during HEARTBEAT message assembly (typically at 1Hz)
 * 
 * @see Mode::Number enum for ArduSub mode definitions
 * @see base_mode() for generic MAVLink mode flags
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:51-54
 */
uint32_t GCS_Sub::custom_mode() const
{
    return (uint32_t)sub.control_mode;
}

/**
 * @brief Determines the current MAVLink system status state
 * 
 * @details Returns the appropriate MAV_STATE value for HEARTBEAT and SYS_STATUS messages
 *          based on vehicle initialization, arming state, and failsafe conditions.
 *          GCS uses this to display vehicle health and operational status.
 *          
 *          State Priority (highest to lowest):
 *          1. MAV_STATE_CRITICAL: Any failsafe triggered (leak, pilot timeout, battery, etc.)
 *          2. MAV_STATE_ACTIVE: Motors armed and vehicle operational
 *          3. MAV_STATE_BOOT: Vehicle still initializing (sensors, EKF not ready)
 *          4. MAV_STATE_STANDBY: Disarmed and ready for operation
 * 
 * @return MAV_STATE Current system state (BOOT, STANDBY, ACTIVE, or CRITICAL)
 * 
 * @note Called during HEARTBEAT and SYS_STATUS message assembly
 * @warning MAV_STATE_CRITICAL indicates emergency condition requiring immediate attention
 * 
 * @see sub.any_failsafe_triggered() for failsafe detection
 * @see MAV_STATE enum in MAVLink common.xml
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:56-71
 */
MAV_STATE GCS_MAVLINK_Sub::vehicle_system_status() const
{
    // set system as critical if any failsafe have triggered
    if (sub.any_failsafe_triggered())  {
        return MAV_STATE_CRITICAL;
    }

    if (sub.motors.armed()) {
        return MAV_STATE_ACTIVE;
    }
    if (!sub.ap.initialised) {
    	return MAV_STATE_BOOT;
    }

    return MAV_STATE_STANDBY;
}

/**
 * @brief Sends the startup banner with vehicle information
 * 
 * @details Transmits a text message to the GCS during connection initialization containing
 *          the vehicle frame configuration. Called by base class send_banner() and adds
 *          submarine-specific frame information (e.g., "Frame: BlueROV2").
 * 
 * @note Called once during GCS connection establishment
 * @see sub.motors.get_frame_string() for frame type description
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:73-77
 */
void GCS_MAVLINK_Sub::send_banner()
{
    GCS_MAVLINK::send_banner();
    send_text(MAV_SEVERITY_INFO, "Frame: %s", sub.motors.get_frame_string());
}

/**
 * @brief Assembles and sends NAV_CONTROLLER_OUTPUT MAVLink message
 * 
 * @details Transmits navigation controller status including attitude targets, waypoint bearing,
 *          distance to destination, and position error. This message provides GCS with real-time
 *          information about autonomous navigation progress and control loop targets.
 *          
 *          Message fields populated:
 *          - nav_roll: Target roll angle in degrees (from attitude controller)
 *          - nav_pitch: Target pitch angle in degrees (from attitude controller)
 *          - nav_bearing: Target yaw angle in degrees (from attitude controller)
 *          - target_bearing: Bearing to next waypoint in degrees
 *          - wp_dist: Distance to destination in meters (clamped to UINT16_MAX)
 *          - alt_error: Vertical position error in meters (Up component)
 *          - aspd_error: 0 (not applicable for submarines)
 *          - xtrack_error: 0 (not implemented for ArduSub)
 * 
 * @note Called periodically based on GCS stream rate configuration (typically 2-10Hz)
 * @note Angles converted from centidegrees to degrees (multiply by 1.0e-2)
 * @note Distance converted from centimeters to meters (multiply by 1.0e-2)
 * @warning Distance clamped to UINT16_MAX (65,535m) to prevent overflow
 * 
 * @see mavlink_msg_nav_controller_output_send() in MAVLink library
 * @see AC_AttitudeControl::get_att_target_euler_cd() for attitude targets
 * @see AC_WPNav::get_wp_bearing_to_destination_cd() for bearing calculation
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:79-92
 */
void GCS_MAVLINK_Sub::send_nav_controller_output() const
{
    const Vector3f &targets = sub.attitude_control.get_att_target_euler_cd();
    mavlink_msg_nav_controller_output_send(
        chan,
        targets.x * 1.0e-2f,
        targets.y * 1.0e-2f,
        targets.z * 1.0e-2f,
        sub.wp_nav.get_wp_bearing_to_destination_cd() * 1.0e-2f,
        MIN(sub.wp_nav.get_wp_distance_to_destination_cm() * 1.0e-2f, UINT16_MAX),
        sub.pos_control.get_pos_error_U_cm() * 1.0e-2f,
        0,
        0);
}

/**
 * @brief Provides throttle percentage for VFR_HUD message
 * 
 * @details Returns the current motor throttle output as a percentage (0-100) for display
 *          in the GCS HUD (Heads-Up Display). The throttle value represents the collective
 *          thrust level across all thrusters.
 * 
 * @return int16_t Throttle percentage (0-100, where 50 is neutral for most configurations)
 * 
 * @note Called during VFR_HUD message assembly (typically 4-10Hz)
 * @note For submarines, "throttle" represents collective vertical thrust
 * @see sub.motors.get_throttle() returns normalized value 0.0-1.0
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:94-97
 */
int16_t GCS_MAVLINK_Sub::vfr_hud_throttle() const
{
    return (int16_t)(sub.motors.get_throttle() * 100);
}

/**
 * @brief Provides altitude for VFR_HUD message
 * 
 * @details Returns the current altitude in meters MSL (Mean Sea Level) for display in
 *          the GCS HUD. For submarines, this represents depth below the surface when
 *          negative, or height above sea level when positive.
 * 
 * @return float Altitude in meters MSL (negative values indicate depth below surface)
 * 
 * @note Called during VFR_HUD message assembly (typically 4-10Hz)
 * @note For underwater operations, this is typically negative
 * @warning Coordinate convention: Down is negative in ArduSub (NED frame)
 * @see sub.get_alt_msl() for altitude calculation from barometer and EKF
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:99-102
 */
float GCS_MAVLINK_Sub::vfr_hud_alt() const
{
    return sub.get_alt_msl();
}

/**
 * @brief Sends temperature sensor data via SCALED_PRESSURE3 message
 * 
 * @details Repurposes the SCALED_PRESSURE3 MAVLink message to transmit external temperature
 *          sensor data to the GCS. This is a workaround to send temperature readings when
 *          AP_TEMPERATURE_SENSOR is enabled, as there is no dedicated temperature telemetry
 *          message in the standard MAVLink set.
 *          
 *          Message field mapping:
 *          - time_boot_ms: System uptime in milliseconds
 *          - press_abs: 0 (not used)
 *          - press_diff: 0 (not used)  
 *          - temperature: Temperature in centidegrees Celsius (value * 100)
 *          - temperature_press_diff: 0 (TODO: differential pressure temperature)
 * 
 * @note Only compiled when AP_TEMPERATURE_SENSOR_ENABLED is defined
 * @note Returns early if no valid temperature reading is available
 * @note Temperature scaled by 100 to preserve precision in integer format
 * @warning This is a non-standard use of SCALED_PRESSURE3 message
 * 
 * @see AP_TemperatureSensor::get_temperature() for sensor data retrieval
 * @see mavlink_msg_scaled_pressure3_send() in MAVLink library
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:104-120
 */
// Work around to get temperature sensor data out
void GCS_MAVLINK_Sub::send_scaled_pressure3()
{
#if AP_TEMPERATURE_SENSOR_ENABLED
    float temperature;
    if (!sub.temperature_sensor.get_temperature(temperature)) {
        return;
    }
    mavlink_msg_scaled_pressure3_send(
        chan,
        AP_HAL::millis(),
        0,
        0,
        temperature * 100,
        0); // TODO: use differential pressure temperature
#endif
}

/**
 * @brief Sends ArduSub-specific telemetry via NAMED_VALUE_FLOAT messages
 * 
 * @details Transmits submarine-specific status information as named floating-point values
 *          for display in GCS. Each value is sent as a separate NAMED_VALUE_FLOAT message
 *          with a 10-character name identifier. This provides telemetry for peripherals
 *          and features unique to underwater vehicles.
 *          
 *          Telemetry values transmitted:
 *          - CamTilt: Camera tilt servo position (0.0-1.0, normalized from -1 to +1)
 *          - CamPan: Camera pan servo position (0.0-1.0, normalized from -1 to +1)
 *          - TetherTrn: Tether turn count in quarter rotations (for tether management)
 *          - Lights1: Primary lights level (0.0-1.0, normalized from -1 to +1)
 *          - Lights2: Secondary lights level (0.0-1.0, from RC input channel 10)
 *          - PilotGain: Pilot gain multiplier for input scaling
 *          - InputHold: Input hold engaged flag (0 or 1)
 *          - RollPitch: Roll/pitch input mode flag
 *          - RFTarget: Rangefinder target distance in meters for bottom tracking
 * 
 * @return bool True if all messages sent successfully
 * 
 * @note Called when MSG_NAMED_FLOAT is requested by stream rate configuration
 * @note Each message checks payload size before sending to prevent buffer overflow
 * @note Servo outputs normalized from [-1,1] to [0,1] for compatibility
 * @warning Name limited to 10 characters by NAMED_VALUE_FLOAT message definition
 * 
 * @see CHECK_PAYLOAD_SIZE macro for buffer space verification
 * @see SRV_Channels::get_output_norm() for servo position retrieval
 * @see mavlink_msg_named_value_float_send() in MAVLink library
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:122-159
 */
bool GCS_MAVLINK_Sub::send_info()
{
    // Just do this all at once, hopefully the hard-wire telemetry requirement means this is ok
    // Name is char[10]
    CHECK_PAYLOAD_SIZE(NAMED_VALUE_FLOAT);
    send_named_float("CamTilt",
                     1 - (SRV_Channels::get_output_norm(SRV_Channel::k_mount_tilt) / 2.0f + 0.5f));

    CHECK_PAYLOAD_SIZE(NAMED_VALUE_FLOAT);
    send_named_float("CamPan",
                     1 - (SRV_Channels::get_output_norm(SRV_Channel::k_mount_pan) / 2.0f + 0.5f));

    CHECK_PAYLOAD_SIZE(NAMED_VALUE_FLOAT);
    send_named_float("TetherTrn",
                     sub.quarter_turn_count/4);

    CHECK_PAYLOAD_SIZE(NAMED_VALUE_FLOAT);
    send_named_float("Lights1",
                     SRV_Channels::get_output_norm(SRV_Channel::k_lights1) / 2.0f + 0.5f);

    CHECK_PAYLOAD_SIZE(NAMED_VALUE_FLOAT);
    send_named_float("Lights2",
                     SRV_Channels::get_output_norm(SRV_Channel::k_rcin10) / 2.0f + 0.5f);

    CHECK_PAYLOAD_SIZE(NAMED_VALUE_FLOAT);
    send_named_float("PilotGain", sub.gain);

    CHECK_PAYLOAD_SIZE(NAMED_VALUE_FLOAT);
    send_named_float("InputHold", sub.input_hold_engaged);

    CHECK_PAYLOAD_SIZE(NAMED_VALUE_FLOAT);
    send_named_float("RollPitch", sub.roll_pitch_flag);

    CHECK_PAYLOAD_SIZE(NAMED_VALUE_FLOAT);
    send_named_float("RFTarget", sub.mode_surftrak.get_rangefinder_target_cm() * 0.01f);

    return true;
}

/**
 * @brief Sends PID controller tuning data via PID_TUNING messages
 * 
 * @details Transmits real-time PID controller performance data for in-flight tuning and
 *          analysis. Sends separate PID_TUNING messages for roll, pitch, yaw rate controllers
 *          and vertical position controller based on the GCS_PID_MASK parameter bitmask.
 *          This allows developers and users to visualize PID performance and tune gains.
 *          
 *          Controllers reported (when enabled by gcs_pid_mask):
 *          - Bit 0 (mask & 1): Roll rate controller (PID_TUNING_ROLL)
 *          - Bit 1 (mask & 2): Pitch rate controller (PID_TUNING_PITCH)
 *          - Bit 2 (mask & 4): Yaw rate controller (PID_TUNING_YAW)
 *          - Bit 3 (mask & 8): Vertical acceleration controller (PID_TUNING_ACCZ)
 *          
 *          PID_TUNING message fields:
 *          - axis: Controller axis identifier (ROLL/PITCH/YAW/ACCZ)
 *          - desired: Target value (deg/s for rates, m/s² for accel)
 *          - achieved: Actual measured value (deg/s for rates, m/s² for accel)
 *          - FF: Feedforward component
 *          - P: Proportional component
 *          - I: Integral component
 *          - D: Derivative component
 *          - slew_rate: Rate of change limit
 *          - Dmod: D-term modifier
 * 
 * @note Called when PID tuning telemetry is enabled via parameter
 * @note Values converted from centidegrees to degrees (multiply by 0.01)
 * @note Returns early if payload space exhausted to prevent buffer overflow
 * @note Typically streamed at 10-50Hz for real-time tuning visualization
 * 
 * @see Parameters::gcs_pid_mask for bitmask configuration
 * @see AC_AttitudeControl_Sub::get_rate_*_pid() for rate controller access
 * @see AC_PosControl::get_accel_U_pid() for vertical position controller
 * @see HAVE_PAYLOAD_SPACE macro for buffer management
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:161-231
 */
void GCS_MAVLINK_Sub::send_pid_tuning()
{
    const Parameters &g = sub.g;
    AP_AHRS &ahrs = AP::ahrs();
    AC_AttitudeControl_Sub &attitude_control = sub.attitude_control;

    const Vector3f &gyro = ahrs.get_gyro();
    if (g.gcs_pid_mask & 1) {
        const AP_PIDInfo &pid_info = attitude_control.get_rate_roll_pid().get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_ROLL,
                                    pid_info.target*0.01f,
                                    degrees(gyro.x),
                                    pid_info.FF*0.01f,
                                    pid_info.P*0.01f,
                                    pid_info.I*0.01f,
                                    pid_info.D*0.01f,
                                    pid_info.slew_rate,
                                    pid_info.Dmod);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
    if (g.gcs_pid_mask & 2) {
        const AP_PIDInfo &pid_info = attitude_control.get_rate_pitch_pid().get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_PITCH,
                                    pid_info.target*0.01f,
                                    degrees(gyro.y),
                                    pid_info.FF*0.01f,
                                    pid_info.P*0.01f,
                                    pid_info.I*0.01f,
                                    pid_info.D*0.01f,
                                    pid_info.slew_rate,
                                    pid_info.Dmod);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
    if (g.gcs_pid_mask & 4) {
        const AP_PIDInfo &pid_info = attitude_control.get_rate_yaw_pid().get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_YAW,
                                    pid_info.target*0.01f,
                                    degrees(gyro.z),
                                    pid_info.FF*0.01f,
                                    pid_info.P*0.01f,
                                    pid_info.I*0.01f,
                                    pid_info.D*0.01f,
                                    pid_info.slew_rate,
                                    pid_info.Dmod);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
    if (g.gcs_pid_mask & 8) {
        const AP_PIDInfo &pid_info = sub.pos_control.get_accel_U_pid().get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_ACCZ,
                                    pid_info.target*0.01f,
                                    -(ahrs.get_accel_ef().z + GRAVITY_MSS),
                                    pid_info.FF*0.01f,
                                    pid_info.P*0.01f,
                                    pid_info.I*0.01f,
                                    pid_info.D*0.01f,
                                    pid_info.slew_rate,
                                    pid_info.Dmod);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
}

/**
 * @brief Checks if vehicle initialization is complete
 * 
 * @details Returns the initialization status flag indicating whether all startup procedures
 *          (sensor calibration, EKF initialization, parameter loading) have completed and
 *          the vehicle is ready for operation.
 * 
 * @return bool True if vehicle fully initialized, false if still booting
 * 
 * @note Used to determine MAV_STATE_BOOT vs MAV_STATE_STANDBY
 * @see sub.ap.initialised flag set by Sub::startup_INS_ground()
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:233-235
 */
bool GCS_Sub::vehicle_initialised() const {
    return sub.ap.initialised;
}

/**
 * @brief Attempts to send a MAVLink message if buffer space available
 * 
 * @details Handles transmission of periodic telemetry messages requested by GCS stream rates.
 *          Checks available buffer space before sending each message type to prevent overflow.
 *          Implements ArduSub-specific message handling, delegating common messages to base class.
 *          
 *          ArduSub-specific messages handled:
 *          - MSG_NAMED_FLOAT: Submarine telemetry (lights, camera, tether, etc.)
 *          - MSG_TERRAIN_REQUEST: Terrain database requests (if AP_TERRAIN_AVAILABLE)
 *          - MSG_TERRAIN_REPORT: Terrain altitude reports (if AP_TERRAIN_AVAILABLE)
 *          - MSG_WIND: Explicitly ignored (returns true without sending)
 *          
 * @param[in] id Message type identifier from ap_message enum
 * 
 * @return bool True if message sent or not applicable, false if insufficient buffer space
 * 
 * @note Called by scheduler at rates configured by SR0_* through SR3_* parameters
 * @note Terrain messages only compiled when AP_TERRAIN_AVAILABLE is defined
 * @note CHECK_PAYLOAD_SIZE macro returns false if insufficient buffer space
 * @note Wind message always returns true (submarines don't estimate wind)
 * 
 * @see ap_message enum for message type identifiers
 * @see GCS_MAVLINK::try_send_message() for base class message handling
 * @see CHECK_PAYLOAD_SIZE macro in GCS_MAVLink.h
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:237-265
 */
// try to send a message, return false if it won't fit in the serial tx buffer
bool GCS_MAVLINK_Sub::try_send_message(enum ap_message id)
{
    switch (id) {

    case MSG_NAMED_FLOAT:
        send_info();
        break;

#if AP_TERRAIN_AVAILABLE
    case MSG_TERRAIN_REQUEST:
        CHECK_PAYLOAD_SIZE(TERRAIN_REQUEST);
        sub.terrain.send_request(chan);
        break;
    case MSG_TERRAIN_REPORT:
        CHECK_PAYLOAD_SIZE(TERRAIN_REPORT);
        sub.terrain.send_report(chan);
        break;
#endif

    case MSG_WIND: // other vehicles do something custom with wind:
        return true;

    default:
        return GCS_MAVLINK::try_send_message(id);
    }

    return true;
}

/**
 * @brief Handles guided mode mission command requests
 * 
 * @details Processes mission command sent to guided mode, allowing GCS to execute
 *          individual mission commands without a full mission plan. Commands are
 *          validated and executed by the vehicle's do_guided() handler.
 * 
 * @param[in] cmd Mission command structure containing command ID and parameters
 * 
 * @return bool True if command accepted and executed, false if rejected
 * 
 * @note Only processed when vehicle is in GUIDED mode
 * @see sub.do_guided() for command execution
 * @see AP_Mission::Mission_Command for command structure
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:267-270
 */
bool GCS_MAVLINK_Sub::handle_guided_request(AP_Mission::Mission_Command &cmd)
{
    return sub.do_guided(cmd);
}

/**
 * @brief Handles barometer calibration preflight command
 * 
 * @details Initiates barometer calibration to establish ground-level pressure reference.
 *          This command performs barometer zeroing to calibrate altitude readings,
 *          essential for accurate depth measurement in underwater operations.
 *          
 *          Calibration requirements:
 *          - Motors must be disarmed (safety check)
 *          - Barometer health check must pass
 *          - Vehicle should be at surface with stable conditions
 * 
 * @param[in] msg Original MAVLink message (for context)
 * 
 * @return MAV_RESULT Command execution result:
 *         - MAV_RESULT_ACCEPTED: Calibration started successfully
 *         - MAV_RESULT_FAILED: Safety check failed or barometer unhealthy
 * 
 * @note Sends informational text to GCS if disarm required
 * @note Calibration takes several seconds with vehicle stationary
 * @warning Vehicle must be disarmed before calibration for safety
 * @warning Calibrate at water surface for accurate depth measurements
 * 
 * @see sub.control_check_barometer() for barometer health validation
 * @see AP::baro().calibrate() for calibration procedure
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:272-285
 */
MAV_RESULT GCS_MAVLINK_Sub::_handle_command_preflight_calibration_baro(const mavlink_message_t &msg)
{
    if (sub.motors.armed()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Disarm before calibration.");
        return MAV_RESULT_FAILED;
    }

    if (!sub.control_check_barometer()) {
        return MAV_RESULT_FAILED;
    }

    AP::baro().calibrate(true);
    return MAV_RESULT_ACCEPTED;
}

/**
 * @brief Handles general preflight calibration commands
 * 
 * @details Processes various preflight calibration requests from GCS. Currently disables
 *          compass-motor (CompassMot) calibration for ArduSub, as motor interference is
 *          typically less significant in underwater thrusters. Other calibration types
 *          are delegated to base class handler.
 *          
 *          Command parameters:
 *          - param1: Gyro calibration flag
 *          - param2: Magnetometer calibration flag
 *          - param3: Ground pressure calibration flag
 *          - param4: RC calibration flag
 *          - param5: Accelerometer calibration flag
 *          - param6: Compass-motor calibration flag (packet.y in command_int)
 *          - param7: ESC calibration flag
 * 
 * @param[in] packet COMMAND_INT packet containing calibration parameters
 * @param[in] msg Original MAVLink message (for context)
 * 
 * @return MAV_RESULT Command execution result:
 *         - MAV_RESULT_UNSUPPORTED: CompassMot not supported for ArduSub
 *         - Other: Result from base class calibration handler
 * 
 * @note CompassMot (packet.y == 1) explicitly disabled for submarines
 * @note All other calibration types handled by GCS_MAVLINK base class
 * 
 * @see GCS_MAVLINK::_handle_command_preflight_calibration() for other calibrations
 * @see MAV_CMD_PREFLIGHT_CALIBRATION in MAVLink common.xml
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:287-297
 */
MAV_RESULT GCS_MAVLINK_Sub::_handle_command_preflight_calibration(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    if (packet.y == 1) {
        // compassmot calibration
        //result = sub.mavlink_compassmot(chan);
        gcs().send_text(MAV_SEVERITY_INFO, "#CompassMot calibration not supported");
        return MAV_RESULT_UNSUPPORTED;
    }

    return GCS_MAVLINK::_handle_command_preflight_calibration(packet, msg);
}

/**
 * @brief Handles DO_SET_ROI command to point vehicle/camera at location
 * 
 * @details Sets the Region Of Interest (ROI) for autonomous yaw control, causing the
 *          vehicle to point towards the specified location during AUTO mode. This is
 *          typically used for camera pointing or sensor orientation towards a target.
 * 
 * @param[in] roi_loc Target location (latitude, longitude, altitude) to point towards
 * 
 * @return MAV_RESULT Command execution result:
 *         - MAV_RESULT_ACCEPTED: ROI set successfully
 *         - MAV_RESULT_FAILED: Invalid location (latitude/longitude check failed)
 * 
 * @note Only affects vehicle behavior in AUTO mode
 * @note ROI continues until cleared or mode changed
 * @warning Location must have valid latitude and longitude
 * 
 * @see Location::check_latlng() for coordinate validation
 * @see Mode_Auto::set_auto_yaw_roi() for ROI implementation
 * @see MAV_CMD_DO_SET_ROI in MAVLink common.xml
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:299-306
 */
MAV_RESULT GCS_MAVLINK_Sub::handle_command_do_set_roi(const Location &roi_loc)
{
    if (!roi_loc.check_latlng()) {
        return MAV_RESULT_FAILED;
    }
    sub.mode_auto.set_auto_yaw_roi(roi_loc);
    return MAV_RESULT_ACCEPTED;
}

/**
 * @brief Handles DO_REPOSITION command to move vehicle to new location
 * 
 * @details Processes MAV_CMD_DO_REPOSITION to command the vehicle to fly/swim to a new
 *          position. Can optionally force mode change to GUIDED. Performs extensive
 *          validation of target location before accepting command.
 *          
 *          Command parameters:
 *          - param1: Ground speed (not used for ArduSub)
 *          - param2: Bitmask flags (MAV_DO_REPOSITION_FLAGS_CHANGE_MODE bit controls mode change)
 *          - param3: Reserved
 *          - param4: Yaw angle (degrees)
 *          - x (lat): Target latitude in degrees * 1E7
 *          - y (lon): Target longitude in degrees * 1E7  
 *          - z (alt): Target altitude in meters
 *          
 *          Validation sequence:
 *          1. Check if already in GUIDED or if mode change allowed
 *          2. Validate latitude/longitude are within valid ranges
 *          3. Parse location from command parameters
 *          4. Sanitize location against current position
 *          5. Set destination in guided mode
 *          6. Change to GUIDED mode if requested and not already in GUIDED
 *          7. Reload destination if mode was changed
 * 
 * @param[in] packet COMMAND_INT packet with position and flags
 * 
 * @return MAV_RESULT Command execution result:
 *         - MAV_RESULT_ACCEPTED: Position command accepted and executed
 *         - MAV_RESULT_DENIED: Not in GUIDED and mode change not allowed, or invalid location
 *         - MAV_RESULT_FAILED: Mode change failed or destination set failed
 * 
 * @note Location must be sanitized to prevent unreasonable position jumps
 * @note If mode change required, destination is set twice (before and after mode change)
 * @warning Ensure vehicle has GPS lock before using global position commands
 * 
 * @see check_latlng() for coordinate range validation
 * @see location_from_command_t() for parameter parsing
 * @see Location::sanitize() for reasonableness check
 * @see Mode_Guided::guided_set_destination() for position command
 * @see MAV_CMD_DO_REPOSITION in MAVLink common.xml
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:308-346
 */
MAV_RESULT GCS_MAVLINK_Sub::handle_command_int_do_reposition(const mavlink_command_int_t &packet)
{
    const bool change_modes = ((int32_t)packet.param2 & MAV_DO_REPOSITION_FLAGS_CHANGE_MODE) == MAV_DO_REPOSITION_FLAGS_CHANGE_MODE;
    if (!sub.flightmode->in_guided_mode() && !change_modes) {
        return MAV_RESULT_DENIED;
    }

    // sanity check location
    if (!check_latlng(packet.x, packet.y)) {
        return MAV_RESULT_DENIED;
    }

    Location request_location;
    if (!location_from_command_t(packet, request_location)) {
        return MAV_RESULT_DENIED;
    }

    if (request_location.sanitize(sub.current_loc)) {
        // if the location wasn't already sane don't load it
        return MAV_RESULT_DENIED; // failed as the location is not valid
    }

    // we need to do this first, as we don't want to change the flight mode unless we can also set the target
    if (!sub.mode_guided.guided_set_destination(request_location)) {
        return MAV_RESULT_FAILED;
    }

    if (!sub.flightmode->in_guided_mode()) {
        if (!sub.set_mode(Mode::Number::GUIDED, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_FAILED;
        }
        // the position won't have been loaded if we had to change the flight mode, so load it again
        if (!sub.mode_guided.guided_set_destination(request_location)) {
            return MAV_RESULT_FAILED;
        }
    }

    return MAV_RESULT_ACCEPTED;
}

/**
 * @brief Dispatches COMMAND_INT messages to appropriate handlers
 * 
 * @details Routes incoming MAVLink COMMAND_INT messages to ArduSub-specific command handlers.
 *          COMMAND_INT uses 32-bit integer latitude/longitude for better precision than
 *          COMMAND_LONG. Unsupported commands are passed to base class handler.
 *          
 *          ArduSub-specific commands handled:
 *          - MAV_CMD_CONDITION_YAW: Set target yaw angle
 *          - MAV_CMD_DO_CHANGE_SPEED: Change horizontal speed
 *          - MAV_CMD_DO_MOTOR_TEST: Test individual motors/thrusters
 *          - MAV_CMD_DO_REPOSITION: Move to new position
 *          - MAV_CMD_MISSION_START: Begin autonomous mission (must be armed)
 *          - MAV_CMD_NAV_LOITER_UNLIM: Enter position hold mode
 *          - MAV_CMD_NAV_LAND: Surface (enter SURFACE mode)
 * 
 * @param[in] packet COMMAND_INT packet containing command and parameters
 * @param[in] msg Original MAVLink message (for context)
 * 
 * @return MAV_RESULT Command execution result from handler function
 * 
 * @note MISSION_START first-item/last-item parameters not supported (must be zero)
 * @note All other commands delegated to GCS_MAVLINK base class
 * 
 * @see mavlink_command_int_t structure in MAVLink library
 * @see GCS_MAVLINK::handle_command_int_packet() for base class commands
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:348-380
 */
MAV_RESULT GCS_MAVLINK_Sub::handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    switch(packet.command) {

    case MAV_CMD_CONDITION_YAW:
        return handle_MAV_CMD_CONDITION_YAW(packet);

    case MAV_CMD_DO_CHANGE_SPEED:
        return handle_MAV_CMD_DO_CHANGE_SPEED(packet);

    case MAV_CMD_DO_MOTOR_TEST:
        return handle_MAV_CMD_DO_MOTOR_TEST(packet);

    case MAV_CMD_DO_REPOSITION:
        return handle_command_int_do_reposition(packet);

    case MAV_CMD_MISSION_START:
        if (!is_zero(packet.param1) || !is_zero(packet.param2)) {
            // first-item/last item not supported
            return MAV_RESULT_DENIED;
        }
        return handle_MAV_CMD_MISSION_START(packet);

    case MAV_CMD_NAV_LOITER_UNLIM:
        return handle_MAV_CMD_NAV_LOITER_UNLIM(packet);

    case MAV_CMD_NAV_LAND:
        return handle_MAV_CMD_NAV_LAND(packet);

    }

    return GCS_MAVLINK::handle_command_int_packet(packet, msg);
}

/**
 * @brief Handles NAV_LOITER_UNLIM command to enter position hold mode
 * 
 * @details Processes MAV_CMD_NAV_LOITER_UNLIM to command the vehicle to hold current position
 *          indefinitely. Changes vehicle mode to POSHOLD (position hold) which maintains
 *          current lateral position and depth using position controller.
 * 
 * @param[in] packet COMMAND_INT packet (parameters not used for this command)
 * 
 * @return MAV_RESULT Command execution result:
 *         - MAV_RESULT_ACCEPTED: Successfully entered POSHOLD mode
 *         - MAV_RESULT_FAILED: Mode change failed
 * 
 * @note Requires position estimate from EKF or GPS for position hold
 * @see Mode::Number::POSHOLD for position hold mode implementation
 * @see MAV_CMD_NAV_LOITER_UNLIM in MAVLink common.xml
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:877-883
 */
MAV_RESULT GCS_MAVLINK_Sub::handle_MAV_CMD_NAV_LOITER_UNLIM(const mavlink_command_int_t &packet)
{
        if (!sub.set_mode(Mode::Number::POSHOLD, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;
}

/**
 * @brief Handles NAV_LAND command to surface the submarine
 * 
 * @details Processes MAV_CMD_NAV_LAND to command the vehicle to ascend to the surface.
 *          For submarines, "landing" means surfacing. Changes vehicle mode to SURFACE
 *          which controls buoyancy and thrust to bring the vehicle to water surface.
 * 
 * @param[in] packet COMMAND_INT packet (parameters not used for this command)
 * 
 * @return MAV_RESULT Command execution result:
 *         - MAV_RESULT_ACCEPTED: Successfully entered SURFACE mode
 *         - MAV_RESULT_FAILED: Mode change failed
 * 
 * @note SURFACE mode continues until vehicle reaches surface or mode is changed
 * @warning Ensure safe surfacing conditions (no obstructions above)
 * @see Mode::Number::SURFACE for surface mode implementation
 * @see MAV_CMD_NAV_LAND in MAVLink common.xml
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:885-891
 */
MAV_RESULT GCS_MAVLINK_Sub::handle_MAV_CMD_NAV_LAND(const mavlink_command_int_t &packet)
{
        if (!sub.set_mode(Mode::Number::SURFACE, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;
}

/**
 * @brief Handles CONDITION_YAW command to set target heading
 * 
 * @details Processes MAV_CMD_CONDITION_YAW to command the vehicle to yaw to a specific
 *          heading at a controlled rate. Can specify absolute heading or relative offset
 *          from current heading, and direction of rotation (clockwise or counter-clockwise).
 *          
 *          Parameter validation:
 *          - param1 must be 0-360 degrees
 *          - param4 must be 0 (absolute) or 1 (relative)
 * 
 * @param[in] packet COMMAND_INT packet with yaw parameters:
 *                   - param1: Target angle [0-360 degrees]
 *                   - param2: Speed during change [deg/s]
 *                   - param3: Direction (-1=CCW, +1=CW)
 *                   - param4: Relative offset (1) or absolute angle (0)
 * 
 * @return MAV_RESULT Command execution result:
 *         - MAV_RESULT_ACCEPTED: Valid yaw command, target heading set
 *         - MAV_RESULT_DENIED: Invalid parameters (angle out of range or invalid mode flag)
 * 
 * @note Only processed in AUTO mode via mode_auto.set_auto_yaw_look_at_heading()
 * @see Mode_Auto::set_auto_yaw_look_at_heading() for yaw control implementation
 * @see MAV_CMD_CONDITION_YAW in MAVLink common.xml
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:893-906
 */
MAV_RESULT GCS_MAVLINK_Sub::handle_MAV_CMD_CONDITION_YAW(const mavlink_command_int_t &packet)
{
        // param1 : target angle [0-360]
        // param2 : speed during change [deg per second]
        // param3 : direction (-1:ccw, +1:cw)
        // param4 : relative offset (1) or absolute angle (0)
        if ((packet.param1 >= 0.0f)   &&
            (packet.param1 <= 360.0f) &&
            (is_zero(packet.param4) || is_equal(packet.param4,1.0f))) {
            sub.mode_auto.set_auto_yaw_look_at_heading(packet.param1, packet.param2, (int8_t)packet.param3, (uint8_t)packet.param4);
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_DENIED;
}

/**
 * @brief Handles DO_CHANGE_SPEED command to change vehicle speed
 * 
 * @details Processes MAV_CMD_DO_CHANGE_SPEED to change the vehicle's horizontal movement speed.
 *          For submarines, only horizontal (groundspeed/airspeed) changes are supported.
 *          Climb/descent speed changes are not implemented.
 *          
 *          Speed type handling:
 *          - SPEED_TYPE_GROUNDSPEED: Sets horizontal navigation speed (supported)
 *          - SPEED_TYPE_AIRSPEED: Treated as groundspeed for GCS compatibility (supported)
 *          - SPEED_TYPE_CLIMB_SPEED: Not supported (denied)
 *          - SPEED_TYPE_DESCENT_SPEED: Not supported (denied)
 *          
 *          Parameter validation:
 *          - param2 (target speed) must be positive (> 0)
 * 
 * @param[in] packet COMMAND_INT packet with speed parameters:
 *                   - param1: Speed type (SPEED_TYPE enum)
 *                   - param2: Target speed in m/s (must be positive)
 *                   - param3: Throttle (not used)
 * 
 * @return MAV_RESULT Command execution result:
 *         - MAV_RESULT_ACCEPTED: Speed changed successfully
 *         - MAV_RESULT_DENIED: Invalid speed type or non-positive speed value
 * 
 * @note Speed is converted from m/s to cm/s for internal waypoint navigation
 * @see AC_WPNav::set_speed_NE_cms() for speed setting
 * @see SPEED_TYPE enum in MAVLink common.xml
 * @see MAV_CMD_DO_CHANGE_SPEED in MAVLink common.xml
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:908-928
 */
MAV_RESULT GCS_MAVLINK_Sub::handle_MAV_CMD_DO_CHANGE_SPEED(const mavlink_command_int_t &packet)
{
    if (!is_positive(packet.param2)) {
        // Target speed must be larger than zero
        return MAV_RESULT_DENIED;
    }

    switch (SPEED_TYPE(packet.param1)) {
        case SPEED_TYPE_CLIMB_SPEED:
        case SPEED_TYPE_DESCENT_SPEED:
        case SPEED_TYPE_ENUM_END:
            break;

        case SPEED_TYPE_AIRSPEED: // Airspeed is treated as ground speed for GCS compatibility
        case SPEED_TYPE_GROUNDSPEED:
            sub.wp_nav.set_speed_NE_cms(packet.param2 * 100.0);
            return MAV_RESULT_ACCEPTED;
    }

    return MAV_RESULT_DENIED;
}

/**
 * @brief Handles MISSION_START command to begin autonomous mission
 * 
 * @details Processes MAV_CMD_MISSION_START to start execution of uploaded mission plan.
 *          Changes vehicle mode to AUTO which executes mission commands sequentially
 *          from the mission list stored in vehicle memory.
 *          
 *          Requirements:
 *          - Vehicle must be armed (safety check)
 *          - Mission must be loaded and valid
 *          - param1 and param2 must be zero (first-item/last-item not supported)
 * 
 * @param[in] packet COMMAND_INT packet (parameters not used beyond validation)
 * 
 * @return MAV_RESULT Command execution result:
 *         - MAV_RESULT_ACCEPTED: Mission started, vehicle in AUTO mode
 *         - MAV_RESULT_FAILED: Vehicle disarmed or mode change failed
 * 
 * @note Partial mission execution (first-item/last-item) not supported
 * @warning Vehicle must be armed before starting mission for safety
 * @see Mode::Number::AUTO for autonomous mission execution
 * @see MAV_CMD_MISSION_START in MAVLink common.xml
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:930-936
 */
MAV_RESULT GCS_MAVLINK_Sub::handle_MAV_CMD_MISSION_START(const mavlink_command_int_t &packet)
{
        if (sub.motors.armed() && sub.set_mode(Mode::Number::AUTO, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;
}

/**
 * @brief Handles DO_MOTOR_TEST command to test individual motors/thrusters
 * 
 * @details Processes MAV_CMD_DO_MOTOR_TEST to command individual thruster testing for
 *          motor configuration verification and system checkout. Allows testing each
 *          thruster independently at specified throttle for a defined duration.
 *          
 *          Throttle types supported:
 *          - MOTOR_TEST_THROTTLE_PERCENT (0): Throttle as percentage (0-100)
 *          - MOTOR_TEST_THROTTLE_PWM (1): Direct PWM value (typically 1000-2000)
 *          - MOTOR_TEST_THROTTLE_PILOT (2): Pilot throttle channel pass-through
 *          
 *          Safety considerations:
 *          - Vehicle must be disarmed or in controlled environment
 *          - Test timeout prevents runaway motor operation
 *          - Individual motor testing validates frame configuration
 * 
 * @param[in] packet COMMAND_INT packet with motor test parameters:
 *                   - param1: Motor sequence number (1 to max motors)
 *                   - param2: Throttle type (MOTOR_TEST_THROTTLE_TYPE enum)
 *                   - param3: Throttle value (range depends on param2)
 *                   - param4: Timeout in seconds
 * 
 * @return MAV_RESULT Command execution result:
 *         - MAV_RESULT_ACCEPTED: Motor test started successfully
 *         - MAV_RESULT_FAILED: Invalid parameters or safety check failed
 * 
 * @note Delegated to sub.handle_do_motor_test() for actual execution
 * @warning Test motors in safe environment only - thrusters will spin!
 * @warning Ensure propellers are clear of obstructions during testing
 * @see Sub::handle_do_motor_test() for motor test implementation
 * @see MOTOR_TEST_THROTTLE_TYPE enum in MAVLink common.xml
 * @see MAV_CMD_DO_MOTOR_TEST in MAVLink common.xml
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:938-948
 */
MAV_RESULT GCS_MAVLINK_Sub::handle_MAV_CMD_DO_MOTOR_TEST(const mavlink_command_int_t &packet)
{
        // param1 : motor sequence number (a number from 1 to max number of motors on the vehicle)
        // param2 : throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)
        // param3 : throttle (range depends upon param2)
        // param4 : timeout (in seconds)
        if (!sub.handle_do_motor_test(packet)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;
}

/**
 * @brief Dispatches incoming MAVLink messages to appropriate handlers
 * 
 * @details Central message routing function for ArduSub-specific MAVLink messages.
 *          Handles incoming messages from ground control stations, companion computers,
 *          and other MAVLink systems. Unhandled messages are passed to base class.
 *          
 *          ArduSub-specific messages handled:
 *          - MANUAL_CONTROL (69): Joystick input for manual control
 *          - RC_CHANNELS_OVERRIDE (70): Direct RC channel overrides
 *          - SET_ATTITUDE_TARGET (82): Attitude + thrust control in guided mode
 *          - SET_POSITION_TARGET_LOCAL_NED (84): Local position control
 *          - SET_POSITION_TARGET_GLOBAL_INT (86): Global position control
 *          
 *          Message validation:
 *          - System ID verification (must be from configured GCS)
 *          - Target ID verification (must be addressed to this vehicle)
 *          - Mode compatibility checks (e.g., GUIDED mode for position targets)
 *          
 *          Safety features:
 *          - Updates failsafe timers on valid control messages
 *          - Tracks GCS heartbeat for communication monitoring
 *          - Validates type masks for hybrid control (attitude + position)
 * 
 * @param[in] msg MAVLink message structure containing message ID and payload
 * 
 * @note Unsupported message IDs are delegated to GCS_MAVLINK::handle_message()
 * @note Control messages reset failsafe.last_pilot_input_ms timer
 * @warning MANUAL_CONTROL and RC_CHANNELS_OVERRIDE only accepted from configured GCS
 * @warning SET_ATTITUDE_TARGET thrust converted to climb rate for submarine operation
 * 
 * @see GCS_MAVLINK::handle_message() for base class message handling
 * @see mavlink_message_t structure in MAVLink library
 * @see transform_manual_control_to_rc_override() for joystick processing
 * @see Mode_Guided for position and attitude target handling
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:455-830
 */
void GCS_MAVLINK_Sub::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {

    // ========== MANUAL_CONTROL (MAV ID: 69) ==========
    // Joystick control input from ground control station
    // Translates joystick axes (x, y, z, r) and button states into RC channel overrides
    // Security: Only accepts messages from configured GCS system ID
    // Updates: Resets failsafe.last_pilot_input_ms timer to prevent failsafe trigger
    case MAVLINK_MSG_ID_MANUAL_CONTROL: {     // MAV ID: 69
        if (msg.sysid != gcs().sysid_gcs()) {
            break;    // Only accept control from our gcs
        }
        mavlink_manual_control_t packet;
        mavlink_msg_manual_control_decode(&msg, &packet);

        if (packet.target != gcs().sysid_this_mav()) {
            break; // only accept control aimed at us
        }

        sub.transform_manual_control_to_rc_override(
            packet.x,
            packet.y,
            packet.z,
            packet.r,
            packet.buttons,
            packet.buttons2,
            packet.enabled_extensions,
            packet.s,
            packet.t,
            packet.aux1,
            packet.aux2,
            packet.aux3,
            packet.aux4,
            packet.aux5,
            packet.aux6
        );

        sub.failsafe.last_pilot_input_ms = AP_HAL::millis();
        // a RC override message is considered to be a 'heartbeat'
        // from the ground station for failsafe purposes
        sysid_mygcs_seen(AP_HAL::millis());
        break;
    }

    // ========== RC_CHANNELS_OVERRIDE (MAV ID: 70) ==========
    // Direct RC channel override values from ground control station
    // Allows GCS to control vehicle by directly setting RC channel PWM values
    // Security: Only accepts messages from configured GCS system ID
    // Updates: Resets failsafe.last_pilot_input_ms timer and GCS heartbeat
    // Note: Commonly used for scripting and automated control sequences
    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {     // MAV ID: 70
        if (msg.sysid != gcs().sysid_gcs()) {
            break;    // Only accept control from our gcs
        }

        sub.failsafe.last_pilot_input_ms = AP_HAL::millis();
        // a RC override message is considered to be a 'heartbeat'
        // from the ground station for failsafe purposes
        
        handle_rc_channels_override(msg);
        break;
    }

    
    // ========== SET_ATTITUDE_TARGET (MAV ID: 82) ==========
    // Attitude and thrust control for guided mode operation
    // Allows external systems (companion computers, GCS) to directly control attitude quaternion
    // Type mask interpretation:
    //   - Bit 6: Attitude control enable (0=use attitude, 1=ignore)
    //   - Bit 7: Thrust control enable (0=use thrust, 1=ignore)
    // Thrust conversion: 0.0-1.0 thrust → climb rate in cm/s
    //   - 0.5 = neutral (0 cm/s)
    //   - >0.5 = ascend up to WPNAV_SPEED_UP
    //   - <0.5 = descend up to WPNAV_SPEED_DN
    // Storage: If only attitude specified (no thrust), stores in set_attitude_target_no_gps
    // Used for: Companion computer attitude control, external stabilization systems
    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET: { // MAV ID: 82
        // decode packet
        mavlink_set_attitude_target_t packet;
        mavlink_msg_set_attitude_target_decode(&msg, &packet);

        // ensure type_mask specifies to use attitude
        // the thrust can be used from the altitude hold
        if (packet.type_mask & (1<<6)) {
            sub.set_attitude_target_no_gps = {AP_HAL::millis(), packet};
        }

        // ensure type_mask specifies to use attitude and thrust
        if ((packet.type_mask & ((1<<7)|(1<<6))) != 0) {
            break;
        }

        // convert thrust to climb rate
        packet.thrust = constrain_float(packet.thrust, 0.0f, 1.0f);
        float climb_rate_cms = 0.0f;
        if (is_equal(packet.thrust, 0.5f)) {
            climb_rate_cms = 0.0f;
        } else if (packet.thrust > 0.5f) {
            // climb at up to WPNAV_SPEED_UP
            climb_rate_cms = (packet.thrust - 0.5f) * 2.0f * sub.wp_nav.get_default_speed_up_cms();
        } else {
            // descend at up to WPNAV_SPEED_DN
            climb_rate_cms = (packet.thrust - 0.5f) * 2.0f * sub.wp_nav.get_default_speed_down_cms();
        }
        sub.mode_guided.guided_set_angle(Quaternion(packet.q[0],packet.q[1],packet.q[2],packet.q[3]), climb_rate_cms);
        break;
    }

    // ========== SET_POSITION_TARGET_LOCAL_NED (MAV ID: 84) ==========
    // Local position, velocity, and yaw control in NED (North-East-Down) frame
    // Mode requirements: GUIDED or AUTO (with Auto_NavGuided sub-mode)
    // Coordinate frames supported:
    //   - MAV_FRAME_LOCAL_NED: Absolute position in local NED origin
    //   - MAV_FRAME_LOCAL_OFFSET_NED: Offset from current position
    //   - MAV_FRAME_BODY_NED: Relative to vehicle body frame (NED convention)
    //   - MAV_FRAME_BODY_OFFSET_NED: Body frame with offset from current
    //   - MAV_FRAME_BODY_FRD: Body frame (Forward-Right-Down convention)
    // Type mask: Selectively ignore position, velocity, acceleration, yaw, yaw_rate
    // Units: Position in meters, velocity in m/s, converted to centimeters internally
    // Processing: Rotates body-frame vectors to NED, adds offsets, dispatches to Mode_Guided
    // Hybrid control: Supports combined position+velocity, velocity-only, position-only
    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED: {   // MAV ID: 84
        // decode packet
        mavlink_set_position_target_local_ned_t packet;
        mavlink_msg_set_position_target_local_ned_decode(&msg, &packet);

        // exit if vehicle is not in Guided mode or Auto-Guided mode
        if ((sub.control_mode != Mode::Number::GUIDED) && !(sub.control_mode == Mode::Number::AUTO && sub.auto_mode == Auto_NavGuided)) {
            break;
        }

        // check for supported coordinate frames
        if (packet.coordinate_frame != MAV_FRAME_LOCAL_NED &&
                packet.coordinate_frame != MAV_FRAME_LOCAL_OFFSET_NED &&
                packet.coordinate_frame != MAV_FRAME_BODY_NED &&
                packet.coordinate_frame != MAV_FRAME_BODY_OFFSET_NED &&
                packet.coordinate_frame != MAV_FRAME_BODY_FRD) {
            break;
        }

        bool pos_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
        bool vel_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
        bool acc_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;
        bool yaw_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
        bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;

        // prepare position
        Vector3f pos_vector;
        if (!pos_ignore) {
            // convert to cm
            pos_vector = Vector3f(packet.x * 100.0f, packet.y * 100.0f, -packet.z * 100.0f);
            // rotate from body-frame if necessary
            if (packet.coordinate_frame == MAV_FRAME_BODY_NED ||
                    packet.coordinate_frame == MAV_FRAME_BODY_FRD ||
                    packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                sub.rotate_body_frame_to_NE(pos_vector.x, pos_vector.y);
            }
            // add body offset if necessary
            if (packet.coordinate_frame == MAV_FRAME_LOCAL_OFFSET_NED ||
                    packet.coordinate_frame == MAV_FRAME_BODY_NED ||
                    packet.coordinate_frame == MAV_FRAME_BODY_FRD ||
                    packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                pos_vector += sub.inertial_nav.get_position_neu_cm();
            }
        }

        // prepare velocity
        Vector3f vel_vector;
        if (!vel_ignore) {
            // convert to cm
            vel_vector = Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f);
            // rotate from body-frame if necessary
            if (packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_FRD || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                sub.rotate_body_frame_to_NE(vel_vector.x, vel_vector.y);
            }
        }

        // prepare yaw
        float yaw_cd =  0.0f;
        bool yaw_relative = false;
        float yaw_rate_cds = 0.0f;
        if (!yaw_ignore) {
            yaw_cd = degrees(packet.yaw) * 100.0f;
            yaw_relative = packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED;
        }
        if (!yaw_rate_ignore) {
            yaw_rate_cds = degrees(packet.yaw_rate) * 100.0f;
        }

        // send request
        if (!pos_ignore && !vel_ignore && acc_ignore) {
            sub.mode_guided.guided_set_destination_posvel(pos_vector, vel_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
        } else if (pos_ignore && !vel_ignore && acc_ignore) {
            sub.mode_guided.guided_set_velocity(vel_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
        } else if (!pos_ignore && vel_ignore && acc_ignore) {
            sub.mode_guided.guided_set_destination(pos_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
        }

        break;
    }

    // ========== SET_POSITION_TARGET_GLOBAL_INT (MAV ID: 86) ==========
    // Global position, velocity control using GPS coordinates (latitude/longitude/altitude)
    // Mode requirements: GUIDED, AUTO (with Auto_NavGuided), or ALT_HOLD
    // Coordinate system: WGS84 latitude/longitude in 1E7 degrees, altitude in meters
    // Special ALT_HOLD mode behavior: Only controls depth (Z-axis) when in ALT_HOLD
    // Type mask: Selectively ignore position (XY), depth (Z), velocity, acceleration
    // Position validation: Checks latitude/longitude validity before use
    // Altitude frames: Supports multiple altitude reference frames via mavlink_coordinate_frame_to_location_alt_frame()
    // Conversion: GPS coordinates → NEU (North-East-Up) in centimeters from origin
    // Hybrid control: Supports combined position+velocity, velocity-only, position-only
    // Use cases: Waypoint navigation, global position hold, depth control
    case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT: {  // MAV ID: 86
        // decode packet
        mavlink_set_position_target_global_int_t packet;
        mavlink_msg_set_position_target_global_int_decode(&msg, &packet);

        // exit if vehicle is not in Guided, Auto-Guided, or Depth Hold modes
        if ((sub.control_mode != Mode::Number::GUIDED)
            && !(sub.control_mode == Mode::Number::AUTO && sub.auto_mode == Auto_NavGuided)
            && !(sub.control_mode == Mode::Number::ALT_HOLD)) {
            break;
        }

        bool z_ignore        = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_Z_IGNORE;
        bool pos_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
        bool vel_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
        bool acc_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;

        /*
         * for future use:
         * bool force           = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_FORCE;
         * bool yaw_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
         * bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;
         */

        if (!z_ignore && sub.control_mode == Mode::Number::ALT_HOLD) { // Control only target depth when in ALT_HOLD
            sub.pos_control.set_pos_desired_U_cm(packet.alt*100);
            break;
        }

        Vector3f pos_neu_cm;  // position (North, East, Up coordinates) in centimeters

        if (!pos_ignore) {
            // sanity check location
            if (!check_latlng(packet.lat_int, packet.lon_int)) {
                break;
            }
            Location::AltFrame frame;
            if (!mavlink_coordinate_frame_to_location_alt_frame((MAV_FRAME)packet.coordinate_frame, frame)) {
                // unknown coordinate frame
                break;
            }
            const Location loc{
                packet.lat_int,
                packet.lon_int,
                int32_t(packet.alt*100),
                frame,
            };
            if (!loc.get_vector_from_origin_NEU_cm(pos_neu_cm)) {
                break;
            }
        }

        if (!pos_ignore && !vel_ignore && acc_ignore) {
            sub.mode_guided.guided_set_destination_posvel(pos_neu_cm, Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f));
        } else if (pos_ignore && !vel_ignore && acc_ignore) {
            sub.mode_guided.guided_set_velocity(Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f));
        } else if (!pos_ignore && vel_ignore && acc_ignore) {
            sub.mode_guided.guided_set_destination(pos_neu_cm);
        }

        break;
    }

    // ========== TERRAIN_DATA / TERRAIN_CHECK (MAV ID: 134/135) ==========
    // Terrain data exchange for terrain following and collision avoidance
    // TERRAIN_DATA: Provides terrain height data from GCS/external source
    // TERRAIN_CHECK: Requests terrain data for specific locations
    // Conditional: Only compiled if AP_TERRAIN_AVAILABLE is enabled
    // Handler: Delegates to AP_Terrain library for terrain database management
    // Use cases: Terrain following missions, altitude-above-terrain display
    case MAVLINK_MSG_ID_TERRAIN_DATA:
    case MAVLINK_MSG_ID_TERRAIN_CHECK:
#if AP_TERRAIN_AVAILABLE
        sub.terrain.handle_data(chan, msg);
#endif
        break;

    // ========== SYS_STATUS (MAV ID: 1) ==========
    // Remote leak detector integration via MAVLink subsystem
    // Purpose: Allows external enclosures with leak detectors to report status
    // Sensor bit: MAV_SENSOR_WATER (0x20000000) indicates leak detector presence
    // Health monitoring: If sensor enabled but not healthy → leak detected
    // Use case: Companion computer in separate enclosure with its own leak detector
    // Action: Sets leak_detector.set_detect() to trigger leak failsafe
    // Note: Does NOT handle primary vehicle SYS_STATUS (different purpose)
    case MAVLINK_MSG_ID_SYS_STATUS: {
        uint32_t MAV_SENSOR_WATER = 0x20000000;
        mavlink_sys_status_t packet;
        mavlink_msg_sys_status_decode(&msg, &packet);
        if ((packet.onboard_control_sensors_enabled & MAV_SENSOR_WATER) && !(packet.onboard_control_sensors_health & MAV_SENSOR_WATER)) {
            sub.leak_detector.set_detect();
        }
    }
        break;

    default:
        GCS_MAVLINK::handle_message(msg);
        break;
    }     // end switch
} // end handle mavlink

/**
 * @brief Reports MAVLink protocol capabilities supported by ArduSub
 * 
 * @details Advertises which MAVLink features and message types this vehicle supports.
 *          Ground control stations query capabilities to enable/disable UI features.
 *          
 *          ArduSub capabilities:
 *          - MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT: Float mission items supported
 *          - MAV_PROTOCOL_CAPABILITY_MISSION_INT: Integer mission items supported
 *          - MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED: Local position control
 *          - MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT: Global position control
 *          - MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION: Emergency motor shutdown
 *          - MAV_PROTOCOL_CAPABILITY_TERRAIN: Terrain following (if AP_TERRAIN_AVAILABLE)
 *          - MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET: Direct attitude control
 *          - Plus base class capabilities from GCS_MAVLINK::capabilities()
 * 
 * @return Bitmask of MAV_PROTOCOL_CAPABILITY flags
 * 
 * @note Terrain capability conditional on AP_TERRAIN_AVAILABLE and terrain.enabled()
 * @see MAV_PROTOCOL_CAPABILITY enum in MAVLink common.xml
 * @see AUTOPILOT_VERSION message that reports these capabilities
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:1461-1474
 */
uint64_t GCS_MAVLINK_Sub::capabilities() const
{
    return (MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
            MAV_PROTOCOL_CAPABILITY_MISSION_INT |
            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
            MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
#if AP_TERRAIN_AVAILABLE
            (sub.terrain.enabled() ? MAV_PROTOCOL_CAPABILITY_TERRAIN : 0) |
#endif
            MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
            GCS_MAVLINK::capabilities()
        );
}

/**
 * @brief Handles MAV_CMD_DO_FLIGHTTERMINATION command for emergency motor shutdown
 * 
 * @details Provides emergency disarm capability via MAVLink command.
 *          Used for safety-critical situations requiring immediate motor shutdown.
 *          
 *          Command interpretation:
 *          - param1 > 0.5: Terminate flight (disarm motors immediately)
 *          - param1 <= 0.5: Reject command (return MAV_RESULT_FAILED)
 *          
 *          Disarm method: Uses AP_Arming::Method::TERMINATION for logging/telemetry
 * 
 * @param[in] packet COMMAND_INT packet with param1 as termination flag
 * 
 * @return MAV_RESULT_ACCEPTED if motors disarmed, MAV_RESULT_FAILED otherwise
 * 
 * @warning Emergency use only - immediately stops all motors without safety checks
 * @note This bypasses normal disarm safety interlocks
 * @see MAV_CMD_DO_FLIGHTTERMINATION in MAVLink common.xml
 * @see AP_Arming::disarm() for disarm implementation
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:1476-1483
 */
MAV_RESULT GCS_MAVLINK_Sub::handle_flight_termination(const mavlink_command_int_t &packet)
{
    if (packet.param1 > 0.5f) {
        sub.arming.disarm(AP_Arming::Method::TERMINATION);
        return MAV_RESULT_ACCEPTED;
    }
    return MAV_RESULT_FAILED;
}

/**
 * @brief Provides MSL (Mean Sea Level) altitude for GLOBAL_POSITION_INT message
 * 
 * @details Returns absolute altitude above mean sea level for telemetry reporting.
 *          Used in GLOBAL_POSITION_INT (MAV ID 33) to report vehicle altitude.
 *          
 *          For submarines:
 *          - Positive values: Above sea surface
 *          - Zero: At sea surface
 *          - Negative values: Below sea surface (depth)
 *          
 *          Unit conversion: meters → millimeters (MAVLink standard)
 * 
 * @return Altitude above MSL in millimeters (int32_t)
 * 
 * @note Calls sub.get_alt_msl() which accounts for baro/depth sensor fusion
 * @see GLOBAL_POSITION_INT message (MAV ID 33) for usage context
 * @see global_position_int_relative_alt() for relative altitude
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:1485-1488
 */
int32_t GCS_MAVLINK_Sub::global_position_int_alt() const
{
    return static_cast<int32_t>(sub.get_alt_msl() * 1000.0f);
}

/**
 * @brief Provides relative altitude for GLOBAL_POSITION_INT message
 * 
 * @details Returns altitude relative to home/origin position for telemetry reporting.
 *          Used in GLOBAL_POSITION_INT (MAV ID 33) alongside absolute MSL altitude.
 *          
 *          For submarines:
 *          - Relative to dive start position (typically surface)
 *          - Positive: Above home
 *          - Negative: Below home (typical during dive)
 *          
 *          Unit conversion: meters → millimeters (MAVLink standard)
 * 
 * @return Altitude relative to home in millimeters (int32_t)
 * 
 * @note Calls sub.get_alt_rel() which provides home-relative altitude
 * @see GLOBAL_POSITION_INT message (MAV ID 33) for usage context
 * @see global_position_int_alt() for absolute altitude
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:1490-1493
 */
int32_t GCS_MAVLINK_Sub::global_position_int_relative_alt() const
{
    return static_cast<int32_t>(sub.get_alt_rel() * 1000.0f);
}

/**
 * @brief Provides target altitude for HIGH_LATENCY2 telemetry message
 * 
 * @details Returns target depth/altitude for satellite or low-bandwidth telemetry.
 *          HIGH_LATENCY2 message designed for Iridium, LoRa, or other constrained links.
 *          
 *          Mode-specific behavior:
 *          - AUTO/GUIDED: Returns target altitude (current position + position error)
 *          - Other modes: Returns 0 (no target)
 *          
 *          Calculation: Current altitude + vertical position error from controller
 *          
 *          Unit conversion: centimeters → meters (multiplies by 0.01)
 * 
 * @return Target altitude in meters (int16_t)
 * 
 * @note Only compiled if HAL_HIGH_LATENCY2_ENABLED defined
 * @see HIGH_LATENCY2 message (MAV ID 235) for low-bandwidth telemetry
 * @see pos_control.get_pos_error_U_cm() for vertical position error
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:1496-1508
 */
#if HAL_HIGH_LATENCY2_ENABLED
int16_t GCS_MAVLINK_Sub::high_latency_target_altitude() const
{
    AP_AHRS &ahrs = AP::ahrs();
    Location global_position_current;
    UNUSED_RESULT(ahrs.get_location(global_position_current));

    //return units are m
    if (sub.control_mode == Mode::Number::AUTO || sub.control_mode == Mode::Number::GUIDED) {
        return 0.01 * (global_position_current.alt + sub.pos_control.get_pos_error_U_cm());
    }
    return 0;
    
}

/**
 * @brief Provides target heading for HIGH_LATENCY2 telemetry message
 * 
 * @details Returns target bearing to destination for low-bandwidth telemetry.
 *          HIGH_LATENCY2 uses compressed heading format to save bandwidth.
 *          
 *          Mode-specific behavior:
 *          - AUTO/GUIDED: Returns bearing to waypoint destination
 *          - Other modes: Returns 0 (no target)
 *          
 *          Unit conversion: centidegrees (-18000 to 18000) → degrees/2 (0 to 180)
 *          Conversion steps:
 *          1. wrap_360_cd() converts -18000→18000 to 0→36000 centidegrees
 *          2. Divide by 200 to get degrees/2 (0→180)
 * 
 * @return Target heading in degrees/2 (uint8_t, range 0-180)
 * 
 * @note Only compiled if HAL_HIGH_LATENCY2_ENABLED defined
 * @note Multiply by 2 to get actual heading in degrees
 * @see HIGH_LATENCY2 message (MAV ID 235) for format specification
 * @see wp_nav.get_wp_bearing_to_destination_cd() for bearing calculation
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:1510-1518
 */
uint8_t GCS_MAVLINK_Sub::high_latency_tgt_heading() const
{
    // return units are deg/2
    if (sub.control_mode == Mode::Number::AUTO || sub.control_mode == Mode::Number::GUIDED) {
        // need to convert -18000->18000 to 0->360/2
        return wrap_360_cd(sub.wp_nav.get_wp_bearing_to_destination_cd()) / 200;
    }
    return 0;      
}

/**
 * @brief Provides target distance for HIGH_LATENCY2 telemetry message
 * 
 * @details Returns remaining distance to waypoint for low-bandwidth telemetry.
 *          HIGH_LATENCY2 uses decimeters for distance to reduce packet size.
 *          
 *          Mode-specific behavior:
 *          - AUTO/GUIDED: Returns distance to waypoint destination
 *          - Other modes: Returns 0 (no target)
 *          
 *          Unit conversion: centimeters → decimeters (multiply by 0.001)
 *          Saturation: Clamped to UINT16_MAX (65535 decimeters = 6553.5 meters)
 * 
 * @return Distance to target in decimeters (uint16_t)
 * 
 * @note Only compiled if HAL_HIGH_LATENCY2_ENABLED defined
 * @note Multiply by 0.1 to get distance in meters
 * @see HIGH_LATENCY2 message (MAV ID 235) for format specification
 * @see wp_nav.get_wp_distance_to_destination_cm() for distance calculation
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:1520-1527
 */    
uint16_t GCS_MAVLINK_Sub::high_latency_tgt_dist() const
{
    // return units are dm
    if (sub.control_mode == Mode::Number::AUTO || sub.control_mode == Mode::Number::GUIDED) {
        return MIN(sub.wp_nav.get_wp_distance_to_destination_cm() * 0.001, UINT16_MAX);
    }
    return 0;
}

/**
 * @brief Provides target velocity for HIGH_LATENCY2 telemetry message
 * 
 * @details Returns desired velocity magnitude for low-bandwidth telemetry.
 *          Despite function name "airspeed", this is water velocity for submarines.
 *          HIGH_LATENCY2 uses compressed velocity format (m/s * 5) to save bandwidth.
 *          
 *          Mode-specific behavior:
 *          - AUTO/GUIDED: Returns magnitude of desired velocity vector (NEU frame)
 *          - Other modes: Returns 0 (no target velocity)
 *          
 *          Unit conversion: cm/s → (m/s * 5)
 *          Conversion steps:
 *          1. Get desired velocity NEU vector in cm/s
 *          2. Calculate vector magnitude (length)
 *          3. Divide by 100 to get m/s
 *          4. Multiply by 5 for compressed format
 *          Saturation: Clamped to UINT8_MAX (255 units = 51 m/s max)
 * 
 * @return Target velocity in m/s*5 (uint8_t, range 0-255)
 * 
 * @note Only compiled if HAL_HIGH_LATENCY2_ENABLED defined
 * @note Divide by 5 to get actual velocity in m/s
 * @note Function name says "airspeed" but calculates water velocity for Sub
 * @see HIGH_LATENCY2 message (MAV ID 235) for format specification
 * @see pos_control.get_vel_desired_NEU_cms() for desired velocity
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:1529-1536
 */
uint8_t GCS_MAVLINK_Sub::high_latency_tgt_airspeed() const
{
    // return units are m/s*5
    if (sub.control_mode == Mode::Number::AUTO || sub.control_mode == Mode::Number::GUIDED) {
        return MIN((sub.pos_control.get_vel_desired_NEU_cms().length()/100) * 5, UINT8_MAX);
    }
    return 0;
}
#endif // HAL_HIGH_LATENCY2_ENABLED

/**
 * @brief Sends AVAILABLE_MODES message for single flight mode by index
 * 
 * @details Transmits mode information to ground control station for UI mode selection.
 *          GCS queries available modes by index to build mode selection menus.
 *          
 *          ArduSub flight modes enumerated:
 *          1. Manual      - Direct thruster control
 *          2. Stabilize   - Attitude stabilization
 *          3. Acro        - Acrobatic mode
 *          4. AltHold     - Depth hold
 *          5. SurfTrak    - Surface tracking
 *          6. PosHold     - Position hold
 *          7. Auto        - Autonomous mission
 *          8. Guided      - External guidance (companion computer)
 *          9. Circle      - Circle mode
 *          10. Surface    - Surface mode
 *          11. MotorDetect - Motor configuration detection
 *          
 *          Message contents:
 *          - mode_count: Total number of available modes (11)
 *          - mode_index: Requested mode index (1-based)
 *          - standard_mode: MAV_STANDARD_MODE_NON_STANDARD (ArduPilot-specific)
 *          - custom_mode: ArduPilot mode number for this mode
 *          - properties: Mode property bitmask (currently 0)
 *          - mode_name: Human-readable mode name string
 * 
 * @param[in] index Mode index to send (1-based, not mode number!)
 * 
 * @return Total number of available modes (uint8_t)
 * 
 * @note Index is 1-based: 1 = first mode, 11 = last mode
 * @note Returns mode_count even if index out of range (for iteration)
 * @note Custom mode number != index (mode numbering separate from list position)
 * @see AVAILABLE_MODES message (MAV ID 435) in MAVLink development.xml
 * @see Mode::number() for custom mode number
 * @see Mode::name() for mode name strings
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.cpp:1541-1581
 */
uint8_t GCS_MAVLINK_Sub::send_available_mode(uint8_t index) const
{
    const Mode* modes[] {
        &sub.mode_manual,
        &sub.mode_stabilize,
        &sub.mode_acro,
        &sub.mode_althold,
        &sub.mode_surftrak,
        &sub.mode_poshold,
        &sub.mode_auto,
        &sub.mode_guided,
        &sub.mode_circle,
        &sub.mode_surface,
        &sub.mode_motordetect,
    };

    const uint8_t mode_count = ARRAY_SIZE(modes);

    // Convert to zero indexed
    const uint8_t index_zero = index - 1;
    if (index_zero >= mode_count) {
        // Mode does not exist!?
        return mode_count;
    }

    // Ask the mode for its name and number
    const char* name = modes[index_zero]->name();
    const uint8_t mode_number = (uint8_t)modes[index_zero]->number();

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
