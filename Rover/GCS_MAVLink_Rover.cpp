/**
 * @file GCS_MAVLink_Rover.cpp
 * @brief Implementation of rover-specific MAVLink command handlers and message processing
 * 
 * @details This file implements the rover-specific extensions to the MAVLink Ground Control Station
 *          interface. It handles incoming MAVLink commands and messages specific to ground vehicles,
 *          boats, and other rover platforms, and provides telemetry output tailored for rover operation.
 *          
 *          Key responsibilities:
 *          - Process rover-specific MAVLink commands (DO_CHANGE_SPEED, DO_SET_REVERSE, DO_REPOSITION, etc.)
 *          - Handle guided mode position/velocity/attitude target messages
 *          - Send rover-specific telemetry (position targets, navigation controller output, servo outputs)
 *          - Coordinate frame transformations for guided mode commands
 *          - Vehicle capability reporting for rover platforms
 *          
 * @note Coordinate Frame Conventions:
 *       - NED (North-East-Down): Earth-fixed frame with X=North, Y=East, Z=Down
 *       - Body Frame: Vehicle-fixed frame with X=forward, Y=right, Z=down
 *       - Global Frames: Latitude/longitude with various altitude references (MSL, terrain, etc.)
 *       
 * @note Unit Conventions:
 *       - Angles: Degrees in MAVLink messages, centidegrees (deg*100) internally
 *       - Speed: m/s in MAVLink messages and internal calculations
 *       - Turn rates: deg/s in MAVLink, centideg/s internally
 *       - Position: Latitude/longitude as 1e7 integers, altitude as float meters
 *       
 * @see GCS_MAVLink_Rover.h for class declarations
 * @see libraries/GCS_MAVLink/GCS.h for base MAVLink implementation
 * 
 * Source: Rover/GCS_MAVLink_Rover.cpp
 */

#include "Rover.h"

#include "GCS_MAVLink_Rover.h"

#include <AP_RPM/AP_RPM_config.h>
#include <AP_RangeFinder/AP_RangeFinder_Backend.h>
#include <AP_EFI/AP_EFI_config.h>
#include <AC_Avoidance/AP_OADatabase.h>

MAV_TYPE GCS_Rover::frame_type() const
{
    if (rover.is_boat()) {
        return MAV_TYPE_SURFACE_BOAT;
    }
    return MAV_TYPE_GROUND_ROVER;
}

uint8_t GCS_MAVLINK_Rover::base_mode() const
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
    if (rover.control_mode->has_manual_input()) {
        _base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    }

    if (rover.control_mode->is_autopilot_mode()) {
        _base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
    }

    if (rover.g2.stick_mixing > 0 && rover.control_mode != &rover.mode_initializing) {
        // all modes except INITIALISING have some form of manual
        // override if stick mixing is enabled
        _base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    }

    // we are armed if we are not initialising
    if (rover.control_mode != &rover.mode_initializing && rover.arming.is_armed()) {
        _base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // indicate we have set a custom mode
    _base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    return _base_mode;
}

uint32_t GCS_Rover::custom_mode() const
{
    return (uint32_t)rover.control_mode->mode_number();
}

MAV_STATE GCS_MAVLINK_Rover::vehicle_system_status() const
{
    if ((rover.failsafe.triggered != 0) || rover.failsafe.ekf) {
        return MAV_STATE_CRITICAL;
    }
    if (rover.control_mode == &rover.mode_initializing) {
        return MAV_STATE_CALIBRATING;
    }
    if (rover.control_mode == &rover.mode_hold) {
        return MAV_STATE_STANDBY;
    }

    return MAV_STATE_ACTIVE;
}

/**
 * @brief Send current rover position target to ground control station
 * 
 * @details Transmits the POSITION_TARGET_GLOBAL_INT MAVLink message containing the rover's
 *          current desired/target position as determined by the active flight mode. This allows
 *          the GCS to display where the rover is trying to navigate to.
 *          
 *          The message only includes position (lat/lon/alt) fields; velocity, acceleration, and
 *          attitude fields are masked as ignored since rovers typically navigate to waypoints
 *          rather than following velocity vectors.
 *          
 *          Coordinate Frame: MAV_FRAME_GLOBAL (global latitude/longitude with MSL altitude)
 *          
 *          Type Mask Behavior:
 *          - Bits set to 1 indicate that field should be IGNORED by receiver
 *          - Only position (lat/lon/alt) fields are valid (mask bits = 0)
 *          - All velocity, acceleration, yaw, and yaw rate fields are masked as ignored (bits = 1)
 *          
 * @note Only sends if the current mode has a defined target location (e.g., Auto, Guided, RTL)
 * @note Altitude is converted from centimeters (internal) to meters (MAVLink) via 0.01f scaling
 * @note Latitude and longitude are sent as 1e7 integers (degrees * 10^7)
 * 
 * @warning This function returns silently if the current mode does not have a target location
 * 
 * Source: Rover/GCS_MAVLink_Rover.cpp:75-101
 */
void GCS_MAVLINK_Rover::send_position_target_global_int()
{
    // Get the desired location from the current control mode
    Location target;
    if (!rover.control_mode->get_desired_location(target)) {
        // Current mode doesn't have a target location (e.g., Manual, Hold)
        return;
    }
    
    // Type mask configuration: Set bits indicate fields to IGNORE
    // Rover only uses position fields; velocity, acceleration, and yaw are not used
    static constexpr uint16_t POSITION_TARGET_TYPEMASK_LAST_BYTE = 0xF000;
    static constexpr uint16_t TYPE_MASK = POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                                          POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                                          POSITION_TARGET_TYPEMASK_YAW_IGNORE | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE | POSITION_TARGET_TYPEMASK_LAST_BYTE;
    
    // Send POSITION_TARGET_GLOBAL_INT message with only position fields populated
    mavlink_msg_position_target_global_int_send(
        chan,
        AP_HAL::millis(), // time_boot_ms - timestamp in milliseconds since system boot
        MAV_FRAME_GLOBAL, // targets are always global altitude (MSL reference)
        TYPE_MASK, // ignore everything except the lat/lon/alt components
        target.lat, // latitude as 1e7 (degrees * 10^7)
        target.lng, // longitude as 1e7 (degrees * 10^7)
        target.alt * 0.01f, // altitude is sent as a float in meters (converted from centimeters)
        0.0f, // vx - North velocity in m/s (ignored, set to 0)
        0.0f, // vy - East velocity in m/s (ignored, set to 0)
        0.0f, // vz - Down velocity in m/s (ignored, set to 0)
        0.0f, // afx - North acceleration in m/s² (ignored, set to 0)
        0.0f, // afy - East acceleration in m/s² (ignored, set to 0)
        0.0f, // afz - Down acceleration in m/s² (ignored, set to 0)
        0.0f, // yaw - Yaw angle in radians (ignored, set to 0)
        0.0f); // yaw_rate - Yaw rate in rad/s (ignored, set to 0)
}

void GCS_MAVLINK_Rover::send_nav_controller_output() const
{
    if (!rover.control_mode->is_autopilot_mode()) {
        return;
    }

    const Mode *control_mode = rover.control_mode;

    mavlink_msg_nav_controller_output_send(
        chan,
        0,  // roll
        degrees(rover.g2.attitude_control.get_desired_pitch()),
        control_mode->nav_bearing(),
        control_mode->wp_bearing(),
        MIN(control_mode->get_distance_to_destination(), UINT16_MAX),
        0,
        control_mode->speed_error(),
        control_mode->crosstrack_error());
}

void GCS_MAVLINK_Rover::send_servo_out()
{
    float motor1, motor3;
    if (rover.g2.motors.have_skid_steering()) {
        motor1 = 10000 * (SRV_Channels::get_output_scaled(SRV_Channel::k_throttleLeft) * 0.001f);
        motor3 = 10000 * (SRV_Channels::get_output_scaled(SRV_Channel::k_throttleRight) * 0.001f);
    } else {
        motor1 = 10000 * (SRV_Channels::get_output_scaled(SRV_Channel::k_steering) / 4500.0f);
        motor3 = 10000 * (SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) * 0.01f);
    }
    mavlink_msg_rc_channels_scaled_send(
        chan,
        millis(),
        0,  // port 0
        motor1,
        0,
        motor3,
        0,
        0,
        0,
        0,
        0,
#if AP_RSSI_ENABLED
        receiver_rssi()
#else
        255
#endif
        );
}

int16_t GCS_MAVLINK_Rover::vfr_hud_throttle() const
{
    return rover.g2.motors.get_throttle();
}

#if AP_MAVLINK_MSG_RANGEFINDER_SENDING_ENABLED
void GCS_MAVLINK_Rover::send_rangefinder() const
{
    float distance = 0;
    float voltage = 0;
    bool got_one = false;

    // report smaller distance of all rangefinders
    for (uint8_t i=0; i<rover.rangefinder.num_sensors(); i++) {
        AP_RangeFinder_Backend *s = rover.rangefinder.get_backend(i);
        if (s == nullptr) {
            continue;
        }
        if (!got_one ||
            s->distance() < distance) {
            distance = s->distance();
            voltage = s->voltage_mv();
            got_one = true;
        }
    }
    if (!got_one) {
        // no relevant data found
        return;
    }

    mavlink_msg_rangefinder_send(
        chan,
        distance,
        voltage);
}
#endif  // AP_MAVLINK_MSG_RANGEFINDER_SENDING_ENABLED

#if AP_RANGEFINDER_ENABLED
void GCS_MAVLINK_Rover::send_water_depth()
{
    if (!HAVE_PAYLOAD_SPACE(chan, WATER_DEPTH)) {
        return;
    }

    // only send for boats:
    if (!rover.is_boat()) {
        return;
    }

    RangeFinder *rangefinder = RangeFinder::get_singleton();

    if (rangefinder == nullptr) {
        return;
    }

    // depth can only be measured by a downward-facing rangefinder:
    if (!rangefinder->has_orientation(ROTATION_PITCH_270)) {
        return;
    }

    // get position
    const AP_AHRS &ahrs = AP::ahrs();
    Location loc;
    IGNORE_RETURN(ahrs.get_location(loc));

    const auto num_sensors = rangefinder->num_sensors();
    for (uint8_t i=0; i<num_sensors; i++) {
        last_WATER_DEPTH_index += 1;
        if (last_WATER_DEPTH_index >= num_sensors) {
            last_WATER_DEPTH_index = 0;
        }

        const AP_RangeFinder_Backend *s = rangefinder->get_backend(last_WATER_DEPTH_index);

        if (s == nullptr || s->orientation() != ROTATION_PITCH_270 || !s->has_data()) {
            continue;
        }

        // get temperature
        float temp_C;
        if (!s->get_temp(temp_C)) {
            temp_C = 0.0f;
        }

        const bool sensor_healthy = (s->status() == RangeFinder::Status::Good);

        mavlink_msg_water_depth_send(
            chan,
            AP_HAL::millis(),   // time since system boot TODO: take time of measurement
            last_WATER_DEPTH_index, // rangefinder instance
            sensor_healthy,     // sensor healthy
            loc.lat,            // latitude of vehicle
            loc.lng,            // longitude of vehicle
            loc.alt * 0.01f,    // altitude of vehicle (MSL)
            ahrs.get_roll_rad(),    // roll in radians
            ahrs.get_pitch_rad(),   // pitch in radians
            ahrs.get_yaw_rad(),     // yaw in radians
            s->distance(),    // distance in meters
            temp_C);            // temperature in degC

        break;  // only send one WATER_DEPTH message per loop
    }

}
#endif  // AP_RANGEFINDER_ENABLED

/*
  send PID tuning message
 */
void GCS_MAVLINK_Rover::send_pid_tuning()
{
    Parameters &g = rover.g;
    ParametersG2 &g2 = rover.g2;

    const AP_PIDInfo *pid_info;

    // steering PID
    if (g.gcs_pid_mask & 1) {
        pid_info = &g2.attitude_control.get_steering_rate_pid().get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_STEER,
                                    degrees(pid_info->target),
                                    degrees(pid_info->actual),
                                    pid_info->FF,
                                    pid_info->P,
                                    pid_info->I,
                                    pid_info->D,
                                    pid_info->slew_rate,
                                    pid_info->Dmod);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }

    // speed to throttle PID
    if (g.gcs_pid_mask & 2) {
        pid_info = &g2.attitude_control.get_throttle_speed_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_ACCZ,
                                    pid_info->target,
                                    pid_info->actual,
                                    pid_info->FF,
                                    pid_info->P,
                                    pid_info->I,
                                    pid_info->D,
                                    pid_info->slew_rate,
                                    pid_info->Dmod);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }

    // pitch to throttle pid
    if (g.gcs_pid_mask & 4) {
        pid_info = &g2.attitude_control.get_pitch_to_throttle_pid().get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_PITCH,
                                    degrees(pid_info->target),
                                    degrees(pid_info->actual),
                                    pid_info->FF,
                                    pid_info->P,
                                    pid_info->I,
                                    pid_info->D,
                                    pid_info->slew_rate,
                                    pid_info->Dmod);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }

    // left wheel rate control pid
    if (g.gcs_pid_mask & 8) {
        pid_info = &g2.wheel_rate_control.get_pid(0).get_pid_info();
        mavlink_msg_pid_tuning_send(chan, 7,
                                    pid_info->target,
                                    pid_info->actual,
                                    pid_info->FF,
                                    pid_info->P,
                                    pid_info->I,
                                    pid_info->D,
                                    pid_info->slew_rate,
                                    pid_info->Dmod);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }

    // right wheel rate control pid
    if (g.gcs_pid_mask & 16) {
        pid_info = &g2.wheel_rate_control.get_pid(1).get_pid_info();
        mavlink_msg_pid_tuning_send(chan, 8,
                                    pid_info->target,
                                    pid_info->actual,
                                    pid_info->FF,
                                    pid_info->P,
                                    pid_info->I,
                                    pid_info->D,
                                    pid_info->slew_rate,
                                    pid_info->Dmod);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }

    // sailboat heel to mainsail pid
    if (g.gcs_pid_mask & 32) {
        pid_info = &g2.attitude_control.get_sailboat_heel_pid().get_pid_info();
        mavlink_msg_pid_tuning_send(chan, 9,
                                    pid_info->target,
                                    pid_info->actual,
                                    pid_info->FF,
                                    pid_info->P,
                                    pid_info->I,
                                    pid_info->D,
                                    pid_info->slew_rate,
                                    pid_info->Dmod);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }

    // Position Controller Velocity North PID
    if (g.gcs_pid_mask & 64) {
        pid_info = &g2.pos_control.get_vel_pid().get_pid_info_x();
        mavlink_msg_pid_tuning_send(chan, 10,
                                    pid_info->target,
                                    pid_info->actual,
                                    pid_info->FF,
                                    pid_info->P,
                                    pid_info->I,
                                    pid_info->D,
                                    pid_info->slew_rate,
                                    pid_info->Dmod);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }

    // Position Controller Velocity East PID
    if (g.gcs_pid_mask & 128) {
        pid_info = &g2.pos_control.get_vel_pid().get_pid_info_y();
        mavlink_msg_pid_tuning_send(chan, 11,
                                    pid_info->target,
                                    pid_info->actual,
                                    pid_info->FF,
                                    pid_info->P,
                                    pid_info->I,
                                    pid_info->D,
                                    pid_info->slew_rate,
                                    pid_info->Dmod);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
}

void Rover::send_wheel_encoder_distance(const mavlink_channel_t chan)
{
    // send wheel encoder data using wheel_distance message
    if (g2.wheel_encoder.num_sensors() > 0) {
        double distances[MAVLINK_MSG_WHEEL_DISTANCE_FIELD_DISTANCE_LEN] {};
        for (uint8_t i = 0; i < g2.wheel_encoder.num_sensors(); i++) {
            distances[i] = wheel_encoder_last_distance_m[i];
        }
        mavlink_msg_wheel_distance_send(chan, 1000UL * AP_HAL::millis(), g2.wheel_encoder.num_sensors(), distances);
    }
}

bool GCS_Rover::vehicle_initialised() const
{
    return rover.control_mode != &rover.mode_initializing;
}

// try to send a message, return false if it won't fit in the serial tx buffer
bool GCS_MAVLINK_Rover::try_send_message(enum ap_message id)
{
    switch (id) {

    case MSG_SERVO_OUT:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_SCALED);
        send_servo_out();
        break;

    case MSG_WHEEL_DISTANCE:
        CHECK_PAYLOAD_SIZE(WHEEL_DISTANCE);
        rover.send_wheel_encoder_distance(chan);
        break;

    case MSG_WIND:
        CHECK_PAYLOAD_SIZE(WIND);
        rover.g2.windvane.send_wind(chan);
        break;

#if AP_OADATABASE_ENABLED
    case MSG_ADSB_VEHICLE: {
        AP_OADatabase *oadb = AP::oadatabase();
        if (oadb != nullptr) {
            CHECK_PAYLOAD_SIZE(ADSB_VEHICLE);
            uint16_t interval_ms = 0;
            if (get_ap_message_interval(id, interval_ms)) {
                oadb->send_adsb_vehicle(chan, interval_ms);
            }
        }
        break;
    }
#endif

#if AP_RANGEFINDER_ENABLED
    case MSG_WATER_DEPTH:
        CHECK_PAYLOAD_SIZE(WATER_DEPTH);
        send_water_depth();
        break;
#endif  // AP_RANGEFINDER_ENABLED

    default:
        return GCS_MAVLINK::try_send_message(id);
    }
    return true;
}
bool GCS_MAVLINK_Rover::handle_guided_request(AP_Mission::Mission_Command &cmd)
{
    if (!rover.control_mode->in_guided_mode()) {
        // only accept position updates when in GUIDED mode
        return false;
    }

    // make any new wp uploaded instant (in case we are already in Guided mode)
    return rover.mode_guided.set_desired_location(cmd.content.location);
}

MAV_RESULT GCS_MAVLINK_Rover::_handle_command_preflight_calibration(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    switch (packet.y) { 
    case 1:
        if (rover.g2.windvane.start_direction_calibration()) {
            return MAV_RESULT_ACCEPTED;
        } else {
            return MAV_RESULT_FAILED;
        }

    case 2:
        if (rover.g2.windvane.start_speed_calibration()) {
            return MAV_RESULT_ACCEPTED;
        } else {
            return MAV_RESULT_FAILED;
        }

    default:
        break;

    }

    return GCS_MAVLINK::_handle_command_preflight_calibration(packet, msg);
}

/**
 * @brief Handle rover-specific MAVLink COMMAND_INT messages
 * 
 * @details Processes MAVLink commands sent via COMMAND_INT messages (commands with integer position fields).
 *          This function extends the base GCS_MAVLink command handler to implement rover-specific behaviors
 *          for commands related to speed control, positioning, motor testing, and mission execution.
 *          
 *          Rover-Specific Commands Handled:
 *          - MAV_CMD_DO_CHANGE_SPEED: Change target ground speed
 *          - MAV_CMD_DO_REPOSITION: Navigate to a new position in Guided mode
 *          - MAV_CMD_DO_SET_REVERSE: Set forward/reverse direction
 *          - MAV_CMD_NAV_RETURN_TO_LAUNCH: Enter RTL mode
 *          - MAV_CMD_DO_MOTOR_TEST: Test individual motors
 *          - MAV_CMD_MISSION_START: Start autonomous mission (Auto mode)
 *          - MAV_CMD_NAV_SET_YAW_SPEED: Set heading and speed in Guided mode
 *          
 * @param[in] packet The COMMAND_INT packet containing command ID and parameters
 * @param[in] msg The complete MAVLink message (for base class processing)
 * 
 * @return MAV_RESULT indicating command execution status:
 *         - MAV_RESULT_ACCEPTED: Command executed successfully
 *         - MAV_RESULT_DENIED: Command not applicable in current state
 *         - MAV_RESULT_FAILED: Command execution failed
 *         - Other: Handled by base class for unsupported commands
 *         
 * @note All commands forwarded to base class if not handled by rover implementation
 * @note Speed units are in m/s, angles in degrees, positions as 1e7 lat/lon integers
 * 
 * @warning Some commands (e.g., DO_REPOSITION) may change vehicle mode without explicit mode change command
 * 
 * Source: Rover/GCS_MAVLink_Rover.cpp:505-571
 */
MAV_RESULT GCS_MAVLINK_Rover::handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    switch (packet.command) {

    case MAV_CMD_DO_CHANGE_SPEED:
        /**
         * Change the rover's target speed
         * 
         * param1 : Speed type (SPEED_TYPE enum)
         *          - SPEED_TYPE_AIRSPEED: Treated as ground speed for rover (GCS compatibility)
         *          - SPEED_TYPE_GROUNDSPEED: Ground speed in m/s
         *          - SPEED_TYPE_CLIMB_SPEED: Not supported for rovers (denied)
         *          - SPEED_TYPE_DESCENT_SPEED: Not supported for rovers (denied)
         * param2 : New target speed in m/s (positive = forward, negative = reverse)
         * 
         * For rovers, both AIRSPEED and GROUNDSPEED are interpreted as ground speed since
         * rovers don't have meaningful airspeed. This provides compatibility with GCS software
         * designed for aircraft.
         * 
         * The speed change applies to the current control mode and persists until changed again
         * or the mode is switched. Not all modes support speed changes.
         */
        switch (SPEED_TYPE(packet.param1)) {
            case SPEED_TYPE_CLIMB_SPEED:
            case SPEED_TYPE_DESCENT_SPEED:
            case SPEED_TYPE_ENUM_END:
                // Climb/descent speeds don't apply to ground vehicles
                return MAV_RESULT_DENIED;

            case SPEED_TYPE_AIRSPEED: // Airspeed is treated as ground speed for GCS compatibility
            case SPEED_TYPE_GROUNDSPEED:
                // Valid speed types for rovers
                break;
        }
        // Attempt to set the desired speed in the current control mode
        if (!rover.control_mode->set_desired_speed(packet.param2)) {
            // Mode doesn't support speed changes or speed value is invalid
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_DO_REPOSITION:
        /**
         * Command rover to navigate to a new position
         * 
         * param1 : Target speed in m/s (optional, -1 = use default)
         * param2 : Bitmask of options (MAV_DO_REPOSITION_FLAGS)
         *          - Bit 0 (MAV_DO_REPOSITION_FLAGS_CHANGE_MODE): Switch to Guided mode if not already
         * x : Target latitude (degrees * 1e7)
         * y : Target longitude (degrees * 1e7)
         * z : Target altitude (meters, frame-dependent)
         * 
         * This command sends the rover to a specific global position. If the rover is not in
         * Guided mode and the CHANGE_MODE flag is set, it will automatically switch to Guided mode.
         * If the flag is not set and the rover is not in Guided mode, the command is denied.
         * 
         * The rover will navigate to the specified location using its current navigation algorithm
         * (typically following a path that avoids obstacles and respects turn radius constraints).
         */
        return handle_command_int_do_reposition(packet);

    case MAV_CMD_DO_SET_REVERSE:
        /**
         * Set rover travel direction (forward or reverse)
         * 
         * param1 : Direction (0 = Forward, 1 = Reverse)
         * 
         * This command instructs the rover to drive in reverse (backwards) or return to forward
         * motion. The rover will maintain its current speed but reverse the direction of travel.
         * This is useful for backing out of tight spaces or reversing along a path.
         * 
         * Not all modes support reversing. The command will be accepted but may have no effect
         * in modes that don't implement reverse behavior.
         */
        // param1 : Direction (0=Forward, 1=Reverse)
        rover.control_mode->set_reversed(is_equal(packet.param1,1.0f));
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        /**
         * Command rover to return to launch position (RTL mode)
         * 
         * No parameters used
         * 
         * Switches the rover to RTL (Return To Launch) mode, which navigates back to the
         * location where the vehicle was armed. The rover will follow the most direct path
         * considering obstacles and terrain, or may use SmartRTL path if configured.
         * 
         * If mode change fails (e.g., due to pre-arm checks or system state), MAV_RESULT_FAILED
         * is returned. Otherwise, the mode change is accepted and rover begins RTL navigation.
         */
        if (rover.set_mode(rover.mode_rtl, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;

    case MAV_CMD_DO_MOTOR_TEST:
        /**
         * Test individual rover motors/servos
         * 
         * param1 : Motor sequence number (1 to max number of motors on vehicle)
         *          For rovers: 1 = left motor, 2 = right motor, 3 = throttle, 4 = steering
         * param2 : Throttle type (MOTOR_TEST_THROTTLE_TYPE enum)
         *          - 0 = Throttle percentage (0-100)
         *          - 1 = PWM (typically 1000-2000 µs)
         *          - 2 = Pilot throttle channel pass-through
         * param3 : Throttle value (range depends on param2 type)
         * param4 : Timeout in seconds (test will run for this duration)
         * 
         * Allows testing of individual motor outputs for diagnostic purposes. This is typically
         * used during setup and calibration to verify motor connections and directions.
         * 
         * @warning Vehicle must be disarmed and in a safe configuration before motor testing
         * @warning Ensure vehicle is secured and cannot move during motor tests
         */
        // param1 : motor sequence number (a number from 1 to max number of motors on the vehicle)
        // param2 : throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)
        // param3 : throttle (range depends upon param2)
        // param4 : timeout (in seconds)
        return rover.mavlink_motor_test_start(*this,
                                              (AP_MotorsUGV::motor_test_order)packet.param1,
                                              static_cast<uint8_t>(packet.param2),
                                              static_cast<int16_t>(packet.param3),
                                              packet.param4);

    case MAV_CMD_MISSION_START:
        /**
         * Start autonomous mission execution (Auto mode)
         * 
         * param1 : First mission item to run (0 = start from beginning, not currently supported)
         * param2 : Last mission item to run (0 = run to end, not currently supported)
         * 
         * Switches the rover to Auto mode and begins executing the pre-loaded mission from the
         * beginning. The rover will navigate through all mission waypoints in sequence until
         * the mission completes or is interrupted.
         * 
         * Currently, selective mission execution (starting/ending at specific items) is not
         * supported. param1 and param2 must both be zero for the command to be accepted.
         * 
         * @note Mission must be uploaded and validated before this command will succeed
         */
        if (!is_zero(packet.param1) || !is_zero(packet.param2)) {
            // first-item/last item selective execution not supported for rovers
            return MAV_RESULT_DENIED;
        }
        if (rover.set_mode(rover.mode_auto, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_ACCEPTED;
        }
        // Mode change failed (likely due to mission validation or arming checks)
        return MAV_RESULT_FAILED;

#if AP_MAVLINK_MAV_CMD_NAV_SET_YAW_SPEED_ENABLED
    case MAV_CMD_NAV_SET_YAW_SPEED:
        /**
         * Set rover heading and speed in Guided mode (DEPRECATED)
         * 
         * @deprecated This command is deprecated. Use SET_POSITION_TARGET_GLOBAL_INT or 
         *             SET_POSITION_TARGET_LOCAL_NED messages for guided mode control instead.
         * 
         * param1 : Target yaw angle in degrees (absolute or relative based on param3)
         * param2 : Target speed in m/s
         * param3 : Angle mode (0 = absolute North-referenced, 1 = relative to current heading)
         * 
         * Commands the rover to turn to a specific heading and travel at the specified speed.
         * Only works when rover is in Guided mode.
         */
        send_received_message_deprecation_warning("MAV_CMD_NAV_SET_YAW_SPEED");
        return handle_command_nav_set_yaw_speed(packet, msg);
#endif

    default:
        return GCS_MAVLINK::handle_command_int_packet(packet, msg);
    }
}

#if AP_MAVLINK_MAV_CMD_NAV_SET_YAW_SPEED_ENABLED
MAV_RESULT GCS_MAVLINK_Rover::handle_command_nav_set_yaw_speed(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
        // param1 : yaw angle (may be absolute or relative)
        // param2 : Speed - in metres/second
        // param3 : 0 = param1 is absolute, 1 = param1 is relative

        // exit if vehicle is not in Guided mode
        if (!rover.control_mode->in_guided_mode()) {
            return MAV_RESULT_FAILED;
        }

        // get final angle, 1 = Relative, 0 = Absolute
        if (packet.param3 > 0) {
            // relative angle
            rover.mode_guided.set_desired_heading_delta_and_speed(packet.param1 * 100.0f, packet.param2);
        } else {
            // absolute angle
            rover.mode_guided.set_desired_heading_and_speed(packet.param1 * 100.0f, packet.param2);
        }
        return MAV_RESULT_ACCEPTED;
}
#endif

MAV_RESULT GCS_MAVLINK_Rover::handle_command_int_do_reposition(const mavlink_command_int_t &packet)
{
    const bool change_modes = ((int32_t)packet.param2 & MAV_DO_REPOSITION_FLAGS_CHANGE_MODE) == MAV_DO_REPOSITION_FLAGS_CHANGE_MODE;
    if (!rover.control_mode->in_guided_mode() && !change_modes) {
        return MAV_RESULT_DENIED;
    }

    // sanity check location
    if (!check_latlng(packet.x, packet.y)) {
        return MAV_RESULT_DENIED;
    }
    if (packet.x == 0 && packet.y == 0) {
        return MAV_RESULT_DENIED;
    }

    Location requested_location {};
    if (!location_from_command_t(packet, requested_location)) {
        return MAV_RESULT_DENIED;
    }

    if (!rover.control_mode->in_guided_mode()) {
        if (!rover.set_mode(Mode::Number::GUIDED, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_FAILED;
        }
    }

    if (is_positive(packet.param1)) {
        if (!rover.control_mode->set_desired_speed(packet.param1)) {
            return MAV_RESULT_FAILED;
        }
    }

    // set the destination
    if (!rover.mode_guided.set_desired_location(requested_location)) {
        return MAV_RESULT_FAILED;
    }

    return MAV_RESULT_ACCEPTED;
}

/**
 * @brief Dispatch rover-specific MAVLink messages to appropriate handlers
 * 
 * @details Routes incoming MAVLink messages to rover-specific message handlers. This function
 *          is called for every MAVLink message received on this channel and provides the entry
 *          point for rover-specific guided mode control messages.
 *          
 *          Rover-Specific Messages Handled:
 *          - SET_ATTITUDE_TARGET: Direct attitude and thrust control in Guided mode
 *          - SET_POSITION_TARGET_LOCAL_NED: Position/velocity targets in NED frame
 *          - SET_POSITION_TARGET_GLOBAL_INT: Position/velocity targets in global (lat/lon) frame
 *          
 *          These messages enable external systems (GCS, companion computers, offboard controllers)
 *          to command the rover's motion in Guided mode with precise control over position,
 *          velocity, or attitude.
 *          
 * @param[in] msg The received MAVLink message to be processed
 * 
 * @note Messages not handled by rover-specific code are forwarded to base class GCS_MAVLINK::handle_message()
 * @note All guided mode messages require the rover to be in Guided mode to take effect
 * @note Coordinate frames and type masks must be properly configured in messages
 * 
 * Source: Rover/GCS_MAVLink_Rover.cpp:637-657
 */
void GCS_MAVLINK_Rover::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {

    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
        // Direct attitude (heading) and thrust (speed) control for Guided mode
        handle_set_attitude_target(msg);
        break;

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
        // Position/velocity targets in local NED or body-fixed frames
        handle_set_position_target_local_ned(msg);
        break;

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:
        // Position/velocity targets in global lat/lon coordinates
        handle_set_position_target_global_int(msg);
        break;

    default:
        // Forward unhandled messages to base class implementation
        GCS_MAVLINK::handle_message(msg);
        break;
    }
}

void GCS_MAVLINK_Rover::handle_manual_control_axes(const mavlink_manual_control_t &packet, const uint32_t tnow)
{
    manual_override(rover.channel_steer, packet.y, 1000, 2000, tnow);
    manual_override(rover.channel_throttle, packet.z, 1000, 2000, tnow);
}

void GCS_MAVLINK_Rover::handle_set_attitude_target(const mavlink_message_t &msg)
{
    // decode packet
    mavlink_set_attitude_target_t packet;
    mavlink_msg_set_attitude_target_decode(&msg, &packet);

    // exit if vehicle is not in Guided mode
    if (!rover.control_mode->in_guided_mode()) {
        return;
    }

    // ensure type_mask specifies to use thrust
    if ((packet.type_mask & MAVLINK_SET_ATT_TYPE_MASK_THROTTLE_IGNORE) != 0) {
        return;
    }

    // convert thrust to ground speed
    packet.thrust = constrain_float(packet.thrust, -1.0f, 1.0f);
    const float target_speed = rover.control_mode->get_speed_default() * packet.thrust;

    // if the body_yaw_rate field is ignored, convert quaternion to heading
    if ((packet.type_mask & MAVLINK_SET_ATT_TYPE_MASK_YAW_RATE_IGNORE) != 0) {
        // convert quaternion to heading
        float target_heading_cd = degrees(Quaternion(packet.q[0], packet.q[1], packet.q[2], packet.q[3]).get_euler_yaw()) * 100.0f;
        rover.mode_guided.set_desired_heading_and_speed(target_heading_cd, target_speed);
    } else {
        // use body_yaw_rate field
        rover.mode_guided.set_desired_turn_rate_and_speed((RAD_TO_DEG * packet.body_yaw_rate) * 100.0f, target_speed);
    }
}

// if we receive a message where the user has not masked out
// acceleration from the input packet we send a curt message
// informing them:
void GCS_MAVLINK_Rover::send_acc_ignore_must_be_set_message(const char *msgname)
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Ignoring %s; set ACC_IGNORE in mask", msgname);
}

/**
 * @brief Handle position/velocity target commands in local NED or body-fixed frames
 * 
 * @details Processes SET_POSITION_TARGET_LOCAL_NED messages for guided mode control using local
 *          or body-relative coordinate frames. This message provides flexible control options:
 *          position-only, velocity-only, or combined position+velocity+yaw commands.
 *          
 *          Supported Coordinate Frames:
 *          - MAV_FRAME_LOCAL_NED: Relative to EKF origin (North-East-Down)
 *          - MAV_FRAME_LOCAL_OFFSET_NED: Offset from current vehicle position in NED
 *          - MAV_FRAME_BODY_NED: Relative to vehicle body frame (forward-right-down)
 *          - MAV_FRAME_BODY_OFFSET_NED: Offset from current position in body frame
 *          
 *          Type Mask Usage (bits set to 1 = IGNORE that field):
 *          - POSITION_IGNORE: Ignore position fields (x, y)
 *          - VEL_IGNORE: Ignore velocity fields (vx, vy)
 *          - ACC_IGNORE: Must be set (acceleration control not supported)
 *          - YAW_IGNORE: Ignore yaw field
 *          - YAW_RATE_IGNORE: Ignore yaw_rate field
 *          
 *          Control Modes Based on Type Mask:
 *          1. Position-only: Set target location (pos_ignore=0, vel_ignore=1)
 *          2. Velocity+Heading: Set speed and direction (pos_ignore=1, vel_ignore=0, yaw_ignore=0)
 *          3. Velocity+TurnRate: Set speed and turn rate (pos_ignore=1, vel_ignore=0, yaw_rate_ignore=0)
 *          4. Heading-only: Set target heading at zero speed (skid-steer only)
 *          5. Turn rate-only: Set turn rate at zero speed (skid-steer only)
 *          
 * @param[in] msg The MAVLink message containing position/velocity target data
 * 
 * @note Only active when rover is in Guided mode
 * @note Requires valid EKF origin for MAV_FRAME_LOCAL_NED
 * @note Acceleration fields are not supported and must be masked as ignored
 * @note Body-frame commands are rotated to NED frame using current vehicle heading
 * @note Velocity magnitude is constrained to vehicle's maximum configured speed
 * 
 * @warning Acceleration control (ACC_IGNORE=0) will cause command to be rejected
 * @warning Invalid coordinate frames are silently ignored
 * 
 * Source: Rover/GCS_MAVLink_Rover.cpp:704-841
 */
void GCS_MAVLINK_Rover::handle_set_position_target_local_ned(const mavlink_message_t &msg)
{
    // decode packet
    mavlink_set_position_target_local_ned_t packet;
    mavlink_msg_set_position_target_local_ned_decode(&msg, &packet);

    // exit if vehicle is not in Guided mode - only process guided commands when in guided mode
    if (!rover.control_mode->in_guided_mode()) {
        return;
    }

    // Need EKF origin to interpret MAV_FRAME_LOCAL_NED coordinates
    Location ekf_origin;
    if (!rover.ahrs.get_origin(ekf_origin)) {
        return;
    }

    // Validate coordinate frame - only accept supported NED and body frames
    switch (packet.coordinate_frame) {
    case MAV_FRAME_LOCAL_NED:        // Position relative to EKF origin
    case MAV_FRAME_LOCAL_OFFSET_NED: // Offset from current position in NED
    case MAV_FRAME_BODY_NED:         // Position relative to vehicle body frame
    case MAV_FRAME_BODY_OFFSET_NED:  // Offset from current position in body frame
        break;

    default:
        // Unsupported frame - silently ignore message
        return;
    }

    // Decode type mask to determine which fields are active
    // Type mask bits set to 1 indicate field should be IGNORED
    bool pos_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
    bool vel_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
    bool acc_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;
    bool yaw_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
    bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;

    // Process position target if not ignored
    // Start with current location and apply offsets based on coordinate frame
    Location target_loc = rover.current_loc;
    if (!pos_ignore) {
        switch (packet.coordinate_frame) {
        case MAV_FRAME_BODY_NED:
        case MAV_FRAME_BODY_OFFSET_NED: {
            // Body-frame: X=forward, Y=right, Z=down
            // Rotate body-frame coordinates to NED frame using current vehicle heading
            const float ne_x = packet.x * rover.ahrs.cos_yaw() - packet.y * rover.ahrs.sin_yaw();
            const float ne_y = packet.x * rover.ahrs.sin_yaw() + packet.y * rover.ahrs.cos_yaw();
            // Apply rotated offset to current location
            target_loc.offset(ne_x, ne_y);
        }
            break;

        case MAV_FRAME_LOCAL_OFFSET_NED:
            // NED offset from current vehicle position (North=X, East=Y, Down=Z)
            target_loc.offset(packet.x, packet.y);
            break;

        case MAV_FRAME_LOCAL_NED:
        default:
            // Absolute position in NED frame relative to EKF origin
            // Start from EKF origin and apply offset
            target_loc = ekf_origin;
            target_loc.offset(packet.x, packet.y);
            break;
        }
    }

    // Initialize target motion parameters
    float target_speed = 0.0f;        // Target ground speed in m/s
    float target_yaw_cd = 0.0f;       // Target heading in centidegrees

    // Process velocity vector if not ignored
    // Velocity is provided as North (vx) and East (vy) components in m/s
    if (!vel_ignore) {
        const float speed_max = rover.control_mode->get_speed_default();
        // Calculate speed magnitude from velocity vector (Pythagoras: sqrt(vx² + vy²))
        // Constrain to configured maximum speed (supports negative for reverse)
        target_speed = constrain_float(safe_sqrt(sq(packet.vx) + sq(packet.vy)), -speed_max, speed_max);
        // Calculate heading from velocity vector direction using atan2(East, North)
        // Convert from radians to centidegrees (deg * 100)
        target_yaw_cd = degrees(atan2f(packet.vy, packet.vx)) * 100.0f;

        // If velocity provided in body frame, rotate to NED frame
        if (packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
            target_yaw_cd = wrap_180_cd(target_yaw_cd + rover.ahrs.yaw_sensor);
        }
    }

    // Process yaw (heading) if explicitly provided
    // Explicit yaw overrides heading derived from velocity vector
    if (!yaw_ignore) {
        // Convert yaw from radians to centidegrees
        target_yaw_cd = degrees(packet.yaw) * 100.0f;
        // Rotate from body frame to NED frame if necessary
        if (packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
            target_yaw_cd = wrap_180_cd(target_yaw_cd + rover.ahrs.yaw_sensor);
        }
    }
    
    // Process yaw rate (turn rate) if provided
    float target_turn_rate_cds = 0.0f; // Target turn rate in centidegrees per second
    if (!yaw_rate_ignore) {
        // Convert from radians/sec to centidegrees/sec
        target_turn_rate_cds = degrees(packet.yaw_rate) * 100.0f;
    }

    // Determine forward/reverse direction when both velocity and yaw/yaw_rate are provided
    // Default assumption is forward motion (positive speed)
    float speed_dir = 1.0f;
    if (!vel_ignore && (!yaw_ignore || !yaw_rate_ignore)) {
        // Check sign of vx (North velocity) to determine if rover should drive backwards
        // Note: Using vx even for offset frames - negative vx indicates reverse motion
        if (is_negative(packet.vx)) {
            speed_dir = -1.0f; // Reverse direction
        }
    }

    // Validate acceleration field is ignored - rovers don't support acceleration control
    if (!acc_ignore) {
        // Reject command if acceleration control was requested
        send_acc_ignore_must_be_set_message("SET_POSITION_TARGET_LOCAL_NED");
        return;
    }

    // Dispatch to appropriate guided mode control based on which fields are provided
    // Priority: Position > Velocity+Heading > Velocity+TurnRate > Heading-only > TurnRate-only
    
    if (!pos_ignore) {
        // Position control mode: Navigate to target location
        // This is highest priority - if position provided, use position control
        if (!rover.mode_guided.set_desired_location(target_loc)) {
            // Failed to set location (e.g., unreachable)
            // GCS should monitor actual desired location to see if command took effect
        }
        return; // Position control is mutually exclusive with velocity control
    }

    // Velocity control modes (only reached if position is ignored)
    if (!vel_ignore && yaw_ignore && yaw_rate_ignore) {
        // Velocity-only: Drive at target speed using heading derived from velocity vector
        rover.mode_guided.set_desired_heading_and_speed(target_yaw_cd, speed_dir * target_speed);
    } else if (!vel_ignore && yaw_ignore && !yaw_rate_ignore) {
        // Velocity + turn rate: Drive at target speed while turning at specified rate
        rover.mode_guided.set_desired_turn_rate_and_speed(target_turn_rate_cds, speed_dir * target_speed);
    } else if (!vel_ignore && !yaw_ignore && yaw_rate_ignore) {
        // Velocity + heading: Drive at target speed toward specified heading
        rover.mode_guided.set_desired_heading_and_speed(target_yaw_cd, speed_dir * target_speed);
    } else if (vel_ignore && !yaw_ignore && yaw_rate_ignore) {
        // Heading-only: Turn to face specified heading at zero speed
        // Note: Typically only skid-steer vehicles can turn in place
        rover.mode_guided.set_desired_heading_and_speed(target_yaw_cd, 0.0f);
    } else if (vel_ignore && yaw_ignore && !yaw_rate_ignore) {
        // Turn rate-only: Turn at specified rate at zero speed
        // Note: Typically only skid-steer vehicles can turn in place
        rover.mode_guided.set_desired_turn_rate_and_speed(target_turn_rate_cds, 0.0f);
    }
}

/**
 * @brief Handle position/velocity target commands in global latitude/longitude coordinates
 * 
 * @details Processes SET_POSITION_TARGET_GLOBAL_INT messages for guided mode control using global
 *          coordinate frames (latitude/longitude). This provides similar control to the local NED
 *          version but uses GPS coordinates instead of relative positioning.
 *          
 *          Supported Coordinate Frames:
 *          - MAV_FRAME_GLOBAL: Global lat/lon with altitude relative to MSL (mean sea level)
 *          - MAV_FRAME_GLOBAL_INT: Same as GLOBAL (integer lat/lon fields)
 *          - MAV_FRAME_GLOBAL_RELATIVE_ALT: Global lat/lon with altitude relative to home
 *          - MAV_FRAME_GLOBAL_RELATIVE_ALT_INT: Same as GLOBAL_RELATIVE_ALT (integer fields)
 *          - MAV_FRAME_GLOBAL_TERRAIN_ALT: Global lat/lon with altitude relative to terrain
 *          - MAV_FRAME_GLOBAL_TERRAIN_ALT_INT: Same as GLOBAL_TERRAIN_ALT (integer fields)
 *          
 *          Type Mask Usage (bits set to 1 = IGNORE that field):
 *          - POSITION_IGNORE: Ignore position fields (lat, lon, alt)
 *          - VEL_IGNORE: Ignore velocity fields (vx, vy, vz)
 *          - ACC_IGNORE: Must be set (acceleration control not supported)
 *          - YAW_IGNORE: Ignore yaw field
 *          - YAW_RATE_IGNORE: Ignore yaw_rate field
 *          
 *          Control Modes (same as local NED version):
 *          1. Position-only: Navigate to GPS waypoint
 *          2. Velocity+Heading: Move at specified speed in specified direction (NED frame)
 *          3. Velocity+TurnRate: Move at specified speed while turning
 *          4. Heading-only: Turn to face specified heading (skid-steer only)
 *          5. Turn rate-only: Turn at specified rate (skid-steer only)
 *          
 * @param[in] msg The MAVLink message containing global position/velocity target data
 * 
 * @note Only active when rover is in Guided mode
 * @note Position coordinates provided as 1e7 integers (degrees * 10^7)
 * @note Velocity vectors are in NED frame (North, East, Down) even though position is global
 * @note Acceleration fields are not supported and must be masked as ignored
 * @note Altitude field is ignored for ground rovers (uses terrain following if enabled)
 * 
 * @warning Invalid latitude/longitude values will cause command to be rejected
 * @warning Acceleration control (ACC_IGNORE=0) will cause command to be rejected
 * @warning Unsupported coordinate frames are silently ignored
 * 
 * Source: Rover/GCS_MAVLink_Rover.cpp:843-950
 */
void GCS_MAVLINK_Rover::handle_set_position_target_global_int(const mavlink_message_t &msg)
{
    // Decode the incoming MAVLink message
    mavlink_set_position_target_global_int_t packet;
    mavlink_msg_set_position_target_global_int_decode(&msg, &packet);

    // Only process if vehicle is in Guided mode - ignore commands in other modes
    if (!rover.control_mode->in_guided_mode()) {
        return;
    }
    
    // Validate coordinate frame - only accept global lat/lon frames
    switch (packet.coordinate_frame) {
    case MAV_FRAME_GLOBAL:                    // MSL altitude reference
    case MAV_FRAME_GLOBAL_INT:                // MSL altitude reference (integer)
    case MAV_FRAME_GLOBAL_RELATIVE_ALT:       // Home altitude reference
    case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:   // Home altitude reference (integer)
    case MAV_FRAME_GLOBAL_TERRAIN_ALT:        // Terrain altitude reference
    case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:    // Terrain altitude reference (integer)
        break;

    default:
        // Unsupported frame - silently ignore message
        return;
    }
    
    // Decode type mask to determine which fields are active
    // Type mask bits set to 1 indicate field should be IGNORED
    bool pos_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
    bool vel_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
    bool acc_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;
    bool yaw_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
    bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;

    // Process position target if not ignored
    Location target_loc = rover.current_loc;
    if (!pos_ignore) {
        // Validate latitude and longitude are within acceptable ranges
        // Reject invalid coordinates (e.g., lat > 90°, lon > 180°)
        if (!check_latlng(packet.lat_int, packet.lon_int)) {
            // Invalid coordinates - reject command silently
            return;
        }
        // Position provided as 1e7 integers (degrees * 10^7)
        target_loc.lat = packet.lat_int;
        target_loc.lng = packet.lon_int;
        // Note: Altitude is not used for ground rovers (uses terrain following if enabled)
    }

    // Initialize target motion parameters
    float target_speed = 0.0f;        // Target ground speed in m/s
    float target_yaw_cd = 0.0f;       // Target heading in centidegrees

    // Process velocity vector if not ignored
    // Velocity is provided in NED frame: vx=North (m/s), vy=East (m/s), vz=Down (m/s)
    if (!vel_ignore) {
        const float speed_max = rover.control_mode->get_speed_default();
        // Calculate speed magnitude from velocity vector (Pythagoras: sqrt(vx² + vy²))
        // Constrain to configured maximum speed (supports negative for reverse)
        target_speed = constrain_float(safe_sqrt(sq(packet.vx) + sq(packet.vy)), -speed_max, speed_max);
        // Calculate heading from velocity vector direction using atan2(East, North)
        // Convert from radians to centidegrees (deg * 100)
        target_yaw_cd = degrees(atan2f(packet.vy, packet.vx)) * 100.0f;
    }

    // Process yaw (heading) if explicitly provided
    // Explicit yaw overrides heading derived from velocity vector
    if (!yaw_ignore) {
        // Convert yaw from radians to centidegrees
        // Yaw is in NED frame: 0=North, 90=East, 180=South, 270=West
        target_yaw_cd = degrees(packet.yaw) * 100.0f;
    }

    // Process yaw rate (turn rate) if provided
    float target_turn_rate_cds = 0.0f; // Target turn rate in centidegrees per second
    if (!yaw_rate_ignore) {
        // Convert from radians/sec to centidegrees/sec
        target_turn_rate_cds = degrees(packet.yaw_rate) * 100.0f;
    }

    // Determine forward/reverse direction when both velocity and yaw/yaw_rate are provided
    // Default assumption is forward motion (positive speed)
    float speed_dir = 1.0f;
    if (!vel_ignore && (!yaw_ignore || !yaw_rate_ignore)) {
        // Check sign of vx (North velocity) to determine if rover should drive backwards
        // Note: Using vx even for global frames - negative vx indicates reverse motion
        if (is_negative(packet.vx)) {
            speed_dir = -1.0f; // Reverse direction
        }
    }

    // Validate acceleration field is ignored - rovers don't support acceleration control
    if (!acc_ignore) {
        // Reject command if acceleration control was requested
        send_acc_ignore_must_be_set_message("SET_POSITION_TARGET_GLOBAL_INT");
        return;
    }

    // Dispatch to appropriate guided mode control based on which fields are provided
    // Priority: Position > Velocity+Heading > Velocity+TurnRate > Heading-only > TurnRate-only
    
    if (!pos_ignore) {
        // Position control mode: Navigate to GPS target location
        // This is highest priority - if position provided, use position control
        if (!rover.mode_guided.set_desired_location(target_loc)) {
            // Failed to set location (e.g., unreachable or invalid)
            // GCS should monitor actual desired location to see if command took effect
        }
        return; // Position control is mutually exclusive with velocity control
    }

    // Velocity control modes (only reached if position is ignored)
    if (!vel_ignore && yaw_ignore && yaw_rate_ignore) {
        // Velocity-only: Drive at target speed using heading derived from velocity vector
        rover.mode_guided.set_desired_heading_and_speed(target_yaw_cd, speed_dir * target_speed);
    } else if (!vel_ignore && yaw_ignore && !yaw_rate_ignore) {
        // Velocity + turn rate: Drive at target speed while turning at specified rate
        rover.mode_guided.set_desired_turn_rate_and_speed(target_turn_rate_cds, speed_dir * target_speed);
    } else if (!vel_ignore && !yaw_ignore && yaw_rate_ignore) {
        // Velocity + heading: Drive at target speed toward specified heading
        rover.mode_guided.set_desired_heading_and_speed(target_yaw_cd, speed_dir * target_speed);
    } else if (vel_ignore && !yaw_ignore && yaw_rate_ignore) {
        // Heading-only: Turn to face specified heading at zero speed
        // Note: Typically only skid-steer vehicles can turn in place
        rover.mode_guided.set_desired_heading_and_speed(target_yaw_cd, 0.0f);
    } else if (vel_ignore && yaw_ignore && !yaw_rate_ignore) {
        // Turn rate-only: Turn at specified rate at zero speed
        // Note: Typically only skid-steer vehicles can turn in place
        rover.mode_guided.set_desired_turn_rate_and_speed(target_turn_rate_cds, 0.0f);
    }
}

/*
  handle a LANDING_TARGET command. The timestamp has been jitter corrected
*/
void GCS_MAVLINK_Rover::handle_landing_target(const mavlink_landing_target_t &packet, uint32_t timestamp_ms)
{
#if AC_PRECLAND_ENABLED
    rover.precland.handle_msg(packet, timestamp_ms);
#endif
}

uint64_t GCS_MAVLINK_Rover::capabilities() const
{
    return (MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
            MAV_PROTOCOL_CAPABILITY_MISSION_INT |
            MAV_PROTOCOL_CAPABILITY_COMMAND_INT |
            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
            MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
            GCS_MAVLINK::capabilities());
}

#if HAL_HIGH_LATENCY2_ENABLED
uint8_t GCS_MAVLINK_Rover::high_latency_tgt_heading() const
{
    const Mode *control_mode = rover.control_mode;
    if (rover.control_mode->is_autopilot_mode()) {
        // need to convert -180->180 to 0->360/2
        return wrap_360(control_mode->wp_bearing()) / 2;
    }
    return 0;
}
    
uint16_t GCS_MAVLINK_Rover::high_latency_tgt_dist() const
{
    const Mode *control_mode = rover.control_mode;
    if (rover.control_mode->is_autopilot_mode()) {
        // return units are dm
        return MIN((control_mode->get_distance_to_destination()) / 10, UINT16_MAX);
    }
    return 0;
}

uint8_t GCS_MAVLINK_Rover::high_latency_tgt_airspeed() const
{
    const Mode *control_mode = rover.control_mode;
    if (rover.control_mode->is_autopilot_mode()) {
        // return units are m/s*5
        return MIN((vfr_hud_airspeed() - control_mode->speed_error()) * 5, UINT8_MAX);
    }
    return 0;
}

uint8_t GCS_MAVLINK_Rover::high_latency_wind_speed() const
{
    if (rover.g2.windvane.enabled()) {
        // return units are m/s*5
        return MIN(rover.g2.windvane.get_true_wind_speed() * 5, UINT8_MAX);
    }
    return 0;
}

uint8_t GCS_MAVLINK_Rover::high_latency_wind_direction() const
{
    if (rover.g2.windvane.enabled()) {
        // return units are deg/2
        return wrap_360(degrees(rover.g2.windvane.get_true_wind_direction_rad())) / 2;
    }
    return 0;
}
#endif // HAL_HIGH_LATENCY2_ENABLED

// Send the mode with the given index (not mode number!) return the total number of modes
// Index starts at 1
uint8_t GCS_MAVLINK_Rover::send_available_mode(uint8_t index) const
{
    const Mode* modes[] {
        &rover.mode_manual,
        &rover.mode_acro,
        &rover.mode_steering,
        &rover.mode_hold,
        &rover.mode_loiter,
#if MODE_FOLLOW_ENABLED
        &rover.mode_follow,
#endif
        &rover.mode_simple,
        &rover.g2.mode_circle,
        &rover.mode_auto,
        &rover.mode_rtl,
        &rover.mode_smartrtl,
        &rover.mode_guided,
        &rover.mode_initializing,
#if MODE_DOCK_ENABLED
        (Mode *)rover.g2.mode_dock_ptr,
#endif
    };

    const uint8_t mode_count = ARRAY_SIZE(modes);

    // Convert to zero indexed
    const uint8_t index_zero = index - 1;
    if (index_zero >= mode_count) {
        // Mode does not exist!?
        return mode_count;
    }

    // Ask the mode for its name and number
    const char* name = modes[index_zero]->name4();
    const uint8_t mode_number = (uint8_t)modes[index_zero]->mode_number();

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
