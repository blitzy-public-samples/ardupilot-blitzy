/**
 * @file GCS_MAVLink_Blimp.cpp
 * @brief MAVLink protocol integration for Blimp ground control station communication
 * 
 * @details This file implements the comprehensive GCS interface for lighter-than-air vehicles (blimps),
 *          providing telemetry streaming, command handling, parameter protocol, mission protocol, and
 *          status reporting specific to airship dynamics. The implementation adapts the standard MAVLink
 *          protocol to handle unique characteristics of lighter-than-air vehicles including buoyancy
 *          effects, wind sensitivity, and low-speed flight dynamics.
 * 
 *          Key features:
 *          - Vehicle state reporting (armed, mode, failsafe, initialization status)
 *          - Position and navigation target telemetry
 *          - PID tuning parameter streaming for position and velocity controllers
 *          - Command handling for calibration, repositioning, and region of interest
 *          - Wind estimation telemetry using EKF3 airspeed vector estimation
 *          - Flight mode enumeration and switching
 *          - Blimp-specific HUD data including airspeed and altitude reporting
 * 
 * @note This implementation inherits from GCS_MAVLINK base class and overrides vehicle-specific
 *       methods to provide blimp-appropriate telemetry and command handling.
 * 
 * @warning All telemetry streaming is rate-limited by the GCS_MAVLINK base class. Message handlers
 *          must validate commands before execution to prevent unsafe vehicle operations.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include "Blimp.h"

#include "GCS_MAVLink_Blimp.h"
#include <AP_RPM/AP_RPM_config.h>
#include <AP_OpticalFlow/AP_OpticalFlow_config.h>

/**
 * @brief Returns the MAVLink frame type identifier for the blimp
 * 
 * @details Queries the blimp object for its configured frame type, which identifies
 *          this vehicle as a lighter-than-air airship in MAVLink communication.
 *          This is used by ground stations to apply appropriate UI elements and
 *          control schemes for blimp vehicles.
 * 
 * @return MAV_TYPE The MAVLink type identifier for this blimp frame
 * 
 * @note Called during heartbeat message generation and initial connection handshake
 * 
 * Source: Blimp/GCS_MAVLink_Blimp.cpp:7-10
 */
MAV_TYPE GCS_Blimp::frame_type() const
{
    return blimp.get_frame_mav_type();
}

/**
 * @brief Returns the base mode flags indicating vehicle capabilities and current state
 * 
 * @details Constructs the MAV_MODE_FLAG bitmask indicating:
 *          - STABILIZE_ENABLED: Vehicle has stabilization (always enabled for blimp)
 *          - MANUAL_INPUT_ENABLED: Manual pilot input is accepted
 *          - SAFETY_ARMED: Motors are armed and ready (set when not initializing)
 *          - CUSTOM_MODE_ENABLED: Vehicle uses custom flight modes
 * 
 *          The armed flag is determined by checking if motors object exists and reports
 *          armed status. This prevents reporting armed state during early initialization
 *          before motor control is fully configured.
 * 
 * @return uint8_t Bitmask of MAV_MODE_FLAG values representing vehicle state
 * 
 * @note This function is called at heartbeat rate (typically 1Hz) and during mode changes
 * 
 * @warning Do not assume armed state means motors are spinning - check throttle separately
 * 
 * Source: Blimp/GCS_MAVLink_Blimp.cpp:12-26
 */
uint8_t GCS_MAVLINK_Blimp::base_mode() const
{
    uint8_t _base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
    _base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

    // we are armed if we are not initialising
    if (blimp.motors != nullptr && blimp.motors->armed()) {
        _base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // indicate we have set a custom mode
    _base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    return _base_mode;
}

/**
 * @brief Returns the current flight mode number for MAVLink reporting
 * 
 * @details Provides the blimp's current control_mode as a 32-bit integer for MAVLink
 *          HEARTBEAT messages. The custom mode allows ground stations to display and
 *          manage vehicle-specific flight modes (MANUAL, VELOCITY, LOITER, RTL, LAND).
 * 
 * @return uint32_t Current flight mode number from Mode::Number enumeration
 * 
 * @note Ground stations must have blimp-specific mode definitions to interpret this value
 * 
 * Source: Blimp/GCS_MAVLink_Blimp.cpp:28-31
 */
uint32_t GCS_Blimp::custom_mode() const
{
    return (uint32_t)blimp.control_mode;
}

/**
 * @brief Reports the current system state of the blimp for MAVLink status reporting
 * 
 * @details Determines vehicle system status based on operational state with priority order:
 *          1. MAV_STATE_CRITICAL - Any failsafe has triggered (highest priority)
 *          2. MAV_STATE_STANDBY - Vehicle has landed (land_complete flag set)
 *          3. MAV_STATE_BOOT - Vehicle not yet initialized (startup sequence)
 *          4. MAV_STATE_ACTIVE - Normal operation (flying or ready to fly)
 * 
 *          This status is reported in HEARTBEAT messages and used by ground stations
 *          to provide visual indication of vehicle health and readiness.
 * 
 * @return MAV_STATE System status code indicating current operational state
 * 
 * @note Failsafe conditions take precedence over all other states to ensure GCS
 *       operators are immediately aware of critical situations
 * 
 * @warning Ground stations may disable certain commands when vehicle is in CRITICAL state
 * 
 * Source: Blimp/GCS_MAVLink_Blimp.cpp:33-48
 */
MAV_STATE GCS_MAVLINK_Blimp::vehicle_system_status() const
{
    // set system as critical if any failsafe have triggered
    if (blimp.any_failsafe_triggered())  {
        return MAV_STATE_CRITICAL;
    }

    if (blimp.ap.land_complete) {
        return MAV_STATE_STANDBY;
    }
    if (!blimp.ap.initialised) {
    	return MAV_STATE_BOOT;
    }

    return MAV_STATE_ACTIVE;
}

/**
 * @brief Sends the current position target to the ground control station
 * 
 * @details Transmits the POSITION_TARGET_GLOBAL_INT MAVLink message containing the vehicle's
 *          current navigation waypoint target. Only position components (lat/lon/alt) are sent;
 *          velocity, acceleration, yaw, and yaw_rate fields are ignored via type mask.
 * 
 *          The target location is retrieved from the current flight mode's get_wp() method.
 *          If the flight mode doesn't provide a waypoint target (e.g., in MANUAL mode),
 *          no message is sent.
 * 
 *          Message format:
 *          - Coordinate frame: MAV_FRAME_GLOBAL (global altitude reference)
 *          - Latitude/longitude: 1e7 integer format (standard MAVLink coordinates)
 *          - Altitude: Float meters (converted from centimeters)
 *          - All velocity/acceleration/yaw fields: Ignored via type mask
 * 
 * @note This message is typically streamed at 1-5Hz depending on stream rate configuration
 * 
 * @note For blimps, position targets are primary navigation references since airspeed
 *       control is less critical than for fixed-wing aircraft
 * 
 * Source: Blimp/GCS_MAVLink_Blimp.cpp:51-78
 */
void GCS_MAVLINK_Blimp::send_position_target_global_int()
{
    Location target;
    if (!blimp.flightmode->get_wp(target)) {
        return;
    }
    static constexpr uint16_t POSITION_TARGET_TYPEMASK_LAST_BYTE = 0xF000;
    static constexpr uint16_t TYPE_MASK = POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                                          POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                                          POSITION_TARGET_TYPEMASK_YAW_IGNORE | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE | POSITION_TARGET_TYPEMASK_LAST_BYTE;

    mavlink_msg_position_target_global_int_send(
        chan,
        AP_HAL::millis(), // time_boot_ms
        MAV_FRAME_GLOBAL, // targets are always global altitude
        TYPE_MASK, // ignore everything except the x/y/z components
        target.lat, // latitude as 1e7
        target.lng, // longitude as 1e7
        target.alt * 0.01f, // altitude is sent as a float
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
 * @brief Sends navigation controller output telemetry (currently unimplemented for blimp)
 * 
 * @details This function would typically send NAV_CONTROLLER_OUTPUT messages containing
 *          navigation controller state such as bearing to target, distance to target,
 *          crosstrack error, and altitude error. Currently empty for blimp implementation.
 * 
 * @note Future implementation may include blimp-specific navigation metrics such as
 *       wind-corrected bearing, drift compensation, and position hold error
 * 
 * @todo Implement navigation controller output for blimp with relevant metrics:
 *       bearing to target, distance remaining, position hold error, wind correction
 * 
 * Source: Blimp/GCS_MAVLink_Blimp.cpp:201-204
 */
void GCS_MAVLINK_Blimp::send_nav_controller_output() const
{

}

/**
 * @brief Calculates airspeed for VFR HUD display
 * 
 * @details Returns the vehicle's estimated airspeed for display in the VFR (Visual Flight Rules)
 *          HUD (Heads-Up Display). For blimps, this uses EKF3 wind estimation to calculate true
 *          airspeed from the body-frame airspeed vector. If EKF3 wind estimation is not available,
 *          falls back to GPS ground speed as a reasonable approximation.
 * 
 *          Calculation method:
 *          1. Attempt to retrieve body-frame true airspeed vector from AHRS/EKF3
 *          2. If successful, return the magnitude of the 3D airspeed vector
 *          3. If wind estimation unavailable, return GPS ground speed instead
 * 
 * @return float Airspeed in meters per second, or ground speed if airspeed unavailable
 * 
 * @note For lighter-than-air vehicles, wind has significant effects on flight characteristics.
 *       Accurate airspeed estimation is important for understanding vehicle control authority.
 * 
 * @note Ground speed fallback is reasonable for blimps at low speeds in calm conditions but
 *       may be misleading in strong winds
 * 
 * Source: Blimp/GCS_MAVLink_Blimp.cpp:206-215
 */
float GCS_MAVLINK_Blimp::vfr_hud_airspeed() const
{
    Vector3f airspeed_vec_bf;
    if (AP::ahrs().airspeed_vector_true(airspeed_vec_bf)) {
        // we are running the EKF3 wind estimation code which can give
        // us an airspeed estimate
        return airspeed_vec_bf.length();
    }
    return AP::gps().ground_speed();
}

/**
 * @brief Returns throttle percentage for VFR HUD display
 * 
 * @details Retrieves the current motor throttle output as a percentage (0-100) for display
 *          in the VFR HUD. For blimps, throttle represents the combined output to all
 *          fin actuators and propulsion motors. Returns 0 if motors object is not yet
 *          initialized (during early bootup).
 * 
 * @return int16_t Throttle percentage from 0 to 100, or 0 if motors not initialized
 * 
 * @note For blimps, "throttle" is an abstraction representing overall control effort
 *       rather than literal engine throttle as in fixed-wing aircraft
 * 
 * @note Called at HUD update rate (typically 4Hz) for display purposes only
 * 
 * Source: Blimp/GCS_MAVLink_Blimp.cpp:255-261
 */
int16_t GCS_MAVLINK_Blimp::vfr_hud_throttle() const
{
    if (blimp.motors == nullptr) {
        return 0;
    }
    return (int16_t)(blimp.motors->get_throttle() * 100);
}

/**
 * @brief Sends PID tuning telemetry for all active control loops
 * 
 * @details Transmits PID_TUNING MAVLink messages for position and velocity controllers
 *          on all axes (X, Y, Z, Yaw). This allows real-time monitoring and tuning of
 *          control loop performance through ground station software.
 * 
 *          PID loops reported (when enabled via gcs_pid_mask parameter):
 *          - VELX, VELY: Horizontal velocity controllers (body frame)
 *          - VELZ: Vertical velocity controller
 *          - VELYAW: Yaw rate controller
 *          - POSX, POSY: Horizontal position controllers
 *          - POSZ: Altitude position controller
 *          - POSYAW: Heading controller
 * 
 *          Each PID_TUNING message contains:
 *          - target: Desired setpoint
 *          - actual: Measured process variable
 *          - FF: Feedforward term
 *          - P: Proportional term
 *          - I: Integral term
 *          - D: Derivative term
 *          - slew_rate: Rate limiting value
 *          - Dmod: Modified derivative (with filtering/limiting)
 * 
 * @note No PID messages are sent in MANUAL or LAND modes as PIDs are not active
 * 
 * @note Message transmission is gated by gcs_pid_mask parameter - only enabled
 *       axes are transmitted to conserve bandwidth
 * 
 * @note Payload space is checked before each message to prevent buffer overrun
 * 
 * @warning High-rate PID streaming can saturate telemetry links. Use selectively
 *          during tuning operations only.
 * 
 * Source: Blimp/GCS_MAVLink_Blimp.cpp:266-328
 */
void GCS_MAVLINK_Blimp::send_pid_tuning()
{
    if (blimp.control_mode == Mode::Number::MANUAL || blimp.control_mode == Mode::Number::LAND) {
        //No PIDs are used in Manual or Land mode.
        return;
    }

    static const int8_t axes[] = {
        PID_SEND::VELX,
        PID_SEND::VELY,
        PID_SEND::VELZ,
        PID_SEND::VELYAW,
        PID_SEND::POSX,
        PID_SEND::POSY,
        PID_SEND::POSZ,
        PID_SEND::POSYAW
    };
    for (uint8_t i=0; i<ARRAY_SIZE(axes); i++) {
        if (!(blimp.g.gcs_pid_mask & (1<<(axes[i]-1)))) {
            continue;
        }
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
        const AP_PIDInfo *pid_info = nullptr;
        switch (axes[i]) {
        case PID_SEND::VELX:
            pid_info = &blimp.pid_vel_xy.get_pid_info_x();
            break;
        case PID_SEND::VELY:
            pid_info = &blimp.pid_vel_xy.get_pid_info_y();
            break;
        case PID_SEND::VELZ:
            pid_info = &blimp.pid_vel_z.get_pid_info();
            break;
        case PID_SEND::VELYAW:
            pid_info = &blimp.pid_vel_yaw.get_pid_info();
            break;
        case PID_SEND::POSX:
            pid_info = &blimp.pid_pos_xy.get_pid_info_x();
            break;
        case PID_SEND::POSY:
            pid_info = &blimp.pid_pos_xy.get_pid_info_y();
            break;
        case PID_SEND::POSZ:
            pid_info = &blimp.pid_pos_z.get_pid_info();
            break;
        case PID_SEND::POSYAW:
            pid_info = &blimp.pid_pos_yaw.get_pid_info();
            break;
        default:
            continue;
        }
        if (pid_info != nullptr) {
            mavlink_msg_pid_tuning_send(chan,
                                        axes[i],
                                        pid_info->target,
                                        pid_info->actual,
                                        pid_info->FF,
                                        pid_info->P,
                                        pid_info->I,
                                        pid_info->D,
                                        pid_info->slew_rate,
                                        pid_info->Dmod);
        }
    }
}

/**
 * @brief Reports whether vehicle has completed initialization
 * 
 * @details Returns the initialization status flag from the blimp autopilot state.
 *          This indicates whether all startup procedures including sensor detection,
 *          calibration loading, and subsystem initialization have completed successfully.
 * 
 * @return bool true if vehicle is fully initialized and ready for operation,
 *              false during bootup sequence
 * 
 * @note Used by base GCS_MAVLINK class to gate certain operations that require
 *       full system initialization
 * 
 * Source: Blimp/GCS_MAVLink_Blimp.cpp:385-388
 */
bool GCS_Blimp::vehicle_initialised() const
{
    return blimp.ap.initialised;
}

/**
 * @brief Attempts to send a vehicle-specific MAVLink message
 * 
 * @details Handles transmission of blimp-specific MAVLink messages that require custom
 *          processing. This function is called by the message streaming scheduler and
 *          processes messages not handled by the base GCS_MAVLINK class.
 * 
 *          Blimp-specific messages handled:
 *          - MSG_WIND: Wind estimate from EKF3 (requires valid airspeed vector)
 *          - MSG_ADSB_VEHICLE: ADS-B traffic (unused for blimp, no action)
 * 
 *          All other message IDs are forwarded to the base class implementation.
 * 
 * @param[in] id Message type identifier from ap_message enumeration
 * 
 * @return bool true if message was sent successfully or is not applicable,
 *              false if message sending failed (e.g., insufficient buffer space)
 * 
 * @note Payload size is checked before transmission to prevent buffer overruns
 * 
 * @note This function is called repeatedly by the message streaming scheduler,
 *       so it must be efficient and return quickly
 * 
 * Source: Blimp/GCS_MAVLink_Blimp.cpp:391-408
 */
bool GCS_MAVLINK_Blimp::try_send_message(enum ap_message id)
{
    switch (id) {

    case MSG_WIND:
        CHECK_PAYLOAD_SIZE(WIND);
        send_wind();
        break;

    case MSG_ADSB_VEHICLE:
        // unused
        break;

    default:
        return GCS_MAVLINK::try_send_message(id);
    }
    return true;
}

/**
 * @brief Processes received MAVLink packets with blimp-specific handling
 * 
 * @details Called by MAVLink parser when a complete packet is received. This override
 *          allows blimp-specific packet processing before forwarding to base class.
 *          Currently performs no blimp-specific processing and immediately delegates
 *          to parent class.
 * 
 * @param[in] status MAVLink parser status containing sequence numbers and packet stats
 * @param[in] msg Received MAVLink message with header and payload
 * 
 * @note This function is called in the communication thread context. Message handlers
 *       must be thread-safe or use appropriate synchronization.
 * 
 * @note Future blimp-specific packet preprocessing could be added here without
 *       modifying base class behavior
 * 
 * Source: Blimp/GCS_MAVLink_Blimp.cpp:449-453
 */
void GCS_MAVLINK_Blimp::packetReceived(const mavlink_status_t &status,
                                       const mavlink_message_t &msg)
{
    GCS_MAVLINK::packetReceived(status, msg);
}

/**
 * @brief Indicates whether parameter system is ready for GCS access
 * 
 * @details Determines if the parameter system is fully initialized and safe to query.
 *          This prevents ground stations from receiving inconsistent parameter counts
 *          or values during bootup before subsystems are fully configured.
 * 
 *          Returns true when:
 *          - Board is in configuration error state (parameter access allowed for recovery)
 *          - All parameters initialized AND motors object allocated
 * 
 * @return bool true if parameters can be safely read/written, false during initialization
 * 
 * @note Prevents confusing GCS behavior where parameter count changes during boot as
 *       subsystems come online and allocate parameter groups
 * 
 * Source: Blimp/GCS_MAVLink_Blimp.cpp:455-465
 */
bool GCS_MAVLINK_Blimp::params_ready() const
{
    if (AP_BoardConfig::in_config_error()) {
        // we may never have parameters "initialised" in this case
        return true;
    }
    // if we have not yet initialised (including allocating the motors
    // object) we drop this request. That prevents the GCS from getting
    // a confusing parameter count during bootup
    return blimp.ap.initialised_params;
}

/**
 * @brief Sends initial connection banner to ground station
 * 
 * @details Transmits startup messages when ground station first connects, including
 *          standard banner from base class plus blimp-specific frame type string.
 *          This helps operators identify the vehicle type and configuration.
 * 
 * @note Called once during initial GCS connection handshake
 * 
 * @note Frame string identifies the specific blimp configuration (e.g., "Blimp")
 * 
 * Source: Blimp/GCS_MAVLink_Blimp.cpp:467-471
 */
void GCS_MAVLINK_Blimp::send_banner()
{
    GCS_MAVLINK::send_banner();
    send_text(MAV_SEVERITY_INFO, "Frame: %s", blimp.get_frame_string());
}

/**
 * @brief Handles preflight calibration commands from ground station
 * 
 * @details Processes MAV_CMD_PREFLIGHT_CALIBRATION commands for sensor calibration
 *          (gyroscope, accelerometer, magnetometer, barometer, etc.). Currently
 *          delegates all calibration handling to base class implementation without
 *          blimp-specific preprocessing.
 * 
 * @param[in] packet COMMAND_INT packet containing calibration parameters
 * @param[in] msg Original MAVLink message (for sequence tracking)
 * 
 * @return MAV_RESULT Command execution result (ACCEPTED, DENIED, FAILED, etc.)
 * 
 * @note Calibration commands are critical for flight safety. Base class performs
 *       extensive validation before initiating calibration procedures.
 * 
 * @warning Vehicle must be stationary and disarmed for most calibration operations
 * 
 * Source: Blimp/GCS_MAVLink_Blimp.cpp:523-526
 */
MAV_RESULT GCS_MAVLINK_Blimp::_handle_command_preflight_calibration(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    return GCS_MAVLINK::_handle_command_preflight_calibration(packet, msg);
}

/**
 * @brief Handles Region of Interest (ROI) pointing commands
 * 
 * @details Processes MAV_CMD_DO_SET_ROI commands that direct the vehicle to point
 *          a camera or sensor at a specified geographic location. Validates the
 *          provided location coordinates before acceptance.
 * 
 * @param[in] roi_loc Target location (latitude, longitude, altitude) for ROI pointing
 * 
 * @return MAV_RESULT MAV_RESULT_ACCEPTED if location valid and command processed,
 *                    MAV_RESULT_FAILED if location coordinates are invalid
 * 
 * @note Current implementation validates location but does not execute ROI pointing
 *       (auto_yaw.set_roi() call is commented out, awaiting full implementation)
 * 
 * @todo Complete ROI implementation by enabling auto_yaw.set_roi() once yaw
 *       controller supports region-of-interest tracking for blimps
 * 
 * Source: Blimp/GCS_MAVLink_Blimp.cpp:529-536
 */
MAV_RESULT GCS_MAVLINK_Blimp::handle_command_do_set_roi(const Location &roi_loc)
{
    if (!roi_loc.check_latlng()) {
        return MAV_RESULT_FAILED;
    }
    // blimp.flightmode->auto_yaw.set_roi(roi_loc);
    return MAV_RESULT_ACCEPTED;
}

/**
 * @brief Handles position repositioning commands (MAV_CMD_DO_REPOSITION)
 * 
 * @details Processes commands to move the vehicle to a new geographic position.
 *          This is typically used in GUIDED mode to command the vehicle to fly to
 *          a specific location without uploading a full mission.
 * 
 *          Validation steps:
 *          1. Check if vehicle is in GUIDED mode OR change_modes flag is set
 *          2. Validate latitude/longitude are within valid ranges
 *          3. Parse location from command packet (lat/lon/alt/frame)
 *          4. Sanitize location relative to current position
 * 
 * @param[in] packet COMMAND_INT packet with position target (x=lat*1e7, y=lon*1e7, z=alt)
 * 
 * @return MAV_RESULT Command execution result:
 *                    - MAV_RESULT_DENIED if not in GUIDED mode and no mode change requested
 *                    - MAV_RESULT_DENIED if location coordinates invalid
 *                    - MAV_RESULT_DENIED if location sanitization fails
 *                    - MAV_RESULT_ACCEPTED if command validated and ready for execution
 * 
 * @note param2 bit MAV_DO_REPOSITION_FLAGS_CHANGE_MODE allows mode switching to GUIDED
 * 
 * @warning Location sanitization ensures target is reasonable relative to current position
 *          (not too far, not underground, etc.) to prevent erratic behavior
 * 
 * Source: Blimp/GCS_MAVLink_Blimp.cpp:538-561
 */
MAV_RESULT GCS_MAVLINK_Blimp::handle_command_int_do_reposition(const mavlink_command_int_t &packet)
{
    const bool change_modes = ((int32_t)packet.param2 & MAV_DO_REPOSITION_FLAGS_CHANGE_MODE) == MAV_DO_REPOSITION_FLAGS_CHANGE_MODE;
    if (!blimp.flightmode->in_guided_mode() && !change_modes) {
        return MAV_RESULT_DENIED;
    }

    // sanity check location
    if (!check_latlng(packet.x, packet.y)) {
        return MAV_RESULT_DENIED;
    }

    Location request_location {};
    if (!location_from_command_t(packet, request_location)) {
        return MAV_RESULT_DENIED;
    }

    if (request_location.sanitize(blimp.current_loc)) {
        // if the location wasn't already sane don't load it
        return MAV_RESULT_DENIED; // failed as the location is not valid
    }

    return MAV_RESULT_ACCEPTED;
}

/**
 * @brief Routes COMMAND_INT messages to appropriate blimp-specific handlers
 * 
 * @details Dispatches incoming COMMAND_INT messages (MAVLink commands with integer
 *          position parameters) to vehicle-specific command handlers. This allows
 *          blimp to override or customize behavior for specific commands while
 *          delegating others to base class implementation.
 * 
 *          Blimp-specific command handling:
 *          - MAV_CMD_DO_REPOSITION: Position command handler with validation
 *          - MAV_CMD_NAV_TAKEOFF: Accepted without action (blimp has no distinct takeoff)
 *          - All others: Forwarded to base class for standard processing
 * 
 * @param[in] packet COMMAND_INT packet containing command ID and parameters
 * @param[in] msg Original MAVLink message (for context and acknowledgment)
 * 
 * @return MAV_RESULT Command execution result from handler
 * 
 * @note COMMAND_INT uses integer lat/lon (1e7 format) which provides higher
 *       precision than COMMAND_LONG floating point representation
 * 
 * @note Takeoff command always accepted since blimps don't require traditional takeoff
 *       sequence due to inherent buoyancy
 * 
 * Source: Blimp/GCS_MAVLink_Blimp.cpp:628-640
 */
MAV_RESULT GCS_MAVLINK_Blimp::handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    switch (packet.command) {
    case MAV_CMD_DO_REPOSITION:
        return handle_command_int_do_reposition(packet);
    case MAV_CMD_NAV_TAKEOFF:
        return MAV_RESULT_ACCEPTED;
    default:
        return GCS_MAVLINK::handle_command_int_packet(packet, msg);
    }
}

#if AP_MAVLINK_COMMAND_LONG_ENABLED
/**
 * @brief Determines coordinate frame for COMMAND_LONG takeoff commands
 * 
 * @details Specifies the coordinate frame to be used for COMMAND_LONG format commands.
 *          For NAV_TAKEOFF, forces use of MAV_FRAME_GLOBAL_RELATIVE_ALT to ensure
 *          altitude is interpreted relative to home position rather than sea level.
 * 
 * @param[out] frame Reference to frame variable to be set
 * @param[in] packet_command Command ID to check for frame override
 * 
 * @return bool true if frame was set by this function, false to use default
 * 
 * @note COMMAND_LONG (floating point parameters) is legacy format compared to COMMAND_INT
 * 
 * @note Relative altitude frame is important for blimps operating at varying ground elevations
 * 
 * Source: Blimp/GCS_MAVLink_Blimp.cpp:643-650
 */
bool GCS_MAVLINK_Blimp::mav_frame_for_command_long(MAV_FRAME &frame, MAV_CMD packet_command) const
{
    if (packet_command == MAV_CMD_NAV_TAKEOFF) {
        frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
        return true;
    }
    return GCS_MAVLINK::mav_frame_for_command_long(frame, packet_command);
}
#endif

/**
 * @brief Main message handler dispatcher for incoming MAVLink messages
 * 
 * @details Routes received MAVLink messages to appropriate handlers based on message ID.
 *          Provides blimp-specific handling for certain messages while delegating most
 *          to base class implementation.
 * 
 *          Blimp-specific message handling:
 *          - TERRAIN_DATA: Ignored (terrain following not implemented for blimp)
 *          - TERRAIN_CHECK: Ignored (terrain database queries not needed)
 *          - All others: Forwarded to GCS_MAVLINK base class for standard processing
 * 
 * @param[in] msg MAVLink message to process
 * 
 * @note This is the main entry point for all received MAVLink messages after packet parsing
 * 
 * @note Message handlers must execute quickly to avoid blocking communication thread.
 *       Long-running operations should be deferred to main loop.
 * 
 * @note Terrain messages ignored because blimps typically operate at low altitude with
 *       visual reference and don't require terrain-following capabilities
 * 
 * Source: Blimp/GCS_MAVLink_Blimp.cpp:653-665
 */
void GCS_MAVLINK_Blimp::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {

    case MAVLINK_MSG_ID_TERRAIN_DATA:
    case MAVLINK_MSG_ID_TERRAIN_CHECK:
        break;

    default:
        GCS_MAVLINK::handle_message(msg);
        break;
    }     // end switch
} // end handle mavlink

/**
 * @brief Handles emergency flight termination commands
 * 
 * @details Processes MAV_CMD_DO_FLIGHTTERMINATION commands that immediately terminate
 *          flight operations by disarming the vehicle. This is an emergency safety
 *          mechanism to stop all motor output in critical situations.
 * 
 *          Termination is triggered when param1 > 0.5, immediately calling disarm()
 *          with TERMINATION method to bypass normal disarming safety checks.
 * 
 * @param[in] packet COMMAND_INT packet with param1 indicating termination request
 * 
 * @return MAV_RESULT MAV_RESULT_ACCEPTED if termination executed,
 *                    MAV_RESULT_FAILED if param1 <= 0.5 (termination not requested)
 * 
 * @warning This command immediately stops all motors without any landing sequence.
 *          Use only in emergency situations where continued flight poses greater risk
 *          than immediate motor shutdown.
 * 
 * @note This bypasses normal arming checks and safety interlocks. Vehicle will drop
 *       if airborne when termination executed.
 * 
 * Source: Blimp/GCS_MAVLink_Blimp.cpp:736-744
 */
MAV_RESULT GCS_MAVLINK_Blimp::handle_flight_termination(const mavlink_command_int_t &packet)
{
    MAV_RESULT result = MAV_RESULT_FAILED;
    if (packet.param1 > 0.5f) {
        blimp.arming.disarm(AP_Arming::Method::TERMINATION);
        result = MAV_RESULT_ACCEPTED;
    }
    return result;
}

/**
 * @brief Returns altitude for VFR HUD display with compatibility option
 * 
 * @details Provides altitude value for VFR HUD display, with optional compatibility
 *          mode for legacy ground stations. When DevOptionVFR_HUDRelativeAlt is set,
 *          returns relative altitude (home-referenced) in meters. Otherwise uses
 *          base class implementation which provides absolute altitude (AMSL).
 * 
 * @return float Altitude in meters (relative or absolute depending on compatibility option)
 * 
 * @note Compatibility option allows support for older GCS software that expects
 *       relative altitude in HUD messages for multicopter-class vehicles
 * 
 * @note Altitude converted from centimeters (internal format) to meters for MAVLink
 * 
 * Source: Blimp/GCS_MAVLink_Blimp.cpp:746-754
 */
float GCS_MAVLINK_Blimp::vfr_hud_alt() const
{
    if (blimp.g2.dev_options.get() & DevOptionVFR_HUDRelativeAlt) {
        // compatibility option for older mavlink-aware devices that
        // assume Blimp returns a relative altitude in VFR_HUD.alt
        return blimp.current_loc.alt * 0.01f;
    }
    return GCS_MAVLINK::vfr_hud_alt();
}

/**
 * @brief Reports blimp MAVLink protocol capabilities to ground station
 * 
 * @details Returns capability flags indicating which MAVLink protocol features are
 *          supported by this blimp implementation. Used during connection handshake
 *          to inform ground stations of available functionality.
 * 
 *          Blimp-specific capabilities:
 *          - MISSION_FLOAT: Supports floating-point mission items
 *          - MISSION_INT: Supports integer-encoded mission items (preferred)
 *          - COMMAND_INT: Supports COMMAND_INT messages (integer coordinates)
 *          - SET_POSITION_TARGET_LOCAL_NED: Position control in local NED frame
 *          - SET_POSITION_TARGET_GLOBAL_INT: Position control in global coordinates
 *          - FLIGHT_TERMINATION: Emergency motor shutdown capability
 *          - SET_ATTITUDE_TARGET: Direct attitude control commands
 * 
 * @return uint64_t Bitmask of MAV_PROTOCOL_CAPABILITY flags
 * 
 * @note Capabilities list informs GCS which commands and message types can be used
 * 
 * Source: Blimp/GCS_MAVLink_Blimp.cpp:756-766
 */
uint64_t GCS_MAVLINK_Blimp::capabilities() const
{
    return (MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
            MAV_PROTOCOL_CAPABILITY_MISSION_INT |
            MAV_PROTOCOL_CAPABILITY_COMMAND_INT |
            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
            MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
            MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
            GCS_MAVLINK::capabilities());
}

/**
 * @brief Reports current landed state for ground station display
 * 
 * @details Determines and reports the vehicle's current landed state for GCS situational
 *          awareness. State is determined by checking flight mode and land_complete flag.
 * 
 *          State priority:
 *          1. ON_GROUND: land_complete flag is set (vehicle has settled on surface)
 *          2. LANDING: Flight mode reports landing sequence in progress
 *          3. IN_AIR: Default state when airborne and not landing
 * 
 * @return MAV_LANDED_STATE Current landed state code
 * 
 * @note For blimps, "landed" means resting on surface despite inherent buoyancy.
 *       Buoyancy compensation or tether attachment maintains ground contact.
 * 
 * @note Takeoff state commented out - blimps don't have distinct takeoff phase due
 *       to continuous buoyancy
 * 
 * Source: Blimp/GCS_MAVLink_Blimp.cpp:768-780
 */
MAV_LANDED_STATE GCS_MAVLINK_Blimp::landed_state() const
{
    if (blimp.ap.land_complete) {
        return MAV_LANDED_STATE_ON_GROUND;
    }
    if (blimp.flightmode->is_landing()) {
        return MAV_LANDED_STATE_LANDING;
    }
    // if (blimp.flightmode->is_taking_off()) {
    //     return MAV_LANDED_STATE_TAKEOFF;
    // }
    return MAV_LANDED_STATE_IN_AIR;
}

/**
 * @brief Sends wind estimation telemetry to ground control station
 * 
 * @details Transmits WIND MAVLink message containing wind direction (degrees), wind speed (m/s),
 *          and vertical wind component (m/s). Wind estimation is derived from EKF3 airspeed
 *          vector estimation, which is particularly important for lighter-than-air vehicles
 *          due to their high sensitivity to wind disturbances.
 * 
 *          Wind vector conversion:
 *          - Direction: Calculated from horizontal wind components (North-East plane) using atan2
 *          - Speed: 3D magnitude of wind vector
 *          - Vertical: Direct Z-axis wind component
 * 
 * @return void No return value. Aborts silently if airspeed vector not available
 * 
 * @note Only sends wind data if EKF3 airspeed vector estimation is active and valid.
 *       Blimps require valid airspeed estimates for reliable wind estimation due to
 *       their low flight speeds relative to wind speeds.
 * 
 * @note Wind direction follows meteorological convention: direction FROM which wind blows
 *       (e.g., 0 degrees = wind from North)
 * 
 * @warning Invalid wind data could lead to poor trajectory planning in autonomous modes.
 *          Message is not sent if airspeed vector unavailable.
 * 
 * Source: Blimp/GCS_MAVLink_Blimp.cpp:865-879
 */
void GCS_MAVLINK_Blimp::send_wind() const
{
    Vector3f airspeed_vec_bf;
    if (!AP::ahrs().airspeed_vector_true(airspeed_vec_bf)) {
        // if we don't have an airspeed estimate then we don't have a
        // valid wind estimate on blimps
        return;
    }
    const Vector3f wind = AP::ahrs().wind_estimate();
    mavlink_msg_wind_send(
        chan,
        degrees(atan2f(-wind.y, -wind.x)),
        wind.length(),
        wind.z);
}

#if HAL_HIGH_LATENCY2_ENABLED
/**
 * @brief Provides wind speed for HIGH_LATENCY2 telemetry message
 * 
 * @details Returns quantized wind speed optimized for low-bandwidth satellite links.
 *          Wind speed is derived from EKF3 wind estimation and scaled by factor of 5
 *          for HIGH_LATENCY2 protocol encoding (allows 0-51 m/s in single byte).
 * 
 * @return uint8_t Wind speed in units of m/s * 5, or 0 if wind estimate unavailable
 * 
 * @note Returns 0 if airspeed vector estimation is not active. For blimps, valid
 *       airspeed estimation is prerequisite for wind estimation due to low speed
 *       flight characteristics.
 * 
 * @note HIGH_LATENCY2 protocol uses compact encoding for satellite communication.
 *       Actual wind speed = returned_value / 5.0 (units: m/s)
 * 
 * Source: Blimp/GCS_MAVLink_Blimp.cpp:882-893
 */
uint8_t GCS_MAVLINK_Blimp::high_latency_wind_speed() const
{
    Vector3f airspeed_vec_bf;
    if (!AP::ahrs().airspeed_vector_true(airspeed_vec_bf)) {
        // if we don't have an airspeed estimate then we don't have a
        // valid wind estimate on blimps
        return 0;
    }
    // return units are m/s*5
    const Vector3f wind = AP::ahrs().wind_estimate();
    return wind.length() * 5;
}

/**
 * @brief Provides wind direction for HIGH_LATENCY2 telemetry message
 * 
 * @details Returns quantized wind direction optimized for low-bandwidth satellite links.
 *          Direction is converted from EKF3 wind estimate NED frame to meteorological
 *          convention (direction FROM which wind blows) and scaled by factor of 2
 *          for HIGH_LATENCY2 encoding (allows 0-360 degrees in single byte with 2° resolution).
 * 
 * @return uint8_t Wind direction in units of degrees / 2 (0-180 represents 0-360°),
 *                 or 0 if wind estimate unavailable
 * 
 * @note Returns 0 if airspeed vector estimation is not active. For blimps, valid
 *       airspeed estimation is prerequisite for wind estimation.
 * 
 * @note HIGH_LATENCY2 protocol uses compact encoding for satellite communication.
 *       Actual wind direction = returned_value * 2.0 (units: degrees)
 * 
 * @note Wind direction follows meteorological convention: direction FROM which wind blows
 *       (e.g., 0 degrees = wind from North, 90 degrees = wind from East)
 * 
 * Source: Blimp/GCS_MAVLink_Blimp.cpp:895-906
 */
uint8_t GCS_MAVLINK_Blimp::high_latency_wind_direction() const
{
    Vector3f airspeed_vec_bf;
    if (!AP::ahrs().airspeed_vector_true(airspeed_vec_bf)) {
        // if we don't have an airspeed estimate then we don't have a
        // valid wind estimate on blimps
        return 0;
    }
    const Vector3f wind = AP::ahrs().wind_estimate();
    // need to convert -180->180 to 0->360/2
    return wrap_360(degrees(atan2f(-wind.y, -wind.x))) / 2;
}
#endif // HAL_HIGH_LATENCY2_ENABLED

/**
 * @brief Sends available flight mode information to ground control station
 * 
 * @details Transmits AVAILABLE_MODES MAVLink message for the specified mode index,
 *          enabling ground stations to display and select from valid blimp flight modes.
 *          This function is part of the mode enumeration protocol where GCS queries
 *          each mode sequentially by index.
 * 
 *          Available blimp flight modes (in order):
 *          1. Land - Controlled descent and landing sequence
 *          2. Manual - Direct pilot control without stabilization
 *          3. Velocity - Velocity-controlled flight with position hold
 *          4. Loiter - Position hold with heading control
 *          5. RTL - Return to launch autonomous navigation
 * 
 * @param[in] index Mode index (1-based) to query. Index 1 returns first mode,
 *                  index 2 returns second mode, etc.
 * 
 * @return uint8_t Total number of available modes (always returns mode_count regardless
 *                 of index validity, allowing GCS to determine range)
 * 
 * @note Index is 1-based per MAVLink protocol convention (not 0-based array indexing).
 *       Function converts to 0-based internally.
 * 
 * @note If index exceeds available modes, function returns mode_count without sending
 *       message. GCS uses this to detect end of mode list.
 * 
 * @note All blimp modes report MAV_STANDARD_MODE_NON_STANDARD as they are vehicle-specific
 *       and not part of the standard MAVLink mode set.
 * 
 * @warning Mode numbers are vehicle-specific enumerations. Ground stations must not
 *          assume mode numbers are consistent across vehicle types.
 * 
 * Source: Blimp/GCS_MAVLink_Blimp.cpp:909-945
 */
uint8_t GCS_MAVLINK_Blimp::send_available_mode(uint8_t index) const
{
    const Mode* modes[] {
        &blimp.mode_land,
        &blimp.mode_manual,
        &blimp.mode_velocity,
        &blimp.mode_loiter,
        &blimp.mode_rtl,
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
