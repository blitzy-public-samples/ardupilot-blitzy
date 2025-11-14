/**
 * @file GCS_MAVLink_Tracker.cpp
 * @brief MAVLink protocol handling for antenna tracker vehicle
 * 
 * @details Implements tracker-specific MAVLink message handlers, telemetry streaming,
 *          and mode/status reporting. This file provides the communication bridge between
 *          ground control stations and the antenna tracker vehicle, translating tracker
 *          state into MAVLink messages and processing incoming commands and vehicle
 *          position updates for tracking.
 * 
 *          Key responsibilities:
 *          - Vehicle identification and mode reporting (frame_type, base_mode, custom_mode)
 *          - Telemetry generation (NAV_CONTROLLER_OUTPUT, GLOBAL_POSITION_INT, PID_TUNING)
 *          - Incoming message dispatch (position updates, manual control, attitude targets)
 *          - Command handling (arming, servo test, mission start)
 *          - Target vehicle locking and data stream requests
 * 
 * @note This implementation follows the MAVLink protocol specification v2
 * @note Vehicle locking allows tracker to automatically target a single vehicle
 * 
 * Source: AntennaTracker/GCS_MAVLink_Tracker.cpp:1-522
 */

#include "GCS_MAVLink_Tracker.h"
#include "Tracker.h"

/**
 * @brief Returns MAVLink vehicle type identifier for antenna tracker
 * 
 * @return MAV_TYPE_ANTENNA_TRACKER identifying this as an antenna tracker vehicle
 * 
 * @note Used by ground control stations to display appropriate UI and capabilities
 * @note This constant type allows GCS to understand vehicle-specific commands and telemetry
 */
MAV_TYPE GCS_Tracker::frame_type() const
{
    return MAV_TYPE_ANTENNA_TRACKER;
}

/**
 * @brief Constructs MAV_MODE base_mode flags from current tracker state
 * 
 * @details Translates tracker mode and arm state to standard MAVLink mode flags.
 *          This provides limited information for generic ground control stations
 *          that don't understand tracker-specific custom modes. The custom_mode
 *          field provides the actual tracker mode number and is the primary
 *          mode indicator.
 * 
 *          Mode flag mappings:
 *          - MANUAL mode → MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
 *          - AUTO/GUIDED/SCAN/SERVOTEST → MAV_MODE_FLAG_GUIDED_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED
 *          - STOP/INITIALISING → No additional flags (only CUSTOM_MODE_ENABLED)
 * 
 *          Arming logic:
 *          - Armed flag set if: safety switch not disarmed AND not in INITIALISING mode AND soft armed
 *          - Armed state indicates servos are actively controlled
 * 
 * @return uint8_t MAV_MODE flags bitfield per MAVLink protocol
 * 
 * @note base_mode has limited usefulness for tracker - custom_mode provides actual mode number
 * @note MAV_MODE_FLAG_AUTO_ENABLED not used as it implies autonomous goal finding
 * @note Generic GCS can still extract basic information (manual vs guided, armed state)
 * 
 * Source: AntennaTracker/GCS_MAVLink_Tracker.cpp:9-51
 */
uint8_t GCS_MAVLINK_Tracker::base_mode() const
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
    switch (tracker.mode->number()) {
    case Mode::Number::MANUAL:
        _base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        break;

    case Mode::Number::STOP:
        break;

    case Mode::Number::SCAN:
    case Mode::Number::SERVOTEST:
    case Mode::Number::AUTO:
    case Mode::Number::GUIDED:
        _base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED |
            MAV_MODE_FLAG_STABILIZE_ENABLED;
        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
        break;

    case Mode::Number::INITIALISING:
        break;
    }

    // we are armed if safety switch is not disarmed
    if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED &&
        tracker.mode != &tracker.mode_initialising &&
        hal.util->get_soft_armed()) {
        _base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    return _base_mode;
}

/**
 * @brief Returns tracker-specific mode number for MAVLink custom_mode field
 * 
 * @details The custom_mode field provides the actual tracker mode as defined in
 *          the Mode::Number enum. This is the primary mode indicator that ground
 *          control stations should use to determine tracker behavior.
 * 
 *          Available modes:
 *          - MANUAL: Direct pilot control of yaw/pitch
 *          - STOP: Servos hold current position
 *          - SCAN: Automated scanning pattern
 *          - GUIDED: External attitude commands accepted
 *          - SERVOTEST: Individual servo control for testing
 *          - AUTO: Automatic target tracking
 *          - INITIALISING: Startup calibration
 * 
 * @return uint32_t Mode number from Mode::Number enum
 * 
 * @note custom_mode is primary mode indicator - more useful than base_mode
 * @note Ground stations use this to display current mode and available mode transitions
 * @note Mode numbers map directly to tracker mode enum for unambiguous interpretation
 * 
 * Source: AntennaTracker/GCS_MAVLink_Tracker.cpp:53-56
 */
uint32_t GCS_Tracker::custom_mode() const
{
    return (uint32_t)tracker.mode->number();
}

/**
 * @brief Returns vehicle status for MAVLink HEARTBEAT message
 * 
 * @details Reports system state to ground control stations for UI indication:
 *          - MAV_STATE_CALIBRATING: During INITIALISING mode (startup/calibration)
 *          - MAV_STATE_ACTIVE: All other modes (ready for operation)
 * 
 *          The CALIBRATING state allows GCS to display initialization progress
 *          and prevents premature mode changes or commands during startup.
 * 
 * @return MAV_STATE_CALIBRATING during initialization, MAV_STATE_ACTIVE otherwise
 * 
 * @note Used by ground control stations to indicate initialization progress
 * @note ACTIVE state indicates tracker is ready to receive commands and track targets
 * @note Does not report error states - those are conveyed through other messages
 * 
 * Source: AntennaTracker/GCS_MAVLink_Tracker.cpp:58-64
 */
MAV_STATE GCS_MAVLINK_Tracker::vehicle_system_status() const
{
    if (tracker.mode == &tracker.mode_initialising) {
        return MAV_STATE_CALIBRATING;
    }
    return MAV_STATE_ACTIVE;
}

/**
 * @brief Sends NAV_CONTROLLER_OUTPUT telemetry message with current tracking state
 * 
 * @details Transmits the tracker's current pointing state and target information to
 *          ground control stations. This message provides situational awareness of
 *          what the tracker is pointing at and how far away the target is.
 * 
 *          Message contents:
 *          - nav_roll: Always 0 (trackers don't have roll axis)
 *          - nav_pitch: Current pitch target in degrees (-90 to +90)
 *          - nav_bearing: Bearing to target in degrees (0-360)
 *          - target_bearing: Same as nav_bearing for tracker
 *          - wp_dist: Distance to target in meters (capped at UINT16_MAX = 65535m)
 *          - alt_error: Altitude difference in meters (target altitude - tracker altitude)
 *          - aspd_error: Always 0 (not applicable to trackers)
 *          - xtrack_error: Always 0 (not applicable to trackers)
 * 
 *          Altitude source selection (ALT_SOURCE parameter):
 *          - ALT_SOURCE_BARO: Use barometric altitude difference
 *          - ALT_SOURCE_GPS: Use GPS altitude difference
 * 
 * @note Called periodically per telemetry stream rate configuration (typically 1-10Hz)
 * @note Distance capped at UINT16_MAX to fit in message field (approximately 65km)
 * @note Altitude difference sign: positive means target is above tracker
 * @note All angle values in degrees, distance in meters per MAVLink convention
 * 
 * Source: AntennaTracker/GCS_MAVLink_Tracker.cpp:66-80
 */
void GCS_MAVLINK_Tracker::send_nav_controller_output() const
{
	// Select altitude source per ALT_SOURCE parameter: barometric or GPS altitude difference
	float alt_diff = (tracker.g.alt_source == ALT_SOURCE_BARO) ? tracker.nav_status.alt_difference_baro : tracker.nav_status.alt_difference_gps;

    mavlink_msg_nav_controller_output_send(
        chan,
        0,
        tracker.nav_status.pitch,
        tracker.nav_status.bearing,
        tracker.nav_status.bearing,
        MIN(tracker.nav_status.distance, UINT16_MAX),
        alt_diff,
        0,
        0);
}

/**
 * @brief Handles MAVLink SET_ATTITUDE_TARGET message for GUIDED mode control
 * 
 * @details Processes external attitude commands to control tracker pointing in GUIDED mode.
 *          This enables external controllers (e.g., companion computers, ROS nodes) to
 *          command tracker orientation using quaternions and optionally yaw rate.
 * 
 *          Message validation sequence:
 *          1. Check tracker is in GUIDED mode (reject if not)
 *          2. Validate body_roll_rate is zero (trackers don't have roll axis)
 *          3. Validate type_mask bit 0 set (body roll rate must be ignored)
 *          4. Validate type_mask bit 6 set (throttle must be ignored)
 *          5. Validate type_mask bit 7 clear (attitude must NOT be ignored)
 *          6. Validate at least one of pitch or yaw rates is active (bits 3,4 not both set)
 * 
 *          type_mask bit definitions (1 = ignore field):
 *          - Bit 0: Ignore body roll rate (MUST be set for tracker)
 *          - Bit 2: Ignore body yaw rate (if clear, use packet.body_yaw_rate)
 *          - Bit 3: Ignore body pitch rate
 *          - Bit 4: Ignore body yaw rate
 *          - Bit 6: Ignore throttle (MUST be set for tracker)
 *          - Bit 7: Ignore attitude (MUST be clear for tracker)
 * 
 *          Quaternion conversion:
 *          - Attitude quaternion extracted from q[0..3] fields (w,x,y,z)
 *          - Quaternion represents desired body frame orientation
 *          - Converted to yaw/pitch angles by ModeGuided::set_angle()
 * 
 * @param[in] msg MAVLink message structure containing SET_ATTITUDE_TARGET
 * 
 * @note Only processed when tracker is in GUIDED mode - silently ignored otherwise
 * @note Roll commands are rejected - tracker only controls yaw and pitch axes
 * @note Continuous motion not yet supported - attitude must be specified
 * @note Yaw rate control optional based on type_mask bit 2
 * 
 * @warning Improper type_mask settings will cause command rejection without error message
 * 
 * Source: AntennaTracker/GCS_MAVLink_Tracker.cpp:82-120
 */
void GCS_MAVLINK_Tracker::handle_set_attitude_target(const mavlink_message_t &msg)
{
    // decode packet
    mavlink_set_attitude_target_t packet;
    mavlink_msg_set_attitude_target_decode(&msg, &packet);

    // exit if vehicle is not in Guided mode
    if (tracker.mode != &tracker.mode_guided) {
        return;
    }

    // sanity checks:
    if (!is_zero(packet.body_roll_rate)) {
        return;
    }
    if (!(packet.type_mask & (1<<0))) {
        // not told to ignore body roll rate
        return;
    }
    if (!(packet.type_mask & (1<<6))) {
        // not told to ignore throttle
        return;
    }
    if (packet.type_mask & (1<<7)) {
        // told to ignore attitude (we don't allow continuous motion yet)
        return;
    }
    if ((packet.type_mask & (1<<3)) && (packet.type_mask&(1<<4))) {
        // told to ignore both pitch and yaw rates - nothing to do?!
        return;
    }

    // Extract yaw rate usage from type_mask bit 2
    const bool use_yaw_rate = !(packet.type_mask & (1<<2));

    // Pass quaternion attitude and optional yaw rate to GUIDED mode handler
    tracker.mode_guided.set_angle(
        Quaternion(packet.q[0],packet.q[1],packet.q[2],packet.q[3]),
        use_yaw_rate,
        packet.body_yaw_rate);
}

/**
 * @brief Sends PID_TUNING telemetry messages for yaw and pitch controllers
 * 
 * @details Transmits real-time PID controller performance data to ground control stations
 *          for tuning visualization and analysis. Data includes target, actual position,
 *          and individual P, I, D, FF components allowing users to tune controller gains.
 * 
 *          PID data sent (per axis):
 *          - target: Desired angle in degrees
 *          - actual: Current angle in degrees
 *          - FF: Feed-forward component
 *          - P: Proportional component
 *          - I: Integral component
 *          - D: Derivative component
 *          - slew_rate: Rate limiting applied
 *          - Dmod: Modified derivative (with filtering)
 * 
 *          Telemetry control via gcs_pid_mask parameter:
 *          - Bit 0 (value 1): Send pitch PID tuning data
 *          - Bit 1 (value 2): Send yaw PID tuning data
 *          - Value 3: Send both pitch and yaw
 *          - Value 0: Disable PID telemetry
 * 
 *          Message priority handling:
 *          - Pitch sent first if enabled
 *          - Yaw sent second if buffer space available
 *          - Stops sending if buffer fills to avoid blocking other telemetry
 * 
 * @note Called periodically per telemetry stream rate configuration
 * @note Used with Mission Planner or MAVProxy PID tuning screens
 * @note Bandwidth consideration: sending both axes increases telemetry load
 * @note Buffer space checked between messages to prevent telemetry blocking
 * 
 * Source: AntennaTracker/GCS_MAVLink_Tracker.cpp:122-162
 */
void GCS_MAVLINK_Tracker::send_pid_tuning()
{
    const Parameters &g = tracker.g;

    // Pitch PID - send if bit 0 of gcs_pid_mask is set
    if (g.gcs_pid_mask & 1) {
        const AP_PIDInfo *pid_info = &g.pidPitch2Srv.get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_PITCH,
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

    // Yaw PID - send if bit 1 of gcs_pid_mask is set
    if (g.gcs_pid_mask & 2) {
        const AP_PIDInfo *pid_info = &g.pidYaw2Srv.get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_YAW,
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

/**
 * @brief Main MAVLink message dispatcher for tracker-specific position/pressure updates
 * 
 * @details This function intercepts specific MAVLink messages for target vehicle tracking
 *          before passing all messages to the base class handler. It implements "eavesdropping"
 *          on vehicle telemetry to extract position and altitude data needed for tracking.
 * 
 *          Message filtering logic:
 *          - If sysid_target is configured (non-zero), only processes messages from that system
 *          - This implements vehicle locking - tracker follows one specific vehicle
 *          - Messages from other systems are ignored for position updates
 *          - All messages still passed to base class for standard handling
 * 
 *          Intercepted messages:
 *          - HEARTBEAT: Triggers vehicle locking via mavlink_check_target()
 *          - GLOBAL_POSITION_INT: Target vehicle position update (lat/lon/alt/velocity)
 *          - SCALED_PRESSURE: Target vehicle barometric altitude update
 * 
 *          Vehicle locking behavior:
 *          - Tracker can lock onto first vehicle that sends HEARTBEAT
 *          - Once locked, only messages from that sysid are processed for tracking
 *          - Prevents tracker from jumping between multiple vehicles
 *          - sysid_target parameter controls locking (0 = auto-lock, other = specific vehicle)
 * 
 * @param[in] status MAVLink channel status (not used in this override)
 * @param[in] msg    Received MAVLink message to process
 * 
 * @note Called by MAVLink library for every received message on this channel
 * @note Position updates drive the tracking algorithm to point antenna at target
 * @note Pressure updates enable altitude-based tracking when target lacks GPS altitude
 * @note Base class handler (GCS_MAVLINK::packetReceived) processes all other messages
 * 
 * Source: AntennaTracker/GCS_MAVLink_Tracker.cpp:164-203
 */
void GCS_MAVLINK_Tracker::packetReceived(const mavlink_status_t &status,
                                         const mavlink_message_t &msg)
{
    // Filter by target sysid if configured - implements vehicle locking
    // return immediately if sysid doesn't match our target sysid
    if ((tracker.g.sysid_target != 0) && (tracker.g.sysid_target != msg.sysid)) {
        GCS_MAVLINK::packetReceived(status, msg);
        return;
    }

    switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
    {
        // Check if this vehicle should be our tracking target
        mavlink_check_target(msg);
        break;
    }

    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
    {
        // decode target vehicle position for tracking algorithm
        mavlink_global_position_int_t packet;
        mavlink_msg_global_position_int_decode(&msg, &packet);
        tracker.tracking_update_position(packet);
        break;
    }
    
    case MAVLINK_MSG_ID_SCALED_PRESSURE:
    {
        // decode target vehicle barometric altitude for tracking
        mavlink_scaled_pressure_t packet;
        mavlink_msg_scaled_pressure_decode(&msg, &packet);
        tracker.tracking_update_pressure(packet);
        break;
    }
    }
    // Pass all messages to base class for standard handling
    GCS_MAVLINK::packetReceived(status, msg);
}

/**
 * @brief Attempts to send a specific telemetry message on this MAVLink channel
 * 
 * @details Overrides base class to suppress wind telemetry (not applicable to trackers)
 *          and delegate all other message types to the base class implementation.
 * 
 * @param[in] id Message identifier from ap_message enum
 * 
 * @return true if message sent or suppressed, false if couldn't send
 * 
 * @note MSG_WIND returns true without sending - trackers don't measure wind
 * @note All other messages delegated to GCS_MAVLINK::try_send_message()
 * 
 * Source: AntennaTracker/GCS_MAVLink_Tracker.cpp:205-214
 */
bool GCS_MAVLINK_Tracker::try_send_message(enum ap_message id)
{
    switch(id) {
    case MSG_WIND: // other vehicles do something custom with wind:
        return true;
    default:
        return GCS_MAVLINK::try_send_message(id);
    }
    return true;
}

/**
 * @brief Locks onto a particular target vehicle and requests position data stream
 * 
 * @details Implements automatic vehicle locking mechanism. On first HEARTBEAT from a
 *          trackable vehicle, the tracker locks onto that vehicle's sysid and requests
 *          position telemetry at minimum 1Hz rate. This prevents tracker from switching
 *          between multiple vehicles and ensures reliable position updates.
 * 
 *          Vehicle type filtering:
 *          - Rejects: MAV_TYPE_ANTENNA_TRACKER (don't track other trackers)
 *          - Rejects: MAV_TYPE_GCS (don't track ground stations)
 *          - Rejects: MAV_TYPE_ONBOARD_CONTROLLER (don't track companion computers)
 *          - Rejects: MAV_TYPE_GIMBAL (don't track gimbals)
 *          - Accepts: All vehicle types (copter, plane, rover, boat, sub, etc.)
 * 
 *          Locking behavior:
 *          - Only locks if target not already set (tracker.target_set == false)
 *          - Only locks if sysid_target == 0 (auto-lock mode) or message matches sysid_target
 *          - Once locked, sysid_target parameter set to vehicle's sysid
 *          - Requests GLOBAL_POSITION_INT and SCALED_PRESSURE streams at 1Hz
 *          - Sets tracker.target_set flag preventing future relocks
 * 
 * @param[in] msg HEARTBEAT message from potential target vehicle
 * 
 * @note Called for every HEARTBEAT message received
 * @note Stream requests sent on all MAVLink channels to ensure vehicle receives
 * @note Stream request success not guaranteed - vehicle may deny or ignore request
 * @note Manual sysid_target configuration allows targeting specific vehicle in multi-vehicle scenarios
 * 
 * Source: AntennaTracker/GCS_MAVLink_Tracker.cpp:216-248
 */
void GCS_MAVLINK_Tracker::mavlink_check_target(const mavlink_message_t &msg)
{
    // exit immediately if the target has already been set
    if (tracker.target_set) {
        return;
    }

    // decode
    mavlink_heartbeat_t packet;
    mavlink_msg_heartbeat_decode(&msg, &packet);

    // exit immediately if this is not a vehicle we would track
    if ((packet.type == MAV_TYPE_ANTENNA_TRACKER) ||
        (packet.type == MAV_TYPE_GCS) ||
        (packet.type == MAV_TYPE_ONBOARD_CONTROLLER) ||
        (packet.type == MAV_TYPE_GIMBAL)) {
        return;
    }

    // set our sysid to the target, this ensures we lock onto a single vehicle
    if (tracker.g.sysid_target == 0) {
        tracker.g.sysid_target.set(msg.sysid);
    }

    // send data stream request to target on all channels
    //  Note: this doesn't check success for all sends meaning it's not guaranteed the vehicle's positions will be sent at 1hz
    tracker.gcs().request_datastream_position(msg.sysid, msg.compid);
    tracker.gcs().request_datastream_airpressure(msg.sysid, msg.compid);

    // flag target has been set
    tracker.target_set = true;
}

/**
 * @brief Handles MAVLink preflight barometer calibration command
 * 
 * @details Processes barometer calibration request and additionally triggers altitude
 *          difference zeroing for target tracking. This ensures altitude error starts
 *          from zero after baro calibration completes.
 * 
 *          Calibration sequence:
 *          1. Delegates to base class for standard baro calibration
 *          2. If calibration accepted, sets flag to zero altitude difference
 *          3. Altitude difference zeroed on next barometric pressure update from target
 * 
 * @param[in] msg MAVLink COMMAND_LONG message with calibration request
 * 
 * @return MAV_RESULT from base class calibration (ACCEPTED, FAILED, etc.)
 * 
 * @note Altitude difference used for elevation tracking and nav_controller_output
 * @note Zeroing prevents large initial altitude errors after calibration
 * @note Base class handles actual barometer calibration algorithm
 * 
 * Source: AntennaTracker/GCS_MAVLink_Tracker.cpp:250-258
 */
MAV_RESULT GCS_MAVLINK_Tracker::_handle_command_preflight_calibration_baro(const mavlink_message_t &msg)
{
    MAV_RESULT ret = GCS_MAVLINK::_handle_command_preflight_calibration_baro(msg);
    if (ret == MAV_RESULT_ACCEPTED) {
        // zero the altitude difference on next baro update
        tracker.nav_status.need_altitude_calibration = true;
    }
    return ret;
}

/**
 * @brief Handles MAVLink arm/disarm command for servo control
 * 
 * @details Arms or disarms tracker servos based on command parameter. When armed,
 *          servos actively control antenna pointing. When disarmed, servos are
 *          disabled for safety during maintenance.
 * 
 *          Command parameter interpretation:
 *          - param1 = 1.0: Arm servos (enable active pointing control)
 *          - param1 = 0.0: Disarm servos (disable servo outputs)
 *          - Other values: Rejected as unsupported
 * 
 * @param[in] packet MAVLink COMMAND_INT packet with arm/disarm request
 * 
 * @return MAV_RESULT_ACCEPTED if armed/disarmed successfully
 * @return MAV_RESULT_UNSUPPORTED if param1 not 0.0 or 1.0
 * 
 * @note Arming enables servo PWM outputs to move antenna
 * @note Disarming useful for safe manual antenna adjustment
 * @note Arming state reported in HEARTBEAT MAV_MODE_FLAG_SAFETY_ARMED
 * 
 * Source: AntennaTracker/GCS_MAVLink_Tracker.cpp:260-271
 */
MAV_RESULT GCS_MAVLINK_Tracker::handle_command_component_arm_disarm(const mavlink_command_int_t &packet)
{
    if (is_equal(packet.param1,1.0f)) {
        tracker.arm_servos();
        return MAV_RESULT_ACCEPTED;
    }
    if (is_zero(packet.param1))  {
        tracker.disarm_servos();
        return MAV_RESULT_ACCEPTED;
    }
    return MAV_RESULT_UNSUPPORTED;
}

/**
 * @brief Handles tracker-specific MAVLink command_int messages
 * 
 * @details Processes commands specific to antenna tracker operation, including
 *          servo test mode and mission start. Delegates unhandled commands to
 *          base class for standard command processing.
 * 
 *          Tracker-specific commands:
 * 
 *          MAV_CMD_DO_SET_SERVO:
 *          - Enters SERVOTEST mode
 *          - Sets specific servo to commanded PWM value
 *          - param1: Servo number
 *          - param2: PWM value (typically 1000-2000 microseconds)
 *          - Used for manual servo calibration and testing
 * 
 *          MAV_CMD_MISSION_START:
 *          - Enters AUTO tracking mode
 *          - Begins automatic target tracking
 *          - Sent by MAVProxy when "auto" command entered
 * 
 * @param[in] packet MAVLink COMMAND_INT packet structure
 * @param[in] msg    Original MAVLink message
 * 
 * @return MAV_RESULT_ACCEPTED if command executed successfully
 * @return MAV_RESULT_FAILED if servo setting failed
 * @return Result from base class for other commands
 * 
 * @note SERVOTEST mode allows direct servo control for calibration
 * @note AUTO mode requires valid target vehicle lock
 * @note Mode changes logged with ModeReason for diagnostics
 * 
 * Source: AntennaTracker/GCS_MAVLink_Tracker.cpp:273-294
 */
MAV_RESULT GCS_MAVLINK_Tracker::handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    switch(packet.command) {

    case MAV_CMD_DO_SET_SERVO:
        // ensure we are in servo test mode
        tracker.set_mode(tracker.mode_servotest, ModeReason::SERVOTEST);

        if (!tracker.mode_servotest.set_servo(packet.param1, packet.param2)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;

        // mavproxy/mavutil sends this when auto command is entered 
    case MAV_CMD_MISSION_START:
        tracker.set_mode(tracker.mode_auto, ModeReason::GCS_COMMAND);
        return MAV_RESULT_ACCEPTED;

    default:
        return GCS_MAVLINK::handle_command_int_packet(packet, msg);
    }
}

/**
 * @brief Main MAVLink message handler for tracker-specific messages
 * 
 * @details Routes incoming MAVLink messages to appropriate handler functions for
 *          tracker-specific processing. Handles attitude commands, position updates,
 *          manual control, and optionally mission upload for home position setting.
 * 
 *          Message routing:
 *          - SET_ATTITUDE_TARGET → handle_set_attitude_target() (GUIDED mode control)
 *          - MISSION_WRITE_PARTIAL_LIST → handle_message_mission_write_partial_list() (home setting)
 *          - MISSION_ITEM → handle_message_mission_item() (receive home waypoint)
 *          - MANUAL_CONTROL → handle_message_manual_control() (joystick input)
 *          - GLOBAL_POSITION_INT → handle_message_global_position_int() (target position)
 *          - SCALED_PRESSURE → handle_message_scaled_pressure() (target altitude)
 * 
 *          Mission upload feature (AP_TRACKER_SET_HOME_VIA_MISSION_UPLOAD_ENABLED):
 *          - Allows setting tracker home position via MAVProxy 'wp sethome' command
 *          - Only waypoint 0 (home) is accepted and stored
 *          - Enables tracker position updates without parameter changes
 * 
 * @param[in] msg MAVLink message to process
 * 
 * @note All messages also passed to base class (GCS_MAVLINK::handle_message)
 * @note Position messages from target vehicle drive tracking algorithm
 * @note Manual control provides direct yaw/pitch control in MANUAL mode
 * @note Message handlers may reject messages based on mode or validation
 * 
 * Source: AntennaTracker/GCS_MAVLink_Tracker.cpp:296-330
 */
void GCS_MAVLINK_Tracker::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {

    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
        handle_set_attitude_target(msg);
        break;

#if AP_TRACKER_SET_HOME_VIA_MISSION_UPLOAD_ENABLED
    // When mavproxy 'wp sethome' 
    case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
        handle_message_mission_write_partial_list(msg);
        break;

    // XXX receive a WP from GCS and store in EEPROM if it is HOME
    case MAVLINK_MSG_ID_MISSION_ITEM:
        handle_message_mission_item(msg);
        break;
#endif

    case MAVLINK_MSG_ID_MANUAL_CONTROL:
        handle_message_manual_control(msg);
        break;

    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        handle_message_global_position_int(msg);
        break;

    case MAVLINK_MSG_ID_SCALED_PRESSURE:
        handle_message_scaled_pressure(msg);
        break;
    }

    // Pass all messages to base class for standard processing
    GCS_MAVLINK::handle_message(msg);
}


#if AP_TRACKER_SET_HOME_VIA_MISSION_UPLOAD_ENABLED
/**
 * @brief Handles mission upload initiation for setting tracker home position
 * 
 * @details Processes MISSION_WRITE_PARTIAL_LIST message to detect home position
 *          update requests. When waypoint 0 (home) is being written, triggers
 *          waypoint reception sequence to receive and store new home location.
 * 
 *          Protocol sequence:
 *          1. Receive MISSION_WRITE_PARTIAL_LIST with start_index=0
 *          2. Set waypoint_receiving flag to expect MISSION_ITEM
 *          3. Send MSG_NEXT_MISSION_REQUEST_WAYPOINTS to request waypoint 0
 *          4. Wait for MISSION_ITEM message with home position
 * 
 * @param[in] msg MAVLink MISSION_WRITE_PARTIAL_LIST message
 * 
 * @note Only responds to start_index=0 (home position updates)
 * @note Other waypoint indices ignored - tracker doesn't use mission waypoints
 * @note Triggered by MAVProxy 'wp sethome' command
 * @note Feature enabled by AP_TRACKER_SET_HOME_VIA_MISSION_UPLOAD_ENABLED
 * 
 * Source: AntennaTracker/GCS_MAVLink_Tracker.cpp:333-345
 */
void GCS_MAVLINK_Tracker::handle_message_mission_write_partial_list(const mavlink_message_t &msg)
{
        // decode
        mavlink_mission_write_partial_list_t packet;
        mavlink_msg_mission_write_partial_list_decode(&msg, &packet);
        if (packet.start_index == 0)
        {
            // New home at wp index 0. Ask for it
            waypoint_receiving = true;
            send_message(MSG_NEXT_MISSION_REQUEST_WAYPOINTS);
        }
}

/**
 * @brief Handles mission item upload for setting tracker home position
 * 
 * @details Receives and validates MISSION_ITEM message containing new home position
 *          for tracker. Supports multiple coordinate frames and validates mission
 *          upload sequence state. Only processes waypoint 0 (home position).
 * 
 *          Supported coordinate frames:
 *          - MAV_FRAME_MISSION/GLOBAL: Absolute lat/lon/alt in degrees and meters
 *          - MAV_FRAME_LOCAL_NED: Local NED frame relative to current home (North/East/Down in meters)
 *          - MAV_FRAME_LOCAL: Local frame relative to current home (North/East/Up in meters)
 *          - MAV_FRAME_GLOBAL_RELATIVE_ALT: Absolute lat/lon, relative altitude to home
 * 
 *          Coordinate conversions:
 *          - Latitude/longitude: degrees * 1e7 → integer representation
 *          - Altitude: meters * 100 → centimeters
 *          - Local coordinates: meters → lat/lon via RADIUS_OF_EARTH approximation
 * 
 *          Validation checks:
 *          - Frame must be supported (rejects unsupported frames)
 *          - Must be in waypoint receiving state (protocol sequence check)
 *          - Must be sequence number 0 (home waypoint only)
 *          - Home position must set successfully
 * 
 *          Error handling:
 *          - Sends MISSION_ACK with appropriate result code
 *          - Clears waypoint_receiving flag on success or error
 *          - Reports success via text message "New HOME received"
 * 
 * @param[in] msg MAVLink MISSION_ITEM message with home position
 * 
 * @note Only waypoint 0 accepted - other sequence numbers rejected
 * @note Local frame conversions use spherical Earth approximation
 * @note Feature enabled by AP_TRACKER_SET_HOME_VIA_MISSION_UPLOAD_ENABLED
 * @note Always sends MISSION_ACK regardless of success/failure
 * 
 * Source: AntennaTracker/GCS_MAVLink_Tracker.cpp:347-438
 */
void GCS_MAVLINK_Tracker::handle_message_mission_item(const mavlink_message_t &msg)
{
        mavlink_mission_item_t packet;
        MAV_MISSION_RESULT result = MAV_MISSION_ACCEPTED;

        mavlink_msg_mission_item_decode(&msg, &packet);

        Location tell_command;

        switch (packet.frame)
        {
        case MAV_FRAME_MISSION:
        case MAV_FRAME_GLOBAL:
        {
            tell_command = Location{
                int32_t(1.0e7f*packet.x), // in as DD converted to * t7
                int32_t(1.0e7f*packet.y), // in as DD converted to * t7
                int32_t(packet.z*1.0e2f), // in as m converted to cm
                Location::AltFrame::ABSOLUTE
            };
            break;
        }

#ifdef MAV_FRAME_LOCAL_NED
        case MAV_FRAME_LOCAL_NED:                         // local (relative to home position)
        {
            tell_command = Location{
                int32_t(1.0e7f*degrees(packet.x/(RADIUS_OF_EARTH*cosf(radians(home.lat/1.0e7f)))) + home.lat),
                int32_t(1.0e7f*degrees(packet.y/RADIUS_OF_EARTH) + home.lng),
                int32_t(-packet.z*1.0e2f),
                Location::AltFrame::ABOVE_HOME
            };
            break;
        }
#endif

#ifdef MAV_FRAME_LOCAL
        case MAV_FRAME_LOCAL:                         // local (relative to home position)
        {
            tell_command = {
                int32_t(1.0e7f*degrees(packet.x/(RADIUS_OF_EARTH*cosf(radians(home.lat/1.0e7f)))) + home.lat),
                int32_t(1.0e7f*degrees(packet.y/RADIUS_OF_EARTH) + home.lng),
                int32_t(packet.z*1.0e2f),
                Location::AltFrame::ABOVE_HOME
            };
            break;
        }
#endif

        case MAV_FRAME_GLOBAL_RELATIVE_ALT:                         // absolute lat/lng, relative altitude
        {
            tell_command = {
                int32_t(1.0e7f * packet.x), // in as DD converted to * t7
                int32_t(1.0e7f * packet.y), // in as DD converted to * t7
                int32_t(packet.z * 1.0e2f),
                Location::AltFrame::ABOVE_HOME
            };
            break;
        }

        default:
            result = MAV_MISSION_UNSUPPORTED_FRAME;
            break;
        }

        if (result != MAV_MISSION_ACCEPTED) goto mission_failed;

        // Check if receiving waypoints (mission upload expected)
        if (!waypoint_receiving) {
            result = MAV_MISSION_ERROR;
            goto mission_failed;
        }

        // check if this is the HOME wp
        if (packet.seq == 0) {
            if (!tracker.set_home(tell_command, false)) {
                result = MAV_MISSION_ERROR;
                goto mission_failed;
            }
            send_text(MAV_SEVERITY_INFO,"New HOME received");
            waypoint_receiving = false;
        }

mission_failed:
        // send ACK (including in success case)
        mavlink_msg_mission_ack_send(
            chan,
            msg.sysid,
            msg.compid,
            result,
            MAV_MISSION_TYPE_MISSION);
}
#endif

/**
 * @brief Handles manual control joystick input for direct tracker control
 * 
 * @details Decodes MANUAL_CONTROL message and passes joystick inputs to tracking
 *          system for direct yaw/pitch control. Used in MANUAL mode for operator
 *          override of automatic tracking.
 * 
 *          MANUAL_CONTROL fields:
 *          - x: Forward/back axis (typically pitch control)
 *          - y: Left/right axis (typically yaw control)
 *          - z: Throttle (not used by tracker)
 *          - r: Rotation (alternative yaw control)
 *          - buttons: Button state (not used by tracker)
 * 
 * @param[in] msg MAVLink MANUAL_CONTROL message with joystick data
 * 
 * @note Joystick axes scaled from -1000 to +1000 per MAVLink spec
 * @note Tracking system converts joystick values to servo commands
 * @note Only active in MANUAL mode - ignored in AUTO/GUIDED modes
 * 
 * Source: AntennaTracker/GCS_MAVLink_Tracker.cpp:441-446
 */
void GCS_MAVLINK_Tracker::handle_message_manual_control(const mavlink_message_t &msg)
{
        mavlink_manual_control_t packet;
        mavlink_msg_manual_control_decode(&msg, &packet);
        tracker.tracking_manual_control(packet);
}

/**
 * @brief Handles target vehicle position update for tracking algorithm
 * 
 * @details Decodes GLOBAL_POSITION_INT message containing target vehicle's GPS
 *          position, altitude, and velocity. This data drives the tracking algorithm
 *          to calculate required antenna pointing angles.
 * 
 *          Position data includes:
 *          - lat: Latitude in degrees * 1e7
 *          - lon: Longitude in degrees * 1e7
 *          - alt: Altitude above MSL in millimeters
 *          - relative_alt: Altitude above home in millimeters
 *          - vx/vy/vz: Velocity in cm/s (North/East/Down)
 *          - hdg: Heading in centidegrees
 * 
 * @param[in] msg MAVLink GLOBAL_POSITION_INT message from target vehicle
 * 
 * @note Position updates typically sent at 1-10 Hz by target vehicle
 * @note Tracking algorithm calculates bearing, pitch, and distance to target
 * @note Velocity data can enable predictive tracking (not currently implemented)
 * 
 * Source: AntennaTracker/GCS_MAVLink_Tracker.cpp:448-454
 */
void GCS_MAVLINK_Tracker::handle_message_global_position_int(const mavlink_message_t &msg)
{
        // decode target vehicle position update
        mavlink_global_position_int_t packet;
        mavlink_msg_global_position_int_decode(&msg, &packet);
        tracker.tracking_update_position(packet);
}

/**
 * @brief Handles target vehicle barometric pressure update for altitude tracking
 * 
 * @details Decodes SCALED_PRESSURE message containing target vehicle's barometric
 *          altitude. Used when GPS altitude is unreliable or unavailable, providing
 *          alternative altitude source for elevation tracking.
 * 
 *          Pressure data includes:
 *          - press_abs: Absolute pressure in hectopascals
 *          - press_diff: Differential pressure (airspeed sensors)
 *          - temperature: Temperature in centidegrees Celsius
 * 
 * @param[in] msg MAVLink SCALED_PRESSURE message from target vehicle
 * 
 * @note Barometric altitude calculated from pressure using standard atmosphere model
 * @note ALT_SOURCE parameter selects between barometric and GPS altitude
 * @note Pressure updates typically sent at 1-5 Hz by target vehicle
 * @note Used for altitude difference calculation in send_nav_controller_output()
 * 
 * Source: AntennaTracker/GCS_MAVLink_Tracker.cpp:456-461
 */
void GCS_MAVLINK_Tracker::handle_message_scaled_pressure(const mavlink_message_t &msg)
{
        mavlink_scaled_pressure_t packet;
        mavlink_msg_scaled_pressure_decode(&msg, &packet);
        tracker.tracking_update_pressure(packet);
}

/**
 * @brief Sends tracker's own GPS position to ground control station
 * 
 * @details Transmits GLOBAL_POSITION_INT message containing tracker's current location.
 *          Behavior differs based on whether tracker is stationary or mobile:
 * 
 *          Stationary tracker (tracker.stationary == true):
 *          - Sends fixed position from tracker.current_loc
 *          - Velocity always zero (tracker not moving)
 *          - Relative altitude set to zero (tracker is the home reference)
 *          - Heading from AHRS (antenna pointing direction)
 * 
 *          Mobile tracker (tracker.stationary == false):
 *          - Delegates to base class for full GPS solution
 *          - Includes actual velocity and GPS-derived position
 *          - Used for tracker-on-vehicle scenarios
 * 
 *          Message fields (stationary mode):
 *          - time_boot_ms: System uptime in milliseconds
 *          - lat/lon: Position in degrees * 1e7
 *          - alt: Altitude in millimeters above MSL
 *          - relative_alt: Always 0 (tracker is reference point)
 *          - vx/vy/vz: Always 0 (stationary)
 *          - hdg: Antenna heading in centidegrees
 * 
 * @note Stationary is typical configuration - tracker at fixed ground location
 * @note Mobile tracker mode allows tracker mounted on moving vehicle
 * @note Position used by GCS to display tracker location on map
 * @note Heading shows which direction antenna is currently pointing
 * 
 * Source: AntennaTracker/GCS_MAVLink_Tracker.cpp:463-482
 */
void GCS_MAVLINK_Tracker::send_global_position_int()
{
    if (!tracker.stationary) {
        GCS_MAVLINK::send_global_position_int();
        return;
    }

    mavlink_msg_global_position_int_send(
        chan,
        AP_HAL::millis(),
        tracker.current_loc.lat,  // in 1E7 degrees
        tracker.current_loc.lng,  // in 1E7 degrees
        tracker.current_loc.alt,  // millimeters above ground/sea level
        0,                        // millimeters above home
        0,                        // X speed cm/s (+ve North)
        0,                        // Y speed cm/s (+ve East)
        0,                        // Z speed cm/s (+ve Down)
        tracker.ahrs.yaw_sensor); // compass heading in 1/100 degree
}

/**
 * @brief Sends available flight mode information to ground control station
 * 
 * @details Transmits AVAILABLE_MODES message for a specific mode index. This allows
 *          ground control stations to discover all available tracker modes and display
 *          them to the user for mode selection. Called repeatedly with incrementing
 *          index to enumerate all modes.
 * 
 *          Available tracker modes:
 *          - MANUAL: Direct pilot control of yaw/pitch servos
 *          - STOP: Servos hold current position
 *          - SCAN: Automated horizon scanning pattern
 *          - GUIDED: Accept external attitude commands
 *          - SERVOTEST: Individual servo testing mode
 *          - AUTO: Automatic target vehicle tracking
 *          - INITIALISING: Startup calibration mode
 * 
 *          Message contents:
 *          - mode_count: Total number of modes (7 for tracker)
 *          - mode_index: Current mode being reported (1-based)
 *          - standard_mode: Always NON_STANDARD (tracker modes are custom)
 *          - custom_mode: Mode number from Mode::Number enum
 *          - properties: Always 0 (no special properties)
 *          - mode_name: Human-readable mode name string
 * 
 * @param[in] index Mode index to report (1-based, 1 to mode_count)
 * 
 * @return uint8_t Total number of available modes (always 7)
 * 
 * @note Index starts at 1 per MAVLink convention (not 0)
 * @note Returns mode_count even if index out of range
 * @note Ground station iterates index from 1 until mode_count reached
 * @note Mode number (custom_mode) used for mode change commands
 * @note Mode name displayed to user in GCS mode selection UI
 * 
 * Source: AntennaTracker/GCS_MAVLink_Tracker.cpp:484-522
 */
uint8_t GCS_MAVLINK_Tracker::send_available_mode(uint8_t index) const
{
    const Mode* modes[] {
        &tracker.mode_manual,
        &tracker.mode_stop,
        &tracker.mode_scan,
        &tracker.mode_guided,
        &tracker.mode_servotest,
        &tracker.mode_auto,
        &tracker.mode_initialising,
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
