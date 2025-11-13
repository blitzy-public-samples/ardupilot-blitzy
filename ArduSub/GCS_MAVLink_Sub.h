/**
 * @file GCS_MAVLink_Sub.h
 * @brief ArduSub-specific MAVLink ground control station backend
 * 
 * This file defines the GCS_MAVLINK_Sub class, which provides the ArduSub-specific
 * implementation of the MAVLink communication protocol for underwater vehicles.
 * It extends the base GCS_MAVLINK class with underwater vehicle-specific message
 * handling, telemetry customizations, and command processing.
 * 
 * Key responsibilities:
 * - Handle Sub-specific MAVLink commands (motor test, yaw control, etc.)
 * - Customize telemetry streams for underwater operations
 * - Override altitude reporting for depth-based navigation
 * - Provide Sub-specific vehicle capabilities and status
 * - Adapt navigation controller output for underwater vehicles
 * 
 * @note This class is instantiated for each active MAVLink channel
 * @see GCS_MAVLINK for base MAVLink functionality
 * @see GCS_Sub for the Sub-specific GCS manager
 * 
 * Source: ArduSub/GCS_MAVLink_Sub.h
 */

#pragma once

#include <GCS_MAVLink/GCS.h>

/**
 * @class GCS_MAVLINK_Sub
 * @brief Sub-specific MAVLink backend providing underwater vehicle communication
 * 
 * @details GCS_MAVLINK_Sub extends the base GCS_MAVLINK class to provide ArduSub-specific
 *          MAVLink protocol handling for underwater vehicles. This includes:
 * 
 *          - Underwater-specific command handling (depth hold, attitude control)
 *          - Depth-based altitude reporting instead of barometric altitude
 *          - Custom telemetry streams adapted for underwater operations
 *          - Sub-specific vehicle capabilities and flight modes
 *          - Temperature sensor data from pressure sensors
 * 
 *          The class overrides numerous base class methods to adapt MAVLink communication
 *          for underwater environments where GPS is unavailable and depth replaces altitude
 *          as the primary vertical reference.
 * 
 *          Thread Safety: Methods are called from the scheduler at various rates
 *                        (typically 50Hz for fast telemetry, slower for low-priority data)
 * 
 * @note Inherits using declaration for base constructors
 * @warning Modifications to message handling can affect ground station compatibility
 */
class GCS_MAVLINK_Sub : public GCS_MAVLINK {

public:

    /**
     * @brief Inherit base class constructors
     * @note Using declaration to inherit all GCS_MAVLINK constructors
     */
    using GCS_MAVLINK::GCS_MAVLINK;

protected:

    /**
     * @brief Handle MAV_CMD_DO_FLIGHTTERMINATION command for emergency termination
     * 
     * @details Processes flight termination requests for underwater vehicles.
     *          For Sub, this typically disarms the vehicle and stops all motor output
     *          to prevent uncontrolled operation in case of critical failure.
     * 
     * @param[in] packet MAVLink command packet containing termination parameters
     * 
     * @return MAV_RESULT_ACCEPTED if termination initiated successfully
     * @return MAV_RESULT_FAILED if termination cannot be performed
     * 
     * @warning This is a safety-critical function - improper handling could leave vehicle armed
     * @see handle_command_int_packet()
     */
    MAV_RESULT handle_flight_termination(const mavlink_command_int_t &packet) override;

    /**
     * @brief Handle MAV_CMD_DO_SET_ROI to point camera/vehicle at region of interest
     * 
     * @details For Sub, ROI commands typically control camera gimbal pointing or
     *          yaw orientation to face a specific location. Depth/altitude component
     *          is adapted for underwater coordinate system.
     * 
     * @param[in] roi_loc Target location for region of interest (NED frame)
     * 
     * @return MAV_RESULT_ACCEPTED if ROI set successfully
     * @return MAV_RESULT_UNSUPPORTED if ROI mode not supported
     * @return MAV_RESULT_FAILED if location invalid
     * 
     * @note ROI commands may control both vehicle yaw and camera gimbal
     */
    MAV_RESULT handle_command_do_set_roi(const Location &roi_loc) override;

    /**
     * @brief Handle barometer/depth sensor calibration command
     * 
     * @details Processes preflight barometer calibration for Sub. Since underwater
     *          vehicles use pressure sensors for depth measurement, this calibration
     *          is critical for accurate depth hold and altitude reporting.
     * 
     * @param[in] msg MAVLink message containing calibration command parameters
     * 
     * @return MAV_RESULT_ACCEPTED if calibration initiated
     * @return MAV_RESULT_TEMPORARILY_REJECTED if vehicle not in appropriate state
     * @return MAV_RESULT_FAILED if calibration cannot be performed
     * 
     * @warning Vehicle must be stationary at known depth for accurate calibration
     * @note Calibration typically takes several seconds to complete
     */
    MAV_RESULT _handle_command_preflight_calibration_baro(const mavlink_message_t &msg) override;

    /**
     * @brief Handle general preflight calibration commands
     * 
     * @details Central handler for various preflight calibration commands including
     *          compass, accelerometer, gyro, and depth sensor calibration. Routes
     *          calibration requests to appropriate subsystems.
     * 
     * @param[in] packet MAVLink command packet with calibration type and parameters
     * @param[in] msg Original MAVLink message for context
     * 
     * @return MAV_RESULT indicating calibration status (accepted, rejected, failed)
     * 
     * @note Different calibrations have different environmental requirements
     * @warning Some calibrations require vehicle rotation or specific positioning
     */
    MAV_RESULT _handle_command_preflight_calibration(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;

    /**
     * @brief Main dispatcher for MAVLink COMMAND_INT messages
     * 
     * @details Routes incoming COMMAND_INT packets to appropriate Sub-specific handlers.
     *          Overrides base implementation to add underwater vehicle-specific commands
     *          and modify behavior for underwater operations.
     * 
     * @param[in] packet MAVLink command_int packet with command ID and parameters
     * @param[in] msg Original MAVLink message for context and acknowledgment
     * 
     * @return MAV_RESULT indicating command processing status
     * 
     * @note Called from main message handling loop at message reception rate
     * @see handle_MAV_CMD_* methods for specific command handlers
     */
    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;

    /**
     * @brief Handle MAV_CMD_DO_REPOSITION to move vehicle to new position
     * 
     * @details Processes reposition commands for underwater vehicles. Unlike aerial
     *          vehicles, Sub repositioning accounts for neutral buoyancy and current
     *          effects. Command may be rejected if vehicle is not in appropriate mode.
     * 
     * @param[in] packet Command packet containing target position (lat/lon/depth)
     * 
     * @return MAV_RESULT_ACCEPTED if reposition command accepted and mode switched
     * @return MAV_RESULT_DENIED if current mode does not support repositioning
     * @return MAV_RESULT_FAILED if position invalid or out of range
     * 
     * @note Requires GPS fix for horizontal positioning (when available)
     * @note Depth component used instead of altitude for underwater vehicles
     */
    MAV_RESULT handle_command_int_do_reposition(const mavlink_command_int_t &packet);

    /**
     * @brief Send SCALED_PRESSURE3 message with on-board temperature data
     * 
     * @details Override of base implementation to send on-board temperature sensor
     *          readings instead of third pressure sensor. Useful for monitoring
     *          internal electronics temperature in underwater enclosures.
     * 
     * @note Called periodically by telemetry scheduler at configured stream rate
     * @note Temperature data helps monitor thermal management in sealed enclosures
     */
    void send_scaled_pressure3() override;

    /**
     * @brief Get global position altitude for GLOBAL_POSITION_INT message
     * 
     * @details Returns current depth (as negative altitude) for underwater vehicles.
     *          Overrides base implementation to use depth sensor instead of
     *          barometric altitude or GPS altitude.
     * 
     * @return Altitude in millimeters (negative values indicate depth below surface)
     * 
     * @note Depth is reported as negative altitude following MAVLink conventions
     * @note Value is relative to surface (0mm = surface, -1000mm = 1m depth)
     */
    int32_t global_position_int_alt() const override;

    /**
     * @brief Get relative altitude for GLOBAL_POSITION_INT message
     * 
     * @details Returns depth relative to dive start position for underwater vehicles.
     *          Provides reference for depth changes during mission execution.
     * 
     * @return Relative altitude in millimeters (negative = deeper than start)
     * 
     * @note Relative altitude reset at arming or when reference position set
     */
    int32_t global_position_int_relative_alt() const override;

    /**
     * @brief Send startup banner with vehicle information
     * 
     * @details Sends vehicle type, version, and configuration information to ground
     *          control station on connection establishment. Sub-specific banner includes
     *          underwater vehicle capabilities and sensor configuration.
     * 
     * @note Called once when MAVLink connection established
     */
    void send_banner() override;

    /**
     * @brief Send NAV_CONTROLLER_OUTPUT message with navigation status
     * 
     * @details Provides navigation controller state for underwater vehicles including
     *          target bearing, distance to waypoint, depth error, and cross-track error.
     *          Adapted for underwater navigation where GPS may be unavailable.
     * 
     * @note Called at NAV_CONTROLLER_OUTPUT stream rate (typically 2-10Hz)
     * @note Values adapted for underwater coordinate system and available sensors
     */
    void send_nav_controller_output() const override;

    /**
     * @brief Send PID_TUNING message with controller performance data
     * 
     * @details Streams PID controller state for real-time tuning of attitude,
     *          depth, and position controllers. Allows ground station to display
     *          controller performance and assist with tuning parameter adjustment.
     * 
     * @note Called at PID_TUNING stream rate when tuning is active
     * @note Useful for in-water tuning of control loops
     */
    void send_pid_tuning() override;

    /**
     * @brief Report vehicle capabilities to ground control station
     * 
     * @details Returns bitfield of MAVLink capabilities supported by ArduSub,
     *          including supported commands, mission types, and flight modes.
     *          Underwater vehicle capabilities differ from aerial vehicles
     *          (no terrain following, different altitude references).
     * 
     * @return Bitfield of MAV_PROTOCOL_CAPABILITY flags indicating supported features
     * 
     * @note Capabilities affect what commands ground station will offer to user
     * @see MAV_PROTOCOL_CAPABILITY enum for capability flags
     */
    uint64_t capabilities() const override;

    /**
     * @brief Send available flight mode information to ground control station
     * 
     * @details Returns information about the flight mode at the given index for
     *          AVAILABLE_MODES message. Allows ground stations to enumerate all
     *          Sub-specific flight modes (Manual, Stabilize, Depth Hold, etc.)
     *          and their properties.
     * 
     * @param[in] index Mode index to query (1-based, not mode number)
     * 
     * @return Total number of available modes (same for all valid indices)
     * @return 0 if index is out of range
     * 
     * @note Index starts at 1, not 0
     * @note Called iteratively by base class to enumerate all modes
     * @note Sub modes include Manual, Stabilize, Depth Hold, Position Hold, etc.
     */
    uint8_t send_available_mode(uint8_t index) const override;

private:

    /**
     * @brief Process incoming MAVLink messages
     * 
     * @details Main message handler that dispatches incoming MAVLink messages to
     *          appropriate Sub-specific handlers. Routes messages that require
     *          underwater vehicle-specific processing.
     * 
     * @param[in] msg MAVLink message to process
     * 
     * @note Called from main loop for each received MAVLink message on this channel
     * @note Override allows Sub-specific handling before calling base implementation
     */
    void handle_message(const mavlink_message_t &msg) override;

    /**
     * @brief Handle guided mode position/attitude requests from ground station
     * 
     * @details Processes guided mode commands for underwater vehicle control,
     *          converting mission command format to vehicle control targets.
     *          Handles depth targets instead of altitude for underwater operations.
     * 
     * @param[in,out] cmd Mission command to process and potentially modify
     * 
     * @return true if guided request accepted and processed
     * @return false if request cannot be handled in current state
     * 
     * @note Vehicle must be in guided mode for these commands to take effect
     */
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override;

    /**
     * @brief Attempt to send a queued telemetry message
     * 
     * @details Called by telemetry scheduler to send queued messages based on
     *          stream rates and priority. Sub-specific implementation handles
     *          underwater vehicle-specific telemetry data and formatting.
     * 
     * @param[in] id Message identifier from ap_message enum
     * 
     * @return true if message sent successfully
     * @return false if message cannot be sent (buffer full, not ready, etc.)
     * 
     * @note Called repeatedly by scheduler to drain telemetry queue
     * @note Message priority and rate limiting handled by scheduler
     */
    bool try_send_message(enum ap_message id) override;

    /**
     * @brief Send vehicle information messages
     * 
     * @details Sends Sub-specific vehicle information including version,
     *          capabilities, and configuration to ground control station.
     * 
     * @return true if info messages sent successfully
     * @return false if sending failed
     * 
     * @note Called during connection establishment
     */
    bool send_info(void);

    /**
     * @brief Get base flight mode flags for HEARTBEAT message
     * 
     * @details Returns MAV_MODE_FLAG bitfield representing current vehicle state
     *          including armed status, guided mode, stabilization, and manual control.
     *          Sub-specific interpretation for underwater vehicle modes.
     * 
     * @return Bitfield of MAV_MODE_FLAG values indicating current mode
     * 
     * @note Sent in every HEARTBEAT message (typically 1Hz)
     * @see MAV_MODE_FLAG enum for flag definitions
     */
    uint8_t base_mode() const override;

    /**
     * @brief Get vehicle system status for HEARTBEAT message
     * 
     * @details Returns current MAV_STATE (standby, active, critical, emergency, etc.)
     *          based on Sub vehicle health, arming state, and error conditions.
     * 
     * @return MAV_STATE enum value indicating system status
     * 
     * @note Affects ground station display and warnings
     * @note Sent in every HEARTBEAT message (typically 1Hz)
     */
    MAV_STATE vehicle_system_status() const override;

    /**
     * @brief Get throttle percentage for VFR_HUD message
     * 
     * @details Returns current throttle level as percentage for heads-up display.
     *          For Sub, represents vertical thrust/motor output level.
     * 
     * @return Throttle percentage (0-100, can be negative for downward thrust)
     * 
     * @note VFR_HUD typically sent at 5-10Hz for display updates
     */
    int16_t vfr_hud_throttle() const override;

    /**
     * @brief Get altitude for VFR_HUD message
     * 
     * @details Returns current depth (as altitude) for VFR heads-up display.
     *          For underwater vehicles, returns depth with appropriate sign convention.
     * 
     * @return Altitude in meters (negative values indicate depth below surface)
     * 
     * @note Used by ground station for primary altitude display
     * @note VFR_HUD typically sent at 5-10Hz
     */
    float vfr_hud_alt() const override;

    /**
     * @brief Handle MAV_CMD_CONDITION_YAW to set target yaw angle or rate
     * 
     * @details Processes yaw control commands for underwater vehicles. Can command
     *          either absolute yaw angle or yaw rate depending on packet parameters.
     *          Yaw control important for camera pointing and navigation.
     * 
     * @param[in] packet Command packet with target yaw (degrees) or rate (deg/s)
     * 
     * @return MAV_RESULT_ACCEPTED if yaw command accepted
     * @return MAV_RESULT_DENIED if vehicle mode doesn't support yaw control
     * 
     * @note Yaw control available in most Sub flight modes
     */
    MAV_RESULT handle_MAV_CMD_CONDITION_YAW(const mavlink_command_int_t &packet);

    /**
     * @brief Handle MAV_CMD_MISSION_START to begin mission execution
     * 
     * @details Initiates mission execution from specified mission item index.
     *          For Sub, ensures vehicle is in appropriate state (armed, auto mode)
     *          before beginning mission.
     * 
     * @param[in] packet Command packet with starting mission item index
     * 
     * @return MAV_RESULT_ACCEPTED if mission started successfully
     * @return MAV_RESULT_DENIED if vehicle cannot start mission (not armed, no mission, etc.)
     * 
     * @note Switches vehicle to Auto mode if accepted
     */
    MAV_RESULT handle_MAV_CMD_MISSION_START(const mavlink_command_int_t &packet);

    /**
     * @brief Handle MAV_CMD_DO_CHANGE_SPEED to adjust vehicle speed
     * 
     * @details Changes target speed for current and subsequent mission waypoints.
     *          For underwater vehicles, adjusts horizontal translation speed.
     * 
     * @param[in] packet Command packet with speed type and target value (m/s)
     * 
     * @return MAV_RESULT_ACCEPTED if speed change applied
     * @return MAV_RESULT_DENIED if speed change not supported in current mode
     * @return MAV_RESULT_FAILED if speed value out of valid range
     * 
     * @note Speed limits enforced based on vehicle configuration
     */
    MAV_RESULT handle_MAV_CMD_DO_CHANGE_SPEED(const mavlink_command_int_t &packet);

    /**
     * @brief Handle MAV_CMD_DO_MOTOR_TEST to test individual motors
     * 
     * @details Allows ground station to command individual motor/thruster testing
     *          for diagnostic purposes. Critical for verifying motor direction and
     *          functionality before diving.
     * 
     * @param[in] packet Command with motor number, throttle, and duration
     * 
     * @return MAV_RESULT_ACCEPTED if motor test started
     * @return MAV_RESULT_TEMPORARILY_REJECTED if vehicle is armed
     * @return MAV_RESULT_FAILED if motor number invalid
     * 
     * @warning Motor testing should only be done when vehicle is secured
     * @note Vehicle must be disarmed for safety
     */
    MAV_RESULT handle_MAV_CMD_DO_MOTOR_TEST(const mavlink_command_int_t &packet);

    /**
     * @brief Handle MAV_CMD_NAV_LOITER_UNLIM to hold position indefinitely
     * 
     * @details Commands vehicle to maintain current position (or specified position)
     *          indefinitely. For Sub, maintains depth and horizontal position using
     *          available sensors.
     * 
     * @param[in] packet Command packet with target position (or empty for current)
     * 
     * @return MAV_RESULT_ACCEPTED if loiter command accepted
     * @return MAV_RESULT_DENIED if positioning not available (no GPS, no depth sensor)
     * 
     * @note Requires depth hold capability at minimum
     */
    MAV_RESULT handle_MAV_CMD_NAV_LOITER_UNLIM(const mavlink_command_int_t &packet);

    /**
     * @brief Handle MAV_CMD_NAV_LAND to perform landing sequence
     * 
     * @details For underwater vehicles, "landing" means controlled descent to
     *          specified depth or bottom. Executes gentle descent with position hold.
     * 
     * @param[in] packet Command packet with target depth/altitude
     * 
     * @return MAV_RESULT_ACCEPTED if landing sequence initiated
     * @return MAV_RESULT_DENIED if unable to perform landing
     * 
     * @note Landing sequence can be aborted by mode change
     * @note For Sub, this is surface or bottom approach maneuver
     */
    MAV_RESULT handle_MAV_CMD_NAV_LAND(const mavlink_command_int_t &packet);

#if HAL_HIGH_LATENCY2_ENABLED
    /**
     * @brief Get target altitude for HIGH_LATENCY2 message
     * 
     * @details Returns target depth (as altitude) for bandwidth-optimized telemetry.
     *          HIGH_LATENCY2 used for satellite or low-bandwidth links where
     *          transmission costs are high.
     * 
     * @return Target altitude in meters (negative for depth targets)
     * 
     * @note HIGH_LATENCY2 messages sent at very low rate (typically 0.1-1Hz)
     * @note Used for remote operations over satellite links
     */
    int16_t high_latency_target_altitude() const override;

    /**
     * @brief Get target heading for HIGH_LATENCY2 message
     * 
     * @details Returns target yaw heading in degrees for low-bandwidth telemetry.
     * 
     * @return Target heading in degrees (0-359, 0=North)
     * 
     * @note Part of HIGH_LATENCY2 compact telemetry format
     */
    uint8_t high_latency_tgt_heading() const override;

    /**
     * @brief Get distance to target for HIGH_LATENCY2 message
     * 
     * @details Returns distance to current navigation target (waypoint, home, etc.)
     *          for low-bandwidth telemetry.
     * 
     * @return Distance to target in meters (scaled to fit uint16_t)
     * 
     * @note Distance may be saturated at maximum uint16_t value for very distant targets
     */
    uint16_t high_latency_tgt_dist() const override;

    /**
     * @brief Get target airspeed for HIGH_LATENCY2 message
     * 
     * @details Returns target horizontal speed for underwater vehicles in
     *          LOW-bandwidth telemetry format. "Airspeed" terminology inherited
     *          from MAVLink, but represents water speed for Sub.
     * 
     * @return Target speed in m/s (scaled to uint8_t)
     * 
     * @note For Sub, this represents horizontal translation speed through water
     */
    uint8_t high_latency_tgt_airspeed() const override;
#endif // HAL_HIGH_LATENCY2_ENABLED
};
