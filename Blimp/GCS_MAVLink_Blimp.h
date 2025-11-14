/**
 * @file GCS_MAVLink_Blimp.h
 * @brief MAVLink interface class definition for Blimp vehicle
 * 
 * @details This file defines the GCS_MAVLINK_Blimp class which provides
 *          blimp-specific MAVLink communication with ground control stations.
 *          It overrides base GCS_MAVLINK methods to implement lighter-than-air
 *          vehicle-specific telemetry, command processing, and status reporting.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <GCS_MAVLink/GCS.h>

#include "defines.h"

/**
 * @class GCS_MAVLINK_Blimp
 * @brief Blimp-specific MAVLink interface implementation
 * 
 * @details This class extends the base GCS_MAVLINK class to provide
 *          lighter-than-air vehicle-specific MAVLink communication.
 *          
 *          Key responsibilities:
 *          - Handles telemetry streaming for blimp state (position, velocity, attitude)
 *          - Processes ground station commands specific to blimp operation
 *          - Reports vehicle status and capabilities for multicopter-like flight control
 *          - Implements message handlers for blimp-specific operations
 *          - Provides PID tuning telemetry for position and velocity controllers
 *          - Supports high-latency telemetry modes
 *          
 *          The blimp is treated as a multicopter-like vehicle (MAV_VTOL_STATE_MC)
 *          with adaptations for lighter-than-air dynamics including buoyancy
 *          compensation and fin control.
 * 
 * @note Inherits scheduler and streaming infrastructure from GCS_MAVLINK base
 * @see GCS_MAVLINK for base class documentation
 * @see Blimp class for vehicle-specific control implementation
 */
class GCS_MAVLINK_Blimp : public GCS_MAVLINK
{

public:

    using GCS_MAVLINK::GCS_MAVLINK;

protected:

    /**
     * @brief Handle flight termination command
     * 
     * @details Processes MAV_CMD_DO_FLIGHTTERMINATION command to immediately
     *          terminate flight operations. This is a safety-critical command
     *          that disarms the vehicle and stops all motor control.
     * 
     * @param[in] packet MAVLink command_int packet containing termination request
     * 
     * @return MAV_RESULT indicating success, failure, or unsupported
     * 
     * @warning This command immediately disarms the vehicle and may result in
     *          uncontrolled descent. Should only be used in emergency situations.
     * 
     * @see Blimp::set_mode() for mode change handling
     */
    MAV_RESULT handle_flight_termination(const mavlink_command_int_t &packet) override;

    /**
     * @brief Check if parameter system is ready
     * 
     * @details Determines if the AP_Param system has completed initialization
     *          and is ready to service parameter requests from ground station.
     * 
     * @return true if parameters are loaded and ready, false otherwise
     * 
     * @note Called before processing parameter protocol requests
     */
    bool params_ready() const override;
    
    /**
     * @brief Send startup banner message to ground station
     * 
     * @details Transmits vehicle identification, firmware version, and
     *          hardware information as STATUSTEXT messages when GCS first connects.
     * 
     * @note Called once during initial GCS connection handshake
     */
    void send_banner() override;

    /**
     * @brief Handle preflight sensor calibration commands
     * 
     * @details Processes MAV_CMD_PREFLIGHT_CALIBRATION to initiate calibration
     *          of various sensors including accelerometers, gyroscopes, magnetometers,
     *          barometers, and airspeed sensors (though airspeed not used on blimp).
     *          
     *          Calibration parameters in packet.param1-param7 specify which
     *          sensors to calibrate.
     * 
     * @param[in] packet MAVLink command_int packet with calibration parameters
     * @param[in] msg    Complete MAVLink message for context
     * 
     * @return MAV_RESULT_ACCEPTED if calibration started successfully,
     *         MAV_RESULT_FAILED if preconditions not met,
     *         MAV_RESULT_UNSUPPORTED if requested calibration not available
     * 
     * @warning Vehicle must be stationary and disarmed for accurate calibration
     * @note Calibration progress reported via STATUSTEXT messages
     */
    MAV_RESULT _handle_command_preflight_calibration(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;

    /**
     * @brief Send position target telemetry to ground station
     * 
     * @details Transmits POSITION_TARGET_GLOBAL_INT message containing the
     *          current navigation target in global coordinates (latitude,
     *          longitude, altitude) along with velocity and acceleration setpoints.
     * 
     * @note Only sent when vehicle has an active position target (guided mode, auto)
     * @see send_nav_controller_output() for navigation controller state
     */
    void send_position_target_global_int() override;

    /**
     * @brief Handle region of interest (ROI) command
     * 
     * @details Processes MAV_CMD_DO_SET_ROI to point vehicle or camera at
     *          a specified location. For blimps, this typically controls
     *          yaw orientation to face the ROI point.
     * 
     * @param[in] roi_loc Location object containing target coordinates (lat, lon, alt)
     * 
     * @return MAV_RESULT_ACCEPTED if ROI set successfully,
     *         MAV_RESULT_UNSUPPORTED if not supported in current mode,
     *         MAV_RESULT_FAILED if location invalid
     * 
     * @note ROI may be cleared when mode changes
     * @see Blimp::set_yaw_mode() for yaw control integration
     */
    MAV_RESULT handle_command_do_set_roi(const Location &roi_loc) override;
    
    /**
     * @brief Main dispatcher for COMMAND_INT messages
     * 
     * @details Routes incoming COMMAND_INT MAVLink messages to appropriate
     *          command handlers based on command ID. COMMAND_INT uses integer
     *          coordinates for improved precision vs COMMAND_LONG.
     * 
     * @param[in] packet Parsed command_int packet with command and parameters
     * @param[in] msg    Complete MAVLink message for context
     * 
     * @return MAV_RESULT indicating command execution status
     * 
     * @note Called by message handler when COMMAND_INT received
     */
    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;
    
    /**
     * @brief Handle vehicle reposition command
     * 
     * @details Processes MAV_CMD_DO_REPOSITION to command vehicle to move
     *          to a new position (latitude, longitude, altitude). Creates
     *          a guided mode target at the specified location.
     * 
     * @param[in] packet Command_int packet with target position in x,y,z fields
     * 
     * @return MAV_RESULT_ACCEPTED if reposition command accepted,
     *         MAV_RESULT_DENIED if not in appropriate mode,
     *         MAV_RESULT_FAILED if position invalid
     * 
     * @note Vehicle must be in guided mode or command will be rejected
     * @see Blimp::set_target_location() for position target handling
     */
    MAV_RESULT handle_command_int_do_reposition(const mavlink_command_int_t &packet);

#if AP_MAVLINK_COMMAND_LONG_ENABLED
    /**
     * @brief Determine coordinate frame for COMMAND_LONG commands
     * 
     * @details Maps command IDs to appropriate MAV_FRAME coordinate reference
     *          for interpreting position parameters in COMMAND_LONG messages.
     *          Returns the frame that should be used for the given command.
     * 
     * @param[out] frame         Reference to store determined MAV_FRAME
     * @param[in]  packet_command MAV_CMD command identifier
     * 
     * @return true if frame determined successfully, false if command unknown
     * 
     * @note COMMAND_LONG is less precise than COMMAND_INT for position commands
     */
    bool mav_frame_for_command_long(MAV_FRAME &frame, MAV_CMD packet_command) const override;
#endif

    /**
     * @brief Send navigation controller output telemetry
     * 
     * @details Transmits NAV_CONTROLLER_OUTPUT message with current navigation
     *          state including cross-track error, bearing to target, distance
     *          to waypoint, altitude error, and airspeed error.
     * 
     * @note Required override even though some fields not applicable to blimp
     * @todo Apparently can't remove this or the build fails.
     */
    void send_nav_controller_output() const override; //TODO Apparently can't remove this or the build fails.
    
    /**
     * @brief Report vehicle capability flags
     * 
     * @details Returns MAV_PROTOCOL_CAPABILITY flags indicating which
     *          MAVLink features this vehicle supports (mission, parameter,
     *          command protocols, terrain, fence, etc.).
     * 
     * @return Bitmask of MAV_PROTOCOL_CAPABILITY flags
     * 
     * @note Sent in AUTOPILOT_VERSION message during connection handshake
     */
    uint64_t capabilities() const override;

    /**
     * @brief Report VTOL state
     * 
     * @details Returns the current VTOL transition state. Blimp always
     *          reports MAV_VTOL_STATE_MC (multicopter mode) since it
     *          operates like a multicopter with omnidirectional control.
     * 
     * @return MAV_VTOL_STATE_MC indicating multicopter-like operation
     * 
     * @note Blimp uses multicopter control algorithms with buoyancy compensation
     */
    virtual MAV_VTOL_STATE vtol_state() const override
    {
        return MAV_VTOL_STATE_MC;
    };
    
    /**
     * @brief Report landing state
     * 
     * @details Returns MAV_LANDED_STATE indicating if vehicle is on ground,
     *          in air, taking off, or landing. For blimps this is based on
     *          altitude and velocity thresholds.
     * 
     * @return MAV_LANDED_STATE_ON_GROUND, MAV_LANDED_STATE_IN_AIR, or
     *         MAV_LANDED_STATE_TAKEOFF/LANDING during transitions
     * 
     * @note Used by ground stations to determine if vehicle is safe to approach
     */
    virtual MAV_LANDED_STATE landed_state() const override;

#if HAL_LOGGING_ENABLED
    /**
     * @brief Get logging mask bit for radio messages
     * 
     * @details Returns the bitmask identifying which log type to use for
     *          radio-related logging (RC input, telemetry radio status).
     * 
     * @return MASK_LOG_PM bitmask for performance monitoring logs
     * 
     * @note Used to control logging of radio communication metrics
     */
    uint32_t log_radio_bit() const override { return MASK_LOG_PM; }
#endif

    /**
     * @brief Send available flight mode enumeration
     * 
     * @details Transmits AVAILABLE_MODES message element for the mode at
     *          the specified index. Used to enumerate all flight modes
     *          supported by the blimp to the ground control station.
     *          
     *          Index is 1-based to match MAVLink convention.
     * 
     * @param[in] index Mode index (1-based, not mode number)
     * 
     * @return Total number of available modes (same for all valid indices)
     * 
     * @note Index starts at 1, not 0, for consistency with MAVLink protocol
     * @see Blimp flight mode definitions in defines.h
     */
    // Send the mode with the given index (not mode number!) return the total number of modes
    // Index starts at 1
    uint8_t send_available_mode(uint8_t index) const override;

private:

    /**
     * @brief Main message handler dispatcher
     * 
     * @details Routes incoming MAVLink messages to appropriate handlers based
     *          on message ID. Handles blimp-specific messages and delegates
     *          common messages to base class.
     * 
     * @param[in] msg Complete MAVLink message to process
     * 
     * @note Called by packet receive handler for each valid message
     * @see try_send_message() for outgoing message handling
     */
    void handle_message(const mavlink_message_t &msg) override;
    
    /**
     * @brief Attempt to send queued message
     * 
     * @details Tries to send the specified message type if bandwidth and
     *          timing constraints allow. Messages are prioritized and rate-limited
     *          based on telemetry stream configuration.
     * 
     * @param[in] id ap_message identifier for message type to send
     * 
     * @return true if message sent successfully, false if deferred
     * 
     * @note Called by scheduler to stream telemetry at configured rates
     */
    bool try_send_message(enum ap_message id) override;

    /**
     * @brief Callback for received MAVLink packet
     * 
     * @details Called when a complete MAVLink packet is received and validated.
     *          Updates link statistics, handles routing, and dispatches to
     *          message handler.
     * 
     * @param[in] status MAVLink channel status including packet sequence info
     * @param[in] msg    Complete received and validated MAVLink message
     * 
     * @note Tracks packet loss and updates link quality metrics
     */
    void packetReceived(const mavlink_status_t &status,
                        const mavlink_message_t &msg) override;

    /**
     * @brief Get base mode flags
     * 
     * @details Returns MAV_MODE_FLAG bitmask indicating current vehicle
     *          mode state (armed, stabilized, guided, auto, manual, etc.).
     * 
     * @return MAV_MODE_FLAG bitmask representing current mode
     * 
     * @note Sent in HEARTBEAT message every second
     * @see Blimp::get_control_mode_flags() for flag generation
     */
    uint8_t base_mode() const override;
    
    /**
     * @brief Get vehicle system status
     * 
     * @details Returns MAV_STATE indicating overall vehicle health and
     *          operational state (uninit, boot, standby, active, critical, emergency).
     * 
     * @return MAV_STATE representing current system status
     * 
     * @note Sent in HEARTBEAT and SYS_STATUS messages
     * @see Blimp::check_ekf_status() for health checks
     */
    MAV_STATE vehicle_system_status() const override;

    /**
     * @brief Get airspeed for VFR_HUD message
     * 
     * @details Returns airspeed value for VFR HUD display. For blimps this
     *          is typically ground speed since airspeed sensors not used.
     * 
     * @return Airspeed in m/s (actually ground speed for blimp)
     * 
     * @note VFR_HUD provides pilot-friendly heads-up display data
     */
    float vfr_hud_airspeed() const override;
    
    /**
     * @brief Get throttle percentage for VFR_HUD message
     * 
     * @details Returns throttle value for VFR HUD display as percentage.
     *          For blimps this represents aggregate motor thrust output.
     * 
     * @return Throttle percentage (0-100)
     * 
     * @note VFR_HUD throttle is 0-100 scale for ground station display
     */
    int16_t vfr_hud_throttle() const override;
    
    /**
     * @brief Get altitude for VFR_HUD message
     * 
     * @details Returns altitude for VFR HUD display. For blimps this is
     *          relative to home position or absolute MSL depending on setting.
     * 
     * @return Altitude in meters
     * 
     * @note VFR_HUD altitude matches altimeter display in cockpit
     */
    float vfr_hud_alt() const override;

    /**
     * @brief Send PID tuning telemetry
     * 
     * @details Transmits PID_TUNING messages for active controller axes
     *          to support in-flight PID tuning via ground station. Cycles
     *          through position and velocity controllers based on tuning mask.
     * 
     * @note Tuning messages sent at reduced rate to conserve bandwidth
     * @see PID_SEND enum for axis identifiers
     * @see AC_PID classes for controller implementation
     */
    void send_pid_tuning() override;

    /**
     * @brief Send wind estimation telemetry
     * 
     * @details Transmits WIND message with estimated wind speed and direction
     *          based on difference between ground velocity and expected velocity
     *          from control commands.
     * 
     * @note Wind estimation helps compensate for environmental disturbances
     */
    void send_wind() const;

    /**
     * @enum PID_SEND
     * @brief PID axis identifiers for telemetry streaming
     * 
     * @details Defines axis identifiers for PID_TUNING message transmission.
     *          Each value corresponds to a specific controller axis (position
     *          or velocity, X/Y/Z/Yaw).
     *          
     *          This enum is 1-indexed (starts at 1, not 0) for consistency
     *          with MAVLink PID_TUNING message axis field convention.
     * 
     * @note 1-indexed unlike most enums for MAVLink protocol compatibility
     * @see send_pid_tuning() for transmission logic
     * @see MAVLink PID_TUNING message definition
     */
    //This is 1-indexed, unlike most enums for consistency with the mavlink PID_TUNING enums.
    enum PID_SEND : uint8_t {
        VELX =        1,  ///< X-axis velocity controller
        VELY =        2,  ///< Y-axis velocity controller
        VELZ =        3,  ///< Z-axis (vertical) velocity controller
        VELYAW =      4,  ///< Yaw rate controller
        POSX =        5,  ///< X-axis position controller
        POSY =        6,  ///< Y-axis position controller
        POSZ =        7,  ///< Z-axis (vertical) position controller
        POSYAW =      8,  ///< Yaw position controller
    };

#if HAL_HIGH_LATENCY2_ENABLED
    /**
     * @brief Get wind speed for high latency telemetry
     * 
     * @details Returns wind speed estimate scaled for HIGH_LATENCY2 message.
     *          High latency mode uses compressed data formats for low-bandwidth
     *          satellite or radio links.
     * 
     * @return Wind speed in scaled units (m/s * 5)
     * 
     * @note HIGH_LATENCY2 optimizes bandwidth for Iridium and similar links
     */
    uint8_t high_latency_wind_speed() const override;
    
    /**
     * @brief Get wind direction for high latency telemetry
     * 
     * @details Returns wind direction estimate scaled for HIGH_LATENCY2 message.
     *          Direction is relative to north.
     * 
     * @return Wind direction in scaled units (degrees / 2)
     * 
     * @note HIGH_LATENCY2 optimizes bandwidth for low-rate telemetry links
     */
    uint8_t high_latency_wind_direction() const override;
#endif // HAL_HIGH_LATENCY2_ENABLED
};
