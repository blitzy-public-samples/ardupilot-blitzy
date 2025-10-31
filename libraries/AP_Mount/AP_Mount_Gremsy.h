/**
 * @file AP_Mount_Gremsy.h
 * @brief Gremsy MAVLink v2 Gimbal Protocol backend for ArduPilot mount control
 * 
 * This file implements the backend driver for Gremsy gimbals using the MAVLink v2
 * gimbal protocol. The driver communicates with the gimbal using GIMBAL_DEVICE_*
 * messages and provides attitude feedback and control via MAVLink.
 * 
 * @note Supports MAVLink gimbal manager protocol v2 with automatic gimbal discovery
 */
#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_GREMSY_ENABLED

#include "AP_Mount_Backend.h"

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

/**
 * @class AP_Mount_Gremsy
 * @brief Backend driver for Gremsy gimbals using MAVLink v2 gimbal protocol
 * 
 * @details This class implements support for Gremsy gimbals using the MAVLink v2
 *          gimbal device protocol. Communication is established through MAVLink
 *          GIMBAL_DEVICE_* messages (GIMBAL_DEVICE_INFORMATION, 
 *          GIMBAL_DEVICE_SET_ATTITUDE, GIMBAL_DEVICE_ATTITUDE_STATUS).
 *          
 *          The driver automatically discovers gimbals on the MAVLink network by
 *          searching the GCS_MAVLink routing table. Once discovered, it streams
 *          AUTOPILOT_STATE_FOR_GIMBAL_DEVICE messages to provide vehicle attitude
 *          information to the gimbal for stabilization.
 *          
 *          Key features:
 *          - Automatic gimbal discovery via MAVLink routing table
 *          - Attitude control in both earth-frame and body-frame
 *          - Rate control for smooth gimbal movements
 *          - Health monitoring via attitude status messages
 *          - Support for gimbal retract (relax) mode
 *          
 * @note Discovery timing: Device info requested every RESEND_MS (1000ms) for up to
 *       SEARCH_MS (60000ms). Attitude updates sent at ATTITUDE_INTERVAL_US (20000us).
 * @note Coordinate frames: Gimbal attitudes reported in NED earth-frame with body-frame yaw
 */
class AP_Mount_Gremsy : public AP_Mount_Backend
{

public:
    /// Constructor - inherits from AP_Mount_Backend
    using AP_Mount_Backend::AP_Mount_Backend;

    /**
     * @brief Update mount position and maintain communication with gimbal
     * 
     * @details Called periodically by the mount driver to update gimbal state.
     *          Handles gimbal discovery, device information requests, attitude
     *          streaming setup, and sending control commands to the gimbal.
     *          Also processes target angles/rates and sends appropriate
     *          GIMBAL_DEVICE_SET_ATTITUDE messages.
     * 
     * @note Called from main thread at mount update rate (typically 10-50Hz)
     */
    void update() override;

    /**
     * @brief Check if gimbal connection is healthy
     * 
     * @details Returns true if gimbal is initialized (device info received) and
     *          recent attitude status messages have been received within timeout period.
     * 
     * @return true if gimbal is healthy and communicating, false otherwise
     */
    bool healthy() const override;

    /**
     * @brief Check if gimbal has pan (yaw) control capability
     * 
     * @details Determines if the gimbal supports yaw control based on the valid
     *          yaw range reported in GIMBAL_DEVICE_INFORMATION message.
     * 
     * @return true if gimbal has valid yaw range, false otherwise
     */
    bool has_pan_control() const override { return yaw_range_valid(); }

    /**
     * @brief Handle incoming GIMBAL_DEVICE_INFORMATION message from gimbal
     * 
     * @details Processes gimbal device information including vendor name, model name,
     *          firmware version, and gimbal angle limits (roll, pitch, yaw ranges).
     *          This message is used to complete gimbal initialization and determine
     *          gimbal capabilities.
     * 
     * @param[in] msg MAVLink message containing GIMBAL_DEVICE_INFORMATION payload
     * 
     * @note This message is requested periodically until received during discovery phase
     */
    void handle_gimbal_device_information(const mavlink_message_t &msg) override;

    /**
     * @brief Handle incoming GIMBAL_DEVICE_ATTITUDE_STATUS message from gimbal
     * 
     * @details Processes gimbal attitude status including current quaternion orientation,
     *          angular velocity, and failure flags. This message provides real-time
     *          feedback of gimbal attitude and is used for health monitoring and
     *          closed-loop control.
     * 
     * @param[in] msg MAVLink message containing GIMBAL_DEVICE_ATTITUDE_STATUS payload
     * 
     * @note Gimbal should send this message at regular intervals (typically 10-50Hz)
     *       for proper health monitoring
     */
    void handle_gimbal_device_attitude_status(const mavlink_message_t &msg) override;

protected:

    /**
     * @brief Get current gimbal attitude as a quaternion
     * 
     * @details Retrieves the most recently received gimbal attitude from the cached
     *          GIMBAL_DEVICE_ATTITUDE_STATUS message. The attitude quaternion represents
     *          the gimbal's orientation in NED earth-frame with body-frame yaw component.
     * 
     * @param[out] att_quat Quaternion to be filled with gimbal attitude
     * 
     * @return true if attitude is available and valid, false if no recent attitude data
     * 
     * @note Coordinate frame: NED earth-frame for roll/pitch, body-frame for yaw
     * @note Returns false if gimbal is not initialized or attitude data is stale
     */
    bool get_attitude_quaternion(Quaternion& att_quat) override;

private:

    /**
     * @brief Search for gimbal in GCS_MAVLink routing table
     * 
     * @details Scans the MAVLink routing table to locate a gimbal component (MAV_COMP_ID_GIMBAL)
     *          connected to any MAVLink channel. Once found, stores the MAVLink link, system ID,
     *          and component ID for subsequent communication. Called repeatedly during discovery
     *          phase until gimbal is found.
     * 
     * @note Discovery searches for MAV_COMP_ID_GIMBAL component type
     * @note Sets _link, _sysid, and _compid when gimbal is discovered
     */
    void find_gimbal();

    /**
     * @brief Request GIMBAL_DEVICE_INFORMATION from gimbal
     * 
     * @details Sends MAVLink message to request gimbal device information including
     *          vendor name, model name, firmware version, and mechanical limits
     *          (roll/pitch/yaw min/max angles). This information is needed to complete
     *          gimbal initialization and configure control limits.
     * 
     * @note Called periodically (every RESEND_MS = 1000ms) until device info received
     * @note Search times out after SEARCH_MS (60000ms) if no response received
     */
    void request_gimbal_device_information() const;

    /**
     * @brief Start sending ATTITUDE and AUTOPILOT_STATE_FOR_GIMBAL_DEVICE to gimbal
     * 
     * @details Configures the MAVLink system to start streaming vehicle attitude information
     *          to the gimbal. The gimbal uses this data for stabilization and to interpret
     *          body-frame vs earth-frame commands. Attitude is sent at ATTITUDE_INTERVAL_US
     *          (20000us = 50Hz) intervals.
     * 
     * @return true if attitude streaming successfully started, false on failure
     * 
     * @note AUTOPILOT_STATE_FOR_GIMBAL_DEVICE includes vehicle attitude quaternion and rates
     * @note Streaming interval: 20000us (50Hz)
     */
    bool start_sending_attitude_to_gimbal();

    /**
     * @brief Send GIMBAL_DEVICE_SET_ATTITUDE to command gimbal to retract (relax)
     * 
     * @details Sends a special attitude command with retract flag set, instructing the
     *          gimbal to enter relaxed/neutral position. This is typically used when
     *          disarming or to allow manual gimbal positioning.
     * 
     * @note Uses GIMBAL_DEVICE_FLAGS_RETRACT flag in GIMBAL_DEVICE_SET_ATTITUDE message
     */
    void send_gimbal_device_retract() const;

    /**
     * @brief Send GIMBAL_DEVICE_SET_ATTITUDE to gimbal to control angular rates
     * 
     * @details Commands the gimbal to move at specified angular rates. Rates can be
     *          specified in either earth-frame or body-frame depending on earth_frame flag.
     * 
     * @param[in] roll_rads  Roll rate target in radians/second
     * @param[in] pitch_rads Pitch rate target in radians/second
     * @param[in] yaw_rads   Yaw rate target in radians/second
     * @param[in] earth_frame true for earth-frame yaw rate, false for body-frame yaw rate
     * 
     * @note Roll and pitch rates are always in gimbal/body frame
     * @note Yaw rate frame is selectable via earth_frame parameter
     * @note Units: radians per second for all axes
     */
    void send_gimbal_device_set_rate(float roll_rads, float pitch_rads, float yaw_rads, bool earth_frame) const;

    /**
     * @brief Send GIMBAL_DEVICE_SET_ATTITUDE to gimbal to control attitude angles
     * 
     * @details Commands the gimbal to specific attitude angles. Angles can be specified
     *          in either earth-frame or body-frame depending on earth_frame flag.
     * 
     * @param[in] roll_rad   Roll angle target in radians
     * @param[in] pitch_rad  Pitch angle target in radians
     * @param[in] yaw_rad    Yaw angle target in radians
     * @param[in] earth_frame true for earth-frame yaw angle, false for body-frame yaw angle
     * 
     * @note Roll and pitch angles are always relative to earth-frame (NED)
     * @note Yaw angle frame is selectable via earth_frame parameter
     * @note Units: radians for all axes
     * @note Gimbal will respect configured angle limits from GIMBAL_DEVICE_INFORMATION
     */
    void send_gimbal_device_set_attitude(float roll_rad, float pitch_rad, float yaw_rad, bool earth_frame) const;

    // Internal state variables for gimbal communication and health monitoring
    
    /// Flag indicating GIMBAL_DEVICE_INFORMATION has been received and processed
    bool _got_device_info;
    
    /// Initialization complete flag - true once GIMBAL_DEVICE_INFORMATION received with valid limits
    bool _initialised;
    
    /// Timestamp (milliseconds) of last GIMBAL_DEVICE_INFORMATION request sent to gimbal
    /// Used to throttle discovery requests to every RESEND_MS (1000ms) during search phase
    uint32_t _last_devinfo_req_ms;
    
    /// Pointer to MAVLink communication link where gimbal was discovered
    /// nullptr until gimbal found in routing table, then points to active GCS_MAVLINK channel
    class GCS_MAVLINK *_link;
    
    /// MAVLink system ID of the gimbal component (from routing table discovery)
    uint8_t _sysid;
    
    /// MAVLink component ID of the gimbal (typically MAV_COMP_ID_GIMBAL)
    uint8_t _compid;
    
    /// Cached copy of most recently received GIMBAL_DEVICE_ATTITUDE_STATUS message
    /// Contains gimbal attitude quaternion, angular velocity, and failure flags
    /// Used to provide current attitude feedback to mount driver
    mavlink_gimbal_device_attitude_status_t _gimbal_device_attitude_status;
    
    /// Timestamp (milliseconds) when last GIMBAL_DEVICE_ATTITUDE_STATUS was received
    /// Used for health monitoring - gimbal considered unhealthy if no recent status messages
    uint32_t _last_attitude_status_ms;
};
#endif // HAL_MOUNT_GREMSY_ENABLED
