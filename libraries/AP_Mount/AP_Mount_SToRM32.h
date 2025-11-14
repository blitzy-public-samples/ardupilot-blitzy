/**
 * @file AP_Mount_SToRM32.h
 * @brief SToRM32 gimbal mount backend using MAVLink protocol
 * 
 * @details This file implements the MAVLink-based interface for SToRM32 gimbal controllers.
 *          The SToRM32 backend communicates with the gimbal using MAVLink COMMAND_LONG messages
 *          with MAV_CMD_DO_MOUNT_CONTROL to send angle or rate control commands. The gimbal
 *          is discovered through the MAVLink routing table by searching for a component with
 *          the appropriate system and component ID.
 * 
 *          Protocol: MAVLink v1/v2 compatible
 *          Command: COMMAND_LONG (MAV_CMD_DO_MOUNT_CONTROL)
 *          Units: Angle commands in centidegrees, rate commands in centidegrees/second
 * 
 * @note The implementation includes automatic gimbal discovery and periodic command resend
 *       for reliability in lossy communication environments.
 */
#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_STORM32MAVLINK_ENABLED

#include "AP_Mount_Backend.h"

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>

/**
 * @class AP_Mount_SToRM32
 * @brief Mount backend for SToRM32 gimbal controllers using MAVLink protocol
 * 
 * @details This class implements control of SToRM32 3-axis gimbals via MAVLink communication.
 *          SToRM32 gimbals are controlled by sending MAVLink COMMAND_LONG messages with
 *          MAV_CMD_DO_MOUNT_CONTROL command ID containing either angle targets or rate commands.
 * 
 *          **Communication Protocol:**
 *          - Uses MAVLink COMMAND_LONG with MAV_CMD_DO_MOUNT_CONTROL
 *          - Angle mode: Sends target angles in centidegrees (pitch, roll, yaw)
 *          - Rate mode: Sends rate commands in centidegrees/second
 *          - Commands are resent periodically for reliability (typically every 1000ms)
 * 
 *          **Gimbal Discovery:**
 *          - Searches MAVLink routing table for gimbal system/component ID
 *          - Initialization completes when gimbal responds to commands
 *          - Stores gimbal's sysid, compid, and communication channel
 * 
 *          **Attitude Feedback:**
 *          - Receives attitude updates from gimbal via MAVLink messages
 *          - Provides current gimbal orientation as quaternion
 *          - Used for stabilization and pointing accuracy verification
 * 
 *          **Coordinate Frames:**
 *          - Commands use vehicle body frame (NED convention)
 *          - Pitch: Positive up, Roll: Positive right, Yaw: Positive clockwise from north
 * 
 * @note The backend handles COMMAND_ACK responses from the gimbal to verify command reception.
 *       Resend intervals ensure commands reach the gimbal even with packet loss.
 * 
 * @warning MAVLink rate limiting may affect command update rates. Ensure MAVLink stream rates
 *          are configured appropriately for mount control responsiveness. Excessive command
 *          rates may saturate the MAVLink channel.
 */
class AP_Mount_SToRM32 : public AP_Mount_Backend
{

public:
    /**
     * @brief Constructor - inherits from AP_Mount_Backend
     * 
     * @details Uses parent class constructor for common mount initialization.
     *          Specific SToRM32 initialization occurs in init() and find_gimbal() methods.
     */
    using AP_Mount_Backend::AP_Mount_Backend;

    /**
     * @brief Update mount position and send commands to gimbal - called periodically by mount library
     * 
     * @details This method is called by the mount library's main update loop (typically at 10-50Hz).
     *          It performs the following operations:
     *          1. Searches for the gimbal in the MAVLink routing table if not yet found
     *          2. Calculates target angles or rates based on mount mode and pilot input
     *          3. Sends MAVLink COMMAND_LONG with MAV_CMD_DO_MOUNT_CONTROL to gimbal
     *          4. Implements resend logic to ensure commands reach gimbal despite packet loss
     * 
     *          Commands are sent periodically (approximately every 1000ms) even if targets haven't
     *          changed, ensuring the gimbal maintains current position despite potential message loss.
     * 
     * @note This override implements the periodic update required by AP_Mount_Backend interface.
     *       The update rate is determined by the mount library scheduler, not this class.
     * 
     * @warning Ensure this is called at consistent intervals for smooth gimbal motion. Irregular
     *          calling patterns may cause jerky gimbal movement.
     */
    void update() override;

    /**
     * @brief Check if this mount can control yaw axis (pan control)
     * 
     * @details Returns true if the gimbal has valid yaw range configuration, indicating
     *          it can perform pan (yaw) control. This is required for multicopters to
     *          enable coordinated yaw control between vehicle and gimbal.
     * 
     *          The yaw range validity is checked through yaw_range_valid() inherited from
     *          AP_Mount_Backend, which verifies the configured yaw min/max parameters.
     * 
     * @return true if gimbal has valid yaw control range configured
     * @return false if yaw control is disabled or invalid range
     * 
     * @note For multicopters, pan control capability determines whether the vehicle should
     *       coordinate yaw movements with gimbal pointing direction.
     */
    bool has_pan_control() const override { return yaw_range_valid(); }

protected:

    /**
     * @brief Get current gimbal attitude as a quaternion
     * 
     * @details Retrieves the current gimbal orientation from cached attitude data received
     *          via MAVLink messages from the SToRM32 gimbal. The gimbal periodically sends
     *          its current attitude, which is stored and made available through this method.
     * 
     *          The attitude represents the gimbal's orientation in the vehicle body frame,
     *          allowing the mount library to determine the current pointing direction for
     *          stabilization and target tracking calculations.
     * 
     *          **Coordinate Frame:**
     *          - Returns attitude in vehicle body frame (NED convention)
     *          - Quaternion represents rotation from body frame to gimbal frame
     * 
     * @param[out] att_quat Quaternion to be filled with current gimbal attitude
     * 
     * @return true if valid attitude data is available and att_quat was updated
     * @return false if no valid attitude data (gimbal not initialized or no recent updates)
     * 
     * @note This method may return false during gimbal initialization or if MAVLink
     *       communication with the gimbal has been lost for an extended period.
     * 
     * @warning Attitude data staleness is not explicitly checked. If MAVLink communication
     *          is lost, this may continue returning the last known attitude until reconnection.
     */
    bool get_attitude_quaternion(Quaternion& att_quat) override;

private:

    /**
     * @brief Search for SToRM32 gimbal in MAVLink routing table
     * 
     * @details Scans the GCS_MAVLink routing table to locate the SToRM32 gimbal by searching
     *          for a MAVLink component matching the configured system and component IDs.
     *          Once found, stores the gimbal's system ID, component ID, and MAVLink channel
     *          for subsequent communication.
     * 
     *          **Discovery Process:**
     *          1. Iterates through MAVLink routing table entries
     *          2. Matches against configured gimbal system/component ID parameters
     *          3. Stores communication channel for efficient message routing
     *          4. Sets initialization flag when gimbal is successfully located
     * 
     *          This method is called repeatedly from update() until the gimbal is found,
     *          supporting hot-plug scenarios where the gimbal may not be available at boot.
     * 
     * @note Called periodically from update() until _initialised flag is set.
     *       The search is lightweight and safe to call repeatedly.
     * 
     * @warning Requires valid gimbal sysid/compid configuration parameters. If parameters
     *          are not set or incorrect, the gimbal will never be found.
     */
    void find_gimbal();

    /**
     * @brief Send MAVLink COMMAND_LONG with MAV_CMD_DO_MOUNT_CONTROL to gimbal
     * 
     * @details Transmits a mount control command to the SToRM32 gimbal via MAVLink.
     *          Converts angle targets from radians to centidegrees as required by the
     *          MAVLink protocol and sends via COMMAND_LONG message.
     * 
     *          **Message Format:**
     *          - Command ID: MAV_CMD_DO_MOUNT_CONTROL
     *          - param1: Pitch angle in centidegrees
     *          - param2: Roll angle in centidegrees  
     *          - param3: Yaw angle in centidegrees
     *          - param7: MAV_MOUNT_MODE (typically MAV_MOUNT_MODE_MAVLINK_TARGETING)
     * 
     *          **Reliability:**
     *          - Commands should be resent periodically for reliability
     *          - Gimbal may send COMMAND_ACK to acknowledge receipt
     *          - Update _last_send timestamp to track resend intervals
     * 
     * @param[in] angle_target_rad Mount angle targets in radians (pitch, roll, yaw)
     *                             Internally converted to centidegrees for MAVLink transmission
     * 
     * @note This method handles unit conversion from radians (used internally) to centidegrees
     *       (required by MAVLink protocol). Angle limits are applied before transmission.
     * 
     * @warning MAVLink channel must be valid before calling. Ensure find_gimbal() has
     *          completed successfully. Excessive call rates may saturate MAVLink bandwidth.
     */
    void send_do_mount_control(const MountTarget& angle_target_rad);

    /**
     * @name Internal State Variables
     * @{
     */
    
    /** @brief Initialization state flag
     *  
     *  Set to true once gimbal discovery completes via find_gimbal().
     *  Prevents repeated routing table searches and enables command transmission.
     *  Remains false until gimbal is located in MAVLink routing table.
     */
    bool _initialised;
    
    /** @brief Gimbal MAVLink system ID
     * 
     *  Stores the MAVLink system ID of the discovered SToRM32 gimbal.
     *  Used for addressing COMMAND_LONG messages to the correct system.
     *  Populated by find_gimbal() during discovery process.
     */
    uint8_t _sysid;
    
    /** @brief Gimbal MAVLink component ID
     * 
     *  Stores the MAVLink component ID of the discovered SToRM32 gimbal.
     *  Used for addressing COMMAND_LONG messages to the correct component.
     *  Populated by find_gimbal() during discovery process.
     */
    uint8_t _compid;
    
    /** @brief MAVLink communication channel for gimbal
     * 
     *  Stores the MAVLink channel used to communicate with the gimbal.
     *  Determined during gimbal discovery by find_gimbal().
     *  Defaults to MAVLINK_COMM_0 if gimbal not yet found.
     *  Used for efficient message routing without routing table lookups.
     */
    mavlink_channel_t _chan = MAVLINK_COMM_0;
    
    /** @brief Timestamp of last command transmission
     * 
     *  Stores system time (milliseconds) when the last MAV_CMD_DO_MOUNT_CONTROL
     *  was sent to the gimbal. Used to implement periodic command resend for
     *  reliability in lossy communication environments.
     * 
     *  Typical resend interval: 1000ms (1 Hz)
     * 
     *  @note Timestamp is in milliseconds from AP_HAL::millis()
     */
    uint32_t _last_send;
    
    /** @} */ // End of Internal State Variables group
};
#endif // HAL_MOUNT_STORM32MAVLINK_ENABLED
