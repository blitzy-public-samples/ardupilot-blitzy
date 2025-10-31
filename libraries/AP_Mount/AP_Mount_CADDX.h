/**
 * @file AP_Mount_CADDX.h
 * @brief CADDX gimbal mount backend using custom serial protocol
 * 
 * @details This file implements the CADDX gimbal serial protocol backend for ArduPilot.
 *          The CADDX protocol uses a custom framing structure with 0xA5/0x5A header bytes,
 *          10-byte packets containing 12-bit axis angle values (0.1 degree resolution),
 *          and CRC16-CCITT checksums for data integrity.
 * 
 * Protocol characteristics:
 * - Framing: 0xA5 0x5A header bytes
 * - Packet length: 10 bytes total
 * - Angle encoding: 12-bit values with 0.1 degree resolution per axis
 * - Error detection: CRC16-CCITT checksum
 * - Update rate: Commands resent periodically every 200ms
 * - Attitude feedback: Gimbal does not report actual angles; attitude is synthesized from targets
 * 
 * @note This backend sends angle targets to the gimbal but does not receive telemetry
 *       with actual gimbal orientation. The get_attitude_quaternion() method synthesizes
 *       attitude from the commanded target angles rather than measured values.
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */
#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_CADDX_ENABLED

#include "AP_Mount_Backend_Serial.h"
#include <AP_Math/quaternion.h>

/**
 * @class AP_Mount_CADDX
 * @brief CADDX gimbal mount backend implementing custom serial protocol
 * 
 * @details This class provides control of CADDX gimbals via their proprietary serial protocol.
 *          The protocol uses a custom packet structure:
 *          - Header: 0xA5 0x5A (2 bytes)
 *          - Payload: Angle data encoded as 12-bit values (6 bytes for roll/pitch/yaw)
 *          - Checksum: CRC16-CCITT for error detection (2 bytes)
 *          - Total packet size: 10 bytes
 * 
 *          Angle values are transmitted with 0.1 degree resolution, allowing precise
 *          gimbal positioning. The backend periodically resends target angles every 200ms
 *          to maintain control authority.
 * 
 *          Key protocol characteristics:
 *          - One-way communication: ArduPilot sends commands, gimbal does not respond
 *          - No attitude telemetry: Gimbal does not report actual angles back
 *          - Attitude synthesis: get_attitude_quaternion() returns commanded angles, not measured
 *          - Axis locking: Supports individual axis lock modes via LockMode flags
 * 
 * @note The CADDX gimbal does not provide attitude feedback. The attitude returned by
 *       get_attitude_quaternion() is synthesized from the last commanded target angles
 *       rather than measured gimbal orientation.
 * 
 * @warning CRC16-CCITT calculation must be correct or gimbal will reject commands.
 * 
 * @see AP_Mount_Backend_Serial
 */
class AP_Mount_CADDX : public AP_Mount_Backend_Serial
{

public:
    // Constructor
    using AP_Mount_Backend_Serial::AP_Mount_Backend_Serial;

    /**
     * @brief Update mount position - should be called periodically
     * 
     * @details This method is called at regular intervals to update the gimbal position.
     *          It retrieves the current angle target from the mount library, then sends
     *          the target angles to the CADDX gimbal via the serial protocol. Commands
     *          are resent every 200ms to maintain gimbal control.
     * 
     * @note Called by the mount library's main update loop, typically at 10-50Hz
     * @note Implements periodic resend (200ms interval) to ensure gimbal maintains position
     */
    void update() override;

    /**
     * @brief Check if gimbal can control pan (yaw) axis
     * 
     * @details Returns true if this mount can control its pan/yaw axis.
     *          Pan control is required for multicopters to maintain camera pointing
     *          as the vehicle yaws. Capability is determined by checking if valid
     *          yaw angle limits are configured.
     * 
     * @return true if yaw axis control is available and configured
     * @return false if yaw axis is not controllable or not configured
     */
    bool has_pan_control() const override { return yaw_range_valid(); };

    /**
     * @brief Check if gimbal can control roll axis
     * 
     * @details Returns true if this mount can control its roll axis.
     *          Roll control capability is determined by checking if valid
     *          roll angle limits are configured.
     * 
     * @return true if roll axis control is available and configured
     * @return false if roll axis is not controllable or not configured
     */
    bool has_roll_control() const override { return roll_range_valid(); };

    /**
     * @brief Check if gimbal can control pitch (tilt) axis
     * 
     * @details Returns true if this mount can control its pitch/tilt axis.
     *          Pitch control is the primary gimbal function for camera stabilization.
     *          Capability is determined by checking if valid pitch angle limits are configured.
     * 
     * @return true if pitch axis control is available and configured
     * @return false if pitch axis is not controllable or not configured
     */
    bool has_pitch_control() const override { return pitch_range_valid(); };

protected:

    /**
     * @brief Get gimbal attitude as a quaternion
     * 
     * @details Returns the gimbal attitude as a quaternion. Since the CADDX gimbal
     *          does not report its actual orientation via telemetry, this method
     *          synthesizes the attitude quaternion from the last commanded target angles
     *          rather than from measured gimbal positions.
     * 
     *          The synthesized attitude assumes the gimbal has achieved the commanded
     *          angles. This may not reflect the true gimbal orientation if:
     *          - The gimbal is still moving to the target
     *          - Physical limits prevent reaching the target
     *          - External disturbances affect gimbal position
     *          - Gimbal power is lost
     * 
     * @param[out] att_quat Quaternion representing synthesized gimbal attitude (NED frame)
     * 
     * @return true if attitude quaternion was successfully synthesized from target angles
     * @return false if no valid target angles are available
     * 
     * @note This is a synthesized attitude, not measured from the gimbal hardware
     * @warning Actual gimbal position may differ from returned attitude if gimbal
     *          cannot achieve commanded angles or loses power
     */
    bool get_attitude_quaternion(Quaternion& att_quat) override;

private:

    /**
     * @brief Axis locking mode flags for CADDX gimbal
     * 
     * @details Defines bit flags for locking individual gimbal axes. When an axis is locked,
     *          the gimbal maintains that axis at a fixed angle regardless of vehicle movement.
     *          Multiple axes can be locked simultaneously by combining flags.
     * 
     * @note Flags can be combined with bitwise OR to lock multiple axes
     */
    enum class LockMode {
        TILT_LOCK = (1<<0),  ///< Lock tilt/pitch axis (bit 0)
        ROLL_LOCK = (1<<1),  ///< Lock roll axis (bit 1)
        YAW_LOCK  = (1<<2),  ///< Lock yaw/pan axis (bit 2)
    };

    /**
     * @brief Send target angles to CADDX gimbal via serial protocol
     * 
     * @details Constructs and transmits a CADDX protocol packet containing target angles
     *          for roll, pitch, and yaw axes. The packet structure is:
     *          - Header: 0xA5 0x5A (2 bytes)
     *          - Angle data: 12-bit values for each axis with 0.1 degree resolution (6 bytes)
     *          - CRC16-CCITT checksum (2 bytes)
     *          Total packet size: 10 bytes
     * 
     *          Angles are converted from radians to 12-bit integer values with 0.1 degree
     *          resolution before transmission. The CRC16-CCITT checksum is calculated over
     *          the entire packet (excluding the checksum bytes themselves) to ensure data integrity.
     * 
     * @param[in] angle_target_rad Target angles for roll, pitch, and yaw axes in radians (NED frame)
     * 
     * @note Called by update() method when new target angles need to be sent
     * @note Angles are internally converted from radians to centidegrees for 12-bit encoding
     * @warning Incorrect CRC calculation will cause gimbal to reject the command
     */
    void send_target_angles(const MountTarget& angle_target_rad);

    // internal variables
    /**
     * @brief System time (milliseconds) when last command was sent to gimbal
     * 
     * @details Tracks the timestamp of the last send_target_angles() call to implement
     *          periodic command resending. Commands are resent every 200ms to maintain
     *          gimbal control authority and ensure the gimbal continues tracking the
     *          target angles even if individual packets are lost.
     * 
     * @note Used to enforce 200ms resend interval in update() method
     */
    uint32_t _last_send_ms;
};
#endif // HAL_MOUNT_CADDX_ENABLED
