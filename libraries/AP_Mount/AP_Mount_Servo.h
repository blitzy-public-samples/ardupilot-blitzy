/**
 * @file AP_Mount_Servo.h
 * @brief Servo-based mount control backend for camera gimbals
 * 
 * @details This backend provides control for servo or PWM-based camera gimbals.
 *          Supports two operational modes:
 *          1. Servo mode with stabilization: Requires earth-frame to body-frame
 *             coordinate transformations to stabilize camera pointing against
 *             vehicle attitude changes. Uses lead compensation for improved tracking.
 *          2. BrushlessPWM mode: Self-stabilizing gimbals that handle their own
 *             stabilization internally, requiring only angle commands.
 * 
 *          The backend integrates with ArduPilot's SRV_Channel system to output
 *          PWM signals to servo motors controlling roll, tilt (pitch), and pan (yaw) axes.
 * 
 * @note Angle units: Targets are in earth-frame radians, servo outputs are in
 *       body-frame degrees*10. Update rate minimum: 10Hz.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */
#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_SERVO_ENABLED

#include "AP_Mount_Backend.h"

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <SRV_Channel/SRV_Channel.h>

/**
 * @class AP_Mount_Servo
 * @brief Backend class for controlling servo/PWM-based camera gimbals
 * 
 * @details Implements mount control using standard PWM servo outputs. This backend
 *          supports two distinct gimbal types based on their stabilization capabilities:
 * 
 *          **Servo Gimbals (requires_stabilization = true)**:
 *          - Simple servos that directly follow commanded angles
 *          - Requires earth-frame to body-frame transformation by backend
 *          - Uses vehicle AHRS attitude to compute stabilized body-frame commands
 *          - Implements lead compensation (MNT_ROLL_STB_LEAD, MNT_PITCH_STB_LEAD)
 *            to compensate for vehicle rotation during attitude updates
 *          - Suitable for basic pan-tilt mechanisms without onboard stabilization
 * 
 *          **BrushlessPWM Gimbals (requires_stabilization = false)**:
 *          - Gimbals with integrated controllers that self-stabilize
 *          - Backend passes through earth-frame angles without transformation
 *          - Gimbal controller handles all stabilization internally
 *          - Suitable for brushless gimbal controllers with IMU integration
 * 
 *          The backend manages servo channels for up to four axes:
 *          - Roll: Camera roll axis rotation
 *          - Tilt (Pitch): Camera elevation angle
 *          - Pan (Yaw): Camera azimuth angle
 *          - Open: Shutter or mechanism activation
 * 
 *          **Frame Convention**:
 *          - Earth frame: NED (North-East-Down) coordinate system
 *          - Body frame: Vehicle-relative coordinates (forward-right-down)
 *          - Servo outputs: Body-frame angles in degrees*10 format
 * 
 *          **Update Rate**: Minimum 10Hz for smooth gimbal control and lead compensation
 * 
 * @note Thread Safety: Called from main thread only, no special locking required
 * @warning Incorrect stabilization mode can cause unstable gimbal behavior
 * 
 * @see AP_Mount_Backend for base class interface
 * @see SRV_Channel for servo output management
 */
class AP_Mount_Servo : public AP_Mount_Backend
{
public:
    /**
     * @brief Constructor for servo-based mount backend
     * 
     * @details Initializes the servo mount backend with specified stabilization mode.
     *          Servo channel function indices are initialized to k_none and will be
     *          assigned during init() based on the instance number.
     * 
     * @param[in] frontend      Reference to AP_Mount frontend for accessing global mount state
     * @param[in] params        Reference to mount parameters (angle limits, rates, lead compensation)
     * @param[in] requires_stab Stabilization requirement flag:
     *                          - true: Simple servos requiring earth→body frame transformation
     *                          - false: Self-stabilizing BrushlessPWM gimbals
     * @param[in] instance      Mount instance number (0-based), determines servo channel functions:
     *                          - Instance 0: Uses SRV_Channel::k_mount_roll, k_mount_tilt, k_mount_pan
     *                          - Instance 1: Uses SRV_Channel::k_mount2_roll, k_mount2_tilt, k_mount2_pan
     * 
     * @note Servo channel assignments are completed in init() method
     */
    AP_Mount_Servo(AP_Mount &frontend, AP_Mount_Params &params, bool requires_stab, uint8_t instance):
        AP_Mount_Backend(frontend, params, instance),
        requires_stabilization(requires_stab),
        _roll_idx(SRV_Channel::k_none),
        _tilt_idx(SRV_Channel::k_none),
        _pan_idx(SRV_Channel::k_none),
        _open_idx(SRV_Channel::k_none)
    {
    }

    /**
     * @brief Initialize servo mount backend
     * 
     * @details Performs initialization for this mount instance:
     *          - Assigns SRV_Channel function indices based on instance number
     *          - Validates servo channel availability
     *          - Configures initial servo positions to neutral/default angles
     * 
     *          Instance-specific channel assignments:
     *          - Instance 0: k_mount_roll, k_mount_tilt, k_mount_pan, k_mount_open
     *          - Instance 1: k_mount2_roll, k_mount2_tilt, k_mount2_pan, k_mount2_open
     * 
     * @note Called once during mount system initialization
     * @note Missing servo channel assignments are left as k_none (axis unavailable)
     */
    void init() override;

    /**
     * @brief Update mount position based on current target
     * 
     * @details Called periodically (minimum 10Hz) to update gimbal position.
     *          Processing sequence:
     *          1. Retrieve current angle target from mount frontend
     *          2. If stabilization required: Transform earth-frame target to body-frame
     *             using vehicle AHRS attitude with lead compensation
     *          3. If no stabilization: Pass through earth-frame angles directly
     *          4. Apply angle limits from parameters
     *          5. Output PWM commands to servo channels via move_servo()
     * 
     *          For stabilization mode, implements lead compensation using parameters:
     *          - MNT_ROLL_STB_LEAD: Roll axis lead time (seconds)
     *          - MNT_PITCH_STB_LEAD: Pitch axis lead time (seconds)
     * 
     * @note Update rate affects lead compensation accuracy and gimbal smoothness
     * @note Must be called regularly to maintain gimbal pointing accuracy
     * 
     * @see update_angle_outputs() for frame transformation details
     * @see move_servo() for PWM output implementation
     */
    void update() override;

    /**
     * @brief Check if this mount can control roll axis
     * 
     * @return true if roll servo channel is assigned and available
     * @return false if roll control not configured for this instance
     * 
     * @note Roll control availability determined by SRV_Channel function assignment
     */
    bool has_roll_control() const override;

    /**
     * @brief Check if this mount can control pitch (tilt) axis
     * 
     * @return true if tilt servo channel is assigned and available
     * @return false if pitch control not configured for this instance
     * 
     * @note Pitch control availability determined by SRV_Channel function assignment
     */
    bool has_pitch_control() const override;

    /**
     * @brief Check if this mount can control pan (yaw) axis
     * 
     * @return true if pan servo channel is assigned and available
     * @return false if pan control not configured for this instance
     * 
     * @note Pan control required for multicopters to maintain camera heading
     * @note Pan control availability determined by SRV_Channel function assignment
     */
    bool has_pan_control() const override;

protected:

    /**
     * @brief Get current gimbal attitude as quaternion
     * 
     * @details Estimates gimbal attitude from current servo positions by:
     *          1. Reading actual servo output angles from _angle_bf_output_rad
     *          2. Converting body-frame servo angles to quaternion representation
     *          3. Combining with vehicle attitude to produce earth-frame gimbal attitude
     * 
     *          This method allows external systems to query the actual gimbal pointing
     *          direction for camera triggering, video overlay, or telemetry purposes.
     * 
     * @param[out] att_quat Quaternion representing gimbal attitude in earth frame (NED)
     * 
     * @return true if attitude successfully calculated from valid servo positions
     * @return false if servo positions invalid or unavailable
     * 
     * @note Attitude accuracy depends on servo position feedback accuracy
     * @note For open-loop servos, returns commanded position (not actual position)
     */
    bool get_attitude_quaternion(Quaternion& att_quat) override;

private:

    /**
     * @brief Transform earth-frame target angles to body-frame servo outputs
     * 
     * @details Converts earth-frame (NED) angle targets to vehicle body-frame commands
     *          for servo positioning. Implements the core stabilization algorithm:
     * 
     *          **Transformation Steps**:
     *          1. Retrieve vehicle attitude from AHRS (roll, pitch, yaw)
     *          2. Apply lead compensation if configured (MNT_ROLL_STB_LEAD, MNT_PITCH_STB_LEAD)
     *             - Predicts future vehicle attitude based on gyro rates
     *             - Compensates for control loop delay in gimbal response
     *          3. Compute earth-frame to body-frame rotation matrix
     *          4. Transform target angles from earth frame to body frame
     *          5. Store result in _angle_bf_output_rad for servo output
     * 
     *          **Frame Conventions**:
     *          - Input (angle_rad): Earth-frame NED angles in radians
     *            * Roll: Rotation about North axis
     *            * Pitch: Rotation about East axis  
     *            * Yaw: Rotation about Down axis
     *          - Output (_angle_bf_output_rad): Body-frame angles in radians
     *            * Roll: Rotation about vehicle forward axis
     *            * Pitch: Rotation about vehicle right axis
     *            * Yaw: Rotation about vehicle down axis
     * 
     *          **Lead Compensation**:
     *          Predicts vehicle attitude at time (now + lead_time) using:
     *          predicted_angle = current_angle + (gyro_rate * lead_time)
     *          
     *          This compensates for:
     *          - Servo mechanical response delay
     *          - Control loop update latency
     *          - Improves tracking performance during vehicle maneuvers
     * 
     * @param[in] angle_rad Earth-frame target angles in radians (roll, pitch, yaw)
     * 
     * @note Only called when requires_stabilization = true
     * @note Lead compensation parameters tuned for typical servo response times
     * @warning Excessive lead values can cause oscillation or instability
     * 
     * @see AP_AHRS for vehicle attitude source
     * @see move_servo() for final servo output
     */
    void update_angle_outputs(const MountTarget& angle_rad);

    /**
     * @brief Command servo to move to specified angle
     * 
     * @details Outputs PWM command to servo channel for single axis positioning.
     *          Converts body-frame angle in degrees*10 format to PWM microsecond value
     *          using SRV_Channel output mapping.
     * 
     *          **PWM Mapping**:
     *          - angle_min → SRVn_MIN parameter (typically 1000μs)
     *          - angle_max → SRVn_MAX parameter (typically 2000μs)
     *          - Linear interpolation for intermediate angles
     *          - SRVn_REVERSED parameter flips direction if needed
     * 
     *          **Angle Format**:
     *          Input angles in degrees*10 (e.g., 450 = 45.0 degrees) allow:
     *          - Integer arithmetic without floating point
     *          - 0.1 degree resolution
     *          - Compatibility with parameter storage format
     * 
     * @param[in] rc         SRV_Channel function ID (e.g., k_mount_roll, k_mount_tilt)
     * @param[in] angle      Desired angle in body-frame degrees*10 format
     *                       (e.g., 450 = 45.0°, -900 = -90.0°)
     * @param[in] angle_min  Minimum angle limit in degrees*10 (parameter MNTx_ANGLE_MIN)
     * @param[in] angle_max  Maximum angle limit in degrees*10 (parameter MNTx_ANGLE_MAX)
     * 
     * @note Angles are clamped to [angle_min, angle_max] range before output
     * @note SRV_Channel handles final PWM output to hardware
     * @note Called separately for each active axis (roll, tilt, pan)
     * 
     * @see SRV_Channel::set_angle() for PWM output implementation
     */
    void move_servo(uint8_t rc, int16_t angle, int16_t angle_min, int16_t angle_max);

    /**
     * @brief Stabilization requirement flag
     * 
     * Determines gimbal control mode:
     * - true: Simple servo gimbal requiring earth→body frame transformation by backend
     *         Backend performs stabilization using vehicle AHRS attitude
     * - false: Self-stabilizing BrushlessPWM gimbal with integrated controller
     *          Backend passes through earth-frame angles without transformation
     * 
     * Set during construction based on gimbal type, immutable during operation.
     * 
     * @note Critical for correct frame transformation - wrong setting causes unstable behavior
     */
    const bool requires_stabilization;

    /**
     * @brief SRV_Channel function index for roll axis servo
     * 
     * Maps to specific servo channel based on instance number:
     * - Instance 0: SRV_Channel::k_mount_roll
     * - Instance 1: SRV_Channel::k_mount2_roll
     * - k_none if roll control not available for this mount
     * 
     * Assigned during init(), used in move_servo() for PWM output
     */
    SRV_Channel::Function    _roll_idx;

    /**
     * @brief SRV_Channel function index for tilt (pitch) axis servo
     * 
     * Maps to specific servo channel based on instance number:
     * - Instance 0: SRV_Channel::k_mount_tilt
     * - Instance 1: SRV_Channel::k_mount2_tilt
     * - k_none if tilt control not available for this mount
     * 
     * Assigned during init(), used in move_servo() for PWM output
     */
    SRV_Channel::Function    _tilt_idx;

    /**
     * @brief SRV_Channel function index for pan (yaw) axis servo
     * 
     * Maps to specific servo channel based on instance number:
     * - Instance 0: SRV_Channel::k_mount_pan
     * - Instance 1: SRV_Channel::k_mount2_pan
     * - k_none if pan control not available for this mount
     * 
     * Pan control typically required for multicopters to maintain camera heading
     * independent of vehicle yaw changes.
     * 
     * Assigned during init(), used in move_servo() for PWM output
     */
    SRV_Channel::Function    _pan_idx;

    /**
     * @brief SRV_Channel function index for open/shutter mechanism
     * 
     * Maps to specific servo channel based on instance number:
     * - Instance 0: SRV_Channel::k_mount_open
     * - Instance 1: SRV_Channel::k_mount2_open
     * - k_none if open mechanism not available for this mount
     * 
     * Used for camera shutter, lens cover, or other auxiliary mechanism control.
     * 
     * Assigned during init(), used in move_servo() for PWM output
     */
    SRV_Channel::Function    _open_idx;

    /**
     * @brief Final body-frame output angles in radians
     * 
     * Stores computed body-frame servo angles after transformation from earth-frame
     * targets (if stabilization required) or direct pass-through (if self-stabilizing).
     * 
     * Vector components:
     * - x: Roll angle (rotation about vehicle forward axis) in radians
     * - y: Pitch angle (rotation about vehicle right axis) in radians
     * - z: Yaw angle (rotation about vehicle down axis) in radians
     * 
     * These angles are converted to degrees*10 format and sent to servos via move_servo().
     * Also used by get_attitude_quaternion() to report gimbal attitude.
     * 
     * Updated every control cycle in update() method.
     * 
     * @note Body-frame convention: forward-right-down relative to vehicle
     * @note Units: radians (converted to degrees*10 for servo output)
     */
    Vector3f _angle_bf_output_rad;
};
#endif // HAL_MOUNT_SERVO_ENABLED
