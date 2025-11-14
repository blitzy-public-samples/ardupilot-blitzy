/**
 * @file AP_VisualOdom_IntelT265.h
 * @brief Intel RealSense T265 and VOXL tracking camera visual odometry backend
 * 
 * This file provides the backend implementation for Intel RealSense T265 and
 * Qualcomm VOXL tracking cameras. These devices provide visual-inertial odometry
 * (VIO) using stereo cameras and an IMU to estimate 6-DOF pose (position and
 * attitude) and velocity information.
 * 
 * The backend handles:
 * - Coordinate frame transformations from camera frame to vehicle NED frame
 * - Yaw alignment between camera heading and vehicle AHRS/EKF heading
 * - Position alignment and offset corrections for sensor fusion
 * - Quality metrics integration with EKF acceptance thresholds
 * - VOXL camera-specific reset jump detection and handling
 * - Camera mounting orientation configuration
 * 
 * Supported cameras:
 * - Intel RealSense T265: Stereo fisheye cameras with integrated IMU
 * - Qualcomm VOXL: Tracking camera module for PX4 Vision system
 * 
 * @note Camera must have unobstructed field of view for accurate tracking
 * @warning Incorrect mounting orientation configuration can lead to position errors
 */

#pragma once

#include "AP_VisualOdom_config.h"

#if AP_VISUALODOM_INTELT265_ENABLED

#include "AP_VisualOdom_Backend.h"

/**
 * @class AP_VisualOdom_IntelT265
 * @brief Backend for Intel RealSense T265 and VOXL tracking cameras
 * 
 * @details This backend implements visual odometry integration for tracking cameras
 *          that provide pose (position and attitude) estimates. The implementation
 *          handles several critical transformations and alignment operations:
 * 
 *          **Coordinate Frame Transformations:**
 *          - Camera frame → Vehicle body frame (via ORIENT parameter)
 *          - Body frame → Earth NED frame (via yaw alignment)
 *          - Position and velocity rotation and correction
 * 
 *          **Yaw Alignment:**
 *          Camera heading is aligned with vehicle AHRS/EKF heading to ensure
 *          consistent navigation reference. Alignment can be triggered via
 *          MAVLink command or automatically on initialization.
 * 
 *          **Position Alignment:**
 *          Position offset corrections allow the visual odometry position to be
 *          aligned with AHRS position estimate, enabling seamless transitions
 *          between position sources (GPS, visual odometry, etc.).
 * 
 *          **Quality Metrics:**
 *          Position and angle error estimates from the camera are used to determine
 *          data quality. Quality values range from -1 (failed), 0 (unknown),
 *          1 (worst) to 100 (best). EKF uses quality thresholds to decide whether
 *          to consume the data.
 * 
 *          **VOXL Reset Handling:**
 *          VOXL cameras may experience tracking resets that cause position/attitude
 *          jumps. The backend detects these resets via reset_counter changes and
 *          implements a configurable ignore period to prevent corrupted data from
 *          entering the EKF.
 * 
 * @note Typical update rates: 30-90 Hz depending on camera and processing load
 * @warning Camera requires sufficient visual features and lighting for tracking
 * @warning Rapid motion or lack of visual features can cause tracking failure
 * 
 * @see AP_VisualOdom_Backend for base class interface
 */
class AP_VisualOdom_IntelT265 : public AP_VisualOdom_Backend
{

public:

    using AP_VisualOdom_Backend::AP_VisualOdom_Backend;

    /**
     * @brief Consume vision pose estimate data and send to EKF
     * 
     * @details This method processes 6-DOF pose estimates (position and attitude)
     *          from the tracking camera and forwards them to the EKF for sensor
     *          fusion. The method performs several critical operations:
     * 
     *          1. Reset detection: Monitors reset_counter for camera tracking resets
     *          2. Coordinate transformation: Rotates position and attitude from
     *             camera frame to vehicle NED frame using configured orientation
     *          3. Yaw alignment: Applies yaw trim to align camera heading with AHRS
     *          4. Position correction: Adds position offsets for alignment with AHRS
     *          5. Quality filtering: Checks quality metrics against EKF thresholds
     *          6. VOXL reset handling: Detects and handles position/attitude jumps
     * 
     *          The pose data originates from MAVLink VISION_POSITION_ESTIMATE message.
     * 
     * @param[in] remote_time_us  Timestamp from camera in microseconds
     * @param[in] time_ms         System time when measurement was received (milliseconds)
     * @param[in] x               Position X component in camera frame (meters)
     * @param[in] y               Position Y component in camera frame (meters)
     * @param[in] z               Position Z component in camera frame (meters)
     * @param[in] attitude        Attitude quaternion in camera frame
     * @param[in] posErr          Position error estimate from camera (meters)
     * @param[in] angErr          Angular error estimate from camera (radians)
     * @param[in] reset_counter   Camera tracking reset counter (increments on reset)
     * @param[in] quality         Quality metric: -1=failed, 0=unknown, 1=worst, 100=best
     * 
     * @note Called at camera update rate (typically 30-90 Hz)
     * @warning Quality value of -1 causes data to be rejected by EKF
     * @warning Large posErr or angErr values may cause EKF to reject data
     * 
     * @see handle_vision_speed_estimate for velocity measurements
     * @see rotate_and_correct_position for coordinate transformation details
     * @see should_consume_sensor_data for reset handling logic
     */
    void handle_pose_estimate(uint64_t remote_time_us, uint32_t time_ms, float x, float y, float z, const Quaternion &attitude, float posErr, float angErr, uint8_t reset_counter, int8_t quality) override;

    /**
     * @brief Consume vision velocity estimate data and send to EKF
     * 
     * @details This method processes velocity estimates from the tracking camera
     *          and forwards them to the EKF. Velocity data provides additional
     *          information for state estimation, particularly useful when position
     *          estimates are less reliable or when operating in GPS-denied environments.
     * 
     *          Processing steps:
     *          1. Reset detection: Monitors reset_counter changes
     *          2. Coordinate rotation: Transforms velocity from camera frame to NED
     *          3. Quality filtering: Validates quality metrics
     *          4. EKF integration: Forwards velocity to navigation filter
     * 
     *          The velocity data originates from MAVLink VISION_SPEED_ESTIMATE message.
     * 
     * @param[in] remote_time_us  Timestamp from camera in microseconds
     * @param[in] time_ms         System time when measurement was received (milliseconds)
     * @param[in] vel             Velocity vector in camera frame (m/s)
     *                            Components should be in camera's local frame before rotation
     * @param[in] reset_counter   Camera tracking reset counter (increments on reset)
     * @param[in] quality         Quality metric: -1=failed, 0=unknown, 1=worst, 100=best
     * 
     * @note Called at camera update rate, may be same or different rate from pose
     * @note Velocity is in meters per second in NED frame after transformation
     * @warning Quality value of -1 causes data to be rejected by EKF
     * 
     * @see handle_pose_estimate for position and attitude measurements
     * @see rotate_velocity for coordinate transformation details
     */
    void handle_vision_speed_estimate(uint64_t remote_time_us, uint32_t time_ms, const Vector3f &vel, uint8_t reset_counter, int8_t quality) override;

    /**
     * @brief Request sensor's yaw be aligned with vehicle's AHRS/EKF attitude
     * 
     * @details Sets a flag to trigger yaw alignment on the next pose update.
     *          Yaw alignment calculates the rotation difference between the
     *          camera's heading and the vehicle's AHRS heading, then applies
     *          this correction to all subsequent pose data.
     * 
     *          This is typically called:
     *          - After vehicle initialization when AHRS has converged
     *          - Via MAVLink command VISION_POSITION_ESTIMATE
     *          - When switching position sources (e.g., GPS to visual odometry)
     * 
     *          The actual alignment occurs in align_yaw_to_ahrs() when the next
     *          pose estimate is received, ensuring both position and attitude
     *          data are available for the alignment calculation.
     * 
     * @note Alignment is performed asynchronously on next handle_pose_estimate call
     * @note Multiple alignment requests are safe; only one alignment is performed
     * @warning Vehicle should be stationary during yaw alignment for best results
     * 
     * @see align_yaw_to_ahrs for alignment implementation
     * @see _align_yaw flag that controls alignment behavior
     */
    void request_align_yaw_to_ahrs() override { _align_yaw = true; }

    /**
     * @brief Update position offsets to align to AHRS position
     * 
     * @details Sets flags to trigger position alignment on the next pose update.
     *          Position alignment calculates offset corrections that are added to
     *          subsequent position estimates to match the vehicle's current AHRS
     *          position. This enables smooth transitions when switching between
     *          position sources (e.g., GPS → visual odometry).
     * 
     *          The alignment can be applied independently to horizontal (XY) and
     *          vertical (Z) components, allowing selective alignment based on
     *          the quality of different position sources.
     * 
     *          Typical use cases:
     *          - Switching from GPS to visual odometry navigation
     *          - Initializing visual odometry in GPS-denied environment
     *          - Correcting accumulated drift in visual odometry
     * 
     * @param[in] align_xy  If true, align horizontal (X,Y) position with AHRS
     * @param[in] align_z   If true, align vertical (Z/altitude) position with AHRS
     * 
     * @note Should only be called when visual odometry is NOT the active position source
     * @note Actual alignment occurs on next handle_pose_estimate call
     * @warning Aligning while visual odometry is the position source can cause jumps
     * @warning Vehicle should be stationary during alignment for best results
     * 
     * @see align_position_to_ahrs(const Vector3f&, bool, bool) for implementation
     * @see _pos_correction for the calculated position offset
     */
    void align_position_to_ahrs(bool align_xy, bool align_z) override { _align_posxy = align_xy; _align_posz = align_z; }

    /**
     * @brief Pre-arming safety check for visual odometry system
     * 
     * @details Performs validation checks before allowing vehicle arming to ensure
     *          the visual odometry system is functioning correctly. This is a critical
     *          safety check for vehicles relying on visual odometry for position control.
     * 
     *          Checks performed:
     *          1. Recent data reception: Verifies camera is providing fresh data
     *          2. Orientation validity: Confirms camera mounting orientation is supported
     *          3. Attitude validity: Checks that attitude quaternion is properly initialized
     *          4. Quality metrics: Validates recent quality indicators are acceptable
     * 
     *          If any check fails, arming is blocked and a descriptive error message
     *          is provided to help diagnose the issue.
     * 
     * @param[out] failure_msg      Buffer to store failure message if check fails
     * @param[in]  failure_msg_len  Size of failure_msg buffer in bytes
     * 
     * @return true if all checks pass and vehicle can arm, false if arming should be blocked
     * 
     * @note Called during arming sequence and during pre-arm checks
     * @warning Do not arm vehicle if this check fails; position control may be unreliable
     * @warning Check for _error_orientation flag indicating unsupported mounting angle
     * 
     * @see AP_Arming for arming system architecture
     */
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;

protected:

    /**
     * @brief Apply rotation and position correction to transform from camera frame to NED frame
     * 
     * @details Transforms position data through multiple coordinate frame rotations and
     *          applies position offset corrections. The transformation pipeline:
     * 
     *          1. **Camera frame → Body frame**: Apply _posvel_rotation based on camera
     *             mounting orientation (ORIENT parameter). This accounts for how the
     *             camera is physically mounted on the vehicle (forward, down, right, etc.).
     * 
     *          2. **Body frame → Earth NED frame**: Apply _yaw_rotation to align camera
     *             heading with vehicle AHRS heading. This compensates for the arbitrary
     *             initial heading of the camera coordinate system.
     * 
     *          3. **Position correction**: Add _pos_correction offset to align visual
     *             odometry position with AHRS position estimate. This enables seamless
     *             transitions between position sources.
     * 
     *          Coordinate system conventions:
     *          - Camera frame: Varies by model and mounting
     *          - Body frame: X-forward, Y-right, Z-down (standard ArduPilot convention)
     *          - NED frame: X-North, Y-East, Z-Down (earth-fixed)
     * 
     * @param[in,out] position  Position vector in camera frame (meters), modified to NED frame
     * 
     * @note Position units are meters throughout transformation
     * @note All rotations are applied in correct order to maintain coordinate validity
     * @warning Incorrect ORIENT parameter will cause position errors in flight
     * @warning _pos_correction must be in NED frame
     * 
     * @see _posvel_rotation for body frame rotation matrix
     * @see _yaw_rotation for heading alignment quaternion
     * @see _pos_correction for position offset in meters
     */
    void rotate_and_correct_position(Vector3f &position) const;

    /**
     * @brief Apply rotation to transform velocity from camera frame to NED frame
     * 
     * @details Rotates velocity vector through the same coordinate frame transformations
     *          as position data (excluding position offset corrections which don't apply
     *          to velocity). The transformation pipeline:
     * 
     *          1. **Camera frame → Body frame**: Apply _posvel_rotation based on camera
     *             mounting orientation (ORIENT parameter)
     * 
     *          2. **Body frame → Earth NED frame**: Apply _yaw_rotation to align with
     *             vehicle AHRS heading
     * 
     *          Note: Unlike position, velocity does NOT receive offset corrections
     *          (_pos_correction), as offset corrections are constant and their
     *          derivative is zero.
     * 
     * @param[in,out] velocity  Velocity vector in camera frame (m/s), modified to NED frame
     * 
     * @note Velocity units are meters per second (m/s) throughout transformation
     * @note Uses same rotation matrices as position transformation for consistency
     * @warning Incorrect ORIENT parameter will cause velocity direction errors
     * 
     * @see rotate_and_correct_position for related position transformation
     * @see _posvel_rotation for body frame rotation matrix
     * @see _yaw_rotation for heading alignment quaternion
     */
    void rotate_velocity(Vector3f &velocity) const;

    /**
     * @brief Rotate attitude quaternion to align camera heading with vehicle AHRS heading
     * 
     * @details Applies yaw rotation to transform camera attitude into vehicle body frame
     *          with heading aligned to AHRS. This compensates for the arbitrary initial
     *          heading of the camera's coordinate system.
     * 
     *          Transformation steps:
     *          1. Apply _att_rotation if camera has non-standard mounting orientation
     *             (converts camera attitude to body frame)
     *          2. Apply _yaw_rotation to align heading with AHRS
     *             (compensates for camera's arbitrary initial yaw)
     * 
     *          The resulting attitude quaternion represents vehicle orientation in
     *          earth NED frame with yaw aligned to AHRS compass heading.
     * 
     * @param[in,out] attitude  Attitude quaternion in camera frame, modified to aligned body frame
     * 
     * @note Attitude is represented as quaternion (w, x, y, z)
     * @note _yaw_trim is stored in radians and converted to _yaw_rotation quaternion
     * @warning Incorrect attitude can cause attitude control instability
     * 
     * @see _yaw_rotation for heading alignment quaternion derived from _yaw_trim
     * @see _att_rotation for camera mounting orientation compensation
     * @see align_yaw for calculation of yaw alignment trim
     */
    void rotate_attitude(Quaternion &attitude) const;

    /**
     * @brief Calculate rotation to align sensor yaw with AHRS/EKF attitude
     * 
     * @details Uses current AHRS attitude and sensor-provided attitude to compute the
     *          yaw angle difference, then calls align_yaw() to apply this correction.
     *          This establishes the heading alignment between the camera's arbitrary
     *          coordinate system and the vehicle's north-referenced AHRS heading.
     * 
     *          Algorithm:
     *          1. Get current AHRS yaw angle (referenced to magnetic/true north)
     *          2. Apply any existing camera orientation corrections to sensor attitude
     *          3. Extract yaw from corrected sensor attitude
     *          4. Calculate yaw difference: ahrs_yaw - sensor_yaw
     *          5. Call align_yaw() to update _yaw_trim and _yaw_rotation
     * 
     *          This method is called when _align_yaw flag is true, typically after
     *          request_align_yaw_to_ahrs() or during initialization.
     * 
     * @param[in] position  Current position from sensor (used by align_yaw for correction)
     * @param[in] attitude  Current attitude from sensor in camera frame
     * 
     * @return true if alignment successful, false if AHRS not available or invalid
     * 
     * @note Requires valid AHRS attitude for alignment calculation
     * @note Vehicle should be stationary during alignment for accuracy
     * @warning Alignment during motion may introduce heading errors
     * 
     * @see align_yaw for the actual trim application
     * @see _yaw_trim for the calculated yaw correction angle in radians
     * @see request_align_yaw_to_ahrs for triggering alignment
     */
    bool align_yaw_to_ahrs(const Vector3f &position, const Quaternion &attitude);

    /**
     * @brief Align sensor yaw with specified yaw angle
     * 
     * @details Updates yaw trim and rotation to align camera heading with the specified
     *          yaw angle. This method also recalculates position correction to prevent
     *          position jumps when the yaw alignment changes.
     * 
     *          Operations performed:
     *          1. Calculate new yaw trim: difference between new_yaw and current sensor yaw
     *          2. Create yaw rotation quaternion from yaw trim
     *          3. Transform current position using NEW yaw rotation
     *          4. Calculate position correction to maintain continuous position
     *             (prevents jump in reported position when yaw alignment changes)
     * 
     *          The position correction ensures that even though the coordinate frame
     *          rotation changes, the vehicle's reported position remains continuous,
     *          avoiding sudden position jumps that could destabilize the EKF.
     * 
     * @param[in] position  Current sensor position before yaw alignment (camera frame, meters)
     * @param[in] attitude  Current sensor attitude before yaw alignment (camera frame)
     * @param[in] yaw_rad   Target yaw angle to align to (radians, NED frame, 0=North)
     * 
     * @note Called by align_yaw_to_ahrs with AHRS yaw as target
     * @note Updates both _yaw_trim (angle) and _yaw_rotation (quaternion)
     * @note Recalculates _pos_correction to maintain position continuity
     * @warning Incorrect yaw_rad will cause persistent heading offset
     * 
     * @see _yaw_trim for the yaw correction angle in radians
     * @see _yaw_rotation for the quaternion representation of yaw correction
     * @see _pos_correction for position offset updated to prevent jumps
     */
    void align_yaw(const Vector3f &position, const Quaternion &attitude, float yaw_rad);

    /**
     * @brief Determine if sensor data should be consumed or ignored
     * 
     * @details Implements reset detection and temporary data ignore logic to prevent
     *          corrupted data from entering the EKF after camera tracking resets.
     *          When a tracking camera loses and reacquires tracking (due to rapid
     *          motion, lack of features, or other failures), the reset_counter
     *          increments and the camera's position/attitude estimates may contain
     *          jumps or errors during the reacquisition period.
     * 
     *          Reset handling algorithm:
     *          1. Monitor reset_counter from VISION_POSITION_ESTIMATE messages
     *          2. When reset_counter changes, start ignore period
     *          3. Ignore all sensor data for VISUALODOM_RESET_IGNORE_DURATION_MS
     *          4. After ignore period expires, resume normal data consumption
     * 
     *          The VISION_POSITION_ESTIMATE message is the authoritative source for
     *          reset detection. VISION_SPEED_ESTIMATE and other messages check
     *          against the same ignore period but don't update the reset counter.
     * 
     * @param[in] vision_position_estimate  True if checking VISION_POSITION_ESTIMATE
     *                                       message, false for other message types
     * @param[in] reset_counter             Camera's reset counter value from message
     * 
     * @return true if data should be processed and sent to EKF, false if data should be ignored
     * 
     * @note VISUALODOM_RESET_IGNORE_DURATION_MS = 1000 milliseconds (1 second)
     * @note Only VISION_POSITION_ESTIMATE messages update _pos_reset_counter_last
     * @note All message types are ignored during the 1-second ignore period
     * @warning Consuming data immediately after reset can cause EKF divergence
     * @warning Ignore period too short may allow corrupted data into EKF
     * 
     * @see _pos_reset_counter_last for tracking reset counter changes
     * @see _pos_reset_ignore_start_ms for ignore period timing
     */
    bool should_consume_sensor_data(bool vision_position_estimate, uint8_t reset_counter);

    /**
     * @brief Align visual odometry position with AHRS position by updating offset correction
     * 
     * @details Calculates position offset correction to align visual odometry position
     *          estimate with current AHRS position. This enables seamless transitions
     *          when switching from GPS to visual odometry, or when initializing visual
     *          odometry with a known position.
     * 
     *          Algorithm:
     *          1. Get current AHRS position (NED frame, relative to EKF origin)
     *          2. Transform sensor position through coordinate rotations (without existing corrections)
     *          3. Calculate offset: ahrs_position - rotated_sensor_position
     *          4. Apply offset selectively to XY and/or Z based on parameters
     * 
     *          Selective alignment allows using different position sources for horizontal
     *          and vertical axes (e.g., GPS for XY, barometer for Z).
     * 
     * @param[in] sensor_pos  Raw sensor position with scaling only, no rotations or corrections (meters)
     * @param[in] align_xy    If true, align horizontal (North/East) position components
     * @param[in] align_z     If true, align vertical (Down) position component
     * 
     * @return true if alignment successful (AHRS position valid), false otherwise
     * 
     * @note sensor_pos must be raw data before yaw rotation and position corrections
     * @note Updates _pos_correction with calculated offset in NED frame (meters)
     * @note Can be called repeatedly; each call recalculates the correction
     * @warning Should not be called when visual odometry is the active position source
     * @warning Alignment during motion may introduce position errors
     * 
     * @see align_position for the underlying calculation with explicit target position
     * @see _pos_correction for the position offset applied to all subsequent measurements
     */
    bool align_position_to_ahrs(const Vector3f &sensor_pos, bool align_xy, bool align_z);

    /**
     * @brief Align visual odometry position with specified target position
     * 
     * @details Core position alignment implementation that calculates the position
     *          offset correction needed to align raw sensor position with a target
     *          position. This is the underlying calculation used by both AHRS
     *          alignment and other position initialization methods.
     * 
     *          Algorithm:
     *          1. Transform sensor_pos through coordinate rotations (camera → body → NED)
     *          2. Calculate offset for each axis: new_pos[axis] - rotated_sensor_pos[axis]
     *          3. Selectively update _pos_correction based on align_xy and align_z flags
     *          4. Preserve unaligned axes (maintain existing correction for those axes)
     * 
     *          The selective alignment allows independent control of horizontal and
     *          vertical position sources, which is useful when different sensors
     *          provide better accuracy for different axes.
     * 
     * @param[in] sensor_pos  Raw sensor position with scaling only, no rotations or corrections (meters)
     * @param[in] new_pos     Target position in NED frame relative to EKF origin (meters)
     * @param[in] align_xy    If true, update X and Y components of _pos_correction
     * @param[in] align_z     If true, update Z component of _pos_correction
     * 
     * @note sensor_pos must be raw data before yaw rotation and position corrections
     * @note new_pos must be in NED frame with same origin as EKF
     * @note Unaligned axes retain their previous _pos_correction values
     * @note Updates _pos_correction which is added to all subsequent position measurements
     * @warning Incorrect new_pos will cause persistent position offset errors
     * 
     * @see _pos_correction for the position offset vector in NED frame (meters)
     * @see rotate_and_correct_position for how _pos_correction is applied
     */
    void align_position(const Vector3f &sensor_pos, const Vector3f &new_pos, bool align_xy, bool align_z);

    /**
     * @brief Record VOXL camera position and reset counter for jump detection
     * 
     * @details Stores the current position and reset counter from VOXL camera to
     *          enable detection of position/attitude jumps that can occur during
     *          camera tracking resets. VOXL cameras may experience sudden position
     *          offsets when tracking is lost and reacquired, which can destabilize
     *          the EKF if not detected and corrected.
     * 
     *          This method should be called after all coordinate transformations
     *          and corrections have been applied to the position, so that jump
     *          detection compares fully-processed positions.
     * 
     *          Recorded data is used by handle_voxl_camera_reset_jump() to:
     *          - Detect reset_counter changes indicating tracking reset
     *          - Calculate position jump magnitude by comparing positions
     *          - Apply position correction to maintain continuity
     * 
     * @param[in] position       Fully corrected position in NED frame (meters)
     *                           (after scaling, rotation, and offset corrections)
     * @param[in] reset_counter  Current reset counter from VOXL camera
     * 
     * @note Called at end of handle_pose_estimate after position processing
     * @note Position should be post-transformation (all corrections applied)
     * @warning Recording pre-transformation position will cause incorrect jump detection
     * 
     * @see _voxl_position_last for stored previous position
     * @see _voxl_reset_counter_last for stored previous reset counter
     * @see handle_voxl_camera_reset_jump for jump detection and correction
     */
    void record_voxl_position_and_reset_count(const Vector3f &position, uint8_t reset_counter);

    /**
     * @brief Detect and handle VOXL camera tracking reset jumps
     * 
     * @details VOXL cameras can experience position and attitude jumps when tracking
     *          is lost and reacquired. This method detects these jumps by monitoring
     *          reset_counter changes and position discontinuities, then applies
     *          corrections to maintain smooth navigation.
     * 
     *          Jump detection and handling algorithm:
     *          1. Check if reset_counter has changed from _voxl_reset_counter_last
     *          2. If changed, tracking was lost and reacquired (potential jump)
     *          3. Transform current sensor_pos with current rotations
     *          4. Compare with _voxl_position_last to calculate position jump magnitude
     *          5. If jump detected, update position and yaw corrections to maintain continuity
     *          6. Log warning about reset event for diagnostics
     * 
     *          The corrections ensure that even though the camera's internal coordinate
     *          frame may have shifted, the vehicle's reported position and heading
     *          remain continuous, preventing EKF divergence.
     * 
     *          Typical jump scenarios:
     *          - Camera loses visual features (texture-less environment)
     *          - Rapid motion exceeds camera tracking capability
     *          - Lighting changes cause tracking failure
     *          - Camera firmware resets or errors
     * 
     * @param[in] sensor_pos     Raw sensor position with scaling only, no corrections (meters)
     * @param[in] sensor_att     Raw sensor attitude from camera frame
     * @param[in] reset_counter  Current reset counter from VOXL camera
     * 
     * @note Called early in handle_pose_estimate before normal processing
     * @note Requires valid _voxl_position_last from previous update
     * @note Updates _pos_correction and potentially _yaw_trim to maintain continuity
     * @warning Jump correction may not be perfect for very large jumps (>10m)
     * @warning Frequent resets indicate poor tracking conditions
     * 
     * @see record_voxl_position_and_reset_count for recording previous state
     * @see _voxl_reset_counter_last for detecting reset_counter changes
     * @see _voxl_position_last for calculating jump magnitude
     */
    void handle_voxl_camera_reset_jump(const Vector3f &sensor_pos, const Quaternion &sensor_att, uint8_t reset_counter);

    /**
     * @name Coordinate Transformation Members
     * Variables controlling coordinate frame transformations from camera frame to vehicle NED frame
     * @{
     */
    
    /** @brief Yaw angle trim in radians to align camera heading with AHRS/EKF heading
     *  @details Calculated during yaw alignment, represents rotation about Z-axis (down)
     *           to transform camera yaw to match AHRS compass heading. Applied as
     *           _yaw_rotation quaternion to all attitude, position, and velocity data.
     *           Units: radians, Range: -π to +π */
    float _yaw_trim;
    
    /** @brief Earth-frame yaw rotation quaternion derived from _yaw_trim
     *  @details Quaternion representation of yaw trim for efficient rotation operations.
     *           Applied to transform data from camera heading to AHRS-aligned heading.
     *           Used when _yaw_trim is non-zero after yaw alignment. */
    Quaternion _yaw_rotation;
    
    /** @brief Body-frame rotation quaternion for camera mounting orientation
     *  @details Rotation corresponding to ORIENT parameter, transforms attitude from
     *           camera mounting frame to standard vehicle body frame (X-forward, Y-right, Z-down).
     *           Examples: ORIENT=0 (forward), ORIENT=25 (down), ORIENT=2 (right).
     *           Used when get_orientation() != ROTATION_NONE. */
    Quaternion _att_rotation;
    
    /** @brief Rotation matrix for position and velocity coordinate transformation
     *  @details 3x3 rotation matrix to transform position and velocity vectors from
     *           camera frame to vehicle body frame based on camera mounting orientation.
     *           Derived from ORIENT parameter. Applied when _use_posvel_rotation is true.
     *           More efficient than quaternion for vector transformations. */
    Matrix3f _posvel_rotation;
    
    /** @brief Position offset correction vector in NED frame
     *  @details Constant offset added to all position measurements after rotation to
     *           align visual odometry position with AHRS or other reference position.
     *           Enables seamless position source transitions (e.g., GPS → visual odom).
     *           Updated by align_position methods. Units: meters (NED frame) */
    Vector3f _pos_correction;
    
    /** @} */
    
    /**
     * @name Control Flags
     * Boolean flags controlling transformation and alignment behavior
     * @{
     */
    
    /** @brief Enable attitude rotation via _att_rotation
     *  @details True if camera mounting orientation requires attitude transformation.
     *           Set based on ORIENT parameter configuration. */
    bool _use_att_rotation;
    
    /** @brief Enable position/velocity rotation via _posvel_rotation
     *  @details True if camera mounting requires position/velocity transformation.
     *           Set based on ORIENT parameter configuration. */
    bool _use_posvel_rotation;
    
    /** @brief Request yaw alignment on next pose update
     *  @details Set by request_align_yaw_to_ahrs(), cleared after alignment performed.
     *           Default true to align on first pose reception. */
    bool _align_yaw = true;
    
    /** @brief Request horizontal (XY) position alignment on next pose update
     *  @details Set by align_position_to_ahrs() public method. */
    bool _align_posxy;
    
    /** @brief Request vertical (Z) position alignment on next pose update
     *  @details Set by align_position_to_ahrs() public method. */
    bool _align_posz;
    
    /** @brief Flag indicating unsupported camera mounting orientation
     *  @details True if ORIENT parameter specifies orientation that cannot be handled.
     *           Causes pre_arm_check to fail with error message. */
    bool _error_orientation;
    
    /** @} */
    
    /**
     * @name State Tracking Members
     * Variables for tracking camera state and detecting resets
     * @{
     */
    
    /** @brief Last received attitude quaternion from camera
     *  @details Stored for pre-arm validation checks to ensure attitude is reasonable.
     *           Updated on each handle_pose_estimate call. */
    Quaternion _attitude_last;
    
    /** @brief Last VISION_POSITION_ESTIMATE reset counter value
     *  @details Tracks reset_counter from position messages to detect camera tracking
     *           resets. Increments indicate lost tracking and reacquisition. */
    uint8_t _pos_reset_counter_last;
    
    /** @brief System time when sensor data ignore period started
     *  @details Timestamp (milliseconds) when reset was detected. Data is ignored for
     *           VISUALODOM_RESET_IGNORE_DURATION_MS (1 second) after this time.
     *           Value of 0 indicates no active ignore period. */
    uint32_t _pos_reset_ignore_start_ms;
    
    /** @} */
    
    /**
     * @name VOXL Camera Reset Jump Handling
     * Variables specific to VOXL camera position/attitude jump detection and correction
     * @{
     */
    
    /** @brief Last reset counter from VOXL camera for jump detection
     *  @details Separate from _pos_reset_counter_last, used specifically for detecting
     *           VOXL camera origin jumps that require position correction. */
    uint8_t _voxl_reset_counter_last;
    
    /** @brief Last recorded position for VOXL jump magnitude calculation
     *  @details Stored position after all transformations and corrections applied.
     *           Used to calculate jump vector when reset_counter changes.
     *           Units: meters in NED frame */
    Vector3f _voxl_position_last;
    
    /** @} */
};

#endif  // AP_VISUALODOM_INTELT265_ENABLED
