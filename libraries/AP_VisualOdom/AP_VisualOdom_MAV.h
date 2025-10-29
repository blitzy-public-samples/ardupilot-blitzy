/**
 * @file AP_VisualOdom_MAV.h
 * @brief MAVLink visual odometry backend for ArduPilot
 * 
 * @details This file implements the MAVLink backend for visual odometry integration,
 *          processing VISION_POSITION_ESTIMATE and VISION_SPEED_ESTIMATE MAVLink
 *          messages from external visual odometry sources (e.g., Intel T265, ZED camera,
 *          AprilTag systems) and forwarding the data to the EKF for sensor fusion.
 * 
 *          The backend handles coordinate frame transformations from camera frames
 *          to the vehicle's NED (North-East-Down) frame, applies quality filtering,
 *          and manages sensor reset detection for proper EKF integration.
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_VisualOdom_config.h"

#if AP_VISUALODOM_MAV_ENABLED

#include "AP_VisualOdom_Backend.h"

/**
 * @class AP_VisualOdom_MAV
 * @brief MAVLink backend for visual odometry sensor integration
 * 
 * @details This backend processes visual odometry data received via MAVLink messages
 *          and integrates it into ArduPilot's Extended Kalman Filter (EKF) for enhanced
 *          position and velocity estimation. It serves as an adapter between MAVLink
 *          visual odometry messages and the AP_VisualOdom framework.
 * 
 *          The backend handles two primary MAVLink message types:
 *          - VISION_POSITION_ESTIMATE: 6DOF pose estimates (position + attitude)
 *          - VISION_SPEED_ESTIMATE: 3D velocity estimates in NED frame
 * 
 *          Key features:
 *          - Quality-based filtering to reject unreliable measurements
 *          - Reset counter tracking for sensor discontinuity detection
 *          - Coordinate frame transformations from camera to vehicle frames
 *          - Timestamp synchronization between visual odometry system and autopilot
 *          - Error metric forwarding for EKF weighting
 * 
 *          Typical use cases:
 *          - Indoor navigation with camera-based positioning (T265, ZED)
 *          - GPS-denied environments with visual-inertial odometry
 *          - Precision landing with visual markers (AprilTags, IR beacons)
 *          - Obstacle avoidance with depth camera integration
 * 
 * @note This backend requires external visual odometry system sending MAVLink messages
 *       at sufficient rate (typically 30-100 Hz for position, 10-50 Hz for velocity)
 * 
 * @warning Improper sensor mounting or frame transformations will cause position drift
 *          and potential loss of position control. Ensure camera frame orientation is
 *          correctly configured in parameters.
 * 
 * @see AP_VisualOdom_Backend for base class interface
 * @see AP_AHRS for coordinate frame definitions
 */
class AP_VisualOdom_MAV : public AP_VisualOdom_Backend
{

public:
    // constructor
    using AP_VisualOdom_Backend::AP_VisualOdom_Backend;

    /**
     * @brief Process vision pose estimate from MAVLink VISION_POSITION_ESTIMATE message
     * 
     * @details This method consumes 6DOF pose data from external visual odometry sources
     *          and forwards it to the EKF for sensor fusion. The pose data includes both
     *          position (x, y, z) and attitude (quaternion) in the camera's reference frame.
     * 
     *          Processing steps:
     *          1. Quality check: Reject measurements with quality < 0 (failed sensor)
     *          2. Coordinate transformation: Convert from camera frame to NED frame
     *          3. Reset detection: Monitor reset_counter for sensor reinitialization
     *          4. Error metric application: Apply position and angle noise floors
     *          5. EKF integration: Forward processed data to navigation filter
     * 
     *          The quality metric is used to weight the measurement in the EKF:
     *          - Quality = -1: Sensor failed, measurement rejected
     *          - Quality = 0: Unknown quality, uses default noise parameters
     *          - Quality = 1-100: Percentage-based quality (1=worst, 100=best)
     * 
     *          Position and angular errors are constrained to minimum noise floors
     *          to prevent over-confidence in the EKF that could lead to position
     *          lock failures or oscillations.
     * 
     * @param[in] remote_time_us Timestamp from visual odometry system in microseconds
     *                           (remote system's clock, not autopilot clock)
     * @param[in] time_ms        Autopilot timestamp in milliseconds when message received
     * @param[in] x              Position X coordinate in meters (camera frame)
     * @param[in] y              Position Y coordinate in meters (camera frame)
     * @param[in] z              Position Z coordinate in meters (camera frame)
     * @param[in] attitude       Orientation quaternion (camera frame relative to reference)
     * @param[in] posErr         Position estimate error (standard deviation) in meters
     * @param[in] angErr         Angular estimate error (standard deviation) in radians
     * @param[in] reset_counter  Increments when sensor resets/reinitializes (for discontinuity detection)
     * @param[in] quality        Quality metric: -1=failed, 0=unknown, 1-100=quality percentage
     * 
     * @note This method is called at the rate of incoming VISION_POSITION_ESTIMATE messages,
     *       typically 30-100 Hz for real-time visual odometry systems
     * 
     * @warning Position and attitude must be in consistent coordinate frames. Mixing frames
     *          (e.g., camera frame position with NED attitude) will cause navigation failures.
     *          Ensure external visual odometry system is configured correctly.
     * 
     * @warning High message rates (>100 Hz) may exceed EKF processing capacity and cause
     *          timing issues. Typical rates of 30-50 Hz are recommended for most applications.
     * 
     * @see handle_vision_speed_estimate() for velocity-only measurements
     * @see AP_AHRS::get_rotation_body_to_ned() for coordinate frame transformations
     */
    void handle_pose_estimate(uint64_t remote_time_us, uint32_t time_ms, float x, float y, float z, const Quaternion &attitude, float posErr, float angErr, uint8_t reset_counter, int8_t quality) override;

    /**
     * @brief Process vision velocity estimate from MAVLink VISION_SPEED_ESTIMATE message
     * 
     * @details This method consumes 3D velocity data from external visual odometry sources
     *          and forwards it to the EKF for sensor fusion. Velocity estimates complement
     *          or substitute position estimates, particularly useful when absolute position
     *          is unavailable but relative motion can be tracked (e.g., optical flow).
     * 
     *          Processing steps:
     *          1. Quality check: Reject measurements with quality < 0 (failed sensor)
     *          2. Frame validation: Ensure velocity is in NED (North-East-Down) frame
     *          3. Reset detection: Monitor reset_counter for sensor reinitialization
     *          4. Velocity constraint: Apply sanity checks for physically unrealistic velocities
     *          5. EKF integration: Forward velocity data to navigation filter
     * 
     *          The quality metric is used to weight the measurement in the EKF:
     *          - Quality = -1: Sensor failed, measurement rejected
     *          - Quality = 0: Unknown quality, uses default velocity noise parameters
     *          - Quality = 1-100: Percentage-based quality (1=worst, 100=best)
     * 
     *          Velocity measurements are particularly valuable for:
     *          - Complementing GPS velocity when visual odometry provides higher rate data
     *          - GPS-denied navigation where velocity integration provides position
     *          - Improving dynamic response during aggressive maneuvers
     *          - Detecting and rejecting GPS velocity glitches
     * 
     * @param[in] remote_time_us Timestamp from visual odometry system in microseconds
     *                           (remote system's clock, not autopilot clock)
     * @param[in] time_ms        Autopilot timestamp in milliseconds when message received
     * @param[in] vel            Velocity vector in NED frame (North, East, Down) in m/s
     *                           - vel.x: North velocity in m/s (positive = north)
     *                           - vel.y: East velocity in m/s (positive = east)
     *                           - vel.z: Down velocity in m/s (positive = down)
     * @param[in] reset_counter  Increments when sensor resets/reinitializes (for discontinuity detection)
     * @param[in] quality        Quality metric: -1=failed, 0=unknown, 1-100=quality percentage
     * 
     * @note This method is called at the rate of incoming VISION_SPEED_ESTIMATE messages,
     *       typically 10-50 Hz for visual odometry velocity estimates. Lower rates than
     *       pose estimates are common since velocity can be derived from pose differences.
     * 
     * @note Velocity must be provided in NED frame, NOT camera frame. The external visual
     *       odometry system is responsible for frame transformation before sending MAVLink.
     * 
     * @warning Incorrect velocity frame (body frame instead of NED) will cause incorrect
     *          velocity estimates during vehicle rotation, leading to position drift and
     *          potential loss of position control.
     * 
     * @warning Velocity measurements must be synchronized with pose measurements if both
     *          are sent. Large timing mismatches (>100ms) can cause EKF inconsistencies.
     * 
     * @see handle_pose_estimate() for position and attitude measurements
     * @see AP_AHRS for NED coordinate frame definition
     */
    void handle_vision_speed_estimate(uint64_t remote_time_us, uint32_t time_ms, const Vector3f &vel, uint8_t reset_counter, int8_t quality) override;
};

#endif  // AP_VISUALODOM_MAV_ENABLED
