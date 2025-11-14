/**
 * @file AP_VisualOdom.h
 * @brief Visual Odometry subsystem for GPS-denied navigation using camera-based positioning
 * 
 * @details This module provides the interface for visual odometry sensors that enable
 *          position and velocity estimation in GPS-denied environments. Visual odometry
 *          uses camera-based systems to track features and estimate motion, providing
 *          6-DOF pose estimates (position + orientation) to the Extended Kalman Filter (EKF).
 * 
 *          The AP_VisualOdom class acts as a singleton manager that:
 *          - Manages backend selection based on the _TYPE parameter
 *          - Handles pose and velocity estimates from various visual odometry sources
 *          - Provides parameter management for sensor configuration
 *          - Integrates visual odometry data with the navigation EKF
 *          - Performs health monitoring and quality assessment
 * 
 *          Supported backends include:
 *          - MAVLink: Generic visual odometry data via MAVLink protocol
 *          - Intel RealSense T265: Tracking camera with onboard VIO processing
 *          - VOXL: Qualcomm Flight RB5 visual-inertial odometry
 * 
 * @note Visual odometry requires careful camera mounting, calibration, and field of view
 *       considerations. Performance degrades in low-texture environments.
 * 
 * @warning Camera mounting orientation must match the _ORIENT parameter. Incorrect
 *          orientation will cause position drift and potential vehicle crashes.
 * 
 * @see AP_VisualOdom_Backend for backend implementation details
 * @see libraries/AP_NavEKF3/ for EKF integration
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "AP_VisualOdom_config.h"

#if HAL_VISUALODOM_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_config.h>
#if HAL_GCS_ENABLED
#include <GCS_MAVLink/GCS_MAVLink.h>
#endif
#include <AP_Math/AP_Math.h>

class AP_VisualOdom_Backend;

/**
 * @def AP_VISUALODOM_TIMEOUT_MS
 * @brief Health timeout threshold in milliseconds for visual odometry sensor
 * 
 * @details If no valid pose or velocity updates are received within this timeout period,
 *          the visual odometry sensor is considered unhealthy. The healthy() method will
 *          return false, and the EKF may switch to alternative navigation sources.
 * 
 * @note Value of 300ms allows for typical sensor update rates of 30-200Hz with margin
 */
#define AP_VISUALODOM_TIMEOUT_MS 300

/**
 * @class AP_VisualOdom
 * @brief Visual Odometry manager providing camera-based position and velocity estimation
 * 
 * @details The AP_VisualOdom class is a singleton manager that integrates visual odometry
 *          sensors for GPS-denied navigation. It provides a hardware abstraction layer for
 *          multiple visual odometry backend types and manages the data flow from sensors
 *          to the navigation Extended Kalman Filter (EKF).
 * 
 *          Key responsibilities:
 *          - Backend Selection: Instantiates appropriate backend based on _TYPE parameter
 *          - Data Processing: Receives pose and velocity estimates from backends
 *          - Coordinate Transformation: Applies orientation and position offset corrections
 *          - Quality Monitoring: Tracks sensor health and estimate quality metrics
 *          - EKF Integration: Forwards validated estimates to navigation filter
 *          - Parameter Management: Exposes configuration parameters for sensor tuning
 * 
 *          Configuration Parameters:
 *          - _TYPE: Backend type selection (None, MAV, IntelT265, VOXL)
 *          - _ORIENT: Camera mounting orientation (ROTATION_* enum)
 *          - _POS_OFFSET: Camera position relative to vehicle center of mass (meters)
 *          - _POS_SCALE: Position scaling factor for calibration correction
 *          - _DELAY_MS: Average sensor measurement delay for EKF time alignment
 *          - _VEL_NOISE: Velocity measurement noise standard deviation (m/s)
 *          - _POS_NOISE: Position measurement noise standard deviation (meters)
 *          - _YAW_NOISE: Yaw measurement noise standard deviation (radians)
 *          - _QUALITY_MIN: Minimum quality threshold for accepting measurements
 * 
 *          Coordinate Frame Conventions:
 *          - Input poses are in the camera frame (depends on physical mounting)
 *          - Poses are transformed to vehicle body frame using _ORIENT parameter
 *          - Velocities are expected in NED (North-East-Down) frame
 *          - Position offsets are expressed in body frame (forward, right, down)
 * 
 *          Health Monitoring:
 *          - healthy() returns true if data received within AP_VISUALODOM_TIMEOUT_MS
 *          - quality() returns sensor-reported quality metric (-1 to 100)
 *          - Quality < _QUALITY_MIN causes measurements to be rejected
 * 
 * @note This is a singleton class - use AP_VisualOdom::get_singleton() or AP::visualodom()
 * 
 * @warning Visual odometry accuracy depends heavily on:
 *          - Sufficient ambient lighting and scene texture
 *          - Correct camera calibration (intrinsics and extrinsics)
 *          - Accurate _ORIENT and _POS_OFFSET configuration
 *          - Appropriate field of view for vehicle velocity and altitude
 *          - Minimal camera motion blur and rolling shutter effects
 * 
 * @see AP_VisualOdom_Backend base class for backend implementations
 * @see AP_VisualOdom_MAV for MAVLink visual odometry backend
 * @see AP_VisualOdom_IntelT265 for Intel RealSense T265 backend
 */
class AP_VisualOdom
{
public:

    /**
     * @brief Constructor for AP_VisualOdom singleton
     * 
     * @details Initializes parameters and sets up the visual odometry manager.
     *          Backend instantiation occurs during init() call.
     */
    AP_VisualOdom();

    /**
     * @brief Get the singleton instance of AP_VisualOdom
     * 
     * @return Pointer to the singleton AP_VisualOdom instance, or nullptr if not instantiated
     * 
     * @note Prefer using AP::visualodom() namespace accessor for cleaner code
     * @see AP::visualodom()
     */
    static AP_VisualOdom *get_singleton() {
        return _singleton;
    }

    /**
     * @enum VisualOdom_Type
     * @brief Visual odometry backend type selection
     * 
     * @details Defines the available visual odometry backends that can be selected via
     *          the _TYPE parameter. Each backend implements the AP_VisualOdom_Backend
     *          interface and handles communication with a specific sensor type.
     * 
     * @note Backend availability depends on compile-time feature flags:
     *       - AP_VISUALODOM_MAV_ENABLED: Enable MAVLink backend
     *       - AP_VISUALODOM_INTELT265_ENABLED: Enable Intel T265 and VOXL backends
     */
    enum class VisualOdom_Type {
        /**
         * @brief Visual odometry disabled
         * 
         * @details No backend instantiated, all methods return invalid/unhealthy status
         */
        None         = 0,
#if AP_VISUALODOM_MAV_ENABLED
        /**
         * @brief MAVLink visual odometry backend
         * 
         * @details Receives visual odometry data via MAVLink messages:
         *          - VISION_POSITION_ESTIMATE (message ID 102)
         *          - VISION_SPEED_ESTIMATE (message ID 103)
         *          - VISION_POSITION_DELTA (message ID 11011)
         * 
         * @note Generic backend supporting any MAVLink-capable visual odometry source
         * @see handle_vision_position_delta_msg()
         */
        MAV          = 1,
#endif
#if AP_VISUALODOM_INTELT265_ENABLED
        /**
         * @brief Intel RealSense T265 tracking camera backend
         * 
         * @details Direct integration with Intel RealSense T265 tracking camera via
         *          librealsense2 API. Provides Visual-Inertial Odometry (VIO) with
         *          onboard IMU fusion and loop closure detection.
         * 
         * @note Requires librealsense2 library and compatible hardware platform
         * @warning T265 performs best with moderate lighting and textured environments
         */
        IntelT265    = 2,
        /**
         * @brief Qualcomm VOXL visual-inertial odometry backend
         * 
         * @details Integration with Qualcomm Flight RB5 (VOXL) platform VIO system.
         *          Uses hardware-accelerated visual-inertial odometry running on
         *          Snapdragon DSP for low-latency position estimation.
         * 
         * @note Specific to Qualcomm VOXL hardware platform
         */
        VOXL         = 3,
#endif
    };

    /**
     * @brief Detect and initialize visual odometry sensors
     * 
     * @details Instantiates the appropriate backend based on the _TYPE parameter and
     *          performs backend-specific initialization. For hardware backends (T265, VOXL),
     *          this includes device detection and configuration. For MAVLink backend,
     *          this prepares message handling.
     * 
     * @note Should be called once during vehicle startup after parameter loading
     * @note If _TYPE is None or backend initialization fails, the system remains disabled
     * 
     * @see enabled() to check if initialization succeeded
     */
    void init();

    /**
     * @brief Check if visual odometry is enabled
     * 
     * @return true if _TYPE parameter is not None and backend is instantiated
     * @return false if visual odometry is disabled or backend initialization failed
     * 
     * @note This checks configuration, not sensor health - use healthy() for health status
     * @see healthy()
     */
    bool enabled() const;

    /**
     * @brief Check if visual odometry sensor is healthy
     * 
     * @return true if valid data received within AP_VISUALODOM_TIMEOUT_MS (300ms)
     * @return false if no recent data, sensor disabled, or backend reports unhealthy
     * 
     * @details Health status is based on:
     *          - Recent measurement reception (within timeout threshold)
     *          - Backend-specific health indicators
     *          - Quality metric above minimum threshold (if configured)
     * 
     * @note The EKF uses this status to determine visual odometry availability
     * @see quality() for sensor quality metric
     * @see AP_VISUALODOM_TIMEOUT_MS
     */
    bool healthy() const;

    /**
     * @brief Get user-defined camera mounting orientation
     * 
     * @return Camera orientation as a Rotation enum (e.g., ROTATION_NONE, ROTATION_YAW_90)
     * 
     * @details The orientation defines how the camera frame relates to the vehicle body frame.
     *          This rotation is applied to incoming pose estimates to transform them from
     *          camera frame to body frame before EKF integration.
     * 
     * @note Configured via _ORIENT parameter (default: ROTATION_NONE)
     * @warning Incorrect orientation causes position drift and navigation failures
     * 
     * @see enum Rotation in AP_Math/rotations.h
     */
    enum Rotation get_orientation() const { return (enum Rotation)_orientation.get(); }

    /**
     * @brief Get user-defined position scaling factor
     * 
     * @return Position scale factor (dimensionless, typically 1.0)
     * 
     * @details Scaling factor applied to all incoming position estimates for calibration
     *          correction. Values != 1.0 indicate sensor miscalibration compensation.
     *          For example, scale = 0.95 reduces all position estimates by 5%.
     * 
     * @note Configured via _POS_SCALE parameter (default: 1.0)
     * @note Typically only needed if sensor camera calibration is incorrect
     */
    float get_pos_scale() const { return _pos_scale; }

    /**
     * @brief Get camera position offset relative to vehicle body frame origin
     * 
     * @return 3D position offset vector in meters (forward, right, down in body frame)
     * 
     * @details Returns the camera's physical mounting position relative to the vehicle's
     *          center of mass or IMU origin. This offset is used to transform camera-based
     *          position estimates to the vehicle reference point.
     * 
     *          Body frame convention:
     *          - X: Forward (positive = ahead of origin)
     *          - Y: Right (positive = right of origin)  
     *          - Z: Down (positive = below origin)
     * 
     * @note Configured via _POS_OFFSET parameter (default: 0,0,0)
     * @warning Incorrect offset causes position errors proportional to vehicle rotation rates
     * 
     * @see libraries/AP_Math/vector3.h for Vector3f details
     */
    const Vector3f &get_pos_offset(void) const { return _pos_offset; }

    /**
     * @brief Get sensor measurement delay for EKF time alignment
     * 
     * @return Measurement delay in milliseconds (non-negative)
     * 
     * @details Returns the average delay between the physical measurement time and when
     *          the data is received by ArduPilot. The EKF uses this to properly time-align
     *          visual odometry measurements with IMU data for optimal fusion.
     * 
     *          Typical delays:
     *          - MAVLink over radio: 50-200ms
     *          - USB connection: 20-50ms
     *          - Direct hardware interface: 10-30ms
     * 
     * @note Configured via _DELAY_MS parameter (default: 25ms)
     * @note Negative values are clamped to zero
     * 
     * @see libraries/AP_NavEKF3/ for EKF time alignment implementation
     */
    uint16_t get_delay_ms() const { return MAX(0, _delay_ms); }

    /**
     * @brief Get velocity measurement noise standard deviation
     * 
     * @return Velocity noise in meters per second (m/s)
     * 
     * @details Returns the expected 1-sigma (standard deviation) noise level in velocity
     *          measurements. The EKF uses this to weight visual odometry velocity updates
     *          relative to other velocity sources (GPS, optical flow, etc.).
     * 
     * @note Configured via _VEL_NOISE parameter (default: 0.1 m/s)
     * @note Lower values give more weight to visual odometry, higher values trust it less
     */
    float get_vel_noise() const { return _vel_noise; }
    
    /**
     * @brief Get position measurement noise standard deviation
     * 
     * @return Position noise in meters (m)
     * 
     * @details Returns the expected 1-sigma (standard deviation) noise level in position
     *          measurements. The EKF uses this to weight visual odometry position updates
     *          relative to other position sources (GPS, beacons, etc.).
     * 
     * @note Configured via _POS_NOISE parameter (default: 0.1 m)
     * @note Lower values give more weight to visual odometry, higher values trust it less
     */
    float get_pos_noise() const { return _pos_noise; }

    /**
     * @brief Get yaw (heading) measurement noise standard deviation
     * 
     * @return Yaw noise in radians (rad)
     * 
     * @details Returns the expected 1-sigma (standard deviation) noise level in yaw
     *          (heading) measurements. The EKF uses this to weight visual odometry
     *          yaw updates relative to compass and other heading sources.
     * 
     * @note Configured via _YAW_NOISE parameter (default: 0.1 rad ≈ 5.7 degrees)
     * @note Lower values give more weight to visual odometry yaw, higher values trust it less
     */
    float get_yaw_noise() const { return _yaw_noise; }

    /**
     * @brief Get minimum quality threshold for accepting measurements
     * 
     * @return Minimum quality value (0-100), or 0 to accept all measurements
     * 
     * @details Returns the configured minimum quality threshold. Measurements with quality
     *          below this value are rejected and not sent to the EKF. Quality of 0 means
     *          all measurements are accepted regardless of reported quality.
     * 
     * @note Configured via _QUALITY_MIN parameter (default: 0)
     * @see quality() for current sensor quality metric
     */
    int8_t get_quality_min() const { return _quality_min; }

    /**
     * @brief Get current visual odometry quality metric
     * 
     * @return Quality value with convention:
     *         - -1: Sensor failed or quality check failed
     *         -  0: Quality unknown (sensor doesn't report quality)
     *         -  1-100: Quality percentage (1 = worst, 100 = best)
     * 
     * @details The quality metric indicates the sensor's confidence in its current estimate.
     *          Different backends report quality differently:
     *          - T265: Tracking confidence level
     *          - MAVLink: Value from VISION_POSITION_ESTIMATE quality field
     *          - VOXL: Platform-specific confidence metric
     * 
     *          Low quality typically indicates:
     *          - Insufficient visual features in the scene
     *          - Poor lighting conditions
     *          - Excessive motion blur
     *          - Tracking loss or drift
     * 
     * @note Quality below _QUALITY_MIN parameter causes measurements to be rejected
     * @see get_quality_min()
     */
    int8_t quality() const;

#if HAL_GCS_ENABLED
    /**
     * @brief Process VISION_POSITION_DELTA MAVLink message
     * 
     * @param msg Reference to received MAVLink message (ID 11011)
     * 
     * @details Handles the VISION_POSITION_DELTA MAVLink message which provides incremental
     *          position and attitude changes from a visual odometry sensor. This message
     *          format is useful for high-rate odometry streams where sending absolute
     *          position would be bandwidth-intensive.
     * 
     *          Message fields processed:
     *          - time_delta_usec: Time since last measurement
     *          - position_delta: Position change in body frame (meters)
     *          - angle_delta: Attitude change in body frame (radians)
     *          - confidence: Measurement quality (0.0-1.0, mapped to 0-100 quality)
     * 
     * @note Only available when HAL_GCS_ENABLED compile flag is set
     * @note Requires MAVLink backend (_TYPE = MAV) to be active
     * 
     * @see handle_pose_estimate() for absolute pose updates
     * @see libraries/GCS_MAVLink/GCS_Common.cpp for message handling
     */
    void handle_vision_position_delta_msg(const mavlink_message_t &msg);
#endif

    /**
     * @brief Process visual odometry pose estimate with Euler angles
     * 
     * @param[in] remote_time_us Sensor timestamp in microseconds (sensor's local time)
     * @param[in] time_ms System time in milliseconds when measurement was received
     * @param[in] x Position X coordinate in meters (forward in NED frame)
     * @param[in] y Position Y coordinate in meters (right in NED frame)
     * @param[in] z Position Z coordinate in meters (down in NED frame)
     * @param[in] roll Roll angle in radians
     * @param[in] pitch Pitch angle in radians
     * @param[in] yaw Yaw angle in radians
     * @param[in] posErr Position error standard deviation in meters
     * @param[in] angErr Angular error standard deviation in radians
     * @param[in] reset_counter Sensor reset counter (increments on tracking reinitialization)
     * @param[in] quality Quality metric: -1 = failed, 0 = unknown, 1-100 = quality percentage
     * 
     * @details General-purpose method to ingest pose estimates from any visual odometry source.
     *          Applies configured transformations (_ORIENT, _POS_OFFSET, _POS_SCALE) and
     *          forwards validated estimates to the EKF for sensor fusion.
     * 
     *          Processing steps:
     *          1. Check quality against _QUALITY_MIN threshold
     *          2. Apply position scaling (_POS_SCALE)
     *          3. Apply camera orientation transformation (_ORIENT)
     *          4. Apply position offset compensation (_POS_OFFSET)
     *          5. Convert Euler angles to quaternion
     *          6. Forward to EKF with configured noise parameters
     * 
     * @note Position coordinates are in NED (North-East-Down) frame
     * @note Euler angles follow aerospace convention: roll-pitch-yaw sequence
     * @note reset_counter changes indicate sensor reinitialization; EKF handles discontinuities
     * @note Measurements below _QUALITY_MIN threshold are silently discarded
     * 
     * @warning Coordinate frame must match EKF expectations (NED). Incorrect frames cause drift.
     * @warning Units must be exact: meters for position, radians for angles
     * 
     * @see handle_pose_estimate(Quaternion) for quaternion attitude variant
     * @see libraries/AP_NavEKF3/AP_NavEKF3_core.cpp for EKF integration
     */
    void handle_pose_estimate(uint64_t remote_time_us, uint32_t time_ms, float x, float y, float z, float roll, float pitch, float yaw, float posErr, float angErr, uint8_t reset_counter, int8_t quality);
    
    /**
     * @brief Process visual odometry pose estimate with quaternion attitude
     * 
     * @param[in] remote_time_us Sensor timestamp in microseconds (sensor's local time)
     * @param[in] time_ms System time in milliseconds when measurement was received
     * @param[in] x Position X coordinate in meters (forward in NED frame)
     * @param[in] y Position Y coordinate in meters (right in NED frame)
     * @param[in] z Position Z coordinate in meters (down in NED frame)
     * @param[in] attitude Attitude quaternion (rotation from NED to body frame)
     * @param[in] posErr Position error standard deviation in meters
     * @param[in] angErr Angular error standard deviation in radians
     * @param[in] reset_counter Sensor reset counter (increments on tracking reinitialization)
     * @param[in] quality Quality metric: -1 = failed, 0 = unknown, 1-100 = quality percentage
     * 
     * @details Quaternion variant of handle_pose_estimate. Preferred over Euler angle variant
     *          as it avoids gimbal lock and trigonometric conversions. Processing is otherwise
     *          identical to the Euler angle version.
     * 
     * @note Quaternion must be normalized (magnitude = 1.0)
     * @note Quaternion represents rotation from NED frame to vehicle body frame
     * @note This overload is more efficient than Euler version (no trig functions)
     * 
     * @warning Non-normalized quaternions may cause attitude estimation errors
     * 
     * @see handle_pose_estimate(Euler) for Euler angle variant
     * @see libraries/AP_Math/quaternion.h for Quaternion implementation
     */
    void handle_pose_estimate(uint64_t remote_time_us, uint32_t time_ms, float x, float y, float z, const Quaternion &attitude, float posErr, float angErr, uint8_t reset_counter, int8_t quality);
    
    /**
     * @brief Process visual odometry velocity estimate
     * 
     * @param[in] remote_time_us Sensor timestamp in microseconds (sensor's local time)
     * @param[in] time_ms System time in milliseconds when measurement was received
     * @param[in] vel Velocity vector in NED frame (m/s): [North, East, Down]
     * @param[in] reset_counter Sensor reset counter (increments on tracking reinitialization)
     * @param[in] quality Quality metric: -1 = failed, 0 = unknown, 1-100 = quality percentage
     * 
     * @details Processes velocity estimates from visual odometry sensors and forwards them
     *          to the EKF for sensor fusion. Velocity estimates are used independently of
     *          position estimates and can improve navigation accuracy, especially during
     *          rapid maneuvers.
     * 
     *          Processing steps:
     *          1. Check quality against _QUALITY_MIN threshold
     *          2. Validate velocity magnitude is reasonable
     *          3. Forward to EKF with _VEL_NOISE parameter for fusion weighting
     * 
     *          NED frame velocity convention:
     *          - vel.x: Velocity North in meters/second (positive = northward)
     *          - vel.y: Velocity East in meters/second (positive = eastward)
     *          - vel.z: Velocity Down in meters/second (positive = downward)
     * 
     * @note Velocity must be in NED (North-East-Down) frame, not body frame
     * @note Measurements below _QUALITY_MIN threshold are silently discarded
     * @note reset_counter changes indicate sensor reinitialization
     * 
     * @warning Incorrect frame (e.g., body frame instead of NED) causes navigation errors
     * @warning Velocity units must be m/s, not cm/s or other units
     * 
     * @see get_vel_noise() for velocity measurement noise configuration
     * @see libraries/AP_Math/vector3.h for Vector3f details
     */
    void handle_vision_speed_estimate(uint64_t remote_time_us, uint32_t time_ms, const Vector3f &vel, uint8_t reset_counter, int8_t quality);

    /**
     * @brief Request visual odometry sensor yaw alignment with AHRS/EKF attitude
     * 
     * @details Sends a request to the visual odometry backend to reset its yaw (heading)
     *          estimate to match the vehicle's current AHRS/EKF attitude. This is useful
     *          for initializing visual odometry in a known orientation or recovering from
     *          yaw drift.
     * 
     *          Backend-specific behavior:
     *          - T265: Resets internal VIO yaw to match AHRS
     *          - MAVLink: Sends alignment request via MAVLink (if supported by sensor)
     *          - VOXL: Platform-specific yaw reset mechanism
     * 
     * @note Not all backends support yaw alignment - method may be ignored
     * @note Yaw alignment is typically performed during pre-arm initialization
     * @note Does not affect position offset - use align_position_to_ahrs() for position
     * 
     * @see align_position_to_ahrs() for position alignment
     * @see libraries/AP_AHRS/AP_AHRS.h for AHRS reference attitude
     */
    void request_align_yaw_to_ahrs();

    /**
     * @brief Align visual odometry position offset to match AHRS position estimate
     * 
     * @param[in] align_xy If true, align horizontal (X-Y) position offset to AHRS
     * @param[in] align_z If true, align vertical (Z/altitude) position offset to AHRS
     * 
     * @details Updates the internal position offset to synchronize the visual odometry
     *          position estimate with the vehicle's AHRS position. This is used when
     *          visual odometry is NOT the primary position source but needs to track
     *          the vehicle's estimated position for seamless source switching.
     * 
     *          Typical usage scenarios:
     *          - GPS is primary position source, VIO is backup
     *          - Transitioning from GPS to VIO navigation
     *          - Initializing VIO position at vehicle startup
     * 
     * @warning Should ONLY be called when visual odometry is not the active position source.
     *          Calling while VIO is the primary source causes position jumps and filter resets.
     * 
     * @note Alignment modifies internal offset, not sensor configuration parameters
     * @note Separate X-Y and Z alignment allows altitude-only or horizontal-only synchronization
     * 
     * @see request_align_yaw_to_ahrs() for yaw alignment
     * @see libraries/AP_AHRS/AP_AHRS.h for AHRS position estimate
     */
    void align_position_to_ahrs(bool align_xy, bool align_z);

    /**
     * @brief Perform pre-arm safety checks for visual odometry system
     * 
     * @param[out] failure_msg Buffer to store failure message string if check fails
     * @param[in] failure_msg_len Length of failure_msg buffer in bytes
     * 
     * @return true if all pre-arm checks pass and vehicle is safe to arm
     * @return false if any check fails, with reason written to failure_msg
     * 
     * @details Validates visual odometry configuration and sensor health before allowing
     *          vehicle arming. Prevents arming with misconfigured or unhealthy VIO that
     *          could cause navigation failures and crashes.
     * 
     *          Checks performed:
     *          - Visual odometry is enabled (_TYPE != None)
     *          - Backend is initialized and responding
     *          - Sensor is healthy (data received within timeout)
     *          - Quality is above minimum threshold (if configured)
     *          - Position offset is reasonable (not extreme values)
     *          - Orientation configuration is valid
     * 
     *          Example failure messages:
     *          - "VisualOdom: not healthy"
     *          - "VisualOdom: quality too low"
     *          - "VisualOdom: bad position offset"
     * 
     * @note Called by vehicle arming system before allowing motor start
     * @note failure_msg is only written on failure (return false)
     * @note Empty checks if visual odometry is disabled - always returns true
     * 
     * @warning Do not bypass pre-arm checks - they prevent dangerous flight conditions
     * 
     * @see healthy() for sensor health status
     * @see quality() for current quality metric
     * @see libraries/AP_Arming/AP_Arming.h for arming system integration
     */
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const;

    /**
     * @brief Parameter table defining all configurable visual odometry parameters
     * 
     * @details Defines the parameter group for visual odometry configuration accessible
     *          via ground control stations and parameter files. Parameters include:
     * 
     *          - _TYPE: Backend selection (VisualOdom_Type enum)
     *          - _ORIENT: Camera mounting orientation (Rotation enum)
     *          - _POS_OFFSET: Camera position offset [X, Y, Z] in meters
     *          - _POS_SCALE: Position scaling factor (dimensionless)
     *          - _DELAY_MS: Measurement delay in milliseconds
     *          - _VEL_NOISE: Velocity measurement noise in m/s
     *          - _POS_NOISE: Position measurement noise in meters
     *          - _YAW_NOISE: Yaw measurement noise in radians
     *          - _QUALITY_MIN: Minimum quality threshold (0-100)
     * 
     * @note Used by AP_Param system for parameter storage and serialization
     * @see libraries/AP_Param/AP_Param.h for parameter system details
     */
    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @brief Get the configured visual odometry backend type
     * 
     * @return Currently configured VisualOdom_Type (None, MAV, IntelT265, or VOXL)
     * 
     * @details Returns the backend type selected via the _TYPE parameter. This indicates
     *          which visual odometry implementation is active (or None if disabled).
     * 
     * @note Backend type is set via parameter and persists across reboots
     * @see VisualOdom_Type enum for available backend types
     */
    VisualOdom_Type get_type(void) const {
        return _type;
    }

private:

    static AP_VisualOdom *_singleton;

    // parameters
    AP_Enum<VisualOdom_Type> _type; // sensor type
    AP_Vector3f _pos_offset;    // position offset of the camera in the body frame
    AP_Int8 _orientation;       // camera orientation on vehicle frame
    AP_Float _pos_scale;        // position scale factor applied to sensor values
    AP_Int16 _delay_ms;         // average delay relative to inertial measurements
    AP_Float _vel_noise;        // velocity measurement noise in m/s
    AP_Float _pos_noise;        // position measurement noise in meters
    AP_Float _yaw_noise;        // yaw measurement noise in radians
    AP_Int8 _quality_min;       // positions and velocities will only be sent to EKF if over this value.  if 0 all values sent to EKF

    // reference to backends
    AP_VisualOdom_Backend *_driver;
};

/**
 * @namespace AP
 * @brief ArduPilot global namespace for singleton accessors
 */
namespace AP {
    /**
     * @brief Access the global AP_VisualOdom singleton instance
     * 
     * @return Pointer to the AP_VisualOdom singleton, or nullptr if not instantiated
     * 
     * @details Provides convenient namespace-based access to the visual odometry manager.
     *          Preferred over AP_VisualOdom::get_singleton() for cleaner code.
     * 
     * @note Usage: `AP::visualodom()->healthy()`
     * 
     * @see AP_VisualOdom::get_singleton() for direct singleton accessor
     */
    AP_VisualOdom *visualodom();
};

#endif // HAL_VISUALODOM_ENABLED
