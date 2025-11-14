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

/**
 * @file AP_VisualOdom_Backend.h
 * @brief Abstract backend interface for visual odometry sensors
 * 
 * This file defines the base class for visual odometry sensor backends in ArduPilot.
 * Visual odometry provides position and velocity estimates by tracking visual features
 * or using camera-based positioning systems (e.g., Intel RealSense T265, VOXL camera).
 * Backends implement this interface to integrate specific sensors with the EKF and AHRS.
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_VisualOdom.h"

#if HAL_VISUALODOM_ENABLED

#include <AP_Logger/AP_Logger_config.h>

/**
 * @class AP_VisualOdom_Backend
 * @brief Abstract base class for visual odometry sensor backends
 * 
 * @details This class defines the interface that all visual odometry sensor backends
 *          must implement. It provides:
 *          - Health monitoring based on data freshness (300ms timeout)
 *          - Quality reporting on -1 to 100 scale
 *          - Integration with EKF for position and velocity estimates
 *          - Reset counter handling to detect sensor resets
 *          - Optional yaw and position alignment with AHRS
 *          - MAVLink message handling for VISION_POSITION_DELTA
 *          - Data logging when enabled
 * 
 *          Backends must implement pure virtual methods for handling pose estimates
 *          and velocity estimates, forwarding the data to the EKF for sensor fusion.
 * 
 * @note Derived classes should update _last_update_ms on each data reception to
 *       maintain healthy() status and update _quality with sensor-reported quality.
 */
class AP_VisualOdom_Backend
{
public:
    /**
     * @brief Constructor for visual odometry backend
     * 
     * @details Constructs the backend and binds it to the frontend instance.
     *          This incorporates initialization of the backend and establishes
     *          the reference to the frontend for accessing configuration parameters
     *          and forwarding data to the EKF.
     * 
     * @param[in] frontend Reference to the AP_VisualOdom frontend instance that
     *                     manages this backend and provides access to parameters
     *                     and system integration points
     */
    AP_VisualOdom_Backend(AP_VisualOdom &frontend);

    /**
     * @brief Check if sensor is healthy and providing recent data
     * 
     * @details Returns true if the backend has received data within the timeout period
     *          defined by AP_VISUALODOM_TIMEOUT_MS (300ms). This indicates the sensor
     *          is actively providing measurements. If no data is received for 300ms,
     *          the sensor is considered unhealthy and its data will not be used by the EKF.
     * 
     * @return true if data received within last 300ms, false if stale or no data
     * 
     * @note Backends must update _last_update_ms on each data reception to maintain
     *       healthy status
     */
    bool healthy() const;

    /**
     * @brief Get current quality estimate from visual odometry sensor
     * 
     * @details Quality indicates the sensor's confidence in its measurements.
     *          The scale is standardized across all visual odometry backends:
     *          - -1: Sensor has failed or lost tracking completely
     *          - 0: Quality unknown (sensor does not report quality)
     *          - 1-100: Quality scale where 1 is worst tracking, 100 is best tracking
     * 
     * @return Quality value from -1 (failed) to 100 (best), or 0 if unknown
     */
    int8_t quality() const { return _quality; }

#if HAL_GCS_ENABLED
    /**
     * @brief Process MAVLink VISION_POSITION_DELTA message
     * 
     * @details Handles incoming VISION_POSITION_DELTA MAVLink messages containing
     *          body-frame odometry data (position and angle deltas since last measurement).
     *          This method unpacks the message, validates the data, and forwards it to
     *          the AHRS for integration into the navigation solution. Body-frame deltas
     *          are useful for dead-reckoning when absolute position is not available.
     * 
     * @param[in] msg MAVLink message containing vision position delta data in body frame
     * 
     * @note This method is only available when HAL_GCS_ENABLED is defined
     */
    void handle_vision_position_delta_msg(const mavlink_message_t &msg);
#endif

    /**
     * @brief Process vision pose estimate and forward to EKF
     * 
     * @details Pure virtual method that backends must implement to handle incoming
     *          pose (position + attitude) estimates from the visual odometry sensor.
     *          The backend should validate the data and forward it to the EKF through
     *          the AHRS interface for fusion with other sensor data.
     * 
     * @param[in] remote_time_us Timestamp from the sensor in microseconds (sensor's timebase)
     * @param[in] time_ms System time in milliseconds when data was received
     * @param[in] x Position X in NED frame, meters
     * @param[in] y Position Y in NED frame, meters
     * @param[in] z Position Z in NED frame, meters (positive down)
     * @param[in] attitude Vehicle attitude as quaternion
     * @param[in] posErr Position estimate error/uncertainty in meters (1-sigma)
     * @param[in] angErr Attitude estimate error/uncertainty in radians (1-sigma)
     * @param[in] reset_counter Increments each time the sensor resets its origin or loses tracking
     * @param[in] quality Quality metric: -1=failed, 0=unknown, 1=worst, 100=best
     * 
     * @note Distances are in meters, attitude angles in radians
     * @note Backends should update _last_update_ms and _quality when this is called
     */
    virtual void handle_pose_estimate(uint64_t remote_time_us, uint32_t time_ms, float x, float y, float z, const Quaternion &attitude, float posErr, float angErr, uint8_t reset_counter, int8_t quality) = 0;

    /**
     * @brief Process vision velocity estimate and forward to EKF
     * 
     * @details Pure virtual method that backends must implement to handle incoming
     *          velocity estimates from the visual odometry sensor. The backend should
     *          validate the data and forward it to the EKF through the AHRS interface
     *          for fusion with other velocity sources (GPS, IMU integration).
     * 
     * @param[in] remote_time_us Timestamp from the sensor in microseconds (sensor's timebase)
     * @param[in] time_ms System time in milliseconds when data was received
     * @param[in] vel Velocity vector in NED frame, m/s (North, East, Down components)
     * @param[in] reset_counter Increments each time the sensor resets its velocity estimate
     * @param[in] quality Quality metric: -1=failed, 0=unknown, 1=worst, 100=best
     * 
     * @note Velocities are in NED frame in meters per second
     * @note Backends should update _last_update_ms and _quality when this is called
     */
    virtual void handle_vision_speed_estimate(uint64_t remote_time_us, uint32_t time_ms, const Vector3f &vel, uint8_t reset_counter, int8_t quality) = 0;

    /**
     * @brief Request sensor to align its yaw estimate with vehicle's AHRS/EKF
     * 
     * @details Virtual method that backends can implement to align the visual odometry
     *          sensor's yaw angle with the vehicle's current AHRS/EKF attitude estimate.
     *          This is useful when the sensor provides relative yaw but absolute heading
     *          is needed, or to recover from yaw drift. The default implementation does
     *          nothing, as not all sensors support yaw realignment.
     * 
     * @note Only sensors with yaw realignment capability need to override this method
     * @note This is typically called when switching modes or recovering from poor yaw estimates
     */
    virtual void request_align_yaw_to_ahrs() {}

    /**
     * @brief Align sensor's position estimate with vehicle's AHRS position
     * 
     * @details Virtual method that backends can implement to align the visual odometry
     *          sensor's position estimate with the vehicle's current AHRS position.
     *          This allows selective alignment of horizontal (XY) and/or vertical (Z)
     *          position components, useful for recovering from drift or initializing
     *          the sensor's position reference. Default implementation does nothing.
     * 
     * @param[in] align_xy If true, align horizontal position (North/East components)
     * @param[in] align_z If true, align vertical position (Down component)
     * 
     * @note Only sensors supporting position realignment need to override this method
     */
    virtual void align_position_to_ahrs(bool align_xy, bool align_z) {}

    /**
     * @brief Pre-arm safety check for visual odometry sensor
     * 
     * @details Virtual method for backends to implement sensor-specific pre-arm checks
     *          before the vehicle is allowed to arm. Backends can verify sensor health,
     *          quality, calibration status, or any other safety requirements. The default
     *          implementation returns true (no checks), which is appropriate for sensors
     *          that are not required for flight.
     * 
     * @param[out] failure_msg Buffer to write failure message if check fails
     * @param[in] failure_msg_len Maximum length of failure message buffer
     * 
     * @return true if pre-arm check passes, false if check fails (prevents arming)
     * 
     * @note Backends requiring visual odometry for flight should override this to
     *       verify sensor health and data quality before allowing arming
     */
    virtual bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const { return true; }

protected:

    /**
     * @brief Get or update timestamp of last sensor reset
     * 
     * @details Manages reset counter tracking to detect when the visual odometry sensor
     *          resets its coordinate frame or loses tracking. When the reset_counter
     *          changes from the last known value, this method updates the internal
     *          reset timestamp to the current system time and returns it. If the
     *          reset_counter has not changed, it returns the timestamp of the last reset.
     *          
     *          This is used by the EKF to properly handle discontinuities in position
     *          estimates when the sensor resets, preventing the EKF from treating the
     *          jump as valid motion.
     * 
     * @param[in] reset_counter Current reset counter from the sensor
     * 
     * @return System time in milliseconds of the last detected reset (when reset_counter changed)
     * 
     * @note Backends should call this method when processing pose/velocity estimates to
     *       maintain proper reset tracking
     * @note The reset timestamp is passed to the EKF along with position data
     */
    uint32_t get_reset_timestamp_ms(uint8_t reset_counter);

    /**
     * @brief Get the configured visual odometry sensor type
     * 
     * @details Helper method to retrieve the sensor type configuration from the frontend.
     *          Backends can use this to adjust behavior based on sensor type if needed.
     * 
     * @return The configured VisualOdom_Type from frontend parameters
     */
    AP_VisualOdom::VisualOdom_Type get_type(void) const {
        return _frontend.get_type();
    }

#if HAL_LOGGING_ENABLED
    /**
     * @brief Log visual odometry delta measurements to dataflash
     * 
     * @details Writes body-frame position and angle deltas to the dataflash log for
     *          post-flight analysis. This is typically used when processing
     *          VISION_POSITION_DELTA messages.
     * 
     * @param[in] time_delta Time elapsed since last measurement in seconds
     * @param[in] angle_delta Body-frame angle change (roll, pitch, yaw) in radians
     * @param[in] position_delta Body-frame position change (forward, right, down) in meters
     * @param[in] confidence Measurement confidence/quality (sensor-specific scale)
     * 
     * @note Only available when HAL_LOGGING_ENABLED is defined
     */
    void Write_VisualOdom(float time_delta, const Vector3f &angle_delta, const Vector3f &position_delta, float confidence);
    
    /**
     * @brief Log visual position estimate to dataflash
     * 
     * @details Writes absolute position and attitude estimates from the visual odometry
     *          sensor to the dataflash log for analysis and EKF tuning. Includes error
     *          estimates, reset counter, and quality metrics.
     * 
     * @param[in] remote_time_us Sensor timestamp in microseconds
     * @param[in] time_ms System time when data received in milliseconds
     * @param[in] x Position X in NED frame, meters
     * @param[in] y Position Y in NED frame, meters
     * @param[in] z Position Z in NED frame, meters (positive down)
     * @param[in] roll Roll angle in radians
     * @param[in] pitch Pitch angle in radians
     * @param[in] yaw Yaw angle in radians
     * @param[in] pos_err Position uncertainty in meters (1-sigma)
     * @param[in] ang_err Attitude uncertainty in radians (1-sigma)
     * @param[in] reset_counter Sensor reset counter value
     * @param[in] ignored True if EKF ignored this measurement
     * @param[in] quality Quality metric: -1=failed, 0=unknown, 1-100=quality scale
     * 
     * @note Only available when HAL_LOGGING_ENABLED is defined
     */
    void Write_VisualPosition(uint64_t remote_time_us, uint32_t time_ms, float x, float y, float z, float roll, float pitch, float yaw, float pos_err, float ang_err, uint8_t reset_counter, bool ignored, int8_t quality);
    
    /**
     * @brief Log visual velocity estimate to dataflash
     * 
     * @details Writes velocity estimates from the visual odometry sensor to the dataflash
     *          log for analysis and EKF tuning. Includes reset counter and quality metrics.
     * 
     * @param[in] remote_time_us Sensor timestamp in microseconds
     * @param[in] time_ms System time when data received in milliseconds
     * @param[in] vel Velocity vector in NED frame, m/s
     * @param[in] reset_counter Sensor reset counter value
     * @param[in] ignored True if EKF ignored this measurement
     * @param[in] quality Quality metric: -1=failed, 0=unknown, 1-100=quality scale
     * 
     * @note Only available when HAL_LOGGING_ENABLED is defined
     */
    void Write_VisualVelocity(uint64_t remote_time_us, uint32_t time_ms, const Vector3f &vel, uint8_t reset_counter, bool ignored, int8_t quality);
#endif

    AP_VisualOdom &_frontend;   ///< Reference to frontend instance for parameter access and EKF integration
    
    uint32_t _last_update_ms;   ///< System time (milliseconds) of last data update from sensor, used for health monitoring (300ms timeout)

    // Reset counter handling - tracks sensor coordinate frame resets
    uint8_t _last_reset_counter;    ///< Last reset counter value received from sensor, increments on sensor reset/tracking loss
    uint32_t _reset_timestamp_ms;   ///< System time (milliseconds) when reset counter last changed, forwarded to EKF

    // Quality tracking
    int8_t _quality;                ///< Last reported quality from sensor: -1=failed, 0=unknown, 1=worst, 100=best
};

#endif  // HAL_VISUALODOM_ENABLED
