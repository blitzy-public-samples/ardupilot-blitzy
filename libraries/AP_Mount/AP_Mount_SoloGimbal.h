/**
 * @file AP_Mount_SoloGimbal.h
 * @brief 3DR Solo gimbal mount backend implementation with EKF-based attitude estimation
 * 
 * @details This file implements the mount backend for the 3DR Solo gimbal, which features:
 *          - Integration with SoloGimbalEKF for high-accuracy attitude estimation
 *          - MAVLink PARAM protocol for gimbal configuration and calibration
 *          - High-rate control updates (update_fast() called at INS rate for smooth tracking)
 *          - Custom MAVLink message handling (GIMBAL_REPORT, GIMBAL_TORQUE_REPORT)
 *          - Binary logging of gimbal state and EKF estimates
 * 
 *          The Solo gimbal uses a complementary filter (EKF) approach to estimate its
 *          attitude independently from the vehicle attitude, providing stable platform
 *          control even during aggressive vehicle maneuvers.
 * 
 * @note Coordinate frames: Gimbal operates in NED earth-frame with body-frame yaw reference
 * @note This backend suppresses normal mount heartbeat messages (uses Solo-specific protocol)
 * 
 * @see SoloGimbal for Solo SDK integration and EKF attitude estimation algorithm
 * @see AP_Mount_Backend for base mount functionality
 */
#pragma once

#include "AP_Mount_config.h"

#if HAL_SOLO_GIMBAL_ENABLED

#include "AP_Mount_Backend.h"
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include "SoloGimbal.h"

/**
 * @class AP_Mount_SoloGimbal
 * @brief Mount backend for 3DR Solo gimbal with EKF-based attitude estimation
 * 
 * @details This class provides the ArduPilot integration for the 3DR Solo gimbal system,
 *          which features an onboard Extended Kalman Filter (EKF) for independent attitude
 *          estimation. Key characteristics of the Solo gimbal implementation:
 * 
 *          **Attitude Estimation:**
 *          - Uses SoloGimbalEKF for high-rate attitude estimation independent of vehicle AHRS
 *          - Fuses IMU data (gyro/accel) with joint encoder feedback
 *          - Provides stable platform control during aggressive maneuvers
 *          - EKF runs at INS sample rate for low-latency response
 * 
 *          **Control Architecture:**
 *          - update_fast() called at INS rate (~400Hz) for smooth gimbal tracking
 *          - update() called at lower rate for MAVLink communication and mode changes
 *          - Supports ROI (Region of Interest) and angle control modes
 *          - Two-axis control: pitch and roll (no independent pan control)
 * 
 *          **MAVLink Protocol:**
 *          - Uses MAVLink PARAM protocol for gimbal configuration (gains, limits, calibration)
 *          - Parameter changes can be written to gimbal flash memory for persistence
 *          - Receives GIMBAL_REPORT messages with attitude and sensor data
 *          - Receives GIMBAL_TORQUE_REPORT for motor load monitoring
 *          - Suppresses standard heartbeat (uses Solo-specific protocol)
 * 
 *          **Calibration and Configuration:**
 *          - Supports gyro bias calibration via parameter commands
 *          - Configurable axis limits and control gains via MAVLink parameters
 *          - Parameters queued and sent to gimbal during initialization
 * 
 *          **Logging:**
 *          - Binary logs include gimbal attitude, rates, control commands
 *          - Logs EKF state for post-flight analysis
 *          - Logs joint angles and motor loads
 * 
 * @note Solo SDK Compatibility: This implementation is designed for 3DR Solo gimbal firmware
 * @note EKF Update Rate: Fast update path runs at INS rate (typically 400Hz on Solo)
 * @note Parameter Flash: Writing parameters to gimbal flash has limited cycle life
 * @note Heartbeat Suppression: Returns true from suppress_heartbeat() to avoid protocol conflicts
 * @note Coordinate Frames: Gimbal attitude in NED earth-frame, angles in body-frame with yaw reference
 * @note Angle Limits: Pitch typically -90° to +30°, roll stabilization only (no commanded roll)
 * 
 * @see SoloGimbal for Solo SDK integration and EKF implementation details
 * @see SoloGimbalEKF for attitude estimation algorithm
 * @see AP_Mount_Backend for base mount interface and common functionality
 */
class AP_Mount_SoloGimbal : public AP_Mount_Backend
{

public:
    /**
     * @brief Constructor for Solo gimbal mount backend
     * 
     * @param[in] frontend Reference to AP_Mount frontend for parameter access
     * @param[in] params Mount parameters specific to this instance
     * @param[in] instance Instance number of this mount (0-based)
     */
    AP_Mount_SoloGimbal(AP_Mount &frontend, AP_Mount_Params &params, uint8_t instance);

    /**
     * @brief Initialize the Solo gimbal mount backend
     * 
     * @details Performs initialization required for Solo gimbal communication:
     *          - Initializes SoloGimbal instance with MAVLink routing information
     *          - Sets up parameter queue for gimbal configuration
     *          - Establishes MAVLink communication channel to gimbal
     *          - Prepares for GIMBAL_REPORT message reception
     * 
     *          This method is called once during mount system initialization.
     * 
     * @note Must be called before update() or update_fast()
     * @note Gimbal parameters are sent during first update cycles after init
     */
    void init() override;

    /**
     * @brief Update mount position and handle MAVLink communication
     * 
     * @details Periodic update function called at scheduler rate (typically 50Hz):
     *          - Sends queued parameter requests/updates to gimbal via MAVLink
     *          - Processes mode changes (angle control, ROI tracking, etc.)
     *          - Updates target angles based on current control mode
     *          - Handles parameter read/write operations with gimbal
     * 
     *          This handles lower-rate communication tasks, while update_fast()
     *          handles high-rate control loop updates.
     * 
     * @note Called at scheduler rate, not INS rate
     * @note Parameter operations may take multiple update cycles to complete
     */
    void update() override;

    /**
     * @brief Query if this mount has independent pan (yaw) control
     * 
     * @details The Solo gimbal is a 2-axis stabilized gimbal with pitch and roll control.
     *          It does not have independent pan (yaw) control - yaw tracking is achieved
     *          by vehicle yaw rotation. This is typical for camera gimbals on multicopters
     *          where the vehicle can yaw to point the camera.
     * 
     * @return false Always returns false - Solo gimbal has no independent pan control
     * 
     * @note Return value affects mount control mode availability
     * @note Vehicle must yaw to change camera heading
     */
    bool has_pan_control() const override { return false; }

    /**
     * @brief Handle GIMBAL_REPORT MAVLink message from Solo gimbal
     * 
     * @details Processes GIMBAL_REPORT messages containing gimbal attitude and sensor data:
     *          - Extracts gimbal attitude (quaternion or Euler angles)
     *          - Receives IMU data (gyro and accelerometer readings)
     *          - Updates EKF with new measurements
     *          - Logs gimbal state for post-flight analysis
     * 
     *          GIMBAL_REPORT is sent by the Solo gimbal at high rate (typically 10-50Hz)
     *          and provides the primary feedback for gimbal state monitoring.
     * 
     * @param[in] chan MAVLink channel the message was received on
     * @param[in] msg MAVLink message containing GIMBAL_REPORT data
     * 
     * @note This is the primary attitude feedback from the gimbal
     * @note Message rate affects EKF update rate and attitude accuracy
     */
    void handle_gimbal_report(mavlink_channel_t chan, const mavlink_message_t &msg) override;

    /**
     * @brief Handle GIMBAL_TORQUE_REPORT MAVLink message from Solo gimbal
     * 
     * @details Processes GIMBAL_TORQUE_REPORT messages containing motor load information:
     *          - Receives motor torque/current values for each axis
     *          - Monitors motor saturation and load conditions
     *          - Can be used to detect mechanical binding or excessive load
     *          - Logged for performance analysis
     * 
     * @param[in] chan MAVLink channel the message was received on
     * @param[in] msg MAVLink message containing GIMBAL_TORQUE_REPORT data
     * 
     * @note High torque values may indicate mechanical issues or tuning problems
     * @note Not all Solo gimbal firmware versions send this message
     */
    void handle_gimbal_torque_report(mavlink_channel_t chan, const mavlink_message_t &msg);

    /**
     * @brief Handle PARAM_VALUE MAVLink message from Solo gimbal
     * 
     * @details Processes parameter read responses from the gimbal:
     *          - Receives current parameter values from gimbal
     *          - Validates parameter read/write operations
     *          - Manages parameter queue for configuration updates
     *          - Handles parameter flash write confirmation
     * 
     *          The Solo gimbal uses standard MAVLink parameter protocol for configuration
     *          (gains, limits, calibration offsets, etc.). This method processes the
     *          gimbal's responses to parameter requests.
     * 
     * @param[in] msg MAVLink message containing PARAM_VALUE data
     * 
     * @note Parameter flash writes have limited cycle life (typically 10,000+ cycles)
     * @note Parameter operations may require multiple message exchanges
     * @see SoloGimbal for parameter queue management
     */
    void handle_param_value(const mavlink_message_t &msg) override;

    /**
     * @brief Fast update for high-rate gimbal control
     * 
     * @details High-rate update function called at INS sample rate (typically 400Hz):
     *          - Updates EKF prediction step with latest IMU data
     *          - Computes gimbal control commands based on current mode
     *          - Sends rate or angle commands to gimbal controller
     *          - Provides smooth, low-latency tracking response
     * 
     *          This method runs in the fast loop to provide responsive gimbal control
     *          with minimal latency between IMU updates and gimbal commands. The high
     *          update rate is critical for stable tracking during vehicle maneuvers.
     * 
     * @note Called at INS rate (typically 400Hz on Solo)
     * @note Runs in fast loop - keep execution time minimal
     * @note EKF prediction step runs at this rate for best performance
     * @warning Fast loop timing is critical - excessive computation here affects flight performance
     */
    void update_fast() override;

protected:

    /**
     * @brief Query if standard mount heartbeat should be suppressed
     * 
     * @details The Solo gimbal uses its own MAVLink protocol messages (GIMBAL_REPORT,
     *          GIMBAL_TORQUE_REPORT) for communication instead of the standard mount
     *          heartbeat and status messages. Suppressing the heartbeat prevents
     *          protocol conflicts and reduces unnecessary MAVLink traffic.
     * 
     * @return true Always returns true - Solo gimbal uses custom protocol
     * 
     * @note Solo gimbal presence is detected via GIMBAL_REPORT messages
     * @note Standard mount heartbeat would conflict with Solo protocol
     */
    bool suppress_heartbeat() const override { return true; }

    /**
     * @brief Get current gimbal attitude as a quaternion from EKF estimate
     * 
     * @details Retrieves the current gimbal attitude from the SoloGimbalEKF estimator.
     *          The attitude is computed by fusing:
     *          - High-rate IMU data (gyro and accelerometer)
     *          - Joint encoder feedback from gimbal motors
     *          - Attitude reports from gimbal controller
     * 
     *          The EKF provides a smooth, low-latency attitude estimate that is more
     *          accurate than raw sensor data alone, especially during vehicle maneuvers.
     * 
     * @param[out] att_quat Quaternion to receive gimbal attitude (NED earth-frame)
     * 
     * @return true if attitude is valid and was successfully retrieved
     * @return false if EKF has not converged or gimbal is not initialized
     * 
     * @note Attitude is in NED earth-frame (North-East-Down)
     * @note Quaternion represents rotation from earth-frame to gimbal body-frame
     * @note Returns false until gimbal initializes and EKF converges (typically 1-2 seconds)
     * @see SoloGimbalEKF for attitude estimation algorithm details
     */
    bool get_attitude_quaternion(Quaternion& att_quat) override;

private:
    /**
     * @brief Initialization state flag
     * 
     * Set to true once init() has been called and the gimbal backend is ready
     * for operation. Used to prevent operations before initialization is complete.
     */
    bool _initialised;

    /**
     * @brief Write gimbal measurement and estimation data to binary log
     * 
     * @details Logs comprehensive gimbal state for post-flight analysis:
     *          - Gimbal attitude (quaternion and Euler angles)
     *          - Gimbal angular rates from gyros
     *          - Joint angles for each axis
     *          - Control commands sent to gimbal
     *          - EKF state estimates and covariances
     *          - Motor loads and torques
     * 
     *          This data is critical for tuning gimbal performance, analyzing
     *          vibration issues, and debugging gimbal behavior.
     * 
     * @param[in] gimbal Reference to SoloGimbal instance containing state to log
     * 
     * @note Called from update() at scheduler rate (typically 50Hz)
     * @note Log messages use GIMBAL message type in dataflash logs
     */
    void Log_Write_Gimbal(SoloGimbal &gimbal);

    /**
     * @brief Parameter save state flag
     * 
     * Tracks whether gimbal parameters have been saved to flash memory.
     * Used to prevent repeated flash writes which have limited cycle life.
     * Set to true after successful parameter save operation.
     * 
     * @note Flash write operations should be minimized to preserve flash life
     * @note Typically written once per configuration change, not every boot
     */
    bool _params_saved;

    /**
     * @brief Solo gimbal SDK integration object
     * 
     * Main interface to Solo gimbal functionality including:
     * - SoloGimbalEKF for attitude estimation
     * - MAVLink message handling and routing
     * - Parameter queue management
     * - Control command generation
     * - Joint angle and motor state management
     * 
     * This object encapsulates the Solo-specific gimbal protocol and EKF
     * implementation, providing a clean interface for the mount backend.
     * 
     * @see SoloGimbal class for detailed implementation
     * @see SoloGimbalEKF for attitude estimation algorithm
     */
    SoloGimbal _gimbal;
};

#endif // HAL_SOLO_GIMBAL_ENABLED
