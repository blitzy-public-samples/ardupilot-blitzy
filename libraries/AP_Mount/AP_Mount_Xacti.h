/**
 * @file AP_Mount_Xacti.h
 * @brief Xacti DroneCAN-based gimbal backend driver
 * 
 * @details This file implements gimbal control and attitude feedback for Xacti
 *          cameras using DroneCAN/UAVCAN protocol with custom com.xacti DSDL messages.
 *          
 *          The driver provides:
 *          - DroneCAN node discovery and registration
 *          - Gimbal attitude control (angle and rate modes)
 *          - Camera control (photo, video recording, zoom, lens selection)
 *          - Real-time attitude feedback via DroneCAN messages
 *          - Camera parameter management over DroneCAN parameter protocol
 *          - MAVLink CAMERA_INFORMATION and CAMERA_SETTINGS message generation
 *          - Time synchronization with the gimbal
 *          - Status monitoring and error reporting
 *          
 *          Protocol: Uses DroneCAN/DSDL/com/xacti custom messages
 *          Reference: https://github.com/dronecan/DSDL/tree/master/com/xacti
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_XACTI_ENABLED

#include "AP_Mount_Backend.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_HAL/utility/RingBuffer.h>
#include "AP_Mount.h"

/**
 * @class AP_Mount_Xacti
 * @brief Xacti gimbal backend driver using DroneCAN/UAVCAN protocol
 * 
 * @details This class implements a complete gimbal and camera control interface for Xacti
 *          devices over DroneCAN. It handles bidirectional communication including:
 *          
 *          **DroneCAN Integration**:
 *          - Subscribes to com.xacti.GimbalAttitudeStatus for real-time gimbal attitude
 *          - Subscribes to com.xacti.GnssStatusReq for GPS information requests
 *          - Uses DroneCAN registry to discover and map gimbal node IDs
 *          - Implements parameter protocol for camera configuration and capability discovery
 *          
 *          **Gimbal Control**:
 *          - Sends gimbal control commands via send_gimbal_control() over DroneCAN
 *          - Supports both angle control (mode 2) and rate control (mode 3)
 *          - Sends vehicle attitude via send_copter_att_status() for stabilization
 *          - Control rates: gimbal commands at 5Hz, vehicle attitude at 10Hz
 *          
 *          **Camera Operations**:
 *          - Photo capture (single shot parameter)
 *          - Video recording start/stop
 *          - Zoom control (digital and optical, rate and absolute)
 *          - Focus control (auto, manual, rate-based)
 *          - Lens/sensor selection (RGB, IR, PIP, NDVI for multi-sensor gimbals)
 *          - Camera source switching (primary/secondary)
 *          
 *          **Parameter Management**:
 *          - Parameter queueing system to throttle set-parameter requests
 *          - Handles int32 and string parameter types
 *          - Parameter get/set with response callbacks
 *          - Parameter save to persistent storage
 *          
 *          **Capability Discovery**:
 *          - Requests firmware version on initialization
 *          - Queries camera parameters to determine optical zoom capability
 *          - Timeout-based capability detection (falls back to defaults)
 *          
 *          **Time Synchronization**:
 *          - Sets date/time on gimbal from vehicle system time
 *          - Required for proper photo/video timestamping
 *          
 *          **Status Monitoring**:
 *          - Periodic status requests to monitor camera state
 *          - Error detection (motor errors, lens errors, media errors, temperature warnings)
 *          - SD card capacity monitoring
 *          - Health reporting based on motor/control/camera errors
 *          
 *          **MAVLink Integration**:
 *          - Generates CAMERA_INFORMATION messages with capabilities
 *          - Generates CAMERA_SETTINGS messages with current camera state
 *          - Reports firmware version, zoom capabilities, sensor modes
 *          
 * @note DroneCAN node discovery: Backend registers itself and waits for gimbal messages
 *       to identify the node ID. Node ID 0 indicates discovery in progress.
 * 
 * @note Message rates: To prevent overwhelming the gimbal, messages are rate-limited:
 *       - Gimbal control: max 5Hz (200ms minimum interval)
 *       - Vehicle attitude: max 10Hz (100ms minimum interval)
 *       - Parameter get/set: throttled with safety delays
 * 
 * @note Parameter queueing: Set-parameter requests are queued and sent sequentially
 *       to improve reliability, as the camera may not respond if overwhelmed.
 * 
 * @note Units: All angles sent over DroneCAN are in radians, rates in rad/s.
 *       Internal ArduPilot representations may use degrees/centidegrees.
 * 
 * @warning DroneCAN node discovery timeout: If no GimbalAttitudeStatus messages
 *          are received within the capability discovery timeout, the backend may
 *          not be fully functional. Ensure gimbal is powered and connected.
 * 
 * @warning Parameter queue limits: The parameter queue has finite capacity. Rapidly
 *          changing multiple parameters may overflow the queue and lose requests.
 * 
 * @warning Firmware version compatibility: Some features may require specific firmware
 *          versions. The driver queries firmware version but does not enforce minimums.
 *          Incompatible firmware may result in unexpected behavior.
 * 
 * @see AP_DroneCAN for DroneCAN protocol implementation details
 * @see AP_Mount_Backend for base class interface
 * @see https://github.com/dronecan/DSDL/tree/master/com/xacti for message definitions
 */
class AP_Mount_Xacti : public AP_Mount_Backend
{

public:
    /**
     * @brief Constructor for Xacti gimbal backend
     * 
     * @param[in] frontend Reference to AP_Mount frontend manager
     * @param[in] params Reference to mount parameters for this instance
     * @param[in] instance Instance number of this mount (0-based)
     */
    AP_Mount_Xacti(class AP_Mount &frontend, class AP_Mount_Params &params, uint8_t instance);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Mount_Xacti);

    /**
     * @brief Initialize the Xacti gimbal backend
     * 
     * @details Performs initialization including:
     *          - Registering backend in detected modules array for DroneCAN mapping
     *          - Initializing parameter queue for set-parameter requests
     *          - Preparing for DroneCAN node discovery
     *          
     *          After init(), the backend waits for DroneCAN messages to identify
     *          the gimbal node ID through message handlers.
     * 
     * @note Called once during system startup by AP_Mount frontend
     */
    void init() override;

    /**
     * @brief Update mount position and camera state - called periodically at main loop rate
     * 
     * @details Handles periodic tasks including:
     *          - Sending target attitude or rate commands to gimbal
     *          - Sending vehicle attitude for gimbal stabilization
     *          - Processing parameter queue for camera configuration
     *          - Requesting firmware version (on first update after init)
     *          - Requesting camera capabilities (to determine zoom support)
     *          - Setting date/time on gimbal (once after discovery)
     *          - Requesting status updates for monitoring
     *          - Updating zoom rate control if active
     *          
     *          All message sends are rate-limited to prevent overwhelming the gimbal.
     * 
     * @note Called at main loop rate (typically 50-400Hz depending on vehicle)
     * @note Message rate limiting ensures gimbal is not overwhelmed with commands
     */
    void update() override;

    /**
     * @brief Check if gimbal is healthy and operational
     * 
     * @details Returns true if:
     *          - Backend has been initialized
     *          - No motor initialization errors detected
     *          - No motor operation errors detected
     *          - No gimbal control errors detected
     *          
     *          Health status is based on error bits in status messages from gimbal.
     * 
     * @return true if gimbal is healthy and operational, false otherwise
     * 
     * @note Health check does not verify DroneCAN connectivity, only gimbal internal state
     */
    bool healthy() const override;

    /**
     * @brief Check if this mount can control yaw/pan angle
     * 
     * @details Yaw control capability depends on whether the mount has a valid
     *          yaw range configured in parameters. Required for multirotors to
     *          enable heading-relative gimbal control.
     * 
     * @return true if yaw control is available, false otherwise
     * 
     * @see yaw_range_valid() in AP_Mount_Backend base class
     */
    bool has_pan_control() const override { return yaw_range_valid(); };

    //
    // camera controls
    //

    /**
     * @brief Trigger camera to take a single photo
     * 
     * @details Sends a SingleShot parameter set command to the camera over DroneCAN.
     *          The command is queued and sent when safe to avoid overwhelming the camera.
     *          
     *          The camera must not be recording video and must have available storage.
     *          Status is checked via error_status field in status messages.
     * 
     * @return true if photo trigger command was queued successfully, false on failure
     * 
     * @note Actual photo capture is asynchronous - this only queues the command
     * @note Check error_status for TAKING_PICTURE or CANNOT_TAKE_PIC flags
     */
    bool take_picture() override;

    /**
     * @brief Start or stop video recording
     * 
     * @details Sends a Recording parameter set command to the camera over DroneCAN.
     *          The command is queued and sent when safe to avoid overwhelming the camera.
     *          
     *          Recording status is tracked in _recording_video member variable and
     *          reflected in status messages (RECORDING_VIDEO error_status bit).
     * 
     * @param[in] start_recording true to start recording, false to stop recording
     * 
     * @return true if recording command was queued successfully, false on failure
     * 
     * @note Actual recording state change is asynchronous - this only queues the command
     * @note Check error_status for RECORDING_VIDEO flag to confirm state
     */
    bool record_video(bool start_recording) override;

    /**
     * @brief Set camera zoom level or zoom rate
     * 
     * @details Controls camera zoom with two modes:
     *          - Rate control: Continuous zoom in/out at specified rate
     *          - Percentage/absolute control: Set specific zoom level
     *          
     *          Supports both digital zoom (always available, 1x to 10x) and optical zoom
     *          (if capability detected, 1x to 2.5x). Digital zoom parameter sent as 100-1000
     *          in steps of 100. Optical zoom parameter sent as 100-250 in steps of 10.
     *          
     *          Rate control uses internal _zoom_rate_control state machine updated
     *          periodically in update_zoom_rate_control().
     * 
     * @param[in] zoom_type Type of zoom command (RATE or ABSOLUTE/PCT)
     * @param[in] zoom_value For RATE: -1 (out) to +1 (in), for PCT: 0-100%
     * 
     * @return true if zoom command was queued successfully, false on failure
     * 
     * @note Rate control continues until zoom_value = 0 or another zoom command is sent
     * @note Zoom limits enforced: digital 1x-10x, optical 1x-2.5x (if available)
     */
    bool set_zoom(ZoomType zoom_type, float zoom_value) override;

    /**
     * @brief Set camera focus mode or focus rate
     * 
     * @details Controls camera focus with multiple modes:
     *          - Auto focus: Camera determines focus automatically
     *          - Manual rate: Continuous focus adjustment in/out
     *          - Manual absolute: Set specific focus position (not implemented)
     *          
     *          Focus commands sent via FocusMode parameter over DroneCAN.
     * 
     * @param[in] focus_type Type of focus command (AUTO, RATE, ABSOLUTE, etc.)
     * @param[in] focus_value For RATE: -1 (focus in), 0 (hold), +1 (focus out)
     * 
     * @return SetFocusResult indicating success/failure/unsupported
     * 
     * @note Focus control implementation may vary by gimbal firmware version
     * @note Some focus modes may not be supported - returns UNSUPPORTED if so
     */
    SetFocusResult set_focus(FocusType focus_type, float focus_value) override;

    /**
     * @brief Set camera sensor/lens mode by numeric ID
     * 
     * @details For multi-sensor Xacti gimbals, switches between sensor modes:
     *          - 0: RGB (visible light)
     *          - 1: IR (infrared/thermal)
     *          - 2: PIP (picture-in-picture RGB with IR overlay)
     *          - 3: NDVI (vegetation analysis)
     *          
     *          Sends SensorMode parameter over DroneCAN.
     * 
     * @param[in] lens Sensor mode ID (0-3, see SensorsMode enum)
     * 
     * @return true if sensor mode change was queued successfully, false on failure
     * 
     * @note Available sensor modes depend on gimbal hardware configuration
     * @note Parameter is queued and sent asynchronously
     */
    bool set_lens(uint8_t lens) override;

    /**
     * @brief Set camera source by primary and secondary sensor types
     * 
     * @details Alternative interface to set_lens() using AP_Camera::CameraSource enum
     *          for more semantic sensor selection. Maps camera source types to
     *          Xacti sensor modes.
     *          
     *          Currently only primary_source is used; secondary_source is reserved
     *          for future multi-view or PIP configuration.
     * 
     * @param[in] primary_source Primary sensor (RGB, IR, etc. from AP_Camera::CameraSource)
     * @param[in] secondary_source Secondary sensor (reserved for future use)
     * 
     * @return true if camera source change was queued successfully, false on failure
     * 
     * @note Uses AP_Camera::CameraSource enum values cast to uint8_t
     * @see set_lens() for low-level sensor mode control
     */
    bool set_camera_source(uint8_t primary_source, uint8_t secondary_source) override;

    /**
     * @brief Send CAMERA_INFORMATION MAVLink message to ground control station
     * 
     * @details Generates and sends MAVLink CAMERA_INFORMATION message containing:
     *          - Camera model name and firmware version
     *          - Sensor resolution and capabilities
     *          - Zoom capabilities (digital always, optical if detected)
     *          - Focus capabilities
     *          - Video/photo mode support
     *          
     *          Information is gathered from firmware version, capability discovery,
     *          and hardcoded camera specifications.
     * 
     * @param[in] chan MAVLink channel to send message on
     * 
     * @note Called periodically by AP_Mount frontend when GCS requests camera info
     * @note Firmware version must be received before full info is available
     */
    void send_camera_information(mavlink_channel_t chan) const override;

    /**
     * @brief Send CAMERA_SETTINGS MAVLink message to ground control station
     * 
     * @details Generates and sends MAVLink CAMERA_SETTINGS message containing:
     *          - Current recording state (photo/video/idle)
     *          - Current zoom level (digital and optical)
     *          - Current focus mode
     *          - Current sensor/lens selection
     *          
     *          Settings reflect the last known state from camera status messages
     *          and commanded values.
     * 
     * @param[in] chan MAVLink channel to send message on
     * 
     * @note Called periodically by AP_Mount frontend when GCS requests camera settings
     * @note May reflect commanded state before camera has confirmed the change
     */
    void send_camera_settings(mavlink_channel_t chan) const override;

    /**
     * @brief Subscribe to Xacti-specific DroneCAN messages
     * 
     * @details Registers callbacks for DroneCAN messages used by Xacti gimbals:
     *          - com.xacti.GimbalAttitudeStatus: Real-time gimbal attitude feedback
     *          - com.xacti.GnssStatusReq: GPS status information requests from gimbal
     *          
     *          Must be called during DroneCAN initialization to enable message reception.
     *          Static method allows subscription before backend instances are created.
     * 
     * @param[in] ap_dronecan Pointer to DroneCAN interface to subscribe on
     * 
     * @return true if subscription successful, false on failure
     * 
     * @note Called once per DroneCAN interface during system initialization
     * @note Failure to subscribe will prevent gimbal communication
     */
    static bool subscribe_msgs(AP_DroneCAN* ap_dronecan);

    /**
     * @brief Handle incoming GimbalAttitudeStatus message from Xacti gimbal
     * 
     * @details Static callback invoked when com.xacti.GimbalAttitudeStatus message
     *          is received over DroneCAN. Performs:
     *          - Locates backend instance using DroneCAN interface and source node ID
     *          - Updates backend's current attitude quaternion
     *          - Records timestamp of attitude update
     *          
     *          This message provides real-time gimbal attitude in NED frame as a
     *          quaternion, used for closed-loop control and MAVLink reporting.
     * 
     * @param[in] ap_dronecan Pointer to DroneCAN interface message was received on
     * @param[in] transfer CanardRxTransfer containing message metadata (node ID, timestamp)
     * @param[in] msg Decoded GimbalAttitudeStatus message with gimbal attitude quaternion
     * 
     * @note Static method allows callback registration before backend instantiation
     * @note Node ID from transfer is used to identify which backend instance to update
     * @note Attitude quaternion is in NED (North-East-Down) earth frame
     */
    static void handle_gimbal_attitude_status(AP_DroneCAN* ap_dronecan, const CanardRxTransfer& transfer, const com_xacti_GimbalAttitudeStatus &msg);

    /**
     * @brief Handle incoming GnssStatusReq message from Xacti gimbal
     * 
     * @details Static callback invoked when com.xacti.GnssStatusReq message is
     *          received over DroneCAN. The gimbal requests GPS information which
     *          may be used for geotagging photos/videos.
     *          
     *          Locates appropriate backend and processes the GPS data request.
     * 
     * @param[in] ap_dronecan Pointer to DroneCAN interface message was received on
     * @param[in] transfer CanardRxTransfer containing message metadata (node ID, timestamp)
     * @param[in] msg Decoded GnssStatusReq message with GPS request details
     * 
     * @note Static method allows callback registration before backend instantiation
     * @note Node ID from transfer is used to identify which backend instance to update
     */
    static void handle_gnss_status_req(AP_DroneCAN* ap_dronecan, const CanardRxTransfer& transfer, const com_xacti_GnssStatusReq &msg);

protected:

    /**
     * @brief Get current gimbal attitude as a quaternion
     * 
     * @details Retrieves the most recently received gimbal attitude from
     *          GimbalAttitudeStatus messages. The attitude quaternion represents
     *          gimbal orientation in NED (North-East-Down) earth frame.
     *          
     *          Used by AP_Mount frontend for MAVLink reporting and UI display.
     *          Attitude is updated by handle_gimbal_attitude_status() callback.
     * 
     * @param[out] att_quat Quaternion to populate with current gimbal attitude
     * 
     * @return true if valid attitude available, false if no recent attitude data
     * 
     * @note Returns false if no GimbalAttitudeStatus received since initialization
     * @note Attitude is in NED earth frame, not body frame
     */
    bool get_attitude_quaternion(Quaternion& att_quat) override;

private:

    // send text prefix string to reduce flash cost
    static const char* send_text_prefix;

    /**
     * @brief Sensor mode enumeration for multi-sensor gimbals
     * 
     * @details Defines available sensor/lens modes for Xacti cameras with
     *          multiple sensors. Values correspond to SensorMode parameter.
     */
    enum class SensorsMode : uint8_t {
        RGB = 0,    ///< RGB visible light sensor
        IR = 1,     ///< Infrared/thermal sensor
        PIP = 2,    ///< Picture-in-picture: RGB with IR overlay
        NDVI = 3,   ///< NDVI sensor for vegetation analysis
    };
    /// Array of sensor mode strings for user display (defined in .cpp)
    static const char* sensor_mode_str[];

    /**
     * @brief Send target angular rates to gimbal for rate control
     * 
     * @details Sends rate control command to gimbal via DroneCAN using mode 3
     *          (rate control). Rates are in radians per second.
     *          
     *          Rate commands are rate-limited to maximum 5Hz to avoid
     *          overwhelming the gimbal.
     * 
     * @param[in] pitch_rads Target pitch rate in rad/s (positive = pitch up)
     * @param[in] yaw_rads Target yaw rate in rad/s (positive = yaw right)
     * @param[in] yaw_is_ef true if yaw_rads is earth-frame rate, false for body-frame
     * 
     * @note Calls send_gimbal_control() with mode=3 (rate control)
     * @note Rates are converted from rad/s to centidegrees/s for CAN message
     */
    void send_target_rates(float pitch_rads, float yaw_rads, bool yaw_is_ef);

    /**
     * @brief Send target angles to gimbal for angle control
     * 
     * @details Sends angle control command to gimbal via DroneCAN using mode 2
     *          (angle control). Angles are in radians.
     *          
     *          Angle commands are rate-limited to maximum 5Hz to avoid
     *          overwhelming the gimbal.
     * 
     * @param[in] pitch_rad Target pitch angle in radians (positive = pitch up)
     * @param[in] yaw_rad Target yaw angle in radians (positive = yaw right)
     * @param[in] yaw_is_ef true if yaw_rad is earth-frame angle, false for body-frame
     * 
     * @note Calls send_gimbal_control() with mode=2 (angle control)
     * @note Angles are converted from radians to centidegrees for CAN message
     */
    void send_target_angles(float pitch_rad, float yaw_rad, bool yaw_is_ef);

    /**
     * @brief Register this backend in detected modules array for node ID mapping
     * 
     * @details Adds this backend instance to the static _detected_modules array,
     *          which maps DroneCAN interface and node ID to backend instances.
     *          
     *          Initially registered with node_id=0, indicating node discovery
     *          in progress. Node ID is updated when first message is received.
     *          
     *          Protected by _sem_registry semaphore for thread safety.
     * 
     * @note Called during init()
     * @note Node ID 0 means "not yet discovered"
     */
    void register_backend();

    /**
     * @brief Find backend instance associated with DroneCAN port and node ID
     * 
     * @details Static helper to locate backend instance from DroneCAN message
     *          callback context. Searches _detected_modules array for matching
     *          ap_dronecan interface and node_id.
     *          
     *          Also handles node ID association: if a backend has node_id=0
     *          (not yet discovered), associates it with the node_id from the message.
     *          
     *          Protected by _sem_registry semaphore for thread safety.
     * 
     * @param[in] ap_dronecan DroneCAN interface message was received on
     * @param[in] node_id Source node ID from CAN message
     * 
     * @return Pointer to matching backend instance, or nullptr if not found
     * 
     * @note Node ID 0 backends are automatically associated with first message
     * @note Used by static message handler callbacks to find instance
     */
    static AP_Mount_Xacti* get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id);

    /**
     * @brief DroneCAN parameter handling methods
     * 
     * @details These methods implement the DroneCAN parameter protocol for
     *          getting, setting, and saving camera parameters. Functors are
     *          registered as callbacks for parameter responses.
     */
    
    /// Functor callback for int32 parameter get/set responses
    FUNCTOR_DECLARE(param_int_cb, bool, AP_DroneCAN*, const uint8_t, const char*, int32_t &);
    
    /// Functor callback for string parameter get/set responses
    FUNCTOR_DECLARE(param_string_cb, bool, AP_DroneCAN*, const uint8_t, const char*, AP_DroneCAN::string &);
    
    /// Functor callback for parameter save responses
    FUNCTOR_DECLARE(param_save_cb, void, AP_DroneCAN*, const uint8_t, bool);
    
    /**
     * @brief Handle parameter get/set response for int32 parameters
     * 
     * @details Callback invoked when int32 parameter response received from gimbal.
     *          Processes parameter values for camera configuration and capability
     *          discovery (e.g., zoom magnification, recording status, firmware version).
     * 
     * @param[in] ap_dronecan DroneCAN interface response was received on
     * @param[in] node_id Source node ID that sent the response
     * @param[in] name Parameter name string
     * @param[in,out] value Parameter value (input for set, output for get)
     * 
     * @return true if parameter response was expected and handled, false otherwise
     */
    bool handle_param_get_set_response_int(AP_DroneCAN* ap_dronecan, const uint8_t node_id, const char* name, int32_t &value);
    
    /**
     * @brief Handle parameter get/set response for string parameters
     * 
     * @details Callback invoked when string parameter response received from gimbal.
     *          Processes string parameters like firmware version, date/time, status.
     * 
     * @param[in] ap_dronecan DroneCAN interface response was received on
     * @param[in] node_id Source node ID that sent the response
     * @param[in] name Parameter name string
     * @param[in,out] value Parameter value string (input for set, output for get)
     * 
     * @return true if parameter response was expected and handled, false otherwise
     */
    bool handle_param_get_set_response_string(AP_DroneCAN* ap_dronecan, const uint8_t node_id, const char* name, AP_DroneCAN::string &value);
    
    /**
     * @brief Handle parameter save response from gimbal
     * 
     * @details Callback invoked when parameter save response received. Indicates
     *          whether gimbal successfully saved parameters to persistent storage.
     * 
     * @param[in] ap_dronecan DroneCAN interface response was received on
     * @param[in] node_id Source node ID that sent the response
     * @param[in] success true if save succeeded, false if save failed
     */
    void handle_param_save_response(AP_DroneCAN* ap_dronecan, const uint8_t node_id, bool success);

    /**
     * @brief Camera parameter enumeration
     * 
     * @details Defines known camera parameters accessible via DroneCAN parameter
     *          protocol. Each enum value maps to a parameter name string in
     *          _param_names array (defined in .cpp file).
     *          
     *          If enum is updated, also update _param_names array definition.
     */
    enum class Param : uint8_t {
        SingleShot = 0,              ///< Trigger photo capture (int32: 1=take picture)
        Recording,                   ///< Video recording control (int32: 0=stop, 1=start)
        FocusMode,                   ///< Focus mode (int32: 0=auto, 1=manual)
        SensorMode,                  ///< Sensor/lens selection (int32: 0=RGB, 1=IR, 2=PIP, 3=NDVI)
        DigitalZoomMagnification,    ///< Digital zoom level (int32: 100-1000, steps of 100)
        FirmwareVersion,             ///< Firmware version string (string parameter)
        Status,                      ///< Camera status information (string parameter)
        DateTime,                    ///< Date and time setting (string parameter)
        OpticalZoomMagnification,    ///< Optical zoom level (int32: 100-250, steps of 10)
        LAST = OpticalZoomMagnification, ///< Last parameter in enum (for bounds checking)
    };
    
    /// Array of parameter name strings matching Param enum (defined in .cpp)
    static const char* _param_names[];

    /**
     * @brief Get parameter name string from Param enum value
     * 
     * @details Converts Param enum to parameter name string for DroneCAN
     *          parameter protocol messages. Uses _param_names lookup table.
     * 
     * @param[in] param Parameter enum value
     * 
     * @return Parameter name string, or empty string if param out of range
     * 
     * @note Empty string return should never happen if enum used correctly
     */
    const char* get_param_name_str(Param param) const;

    /**
     * @brief Set int32 parameter on gimbal via DroneCAN
     * 
     * @details Queues a set-parameter request for an int32 parameter. Request
     *          is added to _set_param_int32_queue and sent when safe via
     *          process_set_param_int32_queue().
     *          
     *          Parameters are queued to avoid overwhelming the gimbal with
     *          rapid parameter changes.
     * 
     * @param[in] param Parameter to set (from Param enum)
     * @param[in] param_value New parameter value (int32)
     * 
     * @return true if parameter request was queued successfully, false if queue full
     * 
     * @note Actual parameter set is asynchronous
     */
    bool set_param_int32(Param param, int32_t param_value);
    
    /**
     * @brief Set string parameter on gimbal via DroneCAN
     * 
     * @details Sends a set-parameter request for a string parameter immediately
     *          (not queued like int32 parameters). Used for date/time setting.
     * 
     * @param[in] param Parameter to set (from Param enum)
     * @param[in] param_value New parameter value (string)
     * 
     * @return true if parameter request was sent successfully, false on failure
     * 
     * @note String parameters are sent immediately, not queued
     */
    bool set_param_string(Param param, const AP_DroneCAN::string& param_value);
    
    /**
     * @brief Get string parameter from gimbal via DroneCAN
     * 
     * @details Sends a get-parameter request for a string parameter. Response
     *          is received asynchronously via handle_param_get_set_response_string().
     * 
     * @param[in] param Parameter to get (from Param enum)
     * 
     * @return true if parameter request was sent successfully, false on failure
     * 
     * @note Parameter value is received asynchronously in callback
     */
    bool get_param_string(Param param);

    /**
     * @brief Process queue of int32 set-parameter requests
     * 
     * @details Checks if safe to send parameter messages (rate limiting), then
     *          sends next queued set-parameter request if available. Queue is
     *          processed sequentially to avoid overwhelming the gimbal.
     *          
     *          Called periodically from update() method.
     * 
     * @return true if a set-parameter message was sent, false if queue empty or not safe
     * 
     * @note Rate-limited to prevent gimbal from becoming unresponsive
     * @note Queue is FIFO (first-in-first-out)
     */
    bool process_set_param_int32_queue();

    /**
     * @brief Send gimbal control command via DroneCAN
     * 
     * @details Sends com.xacti.GimbalControl message over DroneCAN to command
     *          gimbal attitude or rates. Two control modes supported:
     *          - Mode 2: Angle control (pitch_cd and yaw_cd are angles)
     *          - Mode 3: Rate control (pitch_cd and yaw_cd are rates)
     *          
     *          Rate-limited to maximum 5Hz (200ms between sends) to avoid
     *          overwhelming the gimbal.
     * 
     * @param[in] mode Control mode: 2=angle control, 3=rate control
     * @param[in] pitch_cd Pitch angle in centidegrees or pitch rate in centideg/s
     * @param[in] yaw_cd Yaw angle in centidegrees or yaw rate in centideg/s
     * 
     * @note Updates last_send_gimbal_control_ms for rate limiting
     * @note Angles/rates are in centidegrees for CAN protocol efficiency
     */
    void send_gimbal_control(uint8_t mode, int16_t pitch_cd, int16_t yaw_cd);

    /**
     * @brief Send vehicle attitude to gimbal for stabilization
     * 
     * @details Sends com.xacti.CopterAttitudeStatus message with vehicle
     *          quaternion attitude to gimbal. Gimbal uses this for stabilization
     *          and to compute body-frame vs earth-frame control commands.
     *          
     *          Rate-limited to maximum 10Hz (100ms between sends).
     * 
     * @param[in] now_ms Current system time in milliseconds
     * 
     * @return true if message was sent, false if rate-limited or not safe to send
     * 
     * @note Vehicle attitude obtained from AP_AHRS singleton
     * @note Updates last_send_copter_att_status_ms for rate limiting
     */
    bool send_copter_att_status(uint32_t now_ms);

    /**
     * @brief Update zoom rate control and send zoom commands
     * 
     * @details If zoom rate control is enabled (_zoom_rate_control.enabled),
     *          periodically sends zoom parameter updates to create continuous
     *          zoom in/out motion. Zoom direction determined by _zoom_rate_control.dir.
     *          
     *          Updates digital or optical zoom parameters depending on current
     *          zoom level and capabilities.
     * 
     * @param[in] now_ms Current system time in milliseconds
     * 
     * @return true if zoom parameter was sent, false if disabled or rate-limited
     * 
     * @note Zoom rate control disabled by setting zoom_value=0 in set_zoom()
     * @note Respects zoom limits: digital 100-1000, optical 100-250
     */
    bool update_zoom_rate_control(uint32_t now_ms);

    /**
     * @brief Request firmware version from gimbal
     * 
     * @details Sends parameter get request for FirmwareVersion parameter.
     *          Response received asynchronously via handle_param_get_set_response_string().
     *          
     *          Only requests once until firmware version is received.
     *          Rate-limited to once per second.
     * 
     * @param[in] now_ms Current system time in milliseconds
     * 
     * @return true if firmware version request was sent, false if already received or rate-limited
     * 
     * @note Firmware version used for MAVLink CAMERA_INFORMATION message
     * @note Updates _firmware_version.last_request_ms
     */
    bool request_firmware_version(uint32_t now_ms);

    /**
     * @brief Request camera capability parameters
     * 
     * @details Queries camera parameters to determine capabilities such as
     *          optical zoom support. Requests OpticalZoomMagnification parameter
     *          to detect if optical zoom is available.
     *          
     *          Capability discovery has timeout - after 10 seconds, assumes
     *          defaults (digital zoom only).
     * 
     * @param[in] now_ms Current system time in milliseconds
     * 
     * @return true if capability parameter request was sent, false if complete or timed out
     * 
     * @note Sets capabilities.received=true when discovery complete or timed out
     * @note Rate-limited to avoid overwhelming gimbal during discovery
     */
    bool request_capabilities(uint32_t now_ms);

    /**
     * @brief Set date and time on gimbal
     * 
     * @details Sends DateTime parameter with current system time to gimbal.
     *          Required for proper photo/video timestamping and geotagging.
     *          
     *          Only sets once per power cycle after node ID is discovered.
     * 
     * @param[in] now_ms Current system time in milliseconds
     * 
     * @return true if date/time was set, false if already set or rate-limited
     * 
     * @note Uses AP::rtc() for current date/time
     * @note Sets _datetime.set=true after successful send
     */
    bool set_datetime(uint32_t now_ms);

    /**
     * @brief Request camera status from gimbal
     * 
     * @details Sends parameter get request for Status parameter. Response contains
     *          camera state including error flags, SD card space, recording status,
     *          gimbal angles, and camera settings (exposure, ISO, aperture).
     *          
     *          Requested periodically (every 10 seconds) for continuous monitoring.
     * 
     * @param[in] now_ms Current system time in milliseconds
     * 
     * @return true if status request was sent, false if rate-limited
     * 
     * @note Status response received via handle_param_get_set_response_string()
     * @note Updates _status_report.last_request_ms
     */
    bool request_status(uint32_t now_ms);

    /**
     * @brief Check if it's safe to send a message to gimbal
     * 
     * @details Verifies that sufficient time has elapsed since last parameter
     *          get/set message to avoid overwhelming the gimbal. If messages
     *          are sent too frequently, the camera may become unresponsive.
     *          
     *          Minimum interval: 100ms between parameter protocol messages.
     * 
     * @param[in] now_ms Current system time in milliseconds
     * 
     * @return true if safe to send message, false if should wait longer
     * 
     * @note Used to rate-limit parameter get/set operations
     * @note Does not apply to gimbal control or vehicle attitude messages
     */
    bool is_safe_to_send(uint32_t now_ms) const;

    /**
     * @name Internal State Variables
     * @{
     */
    
    /// True once driver has completed initialization (init() called)
    bool _initialised;

    /**
     * @name Gimbal Attitude Tracking
     * @{
     */
    
    /// Current gimbal attitude as quaternion (NED earth frame), from GimbalAttitudeStatus
    Quaternion _current_attitude_quat;
    
    /// System time (ms) when _current_attitude_quat was last updated from gimbal
    uint32_t _last_current_attitude_quat_ms;
    
    /** @} */

    /**
     * @name Camera State Tracking
     * @{
     */
    
    /// True if camera is currently recording video (from status or Recording parameter)
    bool _recording_video;
    
    /// Last digital zoom value sent to camera: 100-1000 in steps of 100 (1.0x to 10.0x)
    uint16_t _last_digital_zoom_param_value = 100;
    
    /// Last optical zoom value sent to camera: 100-250 in steps of 10 (1.0x to 2.5x)
    uint16_t _last_optical_zoom_param_value = 100;
    
    /**
     * @brief Zoom rate control state
     * 
     * @details Used for continuous zoom in/out when set_zoom() called with RATE type.
     *          Zoom continues until dir=0 or new zoom command received.
     */
    struct {
        bool enabled;           ///< True if zoom rate control is active
        int8_t dir;            ///< Zoom direction: -1=zoom out, 0=stop, +1=zoom in
        uint32_t last_update_ms; ///< System time (ms) zoom parameter was last updated
    } _zoom_rate_control;
    
    /** @} */

    /**
     * @name Firmware Version Tracking
     * @{
     */
    
    /**
     * @brief Firmware version information from gimbal
     * 
     * @details Queried via FirmwareVersion parameter on first update after init.
     *          Used for MAVLink CAMERA_INFORMATION message to report to GCS.
     */
    struct {
        uint32_t last_request_ms;  ///< System time (ms) of last firmware version request
        bool received;             ///< True once firmware version response received
        char str[12] {};          ///< Firmware version string (11 chars + null terminator)
        uint32_t mav_ver;         ///< Firmware version formatted as MAVLink version number
    } _firmware_version;
    
    /** @} */

    /**
     * @name Date/Time Synchronization
     * @{
     */
    
    /**
     * @brief Date and time synchronization state
     * 
     * @details Gimbal date/time is set once after node discovery to ensure
     *          proper timestamping of photos and videos for geotagging.
     */
    struct {
        uint32_t last_request_ms;  ///< System time (ms) when date/time was last set
        bool set;                  ///< True once date/time has been successfully set
    } _datetime;
    
    /** @} */

    /**
     * @name Camera Capability Discovery
     * @{
     */
    
    /**
     * @brief Tri-state capability flag
     */
    enum class Capability : uint8_t {
        False = 0,    ///< Capability confirmed absent
        True = 1,     ///< Capability confirmed present
        Unknown = 2,  ///< Capability not yet determined
    };
    
    /**
     * @brief Camera capability detection state
     * 
     * @details Queries camera parameters to determine which features are
     *          available. Has timeout (10 seconds) to prevent indefinite waiting.
     */
    struct {
        bool received;             ///< True once capability discovery complete (success or timeout)
        uint32_t first_request_ms; ///< System time (ms) of first capability request (for timeout)
        uint32_t last_request_ms;  ///< System time (ms) of last capability parameter request
        Capability optical_zoom;   ///< Optical zoom capability (True/False/Unknown)
    } capabilities = {false, 0, 0, Capability::Unknown};
    
    /** @} */

    /**
     * @name Gimbal Status Monitoring
     * @{
     */
    
    /**
     * @brief Error and status bit flags from gimbal Status parameter
     * 
     * @details Bit flags in error_status field of status message. Multiple
     *          flags can be set simultaneously. Used for health reporting and
     *          user notifications.
     */
    enum class ErrorStatus : uint32_t {
        TAKING_PICTURE        = 0x04,      ///< Camera is currently capturing a photo
        RECORDING_VIDEO       = 0x08,      ///< Camera is currently recording video
        CANNOT_TAKE_PIC       = 0x20,      ///< Unable to take picture (e.g., while recording)
        TIME_NOT_SET          = 0x10000,   ///< Date/time not set on camera
        MEDIA_ERROR           = 0x20000,   ///< SD card error or full
        LENS_ERROR            = 0x40000,   ///< Camera lens malfunction
        MOTOR_INIT_ERROR      = 0x100000,  ///< Gimbal motor initialization failed
        MOTOR_OPERATION_ERROR = 0x200000,  ///< Gimbal motor operation error
        GIMBAL_CONTROL_ERROR  = 0x400000,  ///< Gimbal control system error
        TEMP_WARNING          = 0x1000000  ///< Temperature warning (overheating)
    };
    
    /**
     * @brief Comprehensive camera and gimbal status
     * 
     * @details Structure matches Status parameter response format (48 bytes).
     *          Received periodically via parameter get response. Contains camera
     *          operational state, SD card info, gimbal angles, and camera settings.
     */
    struct {
        uint32_t error_status;        ///< Error/status bit field (see ErrorStatus enum)
        uint32_t video_remain_time;   ///< Remaining video recording time in seconds
        uint32_t photo_remain_count;  ///< Remaining photo capacity (number of photos)
        uint32_t sd_card_size_mb;     ///< Total SD card capacity in megabytes
        uint32_t sd_card_free_mb;     ///< Available SD card space in megabytes
        uint16_t body;                ///< Camera body type identifier
        uint16_t cmos;                ///< CMOS sensor type identifier
        uint16_t gimbal_pitch;        ///< Current gimbal pitch angle (units TBD)
        uint16_t gimbal_roll;         ///< Current gimbal roll angle (units TBD)
        uint16_t gimbal_yaw;          ///< Current gimbal yaw angle (units TBD)
        uint16_t reserved1;           ///< Reserved field
        uint8_t date_time[7];         ///< Camera date/time (year, month, day, hour, min, sec, ?)
        uint8_t reserved2;            ///< Reserved field
        uint32_t exposure_time_us;    ///< Current exposure time in microseconds
        uint16_t apeture;             ///< Current aperture value * 100 (e.g., 280 = f/2.8)
        uint16_t iso_sensitivity;     ///< Current ISO sensitivity value
    } _status;
    
    /// Compile-time check that status structure is exactly 48 bytes as expected
    static_assert(sizeof(_status) == 48, "status must be 48 bytes");
    
    /**
     * @brief Status reporting and monitoring state
     */
    struct {
        uint32_t last_request_ms;    ///< System time (ms) when status was last requested
        uint32_t last_error_status;  ///< Previous error_status value (for change detection)
    } _status_report;
    
    /// True if motor or gimbal control error detected (affects healthy() reporting)
    bool _motor_error;
    
    /// True if camera lens or media error detected
    bool _camera_error;
    
    /** @} */

    /**
     * @name DroneCAN Protocol Integration
     * @{
     */
    
    /**
     * @brief Backend registry entry for DroneCAN node mapping
     * 
     * @details Maps DroneCAN interface and node ID to backend instance.
     *          Used by static message handlers to route messages to correct backend.
     *          Protected by _sem_registry semaphore for thread-safe access.
     */
    static struct DetectedModules {
        AP_Mount_Xacti *driver;   ///< Pointer to backend instance (nullptr if slot unused)
        AP_DroneCAN* ap_dronecan; ///< DroneCAN interface this backend uses
        uint8_t node_id;          ///< Gimbal DroneCAN node ID (0 = not yet discovered)
    } _detected_modules[AP_MOUNT_MAX_INSTANCES];
    
    /// Semaphore protecting concurrent access to _detected_modules registry
    static HAL_Semaphore _sem_registry;
    
    /**
     * @name Message Rate Limiting Timestamps
     * @{
     */
    
    /// System time (ms) when send_gimbal_control() was last called (5Hz rate limit)
    uint32_t last_send_gimbal_control_ms;
    
    /// System time (ms) when send_copter_att_status() was last called (10Hz rate limit)
    uint32_t last_send_copter_att_status_ms;
    
    /// System time (ms) when parameter get/set message was last sent (rate limiting)
    uint32_t last_send_getset_param_ms;
    
    /** @} */

    /**
     * @name Parameter Queue Management
     * @{
     */
    
    /**
     * @brief Queue item for int32 set-parameter requests
     * 
     * @details Set-parameter requests are queued and sent sequentially to
     *          avoid overwhelming the camera, which can become unresponsive
     *          if too many parameter changes are sent simultaneously.
     */
    struct SetParamQueueItem {
        Param param;      ///< Parameter to set (name via enum)
        int32_t value;    ///< Parameter value to set
    };
    
    /// Queue of pending set-parameter int32 requests (FIFO processing)
    ObjectArray<SetParamQueueItem> *_set_param_int32_queue;
    
    /** @} */
    /** @} */ // End of DroneCAN Protocol Integration
};

#endif // HAL_MOUNT_XACTI_ENABLED
