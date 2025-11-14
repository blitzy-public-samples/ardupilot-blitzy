/**
 * @file AP_Camera.h
 * @brief Photo or video camera manager, with EEPROM-backed storage of constants.
 * 
 * @details This file implements the AP_Camera singleton manager which provides a unified interface
 *          for controlling cameras and gimbals in ArduPilot. The architecture consists of:
 *          
 *          - Singleton Manager: AP_Camera class coordinates all camera instances and backends
 *          - Backend System: Pluggable drivers support various camera types:
 *            * Servo: PWM-controlled cameras via SRV_Channels
 *            * Relay: Simple relay-triggered cameras
 *            * Mount: Cameras integrated with gimbal mounts (forwards to AP::mount())
 *            * MAVLink: MAVLink v1 camera control
 *            * MAVLinkCamV2: Full MAVLink Camera Protocol v2 support
 *            * SoloGimbal: GoPro cameras in 3DR Solo gimbal
 *            * Scripting: Lua script-controlled cameras
 *            * RunCam: UART-based RunCam protocol
 *          
 *          - Trigger Modes: Multiple triggering methods supported:
 *            * Time-interval: Periodic captures at specified intervals
 *            * Distance-based: Triggered by GPS distance traveled (mission integration)
 *            * Manual: Direct user commands via RC or GCS
 *            * Mission: DO_DIGICAM_CONTROL and DO_SET_CAM_TRIGG_DIST commands
 *          
 *          - MAVLink Camera Protocol v2: Full protocol compliance including:
 *            * CAMERA_INFORMATION, CAMERA_SETTINGS, CAMERA_CAPTURE_STATUS messages
 *            * VIDEO_STREAM_INFORMATION, CAMERA_FOV_STATUS messages
 *            * Zoom, focus, tracking, and multi-sensor control
 *            * Command handling (IMAGE_START_CAPTURE, VIDEO_START_RECORDING, etc.)
 *          
 *          - Geotagging: Links each trigger to GPS position, time, and vehicle attitude
 *            * Logs CAM messages with location, altitude, roll/pitch/yaw
 *            * Logs TRIG messages for external camera triggering
 *            * Integrates with AP_Logger for persistent storage
 *          
 *          - Parameter Management: Uses AP_Param for persistent configuration
 *            * CAM1_TYPE, CAM2_TYPE select backend drivers
 *            * Per-instance parameters (trigger distance, intervals, limits)
 *            * Legacy parameter conversion on initialization
 *          
 *          - Compile-time Feature Gating: Backends enabled via AP_CAMERA_*_ENABLED flags
 *          
 * @note Maximum instances: AP_CAMERA_MAX_INSTANCES (typically 2)
 * @note Update rate: Should be called at 50Hz via update() method
 * @note Thread-safety: Use get_semaphore() for multi-threaded access
 * 
 * @see AP_Camera_Backend for backend interface
 * @see AP_Camera_Params for per-instance parameters
 * @see GCS_MAVLink for MAVLink protocol integration
 * @see AP::mount() for gimbal integration
 */
#pragma once

#include "AP_Camera_config.h"

#if AP_CAMERA_ENABLED

#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/ap_message.h>
#include "AP_Camera_Params.h"
#include "AP_Camera_shareddefs.h"

#define AP_CAMERA_MAX_INSTANCES             2       // maximum number of camera backends

// declare backend classes
class AP_Camera_Backend;
class AP_Camera_Servo;
class AP_Camera_Relay;
class AP_Camera_SoloGimbal;
class AP_Camera_Mount;
class AP_Camera_MAVLink;
class AP_Camera_MAVLinkCamV2;
class AP_Camera_Scripting;
class AP_RunCam;

/**
 * @class AP_Camera
 * @brief Singleton camera manager coordinating multiple camera backends and MAVLink integration
 * 
 * @details AP_Camera is the top-level singleton manager that:
 * 
 *          **Parameter Management:**
 *          - Registers persistent CAMx_ parameters via AP_Param::GroupInfo (var_info)
 *          - Supports up to AP_CAMERA_MAX_INSTANCES camera instances (typically 2)
 *          - Each instance has CAMx_TYPE to select backend driver
 *          - Performs legacy parameter conversion on init() for backward compatibility
 * 
 *          **Backend Instantiation:**
 *          - Instantiates backend drivers based on CAMx_TYPE parameter
 *          - Available backends (compile-time conditional):
 *            * SERVO: PWM control via SRV_Channels (AP_CAMERA_SERVO_ENABLED)
 *            * RELAY: Relay trigger control (AP_CAMERA_RELAY_ENABLED)
 *            * MOUNT: Forwards to AP::mount() for gimbal-integrated cameras (AP_CAMERA_MOUNT_ENABLED)
 *            * MAVLINK: MAVLink v1 camera commands (AP_CAMERA_MAVLINK_ENABLED)
 *            * MAVLINK_CAMV2: Full Camera Protocol v2 (AP_CAMERA_MAVLINKCAMV2_ENABLED)
 *            * SOLOGIMBAL: 3DR Solo GoPro control (AP_CAMERA_SOLOGIMBAL_ENABLED)
 *            * SCRIPTING: Lua script integration (AP_CAMERA_SCRIPTING_ENABLED)
 *            * RUNCAM: UART RunCam protocol (AP_CAMERA_RUNCAM_ENABLED)
 *          - Primary backend pointer maintained for default operations
 * 
 *          **MAVLink Integration:**
 *          - Handles incoming MAVLink messages via handle_message()
 *          - Processes camera commands via handle_command() returning MAV_RESULT
 *          - Generates outgoing MAVLink messages via send_mavlink_message()
 *          - Implements MAVLink Camera Protocol v2 for advanced features:
 *            * CAMERA_INFORMATION: Camera capabilities and identification
 *            * CAMERA_SETTINGS: Current camera configuration
 *            * CAMERA_CAPTURE_STATUS: Image/video capture status
 *            * VIDEO_STREAM_INFORMATION: Video streaming details
 *            * CAMERA_FOV_STATUS: Field of view information
 *            * CAMERA_THERMAL_RANGE: Thermal camera range data
 *            * CAMERA_FEEDBACK: Geotagged trigger events
 * 
 *          **Mission Integration:**
 *          - Distance-based triggering for aerial mapping/surveying
 *          - GPS distance calculation between triggers
 *          - DO_DIGICAM_CONTROL mission command support
 *          - DO_SET_CAM_TRIGG_DIST for automatic interval triggering
 *          - Optional AUTO mode gating via _auto_mode_only parameter
 *          - Roll angle limiting via _max_roll to prevent tilted images
 * 
 *          **Geotagging:**
 *          - Links each trigger to GPS position (lat/lon/alt)
 *          - Records GPS time (milliseconds since GPS week start)
 *          - Logs vehicle attitude (roll/pitch/yaw in degrees)
 *          - Generates CAM log messages via AP_Logger
 *          - Generates TRIG log messages for external cameras
 *          - Sends CAMERA_FEEDBACK to GCS with location data
 * 
 *          **Public API for Vehicle Code and GCS:**
 *          - take_picture(): Single image capture
 *          - take_multiple_pictures(): Time-interval capture sequence
 *          - record_video(): Start/stop video recording
 *          - set_zoom(), set_focus(), set_tracking(): Camera Protocol v2 features
 *          - set_camera_source(): Multi-sensor camera selection
 *          - configure(), control(): Legacy DIGICAM configuration
 *          - set_trigger_distance(): Distance-based triggering
 * 
 *          **Thread-Safety:**
 *          - Protected by _rsem HAL_Semaphore for multi-threaded access
 *          - Use get_semaphore() and WITH_SEMAPHORE() pattern for thread-safe operations
 *          - Update() method typically called from main vehicle loop at 50Hz
 * 
 *          **Gimbal Coordination:**
 *          - Integrates with AP::mount() for gimbal-mounted cameras
 *          - Mount backend forwards camera commands to gimbal
 *          - Coordinates attitude information for geotagging
 * 
 * @note This is a singleton class - use AP_Camera::get_singleton() or AP::camera() to access
 * @note Maximum instances: AP_CAMERA_MAX_INSTANCES = 2
 * @note Thread-safety: Always use get_semaphore() for multi-threaded access
 * @note Update rate: Call update() at 50Hz (typical main loop rate)
 * @note Parameter persistence: All CAMx_ parameters stored in EEPROM via AP_Param
 * @note Compile-time configuration: Backends enabled via AP_CAMERA_*_ENABLED feature flags
 * 
 * @warning Distance triggering respects _auto_mode_only and _max_roll safety limits
 * @warning Timing-critical: update() should be called consistently at 50Hz
 * 
 * @see AP_Camera_Backend for backend interface definition
 * @see AP_Camera_Params for per-instance parameter structure
 * @see AP_Camera_Servo, AP_Camera_Relay, AP_Camera_Mount, AP_Camera_MAVLink, AP_Camera_MAVLinkCamV2
 * @see AP_Camera_Scripting, AP_RunCam for specific backend implementations
 * @see GCS_MAVLink for MAVLink protocol handling
 * @see LogStructure.h for CAM/TRIG message formats
 * @see AP::mount() for gimbal integration
 */
class AP_Camera {

    // declare backends as friends
    friend class AP_Camera_Backend;
    friend class AP_Camera_Servo;
    friend class AP_Camera_Relay;
    friend class AP_Camera_SoloGimbal;
    friend class AP_Camera_Mount;
    friend class AP_Camera_MAVLink;
    friend class AP_Camera_MAVLinkCamV2;
    friend class AP_Camera_Scripting;
    friend class AP_RunCam;

public:

    /**
     * @brief Constructor for AP_Camera singleton manager
     * 
     * @param[in] _log_camera_bit Logging bit from LOG_BITMASK parameter to enable camera logging
     *                            When this bit is set, CAM and TRIG messages are logged
     * 
     * @note Typically called once during vehicle initialization
     * @note The log bit enables geotagged trigger logging in AP_Logger
     * 
     * @see AP_Logger for logging system integration
     */
    AP_Camera(uint32_t _log_camera_bit);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Camera);

    /**
     * @brief Get singleton instance of AP_Camera
     * 
     * @return AP_Camera* Pointer to singleton instance, or nullptr if not initialized
     * 
     * @note Singleton pattern ensures only one AP_Camera instance exists
     * @note Thread-safe access guaranteed after initialization
     * @note Prefer using AP::camera() namespace accessor for cleaner code
     * 
     * @see AP::camera() for namespace accessor
     */
    static AP_Camera *get_singleton() { return _singleton; }

    /**
     * @brief Camera backend types defining the driver implementation
     * 
     * @note Selected via CAMx_TYPE parameter (e.g., CAM1_TYPE, CAM2_TYPE)
     * @note Backend availability depends on compile-time feature flags
     * @note NONE (0) disables the camera instance
     */
    enum class CameraType {
        NONE = 0,           ///< No camera backend (disabled)
#if AP_CAMERA_SERVO_ENABLED
        SERVO = 1,          ///< PWM servo control via SRV_Channels (requires AP_CAMERA_SERVO_ENABLED)
#endif
#if AP_CAMERA_RELAY_ENABLED
        RELAY = 2,          ///< Simple relay trigger control (requires AP_CAMERA_RELAY_ENABLED)
#endif
#if AP_CAMERA_SOLOGIMBAL_ENABLED
        SOLOGIMBAL = 3,     ///< 3DR Solo gimbal with integrated GoPro control (requires AP_CAMERA_SOLOGIMBAL_ENABLED)
#endif
#if AP_CAMERA_MOUNT_ENABLED
        MOUNT = 4,          ///< Camera integrated with gimbal mount, forwards to AP::mount() (requires AP_CAMERA_MOUNT_ENABLED)
#endif
#if AP_CAMERA_MAVLINK_ENABLED
        MAVLINK = 5,        ///< MAVLink v1 camera protocol support (requires AP_CAMERA_MAVLINK_ENABLED)
#endif
#if AP_CAMERA_MAVLINKCAMV2_ENABLED
        MAVLINK_CAMV2 = 6,  ///< Full MAVLink Camera Protocol v2 with zoom/focus/tracking (requires AP_CAMERA_MAVLINKCAMV2_ENABLED)
#endif
#if AP_CAMERA_SCRIPTING_ENABLED
        SCRIPTING = 7,      ///< Lua script-controlled camera backend (requires AP_CAMERA_SCRIPTING_ENABLED)
#endif
#if AP_CAMERA_RUNCAM_ENABLED
        RUNCAM = 8,         ///< UART-based RunCam protocol (requires AP_CAMERA_RUNCAM_ENABLED)
#endif
    };

    /**
     * @brief Detect and initialize camera backends based on CAMx_TYPE parameters
     * 
     * @details Initialization sequence:
     *          1. Performs legacy parameter conversion (convert_params())
     *          2. Iterates through CAMx_TYPE parameters for each instance
     *          3. Instantiates appropriate backend driver for each non-NONE type
     *          4. Calls backend init() method for hardware setup
     *          5. Sets primary backend pointer to first valid instance
     * 
     * @note Must be called once during vehicle initialization before using camera functions
     * @note Backends are instantiated dynamically based on TYPE parameter
     * @note Invalid TYPE values are ignored with error message
     * 
     * @warning Call only once during system initialization
     * @warning Must be called before update() or any camera operations
     * 
     * @see CameraType enum for available backend types
     * @see AP_Camera_Params for per-instance parameters
     */
    void init();

    /**
     * @brief Update all camera backends and enforce distance-based triggering
     * 
     * @details Called periodically (typically 50Hz) from main vehicle loop:
     *          - Iterates through all instantiated backends
     *          - Calls backend update() for time-based operations and state management
     *          - Calculates GPS distance traveled since last trigger
     *          - Enforces distance-based triggering when set_trigger_distance() configured
     *          - Respects AUTO mode gating (_auto_mode_only parameter)
     *          - Enforces roll angle limits (_max_roll parameter)
     * 
     * @note Must be called at consistent 50Hz rate for accurate timing
     * @note Distance calculation uses GPS position and haversine formula
     * @note Timing budget: Keep backend update() operations lightweight
     * 
     * @warning Inconsistent call rate affects time-interval capture accuracy
     * @warning GPS-based distance triggering requires valid GPS fix
     * 
     * @see set_trigger_distance() for configuring distance-based triggering
     * @see vehicle_mode_ok_for_trigg_dist() for AUTO mode checking
     */
    void update();

    /**
     * @brief Handle incoming MAVLink messages from camera devices
     * 
     * @param[in] chan MAVLink channel the message was received on
     * @param[in] msg  MAVLink message structure containing message ID and payload
     * 
     * @details Processes MAVLink messages sent by camera devices (not GCS commands):
     *          - Routes message to appropriate backend based on system/component ID
     *          - Typical messages: CAMERA_SETTINGS, CAMERA_CAPTURE_STATUS, etc.
     *          - Backends may update internal state based on camera feedback
     *          - Supports bidirectional communication with intelligent cameras
     * 
     * @note Called by GCS_MAVLink message routing system
     * @note Backends filter messages based on their target system/component
     * 
     * @see handle_command() for processing GCS commands to camera
     * @see GCS_MAVLink for message routing infrastructure
     */
    void handle_message(mavlink_channel_t chan, const mavlink_message_t &msg);

    /**
     * @brief Handle MAVLink camera control commands from GCS
     * 
     * @param[in] packet MAVLink COMMAND_INT packet with command ID and parameters
     * 
     * @return MAV_RESULT indicating command acceptance/rejection:
     *         - MAV_RESULT_ACCEPTED: Command executed successfully
     *         - MAV_RESULT_UNSUPPORTED: Command not supported by backend
     *         - MAV_RESULT_FAILED: Command failed to execute
     *         - MAV_RESULT_DENIED: Command rejected (e.g., invalid parameters)
     * 
     * @details Supported commands:
     *          - MAV_CMD_DO_DIGICAM_CONTROL: Legacy camera trigger (param5=1 for trigger)
     *          - MAV_CMD_DO_DIGICAM_CONFIGURE: Legacy camera configuration
     *          - MAV_CMD_IMAGE_START_CAPTURE: Start image capture (single or interval)
     *          - MAV_CMD_IMAGE_STOP_CAPTURE: Stop interval capture
     *          - MAV_CMD_VIDEO_START_CAPTURE: Start video recording
     *          - MAV_CMD_VIDEO_STOP_CAPTURE: Stop video recording
     *          - MAV_CMD_SET_CAMERA_ZOOM: Set zoom (rate or percentage)
     *          - MAV_CMD_SET_CAMERA_FOCUS: Set focus (rate, percentage, or auto)
     *          - MAV_CMD_SET_CAMERA_MODE: Toggle between image and video mode
     *          - MAV_CMD_SET_CAMERA_SOURCE: Select camera sensor (if multi-sensor)
     * 
     * @note Command parameters decoded from param1-param7 in COMMAND_INT
     * @note Returns UNSUPPORTED if backend doesn't implement requested feature
     * @note GCS should send COMMAND_ACK based on return value
     * 
     * @see configure() and control() for legacy DIGICAM command implementation
     * @see take_picture(), record_video(), set_zoom(), set_focus() for feature APIs
     */
    MAV_RESULT handle_command(const mavlink_command_int_t &packet);

    /**
     * @brief Send camera-related MAVLink messages to GCS
     * 
     * @param[in] link GCS_MAVLINK link object for sending messages
     * @param[in] id   Message identifier from ap_message enum
     * 
     * @return bool true if message sent successfully, false if insufficient buffer space
     * 
     * @details Supported message types (via ap_message enum):
     *          - MSG_CAMERA_FEEDBACK: Geotagged trigger events (lat/lon/alt/attitude)
     *          - MSG_CAMERA_INFORMATION: Camera capabilities and identification
     *          - MSG_CAMERA_SETTINGS: Current camera mode and settings
     *          - MSG_CAMERA_CAPTURE_STATUS: Image/video capture status and counts
     *          - MSG_VIDEO_STREAM_INFORMATION: Video streaming configuration
     *          - MSG_CAMERA_FOV_STATUS: Field of view and attitude information
     *          - MSG_CAMERA_THERMAL_RANGE: Thermal camera temperature range (if enabled)
     * 
     * @note Called by GCS_MAVLINK streaming system based on stream rates
     * @note Return false indicates telemetry buffer full, message will be retried
     * @note Message generation delegates to send_camera_information(), send_feedback(), etc.
     * 
     * @see send_camera_information(), send_camera_settings(), send_feedback() for specific messages
     * @see GCS_MAVLINK for telemetry streaming system
     */
    bool send_mavlink_message(class GCS_MAVLINK &link, const enum ap_message id);

    /**
     * @brief Configure camera settings (legacy MAV_CMD_DO_DIGICAM_CONFIGURE)
     * 
     * @param[in] shooting_mode  Camera shooting mode (backend-specific)
     * @param[in] shutter_speed  Shutter speed in seconds (1/value for fast speeds)
     * @param[in] aperture       Aperture f-number (e.g., 2.8, 5.6, 8.0)
     * @param[in] ISO            ISO sensitivity value (e.g., 100, 400, 1600)
     * @param[in] exposure_type  Exposure type/compensation (backend-specific)
     * @param[in] cmd_id         Command ID for tracking/acknowledgment
     * @param[in] engine_cutoff_time Time in seconds to cut engine during capture (0=disabled)
     * 
     * @note Configures primary camera instance (first valid backend)
     * @note Legacy command from MAVLink DIGICAM protocol
     * @note Backend capability varies - not all parameters supported by all cameras
     * @note Parameter semantics depend on specific backend implementation
     * 
     * @see configure(uint8_t instance, ...) to configure specific camera instance
     */
    void configure(float shooting_mode, float shutter_speed, float aperture, float ISO, int32_t exposure_type, int32_t cmd_id, float engine_cutoff_time);
    
    /**
     * @brief Configure specific camera instance settings
     * 
     * @param[in] instance       Camera instance number (0 to AP_CAMERA_MAX_INSTANCES-1)
     * @param[in] shooting_mode  Camera shooting mode (backend-specific)
     * @param[in] shutter_speed  Shutter speed in seconds
     * @param[in] aperture       Aperture f-number
     * @param[in] ISO            ISO sensitivity value
     * @param[in] exposure_type  Exposure type/compensation (backend-specific)
     * @param[in] cmd_id         Command ID for tracking/acknowledgment
     * @param[in] engine_cutoff_time Time in seconds to cut engine during capture (0=disabled)
     * 
     * @note Backend must support configuration commands (not all backends do)
     * @note Invalid instance numbers are safely ignored
     * 
     * @see AP_Camera_Backend::configure() for backend implementation
     */
    void configure(uint8_t instance, float shooting_mode, float shutter_speed, float aperture, float ISO, int32_t exposure_type, int32_t cmd_id, float engine_cutoff_time);

    /**
     * @brief Control camera operation (legacy MAV_CMD_DO_DIGICAM_CONTROL)
     * 
     * @param[in] session      Session control (backend-specific, typically unused)
     * @param[in] zoom_pos     Absolute zoom position (backend-specific units)
     * @param[in] zoom_step    Relative zoom step (positive=zoom in, negative=zoom out)
     * @param[in] focus_lock   Focus lock control (0=unlock, 1=lock, backend-specific)
     * @param[in] shooting_cmd Shooting command (0=none, 1=take picture, backend-specific)
     * @param[in] cmd_id       Command ID for tracking/acknowledgment
     * 
     * @note Controls primary camera instance (first valid backend)
     * @note Legacy DIGICAM_CONTROL command mapping to MAVLink protocol
     * @note Parameter semantics vary significantly by backend
     * @note Modern code should use take_picture(), set_zoom(), set_focus() instead
     * 
     * @see control(uint8_t instance, ...) to control specific camera instance
     * @see take_picture(), set_zoom(), set_focus() for modern Camera Protocol v2 APIs
     */
    void control(float session, float zoom_pos, float zoom_step, float focus_lock, int32_t shooting_cmd, int32_t cmd_id);
    
    /**
     * @brief Control specific camera instance operation
     * 
     * @param[in] instance     Camera instance number (0 to AP_CAMERA_MAX_INSTANCES-1)
     * @param[in] session      Session control (backend-specific)
     * @param[in] zoom_pos     Absolute zoom position
     * @param[in] zoom_step    Relative zoom step
     * @param[in] focus_lock   Focus lock control
     * @param[in] shooting_cmd Shooting command (1=take picture)
     * @param[in] cmd_id       Command ID for tracking
     * 
     * @note Invalid instance numbers are safely ignored
     * @note Backend capability varies for each parameter
     * 
     * @see AP_Camera_Backend::control() for backend implementation
     */
    void control(uint8_t instance, float session, float zoom_pos, float zoom_step, float focus_lock, int32_t shooting_cmd, int32_t cmd_id);

    /**
     * @brief Set camera trigger distance for GPS-based triggering (mission integration)
     * 
     * @param[in] distance_m Trigger distance in meters (0 disables distance triggering)
     * 
     * @details Enables automatic camera triggering based on GPS distance traveled:
     *          - Calculates distance using GPS lat/lon and haversine formula
     *          - Triggers camera when specified distance accumulated
     *          - Used for aerial mapping and surveying missions
     *          - Integrates with DO_SET_CAM_TRIGG_DIST mission command
     *          - Respects _auto_mode_only parameter (may require AUTO mode)
     *          - Enforces _max_roll parameter to prevent tilted images
     * 
     * @note Affects primary camera instance (first valid backend)
     * @note Requires valid GPS fix with sufficient accuracy
     * @note Distance calculation performed in update() at 50Hz
     * @note Set to 0 to disable distance-based triggering
     * 
     * @warning GPS accuracy affects trigger spacing precision
     * @warning _auto_mode_only parameter may gate triggering to AUTO mode only
     * @warning _max_roll parameter prevents triggering when roll angle exceeds limit
     * 
     * @see set_trigger_distance(uint8_t instance, float distance_m) for specific instance
     * @see vehicle_mode_ok_for_trigg_dist() for AUTO mode checking
     * @see get_roll_max() for roll angle limit
     */
    void set_trigger_distance(float distance_m);
    
    /**
     * @brief Set trigger distance for specific camera instance
     * 
     * @param[in] instance   Camera instance number (0 to AP_CAMERA_MAX_INSTANCES-1)
     * @param[in] distance_m Trigger distance in meters (0 disables)
     * 
     * @note Invalid instance numbers are safely ignored
     * @note Same triggering rules apply as primary instance version
     * 
     * @see set_trigger_distance(float distance_m) for detailed behavior
     */
    void set_trigger_distance(uint8_t instance, float distance_m);

    /**
     * @brief Momentary switch to toggle camera between picture and video modes
     * 
     * @details Toggles camera mode:
     *          - Picture mode → Video mode
     *          - Video mode → Picture mode
     *          - Typically mapped to RC auxiliary function switch
     *          - Backend-specific implementation (not all support mode switching)
     * 
     * @note Affects primary camera instance (first valid backend)
     * @note Backend must support mode switching (e.g., MAVLink Camera v2)
     * @note No effect if backend doesn't implement mode switching
     * 
     * @see cam_mode_toggle(uint8_t instance) to toggle specific instance
     * @see AP_Camera_Backend::cam_mode_toggle() for backend implementation
     */
    void cam_mode_toggle();
    
    /**
     * @brief Toggle specific camera instance between picture and video modes
     * 
     * @param[in] instance Camera instance number (0 to AP_CAMERA_MAX_INSTANCES-1)
     * 
     * @note Invalid instance numbers are safely ignored
     * @note Backend capability varies
     */
    void cam_mode_toggle(uint8_t instance);

    /**
     * @brief Take a single picture with all available cameras
     * 
     * @return bool true if at least one camera successfully triggered, false if all failed
     * 
     * @details Triggers single image capture:
     *          - Iterates through all instantiated backend instances
     *          - Calls take_picture() on each backend
     *          - Backend-specific trigger mechanism (servo pulse, relay, MAVLink command, etc.)
     *          - Generates CAM log message with GPS position and attitude
     *          - Sends CAMERA_FEEDBACK message to GCS with geotagging data
     * 
     * @note Affects ALL camera instances (no instance parameter = all cameras)
     * @note Returns true if ANY camera succeeds (not all must succeed)
     * @note Geotagging data logged at time of trigger
     * 
     * @see take_picture(uint8_t instance) to trigger specific camera only
     * @see AP_Camera_Backend::take_picture() for backend implementation
     */
    bool take_picture();
    
    /**
     * @brief Take a single picture with specific camera instance
     * 
     * @param[in] instance Camera instance number (0 to AP_CAMERA_MAX_INSTANCES-1)
     * 
     * @return bool true if camera successfully triggered, false if failed or invalid instance
     * 
     * @note Invalid instance numbers return false
     * @note Each backend implements trigger differently (PWM, relay, MAVLink, etc.)
     */
    bool take_picture(uint8_t instance);

    /**
     * @brief Start time-interval capture sequence on all available cameras
     * 
     * @param[in] time_interval_ms Time interval between pictures in milliseconds (must be positive)
     * @param[in] total_num        Total number of pictures to capture, -1 for continuous capture
     * 
     * @return bool true if at least one camera started capture sequence, false if all failed
     * 
     * @details Initiates time-interval image capture:
     *          - Backend manages timing and automatically triggers at specified intervals
     *          - Continues until total_num reached or stop_capture() called
     *          - total_num = -1 enables continuous capture until manually stopped
     *          - Each trigger generates CAM log message and CAMERA_FEEDBACK
     *          - Enforces interval_min parameter to protect camera hardware
     *          - Used for time-lapse photography and periodic monitoring
     * 
     * @note Affects ALL camera instances (no instance parameter = all cameras)
     * @note time_interval_ms must be positive, enforced by backend
     * @note Backend enforces minimum interval (interval_min parameter) to prevent hardware damage
     * @note Returns true if ANY camera succeeds (not all must succeed)
     * 
     * @warning Verify time_interval_ms exceeds camera's minimum recycle time
     * @warning Continuous capture (total_num=-1) continues until stop_capture() called
     * 
     * @see take_multiple_pictures(uint8_t instance, ...) for specific camera
     * @see stop_capture() to terminate sequence
     * @see AP_Camera_Params::interval_min for minimum interval enforcement
     */
    bool take_multiple_pictures(uint32_t time_interval_ms, int16_t total_num);
    
    /**
     * @brief Start time-interval capture on specific camera instance
     * 
     * @param[in] instance         Camera instance number (0 to AP_CAMERA_MAX_INSTANCES-1)
     * @param[in] time_interval_ms Time interval in milliseconds (must be positive)
     * @param[in] total_num        Number of pictures (-1 for continuous)
     * 
     * @return bool true if camera started sequence, false if failed or invalid instance
     * 
     * @note Invalid instance numbers return false
     * @note Backend enforces minimum interval for camera protection
     */
    bool take_multiple_pictures(uint8_t instance, uint32_t time_interval_ms, int16_t total_num);

    /**
     * @brief Stop time-interval capture sequence on all cameras
     * 
     * @details Terminates ongoing take_multiple_pictures() sequence:
     *          - Iterates through all instantiated backends
     *          - Calls stop_capture() on each backend
     *          - Immediately halts interval triggering
     *          - Safe to call even if no capture sequence active
     * 
     * @note Affects ALL camera instances
     * @note No return value (void) - always succeeds
     * @note Has no effect if no capture sequence is running
     * 
     * @see stop_capture(uint8_t instance) to stop specific camera
     * @see take_multiple_pictures() for starting interval capture
     */
    void stop_capture();
    
    /**
     * @brief Stop capture sequence on specific camera instance
     * 
     * @param[in] instance Camera instance number (0 to AP_CAMERA_MAX_INSTANCES-1)
     * 
     * @return bool true if backend supports stop (not necessarily if sequence was running)
     *              false if invalid instance
     * 
     * @note Invalid instance numbers return false
     * @note Returns true even if no sequence was active
     */
    bool stop_capture(uint8_t instance);

    /**
     * @brief Start or stop video recording on all available cameras
     * 
     * @param[in] start_recording true to start recording, false to stop recording
     * 
     * @return bool true if at least one camera started/stopped recording, false if all failed
     * 
     * @details Controls video recording:
     *          - Iterates through all instantiated backends
     *          - Calls record_video() on each backend
     *          - Backend-specific implementation (MAVLink command, relay, etc.)
     *          - May generate log messages for recording events
     *          - Not all backends support video recording
     * 
     * @note Affects ALL camera instances
     * @note Returns true if ANY camera succeeds
     * @note Backend capability required - servo/relay backends may not support video
     * @note MAVLink Camera v2 backends typically support video recording
     * 
     * @see record_video(uint8_t instance, bool) for specific camera
     * @see AP_Camera_Backend::record_video() for backend implementation
     */
    bool record_video(bool start_recording);
    
    /**
     * @brief Start or stop video recording on specific camera instance
     * 
     * @param[in] instance        Camera instance number (0 to AP_CAMERA_MAX_INSTANCES-1)
     * @param[in] start_recording true to start, false to stop
     * 
     * @return bool true if backend supports and executed command, false otherwise
     * 
     * @note Invalid instance numbers return false
     * @note Returns false if backend doesn't support video recording
     */
    bool record_video(uint8_t instance, bool start_recording);

    /**
     * @brief Set zoom level on all available cameras
     * 
     * @param[in] zoom_type  ZoomType enum: RATE (continuous zoom) or PCT (absolute position)
     * @param[in] zoom_value For RATE: -1=zoom out, 0=hold, 1=zoom in
     *                       For PCT: 0-100 (0=wide angle, 100=telephoto)
     * 
     * @return bool true if at least one camera accepted zoom command, false if all failed
     * 
     * @details MAVLink Camera Protocol v2 zoom control:
     *          - RATE mode: Continuous zoom at specified rate until hold (0) sent
     *          - PCT mode: Absolute zoom position as percentage of range
     *          - Backend-specific implementation (MAVLink, gimbal, etc.)
     *          - Not all backends support zoom control
     *          - Typically used with MAVLink Camera v2 or Mount backends
     * 
     * @note Affects ALL camera instances
     * @note Returns true if ANY camera succeeds
     * @note Backend capability varies - servo/relay backends don't support zoom
     * @note MAVLink Camera v2 backends implement full zoom control
     * 
     * @see set_zoom(uint8_t instance, ...) for specific camera
     * @see ZoomType enum in AP_Camera_shareddefs.h
     * @see AP_Camera_Backend::set_zoom() for backend implementation
     */
    bool set_zoom(ZoomType zoom_type, float zoom_value);
    
    /**
     * @brief Set zoom on specific camera instance
     * 
     * @param[in] instance   Camera instance number (0 to AP_CAMERA_MAX_INSTANCES-1)
     * @param[in] zoom_type  RATE or PCT
     * @param[in] zoom_value Rate or percentage value
     * 
     * @return bool true if backend supports and accepted zoom, false otherwise
     * 
     * @note Invalid instance or unsupported backend returns false
     */
    bool set_zoom(uint8_t instance, ZoomType zoom_type, float zoom_value);

    /**
     * @brief Set focus on all available cameras
     * 
     * @param[in] focus_type  FocusType enum: RATE, PCT, or AUTO
     * @param[in] focus_value For RATE: -1=focus in, 0=hold, 1=focus out
     *                        For PCT: 0-100 (0=near, 100=far)
     *                        For AUTO: value ignored (autofocus enabled)
     * 
     * @return SetFocusResult indicating outcome:
     *         - ACCEPTED: Focus command accepted and executed
     *         - FAILED: Command accepted but execution failed
     *         - UNSUPPORTED: Backend doesn't support focus control
     * 
     * @details MAVLink Camera Protocol v2 focus control:
     *          - RATE mode: Continuous focus adjustment until hold (0) sent
     *          - PCT mode: Absolute focus position as percentage of range
     *          - AUTO mode: Enables camera's autofocus system
     *          - Backend-specific implementation
     *          - Not all backends support focus control
     * 
     * @note Affects ALL camera instances (returns ACCEPTED if any succeed)
     * @note Backend capability required - typically MAVLink Camera v2 or Mount
     * @note AUTO mode behavior depends on camera's autofocus capabilities
     * 
     * @see set_focus(uint8_t instance, ...) for specific camera
     * @see FocusType, SetFocusResult enums in AP_Camera_shareddefs.h
     * @see AP_Camera_Backend::set_focus() for backend implementation
     */
    SetFocusResult set_focus(FocusType focus_type, float focus_value);
    
    /**
     * @brief Set focus on specific camera instance
     * 
     * @param[in] instance    Camera instance number (0 to AP_CAMERA_MAX_INSTANCES-1)
     * @param[in] focus_type  RATE, PCT, or AUTO
     * @param[in] focus_value Rate, percentage, or ignored for AUTO
     * 
     * @return SetFocusResult ACCEPTED, FAILED, or UNSUPPORTED
     * 
     * @note Invalid instance returns UNSUPPORTED
     */
    SetFocusResult set_focus(uint8_t instance, FocusType focus_type, float focus_value);

    /**
     * @brief Set object tracking on all available cameras
     * 
     * @param[in] tracking_type TrackingType enum: NONE, POINT, or RECTANGLE
     * @param[in] p1            For POINT: center point (x,y in 0-1 range)
     *                          For RECTANGLE: top-left corner (x,y in 0-1 range)
     *                          For NONE: ignored
     * @param[in] p2            For RECTANGLE: bottom-right corner (x,y in 0-1 range)
     *                          For POINT/NONE: ignored
     * 
     * @return bool true if at least one camera accepted tracking command, false if all failed
     * 
     * @details MAVLink Camera Protocol v2 object tracking:
     *          - NONE: Disables object tracking
     *          - POINT: Track object at specified point (p1.x, p1.y)
     *          - RECTANGLE: Track object in specified region (p1=top-left, p2=bottom-right)
     *          - Coordinates normalized: 0=left/top, 1=right/bottom
     *          - Backend-specific implementation (intelligent cameras only)
     *          - Camera attempts to keep tracked object centered/in-frame
     * 
     * @note Affects ALL camera instances
     * @note Coordinate system: x=0 is left edge, x=1 is right edge
     *                          y=0 is top edge, y=1 is bottom edge
     * @note Point mode uses only p1, rectangle mode uses both p1 and p2
     * @note Backend capability required - typically MAVLink Camera v2 only
     * 
     * @see set_tracking(uint8_t instance, ...) for specific camera
     * @see TrackingType enum in AP_Camera_shareddefs.h
     * @see AP_Camera_Backend::set_tracking() for backend implementation
     */
    bool set_tracking(TrackingType tracking_type, const Vector2f& p1, const Vector2f& p2);
    
    /**
     * @brief Set object tracking on specific camera instance
     * 
     * @param[in] instance      Camera instance number (0 to AP_CAMERA_MAX_INSTANCES-1)
     * @param[in] tracking_type NONE, POINT, or RECTANGLE
     * @param[in] p1            Center/top-left point (normalized 0-1)
     * @param[in] p2            Bottom-right point (normalized 0-1, used for RECTANGLE only)
     * 
     * @return bool true if backend supports and accepted tracking, false otherwise
     * 
     * @note Invalid instance or unsupported backend returns false
     */
    bool set_tracking(uint8_t instance, TrackingType tracking_type, const Vector2f& p1, const Vector2f& p2);

#if AP_CAMERA_SET_CAMERA_SOURCE_ENABLED
    /**
     * @brief Set camera lens/sensor selection on all cameras
     * 
     * @param[in] lens Lens/sensor index 0-5 (camera-specific meaning)
     * 
     * @return bool true if at least one camera accepted lens selection, false if all failed
     * 
     * @details Multi-sensor camera control:
     *          - Selects active camera sensor/lens on multi-sensor gimbals
     *          - Lens index meaning is camera-specific (0-5 typically)
     *          - Used for cameras with multiple sensors (RGB, IR, etc.)
     *          - Backend-specific implementation
     * 
     * @note Affects ALL camera instances
     * @note Requires AP_CAMERA_SET_CAMERA_SOURCE_ENABLED compile flag
     * @note Backend capability required - multi-sensor cameras only
     * @note Prefer set_camera_source() for type-safe sensor selection
     * 
     * @see set_lens(uint8_t instance, uint8_t lens) for specific camera
     * @see set_camera_source() for type-safe alternative using CameraSource enum
     */
    bool set_lens(uint8_t lens);
    
    /**
     * @brief Set lens on specific camera instance
     * 
     * @param[in] instance Camera instance number (0 to AP_CAMERA_MAX_INSTANCES-1)
     * @param[in] lens     Lens index 0-5
     * 
     * @return bool true if backend supports and accepted lens selection, false otherwise
     * 
     * @note Invalid instance or unsupported backend returns false
     */
    bool set_lens(uint8_t instance, uint8_t lens);

    /**
     * @brief Camera source types for multi-sensor gimbal cameras
     * 
     * @note One-to-one mapping with MAVLink CAMERA_SOURCE enum
     * @note Used by set_camera_source() for type-safe sensor selection
     */
    enum class CameraSource {
        DEFAULT = 0,        ///< Default/primary camera sensor
        RGB = 1,            ///< RGB visible light sensor
        IR = 2,             ///< Infrared/thermal sensor
        NDVI = 3,           ///< Normalized Difference Vegetation Index sensor
        RGB_WIDEANGLE = 4,  ///< Wide-angle RGB sensor
    };
    
    /**
     * @brief Set primary and secondary camera sources on specific instance
     * 
     * @param[in] instance         Camera instance number (0 to AP_CAMERA_MAX_INSTANCES-1)
     * @param[in] primary_source   Primary sensor to use (CameraSource enum)
     * @param[in] secondary_source Secondary sensor for picture-in-picture (CameraSource enum)
     * 
     * @return bool true if backend supports and accepted source selection, false otherwise
     * 
     * @details Multi-sensor camera control (type-safe alternative to set_lens):
     *          - Selects primary camera sensor by type (RGB, IR, NDVI, etc.)
     *          - Optionally selects secondary sensor for picture-in-picture
     *          - Maps to MAVLink Camera Protocol v2 SET_CAMERA_SOURCE command
     *          - Backend-specific implementation
     *          - Used with multi-sensor gimbal cameras
     * 
     * @note Functionally equivalent to set_lens() but with type-safe enum
     * @note Requires AP_CAMERA_SET_CAMERA_SOURCE_ENABLED compile flag
     * @note Backend capability required - multi-sensor gimbal cameras only
     * @note Invalid instance or unsupported backend returns false
     * 
     * @see CameraSource enum for available sensor types
     * @see set_lens() for numeric lens selection alternative
     */
    bool set_camera_source(uint8_t instance, CameraSource primary_source, CameraSource secondary_source);
#endif

    /**
     * @brief Set vehicle AUTO mode flag for distance triggering gate
     * 
     * @param[in] enable true if vehicle is in AUTO mode, false otherwise
     * 
     * @details Updates internal flag used for distance triggering:
     *          - Distance triggering may be gated to AUTO mode only
     *          - Controlled by _auto_mode_only parameter
     *          - Vehicle code must call this to update mode state
     *          - Checked by vehicle_mode_ok_for_trigg_dist()
     * 
     * @note Must be called by vehicle code when flight mode changes
     * @note Interacts with _auto_mode_only parameter setting
     * @note If _auto_mode_only=1, distance triggering only works in AUTO mode
     * 
     * @see vehicle_mode_ok_for_trigg_dist() for AUTO mode checking
     * @see set_trigger_distance() for distance-based triggering
     */
    void set_is_auto_mode(bool enable) { _is_in_auto_mode = enable; }

#if AP_CAMERA_SCRIPTING_ENABLED
    /**
     * @brief Camera state structure for scripting backend integration
     * 
     * @details Provides bidirectional communication between AP_Camera and Lua scripts:
     *          - AP_Camera writes requests to state (take_pic_incr, zoom, focus, tracking)
     *          - Scripting backend reads state and implements camera control
     *          - Scripting backend writes recording_video status back
     *          - Enables custom camera implementations via Lua scripts
     * 
     * @note Requires AP_CAMERA_SCRIPTING_ENABLED compile flag
     * @note Used exclusively by scripting backend (CameraType::SCRIPTING)
     * 
     * @see get_state() to retrieve state from script
     * @see AP_Camera_Scripting backend for implementation
     */
    typedef struct {
        uint16_t take_pic_incr; ///< Incremented each time take_picture() called - script detects change
        bool recording_video;   ///< true when video recording active (set by script)
        uint8_t zoom_type;      ///< ZoomType enum: 1=RATE, 2=PCT
        float zoom_value;       ///< For RATE: -1=out, 0=hold, 1=in. For PCT: 0-100
        uint8_t focus_type;     ///< FocusType enum: 1=RATE, 2=PCT, 4=AUTO
        float focus_value;      ///< For RATE: -1=in, 0=hold, 1=out. For PCT: 0-100
        uint8_t tracking_type;  ///< TrackingType enum: 0=NONE, 1=POINT, 2=RECTANGLE
        Vector2f tracking_p1;   ///< Center point (POINT mode) or top-left (RECTANGLE). x,y in 0-1 range
        Vector2f tracking_p2;   ///< Bottom-right point (RECTANGLE mode only). x,y in 0-1 range
    } camera_state_t;

    /**
     * @brief Retrieve camera state for scripting backend
     * 
     * @param[in]  instance  Camera instance number (0 to AP_CAMERA_MAX_INSTANCES-1)
     * @param[out] cam_state Camera state structure filled with current state
     * 
     * @return bool true if instance is valid scripting backend and state retrieved
     *              false if invalid instance or not a scripting backend
     * 
     * @details Allows Lua scripts to retrieve camera control requests:
     *          - Script polls get_state() periodically
     *          - Detects changes in take_pic_incr to trigger camera
     *          - Reads zoom/focus/tracking commands
     *          - Implements camera control in Lua
     *          - Updates recording_video status
     * 
     * @note Only works with CameraType::SCRIPTING backend
     * @note Requires AP_CAMERA_SCRIPTING_ENABLED compile flag
     * @note Returns false if instance is not a scripting backend
     * 
     * @see camera_state_t structure definition
     * @see AP_Camera_Scripting for backend implementation
     */
    bool get_state(uint8_t instance, camera_state_t& cam_state);

    /**
     * @brief Change camera settings not normally controlled by autopilot
     * 
     * @param[in] instance Camera instance number (0 to AP_CAMERA_MAX_INSTANCES-1)
     * @param[in] setting  Camera setting to modify (CameraSetting enum)
     * @param[in] value    Setting value (semantics depend on specific setting)
     * 
     * @return bool true if backend supports setting modification, false otherwise
     * 
     * @details Modifies advanced camera settings:
     *          - Maps to MAVLink Camera Protocol v2 camera settings
     *          - Typical settings: ISO, aperture, shutter speed, white balance, etc.
     *          - Setting semantics defined by CameraSetting enum
     *          - Value interpretation depends on specific setting
     *          - Backend-specific implementation
     * 
     * @note Requires AP_CAMERA_SCRIPTING_ENABLED compile flag
     * @note Backend capability varies - typically MAVLink Camera v2 only
     * @note Invalid instance or unsupported backend returns false
     * 
     * @see CameraSetting enum in AP_Camera_shareddefs.h for available settings
     * @see AP_Camera_Backend::change_setting() for backend implementation
     */
    bool change_setting(uint8_t instance, CameraSetting setting, float value);
#endif

#if AP_CAMERA_INFO_FROM_SCRIPT_ENABLED
    /**
     * @brief Inject CAMERA_INFORMATION message from Lua script for primary camera
     * 
     * @param[in] camera_info MAVLink CAMERA_INFORMATION message structure
     * 
     * @details Allows Lua scripts to provide camera information:
     *          - Scripts populate camera_info structure with capabilities
     *          - Information sent to GCS via send_camera_information()
     *          - Used for emulating cameras or custom camera implementations
     *          - Includes vendor name, model, capabilities, resolution, FOV
     * 
     * @note Affects primary camera instance
     * @note Requires AP_CAMERA_INFO_FROM_SCRIPT_ENABLED compile flag
     * @note Typically used with CameraType::SCRIPTING backend
     * @note camera_info structure follows MAVLink CAMERA_INFORMATION message format
     * 
     * @see set_camera_information(uint8_t instance, ...) for specific instance
     * @see mavlink_camera_information_t in MAVLink headers
     */
    void set_camera_information(mavlink_camera_information_t camera_info);
    
    /**
     * @brief Inject CAMERA_INFORMATION for specific camera instance
     * 
     * @param[in] instance    Camera instance number (0 to AP_CAMERA_MAX_INSTANCES-1)
     * @param[in] camera_info MAVLink CAMERA_INFORMATION structure
     * 
     * @note Invalid instance numbers are safely ignored
     */
    void set_camera_information(uint8_t instance, mavlink_camera_information_t camera_info);

    /**
     * @brief Inject VIDEO_STREAM_INFORMATION message from Lua script for primary camera
     * 
     * @param[in] camera_info MAVLink VIDEO_STREAM_INFORMATION message structure
     * 
     * @details Allows Lua scripts to provide video stream information:
     *          - Scripts populate structure with stream details
     *          - Information sent to GCS via send_video_stream_information()
     *          - Used for advertising video streaming capabilities
     *          - Includes stream URI, resolution, bitrate, rotation
     * 
     * @note Affects primary camera instance
     * @note Requires AP_CAMERA_INFO_FROM_SCRIPT_ENABLED compile flag
     * @note Typically used with CameraType::SCRIPTING backend
     * @note Structure follows MAVLink VIDEO_STREAM_INFORMATION message format
     * 
     * @see set_stream_information(uint8_t instance, ...) for specific instance
     * @see mavlink_video_stream_information_t in MAVLink headers
     */
    void set_stream_information(mavlink_video_stream_information_t camera_info);
    
    /**
     * @brief Inject VIDEO_STREAM_INFORMATION for specific camera instance
     * 
     * @param[in] instance    Camera instance number (0 to AP_CAMERA_MAX_INSTANCES-1)
     * @param[in] camera_info MAVLink VIDEO_STREAM_INFORMATION structure
     * 
     * @note Invalid instance numbers are safely ignored
     */
    void set_stream_information(uint8_t instance, mavlink_video_stream_information_t camera_info);
#endif // AP_CAMERA_INFO_FROM_SCRIPT_ENABLED

    /**
     * @brief Get legacy relay index for relay function conversion
     * 
     * @param[out] index Relay index (0-based) if relay backend selected
     * 
     * @return bool true if relay camera backend is selected and index is valid
     *              false if no relay backend or relay not configured
     * 
     * @details Migration utility for legacy relay configuration:
     *          - Returns relay index if CameraType::RELAY backend in use
     *          - Used for converting old relay camera configuration to relay functions
     *          - Assists with parameter migration and backward compatibility
     *          - Only applicable when AP_CAMERA_RELAY_ENABLED is defined
     * 
     * @note Returns false if camera backend is not RELAY type
     * @note Used primarily for parameter conversion and migration
     * 
     * @see CameraType::RELAY for relay backend
     */
    bool get_legacy_relay_index(int8_t &index) const;

    /**
     * @brief Get HAL semaphore for thread-safe multi-threaded access
     * 
     * @return HAL_Semaphore& Reference to internal semaphore (_rsem)
     * 
     * @details Thread-safety mechanism:
     *          - Protects AP_Camera state from concurrent access
     *          - Use WITH_SEMAPHORE(_rsem) macro for automatic lock/unlock
     *          - Required when accessing camera from multiple threads
     *          - Prevents race conditions in multi-threaded environments
     * 
     * @note Always use WITH_SEMAPHORE() pattern for thread-safe access
     * @note Example: WITH_SEMAPHORE(AP::camera()->get_semaphore());
     * 
     * @warning Multi-threaded access without semaphore can cause data corruption
     * @warning Do not hold semaphore for extended periods (causes scheduling delays)
     * 
     * @see HAL_Semaphore in AP_HAL for semaphore implementation
     */
    HAL_Semaphore &get_semaphore() { return _rsem; }

    /**
     * @brief AP_Param parameter table for persistent parameter storage
     * 
     * @details Defines CAMx_ parameters stored in EEPROM:
     *          - CAM_AUTO_ONLY: Distance trigger AUTO mode gate (_auto_mode_only)
     *          - CAM_MAX_ROLL: Maximum roll angle for triggering (_max_roll)
     *          - Per-instance parameters in _params[] array (CAM1_*, CAM2_*)
     *          - RunCam backend parameters via _backend_var_info[] (if enabled)
     * 
     * @note Parameters registered with AP_Param system during object construction
     * @note CAMx_ prefix used for all camera parameters
     * 
     * @see AP_Param::GroupInfo for parameter system
     * @see AP_Camera_Params for per-instance parameter structure
     */
    static const struct AP_Param::GroupInfo var_info[];

protected:

    /**
     * @brief Check if vehicle mode allows distance-based triggering
     * 
     * @return bool true if triggering allowed, false if gated by AUTO mode requirement
     * 
     * @note Returns true if _auto_mode_only=0 (no restriction)
     * @note Returns true if _auto_mode_only=1 AND _is_in_auto_mode=true
     * @note Used internally by update() to gate distance triggering
     */
    bool vehicle_mode_ok_for_trigg_dist() const { return (_auto_mode_only == 0) || _is_in_auto_mode; }

    /**
     * @brief Get maximum acceptable vehicle roll angle for triggering
     * 
     * @return int16_t Maximum roll angle in degrees (from _max_roll parameter)
     * 
     * @note Used to prevent triggering when vehicle is banked excessively
     * @note Ensures images are captured when vehicle is relatively level
     */
    int16_t get_roll_max() const { return _max_roll; }

    /**
     * @brief Get logging bit mask for camera logging
     * 
     * @return uint32_t Bit from LOG_BITMASK parameter enabling camera logging
     * 
     * @note When this bit is set in LOG_BITMASK, CAM and TRIG messages are logged
     * @note Passed to constructor during initialization
     */
    uint32_t get_log_camera_bit() const { return log_camera_bit; }

    /// Per-instance parameter structures (CAM1_*, CAM2_* parameters)
    AP_Camera_Params _params[AP_CAMERA_MAX_INSTANCES];
#if AP_CAMERA_RUNCAM_ENABLED
    /// Var info table pointers for RunCam backend parameters (AP_CAMERA_RUNCAM_ENABLED only)
    static const struct AP_Param::GroupInfo *_backend_var_info[AP_CAMERA_MAX_INSTANCES];
    /// Number of instantiated RunCam backends
    uint8_t _runcam_instances;
#endif

private:

    /// Singleton instance pointer (only one AP_Camera exists)
    static AP_Camera *_singleton;

    /// Parameters
    AP_Int8 _auto_mode_only;    ///< CAM_AUTO_ONLY: if 1, distance triggering only works in AUTO mode
    AP_Int16 _max_roll;         ///< CAM_MAX_ROLL: Maximum roll angle (degrees) allowed when triggering

    /**
     * @brief Validate instance number and return backend pointer
     * 
     * @param[in] instance Camera instance number to check
     * 
     * @return AP_Camera_Backend* Pointer to backend if valid, nullptr if invalid
     * 
     * @note Returns nullptr for out-of-range or uninitialized instances
     */
    AP_Camera_Backend *get_instance(uint8_t instance) const;

    /**
     * @brief Perform legacy parameter conversion for backward compatibility
     * 
     * @details Converts old parameter names/structures to current format:
     *          - Called during init() before backend instantiation
     *          - Ensures parameters from old firmware versions work correctly
     *          - One-time conversion on first boot with new firmware
     * 
     * @note Called automatically by init()
     */
    void convert_params();

    /**
     * @brief Send CAMERA_FEEDBACK message to GCS with geotagging data
     * 
     * @param[in] chan MAVLink channel to send message on
     * 
     * @details Generates MAVLink CAMERA_FEEDBACK message including:
     *          - GPS position (latitude, longitude, altitude MSL and relative)
     *          - GPS time (milliseconds since GPS week start)
     *          - Vehicle attitude (roll, pitch, yaw in degrees)
     *          - Camera index and trigger count
     *          - Used for image geotagging and photogrammetry
     * 
     * @note Called automatically when camera triggers
     */
    void send_feedback(mavlink_channel_t chan);

    /**
     * @brief Send CAMERA_INFORMATION message to GCS
     * 
     * @param[in] chan MAVLink channel to send message on
     * 
     * @details Generates MAVLink CAMERA_INFORMATION message including:
     *          - Camera capabilities (photo, video, zoom, focus, tracking)
     *          - Vendor name and model name
     *          - Sensor resolution and FOV (horizontal/vertical)
     *          - Firmware version
     *          - Part of MAVLink Camera Protocol v2
     * 
     * @note Sent in response to MSG_CAMERA_INFORMATION stream request
     * @note Backend provides information via get_camera_information()
     */
    void send_camera_information(mavlink_channel_t chan);

#if AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED
    /**
     * @brief Send VIDEO_STREAM_INFORMATION message to GCS
     * 
     * @param[in] chan MAVLink channel to send message on
     * 
     * @details Generates MAVLink VIDEO_STREAM_INFORMATION message including:
     *          - Stream ID and type (RTSP, TCP, UDP, etc.)
     *          - Stream URI/name
     *          - Resolution (width x height)
     *          - Framerate and bitrate
     *          - Horizontal/vertical FOV
     *          - Part of MAVLink Camera Protocol v2 for video streaming
     * 
     * @note Sent in response to MSG_VIDEO_STREAM_INFORMATION stream request
     * @note Backend provides information via get_video_stream_information()
     * @note Compile-time conditional on AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED
     */
    void send_video_stream_information(mavlink_channel_t chan);
#endif // AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED

    /**
     * @brief Send CAMERA_SETTINGS message to GCS
     * 
     * @param[in] chan MAVLink channel to send message on
     * 
     * @details Generates MAVLink CAMERA_SETTINGS message including:
     *          - Current camera mode (photo vs video)
     *          - Zoom level (absolute position or rate)
     *          - Focus level (absolute position or rate)
     *          - Part of MAVLink Camera Protocol v2
     * 
     * @note Sent in response to MSG_CAMERA_SETTINGS stream request
     * @note Backend provides current settings via get_camera_settings()
     */
    void send_camera_settings(mavlink_channel_t chan);

#if AP_CAMERA_SEND_FOV_STATUS_ENABLED
    /**
     * @brief Send CAMERA_FOV_STATUS message to GCS
     * 
     * @param[in] chan MAVLink channel to send message on
     * 
     * @details Generates MAVLink CAMERA_FOV_STATUS message including:
     *          - Current latitude/longitude pointing direction
     *          - Horizontal/vertical FOV in degrees
     *          - Quaternion orientation of camera
     *          - Useful for gimbal-mounted cameras with variable pointing
     * 
     * @note Sent in response to MSG_CAMERA_FOV_STATUS stream request
     * @note Requires gimbal position from AP::mount()
     * @note Compile-time conditional on AP_CAMERA_SEND_FOV_STATUS_ENABLED
     */
    void send_camera_fov_status(mavlink_channel_t chan);
#endif

    /**
     * @brief Send CAMERA_CAPTURE_STATUS message to GCS
     * 
     * @param[in] chan MAVLink channel to send message on
     * 
     * @details Generates MAVLink CAMERA_CAPTURE_STATUS message including:
     *          - Image capture status (idle vs capturing)
     *          - Image capture interval in seconds
     *          - Recording time in seconds (if video recording)
     *          - Available storage capacity in MiB
     *          - Part of MAVLink Camera Protocol v2
     * 
     * @note Sent in response to MSG_CAMERA_CAPTURE_STATUS stream request
     * @note Backend provides status via get_capture_status()
     */
    void send_camera_capture_status(mavlink_channel_t chan);

#if AP_CAMERA_SEND_THERMAL_RANGE_ENABLED
    /**
     * @brief Send CAMERA_THERMAL_RANGE message to GCS
     * 
     * @param[in] chan MAVLink channel to send message on
     * 
     * @details Generates MAVLink CAMERA_THERMAL_RANGE message including:
     *          - Minimum thermal temperature in scene (Celsius)
     *          - Maximum thermal temperature in scene (Celsius)
     *          - Useful for thermal cameras to display temperature scale
     *          - Part of MAVLink Camera Protocol v2 for thermal imaging
     * 
     * @note Sent in response to MSG_CAMERA_THERMAL_RANGE stream request
     * @note Backend provides range via get_thermal_range()
     * @note Compile-time conditional on AP_CAMERA_SEND_THERMAL_RANGE_ENABLED
     */
    void send_camera_thermal_range(mavlink_channel_t chan);
#endif

    HAL_Semaphore _rsem;                ///< Semaphore for thread-safe multi-threaded access (use WITH_SEMAPHORE)
    AP_Camera_Backend *primary;         ///< Primary camera backend pointer (typically instance 0)
    bool _is_in_auto_mode;              ///< True when vehicle is in AUTO flight mode (set via set_is_auto_mode)
    uint32_t log_camera_bit;            ///< Logging bit from LOG_BITMASK to enable CAM/TRIG message logging
    AP_Camera_Backend *_backends[AP_CAMERA_MAX_INSTANCES];  ///< Array of backend instance pointers (nullptr if not instantiated)
};

namespace AP {
    /**
     * @brief Accessor for AP_Camera singleton via AP namespace
     * 
     * @return AP_Camera* Pointer to singleton AP_Camera instance
     * 
     * @note Equivalent to AP_Camera::get_singleton()
     * @note Returns nullptr if camera system not initialized
     * @note Thread-safe singleton access pattern
     * 
     * @see AP_Camera::get_singleton()
     */
    AP_Camera *camera();
};

#endif
