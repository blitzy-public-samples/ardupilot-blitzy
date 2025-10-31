/**
 * @file AP_Mount_Siyi.h
 * @brief Siyi gimbal driver using custom serial protocol backend
 * 
 * @details This driver implements the Siyi custom serial protocol for gimbal control
 *          and camera operations. The protocol uses a binary packet format with CRC16
 *          verification for reliable communication.
 * 
 *          Packet format (courtesy of Siyi's SDK document):
 * 
 *          -------------------------------------------------------------------------------------------
 *          Field     Index   Bytes       Description
 *          -------------------------------------------------------------------------------------------
 *          STX       0       2           0x5566: starting mark
 *          CTRL      2       1           bit 0: need_ack.  set if the current data packet needs ack
 *                                        bit 1: ack_pack.  set if the current data packate IS an ack
 *                                        bit 2-7: reserved
 *          Data_len  3       2           Data field byte length.  Low byte in the front
 *          SEQ       5       2           Frame sequence (0 ~ 65535).  Low byte in the front.  May be used to detect packet loss
 *          CMD_ID    7       1           Command ID
 *          DATA      8       Data_len    Data
 *          CRC16             2           CRC16 check the complete data package.  Low byte in the front
 * 
 * @note Protocol communication rates:
 *       - Attitude updates: 20Hz
 *       - Rangefinder updates: 10Hz
 *       - Thermal data updates: 5Hz
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_SIYI_ENABLED

#include "AP_Mount_Backend_Serial.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>

#define AP_MOUNT_SIYI_PACKETLEN_MAX     42  // maximum number of bytes in a packet sent to or received from the gimbal

/**
 * @class AP_Mount_Siyi
 * @brief Siyi gimbal serial protocol implementation with camera controls and rangefinder support
 * 
 * @details This class implements the Siyi gimbal protocol providing:
 *          - 3-axis gimbal control (roll, pitch, yaw)
 *          - Camera controls (photo, video, zoom, focus)
 *          - Thermal imaging support (palette, gain, raw data)
 *          - Integrated rangefinder/laser distance measurement
 *          - Multiple lens control for ZT30 models
 * 
 *          Supported hardware models:
 *          - A2: Entry-level gimbal
 *          - A8: Advanced gimbal with zoom
 *          - ZR10: Zoom camera gimbal (10x optical zoom)
 *          - ZR30: Zoom camera gimbal (30x optical zoom)
 *          - ZT6: Thermal/zoom hybrid gimbal
 *          - ZT30: Advanced thermal/zoom hybrid gimbal with multi-lens support
 * 
 *          Gimbal motion modes:
 *          - FOLLOW: Roll and pitch are earth-frame, yaw is body-frame
 *          - LOCK: All axes (roll, pitch, yaw) are earth-frame
 *          - FPV: All axes (roll, pitch, yaw) are body-frame
 * 
 * @note Angles are in radians, rates are in rad/s
 * @note Hardware model is auto-detected via HARDWARE_ID command
 * 
 * @warning Requires compatible firmware version for all features
 * @warning Thermal palette and gain settings affect image output characteristics
 */
class AP_Mount_Siyi : public AP_Mount_Backend_Serial
{

public:
    // Constructor
    using AP_Mount_Backend_Serial::AP_Mount_Backend_Serial;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Mount_Siyi);

    /**
     * @brief Update mount position and process incoming packets - called periodically
     * 
     * @details This method handles:
     *          - Processing incoming packets from gimbal
     *          - Sending attitude requests at 20Hz
     *          - Sending rangefinder requests at 10Hz
     *          - Sending thermal data requests at 5Hz
     *          - Updating gimbal control commands
     *          - Processing zoom control
     * 
     * @note Called from main scheduler loop
     */
    void update() override;

    /**
     * @brief Return true if gimbal communication is healthy
     * 
     * @details Health check verifies:
     *          - Serial communication is active
     *          - Recent attitude data received from gimbal
     *          - Gimbal responding to commands
     * 
     * @return true if gimbal is healthy and communicating
     */
    bool healthy() const override;

    /**
     * @brief Check if this mount accepts roll control targets
     * 
     * @return false - Siyi gimbals do not support independent roll control
     */
    bool has_roll_control() const override { return false; }

    /**
     * @brief Check if this mount can control its pan/yaw (required for multicopters)
     * 
     * @return true if yaw range is valid and configured
     */
    bool has_pan_control() const override { return yaw_range_valid(); };

    //
    // camera controls
    //

    /**
     * @brief Take a picture with the camera
     * 
     * @details Sends PHOTO command (0x0C) with PhotoFunction::TAKE_PICTURE to gimbal
     * 
     * @return true on success (command sent), false if serial buffer full
     */
    bool take_picture() override;

    /**
     * @brief Start or stop video recording
     * 
     * @details Sends PHOTO command (0x0C) with PhotoFunction::RECORD_VIDEO_TOGGLE to gimbal
     * 
     * @param[in] start_recording true to start recording, false to stop recording
     * 
     * @return true on success (command sent), false if serial buffer full
     */
    bool record_video(bool start_recording) override;

    /**
     * @brief Set camera zoom specified as a rate or percentage
     * 
     * @details Zoom control supports:
     *          - Rate control: continuous zoom in/out at specified rate
     *          - Percentage/multiple control: absolute zoom level (e.g., 1x, 10x, 30x)
     * 
     * @param[in] zoom_type Type of zoom control (RATE or ABSOLUTE)
     * @param[in] zoom_value For RATE: -1.0 (zoom out) to +1.0 (zoom in), 0=hold
     *                       For ABSOLUTE: zoom multiple (e.g., 1.0 to 30.0)
     * 
     * @return true on success, false if command failed or unsupported
     */
    bool set_zoom(ZoomType zoom_type, float zoom_value) override;

    /**
     * @brief Set camera focus specified as rate, percentage or auto
     * 
     * @details Focus control modes:
     *          - AUTO: Enable autofocus
     *          - RATE: Manual focus control at specified rate
     * 
     * @param[in] focus_type Type of focus control (AUTO or RATE)
     * @param[in] focus_value For RATE: -1.0 (focus in) to +1.0 (focus out), 0=hold
     * 
     * @return SetFocusResult indicating success, failure, or unsupported operation
     */
    SetFocusResult set_focus(FocusType focus_type, float focus_value) override;

    /**
     * @brief Set camera lens/image type (ZT30 only)
     * 
     * @details Lens values 0-8 correspond to CameraImageType enum:
     *          0: Main PIP zoom+thermal, Sub wideangle
     *          1: Main PIP wideangle+thermal, Sub zoom
     *          2: Main PIP zoom+wideangle, Sub thermal
     *          3: Main zoom, Sub thermal
     *          4: Main zoom, Sub wideangle
     *          5: Main wideangle, Sub thermal
     *          6: Main wideangle, Sub zoom
     *          7: Main thermal, Sub zoom
     *          8: Main thermal, Sub wideangle
     * 
     * @param[in] lens Lens configuration value (0-8)
     * 
     * @return true on success, false if unsupported or invalid lens value
     * 
     * @note Only supported on ZT30 hardware model
     */
    bool set_lens(uint8_t lens) override;

    /**
     * @brief Set camera source (functionally same as set_lens but uses camera source types)
     * 
     * @details Maps AP_Camera::CameraSource enum to CameraImageType for ZT30 multi-lens control
     * 
     * @param[in] primary_source Primary camera source (AP_Camera::CameraSource enum cast to uint8_t)
     * @param[in] secondary_source Secondary camera source (AP_Camera::CameraSource enum cast to uint8_t)
     * 
     * @return true on success, false if unsupported or invalid source combination
     * 
     * @note Only supported on ZT30 hardware model
     */
    bool set_camera_source(uint8_t primary_source, uint8_t secondary_source) override;

    /**
     * @brief Send camera information message to GCS via MAVLink
     * 
     * @details Sends CAMERA_INFORMATION message containing:
     *          - Camera capabilities (photo, video, zoom)
     *          - Firmware version
     *          - Hardware model name
     * 
     * @param[in] chan MAVLink channel to send message on
     */
    void send_camera_information(mavlink_channel_t chan) const override;

    /**
     * @brief Send camera settings message to GCS via MAVLink
     * 
     * @details Sends CAMERA_SETTINGS message containing:
     *          - Current recording status
     *          - Current zoom level
     *          - HDR status
     * 
     * @param[in] chan MAVLink channel to send message on
     */
    void send_camera_settings(mavlink_channel_t chan) const override;

#if AP_MOUNT_SEND_THERMAL_RANGE_ENABLED
    /**
     * @brief Send camera thermal range message to GCS via MAVLink
     * 
     * @details Sends CAMERA_THERMAL_RANGE message containing:
     *          - Maximum temperature and pixel position
     *          - Minimum temperature and pixel position
     * 
     * @param[in] chan MAVLink channel to send message on
     * 
     * @note Thermal data updated at 5Hz from gimbal
     */
    void send_camera_thermal_range(mavlink_channel_t chan) const override;
#endif

    /**
     * @brief Change camera settings not normally used by autopilot
     * 
     * @details Supported settings for thermal imaging:
     *          THERMAL_PALETTE values:
     *            0: WhiteHot   - Hot areas white, cold areas black
     *            2: Sepia      - Sepia tone thermal image
     *            3: IronBow    - Iron-bow color palette
     *            4: Rainbow    - Rainbow color palette
     *            5: Night      - Night vision optimized palette
     *            6: Aurora     - Aurora color palette
     *            7: RedHot     - Hot areas red
     *            8: Jungle     - Jungle camouflage palette
     *            9: Medical    - Medical imaging palette
     *            10: BlackHot  - Hot areas black, cold areas white
     *            11: GloryHot  - Glory color palette
     * 
     *          THERMAL_GAIN values:
     *            0: Low gain  - Temperature range 50°C to 550°C
     *            1: High gain - Temperature range -20°C to 150°C
     * 
     *          THERMAL_RAW_DATA values:
     *            0: Disable Raw Data - 30fps output
     *            1: Enable Raw Data  - 25fps output
     * 
     * @param[in] setting Camera setting to change (from CameraSetting enum)
     * @param[in] value New value for the setting
     * 
     * @return true on success (command sent), false if unsupported or failed
     * 
     * @warning Changing thermal settings affects image processing and frame rate
     */
    bool change_setting(CameraSetting setting, float value) override;

    //
    // rangefinder
    //

    /**
     * @brief Get rangefinder/laser distance measurement
     * 
     * @details Retrieves last received distance measurement from gimbal's integrated
     *          rangefinder/laser. Distance is updated at 10Hz from gimbal.
     * 
     * @param[out] distance_m Distance in meters
     * 
     * @return true if valid distance available, false if no recent data or out of range
     * 
     * @note Rangefinder availability depends on hardware model
     */
    bool get_rangefinder_distance(float& distance_m) const override;

protected:

    /**
     * @brief Get gimbal attitude as a quaternion
     * 
     * @details Converts current gimbal angles (roll, pitch, yaw in radians) to
     *          quaternion representation for attitude control.
     * 
     * @param[out] att_quat Attitude quaternion (NED frame)
     * 
     * @return true if valid attitude data available, false if data too old
     * 
     * @note Attitude data received from gimbal at 20Hz
     */
    bool get_attitude_quaternion(Quaternion& att_quat) override;

    /**
     * @brief Get angular velocity of gimbal mount
     * 
     * @details Returns current gimbal angular rates in rad/s for all axes
     * 
     * @param[out] rates Angular velocity vector in rad/s (x=roll, y=pitch, z=yaw)
     * 
     * @return true (always successful - returns last known rates)
     */
    bool get_angular_velocity(Vector3f& rates) override {
        rates = _current_rates_rads;
        return true;
    }
    
private:

    /**
     * @brief Siyi serial protocol command IDs
     * 
     * @details Command IDs used in CMD_ID field of serial protocol packets.
     *          Values range from 0x01 to 0x3e for various gimbal and camera operations.
     */
    enum class SiyiCommandId {
        ACQUIRE_FIRMWARE_VERSION = 0x01,        ///< Request firmware version (camera, gimbal, zoom)
        HARDWARE_ID = 0x02,                     ///< Request hardware model ID
        AUTO_FOCUS = 0x04,                      ///< Enable autofocus
        MANUAL_ZOOM_AND_AUTO_FOCUS = 0x05,      ///< Set zoom rate and enable autofocus
        MANUAL_FOCUS = 0x06,                    ///< Set manual focus rate
        GIMBAL_ROTATION = 0x07,                 ///< Send gimbal rotation rate commands
        CENTER = 0x08,                          ///< Center gimbal to neutral position
        ACQUIRE_GIMBAL_CONFIG_INFO = 0x0A,      ///< Request gimbal configuration (HDR, recording, motion mode)
        FUNCTION_FEEDBACK_INFO = 0x0B,          ///< Request function feedback status
        PHOTO = 0x0C,                           ///< Photo/video control (take picture, record, HDR, motion mode)
        ACQUIRE_GIMBAL_ATTITUDE = 0x0D,         ///< Request gimbal attitude angles and rates
        ABSOLUTE_ZOOM = 0x0F,                   ///< Set absolute zoom multiple
        SET_CAMERA_IMAGE_TYPE = 0x11,           ///< Set camera lens/image type (ZT30)
        GET_TEMP_FULL_IMAGE = 0x14,             ///< Request thermal image min/max temperatures
        READ_RANGEFINDER = 0x15,                ///< Request rangefinder distance
        SET_THERMAL_PALETTE = 0x1B,             ///< Set thermal imaging color palette
        EXTERNAL_ATTITUDE = 0x22,               ///< Send vehicle attitude to gimbal (for stabilization)
        SET_TIME = 0x30,                        ///< Send system time to gimbal
        SET_THERMAL_RAW_DATA = 0x34,            ///< Enable/disable thermal raw data output
        SET_THERMAL_GAIN = 0x38,                ///< Set thermal sensor gain (low/high)
        POSITION_DATA = 0x3e,                   ///< Send vehicle position to gimbal
    };

    /**
     * @brief Function Feedback Info packet info_type values
     * 
     * @details Status codes returned in FUNCTION_FEEDBACK_INFO responses
     */
    enum class FunctionFeedbackInfo : uint8_t {
        SUCCESS = 0,                    ///< Operation completed successfully
        FAILED_TO_TAKE_PHOTO = 1,       ///< Photo capture failed
        HDR_ON = 2,                     ///< HDR mode enabled
        HDR_OFF = 3,                    ///< HDR mode disabled
        FAILED_TO_RECORD_VIDEO = 4      ///< Video recording failed
    };

    /**
     * @brief Photo Function packet func_type values
     * 
     * @details Function codes for PHOTO command (0x0C) operations
     */
    enum class PhotoFunction : uint8_t {
        TAKE_PICTURE = 0,           ///< Trigger camera shutter to take photo
        HDR_TOGGLE = 1,             ///< Toggle HDR mode on/off
        RECORD_VIDEO_TOGGLE = 2,    ///< Toggle video recording on/off
        LOCK_MODE = 3,              ///< Set gimbal to LOCK motion mode (all axes earth-frame)
        FOLLOW_MODE = 4,            ///< Set gimbal to FOLLOW motion mode (roll/pitch earth-frame, yaw body-frame)
        FPV_MODE = 5                ///< Set gimbal to FPV motion mode (all axes body-frame)
    };

    /**
     * @brief Serial packet parsing state machine states
     * 
     * @details Parser state for processing incoming serial protocol packets
     */
    enum class ParseState : uint8_t {
        WAITING_FOR_HEADER_LOW,     ///< Waiting for STX low byte (0x66)
        WAITING_FOR_HEADER_HIGH,    ///< Waiting for STX high byte (0x55)
        WAITING_FOR_CTRL,           ///< Waiting for CTRL byte
        WAITING_FOR_DATALEN_LOW,    ///< Waiting for Data_len low byte
        WAITING_FOR_DATALEN_HIGH,   ///< Waiting for Data_len high byte
        WAITING_FOR_SEQ_LOW,        ///< Waiting for SEQ low byte
        WAITING_FOR_SEQ_HIGH,       ///< Waiting for SEQ high byte
        WAITING_FOR_CMDID,          ///< Waiting for CMD_ID byte
        WAITING_FOR_DATA,           ///< Waiting for DATA bytes
        WAITING_FOR_CRC_LOW,        ///< Waiting for CRC16 low byte
        WAITING_FOR_CRC_HIGH,       ///< Waiting for CRC16 high byte
    };

    /**
     * @brief Hardware model identification
     * 
     * @details Gimbal hardware models detected via HARDWARE_ID command.
     *          Different models have different capabilities (zoom range, thermal, etc.)
     */
    enum class HardwareModel : uint8_t {
        UNKNOWN = 0,    ///< Hardware model not yet detected
        A2,             ///< A2 entry-level gimbal
        A8,             ///< A8 advanced gimbal with zoom
        ZR10,           ///< ZR10 zoom camera gimbal (10x optical)
        ZR30,           ///< ZR30 zoom camera gimbal (30x optical)
        ZT6,            ///< ZT6 thermal/zoom hybrid gimbal
        ZT30            ///< ZT30 advanced thermal/zoom hybrid with multi-lens
    } _hardware_model;

    /**
     * @brief HDR (High Dynamic Range) status
     */
    enum class HdrStatus : uint8_t {
        OFF = 0,        ///< HDR disabled
        ON  = 1,        ///< HDR enabled
    };

    /**
     * @brief Video recording status
     */
    enum class RecordingStatus : uint8_t {
        OFF       = 0,  ///< Recording stopped
        ON        = 1,  ///< Recording active
        NO_CARD   = 2,  ///< No SD card present
        DATA_LOSS = 3,  ///< Data loss detected during recording
    };

    /**
     * @brief Gimbal motion mode defines coordinate frame for gimbal control
     * 
     * @details Motion modes affect how gimbal interprets angle/rate commands:
     *          - LOCK: All axes earth-frame (gimbal maintains absolute heading)
     *          - FOLLOW: Roll/pitch earth-frame, yaw body-frame (gimbal follows vehicle yaw)
     *          - FPV: All axes body-frame (gimbal locked to vehicle orientation)
     */
    enum class GimbalMotionMode : uint8_t {
        LOCK   = 0,     ///< All axes earth-frame (absolute stabilization)
        FOLLOW = 1,     ///< Roll/pitch earth-frame, yaw body-frame (yaw follows vehicle)
        FPV    = 2,     ///< All axes body-frame (locked to vehicle)
    };

    /**
     * @brief Gimbal mounting direction/orientation
     */
    enum class GimbalMountingDirection : uint8_t {
        UNDEFINED = 0,      ///< Mounting direction not configured
        NORMAL = 1,         ///< Normal mounting (camera facing down/forward)
        UPSIDE_DOWN = 2,    ///< Inverted mounting
    };

    /**
     * @brief Video output interface type
     */
    enum class VideoOutputStatus : uint8_t {
        HDMI = 0,       ///< HDMI video output active
        CVBS = 1,       ///< CVBS (analog) video output active
    };

    // Response message for "Acquire Gimbal Confuguration Information" (0x0A)
    typedef struct {
        uint8_t _reserved1;
        HdrStatus hdr_status;
        uint8_t _reserved3;
        RecordingStatus record_status;
        GimbalMotionMode motion_mode;
        GimbalMountingDirection mounting_dir;
        VideoOutputStatus video_mode;
    } GimbalConfigInfo;
    static_assert(sizeof(GimbalConfigInfo) == 7, "GimbalConfigInfo must be 7 bytes");

    /**
     * @brief Camera image types (lens configurations for ZT30)
     * 
     * @details Defines video output configuration for multi-lens ZT30 gimbal:
     *          - MAIN: Primary video stream
     *          - SUB: Secondary video stream (pip or separate)
     *          - PIP: Picture-in-picture overlay mode
     *          - ZOOM: Optical zoom camera
     *          - WIDEANGLE: Wide-angle camera
     *          - THERMAL: Thermal imaging camera
     * 
     * @note Only applicable to ZT30 hardware model
     */
    enum class CameraImageType : uint8_t {
        MAIN_PIP_ZOOM_THERMAL_SUB_WIDEANGLE = 0,    ///< Main: zoom+thermal PIP, Sub: wideangle
        MAIN_PIP_WIDEANGLE_THERMAL_SUB_ZOOM = 1,    ///< Main: wideangle+thermal PIP, Sub: zoom
        MAIN_PIP_ZOOM_WIDEANGLE_SUB_THERMAL = 2,    ///< Main: zoom+wideangle PIP, Sub: thermal
        MAIN_ZOOM_SUB_THERMAL = 3,                  ///< Main: zoom, Sub: thermal
        MAIN_ZOOM_SUB_WIDEANGLE = 4,                ///< Main: zoom, Sub: wideangle
        MAIN_WIDEANGLE_SUB_THERMAL = 5,             ///< Main: wideangle, Sub: thermal
        MAIN_WIDEANGLE_SUB_ZOOM = 6,                ///< Main: wideangle, Sub: zoom
        MAIN_THERMAL_SUB_ZOOM = 7,                  ///< Main: thermal, Sub: zoom
        MAIN_THERMAL_SUB_WIDEANGLE = 8              ///< Main: thermal, Sub: wideangle
    };

    typedef struct {
        uint8_t major;
        uint8_t minor;
        uint8_t patch;
    } Version;
    typedef struct {
        Version camera;
        Version gimbal;
        Version zoom;
        bool received; // true once version information has been received
    } FirmwareVersion;

    /**
     * @brief Read and parse incoming packets from gimbal serial port
     * 
     * @details Implements state machine parser for Siyi protocol packets:
     *          - Reads bytes from serial buffer
     *          - Validates packet structure (STX, length, CRC16)
     *          - Stores complete packets in _parsed_msg for processing
     * 
     * @note Called from update() method each loop iteration
     */
    void read_incoming_packets();

    /**
     * @brief Process successfully decoded packets held in _parsed_msg structure
     * 
     * @details Handles responses based on CMD_ID:
     *          - Firmware version responses
     *          - Hardware ID responses
     *          - Gimbal attitude data
     *          - Rangefinder distance data
     *          - Thermal image data
     *          - Configuration info
     *          - Function feedback status
     */
    void process_packet();

    /**
     * @brief Send packet to gimbal via serial port
     * 
     * @details Constructs complete packet with:
     *          - STX header (0x5566)
     *          - CTRL byte
     *          - Data length
     *          - Sequence number
     *          - Command ID
     *          - Data payload
     *          - CRC16 checksum
     * 
     * @param[in] cmd_id Command ID from SiyiCommandId enum
     * @param[in] databuff Pointer to data payload buffer (can be nullptr if databuff_len=0)
     * @param[in] databuff_len Length of data payload in bytes
     * 
     * @return true on success, false if outgoing serial buffer is full
     */
    bool send_packet(SiyiCommandId cmd_id, const uint8_t* databuff, uint8_t databuff_len);

    /**
     * @brief Send single-byte packet to gimbal (convenience wrapper)
     * 
     * @param[in] cmd_id Command ID from SiyiCommandId enum
     * @param[in] data_byte Single data byte to send
     * 
     * @return true on success, false if outgoing serial buffer is full
     */
    bool send_1byte_packet(SiyiCommandId cmd_id, uint8_t data_byte);

    // request info from gimbal
    void request_firmware_version() { send_packet(SiyiCommandId::ACQUIRE_FIRMWARE_VERSION, nullptr, 0); }
    void request_hardware_id() { send_packet(SiyiCommandId::HARDWARE_ID, nullptr, 0); }
    void request_configuration() { send_packet(SiyiCommandId::ACQUIRE_GIMBAL_CONFIG_INFO, nullptr, 0); }
    void request_function_feedback_info() { send_packet(SiyiCommandId::FUNCTION_FEEDBACK_INFO, nullptr, 0); }
    void request_gimbal_attitude() { send_packet(SiyiCommandId::ACQUIRE_GIMBAL_ATTITUDE, nullptr, 0); }
    void request_rangefinder_distance() { send_packet(SiyiCommandId::READ_RANGEFINDER, nullptr, 0); }

    /**
     * @brief Rotate gimbal with pitch and yaw rate commands
     * 
     * @details Sends GIMBAL_ROTATION command with scaled rate values.
     *          Coordinate frame depends on current motion mode.
     * 
     * @param[in] pitch_scalar Pitch rate scalar in range -100 to +100 (negative=down, positive=up)
     * @param[in] yaw_scalar Yaw rate scalar in range -100 to +100 (negative=left, positive=right)
     * @param[in] yaw_is_ef true if gimbal should maintain earth-frame yaw target (LOCK mode),
     *                      false for body-frame yaw (FOLLOW mode)
     * 
     * @note Actual rotation rate depends on gimbal configuration and motion mode
     */
    void rotate_gimbal(int8_t pitch_scalar, int8_t yaw_scalar, bool yaw_is_ef);

    /**
     * @brief Set gimbal's motion mode if changed
     * 
     * @details Motion mode defines coordinate frames for gimbal control:
     *          - FOLLOW: roll and pitch are in earth-frame, yaw is in body-frame
     *          - LOCK: roll, pitch and yaw are all in earth-frame
     *          - FPV: roll, pitch and yaw are all in body-frame
     * 
     * @param[in] mode Desired motion mode
     * @param[in] force If true, always send command even if mode unchanged (default: false)
     * 
     * @return true if message successfully sent to gimbal, false if failed
     * 
     * @note Only sends command if mode differs from current mode (unless force=true)
     */
    bool set_motion_mode(const GimbalMotionMode mode, const bool force=false);

    /**
     * @brief Send target pitch and yaw rates to gimbal
     * 
     * @details Converts rate commands in rad/s to gimbal protocol format
     * 
     * @param[in] pitch_rads Pitch rate in rad/s (negative=down, positive=up)
     * @param[in] yaw_rads Yaw rate in rad/s (negative=left, positive=right)
     * @param[in] yaw_is_ef true if yaw_rads is earth-frame rate (LOCK mode),
     *                      false if body-frame rate (FOLLOW/FPV mode)
     */
    void send_target_rates(float pitch_rads, float yaw_rads, bool yaw_is_ef);

    /**
     * @brief Send target pitch and yaw angles to gimbal
     * 
     * @details Converts angle commands in radians to gimbal protocol format
     * 
     * @param[in] pitch_rad Pitch angle in radians (negative=down, positive=up)
     * @param[in] yaw_rad Yaw angle in radians (negative=left, positive=right)
     * @param[in] yaw_is_ef true if yaw_rad is earth-frame angle (LOCK mode),
     *                      false if body-frame angle (FOLLOW/FPV mode)
     */
    void send_target_angles(float pitch_rad, float yaw_rad, bool yaw_is_ef);

    /**
     * @brief Send zoom rate command to camera
     * 
     * @details Sends continuous zoom command using MANUAL_ZOOM_AND_AUTO_FOCUS
     * 
     * @param[in] zoom_value Zoom rate: -1.0 (zoom out) to +1.0 (zoom in), 0=hold
     * 
     * @return true on success, false if command failed
     */
    bool send_zoom_rate(float zoom_value);

    /**
     * @brief Send absolute zoom multiple command to camera
     * 
     * @details Sends ABSOLUTE_ZOOM command for precise zoom positioning.
     *          Zoom range depends on hardware model (e.g., 1x-30x for ZR30)
     * 
     * @param[in] zoom_mult Desired zoom multiple (e.g., 1.0, 10.0, 30.0)
     * 
     * @return true on success, false if command failed or unsupported
     */
    bool send_zoom_mult(float zoom_mult);

    /**
     * @brief Get maximum zoom multiple for current hardware model
     * 
     * @return Maximum zoom multiple (e.g., 30.0 for ZR30, 6.0 for A8)
     */
    float get_zoom_mult_max() const;

    /**
     * @brief Update zoom controller to process zoom commands
     * 
     * @details Handles zoom control including:
     *          - Rate-based zoom control
     *          - Absolute zoom positioning
     *          - Zoom feedback from gimbal
     * 
     * @note Called from update() method
     */
    void update_zoom_control();

    /**
     * @brief Get model name string for current hardware
     * 
     * @return Pointer to model name string (e.g., "A2", "ZR30"), or nullptr if unknown
     */
    const char* get_model_name() const;

    /**
     * @brief Check that firmware version meets minimum supported version
     * 
     * @details Validates gimbal firmware version and logs warnings if
     *          firmware is too old for full feature support
     * 
     * @note Called after firmware version is received from gimbal
     */
    void check_firmware_version() const;

#if AP_MOUNT_SEND_THERMAL_RANGE_ENABLED
    /**
     * @brief Request thermal min/max temperatures from gimbal
     * 
     * @details Sends GET_TEMP_FULL_IMAGE command at 5Hz to retrieve:
     *          - Maximum temperature and pixel position
     *          - Minimum temperature and pixel position
     * 
     * @note Only applicable to thermal-equipped models (ZT6, ZT30)
     */
    void request_thermal_minmax();
#endif

    // internal variables
    bool _got_hardware_id;                          ///< true once hardware id has been received from gimbal

    FirmwareVersion _fw_version;                    ///< firmware version (camera, gimbal, zoom) for reporting to GCS

    // buffer holding bytes from latest packet.  This is only used to calculate the crc
    uint8_t _msg_buff[AP_MOUNT_SIYI_PACKETLEN_MAX]; ///< buffer for complete packet (up to 42 bytes)
    uint8_t _msg_buff_len;                          ///< number of bytes currently in _msg_buff
    const uint8_t _msg_buff_data_start = 8;         ///< data payload starts at this byte offset in _msg_buff

    // parser state and unpacked fields
    struct PACKED {
        uint16_t data_len;                          ///< expected number of data bytes in current packet
        uint8_t command_id;                         ///< command ID of current packet
        uint16_t data_bytes_received;               ///< number of data bytes received so far
        uint16_t crc16;                             ///< CRC16 checksum of current packet
        ParseState state;                           ///< current state of packet parser state machine
    } _parsed_msg;

    // variables for sending packets to gimbal
    uint32_t _last_send_ms;                         ///< system time (ms) of last packet sent to gimbal
    uint16_t _last_seq;                             ///< last sequence number used (incremented for each send)

    // actual attitude received from gimbal
    Vector3f _current_angle_rad;                    ///< current gimbal angles in radians (x=roll, y=pitch, z=yaw)
    Vector3f _current_rates_rads;                   ///< current gimbal angular rates in rad/s (x=roll, y=pitch, z=yaw)
    uint32_t _last_current_angle_rad_ms;            ///< system time (ms) when _current_angle_rad was last updated
    uint32_t _last_req_current_angle_rad_ms;        ///< system time (ms) when attitude was last requested

    // absolute zoom control.  only used for A8 that does not support abs zoom control
    ZoomType _zoom_type;                            ///< current zoom control type (RATE or ABSOLUTE)
    float _zoom_rate_target;                        ///< target zoom rate (-1.0 to +1.0)
    float _zoom_mult;                               ///< most recent actual zoom multiple received from camera
    uint32_t _last_zoom_control_ms;                 ///< system time (ms) when zoom control was last run

    // Configuration info received from gimbal
    GimbalConfigInfo _config_info;                  ///< gimbal configuration (HDR, recording, motion mode, mounting, video output)
    
    // rangefinder variables
    uint32_t _last_rangefinder_req_ms;              ///< system time (ms) of last request for rangefinder distance
    uint32_t _last_rangefinder_dist_ms;             ///< system time (ms) of last successful read of rangefinder distance
    float _rangefinder_dist_m;                      ///< last distance measurement received from rangefinder (meters)

    // sending of attitude and position to gimbal
    uint32_t _last_attitude_send_ms;                ///< system time (ms) when vehicle attitude was last sent to gimbal

    /**
     * @brief Send vehicle attitude and position to gimbal for stabilization
     * 
     * @details Sends EXTERNAL_ATTITUDE and POSITION_DATA commands to provide
     *          gimbal with vehicle state for improved stabilization performance
     * 
     * @note Called at regular intervals from update() method
     */
    void send_attitude_position(void);

    /**
     * @brief Hardware lookup table indexed by HardwareModel enum values
     * 
     * @details Maps hardware model enum to:
     *          - 2-byte hardware ID received from gimbal
     *          - Human-readable model name string
     */
    struct HWInfo {
        uint8_t hwid[2];            ///< 2-byte hardware ID from HARDWARE_ID response
        const char* model_name;     ///< model name string (e.g., "A2", "ZR30")
    };
    static const HWInfo hardware_lookup_table[];

#if AP_MOUNT_SEND_THERMAL_RANGE_ENABLED
    /**
     * @brief Thermal imaging data structure
     * 
     * @details Stores thermal min/max temperatures and positions received from
     *          GET_TEMP_FULL_IMAGE responses at 5Hz
     * 
     * @note Only applicable to thermal-equipped models (ZT6, ZT30)
     */
    struct {
        uint32_t last_req_ms;       ///< system time (ms) of last request for thermal data
        uint32_t last_update_ms;    ///< system time (ms) of last thermal data update
        float max_C;                ///< maximum temperature in image (degrees Celsius)
        float min_C;                ///< minimum temperature in image (degrees Celsius)
        Vector2ui max_pos;          ///< pixel position of max temp (x=0 is left, y=0 is top)
        Vector2ui min_pos;          ///< pixel position of min temp (x=0 is left, y=0 is top)
    } _thermal;
#endif

    uint8_t sent_time_count;                        ///< count of SET_TIME packets sent (send 5 times to cope with packet loss)
};

#endif // HAL_MOUNT_SIYI_ENABLED
