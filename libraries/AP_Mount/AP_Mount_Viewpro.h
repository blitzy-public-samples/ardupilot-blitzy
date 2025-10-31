/**
 * @file AP_Mount_Viewpro.h
 * @brief Viewpro gimbal driver backend using custom serial protocol
 * 
 * @details This backend implements support for Viewpro gimbals using their proprietary
 *          serial communication protocol. The protocol uses a custom packet format with
 *          a 3-byte header (0x55, 0xAA, 0xDC), length/frame-count field, frame ID,
 *          variable-length data payload, and XOR checksum.
 * 
 *          The driver handles bidirectional communication including:
 *          - Handshake and initialization sequence
 *          - Target angle and rate commands
 *          - Camera control (zoom, focus, recording, picture capture)
 *          - Tracking control (point and rectangle tracking)
 *          - Telemetry reception (attitude, recording status, tracking status)
 *          - MAVLink camera message generation for GCS integration
 *          - Laser rangefinder support
 * 
 *          Packet format (courtesy of Viewpro's SDK document):
 * 
 *          -------------------------------------------------------------------------------------------
 *          Field     Index   Bytes       Description
 *          -------------------------------------------------------------------------------------------
 *          Header    0~2     3           0x55 0xAA 0xDC
 *          Length    3       1           bit0~5: body length, n=all bytes from byte3 to checksum, min=4, max=63
 *                                        bits6~7: frame counter (increment by 1 compared to previous message sent)
 *          Frame Id  4       1
 *          Data      5~n+1   n           1st byte is command id (?)
 *          Checksum  n+2     1           XOR of byte3 to n+1 (inclusive)
 * 
 * @note Protocol requires successful handshake before normal operation
 * @note Frame counter (bits 6-7 of length byte) must increment with each message
 * @note Checksum is XOR of all bytes from length field to last data byte (inclusive)
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_VIEWPRO_ENABLED

#include "AP_Mount_Backend_Serial.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/utility/sparse-endian.h>

/// Maximum number of bytes in a packet sent to or received from the gimbal
#define AP_MOUNT_VIEWPRO_PACKETLEN_MAX  63

/**
 * @class AP_Mount_Viewpro
 * @brief Backend driver for Viewpro gimbals using custom serial protocol
 * 
 * @details This class implements the Viewpro gimbal communication protocol which uses
 *          a proprietary serial packet format. The protocol features:
 * 
 *          **Protocol Structure:**
 *          - 3-byte header: 0x55, 0xAA, 0xDC (fixed identification sequence)
 *          - Length/frame-count byte: bits 0-5 contain payload length (4-63 bytes),
 *            bits 6-7 contain frame counter that increments with each message
 *          - Frame ID: identifies packet type (handshake, angles, camera control, etc.)
 *          - Data payload: variable length (0-58 bytes) containing command-specific data
 *          - XOR checksum: XOR of all bytes from length field through last data byte
 * 
 *          **Initialization Sequence:**
 *          - Driver sends HANDSHAKE packet (frame ID 0x00) on startup
 *          - Gimbal responds with T1_F1_B1_D1 packet containing current attitude
 *          - Driver queries firmware version and model name via U packets
 *          - Normal operation begins after successful handshake
 * 
 *          **M_AHRS Telemetry (Vehicle to Gimbal):**
 *          - Periodically sends vehicle attitude (roll/pitch/yaw) to gimbal
 *          - Includes GPS position, velocity, and timing information
 *          - Enables gimbal to perform earth-frame stabilization
 *          - Attitude angles use 16-bit representation: 1 bit = 360°/65536 = 0.0055°
 * 
 *          **Attitude Control:**
 *          - Supports both rate and angle control modes
 *          - Lock mode: gimbal maintains earth-frame target (requires M_AHRS data)
 *          - Follow mode: gimbal maintains body-frame target relative to vehicle
 *          - Angles transmitted in 0.01 degree units (centidegrees)
 * 
 *          **Camera Control:**
 *          - Image sensor selection: EO (RGB), IR (thermal), PIP modes, fusion
 *          - Zoom control: rate-based or absolute position (1x to 30x typical range)
 *          - Focus control: manual rate, auto-focus, manual position
 *          - Recording: start/stop video recording, take pictures
 *          - Multi-sensor gimbals support lens switching (up to 7 sensors on some models)
 * 
 *          **Tracking Control:**
 *          - Point tracking: lock onto specific pixel coordinates (0.0-1.0 normalized)
 *          - Rectangle tracking: define tracking region with top-left and bottom-right points
 *          - Tracking source selection: choose which camera sensor to use for tracking
 *          - Status feedback: searching, locked, lost states reported by gimbal
 * 
 *          **MAVLink Camera Integration:**
 *          - Generates CAMERA_INFORMATION messages with gimbal capabilities
 *          - Generates CAMERA_SETTINGS messages with current zoom and mode
 *          - Enables GCS to display camera controls and status
 * 
 *          **Laser Rangefinder:**
 *          - Single measurement or continuous ranging modes
 *          - Distance data received in gimbal telemetry packets
 *          - Can be enabled/disabled via commands
 * 
 * @note Protocol requires continuous communication - gimbal may timeout if no packets
 *       received for several seconds. Driver sends periodic M_AHRS or angle commands.
 * 
 * @note Angle units in protocol are 0.01 degrees (centidegrees). Driver converts
 *       to/from radians internally for consistency with ArduPilot conventions.
 * 
 * @note Multi-sensor gimbals (e.g., 1352 model) support up to 7 different lenses.
 *       Lens selection uses set_lens() or set_camera_source() methods.
 * 
 * @warning Handshake must complete successfully before gimbal accepts control commands.
 *          Driver will continuously retry handshake until gimbal responds.
 * 
 * @warning XOR checksum validation is critical - malformed packets may cause gimbal
 *          to enter error state requiring power cycle.
 * 
 * @see AP_Mount_Backend_Serial for base serial communication functionality
 */
class AP_Mount_Viewpro : public AP_Mount_Backend_Serial
{

public:
    /// Constructor - inherits from AP_Mount_Backend_Serial
    using AP_Mount_Backend_Serial::AP_Mount_Backend_Serial;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Mount_Viewpro);

    /**
     * @brief Update mount position - should be called periodically at main loop rate
     * 
     * @details This method is called periodically (typically at 50Hz) to:
     *          - Read and process incoming packets from gimbal
     *          - Send handshake if not yet established
     *          - Send target angle/rate commands based on pilot input or mission
     *          - Send M_AHRS vehicle attitude telemetry for earth-frame stabilization
     *          - Query firmware version and model name during initialization
     *          - Manage periodic resend timing to prevent gimbal timeout
     * 
     * @note This method implements the main communication loop with the gimbal
     * @note Must be called regularly (at least 1Hz) to maintain gimbal connection
     */
    void update() override;

    /**
     * @brief Return true if gimbal communication is healthy
     * 
     * @details Health check verifies:
     *          - Recent attitude data received from gimbal (within timeout period)
     *          - Handshake completed successfully
     *          - Serial communication active
     * 
     * @return true if gimbal is responding and providing valid attitude data
     * @return false if gimbal communication lost or not yet established
     * 
     * @note Used by mount library to determine if gimbal is operational
     * @note Unhealthy status triggers failsafe actions in some flight modes
     */
    bool healthy() const override;

    /**
     * @brief Return true if this mount accepts roll targets
     * 
     * @details Viewpro gimbals do not support roll axis control in current protocol.
     *          Only pitch (tilt) and yaw (pan) axes are controllable.
     * 
     * @return false - roll control not supported by Viewpro gimbals
     */
    bool has_roll_control() const override { return false; }

    /**
     * @brief Returns true if this mount can control its pan/yaw axis
     * 
     * @details Pan control capability depends on gimbal's configured yaw range.
     *          Required for multicopters to maintain camera pointing during yaw.
     * 
     * @return true if yaw range is valid and gimbal supports pan control
     * @return false if gimbal is pitch-only (no yaw control)
     * 
     * @note Critical for copters - enables "ROI" (Region of Interest) tracking
     */
    bool has_pan_control() const override { return yaw_range_valid(); };

    //
    // camera controls
    //

    /**
     * @brief Take a picture with the gimbal camera
     * 
     * @details Sends TAKE_PICTURE command via C1 packet to currently selected image sensor.
     *          Command is sent to the active lens (EO, IR, or other sensor as selected).
     * 
     * @return true if command sent successfully to gimbal
     * @return false if serial transmit buffer full or gimbal not ready
     * 
     * @note Does not wait for confirmation - assumes gimbal will execute command
     * @note Image storage managed by gimbal (SD card, internal memory)
     */
    bool take_picture() override;

    /**
     * @brief Start or stop video recording
     * 
     * @details Sends START_RECORD or STOP_RECORD command via C1 packet.
     *          Recording state is tracked internally and reported to GCS.
     * 
     * @param[in] start_recording true to start recording, false to stop
     * 
     * @return true if command sent successfully to gimbal
     * @return false if serial transmit buffer full or gimbal not ready
     * 
     * @note Recording status feedback received in D1 telemetry from gimbal
     * @note Video storage managed by gimbal (SD card, internal memory)
     */
    bool record_video(bool start_recording) override;

    /**
     * @brief Set camera zoom specified as rate or percentage
     * 
     * @details Supports two zoom control modes:
     *          - Rate control: continuous zoom in/out at specified speed
     *          - Position control: zoom to absolute position (1x to 30x typical)
     * 
     *          Rate mode uses C1 packet with ZOOM_IN/ZOOM_OUT commands.
     *          Position mode uses C2 packet with SET_EO_ZOOM command.
     * 
     * @param[in] zoom_type Type of zoom control (RATE or POSITION)
     * @param[in] zoom_value For RATE: speed -100 to +100 (negative=out, positive=in)
     *                       For POSITION: absolute zoom 0.0 to 100.0 (percentage of range)
     * 
     * @return true if command sent successfully
     * @return false if invalid parameters or serial transmit failure
     * 
     * @note Zoom range varies by gimbal model and selected sensor
     * @note Current zoom position tracked in _zoom_times member variable
     */
    bool set_zoom(ZoomType zoom_type, float zoom_value) override;

    /**
     * @brief Set focus specified as rate, percentage, or auto
     * 
     * @details Supports multiple focus control modes:
     *          - Rate control: continuous focus near/far at specified speed
     *          - Auto focus: trigger automatic focus on scene center
     *          - Manual position: focus to specific position
     * 
     * @param[in] focus_type Type of focus control (RATE, AUTO, or other)
     * @param[in] focus_value For RATE: -1.0 (focus in), 0.0 (hold), +1.0 (focus out)
     *                        For other modes: may specify focus position
     * 
     * @return SetFocusResult indicating success or failure reason
     * 
     * @note Auto focus uses FOCUS_AUTO command in C1 packet
     * @note Manual focus uses FOCUS_PLUS/FOCUS_MINUS commands for rate control
     */
    SetFocusResult set_focus(FocusType focus_type, float focus_value) override;

    /**
     * @brief Set tracking to none, point, or rectangle mode
     * 
     * @details Configures gimbal's computer vision tracking system:
     *          - NONE: stop tracking
     *          - POINT: track object at specified pixel coordinates
     *          - RECTANGLE: track objects within defined region
     * 
     *          Uses E1 and E2 packets to configure tracking parameters.
     *          Tracking status (searching, locked, lost) received in F1 telemetry.
     * 
     * @param[in] tracking_type Type of tracking (NONE, POINT, RECTANGLE)
     * @param[in] p1 For POINT: target coordinates (x,y in 0.0-1.0 range)
     *               For RECTANGLE: top-left corner (x,y in 0.0-1.0 range)
     * @param[in] p2 For RECTANGLE: bottom-right corner (x,y in 0.0-1.0 range)
     *               For POINT: ignored
     * 
     * @return true if tracking command sent successfully
     * @return false if invalid parameters or serial transmit failure
     * 
     * @note Coordinates are normalized: 0.0 = left/top, 1.0 = right/bottom
     * @note Tracking uses currently selected camera sensor
     * @note Driver converts 0.0-1.0 coordinates to pixel coordinates internally
     */
    bool set_tracking(TrackingType tracking_type, const Vector2f& p1, const Vector2f& p2) override;

    /**
     * @brief Set camera lens/sensor selection by index
     * 
     * @details Selects active image sensor on multi-sensor gimbals.
     *          Lens index maps to ImageSensor enum values:
     *          - 0: No action
     *          - 1: EO1 (primary RGB camera)
     *          - 2: IR (thermal/infrared camera)
     *          - 3: EO1+IR PIP (RGB main, thermal picture-in-picture)
     *          - 4: IR+EO1 PIP (thermal main, RGB picture-in-picture)
     *          - 5: Fusion (RGB and thermal fused)
     * 
     * @param[in] lens Lens index 0-5 (model-specific, see ImageSensor enum)
     * 
     * @return true if lens selection sent successfully
     * @return false if invalid lens index or serial transmit failure
     * 
     * @note Available lenses vary by gimbal model (1352 has 7 sensors)
     * @note Selection stored in _image_sensor for subsequent commands
     */
    bool set_lens(uint8_t lens) override;

    /**
     * @brief Set camera source by primary and secondary sensor types
     * 
     * @details Functionally equivalent to set_lens() but uses AP_Camera::CameraSource
     *          enum for type-based selection instead of index. Enables GCS to select
     *          sensors by capability (RGB, thermal) rather than index.
     * 
     * @param[in] primary_source Primary camera source (AP_Camera::CameraSource enum)
     * @param[in] secondary_source Secondary camera source for PIP modes
     * 
     * @return true if camera source selection sent successfully
     * @return false if invalid source types or serial transmit failure
     * 
     * @note Maps CameraSource enum to ImageSensor protocol values
     * @note Supports PIP (picture-in-picture) mode configuration
     */
    bool set_camera_source(uint8_t primary_source, uint8_t secondary_source) override;

    /**
     * @brief Send CAMERA_INFORMATION MAVLink message to GCS
     * 
     * @details Generates MAVLink message describing gimbal camera capabilities:
     *          - Firmware version
     *          - Model name
     *          - Sensor resolution
     *          - Supported features (zoom, focus, tracking)
     *          - Lens/sensor configuration
     * 
     * @param[in] chan MAVLink channel to send message on
     * 
     * @note Called periodically by mount library to update GCS
     * @note Enables GCS to display appropriate camera controls
     */
    void send_camera_information(mavlink_channel_t chan) const override;

    /**
     * @brief Send CAMERA_SETTINGS MAVLink message to GCS
     * 
     * @details Generates MAVLink message with current camera state:
     *          - Recording status (idle, recording, photo mode)
     *          - Current zoom level
     *          - Active sensor/lens selection
     *          - Tracking status
     * 
     * @param[in] chan MAVLink channel to send message on
     * 
     * @note Called periodically to update GCS with current camera state
     * @note Enables GCS to display real-time camera status
     */
    void send_camera_settings(mavlink_channel_t chan) const override;

    //
    // rangefinder
    //

    /**
     * @brief Get laser rangefinder distance measurement
     * 
     * @details Retrieves most recent distance measurement from gimbal's laser rangefinder.
     *          Distance data received in gimbal telemetry packets (D1 frame).
     * 
     * @param[out] distance_m Distance to target in meters
     * 
     * @return true if valid distance measurement available
     * @return false if no recent measurement or rangefinder disabled
     * 
     * @note Rangefinder must be enabled via set_rangefinder_enable()
     * @note Distance data updated when D1 packets received from gimbal
     * @note Typical range: 3m to 1200m depending on target and conditions
     */
    bool get_rangefinder_distance(float& distance_m) const override;

    /**
     * @brief Enable or disable laser rangefinder
     * 
     * @details Sends rangefinder control command via C1 packet LRF field:
     *          - Enable: CONTINUOUS_RANGING_START command
     *          - Disable: STOP_RANGING command
     * 
     * @param[in] enable true to start continuous ranging, false to stop
     * 
     * @return true if command sent successfully
     * @return false if serial transmit failure or gimbal not ready
     * 
     * @note Continuous mode provides ~10Hz distance updates
     * @note Alternative: SINGLE_RANGING for one-shot measurements
     */
    bool set_rangefinder_enable(bool enable) override;

protected:

    /**
     * @brief Get gimbal attitude as a quaternion
     * 
     * @details Retrieves current gimbal attitude received from T1_F1_B1_D1 telemetry.
     *          Attitude represents gimbal orientation in vehicle body frame.
     *          Data updated when gimbal sends attitude telemetry packets.
     * 
     * @param[out] att_quat Quaternion representing gimbal attitude (body frame)
     * 
     * @return true if recent attitude data available (within health timeout)
     * @return false if no recent attitude data or gimbal not responding
     * 
     * @note Internally uses _current_angle_rad (roll, pitch, yaw) converted to quaternion
     * @note Attitude update rate typically 10-50Hz depending on gimbal configuration
     */
    bool get_attitude_quaternion(Quaternion& att_quat) override;

private:

    /// Send text prefix string to reduce flash cost when logging messages
    static const char* send_text_prefix;

    /**
     * @brief Packet frame IDs identifying packet type and purpose
     * 
     * @details Frame ID appears in byte 4 of every packet and determines packet structure
     *          and interpretation. Used for both commands (sent to gimbal) and telemetry
     *          (received from gimbal). Some IDs are bidirectional.
     */
    enum class FrameId : uint8_t {
        HANDSHAKE = 0x00,       ///< Handshake packet sent to gimbal during initialization
        U = 0x01,               ///< Communication configuration control command (sent to gimbal)
        V = 0x02,               ///< Communication configuration status reply (received from gimbal in response to U)
        HEARTBEAT = 0x10,       ///< Heartbeat/keepalive received periodically from gimbal
        A1 = 0x1A,              ///< Target angle/rate commands sent to gimbal for pitch/yaw control
        C1 = 0x1C,              ///< Camera control commands commonly used (zoom, focus, record, picture)
        E1 = 0x1E,              ///< Tracking control commands commonly used (start, stop, set target)
        C2 = 0x2C,              ///< Camera control commands infrequently used (absolute zoom position)
        E2 = 0x2E,              ///< Tracking control commands infrequently used (advanced tracking parameters)
        T1_F1_B1_D1 = 0x40,     ///< Telemetry packet with actual gimbal angles, tracking status, recording status, rangefinder data
        M_AHRS = 0xB1,          ///< Vehicle AHRS data sent to gimbal (attitude, GPS position, velocity) for earth-frame stabilization
    };

    // U communication configuration control commands
    enum class CommConfigCmd : uint8_t {
        QUERY_FIRMWARE_VER = 0xD0,
        QUERY_MODEL = 0xE4,
    };

    // A1 servo status enum (used in A1, B1 packets)
    enum class ServoStatus : uint8_t {
        MANUAL_SPEED_MODE = 0x01,
        FOLLOW_YAW = 0x03,
        MANUAL_ABSOLUTE_ANGLE_MODE = 0x0B,
        FOLLOW_YAW_DISABLE = 0x0A,
    };

    // C1 image sensor choice
    enum class ImageSensor : uint8_t {
        NO_ACTION = 0x00,   // no image sensor is affected
        EO1 = 0x01,         // electro-optical, aka rgb
        IR = 0x02,          // infrared, aka thermal
        EO1_IR_PIP = 0x03,  // rgb is main, IR is picture-in-picture
        IR_EO1_PIP = 0x04,  // thermal is main, rgb is picture-in-picture
        FUSION = 0x05,      // rgb and thermal are fused
        IR1_13MM = 0x06,    // only valid for 1352 module
        IR2_52MM = 0x07,    // only valid for 1352 module
    };

    // C1 camera commands
    enum class CameraCommand : uint8_t {
        NO_ACTION = 0x00,
        STOP_FOCUS_AND_ZOOM = 0x01,
        ZOOM_OUT = 0x08,
        ZOOM_IN = 0x09,
        FOCUS_PLUS = 0x0A,
        FOCUS_MINUS = 0x0B,
        TAKE_PICTURE = 0x13,
        START_RECORD = 0x14,
        STOP_RECORD = 0x15,
        AUTO_FOCUS = 0x19,
        MANUAL_FOCUS = 0x1A,
        IR_ZOOM_OUT = 0x1B,
        IR_ZOOM_IN = 0x1C
    };

    // C1 rangefinder commands
    enum class LRFCommand : uint8_t {
        NO_ACTION = 0x00,
        SINGLE_RANGING = 0x01,
        CONTINUOUS_RANGING_START = 0x02,
        LPCL_CONTINUOUS_RANGING_START = 0x03,
        STOP_RANGING = 0x05
    };

    // C2 camera commands
    enum class CameraCommand2 : uint8_t {
        SET_EO_ZOOM = 0x53
    };

    // D1 recording status (received from gimbal)
    enum class RecordingStatus : uint8_t {
        RECORDING_STOPPED = 0x00,
        RECORDING = 0x01,
        PICTURE_MODE = 0x02
    };

    // E1 tracking commands
    enum class TrackingCommand : uint8_t {
        STOP = 0x01,
        START = 0x03,
        SET_POINT = 0x0A,
        SET_RECT_TOPLEFT = 0x0B,
        SET_RECT_BOTTOMRIGHT = 0x0C,
    };

    // E1 tracking source (e.g. which camera)
    enum class TrackingSource : uint8_t {
        EO1 = 0x01,         // electro-optical, aka rgb
        IR = 0x02,          // infrared, aka thermal
        EO2 = 0x03,         // electro-optical, aka rgb
    };

    // E2 tracking commands2
    enum class TrackingCommand2 : uint8_t {
        SET_POINT = 0x0A,
        SET_RECT_TOPLEFT = 0x0B,
        SET_RECT_BOTTOMRIGHT = 0x0C,
    };

    // F1 tracking status (received from gimbal)
    enum class TrackingStatus : uint8_t {
        STOPPED = 0x00,     // not tracking
        SEARCHING = 0x01,   // searching
        TRACKING = 0x02,    // locked onto a target
        LOST = 0x03,        // lost target
    };

    /**
     * @brief Packet parsing state machine states
     * 
     * @details State machine for parsing incoming packets byte-by-byte from serial stream.
     *          Progresses through header detection, length/frame-count, frame ID, variable-length
     *          data payload, and finally checksum validation. Invalid bytes reset to HEADER1 state.
     */
    enum class ParseState : uint8_t {
        WAITING_FOR_HEADER1,    ///< Waiting for first header byte (0x55)
        WAITING_FOR_HEADER2,    ///< Waiting for second header byte (0xAA) after receiving 0x55
        WAITING_FOR_HEADER3,    ///< Waiting for third header byte (0xDC) after receiving 0x55 0xAA
        WAITING_FOR_LENGTH,     ///< Waiting for length/frame-count byte (bits 0-5: length, bits 6-7: frame counter)
        WAITING_FOR_FRAMEID,    ///< Waiting for frame ID byte identifying packet type
        WAITING_FOR_DATA,       ///< Receiving variable-length data payload bytes
        WAITING_FOR_CRC         ///< Waiting for XOR checksum byte to validate packet
    };

    // packet formats
    union HandshakePacket {
        struct {
            FrameId frame_id;           // always 0x00
            uint8_t unused;             // always 0x00
        } content;
        uint8_t bytes[sizeof(content)];
    };

    // U packed used to send communication configuration control commands
    // gimbal replies with V packet
    union UPacket {
        struct {
            FrameId frame_id;           // always 0x01
            CommConfigCmd control_cmd;  // see CommConfigCmd enum above
            uint8_t params[9];          // parameters (unused)
        } content;
        uint8_t bytes[sizeof(content)];
    };

    // A1 used to send target angles and rates
    union A1Packet {
        struct {
            FrameId frame_id;           // always 0x1A
            ServoStatus servo_status;   // see ServoStatus enum above
            be16_t yaw_be;              // target yaw angle or rate msb
            be16_t pitch_be;            // target pitch angle or rate msb
            uint8_t unused[4];          // unused
        } content;
        uint8_t bytes[sizeof(content)];
    };

    // C1 used to send camera commands (commonly used)
    union C1Packet {
        struct PACKED {
            FrameId frame_id;           // always 0x1C
            be16_t sensor_zoom_cmd_be;
        } content;
        uint8_t bytes[sizeof(content)];
    };

    // C2 used to send camera commands (less commonly used)
    union C2Packet {
        struct {
            FrameId frame_id;           // always 0x2C
            CameraCommand2 cmd;         // see CameraCommand2 enum above
            be16_t value_be;            // value
        } content;
        uint8_t bytes[sizeof(content)];
    };

    // E1 used to send tracking commands
    union E1Packet {
        struct {
            FrameId frame_id;           // always 0x1E
            TrackingSource source : 3;  // see TrackingSource enum above
            uint8_t unused : 5;         // param1 (unused)
            TrackingCommand cmd;        // see TrackingCommand enum above
            uint8_t param2;             // param2
        } content;
        uint8_t bytes[sizeof(content)];
    };

    // E2 used to send tracking commands2
    union E2Packet {
        struct {
            FrameId frame_id;           // always 0x2E
            TrackingCommand2 cmd;       // see TrackingCommand2 enum above
            be16_t param1_be;
            be16_t param2_be;
        } content;
        uint8_t bytes[sizeof(content)];
    };

    // M_AHRS used to send vehicle attitude and position to gimbal
    union M_AHRSPacket {
        struct PACKED {
            FrameId frame_id;           // always 0xB1
            uint8_t data_type;          // should be 0x07.  Bit0: Attitude, Bit1: GPS, Bit2 Gyro
            uint8_t unused2to8[7];      // unused
            be16_t roll_be;             // vehicle roll angle.  1bit=360deg/65536
            be16_t pitch_be;            // vehicle pitch angle.  1bit=360deg/65536
            be16_t yaw_be;              // vehicle yaw angle.  1bit=360deg/65536
            be16_t date_be;             // bit0~6:year, bit7~10:month, bit11~15:day
            uint8_t seconds_utc[3];     // seconds.  1bit = 0.01sec
            be16_t gps_yaw_be;          // GPS yaw
            uint8_t position_mark_bitmask;  // bit0:new position, bit1:clock fix calced, bit2:horiz calced, bit3:alt calced
            be32_t latitude_be;         // latitude.  1bit = 10e-7
            be32_t longitude_be;        // longitude.  1bit = 10e-7
            be32_t height_be;           // height.  1bit = 1mm
            be16_t ground_speed_N_be;   // ground speed in North direction. 1bit = 0.01m/s
            be16_t ground_speed_E_be;   // ground speed in East direction. 1bit = 0.01m/s
            be16_t vdop_be;             // GPS vdop. 1bit = 0.01
            be16_t ground_speed_D_be;   // speed downwards. 1bit = 0.01m/s
        } content;
        uint8_t bytes[sizeof(content)];
    };

    /**
     * @brief Read and parse incoming packets from gimbal serial port
     * 
     * @details Implements byte-by-byte parsing state machine to detect and validate packets.
     *          Reads available bytes from serial port, progresses through ParseState states,
     *          validates header sequence (0x55, 0xAA, 0xDC), extracts length and frame ID,
     *          accumulates data payload, and verifies XOR checksum. Successfully parsed
     *          packets are stored in _parsed_msg and _msg_buff for processing.
     * 
     * @note Called from update() method each iteration
     * @note Invalid bytes or checksum failures reset parser to WAITING_FOR_HEADER1
     * @note Parser is tolerant of garbage bytes and resynchronizes on valid header
     */
    void read_incoming_packets();

    /**
     * @brief Process successfully decoded packet held in _parsed_msg structure
     * 
     * @details Dispatches packet processing based on frame ID:
     *          - T1_F1_B1_D1: extract gimbal attitude, tracking status, recording status, rangefinder distance
     *          - V: parse firmware version and model name responses
     *          - HEARTBEAT: update connection status
     *          Updates internal state variables with received data.
     * 
     * @note Called by read_incoming_packets() after successful packet validation
     * @note Packet data available in _msg_buff starting at _msg_buff_data_start
     */
    void process_packet();

    /**
     * @brief Calculate XOR checksum of packet
     * 
     * @details Computes XOR of all bytes from length field through last data byte (inclusive).
     *          Checksum calculation starts at byte 3 (length byte) and includes all subsequent
     *          bytes except the checksum byte itself.
     * 
     * @param[in] buf Pointer to packet buffer starting at length byte
     * @param[in] length Number of bytes to include in checksum calculation
     * 
     * @return uint8_t XOR checksum value
     * 
     * @note Used for both validating received packets and generating sent packet checksums
     */
    uint8_t calc_crc(const uint8_t *buf, uint8_t length) const;

    /**
     * @brief Generate length and frame count byte (byte 3 of packet)
     * 
     * @details Constructs byte with:
     *          - Bits 0-5: payload length (data bytes + frame ID + checksum)
     *          - Bits 6-7: frame counter incremented from previous message
     * 
     * @param[in] length Number of payload bytes (4 to 63)
     * 
     * @return uint8_t Combined length and frame counter byte
     * 
     * @note Frame counter automatically increments and wraps 0-3
     * @note Updates _last_frame_counter member variable
     */
    uint8_t get_length_and_frame_count_byte(uint8_t length);

    /**
     * @brief Send packet to gimbal via serial port
     * 
     * @details Constructs complete packet with header (0x55, 0xAA, 0xDC), length/frame-count,
     *          provided data payload, and calculated XOR checksum. Writes packet to serial
     *          transmit buffer.
     * 
     * @param[in] databuff Pointer to data payload (starting with frame ID)
     * @param[in] databuff_len Length of data payload in bytes
     * 
     * @return true if packet sent successfully to serial buffer
     * @return false if serial transmit buffer full or invalid parameters
     * 
     * @note Automatically adds header, length/frame-count, and checksum
     * @note databuff must start with FrameId byte
     * @note Maximum databuff_len is AP_MOUNT_VIEWPRO_PACKETLEN_MAX - 4 (header + length + checksum)
     */
    bool send_packet(const uint8_t* databuff, uint8_t databuff_len);

    /**
     * @brief Send handshake packet to initiate gimbal communication
     * 
     * @details Sends HANDSHAKE frame (ID 0x00) to gimbal. Gimbal responds with T1_F1_B1_D1
     *          packet containing current attitude. Handshake required before gimbal accepts
     *          control commands. Driver retries handshake periodically until successful.
     * 
     * @note Called repeatedly from update() until gimbal responds with attitude data
     * @note Handshake timeout typically 5-10 seconds
     * 
     * @warning Gimbal will not respond to control commands until handshake completes
     */
    void send_handshake();

    /**
     * @brief Set gimbal lock mode (earth-frame vs body-frame stabilization)
     * 
     * @details Configures gimbal stabilization reference frame:
     *          - Lock mode (true): gimbal maintains earth-frame target using M_AHRS vehicle attitude
     *          - Follow mode (false): gimbal maintains body-frame target relative to vehicle
     * 
     * @param[in] lock true for earth-frame lock, false for body-frame follow
     * 
     * @return true if mode change command sent successfully
     * @return false if serial transmit failure
     * 
     * @note Lock mode requires periodic M_AHRS packets with vehicle attitude
     * @note Mode change sent via A1 packet with ServoStatus field
     */
    bool set_lock(bool lock);

    /**
     * @brief Send communication configuration command (U packet)
     * 
     * @details Sends U packet with specified configuration command. Gimbal responds with
     *          V packet containing requested information. Used to query firmware version,
     *          model name, and other configuration parameters.
     * 
     * @param[in] cmd Configuration command from CommConfigCmd enum
     * 
     * @return true if command sent successfully
     * @return false if serial transmit failure
     * 
     * @note Gimbal replies with V packet containing response data
     * @note Used during initialization to query gimbal capabilities
     */
    bool send_comm_config_cmd(CommConfigCmd cmd);

    /**
     * @brief Send target pitch and yaw rates to gimbal
     * 
     * @details Sends A1 packet with rate control mode. Gimbal continuously moves at specified
     *          rates until new command received or rates set to zero.
     * 
     * @param[in] pitch_rads Target pitch rate in radians/second
     * @param[in] yaw_rads Target yaw rate in radians/second
     * @param[in] yaw_is_ef true if yaw rate is earth-frame, false if body-frame
     * 
     * @return true if command sent successfully
     * @return false if serial transmit failure
     * 
     * @note Rates converted to 0.01 degree/sec units for protocol
     * @note ServoStatus field indicates rate control mode
     */
    bool send_target_rates(float pitch_rads, float yaw_rads, bool yaw_is_ef);

    /**
     * @brief Send target pitch and yaw angles to gimbal
     * 
     * @details Sends A1 packet with angle control mode. Gimbal moves to specified absolute
     *          angles and holds position. Angles are relative to vehicle body frame or
     *          earth frame depending on lock mode.
     * 
     * @param[in] pitch_rad Target pitch angle in radians
     * @param[in] yaw_rad Target yaw angle in radians  
     * @param[in] yaw_is_ef true if yaw angle is earth-frame, false if body-frame
     * 
     * @return true if command sent successfully
     * @return false if serial transmit failure
     * 
     * @note Angles converted to 0.01 degree units (centidegrees) for protocol
     * @note ServoStatus field indicates angle control mode and frame reference
     * @note Called periodically from update() to maintain gimbal position
     */
    bool send_target_angles(float pitch_rad, float yaw_rad, bool yaw_is_ef);

    /**
     * @brief Send camera control command (C1 packet)
     * 
     * @details Sends commonly-used camera commands via C1 packet format. Combines image sensor
     *          selection, camera command, value parameter, and optional laser rangefinder command
     *          into single packet.
     * 
     * @param[in] img_sensor Target image sensor (EO, IR, PIP modes, fusion) from ImageSensor enum
     * @param[in] cmd Camera command (zoom, focus, record, picture) from CameraCommand enum
     * @param[in] value Command parameter (e.g., zoom speed 0-100)
     * @param[in] lrf_cmd Laser rangefinder command (default NO_ACTION)
     * 
     * @return true if command sent successfully
     * @return false if serial transmit failure
     * 
     * @note C1 packet format combines multiple control fields in 16-bit encoded value
     * @note Image sensor selection persisted in _image_sensor for future commands
     */
    bool send_camera_command(ImageSensor img_sensor, CameraCommand cmd, uint8_t value, LRFCommand lrf_cmd = LRFCommand::NO_ACTION);

    /**
     * @brief Send camera control command 2 (C2 packet)
     * 
     * @details Sends infrequently-used camera commands via C2 packet format. Used for commands
     *          requiring 16-bit value parameter, such as absolute zoom position.
     * 
     * @param[in] cmd Camera command from CameraCommand2 enum (e.g., SET_EO_ZOOM)
     * @param[in] value 16-bit command parameter (e.g., absolute zoom position 0-1000)
     * 
     * @return true if command sent successfully
     * @return false if serial transmit failure
     * 
     * @note C2 used for commands needing larger parameter range than C1 supports
     * @note Value transmitted in big-endian byte order
     */
    bool send_camera_command2(CameraCommand2 cmd, uint16_t value);

    /**
     * @brief Send tracking control command (E1 packet)
     * 
     * @details Sends commonly-used tracking commands via E1 packet format. Used for basic
     *          tracking operations like start, stop, and simple target selection.
     * 
     * @param[in] cmd Tracking command from TrackingCommand enum (STOP, START, SET_POINT, etc.)
     * @param[in] value Command parameter (typically 0, meaning varies by command)
     * 
     * @return true if command sent successfully
     * @return false if serial transmit failure
     * 
     * @note Tracking source (camera sensor) specified in packet from _image_sensor
     * @note For point/rectangle tracking, use send_tracking_command2() with coordinates
     */
    bool send_tracking_command(TrackingCommand cmd, uint8_t value);

    /**
     * @brief Send tracking control command 2 (E2 packet)
     * 
     * @details Sends advanced tracking commands via E2 packet format with two 16-bit parameters.
     *          Used for specifying tracking target coordinates or region boundaries.
     * 
     * @param[in] cmd Tracking command from TrackingCommand2 enum (SET_POINT, SET_RECT_TOPLEFT, SET_RECT_BOTTOMRIGHT)
     * @param[in] param1 First parameter (typically x-coordinate or width)
     * @param[in] param2 Second parameter (typically y-coordinate or height)
     * 
     * @return true if command sent successfully
     * @return false if serial transmit failure
     * 
     * @note Parameters transmitted in big-endian byte order
     * @note For screen coordinates: 0,0 = top-left, range depends on resolution
     */
    bool send_tracking_command2(TrackingCommand2 cmd, int16_t param1, int16_t param2);

    /**
     * @brief Send vehicle attitude and position to gimbal (M_AHRS packet)
     * 
     * @details Sends M_AHRS telemetry packet containing vehicle state information for gimbal's
     *          earth-frame stabilization. Includes:
     *          - Vehicle attitude (roll, pitch, yaw) in 16-bit encoding: 1 bit = 360°/65536
     *          - GPS position (latitude, longitude, altitude)
     *          - GPS velocity (North, East, Down components)
     *          - UTC time and date
     *          - GPS quality indicators (VDOP, fix status)
     * 
     * @return true if telemetry sent successfully
     * @return false if serial transmit failure or vehicle state unavailable
     * 
     * @note Called periodically from update() (typically 10-20Hz)
     * @note Required for gimbal lock mode (earth-frame stabilization)
     * @note Gimbal uses this data to compensate for vehicle motion
     * @note Attitude angles converted from radians to protocol's 16-bit circular format
     */
    bool send_m_ahrs();

    // Internal variables for packet parsing and communication

    /// Buffer holding bytes of currently-received packet from gimbal (max 63 bytes)
    uint8_t _msg_buff[AP_MOUNT_VIEWPRO_PACKETLEN_MAX];
    
    /// Number of valid bytes currently held in _msg_buff
    uint8_t _msg_buff_len;
    
    /// Index where data payload starts in _msg_buff (skipping header bytes)
    const uint8_t _msg_buff_data_start = 2;

    /**
     * @brief Parser state and unpacked packet fields
     * 
     * @details Maintains state machine position and accumulated packet data during
     *          byte-by-byte parsing of incoming serial stream.
     */
    struct {
        uint8_t data_len;                           ///< Expected number of data bytes in current packet
        uint8_t frame_id;                           ///< Frame ID of current packet (identifies packet type)
        uint16_t data_bytes_received;               ///< Number of data bytes received so far for current packet
        uint8_t crc;                                ///< XOR checksum byte from current packet
        ParseState state;                           ///< Current state of incoming message processing state machine
    } _parsed_msg;

    // Variables for sending packets to gimbal

    /// Frame counter (0-3) sent in last message, increments with each new packet
    uint8_t _last_frame_counter;
    
    /// System time (milliseconds) when angle or rate targets were last sent to gimbal
    uint32_t _last_update_ms;
    
    /// Current gimbal angles in radians received from T1_F1_B1_D1 telemetry (x=roll, y=pitch, z=yaw)
    Vector3f _current_angle_rad;
    
    /// System time (milliseconds) when _current_angle_rad was last updated (used for health timeout)
    uint32_t _last_current_angle_rad_ms;
    
    /// Video recording status received from gimbal (true=recording, false=stopped/photo mode)
    bool _recording;
    
    /// Last lock mode sent to gimbal (true=earth-frame lock, false=body-frame follow)
    bool _last_lock;
    
    /// Last tracking status received from gimbal in F1 telemetry (STOPPED, SEARCHING, TRACKING, LOST)
    TrackingStatus _last_tracking_status;
    
    /// User-selected active image sensor (EO, IR, PIP, fusion) for camera commands
    ImageSensor _image_sensor;
    
    /// Current optical zoom level received from gimbal telemetry (1.0 = no zoom, 30.0 = 30x zoom typical)
    float _zoom_times;
    
    /// Firmware version number received from gimbal during initialization
    uint32_t _firmware_version;
    
    /// True once firmware version has been successfully received from gimbal
    bool _got_firmware_version;
    
    /// Model name string received from gimbal (e.g., "Z10T", "Z30T") max 10 chars + null terminator
    uint8_t _model_name[11] {};
    
    /// True once model name has been successfully received from gimbal
    bool _got_model_name;
    
    /// Latest laser rangefinder distance measurement in meters (from D1 telemetry)
    float _rangefinder_dist_m;
};

#endif // HAL_MOUNT_VIEWPRO_ENABLED
