/**
 * @file AP_Mount_XFRobot.h
 * @brief XFRobot camera gimbal backend using custom binary serial protocol
 * 
 * @details This backend implements the XFRobot SetAttitude binary protocol for
 *          controlling camera gimbals. The protocol uses a custom framing format
 *          with CRC16-CCITT error checking and supports:
 *          - Attitude control (roll, pitch, yaw)
 *          - Vehicle state transmission (acceleration, velocity, GPS position)
 *          - Camera control commands (photo, video, zoom, focus, lens selection)
 *          - Gimbal status and telemetry reception
 *          
 *          Protocol characteristics:
 *          - Binary framing: 0x8A 0x5E header
 *          - CRC16-CCITT validation
 *          - Periodic resend of target angles
 *          - Bidirectional communication (commands and telemetry)
 * 
 * @note All angle values are transmitted in centidegrees (0.01 degrees) in the protocol
 * @note Velocity units are decimeters/s in protocol, acceleration in cm/s/s
 * @note GPS coordinates use degrees * 10^7 format (1E7)
 * 
 * @warning Proper CRC validation is critical for reliable gimbal control
 * @warning Packet synchronization must be maintained to avoid control glitches
 */
#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_XFROBOT_ENABLED

#include "AP_Mount_Backend_Serial.h"
#include <AP_Math/quaternion.h>

#define AP_MOUNT_XFROBOT_RECV_LENGTH_MAX    80  // maximum number of bytes that will be received from the gimbal

/**
 * @class AP_Mount_XFRobot
 * @brief Backend driver for XFRobot camera gimbals using custom binary serial protocol
 * 
 * @details This class implements the XFRobot SetAttitude protocol for gimbal control.
 *          The protocol uses binary framing with the following structure:
 *          
 *          Main Frame Structure:
 *          - Header: 0x8A 0x5E (2 bytes)
 *          - Length: packet length field (2 bytes)
 *          - Version: protocol version (1 byte, always 0x02)
 *          - Attitude data: roll, pitch, yaw control and absolute angles (12 bytes)
 *          - Vehicle state: acceleration and velocity vectors (12 bytes)
 *          - GPS data: position, altitude, satellite count (24 bytes in sub-frame)
 *          - Camera commands: function order byte for camera operations (1 byte)
 *          - CRC16-CCITT: error detection (2 bytes)
 *          
 *          Protocol Features:
 *          - Attitude control with centidegree resolution (0.01°)
 *          - Vehicle state transmission for gimbal stabilization algorithms
 *          - GPS position for geo-pointing and target tracking
 *          - Camera control commands (photo, video, zoom, focus, lens modes)
 *          - Bidirectional telemetry with gimbal status feedback
 *          - CRC16-CCITT error checking for data integrity
 *          
 *          Angle Units in Protocol:
 *          - Roll/Pitch/Yaw: int16 centidegrees (±180.00° = ±18000)
 *          - Pitch range: ±90.00° = ±9000 centidegrees
 *          - Yaw absolute: uint16 0-360.00° = 0-36000 centidegrees
 *          
 *          Velocity and Acceleration Units:
 *          - Velocity: int16 decimeters/s (1 dm/s = 10 cm/s = 0.1 m/s)
 *          - Acceleration: int16 cm/s/s (centimeters per second squared)
 *          
 *          GPS Coordinate Units:
 *          - Latitude/Longitude: int32 degrees * 10^7 (1E7 scaling)
 *          - Altitude: int32 millimeters
 *          
 *          Timing Characteristics:
 *          - Periodic resend of target angles at controlled intervals
 *          - Packet timeout detection for health monitoring
 *          - Recording command timeout handling
 * 
 * @note The protocol requires periodic transmission of attitude commands to maintain control
 * @note CRC16-CCITT is calculated over all bytes from header through data payload
 * @note Packet synchronization state machine handles byte-stream parsing
 * 
 * @warning Incorrect CRC will cause packet rejection and potential control loss
 * @warning Packet framing errors can result in gimbal mode changes or loss of control
 * @warning Recording commands have timeout protection to prevent stuck states
 */
class AP_Mount_XFRobot : public AP_Mount_Backend_Serial
{

public:
    /**
     * @brief Constructor - inherits from AP_Mount_Backend_Serial
     */
    using AP_Mount_Backend_Serial::AP_Mount_Backend_Serial;

    /**
     * @brief Update mount position - should be called periodically from main loop
     * 
     * @details This method performs the following operations:
     *          - Reads and processes incoming telemetry packets from gimbal
     *          - Sends target attitude commands at controlled intervals
     *          - Handles recording timeout protection
     *          - Maintains communication health monitoring
     *          
     *          Called at scheduler rate (typically 50Hz or faster) to ensure
     *          responsive gimbal control and telemetry processing.
     * 
     * @note Must be called regularly for gimbal control to function
     * @note Packet transmission rate is internally limited to avoid flooding
     */
    void update() override;

    /**
     * @brief Return true if gimbal communication is healthy
     * 
     * @details Health is determined by:
     *          - Recent successful packet reception (within timeout period)
     *          - Valid firmware version received from gimbal
     *          - No critical communication errors
     * 
     * @return true if gimbal is responding and healthy, false otherwise
     * 
     * @note Health check is used by mount library to enable/disable control
     */
    bool healthy() const override;

    /**
     * @brief Check if this mount can control its pan (yaw) axis
     * 
     * @details Pan control capability is determined by configured yaw angle limits.
     *          Required for multicopter applications where vehicle can rotate
     *          independently of gimbal yaw angle.
     * 
     * @return true if yaw range is valid and pan control is available
     * 
     * @note Required for multicopter yaw compensation during autonomous missions
     */
    bool has_pan_control() const override { return yaw_range_valid(); };

    /**
     * @brief Check if this mount can control its roll axis
     * 
     * @details Roll control capability is determined by configured roll angle limits.
     *          Most camera gimbals have limited or no roll control to maintain
     *          horizon stabilization.
     * 
     * @return true if roll range is valid and roll control is available
     */
    bool has_roll_control() const override { return roll_range_valid(); };

    /**
     * @brief Check if this mount can control its pitch (tilt) axis
     * 
     * @details Pitch control is the primary gimbal control axis for most camera
     *          applications, allowing vertical camera pointing adjustment.
     * 
     * @return true if pitch range is valid and tilt control is available
     * 
     * @note Pitch is typically the most commonly used gimbal control axis
     */
    bool has_pitch_control() const override { return pitch_range_valid(); };

    //
    // camera controls for gimbals that include a camera
    //

    /**
     * @brief Trigger camera to take a picture
     * 
     * @details Sends SHUTTER command (FunctionOrder::SHUTTER = 0x20) to gimbal
     *          to trigger single photo capture. Command is sent as a simple
     *          1-byte parameter command via the binary protocol.
     * 
     * @return true if command was successfully queued for transmission, false on failure
     * 
     * @note Actual photo capture timing depends on gimbal processing and camera response
     * @note Multiple rapid calls may be rate-limited by protocol transmission timing
     */
    bool take_picture() override;

    /**
     * @brief Start or stop video recording
     * 
     * @details Sends RECORD_VIDEO command (FunctionOrder::RECORD_VIDEO = 0x21) with
     *          parameter 0x01 to start recording or 0x00 to stop. Includes timeout
     *          protection to detect if gimbal fails to respond to recording command.
     * 
     * @param[in] start_recording true to start recording, false to stop recording
     * 
     * @return true if command was successfully queued for transmission, false on failure
     * 
     * @note Recording state is tracked locally with timeout protection
     * @note Gimbal telemetry provides actual recording status feedback
     * 
     * @warning Timeout mechanism prevents stuck recording states if command is lost
     */
    bool record_video(bool start_recording) override;

    /**
     * @brief Set camera zoom level or zoom rate
     * 
     * @details Controls camera zoom using XFRobot protocol commands:
     *          - Continuous zoom: ZOOM_IN (0x22), ZOOM_OUT (0x23), ZOOM_STOP (0x24)
     *          - Rate control: ZOOM_RATE (0x25) with specified rate parameter
     *          
     *          Zoom commands are sent as simple 1-byte parameter commands.
     * 
     * @param[in] zoom_type Type of zoom control (rate, percentage, continuous)
     * @param[in] zoom_value Zoom value or rate (interpretation depends on zoom_type)
     * 
     * @return true if zoom command was successfully queued, false on failure
     * 
     * @note Actual zoom implementation depends on camera capabilities
     * @note Gimbal telemetry provides current zoom rate feedback
     */
    bool set_zoom(ZoomType zoom_type, float zoom_value) override;

    /**
     * @brief Set camera lens mode to cycle through picture-in-picture configurations
     * 
     * @details Controls picture-in-picture (PIP) display modes using PIC_IN_PIC
     *          command (FunctionOrder::PIC_IN_PIC = 0x74). Allows cycling through
     *          different camera view combinations (e.g., thermal + zoom, main + sub).
     * 
     * @param[in] lens Lens mode selection value
     * 
     * @return true if lens command was successfully queued, false on failure
     * 
     * @note Available PIP modes depend on camera hardware configuration
     * @see CameraType enum for available camera combinations
     */
    bool set_lens(uint8_t lens) override;

    /**
     * @brief Set camera focus mode and value
     * 
     * @details Controls camera focus using FOCUS command (FunctionOrder::FOCUS = 0x26).
     *          Supports multiple focus control modes:
     *          - Auto focus
     *          - Manual focus rate control (focus in = -1, hold = 0, out = 1)
     *          - Absolute focus position
     * 
     * @param[in] focus_type Type of focus control (auto, rate, position)
     * @param[in] focus_value Focus value (interpretation depends on focus_type)
     * 
     * @return Result indicating success, failure, or unsupported mode
     * 
     * @note Focus capabilities depend on camera lens and gimbal hardware
     * @note Auto-focus behavior is camera-dependent
     */
    SetFocusResult set_focus(FocusType focus_type, float focus_value) override;

protected:

    /**
     * @brief Get current gimbal attitude as a quaternion
     * 
     * @details Converts the gimbal's earth-frame roll, pitch and body-frame yaw
     *          angles (received in telemetry) into a quaternion representation.
     *          Uses the latest attitude data received from gimbal telemetry packets.
     *          
     *          Coordinate frame convention:
     *          - Roll and pitch are earth-frame referenced (absolute horizon)
     *          - Yaw is body-frame referenced (relative to vehicle heading)
     * 
     * @param[out] att_quat Output quaternion representing gimbal attitude
     * 
     * @return true if attitude data is valid and recent, false if stale or unavailable
     * 
     * @note Attitude freshness is checked against timeout threshold
     * @note Quaternion is constructed from Euler angles (roll, pitch, yaw)
     * 
     * @warning Stale attitude data (beyond timeout) returns false for safety
     */
    bool get_attitude_quaternion(Quaternion& att_quat) override;

private:

    /**
     * @brief Prefix string for GCS text messages to reduce flash memory usage
     * 
     * @details Common prefix used when sending status or error messages to ground
     *          control station. Stored as static const to minimize flash consumption
     *          from repeated string literals.
     */
    static const char* send_text_prefix;

    /**
     * @brief Camera and gimbal function commands (order byte in protocol)
     * 
     * @details Function order values sent in byte 69 of command packets to specify
     *          gimbal mode changes, camera operations, and configuration commands.
     *          Each command triggers specific gimbal or camera behavior.
     *          
     *          Command categories:
     *          - Mode control: 0x10-0x1C (angle control, head lock, follow, FPV, etc.)
     *          - Camera capture: 0x20-0x21 (photo, video recording)
     *          - Camera zoom/focus: 0x22-0x26 (zoom in/out/rate, focus)
     *          - Thermal imaging: 0x2A-0x33 (palette, night vision, temperature)
     *          - Display control: 0x73-0x76 (OSD, PIP, target detection)
     *          - Accessories: 0x80-0x81 (lighting, ranging)
     * 
     * @note Not all commands are supported by all gimbal hardware variants
     * @note Command execution confirmed via reply packets from gimbal
     */
    enum class FunctionOrder : uint8_t {
        NONE = 0x00,                // null command
        CALIBRATION = 0x01,         // calibration
        ISOTHERM = 0x02,            // isotherm mode
        NEUTRAL = 0x03,             // neutral mode
        OSD_COORDINATE = 0x06,      // OSD coordinate control (vehicle or target)
        IMAGE_AUTO_REVERSE = 0x07,  // auto reverse on/off
        TIME_ZONE = 0x08,           // set timezone
        ANGLE_CONTROL = 0x10,       // angle control
        HEAD_LOCK = 0x11,           // head lock mode
        HEAD_FOLLOW = 0x12,         // head follow mode
        ORTHOVIEW = 0x13,           // orthoview mode
        EULER_ANGLE_CONTROL = 0x14, // euler angle control
        GAZE_GEO_COORDINATES = 0x15,// gaze mode with geo coordinates
        GAZE_GEO_TARGET_LOCK = 0x16,// gaze mode with geo target lock
        TRACK = 0x17,               // track mode
        CLICK_TO_AIM = 0x1A,        // click to aim
        FPV = 0x1C,                 // first person view
        SHUTTER = 0x20,             // take picture
        RECORD_VIDEO = 0x21,        // record video
        ZOOM_IN = 0x22,             // zoom in continuously
        ZOOM_OUT = 0x23,            // zoom ou continuously
        ZOOM_STOP = 0x24,           // stop zooming
        ZOOM_RATE = 0x25,           // zoom at specfiied rate
        FOCUS = 0x26,               // auto focus
        PALETTE = 0x2A,             // set palette
        NIGHT_VISION = 0x2B,        // set night vision on/off
        AREA_TEMP_MEASURE = 0x30,   // set area temperature measurement on/off
        AREA_TEMP_ALERT = 0x31,     // set area temperature alert on/off
        SPOT_TEMP_MEASURE = 0x33,   // set spot temperature measurement on/off
        OSD = 0x73,                 // show/hide osd
        PIC_IN_PIC = 0x74,          // show/hide picture in picture
        TARGET_DETECTION = 0x75,    // set target detection on/off
        DIGITAL_ZOOM = 0x76,        // set digital zoom on/off
        LIGHTING = 0x80,            // set lighting intensity (0~255)
        RANGING = 0x81,             // set ranging on/off
    };

    /**
     * @brief Binary packet parsing state machine states
     * 
     * @details Tracks progress through packet reception and framing validation.
     *          The state machine processes incoming byte stream to:
     *          - Detect packet header synchronization (0x8A 0x5E)
     *          - Extract packet length field
     *          - Accumulate data payload bytes
     *          - Validate CRC16-CCITT checksum
     *          
     *          State progression:
     *          WAITING_FOR_HEADER1 → WAITING_FOR_HEADER2 → WAITING_FOR_LENGTH_LOW →
     *          WAITING_FOR_LENGTH_HIGH → WAITING_FOR_DATA → WAITING_FOR_CRC_HIGH →
     *          WAITING_FOR_CRC_LOW → packet processing or resync on error
     * 
     * @note State machine resets to WAITING_FOR_HEADER1 on any framing error
     * @note CRC validation failure discards packet and resyncs
     */
    enum class ParseState : uint8_t {
        WAITING_FOR_HEADER1,        ///< Searching for first header byte (0x8A)
        WAITING_FOR_HEADER2,        ///< Waiting for second header byte (0x5E)
        WAITING_FOR_LENGTH_LOW,     ///< Receiving low byte of length field
        WAITING_FOR_LENGTH_HIGH,    ///< Receiving high byte of length field
        WAITING_FOR_DATA,           ///< Accumulating data payload bytes
        WAITING_FOR_CRC_HIGH,       ///< Receiving high byte of CRC16
        WAITING_FOR_CRC_LOW         ///< Receiving low byte of CRC16
    };

    /**
     * @brief Camera image type configurations for picture-in-picture modes
     * 
     * @details Defines available camera view combinations for dual-camera gimbals
     *          (typically RGB zoom camera + thermal imaging camera). Controls which
     *          camera feed is displayed as main view and which as picture-in-picture.
     * 
     * @note Available modes depend on gimbal hardware configuration
     * @note Mode selection affects both display and recording outputs
     */
    enum class CameraType : uint8_t {
        MAIN_PIP_ZOOM_SUB_THERMAL = 1,  ///< Main: zoom with PIP overlay, Sub: thermal
        MAIN_THERMAL_SUB_ZOOM = 2,      ///< Main: thermal, Sub: zoom
        MAIN_PIP_THERMAL_SUB_ZOOM = 3,  ///< Main: thermal with PIP overlay, Sub: zoom
        MAIN_ZOOM_SUB_THERMAL = 4       ///< Main: zoom, Sub: thermal
       };

    /**
     * @brief Read and parse incoming telemetry packets from gimbal
     * 
     * @details Processes incoming serial byte stream using state machine parser to:
     *          - Detect packet frame synchronization (0x8A 0x5E header)
     *          - Extract packet length and accumulate data bytes
     *          - Validate CRC16-CCITT checksum
     *          - Dispatch successfully parsed packets to process_packet()
     *          
     *          Called from update() at scheduler rate to maintain low latency
     *          telemetry processing.
     * 
     * @note Processes multiple bytes per call if available in serial buffer
     * @note Invalid packets are discarded and parser resyncs to next header
     * 
     * @warning CRC failures indicate communication errors or electrical noise
     */
    void read_incoming_packets();

    /**
     * @brief Process successfully decoded packets held in msg_buff structure
     * 
     * @details Interprets validated packet data based on packet type and content:
     *          - Extracts gimbal attitude (roll, pitch, yaw) from telemetry
     *          - Updates gimbal status flags and error codes
     *          - Processes camera status (recording, zoom rate, mode)
     *          - Extracts target distance and position from ranging data
     *          - Updates firmware version information
     *          
     *          Updates attitude_latest structure with gimbal orientation for
     *          use by get_attitude_quaternion().
     * 
     * @note Only called after packet passes CRC validation
     * @note Attitude data is converted from centidegrees to degrees
     */
    void process_packet();

    /**
     * @brief Send target attitude angles to gimbal for tracking
     * 
     * @details Constructs and transmits SendPacketMainAndSubFrame containing:
     *          - Target attitude (roll, pitch, yaw) in centidegrees
     *          - Vehicle absolute attitude for gimbal stabilization algorithms
     *          - Vehicle acceleration and velocity vectors (NED frame)
     *          - GPS position (latitude, longitude, altitude)
     *          - Satellite count and GPS timing information
     *          - CRC16-CCITT checksum
     *          
     *          Transmission is rate-limited to avoid flooding gimbal with commands.
     * 
     * @param[in] angle_target_rad Target mount angles in radians (earth-frame roll/pitch, body-frame yaw)
     * 
     * @note Angles are converted from radians to centidegrees for protocol transmission
     * @note Vehicle state data improves gimbal stabilization performance
     * @note GPS data enables geo-pointing and target tracking features
     * 
     * @warning Transmission rate limiting prevents command buffer overflow
     */
    void send_target_angles(const MountTarget& angle_target_rad);

    /**
     * @brief Send simple 1-byte parameter command to gimbal
     * 
     * @details Constructs and transmits SimpleCommand packet containing:
     *          - Standard packet header and framing
     *          - Function order byte specifying command type
     *          - Single parameter byte for command argument
     *          - CRC16-CCITT checksum
     *          
     *          Used for camera operations like photo capture, video recording,
     *          zoom control, and mode changes.
     * 
     * @param[in] order Function order specifying command type (from FunctionOrder enum)
     * @param[in] param Single-byte parameter value for command (interpretation depends on order)
     * 
     * @return true if command successfully queued for transmission, false on serial port error
     * 
     * @note Examples: take_picture() sends (SHUTTER, 0x01), record_video() sends (RECORD_VIDEO, 0x01/0x00)
     * @note Command acknowledgment received via reply packets
     */
    bool send_simple_command(FunctionOrder order, uint8_t param);

    /**
     * @brief Check for recording command timeout and recovery
     * 
     * @details Monitors time elapsed since recording start/stop command was sent.
     *          If gimbal fails to acknowledge recording state change within timeout
     *          period, clears pending request to prevent stuck state.
     *          
     *          Protects against:
     *          - Lost command packets
     *          - Gimbal communication failures during recording operations
     *          - Indefinite waiting for recording confirmation
     * 
     * @note Called from update() to maintain timeout monitoring
     * @note Timeout threshold prevents indefinite request pending state
     * 
     * @warning Essential for preventing stuck recording request flags
     */
    void check_recording_timeout();

    // internal variables
    uint32_t last_send_ms;     ///< System time (milliseconds) of last attitude command sent to gimbal (for rate limiting)

    //
    // packet structures
    //

    /**
     * @brief Main and sub-frame structure for sending commands to gimbal (SetAttitude protocol)
     * 
     * @details Complete packet structure transmitted to gimbal for attitude control and
     *          vehicle state updates. Contains 70 bytes of data plus 2-byte CRC16-CCITT.
     *          
     *          Packet layout (72 bytes total):
     *          - Header and framing: bytes 0-4 (header, length, version)
     *          - Gimbal control: bytes 5-11 (roll/pitch/yaw control + status)
     *          - Vehicle attitude: bytes 12-17 (absolute roll/pitch/yaw)
     *          - Vehicle dynamics: bytes 18-29 (acceleration + velocity vectors)
     *          - Request code: byte 30 (sub-frame request from GCU)
     *          - Reserved: bytes 31-36
     *          - GPS sub-frame: bytes 37-68 (position, altitude, satellite data)
     *          - Function order: byte 69 (camera/gimbal command)
     *          - CRC16: bytes 70-71 (error detection, calculated separately)
     *          
     *          Angle encoding: int16 centidegrees (±180° = ±18000, ±90° = ±9000)
     *          Velocity encoding: int16 decimeters/s (dm/s = 10 cm/s)
     *          Acceleration encoding: int16 cm/s/s
     *          GPS encoding: int32 degrees * 1E7, int32 millimeters altitude
     *          
     *          Status byte (byte 11):
     *          - Bit 0: INS (Inertial Navigation System) valid
     *          - Bit 2: Control values valid
     * 
     * @note Structure is PACKED to ensure byte-aligned transmission
     * @note CRC16-CCITT calculated over bytes 0-69, appended as bytes 70-71
     * @note Periodic transmission required to maintain gimbal control
     */
    struct PACKED SendPacketMainAndSubFrame {
        uint8_t header1;        // byte0, header1 (0x8A)
        uint8_t header2;        // byte1, header2 (0x5E)
        uint16_t length;        // byte2~3, length
        uint8_t version;        // byte 4: version (always 0x02)
        int16_t roll_control;   // byte 5~6: roll control value (int16, -18000 ~ +18000)
        int16_t pitch_control;  // byte 7~8: pitch control value (int16, -18000 ~ +18000)
        int16_t yaw_control;    // byte 9~10: yaw control value (int16, -18000 ~ +18000)
        uint8_t status;         // byte 11: status, Bit0:INS valid, Bit2:control values valid
        int16_t roll_abs;       // byte 12~13: absolute roll angle of vehicle (int16, -18000 ~ +18000)
        int16_t pitch_abs;      // byte 14~15: absolute pitch angle of vehicle (int16, -9000 ~ +9000)
        int16_t yaw_abs;        // byte 16~17: absolute yaw angle of vehicle (uint16, 0 ~ 36000)    
        int16_t accel_north;    // byte 18~19: North acceleration of vehicle (int16, cm/s/s)
        int16_t accel_east;     // byte 20~21: East acceleration of vehicle (int16, cm/s/s)
        int16_t accel_up;       // byte 22~23: Upward acceleration of vehicle (int16, cm/s/s)
        int16_t vel_north;      // byte 24~25: North speed of vehicle (int16, decimeter/s)
        int16_t vel_east;       // byte 26~27: East speed of vehicle (int16, decimeter/s)
        int16_t vel_up;         // byte 28~29: Upward speed of vehicle (int16, decimeter/s)
        uint8_t request_code;   // byte 30: request code of sub frame, header of requested sub data frame from GCU (aka camera)
        uint8_t reserved[6];    // byte 31~36: reserved/unused
        uint8_t sub_header;     // byte 37: header (always 0x01)
        int32_t longitude;      // byte 38~41: longitude of vehicle (int32, 1E7)
        int32_t latitude;       // byte 42~45: latitude of vehicle (int32, 1E7)
        int32_t alt_amsl;       // byte 46~49: altitude of vehicle (int32, mm)
        uint8_t gps_num_sats;   // byte 50: number of satellites
        uint32_t gps_week_ms;   // byte 51~54: GNSS milliseconds (uint32)
        uint16_t gps_week;      // byte 55~56: GNSS week number (uint16)
        int32_t alt_rel;        // byte 57~60: relative altitude (int32, mm, can be zero if unavailable)
        uint8_t reserved2[8];   // byte 61~68: reserved/unused
        FunctionOrder order;    // byte 69: order
    };

    /**
     * @brief Main and sub-frame structure for receiving telemetry from gimbal
     * 
     * @details Complete packet structure received from gimbal containing status,
     *          attitude, and operational data. Contains 70 bytes of data plus
     *          2-byte CRC16-CCITT checksum.
     *          
     *          Packet layout (72 bytes total):
     *          - Header and framing: bytes 0-4 (header, length, version)
     *          - Gimbal mode and status: bytes 5-7 (mode, status flags)
     *          - Target tracking: bytes 8-11 (horizontal/vertical target position)
     *          - Camera-frame angles: bytes 12-17 (x/y/z angles relative to vehicle)
     *          - Absolute attitude: bytes 18-23 (earth-frame roll/pitch/yaw in centidegrees)
     *          - Angular velocity: bytes 24-29 (x/y/z absolute angular rates in centidegrees/s)
     *          - Reserved: bytes 30-36
     *          - Version info: bytes 37-42 (hardware/firmware version, pod code, errors)
     *          - Ranging data: bytes 43-58 (target distance and GPS coordinates)
     *          - Camera status: bytes 59-68 (zoom rates, thermal status, camera modes, timezone)
     *          - Function order: byte 69 (command acknowledgment)
     *          - CRC16: bytes 70-71 (error detection)
     *          
     *          Status flags (bytes 6-7):
     *          - Bit 0: Tracking success
     *          - Bit 7: Range and target coordinates valid
     *          - Bit 8: Ranging enabled
     *          - Bit 9: Night vision enabled
     *          - Bit 10: Lighting enabled
     *          - Bit 12: Gimbal powered on
     *          
     *          Camera status (bytes 64-65):
     *          - Bits 0-2: Picture-in-picture mode
     *          - Bit 4: Recording active
     *          - Bit 11: Image auto-reverse disabled
     *          - Bit 12: OSD displays target coordinates
     *          - Bit 13: OSD enabled
     *          - Bit 14: Digital zoom enabled
     *          - Bit 15: Target detection enabled
     * 
     * @note Structure is PACKED to ensure byte-aligned reception
     * @note CRC16-CCITT validation required before processing packet data
     * @note Attitude data provided in centidegrees (0.01° resolution)
     */
    struct PACKED ReplyPacketMainAndSubFrame {
        uint8_t header1;        // byte0, header1 (0x8A)
        uint8_t header2;        // byte1, header2 (0x5E)
        uint16_t length;        // byte2~3, length
        uint8_t version;        // byte 4: version
        uint8_t mode;           // byte 5: mode: 0x10:angle control, 0x11:head lock, 0x12: head follow, 0x13:orthoview, 0x14:euler angle control, 0x16:gaze, 0x17:track,0x1C:FPV
        uint16_t status;        // byte 6~7: status, Bit0:tracking success, Bit7:range and target coordinate valid, Bit8:ranging on, Bit9:night vision on, Bit10:lighting on, Bit12:Upward powered on
        int16_t horizontal_target; // byte 8~9: horizontal target (int16, -1000 ~ +1000, rightward is positive)
        int16_t vertical_target;   // byte 10~11: vertical target (int16, -1000 ~ +1000, downward is positive)
        int16_t angle_x;        // byte 12~13: x-axis angle (camera-frame pitch) of camera relative to vehicle (int16, -18000 ~ +18000)
        int16_t angle_y;        // byte 14~15: y-axis angle (camera-frame roll) of camera relative to vehicle (int16, -18000 ~ +18000)
        int16_t angle_z;        // byte 16~17: z-axis angle (camera-frame yaw) of camera relative to vehicle (int16, -18000 ~ +18000)
        int16_t roll_abs_cd;    // byte 18~19: roll angle of camera (absolute) (int16, -9000 ~ +9000)
        int16_t pitch_abs_cd;   // byte 20~21: pitch angle of camera (absolute) (int16, -18000 ~ +18000)
        uint16_t yaw_abs_cd;    // byte 22~23: yaw angle of camera (absolute) (uint16, 0 ~ +36000)
        int16_t angvel_x;       // byte 24~25: X-axis absolute angular velocity of camera (int16, centi-degrees/s)
        int16_t angvel_y;       // byte 26~27: Y-axis absolute angular velocity of camera (int16, centi-degrees/s)
        int16_t angvel_z;       // byte 28~29: Z-axis absolute angular velocity of camera (int16, centi-degrees/s)
        uint8_t reserved[7];    // byte 30~36: reserved/unused
        uint8_t sub_header;     // byte 37: header (0x01)
        uint8_t hardware_version; // byte 38: hardware version (uint8)
        uint8_t firmware_version; // byte 39: firmware version (uint8)
        uint8_t pod_code;       // byte 40: pod code
        uint16_t error_code;    // byte 41~42: error code, Bit7:hardware error, Bit13: mavlink communication freq anomaly, Bit14:BNSS unpositioned, Bit15:GCU hardware error
        int32_t target_dist_dm; // byte 43~46: distance from target (int32, decimeters, -1m or 0m is invalid measurement)
        int32_t target_lng;     // byte 47~50: longitude of target (int32, 1E7)
        int32_t target_lat;     // byte 51~54: latitude of target (int32, 1E7)
        int32_t target_alt;     // byte 55~58: altitude of target (int32, mm)
        uint16_t zoom_rate_rgb; // byte 59~60: zoom rate of RGB camera (uint16, resolution 0.1x)
        uint16_t zoom_rate_thermal; // byte 61~62: zoom rate of thermal camera (uint16, resolution 0.1x)
        uint8_t thermal_status; // byte 63: thermal camera status, Bit0:low temp alert, Bit1:high temp alert, Bit3:spot temp measurement on, Bit4:Isotherm on, Bit5:temp alert on, Bit6:area temp on, Bit7:temp available
        uint16_t camera_status; // byte 64~65: camera status, Bit0~Bit2:pic-in-pic mode, Bit4:recording, Bit11:image auto reverse off, Bit12:OSD displays target coordinate, Bit13:OSD on, Bit14:digital zoom on, Bit15:target detection on
        int8_t timezone;        // byte 66: timezone (int8, -12 ~ +12)
        uint8_t reserved2[2];   // byte 67~68: reserved/unused
        FunctionOrder order;   // byte 69: order
    };

    /**
     * @brief Single-byte reply packet payload for command acknowledgment
     * 
     * @details Used by GCU (Gimbal Control Unit) to acknowledge simple commands
     *          such as photo capture, video recording, mode changes, etc.
     *          Appears at byte 70 in reply packets (before CRC).
     * 
     * @note Execution states: 0x00=Success, 0x01=Fail, 0x02=Operating/In-progress
     */
    struct PACKED PacketReply1Byte {
        uint8_t execution_state;    ///< Command execution result: 0x00=Success, 0x01=Fail, 0x02=Operating
    };

    /**
     * @brief CRC16-CCITT checksum structure appended to all packets
     * 
     * @details Two-byte CRC calculated over entire packet from header through data
     *          payload (bytes 0-69 for standard packets). Uses CRC16-CCITT algorithm
     *          for error detection.
     *          
     *          High byte transmitted first, followed by low byte.
     * 
     * @note CRC validation is mandatory before processing received packets
     * @warning Packets with invalid CRC must be discarded to prevent control errors
     */
    struct PACKED PacketCRC {
        uint8_t crc_high;    ///< CRC high byte (transmitted first)
        uint8_t crc_low;     ///< CRC low byte (transmitted second)
    };

    /**
     * @brief Union for constructing SetAttitude command packets with CRC
     * 
     * @details Provides convenient byte-level access for serial transmission while
     *          maintaining structured access to packet fields. Used to send periodic
     *          attitude control commands with vehicle state data.
     *          
     *          Total size: sizeof(SendPacketMainAndSubFrame) + sizeof(PacketCRC)
     *                    = 70 bytes + 2 bytes = 72 bytes
     * 
     * @note CRC is calculated over content.main (70 bytes) and stored in content.crc
     * @note bytes[] array used for serial port write operations
     */
    union SetAttitudePacket {
        struct PACKED {
            SendPacketMainAndSubFrame main;
            PacketCRC crc;
        } content;
        uint8_t bytes[sizeof(content)];
    };

    /**
     * @brief Union for constructing simple 1-byte parameter command packets
     * 
     * @details Used for camera control commands that require only a single parameter
     *          byte, such as take_picture, record_video, zoom control, etc.
     *          Extends SendPacketMainAndSubFrame with one parameter byte before CRC.
     *          
     *          Packet structure:
     *          - SendPacketMainAndSubFrame: 70 bytes (header, attitude, vehicle state, order)
     *          - Parameter byte: 1 byte (command-specific argument)
     *          - CRC16: 2 bytes (checksum)
     *          Total: 73 bytes
     * 
     * @note param_1byte interpretation depends on FunctionOrder in main.order
     * @note bytes[] array used for serial transmission
     * 
     * @see send_simple_command() for usage
     */
    union SimpleCommand {
        struct PACKED {
            SendPacketMainAndSubFrame main;  ///< Main packet frame with vehicle state
            uint8_t param_1byte;             ///< Single-byte command parameter
            PacketCRC crc;                   ///< CRC16-CCITT checksum
        } content;
        uint8_t bytes[sizeof(content)];      ///< Byte array for serial transmission
    };

    /**
     * @brief Structure for decoding simple reply packets from GCU
     * 
     * @details Standard reply format from Gimbal Control Unit containing:
     *          - Full telemetry data (ReplyPacketMainAndSubFrame)
     *          - Single-byte execution status (PacketReply1Byte)
     *          - CRC16 checksum (PacketCRC)
     *          
     *          Total size: 70 + 1 + 2 = 73 bytes
     * 
     * @note Used in msg_buff union for packet reception and parsing
     * @note CRC validation performed before accessing reply data
     */
    struct PACKED GCUSimpleReply {
        ReplyPacketMainAndSubFrame main;    ///< Main telemetry frame from gimbal
        PacketReply1Byte param;             ///< Command execution status
        PacketCRC crc;                      ///< CRC16-CCITT checksum
    };

    /**
     * @brief Message buffer union for receiving and parsing packets
     * 
     * @details Dual-purpose buffer providing both structured and byte-array access
     *          to incoming packet data. State machine parser accumulates bytes into
     *          this buffer, then accesses via simple_reply structure after validation.
     *          
     *          Maximum buffer size: AP_MOUNT_XFROBOT_RECV_LENGTH_MAX (80 bytes)
     *          Typical packet size: 73 bytes (70 data + 1 param + 2 CRC)
     * 
     * @note Parser writes to bytes[] during reception
     * @note simple_reply provides structured access after CRC validation
     */
    union {
        GCUSimpleReply simple_reply;                        ///< Structured access to validated reply packet
        uint8_t bytes[AP_MOUNT_XFROBOT_RECV_LENGTH_MAX];    ///< Raw byte array for parser accumulation
    } msg_buff;
    uint8_t msg_buff_len;                                   ///< Current number of bytes accumulated in msg_buff.bytes

    /**
     * @brief Packet parser state and tracking variables
     * 
     * @details Maintains state machine progress and packet metadata during
     *          reception and validation of incoming telemetry packets.
     */
    struct {
        ParseState state;           ///< Current state in packet parsing state machine
        uint8_t len_expected;       ///< Expected number of data bytes to receive (from length field)
        uint16_t crc;               ///< Calculated CRC16-CCITT for current packet
        uint32_t last_received_ms;  ///< System time (milliseconds) of last successfully parsed packet
    } parser;

    /**
     * @brief Latest gimbal attitude data received from telemetry
     * 
     * @details Stores most recent valid attitude information extracted from gimbal
     *          reply packets. Used by get_attitude_quaternion() to provide current
     *          gimbal orientation to mount library.
     *          
     *          Coordinate frame convention:
     *          - roll_ef_deg: Earth-frame roll (absolute horizon reference)
     *          - pitch_ef_deg: Earth-frame pitch (absolute horizon reference)
     *          - yaw_bf_deg: Body-frame yaw (relative to vehicle heading)
     * 
     * @note Attitude freshness checked via update_ms timestamp
     * @note Values converted from centidegrees to degrees during packet processing
     */
    struct {
        float roll_ef_deg;      ///< Earth-frame roll angle in degrees (absolute, ±180°)
        float pitch_ef_deg;     ///< Earth-frame pitch angle in degrees (absolute, ±180°)
        float yaw_bf_deg;       ///< Body-frame yaw angle in degrees (relative to vehicle, 0-360°)
        uint32_t update_ms;     ///< System time (milliseconds) when attitude was last updated
    } attitude_latest;

    /**
     * @brief Video recording state tracking and timeout management
     * 
     * @details Maintains local recording state and request tracking to detect
     *          command acknowledgment failures and prevent stuck recording flags.
     *          
     *          Timeout mechanism:
     *          - request_ms set when record_video() called
     *          - Cleared when gimbal confirms recording state change via telemetry
     *          - Timeout triggers if no confirmation received within threshold
     * 
     * @note recording state reflects last known gimbal telemetry
     * @note request_start and request_ms track pending command confirmation
     */
    bool got_firmware_version = false;  ///< True once hardware and firmware version received (for health check)
    struct {
        bool recording;         ///< True if gimbal telemetry indicates video recording active
        bool request_start;     ///< True if start recording requested, false if stop requested
        uint32_t request_ms;    ///< System time (milliseconds) of recording command, 0 if no pending request
    } recording;
};
#endif // HAL_MOUNT_XFROBOT_ENABLED
