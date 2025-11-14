/**
 * @file AP_Mount_Topotek.h
 * @brief Topotek gimbal driver backend using custom ASCII serial protocol
 * 
 * @details This backend implements support for Topotek gimbals using a custom
 *          ASCII-based serial protocol (prefix "#tp" for variable length packets).
 *          The protocol uses bytewise parsing to handle gimbal control, camera
 *          operations, tracking, and rangefinder integration.
 * 
 *          Packet format (courtesy of Topotek's SDK document):
 * 
 *          -------------------------------------------------------------------------------------------
 *          Field                 Index   Bytes       Description
 *          -------------------------------------------------------------------------------------------
 *          Frame Header          0       3           type of command ("#tp" or "#TP")
 *          Address Bit           3       2           source address first, destination address last
 *          Data_Len              5       1           data length
 *          Control Bit           6       1           'r' -> query, 'w' -> setup and control
 *          Identification Bit    7       3           identification function (e.g., "GIA", "REC")
 *          Data                  10      Data_Len    command-specific data
 *          Check Bit                     2           checksum (HEX sum converted to ASCII, high byte first)
 *          -------------------------------------------------------------------------------------------
 * 
 * @note Protocol uses ASCII characters for all fields, including numeric data encoded as hex ASCII
 * @note Communication rates: Attitude updates at 20Hz, rangefinder updates at 10Hz
 * 
 * Source: libraries/AP_Mount/AP_Mount_Topotek.h
 */

#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_TOPOTEK_ENABLED

#include "AP_Mount_Backend_Serial.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>

#define AP_MOUNT_TOPOTEK_PACKETLEN_MAX              36          // maximum number of bytes in a packet sent to or received from the gimbal
#define AP_MOUNT_RECV_GIMBAL_CMD_CATEGORIES_NUM     7           // parse the number of gimbal command types

/**
 * @class AP_Mount_Topotek
 * @brief Backend driver for Topotek gimbals using custom ASCII serial protocol
 * 
 * @details This class implements the ArduPilot mount backend for Topotek gimbals,
 *          supporting gimbal control, camera operations, tracking, and rangefinder
 *          integration through a custom ASCII serial protocol.
 * 
 *          Protocol Features:
 *          - ASCII-based protocol with "#tp" prefix for variable length packets
 *          - Bytewise parser with state machine for robust packet reception
 *          - Command handlers for: GIA (gimbal attitude), REC (recording),
 *            SDC (SD card status), LRF (laser rangefinder), TRC (tracking),
 *            VSN (version), PA2 (model name)
 *          - Camera controls: photo capture, video recording, zoom, focus
 *          - Tracking support: point and rectangle tracking modes
 *          - Rangefinder distance measurement integration
 * 
 *          Communication:
 *          - Periodic polling: attitude at 20Hz, rangefinder at 10Hz
 *          - Resend logic for critical commands (zoom/focus stop)
 *          - Acknowledgment handling with timeout detection
 * 
 *          Angle/Rate Units:
 *          - Angles: degrees (converted from/to radians internally)
 *          - Rates: deg/s (converted from/to rad/s internally)
 *          - Attitude reported in NED frame (roll, pitch, yaw)
 * 
 * @note Inherits from AP_Mount_Backend_Serial for serial communication
 * @warning Command acknowledgments may be lost; critical commands use resend logic
 * 
 * Source: libraries/AP_Mount/AP_Mount_Topotek.h
 */
class AP_Mount_Topotek : public AP_Mount_Backend_Serial
{

public:
    // Constructor
    using AP_Mount_Backend_Serial::AP_Mount_Backend_Serial;

    // Do not allow copies
    CLASS_NO_COPY(AP_Mount_Topotek);

    /**
     * @brief Update mount position and process incoming data - called periodically
     * 
     * @details Performs periodic tasks including:
     *          - Reading and parsing incoming packets from gimbal
     *          - Sending angle or rate targets to gimbal
     *          - Requesting gimbal status (attitude, tracking, version, etc.)
     *          - Managing tracking state and mode changes
     * 
     * @note Called at main loop rate (typically 50Hz)
     * @note Implements polling schedule for various gimbal information requests
     */
    void update() override;

    /**
     * @brief Check if gimbal communication is healthy
     * 
     * @details Returns true if recent attitude data has been received from gimbal
     *          (within timeout period), indicating active communication.
     * 
     * @return true if gimbal is responding and healthy, false otherwise
     */
    bool healthy() const override;

    /**
     * @brief Check if this mount can control pan/yaw axis
     * 
     * @details Required capability check for multicopters that need independent
     *          yaw control of the gimbal separate from vehicle heading.
     * 
     * @return true if gimbal has valid yaw range configured, false otherwise
     * 
     * @note Delegates to yaw_range_valid() to check parameter configuration
     */
    bool has_pan_control() const override { return yaw_range_valid(); };

    //
    // camera controls for gimbals
    //

    /**
     * @brief Trigger camera to take a picture
     * 
     * @details Sends photo capture command to gimbal using Topotek protocol.
     *          Command is sent immediately without confirmation.
     * 
     * @return true on successful command transmission, false if send fails
     * 
     * @note No acknowledgment is expected from gimbal for this command
     */
    bool take_picture() override;

    /**
     * @brief Start or stop video recording on gimbal camera
     * 
     * @details Sends recording control command to gimbal. Recording state is
     *          tracked locally and updated from gimbal status messages.
     * 
     * @param[in] start_recording true to start recording, false to stop recording
     * 
     * @return true on successful command transmission, false if send fails
     * 
     * @note Recording status is confirmed via REC status messages from gimbal
     */
    bool record_video(bool start_recording) override;

    /**
     * @brief Control camera zoom as a rate or absolute position
     * 
     * @details Supports continuous zoom rate control and absolute position setting.
     *          Stop commands (rate = 0) are resent to handle potential packet loss.
     * 
     * @param[in] zoom_type Type of zoom control (RATE or POSITION)
     * @param[in] zoom_value Zoom rate (typically -1 to +1) or absolute position
     * 
     * @return true on successful command transmission, false if send fails
     * 
     * @note Zero rate (stop) commands are tracked and resent to ensure gimbal stops
     */
    bool set_zoom(ZoomType zoom_type, float zoom_value) override;

    /**
     * @brief Control camera focus as rate, position, or auto mode
     * 
     * @details Supports continuous focus rate control, absolute position, and
     *          auto-focus modes. Stop commands are resent to handle packet loss.
     * 
     * @param[in] focus_type Type of focus control (RATE, POSITION, AUTO, etc.)
     * @param[in] focus_value Focus rate (in=-1, hold=0, out=+1) or position value
     * 
     * @return SetFocusResult indicating success, failure, or unsupported mode
     * 
     * @note Zero rate (stop) commands are tracked and resent to ensure focus holds
     */
    SetFocusResult set_focus(FocusType focus_type, float focus_value) override;

    /**
     * @brief Set visual tracking mode and target area
     * 
     * @details Configures gimbal tracking for point or rectangle target areas.
     *          Coordinates are normalized to 0.0-1.0 range where 0 is left/top
     *          and 1 is right/bottom of frame.
     * 
     * @param[in] tracking_type Type of tracking (NONE, POINT, or RECTANGLE)
     * @param[in] p1 Point target (POINT mode) or top-left corner (RECTANGLE mode)
     * @param[in] p2 Bottom-right corner (RECTANGLE mode only, ignored for POINT)
     * 
     * @return true on successful command transmission, false if send fails
     * 
     * @note p1 and p2 coordinates in range [0.0, 1.0] normalized to frame size
     * @note Tracking status confirmed via TRC status messages from gimbal
     */
    bool set_tracking(TrackingType tracking_type, const Vector2f& p1, const Vector2f& p2) override;

    /**
     * @brief Cancel active tracking on gimbal
     * 
     * @details Sends command to stop current tracking operation. Typically called
     *          automatically when mode changes or tracking is no longer desired.
     * 
     * @return true on successful command transmission, false if send fails
     * 
     * @note Equivalent to set_tracking(TrackingType::NONE, ...) 
     */
    bool cancel_tracking();

    /**
     * @brief Set camera lens mode or picture-in-picture configuration
     * 
     * @details Configures camera lens selection for multi-lens gimbals,
     *          including picture-in-picture display modes.
     * 
     * @param[in] lens Lens mode identifier (gimbal-specific encoding)
     * 
     * @return true on successful command transmission, false if send fails
     */
    bool set_lens(uint8_t lens) override;

#if HAL_MOUNT_SET_CAMERA_SOURCE_ENABLED
    /**
     * @brief Set camera source by type (primary and secondary)
     * 
     * @details Functionally equivalent to set_lens() but specifies lenses by
     *          type rather than mode number. Uses AP_Camera::CameraSource enum.
     * 
     * @param[in] primary_source Primary camera source type (AP_Camera::CameraSource enum)
     * @param[in] secondary_source Secondary camera source type for PiP mode
     * 
     * @return true on successful command transmission, false if send fails
     * 
     * @note Available only when HAL_MOUNT_SET_CAMERA_SOURCE_ENABLED is defined
     */
    bool set_camera_source(uint8_t primary_source, uint8_t secondary_source) override;
#endif

    /**
     * @brief Send MAVLink CAMERA_INFORMATION message to ground station
     * 
     * @details Provides static camera capabilities and identification to GCS,
     *          including model name and firmware version if available.
     * 
     * @param[in] chan MAVLink channel to send message on
     * 
     * @note Called periodically by mount manager for GCS telemetry
     */
    void send_camera_information(mavlink_channel_t chan) const override;

    /**
     * @brief Send MAVLink CAMERA_SETTINGS message to ground station
     * 
     * @details Provides current camera settings including recording state
     *          and other dynamic camera status information.
     * 
     * @param[in] chan MAVLink channel to send message on
     * 
     * @note Called periodically by mount manager for GCS telemetry
     */
    void send_camera_settings(mavlink_channel_t chan) const override;

    //
    // rangefinder
    //

    /**
     * @brief Get latest rangefinder distance measurement from gimbal
     * 
     * @details Returns most recently received laser rangefinder distance from
     *          gimbal's integrated rangefinder (if equipped and enabled).
     * 
     * @param[out] distance_m Distance measurement in meters (negative if invalid)
     * 
     * @return true if valid distance available, false if no measurement or disabled
     * 
     * @note Distance updated at ~10Hz when rangefinder enabled
     * @note Returns false if distance is negative (invalid measurement)
     */
    bool get_rangefinder_distance(float& distance_m) const override;

    /**
     * @brief Enable or disable gimbal's integrated rangefinder
     * 
     * @details Controls power state of gimbal's laser rangefinder. When enabled,
     *          distance measurements are requested at 10Hz rate.
     * 
     * @param[in] enable true to enable rangefinder, false to disable
     * 
     * @return true on successful command transmission, false if send fails
     * 
     * @note Rangefinder distance updates (LRF messages) only received when enabled
     */
    bool set_rangefinder_enable(bool enable) override;

protected:

    /**
     * @brief Get current gimbal attitude as a quaternion
     * 
     * @details Converts stored Euler angle representation (roll, pitch, yaw in NED frame)
     *          received from gimbal into quaternion form for attitude calculations.
     * 
     * @param[out] att_quat Quaternion representing current gimbal attitude in NED frame
     * 
     * @return true if attitude data is recent and valid, false if stale or unavailable
     * 
     * @note Attitude data received via GIA messages at ~20Hz when gimbal active
     * @note Returns false if no attitude update received within timeout period
     */
    bool get_attitude_quaternion(Quaternion& att_quat) override;

private:

    // header type (fixed or variable length)
    // first three bytes of packet determined by this value
    enum class HeaderType : uint8_t {
        FIXED_LEN = 0x00,       // #TP will be sent
        VARIABLE_LEN = 0x01,    // #tp will be sent
    };

    // address (2nd and 3rd bytes of packet)
    // first byte is always U followed by one of the other options
    enum class AddressByte : uint8_t {
        SYSTEM_AND_IMAGE = 68,      // 'D'
        AUXILIARY_EQUIPMENT = 69,   // 'E'
        GIMBAL = 71,                // 'G'
        LENS = 77,                  // 'M'
        NETWORK = 80,               // 'P'
        UART = 85,                  // 'U'
    };

    // control byte (read or write)
    // sent as 7th byte of packet
    enum class ControlByte : uint8_t {
        READ = 114,     // 'r'
        WRITE = 119,    // 'w'
    };

    /**
     * @brief ASCII protocol bytewise parser state machine
     * 
     * @details Parser states for processing incoming Topotek ASCII protocol packets.
     *          Protocol format: "#tp" + address(2) + len(1) + ctrl(1) + id(3) + data + crc(2)
     *          Parser advances through states as each byte is received and validated.
     * 
     * @note Parser handles variable-length packets with ASCII-encoded hex data
     * @note Resets to WAITING_FOR_HEADER1 on any parsing error or after complete packet
     */
    enum class ParseState : uint8_t {
        WAITING_FOR_HEADER1 = 0,///< Waiting for '#' character (byte 0)
        WAITING_FOR_HEADER2,    ///< Waiting for 'T' or 't' character (byte 1)
        WAITING_FOR_HEADER3,    ///< Waiting for 'P' or 'p' character (byte 2)
        WAITING_FOR_ADDR1,      ///< Waiting for first address byte, normally 'U' (byte 3)
        WAITING_FOR_ADDR2,      ///< Waiting for second address byte: 'M', 'D', 'E', 'P', or 'G' (byte 4)
        WAITING_FOR_DATALEN,    ///< Waiting for data length byte (byte 5)
        WAITING_FOR_CONTROL,    ///< Waiting for control byte: 'r' (read) or 'w' (write) (byte 6)
        WAITING_FOR_ID1,        ///< Waiting for first ID character, e.g., 'G' in "GIA" (byte 7)
        WAITING_FOR_ID2,        ///< Waiting for second ID character, e.g., 'I' in "GIA" (byte 8)
        WAITING_FOR_ID3,        ///< Waiting for third ID character, e.g., 'A' in "GIA" (byte 9)
        WAITING_FOR_DATA,       ///< Waiting for data bytes (hex ASCII, e.g., '0', 'A') (bytes 10+)
        WAITING_FOR_CRC_LOW,    ///< Waiting for CRC low byte (ASCII hex)
        WAITING_FOR_CRC_HIGH,   ///< Waiting for CRC high byte (ASCII hex)
    };

    // tracking status
    enum class TrackingStatus : uint8_t {
        STOPPED_TRACKING = 0x30,                // not tracking
        WAITING_FOR_TRACKING = 0x31,            // wait to track command status
        TRACKING_IN_PROGRESS = 0x32             // the status is being tracked.
    };

    // identifier bytes
    typedef char Identifier[3];

    // send text prefix string
    static const char* send_message_prefix;

    /**
     * @brief Read and parse incoming packets from gimbal serial port
     * 
     * @details Processes available bytes from serial port using bytewise state machine
     *          parser. Validates packet format, calculates/checks CRC, and dispatches
     *          complete packets to appropriate handler based on command ID.
     * 
     * @note Called from update() at main loop rate to process incoming data
     * @note Parser resets on any format error and continues with next byte
     * @note Dispatches to handler functions via uart_recv_cmd_compare_list lookup table
     */
    void read_incoming_packets();

    /**
     * @brief Request current gimbal attitude (roll, pitch, yaw)
     * 
     * @details Sends GIA (Gimbal Attitude) query command to request current
     *          gimbal orientation. Response processed by gimbal_angle_analyse().
     * 
     * @note Typically called at 20Hz for continuous attitude updates
     * @note Response provides angles in degrees (converted to radians internally)
     */
    void request_gimbal_attitude();

    /**
     * @brief Request gimbal SD card status information
     * 
     * @details Sends SDC (SD Card) query command to check memory card presence
     *          and status. Response processed by gimbal_sdcard_analyse().
     * 
     * @note Called periodically (lower rate than attitude) to monitor storage
     */
    void request_gimbal_sdcard_info();

    /**
     * @brief Request current gimbal tracking status
     * 
     * @details Sends TRC (Tracking) query command to get current tracking state
     *          (stopped, waiting, or active). Response processed by gimbal_track_analyse().
     * 
     * @note Called periodically to monitor tracking state for status reporting
     */
    void request_track_status();

    /**
     * @brief Request gimbal firmware version information
     * 
     * @details Sends VSN (Version) query command to retrieve firmware version.
     *          Response processed by gimbal_version_analyse(). Typically requested
     *          once at startup.
     * 
     * @note Version used for capabilities detection and GCS reporting
     */
    void request_gimbal_version();

    /**
     * @brief Request gimbal model name/identifier
     * 
     * @details Sends PA2 (Model Name) query command to retrieve gimbal model string.
     *          Response processed by gimbal_model_name_analyse(). Typically requested
     *          once at startup.
     * 
     * @note Model name used for identification and GCS CAMERA_INFORMATION messages
     */
    void request_gimbal_model_name();

    /**
     * @brief Send target angle to gimbal for angle control mode
     * 
     * @details Converts angle target from radians to degrees and sends gimbal
     *          positioning command. Handles lock vs follow mode and respects
     *          configured angle limits.
     * 
     * @param[in] angle_rad Target angles in radians (roll, pitch, yaw in NED frame)
     * 
     * @note Angles converted to degrees (gimbal's native unit) before transmission
     * @note Called when mount mode requires angle/position control
     */
    void send_angle_target(const MountTarget& angle_rad);

    /**
     * @brief Send target rate to gimbal for rate control mode
     * 
     * @details Converts rate target from rad/s to deg/s and sends gimbal rate
     *          command. Implements stop command resend logic to handle packet loss.
     * 
     * @param[in] rate_rads Target rates in rad/s (roll, pitch, yaw rates)
     * 
     * @note Rates converted to deg/s (gimbal's native unit) before transmission
     * @note Zero rates (stop commands) resent multiple times for reliability
     * @note Called when mount mode requires rate/velocity control
     */
    void send_rate_target(const MountTarget& rate_rads);

    /**
     * @brief Send current system time and date to gimbal
     * 
     * @details Sends current UTC time/date for gimbal's internal clock and
     *          geotagging functionality. Sent periodically after startup.
     * 
     * @return true on successful transmission, false if send fails
     * 
     * @note Time format follows Topotek protocol specification
     * @note Sent multiple times at startup to ensure gimbal receives it
     */
    bool send_time_to_gimbal();

    /**
     * @brief Send GPS position information to gimbal
     * 
     * @details Provides current vehicle GPS coordinates to gimbal for
     *          geotagging and awareness of vehicle location.
     * 
     * @return true on successful transmission, false if send fails or no GPS fix
     * 
     * @note Only sent when valid GPS fix available
     */
    bool send_location_info();

    /**
     * @brief Parse and process gimbal attitude (GIA) response message
     * 
     * @details Extracts roll, pitch, yaw angles in degrees from GIA message,
     *          converts to radians, and updates internal attitude state.
     * 
     * @note Called by packet dispatcher when GIA message received
     * @note Updates _current_angle_rad and _last_current_angle_ms
     */
    void gimbal_angle_analyse();

    /**
     * @brief Parse and process video recording (REC) status message
     * 
     * @details Extracts current recording state from REC message and updates
     *          internal recording status flag.
     * 
     * @note Called by packet dispatcher when REC message received
     * @note Updates _recording member variable
     */
    void gimbal_record_analyse();

    /**
     * @brief Parse and process SD card (SDC) status message
     * 
     * @details Extracts SD card presence/status from SDC message and updates
     *          internal storage status flag.
     * 
     * @note Called by packet dispatcher when SDC message received
     * @note Updates _sdcard_status member variable
     */
    void gimbal_sdcard_analyse();

    /**
     * @brief Parse and process tracking (TRC) status message
     * 
     * @details Extracts current tracking state (stopped/waiting/active) from
     *          TRC message and updates internal tracking status.
     * 
     * @note Called by packet dispatcher when TRC message received
     * @note Updates _last_tracking_state member variable
     */
    void gimbal_track_analyse();

    /**
     * @brief Parse and process firmware version (VSN) message
     * 
     * @details Extracts firmware version number from VSN message and stores
     *          for capability detection and GCS reporting.
     * 
     * @note Called by packet dispatcher when VSN message received
     * @note Updates _firmware_ver and sets _got_gimbal_version flag
     */
    void gimbal_version_analyse();

    /**
     * @brief Parse and process model name (PA2) message
     * 
     * @details Extracts gimbal model name string from PA2 message and stores
     *          for identification and GCS CAMERA_INFORMATION reporting.
     * 
     * @note Called by packet dispatcher when PA2 message received
     * @note Updates _model_name array and sets _got_gimbal_model_name flag
     */
    void gimbal_model_name_analyse();

    /**
     * @brief Parse and process laser rangefinder (LRF) distance message
     * 
     * @details Extracts rangefinder distance measurement in meters from LRF
     *          message and updates internal distance variable.
     * 
     * @note Called by packet dispatcher when LRF message received
     * @note Updates _measure_dist_m member variable
     * @note Distance in meters, negative value indicates invalid measurement
     */
    void gimbal_dist_info_analyse();

    /**
     * @brief Calculate checksum for Topotek protocol packet
     * 
     * @details Computes checksum by summing all bytes from header through data,
     *          then converting result to 2-byte ASCII hex representation (high byte first).
     * 
     * @param[in] cmd Pointer to command buffer to checksum
     * @param[in] len Number of bytes to include in checksum
     * 
     * @return Calculated checksum value (8-bit sum)
     * 
     * @note Checksum transmitted as ASCII hex characters, not binary
     */
    uint8_t calculate_crc(const uint8_t *cmd, uint8_t len) const;

    /**
     * @brief Convert hexadecimal nibble to ASCII character
     * 
     * @details Converts 4-bit hex value (0-15) to ASCII character ('0'-'9', 'A'-'F').
     *          Used for encoding numeric data in protocol's ASCII format.
     * 
     * @param[in] data Hex nibble value (0-15)
     * 
     * @return ASCII character representing hex value ('0'-'9' or 'A'-'F')
     */
    uint8_t hex2char(uint8_t data) const;

    /**
     * @brief Convert 4-character ASCII hex string to 16-bit integer
     * 
     * @details Parses 4 ASCII hex characters into signed 16-bit integer value.
     *          Characters represent hex digits with most significant digit first.
     * 
     * @param[in] high Most significant hex digit character (e.g., '1' in "1234")
     * @param[in] mid_high Second hex digit character (e.g., '2' in "1234")
     * @param[in] mid_low Third hex digit character (e.g., '3' in "1234")
     * @param[in] low Least significant hex digit character (e.g., '4' in "1234")
     * 
     * @return Parsed 16-bit signed integer value
     * 
     * @note Used for parsing numeric fields in gimbal response messages
     */
    int16_t hexchar4_to_int16(char high, char mid_high, char mid_low, char low) const;

    /**
     * @brief Send fixed-length packet to gimbal
     * 
     * @details Constructs and transmits fixed-length format packet ("#TP" prefix)
     *          with single-byte value payload. Used for simple commands.
     * 
     * @param[in] address Destination address byte (system/lens/gimbal/etc.)
     * @param[in] id 3-character command identifier (e.g., "GIA", "REC")
     * @param[in] write true for write/control command, false for read/query
     * @param[in] value Single data byte value to send
     * 
     * @return true on successful transmission, false if serial port unavailable
     */
    bool send_fixedlen_packet(AddressByte address, const Identifier id, bool write, uint8_t value);

    /**
     * @brief Send variable-length packet to gimbal
     * 
     * @details Constructs and transmits variable-length format packet ("#tp" prefix)
     *          with arbitrary data payload. Used for complex commands with parameters.
     * 
     * @param[in] header Header type (fixed or variable length indicator)
     * @param[in] address Destination address byte (system/lens/gimbal/etc.)
     * @param[in] id 3-character command identifier (e.g., "GIA", "REC")
     * @param[in] write true for write/control command, false for read/query
     * @param[in] databuff Pointer to data buffer (ASCII hex encoded)
     * @param[in] databuff_len Number of bytes in data buffer
     * 
     * @return true on successful transmission, false if serial port unavailable
     * 
     * @note databuff should contain ASCII hex characters, not binary data
     * @note Automatically calculates and appends checksum
     */
    bool send_variablelen_packet(HeaderType header, AddressByte address, const Identifier id, bool write, const uint8_t* databuff, uint8_t databuff_len);

    /**
     * @brief Set gimbal lock vs follow mode
     * 
     * @details Controls whether gimbal maintains earth-frame (lock) or body-frame
     *          (follow) orientation target. Lock mode stabilizes gimbal relative to
     *          world, follow mode moves gimbal with vehicle body.
     * 
     * @param[in] lock true for earth-frame lock mode, false for body-frame follow mode
     * 
     * @return true on successful command transmission, false if send fails
     * 
     * @note Lock mode essential for maintaining target pointing during vehicle maneuvers
     * @note Mode changes tracked in _last_lock to avoid redundant commands
     */
    bool set_gimbal_lock(bool lock);

    // Camera and gimbal state members
    bool _recording;                                            ///< Video recording status received from gimbal (REC message)
    bool _is_tracking;                                          ///< Local tracking enable flag (set by set_tracking command)
    TrackingStatus _last_tracking_state = TrackingStatus::STOPPED_TRACKING; ///< Last tracking state received from gimbal (TRC message)
    uint8_t _last_mode;                                         ///< Mount mode during last update, used to detect mode changes and cancel tracking
    bool _sdcard_status;                                        ///< SD card presence/status received from gimbal (SDC message)
    bool _last_lock;                                            ///< Last lock mode sent to gimbal (true=earth-frame, false=body-frame)
    bool _got_gimbal_version;                                   ///< true once gimbal firmware version received (VSN message)
    bool _got_gimbal_model_name;                                ///< true once gimbal model name received (PA2 message)
    bool _last_zoom_stop;                                       ///< true if last zoom command was stop (rate=0); triggers resend for reliability
    bool _last_focus_stop;                                      ///< true if last focus command was stop (rate=0); triggers resend for reliability
    uint8_t _model_name[16];                                    ///< Gimbal model name string received from PA2 message
    uint8_t _sent_time_count;                                   ///< Number of time/date messages sent to gimbal (sent multiple times at startup)
    uint32_t _firmware_ver;                                     ///< Gimbal firmware version number received from VSN message
    
    // Attitude tracking members
    Vector3f _current_angle_rad;                                ///< Current gimbal angles in radians (x=roll, y=pitch, z=yaw in NED frame) from GIA message
    uint32_t _last_current_angle_ms;                            ///< System time (ms) when attitude data last received; used for health/timeout checks
    uint32_t _last_req_current_info_ms;                         ///< System time (ms) when info request last sent; drives periodic request scheduling
    uint8_t _last_req_step;                                     ///< Request loop step counter (0-9 at 10Hz); different requests sent at specific steps
    uint8_t _stop_order_count;                                  ///< Number of consecutive stop commands sent; used to resend stops for reliability
    
    // Rangefinder members
    float _measure_dist_m = -1.0f;                              ///< Latest rangefinder distance in meters from LRF message (negative = invalid)
    
    // Packet parsing members
    uint8_t _msg_buff[AP_MOUNT_TOPOTEK_PACKETLEN_MAX];          ///< Buffer accumulating bytes from current incoming packet (for CRC calculation)
    uint8_t _msg_buff_len;                                      ///< Number of bytes currently in _msg_buff
    struct {
        ParseState state;                                       ///< Current bytewise parser state machine position
        uint8_t data_len;                                       ///< Expected data payload length from packet header (byte 5)
    } _parser;                                                  ///< Parser state structure for incoming packet processing

    // mapping from received message key to member function pointer to consume the message
    typedef struct {
        uint8_t uart_cmd_key[4];                                // gimbal message key;
        void (AP_Mount_Topotek::*func)(void);		            // member function to consume messager
    } UartCmdFunctionHandler;

    // stores command ID and corresponding member functions that are compared with the command received by the gimbal
    UartCmdFunctionHandler uart_recv_cmd_compare_list[AP_MOUNT_RECV_GIMBAL_CMD_CATEGORIES_NUM] = {
        {{"GIA"}, &AP_Mount_Topotek::gimbal_angle_analyse},
        {{"REC"}, &AP_Mount_Topotek::gimbal_record_analyse},
        {{"SDC"}, &AP_Mount_Topotek::gimbal_sdcard_analyse},
        {{"LRF"}, &AP_Mount_Topotek::gimbal_dist_info_analyse},
        {{"TRC"}, &AP_Mount_Topotek::gimbal_track_analyse},
        {{"VSN"}, &AP_Mount_Topotek::gimbal_version_analyse},
        {{"PA2"}, &AP_Mount_Topotek::gimbal_model_name_analyse}
    };
};

#endif // HAL_MOUNT_TOPOTEK_ENABLED
