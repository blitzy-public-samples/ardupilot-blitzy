/**
 * @file AP_Mount_Alexmos.h
 * @brief Alexmos/BaseCam SimpleBGC serial protocol gimbal mount backend
 * 
 * @details This file implements the Alexmos/BaseCam SimpleBGC serial protocol backend
 *          for gimbal control. The SimpleBGC protocol uses ASCII-like framing with
 *          '>' as the start byte, followed by command ID, data size, header checksum,
 *          data payload, and data checksum. This backend supports controlling 2-axis
 *          and 3-axis gimbals manufactured by Basecam Electronics and compatible devices.
 * 
 * @note Protocol details: https://www.basecamelectronics.com/serialapi/
 * @note Default baudrate: 115200 bps (configurable via gimbal settings)
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */
#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_ALEXMOS_ENABLED

#include "AP_Mount_Backend.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

/**
 * @class AP_Mount_Alexmos
 * @brief Mount backend for Alexmos/BaseCam SimpleBGC serial protocol gimbals
 * 
 * @details This class implements communication with Alexmos/BaseCam gimbals using the
 *          SimpleBGC serial protocol. The protocol uses binary framing with the following structure:
 *          - Start byte: '>' (0x3E)
 *          - Command ID: 1 byte identifying the command type
 *          - Data size: 1 byte indicating payload length
 *          - Header checksum: 1 byte (command_id + data_size)
 *          - Data payload: variable length based on command
 *          - Data checksum: 1 byte (sum of all data bytes)
 * 
 *          Supported commands include:
 *          - CMD_BOARD_INFO: Query firmware version and board capabilities
 *          - CMD_GET_ANGLES: Read current gimbal angles
 *          - CMD_CONTROL: Send target angles and speeds to gimbal
 *          - CMD_READ_PARAMS: Read gimbal PID and configuration parameters
 *          - CMD_WRITE_PARAMS: Write gimbal configuration parameters
 * 
 *          Angle units: Protocol uses 0.02197 degrees per unit (16384 units = 360 degrees)
 *          Speed units: Fixed at 30 degrees/second for control commands
 * 
 * @note This implementation supports SimpleBGC protocol version 2.x
 * @note Gimbal must be configured to match the serial port baudrate (typically 115200)
 * @note Not all SimpleBGC commands are implemented - only those required for mount control
 * 
 * @warning Incorrect baudrate configuration will result in communication failure
 */
class AP_Mount_Alexmos : public AP_Mount_Backend
{
public:
    /**
     * @brief Constructor - inherits from AP_Mount_Backend
     * @details Uses the parent class constructor to initialize the mount instance
     */
    using AP_Mount_Backend::AP_Mount_Backend;

    /**
     * @brief Initialize the Alexmos gimbal backend
     * 
     * @details Performs initialization of the serial connection to the gimbal and queries
     *          board information (firmware version, capabilities). This method sets up the
     *          UART port and sends initial commands to establish communication with the gimbal.
     *          Called once during mount subsystem initialization.
     * 
     * @note This method must be called before update() or any control commands
     * @note Requires valid serial port configuration in mount parameters
     */
    void init() override;

    /**
     * @brief Update mount position - called periodically by mount controller
     * 
     * @details Main update loop for the Alexmos gimbal backend. This method:
     *          - Processes incoming serial data from the gimbal
     *          - Reads current gimbal angles and status
     *          - Sends target angle commands based on pilot input or mission commands
     *          - Handles parameter read/write requests
     *          Should be called at approximately 10-50 Hz by the mount library.
     * 
     * @note This is called from the main scheduler at the mount update rate
     * @note All serial communication happens within this method and read_incoming()
     */
    void update() override;

    /**
     * @brief Check if this mount can control pan/yaw axis
     * 
     * @details Indicates whether the gimbal has pan (yaw) control capability. This is determined
     *          by querying the gimbal's board info (3-axis capability flag). Pan control is
     *          required for certain flight modes on multicopters where the vehicle may need to
     *          yaw independently of the camera.
     * 
     * @return true if gimbal supports 3-axis control (roll, pitch, yaw), false for 2-axis only
     * 
     * @note 2-axis gimbals typically control only roll and pitch
     * @note 3-axis capability is determined from CMD_BOARD_INFO response
     */
    bool has_pan_control() const override;

protected:

    /**
     * @brief Get current gimbal attitude as a quaternion
     * 
     * @details Retrieves the current gimbal attitude and converts it to a quaternion representation.
     *          Uses the last known gimbal angles (roll, pitch, yaw) received from CMD_GET_ANGLES
     *          or real-time data responses. Angles are converted from the protocol units
     *          (0.02197 degrees per unit) to radians and then to a quaternion.
     * 
     * @param[out] att_quat Quaternion representing the gimbal's current attitude in body frame
     * 
     * @return true if valid attitude data is available, false if no recent data or invalid
     * 
     * @note This method uses cached angle data from the last successful gimbal response
     * @note Coordinate frame: Body frame (NED convention) relative to vehicle
     */
    bool get_attitude_quaternion(Quaternion& att_quat) override;

private:

    /**
     * @brief Request current gimbal angles from the device
     * 
     * @details Sends CMD_GET_ANGLES command to the gimbal to request current roll, pitch, and yaw
     *          angles. The response will be processed asynchronously by parse_body(). This provides
     *          the actual gimbal orientation including any RC override or stabilization adjustments.
     * 
     * @note Response is processed in parse_body() when received
     * @note Angles are in protocol units (0.02197 degrees per unit)
     */
    void get_angles();

    /**
     * @brief Activate or deactivate gimbal motors
     * 
     * @details Sends CMD_MOTORS_ON or CMD_MOTORS_OFF command to enable or disable the gimbal
     *          stabilization motors. When motors are off, the gimbal is free-moving. When on,
     *          the gimbal actively stabilizes based on its internal IMU and control settings.
     * 
     * @param[in] on true to activate motors (stabilization on), false to deactivate (free-moving)
     * 
     * @note Motor state change may take several hundred milliseconds
     * @note Disabling motors while in flight may cause uncontrolled gimbal movement
     * @warning Only disable motors when vehicle is on the ground unless testing
     */
    void set_motor(bool on);

    /**
     * @brief Query gimbal board version and firmware information
     * 
     * @details Sends CMD_BOARD_INFO command to retrieve:
     *          - Board hardware version
     *          - Firmware version (major.minor)
     *          - Beta version number
     *          - Board features flags (3-axis capable, battery monitoring, etc.)
     *          This information is used to determine gimbal capabilities and protocol compatibility.
     * 
     * @note Response is processed in parse_body() when received
     * @note Called during init() to identify gimbal capabilities
     */
    void get_boardinfo();

    /**
     * @brief Send target angles to gimbal for movement
     * 
     * @details Sends CMD_CONTROL command with target roll, pitch, yaw angles to command gimbal
     *          movement. Movement speed is fixed at 30 degrees/second. Angles are converted from
     *          radians (input) to protocol units (0.02197 degrees per unit) before transmission.
     * 
     * @param[in] angle_target_rad Target gimbal angles in radians (roll, pitch, yaw) in body frame
     * 
     * @note Speed is fixed at 30 deg/s and cannot be adjusted per-command
     * @note Angles are relative to vehicle body frame (NED convention)
     * @note Called from update() based on pilot input or mission commands
     */
    void control_axis(const MountTarget& angle_target_rad);

    /**
     * @brief Read gimbal configuration parameters
     * 
     * @details Sends CMD_READ_PARAMS command to retrieve complete gimbal configuration including:
     *          - PID tuning values for all axes (P, I, D gains)
     *          - Motor power settings
     *          - RC input configuration and mixing
     *          - Follow mode settings
     *          - Battery monitoring thresholds
     *          Response contains the full parameter set for the specified profile.
     * 
     * @param[in] profile_id Gimbal profile number to read (typically 0-4)
     * 
     * @note Response is processed in parse_body() and stored in _current_parameters
     * @note Parameter structure is large (200+ bytes)
     * @note Reading parameters is typically done once during initialization
     */
    void read_params(uint8_t profile_id);

    /**
     * @brief Write gimbal configuration parameters
     * 
     * @details Sends CMD_WRITE_PARAMS command to update gimbal configuration. Parameters must be
     *          previously loaded into _current_parameters structure. This writes the complete
     *          parameter set to the gimbal's non-volatile memory.
     * 
     * @note Parameters must be read with read_params() first, then modified, then written
     * @note Writing parameters causes gimbal to save to EEPROM (limited write cycles)
     * @note Gimbal may temporarily stop stabilization during parameter write
     * @warning Incorrect parameters can cause unstable gimbal behavior or motor damage
     */
    void write_params();

    /**
     * @brief Request real-time gimbal data
     * 
     * @details Sends CMD_GET_ANGLES_EXT command to retrieve extended real-time data including
     *          IMU sensor readings and current angles with higher precision. This is an alternative
     *          to get_angles() that provides additional diagnostic information.
     * 
     * @param[out] angle Vector3f to store retrieved angles (roll, pitch, yaw) in degrees
     * 
     * @return true if real-time data was successfully retrieved, false otherwise
     * 
     * @note Not all gimbal firmware versions support CMD_GET_ANGLES_EXT
     * @note May be used for diagnostic purposes or higher-rate angle updates
     */
    bool get_realtimedata(Vector3f& angle);

    /**
     * @brief Send a command to the Alexmos gimbal via serial protocol
     * 
     * @details Formats and sends a SimpleBGC protocol command over the serial port. The command
     *          structure includes:
     *          - Start byte '>' (0x3E)
     *          - Command ID byte
     *          - Data size byte
     *          - Header checksum (command_id + data_size)
     *          - Data payload (if size > 0)
     *          - Data checksum (sum of all data bytes)
     * 
     * @param[in] cmd Command ID byte (e.g., CMD_CONTROL, CMD_GET_ANGLES)
     * @param[in] data Pointer to data payload buffer, NULL if no data
     * @param[in] size Number of bytes in data payload (0 if no data)
     * 
     * @note This is the low-level protocol transmission function
     * @note All commands go through this method for consistent framing
     * @note Checksums are calculated automatically
     */
    void send_command(uint8_t cmd, uint8_t* data, uint8_t size);

    /**
     * @brief Parse the data payload of received gimbal message
     * 
     * @details Processes the data payload based on the command ID after a complete message has
     *          been received. Handles responses for:
     *          - CMD_BOARD_INFO: Extracts firmware version and capabilities
     *          - CMD_GET_ANGLES: Extracts current gimbal angles
     *          - CMD_READ_PARAMS: Extracts complete parameter set
     *          - CMD_CONFIRM: Confirms previous command execution
     *          Data is stored in _buffer union and may be copied to _current_parameters.
     * 
     * @note Called by read_incoming() after complete message reception
     * @note Message data is in _buffer union at time of call
     * @note Invalid checksums result in message being discarded before this is called
     */
    void parse_body();

    /**
     * @brief Read and parse incoming serial data from gimbal
     * 
     * @details State machine that processes incoming bytes from the gimbal serial port.
     *          Implements the SimpleBGC protocol framing:
     *          1. Wait for start byte '>'
     *          2. Read command ID and data size
     *          3. Verify header checksum
     *          4. Read data payload bytes
     *          5. Verify data checksum
     *          6. Call parse_body() if message valid
     * 
     * @note Called from update() to process received messages
     * @note Uses _step state variable to track parsing progress
     * @note Resets to start state if any checksum fails
     * @note All received data is stored in _buffer until message complete
     */
    void read_incoming();

    /**
     * @brief SimpleBGC serial protocol data structures
     * @details These structures define the binary data formats for SimpleBGC protocol messages
     */

    /**
     * @brief Response structure for CMD_BOARD_INFO command
     * 
     * @details Contains gimbal board and firmware identification information returned
     *          when CMD_BOARD_INFO is sent to the gimbal. Used to determine board
     *          capabilities and protocol compatibility.
     */
    struct PACKED alexmos_version {
        uint8_t _board_version;        ///< Board hardware version number
        uint16_t _firmware_version;    ///< Firmware version (major * 10 + minor, e.g., 26 = v2.6)
        uint8_t debug_mode;            ///< Debug mode flags (typically 0 in production)
        uint16_t _board_features;      ///< Feature flags: bit 0 = 3-axis capable, bit 1 = battery monitoring
    };

    /**
     * @brief Response structure for CMD_GET_ANGLES command
     * 
     * @details Contains current gimbal angles and RC (remote control) input values for all three axes.
     *          Angles are in protocol units where 1 unit = 0.02197 degrees (16384 units = 360 degrees).
     *          This structure provides both the actual gimbal position and any RC override values.
     * 
     * @note Angle units: 0.02197 degrees per unit
     * @note Speed units: degrees/second (for rc_speed fields)
     */
    struct PACKED alexmos_angles {
        int16_t angle_roll;       ///< Current roll angle in protocol units (0.02197 deg/unit)
        int16_t rc_angle_roll;    ///< RC commanded roll angle in protocol units
        int16_t rc_speed_roll;    ///< RC commanded roll speed in deg/s
        int16_t angle_pitch;      ///< Current pitch angle in protocol units (0.02197 deg/unit)
        int16_t rc_angle_pitch;   ///< RC commanded pitch angle in protocol units
        int16_t rc_speed_pitch;   ///< RC commanded pitch speed in deg/s
        int16_t angle_yaw;        ///< Current yaw angle in protocol units (0.02197 deg/unit)
        int16_t rc_angle_yaw;     ///< RC commanded yaw angle in protocol units
        int16_t rc_speed_yaw;     ///< RC commanded yaw speed in deg/s
    };

    /**
     * @brief Command structure for CMD_CONTROL command
     * 
     * @details Defines target angles and speeds for commanding gimbal movement. Sent to the gimbal
     *          to request movement to specific angles at specified speeds. Speed is typically fixed
     *          at 30 degrees/second in this implementation.
     * 
     * @note Angle units: 0.02197 degrees per unit (16384 units = 360 degrees)
     * @note Speed units: degrees/second
     * @note Mode byte selects control mode (typically 2 for angle control)
     */
    struct PACKED alexmos_angles_speed {
        int8_t mode;          ///< Control mode: 0=no control, 1=speed, 2=angle
        int16_t speed_roll;   ///< Target roll speed in deg/s (typically 30)
        int16_t angle_roll;   ///< Target roll angle in protocol units (0.02197 deg/unit)
        int16_t speed_pitch;  ///< Target pitch speed in deg/s (typically 30)
        int16_t angle_pitch;  ///< Target pitch angle in protocol units (0.02197 deg/unit)
        int16_t speed_yaw;    ///< Target yaw speed in deg/s (typically 30)
        int16_t angle_yaw;    ///< Target yaw angle in protocol units (0.02197 deg/unit)
    };

    /**
     * @brief Command/response structure for CMD_READ_PARAMS and CMD_WRITE_PARAMS
     * 
     * @details Complete gimbal configuration parameter structure containing all tuning and
     *          configuration settings. This includes PID controller gains, motor power settings,
     *          RC input configuration, follow mode parameters, and battery monitoring thresholds.
     *          Structure is approximately 215 bytes and represents a complete configuration profile.
     * 
     * @note This structure matches SimpleBGC firmware v2.x parameter layout
     * @note Parameters are stored per-profile (gimbals typically support 5 profiles)
     * @note Writing parameters saves to gimbal's non-volatile memory (EEPROM)
     * @warning Incorrect PID values can cause unstable gimbal behavior or motor damage
     */
    struct PACKED alexmos_params {
        uint8_t profile_id;           ///< Profile number (0-4)
        
        // Roll axis PID and motor configuration
        uint8_t roll_P;               ///< Roll axis P gain (proportional)
        uint8_t roll_I;               ///< Roll axis I gain (integral)
        uint8_t roll_D;               ///< Roll axis D gain (derivative)
        uint8_t roll_power;           ///< Roll motor power (0-255)
        uint8_t roll_invert;          ///< Roll motor direction invert flag
        uint8_t roll_poles;           ///< Roll motor pole count
        
        // Pitch axis PID and motor configuration
        uint8_t pitch_P;              ///< Pitch axis P gain (proportional)
        uint8_t pitch_I;              ///< Pitch axis I gain (integral)
        uint8_t pitch_D;              ///< Pitch axis D gain (derivative)
        uint8_t pitch_power;          ///< Pitch motor power (0-255)
        uint8_t pitch_invert;         ///< Pitch motor direction invert flag
        uint8_t pitch_poles;          ///< Pitch motor pole count
        
        // Yaw axis PID and motor configuration
        uint8_t yaw_P;                ///< Yaw axis P gain (proportional)
        uint8_t yaw_I;                ///< Yaw axis I gain (integral)
        uint8_t yaw_D;                ///< Yaw axis D gain (derivative)
        uint8_t yaw_power;            ///< Yaw motor power (0-255)
        uint8_t yaw_invert;           ///< Yaw motor direction invert flag
        uint8_t yaw_poles;            ///< Yaw motor pole count
        uint8_t acc_limiter;          ///< Acceleration limiter (0-255)
        int8_t ext_fc_gain_roll;      ///< External flight controller gain for roll
        int8_t ext_fc_gain_pitch;     ///< External flight controller gain for pitch
        
        // Roll axis RC input configuration
        int16_t roll_rc_min_angle;    ///< Roll RC minimum angle limit (protocol units)
        int16_t roll_rc_max_angle;    ///< Roll RC maximum angle limit (protocol units)
        uint8_t roll_rc_mode;         ///< Roll RC mode (0=angle, 1=speed)
        uint8_t roll_rc_lpf;          ///< Roll RC input low-pass filter
        uint8_t roll_rc_speed;        ///< Roll RC speed setting
        uint8_t roll_rc_follow;       ///< Roll RC follow mode enable
        
        // Pitch axis RC input configuration
        int16_t pitch_rc_min_angle;   ///< Pitch RC minimum angle limit (protocol units)
        int16_t pitch_rc_max_angle;   ///< Pitch RC maximum angle limit (protocol units)
        uint8_t pitch_rc_mode;        ///< Pitch RC mode (0=angle, 1=speed)
        uint8_t pitch_rc_lpf;         ///< Pitch RC input low-pass filter
        uint8_t pitch_rc_speed;       ///< Pitch RC speed setting
        uint8_t pitch_rc_follow;      ///< Pitch RC follow mode enable
        
        // Yaw axis RC input configuration
        int16_t yaw_rc_min_angle;     ///< Yaw RC minimum angle limit (protocol units)
        int16_t yaw_rc_max_angle;     ///< Yaw RC maximum angle limit (protocol units)
        uint8_t yaw_rc_mode;          ///< Yaw RC mode (0=angle, 1=speed)
        uint8_t yaw_rc_lpf;           ///< Yaw RC input low-pass filter
        uint8_t yaw_rc_speed;         ///< Yaw RC speed setting
        uint8_t yaw_rc_follow;        ///< Yaw RC follow mode enable
        // Sensor and system configuration
        uint8_t gyro_trust;           ///< Gyroscope trust level (0-255)
        uint8_t use_model;            ///< Use mathematical model flag
        uint8_t pwm_freq;             ///< PWM frequency for motor control
        uint8_t serial_speed;         ///< Serial port baudrate index
        
        // RC trim and expo settings
        int8_t rc_trim_roll;          ///< Roll RC trim offset
        int8_t rc_trim_pitch;         ///< Pitch RC trim offset
        int8_t rc_trim_yaw;           ///< Yaw RC trim offset
        uint8_t rc_deadband;          ///< RC input deadband
        uint8_t rc_expo_rate;         ///< RC exponential rate
        uint8_t rc_virt_mode;         ///< RC virtual mode setting
        
        // RC channel mapping
        uint8_t rc_map_roll;          ///< RC channel mapped to roll
        uint8_t rc_map_pitch;         ///< RC channel mapped to pitch
        uint8_t rc_map_yaw;           ///< RC channel mapped to yaw
        uint8_t rc_map_cmd;           ///< RC channel mapped to commands
        uint8_t rc_map_fc_roll;       ///< RC channel mapped to FC roll
        uint8_t rc_map_fc_pitch;      ///< RC channel mapped to FC pitch
        uint8_t rc_mix_fc_roll;       ///< RC mixing for FC roll
        uint8_t rc_mix_fc_pitch;      ///< RC mixing for FC pitch
        
        // Follow mode configuration
        uint8_t follow_mode;          ///< Follow mode enable/settings
        uint8_t follow_deadband;      ///< Follow mode deadband
        uint8_t follow_expo_rate;     ///< Follow mode expo rate
        int8_t follow_offset_roll;    ///< Follow mode roll offset
        int8_t follow_offset_pitch;   ///< Follow mode pitch offset
        int8_t follow_offset_yaw;     ///< Follow mode yaw offset
        
        // Frame orientation
        int8_t axis_top;              ///< Top axis orientation
        int8_t axis_right;            ///< Right axis orientation
        
        // Gyro configuration
        uint8_t gyro_lpf;             ///< Gyro low-pass filter setting
        uint8_t gyro_sens;            ///< Gyro sensitivity
        uint8_t i2c_internal_pullups; ///< I2C internal pullup enable
        uint8_t sky_gyro_calib;       ///< SkyGyro calibration flag

        // RC command thresholds
        uint8_t rc_cmd_low;           ///< RC command low threshold
        uint8_t rc_cmd_mid;           ///< RC command mid threshold
        uint8_t rc_cmd_high;          ///< RC command high threshold
        
        // Menu button commands
        uint8_t menu_cmd_1;           ///< Menu button 1 command
        uint8_t menu_cmd_2;           ///< Menu button 2 command
        uint8_t menu_cmd_3;           ///< Menu button 3 command
        uint8_t menu_cmd_4;           ///< Menu button 4 command
        uint8_t menu_cmd_5;           ///< Menu button 5 command
        uint8_t menu_cmd_long;        ///< Menu button long press command
        
        // Output configuration
        uint8_t output_roll;          ///< Roll output configuration
        uint8_t output_pitch;         ///< Pitch output configuration
        uint8_t output_yaw;           ///< Yaw output configuration
        
        // Battery monitoring thresholds
        int16_t bat_threshold_alarm;  ///< Battery voltage alarm threshold (mV)
        int16_t bat_threshold_motors; ///< Battery voltage motor cutoff threshold (mV)
        int16_t bat_comp_ref;         ///< Battery compensation reference voltage (mV)
        
        uint8_t beeper_modes;         ///< Beeper mode configuration flags
        
        // Follow mode advanced settings
        uint8_t follow_roll_mix_start;///< Follow roll mix start point
        uint8_t follow_roll_mix_range;///< Follow roll mix range
        
        // Motor booster power
        uint8_t booster_power_roll;   ///< Roll motor booster power (0-255)
        uint8_t booster_power_pitch;  ///< Pitch motor booster power (0-255)
        uint8_t booster_power_yaw;    ///< Yaw motor booster power (0-255)
        
        // Follow mode speed settings
        uint8_t follow_speed_roll;    ///< Follow mode roll speed
        uint8_t follow_speed_pitch;   ///< Follow mode pitch speed
        uint8_t follow_speed_yaw;     ///< Follow mode yaw speed
        
        uint8_t frame_angle_from_motors; ///< Calculate frame angle from motor position
        
        uint8_t cur_profile_id;       ///< Currently active profile ID

    };
    
    /**
     * @brief Union for protocol message data storage
     * 
     * @details Provides a unified buffer that can hold any of the protocol message structures.
     *          The same memory space is reused for different message types, reducing memory usage.
     *          DEFINE_BYTE_ARRAY_METHODS macro provides byte-level access for checksum calculation.
     */
    union PACKED alexmos_parameters {
        DEFINE_BYTE_ARRAY_METHODS
        alexmos_version version;      ///< Board info response data
        alexmos_angles angles;         ///< Angle response data
        alexmos_params params;         ///< Parameter read/write data
        alexmos_angles_speed angle_speed; ///< Control command data
    } _buffer,           ///< Temporary buffer for incoming/outgoing messages
      _current_parameters; ///< Storage for current gimbal parameters (persistent)

    // Serial port communication
    AP_HAL::UARTDriver *_port;    ///< UART port for gimbal communication
    bool _initialised : 1;        ///< Backend initialization complete flag
    
    // Gimbal board information (from CMD_BOARD_INFO response)
    uint8_t _board_version;              ///< Board hardware version number
    float _current_firmware_version;     ///< Firmware version (e.g., 2.68)
    uint8_t _firmware_beta_version;      ///< Beta version number (0 for release)
    bool _gimbal_3axis : 1;              ///< True if gimbal supports 3-axis (yaw) control
    bool _gimbal_bat_monitoring : 1;     ///< True if gimbal has battery monitoring capability
    
    // Current gimbal state
    Vector3f _current_angle;      ///< Last known gimbal angles (roll, pitch, yaw) in degrees
    
    // Parameter reading state
    bool _param_read_once : 1;    ///< True after first successful CMD_READ_PARAMS
    
    // Serial protocol parsing state machine variables
    uint8_t _checksum;            ///< Running checksum for current message
    uint8_t _step;                ///< Current parsing step (0=wait for start, 1=cmd, 2=size, etc.)
    uint8_t _command_id;          ///< Command ID of message being received
    uint8_t _payload_length;      ///< Expected payload length for current message
    uint8_t _payload_counter;     ///< Number of payload bytes received so far
    
    // Command acknowledgment tracking
    bool _last_command_confirmed : 1; ///< True if last command was acknowledged by gimbal
};
#endif // HAL_MOUNT_ALEXMOS_ENABLED
