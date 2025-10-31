/**
 * @file AP_Mount_SToRM32_serial.h
 * @brief SToRM32 gimbal mount backend using custom serial protocol
 * 
 * @details This file implements the serial communication backend for SToRM32
 *          gimbal controllers using the SToRM32 custom serial protocol.
 *          
 *          Protocol Overview:
 *          - Framing byte: 0xFA marks the start of each packet
 *          - CRC-16 checksum for data integrity validation
 *          - Bidirectional communication with command/reply structure
 *          - Angle commands use cmd_set_angles structure format
 *          
 *          The protocol supports:
 *          - Sending target angles (pitch, roll, yaw) with rate control
 *          - Reading gimbal attitude and status information
 *          - ACK replies for command confirmation
 *          - Data replies with full telemetry
 *          
 * @note SToRM32 is a 32-bit gimbal controller supporting 3-axis stabilization
 * @warning Requires correct baudrate configuration and CRC validation for reliable operation
 * 
 * @see AP_Mount_Backend_Serial for base serial mount functionality
 * @see http://www.olliw.eu/storm32bgc-wiki/STorM32_Communications for protocol documentation
 */
#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_STORM32SERIAL_ENABLED

#include "AP_Mount_Backend_Serial.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>

/// @brief Resend interval for angle targets in milliseconds (reliability mechanism)
#define AP_MOUNT_STORM32_SERIAL_RESEND_MS   1000    // resend angle targets to gimbal once per second

/**
 * @class AP_Mount_SToRM32_serial
 * @brief Backend driver for SToRM32 gimbal using custom serial protocol
 * 
 * @details This class implements communication with SToRM32 gimbal controllers
 *          using their proprietary serial protocol. The protocol uses:
 *          
 *          Packet Structure:
 *          - Start byte: 0xFA (framing marker)
 *          - Command/data payload (variable length)
 *          - CRC-16 checksum (2 bytes)
 *          
 *          Command Types:
 *          - CMD_SETANGLE: Send target angles with optional rate limits
 *          - CMD_GETDATA: Request gimbal attitude and status
 *          
 *          Protocol Features:
 *          - cmd_set_angles structure contains pitch/roll/yaw angles and rates
 *          - Angle units: degrees (float)
 *          - Rate units: degrees/second (float)
 *          - CRC polynomial ensures data integrity
 *          - Automatic resend at 1Hz interval for target angle updates
 *          
 *          Reply Handling:
 *          - ACK replies: Command acknowledgment (short response)
 *          - DATA replies: Full telemetry including attitude, IMU data, status
 *          - Reply parser validates CRC before accepting data
 *          
 * @note Protocol operates at gimbal's configured baudrate (typically 115200)
 * @note Uses big-endian byte ordering for multi-byte values
 * @warning CRC validation is mandatory - corrupted packets are discarded
 * @warning Resend interval ensures gimbal receives commands even if packets are lost
 * 
 * Source: libraries/AP_Mount/AP_Mount_SToRM32_serial.h
 */
class AP_Mount_SToRM32_serial : public AP_Mount_Backend_Serial
{

public:
    /**
     * @brief Constructor - inherits from AP_Mount_Backend_Serial
     * 
     * @details Uses base class constructor for serial port initialization
     */
    using AP_Mount_Backend_Serial::AP_Mount_Backend_Serial;

    /**
     * @brief Update mount position and communication - called periodically
     * 
     * @details This method handles the main update loop for the SToRM32 gimbal:
     *          - Reads and parses incoming serial data from gimbal
     *          - Processes attitude telemetry and status information
     *          - Sends target angle commands to gimbal at regular intervals
     *          - Implements resend logic (1Hz) to ensure reliable command delivery
     *          - Updates internal attitude state from gimbal feedback
     *          
     *          Update sequence:
     *          1. Read incoming bytes from serial port
     *          2. Parse protocol packets (0xFA header, payload, CRC)
     *          3. Validate CRC and extract attitude data
     *          4. Check if resend interval elapsed
     *          5. Send new target angles if needed
     *          
     * @note Called at main loop rate (typically 10-50Hz)
     * @note Resend interval (AP_MOUNT_STORM32_SERIAL_RESEND_MS) ensures
     *       gimbal receives commands even if packets are lost
     * 
     * @see send_target_angles() for angle command transmission
     * @see read_incoming() for packet parsing
     */
    void update() override;

    /**
     * @brief Check if gimbal can control pan/yaw axis
     * 
     * @details Returns true if the gimbal supports yaw control with valid range
     *          configuration. Pan control capability is required for multicopters
     *          to maintain camera pointing while vehicle yaws.
     *          
     *          SToRM32 gimbals support 3-axis control including yaw/pan when
     *          configured appropriately and yaw range parameters are set.
     * 
     * @return true if yaw control is available and configured with valid range
     * @return false if yaw control is not available or range not configured
     * 
     * @note Critical for multicopters that need independent camera yaw control
     * @see yaw_range_valid() checks if yaw limits are properly configured
     */
    bool has_pan_control() const override { return yaw_range_valid(); };

protected:

    /**
     * @brief Get gimbal attitude as quaternion from latest telemetry
     * 
     * @details Retrieves the current gimbal attitude tracked from SToRM32
     *          telemetry data replies. The attitude is converted from the
     *          Euler angles (pitch, roll, yaw) received in protocol DATA replies
     *          to a quaternion representation.
     *          
     *          Attitude Source:
     *          - Read from _current_angle member (Vector3l in centidegrees)
     *          - Updated by parse_reply() when DATA packets received
     *          - Converted from Euler angles to quaternion
     *          - Represents gimbal orientation in vehicle body frame
     *          
     *          Coordinate Frame:
     *          - Roll: rotation about forward axis
     *          - Pitch: rotation about right axis  
     *          - Yaw: rotation about down axis
     *          
     * @param[out] att_quat Quaternion to populate with gimbal attitude
     * 
     * @return true if valid attitude data available and quaternion populated
     * @return false if no attitude data received yet from gimbal
     * 
     * @note Attitude is only valid after receiving at least one DATA reply
     * @note Quaternion provides singularity-free attitude representation
     * @warning Attitude validity depends on successful CRC-validated DATA packets
     * 
     * @see parse_reply() updates _current_angle from telemetry
     */
    bool get_attitude_quaternion(Quaternion& att_quat) override;

private:

    /**
     * @brief Send target angle command to gimbal
     * 
     * @details Constructs and transmits a CMD_SETANGLE packet to the SToRM32
     *          gimbal containing target angles for pitch, roll, and yaw axes.
     *          
     *          Packet Construction:
     *          1. Build cmd_set_angles structure with angles in degrees
     *          2. Add protocol header bytes (0xFA framing)
     *          3. Calculate and append CRC-16 checksum
     *          4. Transmit complete packet over serial port
     *          
     *          Protocol Details:
     *          - Structure: cmd_set_angles (byte markers, angles, flags, CRC)
     *          - Angle units: degrees (float)
     *          - Rate units: degrees/second (float) if rate control enabled
     *          - Flags field: controls rate limiting and coordinate frame
     *          - Type field: specifies angle vs rate command mode
     *          
     * @param[in] angle_target_rad Target mount angles in radians (roll, pitch, yaw)
     *                              Converted internally to degrees for protocol
     * 
     * @note Called by update() at resend interval to maintain gimbal pointing
     * @note Input angles in radians are converted to degrees for protocol
     * @warning CRC must be correctly calculated or gimbal will reject packet
     * 
     * @see cmd_set_angles structure for packet format details
     * @see update() calls this method at regular intervals
     */
    void send_target_angles(const MountTarget& angle_target_rad);

    /**
     * @brief Send request to gimbal for attitude and status data
     * 
     * @details Transmits a CMD_GETDATA packet to request full telemetry from
     *          the gimbal including attitude, IMU readings, and status flags.
     *          Gimbal responds with SToRM32_reply_data_struct containing
     *          comprehensive state information.
     * 
     * @note Response is processed asynchronously by read_incoming()
     * @see SToRM32_reply_data_struct for data format
     * @see parse_reply() processes the response
     */
    void get_angles();

    /**
     * @brief Read and parse incoming bytes from gimbal serial port
     * 
     * @details Implements packet parser for SToRM32 serial protocol with
     *          state machine handling:
     *          
     *          Parsing States:
     *          1. Search for 0xFA start byte (framing synchronization)
     *          2. Read packet length/type indicators
     *          3. Accumulate payload bytes
     *          4. Read CRC-16 checksum (2 bytes)
     *          5. Validate CRC and process complete packet
     *          
     *          Packet Types Handled:
     *          - ACK replies: Short acknowledgment packets
     *          - DATA replies: Full telemetry with attitude and status
     *          
     *          Error Handling:
     *          - Discard packets with invalid CRC
     *          - Reset parser state on framing errors
     *          - Timeout stale partial packets
     *          
     * @note Called by update() each iteration to process available data
     * @note Parser maintains state across calls for byte-by-byte processing
     * @warning Packets with CRC errors are silently discarded
     * 
     * @see parse_reply() processes validated packets
     * @see _buffer union holds received packet data
     */
    void read_incoming();

    /**
     * @brief Parse and process validated packet from gimbal
     * 
     * @details Extracts information from CRC-validated packets received from
     *          gimbal. Handles both ACK and DATA reply types:
     *          
     *          ACK Processing:
     *          - Confirms command receipt by gimbal
     *          - Updates protocol state for flow control
     *          
     *          DATA Processing:
     *          - Extracts gimbal attitude (pitch, roll, yaw)
     *          - Updates _current_angle with latest attitude
     *          - Processes status flags and error conditions
     *          - Stores IMU data for diagnostics
     *          
     * @note Only called after CRC validation passes in read_incoming()
     * @note Updates _current_angle which is used by get_attitude_quaternion()
     * @warning Assumes _buffer contains valid packet data
     * 
     * @see read_incoming() validates CRC before calling this method
     * @see SToRM32_reply_data_struct for DATA packet structure
     * @see SToRM32_reply_ack_struct for ACK packet structure
     */
    void parse_reply();

    /**
     * @brief Protocol reply packet type identifier
     * 
     * @details Identifies the type of reply packet expected or received from
     *          the SToRM32 gimbal. Used by packet parser to determine how to
     *          process incoming data.
     */
    enum ReplyType {
        ReplyType_UNKNOWN = 0,  ///< Unknown or uninitialized reply type
        ReplyType_DATA,         ///< Full telemetry data reply with attitude and status
        ReplyType_ACK           ///< Short acknowledgment reply confirming command receipt
    };

    /**
     * @brief Get expected size in bytes for a reply packet type
     * 
     * @param[in] reply_type Type of reply packet (DATA or ACK)
     * 
     * @return Size in bytes of the complete packet including header and CRC
     * 
     * @note Used by packet parser to know when complete packet received
     * @see SToRM32_reply_data_struct for DATA packet size
     * @see SToRM32_reply_ack_struct for ACK packet size
     */
    uint8_t get_reply_size(ReplyType reply_type);

    /**
     * @brief Check if serial port is ready to send command
     * 
     * @details Determines if it's safe to transmit a command packet based on
     *          protocol state and pending replies. Prevents flooding gimbal
     *          with commands while waiting for responses.
     * 
     * @param[in] with_control True if sending control command, false for query
     * 
     * @return true if safe to send command now
     * @return false if should wait for pending reply
     * 
     * @note Implements basic flow control to avoid overrunning gimbal
     */
    bool can_send(bool with_control);

    /**
     * @brief Full telemetry data reply packet structure from SToRM32 gimbal
     * 
     * @details Complete telemetry packet received in response to CMD_GETDATA request.
     *          Contains comprehensive gimbal state including attitude, IMU readings,
     *          control outputs, and status flags.
     *          
     *          Field Groups:
     *          - State/Status: Overall gimbal operating state and error flags
     *          - System: I2C errors, voltage, timing, cycle time
     *          - IMU1 Sensors: Gyro (gx,gy,gz) and accelerometer (ax,ay,az) raw readings
     *          - AHRS: Attitude solution in arbitrary units
     *          - IMU1 Attitude: Pitch, roll, yaw from primary IMU (0.01 degree units)
     *          - Control PID: Controller outputs for each axis
     *          - Input: RC or commanded input values
     *          - IMU2: Secondary IMU attitude (if available)
     *          - Magnetometer: Yaw and pitch from magnetometer
     *          - Confidence: AHRS/IMU fusion confidence metric
     *          - Functions: Input function values
     *          
     *          Data Units:
     *          - Angles: 0.01 degrees (divide by 100 for degrees)
     *          - Gyro rates: Hardware-dependent ADC units
     *          - Accelerations: Hardware-dependent ADC units
     *          - Voltage: ADC units (requires calibration)
     *          
     * @note Packet uses PACKED attribute for direct serial deserialization
     * @note All multi-byte values are little-endian
     * @note CRC field at end validates entire packet
     * @note Magic byte (0x00) marks end of payload before CRC
     * 
     * @see parse_reply() extracts attitude from imu1_pitch/roll/yaw fields
     */
    struct PACKED SToRM32_reply_data_struct {
        uint16_t state;
        uint16_t status;
        uint16_t status2;

        uint16_t i2c_errors;
        uint16_t lipo_voltage;
        uint16_t systicks;
        uint16_t cycle_time;

        int16_t imu1_gx;
        int16_t imu1_gy;
        int16_t imu1_gz;

        int16_t imu1_ax;
        int16_t imu1_ay;
        int16_t imu1_az;

        int16_t ahrs_x;
        int16_t ahrs_y;
        int16_t ahrs_z;

        int16_t imu1_pitch;
        int16_t imu1_roll;
        int16_t imu1_yaw;

        int16_t cpid_pitch;
        int16_t cpid_roll;
        int16_t cpid_yaw;

        uint16_t input_pitch;
        uint16_t input_roll;
        uint16_t input_yaw;

        int16_t imu2_pitch;
        int16_t imu2_roll;
        int16_t imu2_yaw;

        int16_t mag2_yaw;
        int16_t mag2_pitch;

        int16_t ahrs_imu_confidence;

        uint16_t function_input_values;

        uint16_t crc;
        uint8_t magic;
    };

    /**
     * @brief Short acknowledgment reply packet structure
     * 
     * @details Brief packet sent by gimbal to acknowledge receipt of commands.
     *          Much smaller than DATA replies, used for quick confirmation.
     *          
     *          Structure:
     *          - byte1, byte2, byte3: Protocol markers/command ID
     *          - data: ACK-specific data or status code
     *          - crc: CRC-16 checksum for validation
     *          
     * @note Minimal packet size for efficient acknowledgment
     * @note CRC validation required even for short ACK packets
     * 
     * @see ReplyType_ACK
     */
    struct PACKED SToRM32_reply_ack_struct {
        uint8_t byte1;   ///< Protocol marker byte 1
        uint8_t byte2;   ///< Protocol marker byte 2
        uint8_t byte3;   ///< Protocol marker byte 3
        uint8_t data;    ///< ACK data or status code
        uint16_t crc;    ///< CRC-16 checksum for packet validation
    };

    /**
     * @brief Command packet structure for setting gimbal target angles
     * 
     * @details Protocol packet format for CMD_SETANGLE command that controls
     *          gimbal pointing. Transmitted by send_target_angles() to command
     *          the gimbal to a desired orientation.
     *          
     *          Packet Structure:
     *          - byte1, byte2, byte3: Protocol command markers (0xFA header sequence)
     *          - pitch: Target pitch angle in degrees (float, +/-180 range typical)
     *          - roll: Target roll angle in degrees (float, +/-180 range typical)
     *          - yaw: Target yaw angle in degrees (float, +/-180 range typical)
     *          - flags: Control flags (rate limiting, frame reference, etc.)
     *          - type: Command type (angle mode vs rate mode)
     *          - crc: CRC-16 checksum calculated over entire packet
     *          
     *          Angle Encoding:
     *          - Native units: degrees (IEEE 754 float)
     *          - Positive pitch: nose up
     *          - Positive roll: right side down
     *          - Positive yaw: nose right
     *          
     *          Rate Control:
     *          - When rate limiting enabled via flags, angles specify rates
     *          - Rate units: degrees/second (float)
     *          - Gimbal ramps to target angle at specified rate
     *          
     *          Flags Field Bits:
     *          - Rate control enable
     *          - Coordinate frame selection
     *          - Priority/override modes
     *          
     * @note Packet uses PACKED attribute for direct serial serialization
     * @note All float values are IEEE 754 single precision
     * @note CRC must be calculated over all preceding bytes
     * @warning Incorrect CRC will cause gimbal to reject command
     * @warning Angle limits depend on gimbal mechanical configuration
     * 
     * @see send_target_angles() constructs and transmits this packet
     */
    struct PACKED cmd_set_angles_struct {
        uint8_t byte1;   ///< Protocol header byte 1 (0xFA framing)
        uint8_t byte2;   ///< Protocol header byte 2
        uint8_t byte3;   ///< Protocol header byte 3
        float pitch;     ///< Target pitch angle in degrees (+ nose up)
        float roll;      ///< Target roll angle in degrees (+ right down)
        float yaw;       ///< Target yaw angle in degrees (+ nose right)
        uint8_t flags;   ///< Control flags (rate limiting, frame reference)
        uint8_t type;    ///< Command type (angle vs rate mode)
        uint16_t crc;    ///< CRC-16 checksum for packet validation
    };


    // Internal state variables for protocol handling
    
    /// System time (milliseconds) when last angle command sent to gimbal - used for resend timing
    uint32_t _last_send;
    
    /// Expected length in bytes of current reply packet being parsed
    uint8_t _reply_length;
    
    /// Current byte position in reply packet during parsing (0 to _reply_length-1)
    uint8_t _reply_counter;
    
    /// Type of reply packet currently being received (UNKNOWN, DATA, or ACK)
    ReplyType _reply_type = ReplyType_UNKNOWN;

    /**
     * @brief Receive buffer for incoming gimbal reply packets
     * 
     * @details Union allows interpreting raw byte stream as either DATA or ACK
     *          structure without copying. Parser fills buffer byte-by-byte,
     *          then parse_reply() interprets based on _reply_type.
     *          
     * @note DEFINE_BYTE_ARRAY_METHODS provides byte-level access for parsing
     * @see SToRM32_reply_data_struct for full telemetry format
     * @see SToRM32_reply_ack_struct for acknowledgment format
     */
    union PACKED SToRM32_reply {
        DEFINE_BYTE_ARRAY_METHODS
        SToRM32_reply_data_struct data;  ///< Interpret buffer as DATA reply
        SToRM32_reply_ack_struct ack;    ///< Interpret buffer as ACK reply
    } _buffer;

    /// Most recent gimbal attitude from telemetry (pitch, roll, yaw in centidegrees)
    /// Updated by parse_reply() when DATA packets received, used by get_attitude_quaternion()
    Vector3l _current_angle;
};
#endif // HAL_MOUNT_STORM32SERIAL_ENABLED
