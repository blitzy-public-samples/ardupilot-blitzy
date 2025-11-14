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
 * @file AP_GPS_SBP.h
 * @brief GPS driver for Swift Navigation Piksi receivers using SBP v1 protocol
 * 
 * @details This driver implements the legacy Swift Binary Protocol version 1 (SBP v1)
 *          for Swift Navigation Piksi Multi and Dingo receivers. It parses the following
 *          SBP v1 messages:
 *          - MSG_GPS_TIME (0x0100): GPS time, week number, and nanosecond corrections
 *          - MSG_POS_LLH (0x0201): Geodetic position (lat/lon/height) with accuracy estimates
 *          - MSG_VEL_NED (0x0205): NED velocity with accuracy and satellite count
 *          - MSG_DOPS (0x0206): Dilution of precision values (GDOP, PDOP, TDOP, HDOP, VDOP)
 *          - MSG_HEARTBEAT (0xFFFF): System status and health monitoring
 * 
 *          Protocol features:
 *          - CRC16 validation for message integrity
 *          - RTK positioning support (GPS_OK_FIX_3D_RTK_FIXED capability)
 *          - RTCM correction injection for differential/RTK operation
 *          - MAVLink GPS RTK message support for ground station integration
 * 
 *          Coordinate Systems:
 *          - Position: WGS84 geodetic coordinates (latitude/longitude in degrees, 
 *                     height in meters above WGS84 ellipsoid)
 *          - Velocity: NED (North-East-Down) frame in m/s
 * 
 * @note SBP v1 protocol is DEPRECATED. For current Swift Navigation receivers,
 *       use AP_GPS_SBP2 which implements the current SBP v2 protocol with
 *       enhanced features and improved message definitions.
 * 
 * @warning This driver is maintained for backward compatibility with older Piksi
 *          Multi and Dingo receivers. New integrations should use SBP v2.
 * 
 * @see AP_GPS_SBP2 for current Swift Navigation receiver support
 * @see Swift Binary Protocol documentation: http://docs.swift-nav.com/
 * 
 * @author Niels Joubert
 * @copyright Copyright (c) 2014-2025 ArduPilot.org
 * 
 * Source: libraries/AP_GPS/AP_GPS_SBP.h, libraries/AP_GPS/AP_GPS_SBP.cpp
 */

#pragma once

#include "AP_GPS.h"
#include "GPS_Backend.h"

#if AP_GPS_SBP_ENABLED
/**
 * @class AP_GPS_SBP
 * @brief GPS driver for Swift Navigation Piksi receivers using legacy SBP v1 protocol
 * 
 * @details This class implements parsing and state management for Swift Binary Protocol
 *          version 1 messages from Swift Navigation Piksi Multi and Dingo GPS/GNSS receivers.
 *          
 *          Driver Lifecycle:
 *          1. Construction with UART port assignment
 *          2. Message parsing via read() called at main GPS update rate
 *          3. State machine processes SBP v1 packet structure (preamble, type, sender, length, payload, CRC)
 *          4. Position/velocity updates extracted from parsed messages
 *          5. RTCM corrections injected via inject_data() for RTK operation
 * 
 *          Message Processing:
 *          - Maintains parser state machine for packet assembly
 *          - CRC16 validation on all received packets
 *          - Tracks last received message timestamps for timeout detection
 *          - Supports both SPP (Standard Positioning Service) and RTK modes
 *          - Monitors Integer Ambiguity Resolution (IAR) state for RTK quality
 * 
 *          RTK Operation:
 *          - Accepts RTCM corrections via inject_data() method
 *          - Distinguishes between SPP and RTK position solutions
 *          - Reports RTK fix status (Float vs Fixed) to ArduPilot GPS system
 *          - Provides MAVLink GPS_RTK message support for GCS monitoring
 * 
 *          Position Accuracy:
 *          - Horizontal and vertical accuracy reported in millimeters
 *          - Satellite count from position/velocity messages
 *          - DOP values (GDOP, PDOP, TDOP, HDOP, VDOP) for quality assessment
 * 
 * @note This driver implements the LEGACY SBP v1 protocol. Swift Navigation has
 *       released SBP v2 with improved message definitions and additional features.
 *       For current Swift receivers, use AP_GPS_SBP2 driver instead.
 * 
 * @warning SBP v1 protocol deprecated - maintained for backward compatibility only.
 *          New hardware integrations should use AP_GPS_SBP2 for SBP v2 support.
 * 
 * @see AP_GPS_SBP2 for current Swift Navigation receiver protocol support
 */
class AP_GPS_SBP : public AP_GPS_Backend
{
public:
    /**
     * @brief Construct SBP v1 GPS driver instance
     * 
     * @param[in] _gps Reference to main AP_GPS instance for state management
     * @param[in] _params Reference to GPS parameter storage for this instance
     * @param[in] _state Reference to GPS state structure for this instance
     * @param[in] _port UART driver for serial communication with GPS receiver
     */
    AP_GPS_SBP(AP_GPS &_gps, AP_GPS::Params &_params, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    /**
     * @brief Report highest GPS status this driver can achieve
     * 
     * @details SBP v1 receivers support full RTK operation including:
     *          - GPS_OK_FIX_3D: Standard 3D position fix
     *          - GPS_OK_FIX_3D_DGPS: Differential GPS corrections applied
     *          - GPS_OK_FIX_3D_RTK_FLOAT: RTK integer ambiguities partially resolved
     *          - GPS_OK_FIX_3D_RTK_FIXED: RTK integer ambiguities fully resolved (cm-level accuracy)
     * 
     * @return GPS_OK_FIX_3D_RTK_FIXED indicating full RTK capability
     * 
     * @note Swift Piksi receivers with RTCM corrections can achieve centimeter-level
     *       positioning accuracy in RTK fixed mode
     */
    AP_GPS::GPS_Status highest_supported_status(void) override { return AP_GPS::GPS_OK_FIX_3D_RTK_FIXED; }

#if HAL_GCS_ENABLED
    /**
     * @brief Indicate MAVLink GPS_RTK message support for ground station integration
     * 
     * @details Enables transmission of MAVLink GPS_RTK messages containing:
     *          - RTK baseline vectors
     *          - Integer ambiguity resolution status
     *          - Number of hypotheses in RTK solution
     *          - Accuracy metrics for RTK positioning
     * 
     * @return true - driver supports MAVLink GPS_RTK messages
     * 
     * @note Used by GCS to display RTK status and baseline information
     */
    bool supports_mavlink_gps_rtk_message() const override { return true; }
#endif

    /**
     * @brief Parse incoming SBP v1 protocol messages from GPS receiver
     * 
     * @details Implements state machine for SBP v1 packet parsing:
     *          1. WAITING: Search for preamble byte (0x55)
     *          2. GET_TYPE: Read 16-bit message type
     *          3. GET_SENDER: Read 16-bit sender ID
     *          4. GET_LEN: Read 8-bit payload length
     *          5. GET_MSG: Read payload bytes
     *          6. GET_CRC: Read and validate 16-bit CRC
     * 
     *          Processed message types:
     *          - MSG_GPS_TIME: Updates GPS week and time of week
     *          - MSG_POS_LLH: Updates position (latitude, longitude, height)
     *          - MSG_VEL_NED: Updates velocity (North, East, Down)
     *          - MSG_DOPS: Updates dilution of precision values
     *          - MSG_HEARTBEAT: Updates system health status
     * 
     *          CRC Validation:
     *          - Uses CRC16-CCITT algorithm on all message fields
     *          - Invalid CRC increments crc_error_counter
     *          - Failed packets are discarded without state update
     * 
     * @return true if new GPS position/velocity data available after parsing
     *         false if no new data or parsing incomplete
     * 
     * @note Called at main GPS update rate (typically 5-10 Hz)
     * @note Partial packets are buffered across multiple read() calls
     * 
     * @see _sbp_process() for packet assembly logic
     * @see _sbp_process_message() for message-specific parsing
     */
    bool read() override;

    /**
     * @brief Inject RTCM correction data for RTK positioning
     * 
     * @details Forwards RTCM correction messages to GPS receiver for differential
     *          and RTK operation. RTCM data typically received from:
     *          - Base station via telemetry link
     *          - NTRIP caster via network connection
     *          - Local reference receiver
     * 
     *          RTCM messages enable:
     *          - DGPS: Meter-level accuracy improvements
     *          - RTK Float: Decimeter-level accuracy (ambiguities partially resolved)
     *          - RTK Fixed: Centimeter-level accuracy (ambiguities fully resolved)
     * 
     * @param[in] data Pointer to RTCM message bytes
     * @param[in] len Number of bytes in RTCM message
     * 
     * @note Swift receivers support RTCM 3.x messages
     * @note Injection rate should match correction latency requirements (typically 1 Hz)
     * @note Timestamps last_injected_data_ms for monitoring correction data flow
     * 
     * @see highest_supported_status() for RTK capability
     */
    void inject_data(const uint8_t *data, uint16_t len) override;

    /**
     * @brief Detect SBP v1 protocol on serial stream (static detection method)
     * 
     * @details Examines incoming bytes for SBP v1 packet structure to identify
     *          Swift Navigation receivers during GPS auto-detection phase.
     *          Detection looks for:
     *          - Valid preamble byte (0x55)
     *          - Reasonable message type values
     *          - Valid packet length fields
     *          - Known SBP v1 message types
     * 
     * @param[in,out] state Detection state structure maintaining parser context
     * @param[in] data Single byte from serial stream to examine
     * 
     * @return true if SBP v1 protocol detected with confidence
     *         false if protocol not yet identified
     * 
     * @note Called during GPS initialization for automatic protocol detection
     * @note Static method allows detection without driver instance allocation
     */
    static bool _detect(struct SBP_detect_state &state, uint8_t data);

    /**
     * @brief Return driver name identifier
     * 
     * @return "SBP" string identifier for logging and diagnostics
     * 
     * @note Used in GPS driver selection logs and parameter displays
     */
    const char *name() const override { return "SBP"; }

private:

    // ************************************************************************
    // Swift Navigation SBP v1 protocol types and definitions
    // ************************************************************************

    /**
     * @brief SBP v1 packet parser state machine
     * 
     * @details Maintains parsing context for assembling SBP v1 packets from serial stream.
     *          Packet structure: [Preamble][Type][Sender][Length][Payload][CRC]
     * 
     *          State transitions:
     *          WAITING -> GET_TYPE: On valid preamble (0x55)
     *          GET_TYPE -> GET_SENDER: After reading 2-byte message type
     *          GET_SENDER -> GET_LEN: After reading 2-byte sender ID
     *          GET_LEN -> GET_MSG: After reading 1-byte payload length
     *          GET_MSG -> GET_CRC: After reading complete payload
     *          GET_CRC -> WAITING: After validating 2-byte CRC (packet complete)
     */
    struct sbp_parser_state_t {
      enum {
        WAITING = 0,     ///< Searching for preamble byte (0x55)
        GET_TYPE = 1,    ///< Reading 16-bit message type field
        GET_SENDER = 2,  ///< Reading 16-bit sender ID field
        GET_LEN = 3,     ///< Reading 8-bit payload length field
        GET_MSG = 4,     ///< Reading variable-length payload
        GET_CRC = 5      ///< Reading 16-bit CRC and validating packet
      } state:8;
      uint16_t msg_type;      ///< Current message type being parsed
      uint16_t sender_id;     ///< Sender ID of current packet
      uint16_t crc;           ///< Calculated/received CRC value
      uint8_t msg_len;        ///< Payload length for current packet
      uint8_t n_read;         ///< Number of bytes read for current field
      uint8_t msg_buff[256];  ///< Payload buffer (max 255 bytes per SBP v1 spec)
    } parser_state;

    /// SBP v1 packet preamble byte - always 0x55 at start of every message
    static const uint8_t SBP_PREAMBLE = 0x55;

    /**
     * @brief SBP v1 message type identifiers
     * 
     * @details Message types supported by this driver for position, velocity, time,
     *          and system status reporting from Swift Navigation receivers.
     */
    static const uint16_t SBP_STARTUP_MSGTYPE        = 0xFF00;  ///< Receiver startup notification
    static const uint16_t SBP_HEARTBEAT_MSGTYPE      = 0xFFFF;  ///< System health and status (1 Hz)
    static const uint16_t SBP_GPS_TIME_MSGTYPE       = 0x0100;  ///< GPS time, week number, nanosecond correction
    static const uint16_t SBP_DOPS_MSGTYPE           = 0x0206;  ///< Dilution of precision values
    static const uint16_t SBP_POS_ECEF_MSGTYPE       = 0x0200;  ///< Position in ECEF coordinates (not used)
    static const uint16_t SBP_POS_LLH_MSGTYPE        = 0x0201;  ///< Position in geodetic coordinates (primary)
    static const uint16_t SBP_BASELINE_ECEF_MSGTYPE  = 0x0202;  ///< RTK baseline in ECEF frame
    static const uint16_t SBP_BASELINE_NED_MSGTYPE   = 0x0203;  ///< RTK baseline in NED frame
    static const uint16_t SBP_VEL_ECEF_MSGTYPE       = 0x0204;  ///< Velocity in ECEF frame (not used)
    static const uint16_t SBP_VEL_NED_MSGTYPE        = 0x0205;  ///< Velocity in NED frame (primary)
    static const uint16_t SBP_TRACKING_STATE_MSGTYPE = 0x0016;  ///< Satellite tracking status and SNR
    static const uint16_t SBP_IAR_STATE_MSGTYPE      = 0x0019;  ///< Integer Ambiguity Resolution state for RTK

    /**
     * @brief SBP v1 heartbeat message (MSG_HEARTBEAT 0xFFFF)
     * 
     * @details System health and status message sent periodically (typically 1 Hz)
     *          to indicate receiver operational status and protocol version.
     *          
     *          Error flags indicate hardware/firmware issues:
     *          - sys_error: System-level error condition
     *          - io_error: I/O subsystem error
     *          - nap_error: SwiftNAP (signal processing ASIC) error
     * 
     *          Protocol version allows compatibility checking between receiver
     *          firmware and driver implementation.
     * 
     * @note Absence of heartbeat for >2 seconds indicates receiver communication failure
     */
    struct PACKED sbp_heartbeat_t {
        bool sys_error : 1;        ///< System error flag
        bool io_error : 1;         ///< I/O subsystem error flag
        bool nap_error : 1;        ///< NAP (signal processor) error flag
        uint8_t res : 5;           ///< Reserved bits
        uint8_t protocol_minor : 8; ///< SBP protocol minor version
        uint8_t protocol_major : 8; ///< SBP protocol major version
        uint8_t res2 : 7;          ///< Reserved bits
        bool ext_antenna : 1;      ///< External antenna flag (1=external, 0=internal)
    }; // 4 bytes

    /**
     * @brief SBP v1 GPS time message (MSG_GPS_TIME 0x0100)
     * 
     * @details Provides GPS week number and time of week with nanosecond precision
     *          for accurate timestamping of position and velocity solutions.
     *          
     *          GPS Time System:
     *          - Week number: Weeks since GPS epoch (January 6, 1980)
     *          - Time of week: Milliseconds since Sunday 00:00:00
     *          - Nanosecond remainder: Sub-millisecond correction
     * 
     * @note GPS time does NOT include leap seconds (unlike UTC)
     * @note Used to timestamp sensor measurements and synchronize with other systems
     */
    struct PACKED sbp_gps_time_t {
        uint16_t wn;     ///< GPS week number (weeks since GPS epoch)
        uint32_t tow;    ///< GPS Time of Week rounded to nearest millisecond (ms)
        int32_t ns;      ///< Nanosecond remainder of rounded tow for sub-ms precision (ns)
        uint8_t flags;   ///< Status flags (reserved for future use)
    }; // 11 bytes

    /**
     * @brief SBP v1 dilution of precision message (MSG_DOPS 0x0206)
     * 
     * @details DOP values quantify geometric quality of satellite constellation.
     *          Lower DOP = better geometry = higher position accuracy.
     *          
     *          DOP Types:
     *          - GDOP: Geometric (3D position + time combined)
     *          - PDOP: Position (3D position only)
     *          - TDOP: Time (clock offset only)
     *          - HDOP: Horizontal (2D position)
     *          - VDOP: Vertical (altitude)
     * 
     *          Typical DOP interpretation:
     *          - 1-2: Excellent geometry
     *          - 2-5: Good geometry  
     *          - 5-10: Moderate geometry
     *          - >10: Poor geometry
     * 
     * @note DOP values scaled by 0.01 (e.g., value of 250 = DOP of 2.50)
     */
    struct PACKED sbp_dops_t {
        uint32_t tow;    ///< GPS Time of Week (ms)
        uint16_t gdop;   ///< Geometric Dilution of Precision (unit: 0.01)
        uint16_t pdop;   ///< Position Dilution of Precision (unit: 0.01)
        uint16_t tdop;   ///< Time Dilution of Precision (unit: 0.01)
        uint16_t hdop;   ///< Horizontal Dilution of Precision (unit: 0.01)
        uint16_t vdop;   ///< Vertical Dilution of Precision (unit: 0.01)
    }; // 14 bytes

    /**
     * @brief SBP v1 geodetic position message (MSG_POS_LLH 0x0201)
     * 
     * @details Position solution in WGS84 geodetic coordinates (latitude, longitude, height).
     *          Primary position output used by ArduPilot navigation system.
     *          
     *          Coordinate System: WGS84 geodetic
     *          - Latitude: Degrees North (positive) / South (negative), range ±90°
     *          - Longitude: Degrees East (positive) / West (negative), range ±180°
     *          - Height: Meters above WGS84 ellipsoid (NOT mean sea level)
     * 
     *          Fix Types (encoded in flags field):
     *          - SPP: Single Point Positioning (meter-level accuracy)
     *          - DGPS: Differential GPS (sub-meter accuracy)
     *          - RTK Float: RTK with partial ambiguity resolution (decimeter accuracy)
     *          - RTK Fixed: RTK with full ambiguity resolution (centimeter accuracy)
     * 
     *          Accuracy Estimates:
     *          - h_accuracy: 1-sigma horizontal (2D) position uncertainty
     *          - v_accuracy: 1-sigma vertical position uncertainty
     * 
     * @note Driver maintains separate last_pos_llh_spp and last_pos_llh_rtk for
     *       distinguishing between standard and RTK position solutions
     */
    struct PACKED sbp_pos_llh_t {
        uint32_t tow;         ///< GPS Time of Week (ms)
        double lat;           ///< Latitude in WGS84 geodetic coordinates (degrees)
        double lon;           ///< Longitude in WGS84 geodetic coordinates (degrees)
        double height;        ///< Height above WGS84 ellipsoid (meters)
        uint16_t h_accuracy;  ///< Horizontal position accuracy estimate, 1-sigma (mm)
        uint16_t v_accuracy;  ///< Vertical position accuracy estimate, 1-sigma (mm)
        uint8_t n_sats;       ///< Number of satellites used in solution
        uint8_t flags;        ///< Fix type and status flags
    }; // 34 bytes

    /**
     * @brief SBP v1 velocity in NED frame message (MSG_VEL_NED 0x0205)
     * 
     * @details Velocity solution in local North-East-Down coordinate frame.
     *          Primary velocity output used by ArduPilot navigation and control.
     *          
     *          Coordinate System: NED (North-East-Down) local frame
     *          - North: Positive toward true north (mm/s)
     *          - East: Positive toward east (mm/s)
     *          - Down: Positive toward earth center (mm/s, negative = climbing)
     * 
     *          Velocity components scaled by 1000 (mm/s):
     *          - Example: n=1500 represents 1.5 m/s northward velocity
     *          - Range: ±32767 mm/s = ±32.767 m/s maximum reported velocity
     * 
     *          Accuracy Estimates:
     *          - h_accuracy: 1-sigma horizontal (North/East) velocity uncertainty
     *          - v_accuracy: 1-sigma vertical (Down) velocity uncertainty
     * 
     * @note NED frame origin at current vehicle position, north aligned to true north
     * @note Down is positive downward (opposite of altitude rate)
     */
    struct PACKED sbp_vel_ned_t {
        uint32_t tow;          ///< GPS Time of Week (ms)
        int32_t n;             ///< Velocity North coordinate (mm/s)
        int32_t e;             ///< Velocity East coordinate (mm/s)
        int32_t d;             ///< Velocity Down coordinate (mm/s, positive = descending)
        uint16_t h_accuracy;   ///< Horizontal velocity accuracy estimate, 1-sigma (mm/s)
        uint16_t v_accuracy;   ///< Vertical velocity accuracy estimate, 1-sigma (mm/s)
        uint8_t n_sats;        ///< Number of satellites used in solution
        uint8_t flags;         ///< Status flags (reserved)
    }; // 22 bytes

    /**
     * @brief SBP v1 satellite tracking state (MSG_TRACKING_STATE 0x0016)
     * 
     * @details Reports activity and signal quality for individual tracking channels.
     *          Used to monitor satellite visibility and signal-to-noise ratios.
     * 
     * @note One message sent per tracking channel (multiple messages per update)
     */
    struct PACKED sbp_tracking_state_t {
        uint8_t state;         ///< Channel state: 0=disabled, 1=running
        uint8_t prn;           ///< PRN (Pseudo-Random Noise) code identifier of satellite
        float cn0;             ///< Carrier-to-noise density ratio (dB-Hz)
    };

    /**
     * @brief SBP v1 Integer Ambiguity Resolution state (MSG_IAR_STATE 0x0019)
     * 
     * @details Reports RTK integer ambiguity resolution status. Number of hypotheses
     *          indicates confidence in ambiguity resolution:
     *          - num_hypotheses = 1: Single hypothesis, high confidence (RTK Fixed likely)
     *          - num_hypotheses > 1: Multiple hypotheses, lower confidence (RTK Float)
     * 
     * @note Used to determine RTK fix quality and transition between Float/Fixed states
     */
    struct PACKED sbp_iar_state_t {
        uint32_t num_hypotheses;  ///< Number of ambiguity hypotheses under consideration
    };

    /**
     * @brief Process accumulated bytes in parser state machine
     * 
     * @details Main packet assembly loop that consumes available UART bytes and
     *          advances parser through state machine. Handles byte-by-byte parsing
     *          and calls _sbp_process_message() when complete packet received.
     * 
     * @note Called from read() method for each GPS update cycle
     * @see _sbp_process_message() for message-specific handling
     */
    void _sbp_process();

    /**
     * @brief Parse and handle complete SBP v1 message
     * 
     * @details Decodes message payload based on msg_type and updates internal state
     *          structures (last_gps_time, last_pos_llh_*, last_vel_ned, last_dops).
     *          
     *          Supported messages:
     *          - GPS_TIME: Updates GPS week and time of week
     *          - POS_LLH: Updates position (distinguishes SPP vs RTK via flags)
     *          - VEL_NED: Updates velocity components
     *          - DOPS: Updates dilution of precision values
     *          - HEARTBEAT: Updates system health monitoring
     *          - IAR_STATE: Updates RTK ambiguity resolution status
     * 
     * @note Called after successful CRC validation of complete packet
     * @see _attempt_state_update() to commit parsed data to GPS state
     */
    void _sbp_process_message();

    /**
     * @brief Attempt to update GPS driver state from parsed messages
     * 
     * @details Commits parsed message data to main GPS state when complete set
     *          of messages received (position, velocity, time). Validates data
     *          consistency and updates GPS fix status.
     * 
     * @return true if GPS state successfully updated with new position/velocity
     *         false if insufficient data for state update
     * 
     * @note Requires matching timestamps across position/velocity/time messages
     * @note Updates last_full_update_tow and last_full_update_cpu_ms on success
     */
    bool _attempt_state_update();

    // ************************************************************************
    // Internal Received Messages State
    // ************************************************************************

    uint32_t last_heatbeat_received_ms;  ///< Timestamp of last heartbeat (for timeout detection)
    uint32_t last_injected_data_ms;      ///< Timestamp of last RTCM injection (for monitoring)

    struct sbp_gps_time_t last_gps_time;     ///< Last received GPS time message
    struct sbp_dops_t     last_dops;         ///< Last received DOP values
    struct sbp_pos_llh_t  last_pos_llh_spp;  ///< Last Standard Point Positioning solution
    struct sbp_pos_llh_t  last_pos_llh_rtk;  ///< Last RTK position solution
    struct sbp_vel_ned_t  last_vel_ned;      ///< Last NED velocity message
    uint32_t              last_iar_num_hypotheses;  ///< Last IAR hypothesis count

    uint32_t              last_full_update_tow;     ///< GPS time of week of last complete update (ms)
    uint32_t              last_full_update_cpu_ms;  ///< System milliseconds of last complete update

    // ************************************************************************
    // Monitoring and Performance Counting
    // ************************************************************************

    uint32_t crc_error_counter;  ///< Count of packets with CRC validation failures

    // ************************************************************************
    // Logging to AP_Logger
    // ************************************************************************

    /**
     * @brief Log complete GPS update to dataflash
     * 
     * @details Writes GPS position, velocity, accuracy, and status to ArduPilot
     *          binary log for post-flight analysis and debugging.
     * 
     * @note Called after successful state update in _attempt_state_update()
     */
    void logging_log_full_update();
};
#endif
