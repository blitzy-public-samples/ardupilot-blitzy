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
 * @file AP_GPS_SBP2.h
 * @brief GPS driver for Swift Navigation receivers using SBP protocol version 2
 * 
 * @details Swift Binary Protocol (SBP) version 2 driver for Swift Navigation GNSS receivers.
 *          This is the current protocol version used by modern Swift Navigation products including:
 *          - Piksi Multi
 *          - Duro
 *          - Duro Inertial
 *          
 *          SBP v2 provides enhanced capabilities over v1:
 *          - Multi-GNSS support (GPS, GLONASS, Galileo, BeiDou)
 *          - Full covariance matrices for position and velocity accuracy reporting
 *          - Enhanced fix mode flags (SPP, DGNSS, Float RTK, Fixed RTK, Dead Reckoning)
 *          - Improved time representation with nanosecond resolution
 *          - Inertial navigation integration status for INS-capable units
 *          
 *          Primary message types parsed by this driver:
 *          - SBP_GPS_TIME (0x0102): GPS time with nanosecond precision
 *          - SBP_DOPS (0x0208): Dilution of precision and fix mode
 *          - SBP_POS_LLH (0x020A): Geodetic position in WGS84
 *          - SBP_VEL_NED (0x020E): Velocity in local NED frame
 *          - SBP_HEARTBEAT (0xFFFF): Device status and protocol version
 *          
 *          RTK Support:
 *          This driver supports injection of RTCM3 corrections for Real-Time Kinematic
 *          positioning. The receiver processes corrections to achieve centimeter-level
 *          accuracy in Float RTK (decimeter) or Fixed RTK (centimeter) modes.
 *          
 *          Coordinate Systems:
 *          - Position: WGS84 ellipsoid, latitude/longitude in degrees, height in meters
 *          - Velocity: NED (North-East-Down) frame in m/s
 *          - Accuracy: Horizontal/vertical standard deviations in millimeters
 *          
 *          Protocol Validation:
 *          All messages validated with CRC16-CCITT checksum before processing.
 * 
 * @note Use this driver for all current Swift Navigation products. For legacy Piksi v2.3
 *       receivers, use AP_GPS_SBP driver instead.
 * 
 * @author Niels Joubert
 * @see http://docs.swift-nav.com/ for complete SBP protocol specification
 * 
 * Source: libraries/AP_GPS/AP_GPS_SBP2.h
 */

//
//  Swift Navigation SBP GPS driver for ArduPilot.
//	Code by Niels Joubert
//
//  Swift Binary Protocol format: http://docs.swift-nav.com/
//
#pragma once

#include "AP_GPS.h"
#include "GPS_Backend.h"

#if AP_GPS_SBP2_ENABLED

/**
 * @class AP_GPS_SBP2
 * @brief GPS driver backend for Swift Navigation receivers using SBP protocol version 2
 * 
 * @details This class implements the GPS_Backend interface for Swift Navigation GNSS receivers
 *          communicating via the Swift Binary Protocol version 2. It handles:
 *          
 *          Message Parsing:
 *          - State machine-based packet parsing with preamble detection (0x55)
 *          - CRC16-CCITT validation of all received messages
 *          - Multi-message state aggregation for complete position solution
 *          
 *          Position Solution:
 *          - WGS84 geodetic position from SBP_POS_LLH messages
 *          - NED velocity from SBP_VEL_NED messages
 *          - DOP values and fix mode from SBP_DOPS messages
 *          - GPS time synchronization from SBP_GPS_TIME messages
 *          
 *          RTK Operation:
 *          - Supports Float RTK (decimeter-level accuracy)
 *          - Supports Fixed RTK (centimeter-level accuracy)
 *          - RTCM3 correction data injection via inject_data()
 *          - Fix mode progression: SPP → DGNSS → Float RTK → Fixed RTK
 *          
 *          Multi-GNSS Support:
 *          - GPS, GLONASS, Galileo, BeiDou constellation support
 *          - Automatic constellation selection by receiver
 *          - Enhanced availability and accuracy from multi-constellation tracking
 *          
 *          Accuracy Reporting:
 *          - Horizontal and vertical position accuracy from covariance data
 *          - Horizontal and vertical velocity accuracy estimates
 *          - HDOP/VDOP values for solution quality assessment
 *          - Satellite count for each solution
 *          
 *          Health Monitoring:
 *          - Heartbeat message processing for device status
 *          - CRC error counting for link quality monitoring
 *          - Antenna status detection (external antenna presence/short circuit)
 *          - System error flags (processor, I/O, NAP errors)
 *          
 *          External Event Timestamping:
 *          - Microsecond-accurate timestamping of external events (e.g., camera shutter)
 *          - Pin-level change detection with GPS time correlation
 *          
 *          Thread Safety:
 *          - Called from GPS thread at configured update rate (typically 5-10Hz)
 *          - Parser state maintained across read() calls
 *          - No locking required (single-threaded access)
 * 
 * @note This driver is designed for SBP protocol v2 which differs from v1 in message
 *       formats and available data fields. Do not use with legacy Piksi v2.3 hardware.
 * 
 * @warning Ensure UART is configured for 115200 baud (default) or higher for optimal
 *          message throughput. Lower baud rates may cause message loss at high update rates.
 * 
 * Source: libraries/AP_GPS/AP_GPS_SBP2.h, libraries/AP_GPS/AP_GPS_SBP2.cpp
 */
class AP_GPS_SBP2 : public AP_GPS_Backend
{
public:
    /**
     * @brief Construct SBP2 GPS driver backend
     * 
     * @param[in] _gps Reference to main AP_GPS object for configuration access
     * @param[in] _params Reference to GPS parameters for this instance
     * @param[in] _state Reference to GPS state structure to populate with solutions
     * @param[in] _port Pointer to UART driver for communication with GPS receiver
     * 
     * @note Constructor initializes parser state machine to WAITING state and
     *       clears all message buffers. Actual GPS initialization occurs in read().
     */
    AP_GPS_SBP2(AP_GPS &_gps, AP_GPS::Params &_params, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    /**
     * @brief Report highest GPS fix status this driver can achieve
     * 
     * @return AP_GPS::GPS_OK_FIX_3D_RTK_FIXED indicating support for centimeter-level RTK
     * 
     * @details Swift Navigation receivers with RTK corrections can achieve Fixed RTK solution
     *          providing centimeter-level 3D position accuracy. This represents the highest
     *          quality fix available in the ArduPilot GPS subsystem.
     * 
     * @note Actual fix status depends on:
     *       - Satellite visibility and geometry
     *       - Quality and age of RTCM3 corrections
     *       - Baseline length to reference station
     *       - Environmental conditions (multipath, interference)
     */
    AP_GPS::GPS_Status highest_supported_status(void) override { return AP_GPS::GPS_OK_FIX_3D_RTK_FIXED; }

    /**
     * @brief Read and parse available data from GPS receiver
     * 
     * @return true if a new position solution is available, false otherwise
     * 
     * @details Called repeatedly by GPS thread to process incoming SBP messages.
     *          Implements state machine parser that:
     *          
     *          Parsing States:
     *          1. WAITING: Searching for SBP preamble byte (0x55)
     *          2. GET_TYPE: Reading 2-byte message type identifier
     *          3. GET_SENDER: Reading 2-byte sender ID
     *          4. GET_LEN: Reading 1-byte payload length
     *          5. GET_MSG: Reading message payload (up to 255 bytes)
     *          6. GET_CRC: Reading 2-byte CRC16-CCITT checksum
     *          
     *          Message Processing:
     *          - Validates CRC before processing any message
     *          - Aggregates data from multiple message types (time, position, velocity, DOP)
     *          - Updates GPS state only when complete solution available
     *          - Logs CRC errors for link quality monitoring
     *          
     *          Update Timing:
     *          - Typically called at 50Hz by GPS driver thread
     *          - Receiver outputs messages at configured rate (5-10Hz default)
     *          - Position solution updated when all required messages received
     *          
     * @note Returns true only when _attempt_state_update() succeeds with complete solution.
     *       May process multiple messages per call if data available.
     * 
     * @warning Parser maintains state across calls. Do not call from multiple threads.
     * 
     * Source: libraries/AP_GPS/AP_GPS_SBP2.cpp:_sbp_process()
     */
    bool read() override;

    /**
     * @brief Inject RTCM3 correction data to GPS receiver for RTK positioning
     * 
     * @param[in] data Pointer to RTCM3 message buffer
     * @param[in] len Length of RTCM3 message in bytes
     * 
     * @details Forwards RTCM3 correction data from base station to GPS receiver over UART.
     *          Swift Navigation receivers accept RTCM3 messages including:
     *          - 1005: Station coordinates (required for RTK)
     *          - 1074, 1084, 1094, 1124: GPS, GLONASS, Galileo, BeiDou observations
     *          - 1230: GLONASS code-phase biases
     *          
     *          RTK Fix Progression:
     *          Without corrections: SPP fix (meter-level accuracy)
     *          With corrections: DGNSS → Float RTK → Fixed RTK
     *          
     *          Correction Requirements:
     *          - Data age < 30 seconds for RTK (< 5 seconds for Fixed RTK)
     *          - Baseline < 40km for reliable Fixed RTK
     *          - Consistent message stream (1Hz minimum)
     *          
     *          Data is passed through transparently to receiver's correction input port.
     * 
     * @note Injection rate tracked in last_injected_data_ms for monitoring.
     *       Receiver automatically processes corrections and updates fix mode.
     * 
     * @warning Malformed RTCM3 data will be rejected by receiver but may cause
     *          temporary degradation of fix quality during error recovery.
     * 
     * Source: libraries/AP_GPS/AP_GPS_SBP2.cpp:inject_data()
     */
    void inject_data(const uint8_t *data, uint16_t len) override;

    /**
     * @brief Detect SBP2 protocol in data stream for GPS auto-detection
     * 
     * @param[in,out] state Detection state machine structure
     * @param[in] data Single byte from GPS receiver UART
     * 
     * @return true if valid SBP2 message detected, false otherwise
     * 
     * @details Static method used by GPS driver during auto-detection phase to identify
     *          GPS receiver protocol. Looks for:
     *          - SBP preamble byte (0x55)
     *          - Valid message type from known SBP2 message set
     *          - Correct message length for type
     *          - Valid CRC16-CCITT checksum
     *          
     *          Detection Strategy:
     *          - Requires complete valid message to confirm protocol
     *          - Prevents false positives from other binary protocols
     *          - Called repeatedly with incoming bytes until detection succeeds
     *          
     * @note This is a static method that does not access instance data. State maintained
     *       in passed detection structure to allow multiple concurrent detection attempts.
     * 
     * Source: libraries/AP_GPS/AP_GPS_SBP2.cpp:_detect()
     */
    static bool _detect(struct SBP2_detect_state &state, uint8_t data);

    /**
     * @brief Get driver identification string
     * 
     * @return "SBP2" - Protocol version identifier string
     * 
     * @details Used for logging, debugging, and user display to identify GPS driver type.
     *          "SBP2" distinguishes this driver from legacy "SBP" (v1) driver.
     * 
     * @note String is compile-time constant with no runtime allocation.
     */
    const char *name() const override { return "SBP2"; }

private:

    // ************************************************************************
    // Swift Navigation SBP protocol types and definitions
    // ************************************************************************

    /**
     * @struct sbp_parser_state_t
     * @brief State machine structure for parsing SBP binary protocol
     * 
     * @details Maintains parsing state across multiple read() calls as bytes arrive
     *          from UART. The parser progresses through states sequentially:
     *          WAITING → GET_TYPE → GET_SENDER → GET_LEN → GET_MSG → GET_CRC
     *          
     *          After successful CRC validation, returns to WAITING for next message.
     *          On any error (invalid length, CRC failure), immediately returns to WAITING.
     *          
     * @note Maximum message payload is 255 bytes per SBP specification.
     *       CRC calculated over type, sender, length, and payload fields.
     */
    struct sbp_parser_state_t {
      enum {
        WAITING = 0,   ///< Searching for SBP preamble byte (0x55)
        GET_TYPE = 1,  ///< Reading 2-byte message type identifier
        GET_SENDER = 2,///< Reading 2-byte sender ID (device identifier)
        GET_LEN = 3,   ///< Reading 1-byte payload length (0-255)
        GET_MSG = 4,   ///< Reading variable-length message payload
        GET_CRC = 5    ///< Reading 2-byte CRC16-CCITT checksum
      } state:8;
      uint16_t msg_type;    ///< Current message type being parsed
      uint16_t sender_id;   ///< Device ID of message sender
      uint16_t crc;         ///< Calculated/received CRC value
      uint8_t msg_len;      ///< Length of current message payload
      uint8_t n_read;       ///< Number of bytes read in current state
      uint8_t msg_buff[256];///< Message payload buffer (max 255 bytes + margin)
    } parser_state;

    /// SBP protocol preamble byte - marks start of every message
    static const uint8_t SBP_PREAMBLE = 0x55;

    /**
     * @name SBP Message Type Identifiers
     * @brief Message type constants for Swift Binary Protocol v2
     * 
     * @details Primary message types processed by this driver:
     * - GPS_TIME: GPS week and time of week with nanosecond resolution
     * - DOPS: Dilution of precision values and fix mode status
     * - POS_LLH: Geodetic position in latitude/longitude/height
     * - VEL_NED: Velocity in North-East-Down local frame
     * - HEARTBEAT: Device status and health monitoring
     * - EXT_EVENT: External event timestamping (camera trigger, etc.)
     * 
     * @note Message IDs are defined by Swift Navigation SBP specification.
     *       Values differ from SBP v1 protocol.
     * @{
     */
    static const uint16_t SBP_STARTUP_MSGTYPE        = 0xFF00; ///< Startup message (device initialization)
    static const uint16_t SBP_HEARTBEAT_MSGTYPE      = 0xFFFF; ///< Heartbeat and device status
    static const uint16_t SBP_GPS_TIME_MSGTYPE       = 0x0102; ///< GPS time with nanosecond precision
    static const uint16_t SBP_DOPS_MSGTYPE           = 0x0208; ///< Dilution of precision and fix mode
    static const uint16_t SBP_POS_ECEF_MSGTYPE       = 0x0209; ///< Position in ECEF coordinates
    static const uint16_t SBP_POS_LLH_MSGTYPE        = 0x020A; ///< Position in geodetic coordinates (primary)
    static const uint16_t SBP_BASELINE_ECEF_MSGTYPE  = 0x020B; ///< RTK baseline in ECEF
    static const uint16_t SBP_BASELINE_NED_MSGTYPE   = 0x020C; ///< RTK baseline in NED frame
    static const uint16_t SBP_VEL_ECEF_MSGTYPE       = 0x020D; ///< Velocity in ECEF coordinates
    static const uint16_t SBP_VEL_NED_MSGTYPE        = 0x020E; ///< Velocity in NED coordinates (primary)
    static const uint16_t SBP_TRACKING_STATE_MSGTYPE = 0x0013; ///< Satellite tracking status
    static const uint16_t SBP_IAR_STATE_MSGTYPE      = 0x0019; ///< Integer ambiguity resolution status
    static const uint16_t SBP_EXT_EVENT_MSGTYPE      = 0x0101; ///< External event timestamp
    /** @} */

    /**
     * @struct sbp_heartbeat_t
     * @brief Heartbeat message (0xFFFF) - Device status and health monitoring
     * 
     * @details Sent periodically (typically 1Hz) by receiver to indicate operational status.
     *          Used to detect device presence, monitor health, and verify protocol version.
     *          
     *          Error Flags:
     *          - sys_error: System error detected (requires power cycle)
     *          - io_error: I/O subsystem error
     *          - nap_error: SwiftNAP (correlator) error
     *          
     *          Antenna Status:
     *          - ext_antenna: External antenna detected
     *          - ext_antenna_short: Short circuit detected on antenna line
     *          
     * @note Protocol version should match expected major version (2 for SBP v2).
     *       Mismatch indicates incompatible firmware version.
     */
    struct PACKED sbp_heartbeat_t {
        bool sys_error : 1;          ///< System error flag
        bool io_error : 1;           ///< I/O error flag
        bool nap_error : 1;          ///< NAP (correlator) error flag
        uint8_t res : 5;             ///< Reserved bits
        uint8_t protocol_minor : 8;  ///< Protocol minor version number
        uint8_t protocol_major : 8;  ///< Protocol major version (should be 2)
        uint8_t res2 : 6;            ///< Reserved bits
        bool ext_antenna_short : 1;  ///< External antenna short circuit detected
        bool ext_antenna : 1;        ///< External antenna present
    }; // 4 bytes

    /**
     * @struct sbp_gps_time_t
     * @brief GPS Time message (0x0102) - High-precision GPS time
     * 
     * @details Provides GPS time with nanosecond resolution for precise time synchronization.
     *          GPS time is continuous (no leap seconds) and referenced to GPS epoch
     *          (January 6, 1980 00:00:00 UTC).
     *          
     *          Time Representation:
     *          - Week number: GPS weeks since epoch (0-65535, rolls over ~1260 years)
     *          - Time of week: Milliseconds since week start (0-604800000)
     *          - Nanosecond residual: Sub-millisecond precision (-500000 to +500000 ns)
     *          
     *          Time Source:
     *          - 0: Invalid/unknown
     *          - 1: GNSS solution (synchronized to satellites)
     *          - 2: Propagated (using local oscillator, not satellite-locked)
     *          
     * @note Essential for correlating GPS measurements with vehicle events and sensors.
     *       ArduPilot uses this for timestamp synchronization across subsystems.
     */
    struct PACKED sbp_gps_time_t {
        uint16_t wn;     ///< GPS week number (weeks since Jan 6, 1980)
        uint32_t tow;    ///< GPS Time of Week rounded to nearest millisecond (ms)
        int32_t ns;      ///< Nanosecond remainder of rounded tow (range: -500000 to +500000 ns)
        struct PACKED flags {
            uint8_t time_src:3;  ///< Time source: 0=invalid, 1=GNSS solution, 2=propagated
            uint8_t res:5;       ///< Reserved bits
        } flags;
    }; // 11 bytes

    /**
     * @struct sbp_dops_t
     * @brief DOPS message (0x0208) - Dilution of Precision and fix mode
     * 
     * @details Provides satellite geometry quality metrics and current fix mode.
     *          Lower DOP values indicate better geometry and more accurate solutions.
     *          
     *          DOP Metrics (scaled by 0.01):
     *          - GDOP: Geometric DOP (overall 3D geometry quality)
     *          - PDOP: Position DOP (3D position quality)
     *          - HDOP: Horizontal DOP (2D position quality) - primary accuracy indicator
     *          - VDOP: Vertical DOP (altitude quality)
     *          - TDOP: Time DOP (clock offset quality)
     *          
     *          Typical DOP Interpretation:
     *          - 1-2: Excellent geometry
     *          - 2-5: Good geometry (acceptable for most operations)
     *          - 5-10: Moderate geometry (reduced accuracy)
     *          - >10: Poor geometry (solution unreliable)
     *          
     *          Fix Modes:
     *          - 0: Invalid/no fix
     *          - 1: SPP (Standard Point Positioning) - autonomous GPS, meter-level
     *          - 2: DGNSS (Differential) - correction applied, sub-meter
     *          - 3: Float RTK - decimeter-level with ambiguity float solution
     *          - 4: Fixed RTK - centimeter-level with ambiguity fixed solution
     *          - 5: Dead Reckoning - using IMU propagation
     *          - 6: SBAS Position - using WAAS/EGNOS/MSAS corrections
     *          
     * @note HDOP/VDOP values used by ArduPilot for accuracy estimation and failsafe logic.
     *       High DOP values may trigger GPS failsafes even with valid fix.
     */
    struct PACKED sbp_dops_t {
        uint32_t tow;    ///< GPS Time of Week (ms)
        uint16_t gdop;   ///< Geometric Dilution of Precision (0.01 units)
        uint16_t pdop;   ///< Position Dilution of Precision (0.01 units)
        uint16_t tdop;   ///< Time Dilution of Precision (0.01 units)
        uint16_t hdop;   ///< Horizontal Dilution of Precision (0.01 units)
        uint16_t vdop;   ///< Vertical Dilution of Precision (0.01 units)
        struct PACKED flags {
            uint8_t fix_mode:3;  ///< Fix mode: 0=invalid, 1=SPP, 2=DGNSS, 3=Float RTK, 4=Fixed RTK, 5=DR, 6=SBAS
            uint8_t res:4;       ///< Reserved bits
            bool raim_repair:1;  ///< RAIM repair applied to solution
        } flags;
    }; // 15 bytes

    /**
     * @struct sbp_pos_llh_t
     * @brief Position LLH message (0x020A) - Geodetic position solution
     * 
     * @details Primary position message providing latitude, longitude, and height
     *          in WGS84 geodetic coordinates.
     *          
     *          Position Format:
     *          - Latitude: Degrees, positive North, range [-90, +90]
     *          - Longitude: Degrees, positive East, range [-180, +180]
     *          - Height: Meters above WGS84 ellipsoid (not MSL/sea level)
     *          
     *          Accuracy Reporting:
     *          - Horizontal accuracy: 1-sigma standard deviation in millimeters
     *          - Vertical accuracy: 1-sigma standard deviation in millimeters
     *          - Derived from position covariance matrix for current fix mode
     *          
     *          Expected Accuracies by Fix Mode:
     *          - SPP (no corrections): 2-5 meters horizontal, 5-10 meters vertical
     *          - DGNSS (with corrections): 0.5-2 meters
     *          - Float RTK: 10-50 centimeters
     *          - Fixed RTK: 1-5 centimeters (optimal conditions)
     *          
     *          INS Integration:
     *          For Duro Inertial units, ins_mode indicates if IMU data was used.
     *          INS integration improves accuracy during brief GPS outages and
     *          provides bridging for dynamic applications.
     *          
     * @note Height is ellipsoidal, not mean sea level. ArduPilot converts to MSL
     *       using EGM96 geoid model for altitude calculations.
     * 
     * @warning Always check fix_mode before using position. Invalid fix (0) means
     *          position data is stale or unreliable.
     */
    struct PACKED sbp_pos_llh_t {
        uint32_t tow;         ///< GPS Time of Week (ms)
        double lat;           ///< Latitude (degrees, positive North)
        double lon;           ///< Longitude (degrees, positive East)
        double height;        ///< Height above WGS84 ellipsoid (meters)
        uint16_t h_accuracy;  ///< Horizontal position 1-sigma accuracy (mm)
        uint16_t v_accuracy;  ///< Vertical position 1-sigma accuracy (mm)
        uint8_t n_sats;       ///< Number of satellites used in solution
        struct PACKED flags {
            uint8_t fix_mode:3;  ///< Fix mode: 0=invalid, 1=SPP, 2=DGNSS, 3=Float RTK, 4=Fixed RTK, 5=DR, 6=SBAS
            uint8_t ins_mode:2;  ///< INS mode: 0=none, 1=INS used (Duro Inertial only)
            uint8_t res:3;       ///< Reserved bits
        } flags;
    }; // 34 bytes

    /**
     * @struct sbp_vel_ned_t
     * @brief Velocity NED message (0x020E) - Velocity in local NED frame
     * 
     * @details Provides 3D velocity in local North-East-Down coordinate frame.
     *          NED frame is the standard aviation reference frame used throughout ArduPilot.
     *          
     *          Velocity Components (mm/s):
     *          - North: Positive towards true North
     *          - East: Positive towards true East  
     *          - Down: Positive towards Earth center (negative = climbing)
     *          
     *          Velocity Derivation:
     *          - Measured Doppler: Direct from satellite Doppler shift (most accurate)
     *          - Computed Doppler: Calculated from carrier phase rates
     *          - Dead Reckoning: IMU-based propagation (during brief GPS loss)
     *          
     *          Accuracy Reporting:
     *          - Horizontal: Combined North-East velocity uncertainty (mm/s)
     *          - Vertical: Down component velocity uncertainty (mm/s)
     *          - Typical accuracy: 0.03 m/s (3 cm/s) with good satellite visibility
     *          
     *          INS Integration:
     *          For Duro Inertial, velocity may be augmented with IMU measurements
     *          providing better accuracy during dynamics and brief GPS outages.
     *          
     * @note Velocity accuracy generally better than position accuracy. Used by
     *       ArduPilot for flight control, navigation, and velocity-based failsafes.
     * 
     * @warning Down velocity positive = descending. This matches NED convention
     *          but differs from some GPS receivers that report climb rate.
     */
    struct PACKED sbp_vel_ned_t {
        uint32_t tow;          ///< GPS Time of Week (ms)
        int32_t n;             ///< Velocity North component (mm/s, positive = North)
        int32_t e;             ///< Velocity East component (mm/s, positive = East)
        int32_t d;             ///< Velocity Down component (mm/s, positive = Down/descending)
        uint16_t h_accuracy;   ///< Horizontal velocity 1-sigma accuracy (mm/s)
        uint16_t v_accuracy;   ///< Vertical velocity 1-sigma accuracy (mm/s)
        uint8_t n_sats;        ///< Number of satellites used in solution
        struct PACKED flags {
            uint8_t vel_mode:3;  ///< Velocity mode: 0=invalid, 1=measured Doppler, 2=computed Doppler, 3=dead reckoning
            uint8_t ins_mode:2;  ///< INS mode: 0=none, 1=INS used (Duro Inertial only)
            uint8_t res:3;       ///< Reserved bits
        } flags;
    }; // 22 bytes

    /**
     * @struct sbp_ext_event_t
     * @brief External Event message (0x0101) - Precise event timestamping
     * 
     * @details Reports accurately GPS-timestamped external events such as camera shutter
     *          activation, LiDAR triggers, or other precisely-timed external signals.
     *          
     *          Time Precision:
     *          - GPS week and millisecond time of week
     *          - Nanosecond residual for sub-millisecond precision
     *          - Total timing accuracy typically < 1 microsecond with valid GPS
     *          
     *          Event Detection:
     *          - Pin level change detection (rising or falling edge)
     *          - Up to 10 external event pins supported (0-9)
     *          - Quality flag indicates GPS time validity
     *          
     *          Applications:
     *          - Camera geotagging with microsecond-accurate timestamps
     *          - LiDAR point cloud time correlation
     *          - External sensor synchronization
     *          - Event sequence reconstruction
     *          
     *          Quality Indicator:
     *          - Good (1): GPS-locked, timing accuracy < 1 microsecond
     *          - Unknown (0): No GPS solution, timing based on local oscillator
     *          
     * @note Used by ArduPilot camera trigger logging to correlate images with
     *       vehicle position and attitude at exact moment of capture.
     * 
     * @warning Only use timestamped events with quality=Good for precision applications.
     *          Unknown quality events may have significant timing drift.
     */
    struct PACKED sbp_ext_event_t {
        uint16_t wn;           ///< GPS week number
        uint32_t tow;          ///< GPS Time of Week (ms)
        int32_t ns_residual;   ///< Nanosecond residual of TOW (range: -500000 to +500000 ns)
        struct PACKED flags {
            uint8_t level:1;       ///< Pin level: 0=Low/falling edge, 1=High/rising edge
            uint8_t quality:1;     ///< Time quality: 0=unknown, 1=good (<1μs accuracy)
            uint8_t res:6;         ///< Reserved bits
        } flags;
        uint8_t pin;           ///< Pin number (0-9) that triggered event
    }; // 12 bytes

    /**
     * @brief Process incoming bytes through SBP parser state machine
     * 
     * @details Called by read() to parse available UART data. Implements state machine
     *          that accumulates bytes for complete messages and validates CRC.
     * 
     * Source: libraries/AP_GPS/AP_GPS_SBP2.cpp
     */
    void _sbp_process();

    /**
     * @brief Process complete validated SBP message
     * 
     * @details Dispatches received message to appropriate handler based on message type.
     *          Updates internal state structures (last_gps_time, last_pos_llh, etc.)
     * 
     * Source: libraries/AP_GPS/AP_GPS_SBP2.cpp
     */
    void _sbp_process_message();

    /**
     * @brief Attempt to update GPS state with accumulated message data
     * 
     * @return true if complete solution available and state updated, false otherwise
     * 
     * @details Checks if all required messages (time, position, velocity, DOP) have been
     *          received for current epoch. If complete, updates GPS state and returns true.
     *          Handles timestamp consistency checking across message types.
     * 
     * Source: libraries/AP_GPS/AP_GPS_SBP2.cpp
     */
    bool _attempt_state_update();

    // ************************************************************************
    // Internal Received Messages State
    // ************************************************************************
    
    /// Timestamp of last received heartbeat (system milliseconds) - for health monitoring
    uint32_t last_heartbeat_received_ms;
    
    /// Timestamp of last RTCM3 injection (system milliseconds) - for correction stream monitoring
    uint32_t last_injected_data_ms;

    /// Most recent heartbeat message - device status and health
    struct sbp_heartbeat_t last_heartbeat;
    
    /// Most recent GPS time message - time synchronization
    struct sbp_gps_time_t  last_gps_time;
    
    /// Most recent DOPS message - fix quality and mode
    struct sbp_dops_t      last_dops;
    
    /// Most recent position message - geodetic position
    struct sbp_pos_llh_t   last_pos_llh;
    
    /// Most recent velocity message - NED velocity
    struct sbp_vel_ned_t   last_vel_ned;
    
    /// Most recent external event - camera trigger or other timestamped event
    struct sbp_ext_event_t last_event;

    /// Time of week of last complete state update (ms) - for message synchronization
    uint32_t               last_full_update_tow;
    
    /// Week number of last complete state update - for time consistency checking
    uint16_t               last_full_update_wn;

    // ************************************************************************
    // Monitoring and Performance Counting
    // ************************************************************************

    /// Count of CRC validation failures - indicates link quality issues
    uint32_t crc_error_counter;

    // ************************************************************************
    // Logging to AP_Logger
    // ************************************************************************

    /**
     * @brief Log complete GPS solution to AP_Logger
     * 
     * @details Writes comprehensive GPS log entry including position, velocity,
     *          accuracy, fix mode, and satellite count. Called when complete
     *          solution update occurs.
     * 
     * Source: libraries/AP_GPS/AP_GPS_SBP2.cpp
     */
    void logging_log_full_update();

    /**
     * @brief Log external event timestamp to AP_Logger
     * 
     * @details Records camera trigger or other external event with precise GPS timestamp
     *          for post-flight image geotagging and event reconstruction.
     * 
     * Source: libraries/AP_GPS/AP_GPS_SBP2.cpp
     */
    void logging_ext_event();

    /**
     * @brief Calculate signed distance between two time-of-week values with modulo wrap
     * 
     * @param[in] tow1_ms First time of week in milliseconds
     * @param[in] tow2_ms Second time of week in milliseconds
     * @param[in] mod Modulo value for wrap detection (typically 604800000 ms = 1 week)
     * 
     * @return Signed distance (tow1 - tow2) accounting for week rollover
     * 
     * @details Handles time-of-week wraparound at week boundaries. Used to determine
     *          if messages are from same epoch and calculate time differences.
     * 
     * Source: libraries/AP_GPS/AP_GPS_SBP2.cpp
     */
    int32_t distMod(int32_t tow1_ms, int32_t tow2_ms, int32_t mod);

};
#endif
