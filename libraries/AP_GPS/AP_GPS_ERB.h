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
 * @file AP_GPS_ERB.h
 * @brief Emlid Reach Binary (ERB) protocol GPS driver
 * 
 * @details Implements ERB binary protocol for Emlid Reach RTK GPS receivers.
 *          The ERB protocol is a compact binary format designed for efficient
 *          transmission of position, velocity, and RTK correction data from
 *          Emlid Reach receivers to autopilot systems.
 *          
 *          Protocol specification: http://files.emlid.com/ERB.pdf
 * 
 * @note Supports VER, POS, STAT, DOPS, VEL, and RTK message types
 */

//
//  Emlid Reach Binary (ERB) GPS driver for ArduPilot.
//  ERB protocol: http://files.emlid.com/ERB.pdf

#pragma once

#include "AP_GPS.h"
#include "GPS_Backend.h"

#if AP_GPS_ERB_ENABLED
/**
 * @class AP_GPS_ERB
 * @brief GPS backend for Emlid Reach receivers using ERB binary protocol
 * 
 * @details Implements binary protocol parser for Emlid Reach RTK GPS receivers.
 *          Parses VER (version), POS (position), STAT (status/fix type), DOPS 
 *          (dilution of precision), VEL (velocity), and RTK (baseline/corrections) 
 *          messages from the receiver.
 *          
 *          The parser supports RTK fixed, float, and single solutions, providing
 *          high-precision positioning when RTK corrections are available. The
 *          protocol uses a state machine to process incoming bytes, validating
 *          message structure and checksums.
 * 
 * @note Protocol uses 2-byte preamble (0x45, 0x52 = 'E', 'R'), message ID, 
 *       2-byte length, payload, and 2-byte Fletcher-16 checksum
 * 
 * @warning Time synchronization requires valid GPS time of week (TOW) from receiver.
 *          Ensure receiver has valid time solution before relying on timestamps.
 */
class AP_GPS_ERB : public AP_GPS_Backend
{
public:

    using AP_GPS_Backend::AP_GPS_Backend;

    /**
     * @brief Read and parse ERB protocol bytes from GPS serial port
     * 
     * @details Implements state machine parser that processes incoming bytes
     *          through the following stages:
     *          1. Preamble detection (0x45, 0x52)
     *          2. Header parsing (message ID and length)
     *          3. Payload reception
     *          4. Checksum validation (Fletcher-16)
     *          
     *          When a complete valid message is received, updates internal
     *          GPS state (position, velocity, fix type) accordingly.
     * 
     * @return true if complete valid message received and processed, false otherwise
     * 
     * @note Called repeatedly by AP_GPS at main loop rate to process serial data.
     *       May need multiple calls to receive a complete message.
     */
    bool read() override;

    /**
     * @brief Report highest GPS fix type supported by ERB protocol
     * 
     * @details ERB protocol supports the full range of fix types from NO_FIX
     *          through RTK_FIXED, making it suitable for high-precision
     *          RTK applications.
     * 
     * @return GPS_OK_FIX_3D_RTK_FIXED indicating full RTK fixed solution capability
     * 
     * @note ERB fix types: NO_FIX (0), SINGLE (1), RTK_FLOAT (2), RTK_FIXED (3)
     */
    AP_GPS::GPS_Status highest_supported_status(void) override { return AP_GPS::GPS_OK_FIX_3D_RTK_FIXED; }

#if HAL_GCS_ENABLED
    /**
     * @brief Indicate support for MAVLink GPS_RTK_DATA messages
     * 
     * @details ERB protocol provides comprehensive RTK data including baseline
     *          vectors (North, East, Down), number of satellites used in RTK
     *          calculation, age of corrections, and ambiguity resolution ratio.
     *          This data is suitable for MAVLink GPS_RTK_DATA message reporting.
     * 
     * @return true - ERB provides RTK baseline and correction age for MAVLink reporting
     * 
     * @note Only available when HAL_GCS_ENABLED is defined
     */
    bool supports_mavlink_gps_rtk_message() const override { return true; }
#endif

    /**
     * @brief Static detection method to identify ERB protocol in byte stream
     * 
     * @details Called by AP_GPS during auto-baud/protocol detection phase.
     *          Searches for ERB preamble sequence (0x45, 0x52 = 'E', 'R') in
     *          incoming byte stream to identify when receiver is using ERB protocol.
     *          
     *          Detection state machine tracks progress through preamble detection.
     * 
     * @param[in,out] state Detection state machine structure maintaining search progress
     * @param[in]     data  Single byte to process from GPS serial stream
     * 
     * @return true if ERB preamble sequence detected (0x45, 0x52), false otherwise
     * 
     * @note Called before ERB backend is instantiated, during protocol identification
     */
    static bool _detect(struct ERB_detect_state &state, uint8_t data);

    /**
     * @brief Get backend name for identification and logging
     * 
     * @return "ERB" string identifier for this GPS backend
     * 
     * @note Used in GPS status messages and logging to identify receiver type
     */
    const char *name() const override { return "ERB"; }

private:
    /**
     * @brief ERB protocol header structure (5 bytes)
     * 
     * @details Appears at the start of every ERB message. Header is followed
     *          by payload (length specified in header) and 2-byte Fletcher-16
     *          checksum calculated over header (excluding preamble) and payload.
     */
    struct PACKED erb_header {
        uint8_t preamble1;  ///< First preamble byte (0x45 = 'E')
        uint8_t preamble2;  ///< Second preamble byte (0x52 = 'R')
        uint8_t msg_id;     ///< Message type identifier (VER/POS/STAT/DOPS/VEL/RTK)
        uint16_t length;    ///< Payload length in bytes (does not include header or checksum)
    };
    /**
     * @brief Version message payload (MSG_VER = 0x01)
     * 
     * @details Reports receiver firmware version. Useful for debugging
     *          protocol compatibility issues.
     */
    struct PACKED erb_ver {
        uint32_t time;       ///< GPS time of week of the navigation epoch in milliseconds
        uint8_t ver_high;    ///< Firmware major version number
        uint8_t ver_medium;  ///< Firmware minor version number
        uint8_t ver_low;     ///< Firmware patch version number
    };
    /**
     * @brief Position message payload (MSG_POS = 0x02)
     * 
     * @details Provides GNSS position solution in WGS84 geodetic coordinates
     *          with accuracy estimates. Primary message for position updates.
     */
    struct PACKED erb_pos {
        uint32_t time;                ///< GPS time of week of the navigation epoch in milliseconds
        double longitude;             ///< Longitude in degrees (WGS84, positive East, -180 to +180)
        double latitude;              ///< Latitude in degrees (WGS84, positive North, -90 to +90)
        double altitude_ellipsoid;    ///< Height above WGS84 ellipsoid in meters
        double altitude_msl;          ///< Height above mean sea level (MSL) in meters
        uint32_t horizontal_accuracy; ///< Horizontal position accuracy estimate in millimeters (1σ)
        uint32_t vertical_accuracy;   ///< Vertical position accuracy estimate in millimeters (1σ)
    };
    /**
     * @brief Status message payload (MSG_STAT = 0x03)
     * 
     * @details Reports receiver fix status including fix type (NONE/SINGLE/FLOAT/FIXED),
     *          GPS time reference, and number of satellites used in solution.
     */
    struct PACKED erb_stat {
        uint32_t time;      ///< GPS time of week of the navigation epoch in milliseconds
        uint16_t week;      ///< GPS week number (weeks since January 6, 1980)
        uint8_t fix_type;   ///< Fix quality: 0=NONE, 1=SINGLE, 2=FLOAT, 3=FIXED (see erb_fix_type enum)
        uint8_t fix_status; ///< Additional fix status flags (receiver-specific)
        uint8_t satellites; ///< Number of satellites used in position solution
    };
    /**
     * @brief Dilution of Precision message payload (MSG_DOPS = 0x04)
     * 
     * @details Provides geometric quality indicators for the position solution.
     *          Lower DOP values indicate better satellite geometry and more
     *          accurate position estimates.
     * 
     * @note All DOP values are scaled by 100. Multiply by 0.01 to get actual DOP.
     *       Example: hDOP=150 represents actual hDOP of 1.50
     */
    struct PACKED erb_dops {
        uint32_t time;      ///< GPS time of week of the navigation epoch in milliseconds
        uint16_t gDOP;      ///< Geometric DOP (dimensionless, scale 0.01)
        uint16_t pDOP;      ///< Position DOP (dimensionless, scale 0.01)
        uint16_t vDOP;      ///< Vertical DOP (dimensionless, scale 0.01)
        uint16_t hDOP;      ///< Horizontal DOP (dimensionless, scale 0.01)
    };
    /**
     * @brief Velocity message payload (MSG_VEL = 0x05)
     * 
     * @details Provides 3D velocity vector in NED (North-East-Down) frame,
     *          ground speed, heading, and velocity accuracy estimate.
     * 
     * @note Velocity components use NED coordinate frame convention.
     *       Heading is scaled by 1e-5 (multiply by 0.00001 to get degrees).
     */
    struct PACKED erb_vel {
        uint32_t time;           ///< GPS time of week of the navigation epoch in milliseconds
        int32_t vel_north;       ///< North velocity component in cm/s (NED frame, positive North)
        int32_t vel_east;        ///< East velocity component in cm/s (NED frame, positive East)
        int32_t vel_down;        ///< Down velocity component in cm/s (NED frame, positive Down)
        uint32_t speed_2d;       ///< Ground speed (horizontal magnitude) in cm/s
        int32_t heading_2d;      ///< Heading of motion 2-D in 1e-5 degrees (multiply by 0.00001, 0-360°)
        uint32_t speed_accuracy; ///< Velocity accuracy estimate in cm/s (1σ)
    };
    /**
     * @brief RTK information message payload (MSG_RTK = 0x07)
     * 
     * @details Provides Real-Time Kinematic (RTK) solution information including
     *          3D baseline vector from base station to rover, correction age,
     *          and ambiguity resolution quality indicator.
     *          
     *          The baseline vector describes rover position relative to base
     *          station in NED frame. AR ratio indicates confidence in integer
     *          ambiguity resolution (higher is better, typically >3.0 for FIXED).
     * 
     * @note Baseline components use NED coordinate frame (North-East-Down).
     *       AR ratio is scaled by 10 (divide by 10.0 for actual ratio).
     *       Age overflow (0xFFFF) indicates corrections older than ~655 seconds.
     */
    struct PACKED erb_rtk {
        uint8_t base_num_sats;       ///< Number of satellites used in RTK baseline calculation
        uint16_t age_cs;             ///< Age of RTK corrections in centiseconds (0=no corrections, 0xFFFF=overflow)
        int32_t baseline_N_mm;       ///< North baseline component in millimeters (rover relative to base, NED frame)
        int32_t baseline_E_mm;       ///< East baseline component in millimeters (rover relative to base, NED frame)
        int32_t baseline_D_mm;       ///< Down baseline component in millimeters (rover relative to base, NED frame)
        uint16_t ar_ratio;           ///< Ambiguity resolution ratio * 10 (higher = better, typically >30 for FIXED)
        uint16_t base_week_number;   ///< GPS week number of baseline measurement
        uint32_t base_time_week_ms;  ///< GPS time of week of baseline measurement in milliseconds
    };

    // Receive buffer
    union PACKED {
        DEFINE_BYTE_ARRAY_METHODS
        erb_ver ver;
        erb_pos pos;
        erb_stat stat;
        erb_dops dops;
        erb_vel vel;
        erb_rtk rtk;
    } _buffer;

    /**
     * @brief ERB protocol constants for message framing and identification
     * 
     * @details Defines preamble bytes used for message synchronization and
     *          message ID values for each supported message type.
     */
    enum erb_protocol_bytes {
        PREAMBLE1 = 0x45,  ///< First preamble byte ('E' in ASCII)
        PREAMBLE2 = 0x52,  ///< Second preamble byte ('R' in ASCII)
        MSG_VER = 0x01,    ///< Version information message
        MSG_POS = 0x02,    ///< Position solution message
        MSG_STAT = 0x03,   ///< Status and fix type message
        MSG_DOPS = 0x04,   ///< Dilution of precision message
        MSG_VEL = 0x05,    ///< Velocity solution message
        MSG_RTK = 0x07,    ///< RTK information message
    };

    /**
     * @brief ERB fix type enumeration from STAT message
     * 
     * @details Indicates GNSS position solution quality. RTK modes (FLOAT, FIX)
     *          require base station corrections. FIXED provides centimeter-level
     *          accuracy when ambiguities are resolved.
     */
    enum erb_fix_type {
        FIX_NONE = 0x00,    ///< No position fix available
        FIX_SINGLE = 0x01,  ///< Single point positioning (standalone GPS, meter-level accuracy)
        FIX_FLOAT = 0x02,   ///< RTK float solution (decimeter-level accuracy, ambiguities not resolved)
        FIX_FIX = 0x03,     ///< RTK fixed solution (centimeter-level accuracy, ambiguities resolved)
    };

    /**
     * @brief Fletcher-16 checksum accumulators for message validation
     * 
     * @details ERB protocol uses Fletcher-16 checksum calculated over message
     *          header (excluding preamble bytes) and payload. Checksum is
     *          computed incrementally as bytes are received.
     *          
     *          Algorithm: _ck_a = sum of all bytes (mod 256)
     *                     _ck_b = sum of all _ck_a values (mod 256)
     */
    uint8_t _ck_a;  ///< First checksum accumulator (sum of bytes)
    uint8_t _ck_b;  ///< Second checksum accumulator (sum of sums)

    /**
     * @brief Parser state machine variables
     * 
     * @details Tracks progress through message reception stages:
     *          - Searching for preamble bytes (0x45, 0x52)
     *          - Reading header (message ID and length)
     *          - Receiving payload bytes
     *          - Validating checksum
     */
    uint8_t _step;              ///< Current parser state (preamble search, header, payload, checksum)
    uint8_t _msg_id;            ///< Message ID of packet currently being received
    uint16_t _payload_length;   ///< Expected payload size from header (bytes)
    uint16_t _payload_counter;  ///< Number of payload bytes received so far

    /**
     * @brief Message processing state and timing
     * 
     * @details Tracks message reception timestamps and flags indicating when
     *          new data is available for GPS state update. Fix count used for
     *          periodic processing tasks (e.g., health checks, logging).
     */
    uint8_t _fix_count;      ///< 8-bit counter incremented on each position message (for periodic processing)

    uint32_t _last_pos_time; ///< GPS time of week (ms) of last valid position message
    uint32_t _last_vel_time; ///< GPS time of week (ms) of last valid velocity message

    bool _new_position:1;    ///< Flag: new position data available for state update
    bool _new_speed:1;       ///< Flag: new velocity data available for state update

    /**
     * @brief Parse complete message and update GPS state
     * 
     * @details Called when a complete valid message has been received and
     *          checksum validated. Processes message payload based on message
     *          type and updates internal GPS state (position, velocity, fix
     *          type, accuracy, etc.).
     * 
     * @return true if message successfully parsed and state updated, false on error
     * 
     * @note Updates _new_position and _new_speed flags when respective data is processed
     */
    bool _parse_gps();

    /**
     * @brief Pending fix status from most recent STAT message
     * 
     * @details ERB sends STAT (status) and POS (position) messages separately.
     *          Fix type from STAT message is stored here and applied to GPS
     *          state when the corresponding POS message is received, ensuring
     *          fix type and position are synchronized.
     */
    AP_GPS::GPS_Status next_fix = AP_GPS::NO_FIX;
};
#endif
