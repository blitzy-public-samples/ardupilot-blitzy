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
 * @file AP_GPS_MAV.h
 * @brief MAVLink GPS backend - receives GPS data via MAVLink messages
 * 
 * @details This GPS backend does not communicate directly with GPS hardware.
 *          Instead, it accepts GPS position and velocity data from external sources
 *          via MAVLink GPS_INPUT or HIL_GPS messages. This enables several use cases:
 *          
 *          - GPS injection for testing and development without physical GPS hardware
 *          - External positioning systems (motion capture, UWB beacons, visual odometry)
 *            converted to GPS format by ground station or companion computer
 *          - Hardware-In-Loop (HIL) simulation where GPS data is provided by simulator
 *          - Companion computer GPS forwarding for GPS-denied navigation solutions
 * 
 * @note This backend requires explicit configuration via GPS_TYPE parameter.
 *       It does not auto-detect like serial GPS drivers.
 * 
 * @note Coordinate Systems:
 *       - Position: WGS84 geodetic (latitude/longitude in degrees, altitude in meters)
 *       - Velocity: NED (North-East-Down) frame in m/s
 *       - Time: GPS time (week number and time of week)
 * 
 * Source: libraries/AP_GPS/AP_GPS_MAV.h, libraries/AP_GPS/AP_GPS_MAV.cpp
 */
#pragma once

#include "AP_GPS_config.h"

#if AP_GPS_MAV_ENABLED

#include <AP_HAL/AP_HAL_Boards.h>

#include "AP_GPS.h"
#include "GPS_Backend.h"

/**
 * @class AP_GPS_MAV
 * @brief GPS backend that receives position data via MAVLink messages
 * 
 * @details This backend processes GPS data received through MAVLink GPS_INPUT
 *          or HIL_GPS messages rather than directly from GPS hardware. It
 *          inherits from AP_GPS_Backend and implements the standard GPS driver
 *          interface while receiving data from external sources.
 *          
 *          The backend maintains jitter correction to smooth incoming data
 *          and tracks GPS week rollovers for proper time handling.
 *          
 *          Typical data flow:
 *          1. External source sends GPS_INPUT or HIL_GPS MAVLink message
 *          2. handle_msg() processes message and updates internal state
 *          3. read() checks for new data and updates GPS state in AP_GPS
 *          4. Vehicle uses GPS data for navigation as with hardware GPS
 * 
 * @note This backend does not perform hardware detection. It must be explicitly
 *       configured via GPS_TYPE parameter (typically GPS_TYPE_MAV).
 * 
 * @warning Ensure external GPS source provides valid WGS84 coordinates and
 *          accurate timestamps to prevent navigation errors.
 */
class AP_GPS_MAV : public AP_GPS_Backend {
public:

    using AP_GPS_Backend::AP_GPS_Backend;

    /**
     * @brief Check for new GPS data received via MAVLink
     * 
     * @details Called by AP_GPS at regular intervals to poll for new GPS data.
     *          Returns true if handle_msg() has processed a new GPS message
     *          since the last read() call. When new data is available, this
     *          method triggers the base class to update navigation state.
     * 
     * @return true if new GPS data is available, false otherwise
     * 
     * @note Called at GPS update rate (typically 5-10 Hz)
     */
    bool read() override;

    /**
     * @brief Detection method for serial GPS auto-detection (not applicable)
     * 
     * @details This method is part of the GPS_Backend interface for serial GPS
     *          drivers that need to detect GPS hardware by analyzing serial data.
     *          For MAVLink GPS, this method is not used since the backend does
     *          not communicate via serial port and requires explicit configuration.
     * 
     * @param[in,out] state    Detection state structure (unused for MAVLink GPS)
     * @param[in]     data     Serial byte to analyze (unused for MAVLink GPS)
     * 
     * @return Always returns false - MAVLink GPS does not support auto-detection
     * 
     * @note MAVLink GPS must be explicitly configured via GPS_TYPE parameter.
     *       It cannot be auto-detected from serial data streams.
     */
    static bool _detect(struct MAV_detect_state &state, uint8_t data);

    /**
     * @brief Process incoming MAVLink GPS message
     * 
     * @details Handles GPS_INPUT (message ID 232) and HIL_GPS (message ID 113)
     *          MAVLink messages. Extracts position, velocity, accuracy, and timing
     *          information from the message and updates internal GPS state.
     *          
     *          GPS_INPUT messages provide:
     *          - WGS84 position (lat/lon/alt)
     *          - NED velocity
     *          - GPS accuracy metrics (hdop, vdop, speed accuracy)
     *          - Satellite count and fix type
     *          - GPS time (week and time of week)
     *          
     *          HIL_GPS messages provide similar data for Hardware-In-Loop simulation.
     *          
     *          This method applies jitter correction to smooth timestamp irregularities
     *          and handles GPS week rollovers for proper time conversion.
     * 
     * @param[in] msg MAVLink message structure containing GPS_INPUT or HIL_GPS data
     * 
     * @note This method is called by the MAVLink message handling system when
     *       GPS_INPUT or HIL_GPS messages are received.
     * 
     * @warning Invalid coordinates or time data will be rejected to prevent
     *          navigation failures. Ensure external source provides valid WGS84
     *          positions and accurate GPS time.
     */
    void handle_msg(const mavlink_message_t &msg) override;

    /**
     * @brief Get the name identifier for this GPS backend
     * 
     * @return "MAV" - identifier string for MAVLink GPS backend
     * 
     * @note Used for logging and ground station display to identify GPS type
     */
    const char *name() const override { return "MAV"; }

private:
    bool _new_data;          ///< Flag indicating new GPS data received since last read()
    uint32_t first_week;     ///< GPS week number from first message for week rollover detection
    JitterCorrection jitter{2000};  ///< Jitter correction with 2000ms max delta for timestamp smoothing
};

#endif  // AP_GPS_MAV_ENABLED
