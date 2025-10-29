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
 * @file AP_GPS_ExternalAHRS.h
 * @brief GPS backend adapter for external AHRS systems providing GPS data
 * 
 * @details This file implements a GPS backend that accepts GPS position and velocity
 *          data from external AHRS units (VectorNav, MicroStrain, etc.) via the
 *          AP_ExternalAHRS interface. Rather than directly communicating with GPS
 *          hardware, this backend receives processed GPS information that has been
 *          forwarded from the external AHRS unit.
 * 
 * @note Requires AP_EXTERNAL_AHRS_ENABLED to be defined for compilation.
 * 
 * Coordinate Systems:
 * - Position: WGS84 geodetic (latitude/longitude in degrees, altitude in meters above ellipsoid)
 * - Velocity: NED (North-East-Down) frame in m/s
 * 
 * Units:
 * - Position: degrees (lat/lon), meters (altitude)
 * - Velocity: m/s in NED frame
 * - Accuracy: meters (horizontal/vertical)
 * 
 * Source: libraries/AP_GPS/AP_GPS_ExternalAHRS.h:1-47
 * Source: libraries/AP_GPS/AP_GPS_ExternalAHRS.cpp (implementation)
 */

//
//  ExternalAHRS GPS driver which accepts gps position data from an external AHRS unit
//
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>

#include "AP_GPS.h"
#include "GPS_Backend.h"

#if AP_EXTERNAL_AHRS_ENABLED

/**
 * @class AP_GPS_ExternalAHRS
 * @brief GPS backend that receives position data from external AHRS systems
 * 
 * @details This is a lightweight adapter GPS backend that accepts GPS data from external
 *          AHRS devices. It does not directly communicate with GPS hardware - instead it
 *          receives processed GPS information forwarded from the external AHRS unit.
 *          
 *          Supported external AHRS systems include:
 *          - VectorNav VN-100/200/300 series
 *          - LORD MicroStrain 3DM-GX5/GQ7 series
 *          - Other systems implementing AP_ExternalAHRS interface
 *          
 *          The backend acts as a bridge between the external AHRS GPS data and the
 *          ArduPilot GPS subsystem, allowing the external AHRS's GPS receiver to be
 *          used as the vehicle's primary GPS source.
 *          
 *          Data Flow:
 *          External AHRS Device → AP_ExternalAHRS → handle_external() → GPS state cache
 *          GPS Frontend → read() → Returns cached GPS state
 * 
 * @note External AHRS must be configured to output GPS data messages for this backend
 *       to receive position updates.
 * 
 * @warning Position accuracy depends on the external AHRS GPS integration quality and
 *          the communication latency between the external AHRS and the flight controller.
 */
class AP_GPS_ExternalAHRS : public AP_GPS_Backend
{
public:

    using AP_GPS_Backend::AP_GPS_Backend;

    /**
     * @brief Check for new GPS data from external AHRS
     * 
     * @details Called periodically by the GPS frontend to check if new GPS data is
     *          available. This method returns cached data that was previously set by
     *          handle_external() when the external AHRS forwarded GPS information.
     *          
     *          The method returns true only once per GPS update - subsequent calls
     *          return false until new data arrives via handle_external().
     * 
     * @return true if new GPS data is available since the last call, false otherwise
     * 
     * @note Does not perform any I/O operations - simply returns cached data state.
     *       The actual GPS data reception occurs in handle_external().
     */
    bool read() override;
    
    /**
     * @brief Receive GPS data message from external AHRS system
     * 
     * @details Called by AP_ExternalAHRS when a GPS message is received from the
     *          external device. This method copies the GPS data from the external AHRS
     *          packet into the GPS backend's internal state, making it available for
     *          the next read() call.
     *          
     *          Data copied includes:
     *          - Position: WGS84 latitude, longitude, altitude
     *          - Velocity: NED frame velocity components (north, east, down in m/s)
     *          - Fix quality: GPS fix type (no fix, 2D, 3D, RTK, etc.)
     *          - Satellite count: Number of satellites used in solution
     *          - Accuracy metrics: Horizontal and vertical position accuracy (meters)
     * 
     * @param[in] pkt GPS data packet from external AHRS containing position, velocity,
     *                fix type, satellite count, and accuracy information
     * 
     * @note This method is called from the AP_ExternalAHRS context, not the GPS
     *       frontend thread. Sets the new_data flag to signal read() that fresh
     *       data is available.
     */
    void handle_external(const AP_ExternalAHRS::gps_data_message_t &pkt) override;

    /**
     * @brief Get backend name for identification
     * 
     * @details Returns a string identifier for this GPS backend used in logging,
     *          status messages, and debugging output. This helps distinguish the
     *          external AHRS GPS source from other GPS backends in the system.
     * 
     * @return "ExternalAHRS" string identifier
     */
    const char *name() const override { return "ExternalAHRS"; }

    /**
     * @brief Get GPS measurement lag/latency from external AHRS
     * 
     * @details Returns the time delay between the GPS measurement timestamp and when
     *          the data becomes available to the flight controller. This lag value is
     *          used by the EKF (Extended Kalman Filter) for proper state estimation,
     *          accounting for the processing delay in the external AHRS and the
     *          communication latency.
     *          
     *          The lag depends on:
     *          - External AHRS internal processing time
     *          - Communication protocol latency (serial, CAN, etc.)
     *          - GPS receiver processing time in the external unit
     * 
     * @param[out] lag_sec GPS data lag time in seconds (typically 0.05 to 0.2 seconds)
     * 
     * @return true if lag value is available from the external AHRS, false otherwise
     * 
     * @note If the external AHRS does not provide lag information, this method may
     *       return false or a default estimated value.
     */
    bool get_lag(float &lag_sec) const override;

private:
    /**
     * @brief Flag indicating GPS data has been updated but not yet read
     * 
     * @details Set to true by handle_external() when new GPS data arrives from the
     *          external AHRS. Cleared to false by read() after the GPS frontend has
     *          retrieved the data. This ensures read() returns true only once per
     *          GPS update, preventing the same data from being processed multiple times.
     */
    bool new_data;
};

#endif // AP_EXTERNAL_AHRS_ENABLED

