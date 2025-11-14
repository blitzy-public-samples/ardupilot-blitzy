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
 * @file AP_GPS_MSP.h
 * @brief MSP (MultiWii Serial Protocol) GPS backend for ArduPilot
 * 
 * @details This GPS backend receives GPS position data via the MSP protocol
 *          from external sources such as:
 *          - Betaflight/iNav flight controllers
 *          - DJI OSD devices
 *          - Other MSP-compatible devices
 *          
 *          The driver accepts GPS_RAW_INT data packets and enables GPS sharing
 *          between flight controllers and ArduPilot or GPS injection via MSP OSD.
 *          
 *          Coordinate Systems:
 *          - Position: WGS84 latitude/longitude/altitude
 *          - Velocity: Ground speed and course over ground
 *          
 *          Use Cases:
 *          - Sharing GPS from MSP flight controller to ArduPilot
 *          - GPS injection via DJI OSD or MSP devices
 *          - Integration with Betaflight/iNav systems
 * 
 * @note Requires HAL_MSP_GPS_ENABLED to be defined
 * 
 * Source: libraries/AP_GPS/AP_GPS_MSP.h, libraries/AP_GPS/AP_GPS_MSP.cpp
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#include "AP_GPS.h"
#include "GPS_Backend.h"

#if HAL_MSP_GPS_ENABLED

/**
 * @class AP_GPS_MSP
 * @brief GPS backend that receives position data via MSP protocol
 * 
 * @details This backend processes GPS data received from MSP-compatible devices
 *          rather than directly from a GPS receiver. It's designed to work with
 *          external flight controllers (Betaflight/iNav) or OSD systems that
 *          already have GPS data and can share it via the MSP protocol.
 *          
 *          The driver maintains compatibility with MSP GPS_RAW_INT messages and
 *          translates them into ArduPilot's internal GPS state representation.
 *          
 *          Data Flow:
 *          1. External device reads GPS hardware
 *          2. Device sends GPS_RAW_INT via MSP protocol
 *          3. AP_MSP routes packet to this backend via handle_msp()
 *          4. Backend updates internal GPS state
 *          5. read() method signals new data availability
 * 
 * @note This backend does not communicate directly with GPS hardware
 * @warning Ensure MSP data source has reliable GPS fix before flight
 */
class AP_GPS_MSP : public AP_GPS_Backend
{
public:

    using AP_GPS_Backend::AP_GPS_Backend;

    /**
     * @brief Check for new GPS data received via MSP
     * 
     * @details This method is called by the GPS driver update loop to check
     *          if new GPS data has been received since the last call. Unlike
     *          serial GPS backends, this doesn't actively read data but checks
     *          a flag set by handle_msp() when new MSP GPS packets arrive.
     * 
     * @return true if new GPS data has been received since last call, false otherwise
     * 
     * @note Called at GPS update rate (typically 5-10 Hz)
     */
    bool read() override;
    
    /**
     * @brief Process incoming MSP GPS data packet
     * 
     * @details This method is called by the AP_MSP subsystem when a GPS_RAW_INT
     *          message is received via MSP protocol. It extracts GPS position,
     *          velocity, fix type, and satellite count from the MSP packet and
     *          updates the internal GPS state.
     *          
     *          MSP GPS Data Contents:
     *          - Latitude/Longitude: WGS84 coordinates in degrees * 1e7
     *          - Altitude: MSL altitude in meters
     *          - Ground speed: Speed over ground in cm/s
     *          - Ground course: Course over ground in degrees * 100
     *          - Fix type: 0=No Fix, 1=Dead Reckoning, 2=2D, 3=3D
     *          - Number of satellites
     * 
     * @param[in] pkt MSP GPS data structure containing position, velocity, fix status
     * 
     * @note This method is called from AP_MSP context when MSP packets arrive
     * @see MSP::msp_gps_data_message_t for packet structure definition
     */
    void handle_msp(const MSP::msp_gps_data_message_t &pkt) override;

    /**
     * @brief Get GPS backend type identifier
     * @return "MSP" string identifier for this GPS backend type
     */
    const char *name() const override { return "MSP"; }

    /**
     * @brief Get estimated GPS data lag time
     * 
     * @details Returns the estimated delay between GPS measurement time and
     *          when the data is available to ArduPilot. For MSP GPS, this
     *          includes the lag from the original GPS receiver plus additional
     *          delay from MSP protocol transmission.
     * 
     * @param[out] lag_sec Estimated lag time in seconds
     * 
     * @return true if lag estimate is available, false otherwise
     * 
     * @note GPS lag affects state estimation and should be minimized for best performance
     */
    bool get_lag(float &lag_sec) const override;

private:
    /**
     * @brief Flag indicating new GPS data has been received
     * 
     * @details Set to true by handle_msp() when new MSP GPS packet arrives.
     *          Cleared by read() after processing. Used to signal GPS driver
     *          that fresh data is available.
     */
    bool new_data;
};

#endif // HAL_MSP_GPS_ENABLED
