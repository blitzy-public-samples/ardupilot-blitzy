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
 * @file AP_GPS_SITL.h
 * @brief Software-In-The-Loop GPS simulation backend for ArduPilot
 * 
 * @details This file implements a simulated GPS backend that generates realistic
 *          GPS data from the SITL physics simulation, enabling testing and development
 *          without physical GPS hardware.
 * 
 * Source: libraries/AP_GPS/AP_GPS_SITL.h
 */

#pragma once

#include "GPS_Backend.h"

#include <SITL/SITL.h>

#if AP_SIM_GPS_ENABLED

/**
 * @class AP_GPS_SITL
 * @brief Simulated GPS backend for Software-In-The-Loop testing
 * 
 * @details This class provides a GPS backend that generates realistic GPS measurements
 *          from the SITL vehicle physics simulation. It simulates various GPS behaviors
 *          including position accuracy, fix types, satellite counts, and failure modes.
 * 
 *          Key Features:
 *          - Realistic position derived from SITL vehicle physics simulation
 *          - Velocity data from simulated vehicle motion (NED frame)
 *          - Configurable fix types (2D/3D/DGPS/RTK_FLOAT/RTK_FIXED)
 *          - Simulated satellite count and accuracy metrics (HDOP/VDOP)
 *          - GPS glitches, dropouts, and jamming simulation
 *          - Configurable GPS lag/latency for realistic timing
 *          - Multi-GPS simulation with configurable position offsets
 *          - GPS failure injection for failsafe testing
 * 
 *          Simulation Configuration Parameters:
 *          - SIM_GPS_DISABLE: Simulate complete GPS failure
 *          - SIM_GPS_DELAY: GPS lag in milliseconds (simulates processing delay)
 *          - SIM_GPS_TYPE: Simulated fix type (None/2D/3D/DGPS/RTK)
 *          - SIM_GPS_NUMSATS: Simulated satellite count (affects accuracy)
 *          - SIM_GPS_GLITCH: Inject random position glitches
 *          - SIM_GPS_HERTZ: GPS update rate in Hz
 *          - SIM_GPS_BYTELOSS: Simulate data corruption/loss
 *          - SIM_GPS_POS_*: Position offsets for multi-GPS simulation
 * 
 *          Common Use Cases:
 *          - Development and testing without physical GPS hardware
 *          - Failsafe scenario testing (GPS loss, degraded accuracy)
 *          - EKF tuning and state estimation verification
 *          - RTK GPS simulation and testing
 *          - GPS-denied navigation algorithm development
 *          - Multi-GPS redundancy testing
 *          - Automated regression testing in CI/CD pipelines
 * 
 * @note This class is only available when compiled for SITL (AP_SIM_GPS_ENABLED)
 * @note GPS data is generated from SITL::SIM vehicle state, which must be updated
 *       by the SITL physics simulation before GPS data can be generated
 * 
 * @warning Simulated GPS behavior may not perfectly match real hardware quirks
 *          and limitations. Always validate critical functionality on real hardware.
 * 
 * @see AP_GPS_Backend Base class defining GPS backend interface
 * @see SITL::SIM Physics simulation providing vehicle state
 * 
 * Source: libraries/AP_GPS/AP_GPS_SITL.h, libraries/AP_GPS/AP_GPS_SITL.cpp
 */
class AP_GPS_SITL : public AP_GPS_Backend
{

public:

    /**
     * @brief Constructor inherited from GPS_Backend
     * 
     * @details Uses the base class constructor to initialize the GPS backend
     *          with reference to the GPS frontend state and instance number.
     */
    using AP_GPS_Backend::AP_GPS_Backend;

    /**
     * @brief Generate and read simulated GPS data from SITL physics state
     * 
     * @details This method queries the SITL simulation state to generate realistic
     *          GPS measurements including:
     *          - Position (latitude, longitude, altitude) from simulated vehicle location
     *          - Velocity (North, East, Down) from simulated vehicle motion
     *          - Fix type and satellite count from configured simulation parameters
     *          - Accuracy metrics (HDOP, VDOP) based on simulated conditions
     *          - Timing and lag simulation based on configured GPS delay
     * 
     *          The method applies configured simulation effects such as:
     *          - Position glitches and dropouts
     *          - GPS lag/latency delays
     *          - Accuracy degradation
     *          - Complete GPS failure (if SIM_GPS_DISABLE is set)
     * 
     *          GPS data is only updated at the configured rate (SIM_GPS_HERTZ) to
     *          simulate realistic GPS update timing. Between updates, the method
     *          returns false to indicate no new data is available.
     * 
     * @return true if new GPS data is available and has been written to the frontend
     * @return false if no new data is available (not yet time for next update)
     * 
     * @note This method is called at main loop rate but only generates new GPS data
     *       at the configured GPS update rate (typically 5-10 Hz)
     * @note Simulated GPS data is time-stamped with current simulation time
     * 
     * @see AP_GPS_Backend::publish() Method used to send data to GPS frontend
     * 
     * Source: libraries/AP_GPS/AP_GPS_SITL.cpp
     */
    bool        read() override;

    /**
     * @brief Get the name identifier for this GPS backend
     * 
     * @return "SITL" - Identifier string for Software-In-The-Loop GPS simulation
     * 
     * @note This name appears in log files and status messages to identify the GPS type
     */
    const char *name() const override { return "SITL"; }

private:

    /**
     * @brief Timestamp of last GPS data update in milliseconds
     * 
     * @details Used to implement GPS update rate limiting. GPS data is only generated
     *          when sufficient time has elapsed since the last update, based on the
     *          configured SIM_GPS_HERTZ parameter. This simulates the realistic timing
     *          behavior of physical GPS receivers which provide updates at fixed rates
     *          (typically 5-10 Hz for consumer GPS, up to 20 Hz for high-performance units).
     * 
     * @note Time is in milliseconds since system boot (from AP_HAL::millis())
     */
    uint32_t last_update_ms;
};

#endif  // AP_SIM_GPS_ENABLED
