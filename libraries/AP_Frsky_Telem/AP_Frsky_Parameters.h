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
 * @file AP_Frsky_Parameters.h
 * @brief FrSky telemetry parameter management
 * 
 * @details This file defines the parameter structure for configuring FrSky telemetry
 *          sensor IDs and options. FrSky telemetry uses sensor IDs to identify
 *          different data streams on the Smart Port bus, allowing multiple sensors
 *          to share the same physical connection while maintaining separate logical
 *          channels for different telemetry data types.
 */

#pragma once

#include "AP_Frsky_Telem.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

class AP_Frsky_Telem;

/**
 * @class AP_Frsky_Parameters
 * @brief Configuration parameters for FrSky telemetry sensor IDs and options
 * 
 * @details This class manages the configurable parameters that control FrSky telemetry
 *          behavior, including sensor IDs for downlink and uplink communication streams,
 *          and option flags for enabling various telemetry features.
 *          
 *          FrSky telemetry protocols (D, X, and passthrough) use sensor IDs to multiplex
 *          different data types onto a single Smart Port bus. Each sensor ID represents
 *          a logical channel that can carry specific telemetry information. Multiple
 *          sensor IDs can be configured for simultaneous transmission of different
 *          data streams.
 *          
 *          Downlink IDs are used for sending telemetry data from the vehicle to the
 *          transmitter. When bidirectional telemetry is enabled, uplink IDs allow
 *          receiving commands from the transmitter back to the vehicle.
 * 
 * @note Uplink and additional downlink sensor IDs are only available when
 *       HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL is defined at compile time.
 * 
 * @warning Sensor IDs must not conflict with other FrSky sensors on the same Smart Port
 *          bus. Using duplicate IDs will cause telemetry data corruption and unreliable
 *          communication. Consult FrSky sensor ID documentation to avoid conflicts.
 * 
 * @see AP_Frsky_Telem.h for the main telemetry implementation
 * @see AP_Param documentation for parameter storage and retrieval mechanisms
 * @see AP_Frsky_SPort_Passthrough for passthrough protocol implementation using these parameters
 */
class AP_Frsky_Parameters
{
    /**
     * @brief Friend class declaration for Smart Port Passthrough protocol access
     * @details AP_Frsky_SPort_Passthrough requires direct access to private parameter
     *          members for efficient telemetry packet generation without getter overhead.
     */
    friend class AP_Frsky_SPort_Passthrough;
    
public:
    /**
     * @brief Constructor for FrSky parameters
     * @details Initializes the parameter object. Actual parameter values are loaded
     *          from persistent storage via the AP_Param system during initialization.
     */
    AP_Frsky_Parameters();

    /**
     * @brief AP_Param::GroupInfo array for parameter registration
     * @details Static table defining all FrSky telemetry parameters that can be
     *          configured by users through ground control stations or parameter files.
     *          This table registers parameters with the AP_Param system, enabling
     *          persistent storage, runtime modification, and automatic documentation
     *          generation for ground station interfaces.
     */
    static const struct AP_Param::GroupInfo var_info[];

private:
    /**
     * @brief Primary downlink sensor ID for telemetry transmission
     * @details Sensor ID used for the primary downlink telemetry stream from vehicle
     *          to transmitter. Default value varies by protocol variant:
     *          - FrSky D protocol: typically 0x1B
     *          - FrSky X protocol: typically 0x1B
     *          - Passthrough protocol: configurable based on data type
     *          
     *          This ID identifies the vehicle's main telemetry data on the Smart Port
     *          bus, allowing the receiver to distinguish this data from other sensors.
     * 
     * @note Parameter type: AP_Int8
     * @see FrSky Smart Port protocol specification for valid sensor ID ranges
     */
    AP_Int8 _dnlink_id;
    
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    /**
     * @brief Uplink sensor ID for bidirectional command reception
     * @details Sensor ID used for receiving commands from the transmitter to the vehicle
     *          when bidirectional telemetry is enabled. Allows the transmitter to send
     *          control commands, parameter changes, or mission items back to the autopilot
     *          over the same Smart Port connection used for telemetry downlink.
     *          
     *          Only available when HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL is enabled at
     *          compile time.
     * 
     * @note Parameter type: AP_Int8
     * @warning Must use a different sensor ID than downlink IDs to avoid bus conflicts
     */
    AP_Int8 _uplink_id;
    
    /**
     * @brief First additional downlink sensor ID for multi-stream telemetry
     * @details Allows configuration of a second telemetry downlink stream with a
     *          different sensor ID. This enables simultaneous transmission of multiple
     *          telemetry data types or redundant data streams on the same Smart Port bus.
     *          
     *          Only available when HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL is enabled at
     *          compile time.
     * 
     * @note Parameter type: AP_Int8
     * @warning All sensor IDs (_dnlink_id, _dnlink1_id, _dnlink2_id, _uplink_id) must
     *          be unique to prevent telemetry data corruption
     */
    AP_Int8 _dnlink1_id;
    
    /**
     * @brief Second additional downlink sensor ID for multi-stream telemetry
     * @details Allows configuration of a third telemetry downlink stream with a
     *          different sensor ID. Provides additional capacity for complex telemetry
     *          configurations requiring multiple simultaneous data streams.
     *          
     *          Only available when HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL is enabled at
     *          compile time.
     * 
     * @note Parameter type: AP_Int8
     * @warning All sensor IDs must be unique across all FrSky sensors on the Smart Port
     */
    AP_Int8 _dnlink2_id;
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    
    /**
     * @brief Telemetry options bitmask for feature configuration
     * @details Bit flags controlling optional FrSky telemetry features and behaviors.
     *          Each bit in this mask enables or disables a specific telemetry option:
     *          - EnableAirspeedAndGroundspeed: Include both airspeed and groundspeed
     *            in telemetry streams instead of just groundspeed
     *          - Additional option bits may be defined for future features
     *          
     *          Multiple options can be enabled simultaneously by setting multiple bits.
     *          Consult AP_Frsky_Telem implementation for current option definitions.
     * 
     * @note Parameter type: AP_Int8 (8-bit bitmask)
     * @see AP_Frsky_Telem.h for option bit definitions and descriptions
     */
    AP_Int8 _options;
};
