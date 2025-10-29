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
/*
 Original C Code by Marvelmind (https://github.com/MarvelmindRobotics/marvelmind.c)
 Adapted into Ardupilot by Karthik Desai, Amilcar Lucas
 April 2017
 */

/**
 * @file AP_Beacon_Marvelmind.h
 * @brief Marvelmind indoor positioning beacon system integration for ArduPilot
 * 
 * @details This file implements support for the Marvelmind Indoor "GPS" positioning system,
 *          which provides precise indoor localization through ultrasonic beacons. The system
 *          consists of stationary beacons (anchors) and a mobile beacon (hedge) mounted on
 *          the vehicle. Position data is transmitted via serial protocol.
 *          
 *          Protocol characteristics:
 *          - Serial communication with header bytes (0xFF, 0x47)
 *          - Modbus CRC validation for message integrity
 *          - Support for both low-resolution (centimeter) and high-resolution (millimeter) datagrams
 *          - Finite-state parser implementation for robust datagram decoding
 *          
 *          Coordinate system handling:
 *          - Marvelmind natively uses ENU (East-North-Up) coordinate frame
 *          - ArduPilot uses NED (North-East-Down) coordinate frame
 *          - This driver performs automatic ENU→NED transformation
 *          
 *          Unit conversions:
 *          - Protocol transmits positions in millimeters (high-res) or centimeters (low-res)
 *          - Driver converts to meters for ArduPilot's internal representation
 * 
 * @note Protocol specification and reference implementation available at:
 *       https://github.com/MarvelmindRobotics/marvelmind.c
 * 
 * @warning Coordinate frame conversion (ENU→NED) is critical for correct vehicle positioning.
 *          Incorrect transformation will cause navigation failures.
 * 
 * Source: libraries/AP_Beacon/AP_Beacon_Marvelmind.h
 */

#pragma once

#include "AP_Beacon_Backend.h"

#if AP_BEACON_MARVELMIND_ENABLED

/// Maximum size of Marvelmind protocol input buffer in bytes
#define AP_BEACON_MARVELMIND_BUF_SIZE 255

/**
 * @class AP_Beacon_Marvelmind
 * @brief Marvelmind indoor positioning system backend driver
 * 
 * @details This class implements the ArduPilot backend for Marvelmind's ultrasonic
 *          indoor positioning system. It handles serial communication with the Marvelmind
 *          mobile beacon (hedge), parses position datagrams, manages stationary beacon
 *          (anchor) positions, and provides vehicle position in ArduPilot's NED frame.
 *          
 *          Protocol Implementation:
 *          The driver implements a finite-state parser with two main states:
 *          - RECV_HDR: Searching for packet header (0xFF, 0x47)
 *          - RECV_DGRAM: Receiving and validating datagram payload
 *          
 *          Supported datagram types:
 *          - Position datagrams (low-resolution, centimeter precision)
 *          - Position datagrams (high-resolution, millimeter precision)
 *          - Stationary beacon position datagrams
 *          - Beacon distance datagrams
 *          
 *          Coordinate Transformation:
 *          Marvelmind provides data in ENU (East-North-Up) frame:
 *          - X-axis: East direction
 *          - Y-axis: North direction
 *          - Z-axis: Up direction
 *          
 *          ArduPilot requires NED (North-East-Down) frame:
 *          - X-axis: North direction
 *          - Y-axis: East direction
 *          - Z-axis: Down direction
 *          
 *          Transformation applied: NED.x = ENU.y, NED.y = ENU.x, NED.z = -ENU.z
 *          
 *          Unit Conversions:
 *          - Input: millimeters (high-res) or centimeters (low-res)
 *          - Internal: meters (ArduPilot standard)
 *          - Conversion factors applied during datagram processing
 *          
 *          Beacon Allocation:
 *          - Supports up to AP_BEACON_MAX_BEACONS stationary beacons
 *          - Dynamic allocation as beacons are discovered in datagrams
 *          - Beacon addressing by unique 8-bit address field
 *          
 *          Message Integrity:
 *          - All datagrams validated using Modbus CRC-16
 *          - Invalid CRC causes datagram rejection
 *          - CRC computed via calc_crc_modbus() function
 * 
 * @note This implementation is based on Marvelmind's reference C code, adapted
 *       for ArduPilot's architecture and coordinate conventions.
 * 
 * @warning Coordinate system conversion is critical. Verify ENU→NED transformation
 *          is functioning correctly during initial testing to prevent navigation errors.
 */
class AP_Beacon_Marvelmind : public AP_Beacon_Backend
{
public:

    // constructor
    using AP_Beacon_Backend::AP_Beacon_Backend;

    /**
     * @brief Check if the Marvelmind beacon system is healthy and providing data
     * 
     * @details Determines sensor health by checking if position data has been
     *          received recently. The sensor is considered healthy if valid
     *          datagrams have been received and parsed within the expected
     *          time window.
     * 
     * @return true if sensor is receiving valid position data, false otherwise
     * 
     * @note This is called by the AP_Beacon frontend to determine overall system health
     */
    bool healthy() override;

    /**
     * @brief Update beacon system state by reading and parsing serial data
     * 
     * @details Called periodically by the scheduler to:
     *          - Read available bytes from the serial port
     *          - Feed bytes into the finite-state parser
     *          - Process complete datagrams (position, beacon positions, distances)
     *          - Transform coordinates from ENU to NED frame
     *          - Convert units from mm/cm to meters
     *          - Update vehicle position and beacon positions in the frontend
     *          
     *          The update process maintains parse state across calls, accumulating
     *          bytes until complete datagrams are received and validated.
     *          
     *          Parser state machine:
     *          - RECV_HDR: Search for header bytes 0xFF 0x47
     *          - RECV_DGRAM: Accumulate payload bytes, validate CRC, process datagram
     * 
     * @note Runs at the scheduler's rate for this backend (typically 10-50 Hz)
     * @note Invalid datagrams (CRC failures) are silently discarded
     */
    void update() override;

private:
    // Variables for Marvelmind
    
    /**
     * @struct PositionValue
     * @brief Vehicle (mobile hedge) position from Marvelmind system
     * 
     * @details Stores the current position of the mobile beacon (hedge) mounted on
     *          the vehicle. Position values are in Marvelmind's native ENU coordinate
     *          frame and must be transformed to NED before use in ArduPilot.
     *          
     *          Units: millimeters (for high-resolution mode) or centimeters (low-resolution)
     *          Coordinate frame: ENU (East-North-Up) - requires transformation to NED
     */
    struct PositionValue
    {
        uint8_t address;              ///< Beacon address identifier (unique ID)
        uint32_t timestamp;            ///< Position timestamp in Marvelmind system time (milliseconds)
        int32_t x__mm, y__mm, z__mm;  ///< Position in ENU frame: x=East, y=North, z=Up (mm or cm depending on mode)
        bool high_resolution;          ///< True if millimeter precision, false if centimeter precision
    };

    /**
     * @struct StationaryBeaconPosition
     * @brief Stationary anchor beacon position and distance
     * 
     * @details Stores position information for a single stationary beacon (anchor)
     *          in the Marvelmind positioning system. Multiple anchors form the
     *          reference coordinate system for vehicle localization.
     *          
     *          Units: 
     *          - Position: millimeters (high-res) or centimeters (low-res)
     *          - Distance: meters (already converted)
     *          
     *          Coordinate frame: ENU (East-North-Up) - requires transformation to NED
     */
    struct StationaryBeaconPosition
    {
        uint8_t address;              ///< Beacon address identifier (unique ID)
        int32_t x__mm, y__mm, z__mm;  ///< Beacon position in ENU frame: x=East, y=North, z=Up (mm or cm)
        bool high_resolution;          ///< True if millimeter precision, false if centimeter precision
        float distance__m;             ///< Distance between this beacon and the hedge (meters)
    };

    /**
     * @struct StationaryBeaconsPositions
     * @brief Collection of all stationary beacon positions
     * 
     * @details Maintains an array of all discovered stationary beacons in the
     *          Marvelmind system. Beacons are dynamically allocated as they are
     *          discovered in position datagrams.
     *          
     *          Beacon allocation strategy:
     *          - Beacons added as discovered (up to AP_BEACON_MAX_BEACONS limit)
     *          - Identified by unique address field
     *          - Positions updated when new beacon position datagrams received
     */
    struct StationaryBeaconsPositions
    {
        uint8_t num_beacons;                              ///< Number of beacons currently tracked (≤ AP_BEACON_MAX_BEACONS)
        StationaryBeaconPosition beacons[AP_BEACON_MAX_BEACONS]; ///< Array of beacon positions
        bool updated;                                     ///< Flag indicating new beacon data received
    };

    /**
     * @struct MarvelmindHedge
     * @brief Complete Marvelmind system state (vehicle and beacons)
     * 
     * @details Top-level data structure containing all position information from
     *          the Marvelmind system, including the mobile hedge position and all
     *          stationary beacon positions.
     */
    struct MarvelmindHedge
    {
        StationaryBeaconsPositions positions_beacons; ///< All stationary beacon positions
        PositionValue cur_position;                    ///< Current vehicle (hedge) position
        bool _have_new_values;                         ///< Flag indicating new position data available
    };

    /**
     * @brief Finite-state parser states for Marvelmind protocol
     * 
     * @details The protocol parser operates as a state machine with two states:
     *          
     *          - RECV_HDR (Receive Header):
     *            Searching for packet start sequence (0xFF, 0x47)
     *            Bytes scanned until header detected
     *            Transitions to RECV_DGRAM when header found
     *          
     *          - RECV_DGRAM (Receive Datagram):
     *            Accumulating payload bytes
     *            Length determined from datagram header
     *            CRC validation via calc_crc_modbus()
     *            Transitions back to RECV_HDR after datagram complete
     *            
     *          This state machine provides robust parsing even with corrupted
     *          or incomplete data streams.
     */
    enum {
        RECV_HDR,    ///< Searching for datagram header bytes (0xFF, 0x47)
        RECV_DGRAM   ///< Receiving datagram payload and validating CRC
    } parse_state = RECV_HDR; ///< Current state of receive data parser

    MarvelmindHedge hedge;                                 ///< Complete Marvelmind system state (hedge and beacons)
    uint8_t input_buffer[AP_BEACON_MARVELMIND_BUF_SIZE];  ///< Serial input buffer for accumulating datagram bytes
    uint16_t num_bytes_in_block_received;                  ///< Number of bytes received in current datagram block
    uint16_t data_id;                                      ///< Datagram type identifier from protocol header

    /**
     * @brief Find or allocate a beacon slot by address
     * 
     * @details Searches for an existing beacon with the given address, or allocates
     *          a new beacon slot if not found and space is available. This implements
     *          the dynamic beacon discovery mechanism.
     *          
     *          Beacon allocation strategy:
     *          - Search existing beacons array for matching address
     *          - If found, return pointer to existing beacon
     *          - If not found and space available, allocate new slot
     *          - If array full (num_beacons == AP_BEACON_MAX_BEACONS), return nullptr
     * 
     * @param[in] address Unique beacon address identifier (0-255)
     * @return Pointer to beacon structure, or nullptr if allocation failed
     */
    StationaryBeaconPosition* get_or_alloc_beacon(uint8_t address);
    
    /**
     * @brief Process low-resolution stationary beacon position datagram
     * 
     * @details Parses datagram containing beacon positions in centimeter precision.
     *          Updates beacon positions in the hedge.positions_beacons structure.
     *          
     *          Unit conversion: centimeters → millimeters (internal representation)
     *          Coordinate frame: ENU (as received from Marvelmind)
     */
    void process_beacons_positions_datagram();
    
    /**
     * @brief Process high-resolution stationary beacon position datagram
     * 
     * @details Parses datagram containing beacon positions in millimeter precision.
     *          Updates beacon positions in the hedge.positions_beacons structure.
     *          
     *          Unit: millimeters (no conversion needed)
     *          Coordinate frame: ENU (as received from Marvelmind)
     */
    void process_beacons_positions_highres_datagram();
    
    /**
     * @brief Process high-resolution vehicle position datagram
     * 
     * @details Parses datagram containing hedge (vehicle) position in millimeter
     *          precision. Updates hedge.cur_position structure.
     *          
     *          Unit: millimeters
     *          Coordinate frame: ENU (transformation to NED performed later)
     */
    void process_position_highres_datagram();
    
    /**
     * @brief Process low-resolution vehicle position datagram
     * 
     * @details Parses datagram containing hedge (vehicle) position in centimeter
     *          precision. Updates hedge.cur_position structure.
     *          
     *          Unit conversion: centimeters → millimeters (internal representation)
     *          Coordinate frame: ENU (transformation to NED performed later)
     */
    void process_position_datagram();
    
    /**
     * @brief Process beacon distance datagram
     * 
     * @details Parses datagram containing distances from hedge to each stationary
     *          beacon. Updates distance__m field in beacon structures.
     *          
     *          Unit: meters (as transmitted in protocol)
     */
    void process_beacons_distances_datagram();
    
    /**
     * @brief Update ArduPilot frontend with stationary beacon positions
     * 
     * @details Transforms beacon positions from Marvelmind's ENU frame to ArduPilot's
     *          NED frame, converts units from millimeters to meters, and updates the
     *          AP_Beacon frontend with the transformed positions.
     *          
     *          Coordinate transformation applied: NED.x = ENU.y, NED.y = ENU.x, NED.z = -ENU.z
     *          Unit conversion: millimeters → meters (divide by 1000)
     */
    void set_stationary_beacons_positions();
    
    /**
     * @brief Order beacons by address for consistent indexing
     * 
     * @details Sorts the beacons array to ensure consistent beacon ordering across
     *          updates. This maintains stable beacon instance assignments.
     */
    void order_stationary_beacons();
    
    /**
     * @brief Find beacon instance index by address
     * 
     * @details Searches the beacons array for a beacon with the specified address
     *          and returns its index in the array.
     * 
     * @param[in] address Beacon address to search for
     * @return Beacon index (0 to num_beacons-1), or -1 if not found
     */
    int8_t find_beacon_instance(uint8_t address) const;

    // Variables for ArduPilot integration
    
    uint32_t last_update_ms;  ///< Timestamp of last successful position update (milliseconds, AP_HAL::millis())

    /**
     * @brief Cached vehicle position in NED coordinates
     * 
     * @details Vehicle position after ENU→NED transformation and unit conversion.
     *          This is the position reported to the AP_Beacon frontend.
     *          
     *          Units: meters
     *          Coordinate frame: NED (North-East-Down)
     *          Origin: Marvelmind system origin (typically one of the stationary beacons)
     */
    Vector3f vehicle_position_NED__m;
    
    bool vehicle_position_initialized;  ///< Flag indicating vehicle position has been received at least once

    /**
     * @brief Cached beacon positions in NED coordinates
     * 
     * @details Array of stationary beacon positions after ENU→NED transformation
     *          and unit conversion. These are the positions reported to the
     *          AP_Beacon frontend.
     *          
     *          Units: meters
     *          Coordinate frame: NED (North-East-Down)
     *          Array size: AP_BEACON_MAX_BEACONS
     *          Valid entries: 0 to num_beacons-1
     */
    Vector3f beacon_position_NED__m[AP_BEACON_MAX_BEACONS];
    
    bool beacon_position_initialized;  ///< Flag indicating beacon positions have been received at least once
};

#endif  // AP_BEACON_MARVELMIND_ENABLED
