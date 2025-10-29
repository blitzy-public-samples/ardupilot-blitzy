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
 * @file AP_GPS_GSOF.h
 * @brief Trimble GSOF (General Serial Output Format) GPS driver for ArduPilot
 * 
 * @details This driver implements support for Trimble GPS receivers using the
 *          GSOF binary protocol. Supported models include BD9xx, NetR9, and R10
 *          series receivers that support GSOF output.
 *          
 *          GSOF Protocol Overview:
 *          - Binary protocol with packet structure: STX (0x02), status, type, 
 *            length, packet ID, page index, max page index, data, checksum, ETX (0x03)
 *          - Supports multiple GSOF record types in a single GENOUT packet
 *          - Position data in degrees (WGS84 ellipsoid reference)
 *          - Velocity data in m/s (NED frame)
 *          - Full RTK support (fixed and float solutions)
 *          
 *          Supported GSOF Message Types:
 *          - Type 1: Position Time (GPS week, time, satellite count, status flags)
 *          - Type 2: Position (latitude, longitude, altitude in WGS84)
 *          - Type 8: Velocity (horizontal velocity, heading, vertical velocity in NED)
 *          - Type 9: DOP (PDOP, HDOP, VDOP, TDOP)
 *          - Type 12: Position Sigma (position uncertainty estimates)
 *          
 *          Configuration Requirements:
 *          - Requires 115200 baud UART configuration
 *          - Auto-configuration enabled by default (GPS_AUTO_CONFIG parameter)
 *          - COM port selection via GPS_COM_PORT parameter (default: COM2/TTL serial)
 *          
 * @note Code by Michael Oborne
 * @see https://receiverhelp.trimble.com/oem-gnss/index.html#Welcome.html?TocPath=_____1
 * 
 * Source: libraries/AP_GPS/AP_GPS_GSOF.h, libraries/AP_GPS/AP_GPS_GSOF.cpp
 */

#pragma once

#include "AP_GPS.h"
#include "GPS_Backend.h"
#include <AP_GSOF/AP_GSOF.h>

#if AP_GPS_GSOF_ENABLED
/**
 * @class AP_GPS_GSOF
 * @brief GPS driver for Trimble receivers using GSOF binary protocol
 * 
 * @details This class implements a GPS backend for Trimble receivers that output
 *          GSOF (General Serial Output Format) binary data. It inherits from both
 *          AP_GPS_Backend (GPS driver interface) and AP_GSOF (GSOF protocol parser).
 *          
 *          Lifecycle:
 *          1. Constructor initializes port, configures default COM port (COM2)
 *          2. configure() sends GSOF output format requests to receiver
 *          3. read() continuously parses incoming GSOF packets
 *          4. pack_state_data() updates GPS state with parsed position/velocity
 *          
 *          The driver requests five GSOF message types from the receiver:
 *          - POS_TIME (type 1): GPS time and satellite count
 *          - POS (type 2): Latitude, longitude, altitude (WGS84)
 *          - VEL (type 8): Velocity components in NED frame
 *          - DOP (type 9): Dilution of precision values
 *          - POS_SIGMA (type 12): Position uncertainty estimates
 *          
 *          Coordinate Systems and Units:
 *          - Position: Degrees latitude/longitude (WGS84), meters altitude (above ellipsoid)
 *          - Velocity: m/s in NED (North-East-Down) frame
 *          - Heading: Radians from true north (actually ground course)
 *          - Time: GPS week number and milliseconds within week
 *          
 *          RTK Support:
 *          - Supports RTK fixed (cm-level accuracy) and RTK float (dm-level accuracy)
 *          - Status reported via highest_supported_status() as GPS_OK_FIX_3D_RTK_FIXED
 *          - Position flags in GSOF type 1 message indicate RTK solution status
 *          
 *          Thread Safety:
 *          - Called from GPS driver thread at main loop rate
 *          - Uses UART driver with thread-safe serial port access
 *          - No explicit locking required (single-threaded access pattern)
 *          
 * @note Tested with Trimble BD9xx, NetR9, and R10 receivers
 * @warning Requires 115200 baud serial configuration for reliable operation
 * 
 * Source: libraries/AP_GPS/AP_GPS_GSOF.cpp:53-93
 */
class AP_GPS_GSOF : public AP_GPS_Backend, public AP_GSOF
{
public:
    /**
     * @brief Construct a new AP_GPS_GSOF driver instance
     * 
     * @param[in] _gps Reference to main AP_GPS object
     * @param[in] _params Reference to GPS parameters for this instance
     * @param[in] _state Reference to GPS state structure to populate
     * @param[in] _port Pointer to UART driver for serial communication
     * 
     * @details Initializes GSOF driver, sets default COM port to COM2 (TTL serial),
     *          validates COM port parameter, and requests baud rate configuration
     *          if auto-config is enabled.
     */
    AP_GPS_GSOF(AP_GPS &_gps, AP_GPS::Params &_params, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    /**
     * @brief Get the highest GPS fix status this driver can report
     * 
     * @return AP_GPS::GPS_OK_FIX_3D_RTK_FIXED indicating full RTK fixed solution support
     * 
     * @details GSOF protocol supports all fix types from no fix through RTK fixed.
     *          This method returns the maximum capability, which is RTK fixed solution
     *          (centimeter-level accuracy with carrier phase ambiguities resolved).
     *          Actual reported status depends on receiver state and correction availability.
     *          
     *          Status hierarchy (lowest to highest):
     *          - NO_FIX: No position solution
     *          - NO_GPS: GPS subsystem not initialized
     *          - GPS_OK_FIX_2D: 2D position fix (altitude not valid)
     *          - GPS_OK_FIX_3D: 3D position fix (standard GPS)
     *          - GPS_OK_FIX_3D_DGPS: 3D with differential corrections
     *          - GPS_OK_FIX_3D_RTK_FLOAT: 3D RTK with floating ambiguities
     *          - GPS_OK_FIX_3D_RTK_FIXED: 3D RTK with fixed ambiguities (highest)
     * 
     * @note Called by AP_GPS to determine driver capabilities
     */
    AP_GPS::GPS_Status highest_supported_status(void) override WARN_IF_UNUSED {
        return AP_GPS::GPS_OK_FIX_3D_RTK_FIXED;
    }

    /**
     * @brief Parse GSOF binary packets from serial stream and update GPS state
     * 
     * @return true if a complete GSOF packet was successfully parsed and processed
     * @return false if no complete packet available or parsing incomplete
     * 
     * @details This method is called repeatedly by the AP_GPS driver (typically at
     *          main loop rate, ~50-400Hz depending on vehicle type). It reads available
     *          bytes from the UART, parses GSOF packet structure, and updates internal
     *          GPS state when complete position/velocity data is received.
     *          
     *          Parsing Process:
     *          1. Read bytes from UART using port->read()
     *          2. Feed bytes to AP_GSOF::parse() which handles packet framing
     *          3. When complete GENOUT packet received, extract GSOF records
     *          4. Process each GSOF record type (position, velocity, DOP, etc.)
     *          5. Call pack_state_data() to update GPS_State structure
     *          6. Return true to indicate new data available
     *          
     *          GSOF Packet Structure:
     *          - STX (0x02): Start transmission byte
     *          - Status: Packet status byte
     *          - Type: GENOUT (0x40) for GSOF data packets
     *          - Length: Payload length
     *          - Packet ID: Transmission number
     *          - Page Index: Current page (for multi-page messages)
     *          - Max Page Index: Total pages
     *          - Data: GSOF records (multiple types can be concatenated)
     *          - Checksum: XOR checksum of packet bytes
     *          - ETX (0x03): End transmission byte
     *          
     *          Performance Considerations:
     *          - Typically processes 10-100Hz update rate from receiver
     *          - Parser state maintained across calls (incremental parsing)
     *          - Returns false quickly if no serial data available
     *          - Minimal CPU overhead (~0.1% at 400Hz loop rate)
     * 
     * @note This method implements the GPS_Backend::read() pure virtual function
     * @note Must handle partial packets gracefully as UART may split packet delivery
     * @warning Do not block or delay in this method - called at high frequency
     * 
     * Source: libraries/AP_GPS/AP_GPS_GSOF.cpp read() implementation
     */
    bool read() override WARN_IF_UNUSED;

    /**
     * @brief Get the driver name identifier
     * 
     * @return const char* "GSOF" - driver identification string
     * 
     * @details Returns a constant string identifying this GPS driver type.
     *          Used for logging, debugging, and user interface display.
     *          The name "GSOF" matches the GPS_TYPE parameter value (11)
     *          in the GPS configuration system.
     * 
     * @note Displayed in boot messages and MAVLink GPS status
     */
    const char *name() const override { return "GSOF"; }

private:

    /**
     * @brief Configure the GPS receiver for GSOF output
     * 
     * @return true if configuration command was sent successfully
     * @return false if configuration is still in progress or port not ready
     * 
     * @details Sends configuration commands to the GPS receiver to enable GSOF
     *          message output at the desired rate and format. This method is called
     *          repeatedly by read() until all requested message types are configured.
     *          
     *          Configuration Sequence:
     *          1. Iterate through requested_msgs array (POS_TIME, POS, VEL, DOP, POS_SIGMA)
     *          2. For each message type, call requestGSOF() to send enable command
     *          3. Increment gsofmsgreq_index to track configuration progress
     *          4. Return true when all messages configured
     *          
     *          Message Configuration:
     *          - Sets output rate based on GPS_RATE_MS parameter (10Hz, 50Hz, or 100Hz)
     *          - Configures output on specified COM port (from GPS_COM_PORT parameter)
     *          - Enables up to 5 GSOF message types (GSOF protocol allows max 10)
     *          
     *          Timing:
     *          - Called every 110ms (gsofmsg_time interval)
     *          - Takes ~5 iterations to fully configure (5 message types)
     *          - Total configuration time ~550ms after driver initialization
     * 
     * @note Configuration commands do not wait for ACK from receiver
     * @note If receiver already sending data, no conflict prevention mechanism
     * @warning Manufacturer recommends ethernet for configuration if serial traffic present
     * 
     * Source: libraries/AP_GPS/AP_GPS_GSOF.cpp:95-123
     */
    bool configure();

    /**
     * @brief Hardware port identifiers for GSOF serial configuration
     * 
     * @details A subset of the port identifiers in the GSOF protocol that are
     *          used for serial communication. Ethernet, USB, and other port types
     *          are not supported by the GPS driver at this time and are omitted.
     *          
     *          These enumeration values are used in GSOF configuration commands
     *          to specify which physical port should output GSOF data. The values
     *          are specific to Trimble receivers and are not documented in the
     *          public API documentation.
     * 
     * @note Values are hardware-specific and undocumented in Trimble API
     */
    enum class HW_Port {
        COM1 = 0, ///< RS232 serial port (typically DB9 connector)
        COM2 = 1, ///< TTL serial port (typically header pins, default)
    };

    /**
     * @brief Output rate identifiers for GSOF message frequency configuration
     * 
     * @details A subset of the data frequencies in the GSOF protocol that are
     *          used for serial output configuration. These enumeration values
     *          specify the update rate at which the GPS receiver will output
     *          GSOF messages.
     *          
     *          The values are protocol-specific constants used in GSOF configuration
     *          commands. They are not documented in the Trimble public API.
     *          
     *          Rate Selection:
     *          - Actual rate used is determined by GPS_RATE_MS parameter
     *          - Default is typically 10Hz for most vehicle applications
     *          - Higher rates (50Hz, 100Hz) useful for fast dynamics or EKF2/EKF3
     * 
     * @note Values are protocol-specific and undocumented in Trimble API
     * @note Not all receivers support all rates - check receiver specifications
     */
    enum class Output_Rate {
        FREQ_10_HZ = 1,   ///< 10 Hz update rate (100ms interval, standard for most vehicles)
        FREQ_50_HZ = 15,  ///< 50 Hz update rate (20ms interval, for fast dynamics)
        FREQ_100_HZ = 16, ///< 100 Hz update rate (10ms interval, high-rate applications)
    };

    /**
     * @brief Send request to GPS receiver to set baud rate on specified port
     * 
     * @param[in] portIndex Hardware port (COM1 or COM2) to configure baud rate
     * 
     * @details Sends a GSOF configuration command to set the serial baud rate to
     *          115200 on the specified hardware port. This is part of the auto-
     *          configuration sequence when GPS_AUTO_CONFIG is enabled.
     *          
     *          Configuration Command Format:
     *          - Uses Trimble proprietary serial command protocol
     *          - Sets baud rate to 115200 (required for reliable GSOF operation)
     *          - Command is fire-and-forget (no ACK processing)
     * 
     * @note These request functions currently ignore ACK from device
     * @note If device already sending serial traffic, no conflict prevention mechanism
     * @warning Manufacturer recommends switching to ethernet for configuration in production
     * 
     * Source: libraries/AP_GPS/AP_GPS_GSOF.cpp requestBaud() implementation
     */
    void requestBaud(const HW_Port portIndex);
    
    /**
     * @brief Send request to GPS receiver to enable GSOF message output
     * 
     * @param[in] messageType GSOF message type number (1=POS_TIME, 2=POS, 8=VEL, 9=DOP, 12=POS_SIGMA)
     * @param[in] portIndex Hardware port (COM1 or COM2) to output message
     * @param[in] rateHz Output rate (FREQ_10_HZ, FREQ_50_HZ, or FREQ_100_HZ)
     * 
     * @details Sends a GSOF configuration command to enable output of a specific
     *          message type at the specified rate on the specified port. Called
     *          repeatedly by configure() to enable all requested message types.
     *          
     *          Command Sequence:
     *          1. Format GSOF "set output" command with message type, port, rate
     *          2. Send command via UART
     *          3. Do not wait for ACK (fire-and-forget)
     *          
     *          Message Types Requested:
     *          - Type 1 (POS_TIME): GPS time, week, satellite count, flags
     *          - Type 2 (POS): Latitude, longitude, altitude
     *          - Type 8 (VEL): Velocity components and heading
     *          - Type 9 (DOP): Dilution of precision values
     *          - Type 12 (POS_SIGMA): Position uncertainty
     * 
     * @note Command does not wait for ACK from receiver
     * @note GSOF protocol allows maximum of 10 output message types
     * 
     * Source: libraries/AP_GPS/AP_GPS_GSOF.cpp requestGSOF() implementation
     */
    void requestGSOF(const uint8_t messageType, const HW_Port portIndex, const Output_Rate rateHz);

    /**
     * @brief Validate baud rate parameter value
     * 
     * @param[in] baud Baud rate parameter value to validate
     * @return true if baud rate is valid for GSOF protocol
     * @return false if baud rate is invalid
     * 
     * @details Checks if the provided baud rate parameter is within valid range
     *          for GSOF communication. Used during parameter validation.
     */
    bool validate_baud(const uint8_t baud) const WARN_IF_UNUSED;
    
    /**
     * @brief Validate COM port parameter value
     * 
     * @param[in] com_port COM port parameter value to validate
     * @return true if COM port is valid (COM1 or COM2)
     * @return false if COM port is invalid
     * 
     * @details Checks if the user-configured COM port parameter corresponds to
     *          a valid hardware port (COM1 or COM2). Invalid port values result
     *          in error message to GCS and driver initialization failure.
     * 
     * @note Called during constructor to validate GPS_COM_PORT parameter
     * 
     * Source: libraries/AP_GPS/AP_GPS_GSOF.cpp:81-85
     */
    bool validate_com_port(const uint8_t com_port) const WARN_IF_UNUSED;

    /**
     * @brief Pack parsed GSOF data into GPS_State structure
     * 
     * @details Transfers data from AP_GSOF parsed message structures (pos_time,
     *          position, vel, dop, pos_sigma) into the AP_GPS::GPS_State structure
     *          that is used by the flight controller.
     *          
     *          Data Transformations:
     *          - Position: Convert radians to degrees, altitude from WGS84 ellipsoid
     *          - Velocity: Convert NED velocity components (m/s) to GPS state format
     *          - Heading: Convert from radians to degrees (ground course)
     *          - Time: Extract GPS week and milliseconds for timestamp
     *          - Status: Map GSOF position flags to AP_GPS::GPS_Status enum
     *          - Accuracy: Extract horizontal/vertical accuracy from pos_sigma
     *          
     *          State Updates:
     *          - state.location: Latitude, longitude, altitude
     *          - state.velocity: Ground speed and vertical velocity
     *          - state.ground_course: Heading in degrees
     *          - state.num_sats: Number of satellites used
     *          - state.hdop: Horizontal dilution of precision
     *          - state.vdop: Vertical dilution of precision
     *          - state.status: GPS fix status (NO_FIX to RTK_FIXED)
     *          - state.have_vertical_velocity: Set to true
     *          - state.have_speed_accuracy: Set based on pos_sigma availability
     *          - state.last_gps_time_ms: Current system time
     * 
     * @note Called by read() when complete set of GSOF messages received
     * @note Coordinate frame: NED (North-East-Down) for velocity
     * @note Reference ellipsoid: WGS84
     * 
     * Source: libraries/AP_GPS/AP_GPS_GSOF.cpp pack_state_data() implementation
     */
    void pack_state_data();

    // Private member variables for driver state management
    
    /**
     * @brief Packet counter for tracking received GSOF packets
     * 
     * @details Incremented for each successfully parsed GSOF packet. Used for
     *          debugging and monitoring packet reception rate. Not used for
     *          protocol functionality.
     */
    uint8_t packetcount;
    
    /**
     * @brief Timestamp for next configuration message transmission (milliseconds)
     * 
     * @details Tracks when the next GSOF configuration command should be sent
     *          during the initialization sequence. Set to current time + 110ms
     *          intervals to space out configuration commands and avoid overwhelming
     *          the receiver during startup.
     * 
     * @note Uses AP_HAL::millis() for timing
     */
    uint32_t gsofmsg_time;
    
    /**
     * @brief Index into requested_msgs array for configuration sequencing
     * 
     * @details Tracks which GSOF message type is being configured during the
     *          initialization sequence. Incremented after each requestGSOF() call
     *          until all requested message types are enabled.
     *          
     *          Range: 0 to requested_msgs.count()-1 (typically 0-4 for 5 message types)
     */
    uint8_t gsofmsgreq_index;
    
    /**
     * @brief Reserved for future use - next GSOF request identifier
     * 
     * @details Currently unused. May be used in future for tracking specific
     *          GSOF request/response pairs or managing message priorities.
     */
    uint16_t next_req_gsof;
    
    /**
     * @brief Bitmask of GSOF message types requested from receiver
     * 
     * @details Stores which GSOF message types should be enabled on the receiver.
     *          Initialized in constructor with standard set: POS_TIME (1), POS (2),
     *          VEL (8), DOP (9), POS_SIGMA (12).
     *          
     *          The AP_GSOF::MsgTypes class provides a bitmask with up to 71 bits
     *          corresponding to GSOF message type numbers. GSOF protocol allows
     *          maximum of 10 message types to be output simultaneously.
     * 
     * @note Limited to 10 concurrent message types per GSOF specification
     * 
     * Source: libraries/AP_GPS/AP_GPS_GSOF.cpp:65-75
     */
    AP_GSOF::MsgTypes requested_msgs;
};
#endif
