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
 * @file AP_GPS_NOVA.h
 * @brief GPS driver for Novatel/Tersus/ComNav receivers using NOVA binary protocol
 * 
 * @details This driver implements support for GPS receivers using the Novatel OEM
 *          binary protocol. It parses the following binary messages:
 *          - BESTPOS (ID 42): Position solution with WGS84 coordinates
 *          - BESTVEL (ID 99): Velocity solution in NED frame
 *          - PSRDOP (ID 174): Dilution of precision values
 * 
 *          Supported receivers include:
 *          - Novatel OEM615, OEM7, OEM7600, OEM7700
 *          - Tersus receivers (various models)
 *          - ComNav K708, K803
 * 
 *          The protocol provides full RTK support with:
 *          - RTK Fixed solutions (cm-level accuracy)
 *          - RTK Float solutions (dm-level accuracy)
 *          - SBAS corrections
 *          - Differential corrections
 * 
 * @note Protocol uses 3-byte preamble: 0xAA 0x44 0x12
 * 
 * @note Message Structure:
 *       - Preamble: 3 bytes (0xAA 0x44 0x12)
 *       - Header: 28 bytes (message length, ID, port, sequence, time, status)
 *       - Data: Variable length (message-specific payload)
 *       - CRC32: 4 bytes
 * 
 * @note Coordinate Systems:
 *       - Position: WGS84 latitude/longitude in degrees, height above ellipsoid in meters
 *       - Velocity: NED (North-East-Down) frame in m/s
 * 
 * @note Units:
 *       - Position: degrees (lat/lon), meters (height)
 *       - Velocity: m/s
 *       - Standard deviations: meters
 *       - Time: GPS week and time-of-week (milliseconds)
 * 
 * @author Michael Oborne
 * @see https://hexagondownloads.blob.core.windows.net/public/Novatel/assets/Documents/Manuals/om-20000129/om-20000129.pdf
 * 
 * Source: libraries/AP_GPS/AP_GPS_NOVA.h
 */

#pragma once

#include "AP_GPS.h"
#include "GPS_Backend.h"

#if AP_GPS_NOVA_ENABLED
/**
 * @class AP_GPS_NOVA
 * @brief GPS driver for receivers using Novatel OEM binary protocol (NOVA)
 * 
 * @details This class implements a GPS backend driver for receivers that communicate
 *          using the Novatel OEM binary protocol. The driver performs the following:
 *          
 *          Message Processing:
 *          - State machine-based byte parser for robust message reception
 *          - CRC32 validation for data integrity
 *          - Handles BESTPOS (position), BESTVEL (velocity), PSRDOP (DOP) messages
 *          
 *          Position Solution (BESTPOS - Message ID 42):
 *          - WGS84 latitude/longitude in degrees
 *          - Height above ellipsoid in meters
 *          - Position standard deviations (accuracy metrics)
 *          - Solution status and position type (SINGLE, PSRDIFF, RTK_FLOAT, RTK_FIXED, etc.)
 *          - Satellite count and usage information
 *          - Differential correction age
 *          
 *          Velocity Solution (BESTVEL - Message ID 99):
 *          - Horizontal speed and track over ground
 *          - Vertical speed (positive = up in NED frame)
 *          - Solution status and velocity type
 *          - Latency and age information
 *          
 *          Dilution of Precision (PSRDOP - Message ID 174):
 *          - GDOP, PDOP, HDOP, HTDOP, TDOP values
 *          - Satellite count used in DOP calculation
 *          
 *          RTK Support:
 *          - Full support for RTK Fixed (cm-level accuracy)
 *          - RTK Float solutions (dm-level accuracy)
 *          - Reports base station ID and correction age
 *          - Extended solution status for advanced diagnostics
 *          
 *          Initialization:
 *          - Sends configuration commands on startup
 *          - Configures message output rates
 *          - Sets up logging and data formats
 *          
 * @note The protocol uses a 3-byte preamble (0xAA 0x44 0x12) for message synchronization
 * 
 * @note Thread Safety: Called from GPS thread in AP_GPS, uses UART driver for serial I/O
 * 
 * @warning Ensure receiver is configured to output binary BESTPOS, BESTVEL, and PSRDOP messages
 * 
 * Source: libraries/AP_GPS/AP_GPS_NOVA.h, libraries/AP_GPS/AP_GPS_NOVA.cpp
 */
class AP_GPS_NOVA : public AP_GPS_Backend
{
public:
    /**
     * @brief Constructor for NOVA GPS driver
     * 
     * @param[in] _gps Reference to main AP_GPS object
     * @param[in] _params Reference to GPS parameters for this instance
     * @param[in,out] _state Reference to GPS state structure to update
     * @param[in] _port UART driver for serial communication with GPS receiver
     */
    AP_GPS_NOVA(AP_GPS &_gps, AP_GPS::Params &_params, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    /**
     * @brief Report the highest GPS status this driver can achieve
     * 
     * @details NOVA protocol receivers support full RTK fixed solutions, which provide
     *          centimeter-level positioning accuracy when receiving RTK corrections from
     *          a base station. This is the highest accuracy level in the AP_GPS status
     *          hierarchy.
     * 
     * @return AP_GPS::GPS_OK_FIX_3D_RTK_FIXED indicating RTK fixed solution capability
     * 
     * @note Actual status achieved depends on:
     *       - Receiver configuration and capabilities
     *       - Availability of RTK correction stream
     *       - Satellite visibility and geometry
     *       - Base station distance and quality
     */
    AP_GPS::GPS_Status highest_supported_status(void) override { return AP_GPS::GPS_OK_FIX_3D_RTK_FIXED; }

    /**
     * @brief Read and parse data from GPS receiver
     * 
     * @details This method is called repeatedly by the GPS driver framework to process
     *          incoming data from the receiver. It performs the following operations:
     *          
     *          1. Reads available bytes from UART buffer
     *          2. Feeds bytes to state machine parser (parse())
     *          3. Processes complete messages (process_message())
     *          4. Updates GPS state when valid position/velocity received
     *          5. Handles initialization commands if receiver needs configuration
     *          
     *          The method returns true when a complete message has been successfully
     *          processed and GPS state has been updated, indicating new data is available.
     *          
     *          Message Processing Flow:
     *          - BESTPOS: Updates location, altitude, accuracy, satellite counts, fix type
     *          - BESTVEL: Updates velocity in NED frame, ground speed, vertical speed
     *          - PSRDOP: Updates HDOP, VDOP values for position accuracy estimation
     * 
     * @return true if a complete message was processed and GPS state was updated
     * @return false if no complete message available or message parsing incomplete
     * 
     * @note Called at GPS thread rate (typically 5-10 Hz)
     * @note Must not block - reads only available data from UART buffer
     * 
     * Source: libraries/AP_GPS/AP_GPS_NOVA.cpp
     */
    bool read() override;

    /**
     * @brief Get the name identifier for this GPS driver
     * 
     * @return "NOVA" - identifier string for logging and diagnostic purposes
     * 
     * @note This name appears in log messages and status displays
     */
    const char *name() const override { return "NOVA"; }

private:

    /**
     * @brief State machine byte parser for NOVA binary protocol
     * 
     * @details Implements a state machine to parse incoming bytes and extract complete
     *          NOVA binary messages. The parser progresses through the following states:
     *          
     *          1. PREAMBLE1: Wait for first preamble byte (0xAA)
     *          2. PREAMBLE2: Wait for second preamble byte (0x44)
     *          3. PREAMBLE3: Wait for third preamble byte (0x12)
     *          4. HEADERLENGTH: Read header length field
     *          5. HEADERDATA: Accumulate 28-byte header
     *          6. DATA: Accumulate message payload (length from header)
     *          7. CRC1-CRC4: Read 4-byte CRC32 checksum
     *          
     *          State machine provides robust parsing with:
     *          - Preamble synchronization to recover from data errors
     *          - Length validation to prevent buffer overflows
     *          - CRC32 validation for data integrity
     *          - Automatic recovery from corrupted messages
     * 
     * @param[in] temp Single input byte to process
     * 
     * @return true if a complete message has been received and validated (CRC passed)
     * @return false if still accumulating message or if CRC validation failed
     * 
     * @note Called for each byte received from UART
     * @note Increments crc_error_counter on CRC failures for diagnostics
     * 
     * Source: libraries/AP_GPS/AP_GPS_NOVA.cpp
     */
    bool parse(uint8_t temp);
    
    /**
     * @brief Process a complete validated NOVA message
     * 
     * @details After parse() returns true indicating a complete message, this method
     *          processes the message payload based on message ID from the header:
     *          
     *          BESTPOS (ID 42) - Position Solution:
     *          - Extracts WGS84 lat/lon/height
     *          - Converts solution status to AP_GPS fix type
     *          - Updates position accuracy estimates (standard deviations)
     *          - Records satellite counts and usage
     *          - Stores RTK correction age and base station ID
     *          
     *          BESTVEL (ID 99) - Velocity Solution:
     *          - Extracts horizontal speed and track angle
     *          - Extracts vertical speed (NED frame convention: positive = up)
     *          - Converts to ArduPilot velocity representation
     *          - Validates velocity solution status
     *          
     *          PSRDOP (ID 174) - Dilution of Precision:
     *          - Extracts HDOP (horizontal dilution of precision)
     *          - Extracts VDOP (vertical dilution of precision)
     *          - Used for accuracy estimation and fix quality assessment
     *          
     *          Solution Status Mapping:
     *          - SOL_COMPUTED + NARROW_INT (RTK Fixed) → GPS_OK_FIX_3D_RTK_FIXED
     *          - SOL_COMPUTED + NARROW_FLOAT (RTK Float) → GPS_OK_FIX_3D_RTK_FLOAT
     *          - SOL_COMPUTED + PSRDIFF (SBAS) → GPS_OK_FIX_3D_DGPS
     *          - SOL_COMPUTED + SINGLE → GPS_OK_FIX_3D
     *          - Other states → NO_FIX or NO_GPS
     * 
     * @return true if message was recognized and processed successfully
     * @return false if message ID is unknown or processing failed
     * 
     * @note Sets _new_position flag when position updated
     * @note Sets _new_speed flag when velocity updated
     * @note Message processing is non-blocking
     * 
     * Source: libraries/AP_GPS/AP_GPS_NOVA.cpp
     */
    bool process_message();

    /// @brief First byte of NOVA message preamble (0xAA)
    static const uint8_t NOVA_PREAMBLE1 = 0xaa;
    /// @brief Second byte of NOVA message preamble (0x44)
    static const uint8_t NOVA_PREAMBLE2 = 0x44;
    /// @brief Third byte of NOVA message preamble (0x12)
    static const uint8_t NOVA_PREAMBLE3 = 0x12;

    /// @brief Flag indicating new position data received and ready for consumption
    bool            _new_position:1;
    /// @brief Flag indicating new velocity data received and ready for consumption
    bool            _new_speed:1;
    
    /// @brief Timestamp of last velocity message (milliseconds) for latency calculation
    uint32_t        _last_vel_time;
    
    /// @brief Current index in initialization command array during receiver configuration
    uint8_t _init_blob_index = 0;
    /// @brief Timestamp when last initialization command was sent (milliseconds)
    uint32_t _init_blob_time = 0;
    /// @brief Array of initialization/configuration commands sent to receiver on startup
    static const char* const _initialisation_blob[4];
   
    /// @brief Count of CRC validation failures for diagnostics and error tracking
    uint32_t crc_error_counter = 0;

    /**
     * @brief NOVA binary message header structure (28 bytes)
     * 
     * @details Standard header present in all NOVA binary protocol messages.
     *          Provides message identification, length, timing, and status information.
     *          
     *          Message Identification:
     *          - messageid: Identifies message type (42=BESTPOS, 99=BESTVEL, 174=PSRDOP, etc.)
     *          - messagetype: Binary=0, ASCII=1, Abbreviated ASCII=2
     *          - portaddr: Port that generated the message
     *          
     *          Timing Information:
     *          - week: GPS week number (weeks since GPS epoch)
     *          - tow: Time of week in milliseconds
     *          - timestatus: Quality of time synchronization
     *          
     *          Message Flow Control:
     *          - sequence: Message sequence number for tracking dropped messages
     *          - messagelength: Payload data length in bytes (excludes header and CRC)
     *          - idletime: Receiver idle time percentage
     *          
     *          Status Information:
     *          - recvstatus: Receiver status word (error flags, antenna status, etc.)
     *          - recvswver: Receiver software version number
     * 
     * @note Total header size is fixed at 28 bytes
     * @note Followed by variable-length data payload and 4-byte CRC32
     */
    struct PACKED nova_header
    {
        uint8_t preamble[3];      ///< Message preamble: 0xAA 0x44 0x12
        uint8_t headerlength;     ///< Header length in bytes (always 28)
        uint16_t messageid;       ///< Message ID (42=BESTPOS, 99=BESTVEL, 174=PSRDOP)
        uint8_t messagetype;      ///< Message type: 0=binary, 1=ASCII, 2=abbreviated ASCII
        uint8_t portaddr;         ///< Communication port address that generated message
        uint16_t messagelength;   ///< Message data length in bytes (excludes header and CRC)
        uint16_t sequence;        ///< Message sequence number (wraps at 65535)
        uint8_t idletime;         ///< Receiver idle time as percentage (0-100)
        uint8_t timestatus;       ///< GPS time status (20=FINESTEERING, best quality)
        uint16_t week;            ///< GPS week number (weeks since January 6, 1980)
        uint32_t tow;             ///< GPS time of week in milliseconds (0-604800000)
        uint32_t recvstatus;      ///< Receiver status word (error flags, antenna status)
        uint16_t resv;            ///< Reserved field
        uint16_t recvswver;       ///< Receiver software version number
    };    

    /// @brief Message ID for PSRDOP (Dilution of Precision) message
    static const uint8_t NOVA_PSRDOP = 174;
    
    /**
     * @brief Dilution of Precision (DOP) message payload (PSRDOP - Message ID 174)
     * 
     * @details Provides geometric dilution of precision values that indicate the quality
     *          of satellite geometry for position/time solutions. Lower DOP values indicate
     *          better geometry and more accurate solutions.
     *          
     *          DOP Value Interpretation:
     *          - 1.0 = Ideal geometry
     *          - 1-2 = Excellent geometry
     *          - 2-5 = Good geometry
     *          - 5-10 = Moderate geometry
     *          - 10-20 = Fair geometry
     *          - >20 = Poor geometry
     *          
     *          GDOP (Geometric DOP): Overall geometric strength (3D position + time)
     *          PDOP (Position DOP): 3D position geometric strength
     *          HDOP (Horizontal DOP): 2D horizontal position geometric strength
     *          HTDOP (Horizontal + Time DOP): 2D position + time geometric strength
     *          TDOP (Time DOP): Time solution geometric strength
     *          VDOP (Vertical DOP): Computed as sqrt(PDOP² - HDOP²)
     * 
     * @note HDOP and VDOP are the primary values used for accuracy estimation
     * @note Message may contain additional per-satellite PRN data following these fields
     */
    struct PACKED psrdop
    {
        float gdop;         ///< Geometric dilution of precision (3D position + time)
        float pdop;         ///< Position dilution of precision (3D position)
        float hdop;         ///< Horizontal dilution of precision (2D position)
        float htdop;        ///< Horizontal + time dilution of precision
        float tdop;         ///< Time dilution of precision
        float cutoff;       ///< Elevation cutoff angle in degrees
        uint32_t svcount;   ///< Number of satellites used in DOP calculation
        // Additional per-satellite PRN data may follow
    };

    /// @brief Message ID for BESTPOS (best position solution) message
    static const uint8_t NOVA_BESTPOS = 42;
    
    /**
     * @brief Best Position Solution message payload (BESTPOS - Message ID 42)
     * 
     * @details Primary position message providing complete GNSS position solution with
     *          accuracy metrics, solution type, and satellite information.
     *          
     *          Solution Status (solstat) Values:
     *          - 0 = SOL_COMPUTED: Solution computed
     *          - 1 = INSUFFICIENT_OBS: Insufficient observations
     *          - 2 = NO_CONVERGENCE: No convergence
     *          - 3 = SINGULARITY: Singularity at parameters matrix
     *          
     *          Position Type (postype) Values (when solstat = SOL_COMPUTED):
     *          - 0 = NONE: No solution
     *          - 16 = SINGLE: Single point position (autonomous GPS)
     *          - 17 = PSRDIFF: Pseudorange differential (SBAS/WAAS)
     *          - 32 = L1_FLOAT: L1 float ambiguity solution
     *          - 33 = NARROW_FLOAT: Ionosphere-free float (RTK float)
     *          - 34 = L1_INT: L1 integer ambiguity solution
     *          - 48 = WIDE_INT: Wide-lane integer ambiguity solution
     *          - 49 = NARROW_INT: Narrow-lane integer ambiguity solution (RTK fixed)
     *          - 50 = RTK_DIRECT_INS: RTK with inertial integration
     *          
     *          Coordinate System:
     *          - WGS84 geodetic coordinates (latitude/longitude in degrees)
     *          - Height above WGS84 ellipsoid in meters
     *          - Undulation provides geoid separation (add to hgt for MSL altitude)
     *          
     *          Accuracy Information:
     *          - Standard deviations represent 1-sigma (68% confidence) accuracy
     *          - RTK fixed typically: latsdev/lngsdev < 0.02m, hgtsdev < 0.03m
     *          - RTK float typically: latsdev/lngsdev < 0.5m, hgtsdev < 0.8m
     *          - SBAS typically: latsdev/lngsdev < 1.5m, hgtsdev < 2.5m
     *          
     *          RTK Correction Information:
     *          - diffage: Age of differential corrections in seconds
     *          - sol_age: Age of solution (latency) in seconds
     *          - stnid: Base station identifier (4 characters)
     * 
     * @note Position reported relative to WGS84 ellipsoid, not mean sea level
     * @note For MSL altitude, add undulation to hgt: altitude_msl = hgt + undulation
     * @note RTK solutions require diffage < 30 seconds for reliable fixed solution
     * 
     * Source: libraries/AP_GPS/AP_GPS_NOVA.cpp for solution type processing
     */
    struct PACKED bestpos
    {
        uint32_t solstat;      ///< Solution status: 0=computed, 1=insufficient obs, 2=no convergence
        uint32_t postype;      ///< Position type: 16=SINGLE, 17=PSRDIFF, 33=RTK_FLOAT, 49=RTK_FIXED
        double lat;            ///< Latitude in degrees (WGS84, positive = North)
        double lng;            ///< Longitude in degrees (WGS84, positive = East)
        double hgt;            ///< Height above WGS84 ellipsoid in meters (not MSL)
        float undulation;      ///< Geoid separation in meters (geoid height above ellipsoid)
        uint32_t datumid;      ///< Datum ID number (61 = WGS84)
        float latsdev;         ///< Latitude standard deviation in meters (1-sigma accuracy)
        float lngsdev;         ///< Longitude standard deviation in meters (1-sigma accuracy)
        float hgtsdev;         ///< Height standard deviation in meters (1-sigma accuracy)
        uint8_t stnid[4];      ///< Base station ID (4 ASCII characters)
        float diffage;         ///< Differential/RTK correction age in seconds
        float sol_age;         ///< Solution age (latency) in seconds
        uint8_t svstracked;    ///< Number of satellites tracked by receiver
        uint8_t svsused;       ///< Number of satellites used in position solution
        uint8_t svsl1;         ///< Number of GPS+GLONASS L1 satellites used
        uint8_t svsmultfreq;   ///< Number of GPS+GLONASS L1+L2 satellites used
        uint8_t resv;          ///< Reserved field
        uint8_t extsolstat;    ///< Extended solution status (OEMV and newer)
        uint8_t galbeisigmask; ///< Galileo and BeiDou signal usage mask
        uint8_t gpsglosigmask; ///< GPS and GLONASS signal usage mask
    };

    /// @brief Message ID for BESTVEL (best velocity solution) message
    static const uint8_t NOVA_BESTVEL = 99;
    
    /**
     * @brief Best Velocity Solution message payload (BESTVEL - Message ID 99)
     * 
     * @details Provides 3D velocity solution derived from Doppler measurements.
     *          Velocity is reported in horizontal (speed/track) and vertical components.
     *          
     *          Solution Status (solstat) Values:
     *          - 0 = SOL_COMPUTED: Velocity solution computed
     *          - 1 = INSUFFICIENT_OBS: Insufficient observations
     *          - 2 = NO_CONVERGENCE: No convergence
     *          - 3 = SINGULARITY: Singularity in computation
     *          
     *          Velocity Type (veltype) Values:
     *          - 0 = NONE: No velocity solution
     *          - 8 = DOPPLER_VELOCITY: Instantaneous Doppler-derived velocity
     *          - 16 = SINGLE: Single point velocity (autonomous)
     *          - 17 = PSRDIFF: Pseudorange differential velocity (SBAS)
     *          - 32 = RTK_FLOAT: RTK float ambiguity velocity solution
     *          - 33 = RTK_FIXED: RTK fixed ambiguity velocity solution
     *          
     *          Coordinate System:
     *          - horspd: Horizontal speed magnitude in m/s (always positive)
     *          - trkgnd: Track over ground angle in degrees (0-360, 0=North, 90=East)
     *          - vertspd: Vertical speed in m/s (NED convention: positive = up)
     *          
     *          Velocity Components (NED frame conversion):
     *          - North velocity = horspd * cos(trkgnd)
     *          - East velocity = horspd * sin(trkgnd)
     *          - Down velocity = -vertspd (note sign conversion for NED down-positive)
     *          
     *          Timing Information:
     *          - latency: Computational latency in seconds (time delay in velocity computation)
     *          - age: Solution age in seconds (typically very small for velocity)
     *          
     *          Accuracy:
     *          - Doppler-derived velocity typically accurate to 0.03 m/s
     *          - RTK velocity solutions have minimal advantage over Doppler
     *          - Velocity is less affected by position errors than position itself
     * 
     * @note Velocity uses NED (North-East-Down) convention after conversion
     * @note vertspd sign: positive = upward motion, negative = downward motion
     * @note Track angle 0° = North, 90° = East, 180° = South, 270° = West
     * 
     * Source: libraries/AP_GPS/AP_GPS_NOVA.cpp for velocity processing
     */
    struct PACKED bestvel
    {
        uint32_t solstat;   ///< Solution status: 0=computed, 1=insufficient obs
        uint32_t veltype;   ///< Velocity type: 8=DOPPLER, 16=SINGLE, 33=RTK_FIXED
        float latency;      ///< Velocity computation latency in seconds
        float age;          ///< Solution age in seconds (typically near zero)
        double horspd;      ///< Horizontal speed in m/s (magnitude, always positive)
        double trkgnd;      ///< Track over ground in degrees (0-360, 0=North clockwise)
        double vertspd;     ///< Vertical speed in m/s (positive = upward motion)
        float resv;         ///< Reserved field
    };
    
    /**
     * @brief Union for message payload storage
     * 
     * @details Provides type-safe access to different message payloads while using
     *          a single memory buffer. Maximum size is 256 bytes to accommodate
     *          the largest expected NOVA message payload.
     * 
     * @note Access via appropriate member after checking message ID:
     *       - bestvelu for BESTVEL (ID 99)
     *       - bestposu for BESTPOS (ID 42)
     *       - psrdopu for PSRDOP (ID 174)
     *       - bytes for raw byte access during parsing
     */
    union PACKED msgbuffer {
        bestvel bestvelu;   ///< BESTVEL message payload (velocity solution)
        bestpos bestposu;   ///< BESTPOS message payload (position solution)
        psrdop psrdopu;     ///< PSRDOP message payload (dilution of precision)
        uint8_t bytes[256]; ///< Raw byte array for message reception
    };
    
    /**
     * @brief Union for message header storage
     * 
     * @details Provides type-safe access to message header structure while allowing
     *          byte-by-byte accumulation during parsing.
     */
    union PACKED msgheader {
        nova_header nova_headeru; ///< Structured header access
        uint8_t data[28];          ///< Raw byte array (28 bytes fixed)
    };

    /**
     * @brief Parser state machine structure
     * 
     * @details Maintains the complete state of the message parser including:
     *          - Current parsing state (preamble, header, data, CRC)
     *          - Accumulated message header and payload
     *          - CRC value for validation
     *          - Byte count for tracking parse progress
     *          
     *          Parser State Machine:
     *          1. PREAMBLE1-3: Synchronize on 3-byte preamble (0xAA 0x44 0x12)
     *          2. HEADERLENGTH: Read header length (always 28)
     *          3. HEADERDATA: Accumulate 28-byte header
     *          4. DATA: Accumulate message payload (length from header)
     *          5. CRC1-4: Read and validate 4-byte CRC32
     *          
     *          On successful CRC validation, parser resets to PREAMBLE1 state
     *          and process_message() is called to handle the complete message.
     * 
     * @note Parser automatically recovers from corrupted data by re-synchronizing on preamble
     * @note CRC is computed over header + payload (excludes preamble and CRC itself)
     */
    struct PACKED nova_msg_parser
    {
        /**
         * @brief Parser state machine enumeration
         * 
         * @details Defines all possible states in the message parsing process.
         *          Parser progresses sequentially through states as bytes arrive.
         */
        enum
        {
            PREAMBLE1 = 0,  ///< Waiting for first preamble byte (0xAA)
            PREAMBLE2,      ///< Waiting for second preamble byte (0x44)
            PREAMBLE3,      ///< Waiting for third preamble byte (0x12)
            HEADERLENGTH,   ///< Reading header length field
            HEADERDATA,     ///< Accumulating 28-byte header
            DATA,           ///< Accumulating message payload
            CRC1,           ///< Reading first CRC byte
            CRC2,           ///< Reading second CRC byte
            CRC3,           ///< Reading third CRC byte
            CRC4,           ///< Reading fourth CRC byte
        } nova_state;       ///< Current parser state
        
        msgbuffer data;     ///< Message payload buffer
        uint32_t crc;       ///< Received CRC32 value for validation
        msgheader header;   ///< Message header buffer
        uint16_t read;      ///< Number of bytes read in current state
    } nova_msg;             ///< Parser state instance
};
#endif
