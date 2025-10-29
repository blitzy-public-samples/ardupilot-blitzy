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
 * @file AP_GPS_SBF.h
 * @brief GPS driver for Septentrio receivers using SBF (Septentrio Binary Format) protocol
 * 
 * @details This driver implements support for Septentrio high-precision GNSS receivers
 *          including mosaic-X5, AsteRx, and PolaRx series. It parses SBF binary blocks
 *          to extract position, velocity, attitude, and status information.
 *          
 *          Supported SBF message blocks:
 *          - PVTGeodetic (4007): Position, velocity, time, fix mode, satellite count, DOP
 *          - DOP (4001): Dilution of precision values (PDOP, TDOP, HDOP, VDOP)
 *          - ReceiverStatus (4014): Receiver health monitoring, error flags, CPU load
 *          - VelCovGeodetic (5908): Velocity covariance matrix for accuracy estimation
 *          - AttEulerCov (5939): Dual-antenna attitude (roll/pitch/heading) with covariance
 *          - BaseVectorGeod (4028): RTK baseline vector for moving baseline applications
 *          - AuxAntPositions (5942): Auxiliary antenna positions for multi-antenna systems
 *          
 *          Key Features:
 *          - RTK support with centimeter-level accuracy (GPS_OK_FIX_3D_RTK_FIXED)
 *          - Dual-antenna heading measurement capability
 *          - Disk logging control (mount/unmount)
 *          - Multi-frequency multi-constellation support (GPS, GLONASS, Galileo, BeiDou)
 *          - SBAS augmentation configuration
 *          - Automatic receiver configuration via initialization strings
 *          
 *          Coordinate Systems:
 *          - Position: WGS84 geodetic latitude/longitude/height
 *          - Velocity: NED (North-East-Down) frame in m/s
 *          - Attitude: Roll/pitch/heading in degrees (0-360)
 *          - Baseline: ENU (East-North-Up) frame in meters
 *          
 *          Units:
 *          - Position: degrees (latitude/longitude), meters (height)
 *          - Velocity: m/s
 *          - Accuracy: meters (horizontal/vertical)
 *          - Heading: degrees (0-360, clockwise from north)
 *          - Time: GPS Time of Week (TOW) in milliseconds, Week Number
 *          - DOP: dimensionless (scaled by 0.01 in protocol)
 *          
 * @note Requires SBF protocol configuration on receiver (typically COM1 port at 115200 baud).
 *       The driver automatically configures the receiver on connection.
 * @note Velocity lag is approximately 80ms (0.08s) for 10Hz output rate.
 * 
 * @author Michael Oborne
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 * 
 * Source: libraries/AP_GPS/AP_GPS_SBF.h, libraries/AP_GPS/AP_GPS_SBF.cpp
 */

//  Septentrio GPS driver for ArduPilot.
//	Code by Michael Oborne
#pragma once

#include "AP_GPS.h"
#include "GPS_Backend.h"

#if AP_GPS_SBF_ENABLED

/// Disk activity indicator bit in ReceiverStatus
#define SBF_DISK_ACTIVITY (1 << 7)
/// Disk full warning bit in ReceiverStatus
#define SBF_DISK_FULL     (1 << 8)
/// Disk mounted status bit in ReceiverStatus
#define SBF_DISK_MOUNTED  (1 << 9)

/**
 * @class AP_GPS_SBF
 * @brief GPS driver backend for Septentrio receivers using SBF protocol
 * 
 * @details This class implements a GPS driver for Septentrio high-precision GNSS receivers
 *          (mosaic-X5, AsteRx, PolaRx series) that communicate via the Septentrio Binary Format
 *          (SBF) protocol. It provides full RTK support, dual-antenna heading, and advanced
 *          receiver configuration.
 *          
 *          Driver Lifecycle:
 *          1. Construction with UART port assignment
 *          2. Automatic receiver configuration (baud rate, output messages, constellation settings)
 *          3. Continuous message parsing in read() called at main loop rate
 *          4. State updates for position, velocity, heading, and status
 *          5. Optional disk logging control for post-processing
 *          
 *          Configuration Sequence:
 *          - Baud rate detection/configuration (115200 default)
 *          - Stream output configuration (SSO commands)
 *          - Receiver dynamics model (Moderate, UAV)
 *          - SBAS enable/disable based on parameters
 *          - Constellation selection (GPS, GLONASS, Galileo, BeiDou)
 *          
 *          Message Processing:
 *          - Binary protocol with $@ preamble
 *          - CRC-16 validation on all messages
 *          - Multi-message blocks parsed per main loop iteration
 *          - State machine for robust parsing
 *          
 *          RTK Support:
 *          - Accepts RTCM corrections via GPS_RTCM_DATA MAVLink messages
 *          - Reports RTK baseline via BaseVectorGeod messages
 *          - Provides high-accuracy position (centimeter-level)
 *          - Supports moving baseline configurations
 *          
 *          Dual-Antenna Heading:
 *          - Extracts roll/pitch/heading from AttEulerCov messages
 *          - Requires second antenna configured on receiver
 *          - Provides accurate heading independent of motion
 *          - Covariance data for accuracy estimation
 *          
 *          Health Monitoring:
 *          - ReceiverStatus provides CPU load, error flags, uptime
 *          - Disk logging status for internal logging
 *          - Configuration acknowledgment tracking
 *          - CRC error counting
 *          
 * @note Thread Safety: Called from GPS thread via read() method at 5-10Hz
 * @note Hardware Dependencies: Requires UART port configured for 115200 baud (auto-configured)
 * @note Memory: Allocates 256-byte message buffer on stack
 * 
 * @warning Disk logging control may interfere with receiver's internal logging if not managed carefully
 * @warning Configuration commands sent during initialization may take several seconds to complete
 * 
 * @see AP_GPS_Backend
 * @see AP_GPS
 */
class AP_GPS_SBF : public AP_GPS_Backend
{
public:
    /**
     * @brief Constructor for Septentrio SBF GPS driver
     * 
     * @param[in] _gps         Reference to main AP_GPS instance
     * @param[in] _params      Reference to GPS parameters for this instance
     * @param[in] _state       Reference to GPS state structure to update
     * @param[in] _port        UART port for communication with receiver
     * 
     * @note Initializes configuration state machine and parser
     */
    AP_GPS_SBF(AP_GPS &_gps, AP_GPS::Params &_params, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);
    
    /**
     * @brief Destructor for Septentrio SBF GPS driver
     * 
     * @details Unmounts disk logging if it was mounted during operation
     */
    ~AP_GPS_SBF();

    /**
     * @brief Get the highest status this GPS driver supports
     * 
     * @return AP_GPS::GPS_OK_FIX_3D_RTK_FIXED indicating full RTK support with centimeter accuracy
     * 
     * @note Also supports dual-antenna heading measurement when configured
     * @note Requires RTK corrections via RTCM messages for fixed RTK status
     */
    AP_GPS::GPS_Status highest_supported_status(void) override { return AP_GPS::GPS_OK_FIX_3D_RTK_FIXED; }

    /**
     * @brief Parse incoming SBF data and update GPS state
     * 
     * @details Called repeatedly from GPS thread to process serial data from receiver.
     *          Parses SBF binary blocks including PVTGeodetic, DOP, ReceiverStatus,
     *          VelCovGeodetic, AttEulerCov, and BaseVectorGeod messages. Updates position,
     *          velocity, heading, and status information in GPS state structure.
     *          
     *          Message Processing:
     *          - Reads available bytes from UART
     *          - State machine parsing with CRC validation
     *          - Multiple messages processed per call
     *          - Configuration state machine advancement
     *          
     * @return true if a valid SBF block was successfully parsed, false otherwise
     * 
     * @note Called at GPS thread rate (typically 5-10Hz)
     * @note Processes all available data in UART buffer each call
     */
    bool read() override;

    /**
     * @brief Get the driver identifier string
     * 
     * @return "SBF" identifier string
     * 
     * @note Used for logging and debugging to identify GPS type
     */
    const char *name() const override { return "SBF"; }

    /**
     * @brief Check if receiver configuration is complete
     * 
     * @details Verifies that all initialization commands have been sent and acknowledged
     *          by the receiver. Configuration includes baud rate, output messages, dynamics
     *          model, SBAS settings, and constellation selection.
     * 
     * @return true if configuration sequence reached Complete state, false otherwise
     * 
     * @note Configuration typically takes 2-5 seconds after power-on
     */
    bool is_configured (void) const override;

    /**
     * @brief Broadcast configuration failure reason to ground station
     * 
     * @details Sends STATUS_TEXT message explaining why receiver configuration failed,
     *          typically due to timeout waiting for command acknowledgment or incorrect
     *          baud rate detection.
     */
    void broadcast_configuration_failure_reason(void) const override;

#if HAL_GCS_ENABLED
    /**
     * @brief Indicate support for MAVLink GPS_RTK_DATA message
     * 
     * @return true indicating this driver can populate full RTK data including baseline
     * 
     * @note Enables ground station to request RTK-specific information
     * @note RTK data includes baseline length, accuracy, and satellite count
     */
    bool supports_mavlink_gps_rtk_message(void) const override { return true; };
#endif

    /**
     * @brief Get velocity measurement lag time
     * 
     * @details Septentrio receivers at 10Hz output rate have approximately 80ms lag
     *          between measurement and output due to processing time.
     * 
     * @param[out] lag_sec Velocity lag time in seconds (0.08s)
     * 
     * @return true indicating confident in lag value
     * 
     * @note Used by EKF for compensating velocity measurements in state prediction
     */
    bool get_lag(float &lag_sec) const override { lag_sec = 0.08f; return true; } ;

    /**
     * @brief Check if GPS receiver is healthy
     * 
     * @details Evaluates receiver status including configuration state, recent data,
     *          and error flags from ReceiverStatus messages.
     * 
     * @return true if receiver is configured and providing valid data, false otherwise
     * 
     * @note Checks for software errors, watchdog resets, CPU overload, and congestion
     */
    bool is_healthy(void) const override;

    /**
     * @brief Check if internal disk logging is healthy
     * 
     * @details Monitors disk status bits from ReceiverStatus to verify logging is active
     *          and disk is not full.
     * 
     * @return true if disk mounted, active, and not full; false otherwise
     * 
     * @note Only relevant if internal logging to receiver's SD card is configured
     */
    bool logging_healthy(void) const override;

    /**
     * @brief Prepare receiver for vehicle arming
     * 
     * @details Mounts receiver's internal disk for logging when vehicle arms for the first time.
     *          Subsequent arming does not remount to preserve continuous logging.
     * 
     * @return true if preparation successful, false otherwise
     * 
     * @note Disk remains mounted until vehicle disarms to avoid log file fragmentation
     */
    bool prepare_for_arming(void) override;

    /**
     * @brief Get receiver error codes
     * 
     * @details Retrieves RxError field from most recent ReceiverStatus message indicating
     *          software errors, watchdog resets, congestion, CPU overload, and configuration issues.
     * 
     * @param[out] error_codes Receiver error bitmask from ReceiverStatus.RxError
     * 
     * @return true indicating error codes are available
     * 
     * @note Error bits defined: SOFTWARE, WATCHDOG, CONGESTION, MISSEDEVENT, CPUOVERLOAD, INVALIDCONFIG, OUTOFGEOFENCE
     */
    bool get_error_codes(uint32_t &error_codes) const override { error_codes = RxError; return true; };

private:
    /**
     * @brief Parse single byte of SBF protocol data
     * 
     * @param[in] temp Byte received from UART
     * 
     * @return true if complete valid message parsed, false otherwise
     * 
     * @note State machine parser handles preamble, CRC, block ID, length, and data
     */
    bool parse(uint8_t temp);
    
    /**
     * @brief Process complete SBF message block
     * 
     * @details Decodes message based on block ID and updates GPS state accordingly.
     *          Handles PVTGeodetic, DOP, ReceiverStatus, VelCovGeodetic, AttEulerCov,
     *          and BaseVectorGeod message types.
     * 
     * @return true if message processed successfully, false otherwise
     */
    bool process_message();

    /// SBF protocol preamble first byte: '$'
    static const uint8_t SBF_PREAMBLE1 = '$';
    /// SBF protocol preamble second byte: '@'
    static const uint8_t SBF_PREAMBLE2 = '@';

    /// Current index in initialization command array
    uint8_t _init_blob_index;
    /// Timestamp of last configuration command sent (milliseconds)
    uint32_t _init_blob_time;
    
    /**
     * @enum Config_State
     * @brief Receiver configuration sequence states
     * 
     * @details Configuration proceeds through these states in order, sending commands
     *          and waiting for acknowledgment before advancing. Timeout causes retry.
     */
    enum class Config_State {
        Baud_Rate,      ///< Detecting/setting baud rate to 115200
        SSO,            ///< Configuring stream output (SSO commands)
        Blob,           ///< Sending initialization blob commands
        SBAS,           ///< Configuring SBAS augmentation enable/disable
        SGA,            ///< Setting signal processing algorithm
        Constellation,  ///< Configuring constellation selection (GPS/GLONASS/Galileo/BeiDou)
        Complete        ///< Configuration finished, normal operation
    };
    /// Current configuration state
    Config_State config_step;
    char *config_string;
    static constexpr const char* _initialisation_blob[] = {
    "srd,Moderate,UAV",
    "sem,PVT,5",
    "spm,Rover,all",
    "sso,Stream2,Dsk1,postprocess+event+comment+ReceiverStatus,msec100",
#if defined (GPS_SBF_EXTRA_CONFIG)
    GPS_SBF_EXTRA_CONFIG
#endif
    };
    static constexpr const char* sbas_off = "sst, -SBAS";
    static constexpr const char* sbas_on_blob[] = {
                                                   "snt,+GEOL1+GEOL5",
                                                   "sst,+SBAS",
                                                   "ssbc,auto,Operational,MixedSystems,auto",
                                                  };
    uint32_t _config_last_ack_time;

    const char* _port_enable = "\nSSSSSSSSSS\n";
   
    uint32_t crc_error_counter = 0;
    uint32_t RxState;
    uint32_t RxError;

    /**
     * @brief Mount receiver's internal SD card for logging
     * 
     * @details Sends disk mount command to enable internal logging to receiver's SD card
     * 
     * @note Used during arming to start logging flight data
     */
    void mount_disk(void) const;
    
    /**
     * @brief Unmount receiver's internal SD card
     * 
     * @details Sends disk unmount command to safely close log files
     * 
     * @note Called on disarm or in destructor
     */
    void unmount_disk(void) const;
    
    /// Flag indicating if vehicle has been armed (used for disk logging control)
    bool _has_been_armed;

    /**
     * @enum sbf_ids
     * @brief SBF message block identifiers
     * 
     * @details Block IDs for SBF binary messages parsed by this driver.
     *          Each message type provides specific navigation or status information.
     */
    enum sbf_ids {
        DOP = 4001,             ///< Dilution of Precision: PDOP, TDOP, HDOP, VDOP values
        PVTGeodetic = 4007,     ///< Position/Velocity/Time: primary navigation solution in WGS84
        ReceiverStatus = 4014,  ///< Receiver status: health, errors, CPU load, disk status
        BaseVectorGeod = 4028,  ///< RTK baseline vector: relative position for moving baseline
        VelCovGeodetic = 5908,  ///< Velocity covariance: uncertainty matrix for velocity estimate
        AttEulerCov = 5939,     ///< Attitude Euler angles and covariance: dual-antenna heading
        AuxAntPositions = 5942, ///< Auxiliary antenna positions: multi-antenna configurations
    };

    /**
     * @struct msg4007
     * @brief SBF PVTGeodetic message (Block ID 4007)
     * 
     * @details Primary navigation solution providing position, velocity, and time in WGS84 geodetic coordinates.
     *          This is the main message used to update GPS state with position and velocity.
     *          
     *          Coordinate System: WGS84 geodetic (latitude/longitude in degrees, height in meters above ellipsoid)
     *          Velocity Frame: NED (North-East-Down) in m/s
     *          
     *          Mode field indicates fix type:
     *          - 0: No PVT solution
     *          - 1: Stand-alone PVT
     *          - 2: Differential PVT
     *          - 4: Fixed RTK
     *          - 5: Float RTK
     *          - 6: Dead reckoning
     */
    struct PACKED msg4007 // PVTGeodetic
    {
         uint32_t TOW;          ///< GPS Time of Week in milliseconds
         uint16_t WNc;          ///< GPS Week Number (continuous, not modulo 1024)
         uint8_t Mode;          ///< PVT mode: 0=No solution, 1=Stand-alone, 2=Differential, 4=RTK fixed, 5=RTK float
         uint8_t Error;         ///< Error code (0 = no error)
         double Latitude;       ///< Latitude in degrees (WGS84, positive = North)
         double Longitude;      ///< Longitude in degrees (WGS84, positive = East)
         double Height;         ///< Height above WGS84 ellipsoid in meters
         float Undulation;      ///< Geoid undulation in meters (ellipsoid height - MSL height)
         float Vn;              ///< Velocity North in m/s (NED frame)
         float Ve;              ///< Velocity East in m/s (NED frame)
         float Vu;              ///< Velocity Up in m/s (NED frame)
         float COG;             ///< Course over ground in degrees (0-360, clockwise from North)
         double RxClkBias;      ///< Receiver clock bias in meters
         float RxClkDrift;      ///< Receiver clock drift in m/s
         uint8_t TimeSystem;    ///< Time system: 0=GPS, 3=GLONASS, 6=Galileo, etc.
         uint8_t Datum;         ///< Datum: 0=WGS84
         uint8_t NrSV;          ///< Number of satellites used in solution
         uint8_t WACorrInfo;    ///< Wide area correction info
         uint16_t ReferenceID;  ///< Reference station ID for differential/RTK
         uint16_t MeanCorrAge;  ///< Mean age of differential corrections in 0.01s units
         uint32_t SignalInfo;   ///< Signal type bitmask
         uint8_t AlertFlag;     ///< Alert flag for integrity monitoring
         // rev1
         uint8_t NrBases;       ///< Number of base stations used
         uint16_t PPPInfo;      ///< PPP (Precise Point Positioning) status info
         // rev2
         uint16_t Latency;      ///< Solution latency in 0.0001s units
         uint16_t HAccuracy;    ///< Horizontal accuracy (2DRMS) in 0.01m units
         uint16_t VAccuracy;    ///< Vertical accuracy (2-sigma) in 0.01m units
         uint8_t Misc;          ///< Miscellaneous flags
    };

    /**
     * @struct msg4001
     * @brief SBF DOP message (Block ID 4001)
     * 
     * @details Dilution of Precision values indicating geometric quality of satellite constellation.
     *          Lower DOP values indicate better geometry and more accurate position estimates.
     *          
     *          DOP values are dimensionless and scaled by 0.01 in the protocol.
     *          Typical values: <2 excellent, 2-5 good, 5-10 moderate, 10-20 fair, >20 poor
     */
    struct PACKED msg4001 // DOP
    {
         uint32_t TOW;          ///< GPS Time of Week in milliseconds
         uint16_t WNc;          ///< GPS Week Number
         uint8_t NrSV;          ///< Number of satellites used in DOP calculation
         uint8_t Reserved;      ///< Reserved byte
         uint16_t PDOP;         ///< Position DOP (3D) in 0.01 units
         uint16_t TDOP;         ///< Time DOP in 0.01 units
         uint16_t HDOP;         ///< Horizontal DOP (2D) in 0.01 units
         uint16_t VDOP;         ///< Vertical DOP in 0.01 units
         float HPL;             ///< Horizontal Protection Level in meters (integrity)
         float VPL;             ///< Vertical Protection Level in meters (integrity)
    };

    /**
     * @struct msg4014
     * @brief SBF ReceiverStatus message (Block ID 4014, revision 2)
     * 
     * @details Receiver health and status information including CPU load, error flags,
     *          and operational state. Used to monitor receiver health and detect issues.
     *          
     *          RxError bitmask flags (see enum below):
     *          - Bit 3: SOFTWARE error
     *          - Bit 4: WATCHDOG reset
     *          - Bit 6: CONGESTION
     *          - Bit 8: MISSEDEVENT
     *          - Bit 9: CPUOVERLOAD
     *          - Bit 10: INVALIDCONFIG
     *          - Bit 11: OUTOFGEOFENCE
     */
    struct PACKED msg4014 // ReceiverStatus (v2)
    {
         uint32_t TOW;          ///< GPS Time of Week in milliseconds
         uint16_t WNc;          ///< GPS Week Number
         uint8_t CPULoad;       ///< CPU load percentage (0-100, >90 triggers CPUOVERLOAD flag)
         uint8_t ExtError;      ///< Extended error code
         uint32_t UpTime;       ///< Receiver uptime in seconds since power-on
         uint32_t RxState;      ///< Receiver state bitmask (includes disk status bits)
         uint32_t RxError;      ///< Receiver error bitmask (see enum for bit definitions)
         // remaining data is AGCData, which we don't have a use for, don't extract the data
    };

    /**
     * @struct VectorInfoGeod
     * @brief Baseline vector information sub-block for RTK
     * 
     * @details Contains relative position and velocity between base and rover antennas
     *          in ENU (East-North-Up) geodetic frame. Used for RTK moving baseline applications.
     */
    struct PACKED VectorInfoGeod {
        uint8_t NrSV;           ///< Number of satellites used in baseline solution
        uint8_t Error;          ///< Error code (0 = no error)
        uint8_t Mode;           ///< Solution mode: 0=No solution, 1=Stand-alone, 4=RTK fixed, 5=RTK float
        uint8_t Misc;           ///< Miscellaneous flags
        double DeltaEast;       ///< Baseline East component in meters (ENU frame)
        double DeltaNorth;      ///< Baseline North component in meters (ENU frame)
        double DeltaUp;         ///< Baseline Up component in meters (ENU frame)
        float DeltaVe;          ///< Relative velocity East in m/s
        float DeltaVn;          ///< Relative velocity North in m/s
        float DeltaVu;          ///< Relative velocity Up in m/s
        uint16_t Azimuth;       ///< Baseline azimuth in 0.01 degree units (0-36000)
        int16_t Elevation;      ///< Baseline elevation in 0.01 degree units
        uint8_t ReferenceID;    ///< Base station ID
        uint16_t CorrAge;       ///< Age of corrections in 0.01s units
        uint32_t SignalInfo;    ///< Signal type bitmask
    };

    /**
     * @struct msg4028
     * @brief SBF BaseVectorGeod message (Block ID 4028)
     * 
     * @details RTK baseline vector from base station to rover. Provides relative position
     *          and velocity for moving baseline applications and RTK status reporting.
     *          
     *          Coordinate System: ENU (East-North-Up) geodetic frame relative to base antenna
     *          
     * @note Message can contain multiple baselines, but driver only processes the first one
     */
    struct PACKED msg4028 // BaseVectorGeod
    {
        uint32_t TOW;           ///< GPS Time of Week in milliseconds
        uint16_t WNc;           ///< GPS Week Number
        uint8_t N;              ///< Number of baseline vectors in message (typically 1)
        uint8_t SBLength;       ///< Length of each VectorInfoGeod sub-block in bytes
        VectorInfoGeod info;    ///< First baseline vector info (only first one processed)
    };

    /**
     * @struct msg5908
     * @brief SBF VelCovGeodetic message (Block ID 5908)
     * 
     * @details Velocity covariance matrix for uncertainty estimation of velocity measurements.
     *          Provides full 3x3 covariance matrix plus clock drift variance in NED frame.
     *          Used by EKF for proper weighting of velocity measurements.
     *          
     *          Coordinate System: NED (North-East-Down) frame
     *          Units: Covariance in (m/s)^2, clock drift covariance in (m/s)^2
     */
    struct PACKED msg5908 // VelCovGeodetic
    {
        uint32_t TOW;           ///< GPS Time of Week in milliseconds
        uint16_t WNc;           ///< GPS Week Number
        uint8_t Mode;           ///< PVT mode (same as msg4007)
        uint8_t Error;          ///< Error code (0 = no error)
        float Cov_VnVn;         ///< Variance of North velocity in (m/s)^2
        float Cov_VeVe;         ///< Variance of East velocity in (m/s)^2
        float Cov_VuVu;         ///< Variance of Up velocity in (m/s)^2
        float Cov_DtDt;         ///< Variance of receiver clock drift in (m/s)^2
        float Cov_VnVe;         ///< Covariance between North and East velocity in (m/s)^2
        float Cov_VnVu;         ///< Covariance between North and Up velocity in (m/s)^2
        float Cov_VnDt;         ///< Covariance between North velocity and clock drift in (m/s)^2
        float Cov_VeVu;         ///< Covariance between East and Up velocity in (m/s)^2
        float Cov_VeDt;         ///< Covariance between East velocity and clock drift in (m/s)^2
        float Cov_VuDt;         ///< Covariance between Up velocity and clock drift in (m/s)^2
    };

    /**
     * @struct msg5939
     * @brief SBF AttEulerCov message (Block ID 5939)
     * 
     * @details Attitude covariance matrix for dual-antenna heading measurement uncertainty.
     *          Provides variance estimates for roll, pitch, and heading angles derived from
     *          relative position of two GNSS antennas on the vehicle.
     *          
     *          Requires two antennas configured on receiver with known baseline constraints.
     *          Heading accuracy depends on antenna separation (longer baseline = better accuracy).
     *          
     *          Coordinate System: Vehicle body frame (roll/pitch/heading in degrees)
     *          Units: Covariance in degrees^2
     *          
     *          Error field bit encoding:
     *          - Bits 0-1: Antenna 1 error (00=no error, 01=not enough measurements, 10=collinear, 11=config mismatch)
     *          - Bits 2-3: Antenna 2 error (same encoding)
     *          - Bit 7: Attitude not requested
     */
    struct PACKED msg5939       // AttEulerCov
    {
        uint32_t TOW;           ///< GPS Time of Week in milliseconds
        uint16_t WNc;           ///< GPS Week Number
        uint8_t Reserved;       ///< Reserved byte (unused)
        uint8_t Error;          ///< Error code bitmask for antenna status
        float Cov_HeadHead;     ///< Heading variance in degrees^2
        float Cov_PitchPitch;   ///< Pitch variance in degrees^2
        float Cov_RollRoll;     ///< Roll variance in degrees^2
        float Cov_HeadPitch;    ///< Heading-Pitch covariance in degrees^2 (typically Do-Not-Use)
        float Cov_HeadRoll;     ///< Heading-Roll covariance in degrees^2 (typically Do-Not-Use)
        float Cov_PitchRoll;    ///< Pitch-Roll covariance in degrees^2 (typically Do-Not-Use)
    };

    /**
     * @struct AuxAntPositionSubBlock
     * @brief Auxiliary antenna position information sub-block
     * 
     * @details Relative position and velocity of an auxiliary antenna with respect to the main antenna.
     *          Used in multi-antenna configurations for attitude determination or diversity.
     *          
     *          Coordinate System: ENU (East-North-Up) relative to main antenna
     *          Units: Position in meters, velocity in m/s
     */
    struct PACKED AuxAntPositionSubBlock {
        uint8_t NrSV;           ///< Number of satellites tracked by this auxiliary antenna
        uint8_t Error;          ///< Error code for auxiliary antenna solution (0 = no error)
        uint8_t AmbiguityType;  ///< Ambiguity resolution: 0=fixed (RTK), 1=float
        uint8_t AuxAntID;       ///< Auxiliary antenna identifier (1, 2, 3...)
        double DeltaEast;       ///< Position East component in meters (ENU frame, relative to main antenna)
        double DeltaNorth;      ///< Position North component in meters (ENU frame, relative to main antenna)
        double DeltaUp;         ///< Position Up component in meters (ENU frame, relative to main antenna)
        double EastVel;         ///< Velocity East component in m/s (relative to main antenna)
        double NorthVel;        ///< Velocity North component in m/s (relative to main antenna)
        double UpVel;           ///< Velocity Up component in m/s (relative to main antenna)
    };

    /**
     * @struct msg5942
     * @brief SBF AuxAntPositions message (Block ID 5942)
     * 
     * @details Positions of auxiliary antennas relative to main antenna for multi-antenna systems.
     *          Enables attitude determination from antenna baseline vectors and antenna diversity.
     *          
     *          Coordinate System: ENU (East-North-Up) relative to main antenna location
     *          
     * @note Message can contain multiple auxiliary antenna sub-blocks, driver processes first one
     */
    struct PACKED msg5942   // AuxAntPositions
    {
        uint32_t TOW;           ///< GPS Time of Week in milliseconds
        uint16_t WNc;           ///< GPS Week Number
        uint8_t N;              ///< Number of AuxAntPosition sub-blocks in message
        uint8_t SBLength;       ///< Length of each sub-block in bytes
        AuxAntPositionSubBlock ant1;    ///< First auxiliary antenna position (only first processed)
    };

    /**
     * @union msgbuffer
     * @brief Union for type-safe access to SBF message data
     * 
     * @details Allows access to received message bytes as appropriate message structure type
     *          based on block ID. Saves memory by overlaying all message types.
     */
    union PACKED msgbuffer {
        msg4007 msg4007u;       ///< PVTGeodetic message structure
        msg4001 msg4001u;       ///< DOP message structure
        msg4014 msg4014u;       ///< ReceiverStatus message structure
        msg4028 msg4028u;       ///< BaseVectorGeod message structure
        msg5908 msg5908u;       ///< VelCovGeodetic message structure
        msg5939 msg5939u;       ///< AttEulerCov message structure
        msg5942 msg5942u;       ///< AuxAntPositions message structure
        uint8_t bytes[256];     ///< Raw byte access (256 byte maximum SBF message size)
    };

    /**
     * @struct sbf_msg_parser_t
     * @brief SBF protocol parser state machine
     * 
     * @details Maintains state for parsing binary SBF protocol with preamble detection,
     *          block ID extraction, length validation, CRC checking, and data buffering.
     *          
     *          Protocol format:
     *          1. Preamble: '$' '@' (0x24 0x40)
     *          2. CRC: 16-bit CRC-CCITT
     *          3. Block ID: 16-bit message type identifier
     *          4. Length: 16-bit message length in bytes
     *          5. Data: Variable length payload
     *          
     *          State machine processes one byte at a time from UART, advancing through
     *          states to detect and validate complete messages.
     */
    struct sbf_msg_parser_t
    {
        /**
         * @enum (anonymous)
         * @brief Parser state machine states
         */
        enum
        {
            PREAMBLE1 = 0,      ///< Waiting for first preamble byte '$'
            PREAMBLE2,          ///< Waiting for second preamble byte '@'
            CRC1,               ///< Reading CRC byte 1
            CRC2,               ///< Reading CRC byte 2
            BLOCKID1,           ///< Reading block ID byte 1
            BLOCKID2,           ///< Reading block ID byte 2
            LENGTH1,            ///< Reading length byte 1
            LENGTH2,            ///< Reading length byte 2
            DATA,               ///< Reading message data bytes
            COMMAND_LINE        ///< Parsing ASCII command response (not binary)
        } sbf_state;            ///< Current parser state
        uint16_t preamble;      ///< Detected preamble value
        uint16_t crc;           ///< Message CRC value
        uint16_t blockid;       ///< Message block ID (sbf_ids enum)
        uint16_t length;        ///< Message payload length in bytes
        msgbuffer data;         ///< Message data buffer
        uint16_t read;          ///< Number of data bytes read so far
    } sbf_msg;                  ///< Parser state instance

    /**
     * @enum (anonymous)
     * @brief Receiver error flag bitmask values from ReceiverStatus.RxError field
     * 
     * @details Error flags indicating various receiver health issues. Multiple flags
     *          can be set simultaneously. Used by is_healthy() to determine GPS health.
     */
    enum {
        SOFTWARE      = (1 << 3),   ///< Software warning or error detected (bit 3) - cleared by 'lif, error' command
        WATCHDOG      = (1 << 4),   ///< Watchdog timer expired at least once since power-on (bit 4) - indicates firmware hang
        CONGESTION    = (1 << 6),   ///< Output data congestion on communication port (bit 6) - data rate too high
        MISSEDEVENT   = (1 << 8),   ///< External event congestion detected (bit 8) - too many events on EVENTx pins
        CPUOVERLOAD   = (1 << 9),   ///< CPU load exceeds 90% (bit 9) - receiver operation may be unreliable
        INVALIDCONFIG = (1 << 10),  ///< Invalid or missing configuration file (bit 10) - permission or channel config
        OUTOFGEOFENCE = (1 << 11),  ///< Receiver outside permitted geofence region (bit 11) - geo-fencing violation
    };

    static constexpr const char *portIdentifiers[] = { "COM", "USB", "IP1", "NTR", "IPS", "IPR" };
    char portIdentifier[5];
    uint8_t portLength;
    bool readyForCommand;
};
#endif
