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
 * @file AP_GPS.h
 * @brief ArduPilot GPS subsystem frontend manager
 * 
 * @details Central GPS manager (singleton) coordinating multiple GPS receivers.
 * 
 * Core Responsibilities:
 * - Backend detection and instantiation for multiple GPS protocols
 * - Auto-baud rate probing (9600-460800 baud)
 * - Protocol auto-detection (UBX, NMEA, SBF, SBP, GSOF, NOVA, etc.)
 * - Primary GPS selection based on fix quality
 * - GPS blending for improved accuracy with multiple receivers
 * - RTCM injection for RTK corrections
 * - Pre-arm checks for GPS health and consistency
 * - EKF integration with position, velocity, and timing data
 * - Binary logging (GPS/GPA/GPR messages)
 * - MAVLink telemetry streaming
 * 
 * Supports up to GPS_MAX_RECEIVERS (typically 2) simultaneous receivers with
 * automatic failover and blending capabilities.
 * 
 * @note Access via AP::gps() singleton
 * 
 * Source: libraries/AP_GPS/AP_GPS.h:1-820
 * Source: libraries/AP_GPS/AP_GPS.cpp (implementation)
 */

#pragma once

#include "AP_GPS_config.h"

#if AP_GPS_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Param/AP_Param.h>
#include "GPS_detect_state.h"
#include <AP_Math/AP_Math.h>
#include <AP_MSP/msp.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <SITL/SIM_GPS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#define GPS_UNKNOWN_DOP UINT16_MAX // set unknown DOP's to maximum value, which is also correct for MAVLink

// the number of GPS leap seconds - copied into SIM_GPS.cpp
#define GPS_LEAPSECONDS_MILLIS 18000ULL

#define UNIX_OFFSET_MSEC (17000ULL * 86400ULL + 52ULL * 10ULL * AP_MSEC_PER_WEEK - GPS_LEAPSECONDS_MILLIS)

#ifndef GPS_MOVING_BASELINE
#define GPS_MOVING_BASELINE GPS_MAX_RECEIVERS>1
#endif

#if GPS_MOVING_BASELINE
#include "MovingBase.h"
#endif // GPS_MOVING_BASELINE

class AP_GPS_Backend;
class RTCM3_Parser;

/**
 * @class AP_GPS
 * @brief GPS subsystem frontend manager coordinating all GPS receivers
 * 
 * @details Singleton class managing the complete GPS lifecycle:
 * 
 * **Detection Phase:**
 * - Auto-baud rate probing: 9600, 19200, 38400, 57600, 115200, 230400, 460800 baud
 * - Protocol detection: UBX, NMEA, SBF, SBP, GSOF, NOVA, SIRF, ERB, MSP, etc.
 * - Instantiate appropriate backend drivers (AP_GPS_UBLOX, AP_GPS_NMEA, etc.)
 * 
 * **Configuration Phase:**
 * - Configure message rates for position, velocity, and status
 * - Set navigation engine mode (portable, automotive, airborne 1G/2G/4G)
 * - Configure GNSS constellation usage (GPS, GLONASS, Galileo, BeiDou)
 * - Setup RTK base/rover roles for moving baseline configurations
 * 
 * **Data Flow:**
 * - Read data from all GPS backend instances
 * - Update primary GPS selection based on fix quality, HDOP, satellite count
 * - Optionally blend multiple GPS solutions for improved accuracy
 * - Publish position, velocity, and timing to EKF (Extended Kalman Filter)
 * 
 * **Health Monitoring:**
 * - Track message timeouts (>5 seconds = unhealthy)
 * - Monitor pre-arm check status (minimum fix quality, HDOP thresholds)
 * - Count visible satellites (minimum 6 recommended for good fix)
 * - Calculate HDOP/VDOP (Dilution of Precision quality metrics)
 * 
 * **RTK Support:**
 * - RTCM3 injection to all RTK-capable receivers
 * - Moving baseline coordination between base and rover receivers
 * - RTK fix status reporting (Float vs Fixed)
 * 
 * **System Integration:**
 * - Logging: GPS (position), GPA (accuracy), GPR (RTK data) messages
 * - MAVLink telemetry: GPS_RAW_INT, GPS2_RAW, GPS_RTK
 * - Parameter management: GPS type, rate, GNSS mode, antenna offsets
 * - Thread-safe access via semaphore for multi-threaded EKF reads
 * 
 * **Coordinate Systems:**
 * - Position: WGS84 geodetic (latitude/longitude in degrees, altitude in meters)
 * - Velocity: NED (North-East-Down) frame in m/s
 * - Antenna offset: NED frame in meters relative to vehicle center of gravity
 * 
 * **Units:**
 * - Position: degrees (lat/lon), meters (altitude)
 * - Velocity: m/s (NED frame)
 * - Accuracy estimates: meters (position), m/s (velocity)
 * - DOP values: dimensionless, scaled by 100 internally (HDOP 155 = 1.55)
 * - Time: milliseconds (GPS time of week), GPS week number
 * - Course: radians (0 = North, increases clockwise) or degrees based on method
 * 
 * @warning GPS initialization timing is critical for EKF convergence. A stable
 * GPS fix is required before the EKF can set its origin and the vehicle can arm.
 * Ensure GPS has clear sky view during initialization.
 * 
 * @note Most accurate configuration: Dual RTK GPS (primary + secondary for
 * redundancy), GPS blending enabled, moving baseline for heading. HDOP < 2
 * indicates excellent geometry, HDOP > 10 indicates poor geometry.
 * 
 * @note Thread safety: Access GPS data through get_semaphore() when reading
 * from non-main threads. Backend updates are protected by rsem semaphore.
 * 
 * Example usage:
 * @code
 * // Check GPS status and get position
 * const AP_GPS &gps = AP::gps();
 * if (gps.status() >= AP_GPS_FixType::FIX_3D) {
 *     const Location &loc = gps.location();
 *     // Use loc.lat, loc.lng, loc.alt
 *     float speed = gps.ground_speed();  // m/s
 *     float course = gps.ground_course(); // radians
 * }
 * 
 * // Get velocity for navigation
 * const Vector3f &vel_ned = gps.velocity();
 * // vel_ned.x = north m/s, vel_ned.y = east m/s, vel_ned.z = down m/s
 * @endcode
 * 
 * Source: libraries/AP_GPS/AP_GPS.h:51-814
 * Source: libraries/AP_GPS/AP_GPS.cpp (implementation)
 */
class AP_GPS
{
    friend class AP_GPS_Blended;
    friend class AP_GPS_ERB;
    friend class AP_GPS_GSOF;
    friend class AP_GPS_MAV;
    friend class AP_GPS_MSP;
    friend class AP_GPS_ExternalAHRS;
    friend class AP_GPS_NMEA;
    friend class AP_GPS_NOVA;
    friend class AP_GPS_PX4;
    friend class AP_GPS_SBF;
    friend class AP_GPS_SBP;
    friend class AP_GPS_SBP2;
    friend class AP_GPS_SIRF;
    friend class AP_GPS_UBLOX;
    friend class AP_GPS_Backend;
    friend class AP_GPS_DroneCAN;

public:
    AP_GPS();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_GPS);

    static AP_GPS *get_singleton() {
        return _singleton;
    }

    /**
     * @brief Get semaphore for thread-safe GPS data access
     * 
     * @details Provides lock for multi-threaded access to GPS data. Use this
     * semaphore when reading GPS data from non-main threads (e.g., EKF thread).
     * Backend updates are protected by this semaphore.
     * 
     * @return HAL_Semaphore& Reference to the GPS data protection semaphore
     * 
     * @note Must be held when reading GPS state from threads other than main loop
     */
    HAL_Semaphore &get_semaphore(void) {
        return rsem;
    }
    
    /**
     * @enum GPS_Type
     * @brief GPS receiver type identifiers for backend driver selection
     * 
     * @details Specifies which GPS protocol driver to use for each GPS instance.
     * GPS_TYPE_AUTO enables automatic protocol detection through message parsing.
     */
    enum GPS_Type {
        GPS_TYPE_NONE  = 0,  ///< No GPS enabled
        GPS_TYPE_AUTO  = 1,  ///< Auto-detect protocol (UBX/NMEA/SBF/SBP/GSOF/NOVA/etc.)
        GPS_TYPE_UBLOX = 2,  ///< u-blox UBX binary protocol (most common, supports RTK)
        // GPS_TYPE_MTK   = 3,  // driver removed
        // GPS_TYPE_MTK19 = 4,  // driver removed
        GPS_TYPE_NMEA  = 5,  ///< NMEA 0183 ASCII protocol (universal but limited accuracy)
        GPS_TYPE_SIRF  = 6,  ///< SiRF binary protocol (legacy)
        GPS_TYPE_HIL   = 7,  ///< Hardware-in-loop simulation GPS
        GPS_TYPE_SBP   = 8,  ///< Swift Binary Protocol v1 (Swift Navigation)
        GPS_TYPE_UAVCAN = 9, ///< DroneCAN/UAVCAN GPS (CAN bus interface)
        GPS_TYPE_SBF   = 10, ///< Septentrio Binary Format (high-precision receivers)
        GPS_TYPE_GSOF  = 11, ///< Trimble GSOF protocol
        GPS_TYPE_ERB = 13,   ///< Emlid Reach Binary protocol
        GPS_TYPE_MAV = 14,   ///< MAVLink GPS_INPUT message injection
        GPS_TYPE_NOVA = 15,  ///< Novatel/Tersus/ComNav binary protocol
        GPS_TYPE_HEMI = 16,  ///< Hemisphere NMEA (deprecated, use NMEA)
        GPS_TYPE_UBLOX_RTK_BASE = 17,  ///< u-blox configured as RTK base station
        GPS_TYPE_UBLOX_RTK_ROVER = 18, ///< u-blox configured as RTK rover
        GPS_TYPE_MSP = 19,   ///< MSP protocol (for OSD integration)
        GPS_TYPE_ALLYSTAR = 20,  ///< AllyStar NMEA protocol
        GPS_TYPE_EXTERNAL_AHRS = 21,  ///< GPS data from external AHRS system
        GPS_TYPE_UAVCAN_RTK_BASE = 22,  ///< DroneCAN GPS as RTK base
        GPS_TYPE_UAVCAN_RTK_ROVER = 23, ///< DroneCAN GPS as RTK rover
        GPS_TYPE_UNICORE_NMEA = 24,  ///< Unicore NMEA protocol
        GPS_TYPE_UNICORE_MOVINGBASE_NMEA = 25, ///< Unicore moving baseline NMEA
        GPS_TYPE_SBF_DUAL_ANTENNA = 26, ///< Septentrio dual-antenna configuration
#if AP_SIM_GPS_ENABLED
        GPS_TYPE_SITL = 100, ///< Software-in-the-loop simulation GPS
#endif
    };

    // convenience methods for working out what general type an instance is:
    bool is_rtk_base(uint8_t instance) const;
    bool is_rtk_rover(uint8_t instance) const;

    // params for an instance:
    class Params {
    public:
        // Constructor
        Params(void);

        AP_Enum<GPS_Type> type;
        AP_Int8 gnss_mode;
        AP_Int16 rate_ms;   // this parameter should always be accessed using get_rate_ms()
        AP_Vector3f antenna_offset;
        AP_Int16 delay_ms;
        AP_Int8  com_port;
#if HAL_ENABLE_DRONECAN_DRIVERS
        AP_Int32 node_id;
        AP_Int32 override_node_id;
#endif
#if GPS_MOVING_BASELINE
        MovingBase mb_params;
#endif // GPS_MOVING_BASELINE

        static const struct AP_Param::GroupInfo var_info[];
    };

    /**
     * @enum GPS_Status
     * @brief GPS fix type and quality status codes
     * 
     * @details These status codes indicate the current GPS fix quality, from no
     * GPS detected to RTK Fixed (centimeter-level accuracy). Status values are
     * kept aligned with MAVLink GPS_FIX_TYPE by static_assert in AP_GPS.cpp.
     * 
     * @note Check status() >= FIX_3D before using GPS for navigation
     * @warning EKF initialization requires at least GPS_OK_FIX_3D
     */
    enum GPS_Status {
        NO_GPS = 0,                  ///< No GPS connected/detected (no valid messages)
        NO_FIX = 1,                  ///< Receiving valid GPS messages but no position lock
        GPS_OK_FIX_2D = 2,           ///< 2D position lock (lat/lon only, no altitude) - insufficient for flight
        GPS_OK_FIX_3D = 3,           ///< 3D position lock (lat/lon/alt) - minimum for arming
        GPS_OK_FIX_3D_DGPS = 4,      ///< 3D lock with SBAS differential corrections (~1m accuracy)
        GPS_OK_FIX_3D_RTK_FLOAT = 5, ///< 3D RTK Float solution (~10cm horizontal accuracy)
        GPS_OK_FIX_3D_RTK_FIXED = 6, ///< 3D RTK Fixed solution (cm-level accuracy, ambiguities resolved)
    };

    /**
     * @enum GPS_Engine_Setting
     * @brief GPS navigation engine dynamic platform model settings
     * 
     * @details Configures the GPS receiver's motion model for optimal performance
     * in different vehicle dynamics scenarios. The engine setting affects:
     * - Kalman filter tuning
     * - Maximum velocity/acceleration limits
     * - Position update algorithms
     * 
     * @note Not all GPS receivers support dynamic platform models (mainly u-blox)
     * @warning Using incorrect setting can degrade performance (e.g., PORTABLE on
     * high-speed aircraft will limit velocity estimates)
     */
    enum GPS_Engine_Setting {
        GPS_ENGINE_NONE        = -1, ///< No specific engine setting (use GPS default)
        GPS_ENGINE_PORTABLE    = 0,  ///< Portable mode (<2G, general purpose)
        GPS_ENGINE_STATIONARY  = 2,  ///< Stationary applications (minimal velocity)
        GPS_ENGINE_PEDESTRIAN  = 3,  ///< Pedestrian motion (<4 km/h typical)
        GPS_ENGINE_AUTOMOTIVE  = 4,  ///< Ground vehicles (<4G acceleration)
        GPS_ENGINE_SEA         = 5,  ///< Maritime applications (<2G)
        GPS_ENGINE_AIRBORNE_1G = 6,  ///< Aircraft <1G acceleration, <100 m/s vertical
        GPS_ENGINE_AIRBORNE_2G = 7,  ///< Aircraft <2G acceleration, <250 m/s vertical
        GPS_ENGINE_AIRBORNE_4G = 8   ///< Aircraft <4G acceleration, no velocity limit
    };

    /**
     * @enum GPS_Role
     * @brief GPS role assignment for automatic configuration in moving baseline setups
     * 
     * @details Defines the role of each GPS instance in moving baseline RTK
     * configurations where one GPS acts as a base station providing corrections
     * to the other (rover).
     */
    enum GPS_Role {
        GPS_ROLE_NORMAL,   ///< Standard GPS operation (no moving baseline)
        GPS_ROLE_MB_BASE,  ///< Moving baseline base station (provides RTCM corrections)
        GPS_ROLE_MB_ROVER, ///< Moving baseline rover (receives corrections, outputs heading)
    };

    /**
     * @enum CovarianceType
     * @brief GPS position covariance reporting capability types
     * 
     * @details Indicates the type of covariance/accuracy information the GPS
     * receiver can provide. Matches ROS2 sensor_msgs/msg/NavSatFix covariance types.
     * Higher types provide more detailed accuracy characterization.
     */
    enum class CovarianceType : uint8_t {
        UNKNOWN = 0,  ///< GPS does not report any accuracy metrics
        APPROXIMATED = 1,  ///< Accuracy approximated from HDOP/VDOP (typical for most GPS)
        DIAGONAL_KNOWN = 2, ///< Diagonal covariance components (east, north, up) reported
        KNOWN = 3, ///< Full 3x3 covariance matrix reported (rare, high-end receivers)
    };

    /**
     * @struct GPS_State
     * @brief Complete GPS state data filled by backend drivers
     * 
     * @details This structure contains all GPS measurement data reported by the
     * backend driver after parsing GPS messages. Fields are updated at GPS message
     * rate (typically 1-10Hz for position, 50Hz for availability checks).
     * 
     * **Coordinate Systems:**
     * - location: WGS84 geodetic (lat/lon degrees, altitude meters above ellipsoid or MSL)
     * - velocity: NED (North-East-Down) frame in m/s
     * - rtk_baseline: ECEF (Earth-Centered Earth-Fixed) or NED in millimeters
     * 
     * **Units:**
     * - Position: degrees (lat/lon), meters (altitude)
     * - Velocity: m/s (NED frame)
     * - Accuracy: meters (horizontal/vertical position), m/s (speed)
     * - DOP: dimensionless, scaled by 100 (value 155 = HDOP of 1.55)
     * - Time: milliseconds (GPS time of week), week number
     * - Course/heading: degrees (ground_course, gps_yaw), wrapped 0-360
     * 
     * @note All fields must be filled by backend driver. Optional fields use
     * have_* flags to indicate availability.
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:192-245
     */
    struct GPS_State {
        uint8_t instance; // the instance number of this GPS

        // all the following fields must all be filled by the backend driver
        GPS_Status status;                  ///< driver fix status
        uint32_t time_week_ms;              ///< GPS time (milliseconds from start of GPS week)
        uint16_t time_week;                 ///< GPS week number
        Location location;                  ///< last fix location
        float ground_speed;                 ///< ground speed in m/s
        float ground_course;                ///< ground course in degrees, wrapped 0-360
        float gps_yaw;                      ///< GPS derived yaw information, if available (degrees)
        uint32_t gps_yaw_time_ms;           ///< timestamp of last GPS yaw reading
        bool  gps_yaw_configured;           ///< GPS is configured to provide yaw
        uint16_t hdop;                      ///< horizontal dilution of precision, scaled by a factor of 100 (155 means the HDOP value is 1.55)
        uint16_t vdop;                      ///< vertical dilution of precision, scaled by a factor of 100 (155 means the VDOP value is 1.55)
        uint8_t num_sats;                   ///< Number of visible satellites
        Vector3f velocity;                  ///< 3D velocity in m/s, in NED format
        float speed_accuracy;               ///< 3D velocity RMS accuracy estimate in m/s
        float horizontal_accuracy;          ///< horizontal RMS accuracy estimate in m
        float vertical_accuracy;            ///< vertical RMS accuracy estimate in m
        float gps_yaw_accuracy;           ///< heading accuracy of the GPS in degrees
        bool have_vertical_velocity;      ///< does GPS give vertical velocity? Set to true only once available.
        bool have_speed_accuracy;         ///< does GPS give speed accuracy? Set to true only once available.
        bool have_horizontal_accuracy;    ///< does GPS give horizontal position accuracy? Set to true only once available.
        bool have_vertical_accuracy;      ///< does GPS give vertical position accuracy? Set to true only once available.
        bool have_gps_yaw;                ///< does GPS give yaw? Set to true only once available.
        bool have_gps_yaw_accuracy;       ///< does the GPS give a heading accuracy estimate? Set to true only once available
        float undulation;                   //<height that WGS84 is above AMSL at the current location
        bool have_undulation;               ///<do we have a value for the undulation
        uint32_t last_gps_time_ms;          ///< the system time we got the last GPS timestamp, milliseconds
        bool announced_detection;           ///< true once we have announced GPS has been seen to the user
        uint64_t last_corrected_gps_time_us;///< the system time we got the last corrected GPS timestamp, microseconds
        bool corrected_timestamp_updated;  ///< true if the corrected timestamp has been updated
        uint32_t lagged_sample_count;       ///< number of samples with 50ms more lag than expected

        // all the following fields must only all be filled by RTK capable backend drivers
        uint32_t rtk_time_week_ms;         ///< GPS Time of Week of last baseline in milliseconds
        uint16_t rtk_week_number;          ///< GPS Week Number of last baseline
        uint32_t rtk_age_ms;               ///< GPS age of last baseline correction in milliseconds  (0 when no corrections, 0xFFFFFFFF indicates overflow)
        uint8_t  rtk_num_sats;             ///< Current number of satellites used for RTK calculation
        uint8_t  rtk_baseline_coords_type; ///< Coordinate system of baseline. 0 == ECEF, 1 == NED
        int32_t  rtk_baseline_x_mm;        ///< Current baseline in ECEF x or NED north component in mm
        int32_t  rtk_baseline_y_mm;        ///< Current baseline in ECEF y or NED east component in mm
        int32_t  rtk_baseline_z_mm;        ///< Current baseline in ECEF z or NED down component in mm
        uint32_t rtk_accuracy;             ///< Current estimate of 3D baseline accuracy (receiver dependent, typical 0 to 9999)
        int32_t  rtk_iar_num_hypotheses;   ///< Current number of integer ambiguity hypotheses
        
        // UBX Relative Position and Heading message information
        float relPosHeading;               ///< Reported Heading in degrees
        float relPosLength;                ///< Reported Position horizontal distance in meters
        float relPosD;                     ///< Reported Vertical distance in meters
        float accHeading;                  ///< Reported Heading Accuracy in degrees
        uint32_t relposheading_ts;        ///< True if new data has been received since last time it was false
    };

    /**
     * @brief Initialize GPS subsystem and start detection on configured serial ports
     * 
     * @details Begins auto-baud rate probing and protocol detection on each
     * configured GPS instance. For each enabled GPS:
     * - Opens configured serial port (SERIAL3, SERIAL4, etc.)
     * - Starts auto-baud detection sequence (9600-460800 baud)
     * - Initializes protocol detection state machines
     * - Allocates backend driver when protocol identified
     * 
     * @note Call during vehicle setup() phase before main loop starts
     * @note Non-blocking - detection continues in background via update() calls
     * 
     * Source: libraries/AP_GPS/AP_GPS.cpp
     */
    void init();

    // ethod for APPPeriph to set the default type for the first GPS instance:
    void set_default_type_for_gps1(uint8_t default_type) {
        params[0].type.set_default(default_type);
    }

    /**
     * @brief Main GPS update function - reads backends, updates blending, selects primary
     * 
     * @details Processes GPS data from all backends and updates system state:
     * 1. Read available data from each GPS backend instance
     * 2. Update primary GPS selection based on fix quality, HDOP, satellite count
     * 3. Compute blended GPS solution if enabled (GPS_BLEND_MASK)
     * 4. Update timing information and detect message delays
     * 5. Perform logging if enabled
     * 6. Update MAVLink telemetry streams
     * 
     * Must be called periodically (typically at 10-50Hz) to process incoming data.
     * Called at GPS_MAX_RATE - typically 10Hz for position data availability,
     * but may be called at higher rates (50Hz) for data availability checks by EKF.
     * 
     * @return true if any GPS backend has new data available
     * 
     * @note Must be called from main thread with consistent timing for EKF integration
     * @warning Delayed or irregular calls will affect EKF performance and increase
     * position lag compensation errors
     * 
     * Source: libraries/AP_GPS/AP_GPS.cpp
     */
    void update(void);

    // Pass mavlink data to message handlers (for MAV type)
    void handle_msg(mavlink_channel_t chan, const mavlink_message_t &msg);
#if HAL_MSP_GPS_ENABLED
    void handle_msp(const MSP::msp_gps_data_message_t &pkt);
#endif
#if AP_EXTERNAL_AHRS_ENABLED
    // Retrieve the first instance ID that is configured as type GPS_TYPE_EXTERNAL_AHRS.
    // Can be used by external AHRS systems that only report one GPS to get the instance ID.
    // Returns true if an instance was found, false otherwise.
    bool get_first_external_instance(uint8_t& instance) const WARN_IF_UNUSED;
    void handle_external(const AP_ExternalAHRS::gps_data_message_t &pkt, const uint8_t instance);
#endif

    // Accessor functions

    // return number of active GPS sensors. Note that if the first GPS
    // is not present but the 2nd is then we return 2. Note that a blended
    // GPS solution is treated as an additional sensor.
    uint8_t num_sensors(void) const;

    // Return the index of the primary sensor which is the index of the sensor contributing to
    // the output. A blended solution is available as an additional instance
    uint8_t primary_sensor(void) const {
        return primary_instance;
    }

    /**
     * @brief Query GPS fix type and quality status
     * 
     * @details Returns current GPS fix status from NO_GPS to RTK_FIXED.
     * Status indicates position solution quality and determines whether
     * GPS data is suitable for navigation.
     * 
     * @param instance GPS instance number (0 = primary, 1 = secondary, etc.)
     * @return GPS_Status Fix type (NO_GPS, NO_FIX, FIX_2D, FIX_3D, DGPS, RTK_FLOAT, RTK_FIXED)
     * 
     * @note Check status() >= GPS_OK_FIX_3D before using GPS for navigation
     * @warning EKF initialization requires at least GPS_OK_FIX_3D. Vehicle cannot
     * arm without valid 3D fix on primary GPS.
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:287-295
     */
    GPS_Status status(uint8_t instance) const {
        if (_force_disable_gps && state[instance].status > NO_FIX) {
            return NO_FIX;
        }
        return state[instance].status;
    }
    
    /**
     * @brief Query primary GPS fix status
     * @return GPS_Status Fix type of primary GPS instance
     * @see status(uint8_t instance)
     */
    GPS_Status status(void) const {
        return status(primary_instance);
    }

    // return a single human-presentable character representing the
    // fix type.  For space-constrained human-readable displays
    char status_onechar(void) const {
        switch (status()) {
        case AP_GPS::NO_GPS:
            return ' ';
        case AP_GPS::NO_FIX:
            return '-';
        case AP_GPS::GPS_OK_FIX_2D:
            return '2';
        case AP_GPS::GPS_OK_FIX_3D:
            return '3';
        case AP_GPS::GPS_OK_FIX_3D_DGPS:
            return '4';
        case AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT:
            return '5';
        case AP_GPS::GPS_OK_FIX_3D_RTK_FIXED:
            return '6';
        }
        // should never reach here; compiler flags guarantees this.
        return '?';
    }

    // Query the highest status this GPS supports (always reports GPS_OK_FIX_3D for the blended GPS)
    GPS_Status highest_supported_status(uint8_t instance) const WARN_IF_UNUSED;

    /**
     * @brief Get current GPS position from specified instance
     * 
     * @details Returns the most recent GPS position fix. Position is in
     * WGS84 geodetic coordinates:
     * - Latitude: degrees (positive = North)
     * - Longitude: degrees (positive = East)  
     * - Altitude: meters (above MSL or ellipsoid based on GPS config)
     * 
     * @param instance GPS instance number (0 = primary, 1 = secondary)
     * @return Location structure containing lat/lon/alt
     * 
     * @note Returns last valid position even if GPS has temporarily lost lock.
     * Check status() >= FIX_3D before using for navigation.
     * @warning Position continues to report last fix during GPS outages. Monitor
     * last_message_time_ms() to detect stale data.
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:324-329
     */
    const Location &location(uint8_t instance) const {
        return state[instance].location;
    }
    
    /**
     * @brief Get current GPS position from primary GPS
     * @return Location structure (lat/lon/alt) from primary GPS
     * @see location(uint8_t instance)
     */
    const Location &location() const {
        return location(primary_instance);
    }

    // get the difference between WGS84 and AMSL. A positive value means
    // the AMSL height is higher than WGS84 ellipsoid height
    bool get_undulation(uint8_t instance, float &undulation) const;

    // get the difference between WGS84 and AMSL. A positive value means
    // the AMSL height is higher than WGS84 ellipsoid height
    bool get_undulation(float &undulation) const {
        return get_undulation(primary_instance, undulation);
    }

    // report speed accuracy
    bool speed_accuracy(uint8_t instance, float &sacc) const;
    bool speed_accuracy(float &sacc) const {
        return speed_accuracy(primary_instance, sacc);
    }

    bool horizontal_accuracy(uint8_t instance, float &hacc) const;
    bool horizontal_accuracy(float &hacc) const {
        return horizontal_accuracy(primary_instance, hacc);
    }

    bool vertical_accuracy(uint8_t instance, float &vacc) const;
    bool vertical_accuracy(float &vacc) const {
        return vertical_accuracy(primary_instance, vacc);
    }

    CovarianceType position_covariance(const uint8_t instance, Matrix3f& cov) const WARN_IF_UNUSED;

    /**
     * @brief Get current 3D velocity from GPS in NED frame
     * 
     * @details Returns velocity vector in North-East-Down frame:
     * - velocity.x: North velocity in m/s (positive = moving north)
     * - velocity.y: East velocity in m/s (positive = moving east)
     * - velocity.z: Down velocity in m/s (positive = descending)
     * 
     * @param instance GPS instance number
     * @return Vector3f Velocity in NED frame (m/s)
     * 
     * @note Returns (0,0,0) if no velocity data available. Check
     * have_vertical_velocity() to verify if Z component is valid.
     * @note Typical GPS velocity accuracy: 0.1-0.5 m/s (dependent on fix quality)
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:360-365
     */
    const Vector3f &velocity(uint8_t instance) const {
        return state[instance].velocity;
    }
    
    /**
     * @brief Get velocity from primary GPS
     * @return Vector3f Velocity NED (m/s) from primary GPS
     * @see velocity(uint8_t instance)
     */
    const Vector3f &velocity() const {
        return velocity(primary_instance);
    }

    /**
     * @brief Get 2D ground speed (horizontal velocity magnitude)
     * 
     * @details Computed from GPS velocity: sqrt(vN² + vE²)
     * Represents horizontal speed over ground regardless of direction.
     * 
     * @param instance GPS instance number
     * @return Ground speed in m/s
     * 
     * @note This is 2D speed only (does not include vertical velocity component)
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:368-373
     */
    float ground_speed(uint8_t instance) const {
        return state[instance].ground_speed;
    }
    
    /**
     * @brief Get ground speed from primary GPS
     * @return Ground speed in m/s from primary GPS
     */
    float ground_speed() const {
        return ground_speed(primary_instance);
    }

    /**
     * @brief Get ground speed in centimeters per second
     * @return Ground speed in cm/s (for legacy code compatibility)
     */
    uint32_t ground_speed_cm(void) const {
        return ground_speed() * 100;
    }

    /**
     * @brief Get ground course (direction of travel over ground)
     * 
     * @details Returns heading/course in degrees (0-360):
     * - 0° = North
     * - 90° = East
     * - 180° = South
     * - 270° = West
     * 
     * @param instance GPS instance number
     * @return Course over ground in degrees, wrapped to 0-360 range
     * 
     * @note Returns 0 if vehicle is stationary (speed below threshold)
     * @note Course is direction of motion, not heading (vehicle may be pointing
     * different direction than travel direction, especially in wind)
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:381-386
     */
    float ground_course(uint8_t instance) const {
        return state[instance].ground_course;
    }
    
    /**
     * @brief Get ground course from primary GPS
     * @return Course in degrees from primary GPS
     */
    float ground_course() const {
        return ground_course(primary_instance);
    }
    
    /**
     * @brief Get ground course in centidegrees
     * @param instance GPS instance number
     * @return Course in centidegrees (0-36000)
     */
    int32_t ground_course_cd(uint8_t instance) const {
        return ground_course(instance) * 100;
    }
    
    /**
     * @brief Get ground course from primary GPS in centidegrees
     * @return Course in centidegrees (0-36000)
     */
    int32_t ground_course_cd() const {
        return ground_course_cd(primary_instance);
    }

    // yaw in degrees if available
    bool gps_yaw_deg(uint8_t instance, float &yaw_deg, float &accuracy_deg, uint32_t &time_ms) const;
    bool gps_yaw_deg(float &yaw_deg, float &accuracy_deg, uint32_t &time_ms) const {
        return gps_yaw_deg(primary_instance, yaw_deg, accuracy_deg, time_ms);
    }

    /**
     * @brief Get number of satellites used in position solution
     * 
     * @details Returns count of satellites currently contributing to the
     * position fix. More satellites generally indicates better accuracy and
     * geometry (lower HDOP).
     * 
     * @param instance GPS instance number
     * @return Satellite count (typically 6-20 satellites visible)
     * 
     * @note Minimum 4 satellites required for 3D fix. 6+ satellites recommended
     * for good accuracy. Counts >12 are typical with multi-constellation GNSS
     * (GPS+GLONASS+Galileo+BeiDou).
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:402-407
     */
    uint8_t num_sats(uint8_t instance) const {
        return state[instance].num_sats;
    }
    
    /**
     * @brief Get satellite count from primary GPS
     * @return Number of satellites used in solution
     */
    uint8_t num_sats() const {
        return num_sats(primary_instance);
    }

    // GPS time of week in milliseconds
    uint32_t time_week_ms(uint8_t instance) const {
        return state[instance].time_week_ms;
    }
    uint32_t time_week_ms() const {
        return time_week_ms(primary_instance);
    }

    // GPS week
    uint16_t time_week(uint8_t instance) const {
        return state[instance].time_week;
    }
    uint16_t time_week() const {
        return time_week(primary_instance);
    }

    /**
     * @brief Get Horizontal Dilution of Precision (HDOP)
     * 
     * @details HDOP indicates geometric quality of satellite constellation for
     * horizontal position accuracy. Lower values indicate better geometry.
     * Value is scaled by 100 internally (returned value 155 means HDOP of 1.55).
     * 
     * HDOP quality guide:
     * - <1.0 (100): Ideal - highest precision available
     * - 1-2 (100-200): Excellent - high precision
     * - 2-5 (200-500): Good - acceptable for navigation
     * - 5-10 (500-1000): Moderate - usable but degraded
     * - >10 (>1000): Poor - position accuracy significantly degraded
     * 
     * @param instance GPS instance number
     * @return HDOP scaled by 100 (dimensionless)
     * 
     * @note HDOP is a geometric factor, not an accuracy measure. Actual position
     * error = HDOP × User Equivalent Range Error (UERE)
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:426-431
     */
    uint16_t get_hdop(uint8_t instance) const {
        return state[instance].hdop;
    }
    
    /**
     * @brief Get HDOP from primary GPS
     * @return HDOP scaled by 100
     */
    uint16_t get_hdop() const {
        return get_hdop(primary_instance);
    }

    /**
     * @brief Get Vertical Dilution of Precision (VDOP)
     * 
     * @details VDOP indicates geometric quality for vertical (altitude) position.
     * Interpretation same as HDOP. Vertical accuracy typically worse than
     * horizontal (VDOP > HDOP) due to satellite geometry limitations.
     * Value scaled by 100 internally (155 = VDOP of 1.55).
     * 
     * @param instance GPS instance number
     * @return VDOP scaled by 100 (dimensionless)
     * 
     * @note Altitude measurements more sensitive to satellite geometry than
     * horizontal position. VDOP typically 1.5-2x larger than HDOP.
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:434-439
     */
    uint16_t get_vdop(uint8_t instance) const {
        return state[instance].vdop;
    }
    
    /**
     * @brief Get VDOP from primary GPS
     * @return VDOP scaled by 100
     */
    uint16_t get_vdop() const {
        return get_vdop(primary_instance);
    }

    /**
     * @brief Get system time of last GPS position fix
     * 
     * @details Returns system millisecond timestamp when GPS last reported
     * a valid position fix. Used to calculate dead reckoning drift when
     * GPS temporarily loses lock.
     * 
     * @param instance GPS instance number
     * @return System time in milliseconds (AP_HAL::millis())
     * 
     * @note Different from last_message_time_ms() - this updates only on
     * position fixes, while last_message_time_ms() updates on any GPS message
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:441-448
     */
    uint32_t last_fix_time_ms(uint8_t instance) const {
        return timing[instance].last_fix_time_ms;
    }
    
    /**
     * @brief Get last fix time from primary GPS
     * @return System time in milliseconds
     */
    uint32_t last_fix_time_ms(void) const {
        return last_fix_time_ms(primary_instance);
    }

    /**
     * @brief Get system time when GPS last sent any message
     * 
     * @details Returns timestamp of most recent GPS message processing.
     * Used to detect GPS timeouts and stale data. GPS considered unhealthy
     * if no messages received for >5 seconds.
     * 
     * @param instance GPS instance number
     * @return System time in milliseconds of last message
     * 
     * @note Updates on any GPS message, not just position fixes. Use to
     * detect GPS communication failures vs. loss of position lock.
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:450-457
     */
    uint32_t last_message_time_ms(uint8_t instance) const {
        return timing[instance].last_message_time_ms;
    }
    
    /**
     * @brief Get last message time from primary GPS
     * @return System time in milliseconds
     */
    uint32_t last_message_time_ms(void) const {
        return last_message_time_ms(primary_instance);
    }

    /**
     * @brief Get time interval between last two GPS position reports
     * 
     * @details Returns delta time between the two most recent position
     * updates. Indicates actual GPS update rate (typically 5-10Hz).
     * 
     * @param instance GPS instance number
     * @return Delta time in milliseconds
     * 
     * @note Useful for detecting irregular GPS update rates which can
     * indicate communication problems or GPS receiver issues
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:459-465
     */
    uint16_t last_message_delta_time_ms(uint8_t instance) const {
        return timing[instance].delta_time_ms;
    }
    
    /**
     * @brief Get message delta time from primary GPS
     * @return Delta time in milliseconds
     */
    uint16_t last_message_delta_time_ms(void) const {
        return last_message_delta_time_ms(primary_instance);
    }

    /**
     * @brief Check if GPS provides valid vertical velocity
     * 
     * @details Some GPS receivers do not report vertical velocity (down component).
     * Returns true if velocity.z is valid and can be used for navigation.
     * 
     * @param instance GPS instance number
     * @return true if vertical velocity (Down component) is available
     * 
     * @note Most modern GPS receivers provide 3D velocity. Older or basic
     * receivers may only provide horizontal velocity.
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:467-473
     */
    bool have_vertical_velocity(uint8_t instance) const {
        return state[instance].have_vertical_velocity;
    }
    
    /**
     * @brief Check if primary GPS provides vertical velocity
     * @return true if vertical velocity available
     */
    bool have_vertical_velocity(void) const {
        return have_vertical_velocity(primary_instance);
    }

    /**
     * @brief Check if GPS currently provides yaw/heading measurement
     * 
     * @details Returns true if GPS is providing yaw angle (typically from
     * dual-antenna GPS or moving baseline configuration). Yaw from GPS
     * eliminates need for compass and provides better heading accuracy.
     * 
     * @param instance GPS instance number
     * @return true if GPS yaw is currently available
     * 
     * @note GPS yaw requires dual-antenna setup or moving baseline with
     * two GPS receivers. Yaw accuracy typically 0.1-0.5 degrees.
     * @warning GPS yaw only valid when vehicle is moving (>1 m/s typically)
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:475-481
     */
    bool have_gps_yaw(uint8_t instance) const {
        return !_force_disable_gps_yaw && state[instance].have_gps_yaw;
    }
    
    /**
     * @brief Check if primary GPS provides yaw
     * @return true if GPS yaw available
     */
    bool have_gps_yaw(void) const {
        return have_gps_yaw(primary_instance);
    }

    /**
     * @brief Check if GPS is configured to provide yaw
     * 
     * @details Returns true if GPS hardware is configured for yaw output
     * (dual-antenna or moving baseline), even if yaw is not currently
     * available (e.g., during initialization or when stationary).
     * 
     * @param instance GPS instance number
     * @return true if GPS is configured to provide yaw eventually
     * 
     * @note Use to distinguish between "GPS doesn't support yaw" vs.
     * "GPS supports yaw but not available yet"
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:483-488
     */
    bool have_gps_yaw_configured(uint8_t instance) const {
        return state[instance].gps_yaw_configured;
    }
    
    /**
     * @brief Get expected GPS measurement lag time
     * 
     * @details Returns the expected delay between GPS measurement time and
     * when position/velocity data arrives at autopilot. EKF uses this to
     * compensate for lag and improve fusion accuracy.
     * 
     * Lag depends on:
     * - GPS receiver processing time (50-200ms typical)
     * - Serial transmission time (minimal at high baud rates)
     * - Autopilot processing delays
     * 
     * @param[in] instance GPS instance number
     * @param[out] lag_sec Lag time in seconds (typically 0.05-0.25s)
     * @return true if lag value is known or configured
     * 
     * @note Incorrect lag values degrade EKF performance. If auto-detected
     * lag unavailable, set GPS_DELAY_MS parameter manually.
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:490-495
     */
    bool get_lag(uint8_t instance, float &lag_sec) const;
    
    /**
     * @brief Get lag time for primary GPS
     * @param[out] lag_sec Lag time in seconds
     * @return true if lag known
     */
    bool get_lag(float &lag_sec) const {
        return get_lag(primary_instance, lag_sec);
    }

    /**
     * @brief Get GPS antenna position offset from vehicle center of gravity
     * 
     * @details Returns 3D offset vector in NED frame (meters):
     * - X: Forward offset (positive = ahead of CG)
     * - Y: Right offset (positive = right of CG)
     * - Z: Down offset (positive = below CG)
     * 
     * EKF uses this offset to correct GPS position measurements to vehicle CG.
     * 
     * @param instance GPS instance number
     * @return Vector3f Antenna offset in meters (NED frame)
     * 
     * @note Set via GPS_POS1_X/Y/Z, GPS_POS2_X/Y/Z parameters. Critical for
     * accuracy when GPS antenna is far from CG (e.g., tail-mounted GPS on plane).
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:497-498
     */
    const Vector3f &get_antenna_offset(uint8_t instance) const;

    /**
     * @brief Lock/unlock GPS serial port for exclusive external access
     * 
     * @details Allows temporary exclusive access to GPS serial port by external
     * applications (e.g., u-center configuration tool). While locked, GPS driver
     * stops reading from port.
     * 
     * @param instance GPS instance number
     * @param locked true to lock port (disable driver), false to unlock
     * 
     * @warning While port is locked, GPS data is unavailable and vehicle cannot arm
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:500-501
     */
    void lock_port(uint8_t instance, bool locked);

    /**
     * @brief Send MAVLink GPS_RAW_INT message for primary GPS
     * @param chan MAVLink channel to send on
     * Source: libraries/AP_GPS/AP_GPS.cpp
     */
    void send_mavlink_gps_raw(mavlink_channel_t chan);
    
    /**
     * @brief Send MAVLink GPS2_RAW message for secondary GPS
     * @param chan MAVLink channel to send on
     * Source: libraries/AP_GPS/AP_GPS.cpp
     */
    void send_mavlink_gps2_raw(mavlink_channel_t chan);

    /**
     * @brief Send MAVLink GPS_RTK message with RTK status
     * @param chan MAVLink channel to send on
     * @param inst GPS instance number
     * Source: libraries/AP_GPS/AP_GPS.cpp
     */
    void send_mavlink_gps_rtk(mavlink_channel_t chan, uint8_t inst);

    /**
     * @brief Check if any GPS is still unconfigured during initialization
     * 
     * @details Returns true during GPS detection/configuration phase. Used to
     * determine when all GPS instances are fully initialized and ready.
     * 
     * @param[out] instance Instance number of first unconfigured GPS
     * @return true if at least one GPS is still configuring
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:509-510
     */
    bool first_unconfigured_gps(uint8_t &instance) const WARN_IF_UNUSED;
    
    /**
     * @brief Send MAVLink error message about GPS configuration failure
     * 
     * @details Broadcasts STATUSTEXT message describing why GPS failed to
     * configure (e.g., wrong baud rate, unsupported protocol, hardware issue).
     * 
     * Source: libraries/AP_GPS/AP_GPS.cpp
     */
    void broadcast_first_configuration_failure_reason(void) const;

    /**
     * @brief Pre-arm check that all GPS instances are consistent
     * 
     * @details Verifies multiple GPS receivers agree on position within
     * acceptable tolerance. Catches GPS failures, interference, or
     * configuration errors before flight.
     * 
     * @param[out] distance Maximum distance between any two GPS instances (meters)
     * @return true if all GPS positions are consistent
     * 
     * @note Typical consistency threshold: ~50 meters. Larger discrepancies
     * indicate GPS problem requiring investigation.
     * @warning Inconsistent GPS can indicate antenna placement issue, interference,
     * or one GPS receiving incorrect RTCM corrections
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:513-514
     */
    bool all_consistent(float &distance) const;

    /**
     * @brief Start sending initialization blob to GPS (backend use only)
     * @param instance GPS instance number
     * @note Internal method for backend drivers
     */
    void send_blob_start(uint8_t instance);
    
    /**
     * @brief Start sending custom initialization blob to GPS
     * @param instance GPS instance number
     * @param _blob Pointer to initialization data
     * @param size Size of initialization data in bytes
     * @note Internal method for backend drivers
     */
    void send_blob_start(uint8_t instance, const char *_blob, uint16_t size);
    
    /**
     * @brief Continue sending initialization blob (called periodically)
     * @param instance GPS instance number
     * @note Internal method for backend drivers
     */
    void send_blob_update(uint8_t instance);

    /**
     * @brief Get GPS fix time as Unix epoch timestamp
     * 
     * @details Returns last GPS position fix time as microseconds since
     * Unix epoch (1/1/1970 00:00:00 UTC). Converts from GPS time (week +
     * time-of-week) to Unix time.
     * 
     * @param instance GPS instance number
     * @return Unix timestamp in microseconds
     * 
     * @note GPS time is ahead of UTC by GPS leap seconds (currently 18s).
     * Conversion accounts for leap seconds.
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:521-525
     */
    uint64_t time_epoch_usec(uint8_t instance) const;
    
    /**
     * @brief Get fix time from primary GPS as Unix epoch
     * @return Unix timestamp in microseconds
     */
    uint64_t time_epoch_usec(void) const {
        return time_epoch_usec(primary_instance);
    }

    /**
     * @brief Get time of last GPS message as Unix epoch timestamp
     * 
     * @details Returns timestamp of most recent GPS message (any type, not
     * just position fixes) as Unix epoch microseconds.
     * 
     * @param instance GPS instance number
     * @return Unix timestamp in microseconds
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:527-530
     */
    uint64_t last_message_epoch_usec(uint8_t instance) const;
    
    /**
     * @brief Get last message time from primary GPS as Unix epoch
     * @return Unix timestamp in microseconds
     */
    uint64_t last_message_epoch_usec() const {
        return last_message_epoch_usec(primary_instance);
    }

    /**
     * @brief Convert GPS time (week + milliseconds) to Unix epoch
     * 
     * @details Static utility for converting GPS time format to Unix timestamp.
     * GPS time is weeks since GPS epoch (6 January 1980) plus milliseconds
     * within week.
     * 
     * @param gps_week GPS week number
     * @param gps_ms Milliseconds into GPS week
     * @return Unix epoch timestamp in milliseconds
     * 
     * @note Accounts for GPS leap seconds (18 seconds as of 2024)
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:532-533
     */
    static uint64_t istate_time_to_epoch_ms(uint16_t gps_week, uint32_t gps_ms);

    static const struct AP_Param::GroupInfo var_info[];

#if HAL_LOGGING_ENABLED
    void Write_AP_Logger_Log_Startup_messages();
#endif

    /**
     * @brief Set LOG_BITMASK bit for GPS logging
     * @param bit Bit position in LOG_BITMASK parameter
     * @note Internal configuration method
     */
    void set_log_gps_bit(uint32_t bit) { _log_gps_bit = bit; }

    /**
     * @brief Check if GPS is healthy and providing reliable data
     * 
     * @details GPS considered healthy if:
     * - GPS instance exists and is configured
     * - Receiving updates at >4Hz (position or status messages)
     * - Fix quality sufficient (typically >= FIX_3D)
     * - No recent timeouts (messages within last 5 seconds)
     * - Backend-specific health checks pass
     * 
     * @param instance GPS instance number
     * @return true if GPS is healthy and data is reliable
     * 
     * @note Used by EKF to determine if GPS data should be fused. Unhealthy
     * GPS will be excluded from navigation solution.
     * @warning If all GPS instances are unhealthy, vehicle will fail pre-arm
     * checks and cannot be armed
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:544-547
     */
    bool is_healthy(uint8_t instance) const;
    
    /**
     * @brief Check if primary GPS is healthy
     * @return true if primary GPS is healthy
     */
    bool is_healthy(void) const { return is_healthy(primary_instance); }

    /**
     * @brief Perform final GPS arming preparation
     * 
     * @details Called immediately before arming to perform final GPS state
     * transitions and checks. Ensures GPS is in optimal state for flight.
     * 
     * @return true if all GPS instances are ready for arming
     * 
     * @note This is distinct from pre_arm_checks() - this performs final
     * state changes, while pre_arm_checks() only validates state.
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:549-550
     */
    bool prepare_for_arming(void);

    /**
     * @brief Execute pre-arm GPS health and consistency checks
     * 
     * @details Comprehensive GPS validation before allowing vehicle to arm:
     * - At least one GPS with FIX_3D or better
     * - HDOP within acceptable limits (<= GPS_HDOP_GOOD parameter)
     * - Sufficient satellites (>= GPS_MIN_SATS parameter)
     * - Multiple GPS positions are consistent (<50m discrepancy)
     * - No excessive drift or glitches detected
     * - Backend-specific checks pass
     * 
     * @param[out] failure_msg Buffer for human-readable failure description
     * @param[in] failure_msg_len Size of failure_msg buffer
     * @return true if all pre-arm checks pass, false with failure message
     * 
     * @warning Vehicle will not arm if this returns false. Failure message
     * displayed to pilot via GCS. Common failures: insufficient satellites,
     * high HDOP, GPS position inconsistency, no 3D fix.
     * 
     * @note Called repeatedly during pre-arm check sequence. Must be fast
     * and deterministic.
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:552-555
     */
    bool pre_arm_checks(char failure_msg[], uint16_t failure_msg_len);

    /**
     * @brief Check if GPS logging is functioning correctly
     * 
     * @details Verifies GPS raw data logging is working (if enabled).
     * Used to detect SD card full or logging system failures.
     * 
     * @return false if logging is enabled but failing
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:557-558
     */
    bool logging_failed(void) const;

    /**
     * @brief Check if GPS raw data logging is enabled
     * @return true if raw GPS data logging is configured
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:560
     */
    bool logging_present(void) const { return _raw_data != 0; }
    /**
     * @brief Check if GPS logging is enabled
     * @return true if raw GPS data logging enabled
     */
    bool logging_enabled(void) const { return _raw_data != 0; }

    /**
     * @brief Force disable GPS for failure testing
     * 
     * @details Allows disabling GPS in flight for testing GPS failure modes
     * and failsafe behaviors. GPS will report NO_FIX even if actually locked.
     * 
     * @param disable true to disable GPS, false to re-enable
     * 
     * @warning For testing only. Disabling GPS in flight triggers GPS failsafe
     * and may result in loss of position control.
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:563-566
     */
    void force_disable(bool disable) {
        _force_disable_gps = disable;
    }

    /**
     * @brief Force disable GPS yaw for failure testing
     * 
     * @details Disables GPS-based yaw/heading while keeping position valid.
     * For testing compass fallback behavior.
     * 
     * @param disable true to disable GPS yaw, false to re-enable
     * 
     * @warning For testing only. May affect heading accuracy if compass
     * not properly calibrated.
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:568-571
     */
    void set_force_disable_yaw(bool disable) {
        _force_disable_gps_yaw = disable;
    }

    /**
     * @brief Handle fragmented RTCM correction data
     * 
     * @details Processes RTCM3 data that may arrive fragmented across multiple
     * packets. Reassembles fragments and injects complete RTCM messages to GPS.
     * 
     * @param flags Fragment flags (first/last fragment indicators)
     * @param data RTCM data bytes
     * @param len Length of data
     * 
     * @note Used for RTCM injection via MAVLink GPS_RTCM_DATA messages
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:573-574
     */
    void handle_gps_rtcm_fragment(uint8_t flags, const uint8_t *data, uint8_t len);

    /**
     * @brief Get configured GPS type for instance
     * 
     * @details Returns GPS_TYPE parameter value for specified instance
     * (GPS_TYPE, GPS_TYPE2, etc.). Indicates which protocol driver is
     * configured (AUTO, UBLOX, NMEA, SBF, etc.).
     * 
     * @param instance GPS instance number
     * @return GPS_Type Configured GPS type
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:576-579
     */
    GPS_Type get_type(uint8_t instance) const {
        return instance>=ARRAY_SIZE(params) ? GPS_Type::GPS_TYPE_NONE : params[instance].type;
    }

    /**
     * @brief Get GPS time of week (iTOW) in milliseconds
     * 
     * @details Returns GPS time-of-week if supported by receiver. Time is
     * milliseconds since start of current GPS week (weeks start Sunday 00:00).
     * 
     * @param instance GPS instance number
     * @return iTOW in milliseconds, or 0 if not supported
     * 
     * @note u-blox and most modern receivers provide iTOW. Used for precise
     * time synchronization between multiple sensors.
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:581-582
     */
    uint32_t get_itow(uint8_t instance) const;

    /**
     * @brief Get GPS receiver error codes
     * 
     * @details Returns receiver-specific error/status codes if available.
     * Error codes are GPS-type dependent (u-blox, Septentrio, etc. each
     * have different error reporting formats).
     * 
     * @param instance GPS instance number
     * @param[out] error_codes Receiver error code bitmask
     * @return true if error codes available
     * 
     * @note Used for advanced diagnostics. Check GPS manufacturer documentation
     * for error code meanings.
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:584-585
     */
    bool get_error_codes(uint8_t instance, uint32_t &error_codes) const;
    
    /**
     * @brief Get error codes from primary GPS
     * @param[out] error_codes Receiver error code bitmask
     * @return true if error codes available
     */
    bool get_error_codes(uint32_t &error_codes) const { return get_error_codes(primary_instance, error_codes); }

    /**
     * @enum SBAS_Mode
     * @brief SBAS (Satellite-Based Augmentation System) configuration mode
     * 
     * @details Controls whether GPS uses SBAS corrections for improved accuracy.
     * SBAS includes systems like WAAS (North America), EGNOS (Europe), MSAS (Japan).
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:587-591
     */
    enum class SBAS_Mode : int8_t {
        Disabled = 0,       ///< SBAS disabled - no differential corrections
        Enabled = 1,        ///< SBAS enabled - use differential corrections if available
        DoNotChange = 2,    ///< Leave SBAS configuration as factory default
    };

#if GPS_MOVING_BASELINE
    /**
     * @brief Inject moving baseline (MBL) data from rover to base
     * 
     * @details Used in moving baseline RTK configurations where two GPS receivers
     * on same vehicle provide centimeter-accurate relative positioning and heading.
     * Injects data from rover GPS to base GPS.
     * 
     * @param data MBL data bytes
     * @param length Data length
     * 
     * @note Only available when GPS_MOVING_BASELINE enabled (requires 2+ GPS)
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:595
     */
    void inject_MBL_data(uint8_t* data, uint16_t length);
    
    /**
     * @brief Get relative position heading from moving baseline
     * 
     * @details Retrieves heading computed from relative position between two GPS
     * antennas. Provides compass-independent heading with RTK-level accuracy.
     * 
     * @param[out] timestamp Time of measurement (milliseconds)
     * @param[out] relPosHeading Heading in degrees (0-360)
     * @param[out] relPosLength Distance between antennas in meters
     * @param[out] relPosD Relative down position in meters
     * @param[out] accHeading Heading accuracy estimate in degrees
     * @return true if valid moving baseline heading available
     * 
     * @note Requires dual RTK GPS in moving baseline configuration. Antenna
     * separation must be at least 30cm for reliable heading.
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:596
     */
    bool get_RelPosHeading(uint32_t &timestamp, float &relPosHeading, float &relPosLength, float &relPosD, float &accHeading) WARN_IF_UNUSED;
    
    /**
     * @brief Get RTCM3 data for moving baseline transmission
     * 
     * @details Retrieves RTCM3 correction data to be sent from base to rover
     * in moving baseline configuration.
     * 
     * @param[out] bytes Pointer to RTCM3 data buffer
     * @param[out] len Length of RTCM3 data
     * @return true if RTCM3 data available
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:597
     */
    bool get_RTCMV3(const uint8_t *&bytes, uint16_t &len);
    
    /**
     * @brief Clear buffered RTCM3 data
     * @details Clears RTCM3 buffer after successful transmission
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:598
     */
    void clear_RTCMV3();
#endif // GPS_MOVING_BASELINE

#if !AP_GPS_BLENDED_ENABLED
    /**
     * @brief Get auto-switch configuration
     * @return Auto-switch mode setting
     */
    uint8_t get_auto_switch_type() const { return _auto_switch; }
#endif

    /**
     * @brief Inject raw binary data to GPS receiver
     * 
     * @details Sends raw binary data directly to GPS receiver. Used for:
     * - RTCM3 correction data injection for RTK
     * - Configuration commands
     * - Firmware updates
     * - Custom protocol data
     * 
     * Data routed to GPS instance configured by GPS_INJECT_TO parameter.
     * 
     * @param data Raw binary data bytes
     * @param len Length of data
     * 
     * @note Most commonly used for RTCM injection from NTRIP caster or base station
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:606
     */
    void inject_data(const uint8_t *data, uint16_t len);

protected:

    // Configuration parameters (GPS_TYPE, GPS_RATE_MS, etc. via AP_Param)
    Params params[GPS_MAX_INSTANCES];           ///< Per-instance GPS parameters
    AP_Int8 _navfilter;                         ///< Navigation filter setting (GPS_NAVFILTER)
    AP_Int8 _auto_switch;                       ///< Automatic GPS switching mode (GPS_AUTO_SWITCH)
    AP_Int16 _sbp_logmask;                      ///< SBP (Swift) logging mask (GPS_SBP_LOGMASK)
    AP_Int8 _inject_to;                         ///< Which GPS receives injected data (GPS_INJECT_TO)
    uint32_t _last_instance_swap_ms;            ///< Time of last primary GPS switch
    AP_Enum<SBAS_Mode> _sbas_mode;              ///< SBAS configuration mode (GPS_SBAS_MODE)
    AP_Int8 _min_elevation;                     ///< Minimum satellite elevation angle (GPS_MIN_ELEV)
    AP_Int8 _raw_data;                          ///< Raw data logging enable (GPS_RAW_DATA)
    AP_Int8 _save_config;                       ///< Save GPS config to flash (GPS_SAVE_CFG)
    AP_Int8 _auto_config;                       ///< Auto-configure GPS (GPS_AUTO_CONFIG)
    AP_Int8 _blend_mask;                        ///< GPS blending mask (GPS_BLEND_MASK)
    AP_Int16 _driver_options;                   ///< Driver-specific options (GPS_DRV_OPTIONS)
    AP_Int8 _primary;                           ///< Manually select primary GPS (GPS_PRIMARY)

    uint32_t _log_gps_bit = -1;                 ///< Logging mask bit for GPS data

    /**
     * @enum DriverOptions
     * @brief Driver-specific option flags (GPS_DRV_OPTIONS bitmask)
     * 
     * @details Controls GPS-type and hardware-specific behaviors via
     * GPS_DRV_OPTIONS parameter bitmask.
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:628-637
     */
    enum DriverOptions : int16_t {
        UBX_MBUseUart2    = (1U << 0U),         ///< u-blox: Use UART2 for moving baseline
        SBF_UseBaseForYaw = (1U << 1U),         ///< Septentrio: Use base GPS for yaw
        UBX_Use115200     = (1U << 2U),         ///< u-blox: Use 115200 baud (default 230400)
        UAVCAN_MBUseDedicatedBus  = (1 << 3U),  ///< DroneCAN: Use dedicated bus for moving baseline
        HeightEllipsoid   = (1U << 4),          ///< Report height above ellipsoid (not MSL)
        GPSL5HealthOverride = (1U << 5),        ///< Override GPS L5 band health check
        AlwaysRTCMDecode = (1U << 6),           ///< Always decode RTCM from GPS output
        DisableRTCMDecode = (1U << 7),          ///< Never decode RTCM from GPS output
    };

    /**
     * @brief Check if driver option is set
     * @param option DriverOptions flag to check
     * @return true if option enabled in GPS_DRV_OPTIONS
     */
    bool option_set(const DriverOptions option) const {
        return (uint8_t(_driver_options.get()) & uint8_t(option)) != 0;
    }

private:
    static AP_GPS *_singleton;
    HAL_Semaphore rsem;

    // returns the desired gps update rate in milliseconds
    // this does not provide any guarantee that the GPS is updating at the requested
    // rate it is simply a helper for use in the backends for determining what rate
    // they should be configuring the GPS to run at
    uint16_t get_rate_ms(uint8_t instance) const;

    /**
     * @struct GPS_timing
     * @brief GPS timing and update rate tracking
     * 
     * @details Tracks timing information for each GPS instance to monitor
     * update rates, detect delays, and compute average update intervals.
     * Used for health monitoring and adaptive filtering.
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:654-669
     */
    struct GPS_timing {
        uint32_t last_fix_time_ms;              ///< System time of last position fix (milliseconds)
        uint32_t last_message_time_ms;          ///< System time of last GPS message (milliseconds)
        uint16_t delta_time_ms;                 ///< Time between last two GPS updates (milliseconds)
        uint8_t delayed_count;                  ///< Count of delayed/missed updates
        float average_delta_ms;                 ///< Average time between updates (milliseconds)
    };
    
    // Note: Array sized for GPS_MAX_INSTANCES to allow additional instance for blended data
    GPS_timing timing[GPS_MAX_INSTANCES];       ///< Timing data for each GPS instance
    GPS_State state[GPS_MAX_INSTANCES];         ///< State data for each GPS instance
    AP_GPS_Backend *drivers[GPS_MAX_INSTANCES]; ///< Backend driver pointers
    AP_HAL::UARTDriver *_port[GPS_MAX_RECEIVERS]; ///< Serial port for each GPS receiver

    /// Primary GPS instance (selected based on fix quality, satellites, HDOP)
    uint8_t primary_instance;

    /// Number of GPS instances currently active
    uint8_t num_instances;

    /// Bitmask of serial ports that have been locked to a specific GPS type
    uint8_t locked_ports;

    /**
     * @struct detect_state
     * @brief Auto-detection state machine for each GPS instance
     * 
     * @details Tracks auto-detection progress through baud rate probing and
     * protocol detection. When GPS_TYPE set to AUTO, cycles through supported
     * baud rates (9600, 19200, 38400, 57600, 115200, 230400, 460800) and
     * protocol-specific detection sequences until GPS identified.
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:686-709
     */
    struct detect_state {
        uint32_t last_baud_change_ms;           ///< Time of last baud rate change
        uint8_t current_baud;                   ///< Current baud rate index
        uint32_t probe_baud;                    ///< Actual baud rate being probed
        bool auto_detected_baud;                ///< true if baud rate successfully detected
#if AP_GPS_UBLOX_ENABLED
        struct UBLOX_detect_state ublox_detect_state; ///< u-blox detection state
#endif
#if AP_GPS_SIRF_ENABLED
        struct SIRF_detect_state sirf_detect_state;   ///< SiRF detection state
#endif
#if AP_GPS_NMEA_ENABLED
        struct NMEA_detect_state nmea_detect_state;   ///< NMEA detection state
#endif
#if AP_GPS_SBP_ENABLED
        struct SBP_detect_state sbp_detect_state;     ///< Swift Binary v1 detection state
#endif
#if AP_GPS_SBP2_ENABLED
        struct SBP2_detect_state sbp2_detect_state;   ///< Swift Binary v2 detection state
#endif
#if AP_GPS_ERB_ENABLED
        struct ERB_detect_state erb_detect_state;     ///< Emlid Reach Binary detection state
#endif
    } detect_state[GPS_MAX_RECEIVERS];

    /**
     * @brief Initialization blob transmission state
     * @details Tracks progress sending initialization strings to GPS receivers
     * during startup configuration.
     */
    struct {
        const char *blob;                       ///< Pointer to initialization data
        uint16_t remaining;                     ///< Bytes remaining to send
    } initblob_state[GPS_MAX_RECEIVERS];

    static const uint32_t  _baudrates[];
    static const char _initialisation_blob[];
    static const char _initialisation_raw_blob[];

    void detect_instance(uint8_t instance);
    // run detection step for one GPS instance. If this finds a GPS then it
    // will return it - otherwise nullptr
    AP_GPS_Backend *_detect_instance(uint8_t instance);

    void update_instance(uint8_t instance);

    /**
     * @struct rtcm_buffer
     * @brief RTCM data reassembly buffer for fragmented injection
     * 
     * @details Buffer for re-assembling RTCM correction data received in
     * fragments via MAVLink GPS_RTCM_DATA messages. The 8-bit flags field
     * in GPS_RTCM_DATA is interpreted as:
     * - 1 bit: "is fragmented" flag
     * - 2 bits: Fragment number (0-3)
     * - 5 bits: Sequence number (0-31)
     * 
     * The rtcm_buffer is allocated on first use. Once a block of data is
     * successfully reassembled, it is injected into all active GPS backends.
     * Maximum RTCM block size: 4 fragments × 180 bytes/fragment = 720 bytes
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:727-745
     */
    struct rtcm_buffer {
        uint8_t fragments_received;             ///< Bitmask of received fragments
        uint8_t sequence;                       ///< Current sequence number
        uint8_t fragment_count;                 ///< Total fragments in message
        uint16_t total_length;                  ///< Total message length in bytes
        uint8_t buffer[MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN*4]; ///< Reassembly buffer (720 bytes)
    } *rtcm_buffer;

    /**
     * @brief RTCM fragment statistics
     * @details Tracks RTCM injection performance and fragment loss
     */
    struct {
        uint16_t fragments_used;                ///< Successfully processed RTCM fragments
        uint16_t fragments_discarded;           ///< Discarded fragments (out of sequence, timeout)
    } rtcm_stats;

    // re-assemble GPS_RTCM_DATA message
    void handle_gps_rtcm_data(mavlink_channel_t chan, const mavlink_message_t &msg);
    void handle_gps_inject(const mavlink_message_t &msg);

    //Inject a packet of raw binary to a GPS
    void inject_data(uint8_t instance, const uint8_t *data, uint16_t len);

#if AP_GPS_BLENDED_ENABLED
    bool _output_is_blended; // true when a blended GPS solution being output
#endif

    bool should_log() const;

    bool needs_uart(GPS_Type type) const;

#if GPS_MAX_RECEIVERS > 1
    /// Update primary instance
    void update_primary(void);
#endif

    /**
     * @brief Get GPS yaw in centidegrees
     * @param instance GPS instance number
     * @return Yaw in centidegrees (0-36000), or 0 if not available
     */
    uint16_t gps_yaw_cdeg(uint8_t instance) const;

    /**
     * @enum GPS_AUTO_CONFIG
     * @brief GPS automatic configuration modes
     * 
     * @details Controls which GPS receivers are automatically configured
     * by ArduPilot at startup.
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:776-780
     */
    enum GPS_AUTO_CONFIG {
        GPS_AUTO_CONFIG_DISABLE = 0,            ///< No auto-configuration
        GPS_AUTO_CONFIG_ENABLE_SERIAL_ONLY = 1, ///< Auto-config serial GPS only
        GPS_AUTO_CONFIG_ENABLE_ALL = 2,         ///< Auto-config all GPS types
    };

    /**
     * @enum GPSAutoSwitch
     * @brief GPS automatic switching and blending modes
     * 
     * @details Determines how primary GPS is selected when multiple receivers
     * are available (GPS_AUTO_SWITCH parameter).
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:782-788
     */
    enum class GPSAutoSwitch {
        NONE        = 0,                        ///< Use GPS_PRIMARY, no switching
        USE_BEST    = 1,                        ///< Automatically select best GPS
        BLEND       = 2,                        ///< Blend multiple GPS sources
        //USE_SECOND  = 3,                      ///< Deprecated - use GPS_PRIMARY instead
        USE_PRIMARY_IF_3D_FIX = 4,              ///< Use primary if 3D fix, else best
    };

    /// GPS disable flag for failure testing (set via force_disable())
    bool _force_disable_gps;

    /// GPS yaw disable flag for failure testing (set via set_force_disable_yaw())
    bool _force_disable_gps_yaw;

    // logging support
    void Write_GPS(uint8_t instance);

#if AP_GPS_RTCM_DECODE_ENABLED
    /**
     * @brief RTCM decoder state for MAVLink channels
     * 
     * @details Per-MAVLink-channel RTCM decoders, enabled with AlwaysRTCMDecode
     * option in GPS_DRV_OPTIONS. Allows decoding RTCM messages from GPS output
     * to forward to other systems or for debugging.
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:800-809
     */
    struct {
        RTCM3_Parser *parsers[MAVLINK_COMM_NUM_BUFFERS]; ///< RTCM parser per MAVLink channel
        uint32_t sent_crc[32];                  ///< CRC of recently sent RTCM messages
        uint8_t sent_idx;                       ///< Index into sent_crc array
        uint16_t seen_mav_channels;             ///< Bitmask of active MAVLink channels
    } rtcm;
    
    /**
     * @brief Parse and forward RTCM data from MAVLink
     * @param chan MAVLink channel
     * @param pkt RTCM packet data
     * @return true if packet successfully parsed
     */
    bool parse_rtcm_injection(mavlink_channel_t chan, const mavlink_gps_rtcm_data_t &pkt);
#endif

    /**
     * @brief Convert deprecated parameters to new format
     * @details Called during initialization to migrate old parameter names
     */
    void convert_parameters();
};

/**
 * @namespace AP
 * @brief ArduPilot singleton accessors
 */
namespace AP {
    /**
     * @brief Get GPS subsystem singleton
     * @return Reference to global AP_GPS instance
     * 
     * @details Primary access method for GPS subsystem. Returns singleton
     * managing all GPS receivers.
     * 
     * Example usage:
     * @code
     * const AP_GPS &gps = AP::gps();
     * if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
     *     Location loc = gps.location();
     *     // Use location...
     * }
     * @endcode
     * 
     * Source: libraries/AP_GPS/AP_GPS.h:817
     */
    AP_GPS &gps();
};

#endif  // AP_GPS_ENABLED
