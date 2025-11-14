/**
 * @file Log.cpp
 * @brief Data logging functions for antenna tracker
 * 
 * @details Implements binary logging of attitude, vehicle barometer, and vehicle position
 *          for analysis and debugging. Log data includes tracker's own attitude and target
 *          angles, plus information received from the tracked vehicle via MAVLink.
 * 
 * Source: AntennaTracker/Log.cpp
 */

#include "Tracker.h"

// Logging functionality only compiled if HAL_LOGGING_ENABLED defined
#if HAL_LOGGING_ENABLED

// Code to Write and Read packets from AP_Logger log memory

/**
 * @brief Logs current tracker attitude and target angles
 * 
 * @details Writes attitude packet containing:
 *          - Tracker's own attitude (roll, pitch, yaw) from AHRS
 *          - Target pitch (nav_status.pitch) in degrees
 *          - Target bearing (nav_status.bearing) in centidegrees
 * 
 * @note Attitude logged to standard ATT message format compatible with analysis tools
 * @note Targets vector passed as {0, pitch, bearing} since roll target always zero for tracker
 * 
 * Source: AntennaTracker/Log.cpp:8-13
 */
void Tracker::Log_Write_Attitude()
{
    const Vector3f targets{0.0f, nav_status.pitch, nav_status.bearing};
    ahrs.Write_Attitude(targets);
    AP::ahrs().Log_Write();
}

/**
 * @brief Log message structure for vehicle barometer data
 * 
 * @details PACKED structure containing tracked vehicle's barometric pressure and
 *          altitude difference. Data received via MAVLink from tracked vehicle.
 * 
 * Source: AntennaTracker/Log.cpp:15-20
 */
struct PACKED log_Vehicle_Baro {
    LOG_PACKET_HEADER;
    uint64_t time_us;   ///< Timestamp in microseconds since tracker boot
    float    press;     ///< Vehicle barometric pressure in Pascals
    float    alt_diff;  ///< Altitude difference between tracker and vehicle in meters
};

/**
 * @brief Logs barometer data received from tracked vehicle
 * 
 * @details Writes VBAR log message containing vehicle pressure and altitude difference.
 *          Data received via MAVLink SCALED_PRESSURE messages from tracked vehicle.
 * 
 * @param[in] pressure Vehicle barometric pressure in Pascals
 * @param[in] altitude Altitude difference (vehicle - tracker) in meters, positive when vehicle above tracker
 * 
 * @note Used for altitude source selection via ALT_SOURCE parameter
 * @note One-time barometer calibration performed on first message to align altitude references
 * 
 * Source: AntennaTracker/Log.cpp:23-32
 */
void Tracker::Log_Write_Vehicle_Baro(float pressure, float altitude)
{
    struct log_Vehicle_Baro pkt = {
        LOG_PACKET_HEADER_INIT(LOG_V_BAR_MSG),
        time_us         : AP_HAL::micros64(),
        press           : pressure,
        alt_diff        : altitude
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

/**
 * @brief Log message structure for vehicle position and velocity
 * 
 * @details PACKED structure containing tracked vehicle's GPS position and 3D velocity vector.
 *          Data received via MAVLink from tracked vehicle.
 * 
 * Source: AntennaTracker/Log.cpp:34-43
 */
struct PACKED log_Vehicle_Pos {
    LOG_PACKET_HEADER;
    uint64_t time_us;         ///< Timestamp in microseconds since tracker boot
    int32_t vehicle_lat;      ///< Vehicle latitude in degrees * 1e7
    int32_t vehicle_lng;      ///< Vehicle longitude in degrees * 1e7
    int32_t vehicle_alt;      ///< Vehicle altitude in meters * 100
    float vehicle_vel_x;      ///< Vehicle velocity North component in m/s (NED frame)
    float vehicle_vel_y;      ///< Vehicle velocity East component in m/s (NED frame)
    float vehicle_vel_z;      ///< Vehicle velocity Down component in m/s (NED frame)
};

/**
 * @brief Logs position and velocity of tracked vehicle
 * 
 * @details Writes VPOS log message containing vehicle GPS location and velocity.
 *          Data received via MAVLink GLOBAL_POSITION_INT messages from tracked vehicle.
 * 
 * @param[in] lat Vehicle latitude in degrees * 1e7
 * @param[in] lng Vehicle longitude in degrees * 1e7
 * @param[in] alt Vehicle altitude in meters * 100
 * @param[in] vel Vehicle velocity vector in m/s (NED frame: North, East, Down)
 * 
 * @note Position used to calculate bearing and distance for tracking
 * @note Velocity used for lead angle prediction in update_vehicle_pos_estimate()
 * 
 * Source: AntennaTracker/Log.cpp:46-59
 */
void Tracker::Log_Write_Vehicle_Pos(int32_t lat, int32_t lng, int32_t alt, const Vector3f& vel)
{
    struct log_Vehicle_Pos pkt = {
        LOG_PACKET_HEADER_INIT(LOG_V_POS_MSG),
        time_us         : AP_HAL::micros64(),
        vehicle_lat     : lat,
        vehicle_lng     : lng,
        vehicle_alt     : alt,
        vehicle_vel_x   : vel.x,
        vehicle_vel_y   : vel.y,
        vehicle_vel_z   : vel.z,
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

// @LoggerMessage: VBAR
// @Description: Information received from tracked vehicle; barometer data
// @Field: TimeUS: Time since system startup
// @Field: Press: vehicle barometric pressure
// @Field: AltDiff: altitude difference based on difference on barometric pressure

// @LoggerMessage: VPOS
// @Description: Information received from tracked vehicle; barometer position data
// @Field: TimeUS: Time since system startup
// @Field: Lat: tracked vehicle latitude
// @Field: Lng: tracked vehicle longitude
// @Field: Alt: tracked vehicle altitude
// @Field: VelX: tracked vehicle velocity, latitude component
// @Field: VelY: tracked vehicle velocity, longitude component
// @Field: VelZ: tracked vehicle velocity, vertical component, down

// type and unit information can be found in
// libraries/AP_Logger/Logstructure.h; search for "log_Units" for
// units and "Format characters" for field type information

/**
 * @brief Defines tracker-specific log message structures
 * 
 * @details Combined with LOG_COMMON_STRUCTURES for complete log format.
 *          Format string components:
 *          - "Qff" = uint64_t, float, float
 *          - "QLLefff" = uint64_t, int32_t, int32_t, int32_t, float, float, float
 *          Units:
 *          - "sPm" = seconds, Pascals, meters
 *          - "sddmnnn" = seconds, degrees, degrees, meters, m/s, m/s, m/s
 *          Multipliers:
 *          - "F00" = field 0 (TimeUS) is multiplier base
 *          - "FGGB000" = GPS precision for lat/lng fields (1e7), altitude (100)
 * 
 * Source: AntennaTracker/Log.cpp:80-86
 */
const struct LogStructure Tracker::log_structure[] = {
    LOG_COMMON_STRUCTURES,
    // VBAR: Vehicle barometer data (pressure and altitude difference)
    {LOG_V_BAR_MSG, sizeof(log_Vehicle_Baro),
        "VBAR", "Qff", "TimeUS,Press,AltDiff", "sPm", "F00" , true },
    // VPOS: Vehicle position and velocity from GPS
    {LOG_V_POS_MSG, sizeof(log_Vehicle_Pos),
       "VPOS", "QLLefff", "TimeUS,Lat,Lng,Alt,VelX,VelY,VelZ", "sddmnnn", "FGGB000", true }
};

/**
 * @brief Returns number of tracker-specific log structures
 * 
 * @details Used by AP_Logger to register tracker log messages during initialization.
 * 
 * @return Number of elements in log_structure array
 * 
 * Source: AntennaTracker/Log.cpp:88-91
 */
uint8_t Tracker::get_num_log_structures() const
{
    return ARRAY_SIZE(log_structure);
}

/**
 * @brief Logs startup information for tracker and GPS
 * 
 * @details Called during initialization to record:
 *          - Initial mode (typically INITIALISING)
 *          - GPS startup messages (version, configuration)
 * 
 * @note Startup logging helps diagnose initialization issues from log files
 * 
 * Source: AntennaTracker/Log.cpp:93-97
 */
void Tracker::Log_Write_Vehicle_Startup_Messages()
{
    logger.Write_Mode((uint8_t)mode->number(), ModeReason::INITIALISED);
    gps.Write_AP_Logger_Log_Startup_messages();
}

// End of conditional compilation block
#endif // HAL_LOGGING_ENABLED
