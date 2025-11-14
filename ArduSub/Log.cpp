/**
 * @file Log.cpp
 * @brief ArduSub-specific dataflash/binary logging implementation
 * 
 * @details This file implements the vehicle-specific logging functions for ArduSub,
 *          writing binary log data to persistent storage via the AP_Logger library.
 *          Logs are used for post-flight analysis, debugging, algorithm tuning,
 *          and incident investigation.
 *          
 *          ArduSub-specific log messages include:
 *          - CTUN: Control tuning data (depth control, climb rates, throttle)
 *          - GUIP: Guided mode position and velocity targets
 *          - D16/D32/DFLT/DU16/DU32: Generic data logging for debugging
 *          
 *          Integration:
 *          - Uses AP_Logger (libraries/AP_Logger) for actual log writing
 *          - Log messages defined via log_structure[] table
 *          - WriteBlock() for normal priority, WriteCriticalBlock() for high priority
 *          - All timestamps use AP_HAL::micros64() for microsecond precision
 *          
 * @note Log files can be analyzed using MAVExplorer, pymavlink, or Mission Planner
 * @note Log replay is possible using Tools/Replay for algorithm debugging
 * @note This file is only compiled when HAL_LOGGING_ENABLED is true
 * 
 * @see libraries/AP_Logger for the logging framework
 * @see libraries/AP_Logger/LogStructure.h for log format definitions
 * 
 * Source: ArduSub/Log.cpp:1-end
 */

#include "Sub.h"

#if HAL_LOGGING_ENABLED

// Code to Write and Read packets from AP_Logger log memory
// Code to interact with the user to dump or erase logs

/**
 * @struct log_Control_Tuning
 * @brief Control tuning log packet for depth/altitude control performance monitoring
 * 
 * @details This PACKED structure defines the CTUN log message format, which records
 *          critical data for analyzing depth hold, altitude control, and vertical
 *          velocity performance in ArduSub. Essential for PID tuning and debugging
 *          unexpected depth behavior.
 *          
 *          Logged at regular intervals during flight to capture control loop performance.
 *          
 * @note PACKED attribute ensures consistent binary log format across platforms
 * @note All altitude/depth fields in meters, rates in cm/s
 */
struct PACKED log_Control_Tuning {
    LOG_PACKET_HEADER;              ///< Standard log packet header with message type
    uint64_t time_us;               ///< Timestamp in microseconds since system boot
    float  throttle_in;             ///< Pilot throttle input (normalized, typically -1.0 to 1.0)
    float  angle_boost;             ///< Throttle adjustment for attitude compensation
    float    throttle_out;          ///< Final motor throttle output (normalized 0.0 to 1.0)
    float    throttle_hover;        ///< Calculated hover throttle for neutral buoyancy
    float    desired_alt;           ///< Target altitude/depth from position controller (meters)
    float    inav_alt;              ///< Actual altitude/depth from inertial navigation (meters)
    float    baro_alt;              ///< Barometric altitude above sea level (meters)
    float    desired_rangefinder_alt; ///< Target rangefinder altitude for terrain following (meters)
    float    rangefinder_alt;       ///< Actual rangefinder measured altitude (meters)
    float    terr_alt;              ///< Terrain altitude - height above seafloor/ground (meters)
    int16_t  target_climb_rate;     ///< Desired vertical velocity from position controller (cm/s)
    int16_t  climb_rate;            ///< Actual vertical velocity (cm/s, positive = up)
};

/**
 * @brief Write control tuning data to dataflash log
 * 
 * @details Collects current control loop state including depth/altitude targets,
 *          actual positions, climb rates, and throttle outputs. This data is
 *          essential for:
 *          - Tuning PID controllers for depth hold and altitude control
 *          - Diagnosing unexpected depth changes or oscillations
 *          - Analyzing terrain following performance
 *          - Verifying hover throttle learning
 *          
 *          Data sources:
 *          - attitude_control: Throttle input and angle boost
 *          - motors: Actual throttle output and hover throttle estimate
 *          - pos_control: Position and velocity targets
 *          - inertial_nav: Actual position estimates
 *          - barometer: Pressure altitude
 *          - rangefinder_state: Range measurements for terrain following
 *          - terrain: Terrain database altitude (if enabled)
 *          
 * @note Called periodically during flight (frequency depends on logging rate)
 * @note Terrain altitude calculated from terrain database or rangefinder offset
 * @note Uses WriteBlock() - normal priority logging
 * 
 * @see log_Control_Tuning for field definitions
 * @see libraries/AC_AttitudeControl for attitude control interface
 * @see libraries/AC_PosControl for position control interface
 */
void Sub::Log_Write_Control_Tuning()
{
    // get terrain altitude
    float terr_alt = 0.0f;
#if AP_TERRAIN_AVAILABLE
    if (terrain.enabled()) {
        terrain.height_above_terrain(terr_alt, true);
    } else {
        terr_alt = rangefinder_state.rangefinder_terrain_offset_cm * 0.01f;
    }
#else
    terr_alt = rangefinder_state.rangefinder_terrain_offset_cm * 0.01f;
#endif

    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CONTROL_TUNING_MSG),
        time_us             : AP_HAL::micros64(),
        throttle_in         : attitude_control.get_throttle_in(),
        angle_boost         : attitude_control.angle_boost(),
        throttle_out        : motors.get_throttle(),
        throttle_hover      : motors.get_throttle_hover(),
        desired_alt         : pos_control.get_pos_target_U_cm() * 0.01f,
        inav_alt            : inertial_nav.get_position_z_up_cm() * 0.01f,
        baro_alt            : barometer.get_altitude(),
        desired_rangefinder_alt   : mode_surftrak.get_rangefinder_target_cm() * 0.01,
        rangefinder_alt           : rangefinder_state.alt,
        terr_alt            : terr_alt,
        target_climb_rate   : (int16_t)pos_control.get_vel_target_U_cms(),
        climb_rate          : climb_rate
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

/**
 * @brief Write attitude and attitude target data to dataflash log
 * 
 * @details Logs vehicle attitude (roll, pitch, yaw) and attitude controller targets.
 *          Delegates to AHRS and attitude controller for actual data collection.
 *          Critical for analyzing vehicle stability, control response, and attitude
 *          tracking performance.
 *          
 *          Two separate log writes occur:
 *          1. ahrs.Write_Attitude() - Logs actual attitude and desired attitude
 *          2. AP::ahrs().Log_Write() - Logs AHRS state and sensor fusion data
 *          
 * @note Called at main loop rate (typically 50-400 Hz depending on vehicle)
 * @note Attitude targets converted from radians to degrees for logging
 * @note Essential for debugging stability issues and control tuning
 * 
 * @see libraries/AP_AHRS for attitude estimation
 * @see libraries/AC_AttitudeControl for attitude control targets
 */
void Sub::Log_Write_Attitude()
{
    ahrs.Write_Attitude(attitude_control.get_att_target_euler_rad() * RAD_TO_DEG);

    AP::ahrs().Log_Write();
}

/**
 * @struct log_Data_Int16t
 * @brief Generic 16-bit signed integer data logging packet
 * 
 * @details PACKED structure for logging arbitrary 16-bit signed integer values
 *          identified by an ID. Used for debugging and logging custom data points
 *          that don't warrant a dedicated log message type.
 *          
 * @note PACKED ensures consistent binary format across platforms
 * @note ID field identifies what the data represents (application-specific)
 */
struct PACKED log_Data_Int16t {
    LOG_PACKET_HEADER;              ///< Standard log packet header
    uint64_t time_us;               ///< Timestamp in microseconds since system boot
    uint8_t id;                     ///< Data identifier (LogDataID enum value)
    int16_t data_value;             ///< Signed 16-bit integer value
};

/**
 * @brief Write generic 16-bit signed integer data to dataflash log
 * 
 * @details Logs an arbitrary signed 16-bit integer value with an associated ID.
 *          Useful for debugging custom algorithms or logging transient data points
 *          without creating dedicated log message types.
 *          
 * @param[in] id    Data identifier from LogDataID enum
 * @param[in] value Signed 16-bit integer value to log
 * 
 * @note UNUSED_FUNCTION attribute indicates this may not be called in current code
 * @note Uses WriteCriticalBlock() for high-priority logging
 * @note Only logs if MASK_LOG_ANY logging is enabled
 * 
 * @see LogDataID enum for available identifiers
 */
UNUSED_FUNCTION
void Sub::Log_Write_Data(LogDataID id, int16_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Int16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT16_MSG),
            time_us     : AP_HAL::micros64(),
            id          : (uint8_t)id,
            data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

/**
 * @struct log_Data_UInt16t
 * @brief Generic 16-bit unsigned integer data logging packet
 * 
 * @details PACKED structure for logging arbitrary 16-bit unsigned integer values.
 *          Used for counters, flags, or other non-negative integer data.
 *          
 * @note PACKED ensures consistent binary format across platforms
 */
struct PACKED log_Data_UInt16t {
    LOG_PACKET_HEADER;              ///< Standard log packet header
    uint64_t time_us;               ///< Timestamp in microseconds since system boot
    uint8_t id;                     ///< Data identifier (LogDataID enum value)
    uint16_t data_value;            ///< Unsigned 16-bit integer value
};

/**
 * @brief Write generic 16-bit unsigned integer data to dataflash log
 * 
 * @details Logs an arbitrary unsigned 16-bit integer value with an associated ID.
 *          Suitable for counters, bitmasks, or other non-negative values.
 *          
 * @param[in] id    Data identifier from LogDataID enum
 * @param[in] value Unsigned 16-bit integer value to log
 * 
 * @note UNUSED_FUNCTION attribute indicates this may not be called in current code
 * @note Uses WriteCriticalBlock() for high-priority logging
 * @note Only logs if MASK_LOG_ANY logging is enabled
 */
UNUSED_FUNCTION
void Sub::Log_Write_Data(LogDataID id, uint16_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_UInt16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT16_MSG),
            time_us     : AP_HAL::micros64(),
            id          : (uint8_t)id,
            data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

/**
 * @struct log_Data_Int32t
 * @brief Generic 32-bit signed integer data logging packet
 * 
 * @details PACKED structure for logging arbitrary 32-bit signed integer values.
 *          Provides wider range than 16-bit variant for larger values or
 *          higher precision measurements.
 *          
 * @note PACKED ensures consistent binary format across platforms
 */
struct PACKED log_Data_Int32t {
    LOG_PACKET_HEADER;              ///< Standard log packet header
    uint64_t time_us;               ///< Timestamp in microseconds since system boot
    uint8_t id;                     ///< Data identifier (LogDataID enum value)
    int32_t data_value;             ///< Signed 32-bit integer value
};

/**
 * @brief Write generic 32-bit signed integer data to dataflash log
 * 
 * @details Logs an arbitrary signed 32-bit integer value with an associated ID.
 *          Suitable for larger values, timestamps, or high-resolution counters.
 *          
 * @param[in] id    Data identifier from LogDataID enum
 * @param[in] value Signed 32-bit integer value to log
 * 
 * @note Uses WriteCriticalBlock() for high-priority logging
 * @note Only logs if MASK_LOG_ANY logging is enabled
 * @note This overload is actively used (no UNUSED_FUNCTION attribute)
 */
void Sub::Log_Write_Data(LogDataID id, int32_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Int32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT32_MSG),
            time_us  : AP_HAL::micros64(),
            id          : (uint8_t)id,
            data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

/**
 * @struct log_Data_UInt32t
 * @brief Generic 32-bit unsigned integer data logging packet
 * 
 * @details PACKED structure for logging arbitrary 32-bit unsigned integer values.
 *          Suitable for large counters, memory addresses, or bitmasks requiring
 *          32-bit range.
 *          
 * @note PACKED ensures consistent binary format across platforms
 */
struct PACKED log_Data_UInt32t {
    LOG_PACKET_HEADER;              ///< Standard log packet header
    uint64_t time_us;               ///< Timestamp in microseconds since system boot
    uint8_t id;                     ///< Data identifier (LogDataID enum value)
    uint32_t data_value;            ///< Unsigned 32-bit integer value
};

/**
 * @brief Write generic 32-bit unsigned integer data to dataflash log
 * 
 * @details Logs an arbitrary unsigned 32-bit integer value with an associated ID.
 *          Provides full 32-bit non-negative range for large values.
 *          
 * @param[in] id    Data identifier from LogDataID enum
 * @param[in] value Unsigned 32-bit integer value to log
 * 
 * @note Uses WriteCriticalBlock() for high-priority logging
 * @note Only logs if MASK_LOG_ANY logging is enabled
 * @note This overload is actively used (no UNUSED_FUNCTION attribute)
 */
void Sub::Log_Write_Data(LogDataID id, uint32_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_UInt32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT32_MSG),
            time_us     : AP_HAL::micros64(),
            id          : (uint8_t)id,
            data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

/**
 * @struct log_Data_Float
 * @brief Generic floating-point data logging packet
 * 
 * @details PACKED structure for logging arbitrary single-precision floating-point
 *          values. Used for analog measurements, calculated values, or any data
 *          requiring decimal precision.
 *          
 * @note PACKED ensures consistent binary format across platforms
 * @note Uses IEEE 754 single-precision format (32-bit float)
 */
struct PACKED log_Data_Float {
    LOG_PACKET_HEADER;              ///< Standard log packet header
    uint64_t time_us;               ///< Timestamp in microseconds since system boot
    uint8_t id;                     ///< Data identifier (LogDataID enum value)
    float data_value;               ///< Single-precision floating-point value
};

/**
 * @brief Write generic floating-point data to dataflash log
 * 
 * @details Logs an arbitrary single-precision floating-point value with an
 *          associated ID. Suitable for analog sensor readings, calculated values,
 *          or any measurement requiring decimal precision.
 *          
 * @param[in] id    Data identifier from LogDataID enum
 * @param[in] value Single-precision floating-point value to log
 * 
 * @note UNUSED_FUNCTION attribute indicates this may not be called in current code
 * @note Uses WriteCriticalBlock() for high-priority logging
 * @note Only logs if MASK_LOG_ANY logging is enabled
 */
UNUSED_FUNCTION
void Sub::Log_Write_Data(LogDataID id, float value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Float pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_FLOAT_MSG),
            time_us     : AP_HAL::micros64(),
            id          : (uint8_t)id,
            data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

/**
 * @struct log_GuidedTarget
 * @brief Guided mode position and velocity target logging packet
 * 
 * @details PACKED structure for logging commanded position and velocity targets
 *          when the vehicle is operating in Guided mode. Essential for debugging
 *          guided mode behavior and analyzing autonomous navigation performance.
 *          
 *          Coordinate frames:
 *          - Position targets are in NED (North-East-Down) frame relative to origin
 *          - Velocity targets are in NED frame (m/s)
 *          
 * @note PACKED ensures consistent binary format across platforms
 * @note All position fields in meters, velocity fields in m/s
 */
struct PACKED log_GuidedTarget {
    LOG_PACKET_HEADER;              ///< Standard log packet header
    uint64_t time_us;               ///< Timestamp in microseconds since system boot
    uint8_t type;                   ///< Guided target type (position, velocity, angle, etc.)
    float pos_target_x;             ///< Target position North component (meters, NED frame)
    float pos_target_y;             ///< Target position East component (meters, NED frame)
    float pos_target_z;             ///< Target position Down component (meters, NED frame)
    float vel_target_x;             ///< Target velocity North component (m/s, NED frame)
    float vel_target_y;             ///< Target velocity East component (m/s, NED frame)
    float vel_target_z;             ///< Target velocity Down component (m/s, NED frame)
};

/**
 * @brief Write guided mode target position and velocity to dataflash log
 * 
 * @details Logs the commanded position and velocity targets when operating in
 *          Guided mode. Critical for analyzing autonomous navigation behavior,
 *          debugging waypoint tracking issues, and verifying external control
 *          commands from companion computers or ground stations.
 *          
 *          Typical usage scenarios:
 *          - MAVLink SET_POSITION_TARGET commands from ground station
 *          - Companion computer guidance commands
 *          - Automatic mission execution
 *          
 * @param[in] target_type Type of guided target (position, velocity, angle, etc.)
 * @param[in] pos_target  Target position vector in NED frame (meters)
 * @param[in] vel_target  Target velocity vector in NED frame (m/s)
 * 
 * @note Uses WriteBlock() - normal priority logging
 * @note All coordinates in NED (North-East-Down) frame
 * @note Called when new guided targets are set
 * 
 * @see log_GuidedTarget for field definitions
 * @see mode_guided.cpp for guided mode implementation
 */
void Sub::Log_Write_GuidedTarget(uint8_t target_type, const Vector3f& pos_target, const Vector3f& vel_target)
{
    struct log_GuidedTarget pkt = {
        LOG_PACKET_HEADER_INIT(LOG_GUIDEDTARGET_MSG),
        time_us         : AP_HAL::micros64(),
        type            : target_type,
        pos_target_x    : pos_target.x,
        pos_target_y    : pos_target.y,
        pos_target_z    : pos_target.z,
        vel_target_x    : vel_target.x,
        vel_target_y    : vel_target.y,
        vel_target_z    : vel_target.z
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

// @LoggerMessage: CTUN
// @Description: Control Tuning information
// @Field: TimeUS: Time since system startup
// @Field: ThI: throttle input
// @Field: ABst: angle boost
// @Field: ThO: throttle output
// @Field: ThH: calculated hover throttle
// @Field: DAlt: desired altitude
// @Field: Alt: achieved altitude
// @Field: BAlt: barometric altitude
// @Field: DSAlt: desired rangefinder altitude
// @Field: SAlt: achieved rangefinder altitude
// @Field: TAlt: terrain altitude
// @Field: DCRt: desired climb rate
// @Field: CRt: climb rate

// @LoggerMessage: D16
// @Description: Generic 16-bit-signed-integer storage
// @Field: TimeUS: Time since system startup
// @Field: Id: Data type identifier
// @Field: Value: Value

// @LoggerMessage: D32
// @Description: Generic 32-bit-signed-integer storage
// @Field: TimeUS: Time since system startup
// @Field: Id: Data type identifier
// @Field: Value: Value

// @LoggerMessage: DFLT
// @Description: Generic float storage
// @Field: TimeUS: Time since system startup
// @Field: Id: Data type identifier
// @Field: Value: Value

// @LoggerMessage: DU16
// @Description: Generic 16-bit-unsigned-integer storage
// @Field: TimeUS: Time since system startup
// @Field: Id: Data type identifier
// @Field: Value: Value

// @LoggerMessage: DU32
// @Description: Generic 32-bit-unsigned-integer storage
// @Field: TimeUS: Time since system startup
// @Field: Id: Data type identifier
// @Field: Value: Value

// @LoggerMessage: GUIP
// @Description: Guided mode target information
// @Field: TimeUS: Time since system startup
// @Field: Type: Type of guided mode
// @Field: pX: Target position, X-Axis
// @Field: pY: Target position, Y-Axis
// @Field: pZ: Target position, Z-Axis
// @Field: vX: Target velocity, X-Axis
// @Field: vY: Target velocity, Y-Axis
// @Field: vZ: Target velocity, Z-Axis

/**
 * @var log_structure
 * @brief ArduSub-specific log message format definitions
 * 
 * @details This array defines the binary format for all ArduSub-specific log messages.
 *          Each entry specifies:
 *          - Message ID (e.g., LOG_CONTROL_TUNING_MSG)
 *          - Structure size in bytes
 *          - Message name (4-character identifier, e.g., "CTUN")
 *          - Format string: Type codes for each field (Q=uint64, f=float, h=int16, etc.)
 *          - Field names: Comma-separated list of column headers
 *          - Units: Character codes for each field's unit (m=meters, s=seconds, etc.)
 *          - Multipliers: Scaling factors for display (0=none, B=1/100, etc.)
 *          
 *          The LOG_COMMON_STRUCTURES macro includes standard messages shared across
 *          all vehicle types (ATT, GPS, IMU, etc.).
 *          
 *          Format character reference:
 *          - Q: uint64_t (8 bytes)
 *          - f: float (4 bytes)
 *          - h: int16_t (2 bytes)
 *          - H: uint16_t (2 bytes)
 *          - i: int32_t (4 bytes)
 *          - I: uint32_t (4 bytes)
 *          - B: uint8_t (1 byte)
 *          
 *          Unit character reference (see libraries/AP_Logger/LogStructure.h):
 *          - s: seconds
 *          - m: meters
 *          - n: meters/second
 *          - -: dimensionless
 *          
 * @note Changes to this table require corresponding struct definition changes
 * @note Message names must be unique and exactly 4 characters
 * @note Field count must match format string length
 * 
 * @see libraries/AP_Logger/LogStructure.h for complete format definitions
 * @see libraries/AP_Logger/AP_Logger.h for logging interface
 */
// type and unit information can be found in
// libraries/AP_Logger/Logstructure.h; search for "log_Units" for
// units and "Format characters" for field type information
const struct LogStructure Sub::log_structure[] = {
    LOG_COMMON_STRUCTURES,
    { LOG_CONTROL_TUNING_MSG, sizeof(log_Control_Tuning),
      "CTUN", "Qffffffffffhh", "TimeUS,ThI,ABst,ThO,ThH,DAlt,Alt,BAlt,DSAlt,SAlt,TAlt,DCRt,CRt", "s----mmmmmmnn", "F----00B00BBB" },
    { LOG_DATA_INT16_MSG, sizeof(log_Data_Int16t),         
      "D16",   "QBh",         "TimeUS,Id,Value", "s--", "F--" },
    { LOG_DATA_UINT16_MSG, sizeof(log_Data_UInt16t),         
      "DU16",  "QBH",         "TimeUS,Id,Value", "s--", "F--" },
    { LOG_DATA_INT32_MSG, sizeof(log_Data_Int32t),         
      "D32",   "QBi",         "TimeUS,Id,Value", "s--", "F--" },
    { LOG_DATA_UINT32_MSG, sizeof(log_Data_UInt32t),         
      "DU32",  "QBI",         "TimeUS,Id,Value", "s--", "F--" },
    { LOG_DATA_FLOAT_MSG, sizeof(log_Data_Float),         
      "DFLT",  "QBf",         "TimeUS,Id,Value", "s--", "F--" },
    { LOG_GUIDEDTARGET_MSG, sizeof(log_GuidedTarget),
      "GUIP",  "QBffffff",    "TimeUS,Type,pX,pY,pZ,vX,vY,vZ", "s-mmmnnn", "F-000000" },
};

/**
 * @brief Get the number of log message types defined for ArduSub
 * 
 * @details Returns the total count of log message structures in the log_structure[]
 *          array, including both ArduSub-specific messages and common messages from
 *          LOG_COMMON_STRUCTURES. Used by AP_Logger to register all available log
 *          message types during initialization.
 *          
 * @return uint8_t Number of log structure definitions
 * 
 * @note This is a const member function - does not modify object state
 * @note ARRAY_SIZE macro calculates element count at compile time
 * 
 * @see log_structure for the message definitions
 * @see libraries/AP_Logger for registration mechanism
 */
uint8_t Sub::get_num_log_structures() const
{
    return ARRAY_SIZE(log_structure);
}

/**
 * @brief Write critical vehicle state to log at startup/arming
 * 
 * @details Logs essential vehicle configuration and initial state information at
 *          system startup or when arming. This data provides context for the entire
 *          flight log and is critical for post-flight analysis.
 *          
 *          Startup messages logged:
 *          1. Initial flight mode and reason for mode selection
 *          2. Home position and EKF origin coordinates
 *          3. GPS configuration and initial satellite lock status
 *          
 *          These messages are written to the log header area which has limited
 *          space (approximately 200 bytes guaranteed by AP_Logger). Only critical
 *          startup information should be included.
 *          
 * @note Called during vehicle initialization and arming sequences
 * @note Limited space available - only essential startup data
 * @note Helps correlate log analysis with vehicle configuration
 * 
 * @warning Do not add excessive startup messages - header space is limited
 * 
 * @see libraries/AP_Logger for startup message handling
 * @see Sub::init_ardupilot() for initialization sequence
 */
void Sub::Log_Write_Vehicle_Startup_Messages()
{
    // only 200(?) bytes are guaranteed by AP_Logger
    logger.Write_Mode((uint8_t)control_mode, control_mode_reason);
    ahrs.Log_Write_Home_And_Origin();
    gps.Write_AP_Logger_Log_Startup_messages();
}


#endif // HAL_LOGGING_ENABLED
