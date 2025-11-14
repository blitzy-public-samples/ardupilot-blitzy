/**
 * @file Log.cpp
 * @brief Binary logging implementation for Blimp vehicles
 * 
 * @details This file implements the binary logging system for the Blimp (lighter-than-air)
 *          vehicle type in ArduPilot. It defines packed binary log message structures and
 *          provides Write_* wrapper functions for logging blimp-specific telemetry data.
 * 
 *          The logging system captures:
 *          - Fin control inputs (FINI) - pilot commands to fins
 *          - Fin control outputs (FINO) - actual fin amplitude and offset values
 *          - PID controller data for position and velocity control
 *          - Attitude data (with zero attitude targets as blimp uses velocity control)
 *          - EKF state estimation and position data
 *          - Control tuning parameters
 *          - Generic data logging for debugging (int16, uint16, int32, uint32, float)
 * 
 *          Binary log messages are written to the AP_Logger system and stored in dataflash
 *          memory or SD card for post-flight analysis. Log structure definitions are
 *          registered in the log_structure array with field formats, units, and multipliers.
 * 
 * @note Blimp uses velocity control rather than attitude control, so attitude targets
 *       are always zero. The focus is on position/velocity PID controllers and fin outputs.
 * 
 * @see libraries/AP_Logger/AP_Logger.h for base logging system
 * @see Blimp.h for main Blimp class and logging interface declarations
 * 
 * Source: Blimp/Log.cpp
 */

#include "Blimp.h"

#if HAL_LOGGING_ENABLED

// Code to Write and Read packets from AP_Logger log memory
// Code to interact with the user to dump or erase logs

/**
 * @struct log_FINI
 * @brief Fin input control command log message structure
 * 
 * @details Logs the pilot's desired fin control inputs before any processing or mixing.
 *          These are the commanded control values from the flight mode to the fin mixer.
 *          For blimps, fins provide control authority in all axes similar to how motor
 *          thrust provides control for multirotors.
 * 
 *          Logging frequency: Updated at main loop rate (typically 50-400Hz depending on board)
 * 
 * @note Values are normalized control inputs, not physical angles or thrust values
 * @see Write_FINI() for the function that logs this structure
 */
struct PACKED log_FINI {
    LOG_PACKET_HEADER;          ///< Standard log packet header with message type and size
    uint64_t time_us;            ///< Timestamp in microseconds since system boot
    float Right;                 ///< Right direction control input (normalized, unitless)
    float Front;                 ///< Front direction control input (normalized, unitless)
    float Down;                  ///< Down direction control input (normalized, unitless)
    float Yaw;                   ///< Yaw rotation control input (normalized, unitless)
};

/**
 * @struct log_FINO
 * @brief Fin output control values log message structure
 * 
 * @details Logs the actual fin servo outputs after mixing and processing. Blimp fin control
 *          uses amplitude and offset values for each of the 4 fins to generate oscillating
 *          fin motion. The amplitude controls the magnitude of fin movement and the offset
 *          controls the center point of oscillation.
 * 
 *          Logging frequency: Updated at main loop rate (typically 50-400Hz depending on board)
 * 
 * @note These are the final output values sent to the servo/motor controllers
 * @see Write_FINO() for the function that logs this structure
 */
struct PACKED log_FINO {
    LOG_PACKET_HEADER;          ///< Standard log packet header with message type and size
    uint64_t time_us;            ///< Timestamp in microseconds since system boot
    float Fin1_Amp;              ///< Fin 1 amplitude (oscillation magnitude, unitless)
    float Fin1_Off;              ///< Fin 1 offset (center point of oscillation, unitless)
    float Fin2_Amp;              ///< Fin 2 amplitude (oscillation magnitude, unitless)
    float Fin2_Off;              ///< Fin 2 offset (center point of oscillation, unitless)
    float Fin3_Amp;              ///< Fin 3 amplitude (oscillation magnitude, unitless)
    float Fin3_Off;              ///< Fin 3 offset (center point of oscillation, unitless)
    float Fin4_Amp;              ///< Fin 4 amplitude (oscillation magnitude, unitless)
    float Fin4_Off;              ///< Fin 4 offset (center point of oscillation, unitless)
};

/**
 * @brief Log fin control input commands
 * 
 * @details Writes a FINI (Fin Input) log message containing the pilot's desired control
 *          inputs for fin control. These values represent the commanded control authority
 *          in each axis before being processed by the fin mixer. This is useful for
 *          debugging control issues and understanding pilot commands.
 * 
 *          Call frequency: Typically called at main loop rate (50-400Hz)
 * 
 * @param[in] right  Right direction control input (normalized, unitless)
 * @param[in] front  Front direction control input (normalized, unitless)
 * @param[in] down   Down direction control input (normalized, unitless)
 * @param[in] yaw    Yaw rotation control input (normalized, unitless)
 * 
 * @note Values are typically in the range [-1.0, 1.0] but may exceed for special cases
 * @see log_FINI for the logged data structure
 * @see Fins.cpp for fin mixing implementation that consumes these inputs
 */
void Blimp::Write_FINI(float right, float front, float down, float yaw)
{
    const struct log_FINI pkt {
        LOG_PACKET_HEADER_INIT(LOG_FINI_MSG),
        time_us       : AP_HAL::micros64(),
        Right         : right,
        Front         : front,
        Down          : down,
        Yaw           : yaw
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

/**
 * @brief Log fin control output values
 * 
 * @details Writes a FINO (Fin Output) log message containing the actual amplitude and
 *          offset values sent to each of the 4 fins. These are the final output values
 *          after all mixing and processing. Amplitude controls the magnitude of fin
 *          oscillation and offset controls the center point. Comparing FINO with FINI
 *          logs helps diagnose mixing and output limiting issues.
 * 
 *          Call frequency: Typically called at main loop rate (50-400Hz)
 * 
 * @param[in] amp  Array of 4 fin amplitude values (oscillation magnitude, unitless)
 * @param[in] off  Array of 4 fin offset values (center point of oscillation, unitless)
 * 
 * @note Arrays must contain exactly 4 elements corresponding to fins 1-4
 * @see log_FINO for the logged data structure
 * @see Fins.cpp for fin mixing implementation that generates these outputs
 */
void Blimp::Write_FINO(float *amp, float *off)
{
    const struct log_FINO pkt {
        LOG_PACKET_HEADER_INIT(LOG_FINO_MSG),
        time_us       : AP_HAL::micros64(),
        Fin1_Amp      : amp[0],
        Fin1_Off      : off[0],
        Fin2_Amp      : amp[1],
        Fin2_Off      : off[1],
        Fin3_Amp      : amp[2],
        Fin3_Off      : off[2],
        Fin4_Amp      : amp[3],
        Fin4_Off      : off[3],
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

/**
 * @struct log_Control_Tuning
 * @brief Control tuning parameters log message structure
 * 
 * @details Logs altitude control and throttle tuning parameters for the blimp. This message
 *          captures desired vs achieved altitude from multiple sensors (baro, rangefinder,
 *          terrain) along with throttle commands and climb rates. Essential for tuning
 *          altitude hold performance and diagnosing vertical control issues.
 * 
 *          Logging frequency: Updated at main loop rate (typically 50-400Hz depending on board)
 * 
 * @note While blimp doesn't have traditional throttle, these fields are reused for
 *       vertical thrust/buoyancy control similar to multicopter
 * @see libraries/AP_Logger/LogStructure.h for CTUN message format details
 */
struct PACKED log_Control_Tuning {
    LOG_PACKET_HEADER;                ///< Standard log packet header with message type and size
    uint64_t time_us;                  ///< Timestamp in microseconds since system boot
    float    throttle_in;              ///< Throttle input command (normalized, 0-1)
    float    angle_boost;              ///< Angle boost compensation (not typically used for blimp)
    float    throttle_out;             ///< Final throttle output after processing (normalized, 0-1)
    float    throttle_hover;           ///< Calculated hover throttle/buoyancy compensation
    float    desired_alt;              ///< Target altitude from flight mode (meters)
    float    inav_alt;                 ///< Inertial navigation estimated altitude (meters)
    int32_t  baro_alt;                 ///< Barometric altitude (centimeters)
    float    desired_rangefinder_alt;  ///< Target rangefinder altitude if terrain following (meters)
    float    rangefinder_alt;          ///< Rangefinder measured altitude above ground (meters)
    float    terr_alt;                 ///< Terrain database altitude (meters)
    int16_t  target_climb_rate;        ///< Desired vertical velocity (cm/s)
    int16_t  climb_rate;               ///< Actual vertical velocity from inertial nav (cm/s)
};

/**
 * @brief Log all PID controller states for blimp
 * 
 * @details Writes PID log messages for all position and velocity controllers used in blimp
 *          control. Blimp uses a cascaded control architecture with outer position loops
 *          feeding inner velocity loops for each axis (X/North, Y/East, Z/Down, Yaw).
 * 
 *          PID controllers logged:
 *          - PIVN: Velocity controller North/X axis
 *          - PIVE: Velocity controller East/Y axis  
 *          - PIVD: Velocity controller Down/Z axis
 *          - PIVY: Velocity controller Yaw
 *          - PIDN: Position controller North/X axis
 *          - PIDE: Position controller East/Y axis
 *          - PIDD: Position controller Down/Z axis
 *          - PIDY: Position controller Yaw
 * 
 *          Call frequency: Typically called at main loop rate (50-400Hz)
 * 
 * @note Essential for tuning PID gains and diagnosing control stability issues
 * @see libraries/AP_Logger/AP_Logger.h Write_PID() method for log format
 * @see pid_vel_xy, pid_vel_z, pid_vel_yaw for velocity controllers
 * @see pid_pos_xy, pid_pos_z, pid_pos_yaw for position controllers
 */
void Blimp::Log_Write_PIDs()
{
    logger.Write_PID(LOG_PIVN_MSG, pid_vel_xy.get_pid_info_x());
    logger.Write_PID(LOG_PIVE_MSG, pid_vel_xy.get_pid_info_y());
    logger.Write_PID(LOG_PIVD_MSG, pid_vel_z.get_pid_info());
    logger.Write_PID(LOG_PIVY_MSG, pid_vel_yaw.get_pid_info());
    logger.Write_PID(LOG_PIDN_MSG, pid_pos_xy.get_pid_info_x());
    logger.Write_PID(LOG_PIDE_MSG, pid_pos_xy.get_pid_info_y());
    logger.Write_PID(LOG_PIDD_MSG, pid_pos_z.get_pid_info());
    logger.Write_PID(LOG_PIDY_MSG, pid_pos_yaw.get_pid_info());
}

/**
 * @brief Log vehicle attitude data
 * 
 * @details Writes attitude (roll, pitch, yaw) log message for the blimp. Unlike multirotors
 *          and fixed-wing aircraft, blimps don't use attitude control - they use direct
 *          velocity control. Therefore, attitude targets are always set to zero (0,0,0).
 *          However, the actual measured attitude from AHRS is still logged and useful for
 *          understanding vehicle orientation and detecting excessive tilt.
 * 
 *          Call frequency: Typically called at main loop rate (50-400Hz)
 * 
 * @note Blimp uses velocity control rather than attitude control, so target attitude is
 *       always zero. The logged attitude data shows actual vehicle orientation from AHRS.
 * 
 * @see libraries/AP_AHRS/AP_AHRS.h Write_Attitude() method
 * @see AP_AHRS for attitude estimation system
 */
void Blimp::Log_Write_Attitude()
{
    //Attitude targets are all zero since Blimp doesn't have attitude control,
    //but the rest of the log message is useful.
    ahrs.Write_Attitude(Vector3f{0,0,0});
}

/**
 * @brief Log Extended Kalman Filter and position estimation data
 * 
 * @details Writes EKF state estimation and position data from the AHRS (Attitude Heading
 *          Reference System). The EKF fuses data from IMU, GPS, barometer, compass and
 *          other sensors to estimate vehicle position, velocity and attitude. This log
 *          message is essential for diagnosing navigation issues and EKF health problems.
 * 
 *          Call frequency: Typically called at main loop rate (50-400Hz)
 * 
 * @note The EKF is critical for autonomous navigation - monitor EKF health in logs
 * @see libraries/AP_AHRS/AP_AHRS.h Log_Write() method for log format
 * @see libraries/AP_NavEKF3/ for EKF3 implementation
 */
void Blimp::Log_Write_EKF_POS()
{
    AP::ahrs().Log_Write();
}

/**
 * @struct log_Data_Int16t
 * @brief Generic 16-bit signed integer data log message structure
 * 
 * @details Provides a flexible logging mechanism for arbitrary 16-bit signed integer values
 *          identified by a LogDataID. Used for debugging and logging temporary values
 *          without defining dedicated log message types.
 * 
 * @see Log_Write_Data(LogDataID, int16_t) for the logging function
 */
struct PACKED log_Data_Int16t {
    LOG_PACKET_HEADER;     ///< Standard log packet header with message type and size
    uint64_t time_us;       ///< Timestamp in microseconds since system boot
    uint8_t id;             ///< Data identifier from LogDataID enum
    int16_t data_value;     ///< Signed 16-bit integer value to log
};

/**
 * @brief Log a generic 16-bit signed integer value
 * 
 * @details Writes a D16 log message for debugging purposes. Allows logging arbitrary
 *          int16_t values identified by a LogDataID without creating dedicated log
 *          message structures. Only logs if MASK_LOG_ANY is enabled.
 * 
 * @param[in] id     Data identifier from LogDataID enum
 * @param[in] value  16-bit signed integer value to log (range: -32768 to 32767)
 * 
 * @note Uses WriteCriticalBlock to ensure data is not dropped
 * @note Function marked UNUSED_FUNCTION - may not be actively used in current code
 * @see should_log() for logging mask check
 */
UNUSED_FUNCTION
void Blimp::Log_Write_Data(LogDataID id, int16_t value)
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
 * @brief Generic 16-bit unsigned integer data log message structure
 * 
 * @details Provides a flexible logging mechanism for arbitrary 16-bit unsigned integer
 *          values identified by a LogDataID. Used for debugging and logging temporary
 *          values without defining dedicated log message types.
 * 
 * @see Log_Write_Data(LogDataID, uint16_t) for the logging function
 */
struct PACKED log_Data_UInt16t {
    LOG_PACKET_HEADER;     ///< Standard log packet header with message type and size
    uint64_t time_us;       ///< Timestamp in microseconds since system boot
    uint8_t id;             ///< Data identifier from LogDataID enum
    uint16_t data_value;    ///< Unsigned 16-bit integer value to log
};

/**
 * @brief Log a generic 16-bit unsigned integer value
 * 
 * @details Writes a DU16 log message for debugging purposes. Allows logging arbitrary
 *          uint16_t values identified by a LogDataID without creating dedicated log
 *          message structures. Only logs if MASK_LOG_ANY is enabled.
 * 
 * @param[in] id     Data identifier from LogDataID enum
 * @param[in] value  16-bit unsigned integer value to log (range: 0 to 65535)
 * 
 * @note Uses WriteCriticalBlock to ensure data is not dropped
 * @note Function marked UNUSED_FUNCTION - may not be actively used in current code
 * @see should_log() for logging mask check
 */
UNUSED_FUNCTION
void Blimp::Log_Write_Data(LogDataID id, uint16_t value)
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
 * @brief Generic 32-bit signed integer data log message structure
 * 
 * @details Provides a flexible logging mechanism for arbitrary 32-bit signed integer
 *          values identified by a LogDataID. Used for debugging and logging temporary
 *          values without defining dedicated log message types.
 * 
 * @see Log_Write_Data(LogDataID, int32_t) for the logging function
 */
struct PACKED log_Data_Int32t {
    LOG_PACKET_HEADER;     ///< Standard log packet header with message type and size
    uint64_t time_us;       ///< Timestamp in microseconds since system boot
    uint8_t id;             ///< Data identifier from LogDataID enum
    int32_t data_value;     ///< Signed 32-bit integer value to log
};

/**
 * @brief Log a generic 32-bit signed integer value
 * 
 * @details Writes a D32 log message for debugging purposes. Allows logging arbitrary
 *          int32_t values identified by a LogDataID without creating dedicated log
 *          message structures. Only logs if MASK_LOG_ANY is enabled. Common uses include
 *          logging GPS coordinates (in 1e-7 degrees), timestamps, or other large integers.
 * 
 * @param[in] id     Data identifier from LogDataID enum
 * @param[in] value  32-bit signed integer value to log (range: -2147483648 to 2147483647)
 * 
 * @note Uses WriteCriticalBlock to ensure data is not dropped
 * @see should_log() for logging mask check
 */
void Blimp::Log_Write_Data(LogDataID id, int32_t value)
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
 * @brief Generic 32-bit unsigned integer data log message structure
 * 
 * @details Provides a flexible logging mechanism for arbitrary 32-bit unsigned integer
 *          values identified by a LogDataID. Used for debugging and logging temporary
 *          values without defining dedicated log message types.
 * 
 * @see Log_Write_Data(LogDataID, uint32_t) for the logging function
 */
struct PACKED log_Data_UInt32t {
    LOG_PACKET_HEADER;     ///< Standard log packet header with message type and size
    uint64_t time_us;       ///< Timestamp in microseconds since system boot
    uint8_t id;             ///< Data identifier from LogDataID enum
    uint32_t data_value;    ///< Unsigned 32-bit integer value to log
};

/**
 * @brief Log a generic 32-bit unsigned integer value
 * 
 * @details Writes a DU32 log message for debugging purposes. Allows logging arbitrary
 *          uint32_t values identified by a LogDataID without creating dedicated log
 *          message structures. Only logs if MASK_LOG_ANY is enabled. Common uses include
 *          logging counters, flags, or other unsigned values.
 * 
 * @param[in] id     Data identifier from LogDataID enum
 * @param[in] value  32-bit unsigned integer value to log (range: 0 to 4294967295)
 * 
 * @note Uses WriteCriticalBlock to ensure data is not dropped
 * @see should_log() for logging mask check
 */
void Blimp::Log_Write_Data(LogDataID id, uint32_t value)
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
 * @brief Generic floating-point data log message structure
 * 
 * @details Provides a flexible logging mechanism for arbitrary floating-point values
 *          identified by a LogDataID. Used for debugging and logging temporary values
 *          without defining dedicated log message types.
 * 
 * @see Log_Write_Data(LogDataID, float) for the logging function
 */
struct PACKED log_Data_Float {
    LOG_PACKET_HEADER;     ///< Standard log packet header with message type and size
    uint64_t time_us;       ///< Timestamp in microseconds since system boot
    uint8_t id;             ///< Data identifier from LogDataID enum
    float data_value;       ///< 32-bit floating-point value to log
};

/**
 * @brief Log a generic floating-point value
 * 
 * @details Writes a DFLT log message for debugging purposes. Allows logging arbitrary
 *          float values identified by a LogDataID without creating dedicated log message
 *          structures. Only logs if MASK_LOG_ANY is enabled. Useful for logging sensor
 *          readings, computed values, or debug data during development.
 * 
 * @param[in] id     Data identifier from LogDataID enum
 * @param[in] value  Floating-point value to log (32-bit IEEE 754 format)
 * 
 * @note Uses WriteCriticalBlock to ensure data is not dropped
 * @note Function marked UNUSED_FUNCTION - may not be actively used in current code
 * @see should_log() for logging mask check
 */
UNUSED_FUNCTION
void Blimp::Log_Write_Data(LogDataID id, float value)
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
 * @struct log_ParameterTuning
 * @brief In-flight parameter tuning log message structure
 * 
 * @details Logs parameter values during in-flight tuning operations. Allows pilots to
 *          adjust parameters using RC channel knobs during flight and records the changes
 *          for later analysis. Essential for iterative tuning of PID gains and other
 *          control parameters.
 * 
 * @see Log_Write_Parameter_Tuning() for the logging function
 * @see https://ardupilot.org/blimp/docs/tuning.html#in-flight-tuning for usage guide
 */
struct PACKED log_ParameterTuning {
    LOG_PACKET_HEADER;                ///< Standard log packet header with message type and size
    uint64_t time_us;                  ///< Timestamp in microseconds since system boot
    uint8_t  parameter;                ///< Parameter being tuned, e.g. 39 is CH6_CIRCLE_RATE
    float    tuning_value;             ///< Normalized value used inside tuning() function
    float    tuning_min;               ///< Tuning minimum limit for this parameter
    float    tuning_max;               ///< Tuning maximum limit for this parameter
};

/**
 * @brief Log in-flight parameter tuning adjustments
 * 
 * @details Writes a PTUN log message recording parameter changes during in-flight tuning.
 *          This allows pilots to adjust control parameters using RC transmitter knobs while
 *          flying and provides a record of the tested values. Critical for safe parameter
 *          tuning as it documents the correlation between parameter values and vehicle behavior.
 * 
 * @param[in] param       Parameter identifier being tuned
 * @param[in] tuning_val  Current normalized tuning value (typically 0-1 range)
 * @param[in] tune_min    Minimum value limit for this tuning parameter
 * @param[in] tune_max    Maximum value limit for this tuning parameter
 * 
 * @note Tuning_val is normalized but tune_min/tune_max are in actual parameter units
 * @see https://ardupilot.org/blimp/docs/tuning.html#in-flight-tuning
 */
void Blimp::Log_Write_Parameter_Tuning(uint8_t param, float tuning_val, float tune_min, float tune_max)
{
    struct log_ParameterTuning pkt_tune = {
        LOG_PACKET_HEADER_INIT(LOG_PARAMTUNE_MSG),
time_us        : AP_HAL::micros64(),
parameter      : param,
tuning_value   : tuning_val,
tuning_min     : tune_min,
tuning_max     : tune_max
    };

    logger.WriteBlock(&pkt_tune, sizeof(pkt_tune));
}

/**
 * @brief Log message structure definitions for Blimp vehicle type
 * 
 * @details Defines the binary format for all log messages specific to the Blimp vehicle.
 *          Each entry specifies:
 *          - Message ID and size
 *          - Message name (4 characters)
 *          - Format string (data types for each field)
 *          - Label string (field names)
 *          - Units string (units for each field)
 *          - Multipliers string (scaling factors for each field)
 * 
 *          Type and unit information can be found in libraries/AP_Logger/LogStructure.h:
 *          - Search for "log_Units" for unit definitions
 *          - Search for "Format characters" for field type information
 * 
 *          Format characters:
 *          - Q: uint64_t (8 bytes)
 *          - f: float (4 bytes)
 *          - e: int32_t (4 bytes)
 *          - h: int16_t (2 bytes)
 *          - B: uint8_t (1 byte)
 *          - i: int32_t (4 bytes)
 *          - H: uint16_t (2 bytes)
 *          - I: uint32_t (4 bytes)
 * 
 * @note LOG_COMMON_STRUCTURES provides standard messages shared across all vehicles
 * @see libraries/AP_Logger/LogStructure.h for format details
 * @see AP_Logger documentation for log analysis tools
 */
// type and unit information can be found in
// libraries/AP_Logger/Logstructure.h; search for "log_Units" for
// units and "Format characters" for field type information
const struct LogStructure Blimp::log_structure[] = {
    LOG_COMMON_STRUCTURES,

    // @LoggerMessage: FINI
    // @Description: Fin input
    // @Field: TimeUS: Time since system startup
    // @Field: R: Right
    // @Field: F: Front
    // @Field: D: Down
    // @Field: Y: Yaw

    {
        LOG_FINI_MSG, sizeof(log_FINI),
        "FINI",  "Qffff",     "TimeUS,R,F,D,Y", "s----", "F----"
    },

    // @LoggerMessage: FINO
    // @Description: Fin output
    // @Field: TimeUS: Time since system startup
    // @Field: F1A: Fin 1 Amplitude
    // @Field: F1O: Fin 1 Offset
    // @Field: F2A: Fin 2 Amplitude
    // @Field: F2O: Fin 2 Offset
    // @Field: F3A: Fin 3 Amplitude
    // @Field: F3O: Fin 3 Offset
    // @Field: F4A: Fin 4 Amplitude
    // @Field: F4O: Fin 4 Offset

    {
        LOG_FINO_MSG, sizeof(log_FINO),
        "FINO",  "Qffffffff",     "TimeUS,F1A,F1O,F2A,F2O,F3A,F3O,F4A,F4O", "s--------", "F--------"
    },

    // @LoggerMessage: PIDD,PIVN,PIVE,PIVD,PIVY
    // @Description: Proportional/Integral/Derivative gain values
    // @Field: TimeUS: Time since system startup
    // @Field: Tar: desired value
    // @Field: Act: achieved value
    // @Field: Err: error between target and achieved
    // @Field: P: proportional part of PID
    // @Field: I: integral part of PID
    // @Field: D: derivative part of PID
    // @Field: FF: controller feed-forward portion of response
    // @Field: DFF: controller derivative feed-forward portion of response
    // @Field: Dmod: scaler applied to D gain to reduce limit cycling
    // @Field: SRate: slew rate
    // @Field: Flags: bitmask of PID state flags
    // @FieldBitmaskEnum: Flags: log_PID_Flags
    {
        LOG_PIDD_MSG, sizeof(log_PID),
        "PIDD", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS
    },
    {
        LOG_PIVN_MSG, sizeof(log_PID),
        "PIVN", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS
    },
    {
        LOG_PIVE_MSG, sizeof(log_PID),
        "PIVE", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS
    },
    {
        LOG_PIVD_MSG, sizeof(log_PID),
        "PIVD", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS
    },
    {
        LOG_PIVY_MSG, sizeof(log_PID),
        "PIVY", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS
    },

    // @LoggerMessage: PTUN
    // @Description: Parameter Tuning information
    // @URL: https://ardupilot.org/blimp/docs/tuning.html#in-flight-tuning
    // @Field: TimeUS: Time since system startup
    // @Field: Param: Parameter being tuned
    // @Field: TunVal: Normalized value used inside tuning() function
    // @Field: TunMin: Tuning minimum limit
    // @Field: TunMax: Tuning maximum limit

    {
        LOG_PARAMTUNE_MSG, sizeof(log_ParameterTuning),
        "PTUN", "QBfff",         "TimeUS,Param,TunVal,TunMin,TunMax", "s----", "F----"
    },

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

    // @LoggerMessage: DU16
    // @Description: Generic 16-bit-unsigned-integer storage
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

    // @LoggerMessage: DU32
    // @Description: Generic 32-bit-unsigned-integer storage
    // @Field: TimeUS: Time since system startup
    // @Field: Id: Data type identifier
    // @Field: Value: Value

    {
        LOG_CONTROL_TUNING_MSG, sizeof(log_Control_Tuning),
        "CTUN", "Qffffffefffhh", "TimeUS,ThI,ABst,ThO,ThH,DAlt,Alt,BAlt,DSAlt,SAlt,TAlt,DCRt,CRt", "s----mmmmmmnn", "F----00B000BB"
    },

    {
        LOG_DATA_INT16_MSG, sizeof(log_Data_Int16t),
        "D16",   "QBh",         "TimeUS,Id,Value", "s--", "F--"
    },
    {
        LOG_DATA_UINT16_MSG, sizeof(log_Data_UInt16t),
        "DU16",  "QBH",         "TimeUS,Id,Value", "s--", "F--"
    },
    {
        LOG_DATA_INT32_MSG, sizeof(log_Data_Int32t),
        "D32",   "QBi",         "TimeUS,Id,Value", "s--", "F--"
    },
    {
        LOG_DATA_UINT32_MSG, sizeof(log_Data_UInt32t),
        "DU32",  "QBI",         "TimeUS,Id,Value", "s--", "F--"
    },
    {
        LOG_DATA_FLOAT_MSG, sizeof(log_Data_Float),
        "DFLT",  "QBf",         "TimeUS,Id,Value", "s--", "F--"
    },
};

/**
 * @brief Get the number of log structure definitions
 * 
 * @details Returns the count of log message structures defined in the log_structure array.
 *          Used by the AP_Logger system during initialization to register all blimp-specific
 *          log message formats. This count includes both blimp-specific messages and common
 *          messages inherited from LOG_COMMON_STRUCTURES.
 * 
 * @return Number of log structures in the log_structure array
 * 
 * @note This is called during logger initialization to register message formats
 * @see log_structure array for the complete list of message definitions
 */
uint8_t Blimp::get_num_log_structures() const
{
    return ARRAY_SIZE(log_structure);

}

/**
 * @brief Log vehicle startup and configuration messages
 * 
 * @details Writes essential vehicle configuration and state information to the log at
 *          startup or when logging begins. This provides context for log analysis by
 *          recording the initial vehicle state. Messages include:
 *          - Frame type configuration
 *          - Initial flight mode and reason for mode
 *          - Home position and EKF origin
 *          - GPS startup information
 * 
 *          Call frequency: Called once at logging initialization
 * 
 * @note Only first ~200 bytes of messages are guaranteed by AP_Logger buffer
 * @warning Keep startup messages concise to avoid buffer overflow
 * 
 * @see logger.Write_MessageF() for text message logging
 * @see get_frame_string() for frame type identification
 * @see control_mode for current flight mode
 */
void Blimp::Log_Write_Vehicle_Startup_Messages()
{
    // only 200(?) bytes are guaranteed by AP_Logger
    logger.Write_MessageF("Frame: %s", get_frame_string());
    logger.Write_Mode((uint8_t)control_mode, control_mode_reason);
    ahrs.Log_Write_Home_And_Origin();
    gps.Write_AP_Logger_Log_Startup_messages();
}

#endif // HAL_LOGGING_ENABLED
