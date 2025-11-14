/**
 * @file Log.cpp
 * @brief ArduCopter dataflash logging implementation
 * 
 * @details This file implements the copter-specific logging functions that write
 *          telemetry data to the AP_Logger binary dataflash logging system.
 *          These logs are essential for post-flight analysis, PID tuning, and
 *          debugging flight issues.
 * 
 *          Key copter-specific log messages:
 *          - CTUN (Control Tuning): Altitude control, throttle, climb rates
 *          - PIDS: PID controller performance data for rate controllers
 *          - GUIP/GUIA: Guided mode position and attitude targets
 *          - SIDD/SIDS: System identification data for autotuning
 * 
 *          Log messages are written at various rates:
 *          - Fast loop (400Hz): Attitude, rate data
 *          - Medium loop (10-50Hz): Control tuning, navigation
 *          - Event-driven: Mode changes, parameter tuning, guided commands
 * 
 *          All log structures integrate with AP_Logger which handles:
 *          - Binary encoding and storage management
 *          - Log download via MAVLink or direct SD card access
 *          - Message format definitions for ground station tools
 * 
 * @note Log messages use packed structures to minimize storage space
 * @note Field formats defined in log_structure array match AP_Logger conventions
 * @see libraries/AP_Logger for the underlying logging infrastructure
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include "Copter.h"
#include <AP_InertialSensor/AP_InertialSensor_rate_config.h>

#if HAL_LOGGING_ENABLED

// Code to Write and Read packets from AP_Logger log memory
// Code to interact with the user to dump or erase logs

/**
 * @struct log_Control_Tuning
 * @brief CTUN log message structure for altitude and throttle control tuning data
 * 
 * @details This structure defines the CTUN (Control Tuning) log message which records
 *          critical altitude control performance data for multicopter flight. This is
 *          one of the most important log messages for tuning altitude hold performance,
 *          analyzing climb/descent behavior, and diagnosing altitude control issues.
 * 
 *          The CTUN message combines data from multiple sources:
 *          - Attitude controller: throttle inputs and angle boost
 *          - Position controller: altitude targets and estimates
 *          - Motor mixer: actual throttle output and hover throttle
 *          - Sensor systems: barometer, rangefinder, terrain database
 * 
 *          Common uses in post-flight analysis:
 *          - Compare desired_alt vs inav_alt to assess altitude tracking
 *          - Monitor throttle_hover to verify hover throttle learning
 *          - Analyze climb_rate vs target_climb_rate for rate controller tuning
 *          - Check angle_boost during aggressive maneuvers
 *          - Verify rangefinder-based surface tracking performance
 * 
 * @note Logged at approximately 10-25 Hz depending on LOG_BITMASK setting
 * @note All altitude values in different frames to support various flight modes
 * @warning throttle_hover learning affects altitude hold - sudden changes indicate issues
 */
struct PACKED log_Control_Tuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;                  ///< Timestamp in microseconds since system boot
    float    throttle_in;              ///< Pilot throttle input (0.0-1.0, or climb rate in alt-hold modes)
    float    angle_boost;              ///< Additional throttle for aggressive attitude angles (0.0-1.0)
    float    throttle_out;             ///< Final throttle output sent to motors (0.0-1.0)
    float    throttle_hover;           ///< Learned hover throttle value (0.0-1.0)
    float    desired_alt;              ///< Target altitude in meters above EKF origin
    float    inav_alt;                 ///< Estimated altitude in meters from position controller
    int32_t  baro_alt;                 ///< Barometric altitude in centimeters above home
    float    desired_rangefinder_alt;  ///< Target distance from ground in meters (surface tracking)
    float    rangefinder_alt;          ///< Measured distance from ground in meters (rangefinder)
    float    terr_alt;                 ///< Altitude above terrain in meters (terrain database)
    int16_t  target_climb_rate;        ///< Desired climb rate in cm/s
    int16_t  climb_rate;               ///< Actual climb rate in cm/s from position estimate
};

/**
 * @brief Write CTUN (Control Tuning) log message for altitude control analysis
 * 
 * @details This function gathers altitude control data from multiple subsystems and
 *          writes a CTUN log message to the dataflash. The CTUN message is essential
 *          for analyzing altitude hold performance, tuning vertical position/velocity
 *          controllers, and diagnosing altitude-related flight issues.
 * 
 *          Data collection process:
 *          1. Retrieves terrain altitude from terrain database (if available)
 *          2. Gets desired altitude and climb rate from position controller (if not manual throttle)
 *          3. Collects surface tracking targets from rangefinder system
 *          4. Gathers throttle data from attitude controller and motor mixer
 *          5. Records current altitude estimates from multiple sources
 * 
 *          Altitude sources logged:
 *          - desired_alt: Target from position controller (NED U axis, meters)
 *          - inav_alt: Inertial nav estimate (NED U axis, meters)
 *          - baro_alt: Barometer-only altitude (centimeters above home)
 *          - rangefinder_alt: Distance to ground (meters, from rangefinder)
 *          - terr_alt: Height above terrain (meters, from terrain database)
 * 
 *          The CTUN message combines multiple coordinate frames:
 *          - EKF origin frame: desired_alt, inav_alt (NED convention: U is down)
 *          - Home frame: baro_alt (up is positive)
 *          - Ground-relative: rangefinder_alt, terr_alt (up is positive)
 * 
 * @note Called from medium-speed loop (typically 10-25 Hz) when MASK_LOG_CTUN is enabled
 * @note Writes quiet_nan() for unavailable sensors (terrain, rangefinder)
 * @note Altitude targets only valid when not in manual throttle mode
 * 
 * @see log_Control_Tuning for field descriptions and units
 * @see AC_PosControl for position controller altitude targets
 * @see AP_Terrain for terrain altitude database
 */
void Copter::Log_Write_Control_Tuning()
{
    // get terrain altitude
    float terr_alt = 0.0f;
#if AP_TERRAIN_AVAILABLE
    if (!terrain.height_above_terrain(terr_alt, true)) {
        terr_alt = logger.quiet_nan();
    }
#endif
    float des_alt_m = 0.0f;
    int16_t target_climb_rate_cms = 0;
    if (!flightmode->has_manual_throttle()) {
        des_alt_m = pos_control->get_pos_target_U_cm() * 0.01f;
        target_climb_rate_cms = pos_control->get_vel_target_U_cms();
    }

    float desired_rangefinder_alt;
#if AP_RANGEFINDER_ENABLED
    if (!surface_tracking.get_target_dist_for_logging(desired_rangefinder_alt)) {
        desired_rangefinder_alt = AP::logger().quiet_nan();
    }
#else
    // get surface tracking alts
    desired_rangefinder_alt = AP::logger().quiet_nan();
#endif

    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CONTROL_TUNING_MSG),
        time_us             : AP_HAL::micros64(),
        throttle_in         : attitude_control->get_throttle_in(),
        angle_boost         : attitude_control->angle_boost(),
        throttle_out        : motors->get_throttle(),
        throttle_hover      : motors->get_throttle_hover(),
        desired_alt         : des_alt_m,
        inav_alt            : float(pos_control->get_pos_estimate_NEU_cm().z) * 0.01,
        baro_alt            : baro_alt,
        desired_rangefinder_alt : desired_rangefinder_alt,
#if AP_RANGEFINDER_ENABLED
        rangefinder_alt     : surface_tracking.get_dist_for_logging(),
#else
        rangefinder_alt     : AP::logger().quiet_nanf(),
#endif
        terr_alt            : terr_alt,
        target_climb_rate   : target_climb_rate_cms,
        climb_rate          : int16_t(pos_control->get_vel_estimate_NEU_cms().z) // float -> int16_t
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

/**
 * @brief Write attitude angle controller log message (ANG)
 * 
 * @details Delegates to the attitude controller to write angle controller performance data.
 *          The ANG message contains attitude targets (desired roll, pitch, yaw) and actual
 *          achieved attitudes, enabling analysis of attitude tracking performance.
 * 
 *          This is a critical high-frequency log message (typically 10-400 Hz) used for:
 *          - Verifying attitude command tracking
 *          - Analyzing controller responsiveness
 *          - Diagnosing attitude oscillations or drift
 *          - Tuning attitude PID gains
 * 
 * @note Logging rate controlled by MASK_LOG_ATTITUDE_FAST or MASK_LOG_ATTITUDE_MED
 * @note Actual message format defined in AC_AttitudeControl library
 * 
 * @see AC_AttitudeControl::Write_ANG() for message structure and fields
 * @see Log_Write_Rate() for rate controller data
 */
void Copter::Log_Write_Attitude()
{
    attitude_control->Write_ANG();
}

/**
 * @brief Write rate controller log message (RATE)
 * 
 * @details Delegates to the attitude controller to write rate controller performance data,
 *          passing the position controller reference for additional context. The RATE message
 *          contains angular rate targets and achieved rates for roll, pitch, and yaw axes.
 * 
 *          This high-frequency log message (typically 400 Hz - same as main loop) is essential for:
 *          - Analyzing gyro-based rate control performance
 *          - Tuning rate PID controllers (RATE_RLL_P, RATE_PIT_P, RATE_YAW_P, etc.)
 *          - Diagnosing high-frequency oscillations or control instability
 *          - Verifying motor response to control commands
 * 
 *          The rate controller is the innermost loop in the cascade control architecture:
 *          Position Controller → Attitude Controller → Rate Controller → Motor Mixer
 * 
 * @note Logged at fast loop rate when MASK_LOG_ATTITUDE_FAST is enabled
 * @note Rate control runs at main loop frequency (typically 400 Hz)
 * @note Message format defined in AC_AttitudeControl library
 * 
 * @see AC_AttitudeControl::Write_Rate() for message structure
 * @see Log_Write_Attitude() for angle controller data
 * @see Log_Write_PIDS() for detailed PID component analysis
 */
void Copter::Log_Write_Rate()
{
    attitude_control->Write_Rate(*pos_control);
}

/**
 * @brief Write PID controller component analysis log messages
 * 
 * @details Writes detailed PID controller performance data for multiple control loops,
 *          breaking down P (proportional), I (integral), D (derivative), and FF (feedforward)
 *          components for analysis. This is essential for systematic PID tuning and
 *          diagnosing control loop performance issues.
 * 
 *          PID messages written (when MASK_LOG_PID enabled):
 *          - PIDR: Roll rate controller components (400 Hz rate loop)
 *          - PIDP: Pitch rate controller components (400 Hz rate loop)
 *          - PIDY: Yaw rate controller components (400 Hz rate loop)
 *          - PIDA: Altitude acceleration controller (position control loop)
 * 
 *          Additional navigation PIDs (when MASK_LOG_NTUN enabled + GPS flight):
 *          - PIDN: North velocity controller (horizontal position control)
 *          - PIDE: East velocity controller (horizontal position control)
 * 
 *          Each PID message includes:
 *          - Target: Desired setpoint for the controller
 *          - Actual: Measured value from sensors
 *          - P: Proportional term contribution
 *          - I: Integral term contribution (accumulated error)
 *          - D: Derivative term contribution (rate of change)
 *          - FF: Feedforward term (predictive component)
 *          - Error: Current tracking error (target - actual)
 * 
 *          PID tuning workflow using these logs:
 *          1. Start with P term - increase until oscillation appears, then back off
 *          2. Add D term to dampen oscillations and improve responsiveness
 *          3. Add I term to eliminate steady-state error (use cautiously)
 *          4. Tune FF term for faster response in position/velocity controllers
 * 
 * @note Logged at fast loop rate (typically 400 Hz) when enabled
 * @note Navigation PIDs only logged when GPS-dependent flight modes active
 * @note I-term growth indicates consistent tracking error or misconfiguration
 * @note D-term spikes indicate measurement noise or too-high D gain
 * 
 * @warning Excessive I-term accumulation can cause control instability (windup)
 * 
 * @see AC_AttitudeControl::get_rate_roll_pid() for rate controller details
 * @see AC_PosControl::get_accel_U_pid() for altitude controller details
 * @see AC_PosControl::get_vel_NE_pid() for horizontal velocity controller
 */
void Copter::Log_Write_PIDS()
{
   if (should_log(MASK_LOG_PID)) {
        logger.Write_PID(LOG_PIDR_MSG, attitude_control->get_rate_roll_pid().get_pid_info());
        logger.Write_PID(LOG_PIDP_MSG, attitude_control->get_rate_pitch_pid().get_pid_info());
        logger.Write_PID(LOG_PIDY_MSG, attitude_control->get_rate_yaw_pid().get_pid_info());
        logger.Write_PID(LOG_PIDA_MSG, pos_control->get_accel_U_pid().get_pid_info() );
        if (should_log(MASK_LOG_NTUN) && (flightmode->requires_GPS() || landing_with_GPS())) {
            logger.Write_PID(LOG_PIDN_MSG, pos_control->get_vel_NE_pid().get_pid_info_x());
            logger.Write_PID(LOG_PIDE_MSG, pos_control->get_vel_NE_pid().get_pid_info_y());
        }
    }
}

/**
 * @brief Write Extended Kalman Filter and position estimate log messages
 * 
 * @details Delegates to the AHRS (Attitude and Heading Reference System) to write
 *          comprehensive EKF state estimation data. This includes EKF innovations,
 *          covariances, and position/velocity estimates critical for diagnosing
 *          navigation and sensor fusion issues.
 * 
 *          AHRS logging includes multiple message types:
 *          - EKF: EKF status, innovations, and variance data
 *          - POS: Position and velocity estimates in NED frame
 *          - Additional backend-specific messages (EKF2, EKF3, etc.)
 * 
 *          These messages are essential for:
 *          - Diagnosing EKF lane switching or failovers
 *          - Analyzing GPS glitches and their impact on position estimate
 *          - Monitoring innovation sequences (prediction error checks)
 *          - Verifying sensor health and fusion quality
 *          - Debugging position estimate drift or jumps
 * 
 * @note Logging rate depends on AHRS backend and log bitmask settings
 * @note Message format varies by EKF version (EKF2, EKF3, DCM)
 * 
 * @see AP_AHRS::Log_Write() for complete message details
 * @see AP_NavEKF3 for EKF3 innovation and covariance details
 */
void Copter::Log_Write_EKF_POS()
{
    AP::ahrs().Log_Write();
}

/**
 * @struct log_Data_Int16t
 * @brief D16 log message for generic 16-bit signed integer storage
 * 
 * @details Generic data logging structure for ad-hoc debugging and development.
 *          Allows logging arbitrary 16-bit signed values with an ID to distinguish
 *          between different data sources or purposes.
 * 
 * @note Typically used during development for temporary debugging
 * @note ID field allows multiplexing multiple data sources into one message type
 */
struct PACKED log_Data_Int16t {
    LOG_PACKET_HEADER;
    uint64_t time_us;      ///< Timestamp in microseconds since system boot
    uint8_t id;            ///< Data identifier to distinguish different logged values
    int16_t data_value;    ///< Signed 16-bit data value
};

/**
 * @brief Write generic 16-bit signed integer data packet (D16)
 * 
 * @details Writes a D16 log message containing an arbitrary signed 16-bit value
 *          with an identifier. Used for temporary debugging during development
 *          or logging custom data that doesn't fit standard message structures.
 * 
 *          Uses WriteCriticalBlock to ensure high-priority logging even when
 *          buffer is nearly full.
 * 
 * @param[in] id Data identifier (enum LogDataID) to tag the data source
 * @param[in] value Signed 16-bit integer value to log
 * 
 * @note Only logs if MASK_LOG_ANY is enabled in LOG_BITMASK parameter
 * @note Uses critical block write for guaranteed logging
 * @note UNUSED_FUNCTION marker indicates this may not be actively used
 */
UNUSED_FUNCTION
void Copter::Log_Write_Data(LogDataID id, int16_t value)
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
 * @brief DU16 log message for generic 16-bit unsigned integer storage
 * 
 * @details Generic data logging structure for 16-bit unsigned values, useful for
 *          counters, flags, or small positive-only measurements during debugging.
 */
struct PACKED log_Data_UInt16t {
    LOG_PACKET_HEADER;
    uint64_t time_us;      ///< Timestamp in microseconds since system boot
    uint8_t id;            ///< Data identifier to distinguish different logged values
    uint16_t data_value;   ///< Unsigned 16-bit data value
};

/**
 * @brief Write generic 16-bit unsigned integer data packet (DU16)
 * 
 * @details Writes a DU16 log message for debugging or custom unsigned 16-bit data.
 *          Similar to Log_Write_Data(int16_t) but for unsigned values.
 * 
 * @param[in] id Data identifier (enum LogDataID) to tag the data source
 * @param[in] value Unsigned 16-bit integer value to log (0-65535)
 * 
 * @note Only logs if MASK_LOG_ANY is enabled
 * @note UNUSED_FUNCTION marker indicates this may not be actively used
 */
UNUSED_FUNCTION 
void Copter::Log_Write_Data(LogDataID id, uint16_t value)
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
 * @brief D32 log message for generic 32-bit signed integer storage
 * 
 * @details Generic data logging structure for larger signed integer values, commonly
 *          used for GPS coordinates in degrees * 1e7, timestamps, or large counters.
 */
struct PACKED log_Data_Int32t {
    LOG_PACKET_HEADER;
    uint64_t time_us;      ///< Timestamp in microseconds since system boot
    uint8_t id;            ///< Data identifier to distinguish different logged values
    int32_t data_value;    ///< Signed 32-bit data value
};

/**
 * @brief Write generic 32-bit signed integer data packet (D32)
 * 
 * @details Writes a D32 log message for debugging or logging large signed integer values.
 *          Commonly used for GPS coordinates, large timestamps, or other 32-bit signed data.
 * 
 * @param[in] id Data identifier (enum LogDataID) to tag the data source
 * @param[in] value Signed 32-bit integer value to log
 * 
 * @note Only logs if MASK_LOG_ANY is enabled
 * @note Uses critical block write for guaranteed logging
 */
void Copter::Log_Write_Data(LogDataID id, int32_t value)
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
 * @brief DU32 log message for generic 32-bit unsigned integer storage
 * 
 * @details Generic data logging structure for large unsigned integer values, useful
 *          for timestamps, large counters, or bit fields during debugging.
 */
struct PACKED log_Data_UInt32t {
    LOG_PACKET_HEADER;
    uint64_t time_us;      ///< Timestamp in microseconds since system boot
    uint8_t id;            ///< Data identifier to distinguish different logged values
    uint32_t data_value;   ///< Unsigned 32-bit data value
};

/**
 * @brief Write generic 32-bit unsigned integer data packet (DU32)
 * 
 * @details Writes a DU32 log message for debugging or logging large unsigned values
 *          such as timestamps, counters, or status bit fields.
 * 
 * @param[in] id Data identifier (enum LogDataID) to tag the data source
 * @param[in] value Unsigned 32-bit integer value to log (0-4294967295)
 * 
 * @note Only logs if MASK_LOG_ANY is enabled
 * @note Uses critical block write for guaranteed logging
 */
void Copter::Log_Write_Data(LogDataID id, uint32_t value)
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
 * @brief DFLT log message for generic floating-point value storage
 * 
 * @details Generic data logging structure for IEEE 754 single-precision floating-point
 *          values, useful for logging arbitrary sensor readings or calculated values
 *          during development and debugging.
 */
struct PACKED log_Data_Float {
    LOG_PACKET_HEADER;
    uint64_t time_us;      ///< Timestamp in microseconds since system boot
    uint8_t id;            ///< Data identifier to distinguish different logged values
    float data_value;      ///< IEEE 754 single-precision float value
};

/**
 * @brief Write generic floating-point data packet (DFLT)
 * 
 * @details Writes a DFLT log message for debugging or logging arbitrary float values
 *          such as sensor readings, calculations, or intermediate results.
 * 
 * @param[in] id Data identifier (enum LogDataID) to tag the data source
 * @param[in] value Single-precision floating-point value to log
 * 
 * @note Only logs if MASK_LOG_ANY is enabled
 * @note Uses critical block write for guaranteed logging
 * @note UNUSED_FUNCTION marker indicates this may not be actively used
 */
UNUSED_FUNCTION
void Copter::Log_Write_Data(LogDataID id, float value)
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
 * @brief PTUN log message for in-flight parameter tuning data
 * 
 * @details Structure for logging in-flight parameter adjustment via transmitter channel
 *          (typically channel 6). Records which parameter is being tuned and the range
 *          being explored, enabling analysis of tuning effectiveness post-flight.
 * 
 *          In-flight tuning allows real-time adjustment of parameters like:
 *          - Rate PID gains (P, I, D for roll, pitch, yaw)
 *          - Position/velocity controller gains
 *          - Navigation parameters (waypoint speed, radius, etc.)
 *          - Filter settings (notch filter frequency, etc.)
 * 
 * @note Enables safe parameter exploration without reflashing firmware
 * @see https://ardupilot.org/copter/docs/tuning.html#in-flight-tuning
 */
struct PACKED log_ParameterTuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;       ///< Timestamp in microseconds since system boot
    uint8_t  parameter;     ///< Parameter ID being tuned (e.g. 39 is CH6_CIRCLE_RATE)
    float    tuning_value;  ///< Normalized value (0.0-1.0) from RC input
    float    tuning_min;    ///< Minimum value of parameter tuning range
    float    tuning_max;    ///< Maximum value of parameter tuning range
};

/**
 * @brief Write in-flight parameter tuning log message (PTUN)
 * 
 * @details Logs the current state of in-flight parameter tuning via transmitter knob.
 *          Records which parameter is being adjusted, the normalized input value, and
 *          the actual parameter range being swept.
 * 
 *          Post-flight analysis workflow:
 *          1. Correlate PTUN tuning_value with flight behavior
 *          2. Identify optimal parameter value from performance metrics
 *          3. Update parameter in ground station to optimal value
 *          4. Verify improved performance on subsequent flights
 * 
 * @param[in] param Parameter ID being tuned (from tune parameter enumeration)
 * @param[in] tuning_val Normalized tuning input (0.0-1.0) from RC channel
 * @param[in] tune_min Minimum value in the tuning range (maps to 0.0)
 * @param[in] tune_max Maximum value in the tuning range (maps to 1.0)
 * 
 * @note Logged whenever in-flight tuning is active
 * @note Actual parameter value = tune_min + (tuning_val * (tune_max - tune_min))
 * 
 * @see Copter::tuning() for in-flight tuning implementation
 */
void Copter::Log_Write_Parameter_Tuning(uint8_t param, float tuning_val, float tune_min, float tune_max)
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
 * @brief Write video stabilization attitude data log message
 * 
 * @details Delegates to AHRS to write high-rate attitude data specifically for
 *          post-processing video stabilization. This data can be used with tools
 *          like Gyroflow to digitally stabilize FPV or action camera footage.
 * 
 *          Video stabilization logging provides:
 *          - High-rate gyro data synchronized with timestamps
 *          - Attitude information for frame-by-frame stabilization
 *          - Camera orientation data for perspective correction
 * 
 *          Typical workflow:
 *          1. Enable MASK_LOG_VIDEO_STABILISATION before flight
 *          2. Record video with action camera
 *          3. Download dataflash log after flight
 *          4. Use stabilization software (Gyroflow, etc.) to process video
 *          5. Align log timestamps with video timestamps for correction
 * 
 * @note Only logs if MASK_LOG_VIDEO_STABILISATION bit set in LOG_BITMASK
 * @note Requires high logging rate for smooth stabilization results
 * @note Message format defined in AP_AHRS
 * 
 * @see AP_AHRS::write_video_stabilisation() for message details
 */
void Copter::Log_Video_Stabilisation()
{
    if (!should_log(MASK_LOG_VIDEO_STABILISATION)) {
        return;
    }
    ahrs.write_video_stabilisation();
}

/**
 * @struct log_SysIdD
 * @brief SIDD log message for system identification test data
 * 
 * @details Records system identification input waveform and vehicle response data used
 *          for automated controller tuning. System ID mode applies swept-frequency
 *          excitation (chirp) to each axis and measures the vehicle's dynamic response.
 * 
 *          The data includes:
 *          - Input chirp waveform amplitude and frequency
 *          - Vehicle angular response (delta angles on each axis)
 *          - Vehicle linear response (delta velocities on each axis)
 * 
 *          This data enables frequency domain analysis to determine:
 *          - System natural frequencies and resonances
 *          - Phase margins for stability
 *          - Optimal PID gains based on measured dynamics
 *          - Damping characteristics and response times
 * 
 * @note Used in MODE_SYSTEMID flight mode for automated frequency sweep testing
 * @note Logged at high rate (400 Hz) during system ID tests
 * @see Mode_SystemID for system identification implementation
 */
struct PACKED log_SysIdD {
    LOG_PACKET_HEADER;
    uint64_t time_us;           ///< Timestamp in microseconds since system boot
    float    waveform_time;     ///< Time reference within the chirp waveform (seconds)
    float    waveform_sample;   ///< Current chirp waveform amplitude sample
    float    waveform_freq;     ///< Instantaneous chirp frequency (Hz)
    float    angle_x;           ///< Delta angle response, roll axis (radians)
    float    angle_y;           ///< Delta angle response, pitch axis (radians)
    float    angle_z;           ///< Delta angle response, yaw axis (radians)
    float    accel_x;           ///< Delta velocity response, X-axis (m/s)
    float    accel_y;           ///< Delta velocity response, Y-axis (m/s)
    float    accel_z;           ///< Delta velocity response, Z-axis (m/s)
};

/**
 * @brief Write system identification measurement data packet (SIDD)
 * 
 * @details Logs real-time system identification data during a chirp excitation test.
 *          Records the input waveform and vehicle's angular/linear response for
 *          post-flight frequency domain analysis and automated PID tuning.
 * 
 *          System ID process:
 *          1. Apply swept-frequency chirp to one axis
 *          2. Record vehicle response at high rate (400 Hz)
 *          3. Post-flight: FFT analysis to identify frequency response
 *          4. Calculate optimal PID gains from frequency response
 *          5. Verify gains with additional test flights
 * 
 * @param[in] waveform_time Time index within chirp waveform (seconds)
 * @param[in] waveform_sample Current amplitude of chirp input
 * @param[in] waveform_freq Instantaneous chirp frequency (Hz)
 * @param[in] angle_x Roll axis delta angle response (radians)
 * @param[in] angle_y Pitch axis delta angle response (radians)
 * @param[in] angle_z Yaw axis delta angle response (radians)
 * @param[in] accel_x X-axis delta velocity response (m/s)
 * @param[in] accel_y Y-axis delta velocity response (m/s)
 * @param[in] accel_z Z-axis delta velocity response (m/s)
 * 
 * @note Only available when MODE_SYSTEMID_ENABLED is compiled in
 * @note Logged at 400 Hz during active system ID tests
 * @note Requires SIDS setup message logged first to define test parameters
 * 
 * @see Log_Write_SysID_Setup() for chirp configuration parameters
 * @see Mode_SystemID for system identification mode implementation
 */
void Copter::Log_Write_SysID_Data(float waveform_time, float waveform_sample, float waveform_freq, float angle_x, float angle_y, float angle_z, float accel_x, float accel_y, float accel_z)
{
#if MODE_SYSTEMID_ENABLED
    struct log_SysIdD pkt_sidd = {
        LOG_PACKET_HEADER_INIT(LOG_SYSIDD_MSG),
        time_us         : AP_HAL::micros64(),
        waveform_time   : waveform_time,
        waveform_sample : waveform_sample,
        waveform_freq   : waveform_freq,
        angle_x         : angle_x,
        angle_y         : angle_y,
        angle_z         : angle_z,
        accel_x         : accel_x,
        accel_y         : accel_y,
        accel_z         : accel_z
    };
    logger.WriteBlock(&pkt_sidd, sizeof(pkt_sidd));
#endif
}

/**
 * @struct log_SysIdS
 * @brief SIDS log message for system identification test configuration
 * 
 * @details Records the parameters of a system identification chirp test before it begins.
 *          This setup information is essential for post-flight analysis tools to properly
 *          interpret the SIDD measurement data.
 * 
 *          Chirp waveform phases:
 *          1. Fade-in: Gradually increase amplitude from 0 to full magnitude
 *          2. Constant frequency: Brief hold at starting frequency
 *          3. Chirp sweep: Linearly sweep from start to stop frequency
 *          4. Fade-out: Gradually decrease amplitude to 0
 * 
 *          The chirp parameters define:
 *          - Which axis to excite (roll, pitch, or yaw)
 *          - Frequency range to sweep (determines bandwidth analyzed)
 *          - Excitation amplitude (must be large enough for measurement, small enough for safety)
 *          - Timing for smooth transitions to avoid transients
 * 
 * @note Logged once at the start of each system ID axis test
 * @see log_SysIdD for the corresponding measurement data
 */
struct PACKED log_SysIdS {
    LOG_PACKET_HEADER;
    uint64_t time_us;               ///< Timestamp in microseconds since system boot
    uint8_t  systemID_axis;         ///< Axis being excited (0=roll, 1=pitch, 2=yaw)
    float    waveform_magnitude;    ///< Maximum chirp amplitude (degrees or deg/s)
    float    frequency_start;       ///< Starting frequency of chirp sweep (Hz)
    float    frequency_stop;        ///< Ending frequency of chirp sweep (Hz)
    float    time_fade_in;          ///< Duration of amplitude fade-in (seconds)
    float    time_const_freq;       ///< Duration at constant start frequency (seconds)
    float    time_record;           ///< Total duration of chirp sweep (seconds)
    float    time_fade_out;         ///< Duration of amplitude fade-out (seconds)
};

/**
 * @brief Write system identification test setup parameters (SIDS)
 * 
 * @details Logs the configuration of a system ID chirp test before execution begins.
 *          This provides the context needed to interpret the subsequent SIDD measurement
 *          data in post-flight analysis.
 * 
 *          Chirp design considerations:
 *          - Frequency range should span expected system dynamics (typically 0.1-20 Hz)
 *          - Lower frequencies test slow position control loops
 *          - Higher frequencies test fast rate control loops and structural modes
 *          - Magnitude must be large enough for good signal-to-noise ratio
 *          - Fade times prevent abrupt transients that corrupt measurements
 * 
 * @param[in] systemID_axis Axis to excite (0=roll, 1=pitch, 2=yaw)
 * @param[in] waveform_magnitude Maximum excitation amplitude (degrees or deg/s)
 * @param[in] frequency_start Starting chirp frequency (Hz)
 * @param[in] frequency_stop Ending chirp frequency (Hz)
 * @param[in] time_fade_in Fade-in duration (seconds)
 * @param[in] time_const_freq Constant frequency hold duration (seconds)
 * @param[in] time_record Total chirp sweep duration (seconds)
 * @param[in] time_fade_out Fade-out duration (seconds)
 * 
 * @note Only available when MODE_SYSTEMID_ENABLED is compiled in
 * @note Logged once at start of each axis test in system ID mode
 * @note Must be paired with SIDD data messages for complete analysis
 * 
 * @see Log_Write_SysID_Data() for measurement data collection
 * @see Mode_SystemID for system identification mode implementation
 */
void Copter::Log_Write_SysID_Setup(uint8_t systemID_axis, float waveform_magnitude, float frequency_start, float frequency_stop, float time_fade_in, float time_const_freq, float time_record, float time_fade_out)
{
#if MODE_SYSTEMID_ENABLED
    struct log_SysIdS pkt_sids = {
        LOG_PACKET_HEADER_INIT(LOG_SYSIDS_MSG),
        time_us             : AP_HAL::micros64(),
        systemID_axis       : systemID_axis,
        waveform_magnitude  : waveform_magnitude,
        frequency_start     : frequency_start,
        frequency_stop      : frequency_stop,
        time_fade_in        : time_fade_in,
        time_const_freq     : time_const_freq,
        time_record         : time_record,
        time_fade_out       : time_fade_out
    };
    logger.WriteBlock(&pkt_sids, sizeof(pkt_sids));
#endif
}

/**
 * @struct log_Guided_Position_Target
 * @brief GUIP log message for guided mode position target commands
 * 
 * @details Records position, velocity, and acceleration targets commanded to the vehicle
 *          in Guided mode via MAVLink. Essential for debugging companion computer control,
 *          verifying target tracking, and analyzing autonomous navigation performance.
 * 
 *          Guided mode supports multiple target types:
 *          - Position-only: Target 3D position, vehicle determines velocity
 *          - Position + velocity: Target position with desired velocity vector
 *          - Position + velocity + acceleration: Full motion profile specification
 * 
 *          Coordinate frame interpretation:
 *          - pos_target: Either absolute (lat/lon/alt) or relative to EKF origin
 *          - vel_target: Velocity in NED frame (meters/second)
 *          - accel_target: Acceleration in NED frame (meters/second²)
 *          - terrain flag: If true, Z axis is altitude above terrain, else above EKF origin
 * 
 * @note Logged whenever guided mode receives new position target commands
 * @note Position values converted from centimeters to meters for logging
 * @see ModeGuided for guided mode implementation and command handling
 */
struct PACKED log_Guided_Position_Target {
    LOG_PACKET_HEADER;
    uint64_t time_us;           ///< Timestamp in microseconds since system boot
    uint8_t type;               ///< Guided mode submode type (position, velocity, etc.)
    float pos_target_x;         ///< Target position X (North) in meters or latitude
    float pos_target_y;         ///< Target position Y (East) in meters or longitude
    float pos_target_z;         ///< Target position Z (altitude) in meters
    uint8_t terrain;            ///< If 1, Z is above terrain; if 0, Z is above EKF origin
    float vel_target_x;         ///< Target velocity North (m/s) in NED frame
    float vel_target_y;         ///< Target velocity East (m/s) in NED frame
    float vel_target_z;         ///< Target velocity Down (m/s) in NED frame
    float accel_target_x;       ///< Target acceleration North (m/s²) in NED frame
    float accel_target_y;       ///< Target acceleration East (m/s²) in NED frame
    float accel_target_z;       ///< Target acceleration Down (m/s²) in NED frame
};

/**
 * @struct log_Guided_Attitude_Target
 * @brief GUIA log message for guided mode attitude target commands
 * 
 * @details Records attitude and thrust targets commanded to the vehicle in Guided mode
 *          via MAVLink. Used when external controller (companion computer) directly
 *          commands vehicle attitude instead of position, enabling lower-level control
 *          for acrobatic maneuvers or custom control algorithms.
 * 
 *          Guided attitude mode supports:
 *          - Attitude-only: Target roll/pitch/yaw angles, vehicle stabilizes rates
 *          - Attitude + rates: Target angles with desired angular velocities
 *          - Direct thrust: Manual control of collective thrust (0.0-1.0)
 *          - Climb rate: Vertical velocity target (meters/second)
 * 
 * @note Logged whenever guided mode receives new attitude target commands
 * @note Angles converted from radians to degrees for logging
 * @note Rates converted from rad/s to deg/s for logging
 * @see ModeGuided for guided mode attitude control implementation
 */
struct PACKED log_Guided_Attitude_Target {
    LOG_PACKET_HEADER;
    uint64_t time_us;       ///< Timestamp in microseconds since system boot
    uint8_t type;           ///< Guided mode submode type (attitude, angle rate, etc.)
    float roll;             ///< Target roll angle in degrees (positive = right)
    float pitch;            ///< Target pitch angle in degrees (positive = forward)
    float yaw;              ///< Target yaw angle in degrees (0-360, North = 0)
    float roll_rate;        ///< Target roll rate in deg/s (body frame)
    float pitch_rate;       ///< Target pitch rate in deg/s (body frame)
    float yaw_rate;         ///< Target yaw rate in deg/s (body frame)
    float thrust;           ///< Target thrust (0.0-1.0, normalized)
    float climb_rate;       ///< Target climb rate in m/s (positive = up)
};

/**
 * @struct log_Rate_Thread_Dt
 * @brief RTDT log message for attitude controller loop timing statistics
 * 
 * @details Records timing performance of the fast attitude controller loop, which
 *          typically runs at 400 Hz. Monitoring loop timing is critical for:
 *          - Detecting CPU overload or scheduler issues
 *          - Verifying consistent controller execution rate
 *          - Diagnosing timing jitter that affects control quality
 * 
 *          Timing statistics tracked:
 *          - dt: Current time delta (seconds) between controller updates
 *          - dtAvg: Average time delta over recent samples
 *          - dtMax: Maximum time delta since last log output
 *          - dtMin: Minimum time delta since last log output
 * 
 *          Expected values (for 400 Hz loop):
 *          - Nominal dt: 0.0025 seconds (2.5 milliseconds)
 *          - dtMax should stay below 0.003 seconds (indicates missed deadline)
 *          - Large variations indicate CPU saturation or interrupt issues
 * 
 * @note Only logged when AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED
 * @warning Large dtMax values indicate control loop timing violations
 * @warning Timing jitter degrades control performance
 */
struct PACKED log_Rate_Thread_Dt {
    LOG_PACKET_HEADER;
    uint64_t time_us;   ///< Timestamp in microseconds since system boot
    float dt;           ///< Current loop time delta in seconds
    float dtAvg;        ///< Average loop time delta in seconds
    float dtMax;        ///< Maximum loop time delta since last output (seconds)
    float dtMin;        ///< Minimum loop time delta since last output (seconds)
};

/**
 * @brief Write guided mode position target log message (GUIP)
 * 
 * @details Logs position, velocity, and acceleration targets commanded via MAVLink in
 *          Guided mode. Essential for verifying that companion computer commands are
 *          being received correctly and for post-flight analysis of autonomous behavior.
 * 
 *          Input units vs logged units:
 *          - pos_target_cm: Input in centimeters, logged as meters
 *          - vel_target_cms: Input in cm/s, logged as m/s
 *          - accel_target_cmss: Input in cm/s², logged as m/s²
 * 
 *          Position interpretation depends on guided submode:
 *          - Absolute: pos_target is latitude/longitude/altitude
 *          - Relative: pos_target is offset from EKF origin in NED frame
 *          - The terrain_alt flag determines Z-axis reference (terrain vs EKF origin)
 * 
 *          Common analysis scenarios:
 *          - Compare GUIP targets vs actual position (from POS message)
 *          - Verify companion computer is sending reasonable commands
 *          - Analyze trajectory tracking performance
 *          - Debug waypoint following or obstacle avoidance algorithms
 * 
 * @param[in] target_type Guided submode type (position, velocity, accel, etc.)
 * @param[in] pos_target_cm Target position in centimeters (NED frame or lat/lon/alt)
 * @param[in] terrain_alt If true, Z is altitude above terrain; if false, above EKF origin
 * @param[in] vel_target_cms Target velocity in cm/s (NED frame)
 * @param[in] accel_target_cmss Target acceleration in cm/s² (NED frame)
 * 
 * @note Units automatically converted from cm to meters for logging
 * @note Logged whenever new position target received in guided mode
 * 
 * @see ModeGuided::set_destination_posvel() for command handling
 * @see log_Guided_Position_Target for message field descriptions
 */
void Copter::Log_Write_Guided_Position_Target(ModeGuided::SubMode target_type, const Vector3f& pos_target_cm, bool terrain_alt, const Vector3f& vel_target_cms, const Vector3f& accel_target_cmss)
{
    const log_Guided_Position_Target pkt {
        LOG_PACKET_HEADER_INIT(LOG_GUIDED_POSITION_TARGET_MSG),
        time_us         : AP_HAL::micros64(),
        type            : (uint8_t)target_type,
        pos_target_x    : pos_target_cm.x * 0.01f,
        pos_target_y    : pos_target_cm.y * 0.01f,
        pos_target_z    : pos_target_cm.z * 0.01f,
        terrain         : terrain_alt,
        vel_target_x    : vel_target_cms.x * 0.01f,
        vel_target_y    : vel_target_cms.y * 0.01f,
        vel_target_z    : vel_target_cms.z * 0.01f,
        accel_target_x  : accel_target_cmss.x * 0.01f,
        accel_target_y  : accel_target_cmss.y * 0.01f,
        accel_target_z  : accel_target_cmss.z * 0.01f
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

/**
 * @brief Write guided mode attitude target log message (GUIA)
 * 
 * @details Logs attitude, angular rate, thrust, and climb rate targets commanded via
 *          MAVLink in Guided mode. Used when companion computer directly controls vehicle
 *          attitude for precision maneuvers or custom control algorithms.
 * 
 *          Input units vs logged units:
 *          - roll, pitch, yaw: Input in radians, logged as degrees
 *          - ang_vel: Input in rad/s, logged as deg/s
 *          - thrust: Input and logged as 0.0-1.0 normalized value
 *          - climb_rate: Input and logged as m/s
 * 
 *          Attitude control modes:
 *          - Angle-only: Attitude controller stabilizes to target angles
 *          - Angle + rate: Combined angle and rate targets for faster response
 *          - Rate-only: Direct angular velocity control (advanced users)
 * 
 *          Common use cases:
 *          - Precision cinematography (smooth camera movements)
 *          - Acrobatic flight (flips, rolls with companion computer)
 *          - Custom control algorithms (research, specialized applications)
 *          - Failsafe recovery (external safety monitor taking control)
 * 
 * @param[in] target_type Guided submode (angle, angle_rate, etc.)
 * @param[in] roll Target roll angle in radians (positive = right)
 * @param[in] pitch Target pitch angle in radians (positive = nose up)
 * @param[in] yaw Target yaw angle in radians (0-2π)
 * @param[in] ang_vel Target angular velocity [roll, pitch, yaw] in rad/s (body frame)
 * @param[in] thrust Target thrust 0.0-1.0 (normalized, 0=minimum, 1=maximum)
 * @param[in] climb_rate Target vertical velocity in m/s (positive = up)
 * 
 * @note Units automatically converted from radians to degrees for logging
 * @note Logged whenever new attitude target received in guided mode
 * 
 * @see ModeGuided::set_angle() for attitude command handling
 * @see log_Guided_Attitude_Target for message field descriptions
 */
void Copter::Log_Write_Guided_Attitude_Target(ModeGuided::SubMode target_type, float roll, float pitch, float yaw, const Vector3f &ang_vel, float thrust, float climb_rate)
{
    const log_Guided_Attitude_Target pkt {
        LOG_PACKET_HEADER_INIT(LOG_GUIDED_ATTITUDE_TARGET_MSG),
        time_us         : AP_HAL::micros64(),
        type            : (uint8_t)target_type,
        roll            : degrees(roll),       // rad to deg
        pitch           : degrees(pitch),      // rad to deg
        yaw             : degrees(yaw),        // rad to deg
        roll_rate       : degrees(ang_vel.x),  // rad/s to deg/s
        pitch_rate      : degrees(ang_vel.y),  // rad/s to deg/s
        yaw_rate        : degrees(ang_vel.z),  // rad/s to deg/s
        thrust          : thrust,
        climb_rate      : climb_rate
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

/**
 * @brief Write rate controller loop timing statistics log message (RTDT)
 * 
 * @details Logs timing performance metrics for the fast rate controller loop, enabling
 *          detection of CPU overload, scheduler issues, or timing jitter that could
 *          degrade flight control performance.
 * 
 *          The rate controller loop timing is critical because:
 *          - Attitude rate control runs at highest frequency (typically 400 Hz)
 *          - Consistent timing ensures predictable PID controller behavior
 *          - Timing jitter introduces phase lag and can cause oscillations
 *          - Missed deadlines indicate CPU saturation or interrupt storms
 * 
 *          Timing analysis workflow:
 *          1. Check dtAvg is close to expected (0.0025s for 400Hz)
 *          2. Verify dtMax stays below 1.5x nominal (max 0.00375s for 400Hz)
 *          3. Check dtMin is not too small (indicates scheduler anomaly)
 *          4. Look for dtMax spikes correlated with flight events
 * 
 *          Common causes of timing violations:
 *          - Too many enabled features (logging, sensors, peripherals)
 *          - Slow SD card causing logger delays
 *          - Excessive I2C/SPI device polling
 *          - Interrupt storms from malfunctioning peripherals
 *          - Insufficient CPU for configured sample rate
 * 
 * @param[in] dt Current loop time delta in seconds
 * @param[in] dtAvg Average loop time delta in seconds (recent window)
 * @param[in] dtMax Maximum loop time delta since last log output (seconds)
 * @param[in] dtMin Minimum loop time delta since last log output (seconds)
 * 
 * @note Only available when AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED
 * @note Logged periodically during flight (rate depends on settings)
 * 
 * @warning dtMax significantly above nominal indicates timing problems
 * @warning Sustained timing violations degrade control performance
 * 
 * @see AP_InertialSensor for fast sampling configuration
 * @see log_Rate_Thread_Dt for message structure
 */
void Copter::Log_Write_Rate_Thread_Dt(float dt, float dtAvg, float dtMax, float dtMin)
{
#if AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED
    const log_Rate_Thread_Dt pkt {
        LOG_PACKET_HEADER_INIT(LOG_RATE_THREAD_DT_MSG),
        time_us         : AP_HAL::micros64(),
        dt              : dt,
        dtAvg           : dtAvg,
        dtMax           : dtMax,
        dtMin           : dtMin
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
#endif
}

/**
 * @brief Copter-specific log message format definitions for AP_Logger
 * 
 * @details This array defines the binary format, field names, units, and multipliers
 *          for all copter-specific log messages. The AP_Logger system uses these
 *          definitions to:
 *          - Encode data efficiently in binary format
 *          - Generate message format (FMT) headers in log files
 *          - Enable ground station tools to decode and display log data
 *          - Support log download and analysis with correct units
 * 
 *          LogStructure format specification:
 *          - Message ID: Unique identifier (e.g., LOG_CONTROL_TUNING_MSG)
 *          - Size: sizeof(structure) for binary encoding
 *          - Name: 4-character message name (e.g., "CTUN")
 *          - Format: Type codes for each field (Q=uint64_t, f=float, h=int16_t, etc.)
 *          - Labels: Comma-separated field names
 *          - Units: Single-character unit codes (s=seconds, m=meters, d=degrees, etc.)
 *          - Multipliers: Scaling indicators (F=full precision, B=base-2, 0=integer)
 * 
 *          Format character reference (from AP_Logger/LogStructure.h):
 *          - Q: uint64_t (8 bytes) - typically timestamps
 *          - q: int64_t (8 bytes)
 *          - I: uint32_t (4 bytes)
 *          - i: int32_t (4 bytes)
 *          - H: uint16_t (2 bytes)
 *          - h: int16_t (2 bytes)
 *          - B: uint8_t (1 byte)
 *          - b: int8_t (1 byte)
 *          - f: float (4 bytes)
 *          - d: double (8 bytes)
 *          - n: char[4] (4 bytes)
 *          - N: char[16] (16 bytes)
 *          - Z: char[64] (64 bytes)
 *          - c: int16_t * 100 (lat/lon in degrees * 1e7)
 *          - C: uint16_t * 100
 *          - e: int32_t * 100
 *          - E: uint32_t * 100
 *          - L: int32_t lat/lon (degrees * 1e7)
 *          - M: uint8_t flight mode
 * 
 *          Unit codes (from AP_Logger log_Units):
 *          - s: seconds
 *          - d: degrees
 *          - k: degrees/second (angular rate)
 *          - m: meters
 *          - n: meters/second (velocity)
 *          - o: meters/second² (acceleration)
 *          - %: percent
 *          - A: amperes
 *          - V: volts
 *          - #: instance number
 *          - -: dimensionless
 * 
 *          The final boolean parameter indicates whether the message should be
 *          streamed over MAVLink during real-time logging.
 * 
 * @note LOG_COMMON_STRUCTURES included from AP_Logger for shared message types
 * @note Message format must exactly match structure definition
 * @note Units and multipliers inform ground station display scaling
 * 
 * @see libraries/AP_Logger/LogStructure.h for complete format reference
 * @see AP_Logger for binary logging implementation
 */
const struct LogStructure Copter::log_structure[] = {
    LOG_COMMON_STRUCTURES,
    
// @LoggerMessage: PTUN
// @Description: Parameter Tuning information
// @URL: https://ardupilot.org/copter/docs/tuning.html#in-flight-tuning
// @Field: TimeUS: Time since system startup
// @Field: Param: Parameter being tuned
// @Field: TunVal: Normalized value used inside tuning() function
// @Field: TunMin: Tuning minimum limit
// @Field: TunMax: Tuning maximum limit

    { LOG_PARAMTUNE_MSG, sizeof(log_ParameterTuning),
      "PTUN", "QBfff",         "TimeUS,Param,TunVal,TunMin,TunMax", "s----", "F----" },

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

    { LOG_CONTROL_TUNING_MSG, sizeof(log_Control_Tuning),
      "CTUN", "Qffffffefffhh", "TimeUS,ThI,ABst,ThO,ThH,DAlt,Alt,BAlt,DSAlt,SAlt,TAlt,DCRt,CRt", "s----mmmmmmnn", "F----00B000BB" , true },
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

// @LoggerMessage: SIDD
// @Description: System ID data
// @Field: TimeUS: Time since system startup
// @Field: Time: Time reference for waveform
// @Field: Targ: Current waveform sample
// @Field: F: Instantaneous waveform frequency
// @Field: Gx: Delta angle, X-Axis
// @Field: Gy: Delta angle, Y-Axis
// @Field: Gz: Delta angle, Z-Axis
// @Field: Ax: Delta velocity, X-Axis
// @Field: Ay: Delta velocity, Y-Axis
// @Field: Az: Delta velocity, Z-Axis

    { LOG_SYSIDD_MSG, sizeof(log_SysIdD),
      "SIDD", "Qfffffffff",  "TimeUS,Time,Targ,F,Gx,Gy,Gz,Ax,Ay,Az", "ss-zkkkooo", "F---------" , true },

// @LoggerMessage: SIDS
// @Description: System ID settings
// @Field: TimeUS: Time since system startup
// @Field: Ax: The axis which is being excited
// @Field: Mag: Magnitude of the chirp waveform
// @Field: FSt: Frequency at the start of chirp
// @Field: FSp: Frequency at the end of chirp
// @Field: TFin: Time to reach maximum amplitude of chirp
// @Field: TC: Time at constant frequency before chirp starts
// @Field: TR: Time taken to complete chirp waveform
// @Field: TFout: Time to reach zero amplitude after chirp finishes

    { LOG_SYSIDS_MSG, sizeof(log_SysIdS),
      "SIDS", "QBfffffff",  "TimeUS,Ax,Mag,FSt,FSp,TFin,TC,TR,TFout", "s--ssssss", "F--------" , true },

// @LoggerMessage: GUIP
// @Description: Guided mode position target information
// @Field: TimeUS: Time since system startup
// @Field: Type: Type of guided mode
// @Field: pX: Target position, X-Axis
// @Field: pY: Target position, Y-Axis
// @Field: pZ: Target position, Z-Axis
// @Field: Terrain: Target position, Z-Axis is alt above terrain
// @Field: vX: Target velocity, X-Axis
// @Field: vY: Target velocity, Y-Axis
// @Field: vZ: Target velocity, Z-Axis
// @Field: aX: Target acceleration, X-Axis
// @Field: aY: Target acceleration, Y-Axis
// @Field: aZ: Target acceleration, Z-Axis

    { LOG_GUIDED_POSITION_TARGET_MSG, sizeof(log_Guided_Position_Target),
      "GUIP",  "QBfffbffffff",    "TimeUS,Type,pX,pY,pZ,Terrain,vX,vY,vZ,aX,aY,aZ", "s-mmm-nnnooo", "F-000-000000" , true },

// @LoggerMessage: GUIA
// @Description: Guided mode attitude target information
// @Field: TimeUS: Time since system startup
// @Field: Type: Type of guided mode
// @Field: Roll: Target attitude, Roll
// @Field: Pitch: Target attitude, Pitch
// @Field: Yaw: Target attitude, Yaw
// @Field: RollRt: Roll rate
// @Field: PitchRt: Pitch rate
// @Field: YawRt: Yaw rate
// @Field: Thrust: Thrust 
// @Field: ClimbRt: Climb rate

    { LOG_GUIDED_ATTITUDE_TARGET_MSG, sizeof(log_Guided_Attitude_Target),
      "GUIA",  "QBffffffff",    "TimeUS,Type,Roll,Pitch,Yaw,RollRt,PitchRt,YawRt,Thrust,ClimbRt", "s-dddkkk-n", "F-000000-0" , true },

// @LoggerMessage: RTDT
// @Description: Attitude controller time deltas
// @Field: TimeUS: Time since system startup
// @Field: dt: current time delta
// @Field: dtAvg: current time delta average
// @Field: dtMax: Max time delta since last log output
// @Field: dtMin: Min time delta since last log output

    { LOG_RATE_THREAD_DT_MSG, sizeof(log_Rate_Thread_Dt),
      "RTDT", "Qffff", "TimeUS,dt,dtAvg,dtMax,dtMin", "sssss", "F----" , true },

};

/**
 * @brief Get the number of copter-specific log message structures
 * 
 * @details Returns the count of log message format definitions in the log_structure
 *          array. This is used by AP_Logger to iterate through and register all
 *          copter-specific message types during initialization.
 * 
 *          The count includes:
 *          - All copter-specific message types (CTUN, PTUN, SIDD, SIDS, GUIP, GUIA, RTDT)
 *          - Generic data messages (D16, DU16, D32, DU32, DFLT)
 *          - Common structures inherited via LOG_COMMON_STRUCTURES
 * 
 * @return Number of LogStructure entries in log_structure array
 * 
 * @note Used by AP_Logger during initialization to register message formats
 * @note ARRAY_SIZE macro provides compile-time array element count
 * 
 * @see log_structure for the complete message format array
 * @see AP_Logger::Init() for logger initialization sequence
 */
uint8_t Copter::get_num_log_structures() const
{
    return ARRAY_SIZE(log_structure);
}

/**
 * @brief Write vehicle configuration and initial state messages at startup
 * 
 * @details Logs critical vehicle identification and configuration information at the
 *          beginning of each log file. This startup banner provides essential context
 *          for log analysis, including vehicle type, initial mode, home position, and
 *          GPS status.
 * 
 *          Startup messages logged:
 *          1. Frame and motor configuration: Motor matrix type (quad, hexa, octo, Y6, etc.)
 *             and frame class (identifies vehicle geometry for proper log interpretation)
 *          
 *          2. Initial flight mode: Starting mode (typically STABILIZE) and the reason
 *             for the mode (system startup, pilot selection, etc.)
 *          
 *          3. Home and origin: EKF origin position and home location coordinates,
 *             critical for interpreting relative position data in logs
 *          
 *          4. GPS startup info: GPS receiver configuration, satellite constellation
 *             data, and initial fix quality
 * 
 *          These messages are essential for:
 *          - Identifying which vehicle generated the log (frame type)
 *          - Understanding coordinate frame references (EKF origin, home location)
 *          - Verifying GPS health at boot (satellite count, fix type)
 *          - Correlating log data with known vehicle configurations
 *          - Debugging multi-vehicle operations (identifying specific airframe)
 * 
 *          Log analysis tools use this information to:
 *          - Select appropriate motor mixing visualization
 *          - Transform position data into correct coordinate frames
 *          - Display vehicle-specific parameters and limits
 *          - Validate log file integrity and completeness
 * 
 * @note Called once at startup after sensors initialized but before arming
 * @note Limited to ~200 bytes total by AP_Logger buffer constraints
 * @note Frame string format: "Frame: <CLASS>/<TYPE>" (e.g., "Frame: QUAD/X")
 * @note Subsequent mode changes logged separately via logger.Write_Mode()
 * 
 * @warning GPS home position may not be set if GPS lacks fix at boot
 * @warning EKF origin may differ from home if GPS unavailable at startup
 * 
 * @see AP_Motors::get_frame_and_type_string() for frame identification
 * @see AP_AHRS::Log_Write_Home_And_Origin() for coordinate frame setup
 * @see AP_GPS::Write_AP_Logger_Log_Startup_messages() for GPS configuration
 */
void Copter::Log_Write_Vehicle_Startup_Messages()
{
    // only 200(?) bytes are guaranteed by AP_Logger
    char frame_and_type_string[30];
    copter.motors->get_frame_and_type_string(frame_and_type_string, ARRAY_SIZE(frame_and_type_string));
    logger.Write_MessageF("%s", frame_and_type_string);
    logger.Write_Mode((uint8_t)flightmode->mode_number(), control_mode_reason);
    ahrs.Log_Write_Home_And_Origin();
    gps.Write_AP_Logger_Log_Startup_messages();
}

#endif // HAL_LOGGING_ENABLED
