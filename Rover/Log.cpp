/**
 * @file Log.cpp
 * @brief Implementation of rover-specific binary logging functionality
 * 
 * @details This file implements rover-specific logging functions that write
 *          vehicle state data to the binary log system. It includes specialized
 *          log structures and write methods for rover operations including:
 *          - Throttle control data (THR)
 *          - Steering control data (STER)
 *          - Navigation tuning information (NTUN)
 *          - Guided mode targets (GUIP)
 *          - Depth information for boats (DPTH)
 *          - Sailboat-specific data (SAIL)
 *          
 *          All logging functionality is conditionally compiled based on
 *          HAL_LOGGING_ENABLED to support configurations without logging.
 *          
 *          Log message structures and formatting information are defined at
 *          the bottom of this file in the log_structure array, following
 *          ArduPilot's logger message documentation format.
 * 
 * @note This file only implements logging - it does not modify vehicle behavior
 * @see libraries/AP_Logger/AP_Logger.h for the core logging infrastructure
 * @see libraries/AP_Logger/LogStructure.h for log format documentation
 * 
 * Source: Rover/Log.cpp
 */

#include "Rover.h"

#include <AP_RangeFinder/AP_RangeFinder_Backend.h>

/**
 * @note All logging functionality in this file is conditionally compiled.
 *       When HAL_LOGGING_ENABLED is false, none of these functions are included
 *       in the binary, saving flash space on memory-constrained boards.
 */
#if HAL_LOGGING_ENABLED

/**
 * @brief Write rover attitude and control data to the log
 * 
 * @details Logs the vehicle's attitude information and PID controller states.
 *          For rovers, this primarily includes:
 *          - Desired pitch angle (relevant for balance bots)
 *          - Steering rate controller PID state (LOG_PIDS)
 *          - Throttle/speed controller PID state (LOG_PIDA)
 *          - Pitch-to-throttle PID state for balance bots (LOG_PIDP)
 *          - Heel control PID state for sailboats (LOG_PIDR)
 *          
 *          This method is called at the main loop rate to provide high-resolution
 *          controller state logging for tuning and analysis.
 * 
 * @note Vehicle-specific: Balance bots log pitch control, sailboats log heel control
 * @note Called at main loop rate (typically 50Hz for rovers)
 * 
 * @see Log_Write_Steering() for steering output logging
 * @see Log_Write_Throttle() for throttle output logging
 */
void Rover::Log_Write_Attitude()
{
    float desired_pitch = degrees(g2.attitude_control.get_desired_pitch());
    const Vector3f targets(0.0f, desired_pitch, 0.0f);

    ahrs.Write_Attitude(targets);

    AP::ahrs().Log_Write();

    // log steering rate controller
    logger.Write_PID(LOG_PIDS_MSG, g2.attitude_control.get_steering_rate_pid().get_pid_info());
    logger.Write_PID(LOG_PIDA_MSG, g2.attitude_control.get_throttle_speed_pid_info());

    // log pitch control for balance bots
    if (is_balancebot()) {
        logger.Write_PID(LOG_PIDP_MSG, g2.attitude_control.get_pitch_to_throttle_pid().get_pid_info());
    }

    // log heel to sail control for sailboats
    if (g2.sailboat.sail_enabled()) {
        logger.Write_PID(LOG_PIDR_MSG, g2.attitude_control.get_sailboat_heel_pid().get_pid_info());
    }
}

#if AP_RANGEFINDER_ENABLED
/**
 * @brief Write depth information to log for boats with downward-facing rangefinders
 * 
 * @details Logs underwater depth measurements from downward-facing rangefinders
 *          (ROTATION_PITCH_270) on boat-type rovers. Records depth, temperature,
 *          and GPS position for each active depth sensor. Only logs when:
 *          - Vehicle is configured as a boat
 *          - Downward-facing rangefinder is present
 *          - New sensor reading is available (prevents duplicate log entries)
 *          
 *          Creates DPTH log messages with the following information:
 *          - GPS coordinates (lat/lng in degrees)
 *          - Depth measurement (meters)
 *          - Water temperature (Celsius, if supported by sensor)
 * 
 * @note Only active for boat-type rovers (is_boat() returns true)
 * @note Only logs sensors with ROTATION_PITCH_270 orientation
 * @note Temperature field set to 0.0 if sensor doesn't support temperature
 * @note GPS coordinates are from vehicle position (not sensor-specific)
 * 
 * @warning Depth values are sensor distances - not compensated for sensor mounting position
 * 
 * @see Log message format: DPTH in log_structure array
 * Source: Rover/Log.cpp:34-82
 */
void Rover::Log_Write_Depth()
{
    // only log depth on boats
    if (!rover.is_boat() || !rangefinder.has_orientation(ROTATION_PITCH_270)) {
        return;
    }

    // get position
    Location loc;
    IGNORE_RETURN(ahrs.get_location(loc));

    for (uint8_t i=0; i<rangefinder.num_sensors(); i++) {
        const AP_RangeFinder_Backend *s = rangefinder.get_backend(i);
        
        if (s == nullptr || s->orientation() != ROTATION_PITCH_270 || !s->has_data()) {
            continue;
        }

        // check if new sensor reading has arrived
        const uint32_t reading_ms = s->last_reading_ms();
        if (reading_ms == rangefinder_last_reading_ms[i]) {
            continue;
        }
        rangefinder_last_reading_ms[i] = reading_ms;

        float temp_C;
        if (!s->get_temp(temp_C)) {
            temp_C = 0.0f;
        }

        // @LoggerMessage: DPTH
        // @Description: Depth messages on boats with downwards facing range finder
        // @Field: TimeUS: Time since system startup
        // @Field: Inst: Instance
        // @Field: Lat: Latitude 
        // @Field: Lng: Longitude   
        // @Field: Depth: Depth as detected by the sensor
        // @Field: Temp: Temperature

        logger.Write("DPTH", "TimeUS,Inst,Lat,Lng,Depth,Temp",
                            "s#DUmO", "F-GG00", "QBLLff",
                            AP_HAL::micros64(),
                            i,
                            loc.lat,
                            loc.lng,
                            (double)(s->distance()),
                            temp_C);
    }
}
#endif

/**
 * @brief Log structure for guided mode target information (GUIP message)
 * 
 * @details Stores the current target position and velocity when operating in
 *          guided mode. This allows post-flight analysis of commanded targets
 *          versus actual vehicle performance.
 *          
 *          Coordinate system: NED (North-East-Down) frame relative to home position
 *          - X axis: North (meters)
 *          - Y axis: East (meters)
 *          - Z axis: Down (meters)
 *          
 *          Velocity targets in NED frame:
 *          - X velocity: North component (m/s)
 *          - Y velocity: East component (m/s)
 *          - Z velocity: Down component (m/s, typically 0 for ground rovers)
 */
struct PACKED log_GuidedTarget {
    LOG_PACKET_HEADER;          ///< Standard log packet header with message type
    uint64_t time_us;            ///< Timestamp in microseconds since system boot
    uint8_t type;                ///< Guided mode type (position, velocity, angle, etc.)
    float pos_target_x;          ///< Target position X in NED frame (meters north from home)
    float pos_target_y;          ///< Target position Y in NED frame (meters east from home)
    float pos_target_z;          ///< Target position Z in NED frame (meters down from home, typically 0 for rovers)
    float vel_target_x;          ///< Target velocity X in NED frame (m/s north)
    float vel_target_y;          ///< Target velocity Y in NED frame (m/s east)
    float vel_target_z;          ///< Target velocity Z in NED frame (m/s down, typically 0 for rovers)
};

/**
 * @brief Write guided mode target information to the log
 * 
 * @details Records the current target position and velocity commanded in guided mode.
 *          This is essential for analyzing guided mode performance and debugging
 *          issues with external guidance systems (companion computers, GCS, etc.).
 *          
 *          The target data is logged in the NED (North-East-Down) coordinate frame
 *          relative to the home position, allowing easy visualization and analysis
 *          of commanded paths.
 * 
 * @param[in] target_type Type of guided target (position, velocity, angle, etc.)
 * @param[in] pos_target Target position vector in NED frame (meters)
 *                       - x: meters north from home
 *                       - y: meters east from home  
 *                       - z: meters down from home (typically 0 for ground rovers)
 * @param[in] vel_target Target velocity vector in NED frame (m/s)
 *                       - x: m/s north
 *                       - y: m/s east
 *                       - z: m/s down (typically 0 for ground rovers)
 * 
 * @note Coordinate frame: NED (North-East-Down) relative to home position
 * @note Called when guided mode receives new target commands
 * @note Z-axis values typically 0 for ground rovers (no altitude control)
 * 
 * @see log_GuidedTarget structure definition
 * @see Mode::guided for guided mode implementation
 * Source: Rover/Log.cpp:99-113
 */
void Rover::Log_Write_GuidedTarget(uint8_t target_type, const Vector3f& pos_target, const Vector3f& vel_target)
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

/**
 * @brief Log structure for navigation tuning data (NTUN message)
 * 
 * @details Stores navigation controller state for tuning and performance analysis.
 *          This data is essential for diagnosing navigation issues and tuning
 *          path following parameters. All angular values use ArduPilot's standard
 *          conventions.
 *          
 *          Angular conventions:
 *          - Bearing: Degrees from north (0-360°, clockwise)
 *          - Yaw: Centidegrees from north (0-35999, clockwise)
 *          
 *          Distance conventions:
 *          - Distances in meters
 *          - Cross-track error: positive = right of path, negative = left of path
 */
struct PACKED log_Nav_Tuning {
    LOG_PACKET_HEADER;          ///< Standard log packet header with message type
    uint64_t time_us;            ///< Timestamp in microseconds since system boot
    float wp_distance;           ///< Distance to current waypoint/destination (meters)
    float wp_bearing;            ///< Bearing to current waypoint from vehicle (degrees, 0-360° from north)
    float nav_bearing;           ///< Desired heading to follow path (degrees, 0-360° from north)
    uint16_t yaw;                ///< Current vehicle heading (centidegrees, 0-35999 from north)
    float xtrack_error;          ///< Cross-track error from desired path (meters, positive = right of path)
};

/**
 * @brief Write navigation tuning data to the log
 * 
 * @details Records the navigation controller's current state including distance
 *          to destination, bearing errors, and cross-track error. This data is
 *          critical for:
 *          - Tuning navigation parameters (L1 period, max lateral acceleration)
 *          - Analyzing path following performance
 *          - Diagnosing navigation failures
 *          - Evaluating different navigation algorithms
 *          
 *          The navigation data varies by control mode:
 *          - Auto/Guided: Distance and bearing to next waypoint
 *          - RTL: Distance and bearing to home
 *          - Steering/Acro: May return 0 or NaN for navigation fields
 * 
 * @note Called at main loop rate (typically 50Hz) during navigation modes
 * @note Distance and bearing data depends on current control mode
 * @note Yaw is in centidegrees (0-35999) for precision; other angles in degrees
 * @note Cross-track error: positive = vehicle is right of desired path
 * 
 * @warning Navigation bearing and waypoint bearing may differ - nav bearing
 *          is the desired heading to stay on path, wp bearing is direct to target
 * 
 * @see log_Nav_Tuning structure definition
 * @see Mode classes for mode-specific navigation implementations
 * Source: Rover/Log.cpp:126-138
 */
void Rover::Log_Write_Nav_Tuning()
{
    struct log_Nav_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NTUN_MSG),
        time_us             : AP_HAL::micros64(),
        wp_distance         : control_mode->get_distance_to_destination(),
        wp_bearing          : control_mode->wp_bearing(),
        nav_bearing         : control_mode->nav_bearing(),
        yaw                 : (uint16_t)ahrs.yaw_sensor,
        xtrack_error        : control_mode->crosstrack_error()
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

/**
 * @brief Write sailboat-specific data to the log
 * 
 * @details Records sailboat sail control and performance data including:
 *          - Current tack (port or starboard)
 *          - Tack threshold angle (apparent wind angle for tack decisions)
 *          - Mainsail output (normalized 0.0-1.0)
 *          - Wingsail output (normalized 0.0-1.0)
 *          - Mast rotation output (normalized 0.0-1.0)
 *          - Velocity Made Good (VMG) - speed toward destination
 *          
 *          VMG is the component of boat speed directly toward the destination,
 *          accounting for tacking angles and wind conditions.
 * 
 * @note Only logs if sailboat features are enabled
 * @note Tack: 0 = port tack (wind from port side), 1 = starboard tack
 * @note Output values normalized: 0.0 = minimum, 1.0 = maximum
 * @note VMG: m/s, can be negative if moving away from destination
 * 
 * @see Sailboat class for sail control implementation
 * @see WindVane class for wind sensing and tack determination
 * Source: Rover/Log.cpp:140-173
 */
void Rover::Log_Write_Sail()
{
    // only log sail if present
    if (!g2.sailboat.sail_enabled()) {
        return;
    }

    float wind_dir_tack = logger.quiet_nanf();
    uint8_t current_tack = 0;
    if (g2.windvane.enabled()) {
        wind_dir_tack = degrees(g2.windvane.get_tack_threshold_wind_dir_rad());
        current_tack = uint8_t(g2.windvane.get_current_tack());
    }

// @LoggerMessage: SAIL
// @Description: Sailboat information
// @Field: TimeUS: Time since system startup
// @Field: Tack: Current tack, 0 = port, 1 = starboard
// @Field: TackThr: Apparent wind angle used for tack threshold
// @Field: MainOut: Normalized mainsail output
// @Field: WingOut: Normalized wingsail output
// @Field: MastRotOut: Normalized direct-rotation mast output
// @Field: VMG: Velocity made good (speed at which vehicle is making progress directly towards destination)

    logger.Write("SAIL", "TimeUS,Tack,TackThr,MainOut,WingOut,MastRotOut,VMG",
                        "s-d%%%n", "F000000", "QBfffff",
                        AP_HAL::micros64(),
                        current_tack,
                        (double)wind_dir_tack,
                        (double)g2.motors.get_mainsail(),
                        (double)g2.motors.get_wingsail(),
                        (double)g2.motors.get_mast_rotation(),
                        (double)g2.sailboat.get_VMG());
}

/**
 * @brief Log structure for steering control data (STER message)
 * 
 * @details Stores steering input, output, and control loop state for analysis
 *          of steering performance and controller tuning. Records both the
 *          commanded values and achieved values for lateral acceleration and
 *          turn rate.
 *          
 *          Control flow: pilot input → desired lat accel → desired turn rate → steering output
 *          
 *          Units:
 *          - Steering in/out: -4500 to +4500 (internal units, maps to full deflection)
 *          - Lateral acceleration: m/s²
 *          - Turn rate: degrees/second (positive = turning right)
 */
struct PACKED log_Steering {
    LOG_PACKET_HEADER;          ///< Standard log packet header with message type
    uint64_t time_us;            ///< Timestamp in microseconds since system boot
    int16_t steering_in;         ///< Pilot steering input in internal units (-4500 to +4500)
    float steering_out;          ///< Normalized steering output to motors (-1.0 to +1.0)
    float desired_lat_accel;     ///< Desired lateral acceleration from mode (m/s²)
    float lat_accel;             ///< Actual lateral acceleration from IMU (m/s²)
    float desired_turn_rate;     ///< Desired turn rate from controller (degrees/second)
    float turn_rate;             ///< Actual turn rate from AHRS (degrees/second, earth frame)
};

/**
 * @brief Write steering control data to the log
 * 
 * @details Records complete steering control loop state including pilot input,
 *          controller commands, and achieved results. Essential for:
 *          - Tuning steering rate PID controller
 *          - Diagnosing steering responsiveness issues
 *          - Analyzing difference between commanded and achieved turn rates
 *          - Evaluating lateral acceleration control performance
 *          
 *          The steering control chain:
 *          1. Pilot input or mode generates desired lateral acceleration
 *          2. Lateral acceleration converted to desired turn rate
 *          3. Turn rate controller generates steering output
 *          4. Steering output sent to motors/servos
 * 
 * @note Called at main loop rate (typically 50Hz)
 * @note steering_in range: -4500 to +4500 (full left to full right)
 * @note steering_out range: -1.0 to +1.0 (normalized motor output)
 * @note Turn rate in degrees/second: positive = turning right (clockwise from above)
 * @note Lateral acceleration in m/s²: positive = acceleration to the right
 * 
 * @warning Actual lateral acceleration may be NaN if IMU doesn't provide it
 * 
 * @see log_Steering structure definition
 * @see AR_AttitudeControl for steering controller implementation
 * Source: Rover/Log.cpp:187-202
 */
void Rover::Log_Write_Steering()
{
    float lat_accel = logger.quiet_nanf();
    g2.attitude_control.get_lat_accel(lat_accel);
    struct log_Steering pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STEERING_MSG),
        time_us        : AP_HAL::micros64(),
        steering_in        : channel_steer->get_control_in(),
        steering_out       : g2.motors.get_steering(),
        desired_lat_accel  : control_mode->get_desired_lat_accel(),
        lat_accel          : lat_accel,
        desired_turn_rate  : degrees(g2.attitude_control.get_desired_turn_rate()),
        turn_rate          : degrees(ahrs.get_yaw_rate_earth())
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

/**
 * @brief Log structure for throttle/speed control data (THR message)
 * 
 * @details Stores throttle input, output, and speed control state for tuning
 *          and performance analysis. Records both commanded and actual speeds,
 *          allowing evaluation of speed controller performance.
 *          
 *          Control flow: pilot input or mode → desired speed → speed controller → throttle output
 *          
 *          Units:
 *          - Throttle in: -4500 to +4500 (internal units, maps to full reverse/forward)
 *          - Throttle out: -1.0 to +1.0 (normalized motor output)
 *          - Speed: m/s (positive = forward, negative = reverse)
 *          - Acceleration: m/s² (body frame X-axis, positive = forward)
 */
struct PACKED log_Throttle {
    LOG_PACKET_HEADER;          ///< Standard log packet header with message type
    uint64_t time_us;            ///< Timestamp in microseconds since system boot
    int16_t throttle_in;         ///< Pilot throttle input in internal units (-4500 to +4500)
    float throttle_out;          ///< Normalized throttle output to motors (-1.0 to +1.0)
    float desired_speed;         ///< Desired forward speed from mode (m/s)
    float speed;                 ///< Actual forward speed from navigation system (m/s)
    float accel_x;               ///< Forward acceleration from IMU in body frame (m/s²)
};

/**
 * @brief Write throttle/speed control data to the log
 * 
 * @details Records complete throttle and speed control state including pilot input,
 *          speed controller commands, and achieved results. Essential for:
 *          - Tuning speed PID controller (throttle to speed loop)
 *          - Diagnosing acceleration performance issues
 *          - Analyzing difference between commanded and achieved speeds
 *          - Evaluating motor/ESC response characteristics
 *          
 *          The speed control chain:
 *          1. Pilot input or mode generates desired speed
 *          2. Speed controller compares desired vs actual speed
 *          3. Speed PID controller generates throttle output
 *          4. Throttle output sent to motors/ESCs
 * 
 * @note Called at main loop rate (typically 50Hz)
 * @note throttle_in range: -4500 to +4500 (full reverse to full forward)
 * @note throttle_out range: -1.0 to +1.0 (normalized motor output)
 * @note Speed in m/s: positive = forward motion, negative = reverse motion
 * @note accel_x in m/s²: positive = forward acceleration (body frame)
 * 
 * @warning Actual speed may be NaN if speed estimation unavailable
 * @warning accel_x is in body frame (vehicle's forward direction), not earth frame
 * 
 * @see log_Throttle structure definition
 * @see AR_AttitudeControl for speed controller implementation
 * Source: Rover/Log.cpp:215-230
 */
void Rover::Log_Write_Throttle()
{
    const Vector3f accel = ins.get_accel();
    float speed = logger.quiet_nanf();
    g2.attitude_control.get_forward_speed(speed);
    struct log_Throttle pkt = {
        LOG_PACKET_HEADER_INIT(LOG_THR_MSG),
        time_us         : AP_HAL::micros64(),
        throttle_in     : channel_throttle->get_control_in(),
        throttle_out    : g2.motors.get_throttle(),
        desired_speed   : g2.attitude_control.get_desired_speed(),
        speed           : speed,
        accel_x         : accel.x
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

/**
 * @brief Write RC (Radio Control) input and output data to the log
 * 
 * @details Records all RC channel inputs from the receiver and all RC channel
 *          outputs to servos/ESCs. Also logs RSSI (Received Signal Strength
 *          Indicator) if available. This data is essential for:
 *          - Diagnosing RC link issues
 *          - Analyzing pilot inputs during flights
 *          - Verifying servo outputs match commands
 *          - Monitoring RC link quality via RSSI
 * 
 * @note RCIN: Raw PWM values from RC receiver (typically 1000-2000 microseconds)
 * @note RCOUT: PWM values output to servos/ESCs (typically 1000-2000 microseconds)
 * @note RSSI: Only logged if RSSI monitoring is enabled and supported
 * @note Called at main loop rate
 * 
 * @see AP_Logger::Write_RCIN() for input channel logging
 * @see AP_Logger::Write_RCOUT() for output channel logging
 * @see AP_RSSI for signal strength monitoring
 */
void Rover::Log_Write_RC(void)
{
    logger.Write_RCIN();
    logger.Write_RCOUT();
#if AP_RSSI_ENABLED
    if (rssi.enabled()) {
        logger.Write_RSSI();
    }
#endif
}

/**
 * @brief Write critical startup information at the beginning of each log
 * 
 * @details Records essential vehicle configuration and state information at
 *          log initialization. This provides context for the entire log session:
 *          - Initial flight mode and reason for that mode
 *          - Home position (takeoff/arming location)
 *          - EKF origin (reference point for local coordinates)
 *          - GPS configuration and status
 *          
 *          This information is critical for properly interpreting all subsequent
 *          log data, as many position/velocity values are relative to these
 *          reference points.
 * 
 * @note Called once at log initialization (typically at boot or arming)
 * @note Limited to ~200 bytes by AP_Logger buffer constraints
 * @note Home position defines the reference for NED coordinates
 * @note EKF origin defines the reference for EKF state estimates
 * 
 * @warning Do not log large amounts of data in this function due to buffer limits
 * 
 * @see AP_Logger for logging infrastructure
 * @see AP_AHRS::Log_Write_Home_And_Origin() for reference point logging
 * Source: Rover/Log.cpp:243-249
 */
void Rover::Log_Write_Vehicle_Startup_Messages()
{
    // only 200(?) bytes are guaranteed by AP_Logger
    logger.Write_Mode((uint8_t)control_mode->mode_number(), control_mode_reason);
    ahrs.Log_Write_Home_And_Origin();
    gps.Write_AP_Logger_Log_Startup_messages();
}

/**
 * @brief Rover-specific log message structure definitions
 * 
 * @details This array defines the format, field names, units, and multipliers
 *          for all rover-specific log messages. Each entry specifies:
 *          - Message ID (e.g., LOG_THR_MSG)
 *          - Structure size for validation
 *          - Message name (4 character code, e.g., "THR")
 *          - Format string (data types: Q=uint64, h=int16, f=float, etc.)
 *          - Field names (comma-separated)
 *          - Unit types (s=seconds, m=meters, n=m/s, o=m/s², etc.)
 *          - Multipliers (F=format, 0=raw value, B=boolean, etc.)
 *          
 *          This information is used by:
 *          - AP_Logger to encode binary log data efficiently
 *          - Log analysis tools to decode and display log data
 *          - Auto-documentation system to generate log message reference
 *          
 *          The @LoggerMessage comments above each entry provide human-readable
 *          documentation that is extracted for documentation generation.
 * 
 * @note Type and unit information reference:
 *       - Format characters: libraries/AP_Logger/LogStructure.h "Format characters"
 *       - Unit codes: libraries/AP_Logger/LogStructure.h "log_Units"
 * @note LOG_COMMON_STRUCTURES includes standard messages shared across all vehicles
 * 
 * @see libraries/AP_Logger/LogStructure.h for format documentation
 * @see libraries/AP_Logger/AP_Logger.h for logging infrastructure
 * Source: Rover/Log.cpp:255-315
 */

// type and unit information can be found in
// libraries/AP_Logger/Logstructure.h; search for "log_Units" for
// units and "Format characters" for field type information

const LogStructure Rover::log_structure[] = {
    LOG_COMMON_STRUCTURES,

// @LoggerMessage: THR
// @Description: Throttle related messages
// @Field: TimeUS: Time since system startup
// @Field: ThrIn: Throttle Input
// @Field: ThrOut: Throttle Output 
// @Field: DesSpeed: Desired speed 
// @Field: Speed: Actual speed
// @Field: AccX: Acceleration

    { LOG_THR_MSG, sizeof(log_Throttle),
      "THR", "Qhffff", "TimeUS,ThrIn,ThrOut,DesSpeed,Speed,AccX", "s--nno", "F--000" },

// @LoggerMessage: NTUN
// @Description: Navigation Tuning information - e.g. vehicle destination
// @URL: http://ardupilot.org/rover/docs/navigation.html
// @Field: TimeUS: Time since system startup
// @Field: WpDist: distance to the current navigation waypoint
// @Field: WpBrg: bearing to the current navigation waypoint
// @Field: DesYaw: the vehicle's desired heading
// @Field: Yaw: the vehicle's current heading
// @Field: XTrack: the vehicle's current distance from the current travel segment

    { LOG_NTUN_MSG, sizeof(log_Nav_Tuning),
      "NTUN", "QfffHf", "TimeUS,WpDist,WpBrg,DesYaw,Yaw,XTrack", "smhhhm", "F000B0" },
    
// @LoggerMessage: STER
// @Description: Steering related messages
// @Field: TimeUS: Time since system startup
// @Field: SteerIn: Steering input
// @Field: SteerOut: Normalized steering output 
// @Field: DesLatAcc: Desired lateral acceleration
// @Field: LatAcc: Actual lateral acceleration
// @Field: DesTurnRate: Desired turn rate
// @Field: TurnRate: Actual turn rate
    
    { LOG_STEERING_MSG, sizeof(log_Steering),
      "STER", "Qhfffff",   "TimeUS,SteerIn,SteerOut,DesLatAcc,LatAcc,DesTurnRate,TurnRate", "s--ookk", "F--0000" },

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
    
    { LOG_GUIDEDTARGET_MSG, sizeof(log_GuidedTarget),
      "GUIP",  "QBffffff",    "TimeUS,Type,pX,pY,pZ,vX,vY,vZ", "s-mmmnnn", "F-000000" },
};

/**
 * @brief Get the number of rover-specific log message structures
 * 
 * @details Returns the count of log message definitions in the log_structure
 *          array. Used by AP_Logger to properly size internal buffers and
 *          iterate through available message types.
 * 
 * @return uint8_t Number of log structures defined in log_structure array
 * 
 * @note This count includes LOG_COMMON_STRUCTURES plus rover-specific messages
 * @note Called during logger initialization
 * 
 * @see log_structure array for message definitions
 * Source: Rover/Log.cpp:311-314
 */
uint8_t Rover::get_num_log_structures() const
{
    return ARRAY_SIZE(log_structure);
}

#endif  // LOGGING_ENABLED
