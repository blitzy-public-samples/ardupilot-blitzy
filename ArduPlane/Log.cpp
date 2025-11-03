/**
 * @file Log.cpp
 * @brief ArduPlane flight data logging implementation
 * 
 * @details This file implements flight data logging for fixed-wing aircraft, including:
 * - Custom ArduPlane-specific log message definitions (CTUN, NTUN, STATUS, AETR, etc.)
 * - Logging functions for attitude, control tuning, navigation, and vehicle status
 * - Integration with AP_Logger subsystem for dataflash/SD card storage
 * - QuadPlane-specific logging when HAL_QUADPLANE_ENABLED
 * 
 * All logged data is written in binary format for efficient storage and can be
 * analyzed post-flight using tools like MAVExplorer or Mission Planner.
 * 
 * Key log message types:
 * - CTUN: Control tuning (roll/pitch/yaw controller performance, airspeed, throttle)
 * - NTUN: Navigation tuning (waypoint tracking, crosstrack error, altitude error)
 * - STATUS: Vehicle status (flying state, armed state, crash detection)
 * - AETR: Control surface outputs (aileron, elevator, throttle, rudder)
 * - QTUN: QuadPlane vertical tuning (when in VTOL mode)
 * 
 * @note This file is only compiled when HAL_LOGGING_ENABLED is defined
 * 
 * Source: ArduPlane/Log.cpp
 */

#include "Plane.h"

#if HAL_LOGGING_ENABLED

/**
 * @brief Write attitude and PID controller data to dataflash log
 * 
 * @details Logs the current vehicle attitude (roll, pitch, yaw) along with desired
 * attitude targets and PID controller performance data. This function is called at
 * the logging rate configured by MASK_LOG_ATTITUDE_* parameters.
 * 
 * For standard fixed-wing flight, logs:
 * - Desired nav_roll_cd and nav_pitch_cd targets
 * - Actual roll/pitch from AHRS
 * - Roll, pitch, yaw, and steering PID controller data
 * 
 * For QuadPlane VTOL flight (when HAL_QUADPLANE_ENABLED), additionally logs:
 * - QuadPlane rate controller PIDs (roll/pitch/yaw rates)
 * - Position controller PIDs (velocity NE, acceleration Z)
 * - Tailsitter-specific data
 * 
 * @note Called from Log_Write_FullRate() at rates up to 400Hz depending on configuration
 * @warning High logging rates consume SD card space quickly and may impact performance
 * 
 * @see Log_Write_FullRate()
 * @see Plane::nav_roll_cd
 * @see Plane::nav_pitch_cd
 */
void Plane::Log_Write_Attitude(void)
{
    Vector3f targets {       // Package up the targets into a vector for commonality with Copter usage of Log_Wrote_Attitude
        nav_roll_cd * 0.01f,
        nav_pitch_cd * 0.01f,
        0 //Plane does not have the concept of navyaw. This is a placeholder.
    };

#if HAL_QUADPLANE_ENABLED
    if (quadplane.show_vtol_view()) {
        // we need the attitude targets from the AC_AttitudeControl controller, as they
        // account for the acceleration limits.
        // Also, for bodyframe roll input types, _attitude_target_euler_angle is not maintained
        // since Euler angles are not used and it is a waste of cpu to compute them at the loop rate.
        // Get them from the quaternion instead:
        quadplane.attitude_control->get_attitude_target_quat().to_euler(targets.x, targets.y, targets.z);
        quadplane.ahrs_view->Write_AttitudeView(targets * RAD_TO_DEG);
    } else
#endif
            {
        ahrs.Write_Attitude(targets);
    }

#if HAL_QUADPLANE_ENABLED
    if (AP_HAL::millis() - quadplane.last_att_control_ms < 100) {
        // log quadplane PIDs separately from fixed wing PIDs
        logger.Write_PID(LOG_PIQR_MSG, quadplane.attitude_control->get_rate_roll_pid().get_pid_info());
        logger.Write_PID(LOG_PIQP_MSG, quadplane.attitude_control->get_rate_pitch_pid().get_pid_info());
        logger.Write_PID(LOG_PIQY_MSG, quadplane.attitude_control->get_rate_yaw_pid().get_pid_info());
        logger.Write_PID(LOG_PIQA_MSG, quadplane.pos_control->get_accel_U_pid().get_pid_info() );

        // Write tailsitter specific log at same rate as PIDs
        quadplane.tailsitter.write_log();
    }
    if (quadplane.in_vtol_mode() && quadplane.pos_control->is_active_NE()) {
        logger.Write_PID(LOG_PIDN_MSG, quadplane.pos_control->get_vel_NE_pid().get_pid_info_x());
        logger.Write_PID(LOG_PIDE_MSG, quadplane.pos_control->get_vel_NE_pid().get_pid_info_y());
    }
#endif

    logger.Write_PID(LOG_PIDR_MSG, rollController.get_pid_info());
    logger.Write_PID(LOG_PIDP_MSG, pitchController.get_pid_info());

    if (yawController.enabled()) {
        logger.Write_PID(LOG_PIDY_MSG, yawController.get_pid_info());
    }

    if (steerController.active()) {
        logger.Write_PID(LOG_PIDS_MSG, steerController.get_pid_info());
    }

    AP::ahrs().Log_Write();
}

/**
 * @brief High-rate data logging function called from main scheduler loop
 * 
 * @details Performs fast-rate logging of time-critical data based on configured
 * log bitmasks. This function is called from the scheduler at the highest logging
 * rate and selectively logs data based on these masks:
 * - MASK_LOG_ATTITUDE_FULLRATE: Logs attitude at 400Hz (highest rate)
 * - MASK_LOG_ATTITUDE_FAST: Logs attitude at 25Hz
 * - MASK_LOG_ATTITUDE_MED: Logs attitude at 10Hz
 * - MASK_LOG_NOTCH_FULLRATE: Logs harmonic notch filter data at high rate
 * 
 * The highest rate selected by the bitmask wins, allowing users to balance
 * logging detail against SD card space and write performance.
 * 
 * @note Called from main scheduler fast loop
 * @warning FULLRATE logging at 400Hz generates large log files quickly
 * 
 * @see Log_Write_Attitude()
 * @see should_log()
 */
void Plane::Log_Write_FullRate(void)
{
    // MASK_LOG_ATTITUDE_FULLRATE logs at 400Hz, MASK_LOG_ATTITUDE_FAST at 25Hz, MASK_LOG_ATTIUDE_MED logs at 10Hz
    // highest rate selected wins
    if (should_log(MASK_LOG_ATTITUDE_FULLRATE)) {
        Log_Write_Attitude();
    }
#if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
    if (should_log(MASK_LOG_NOTCH_FULLRATE)) {
        AP::ins().write_notch_log_messages();
    }
#endif
}


/**
 * @brief Log structure for control tuning data (CTUN message)
 * 
 * @details Packed structure containing fixed-wing control loop performance data.
 * This message logs the commanded vs achieved attitude, throttle outputs, and
 * airspeed estimates for analyzing controller tuning and performance.
 * 
 * Key fields:
 * - nav_roll_cd/nav_pitch_cd: Desired attitude in centidegrees
 * - roll/pitch: Achieved attitude in centidegrees
 * - throttle_out/rudder_out: Normalized control surface outputs
 * - throttle_dem: TECS energy controller throttle demand
 * - airspeed_estimate: Current airspeed estimate or measurement
 * - EAS2TAS: Equivalent to true airspeed ratio for altitude compensation
 * - groundspeed_undershoot: Undershoot when flying at minimum groundspeed (cm/s)
 * 
 * @note PACKED ensures no padding bytes for consistent binary log format
 */
struct PACKED log_Control_Tuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t nav_roll_cd;
    int16_t roll;
    int16_t nav_pitch_cd;
    int16_t pitch;
    float throttle_out;
    float rudder_out;
    float throttle_dem;
    float airspeed_estimate;
    uint8_t airspeed_estimate_status;
    float synthetic_airspeed;
    float EAS2TAS;
    int32_t groundspeed_undershoot;
};

/**
 * @brief Write control tuning data to dataflash log (CTUN message)
 * 
 * @details Logs comprehensive control loop performance data for fixed-wing flight,
 * including attitude targets, achieved attitude, control outputs, and airspeed.
 * This data is essential for:
 * - PID controller tuning (analyzing commanded vs achieved attitude)
 * - Airspeed sensor validation and tuning
 * - TECS energy management performance analysis
 * - Diagnosing control oscillations or instability
 * 
 * For QuadPlane vehicles, the pitch value is adjusted to show VTOL view when
 * in QuadPlane mode (quadplane.show_vtol_view() == true).
 * 
 * Airspeed sources logged:
 * - airspeed_estimate: Primary airspeed (sensor or EKF estimate)
 * - synthetic_airspeed: DCM synthetic airspeed (NaN if unavailable)
 * - airspeed_estimate_status: Type/source of airspeed estimate
 * 
 * @note Typically called at 10-25Hz (MASK_LOG_ATTITUDE_MED or _FAST)
 * @warning Accurate airspeed logging critical for tuning TECS and airspeed limits
 * 
 * @see TECS_controller
 * @see Plane::nav_roll_cd
 * @see Plane::nav_pitch_cd
 */
void Plane::Log_Write_Control_Tuning()
{
    float est_airspeed = 0;
    AP_AHRS::AirspeedEstimateType airspeed_estimate_type = AP_AHRS::AirspeedEstimateType::NO_NEW_ESTIMATE;
    ahrs.airspeed_estimate(est_airspeed, airspeed_estimate_type);

    float synthetic_airspeed;
    if (!ahrs.synthetic_airspeed(synthetic_airspeed)) {
        synthetic_airspeed = logger.quiet_nan();
    }

    int16_t pitch = ahrs.pitch_sensor - g.pitch_trim * 100;
#if HAL_QUADPLANE_ENABLED
    if (quadplane.show_vtol_view()) {
        pitch = quadplane.ahrs_view->pitch_sensor;
    }
#endif      
    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CTUN_MSG),
        time_us         : AP_HAL::micros64(),
        nav_roll_cd     : (int16_t)nav_roll_cd,
        roll            : (int16_t)ahrs.roll_sensor,
        nav_pitch_cd    : (int16_t)nav_pitch_cd,
        pitch           : pitch,
        throttle_out    : SRV_Channels::get_output_scaled(SRV_Channel::k_throttle),
        rudder_out      : SRV_Channels::get_output_scaled(SRV_Channel::k_rudder),
        throttle_dem    : TECS_controller.get_throttle_demand(),
        airspeed_estimate : est_airspeed,
        airspeed_estimate_status : (uint8_t)airspeed_estimate_type,
        synthetic_airspeed : synthetic_airspeed,
        EAS2TAS            : ahrs.get_EAS2TAS(),
        groundspeed_undershoot  : groundspeed_undershoot,
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
/**
 * @brief Log structure for offboard guided mode targets (OFG message)
 * 
 * @details Packed structure for advanced offboard guided mode control from companion
 * computers. This extends basic GUIDED mode with rate limiting and acceleration control.
 * Logs the target trajectory including position, velocity, and acceleration limits.
 * 
 * Fields:
 * - target_airspeed_cm: Commanded airspeed in cm/s
 * - target_airspeed_accel: Airspeed acceleration limit
 * - target_alt: Target altitude (frame specified by target_alt_frame)
 * - target_alt_rate: Vertical velocity target/limit
 * - target_mav_frame: MAVLink frame type received from GCS
 * - target_heading: Desired heading in degrees
 * - target_heading_limit: Heading rate/acceleration limit
 * - target_alt_frame: Internal altitude frame representation
 * 
 * @note Only compiled when AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
 * @see Plane::guided_state
 */
struct PACKED log_OFG_Guided {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float target_airspeed_cm;
    float target_airspeed_accel;
    float target_alt;
    float target_alt_rate;
    uint8_t target_mav_frame;   // received MavLink frame
    float target_heading;
    float target_heading_limit;
    uint8_t target_alt_frame;   // internal AltFrame
};

/**
 * @brief Write offboard guided mode targets to dataflash log (OFG message)
 * 
 * @details Logs the current guided mode trajectory targets when using advanced
 * offboard guided control with slew rate limiting. This data is useful for:
 * - Debugging companion computer guidance commands
 * - Analyzing trajectory smoothness and rate limiting
 * - Verifying MAVLink frame transformations
 * 
 * Only logs when in guided mode with active offboard control targets.
 * 
 * @note Only available when AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
 * @see Plane::guided_state
 * @see Log_Write_Guided()
 */
void Plane::Log_Write_OFG_Guided()
{
    struct log_OFG_Guided pkt = {
        LOG_PACKET_HEADER_INIT(LOG_OFG_MSG),
        time_us                : AP_HAL::micros64(),
        target_airspeed_cm     : (float)guided_state.target_airspeed_cm*(float)0.01,
        target_airspeed_accel  : guided_state.target_airspeed_accel,
        target_alt             : guided_state.target_location.alt * 0.01,
        target_alt_rate        : guided_state.target_alt_rate,
        target_mav_frame       : guided_state.target_mav_frame,
        target_heading         : guided_state.target_heading,
        target_heading_limit   : guided_state.target_heading_accel_limit,
        target_alt_frame       : static_cast<uint8_t>(guided_state.target_location.get_alt_frame()),
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}
#endif

/**
 * @brief Log structure for navigation tuning data (NTUN message)
 * 
 * @details Packed structure containing fixed-wing navigation controller performance.
 * Logs waypoint tracking accuracy, crosstrack error, altitude error, and target
 * trajectory for analyzing L1 controller performance and mission execution.
 * 
 * Key fields:
 * - wp_distance: Distance to current waypoint (meters)
 * - target_bearing_cd: Bearing to waypoint in centidegrees
 * - nav_bearing_cd: Desired track bearing in centidegrees
 * - altitude_error_cm: Altitude error in centimeters
 * - xtrack_error: Crosstrack distance from desired track (meters)
 * - xtrack_error_i: Integral of crosstrack error
 * - airspeed_error: Difference between actual and target airspeed
 * - target_lat/lng: Target waypoint coordinates
 * - target_alt_wp: Waypoint altitude
 * - target_alt_tecs: TECS controller target altitude
 * - target_airspeed: Commanded airspeed in cm/s
 * 
 * @note Essential for tuning L1 navigation controller and TECS
 */
struct PACKED log_Nav_Tuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float wp_distance;
    int16_t target_bearing_cd;
    int16_t nav_bearing_cd;
    int16_t altitude_error_cm;
    float   xtrack_error;
    float   xtrack_error_i;
    float   airspeed_error;
    int32_t target_lat;
    int32_t target_lng;
    int32_t target_alt_wp;
    int32_t target_alt_tecs;
    int32_t target_airspeed;
};

/**
 * @brief Write navigation tuning data to dataflash log (NTUN message)
 * 
 * @details Logs comprehensive navigation controller performance for fixed-wing
 * waypoint tracking. This data is critical for:
 * - Tuning L1 navigation controller parameters
 * - Analyzing waypoint tracking accuracy
 * - Diagnosing crosstrack error and path following
 * - Verifying TECS altitude tracking performance
 * - Monitoring airspeed controller performance
 * 
 * The logged data shows both the desired trajectory (target waypoint, bearing,
 * altitude) and tracking errors (crosstrack, altitude error, airspeed error).
 * 
 * Crosstrack error indicates lateral deviation from the desired track between
 * waypoints. Large crosstrack errors suggest L1 controller needs tuning.
 * 
 * @note Typically logged at 10-25Hz
 * @warning Large persistent errors indicate controller tuning issues
 * 
 * @see AP_L1_Control
 * @see TECS_controller
 * @see Plane::nav_controller
 */
void Plane::Log_Write_Nav_Tuning()
{
    struct log_Nav_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NTUN_MSG),
        time_us             : AP_HAL::micros64(),
        wp_distance         : auto_state.wp_distance,
        target_bearing_cd   : (int16_t)nav_controller->target_bearing_cd(),
        nav_bearing_cd      : (int16_t)nav_controller->nav_bearing_cd(),
        altitude_error_cm   : (int16_t)plane.calc_altitude_error_cm(),
        xtrack_error        : nav_controller->crosstrack_error(),
        xtrack_error_i      : nav_controller->crosstrack_error_integrator(),
        airspeed_error      : airspeed_error,
        target_lat          : next_WP_loc.lat,
        target_lng          : next_WP_loc.lng,
        target_alt_wp       : next_WP_loc.alt,
        target_alt_tecs     : tecs_target_alt_cm,
        target_airspeed     : target_airspeed_cm,
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

/**
 * @brief Log structure for vehicle status information (STATUS message)
 * 
 * @details Packed structure containing critical vehicle state information including
 * flight detection, arming status, crash detection, and flight stage. This data
 * is essential for post-flight analysis of vehicle behavior and safety events.
 * 
 * Fields:
 * - is_flying: Boolean indicating if aircraft is detected as flying
 * - is_flying_probability: Confidence level (0.0-1.0) that vehicle is airborne
 * - armed: Vehicle armed state (1=armed, 0=disarmed)
 * - safety: Hardware safety switch state
 * - is_crashed: Crash detection flag
 * - is_still: True when vehicle has no motion detected on any axis
 * - stage: Current flight stage (takeoff, normal, land, abort, etc.)
 * - impact: True if impact/collision detected
 * - throttle_supressed: True if throttle output is being suppressed
 * 
 * @note Critical for analyzing safety events and flight phase transitions
 * @warning is_crashed and impact flags indicate potential vehicle damage
 */
struct PACKED log_Status {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t is_flying;
    float is_flying_probability;
    uint8_t armed;
    uint8_t safety;
    bool is_crashed;
    bool is_still;
    uint8_t stage;
    bool impact;
    bool throttle_supressed;
};

/**
 * @brief Write vehicle status information to dataflash log (STATUS message)
 * 
 * @details Logs critical vehicle state for safety analysis and flight phase tracking.
 * This message provides essential context for understanding vehicle behavior:
 * 
 * Flying detection (is_flying, is_flying_probability):
 * - Used to determine when vehicle has launched and landed
 * - Affects arming checks, throttle suppression, and mode transitions
 * 
 * Safety states (armed, safety, is_crashed):
 * - Records arming events and safety switch state
 * - Logs crash detection events for post-flight analysis
 * 
 * Flight stage tracking:
 * - Records progression through takeoff, cruise, approach, landing phases
 * - Essential for analyzing automated takeoff/landing performance
 * 
 * @note Logged at moderate rate (typically 5-10Hz)
 * @warning Crash and impact flags should trigger immediate log download and inspection
 * 
 * @see Plane::is_flying()
 * @see Plane::crash_state
 * @see Plane::flight_stage
 */
void Plane::Log_Write_Status()
{
    struct log_Status pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STATUS_MSG)
        ,time_us   : AP_HAL::micros64()
        ,is_flying   : is_flying()
        ,is_flying_probability : isFlyingProbability
        ,armed       : hal.util->get_soft_armed()
        ,safety      : static_cast<uint8_t>(hal.util->safety_switch_state())
        ,is_crashed  : crash_state.is_crashed
        ,is_still    : AP::ins().is_still()
        ,stage       : static_cast<uint8_t>(flight_stage)
        ,impact      : crash_state.impact_detected
        ,throttle_supressed : throttle_suppressed
        };

    logger.WriteBlock(&pkt, sizeof(pkt));
}

/**
 * @brief Log structure for normalized control surface outputs (AETR message)
 * 
 * @details Packed structure containing pre-mixer control surface demand values.
 * These are the normalized outputs from the attitude/navigation controllers
 * before being mixed and scaled to servo PWM values. AETR stands for:
 * Aileron, Elevator, Throttle, Rudder.
 * 
 * Fields:
 * - aileron: Normalized aileron output (-4500 to 4500, where ±4500 = ±45°)
 * - elevator: Normalized elevator output (-4500 to 4500)
 * - throttle: Normalized throttle output (-100 to 100, where 100 = full throttle)
 * - rudder: Normalized rudder output (-4500 to 4500)
 * - flap: Normalized flap output (0 to 100, where 100 = full flaps)
 * - steering: Normalized ground steering output (-4500 to 4500)
 * - speed_scaler: Surface movement scaling factor based on airspeed
 * 
 * @note Pre-mixer values allow analysis of controller outputs independent of servo mixing
 * @see SRV_Channels for servo mixing and output scaling
 */
struct PACKED log_AETR {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float aileron;
    float elevator;
    float throttle;
    float rudder;
    float flap;
    float steering;
    float speed_scaler;
};

/**
 * @brief Write normalized control surface outputs to dataflash log (AETR message)
 * 
 * @details Logs pre-mixer control surface demand values from the attitude and
 * navigation controllers. These normalized values show what the controllers are
 * commanding before servo mixing, reversing, and trim are applied.
 * 
 * This data is valuable for:
 * - Analyzing controller output saturation
 * - Verifying control mixing is working correctly
 * - Diagnosing control surface coordination issues
 * - Understanding speed scaling effects on control authority
 * 
 * Speed scaler (get_speed_scaler()):
 * - Reduces control surface deflection at high airspeeds
 * - Increases deflection at low airspeeds to maintain authority
 * - Helps prevent over-control and maintains stability across speed range
 * 
 * @note Compare with RCOU (servo outputs) to verify mixing and scaling
 * @see Log_Write_RC()
 * @see SRV_Channels::get_output_scaled()
 */
void Plane::Log_Write_AETR()
{
    struct log_AETR pkt = {
        LOG_PACKET_HEADER_INIT(LOG_AETR_MSG)
        ,time_us  : AP_HAL::micros64()
        ,aileron  : SRV_Channels::get_output_scaled(SRV_Channel::k_aileron)
        ,elevator : SRV_Channels::get_output_scaled(SRV_Channel::k_elevator)
        ,throttle : SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)
        ,rudder   : SRV_Channels::get_output_scaled(SRV_Channel::k_rudder)
        ,flap     : SRV_Channels::get_slew_limited_output_scaled(SRV_Channel::k_flap_auto)
        ,steering : SRV_Channels::get_output_scaled(SRV_Channel::k_steering)
        ,speed_scaler : get_speed_scaler(),
        };

    logger.WriteBlock(&pkt, sizeof(pkt));
}

/**
 * @brief Write RC (radio control) input and output data to dataflash log
 * 
 * @details Comprehensive logging of all RC-related data including:
 * - RCIN: Raw RC input values from receiver
 * - RCOUT: Servo/motor PWM output values
 * - RSSI: Receiver signal strength (if enabled and available)
 * - AETR: Pre-mixer control surface demands
 * 
 * This combined logging provides complete visibility into the RC input chain
 * through to final servo outputs, essential for diagnosing:
 * - RC signal quality and failsafe events
 * - Servo mixing and output configuration
 * - Control surface response and trim
 * - Radio link performance
 * 
 * @note Called at RC logging rate (typically 10-25Hz)
 * @see Log_Write_AETR()
 * @see AP_Logger::Write_RCIN()
 * @see AP_Logger::Write_RCOUT()
 */
void Plane::Log_Write_RC(void)
{
    logger.Write_RCIN();
    logger.Write_RCOUT();
#if AP_RSSI_ENABLED
    if (rssi.enabled()) {
        logger.Write_RSSI();
    }
#endif
    Log_Write_AETR();
}

/**
 * @brief Write guided mode control data to dataflash log
 * 
 * @details Logs guided mode-specific information when vehicle is under external
 * guidance control (typically from companion computer or GCS). Only logs when
 * in GUIDED mode and active guidance commands are present.
 * 
 * Logged data includes:
 * - PIDG: Heading PID controller when target heading is active
 * - OFG: Offboard guided trajectory targets (altitude, airspeed, heading with rates)
 * 
 * This data is essential for:
 * - Debugging companion computer guidance commands
 * - Tuning guided mode heading controller
 * - Analyzing trajectory tracking performance
 * - Verifying MAVLink guidance message handling
 * 
 * @note Only active when control_mode == mode_guided
 * @note Only compiled when AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
 * 
 * @see Log_Write_OFG_Guided()
 * @see Plane::guided_state
 * @see Plane::mode_guided
 */
void Plane::Log_Write_Guided(void)
{
#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    if (control_mode != &mode_guided) {
        return;
    }

    if (guided_state.target_heading_time_ms != 0) {
        logger.Write_PID(LOG_PIDG_MSG, g2.guidedHeading.get_pid_info());
    }

    if ( guided_state.target_location.alt != -1 || is_positive(guided_state.target_airspeed_cm) ) {
        Log_Write_OFG_Guided();
    }
#endif // AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
}

/**
 * @brief Log structure for incoming MAVLink COMMAND_INT messages
 * 
 * @details Packed structure for logging COMMAND_INT MAVLink messages received
 * by the vehicle. COMMAND_INT provides precise position commands using integer
 * coordinates (latitude/longitude in 1e7 format).
 * 
 * Fields:
 * - TimeUS: Timestamp in microseconds
 * - CId: Command ID (MAVLink command number)
 * - TSys: Target system ID
 * - TCmp: Target component ID
 * - cur: Current waypoint flag
 * - cont: Autocontinue flag
 * - Prm1-4: Command-specific parameters
 * - Lat: Latitude in 1e7 degrees
 * - Lng: Longitude in 1e7 degrees
 * - Alt: Altitude (frame depends on command)
 * - F: Frame type (MAV_FRAME enum)
 * 
 * @note Used for debugging MAVLink command reception and parameter values
 * @see MAVLink COMMAND_INT message definition
 */
struct PACKED log_CMDI {
    LOG_PACKET_HEADER;
    uint64_t TimeUS;
    uint16_t CId;
    uint8_t TSys;
    uint8_t TCmp;
    uint8_t cur;
    uint8_t cont;
    float Prm1;
    float Prm2;
    float Prm3;
    float Prm4;
    int32_t Lat;
    int32_t Lng;
    float Alt;
    uint8_t F;
};

/**
 * @brief Log message structure definitions for ArduPlane
 * 
 * @details Array defining all custom log message formats specific to fixed-wing
 * aircraft. Each entry specifies:
 * - Message ID (LOG_*_MSG constants)
 * - Structure size for validation
 * - Format string defining field types
 * - Field names for log analysis tools
 * - Units for each field
 * - Multipliers for fixed-point encoding
 * 
 * This array is used by AP_Logger to:
 * - Write correctly formatted binary log messages
 * - Generate log message documentation
 * - Enable post-flight log analysis tools (MAVExplorer, Mission Planner)
 * 
 * Type and unit information reference:
 * - Type format characters: See libraries/AP_Logger/LogStructure.h "Format characters"
 * - Units: See libraries/AP_Logger/LogStructure.h "log_Units"
 * - Common units: s=seconds, m=meters, d=degrees, n=no units, -=N/A
 * 
 * Message documentation uses @LoggerMessage tags for auto-generation.
 * 
 * @note LOG_COMMON_STRUCTURES included first (attitude, GPS, IMU, etc.)
 * @see AP_Logger::WriteBlock()
 * @see libraries/AP_Logger/LogStructure.h
 */
const struct LogStructure Plane::log_structure[] = {
    LOG_COMMON_STRUCTURES,

// @LoggerMessage: CTUN
// @Description: Control Tuning information
// @Field: TimeUS: Time since system startup
// @Field: NavRoll: desired roll
// @Field: Roll: achieved roll
// @Field: NavPitch: desired pitch assuming pitch trims are already applied
// @Field: Pitch: achieved pitch assuming pitch trims are already applied,ie "0deg" is level flight trimmed pitch attitude as shown on artificial horizon level line.
// @Field: ThO: scaled output throttle
// @Field: RdO: scaled output rudder
// @Field: ThD: demanded speed-height-controller throttle
// @Field: As: airspeed estimate (or measurement if airspeed sensor healthy and ARSPD_USE>0)
// @Field: AsT: airspeed type ( old estimate or source of new estimate)
// @FieldValueEnum: AsT: AP_AHRS::AirspeedEstimateType
// @Field: SAs: DCM's airspeed estimate, NaN if not available
// @Field: E2T: equivalent to true airspeed ratio
// @Field: GU: groundspeed undershoot when flying with minimum groundspeed

    { LOG_CTUN_MSG, sizeof(log_Control_Tuning),     
      "CTUN", "QccccffffBffi",    "TimeUS,NavRoll,Roll,NavPitch,Pitch,ThO,RdO,ThD,As,AsT,SAs,E2T,GU", "sdddd---n-n-n", "FBBBB---000-B" , true },

// @LoggerMessage: NTUN
// @Description: Navigation Tuning information - e.g. vehicle destination
// @Field: TimeUS: Time since system startup
// @Field: Dist: distance to the current navigation waypoint
// @Field: TBrg: bearing to the current navigation waypoint
// @Field: NavBrg: the vehicle's desired heading
// @Field: AltE: difference between current vehicle height and target height
// @Field: XT: the vehicle's current distance from the current travel segment
// @Field: XTi: integration of the vehicle's crosstrack error
// @Field: AsE: difference between vehicle's airspeed and desired airspeed
// @Field: TLat: target latitude
// @Field: TLng: target longitude
// @Field: TAW: target altitude WP
// @Field: TAT: target altitude TECS
// @Field: TAsp: target airspeed
    { LOG_NTUN_MSG, sizeof(log_Nav_Tuning),         
      "NTUN", "QfcccfffLLeee",  "TimeUS,Dist,TBrg,NavBrg,AltE,XT,XTi,AsE,TLat,TLng,TAW,TAT,TAsp", "smddmmmnDUmmn", "F0BBB0B0GG000" , true },

// @LoggerMessage: ATRP
// @Description: Plane AutoTune
// @Vehicles: Plane
// @Field: TimeUS: Time since system startup
// @Field: Axis: tuning axis
// @Field: State: tuning state
// @Field: Sur: control surface deflection
// @Field: PSlew: P slew rate
// @Field: DSlew: D slew rate
// @Field: FF0: FF value single sample
// @Field: FF: FF value
// @Field: P: P value
// @Field: I: I value
// @Field: D: D value
// @Field: Action: action taken
// @Field: RMAX: Rate maximum
// @Field: TAU: time constant
    { LOG_ATRP_MSG, sizeof(AP_AutoTune::log_ATRP),
      "ATRP", "QBBffffffffBff", "TimeUS,Axis,State,Sur,PSlew,DSlew,FF0,FF,P,I,D,Action,RMAX,TAU", "s#-dkk------ks", "F--00000000-00" , true },

// @LoggerMessage: STAT
// @Description: Current status of the aircraft
// @Field: TimeUS: Time since system startup
// @Field: isFlying: True if aircraft is probably flying
// @Field: isFlyProb: Probability that the aircraft is flying
// @Field: Armed: Arm status of the aircraft
// @Field: Safety: State of the safety switch
// @Field: Crash: True if crash is detected
// @Field: Still: True when vehicle is not moving in any axis
// @Field: Stage: Current stage of the flight
// @Field: Hit: True if impact is detected
// @Field: Sup: True if throttle is suppressed
    { LOG_STATUS_MSG, sizeof(log_Status),
      "STAT", "QBfBBBBBBB",  "TimeUS,isFlying,isFlyProb,Armed,Safety,Crash,Still,Stage,Hit,Sup", "s---------", "F---------" , true },

// @LoggerMessage: QTUN
// @Description: QuadPlane vertical tuning message
// @Field: TimeUS: Time since system startup
// @Field: ThI: throttle input
// @Field: ABst: angle boost
// @Field: ThO: throttle output
// @Field: ThH: calculated hover throttle
// @Field: DAlt: desired altitude
// @Field: Alt: achieved altitude
// @Field: BAlt: barometric altitude
// @Field: DCRt: desired climb rate
// @Field: CRt: climb rate
// @Field: TMix: transition throttle mix value
// @Field: Trn: Transition state: 0-AirspeedWait,1-Timer,2-Done / TailSitter: 0-FW Wait,1-VTOL Wait,2-Done
// @Field: Ast: bitmask of assistance flags
// @FieldBitmaskEnum: Ast: log_assistance_flags
#if HAL_QUADPLANE_ENABLED
    { LOG_QTUN_MSG, sizeof(QuadPlane::log_QControl_Tuning),
      "QTUN", "QffffffeccfBB", "TimeUS,ThI,ABst,ThO,ThH,DAlt,Alt,BAlt,DCRt,CRt,TMix,Trn,Ast", "s----mmmnn---", "F----00000---" , true },
#endif

// @LoggerMessage: PIQR
// @Description: QuadPlane Proportional/Integral/Derivative gain values for Roll rate
// @LoggerMessage: PIQP
// @Description: QuadPlane Proportional/Integral/Derivative gain values for Pitch rate
// @LoggerMessage: PIQY
// @Description: QuadPlane Proportional/Integral/Derivative gain values for Yaw rate
// @LoggerMessage: PIQA
// @Description: QuadPlane Proportional/Integral/Derivative gain values for vertical acceleration
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
#if HAL_QUADPLANE_ENABLED
    { LOG_PIQR_MSG, sizeof(log_PID),
      "PIQR", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS , true },
    { LOG_PIQP_MSG, sizeof(log_PID),
      "PIQP", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS , true },
    { LOG_PIQY_MSG, sizeof(log_PID),
      "PIQY", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS , true },
    { LOG_PIQA_MSG, sizeof(log_PID),
      "PIQA", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS , true },
#endif

// @LoggerMessage: TSIT
// @Description: tailsitter speed scailing values
// @Field: TimeUS: Time since system startup
// @Field: Ts: throttle scaling used for tilt motors
// @Field: Ss: speed scailing used for control surfaces method from Q_TAILSIT_GSCMSK
// @Field: Tmin: minimum output throttle calculated from disk thoery gain scale with Q_TAILSIT_MIN_VO
#if HAL_QUADPLANE_ENABLED
    { LOG_TSIT_MSG, sizeof(Tailsitter::log_tailsitter),
      "TSIT", "Qfff",  "TimeUS,Ts,Ss,Tmin", "s---", "F---" , true },
#endif

// @LoggerMessage: TILT
// @Description: Tiltrotor tilt values
// @Field: TimeUS: Time since system startup
// @Field: Tilt: Current tilt angle, 0 deg vertical, 90 deg horizontal
// @Field: FL: Front left tilt angle, 0 deg vertical, 90 deg horizontal
// @Field: FR: Front right tilt angle, 0 deg vertical, 90 deg horizontal
#if HAL_QUADPLANE_ENABLED
    { LOG_TILT_MSG, sizeof(Tiltrotor::log_tiltrotor),
      "TILT", "Qfff",  "TimeUS,Tilt,FL,FR", "sddd", "F---" , true },
#endif

// @LoggerMessage: PIDG
// @Description: Plane Proportional/Integral/Derivative gain values for Heading when using COMMAND_INT control.
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
    { LOG_PIDG_MSG, sizeof(log_PID),
      "PIDG", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS , true },

// @LoggerMessage: AETR
// @Description: Normalised pre-mixer control surface outputs
// @Field: TimeUS: Time since system startup
// @Field: Ail: Pre-mixer value for aileron output (between -4500 and 4500)
// @Field: Elev: Pre-mixer value for elevator output (between -4500 and 4500)
// @Field: Thr: Pre-mixer value for throttle output (between -100 and 100)
// @Field: Rudd: Pre-mixer value for rudder output (between -4500 and 4500)
// @Field: Flap: Pre-mixer value for flaps output (between 0 and 100)
// @Field: Steer: Pre-mixer value for steering output (between -4500 and 4500)
// @Field: SS: Surface movement / airspeed scaling value
    { LOG_AETR_MSG, sizeof(log_AETR),
      "AETR", "Qfffffff",  "TimeUS,Ail,Elev,Thr,Rudd,Flap,Steer,SS", "s-------", "F-------" , true },

#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
// @LoggerMessage: OFG
// @Description: OFfboard-Guided - an advanced version of GUIDED for companion computers that includes rate/s.  
// @Field: TimeUS: Time since system startup
// @Field: Arsp:  target airspeed cm
// @Field: ArspA:  target airspeed accel
// @Field: Alt:  target alt
// @Field: AltA: target alt velocity (rate of change)
// @Field: AltF: target alt frame (MAVLink)
// @Field: Hdg:  target heading
// @Field: HdgA: target heading lim
// @Field: AltL: target alt frame (Location)
    { LOG_OFG_MSG, sizeof(log_OFG_Guided),     
      "OFG", "QffffBffB",    "TimeUS,Arsp,ArspA,Alt,AltA,AltF,Hdg,HdgA,AltL", "snnmo-d--", "F--------" , true }, 
#endif
};

/**
 * @brief Get the number of custom log message structures defined for ArduPlane
 * 
 * @details Returns the count of ArduPlane-specific log message definitions in
 * the log_structure array. This count is used by AP_Logger to allocate resources
 * and iterate through message definitions.
 * 
 * @return Number of log structures (size of log_structure array)
 * 
 * @note Called by AP_Logger during initialization
 * @see Plane::log_structure
 */
uint8_t Plane::get_num_log_structures() const
{
    return ARRAY_SIZE(log_structure);
}

/**
 * @brief Write vehicle-specific startup messages to log
 * 
 * @details Logs important vehicle configuration and state information at startup
 * or when logging begins. This provides essential context for log analysis:
 * 
 * Logged information:
 * - QuadPlane configuration: Frame type and motor configuration (if enabled)
 * - Initial flight mode: Mode at startup and reason for mode selection
 * - Home position: Logged home and origin coordinates from AHRS
 * - GPS startup data: Satellite count, fix type, initial position
 * 
 * This startup data helps correlate log files with specific flights and provides
 * initial state for understanding subsequent events.
 * 
 * @note Called when logging starts (arm, log-on-boot, or explicit log start)
 * @warning Only first 200 bytes guaranteed available at startup
 * 
 * @see Plane::control_mode
 * @see QuadPlane::motors
 */
void Plane::Log_Write_Vehicle_Startup_Messages()
{
    // only 200(?) bytes are guaranteed by AP_Logger
#if HAL_QUADPLANE_ENABLED
    if (quadplane.initialised) {
        char frame_and_type_string[30];
        quadplane.motors->get_frame_and_type_string(frame_and_type_string, ARRAY_SIZE(frame_and_type_string));
        logger.Write_MessageF("QuadPlane %s", frame_and_type_string);
    }
#endif
    logger.Write_Mode(control_mode->mode_number(), control_mode_reason);
    ahrs.Log_Write_Home_And_Origin();
    gps.Write_AP_Logger_Log_Startup_messages();
}

#endif // HAL_LOGGING_ENABLED
