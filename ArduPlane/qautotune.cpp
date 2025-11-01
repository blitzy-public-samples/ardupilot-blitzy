/**
 * @file qautotune.cpp
 * @brief QuadPlane automatic tuning implementation for ArduPlane
 * 
 * @details This file implements the QAutoTune class which provides automatic
 *          tuning functionality for QuadPlane attitude controllers. It inherits
 *          from AC_AutoTune_Multi and adapts the multicopter autotuning algorithm
 *          for use in QuadPlane vehicles (fixed-wing aircraft with VTOL capabilities).
 *          
 *          The autotuning process optimizes PID controller parameters for roll,
 *          pitch, and yaw rate controllers by performing systematic test maneuvers
 *          and analyzing the vehicle's response characteristics.
 *          
 *          Key responsibilities:
 *          - Initialize autotuning mode with appropriate position hold behavior
 *          - Provide pilot input interfaces during tuning sequences
 *          - Configure vertical motion limits for safe tuning operations
 *          - Log PID performance data during tuning maneuvers
 *          
 * @note This implementation is conditionally compiled with QAUTOTUNE_ENABLED,
 *       typically available only in SITL (Software In The Loop) simulation
 *       unless explicitly enabled for hardware platforms.
 *       
 * @warning Autotuning performs aggressive maneuvers and should only be conducted
 *          with adequate altitude, in calm wind conditions, and with an experienced
 *          pilot ready to take manual control if needed.
 * 
 * @see AC_AutoTune_Multi for the base autotuning algorithm implementation
 * @see QuadPlane for the QuadPlane flight mode and control integration
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include "Plane.h"
#include "qautotune.h"

#if QAUTOTUNE_ENABLED

/**
 * @brief Initialize QAUTOTUNE mode for QuadPlane automatic tuning
 * 
 * @details This method sets up the autotuning mode by verifying QuadPlane
 *          availability and initializing the internal autotuning state machine.
 *          It determines whether to use position hold based on the previous
 *          flight mode, enabling hover-in-place tuning if coming from QLOITER.
 *          
 *          Initialization sequence:
 *          1. Verify QuadPlane subsystem is available and functional
 *          2. Determine position hold requirement from previous mode
 *          3. Initialize autotuning internals with attitude and position controllers
 *          4. Configure AHRS view for attitude reference
 *          
 *          Position hold behavior:
 *          - Enabled if previous mode was QLOITER (maintains GPS position)
 *          - Disabled otherwise (allows manual positioning during tuning)
 * 
 * @return true if initialization successful and autotuning can proceed
 * @return false if QuadPlane unavailable or initialization failed
 * 
 * @note This is called automatically when entering QAUTOTUNE mode via mode switch
 * @note Requires functional QuadPlane with operational attitude and position controllers
 * 
 * @warning Initialization failure will prevent autotuning mode entry and return
 *          vehicle to previous flight mode for safety
 * 
 * @see AC_AutoTune_Multi::init_internals() for base initialization logic
 * @see QuadPlane::available() for QuadPlane availability check
 */
bool QAutoTune::init()
{
    if (!plane.quadplane.available()) {
        return false;
    }

    // use position hold while tuning if we were in QLOITER
    bool position_hold = (plane.previous_mode == &plane.mode_qloiter);

    return init_internals(position_hold,
                          plane.quadplane.attitude_control,
                          plane.quadplane.pos_control,
                          plane.quadplane.ahrs_view);
}

/**
 * @brief Get pilot's desired climb rate during autotuning
 * 
 * @details Retrieves the pilot's commanded vertical velocity from throttle
 *          stick input, allowing the pilot to maintain or adjust altitude
 *          during autotuning maneuvers. This override delegates to the
 *          QuadPlane's standard climb rate calculation, ensuring consistent
 *          behavior with normal VTOL flight modes.
 *          
 *          The climb rate is derived from:
 *          - Throttle stick position (center = 0 climb rate)
 *          - Configured climb rate limits (PILOT_SPEED_UP, PILOT_SPEED_DN)
 *          - Expo curves applied to pilot input
 * 
 * @return float Desired climb rate in centimeters per second (cm/s)
 *         - Positive values indicate climb (ascent)
 *         - Negative values indicate descent
 *         - Zero indicates altitude hold
 * 
 * @note This method is called continuously during autotuning to allow pilot
 *       altitude control while the autotuning algorithm controls roll/pitch/yaw
 * @note Units are centimeters per second to maintain consistency with
 *       ArduPilot's internal position controller units
 * 
 * @see QuadPlane::get_pilot_desired_climb_rate_cms() for implementation details
 * @see AC_PosControl for vertical velocity controller usage
 */
float QAutoTune::get_pilot_desired_climb_rate_cms(void) const
{
    return plane.quadplane.get_pilot_desired_climb_rate_cms();
}

/**
 * @brief Get pilot's desired roll, pitch angles and yaw rate during autotuning
 * 
 * @details Retrieves pilot's commanded attitude from RC stick inputs, allowing
 *          manual attitude corrections during autotuning maneuvers. This method
 *          implements special handling to distinguish between centered sticks
 *          (autotuning takes full control) and active pilot input (pilot override).
 *          
 *          Input processing logic:
 *          - If roll AND pitch sticks centered: Returns zero for both axes
 *            (allows autotuning algorithm full control of roll/pitch)
 *          - If either stick moved: Returns navigation controller attitude targets
 *            (allows pilot to reposition or correct attitude)
 *          - Yaw rate: Always derived from rudder stick input regardless of
 *            roll/pitch stick positions
 *          
 *          Coordinate frame: Body frame angles
 *          - Roll: Right-hand rule around forward axis (positive = right wing down)
 *          - Pitch: Right-hand rule around right axis (positive = nose up)
 *          - Yaw rate: Right-hand rule around down axis (positive = nose right)
 * 
 * @param[out] des_roll_rad  Desired roll angle in radians
 * @param[out] des_pitch_rad Desired pitch angle in radians  
 * @param[out] des_yaw_rate_rads Desired yaw rate in radians per second
 * 
 * @note RC stick deadzone is applied before checking for centered position
 * @note Navigation controller targets (nav_roll_cd, nav_pitch_cd) are used
 *       when pilot provides input, maintaining consistency with other flight modes
 * @note Unit conversion: centidegrees (cd) to radians via cd_to_rad() utility
 * 
 * @see cd_to_rad() for unit conversion implementation
 * @see QuadPlane::get_desired_yaw_rate_cds() for yaw rate calculation
 */
void QAutoTune::get_pilot_desired_rp_yrate_rad(float &des_roll_rad, float &des_pitch_rad, float &des_yaw_rate_rads)
{
    if (plane.channel_roll->get_control_in() == 0 && plane.channel_pitch->get_control_in() == 0) {
        des_roll_rad = 0.0;
        des_pitch_rad = 0.0;
    } else {
        des_roll_rad = cd_to_rad(plane.nav_roll_cd);
        des_pitch_rad = cd_to_rad(plane.nav_pitch_cd);
    }
    des_yaw_rate_rads = cd_to_rad(plane.quadplane.get_desired_yaw_rate_cds());
}

/**
 * @brief Initialize vertical axis speed and acceleration limits for autotuning
 * 
 * @details Configures the position controller's vertical motion constraints to
 *          ensure safe altitude management during autotuning maneuvers. Sets
 *          both commanded velocity limits and error correction limits to prevent
 *          excessive vertical rates that could interfere with tuning or cause
 *          unsafe altitude deviations.
 *          
 *          Configured limits:
 *          - Maximum descent rate: PILOT_SPEED_DN (typically 2.5 m/s)
 *          - Maximum climb rate: PILOT_SPEED_UP (typically 2.5 m/s)
 *          - Vertical acceleration: PILOT_ACCEL_Z (typically 2.5 m/s²)
 *          
 *          Two limit categories configured:
 *          1. Primary motion limits (set_max_speed_accel_U_cm):
 *             Controls commanded vertical velocity targets
 *          2. Correction limits (set_correction_speed_accel_U_cmss):
 *             Controls position error correction rates
 *          
 *          Coordinate system: NED frame (North-East-Down)
 *          - Positive U velocity = downward motion (descent)
 *          - Negative U velocity = upward motion (climb)
 * 
 * @note Called during autotuning initialization to establish safe flight envelope
 * @note Uses pilot-configured parameters to match normal flight behavior
 * @note Units converted from m/s to cm/s (multiply by 100) for internal consistency
 * 
 * @warning These limits are critical for preventing altitude loss during aggressive
 *          autotuning maneuvers - do not disable or modify without careful testing
 * 
 * @see AC_PosControl::set_max_speed_accel_U_cm() for velocity limit implementation
 * @see AC_PosControl::set_correction_speed_accel_U_cmss() for correction limits
 * @see QuadPlane parameters PILOT_SPEED_UP, PILOT_SPEED_DN, PILOT_ACCEL_Z
 */
void QAutoTune::init_z_limits()
{
    // set vertical speed and acceleration limits
    plane.quadplane.pos_control->set_max_speed_accel_U_cm(-plane.quadplane.get_pilot_velocity_z_max_dn(),
                                                       plane.quadplane.pilot_speed_z_max_up*100,
                                                       plane.quadplane.pilot_accel_z*100);
    plane.quadplane.pos_control->set_correction_speed_accel_U_cmss(-plane.quadplane.get_pilot_velocity_z_max_dn(),
                                                              plane.quadplane.pilot_speed_z_max_up*100,
                                                              plane.quadplane.pilot_accel_z*100);
}


#if HAL_LOGGING_ENABLED
/**
 * @brief Log VTOL PID controller data during autotuning test maneuvers
 * 
 * @details Records detailed PID controller performance data to binary logs during
 *          autotuning "twitch" maneuvers. This data captures controller response
 *          characteristics used by the autotuning algorithm to evaluate and optimize
 *          PID parameters. Each axis (roll, pitch, yaw) is logged separately with
 *          complete PID state information.
 *          
 *          Logged data for each axis includes:
 *          - Target rate (desired angular velocity)
 *          - Actual rate (measured angular velocity from gyroscopes)
 *          - Error (target - actual)
 *          - P term (proportional response)
 *          - I term (integral accumulator state)
 *          - D term (derivative response)
 *          - FF term (feedforward contribution)
 *          
 *          Log message types:
 *          - LOG_PIQR_MSG: QuadPlane roll rate PID data
 *          - LOG_PIQP_MSG: QuadPlane pitch rate PID data
 *          - LOG_PIQY_MSG: QuadPlane yaw rate PID data
 *          
 * @note This method is called at high frequency during test maneuvers to capture
 *       detailed response dynamics for analysis
 * @note Logging is conditionally compiled with HAL_LOGGING_ENABLED flag
 * @note Log data essential for post-flight analysis and manual tuning verification
 * 
 * @see AP_Logger::Write_PID() for PID log message format
 * @see AC_AttitudeControl for rate controller implementations
 * @see AC_AutoTune_Multi for autotuning algorithm that processes this data
 */
void QAutoTune::log_pids(void)
{
    AP::logger().Write_PID(LOG_PIQR_MSG, plane.quadplane.attitude_control->get_rate_roll_pid().get_pid_info());
    AP::logger().Write_PID(LOG_PIQP_MSG, plane.quadplane.attitude_control->get_rate_pitch_pid().get_pid_info());
    AP::logger().Write_PID(LOG_PIQY_MSG, plane.quadplane.attitude_control->get_rate_yaw_pid().get_pid_info());
}
#endif

#endif // QAUTOTUNE_ENABLED

