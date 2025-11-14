/**
 * @file servos.cpp
 * @brief Servo control implementation for pitch and yaw tracking
 * 
 * @details Implements PID-based servo control with support for three servo types:
 *          - SERVO_TYPE_POSITION: Position-controlled servos with PID feedback (default)
 *          - SERVO_TYPE_ONOFF: Bang-bang control for limit-switch actuators
 *          - SERVO_TYPE_CR: Continuous rotation servos with speed control
 * 
 *          The servo control system drives the antenna tracker's pitch (elevation) and
 *          yaw (azimuth) axes to point at a target. The position-controlled mode uses
 *          closed-loop PID control for smooth, accurate tracking. The on/off mode provides
 *          bang-bang control for mechanical systems with limit switches. The continuous
 *          rotation mode controls rotation speed rather than position.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include "Tracker.h"

/*
 * Code to move pitch and yaw servos to attain a target heading or pitch
 */

/**
 * @brief Initializes servo channels and configures tracking parameters
 * 
 * @details Sets up servo channel mappings, angle ranges, and low-pass filters. Configures:
 *          - CH_YAW (typically RC1) for yaw/azimuth control with default function k_tracker_yaw
 *          - CH_PITCH (typically RC2) for pitch/elevation control with default function k_tracker_pitch
 *          - Angle ranges in centidegrees: yaw ±(YAW_RANGE/2), pitch from PITCH_MIN to PITCH_MAX
 *          - Low-pass filters at SERVO_OUT_FILT_HZ cutoff frequency for smooth servo motion
 * 
 *          This initialization must be called during tracker startup to ensure servo channels
 *          are properly configured before any tracking operations begin.
 * 
 * @note Servo angle ranges configured in centidegrees for precision
 * @note Low-pass filtering smooths servo commands to prevent jitter
 */
void Tracker::init_servos()
{
    // update assigned functions and enable auxiliary servos
    AP::srv().enable_aux_servos();

    // Assign default servo functions: CH_YAW (RC1) for azimuth, CH_PITCH (RC2) for elevation
    SRV_Channels::set_default_function(CH_YAW, SRV_Channel::k_tracker_yaw);
    SRV_Channels::set_default_function(CH_PITCH, SRV_Channel::k_tracker_pitch);

    // yaw range is +/- (YAW_RANGE parameter/2) converted to centi-degrees
    SRV_Channels::set_angle(SRV_Channel::k_tracker_yaw, g.yaw_range * 100/2);

    // pitch range is +/- (PITCH_MIN/MAX parameters/2) converted to centi-degrees
    SRV_Channels::set_angle(SRV_Channel::k_tracker_pitch, (-g.pitch_min+g.pitch_max) * 100/2);

    SRV_Channels::calc_pwm();
    SRV_Channels::output_ch_all();

    yaw_servo_out_filt.set_cutoff_frequency(SERVO_OUT_FILT_HZ);
    pitch_servo_out_filt.set_cutoff_frequency(SERVO_OUT_FILT_HZ);
}

/**
 * @brief Updates pitch servo based on configured servo type
 * 
 * @details Dispatches to appropriate pitch control function based on SERVO_PITCH_TYPE parameter:
 *          - SERVO_TYPE_POSITION: Standard position-controlled servo with PID (default)
 *          - SERVO_TYPE_ONOFF: Bang-bang control for limit-switch actuators
 *          - SERVO_TYPE_CR: Continuous rotation servo with speed control
 * 
 *          The aim is to drive the board's AHRS pitch to the requested pitch, so the board
 *          (and therefore the antenna) will be pointing at the target elevation angle.
 * 
 * @param[in] pitch Target pitch angle in degrees
 * 
 * @note Servo type determines control strategy - position servos most common
 * 
 * @see update_pitch_position_servo()
 * @see update_pitch_onoff_servo()
 * @see update_pitch_cr_servo()
 */
void Tracker::update_pitch_servo(float pitch)
{
    switch ((enum ServoType)g.servo_pitch_type.get()) {
    case SERVO_TYPE_ONOFF:
        update_pitch_onoff_servo(pitch);
        break;

    case SERVO_TYPE_CR:
        update_pitch_cr_servo(pitch);
        break;

    case SERVO_TYPE_POSITION:
    default:
        update_pitch_position_servo();
        break;
    }
}

/**
 * @brief Updates position-controlled pitch servo using PID feedback
 * 
 * @details Implements closed-loop PID control to drive tracker pitch to target angle:
 *          - Calculates PID correction based on nav_status.angle_error_pitch
 *          - Applies position limits (PITCH_MIN to PITCH_MAX) with integrator anti-windup
 *          - Uses low-pass filter for smooth servo motion
 *          - Servo output in centidegrees scaled to PWM by SRV_Channels
 * 
 *          The aim is to drive the board's AHRS pitch to the requested pitch, so the board
 *          (and therefore the antenna) will be pointing at the target elevation angle.
 * 
 * @note Requires RC2_MIN/MAX configured so servo drives full pitch range (-90° to +90°)
 * @note PID tuning via PITCH2SRV_P/I/D/IMAX parameters affects tracking smoothness
 * 
 * @warning Incorrect RC2_MIN/MAX or RC2_REV settings can cause unstable tracking
 */
void Tracker::update_pitch_position_servo()
{
    // Convert pitch limits from degrees to centidegrees for precision
    int32_t pitch_min_cd = g.pitch_min*100;
    int32_t pitch_max_cd = g.pitch_max*100;
    
    // Servo Configuration Requirements:
    // Need to configure your servo so that increasing servo_out causes increase in pitch/elevation (ie pointing higher into the sky,
    // above the horizon. On my antenna tracker this requires the pitch/elevation servo to be reversed
    // param set RC2_REV -1
    //
    // The pitch servo (RC channel 2) is configured for servo_out of -9000-0-9000 servo_out,
    // which will drive the servo from RC2_MIN to RC2_MAX usec pulse width.
    // Therefore, you must set RC2_MIN and RC2_MAX so that your servo drives the antenna altitude between -90 to 90 exactly
    // To drive my HS-645MG servos through their full 180 degrees of rotational range, I have to set:
    // param set RC2_MAX 2540
    // param set RC2_MIN 640
    //
    // You will also need to tune the pitch PID to suit your antenna and servos. I use:
    // PITCH2SRV_P      0.100000
    // PITCH2SRV_I      0.020000
    // PITCH2SRV_D      0.000000
    // PITCH2SRV_IMAX   4000.000000

    // Calculate new servo position: PID update adds correction to current position
    float new_servo_out = SRV_Channels::get_output_scaled(SRV_Channel::k_tracker_pitch) + g.pidPitch2Srv.update_error(nav_status.angle_error_pitch, G_Dt);

    // Position limit pitch servo and reset integrator to prevent windup
    if (new_servo_out <= pitch_min_cd) {
        new_servo_out = pitch_min_cd;
        g.pidPitch2Srv.reset_I();  // Reset integrator at lower limit to prevent windup
    }
    if (new_servo_out >= pitch_max_cd) {
        new_servo_out = pitch_max_cd;
        g.pidPitch2Srv.reset_I();  // Reset integrator at upper limit to prevent windup
    }
    // rate limit pitch servo
    SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_pitch, new_servo_out);

    // Apply low-pass filter for smooth servo motion (prevents jitter)
    if (pitch_servo_out_filt_init) {
        pitch_servo_out_filt.apply(new_servo_out, G_Dt);
    } else {
        // Initialize filter on first call with current servo position
        pitch_servo_out_filt.reset(new_servo_out);
        pitch_servo_out_filt_init = true;
    }
}


/**
 * @brief Updates on/off pitch servo for bang-bang control
 * 
 * @details Implements bang-bang control for actuators with limit switches:
 *          - Stops if error within acceptable threshold (ONOFF_PITCH_RATE * ONOFF_PITCH_MINTIME)
 *          - Commands full speed in direction to reduce error
 *          - Used for winch-style or limit-switched elevation actuators
 * 
 *          The aim is to drive the board's AHRS pitch to the requested pitch using
 *          simple on/off commands rather than proportional control.
 * 
 * @param[in] pitch Current pitch angle in degrees
 * 
 * @note On/off mode for mechanical systems without position feedback
 */
void Tracker::update_pitch_onoff_servo(float pitch) const
{
    int32_t pitch_min_cd = g.pitch_min*100;
    int32_t pitch_max_cd = g.pitch_max*100;

    float acceptable_error = g.onoff_pitch_rate * g.onoff_pitch_mintime;
    if (fabsf(nav_status.angle_error_pitch) < acceptable_error) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_pitch, 0);
    } else if ((nav_status.angle_error_pitch > 0) && (pitch*100>pitch_min_cd)) {
        // Positive error means we are pointing too low, so push the servo up
        // (negative servo output raises elevation in typical configuration)
        SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_pitch, -9000);
    } else if (pitch*100<pitch_max_cd) {
        // Negative error means we are pointing too high, so push the servo down
        // (positive servo output lowers elevation in typical configuration)
        SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_pitch, 9000);
    }
}

/**
 * @brief Updates continuous rotation pitch servo with speed control
 * 
 * @details Uses PID to output speed command for continuous rotation servos. Output constrained
 *          to configured pitch range. The PID controller calculates rotation speed based on
 *          tracking error rather than position.
 * 
 * @param[in] pitch Current pitch angle in degrees
 * 
 * @note CR mode for continuous rotation servos - output is speed not position
 */
void Tracker::update_pitch_cr_servo(float pitch)
{
    const float pitch_out = constrain_float(g.pidPitch2Srv.update_error(nav_status.angle_error_pitch, G_Dt), -(-g.pitch_min+g.pitch_max) * 100/2, (-g.pitch_min+g.pitch_max) * 100/2);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_pitch, pitch_out);
}

/**
 * @brief Updates yaw servo based on configured servo type
 * 
 * @details Dispatches to appropriate yaw control function based on SERVO_YAW_TYPE parameter:
 *          - SERVO_TYPE_POSITION: Standard position-controlled servo with PID (default)
 *          - SERVO_TYPE_ONOFF: Bang-bang control for limit-switch actuators
 *          - SERVO_TYPE_CR: Continuous rotation servo with speed control
 * 
 *          The aim is to drive the board's AHRS yaw to the requested yaw, so the board
 *          (and therefore the antenna) will be pointing at the target azimuth angle.
 * 
 * @param[in] yaw Target yaw angle in degrees
 * 
 * @note Yaw control similar to pitch but handles 360° azimuth range
 * 
 * @see update_yaw_position_servo()
 * @see update_yaw_onoff_servo()
 * @see update_yaw_cr_servo()
 */
void Tracker::update_yaw_servo(float yaw)
{
	switch ((enum ServoType)g.servo_yaw_type.get()) {
    case SERVO_TYPE_ONOFF:
        update_yaw_onoff_servo(yaw);
        break;

    case SERVO_TYPE_CR:
        update_yaw_cr_servo(yaw);
        break;

    case SERVO_TYPE_POSITION:
    default:
        update_yaw_position_servo();
        break;
    }
}

/**
 * @brief Updates position-controlled yaw servo using PID feedback
 * 
 * @details Implements "Antenna as Ballerina" algorithm for azimuth tracking:
 *          - PID control drives tracker yaw to target azimuth
 *          - Handles antenna mount misalignment with North
 *          - Accommodates moving/rotating mount platforms
 *          - Position limits at ±(YAW_RANGE/2) with integrator anti-windup
 *          - Low-pass filtered for smooth motion
 * 
 *          The aim is to drive the board's AHRS yaw to the requested yaw, so the board
 *          (and therefore the antenna) will be pointing at the target azimuth.
 * 
 * @note Requires RC1_MIN/MAX configured for full 360° azimuth range (-180° to +180° relative to mount)
 * @note 'Ballerina' algorithm handles non-North-aligned and moving mounts
 * 
 * @warning Incorrect RC1_MIN/MAX or RC1_REV causes tracking instability
 */
void Tracker::update_yaw_position_servo()
{
    int32_t yaw_limit_cd = g.yaw_range*100/2;

    // "Antenna as Ballerina" Algorithm:
    // Use with antenna that do not have continuously rotating servos, ie at some point in rotation
    // the servo limits are reached and the servo has to slew 360 degrees to the 'other side' to keep tracking.
    //
    // This algorithm accounts for the fact that the antenna mount may not be aligned with North
    // (in fact, any alignment is permissible), and that the alignment may change (possibly rapidly) over time
    // (as when the antenna is mounted on a moving, turning vehicle)
    //
    // Servo Configuration Requirements:
    // With my antenna mount, large pwm output drives the antenna anticlockwise, so need:
    // param set RC1_REV -1
    // to reverse the servo. Yours may be different
    //
    // You MUST set RC1_MIN and RC1_MAX so that your servo drives the antenna azimuth from -180 to 180 relative
    // to the mount.
    // To drive my HS-645MG servos through their full 180 degrees of rotational range and therefore the
    // antenna through a full 360 degrees, I have to set:
    // param set RC1_MAX 2380
    // param set RC1_MIN 680
    // According to the specs at https://www.servocity.com/html/hs-645mg_ultra_torque.html,
    // that should be 600 through 2400, but the azimuth gearing in my antenna pointer is not exactly 2:1

    /*
      Error Sign Convention:
      - A positive error means that we need to rotate clockwise
      - A negative error means that we need to rotate counter-clockwise

      Use our current yawspeed to determine if we are moving in the
      right direction
     */

    // PID update calculates servo correction based on angle error
    float servo_change = g.pidYaw2Srv.update_error(nav_status.angle_error_yaw, G_Dt);
    // Constrain changes and absolute position to prevent excessive servo motion
    servo_change = constrain_float(servo_change, -18000, 18000);
    float new_servo_out = constrain_float(SRV_Channels::get_output_scaled(SRV_Channel::k_tracker_yaw) + servo_change, -18000, 18000);

    // Position limit yaw servo and reset integrator to prevent windup
    if (new_servo_out <= -yaw_limit_cd) {
        new_servo_out = -yaw_limit_cd;
        g.pidYaw2Srv.reset_I();  // Reset integrator at lower limit to prevent windup
    }
    if (new_servo_out >= yaw_limit_cd) {
        new_servo_out = yaw_limit_cd;
        g.pidYaw2Srv.reset_I();  // Reset integrator at upper limit to prevent windup
    }

    SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_yaw, new_servo_out);

    // Apply low-pass filter for smooth servo motion (prevents jitter)
    if (yaw_servo_out_filt_init) {
        yaw_servo_out_filt.apply(new_servo_out, G_Dt);
    } else {
        // Initialize filter on first call with current servo position
        yaw_servo_out_filt.reset(new_servo_out);
        yaw_servo_out_filt_init = true;
    }
}


/**
 * @brief Updates on/off yaw servo for bang-bang control
 * 
 * @details Implements bang-bang azimuth control:
 *          - Stops if error within acceptable threshold (ONOFF_YAW_RATE * ONOFF_YAW_MINTIME)
 *          - Commands full speed clockwise or counter-clockwise to reduce error
 *          - Used for limit-switched azimuth actuators
 * 
 *          The aim is to drive the board's AHRS yaw to the requested yaw using
 *          simple on/off commands rather than proportional control.
 * 
 * @param[in] yaw Current yaw angle in degrees
 * 
 * @note On/off mode for azimuth systems without position feedback
 */
void Tracker::update_yaw_onoff_servo(float yaw) const
{
    float acceptable_error = g.onoff_yaw_rate * g.onoff_yaw_mintime;
    if (fabsf(nav_status.angle_error_yaw * 0.01f) < acceptable_error) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_yaw, 0);
    } else if (nav_status.angle_error_yaw * 0.01f > 0) {
        // Positive error means we are counter-clockwise of the target, so
        // move clockwise (positive servo output)
        SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_yaw, 18000);
    } else {
        // Negative error means we are clockwise of the target, so
        // move counter-clockwise (negative servo output)
        SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_yaw, -18000);
    }
}

/**
 * @brief Updates continuous rotation yaw servo with speed control
 * 
 * @details Uses PID to output speed command for CR servos. Output constrained to yaw range.
 *          The PID controller calculates rotation speed based on tracking error rather than
 *          position.
 * 
 * @param[in] yaw Current yaw angle in degrees
 * 
 * @note CR mode for continuous rotation servos - output is rotation speed
 */
void Tracker::update_yaw_cr_servo(float yaw)
{
    const float yaw_out = constrain_float(-g.pidYaw2Srv.update_error(nav_status.angle_error_yaw, G_Dt), -g.yaw_range * 100/2, g.yaw_range * 100/2);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_yaw, yaw_out);
}
