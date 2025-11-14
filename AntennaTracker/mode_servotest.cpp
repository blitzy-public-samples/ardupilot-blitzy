/**
 * @file mode_servotest.cpp
 * @brief Servo test mode implementation for antenna tracker
 * 
 * @details Provides GCS-controlled direct servo testing for setup and calibration.
 *          This mode allows ground control station to command specific PWM values
 *          directly to yaw and pitch servos for testing, setup, and calibration
 *          purposes without engaging normal tracking algorithms.
 * 
 * @note This mode is intended for ground testing and servo calibration only,
 *       not for normal operational use.
 */

#include "Tracker.h"

/*
 * GCS controlled servo test mode
 */

/**
 * @brief Sets yaw or pitch servo PWM directly via GCS command
 * 
 * @details Allows ground control station to command specific PWM values to servos
 *          for testing and calibration. Validates servo number, constrains PWM to
 *          configured limits (SERVOx_MIN/MAX parameters), and outputs immediately
 *          to hardware. This bypasses normal tracking control loops and directly
 *          controls servo positions.
 *          
 *          The function performs the following operations:
 *          1. Converts servo_num from 1-indexed (GCS convention) to 0-indexed
 *          2. Validates the servo number is within valid range (yaw or pitch only)
 *          3. Sets the PWM value for the specified servo channel
 *          4. Constrains PWM to configured min/max limits
 *          5. Calculates final PWM values and outputs to all servo channels
 * 
 * @param[in] servo_num Servo identifier (1 for yaw, 2 for pitch) - gets decremented
 *                      internally to 0-indexed representation (0=CH_YAW, 1=CH_PITCH)
 * @param[in] pwm       PWM value in microseconds to command to servo (typically 1000-2000)
 * 
 * @return true if servo was set successfully, false if servo_num invalid
 * 
 * @warning Direct PWM commands bypass normal safety limits and tracking logic - use
 *          cautiously during setup only. Improper PWM values may cause mechanical
 *          damage or unexpected servo behavior.
 * 
 * @note This mode is intended for ground testing and servo calibration, not normal
 *       operation. Servos will hold commanded positions until mode is exited or new
 *       commands are received.
 * 
 * @see SRV_Channels::set_output_pwm()
 * @see SRV_Channels::constrain_pwm()
 * @see SRV_Channels::calc_pwm()
 * @see SRV_Channels::output_ch_all()
 */
bool ModeServoTest::set_servo(uint8_t servo_num, uint16_t pwm)
{
    // Convert servo_num from 1-indexed (GCS convention: 1=yaw, 2=pitch) to 
    // 0-indexed internal representation (0=CH_YAW, 1=CH_PITCH)
    servo_num--;

    // Validation ensures only valid servos (CH_YAW=0, CH_PITCH=1) are commanded
    // Exit immediately if servo_num is invalid (protects against out-of-range commands)
    if (servo_num != CH_YAW && servo_num != CH_PITCH) {
        return false;
    }

    // set yaw servo pwm and send output to servo
    if (servo_num == CH_YAW) {
        SRV_Channels::set_output_pwm(SRV_Channel::k_tracker_yaw, pwm);
        // SRV_Channels::constrain_pwm() enforces configured SERVOx_MIN/MAX limits
        // to prevent commanding PWM values outside safe mechanical range
        SRV_Channels::constrain_pwm(SRV_Channel::k_tracker_yaw);
    }

    // set pitch servo pwm and send output to servo
    if (servo_num == CH_PITCH) {
        SRV_Channels::set_output_pwm(SRV_Channel::k_tracker_pitch, pwm);
        // SRV_Channels::constrain_pwm() enforces configured SERVOx_MIN/MAX limits
        // to prevent commanding PWM values outside safe mechanical range
        SRV_Channels::constrain_pwm(SRV_Channel::k_tracker_pitch);
    }

    // calc_pwm() calculates final PWM values considering all configured parameters
    // output_ch_all() immediately outputs computed values to hardware servo channels
    SRV_Channels::calc_pwm();
    SRV_Channels::output_ch_all();
    
    // return success
    return true;
}
