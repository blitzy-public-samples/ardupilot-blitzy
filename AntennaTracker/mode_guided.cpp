/**
 * @file mode_guided.cpp
 * @brief Guided mode implementation for antenna tracker
 * 
 * @details Allows external control (GCS or companion computer) to command 
 *          specific tracker attitude via MAVLink SET_ATTITUDE_TARGET messages.
 *          This mode enables programmatic control for applications like automated 
 *          tracking or remote operation.
 */

#include "mode.h"

#include "Tracker.h"
#include <GCS_MAVLink/GCS.h>

/**
 * @brief Updates guided mode by commanding tracker to GCS-specified attitude
 * 
 * @details Called at 50Hz while in GUIDED mode. Converts quaternion attitude target 
 *          (set via MAVLink) to Euler angles, calculates angle errors in earth frame 
 *          (centidegrees), transforms to body frame, and commands servos. Sends debug 
 *          telemetry every 5 seconds.
 *          
 *          Processing flow:
 *          1. Converts quaternion target (from MAVLink SET_ATTITUDE_TARGET) to Euler angles (radians)
 *          2. Sends periodic debug telemetry (5-second intervals) showing target angles in degrees
 *          3. Calculates earth-frame tracking errors in centidegrees via calc_angle_error()
 *          4. Transforms earth-frame angles to body-frame for servo control via convert_ef_to_bf()
 *          5. Commands pitch and yaw servos with body-frame angles
 * 
 * @note Guided mode enables external control for applications like automated tracking or remote operation
 * @note Target attitude set via ModeGuided::set_angle() called from MAVLink handler
 * @note Angles converted: Quaternion → Euler (radians) → Earth frame (centidegrees) → Body frame (radians) → Servo PWM
 * 
 * @see ModeGuided::set_angle()
 * @see calc_angle_error()
 * @see convert_ef_to_bf()
 */
void ModeGuided::update()
{
    // Static timestamp for 5-second debug output rate limiting
    static uint32_t last_debug;
    const uint32_t now = AP_HAL::millis();
    float target_roll, target_pitch, target_yaw;
    
    // Convert quaternion target (from MAVLink SET_ATTITUDE_TARGET) to Euler angles (radians)
    _target_att.to_euler(target_roll, target_pitch, target_yaw);
    
    // Rate-limited GCS debug text showing target angles in degrees (every 5 seconds)
    if (now - last_debug > 5000) {
        last_debug = now;
        gcs().send_text(MAV_SEVERITY_INFO, "target_yaw=%f target_pitch=%f", degrees(target_yaw), degrees(target_pitch));
    }
    
    // calc_angle_error() computes earth-frame tracking errors in centidegrees
    calc_angle_error(degrees(target_pitch)*100, degrees(target_yaw)*100, false);
    float bf_pitch;
    float bf_yaw;
    
    // convert_ef_to_bf() transforms earth-frame angles to body-frame for servo control
    convert_ef_to_bf(target_pitch, target_yaw, bf_pitch, bf_yaw);
    
    // Command pitch and yaw servos with body-frame angles
    tracker.update_pitch_servo(bf_pitch);
    tracker.update_yaw_servo(bf_yaw);
}
