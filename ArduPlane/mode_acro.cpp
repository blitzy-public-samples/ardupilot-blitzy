/**
 * @file mode_acro.cpp
 * @brief ACRO flight mode implementation for aggressive aerobatic flight
 * 
 * @details This file implements ACRO mode for ArduPlane, designed for aggressive
 *          aerobatic flight with rate control on all three axes. ACRO mode provides
 *          direct rate control for experienced pilots performing aerobatic maneuvers.
 *          
 *          Features:
 *          - Pure rate control on roll, pitch, and optionally yaw axes
 *          - Optional attitude locking when sticks are centered (ACRO_LOCKING parameter)
 *          - Traditional rate control with axis locking (ACRO_LOCKING=1)
 *          - Advanced quaternion-based 3D acro with continuous attitude hold (ACRO_LOCKING=2)
 *          - Expo curves on all stick inputs for smoother control feel
 *          - Airspeed-based control scaling for consistent response across speed range
 *          
 *          ACRO mode is intended for experienced pilots and provides minimal stabilization,
 *          allowing full aerobatic capability including sustained inverted flight, knife-edge,
 *          and other advanced maneuvers.
 * 
 * @note ACRO mode requires well-tuned rate controllers (PTCH_RATE_*, RLL_RATE_*)
 * @warning This mode provides minimal stability augmentation - not suitable for beginners
 * 
 * @see Mode class for base flight mode interface
 * @see ArduPlane/mode.h for ModeAcro class definition
 */
#include "mode.h"
#include "Plane.h"

/**
 * @brief Initialize ACRO mode on entry
 * 
 * @details This method is called when the flight mode transitions into ACRO mode.
 *          It initializes the ACRO mode state by:
 *          - Clearing roll lock state (acro_state.locked_roll = false)
 *          - Clearing pitch lock state (acro_state.locked_pitch = false)
 *          - Capturing current aircraft attitude quaternion for quaternion-based acro
 *          
 *          The captured quaternion serves as the initial target attitude for
 *          quaternion-based 3D acro mode (when ACRO_LOCKING=2) and ensures smooth
 *          transition into the mode without sudden attitude jumps.
 * 
 * @return true Always returns true, indicating mode entry always succeeds
 * 
 * @note Entry cannot fail for ACRO mode - mode is always available when armed
 * @note The attitude quaternion is captured from AHRS regardless of which acro
 *       stabilization method will be used
 * 
 * Source: ArduPlane/mode_acro.cpp:4-10
 */
bool ModeAcro::_enter()
{
    acro_state.locked_roll = false;
    acro_state.locked_pitch = false;
    IGNORE_RETURN(ahrs.get_quaternion(acro_state.q));
    return true;
}

/**
 * @brief Update navigation targets for ACRO mode
 * 
 * @details This method updates the navigation roll and pitch targets (nav_roll_cd,
 *          nav_pitch_cd) based on the current lock state for telemetry and logging.
 *          
 *          **Roll Target Update**:
 *          - If roll is locked (acro_state.locked_roll): Sets nav_roll_cd to the
 *            accumulated locked roll error for telemetry display
 *          - If roll is unlocked: Sets nav_roll_cd to current AHRS roll angle for
 *            telemetry to track actual aircraft attitude
 *          
 *          **Pitch Target Update**:
 *          - If pitch is locked (acro_state.locked_pitch): Sets nav_pitch_cd to the
 *            locked pitch angle target
 *          - If pitch is unlocked: Sets nav_pitch_cd to current AHRS pitch angle
 *          
 *          These targets are used primarily for ground control station display and
 *          logging, and do not directly affect the control outputs in ACRO mode.
 * 
 * @note Called at navigation rate (typically 10-50Hz depending on scheduler configuration)
 * @note In traditional ACRO mode, these values reflect either the lock target or current attitude
 * @note In quaternion ACRO mode (ACRO_LOCKING=2), these are set differently in stabilize_quaternion()
 * 
 * Source: ArduPlane/mode_acro.cpp:12-25
 */
void ModeAcro::update()
{
    // handle locked/unlocked control
    if (acro_state.locked_roll) {
        plane.nav_roll_cd = acro_state.locked_roll_err;
    } else {
        plane.nav_roll_cd = ahrs.roll_sensor;
    }
    if (acro_state.locked_pitch) {
        plane.nav_pitch_cd = acro_state.locked_pitch_cd;
    } else {
        plane.nav_pitch_cd = ahrs.pitch_sensor;
    }
}

/**
 * @brief Main control loop execution for ACRO mode
 * 
 * @details This is the primary execution method called at the main loop rate for ACRO mode.
 *          It orchestrates throttle output and selects the appropriate stabilization method
 *          based on configuration parameters.
 *          
 *          **Execution Sequence**:
 *          1. Output pilot throttle directly (no throttle stabilization in ACRO)
 *          2. Select stabilization method based on configuration:
 *          
 *          **Quaternion-Based 3D Acro** (stabilize_quaternion):
 *          - Enabled when ALL conditions met:
 *            - ACRO_LOCKING parameter is set to 2
 *            - ACRO_YAW_RATE parameter is greater than 0
 *            - Yaw rate controller is enabled and available
 *          - Provides continuous 3D attitude locking with quaternion math
 *          - Allows sustained inverted, knife-edge, and arbitrary 3D orientations
 *          
 *          **Traditional Rate Control with Locking** (stabilize):
 *          - Used when quaternion acro conditions not met
 *          - Provides rate control on roll/pitch with optional axis locking
 *          - Roll and pitch can lock independently when sticks centered (if ACRO_LOCKING enabled)
 *          - Yaw control can be rate, damped, or manual depending on parameters
 * 
 * @note Called at main loop rate (typically 400Hz for most ArduPlane configurations)
 * @note Throttle is passed through directly without any stabilization or limiting
 * @warning Quaternion acro requires well-tuned rate controllers - ensure proper tuning before use
 * 
 * Source: ArduPlane/mode_acro.cpp:27-40
 */
void ModeAcro::run()
{
    output_pilot_throttle();

    if (plane.g.acro_locking == 2 && plane.g.acro_yaw_rate > 0 &&
        plane.yawController.rate_control_enabled()) {
        // we can do 3D acro locking
        stabilize_quaternion();
        return;
    }

    // Normal acro
    stabilize();
}

/**
 * @brief Traditional ACRO stabilization with rate control and optional axis locking
 * 
 * @details This method implements the classic ACRO mode stabilization algorithm with
 *          sophisticated control that varies by axis based on stick input and configuration.
 *          It provides rate control when sticks are active, with optional attitude locking
 *          when sticks are centered (if ACRO_LOCKING parameter is enabled).
 *          
 *          **Roll Control**:
 *          - **When ACRO_LOCKING enabled and roll stick centered**:
 *            - Locks current roll angle when stick is first centered
 *            - Accumulates gyro error (locked_roll_err) to track drift from locked angle
 *            - Uses position controller (rollController.get_servo_out) to hold the locked angle
 *            - Disables roll integrator to prevent windup during position hold
 *            - Prevents inverted spin situations that can occur with pure rate control near poles
 *          - **When roll stick is active**:
 *            - Pure rate control at ACRO_ROLL_RATE degrees/second
 *            - Rate proportional to stick deflection with expo curve applied
 *            - Clears lock state to allow immediate return to rate control
 *          
 *          **Pitch Control**:
 *          - **When ACRO_LOCKING enabled and pitch stick centered**:
 *            - Locks pitch angle when stick is first released (captures ahrs.pitch_sensor)
 *            - Uses position controller to maintain locked pitch angle
 *            - Enables pitch integrator (unlike roll) to help with inverted flight stability
 *          - **When pitch stick is active**:
 *            - Pure rate control at ACRO_PITCH_RATE degrees/second
 *            - Rate proportional to stick deflection with expo curve applied
 *            - Clears lock state
 *          
 *          **Yaw Control** (three possible modes):
 *          - **If ACRO_YAW_RATE > 0 and rate control available**:
 *            - Yaw rate control at ACRO_YAW_RATE degrees/second
 *            - Allows yaw rate commands proportional to rudder stick with expo
 *          - **If ACRO_YAW_DAMPER flight option enabled**:
 *            - Coordinated turn yaw damping (calc_nav_yaw_coordinated)
 *            - Automatically coordinates turns to minimize sideslip
 *          - **Otherwise**:
 *            - Direct manual rudder pass-through (rudder_input)
 *            - No stabilization, pilot has direct control
 * 
 * @note Expo curves are applied to all pilot stick inputs for smoother control feel around center
 * @note Speed scaling is applied to all control outputs for airspeed compensation
 * @note nav_roll_cd and nav_pitch_cd are updated by update() method for telemetry
 * @warning ACRO_LOCKING helps prevent inverted spin situations but requires properly tuned
 *          rate controllers - ensure RLL_RATE_* and PTCH_RATE_* parameters are well tuned
 * @warning Integrator is disabled for roll lock but enabled for pitch lock - different
 *          behavior helps with different axis characteristics during aerobatics
 * 
 * Source: ArduPlane/mode_acro.cpp:46-127
 */
void ModeAcro::stabilize()
{
    const float speed_scaler = plane.get_speed_scaler();
    const float rexpo = plane.roll_in_expo(true);
    const float pexpo = plane.pitch_in_expo(true);
    float roll_rate = (rexpo/SERVO_MAX) * plane.g.acro_roll_rate;
    float pitch_rate = (pexpo/SERVO_MAX) * plane.g.acro_pitch_rate;

    IGNORE_RETURN(ahrs.get_quaternion(acro_state.q));

    /*
      check for special roll handling near the pitch poles
     */
    if (plane.g.acro_locking && is_zero(roll_rate)) {
        /*
          we have no roll stick input, so we will enter "roll locked"
          mode, and hold the roll we had when the stick was released
         */
        if (!acro_state.locked_roll) {
            acro_state.locked_roll = true;
            acro_state.locked_roll_err = 0;
        } else {
            acro_state.locked_roll_err += ahrs.get_gyro().x * plane.G_Dt;
        }
        int32_t roll_error_cd = -degrees(acro_state.locked_roll_err)*100;
        plane.nav_roll_cd = ahrs.roll_sensor + roll_error_cd;
        // try to reduce the integrated angular error to zero. We set
        // 'stabilize' to true, which disables the roll integrator
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.rollController.get_servo_out(roll_error_cd,
                                                                                             speed_scaler,
                                                                                             true, false));
    } else {
        /*
          aileron stick is non-zero, use pure rate control until the
          user releases the stick
         */
        acro_state.locked_roll = false;
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.rollController.get_rate_out(roll_rate,  speed_scaler));
    }

    if (plane.g.acro_locking && is_zero(pitch_rate)) {
        /*
          user has zero pitch stick input, so we lock pitch at the
          point they release the stick
         */
        if (!acro_state.locked_pitch) {
            acro_state.locked_pitch = true;
            acro_state.locked_pitch_cd = ahrs.pitch_sensor;
        }
        // try to hold the locked pitch. Note that we have the pitch
        // integrator enabled, which helps with inverted flight
        plane.nav_pitch_cd = acro_state.locked_pitch_cd;
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.pitchController.get_servo_out(plane.nav_pitch_cd - ahrs.pitch_sensor,
                                                                                               speed_scaler,
                                                                                               false, false));
    } else {
        /*
          user has non-zero pitch input, use a pure rate controller
         */
        acro_state.locked_pitch = false;
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.pitchController.get_rate_out(pitch_rate, speed_scaler));
    }

    float rudder_output;
    if (plane.g.acro_yaw_rate > 0 && plane.yawController.rate_control_enabled()) {
        // user has asked for yaw rate control with yaw rate scaled by ACRO_YAW_RATE
        const float rudd_expo = plane.rudder_in_expo(true);
        const float yaw_rate = (rudd_expo/SERVO_MAX) * plane.g.acro_yaw_rate;
        rudder_output = plane.yawController.get_rate_out(yaw_rate,  speed_scaler, false);
    } else if (plane.flight_option_enabled(FlightOptions::ACRO_YAW_DAMPER)) {
        // use yaw controller
        rudder_output = plane.calc_nav_yaw_coordinated();
    } else {
        /*
          manual rudder
        */
        rudder_output = plane.rudder_input();
    }

    output_rudder_and_steering(rudder_output);

}

/**
 * @brief Advanced 3D quaternion-based ACRO with continuous attitude locking
 * 
 * @details This method implements an advanced quaternion-based attitude control system for
 *          full 3D aerobatic flight with continuous attitude locking capability. Unlike
 *          traditional ACRO, this method can maintain arbitrary 3D attitudes including
 *          sustained inverted flight, knife-edge, and any orientation in 3D space.
 *          
 *          **Algorithm Overview**:
 *          The algorithm maintains a target attitude quaternion that is continuously updated
 *          based on pilot input and current aircraft state. When pilot releases the sticks,
 *          the aircraft locks to the current attitude regardless of orientation.
 *          
 *          **Detailed Algorithm Steps**:
 *          
 *          1. **Get Pilot Desired Rates**:
 *             - Extract roll, pitch, yaw rates from stick inputs
 *             - Apply expo curves for smoother control feel around center
 *             - Scale by ACRO_ROLL_RATE, ACRO_PITCH_RATE, ACRO_YAW_RATE parameters
 *             - Determine active/inactive state for each axis
 *          
 *          2. **Integrate Target Quaternion**:
 *             - Convert rate vector (deg/s) to radians and scale by time step (G_Dt)
 *             - Rotate target quaternion by rate vector using rotate_fast()
 *             - Normalize quaternion to prevent numerical drift
 *             - This continuously updates the desired attitude based on pilot commands
 *          
 *          3. **Calculate Attitude Error**:
 *             - Get current attitude quaternion from AHRS
 *             - Compute error quaternion: error = ahrs_q.inverse() * target_q
 *             - Convert error quaternion to axis-angle representation (error_angle1)
 *             - Error vector represents rotation needed to reach target attitude
 *          
 *          4. **Limit Error Accumulation**:
 *             - Constrain error to maximum of 0.2 seconds worth of rotation per axis
 *             - Max error = ACRO_*_RATE * 0.2 seconds (prevents excessive corrections)
 *             - On stick release transition: Reset error to zero for smooth lock engagement
 *             - Prevents violent corrections on rapid maneuvers or loss of control
 *          
 *          5. **Reset Error on Stick Release**:
 *             - Detects transition from active to inactive on each axis
 *             - Sets max_err_*_rad to 0 on transition to zero error instantly
 *             - Enables smooth lock-in to current attitude without overshoot
 *          
 *          6. **Convert Error to Desired Rates**:
 *             - Divide error angle by tau time constant (from rate controllers)
 *             - tau determines how aggressively error is corrected
 *             - Smaller tau = faster correction, larger tau = gentler correction
 *             - Convert from rad/s to deg/s for controller compatibility
 *          
 *          7. **Override with Pilot Rates When Active**:
 *             - If roll stick active: Use pilot roll rate instead of error correction
 *             - If pitch stick active: Use pilot pitch rate instead of error correction
 *             - If yaw stick active: Use pilot yaw rate instead of error correction
 *             - Allows direct rate control when pilot is actively maneuvering
 *          
 *          8. **Call Rate Controllers**:
 *             - Pass desired rates (pilot or error-correction) to rate controllers
 *             - Rate controllers output servo positions with airspeed scaling
 *             - Aileron, elevator, rudder all driven by rate control
 *          
 *          **Behavior Modes**:
 *          - **Active Stick Input**: Aircraft responds with direct rate control at pilot commanded rates
 *          - **Stick Centered**: Aircraft locks current 3D attitude and actively corrects to maintain it
 *          - **Zero Throttle on Ground**: Target quaternion reset to current attitude to prevent windup
 *          
 *          **3D Capability**:
 *          Unlike traditional ACRO which can have issues near pitch poles (near vertical),
 *          quaternion acro handles all attitudes equally well including:
 *          - Sustained inverted flight (180° roll)
 *          - Knife-edge flight (90° roll)
 *          - Vertical climbs/dives (90° pitch)
 *          - Arbitrary 3D orientations
 * 
 * @note Requires ACRO_LOCKING=2 parameter setting to enable
 * @note Requires ACRO_YAW_RATE > 0 to enable yaw rate control
 * @note Requires yaw rate controller to be enabled and available
 * @note Enables true 3D flight capability including sustained inverted and knife-edge
 * @note Max error limit (0.2s) prevents excessive correction attempts on rapid maneuvers
 * @note Target quaternion automatically resets when on ground with zero throttle to prevent windup
 * @note Telemetry values (nav_roll_cd, nav_pitch_cd) updated with target attitude for GCS display
 * 
 * @warning Requires well-tuned rate controllers - untested or poorly tuned rate controllers
 *          can cause oscillations or loss of control in arbitrary orientations
 * @warning Advanced feature for experienced pilots - practice in SITL or at altitude first
 * @warning The 0.2s error limit protects against excessive corrections but may allow drift
 *          in very aggressive maneuvers with weak rate controller tuning
 * 
 * Source: ArduPlane/mode_acro.cpp:132-227
 */
void ModeAcro::stabilize_quaternion()
{
    const float speed_scaler = plane.get_speed_scaler();
    auto &q = acro_state.q;
    const float rexpo = plane.roll_in_expo(true);
    const float pexpo = plane.pitch_in_expo(true);
    const float yexpo = plane.rudder_in_expo(true);

    // get pilot desired rates
    float roll_rate = (rexpo/SERVO_MAX) * plane.g.acro_roll_rate;
    float pitch_rate = (pexpo/SERVO_MAX) * plane.g.acro_pitch_rate;
    float yaw_rate = (yexpo/SERVO_MAX) * plane.g.acro_yaw_rate;
    bool roll_active = !is_zero(roll_rate);
    bool pitch_active = !is_zero(pitch_rate);
    bool yaw_active = !is_zero(yaw_rate);

    // integrate target attitude
    Vector3f r{ float(radians(roll_rate)), float(radians(pitch_rate)), float(radians(yaw_rate)) };
    r *= plane.G_Dt;
    q.rotate_fast(r);
    q.normalize();

    // fill in target roll/pitch for GCS/logs
    plane.nav_roll_cd = degrees(q.get_euler_roll())*100;
    plane.nav_pitch_cd = degrees(q.get_euler_pitch())*100;

    // get AHRS attitude
    Quaternion ahrs_q;
    IGNORE_RETURN(ahrs.get_quaternion(ahrs_q));

    // zero target if not flying, no stick input and zero throttle
    if (is_zero(plane.get_throttle_input()) &&
        !plane.is_flying() &&
        is_zero(roll_rate) &&
        is_zero(pitch_rate) &&
        is_zero(yaw_rate)) {
        // cope with sitting on the ground with neutral sticks, no throttle
        q = ahrs_q;
    }

    // get error in attitude
    Quaternion error_quat = ahrs_q.inverse() * q;
    Vector3f error_angle1;
    error_quat.to_axis_angle(error_angle1);

    // don't let too much error build up, limit to 0.2s
    const float max_error_t = 0.2;
    float max_err_roll_rad  = radians(plane.g.acro_roll_rate*max_error_t);
    float max_err_pitch_rad = radians(plane.g.acro_pitch_rate*max_error_t);
    float max_err_yaw_rad   = radians(plane.g.acro_yaw_rate*max_error_t);

    if (!roll_active && acro_state.roll_active_last) {
        max_err_roll_rad = 0;
    }
    if (!pitch_active && acro_state.pitch_active_last) {
        max_err_pitch_rad = 0;
    }
    if (!yaw_active && acro_state.yaw_active_last) {
        max_err_yaw_rad = 0;
    }

    Vector3f desired_rates = error_angle1;
    desired_rates.x = constrain_float(desired_rates.x, -max_err_roll_rad, max_err_roll_rad);
    desired_rates.y = constrain_float(desired_rates.y, -max_err_pitch_rad, max_err_pitch_rad);
    desired_rates.z = constrain_float(desired_rates.z, -max_err_yaw_rad, max_err_yaw_rad);

    // correct target based on max error
    q.rotate_fast(desired_rates - error_angle1);
    q.normalize();

    // convert to desired body rates
    desired_rates.x /= plane.rollController.tau();
    desired_rates.y /= plane.pitchController.tau();
    desired_rates.z /= plane.pitchController.tau(); // no yaw tau parameter, use pitch

    desired_rates *= degrees(1.0);

    if (roll_active) {
        desired_rates.x = roll_rate;
    }
    if (pitch_active) {
        desired_rates.y = pitch_rate;
    }
    if (yaw_active) {
        desired_rates.z = yaw_rate;
    }

    // call to rate controllers
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron,  plane.rollController.get_rate_out(desired_rates.x, speed_scaler));
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.pitchController.get_rate_out(desired_rates.y, speed_scaler));
    output_rudder_and_steering(plane.yawController.get_rate_out(desired_rates.z,  speed_scaler, false));

    acro_state.roll_active_last = roll_active;
    acro_state.pitch_active_last = pitch_active;
    acro_state.yaw_active_last = yaw_active;
}
