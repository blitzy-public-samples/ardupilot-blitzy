/**
 * @file tuning.cpp
 * @brief In-flight transmitter tuning system for real-time parameter adjustment
 * 
 * @details This file implements the transmitter-based tuning system that allows
 *          pilots to adjust critical flight parameters in real-time using an RC
 *          channel (typically channel 6) and a tuning knob or slider on the transmitter.
 *          
 *          This system is distinct from AutoTune (control_autotune.cpp) which
 *          automatically determines optimal PID values through automated test maneuvers.
 *          The transmitter tuning system provides manual, real-time adjustment for
 *          fine-tuning during flight testing and development.
 *          
 *          **System Architecture:**
 *          - RC Channel: Dedicated tuning channel (rc_tuning) provides knob position
 *          - Parameter Selection: TUNE parameter (g.rc_tuning_param) selects which parameter to adjust
 *          - Value Range: TUNE_MIN and TUNE_MAX (g2.tuning_min/max) define safe parameter bounds
 *          - Update Rate: Called at 3.3Hz from scheduler
 *          - Value Scaling: Linear interpolation from knob position to parameter value
 *          
 *          **Safety Features:**
 *          - Bounded parameter ranges prevent dangerous values
 *          - Failsafe detection disables tuning on signal loss
 *          - Non-persistent changes (not saved to EEPROM during flight)
 *          - Logging of all tuning adjustments for post-flight analysis
 *          
 *          **Common Use Cases:**
 *          - PID tuning during initial vehicle setup and test flights
 *          - Real-time adjustment of rate controller gains
 *          - Optimizing position controller parameters
 *          - Adjusting navigation speeds and limits
 *          - Fine-tuning attitude response characteristics
 *          
 * @note This should not be confused with the AutoTune feature which can be found
 *       in control_autotune.cpp. AutoTune performs automatic PID optimization,
 *       while this system provides manual real-time adjustment.
 *       
 * @warning Improper tuning parameter ranges can lead to unstable flight or crashes.
 *          Always set conservative TUNE_MIN and TUNE_MAX values and test cautiously.
 *          
 * @see control_autotune.cpp for automated PID tuning
 * @see Parameters.cpp for TUNE parameter definitions
 * 
 * Source: ArduCopter/tuning.cpp
 */

#include "Copter.h"

/**
 * @brief Update flight parameters in real-time based on transmitter tuning knob position
 * 
 * @details This function implements the core transmitter tuning system, allowing pilots
 *          to adjust critical flight parameters during flight using a dedicated RC channel
 *          (typically channel 6). The system provides real-time parameter adjustment for
 *          PID gains, rate limits, navigation speeds, and other flight characteristics.
 *          
 *          **Tuning System Operation:**
 *          
 *          1. **Parameter Selection** (TUNE parameter):
 *             - Ground station or transmitter sets g.rc_tuning_param to select which parameter to tune
 *             - Valid values defined in enum (TUNING_STABILIZE_ROLL_PITCH_KP, TUNING_RATE_ROLL_PITCH_KP, etc.)
 *             - Only one parameter can be tuned at a time
 *          
 *          2. **Value Range Configuration** (TUNE_MIN and TUNE_MAX):
 *             - g2.tuning_min: Parameter value when knob is at minimum position
 *             - g2.tuning_max: Parameter value when knob is at maximum position
 *             - These bounds provide safety limits preventing dangerous parameter values
 *             - Must be set appropriately for each parameter being tuned
 *          
 *          3. **RC Input Processing**:
 *             - Reads normalized input from rc_tuning channel (-1.0 to 1.0)
 *             - Uses norm_input_ignore_trim() to get position independent of trim settings
 *             - Validates that RC signal is active and not in failsafe
 *          
 *          4. **Value Scaling and Interpolation**:
 *             - Linear interpolation from knob position to parameter value
 *             - Formula: value = lerp(tuning_min, tuning_max, knob_position, -1, 1)
 *             - Provides smooth, proportional control across full knob range
 *          
 *          5. **Parameter Application**:
 *             - Directly updates active controller parameters (not persistent)
 *             - Changes take effect immediately in current flight
 *             - Not saved to EEPROM automatically (prevents flash wear during flight)
 *             - Pilot must manually save desired values after landing
 *          
 *          6. **Logging**:
 *             - All tuning adjustments logged with TUNE message type
 *             - Records: parameter ID, current value, min/max bounds
 *             - Enables post-flight analysis of parameter effects
 *          
 *          **Supported Tuning Parameters:**
 *          
 *          - **Attitude Control**:
 *            - TUNING_STABILIZE_ROLL_PITCH_KP: Angle controller P gain for roll/pitch
 *            - TUNING_STABILIZE_YAW_KP: Angle controller P gain for yaw
 *            - TUNING_RC_FEEL_RP: Input shaping time constant (responsiveness)
 *          
 *          - **Rate Control PID**:
 *            - TUNING_RATE_ROLL_PITCH_KP/KI/KD: Combined roll/pitch rate PID gains
 *            - TUNING_RATE_ROLL_KP/KI/KD: Individual roll rate PID gains
 *            - TUNING_RATE_PITCH_KP/KI/KD: Individual pitch rate PID gains
 *            - TUNING_RATE_YAW_KP/KD: Yaw rate PID gains
 *            - TUNING_RATE_YAW_FILT: Yaw rate filter cutoff frequency
 *          
 *          - **Position Control**:
 *            - TUNING_LOITER_POSITION_KP: Horizontal position P gain
 *            - TUNING_VEL_XY_KP/KI: Horizontal velocity PID gains
 *            - TUNING_ALTITUDE_HOLD_KP: Vertical position P gain
 *            - TUNING_POS_CONTROL_ANGLE_MAX: Maximum lean angle for position control
 *          
 *          - **Altitude Control**:
 *            - TUNING_THROTTLE_RATE_KP: Vertical velocity P gain
 *            - TUNING_ACCEL_Z_KP/KI/KD: Vertical acceleration PID gains
 *          
 *          - **Navigation**:
 *            - TUNING_WP_SPEED: Waypoint navigation horizontal speed (cm/s)
 *            - TUNING_LOITER_MAX_XY_SPEED: Maximum loiter speed (cm/s)
 *            - TUNING_CIRCLE_RATE: Circle mode rotation rate (deg/s)
 *          
 *          - **Acro Mode**:
 *            - TUNING_ACRO_RP_RATE: Acro mode roll/pitch rate limit (deg/s)
 *            - TUNING_ACRO_YAW_RATE: Acro mode yaw rate limit (deg/s)
 *          
 *          - **Helicopter-Specific** (HELI_FRAME):
 *            - TUNING_HELI_EXTERNAL_GYRO: External gyro gain
 *            - TUNING_RATE_PITCH/ROLL/YAW_FF: Feedforward gains
 *          
 *          - **Multi-Rotor Specific**:
 *            - TUNING_RATE_MOT_YAW_HEADROOM: Motor yaw headroom
 *          
 *          - **Other**:
 *            - TUNING_DECLINATION: Compass declination (degrees)
 *            - TUNING_SYSTEM_ID_MAGNITUDE: System identification test magnitude
 *          
 *          **Safety Considerations:**
 *          
 *          - **Failsafe Protection**: Tuning disabled when RC signal lost or invalid
 *          - **Bounded Values**: TUNE_MIN/MAX prevent out-of-range parameter values
 *          - **Non-Persistent**: Changes not saved during flight to prevent EEPROM wear
 *          - **Validation**: Exits early if tuning channel or parameter not configured
 *          - **Rate Limiting**: 3.3Hz update rate prevents excessive parameter changes
 *          
 *          **Typical Usage Workflow:**
 *          
 *          1. Configure tuning channel on transmitter (assign knob/slider to channel 6)
 *          2. Set TUNE parameter via ground station to select parameter to adjust
 *          3. Set TUNE_MIN and TUNE_MAX to define safe parameter range
 *          4. Fly vehicle and adjust knob while observing flight characteristics
 *          5. Monitor telemetry or OSD for current parameter value
 *          6. Land and save parameters if desired values found
 *          7. Review TUNE logs to analyze parameter effects
 *          
 *          **Example Configuration:**
 *          ```
 *          // Tune roll/pitch rate P gain from 0.08 to 0.20
 *          TUNE = 4  // TUNING_RATE_ROLL_PITCH_KP
 *          TUNE_MIN = 0.08
 *          TUNE_MAX = 0.20
 *          // Adjust channel 6 knob during flight to find optimal value
 *          ```
 *          
 * @note This function should be called at 3.3Hz from the main scheduler.
 *       More frequent updates may cause excessive parameter changes, while
 *       less frequent updates reduce responsiveness to knob adjustments.
 *       
 * @note Parameter changes are not saved to EEPROM automatically. Pilots must
 *       manually save parameters after landing using the ground station if
 *       they wish to persist the tuned values.
 *       
 * @note The tuning channel (rc_tuning) is typically mapped to RC channel 6,
 *       but can be configured to any available channel via the RCn_OPTION parameters.
 *       
 * @warning Improper TUNE_MIN and TUNE_MAX values can result in unstable flight,
 *          loss of control, or vehicle crashes. Always set conservative bounds
 *          and test cautiously at altitude with sufficient margin for recovery.
 *          
 * @warning Do not tune parameters during critical flight phases (takeoff, landing,
 *          proximity to obstacles). Parameter changes can cause sudden attitude
 *          or position changes that may be dangerous in confined spaces.
 *          
 * @warning Some parameters (especially rate controller gains) interact with each
 *          other. Tuning one parameter may require retuning related parameters.
 *          Refer to tuning documentation for parameter interdependencies.
 *          
 * @see Parameters.cpp for TUNE parameter definitions and valid values
 * @see AC_AttitudeControl library for attitude and rate controller implementation
 * @see AC_PosControl library for position controller implementation
 * @see Log_Write_Parameter_Tuning() for tuning event logging
 * 
 * Source: ArduCopter/tuning.cpp:10-202
 */
void Copter::tuning()
{
    // **Safety Check 1: Verify tuning channel is configured**
    // The rc_tuning pointer references the RC channel designated for tuning (typically CH6).
    // If no channel is assigned (RCn_OPTION not set to TRANSMITTER_TUNING), exit immediately.
    // This prevents null pointer dereferencing and ensures pilot has explicitly enabled tuning.
    if (rc_tuning == nullptr) {
        return;
    }
    
    // **Safety Check 2: Validate tuning parameter and range configuration**
    // g.rc_tuning_param: Selects which parameter to tune (0 = disabled, >0 = specific parameter)
    // g2.tuning_min/max: Define the parameter value range for minimum and maximum knob positions
    // Exit if:
    //   - No tuning parameter selected (g.rc_tuning_param <= 0)
    //   - Both min and max are zero (invalid/unconfigured range)
    // This prevents undefined behavior when parameter or range is not properly configured.
    if ((g.rc_tuning_param <= 0) || (is_zero(g2.tuning_min.get()) && is_zero(g2.tuning_max.get()))) {
        return;
    }

    // **Safety Check 3: Verify valid RC signal (failsafe protection)**
    // Exit immediately if:
    //   - RC failsafe is active (!rc().has_valid_input())
    //   - Transmitter has not sent any signal (get_radio_in() == 0)
    // This prevents parameter changes during signal loss, which could occur at
    // unpredictable times and cause dangerous flight behavior. Tuning only proceeds
    // when pilot has positive control of the aircraft.
    if (!rc().has_valid_input() || rc_tuning->get_radio_in() == 0) {
        return;
    }

    // **RC Input Normalization**
    // Get the tuning channel position as a normalized value from -1.0 to 1.0:
    //   -1.0 = knob at minimum position (stick fully down/left)
    //    0.0 = knob at center position
    //   +1.0 = knob at maximum position (stick fully up/right)
    // Uses norm_input_ignore_trim() to get raw knob position independent of trim settings,
    // since tuning should respond to absolute knob position, not trimmed input.
    const float control_in = rc_tuning->norm_input_ignore_trim();
    
    // **Value Scaling: Linear Interpolation from Knob Position to Parameter Value**
    // Maps normalized RC input (-1.0 to 1.0) to configured parameter range (tuning_min to tuning_max).
    // Formula: tuning_value = tuning_min + (control_in + 1) / 2 * (tuning_max - tuning_min)
    // Examples:
    //   - Knob at min (-1.0) -> tuning_value = tuning_min
    //   - Knob at center (0.0) -> tuning_value = (tuning_min + tuning_max) / 2
    //   - Knob at max (+1.0) -> tuning_value = tuning_max
    // This provides smooth, proportional control across the full knob travel.
    const float tuning_value = linear_interpolate(g2.tuning_min, g2.tuning_max, control_in, -1, 1);

#if HAL_LOGGING_ENABLED
    // **Logging: Record Tuning Adjustment**
    // Log every parameter change with TUNE message containing:
    //   - Parameter ID (g.rc_tuning_param): Which parameter is being tuned
    //   - Current value (tuning_value): The interpolated value being applied
    //   - Min/max bounds (g2.tuning_min/max): Configured range for reference
    // This logging enables post-flight analysis of parameter effects on flight characteristics
    // and helps identify optimal values by correlating parameter changes with vehicle behavior.
    Log_Write_Parameter_Tuning(g.rc_tuning_param, tuning_value, g2.tuning_min, g2.tuning_max);
#endif

    // **Parameter Application: Switch on selected tuning parameter**
    // Each case applies the calculated tuning_value to the appropriate flight controller parameter.
    // Parameters are applied directly to active controllers (not saved to EEPROM during flight).
    // Grouped by functional area: attitude, rate, altitude, position, navigation, and vehicle-specific.
    switch(g.rc_tuning_param) {

    // ========================================================================
    // ATTITUDE CONTROL: Roll and Pitch Angle Tuning
    // ========================================================================
    // These parameters control the angle (stabilize) controller that converts
    // desired attitude angles to rate commands. Higher P gains increase the
    // aggressiveness of attitude corrections but may cause overshoots or oscillations.
    
    case TUNING_STABILIZE_ROLL_PITCH_KP:
        // Simultaneously adjusts roll and pitch angle controller P gains.
        // Affects how aggressively the vehicle corrects attitude errors in stabilized modes.
        // Typical range: 3.0 to 12.0 (depends on vehicle size and responsiveness desired)
        attitude_control->get_angle_roll_p().set_kP(tuning_value);
        attitude_control->get_angle_pitch_p().set_kP(tuning_value);
        break;

    // ========================================================================
    // RATE CONTROL: Roll and Pitch Rate PID Tuning (Combined)
    // ========================================================================
    // These parameters control the rate controller PID gains that convert desired
    // rotation rates (deg/s) to motor outputs. The rate controller is the innermost
    // control loop and critically affects flight stability and responsiveness.
    // These combined tuning options adjust both roll and pitch axes simultaneously.
    
    case TUNING_RATE_ROLL_PITCH_KP:
        // Proportional gain: Provides immediate response to rate errors.
        // Higher P increases responsiveness but may cause high-frequency oscillations.
        // Typical range: 0.08 to 0.25 (larger vehicles use lower values)
        attitude_control->get_rate_roll_pid().set_kP(tuning_value);
        attitude_control->get_rate_pitch_pid().set_kP(tuning_value);
        break;

    case TUNING_RATE_ROLL_PITCH_KI:
        // Integral gain: Eliminates steady-state rate errors and compensates for disturbances.
        // Too high causes low-frequency oscillations and overshoot.
        // Typical range: 0.08 to 0.25 (often set equal to P gain)
        attitude_control->get_rate_roll_pid().set_kI(tuning_value);
        attitude_control->get_rate_pitch_pid().set_kI(tuning_value);
        break;

    case TUNING_RATE_ROLL_PITCH_KD:
        // Derivative gain: Dampens oscillations and improves stability.
        // Higher D reduces overshoot but increases sensitivity to noise.
        // Typical range: 0.001 to 0.006 (usually 10-30x smaller than P)
        attitude_control->get_rate_roll_pid().set_kD(tuning_value);
        attitude_control->get_rate_pitch_pid().set_kD(tuning_value);
        break;

    // ========================================================================
    // YAW CONTROL: Yaw Angle and Rate Tuning
    // ========================================================================
    // Yaw control is typically more challenging than roll/pitch due to motor
    // coupling effects and slower yaw response. Yaw parameters are usually
    // tuned separately from roll/pitch.
    
    case TUNING_STABILIZE_YAW_KP:
        // Yaw angle controller P gain: Converts heading errors to yaw rate commands.
        // Typical range: 3.0 to 6.0 (lower than roll/pitch due to yaw dynamics)
        attitude_control->get_angle_yaw_p().set_kP(tuning_value);
        break;

    case TUNING_YAW_RATE_KP:
        // Yaw rate controller P gain: Primary control of yaw rotation rate.
        // Higher values increase yaw responsiveness but may cause yaw oscillations.
        // Typical range: 0.5 to 1.0 (much higher than roll/pitch due to yaw inertia)
        attitude_control->get_rate_yaw_pid().set_kP(tuning_value);
        break;

    case TUNING_YAW_RATE_KD:
        // Yaw rate controller D gain: Dampens yaw oscillations.
        // Note: Yaw rate controller typically does not use I gain (handled by angle controller).
        // Typical range: 0.0 to 0.02
        attitude_control->get_rate_yaw_pid().set_kD(tuning_value);
        break;

    // ========================================================================
    // ALTITUDE CONTROL: Vertical Position and Velocity Tuning
    // ========================================================================
    // Altitude control uses a cascaded PID structure:
    //   Position P -> Velocity PI -> Acceleration PID
    // These parameters control vertical position hold, climb/descent rates,
    // and throttle response in altitude-holding flight modes.
    
    case TUNING_ALTITUDE_HOLD_KP:
        // Altitude position controller P gain: Converts altitude errors to climb rate commands.
        // Higher values make altitude corrections more aggressive.
        // Note: 'U' denotes Up axis in NED coordinate frame (vertical position/velocity).
        // Typical range: 1.0 to 3.0
        pos_control->get_pos_U_p().set_kP(tuning_value);
        break;

    case TUNING_THROTTLE_RATE_KP:
        // Vertical velocity controller P gain: Converts climb rate errors to acceleration commands.
        // Affects how quickly vehicle responds to desired climb/descent rates.
        // Typical range: 5.0 to 15.0
        pos_control->get_vel_U_pid().set_kP(tuning_value);
        break;

    case TUNING_ACCEL_Z_KP:
        // Vertical acceleration controller P gain: Converts acceleration errors to throttle.
        // This is the innermost altitude control loop affecting throttle response.
        // Typical range: 0.3 to 1.0
        pos_control->get_accel_U_pid().set_kP(tuning_value);
        break;

    case TUNING_ACCEL_Z_KI:
        // Vertical acceleration controller I gain: Compensates for hover throttle variations.
        // Critical for maintaining altitude against thrust changes (battery sag, weight changes).
        // Typical range: 0.5 to 1.5 (often 2x the P gain)
        pos_control->get_accel_U_pid().set_kI(tuning_value);
        break;

    case TUNING_ACCEL_Z_KD:
        // Vertical acceleration controller D gain: Dampens vertical oscillations.
        // Helps smooth throttle response and reduce bouncing in altitude hold.
        // Typical range: 0.0 to 0.5
        pos_control->get_accel_U_pid().set_kD(tuning_value);
        break;

    // ========================================================================
    // HORIZONTAL POSITION CONTROL: Loiter and Navigation Tuning
    // ========================================================================
    // Horizontal position control uses similar cascaded structure to altitude:
    //   Position P -> Velocity PI
    // Controls position hold (loiter), waypoint navigation, and GPS-based modes.
    // Note: 'NE' denotes North-East plane in NED coordinate frame (horizontal position/velocity).
    
    case TUNING_LOITER_POSITION_KP:
        // Horizontal position controller P gain: Converts position errors to velocity commands.
        // Affects how aggressively vehicle returns to target position in loiter and position hold.
        // Higher values cause faster position corrections but may cause overshoots.
        // Typical range: 0.5 to 2.0
        pos_control->get_pos_NE_p().set_kP(tuning_value);
        break;

    case TUNING_VEL_XY_KP:
        // Horizontal velocity controller P gain: Converts velocity errors to lean angle commands.
        // Affects responsiveness of velocity control in GPS-based modes.
        // Typical range: 1.0 to 5.0
        pos_control->get_vel_NE_pid().set_kP(tuning_value);
        break;

    case TUNING_VEL_XY_KI:
        // Horizontal velocity controller I gain: Compensates for wind and drift.
        // Critical for maintaining position in windy conditions.
        // Too high may cause slow oscillations or "toilet bowling" effect.
        // Typical range: 0.5 to 2.0
        pos_control->get_vel_NE_pid().set_kI(tuning_value);
        break;

    case TUNING_WP_SPEED:
        // Waypoint navigation horizontal speed: Maximum speed during auto missions (cm/s).
        // Affects how fast vehicle flies between waypoints in Auto, Guided, and RTL modes.
        // Higher speeds reduce mission time but may reduce position accuracy.
        // Typical range: 500 to 2000 cm/s (5 to 20 m/s)
        wp_nav->set_speed_NE_cms(tuning_value);
        break;

#if MODE_ACRO_ENABLED || MODE_SPORT_ENABLED
    // ========================================================================
    // ACRO MODE: Manual Rate Control Limits
    // ========================================================================
    // Acro mode allows direct control of rotation rates without stabilization.
    // These parameters set the maximum rotation rates achievable with full stick deflection.
    
    case TUNING_ACRO_RP_RATE:
        // Acro mode roll and pitch rate limit (deg/s): Maximum rotation rate with full stick input.
        // Higher rates allow more aggressive aerobatic maneuvers but require more skill.
        // Typical range: 180 to 720 deg/s (1/2 to 2 rotations per second)
        g2.command_model_acro_rp.set_rate(tuning_value);
        break;
#endif

#if MODE_ACRO_ENABLED || MODE_DRIFT_ENABLED
    case TUNING_ACRO_YAW_RATE:
        // Acro mode yaw rate limit (deg/s): Maximum yaw rotation rate with full stick input.
        // Often set lower than roll/pitch rates due to yaw response limitations.
        // Typical range: 90 to 360 deg/s
        g2.command_model_acro_y.set_rate(tuning_value);
        break;
#endif

#if FRAME_CONFIG == HELI_FRAME
    // ========================================================================
    // HELICOPTER-SPECIFIC: Traditional Helicopter Parameters
    // ========================================================================
    // These tuning options are only available for traditional helicopters (HELI_FRAME).
    // Helicopters have different dynamics and control mechanisms than multirotors.
    
    case TUNING_HELI_EXTERNAL_GYRO:
        // External gyro gain: Adjusts gain signal sent to external tail gyro.
        // Used with traditional helicopters that have external heading hold gyros.
        // Affects tail authority and heading hold performance.
        // Typical range: 0 to 1000 (depends on specific gyro and setup)
        motors->ext_gyro_gain(tuning_value);
        break;

    case TUNING_RATE_PITCH_FF:
        // Pitch rate feedforward: Anticipatory control based on desired pitch rate.
        // Improves tracking of rapid pitch rate commands without waiting for error to develop.
        // Typical range: 0.0 to 0.5
        attitude_control->get_rate_pitch_pid().set_ff(tuning_value);
        break;

    case TUNING_RATE_ROLL_FF:
        // Roll rate feedforward: Anticipatory control based on desired roll rate.
        // Improves tracking of rapid roll rate commands without waiting for error to develop.
        // Typical range: 0.0 to 0.5
        attitude_control->get_rate_roll_pid().set_ff(tuning_value);
        break;

    case TUNING_RATE_YAW_FF:
        // Yaw rate feedforward: Anticipatory control based on desired yaw rate.
        // Improves yaw response and reduces lag in heading changes.
        // Typical range: 0.0 to 0.5
        attitude_control->get_rate_yaw_pid().set_ff(tuning_value);
        break;
#endif

    // ========================================================================
    // MISCELLANEOUS PARAMETERS
    // ========================================================================
    
    case TUNING_DECLINATION:
        // Compass declination: Angle between magnetic north and true north (degrees).
        // Positive values for easterly declination, negative for westerly.
        // Allows in-flight correction of compass declination errors affecting heading.
        // Note: Not saved to EEPROM during tuning to avoid flash wear and performance impact.
        // Pilot must manually save if desired value found.
        // Typical range: -30 to +30 degrees (depends on geographic location)
        compass.set_declination(radians(tuning_value), false);     // 2nd parameter is false because we do not want to save to eeprom because this would have a performance impact
        break;

#if MODE_CIRCLE_ENABLED
    case TUNING_CIRCLE_RATE:
        // Circle mode rotation rate (deg/s): Speed of rotation around the circle center.
        // Positive values for clockwise, negative for counter-clockwise.
        // Affects how fast vehicle orbits the target point in Circle mode.
        // Typical range: -90 to +90 deg/s
        circle_nav->set_rate_degs(tuning_value);
        break;
#endif

    case TUNING_RC_FEEL_RP:
        // RC feel (input shaping time constant): Controls responsiveness of stick inputs (seconds).
        // Lower values (e.g., 0.15) = more responsive, more aggressive
        // Higher values (e.g., 0.50) = smoother, less responsive, easier for beginners
        // Affects the input shaping filter that smooths pilot stick commands.
        // Typical range: 0.15 to 0.50 seconds
        attitude_control->set_input_tc(tuning_value);
        break;

    // ========================================================================
    // INDIVIDUAL AXIS RATE CONTROL: Separate Roll and Pitch Tuning
    // ========================================================================
    // These parameters allow independent tuning of roll and pitch rate controllers.
    // Use when roll and pitch axes have different dynamics (e.g., asymmetric frames,
    // different propeller configurations, or unusual mass distributions).
    
    case TUNING_RATE_PITCH_KP:
        // Pitch-only rate controller P gain: Independent pitch axis tuning.
        // Use when pitch dynamics differ from roll (e.g., asymmetric frame or payload).
        // Typical range: 0.08 to 0.25
        attitude_control->get_rate_pitch_pid().set_kP(tuning_value);
        break;

    case TUNING_RATE_PITCH_KI:
        // Pitch-only rate controller I gain: Independent pitch axis tuning.
        // Typical range: 0.08 to 0.25
        attitude_control->get_rate_pitch_pid().set_kI(tuning_value);
        break;

    case TUNING_RATE_PITCH_KD:
        // Pitch-only rate controller D gain: Independent pitch axis tuning.
        // Typical range: 0.001 to 0.006
        attitude_control->get_rate_pitch_pid().set_kD(tuning_value);
        break;

    case TUNING_RATE_ROLL_KP:
        // Roll-only rate controller P gain: Independent roll axis tuning.
        // Use when roll dynamics differ from pitch (unusual but possible).
        // Typical range: 0.08 to 0.25
        attitude_control->get_rate_roll_pid().set_kP(tuning_value);
        break;

    case TUNING_RATE_ROLL_KI:
        // Roll-only rate controller I gain: Independent roll axis tuning.
        // Typical range: 0.08 to 0.25
        attitude_control->get_rate_roll_pid().set_kI(tuning_value);
        break;

    case TUNING_RATE_ROLL_KD:
        // Roll-only rate controller D gain: Independent roll axis tuning.
        // Typical range: 0.001 to 0.006
        attitude_control->get_rate_roll_pid().set_kD(tuning_value);
        break;

#if FRAME_CONFIG != HELI_FRAME
    case TUNING_RATE_MOT_YAW_HEADROOM:
        // Motor yaw headroom: Percentage of motor output reserved for yaw control (0-500).
        // Higher values improve yaw authority but reduce maximum available thrust.
        // Too low may cause yaw control loss during aggressive maneuvers.
        // Typical range: 0 to 200 (0% to 200% of default headroom)
        // Only applicable to multirotor frames (not helicopters).
        motors->set_yaw_headroom(tuning_value);
        break;
#endif

    case TUNING_RATE_YAW_FILT:
        // Yaw rate error filter cutoff frequency (Hz): Low-pass filter on yaw rate error.
        // Higher frequencies allow faster yaw response but increase sensitivity to noise.
        // Lower frequencies smooth yaw response but may cause lag.
        // Typical range: 5 to 30 Hz
        attitude_control->get_rate_yaw_pid().set_filt_E_hz(tuning_value);
        break;

    // ========================================================================
    // ADVANCED FEATURES
    // ========================================================================
    
    case TUNING_SYSTEM_ID_MAGNITUDE:
        // System identification test magnitude: Amplitude of excitation signal for frequency response testing.
        // Used in SystemID flight mode to characterize vehicle dynamics for advanced tuning.
        // Higher magnitudes provide clearer frequency response but may affect stability.
        // Only active when SystemID mode is enabled and in use.
        // Typical range: 0.05 to 0.30 (vehicle-dependent)
#if MODE_SYSTEMID_ENABLED
        copter.mode_systemid.set_magnitude(tuning_value);
#endif
        break;

    case TUNING_POS_CONTROL_ANGLE_MAX:
        // Position control maximum lean angle (degrees): Limits lean angle during position corrections.
        // Lower values result in gentler, slower position corrections (safer, more conservative).
        // Higher values allow faster position corrections (more aggressive, higher risk).
        // Affects loiter, position hold, auto modes, and GPS-based navigation.
        // Typical range: 10 to 45 degrees (default usually 45 degrees)
        pos_control->set_lean_angle_max_deg(tuning_value);
        break;

    case TUNING_LOITER_MAX_XY_SPEED:
        // Loiter mode maximum horizontal speed (cm/s): Speed limit during pilot input in loiter.
        // Controls how fast pilot can move the vehicle while in loiter mode using stick inputs.
        // Lower values provide more precise positioning control.
        // Higher values allow faster repositioning but may reduce position accuracy.
        // Typical range: 500 to 2500 cm/s (5 to 25 m/s)
        loiter_nav->set_speed_max_NE_cms(tuning_value);
        break;
    }
    
    // End of tuning parameter switch statement.
    // All parameter changes have been applied and logged.
    // Changes take effect immediately but are not saved to EEPROM.
}
