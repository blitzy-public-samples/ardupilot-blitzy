/**
 * @file VTOL_Assist.cpp
 * @brief VTOL assistance system for fixed-wing flight support in quadplanes
 * 
 * @details This module provides automatic VTOL (Vertical Take-Off and Landing) motor
 *          engagement to assist fixed-wing flight during dangerous or marginal conditions.
 *          The assist system monitors multiple parameters and automatically activates
 *          multicopter motors to prevent stalls, attitude loss, and other hazardous
 *          flight conditions.
 * 
 *          Key assist triggers:
 *          - Airspeed below threshold (Q_ASSIST_SPEED) - prevents stalls
 *          - Altitude too low (Q_ASSIST_ALT) - prevents ground collision
 *          - Attitude error exceeds limits (Q_ASSIST_ANGLE) - prevents loss of control
 *          - Dangerous attitudes beyond 2x angle limits - triggers recovery mode
 *          - Spin detection - applies automatic spin recovery
 * 
 *          The system uses hysteresis to prevent assist mode oscillation, requiring
 *          conditions to persist for Q_ASSIST_DELAY seconds before activating, and
 *          continuing for 2x that duration after conditions clear.
 * 
 *          VTOL Recovery Mode:
 *          When attitude exceeds 2x Q_ANGLE_MAX, the system enters forced fixed-wing
 *          control recovery, using VTOL motors to bring the aircraft back within normal
 *          attitude limits. If a spin is detected (opposing roll/yaw rates with high
 *          rotation rates and nose-down attitude), spin recovery procedures are applied.
 * 
 * @note This system is safety-critical and directly affects vehicle stability
 * @warning Incorrect parameter tuning can lead to unexpected motor activation or
 *          delayed assistance in emergency situations
 * 
 * @see QuadPlane class for VTOL motor control integration
 * @see should_assist() for assist trigger logic
 * @see check_VTOL_recovery() for recovery mode monitoring
 * 
 * Source: ArduPlane/VTOL_Assist.cpp
 */

#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

// Assistance hysteresis helpers

/**
 * @brief Reset hysteresis state to inactive
 * 
 * @details Clears all timing and state variables, returning the hysteresis
 *          tracker to its initial inactive state. Called when assist conditions
 *          are no longer met or when the system is reset.
 * 
 * @note This prevents assist oscillation by requiring full re-triggering
 */
void VTOL_Assist::Assist_Hysteresis::reset()
{
    start_ms = 0;
    last_ms = 0;
    active = false;
}

/**
 * @brief Update hysteresis state with current trigger condition
 * 
 * @details Implements hysteresis timing for assist triggers to prevent mode oscillation.
 *          The trigger condition must remain true for trigger_delay_ms before becoming
 *          active. Once active, the condition must be false for clear_delay_ms before
 *          deactivating. This provides stability in marginal flight conditions.
 * 
 *          State machine:
 *          1. Trigger asserted -> start timing
 *          2. Trigger held for trigger_delay_ms -> activate (return true once)
 *          3. Trigger released while active -> start clear timer
 *          4. Clear delay expires -> reset to inactive
 * 
 * @param[in] trigger Current trigger condition (true = assist needed)
 * @param[in] now_ms Current system time in milliseconds
 * @param[in] trigger_delay_ms Time in ms trigger must persist before activating
 * @param[in] clear_delay_ms Time in ms trigger must be clear before deactivating
 * 
 * @return true on first activation (for logging/notification), false otherwise
 * 
 * @note Called at main loop rate (typically 400Hz for Plane)
 */
bool VTOL_Assist::Assist_Hysteresis::update(const bool trigger, const uint32_t &now_ms, const uint32_t &trigger_delay_ms, const uint32_t &clear_delay_ms)
{
    bool ret = false;

    if (trigger) {
        last_ms = now_ms;
        if (start_ms == 0) {
            start_ms = now_ms;
        }
        if ((now_ms - start_ms) > trigger_delay_ms) {
            // trigger delay has elapsed
            if (!active) {
                // return true on first trigger
                ret = true;
            }
            active = true;
        }

    } else if (active) {
        if ((last_ms == 0) || ((now_ms - last_ms) > clear_delay_ms)) {
            // Clear delay passed
            reset();
        }

    } else {
        reset();
    }

    return ret;
}

/**
 * @brief Reset all VTOL assistance state when assistance is not needed
 * 
 * @details Clears all assist triggers and hysteresis timers. Called when the vehicle
 *          is disarmed, assist is disabled, or conditions don't warrant monitoring.
 *          This ensures a clean state when re-entering conditions where assist may
 *          be needed.
 * 
 * @note Called automatically when assist system is inactive
 */
void VTOL_Assist::reset()
{
    force_assist = false;
    speed_assist = false;
    angle_error.reset();
    alt_error.reset();
}

/**
 * @brief Determine if VTOL motors should provide automatic assistance to fixed-wing flight
 * 
 * @details This is the primary decision function for automatic VTOL motor engagement during
 *          fixed-wing flight. It monitors multiple flight parameters and activates VTOL
 *          assistance when dangerous or marginal conditions are detected.
 * 
 *          Automatic VTOL motor engagement is triggered by:
 * 
 *          1. **Speed Assist** (Q_ASSIST_SPEED):
 *             - Airspeed drops below configured threshold
 *             - Prevents fixed-wing stalls by adding multicopter thrust
 *             - Only uses real airspeed sensor if DISABLE_SYNTHETIC_AIRSPEED_ASSIST option set
 * 
 *          2. **Altitude Assist** (Q_ASSIST_ALT):
 *             - Height above ground falls below threshold
 *             - Prevents ground collision during low-altitude flight
 *             - Uses rangefinder when available for accuracy
 * 
 *          3. **Angle Assist** (Q_ASSIST_ANGLE):
 *             - Monitors attitude error and envelope limits
 *             - Triggers if outside configured roll/pitch limits (ROLL_LIMIT, PTCH_LIM_MAX/MIN)
 *             - Also triggers if attitude error relative to desired attitude exceeds threshold
 *             - Provides 5° envelope error margin before triggering
 *             - **This is critical for preventing loss of control in unusual attitudes**
 * 
 *          4. **Force Enabled** (Q_ASSIST_STATE = 2):
 *             - Manual override to force assistance active
 *             - Bypasses all other checks
 * 
 *          Safety Interlocks:
 *          - Disabled when disarmed or safety not off
 *          - Disabled in flare mode (landing phase)
 *          - Disabled for control surface tailsitters (different control model)
 *          - Requires throttle active or flight mode with auto throttle
 * 
 *          Hysteresis Behavior:
 *          - All triggers use Q_ASSIST_DELAY second delay before activating
 *          - Once active, continues for 2x delay period after conditions clear
 *          - Prevents rapid oscillation between assist and non-assist modes
 * 
 * @param[in] aspeed Current airspeed in m/s (synthetic or measured)
 * @param[in] have_airspeed True if airspeed reading is available
 * 
 * @return true if VTOL assistance should be active, false otherwise
 * 
 * @note Called at main loop rate (typically 50Hz for fixed-wing)
 * @warning This function directly controls automatic VTOL motor activation in flight
 * @warning Incorrect parameter settings can cause unexpected motor engagement or
 *          delayed assistance in stall conditions
 * 
 * @see Q_ASSIST_SPEED parameter for airspeed threshold
 * @see Q_ASSIST_ALT parameter for altitude threshold  
 * @see Q_ASSIST_ANGLE parameter for attitude error threshold
 * @see Q_ASSIST_DELAY parameter for hysteresis timing
 * 
 * Source: ArduPlane/VTOL_Assist.cpp:59-144
 */
bool VTOL_Assist::should_assist(float aspeed, bool have_airspeed)
{
    if (!plane.arming.is_armed_and_safety_off() || (state == STATE::ASSIST_DISABLED) || quadplane.tailsitter.is_control_surface_tailsitter()) {
        // disarmed or disabled by aux switch or because a control surface tailsitter
        reset();
        return false;
    }

    if (!quadplane.tailsitter.enabled() && !( (plane.control_mode->does_auto_throttle() && !plane.throttle_suppressed)
                                                                      || is_positive(plane.get_throttle_input()) 
                                                                      || plane.is_flying() ) ) {
        // not in a flight mode and condition where it would be safe to turn on vertical lift motors
        // skip this check for tailsitters because the forward and vertical motors are the same and are controlled directly by throttle input unlike other quadplanes
        reset();
        return false;
    }

    if (plane.flare_mode != Plane::FlareMode::FLARE_DISABLED) {
        // Never active in fixed wing flare
        reset();
        return false;
    }

    force_assist = state == STATE::FORCE_ENABLED;

    if (speed <= 0) {
        // all checks disabled via speed threshold, still allow force enabled
        speed_assist = false;
        alt_error.reset();
        angle_error.reset();
        return force_assist;
    }

    // assistance due to Q_ASSIST_SPEED
    // if option bit is enabled only allow assist with real airspeed sensor
    speed_assist = (have_airspeed && aspeed < speed) && 
       (!quadplane.option_is_set(QuadPlane::OPTION::DISABLE_SYNTHETIC_AIRSPEED_ASSIST) || plane.ahrs.using_airspeed_sensor());

    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t tigger_delay_ms = delay * 1000;
    const uint32_t clear_delay_ms = tigger_delay_ms * 2;

    /*
      optional assistance when altitude is too close to the ground
     */
    if (alt <= 0) {
        // Alt assist disabled
        alt_error.reset();

    } else {
        const float height_above_ground = plane.relative_ground_altitude(RangeFinderUse::ASSIST);
        if (alt_error.update(height_above_ground < alt, now_ms, tigger_delay_ms, clear_delay_ms)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Alt assist %.1fm", height_above_ground);
        }
    }

    if (angle <= 0) {
        // Angle assist disabled
        angle_error.reset();

    } else {

        /*
        now check if we should provide assistance due to attitude error
        */
        const auto ahrs_roll_deg = plane.ahrs.get_roll_deg();
        const auto ahrs_pitch_deg = plane.ahrs.get_pitch_deg();
        constexpr float allowed_envelope_error_deg = 5.0;
        const bool inside_envelope =
            (fabsf(ahrs_roll_deg) <= (plane.aparm.roll_limit + allowed_envelope_error_deg)) &&
            (ahrs_pitch_deg < (plane.aparm.pitch_limit_max + allowed_envelope_error_deg)) &&
            (ahrs_pitch_deg > (plane.aparm.pitch_limit_min - allowed_envelope_error_deg));

        const bool inside_angle_error =
            (fabsf(ahrs_roll_deg - plane.nav_roll_cd*0.01) < angle) &&
            (fabsf(ahrs_pitch_deg - plane.nav_pitch_cd*0.01) < angle);

        if (angle_error.update(!inside_envelope && !inside_angle_error, now_ms, tigger_delay_ms, clear_delay_ms)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Angle assist r=%d p=%d",
                            (int)ahrs_roll_deg,
                            (int)ahrs_pitch_deg);
        }
    }

    return force_assist || speed_assist || alt_error.is_active() || angle_error.is_active();
}

/**
 * @brief Check for dangerous attitudes and engage VTOL recovery mode
 * 
 * @details Monitors vehicle attitude for dangerously large deviations that indicate loss
 *          of fixed-wing control authority. When detected, forces VTOL motor engagement
 *          to recover the vehicle to safe attitudes using multicopter stabilization.
 * 
 *          **Dangerous Attitude Detection:**
 *          - Calculates combined roll/pitch angle magnitude from AHRS
 *          - Triggers recovery if attitude exceeds 2x Q_ANGLE_MAX
 *          - Example: If Q_ANGLE_MAX = 30°, recovery triggers at 60° deviation
 *          - Once triggered, maintains recovery until within 1x Q_ANGLE_MAX
 * 
 *          **Recovery Mode Behavior:**
 *          - Sets quadplane.force_fw_control_recovery flag
 *          - VTOL motors provide stabilization to reduce attitude error
 *          - Exits recovery when attitude returns to within Q_ANGLE_MAX limits
 *          - Resets attitude, position, and height controllers on recovery exit
 *          - Position/height controller reset skipped if groundspeed is low
 * 
 *          **Spin Detection and Recovery:**
 *          - Additionally monitors for spin condition during recovery
 *          - Spin detected when:
 *            * Yaw rate > 10°/s
 *            * Roll rate > 30°/s  
 *            * Pitch rate > 30°/s
 *            * Roll and yaw rates have opposite sign (characteristic of spin)
 *            * Pitch attitude < -45° (nose down)
 *          - Sets quadplane.in_spin_recovery flag when spin detected
 *          - Spin recovery applies specific control surface commands (see output_spin_recovery)
 * 
 *          Safety Interlocks:
 *          - Disabled if Q_OPTIONS FW_FORCE_DISABLED bit set
 *          - Disabled for tailsitters (use different control model)
 *          - Disabled in QACRO mode (manual VTOL control)
 *          - Spin recovery disabled if Q_OPTIONS SPIN_DISABLED bit set
 * 
 * @return true if vehicle is in VTOL recovery mode, false otherwise
 * 
 * @note Called at main loop rate during fixed-wing flight
 * @warning This is a safety-critical function for preventing loss of control
 * @warning Recovery engages VTOL motors automatically even in fixed-wing modes
 * @warning Spin detection thresholds are tuned for typical aircraft - may need
 *          adjustment for unusual configurations
 * 
 * @see output_spin_recovery() for spin recovery control outputs
 * @see Q_ANGLE_MAX parameter for maximum attitude limits
 * @see Q_OPTIONS parameter bits FW_FORCE_DISABLED and SPIN_DISABLED
 * 
 * Source: ArduPlane/VTOL_Assist.cpp:149-206
 */
bool VTOL_Assist::check_VTOL_recovery(void)
{
    const bool allow_fw_recovery =
        !option_is_set(OPTION::FW_FORCE_DISABLED) &&
        !quadplane.tailsitter.enabled() &&
        plane.control_mode != &plane.mode_qacro;
    if (!allow_fw_recovery) {
        quadplane.force_fw_control_recovery = false;
        quadplane.in_spin_recovery = false;
        return false;
    }

    // see if the attitude is outside twice the Q_ANGLE_MAX
    const auto &ahrs = plane.ahrs;
    const int16_t angle_max_cd = quadplane.aparm.angle_max;
    const float abs_angle_cd = fabsf(Vector2f{float(ahrs.roll_sensor), float(ahrs.pitch_sensor)}.length());

    if (abs_angle_cd > 2*angle_max_cd) {
        // we are 2x the angle limits, trigger fw recovery
        quadplane.force_fw_control_recovery = true;
    }

    if (quadplane.force_fw_control_recovery) {
        // stop fixed wing recovery if inside Q_ANGLE_MAX
        if (abs_angle_cd <= angle_max_cd) {
            quadplane.force_fw_control_recovery = false;
            quadplane.attitude_control->reset_target_and_rate(false);

            if (ahrs.groundspeed() > quadplane.wp_nav->get_default_speed_NE_cms()*0.01) {
                /* if moving at high speed also reset position
                   controller and height controller

                   this avoids an issue where the position
                   controller may limit pitch after a strong
                   acceleration event
                */
                quadplane.pos_control->init_U_controller();
                quadplane.pos_control->init_NE_controller();
            }
        }
    }

    if (!option_is_set(OPTION::SPIN_DISABLED) &&
        quadplane.force_fw_control_recovery) {
        // additionally check for needing spin recovery
        const auto &gyro = plane.ahrs.get_gyro();
        quadplane.in_spin_recovery =
            fabsf(gyro.z) > radians(10) &&
            fabsf(gyro.x) > radians(30) &&
            fabsf(gyro.y) > radians(30) &&
            gyro.x * gyro.z < 0 &&
            plane.ahrs.get_pitch_deg() < -45;
    } else {
        quadplane.in_spin_recovery = false;
    }
    
    return quadplane.force_fw_control_recovery;
}


/**
 * @brief Apply control surface commands for automatic spin recovery
 * 
 * @details When a spin condition is detected (quadplane.in_spin_recovery = true),
 *          this function overrides normal control surface commands to apply standard
 *          spin recovery procedures using the rudder and elevator.
 * 
 *          **Spin Recovery Technique:**
 *          - Applies full opposite rudder to counter the yaw rotation
 *          - Rudder direction based on yaw rate sign (gyro.z):
 *            * Positive yaw rate -> full left rudder (-SERVO_MAX)
 *            * Negative yaw rate -> full right rudder (+SERVO_MAX)
 *          - Sets elevator to neutral (0) to reduce angle of attack
 *          - VTOL motors provide thrust to aid recovery
 * 
 *          **Operation:**
 *          - Only active when quadplane.in_spin_recovery flag is set
 *          - Only active while VTOL motors are spooled up (THROTTLE_UNLIMITED state)
 *          - Directly overrides SRV_Channel outputs for rudder and elevator
 *          - Continues until spin condition clears (see check_VTOL_recovery)
 * 
 *          **Safety Checks:**
 *          - Automatically clears spin_recovery flag if VTOL motors shut down
 *          - Prevents application of recovery commands without motor thrust
 * 
 * @note This function is called after normal control surface mixing
 * @note Recovery commands override normal pilot/autopilot control inputs
 * 
 * @warning This is a safety-critical automatic recovery function
 * @warning Only active when both spin detected AND VTOL motors running
 * @warning Full rudder deflection applied - ensure rudder mechanically limited
 * 
 * @see check_VTOL_recovery() for spin detection logic
 * @see SRV_Channels::set_output_scaled() for servo output control
 * @see SERVO_MAX constant for maximum servo deflection value
 * 
 * Source: ArduPlane/VTOL_Assist.cpp:215-233
 */
void VTOL_Assist::output_spin_recovery(void)
{
    if (!quadplane.in_spin_recovery) {
        return;
    }
    if (quadplane.motors->get_desired_spool_state() !=
        AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED) {
        // if we and no longer running the VTOL motors we need to
        // clear the spin flag
        quadplane.in_spin_recovery = false;
        return;
    }
    const Vector3f &gyro = plane.ahrs.get_gyro();

    // put in opposite rudder to counter yaw, and neutral
    // elevator until we're out of the spin
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, gyro.z > 0 ? -SERVO_MAX : SERVO_MAX);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, 0);
}


#endif  // HAL_QUADPLANE_ENABLED
