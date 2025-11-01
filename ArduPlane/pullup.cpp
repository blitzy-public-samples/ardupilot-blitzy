#include "Plane.h"

/**
 * @file pullup.cpp
 * @brief Glider pullup maneuver support for high-altitude operations
 * 
 * @details This file implements the GliderPullup class which manages the pullup
 *          maneuver sequence after a NAV_ALTITUDE_WAIT mission command. This is
 *          specifically designed for high-altitude gliders (e.g., balloon-dropped
 *          gliders) that need controlled energy management during the transition
 *          from high-altitude descent to normal flight.
 *          
 *          The pullup sequence consists of multiple stages:
 *          - WAIT_AIRSPEED: Wait for airspeed to build after balloon release
 *          - WAIT_PITCH: Monitor pitch angle during initial pullup
 *          - PUSH_NOSE_DOWN: Emergency recovery if roll control is lost
 *          - WAIT_LEVEL: Wait for aircraft to stabilize before returning to normal flight
 *          
 *          Key safety features:
 *          - Load factor limiting (ng_limit) to prevent structural damage
 *          - Jerk limiting for smooth load factor transitions
 *          - Roll angle monitoring with automatic nose-down recovery
 *          - Airspeed prediction to prevent stall during pitch-up
 *          
 *          Energy Management:
 *          During high-altitude operations, gliders trade altitude for airspeed.
 *          This code manages the critical pullup phase where the aircraft must
 *          convert potential energy (altitude) into kinetic energy (airspeed)
 *          while maintaining safe load factors and preventing stall or loss of control.
 * 
 * @note This feature is conditionally compiled with AP_PLANE_GLIDER_PULLUP_ENABLED
 * @warning Incorrect parameter tuning can result in structural damage (excessive g-loads)
 *          or loss of control (stall during pullup)
 * 
 * @see NAV_ALTITUDE_WAIT mission command
 * @see GliderPullup class in pullup.h
 */

#if AP_PLANE_GLIDER_PULLUP_ENABLED

/**
 * @brief Parameter table for glider pullup configuration
 * 
 * @details This parameter group controls all aspects of the pullup maneuver including:
 *          - Enable/disable functionality
 *          - Initial elevator deflection during airspeed buildup
 *          - Load factor limits for structural safety
 *          - Load factor rate-of-change (jerk) limiting for smooth transitions
 *          - Target pitch angles for different phases
 *          - Airspeed thresholds for phase transitions
 *          
 *          Parameters are prefixed with PUP_ in the ground control station.
 *          All parameters are in the Advanced user category due to safety implications.
 * 
 * @note Parameter values must be tuned for specific aircraft and operational conditions
 * @warning Improper parameter values can lead to structural damage or loss of control
 */
const AP_Param::GroupInfo GliderPullup::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable pullup after altitude wait
    // @Description: Enable pullup after altitude wait
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, GliderPullup,  enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: ELEV_OFS
    // @DisplayName: Elevator deflection used before starting pullup
    // @Description: Elevator deflection offset from -1 to 1 while waiting for airspeed to rise before starting close loop control of the pullup.
    // @Range: -1.0 1.0
    // @User: Advanced
    AP_GROUPINFO("ELEV_OFS", 2, GliderPullup,  elev_offset, 0.1f),

    // @Param: NG_LIM
    // @DisplayName: Maximum normal load factor during pullup
    // @Description: This is the nominal maximum value of normal load factor used during the closed loop pitch rate control of the pullup.
    // @Range: 1.0 4.0
    // @User: Advanced
    AP_GROUPINFO("NG_LIM", 3, GliderPullup,  ng_limit, 2.0f),

    // @Param: NG_JERK_LIM
    // @DisplayName: Maximum normal load factor rate of change during pullup
    // @Description: The normal load factor used for closed loop pitch rate control of the pullup will be ramped up to the value set by PUP_NG_LIM at the rate of change set by this parameter. The parameter value specified will be scaled internally by 1/EAS2TAS.
    // @Units: 1/s
    // @Range: 0.1 10.0
    // @User: Advanced
    AP_GROUPINFO("NG_JERK_LIM", 4, GliderPullup,  ng_jerk_limit, 4.0f),

    // @Param: PITCH
    // @DisplayName: Target pitch angle during pullup
    // @Description: The vehicle will attempt achieve this pitch angle during the pull-up maneouvre.
    // @Units: deg
    // @Range: -5 15
    // @User: Advanced
    AP_GROUPINFO("PITCH", 5, GliderPullup,  pitch_dem, 3),

    // @Param: ARSPD_START
    // @DisplayName: Pullup target airspeed
    // @Description: Target airspeed for initial airspeed wait
    // @Units: m/s
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("ARSPD_START", 6, GliderPullup,  airspeed_start, 30),

    // @Param: PITCH_START
    // @DisplayName: Pullup target pitch
    // @Description: Target pitch for initial pullup
    // @Units: deg
    // @Range: -80 0
    // @User: Advanced
    AP_GROUPINFO("PITCH_START", 7, GliderPullup,  pitch_start, -60),
    
    AP_GROUPEND
};

/**
 * @brief Constructor for GliderPullup class
 * 
 * @details Initializes the glider pullup manager with default parameter values
 *          from the var_info parameter table. The constructor sets up the
 *          AP_Param system for persistent parameter storage and ground station
 *          parameter access.
 *          
 *          Default values are defined in the var_info array and can be overridden
 *          by values stored in EEPROM or set via MAVLink parameter protocol.
 * 
 * @note Called once during vehicle initialization
 */
GliderPullup::GliderPullup(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/**
 * @brief Check if vehicle is currently executing a pullup maneuver
 * 
 * @details Returns true if the vehicle is in AUTO mode, executing a
 *          NAV_ALTITUDE_WAIT mission command, and has progressed beyond
 *          the initial NONE stage into an active pullup sequence.
 *          
 *          This status check is used by other flight code to determine
 *          if special pullup control logic should be active instead of
 *          normal AUTO mode control.
 * 
 * @return true if actively performing pullup maneuver
 * @return false if not in pullup or pullup not started
 * 
 * @note This is a const method safe to call from any context
 * @see pullup_start() for pullup initiation
 * @see Stage enum for pullup stage definitions
 */
bool GliderPullup::in_pullup(void) const
{
    return plane.control_mode == &plane.mode_auto &&
        plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_ALTITUDE_WAIT &&
        stage != Stage::NONE;
}

/**
 * @brief Initiate the glider pullup maneuver sequence
 * 
 * @details Called when NAV_ALTITUDE_WAIT mission command has reached the target
 *          altitude or exceeded the maximum descent rate threshold. This method:
 *          
 *          1. Checks if pullup is enabled via PUP_ENABLE parameter
 *          2. Triggers balloon release servo (k_lift_release) for balloon-dropped gliders
 *          3. Transitions to WAIT_AIRSPEED stage to begin pullup sequence
 *          4. Disables idle mode to ensure full throttle/control authority
 *          5. Logs initial conditions (airspeed and altitude) to GCS
 *          
 *          Typical use case:
 *          High-altitude balloon carries glider to altitude, NAV_ALTITUDE_WAIT
 *          monitors descent, and when conditions are met, this initiates the
 *          pullup sequence with balloon release.
 * 
 * @return true if pullup successfully started
 * @return false if pullup is disabled via PUP_ENABLE parameter
 * 
 * @note Sends telemetry message to ground control station with initial state
 * @warning Balloon release servo is activated unconditionally if pullup is enabled
 * 
 * @see NAV_ALTITUDE_WAIT mission command documentation
 * @see SRV_Channel::k_lift_release for balloon release servo function
 */
bool GliderPullup::pullup_start(void)
{
    if (enable != 1) {
        return false;
    }

    // release balloon
    SRV_Channels::set_output_scaled(SRV_Channel::k_lift_release, 100);

    stage = Stage::WAIT_AIRSPEED;
    plane.auto_state.idle_mode = false;
    float aspeed;
    if (!plane.ahrs.airspeed_estimate(aspeed)) {
        aspeed = -1;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Start pullup airspeed %.1fm/s at %.1fm AMSL", aspeed, plane.current_loc.alt*0.01);
    return true;
}

/**
 * @brief Monitor and verify pullup maneuver progression through stages
 * 
 * @details This method implements the state machine for pullup maneuver execution.
 *          Called repeatedly during NAV_ALTITUDE_WAIT to check stage completion
 *          conditions and transition between pullup stages.
 *          
 *          Stage Progression:
 *          
 *          1. WAIT_AIRSPEED: Monitor airspeed buildup after balloon release
 *             - Transition when airspeed > PUP_ARSPD_START OR pitch > PUP_PITCH_START
 *             - Ensures adequate airflow before closed-loop control
 *          
 *          2. WAIT_PITCH: Monitor pitch angle during initial pullup
 *             - Transition when pitch > PUP_PITCH_START AND abs(roll) < 90 degrees
 *             - Confirms aircraft is pitching up and not inverted
 *          
 *          3. PUSH_NOSE_DOWN: Emergency recovery stage (entered from WAIT_LEVEL)
 *             - Transition when abs(roll) < aparm.roll_limit
 *             - Used if roll control is lost during final leveling
 *          
 *          4. WAIT_LEVEL: Final stabilization before returning to normal AUTO
 *             - Monitors pitch > pitch_limit_min, airspeed approaching target, roll control OK
 *             - Uses predictive airspeed calculation to prevent premature handoff
 *             - Can transition to PUSH_NOSE_DOWN if roll control lost
 *             - Completes when aircraft is stable and slow enough for speed controller
 *          
 *          Safety Logic:
 *          - Continuous roll angle monitoring with automatic nose-down recovery
 *          - Predictive airspeed calculation accounts for pitch lag time
 *          - Separate checks for pitch completion, airspeed, and roll control
 * 
 * @return true if pullup sequence is complete and can return to normal AUTO mode
 * @return false if pullup is still in progress
 * 
 * @note Called at main loop rate (typically 50Hz for fixed-wing)
 * @warning Roll angles exceeding roll_limit during WAIT_LEVEL trigger emergency nose-down
 * 
 * @see stabilize_pullup() for control surface commands during each stage
 */
bool GliderPullup::verify_pullup(void)
{
    const auto &ahrs = plane.ahrs;
    const auto &current_loc = plane.current_loc;
    const auto &aparm = plane.aparm;

    switch (stage) {
    case Stage::WAIT_AIRSPEED: {
        float aspeed;
        if (ahrs.airspeed_estimate(aspeed) && (aspeed > airspeed_start || ahrs.get_pitch_deg() > pitch_start)) {
            gcs().send_text(MAV_SEVERITY_INFO, "Pullup airspeed %.1fm/s alt %.1fm AMSL", aspeed, current_loc.alt*0.01);
            stage = Stage::WAIT_PITCH;
        }
        return false;
    }

    case Stage::WAIT_PITCH: {
        if (ahrs.get_pitch_deg() > pitch_start && fabsf(ahrs.get_roll_deg()) < 90) {
            gcs().send_text(MAV_SEVERITY_INFO, "Pullup pitch p=%.1f r=%.1f alt %.1fm AMSL",
                            ahrs.get_pitch_deg(),
                            ahrs.get_roll_deg(),
                            current_loc.alt*0.01);
            stage = Stage::WAIT_LEVEL;
        }
        return false;
    }

    case Stage::PUSH_NOSE_DOWN: {
        if (fabsf(ahrs.get_roll_deg()) < aparm.roll_limit) {
            stage = Stage::WAIT_LEVEL;
        }
        return false;
    }

    case Stage::WAIT_LEVEL: {
        // When pitch has raised past lower limit used by speed controller, wait for airspeed to approach
        // target value before handing over control of pitch demand to speed controller
        
        // Check if pitch angle is high enough for speed controller to take over
        bool pitchup_complete = ahrs.get_pitch_deg() > MIN(0, aparm.pitch_limit_min);
        
        // Calculate prediction time horizon - scales with air density (EAS2TAS)
        // At high altitude (low density), aircraft responds more slowly, so use longer prediction horizon
        const float pitch_lag_time = 1.0f * sqrtf(ahrs.get_EAS2TAS());
        
        float aspeed;
        // Calculate rate of change of airspeed in body frame
        // Uses x-axis acceleration plus gravity component in body x-axis
        // Divided by EAS2TAS to get equivalent airspeed derivative
        const float aspeed_derivative = (ahrs.get_accel().x + GRAVITY_MSS * ahrs.get_DCM_rotation_body_to_ned().c.x) / ahrs.get_EAS2TAS();
        
        // Predict future airspeed using current rate of change
        // This prevents premature handoff to speed controller when airspeed is still decreasing rapidly
        bool airspeed_low = ahrs.airspeed_estimate(aspeed) ? (aspeed + aspeed_derivative * pitch_lag_time) < 0.01f * (float)plane.target_airspeed_cm : true;
        
        // Safety check: ensure we still have roll authority
        bool roll_control_lost = fabsf(ahrs.get_roll_deg()) > aparm.roll_limit;
        if (pitchup_complete && airspeed_low && !roll_control_lost) {
                gcs().send_text(MAV_SEVERITY_INFO, "Pullup level r=%.1f p=%.1f alt %.1fm AMSL",
                                ahrs.get_roll_deg(), ahrs.get_pitch_deg(), current_loc.alt*0.01);
                break;
        } else if (pitchup_complete && roll_control_lost) {
                // push nose down and wait to get roll control back
                gcs().send_text(MAV_SEVERITY_ALERT, "Pullup level roll bad r=%.1f p=%.1f",
                                ahrs.get_roll_deg(),
                                ahrs.get_pitch_deg());
                stage = Stage::PUSH_NOSE_DOWN;
        }
        return false;
    }
    case Stage::NONE:
        break;
    }

    // all done
    stage = Stage::NONE;
    return true;
}

/**
 * @brief Generate control surface commands during pullup maneuver
 * 
 * @details Implements stage-specific control logic for the pullup sequence.
 *          This method is called from the main stabilization loop when in_pullup()
 *          returns true, replacing the normal AUTO mode stabilization logic.
 *          
 *          Control Strategy by Stage:
 *          
 *          WAIT_AIRSPEED:
 *          - Fixed elevator deflection (PUP_ELEV_OFS) to build airspeed
 *          - Zero rudder, zero aileron rate demand (wings level)
 *          - Resets pitch and yaw integrators to prevent windup
 *          - Initializes ng_demand to zero for smooth transition to next stage
 *          
 *          WAIT_PITCH:
 *          - Closed-loop load factor (ng) control for smooth pullup
 *          - Ramps ng_demand up to PUP_NG_LIM at rate PUP_NG_JERK_LIM
 *          - Converts ng to pitch rate demand: pitch_rate = (ng * g) / TAS
 *          - Blends elevator offset with pitch rate controller output
 *          - Maintains wings level with aileron rate controller
 *          - Jerk limit scaled by EAS2TAS for altitude compensation
 *          
 *          PUSH_NOSE_DOWN:
 *          - Emergency recovery mode if roll control lost during WAIT_LEVEL
 *          - Commands pitch_limit_min (most nose-down allowed)
 *          - Full attitude control (pitch, roll, yaw stabilization)
 *          - Resets ng_demand to zero
 *          
 *          WAIT_LEVEL:
 *          - Commands gentle positive pitch (pitch_limit_min + 5 or pitch_dem)
 *          - Full attitude stabilization while waiting for airspeed to stabilize
 *          - Resets ng_demand to zero
 *          
 *          Load Factor Control (WAIT_PITCH stage):
 *          The load factor (ng) represents how many "g's" the aircraft experiences.
 *          ng = 1.0 means 1g (straight and level flight)
 *          ng = 2.0 means 2g (pullup with 2x the weight force)
 *          
 *          The demanded pitch rate for a given load factor is:
 *          pitch_rate [rad/s] = (ng * g) / TAS
 *          where g = 9.81 m/s^2 and TAS = true airspeed
 *          
 *          Jerk limiting (ng_jerk_limit) ensures smooth transitions to prevent:
 *          - Structural stress from rapid load changes
 *          - Pilot/payload discomfort
 *          - Control surface saturation
 * 
 * @note Called from main stabilization loop, sets plane.last_stabilize_ms
 * @note Load factor calculations account for altitude effects via EAS2TAS scaling
 * 
 * @warning WAIT_PITCH stage can command high load factors - ensure PUP_NG_LIM is appropriate
 * @warning Fixed elevator offset in WAIT_AIRSPEED assumes aircraft is in clean configuration
 * 
 * @see verify_pullup() for stage transition logic
 * @see PID controllers in Plane class (pitchController, rollController, yawController)
 */
void GliderPullup::stabilize_pullup(void)
{
    const float speed_scaler = plane.get_speed_scaler();
    switch (stage) {
    case Stage::WAIT_AIRSPEED: {
        plane.pitchController.reset_I();
        plane.yawController.reset_I();
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, elev_offset*4500);
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, 0);
        plane.nav_pitch_cd = 0;
        plane.nav_roll_cd = 0;
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.rollController.get_rate_out(0, speed_scaler));
        ng_demand = 0.0;
        break;
    }
    case Stage::WAIT_PITCH: {
        plane.yawController.reset_I();
        plane.nav_roll_cd = 0;
        plane.nav_pitch_cd = 0;
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, 0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.rollController.get_rate_out(0, speed_scaler));
        float aspeed;
        const auto &ahrs = plane.ahrs;
        if (ahrs.airspeed_estimate(aspeed)) {
            // Apply jerk limiting: ramp ng_demand up gradually
            // Jerk limit is scaled by 1/EAS2TAS to account for reduced dynamic pressure at altitude
            // Minimum rate of 0.1 g/s prevents excessively slow pullup at very high altitudes
            ng_demand += MAX(ng_jerk_limit / ahrs.get_EAS2TAS(), 0.1f) * plane.scheduler.get_loop_period_s();
            ng_demand = MIN(ng_demand, ng_limit);
            
            // Calculate true airspeed from equivalent airspeed
            const float VTAS_ref = ahrs.get_EAS2TAS() * aspeed;
            
            // Convert load factor to centripetal acceleration: a = ng * g
            const float pullup_accel = ng_demand * GRAVITY_MSS;
            
            // Calculate required pitch rate for desired load factor
            // For coordinated turn/pullup: pitch_rate = acceleration / velocity
            const float demanded_rate_dps = degrees(pullup_accel / VTAS_ref);
            
            // Blend fixed elevator offset with pitch rate control
            // Offset fades out as ng_demand approaches ng_limit (smooth transition to pure rate control)
            const uint32_t elev_trim_offset_cd = 4500.0f * elev_offset * (1.0f - ng_demand / ng_limit);
            
            // Command elevator: fixed offset + pitch rate controller output
            SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, elev_trim_offset_cd + plane.pitchController.get_rate_out(demanded_rate_dps, speed_scaler));
        } else {
            // Fallback if no airspeed: use fixed elevator offset only
            SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, elev_offset*4500);
        }
        break;
    }
    case Stage::PUSH_NOSE_DOWN: {
        plane.nav_pitch_cd = plane.aparm.pitch_limit_min*100;
        plane.stabilize_pitch();
        plane.nav_roll_cd = 0;
        plane.stabilize_roll();
        plane.stabilize_yaw();
        ng_demand = 0.0f;
        break;
    }
    case Stage::WAIT_LEVEL:
        plane.nav_pitch_cd = MAX((plane.aparm.pitch_limit_min + 5), pitch_dem)*100;
        plane.stabilize_pitch();
        plane.nav_roll_cd = 0;
        plane.stabilize_roll();
        plane.stabilize_yaw();
        ng_demand = 0.0f;
        break;
    case Stage::NONE:
        break;
    }

    // we have done stabilisation
    plane.last_stabilize_ms = AP_HAL::millis();
}

#endif // AP_PLANE_GLIDER_PULLUP_ENABLED
