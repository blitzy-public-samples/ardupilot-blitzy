/**
 * @file mode_fbwa.cpp
 * @brief Implementation of FLY BY WIRE A (FBWA) flight mode for ArduPlane
 * 
 * @details FBWA mode provides pilot-commanded roll and pitch attitude stabilization
 *          with direct throttle pass-through. This mode is commonly used for manual
 *          flight with stabilization assistance, allowing the pilot to control the
 *          aircraft's attitude while the autopilot maintains stability within configured
 *          limits. FBWA mode includes failsafe glide capability and optional taildragger
 *          takeoff mode support.
 * 
 * Source: ArduPlane/mode_fbwa.cpp
 */

#include "mode.h"
#include "Plane.h"

/**
 * @brief Update navigation targets for FBWA mode based on pilot stick inputs
 * 
 * @details FBWA mode implements pilot-commanded attitude control with the following features:
 * 
 *          Roll Control:
 *          - Maps pilot roll stick input to nav_roll_cd within configured roll limits
 *          - Roll limit defined by ROLL_LIMIT_DEG parameter
 * 
 *          Pitch Control:
 *          - Maps pilot pitch stick to nav_pitch_cd with asymmetric limits
 *          - Positive (nose up): Limited by PTCH_LIM_MAX_DEG parameter
 *          - Negative (nose down): Limited by PTCH_LIM_MIN_DEG parameter (typically more restrictive)
 *          - Adjusts pitch/throttle coupling via adjust_nav_pitch_throttle()
 *          - Final pitch constrained to configured min/max limits
 * 
 *          Inverted Flight:
 *          - Negates pitch commands when flying inverted (auto-detected or pilot-commanded)
 *          - Maintains intuitive pitch control regardless of aircraft orientation
 * 
 *          FBWA Failsafe Glide Mode:
 *          - Activates on RC loss when FS_SHORT_ACTN parameter = 3 (FBWA)
 *          - Sets wings level (nav_roll_cd = 0)
 *          - Sets neutral pitch (nav_pitch_cd = 0)
 *          - Sets throttle to idle (minimum output limit)
 *          - Provides controlled glide in case of RC link loss
 * 
 *          Taildragger Takeoff Mode (Optional):
 *          - Activates via auxiliary switch (RC_CHANNEL AUX_FUNC = FBWA_TAILDRAGGER)
 *          - Only activates if airspeed < TKOFF_TDRAG_SPD1 parameter
 *          - Helps taildraggers maintain directional control during ground roll
 *          - Sends GCS notification when mode activates
 * 
 * @note Failsafe glide requires FS_SHORT_ACTN = 3 (FBWA mode)
 * @note Taildragger mode only activates at low airspeed to prevent inadvertent activation in flight
 * @note This method is called at the navigation update rate (typically 10-50Hz depending on configuration)
 * @note Load factor is updated to account for coordinated turns affecting pitch requirements
 * 
 * @see ModeFBWA::run()
 * @see Plane::adjust_nav_pitch_throttle()
 * @see Plane::fly_inverted()
 * 
 * Source: ArduPlane/mode_fbwa.cpp:4-36
 */
void ModeFBWA::update()
{
    // set nav_roll and nav_pitch using sticks
    plane.nav_roll_cd  = plane.channel_roll->norm_input() * plane.roll_limit_cd;
    plane.update_load_factor();
    float pitch_input = plane.channel_pitch->norm_input();
    if (pitch_input > 0) {
        plane.nav_pitch_cd = pitch_input * plane.aparm.pitch_limit_max*100;
    } else {
        plane.nav_pitch_cd = -(pitch_input * plane.pitch_limit_min*100);
    }
    plane.adjust_nav_pitch_throttle();
    plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, plane.pitch_limit_min*100, plane.aparm.pitch_limit_max.get()*100);
    if (plane.fly_inverted()) {
        plane.nav_pitch_cd = -plane.nav_pitch_cd;
    }
    if (plane.failsafe.rc_failsafe && plane.g.fs_action_short == FS_ACTION_SHORT_FBWA) {
        // FBWA failsafe glide
        plane.nav_roll_cd = 0;
        plane.nav_pitch_cd = 0;
        SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::MIN);
    }
    RC_Channel *chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::FBWA_TAILDRAGGER);
    if (chan != nullptr) {
        // check for the user enabling FBWA taildrag takeoff mode
        bool tdrag_mode = chan->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::HIGH;
        if (tdrag_mode && !plane.auto_state.fbwa_tdrag_takeoff_mode) {
            if (plane.auto_state.highest_airspeed < plane.g.takeoff_tdrag_speed1) {
                plane.auto_state.fbwa_tdrag_takeoff_mode = true;
                plane.gcs().send_text(MAV_SEVERITY_WARNING, "FBWA tdrag mode");
            }
        }
    }
}

/**
 * @brief Main control loop execution for FBWA mode
 * 
 * @details Executes the FBWA mode control sequence:
 *          1. Calls base Mode::run() to execute standard stabilization control loops
 *             (roll controller, pitch controller, yaw damper)
 *          2. Outputs pilot throttle directly without throttle controller intervention
 * 
 *          Throttle Handling:
 *          - Direct pass-through from pilot RC input to throttle servo output
 *          - No altitude hold or throttle controller active in FBWA
 *          - Pilot has full manual throttle control
 *          - Throttle failsafe handled separately in update() method
 * 
 * @note FBWA mode provides attitude stabilization but not altitude hold
 * @note Throttle is completely manual - pilot must manage airspeed and altitude
 * @note Called at main loop rate (typically 50-400Hz depending on scheduler configuration)
 * 
 * @see ModeFBWA::update()
 * @see Mode::run()
 * 
 * Source: ArduPlane/mode_fbwa.cpp:39-45
 */
void ModeFBWA::run()
{
    // Run base class function and then output throttle
    Mode::run();

    output_pilot_throttle();
}
