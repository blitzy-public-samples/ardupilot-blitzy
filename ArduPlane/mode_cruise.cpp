/**
 * @file mode_cruise.cpp
 * @brief CRUISE mode implementation for ArduPlane
 * 
 * @details CRUISE mode provides semi-autonomous flight control where the aircraft
 *          maintains a user-commanded altitude and automatically locks heading to
 *          the GPS ground course when specific conditions are met. This mode is
 *          designed for efficient long-distance cruise flight with minimal pilot
 *          input while retaining the ability to manually adjust heading when needed.
 *          
 *          Key features:
 *          - Altitude hold using TECS (Total Energy Control System)
 *          - Automatic heading lock to GPS ground course after stable flight
 *          - Manual heading override via aileron or rudder input
 *          - Ground speed-based heading lock activation (minimum 3 m/s)
 *          - Integration with soaring controller for thermal exploitation
 *          - Support for scripting-based maneuvers with automatic unlock
 *          
 *          Heading Lock Behavior:
 *          - Heading locks to GPS ground course when:
 *            * No aileron or rudder input for 0.5 seconds
 *            * GPS fix quality is 2D or better
 *            * Ground speed exceeds GPS_GND_CRS_MIN_SPD (3 m/s)
 *            * Aircraft moving in forward direction
 *          - Heading unlocks immediately on any aileron or rudder input
 *          - Locked heading maintained via L1 navigation controller
 *          
 * @note This mode is particularly useful for long cross-country flights where
 *       the pilot wants altitude hold with occasional heading adjustments
 * 
 * @warning CRUISE mode requires GPS lock and sufficient airspeed for safe operation.
 *          Always ensure adequate altitude for failsafe recovery.
 * 
 * @see ModeAuto for fully autonomous waypoint navigation
 * @see ModeFBWB for fly-by-wire with altitude hold but no heading lock
 * @see AP_TECS for altitude and speed control implementation
 */

#include "mode.h"
#include "Plane.h"

/**
 * @brief Enter CRUISE mode and initialize heading/altitude targets
 * 
 * @details Called when transitioning into CRUISE mode from another flight mode.
 *          Initializes the mode by:
 *          - Unlocking heading to allow manual control initially
 *          - Resetting the heading lock timer
 *          - Initializing soaring controller if enabled
 *          - Setting target altitude to current altitude
 *          
 *          The heading starts unlocked to give the pilot immediate roll control
 *          upon entering the mode. Heading lock will engage automatically once
 *          the conditions in navigate() are satisfied.
 * 
 * @return true if mode entry successful, false otherwise
 * 
 * @note This method is called by the mode switching logic in the main vehicle code
 */
bool ModeCruise::_enter()
{
    locked_heading = false;
    lock_timer_ms = 0;

#if HAL_SOARING_ENABLED
    // for ArduSoar soaring_controller
    plane.g2.soaring_controller.init_cruising();
#endif

    plane.set_target_altitude_current();

    return true;
}

/**
 * @brief Update CRUISE mode control outputs for current flight loop iteration
 * 
 * @details Called at main loop rate (typically 50-400 Hz depending on board) to update
 *          attitude control outputs for CRUISE mode. This method manages the transition
 *          between manual roll control and heading-locked navigation control based on
 *          pilot input and heading lock state.
 *          
 *          Roll Control Behavior:
 *          - Unlocked heading: Uses direct pilot aileron input scaled to roll_limit_cd
 *          - Locked heading: Uses L1 navigation controller via calc_nav_roll()
 *          
 *          Heading Unlock Triggers:
 *          - Any non-zero aileron input from pilot (channel_roll)
 *          - Any non-zero rudder input from pilot (channel_rudder)
 *          - Active scripting maneuver execution
 *          
 *          When heading unlocks, the lock timer is reset to zero, requiring the full
 *          0.5 second stable period before heading can lock again (see navigate()).
 *          
 *          Speed/Altitude Management:
 *          Calls update_fbwb_speed_height() which integrates with TECS to maintain:
 *          - Target altitude from pilot throttle input or altitude hold
 *          - Target airspeed from TECS tuning parameters
 *          - Energy distribution between altitude and speed
 * 
 * @note This method is called at main loop rate, so efficiency is important
 * @note Scripting integration allows automated aerobatic maneuvers with temporary
 *       heading unlock and altitude reset
 * 
 * @warning Do not modify roll outputs directly in other code when in CRUISE mode,
 *          as this will conflict with the heading lock navigation control
 * 
 * @see navigate() for heading lock acquisition logic
 * @see update_fbwb_speed_height() for TECS altitude/speed control
 * @see calc_nav_roll() for L1 navigation-based roll control
 */
void ModeCruise::update()
{
    /*
      in CRUISE mode we use the navigation code to control
      roll when heading is locked. Heading becomes unlocked on
      any aileron or rudder input
    */
    if (plane.channel_roll->get_control_in() != 0 || plane.channel_rudder->get_control_in() != 0) {
        locked_heading = false;
        lock_timer_ms = 0;
    }

#if AP_SCRIPTING_ENABLED
    if (plane.nav_scripting_active()) {
        // while a trick is running unlock heading and zero altitude offset
        locked_heading = false;
        lock_timer_ms = 0;
        plane.set_target_altitude_current();
    }
#endif
    
    if (!locked_heading) {
        plane.nav_roll_cd = plane.channel_roll->norm_input() * plane.roll_limit_cd;
        plane.update_load_factor();
    } else {
        plane.calc_nav_roll();
    }
    plane.update_fbwb_speed_height();
}

/**
 * @brief Handle CRUISE mode navigation and automatic heading lock acquisition
 * 
 * @details Called during navigation processing to manage automatic heading lock to
 *          GPS ground course. This method implements a state machine that monitors
 *          flight conditions and pilot inputs to determine when to lock/unlock heading.
 *          
 *          Heading Lock Acquisition Conditions (all must be true):
 *          - Heading currently unlocked (locked_heading == false)
 *          - Zero aileron input from pilot (channel_roll == 0)
 *          - Zero rudder input from pilot (rudder_input() == 0)
 *          - GPS fix quality 2D or better (GPS_OK_FIX_2D minimum)
 *          - Ground speed >= GPS_GND_CRS_MIN_SPD (3 m/s default)
 *          - Aircraft moving in forward direction (within 90° of nose heading)
 *          - Conditions stable for 0.5 seconds (500ms timer)
 *          
 *          Ground Speed Control:
 *          The GPS_GND_CRS_MIN_SPD threshold (3 m/s) prevents heading lock during:
 *          - Taxi operations or slow flight where GPS course is unreliable
 *          - Wind drift situations where ground course != aircraft heading
 *          - Launch and landing phases with low ground speed
 *          
 *          Once conditions are met for 500ms, the heading locks to the current GPS
 *          ground course and the L1 navigation controller maintains that heading.
 *          
 *          Forward Motion Detection:
 *          Compares GPS ground course to aircraft yaw angle. If difference exceeds
 *          90 degrees, aircraft is considered to be flying backwards or sideways
 *          (e.g., in strong tailwind), and heading lock is inhibited.
 *          
 *          Locked Heading Navigation:
 *          When locked, creates a virtual waypoint 1km ahead on the locked bearing
 *          and uses L1 controller to track that bearing. The waypoint continuously
 *          updates based on current position to maintain constant ground track.
 * 
 * @note The 0.5 second timer prevents heading lock oscillation during turbulence
 *       or brief pilot stick movements
 * 
 * @note Scripting maneuvers disable navigation entirely to give full control to
 *       the script-based flight plan
 * 
 * @warning Heading lock requires valid GPS ground course. In GPS-denied environments
 *          or during GPS jamming, this mode will not lock heading and will remain
 *          in manual roll control
 * 
 * @see update() for roll control implementation with locked heading
 * @see AP_GPS::ground_course_cd() for GPS course calculation
 * @see AP_L1_Control::update_waypoint() for L1 navigation controller
 */
void ModeCruise::navigate()
{
#if AP_SCRIPTING_ENABLED
    if (plane.nav_scripting_active()) {
        // don't try to navigate while running trick
        return;
    }
#endif

    // check if we are moving in the direction of the front of the vehicle
    const int32_t ground_course_cd = plane.gps.ground_course_cd();
    const bool moving_forwards = fabsf(wrap_PI(cd_to_rad(ground_course_cd) - plane.ahrs.get_yaw_rad())) < M_PI_2;

    if (!locked_heading &&
        plane.channel_roll->get_control_in() == 0 &&
        plane.rudder_input() == 0 &&
        plane.gps.status() >= AP_GPS::GPS_OK_FIX_2D &&
        plane.gps.ground_speed() >= GPS_GND_CRS_MIN_SPD &&
        moving_forwards &&
        lock_timer_ms == 0) {
        // user wants to lock the heading - start the timer
        lock_timer_ms = millis();
    }
    if (lock_timer_ms != 0 &&
        (millis() - lock_timer_ms) > 500) {
        // lock the heading after 0.5 seconds of zero heading input
        // from user
        locked_heading = true;
        lock_timer_ms = 0;
        locked_heading_cd = ground_course_cd;
        plane.prev_WP_loc = plane.current_loc;
    }
    if (locked_heading) {
        plane.next_WP_loc = plane.prev_WP_loc;
        // always look 1km ahead
        plane.next_WP_loc.offset_bearing(locked_heading_cd*0.01f, plane.prev_WP_loc.get_distance(plane.current_loc) + 1000);
        plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);
    }
}

/**
 * @brief Get the current target heading for CRUISE mode
 * 
 * @details Returns the locked heading value if heading is currently locked to
 *          GPS ground course. This method is used by telemetry systems and
 *          other modes to query what heading CRUISE mode is maintaining.
 *          
 *          When heading is locked, returns the GPS ground course (in centidegrees)
 *          that was captured at the moment of heading lock. This heading remains
 *          constant until heading is unlocked by pilot input.
 *          
 *          When heading is not locked, the returned heading value is undefined
 *          and the return value indicates no valid target heading exists.
 * 
 * @param[out] target_heading Reference to store target heading in centidegrees
 *                            (0-36000, where 36000 = 360 degrees). Only valid
 *                            if return value is true.
 * 
 * @return true if heading is locked and target_heading is valid
 * @return false if heading is not locked (manual roll control active)
 * 
 * @note Heading values are in centidegrees: 0 = North, 9000 = East, 18000 = South
 * @note This method does not modify any state, it only queries current mode state
 */
bool ModeCruise::get_target_heading_cd(int32_t &target_heading) const
{
    target_heading = locked_heading_cd;
    return locked_heading;
}
