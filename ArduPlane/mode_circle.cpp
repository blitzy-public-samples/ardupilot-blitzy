/**
 * @file mode_circle.cpp
 * @brief Implementation of CIRCLE flight mode for fixed-wing aircraft
 * 
 * @details This file implements the CIRCLE mode for ArduPlane, which commands
 *          the aircraft to fly in a continuous circular pattern. CIRCLE mode
 *          is typically used in scenarios where GPS navigation is unavailable
 *          or when a simple holding pattern is desired without waypoint navigation.
 *          
 *          The mode operates by:
 *          - Capturing the current altitude when entering the mode
 *          - Applying a constant bank angle (1/3 of roll limit) to maintain circular flight
 *          - Holding the captured altitude throughout the circle
 *          - Operating without GPS waypoint guidance
 *          
 *          Circle Characteristics:
 *          - Direction: Determined by roll direction (bank angle sign)
 *          - Radius: Calculated based on airspeed, bank angle, and gravity
 *          - Altitude: Fixed at the altitude when mode was entered
 *          
 *          Use Cases:
 *          - Emergency navigation when GPS is lost
 *          - Holding pattern without GPS waypoints
 *          - Gentle circling for payload operations
 *          - Failsafe mode for maintaining safe flight
 *          
 * @note This mode does not use GPS waypoints or L1 navigation controller
 * @warning The circle radius and behavior depend on airspeed and roll limits,
 *          ensure proper tuning to avoid terrain conflicts
 * 
 * @see ModeCircle class definition in mode.h
 * @see Mode base class for mode interface contract
 * 
 * Source: ArduPlane/mode_circle.cpp
 */

#include "mode.h"
#include "Plane.h"

/**
 * @brief Enter CIRCLE mode and initialize circling parameters
 * 
 * @details This method is called when the flight mode transitions to CIRCLE mode.
 *          It captures the current aircraft location (including altitude) and sets
 *          it as the target waypoint location. This altitude will be maintained
 *          throughout the circular flight pattern.
 *          
 *          Initialization Process:
 *          1. Capture current aircraft location (lat, lon, alt)
 *          2. Set next_WP_loc to current position to lock altitude reference
 *          3. Altitude hold will maintain this captured altitude during circling
 *          
 *          The circular flight pattern begins immediately after this initialization,
 *          with the aircraft banking to the configured roll angle (1/3 of roll limit)
 *          to establish the circular path.
 * 
 * @return true Always returns true indicating successful mode entry
 * 
 * @note The circle altitude is fixed at mode entry and does not update during flight
 * @note No GPS waypoint validation is performed, mode can operate without GPS
 * @warning Ensure safe altitude above terrain before entering this mode
 * 
 * @see update() for the continuous circle navigation control
 * 
 * Source: ArduPlane/mode_circle.cpp:4-10
 */
bool ModeCircle::_enter()
{
    // the altitude to circle at is taken from the current altitude
    plane.next_WP_loc = plane.current_loc;

    return true;
}

/**
 * @brief Compute and execute circle navigation control for continuous circling flight
 * 
 * @details This method is called continuously (typically at 50Hz) to maintain circular
 *          flight. It implements a simple but effective circle navigation algorithm that
 *          does not require GPS waypoint tracking or L1 guidance controller.
 *          
 *          Circle Navigation Algorithm:
 *          1. Set constant bank angle (1/3 of roll limit) to maintain circular path
 *          2. Update load factor based on bank angle for coordinated turn
 *          3. Calculate pitch to maintain altitude at captured reference
 *          4. Calculate throttle to maintain airspeed and energy state
 *          
 *          Bank Angle Strategy:
 *          - Uses conservative bank angle (1/3 of maximum roll limit)
 *          - Gentle bank ensures safe, stable circling without excessive G-loading
 *          - Circle radius naturally determined by: R = V² / (g × tan(bank_angle))
 *          - Positive roll creates right turn, negative roll creates left turn
 *          
 *          Altitude Hold:
 *          - calc_nav_pitch() computes pitch angle to maintain altitude set in _enter()
 *          - Uses altitude error to generate pitch demand
 *          - Works with calc_throttle() for energy management
 *          
 *          Operational Modes:
 *          - GPS unavailable: Provides safe holding pattern without position reference
 *          - Radio loss: Automated circling reduces risk during communication failure
 *          - Manual selection: Pilot-commanded gentle circling for specific operations
 *          
 *          Control Output (centidegrees):
 *          - nav_roll_cd: Desired roll angle in centidegrees (roll_limit_cd / 3)
 *          - nav_pitch_cd: Calculated by calc_nav_pitch() for altitude hold
 *          - throttle: Calculated by calc_throttle() for airspeed/energy management
 * 
 * @note Called at scheduler task rate (typically 50Hz for navigation tasks)
 * @note Does not use GPS position - suitable for GPS-denied environments
 * @note Circle radius varies with airspeed: higher speed = larger radius
 * @note Bank angle is constant regardless of wind conditions
 * 
 * @warning Ensure roll_limit_cd is properly configured for safe bank angles
 * @warning Circle may drift with wind since no position correction is applied
 * @warning Verify adequate altitude above terrain for the resulting circle radius
 * 
 * @see _enter() for initialization of circle altitude reference
 * @see Plane::update_load_factor() for coordinated turn load factor calculation
 * @see Plane::calc_nav_pitch() for altitude hold pitch computation
 * @see Plane::calc_throttle() for airspeed and energy management
 * 
 * Source: ArduPlane/mode_circle.cpp:12-22
 */
void ModeCircle::update()
{
    // we have no GPS installed and have lost radio contact
    // or we just want to fly around in a gentle circle w/o GPS,
    // holding altitude at the altitude we set when we
    // switched into the mode
    plane.nav_roll_cd  = plane.roll_limit_cd / 3;
    plane.update_load_factor();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}

