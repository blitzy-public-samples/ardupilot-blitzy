/**
 * @file mode_simple.cpp
 * @brief Implementation of Simple mode providing heading-hold control
 * 
 * @details Simple mode provides an intuitive control scheme where the pilot
 *          controls heading and speed independently. The pilot's yaw input
 *          commands an absolute heading (relative to north) while the throttle
 *          commands forward/reverse speed. The vehicle maintains the commanded
 *          heading using GPS and compass feedback.
 *          
 *          This mode simplifies control for new pilots by decoupling heading
 *          control from steering input, making the vehicle easier to operate
 *          compared to manual mode where steering affects rate of turn.
 *          
 *          Two variants are supported via g2.simple_type parameter:
 *          - Simple_InitialHeading: Pilot input is relative to initial heading at mode entry
 *          - Default: Pilot input commands absolute heading relative to north
 *          
 * @note Use cases: Easier control for new pilots, intuitive heading control,
 *       applications requiring absolute heading commands
 * 
 * Source: Rover/mode_simple.cpp
 */

#include "Rover.h"

/**
 * @brief Initialize heading references when entering Simple mode
 * 
 * @details Records the current vehicle heading as both the initial heading
 *          reference (used in Simple_InitialHeading variant) and the desired
 *          heading. This ensures smooth transition into Simple mode without
 *          sudden heading changes.
 *          
 *          Coordinate frame: Heading is measured in centidegrees, clockwise
 *          from north (0 = north, 9000 = east, 18000 = south, 27000 = west)
 */
void ModeSimple::init_heading()
{
    // Record current heading as initial reference for Simple_InitialHeading mode
    _initial_heading_cd = ahrs.yaw_sensor;
    
    // Initialize desired heading to current heading to prevent sudden turns
    _desired_heading_cd = ahrs.yaw_sensor;
}

/**
 * @brief Main control loop for Simple mode - updates heading and speed control
 * 
 * @details Simple mode control logic:
 *          1. Pilot yaw input commands absolute heading (relative to north)
 *          2. Pilot throttle input commands forward/reverse speed
 *          3. Vehicle maintains commanded heading using steering controller
 *          4. Heading hold persists when throttle is zero (vehicle coasting)
 *          
 *          Control independence: Unlike manual mode where steering affects
 *          rate of turn, Simple mode allows direct heading commands while
 *          throttle independently controls speed. This decoupling makes the
 *          vehicle respond more like a GPS waypoint navigation.
 *          
 *          Coordinate frame: Heading is absolute (relative to north), not
 *          relative to vehicle's current orientation. 0° = north, 90° = east,
 *          180° = south, 270° = west. This provides intuitive control where
 *          the same stick position always commands the same compass direction.
 *          
 * @note Called at main loop rate (typically 50Hz for Rover)
 * @warning Requires valid GPS and compass for proper heading hold operation
 */
void ModeSimple::update()
{
    float desired_heading_cd, desired_speed;

    // Get pilot input: yaw stick commands heading (centidegrees), throttle commands speed (m/s)
    // Yaw input is converted to absolute heading based on current simple_type configuration
    get_pilot_desired_heading_and_speed(desired_heading_cd, desired_speed);

    // Simple_InitialHeading mode: Pilot input is relative to heading at mode entry
    // Example: If entered mode facing east (90°) and pilot commands 45° right,
    // the absolute heading becomes 90° + 45° = 135° (southeast)
    // This mode is useful for maintaining orientation relative to initial direction
    if (g2.simple_type == Simple_InitialHeading) {
        desired_heading_cd = wrap_360_cd(_initial_heading_cd + desired_heading_cd);
    }

    // Heading hold when stopped: If throttle is in middle (zero speed command),
    // maintain previous desired heading rather than using potentially zero/neutral
    // stick input. This is important when vehicle is coasting to a stop - we want
    // to hold the last commanded heading, not snap to a neutral stick position.
    if (!is_positive(desired_speed)) {
        desired_heading_cd = _desired_heading_cd;
    } else {
        // Record desired heading for next iteration to support heading hold during coast
        _desired_heading_cd = desired_heading_cd;
    }

    // Run heading and speed controllers:
    // - Steering controller: Turns vehicle to achieve desired_heading_cd using GPS/compass feedback
    // - Throttle controller: Accelerates/decelerates to achieve desired_speed using speed feedback
    calc_steering_to_heading(desired_heading_cd);
    calc_throttle(desired_speed, true);
}
