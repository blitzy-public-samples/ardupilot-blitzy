/**
 * @file mode_avoidADSB.cpp
 * @brief Implementation of AVOID_ADSB flight mode for ADS-B based collision avoidance
 * 
 * @details This file implements the AVOID_ADSB flight mode which provides automatic
 *          collision avoidance using ADS-B (Automatic Dependent Surveillance-Broadcast)
 *          traffic information. This mode is designed to maintain safe separation from
 *          other aircraft equipped with ADS-B transponders.
 * 
 *          The AVOID_ADSB mode works by:
 *          - Monitoring ADS-B traffic for potential collision threats
 *          - Automatically taking control when a threat is detected
 *          - Executing avoidance maneuvers using GUIDED mode navigation
 *          - Maintaining loiter pattern while monitoring threats
 * 
 *          This mode is only available when HAL_ADSB_ENABLED is defined at compile time,
 *          which requires appropriate hardware support for ADS-B reception.
 * 
 *          Operational Flow:
 *          1. Mode is entered when ADS-B threat detection triggers avoidance
 *          2. Aircraft transitions to GUIDED mode for precise control
 *          3. Navigate to safe position using loiter pattern
 *          4. Monitor threat until clear, then return to previous mode
 * 
 * @note This mode requires external ADS-B receiver hardware and AP_ADSB library support
 * @warning Collision avoidance is safety-critical; this mode should only be triggered
 *          by validated threat detection algorithms with appropriate safety margins
 * 
 * @see AP_ADSB for traffic monitoring and threat detection
 * @see ModeGuided for underlying navigation implementation
 * 
 * Source: ArduPlane/mode_avoidADSB.cpp
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include "mode.h"
#include "Plane.h"

#if HAL_ADSB_ENABLED

/**
 * @brief Enter AVOID_ADSB mode and initialize avoidance behavior
 * 
 * @details This method is called when the aircraft needs to enter ADS-B collision
 *          avoidance mode. It delegates to the GUIDED mode entry logic to establish
 *          precise navigation control necessary for executing avoidance maneuvers.
 * 
 *          The GUIDED mode provides the low-level navigation framework that allows
 *          the avoidance system to command specific waypoints, loiter circles, and
 *          altitude changes needed to maintain safe separation from traffic.
 * 
 *          Entry Process:
 *          1. Calls ModeGuided::enter() to initialize guided navigation
 *          2. Establishes navigation targets for initial avoidance
 *          3. Prepares aircraft for automated threat response
 * 
 * @return true if mode entry successful and avoidance mode is active
 * @return false if mode entry failed (e.g., GUIDED mode unavailable or pre-arm checks failed)
 * 
 * @note Mode entry may fail if GUIDED mode preconditions are not met
 * @warning Ensure proper ADS-B receiver initialization before entering this mode
 * 
 * @see ModeGuided::enter() for underlying entry logic
 * @see AP_ADSB for threat detection that triggers this mode
 */
bool ModeAvoidADSB::_enter()
{
    return plane.mode_guided.enter();
}

/**
 * @brief Update AVOID_ADSB mode navigation and execute avoidance maneuvers
 * 
 * @details This method is called at the main loop rate to update the avoidance mode's
 *          navigation state. It delegates to GUIDED mode's update logic which handles
 *          the continuous adjustment of flight controls to track avoidance waypoints
 *          and maintain safe separation from detected ADS-B traffic threats.
 * 
 *          The update cycle performs:
 *          - Processes current navigation targets from AP_Avoidance integration
 *          - Updates attitude and throttle controllers for avoidance trajectory
 *          - Monitors progress toward safe position
 *          - Adjusts control outputs based on wind and aircraft state
 * 
 *          Integration with AP_Avoidance:
 *          The AP_Avoidance library (when enabled) provides high-level avoidance
 *          guidance by analyzing ADS-B traffic data and computing safe escape
 *          vectors. This update() method ensures those guidance commands are
 *          translated into actual aircraft control through the GUIDED mode framework.
 * 
 * @note Called at main loop rate (typically 50-400 Hz depending on scheduler configuration)
 * @note AP_Avoidance integration provides threat assessment and avoidance vectors
 * @warning Continuous execution is critical for maintaining safe separation during avoidance
 * 
 * @see ModeGuided::update() for control law implementation
 * @see AP_Avoidance::update() for threat vector computation
 * @see navigate() for loiter pattern management during avoidance
 */
void ModeAvoidADSB::update()
{
    plane.mode_guided.update();
}

/**
 * @brief Execute navigation pattern during ADS-B collision avoidance
 * 
 * @details This method implements the navigation behavior while in AVOID_ADSB mode,
 *          establishing a loiter (circular holding) pattern at the current position.
 *          The loiter pattern provides a predictable, stable flight path while the
 *          avoidance system monitors ADS-B traffic and determines when it is safe
 *          to return to the previous flight mode.
 * 
 *          Navigation Strategy:
 *          - Maintains circular loiter pattern at current location
 *          - Uses configured WP_LOITER_RAD parameter for loiter radius
 *          - Provides stable platform for continuous threat assessment
 *          - Allows aircraft to "wait out" passing traffic threats
 * 
 *          Loiter Radius Configuration:
 *          The radius parameter of 0 indicates that the standard waypoint loiter
 *          radius (WP_LOITER_RAD) should be used. This ensures consistency with
 *          other loiter-based modes and respects pilot-configured loiter preferences.
 * 
 *          Threat Detection and Avoidance Maneuvers:
 *          While loitering, the AP_ADSB library continuously monitors traffic:
 *          - Tracks position, altitude, and velocity of nearby aircraft
 *          - Computes closest point of approach (CPA) and time to CPA
 *          - Triggers altitude or lateral offset if threat persists
 *          - Updates loiter center if avoidance vector requires repositioning
 * 
 *          The loiter pattern serves as the default "safe" behavior, providing:
 *          1. Predictable path for conflict resolution with traffic
 *          2. Minimal altitude change (maintains vertical separation)
 *          3. Tight turn radius for quick response to new threats
 *          4. Low ground speed for extended monitoring time
 * 
 * @note Called by the navigation controller when lateral navigation updates are needed
 * @note Loiter radius determined by WP_LOITER_RAD parameter (0 = use default radius)
 * @warning Aircraft will remain in loiter until threat clears or pilot intervention
 * @warning Ensure sufficient altitude for safe loiter pattern execution
 * 
 * @see Plane::update_loiter() for loiter pattern implementation
 * @see WP_LOITER_RAD parameter for configured loiter radius
 * @see AP_ADSB for traffic threat detection and collision prediction
 */
void ModeAvoidADSB::navigate()
{
    // Zero indicates to use WP_LOITER_RAD
    plane.update_loiter(0);
}

#endif // HAL_ADSB_ENABLED

