/**
 * @file avoidance_adsb.h
 * @brief ADS-B collision avoidance implementation for ArduPlane
 * 
 * @details This file defines the AP_Avoidance_Plane class which provides
 *          fixed-wing aircraft-specific implementation of ADS-B (Automatic
 *          Dependent Surveillance-Broadcast) based collision avoidance.
 *          
 *          The class extends the generic AP_Avoidance base class with
 *          Plane-specific behavior such as:
 *          - Disabling avoidance while on the ground
 *          - Fixed-wing specific horizontal avoidance maneuvers
 *          - Altitude-based vertical avoidance appropriate for planes
 *          - Mode management for entering/exiting avoidance maneuvers
 *          
 *          ADS-B avoidance allows the aircraft to automatically detect and
 *          avoid other aircraft equipped with ADS-B transponders by modifying
 *          the flight path when a collision threat is detected.
 * 
 * @note This implementation is only compiled when AP_ADSB_AVOIDANCE_ENABLED
 *       is defined and requires an ADS-B receiver connected to the autopilot.
 * 
 * @see AP_Avoidance for the base avoidance framework
 * @see libraries/AP_Avoidance/AP_Avoidance.h for obstacle detection logic
 */

#pragma once

#include <AP_Avoidance/AP_Avoidance.h>

#if AP_ADSB_AVOIDANCE_ENABLED

/**
 * @class AP_Avoidance_Plane
 * @brief Plane-specific implementation of ADS-B collision avoidance
 * 
 * @details Provides fixed-wing aircraft-specific overrides of the generic
 *          AP_Avoidance collision avoidance system. While the base AP_Avoidance
 *          class handles obstacle detection, threat assessment, and general
 *          avoidance algorithms, this class customizes the behavior for the
 *          unique characteristics of fixed-wing aircraft.
 *          
 *          Key Plane-specific behaviors:
 *          - Avoidance is disabled while landed (unlike multirotors)
 *          - Horizontal avoidance uses waypoint-based navigation changes
 *          - Vertical avoidance modifies target altitude smoothly
 *          - Mode transitions preserve the original flight mode for recovery
 *          - Coordinated turns and bank angle limits are respected
 *          
 *          The avoidance system operates by:
 *          1. Receiving obstacle data from ADS-B receiver via AP_ADSB
 *          2. Calculating threat vectors and time-to-collision
 *          3. Determining avoidance action (vertical, horizontal, or both)
 *          4. Temporarily modifying navigation to avoid the threat
 *          5. Returning to original flight plan after threat passes
 * 
 * @note This class uses the constructor from the base class via 'using'
 *       declaration and is not copyable.
 * 
 * @warning Avoidance maneuvers can override pilot commands and change the
 *          flight path automatically. Ensure ADS-B avoidance parameters are
 *          properly tuned and tested in simulation before flight.
 */
class AP_Avoidance_Plane : public AP_Avoidance {
public:

    /// @brief Inherit base class constructor
    using AP_Avoidance::AP_Avoidance;

    /// @brief Prevent copy construction and assignment
    /* Do not allow copies */
    CLASS_NO_COPY(AP_Avoidance_Plane);

protected:
    /**
     * @brief Handle avoidance maneuver for detected obstacle
     * 
     * @details This is the main avoidance handler that overrides the base class
     *          implementation to provide Plane-specific avoidance behavior. Called
     *          when an obstacle (another aircraft) is detected and a collision
     *          threat is assessed. The method determines the appropriate avoidance
     *          action and executes the maneuver.
     *          
     *          Plane-specific handling includes:
     *          - Checking if the aircraft is on the ground (no avoidance if landed)
     *          - Evaluating vertical vs horizontal avoidance based on relative positions
     *          - Coordinating flight mode changes for avoidance maneuvers
     *          - Calculating new navigation targets that clear the threat
     *          
     *          The avoidance action is selected based on:
     *          - Relative altitude of the threat
     *          - Horizontal separation and closure rate
     *          - Current flight mode and aircraft state
     *          - Configured avoidance parameters (ADSB_*)
     * 
     * @param[in] obstacle Pointer to obstacle data including position, velocity,
     *                     and threat assessment. Contains the detected aircraft's
     *                     location, heading, altitude, and calculated threat level.
     * @param[in] requested_action The collision avoidance action recommended by
     *                             the base avoidance algorithm (climb, descend,
     *                             turn left, turn right, etc.)
     * 
     * @return MAV_COLLISION_ACTION The actual avoidance action taken, which may
     *         differ from requested_action based on Plane-specific constraints
     *         (e.g., MAV_COLLISION_ACTION_NONE if on ground)
     * 
     * @note Called at the main loop rate when an active threat is present
     * @warning This method can change flight mode and override current navigation
     *          commands. Ensure proper parameters are set to limit avoidance aggressiveness.
     * 
     * @see handle_avoidance_vertical() for altitude-based avoidance
     * @see handle_avoidance_horizontal() for heading-based avoidance
     */
    MAV_COLLISION_ACTION handle_avoidance(const AP_Avoidance::Obstacle *obstacle, MAV_COLLISION_ACTION requested_action) override;

    /**
     * @brief Handle recovery from avoidance maneuver
     * 
     * @details This method is called when the collision threat has passed and
     *          the aircraft should return to its original flight plan. Overrides
     *          the base class recovery handler to provide Plane-specific behavior.
     *          
     *          Recovery actions include:
     *          - Returning to the previous flight mode stored before avoidance
     *          - Resuming the original mission or navigation command
     *          - Smoothly transitioning from avoidance waypoint back to flight plan
     *          - Clearing avoidance state flags and temporary navigation data
     *          
     *          The recovery process ensures the aircraft doesn't make abrupt
     *          changes that could be uncomfortable or unsafe. For fixed-wing
     *          aircraft, this means coordinated transitions back to the original
     *          heading and altitude.
     * 
     * @param[in] recovery_action The type of recovery to perform as determined
     *                            by the base avoidance system (resume mission,
     *                            RTL, loiter, etc.)
     * 
     * @note Recovery may take several seconds as the aircraft smoothly transitions
     *       back to the original flight path
     * @warning Rapid mode changes during recovery could cause instability in
     *          some flight conditions. The recovery is designed to be gradual.
     * 
     * @see handle_avoidance() for the avoidance entry point
     */
    void handle_recovery(RecoveryAction recovery_action) override;

    /**
     * @brief Check if current flight mode supports avoidance
     * 
     * @details Verifies that the aircraft is in an appropriate flight mode for
     *          executing avoidance maneuvers. Also handles switching to the
     *          AVOID_ADSB flight mode if automatic mode changes are allowed.
     *          
     *          Some flight modes (like MANUAL, ACRO, or TRAINING) should not
     *          have avoidance override pilot commands. This method checks the
     *          current mode and optionally switches to AVOID_ADSB mode which
     *          is specifically designed for collision avoidance maneuvers.
     *          
     *          The AVOID_ADSB mode is a variant of GUIDED mode that allows
     *          the avoidance system to command waypoints and altitude changes
     *          while maintaining awareness that it's in an avoidance situation.
     * 
     * @param[in] allow_mode_change If true, the method will attempt to switch
     *                              to AVOID_ADSB mode if not already in it.
     *                              If false, only checks current mode validity.
     * 
     * @return true if the aircraft is in AVOID_ADSB mode or successfully switched
     *              to it (when allow_mode_change is true)
     * @return false if the current mode doesn't support avoidance and mode
     *               change was not allowed or failed
     * 
     * @note This method stores the previous mode in prev_control_mode_number
     *       so it can be restored during recovery
     * @warning Automatic mode changes will interrupt current flight operations.
     *          Configure ADSB_ENABLE and related parameters carefully.
     */
    bool check_flightmode(bool allow_mode_change);

    /**
     * @brief Execute vertical (altitude-based) avoidance maneuver
     * 
     * @details Calculates and commands an altitude change to avoid a collision
     *          threat when the primary avoidance strategy is vertical separation.
     *          This is typically used when the threat aircraft is at a similar
     *          altitude and horizontal avoidance would not provide sufficient
     *          separation.
     *          
     *          The method:
     *          - Calculates a safe altitude that provides vertical separation
     *          - Determines climb or descent based on relative positions
     *          - Respects aircraft performance limits (climb rate, ceiling)
     *          - Considers terrain and altitude constraints
     *          - Updates the target location with the new altitude
     *          
     *          For fixed-wing aircraft, vertical avoidance must account for:
     *          - Minimum airspeed during climbs
     *          - Stall margins during aggressive pull-ups
     *          - Engine power limitations affecting climb rate
     *          - Descent rate limits to avoid overspeeding
     * 
     * @param[in]  obstacle Pointer to obstacle data with position, velocity,
     *                      and threat level of the detected aircraft
     * @param[in]  allow_mode_change If true, allows switching to AVOID_ADSB mode
     *                               to execute the maneuver
     * @param[out] new_loc Updated location with modified altitude for avoidance.
     *                     Latitude and longitude remain unchanged, only altitude
     *                     (AMSL) is modified.
     * 
     * @return true if vertical avoidance was successfully calculated and the
     *              new altitude command was set
     * @return false if vertical avoidance is not appropriate (e.g., insufficient
     *               vertical separation possible, aircraft on ground, mode change
     *               not allowed when required)
     * 
     * @note The altitude change is commanded immediately but the aircraft response
     *       time depends on climb/descent rate and TECS controller tuning
     * @warning Aggressive climb commands near stall speed could be dangerous.
     *          Ensure ADSB_VEL_Z_MAX parameter is set conservatively.
     * 
     * @see handle_avoidance_horizontal() for lateral avoidance
     * @see AP_TECS for the energy management system controlling altitude changes
     */
    bool handle_avoidance_vertical(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change, Location &new_loc);

    /**
     * @brief Execute horizontal (heading-based) avoidance maneuver
     * 
     * @details Calculates and commands a heading or waypoint change to avoid a
     *          collision threat when the primary avoidance strategy is lateral
     *          separation. This is typically used when the threat aircraft is
     *          at a significantly different altitude or when horizontal maneuvering
     *          provides the fastest escape path.
     *          
     *          The method:
     *          - Calculates safe bearing that avoids the threat vector
     *          - Determines turn direction (left or right) for minimum deviation
     *          - Computes waypoint location to achieve lateral separation
     *          - Respects bank angle and turn radius limitations
     *          - Updates target location with new lateral position
     *          
     *          For fixed-wing aircraft, horizontal avoidance must account for:
     *          - Minimum turn radius based on airspeed and bank angle
     *          - Coordinated turn requirements (no skidding/slipping)
     *          - Wind effects on ground track during turns
     *          - Airspace boundaries and terrain clearance
     *          - L1 navigation controller response time
     * 
     * @param[in]  obstacle Pointer to obstacle data with position, velocity,
     *                      and threat level of the detected aircraft
     * @param[in]  allow_mode_change If true, allows switching to AVOID_ADSB mode
     *                               to execute the maneuver
     * @param[out] new_loc Updated location with modified latitude/longitude for
     *                     avoidance. Altitude remains unchanged, only horizontal
     *                     position is modified to create a new waypoint.
     * 
     * @return true if horizontal avoidance was successfully calculated and the
     *              new waypoint command was set
     * @return false if horizontal avoidance is not appropriate (e.g., insufficient
     *               lateral separation possible, aircraft on ground, mode change
     *               not allowed when required)
     * 
     * @note The waypoint is set at a distance that provides safe separation
     *       margin beyond the minimum required clearance
     * @warning Sharp turns at low altitude or low airspeed increase stall risk.
     *          Ensure LIM_ROLL_CD and ADSB_VEL_XY_MAX are set appropriately.
     * 
     * @see handle_avoidance_vertical() for altitude-based avoidance
     * @see AP_L1_Control for the navigation controller managing waypoint tracking
     */
    bool handle_avoidance_horizontal(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change, Location &new_loc);

    /**
     * @brief Stores the flight mode active before avoidance maneuver started
     * 
     * @details This member variable preserves the flight mode that was active
     *          when the avoidance system took control by switching to AVOID_ADSB
     *          mode. After the collision threat passes and recovery is initiated,
     *          this stored mode is used to return the aircraft to its original
     *          operation.
     *          
     *          Default value is RTL (Return To Launch) as a safe fallback if
     *          recovery is attempted without a stored previous mode. In normal
     *          operation, this is set by check_flightmode() when entering
     *          avoidance and restored by handle_recovery() when exiting.
     * 
     * @note This only stores the mode number, not the full mode state or
     *       navigation targets, which are managed separately
     */
    enum Mode::Number prev_control_mode_number = Mode::Number::RTL;
};

#endif  // AP_ADSB_AVOIDANCE_ENABLED
