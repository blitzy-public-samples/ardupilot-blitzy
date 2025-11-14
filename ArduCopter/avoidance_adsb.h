/**
 * @file avoidance_adsb.h
 * @brief Copter-specific implementation of ADS-B RF-based collision avoidance for manned aircraft
 * 
 * @details This file implements the AP_Avoidance_Copter class which provides multicopter-specific
 *          handling of ADS-B (Automatic Dependent Surveillance-Broadcast) collision avoidance with
 *          manned aircraft. ADS-B receivers detect transponder signals from nearby aircraft and
 *          calculate collision threats based on closure rate, relative position, and predicted
 *          future positions.
 * 
 *          The system implements vector-based avoidance maneuvers in three dimensions:
 *          - Vertical avoidance: Altitude changes to avoid threats above/below
 *          - Horizontal avoidance: Lateral position changes to avoid threats at similar altitude
 *          - Perpendicular avoidance: 3D vector maneuvers combining horizontal and vertical
 * 
 *          When a collision threat is detected, the AVOID_ADSB flight mode is activated,
 *          which delegates to GUIDED mode for autonomous waypoint navigation while continuously
 *          monitoring threat aircraft positions and adjusting avoidance vectors as needed.
 * 
 *          This copter-specific implementation extends the base AP_Avoidance library to handle
 *          multicopter constraints such as not performing avoidance while landed, managing
 *          altitude limits, and mode transition safety for copter flight characteristics.
 * 
 * @note Requires ADS-B receiver hardware connected via serial/CAN interface
 * @note Configuration via ADSB_* and AVOID_* parameters
 * 
 * @warning Safety-critical collision avoidance system - incorrect configuration or behavior
 *          modifications can result in mid-air collisions with manned aircraft
 * 
 * @see AP_Avoidance Base avoidance library implementing threat detection algorithms
 * @see libraries/AP_ADSB/ ADS-B receiver driver and MAVLink integration
 * 
 * Source: ArduCopter/avoidance_adsb.h
 */

#pragma once

#include <AP_Avoidance/AP_Avoidance_config.h>

#if AP_ADSB_AVOIDANCE_ENABLED

#include <AP_Avoidance/AP_Avoidance.h>

/**
 * @class AP_Avoidance_Copter
 * @brief Copter-specific ADS-B collision avoidance implementation
 * 
 * @details Provides multicopter-specific implementation of ADS-B-based collision avoidance
 *          with manned aircraft. While most of the core avoidance logic (threat detection,
 *          velocity-based prediction, collision assessment) is present in the base AP_Avoidance
 *          class, this derived class allows ArduCopter to override functionality specific to
 *          multicopter flight characteristics.
 * 
 *          Key copter-specific behaviors:
 *          - No avoidance actions while landed (motors disarmed or on ground)
 *          - Respect copter altitude limits (fence, terrain following)
 *          - Mode transition handling compatible with copter flight modes
 *          - Failsafe integration with copter-specific RTL and LAND behaviors
 *          - Vertical avoidance preferred over horizontal when feasible (copters excel at altitude changes)
 * 
 *          Avoidance Strategy Selection:
 *          The system evaluates threats and selects appropriate avoidance strategy based on:
 *          - Threat closure rate and time-to-collision
 *          - Relative altitude of threat aircraft
 *          - Current vehicle state (altitude, position, velocity)
 *          - Configured altitude limits and geofence constraints
 * 
 *          Thread Safety: Methods called from main loop (typically 400Hz) and avoidance thread
 *          (typically 10Hz). No explicit locking required as mode changes are atomic.
 * 
 * @note Integrates with AVOID_ADSB flight mode which delegates to GUIDED mode for execution
 * @note Collision assessment and threat ranking performed in base AP_Avoidance class
 * 
 * @warning Safety-critical system - modifications must be thoroughly tested in SITL and
 *          with caution on physical aircraft. Incorrect behavior can result in collisions.
 * 
 * Source: ArduCopter/avoidance_adsb.h
 */
class AP_Avoidance_Copter : public AP_Avoidance {
public:

    /**
     * @brief Inherit constructors from base AP_Avoidance class
     * 
     * @details Uses C++11 constructor inheritance to forward all constructor arguments
     *          to the base AP_Avoidance class. This avoids duplicating constructor
     *          implementations while maintaining proper initialization.
     */
    using AP_Avoidance::AP_Avoidance;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Avoidance_Copter);

private:
    /**
     * @brief Helper function to set flight mode with cascading fallback to RTL then LAND
     * 
     * @details Attempts to set the requested flight mode, and if that fails, tries RTL
     *          (Return To Launch), and if that also fails, sets LAND mode as final failsafe.
     *          This ensures the vehicle always ends up in a safe mode even if mode transition
     *          errors occur during avoidance maneuvers.
     * 
     *          Mode transition sequence:
     *          1. Try requested mode (typically GUIDED for avoidance waypoint)
     *          2. If fails -> Try RTL (return to launch point)
     *          3. If fails -> Force LAND (emergency landing at current position)
     * 
     *          This function always succeeds because LAND mode cannot fail to activate.
     * 
     * @param[in] mode Desired flight mode to attempt setting (typically Mode::Number::GUIDED)
     * 
     * @note Called during avoidance activation when transitioning to AVOID_ADSB/GUIDED mode
     * @note Function does not return status - guaranteed to end in a valid flight mode
     * 
     * @warning Safety-critical function ensuring vehicle always enters a safe mode during
     *          collision avoidance. Do not modify fallback sequence without thorough testing.
     * 
     * Source: ArduCopter/avoidance_adsb.cpp
     */
    void set_mode_else_try_RTL_else_LAND(Mode::Number mode);

    /**
     * @brief Get minimum altitude limit allowed during descending avoidance maneuvers
     * 
     * @details Calculates the minimum safe altitude the vehicle is allowed to descend to
     *          during collision avoidance. This prevents avoidance maneuvers from violating
     *          configured altitude limits from geofencing, terrain following, or absolute
     *          altitude constraints.
     * 
     *          Considers multiple altitude limit sources:
     *          - Geofence minimum altitude (if fence enabled)
     *          - Terrain following minimum clearance (if terrain following active)
     *          - Absolute minimum safe altitude above ground/home
     *          - Current altitude if already below limits
     * 
     * @return Minimum allowed altitude in centimeters relative to home
     * 
     * @note Return value is in centimeters (ArduPilot standard altitude unit)
     * @note Altitude reference frame is relative to home/launch position
     * @note Called during vertical and perpendicular avoidance calculations
     * 
     * Source: ArduCopter/avoidance_adsb.cpp
     */
    int32_t get_altitude_minimum() const;

protected:
    /**
     * @brief Main avoidance handler that processes detected collision threats
     * 
     * @details Override of base AP_Avoidance::handle_avoidance() to provide copter-specific
     *          collision avoidance behavior. Called when ADS-B system detects a potential
     *          collision threat and determines appropriate avoidance action.
     * 
     *          Decision flow:
     *          1. Check if copter is in valid state for avoidance (not landed, motors armed)
     *          2. Evaluate requested avoidance action from threat assessment
     *          3. Select appropriate avoidance strategy (vertical/horizontal/perpendicular)
     *          4. Execute avoidance maneuver via GUIDED mode waypoint
     *          5. Return actual action taken to base avoidance system
     * 
     *          May override requested action based on copter state (e.g., ignore avoidance
     *          if landed, prefer vertical avoidance if horizontal blocked by terrain).
     * 
     * @param[in] obstacle Pointer to detected obstacle/threat aircraft with position, velocity,
     *                     time-to-collision, and threat assessment data
     * @param[in] requested_action Avoidance action recommended by base threat assessment
     *                             (MAV_COLLISION_ACTION_NONE, ASCEND_OR_DESCEND, MOVE_HORIZONTALLY,
     *                             MOVE_PERPENDICULAR, RTL, LAND)
     * 
     * @return Actual avoidance action taken by the copter (may differ from requested_action
     *         based on copter-specific constraints)
     * 
     * @note Called from avoidance thread at approximately 10Hz when threats detected
     * @note Delegates to handle_avoidance_vertical(), handle_avoidance_horizontal(), or
     *       handle_avoidance_perpendicular() based on selected strategy
     * 
     * @warning Safety-critical collision avoidance decision point - thorough testing required
     *          for any modifications. Incorrect action selection can result in collision.
     * 
     * @see AP_Avoidance::handle_avoidance() Base class threat assessment
     * @see handle_avoidance_vertical() Vertical avoidance implementation
     * @see handle_avoidance_horizontal() Horizontal avoidance implementation
     * @see handle_avoidance_perpendicular() 3D perpendicular avoidance implementation
     * 
     * Source: ArduCopter/avoidance_adsb.cpp
     */
    MAV_COLLISION_ACTION handle_avoidance(const AP_Avoidance::Obstacle *obstacle, MAV_COLLISION_ACTION requested_action) override;

    /**
     * @brief Handle recovery from collision avoidance back to normal flight
     * 
     * @details Override of base AP_Avoidance::handle_recovery() to provide copter-specific
     *          recovery behavior after a collision threat has passed. Called when ADS-B system
     *          determines the threat aircraft has moved to a safe distance and normal flight
     *          operations can resume.
     * 
     *          Recovery actions:
     *          - REMAIN_IN_AVOID_ADSB: Stay in avoidance mode, continue monitoring
     *          - RESUME_PREVIOUS_FLIGHTMODE: Return to flight mode active before avoidance
     *          - RTL: Return to launch as conservative recovery
     *          - RESUME_AUTO_MISSION: Continue autonomous mission from interruption point
     * 
     *          Copter-specific considerations:
     *          - Ensure smooth mode transition without destabilizing vehicle
     *          - Restore previous flight mode if it's still valid
     *          - Fall back to RTL if previous mode cannot be restored
     * 
     * @param[in] recovery_action Recovery action recommended by base avoidance system
     *                            based on threat clearance and previous flight state
     * 
     * @note Called from avoidance thread when threat is no longer present
     * @note Restores prev_control_mode member variable if RESUME_PREVIOUS_FLIGHTMODE selected
     * 
     * @warning Ensure recovery doesn't inadvertently return to unsafe mode or position
     * 
     * @see handle_avoidance() Corresponding avoidance activation handler
     * 
     * Source: ArduCopter/avoidance_adsb.cpp
     */
    void handle_recovery(RecoveryAction recovery_action) override;

    /**
     * @brief Check if flight mode is compatible with ADS-B avoidance activation
     * 
     * @details Verifies the current or requested flight mode is AVOID_ADSB, or if mode
     *          changes are allowed, attempts to transition to AVOID_ADSB mode. Used as
     *          precondition check before executing avoidance maneuvers.
     * 
     *          Mode Compatibility:
     *          - AVOID_ADSB mode: Always compatible (avoidance already active)
     *          - Other modes with allow_mode_change=true: Attempt transition to AVOID_ADSB
     *          - Other modes with allow_mode_change=false: Reject avoidance
     * 
     * @param[in] allow_mode_change If true, attempt to change to AVOID_ADSB mode if not
     *                              already active. If false, only succeed if already in
     *                              AVOID_ADSB mode.
     * 
     * @return true if currently in AVOID_ADSB mode or successfully transitioned to it,
     *         false if mode is incompatible and mode change not allowed/failed
     * 
     * @note Called at start of each avoidance maneuver calculation
     * @note Stores previous mode in prev_control_mode for recovery
     * 
     * Source: ArduCopter/avoidance_adsb.cpp
     */
    bool check_flightmode(bool allow_mode_change);

    /**
     * @brief Execute vertical avoidance maneuver (altitude change) to avoid collision threat
     * 
     * @details Implements vertical collision avoidance by commanding altitude changes to
     *          move above or below threat aircraft flight path. Vertical avoidance is often
     *          preferred for multirotors due to superior vertical agility compared to
     *          horizontal translation.
     * 
     *          Algorithm:
     *          1. Calculate safe altitude offset from threat aircraft (typically +/- 100m)
     *          2. Check altitude limits (minimum altitude, geofence, terrain clearance)
     *          3. Prefer descending if threat above, ascending if threat below
     *          4. Command GUIDED mode waypoint at current horizontal position, new altitude
     *          5. Monitor altitude change progress and threat movement
     * 
     *          Altitude Selection Priorities:
     *          - Move opposite direction of threat (if threat climbing, we descend)
     *          - Respect minimum altitude limits (geofence, terrain)
     *          - Maintain maximum safe separation from threat aircraft
     * 
     * @param[in] obstacle Pointer to detected threat with position, velocity, and trajectory data
     * @param[in] allow_mode_change If true, allows transition to AVOID_ADSB/GUIDED mode.
     *                              If false, only executes if already in compatible mode.
     * 
     * @return true if vertical avoidance maneuver successfully initiated, false if cannot
     *         execute (altitude limits prevent safe maneuver, mode change failed, etc.)
     * 
     * @note Altitude changes commanded via GUIDED mode waypoint at current lat/lon
     * @note Respects AVOID_ALT_MIN parameter for minimum safe altitude
     * @note Typical vertical separation: 100m (configurable via AVOID_MARGIN parameter)
     * 
     * @warning Verify terrain clearance before descending - collision with ground is worse
     *          than potential aircraft collision. Terrain following must be active in
     *          mountainous areas.
     * 
     * @see get_altitude_minimum() Calculates minimum safe descent altitude
     * @see handle_avoidance() Main avoidance dispatcher that calls this method
     * 
     * Source: ArduCopter/avoidance_adsb.cpp
     */
    bool handle_avoidance_vertical(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change);

    /**
     * @brief Execute horizontal avoidance maneuver (lateral movement) to avoid collision threat
     * 
     * @details Implements horizontal collision avoidance by commanding lateral position changes
     *          to move away from threat aircraft flight path while maintaining current altitude.
     *          Used when vertical avoidance is not feasible (altitude limits) or when threat
     *          trajectory makes horizontal separation more effective.
     * 
     *          Algorithm:
     *          1. Calculate threat velocity vector and predicted intersection point
     *          2. Compute perpendicular avoidance vector (90° from threat path)
     *          3. Calculate waypoint offset distance (typically 100m+) in safe direction
     *          4. Command GUIDED mode waypoint at new horizontal position, current altitude
     *          5. Monitor horizontal separation and threat movement
     * 
     *          Direction Selection:
     *          - Vector perpendicular to threat's velocity (avoids predicted path)
     *          - Prefer direction that maximizes separation distance
     *          - Avoid maneuvering toward terrain or obstacles
     *          - Consider wind drift in position calculations
     * 
     * @param[in] obstacle Pointer to detected threat with position, velocity, and trajectory data
     * @param[in] allow_mode_change If true, allows transition to AVOID_ADSB/GUIDED mode.
     *                              If false, only executes if already in compatible mode.
     * 
     * @return true if horizontal avoidance maneuver successfully initiated, false if cannot
     *         execute (geofence limits, mode change failed, insufficient separation possible)
     * 
     * @note Horizontal displacement commanded via GUIDED mode waypoint at current altitude
     * @note Respects geofence boundaries - will not maneuver outside fence
     * @note Typical horizontal separation: 100m+ (configurable via AVOID_MARGIN parameter)
     * @note More energy-intensive than vertical avoidance for multirotors
     * 
     * @warning Ensure adequate GPS accuracy for horizontal position hold - poor GPS can
     *          result in drift back toward threat aircraft
     * 
     * @see handle_avoidance() Main avoidance dispatcher that calls this method
     * 
     * Source: ArduCopter/avoidance_adsb.cpp
     */
    bool handle_avoidance_horizontal(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change);

    /**
     * @brief Execute perpendicular 3D avoidance maneuver combining horizontal and vertical movement
     * 
     * @details Implements three-dimensional collision avoidance by commanding simultaneous
     *          horizontal and vertical position changes. This creates a 3D vector perpendicular
     *          to the threat aircraft's velocity vector, maximizing separation in the shortest
     *          time. Most aggressive avoidance strategy used for high-closure-rate threats.
     * 
     *          Algorithm:
     *          1. Calculate 3D velocity vector of threat aircraft
     *          2. Compute 3D perpendicular vector (orthogonal to threat velocity in 3D space)
     *          3. Scale vector by desired separation distance (typically 100m+)
     *          4. Apply altitude and geofence constraints to resulting waypoint
     *          5. Command GUIDED mode waypoint at calculated 3D position
     *          6. Monitor 3D separation distance and threat movement
     * 
     *          When Used:
     *          - High closure rate threats requiring maximum evasion
     *          - Both horizontal and vertical maneuvers feasible (no altitude/fence limits)
     *          - Threat trajectory intersects current position in 3D space
     *          - Most effective use of multirotor agility in all axes
     * 
     * @param[in] obstacle Pointer to detected threat with 3D position, velocity, and trajectory
     * @param[in] allow_mode_change If true, allows transition to AVOID_ADSB/GUIDED mode.
     *                              If false, only executes if already in compatible mode.
     * 
     * @return true if perpendicular avoidance maneuver successfully initiated, false if cannot
     *         execute (constraints prevent 3D maneuver, mode change failed)
     * 
     * @note Combines benefits of vertical and horizontal avoidance
     * @note Computationally more intensive than single-axis avoidance
     * @note Requires good GPS and barometer accuracy for 3D position hold
     * @note Most rapid threat separation but highest energy consumption
     * 
     * @warning Most aggressive maneuver - can result in rapid position changes that may
     *          startle pilot or trigger other failsafes. Ensure geofence configured with
     *          adequate margin. Test thoroughly in SITL before flight.
     * 
     * @see handle_avoidance_vertical() Single-axis vertical avoidance
     * @see handle_avoidance_horizontal() Single-axis horizontal avoidance
     * @see handle_avoidance() Main avoidance dispatcher that calls this method
     * 
     * Source: ArduCopter/avoidance_adsb.cpp
     */
    bool handle_avoidance_perpendicular(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change);

    /**
     * @brief Flight control mode that was active before avoidance maneuver began
     * 
     * @details Stores the flight mode the copter was in immediately before transitioning to
     *          AVOID_ADSB mode for collision avoidance. Used during recovery to restore the
     *          previous flight state after the threat has passed, allowing mission continuation
     *          or return to pilot control.
     * 
     *          Typical stored modes:
     *          - LOITER: Pilot was hovering, return to position hold
     *          - AUTO: Mission was running, resume mission
     *          - GUIDED: External control was active, restore GCS control
     *          - STABILIZE/ALT_HOLD: Manual flight, return control to pilot
     * 
     * @note Default value RTL provides conservative fallback if mode not properly saved
     * @note Updated by check_flightmode() when entering avoidance
     * @note Read by handle_recovery() when exiting avoidance
     * 
     * Source: ArduCopter/avoidance_adsb.h:48
     */
    Mode::Number prev_control_mode = Mode::Number::RTL;
};

#endif  // AP_ADSB_AVOIDANCE_ENABLED
