/**
 * @file mode_LoiterAltQLand.cpp
 * @brief LOITER_ALT_QLAND mode implementation for altitude-triggered QuadPlane VTOL landing
 * 
 * @details This mode implements an autonomous landing sequence for QuadPlane vehicles that
 *          combines fixed-wing loiter with altitude monitoring to trigger VTOL landing.
 *          
 *          The mode provides a smooth transition from fixed-wing flight to VTOL landing by:
 *          1. Loitering at the target location while descending
 *          2. Continuously monitoring altitude relative to the landing waypoint
 *          3. Automatically transitioning to QLAND mode when reaching the configured VTOL landing altitude
 *          
 *          **Flight Sequence**:
 *          - Aircraft loiters at the specified waypoint (either previous loiter point or current location)
 *          - Descends to the Q_RTL_ALT altitude (configured VTOL transition altitude)
 *          - When altitude target is reached and loiter target is acquired, transitions to QLAND
 *          - QLAND mode completes the vertical landing phase
 *          
 *          **Mode Entry Behavior**:
 *          - If already in VTOL mode: Immediately transitions to QLAND (no fixed-wing phase)
 *          - If in fixed-wing mode: Sets up loiter at current or previous waypoint, descends to VTOL altitude
 *          
 *          **Altitude Reference**:
 *          - Supports both above-home and terrain-following altitude references
 *          - Altitude target determined by Q_RTL_ALT parameter and terrain settings
 *          
 *          **Safety Considerations**:
 *          - Requires valid QuadPlane configuration (HAL_QUADPLANE_ENABLED)
 *          - Altitude monitoring ensures smooth transition without premature VTOL conversion
 *          - Loiter target acquisition required before transition (prevents transition during waypoint approach)
 *          
 *          **Typical Use Cases**:
 *          - Autonomous return-to-launch with VTOL landing
 *          - Precision landing at surveyed locations
 *          - Emergency landing with altitude-based VTOL transition
 *          - Mission-planned landing sequences
 * 
 * @note This mode is only available when HAL_QUADPLANE_ENABLED is defined
 * @warning Incorrect Q_RTL_ALT configuration may cause premature or late VTOL transition
 * 
 * Source: ArduPlane/mode_LoiterAltQLand.cpp
 */

#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

/**
 * @brief Enter LOITER_ALT_QLAND mode and initialize landing sequence
 * 
 * @details This method handles mode entry logic for the altitude-triggered VTOL landing sequence.
 *          It determines the appropriate loiter waypoint and sets up the descent profile to the
 *          VTOL transition altitude.
 *          
 *          **Entry Logic**:
 *          1. Check if vehicle is already in VTOL mode (previous mode or current state)
 *             - If yes: Immediately transition to QLAND mode (skip fixed-wing loiter phase)
 *             - If no: Proceed with fixed-wing loiter setup
 *          
 *          2. Determine loiter waypoint:
 *             - If already established in a loiter (target reached, data valid): Use existing loiter waypoint
 *             - Otherwise: Use current vehicle location as loiter center
 *          
 *          3. Initialize parent ModeLoiter behavior (loiter navigation controller setup)
 *          
 *          4. Configure guided waypoint with altitude target via handle_guided_request()
 *             - Sets altitude to Q_RTL_ALT (VTOL transition altitude)
 *             - Applies terrain-following if enabled and configured for QLAND mode
 *          
 *          5. Perform initial altitude check via switch_qland()
 *             - Transitions to QLAND immediately if already at or below target altitude
 *          
 *          **Mode Transition Behavior**:
 *          - From VTOL modes → QLAND: Immediate transition, no fixed-wing phase
 *          - From fixed-wing modes → LOITER_ALT_QLAND: Descending loiter until altitude reached
 *          
 *          **Waypoint Selection Strategy**:
 *          - Preserves existing loiter: Maintains continuity if already loitering
 *          - Uses current location: Starts new loiter at vehicle position if not in established loiter
 * 
 * @return true Always returns true (mode entry always succeeds)
 * 
 * @note This method is called automatically by the mode switching system
 * @note The immediate QLAND transition for VTOL modes prevents unnecessary mode transitions
 * @warning Ensure QuadPlane is properly configured before using this mode
 * 
 * @see switch_qland() for altitude monitoring and QLAND transition logic
 * @see handle_guided_request() for altitude configuration details
 * @see ModeLoiter::_enter() for base loiter mode initialization
 */
bool ModeLoiterAltQLand::_enter()
{
    if (plane.previous_mode->is_vtol_mode() || plane.quadplane.in_vtol_mode()) {
        plane.set_mode(plane.mode_qland, ModeReason::LOITER_ALT_IN_VTOL);
        return true;
    }

    // If we were already in a loiter then use that waypoint. Else, use the current point
    const bool already_in_a_loiter = plane.nav_controller->reached_loiter_target() && !plane.nav_controller->data_is_stale();
    const Location loiter_wp = already_in_a_loiter ? plane.next_WP_loc : plane.current_loc;

    ModeLoiter::_enter();

    handle_guided_request(loiter_wp);

    switch_qland();

    return true;
}

/**
 * @brief Navigation update for LOITER_ALT_QLAND mode with altitude monitoring
 * 
 * @details This method is called periodically during the loiter descent phase to update navigation
 *          and monitor altitude for QLAND mode transition. It combines altitude-based transition
 *          monitoring with standard loiter navigation behavior.
 *          
 *          **Navigation Sequence** (called at navigation update rate, typically 10-50 Hz):
 *          1. Check current altitude and transition to QLAND if conditions met (switch_qland())
 *          2. Execute standard loiter navigation (ModeLoiter::navigate())
 *          
 *          **Altitude Monitoring**:
 *          - Continuously checks if vehicle has descended to or below Q_RTL_ALT altitude
 *          - Verifies loiter target has been reached (prevents transition during approach)
 *          - Triggers automatic mode change to QLAND when conditions satisfied
 *          
 *          **Loiter Navigation**:
 *          - Maintains circular loiter pattern around configured waypoint
 *          - Controls bank angle and airspeed for coordinated turns
 *          - Manages descent rate to reach target altitude
 *          
 *          **Update Frequency**:
 *          - Called at navigation controller update rate (typically 10-50 Hz)
 *          - Altitude checks performed on every navigation update for responsive transition
 * 
 * @note This method overrides ModeLoiter::navigate() to add altitude monitoring
 * @note The switch_qland() call occurs before navigation to allow immediate transition
 * @warning Mode transition may occur mid-navigation update if altitude reached
 * 
 * @see switch_qland() for altitude check and transition logic
 * @see ModeLoiter::navigate() for base loiter navigation implementation
 */
void ModeLoiterAltQLand::navigate()
{
    switch_qland();

    ModeLoiter::navigate();
}

/**
 * @brief Check altitude conditions and transition to QLAND mode when target altitude reached
 * 
 * @details This method implements the core altitude monitoring and mode transition logic for
 *          LOITER_ALT_QLAND. It verifies that the vehicle has descended to the configured VTOL
 *          landing altitude and that the loiter position is established before triggering the
 *          transition to vertical landing mode.
 *          
 *          **Transition Conditions** (ALL must be true):
 *          1. **Altitude Check**: Vehicle is at or below the target waypoint altitude
 *             - Calculated as vertical distance from current position to next_WP_loc
 *             - Negative distance indicates vehicle is below target (trigger condition)
 *             - get_height_above() returns false if calculation fails (prevents transition)
 *          
 *          2. **Loiter Established**: Navigation controller has reached the loiter target
 *             - Ensures vehicle is circling at the intended location
 *             - Prevents transition while still approaching the loiter waypoint
 *             - Provides stable position before VTOL conversion
 *          
 *          **When Conditions Met**:
 *          - Automatically switches to QLAND mode (QuadPlane vertical landing)
 *          - Sets mode reason to LOITER_ALT_REACHED_QLAND for logging and telemetry
 *          - QLAND mode takes over and completes the vertical descent to ground
 *          
 *          **Altitude Calculation**:
 *          - Uses get_height_above() to compute vertical separation in current altitude frame
 *          - Respects altitude frame of next_WP_loc (ABOVE_HOME or ABOVE_TERRAIN)
 *          - Distance is positive when vehicle is above waypoint, negative when below
 *          
 *          **Safety Logic**:
 *          - Failed altitude calculation (get_height_above returns false): No transition
 *          - Loiter target not reached: No transition
 *          - Conservative approach prevents premature VTOL conversion
 * 
 * @note Called from both _enter() and navigate() to check conditions at entry and during flight
 * @note The is_negative(dist) check triggers when vehicle is at or below target altitude
 * @warning Altitude calculation failure is treated as "not ready" - transition will not occur
 * @warning Ensure Q_RTL_ALT is set appropriately for safe VTOL transition altitude
 * 
 * @see _enter() for initial altitude check at mode entry
 * @see navigate() for periodic altitude monitoring during loiter
 * @see handle_guided_request() for altitude target configuration
 */
void ModeLoiterAltQLand::switch_qland()
{
    ftype dist;
    if ((!plane.current_loc.get_height_above(plane.next_WP_loc, dist) || is_negative(dist)) && plane.nav_controller->reached_loiter_target()) {
        plane.set_mode(plane.mode_qland, ModeReason::LOITER_ALT_REACHED_QLAND);
    }
}

/**
 * @brief Configure loiter waypoint with VTOL transition altitude
 * 
 * @details This method sets up the guided loiter waypoint for the descent phase, configuring
 *          the target altitude based on QuadPlane RTL settings and terrain-following configuration.
 *          The altitude target determines when the vehicle will transition from fixed-wing loiter
 *          to VTOL landing mode.
 *          
 *          **Altitude Configuration Logic**:
 *          
 *          1. **With Terrain Following** (if AP_TERRAIN_AVAILABLE and enabled for QLAND):
 *             - Altitude set to Q_RTL_ALT in ABOVE_TERRAIN frame
 *             - Vehicle maintains constant height above ground regardless of terrain elevation
 *             - Requires valid terrain data from mission planner or SRTM database
 *          
 *          2. **Without Terrain Following** (terrain disabled or unavailable):
 *             - Altitude set to Q_RTL_ALT in ABOVE_HOME frame
 *             - Vehicle descends to fixed altitude relative to home/arming location
 *             - More predictable but doesn't account for terrain variations
 *          
 *          **Parameter Usage**:
 *          - Q_RTL_ALT (quadplane.qrtl_alt): VTOL transition altitude in meters
 *            - Typical values: 15-30 meters for safe VTOL transition
 *            - Must be high enough for safe rotor spool-up and transition
 *            - Must be low enough for efficient landing approach
 *          
 *          **Waypoint Setup**:
 *          - Preserves horizontal position (lat/lon) from input target_loc
 *          - Overrides altitude with Q_RTL_ALT in appropriate frame
 *          - Sends waypoint to guided mode controller via set_guided_WP()
 *          
 *          **Terrain-Following Safety**:
 *          - Terrain mode checked specifically for QLAND (not current mode)
 *          - Ensures terrain settings are appropriate for landing phase
 *          - Falls back to ABOVE_HOME if terrain unavailable
 * 
 * @param[in,out] target_loc Loiter waypoint location (lat/lon preserved, altitude overridden)
 * 
 * @return true Always returns true (waypoint setup always succeeds)
 * 
 * @note The input location's horizontal position is preserved, only altitude is modified
 * @note Terrain following requires valid terrain data and configuration for QLAND mode
 * @warning Ensure Q_RTL_ALT provides sufficient altitude for safe VTOL transition (typically 15-30m)
 * @warning Terrain-following mode requires terrain database - verify coverage for operating area
 * 
 * @see plane.terrain_enabled_in_mode() for terrain configuration check
 * @see Location::set_alt_m() for altitude frame configuration
 * @see plane.set_guided_WP() for waypoint activation
 */
bool ModeLoiterAltQLand::handle_guided_request(Location target_loc)
{
    // setup altitude
#if AP_TERRAIN_AVAILABLE
    if (plane.terrain_enabled_in_mode(Mode::Number::QLAND)) {
        target_loc.set_alt_m(quadplane.qrtl_alt, Location::AltFrame::ABOVE_TERRAIN);
    } else {
        target_loc.set_alt_m(quadplane.qrtl_alt, Location::AltFrame::ABOVE_HOME);
    }
#else
    target_loc.set_alt_m(quadplane.qrtl_alt, Location::AltFrame::ABOVE_HOME);
#endif

    plane.set_guided_WP(target_loc);

    return true;
}

#endif
