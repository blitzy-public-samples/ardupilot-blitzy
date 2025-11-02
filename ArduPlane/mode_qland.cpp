/**
 * @file mode_qland.cpp
 * @brief QuadPlane QLAND (VTOL Landing) mode implementation
 * 
 * @details This file implements the QLAND flight mode for QuadPlane VTOL aircraft,
 *          providing automated vertical landing capability. QLAND mode orchestrates
 *          a controlled descent from current altitude to touchdown, managing landing
 *          phases including descent, final approach, and safe disarm sequence.
 *          
 *          Landing Phases:
 *          - QPOS_LAND_DESCEND: Controlled vertical descent maintaining horizontal position
 *          - Final approach: Reduced descent rate near ground with touchdown detection
 *          - Ground contact: Landing detection triggers disarm sequence
 *          
 *          The mode delegates primary control to QLOITER for position hold during descent,
 *          while managing landing-specific state including throttle management, gear
 *          deployment, and touchdown detection.
 *          
 *          Safety Features:
 *          - Automatic landing gear deployment when equipped
 *          - Touchdown detection via multiple sensors (contact switches, current monitoring)
 *          - Altitude monitoring with rangefinder integration
 *          - Position hold during descent prevents horizontal drift
 * 
 * @note This mode is only available when HAL_QUADPLANE_ENABLED is defined
 * @warning Landing operations are safety-critical; touchdown detection must be reliable
 *          to prevent motor disarm before ground contact or excessive ground contact time
 * 
 * @see ModeQLoiter - Provides position control during landing
 * @see QuadPlane::QPOS_LAND_DESCEND - Landing state machine
 * 
 * Source: ArduPlane/mode_qland.cpp
 */

#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

/**
 * @brief Enter QLAND mode and initialize landing sequence
 * 
 * @details Initializes QuadPlane landing mode by setting up position control,
 *          configuring landing state machine, and preparing touchdown detection.
 *          This method performs the following initialization sequence:
 *          
 *          1. Initialize QLOITER mode for position hold control
 *          2. Disable throttle wait to allow immediate descent
 *          3. Capture current position as landing target
 *          4. Set landing state to QPOS_LAND_DESCEND
 *          5. Record initial altitude AGL for descent monitoring
 *          6. Reset touchdown detection timers
 *          7. Deploy landing gear if equipped
 *          
 *          The landing sequence starts with controlled descent from current altitude,
 *          maintaining horizontal position over the landing point. Touchdown detection
 *          monitors multiple indicators including landing gear contact switches and
 *          motor current to identify ground contact.
 * 
 * @return true always (mode entry cannot fail)
 * 
 * @note Called automatically by flight mode switching logic
 * @note Landing gear deployment is conditional on AP_LANDINGGEAR_ENABLED
 * 
 * @warning Ensure sufficient altitude for safe landing before entering this mode
 * @warning Landing will occur at current horizontal position - verify landing site is clear
 * 
 * @see ModeQLoiter::_enter() - Position control initialization
 * @see QuadPlane::setup_target_position() - Captures current position as target
 * @see QuadPlane::QPOS_LAND_DESCEND - Initial landing state
 */
bool ModeQLand::_enter()
{
    // Initialize QLOITER mode for position hold during descent
    plane.mode_qloiter._enter();
    
    // Allow immediate throttle control without waiting
    quadplane.throttle_wait = false;
    
    // Capture current position as landing target point
    quadplane.setup_target_position();
    
    // Set position control state to landing descent phase
    poscontrol.set_state(QuadPlane::QPOS_LAND_DESCEND);
    
    // Record initial altitude above ground level (AGL) for descent monitoring
    // Uses rangefinder when available for accurate ground clearance
    quadplane.last_land_final_agl = plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING);
    
    // Reset touchdown detection timing counters
    quadplane.landing_detect.lower_limit_start_ms = 0;  // Motor lower limit detection
    quadplane.landing_detect.land_start_ms = 0;          // Ground contact detection
    
#if AP_LANDINGGEAR_ENABLED
    // Deploy landing gear for touchdown if equipped
    plane.g2.landing_gear.deploy_for_landing();
#endif

    return true;
}

/**
 * @brief Update QLAND mode at main loop rate
 * 
 * @details Performs high-rate attitude updates by delegating to QSTABILIZE mode.
 *          This method is called at the main loop rate (typically 400Hz for QuadPlane)
 *          to maintain responsive attitude control during landing descent.
 *          
 *          The update() method handles fast inner-loop attitude stabilization while
 *          run() manages the slower outer-loop position and descent control. This
 *          separation ensures stable attitude control throughout the landing sequence.
 * 
 * @note Called at main loop rate (typically 400Hz)
 * @note Attitude control updates occur more frequently than position control
 * 
 * @see ModeQStabilize::update() - Attitude stabilization implementation
 * @see ModeQLand::run() - Outer loop position and landing control
 */
void ModeQLand::update()
{
    // Delegate to QSTABILIZE for high-rate attitude control
    plane.mode_qstabilize.update();
}

/**
 * @brief Execute QLAND main control loop for landing descent and touchdown
 * 
 * @details Orchestrates the QuadPlane landing sequence by delegating primary control
 *          to QLOITER mode while managing landing-specific state transitions. This
 *          method runs at the outer loop rate (typically 50Hz) to control descent,
 *          monitor touchdown, and manage landing phases.
 *          
 *          Landing Sequence:
 *          
 *          1. **Descent Phase** (QPOS_LAND_DESCEND):
 *             - Controlled vertical descent at configured landing speed
 *             - QLOITER maintains horizontal position over landing point
 *             - Continuous altitude monitoring via barometer and rangefinder
 *             - Descent rate adjusted based on altitude above ground
 *          
 *          2. **Final Approach**:
 *             - Reduced descent rate when approaching ground (< 2m AGL typically)
 *             - Enhanced touchdown detection monitoring activated
 *             - Landing gear contact switches monitored if equipped
 *             - Motor current monitoring for ground contact indication
 *          
 *          3. **Touchdown Detection**:
 *             - Multiple detection methods for reliability:
 *               * Landing gear contact switches (if equipped)
 *               * Motor throttle at lower limit for extended duration
 *               * Barometer/rangefinder altitude stable near zero
 *             - Detection thresholds prevent false positives from turbulence
 *          
 *          4. **Ground Phase**:
 *             - Motors maintain minimum thrust briefly to settle vehicle
 *             - Touchdown confirmation period prevents premature disarm
 *             - Automatic disarm sequence initiated after stable ground contact
 *          
 *          Position Control:
 *          - QLOITER provides GPS position hold during entire landing sequence
 *          - Wind compensation maintains position over landing point
 *          - Position errors corrected without interrupting descent
 *          
 *          Safety Checks:
 *          - Altitude monitoring prevents excessive descent rates
 *          - Position deviation from landing point monitored
 *          - Failsafe triggers (GPS loss, battery failsafe) cause abort to QRTL
 *          - Touchdown detection requires sustained ground contact signal
 * 
 * @note Called at outer loop rate (typically 50Hz)
 * @note Actual landing control implemented in QLOITER and QuadPlane position control
 * @note Disarm sequence automatic after confirmed touchdown
 * 
 * @warning Landing operations are safety-critical - touchdown detection failure could
 *          result in motor disarm before ground contact or prolonged ground contact
 * @warning Ensure landing site is level and clear - mode does not perform obstacle avoidance
 * @warning GPS position hold required - GPS loss during landing may trigger failsafe
 * 
 * @see ModeQLoiter::run() - Position hold and descent control implementation
 * @see QuadPlane::poscontrol_run() - Landing state machine execution
 * @see QuadPlane::check_land_complete() - Touchdown detection logic
 */
void ModeQLand::run()
{
    /*
     * Delegate main control to QLOITER mode which provides:
     * - Horizontal position hold over landing point
     * - Controlled vertical descent at configured landing speed
     * - Touchdown detection and automatic disarm sequence
     * 
     * QLOITER handles the landing state machine transitions and
     * coordinates with QuadPlane position control for descent management.
     */
    plane.mode_qloiter.run();
}

#endif
