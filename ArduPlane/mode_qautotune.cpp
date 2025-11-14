/**
 * @file mode_qautotune.cpp
 * @brief QuadPlane Autotune flight mode implementation for automated PID tuning
 * 
 * @details This file implements the QAUTOTUNE flight mode, which provides automated
 *          PID controller tuning for QuadPlane VTOL aircraft. The mode uses systematic
 *          test maneuvers to determine optimal PID gains for attitude control loops.
 *          
 *          QAUTOTUNE mode leverages the quadplane.qautotune subsystem to perform:
 *          - Automated test sequences (pitch, roll, yaw axes)
 *          - Response measurement and analysis
 *          - Real-time PID parameter optimization
 *          - Safe parameter testing with automatic reversion on failure
 *          
 *          During autotuning:
 *          - Vehicle maintains VTOL hover using multirotor motors
 *          - Fixed-wing control surfaces are stabilized (roll/pitch)
 *          - Rudder is centered to isolate quadplane control response
 *          - Tailsitter aircraft receive special handling during transitions
 *          
 *          The mode is only available when QAUTOTUNE_ENABLED is defined at compile time.
 * 
 * @note This is a safety-critical flight mode requiring sufficient altitude and space
 * @warning Autotuning induces oscillations - ensure adequate clearance from obstacles
 * 
 * @see QAutoTune class for tuning algorithm implementation
 * @see ModeQStabilize for baseline VTOL stabilization
 * 
 * Source: ArduPlane/mode_qautotune.cpp
 */

#include "mode.h"
#include "Plane.h"

#include "qautotune.h"

#if QAUTOTUNE_ENABLED

/**
 * @brief Enter QAUTOTUNE mode and initialize automated tuning system
 * 
 * @details This method is called when the pilot switches into QAUTOTUNE mode.
 *          It initializes the quadplane autotune subsystem which prepares the
 *          tuning state machine, resets test sequence counters, and validates
 *          that the vehicle is ready for autotuning.
 *          
 *          Pre-entry conditions checked by qautotune.init():
 *          - Vehicle is in VTOL mode (not fixed-wing flight)
 *          - Sufficient altitude for safe tuning maneuvers
 *          - No critical failsafes active
 *          - IMU and attitude estimation healthy
 *          
 *          If initialization fails, the mode entry is rejected and the vehicle
 *          remains in its previous flight mode.
 * 
 * @return true if autotune initialization successful and mode entry allowed
 * @return false if autotune cannot initialize or QAUTOTUNE_ENABLED not defined
 * 
 * @note Called automatically by mode switching logic
 * @warning Autotuning should only be attempted in safe, open areas with adequate altitude
 * 
 * @see QAutoTune::init() for initialization logic and safety checks
 */
bool ModeQAutotune::_enter()
{
#if QAUTOTUNE_ENABLED
    return quadplane.qautotune.init();
#else
    return false;
#endif
}

/**
 * @brief Update QAUTOTUNE mode target attitude and navigation
 * 
 * @details This method is called at the scheduler update rate to compute target
 *          attitude, position, and velocity for the current flight phase. For
 *          QAUTOTUNE mode, all attitude target computation is delegated to
 *          QSTABILIZE mode since autotuning uses stabilize flight as the baseline.
 *          
 *          The delegation to QSTABILIZE ensures:
 *          - Pilot stick inputs are processed for roll/pitch/yaw/throttle
 *          - Target attitude is set based on stick deflection and rate limits
 *          - Hover throttle is maintained for altitude control
 *          - Rate-controlled stabilization mode behavior is active
 *          
 *          The actual autotune test maneuvers are injected during the run() phase,
 *          which modifies the attitude targets computed here.
 * 
 * @note Called at main scheduler update rate (typically 50-400Hz depending on vehicle)
 * @note This is separate from run() which executes motor output and control loops
 * 
 * @see ModeQStabilize::update() for attitude target computation
 * @see run() for autotune test sequence execution
 */
void ModeQAutotune::update()
{
    plane.mode_qstabilize.update();
}

/**
 * @brief Execute QAUTOTUNE mode control loops and motor output
 * 
 * @details This method is called at the main loop rate to execute attitude control
 *          and generate motor/servo outputs. It integrates the quadplane autotune
 *          test sequence with fixed-wing control surface stabilization.
 *          
 *          Execution sequence:
 *          1. Check for tailsitter VTOL transition - use fixed-wing control if transitioning
 *          2. Run quadplane autotune test sequence (injects attitude rate commands)
 *          3. Stabilize fixed-wing roll/pitch surfaces to assist VTOL hover
 *          4. Center rudder to prevent yaw interference from fixed-wing aerodynamics
 *          
 *          Tailsitter Special Handling:
 *          During the fixed-wing pull-up phase of a VTOL transition, tailsitters
 *          temporarily use fixed-wing controllers (Mode::run()) instead of VTOL
 *          controllers. This provides smooth aerodynamic control during the transition
 *          when the vehicle is still moving fast enough for control surface effectiveness.
 *          
 *          Autotune Test Sequence:
 *          The quadplane.qautotune.run() method performs the tuning state machine:
 *          - Injects test inputs (step responses, frequency sweeps)
 *          - Measures attitude response (rate, overshoot, settling time)
 *          - Calculates optimal PID gains based on response characteristics
 *          - Updates parameters in real-time and validates stability
 *          - Reverts parameters automatically if instability detected
 *          
 *          Fixed-Wing Surface Stabilization:
 *          Even during VTOL hover, fixed-wing surfaces assist with:
 *          - Roll stability (ailerons provide additional roll authority)
 *          - Pitch stability (elevator assists with pitch control)
 *          - Reduced multirotor motor load through aerodynamic assistance
 *          
 *          The rudder is centered (0.0) to isolate quadplane yaw control from
 *          fixed-wing aerodynamic yaw coupling, ensuring clean yaw axis tuning.
 * 
 * @note Called at main loop rate (typically 400Hz for quadplanes)
 * @note Autotune test maneuvers may induce significant vehicle oscillations
 * 
 * @warning Tailsitter transition handling is critical for safe VTOL to FW transitions
 * @warning Fixed-wing surface stabilization must not interfere with autotune measurements
 * 
 * @see QAutoTune::run() for tuning algorithm and test sequence details
 * @see Plane::stabilize_roll() for aileron stabilization during VTOL
 * @see Plane::stabilize_pitch() for elevator stabilization during VTOL
 * @see Mode::run() for fixed-wing control during tailsitter transitions
 */
void ModeQAutotune::run()
{
    const uint32_t now = AP_HAL::millis();
    if (quadplane.tailsitter.in_vtol_transition(now)) {
        // Tailsitters in FW pull up phase of VTOL transition run FW controllers
        Mode::run();
        return;
    }

#if QAUTOTUNE_ENABLED
    quadplane.qautotune.run();
#endif

    // Stabilize with fixed wing surfaces
    plane.stabilize_roll();
    plane.stabilize_pitch();

    // Center rudder
    output_rudder_and_steering(0.0);
}

/**
 * @brief Exit QAUTOTUNE mode and finalize tuning results
 * 
 * @details This method is called when the pilot switches out of QAUTOTUNE mode
 *          or when the autotune sequence completes. It stops the autotune process
 *          and performs cleanup operations.
 *          
 *          Exit operations performed by qautotune.stop():
 *          - Halts any in-progress tuning test sequences
 *          - Saves successfully tuned parameters to persistent storage
 *          - Logs final tuning results for post-flight analysis
 *          - Resets tuning state machine to idle state
 *          - Reverts to original parameters if tuning was unsuccessful
 *          
 *          If the autotune successfully completed all axes (roll, pitch, yaw),
 *          the new PID gains are retained. If tuning was interrupted or failed
 *          safety checks, the original parameters are restored.
 * 
 * @note Called automatically by mode switching logic
 * @note Parameter changes are logged to dataflash for verification and analysis
 * 
 * @warning Ensure sufficient time for parameter save to complete before power cycling
 * 
 * @see QAutoTune::stop() for cleanup logic and parameter finalization
 */
void ModeQAutotune::_exit()
{
#if QAUTOTUNE_ENABLED
    plane.quadplane.qautotune.stop();
#endif
}

#endif
