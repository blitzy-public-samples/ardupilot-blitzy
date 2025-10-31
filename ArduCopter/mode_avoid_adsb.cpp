#include "Copter.h"

#if AP_ADSB_AVOIDANCE_ENABLED

/**
 * @file mode_avoid_adsb.cpp
 * @brief ADS-B avoidance flight mode implementation for ArduCopter
 * 
 * @details This mode provides automatic RF-based collision avoidance for manned aircraft
 *          equipped with ADS-B transponders. The AVOID_ADSB mode is triggered automatically
 *          by the avoidance system when an ADS-B-equipped aircraft is detected on a collision
 *          course with the vehicle.
 * 
 *          Architecture:
 *          - Mode activation: Automatic when ADS-B threat detected by AP_Avoidance_Copter
 *          - Evasion calculation: AP_Avoidance_Copter computes safe evasion velocity vectors
 *          - Maneuver execution: Delegates velocity control to Guided mode infrastructure
 *          - Mode exit: Returns to previous flight mode after threat clears
 * 
 *          This implementation re-uses GUIDED mode functions but does not interfere with 
 *          the GCS or companion computer's use of guided mode because velocity requests 
 *          arrive from different sources (MAVLink messages for GCS/companion computers 
 *          vs. the AP_Avoidance_Copter class for ADS-B avoidance). Inputs from each source 
 *          are only accepted and processed in the appropriate flight mode, ensuring clean
 *          separation of control authority.
 * 
 * @note This mode requires AP_ADSB_AVOIDANCE_ENABLED feature flag
 * @warning ADS-B avoidance maneuvers take priority over normal mission execution
 * 
 * @see AP_Avoidance_Copter
 * @see ModeGuided
 */

/**
 * @brief Initialize the ADS-B avoidance flight mode
 * 
 * @details This function initializes the AVOID_ADSB mode by delegating to the Guided mode
 *          initialization. Since AVOID_ADSB leverages Guided mode's velocity control
 *          infrastructure, all necessary setup (attitude controllers, position controllers,
 *          navigation state) is handled by ModeGuided::init().
 * 
 *          Mode entry sequence:
 *          1. AP_Avoidance_Copter detects ADS-B collision threat
 *          2. Avoidance system requests mode change to AVOID_ADSB
 *          3. This init() function is called to prepare mode
 *          4. Guided mode infrastructure is initialized for velocity control
 *          5. Mode becomes active and ready to receive evasion velocity commands
 * 
 * @param[in] ignore_checks If true, bypass standard pre-flight mode checks
 *                          (typically false for ADS-B avoidance as safety checks
 *                          should be maintained during emergency maneuvers)
 * 
 * @return true if initialization successful and mode ready for evasion maneuvers,
 *         false if initialization failed (fallback to previous mode)
 * 
 * @note This is called automatically by the mode switching logic when an ADS-B threat
 *       is detected - not invoked directly by pilot or GCS
 * @warning Initialization failure during ADS-B avoidance may leave vehicle on collision
 *          course - failsafe mechanisms should be armed
 * 
 * @see ModeGuided::init()
 * @see AP_Avoidance_Copter::handle_avoidance_vertical()
 * @see AP_Avoidance_Copter::handle_avoidance_horizontal()
 */
bool ModeAvoidADSB::init(const bool ignore_checks)
{
    // Re-use Guided mode initialization - provides velocity control infrastructure
    // needed for executing evasion maneuvers calculated by AP_Avoidance_Copter
    return ModeGuided::init(ignore_checks);
}

/**
 * @brief Set evasion velocity vector for ADS-B collision avoidance
 * 
 * @details This function receives evasion velocity commands from AP_Avoidance_Copter
 *          and forwards them to the Guided mode velocity controller for execution.
 *          The evasion velocities are calculated by the avoidance system to move
 *          the vehicle away from detected ADS-B threats while maintaining stable flight.
 * 
 *          Velocity command flow:
 *          1. AP_Avoidance_Copter calculates safe evasion velocity vector
 *          2. Avoidance system calls this function with calculated velocity
 *          3. Mode validation ensures AVOID_ADSB is currently active
 *          4. Velocity forwarded to Guided mode's velocity controller
 *          5. Attitude and position controllers execute the evasion maneuver
 * 
 *          This approach provides clean separation from GCS/companion computer
 *          guided commands - those are rejected when not in GUIDED mode, while
 *          avoidance commands are rejected when not in AVOID_ADSB mode.
 * 
 * @param[in] velocity_neu Desired evasion velocity vector in North-East-Up frame (NED)
 *                         - North component: Forward/back velocity in m/s
 *                         - East component: Right/left velocity in m/s  
 *                         - Up component: Climb/descent velocity in m/s
 *                         Calculated by AP_Avoidance_Copter based on threat geometry
 * 
 * @return true if velocity command accepted and forwarded to controller,
 *         false if not in AVOID_ADSB mode (command rejected for safety)
 * 
 * @note This function is called repeatedly by AP_Avoidance_Copter (typically at
 *       10-20 Hz) while threat persists, providing continuous evasion guidance
 * @warning Velocity commands are only processed when in AVOID_ADSB mode to prevent
 *          avoidance system from interfering with pilot or GCS control
 * 
 * @see ModeGuided::set_velocity()
 * @see AP_Avoidance_Copter::handle_avoidance_horizontal()
 */
bool ModeAvoidADSB::set_velocity(const Vector3f& velocity_neu)
{
    // Safety check: Only accept avoidance velocity commands when in AVOID_ADSB mode
    // This prevents AP_Avoidance from interfering with normal flight operations
    // or GCS/companion computer control in other modes
    if (copter.flightmode->mode_number() != Mode::Number::AVOID_ADSB) {
        return false;
    }

    // Forward evasion velocity to Guided mode's velocity controller
    // The Guided mode infrastructure handles the conversion from velocity
    // commands to attitude targets and motor outputs
    ModeGuided::set_velocity(velocity_neu);
    return true;
}

/**
 * @brief Execute ADS-B avoidance control loop (called at main loop rate, typically 400Hz)
 * 
 * @details This function runs the AVOID_ADSB mode control loop by delegating to Guided
 *          mode's velocity controller. It is called continuously by the scheduler at the
 *          main loop rate while the vehicle is in AVOID_ADSB mode, providing real-time
 *          execution of evasion maneuvers commanded by AP_Avoidance_Copter.
 * 
 *          Control flow during AVOID_ADSB mode:
 *          1. This run() function called at 400Hz by main scheduler
 *          2. Delegates to ModeGuided::run() for velocity control execution
 *          3. Guided mode converts current velocity target to attitude commands
 *          4. Attitude controller converts attitude targets to motor outputs
 *          5. Vehicle executes evasion maneuver away from ADS-B threat
 *          
 *          Mode exit and return:
 *          - AP_Avoidance_Copter monitors threat status continuously
 *          - When ADS-B threat clears (aircraft passes or moves away)
 *          - Avoidance system automatically switches back to previous flight mode
 *          - Normal mission/operation resumes
 * 
 *          Safety architecture:
 *          This delegation approach provides clean separation of control sources.
 *          While in AVOID_ADSB mode, GCS and companion computer guided mode requests
 *          (position, velocity, attitude commands via MAVLink) are ignored because
 *          the vehicle is not in GUIDED mode. Conversely, avoidance velocity commands
 *          via set_velocity() are only accepted in AVOID_ADSB mode. This prevents
 *          conflicting commands and ensures clear control authority.
 * 
 * @note Called at main loop rate (typically 400Hz on most flight controllers)
 * @note Re-uses Guided mode infrastructure without interfering with normal Guided operation
 * @warning Do not call this function directly - invoked automatically by mode scheduler
 * @warning Evasion maneuvers may be aggressive - ensure adequate clearance margins
 *          are configured in avoidance parameters
 * 
 * @see ModeGuided::run()
 * @see AC_AttitudeControl::input_euler_angle_roll_pitch_euler_rate_yaw()
 * @see AP_Avoidance_Copter::adjust_velocity()
 */
void ModeAvoidADSB::run()
{
    // Execute velocity control loop using Guided mode infrastructure
    // 
    // Safety note: This is safe from interference from GCS and companion computers
    // whose guided mode position and velocity requests will be ignored while the
    // vehicle is not in GUIDED mode. Only AP_Avoidance_Copter's velocity commands
    // via set_velocity() are processed while in AVOID_ADSB mode.
    //
    // The Guided mode run() function:
    // - Reads current velocity target (set by AP_Avoidance_Copter via set_velocity())
    // - Converts velocity target to attitude commands using position controller
    // - Executes attitude control loop to achieve desired evasion velocity
    // - Updates motor outputs to execute the avoidance maneuver
    ModeGuided::run();
}

#endif  // AP_ADSB_AVOIDANCE_ENABLED
