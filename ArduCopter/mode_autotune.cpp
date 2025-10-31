/**
 * @file mode_autotune.cpp
 * @brief AutoTune flight mode implementation for ArduCopter
 * 
 * @details AutoTune is an automated PID tuning mode that performs frequency sweep
 *          tests on each axis (roll, pitch, yaw) to measure vehicle response 
 *          characteristics and automatically calculate optimal rate controller 
 *          PID gains. This mode wraps the AC_AutoTune library which implements 
 *          the core tuning algorithms.
 *          
 *          The AutoTune process:
 *          1. Commands small test maneuvers (twitches) on one axis at a time
 *          2. Measures vehicle angular rate and attitude response using IMU data
 *          3. Analyzes frequency response to determine optimal gains
 *          4. Applies calculated gains and verifies stability
 *          5. Moves to next axis in sequence: roll → pitch → yaw
 *          6. Saves tuned parameters to persistent storage on completion
 *          
 *          Safety features:
 *          - Pilot can take control at any time by moving sticks
 *          - Automatically stops if vehicle drifts beyond safe limits
 *          - Requires stable hover capability before starting
 *          - Can be exited immediately by mode change
 *          - Only operates from specific safe flight modes
 *          
 *          Requirements for successful tuning:
 *          - Vehicle must be in stable hover
 *          - Good GPS position hold or optical flow (for position control)
 *          - Low wind conditions (< 3 m/s recommended)
 *          - Sufficient battery (> 50% recommended)
 *          - Vehicle mechanically sound with balanced props
 *          
 * @note This mode requires AUTOTUNE_ENABLED=1 in build configuration
 * @warning AutoTune commands aggressive maneuvers. Always test in safe area with
 *          sufficient altitude (minimum 10m recommended).
 * 
 * Source: ArduCopter/mode_autotune.cpp
 */
#include "Copter.h"

/*
  autotune mode is a wrapper around the AC_AutoTune library
 */

#if AUTOTUNE_ENABLED

/**
 * @brief Initialize AutoTune mode and validate preconditions for safe tuning
 * 
 * @details This function performs comprehensive safety checks before allowing
 *          AutoTune to begin. It verifies the vehicle is in a suitable state
 *          for automated PID tuning operations. The checks ensure:
 *          
 *          1. Flight mode compatibility - Only allows AutoTune from modes that
 *             provide stable manual or position control (Stabilize, AltHold,
 *             PosHold, Loiter). This prevents AutoTune from unstable states.
 *             
 *          2. Throttle validation - Ensures pilot has throttle above zero,
 *             indicating intention to maintain flight during tuning.
 *             
 *          3. Flight status - Confirms vehicle is armed, auto-armed (safety
 *             check passed), and airborne (not landed). AutoTune requires
 *             the vehicle to be flying to perform rate response tests.
 *             
 *          4. Position control selection - Determines whether to use position
 *             hold during tuning based on current flight mode. Position hold
 *             is enabled for LOITER and POSHOLD modes to minimize drift during
 *             tuning maneuvers. For other modes (Stabilize, AltHold), pilot
 *             stick inputs maintain position.
 *          
 *          The function then calls init_internals() which sets up the AC_AutoTune
 *          library with references to the attitude controller, position controller,
 *          and AHRS for vehicle state estimation during tuning tests.
 * 
 * @return true if all preconditions met and AutoTune initialized successfully
 * @return false if any safety check fails or vehicle not ready for tuning
 * 
 * @note AutoTune will not start if vehicle is on the ground or throttle is zero
 * @warning Ensure adequate altitude (>10m) and clear airspace before initiating
 * 
 * @see AC_AutoTune::init_internals()
 * @see Copter::Mode::allows_autotune()
 */
bool AutoTune::init()
{
    // only allow AutoTune from some flight modes, for example Stabilize, AltHold,  PosHold or Loiter modes
    if (!copter.flightmode->allows_autotune()) {
        return false;
    }

    // ensure throttle is above zero
    if (copter.ap.throttle_zero) {
        return false;
    }

    // ensure we are flying
    if (!copter.motors->armed() || !copter.ap.auto_armed || copter.ap.land_complete) {
        return false;
    }

    // use position hold while tuning if we were in QLOITER
    bool position_hold = (copter.flightmode->mode_number() == Mode::Number::LOITER || copter.flightmode->mode_number() == Mode::Number::POSHOLD);

    return init_internals(position_hold,
                          copter.attitude_control,
                          copter.pos_control,
                          copter.ahrs_view);
}

/**
 * @brief Main AutoTune loop executed at scheduler rate (typically 400Hz)
 * 
 * @details This function is called continuously while AutoTune mode is active.
 *          It handles safety monitoring and delegates the core tuning operations
 *          to the AC_AutoTune library. The execution flow:
 *          
 *          1. Simple mode transform - Applies SIMPLE mode coordinate frame
 *             transformation to pilot stick inputs if SIMPLE mode is active.
 *             This allows pilot to maintain orientation awareness during tuning.
 *             
 *          2. Landing detection and disarm - Monitors landing detector and motor
 *             spool state. Automatically disarms if vehicle has landed and motors
 *             are at ground idle, preventing unexpected motor commands on ground.
 *             
 *          3. Ground safety handling - If vehicle is detected as landed (land_complete),
 *             enters safe ground handling mode which zeros throttle and prevents
 *             further tuning operations until vehicle becomes airborne again.
 *             
 *          4. AC_AutoTune execution - Calls the core AutoTune library run() method
 *             which performs the actual tuning operations:
 *             - Commands test maneuvers (frequency sweeps) on current axis
 *             - Measures vehicle angular rate response from gyros
 *             - Analyzes frequency response characteristics
 *             - Calculates optimal PID gains using system identification
 *             - Applies and verifies new gains
 *             - Progresses through tuning sequence: roll → pitch → yaw
 *             - Monitors pilot input for manual takeover
 *             - Maintains position hold if configured
 *             
 *          The AC_AutoTune library implements the frequency sweep methodology:
 *          - Applies sinusoidal torque commands at varying frequencies
 *          - Records amplitude and phase of vehicle response
 *          - Constructs Bode plot of system frequency response
 *          - Identifies critical frequencies and gain/phase margins
 *          - Computes PID gains for target phase margin (typically 60°)
 *          - Validates stability with step response tests
 *          
 *          Safety features during execution:
 *          - Pilot stick input above threshold immediately stops tuning
 *          - Position error limits prevent excessive drift
 *          - Attitude error limits abort tuning if vehicle unstable
 *          - Timeout limits prevent infinite tuning attempts
 * 
 * @note Called at main loop rate - typically 400Hz for multicopters
 * @warning This function commands aggressive vehicle movements during tuning tests
 * 
 * @see AC_AutoTune::run() - Core tuning algorithm implementation
 * @see Copter::update_simple_mode()
 */
void AutoTune::run()
{
    // apply SIMPLE mode transform to pilot inputs
    copter.update_simple_mode();

    // disarm when the landing detector says we've landed and spool state is ground idle
    if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
        copter.arming.disarm(AP_Arming::Method::LANDED);
    }

    // if not armed set throttle to zero and exit immediately
    if (copter.ap.land_complete) {
        copter.flightmode->make_safe_ground_handling();
        return;
    }

    // run autotune mode
    AC_AutoTune::run();
}


/**
 * @brief Get pilot's desired climb rate from throttle stick with avoidance adjustments
 * 
 * @details This function retrieves the pilot's commanded vertical velocity from
 *          the throttle stick input and applies object avoidance adjustments if
 *          enabled. During AutoTune, the pilot maintains altitude control while
 *          the AutoTune algorithm commands roll, pitch, and yaw movements.
 *          
 *          Processing steps:
 *          1. Read throttle stick position and convert to climb rate in cm/s
 *             using configured expo curves and maximum climb rates
 *          2. Apply object avoidance system adjustments if proximity sensors
 *             detect obstacles above or below the vehicle
 *          3. Return final climb rate command to position controller
 *          
 *          The avoidance system may reduce climb rate to zero or command descent
 *          if obstacles are detected in the flight path, providing additional
 *          safety during AutoTune operations in confined spaces.
 * 
 * @return float Desired climb rate in cm/s (positive=up, negative=down)
 *               Range typically -250 to +250 cm/s depending on parameters
 * 
 * @note Called by AC_AutoTune position controller during tuning maneuvers
 * @note Climb rate is in NED frame (negative = climb, positive = descend) internally
 *       but returned as positive-up convention
 * 
 * @see Copter::get_pilot_desired_climb_rate()
 * @see ModeAutoTune::get_avoidance_adjusted_climbrate_cms()
 */
float AutoTune::get_pilot_desired_climb_rate_cms(void) const
{
    float target_climb_rate_cms = copter.get_pilot_desired_climb_rate();

    // get avoidance adjusted climb rate
    target_climb_rate_cms = copter.mode_autotune.get_avoidance_adjusted_climbrate_cms(target_climb_rate_cms);

    return target_climb_rate_cms;
}

/**
 * @brief Get pilot's desired roll/pitch angles and yaw rate from stick inputs
 * 
 * @details This function reads pilot stick inputs and converts them to desired
 *          roll and pitch lean angles and yaw rotation rate. During AutoTune,
 *          pilot inputs are monitored to detect manual takeover. If pilot moves
 *          sticks beyond a small threshold, AutoTune pauses tuning and returns
 *          control to the pilot, allowing immediate intervention if needed.
 *          
 *          Input processing:
 *          - Roll/Pitch sticks: Converted to desired lean angles in radians
 *            with respect to maximum allowed lean angle limits. Uses separate
 *            limits for normal flight vs altitude hold modes.
 *          - Yaw stick: Converted to desired yaw rotation rate in rad/s using
 *            configured yaw rate limits and expo curves
 *            
 *          The function respects two different lean angle limits:
 *          - lean_angle_max: Maximum lean for aggressive flight
 *          - althold_lean_angle_max: Reduced lean limit for altitude hold modes
 *            to maintain better vertical position control during AutoTune
 * 
 * @param[out] des_roll_rad Desired roll lean angle in radians (body frame)
 *                          Positive = lean right, range ±lean_angle_max
 * @param[out] des_pitch_rad Desired pitch lean angle in radians (body frame)
 *                           Positive = lean forward, range ±lean_angle_max
 * @param[out] des_yaw_rate_rads Desired yaw rotation rate in rad/s (body frame)
 *                               Positive = rotate right, range ±max_yaw_rate
 * 
 * @note All outputs are in body frame coordinates
 * @note AutoTune detects pilot intervention by monitoring these values for
 *       changes above threshold (typically 5° for angles, 10°/s for yaw rate)
 * 
 * @see ModeAutoTune::get_pilot_desired_lean_angles_rad()
 * @see ModeAutoTune::get_pilot_desired_yaw_rate_rads()
 * @see AC_AutoTune::run() - Uses these values to detect pilot takeover
 */
void AutoTune::get_pilot_desired_rp_yrate_rad(float &des_roll_rad, float &des_pitch_rad, float &des_yaw_rate_rads)
{
    copter.mode_autotune.get_pilot_desired_lean_angles_rad(des_roll_rad, des_pitch_rad, attitude_control->lean_angle_max_rad(),
                                                       copter.attitude_control->get_althold_lean_angle_max_rad());
    des_yaw_rate_rads = copter.mode_autotune.get_pilot_desired_yaw_rate_rads();
}

/**
 * @brief Configure vertical velocity and acceleration limits for position controller
 * 
 * @details This function initializes the altitude position controller with velocity
 *          and acceleration limits for the vertical (Z) axis. It is called during
 *          AutoTune initialization to ensure the position controller uses appropriate
 *          limits for maintaining altitude during tuning maneuvers.
 *          
 *          Sets two types of limits:
 *          
 *          1. Maximum speed/accel limits - Applied to pilot commanded climb rates
 *             to prevent excessive vertical velocity that could interfere with
 *             AutoTune's horizontal axis testing. Uses configured parameters:
 *             - pilot_speed_dn: Maximum descent rate (m/s)
 *             - pilot_speed_up: Maximum climb rate (m/s)
 *             - pilot_accel_z: Maximum vertical acceleration (cm/s²)
 *             
 *          2. Correction speed/accel limits - Applied to position controller's
 *             automatic altitude corrections to maintain target altitude. Uses
 *             same limits as pilot commands to ensure consistent behavior.
 *             
 *          The 'U' notation indicates "up" in NED frame (negative Z axis), so
 *          negative values represent descent and positive values represent climb.
 *          This follows the standard NED (North-East-Down) convention used
 *          throughout the ArduPilot position control system.
 * 
 * @note Called once during AutoTune initialization via init_internals()
 * @note Units: speeds in cm/s, accelerations in cm/s²
 * @note NED frame convention: U-axis is vertical, positive down
 * 
 * @see AC_PosControl::set_max_speed_accel_U_cm()
 * @see AC_PosControl::set_correction_speed_accel_U_cmss()
 */
void AutoTune::init_z_limits()
{
    // set vertical speed and acceleration limits
    copter.pos_control->set_max_speed_accel_U_cm(-copter.get_pilot_speed_dn(), copter.g.pilot_speed_up, copter.g.pilot_accel_z);
    copter.pos_control->set_correction_speed_accel_U_cmss(-copter.get_pilot_speed_dn(), copter.g.pilot_speed_up, copter.g.pilot_accel_z);
}

#if HAL_LOGGING_ENABLED
/**
 * @brief Log current PID controller information for all three axes to dataflash
 * 
 * @details This function logs detailed PID controller state for roll, pitch, and
 *          yaw rate controllers to the onboard dataflash for post-flight analysis.
 *          During AutoTune, these logs are critical for:
 *          
 *          - Verifying tuning algorithm performance
 *          - Analyzing frequency response characteristics
 *          - Debugging unexpected tuning results
 *          - Validating calculated PID gains
 *          - Comparing before/after tuning performance
 *          
 *          For each axis (roll, pitch, yaw), logs the following PID information:
 *          - Target: Desired angular rate (deg/s or rad/s)
 *          - Actual: Measured angular rate from gyros
 *          - Error: Target - Actual
 *          - P: Proportional term output
 *          - I: Integral term output
 *          - D: Derivative term output
 *          - FF: Feedforward term output
 *          - Total: Sum of all terms (motor command contribution)
 *          
 *          Log messages are written as:
 *          - LOG_PIDR_MSG: Roll rate PID data
 *          - LOG_PIDP_MSG: Pitch rate PID data
 *          - LOG_PIDY_MSG: Yaw rate PID data
 *          
 *          These logs can be analyzed with MAVExplorer or other log analysis
 *          tools to evaluate AutoTune performance and manually fine-tune if needed.
 * 
 * @note Only compiled if HAL_LOGGING_ENABLED=1
 * @note Called by AC_AutoTune at regular intervals during tuning process
 * @note High logging rate during AutoTune may fill dataflash quickly on small boards
 * 
 * @see AP_Logger::Write_PID()
 * @see AC_AttitudeControl::get_rate_roll_pid()
 */
void AutoTune::log_pids()
{
    copter.logger.Write_PID(LOG_PIDR_MSG, copter.attitude_control->get_rate_roll_pid().get_pid_info());
    copter.logger.Write_PID(LOG_PIDP_MSG, copter.attitude_control->get_rate_pitch_pid().get_pid_info());
    copter.logger.Write_PID(LOG_PIDY_MSG, copter.attitude_control->get_rate_yaw_pid().get_pid_info());
}
#endif

/**
 * @brief Verify vehicle has reliable position estimate for AutoTune operations
 * 
 * @details This function checks whether the vehicle has a valid and accurate
 *          position estimate, which is required for position hold mode during
 *          AutoTune. A good position estimate prevents the vehicle from drifting
 *          excessively during tuning maneuvers, ensuring test results are not
 *          contaminated by position controller corrections.
 *          
 *          Position estimate validity is determined by checking:
 *          - GPS: 3D fix with sufficient satellites and HDOP < threshold
 *          - Optical flow: Valid flow data with sufficient quality
 *          - Visual odometry: Recent valid position updates
 *          - Indoor positioning: Beacon or UWB position data
 *          
 *          If position estimate is not available or unreliable, AutoTune can
 *          still operate but will not use position hold mode. The pilot must
 *          manually maintain position with stick inputs, which may reduce
 *          tuning accuracy due to pilot-induced disturbances.
 *          
 *          Called by AC_AutoTune library to determine whether to enable
 *          position hold mode or fall back to manual position control.
 * 
 * @return true if vehicle has reliable position estimate (GPS, optical flow, etc.)
 * @return false if no position estimate available or estimate unreliable
 * 
 * @note AutoTune can operate without position hold but results may be suboptimal
 * @note For best results, fly in GPS mode with 3D lock and HDOP < 1.5
 * 
 * @see Copter::position_ok() - Checks all available position sources
 * @see AC_AutoTune::run() - Uses this to enable/disable position hold
 */
bool AutoTune::position_ok()
{
    return copter.position_ok();
}

/**
 * @brief Initialize ModeAutoTune flight mode when pilot selects AutoTune
 * 
 * @details This is the entry point called by the flight mode manager when the
 *          pilot switches to AutoTune mode via transmitter mode switch or
 *          ground control station command. It delegates to the AutoTune object's
 *          init() method which performs comprehensive safety checks and
 *          initializes the AC_AutoTune library for tuning operations.
 *          
 *          The ModeAutoTune class acts as a thin wrapper implementing the
 *          standard Copter flight mode interface, while the AutoTune class
 *          (derived from AC_AutoTune) contains the actual tuning logic.
 *          
 *          If initialization succeeds, the mode switch completes and AutoTune
 *          begins commanding test maneuvers. If initialization fails (e.g.,
 *          vehicle on ground, invalid starting mode), the mode switch is
 *          rejected and vehicle remains in previous flight mode.
 * 
 * @param[in] ignore_checks If true, skip some safety checks (not used for AutoTune,
 *                          included for interface compatibility)
 * 
 * @return true if AutoTune initialized successfully and mode switch completed
 * @return false if preconditions not met and mode switch rejected
 * 
 * @note Called once when pilot switches to AutoTune mode
 * @warning Vehicle must be flying in stable hover before switching to AutoTune
 * 
 * @see AutoTune::init() - Actual initialization and safety checks
 */
bool ModeAutoTune::init(bool ignore_checks)
{
    return autotune.init();
}

/**
 * @brief Execute AutoTune main loop at scheduler rate
 * 
 * @details This is the main run function called continuously by the flight mode
 *          manager while AutoTune mode is active. It delegates to the AutoTune
 *          object's run() method which handles safety monitoring and executes
 *          the core tuning algorithm via the AC_AutoTune library.
 *          
 *          This function is called at the main loop rate (typically 400Hz for
 *          multirotors) to provide consistent timing for control algorithms
 *          and frequency response measurements.
 * 
 * @note Called at main loop rate (typically 400Hz) while AutoTune mode active
 * 
 * @see AutoTune::run() - Core tuning algorithm execution
 */
void ModeAutoTune::run()
{
    autotune.run();
}

/**
 * @brief Clean up and finalize AutoTune when exiting mode
 * 
 * @details This function is called when the pilot switches out of AutoTune mode
 *          to another flight mode, or when AutoTune completes successfully. It
 *          delegates to the AutoTune object's stop() method which:
 *          
 *          - Stops any in-progress tuning maneuvers
 *          - Saves tuned parameters to persistent storage if tuning completed
 *          - Restores original PID gains if tuning was incomplete or failed
 *          - Cleans up internal state for next AutoTune session
 *          
 *          Parameter saving behavior:
 *          - If all axes tuned successfully: Saves new PID gains permanently
 *          - If tuning incomplete: Restores original gains, no save
 *          - If tuning failed safety checks: Restores original gains
 *          
 *          After exit, the vehicle transitions to the newly selected flight mode
 *          with appropriate PID gains active (either new tuned gains or original).
 * 
 * @note Called once when exiting AutoTune mode
 * @note May take several seconds if saving parameters to EEPROM/flash
 * 
 * @see AC_AutoTune::stop() - Parameter save logic and cleanup
 */
void ModeAutoTune::exit()
{
    autotune.stop();
}

#endif  // AUTOTUNE_ENABLED
