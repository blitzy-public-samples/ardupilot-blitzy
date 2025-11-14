/**
 * @file AP_Arming_Blimp.cpp
 * @brief Blimp-specific arming checks implementation for lighter-than-air vehicles
 * 
 * @details This file implements the arming system for the Blimp (airship/lighter-than-air)
 *          vehicle type in ArduPilot. The arming system performs comprehensive pre-flight
 *          safety checks before allowing motor activation, including:
 *          - Pre-arm sensor validation (IMU, compass, GPS, barometer)
 *          - Parameter validation for failsafe thresholds and motor configuration
 *          - Vehicle-specific checks for lighter-than-air flight characteristics
 *          - Altitude and position estimate validation
 *          - EKF attitude and navigation filter health checks
 *          - Motor and actuator initialization verification
 *          - GCS communication and failsafe state validation
 *          
 *          The arming sequence includes proper initialization of home position,
 *          altitude datum, and bearing references specific to buoyant vehicle operations.
 *          
 *          Disarming includes saving learned compass offsets from EKF and proper
 *          shutdown sequencing for lighter-than-air vehicle safety.
 * 
 * @warning This is safety-critical code. All arming checks must pass before motors
 *          can be armed to prevent unsafe flight conditions. Modifications to arming
 *          logic can directly affect vehicle and operator safety.
 * 
 * @note Inherits from AP_Arming base class and overrides vehicle-specific checks
 *       while calling parent class methods for common validation.
 * 
 * @see AP_Arming base class for common arming functionality
 * @see Blimp/Blimp.h for vehicle-specific state and configuration
 * 
 * Source: Blimp/AP_Arming_Blimp.cpp
 */

#include "Blimp.h"

/**
 * @brief Entry point for pre-arm safety checks
 * 
 * @details Executes all configured pre-arm checks and updates the vehicle's pre-arm
 *          status flag. This is the primary interface called by the vehicle to determine
 *          if it is safe to arm motors.
 * 
 * @param[in] display_failure If true, failure messages are sent to GCS for pilot notification
 * 
 * @return true if all pre-arm checks pass, false if any check fails
 * 
 * @note This function updates AP_Notify flags to indicate pre-arm check status to pilot
 * 
 * @see run_pre_arm_checks() for detailed check implementation
 */
bool AP_Arming_Blimp::pre_arm_checks(bool display_failure)
{
    const bool passed = run_pre_arm_checks(display_failure);
    set_pre_arm_check(passed);
    return passed;
}

/**
 * @brief Comprehensive pre-arm check execution for Blimp vehicle
 * 
 * @details Performs the complete suite of pre-arm safety checks required before
 *          allowing motor arming on a lighter-than-air vehicle. The function:
 *          - Verifies system initialization is complete
 *          - Checks for conflicting RC auxiliary switch assignments (motor interlock vs emergency stop)
 *          - Executes parameter validation for failsafe thresholds
 *          - Validates fence configuration if geofencing is enabled
 *          - Verifies motor system initialization and configuration
 *          - Checks GCS communication and failsafe state
 *          - Validates altitude estimation capability for non-manual flight modes
 *          - Calls parent class pre-arm checks for common validation
 *          
 *          If ARMING_CHECK parameter is set to 0, only mandatory checks are performed.
 * 
 * @param[in] display_failure If true, sends failure messages to GCS for pilot notification
 * 
 * @return true if all enabled pre-arm checks pass successfully, false if any check fails
 * 
 * @note Uses bitwise AND (&) instead of logical AND (&&) to ensure all checks run
 *       even if early checks fail, providing complete failure information to pilot.
 * 
 * @warning Motor arming is blocked if this function returns false. All checks must
 *          pass to ensure safe flight conditions.
 * 
 * @see mandatory_checks() for minimum required checks when ARMING_CHECK is disabled
 * @see parameter_checks(), motor_checks(), alt_checks() for specific validation
 */
bool AP_Arming_Blimp::run_pre_arm_checks(bool display_failure)
{
    // exit immediately if already armed
    if (blimp.motors->armed()) {
        return true;
    }

    if (!hal.scheduler->is_system_initialized()) {
        check_failed(display_failure, "System not initialised");
        return false;
    }

    // check if motor interlock and Emergency Stop aux switches are used
    // at the same time.  This cannot be allowed.
    if (rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOTOR_INTERLOCK) &&
        rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOTOR_ESTOP)) {
        check_failed(display_failure, "Interlock/E-Stop Conflict");
        return false;
    }

    // if pre arm checks are disabled run only the mandatory checks
    if (checks_to_perform == 0) {
        return mandatory_checks(display_failure);
    }

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wbitwise-instead-of-logical"
    return parameter_checks(display_failure)
#if AP_FENCE_ENABLED
           & fence_checks(display_failure)
#endif
           & motor_checks(display_failure)
           & gcs_failsafe_check(display_failure)
           & alt_checks(display_failure)
           & AP_Arming::pre_arm_checks(display_failure);
#pragma clang diagnostic pop
}

/**
 * @brief Validate barometer sensor health and altitude estimate consistency
 * 
 * @details Performs blimp-specific barometric pressure sensor checks in addition to
 *          base class validation. For absolute position flight modes, verifies that
 *          the barometer altitude and inertial navigation altitude estimates agree
 *          within acceptable tolerance (1 meter).
 *          
 *          The altitude disparity check is only performed when:
 *          - EKF is operating in absolute position mode (using barometer reference)
 *          - Not operating in ground-relative height mode (where EKF may intentionally differ)
 *          
 *          This is critical for lighter-than-air vehicles where accurate altitude
 *          control is essential for buoyancy management and station keeping.
 * 
 * @param[in] display_failure If true, sends failure messages to GCS for pilot notification
 * 
 * @return true if barometer is healthy and altitude estimates are consistent, false otherwise
 * 
 * @note Altitude disparity threshold is defined by PREARM_MAX_ALT_DISPARITY_CM (100cm = 1 meter)
 * 
 * @warning Altitude estimation errors can cause unexpected climb/descent behavior in
 *          altitude-hold modes, especially critical for buoyant vehicles.
 * 
 * @see AP_Arming::barometer_checks() for base barometer validation
 */
bool AP_Arming_Blimp::barometer_checks(bool display_failure)
{
    if (!AP_Arming::barometer_checks(display_failure)) {
        return false;
    }

    bool ret = true;
    // Check Baro - verify altitude estimate consistency for absolute position modes
    if (check_enabled(Check::BARO)) {
        // Check baro & inav alt are within 1m if EKF is operating in an absolute position mode.
        // Do not check if intending to operate in a ground relative height mode as EKF will output a ground relative height
        // that may differ from the baro height due to baro drift.
        const auto &ahrs = AP::ahrs();
        const bool using_baro_ref = !ahrs.has_status(AP_AHRS::Status::PRED_HORIZ_POS_REL) && ahrs.has_status(AP_AHRS::Status::PRED_HORIZ_POS_ABS);
        if (using_baro_ref) {
            if (fabsf(blimp.inertial_nav.get_position_z_up_cm() - blimp.baro_alt) > PREARM_MAX_ALT_DISPARITY_CM) {
                check_failed(Check::BARO, display_failure, "Altitude disparity");
                ret = false;
            }
        }
    }
    return ret;
}

/**
 * @brief Validate Inertial Navigation System (INS) sensor health for blimp
 * 
 * @details Performs inertial measurement unit (IMU) checks including accelerometer
 *          and gyroscope validation. In addition to base class INS checks, verifies
 *          that the Extended Kalman Filter (EKF) has achieved a valid attitude solution.
 *          
 *          EKF attitude validation is critical as attitude errors are typically caused by:
 *          - Uncalibrated or drifting gyroscope biases
 *          - IMU sensor failures or errors
 *          - Excessive vibration affecting sensor readings
 *          - Incorrect sensor orientation configuration
 * 
 * @param[in] display_failure If true, sends failure messages to GCS for pilot notification
 * 
 * @return true if all INS sensors are healthy and EKF attitude is valid, false otherwise
 * 
 * @warning Invalid attitude estimates will cause unstable flight control. This check
 *          protects against arming with bad gyroscope biases or IMU failures.
 * 
 * @note EKF attitude check is essential for lighter-than-air vehicles which require
 *       precise attitude control for directional stability and fin effectiveness.
 * 
 * @see pre_arm_ekf_attitude_check() for EKF attitude validation details
 * @see AP_Arming::ins_checks() for base INS validation
 */
bool AP_Arming_Blimp::ins_checks(bool display_failure)
{
    bool ret = AP_Arming::ins_checks(display_failure);

    if (check_enabled(Check::INS)) {

        // Get EKF attitude status (if bad, it's usually due to uncalibrated gyro biases)
        if (!pre_arm_ekf_attitude_check()) {
            check_failed(Check::INS, display_failure, "EKF attitude is bad");
            ret = false;
        }
    }

    return ret;
}

/**
 * @brief Validate board power supply and battery voltage levels
 * 
 * @details Performs electrical system checks to ensure adequate power for safe flight:
 *          - Validates autopilot board voltage is within acceptable range
 *          - Checks battery voltage has not triggered failsafe condition
 *          - Verifies battery monitoring system is functional
 *          
 *          Battery failsafe triggers when voltage drops below configured threshold,
 *          indicating insufficient power for safe flight operations. For lighter-than-air
 *          vehicles, adequate power is essential for actuator control and buoyancy management.
 * 
 * @param[in] display_failure If true, sends failure messages to GCS for pilot notification
 * 
 * @return true if board voltage and battery levels are acceptable, false if failsafe triggered
 * 
 * @warning Low battery voltage can cause brownouts, loss of control, or unexpected behavior.
 *          Arming is prevented if battery failsafe is active to avoid in-flight power loss.
 * 
 * @note Battery failsafe state is checked before parent class battery_checks() to provide
 *       early detection of critical power issues.
 * 
 * @see AP_Arming::board_voltage_checks() for autopilot board voltage validation
 * @see AP_Arming::battery_checks() for detailed battery monitoring validation
 */
bool AP_Arming_Blimp::board_voltage_checks(bool display_failure)
{
    if (!AP_Arming::board_voltage_checks(display_failure)) {
        return false;
    }

    // Check battery voltage and failsafe state
    if (check_enabled(Check::VOLTAGE)) {
        if (blimp.battery.has_failsafed()) {
            check_failed(Check::VOLTAGE, display_failure, "Battery failsafe");
            return false;
        }

        // call parent battery checks
        if (!AP_Arming::battery_checks(display_failure)) {
            return false;
        }
    }

    return true;
}

/**
 * @brief Validate critical parameter configuration for blimp operation
 * 
 * @details Verifies that key parameters are configured with safe and consistent values.
 *          Currently validates throttle failsafe configuration:
 *          
 *          - FS_THR_VALUE must be at least 10 PWM units below throttle channel minimum
 *          - FS_THR_VALUE must be above 910 PWM (above PPM encoder loss-of-signal value of 900)
 *          
 *          These checks ensure the failsafe trigger threshold is distinguishable from
 *          normal throttle input range and from RC receiver signal loss conditions.
 * 
 * @param[in] display_failure If true, sends failure messages to GCS for pilot notification
 * 
 * @return true if all checked parameters are configured correctly, false if validation fails
 * 
 * @warning Incorrect failsafe parameters can prevent proper failsafe activation or cause
 *          false failsafe triggers during normal flight, both of which compromise safety.
 * 
 * @note Throttle failsafe is particularly important for lighter-than-air vehicles to
 *       prevent loss of control if RC link is degraded or lost.
 * 
 * @see blimp.g.failsafe_throttle for failsafe enable flag
 * @see blimp.g.failsafe_throttle_value for configured failsafe PWM threshold
 */
bool AP_Arming_Blimp::parameter_checks(bool display_failure)
{
    // Check various parameter values for safe and consistent configuration
    if (check_enabled(Check::PARAMETERS)) {

        // Validate throttle failsafe parameter configuration
        if (blimp.g.failsafe_throttle) {
            // check throttle min is above throttle failsafe trigger and that the trigger is above ppm encoder's loss-of-signal value of 900
            if (blimp.channel_up->get_radio_min() <= blimp.g.failsafe_throttle_value+10 || blimp.g.failsafe_throttle_value < 910) {
                check_failed(Check::PARAMETERS, display_failure, "Check FS_THR_VALUE");
                return false;
            }
        }
    }

    return true;
}

/**
 * @brief Validate motor and actuator system initialization
 * 
 * @details Verifies that the motor control system has been properly initialized
 *          for the configured blimp frame type. Checks include:
 *          - Motor library successfully initialized (motors->initialised_ok())
 *          - Correct firmware compiled for configured FRAME_CLASS parameter
 *          - Actuator output channels properly configured
 *          
 *          For lighter-than-air vehicles, the motor system includes fin actuators
 *          and thrust motors specific to airship configurations.
 * 
 * @param[in] display_failure If true, sends failure messages to GCS for pilot notification
 * 
 * @return true if motor system is properly initialized, false if initialization failed
 * 
 * @warning Motor initialization failure indicates firmware/parameter mismatch or
 *          configuration error. Flight without proper motor control is unsafe.
 * 
 * @note If motor initialization fails, check that FRAME_CLASS parameter matches
 *       the compiled firmware variant and that all required actuator outputs are configured.
 * 
 * @see AP_Motors_Blimp for blimp-specific motor control implementation
 */
bool AP_Arming_Blimp::motor_checks(bool display_failure)
{
    // Verify motors/actuators initialized correctly for configured frame type
    if (!blimp.motors->initialised_ok()) {
        check_failed(display_failure, "Check firmware or FRAME_CLASS");
        return false;
    }

    // further checks enabled with parameters
    if (!check_enabled(Check::PARAMETERS)) {
        return true;
    }

    return true;
}

/**
 * @brief Validate RC (radio control) input calibration
 * 
 * @details RC calibration check for blimp vehicle. Currently no blimp-specific
 *          RC calibration validation is required beyond base class checks.
 * 
 * @param[in] display_failure If true, sends failure messages to GCS for pilot notification
 * 
 * @return true (always passes for blimp)
 * 
 * @note This function is a placeholder for potential future blimp-specific RC validation
 */
bool AP_Arming_Blimp::rc_calibration_checks(bool display_failure)
{
    return true;
}

/**
 * @brief Validate GPS sensor health and position accuracy for blimp
 * 
 * @details Performs comprehensive GPS validation including:
 *          - Mandatory GPS checks (AHRS initialization, position estimate availability)
 *          - Flight mode GPS requirement validation
 *          - HDOP (Horizontal Dilution of Precision) threshold check
 *          - Parent class GPS health checks (satellite count, fix type, etc.)
 *          
 *          GPS requirements vary by flight mode:
 *          - Modes requiring GPS: Must have valid position estimate and acceptable HDOP
 *          - Manual modes: GPS validation skipped
 *          
 *          HDOP check ensures GPS accuracy is sufficient for navigation. High HDOP
 *          indicates poor satellite geometry and degraded position accuracy.
 * 
 * @param[in] display_failure If true, sends failure messages to GCS for pilot notification
 * 
 * @return true if GPS meets requirements for current/intended flight mode, false otherwise
 * 
 * @warning GPS-dependent flight modes (Loiter, Auto, Guided) require valid GPS fix.
 *          Arming without adequate GPS in these modes will cause navigation failures.
 * 
 * @note Updates AP_Notify::flags.pre_arm_gps_check to indicate GPS status to pilot
 * @note HDOP threshold is configured via GPS_HDOP_GOOD parameter
 * 
 * @see mandatory_gps_checks() for essential GPS validation
 * @see AP_Arming::gps_checks() for base GPS health validation
 */
bool AP_Arming_Blimp::gps_checks(bool display_failure)
{
    // run mandatory gps checks first
    if (!mandatory_gps_checks(display_failure)) {
        AP_Notify::flags.pre_arm_gps_check = false;
        return false;
    }

    // check if flight mode requires GPS
    bool mode_requires_gps = blimp.flightmode->requires_GPS();


    // return true if GPS is not required
    if (!mode_requires_gps) {
        AP_Notify::flags.pre_arm_gps_check = true;
        return true;
    }

    // return true immediately if gps check is disabled
    if (!check_enabled(Check::GPS)) {
        AP_Notify::flags.pre_arm_gps_check = true;
        return true;
    }

    // warn about hdop separately - to prevent user confusion with no gps lock
    if (blimp.gps.get_hdop() > blimp.g.gps_hdop_good) {
        check_failed(Check::GPS, display_failure, "High GPS HDOP");
        AP_Notify::flags.pre_arm_gps_check = false;
        return false;
    }

    // call parent gps checks
    if (!AP_Arming::gps_checks(display_failure)) {
        AP_Notify::flags.pre_arm_gps_check = false;
        return false;
    }

    // if we got here all must be ok
    AP_Notify::flags.pre_arm_gps_check = true;
    return true;
}

/**
 * @brief Verify Extended Kalman Filter has valid attitude estimate
 * 
 * @details Checks that the EKF has converged to a valid attitude solution before
 *          allowing motor arming. Invalid EKF attitude typically indicates:
 *          - Gyroscope biases have not converged (vehicle needs to sit stationary)
 *          - IMU sensor errors or failures
 *          - Excessive vibration preventing convergence
 *          - Incorrect IMU orientation configuration
 * 
 * @return true if EKF attitude is valid, false if attitude estimate is not reliable
 * 
 * @warning Flying with invalid attitude estimate will cause erratic flight control
 *          behavior and potential loss of control. This is a critical safety check.
 * 
 * @note Vehicle must remain stationary for 10-30 seconds after power-on to allow
 *       EKF initialization and gyro bias estimation to complete.
 * 
 * @see AP_AHRS::Status::ATTITUDE_VALID for attitude validity criteria
 */
bool AP_Arming_Blimp::pre_arm_ekf_attitude_check()
{
    return AP::ahrs().has_status(AP_AHRS::Status::ATTITUDE_VALID);
}

/**
 * @brief Perform mandatory GPS and navigation checks required for safe flight
 * 
 * @details Executes essential navigation system validation that cannot be bypassed:
 *          - AHRS (Attitude Heading Reference System) pre-arm validation
 *          - Position estimate availability check for GPS-dependent flight modes
 *          - GPS glitch detection from EKF filter status
 *          
 *          These checks are performed even when ARMING_CHECK parameter is disabled,
 *          as they validate fundamental navigation capabilities required for controlled flight.
 *          
 *          For GPS-dependent modes, verifies vehicle has valid position estimate from
 *          EKF fusion of GPS and inertial sensors. For manual modes, GPS validation is skipped.
 * 
 * @param[in] display_failure If true, sends failure messages to GCS for pilot notification
 * 
 * @return true if mandatory navigation checks pass, false if critical validation fails
 * 
 * @warning GPS glitching (as detected by EKF innovation checks) indicates unreliable
 *          position data. Arming with GPS glitches can cause navigation failures and
 *          unexpected flight behavior.
 * 
 * @note This function is called by both standard pre-arm checks and mandatory_checks()
 *       to ensure GPS validation cannot be bypassed for GPS-dependent flight modes.
 * 
 * @see blimp.flightmode->requires_GPS() to determine if current mode needs GPS
 * @see blimp.position_ok() for position estimate availability check
 */
bool AP_Arming_Blimp::mandatory_gps_checks(bool display_failure)
{
    // always check if inertial nav has started and is ready
    const auto &ahrs = AP::ahrs();
    char failure_msg[50] = {};
    if (!ahrs.pre_arm_check(false, failure_msg, sizeof(failure_msg))) {
        check_failed(display_failure, "AHRS: %s", failure_msg);
        return false;
    }

    // check if flight mode requires GPS
    bool mode_requires_gps = blimp.flightmode->requires_GPS();

    if (mode_requires_gps) {
        if (!blimp.position_ok()) {
            // vehicle level position estimate checks
            check_failed(display_failure, "Need Position Estimate");
            return false;
        }
    } else  {
        // return true if GPS is not required
        return true;
    }

    // check for GPS glitch (as reported by EKF)
    nav_filter_status filt_status;
    if (ahrs.get_filter_status(filt_status)) {
        if (filt_status.flags.gps_glitching) {
            check_failed(display_failure, "GPS glitching");
            return false;
        }
    }

    // if we got here all must be ok
    return true;
}

/**
 * @brief Validate Ground Control Station (GCS) communication status
 * 
 * @details Checks that GCS failsafe is not currently active before allowing arming.
 *          GCS failsafe triggers when MAVLink heartbeat messages from ground station
 *          are not received within configured timeout period, indicating loss of
 *          communication link.
 *          
 *          Prevents arming when GCS link is degraded or lost to ensure pilot has
 *          reliable command and telemetry during flight operations.
 * 
 * @param[in] display_failure If true, sends failure messages to GCS for pilot notification
 * 
 * @return true if GCS communication is healthy, false if failsafe is active
 * 
 * @warning Arming with GCS failsafe active indicates communication problems that
 *          could prevent mission monitoring or emergency intervention.
 * 
 * @note GCS failsafe configuration is set via FS_GCS_ENABLE and FS_GCS_TIMEOUT parameters
 */
bool AP_Arming_Blimp::gcs_failsafe_check(bool display_failure)
{
    if (blimp.failsafe.gcs) {
        check_failed(display_failure, "GCS failsafe on");
        return false;
    }
    return true;
}

/**
 * @brief Validate altitude estimation capability for blimp flight
 * 
 * @details Verifies that altitude estimate is available and reliable for flight modes
 *          that require altitude control. Check is skipped for manual throttle modes
 *          where pilot directly controls vertical movement.
 *          
 *          Altitude estimate validation uses ekf_alt_ok() which checks:
 *          - EKF has valid altitude estimate from barometer and/or GPS fusion
 *          - Altitude innovation (difference between predicted and measured) is acceptable
 *          - Altitude estimate covariance indicates adequate confidence
 *          
 *          Critical for lighter-than-air vehicles which rely on altitude hold for
 *          buoyancy compensation and station keeping in non-manual modes.
 * 
 * @param[in] display_failure If true, sends failure messages to GCS for pilot notification
 * 
 * @return true if altitude estimate is available or not required, false if needed but unavailable
 * 
 * @warning Altitude control modes without valid altitude estimate will cause erratic
 *          vertical behavior and inability to maintain desired altitude.
 * 
 * @note Manual throttle modes (like Stabilize) do not require altitude estimate
 * 
 * @see blimp.ekf_alt_ok() for detailed altitude estimate validation criteria
 * @see blimp.flightmode->has_manual_throttle() for mode throttle control type
 */
bool AP_Arming_Blimp::alt_checks(bool display_failure)
{
    // always EKF altitude estimate
    if (!blimp.flightmode->has_manual_throttle() && !blimp.ekf_alt_ok()) {
        check_failed(display_failure, "Need Alt Estimate");
        return false;
    }

    return true;
}

/**
 * @brief Final validation before allowing motor arming
 * 
 * @details Performs last-moment checks immediately before arming motors. Called after
 *          pre-arm checks have passed and arming is requested via RC stick command,
 *          GCS command, or other arming method.
 *          
 *          Currently defers to parent class arm_checks() for standard validation.
 *          Has side effect of starting onboard logging if configured.
 * 
 * @param[in] method Arming method being used (rudder stick, MAVLink command, switch, etc.)
 * 
 * @return true if final checks pass and arming should proceed, false to reject arming
 * 
 * @warning This is the last safety gate before motors become active. Return false to
 *          prevent arming if any unsafe condition is detected.
 * 
 * @note Logging is started during this check sequence to ensure all arming events
 *       and subsequent flight data are captured.
 * 
 * @see AP_Arming::arm_checks() for standard final validation
 * @see arm() for complete arming sequence
 */
bool AP_Arming_Blimp::arm_checks(AP_Arming::Method method)
{
    return AP_Arming::arm_checks(method);
}

/**
 * @brief Minimum required checks performed when arming checks are disabled
 * 
 * @details Executes critical safety checks that cannot be bypassed even when
 *          ARMING_CHECK parameter is set to 0 or arming is forced. These represent
 *          the absolute minimum validation required for any flight attempt:
 *          
 *          - Mandatory GPS checks (AHRS health, position estimate for GPS modes)
 *          - Altitude estimate validation for non-manual throttle modes
 *          - Parent class mandatory checks (system initialization, etc.)
 *          
 *          These checks protect against fundamental system failures that would
 *          make controlled flight impossible regardless of pilot skill or intent.
 * 
 * @param[in] display_failure If true, sends failure messages to GCS for pilot notification
 * 
 * @return true if mandatory checks pass, false if critical validation fails
 * 
 * @warning These checks represent minimum safety requirements. Bypassing standard
 *          arming checks (ARMING_CHECK=0) is dangerous and should only be used
 *          for testing in controlled environments.
 * 
 * @note Updates AP_Notify GPS check flag to ensure pilot is informed of GPS status
 * @note Uses bitwise AND (&) to ensure all checks execute even if early checks fail
 * 
 * @see mandatory_gps_checks() for GPS/navigation validation
 * @see alt_checks() for altitude estimation validation
 */
bool AP_Arming_Blimp::mandatory_checks(bool display_failure)
{
    // call mandatory gps checks and update notify status because regular gps checks will not run
    bool result = mandatory_gps_checks(display_failure);
    AP_Notify::flags.pre_arm_gps_check = result;

    // call mandatory alt check
    if (!alt_checks(display_failure)) {
        result = false;
    }

    return result & AP_Arming::mandatory_checks(display_failure);
}

/**
 * @brief Update vehicle pre-arm check status flags
 * 
 * @details Sets both internal vehicle state (blimp.ap.pre_arm_check) and notification
 *          system flag (AP_Notify) to reflect current pre-arm check status. This
 *          updates LED indicators and telemetry to inform pilot of readiness to arm.
 * 
 * @param[in] b true if pre-arm checks have passed, false if checks failed
 * 
 * @note This function synchronizes multiple status flags to ensure consistent
 *       pre-arm status indication across the system.
 */
void AP_Arming_Blimp::set_pre_arm_check(bool b)
{
    blimp.ap.pre_arm_check = b;
    AP_Notify::flags.pre_arm_check = b;
}

/**
 * @brief Complete arming sequence for blimp vehicle
 * 
 * @details Executes full motor arming sequence including safety checks, initialization,
 *          and state transitions required to enable flight. The sequence includes:
 *          
 *          1. Reentrancy protection (prevent concurrent arming attempts)
 *          2. Check if already armed (return success immediately)
 *          3. Execute arming checks via AP_Arming::arm() unless bypassed
 *          4. Enable vehicle armed status and notify pilot (LED, telemetry)
 *          5. Initialize home position and altitude datum if not already set:
 *             - If no home: Reset EKF altitude datum and set arming altitude to zero
 *             - If home not locked: Update home to current location and record arming altitude
 *          6. Record initial armed bearing for navigation reference
 *          7. Set system armed flags (hal.util soft armed, AP_Notify armed)
 *          8. Activate motors (motors->armed(true))
 *          9. Log arming event and current flight mode
 *          10. Record arming timestamp and start arming delay period
 *          
 *          The arming delay (ap.in_arming_delay) provides brief period after arming
 *          before motors respond to throttle, allowing pilot to verify armed state.
 * 
 * @param[in] method Arming method used (RUDDER stick, MAVLINK command, SWITCH, etc.)
 * @param[in] do_arming_checks If true, execute full arming checks; if false, bypass checks
 * 
 * @return true if arming completed successfully, false if arming was rejected
 * 
 * @warning This is safety-critical code. Motors become active after this function
 *          completes successfully. All safety checks must pass before this point.
 * 
 * @warning Static reentrancy guard (in_arm_motors) prevents concurrent execution.
 *          Do not call this function from interrupt context or multiple threads.
 * 
 * @note Home position initialization is specific to blimp operation:
 *       - Altitude datum reset allows relative altitude tracking for buoyancy control
 *       - Arming altitude recording enables altitude-relative navigation
 * 
 * @note Initial armed bearing provides navigation reference for heading-hold modes
 * 
 * @see AP_Arming::arm() for common arming validation
 * @see disarm() for complementary disarm sequence
 * 
 * Source: Blimp/AP_Arming_Blimp.cpp:290-371
 */
bool AP_Arming_Blimp::arm(const AP_Arming::Method method, const bool do_arming_checks)
{
    static bool in_arm_motors = false;

    // exit immediately if already in this function
    if (in_arm_motors) {
        return false;
    }
    in_arm_motors = true;

    // return true if already armed
    if (blimp.motors->armed()) {
        in_arm_motors = false;
        return true;
    }

    if (!AP_Arming::arm(method, do_arming_checks)) {
        AP_Notify::events.arming_failed = true;
        in_arm_motors = false;
        return false;
    }

#if HAL_LOGGING_ENABLED
    // let logger know that we're armed (it may open logs e.g.)
    AP::logger().set_vehicle_armed(true);
#endif

    // notify that arming will occur (we do this early to give plenty of warning)
    AP_Notify::flags.armed = true;
    // call notify update a few times to ensure the message gets out
    for (uint8_t i=0; i<=10; i++) {
        AP::notify().update();
    }

    send_arm_disarm_statustext("Arming motors"); //MIR kept in - usually only in SITL

    auto &ahrs = AP::ahrs();

    blimp.initial_armed_bearing = ahrs.yaw_sensor;

    if (!ahrs.home_is_set()) {
        // Reset EKF altitude if home hasn't been set yet (we use EKF altitude as substitute for alt above home)
        ahrs.resetHeightDatum();
        LOGGER_WRITE_EVENT(LogEvent::EKF_ALT_RESET);

        // we have reset height, so arming height is zero
        blimp.arming_altitude_m = 0;
    } else if (!ahrs.home_is_locked()) {
        // Reset home position if it has already been set before (but not locked)
        if (!blimp.set_home_to_current_location(false)) {
            // ignore failure
        }

        // remember the height when we armed
        blimp.arming_altitude_m = blimp.inertial_nav.get_position_z_up_cm() * 0.01;
    }

    hal.util->set_soft_armed(true);

    // finally actually arm the motors
    blimp.motors->armed(true);

#if HAL_LOGGING_ENABLED
    // log flight mode in case it was changed while vehicle was disarmed
    AP::logger().Write_Mode((uint8_t)blimp.control_mode, blimp.control_mode_reason);
#endif

    // perf monitor ignores delay due to arming
    AP::scheduler().perf_info.ignore_this_loop();

    // flag exiting this function
    in_arm_motors = false;

    // Log time stamp of arming event
    blimp.arm_time_ms = millis();

    // Start the arming delay
    blimp.ap.in_arming_delay = true;

    // return success
    return true;
}

/**
 * @brief Complete disarming sequence for blimp vehicle
 * 
 * @details Executes full motor disarming sequence including safety checks, compass
 *          offset learning, and state transitions to disable flight. The sequence includes:
 *          
 *          1. Check if already disarmed (return success immediately)
 *          2. For rudder stick disarm method, verify safety conditions:
 *             - Manual throttle control mode OR vehicle has landed (land_complete)
 *          3. Execute disarm checks via AP_Arming::disarm() unless bypassed
 *          4. Send disarm notification to GCS
 *          5. Save learned compass offsets from EKF:
 *             - If compass learning enabled (COMPASS_LEARN_TYPE = COPY_FROM_EKF)
 *             - Copies EKF-estimated compass offsets to compass calibration
 *             - Persists offsets to non-volatile storage for future flights
 *          6. Deactivate motors (motors->armed(false))
 *          7. Clear system armed flags (logger, hal.util, AP_Notify)
 *          8. Clear arming delay flag
 *          
 *          Compass offset learning during disarm allows EKF to improve compass
 *          calibration based on in-flight data, particularly useful for lighter-than-air
 *          vehicles which may experience different magnetic interference patterns in flight.
 * 
 * @param[in] method Disarm method used (RUDDER stick, MAVLINK command, etc.)
 * @param[in] do_disarm_checks If true, execute disarm safety checks; if false, bypass checks
 * 
 * @return true if disarming completed successfully, false if disarm was rejected
 * 
 * @warning For safety, rudder stick disarm is only allowed in manual throttle modes or
 *          after landing. This prevents accidental disarm during automated flight or hover.
 * 
 * @note Compass offset saving is conditional on:
 *       - Compass in use (ahrs.use_compass())
 *       - COMPASS_LEARN_TYPE set to COPY_FROM_EKF (learn type 1)
 *       - EKF has valid compass offset estimates for each compass instance
 * 
 * @note Iterates through all compass instances (up to COMPASS_MAX_INSTANCES) to save
 *       offsets for multi-compass configurations.
 * 
 * @see AP_Arming::disarm() for common disarm validation
 * @see arm() for complementary arming sequence
 * @see Compass::LearnType for compass calibration learning modes
 * 
 * Source: Blimp/AP_Arming_Blimp.cpp:374-418
 */
bool AP_Arming_Blimp::disarm(const AP_Arming::Method method, bool do_disarm_checks)
{
    // return immediately if we are already disarmed
    if (!blimp.motors->armed()) {
        return true;
    }

    if (method == AP_Arming::Method::RUDDER) {
        if (!blimp.flightmode->has_manual_throttle() && !blimp.ap.land_complete) {
            return false;
        }
    }

    if (!AP_Arming::disarm(method, do_disarm_checks)) {
        return false;
    }

    send_arm_disarm_statustext("Disarming motors"); //MIR keeping in - usually only in SITL

    auto &ahrs = AP::ahrs();

    // save compass offsets learned by the EKF if enabled
    Compass &compass = AP::compass();
    if (ahrs.use_compass() && compass.get_learn_type() == Compass::LearnType::COPY_FROM_EKF) {
        for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
            Vector3f magOffsets;
            if (ahrs.getMagOffsets(i, magOffsets)) {
                compass.set_and_save_offsets(i, magOffsets);
            }
        }
    }

    // send disarm command to motors
    blimp.motors->armed(false);

#if HAL_LOGGING_ENABLED
    AP::logger().set_vehicle_armed(false);
#endif

    hal.util->set_soft_armed(false);

    blimp.ap.in_arming_delay = false;

    return true;
}
