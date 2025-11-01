/**
 * @file AP_Arming_Copter.h
 * @brief Copter-specific arming checks and safety validation
 * 
 * @details This file defines the AP_Arming_Copter class which extends the base
 *          AP_Arming class with multicopter-specific pre-arm and arming checks.
 *          
 *          The arming system implements a two-stage safety validation process:
 *          
 *          1. **Pre-arm checks**: Run continuously when the vehicle is disarmed
 *             to verify that sensors, configuration, and vehicle state are safe
 *             for flight. These checks must pass before arming is allowed.
 *          
 *          2. **Arm checks**: Run once when arming is attempted to perform
 *             final validation that conditions are safe for immediate takeoff.
 *          
 *          The comprehensive safety philosophy prevents takeoff with:
 *          - Uncalibrated or unhealthy sensors (IMU, compass, barometer, GPS)
 *          - Unsafe configuration (invalid parameters, missing geofence data)
 *          - Environmental hazards (low battery, GPS glitches, proximity alerts)
 *          - Pilot input issues (RC calibration, throttle position)
 *          
 *          Copter-specific checks extend base AP_Arming checks with validation
 *          for multicopter flight modes, motor configuration, altitude limits,
 *          terrain database requirements, and object avoidance systems.
 * 
 * @note Pre-arm checks can be bypassed via ARMING_CHECK parameter, but
 *       mandatory_checks() always run and cannot be bypassed for safety.
 * 
 * @warning Modifying arming checks affects flight safety. All changes must
 *          be thoroughly tested in SITL and on hardware before flight.
 * 
 * @see AP_Arming base class in libraries/AP_Arming/
 * @see Copter::init_arm_motors() for integration with flight code
 * 
 * Source: ArduCopter/AP_Arming_Copter.h
 */

#pragma once

#include <AP_Arming/AP_Arming.h>

/**
 * @class AP_Arming_Copter
 * @brief Multicopter-specific arming and pre-arm check implementation
 * 
 * @details This class extends AP_Arming with copter-specific safety validation
 *          checks that must pass before the vehicle is allowed to arm and takeoff.
 *          
 *          **Two-Stage Arming Process**:
 *          
 *          - **Stage 1 - Pre-arm checks** (run_pre_arm_checks/pre_arm_checks):
 *            Execute continuously while disarmed at approximately 1Hz. These
 *            comprehensive checks validate sensor health, calibration state,
 *            configuration parameters, and environmental conditions. Results
 *            are displayed to the pilot via GCS messages and must pass before
 *            arming is permitted.
 *          
 *          - **Stage 2 - Arm checks** (arm_checks):
 *            Execute once when arming is requested. Perform final validation
 *            that immediate takeoff conditions are safe, including RC input
 *            position, mode validity, and last-moment sensor status.
 *          
 *          **Copter-Specific Validations**:
 *          
 *          This class overrides base AP_Arming methods to add multicopter-specific
 *          requirements:
 *          
 *          - **INS checks**: Validate gyroscope and accelerometer health for
 *            attitude control stability
 *          - **GPS checks**: Verify GPS quality for position-hold and navigation
 *            modes, enforce minimum satellite count and HDOP limits
 *          - **Barometer checks**: Ensure altitude sensing for altitude-hold modes
 *          - **Voltage checks**: Prevent arming with critically low battery
 *          - **Parameter checks**: Validate copter-specific parameter ranges
 *          - **Altitude checks**: Enforce maximum altitude limits if configured
 *          - **Object avoidance checks**: Verify proximity sensor health if enabled
 *          - **Terrain database checks**: Ensure terrain data loaded for terrain modes
 *          
 *          **Safety Philosophy**:
 *          
 *          The arming system implements defense-in-depth safety validation:
 *          
 *          1. **Fail-safe by default**: Vehicle cannot arm unless all checks pass
 *          2. **Clear feedback**: Failed checks reported to pilot with specific messages
 *          3. **Configurable strictness**: ARMING_CHECK bitmask allows selective bypass
 *          4. **Mandatory checks**: Critical safety checks cannot be bypassed
 *          5. **Mode-aware validation**: Checks adapt to flight mode requirements
 *          
 *          **Check Bypass System**:
 *          
 *          The ARMING_CHECK parameter allows selective bypass of pre-arm checks
 *          for testing or emergency situations. However, mandatory_checks() always
 *          execute and cannot be bypassed, ensuring minimum safety standards.
 *          
 * @note Constructor sets REQUIRE parameter default to YES_MIN_PWM, enforcing
 *       rudder-right or throttle-minimum arming gesture for copters.
 * 
 * @note Pre-arm check results are cached to avoid redundant sensor queries
 *       and provide consistent status during the arming attempt window.
 * 
 * @warning Disabling arming checks (ARMING_CHECK=0) removes critical safety
 *          validation and should only be used for testing in controlled environments.
 * 
 * @see AP_Arming base class for common arming infrastructure
 * @see Copter::init_arm_motors() for arming sequence integration
 * @see Copter::init_disarm_motors() for disarming sequence
 * 
 * Source: ArduCopter/AP_Arming_Copter.h:5-65
 */
class AP_Arming_Copter : public AP_Arming
{
public:
    friend class Copter;
    friend class ToyMode;

    /**
     * @brief Constructor for copter arming checks
     * 
     * @details Initializes the copter-specific arming system by calling the
     *          base AP_Arming constructor and setting the REQUIRE parameter
     *          default to YES_MIN_PWM. This enforces that copters require
     *          either a rudder-right stick gesture or minimum throttle position
     *          to arm, preventing accidental arming.
     * 
     * @note Copter does not have a separate ARMING_REQUIRE parameter; this
     *       default is applied to the base class require parameter.
     * 
     * @note Called once during Copter object construction at startup.
     */
    AP_Arming_Copter() : AP_Arming()
    {
        // default REQUIRE parameter to 1 (Copter does not have an
        // actual ARMING_REQUIRE parameter)
        require.set_default((uint8_t)Required::YES_MIN_PWM);
    }

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Arming_Copter);

    /**
     * @brief Validate RC transmitter calibration for copter
     * 
     * @details Verifies that RC input channels are properly calibrated with
     *          appropriate min/max/trim values. For copters, this checks all
     *          flight control channels (roll, pitch, yaw, throttle) to ensure
     *          stick inputs will be correctly interpreted for flight control.
     *          
     *          Overrides base AP_Arming implementation to add copter-specific
     *          throttle channel validation requirements.
     * 
     * @param[in] display_failure If true, send GCS message describing failure reason
     * 
     * @return true if RC calibration is valid, false if calibration check fails
     * 
     * @note Called during pre-arm checks if ARMING_CHECK includes RC bit
     * 
     * @warning Invalid RC calibration can cause incorrect control surface
     *          deflection or throttle response during flight
     * 
     * @see RC_Channel::rc_calibration_checks() for per-channel validation
     */
    bool rc_calibration_checks(bool display_failure) override;

    /**
     * @brief Disarm the copter motors
     * 
     * @details Executes the motor disarming sequence, stopping motor output
     *          and transitioning vehicle to disarmed state. Optionally performs
     *          post-disarm checks and logging before completing disarm.
     *          
     *          Overrides base AP_Arming to add copter-specific disarm actions
     *          such as motor output ramping and control surface neutralization.
     * 
     * @param[in] method Disarm method (RUDDER, GCS_COMMAND, BUTTON, etc.)
     * @param[in] do_disarm_checks If true, perform post-disarm validation checks
     * 
     * @return true if disarm successful, false if disarm rejected
     * 
     * @note Disarm can be rejected if in-flight or in certain flight modes
     * 
     * @see Copter::init_disarm_motors() for copter disarm sequence
     */
    bool disarm(AP_Arming::Method method, bool do_disarm_checks=true) override;
    
    /**
     * @brief Arm the copter motors for flight
     * 
     * @details Executes the motor arming sequence after validation checks pass.
     *          This method orchestrates pre-arm checks, arm checks, and the
     *          actual motor arming sequence including motor spin-up and control
     *          system initialization.
     *          
     *          Overrides base AP_Arming to add copter-specific arming sequence:
     *          - Enable motor output
     *          - Initialize attitude controllers
     *          - Reset navigation systems
     *          - Configure motor interlock
     *          - Start pre-arm position logging
     * 
     * @param[in] method Arm method (RUDDER, MAVLINK, SWITCH, etc.)
     * @param[in] do_arming_checks If true, perform pre-arm and arm validation checks
     * 
     * @return true if arming successful, false if checks fail or arming rejected
     * 
     * @note Pre-arm checks must pass before arm checks execute
     * @note Arming can be forced with ARMING_CHECK=0 but mandatory checks still run
     * @note Arming rejected if in certain flight modes or unsafe conditions detected
     * 
     * @warning Forced arming bypasses safety checks and should only be used
     *          in controlled test environments
     * 
     * @see Copter::init_arm_motors() for copter arm sequence
     * @see arm_checks() for final validation before arming
     */
    bool arm(AP_Arming::Method method, bool do_arming_checks=true) override;

protected:

    /**
     * @brief Execute comprehensive pre-arm safety checks for copter
     * 
     * @details Wrapper function that calls run_pre_arm_checks() and caches the
     *          result for consistent reporting during arming attempts. This is
     *          the main entry point for pre-arm validation, called at ~1Hz while
     *          disarmed and once more during arm attempt.
     *          
     *          Copter pre-arm checks validate:
     *          - Sensor health (INS, compass, barometer, GPS)
     *          - Sensor calibration state
     *          - EKF attitude initialization
     *          - Parameter validity
     *          - Battery voltage
     *          - RC calibration
     *          - Flight mode readiness
     *          - Geofence data loaded
     *          - Terrain database (if required)
     *          - Object avoidance system (if enabled)
     * 
     * @param[in] display_failure If true, send GCS messages for each failed check
     * 
     * @return true if all enabled pre-arm checks pass, false if any check fails
     * 
     * @note Checks can be selectively disabled via ARMING_CHECK bitmask parameter
     * @note Results cached to avoid redundant sensor queries during arm attempt
     * @note Called from both scheduler task (while disarmed) and arm() method
     * 
     * @see run_pre_arm_checks() for actual check implementation
     * @see arm_checks() for final validation before motor arming
     */
    bool pre_arm_checks(bool display_failure) override;
    
    /**
     * @brief Verify EKF attitude estimate is initialized and healthy
     * 
     * @details Validates that the Extended Kalman Filter has converged to a
     *          stable attitude solution before arming. Checks EKF initialization
     *          status and attitude estimate quality to ensure flight controllers
     *          have accurate orientation data.
     *          
     *          Prevents arming with poor attitude estimate which could cause
     *          incorrect control response or unstable flight immediately after takeoff.
     * 
     * @return true if EKF attitude estimate is healthy and converged, false otherwise
     * 
     * @note Called as part of pre_arm_checks validation sequence
     * @note EKF typically requires 10-30 seconds to converge after power-on
     * 
     * @warning Arming with unconverged EKF can cause immediate loss of control
     * 
     * @see AP_AHRS_NavEKF::healthy() for EKF health status
     */
    bool pre_arm_ekf_attitude_check();
    
#if HAL_PROXIMITY_ENABLED
    /**
     * @brief Validate proximity sensor health for object avoidance
     * 
     * @details Checks that proximity/rangefinder sensors used for object avoidance
     *          are functioning correctly and providing valid distance measurements.
     *          Only enforced if object avoidance is enabled via OA_TYPE parameter.
     *          
     *          Overrides base AP_Arming implementation to add copter-specific
     *          proximity sensor requirements for low-altitude obstacle detection.
     * 
     * @param[in] display_failure If true, send GCS message describing failure
     * 
     * @return true if proximity sensors healthy or OA disabled, false if check fails
     * 
     * @note Only compiled if HAL_PROXIMITY_ENABLED is defined
     * @note Called during pre-arm checks if ARMING_CHECK includes proximity bit
     * 
     * @see AP_Proximity::healthy() for sensor health status
     */
    bool proximity_checks(bool display_failure) const override;
#endif
    
    /**
     * @brief Execute final validation checks immediately before arming
     * 
     * @details Performs last-moment safety validation when arming is requested.
     *          These checks complement pre-arm checks by validating immediate
     *          conditions that must be true at the moment of arming:
     *          
     *          - RC throttle stick position (must be low)
     *          - Flight mode is armable (not RTL, AUTO with no mission, etc.)
     *          - Rotor speed governor ready (if equipped)
     *          - GCS failsafe not active
     *          - Motor interlock not engaged
     *          
     *          Overrides base AP_Arming to add copter-specific arm-time validation.
     * 
     * @param[in] method Arm method being used (RUDDER, MAVLINK, SWITCH, etc.)
     * 
     * @return true if all arm checks pass, false if any check fails
     * 
     * @note Called by arm() after pre-arm checks pass, immediately before motor arming
     * @note These checks always run regardless of ARMING_CHECK parameter
     * @note Arm checks are more time-sensitive than pre-arm checks
     * 
     * @warning Failures at arm-check stage indicate unsafe conditions for
     *          immediate takeoff even if pre-arm checks passed
     * 
     * @see pre_arm_checks() for earlier validation stage
     * @see mandatory_checks() for checks that cannot be bypassed
     */
    bool arm_checks(AP_Arming::Method method) override;

    /**
     * @brief Execute critical safety checks that cannot be bypassed
     * 
     * @details Performs minimum safety validation that always runs regardless
     *          of ARMING_CHECK parameter value. These checks enforce absolute
     *          minimum safety standards that cannot be disabled:
     *          
     *          - Critical sensor failures
     *          - Hardware malfunction detection  
     *          - Flight controller internal errors
     *          - Crash detection active
     *          - Critical battery failsafe
     *          
     *          This function only executes when ARMING_CHECK is set to 0
     *          (all checks disabled) or arming is forced, providing a safety
     *          net against completely unsafe arming attempts.
     * 
     * @param[in] display_failure If true, send GCS message describing failure
     * 
     * @return true if mandatory safety minimums met, false if critical failure detected
     * 
     * @note Only called when normal arming checks are bypassed (ARMING_CHECK=0)
     * @note These checks cannot be disabled by any parameter
     * @note Failure indicates vehicle is unsafe to fly under any circumstances
     * 
     * @warning Mandatory check failures indicate serious hardware or software
     *          problems that must be resolved before flight
     * 
     * @see pre_arm_checks() for full validation when checks enabled
     */
    bool mandatory_checks(bool display_failure) override;

    // NOTE! the following check functions *DO* call into AP_Arming:
    
    /**
     * @brief Validate inertial navigation system (INS) health for copter
     * 
     * @details Checks IMU sensor health (gyroscopes and accelerometers) with
     *          copter-specific requirements. Calls base AP_Arming::ins_checks()
     *          then adds multicopter-specific validation for attitude control.
     *          
     *          Validates:
     *          - All configured IMUs are healthy and calibrated
     *          - Gyroscope bias estimation converged
     *          - Accelerometer calibration valid
     *          - IMU consistency between multiple units
     *          - No excessive vibration detected
     *          - Temperature calibration applied (if enabled)
     * 
     * @param[in] display_failure If true, send GCS message describing failure
     * 
     * @return true if INS healthy for copter flight, false if check fails
     * 
     * @note Copters require higher IMU reliability than other vehicle types
     *       due to continuous attitude control loop requirements
     * @note Called during pre_arm_checks if ARMING_CHECK includes INS bit
     * 
     * @warning Flying with unhealthy IMU can cause attitude estimation errors
     *          leading to loss of control
     * 
     * @see AP_InertialSensor::healthy() for IMU health status
     * @see AP_Arming::ins_checks() for base validation
     */
    bool ins_checks(bool display_failure) override;
    
    /**
     * @brief Validate GPS health and accuracy for copter
     * 
     * @details Checks GPS receiver health with copter-specific position accuracy
     *          requirements. Calls base AP_Arming::gps_checks() then adds
     *          multicopter validation for position-dependent flight modes.
     *          
     *          Validates:
     *          - GPS lock status (2D/3D fix)
     *          - Minimum number of satellites visible
     *          - Horizontal Dilution of Precision (HDOP) within limits
     *          - GPS velocity accuracy sufficient
     *          - Position jump detection (glitching)
     *          - RTK fix status (if RTK GPS used)
     *          
     *          GPS requirements vary by flight mode - stricter for AUTO/GUIDED/LOITER,
     *          relaxed or bypassed for stabilize/alt-hold modes.
     * 
     * @param[in] display_failure If true, send GCS message describing failure
     * 
     * @return true if GPS meets copter requirements, false if check fails
     * 
     * @note GPS check may be skipped if flight mode doesn't require position hold
     * @note Called during pre_arm_checks if ARMING_CHECK includes GPS bit
     * @note Also validates GPS compass (if integrated GPS+compass unit)
     * 
     * @see mandatory_gps_checks() for additional copter GPS validation
     * @see AP_GPS::status() for GPS fix status
     * @see AP_Arming::gps_checks() for base validation
     */
    bool gps_checks(bool display_failure) override;
    
    /**
     * @brief Validate barometer health for copter altitude control
     * 
     * @details Checks barometric pressure sensor health with copter-specific
     *          altitude sensing requirements. Calls base AP_Arming::barometer_checks()
     *          then adds multicopter altitude-hold validation.
     *          
     *          Validates:
     *          - All configured barometers are healthy
     *          - Pressure readings are reasonable
     *          - Barometer calibration complete
     *          - Multi-barometer consistency
     *          - Altitude reading rate sufficient for control loop
     * 
     * @param[in] display_failure If true, send GCS message describing failure
     * 
     * @return true if barometer healthy for copter, false if check fails
     * 
     * @note Barometer critical for altitude-hold, loiter, and autonomous modes
     * @note Called during pre_arm_checks if ARMING_CHECK includes baro bit
     * 
     * @warning Barometer failure causes altitude hold modes to fail
     * 
     * @see AP_Baro::healthy() for barometer health status
     * @see AP_Arming::barometer_checks() for base validation
     */
    bool barometer_checks(bool display_failure) override;
    
    /**
     * @brief Validate power supply voltage for copter
     * 
     * @details Checks battery voltage and power board health with copter-specific
     *          power requirements. Calls base AP_Arming::board_voltage_checks()
     *          then adds multicopter-specific battery validation.
     *          
     *          Validates:
     *          - Battery voltage above minimum safe level
     *          - Power board voltages within acceptable ranges
     *          - Battery monitoring functioning correctly
     *          - Sufficient capacity for flight (if configured)
     *          - No power brownout conditions detected
     * 
     * @param[in] display_failure If true, send GCS message describing failure
     * 
     * @return true if power system healthy, false if voltage too low
     * 
     * @note Low battery at arming prevents takeoff when insufficient power
     *       for controlled flight and landing
     * @note Called during pre_arm_checks if ARMING_CHECK includes voltage bit
     * 
     * @warning Arming with low battery risks mid-flight power loss
     * 
     * @see AP_BattMonitor::healthy() for battery health status
     * @see AP_Arming::board_voltage_checks() for base validation
     */
    bool board_voltage_checks(bool display_failure) override;

    // NOTE! the following check functions *DO NOT* call into AP_Arming!
    
    /**
     * @brief Validate copter-specific parameter configuration
     * 
     * @details Checks that copter flight parameters are set to valid and safe
     *          values. This is a copter-only check that does NOT call base
     *          AP_Arming parameter validation.
     *          
     *          Validates copter parameters including:
     *          - Motor output ranges (MOT_* parameters)
     *          - Attitude controller gains (ATC_* parameters)  
     *          - Position controller limits (PSC_* parameters)
     *          - Flight mode specific parameters
     *          - Failsafe configuration parameters
     *          - Frame type and motor configuration consistency
     * 
     * @param[in] display_failure If true, send GCS message describing invalid parameter
     * 
     * @return true if parameters valid, false if invalid configuration detected
     * 
     * @note Parameter validation prevents arming with configuration that could
     *       cause unstable flight or control failures
     * @note Called during pre_arm_checks if ARMING_CHECK includes parameters bit
     * 
     * @warning Invalid parameters can cause loss of control or unexpected behavior
     */
    bool parameter_checks(bool display_failure);
    
    /**
     * @brief Validate object avoidance system health
     * 
     * @details Checks that object avoidance (OA) system is ready if enabled.
     *          Validates proximity sensors, path planning database, and avoidance
     *          algorithm initialization. This is copter-specific and does NOT
     *          call base AP_Arming checks.
     *          
     *          Only enforced if OA_TYPE parameter is non-zero (avoidance enabled).
     * 
     * @param[in] display_failure If true, send GCS message describing failure
     * 
     * @return true if OA system ready or disabled, false if check fails
     * 
     * @note Object avoidance requires working proximity sensors
     * @note Called during pre_arm_checks if object avoidance enabled
     * 
     * @see proximity_checks() for proximity sensor validation
     * @see AP_OADatabase for object avoidance database
     */
    bool oa_checks(bool display_failure);
    
    /**
     * @brief Execute additional mandatory GPS checks for copter
     * 
     * @details Performs copter-specific GPS validation beyond base AP_Arming
     *          GPS checks. Validates GPS requirements for the current flight mode
     *          and configuration. This does NOT call base AP_Arming GPS checks.
     *          
     *          Validates:
     *          - GPS required for current flight mode (AUTO, GUIDED, LOITER, etc.)
     *          - Home position set and valid
     *          - GPS altitude reading available
     *          - Origin position initialized for EKF
     * 
     * @param[in] display_failure If true, send GCS message describing failure
     * 
     * @return true if mandatory GPS requirements met, false if check fails
     * 
     * @note Flight modes requiring GPS cannot be armed without valid GPS
     * @note Called during pre_arm_checks after base GPS checks
     * 
     * @see gps_checks() for general GPS health validation
     */
    bool mandatory_gps_checks(bool display_failure);
    
    /**
     * @brief Validate that GCS failsafe is not active
     * 
     * @details Checks that Ground Control Station communication failsafe is
     *          not currently triggered. Prevents arming when GCS link has been
     *          lost, as this could indicate communication problems that would
     *          prevent pilot control during flight.
     *          
     *          This is copter-specific and does NOT call base AP_Arming checks.
     * 
     * @param[in] display_failure If true, send GCS message describing failure
     * 
     * @return true if GCS failsafe not active, false if in failsafe
     * 
     * @note GCS failsafe triggered when telemetry link lost for configured timeout
     * @note Prevents arming during communication problems
     * @note Called during pre_arm_checks
     * 
     * @see Copter::failsafe_gcs_check() for GCS failsafe state
     */
    bool gcs_failsafe_check(bool display_failure);
    
    /**
     * @brief Validate winch system ready if equipped
     * 
     * @details Checks that cargo winch is in safe state for arming. Validates
     *          winch position, health, and configuration. This is copter-specific
     *          and does NOT call base AP_Arming checks.
     *          
     *          Only enforced if winch is enabled via WINCH_ENABLE parameter.
     * 
     * @param[in] display_failure If true, send GCS message describing failure
     * 
     * @return true if winch safe or disabled, false if check fails
     * 
     * @note Winch must be retracted or in neutral position to arm
     * @note Called during pre_arm_checks if winch enabled
     * 
     * @see AP_Winch for winch control system
     */
    bool winch_checks(bool display_failure) const;
    
    /**
     * @brief Validate altitude within configured limits
     * 
     * @details Checks that current altitude is below maximum altitude limit
     *          if configured. Prevents arming at excessive altitude where
     *          flight ceiling might be immediately exceeded.
     *          
     *          This is copter-specific and does NOT call base AP_Arming checks.
     * 
     * @param[in] display_failure If true, send GCS message describing failure
     * 
     * @return true if altitude within limits or unconfigured, false if too high
     * 
     * @note Only enforced if PILOT_ALT_MAX parameter is non-zero
     * @note Prevents arming near or above configured altitude ceiling
     * @note Called during pre_arm_checks
     * 
     * @warning Arming above altitude limit may cause immediate failsafe
     */
    bool alt_checks(bool display_failure);
    
    /**
     * @brief Validate that RC throttle failsafe is not active
     * 
     * @details Checks that RC receiver throttle failsafe is not currently
     *          triggered. Prevents arming when RC link has been lost or
     *          throttle signal is invalid, as this indicates control problems.
     *          
     *          This is copter-specific and does NOT call base AP_Arming checks.
     * 
     * @param[in] display_failure If true, send GCS message describing failure
     * 
     * @return true if throttle signal valid, false if failsafe active
     * 
     * @note Throttle failsafe triggered by RC link loss or invalid signal
     * @note Prevents arming during RC control problems
     * @note Called during pre_arm_checks
     * 
     * @see RC_Channel::get_radio_in() for RC input status
     */
    bool rc_throttle_failsafe_checks(bool display_failure) const;

    /**
     * @brief Enable or disable pre-arm check execution
     * 
     * @details Controls whether pre-arm checks are actively running. Used by
     *          the main scheduler to enable pre-arm checks when disarmed and
     *          disable them when armed to reduce CPU load during flight.
     * 
     * @param[in] b true to enable pre-arm checks, false to disable
     * 
     * @note Pre-arm checks typically run at 1Hz when enabled while disarmed
     * @note Checks are automatically disabled during flight
     * @note Even when disabled, checks still run during arm attempt
     * 
     * @see Copter::update_arming_checks() for scheduler integration
     */
    void set_pre_arm_check(bool b);

    /**
     * @brief Check if terrain database is required for current configuration
     * 
     * @details Determines whether terrain elevation database must be fully
     *          loaded before arming. Returns true if current flight mode or
     *          configured features require terrain data (e.g., terrain-following
     *          missions, surface tracking modes).
     *          
     *          Overrides base AP_Arming to add copter-specific terrain requirements.
     * 
     * @return true if terrain database must be loaded to arm, false otherwise
     * 
     * @note Terrain database required for:
     *       - Terrain-following AUTO missions with terrain altitude waypoints
     *       - SURFACE_TRACKING modes using rangefinder
     *       - Terrain-aware fence enabled
     * @note Called during pre_arm_checks to validate terrain data availability
     * 
     * @see AP_Terrain for terrain elevation database
     * @see Copter::surface_tracking for surface tracking system
     */
    bool terrain_database_required() const override;

private:

    /**
     * @brief Internal implementation of pre-arm check execution
     * 
     * @details Contains the actual pre-arm check logic that validates all
     *          copter safety requirements. This method is wrapped by the
     *          public pre_arm_checks() method which caches results for
     *          consistent reporting during the arming attempt window.
     *          
     *          Executes checks in sequence, with each check potentially
     *          sending GCS messages if display_failure is true. Checks
     *          include sensor health, calibration, parameters, GPS, battery,
     *          and flight mode readiness validation.
     *          
     *          The wrapper pattern allows caching check results to avoid
     *          redundant sensor queries and provide consistent pre-arm
     *          status during rapid arming attempts.
     * 
     * @param[in] display_failure If true, send GCS message for each failed check
     * 
     * @return true if all enabled pre-arm checks pass, false if any check fails
     * 
     * @note Called by pre_arm_checks() wrapper method
     * @note Check execution controlled by ARMING_CHECK bitmask parameter
     * @note Results cached by wrapper to avoid redundant validation
     * @note Private to enforce use of wrapper for consistent behavior
     * 
     * @see pre_arm_checks() for public interface with result caching
     */
    bool run_pre_arm_checks(bool display_failure);

};
