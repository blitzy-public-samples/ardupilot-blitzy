/**
 * @file AP_Arming_Plane.cpp
 * @brief Fixed-wing specific arming checks and validation for ArduPlane
 * 
 * @details This file implements the plane-specific arming safety system that extends
 *          the base AP_Arming class with fixed-wing aircraft validation requirements.
 *          
 *          The arming system provides multiple layers of safety validation:
 *          - Pre-arm checks: Comprehensive sensor, configuration, and safety validation
 *          - Arm checks: Final validation immediately before arming
 *          - Mandatory checks: Minimal checks when arming is disabled
 *          - Plane-specific checks: Airspeed calibration, control surface setup, pitch/roll limits
 *          - Quadplane checks: Additional validation for VTOL-capable aircraft
 *          
 *          Safety Philosophy:
 *          The arming system is designed to prevent flight with misconfigured or
 *          malfunctioning systems. Each check validates a specific aspect of flight
 *          readiness and can independently prevent arming to ensure safe operation.
 *          
 *          Key Validation Areas:
 *          - Sensor health and calibration (IMU, compass, airspeed, GPS)
 *          - Control surface configuration and limits
 *          - Failsafe system configuration
 *          - Flight mode readiness
 *          - RC receiver connectivity
 *          - Mission validity
 *          - Terrain database availability
 *          - Quadplane-specific systems (if enabled)
 *          
 * @warning This is safety-critical code. All arming checks must be thoroughly tested.
 *          Bypassing or weakening arming checks can lead to vehicle crashes or loss.
 *          
 * @note Arming checks can be bypassed after watchdog reset to allow in-flight recovery
 *       via telemetry for BVLOS (Beyond Visual Line of Sight) operations.
 * 
 * @see AP_Arming for base arming system implementation
 * @see Plane.h for main vehicle control interface
 * 
 * Source: ArduPlane/AP_Arming_Plane.cpp
 */
#include "AP_Arming_Plane.h"
#include "Plane.h"

#include "qautotune.h"

constexpr uint32_t AP_ARMING_DELAY_MS = 2000; // delay from arming to start of motor spoolup

const AP_Param::GroupInfo AP_Arming_Plane::var_info[] = {
    // variables from parent vehicle
    AP_NESTEDGROUPINFO(AP_Arming, 0),

    // index 3 was RUDDER and should not be used

#if AP_PLANE_BLACKBOX_LOGGING
    // @Param: BBOX_SPD
    // @DisplayName: Blackbox speed
    // @Description: This is a 3D GPS speed threshold above which we will force arm the vehicle to start logging. WARNING: This should only be used on a vehicle with no propellers attached to the flight controller and when the flight controller is not in control of the vehicle.
    // @Units: m/s
    // @Increment: 1
    // @Range: 1 20
    // @User: Advanced
    AP_GROUPINFO("BBOX_SPD", 4, AP_Arming_Plane, blackbox_speed, 5),
#endif // AP_PLANE_BLACKBOX_LOGGING
    
    AP_GROUPEND
};

/**
 * @brief Check if terrain database is required for arming
 * 
 * @details Determines whether the terrain database must have all data loaded
 *          before the vehicle can arm. This is required when terrain following
 *          is enabled to ensure safe flight operations.
 *          
 *          Terrain following requires accurate terrain data to maintain
 *          a safe altitude above ground level during autonomous flight.
 * 
 * @return true if terrain database must be fully loaded before arming
 * @return false if terrain database is optional for current configuration
 * 
 * @note Terrain following is configured via the TERRAIN_FOLLOW parameter
 * 
 * Source: ArduPlane/AP_Arming_Plane.cpp:33-41
 */
bool AP_Arming_Plane::terrain_database_required() const
{
#if AP_TERRAIN_AVAILABLE
    if (plane.g.terrain_follow) {
        return true;
    }
#endif
    return AP_Arming::terrain_database_required();
}

/**
 * @brief Perform comprehensive pre-arming checks for fixed-wing aircraft
 * 
 * @details This is the primary safety validation function that performs extensive
 *          checks before allowing the vehicle to arm. It validates sensors,
 *          configuration parameters, failsafe settings, and flight mode readiness.
 *          
 *          Check Categories:
 *          1. System Initialization: Verify scheduler and hardware are ready
 *          2. Parent Class Checks: Base AP_Arming validations (sensors, battery, GPS, etc.)
 *          3. Airspeed Sensor: Calibration and health checks
 *          4. Failsafe Configuration: FS_LONG_TIMEOUT vs FS_SHORT_TIMEOUT validation
 *          5. Attitude Limits: ROLL_LIMIT_DEG, PTCH_LIM_MAX_DEG, PTCH_LIM_MIN_DEG ranges
 *          6. Airspeed Configuration: AIRSPEED_MIN validation
 *          7. Throttle Configuration: Reverse throttle failsafe validation
 *          8. RC Receiver: Connectivity check if RC failsafe enabled
 *          9. Quadplane Systems: VTOL-specific validations (if HAL_QUADPLANE_ENABLED)
 *          10. ADSB Avoidance: Check for collision threats
 *          11. Throttle Trim: Center throttle validation (if CENTER_THROTTLE_TRIM enabled)
 *          12. Mission State: Prevent arming in landing sequence
 *          13. Flight Mode: Mode-specific pre-arm checks
 *          
 *          Watchdog Reset Bypass:
 *          After a watchdog reset, arming checks are bypassed to allow in-flight
 *          recovery via telemetry for BVLOS operations. This enables the operator
 *          to command arming and RTL after an in-flight reset.
 *          
 * @param[in] display_failure If true, send failure messages to GCS and ground station
 *                             If false, silently check without reporting
 * 
 * @return true if all pre-arm checks pass and vehicle is safe to arm
 * @return false if any check fails, preventing arming for safety
 * 
 * @warning This is safety-critical code. Each check validates a specific safety
 *          requirement. Disabling or bypassing checks can lead to crashes.
 *          
 * @note Checks can be disabled by setting ARMING_CHECK parameter to 0, which
 *       triggers mandatory_checks() instead of full validation.
 *       
 * @note If already armed, checks are skipped (returns true immediately)
 * 
 * @see mandatory_checks() for minimal checks when arming validation is disabled
 * @see AP_Arming::pre_arm_checks() for base class validation
 * @see quadplane_checks() for VTOL-specific validation
 * 
 * Source: ArduPlane/AP_Arming_Plane.cpp:47-144
 */
bool AP_Arming_Plane::pre_arm_checks(bool display_failure)
{
    if (armed || require == (uint8_t)Required::NO) {
        // if we are already armed or don't need any arming checks
        // then skip the checks
        return true;
    }
    if (!hal.scheduler->is_system_initialized()) {
        check_failed(display_failure, "System not initialised");
        return false;
    }
    //are arming checks disabled?
    if (checks_to_perform == 0) {
        return mandatory_checks(display_failure);
    }
    if (hal.util->was_watchdog_armed()) {
        // on watchdog reset bypass arming checks to allow for
        // in-flight arming if we were armed before the reset. This
        // allows a reset on a BVLOS flight to return home if the
        // operator can command arming over telemetry
        return true;
    }

    // call parent class checks
    bool ret = AP_Arming::pre_arm_checks(display_failure);

#if AP_AIRSPEED_ENABLED
    // Check airspeed sensor
    ret &= AP_Arming::airspeed_checks(display_failure);
#endif

    // Validate failsafe timeout configuration
    // Long timeout must be >= short timeout to ensure proper failsafe escalation
    // Short failsafe triggers first, then long failsafe for extended loss
    if (plane.g.fs_timeout_long < plane.g.fs_timeout_short && plane.g.fs_action_short != FS_ACTION_SHORT_DISABLED) {
        check_failed(display_failure, "FS_LONG_TIMEOUT < FS_SHORT_TIMEOUT");
        ret = false;
    }

    // Validate attitude limit parameters for safe flight envelope
    // Minimum 3 degrees required for each axis to allow effective control
    // @warning Insufficient limits can prevent proper attitude control
    if (plane.aparm.roll_limit < 3) {
        check_failed(display_failure, "ROLL_LIMIT_DEG too small (%.1f)", plane.aparm.roll_limit.get());
        ret = false;
    }

    if (plane.aparm.pitch_limit_max < 3) {
        check_failed(display_failure, "PTCH_LIM_MAX_DEG too small (%.1f)", plane.aparm.pitch_limit_max.get());
        ret = false;
    }

    if (plane.aparm.pitch_limit_min > -3) {
        check_failed(display_failure, "PTCH_LIM_MIN_DEG too large (%.1f)", plane.aparm.pitch_limit_min.get());
        ret = false;
    }

    // Validate minimum airspeed configuration
    // AIRSPEED_MIN must meet safety threshold to prevent stall
    // @warning Airspeed below minimum can lead to loss of control
    if (plane.aparm.airspeed_min < MIN_AIRSPEED_MIN) {
        check_failed(display_failure, "AIRSPEED_MIN too low (%i < %i)", plane.aparm.airspeed_min.get(), MIN_AIRSPEED_MIN);
        ret = false;
    }

    // Validate throttle failsafe configuration for reverse throttle
    // With reversed throttle, failsafe value must be properly configured
    // to ensure safe failsafe activation
    if (plane.channel_throttle->get_reverse() && 
        plane.g.throttle_fs_enabled != Plane::ThrFailsafe::Disabled &&
        plane.g.throttle_fs_value < 
        plane.channel_throttle->get_radio_max()) {
        check_failed(display_failure, "Invalid THR_FS_VALUE for rev throttle");
        ret = false;
    }

    ret &= rc_received_if_enabled_check(display_failure);

#if HAL_QUADPLANE_ENABLED
    ret &= quadplane_checks(display_failure);
#endif

    // Check ADSB (Automatic Dependent Surveillance-Broadcast) collision avoidance
    // Prevent arming if ADSB system has detected a collision threat
    // @warning Arming with ADSB threat present could lead to mid-air collision
    if (plane.failsafe.adsb) {
        check_failed(display_failure, "ADSB threat detected");
        ret = false;
    }

    // Validate throttle trim position for CENTER_THROTTLE_TRIM flight option
    // When center throttle is enabled, trim must be near center (1250-1750 μs)
    // to ensure proper throttle control behavior
    if (plane.flight_option_enabled(FlightOptions::CENTER_THROTTLE_TRIM)){
       int16_t trim = plane.channel_throttle->get_radio_trim();
       if (trim < 1250 || trim > 1750) {
           check_failed(display_failure, "Throttle trim not near center stick(%u)",trim );
           ret = false;
       }
    }

    // Prevent arming if already in landing sequence without takeoff command
    // Mission must start with takeoff if in landing sequence to prevent
    // immediate landing attempt upon arming
    // @warning Arming in landing sequence can cause immediate uncontrolled landing
    if (plane.mission.get_in_landing_sequence_flag() &&
        !plane.mission.starts_with_takeoff_cmd()) {
        check_failed(display_failure,"In landing sequence");
        ret = false;
    }

    char failure_msg[50] {};
    if (!plane.control_mode->pre_arm_checks(ARRAY_SIZE(failure_msg), failure_msg)) {
        check_failed(display_failure, "%s %s", plane.control_mode->name(), failure_msg);
        return false;
    }

    return ret;
}

/**
 * @brief Perform minimal mandatory arming checks when full checks are disabled
 * 
 * @details This function provides a minimal set of safety checks when the operator
 *          has disabled comprehensive arming validation (ARMING_CHECK = 0).
 *          Even with checks disabled, certain critical validations are still required.
 *          
 *          Mandatory checks include:
 *          - RC receiver connectivity (if RC failsafe enabled)
 *          - Base class mandatory checks (core safety requirements)
 *          
 * @param[in] display_failure If true, send failure messages to GCS
 *                             If false, silently check without reporting
 * 
 * @return true if mandatory checks pass
 * @return false if critical mandatory check fails
 * 
 * @warning Even though this is the "minimal" check set, these checks are still
 *          safety-critical. Disabling arming checks is not recommended for normal
 *          operations and should only be used by experienced developers.
 * 
 * @note Called by pre_arm_checks() when checks_to_perform == 0
 * 
 * @see pre_arm_checks() for comprehensive validation
 * @see AP_Arming::mandatory_checks() for base class requirements
 * 
 * Source: ArduPlane/AP_Arming_Plane.cpp:146-156
 */
bool AP_Arming_Plane::mandatory_checks(bool display_failure)
{
    bool ret = true;

    ret &= rc_received_if_enabled_check(display_failure);

    // Call parent class checks
    ret &= AP_Arming::mandatory_checks(display_failure);

    return ret;
}


#if HAL_QUADPLANE_ENABLED
/**
 * @brief Perform quadplane-specific arming validation checks
 * 
 * @details Validates VTOL (Vertical Take-Off and Landing) specific configuration
 *          and systems for quadplane-equipped fixed-wing aircraft. These checks
 *          ensure the multicopter motors, controllers, and configuration are
 *          ready for safe VTOL operation.
 *          
 *          Quadplane Check Categories:
 *          1. Availability: Verify quadplane system is running
 *          2. Scheduler Rate: Ensure SCHED_LOOP_RATE >= 100 Hz for motor control
 *          3. Motor System: Validate motor configuration and readiness
 *          4. Angle Limits: Check Q_ANGLE_MAX is in valid range (1000-8000 centidegrees)
 *          5. Configuration Conflicts: Prevent tailsitter + tiltrotor simultaneous enable
 *          6. Subsystem Initialization: Verify tailsitter/tiltrotor setup complete
 *          7. Controller Validation: Position and attitude controller parameter checks
 *          8. Assist Speed: Verify Q_ASSIST_SPEED configured for non-tailsitters
 *          9. Forward Throttle: Validate Q_FWD_THR_USE disabled for tailsitters
 *          
 * @param[in] display_failure If true, send failure messages to GCS
 *                             If false, silently check without reporting
 * 
 * @return true if all quadplane checks pass or quadplane not enabled
 * @return false if any quadplane-specific check fails
 * 
 * @warning Quadplane systems require careful configuration. Misconfiguration can
 *          lead to loss of control during VTOL transitions or hover operations.
 *          
 * @note This function is only compiled when HAL_QUADPLANE_ENABLED is defined
 * @note Returns true immediately if quadplane is not enabled
 * 
 * @see pre_arm_checks() for comprehensive arming validation
 * @see QuadPlane class for VTOL implementation details
 * 
 * Source: ArduPlane/AP_Arming_Plane.cpp:159-234
 */
bool AP_Arming_Plane::quadplane_checks(bool display_failure)
{
    if (!plane.quadplane.enabled()) {
        return true;
    }

    if (!plane.quadplane.available()) {
        check_failed(display_failure, "Quadplane enabled but not running");
        return false;
    }

    bool ret = true;

    // Validate scheduler loop rate for quadplane motor control
    // Minimum 100 Hz required for stable multirotor attitude control
    // @warning Lower loop rates cause unstable hover and potential crashes
    if (plane.scheduler.get_loop_rate_hz() < 100) {
        check_failed(display_failure, "quadplane needs SCHED_LOOP_RATE >= 100");
        ret = false;
    }

    char failure_msg[50] {};
    if (!plane.quadplane.motors->arming_checks(ARRAY_SIZE(failure_msg), failure_msg)) {
        check_failed(display_failure, "Motors: %s", failure_msg);
        ret = false;
    }

    // lean angle parameter check
    if (plane.quadplane.aparm.angle_max < 1000 || plane.quadplane.aparm.angle_max > 8000) {
        check_failed(Check::PARAMETERS, display_failure, "Check Q_ANGLE_MAX");
        ret = false;
    }

    if ((plane.quadplane.tailsitter.enable > 0) && (plane.quadplane.tiltrotor.enable > 0)) {
        check_failed(Check::PARAMETERS, display_failure, "set TAILSIT_ENABLE 0 or TILT_ENABLE 0");
        ret = false;

    } else {

        if ((plane.quadplane.tailsitter.enable > 0) && !plane.quadplane.tailsitter.enabled()) {
            check_failed(Check::PARAMETERS, display_failure, "tailsitter setup not complete, reboot");
            ret = false;
        }

        if ((plane.quadplane.tiltrotor.enable > 0) && !plane.quadplane.tiltrotor.enabled()) {
            check_failed(Check::PARAMETERS, display_failure, "tiltrotor setup not complete, reboot");
            ret = false;
        }
    }

    // ensure controllers are OK with us arming:
    if (!plane.quadplane.pos_control->pre_arm_checks("PSC", failure_msg, ARRAY_SIZE(failure_msg))) {
        check_failed(Check::PARAMETERS, display_failure, "Bad parameter: %s", failure_msg);
        ret = false;
    }
    if (!plane.quadplane.attitude_control->pre_arm_checks("ATC", failure_msg, ARRAY_SIZE(failure_msg))) {
        check_failed(Check::PARAMETERS, display_failure, "Bad parameter: %s", failure_msg);
        ret = false;
    }

    // Validate Q_ASSIST_SPEED parameter for quadplane assistance
    // Q_ASSIST_SPEED enables VTOL motors to assist during fixed-wing flight
    // when airspeed drops below threshold, preventing stalls
    // @warning Tailsitters excepted, all quadplanes should have Q_ASSIST_SPEED configured
    //          to provide safety margin during low-speed flight
    if (check_enabled(Check::PARAMETERS) &&
        is_zero(plane.quadplane.assist.speed) &&
        !plane.quadplane.tailsitter.enabled()) {
        check_failed(display_failure,"Q_ASSIST_SPEED is not set");
        ret = false;
    }

    if ((plane.quadplane.tailsitter.enable > 0) && (plane.quadplane.q_fwd_thr_use != QuadPlane::FwdThrUse::OFF)) {
        check_failed(Check::PARAMETERS, display_failure, "set Q_FWD_THR_USE to 0");
        ret = false;
    }

    return ret;
}
#endif // HAL_QUADPLANE_ENABLED

/**
 * @brief Perform inertial navigation system (INS) checks for arming
 * 
 * @details Validates IMU (Inertial Measurement Unit) sensors and AHRS (Attitude
 *          and Heading Reference System) are calibrated and healthy before arming.
 *          This includes gyroscope, accelerometer calibration validation and
 *          AHRS convergence checks.
 *          
 *          INS Validation includes:
 *          - Parent class INS checks (gyro/accel calibration, consistency)
 *          - AHRS pre-arm validation (attitude estimation convergence)
 *          - Sensor health monitoring
 *          
 * @param[in] display_failure If true, send failure messages to GCS
 *                             If false, silently check without reporting
 * 
 * @return true if INS and AHRS checks pass
 * @return false if sensor calibration or health check fails
 * 
 * @warning INS failures indicate uncalibrated or malfunctioning sensors that
 *          will cause erratic flight behavior and potential loss of control.
 *          
 * @note AHRS checks ensure attitude estimation has converged before flight
 * 
 * @see AP_Arming::ins_checks() for base class validation
 * @see AP_AHRS::pre_arm_check() for attitude estimation validation
 * 
 * Source: ArduPlane/AP_Arming_Plane.cpp:236-253
 */
bool AP_Arming_Plane::ins_checks(bool display_failure)
{
    // call parent class checks
    if (!AP_Arming::ins_checks(display_failure)) {
        return false;
    }

    // additional plane specific checks
    if (check_enabled(Check::INS)) {
        char failure_msg[50] = {};
        if (!AP::ahrs().pre_arm_check(true, failure_msg, sizeof(failure_msg))) {
            check_failed(Check::INS, display_failure, "AHRS: %s", failure_msg);
            return false;
        }
    }

    return true;
}

/**
 * @brief Perform final immediate arming checks before allowing arm command
 * 
 * @details This function performs the final validation checks immediately before
 *          arming is permitted. It is called after pre_arm_checks() have passed
 *          and provides a last opportunity to prevent arming if conditions have
 *          changed since pre-arm validation.
 *          
 *          The arm_checks() function:
 *          - Validates arming checks are not disabled (checks_to_perform != 0)
 *          - Bypasses checks after watchdog reset for in-flight recovery
 *          - Calls parent class arm_checks() for base validation
 *          
 *          Watchdog Reset Bypass:
 *          After a watchdog-triggered reset during flight, arming checks are
 *          bypassed to allow the operator to arm via telemetry and command RTL
 *          for BVLOS (Beyond Visual Line of Sight) recovery scenarios.
 * 
 * @param[in] method Arming method used (rudder stick, GCS command, switch, etc.)
 *                   See AP_Arming::Method enum for available methods
 * 
 * @return true if immediate arming checks pass and vehicle can arm
 * @return false if any final check fails, preventing arming
 * 
 * @warning This is the final safety gate before motors can be armed. All checks
 *          must pass to ensure safe vehicle operation.
 *          
 * @note If checks_to_perform == 0, arming is allowed (checks disabled)
 * @note Watchdog reset bypass logs a warning message to GCS
 * 
 * @see pre_arm_checks() for comprehensive pre-arming validation
 * @see AP_Arming::arm_checks() for base class final checks
 * @see arm() for actual arming sequence after checks pass
 * 
 * Source: ArduPlane/AP_Arming_Plane.cpp:255-274
 */
bool AP_Arming_Plane::arm_checks(AP_Arming::Method method)
{

    //are arming checks disabled?
    if (checks_to_perform == 0) {
        return true;
    }

    if (hal.util->was_watchdog_armed()) {
        // on watchdog reset bypass arming checks to allow for
        // in-flight arming if we were armed before the reset. This
        // allows a reset on a BVLOS flight to return home if the
        // operator can command arming over telemetry
        gcs().send_text(MAV_SEVERITY_WARNING, "watchdog: Bypassing arming checks");
        return true;
    }

    // call parent class checks
    return AP_Arming::arm_checks(method);
}

/**
 * @brief Update hardware abstraction layer (HAL) soft-arm state
 * 
 * @details Synchronizes the HAL soft-armed state with the vehicle armed status.
 *          Soft-arm state controls safety features and hardware outputs.
 *          For quadplane-equipped aircraft, also updates quadplane motor arming.
 *          
 *          This function is called whenever the armed state changes to ensure
 *          all subsystems are notified of the state transition.
 * 
 * @note Soft-arm state affects:
 *       - Safety switch behavior
 *       - Motor output enable/disable
 *       - Servo output configuration
 *       - Hardware safety features
 * 
 * @see update_soft_armed() for detailed soft-arm state management
 * @see arm() and disarm() for state change triggers
 * 
 * Source: ArduPlane/AP_Arming_Plane.cpp:279-285
 */
void AP_Arming_Plane::change_arm_state(void)
{
    update_soft_armed();
#if HAL_QUADPLANE_ENABLED
    plane.quadplane.set_armed(hal.util->get_soft_armed());
#endif
}

/**
 * @brief Arm the vehicle and prepare for flight
 * 
 * @details Performs the complete arming sequence for fixed-wing aircraft after
 *          all arming checks have passed. This includes home position updates,
 *          motor arming, and flight mode initialization.
 *          
 *          Arming Sequence:
 *          1. Call parent class arm() to perform base arming operations
 *          2. Update home position to current location for relative altitude takeoffs
 *          3. Force home position via AHRS for accurate reference
 *          4. Update soft-armed state in HAL
 *          5. Enable arming delay for motor spool-up protection
 *          6. Perform mode-specific arming actions (AUTOLAND arm check)
 *          7. Initialize rudder takeoff timer for stick-over warning
 *          8. Send arming confirmation to GCS
 *          
 *          Home Position Update:
 *          The home position is set to the current location to ensure relative
 *          altitude commands (e.g., NAV_TAKEOFF) use the correct reference point.
 *          The AHRS height datum is reset to maintain consistency.
 * 
 * @param[in] method Arming method used (rudder stick, GCS, switch, etc.)
 *                   See AP_Arming::Method enum for available methods
 * @param[in] do_arming_checks If true, perform arming checks before arming
 *                              If false, skip checks (for forced arming scenarios)
 * 
 * @return true if arming successful and vehicle is armed
 * @return false if arming failed (checks failed or arming not permitted)
 * 
 * @warning Vehicle is armed and motors/propellers can start after this function
 *          returns true. Ensure clear area around aircraft.
 *          
 * @note A 2-second delay (AP_ARMING_DELAY_MS) is enforced after arming before
 *       full motor output is permitted to allow ESC initialization
 *       
 * @note For RUDDER arming method, a timer is started to warn if stick is held
 *       too long, which could interfere with takeoff
 * 
 * @see arm_checks() for validation before arming
 * @see disarm() for disarming sequence
 * @see change_arm_state() for HAL state synchronization
 * 
 * Source: ArduPlane/AP_Arming_Plane.cpp:287-323
 */
bool AP_Arming_Plane::arm(const AP_Arming::Method method, const bool do_arming_checks)
{
    if (!AP_Arming::arm(method, do_arming_checks)) {
        return false;
    }

    if (plane.update_home()) {
        // after update_home the home position could still be
        // different from the current_loc if the EKF refused the
        // resetHeightDatum call. If we are updating home then we want
        // to force the home to be the current_loc so relative alt
        // takeoffs work correctly
        if (plane.ahrs.set_home(plane.current_loc)) {
            // update current_loc
            plane.update_current_loc();
        }
    }

    change_arm_state();

    // rising edge of delay_arming oneshot
    delay_arming = true;

#if MODE_AUTOLAND_ENABLED
    plane.mode_autoland.arm_check();
#endif

    if (method == AP_Arming::Method::RUDDER) {
        // initialise the timer used to warn the user they're holding
        // their stick over:
        plane.takeoff_state.rudder_takeoff_warn_ms = AP_HAL::millis();
    }

    send_arm_disarm_statustext("Throttle armed");

    return true;
}

/**
 * @brief Disarm the vehicle and shutdown motors
 * 
 * @details Performs the complete disarming sequence for fixed-wing aircraft,
 *          shutting down motors and resetting flight systems to safe state.
 *          
 *          Disarming Sequence:
 *          1. Verify disarm is safe (not flying if using GCS or rudder method)
 *          2. Call parent class disarm() for base operations
 *          3. Reset mission (unless in AUTO mode)
 *          4. Suppress throttle in auto-throttle modes
 *          5. Disable quadplane air mode (if not assigned to switch)
 *          6. Update soft-armed state in HAL
 *          7. Save quadplane autotune parameters (if QAUTOTUNE was running)
 *          8. Reset airspeed change command state
 *          9. Clear takeoff direction for next flight
 *          10. Send disarm confirmation to GCS
 *          
 *          Safety Checks:
 *          When using GCS or rudder disarm methods, the vehicle must not be
 *          flying to prevent mid-air disarm. This check is bypassed for other
 *          disarm methods (e.g., emergency disarm, loss of control).
 * 
 * @param[in] method Disarm method used (rudder stick, GCS, emergency, etc.)
 *                   See AP_Arming::Method enum for available methods
 * @param[in] do_disarm_checks If true, verify it's safe to disarm
 *                              If false, force disarm without safety checks
 * 
 * @return true if disarm successful and vehicle is disarmed
 * @return false if disarm not permitted (e.g., still flying)
 * 
 * @warning Disarming while flying will cause immediate loss of power and crash.
 *          GCS and rudder disarm methods prevent this, but other methods may
 *          force disarm in emergency situations.
 *          
 * @note Mission is reset on disarm unless in AUTO mode, allowing mission resume
 * @note Throttle suppression prevents uncommanded throttle after disarm
 * @note Air mode is automatically disabled on disarm if not controlled by switch
 * 
 * @see arm() for arming sequence
 * @see is_flying() for flight state detection
 * @see change_arm_state() for HAL state synchronization
 * 
 * Source: ArduPlane/AP_Arming_Plane.cpp:328-375
 */
bool AP_Arming_Plane::disarm(const AP_Arming::Method method, bool do_disarm_checks)
{
    if (do_disarm_checks &&
        (AP_Arming::method_is_GCS(method) ||
         method == AP_Arming::Method::RUDDER)) {
        if (plane.is_flying()) {
            // don't allow mavlink or rudder disarm while flying
            return false;
        }
    }

    if (!AP_Arming::disarm(method, do_disarm_checks)) {
        return false;
    }
    if (plane.control_mode != &plane.mode_auto) {
        // reset the mission on disarm if we are not in auto
        plane.mission.reset();
    }

    // suppress the throttle in auto-throttle modes
    plane.throttle_suppressed = plane.control_mode->does_auto_throttle();

    // if no airmode switch assigned, ensure airmode is off:
#if HAL_QUADPLANE_ENABLED
    if ((plane.quadplane.air_mode == AirMode::ON) && (rc().find_channel_for_option(RC_Channel::AUX_FUNC::AIRMODE) == nullptr)) {
        plane.quadplane.air_mode = AirMode::OFF;
    }
#endif

    //only log if disarming was successful
    change_arm_state();

#if QAUTOTUNE_ENABLED
    // Possibly save auto tuned parameters
    plane.quadplane.qautotune.disarmed(plane.control_mode == &plane.mode_qautotune);
#endif

    // re-initialize speed variable used in AUTO and GUIDED for
    // DO_CHANGE_SPEED commands
    plane.new_airspeed_cm = -1;

#if MODE_AUTOLAND_ENABLED
    // takeoff direction always cleared on disarm
    plane.takeoff_state.initial_direction.initialized = false;
#endif
    send_arm_disarm_statustext("Throttle disarmed");
    return true;
}

/**
 * @brief Update soft-armed state and handle special arming modes
 * 
 * @details Manages the HAL soft-armed state and implements special arming features
 *          including quadplane motor testing and blackbox logging mode.
 *          
 *          Soft-Armed State Management:
 *          - Synchronizes HAL armed state with vehicle armed status
 *          - Considers quadplane motor test as "armed" for safety
 *          - Updates datalogger armed state for proper log marking
 *          - Manages arming delay timer for motor spool-up protection
 *          
 *          Blackbox Logging Mode (AP_PLANE_BLACKBOX_LOGGING):
 *          Special mode for data logging without flight control.
 *          - Auto-arms when 3D GPS speed exceeds threshold (ARMING_BBOX_SPD)
 *          - Auto-disarms after 20 seconds below speed threshold
 *          - Forces safety on to prevent motor output
 *          - Disables RC protocols for safety
 *          
 * @warning Blackbox mode should ONLY be used on vehicles with propellers removed
 *          and when the flight controller is not controlling the vehicle.
 *          
 * @note Arming delay (AP_ARMING_DELAY_MS = 2000ms) prevents immediate full throttle
 *       after arming, allowing ESCs to initialize safely
 *       
 * @note Motor test mode is treated as armed state to enable safety features
 *       during ground testing
 * 
 * @see change_arm_state() for state change notifications
 * @see arm() and disarm() for arming state transitions
 * 
 * Source: ArduPlane/AP_Arming_Plane.cpp:377-418
 */
void AP_Arming_Plane::update_soft_armed()
{
    bool _armed = is_armed();
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.motor_test.running){
        _armed = true;
    }
#endif

    hal.util->set_soft_armed(_armed);
#if HAL_LOGGING_ENABLED
    AP::logger().set_vehicle_armed(hal.util->get_soft_armed());
#endif

    // update delay_arming oneshot
    if (delay_arming &&
        (AP_HAL::millis() - hal.util->get_last_armed_change() >= AP_ARMING_DELAY_MS)) {

        delay_arming = false;
    }

#if AP_PLANE_BLACKBOX_LOGGING
    // Blackbox logging mode: Auto-arm/disarm based on GPS velocity for data collection
    // @warning EXTREMELY DANGEROUS if propellers are attached or FC controls the vehicle
    //          This mode is ONLY for data logging on moving platforms (e.g., cars, sleds)
    //          where the flight controller is monitoring but not controlling
    if (blackbox_speed > 0) {
        const float speed3d = plane.gps.status() >= AP_GPS::GPS_OK_FIX_3D?plane.gps.velocity().length():0;
        const uint32_t now = AP_HAL::millis();
        if (speed3d > blackbox_speed) {
            last_over_3dspeed_ms = now;
        }
        // Auto-arm when speed exceeds threshold
        if (!_armed && speed3d > blackbox_speed) {
            // Force safety on to prevent any motor output
            hal.rcout->force_safety_on();
            // Disable RC protocols for additional safety
            AP_Param::set_by_name("RC_PROTOCOLS", 0);
            arm(Method::BLACKBOX, false);
            gcs().send_text(MAV_SEVERITY_WARNING, "BlackBox: arming at %.1f m/s", speed3d);
        }
        // Auto-disarm after 20 seconds below speed threshold
        if (_armed && now - last_over_3dspeed_ms > 20000U) {
            gcs().send_text(MAV_SEVERITY_WARNING, "BlackBox: disarming at %.1f m/s", speed3d);
            disarm(Method::BLACKBOX, false);
        }
    }
#endif
}

/**
 * @brief Validate mission commands for fixed-wing aircraft compatibility
 * 
 * @details Performs plane-specific mission validation to ensure mission commands
 *          are compatible with current configuration and safe for execution.
 *          
 *          Mission Validation Checks:
 *          1. Base class mission checks (waypoint validity, command structure)
 *          2. RTL_AUTOLAND configuration vs DO_LAND_START usage
 *          3. RTL_AUTOLAND configuration vs DO_RETURN_PATH_START usage
 *          4. Quadplane VTOL landing approach distance validation
 *          
 *          RTL_AUTOLAND Checks:
 *          When RTL_AUTOLAND is disabled, missions cannot use DO_LAND_START or
 *          DO_RETURN_PATH_START commands as there is no landing sequence configured.
 *          
 *          Quadplane Landing Distance Check:
 *          For VTOL landings, validates there is sufficient distance between the
 *          final waypoint and landing point for the quadplane to decelerate safely.
 *          Minimum distance is 75% of the calculated stopping distance at landing speed.
 * 
 * @param[in] report If true, send failure messages to GCS
 *                   If false, silently validate without reporting
 * 
 * @return true if mission is valid and safe to execute
 * @return false if mission contains incompatible or unsafe commands
 * 
 * @warning Invalid missions can lead to unexpected flight behavior or inability
 *          to complete landing sequences safely.
 *          
 * @note Quadplane landing checks use TECS landing airspeed or cruise speed
 *       to calculate required stopping distance
 *       
 * @note Mission checks are performed during pre-arm validation if mission is loaded
 * 
 * @see pre_arm_checks() for when mission checks are triggered
 * @see AP_Arming::mission_checks() for base class validation
 * @see QuadPlane::stopping_distance() for VTOL deceleration calculation
 * 
 * Source: ArduPlane/AP_Arming_Plane.cpp:423-462
 */
bool AP_Arming_Plane::mission_checks(bool report)
{
    // base checks
    bool ret = AP_Arming::mission_checks(report);
    if (plane.g.rtl_autoland == RtlAutoland::RTL_DISABLE) {
        if (plane.mission.contains_item(MAV_CMD_DO_LAND_START)) {
            ret = false;
            check_failed(Check::MISSION, report, "DO_LAND_START set and RTL_AUTOLAND disabled");
        }
        if (plane.mission.contains_item(MAV_CMD_DO_RETURN_PATH_START)) {
            ret = false;
            check_failed(Check::MISSION, report, "DO_RETURN_PATH_START set and RTL_AUTOLAND disabled");
        }
    }
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.available()) {
        const uint16_t num_commands = plane.mission.num_commands();
        AP_Mission::Mission_Command prev_cmd {};
        for (uint16_t i=1; i<num_commands; i++) {
            AP_Mission::Mission_Command cmd;
            if (!plane.mission.read_cmd_from_storage(i, cmd)) {
                break;
            }
            if (plane.is_land_command(cmd.id) &&
                prev_cmd.id == MAV_CMD_NAV_WAYPOINT) {
                const float dist = cmd.content.location.get_distance(prev_cmd.content.location);
                const float tecs_land_speed = plane.TECS_controller.get_land_airspeed();
                const float landing_speed = is_positive(tecs_land_speed)?tecs_land_speed:plane.aparm.airspeed_cruise;
                const float min_dist = 0.75 * plane.quadplane.stopping_distance(sq(landing_speed));
                if (dist < min_dist) {
                    ret = false;
                    check_failed(Check::MISSION, report, "VTOL land too short, min %.0fm", min_dist);
                }
            }
            prev_cmd = cmd;
        }
    }
#endif
    return ret;
}

/**
 * @brief Check RC receiver connectivity if RC failsafe is enabled
 * 
 * @details Validates that RC (Radio Control) receiver has received valid signal
 *          before allowing arming, but only if RC protocols are enabled and
 *          RC failsafe is configured.
 *          
 *          Check Logic:
 *          - If no RC protocols enabled → Allow arming (RC not required)
 *          - If RC failsafe enabled → Require RC signal before arming
 *          - If RC failsafe disabled → Allow arming without RC
 *          
 *          This ensures that when the pilot has configured RC failsafe protection,
 *          the vehicle will not arm without a valid RC connection, preventing
 *          loss of control if RC link is not established.
 * 
 * @param[in] display_failure If true, send "Waiting for RC" message to GCS
 *                             If false, silently check without reporting
 * 
 * @return true if RC not required or valid RC signal received
 * @return false if RC failsafe enabled but no RC signal received
 * 
 * @note This check allows arming without RC if:
 *       - RC protocols are completely disabled (autonomous operation)
 *       - RC failsafe is disabled (operator accepts risk)
 *       
 * @note RC override (GCS control via MAVLink) counts as valid RC for this check
 * 
 * @see pre_arm_checks() where this check is called
 * @see mandatory_checks() where this check is also enforced
 * 
 * Source: ArduPlane/AP_Arming_Plane.cpp:465-480
 */
bool AP_Arming_Plane::rc_received_if_enabled_check(bool display_failure)
{
    if (rc().enabled_protocols() == 0) {
        // No protocols enabled, will never get RC, don't block arming
        return true;
    }

    // If RC failsafe is enabled we must receive RC before arming
    if ((plane.g.throttle_fs_enabled == Plane::ThrFailsafe::Enabled) &&
        !(rc().has_had_rc_receiver() || rc().has_had_rc_override())) {
        check_failed(display_failure, "Waiting for RC");
        return false;
    }

    return true;
}
