/**
 * @file AP_Arming_Rover.cpp
 * @brief Implementation of rover-specific pre-arm and arming checks
 * 
 * @details This file implements the AP_Arming_Rover class which extends AP_Arming
 *          with rover-specific safety checks before allowing the vehicle to arm.
 *          
 *          Key responsibilities:
 *          - RC calibration validation for steering and throttle channels
 *          - GPS health and EKF status verification
 *          - Mode-specific arming permission checks
 *          - Motor and ESC readiness validation
 *          - Object avoidance system initialization checks
 *          - Parameter validity verification
 *          - Soft-armed state propagation to HAL and logging
 *          
 *          The pre-arm checks are safety-critical and prevent the vehicle from arming
 *          in unsafe conditions that could lead to loss of control or crashes.
 *          
 *          Pre-arm checks run before arming is allowed. Arm checks run during the
 *          arming process itself. Both must pass for the vehicle to arm successfully.
 * 
 * @warning All checks in this file are safety-critical. Modifications could allow
 *          the vehicle to arm in unsafe conditions leading to crashes or injury.
 * 
 * Source: Rover/AP_Arming_Rover.cpp
 */

#include "AP_Arming_Rover.h"
#include "Rover.h"

/**
 * @brief Perform RC calibration checks for steering and throttle channels
 * 
 * @details Validates that the RC radio has been properly calibrated by checking
 *          that the min/max PWM values are within acceptable ranges. This ensures
 *          the rover can receive valid control inputs from the transmitter.
 *          
 *          Checks performed:
 *          - Steering channel min PWM > RC_CALIB_MIN_LIMIT_PWM (typically 1300)
 *          - Steering channel max PWM < RC_CALIB_MAX_LIMIT_PWM (typically 1700)
 *          - Throttle channel min PWM > RC_CALIB_MIN_LIMIT_PWM
 *          - Throttle channel max PWM < RC_CALIB_MAX_LIMIT_PWM
 * 
 * @param[in] display_failure If true, send failure messages to GCS
 * 
 * @return true if RC calibration is valid or checks are disabled, false otherwise
 * 
 * @note This check can be disabled via the ARMING_CHECK parameter bitmask
 * @warning Invalid RC calibration can result in limited control authority or
 *          reversed controls leading to loss of vehicle control
 */
bool AP_Arming_Rover::rc_calibration_checks(const bool display_failure)
{
    // set rc-checks to success if RC checks are disabled
    if (!check_enabled(Check::RC)) {
        return true;
    }

    const RC_Channel *channels[] = {
        rover.channel_steer,
        rover.channel_throttle
    };
    const char *channel_names[] = {"Steer", "Throttle"};

    for (uint8_t i= 0 ; i < ARRAY_SIZE(channels); i++) {
        const RC_Channel *channel = channels[i];
        const char *channel_name = channel_names[i];
        // check if radio has been calibrated
        if (channel->get_radio_min() > RC_Channel::RC_CALIB_MIN_LIMIT_PWM) {
            check_failed(Check::RC, display_failure, "%s radio min too high", channel_name);
            return false;
        }
        if (channel->get_radio_max() < RC_Channel::RC_CALIB_MAX_LIMIT_PWM) {
            check_failed(Check::RC, display_failure, "%s radio max too low", channel_name);
            return false;
        }
    }
    return AP_Arming::rc_calibration_checks(display_failure);
}

/**
 * @brief Perform GPS and navigation system health checks
 * 
 * @details Validates that GPS and Extended Kalman Filter (EKF) systems are healthy
 *          and providing valid position/velocity estimates. This is critical for
 *          modes that require position hold or autonomous navigation.
 *          
 *          Checks performed:
 *          1. Skip if current mode doesn't require position or velocity
 *          2. Call parent GPS checks (satellite count, HDOP, fix type)
 *          3. Verify AHRS system has initialized and passed pre-arm checks
 *          4. Check that EKF failsafe is not active
 *          5. Verify position estimate quality via ekf_position_ok()
 *          
 *          The position estimate check ensures the EKF has converged and is
 *          providing reliable position data before allowing autonomous operation.
 * 
 * @param[in] display_failure If true, send failure messages to GCS
 * 
 * @return true if GPS and navigation systems are healthy, false otherwise
 * 
 * @note Position checks are skipped in manual-only modes (MANUAL, ACRO)
 * @warning Poor GPS/EKF health can cause flyaways in autonomous modes or
 *          position-hold failures leading to crashes
 */
bool AP_Arming_Rover::gps_checks(bool display_failure)
{
    if (!require_location &&
        !rover.control_mode->requires_position() &&
        !rover.control_mode->requires_velocity()) {
        // we don't care!
        return true;
    }

    // call parent gps checks
    if (!AP_Arming::gps_checks(display_failure)) {
        return false;
    }

    const AP_AHRS &ahrs = AP::ahrs();

    // always check if inertial nav has started and is ready
    char failure_msg[50] = {};
    if (!ahrs.pre_arm_check(true, failure_msg, sizeof(failure_msg))) {
        check_failed(display_failure, "AHRS: %s", failure_msg);
        return false;
    }

    // check for ekf failsafe
    if (rover.failsafe.ekf) {
        check_failed(display_failure, "EKF failsafe");
        return false;
    }

    // ensure position estimate is ok
    if (!rover.ekf_position_ok()) {
        // vehicle level position estimate checks
        check_failed(display_failure, "Need Position Estimate");
        return false;
    }

    return true;
}

/**
 * @brief Master pre-arm check function that orchestrates all rover-specific checks
 * 
 * @details This is the main entry point for pre-arm validation before the rover
 *          can be armed. It coordinates multiple safety checks including GPS health,
 *          EKF status, mode validity, motor configuration, and parameter validation.
 *          
 *          Check sequence:
 *          1. Skip all checks if already armed (prevents mid-flight check failures)
 *          2. Verify system initialization complete (HAL, libraries loaded)
 *          3. If checks disabled, run only mandatory_checks() and return
 *          4. Sailboat-specific: Verify WindVane enabled if sailing mode active
 *          5. Run parent class checks: GPS, compass, battery, rangefinder, etc.
 *          6. Run motor_checks: Motor configuration and ESC readiness
 *          7. Run oa_check: Object avoidance system initialization (if enabled)
 *          8. Run parameter_checks: Critical parameter validity (WP_SPEED, etc.)
 *          9. Run mode_checks: Verify current mode allows arming
 *          
 *          All checks use bitwise-AND (&) to ensure ALL checks run even if early
 *          checks fail, providing complete failure feedback to the pilot/GCS.
 * 
 * @param[in] report If true, send failure messages to GCS for pilot feedback
 * 
 * @return true if all pre-arm checks pass, false if any check fails
 * 
 * @note Bitwise-AND (&) is intentionally used instead of logical-AND (&&) to
 *       ensure all check functions execute even after a failure, providing
 *       comprehensive failure reporting to the pilot
 * 
 * @warning This is a safety-critical function. All checks must pass before arming.
 *          Bypassing or weakening these checks could allow arming in unsafe
 *          conditions leading to loss of control, crashes, or injury.
 * 
 * @see AP_Arming::pre_arm_checks() for base class checks
 * @see motor_checks() for motor configuration validation
 * @see mode_checks() for mode arming permission validation
 */
bool AP_Arming_Rover::pre_arm_checks(bool report)
{
    // Safety check: If already armed, skip pre-arm checks to avoid mid-operation failures
    if (armed) {
        // if we are already armed then skip the checks
        return true;
    }

    // Critical check: Ensure HAL scheduler and core libraries have completed initialization
    // Arming before initialization could cause undefined behavior or crashes
    if (!hal.scheduler->is_system_initialized()) {
        check_failed(report, "System not initialised");
        return false;
    }

    // If operator has disabled all arming checks (ARMING_CHECK=0), only run mandatory checks
    // Mandatory checks include critical safety items that cannot be bypassed
    //are arming checks disabled?
    if (checks_to_perform == 0) {
        return mandatory_checks(report);
    }

    // Sailboat-specific safety check: Sailing mode requires wind vane for wind direction
    // Without wind sensing, sailboat cannot navigate or control sails safely
    if (rover.g2.sailboat.sail_enabled() && !rover.g2.windvane.enabled()) {
        check_failed(report, "Sailing enabled with no WindVane");
        return false;
    }

    // Run all pre-arm checks using bitwise-AND to ensure comprehensive failure reporting
    // Bitwise-AND (&) ensures ALL checks execute even if earlier checks fail, unlike
    // logical-AND (&&) which would short-circuit on first failure
    // 
    // Check hierarchy:
    // 1. AP_Arming::pre_arm_checks - Base class checks (GPS, compass, battery, etc.)
    // 2. motor_checks - Motor/ESC configuration and readiness
    // 3. oa_check - Object avoidance system initialization (if compiled in)
    // 4. parameter_checks - Critical parameter validity (WP_SPEED positive, etc.)
    // 5. mode_checks - Current mode allows arming
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wbitwise-instead-of-logical"
    return (AP_Arming::pre_arm_checks(report)
            & motor_checks(report)
#if AP_OAPATHPLANNER_ENABLED
            & oa_check(report)
#endif
            & parameter_checks(report)
            & mode_checks(report));
#pragma clang diagnostic pop
}

/**
 * @brief Perform final checks during the arming sequence based on arming method
 * 
 * @details This function runs during the actual arming process after pre_arm_checks
 *          have passed. It validates that the requested arming method is appropriate
 *          for the current flight mode.
 *          
 *          Arming methods:
 *          - RUDDER: Arming via full right rudder stick held for 2 seconds
 *          - MAVLINK: Arming via ground control station command
 *          - SWITCH: Arming via auxiliary switch
 *          - SCRIPTING: Arming via Lua script
 *          
 *          For rudder arming, verifies the current mode permits stick-based arming
 *          (e.g., MANUAL, ACRO allow rudder arming; GUIDED may not).
 * 
 * @param[in] method The arming method being attempted (RUDDER, MAVLINK, etc.)
 * 
 * @return true if arming via this method is allowed, false otherwise
 * 
 * @note If ARMING_CHECK=0 (all checks disabled), this function returns true
 * @warning Bypassing mode-specific arming restrictions could allow arming in
 *          inappropriate modes leading to unexpected vehicle behavior
 */
bool AP_Arming_Rover::arm_checks(AP_Arming::Method method)
{
    // Rudder arming requires mode-specific permission
    // Some modes like GUIDED or AUTO may restrict stick-based arming for safety
    if (method == AP_Arming::Method::RUDDER) {
        // check if arming/disarming allowed from this mode
        if (!rover.control_mode->allows_arming_from_transmitter()) {
            check_failed(true, "Mode not rudder-armable");
            return false;
        }
    }

    // If operator has disabled all arming checks, bypass arm checks
    //are arming checks disabled?
    if (checks_to_perform == 0) {
        return true;
    }
    return AP_Arming::arm_checks(method);
}

/**
 * @brief Update the soft-armed state and propagate to HAL and logging systems
 * 
 * @details The "soft-armed" state represents whether the vehicle is truly ready
 *          to apply throttle to motors/ESCs. This is distinct from the logical
 *          armed state and incorporates the hardware safety switch position.
 *          
 *          Soft-armed state determination:
 *          - Vehicle must be logically armed (is_armed() == true)
 *          - AND hardware safety switch must not be in DISARMED position
 *          
 *          The soft-armed state is propagated to:
 *          1. HAL layer: Controls motor output enable at hardware level
 *          2. Logger: Records armed state in binary logs for analysis
 *          
 *          State propagation flow:
 *          1. Check logical armed state (set by arm() function)
 *          2. Check hardware safety switch state (physical switch or button)
 *          3. Compute soft-armed = armed AND safety_not_disarmed
 *          4. Propagate to HAL via hal.util->set_soft_armed()
 *          5. Propagate to logger via AP::logger().set_vehicle_armed()
 *          
 *          This two-level arming system provides defense-in-depth:
 *          - Software arming checks prevent unsafe arming conditions
 *          - Hardware safety switch provides physical motor disable override
 * 
 * @note Called from arm() and disarm() functions to update state after transitions
 * @note The hardware safety switch can disable motors even when logically armed
 * 
 * @warning This function directly controls motor enable state. The soft-armed
 *          state gates all motor output. Incorrect implementation could allow
 *          motors to spin when disarmed (dangerous) or prevent motors from
 *          spinning when armed (mission failure).
 * 
 * @see arm() for the arming procedure that sets logical armed state
 * @see disarm() for the disarming procedure that clears logical armed state
 */
void AP_Arming_Rover::update_soft_armed()
{
    // Calculate soft-armed state: Logical armed AND safety switch not disarmed
    // This combines software arming state with hardware safety switch override
    // 
    // is_armed() returns the logical armed state set by arm() function
    // safety_switch_state() returns hardware safety switch position:
    //   - SAFETY_ARMED: Safety switch pressed/enabled, motors can run
    //   - SAFETY_DISARMED: Safety switch not pressed, motors force-disabled
    //   - SAFETY_NONE: No safety switch present, always allow if armed
    //
    // Result: Motors can only spin when both software AND hardware permit
    hal.util->set_soft_armed(is_armed() &&
                             hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED);
    
    // Propagate the soft-armed state to the logging system for telemetry and analysis
    // This records armed state transitions in binary logs (*.BIN files) for post-flight
    // review and debugging. The ARMED message in logs reflects soft-armed, not logical armed.
#if HAL_LOGGING_ENABLED
    AP::logger().set_vehicle_armed(hal.util->get_soft_armed());
#endif
}

/**
 * @brief Arm the rover motors and initialize navigation systems
 * 
 * @details This function arms the vehicle, enabling motor output and initializing
 *          autonomous navigation systems. It is called after all pre-arm and arm
 *          checks have passed successfully.
 *          
 *          Arming sequence:
 *          1. Call parent arm() to set logical armed state and log arming event
 *          2. If parent fails, trigger arming_failed notification and abort
 *          3. Set SmartRTL home location for intelligent return-to-launch path
 *          4. Initialize simple mode heading reference for simplified control
 *          5. Record home heading for sailboat wind-relative navigation
 *          6. Update soft-armed state to enable motor output at HAL level
 *          7. Send "Throttle armed" message to GCS and pilot
 *          
 *          After successful arming:
 *          - Motors can spin when throttle applied
 *          - Autonomous modes can command throttle
 *          - Failsafes are active and will trigger if conditions occur
 *          - SmartRTL begins recording path for return navigation
 * 
 * @param[in] method Arming method (RUDDER, MAVLINK, SWITCH, SCRIPTING)
 * @param[in] do_arming_checks If true, perform arm_checks(); if false, bypass checks
 * 
 * @return true if arming successful, false if arming failed
 * 
 * @note Setting do_arming_checks=false bypasses safety checks and should only be
 *       used in controlled testing environments, never in normal operation
 * 
 * @warning Arming enables motor output. Ensure vehicle is in a safe location with
 *          adequate clearance before arming. Keep clear of wheels/motors after arming.
 * 
 * @see disarm() for the disarming procedure
 * @see update_soft_armed() for motor enable state propagation
 */
bool AP_Arming_Rover::arm(AP_Arming::Method method, const bool do_arming_checks)
{
    // Attempt to arm via parent class (performs arm_checks if do_arming_checks=true)
    // Parent class sets logical armed state and logs ARM event to dataflash
    if (!AP_Arming::arm(method, do_arming_checks)) {
        // Arming failed - trigger notification LED/sound pattern for pilot feedback
        AP_Notify::events.arming_failed = true;
        return false;
    }

    // Initialize SmartRTL home location - SmartRTL will return to this point if activated
    // SmartRTL records the path taken and returns along the safest reverse path
    // Set home to current location (true = use current position)
    rover.g2.smart_rtl.set_home(true);

    // Initialize simple mode heading reference to current vehicle heading
    // Simple mode uses this reference to convert pilot stick inputs to earth-frame commands
    rover.mode_simple.init_heading();

    // Record home heading for sailboat wind-relative navigation
    // Sailboats use wind direction relative to home heading for autonomous sail control
    rover.g2.windvane.record_home_heading();

    // Update soft-armed state: Propagate armed state to HAL (enables motor output)
    // and to logger (records arming in binary logs)
    update_soft_armed();

    // Notify pilot and GCS that throttle is now active
    send_arm_disarm_statustext("Throttle armed");

    return true;
}

/**
 * @brief Disarm the rover motors and reset navigation state
 * 
 * @details This function disarms the vehicle, disabling motor output and resetting
 *          certain navigation systems. It can be called via multiple methods including
 *          rudder stick, GCS command, or auxiliary switch.
 *          
 *          Disarming safety checks:
 *          1. Rudder disarm: Reject if motors currently active (prevents accidental
 *             disarm while driving, as full rudder is common during maneuvers)
 *          2. Call parent disarm checks (may verify vehicle stopped, landed, etc.)
 *          
 *          Disarming sequence:
 *          1. Perform method-specific safety checks (rudder requires motors stopped)
 *          2. Call parent disarm() to clear logical armed state and log disarm event
 *          3. Reset mission if not in AUTO mode (prevents mission resume in manual modes)
 *          4. Update soft-armed state to disable motor output at HAL level
 *          5. Send "Throttle disarmed" message to GCS and pilot
 *          
 *          After successful disarming:
 *          - Motor output disabled at hardware level
 *          - Throttle stick/commands will not spin motors
 *          - SmartRTL path recording paused
 *          - Mission reset (if not in AUTO mode)
 * 
 * @param[in] method Disarming method (RUDDER, MAVLINK, SWITCH, SCRIPTING)
 * @param[in] do_disarm_checks If true, perform disarm safety checks; if false, bypass
 * 
 * @return true if disarming successful, false if disarming rejected
 * 
 * @note Rudder disarm requires motors to be stopped to prevent accidental disarm
 *       during aggressive driving maneuvers where full rudder is common
 * @note Mission is reset on disarm in manual modes to prevent unintended mission
 *       resume when re-arming in a different location
 * 
 * @warning Disarming while moving will cause loss of motor control. Vehicle will
 *          coast to a stop with no active braking or steering.
 * 
 * @see arm() for the arming procedure
 * @see update_soft_armed() for motor disable state propagation
 */
bool AP_Arming_Rover::disarm(const AP_Arming::Method method, bool do_disarm_checks)
{
    // Special safety check for rudder disarm: Prevent disarm while motors are actively driving
    // Full right rudder (disarm stick position) is commonly used during tight turns,
    // so we reject rudder disarm when motors are spinning to prevent accidental disarm
    if (method == AP_Arming::Method::RUDDER) {
        if (rover.g2.motors.active()) {
            // can't emit a message here as full-rudder while driving
            // is not uncommon
            return false;
        }
    }

    // Attempt to disarm via parent class (performs disarm checks if do_disarm_checks=true)
    // Parent class clears logical armed state and logs DISARM event to dataflash
    if (!AP_Arming::disarm(method, do_disarm_checks)) {
        return false;
    }
    
    // Reset mission to beginning if not currently in AUTO mode
    // This prevents unintended mission resume when re-arming in manual modes after
    // moving to a different location. In AUTO mode, mission state is preserved to
    // allow intentional pause/resume workflows.
    if (rover.control_mode != &rover.mode_auto) {
        // reset the mission on disarm if we are not in auto
        rover.mode_auto.mission.reset();
    }

    // Update soft-armed state: Propagate disarmed state to HAL (disables motor output)
    // and to logger (records disarming in binary logs)
    update_soft_armed();

    // Notify pilot and GCS that throttle is now disabled
    send_arm_disarm_statustext("Throttle disarmed");

    return true;
}

#if AP_OAPATHPLANNER_ENABLED
/**
 * @brief Verify object avoidance system has initialized correctly
 * 
 * @details Validates that the object avoidance (OA) path planner has successfully
 *          initialized and is ready for operation. The OA system uses proximity
 *          sensors (lidar, sonar, radar) to detect obstacles and automatically
 *          adjust the vehicle path to avoid collisions.
 *          
 *          This check ensures:
 *          - OA database structures are initialized
 *          - Proximity sensors are detected and responsive
 *          - Path planning algorithms are ready
 *          - Memory allocation succeeded
 *          
 *          The OA system is critical for autonomous navigation in cluttered
 *          environments and must be fully functional before arming.
 * 
 * @param[in] report If true, send failure messages to GCS with specific error details
 * 
 * @return true if object avoidance system is ready, false if initialization failed
 * 
 * @note This check only runs if AP_OAPATHPLANNER_ENABLED is compiled in
 * @note Specific failure reasons are provided via the failure_msg buffer
 * 
 * @warning Operating with failed OA initialization will disable collision avoidance,
 *          potentially leading to crashes in autonomous modes near obstacles
 */
bool AP_Arming_Rover::oa_check(bool report)
{
    // Buffer for detailed failure message from OA system
    char failure_msg[50] = {};
    
    // Query OA system pre-arm status - returns true if ready, false with error message
    if (rover.g2.oa.pre_arm_check(failure_msg, ARRAY_SIZE(failure_msg))) {
        return true;
    }

    // OA system not ready - report specific failure reason to pilot/GCS
    check_failed(report, "%s", failure_msg);
    return false;
}
#endif  // AP_OAPATHPLANNER_ENABLED

/**
 * @brief Validate critical rover parameters are within safe operational ranges
 * 
 * @details Checks that key configuration parameters have valid values that won't
 *          cause operational failures or undefined behavior during flight. Invalid
 *          parameters can lead to mission failures, instability, or crashes.
 *          
 *          Current parameter validations:
 *          - WP_SPEED: Waypoint navigation speed must be positive (> 0)
 *            A zero or negative speed would prevent autonomous navigation,
 *            causing the rover to not move toward waypoints in AUTO mode.
 *          
 *          Additional checks may be added as needed for other critical parameters
 *          affecting rover safety and functionality.
 * 
 * @param[in] report If true, send failure messages to GCS identifying invalid parameters
 * 
 * @return true if all checked parameters are valid, false if any parameter is invalid
 * 
 * @note This check can be disabled via ARMING_CHECK parameter (Check::PARAMETERS bit)
 * @note Parameter checks are less strict than hardware checks since parameters can
 *       be corrected via GCS without physical changes
 * 
 * @warning Invalid parameters can cause mission failures. Always validate parameter
 *          values before autonomous operation, especially after parameter file uploads.
 */
bool AP_Arming_Rover::parameter_checks(bool report)
{
    // success if parameter checks are disabled
    if (!check_enabled(Check::PARAMETERS)) {
        return true;
    }

    // Validate waypoint navigation speed is positive
    // WP_SPEED controls autonomous navigation speed in AUTO, GUIDED, and RTL modes
    // A value <= 0 would prevent waypoint navigation, causing mission failure
    // is_positive() returns true if value > 0
    if (!is_positive(rover.g2.wp_nav.get_default_speed())) {
        check_failed(Check::PARAMETERS, report, "WP_SPEED too low");
        return false;
    }

    return true;
}

/**
 * @brief Verify the current flight mode permits arming
 * 
 * @details Validates that the rover's current control mode allows the vehicle to
 *          be armed. Some modes may restrict arming for safety reasons or because
 *          arming in that mode would lead to undefined behavior.
 *          
 *          Mode arming permissions (examples):
 *          - MANUAL: Allows arming - direct pilot control with no automation
 *          - ACRO: Allows arming - rate-based control for advanced users
 *          - AUTO: Allows arming - autonomous mission execution
 *          - GUIDED: May allow arming - external computer control
 *          - HOLD: Allows arming - vehicle holds position
 *          - Special modes: May restrict arming depending on state requirements
 *          
 *          Each mode class implements allows_arming() to indicate whether arming
 *          is safe and appropriate in that mode. This provides mode-specific
 *          safety logic beyond generic pre-arm checks.
 * 
 * @param[in] report If true, send failure message to GCS indicating mode restriction
 * 
 * @return true if current mode allows arming, false if mode restricts arming
 * 
 * @note Mode arming permissions are defined in each mode's allows_arming() method
 * @note This is separate from rudder-arming permission (allows_arming_from_transmitter)
 * 
 * @warning Bypassing mode arming restrictions could allow arming in modes that
 *          require specific preconditions, leading to unexpected vehicle behavior
 *          or mode initialization failures.
 */
bool AP_Arming_Rover::mode_checks(bool report)
{
    // Query current control mode whether it permits arming
    // Each mode implements allows_arming() based on mode-specific safety considerations
    //display failure if arming in this mode is not allowed
    if (!rover.control_mode->allows_arming()) {
        check_failed(report, "Mode not armable");
        return false;
    }
    return true;
}

/**
 * @brief Validate motor and ESC configuration and readiness
 * 
 * @details Performs comprehensive checks on the motor/ESC subsystem to ensure
 *          safe operation before arming. This includes motor configuration,
 *          ESC communication, and specialized motor system health checks.
 *          
 *          Standard motor checks (AR_Motors):
 *          - Motor configuration valid (steering, throttle channels assigned)
 *          - Servo output ranges configured properly
 *          - Motor mixing parameters within limits
 *          - ESC telemetry communication (if supported)
 *          - Motor output limits configured appropriately
 *          
 *          Torqeedo electric motor checks (if enabled):
 *          - Communication with Torqeedo motor controller established
 *          - Motor controller firmware compatible
 *          - Motor temperature within operating range
 *          - Battery voltage acceptable for motor operation
 *          - No motor controller error flags active
 *          
 *          Motor checks are critical because:
 *          - Invalid configuration can cause loss of steering or throttle
 *          - ESC communication failures can cause sudden motor cutouts
 *          - Motor controller errors can indicate hardware damage
 *          - Torqeedo-specific issues can prevent electric propulsion
 * 
 * @param[in] report If true, send detailed failure messages to GCS
 * 
 * @return true if all motor checks pass, false if any check fails
 * 
 * @note Motor checks are always performed and cannot be disabled via ARMING_CHECK
 * @note Torqeedo checks only run if HAL_TORQEEDO_ENABLED and motor detected
 * 
 * @warning Motor configuration errors can cause loss of vehicle control. Invalid
 *          steering configuration can prevent turning. Invalid throttle configuration
 *          can prevent movement or cause uncontrolled acceleration. ESC communication
 *          failures can cause sudden motor cutouts leading to loss of control.
 * 
 * @see AR_Motors::pre_arm_check() for standard motor validation
 * @see AP_Torqeedo::pre_arm_checks() for Torqeedo-specific validation
 */
bool AP_Arming_Rover::motor_checks(bool report)
{
    // Perform standard rover motor configuration and health checks
    // AR_Motors validates steering/throttle channel configuration, output limits,
    // and ESC communication health
    bool ret = rover.g2.motors.pre_arm_check(report);

    // If Torqeedo electric outboard motor support is compiled in, perform
    // Torqeedo-specific pre-arm checks
#if HAL_TORQEEDO_ENABLED
    char failure_msg[50] = {};
    AP_Torqeedo *torqeedo = AP_Torqeedo::get_singleton();
    
    // Check if Torqeedo motor is present and configured
    if (torqeedo != nullptr) {
        // Perform Torqeedo motor controller pre-arm checks
        // Validates: Communication link, motor temp, battery voltage, controller errors
        if (!torqeedo->pre_arm_checks(failure_msg, ARRAY_SIZE(failure_msg))) {
            // Torqeedo check failed - report specific error to pilot/GCS
            check_failed(report, "Torqeedo: %s", failure_msg);
            ret = false;
        }
    }
#endif

    return ret;
}
