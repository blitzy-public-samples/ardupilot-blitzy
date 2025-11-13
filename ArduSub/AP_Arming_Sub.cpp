/**
 * @file AP_Arming_Sub.cpp
 * @brief ArduSub-specific arming and pre-arm safety checks implementation
 * 
 * @details This file implements underwater vehicle (ROV/submarine) specific pre-arm and arming
 *          checks that extend the base AP_Arming class. ArduSub has unique requirements
 *          compared to aerial vehicles:
 * 
 *          - Mandatory disarm button/function configuration for underwater safety
 *          - Barometer health is critical for depth sensing and control
 *          - Joystick/RC input validation for underwater manual control
 *          - AHRS pre-arm checks for orientation estimation underwater
 *          - EKF origin handling for compass calibration in submerged environments
 * 
 *          The arming system prevents motor activation until all safety checks pass,
 *          protecting the vehicle and preventing underwater accidents. Key safety features:
 * 
 *          - Prevents arming without a configured disarm method (button or aux function)
 *          - Validates sensor health (INS, barometer, compass, AHRS)
 *          - Checks RC/joystick input calibration and centering
 *          - Ensures proper mode configuration before allowing motor output
 *          - Integrates with AP_Notify for LED/buzzer feedback on arming state
 *          - Integrates with AP_Logger for arming event logging
 *          - Uses HAL soft-armed flag for system-wide armed state tracking
 * 
 * @note Underwater vehicles have different failure modes than aircraft - loss of control
 *       underwater can result in loss of vehicle, entanglement, or collision hazards
 * 
 * @warning All arming checks are safety-critical. Modifications must be thoroughly tested
 *          in SITL and controlled underwater environments before deployment
 * 
 * @see AP_Arming base class in libraries/AP_Arming/
 * @see Sub.h for main ArduSub vehicle class
 * @see JSButton.h for joystick button function definitions
 * 
 * Source: ArduSub/AP_Arming_Sub.cpp
 */

#include "AP_Arming_Sub.h"
#include "Sub.h"

/**
 * @brief Validate RC/joystick input calibration for critical control channels
 * 
 * @details Performs pre-arm validation of RC input calibration for the four primary
 *          control channels used in underwater vehicle control:
 *          - Roll: Lateral translation control
 *          - Pitch: Forward/backward translation control  
 *          - Throttle: Vertical (depth) control
 *          - Yaw: Rotation control
 * 
 *          This check verifies that:
 *          - All channels have valid radio min/max values configured
 *          - Channel trim values are within acceptable ranges
 *          - No channels are reversed incorrectly
 *          - Input ranges allow full control authority
 * 
 *          Uses the shared rc_checks_copter_sub() implementation which validates
 *          similar vehicles (multirotors and underwater vehicles) that require
 *          precise multi-axis control.
 * 
 * @param[in] display_failure If true, send failure message to ground station and
 *                             trigger arming failure notification (LED/buzzer)
 * 
 * @return true if all RC channels are properly calibrated, false otherwise
 * 
 * @note RC calibration must be performed in a joystick configuration tool or
 *       QGroundControl before the vehicle can be armed
 * @note This is called during pre-arm checks before arming is permitted
 * 
 * @see rc_checks_copter_sub() in AP_Arming.cpp for validation implementation
 * @see RC_Channel for channel configuration and calibration values
 * 
 * Source: ArduSub/AP_Arming_Sub.cpp:4-13
 */
bool AP_Arming_Sub::rc_calibration_checks(bool display_failure)
{
    const RC_Channel *channels[] = {
        sub.channel_roll,
        sub.channel_pitch,
        sub.channel_throttle,
        sub.channel_yaw
    };
    return rc_checks_copter_sub(display_failure, channels);
}

/**
 * @brief Check if vehicle has a configured disarm button or auxiliary function
 * 
 * @details ArduSub requires a disarm method to be configured before arming is permitted.
 *          This is a critical safety feature for underwater vehicles where loss of control
 *          or emergency situations require immediate motor shutdown capability.
 * 
 *          This function searches for disarm capability in three locations:
 * 
 *          1. **Standard joystick button functions** (unshifted):
 *             - JSButton::k_disarm - Dedicated disarm button
 *             - JSButton::k_arm_toggle - Toggle between armed/disarmed states
 *             - JSButton::k_shift - Enables shifted button functions (checked separately)
 * 
 *          2. **Shifted joystick button functions** (if shift button exists):
 *             - JSButton::k_disarm with shift pressed
 *             - JSButton::k_arm_toggle with shift pressed
 *             Checks all 16 possible button positions for shifted functions
 * 
 *          3. **RC auxiliary channel functions**:
 *             - AUX_FUNC::MOTOR_ESTOP - Emergency stop (immediate motor cutoff)
 *             - AUX_FUNC::DISARM - Standard disarm function
 *             - AUX_FUNC::ARMDISARM - Toggle arm/disarm via switch/button
 *             - AUX_FUNC::ARM_EMERGENCY_STOP - Combined arm control with emergency stop
 * 
 *          The function validates that at least ONE disarm method is available before
 *          the vehicle can be armed, preventing scenarios where the pilot cannot
 *          safely disarm the vehicle in an emergency.
 * 
 * @return true if a disarm button/function is configured, false otherwise
 * 
 * @note This check is mandatory in pre_arm_checks() - arming will fail without a disarm method
 * @note Underwater environments present unique hazards (entanglement, collision, loss of vehicle)
 *       making immediate disarm capability essential
 * @note The shift function allows a single button to have two functions (normal + shifted),
 *       effectively doubling the available button functions on limited joystick hardware
 * 
 * @warning Disabling this check or bypassing it could result in inability to disarm the
 *          vehicle in an emergency, leading to uncontrolled motor operation underwater
 * 
 * @see JSButton::Func enum for all available button function types
 * @see RC_Channel::AUX_FUNC enum for auxiliary channel function types
 * @see pre_arm_checks() where this validation is enforced
 * 
 * Source: ArduSub/AP_Arming_Sub.cpp:15-49
 */
bool AP_Arming_Sub::has_disarm_function() const {
    bool has_shift_function = false;
    // make sure the craft has a disarm button assigned before it is armed
    // check all the standard btn functions
    for (uint8_t i = 0; i < 16; i++) {
        switch (sub.get_button(i)->function(false)) {
            case JSButton::k_shift :
                has_shift_function = true;
                break;
            case JSButton::k_arm_toggle :
                return true;
            case JSButton::k_disarm :
                return true;
        }
    }

    // check all the shift functions if there's shift assigned
    if (has_shift_function) {
        for (uint8_t i = 0; i < 16; i++) {
            switch (sub.get_button(i)->function(true)) {
                case JSButton::k_arm_toggle :
                case JSButton::k_disarm :
                    return true;
            }
        }
    }
    // check if an AUX function that disarms or estops is setup
    if (rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOTOR_ESTOP) || 
        rc().find_channel_for_option(RC_Channel::AUX_FUNC::DISARM) || 
        rc().find_channel_for_option(RC_Channel::AUX_FUNC::ARMDISARM) || 
        rc().find_channel_for_option(RC_Channel::AUX_FUNC::ARM_EMERGENCY_STOP)) {
        return true;
    }  
    return false;
}

/**
 * @brief Perform comprehensive pre-arm safety validation for underwater vehicle
 * 
 * @details Executes all pre-arm checks required before the vehicle can be armed and motors
 *          enabled. This is the primary safety gate preventing unsafe arming conditions.
 * 
 *          ArduSub-specific checks performed:
 *          1. **Already Armed Check**: If vehicle is already armed, immediately return success
 *             (pre-arm checks only apply to disarmed state)
 * 
 *          2. **Disarm Function Validation**: Verifies a disarm button/function is configured
 *             - MANDATORY for ArduSub due to underwater safety requirements
 *             - Prevents arming if pilot would have no way to disarm in emergency
 *             - Checks joystick buttons and RC auxiliary functions
 *             - Failure message: "Must assign a disarm or arm_toggle button or disarm aux function"
 * 
 *          Base class AP_Arming::pre_arm_checks() performs additional validations:
 *          - **Barometer Health**: Critical for depth sensing and altitude hold
 *             * Validates all configured barometers are healthy
 *             * Checks for excessive drift or noise in pressure readings
 *             * Verifies barometer temperature compensation is working
 * 
 *          - **INS (Inertial Navigation System) Health**: Validates gyroscope and accelerometer
 *             * Checks all IMUs are calibrated and producing valid data
 *             * Verifies gyro/accel consistency across multiple IMUs
 *             * Validates sensor temperature ranges
 *             * Calls ins_checks() for additional Sub-specific INS validation
 * 
 *          - **Compass Calibration**: Validates magnetometer configuration
 *             * Checks compass is calibrated (offsets configured)
 *             * Verifies compass health and consistency between multiple compasses
 *             * Validates declination is set for vehicle location
 *             * Important for orientation estimation underwater where GPS is unavailable
 * 
 *          - **RC Input Validation**: Ensures joystick/RC control is functional
 *             * Calls rc_calibration_checks() for channel validation
 *             * Verifies all required control channels are receiving input
 *             * Checks for RC failsafe conditions
 * 
 *          - **Battery Monitoring**: Validates battery state
 *             * Checks battery voltage is above minimum threshold
 *             * Verifies battery monitoring is configured and healthy
 * 
 *          - **Safety Switch**: Checks hardware safety switch state (if configured)
 * 
 *          - **Mode-Specific Requirements**: Validates current flight mode allows arming
 * 
 * @param[in] display_failure If true, send failure messages to ground station via GCS,
 *                             set AP_Notify::events.arming_failed for LED/buzzer indication,
 *                             and log failure to dataflash
 * 
 * @return true if all pre-arm checks pass and vehicle is safe to arm, false otherwise
 * 
 * @note Pre-arm checks run continuously while disarmed and can be viewed in ground station
 * @note Some checks can be disabled via ARMING_CHECK parameter bitmask, but disarm function
 *       check is always enforced for ArduSub
 * @note Underwater vehicles rely heavily on barometer for depth control - barometer health
 *       is more critical than for surface vehicles
 * 
 * @warning Bypassing or disabling pre-arm checks can result in arming with failed sensors,
 *          leading to loss of control, vehicle loss, or safety hazards underwater
 * @warning The disarm function requirement cannot be bypassed - this is a fundamental
 *          safety requirement for underwater operations
 * 
 * @see has_disarm_function() for disarm button validation logic
 * @see AP_Arming::pre_arm_checks() for base class validation implementation
 * @see ins_checks() for INS-specific validation
 * @see rc_calibration_checks() for RC input validation
 * 
 * Source: ArduSub/AP_Arming_Sub.cpp:51-63
 */
bool AP_Arming_Sub::pre_arm_checks(bool display_failure)
{
    if (armed) {
        return true;
    }
    // don't allow arming unless there is a disarm button configured
    if (!has_disarm_function()) {
        check_failed(display_failure, "Must assign a disarm or arm_toggle button or disarm aux function");
        return false;
    }

    return AP_Arming::pre_arm_checks(display_failure);
}

/**
 * @brief Validate Inertial Navigation System (INS) and AHRS for underwater vehicle
 * 
 * @details Performs comprehensive validation of the inertial navigation system, which includes
 *          gyroscopes, accelerometers, and the AHRS (Attitude and Heading Reference System).
 *          These sensors are critical for underwater vehicle orientation and motion estimation.
 * 
 *          **Two-stage validation process:**
 * 
 *          1. **Base INS Checks** (AP_Arming::ins_checks()):
 *             - Validates all configured IMUs (Inertial Measurement Units) are healthy
 *             - Checks gyroscope calibration and bias stability
 *             - Verifies accelerometer calibration and scale factors
 *             - Ensures IMU sample rates are within acceptable ranges
 *             - Validates IMU temperature compensation is functioning
 *             - Checks for excessive vibration levels that could corrupt IMU data
 *             - Verifies consistency between multiple IMUs (if available)
 * 
 *          2. **AHRS Pre-Arm Checks** (ArduSub-specific):
 *             - Validates AHRS orientation estimation is converged and stable
 *             - Checks EKF (Extended Kalman Filter) initialization state
 *             - Verifies attitude estimation quality (roll, pitch, yaw)
 *             - Ensures velocity and position estimates are valid (if using EKF)
 *             - Validates sensor fusion between IMU, compass, and barometer
 *             - Checks for AHRS innovation test failures (sensor disagreement)
 * 
 *          AHRS is particularly important for underwater vehicles because:
 *          - GPS is unavailable underwater for position/velocity correction
 *          - Visual odometry may be limited in turbid water
 *          - Attitude estimation relies primarily on IMU and compass fusion
 *          - Barometer provides primary vertical (depth) reference
 * 
 * @param[in] display_failure If true, send detailed failure message to ground station
 *                             including specific AHRS failure reason from pre_arm_check()
 * 
 * @return true if both base INS checks and AHRS pre-arm checks pass, false otherwise
 * 
 * @note INS checks can be disabled via ARMING_CHECK parameter bit, but this is not
 *       recommended for underwater vehicles where IMU data quality is critical
 * @note The AHRS pre_arm_check() returns detailed failure messages explaining specific
 *       issues (e.g., "EKF not started", "High velocity variance", "Large innovations")
 * @note IMU health is continuously monitored during flight, not just during pre-arm
 * 
 * @warning Flying with failed INS/AHRS checks can result in incorrect attitude estimation,
 *          leading to uncontrolled vehicle behavior, inability to maintain orientation,
 *          or failure to maintain depth control underwater
 * @warning Underwater vehicles have limited sensory feedback compared to aircraft, making
 *          IMU/AHRS reliability even more critical for safe operation
 * 
 * @see AP_Arming::ins_checks() for base INS validation implementation
 * @see AP_AHRS::pre_arm_check() for AHRS-specific validation
 * @see AP_InertialSensor for IMU management and health monitoring
 * @see AP_NavEKF2/AP_NavEKF3 for Extended Kalman Filter implementation
 * 
 * Source: ArduSub/AP_Arming_Sub.cpp:65-82
 */
bool AP_Arming_Sub::ins_checks(bool display_failure)
{
    // call parent class checks
    if (!AP_Arming::ins_checks(display_failure)) {
        return false;
    }

    // additional sub-specific checks
    if (check_enabled(Check::INS)) {
        char failure_msg[50] = {};
        if (!AP::ahrs().pre_arm_check(false, failure_msg, sizeof(failure_msg))) {
            check_failed(Check::INS, display_failure, "AHRS: %s", failure_msg);
            return false;
        }
    }

    return true;
}

/**
 * @brief Arm the underwater vehicle and enable motor output
 * 
 * @details This is the safety-critical function that transitions the vehicle from disarmed
 *          to armed state, enabling motor output. The arming process performs final validation
 *          checks, initializes armed state, and enables motor control.
 * 
 *          **Arming Sequence:**
 * 
 *          1. **Re-entrancy Protection**: Prevents multiple simultaneous arming attempts
 *             - Uses static flag to detect if already in arming process
 *             - Returns false immediately if re-entered (safety feature)
 * 
 *          2. **Throttle Position Check** (if RC checks enabled and throttle arming check enabled):
 *             - Validates throttle is centered/within trim deadzone
 *             - Prevents arming with throttle not at neutral (safety feature)
 *             - Controlled by RC_OPTIONS and g.thr_arming_position parameter
 *             - Triggers AP_Notify arming failure event on check failure
 * 
 *          3. **Base Arming Checks**: Calls AP_Arming::arm() to perform:
 *             - Final pre-arm check validation (if do_arming_checks=true)
 *             - Arming method validation (stick, GCS, button, etc.)
 *             - Rudder arming sequence verification
 *             - Mode-specific arming permission checks
 * 
 *          4. **Logger Initialization**: Notifies logger that vehicle is arming
 *             - Opens log files for this flight session
 *             - Begins logging armed state and events
 * 
 *          5. **Failsafe Disable**: Temporarily disables CPU failsafe during initialization
 *             - Prevents false failsafe triggers during startup delays
 *             - Re-enabled after arming sequence completes
 * 
 *          6. **Notification System**: Updates AP_Notify armed state
 *             - Sets AP_Notify::flags.armed = true for LED/buzzer indication
 *             - Calls notify update multiple times to ensure indication is visible
 *             - Sends "Arming motors" text to ground station
 * 
 *          7. **Initial Bearing**: Captures vehicle heading at arming time
 *             - Used for heading-hold modes and navigation reference
 * 
 *          8. **Home Position Handling**:
 *             - For ROVs, absolute altitude is always used (no home reset)
 *             - If home set but not locked: reset home to current location
 *             - Commented code shows EKF altitude reset (not used for ROV)
 * 
 *          9. **HAL Soft-Armed State**: Sets system-wide armed flag
 *             - hal.util->set_soft_armed(true) enables hardware safety features
 *             - Other subsystems can check armed state via HAL
 * 
 *          10. **Motor Output Enable**: Enables PWM output to motors
 *              - sub.enable_motor_output() allows motor commands to hardware
 *              - sub.motors.armed(true) sets motor library armed state
 * 
 *          11. **Flight Mode Logging**: Records current mode at arming time
 * 
 *          12. **Failsafe Re-enable**: Restores CPU failsafe monitoring
 * 
 *          13. **Performance Monitor**: Tells scheduler to ignore this loop for timing
 *              - Arming initialization delays are excluded from loop time statistics
 * 
 *          14. **EKF Origin Check**: Validates EKF origin for compass performance
 *              - Without world position, compass calibration may be degraded
 *              - Checks ORIGIN_* parameters for manual origin configuration
 *              - Warns ground station if compass performance may be affected
 * 
 * @param[in] method Arming method used (RUDDER, MAVLINK, SWITCH, etc.)
 * @param[in] do_arming_checks If true, perform full pre-arm validation before arming.
 *                              If false, skip checks (used for emergency arming scenarios)
 * 
 * @return true if arming successful and motors enabled, false if arming failed
 * 
 * @note Arming can be triggered by multiple methods: rudder stick, GCS command, button, aux switch
 * @note The function uses a static re-entrancy guard to prevent multiple simultaneous arming attempts
 * @note AP_Notify provides visual/audible feedback (LEDs, buzzer) when arming succeeds or fails
 * @note All arming events are logged to dataflash for post-flight analysis
 * 
 * @warning SAFETY CRITICAL: This function enables motor output. Arming with people or objects
 *          near thrusters can cause injury. Always ensure clear area before arming.
 * @warning Arming underwater with failed sensors (bypassing checks) can lead to loss of vehicle
 *          control, uncontrolled ascent/descent, or vehicle loss
 * @warning The throttle position check prevents arming with non-zero throttle, which could cause
 *          sudden vehicle motion when armed. Do not disable unless absolutely necessary.
 * @warning Modifying the arming sequence or disabling checks can create safety hazards and should
 *          only be done with full understanding of failure modes and thorough testing in SITL
 * @warning EKF origin validation affects compass calibration quality. Operating without proper
 *          origin may result in degraded heading estimation underwater
 * 
 * @see AP_Arming::arm() for base class arming implementation
 * @see pre_arm_checks() for validation performed before arming
 * @see AP_Notify for LED/buzzer armed state indication
 * @see AP_Logger for arming event logging
 * @see AP_Motors::armed() for motor library armed state
 * 
 * Source: ArduSub/AP_Arming_Sub.cpp:84-181
 */
bool AP_Arming_Sub::arm(AP_Arming::Method method, bool do_arming_checks)
{
    static bool in_arm_motors = false;

    // exit immediately if already in this function
    if (in_arm_motors) {
        return false;
    }

    in_arm_motors = true;

    //if RC checks enabled, and RC_OPTIONS enabled for "0" throttle, and enabled check for throttle within trim position
    if (check_enabled(Check::RC) &&
     rc().option_is_enabled(RC_Channels::Option::ARMING_CHECK_THROTTLE) &&
     (sub.g.thr_arming_position == WITHIN_THR_TRIM)) {
        const char *rc_item = "Throttle";
        // check throttle is within trim+/- dz, ie centered throttle
        if (!sub.channel_throttle->in_trim_dz()) {
           check_failed(Check::RC, true, "%s not centered/close to trim", rc_item);
           AP_Notify::events.arming_failed = true;
           in_arm_motors = false;
           return false;
        }
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

    // disable cpu failsafe because initialising everything takes a while
    sub.mainloop_failsafe_disable();

    // notify that arming will occur (we do this early to give plenty of warning)
    AP_Notify::flags.armed = true;
    // call notify update a few times to ensure the message gets out
    for (uint8_t i=0; i<=10; i++) {
        AP::notify().update();
    }

    send_arm_disarm_statustext("Arming motors");

    AP_AHRS &ahrs = AP::ahrs();

    sub.initial_armed_bearing = ahrs.yaw_sensor;

    if (!ahrs.home_is_set()) {
        // Reset EKF altitude if home hasn't been set yet (we use EKF altitude as substitute for alt above home)

        // Always use absolute altitude for ROV
        // ahrs.resetHeightDatum();
        // AP::logger().Write_Event(LogEvent::EKF_ALT_RESET);
    } else if (!ahrs.home_is_locked()) {
        // Reset home position if it has already been set before (but not locked)
        if (!sub.set_home_to_current_location(false)) {
            // ignore this failure
        }
    }

    hal.util->set_soft_armed(true);

    // enable output to motors
    sub.enable_motor_output();

    // finally actually arm the motors
    sub.motors.armed(true);

#if HAL_LOGGING_ENABLED
    // log flight mode in case it was changed while vehicle was disarmed
    AP::logger().Write_Mode((uint8_t)sub.control_mode, sub.control_mode_reason);
#endif

    // reenable failsafe
    sub.mainloop_failsafe_enable();

    // perf monitor ignores delay due to arming
    AP::scheduler().perf_info.ignore_this_loop();

    // flag exiting this function
    in_arm_motors = false;

    // if we do not have an ekf origin then we can't use the WMM tables
    if (!sub.ensure_ekf_origin()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Compass performance degraded");
        if (check_enabled(Check::PARAMETERS)) {
            check_failed(Check::PARAMETERS, true, "No world position, check ORIGIN_* parameters");
            return false;
        }
    }
    // return success
    return true;
}

/**
 * @brief Disarm the underwater vehicle and disable motor output
 * 
 * @details This is the safety-critical function that transitions the vehicle from armed to
 *          disarmed state, disabling motor output and performing shutdown procedures. The
 *          disarm process validates the disarm request, saves calibration data, and safely
 *          shuts down motor control.
 * 
 *          **Disarming Sequence:**
 * 
 *          1. **Already Disarmed Check**: Returns false immediately if already disarmed
 *             - Checks motor library armed state via sub.motors.armed()
 *             - Prevents redundant disarm operations
 * 
 *          2. **Base Disarm Validation**: Calls AP_Arming::disarm() to perform:
 *             - Disarm method validation (stick, GCS, button, failsafe, etc.)
 *             - Optional disarm safety checks (if do_disarm_checks=true)
 *             - Mode-specific disarm permission validation
 *             - Prevents accidental disarm during critical flight phases
 * 
 *          3. **Ground Station Notification**: Sends "Disarming motors" message
 *             - Informs pilot/operator that disarm is in progress
 * 
 *          4. **Compass Calibration Save**: Saves EKF-learned compass offsets (if enabled)
 *             - Checks if AHRS is using compass (underwater navigation)
 *             - Validates compass learn type is set to COPY_FROM_EKF
 *             - Retrieves compass offset corrections from EKF for each compass instance
 *             - Saves updated offsets to persistent storage via AP::compass()
 *             - Improves compass calibration over time through in-flight learning
 *             - Critical for underwater vehicles where compass is primary heading reference
 * 
 *          5. **Motor Shutdown**: Disables motor output
 *             - sub.motors.armed(false) immediately stops motor control
 *             - PWM outputs return to disarmed/safe state
 * 
 *          6. **Mission Reset**: Clears current mission state
 *             - sub.mission.reset() returns mission to start
 *             - Prepares mission system for next flight
 * 
 *          7. **Logger State Update**: Notifies logger vehicle is disarmed
 *             - AP::logger().set_vehicle_armed(false) marks end of armed session
 *             - May trigger log file closing or flushing
 * 
 *          8. **HAL Soft-Armed State**: Clears system-wide armed flag
 *             - hal.util->set_soft_armed(false) updates hardware safety state
 *             - Other subsystems can detect disarmed state via HAL
 * 
 *          9. **Input Hold Clear**: Resets any active input hold states
 *             - sub.clear_input_hold() releases held control inputs
 *             - Ensures clean state for next arming
 * 
 * @param[in] method Disarm method used (RUDDER, MAVLINK, SWITCH, TIMEOUT, BADFLIGHTMODE, etc.)
 * @param[in] do_disarm_checks If true, perform disarm safety validation before disarming.
 *                              If false, force disarm regardless of conditions (emergency disarm)
 * 
 * @return true if disarm successful and motors disabled, false if disarm not permitted or failed
 * 
 * @note Disarm can be triggered by multiple methods: rudder stick, GCS command, button, auto-disarm, failsafe
 * @note The compass offset saving feature improves heading accuracy over multiple flights by learning
 *       corrections from the EKF's in-flight compass calibration
 * @note Mission state is reset on disarm, so resuming a mission requires re-uploading or selecting waypoint
 * @note All disarm events are logged to dataflash via the base AP_Arming::disarm() call
 * 
 * @warning SAFETY CRITICAL: Disarming underwater with active motor commands will immediately
 *          stop thrusters, potentially causing uncontrolled drift, sinking, or surfacing
 * @warning Emergency disarm (do_disarm_checks=false) bypasses safety checks and should only
 *          be used when immediate motor shutdown is required regardless of vehicle state
 * @warning Compass offset saving assumes EKF compass learning is properly configured and
 *          converged. Saving offsets from poorly calibrated EKF can degrade compass accuracy
 * @warning Do not disarm during critical operations (deep dive, under obstacles, in current)
 *          unless emergency requires immediate motor shutdown
 * 
 * @see AP_Arming::disarm() for base class disarm implementation and validation
 * @see AP_Compass for compass calibration and offset management
 * @see AP_AHRS::getMagOffsets() for retrieving EKF-learned compass corrections
 * @see AP_Motors::armed() for motor library armed state
 * @see AP_Mission::reset() for mission state management
 * 
 * Source: ArduSub/AP_Arming_Sub.cpp:183-224
 */
bool AP_Arming_Sub::disarm(const AP_Arming::Method method, bool do_disarm_checks)
{
    // return immediately if we are already disarmed
    if (!sub.motors.armed()) {
        return false;
    }

    if (!AP_Arming::disarm(method, do_disarm_checks)) {
        return false;
    }

    send_arm_disarm_statustext("Disarming motors");

    auto &ahrs = AP::ahrs();

    // save compass offsets learned by the EKF if enabled
    if (ahrs.use_compass() && AP::compass().get_learn_type() == Compass::LearnType::COPY_FROM_EKF) {
        for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
            Vector3f magOffsets;
            if (ahrs.getMagOffsets(i, magOffsets)) {
                AP::compass().set_and_save_offsets(i, magOffsets);
            }
        }
    }

    // send disarm command to motors
    sub.motors.armed(false);

    // reset the mission
    sub.mission.reset();

#if HAL_LOGGING_ENABLED
    AP::logger().set_vehicle_armed(false);
#endif

    hal.util->set_soft_armed(false);

    // clear input holds
    sub.clear_input_hold();

    return true;
}
