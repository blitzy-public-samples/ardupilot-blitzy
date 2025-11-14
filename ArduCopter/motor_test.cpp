#include "Copter.h"

/**
 * @file motor_test.cpp
 * @brief Motor testing functionality for individual motor verification and calibration
 * 
 * @details This file implements the MAV_CMD_DO_MOTOR_TEST MAVLink command, allowing
 *          ground control stations (GCS) or pilots to test individual motors or control
 *          surfaces to verify proper wiring, rotation direction, and motor ordering.
 *          
 *          Motor testing is a critical pre-flight procedure to ensure:
 *          - Correct motor-to-output mapping
 *          - Proper rotation direction for each motor
 *          - ESC responsiveness and calibration
 *          - Control surface movement direction
 *          
 *          The system supports multiple test modes:
 *          - Throttle percentage (0-100%)
 *          - Direct PWM values (typically 1000-2000 μs)
 *          - Pilot throttle channel pass-through
 *          - Compass calibration mode (for interference mapping)
 *          
 * @warning SAFETY CRITICAL: Motor testing must ONLY be performed with:
 *          - Vehicle completely disarmed
 *          - Propellers removed from all motors
 *          - Vehicle secured and unable to move
 *          - Clear area around vehicle
 *          - Safety observer present
 *          
 *          Accidental motor activation with propellers attached can cause
 *          serious injury or death. Multiple safety interlocks are enforced.
 * 
 * @note This module temporarily disables failsafes during testing to prevent
 *       interference with the test sequence. Failsafes are automatically
 *       restored when testing completes.
 * 
 * @see Copter::motor_test_output()
 * @see Copter::mavlink_motor_test_start()
 * @see Copter::mavlink_motor_control_check()
 * @see Copter::motor_test_stop()
 */

/// @defgroup motor_test_constants Motor Test Configuration Constants
/// @{

/// Maximum motor test duration in seconds (10 minutes safety limit)
#define MOTOR_TEST_TIMEOUT_SEC          600

/// @}

/// @defgroup motor_test_state Motor Test State Variables
/// @{

/// System time (milliseconds) when the current motor test sequence began
static uint32_t motor_test_start_ms;

/// Duration (milliseconds) before the current motor test times out for safety
static uint32_t motor_test_timeout_ms;

/// Motor sequence number currently being tested (hardware-dependent motor ordering)
static uint8_t motor_test_seq;

/// Number of motors remaining to test in multi-motor sequence
static uint8_t motor_test_count;

/// Throttle control mode: MOTOR_TEST_THROTTLE_PERCENT, MOTOR_TEST_THROTTLE_PWM, or MOTOR_TEST_THROTTLE_PILOT
static uint8_t motor_test_throttle_type;

/// Throttle value to apply (interpretation depends on motor_test_throttle_type)
static float motor_test_throttle_value;

/// @}

/**
 * @brief Main motor test output function - generates PWM signals for motor testing
 * 
 * @details This function is called from the main loop to output test signals to motors.
 *          It manages the complete motor test sequence including:
 *          - Timeout monitoring for safety
 *          - Multi-motor sequential testing
 *          - PWM value calculation based on test mode
 *          - Safety range validation
 *          - Motor output via motors library
 *          
 *          For multi-motor sequences (motor_test_count > 1), the function cycles through
 *          motors with an inter-motor gap (50% of test duration at zero output) to clearly
 *          identify which motor is active.
 *          
 *          Safety features:
 *          - Automatic timeout after configured duration (max 600 seconds)
 *          - PWM range validation (1000-2000 μs)
 *          - Invalid throttle type detection
 *          - Motor arming state management
 * 
 * @note Called at main loop rate (typically 400 Hz for multicopters)
 * 
 * @warning This function temporarily arms motors while maintaining disarmed state.
 *          This is a special testing mode that bypasses normal arming checks.
 *          Vehicle MUST be secured and propellers removed.
 * 
 * @see Copter::mavlink_motor_test_start() - Initiates motor test
 * @see Copter::motor_test_stop() - Ends motor test and restores safety
 * @see AP_Motors::output_test_seq() - Low-level motor output function
 */
void Copter::motor_test_output()
{
    // Exit immediately if the motor test is not running
    // This check ensures we don't process motor test logic during normal flight
    if (!ap.motor_test) {
        return;
    }

    // Inform scheduler this function may take up to 2000ms in some circumstances
    EXPECT_DELAY_MS(2000);

    // Check for test timeout - safety feature to prevent extended motor activation
    uint32_t now = AP_HAL::millis();
    if ((now - motor_test_start_ms) >= motor_test_timeout_ms) {
        // Handle multi-motor sequential testing
        if (motor_test_count > 1) {
            // Use extended timeout period (1.5x base timeout) to create inter-motor gap
            // This creates a clear pause between motors for easier identification
            if (now - motor_test_start_ms < motor_test_timeout_ms*1.5) {
                // Output zero for 50% of the test time (the gap between motors)
                // This allows operator to clearly distinguish which motor was spinning
                motors->output_min();
            } else {
                // Move onto next motor in sequence
                motor_test_seq++;  // Increment to next motor number
                motor_test_count--; // Decrement remaining motor count
                motor_test_start_ms = now; // Reset timer for next motor
                
                // Ensure motors remain armed for the next test
                // (may have been disarmed between sequences)
                if (!motors->armed()) {
                    motors->armed(true);
                    hal.util->set_soft_armed(true);
                }
            }
            return;
        }
        // Single motor test or final motor in sequence - stop testing
        motor_test_stop();
    } else {
        // Test still active - calculate and output PWM value
        // PWM value that will be output to the motors (typically 1000-2000 μs)
        int16_t pwm = 0;

        // Calculate PWM output based on throttle type specified by MAVLink command
        // Different modes allow for various testing and calibration workflows
        switch (motor_test_throttle_type) {

            case MOTOR_TEST_COMPASS_CAL:
                // Compass calibration mode - runs motors to map magnetic interference
                // Update compass with current battery voltage for accurate field compensation
                compass.set_voltage(battery.voltage());
                compass.per_motor_calibration_update();
                // Fall through to percentage mode for motor output
                FALLTHROUGH;

            case MOTOR_TEST_THROTTLE_PERCENT:
                // Throttle percentage mode (0-100%) - most common for basic motor tests
                // Converts percentage to PWM value within configured motor output range
#if FRAME_CONFIG != HELI_FRAME
                // Sanity check throttle value is within 0-100% range
                if (motor_test_throttle_value <= 100) {
                    // Get configured PWM range for this vehicle's motors
                    int16_t pwm_min = motors->get_pwm_output_min();
                    int16_t pwm_max = motors->get_pwm_output_max();
                    // Linear interpolation: pwm = min + (max-min) * percentage
                    pwm = (int16_t) (pwm_min + (pwm_max - pwm_min) * motor_test_throttle_value * 1e-2f);
                }
#endif
                break;

            case MOTOR_TEST_THROTTLE_PWM:
                // Direct PWM mode - allows precise ESC calibration and testing
                // Value is PWM pulse width in microseconds (typically 1000-2000)
                pwm = (int16_t)motor_test_throttle_value;
                break;

            case MOTOR_TEST_THROTTLE_PILOT:
                // Pilot throttle pass-through mode - uses pilot's throttle stick input
                // Useful for manual motor testing while observing real-time response
                pwm = channel_throttle->get_radio_in();
                break;

            default:
                // Invalid throttle type - stop test for safety
                motor_test_stop();
                return;
        }

        // Sanity check throttle values against RC PWM limits
        // Prevents out-of-range values that could damage ESCs or cause erratic behavior
        // RC_MIN_LIMIT_PWM is typically 1000μs, RC_MAX_LIMIT_PWM is typically 2000μs
        if (pwm < RC_Channel::RC_MIN_LIMIT_PWM || pwm > RC_Channel::RC_MAX_LIMIT_PWM) {
            motor_test_stop();
            return;
        }

        // Output PWM signal to the specified motor in sequence
        // motor_test_seq identifies which physical motor to activate
        // The motors library handles frame-specific motor mapping
        if (!motors->output_test_seq(motor_test_seq, pwm)) {
            // output_test_seq returns false if motor sequence number is invalid
            // or if the motor library cannot safely output to that motor
            gcs().send_text(MAV_SEVERITY_INFO, "Motor Test: cancelled");
            motor_test_stop();
        }
    }
}

/**
 * @brief Comprehensive safety checks before motor testing can begin
 * 
 * @details This function implements multiple layers of safety interlocks to prevent
 *          accidental motor activation that could cause injury or vehicle damage.
 *          
 *          Safety checks performed (in order):
 *          1. Board initialization complete (sensors, calibration loaded)
 *          2. Motor-specific safety checks via motors library
 *          3. RC calibration verified (if required by test mode)
 *          4. Vehicle confirmed on ground (land_complete flag)
 *          5. Safety switch in armed position (if present)
 *          6. Emergency stop (E-Stop) not active
 *          
 *          Any failed check sends an error message to the GCS and prevents testing.
 *          
 * @param[in] gcs_chan      MAVLink channel for sending failure messages to GCS
 * @param[in] check_rc      If true, verify RC calibration; false skips check (for PWM mode)
 * @param[in] mode          Mode name string for error messages (e.g., "Motor Test")
 * 
 * @return true if all safety checks pass and testing can proceed
 * @return false if any safety check fails
 * 
 * @warning SAFETY CRITICAL: This is the primary gatekeeper preventing dangerous motor
 *          activation. Do not bypass or weaken these checks without thorough safety analysis.
 *          Each check serves a specific safety purpose:
 *          - Board init: Ensures sensors and parameters are ready
 *          - Motor checks: Frame-specific safety (e.g., helicopter rotor checks)
 *          - RC calibration: Prevents erratic response from miscalibrated inputs
 *          - Land complete: Prevents mid-flight motor testing
 *          - Safety switch: Hardware interlock requiring physical action
 *          - E-Stop: Emergency kill switch must not be active
 * 
 * @note The check_rc parameter allows skipping RC calibration verification when using
 *       direct PWM mode (MOTOR_TEST_THROTTLE_PWM), as RC input is not used.
 * 
 * @see Copter::mavlink_motor_test_start() - Initiates test after checks pass
 * @see AP_Motors::motor_test_checks() - Frame-specific motor safety checks
 */
bool Copter::mavlink_motor_control_check(const GCS_MAVLINK &gcs_chan, bool check_rc, const char* mode)
{
    // SAFETY CHECK 1: Verify board initialization is complete
    // Ensures all sensors, parameters, and subsystems are ready before motor activation
    // Without complete initialization, vehicle state may be unreliable
    if (!ap.initialised) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL,"%s: Board initialising", mode);
        return false;
    }

    // SAFETY CHECK 2: Frame-specific motor test validation
    // Different vehicle frames have different safety requirements:
    // - Helicopters: Check collective position and rotor RSC mode
    // - Multirotors: Verify motor output configuration is valid
    // - Tailsitters: Additional orientation checks
    char failure_msg[100] {};
    if (!motors->motor_test_checks(ARRAY_SIZE(failure_msg), failure_msg)) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL,"%s: %s", mode, failure_msg);
        return false;
    }

    // SAFETY CHECK 3: RC calibration verification (if required)
    // Skipped for direct PWM mode since RC input is not used
    // Critical for throttle percentage and pilot pass-through modes
    if (check_rc && !arming.rc_calibration_checks(true)) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL,"%s: RC not calibrated", mode);
        return false;
    }

    // SAFETY CHECK 4: Verify vehicle is on the ground
    // Prevents motor testing during flight which could cause loss of control
    // land_complete flag is set by position controller and landing detector
    if (!ap.land_complete) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL,"%s: vehicle not landed", mode);
        return false;
    }

    // SAFETY CHECK 5: Hardware safety switch must be in armed position
    // Physical switch on many autopilots that requires manual action
    // SAFETY_DISARMED means switch is preventing motor outputs
    if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL,"%s: Safety switch", mode);
        return false;
    }

    // SAFETY CHECK 6: Emergency stop (E-Stop) must not be active
    // E-Stop is a kill switch that immediately disables all motor outputs
    // Can be triggered by RC switch, GCS command, or external hardware
    if (SRV_Channels::get_emergency_stop()) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL,"%s: Motor Emergency Stopped", mode);
        return false;
    }

    // All safety checks passed - motor test can proceed safely
    return true;
}

/**
 * @brief Initiate motor test sequence via MAVLink command
 * 
 * @details Starts motor testing by performing safety checks, configuring test parameters,
 *          and preparing the vehicle for controlled motor activation. This function
 *          implements the MAV_CMD_DO_MOTOR_TEST MAVLink command.
 *          
 *          Test sequence initialization:
 *          1. Validate motor_count (default to 1 if zero)
 *          2. If not already testing, perform comprehensive safety checks
 *          3. Arm motors in special test mode (bypasses normal arming checks)
 *          4. Disable failsafes that would interfere with testing:
 *             - Throttle failsafe (prevents RC loss from stopping test)
 *             - GCS failsafe (prevents telemetry loss from stopping test)
 *             - EKF failsafe (prevents EKF errors from stopping test)
 *          5. Configure test parameters (sequence, duration, throttle settings)
 *          6. Enable ESC calibration notification (LED indication)
 *          
 *          Multi-motor testing:
 *          If motor_count > 1, motors are tested sequentially with inter-motor gaps.
 *          This allows clear identification of each motor in the sequence.
 *          
 * @param[in] gcs_chan          MAVLink channel for status messages
 * @param[in] motor_seq         Motor sequence number to test (hardware-dependent numbering)
 * @param[in] throttle_type     Throttle mode: 0=percentage, 1=PWM, 2=pilot input, 3=compass cal
 * @param[in] throttle_value    Throttle value (interpretation depends on throttle_type)
 * @param[in] timeout_sec       Test duration in seconds (capped at MOTOR_TEST_TIMEOUT_SEC)
 * @param[in] motor_count       Number of motors to test sequentially (0 or 1 = single motor)
 * 
 * @return MAV_RESULT_ACCEPTED if test started successfully
 * @return MAV_RESULT_FAILED if safety checks failed
 * 
 * @warning SAFETY CRITICAL: This function temporarily arms motors and disables failsafes.
 *          Multiple safety interlocks via mavlink_motor_control_check() prevent dangerous
 *          activation, but operator MUST ensure:
 *          - Propellers are removed
 *          - Vehicle is secured and cannot move
 *          - Clear area around vehicle
 *          - Ready to immediately stop test if needed
 *          
 * @note Failsafes are automatically restored when test completes or times out.
 *       The motor test flag (ap.motor_test) prevents normal flight operations.
 * 
 * @note For compass calibration mode (throttle_type=3), this function initiates
 *       per-motor magnetic interference mapping for compensation.
 * 
 * @see Copter::mavlink_motor_control_check() - Safety validation before testing
 * @see Copter::motor_test_output() - Generates motor test PWM signals
 * @see Copter::motor_test_stop() - Ends test and restores normal operation
 */
MAV_RESULT Copter::mavlink_motor_test_start(const GCS_MAVLINK &gcs_chan, uint8_t motor_seq, uint8_t throttle_type, float throttle_value,
                                         float timeout_sec, uint8_t motor_count)
{
    // Normalize motor count: treat 0 as single motor test
    if (motor_count == 0) {
        motor_count = 1;
    }
    
    // Initialize test if not already running (allows updating parameters of active test)
    if (!ap.motor_test) {
        /* Perform comprehensive safety checks before allowing motor activation
         * RC calibration check can be skipped for direct PWM mode (throttle_type == 1)
         * since RC input is not used and calibration is irrelevant
         */
        if (!mavlink_motor_control_check(gcs_chan, throttle_type != 1, "Motor Test")) {
            return MAV_RESULT_FAILED;
        } else {
            // All safety checks passed - begin test initialization
            gcs().send_text(MAV_SEVERITY_INFO, "starting motor test");

            // Inform scheduler this initialization may take up to 3 seconds
            EXPECT_DELAY_MS(3000);

            // Arm motors in special test mode
            // This bypasses normal arming checks but maintains safety interlocks
            if (!motors->armed()) {
                motors->output_min();  // Initialize outputs to minimum safe value
                motors->armed(true);   // Set motor library armed state
                hal.util->set_soft_armed(true);  // Set HAL soft-armed flag
            }

            // Temporarily disable failsafes that would interfere with testing
            // These are restored by motor_test_stop() when testing completes
            g.failsafe_throttle.set(FS_THR_DISABLED);  // Disable throttle/RC loss failsafe
            g.failsafe_gcs.set(FS_GCS_DISABLED);       // Disable GCS telemetry loss failsafe
            g.fs_ekf_action.set(FS_EKF_ACTION_REPORT_ONLY);  // Disable EKF error failsafe

            // Enable ESC calibration notification (typically flashing LEDs)
            // Visual indicator that vehicle is in motor test mode
            AP_Notify::flags.esc_calibration = true;
            
            // Set motor test active flag - prevents normal flight operations
            ap.motor_test = true;
        }
    }

    // Configure test timing with safety timeout limit
    motor_test_start_ms = AP_HAL::millis();
    motor_test_timeout_ms = MIN(timeout_sec, MOTOR_TEST_TIMEOUT_SEC) * 1000;

    // Store test parameters for motor_test_output() to use
    motor_test_seq = motor_seq;              // Which motor(s) to test
    motor_test_count = motor_count;          // How many motors in sequence
    motor_test_throttle_type = throttle_type;  // How to interpret throttle_value
    motor_test_throttle_value = throttle_value;  // Throttle setting

    // Special handling for compass calibration mode
    // Initiates magnetic interference mapping per motor
    if (motor_test_throttle_type == MOTOR_TEST_COMPASS_CAL) {
        compass.per_motor_calibration_start();
    }            

    // Test successfully started or parameters updated
    return MAV_RESULT_ACCEPTED;
}

/**
 * @brief Stop motor test and restore normal vehicle operation
 * 
 * @details Safely terminates motor testing and restores all normal vehicle systems
 *          that were modified during test initialization. This function is called:
 *          - When test duration expires (timeout)
 *          - When commanded to stop via MAVLink
 *          - When safety violation detected during test
 *          - When invalid motor sequence or PWM value encountered
 *          
 *          Cleanup sequence:
 *          1. Notify GCS that test is complete
 *          2. Disarm motors (stop all motor outputs)
 *          3. Clear timing parameters
 *          4. Restore all failsafe settings from EEPROM
 *          5. End compass calibration if active
 *          6. Clear visual indicators (LEDs)
 *          7. Clear motor test active flag
 *          
 *          After this function completes, vehicle returns to normal pre-test state
 *          with all safety systems restored and ready for normal operation.
 * 
 * @note This function is idempotent - safe to call multiple times. If motor test
 *       is not active, function returns immediately without side effects.
 * 
 * @note Failsafe parameters are reloaded from EEPROM to ensure correct restoration
 *       even if parameters were changed during the test.
 * 
 * @see Copter::mavlink_motor_test_start() - Initializes test and disables failsafes
 * @see Copter::motor_test_output() - Calls this function on timeout or error
 */
void Copter::motor_test_stop()
{
    // Exit immediately if the test is not running
    // Prevents redundant cleanup if called multiple times
    if (!ap.motor_test) {
        return;
    }

    // Notify ground control station that motor test has completed
    gcs().send_text(MAV_SEVERITY_INFO, "finished motor test");    

    // Disarm motors - immediately stops all motor outputs
    // Returns motors to safe disabled state
    motors->armed(false);
    hal.util->set_soft_armed(false);

    // Reset timeout tracking variables
    // Clears timing state for next test
    motor_test_start_ms = 0;
    motor_test_timeout_ms = 0;

    // Restore failsafe settings from EEPROM
    // Reloads user-configured failsafe behavior that was disabled during testing
    g.failsafe_throttle.load();  // Restore throttle/RC loss failsafe
    g.failsafe_gcs.load();        // Restore GCS telemetry loss failsafe
    g.fs_ekf_action.load();       // Restore EKF error failsafe action

    // End compass calibration if motor test was in compass cal mode
    // Finalizes magnetic interference compensation mapping
    if (motor_test_throttle_type == MOTOR_TEST_COMPASS_CAL) {
        compass.per_motor_calibration_end();
    }

    // Clear ESC calibration notification (stop flashing LEDs)
    // Removes visual indication that vehicle was in test mode
    AP_Notify::flags.esc_calibration = false;

    // Clear motor test active flag - re-enables normal flight operations
    // Vehicle can now be armed and flown normally
    ap.motor_test = false;
}
