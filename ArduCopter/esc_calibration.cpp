/**
 * @file esc_calibration.cpp
 * @brief ESC (Electronic Speed Controller) calibration procedures for multicopter
 * 
 * @details This file implements the ESC calibration system for ArduCopter, which allows
 *          ESCs to learn the PWM range (typically 1000-2000μs) from the flight controller.
 *          ESC calibration is necessary when:
 *          - Installing new ESCs
 *          - Replacing the flight controller
 *          - ESCs are not responding correctly to throttle commands
 *          
 *          Three calibration modes are supported:
 *          1. Manual Passthrough (ESCCAL_PASSTHROUGH_IF_THROTTLE_HIGH): User controls throttle
 *          2. Always Passthrough (ESCCAL_PASSTHROUGH_ALWAYS): Continuous passthrough mode
 *          3. Automatic (ESCCAL_AUTO): Automated calibration with timing sequence
 *          
 *          Calibration Process Overview:
 *          - Manual: User raises throttle to max, power cycles vehicle, lowers throttle to min
 *          - Auto: System automatically outputs max throttle for 5s, then min throttle until power cycle
 *          
 * @warning ESC CALIBRATION IS DANGEROUS - ALL PROPELLERS MUST BE REMOVED
 * @warning Must be performed in a controlled environment with safety precautions
 * @warning Vehicle will not be responsive during calibration - manual power cycle required
 * 
 * @note Not available for helicopters (FRAME_CONFIG == HELI_FRAME)
 * @note Not available for brushed motor types
 * 
 * @see AP_Motors::set_throttle_passthrough_for_esc_calibration()
 * @see ESCCalibrationModes enum in defines.h
 * 
 * Source: ArduCopter/esc_calibration.cpp
 */

#include "Copter.h"

/*****************************************************************************
* Functions to check and perform ESC calibration
*****************************************************************************/

/**
 * @brief Threshold for detecting high throttle stick position during ESC calibration setup
 * @details Value in PWM input units (0-1000 range). Throttle must be above 950 to initiate
 *          manual ESC calibration mode. This high threshold ensures intentional activation.
 */
#define ESC_CALIBRATION_HIGH_THROTTLE   950

/**
 * @brief Check ESC calibration mode and execute appropriate calibration procedure
 * 
 * @details This function implements the ESC calibration state machine, called during vehicle
 *          startup to determine if ESC calibration should be performed. The state machine
 *          handles the following transitions:
 *          
 *          State Machine Flow:
 *          1. ESCCAL_NONE + High Throttle → Set ESCCAL_PASSTHROUGH_IF_THROTTLE_HIGH, request reboot
 *          2. ESCCAL_PASSTHROUGH_IF_THROTTLE_HIGH + High Throttle → Execute passthrough calibration
 *          3. ESCCAL_PASSTHROUGH_ALWAYS → Execute passthrough calibration (no throttle check)
 *          4. ESCCAL_AUTO → Execute automatic calibration sequence
 *          5. ESCCAL_DISABLED → No calibration performed
 *          
 *          Manual Calibration Procedure (ESCCAL_PASSTHROUGH_IF_THROTTLE_HIGH):
 *          - User raises throttle stick to maximum
 *          - System requests reboot with throttle high
 *          - After reboot, passthrough mode activated
 *          - User can then follow ESC-specific calibration (typically: max throttle → power cycle → min throttle)
 *          
 *          Safety Checks:
 *          - Waits up to 2 seconds for first radio input
 *          - Validates RC calibration before proceeding
 *          - Clears calibration flag after completion to prevent repeated calibration
 * 
 * @note Called once during Copter::init_ardupilot() startup sequence
 * @note Automatically clears ESC_CALIBRATE parameter after execution (except ESCCAL_DISABLED)
 * @note Blocks execution during calibration - vehicle is not flight-ready
 * 
 * @warning REMOVE ALL PROPELLERS before initiating ESC calibration
 * @warning Vehicle must be powered off and on (not just rebooted) between calibration steps for most ESCs
 * @warning Do not attempt to arm or fly - calibration mode requires manual power cycle to exit
 * 
 * @see esc_calibration_passthrough() for manual calibration implementation
 * @see esc_calibration_auto() for automatic calibration implementation
 * @see ESCCalibrationModes enum for available calibration modes
 */
void Copter::esc_calibration_startup_check()
{
    // ESC calibration not supported for brushed motors (no PWM range learning needed)
    if (motors->is_brushed_pwm_type()) {
        return;
    }

#if FRAME_CONFIG != HELI_FRAME
    // Wait up to 2 seconds (100 iterations × 20ms) for first radio input to be received
    // This ensures we have valid throttle stick position before checking calibration conditions
    uint8_t i = 0;
    while ((i++ < 100) && (last_radio_update_ms == 0)) {
        hal.scheduler->delay(20);
        read_radio();
    }

    // Verify RC radio is properly calibrated before allowing ESC calibration
    // Prevents calibration with invalid or uncalibrated radio inputs
    if (!arming.rc_calibration_checks(true)) {
        // Clear any pending ESC calibration flag to prevent stuck state
        if ((g.esc_calibrate != ESCCalibrationModes::ESCCAL_NONE) && (g.esc_calibrate != ESCCalibrationModes::ESCCAL_DISABLED)) {
            g.esc_calibrate.set_and_save(ESCCalibrationModes::ESCCAL_NONE);
        }
        return;
    }

    // ESC Calibration State Machine - Execute appropriate calibration mode based on ESC_CALIBRATE parameter
    switch (g.esc_calibrate) {
        case ESCCalibrationModes::ESCCAL_NONE:
            // State 1: Normal operation - check for calibration initiation
            // If pilot has raised throttle stick to maximum (>95%), prepare for manual calibration
            if (channel_throttle->get_control_in() >= ESC_CALIBRATION_HIGH_THROTTLE) {
                // Transition to passthrough state for next boot cycle
                // This two-step process ensures intentional activation (throttle high on two consecutive boots)
                g.esc_calibrate.set_and_save(ESCCalibrationModes::ESCCAL_PASSTHROUGH_IF_THROTTLE_HIGH);
                
                // Notify ground station that reboot is required with throttle still high
                gcs().send_text(MAV_SEVERITY_CRITICAL,"ESC calibration: Restart board");
                
                // Activate visual notification (LEDs/buzzer) to indicate calibration mode pending
                AP_Notify::flags.esc_calibration = true;
                
                // Block execution in infinite loop until user power cycles the vehicle
                // This prevents any flight operations and ensures clean state for calibration
                while(1) { hal.scheduler->delay(5); }
            }
            break;
            
        case ESCCalibrationModes::ESCCAL_PASSTHROUGH_IF_THROTTLE_HIGH:
            // State 2: Conditional passthrough mode - verify throttle still high after reboot
            // This confirms user intent (throttle must remain high through power cycle)
            if (channel_throttle->get_control_in() >= ESC_CALIBRATION_HIGH_THROTTLE) {
                // Enter manual passthrough calibration mode
                // User controls throttle directly - typically: keep high, power cycle ESCs, then lower to min
                esc_calibration_passthrough();
            }
            // If throttle not high, fall through to clear the flag below (abort calibration)
            break;
            
        case ESCCalibrationModes::ESCCAL_PASSTHROUGH_ALWAYS:
            // State 3: Unconditional passthrough mode - always enter calibration regardless of throttle
            // Used for testing or when throttle position check is undesirable
            esc_calibration_passthrough();
            break;
            
        case ESCCalibrationModes::ESCCAL_AUTO:
            // State 4: Automatic calibration mode - system controls throttle sequence
            // No pilot input required - automatically outputs max throttle then min throttle
            esc_calibration_auto();
            break;
            
        case ESCCalibrationModes::ESCCAL_DISABLED:
        default:
            // State 5: Calibration disabled - no action taken
            // Used to permanently disable calibration feature
            break;
    }

    // Reset calibration flag to ESCCAL_NONE after execution to prevent repeated calibration on subsequent boots
    // Exception: ESCCAL_DISABLED is sticky and remains set (allows permanent disabling of feature)
    if (g.esc_calibrate != ESCCalibrationModes::ESCCAL_DISABLED) {
        g.esc_calibrate.set_and_save(ESCCalibrationModes::ESCCAL_NONE);
    }
#endif  // FRAME_CONFIG != HELI_FRAME
}

/**
 * @brief Manual ESC calibration mode - passes pilot throttle input directly to ESCs
 * 
 * @details Implements passthrough mode where the pilot's throttle stick position is directly
 *          output to all motors without any processing or safety checks. This allows ESCs to
 *          learn the PWM range from the flight controller.
 *          
 *          Typical Manual Calibration Procedure:
 *          1. User raises throttle stick to maximum before powering on
 *          2. System boots into this passthrough mode
 *          3. ESCs receive maximum PWM signal and enter calibration mode (usually indicated by beeps)
 *          4. User power cycles the ESCs (or entire vehicle depending on ESC type)
 *          5. ESCs store maximum PWM value
 *          6. User lowers throttle stick to minimum
 *          7. ESCs receive minimum PWM signal
 *          8. ESCs store minimum PWM value and exit calibration mode (completion beeps)
 *          9. User power cycles vehicle to exit passthrough mode
 *          
 *          Implementation Details:
 *          - Runs in infinite loop at ~333Hz (3ms delay) for oneshot ESC compatibility
 *          - Converts throttle input (0-1000 range) to normalized float (0.0-1.0)
 *          - Uses SRV_Channels cork/push mechanism for synchronized output
 *          - Normal PWM ESCs receive pulses at RC_SPEED rate (configured update rate)
 *          - Oneshot ESCs receive pulses on every push() call
 * 
 * @note This function never returns - infinite loop until power cycle
 * @note Bypasses all normal flight control, safety checks, and motor mixing
 * @note LED notification flashes continuously to indicate calibration active
 * @note Called at 333Hz to support high-rate oneshot ESC protocols
 * 
 * @warning PROPELLERS MUST BE REMOVED - Motors will spin at full throttle
 * @warning Vehicle is completely unresponsive - no safety features active
 * @warning Requires manual power cycle to exit - no software escape
 * @warning Pilot has direct control of motor outputs - exercise extreme caution
 * @warning Ensure adequate power supply - all ESCs will be driven simultaneously
 * 
 * @see esc_calibration_setup() for initialization sequence
 * @see esc_calibration_notify() for LED feedback implementation
 * @see AP_Motors::set_throttle_passthrough_for_esc_calibration() for motor output
 */
void Copter::esc_calibration_passthrough()
{
#if FRAME_CONFIG != HELI_FRAME
    // Notify ground control station that passthrough mode is active
    gcs().send_text(MAV_SEVERITY_INFO,"ESC calibration: Passing pilot throttle to ESCs");

    // Perform common calibration setup: clear flag, set update rate, arm motors, wait for safety switch
    esc_calibration_setup();

    // Infinite loop - only exits via power cycle
    while(1) {
        // Flash LEDs and update notification system to indicate calibration in progress
        esc_calibration_notify();

        // Read current radio input values from receiver
        // Updates channel_throttle with pilot's stick position
        read_radio();

        // Run at 333Hz (3ms period) to support oneshot ESC protocols
        // Oneshot ESCs require high-rate pulses for calibration
        // Normal PWM ESCs will only output at RC_SPEED rate despite this loop frequency
        hal.scheduler->delay(3);

        // Output pilot throttle directly to all motors via passthrough mechanism
        auto &srv = AP::srv();
        srv.cork();  // Batch servo outputs for atomic update
        
        // Convert throttle input from 0-1000 range to normalized 0.0-1.0 float
        // set_throttle_passthrough_for_esc_calibration() bypasses motor mixing and outputs to all motors
        motors->set_throttle_passthrough_for_esc_calibration(channel_throttle->get_control_in() * 0.001f);
        
        srv.push();  // Commit all servo outputs simultaneously
    }
#endif  // FRAME_CONFIG != HELI_FRAME
}

/**
 * @brief Automatic ESC calibration - executes timed calibration sequence without pilot input
 * 
 * @details Implements fully automatic ESC calibration using a predefined timing sequence.
 *          No pilot throttle input required - system controls the entire calibration process.
 *          
 *          Automatic Calibration Sequence:
 *          1. System initializes calibration mode (setup, arm, enable motors)
 *          2. Immediately output maximum throttle (1.0 = 100%) to all ESCs
 *          3. Hold maximum throttle for 5000ms (5 seconds)
 *          4. ESCs detect and store maximum PWM value during this period
 *          5. After 5 seconds, switch to minimum throttle (0.0 = 0%)
 *          6. Hold minimum throttle indefinitely until power cycle
 *          7. ESCs detect and store minimum PWM value
 *          8. User must power cycle vehicle to complete calibration and exit mode
 *          
 *          Timing Details:
 *          - Maximum throttle phase: 5000ms (provides time for ESC calibration mode entry)
 *          - Minimum throttle phase: Infinite (until power cycle)
 *          - Update rate: 333Hz (3ms loop period) for oneshot ESC compatibility
 *          
 *          Advantages over Manual Calibration:
 *          - No pilot input required - eliminates stick position errors
 *          - Consistent timing - reliable calibration for all ESC types
 *          - Simpler procedure - just power on and wait
 *          
 *          ESC Compatibility:
 *          - Works with most ESCs that support traditional calibration
 *          - 5-second maximum throttle period sufficient for most ESC calibration entry
 *          - Some ESCs may require longer periods - use manual mode if auto fails
 * 
 * @note This function never returns - infinite loop until power cycle
 * @note Bypasses all normal flight control and safety checks
 * @note LED notification flashes continuously during calibration
 * @note Total calibration time: ~5 seconds active, then wait for user power cycle
 * 
 * @warning PROPELLERS MUST BE REMOVED - Motors will spin at full throttle for 5 seconds
 * @warning Ensure adequate power supply - all motors driven at maximum simultaneously
 * @warning Vehicle completely unresponsive during calibration
 * @warning Must power cycle (not reboot) to exit calibration mode
 * @warning Do not disconnect power during the 5-second maximum throttle phase
 * @warning Recommended for experienced users - manual mode safer for beginners
 * 
 * @see esc_calibration_setup() for initialization sequence
 * @see esc_calibration_notify() for LED feedback implementation
 * @see AP_Motors::set_throttle_passthrough_for_esc_calibration() for motor output
 */
void Copter::esc_calibration_auto()
{
#if FRAME_CONFIG != HELI_FRAME
    // Notify ground control station that automatic calibration sequence is starting
    gcs().send_text(MAV_SEVERITY_INFO,"ESC calibration: Auto calibration");

    // Perform common calibration setup: clear flag, set update rate, arm motors, wait for safety switch
    esc_calibration_setup();

    // Phase 1: Output maximum throttle to all ESCs for calibration high point detection
    auto &srv = AP::srv();
    srv.cork();
    motors->set_throttle_passthrough_for_esc_calibration(1.0f);  // 1.0 = 100% throttle (maximum PWM)
    srv.push();

    // Maintain maximum throttle for exactly 5000ms (5 seconds)
    // This duration allows ESCs to:
    // 1. Detect sustained high throttle and enter calibration mode
    // 2. Sample and store the maximum PWM value
    // 3. Provide user feedback (typically beep patterns)
    uint32_t tstart = millis();
    while (millis() - tstart < 5000) {
        srv.cork();
        motors->set_throttle_passthrough_for_esc_calibration(1.0f);
        srv.push();
        esc_calibration_notify();  // Flash LEDs to indicate calibration active
        hal.scheduler->delay(3);   // 333Hz update rate for oneshot ESC support
    }

    // Phase 2: Output minimum throttle indefinitely until power cycle
    // ESCs will detect zero throttle, store minimum PWM value, and complete calibration
    // Infinite loop provides stable minimum signal - user must power cycle to exit
    while(1) {
        srv.cork();
        motors->set_throttle_passthrough_for_esc_calibration(0.0f);  // 0.0 = 0% throttle (minimum PWM)
        srv.push();
        esc_calibration_notify();  // Continue LED notification
        hal.scheduler->delay(3);   // Maintain 333Hz rate
    }
#endif // FRAME_CONFIG != HELI_FRAME
}

/**
 * @brief Flash LEDs and activate buzzer to provide visual/audio feedback during ESC calibration
 * 
 * @details Provides continuous notification to user that ESC calibration is in progress.
 *          Updates notification system at controlled rate to produce flashing pattern without
 *          overwhelming the notification system.
 *          
 *          Notification Implementation:
 *          - Sets esc_calibration flag to trigger special LED flash pattern
 *          - Updates at maximum rate of 50Hz (20ms minimum interval)
 *          - Rate limiting prevents excessive CPU usage in tight calibration loops
 *          - Notification system handles actual LED/buzzer patterns based on flag
 *          
 *          User Feedback:
 *          - Provides clear indication that vehicle is in calibration mode
 *          - Warns user not to attempt flight operations
 *          - Confirms system is active and processing calibration
 * 
 * @note Called repeatedly in calibration loops (typically at 333Hz)
 * @note Internal rate limiting ensures notify.update() called at max 50Hz
 * @note Actual LED pattern defined by AP_Notify based on esc_calibration flag
 * @note Uses esc_calibration_notify_update_ms member variable for rate limiting
 * 
 * @see AP_Notify for LED pattern implementations
 * @see esc_calibration_passthrough() and esc_calibration_auto() for usage context
 */
void Copter::esc_calibration_notify()
{
    // Set flag to trigger ESC calibration notification pattern
    // AP_Notify uses this flag to select appropriate LED flash and buzzer pattern
    AP_Notify::flags.esc_calibration = true;
    
    // Rate limit notification updates to 50Hz maximum (20ms minimum interval)
    // Prevents excessive update calls when running in fast calibration loops
    uint32_t now = AP_HAL::millis();
    if (now - esc_calibration_notify_update_ms > 20) {
        esc_calibration_notify_update_ms = now;
        notify.update();  // Trigger LED and buzzer update
    }
}

/**
 * @brief Initialize system for ESC calibration mode
 * 
 * @details Performs common setup sequence required before any ESC calibration mode can execute.
 *          Configures motor output rates, handles safety switch requirements, and arms the
 *          vehicle in a special calibration-safe state that bypasses normal arming checks.
 *          
 *          Setup Sequence:
 *          1. Clear ESC_CALIBRATE parameter (prevents repeated calibration)
 *          2. Configure motor update rate based on ESC protocol type
 *          3. Initialize safety switch system
 *          4. Wait for safety switch to be pressed (if equipped)
 *          5. Arm motors and enable outputs
 *          
 *          Motor Update Rate Configuration:
 *          - Normal PWM/Oneshot ESCs: Use RC_SPEED parameter (typically 50-490Hz)
 *          - Digital protocols (DShot, etc.): Fixed 50Hz rate
 *          - Rate selection ensures compatibility with ESC calibration requirements
 *          
 *          Safety Switch Handling:
 *          - Waits indefinitely for safety switch press if switch is present
 *          - Reminds user every 5 seconds to press safety switch
 *          - Some boards have no safety switch (automatically passes)
 *          - Safety switch press is final safety confirmation before motor output
 *          
 *          Arming Procedure:
 *          - Bypasses normal pre-arm checks (GPS, calibration, etc.)
 *          - Directly arms motors via motors->armed(true)
 *          - Enables motor output channels via SRV_Channels
 *          - Sets soft_armed flag for system state consistency
 * 
 * @note Called by both esc_calibration_passthrough() and esc_calibration_auto()
 * @note Blocks until safety switch pressed (if equipped)
 * @note Saves ESC_CALIBRATE parameter as ESCCAL_NONE to EEPROM
 * @note Motor arming bypasses all normal safety checks - only for calibration use
 * 
 * @warning Enables motor outputs - propellers will spin if attached
 * @warning Safety checks are bypassed - vehicle is not flight-ready
 * @warning Safety switch is last line of defense - ensure props removed before pressing
 * @warning Writes to EEPROM (parameter save) - rapid repeated calls can wear EEPROM
 * 
 * @see esc_calibration_passthrough() for manual calibration usage
 * @see esc_calibration_auto() for automatic calibration usage
 * @see AP_Motors::set_update_rate() for motor output rate configuration
 */
void Copter::esc_calibration_setup()
{
    // Clear calibration parameter and save to EEPROM to prevent repeated calibration on next boot
    // CRITICAL: Must save before enabling motors to ensure clean state if power lost during calibration
    g.esc_calibrate.set_and_save(ESCCAL_NONE);

    // Configure motor update rate based on ESC protocol type
    if (motors->is_normal_pwm_type()) {
        // Normal PWM or Oneshot ESCs: Use configured RC_SPEED parameter
        // Oneshot ESCs benefit from higher update rates (actual pulses sent on srv.push())
        // Typical range: 50Hz (normal PWM) to 490Hz (oneshot)
        motors->set_update_rate(g.rc_speed);
    } else {
        // Digital ESC protocols (DShot, etc.): Use fixed 50Hz rate
        // Digital protocols don't require high update rates for calibration
        // 50Hz provides stable, predictable pulses for calibration detection
        motors->set_update_rate(50);
    }

    // Initialize safety switch system according to board configuration
    // Respects SAFETY_ENABLE parameter and board-specific safety requirements
    BoardConfig.init_safety();

    // Wait for safety switch to be pressed (if equipped)
    // This provides final user confirmation before enabling motor outputs
    // Prevents accidental motor activation during setup
    uint32_t tstart = 0;
    while (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        const uint32_t tnow = AP_HAL::millis();
        
        // Remind user every 5 seconds to press safety switch
        // Prevents user confusion if waiting without feedback
        if (tnow - tstart >= 5000) {
            gcs().send_text(MAV_SEVERITY_INFO,"ESC calibration: Push safety switch");
            tstart = tnow;
        }
        
        // Continue LED notification while waiting
        esc_calibration_notify();
        hal.scheduler->delay(3);
    }

    // Arm motors and enable outputs for calibration
    // CRITICAL: This bypasses all normal safety checks (GPS, pre-arm, etc.)
    motors->armed(true);                                      // Set motors to armed state
    SRV_Channels::enable_by_mask(motors->get_motor_mask());  // Enable output channels for all motors
    hal.util->set_soft_armed(true);                          // Set system armed flag for state consistency
}
