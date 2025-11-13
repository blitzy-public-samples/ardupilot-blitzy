/**
 * @file radio.cpp
 * @brief RC receiver input and servo output initialization for ArduSub
 * 
 * @details This file implements the RC (Radio Control) input and output initialization
 *          for ArduSub underwater vehicles. It handles:
 *          - RC channel mapping and configuration for 6DOF control (roll, pitch, yaw, throttle, forward, lateral)
 *          - Servo output initialization and motor setup
 *          - RC failsafe detection with throttle hysteresis
 *          - ESC calibration mode gating
 *          
 *          ArduSub uses a 6-axis control scheme unique to underwater vehicles, combining
 *          traditional aircraft controls with forward/lateral translation for omnidirectional movement.
 * 
 * @note This implementation is specific to underwater vehicles and differs from surface vehicle RC handling
 * @warning Incorrect RC configuration can lead to loss of vehicle control underwater
 * 
 * Source: ArduSub/radio.cpp:1-131
 */

#include "Sub.h"

/**
 * @brief Initialize RC input channels for ArduSub 6DOF control
 * 
 * @details Configures the RC receiver input channels for underwater vehicle control.
 *          Performs the following initialization sequence:
 *          1. Maps RC channels to vehicle control axes (roll, pitch, yaw, throttle, forward, lateral)
 *          2. Sets angle ranges for attitude channels (±4500 centidegrees = ±45°)
 *          3. Configures throttle channel range (1000 units for vertical thrust)
 *          4. Sets dead zones to prevent control jitter near neutral
 *          5. Initializes RC overrides to 1500μs (neutral position) for all control channels
 *          6. Initializes camera pan/tilt channels to neutral
 *          
 *          Channel mapping:
 *          - Roll: Bank angle control (typically right stick left/right)
 *          - Pitch: Nose up/down control (typically right stick forward/back)
 *          - Yaw: Heading control (typically left stick left/right)
 *          - Throttle: Vertical thrust (typically left stick up/down)
 *          - Forward: Forward/backward translation (Sub-specific)
 *          - Lateral: Left/right translation (Sub-specific)
 *          
 *          Dead zones prevent oscillation when stick is near center. Larger dead zone
 *          for yaw (40) provides more stable heading hold than roll/pitch (30).
 * 
 * @note Called during vehicle initialization before arming is permitted
 * @note RC overrides default to 1500μs to prevent invalid zero values
 * @warning Dead zones must be tuned for specific RC hardware to prevent control issues
 * 
 * @see Sub::init_rc_out()
 * @see RC_Channel::set_angle()
 * @see RC_Channel::set_range()
 */
void Sub::init_rc_in()
{
    // Map RC channels to Sub control axes (6DOF control for underwater vehicle)
    channel_roll     = &rc().get_roll_channel();      // Bank angle (left/right tilt)
    channel_pitch    = &rc().get_pitch_channel();     // Nose angle (up/down tilt)
    channel_throttle = &rc().get_throttle_channel();  // Vertical thrust (up/down movement)
    channel_yaw      = &rc().get_yaw_channel();       // Heading rotation (spin left/right)
    channel_forward  = &rc().get_forward_channel();   // Forward/backward translation
    channel_lateral  = &rc().get_lateral_channel();   // Left/right translation (strafing)

    // Set RC channel input ranges
    // Angle channels (roll, pitch, yaw, forward, lateral): ±ROLL_PITCH_INPUT_MAX centidegrees (±45°)
    channel_roll->set_angle(ROLL_PITCH_INPUT_MAX);
    channel_pitch->set_angle(ROLL_PITCH_INPUT_MAX);
    channel_yaw->set_angle(ROLL_PITCH_INPUT_MAX);
    channel_throttle->set_range(1000);  // Throttle: 1000 units for full vertical thrust range
    channel_forward->set_angle(ROLL_PITCH_INPUT_MAX);
    channel_lateral->set_angle(ROLL_PITCH_INPUT_MAX);

    // Set default dead zones to prevent control jitter near stick neutral position
    // Dead zone units are in PWM microseconds around 1500μs center point
    channel_roll->set_default_dead_zone(30);      // ±30μs dead zone for roll
    channel_pitch->set_default_dead_zone(30);     // ±30μs dead zone for pitch
    channel_throttle->set_default_dead_zone(30);  // ±30μs dead zone for throttle
    channel_yaw->set_default_dead_zone(40);       // ±40μs dead zone for yaw (larger for stable heading hold)
    channel_forward->set_default_dead_zone(30);   // ±30μs dead zone for forward/back
    channel_lateral->set_default_dead_zone(30);   // ±30μs dead zone for lateral

    // Initialize RC input overrides to 1500μs (neutral) on all control channels
    // This prevents invalid zero values before RC receiver provides actual input
    for (int i = 0; i < 6; i++) {
        RC_Channels::set_override(i, 1500);  // Channels 0-5: roll, pitch, throttle, yaw, forward, lateral
    }

    // Initialize camera gimbal channels to neutral position
    RC_Channels::set_override(6, 1500); // Camera pan channel (left/right)
    RC_Channels::set_override(7, 1500); // Camera tilt channel (up/down)
}

/**
 * @brief Initialize motor outputs and perform ESC calibration check
 * 
 * @details Configures the motor output subsystem for ArduSub vehicle control.
 *          Performs the following initialization sequence:
 *          1. Sets motor update rate from RC_SPEED parameter (typically 50Hz or 400Hz)
 *          2. Initializes motor library with frame configuration (vectored, custom, etc.)
 *          3. Converts throttle PWM min/max values to motor output range
 *          4. Updates motor throttle range mapping
 *          5. Performs RC calibration checks and enables motor output if checks pass
 *          6. Updates auxiliary servo function mappings for peripherals
 *          
 *          ESC Calibration Mode Gating:
 *          The function calls arming.rc_calibration_checks(true) which checks if:
 *          - Pilot is requesting ESC calibration (throttle high on boot)
 *          - RC calibration is required or in progress
 *          
 *          Only if these checks pass will motor output be enabled. This prevents
 *          accidental motor startup and allows safe ESC calibration procedures.
 *          
 *          Motor Update Rate:
 *          - Standard: 50Hz for traditional ESCs
 *          - High-rate: 400Hz for modern ESCs with fast update support
 *          - Configured via RC_SPEED parameter
 * 
 * @note Called during vehicle initialization after init_rc_in()
 * @note Motor output remains disabled until arming checks pass
 * @warning ESC calibration mode can cause motors to spin at full throttle - ensure propellers are removed
 * @warning Incorrect frame configuration can cause uncontrollable vehicle behavior
 * 
 * @see Sub::init_rc_in()
 * @see Sub::enable_motor_output()
 * @see AP_Arming::rc_calibration_checks()
 */
void Sub::init_rc_out()
{
    // Set motor/ESC update rate from RC_SPEED parameter (typically 50Hz or 400Hz)
    motors.set_update_rate(g.rc_speed);
    
    // Initialize motor library with frame class and type from configuration
    // Frame class determines motor mixing algorithm (vectored, simpleROV, custom, etc.)
    motors.init((AP_Motors::motor_frame_class)g.frame_configuration.get(), AP_Motors::motor_frame_type::MOTOR_FRAME_TYPE_PLUS);
    
    // Convert RC throttle channel PWM range (typically 1000-2000μs) to motor output range
    // This ensures full motor authority across the configured RC transmitter range
    motors.convert_pwm_min_max_param(channel_throttle->get_radio_min(), channel_throttle->get_radio_max());
    
    // Update internal throttle range mapping for motor mixing calculations
    motors.update_throttle_range();

    // Enable output to motors only if RC calibration checks pass
    // This gates ESC calibration mode - motors only enabled if:
    // - Not in ESC calibration mode (throttle not high on boot), OR
    // - ESC calibration explicitly requested and safe to proceed
    if (arming.rc_calibration_checks(true)) {
        enable_motor_output();
    }

    // Refresh auxiliary servo function mappings for camera, lights, gripper, etc.
    // Maps RC channels and relay outputs to peripheral functions
    SRV_Channels::update_aux_servo_function();
}
#if AP_SUB_RC_ENABLED
/**
 * @brief Read RC receiver input and handle radio failsafe detection
 * 
 * @details Processes RC receiver input and monitors for radio link loss.
 *          Performs the following operations:
 *          1. Attempts to read RC input from receiver
 *          2. Updates last valid input timestamp if successful
 *          3. Processes throttle value and checks for failsafe conditions
 *          4. Triggers radio failsafe if no input received within timeout period
 *          
 *          Failsafe Trigger Conditions:
 *          - No RC input for RC_FS_TIMEOUT period (default 2 seconds)
 *          - Throttle failsafe enabled (g.failsafe_throttle != FS_THR_DISABLED)
 *          - Vehicle is armed OR has previously seen RC input
 *          
 *          The last condition prevents failsafe triggering on boot before RC receiver
 *          binds, while still protecting armed vehicles from radio loss.
 *          
 *          RC Input Processing:
 *          - Valid input: Updates timestamps and processes throttle via set_throttle_and_failsafe()
 *          - No input: Checks timeout and triggers failsafe if conditions met
 *          - Already in failsafe: No further action (failsafe handler manages recovery)
 * 
 * @note Called at main loop rate (typically 50Hz for Sub)
 * @note RC failsafe is independent of GCS failsafe (different timeout and actions)
 * @warning Loss of RC link underwater can be catastrophic - ensure proper failsafe configuration
 * 
 * @see Sub::set_throttle_and_failsafe()
 * @see Sub::set_failsafe_radio()
 * @see RC_Channels::read_input()
 */
void Sub::read_radio()
{ 
    const uint32_t tnow_ms = AP_HAL::millis();  // Current time in milliseconds

    // Attempt to read RC receiver input
    if (rc().read_input()) {
        // Got valid RC input - update timestamps and process throttle
        last_radio_update_ms = tnow_ms;           // Update last successful RC read time
        failsafe.last_pilot_input_ms = tnow_ms;   // Update last pilot input time (for GCS failsafe)
        set_throttle_and_failsafe(channel_throttle->get_radio_in());  // Process throttle and check for low-throttle failsafe
        return;
    }
    
    // No radio input received this iteration
    if (failsafe.radio) {
        // Already in radio failsafe state - no further action needed
        // Failsafe handler will manage recovery when RC input returns
        return;
    }

    // Check if we should trigger radio failsafe due to timeout
    // Trigger failsafe if no update from the RC receiver for RC_FS_TIMEOUT period
    const uint32_t elapsed_ms = tnow_ms - last_radio_update_ms;
    if (elapsed_ms < rc().get_fs_timeout_ms()) {
        // Not timed out yet - still within acceptable delay
        return;
    }
    
    // RC timeout detected - check if failsafe is enabled
    if (!g.failsafe_throttle) {
        // Throttle/radio failsafe not enabled in parameters - do not trigger
        return;
    }
    
    // Check if failsafe should trigger based on vehicle state
    if (!rc().has_ever_seen_rc_input() && !sub.motors.armed()) {
        // Don't trigger failsafe if:
        // - We've never seen RC input (receiver hasn't bound yet), AND
        // - Vehicle is not armed (not flying/diving)
        // This prevents nuisance failsafes during initial setup or before RC bind
        return;
    }
    
    // All failsafe trigger conditions met - log error and enter failsafe
    LOGGER_WRITE_ERROR(LogErrorSubsystem::RADIO, LogErrorCode::RADIO_LATE_FRAME);
    set_failsafe_radio(true);
}

/**
 * @brief Process throttle input and detect low-throttle radio failsafe with hysteresis
 * 
 * @details Implements throttle-based radio failsafe detection with hysteresis counter
 *          to prevent false triggers from noise or brief throttle drops.
 *          
 *          Failsafe Detection Algorithm (Hysteresis):
 *          - Monitors throttle PWM value against FS_THR_VALUE parameter threshold
 *          - Increments failsafe.radio_counter when throttle < FS_THR_VALUE
 *          - Decrements failsafe.radio_counter when throttle >= FS_THR_VALUE
 *          - Triggers failsafe only after FS_COUNTER consecutive low readings (3 iterations)
 *          - Clears failsafe only after 3 consecutive good readings
 *          
 *          This hysteresis prevents:
 *          - False failsafe from brief RC glitches or interference
 *          - Rapid failsafe oscillation near threshold
 *          - Accidental failsafe from momentary stick movements
 *          
 *          Failsafe Trigger Conditions:
 *          - Throttle failsafe enabled (g.failsafe_throttle != FS_THR_DISABLED)
 *          - Throttle PWM < g.failsafe_throttle_value for FS_COUNTER consecutive reads
 *          - Vehicle has seen RC input OR is armed
 *          - Not already in failsafe state
 *          
 *          Typical Configuration:
 *          - FS_THR_VALUE: 975μs (below normal RC low of 1000μs)
 *          - FS_COUNTER: 3 iterations (60ms at 50Hz loop rate)
 * 
 * @param[in] throttle_pwm Current throttle channel PWM value in microseconds (typically 1000-2000μs)
 * 
 * @note Called from read_radio() at main loop rate (typically 50Hz)
 * @note Hysteresis counter prevents false triggers but adds 60ms detection delay
 * @warning Setting FS_THR_VALUE too high can cause failsafe during normal low throttle operation
 * 
 * @see Sub::read_radio()
 * @see Sub::set_failsafe_radio()
 */
#define FS_COUNTER 3        // Radio failsafe requires 3 consecutive throttle values below failsafe threshold
void Sub::set_throttle_and_failsafe(uint16_t throttle_pwm)
{
    // If failsafe not enabled, pass through throttle and clear any existing RC failsafe
    if(g.failsafe_throttle == FS_THR_DISABLED) {
        set_failsafe_radio(false);  // Ensure failsafe is disengaged
        return;
    }

    // Check for low throttle value below failsafe threshold
    if (throttle_pwm < (uint16_t)g.failsafe_throttle_value) {
        // Throttle is below failsafe threshold (typically < 975μs)

        // Skip failsafe detection if already in failsafe or vehicle not ready
        if (failsafe.radio || !(rc().has_ever_seen_rc_input() || sub.motors.armed())) {
            // Don't process if:
            // - Already in failsafe (let failsafe handler manage recovery), OR
            // - Never seen RC input AND not armed (pre-bind state)
            return;
        }

        // Increment hysteresis counter for consecutive low throttle readings
        // Requires FS_COUNTER (3) consecutive low readings to trigger failsafe
        // This prevents false triggers from RC noise, interference, or brief glitches
        failsafe.radio_counter++;
        if( failsafe.radio_counter >= FS_COUNTER ) {
            failsafe.radio_counter = FS_COUNTER;  // Clamp counter to prevent overflow
            set_failsafe_radio(true);  // Trigger radio failsafe after 3 consecutive low readings
        }
    }else{
        // Throttle is at or above failsafe threshold - good signal
        
        // Decrement hysteresis counter for consecutive good throttle readings
        failsafe.radio_counter--;
        if( failsafe.radio_counter <= 0 ) {
            failsafe.radio_counter = 0;   // Clamp counter to prevent underflow

            // Disengage failsafe after FS_COUNTER (3) consecutive valid throttle values
            // This hysteresis prevents rapid failsafe oscillation near threshold
            if (failsafe.radio) {
                set_failsafe_radio(false);  // Clear radio failsafe
            }
        }
        // Throttle value is valid and passed through to flight controller
    }
}
#endif
