/**
 * @file motors.cpp
 * @brief Motor control integration and safety management for ArduCopter
 * 
 * @details This file implements the critical interface between the flight control
 *          system and the AP_Motors library, managing motor output, arming/disarming
 *          logic, motor interlock control, and safety features. It handles the
 *          translation of flight controller commands into ESC signals while enforcing
 *          multiple layers of safety checks.
 * 
 *          Key responsibilities:
 *          - Motor output coordination with flight modes
 *          - ESC arming protocol management
 *          - Motor interlock state control
 *          - Auto-disarm safety timer
 *          - Lost vehicle alarm triggering
 *          - Emergency motor shutoff enforcement
 *          - Servo channel output coordination
 * 
 *          Safety-critical paths in this file directly control motor output and
 *          must be thoroughly tested before any modifications.
 * 
 * @note This file is called at multiple rates: 10Hz for safety checks, main loop
 *       rate (typically 400Hz) for motor output, and rate thread for fast updates
 * 
 * @warning Any modifications to motor output logic can affect vehicle stability
 *          and must be tested extensively in SITL before hardware testing
 * 
 * @see AP_Motors - Motor mixing and output library
 * @see SRV_Channels - Servo output management
 * @see Copter::init_rc_out() - Motor output initialization
 * 
 * Source: ArduCopter/motors.cpp
 */

#include "Copter.h"

// Timing constants for safety checks (called at 10Hz scheduler rate)
#define ARM_DELAY               20  // called at 10hz so 2 seconds
#define DISARM_DELAY            20  // called at 10hz so 2 seconds
#define LOST_VEHICLE_DELAY      10  // called at 10hz so 1 second

// Auto-disarm timer start time (milliseconds since boot)
static uint32_t auto_disarm_begin;

/**
 * @brief Automatic disarm safety check - disarms copter after timeout on ground
 * 
 * @details Monitors vehicle state and automatically disarms the motors if the copter
 *          has been sitting on the ground with throttle low for a configurable period.
 *          This safety feature prevents accidental motor spin-up and conserves battery.
 * 
 *          Disarm conditions checked:
 *          - Motors armed (otherwise nothing to do)
 *          - Auto-disarm enabled (g.disarm_delay > 0)
 *          - Not in THROW mode (special handling for throw launches)
 *          - Motors at or below GROUND_IDLE spool state
 *          - Throttle stick at zero (or in deadband for sprung throttles)
 *          - Vehicle detected as landed (ap.land_complete)
 * 
 *          Special cases for faster disarm:
 *          - Motor interlock switch disengaged: Uses half the configured delay
 *          - Emergency stop active: Uses half the configured delay
 *          These conditions are less obvious to the pilot as motors aren't spinning
 * 
 *          Algorithm:
 *          1. Check if vehicle is armed and auto-disarm enabled
 *          2. Reset timer if motors spooling up or conditions not met
 *          3. Check for interlock/emergency stop (reduces delay by 50%)
 *          4. For normal operation, verify throttle low and landed
 *          5. Disarm when timer exceeds configured delay
 * 
 * @note Called at 10Hz from the scheduler (100ms intervals)
 * @note Configurable delay via g.disarm_delay parameter (0-127 seconds)
 * @note For helicopters (HELI_FRAME), interlock delay is NOT reduced
 * 
 * @warning Disabling auto-disarm (g.disarm_delay=0) means vehicle will remain
 *          armed indefinitely until manually disarmed - use with caution
 * 
 * @see AP_Arming::disarm() - Performs the actual disarm operation
 * @see AP_Motors::get_spool_state() - Motor spin-up state
 * @see Copter::set_land_complete() - Landing detection
 * 
 * Source: ArduCopter/motors.cpp:10-56
 */
void Copter::auto_disarm_check()
{
    // Get current time for timeout calculations
    uint32_t tnow_ms = millis();
    
    // Convert g.disarm_delay parameter (seconds) to milliseconds
    // Parameter constrained to 0-127 seconds for safety
    uint32_t disarm_delay_ms = 1000*constrain_int16(g.disarm_delay, 0, 127);

    // Exit immediately if we are already disarmed, or if auto
    // disarming is disabled (g.disarm_delay == 0)
    // Also skip for THROW mode which has special arming behavior
    if (!motors->armed() || disarm_delay_ms == 0 || flightmode->mode_number() == Mode::Number::THROW) {
        auto_disarm_begin = tnow_ms;  // Reset timer for next check
        return;
    }

    // If the rotors are still spinning above ground idle, don't initiate auto disarm
    // Motor spool states: SHUT_DOWN < GROUND_IDLE < SPOOLING_UP < THROTTLE_UNLIMITED < SPOOLING_DOWN
    // This prevents disarming during takeoff or while motors are winding down
    if (motors->get_spool_state() > AP_Motors::SpoolState::GROUND_IDLE) {
        auto_disarm_begin = tnow_ms;  // Reset timer - motors active
        return;
    }

    // Always allow auto disarm if using interlock switch or motors are Emergency Stopped
    // These conditions bypass normal throttle/landing checks for faster disarm
    if ((ap.using_interlock && !motors->get_interlock()) || SRV_Channels::get_emergency_stop()) {
#if FRAME_CONFIG != HELI_FRAME
        // Use a shorter delay (50%) if using throttle interlock switch or Emergency Stop
        // because it is less obvious the copter is armed as the motors will not be spinning.
        // This provides faster disarm when pilot has explicitly cut motor power.
        // Note: Helicopters exempt from this - they use full delay regardless
        disarm_delay_ms /= 2;
#endif
    } else {
        // Normal disarm path - check throttle position and landing status
        
        // Determine if using sprung (self-centering) throttle stick
        // THR_BEHAVE_FEEDBACK_FROM_MID_STICK means throttle centers at mid position
        bool sprung_throttle_stick = (g.throttle_behavior & THR_BEHAVE_FEEDBACK_FROM_MID_STICK) != 0;
        bool thr_low;
        
        // Check if throttle is in low position (disarm-safe position)
        if (flightmode->has_manual_throttle() || !sprung_throttle_stick) {
            // For manual throttle modes or non-sprung sticks, use direct throttle zero flag
            thr_low = ap.throttle_zero;
        } else {
            // For sprung throttles in auto modes, check if stick is at or below mid+deadband
            // This allows disarm when stick returns to center in modes like Loiter/PosHold
            float deadband_top = get_throttle_mid() + g.throttle_deadzone;
            thr_low = channel_throttle->get_control_in() <= deadband_top;
        }

        // Reset timer if throttle is not low or if vehicle is not detected as landed
        // Both conditions must be true to proceed with auto disarm
        if (!thr_low || !ap.land_complete) {
            auto_disarm_begin = tnow_ms;  // Reset timer - not safe to disarm yet
        }
    }

    // Disarm once timer expires - timeout period has elapsed with all conditions met
    if ((tnow_ms-auto_disarm_begin) >= disarm_delay_ms) {
        arming.disarm(AP_Arming::Method::DISARMDELAY);  // Trigger disarm with delay method
        auto_disarm_begin = tnow_ms;  // Reset timer for next arm cycle
    }
}

/**
 * @brief Main motor output function - coordinates all motor and servo output to ESCs
 * 
 * @details This is the central motor control function that manages the complete output
 *          pipeline from flight controller to ESCs and servos. It handles motor arming
 *          protocols, interlock state management, emergency stops, and coordinates
 *          between the flight mode output and the AP_Motors library.
 * 
 *          Output pipeline sequence:
 *          1. Check for advanced failsafe vehicle termination
 *          2. Update arming delay state tracking
 *          3. Calculate servo PWM values (SRV_Channels)
 *          4. Cork output to synchronize all channels
 *          5. Output auxiliary/passthrough channels
 *          6. Update motor interlock state with safety checks
 *          7. Generate motor outputs (either test or flight mode)
 *          8. Push outputs to hardware (ESCs and servos)
 * 
 *          Motor interlock conditions (all must be true):
 *          - Motors armed (pilot armed the vehicle)
 *          - Not in arming delay period (ARMING_DELAY_SEC seconds after arm)
 *          - Interlock switch enabled (if using interlock, ap.motor_interlock_switch)
 *          - Emergency stop not active (hardware emergency stop button)
 * 
 *          The function supports two output modes:
 *          - Normal: Flight mode generates motor outputs via output_to_motors()
 *          - Test: Motor test sequence overrides normal output
 * 
 * @param[in] full_push If true, pushes all outputs including servos at main loop rate.
 *                      If false, pushes only motor outputs for higher rate updates.
 *                      Set true when slower servo updates are needed (typically main loop).
 *                      Set false for fast motor-only updates (rate thread).
 * 
 * @note Called at main loop rate (typically 400Hz) or from rate thread for fast updates
 * @note Motor outputs are synchronized using cork/push mechanism to ensure simultaneous
 *       ESC signal transmission
 * @note Arming delay (ARMING_DELAY_SEC) provides safety window after arming before
 *       motors can spin - prevents immediate throttle response on arm
 * 
 * @warning This function directly controls motor output - modifications can affect
 *          vehicle stability, safety, and flight characteristics. Extensive SITL
 *          and ground testing required before flight testing any changes.
 * 
 * @warning Motor interlock state changes are safety-critical and logged as events
 *          (MOTORS_INTERLOCK_ENABLED/DISABLED) for post-flight analysis
 * 
 * @warning Advanced failsafe termination can deliberately crash the vehicle to meet
 *          OBC (Onboard Controller) safety requirements - this is intentional behavior
 * 
 * @see AP_Motors::output() - Motor mixing and ESC signal generation
 * @see Mode::output_to_motors() - Flight mode specific motor output
 * @see Copter::motor_test_output() - Motor testing output generation
 * @see SRV_Channels::calc_pwm() - Servo PWM calculation
 * @see AP_Motors::set_interlock() - Motor interlock control
 * 
 * Source: ArduCopter/motors.cpp:60-117
 */
void Copter::motors_output(bool full_push)
{
#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
    // Advanced Failsafe System (AFS) - OBC compliance for autonomous operations
    // This is to allow the failsafe module to deliberately crash the vehicle
    // in extreme circumstances to meet regulatory OBC (Onboard Controller) rules.
    // Only used when geofence breach or other critical failure requires immediate
    // vehicle termination for safety (e.g., flying over restricted airspace).
    if (g2.afs.should_crash_vehicle()) {
        g2.afs.terminate_vehicle();  // Cut motors or initiate controlled crash
        if (!g2.afs.terminating_vehicle_via_landing()) {
            return;  // Exit immediately if hard termination - no further motor output
        }
        // If terminating via controlled landing, continue running motors_output
        // to allow the landing sequence to complete normally
    }
#endif

    // Update arming delay state - tracks the safety period after arming
    // Exit arming delay if:
    // - Motors disarmed (pilot disarmed, no delay needed)
    // - Delay period expired (ARMING_DELAY_SEC seconds elapsed)
    // - THROW mode active (throw launches need immediate response)
    if (ap.in_arming_delay && (!motors->armed() || millis()-arm_time_ms > ARMING_DELAY_SEC*1.0e3f || flightmode->mode_number() == Mode::Number::THROW)) {
        ap.in_arming_delay = false;
    }

    // Calculate PWM values for all servo channels
    // This updates servo output values based on configured functions and input sources
    SRV_Channels::calc_pwm();

    // Get reference to servo output singleton for coordinated channel updates
    auto &srv = AP::srv();

    // Cork output channels - buffers all changes until push() is called
    // This ensures all ESC and servo signals update simultaneously on the same
    // hardware timer cycle, preventing timing skew between channels that could
    // cause unstable motor mixing or servo jitter
    srv.cork();

    // Update output on any auxiliary channels configured for manual passthrough
    // Allows direct RC input → servo output for camera gimbals, grippers, etc.
    SRV_Channels::output_ch_all();

    // Update motor interlock state - critical safety gate before motors can spin
    // Motor interlock is the final safety check that enables motor output.
    // All conditions must be TRUE for interlock to engage:
    // 1. motors->armed() - Vehicle armed by pilot
    // 2. !ap.in_arming_delay - Arming delay period has expired
    // 3. (!ap.using_interlock || ap.motor_interlock_switch) - If interlock switch
    //    configured, it must be in the enabled position
    // 4. !SRV_Channels::get_emergency_stop() - Hardware emergency stop not active
    bool interlock = motors->armed() && !ap.in_arming_delay && (!ap.using_interlock || ap.motor_interlock_switch) && !SRV_Channels::get_emergency_stop();
    
    // Enable interlock if conditions met and currently disabled
    if (!motors->get_interlock() && interlock) {
        motors->set_interlock(true);
        LOGGER_WRITE_EVENT(LogEvent::MOTORS_INTERLOCK_ENABLED);  // Log for analysis
    } 
    // Disable interlock if conditions not met and currently enabled
    else if (motors->get_interlock() && !interlock) {
        motors->set_interlock(false);
        LOGGER_WRITE_EVENT(LogEvent::MOTORS_INTERLOCK_DISABLED);  // Log for analysis
    }

    // Generate motor outputs - either test mode or normal flight mode
    if (ap.motor_test) {
        // Motor test mode active - output test signals to motors
        // Test mode allows individual motor testing for diagnostics and setup
        // Bypasses normal flight controller to directly command specific motors
        motor_test_output();
    } else {
        // Normal flight operation - let the active flight mode generate motor outputs
        // Each mode (Stabilize, Loiter, Auto, etc.) implements its own motor control
        // strategy via output_to_motors(), which calls into AP_Motors for mixing
        flightmode->output_to_motors();
    }

    // Push buffered outputs to hardware ESCs and servos
    // This "uncorks" the outputs and sends all PWM signals simultaneously
    if (full_push) {
        // Full push: Updates motors, servos, and all auxiliary outputs
        // Used at main loop rate to ensure servos and slower devices get updates
        // Typically 400Hz for motors, with servo updates as needed
        srv.push();
    } else {
        // Fast push: Motor outputs only for high-rate control
        // Used in rate thread to update motors at faster rates (e.g., 1000Hz+)
        // without the overhead of servo calculations
        // Provides better motor response for aggressive flight
        hal.rcout->push();
    }
}

/**
 * @brief Main thread motor output wrapper - handles thread-aware motor updates
 * 
 * @details Provides thread-safe motor output coordination for systems running
 *          separate rate threads. When a dedicated rate thread is active, it
 *          handles high-frequency motor updates, and this function becomes a no-op
 *          from the main thread to avoid conflicting outputs.
 * 
 *          Thread configuration:
 *          - using_rate_thread = false: Main thread calls motors_output() directly
 *          - using_rate_thread = true: Rate thread handles motor output, main thread skips
 * 
 *          This architecture allows high-performance flight controllers to run
 *          motor updates at very high rates (1000Hz+) in a dedicated thread while
 *          the main thread handles lower-priority tasks at 400Hz.
 * 
 * @note Called from the main scheduler loop at main loop rate (typically 400Hz)
 * @note When rate thread is active, actual motor output occurs in rate_loop()
 * @note Full push (servos + motors) always done from main thread context
 * 
 * @see Copter::motors_output() - Actual motor output implementation
 * @see Copter::rate_loop() - High-rate motor updates in rate thread
 * @see Copter::setup() - Rate thread initialization
 * 
 * Source: ArduCopter/motors.cpp:120-125
 */
void Copter::motors_output_main()
{
    // Only output motors from main thread if not using a dedicated rate thread
    // If rate thread is active, it handles motor updates at higher frequency
    if (!using_rate_thread) {
        motors_output();  // Call main motor output function with default (full) push
    }
}

/**
 * @brief Lost vehicle alarm - triggers audible/visual alarm from stick input
 * 
 * @details Monitors pilot RC stick inputs for a special pattern that triggers the
 *          lost vehicle alarm. This helps pilots locate their copter after landing
 *          in tall grass, trees, or other hard-to-see locations. The alarm is
 *          triggered by holding roll and pitch sticks to maximum with throttle low
 *          and motors disarmed for LOST_VEHICLE_DELAY iterations.
 * 
 *          Activation conditions (all must be true for LOST_VEHICLE_DELAY):
 *          - Throttle at zero (ap.throttle_zero)
 *          - Motors disarmed (safe state, won't spin up)
 *          - Roll stick > 4000 (full right in PWM units)
 *          - Pitch stick > 4000 (full forward in PWM units)
 * 
 *          Safety features:
 *          - Disabled if aux switch configured for lost vehicle alarm (prevents conflicts)
 *          - Requires motors disarmed (cannot trigger during flight)
 *          - Requires sustained input (LOST_VEHICLE_DELAY = 1 second at 10Hz)
 *          - Auto-deactivates when sticks return to normal
 * 
 *          When activated:
 *          - Sets AP_Notify::flags.vehicle_lost = true
 *          - AP_Notify triggers buzzer pattern and LED sequence
 *          - GCS receives "Locate Copter alarm" text message
 * 
 * @note Called at 10Hz from the scheduler (100ms intervals)
 * @note LOST_VEHICLE_DELAY = 10 iterations = 1 second hold time
 * @note Counter is static to persist between calls but reset on condition failure
 * @note RC input values: 0 = full left/back, 4500 = center, 9000 = full right/forward
 * 
 * @see AP_Notify - Controls buzzers, LEDs, and other notification devices
 * @see RC_Channel - RC input processing and scaling
 * @see RC_Channel::AUX_FUNC::LOST_VEHICLE_SOUND - Aux switch alternative
 * 
 * Source: ArduCopter/motors.cpp:128-153
 */
void Copter::lost_vehicle_check()
{
    // Static counter persists between function calls to track sustained input
    static uint8_t soundalarm_counter;

    // Disable automatic stick-triggered alarm if aux switch is configured for this function
    // This prevents conflicts between manual switch control and stick position triggering
    if (rc().find_channel_for_option(RC_Channel::AUX_FUNC::LOST_VEHICLE_SOUND)) {
        return;
    }

    // Check if pilot is holding the activation stick pattern
    // Ensure throttle is down, motors not armed, pitch and roll RC at maximum
    // Note: rc1=roll rc2=pitch in traditional RC mapping
    // Stick values > 4000 represent near-maximum deflection (range is 0-9000 internal units)
    if (ap.throttle_zero && !motors->armed() && (channel_roll->get_control_in() > 4000) && (channel_pitch->get_control_in() > 4000)) {
        // Pattern detected - increment counter toward activation threshold
        if (soundalarm_counter >= LOST_VEHICLE_DELAY) {
            // Counter has reached threshold (1 second sustained) - activate alarm
            if (AP_Notify::flags.vehicle_lost == false) {
                AP_Notify::flags.vehicle_lost = true;  // Trigger buzzer/LED notification
                gcs().send_text(MAV_SEVERITY_NOTICE,"Locate Copter alarm");  // Notify GCS
            }
            // Once activated, stays active until sticks released (else clause below)
        } else {
            // Still building up to threshold - increment counter
            soundalarm_counter++;
        }
    } else {
        // Stick pattern not detected - reset counter and deactivate alarm
        soundalarm_counter = 0;
        if (AP_Notify::flags.vehicle_lost == true) {
            AP_Notify::flags.vehicle_lost = false;  // Clear alarm state
        }
    }
}
