/**
 * @file failsafe.cpp
 * @brief Main loop watchdog failsafe system for ArduPlane
 * 
 * @details This file implements the main loop watchdog failsafe system that detects 
 *          scheduler lockup and provides RC pass-through recovery. This is a critical
 *          last-resort safety mechanism that operates independently of the main scheduler
 *          to ensure RC control is maintained even during complete software failures.
 * 
 * @author Andrew Tridgell, December 2011
 * @copyright Copyright (c) 2011-2025 ArduPilot.org
 * 
 * @warning This is a critical safety system - modifications require extensive testing
 * @warning This code runs in interrupt context and must not perform blocking operations
 */

#include "Plane.h"

/**
 * @brief Main loop watchdog failsafe implementation
 * 
 * @details Monitors scheduler health and provides emergency RC pass-through if main loop fails.
 *          This failsafe strategy detects main loop lockup by monitoring scheduler tick counts
 *          and switches to passing inputs straight from the RC inputs to RC outputs when a
 *          lockup is detected (>200ms without scheduler progress).
 * 
 *          This is the absolute last line of defense for maintaining vehicle control when
 *          the main flight control software has failed or become unresponsive.
 */

/**
 * @brief Emergency failsafe check called from timer interrupt to detect main loop lockup
 * 
 * @details Implements critical last-resort failsafe with the following behavior:
 * 
 *          **Normal Operation**:
 *          - Called at 1kHz from core timer interrupt
 *          - Monitors scheduler.ticks() to verify main loop is executing
 *          - If main loop running: Updates timestamp and exits
 * 
 *          **Lockup Detection**:
 *          - If >200ms elapsed without scheduler tick: Enters failsafe state
 *          - Possible causes: Main loop deadlock, log erase, initialization routine, infinite loop
 * 
 *          **Recovery Action** (executed every 20ms while in failsafe):
 *          1. Read latest RC inputs
 *          2. Apply expo curves to pilot inputs (roll, pitch, yaw, throttle)
 *          3. If disarmed: Force throttle to zero
 *          4. Pass RC inputs directly to servo outputs, bypassing all flight control
 *          5. Clear any RC overrides
 *          6. Update flaperons
 *          7. Output servos immediately
 * 
 *          **Advanced Failsafe Integration** (AP_ADVANCEDFAILSAFE_ENABLED):
 *          - Sends heartbeat during sensor calibration to prevent false trigger
 *          - Checks should_crash_vehicle() for OBC (Outback Challenge) rules compliance
 *          - Can deliberately terminate vehicle if extreme safety violation detected
 * 
 *          **SITL Support**:
 *          - Sends SERVO_OUTPUT_RAW MAVLink message for verification in simulation
 * 
 * @warning This is the absolute last line of defense - runs even if main loop is completely locked
 * @warning Only basic RC pass-through provided - no stabilization, no navigation, no safety checks
 * @warning 200ms threshold chosen to avoid false triggers during normal operations (log writes, param saves)
 * 
 * @note Requires at least 5 valid RC channels to provide pass-through
 * @note All secondary functions (flaps, auto functions) are disabled in failsafe
 * @note This function is interrupt-safe and must not call blocking operations
 * @note Called from timer ISR, not from main scheduler thread
 * 
 * @see Plane::roll_in_expo() For RC input expo curve application
 * @see Plane::pitch_in_expo() For RC input expo curve application
 * @see Plane::rudder_in_expo() For RC input expo curve application
 * @see Plane::get_throttle_input() For throttle processing
 * @see Plane::servos_output() For servo output handling
 * 
 * Source: ArduPlane/failsafe.cpp:17-115
 */
void Plane::failsafe_check(void)
{
    static uint16_t last_ticks;
    static uint32_t last_timestamp;
    static bool in_failsafe;
    uint32_t tnow = micros();

    const uint16_t ticks = scheduler.ticks();
    if (ticks != last_ticks) {
        // the main loop is running, all is OK
        last_ticks = ticks;
        last_timestamp = tnow;
        in_failsafe = false;
        return;
    }

    if (tnow - last_timestamp > 200000) {
        // we have gone at least 0.2 seconds since the main loop
        // ran. That means we're in trouble, or perhaps are in
        // an initialisation routine or log erase. Start passing RC
        // inputs through to outputs
        in_failsafe = true;
    }

    if (in_failsafe && tnow - last_timestamp > 20000) {

        // ensure we have the latest RC inputs
        rc().read_input();

        last_timestamp = tnow;

        rc().read_input();

#if AP_ADVANCEDFAILSAFE_ENABLED
        if (in_calibration) {
            // tell the failsafe system that we are calibrating
            // sensors, so don't trigger failsafe
            afs.heartbeat();
        }
#endif

        if (RC_Channels::get_valid_channel_count() < 5) {
            // we don't have any RC input to pass through
            return;
        }

        // pass RC inputs to outputs every 20ms
        RC_Channels::clear_overrides();

        float roll = roll_in_expo(false);
        float pitch = pitch_in_expo(false);
        float throttle = get_throttle_input(true);
        float rudder = rudder_in_expo(false);

        if (!arming.is_armed_and_safety_off()) {
            throttle = 0;
        }
        
        // setup secondary output channels that don't have
        // corresponding input channels
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, roll);
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, pitch);
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, rudder);
        SRV_Channels::set_output_scaled(SRV_Channel::k_steering, rudder);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);

        // this is to allow the failsafe module to deliberately crash 
        // the plane. Only used in extreme circumstances to meet the
        // OBC rules
#if AP_ADVANCEDFAILSAFE_ENABLED
        if (afs.should_crash_vehicle()) {
            afs.terminate_vehicle();
            if (!afs.terminating_vehicle_via_landing()) {
                return;
            }
        }
#endif

        // setup secondary output channels that do have
        // corresponding input channels
        SRV_Channels::copy_radio_in_out(SRV_Channel::k_manual, true);
        SRV_Channels::set_output_scaled(SRV_Channel::k_flap, 0.0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_flap_auto, 0.0);

        // setup flaperons
        flaperon_update();

        servos_output();

        // in SITL we send through the servo outputs so we can verify
        // we're manipulating surfaces
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        GCS_MAVLINK *chan = gcs().chan(0);
        if (HAVE_PAYLOAD_SPACE(chan->get_chan(), SERVO_OUTPUT_RAW)) {
            chan->send_servo_output_raw();
        }
#endif
    }
}
