/**
 * @file mode_training.cpp
 * @brief TRAINING flight mode implementation for ArduPlane
 * 
 * @details TRAINING mode is a flight mode designed as a learning aid for new pilots
 *          transitioning from manual flight to autonomous control. It allows full
 *          manual control under normal conditions but automatically intervenes with
 *          stabilization when the aircraft exceeds configured bank angle or pitch limits.
 * 
 *          This mode provides a safety net for pilot training by:
 *          - Allowing manual roll/pitch control within safe limits
 *          - Automatically stabilizing when bank angle exceeds ROLL_LIMIT_DEG
 *          - Automatically stabilizing when pitch exceeds configured min/max limits
 *          - Maintaining full manual rudder control at all times
 *          - Maintaining manual throttle control
 * 
 *          The mode monitors aircraft attitude continuously and seamlessly transitions
 *          between manual pass-through and automatic stabilization to prevent
 *          excessive attitudes while allowing the pilot to learn aircraft handling.
 * 
 *          Typical use case: New pilots learning fixed-wing flight characteristics
 *          with automatic protection against stalls, spins, and excessive bank angles.
 * 
 * @note This mode is vehicle-specific to ArduPlane (fixed-wing aircraft)
 * @warning Proper configuration of ROLL_LIMIT_DEG, PTCH_LIM_MAX_DEG, and PTCH_LIM_MIN_DEG
 *          is critical for safe training mode operation
 * 
 * @see ModeTraining class definition in mode.h
 * @see ArduPlane parameter documentation for ROLL_LIMIT_DEG, PTCH_LIM_MAX_DEG, PTCH_LIM_MIN_DEG
 * 
 * Source: ArduPlane/mode_training.cpp
 */

#include "mode.h"
#include "Plane.h"

/**
 * @brief Update navigation targets based on current aircraft attitude for TRAINING mode
 * 
 * @details This function monitors the aircraft's current roll and pitch angles against
 *          configured limits and determines whether manual control should be allowed or
 *          automatic stabilization should be engaged.
 * 
 *          Roll monitoring logic:
 *          - If roll angle exceeds +ROLL_LIMIT_DEG: Set nav_roll target to limit (stabilize)
 *          - If roll angle exceeds -ROLL_LIMIT_DEG: Set nav_roll target to -limit (stabilize)
 *          - If roll within limits: Allow full manual control (nav_roll = 0)
 * 
 *          Pitch monitoring logic:
 *          - If pitch exceeds PTCH_LIM_MAX_DEG: Set nav_pitch target to max limit
 *          - If pitch below PTCH_LIM_MIN_DEG: Set nav_pitch target to min limit
 *          - If pitch within limits: Allow full manual control (nav_pitch = 0)
 * 
 *          The training_manual_roll and training_manual_pitch flags are set to indicate
 *          when manual control is permitted, which is used by run() to determine control
 *          surface outputs.
 * 
 *          Special handling for inverted flight: Pitch targets are inverted when
 *          fly_inverted() returns true based on FLIGHT_OPTIONS parameter.
 * 
 * @note Called by the mode scheduler at the navigation update rate (typically 10-50 Hz)
 * @note Attitude values from AHRS are in centidegrees (e.g., 4500 = 45 degrees)
 * 
 * @warning This function sets global navigation targets (nav_roll_cd, nav_pitch_cd) that
 *          affect the stabilization controllers. Manual control flags must be properly
 *          synchronized with the run() method.
 * 
 * @see ModeTraining::run() for control surface output logic
 * @see Plane::stabilize_roll() and Plane::stabilize_pitch() for stabilization implementation
 * 
 * Source: ArduPlane/mode_training.cpp:4-34
 */
void ModeTraining::update()
{
    plane.training_manual_roll = false;
    plane.training_manual_pitch = false;
    plane.update_load_factor();

    // if the roll is past the set roll limit, then
    // we set target roll to the limit
    if (ahrs.roll_sensor >= plane.roll_limit_cd) {
        plane.nav_roll_cd = plane.roll_limit_cd;
    } else if (ahrs.roll_sensor <= -plane.roll_limit_cd) {
        plane.nav_roll_cd = -plane.roll_limit_cd;
    } else {
        plane.training_manual_roll = true;
        plane.nav_roll_cd = 0;
    }

    // if the pitch is past the set pitch limits, then
    // we set target pitch to the limit
    if (ahrs.pitch_sensor >= plane.aparm.pitch_limit_max*100) {
        plane.nav_pitch_cd = plane.aparm.pitch_limit_max*100;
    } else if (ahrs.pitch_sensor <= plane.pitch_limit_min*100) {
        plane.nav_pitch_cd = plane.pitch_limit_min*100;
    } else {
        plane.training_manual_pitch = true;
        plane.nav_pitch_cd = 0;
    }
    if (plane.fly_inverted()) {
        plane.nav_pitch_cd = -plane.nav_pitch_cd;
    }
}

/**
 * @brief Control surface output function for TRAINING mode with automatic stabilization override
 * 
 * @details This is a specialized stabilization function that implements the core behavior
 *          of TRAINING mode: seamless blending between manual pilot control and automatic
 *          stabilization based on attitude limits determined by update().
 * 
 *          Roll control logic:
 *          - If training_manual_roll is true (within limits): Direct pass-through of pilot
 *            aileron input with exponential curve applied
 *          - If training_manual_roll is false (exceeding limits): Automatic stabilization
 *            via stabilize_roll(), BUT allows pilot to override if they are commanding
 *            back toward level flight (prevents fighting the pilot)
 * 
 *          Pitch control logic:
 *          - If training_manual_pitch is true (within limits): Direct pass-through of pilot
 *            elevator input with exponential curve applied
 *          - If training_manual_pitch is false (exceeding limits): Automatic stabilization
 *            via stabilize_pitch(), BUT allows pilot to override if commanding back to level
 * 
 *          The "allow user to get out" logic compares pilot stick input against the
 *          stabilization command: if the pilot is pushing in a direction that reduces
 *          the attitude error, their input takes precedence over stabilization. This
 *          creates smooth, non-fighting behavior when the pilot attempts recovery.
 * 
 *          Rudder control:
 *          - Always manual pass-through with exponential curve (no automatic intervention)
 * 
 *          Throttle control:
 *          - Always manual via output_pilot_throttle()
 * 
 * @note Called at the main loop rate (typically 50-400 Hz depending on scheduler configuration)
 * @note Expo curves (roll_in_expo, pitch_in_expo, rudder_in_expo) apply configured exponential
 *       response curves to raw pilot input for smoother control feel
 * @note Control outputs are in PWM range normalized to ±4500 (representing ±45° equivalent)
 * 
 * @warning This function directly sets servo outputs via SRV_Channels::set_output_scaled(),
 *          bypassing normal mode control allocation. The stabilization override logic must
 *          be carefully maintained to prevent control conflicts or oscillations.
 * 
 * @warning The comparison logic for "allow user to get out" relies on correct sign conventions:
 *          positive nav_roll/pitch corresponds to right roll and nose-up pitch in NED frame.
 *          Changes to coordinate frame conventions could break this safety feature.
 * 
 * @see ModeTraining::update() for attitude monitoring and limit detection
 * @see Plane::stabilize_roll() for roll stabilization controller
 * @see Plane::stabilize_pitch() for pitch stabilization controller
 * @see Plane::roll_in_expo() for pilot roll input processing with exponential curve
 * @see Plane::pitch_in_expo() for pilot pitch input processing with exponential curve
 * @see SRV_Channels::set_output_scaled() for servo output interface
 * 
 * Source: ArduPlane/mode_training.cpp:39-71
 */
void ModeTraining::run()
{
    const float rexpo = plane.roll_in_expo(false);
    const float pexpo = plane.pitch_in_expo(false);
    if (plane.training_manual_roll) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, rexpo);
    } else {
        // calculate what is needed to hold
        plane.stabilize_roll();
        if ((plane.nav_roll_cd > 0 && rexpo < SRV_Channels::get_output_scaled(SRV_Channel::k_aileron)) ||
            (plane.nav_roll_cd < 0 && rexpo > SRV_Channels::get_output_scaled(SRV_Channel::k_aileron))) {
            // allow user to get out of the roll
            SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, rexpo);
        }
    }

    if (plane.training_manual_pitch) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, pexpo);
    } else {
        plane.stabilize_pitch();
        if ((plane.nav_pitch_cd > 0 && pexpo < SRV_Channels::get_output_scaled(SRV_Channel::k_elevator)) ||
            (plane.nav_pitch_cd < 0 && pexpo > SRV_Channels::get_output_scaled(SRV_Channel::k_elevator))) {
            // allow user to get back to level
            SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, pexpo);
        }
    }

    // Always manual rudder control
    output_rudder_and_steering(plane.rudder_in_expo(false));

    output_pilot_throttle();

}
