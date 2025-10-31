/**
 * @file mode_flip.cpp
 * @brief Flip flight mode implementation for aerobatic maneuvers
 * 
 * @details This file implements the Flip mode, which performs automated front/back/left/right
 *          flips based on pilot roll/pitch input. The mode executes a multi-stage flip sequence:
 *          1. Detect pilot input direction (roll or pitch stick position)
 *          2. Increase throttle and begin rotation at 400 deg/sec
 *          3. Apply continuous rotation rate through the flip
 *          4. Recover to original level attitude using earth-frame attitude control
 * 
 *          Original implementation by Jose Julio (2010)
 *          Adapted and updated for AC2 by Jason Short (2011)
 * 
 * @section flip_controls Control Configuration
 *          - RC7_OPTION through RC12_OPTION parameter must be set to "Flip" (AUXSW_FLIP = 2)
 *          - Pilot must be in Stabilize, Acro, or AltHold mode before triggering flip
 *          - Activating the configured aux switch initiates the flip sequence
 *          - Flip direction determined by pilot stick position at activation:
 *            * Neutral or right stick: Roll right
 *            * Left stick: Roll left  
 *            * Forward stick: Pitch forward flip
 *            * Back stick: Pitch back flip
 *          - Flip completes within 2.5 seconds and returns to original flight mode
 *          - Manual exit: Switch off aux switch or move roll/pitch stick >40 degrees
 * 
 * @section flip_state_machine State Machine Sequence
 *          - FlipState::Start (angle < 45°): Apply 400 deg/sec rotation rate, increase throttle
 *          - FlipState::Roll (45° to -90° roll): Continue 400 deg/sec roll rate, decrease throttle
 *          - FlipState::Pitch_A (45° to inverted pitch): Continue 400 deg/sec pitch rate, decrease throttle
 *          - FlipState::Pitch_B (inverted to -45° pitch): Continue 400 deg/sec pitch rate, decrease throttle
 *          - FlipState::Recover (-90° to level): Earth-frame attitude control to original attitude, increase throttle
 *          - FlipState::Abandon (timeout or pilot override): Return to original flight mode immediately
 * 
 * @warning Flip mode is intended for EXPERIENCED PILOTS ONLY with properly tuned vehicles.
 *          Improper use can result in loss of control and vehicle crash.
 * 
 * @warning Flip mode requires sufficient altitude (recommended minimum 10m above ground).
 *          Insufficient altitude will result in ground impact during the maneuver.
 * 
 * @warning Vehicle must have adequate thrust margin. Throttle should be at or above mid-stick
 *          when flip is initiated to ensure sufficient power throughout the maneuver.
 * 
 * @warning Flip mode temporarily commands very high rotation rates and throttle changes.
 *          Vehicle must be properly tuned in ACRO mode before attempting flips.
 * 
 * @note Flight controller will log FLIP_START and FLIP_END events for post-flight analysis.
 * @note If flip is abandoned or fails, FLIP_ABANDONED error is logged.
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include "Copter.h"

#if MODE_FLIP_ENABLED

/**
 * @name Flip Mode Throttle Adjustments
 * @{
 */

/** 
 * @brief Throttle increase during FlipState::Start stage (angle < 45 degrees)
 * @details Adds 0.20 (20%) to pilot's throttle input to build energy and prevent altitude loss
 *          during initial rotation phase. Applied when vehicle lean angle is under 45 degrees.
 */
#define FLIP_THR_INC            0.20f

/** 
 * @brief Throttle decrease during rotation stages (angle between 45° and -90°)
 * @details Reduces throttle by 0.24 (24%) during FlipState::Roll, FlipState::Pitch_A, and
 *          FlipState::Pitch_B stages to prevent excessive altitude gain while inverted and
 *          to help complete the rotation. Larger decrease than increase to account for
 *          reduced effective thrust when not upright.
 */
#define FLIP_THR_DEC            0.24f

/** @} */ // End of throttle adjustments group

/**
 * @name Flip Mode Rotation Parameters
 * @{
 */

/** 
 * @brief Target rotation rate during flip maneuver
 * @details Commands 400 degrees/second rotation rate about the flip axis (roll or pitch).
 *          This aggressive rate is necessary to complete the flip within the timeout period
 *          and maintain sufficient centripetal effect. Rate is applied in body frame.
 * @note Value is in radians/second for use with attitude controller rate commands
 */
#define FLIP_ROTATION_RATE_RADS radians(400.0)

/** 
 * @brief Maximum time allowed for flip completion
 * @details Flip mode will automatically abandon and return to original flight mode after
 *          2500 milliseconds (2.5 seconds) if the flip has not completed. This timeout
 *          prevents the vehicle from remaining in flip mode indefinitely if the maneuver
 *          fails or pilot loses orientation.
 * @warning Timeout triggers FlipState::Abandon which immediately returns to original mode
 */
#define FLIP_TIMEOUT_MS         2500

/** 
 * @brief Angular threshold for successful flip recovery
 * @details Flip is considered successfully recovered when vehicle attitude is within
 *          5 degrees of the original pre-flip attitude. Once within this threshold,
 *          the mode automatically exits and returns to the original flight mode.
 * @note Measured in radians for comparison with AHRS attitude estimates
 */
#define FLIP_RECOVERY_ANGLE_RAD radians(5.0)

/** @} */ // End of rotation parameters group

/**
 * @name Flip Direction Constants
 * @{
 */

/** 
 * @brief Direction multiplier for right roll flip
 * @details Positive direction for roll axis flip (roll to the right when viewed from behind)
 */
#define FLIP_ROLL_RIGHT      1

/** 
 * @brief Direction multiplier for left roll flip
 * @details Negative direction for roll axis flip (roll to the left when viewed from behind)
 */
#define FLIP_ROLL_LEFT      -1

/** 
 * @brief Direction multiplier for backward pitch flip
 * @details Positive direction for pitch axis flip (pitch backward/tail down when viewed from side)
 */
#define FLIP_PITCH_BACK      1

/** 
 * @brief Direction multiplier for forward pitch flip
 * @details Negative direction for pitch axis flip (pitch forward/nose down when viewed from side)
 */
#define FLIP_PITCH_FORWARD  -1

/** @} */ // End of direction constants group

/**
 * @brief Initialize flip controller and perform pre-flip safety checks
 * 
 * @details This function performs comprehensive safety validation before allowing the flip
 *          maneuver to begin. It checks that:
 *          - Current flight mode supports flip transitions (ACRO, Stabilize, AltHold, FlowHold)
 *          - Throttle is above zero (for modes that report throttle state)
 *          - Roll stick input is less than 40 degrees (to prevent conflicting manual input)
 *          - Vehicle is armed and airborne (not landed)
 * 
 *          If all checks pass, the function:
 *          - Captures the original flight mode for restoration after flip completion
 *          - Initializes state machine to FlipState::Start
 *          - Records start time for timeout monitoring
 *          - Determines flip direction (roll or pitch, positive or negative) based on
 *            pilot stick position at the moment of flip activation
 *          - Captures current vehicle attitude as recovery target
 *          - Logs FLIP_START event to dataflash
 * 
 * @param[in] ignore_checks  Not used in flip mode (safety checks are always enforced)
 * 
 * @return true if all safety checks pass and flip mode successfully initialized
 * @return false if any safety check fails or flip is not allowed from current state
 * 
 * @note Flip direction priority: Pitch stick > 300 = back flip, < -300 = forward flip,
 *       otherwise roll stick >= 0 = right roll, < 0 = left roll
 * 
 * @warning This function will reject flip attempts if throttle is at zero in ACRO or
 *          Stabilize modes, as insufficient power is available to complete the maneuver safely.
 * 
 * @warning Roll stick position must be within ±40 degrees to prevent pilot input from
 *          interfering with the automated flip sequence.
 * 
 * @see ModeFlip::run() for flip execution state machine
 * @see FlipState enum for state machine stages
 */
bool ModeFlip::init(bool ignore_checks)
{
    // Safety Check 1: Verify current flight mode supports flip transitions
    // Only modes with rate or attitude control are suitable (ACRO, Stabilize, AltHold, FlowHold)
    if (!copter.flightmode->allows_flip()) {
        return false;
    }

    // Safety Check 2: Ensure sufficient throttle for maneuver completion
    // In ACRO or Stabilize modes, throttle_zero flag indicates insufficient power available
    // to safely complete the flip and recover altitude
    if (copter.ap.throttle_zero && (copter.flightmode->mode_number() == Mode::Number::ACRO || copter.flightmode->mode_number() == Mode::Number::STABILIZE)) {
        return false;
    }

    // Safety Check 3: Verify pilot is not providing large roll input
    // Roll input >= 4000 centidegrees (40 degrees) indicates pilot may be trying to manually
    // control the vehicle, which would conflict with the automated flip sequence
    if (abs(channel_roll->get_control_in()) >= 4000) {
        return false;
    }

    // Safety Check 4: Confirm vehicle is armed and airborne
    // Flip cannot be initiated on the ground or when disarmed
    if (!motors->armed() || copter.ap.land_complete) {
        return false;
    }

    // All safety checks passed - begin flip initialization

    // Store original flight mode for restoration after flip completion
    // This allows seamless return to the mode the pilot was flying before the flip
    orig_control_mode = copter.flightmode->mode_number();

    // Initialize state machine to start stage
    _state = FlipState::Start;
    start_time_ms = millis();  // Record start time for timeout monitoring

    // Reset direction flags - will be set based on pilot stick position
    roll_dir = pitch_dir = 0;

    // Determine flip direction from pilot stick position at moment of activation
    // Priority: Pitch input takes precedence over roll input
    // Pitch stick > 300 centidegrees (3 degrees): Backward flip
    if (channel_pitch->get_control_in() > 300) {
        pitch_dir = FLIP_PITCH_BACK;
    } 
    // Pitch stick < -300 centidegrees (-3 degrees): Forward flip
    else if (channel_pitch->get_control_in() < -300) {
        pitch_dir = FLIP_PITCH_FORWARD;
    } 
    // If pitch is near neutral, use roll stick direction
    // Roll stick >= 0: Right roll (default if both sticks centered)
    else if (channel_roll->get_control_in() >= 0) {
        roll_dir = FLIP_ROLL_RIGHT;
    } 
    // Roll stick < 0: Left roll
    else {
        roll_dir = FLIP_ROLL_LEFT;
    }

    // Log flip initiation to dataflash for post-flight analysis
    LOGGER_WRITE_EVENT(LogEvent::FLIP_START);

    // Capture current vehicle attitude as the recovery target
    // After completing the flip rotation, the controller will return to this attitude
    // Constrain captured angles to maximum lean angle limits to ensure valid recovery target
    const float angle_max_rad = attitude_control->lean_angle_max_rad();
    orig_attitude_euler_rad.x = constrain_float(ahrs.get_roll_rad(), -angle_max_rad, angle_max_rad);
    orig_attitude_euler_rad.y = constrain_float(ahrs.get_pitch_rad(), -angle_max_rad, angle_max_rad);
    orig_attitude_euler_rad.z = ahrs.get_yaw_rad();  // Yaw can be full range

    return true;  // Flip mode successfully initialized
}

/**
 * @brief Execute flip controller state machine
 * 
 * @details This function implements the main flip execution state machine, which is called
 *          continuously while flip mode is active. The state machine progresses through
 *          multiple stages to complete the aerobatic flip maneuver:
 * 
 *          State Machine Flow:
 *          1. FlipState::Start - Initial rotation phase (0° to 45° lean)
 *             - Commands 400 deg/sec rotation rate on flip axis
 *             - Increases throttle by FLIP_THR_INC to build energy
 *             - Transitions to Roll/Pitch_A state when lean angle exceeds 45°
 * 
 *          2. FlipState::Roll - Roll flip continuation (45° to -90° roll)
 *             - Continues 400 deg/sec roll rate on roll axis
 *             - Decreases throttle by FLIP_THR_DEC (less efficient thrust when inverted)
 *             - Transitions to Recover when angle crosses through -90° back toward -45°
 * 
 *          3. FlipState::Pitch_A - Pitch flip first half (45° to inverted)
 *             - Continues 400 deg/sec pitch rate
 *             - Decreases throttle by FLIP_THR_DEC
 *             - Monitors roll angle for inversion detection (roll > 90°)
 *             - Transitions to Pitch_B when inverted and pitch > 45°
 * 
 *          4. FlipState::Pitch_B - Pitch flip second half (inverted to -45°)
 *             - Continues 400 deg/sec pitch rate
 *             - Decreases throttle by FLIP_THR_DEC
 *             - Monitors roll angle for upright transition (roll < 90°)
 *             - Transitions to Recover when upright and pitch > -45°
 * 
 *          5. FlipState::Recover - Return to level flight
 *             - Uses earth-frame attitude control to return to original pre-flip attitude
 *             - Increases throttle by FLIP_THR_INC to regain lost altitude
 *             - Automatically exits flip mode when within FLIP_RECOVERY_ANGLE_RAD of target
 *             - Returns to original flight mode that was active before flip
 * 
 *          6. FlipState::Abandon - Emergency exit
 *             - Triggered by timeout, large pilot input, or disarm
 *             - Immediately returns to original flight mode
 *             - Logs FLIP_ABANDONED error to dataflash
 * 
 *          Abandon Conditions:
 *          - Motors disarmed during flip
 *          - Roll or pitch stick moved > 40 degrees
 *          - Timeout exceeded (FLIP_TIMEOUT_MS = 2.5 seconds)
 * 
 * @note This function must be called at 100 Hz or higher for smooth flip execution and
 *       proper state machine timing. Lower rates may result in jerky motion or incomplete flips.
 * 
 * @note Throttle adjustments are applied as offsets to pilot's throttle input, so pilot
 *       maintains some control authority throughout the maneuver.
 * 
 * @warning Motors are set to THROTTLE_UNLIMITED spool state during flip to allow full
 *          throttle range including very low values. This can result in motor shutdown if
 *          throttle decreases are too aggressive.
 * 
 * @warning Flip abort (via pilot stick input > 40° or timeout) returns to original mode
 *          immediately without completing recovery to level attitude. Vehicle may still be
 *          inverted or at extreme angles.
 * 
 * @see ModeFlip::init() for flip initialization and direction determination
 * @see FlipState enum for state machine stage definitions
 * @see attitude_control->input_rate_bf_roll_pitch_yaw_rads() for body-frame rate control
 * @see attitude_control->input_euler_angle_roll_pitch_yaw_rad() for earth-frame attitude control
 */
void ModeFlip::run()
{
    // Check abandon conditions - any of these will immediately exit flip mode
    // 1. Motors disarmed (safety system or pilot action)
    // 2. Large roll stick input (>40 degrees = 4000 centidegrees) - pilot override
    // 3. Large pitch stick input (>40 degrees) - pilot override  
    // 4. Timeout exceeded (flip should complete within 2.5 seconds)
    if (!motors->armed() || (abs(channel_roll->get_control_in()) >= 4000) || (abs(channel_pitch->get_control_in()) >= 4000) || ((millis() - start_time_ms) > FLIP_TIMEOUT_MS)) {
        _state = FlipState::Abandon;
    }

    // Get pilot's current throttle input as baseline
    // Flip state machine will add or subtract from this value
    float throttle_out = get_pilot_desired_throttle();

    // Enable full throttle range for flip maneuver
    // THROTTLE_UNLIMITED allows throttle to go to very low values during inverted phase
    // and full throttle during recovery without artificial limiting
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // Calculate current flip angle normalized to flip direction
    // This allows the same state machine logic to work for all four flip directions
    // flip_angle_rad will be positive when rotating in the flip direction
    float flip_angle_rad;

    if (roll_dir != 0) {
        // Roll flip: Multiply current roll angle by direction (+1 or -1)
        // Positive flip_angle_rad indicates rotation in the intended direction
        flip_angle_rad = ahrs.get_roll_rad() * roll_dir;
    } else {
        // Pitch flip: Multiply current pitch angle by direction (+1 or -1)
        // Positive flip_angle_rad indicates rotation in the intended direction
        flip_angle_rad = ahrs.get_pitch_rad() * pitch_dir;
    }

    // Execute state machine based on current flip state
    // Each state commands specific rotation rates and throttle adjustments
    switch (_state) {

    case FlipState::Start:
        // Initial flip stage: 0° to 45° lean angle
        // Command aggressive rotation rate (400 deg/sec) on the flip axis to begin the maneuver
        // Roll axis: roll_dir is ±1, pitch_dir is 0 → only roll rate commanded
        // Pitch axis: pitch_dir is ±1, roll_dir is 0 → only pitch rate commanded
        attitude_control->input_rate_bf_roll_pitch_yaw_rads(FLIP_ROTATION_RATE_RADS * roll_dir, FLIP_ROTATION_RATE_RADS * pitch_dir, 0.0);

        // Increase throttle to build energy and prevent altitude loss during initial rotation
        // FLIP_THR_INC = 0.20 adds 20% to pilot's throttle input
        throttle_out += FLIP_THR_INC;

        // Transition to next stage when vehicle passes 45° lean angle
        // At this point vehicle has sufficient angular momentum to continue through the flip
        if (flip_angle_rad >= radians(45.0)) {
            if (roll_dir != 0) {
                // Roll flip: Continue to Roll state
                _state = FlipState::Roll;
            } else {
                // Pitch flip: Continue to Pitch_A state (first half of pitch flip)
                _state = FlipState::Pitch_A;
            }
        }
        break;

    case FlipState::Roll:
        // Roll flip stage: 45° through inverted (180°) toward -90°
        // Continue commanding 400 deg/sec roll rate to maintain flip momentum
        // This carries the vehicle through the inverted portion of the roll
        attitude_control->input_rate_bf_roll_pitch_yaw_rads(FLIP_ROTATION_RATE_RADS * roll_dir, 0.0, 0.0);
        
        // Decrease throttle during inverted phase
        // When inverted, thrust points downward, so reduce throttle to prevent excessive altitude gain
        // and to help the vehicle complete the rotation. FLIP_THR_DEC = 0.24 reduces throttle by 24%.
        // Ensure throttle doesn't go negative (constrain to 0.0 minimum)
        throttle_out = MAX(throttle_out - FLIP_THR_DEC, 0.0f);

        // Transition to recovery when:
        // 1. flip_angle_rad < 45° - vehicle has rotated past 135° (coming back toward upright)
        // 2. flip_angle_rad > -90° - haven't over-rotated past the recovery point
        // This represents the vehicle passing through inverted and approaching upright again
        if ((flip_angle_rad < radians(45.0)) && (flip_angle_rad > -radians(90.0))) {
            _state = FlipState::Recover;
        }
        break;

    case FlipState::Pitch_A:
        // Pitch flip first half: 45° pitch toward inverted
        // Continue commanding 400 deg/sec pitch rate on pitch axis
        // During a pitch flip, the vehicle pitches nose-down (forward flip) or tail-down (back flip)
        attitude_control->input_rate_bf_roll_pitch_yaw_rads(0.0f, FLIP_ROTATION_RATE_RADS * pitch_dir, 0.0);
        
        // Decrease throttle during the rotation to prevent altitude gain
        // Vehicle is pitching through increasing angles toward inverted
        throttle_out = MAX(throttle_out - FLIP_THR_DEC, 0.0f);

        // Transition to Pitch_B (second half) when vehicle becomes inverted
        // Check for inversion by monitoring roll angle: abs(roll) > 90° means inverted
        // Also verify pitch angle is still increasing (flip_angle_rad > 45°)
        // Pitch flips cause the vehicle to roll past 90° during the inverted portion
        if ((fabsf(ahrs.get_roll_rad()) > radians(90.0)) && (flip_angle_rad > radians(45.0))) {
            _state = FlipState::Pitch_B;
        }
        break;

    case FlipState::Pitch_B:
        // Pitch flip second half: Inverted to upright (-90° to recovery)
        // Continue commanding 400 deg/sec pitch rate to complete the flip rotation
        // Vehicle continues pitching through inverted back toward upright
        attitude_control->input_rate_bf_roll_pitch_yaw_rads(0.0, FLIP_ROTATION_RATE_RADS * pitch_dir, 0.0);
        
        // Continue decreasing throttle while inverted and rotating
        // Thrust is pointing downward during inversion, so reduced throttle prevents altitude gain
        throttle_out = MAX(throttle_out - FLIP_THR_DEC, 0.0f);

        // Transition to recovery when vehicle returns to upright orientation
        // Check for upright by monitoring roll angle: abs(roll) < 90° means right-side up
        // Also verify pitch angle is approaching level (flip_angle_rad > -45°)
        // When both conditions met, vehicle has completed the full pitch rotation
        if ((fabsf(ahrs.get_roll_rad()) < radians(90.0)) && (flip_angle_rad > -radians(45.0))) {
            _state = FlipState::Recover;
        }
        break;

    case FlipState::Recover: {
        // Recovery stage: Return vehicle to original pre-flip attitude
        // Switch from body-frame rate control to earth-frame attitude control
        // Uses the attitude captured in init() as the target for recovery
        // This smoothly brings the vehicle back to level flight (or original lean angle)
        attitude_control->input_euler_angle_roll_pitch_yaw_rad(orig_attitude_euler_rad.x, orig_attitude_euler_rad.y, orig_attitude_euler_rad.z, false);

        // Increase throttle to regain any altitude lost during the flip maneuver
        // The vehicle typically loses some altitude during rotation and inverted flight
        // FLIP_THR_INC = 0.20 adds 20% to pilot's throttle input
        throttle_out += FLIP_THR_INC;

        // Calculate angular error between current attitude and recovery target
        // Only check the axis that was flipped (roll or pitch)
        float recovery_angle_rad;
        if (roll_dir != 0) {
            // Roll flip: Check roll axis error
            // Difference between target roll and current roll indicates recovery progress
            recovery_angle_rad = fabsf(orig_attitude_euler_rad.x - ahrs.get_roll_rad());
        } else {
            // Pitch flip: Check pitch axis error  
            // Difference between target pitch and current pitch indicates recovery progress
            recovery_angle_rad = fabsf(orig_attitude_euler_rad.y - ahrs.get_pitch_rad());
        }

        // Check if vehicle has successfully recovered to near-level attitude
        // FLIP_RECOVERY_ANGLE_RAD = 5° is the threshold for successful recovery
        if (fabsf(recovery_angle_rad) <= FLIP_RECOVERY_ANGLE_RAD) {
            // Recovery complete - return to the flight mode that was active before flip
            // This seamlessly returns control to the pilot in their original mode
            if (!copter.set_mode(orig_control_mode, ModeReason::FLIP_COMPLETE)) {
                // Fallback to Stabilize if mode switch fails (should never happen)
                // Stabilize is safe default as it provides attitude stabilization
                copter.set_mode(Mode::Number::STABILIZE, ModeReason::UNKNOWN);
            }
            // Log successful flip completion to dataflash for post-flight analysis
            LOGGER_WRITE_EVENT(LogEvent::FLIP_END);
        }
        break;

    }
    case FlipState::Abandon:
        // Abandon state: Emergency exit from flip mode
        // Triggered by: timeout, large pilot input (>40°), or motor disarm
        // Immediately returns to original flight mode without completing recovery
        // WARNING: Vehicle may still be at extreme angles or inverted when this occurs
        
        // Attempt to restore original flight mode
        if (!copter.set_mode(orig_control_mode, ModeReason::FLIP_COMPLETE)) {
            // Fallback to Stabilize if mode switch fails
            // Stabilize provides basic attitude control to help pilot regain control
            copter.set_mode(Mode::Number::STABILIZE, ModeReason::UNKNOWN);
        }
        
        // Log flip abandonment as an error for post-flight analysis
        // Indicates flip did not complete successfully - investigate cause
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FLIP, LogErrorCode::FLIP_ABANDONED);
        break;
    }

    // Apply final throttle output to motors
    // throttle_out has been adjusted by state machine (increased or decreased)
    // false = do not apply angle boost (flip manages throttle explicitly)
    // g.throttle_filt = apply configured throttle filtering for smooth motor response
    attitude_control->set_throttle_out(throttle_out, false, g.throttle_filt);
}

#endif
