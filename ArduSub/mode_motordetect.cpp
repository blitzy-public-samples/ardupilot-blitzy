#include "Sub.h"
#include "stdio.h"

/**
 * @file mode_motordetect.cpp
 * @brief Motor Detection mode implementation for ArduSub
 * 
 * @details This file implements the Motor Detection flight mode which automatically
 *          detects the correct orientation (normal vs reversed) for each thruster
 *          by testing them individually and analyzing the vehicle's gyroscopic response.
 *          
 *          The detection algorithm pulses each thruster sequentially and compares the
 *          measured angular rates against the expected motor angular factors from the
 *          frame configuration. This allows automatic identification of reversed motor
 *          connections without manual testing.
 *          
 *          **Algorithm Overview:**
 *          For each enabled motor:
 *          1. Wait until vehicle is stationary for >500ms (gyro product < 0.01)
 *          2. Apply positive throttle (1800 PWM) for 500ms
 *          3. Measure gyroscopic response (roll, pitch, yaw rates)
 *          4. Compare measured response to expected motor angular factors:
 *             - If match: Motor is correctly oriented
 *             - If inverted match: Motor is reversed, save reversal flag
 *             - If no match: Try negative throttle (1200 PWM) direction
 *          5. If second direction also fails: Abort with error
 *          6. Proceed to next motor
 *          
 *          **State Machine:**
 *          STANDBY → SETTLING → THRUSTING → DETECTING → [repeat for next motor] → DONE
 *          
 * @note Prerequisites for successful motor detection:
 *       - Correct frame type must be selected (vectored, vectored6dof, etc.)
 *       - Motors must be connected to correct ESC outputs per frame definition
 *       - Vehicle must be in water with sufficient depth for free movement
 *       - Vehicle should not be tethered or constrained
 *       - Sufficient battery voltage for consistent motor response
 *       
 * @note Detection thresholds:
 *       - Settling gyro threshold: 0.01 rad²/s² (gyro vector magnitude squared)
 *       - Settling time: 500ms
 *       - Thrust duration: 500ms
 *       - Thrust PWM: 1500 ± 300 (1200 or 1800)
 *       - Roll/Pitch detection threshold: ±0.4 rad/s
 *       - Yaw detection threshold: ±0.5 rad/s
 *       
 * @warning This mode requires the vehicle to be armed and will actively thrust motors.
 *          Ensure adequate space and safety precautions before use.
 *          
 * @warning Detection may fail in shallow water, confined spaces, or with heavily
 *          loaded vehicles where motor thrust cannot produce measurable angular rates.
 *          
 * Source: ArduSub/mode_motordetect.cpp
 */

namespace {

    /**
     * @enum test_state
     * @brief State machine states for motor detection sequence
     * 
     * @details The motor detection algorithm uses a state machine to sequence
     *          through the testing of each motor. Each motor goes through
     *          multiple states before detection is complete.
     */
    enum test_state {
        STANDBY,    ///< Initial state, waiting to start detection sequence
        SETTLING,   ///< Waiting for vehicle to stabilize (gyro settling)
        THRUSTING,  ///< Actively thrusting current motor for 500ms
        DETECTING,  ///< Analyzing gyro response to determine motor orientation
        DONE        ///< All motors tested, exiting mode
    };

    /**
     * @enum direction
     * @brief Thrust direction for motor testing
     * 
     * @details Motors are tested in both directions to handle cases where
     *          initial thrust direction doesn't produce clear results.
     *          UP direction applies positive PWM offset (+300 from neutral 1500).
     *          DOWN direction applies negative PWM offset (-300 from neutral 1500).
     */
    enum direction {
        UP = 1,     ///< Positive thrust direction (1800 PWM)
        DOWN = -1   ///< Negative thrust direction (1200 PWM)
    };

    /// @brief Timer for settling state - tracks when vehicle became stationary
    static uint32_t settling_timer;
    
    /// @brief Timer for thrusting state - tracks thrust pulse duration
    static uint32_t thrusting_timer;
    
    /// @brief Current state in the detection state machine
    static uint8_t md_state;
    
    /// @brief Index of motor currently being tested (0-based)
    static uint8_t current_motor;
    
    /// @brief Current thrust direction being tested (UP or DOWN)
    static int16_t current_direction;
}

/**
 * @brief Initialize Motor Detection mode
 * 
 * @details Resets the motor detection state machine to begin testing sequence.
 *          Initializes static state variables to start testing from motor 0
 *          in the positive (UP) thrust direction.
 *          
 *          This function is called when the pilot switches into Motor Detection mode.
 *          The actual detection sequence begins in the run() method after the vehicle
 *          is armed.
 *          
 * @param[in] ignore_checks Arming checks flag (ignored in this mode)
 * 
 * @return true Always returns true - mode initialization always succeeds
 * 
 * @note The vehicle must be armed separately after entering this mode.
 *       Detection does not begin until the vehicle is armed.
 *       
 * @note State machine is initialized to STANDBY. First call to run() after
 *       arming will transition to SETTLING state to begin detection sequence.
 *       
 * @see ModeMotordetect::run()
 */
bool ModeMotordetect::init(bool ignore_checks)
{
    current_motor = 0;
    md_state = STANDBY;
    current_direction = UP;
    return true;
}

/**
 * @brief Main execution loop for Motor Detection mode
 * 
 * @details Implements the motor detection state machine that sequences through
 *          testing each enabled motor. Called at the main loop rate (typically 50Hz
 *          for ArduSub) while in Motor Detection mode.
 *          
 *          **State Machine Flow:**
 *          
 *          1. **STANDBY**: Initial state when mode first entered or after init()
 *             - Initializes motor index to 0 and direction to UP
 *             - Transitions immediately to SETTLING
 *             
 *          2. **SETTLING**: Waits for vehicle to stabilize
 *             - Forces all motors to neutral (1500 PWM)
 *             - Monitors gyro magnitude squared (gyro · gyro)
 *             - Resets timer if gyro product > 0.01 rad²/s²
 *             - Requires 500ms continuous stability before proceeding
 *             - Transitions to THRUSTING when stable
 *             
 *          3. **THRUSTING**: Actively tests current motor
 *             - Pulses current motor at 1500 ± 300 PWM (depending on direction)
 *             - Other motors remain at neutral (1500 PWM)
 *             - Continues for 500ms
 *             - Transitions to DETECTING after thrust pulse complete
 *             
 *          4. **DETECTING**: Analyzes vehicle response to determine motor orientation
 *             - Reads current gyro rates (roll, pitch, yaw)
 *             - Applies thresholds to classify angular response:
 *               * Roll: ±0.4 rad/s threshold
 *               * Pitch: ±0.4 rad/s threshold  
 *               * Yaw: ±0.5 rad/s threshold
 *             - Constructs response vector: (roll, pitch, yaw) with signs
 *             - Compares to expected motor angular factors from frame definition:
 *               * Match: Motor correctly oriented → proceed to next motor
 *               * Inverted match: Motor reversed → set reversal flag, next motor
 *               * No match: Bad reading → try opposite direction (DOWN)
 *               * Second direction also fails → ABORT to DONE state
 *             - Transitions to SETTLING for next motor or DONE if complete
 *             
 *          5. **DONE**: Detection complete or failed
 *             - Switches back to previous control mode
 *             - Disarms vehicle with MOTORDETECTDONE reason
 *             
 *          **Motor Direction Detection Algorithm:**
 *          
 *          The detection compares measured gyroscopic response to the motor's
 *          configured angular factors (contribution to roll, pitch, yaw). Each
 *          motor in the frame definition has angular factors like (0, 1, 0) for
 *          a forward motor producing pitch moment.
 *          
 *          By thrusting a motor and measuring the gyro response, the algorithm
 *          can determine:
 *          - If response matches expected: Motor wired correctly
 *          - If response is inverted: Motor reversed, needs reversal parameter set
 *          - If response doesn't match: Wrong motor connection or mechanical issue
 *          
 *          Detected reversals are automatically saved to the motor configuration
 *          using motors.set_reversed(motor_index, true).
 *          
 * @note This function requires the vehicle to be armed. If disarmed, it immediately
 *       stops all motors and returns to STANDBY state.
 *       
 * @note Detection thresholds (roll/pitch: ±0.4 rad/s, yaw: ±0.5 rad/s) are currently
 *       hardcoded. Future versions may expose these as parameters for tuning.
 *       
 * @note The algorithm tests motors sequentially by iterating through motor indices
 *       and checking motors.motor_is_enabled() for each. Disabled motors are skipped.
 *       
 * @note GCS text messages are sent at each stage:
 *       - INFO: "Thruster X is ok!" - Motor correctly oriented
 *       - INFO: "Thruster X is reversed! Saving it!" - Reversal detected and saved
 *       - INFO: "Bad thrust read, trying to push the other way..." - Retry with opposite direction
 *       - WARNING: "Failed! Please check Thruster X and frame setup!" - Detection failed
 *       - WARNING: "Motor direction detection is complete." - All motors tested successfully
 *       
 * @warning Detection can fail if:
 *          - Vehicle is in shallow water (insufficient movement freedom)
 *          - Vehicle is tethered or mechanically constrained
 *          - Motor thrust too weak to produce measurable gyro response
 *          - Wrong frame type selected (expected angular factors incorrect)
 *          - Motor connected to wrong ESC output
 *          - ESC calibration issues preventing consistent thrust
 *          
 * @warning Motor numbering in GCS messages is 1-based (current_motor + 1) for
 *          user readability, while internal indexing is 0-based.
 *          
 * Source: ArduSub/mode_motordetect.cpp:56-174
 */
void ModeMotordetect::run()
{
    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Force all motors to stop
        for(uint8_t i=0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motors.motor_is_enabled(i)) {
                motors.output_test_num(i, 1500);
            }
        }
        md_state = STANDBY;
        return;
    }

    switch(md_state) {
    /**
     * STANDBY state: Initialize detection sequence
     * 
     * Sets up initial conditions for motor detection:
     * - Reset to first motor (index 0)
     * - Set initial thrust direction to UP (positive)
     * - Start settling timer
     * - Transition immediately to SETTLING state
     * 
     * This state is entered on first run() call after arming, or after
     * disarm/rearm cycle.
     */
    case STANDBY:
        current_direction = UP;
        current_motor = 0;
        settling_timer = AP_HAL::millis();
        md_state = SETTLING;
        break;

    /**
     * SETTLING state: Wait for vehicle to stabilize before motor test
     * 
     * Forces all motors to neutral position (1500 PWM) and monitors gyroscope
     * to ensure vehicle has stopped moving. This provides a stable baseline
     * for detecting the gyro response when the next motor is pulsed.
     * 
     * Stability criteria:
     * - Gyro dot product (gyro · gyro) must be < 0.01 rad²/s²
     * - Must remain stable continuously for 500ms
     * - Timer resets if gyro threshold exceeded
     * 
     * Once stability achieved, transitions to THRUSTING state and starts
     * the thrust pulse timer.
     * 
     * @note The 0.01 rad²/s² threshold is experimentally determined and may
     *       need adjustment for different vehicle sizes or operating conditions.
     */
    case SETTLING:
        // Force all motors to stop
        for (uint8_t i=0; i <AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motors.motor_is_enabled(i)) {
                motors.output_test_num(i, 1500);
            }
        }
        // wait until gyro product is under a certain(experimental) threshold
        if ((ahrs.get_gyro()*ahrs.get_gyro()) > 0.01) {
            settling_timer = AP_HAL::millis();
        }
        // then wait 500ms more
        if (AP_HAL::millis() > (settling_timer + 500)) {
            md_state = THRUSTING;
            thrusting_timer = AP_HAL::millis();
        }

        break;

    /**
     * THRUSTING state: Pulse current motor for 500ms
     * 
     * Applies thrust to the current motor being tested while vehicle is stable.
     * PWM output is calculated as: 1500 + (300 * current_direction)
     * - UP direction: 1500 + 300 = 1800 PWM (positive thrust)
     * - DOWN direction: 1500 - 300 = 1200 PWM (negative thrust)
     * 
     * The 500ms pulse duration provides sufficient time for:
     * - Motor to spin up to consistent RPM
     * - Vehicle to develop measurable angular rates
     * - Gyroscope to capture steady-state response
     * 
     * After 500ms, transitions to DETECTING state to analyze the gyro response.
     * 
     * If output_test_num() fails (returns false), immediately transitions to
     * DONE state as motor output failure indicates a critical error.
     * 
     * @note PWM values assume standard ESC calibration (1100-1900 range).
     *       1500 is neutral, values above produce positive thrust.
     */
    case THRUSTING:
        if (AP_HAL::millis() < (thrusting_timer + 500)) {
            if (!motors.output_test_num(current_motor, 1500 + 300*current_direction)) {
                md_state = DONE;
            };

        } else {
            md_state = DETECTING;
        }
        break;

    /**
     * DETECTING state: Analyze gyro response to determine motor orientation
     * 
     * This is the core motor detection logic that determines if a motor is
     * wired correctly or reversed by comparing the measured gyroscopic response
     * to the expected angular factors from the frame configuration.
     * 
     * Algorithm steps:
     * 
     * 1. Read current gyro rates (roll, pitch, yaw) in rad/s
     * 
     * 2. Apply thresholds to classify each axis as up/down/neutral:
     *    - Roll/Pitch: ±0.4 rad/s threshold
     *    - Yaw: ±0.5 rad/s threshold (higher due to typically slower yaw response)
     *    
     * 3. Construct direction vector accounting for thrust direction:
     *    - result = (sign of measured rate) * current_direction
     *    - This normalizes both UP and DOWN thrust tests to same comparison
     *    
     * 4. Compare direction vector to motor angular factors:
     *    - Exact match: Motor correctly oriented (normal)
     *    - Inverted match: Motor reversed (set reversal flag)
     *    - No match: Bad reading
     *    
     * 5. Handle bad readings:
     *    - If current_direction == UP: Try DOWN direction
     *    - If current_direction == DOWN: Second failure, ABORT
     *    
     * 6. Handle good readings:
     *    - Send confirmation to GCS
     *    - Save reversal flag if needed
     *    - Increment to next motor
     *    - Reset direction to UP
     *    - Check if more motors to test or go to DONE
     * 
     * Motor angular factors define the expected moment contribution:
     * - Forward motor: (0, 1, 0) - produces pitch moment
     * - Lateral motor: (1, 0, 0) - produces roll moment  
     * - Vertical motor: (0, 0, 1) - produces yaw moment (slight)
     * - Vectored motors: mixed factors like (0.7, 0.7, 0) for diagonal
     * 
     * Detection can fail if:
     * - Gyro response too weak (shallow water, light vehicle)
     * - Gyro response too strong (overshoots threshold in wrong direction)
     * - Wrong frame selected (angular factors don't match hardware)
     * - Motor connected to wrong output channel
     * 
     * @note Motor indices in GCS messages are 1-based (current_motor + 1)
     *       for user-friendly display.
     *       
     * @note TODO comment in code indicates thresholds should become parameters
     *       for user tuning based on vehicle characteristics.
     */
    case DETECTING:
    {
        /**
         * Construct direction vector from gyro response:
         * 
         * For each axis (roll, pitch, yaw):
         * 1. Classify gyro rate as positive, negative, or neutral using thresholds
         * 2. Convert to direction: +1 (positive), -1 (negative), 0 (neutral)
         * 3. Multiply by current_direction to normalize thrust direction
         * 
         * Result is a direction vector like (1, -1, 0) indicating:
         * - Positive roll rate detected
         * - Negative pitch rate detected  
         * - No significant yaw rate
         * 
         * This vector is then compared to the motor's angular factors from
         * the frame configuration to determine if motor is correctly oriented.
         */
        // TODO: make these thresholds parameters
        Vector3f gyro = ahrs.get_gyro();
        
        // Roll axis detection (rad/s threshold: ±0.4)
        bool roll_up = gyro.x > 0.4;
        bool roll_down = gyro.x < -0.4;
        int roll = (int(roll_up) - int(roll_down))*current_direction;

        // Pitch axis detection (rad/s threshold: ±0.4)
        bool pitch_up = gyro.y > 0.4;
        bool pitch_down = gyro.y < -0.4;
        int pitch = (int(pitch_up) - int(pitch_down))*current_direction;

        // Yaw axis detection (rad/s threshold: ±0.5, higher due to slower yaw dynamics)
        bool yaw_up = gyro.z > 0.5;
        bool yaw_down = gyro.z < -0.5;
        int yaw = (+int(yaw_up) - int(yaw_down))*current_direction;

        // Construct direction vector from classified gyro response
        Vector3f directions(roll, pitch, yaw);
        
        /**
         * Compare measured direction vector to expected motor angular factors:
         * 
         * Case 1: Exact match (directions == angular_factors)
         *   Motor is correctly oriented, proceed to next motor
         *   
         * Case 2: Inverted match (-directions == angular_factors)  
         *   Motor is reversed, set reversal flag and proceed to next motor
         *   The reversal is saved persistently using motors.set_reversed()
         *   
         * Case 3: No match
         *   Detection failed, try opposite thrust direction or abort
         */
        // Good read, not inverted
        if (directions == motors.get_motor_angular_factors(current_motor)) {
            gcs().send_text(MAV_SEVERITY_INFO, "Thruster %d is ok!", current_motor + 1);
        }
        // Good read, inverted
        else if (-directions == motors.get_motor_angular_factors(current_motor)) {
            gcs().send_text(MAV_SEVERITY_INFO, "Thruster %d is reversed! Saving it!", current_motor + 1);
            motors.set_reversed(current_motor, true);
        }
        // Bad read!
        else {
            gcs().send_text(MAV_SEVERITY_INFO, "Bad thrust read, trying to push the other way...");
            // If we got here, we couldn't identify anything that made sense.
            // Let's try pushing the thruster the other way, maybe we are in too shallow waters or hit something
            if (current_direction == DOWN) {
                // The reading for the second direction was also bad, we failed.
                gcs().send_text(MAV_SEVERITY_WARNING, "Failed! Please check Thruster %d and frame setup!", current_motor + 1);
                md_state = DONE;
                break;
            }
            // Retry with opposite thrust direction (DOWN instead of UP)
            current_direction = DOWN;
            md_state = SETTLING;
            break;
        }
        
        /**
         * Motor detection successful for current motor:
         * - Reversal flag saved if needed (persistent across reboots)
         * - Advance to next motor
         * - Reset thrust direction to UP for next motor
         * - Check if more motors remain or transition to DONE
         */
        // If we got here, we have a decent motor reading
        md_state = SETTLING;
        // Test the next motor, if it exists
        current_motor++;
        current_direction = UP;
        if (!motors.motor_is_enabled(current_motor)) {
            md_state = DONE;
            gcs().send_text(MAV_SEVERITY_WARNING, "Motor direction detection is complete.");
        }
        break;
    }
    /**
     * DONE state: Motor detection complete or aborted
     * 
     * Entered when:
     * - All enabled motors successfully tested
     * - Detection failed on a motor (after trying both directions)
     * - output_test_num() failed during THRUSTING
     * 
     * Actions:
     * - Restore previous flight mode (mode before entering Motor Detection)
     * - Disarm vehicle with MOTORDETECTDONE reason
     * 
     * Any detected motor reversals have been automatically saved to the
     * motor configuration and will persist across reboots. User should verify
     * results and can test vehicle behavior in Manual mode.
     * 
     * @note ModeReason::MISSION_END is used for the mode change to indicate
     *       automated completion of a procedure.
     *       
     * @note Disarm reason MOTORDETECTDONE allows logging systems to identify
     *       that disarm occurred due to motor detection completion, not pilot action.
     */
    case DONE:
        set_mode(sub.prev_control_mode, ModeReason::MISSION_END);
        sub.arming.disarm(AP_Arming::Method::MOTORDETECTDONE);
        break;
    }
}
