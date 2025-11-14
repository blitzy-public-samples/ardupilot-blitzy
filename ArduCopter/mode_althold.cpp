#include "Copter.h"

/**
 * @file mode_althold.cpp
 * @brief AltHold flight mode implementation for ArduCopter
 * 
 * @details AltHold mode provides manual horizontal control (like Stabilize mode) combined
 *          with automatic altitude hold. This mode is recommended as the first altitude-holding
 *          mode for new pilots because it provides direct manual control in roll and pitch while
 *          automatically maintaining altitude.
 *          
 *          Key Features:
 *          - Pilot roll/pitch stick inputs control lean angles (same as Stabilize mode)
 *          - Centered throttle stick = hold current altitude
 *          - Throttle stick up/down = climb/descend at commanded rate
 *          - Automatic return to altitude hold when stick returns to center
 *          - Uses barometer and optionally rangefinder for altitude estimation
 *          - Learns and applies hover throttle automatically
 *          
 *          Control Algorithm:
 *          1. Pilot throttle stick position is converted to desired climb rate
 *          2. Position controller integrates climb rate to maintain altitude target
 *          3. Barometer provides primary altitude reference (fused with rangefinder if available)
 *          4. Hover throttle learning compensates for vehicle weight and conditions
 *          5. Vertical position controller outputs throttle command to motors
 *          
 *          The mode implements a state machine handling: MotorStopped, Landed_Ground_Idle,
 *          Landed_Pre_Takeoff, Takeoff, and Flying states to ensure smooth transitions
 *          and safe operation throughout the flight envelope.
 * 
 * @note This mode requires a barometer for altitude estimation
 * @warning Altitude hold accuracy depends on barometer calibration and environmental conditions
 * 
 * @see ModeAltHold class definition in mode.h
 * @see AC_PosControl for vertical position controller implementation
 */

/**
 * @brief Initialize AltHold mode controller
 * 
 * @details This function initializes the AltHold flight mode by setting up the vertical
 *          position controller with appropriate speed and acceleration limits. Called once
 *          when the pilot switches into AltHold mode.
 *          
 *          Initialization sequence:
 *          1. Initialize vertical (U-axis) position controller if not already active
 *          2. Configure maximum climb/descent speeds from parameters
 *          3. Configure acceleration limits for smooth altitude changes
 *          
 *          The vertical position controller (pos_control) manages altitude hold by:
 *          - Integrating desired climb rate commands into altitude targets
 *          - Computing throttle output to track altitude setpoint
 *          - Learning and applying hover throttle for the current vehicle configuration
 *          
 *          Speed limits are configured asymmetrically:
 *          - Descent speed: Configured via PILOT_SPEED_DN parameter (typically slower for safety)
 *          - Climb speed: Configured via PILOT_SPEED_UP parameter (g.pilot_speed_up)
 *          - Acceleration: Controlled by PILOT_ACCEL_Z parameter (g.pilot_accel_z)
 * 
 * @param[in] ignore_checks If true, skip pre-flight checks (used for emergency mode changes)
 * 
 * @return true Always returns true (AltHold mode initialization cannot fail)
 * 
 * @note This function is called at mode entry, not in the main control loop
 * @note The pos_control object is shared across multiple flight modes
 * 
 * @see ModeAltHold::run() for the main control loop
 * @see AC_PosControl::init_U_controller() for position controller initialization
 */
bool ModeAltHold::init(bool ignore_checks)
{

    // Initialize the vertical position controller if not already active.
    // The U-axis controller manages altitude by integrating climb rate commands
    // and outputting appropriate throttle to maintain or change altitude.
    if (!pos_control->is_active_U()) {
        pos_control->init_U_controller();
    }

    // Set vertical speed and acceleration limits from parameters.
    // These limits define how fast the vehicle can climb/descend in response to pilot input.
    // - First parameter: Maximum descent speed in cm/s (negative, from PILOT_SPEED_DN)
    // - Second parameter: Maximum climb speed in cm/s (from PILOT_SPEED_UP)
    // - Third parameter: Vertical acceleration limit in cm/s/s (from PILOT_ACCEL_Z)
    pos_control->set_max_speed_accel_U_cm(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_U_cmss(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    return true;
}

/**
 * @brief Main control loop for AltHold mode
 * 
 * @details Runs the AltHold flight mode controller, combining manual horizontal control
 *          (roll/pitch/yaw) with automatic altitude hold. This function implements the core
 *          AltHold algorithm: pilot throttle stick controls climb rate, which is integrated
 *          into an altitude target that is automatically maintained.
 *          
 *          Control Flow:
 *          1. Update speed/acceleration limits (allows dynamic parameter changes)
 *          2. Process pilot inputs:
 *             - Roll/pitch sticks → desired lean angles (manual horizontal control)
 *             - Yaw stick → desired yaw rate (manual heading control)
 *             - Throttle stick → desired climb rate (center = hold, up = climb, down = descend)
 *          3. Determine flight state (MotorStopped, Landed, Takeoff, Flying)
 *          4. Execute state-specific logic:
 *             - Landed states: Relax controllers, prepare for takeoff
 *             - Takeoff state: Execute automatic takeoff sequence
 *             - Flying state: Full altitude hold with pilot climb rate commands
 *          5. Apply attitude control (roll/pitch angles, yaw rate)
 *          6. Update vertical position controller to output throttle command
 *          
 *          Altitude Hold Algorithm:
 *          - Centered throttle stick (typically mid-stick) commands zero climb rate = hold altitude
 *          - Stick deflection is converted to climb rate proportional to deflection amount
 *          - Position controller integrates climb rate into altitude setpoint
 *          - Barometer (and optionally rangefinder) provides altitude feedback
 *          - PID controller computes throttle to track altitude setpoint
 *          - Hover throttle is learned automatically and compensates for vehicle weight
 *          
 *          The relationship between throttle stick and climb rate:
 *          - Stick at center (50%): 0 cm/s climb rate → hold current altitude
 *          - Stick full up (100%): +PILOT_SPEED_UP cm/s → climb at maximum rate
 *          - Stick full down (0%): -PILOT_SPEED_DN cm/s → descend at maximum rate
 *          - Exponential curve can be applied via PILOT_Y_EXPO parameter for finer control
 *          
 *          Barometer/Rangefinder Fusion:
 *          - Primary altitude source: Barometer (calibrated at startup)
 *          - Secondary altitude source: Rangefinder (when available and in range)
 *          - Surface tracking adjusts altitude target to maintain height above ground
 *          - EKF fuses barometer and rangefinder data for optimal altitude estimate
 *          
 *          Hover Throttle Learning:
 *          - Position controller learns the throttle required to hover (typically 40-60%)
 *          - Compensates for battery voltage, vehicle weight, and atmospheric conditions
 *          - Enables accurate altitude hold without manual throttle adjustment
 *          - Learning happens automatically during altitude hold (STICK_NEUTRAL state)
 *          
 *          Why AltHold is Recommended for New Pilots:
 *          - Simpler than Loiter (no GPS required, no horizontal position hold complexity)
 *          - More capable than Stabilize (automatic altitude control reduces pilot workload)
 *          - Intuitive control: pilot directly commands climb/descend rate
 *          - Forgiving: releasing throttle stick returns to altitude hold
 *          - Good training mode: teaches altitude awareness while simplifying one axis of control
 * 
 * @note This function should be called at 400Hz (main loop rate) for optimal performance
 * @note Requires barometer for altitude reference
 * @note Rangefinder is optional but improves accuracy near ground
 * 
 * @warning Altitude hold accuracy degrades in high wind or rapidly changing atmospheric pressure
 * @warning Always monitor altitude when first using AltHold in a new location
 * 
 * @see ModeAltHold::init() for initialization
 * @see AC_PosControl::set_pos_target_U_from_climb_rate_cm() for climb rate integration
 * @see AC_PosControl::update_U_controller() for throttle output computation
 */
void ModeAltHold::run()
{
    // Set vertical speed and acceleration limits from parameters.
    // This is done every loop to allow real-time parameter changes during flight.
    // Pilots can adjust PILOT_SPEED_UP, PILOT_SPEED_DN, and PILOT_ACCEL_Z parameters
    // and the changes take effect immediately.
    pos_control->set_max_speed_accel_U_cm(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // Apply SIMPLE mode transform to pilot inputs if enabled.
    // SIMPLE mode rotates pilot stick inputs relative to the direction the vehicle was pointing
    // at arming, making it easier to fly for beginners by maintaining consistent stick directions
    // regardless of vehicle yaw angle.
    update_simple_mode();

    // Get pilot desired lean angles from roll and pitch stick inputs.
    // This provides the same manual horizontal control as Stabilize mode.
    // The lean angle is limited by two parameters:
    // - attitude_control->lean_angle_max_rad(): Maximum lean angle for all modes (ANGLE_MAX)
    // - attitude_control->get_althold_lean_angle_max_rad(): AltHold-specific limit (can be more conservative)
    // Using a lower lean angle limit in AltHold helps maintain altitude accuracy since
    // aggressive maneuvering can cause altitude deviations.
    float target_roll_rad, target_pitch_rad;
    get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, attitude_control->lean_angle_max_rad(), attitude_control->get_althold_lean_angle_max_rad());

    // Get pilot's desired yaw rate from yaw stick input.
    // Yaw control in AltHold is rate-based: stick deflection commands rotation rate,
    // not absolute heading. This provides intuitive manual heading control.
    float target_yaw_rate_rads = get_pilot_desired_yaw_rate_rads();

    // Get pilot desired climb rate from throttle stick input.
    // This is the core of AltHold mode's altitude control:
    // - Throttle stick centered (50%) = 0 cm/s climb rate = hold altitude
    // - Stick pushed up = positive climb rate (vehicle climbs)
    // - Stick pulled down = negative climb rate (vehicle descends)
    // - When stick returns to center, climb rate returns to 0 and altitude is held
    // The climb rate is proportional to stick deflection, with expo curve for fine control.
    float target_climb_rate_cms = get_pilot_desired_climb_rate();
    
    // Constrain the climb rate to configured maximum values.
    // This ensures the vehicle doesn't climb or descend faster than safe limits,
    // even if the stick input calculation produces values outside the range.
    // Descent rate is typically limited to a slower value than climb rate for safety.
    target_climb_rate_cms = constrain_float(target_climb_rate_cms, -get_pilot_speed_dn(), g.pilot_speed_up);

    // Determine current state in the AltHold state machine.
    // The state machine ensures safe and smooth operation throughout the flight envelope:
    // - MotorStopped: Motors disarmed or stopped
    // - Landed_Ground_Idle: On ground with motors at idle
    // - Landed_Pre_Takeoff: On ground, ready for takeoff
    // - Takeoff: Executing automatic takeoff sequence
    // - Flying: Normal flight with full altitude hold active
    // State is determined based on climb rate command, throttle position, and vehicle sensors.
    AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate_cms);

    // Execute state-specific control logic.
    // Each state handles controller initialization, reset, and output appropriately
    // to ensure smooth transitions and safe operation.
    switch (althold_state) {

    case AltHoldModeState::MotorStopped:
        // Motors are stopped (vehicle disarmed or emergency stop).
        // Reset all controllers to prevent integrator windup and prepare for next flight.
        
        // Reset attitude rate controller integral terms to zero.
        // This prevents accumulated I-term from previous flight affecting the next flight.
        attitude_control->reset_rate_controller_I_terms();
        
        // Reset yaw target and rate without updating the actual yaw.
        // This ensures yaw controller starts fresh when motors restart.
        attitude_control->reset_yaw_target_and_rate(false);
        
        // Relax the vertical position controller and force throttle to zero.
        // This ensures no throttle output while motors are stopped and resets altitude hold state.
        pos_control->relax_U_controller(0.0f);
        break;

    case AltHoldModeState::Landed_Ground_Idle:
        // Vehicle is on the ground with motors at idle.
        // This state is active when the vehicle has landed or is sitting on the ground
        // with motors armed but no pilot climb command.
        
        // Reset yaw target to current heading to prevent unexpected rotation on takeoff.
        // This ensures the vehicle maintains its current heading when transitioning to flight.
        attitude_control->reset_yaw_target_and_rate();
        
        // Fall through to Landed_Pre_Takeoff case to handle common landed state logic.
        FALLTHROUGH;

    case AltHoldModeState::Landed_Pre_Takeoff:
        // Vehicle is on the ground and preparing for takeoff.
        // This state is active when the pilot has not yet commanded significant climb rate
        // but the vehicle is armed and ready to fly.
        
        // Smoothly reset attitude rate controller integral terms.
        // Unlike MotorStopped state, this is done gradually to avoid sudden control changes
        // if the pilot begins takeoff while controllers are resetting.
        attitude_control->reset_rate_controller_I_terms_smoothly();
        
        // Relax vertical position controller with zero throttle target.
        // The controller maintains its state but outputs minimal throttle while on ground.
        // This allows smooth transition to altitude hold when takeoff begins.
        pos_control->relax_U_controller(0.0f);
        break;

    case AltHoldModeState::Takeoff:
        // Automatic takeoff sequence is active.
        // This state manages the initial climb phase from ground to a safe altitude
        // where normal altitude hold can take over.
        
        // Initiate takeoff sequence if not already running.
        // The takeoff controller manages the initial climb to PILOT_TAKEOFF_ALT parameter.
        if (!takeoff.running()) {
            // Start takeoff to the configured takeoff altitude (constrained to 0-1000cm range).
            // This ensures the vehicle climbs to a safe altitude before transitioning to
            // normal altitude hold. The parameter g.pilot_takeoff_alt is typically 50-200cm.
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // Apply avoidance system adjustments to climb rate if proximity sensors detect obstacles.
        // This can slow or stop the climb if an obstacle is detected above the vehicle.
        target_climb_rate_cms = get_avoidance_adjusted_climbrate_cms(target_climb_rate_cms);

        // Execute pilot-controlled takeoff sequence.
        // This allows the pilot to adjust climb rate during takeoff with the throttle stick
        // while the takeoff controller manages the overall sequence and prevents early landing detection.
        takeoff.do_pilot_takeoff(target_climb_rate_cms);
        break;

    case AltHoldModeState::Flying:
        // Normal flight state - full altitude hold is active.
        // This is the primary state where AltHold mode operates: pilot commands climb rate
        // via throttle stick, and the position controller maintains altitude automatically.
        
        // Allow motors to use full throttle range (0-100%).
        // This enables the position controller to output whatever throttle is needed to
        // maintain altitude, including throttle values above or below hover throttle.
        // THROTTLE_UNLIMITED state allows climbing and descending as commanded.
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

#if AP_AVOIDANCE_ENABLED
        // Apply obstacle avoidance adjustments to horizontal control.
        // If proximity sensors (rangefinders, lidar) detect obstacles in the flight path,
        // this modifies roll and pitch commands to avoid collision while maintaining altitude hold.
        // The avoidance system can reduce lean angles or bias the vehicle away from obstacles.
        copter.avoid.adjust_roll_pitch_rad(target_roll_rad, target_pitch_rad, attitude_control->lean_angle_max_rad());
#endif

        // Apply avoidance system adjustments to vertical climb rate.
        // This can slow, stop, or reverse climb rate if obstacles are detected above or below.
        // Ensures the vehicle doesn't climb into overhead obstacles or descend into ground/obstacles.
        target_climb_rate_cms = get_avoidance_adjusted_climbrate_cms(target_climb_rate_cms);

#if AP_RANGEFINDER_ENABLED
        // Update surface tracking altitude offset using rangefinder measurements.
        // When flying close to the ground and a rangefinder is available, surface tracking
        // adjusts the altitude target to maintain consistent height above terrain.
        // This improves altitude hold accuracy near the ground by using rangefinder data
        // (which has better short-range accuracy than barometer) fused with barometer data.
        // The EKF combines barometer and rangefinder to provide optimal altitude estimate.
        copter.surface_tracking.update_surface_offset();
#endif

        // Send the commanded climb rate to the position controller.
        // This is the heart of AltHold mode's altitude control:
        // - If climb rate is 0 (throttle centered): Controller maintains current altitude
        // - If climb rate is positive (throttle up): Controller climbs at commanded rate
        // - If climb rate is negative (throttle down): Controller descends at commanded rate
        // The position controller integrates the climb rate into an altitude target and
        // computes the throttle output needed to track that target. Hover throttle learning
        // happens automatically within the position controller, compensating for vehicle weight,
        // battery voltage, and atmospheric conditions.
        pos_control->set_pos_target_U_from_climb_rate_cm(target_climb_rate_cms);
        break;
    }

    // Call the attitude controller with pilot-commanded roll, pitch, and yaw inputs.
    // This provides the manual horizontal control that makes AltHold feel like Stabilize mode
    // in roll, pitch, and yaw. The attitude controller converts the desired lean angles and
    // yaw rate into motor commands, working in parallel with the altitude controller.
    // - target_roll_rad: Desired roll angle in radians (from pilot stick input)
    // - target_pitch_rad: Desired pitch angle in radians (from pilot stick input)
    // - target_yaw_rate_rads: Desired yaw rate in radians/second (from pilot stick input)
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(target_roll_rad, target_pitch_rad, target_yaw_rate_rads);

    // Update the vertical position controller and compute output throttle.
    // This is the final step that completes the altitude hold control loop:
    // 1. Reads current altitude from barometer/rangefinder (via EKF)
    // 2. Compares current altitude to target altitude (set by climb rate integration)
    // 3. Computes altitude error and error derivative
    // 4. Runs PID control algorithm to determine required throttle
    // 5. Applies learned hover throttle as feedforward term
    // 6. Outputs throttle command to motors (combined with attitude controller output)
    // The resulting throttle command is merged with roll/pitch/yaw commands from the
    // attitude controller to produce final motor outputs that maintain both attitude and altitude.
    pos_control->update_U_controller();
}
