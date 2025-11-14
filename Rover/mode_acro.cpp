/**
 * @file mode_acro.cpp
 * @brief Implementation of Acro mode for Rover - direct rate control without stabilization
 * 
 * @details Acro (Acrobatic) mode provides direct manual control of the vehicle's turn rate
 *          and speed/throttle without any GPS-based position hold or heading stabilization.
 *          
 *          Unlike other autonomous or stabilized modes, Acro mode:
 *          - Directly maps pilot stick input to turn rate (degrees per second)
 *          - Does not use GPS for position or heading hold
 *          - Does not provide any stabilization or corrections
 *          - Provides pure manual rate control similar to rate mode on multirotors
 *          
 *          Control Algorithm:
 *          1. Pilot roll/steering input is converted to desired turn rate (radians/sec)
 *          2. Pilot throttle input is converted to desired speed or direct throttle
 *          3. Steering controller outputs steering angle to achieve desired turn rate
 *          4. Throttle controller outputs motor commands to achieve desired speed
 *          
 *          The mode supports both regular vehicles and special configurations:
 *          - Balance bots: Uses pitch control for balancing while maintaining throttle control
 *          - Sailboats: Integrates with tacking controller for sail-based maneuvering
 *          
 * @note Use Cases:
 *       - Manual control when GPS is unavailable or unreliable
 *       - Aggressive maneuvering and stunts requiring direct control
 *       - Training and familiarization with vehicle handling characteristics
 *       - Testing motor and steering response without stabilization interference
 * 
 * @warning This mode provides no position or heading stabilization. Vehicle will not
 *          maintain heading or prevent drift. Requires constant pilot input.
 */
#include "Rover.h"

/**
 * @brief Main update function for Acro mode - executes once per control loop iteration
 * 
 * @details This function implements the core Acro mode control algorithm, providing direct
 *          rate control from pilot input without GPS-based stabilization or position hold.
 *          
 *          Control Flow:
 *          1. Check if forward speed measurement is available from wheel encoders
 *          2a. If NO speed available: Use direct throttle control from pilot stick
 *          2b. If speed available: Use speed controller with pilot-commanded speed
 *          3. Convert pilot steering input to desired turn rate (deg/s)
 *          4. Use rate controller to generate steering output for desired turn rate
 *          5. Handle special cases (balance bots, sailboat tacking)
 *          
 *          Key Differences from Other Modes:
 *          - No GPS position hold (unlike Loiter, Auto)
 *          - No heading hold (unlike Steering, Hold)
 *          - Direct turn rate command, not heading target (unlike Guided)
 *          - Pilot has full manual authority over turn rate and speed
 *          
 * @note Called at main control loop rate (typically 50Hz for Rover)
 * @note Maximum turn rate is limited by ACRO_TURN_RATE parameter
 * @note Steering range is ±4500 (representing ±100% servo output)
 * 
 * @warning No stabilization is provided - vehicle will not maintain heading or position
 */
void ModeAcro::update()
{
    // Get current forward speed from wheel encoders or speed sensor
    // Speed measurement enables closed-loop speed control vs open-loop throttle
    float speed, desired_steering;
    // Branch 1: No valid speed sensor available - use direct throttle control
    // This path is taken when wheel encoders are not available or not functioning
    if (!attitude_control.get_forward_speed(speed)) {
        float desired_throttle;
        
        // Convert pilot stick input into desired steering (±4500) and throttle (0-100%)
        // This is direct pass-through of pilot commands without any stabilization
        get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);

        // Special handling for balance bots (two-wheeled self-balancing vehicles)
        // Calculate pitch control commands to maintain balance while applying desired throttle
        if (rover.is_balancebot()) {
            rover.balancebot_pitch_control(desired_throttle);
        }

        // Apply throttle directly to motors without speed feedback control
        // This is open-loop control - actual speed will vary with terrain and load
        g2.motors.set_throttle(desired_throttle);
    } else {
        // Branch 2: Valid speed sensor available - use closed-loop speed control
        // This path provides better speed regulation using feedback from encoders
        float desired_speed;
        
        // Convert pilot stick input into desired steering (±4500) and speed (m/s)
        // Throttle stick position is scaled to target speed based on configuration
        get_pilot_desired_steering_and_speed(desired_steering, desired_speed);
        
        // Use speed controller to calculate throttle needed to achieve desired speed
        // This is closed-loop control providing consistent speed despite terrain variations
        calc_throttle(desired_speed, true);
    }

    float steering_out;

    // Special handling for sailboats which use wind for propulsion
    // Sailboats may perform tacking maneuvers (zigzag pattern to sail upwind)
    if (!is_zero(desired_steering)) {
        // Any pilot steering input cancels automatic tacking and returns control to user
        // This allows pilot to override automated sail maneuvers at any time
        g2.sailboat.clear_tack();
    }
    
    if (g2.sailboat.tacking()) {
        // During automatic tacking maneuver, use heading controller to follow tack trajectory
        // Tacking requires precise heading control to efficiently sail against the wind
        steering_out = attitude_control.get_steering_out_heading(g2.sailboat.get_tack_heading_rad(),
                                                                 g2.wp_nav.get_pivot_rate(),
                                                                 g2.motors.limit.steer_left,
                                                                 g2.motors.limit.steer_right,
                                                                 rover.G_Dt);
    } else {
        // Core Acro mode steering control: Convert pilot input to turn rate command
        // This is the key difference from other modes - direct rate control without heading hold
        
        // Scale pilot steering input (±4500) to desired turn rate (radians/sec)
        // ACRO_TURN_RATE parameter sets maximum turn rate (default typically 180 deg/s)
        // Example: Full stick (4500) → 1.0 × ACRO_TURN_RATE degrees/sec
        const float target_turn_rate = (desired_steering / 4500.0f) * radians(g2.acro_turn_rate);

        // Run rate controller to generate steering servo output achieving target turn rate
        // Controller uses feedback from gyro/heading sensors to achieve commanded rate
        // This provides rate stabilization (constant turn rate) but no heading hold
        steering_out = attitude_control.get_steering_out_rate(target_turn_rate,
                                                              g2.motors.limit.steer_left,
                                                              g2.motors.limit.steer_right,
                                                              rover.G_Dt);
    }

    // Convert normalized steering output (±1.0) back to servo range (±4500)
    // Final steering command is sent to servo/motor mixer for execution
    set_steering(steering_out * 4500.0f);
}

/**
 * @brief Indicates whether Acro mode requires velocity estimation from EKF
 * 
 * @details This method determines if the Extended Kalman Filter (EKF) needs to estimate
 *          vehicle velocity for this mode to function properly. Velocity estimation is
 *          required for vehicles with steering servos (car-like steering) to properly
 *          control vehicle motion, but not for skid-steering vehicles (tank-like) which
 *          can be controlled purely through differential motor speeds.
 *          
 *          Velocity Requirement Logic:
 *          - Skid-steering vehicles (differential drive): NO velocity required
 *            * Turn rate directly controlled by differential motor speeds
 *            * No need for velocity feedback
 *          - Regular steering vehicles (Ackermann/car-like): YES velocity required
 *            * Steering angle depends on velocity for desired turn rate
 *            * Velocity feedback needed for accurate rate control
 * 
 * @return true if velocity estimation is required, false otherwise
 * @return false for skid-steering vehicles (tank drive)
 * @return true for steering servo vehicles (car-like steering)
 * 
 * @note This affects EKF configuration and sensor fusion requirements
 */
bool ModeAcro::requires_velocity() const
{
    return !g2.motors.have_skid_steering();
}

/**
 * @brief Handle pilot-initiated tacking request for sailboats in Acro mode
 * 
 * @details Sailboats sailing upwind must perform tacking maneuvers (zigzag pattern) to
 *          make progress against the wind. In Acro mode, pilots can manually initiate
 *          a tack using a transmitter switch or button, allowing them to control when
 *          tacking occurs while letting the autopilot execute the precise maneuver.
 *          
 *          Tacking Process:
 *          1. Pilot activates tack request via transmitter switch
 *          2. Sailboat controller calculates optimal tack heading (typically 90-120° turn)
 *          3. Acro mode temporarily uses heading controller to execute tack
 *          4. Once tack complete, returns to rate control mode
 *          
 *          This provides a hybrid control mode:
 *          - Pilot has manual rate control most of the time (pure Acro)
 *          - Autopilot assists with precise tacking execution when requested
 *          - Pilot can override tack at any time with steering input
 * 
 * @note Only applies to sailboat configurations (FRAME_CLASS = Sailboat)
 * @note Tack request typically mapped to auxiliary RC channel switch
 * @note Manual steering input during tack will cancel the automated maneuver
 * 
 * @see ModeAcro::update() for tacking execution logic
 */
void ModeAcro::handle_tack_request()
{
    g2.sailboat.handle_tack_request_acro();
}
