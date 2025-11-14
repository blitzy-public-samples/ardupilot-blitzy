/**
 * @file mode_steering.cpp
 * @brief Implementation of Steering mode for Rover - stabilized manual control with heading hold
 * 
 * @details Steering mode provides driver-friendly manual control with the following features:
 * 
 * **Control Characteristics**:
 * - Pilot controls desired lateral acceleration (not direct servo output)
 * - Vehicle automatically compensates for speed changes to maintain consistent feel
 * - Smooth, predictable steering response with physics-based limits
 * - Separate handling for forward/reverse driving
 * 
 * **Steering Strategies**:
 * 
 * 1. **Ackermann/Normal Steering** (standard wheeled vehicles):
 *    - Lateral acceleration control based on speed and turn radius
 *    - Uses equation: max_lateral_accel = speed² / turn_radius
 *    - Attitude controller converts lateral accel to steering servo commands
 * 
 * 2. **Skid-Steering Pivot Turns** (tank-style vehicles):
 *    - When speed = 0 and steering input present, perform zero-radius turn
 *    - Converts pilot steering input to desired yaw rate (radians/sec)
 *    - Turn rate controller drives left/right motor differential
 * 
 * **Coordinate Frames**:
 * - Lateral acceleration: Body-frame (positive = right turn)
 * - Heading: Earth-frame NED (North-East-Down convention)
 * - Steering output: Normalized -1.0 to +1.0, scaled to -4500 to +4500 for servo
 * 
 * **Integration Points**:
 * - g2.attitude_control: Converts lateral accel/turn rate to steering servo output
 * - g2.motors: Receives steering and throttle commands for motor output
 * - Base class methods: calc_steering_from_lateral_acceleration(), calc_throttle()
 * 
 * @note Steering mode provides more stable and predictable handling compared to Manual mode,
 *       especially at varying speeds. The vehicle will drive similarly at different speeds
 *       because lateral acceleration limits are automatically adjusted.
 * 
 * @see Mode::calc_steering_from_lateral_acceleration()
 * @see Mode::calc_throttle()
 * @see AR_AttitudeControl
 */

#include "Rover.h"

/**
 * @brief Main update loop for Steering mode - runs at scheduler frequency (typically 50Hz)
 * 
 * @details This method implements the core steering mode control algorithm:
 * 
 * **Control Flow**:
 * 1. Obtain current vehicle forward speed from attitude controller
 * 2. Get pilot inputs (steering stick and throttle stick)
 * 3. Determine if skid-steer pivot turn is requested (speed=0 with steering input)
 * 4. Execute either:
 *    - Pivot turn controller: steering input → yaw rate → motor differential
 *    - Lateral acceleration controller: steering input → lateral accel → steering angle
 * 5. Execute speed controller: throttle input → motor throttle output
 * 
 * **Safety Behavior**:
 * - If forward speed unavailable (sensor failure): Stop all motors immediately
 * - Lateral acceleration limited by turn_lat_accel_max parameter
 * - Turn radius constrained to minimum 0.1m to prevent divide-by-zero
 * 
 * **Performance Notes**:
 * - Called at main loop rate (50Hz typical, configurable via SCHED_LOOP_RATE)
 * - Must complete within loop time budget (20ms @ 50Hz)
 * - Integrates with attitude controller for smooth servo output
 */
void ModeSteering::update()
{
    // ========== Speed Acquisition ==========
    // Obtain current forward speed from attitude controller (fuses wheel encoders, GPS, IMU)
    // Speed used for: (1) lateral accel calculation, (2) determining if vehicle is stopped
    // Units: meters/second, positive = forward, negative = reverse
    float speed;
    if (!attitude_control.get_forward_speed(speed)) {
        // Speed estimation failed (sensor fault or insufficient data)
        // SAFETY: Stop vehicle immediately - cannot safely calculate steering without speed
        g2.motors.set_throttle(0.0f);    // Zero throttle output
        g2.motors.set_steering(0.0f);    // Center steering
        _desired_lat_accel = 0.0f;       // Clear desired lateral acceleration state
        return;
    }

    // ========== Pilot Input Processing ==========
    // Retrieve pilot commands from RC transmitter (with deadzone and expo applied)
    // desired_steering: -4500 to +4500 (left to right, scaled from RC input)
    // desired_speed: -1.0 to +1.0 (normalized speed, negative = reverse)
    float desired_steering, desired_speed;
    get_pilot_desired_steering_and_speed(desired_steering, desired_speed);

    // Determine driving direction for steering reversal logic
    // When reversing, lateral acceleration signs must be flipped for correct steering
    bool reversed = is_negative(desired_speed);

    // ========== Pivot Turn Detection (Skid-Steering Only) ==========
    // Pivot turn = zero-radius turn in place (like a tank)
    // Requirements: (1) Vehicle has skid-steering capability, (2) Pilot requests zero speed
    if (g2.motors.have_skid_steering() && is_zero(desired_speed)) {
        // **Pivot Turn Mode**: Use yaw rate controller instead of lateral acceleration
        // This allows the vehicle to spin in place by driving left/right tracks oppositely
        
        // Convert pilot steering stick position to desired turn rate
        // Calculation: Full stick (±4500) → ±g2.acro_turn_rate (degrees/sec) → radians/sec
        // Example: If acro_turn_rate=180°/s, full right stick = +π rad/s clockwise
        const float target_turn_rate = (desired_steering / 4500.0f) * radians(g2.acro_turn_rate);
        _desired_lat_accel = 0.0f;  // Not using lateral accel control in pivot turn mode

        // Run turn rate controller: Converts yaw rate to differential motor output
        // Inputs:
        //   - target_turn_rate: Desired rotation rate in radians/sec (body-frame Z-axis)
        //   - limit flags: Prevent integrator windup when motors saturated
        //   - rover.G_Dt: Loop time for discrete integration (typically 0.02s @ 50Hz)
        // Output: steering_out in range -1.0 to +1.0 (normalized motor differential)
        const float steering_out = attitude_control.get_steering_out_rate(target_turn_rate,
                                                                          g2.motors.limit.steer_left,
                                                                          g2.motors.limit.steer_right,
                                                                          rover.G_Dt);
        set_steering(steering_out * 4500.0f);  // Scale to -4500 to +4500 for motor output
    } else {
        // ========== Normal Steering Mode (Lateral Acceleration Control) ==========
        // **Lateral Acceleration Control Strategy**:
        // Control lateral acceleration directly instead of steering angle. This provides:
        // - Consistent steering feel across different speeds
        // - Automatic speed-dependent steering gain adjustment
        // - Physics-based limits that prevent vehicle rollover or tire slip
        
        // Calculate maximum achievable lateral acceleration at current speed
        // Physics: For circular motion, centripetal acceleration a = v²/r
        //   where v = forward speed (m/s), r = turn radius (m)
        // This represents the lateral acceleration at full steering lock
        // Example: At 5 m/s with 3m turn radius → max_g = 25/3 = 8.33 m/s²
        float max_g_force = speed * speed / MAX(g2.turn_radius, 0.1f);
        
        // Constrain to safe limits:
        //   - Minimum 0.1 m/s²: Ensure some steering authority at very low speeds
        //   - Maximum turn_lat_accel_max: Prevent excessive lateral G that could tip vehicle
        //     Typical values: 2-5 m/s² depending on vehicle stability
        max_g_force = constrain_float(max_g_force, 0.1f, attitude_control.get_turn_lat_accel_max());

        // Convert pilot steering stick position to desired lateral acceleration
        // Calculation: normalized_input (-1.0 to +1.0) * max_achievable_lateral_accel
        // Result: _desired_lat_accel in m/s² (body-frame, positive = right turn)
        // Example: Half right stick (2250) with max_g=5 m/s² → 2.5 m/s² rightward
        _desired_lat_accel = max_g_force * (desired_steering / 4500.0f);

        // **Reverse Driving Correction**:
        // When driving backwards, steering direction must be reversed for intuitive control
        // (turning the wheel right should still turn the vehicle right from driver's perspective)
        // This flip maintains natural steering feel regardless of driving direction
        if (reversed) {
            _desired_lat_accel = -_desired_lat_accel;
        }

        // Execute lateral acceleration → steering angle conversion
        // This method (inherited from Mode base class) performs:
        //   1. Converts lateral accel (m/s²) to required steering angle using vehicle kinematics
        //   2. Applies attitude controller to achieve desired steering angle
        //   3. Handles Ackermann steering geometry for front-wheel steering vehicles
        //   4. Outputs servo command to g2.motors for execution
        // The 'reversed' flag adjusts controller behavior for reverse driving
        calc_steering_from_lateral_acceleration(_desired_lat_accel, reversed);
    }

    // ========== Speed Control ==========
    // Execute speed controller: Converts desired speed to throttle output
    // Parameters:
    //   - desired_speed: Pilot's requested speed (-1.0 to +1.0 normalized)
    //   - true: Enable active braking (important for stopping on slopes)
    // Controller applies:
    //   - Speed limit constraints from parameters
    //   - Acceleration/deceleration limits for smooth driving
    //   - Throttle curve and expo from parameters
    //   - PID control to maintain desired speed
    calc_throttle(desired_speed, true);
}
