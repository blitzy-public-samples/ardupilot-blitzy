/**
 * @file AP_MotorsUGV.h
 * @brief Motor control library for Unmanned Ground Vehicles (UGV)
 * 
 * This file defines the AP_MotorsUGV class which provides motor and servo control
 * for ground vehicles including rovers, boats, and omni-directional vehicles.
 * Supports multiple drive configurations: regular steering/throttle, skid-steering,
 * vectored thrust, omni-directional, and sailboat controls.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <AP_Arming/AP_Arming.h>
#include <AP_WheelEncoder/AP_WheelRateControl.h>
#include <SRV_Channel/SRV_Channel.h>

/**
 * @class AP_MotorsUGV
 * @brief Rover motor control for unmanned ground vehicles
 * 
 * @details This class implements motor and servo control for ground vehicles with
 * support for multiple drive configurations:
 * - Regular steering/throttle (Ackermann-style)
 * - Skid-steering (differential drive)
 * - Vectored thrust (boat with motor on steering servo)
 * - Omni-directional (mecanum, kiwi, omnix configurations)
 * - Sailboat controls (mainsail, wingsail, mast rotation)
 * 
 * Key features:
 * - Throttle and brake control with configurable limits (-100 to +100 percent)
 * - Steering control with optional speed-based scaling (-4500 to +4500 range)
 * - Motor output scaling and thrust curve compensation
 * - Slew rate limiting for smooth transitions (especially forward/reverse)
 * - Safety features including arming checks and emergency stop
 * - Support for various PWM output types (Normal, OneShot, DShot, Brushed)
 * - Wheel rate control integration for closed-loop speed control
 * 
 * Coordinate conventions:
 * - Steering: -4500 (full left) to +4500 (full right), represents normalized 
 *   steering input roughly equivalent to -45 to +45 degrees
 * - Throttle: -100 (full reverse) to +100 (full forward) percent
 * - Lateral: -100 (left) to +100 (right) percent for omni vehicles
 * - Roll/Pitch/Walking height: -1 to +1 normalized range
 * 
 * @note This is called at the main loop rate (typically 50Hz for ground vehicles)
 * @warning Incorrect motor configuration can cause vehicle instability or loss of control
 * 
 * @see AP_WheelRateControl for closed-loop speed control
 * @see SRV_Channel for servo output configuration
 */
class AP_MotorsUGV {
public:
    /**
     * @brief Constructor for AP_MotorsUGV
     * 
     * @param[in] rate_controller Reference to wheel rate controller for closed-loop speed control
     * 
     * @details Initializes the motor control system with a reference to the wheel rate
     * controller which provides closed-loop speed control capabilities for ground vehicles.
     */
    AP_MotorsUGV(AP_WheelRateControl& rate_controller);

    /**
     * @brief Get singleton instance of AP_MotorsUGV
     * 
     * @return Pointer to the singleton instance, or nullptr if not initialized
     * 
     * @note Only one instance of AP_MotorsUGV should exist per vehicle
     */
    static AP_MotorsUGV    *get_singleton(void) { return _singleton; }

    /**
     * @enum motor_test_order
     * @brief Motor test sequence options for DO_MOTOR_TEST MAVLink command
     * 
     * @details Defines the order in which motors and servos can be tested individually.
     * Used by ground control stations to verify correct motor/servo operation and direction.
     */
    enum motor_test_order {
        MOTOR_TEST_THROTTLE = 1,        ///< Test main throttle output
        MOTOR_TEST_STEERING = 2,        ///< Test steering servo output
        MOTOR_TEST_THROTTLE_LEFT = 3,   ///< Test left throttle (skid-steer vehicles)
        MOTOR_TEST_THROTTLE_RIGHT = 4,  ///< Test right throttle (skid-steer vehicles)
        MOTOR_TEST_MAINSAIL = 5,        ///< Test mainsail servo (sailboats)
        MOTOR_TEST_LAST                 ///< Boundary marker, not a valid test option
    };

    /**
     * @enum frame_type
     * @brief Supported omni-directional motor configurations
     * 
     * @details Defines the geometric arrangement of motors for omni-directional vehicles.
     * Each configuration uses different motor mixing matrices to achieve omnidirectional
     * movement capability (forward/backward, lateral left/right, and rotation).
     */
    enum frame_type {
        FRAME_TYPE_UNDEFINED = 0,    ///< Not an omni vehicle, uses regular or skid-steering
        FRAME_TYPE_OMNI3 = 1,        ///< Three-motor kiwi drive configuration (120° spacing)
        FRAME_TYPE_OMNIX = 2,        ///< Four-motor X configuration (45° motor angles)
        FRAME_TYPE_OMNIPLUS = 3,     ///< Four-motor + configuration (0°, 90°, 180°, 270°)
        FRAME_TYPE_OMNI3MECANUM = 4, ///< Three-motor mecanum wheel configuration
    };

    /**
     * @brief Initialize motors with specified frame type
     * 
     * @param[in] ftype Frame type from frame_type enum (OMNI3, OMNIX, OMNIPLUS, OMNI3MECANUM, or UNDEFINED)
     * 
     * @details Performs motor system initialization including:
     * - Configuring motor mixing based on frame type
     * - Setting up PWM output type (Normal, OneShot, DShot, Brushed)
     * - Initializing servo output ranges
     * - Configuring safety outputs for CPU failure scenarios
     * - Setting up wheel rate control integration
     * 
     * Must be called during vehicle initialization before motor outputs are used.
     * For regular steering/throttle vehicles, use FRAME_TYPE_UNDEFINED.
     * 
     * @note Should be called once during vehicle setup
     */
    void init(uint8_t ftype);

    /**
     * @brief Check if motors are currently active
     * 
     * @return true if any motor output is non-zero, false otherwise
     * 
     * @details Used to determine if the vehicle is actively outputting motor commands.
     * Can be used for safety checks and mode transition logic.
     */
    bool active() const;

    /**
     * @brief Setup motor output in case of main CPU failure
     * 
     * @details Configures failsafe PWM output values that will be output by the
     * I/O processor if the main CPU fails or stops communicating. Typically sets
     * throttle to zero and steering to neutral for safety.
     * 
     * @note Called during initialization and when safety settings change
     * @warning Critical safety function - incorrect configuration could lead to
     * unsafe vehicle behavior during CPU failure
     */
    void setup_safety_output();

    /**
     * @brief Setup servo output ranges for all motor and steering channels
     * 
     * @details Configures min/max/trim PWM values for all servo outputs based on
     * vehicle configuration and parameter settings. Sets appropriate output ranges
     * for regular servos, ESCs, and brushed motor controllers.
     * 
     * @note Called during initialization and when servo parameters change
     */
    void setup_servo_output();

    /**
     * @brief Get current steering input value
     * 
     * @return Current steering input in range -4500 (full left) to +4500 (full right)
     * 
     * @details Returns the most recently set steering value. This value may have
     * had speed-based or angle-based scaling applied depending on vehicle configuration
     * and the apply_scaling parameter used in set_steering().
     * 
     * @note Steering convention: -4500 represents full left, 0 is center, +4500 is full right.
     * This unusual range (vs typical -1 to +1) maintains compatibility with ArduPilot's
     * angle representation where 4500 = 45 degrees = 0.45 in normalized units.
     */
    float get_steering() const { return _steering; }

    /**
     * @brief Set steering input value
     * 
     * @param[in] steering Desired steering input from -4500 (full left) to +4500 (full right)
     * @param[in] apply_scaling If true (default), apply speed-based and angle-based scaling.
     *                          Set to false for manual modes where direct control is desired.
     * 
     * @details Sets the desired steering angle for the vehicle. The steering value is:
     * - Stored for use in the next output() call
     * - Optionally scaled based on vehicle speed to improve handling at high speeds
     * - Optionally limited based on vectored thrust angle constraints
     * - Applied differently based on vehicle type (regular steering, skid-steer, or omni)
     * 
     * The apply_scaling parameter should be:
     * - true for autonomous modes (auto, guided, RTL) where speed-dependent scaling improves tracking
     * - false for manual modes (manual, hold) where pilot expects direct steering response
     * 
     * @note Steering is applied during the next output() call, not immediately
     * @warning Steering convention differs from typical -1 to +1 range. Use -4500 to +4500.
     */
    void set_steering(float steering, bool apply_scaling = true);

    /**
     * @brief Get current throttle input value
     * 
     * @return Current throttle input in range -100 (full reverse) to +100 (full forward) percent
     * 
     * @details Returns the most recently set throttle value before any slew rate limiting
     * or thrust curve scaling is applied.
     */
    float get_throttle() const { return _throttle; }

    /**
     * @brief Set throttle input value
     * 
     * @param[in] throttle Desired throttle from -100 (full reverse) to +100 (full forward) percent
     * 
     * @details Sets the desired throttle for the vehicle. The throttle value is:
     * - Stored for use in the next output() call
     * - Limited to configured min/max throttle parameters
     * - Slew rate limited to prevent abrupt forward/reverse transitions
     * - Scaled by thrust curve exponent for non-linear thrust response
     * - Applied with asymmetry compensation for skid-steering vehicles
     * 
     * Throttle application varies by vehicle type:
     * - Regular vehicles: Applied to single throttle output
     * - Skid-steering: Mixed with steering to create left/right motor commands
     * - Vectored thrust: Applied through steering servo angle
     * - Omni vehicles: Mixed with steering and lateral for omnidirectional control
     * 
     * @note Throttle is applied during the next output() call, not immediately
     * @note Negative values indicate reverse motion
     * @warning Rapid throttle reversals may be slew rate limited for vehicle stability
     */
    void set_throttle(float throttle);

    /**
     * @brief Get current roll input value
     * 
     * @return Current roll input in normalized range -1 (full left) to +1 (full right)
     * 
     * @details Returns roll input for vehicles with independent roll control capability,
     * such as legged robots or vehicles with active suspension.
     */
    float get_roll() const { return _roll; }

    /**
     * @brief Set roll input value
     * 
     * @param[in] roll Desired roll from -1 (full left) to +1 (full right), normalized
     * 
     * @details Sets roll input for vehicles with independent roll control capability.
     * The roll value is stored and applied during the next output() call to appropriate
     * servo channels configured for roll function.
     * 
     * @note Primarily used by legged robots and vehicles with active suspension
     */
    void set_roll(float roll);

    /**
     * @brief Get current pitch input value
     * 
     * @return Current pitch input in normalized range -1 (nose down) to +1 (nose up)
     * 
     * @details Returns pitch input for vehicles with independent pitch control capability,
     * such as legged robots or vehicles with active suspension.
     */
    float get_pitch() const { return _pitch; }

    /**
     * @brief Set pitch input value
     * 
     * @param[in] pitch Desired pitch from -1 (nose down) to +1 (nose up), normalized
     * 
     * @details Sets pitch input for vehicles with independent pitch control capability.
     * The pitch value is stored and applied during the next output() call to appropriate
     * servo channels configured for pitch function.
     * 
     * @note Primarily used by legged robots and vehicles with active suspension
     */
    void set_pitch(float pitch);

    /**
     * @brief Get current walking height input value
     * 
     * @return Current walking height in normalized range -1 (lowest) to +1 (highest)
     * 
     * @details Returns walking height input for legged robots with variable stance height.
     */
    float get_walking_height() const { return _walking_height; }

    /**
     * @brief Set walking height input value
     * 
     * @param[in] walking_height Desired walking height from -1 (lowest) to +1 (highest), normalized
     * 
     * @details Sets walking height for legged robots with adjustable stance height.
     * The walking height value is stored and applied during the next output() call to
     * appropriate servo channels configured for walking height function.
     * 
     * @note Primarily used by legged robots (hexapods, quadrupeds, etc.)
     */
    void set_walking_height(float walking_height);

    /**
     * @brief Get current lateral input value
     * 
     * @return Current lateral input in range -100 (left) to +100 (right) percent
     * 
     * @details Returns lateral (sideways) motion input for omni-directional vehicles.
     * Only applicable for vehicles with omnidirectional capability (mecanum, kiwi, etc.).
     */
    float get_lateral() const { return _lateral; }

    /**
     * @brief Set lateral input value
     * 
     * @param[in] lateral Desired lateral motion from -100 (left) to +100 (right) percent
     * 
     * @details Sets lateral (sideways) motion input for omni-directional vehicles.
     * The lateral value is mixed with throttle and steering to produce individual motor
     * commands that achieve the desired omnidirectional motion.
     * 
     * Motor mixing is based on frame type:
     * - OMNI3: Three motors at 120° spacing
     * - OMNIX: Four motors in X configuration (45° angles)
     * - OMNIPLUS: Four motors in + configuration (0°, 90°, 180°, 270°)
     * - OMNI3MECANUM: Three mecanum wheels
     * 
     * @note Only has effect on vehicles configured with omni frame types
     * @note Lateral motion is achieved through appropriate motor mixing, not separate servos
     */
    void set_lateral(float lateral);

    /**
     * @brief Set mainsail input value
     * 
     * @param[in] mainsail Desired mainsail position from 0 (fully eased/out) to 100 (fully sheeted in/tight) percent
     * 
     * @details Sets the mainsail servo position for sailboats. The mainsail controls the
     * primary sail angle relative to the wind. Value of 0 releases the sail fully, 100
     * pulls it in tight to the centerline.
     * 
     * @note Only applicable for sailboat vehicles
     * @note Mainsail control is typically automated based on wind direction and desired course
     */
    void set_mainsail(float mainsail);

    /**
     * @brief Get current mainsail input value
     * 
     * @return Current mainsail position in range 0 (fully eased out) to 100 (fully sheeted in) percent
     */
    float get_mainsail() const { return _mainsail; }

    /**
     * @brief Set wingsail input value
     * 
     * @param[in] wingsail Desired wingsail position from -100 (full port) to +100 (full starboard) percent
     * 
     * @details Sets the wingsail servo position for sailboats with rigid wing sails.
     * Unlike mainsails which are eased in/out, wingsails rotate around a central mast
     * and can be positioned on either side (port/starboard).
     * 
     * - Negative values: Wing angled to port (left) side
     * - Positive values: Wing angled to starboard (right) side
     * - Zero: Wing aligned with centerline
     * 
     * @note Only applicable for sailboat vehicles with wingsail configuration
     */
    void set_wingsail(float wingsail);

    /**
     * @brief Get current wingsail input value
     * 
     * @return Current wingsail position in range -100 (full port) to +100 (full starboard) percent
     */
    float get_wingsail() const { return _wingsail; }

    /**
     * @brief Set mast rotation input value
     * 
     * @param[in] mast_rotation Desired mast rotation from -100 (full counter-clockwise) to +100 (full clockwise) percent
     * 
     * @details Sets the mast rotation servo position for sailboats with rotating masts.
     * Mast rotation can improve sail efficiency by optimizing the sail shape and angle
     * of attack relative to the wind.
     * 
     * @note Only applicable for sailboat vehicles with rotating mast configuration
     */
    void set_mast_rotation(float mast_rotation);

    /**
     * @brief Get current mast rotation input value
     * 
     * @return Current mast rotation in range -100 (full counter-clockwise) to +100 (full clockwise) percent
     */
    float get_mast_rotation() const { return _mast_rotation; }

    /**
     * @brief Get slew rate limited throttle value
     * 
     * @param[in] throttle Desired throttle from -100 to +100 percent
     * @param[in] dt Time delta in seconds since last call (typically main loop period)
     * @return Slew rate limited throttle value in range -100 to +100 percent
     * 
     * @details Calculates throttle with slew rate limiting applied, but does NOT update
     * internal throttle state. This is useful for manual modes to preview the slew-limited
     * throttle without affecting autonomous mode throttle tracking.
     * 
     * Slew rate limiting prevents abrupt throttle changes, particularly important during
     * forward-to-reverse transitions where instantaneous reversals could cause:
     * - Mechanical stress on drivetrain
     * - Loss of steering control during transition
     * - Vehicle instability
     * 
     * The slew rate is configured via parameters (percent change per second).
     * 
     * @note This is a const method that does not modify internal state
     * @note Used primarily by manual mode to avoid bad steering behavior during reversals
     */
    float get_slew_limited_throttle(float throttle, float dt) const;

    /**
     * @brief Check if vehicle is capable of skid steering
     * 
     * @return true if vehicle uses skid steering (differential drive), false otherwise
     * 
     * @details Returns true if the vehicle is configured for skid steering operation
     * where left and right motors are controlled independently to achieve steering.
     * Skid steering vehicles have:
     * - Separate left and right throttle channels
     * - Steering achieved by differential motor speeds
     * - Ability to turn in place (zero radius turns)
     * 
     * Examples: tank-style rovers, differential drive robots.
     * 
     * @note Non-skid-steering vehicles use separate steering servo and throttle motor
     */
    bool have_skid_steering() const;

    /**
     * @brief Check if vehicle has vectored thrust capability
     * 
     * @return true if vehicle uses vectored thrust, false otherwise
     * 
     * @details Returns true if the vehicle has vectored thrust configuration where the
     * thrust direction is controlled by the steering servo angle. Typical for boats
     * where the motor/propeller is mounted on a steering servo and can rotate to
     * direct thrust at various angles.
     * 
     * Vectored thrust vehicles:
     * - Have motor mounted on steerable mechanism
     * - Can achieve lateral motion through thrust vectoring
     * - Steering angle affects thrust vector and vehicle motion
     * 
     * Configuration is determined by _vector_angle_max parameter being positive.
     * 
     * @note Regular boats without vectoring have fixed motor direction
     */
    bool have_vectored_thrust() const { return is_positive(_vector_angle_max); }

    /**
     * @brief Output motor and steering servo commands
     * 
     * @param[in] armed true if vehicle is armed and motors should be active, false to output zero/disarmed values
     * @param[in] ground_speed Vehicle's speed over the surface in m/s (used for speed-based steering scaling)
     * @param[in] dt Time delta in seconds since last call (typically main loop period, ~0.02s for 50Hz)
     * 
     * @details This is the main output function that sends motor and servo commands to hardware.
     * Called at the main loop rate (typically 50Hz for ground vehicles) to apply the desired
     * throttle, steering, and lateral inputs to the appropriate output channels.
     * 
     * Processing performed:
     * - Applies steering/throttle mixing based on vehicle type (regular, skid-steer, omni)
     * - Performs slew rate limiting on throttle to prevent abrupt changes
     * - Applies thrust curve scaling for non-linear motor response
     * - Implements speed-based steering scaling for better high-speed handling
     * - Applies output limits and safety constraints
     * - Handles forward/reverse transition delays
     * - Outputs appropriate PWM signals based on configured PWM type
     * 
     * Vehicle-specific output methods called:
     * - Regular: output_regular() for separate steering servo and throttle motor
     * - Skid-steering: output_skid_steering() for differential drive
     * - Omni: output_omni() for omnidirectional motor mixing
     * - Sailboat: output_sail() for sail control servos
     * 
     * @note This is called at main loop rate and should execute quickly
     * @warning If armed is false, motors will output disarmed PWM values (typically stopped)
     * @warning Ensure proper arming checks before calling with armed=true
     * 
     * @see set_throttle(), set_steering(), set_lateral()
     */
    void output(bool armed, float ground_speed, float dt);

    /**
     * @brief Test motor or servo output using percentage value
     * 
     * @param[in] motor_seq Which motor/servo to test (from motor_test_order enum)
     * @param[in] pct Test output level from -100 to +100 percent
     * @return true if test output was successful, false if motor_seq invalid or not applicable
     * 
     * @details Outputs a test command to a specific motor or servo channel for verification
     * purposes. Used in response to MAVLink DO_MOTOR_TEST command from ground control station.
     * 
     * Allows testing:
     * - MOTOR_TEST_THROTTLE: Main throttle output (regular vehicles)
     * - MOTOR_TEST_STEERING: Steering servo output
     * - MOTOR_TEST_THROTTLE_LEFT: Left motor (skid-steer)
     * - MOTOR_TEST_THROTTLE_RIGHT: Right motor (skid-steer)
     * - MOTOR_TEST_MAINSAIL: Mainsail servo (sailboats)
     * 
     * @note Vehicle must be armed for motor test to take effect (safety requirement)
     * @warning Use caution when testing motors - vehicle may move unexpectedly
     * @warning Ensure vehicle is properly supported/restrained before testing
     */
    bool output_test_pct(motor_test_order motor_seq, float pct);

    /**
     * @brief Test motor or servo output using PWM value
     * 
     * @param[in] motor_seq Which motor/servo to test (from motor_test_order enum)
     * @param[in] pwm Test output PWM value in microseconds (typically 1000-2000)
     * @return true if test output was successful, false if motor_seq invalid or not applicable
     * 
     * @details Outputs a test PWM command directly to a specific motor or servo channel.
     * Provides low-level control for testing exact PWM values. Used for verifying correct
     * servo ranges and motor calibration.
     * 
     * @note Vehicle must be armed for motor test to take effect (safety requirement)
     * @warning Direct PWM control bypasses safety limits - use with extreme caution
     * @warning Ensure vehicle is properly supported/restrained before testing
     */
    bool output_test_pwm(motor_test_order motor_seq, float pwm);

    /**
     * @brief Perform pre-arm safety checks
     * 
     * @param[in] report true to send text messages to GCS about failures, false for silent check
     * @return true if all checks pass and vehicle is safe to arm, false if any check fails
     * 
     * @details Performs comprehensive safety validation before allowing vehicle to arm.
     * Checks include:
     * - Motor output channels configured correctly
     * - Steering servo range configured for vehicle type
     * - PWM type configuration valid for selected outputs
     * - No conflicting parameter settings
     * - Servo output functions assigned correctly
     * 
     * If report is true and checks fail, sends detailed failure messages to ground
     * control station to help diagnose configuration issues.
     * 
     * @note Called by the arming system before allowing vehicle to arm
     * @warning Vehicle should not be armed if this check fails
     * 
     * @see AP_Arming for overall arming check system
     */
    bool pre_arm_check(bool report) const;

    /**
     * @brief Get motor output channel mask
     * 
     * @return Bitmask of motor output channels (bit set = channel used)
     * 
     * @details Returns a bitmask indicating which servo output channels are configured
     * for motor control. Used by the servo output system to identify motor channels for
     * special handling (e.g., for digital PWM protocols).
     * 
     * Bit positions correspond to servo output channel numbers.
     */
    uint32_t get_motor_mask() const { return _motor_mask; }

    /**
     * @brief Check if PWM type is digital protocol
     * 
     * @return true if using digital PWM protocol (DShot, OneShot), false for analog (Normal PWM)
     * 
     * @details Digital PWM protocols (DShot150/300/600/1200, OneShot, OneShot125) have
     * different characteristics than traditional analog PWM:
     * - Fixed endpoint calibration (no ESC calibration needed)
     * - Higher update rates
     * - Built-in CRC for noise immunity (DShot)
     * - Bidirectional telemetry support (DShot)
     * 
     * This affects servo output configuration and calibration procedures.
     */
    bool is_digital_pwm_type() const;

    /**
     * @brief Check if vehicle is configured as omni-directional
     * 
     * @return true if vehicle has omni-directional capability, false otherwise
     * 
     * @details Returns true if the vehicle is configured with an omni-directional frame
     * type (OMNI3, OMNIX, OMNIPLUS, OMNI3MECANUM) and has motors configured.
     * 
     * Omni-directional vehicles can move:
     * - Forward/backward (throttle)
     * - Left/right sideways (lateral)
     * - Rotate in place (steering)
     * - Any combination of the above simultaneously
     * 
     * @note Regular and skid-steering vehicles return false
     */
    bool is_omni() const { return _frame_type != FRAME_TYPE_UNDEFINED && _motors_num > 0; }

    /**
     * @brief Get legacy relay indices for parameter conversion
     * 
     * @param[out] index1 First relay index (if used)
     * @param[out] index2 Second relay index (if used)
     * @param[out] index3 Third relay index (if used)
     * @param[out] index4 Fourth relay index (if used)
     * @return true if legacy relay configuration exists, false otherwise
     * 
     * @details Retrieves relay indices that were used in legacy parameter configurations
     * for brushed motor direction control. Used during parameter migration to convert
     * old relay-based configurations to new SRV_Channel relay functions.
     * 
     * @note This is for parameter migration support and backward compatibility
     */
    bool get_legacy_relay_index(int8_t &index1, int8_t &index2, int8_t &index3, int8_t &index4) const;

    /**
     * @struct AP_MotorsUGV_limit
     * @brief Motor and steering limit flags
     * 
     * @details Structure containing flags that indicate when motor or steering outputs
     * have reached their configured limits. Used by control systems to:
     * - Prevent integral windup in controllers when at limits
     * - Provide feedback to navigation systems about control authority
     * - Implement anti-windup in steering and throttle controllers
     * 
     * Limit flags are set during output() processing when:
     * - Steering reaches maximum left or right angle
     * - Throttle reaches minimum or maximum configured values
     * - Speed controller reaches maximum achievable velocity
     * 
     * Controllers should check these flags and disable integral accumulation
     * when limits are reached to prevent windup.
     */
    struct AP_MotorsUGV_limit {
        uint8_t steer_left      : 1; ///< Steering at maximum left limit
        uint8_t steer_right     : 1; ///< Steering at maximum right limit
        uint8_t throttle_lower  : 1; ///< Throttle at minimum limit (maximum reverse or minimum forward)
        uint8_t throttle_upper  : 1; ///< Throttle at maximum limit (maximum forward)
    } limit;

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

private:

    /**
     * @enum PWMType
     * @brief PWM output protocol types for motor control
     * 
     * @details Defines the PWM signaling protocol used to control motors and ESCs.
     * Different protocols have different update rates, calibration requirements,
     * and feature support.
     */
    enum PWMType {
        NORMAL = 0,              ///< Standard analog PWM (50-490Hz), requires ESC calibration
        ONESHOT = 1,             ///< OneShot125 protocol, 8x faster than standard PWM
        ONESHOT125 = 2,          ///< OneShot125 protocol (same as ONESHOT)
        BRUSHED_WITH_RELAY = 3,  ///< Brushed motor with relay for direction control
        BRUSHED_BIPOLAR = 4,     ///< Brushed motor with bipolar H-bridge (no relay needed)
        DSHOT150 = 5,            ///< DShot digital protocol at 150kHz (robust, low latency)
        DSHOT300 = 6,            ///< DShot digital protocol at 300kHz (standard speed)
        DSHOT600 = 7,            ///< DShot digital protocol at 600kHz (high speed)
        DSHOT1200 = 8            ///< DShot digital protocol at 1200kHz (maximum speed)
    };

    // sanity check parameters
    void sanity_check_parameters();

    // setup pwm output type
    void setup_pwm_type();

    // setup for frames with omni motors
    void setup_omni();

    // add omni motor using separate throttle, steering and lateral factors
    void add_omni_motor(int8_t motor_num, float throttle_factor, float steering_factor, float lateral_factor);

    // add a motor and set up output function
    void add_omni_motor_num(int8_t motor_num);

    // disable omni motor and remove all throttle, steering and lateral factor for this motor
    void clear_omni_motors(int8_t motor_num);

    // output to regular steering and throttle channels
    void output_regular(bool armed, float ground_speed, float steering, float throttle);

    // output to skid steering channels
    void output_skid_steering(bool armed, float steering, float throttle, float dt);

    // output for omni motors
    void output_omni(bool armed, float steering, float throttle, float lateral);

    // output throttle (-100 ~ +100) to a throttle channel.  Sets relays if required
    // dt is the main loop time interval and is required when rate control is required
    void output_throttle(SRV_Channel::Function function, float throttle, float dt = 0.0f);

    // output for sailboat's mainsail in the range of 0 to 100 and wing sail in the range +- 100
    void output_sail();

    // true if the vehicle has a mainsail or wing sail
    bool has_sail() const;

    // slew limit throttle for one iteration
    void slew_limit_throttle(float dt);

    // set limits based on steering and throttle input
    void set_limits_from_input(bool armed, float steering, float throttle);

    // scale a throttle using the _thrust_curve_expo parameter.  throttle should be in the range -100 to +100
    float get_scaled_throttle(float throttle) const;

    // use rate controller to achieve desired throttle
    float get_rate_controlled_throttle(SRV_Channel::Function function, float throttle, float dt);

    // external references
    AP_WheelRateControl &_rate_controller;

    static const int8_t AP_MOTORS_NUM_MOTORS_MAX = 4;

    // parameters
    AP_Int8 _pwm_type;  // PWM output type
    AP_Int8 _pwm_freq;  // PWM output freq for brushed motors
    AP_Int8 _disarm_disable_pwm;    // disable PWM output while disarmed
    AP_Int16 _slew_rate; // slew rate expressed as a percentage / second
    AP_Int8 _throttle_min; // throttle minimum percentage
    AP_Int8 _throttle_max; // throttle maximum percentage
    AP_Float _thrust_curve_expo; // thrust curve exponent from -1 to +1 with 0 being linear
    AP_Float _thrust_asymmetry; // asymmetry factor, how much better your skid-steering motors are at going forward than backwards (forward/backward thrust ratio)
    AP_Float _vector_angle_max;  // angle between steering's middle position and maximum position when using vectored thrust.  zero to disable vectored thrust
    AP_Float _speed_scale_base;  // speed above which steering is scaled down when using regular steering/throttle vehicles.  zero to disable speed scaling
    AP_Float _steering_throttle_mix; // Steering vs Throttle priorisation.  Higher numbers prioritise steering, lower numbers prioritise throttle.  Only valid for Skid Steering vehicles
    AP_Float _reverse_delay; // delay in seconds when reversing motor

    // internal variables
    float   _steering;  // requested steering as a value from -4500 to +4500
    float   _throttle;  // requested throttle as a value from -100 to 100
    float   _throttle_prev; // throttle input from previous iteration
    bool    _scale_steering = true; // true if we should scale steering by speed or angle
    float   _lateral;  // requested lateral input as a value from -100 to +100
    float   _roll;      // requested roll as a value from -1 to +1
    float   _pitch;     // requested pitch as a value from -1 to +1
    float   _walking_height; // requested height as a value from -1 to +1   
    float   _mainsail;  // requested mainsail input as a value from 0 to 100
    float   _wingsail;  // requested wing sail input as a value in the range +- 100
    float   _mast_rotation;  // requested mast rotation input as a value in the range +- 100
    uint32_t _motor_mask;   // mask of motors configured with pwm_type
    frame_type _frame_type; // frame type requested at initialisation

    // omni variables
    float   _throttle_factor[AP_MOTORS_NUM_MOTORS_MAX];
    float   _steering_factor[AP_MOTORS_NUM_MOTORS_MAX];
    float   _lateral_factor[AP_MOTORS_NUM_MOTORS_MAX];
    uint8_t   _motors_num;

    /*
      3 reversal handling structures, for k_throttle, k_throttleLeft and k_throttleRight
     */
    struct ReverseThrottle {
        float last_throttle;
        uint32_t last_output_ms;

        // output with delay for reversal
        void output(SRV_Channel::Function function, float throttle, float delay);
    } rev_delay_throttle, rev_delay_throttleLeft, rev_delay_throttleRight;

    static AP_MotorsUGV *_singleton;
};

namespace AP {
    AP_MotorsUGV *motors_ugv();
};
