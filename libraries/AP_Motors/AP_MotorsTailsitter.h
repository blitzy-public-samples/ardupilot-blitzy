/**
 * @file AP_MotorsTailsitter.h
 * @brief Motor control class for tailsitter VTOL aircraft
 * 
 * @details This file implements motor control for tailsitter VTOL configurations,
 *          which use multicopter-style motors for both vertical takeoff/landing
 *          and forward flight. Tailsitters transition between vertical orientation
 *          (multicopter mode) and horizontal orientation (airplane mode) by rotating
 *          the entire aircraft frame.
 * 
 *          Key characteristics:
 *          - Motors provide thrust in both vertical and horizontal flight
 *          - Differential thrust used for yaw control in hover
 *          - Optional tilt servos for thrust vectoring in forward flight
 *          - Smooth transition between multicopter and airplane control regimes
 *          - Supports bicopter (2 motors) and quadcopter (4 motors) configurations
 * 
 *          Control authority:
 *          - Hover mode: Differential thrust for yaw, body tilt for pitch/roll
 *          - Forward flight: Control surfaces for pitch/roll/yaw, motors for thrust
 *          - Transition: Blended control with both motors and surfaces active
 * 
 * Source: libraries/AP_Motors/AP_MotorsTailsitter.h
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsMulticopter.h"

/**
 * @class AP_MotorsTailsitter
 * @brief Motor control for tailsitter VTOL aircraft with transitioning thrust orientation
 * 
 * @details This class manages motor control for tailsitter VTOL aircraft, which use
 *          multicopter-style motors that transition between vertical thrust (for hover)
 *          and horizontal thrust (for forward flight) by rotating the entire airframe.
 * 
 *          Dual Mode Operation:
 *          - **Vertical Flight**: Aircraft oriented vertically, motors point upward.
 *            Uses differential thrust for yaw control, body tilt for pitch/roll control.
 *            Operates similar to standard multicopter control laws.
 *          
 *          - **Forward Flight**: Aircraft oriented horizontally, motors provide forward thrust.
 *            Control surfaces (elevons, rudder) provide pitch/roll/yaw authority.
 *            Motors provide constant thrust with throttle adjustments.
 * 
 *          Tilt Mechanism:
 *          - Optional tilt servos can vector motor thrust for enhanced control
 *          - Tilt vectoring provides pitch and yaw authority in forward flight
 *          - Left and right motors can tilt independently for differential control
 *          - Tilt angles typically limited to prevent motor interference
 * 
 *          Transition Management:
 *          - Smooth handoff between multicopter and airplane control regimes
 *          - Critical phase requiring careful thrust and control surface blending
 *          - Thrust scheduling ensures adequate lift during attitude changes
 *          - Q_TAILSIT_THSCMX parameter limits maximum thrust scaling
 * 
 *          Supported Configurations:
 *          - **Bicopter**: 2 motors with differential thrust for yaw in hover
 *          - **Quadcopter**: 4 motors with standard multicopter mixing
 * 
 * @note Bicopter configuration uses differential thrust for all yaw control in hover,
 *       making yaw authority directly proportional to available thrust margin.
 * 
 * @note Quadcopter configuration provides better redundancy and control authority
 *       but requires more complex motor mixing during transitions.
 * 
 * @note Tilt servos are optional but significantly improve control authority and
 *       efficiency in forward flight by allowing thrust vectoring.
 * 
 * @note Transition phase requires extensive tuning - vehicle must maintain adequate
 *       thrust while changing orientation from vertical to horizontal.
 * 
 * @note Q_TAILSIT_THSCMX parameter controls maximum thrust scaling during transitions
 *       to prevent excessive motor output that could destabilize the vehicle.
 * 
 * @warning CRITICAL: Transition phase is safety-critical. Loss of adequate thrust
 *          during transition can result in loss of control and crash. Always test
 *          transitions at safe altitude with sufficient battery capacity.
 * 
 * @warning Motor failures are more critical in tailsitters than conventional VTOL
 *          because motors provide both lift and forward thrust. Single motor failure
 *          in bicopter is non-recoverable. Quadcopter has limited failure tolerance.
 * 
 * @warning Thrust vectoring servo failures can cause asymmetric thrust in forward
 *          flight, leading to uncontrollable roll or yaw. Servo health monitoring
 *          and mechanical safety limits are essential.
 * 
 * @warning Requires extensive tuning for stable transitions. Poor tuning can result
 *          in oscillations, loss of altitude, or complete loss of control during
 *          the transition phase. Always tune conservatively and incrementally.
 * 
 * @warning Wind conditions significantly affect transition performance. High winds
 *          can make transitions difficult or impossible. Always check wind limits
 *          before attempting transitions in real flight conditions.
 * 
 * @see AP_MotorsMulticopter Base class for multicopter motor control
 * @see Q_TAILSIT_THSCMX Thrust scaling maximum parameter in QuadPlane
 */
class AP_MotorsTailsitter : public AP_MotorsMulticopter {
public:

    /**
     * @brief Constructor for tailsitter motor controller
     * 
     * @details Initializes the tailsitter motor control system with specified
     *          update rate. Sets up motor output channels and tilt servo channels
     *          for thrust vectoring if configured.
     * 
     * @param[in] speed_hz Motor update rate in Hz (default: AP_MOTORS_SPEED_DEFAULT)
     *                     Typical values: 50Hz for analog servos, 400Hz for digital ESCs
     * 
     * @note Higher update rates provide better control response but may not be
     *       supported by all ESC types. Match speed_hz to ESC capabilities.
     */
    AP_MotorsTailsitter(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT);

    /**
     * @brief Initialize motors and tilt servos for tailsitter configuration
     * 
     * @details Initializes the motor control system based on frame configuration.
     *          Sets up motor output channels, configures differential thrust detection,
     *          and initializes tilt servo channels if thrust vectoring is enabled.
     *          
     *          Initialization sequence:
     *          1. Configure motor output channels based on frame type
     *          2. Detect if differential thrust is available (bicopter vs quadcopter)
     *          3. Initialize tilt servo channels if configured
     *          4. Set initial motor and servo positions to safe values
     *          5. Configure motor mixing matrices for hover mode
     * 
     * @param[in] frame_class Motor frame class (e.g., MOTOR_FRAME_TAILSITTER)
     * @param[in] frame_type  Frame configuration type (bicopter or quadcopter layout)
     * 
     * @note Bicopter frame_type sets _has_diff_thrust = true for yaw control
     * @note Quadcopter frame_type uses standard quad motor mixing
     * @note Tilt servo channels initialized only if Q_TAILSIT_VECTHR is configured
     * @note This method must be called before any motor output operations
     * 
     * @warning Incorrect frame_class or frame_type configuration will result in
     *          improper motor mixing and potential loss of control. Verify configuration
     *          matches physical vehicle before flight testing.
     */
    void init(motor_frame_class frame_class, motor_frame_type frame_type) override;

    /**
     * @brief Set frame class and type (not used for tailsitters)
     * 
     * @details This method is overridden but not implemented for tailsitters
     *          because frame configuration is set during init() and cannot be
     *          changed at runtime. Tailsitter frame configuration is fixed and
     *          determined by hardware configuration.
     * 
     * @param[in] frame_class Motor frame class (unused)
     * @param[in] frame_type  Frame type (unused)
     * 
     * @note Empty implementation - frame configuration is immutable after init()
     * @note Use init() method to set initial frame configuration
     */
    void set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override {}

    /**
     * @brief Set motor and servo update rate
     * 
     * @details Configures the PWM update rate for motor ESCs and tilt servos.
     *          Higher rates provide better control response but must match ESC
     *          and servo capabilities. Rate applies to both motor outputs and
     *          tilt servo outputs.
     * 
     * @param[in] speed_hz Update rate in Hertz
     *                     - 50Hz: Standard analog servos
     *                     - 200Hz: High-speed digital servos
     *                     - 400Hz: Modern digital ESCs
     * 
     * @note ESCs and servos must support the configured rate
     * @note Mismatch between rate and hardware capabilities can cause jitter or no response
     * @note Rate change takes effect on next output cycle
     */
    void set_update_rate( uint16_t speed_hz ) override;

    /**
     * @brief Output motor thrust and tilt servo positions to hardware
     * 
     * @details Sends calculated motor thrust values and tilt servo positions to
     *          the physical motor ESCs and servos. Applies safety limits, thrust
     *          curves, and servo range constraints before output.
     *          
     *          Output sequence:
     *          1. Apply thrust scaling and minimum throttle limits
     *          2. Constrain motor outputs to safe ranges (0.0 to 1.0)
     *          3. Calculate tilt servo positions from _tilt_left and _tilt_right
     *          4. Apply servo range limits and reversals
     *          5. Convert to PWM values and send to hardware outputs
     *          6. Update motor telemetry if logging enabled
     * 
     * @note Called by main control loop after output_armed_stabilizing() calculates desired outputs
     * @note Applies _external_min_throttle if set by diskloading calculations
     * @note Tilt servos only updated if thrust vectoring is configured
     * @note All outputs constrained to safe ranges regardless of calculated values
     * 
     * @warning This method must be called at consistent rate (main loop rate) for
     *          stable motor control. Irregular calling can cause motor jitter and
     *          control oscillations.
     */
    void output_to_motors() override;

    /**
     * @brief Get bitmask of output channels used for motors and tilt servos
     * 
     * @details Returns a bitmask indicating which PWM output channels are allocated
     *          to motors and tilt servos for this tailsitter configuration. Used to
     *          prevent conflicts with other PWM outputs (e.g., control surface servos,
     *          camera gimbals, auxiliary functions).
     *          
     *          Bitmask format: Bit N set (1) means output channel N is in use
     *          Example for bicopter with tilt servos:
     *          - Bits 0-1: Motor outputs (left and right)
     *          - Bits 2-3: Tilt servo outputs (left and right)
     *          - Remaining bits: Available for other functions
     * 
     * @return Bitmask of used output channels (1 = in use, 0 = available)
     *         Bit position corresponds to output channel number
     * 
     * @note Bicopter uses 2 motor channels, quadcopter uses 4 motor channels
     * @note Tilt servos add 2 additional channels if thrust vectoring configured
     * @note This mask must be checked before assigning other servo functions to
     *       prevent output conflicts that could cause control interference
     * 
     * @see SRV_Channel for servo output management
     */
    uint32_t get_motor_mask() override;

    /**
     * @brief Set minimum throttle based on diskloading calculations
     * 
     * @details Sets external minimum throttle limit based on diskloading theory
     *          minimum outflow velocity calculations. This ensures motors maintain
     *          sufficient thrust to prevent vortex ring state and maintain control
     *          authority during transitions and low-speed flight.
     *          
     *          Diskloading calculation determines minimum required thrust to maintain
     *          adequate propeller wash velocity and prevent aerodynamic instabilities.
     *          This limit is dynamically calculated based on vehicle mass, propeller
     *          disk area, and current flight conditions.
     * 
     * @param[in] val Minimum throttle value (0.0 to 1.0) from diskloading calculations
     *                Values typically range from 0.1 to 0.3 depending on vehicle configuration
     * 
     * @note Used during transitions to prevent insufficient motor authority
     * @note Value is continuously updated based on flight conditions
     * @note Applied as lower bound in output_to_motors() before sending to ESCs
     * @note Essential for preventing loss of control during low-speed maneuvering
     * 
     * @warning Setting this value too low can result in vortex ring state or loss
     *          of control during transitions. Trust diskloading calculations.
     */
    void set_min_throttle(float val) {_external_min_throttle = val;}

protected:
    /**
     * @brief Calculate motor thrust and tilt servo outputs for stabilization
     * 
     * @details Calculates motor thrust and tilt servo positions based on pilot
     *          input and stabilization demands from attitude controllers. Implements
     *          the core motor mixing and control allocation for tailsitter flight.
     *          
     *          Calculation sequence:
     *          1. Read desired roll/pitch/yaw rates and throttle from attitude controller
     *          2. Apply motor mixing based on frame type (bicopter or quadcopter)
     *          3. Calculate differential thrust for yaw control if available
     *          4. Compute tilt servo positions for thrust vectoring
     *          5. Apply thrust scaling and limits
     *          6. Store results in _thrust_left, _thrust_right, _tilt_left, _tilt_right
     *          
     *          Control modes:
     *          - Hover: Differential thrust for yaw, body tilt for pitch/roll
     *          - Transition: Blended motor and control surface authority
     *          - Forward: Constant thrust with tilt vectoring for trim
     * 
     * @note Called by main control loop before output_to_motors()
     * @note Results stored in member variables for output_to_motors() to send to hardware
     * @note Applies Q_TAILSIT_THSCMX thrust scaling limit during transitions
     * @note Differential thrust only used if _has_diff_thrust is true
     * 
     * @warning This is called at main loop rate (typically 400Hz) and must execute
     *          quickly to maintain real-time control performance.
     */
    void output_armed_stabilizing() override;

    /**
     * @brief Get frame type string for logging and display
     * 
     * @details Returns human-readable frame type identifier for telemetry,
     *          logging, and ground station display purposes.
     * 
     * @return Frame type string "TAILSITTER"
     * 
     * @note Used in log messages and parameter documentation
     * @note Helps identify vehicle type in multi-vehicle systems
     */
    const char* _get_frame_string() const override { return "TAILSITTER"; }

    /**
     * @brief Output test sequence for individual motor testing
     * 
     * @details Sends specified PWM value to individual motor for ground testing
     *          and motor identification. Used during pre-flight checks and setup
     *          to verify motor ordering and direction.
     *          
     *          Safety features:
     *          - Only operates when vehicle is disarmed
     *          - Supports motor sequence identification (which motor is which)
     *          - Allows verification of motor rotation direction
     *          - Used by motor test functions in GCS
     * 
     * @param[in] motor_seq Motor sequence number (0-based index)
     *                      - Bicopter: 0 (left), 1 (right)
     *                      - Quadcopter: 0-3 (standard quad ordering)
     * @param[in] pwm PWM output value in microseconds (typically 1000-2000)
     * 
     * @note Vehicle must be disarmed for motor test to function
     * @note Use carefully - motors will spin at specified power
     * @note Verify motor sequence matches physical configuration before flight
     * 
     * @warning SAFETY CRITICAL: Only use motor test with propellers removed or
     *          vehicle securely restrained. Motor test can cause unexpected motor
     *          activation. Always verify motor_seq matches physical layout.
     */
    void _output_test_seq(uint8_t motor_seq, int16_t pwm) override;

    // Calculated motor and servo outputs
    
    /**
     * @brief Main throttle demand (0.0 to 1.0)
     * @details Base throttle input from pilot or altitude controller
     *          Used as baseline for left/right thrust calculations
     */
    float _throttle; // 0..1
    
    /**
     * @brief Left motor tilt servo position (-1.0 to 1.0)
     * @details Controls left motor thrust vectoring angle
     *          - -1.0: Maximum tilt in one direction
     *          - 0.0: Neutral (thrust aligned with motor mount)
     *          - +1.0: Maximum tilt in opposite direction
     * @note Only used if thrust vectoring is configured (Q_TAILSIT_VECTHR)
     */
    float _tilt_left;  // -1..1
    
    /**
     * @brief Right motor tilt servo position (-1.0 to 1.0)
     * @details Controls right motor thrust vectoring angle
     *          - -1.0: Maximum tilt in one direction
     *          - 0.0: Neutral (thrust aligned with motor mount)
     *          - +1.0: Maximum tilt in opposite direction
     * @note Only used if thrust vectoring is configured (Q_TAILSIT_VECTHR)
     */
    float _tilt_right;  // -1..1
    
    /**
     * @brief Left motor thrust output (0.0 to 1.0)
     * @details Calculated thrust for left motor(s) after mixing and limits
     *          - 0.0: Motor stopped or minimum throttle
     *          - 1.0: Maximum thrust
     * @note For bicopter: left motor thrust
     * @note For quadcopter: combined thrust for left motors
     */
    float _thrust_left;  // 0..1
    
    /**
     * @brief Right motor thrust output (0.0 to 1.0)
     * @details Calculated thrust for right motor(s) after mixing and limits
     *          - 0.0: Motor stopped or minimum throttle
     *          - 1.0: Maximum thrust
     * @note For bicopter: right motor thrust
     * @note For quadcopter: combined thrust for right motors
     */
    float _thrust_right;  // 0..1

    /**
     * @brief External minimum throttle from diskloading calculations (0.0 to 1.0)
     * @details Minimum throttle value calculated from diskloading theory to ensure
     *          adequate propeller outflow velocity. Prevents vortex ring state and
     *          maintains control authority during transitions and low-speed flight.
     *          
     *          Set by set_min_throttle() based on:
     *          - Vehicle mass and propeller disk area
     *          - Current airspeed and flight phase
     *          - Aerodynamic efficiency requirements
     * 
     * @note Applied as lower bound in motor output calculations
     * @note Dynamically updated based on flight conditions
     * @note Critical for safe transition performance
     */
    float _external_min_throttle;

    /**
     * @brief Differential thrust availability flag
     * @details True if vehicle configuration supports differential thrust for yaw control
     *          - Bicopter: true (yaw via differential thrust)
     *          - Quadcopter: false (yaw via motor mixing)
     * 
     * @note Determines control allocation strategy in hover mode
     * @note Set during init() based on frame_type
     * @note Affects yaw control authority and response
     */
    bool _has_diff_thrust;

};
