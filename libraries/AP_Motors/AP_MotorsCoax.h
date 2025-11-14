/**
 * @file AP_MotorsCoax.h
 * @brief Motor and Servo control class for coaxial counter-rotating helicopters
 * 
 * @details This file implements motor control for coaxial helicopter configurations
 *          featuring two counter-rotating rotors mounted on the same axis (one above
 *          the other) plus four control surfaces (flaps or elevons) for cyclic control.
 *          
 *          Configuration:
 *          - Upper rotor: Rotates counter-clockwise (CCW) when viewed from above
 *          - Lower rotor: Rotates clockwise (CW) when viewed from above
 *          - Control surfaces: 4 servos providing roll/pitch control via differential deflection
 *          
 *          The coaxial design eliminates the need for a tail rotor since the counter-rotating
 *          rotors cancel out torque. Yaw control is achieved through differential collective
 *          (speed difference between upper and lower rotors). Roll and pitch are controlled
 *          via the four control surfaces arranged around the rotor mast.
 *          
 *          This is a mechanically simpler and more compact design compared to conventional
 *          single-rotor helicopters, with improved yaw authority and reduced mechanical
 *          complexity.
 * 
 * @note Motor rotation direction is critical - reversed motor directions will cause
 *       immediate loss of control and potential vehicle damage.
 * 
 * @warning Both motors must spool up synchronously. Loss of one motor causes severe
 *          yaw instability and requires immediate emergency landing.
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include "AP_MotorsMulticopter.h"

// feedback direction
#define AP_MOTORS_COAX_POSITIVE      1
#define AP_MOTORS_COAX_NEGATIVE     -1

#define NUM_ACTUATORS 4

#define AP_MOTORS_SINGLE_SPEED_DIGITAL_SERVOS 250 // update rate for digital servos
#define AP_MOTORS_SINGLE_SPEED_ANALOG_SERVOS 125  // update rate for analog servos

#define AP_MOTORS_COAX_SERVO_INPUT_RANGE    4500    // roll or pitch input of -4500 will cause servos to their minimum (i.e. radio_min), +4500 will move them to their maximum (i.e. radio_max)

/**
 * @class AP_MotorsCoax
 * @brief Motor control implementation for coaxial counter-rotating helicopter with four control surfaces
 * 
 * @details This class provides motor mixing and control for coaxial helicopter configurations.
 *          The vehicle uses two counter-rotating rotors mounted coaxially (on the same vertical
 *          axis) plus four servo-driven control surfaces for cyclic pitch/roll control.
 *          
 *          Physical Configuration:
 *          - Motor 1 (Upper): Counter-clockwise (CCW) rotation when viewed from above
 *          - Motor 2 (Lower): Clockwise (CW) rotation when viewed from above  
 *          - Servos 1-4: Control surfaces positioned around the rotor mast
 *          
 *          Control Strategy:
 *          - Collective (altitude): Both motors increase/decrease thrust together
 *          - Yaw: Differential thrust between upper and lower rotors
 *                 (increasing upper rotor speed relative to lower creates CCW yaw)
 *          - Roll/Pitch: Four control surfaces deflect differentially to create cyclic effect
 *                        Control surface deflection range: ±4500 centidegrees
 *          
 *          Torque Cancellation:
 *          Counter-rotating rotors naturally cancel torque reactions, eliminating the need
 *          for a tail rotor. This provides several advantages:
 *          - Mechanical simplicity (no tail rotor drive system)
 *          - Improved yaw authority at all airspeeds
 *          - More compact airframe design
 *          - Better hover efficiency (no power lost to tail rotor)
 *          
 *          The mixing algorithm distributes pilot inputs across the six actuators
 *          (2 motors + 4 servos) to achieve desired vehicle motion while maintaining
 *          rotor synchronization and control surface coordination.
 * 
 * @note NUM_ACTUATORS is set to 4 for the control surfaces. Motors are handled separately
 *       through the inherited multicopter motor management system.
 * 
 * @note Upper rotor must rotate CCW and lower rotor CW (viewed from above). Reversed
 *       motor directions will cause immediate instability as the torque cancellation
 *       becomes torque addition.
 * 
 * @warning Motor direction is safety-critical. Verify motor rotation direction during
 *          initial setup before attempting flight. Incorrect rotation will cause immediate
 *          loss of control and likely vehicle destruction.
 * 
 * @warning Control surface binding or mechanical failure can cause loss of cyclic control.
 *          Always verify full, free servo movement before flight.
 * 
 * @warning Both motors must spool up synchronously. Unequal spool rates will cause yaw
 *          instability during takeoff. Loss of one motor in flight causes severe yaw
 *          instability requiring immediate controlled descent.
 */
class AP_MotorsCoax : public AP_MotorsMulticopter {
public:

    /**
     * @brief Constructor for coaxial helicopter motor controller
     * 
     * @param[in] speed_hz Motor update rate in Hertz (default: AP_MOTORS_SPEED_DEFAULT)
     *                     Typical values: 50Hz for analog servos, 250Hz for digital servos
     * 
     * @note Initializes the base multicopter motor class with the specified update rate.
     *       Actual motor and servo initialization occurs in init() method.
     */
    AP_MotorsCoax(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMulticopter(speed_hz)
    {
    };

    /**
     * @brief Initialize coaxial helicopter motor and servo outputs
     * 
     * @details Configures PWM outputs for the two coaxial motors and four control surface servos.
     *          Sets up motor channels, servo channels, and initializes the mixing algorithm
     *          for distributing control inputs across all actuators.
     *          
     *          This method must be called during vehicle initialization before any motor
     *          commands are sent. It establishes the output mappings and prepares the
     *          hardware abstraction layer for motor and servo control.
     * 
     * @param[in] frame_class Motor frame class designation (should be MOTOR_FRAME_COAX)
     * @param[in] frame_type  Motor frame type/variant identifier
     * 
     * @note This overrides the base class init() to configure coaxial-specific output channels.
     * @note Called once during vehicle startup, typically from Copter::init_ardupilot()
     * 
     * @warning Must complete successfully before attempting to arm or spin motors.
     */
    void                init(motor_frame_class frame_class, motor_frame_type frame_type) override;

    /**
     * @brief Set motor frame class and type configuration
     * 
     * @details Updates the frame class and type identifiers for this motor controller.
     *          For coaxial helicopters, this typically sets MOTOR_FRAME_COAX class.
     *          The frame type parameter can be used to distinguish between different
     *          coaxial variants if multiple configurations are supported.
     * 
     * @param[in] frame_class Motor frame class (e.g., MOTOR_FRAME_COAX)
     * @param[in] frame_type  Frame type/variant within the class
     * 
     * @note This method can be called to reconfigure the frame after initialization,
     *       though this is uncommon during normal operation.
     */
    void                set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override;

    /**
     * @brief Set PWM update rate for motors and servos
     * 
     * @details Configures the PWM output frequency for both motors and control surface servos.
     *          Different update rates are optimal for different actuator types:
     *          - Digital servos: 250Hz (AP_MOTORS_SINGLE_SPEED_DIGITAL_SERVOS)
     *          - Analog servos: 125Hz (AP_MOTORS_SINGLE_SPEED_ANALOG_SERVOS)
     *          - Brushless motors: Typically 400Hz or higher
     *          
     *          The update rate affects servo responsiveness and motor control precision.
     *          Higher rates provide better control authority but may cause issues with
     *          analog servos or some ESCs. Lower rates are more compatible but reduce
     *          control bandwidth.
     * 
     * @param[in] speed_hz PWM update frequency in Hertz (typical range: 50-490Hz)
     * 
     * @note This method configures the HAL timer for the appropriate PWM frequency.
     *       Changes take effect on the next output cycle.
     * 
     * @note For coaxial helicopters, the same update rate is typically used for both
     *       motors and servos, though the HAL may support different rates per channel.
     * 
     * @warning Setting update rate too high for analog servos can cause servo jitter,
     *          overheating, or reduced lifespan. Verify servo specifications before
     *          using rates above 125Hz with analog servos.
     */
    void                set_update_rate( uint16_t speed_hz ) override;

    /**
     * @brief Send motor and servo commands to hardware outputs
     * 
     * @details Transmits the calculated motor thrust commands and servo deflection commands
     *          to the PWM output channels. This method converts the normalized control values
     *          (stored in _actuator_out, _thrust_yt_ccw, _thrust_yt_cw) into actual PWM
     *          microsecond values and sends them to the motors and servos via the HAL.
     *          
     *          For coaxial helicopters, this outputs:
     *          - Motor 1 (upper, CCW): Throttle + differential yaw component
     *          - Motor 2 (lower, CW): Throttle - differential yaw component
     *          - Servos 1-4: Control surface deflections for roll/pitch cyclic control
     *          
     *          The method handles PWM range scaling, ensuring outputs stay within configured
     *          minimum and maximum PWM values for each channel.
     * 
     * @note Called from output_armed_stabilizing() during normal armed operation, or can be
     *       called directly to send specific output values (e.g., during motor testing).
     * 
     * @note Typical call frequency: Main loop rate (400Hz for copters)
     * 
     * @warning This method directly controls actuators. Incorrect values can cause
     *          violent vehicle motion or loss of control. Always ensure mixing calculations
     *          are complete and valid before calling this method.
     */
    virtual void        output_to_motors() override;

    /**
     * @brief Get bitmask of PWM outputs used by motors and servos
     * 
     * @details Returns a 32-bit bitmask indicating which PWM output channels are currently
     *          allocated to motors and control surface servos for this coaxial helicopter.
     *          Each bit position corresponds to a PWM output channel number, with bit=1
     *          indicating that channel is in use.
     *          
     *          For coaxial helicopters, this typically includes:
     *          - 2 motor outputs (upper and lower rotors)
     *          - 4 servo outputs (control surfaces)
     *          Total: 6 outputs in use
     *          
     *          This information is used to prevent conflicts with auxiliary functions
     *          (cameras, grippers, etc.) that also require PWM outputs. The vehicle
     *          configuration system uses this to ensure no two systems attempt to
     *          control the same output channel.
     * 
     * @return uint32_t Bitmask where bit N=1 indicates output channel N is used
     *                  (bit 0 = channel 1, bit 1 = channel 2, etc.)
     * 
     * @note This is called during vehicle initialization and parameter changes to
     *       validate output channel assignments.
     */
    uint32_t            get_motor_mask() override;

    /**
     * @brief Perform pre-arming safety checks for coaxial helicopter
     * 
     * @details Executes safety validation checks that must pass before the vehicle can be armed.
     *          Uses the base AP_Motors arming checks which verify motor configuration,
     *          output channel setup, and basic safety parameters.
     * 
     * @param[in]  buflen Size of error message buffer in bytes
     * @param[out] buffer Character buffer to receive error messages if checks fail
     * 
     * @return true if all arming checks pass, false if any check fails
     *         (failure reasons written to buffer)
     * 
     * @note Called by the arming system before allowing vehicle to arm.
     */
    bool arming_checks(size_t buflen, char *buffer) const override { return AP_Motors::arming_checks(buflen, buffer); }

protected:
    /**
     * @brief Calculate and output motor/servo commands during armed stabilizing flight
     * 
     * @details Main mixing algorithm that converts pilot inputs and attitude controller
     *          commands into individual motor thrust values and control surface deflections.
     *          This method is called every main loop iteration when the vehicle is armed
     *          and in a stabilizing flight mode.
     *          
     *          The mixing process:
     *          1. Read pilot inputs: roll, pitch, yaw, throttle
     *          2. Calculate differential thrust for yaw control (upper vs lower rotor)
     *          3. Calculate control surface deflections for roll/pitch from pilot input
     *          4. Apply collective thrust to both motors
     *          5. Limit outputs to valid ranges
     *          6. Call output_to_motors() to send PWM commands to hardware
     *          
     *          Roll/pitch control is achieved through differential deflection of the four
     *          control surfaces. Yaw control is achieved through differential collective
     *          between the upper (CCW) and lower (CW) rotors.
     * 
     * @note Called at main loop rate (typically 400Hz) during normal flight operation.
     * 
     * @note This is a protected method called by the base class output() method through
     *       the motor control state machine.
     * 
     * @warning Contains safety-critical mixing calculations. Errors in this algorithm
     *          can cause immediate loss of control. Changes require thorough SITL and
     *          ground testing before flight testing.
     */
    void                output_armed_stabilizing() override;

    /**
     * @brief Calculated control surface deflection outputs for four servos
     * 
     * @details Array of 4 normalized actuator outputs representing the commanded deflection
     *          for each control surface servo. Values are in the range 0.0 to 1.0, where
     *          0.0 represents minimum deflection and 1.0 represents maximum deflection.
     *          
     *          These values are calculated by the mixing algorithm in output_armed_stabilizing()
     *          based on pilot roll/pitch inputs and attitude controller commands. The values
     *          are then converted to PWM microsecond values in output_to_motors().
     *          
     *          Index mapping:
     *          [0] - Servo 1: Control surface 1
     *          [1] - Servo 2: Control surface 2
     *          [2] - Servo 3: Control surface 3
     *          [3] - Servo 4: Control surface 4
     *          
     *          Control surface arrangement provides differential deflection for roll/pitch cyclic control.
     */
    float               _actuator_out[NUM_ACTUATORS];
    
    /**
     * @brief Normalized thrust output for upper rotor (counter-clockwise)
     * 
     * @details Calculated thrust value for the upper CCW rotor in normalized range 0.0 to 1.0.
     *          Includes base collective thrust plus/minus differential component for yaw control.
     *          Computed as: base_throttle + yaw_differential
     *          
     *          Increasing this relative to _thrust_yt_cw produces counter-clockwise (left) yaw
     *          since the upper rotor's torque reaction becomes dominant.
     */
    float               _thrust_yt_ccw;
    
    /**
     * @brief Normalized thrust output for lower rotor (clockwise)
     * 
     * @details Calculated thrust value for the lower CW rotor in normalized range 0.0 to 1.0.
     *          Includes base collective thrust minus/plus differential component for yaw control.
     *          Computed as: base_throttle - yaw_differential
     *          
     *          Increasing this relative to _thrust_yt_ccw produces clockwise (right) yaw
     *          since the lower rotor's torque reaction becomes dominant.
     */
    float               _thrust_yt_cw;

    /**
     * @brief Get string identifier for coaxial frame type
     * 
     * @return const char* Returns "COAX" identifying this as a coaxial helicopter frame
     * 
     * @note Used for logging, parameter naming, and user interface display
     */
    const char* _get_frame_string() const override { return "COAX"; }

    /**
     * @brief Output test PWM sequence to individual motor or servo
     * 
     * @details Sends a specific PWM value to a single motor or servo for testing purposes.
     *          Used during motor test mode to verify correct motor/servo operation, wiring,
     *          and rotation direction before flight. This bypasses the normal mixing algorithm
     *          and directly commands a single actuator.
     *          
     *          For coaxial helicopters:
     *          - motor_seq 1: Upper rotor (CCW) - verify counter-clockwise rotation
     *          - motor_seq 2: Lower rotor (CW) - verify clockwise rotation  
     *          - motor_seq 3-6: Control surface servos 1-4 - verify deflection direction
     *          
     *          This is a safety-critical test function used to detect wiring errors,
     *          incorrect motor rotation, and servo binding before first flight.
     * 
     * @param[in] motor_seq Motor/servo sequence number (1-based indexing: 1 to 6)
     * @param[in] pwm       PWM output value in microseconds (typically 1000-2000μs)
     * 
     * @note Only functional when vehicle is disarmed and motor test mode is enabled.
     * 
     * @warning NEVER run this test with propellers attached to motors. Use only with
     *          propellers removed for initial configuration verification. Even at low
     *          PWM values, spinning propellers can cause serious injury.
     * 
     * @warning Verify motor rotation direction matches expected (upper CCW, lower CW).
     *          Incorrect rotation will cause immediate loss of control if attempted in flight.
     */
    virtual void _output_test_seq(uint8_t motor_seq, int16_t pwm) override;
};
