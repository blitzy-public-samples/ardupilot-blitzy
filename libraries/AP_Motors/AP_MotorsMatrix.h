/**
 * @file AP_MotorsMatrix.h
 * @brief Motor control class for matrix-based multicopter motor mixing
 * 
 * @details This file implements matrix-style motor mixing for standard multicopter frames.
 *          The AP_MotorsMatrix class converts roll, pitch, yaw, and throttle (RPYT) commands
 *          into individual motor thrust values using a geometric mixing matrix. This approach
 *          supports standard frame configurations (quad, hexa, octa) as well as custom
 *          motor arrangements with up to 32 motors.
 *          
 *          The mixing algorithm computes motor thrust using:
 *          thrust[i] = throttle + roll*roll_factor[i] + pitch*pitch_factor[i] + yaw*yaw_factor[i]
 *          
 *          Motor factors are derived from geometric position and rotation direction.
 *          The class handles motor failure detection and compensation, test sequencing,
 *          and various frame-specific motor arrangements.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include "AP_MotorsMulticopter.h"

#define AP_MOTORS_MATRIX_YAW_FACTOR_CW   -1
#define AP_MOTORS_MATRIX_YAW_FACTOR_CCW   1

/**
 * @class AP_MotorsMatrix
 * @brief Matrix-style motor mixing for standard multicopter frames
 * 
 * @details This class implements motor control for multicopters using a mixing matrix approach.
 *          It converts pilot input (roll, pitch, yaw, throttle) into individual motor thrust
 *          commands through geometric mixing factors.
 *          
 *          **Purpose**: Provides flexible motor mixing for standard and custom multicopter configurations
 *          
 *          **Mixing Algorithm**: 
 *          For each motor i, the output thrust is calculated as:
 *          ```
 *          thrust[i] = throttle + roll*roll_factor[i] + pitch*pitch_factor[i] + yaw*yaw_factor[i]
 *          ```
 *          
 *          **Factor Calculation from Geometry**:
 *          - roll_factor[i] = sin(motor_angle[i]) - describes motor's roll authority
 *          - pitch_factor[i] = cos(motor_angle[i]) - describes motor's pitch authority  
 *          - yaw_factor[i] = ±1 where -1=CW rotation (negative torque), +1=CCW (positive torque)
 *          - throttle_factor[i] = typically 1.0, can vary for thrust vectoring
 *          
 *          **Normalization**: All RPY factors are normalized so maximum magnitude = 0.5
 *          to prevent saturation and ensure control authority remains balanced.
 *          
 *          **Frame Support**: 
 *          - Quad: Plus (+), X, V, H configurations
 *          - Hexa: Plus, X, DJI, IIR configurations  
 *          - Octa: Plus, X, V, DJI, IIR, Wide configurations
 *          - Y6: Coaxial tri-rotor configuration
 *          - Deca, OctaQuad, DodecaHexa: High motor count configurations
 *          - Custom: User-defined motor arrangements via scripting
 *          
 *          **Motor Numbering**:
 *          - Internal: 0-based indexing (AP_MOTORS_MOT_1 = 0)
 *          - User parameters: 1-based (MOT_1 in parameters = motor 0 internally)
 *          - Maximum: 32 motors (AP_MOTORS_MAX_NUM_MOTORS)
 *          
 *          **Motor Ordering**: Configurable test sequence determines motor test order,
 *          typically 1,2,3,4... proceeding around the frame. Supports standard ArduPilot
 *          numbering as well as Betaflight and DJI conventions.
 *          
 *          **Motor Failure Handling**: Detects failed motors through thrust monitoring
 *          and can compensate by increasing thrust on remaining motors via thrust boost.
 * 
 * @note Motor factors are stored in arrays: _roll_factor[], _pitch_factor[], _yaw_factor[],
 *       _throttle_factor[], with outputs in _thrust_rpyt_out[]
 * 
 * @note Testing order array (_test_order[]) determines sequence for motor test mode
 * 
 * @warning Motor test mode WILL SPIN MOTORS - always remove propellers before testing
 * 
 * @warning Incorrect motor angles cause wrong control response (e.g., roll commands produce pitch)
 * 
 * @warning Wrong yaw factors (CW/CCW reversed) cause yaw instability and oscillation
 * 
 * @see AP_MotorsMulticopter for base multicopter functionality
 * @see libraries/AP_Motors/AP_MotorsMatrix.cpp for implementation details
 */
class AP_MotorsMatrix : public AP_MotorsMulticopter {
public:

    /**
     * @brief Constructor for AP_MotorsMatrix
     * 
     * @details Initializes the matrix-based motor mixer with specified update rate.
     *          Enforces singleton pattern to ensure only one instance exists.
     * 
     * @param[in] speed_hz Motor update rate in Hz (default: AP_MOTORS_SPEED_DEFAULT, typically 490Hz)
     * 
     * @note This enforces singleton pattern - only one AP_MotorsMatrix instance allowed per system
     * 
     * @warning Will trigger panic if multiple instances are created
     */
    AP_MotorsMatrix(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMulticopter(speed_hz)
    {
        if (_singleton != nullptr) {
            AP_HAL::panic("AP_MotorsMatrix must be singleton");
        }
        _singleton = this;
    };

    /**
     * @brief Get singleton instance
     * 
     * @details Returns pointer to the singleton AP_MotorsMatrix instance
     * 
     * @return Pointer to AP_MotorsMatrix singleton instance, nullptr if not yet created
     */
    static AP_MotorsMatrix *get_singleton() {
        return _singleton;
    }

    /**
     * @brief Initialize motor layout for specified frame class and type
     * 
     * @details Configures the motor mixing matrix based on the vehicle frame configuration.
     *          Sets up motor positions, mixing factors, and test ordering for standard
     *          frame types. This is the primary initialization method called during vehicle setup.
     *          
     *          Internally calls setup_motors() to configure the geometry-specific mixing factors
     *          for the selected frame class (QUAD, HEXA, OCTA, etc.) and frame type (PLUS, X, V, etc.).
     * 
     * @param[in] frame_class Frame class enum: MOTOR_FRAME_QUAD, MOTOR_FRAME_HEXA, MOTOR_FRAME_OCTA, etc.
     * @param[in] frame_type Frame type enum: MOTOR_FRAME_TYPE_PLUS, MOTOR_FRAME_TYPE_X, MOTOR_FRAME_TYPE_V, etc.
     * 
     * @note This method must be called before motors can be used
     * @note Motor numbering follows standard ArduPilot conventions unless overridden by frame type
     * 
     * @see setup_motors() for frame-specific configuration details
     */
    virtual void        init(motor_frame_class frame_class, motor_frame_type frame_type) override;

#if AP_SCRIPTING_ENABLED
    /**
     * @brief Initialize motor matrix for scripting-defined custom frame
     * 
     * @details Alternative initialization method for custom motor configurations defined via Lua scripting.
     *          Prepares the motor arrays for the expected number of motors. Motors must then be added
     *          individually using add_motor_raw() or add_motor() calls from the script.
     *          
     *          This allows fully custom frame geometries not covered by standard frame types.
     * 
     * @param[in] expected_num_motors Number of motors to be configured (1-32)
     * 
     * @return true if initialization successful, false if expected_num_motors exceeds AP_MOTORS_MAX_NUM_MOTORS
     * 
     * @note Only available when AP_SCRIPTING_ENABLED
     * @note Must call add_motor_raw() for each motor after this initialization
     * 
     * @see add_motor_raw() for adding individual motors with explicit factors
     */
    virtual bool        init(uint8_t expected_num_motors);

    /**
     * @brief Set throttle factor for a specific motor (scripting interface)
     * 
     * @details Allows per-motor throttle scaling, useful for thrust vectoring or tilt-rotor
     *          configurations where motors contribute differently to vertical thrust.
     *          Standard multirotors use throttle_factor = 1.0 for all motors.
     * 
     * @param[in] motor_num Motor index (0-31, where 0 = MOT_1 in parameters)
     * @param[in] throttle_factor Throttle contribution factor (typically 0.0-1.0, where 1.0 = full contribution)
     * 
     * @return true if throttle factor set successfully, false if motor_num invalid
     * 
     * @note Only available when AP_SCRIPTING_ENABLED
     * @note Throttle factor affects vertical thrust contribution, not total motor output
     * @note Values > 1.0 possible for vectored thrust configurations
     */
    bool                set_throttle_factor(int8_t motor_num, float throttle_factor);

#endif // AP_SCRIPTING_ENABLED

    /**
     * @brief Set or change frame class and type
     * 
     * @details Reconfigures the motor mixing matrix for a different frame configuration.
     *          This allows dynamic frame changes and is called during vehicle setup or
     *          when parameters are changed. Triggers setup_motors() to recalculate mixing factors.
     * 
     * @param[in] frame_class Frame class enum: MOTOR_FRAME_QUAD, MOTOR_FRAME_HEXA, MOTOR_FRAME_OCTA, etc.
     * @param[in] frame_type Frame type enum: MOTOR_FRAME_TYPE_PLUS, MOTOR_FRAME_TYPE_X, MOTOR_FRAME_TYPE_V, etc.
     * 
     * @note Can be called at runtime to change frame configuration
     * @note Recalculates all motor mixing factors for the new frame geometry
     */
    void                set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override;

    /**
     * @brief Set motor PWM update rate
     * 
     * @details Configures the PWM output frequency for motor control signals.
     *          Higher rates (e.g., 400-490Hz) provide better control response but may be
     *          incompatible with some ESCs. Lower rates (e.g., 50Hz) required for standard servos.
     *          
     *          This method must be called after setup_motors() has configured the motor layout.
     * 
     * @param[in] speed_hz Update frequency in Hertz (typical range: 50-490Hz, can be higher for digital protocols)
     * 
     * @note Must call setup_motors() before calling this method
     * @note Different ESC types support different update rates - verify ESC compatibility
     * @note Digital protocols (DShot, etc.) use different rate mechanisms
     * 
     * @warning Incorrect update rates may cause ESC malfunction or motor synchronization issues
     */
    void                set_update_rate(uint16_t speed_hz) override;

    /**
     * @brief Test individual motor by spinning at specified PWM value
     * 
     * @details Outputs a specific PWM value to one motor for testing purposes.
     *          Used during vehicle setup, motor ordering verification, and troubleshooting.
     *          If motor output channels have been remapped, the remapped channel is used.
     *          
     *          This function bypasses normal mixing and directly controls motor output.
     * 
     * @param[in] motor Motor number to test (0-31, where 0 = MOT_1 in parameters)
     * @param[in] pwm PWM value in microseconds (typical range: 1000-2000μs, 1000=stopped, 2000=full throttle)
     * 
     * @return true if motor output successfully set, false if motor number invalid or not configured
     * 
     * @note This should ONLY be performed during testing with propellers removed
     * 
     * @warning WILL SPIN MOTOR - Remove propellers before use to prevent injury or damage
     * @warning Bypasses all safety checks - use only in controlled test environment
     * 
     * @see _output_test_seq() for test sequence implementation
     */
    bool                output_test_num(uint8_t motor, int16_t pwm);

    /**
     * @brief Send motor commands to outputs
     * 
     * @details Executes the mixing algorithm and writes PWM values to motor outputs.
     *          This is the main method called from the vehicle code to update motor outputs
     *          based on current control inputs. Applies mixing matrix, saturation limiting,
     *          and writes final PWM values to hardware.
     * 
     * @note Called at main loop rate (typically 400Hz for multicopters)
     * @note Internally calls output_armed_stabilizing() for normal flight operations
     * 
     * @see output_armed_stabilizing() for mixing implementation
     */
    virtual void        output_to_motors() override;

    /**
     * @brief Get bitmask of active motor outputs
     * 
     * @details Returns a 32-bit bitmask indicating which PWM outputs are being used for motors.
     *          Each bit position corresponds to a motor number (bit N = motor N, 0-based).
     *          Bit value 1 means that motor is active, 0 means unused.
     *          
     *          This is used to ensure other PWM outputs (e.g., for servos, camera triggers)
     *          do not conflict with motor outputs.
     * 
     * @return 32-bit bitmask where bit N set to 1 indicates motor N is active (0-31)
     * 
     * @note Bit positions are 0-based: bit 0 = motor 0 (MOT_1), bit 1 = motor 1 (MOT_2), etc.
     * @note Maximum 32 motors supported (AP_MOTORS_MAX_NUM_MOTORS)
     * 
     * Example: 0x0000000F (binary: 1111) indicates motors 0,1,2,3 are active (standard quad)
     */
    uint32_t            get_motor_mask() override;

    /**
     * @brief Get index of failed motor
     * 
     * @details Returns the motor number that has been detected as failed through thrust monitoring.
     *          The check_for_failed_motor() method continuously monitors motor thrust outputs
     *          and identifies motors producing insufficient thrust. This method should only be
     *          called when get_thrust_boost() returns true, indicating failure compensation is active.
     * 
     * @return Motor index of failed motor (0-31, where 0 = MOT_1)
     * 
     * @note Only valid when get_thrust_boost() returns true indicating active motor failure compensation
     * @note Motor failure detection uses filtered thrust output comparison
     * 
     * @warning Returned value is undefined if no motor failure detected (check get_thrust_boost() first)
     * 
     * @see check_for_failed_motor() for failure detection implementation
     * @see get_thrust_boost() to check if motor failure compensation is active
     */
    uint8_t             get_lost_motor() const override { return _motor_lost_index; }

    /**
     * @brief Get roll mixing factor for specified motor
     * 
     * @details Returns the roll contribution factor for a specific motor.
     *          This factor determines how much roll control authority the motor provides.
     *          Typically roll_factor = sin(motor_angle), where 0° is forward.
     *          
     *          This is used for tilt-rotors and tail-sitters that use copter motors
     *          for forward flight control, allowing vehicle code to understand motor
     *          control authority in different flight modes.
     * 
     * @param[in] i Motor index (0-31, where 0 = MOT_1)
     * 
     * @return Roll mixing factor (typically -0.5 to +0.5 after normalization)
     * 
     * @note Positive values indicate motor produces positive roll (right bank) when thrust increases
     * @note Used by tilt-rotor and tailsitter control logic
     */
    float               get_roll_factor(uint8_t i) override { return _roll_factor[i]; }
    
    /**
     * @brief Get pitch mixing factor for specified motor
     * 
     * @details Returns the pitch contribution factor for a specific motor.
     *          This factor determines how much pitch control authority the motor provides.
     *          Typically pitch_factor = cos(motor_angle), where 0° is forward.
     * 
     * @param[in] i Motor index (0-31, where 0 = MOT_1)
     * 
     * @return Pitch mixing factor (typically -0.5 to +0.5 after normalization)
     * 
     * @note Positive values indicate motor produces positive pitch (nose up) when thrust increases
     */
    float               get_pitch_factor(uint8_t i) override { return _pitch_factor[i]; }

    /**
     * @brief Disable yaw control via motor torque
     * 
     * @details Disables the use of differential motor torque for yaw control.
     *          Sets all motor yaw factors to zero, removing yaw component from mixing.
     *          
     *          This is used when an external mechanism provides yaw control, such as:
     *          - Thrust vectoring (tilting motors or nozzles)
     *          - Rudder or tail rotor (on hybrid vehicles)
     *          - Differential thrust via tilt (tilt-rotors)
     *          
     *          After calling this, yaw commands will not affect motor mixing.
     * 
     * @note Used by tilt-rotors and other hybrid vehicle types
     * @note Once disabled, yaw control must come from external mechanism
     * 
     * @warning Disabling yaw torque without alternative yaw control will result in uncontrollable yaw
     */
    void                disable_yaw_torque(void) override;

    /**
     * @brief Add motor with explicit mixing factors
     * 
     * @details Adds a motor to the mixing matrix using explicitly specified roll, pitch, yaw,
     *          and throttle factors. This provides complete control over motor mixing behavior
     *          and is used for custom frame configurations or scripting-defined geometries.
     *          
     *          The mixing equation for this motor becomes:
     *          thrust = throttle*throttle_factor + roll*roll_fac + pitch*pitch_fac + yaw*yaw_fac
     *          
     *          For standard multirotors with motors at angle θ from forward:
     *          - roll_fac = sin(θ)  
     *          - pitch_fac = cos(θ)
     *          - yaw_fac = -1 (CW rotation) or +1 (CCW rotation)
     *          - throttle_factor = 1.0 (full vertical thrust contribution)
     * 
     * @param[in] motor_num Motor index (0-31, where 0 = MOT_1 in parameters)
     * @param[in] roll_fac Roll mixing factor (range typically -1.0 to +1.0, normalized to ±0.5)
     * @param[in] pitch_fac Pitch mixing factor (range typically -1.0 to +1.0, normalized to ±0.5)
     * @param[in] yaw_fac Yaw mixing factor (-1=CW rotation, +1=CCW rotation, 0=no yaw contribution)
     * @param[in] testing_order Motor test sequence position (1-32, order in which motor spins during motor test)
     * @param[in] throttle_factor Throttle contribution factor (default 1.0, can vary for vectored thrust)
     * 
     * @note Factors will be normalized by normalise_rpy_factors() so maximum magnitude = 0.5
     * @note Motor numbering: motor_num=0 corresponds to MOT_1 in parameters (0-based internal, 1-based user)
     * @note Testing order determines spin sequence in motor test mode
     * 
     * @warning Incorrect factors will cause wrong control response - verify with motor test before flight
     * 
     * @see add_motor() for geometric angle-based motor addition
     * @see normalise_rpy_factors() for factor scaling
     */
    void                add_motor_raw(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, uint8_t testing_order, float throttle_factor = 1.0f);

    /**
     * @struct MotorDef
     * @brief Motor definition using geometric angle and rotation direction
     * 
     * @details Structure for defining motors by their geometric position angle and
     *          propeller rotation direction. Used for convenient bulk motor definition
     *          in frame setup code. Angle is measured from forward (0°) clockwise when
     *          viewed from above.
     */
    struct MotorDef {
        float angle_degrees;     ///< Motor angle in degrees (0=forward, 90=right, 180=back, 270=left)
        float yaw_factor;        ///< Yaw factor: -1=CW prop rotation, +1=CCW prop rotation
        uint8_t testing_order;   ///< Motor test sequence position (1=first motor to test, 2=second, etc.)
    };

    /**
     * @brief Add multiple motors using geometric definitions
     * 
     * @details Convenience method to add multiple motors at once using geometric angle
     *          definitions. This is the typical method used in frame setup functions
     *          (setup_quad_matrix, setup_hexa_matrix, etc.).
     *          
     *          Each motor is defined by its angle and rotation direction. Roll and pitch
     *          factors are calculated automatically from the angle:
     *          - roll_factor = sin(angle_degrees)
     *          - pitch_factor = cos(angle_degrees)
     * 
     * @param[in] motors Pointer to array of MotorDef structures defining motor geometry
     * @param[in] num_motors Number of motors in the array (must be ≤ AP_MOTORS_MAX_NUM_MOTORS)
     * 
     * @note This is a convenience wrapper around add_motor() called for each motor in array
     * @note Standard frame configurations (X, +, etc.) use this method in their setup functions
     * 
     * @see MotorDef for structure definition
     * @see add_motor() for individual motor addition with angle
     */
    void add_motors(const struct MotorDef *motors, uint8_t num_motors);

    /**
     * @struct MotorDefRaw
     * @brief Motor definition using explicit roll/pitch/yaw factors
     * 
     * @details Structure for defining motors with explicit mixing factors rather than
     *          geometric angles. Used for asymmetric frames or special configurations
     *          where simple angle-based calculation is insufficient.
     *          
     *          Note: This structure does NOT include throttle_factor (defaults to 1.0).
     *          The throttle_factor parameter is primarily used in scripting bindings
     *          for thrust vectoring configurations.
     */
    struct MotorDefRaw {
        float roll_fac;          ///< Roll mixing factor (typically -1.0 to +1.0, will be normalized)
        float pitch_fac;         ///< Pitch mixing factor (typically -1.0 to +1.0, will be normalized)
        float yaw_fac;           ///< Yaw mixing factor (-1=CW, +1=CCW, 0=no yaw contribution)
        uint8_t testing_order;   ///< Motor test sequence position (1=first, 2=second, etc.)
    };
    
    /**
     * @brief Add multiple motors using explicit mixing factors
     * 
     * @details Convenience method to add multiple motors with explicit roll/pitch/yaw factors.
     *          Used for asymmetric frame configurations where geometric angle calculation
     *          would be insufficient or incorrect.
     *          
     *          Each motor is added with throttle_factor defaulting to 1.0 (full contribution).
     * 
     * @param[in] motors Pointer to array of MotorDefRaw structures defining motor factors
     * @param[in] num_motors Number of motors in the array (must be ≤ AP_MOTORS_MAX_NUM_MOTORS)
     * 
     * @note This is a convenience wrapper around add_motor_raw() called for each motor
     * @note Throttle factor defaults to 1.0 for all motors added this way
     * 
     * @see MotorDefRaw for structure definition
     * @see add_motor_raw() for individual motor addition with explicit factors
     */
    void add_motors_raw(const struct MotorDefRaw *motors, uint8_t num_motors);

    /**
     * @brief Get combined RPYT output thrust for a motor
     * 
     * @details Returns the final mixed thrust output value for a specific motor after
     *          roll, pitch, yaw, and throttle have been combined through the mixing matrix.
     *          This is the value that will be converted to PWM and sent to the motor.
     *          
     *          Value range is typically 0.0 to 1.0, but may exceed during boost conditions.
     * 
     * @param[in] i Motor index (0-31, where 0 = MOT_1)
     * 
     * @return Combined thrust output value (range 0.0-1.0 normally, may exceed during boost)
     * 
     * @note This is the output of the mixing calculation before PWM conversion
     * @note Available for diagnostic purposes and custom vehicle implementations
     */
    float get_thrust_rpyt_out(uint8_t i) const;
    
    /**
     * @brief Get all mixing factors and test order for a motor
     * 
     * @details Retrieves complete mixing configuration for a specific motor including
     *          roll, pitch, yaw, and throttle factors plus the test sequence order.
     *          Useful for diagnostics, custom vehicle types, and scripting interfaces.
     * 
     * @param[in]  i Motor index (0-31, where 0 = MOT_1)
     * @param[out] roll Roll mixing factor (-0.5 to +0.5 after normalization)
     * @param[out] pitch Pitch mixing factor (-0.5 to +0.5 after normalization)
     * @param[out] yaw Yaw mixing factor (-1, 0, or +1 typically)
     * @param[out] throttle Throttle contribution factor (typically 1.0)
     * @param[out] testing_order Motor test sequence position (1-32)
     * 
     * @return true if motor is configured (factors retrieved), false if motor_num invalid or not configured
     * 
     * @note All factors returned are post-normalization values
     * @note Useful for custom vehicle types that need to understand motor configuration
     */
    bool get_factors(uint8_t i, float &roll, float &pitch, float &yaw, float &throttle, uint8_t &testing_order) const;

protected:
    /**
     * @brief Main mixing and motor output function
     * 
     * @details This is the core mixing function that computes individual motor thrust values
     *          from roll, pitch, yaw, and throttle (RPYT) inputs. Called at main loop rate
     *          during normal flight when vehicle is armed and stabilizing.
     *          
     *          Algorithm:
     *          For each motor i:
     *          ```
     *          thrust[i] = throttle*throttle_factor[i] + 
     *                      roll*roll_factor[i] + 
     *                      pitch*pitch_factor[i] + 
     *                      yaw*yaw_factor[i]
     *          ```
     *          
     *          After mixing, applies:
     *          - Thrust limiting (0.0 to 1.0 range)
     *          - Saturation handling (scales all motors if any exceed limits)
     *          - Converts thrust to PWM values
     *          - Writes PWM to motor outputs
     * 
     * @note Called at main loop rate (typically 400Hz for multicopters)
     * @note Mixing saturation (thrust > 1.0) handled by proportional scaling to maintain control ratios
     * 
     * @warning Saturation reduces control authority - may affect stability in extreme maneuvers
     * 
     * @see output_to_motors() for the public interface that calls this method
     */
    void                output_armed_stabilizing() override;

    /**
     * @brief Detect motor failure through thrust monitoring
     * 
     * @details Continuously monitors motor thrust outputs to detect failed motors.
     *          Compares each motor's filtered thrust output against expected values.
     *          When a motor consistently produces insufficient thrust, it is marked as failed
     *          and thrust boost mode is enabled to compensate with remaining motors.
     *          
     *          Uses 1-second time constant filtering on thrust outputs to avoid
     *          false positives from transient conditions.
     * 
     * @param[in] throttle_thrust_best Expected baseline throttle/thrust for comparison
     * 
     * @note Sets _motor_lost_index to the failed motor number
     * @note Enables thrust boost mode when failure detected
     * @note Filtered outputs stored in _thrust_rpyt_out_filt[]
     * 
     * @see get_lost_motor() to retrieve failed motor index
     * @see get_thrust_boost() to check if compensation is active
     */
    void                check_for_failed_motor(float throttle_thrust_best);

    /**
     * @brief Add motor using geometric angle and rotation direction
     * 
     * @details Adds a motor to the mixing matrix using geometric position angle.
     *          Roll and pitch factors are calculated automatically:
     *          - roll_factor = sin(angle_degrees)
     *          - pitch_factor = cos(angle_degrees)
     *          
     *          This is the typical method used for symmetric frame configurations.
     *          Angle is measured from forward (0°) proceeding clockwise when viewed from above.
     * 
     * @param[in] motor_num Motor index (0-31, where 0 = MOT_1 in parameters)
     * @param[in] angle_degrees Motor angle in degrees (0=forward, 90=right, 180=back, 270=left)
     * @param[in] yaw_factor Yaw contribution: -1=CW prop rotation, +1=CCW prop rotation
     * @param[in] testing_order Motor test sequence position (1=first to test, 2=second, etc.)
     * 
     * @note Throttle factor defaults to 1.0 (full vertical thrust contribution)
     * @note This is a convenience wrapper that calculates factors and calls add_motor_raw()
     * 
     * @see add_motor_raw() for explicit factor-based motor addition
     */
    void                add_motor(int8_t motor_num, float angle_degrees, float yaw_factor, uint8_t testing_order);

    /**
     * @brief Add motor using separate roll and pitch angles (asymmetric frames)
     * 
     * @details Adds a motor using separate roll and pitch angle components instead of
     *          a single position angle. Used for asymmetric frame configurations where
     *          motor positions don't align with simple radial geometry.
     *          
     *          Factors calculated as:
     *          - roll_factor = sin(roll_factor_in_degrees)
     *          - pitch_factor = sin(pitch_factor_in_degrees)
     * 
     * @param[in] motor_num Motor index (0-31, where 0 = MOT_1 in parameters)
     * @param[in] roll_factor_in_degrees Angle in degrees for roll factor calculation
     * @param[in] pitch_factor_in_degrees Angle in degrees for pitch factor calculation
     * @param[in] yaw_factor Yaw contribution: -1=CW rotation, +1=CCW rotation
     * @param[in] testing_order Motor test sequence position
     * 
     * @note Used for H-frames, V-frames, and other asymmetric configurations
     * @note Both roll and pitch use sin() of their respective angles
     */
    void                add_motor(int8_t motor_num, float roll_factor_in_degrees, float pitch_factor_in_degrees, float yaw_factor, uint8_t testing_order);

    /**
     * @brief Remove motor from mixing matrix
     * 
     * @details Removes a motor from the active mixing configuration by clearing
     *          all its mixing factors to zero. Used during reconfiguration or
     *          for dynamic motor count changes.
     * 
     * @param[in] motor_num Motor index to remove (0-31)
     * 
     * @note Sets all factors (roll, pitch, yaw, throttle) to 0.0 for specified motor
     * @note Motor output will remain at minimum (stopped) after removal
     */
    void                remove_motor(int8_t motor_num);

    /**
     * @brief Configure motors for specified frame class and type
     * 
     * @details Sets up the complete motor mixing matrix for standard frame configurations.
     *          This is called during initialization and when frame type changes.
     *          Delegates to frame-specific setup functions:
     *          - setup_quad_matrix() - 4 motor configurations
     *          - setup_hexa_matrix() - 6 motor configurations
     *          - setup_octa_matrix() - 8 motor configurations  
     *          - setup_y6_matrix() - Y6 coaxial configuration
     *          - setup_deca_matrix() - 10 motor configurations
     *          - setup_dodecahexa_matrix() - 12 motor configurations
     *          - setup_octaquad_matrix() - 8 motor quad configuration
     *          
     *          After adding motors, calls normalise_rpy_factors() to scale factors.
     * 
     * @param[in] frame_class Frame class enum (QUAD, HEXA, OCTA, Y6, DECA, etc.)
     * @param[in] frame_type Frame type enum (PLUS, X, V, H, DJI, etc.)
     * 
     * @note Virtual function - can be overridden for custom frame types
     * @note Calls frame-specific setup function based on frame_class
     * @note Always calls normalise_rpy_factors() after motor configuration
     * 
     * @see normalise_rpy_factors() for factor scaling
     */
    virtual void        setup_motors(motor_frame_class frame_class, motor_frame_type frame_type);

    /**
     * @brief Normalize roll, pitch, yaw factors for numerical stability
     * 
     * @details Scales roll, pitch, and yaw mixing factors so that the maximum magnitude
     *          across all motors is 0.5. This normalization:
     *          - Prevents saturation in mixing calculations
     *          - Ensures consistent control authority across different frame types
     *          - Maintains proper headroom for mixing without exceeding thrust limits
     *          - Preserves relative factor ratios between motors
     *          
     *          Normalization equation:
     *          ```
     *          scale = 0.5 / max(|roll_factor|, |pitch_factor|, |yaw_factor|)
     *          all_factors *= scale
     *          ```
     * 
     * @note Called automatically by setup_motors() after motor configuration
     * @note Does not modify throttle_factor values
     * @note Essential for preventing control saturation and maintaining stability
     * 
     * @warning Do not skip normalization - unnormalized factors can cause saturation and instability
     */
    void                normalise_rpy_factors();

    /**
     * @brief Apply vehicle-specific thrust compensation
     * 
     * @details Calls vehicle-supplied thrust compensation callback if configured.
     *          This allows vehicle-specific code to modify thrust outputs based on
     *          flight conditions, battery voltage, or other factors.
     *          
     *          Used for advanced features like thrust curve compensation,
     *          battery voltage compensation, or propeller efficiency corrections.
     * 
     * @note Override point for vehicle-specific thrust modifications
     * @note Called during motor output processing
     */
    void                thrust_compensation(void) override;

    /**
     * @brief Get frame class string representation
     * 
     * @return String name of frame class (e.g., "Quad", "Hexa", "Octa")
     */
    const char*         _get_frame_string() const override { return _frame_class_string; }
    
    /**
     * @brief Get frame type string representation
     * 
     * @return String name of frame type (e.g., "X", "Plus", "V", "H")
     */
    const char*         get_type_string() const override { return _frame_type_string; }

    /**
     * @brief Test motor by sequence number
     * 
     * @details Spins a motor at the specified PWM value based on its position in the
     *          test sequence rather than motor number. The testing_order value assigned
     *          to each motor determines its sequence number.
     *          
     *          This allows testing motors in a logical order around the frame
     *          (e.g., front-right, rear-right, rear-left, front-left for X-quad).
     * 
     * @param[in] motor_seq Motor sequence number in test order (1 to number of motors)
     * @param[in] pwm PWM value in microseconds (typical range: 1000-2000μs)
     * 
     * @note motor_seq is 1-based (1 = first motor in test sequence)
     * @note Converts sequence number to motor number using _test_order[] array
     * 
     * @warning WILL SPIN MOTOR - Remove propellers before testing
     */
    virtual void        _output_test_seq(uint8_t motor_seq, int16_t pwm) override;

    // Mixing factor arrays - define each motor's contribution to control axes
    float               _roll_factor[AP_MOTORS_MAX_NUM_MOTORS];     ///< Each motor's contribution to roll control (typically ±0.5 after normalization)
    float               _pitch_factor[AP_MOTORS_MAX_NUM_MOTORS];    ///< Each motor's contribution to pitch control (typically ±0.5 after normalization)
    float               _yaw_factor[AP_MOTORS_MAX_NUM_MOTORS];      ///< Each motor's contribution to yaw control (typically -1=CW, +1=CCW, 0=none)
    float               _throttle_factor[AP_MOTORS_MAX_NUM_MOTORS]; ///< Each motor's contribution to vertical thrust (typically 1.0, range 0.0-1.0)
    
    float               _thrust_rpyt_out[AP_MOTORS_MAX_NUM_MOTORS]; ///< Combined RPYT output thrust for each motor (range 0.0-1.0, may exceed during boost)
    uint8_t             _test_order[AP_MOTORS_MAX_NUM_MOTORS];      ///< Motor test sequence order (1-based, defines spin order in motor test mode)

    // Motor failure detection and handling
    float               _thrust_rpyt_out_filt[AP_MOTORS_MAX_NUM_MOTORS]; ///< Filtered thrust outputs for failure detection (1 second time constant)
    uint8_t             _motor_lost_index;                               ///< Index of detected failed motor (0-31, valid only when thrust boost active)

    // Frame configuration
    motor_frame_class   _active_frame_class;        ///< Active frame class (QUAD, HEXA, OCTA, Y6, etc.)
    motor_frame_type    _active_frame_type;         ///< Active frame type (PLUS, X, V, H, DJI, etc.)

    const char*         _frame_class_string = "";   ///< String representation of frame class for logging/display
    const char*         _frame_type_string = "";    ///< String representation of frame type for logging/display

private:

    /**
     * @brief Interpolate value based on thrust boost ratio
     * 
     * @details Helper function that returns a value interpolated between boost_value
     *          and normal_value based on the current _thrust_boost_ratio.
     *          Used during motor failure compensation to smoothly transition between
     *          normal operation and boosted thrust on remaining motors.
     * 
     * @param[in] boost_value Value to use when thrust boost is fully active (ratio = 1.0)
     * @param[in] normal_value Value to use during normal operation (ratio = 0.0)
     * 
     * @return Interpolated value based on current _thrust_boost_ratio
     */
    float boost_ratio(float boost_value, float normal_value) const;

    // Frame-specific motor matrix setup functions
    
    /**
     * @brief Configure motor mixing for quadcopter frames
     * @param[in] frame_type Frame type (PLUS, X, V, H, etc.)
     * @return true if frame type supported and configured, false otherwise
     */
    bool setup_quad_matrix(motor_frame_type frame_type);
    
    /**
     * @brief Configure motor mixing for hexacopter frames
     * @param[in] frame_type Frame type (PLUS, X, DJI, etc.)
     * @return true if frame type supported and configured, false otherwise
     */
    bool setup_hexa_matrix(motor_frame_type frame_type);
    
    /**
     * @brief Configure motor mixing for octocopter frames
     * @param[in] frame_type Frame type (PLUS, X, V, DJI, WIDE, etc.)
     * @return true if frame type supported and configured, false otherwise
     */
    bool setup_octa_matrix(motor_frame_type frame_type);
    
    /**
     * @brief Configure motor mixing for decacopter (10 motor) frames
     * @param[in] frame_type Frame type configuration
     * @return true if frame type supported and configured, false otherwise
     */
    bool setup_deca_matrix(motor_frame_type frame_type);
    
    /**
     * @brief Configure motor mixing for dodecahexa (12 motor) frames
     * @param[in] frame_type Frame type configuration
     * @return true if frame type supported and configured, false otherwise
     */
    bool setup_dodecahexa_matrix(motor_frame_type frame_type);
    
    /**
     * @brief Configure motor mixing for Y6 coaxial tri-rotor frames
     * @param[in] frame_type Frame type configuration
     * @return true if frame type supported and configured, false otherwise
     * @note Y6 uses 6 motors in 3 coaxial pairs (top and bottom motor on each arm)
     */
    bool setup_y6_matrix(motor_frame_type frame_type);
    
    /**
     * @brief Configure motor mixing for octaquad frames
     * @param[in] frame_type Frame type configuration
     * @return true if frame type supported and configured, false otherwise
     * @note OctaQuad uses 8 motors in quad configuration for redundancy
     */
    bool setup_octaquad_matrix(motor_frame_type frame_type);

    static AP_MotorsMatrix *_singleton;  ///< Singleton instance pointer
};
