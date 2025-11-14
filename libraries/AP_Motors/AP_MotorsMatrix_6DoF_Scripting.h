/**
 * @file AP_MotorsMatrix_6DoF_Scripting.h
 * @brief Scripting-enabled 6DoF motor matrix for custom vectored thrust configurations
 * 
 * @details This file implements a specialized motor matrix class that allows Lua scripts
 *          to dynamically configure motor layouts with full six degrees of freedom (6DoF)
 *          control. Unlike standard multicopter configurations, this enables:
 *          - Custom vectored thrust configurations
 *          - ROV (Remotely Operated Vehicle) thrust arrangements
 *          - Experimental and research vehicle configurations
 *          - Runtime motor configuration via scripting API
 * 
 *          The 6DoF control provides independent control over:
 *          - Translation: Forward/Back (X), Left/Right (Y), Up/Down (Z)
 *          - Rotation: Roll, Pitch, Yaw
 * 
 *          This is particularly useful for underwater vehicles, VTOL aircraft with
 *          vectored thrust, and other specialized applications requiring non-standard
 *          motor arrangements.
 * 
 * @note This class is only available when AP_SCRIPTING_ENABLED is defined
 * 
 * Source: libraries/AP_Motors/AP_MotorsMatrix_6DoF_Scripting.h
 */

#pragma once
#if AP_SCRIPTING_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>
#include "AP_MotorsMatrix.h"

/**
 * @class AP_MotorsMatrix_6DoF_Scripting
 * @brief Scripting-enabled motor matrix with full 6DoF (six degrees of freedom) control
 * 
 * @details This class extends AP_MotorsMatrix to provide runtime-configurable motor
 *          layouts through the Lua scripting API. It enables full 6DoF thrust control
 *          by allowing arbitrary motor positions and orientations to be defined
 *          dynamically rather than using pre-defined frame configurations.
 * 
 *          **Purpose:**
 *          - Allow Lua scripts to dynamically configure motor layouts at runtime
 *          - Support non-standard vehicle configurations (ROVs, vectored-thrust VTOL)
 *          - Enable experimental vehicle designs without firmware recompilation
 *          - Provide full 6DoF thrust control (translation X/Y/Z + rotation roll/pitch/yaw)
 * 
 *          **Use Cases:**
 *          - ROVs (Remotely Operated Vehicles) with arbitrary thruster arrangements
 *          - Vectored-thrust aircraft with tilting motors
 *          - Research vehicles with experimental configurations
 *          - Custom underwater vehicles with non-standard thruster layouts
 * 
 *          **Runtime Configuration:**
 *          - Motors can be added dynamically via scripting API using add_motor()
 *          - Each motor defined by contribution factors for all 6 degrees of freedom
 *          - Supports reversible motors for bidirectional thrust (typical in ROVs)
 *          - Configuration validated before flight through init() method
 * 
 *          **Thrust Vectoring:**
 *          - Each motor has factors defining its contribution to each axis
 *          - Motor factors: roll, pitch, yaw, throttle (Z), forward (X), right (Y)
 *          - Thrust vector can be rotated in body frame using set_roll_pitch()
 *          - Enables arbitrary motor orientations for full 6DoF control
 * 
 *          **6DoF Control Axes:**
 *          - Roll: Rotation about forward axis (body X)
 *          - Pitch: Rotation about right axis (body Y)
 *          - Yaw: Rotation about up axis (body Z)
 *          - Throttle: Translation along up axis (body Z)
 *          - Forward: Translation along forward axis (body X)
 *          - Right: Translation along right axis (body Y)
 * 
 * @note Designed specifically for Lua scripting API exposure (AP_SCRIPTING_ENABLED)
 * @note Supports reversible motors for bidirectional thrust (e.g., ROV thrusters)
 * @note Provides full translation (X,Y,Z) and rotation (roll, pitch, yaw) control
 * @note Thread-safe: Uses semaphores for concurrent script access to motor configuration
 * 
 * @warning Script errors in motor configuration can cause loss of vehicle control
 * @warning Motor factors MUST be validated through testing before flight operations
 * @warning Requires thorough testing in a controlled environment (pool, test stand, SITL)
 * @warning NOT for general multicopter use - specialized applications only
 * @warning Incorrect motor factor configuration can result in unstable or uncontrollable vehicle
 * 
 * @see AP_MotorsMatrix Base class for motor mixing and output
 * @see AP_Scripting Lua scripting interface for runtime configuration
 */
class AP_MotorsMatrix_6DoF_Scripting : public AP_MotorsMatrix {
public:

    /**
     * @brief Constructor for scripting-enabled 6DoF motor matrix
     * 
     * @param[in] speed_hz Output update rate in Hz (default: AP_MOTORS_SPEED_DEFAULT)
     * 
     * @note Enforces singleton pattern - only one instance allowed
     * @note Panics if multiple instances are created
     */
    AP_MotorsMatrix_6DoF_Scripting(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMatrix(speed_hz)
    {
        if (_singleton != nullptr) {
            AP_HAL::panic("AP_MotorsMatrix 6DoF must be singleton");
        }
        _singleton = this;
    };

    /**
     * @brief Get singleton instance of AP_MotorsMatrix_6DoF_Scripting
     * 
     * @return Pointer to singleton instance, or nullptr if not created
     * 
     * @note Used by scripting binding to access the motor matrix instance
     */
    static AP_MotorsMatrix_6DoF_Scripting *get_singleton() {
        return _singleton;
    }

    /**
     * @brief Send minimum (safe idle) values to all motors
     * 
     * @details Outputs minimum PWM values to motors when vehicle is disarmed or
     *          in a safe state. This keeps ESCs alive and ready but motors not spinning.
     *          Overrides base class to handle 6DoF motor configuration.
     * 
     * @note Called when vehicle is disarmed or during initialization
     * @note For reversible motors, outputs mid-point (neutral) throttle value
     */
    void output_to_motors() override;

    /**
     * @brief Set roll and pitch offset angles to rotate thrust vector in body frame
     * 
     * @details This method rotates the entire thrust vector in the vehicle body frame,
     *          typically used to align the throttle thrust vector with earth frame up
     *          when the vehicle is at a non-level attitude. Common in ROVs and other
     *          6DoF vehicles where the base orientation may not be level.
     * 
     * @param[in] roll_deg  Roll offset angle in degrees
     * @param[in] pitch_deg Pitch offset angle in degrees
     * 
     * @note Offsets are stored internally in radians (_roll_offset, _pitch_offset)
     * @note Typically used to compensate for vehicle mounting angle or trim
     * @note Can be called from Lua scripts to adjust thrust orientation dynamically
     */
    void set_roll_pitch(float roll_deg, float pitch_deg) override;

    /**
     * @brief Add or configure a motor with specified thrust contribution factors
     * 
     * @details This method defines a motor's contribution to each of the six degrees of
     *          freedom. Each factor represents how much thrust from this motor contributes
     *          to movement or rotation along that axis. Called from Lua scripts to
     *          dynamically configure vehicle motor layout.
     * 
     *          **Motor Factor Parameters:**
     *          - **roll_factor**: Motor's contribution to roll moment (rotation about X axis)
     *          - **pitch_factor**: Motor's contribution to pitch moment (rotation about Y axis)
     *          - **yaw_factor**: Motor's contribution to yaw moment (rotation about Z axis)
     *          - **throttle_factor**: Motor's contribution to vertical (Z) thrust
     *          - **forward_factor**: Motor's contribution to forward (X) thrust
     *          - **right_factor**: Motor's contribution to right (Y) thrust
     * 
     *          **Factor Normalization:**
     *          - Factors are typically normalized so maximum magnitude is 0.5
     *          - Positive/negative values indicate direction of contribution
     *          - Zero factor means motor doesn't contribute to that axis
     *          - Factors determine motor mixing matrix for control allocation
     * 
     *          **Example Factor Usage:**
     *          - Forward-facing motor: forward_factor=0.5, other factors=0
     *          - Upward thruster: throttle_factor=0.5, other factors=0
     *          - Tilted motor: Non-zero values for multiple factors
     * 
     * @param[in] motor_num       Motor number (0 to AP_MOTORS_MAX_NUM_MOTORS-1)
     * @param[in] roll_factor     Contribution to roll moment (normalized, typically ±0.5)
     * @param[in] pitch_factor    Contribution to pitch moment (normalized, typically ±0.5)
     * @param[in] yaw_factor      Contribution to yaw moment (normalized, typically ±0.5)
     * @param[in] throttle_factor Contribution to vertical thrust (normalized, typically ±0.5)
     * @param[in] forward_factor  Contribution to forward thrust (normalized, typically ±0.5)
     * @param[in] right_factor    Contribution to right thrust (normalized, typically ±0.5)
     * @param[in] reversible      True if motor can reverse direction (bidirectional thrust)
     * @param[in] testing_order   Order in which motor is tested during motor tests
     * 
     * @note Called from Lua scripting API to configure motor layout dynamically
     * @note Must be called for each motor before calling init()
     * @note Factors determine control allocation - test thoroughly before flight
     * 
     * @warning Incorrect factor configuration can cause loss of control
     * @warning All motors must be configured before vehicle can arm
     * @warning Factor values must be validated through controlled testing
     * 
     * @see init() Must be called after all motors are added to mark initialization complete
     */
    void add_motor(int8_t motor_num, float roll_factor, float pitch_factor, float yaw_factor, float throttle_factor, float forward_factor, float right_factor, bool reversible, uint8_t testing_order);

    /**
     * @brief Initialize motor matrix after all motors have been configured
     * 
     * @details Validates that the expected number of motors have been configured via
     *          add_motor() calls and marks the motor matrix as initialized. This must
     *          be called from the Lua script after all motors are added, otherwise the
     *          vehicle will not be able to arm.
     * 
     * @param[in] expected_num_motors Number of motors that should have been configured
     * 
     * @return true if initialization successful (correct number of motors configured),
     *         false otherwise
     * 
     * @note Must be called after all add_motor() calls are complete
     * @note Vehicle cannot arm until initialization is successful
     * @note Typical usage in Lua: after adding all motors, call init(num_motors)
     * 
     * @warning Initialization failure will prevent vehicle from arming
     * @warning Ensure all motors are properly configured before calling init()
     */
    bool init(uint8_t expected_num_motors) override;

protected:
    /**
     * @brief Send stabilized motor commands when vehicle is armed
     * 
     * @details Computes motor outputs based on pilot inputs and attitude controller
     *          demands, then sends PWM commands to motors. Implements 6DoF thrust
     *          allocation using configured motor factors. Overrides base class to
     *          handle forward/right thrust factors and reversible motors.
     * 
     * @note Called at motor output rate when vehicle is armed and stabilizing
     * @note Applies roll/pitch offset rotation to thrust vector
     * @note Implements deadzone logic for reversible motors
     */
    void output_armed_stabilizing() override;

    /**
     * @brief Setup motors for a frame class and type (not used in scripting mode)
     * 
     * @details This method is intentionally empty because motor configuration is done
     *          dynamically through scripting API rather than predefined frame types.
     *          Scripting calls add_motor() and init() instead.
     * 
     * @param[in] frame_class Motor frame class (ignored)
     * @param[in] frame_type  Motor frame type (ignored)
     * 
     * @note Empty implementation - scripting provides configuration instead
     * @note Motors are configured via add_motor() and marked initialized via init()
     */
    void setup_motors(motor_frame_class frame_class, motor_frame_type frame_type) override {};

    /**
     * @brief Get frame type string for logging and display
     * 
     * @return Constant string "6DoF scripting" identifying this frame type
     */
    const char* _get_frame_string() const override { return "6DoF scripting"; }

    /**
     * @brief Each motor's contribution to forward (body X axis) thrust
     * 
     * @details Array of factors defining how much each motor contributes to forward
     *          thrust. Positive values produce forward thrust, negative produces
     *          backward thrust. Typically normalized to maximum magnitude of 0.5.
     *          Set via add_motor() forward_factor parameter.
     */
    float _forward_factor[AP_MOTORS_MAX_NUM_MOTORS];

    /**
     * @brief Each motor's contribution to right (body Y axis) thrust
     * 
     * @details Array of factors defining how much each motor contributes to right
     *          thrust. Positive values produce right thrust, negative produces
     *          left thrust. Typically normalized to maximum magnitude of 0.5.
     *          Set via add_motor() right_factor parameter.
     */
    float _right_factor[AP_MOTORS_MAX_NUM_MOTORS];

    /**
     * @brief True if motor is reversible (bidirectional thrust capability)
     * 
     * @details If true, motor can produce thrust in both directions, ranging from
     *          -Spin_Max to +Spin_Max (typical for ESCs with reversible firmware or
     *          ROV thrusters). If false, motor can only spin forward from Spin_Min
     *          to Spin_Max (typical for standard multicopter ESCs).
     * 
     * @note Reversible motors use mid-point PWM as neutral (zero thrust)
     * @note Non-reversible motors use minimum PWM as zero thrust
     */
    bool _reversible[AP_MOTORS_MAX_NUM_MOTORS];

    /**
     * @brief Store last motor thrust output values to implement deadzone
     * 
     * @details Stores previous motor output values to allow deadzone logic for
     *          reversible motors. Prevents chatter around zero thrust by maintaining
     *          zero output until commanded thrust exceeds deadzone threshold.
     * 
     * @note Used to implement smooth transition through zero for reversible motors
     */
    float _last_thrust_out[AP_MOTORS_MAX_NUM_MOTORS];

    /**
     * @brief Current roll offset angle in radians
     * 
     * @details Roll component of thrust vector rotation in body frame. Set via
     *          set_roll_pitch() method (input in degrees, stored in radians).
     *          Applied to rotate entire thrust vector for vehicle trim.
     */
    float _roll_offset;

    /**
     * @brief Current pitch offset angle in radians
     * 
     * @details Pitch component of thrust vector rotation in body frame. Set via
     *          set_roll_pitch() method (input in degrees, stored in radians).
     *          Applied to rotate entire thrust vector for vehicle trim.
     */
    float _pitch_offset;

private:
    /**
     * @brief Singleton instance pointer
     * 
     * @details Static pointer to the single instance of this class. Enforces singleton
     *          pattern - constructor will panic if multiple instances are created.
     *          Accessed via get_singleton() for scripting binding.
     * 
     * @note Only one instance allowed per vehicle
     */
    static AP_MotorsMatrix_6DoF_Scripting *_singleton;

};

#endif // AP_SCRIPTING_ENABLED
