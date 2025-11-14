/**
 * @file AP_MotorsMatrix_Scripting_Dynamic.h
 * @brief Dynamic motor matrix configuration via Lua scripting
 * 
 * @details This file implements runtime-reconfigurable motor mixing for multirotors,
 *          enabling Lua scripts to dynamically modify motor layouts, thrust factors,
 *          and control authority during vehicle operation. Designed for morphing vehicles,
 *          reconfigurable research platforms, and experimental multirotor configurations
 *          that require flight-time adaptation of motor mixing parameters.
 * 
 *          The dynamic matrix system bypasses the standard fixed frame configuration
 *          (X, +, H-quad, etc.) and allows scripts to define arbitrary motor positions,
 *          roll/pitch/yaw factors, throttle factors, and testing order.
 * 
 * @note This module is only available when AP_SCRIPTING_ENABLED is defined
 * 
 * @warning SAFETY CRITICAL: Runtime motor reconfiguration is EXTREMELY DANGEROUS
 *          and should only be used by experienced developers on research platforms.
 *          Incorrect motor factors can cause immediate loss of control and crashes.
 * 
 * Source: libraries/AP_Motors/AP_MotorsMatrix_Scripting_Dynamic.h
 */

#pragma once
#if AP_SCRIPTING_ENABLED

#include "AP_MotorsMatrix.h"

/**
 * @class AP_MotorsMatrix_Scripting_Dynamic
 * @brief Motor matrix with runtime-reconfigurable mixing via Lua scripting
 * 
 * @details This class extends AP_MotorsMatrix to support dynamic motor layout changes
 *          controlled by Lua scripts. Unlike standard motor matrices with fixed frame
 *          types, this implementation allows scripts to:
 *          
 *          - Add motors with custom roll/pitch/yaw/throttle factors
 *          - Load complete factor tables for all configured motors
 *          - Reconfigure motor mixing during flight (with extreme caution)
 *          - Support morphing vehicle geometries and experimental configurations
 * 
 *          **Use Cases:**
 *          - Morphing vehicles that change configuration in flight
 *          - Research platforms testing novel motor arrangements
 *          - Reconfigurable drones for multi-mission profiles
 *          - Fault-tolerant systems adapting to motor failures
 * 
 *          **Dynamic Mixing Architecture:**
 *          Motor factors define how each motor responds to roll/pitch/yaw commands
 *          and throttle input. Factors are normalized and validated to ensure:
 *          - Sufficient control authority in all axes
 *          - Numerical stability in mixing calculations
 *          - Balanced yaw authority for counter-rotating motors
 *          - Positive throttle contribution from all motors
 * 
 *          **Thread Safety:**
 *          Uses HAL_Semaphore to protect factor tables during updates, preventing
 *          corruption when scripts modify factors while motors are being output.
 * 
 * @note Singleton pattern enforced - only one instance allowed per vehicle
 * @note Supports up to AP_MOTORS_MAX_NUM_MOTORS (typically 12-32 depending on platform)
 * @note Factor validation ensures motor authority and stability before enabling
 * @note Exposed to Lua scripting via AP_SCRIPTING_ENABLED bindings
 * 
 * @warning Runtime motor reconfiguration during flight is EXTREMELY DANGEROUS
 * @warning Only for experienced developers and research platforms with safety protocols
 * @warning Requires extensive ground testing before any flight use
 * @warning Invalid factor combinations can cause immediate loss of control
 * @warning Must maintain sufficient control authority in all axes
 * @warning Yaw control requires balanced counter-rotating motor pairs
 * 
 * @see AP_MotorsMatrix for base motor matrix implementation
 * @see libraries/AP_Scripting for Lua scripting interface
 * 
 * Source: libraries/AP_Motors/AP_MotorsMatrix_Scripting_Dynamic.h
 */
class AP_MotorsMatrix_Scripting_Dynamic : public AP_MotorsMatrix {
public:

    /**
     * @brief Constructor for dynamic scripting motor matrix
     * 
     * @details Initializes the motor matrix with configurable output rate and enforces
     *          singleton pattern to ensure only one instance exists per vehicle.
     *          The constructor sets up the base motor matrix infrastructure but does
     *          not configure motors - configuration must be done via scripting using
     *          init() and add_motor() or load_factors() methods.
     * 
     * @param[in] speed_hz Motor output update rate in Hz (default: AP_MOTORS_SPEED_DEFAULT)
     *                     Typical values: 50Hz for standard PWM, 400Hz for digital protocols
     * 
     * @note Enforces singleton pattern - will panic if multiple instances created
     * @note Motor configuration must be completed via scripting before use
     * @note Output rate should match ESC protocol capabilities
     * 
     * @warning Creating multiple instances will trigger AP_HAL::panic() and halt system
     * 
     * Source: libraries/AP_Motors/AP_MotorsMatrix_Scripting_Dynamic.h:10-17
     */
    AP_MotorsMatrix_Scripting_Dynamic(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMatrix(speed_hz)
    {
        if (_singleton != nullptr) {
            AP_HAL::panic("AP_MotorsMatrix_Scripting_Dynamic must be singleton");
        }
        _singleton = this;
    };

    /**
     * @brief Get singleton instance of dynamic motor matrix
     * 
     * @details Provides access to the single instance of the dynamic motor matrix
     *          for use by scripting bindings and other system components.
     * 
     * @return Pointer to singleton instance, or nullptr if not yet created
     * 
     * @note Returns nullptr if constructor has not been called
     * @note Used primarily by scripting bindings to access the motor matrix
     * 
     * Source: libraries/AP_Motors/AP_MotorsMatrix_Scripting_Dynamic.h:20-22
     */
    static AP_MotorsMatrix_Scripting_Dynamic *get_singleton() {
        return _singleton;
    }

    /**
     * @struct factor_table
     * @brief Complete motor factor table for all axes
     * 
     * @details Defines the mixing factors for all motors across all control axes.
     *          Each array index corresponds to a motor number (0 to AP_MOTORS_MAX_NUM_MOTORS-1).
     *          Factors determine how each motor responds to roll/pitch/yaw commands and throttle.
     * 
     *          **Factor Interpretation:**
     *          - **roll[i]**: Motor contribution to roll control (-1.0 to +1.0)
     *                         Positive = motor increases speed for right roll
     *          - **pitch[i]**: Motor contribution to pitch control (-1.0 to +1.0)
     *                          Positive = motor increases speed for nose-up pitch
     *          - **yaw[i]**: Motor contribution to yaw control (-1.0 to +1.0)
     *                        Positive = clockwise rotation, Negative = counter-clockwise
     *          - **throttle[i]**: Motor contribution to vertical thrust (0.0 to 1.0)
     *                             Typically 1.0 for all motors on conventional multirotors
     * 
     * @note Factor validation ensures numerical stability and control authority
     * @note All motors in the array must be populated, even if not all are used
     * @note Factors are normalized internally for consistent control response
     * 
     * Source: libraries/AP_Motors/AP_MotorsMatrix_Scripting_Dynamic.h:24-29
     */
    struct factor_table {
        float roll[AP_MOTORS_MAX_NUM_MOTORS];      ///< Roll axis mixing factors for each motor
        float pitch[AP_MOTORS_MAX_NUM_MOTORS];     ///< Pitch axis mixing factors for each motor
        float yaw[AP_MOTORS_MAX_NUM_MOTORS];       ///< Yaw axis mixing factors for each motor
        float throttle[AP_MOTORS_MAX_NUM_MOTORS];  ///< Throttle/thrust factors for each motor
    };

    /**
     * @brief Base class init method - intentionally disabled for scripting use
     * 
     * @details The standard frame-based initialization is not used for dynamic matrices.
     *          Scripts must use the alternative init(uint8_t) method instead, which
     *          allows configuration without predefined frame types.
     * 
     * @param[in] frame_class Unused - standard frame class (not applicable)
     * @param[in] frame_type Unused - standard frame type (not applicable)
     * 
     * @note This method intentionally does nothing to prevent frame-based initialization
     * @note Use init(uint8_t expected_num_motors) from scripts instead
     * 
     * @warning Do not call this method - it has no effect for dynamic matrices
     * 
     * Source: libraries/AP_Motors/AP_MotorsMatrix_Scripting_Dynamic.h:32
     */
    void init(motor_frame_class frame_class, motor_frame_type frame_type) override {};

    /**
     * @brief Initialize dynamic motor matrix with expected motor count
     * 
     * @details Initializes the motor matrix for scripting configuration with the specified
     *          number of motors. Must be called from Lua script before adding motors or
     *          loading factor tables. Prepares the motor matrix for dynamic configuration
     *          and validates that the system is ready for motor factor definition.
     * 
     *          **Initialization Sequence (from script):**
     *          1. Call init(num_motors) to prepare matrix
     *          2. Call add_motor() for each motor OR load_factors() for complete table
     *          3. System validates factors and enables motor output
     * 
     * @param[in] expected_num_motors Number of motors to configure (1 to AP_MOTORS_MAX_NUM_MOTORS)
     * 
     * @return true if initialization successful, false if invalid motor count or already initialized
     * 
     * @note Must be called before add_motor() or load_factors()
     * @note Motor count must be between 1 and AP_MOTORS_MAX_NUM_MOTORS
     * @note Cannot re-initialize after motors are configured
     * 
     * @warning Ensure expected_num_motors matches actual motor configuration
     * @warning Must complete motor configuration before attempting flight
     * 
     * Source: libraries/AP_Motors/AP_MotorsMatrix_Scripting_Dynamic.h:35
     */
    bool init(uint8_t expected_num_motors) override;

    /**
     * @brief Add a motor with specified number and testing order
     * 
     * @details Registers a motor in the dynamic matrix with its motor number and testing
     *          sequence order. This method is called from Lua scripts to configure individual
     *          motors. After adding motors, scripts must call load_factors() to define the
     *          mixing factors for roll/pitch/yaw/throttle control.
     * 
     *          **Testing Order:**
     *          Defines the sequence for motor testing (used by motor test feature in ground
     *          stations). Motors are tested in ascending testing_order during pre-flight checks.
     * 
     * @param[in] motor_num Motor number (0 to AP_MOTORS_MAX_NUM_MOTORS-1), typically matches
     *                      output channel number
     * @param[in] testing_order Sequence for motor testing (0 = first, 1 = second, etc.)
     * 
     * @return true if motor added successfully, false if invalid motor_num or already configured
     * 
     * @note Motor factors must still be defined via load_factors() after adding motors
     * @note Motor numbers should match ESC output channel assignments
     * @note Testing order determines pre-flight motor check sequence
     * @note Must call init() before adding motors
     * 
     * @warning Motor number must be within valid range for board configuration
     * @warning Do not assign duplicate motor numbers
     * 
     * Source: libraries/AP_Motors/AP_MotorsMatrix_Scripting_Dynamic.h:38
     */
    bool add_motor(uint8_t motor_num, uint8_t testing_order);

    /**
     * @brief Load complete motor factor table from script
     * 
     * @details Loads a complete set of roll/pitch/yaw/throttle factors for all configured
     *          motors. This method is called from Lua scripts to define the motor mixing
     *          matrix that determines how control inputs are translated to motor commands.
     *          
     *          **Factor Validation:**
     *          - Normalizes factors for numerical stability
     *          - Verifies minimum control authority in each axis
     *          - Validates yaw balance for counter-rotating motors
     *          - Ensures all throttle factors are positive
     *          - Checks for sufficient control coupling
     * 
     *          **Thread Safety:**
     *          Uses semaphore locking to prevent factor updates during motor output,
     *          ensuring atomic factor table updates without corrupting active mixing.
     * 
     * @param[in] table Complete factor table with roll/pitch/yaw/throttle factors for all motors
     * 
     * @note Blocks motor output briefly during factor loading for thread safety
     * @note Factors are validated before being applied to active mixer
     * @note Invalid factor tables are rejected to prevent loss of control
     * @note Can be called multiple times to update factors (e.g., for morphing vehicles)
     * 
     * @warning Runtime factor changes during flight are EXTREMELY DANGEROUS
     * @warning Thoroughly test factor tables on ground before flight use
     * @warning Invalid factors can cause immediate loss of control and crash
     * @warning Ensure sufficient control authority in all axes (roll/pitch/yaw)
     * @warning Verify yaw factors provide balanced counter-rotation
     * 
     * Source: libraries/AP_Motors/AP_MotorsMatrix_Scripting_Dynamic.h:41
     */
    void load_factors(const factor_table &table);

    /**
     * @brief Output motor commands to ESCs
     * 
     * @details Sends calculated motor commands to ESC outputs based on current control
     *          inputs and configured motor factors. Called at motor output rate (typically
     *          50-400Hz depending on protocol). Uses semaphore protection to ensure factor
     *          tables are not modified during mixing calculations.
     * 
     *          **Output Sequence:**
     *          1. Acquire semaphore to lock factor table
     *          2. Apply mixing: combine roll/pitch/yaw/throttle inputs using factors
     *          3. Apply output limits and scaling
     *          4. Send PWM/digital commands to ESC outputs
     *          5. Release semaphore
     * 
     * @note Called automatically by motor output task at configured rate
     * @note Thread-safe with semaphore protection during factor updates
     * @note Respects motor output limits and arming state
     * 
     * @warning Do not call directly unless you understand motor output timing
     * 
     * Source: libraries/AP_Motors/AP_MotorsMatrix_Scripting_Dynamic.h:44
     */
    void output_to_motors() override;

protected:

    /**
     * @brief Thrust compensation override - intentionally disabled
     * 
     * @details Disables automatic thrust compensation because dynamic matrices may be used
     *          in configurations where compensation is handled externally (e.g., by the
     *          script or parent vehicle controller). For example, Quadplane tiltrotors
     *          manage thrust compensation in their own mixer logic.
     * 
     * @note Thrust compensation must be handled by the script or parent controller if needed
     * @note Standard multirotor thrust curves may not apply to dynamic configurations
     * 
     * Source: libraries/AP_Motors/AP_MotorsMatrix_Scripting_Dynamic.h:50
     */
    void thrust_compensation(void) override {};

    /**
     * @brief Get frame type string for identification
     * 
     * @details Returns a string identifier for the frame type, used in logging and
     *          ground station displays to identify the vehicle as using a dynamic matrix.
     * 
     * @return "Dynamic Matrix" string identifier
     * 
     * @note Used for logging and telemetry identification
     * 
     * Source: libraries/AP_Motors/AP_MotorsMatrix_Scripting_Dynamic.h:52
     */
    const char* _get_frame_string() const override { return "Dynamic Matrix"; }

private:

    /**
     * @brief Flag indicating if a factor table has been received
     * 
     * @details Set to true when load_factors() successfully loads a valid factor table.
     *          The motor matrix will only complete initialization and enable output after
     *          receiving a factor table, ensuring motors are never run without proper
     *          mixing configuration.
     * 
     * @note Prevents motor output before factors are configured
     * @note Checked during initialization to validate configuration completeness
     * 
     * Source: libraries/AP_Motors/AP_MotorsMatrix_Scripting_Dynamic.h:57
     */
    bool had_table;

    /**
     * @brief Semaphore for thread-safe factor table updates
     * 
     * @details Protects motor factor tables from concurrent access during updates.
     *          Prevents corruption when scripts call load_factors() while output_to_motors()
     *          is reading factors for mixing calculations. Ensures atomic factor table
     *          updates without disrupting active motor output.
     * 
     * @note Held briefly during factor loading and motor output mixing
     * @note Prevents race conditions between script updates and motor output task
     * @note Critical for safe runtime reconfiguration
     * 
     * Source: libraries/AP_Motors/AP_MotorsMatrix_Scripting_Dynamic.h:60
     */
    HAL_Semaphore _sem;

    /**
     * @brief Singleton instance pointer
     * 
     * @details Static pointer to the single instance of AP_MotorsMatrix_Scripting_Dynamic.
     *          Enforces singleton pattern to ensure only one dynamic motor matrix exists
     *          per vehicle, preventing conflicts and ensuring consistent motor control.
     * 
     * @note Set during constructor, checked to prevent multiple instances
     * @note Used by get_singleton() to provide access to scripting bindings
     * 
     * Source: libraries/AP_Motors/AP_MotorsMatrix_Scripting_Dynamic.h:62
     */
    static AP_MotorsMatrix_Scripting_Dynamic *_singleton;
};

#endif // AP_SCRIPTING_ENABLED
