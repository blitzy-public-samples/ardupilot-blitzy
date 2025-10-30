/**
 * @file   AP_MotorsHeli_Quad.h
 * @brief  Motor control class for collective pitch quadcopter-style helicopter
 * 
 * @details This file implements motor control for a quadcopter-style helicopter
 *          configuration where four rotors each have independent collective pitch
 *          control via individual swashplates. This is distinct from traditional
 *          single-rotor helicopters and standard multirotors:
 *          
 *          - Each of the 4 rotors has its own swashplate for collective pitch control
 *          - All rotors spin in the same direction (no counter-rotating pairs needed)
 *          - Control is achieved through differential collective pitch (like multicopter
 *            differential thrust, but using blade pitch instead of motor speed)
 *          - Motor layout follows standard multicopter quad patterns (X or + configuration)
 *          - Each rotor has its own Rotor Speed Controller (RSC) for governor control
 *          
 *          This configuration provides full control authority with redundancy benefits
 *          and eliminates the need for a tail rotor, while maintaining the efficiency
 *          advantages of collective pitch control.
 *          
 *          Example vehicles: Stingray 500 quad helicopter
 * 
 * @note This is a specialized configuration requiring four independent swashplate
 *       servos and four rotor speed controllers with careful mechanical setup.
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>

#include "AP_MotorsHeli.h"
#include "AP_MotorsHeli_RSC.h"

// default collective min, max and midpoints for the rear swashplate
#define AP_MOTORS_HELI_QUAD_COLLECTIVE_MIN 1100
#define AP_MOTORS_HELI_QUAD_COLLECTIVE_MAX 1900

#define AP_MOTORS_HELI_QUAD_NUM_MOTORS 4

/**
 * @class AP_MotorsHeli_Quad
 * @brief Motor control for quadcopter-style helicopter with four swashplate-controlled rotors
 * 
 * @details This class implements motor mixing and control for a quad helicopter configuration
 *          where each of the four rotors has independent collective pitch control via its own
 *          swashplate mechanism. This hybrid design combines elements of both helicopters and
 *          multirotors:
 *          
 *          **Architecture**:
 *          - Four independent swashplate assemblies (one per rotor)
 *          - Four independent RSC (Rotor Speed Controller) units for governor control
 *          - Inherits from AP_MotorsHeli base class for common helicopter functionality
 *          - Each rotor position follows standard multicopter quad layout
 *          
 *          **Control Mixing**:
 *          The class distributes pilot inputs to individual rotor collectives:
 *          - Roll: Differential collective on left/right rotors
 *          - Pitch: Differential collective on front/rear rotors  
 *          - Yaw: Differential collective in X-pattern (similar to multicopter differential thrust)
 *          - Throttle: Common collective added to all rotors
 *          
 *          Mixing is performed by calculate_roll_pitch_collective_factors() which computes
 *          the contribution factors for each rotor, then move_actuators() applies these
 *          factors to generate individual rotor collective commands.
 *          
 *          **Advantages**:
 *          - Full control authority on all axes without tail rotor
 *          - Redundancy: Can potentially survive single rotor failure
 *          - Efficiency: Collective pitch control more efficient than pure motor speed modulation
 *          - Symmetrical design simplifies dynamics compared to traditional helicopter
 *          
 *          **Configuration Requirements**:
 *          - All four swashplates must be mechanically configured and calibrated
 *          - Collective pitch range limits (H_COL_MAX, H_COL_MIN) must be set per rotor
 *          - All rotors should spin in the same direction (typically)
 *          - Rotor speed governors must be configured for constant head speed
 * 
 * @note Each rotor operates independently with its own swashplate servo driving collective
 *       pitch. There are no cyclic controls - only collective pitch is varied per rotor.
 * 
 * @note Motor/rotor numbering typically follows multicopter conventions:
 *       Motor 1: Front-right, Motor 2: Rear-left, Motor 3: Front-left, Motor 4: Rear-right
 *       (for X configuration)
 * 
 * @warning All four swashplates must be correctly configured, calibrated, and mechanically
 *          sound before flight. Failure of any swashplate servo can result in loss of control.
 * 
 * @warning Collective range limits are critical - over-pitching the blades can cause excessive
 *          loads, vibration, or blade stall. Conservative limits recommended initially.
 * 
 * @warning This configuration has high mechanical complexity. Regular inspection and maintenance
 *          of all four swashplate mechanisms, servos, and linkages is essential for safe operation.
 */
class AP_MotorsHeli_Quad : public AP_MotorsHeli {
public:
    /**
     * @brief Constructor for quad helicopter motor control
     * 
     * @param[in] speed_hz Update rate for servo outputs in Hz (default: AP_MOTORS_HELI_SPEED_DEFAULT)
     * 
     * @details Initializes the quad helicopter motor controller with specified update rate.
     *          Sets up parameter object defaults for all quad-specific parameters.
     */
    AP_MotorsHeli_Quad(uint16_t speed_hz = AP_MOTORS_HELI_SPEED_DEFAULT) :
        AP_MotorsHeli(speed_hz)
    {
        AP_Param::setup_object_defaults(this, var_info);
    };

    /**
     * @brief Set update rate for all four rotor servo outputs
     * 
     * @param[in] speed_hz Desired update frequency in Hz (typically 50-400 Hz)
     * 
     * @details Sets the PWM update rate for all servo outputs controlling the four
     *          swashplates. Higher rates provide better control response but may
     *          exceed servo bandwidth. Common values: 50Hz (analog servos), 
     *          200-400Hz (digital servos).
     *          
     *          This rate applies to all swashplate servos for all four rotors.
     * 
     * @note All four rotors will use the same update rate. Individual rotor rate
     *       control is not supported.
     */
    void set_update_rate( uint16_t speed_hz ) override;

    /**
     * @brief Send control outputs to all four swashplates and RSC controllers
     * 
     * @details This is the main output function called by the motor library at the
     *          scheduled update rate. It performs the following sequence:
     *          
     *          1. Calculates individual rotor collective commands from mixing
     *          2. Applies collective limits to prevent over-pitching
     *          3. Outputs PWM commands to all four swashplate servos
     *          4. Updates all four RSC (Rotor Speed Controller) outputs
     *          5. Logs motor outputs if logging is enabled
     *          
     *          This function is called at the main loop rate (typically 400 Hz for copters).
     * 
     * @note This function outputs to both swashplate servos (collective pitch) and
     *       motor ESCs/governors (rotor speed control) for all four rotors.
     * 
     * @warning This function must complete quickly as it runs in the main control loop.
     *          Typical execution time should be under 100 microseconds.
     */
    void output_to_motors() override;

    /**
     * @brief Recalculate various scaling factors for motor control
     * 
     * @details Computes scaling factors used in motor mixing and output generation.
     *          Called during initialization and when parameters change. Recalculates:
     *          
     *          - Collective pitch scaling factors
     *          - Roll/pitch/yaw mixing factors for each rotor
     *          - Output range scaling based on configured limits
     *          
     *          This function calls calculate_roll_pitch_collective_factors() to compute
     *          the mixing matrix for the quad configuration.
     * 
     * @note This is typically called during initialization and when parameters are
     *       modified through ground station or parameter reload.
     */
    void calculate_scalars() override;

    /**
     * @brief Recalculate scalars that are allowed to change while vehicle is armed
     * 
     * @details Recomputes scaling factors that can safely be changed during flight.
     *          This is a subset of calculate_scalars() that only updates parameters
     *          which are safe to modify while armed. Called periodically during flight.
     * 
     * @note This runs less frequently than the main control loop, typically at 1-10 Hz.
     * 
     * @warning Only parameters that are safe to change in flight should be recalculated
     *          here. Changing fundamental mixing or geometry parameters during flight
     *          could cause control instability.
     */
    void calculate_armed_scalars() override;

    /**
     * @brief Test function to move all swashplate servos through their full range
     * 
     * @details Cycles all four swashplate servos through their complete range of motion
     *          for mechanical setup and troubleshooting. Used during initial setup to:
     *          
     *          - Verify servo directions are correct
     *          - Check mechanical range and binding
     *          - Identify servo/linkage issues
     *          - Validate collective pitch limits
     *          
     *          Servos are moved slowly and sequentially to allow visual inspection.
     * 
     * @note This function should only be called when vehicle is disarmed and in a
     *       safe configuration. Main rotor blades should be removed during initial testing.
     * 
     * @warning Ensure all linkages are secure and rotors are clear of obstructions
     *          before running servo test. Full collective pitch range will be commanded.
     */
    void servo_test() override;

#if HAL_LOGGING_ENABLED
    /**
     * @brief Log motor control outputs for all four rotors
     * 
     * @details Writes motor control data to onboard log for post-flight analysis.
     *          Logs include:
     *          
     *          - Individual rotor collective commands (all 4 rotors)
     *          - Rotor speed controller outputs
     *          - Mixing factor contributions
     *          - Control input demands (roll, pitch, yaw, throttle)
     *          
     *          Called at 10 Hz to balance data resolution with log storage constraints.
     * 
     * @note Logging must be enabled (HAL_LOGGING_ENABLED) and LOG_BITMASK must
     *       include RCOUT logging for this data to be recorded.
     */
    void Log_Write(void) override;
#endif

    /**
     * @brief Parameter table for quad helicopter configuration
     * 
     * @details Defines all user-configurable parameters for the quad helicopter.
     *          Key parameters include:
     *          
     *          - H_COL_MAX: Maximum collective pitch (PWM microseconds)
     *          - H_COL_MIN: Minimum collective pitch (PWM microseconds)  
     *          - H_COL_MID: Mid-point collective pitch
     *          - H_CYC_MAX: Maximum cyclic input per rotor (if applicable)
     *          - H_RSC_MODE: Rotor speed control mode (governor settings)
     *          - H_RSC_SETPOINT: Target rotor RPM for governor
     *          
     *          Parameters are accessible via ground control station and stored in EEPROM.
     * 
     * @note Parameter changes typically require calculate_scalars() to be called
     *       to update mixing and scaling factors.
     */
    static const struct AP_Param::GroupInfo var_info[];

protected:

    /**
     * @brief Initialize hardware outputs for all four rotors
     * 
     * @details Configures hardware outputs for the quad helicopter system:
     *          
     *          - Maps swashplate servo channels for all four rotors
     *          - Configures RSC (Rotor Speed Controller) outputs for all rotors
     *          - Sets initial servo positions to safe neutral values
     *          - Initializes PWM output ranges for all channels
     *          
     *          Called once during system initialization before first use.
     * 
     * @note This function must complete successfully for the motor controller to
     *       function. Failure typically indicates hardware configuration issues.
     */
    void init_outputs () override;

    /**
     * @brief Update rotor speed controllers for all four rotors
     * 
     * @param[in] state Current rotor control state (IDLE, RUNUP, ACTIVE, etc.)
     * 
     * @details Sends commands to all four RSC (Rotor Speed Controller) units based on
     *          the current flight state. The state parameter controls governor behavior:
     *          
     *          - IDLE: Rotors stopped or at idle speed
     *          - RUNUP: Rotors spinning up to flight speed
     *          - ACTIVE: Normal flight operation with governor maintaining RPM
     *          - SHUTDOWN: Controlled rotor slowdown
     *          
     *          Each rotor's RSC operates independently but typically at the same
     *          target speed for balanced lift and control characteristics.
     * 
     * @note All four rotors typically run at the same head speed for optimal control
     *       mixing and predictable flight dynamics.
     */
    void update_motor_control(AP_MotorsHeli_RSC::RotorControlState state) override;

    /**
     * @brief Compute mixing matrix for quad helicopter configuration
     * 
     * @details Calculates the mixing factors that distribute roll, pitch, yaw, and throttle
     *          commands to individual rotor collective pitch values. The mixing is similar
     *          to standard multicopter mixing but applied to collective pitch instead of
     *          motor speeds.
     *          
     *          For a standard X-configuration quad:
     *          
     *          **Roll Mixing** (differential left/right collective):
     *          - Motor 1 (Front-Right): +1.0 (increases collective for right roll)
     *          - Motor 2 (Rear-Left): -1.0 (decreases collective)
     *          - Motor 3 (Front-Left): -1.0 (decreases collective)
     *          - Motor 4 (Rear-Right): +1.0 (increases collective)
     *          
     *          **Pitch Mixing** (differential front/rear collective):
     *          - Motor 1 (Front-Right): +1.0 (increases collective for forward pitch)
     *          - Motor 2 (Rear-Left): -1.0 (decreases collective)
     *          - Motor 3 (Front-Left): +1.0 (increases collective)
     *          - Motor 4 (Rear-Right): -1.0 (decreases collective)
     *          
     *          **Yaw Mixing** (differential diagonal collective):
     *          - Motor 1 (Front-Right): +1.0
     *          - Motor 2 (Rear-Left): +1.0
     *          - Motor 3 (Front-Left): -1.0
     *          - Motor 4 (Rear-Right): -1.0
     *          
     *          **Collective (Throttle)**: Equal contribution +1.0 for all rotors
     *          
     *          Factors are normalized and stored in member arrays for use by move_actuators().
     * 
     * @note This mixing approach treats collective pitch like multicopter thrust - higher
     *       collective pitch on one side creates a moment just like higher thrust does.
     * 
     * @note All rotors spin in the same direction, so yaw control is purely through
     *       differential collective, not through torque reaction differences.
     */
    void calculate_roll_pitch_collective_factors ();

    /**
     * @brief Distribute control inputs to individual rotor collective commands
     * 
     * @param[in] roll_out  Desired roll moment (normalized -1.0 to 1.0)
     * @param[in] pitch_out Desired pitch moment (normalized -1.0 to 1.0)
     * @param[in] coll_in   Desired collective/throttle (normalized 0.0 to 1.0)
     * @param[in] yaw_out   Desired yaw moment (normalized -1.0 to 1.0)
     * 
     * @details This is the core mixing function that converts pilot/autopilot control
     *          demands into individual rotor collective pitch commands. For each rotor:
     *          
     *          rotor_collective[i] = (roll_out * _rollFactor[i]) +
     *                                 (pitch_out * _pitchFactor[i]) +
     *                                 (yaw_out * _yawFactor[i]) +
     *                                 (coll_in * _collectiveFactor[i])
     *          
     *          The resulting values are then:
     *          1. Scaled to the configured collective range (H_COL_MIN to H_COL_MAX)
     *          2. Limited to prevent over-pitching
     *          3. Converted to PWM microsecond values
     *          4. Sent to the corresponding swashplate servo
     *          
     *          This function is called at the main control loop rate (typically 400 Hz).
     * 
     * @note Unlike traditional helicopters with a single swashplate controlling cyclic
     *       and collective, this moves four independent collectives - no cyclic control.
     * 
     * @note The mixing factors (_rollFactor, _pitchFactor, etc.) are pre-computed by
     *       calculate_roll_pitch_collective_factors() and remain constant during flight.
     * 
     * @warning Output limiting is critical - excessive collective pitch can cause blade
     *          stall, excessive loads, or mechanical damage. Conservative limits recommended.
     */
    void move_actuators(float roll_out, float pitch_out, float coll_in, float yaw_out)  override;

    /**
     * @brief Get frame type string identifier
     * 
     * @return "HELI_QUAD" frame type identifier string
     * 
     * @details Returns the frame type string used for logging, telemetry, and
     *          ground station display. Identifies this as a quad helicopter.
     */
    const char* _get_frame_string() const override { return "HELI_QUAD"; }

    // Mixing factor arrays - one element per rotor
    
    /**
     * @brief Roll mixing factors for each rotor
     * 
     * @details Defines how much each rotor's collective contributes to roll moment.
     *          Positive values increase collective for right roll, negative for left roll.
     *          Typically: front-right/rear-right = +1.0, front-left/rear-left = -1.0
     */
    float _rollFactor[AP_MOTORS_HELI_QUAD_NUM_MOTORS];
    
    /**
     * @brief Pitch mixing factors for each rotor
     * 
     * @details Defines how much each rotor's collective contributes to pitch moment.
     *          Positive values increase collective for forward pitch, negative for aft pitch.
     *          Typically: front rotors = +1.0, rear rotors = -1.0
     */
    float _pitchFactor[AP_MOTORS_HELI_QUAD_NUM_MOTORS];
    
    /**
     * @brief Collective (throttle) factors for each rotor
     * 
     * @details Defines how throttle command distributes to each rotor's collective.
     *          Typically all rotors have +1.0 (equal collective increase with throttle).
     *          Could be adjusted for asymmetric configurations or CoG compensation.
     */
    float _collectiveFactor[AP_MOTORS_HELI_QUAD_NUM_MOTORS];
    
    /**
     * @brief Yaw mixing factors for each rotor
     * 
     * @details Defines how much each rotor's collective contributes to yaw moment.
     *          Creates differential collective in a pattern that produces yaw torque.
     *          For X-config: front-right/rear-left = +1.0, front-left/rear-right = -1.0
     */
    float _yawFactor[AP_MOTORS_HELI_QUAD_NUM_MOTORS];
    
    /**
     * @brief Final output collective commands for each rotor
     * 
     * @details Stores the computed collective pitch command for each rotor after mixing,
     *          scaling, and limiting. Values are in PWM microseconds ready for servo output.
     *          Updated by move_actuators() and sent to servos by output_to_motors().
     */
    float _out[AP_MOTORS_HELI_QUAD_NUM_MOTORS];
};


