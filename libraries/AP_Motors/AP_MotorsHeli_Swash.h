/**
 * @file AP_MotorsHeli_Swash.h
 * @brief Swashplate control library for traditional helicopters with CCPM (Cyclic/Collective Pitch Mixing)
 * 
 * @details This library implements swashplate servo mixing for traditional helicopters.
 *          The swashplate is the mechanical linkage that translates servo movements into
 *          blade pitch control. CCPM (Cyclic/Collective Pitch Mixing) is the process of
 *          mixing pilot roll, pitch, and collective inputs into individual servo positions
 *          based on the geometric configuration of the swashplate.
 * 
 *          Key features:
 *          - Supports multiple swashplate configurations (H1, H3, H4 layouts)
 *          - Geometric servo mixing based on azimuth positions
 *          - Optional servo linearization for mechanical linkage compensation
 *          - Rotor phase angle correction for head dynamics
 *          - Multi-instance support for dual swashplate helicopters
 * 
 * Source: libraries/AP_Motors/AP_MotorsHeli_Swash.h:1-103
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_Param/AP_Param.h>
#include <AP_Logger/AP_Logger_config.h>

/**
 * @enum SwashPlateType
 * @brief Swashplate servo configuration types defining geometric servo layout
 * 
 * @details Swashplate type determines the number of servos, their azimuth positions,
 *          and the mixing factors used to translate roll/pitch/collective inputs into
 *          individual servo movements. The type must match the physical helicopter
 *          swashplate configuration.
 */
enum SwashPlateType {
    SWASHPLATE_TYPE_H3 = 0,  ///< 3-servo swashplate with servos at 0°, 120°, 240° (standard 120° spacing)
    SWASHPLATE_TYPE_H1,      ///< 1-servo direct cyclic control (typically for flybar heads with mechanical mixing)
    SWASHPLATE_TYPE_H3_140,  ///< 3-servo swashplate with 140° servo spacing (rear servo at 0°, front servos at 140° and 220°)
    SWASHPLATE_TYPE_H3_120,  ///< 3-servo swashplate with 120° servo spacing (front servo at 0°, rear servos at 120° and 240°)
    SWASHPLATE_TYPE_H4_90,   ///< 4-servo swashplate with servos at 90° intervals (0°, 90°, 180°, 270°)
    SWASHPLATE_TYPE_H4_45    ///< 4-servo swashplate with 45° offset (45°, 135°, 225°, 315°)
};

/**
 * @class AP_MotorsHeli_Swash
 * @brief Manages swashplate servo mixing for traditional helicopters
 * 
 * @details This class implements CCPM (Cyclic/Collective Pitch Mixing) to convert pilot
 *          roll, pitch, and collective inputs into individual servo positions based on
 *          the geometric configuration of the swashplate.
 * 
 *          CCPM Mixing Process:
 *          1. Pilot inputs (roll, pitch, collective) are received in normalized form
 *          2. Geometric mixing factors are calculated based on servo azimuth positions
 *          3. Each servo position is computed using trigonometric mixing
 *          4. Optional servo linearization compensates for mechanical non-linearity
 *          5. Phase angle correction accounts for rotor head dynamics
 *          6. Final servo positions are output to motor channels
 * 
 *          The class supports multiple swashplate configurations (H1, H3, H4) and
 *          can handle dual swashplate helicopters through multi-instance support.
 * 
 *          Thread Safety: Not thread-safe, should be called from main thread only
 * 
 * @note Servo azimuth positions are specified in centidegrees with 0° = front of helicopter
 * @note Roll/pitch inputs range from -1 to +1, collective from 0 to 1
 */
class AP_MotorsHeli_Swash {
public:

    /**
     * @brief Constructor for swashplate controller
     * 
     * @param mot_0 Motor function number for first servo
     * @param mot_1 Motor function number for second servo
     * @param mot_2 Motor function number for third servo
     * @param mot_3 Motor function number for fourth servo (unused for H1/H3 types)
     * @param instance Swashplate instance number for multi-swashplate helicopters (used for logging)
     */
    AP_MotorsHeli_Swash(uint8_t mot_0, uint8_t mot_1, uint8_t mot_2, uint8_t mot_3, uint8_t instance);

    /**
     * @brief Configure swashplate settings from updated parameters
     * 
     * @details Updates the swashplate configuration based on parameter changes.
     *          This includes swashplate type, servo positions, collective direction,
     *          linearization settings, and phase angle. Recalculates geometric mixing
     *          factors based on the configured servo positions.
     * 
     *          Should be called after parameter updates or on initialization.
     * 
     * @note Recalculates roll, pitch, and collective mixing factors
     * @warning Changing swashplate type requires ground testing before flight
     */
    void configure();

    /**
     * @brief Get current swashplate type
     * 
     * @return SwashPlateType Currently configured swashplate servo layout
     */
    SwashPlateType get_swash_type() const { return _swash_type; }

    /**
     * @brief Calculate servo positions from pilot inputs using CCPM mixing
     * 
     * @details Performs geometric mixing to convert roll, pitch, and collective inputs
     *          into individual servo positions. The mixing uses trigonometric calculations
     *          based on servo azimuth positions to determine how much each input contributes
     *          to each servo's movement.
     * 
     *          Mixing formula for each servo:
     *          servo_pos = (roll * rollFactor) + (pitch * pitchFactor) + (collective * collectiveFactor)
     * 
     *          If servo linearization is enabled, applies compensation for mechanical
     *          linkage non-linearity. Phase angle correction is applied to compensate
     *          for rotor head dynamics and blade phasing effects.
     * 
     * @param[in] roll Normalized roll input in range -1.0 to +1.0 (negative = roll left)
     * @param[in] pitch Normalized pitch input in range -1.0 to +1.0 (negative = pitch forward)
     * @param[in] collective Normalized collective input in range 0.0 to 1.0 (0 = full down, 1 = full up)
     * 
     * @note Called at main loop rate (typically 400Hz for helicopters)
     * @note Phase angle correction accounts for rotor head dynamics where pitch input may induce roll
     * @warning Direct servo control - incorrect mixing parameters can cause loss of control
     */
    void calculate(float roll, float pitch, float collective);

    /**
     * @brief Output calculated servo positions to motor channels
     * 
     * @details Writes the servo positions calculated by calculate() to the actual
     *          servo output channels. Must be called after calculate() to update
     *          physical servo positions.
     * 
     * @note Only outputs to enabled servos based on swashplate configuration
     * @warning Direct servo output - ensure calculate() was called with valid inputs
     */
    void output();

    /**
     * @brief Get rotor phase angle correction
     * 
     * @details Phase angle compensates for rotor head dynamics where cyclic input
     *          in one axis may induce movement in another axis due to blade phasing,
     *          rotor precession, or mechanical linkage effects. A positive phase angle
     *          advances the swashplate position relative to pilot input.
     * 
     * @return int16_t Phase angle correction in centidegrees (0.01 degrees)
     * 
     * @note Typical values range from -9000 to +9000 centidegrees (-90° to +90°)
     * @note Sign depends on rotor rotation direction and mechanical linkage
     */
    int16_t get_phase_angle() const { return _phase_angle; }

    /**
     * @brief Get output channel mask for active servos
     * 
     * @details Returns a bitmask indicating which motor function outputs are used
     *          by this swashplate instance. Used for output allocation and conflict
     *          detection in multi-swashplate configurations.
     * 
     * @return uint32_t Bitmask of active servo outputs (bit n set = motor function n in use)
     */
    uint32_t get_output_mask() const;

#if HAL_LOGGING_ENABLED
    /**
     * @brief Write swashplate state to dataflash log
     * 
     * @details Logs swashplate inputs, outputs, and configuration for post-flight analysis.
     *          Creates SWSH log message containing servo positions, mixing inputs, and
     *          scaling factors.
     * 
     * @param[in] cyclic_scaler Cyclic (roll/pitch) scaling factor applied to inputs
     * @param[in] col_ang_min Minimum collective blade angle in degrees
     * @param[in] col_ang_max Maximum collective blade angle in degrees
     * @param[in] col_min Minimum collective servo position
     * @param[in] col_max Maximum collective servo position
     */
    void write_log(float cyclic_scaler, float col_ang_min, float col_ang_max, int16_t col_min, int16_t col_max) const;
#endif

    // var_info
    static const struct AP_Param::GroupInfo var_info[];

private:

    /**
     * @brief Apply servo linearization to compensate for mechanical linkage non-linearity
     * 
     * @details Many helicopter swashplate linkages have non-linear mechanical response
     *          where servo arm position does not translate linearly to swashplate movement.
     *          This function applies a compensation curve if linearization is enabled.
     * 
     * @param[in] input Normalized servo input in range -1.0 to 1.0
     * @return float Linearized servo output in range -1.0 to 1.0
     * 
     * @note Only applied if _make_servo_linear is true
     */
    float get_linear_servo_output(float input) const;

    /**
     * @brief Calculate geometric mixing factors for all servos based on swashplate type
     * 
     * @details Computes roll, pitch, and collective mixing factors for each servo based on
     *          their azimuth positions on the swashplate. Uses trigonometric calculations
     *          to determine how much each input axis contributes to each servo's movement.
     * 
     *          For a servo at azimuth angle θ:
     *          - rollFactor = cos(θ)
     *          - pitchFactor = sin(θ)
     *          - collectiveFactor = 1.0 (or -1.0 if collective reversed)
     * 
     * @note Called by configure() when swashplate parameters change
     */
    void calculate_roll_pitch_collective_factors();

    /**
     * @brief Configure servo mixing using angle-based positioning
     * 
     * @param[in] num Servo number (0-3)
     * @param[in] angle Servo azimuth position in degrees (0° = front of helicopter)
     * @param[in] collective Collective contribution factor (typically 1.0 or -1.0)
     */
    void add_servo_angle(uint8_t num, float angle, float collective);

    /**
     * @brief Configure servo mixing using raw mixing factors
     * 
     * @param[in] num Servo number (0-3)
     * @param[in] roll Roll mixing factor for this servo
     * @param[in] pitch Pitch mixing factor for this servo
     * @param[in] collective Collective mixing factor for this servo
     */
    void add_servo_raw(uint8_t num, float roll, float pitch, float collective);

    /**
     * @brief Write servo position to output channel
     * 
     * @param[in] chan Motor function number for output
     * @param[in] swash_in Normalized servo position in range -1.0 to 1.0
     */
    void rc_write(uint8_t chan, float swash_in);

    /**
     * @enum CollectiveDirection
     * @brief Collective control direction (normal or reversed)
     */
    enum CollectiveDirection {
        COLLECTIVE_DIRECTION_NORMAL = 0,    ///< Increasing collective input raises swashplate (standard)
        COLLECTIVE_DIRECTION_REVERSED       ///< Increasing collective input lowers swashplate (reversed for inverted flight or mechanical setup)
    };

    static const uint8_t _max_num_servos {4};  ///< Maximum number of swashplate servos (H4 configuration)

    // Currently configured setup
    SwashPlateType       _swash_type;                 ///< Currently configured swashplate type (H1, H3, or H4)
    CollectiveDirection  _collective_direction;       ///< Collective control direction (normal or reversed)
    bool                 _make_servo_linear;          ///< Enable servo linearization to compensate for mechanical non-linearity

    /**
     * @name CCPM Mixing Variables
     * @brief Internal variables for geometric servo mixing calculations
     * @{
     */
    bool                 _enabled[_max_num_servos];                 ///< True if servo is active for current swashplate type
    float                _rollFactor[_max_num_servos];              ///< Roll axis geometric mixing coefficient (computed from servo azimuth)
    float                _pitchFactor[_max_num_servos];             ///< Pitch axis geometric mixing coefficient (computed from servo azimuth)
    float                _collectiveFactor[_max_num_servos];        ///< Collective axis mixing coefficient (typically ±1.0)
    float                _output[_max_num_servos];                  ///< Calculated servo output position (-1.0 to 1.0)
    const uint8_t        _motor_num[_max_num_servos];               ///< Motor function numbers for servo outputs
    const uint8_t        _instance;                                 ///< Swashplate instance number (for multi-swashplate helicopters)
    /** @} */

    /**
     * @name Logging Variables
     * @brief Variables cached for dataflash logging
     * @{
     */
    float _roll_input;                  ///< Last roll input provided to calculate()
    float _pitch_input;                 ///< Last pitch input provided to calculate()
    float _collective_input_scaled;     ///< Last scaled collective input
    /** @} */

    /**
     * @name Configuration Parameters
     * @brief User-configurable parameters for swashplate setup
     * @{
     */
    AP_Int8  _swashplate_type;                   ///< Swashplate type setting (SwashPlateType enum)
    AP_Int8  _swash_coll_dir;                    ///< Collective direction (CollectiveDirection enum: 0=normal, 1=reversed)
    AP_Int8  _linear_swash_servo;                ///< Enable servo linearization (0=disabled, 1=enabled)
    AP_Int8  enable;                              ///< Enable this swashplate instance (0=disabled, 1=enabled)
    AP_Int16 _servo1_pos;                        ///< Servo 1 azimuth position in centidegrees (0 = front of helicopter, positive = clockwise)
    AP_Int16 _servo2_pos;                        ///< Servo 2 azimuth position in centidegrees (0 = front of helicopter, positive = clockwise)
    AP_Int16 _servo3_pos;                        ///< Servo 3 azimuth position in centidegrees (0 = front of helicopter, positive = clockwise)
    AP_Int16 _phase_angle;                       ///< Rotor head phase angle correction in centidegrees. Compensates for rotor dynamics where pitch input induces roll. Can be negative depending on rotor rotation and mechanical linkage.
    /** @} */

};

