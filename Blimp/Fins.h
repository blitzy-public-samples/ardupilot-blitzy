/**
 * @file Fins.h
 * @brief Fin actuation class definition for lighter-than-air blimp vehicles
 * 
 * @details This file defines the Fins class which provides sinusoidal fin control
 *          unique to lighter-than-air vehicles. The class converts horizontal acceleration
 *          commands to fin flapping commands, implementing a biomimetic approach where
 *          fins oscillate to generate thrust and control forces in multiple axes.
 *          
 *          The Fins class manages up to NUM_FINS (currently 4) independent fin actuators,
 *          each with configurable amplitude, offset, and frequency parameters to achieve
 *          coordinated movement for right/left, forward/backward, yaw, and altitude control.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 * 
 * Source: Blimp/Fins.h
 */

#pragma once
#include <AP_Notify/AP_Notify.h>

extern const AP_HAL::HAL& hal;

/// @brief Maximum number of fins that can be controlled simultaneously
#define NUM_FINS 4

/// @brief RC input scaling factor for converting PWM values
#define RC_SCALE 1000
/**
 * @class Fins
 * @brief Sinusoidal fin control system for lighter-than-air blimp vehicles
 * 
 * @details The Fins class implements a unique control system for blimps using oscillating
 *          fins (similar to fish locomotion) to generate thrust and control forces. Unlike
 *          traditional multicopter motor control, this system converts commanded accelerations
 *          in right/left, forward/backward, yaw, and vertical axes into sinusoidal fin
 *          movements with variable amplitude, offset, and frequency.
 *          
 *          Key features:
 *          - Supports up to NUM_FINS independent fin actuators
 *          - Each fin has configurable amplitude and offset factors for each control axis
 *          - Sinusoidal output generation at configurable frequency (freq_hz parameter)
 *          - Coordinated multi-fin control for 4-DOF movement (right, front, yaw, down)
 *          - Arm/disarm state management with safety interlocks
 *          - Integration with ArduPilot's AP_Param system for configuration
 *          
 *          Lifecycle:
 *          1. Construction with loop rate
 *          2. setup_fins() - configure fin actuators and mixing factors
 *          3. output() - called at loop rate to generate sinusoidal servo commands
 *          
 *          Thread Safety: This class is designed to be called from the main vehicle thread
 *          at the configured loop rate (typically 400Hz).
 * 
 * @note This is specialized for airfish-style blimp configurations with oscillating fins
 * @warning Fin configuration (add_fin parameters) must match physical vehicle layout
 * 
 * @see Blimp class for vehicle-level integration
 * @see Loiter mode for example usage
 */
class Fins
{
public:
    friend class Blimp;
    friend class Loiter;

    /**
     * @enum motor_frame_class
     * @brief Frame class enumeration for blimp configurations
     */
    enum motor_frame_class {
        MOTOR_FRAME_UNDEFINED = 0,  ///< Undefined/unconfigured frame
        MOTOR_FRAME_AIRFISH = 1,    ///< Airfish-style oscillating fin configuration
    };
    
    /**
     * @enum motor_frame_type
     * @brief Frame type enumeration for specific blimp variants
     */
    enum motor_frame_type {
        MOTOR_FRAME_TYPE_AIRFISH = 1,  ///< Airfish frame type with oscillating fins
    };

    /**
     * @brief Construct a new Fins object
     * 
     * @param[in] loop_rate Rate in Hz at which output() function will be called (normally 400Hz)
     * 
     * @details Initializes the fin control system with the specified loop rate. This rate
     *          determines the update frequency for sinusoidal fin position calculations.
     */
    Fins(uint16_t loop_rate);

    /**
     * @brief Parameter table for AP_Param system
     * 
     * @details This table defines configurable parameters for the fin system including:
     *          - freq_hz: Base frequency for fin oscillation
     *          - turbo_mode: Enable/disable enhanced oscillation mode
     *          
     *          Parameters are stored in EEPROM and accessible via ground control station.
     */
    static const struct AP_Param::GroupInfo        var_info[];

    /**
     * @brief Check if fin system initialized successfully
     * 
     * @return true Always returns true (initialization always succeeds for fin system)
     * 
     * @note Currently always returns true - kept for API compatibility with motor classes
     */
    bool initialised_ok() const
    {
        return true;
    }

    /**
     * @brief Set the armed state of the fin system
     * 
     * @param[in] arm true to arm (enable fin movement), false to disarm (disable)
     * 
     * @details When armed state changes, updates internal _armed flag and notifies
     *          the AP_Notify system to update LED/buzzer indicators. Fins only produce
     *          movement when armed and interlock is enabled.
     * 
     * @see armed() const
     * @see _interlock
     */
    void armed(bool arm)
    {
        if (arm != _armed) {
            _armed = arm;
            AP_Notify::flags.armed = arm;
        }

    }
    
    /**
     * @brief Get current armed state
     * 
     * @return true if fin system is armed (fins can move)
     * @return false if fin system is disarmed (fins will not move)
     * 
     * @see armed(bool)
     */
    bool armed() const
    {
        return _armed;
    }

protected:
    // Internal state variables
    
    /// @brief Rate in Hz at which output() function is called (normally 400Hz)
    const uint16_t      _loop_rate;
    
    /// @brief Speed in Hz to send updates to servo outputs
    uint16_t            _speed_hz;
    
    /// @brief Last throttle input from set_throttle_avg_max (currently unused in fin system)
    float               _throttle_avg_max;

    /// @brief Current timestamp in seconds for sinusoidal calculations
    float               _time;

    /// @brief Armed state: false if disarmed, true if armed
    bool _armed;

    // Per-fin sinusoidal control parameters
    
    /// @brief Current amplitude for each fin's sinusoidal oscillation
    float              _amp[NUM_FINS];
    
    /// @brief Current offset (bias) for each fin's sinusoidal oscillation
    float              _off[NUM_FINS];
    
    /// @brief Frequency multiplier for each fin relative to base freq_hz
    float              _freq[NUM_FINS];
    
    /// @brief Current calculated servo position for each fin
    float              _pos[NUM_FINS];

    // Amplitude mixing factors: how much each control axis affects each fin's amplitude
    
    /// @brief Right/left control amplitude contribution factor for each fin
    float               _right_amp_factor[NUM_FINS];
    
    /// @brief Forward/backward control amplitude contribution factor for each fin
    float               _front_amp_factor[NUM_FINS];
    
    /// @brief Vertical (down) control amplitude contribution factor for each fin
    float               _down_amp_factor[NUM_FINS];
    
    /// @brief Yaw control amplitude contribution factor for each fin
    float               _yaw_amp_factor[NUM_FINS];

    // Offset mixing factors: how much each control axis affects each fin's offset
    
    /// @brief Right/left control offset contribution factor for each fin
    float               _right_off_factor[NUM_FINS];
    
    /// @brief Forward/backward control offset contribution factor for each fin
    float               _front_off_factor[NUM_FINS];
    
    /// @brief Vertical (down) control offset contribution factor for each fin
    float               _down_off_factor[NUM_FINS];
    
    /// @brief Yaw control offset contribution factor for each fin
    float               _yaw_off_factor[NUM_FINS];

    /// @brief Number of fins that have been configured via add_fin()
    int8_t              _num_added;

    // Public control inputs and configuration
    // @todo These should probably become private in future with accessor methods
public:
    /// @brief Right movement command: positive = right, negative = left, range [-1.0, +1.0]
    float               right_out;
    
    /// @brief Forward movement command: positive = forward, negative = backward, range [-1.0, +1.0]
    float               front_out;
    
    /// @brief Yaw command: positive = clockwise, negative = counter-clockwise, range [-1.0, +1.0]
    float               yaw_out;
    
    /// @brief Vertical command: positive = down, negative = up, range [-1.0, +1.0]
    float               down_out;

    /// @brief Base frequency for fin oscillation in Hz (AP_Param configurable)
    AP_Float            freq_hz;
    
    /// @brief Turbo mode enable flag: 0 = normal, 1 = enhanced oscillation mode (AP_Param configurable)
    AP_Int8             turbo_mode;

    /// @brief Motor interlock state: true = fins can move, false = fins disabled
    bool _interlock;
    
    /// @brief Initialization success flag: true if initialization was successful
    bool _initialised_ok;

    /**
     * @brief Set all fin outputs to minimum (neutral) position
     * 
     * @details Sends neutral PWM values to all configured fin servos. Typically called
     *          when disarmed or during initialization to ensure fins are in safe position.
     */
    void output_min();

    /**
     * @brief Add and configure a fin actuator to the control system
     * 
     * @param[in] fin_num     Fin index (0 to NUM_FINS-1)
     * @param[in] right_amp_fac  Right control amplitude factor for this fin
     * @param[in] front_amp_fac  Forward control amplitude factor for this fin
     * @param[in] yaw_amp_fac    Yaw control amplitude factor for this fin
     * @param[in] down_amp_fac   Vertical control amplitude factor for this fin
     * @param[in] right_off_fac  Right control offset factor for this fin
     * @param[in] front_off_fac  Forward control offset factor for this fin
     * @param[in] yaw_off_fac    Yaw control offset factor for this fin
     * @param[in] down_off_fac   Vertical control offset factor for this fin
     * 
     * @details Configures a fin with mixing factors that determine how each control axis
     *          affects the fin's amplitude and offset. Positive factors produce positive
     *          contribution, negative factors reverse the contribution. Factors are typically
     *          in range [-1.0, +1.0] but can exceed for gain adjustment.
     *          
     *          The amplitude factors control the oscillation magnitude, while offset factors
     *          control the bias position. This allows each fin to contribute differently to
     *          each control axis based on its physical location and orientation.
     * 
     * @note Must be called during setup_fins() before fins are operational
     * @warning fin_num must be < NUM_FINS to avoid array overflow
     */
    void add_fin(int8_t fin_num, float right_amp_fac, float front_amp_fac, float yaw_amp_fac, float down_amp_fac,
                 float right_off_fac, float front_off_fac, float yaw_off_fac, float down_off_fac);

    /**
     * @brief Initialize and configure all fins for the vehicle
     * 
     * @details Calls add_fin() for each physical fin actuator on the vehicle to configure
     *          the mixing factors based on the frame type. This is vehicle-specific and
     *          defines how the fins are arranged and oriented.
     *          
     *          Must be called once during vehicle initialization before fins can be used.
     * 
     * @see add_fin()
     */
    void setup_fins();

    /**
     * @brief Generate and output sinusoidal servo commands to all fins
     * 
     * @details This is the main update function called at _loop_rate (typically 400Hz).
     *          Performs the following:
     *          1. Updates timestamp for sinusoidal calculations
     *          2. Calculates amplitude for each fin by mixing right_out, front_out, yaw_out, down_out
     *             using the amplitude factors
     *          3. Calculates offset for each fin using the offset factors
     *          4. Generates sinusoidal position: pos = amp * sin(2*pi*freq*time) + offset
     *          5. Converts positions to PWM values and sends to servo outputs
     *          
     *          Only produces output if armed and interlock enabled, otherwise outputs neutral.
     * 
     * @note Called at main loop rate (typically 400Hz)
     * @warning Requires setup_fins() to have been called during initialization
     * 
     * @see setup_fins()
     * @see add_fin()
     */
    void output();

    /**
     * @brief Get approximate throttle/effort level for telemetry reporting
     * 
     * @return float Throttle percentage as maximum of all control axis commands (0.0 to 1.0+)
     * 
     * @details Calculates a single "throttle" value for MAVLink telemetry by taking the
     *          maximum absolute value across all four control axes (right, front, yaw, down).
     *          This provides ground control stations with an indicator of how hard the fins
     *          are working.
     *          
     *          Note: This is the unconstrained version - if higher level control provides
     *          inputs > 1.0, the reported throttle can exceed 100%. This helps identify
     *          control saturation conditions.
     * 
     * @note This is a telemetry aid only and doesn't represent actual power consumption
     * @see right_out, front_out, yaw_out, down_out
     */
    float get_throttle()
    {
        //Only for Mavlink - essentially just an indicator of how hard the fins are working.
        //Note that this is the unconstrained version, so if the higher level control gives too high input,
        //throttle will be displayed as more than 100.
        return fmaxf(fmaxf(fabsf(down_out),fabsf(front_out)), fmaxf(fabsf(right_out),fabsf(yaw_out)));
    }

    /**
     * @brief Write PWM value to a servo channel for fin control
     * 
     * @param[in] chan Servo channel number to write to
     * @param[in] pwm  PWM value in microseconds (typically 1000-2000 µs)
     * 
     * @details Low-level function to send PWM commands to physical servo outputs.
     *          Called by output() to actuate the fin servos with calculated positions.
     * 
     * @note PWM values are typically in range 1000-2000 µs for standard servos
     * @see output()
     */
    void rc_write(uint8_t chan, uint16_t pwm);
};
