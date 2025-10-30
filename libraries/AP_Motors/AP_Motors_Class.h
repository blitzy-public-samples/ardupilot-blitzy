/**
 * @file AP_Motors_Class.h
 * @brief Abstract base class for all ArduPilot motor control implementations
 * 
 * @details This file defines the AP_Motors base class which provides the common interface
 *          and functionality for all motor control implementations in ArduPilot. The class
 *          hierarchy supports multiple vehicle types including multicopters, helicopters,
 *          and 6DOF vehicles.
 *          
 *          Key responsibilities:
 *          - Define abstract interface that all motor implementations must provide
 *          - Manage motor arming, interlock, and spoolup state machines
 *          - Process roll, pitch, yaw, and throttle control inputs
 *          - Provide common functionality for motor testing and limit flagging
 *          - Support multiple frame classes and types
 *          
 *          Inheritance hierarchy:
 *          AP_Motors (abstract base)
 *            ├─ AP_MotorsMulticopter (multirotor base)
 *            │   ├─ AP_MotorsMatrix (quad, hexa, octa, Y6, etc.)
 *            │   └─ AP_MotorsTri (tricopter)
 *            ├─ AP_MotorsHeli (traditional helicopter base)
 *            │   ├─ AP_MotorsHeli_Single
 *            │   ├─ AP_MotorsHeli_Dual
 *            │   └─ AP_MotorsHeli_Quad
 *            └─ AP_Motors6DOF (underwater vehicles)
 *          
 * @note This is an abstract base class - instantiate a derived class for actual use
 * @note Singleton pattern - only one global AP_Motors instance should exist
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 * 
 * Source: libraries/AP_Motors/AP_Motors_Class.h
 */

#pragma once

#include "AP_Motors_config.h"

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <Filter/Filter.h>         // filter library
#include <Filter/DerivativeFilter.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Logger/AP_Logger_config.h>
#include <SRV_Channel/SRV_Channel_config.h>

/**
 * @name Motor Index Definitions
 * @brief Motor array indices for up to 32 motors
 * 
 * @details These defines map motor numbers 1-32 (used in parameters and user interface)
 *          to internal array indices 0-31. Motor numbering and layout varies by frame type.
 *          
 * @note Motor numbers are 1-based for user interface but 0-based internally
 * @note Maximum 32 motors supported across all frame types
 * @{
 */
#define AP_MOTORS_MOT_1 0U   ///< Motor 1 array index (user-facing motor number 1)
#define AP_MOTORS_MOT_2 1U   ///< Motor 2 array index (user-facing motor number 2)
#define AP_MOTORS_MOT_3 2U   ///< Motor 3 array index (user-facing motor number 3)
#define AP_MOTORS_MOT_4 3U   ///< Motor 4 array index (user-facing motor number 4)
#define AP_MOTORS_MOT_5 4U   ///< Motor 5 array index (user-facing motor number 5)
#define AP_MOTORS_MOT_6 5U   ///< Motor 6 array index (user-facing motor number 6)
#define AP_MOTORS_MOT_7 6U   ///< Motor 7 array index (user-facing motor number 7)
#define AP_MOTORS_MOT_8 7U   ///< Motor 8 array index (user-facing motor number 8)
#define AP_MOTORS_MOT_9 8U   ///< Motor 9 array index (user-facing motor number 9)
#define AP_MOTORS_MOT_10 9U  ///< Motor 10 array index (user-facing motor number 10)
#define AP_MOTORS_MOT_11 10U ///< Motor 11 array index (user-facing motor number 11)
#define AP_MOTORS_MOT_12 11U ///< Motor 12 array index (user-facing motor number 12)
#define AP_MOTORS_MOT_13 12U ///< Motor 13 array index (user-facing motor number 13)
#define AP_MOTORS_MOT_14 13U ///< Motor 14 array index (user-facing motor number 14)
#define AP_MOTORS_MOT_15 14U ///< Motor 15 array index (user-facing motor number 15)
#define AP_MOTORS_MOT_16 15U ///< Motor 16 array index (user-facing motor number 16)
#define AP_MOTORS_MOT_17 16U ///< Motor 17 array index (user-facing motor number 17)
#define AP_MOTORS_MOT_18 17U ///< Motor 18 array index (user-facing motor number 18)
#define AP_MOTORS_MOT_19 18U ///< Motor 19 array index (user-facing motor number 19)
#define AP_MOTORS_MOT_20 19U ///< Motor 20 array index (user-facing motor number 20)
#define AP_MOTORS_MOT_21 20U ///< Motor 21 array index (user-facing motor number 21)
#define AP_MOTORS_MOT_22 21U ///< Motor 22 array index (user-facing motor number 22)
#define AP_MOTORS_MOT_23 22U ///< Motor 23 array index (user-facing motor number 23)
#define AP_MOTORS_MOT_24 23U ///< Motor 24 array index (user-facing motor number 24)
#define AP_MOTORS_MOT_25 24U ///< Motor 25 array index (user-facing motor number 25)
#define AP_MOTORS_MOT_26 25U ///< Motor 26 array index (user-facing motor number 26)
#define AP_MOTORS_MOT_27 26U ///< Motor 27 array index (user-facing motor number 27)
#define AP_MOTORS_MOT_28 27U ///< Motor 28 array index (user-facing motor number 28)
#define AP_MOTORS_MOT_29 28U ///< Motor 29 array index (user-facing motor number 29)
#define AP_MOTORS_MOT_30 29U ///< Motor 30 array index (user-facing motor number 30)
#define AP_MOTORS_MOT_31 30U ///< Motor 31 array index (user-facing motor number 31)
#define AP_MOTORS_MOT_32 31U ///< Motor 32 array index (user-facing motor number 32)
/** @} */ // end of Motor Index Definitions group

/**
 * @brief Default motor output update rate
 * 
 * @details Default frequency for sending motor commands to ESCs. This is appropriate for
 *          most digital ESC protocols (DShot, OneShot125). Traditional PWM ESCs typically
 *          use 50Hz (configured via MOT_PWM_FREQ parameter).
 *          
 * @note Actual update rate may be overridden by derived classes or parameters
 * @note Higher rates improve control responsiveness but may not be supported by all ESCs
 */
#define AP_MOTORS_SPEED_DEFAULT     490 // default output rate to the motors in Hz

/**
 * @class AP_Motors
 * @brief Abstract base class for all ArduPilot motor control implementations
 * 
 * @details AP_Motors defines the common interface and functionality for controlling motors
 *          across all ArduPilot vehicle types. This abstract base class provides:
 *          
 *          - Unified control input interface (roll, pitch, yaw, throttle, forward, lateral)
 *          - Motor arming and safety interlock management
 *          - Spoolup state machine for smooth motor startup
 *          - Control limit flagging for attitude controller feedback
 *          - Motor testing capabilities for configuration and diagnostics
 *          - Support for multiple frame configurations and motor layouts
 *          
 *          Pure virtual methods that derived classes must implement:
 *          - init() - Initialize motor configuration for specific frame type
 *          - output() - Compute motor mixing and send output to ESCs
 *          - output_min() - Send minimum safe output values
 *          - get_motor_mask() - Return bitmask of active motor outputs
 *          - update_throttle_filter() - Update throttle input filtering
 *          - output_armed_stabilizing() - Protected output for armed state
 *          
 *          Common usage pattern:
 *          1. Construct appropriate derived class (e.g., AP_MotorsMatrix for multicopter)
 *          2. Call init() with desired frame_class and frame_type
 *          3. In main control loop:
 *             - Call set_roll(), set_pitch(), set_yaw(), set_throttle() with control inputs
 *             - Call set_desired_spool_state() to manage motor spoolup
 *             - Call output() to compute mixing and command motors
 *          4. Check limit flags to prevent integrator windup in attitude controllers
 *          
 * @note This is a singleton class - use get_singleton() or AP::motors() to access instance
 * @note Motor output must be called every control loop (typically 400Hz) for stable operation
 * @note Control inputs are in body frame: roll right +, pitch forward +, yaw clockwise +
 * 
 * @warning Arming checks must pass before motors will spin
 * @warning Motor interlock provides emergency stop - disables motors immediately
 * @warning Frame class and type must match physical vehicle configuration
 * 
 * @see AP_MotorsMulticopter for multirotor implementations
 * @see AP_MotorsHeli for helicopter implementations  
 * @see AP_Motors6DOF for underwater vehicle implementations
 * 
 * Source: libraries/AP_Motors/AP_Motors_Class.h:51-402
 */
class AP_Motors {
public:

    /**
     * @enum motor_frame_class
     * @brief Frame class defines the fundamental vehicle configuration and number of motors
     * 
     * @details Frame class determines the motor mixing algorithm, number of motors, and basic
     *          vehicle architecture. Each class has specific motor arrangements and control
     *          characteristics. Frame class selection must match the physical vehicle build.
     *          
     *          Multicopter frame classes use differential motor thrust for attitude control.
     *          Helicopter frame classes use collective pitch and cyclic swashplate control.
     *          Special classes support scripting-based custom configurations.
     *          
     * @note Frame class is typically set via FRAME_CLASS parameter
     * @note Frame class cannot be changed during flight - requires reboot
     * 
     * @warning Incorrect frame class will result in unstable or dangerous flight behavior
     */
    enum motor_frame_class {
        MOTOR_FRAME_UNDEFINED = 0,           ///< Undefined or not yet configured
        MOTOR_FRAME_QUAD = 1,                ///< Quadcopter (4 motors)
        MOTOR_FRAME_HEXA = 2,                ///< Hexacopter (6 motors)
        MOTOR_FRAME_OCTA = 3,                ///< Octacopter (8 motors)
        MOTOR_FRAME_OCTAQUAD = 4,            ///< Octa-Quad (8 motors, X8 configuration)
        MOTOR_FRAME_Y6 = 5,                  ///< Y6 configuration (6 motors on 3 arms)
        MOTOR_FRAME_HELI = 6,                ///< Traditional helicopter (single main rotor + tail rotor)
        MOTOR_FRAME_TRI = 7,                 ///< Tricopter (3 motors + tail servo)
        MOTOR_FRAME_SINGLE = 8,              ///< Single motor + control surfaces
        MOTOR_FRAME_COAX = 9,                ///< Coaxial helicopter (2 counter-rotating main rotors)
        MOTOR_FRAME_TAILSITTER = 10,         ///< VTOL tailsitter configuration
        MOTOR_FRAME_HELI_DUAL = 11,          ///< Dual rotor helicopter (tandem or transverse)
        MOTOR_FRAME_DODECAHEXA = 12,         ///< Dodeca-Hexa (12 motors)
        MOTOR_FRAME_HELI_QUAD = 13,          ///< Quad rotor helicopter
        MOTOR_FRAME_DECA = 14,               ///< Decacopter (10 motors)
        MOTOR_FRAME_SCRIPTING_MATRIX = 15,   ///< Custom matrix defined via scripting
        MOTOR_FRAME_6DOF_SCRIPTING = 16,     ///< 6DOF vehicle with scripting control (for ROVs)
        MOTOR_FRAME_DYNAMIC_SCRIPTING_MATRIX = 17, ///< Runtime-configurable motor matrix via scripting
    };

    /**
     * @brief Get human-readable string for current frame class
     * 
     * @return Pointer to constant string (e.g., "QUAD", "HEXA", "HELI")
     * 
     * @note Used for logging and ground station display
     */
    const char* get_frame_string() const;

    /**
     * @enum motor_frame_type
     * @brief Frame type defines motor layout and ordering within a frame class
     * 
     * @details Frame type specifies the specific motor arrangement and ordering convention
     *          within a given frame class. For example, quadcopters (MOTOR_FRAME_QUAD) can
     *          be configured as PLUS (+), X, H, or V configurations with different motor
     *          positions and numbering schemes.
     *          
     *          Motor ordering affects:
     *          - Which physical motor responds to each output channel
     *          - Motor rotation directions (CW vs CCW)
     *          - Coordinate frame alignment relative to vehicle body
     *          
     *          Common frame types:
     *          - PLUS: Motors aligned with body axes (0°, 90°, 180°, 270°)
     *          - X: Motors at 45° angles (45°, 135°, 225°, 315°) - most common
     *          - Different ordering conventions: ArduPilot standard, Betaflight, DJI
     *          
     * @note Frame type is typically set via FRAME_TYPE parameter
     * @note Some frame types are only valid for specific frame classes
     * @note Motor ordering conventions vary between flight controller firmware
     * 
     * @warning Incorrect frame type causes incorrect motor response to control inputs
     */
    enum motor_frame_type {
        MOTOR_FRAME_TYPE_PLUS = 0,        ///< + configuration (arms at 0°/90°/180°/270°)
        MOTOR_FRAME_TYPE_X = 1,           ///< X configuration (arms at 45°/135°/225°/315°) - most common
        MOTOR_FRAME_TYPE_V = 2,           ///< V-tail configuration
        MOTOR_FRAME_TYPE_H = 3,           ///< H-frame configuration (wide stance)
        MOTOR_FRAME_TYPE_VTAIL = 4,       ///< V-tail specific layout
        MOTOR_FRAME_TYPE_ATAIL = 5,       ///< A-tail specific layout
        MOTOR_FRAME_TYPE_PLUSREV = 6,     ///< Plus configuration with reversed motor directions
        MOTOR_FRAME_TYPE_Y6B = 10,        ///< Y6 with bottom motors spinning clockwise
        MOTOR_FRAME_TYPE_Y6F = 11,        ///< Y6 FireFly configuration (top motors CW)
        MOTOR_FRAME_TYPE_BF_X = 12,       ///< X frame with Betaflight motor ordering (1:rear-right)
        MOTOR_FRAME_TYPE_DJI_X = 13,      ///< X frame with DJI motor ordering (1:front-right)
        MOTOR_FRAME_TYPE_CW_X = 14,       ///< X frame with clockwise-ordered motor numbering
        MOTOR_FRAME_TYPE_I = 15,          ///< I-frame (sideways H, octo only)
        MOTOR_FRAME_TYPE_NYT_PLUS = 16,   ///< Plus frame, no yaw torque (all motors same rotation)
        MOTOR_FRAME_TYPE_NYT_X = 17,      ///< X frame, no yaw torque (all motors same rotation)
        MOTOR_FRAME_TYPE_BF_X_REV = 18,   ///< X frame, Betaflight ordering, reversed motor directions
        MOTOR_FRAME_TYPE_Y4 = 19,         ///< Y4 configuration (4 motors, Y layout)
    };


    /**
     * @brief Format frame class and type into a string (e.g., "QUAD/X")
     * 
     * @param[out] buffer Output buffer to receive formatted string
     * @param[in]  buflen Maximum length of output buffer including null terminator
     * 
     * @note Used for logging and ground station display
     */
    void get_frame_and_type_string(char *buffer, uint8_t buflen) const;

    /**
     * @brief Constructor for AP_Motors base class
     * 
     * @param[in] speed_hz Motor output update rate in Hz (default 490Hz)
     * 
     * @details Initializes the motor control base class with specified update rate.
     *          Sets initial state to disarmed with motors shut down.
     *          
     * @note Actual motor configuration happens in derived class init() method
     */
    AP_Motors(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT);

    /**
     * @brief Get pointer to the global AP_Motors singleton instance
     * 
     * @return Pointer to singleton instance, or nullptr if not yet constructed
     * 
     * @note Alternative access via AP::motors() in AP namespace
     * @note Only one AP_Motors instance should exist per vehicle
     */
    static AP_Motors    *get_singleton(void) { return _singleton; }

    /**
     * @brief Check if vehicle passes all pre-arming safety checks
     * 
     * @param[in]  buflen Maximum length of error message buffer
     * @param[out] buffer Buffer to receive human-readable failure reason
     * 
     * @return true if all arming checks pass, false if any check fails
     * 
     * @details Derived classes override to add frame-specific checks.
     *          Common checks include motor configuration validation.
     *          
     * @note Called by arming logic before allowing motor arming
     * @warning Motors will not arm if this returns false
     */
    virtual bool        arming_checks(size_t buflen, char *buffer) const;
    
    /**
     * @brief Check if vehicle is safe for motor testing
     * 
     * @param[in]  buflen Maximum length of error message buffer
     * @param[out] buffer Buffer to receive human-readable failure reason
     * 
     * @return true if motor testing is safe, false otherwise
     * 
     * @warning Motor testing WILL SPIN MOTORS - remove propellers first
     * @note Typically requires vehicle to be disarmed
     */
    virtual bool        motor_test_checks(size_t buflen, char *buffer) const;
    
    /**
     * @brief Check if motor initialization completed successfully
     * 
     * @return true if initialization succeeded, false otherwise
     * 
     * @note Should return true after successful call to init()
     */
    bool                initialised_ok() const { return _initialised_ok; }
    
    /**
     * @brief Set motor initialization status
     * 
     * @param[in] val true if initialization successful, false otherwise
     * 
     * @note Typically called by derived classes after completing initialization
     */
    void                set_initialised_ok(bool val) { _initialised_ok = val; }

    /**
     * @brief Query current motor armed state
     * 
     * @return true if motors are armed and can spin, false if disarmed
     * 
     * @note Armed state allows motors to spin when other conditions are met
     * @note Motors must be armed AND interlock enabled to actually spin
     */
    bool                armed() const { return _armed; }
    
    /**
     * @brief Set motor armed/disarmed state
     * 
     * @param[in] arm true to arm motors, false to disarm
     * 
     * @details Arming enables motors to spin (when interlock also enabled).
     *          Disarming immediately stops motors and saves parameters.
     *          
     * @warning Arming motors can cause immediate spinning if throttle is raised
     * @note Arming checks must pass before arming is allowed
     * @note Disarming triggers parameter save in some vehicle types
     */
    void                armed(bool arm);

    /**
     * @brief Set motor interlock state (emergency motor stop)
     * 
     * @param[in] set true to enable interlock (motors can run), false to disable (emergency stop)
     * 
     * @details Motor interlock provides emergency motor cutoff independent of arming.
     *          Disabling interlock immediately stops all motors regardless of other state.
     *          Both armed=true AND interlock=true are required for motors to spin.
     *          
     *          Interlock is typically controlled by:
     *          - Auxiliary function switch (RCx_OPTION = 32)
     *          - Automatic activation during spoolup
     *          - Emergency stop conditions
     *          
     * @warning Disabling interlock causes immediate motor stop - may cause crash if in flight
     * @note Interlock is separate from arming - both must be true for motor operation
     */
    void                set_interlock(bool set) { _interlock = set;}

    /**
     * @brief Get current motor interlock status
     * 
     * @return true if interlock enabled (motors can run), false if disabled (emergency stop active)
     * 
     * @note True means motors are allowed to run (if also armed)
     * @note False means motors are forcibly stopped regardless of arming
     */
    bool                get_interlock() const { return _interlock; }

    /**
     * @brief Get spoolup block state
     * 
     * @return true if spoolup is blocked, false if spoolup allowed
     * 
     * @details Spoolup block prevents motors from leaving ground idle during landing
     *          or other situations where controlled spooldown is required.
     */
    bool                get_spoolup_block() const { return _spoolup_block; }
    
    /**
     * @brief Set spoolup block state
     * 
     * @param[in] set true to block spoolup, false to allow spoolup
     * 
     * @note Used by landing controllers to prevent motors spooling up during touchdown
     */
    void                set_spoolup_block(bool set) { _spoolup_block = set; }

    /**
     * @brief Set desired roll control input from attitude controller
     * 
     * @param[in] roll_in Desired roll control, range -1.0 to +1.0 (right positive)
     * 
     * @details Positive values command roll right (right wing down for multirotors).
     *          This is the primary roll control from the attitude controller PID output.
     *          Value is mixed with pitch/yaw/throttle to compute individual motor outputs.
     *          
     * @note Called every control loop (typically 400Hz) with attitude controller output
     * @note Roll control is in body frame (right = positive)
     */
    void                set_roll(float roll_in) { _roll_in = roll_in; };        // range -1 ~ +1
    
    /**
     * @brief Set roll feedforward control input
     * 
     * @param[in] roll_in Desired roll feedforward, range -1.0 to +1.0
     * 
     * @details Feedforward provides direct control authority bypassing attitude PID.
     *          Used for rate feedforward from attitude controller to improve response.
     */
    void                set_roll_ff(float roll_in) { _roll_in_ff = roll_in; };    // range -1 ~ +1
    
    /**
     * @brief Set desired pitch control input from attitude controller
     * 
     * @param[in] pitch_in Desired pitch control, range -1.0 to +1.0 (forward positive)
     * 
     * @details Positive values command pitch forward (nose down for multirotors).
     *          This is the primary pitch control from the attitude controller PID output.
     *          Value is mixed with roll/yaw/throttle to compute individual motor outputs.
     *          
     * @note Called every control loop (typically 400Hz) with attitude controller output
     * @note Pitch control is in body frame (forward = positive)
     */
    void                set_pitch(float pitch_in) { _pitch_in = pitch_in; };    // range -1 ~ +1
    
    /**
     * @brief Set pitch feedforward control input
     * 
     * @param[in] pitch_in Desired pitch feedforward, range -1.0 to +1.0
     * 
     * @details Feedforward provides direct control authority bypassing attitude PID.
     *          Used for rate feedforward from attitude controller to improve response.
     */
    void                set_pitch_ff(float pitch_in) { _pitch_in_ff = pitch_in; };  // range -1 ~ +1
    
    /**
     * @brief Set desired yaw control input from attitude controller
     * 
     * @param[in] yaw_in Desired yaw control, range -1.0 to +1.0 (clockwise positive)
     * 
     * @details Positive values command yaw clockwise (viewed from above).
     *          This is the primary yaw control from the attitude controller PID output.
     *          For multirotors, yaw is controlled by differential motor thrust.
     *          
     * @note Called every control loop (typically 400Hz) with attitude controller output
     * @note Yaw control is in body frame (clockwise = positive when viewed from above)
     */
    void                set_yaw(float yaw_in) { _yaw_in = yaw_in; };            // range -1 ~ +1
    
    /**
     * @brief Set yaw feedforward control input
     * 
     * @param[in] yaw_in Desired yaw feedforward, range -1.0 to +1.0
     * 
     * @details Feedforward provides direct control authority bypassing attitude PID.
     *          Used for rate feedforward from attitude controller to improve response.
     */
    void                set_yaw_ff(float yaw_in) { _yaw_in_ff = yaw_in; };      // range -1 ~ +1
    
    /**
     * @brief Set desired throttle input from pilot or position controller
     * 
     * @param[in] throttle_in Desired throttle, range 0.0 to 1.0
     * 
     * @details Throttle controls overall thrust magnitude. 0.0 = minimum thrust,
     *          1.0 = maximum thrust. Input is filtered before use. Throttle is mixed
     *          with roll/pitch/yaw to compute individual motor commands.
     *          
     * @note Called every control loop with pilot stick input or position controller output
     * @note Throttle is always positive (0-1), unlike roll/pitch/yaw
     * @note Actual motor output depends on spool state and arming
     */
    void                set_throttle(float throttle_in) { _throttle_in = throttle_in; };   // range 0 ~ 1
    
    /**
     * @brief Set maximum average throttle for thrust limiting
     * 
     * @param[in] throttle_avg_max Maximum average throttle, range 0.0 to 1.0
     * 
     * @details Used to limit average motor output, typically for battery protection
     *          or thermal management. Value is constrained to valid range.
     */
    void                set_throttle_avg_max(float throttle_avg_max) { _throttle_avg_max = constrain_float(throttle_avg_max, 0.0f, 1.0f); };   // range 0 ~ 1
    
    /**
     * @brief Set throttle input filter cutoff frequency
     * 
     * @param[in] filt_hz Low-pass filter cutoff frequency in Hz
     * 
     * @details Filters rapid throttle changes to smooth motor response.
     *          Lower frequencies provide more smoothing but slower response.
     */
    void                set_throttle_filter_cutoff(float filt_hz) { _throttle_filter.set_cutoff_frequency(filt_hz); }
    
    /**
     * @brief Set throttle slew rate filter cutoff frequency
     * 
     * @param[in] filt_hz Low-pass filter cutoff frequency in Hz
     * 
     * @details Filters the detected throttle slew rate for smooth limiting.
     */
    void                set_slew_filter_cutoff(float filt_hz) { _throttle_slew_filter.set_cutoff_frequency(filt_hz); }
    
    /**
     * @brief Set desired forward thrust input (for vehicles with forward motion)
     * 
     * @param[in] forward_in Desired forward thrust, range -1.0 to +1.0
     * 
     * @details Used by submarines and vectored thrust vehicles for forward/backward motion.
     *          Positive = forward, negative = backward. Not used by standard multirotors.
     *          
     * @note Only relevant for 6DOF vehicles (submarines) and some tailsitters
     */
    void                set_forward(float forward_in) { _forward_in = forward_in; }; // range -1 ~ +1
    
    /**
     * @brief Set desired lateral thrust input (for vehicles with lateral motion)
     * 
     * @param[in] lateral_in Desired lateral thrust, range -1.0 to +1.0
     * 
     * @details Used by submarines and vectored thrust vehicles for left/right motion.
     *          Positive = right, negative = left. Not used by standard multirotors.
     *          
     * @note Only relevant for 6DOF vehicles (submarines)
     */
    void                set_lateral(float lateral_in) { _lateral_in = lateral_in; };     // range -1 ~ +1

    /**
     * @brief Set thrust vector tilt angles for 6DoF vehicles
     * 
     * @param[in] roll_deg  Roll tilt angle in degrees
     * @param[in] pitch_deg Pitch tilt angle in degrees
     * 
     * @details For vehicles with vectored thrust (tiltrotors, submarines),
     *          this rotates the thrust vector in body frame. Not used by standard multirotors.
     *          
     * @note Virtual function - only implemented by 6DOF motor classes
     * @note Default implementation does nothing (standard multirotors)
     */
    virtual void        set_roll_pitch(float roll_deg, float pitch_deg) {};

    /**
     * @brief Get current roll control input
     * @return Roll control value, range -1.0 to +1.0 (right positive)
     */
    float               get_roll() const { return _roll_in; }
    
    /**
     * @brief Get current roll feedforward input
     * @return Roll feedforward value, range -1.0 to +1.0
     */
    float               get_roll_ff() const { return _roll_in_ff; }
    
    /**
     * @brief Get current pitch control input
     * @return Pitch control value, range -1.0 to +1.0 (forward positive)
     */
    float               get_pitch() const { return _pitch_in; }
    
    /**
     * @brief Get current pitch feedforward input
     * @return Pitch feedforward value, range -1.0 to +1.0
     */
    float               get_pitch_ff() const { return _pitch_in_ff; }
    
    /**
     * @brief Get current yaw control input
     * @return Yaw control value, range -1.0 to +1.0 (clockwise positive)
     */
    float               get_yaw() const { return _yaw_in; }
    
    /**
     * @brief Get current yaw feedforward input
     * @return Yaw feedforward value, range -1.0 to +1.0
     */
    float               get_yaw_ff() const { return _yaw_in_ff; }
    
    /**
     * @brief Get actual throttle output after mixing
     * @return Throttle output value, range 0.0 to 1.0
     * @note This is the final throttle value sent to motors after all mixing
     */
    float               get_throttle_out() const { return _throttle_out; }
    
    /**
     * @brief Get thrust output for a specific motor
     * 
     * @param[in]  motor_num Motor index (0-based)
     * @param[out] thr_out   Thrust output value (0.0 to 1.0)
     * 
     * @return true if motor_num is valid and thr_out was set, false otherwise
     * 
     * @note Virtual function - must be implemented by derived classes
     * @note Default implementation returns false (not supported)
     */
    virtual bool        get_thrust(uint8_t motor_num, float& thr_out) const { return false; }
    
    /**
     * @brief Get raw throttle output for a specific motor before thrust curve
     * 
     * @param[in]  motor_num Motor index (0-based)
     * @param[out] thr_out   Raw throttle output value (0.0 to 1.0)
     * 
     * @return true if motor_num is valid and thr_out was set, false otherwise
     * 
     * @note Virtual function - must be implemented by derived classes
     * @note Default implementation returns false (not supported)
     */
    virtual bool        get_raw_motor_throttle(uint8_t motor_num, float& thr_out) const { return false; }
    
    /**
     * @brief Get filtered throttle input
     * @return Filtered throttle value, range 0.0 to 1.0
     * @note This is the throttle input after low-pass filtering
     */
    float               get_throttle() const { return constrain_float(_throttle_filter.get(), 0.0f, 1.0f); }
    
    /**
     * @brief Get bidirectional throttle input (for reversible motors)
     * @return Throttle value remapped to -1.0 to +1.0 range
     * @note Converts 0.0-1.0 throttle to -1.0 to +1.0 (0.5 = 0.0 output)
     */
    float               get_throttle_bidirectional() const { return constrain_float(2 * (_throttle_filter.get() - 0.5f), -1.0f, 1.0f); }
    
    /**
     * @brief Get current throttle slew rate
     * @return Throttle slew rate in throttle units per second
     * @note Used to detect rapid throttle changes
     */
    float               get_throttle_slew_rate() const { return _throttle_slew_rate; }
    
    /**
     * @brief Get current forward thrust input
     * @return Forward thrust value, range -1.0 to +1.0
     * @note Only used by 6DOF vehicles
     */
    float               get_forward() const { return _forward_in; }
    
    /**
     * @brief Get current lateral thrust input
     * @return Lateral thrust value, range -1.0 to +1.0
     * @note Only used by 6DOF vehicles
     */
    float               get_lateral() const { return _lateral_in; }
    
    /**
     * @brief Get learned hover throttle value
     * 
     * @return Throttle value that maintains hover (0.0 to 1.0)
     * 
     * @details This is the throttle value that results in zero vertical acceleration.
     *          Value is learned over time during hover flight and used by altitude
     *          controllers for feedforward. Pure virtual - must be implemented by derived classes.
     *          
     * @note Value is vehicle-specific and adapts to payload/battery changes
     * @note Used by position controllers for hover thrust feedforward
     */
    virtual float       get_throttle_hover() const = 0;

    /**
     * @brief Enable/disable thrust boost for motor failure compensation
     * 
     * @param[in] enable true to enable thrust boost, false to disable
     * 
     * @details Thrust boost increases remaining motor outputs to compensate for a failed motor.
     *          When enabled, motors can spin faster to maintain attitude control.
     *          
     * @note Used when motor failure is detected to maintain limited control authority
     */
    void                set_thrust_boost(bool enable) { _thrust_boost = enable; }
    
    /**
     * @brief Query if thrust boost is currently enabled
     * @return true if thrust boost enabled, false otherwise
     */
    bool                get_thrust_boost() const { return _thrust_boost; }
    
    /**
     * @brief Get the index of a failed/lost motor
     * 
     * @return Motor index (0-based) of failed motor, or 0 if no failure detected
     * 
     * @note Virtual function - implemented by derived classes with failure detection
     * @note Default implementation returns 0 (no failure detection)
     */
    virtual uint8_t     get_lost_motor() const { return 0; }

    /**
     * @enum DesiredSpoolState
     * @brief Requested motor spool state from flight mode or pilot
     * 
     * @details Desired spool state is the target state requested by the flight mode
     *          or pilot input. Actual spool state (SpoolState) may lag behind desired
     *          state during controlled spoolup/spooldown transitions.
     */
    enum class DesiredSpoolState : uint8_t {
        SHUT_DOWN = 0,              ///< Request motors stop completely
        GROUND_IDLE = 1,            ///< Request motors spin at ground idle (armed but not flying)
        THROTTLE_UNLIMITED = 2,     ///< Request full throttle authority (flying)
    };

    /**
     * @brief Set desired motor spool state
     * 
     * @param[in] spool Desired spool state (SHUT_DOWN, GROUND_IDLE, THROTTLE_UNLIMITED)
     * 
     * @details Requests transition to specified spool state. Actual transition may be
     *          gradual to provide smooth throttle ramp. Flight modes set this based on
     *          flight phase (landed, taking off, flying, landing).
     *          
     * @note Actual spool state may lag desired state during spoolup/spooldown
     * @note SHUT_DOWN: Motors stop (disarmed or landing complete)
     * @note GROUND_IDLE: Motors at minimum safe RPM (armed on ground)
     * @note THROTTLE_UNLIMITED: Full throttle control (airborne)
     */
    void set_desired_spool_state(enum DesiredSpoolState spool);

    /**
     * @brief Get currently requested spool state
     * @return Current desired spool state
     */
    enum DesiredSpoolState get_desired_spool_state(void) const { return _spool_desired; }

    /**
     * @enum SpoolState
     * @brief Actual current motor spool state
     * 
     * @details Spool state tracks the actual motor state during startup and shutdown.
     *          Provides smooth transitions between states to prevent abrupt throttle changes
     *          that could destabilize the vehicle.
     *          
     *          State transitions:
     *          SHUT_DOWN → GROUND_IDLE → SPOOLING_UP → THROTTLE_UNLIMITED
     *          THROTTLE_UNLIMITED → SPOOLING_DOWN → GROUND_IDLE → SHUT_DOWN
     *          
     * @note Spool state machine prevents sudden throttle jumps
     * @note Spoolup typically takes 0.5-1.0 seconds for safety
     */
    enum class SpoolState : uint8_t {
        SHUT_DOWN = 0,                      ///< Motors stopped completely
        GROUND_IDLE = 1,                    ///< Motors at minimum safe RPM (armed on ground)
        SPOOLING_UP = 2,                    ///< Transitioning from ground idle to full authority
        THROTTLE_UNLIMITED = 3,             ///< Full throttle control available (flying)
        SPOOLING_DOWN = 4,                  ///< Transitioning from full authority to ground idle
    };

    /**
     * @brief Get current actual spool state
     * 
     * @return Current spool state
     * 
     * @details Returns actual motor spool state which may lag behind desired state
     *          during spoolup/spooldown transitions. Controllers should check spool state
     *          to determine if full throttle authority is available.
     *          
     * @note THROTTLE_UNLIMITED means full control authority is available
     * @note SPOOLING_UP/DOWN means throttle is ramping (limited authority)
     */
    enum SpoolState  get_spool_state(void) const { return _spool_state; }

    /**
     * @brief Set time delta for motor mixing calculations
     * 
     * @param[in] dt_s Time since last update in seconds
     * 
     * @details Sets the time delta used for rate limiting and filtering in motor mixing.
     *          Should be set to the time of the IMU sample used by attitude controllers
     *          to maintain consistent timing. Typically called every control loop.
     *          
     * @note Should match IMU update timing for consistent control
     * @note Typical values: 0.0025s (400Hz) or 0.002s (500Hz)
     */
    void set_dt_s(float dt_s) { _dt_s = dt_s; }
    
    /**
     * @brief Get current time delta
     * @return Time since last update in seconds
     */
    float get_dt_s() const { return _dt_s; }

    /**
     * @struct AP_Motors_limit
     * @brief Motor output saturation/limit flags for attitude controller feedback
     * 
     * @details This structure contains flags indicating when motor outputs have reached
     *          their limits and cannot provide additional control authority. Attitude
     *          controllers monitor these flags to prevent integrator windup and adjust
     *          control strategies when limits are reached.
     *          
     *          When a limit is reached:
     *          - The requested control output cannot be fully achieved
     *          - Attitude controllers should stop integrator accumulation in that axis
     *          - Flight modes may adjust behavior (e.g., reduce climb rate if throttle_upper)
     *          
     *          Common scenarios:
     *          - throttle_upper: Trying to climb too fast (max thrust reached)
     *          - throttle_lower: Trying to descend too fast (min thrust reached)
     *          - roll/pitch/yaw: Requesting rotation faster than motors can provide
     *          
     * @note Limit flags are updated every output() call
     * @note Controllers must check these flags to prevent integrator windup
     * @note Multiple limits can be active simultaneously
     * 
     * Source: libraries/AP_Motors/AP_Motors_Class.h:232-239
     */
    struct AP_Motors_limit {
        bool roll;           ///< Roll control saturated - cannot achieve desired roll rate
        bool pitch;          ///< Pitch control saturated - cannot achieve desired pitch rate
        bool yaw;            ///< Yaw control saturated - cannot achieve desired yaw rate
        bool throttle_lower; ///< At minimum throttle limit (cannot reduce thrust further)
        bool throttle_upper; ///< At maximum throttle limit (cannot increase thrust further)
    } limit;

    /**
     * @brief Set all attitude limit flags (roll, pitch, yaw) to same value
     * 
     * @param[in] flag Value to set for roll, pitch, and yaw limit flags
     * 
     * @details Convenience function to set all attitude axis limits at once.
     *          Typically used to clear all limits (flag=false) at start of mixing.
     */
    void set_limit_flag_pitch_roll_yaw(bool flag);

#if AP_SCRIPTING_ENABLED
    /**
     * @brief Set motor limit flags from Lua scripting
     * 
     * @param[in] roll            Roll limit flag
     * @param[in] pitch           Pitch limit flag
     * @param[in] yaw             Yaw limit flag
     * @param[in] throttle_lower  Lower throttle limit flag
     * @param[in] throttle_upper  Upper throttle limit flag
     * 
     * @details Allows Lua scripts to indicate motor limits for custom motor mixing.
     *          Used with scripting-based motor control (MOTOR_FRAME_SCRIPTING_MATRIX).
     *          
     * @note Only available when AP_SCRIPTING_ENABLED is defined
     */
    void set_external_limits(bool roll, bool pitch, bool yaw, bool throttle_lower, bool throttle_upper);
#endif

    //
    // virtual functions that should be implemented by child classes
    //

    /**
     * @brief Set motor output update rate
     * 
     * @param[in] speed_hz Output update frequency in Hz
     * 
     * @details Sets the rate at which motor commands are sent to ESCs. Higher rates
     *          improve control response but may not be supported by all ESCs.
     *          
     *          Common rates:
     *          - 50 Hz: Traditional PWM ESCs
     *          - 400 Hz: Fast PWM, OneShot125
     *          - 490 Hz: Default for most configurations
     *          - 1000+ Hz: DShot protocols
     *          
     * @note Virtual function - derived classes may override for protocol-specific handling
     * @note Actual achievable rate depends on ESC protocol and hardware
     */
    virtual void        set_update_rate( uint16_t speed_hz ) { _speed_hz = speed_hz; }

    /**
     * @brief Initialize motor configuration for specific frame
     * 
     * @param[in] frame_class Motor frame class (QUAD, HEXA, HELI, etc.)
     * @param[in] frame_type  Motor frame type (X, PLUS, etc.)
     * 
     * @details Pure virtual function that must be implemented by derived classes.
     *          Configures motor mixing, output channels, and frame-specific parameters
     *          based on frame class and type. Must be called before motors can be used.
     *          
     * @note Pure virtual - must be implemented by derived classes
     * @note Should set _initialised_ok = true on success
     * @warning Frame class/type must match physical vehicle or flight will be unstable
     */
    virtual void        init(motor_frame_class frame_class, motor_frame_type frame_type) = 0;

    /**
     * @brief Set or change frame class and type
     * 
     * @param[in] frame_class Motor frame class (QUAD, HEXA, HELI, etc.)
     * @param[in] frame_type  Motor frame type (X, PLUS, etc.)
     * 
     * @details Pure virtual function to change frame configuration. Reconfigures
     *          motor mixing for new frame type. Typically called during initialization
     *          or when parameters change.
     *          
     * @note Pure virtual - must be implemented by derived classes
     * @warning Should only be called when disarmed - not safe to change during flight
     */
    virtual void        set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) = 0;

    /**
     * @brief Compute motor mixing and send commands to motors
     * 
     * @details Pure virtual function that must be implemented by derived classes.
     *          This is the main motor output function called every control loop.
     *          
     *          Responsibilities:
     *          - Mix roll/pitch/yaw/throttle inputs into individual motor commands
     *          - Apply thrust curve and compensation factors
     *          - Enforce output limits and saturation
     *          - Update limit flags
     *          - Send PWM/DShot commands to ESCs
     *          
     * @note Pure virtual - must be implemented by derived classes
     * @note MUST be called every control loop (typically 400Hz) for stable operation
     * @warning Missing output() calls will cause erratic motor behavior and instability
     */
    virtual void        output() = 0;

    /**
     * @brief Send minimum safe output values to all motors
     * 
     * @details Pure virtual function that sends minimum safe PWM values to motors.
     *          Used during arming checks, disarming, and failsafes to ensure motors
     *          are in a known safe state.
     *          
     * @note Pure virtual - must be implemented by derived classes
     * @note Minimum output is typically zero throttle or motor stop command
     */
    virtual void        output_min() = 0;

    /**
     * @brief Test a specific motor at specified PWM value
     * 
     * @param[in] motor_seq Motor sequence number (1-based, 1 to num_motors)
     * @param[in] pwm       PWM output value in microseconds (typically 1000-2000)
     * 
     * @return true if motor test output successful, false if not possible
     * 
     * @details Spins a single motor at specified PWM value for testing and configuration.
     *          Used by motor test mode and ESC calibration procedures.
     *          
     * @warning WILL SPIN MOTORS - Remove propellers before testing!
     * @warning Vehicle should be disarmed and restrained before motor testing
     * @note Motor sequence numbers are 1-based (1 to N), not 0-based array indices
     * @note PWM values: 1000 µs = motor stop, 2000 µs = full throttle (typical)
     */
    bool                output_test_seq(uint8_t motor_seq, int16_t pwm);

    /**
     * @brief Get bitmask of active motor output channels
     * 
     * @return 32-bit bitmask where bit N set indicates motor/servo output N is used
     * 
     * @details Pure virtual function that returns which output channels are used for motors.
     *          Bit 0 = output 1, bit 1 = output 2, etc. Used to prevent conflicts between
     *          motor outputs and servo outputs (e.g., camera gimbal, landing gear).
     *          
     * @note Pure virtual - must be implemented by derived classes
     * @note Used by SRV_Channel to avoid assigning servos to motor outputs
     * @note Maximum 32 outputs supported (bits 0-31)
     */
    virtual uint32_t    get_motor_mask() = 0;

    /**
     * @brief Set direct pilot radio input passthrough (for setup/testing)
     * 
     * @param[in] roll_input     Pilot roll input, range -1.0 to +1.0
     * @param[in] pitch_input    Pilot pitch input, range -1.0 to +1.0
     * @param[in] throttle_input Pilot throttle input, range 0.0 to 1.0
     * @param[in] yaw_input      Pilot yaw input, range -1.0 to +1.0
     * 
     * @details Sets direct pilot input bypassing normal attitude control.
     *          Used for setup screens and providing servo feedback while landed.
     *          Allows ground station to display direct stick positions.
     *          
     * @note Only used for display/feedback - not for normal flight control
     * @note Typically only active when armed on ground or during setup
     */
    void                set_radio_passthrough(float roll_input, float pitch_input, float throttle_input, float yaw_input);

    /**
     * @brief Get roll control factor for a specific motor
     * 
     * @param[in] i Motor index (0-based)
     * 
     * @return Roll factor for motor i (-1.0 to +1.0), or 0.0 if not applicable
     * 
     * @details Returns how much a motor contributes to roll control. Used for
     *          tiltrotors and tailsitters that use copter motors for forward flight.
     *          Positive = contributes to right roll, negative = left roll.
     *          
     * @note Virtual function - overridden by frame types that need it
     * @note Default implementation returns 0.0 (not used)
     */
    virtual float       get_roll_factor(uint8_t i) { return 0.0f; }

    /**
     * @brief Get pitch control factor for a specific motor
     * 
     * @param[in] i Motor index (0-based)
     * 
     * @return Pitch factor for motor i (-1.0 to +1.0), or 0.0 if not applicable
     * 
     * @details Returns how much a motor contributes to pitch control.
     *          Positive = contributes to forward pitch, negative = aft pitch.
     *          
     * @note Virtual function - overridden by frame types that need it
     * @note Default implementation returns 0.0 (not used)
     */
    virtual float       get_pitch_factor(uint8_t i) { return 0.0f; }

    /**
     * @brief Check if a specific motor is enabled
     * 
     * @param[in] i Motor index (0-based)
     * 
     * @return true if motor i is enabled and will receive output, false otherwise
     * 
     * @note Virtual function - implemented by derived classes
     * @note Default implementation returns false
     */
    virtual bool        is_motor_enabled(uint8_t i) { return false; }

    /**
     * @brief Check if motor targets should be initialized when arming
     * 
     * @return true if targets should be reset on arming, false otherwise
     * 
     * @details Traditional helicopters need to initialize swashplate targets when
     *          transitioning from unarmed to armed state. Multicopters do not.
     *          
     * @note Virtual function - overridden by AP_MotorsHeli
     * @note Default implementation returns true (multicopter behavior)
     */
    virtual bool init_targets_on_arming() const { return true; }

    /**
     * @brief Check if configured PWM type is digital with fixed endpoints
     * 
     * @return true if PWM type is digital (DShot, OneShot), false for analog PWM
     * 
     * @details Digital protocols (DShot150/300/600/1200, OneShot) use fixed timing
     *          and don't require endpoint calibration. Analog PWM requires ESC calibration.
     */
    bool is_digital_pwm_type() const;

    /**
     * @brief Check if configured PWM type is for brushed motors
     * @return true if PWM type is BRUSHED, false otherwise
     * @note Brushed motors use continuous PWM voltage control, not pulses
     */
    bool is_brushed_pwm_type() const { return _pwm_type == PWMType::BRUSHED; }

    /**
     * @brief Check if configured PWM type is normal analog PWM
     * @return true if PWM type is NORMAL, PWM_RANGE, or PWM_ANGLE, false otherwise
     * @note Normal PWM types require ESC calibration and use standard pulse timing
     */
    bool is_normal_pwm_type() const { return (_pwm_type == PWMType::NORMAL) || (_pwm_type == PWMType::PWM_RANGE) || (_pwm_type == PWMType::PWM_ANGLE); }

    /**
     * @brief Get MAVLink vehicle type for this frame configuration
     * @return MAV_TYPE enum value (e.g., MAV_TYPE_QUADROTOR, MAV_TYPE_HELICOPTER)
     * @note Used for MAVLink HEARTBEAT message to identify vehicle type to ground station
     */
    MAV_TYPE get_frame_mav_type() const { return _mav_type; }

    /**
     * @brief Write PWM value directly to an output channel
     * 
     * @param[in] chan Output channel number (0-based)
     * @param[in] pwm  PWM value in microseconds (typically 1000-2000)
     * 
     * @details Directly writes PWM to specified output channel bypassing normal mixing.
     *          Used for special cases like servo outputs or direct motor control.
     *          
     * @note Virtual function - may be overridden for protocol-specific handling
     * @warning Bypasses normal safety checks and mixing - use with caution
     */
    virtual void        rc_write(uint8_t chan, uint16_t pwm);

#if AP_SCRIPTING_ENABLED
    /**
     * @brief Set custom frame description string for scripting
     * 
     * @param[in] str Custom frame string (e.g., "Custom Octo")
     * 
     * @details Allows Lua scripts to set custom frame description for display
     *          in ground station and logs. Used with scripting-based motor control.
     *          
     * @note Only available when AP_SCRIPTING_ENABLED is defined
     * @note String is stored in dynamically allocated memory
     */
    void set_frame_string(const char * str);
#endif

#if HAL_LOGGING_ENABLED
    /**
     * @brief Write motor status to dataflash log
     * 
     * @details Virtual function called at 10Hz to log motor-related data.
     *          Derived classes override to log frame-specific information.
     *          
     * @note Virtual function - default implementation does nothing
     * @note Called at 10Hz from main logging loop
     * @note Only available when HAL_LOGGING_ENABLED is defined
     */
    virtual void Log_Write() {};
#endif

    /**
     * @enum MotorOptions
     * @brief Motor behavior option flags (bitmask)
     * 
     * @details Configurable options that modify motor behavior. Set via MOT_OPTIONS parameter.
     */
    enum MotorOptions : uint8_t {
        BATT_RAW_VOLTAGE = (1 << 0U)  ///< Use raw battery voltage for thrust compensation (no sag correction)
    };
    
    /**
     * @brief Check if a motor option flag is enabled
     * 
     * @param[in] option MotorOptions flag to check
     * 
     * @return true if option is enabled, false otherwise
     * 
     * @note Options are set via MOT_OPTIONS parameter (bitmask)
     */
    bool has_option(MotorOptions option) { return _options.get() & uint8_t(option); }

protected:
    /**
     * @brief Output motor commands when armed and stabilizing
     * 
     * @details Pure virtual function that must be implemented by derived classes.
     *          Called by output() when vehicle is armed and actively stabilizing.
     *          Implements the actual motor mixing algorithm and sends commands to ESCs.
     *          
     * @note Pure virtual - must be implemented by derived classes
     * @note Called from output() when armed and interlock enabled
     */
    virtual void        output_armed_stabilizing() = 0;
    
    /**
     * @brief Write servo angle command to output channel
     * 
     * @param[in] chan     Output channel number (0-based)
     * @param[in] angle_cd Servo angle in centidegrees
     * 
     * @details Converts angle in centidegrees to PWM and writes to output channel.
     *          Used for servo outputs (e.g., tricopter tail servo, tiltrotor servos).
     *          
     * @note Virtual function - may be overridden for protocol-specific handling
     * @note angle_cd: -18000 to +18000 centidegrees (-180° to +180°)
     */
    virtual void        rc_write_angle(uint8_t chan, int16_t angle_cd);
    
    /**
     * @brief Set PWM output frequency for specified channels
     * 
     * @param[in] mask    Bitmask of channels to configure (bit 0 = channel 1, etc.)
     * @param[in] freq_hz Output frequency in Hz
     * 
     * @details Sets the PWM update frequency for specified output channels.
     *          Used to configure different frequencies for motors vs servos.
     *          
     * @note Virtual function - may be overridden for HAL-specific handling
     * @note Common frequencies: 50Hz (analog servos), 400Hz (digital servos), 490Hz (motors)
     */
    virtual void        rc_set_freq(uint32_t mask, uint16_t freq_hz);


    /**
     * @brief Map internal motor mask to actual servo channel mask
     * 
     * @param[in] mask Internal motor bitmask
     * 
     * @return Servo channel output bitmask
     * 
     * @details Converts internal motor numbering to actual hardware output channels
     *          accounting for SERVOn_FUNCTION parameter mappings. Handles cases where
     *          multiple outputs map to same motor number (e.g., redundant ESCs).
     *          
     * @note Accounts for user-configurable SERVOn_FUNCTION mappings
     */
    uint32_t    motor_mask_to_srv_channel_mask(uint32_t mask) const;

    /**
     * @brief Register a motor number in the motor map
     * 
     * @param[in] motor_num Motor number to add (0-based index)
     * 
     * @details Adds a motor to the internal motor map for tracking active motors.
     *          Used during initialization to build the list of configured motors.
     */
    void add_motor_num(int8_t motor_num);
    
    /**
     * @brief Update throttle input filtering
     * 
     * @details Pure virtual function that must be implemented by derived classes.
     *          Applies low-pass filtering to throttle input to smooth pilot commands
     *          and calculate throttle slew rate.
     *          
     * @note Pure virtual - must be implemented by derived classes
     * @note Typically called at beginning of output() function
     */
    virtual void update_throttle_filter() = 0;

    /**
     * @brief Save parameters to EEPROM on disarm
     * 
     * @details Virtual function called when vehicle disarms. Derived classes can
     *          override to save learned parameters (e.g., hover throttle for multirotors).
     *          Default implementation does nothing.
     *          
     * @note Virtual function - override if parameters should be saved on disarm
     * @note Only parameters marked for save-on-disarm should be saved here
     */
    virtual void save_params_on_disarm() {}

    /**
     * @name Protected Member Variables
     * @brief Internal state variables used by motor control implementations
     * @{
     */
    
    float               _dt_s;                      ///< Time step in seconds since last motor update (typically 0.0025s for 400Hz)
    uint16_t            _speed_hz;                  ///< Motor output update rate in Hz (typically 490Hz)
    float               _roll_in;                   ///< Desired roll control from attitude controller, range -1.0 to +1.0
    float               _roll_in_ff;                ///< Desired roll feedforward control for direct control authority, range -1.0 to +1.0
    float               _pitch_in;                  ///< Desired pitch control from attitude controller, range -1.0 to +1.0
    float               _pitch_in_ff;               ///< Desired pitch feedforward control for direct control authority, range -1.0 to +1.0
    float               _yaw_in;                    ///< Desired yaw control from attitude controller, range -1.0 to +1.0
    float               _yaw_in_ff;                 ///< Desired yaw feedforward control for direct control authority, range -1.0 to +1.0
    float               _throttle_in;               ///< Raw throttle input from set_throttle(), range 0.0 to 1.0
    float               _throttle_out;              ///< Final throttle output after mixing and limiting, range 0.0 to 1.0
    float               _throttle_slew_rate;        ///< Rate of throttle change in per-second (for slew rate limiting)
    float               _forward_in;                ///< Forward/backward control input for 6DOF vehicles, range -1.0 to +1.0
    float               _lateral_in;                ///< Left/right control input for 6DOF vehicles, range -1.0 to +1.0
    float               _throttle_avg_max;          ///< Maximum average throttle across all motors (for thrust limiting)
    LowPassFilterFloat  _throttle_filter;           ///< Low-pass filter for pilot throttle input smoothing
    DerivativeFilterFloat_Size7  _throttle_slew;    ///< Derivative filter to detect throttle slew rate
    LowPassFilterFloat  _throttle_slew_filter;      ///< Low-pass filter for smoothed throttle slew rate output
    DesiredSpoolState   _spool_desired;             ///< Requested spool state from flight mode
    SpoolState          _spool_state;               ///< Current actual spool state of motors

    uint32_t            _motor_fast_mask;           ///< Bitmask of channels requiring fast output (high update rate)

    /**
     * @brief PWM scaling parameters for PWM_RANGE and PWM_ANGLE modes
     * 
     * @details Used when PWM output needs custom scaling instead of standard 1000-2000µs range
     */
    struct {
        uint32_t mask;      ///< Bitmask of motors using scaled output
        float offset;       ///< Offset value to convert from PWM to scaled output
    } _motor_pwm_scaled;

    
    /**
     * @name Radio Passthrough Variables
     * @brief Direct pilot input values for display and setup
     * @{
     */
    float _roll_radio_passthrough;     ///< Direct pilot roll input, range -1.0 to +1.0 (for setup/feedback only)
    float _pitch_radio_passthrough;    ///< Direct pilot pitch input, range -1.0 to +1.0 (for setup/feedback only)
    float _throttle_radio_passthrough; ///< Direct pilot throttle input, range 0.0 to 1.0 (for setup/feedback only)
    float _yaw_radio_passthrough;      ///< Direct pilot yaw input, range -1.0 to +1.0 (for setup/feedback only)
    /** @} */

    /**
     * @enum PWMType
     * @brief ESC protocol/PWM output type
     * 
     * @details Defines the communication protocol used to send motor commands to ESCs.
     *          Different protocols have different timing, resolution, and capabilities.
     */
    enum class PWMType : uint8_t {
        NORMAL     = 0,  ///< Standard analog PWM (1000-2000µs pulses, typically 50-490Hz)
        ONESHOT    = 1,  ///< OneShot (125-250µs pulses, 8x faster than standard PWM)
        ONESHOT125 = 2,  ///< OneShot125 (125-250µs pulses with 125µs base)
        BRUSHED    = 3,  ///< Brushed motor control (continuous PWM for voltage control)
        DSHOT150   = 4,  ///< DShot digital protocol at 150kbit/s
        DSHOT300   = 5,  ///< DShot digital protocol at 300kbit/s
        DSHOT600   = 6,  ///< DShot digital protocol at 600kbit/s
        DSHOT1200  = 7,  ///< DShot digital protocol at 1200kbit/s
        PWM_RANGE  = 8,  ///< PWM with custom range scaling
        PWM_ANGLE  = 9,  ///< PWM with angle-based scaling (for servos)
    };

    AP_Enum<PWMType>             _pwm_type;            ///< Configured PWM output protocol type (from MOT_PWM_TYPE parameter)

    /**
     * @name Motor Failure Handling
     * @brief Variables for motor failure compensation
     * @{
     */
    bool                _thrust_boost;          ///< True if thrust boost enabled to compensate for failed motor
    bool                _thrust_balanced;       ///< True when motor outputs are well balanced (no failure detected)
    float               _thrust_boost_ratio;    ///< Mixing ratio between highest and second-highest motor (0=normal, 1=boost)
    /** @} */

    AP_Int16            _options;               ///< Motor option flags bitmask (MOT_OPTIONS parameter)

    MAV_TYPE _mav_type;                         ///< MAVLink vehicle type identifier for this frame configuration

    /**
     * @brief Get frame class string for this motor configuration
     * @return String describing frame class (e.g., "QUAD", "HEXA", "HELI")
     * @note Pure virtual - must be implemented by derived classes
     */
    virtual const char* _get_frame_string() const = 0;

    /**
     * @brief Get frame type string for this motor configuration
     * @return String describing frame type (e.g., "X", "PLUS", "H") or empty string if not applicable
     * @note Virtual function - override to provide frame type string
     */
    virtual const char* get_type_string() const { return ""; }

    /**
     * @brief Internal implementation of motor test sequence
     * 
     * @param[in] motor_seq Motor sequence number (1-based, 1 to num_motors)
     * @param[in] pwm       PWM value in microseconds (typically 1000-2000µs)
     * 
     * @details Pure virtual function that implements the actual motor test output.
     *          Called by public output_test_seq() after safety checks.
     *          
     * @note Pure virtual - must be implemented by derived classes
     * @warning WILL SPIN MOTORS - Remove propellers before testing
     */
    virtual void _output_test_seq(uint8_t motor_seq, int16_t pwm) = 0;

#if AP_SCRIPTING_ENABLED
    /**
     * @brief Custom frame description string from Lua scripting
     * 
     * @details Pointer to dynamically allocated string set by Lua scripts.
     *          Allows custom frame descriptions for scripting-based motor control.
     *          
     * @note Only available when AP_SCRIPTING_ENABLED is defined
     * @note Memory managed by set_frame_string() function
     */
    char* custom_frame_string;

    /**
     * @brief Motor limit flags set by Lua scripting
     * 
     * @details Allows Lua scripts to report motor saturation for custom motor mixing.
     *          Used with MOTOR_FRAME_SCRIPTING_MATRIX and similar scripting modes.
     *          
     * @note Only available when AP_SCRIPTING_ENABLED is defined
     * @see set_external_limits()
     */
    AP_Motors_limit external_limits;
#endif

private:
    /**
     * @name Private Member Variables
     * @brief Internal state variables not accessible to derived classes
     * @{
     */

    bool _armed;             ///< Motor armed state: true if armed, false if disarmed
    bool _interlock;         ///< Motor interlock state: true=motors can run, false=motors disabled (emergency stop)
    bool _initialised_ok;    ///< Initialization status: true if motor setup successful, false if failed
    bool _spoolup_block;     ///< Spoolup blocking flag: true prevents motors from spooling up even if armed

    static AP_Motors *_singleton;  ///< Singleton instance pointer for global access
    /** @} */
};

/**
 * @namespace AP
 * @brief ArduPilot global singleton accessors
 */
namespace AP {
    /**
     * @brief Access the global AP_Motors singleton instance
     * 
     * @return Pointer to the global AP_Motors instance, or nullptr if not initialized
     * 
     * @details Convenience accessor for the global motor control object.
     *          Equivalent to AP_Motors::get_singleton().
     *          
     * @note Returns nullptr if motors have not been initialized yet
     * @note Prefer using this over direct singleton access for cleaner code
     * 
     * Example usage:
     * @code
     * AP_Motors *motors = AP::motors();
     * if (motors != nullptr) {
     *     motors->set_throttle(0.5f);
     *     motors->output();
     * }
     * @endcode
     */
    AP_Motors *motors();
};
