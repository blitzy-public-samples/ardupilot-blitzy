/**
 * @file AP_MotorsHeli.h
 * @brief Base class for traditional helicopter motor control
 * 
 * @details This file defines the AP_MotorsHeli base class, which provides the foundation
 *          for all traditional helicopter variants in ArduPilot (Single, Dual, Quad rotor).
 *          
 *          Unlike multirotors that control thrust via motor speed, helicopters use:
 *          - Collective pitch: Primary vertical thrust control by changing blade pitch angle
 *          - Cyclic pitch: Attitude control via swashplate tilting
 *          - Rotor Speed Control (RSC): Maintains constant rotor RPM independent of collective
 *          
 *          This base class handles common helicopter functionality including rotor speed
 *          management, collective pitch calculations, hover learning, and autorotation support.
 * 
 * @note Helicopters require fundamentally different control logic than multirotors.
 *       Motor speed remains relatively constant while blade pitch varies.
 * 
 * @warning Proper swashplate setup and calibration is mandatory before any flight attempts.
 *          Incorrect collective limits can cause rotor stall or dangerous overspeeding.
 */
#pragma once

#include <inttypes.h>

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>
#include "AP_Motors_Class.h"
#include "AP_MotorsHeli_RSC.h"

// servo output rates
#define AP_MOTORS_HELI_SPEED_DEFAULT            125     // default servo update rate for helicopters

// default swash min and max angles and positions
#define AP_MOTORS_HELI_SWASH_CYCLIC_MAX         2500
#define AP_MOTORS_HELI_COLLECTIVE_MIN           1250
#define AP_MOTORS_HELI_COLLECTIVE_MAX           1750
#define AP_MOTORS_HELI_COLLECTIVE_HOVER_DEFAULT 0.5f  // the estimated hover throttle, 0 ~ 1
#define AP_MOTORS_HELI_COLLECTIVE_HOVER_TC      10.0f // time constant used to update estimated hover throttle, 0 ~ 1
#define AP_MOTORS_HELI_COLLECTIVE_HOVER_MIN     0.3f  // minimum possible hover throttle
#define AP_MOTORS_HELI_COLLECTIVE_HOVER_MAX     0.8f // maximum possible hover throttle
#define AP_MOTORS_HELI_COLLECTIVE_MIN_DEG      -90.0f // minimum collective blade pitch angle in deg
#define AP_MOTORS_HELI_COLLECTIVE_MAX_DEG       90.0f // maximum collective blade pitch angle in deg
#define AP_MOTORS_HELI_COLLECTIVE_LAND_MIN      -2.0f // minimum landed collective blade pitch angle in deg for modes using althold


// flybar types
#define AP_MOTORS_HELI_NOFLYBAR                 0

// rsc function output channels.
#define AP_MOTORS_HELI_RSC                      CH_8

class AP_HeliControls;

/**
 * @class AP_MotorsHeli
 * @brief Base class for traditional helicopter motor control
 * 
 * @details AP_MotorsHeli serves as the abstract base class for all traditional helicopter
 *          variants including single rotor (AP_MotorsHeli_Single), dual rotor tandem/transverse
 *          (AP_MotorsHeli_Dual), and quad rotor (AP_MotorsHeli_Quad) configurations.
 *          
 *          Key Helicopter Control Concepts:
 *          
 *          **Collective Pitch Control:**
 *          - Primary method of thrust control, NOT motor speed
 *          - Increases/decreases pitch angle of all rotor blades simultaneously
 *          - Typical range: -2° to +12° blade angle (varies by helicopter)
 *          - PWM range typically 1250-1750 μs maps to min/max collective
 *          - Zero-thrust collective (hover baseline) around 0-5° depending on blade design
 *          
 *          **Cyclic Pitch Control:**
 *          - Controls helicopter attitude (roll/pitch) via swashplate
 *          - Varies blade pitch cyclically as rotor rotates
 *          - Tilts rotor disk to generate horizontal thrust components
 *          - Maximum cyclic typically 10-12° deflection from neutral
 *          
 *          **Rotor Speed Control (RSC):**
 *          - Maintains constant rotor RPM independent of collective demand
 *          - Modes: Throttle curve, governor, external ESC governor
 *          - Typical head speed: 1500-2200 RPM (scale dependent)
 *          - Critical safety threshold must be exceeded before flight
 *          
 *          **Swashplate Mechanics:**
 *          - Mechanical linkage translating servo positions to blade pitch
 *          - Three or more servos control swashplate position
 *          - Mixing converts roll/pitch/collective inputs to servo commands
 *          - Requires careful geometric calibration
 *          
 *          **Throttle Curve (Piston/Turbine Engines):**
 *          - Maps collective position to engine throttle
 *          - Compensates for increased power demand at high collective
 *          - Not used with electric motors (ESC governor handles this)
 *          
 *          **Governor (Electric Motors):**
 *          - Electronic rotor speed regulation using motor RPM feedback
 *          - Automatically adjusts motor power to maintain target RPM
 *          - Compensates for changing collective load
 *          
 * @note This is an abstract base class - use derived classes for specific helicopter types:
 *       - AP_MotorsHeli_Single: Single main rotor + tail rotor
 *       - AP_MotorsHeli_Dual: Tandem or transverse dual rotor
 *       - AP_MotorsHeli_Quad: Quadcopter-style collective pitch configuration
 * 
 * @warning Helicopter control is fundamentally different from multicopter control.
 *          Improper collective limits or rotor speed settings can result in:
 *          - Rotor blade stall (insufficient lift, immediate loss of control)
 *          - Rotor overspeed (structural failure risk)
 *          - Insufficient rotor inertia for autorotation recovery
 * 
 * @warning Rotor MUST reach critical speed (rotor_runup_complete) before takeoff.
 *          Taking off with insufficient rotor speed will result in crash.
 *          Typical runup time: 3-10 seconds depending on rotor inertia.
 * 
 * @warning Swashplate binding or servo mechanical issues can cause rapid loss of control.
 *          Always verify free swashplate movement during ground testing.
 * 
 * @see AP_MotorsHeli_Single for single rotor helicopters
 * @see AP_MotorsHeli_Dual for tandem/transverse configurations  
 * @see AP_MotorsHeli_Quad for collective pitch quadcopters
 * @see AP_MotorsHeli_RSC for rotor speed controller implementation
 */
class AP_MotorsHeli : public AP_Motors {
public:

    /**
     * @brief Constructor for AP_MotorsHeli base class
     * 
     * @param[in] speed_hz Servo update rate in Hz (default: 125 Hz for helicopters)
     * 
     * @details Initializes the helicopter motor control system with specified servo update rate.
     *          Default rate of 125 Hz is suitable for most helicopter servos. Higher rates
     *          (up to 333 Hz) may be used with digital servos for improved response.
     *          
     *          Also initializes the main rotor RSC (Rotor Speed Controller) on channel 8.
     * 
     * @note Helicopter servo update rates are typically lower than multicopter ESC rates
     *       to avoid overheating analog servos and reduce electrical noise.
     */
    AP_MotorsHeli( uint16_t speed_hz = AP_MOTORS_HELI_SPEED_DEFAULT) :
        AP_Motors(speed_hz),
        _main_rotor(SRV_Channel::k_heli_rsc, AP_MOTORS_HELI_RSC, 0U)
    {
        AP_Param::setup_object_defaults(this, var_info);
    };

    /**
     * @brief Initialize helicopter-specific motor control systems
     * 
     * @param[in] frame_class Motor frame class (should be MOTOR_FRAME_HELI for helicopters)
     * @param[in] frame_type Specific helicopter type (single, dual, quad rotor configuration)
     * 
     * @details Initializes helicopter-specific control systems including:
     *          - Rotor speed controller (RSC) initialization
     *          - Swashplate servo initialization and calibration checks
     *          - Collective pitch range setup from parameters
     *          - Governor or throttle curve mode configuration
     *          - Autorotation system initialization
     *          
     *          This method must be called before any motor output commands.
     * 
     * @note Called during vehicle initialization, typically in Copter::init_ardupilot()
     * @warning Must complete successfully before armed flight is permitted
     */
    void init(motor_frame_class frame_class, motor_frame_type frame_type) override;

    /**
     * @brief Set helicopter frame class and type
     * 
     * @param[in] frame_class Motor frame class (MOTOR_FRAME_HELI for helicopters)
     * @param[in] frame_type Specific helicopter configuration type
     * 
     * @details Sets the frame configuration for the helicopter. Frame types include:
     *          - Single rotor: Traditional main rotor + tail rotor
     *          - Dual rotor: Tandem or transverse counter-rotating
     *          - Quad rotor: Four rotors with collective pitch control
     */
    void set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override {
        _frame_class = frame_class;
        _frame_type = frame_type;
    }

    // set update rate to motors - a value in hertz
    virtual void set_update_rate( uint16_t speed_hz ) override = 0;

    /**
     * @brief Set servos to safe neutral positions with rotor stopped
     * 
     * @details Outputs minimum/safe servo positions when disarmed or in failsafe:
     *          - Swashplate servos: Centered (neutral cyclic, minimum collective)
     *          - Tail rotor servo: Centered (neutral yaw)
     *          - RSC output: Minimum (motor stopped or idle)
     *          
     *          This prevents uncommanded helicopter movement on the ground and
     *          ensures safe servo positions during power-up or after disarm.
     * 
     * @note Called automatically when disarmed or during initialization
     * @warning Swashplate must move freely - binding can prevent reaching neutral
     */
    void output_min() override;

    //
    // heli specific methods
    //

    //set turbine start flag on to initiaize starting sequence
    void set_turb_start(bool turb_start) { _heliflags.start_engine = turb_start; }

    // has_flybar - returns true if we have a mechical flybar
    virtual bool has_flybar() const { return AP_MOTORS_HELI_NOFLYBAR; }

    /**
     * @brief Set special collective limits for landing condition
     * 
     * @param[in] landing True when helicopter is landed or landing, false otherwise
     * 
     * @details When landing flag is set, prevents collective from going below the landing
     *          minimum (H_COL_LAND_MIN parameter, typically -2°). This prevents the helicopter
     *          from settling excessively into the ground with negative collective, which could
     *          cause rotor strikes or tip-overs on uneven terrain.
     *          
     *          During normal flight, lower collective values may be used for descents.
     *          
     *          This is automatically managed by flight mode logic - modes using altitude hold
     *          set this true when detecting landing, while manual throttle modes leave it false.
     * 
     * @note Typical H_COL_LAND_MIN: -2° to 0° depending on helicopter design
     * @warning Essential for safe landing - prevents collective going too negative on ground
     */
    void set_collective_for_landing(bool landing) { _heliflags.landing_collective = landing; }

    /**
     * @brief Get current rotor speed control (RSC) mode
     * 
     * @return RSC mode value (see AP_MotorsHeli_RSC::ControlModes)
     * 
     * @details Returns the currently active RSC mode:
     *          - Mode 1: Throttle curve (open-loop, for piston/turbine engines)
     *          - Mode 2: External governor (ESC handles RPM regulation)
     *          - Mode 3: Throttle curve with idle (for turbine start sequences)
     *          - Mode 4: Internal governor (ArduPilot maintains RPM via feedback)
     *          - Mode 5: Autorotation mode (motor off, windmilling rotor)
     * 
     * @note Mode selection via H_RSC_MODE parameter
     */
    uint8_t get_rsc_mode() const { return _main_rotor.get_control_mode(); }

    /**
     * @brief Get target rotor speed setpoint
     * 
     * @return Target rotor speed as normalized value (0.0 to 1.0)
     * 
     * @details Returns the configured rotor speed setpoint from H_RSC_SETPOINT parameter.
     *          This represents the target percentage of maximum rotor RPM:
     *          - 0.0 = 0% (rotor stopped)
     *          - 1.0 = 100% (maximum configured RPM)
     *          - Typical values: 0.70 to 0.90 (70-90% of maximum)
     *          
     *          The actual RPM corresponding to this setpoint depends on motor KV,
     *          voltage, and physical rotor characteristics. Most helicopters operate
     *          at 75-85% of theoretical maximum for optimal efficiency and headroom.
     * 
     * @note H_RSC_SETPOINT parameter range: 0-100 (converted to 0.0-1.0)
     * @warning Setpoint too low (<50%) may not provide sufficient rotor inertia for autorotation
     * @warning Setpoint too high (>95%) reduces governor headroom for load compensation
     */
    float get_rsc_setpoint() const { return _main_rotor._rsc_setpoint.get() * 0.01f; }

    /**
     * @brief Set desired rotor speed for dynamic operation
     * 
     * @param[in] desired_speed Target rotor speed (0.0 = stopped, 1.0 = full setpoint speed)
     * 
     * @details Allows flight modes to dynamically adjust rotor speed during flight.
     *          Typically used for:
     *          - Reducing rotor speed during descent to save power
     *          - Increasing rotor speed for aggressive maneuvers
     *          - Ramping down rotor speed for landing
     *          - Autorotation entry (setting to 0.0 with motor off)
     *          
     *          The desired speed is scaled by H_RSC_SETPOINT to get actual target.
     *          For example, with H_RSC_SETPOINT=80 and desired_speed=0.9:
     *          Actual target = 80% * 0.9 = 72% of maximum RPM
     * 
     * @note Most flight modes use 1.0 (full H_RSC_SETPOINT speed)
     * @note Autorotation mode uses 0.0 (motor off, windmilling)
     */
    virtual void set_desired_rotor_speed(float desired_speed);

    // get_desired_rotor_speed - gets target rotor speed as a number from 0 ~ 1
    float get_desired_rotor_speed() const { return _main_rotor.get_desired_speed(); }

    /**
     * @brief Check if main rotor has completed runup to flight speed
     * 
     * @return true if rotor is at sufficient speed for flight, false otherwise
     * 
     * @details Critical safety check that must return true before helicopter can take off.
     *          Rotor runup completion requires:
     *          1. Rotor speed reached target setpoint (within threshold)
     *          2. Sufficient time elapsed for rotor to stabilize (typically 3-10 seconds)
     *          3. Governor locked (if using governor mode)
     *          
     *          During runup phase after arming:
     *          - Collective is limited to prevent premature liftoff
     *          - Motors ramp up to target speed
     *          - Rotor accelerates against air resistance
     *          - System waits for stable RPM
     *          
     *          Flight controller will not allow takeoff or altitude hold modes until
     *          this returns true. Attempting flight with incomplete runup results in
     *          insufficient lift and immediate loss of control.
     * 
     * @note Typical runup time: 3-10 seconds depending on rotor size/inertia
     * @note Runup time configured via H_RSC_RUNUP_TIME parameter (default 10 seconds)
     * 
     * @warning CRITICAL SAFETY CHECK - Never attempt takeoff if this returns false
     * @warning Insufficient rotor speed = insufficient lift = crash on takeoff attempt
     * @warning Wind gusts during incomplete runup can destabilize aircraft
     * 
     * @see AP_MotorsHeli_RSC::rotor_speed_above_critical()
     */
    bool rotor_runup_complete() const { return _heliflags.rotor_runup_complete; }

    // get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    virtual uint32_t get_motor_mask() override;

    virtual void set_acro_tail(bool set) {}

    // ext_gyro_gain - set external gyro gain in range 0 ~ 1
    virtual void ext_gyro_gain(float gain) {}

    /**
     * @brief Send motor and servo commands to hardware
     * 
     * @details Main output function called at scheduler rate (typically 400 Hz) to:
     *          1. Run rotor speed control logic (RSC governor/throttle curve)
     *          2. Calculate swashplate servo positions from roll/pitch/collective
     *          3. Apply servo mixing for specific helicopter configuration
     *          4. Output PWM signals to all servos and ESC
     *          5. Update rotor runup state machine
     *          6. Handle autorotation state transitions
     *          
     *          This function coordinates the complex interactions between collective,
     *          cyclic, rotor speed, and yaw control to generate final servo commands.
     * 
     * @note Called from main loop at AP_MOTORS_MATRIX_YAW_RATE_MAX (typically 400 Hz)
     * @note Higher call rate than servo update rate allows precise rotor speed control
     */
    void output() override;

    // supports_yaw_passthrough
    virtual bool supports_yaw_passthrough() const { return false; }

    /**
     * @brief Update learned collective pitch required to hover
     * 
     * @param[in] dt Time step in seconds since last update
     * 
     * @details Helicopter equivalent of throttle hover learning for multirotors.
     *          Learns the collective pitch position required to maintain hover by
     *          monitoring pilot collective input when in stabilized hover conditions.
     *          
     *          Learning occurs when:
     *          - Helicopter is armed and flying
     *          - Altitude hold mode is active (or manual hover maintained)
     *          - Minimal horizontal or vertical movement
     *          - Sufficient rotor speed (runup complete)
     *          
     *          Learned value stored in H_COL_HOVER parameter (range 0.3-0.8, typically 0.4-0.5).
     *          This helps altitude hold modes better estimate required collective.
     * 
     * @note Learning rate controlled by H_COL_HOVER_TC (time constant, default 10 seconds)
     * @note Enable via H_COL_HOVER_LEARN: 0=disabled, 1=learn only, 2=learn and save
     * @note Hover collective varies with helicopter weight, DA, blade pitch range
     */
    void update_throttle_hover(float dt);
    
    /**
     * @brief Get learned hover collective pitch position
     * 
     * @return Normalized collective position required to hover (0.3 to 0.8)
     * 
     * @details Returns the learned or configured collective pitch position that produces
     *          zero vertical acceleration (hover). Used by altitude hold controller as
     *          feedforward term to reduce learning time and improve initial response.
     *          
     *          Value is constrained to safe range (0.3-0.8) to prevent unrealistic
     *          collective positions from corrupting altitude hold performance.
     * 
     * @note Corresponds to H_COL_HOVER parameter value
     */
    float get_throttle_hover() const override { return constrain_float(_collective_hover, AP_MOTORS_HELI_COLLECTIVE_HOVER_MIN, AP_MOTORS_HELI_COLLECTIVE_HOVER_MAX); }

    // accessor to get the takeoff collective flag signifying that current collective is greater than collective required to indicate takeoff
    bool get_takeoff_collective() const { return _heliflags.takeoff_collective; }

    // accessor to get the land min collective flag signifying that current collective is lower than collective required for landing
    bool get_below_land_min_coll() const { return _heliflags.below_land_min_coll; }

    // support passing init_targets_on_arming flag to greater code
    bool init_targets_on_arming() const override { return _heliflags.init_targets_on_arming; }

    /**
     * @brief Activate or deactivate autorotation mode
     * 
     * @param[in] tf True to enable autorotation, false to disable
     * 
     * @details Autorotation is the emergency flight mode used when engine/motor fails.
     *          The rotor continues spinning due to upward airflow through the rotor disk,
     *          storing kinetic energy that can be used for a controlled landing flare.
     *          
     *          When activated:
     *          - Motor is immediately cut (RSC to zero)
     *          - Collective is reduced to minimize rotor drag
     *          - Helicopter enters controlled descent (~1500 ft/min)
     *          - Pilot uses stored rotor inertia to arrest descent during flare
     *          
     *          ArduPilot's autorotation system:
     *          - Automatically detects motor failure
     *          - Transitions to autorotation RSC mode
     *          - Manages entry/descent/bail-out phases
     *          - Assists with landing flare collective management
     * 
     * @note Requires sufficient altitude (>100ft) for successful autorotation
     * @note Rotor must maintain minimum RPM (typically 80-90% of normal) during descent
     * @warning Autorotation requires pilot skill - practice in simulator first
     * @warning Insufficient rotor RPM during flare = hard landing/crash
     * 
     * @see AP_MotorsHeli_RSC::Autorotation for state machine details
     */
    void set_autorotation_active(bool tf) { _main_rotor.autorotation.set_active(tf, false); }

    // helper to force the RSC autorotation state to deactivated
    void force_deactivate_autorotation(void) { _main_rotor.autorotation.set_active(false, true); }

    /**
     * @brief Check if helicopter is currently in autorotation mode
     * 
     * @return true if autorotating or executing bailout, false otherwise
     * 
     * @details Returns true during any autorotation phase:
     *          - Entry: Initial motor cutoff and collective reduction
     *          - Descent: Steady-state autorotation descent
     *          - Bailout: Re-engaging motor power (recovery from autorotation)
     *          
     *          Flight modes use this to modify control behavior during autorotation:
     *          - Disable altitude hold (glide slope control instead)
     *          - Modify attitude controller gains for unpowered flight
     *          - Enable autorotation-specific navigation modes
     * 
     * @note Autorotation mode persists until bailout completes or landing detected
     */
    bool in_autorotation(void) const { return _main_rotor.in_autorotation(); }

    /**
     * @brief Check if currently bailing out of autorotation
     * 
     * @return true if motor is being re-engaged after autorotation, false otherwise
     * 
     * @details Bailout is the transition from autorotation back to powered flight.
     *          Occurs when:
     *          - Pilot manually aborts autorotation (if altitude permits)
     *          - Motor/engine restarts after failure
     *          - Practice autorotation is terminated
     *          
     *          During bailout:
     *          - Motor power gradually restored (prevents torque spike)
     *          - Rotor speed governor re-engaged
     *          - Collective management transitioned back to normal control
     *          - Flight mode may return to previous mode after stabilization
     * 
     * @note Bailout requires sufficient altitude and rotor RPM
     * @warning Bailout too late (low altitude/low RPM) may not prevent ground contact
     */
    bool autorotation_bailout(void) const { return _main_rotor.autorotation.bailing_out(); }

    // true if the autorotation functionality within the rsc has been enabled
    bool rsc_autorotation_enabled(void) const { return _main_rotor.autorotation.enabled(); }

    // set land complete flag
    void set_land_complete(bool landed) { _heliflags.land_complete = landed; }

    /**
     * @brief Set normalized collective position from desired blade pitch angle
     * 
     * @param[in] col_ang_deg Desired blade pitch angle in degrees
     * 
     * @details Converts a physical blade pitch angle (in degrees) to the normalized
     *          collective position (0.0 to 1.0) used internally by the motor library.
     *          
     *          Conversion uses the configured collective pitch range:
     *          - H_COL_MIN_DEG: Minimum blade angle (typically -2° to 0°)
     *          - H_COL_MAX_DEG: Maximum blade angle (typically +10° to +14°)
     *          - H_COL_MID (zero thrust): Zero-lift blade angle (typically 0° to +5°)
     *          
     *          Formula: normalized = (angle - min) / (max - min)
     *          
     *          This allows flight modes to command specific blade angles for:
     *          - Autorotation entry collective
     *          - Landing flare collective
     *          - Known aerodynamic configurations
     * 
     * @note Blade pitch angles are measured from zero-lift line, not horizontal
     * @note Typical helicopter pitch range: -2° to +12° (-2000 to +2000 in PWM scaling)
     */
    void set_coll_from_ang(float col_ang_deg);

    //return zero lift collective position
    float get_coll_mid() const { return _collective_zero_thrust_pct; }

    /**
     * @enum HeliOption
     * @brief Optional helicopter features bitmask
     * 
     * @details Bitmask flags for enabling optional helicopter-specific features via
     *          the H_OPTIONS parameter. Multiple options can be combined using bitwise OR.
     *          
     *          **USE_LEAKY_I (bit 0, value 1):**
     *          Enables "leaky" integrator management for attitude control. When enabled,
     *          integrator terms gradually decay ("leak") rather than being held constant.
     *          This prevents integrator windup during aggressive maneuvers and provides
     *          better recovery from unusual attitudes. Particularly useful for aerobatic
     *          helicopters or 3D flight.
     *          
     *          Benefits of leaky integrator:
     *          - Prevents integrator windup during flips/rolls
     *          - Faster recovery from extreme attitudes
     *          - Reduced drift accumulation
     *          - Better performance in variable wind conditions
     *          
     *          Drawbacks:
     *          - Slightly reduced steady-state accuracy in hover
     *          - May require slight increase in I gain to compensate for leak
     * 
     * @note Set via H_OPTIONS parameter (default: 0 = all options disabled)
     * @note Value 1 enables leaky integrator (H_OPTIONS = 1)
     */
    enum class HeliOption {
        USE_LEAKY_I                     = (1<<0),   // 1
    };

    // use leaking integrator management scheme
    bool using_leaky_integrator() const { return heli_option(HeliOption::USE_LEAKY_I); }

    /**
     * @brief Perform helicopter-specific pre-arm safety checks
     * 
     * @param[in] buflen Maximum length of error message buffer
     * @param[out] buffer Buffer to store failure messages (if any)
     * @return true if all checks pass, false if any check fails
     * 
     * @details Validates helicopter configuration before allowing arming. Checks include:
     *          
     *          **Collective Pitch Range:**
     *          - H_COL_MIN < H_COL_MID < H_COL_MAX (proper servo range)
     *          - H_COL_MIN_DEG < H_COL_MAX_DEG (valid blade angle range)
     *          - Zero-thrust collective (H_COL_MID) within expected range
     *          
     *          **Rotor Speed Controller:**
     *          - Valid RSC mode selected (H_RSC_MODE)
     *          - RSC setpoint within valid range (H_RSC_SETPOINT 10-100%)
     *          - Governor configured if using internal governor mode
     *          - Throttle curve points valid if using curve mode
     *          
     *          **Swashplate Configuration:**
     *          - Swashplate type selected and valid
     *          - Servo assignments configured (no conflicts)
     *          - Swashplate geometry parameters within limits
     *          
     *          **Servo Function Assignments:**
     *          - All required servo outputs assigned (swash plate, tail rotor, RSC)
     *          - No duplicate servo function assignments
     *          - Servo min/max/trim values configured
     *          
     *          Failure messages written to buffer indicate which check failed and
     *          provide guidance for fixing the configuration issue.
     * 
     * @note Called automatically by AP_Arming during pre-arm checks
     * @warning Helicopter WILL NOT ARM if any check fails - all errors must be resolved
     * @warning Flying with misconfigured parameters extremely dangerous
     * 
     * @see AP_Arming::pre_arm_checks()
     */
    bool arming_checks(size_t buflen, char *buffer) const override;

    // Tell user motor test is disabled on heli
    bool motor_test_checks(size_t buflen, char *buffer) const override;

    // output_test_seq - disabled on heli, do nothing
    void _output_test_seq(uint8_t motor_seq, int16_t pwm) override {};

    // Helper function for param conversions to be done in motors class
    virtual void heli_motors_param_conversions(void);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:

    /**
     * @enum ServoControlModes
     * @brief Manual servo control modes for swashplate setup and testing
     * 
     * @details Used during initial helicopter setup to verify servo connections,
     *          directions, and mechanical range before flight. Set via H_SV_MAN parameter.
     *          
     *          AUTOMATED: Normal flight operation (servo commands from flight controller)
     *          MANUAL_PASSTHROUGH: Servos directly follow pilot stick inputs (no mixing)
     *          MANUAL_MAX: All servos move to maximum position (for checking limits)
     *          MANUAL_CENTER: All servos move to center position (for neutral setup)
     *          MANUAL_MIN: All servos move to minimum position (for checking limits)
     *          MANUAL_OSCILLATE: Servos oscillate slowly (for checking mechanical binding)
     * 
     * @warning Only use manual servo modes during ground setup with rotor secured
     * @warning NEVER arm or apply motor power in manual servo modes
     * @warning Check for swashplate binding and servo binding in all positions
     */
    enum ServoControlModes {
        SERVO_CONTROL_MODE_AUTOMATED = 0,
        SERVO_CONTROL_MODE_MANUAL_PASSTHROUGH,
        SERVO_CONTROL_MODE_MANUAL_MAX,
        SERVO_CONTROL_MODE_MANUAL_CENTER,
        SERVO_CONTROL_MODE_MANUAL_MIN,
        SERVO_CONTROL_MODE_MANUAL_OSCILLATE,
    };

    // output - sends commands to the motors
    void output_armed_stabilizing() override;
    void output_disarmed();

    // external objects we depend upon
    AP_MotorsHeli_RSC   _main_rotor;            // main rotor

    /**
     * @brief Update motor control outputs based on rotor control state
     * 
     * @param[in] state Current rotor control state (idle, runup, active, autorotation, etc.)
     * 
     * @details Pure virtual function that derived classes implement to update motor/ESC outputs
     *          based on the current RSC state machine phase. State determines motor behavior:
     *          
     *          - IDLE: Motor at idle speed or stopped
     *          - RUNUP: Motor ramping up to target speed
     *          - ACTIVE: Normal flight operations, governor active
     *          - AUTOROTATION: Motor off, windmilling rotor
     *          - BAILOUT: Motor restarting after autorotation
     *          
     *          Implementation varies by helicopter type (single/dual/quad rotor).
     * 
     * @note Called from output() at main loop rate
     */
    virtual void update_motor_control(AP_MotorsHeli_RSC::RotorControlState state) = 0;

    // Converts AP_Motors::SpoolState from _spool_state variable to AP_MotorsHeli_RSC::RotorControlState
    AP_MotorsHeli_RSC::RotorControlState get_rotor_control_state() const;

    // run spool logic
    void                output_logic();

    // output_to_motors - sends commands to the motors
    virtual void        output_to_motors() = 0;

    // reset_flight_controls - resets all controls and scalars to flight status
    void reset_flight_controls();

    // update the throttle input filter
    void update_throttle_filter() override;

    /**
     * @brief Move swashplate and tail rotor actuators
     * 
     * @param[in] roll_out Desired roll cyclic input (-1.0 to +1.0)
     * @param[in] pitch_out Desired pitch cyclic input (-1.0 to +1.0)
     * @param[in] coll_in Desired collective input (0.0 to 1.0)
     * @param[in] yaw_out Desired yaw (tail rotor) input (-1.0 to +1.0)
     * 
     * @details Pure virtual function implemented by derived classes to convert roll/pitch/
     *          collective/yaw commands into individual servo positions. Implementation is
     *          specific to swashplate type and helicopter configuration.
     *          
     *          Typical implementation sequence:
     *          1. Apply collective-to-throttle curve (if piston/turbine)
     *          2. Calculate swashplate servo positions via mixing matrix
     *          3. Apply servo linearization and trim
     *          4. Constrain to servo min/max limits
     *          5. Output PWM to servos
     *          
     *          Different swashplate types require different mixing:
     *          - H1 (1 servo + flybar): Simplified mixing
     *          - H3/H4: Three or four servo mixing with configurable geometry
     *          - Dual rotor: Tandem/transverse mixing
     *          - Quad rotor: Four independent rotor collective + motor speed
     * 
     * @note Input values are normalized (-1 to +1 or 0 to 1)
     * @note Called from output_to_motors() at servo update rate
     * @warning Incorrect mixing can cause inverted controls or unstable flight
     */
    virtual void move_actuators(float roll_out, float pitch_out, float coll_in, float yaw_out) = 0;

    // init_outputs - initialise Servo/PWM ranges and endpoints.  This
    // method also updates the initialised flag.
    virtual void init_outputs() = 0;

    // calculate_armed_scalars - must be implemented by child classes
    virtual void calculate_armed_scalars() = 0;

    // calculate_scalars - must be implemented by child classes
    virtual void calculate_scalars() = 0;

    // servo_test - move servos through full range of movement
    // to be overloaded by child classes, different vehicle types would have different movement patterns
    virtual void servo_test() = 0;

    // save parameters as part of disarming
    void save_params_on_disarm() override;

    // Determines if _heli_options bit is set
    bool heli_option(HeliOption opt) const;

    // updates the takeoff collective flag indicating that current collective is greater than collective required to indicate takeoff.
    void update_takeoff_collective_flag(float coll_out);

    const char* _get_frame_string() const override { return "HELI"; }

    // update turbine start flag
    void update_turbine_start();

    // Update _heliflags.rotor_runup_complete value writing log event on state change
    void set_rotor_runup_complete(bool new_value);

#if HAL_LOGGING_ENABLED
    // Returns the scaling value required to convert the collective angle parameters into the cyclic-output-to-angle conversion for blade angle logging
    float get_cyclic_angle_scaler(void) const;
#endif

    /**
     * @enum HoverLearn
     * @brief Hover collective learning modes
     * 
     * @details Controls whether and how the helicopter learns the collective position
     *          required to hover. Used by H_COL_HOVER_LEARN parameter.
     *          
     *          **HOVER_LEARN_DISABLED (0):**
     *          Learning disabled. Uses fixed H_COL_HOVER value.
     *          Use when hover collective is known and stable, or for manual tuning.
     *          
     *          **HOVER_LEARN_ONLY (1):**
     *          Learning enabled but not saved to EEPROM.
     *          Learned value resets to parameter value on reboot.
     *          Good for testing or when hover collective varies by battery/payload.
     *          
     *          **HOVER_LEARN_AND_SAVE (2):**
     *          Learning enabled and automatically saved to EEPROM.
     *          Learned value persists across reboots.
     *          Recommended for most users - adapts to weight changes over time.
     *          
     *          Learning process:
     *          - Monitors collective input during stable hover
     *          - Uses exponential filter (time constant H_COL_HOVER_TC)
     *          - Constrained to 0.3-0.8 (30-80% of collective range)
     *          - Only learns when altitude hold active and minimal movement
     * 
     * @note Faster convergence with mode 2 since value persists between flights
     * @note Hover collective varies with helicopter weight, DA, and blade configuration
     */
    enum HoverLearn {
        HOVER_LEARN_DISABLED = 0,
        HOVER_LEARN_ONLY = 1,
        HOVER_LEARN_AND_SAVE = 2
    };

    /**
     * @struct heliflags_type
     * @brief Helicopter state flags for mode and safety management
     * 
     * @details Packed bitfield containing helicopter operational state flags.
     *          These flags track critical state transitions and enable mode-specific
     *          control behaviors.
     *          
     *          **landing_collective**: When true, limits collective minimum to H_COL_LAND_MIN
     *          to prevent excessive negative pitch on ground (prevents blade strikes).
     *          Set by altitude hold modes when landing detected.
     *          
     *          **rotor_runup_complete**: Critical safety flag - true when rotor has reached
     *          target speed and stabilized. Flight controller will not allow takeoff or
     *          altitude hold until this is true. Prevents takeoff with insufficient rotor speed.
     *          
     *          **init_targets_on_arming**: Used by attitude controller to determine if
     *          controller targets need initialization after arming. Prevents sudden movements
     *          when transitioning from disarmed to armed state.
     *          
     *          **save_rsc_mode**: Indicates RSC mode parameter needs to be saved to EEPROM.
     *          Set when RSC mode changes during operation.
     *          
     *          **servo_test_running**: True during automated servo movement tests on bootup.
     *          Prevents normal flight operations during servo testing sequence.
     *          
     *          **land_complete**: Set by landing detector when helicopter is confirmed on ground.
     *          Used to modify control behavior and enable ground-only operations (disarm, etc.).
     *          
     *          **takeoff_collective**: True when collective exceeds 30% of range from mid to max.
     *          Indicates pilot intent to take off, used by takeoff detection logic.
     *          
     *          **below_land_min_coll**: True when collective below H_COL_LAND_MIN.
     *          Additional confirmation of landing condition.
     *          
     *          **rotor_spooldown_complete**: True when rotor has fully stopped after disarm.
     *          Allows safe parameter changes and prevents accidental rotor startup.
     *          
     *          **start_engine**: RC option flag to initiate turbine/engine startup sequence.
     *          Used with turbine helicopters for managed engine start procedure.
     * 
     * @note All flags are single bits to minimize memory footprint
     */
    struct heliflags_type {
        uint8_t landing_collective      : 1;    // true if collective is setup for landing which has much higher minimum
        uint8_t rotor_runup_complete    : 1;    // true if the rotors have had enough time to wind up
        uint8_t init_targets_on_arming  : 1;    // 0 if targets were initialized, 1 if targets were not initialized after arming
        uint8_t save_rsc_mode           : 1;    // used to determine the rsc mode needs to be saved while disarmed
        uint8_t servo_test_running      : 1;    // true if servo_test is running
        uint8_t land_complete           : 1;    // true if aircraft is landed
        uint8_t takeoff_collective      : 1;    // true if collective is above 30% between H_COL_MID and H_COL_MAX
        uint8_t below_land_min_coll     : 1;    // true if collective is below H_COL_LAND_MIN
        uint8_t rotor_spooldown_complete : 1;    // true if the rotors have spooled down completely
        uint8_t start_engine            : 1;    // true if turbine start RC option is initiated
    } _heliflags;

    /**
     * @name Helicopter Configuration Parameters
     * @{
     * 
     * @details Core parameters for helicopter control configuration. These define the mechanical
     *          limits and control behavior of the helicopter. All parameters use H_ prefix.
     *          
     *          CRITICAL PARAMETERS - Must be configured correctly before first flight:
     *          
     *          **H_CYC_MAX** (Cyclic Max):
     *          - Maximum cyclic deflection in centi-degrees (2500 = 25°)
     *          - Controls maximum roll/pitch attitude command authority
     *          - Typical range: 1500-3000 (15° to 30°)
     *          - Too high: Risk of blade stall or excessive control sensitivity
     *          - Too low: Insufficient control authority for aggressive maneuvers
     *          
     *          **H_COL_MIN/MAX** (Collective Servo Range):
     *          - PWM values for minimum/maximum collective servo position
     *          - NOT the same as blade pitch angles (servo output limits)
     *          - Typical: 1250 μs (min) to 1750 μs (max)
     *          - Must allow full blade pitch range without servo binding
     *          - Set using servo travel adjustment (ATV) first, then these limits
     *          
     *          **H_COL_MIN_DEG/MAX_DEG** (Collective Blade Pitch Range):
     *          - Physical blade pitch angles in degrees at servo limits
     *          - Maps servo PWM range to actual blade angles for altitude hold
     *          - Typical: -2° (min) to +12° (max) depending on blade design
     *          - Measured with pitch gauge on main rotor blade
     *          - Used by controller to convert desired pitch to servo position
     *          
     *          **H_COL_MID** (Zero-Thrust Collective):
     *          - Collective position producing zero net thrust (blade zero-lift angle)
     *          - Typical: 0° to +5° depending on blade airfoil
     *          - Used as reference for altitude hold calculations
     *          - Critical for accurate altitude control
     *          
     *          **H_COL_LAND_MIN** (Landing Collective Minimum):
     *          - Minimum collective angle when landed (degrees)
     *          - Prevents excessive negative collective on ground
     *          - Typical: -2° to 0° (prevents rotor blade strikes)
     *          - Used in altitude hold modes when landing detected
     *          
     *          **H_COL_HOVER** (Hover Collective):
     *          - Learned collective position to maintain hover (0.0-1.0)
     *          - Typical: 0.4-0.5 (40-50% of collective range)
     *          - Automatically learned if H_COL_HOVER_LEARN enabled
     *          - Used as feedforward in altitude hold for faster response
     *          - Varies with helicopter weight, density altitude, blade design
     *          
     *          **H_RSC_MODE** (Rotor Speed Control Mode):
     *          - 1: Throttle curve (open-loop, for piston/turbine)
     *          - 2: External governor (ESC handles RPM)
     *          - 3: Throttle curve with idle (turbine startup)
     *          - 4: Internal governor (ArduPilot maintains RPM)
     *          - 5: Autorotation (motor off)
     *          
     *          **H_RSC_SETPOINT** (Rotor Speed Target):
     *          - Target rotor speed as percentage (0-100%)
     *          - Typical: 70-90% (leaves headroom for governor)
     *          - Too low: Insufficient rotor inertia, poor autorotation
     *          - Too high: Reduced governor authority, increased stress
     *          
     *          **H_SV_MAN** (Manual Servo Mode):
     *          - Servo control mode for ground setup/testing
     *          - 0: Automated (normal flight)
     *          - 1: Manual passthrough (direct pilot control)
     *          - 2-6: Test modes (max, center, min, oscillate)
     *          - Only use during ground setup with rotor secured
     *          
     *          Parameter naming convention: H_ = Helicopter, COL = Collective, CYC = Cyclic,
     *          RSC = Rotor Speed Control, SV = Servo
     */
    AP_Int16        _cyclic_max;                // Maximum cyclic angle of the swash plate in centi-degrees
    AP_Int16        _collective_min;            // Lowest possible servo position for the swashplate
    AP_Int16        _collective_max;            // Highest possible servo position for the swashplate
    AP_Int8         _servo_mode;                // Pass radio inputs directly to servos during set-up through mission planner
    AP_Int8         _servo_test;                // sets number of cycles to test servo movement on bootup
    AP_Float        _collective_hover;          // estimated collective required to hover throttle in the range 0 ~ 1
    AP_Int8         _collective_hover_learn;    // enable/disabled hover collective learning
    AP_Int8         _heli_options;              // bitmask for optional features
    AP_Float        _collective_zero_thrust_deg;// Zero thrust blade collective pitch in degrees
    AP_Float        _collective_land_min_deg;   // Minimum Landed collective blade pitch in degrees for non-manual collective modes (i.e. modes that use altitude hold)
    AP_Float        _collective_max_deg;        // Maximum collective blade pitch angle in deg that corresponds to the PWM set for maximum collective pitch (H_COL_MAX)
    AP_Float        _collective_min_deg;        // Minimum collective blade pitch angle in deg that corresponds to the PWM set for minimum collective pitch (H_COL_MIN)
    /** @} */

    // internal variables
    float           _collective_zero_thrust_pct;      // collective zero thrutst parameter value converted to 0 ~ 1 range
    float           _collective_land_min_pct;      // collective land min parameter value converted to 0 ~ 1 range
    uint8_t         _servo_test_cycle_counter = 0;   // number of test cycles left to run after bootup

    motor_frame_type _frame_type;
    motor_frame_class _frame_class;
};
