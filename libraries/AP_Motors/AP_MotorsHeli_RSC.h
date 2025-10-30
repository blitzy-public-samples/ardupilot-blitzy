/**
 * @file AP_MotorsHeli_RSC.h
 * @brief Helicopter Rotor Speed Control (RSC) system implementation
 * 
 * @details This file implements the Rotor Speed Control system for ArduPilot helicopters,
 *          providing comprehensive control over main rotor speed through multiple control modes.
 *          The RSC manages rotor startup/shutdown sequences, maintains desired rotor speeds,
 *          and coordinates with the autorotation system for emergency procedures.
 * 
 *          Key responsibilities:
 *          - Rotor speed control via multiple modes (passthrough, setpoint, throttle curve, governor)
 *          - Safe startup and shutdown ramp timing
 *          - Runup state machine management (STOP → IDLE → ACTIVE)
 *          - Integration with ESC/servo outputs
 *          - Governor functionality for constant rotor speed under varying loads
 *          - Throttle curve interpolation using collective input
 *          - Autorotation coordination for engine-off landings
 *          - Critical speed monitoring for flight safety
 * 
 *          The RSC integrates with helicopter motor classes (Single, Dual, Quad) and provides
 *          thread-safe state management for rotor control throughout all flight phases.
 * 
 * @note Rotor speed is safety-critical - improper configuration can lead to loss of control
 * @warning All timing parameters (ramp_time, runup_time) must be configured appropriately
 *          for the specific helicopter and ESC/engine combination
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Logger/AP_Logger_config.h>
#include <AC_Autorotation/RSC_Autorotation.h>

/**
 * @enum RotorControlMode
 * @brief Rotor Speed Control operating modes
 * 
 * @details Defines the available control modes for managing helicopter rotor speed.
 *          Each mode provides different levels of automation and pilot control over
 *          the main rotor speed, from direct passthrough to fully automated governor control.
 */
enum RotorControlMode {
    /**
     * @brief RSC system disabled - no rotor speed control active
     * @details Motor output is disabled. Use for ground maintenance or when RSC is not required.
     */
    ROTOR_CONTROL_MODE_DISABLED = 0,
    
    /**
     * @brief Direct passthrough from pilot throttle input to ESC/servo output
     * @details Pilot has direct manual control of rotor speed via throttle stick.
     *          No automation or speed regulation is applied. Suitable for experienced
     *          pilots or specific flight scenarios requiring direct control.
     */
    ROTOR_CONTROL_MODE_PASSTHROUGH,
    
    /**
     * @brief Fixed setpoint rotor speed control
     * @details Maintains a constant rotor speed setpoint defined by RSC_SETPOINT parameter.
     *          Output ramps smoothly to the setpoint value without load compensation.
     *          Suitable for electric helicopters with stable motor characteristics.
     */
    ROTOR_CONTROL_MODE_SETPOINT,
    
    /**
     * @brief Collective-based throttle curve control
     * @details Uses a 5-point spline interpolation throttle curve based on collective pitch input.
     *          Throttle output varies with collective to maintain rotor speed under changing loads.
     *          Curve defined by H_RSC_THRCRV_0 through H_RSC_THRCRV_100 parameters.
     *          Suitable for nitro/gas engines and applications requiring load-based throttle management.
     */
    ROTOR_CONTROL_MODE_THROTTLECURVE,
    
    /**
     * @brief Automatic throttle management with closed-loop governor
     * @details Actively maintains constant rotor RPM using feedback from rotor speed sensor.
     *          Automatically adjusts throttle to compensate for load changes and maintain
     *          target RPM defined by H_RSC_GOV_RPM parameter. Provides best rotor speed
     *          stability under varying flight conditions.
     * @note Requires RPM sensor and proper H_RSC_GOV_RPM parameter configuration
     * @warning Governor fault detection will trigger if RPM sensor signal is lost
     */
    ROTOR_CONTROL_MODE_AUTOTHROTTLE
};

/**
 * @class AP_MotorsHeli_RSC
 * @brief Helicopter Rotor Speed Control management class
 * 
 * @details Manages all aspects of helicopter main rotor speed control including startup/shutdown
 *          sequences, multiple control modes, governor functionality, and integration with
 *          autorotation systems. This class provides a state machine-based approach to safely
 *          managing rotor speed transitions and maintaining stable rotor operation.
 * 
 *          Key Features:
 *          - Multiple control modes: Disabled, Passthrough, Setpoint, Throttle Curve, Governor
 *          - Safe state machine transitions: STOP → IDLE → ACTIVE
 *          - Configurable ramp timing for smooth ESC/servo output transitions
 *          - Runup timing to track physical rotor acceleration
 *          - Critical speed monitoring for flight safety
 *          - Governor with torque compensation for load management
 *          - 5-point spline-based throttle curve interpolation
 *          - Autorotation detection and coordination
 *          - Turbine-specific startup sequence support
 * 
 *          Integration Points:
 *          - Works with AP_MotorsHeli_Single, AP_MotorsHeli_Dual, AP_MotorsHeli_Quad
 *          - Receives collective input for throttle curve calculation
 *          - Outputs to ESC/servo via SRV_Channel
 *          - Integrates with RSC_Autorotation for emergency procedures
 *          - Logs RSC state for flight analysis
 * 
 *          State Machine:
 *          STOP: Rotor stopped, output at zero or idle
 *          IDLE: Output at idle setting, rotor beginning to spin up
 *          ACTIVE: Full rotor speed control active, flight-ready
 * 
 *          Thread Safety:
 *          - Called from main thread at scheduler rate (typically 400Hz)
 *          - State transitions managed internally with appropriate guards
 *          - Output method must be called consistently each control loop
 * 
 * @note Timing parameters critical: runup_time must be longer than ramp_time to ensure
 *       physical rotor reaches speed before engaging aggressive maneuvers
 * @warning Incorrect configuration can lead to rotor overspeed, underspeed, or oscillations
 * @warning Changes to control mode during flight should be avoided except in emergencies
 */
class AP_MotorsHeli_RSC {
public:
    friend class AP_MotorsHeli_Single;
    friend class AP_MotorsHeli_Dual;
    friend class AP_MotorsHeli_Quad;

    AP_MotorsHeli_RSC(SRV_Channel::Function aux_fn,
                      uint8_t default_channel,
                      uint8_t inst) :
        _instance(inst),
        _aux_fn(aux_fn),
        _default_channel(default_channel)
    {
        AP_Param::setup_object_defaults(this, var_info);
    };

    /**
     * @enum RotorControlState
     * @brief RSC state machine states for rotor startup and operation
     * 
     * @details Defines the operational states of the Rotor Speed Control system.
     *          State transitions follow a strict sequence to ensure safe rotor operation:
     *          STOP → IDLE → ACTIVE (normal startup)
     *          ACTIVE → IDLE → STOP (normal shutdown)
     */
    enum class RotorControlState {
        /**
         * @brief Rotor stopped or stopping
         * @details ESC/servo output at zero or minimal. Rotor is not spinning or decelerating.
         *          Safe for ground operations. No flight operations permitted.
         */
        STOP = 0,
        
        /**
         * @brief Rotor at idle speed, beginning spool-up
         * @details ESC/servo output at idle setting, rotor beginning to accelerate.
         *          Transition state during startup and shutdown sequences.
         *          Limited flight operations - not yet at flight-ready rotor speed.
         */
        IDLE,
        
        /**
         * @brief Rotor at full speed, flight-ready
         * @details ESC/servo output actively controlled per selected mode. Rotor at or
         *          approaching target speed. Full flight operations permitted.
         *          Runup complete flag set when rotor reaches critical speed threshold.
         */
        ACTIVE
    };

    /**
     * @brief Initialize servo output channel on system start-up
     * 
     * @details Configures the SRV_Channel for RSC output, sets up default channel mapping,
     *          and initializes output ranges. Must be called during motor initialization
     *          before any output operations.
     * 
     * @note Called once during AP_MotorsHeli initialization
     */
    void        init_servo();

    /**
     * @brief Set the active rotor speed control mode
     * 
     * @details Changes the RSC operating mode, affecting how rotor speed is controlled.
     *          Mode changes should typically occur when rotor is stopped or at idle.
     *          Different modes provide varying levels of automation and pilot control.
     * 
     * @param[in] mode Desired RotorControlMode (DISABLED, PASSTHROUGH, SETPOINT, 
     *                 THROTTLECURVE, or AUTOTHROTTLE)
     * 
     * @warning Changing modes during active flight can cause abrupt rotor speed changes
     *          and should be avoided except in emergency situations
     * @note Mode changes take effect immediately on next output() call
     * 
     * @see RotorControlMode for detailed mode descriptions
     */
    void        set_control_mode(RotorControlMode mode) { _control_mode = mode; }

    /**
     * @brief Reset the RSC mode parameter to match current active control mode
     * 
     * @details Synchronizes the persistent _rsc_mode parameter with the currently active
     *          _control_mode. Used to update saved parameters after runtime mode changes.
     * 
     * @note Typically called after temporary mode overrides to persist the active mode
     */
    void        reset_rsc_mode_param() { _rsc_mode.set((uint8_t)_control_mode); }

    /**
     * @brief Get the currently active rotor speed control mode
     * 
     * @return uint8_t Current RotorControlMode value (cast to uint8_t)
     * 
     * @note Return value can be cast back to RotorControlMode enum
     */
    uint8_t     get_control_mode() const { return _control_mode; }

    /**
     * @brief Set the critical rotor speed threshold for flight safety
     * 
     * @details Configures the minimum rotor speed (as percentage) below which flight
     *          operations should not be attempted. Used to set the _critical_speed parameter.
     *          This threshold determines when rotor_speed_above_critical() returns true.
     * 
     * @param[in] critical_speed Critical speed threshold as percentage (0-100)
     * 
     * @note Typical values: 70-90% depending on helicopter characteristics
     * @warning Setting too low may permit flight at unsafe rotor speeds
     */
    void        set_critical_speed(float critical_speed) { _critical_speed.set(critical_speed); }

    /**
     * @brief Get the current desired rotor speed
     * 
     * @details Returns the target rotor speed that the RSC is attempting to achieve.
     *          Value represents normalized speed (0.0 = stopped, 1.0 = full speed).
     * 
     * @return float Desired rotor speed in range 0.0-1.0
     * 
     * @note Actual rotor speed may differ from desired during transients or faults
     */
    float       get_desired_speed() const { return _desired_speed; }

    /**
     * @brief Set the desired target rotor speed
     * 
     * @details Configures the target rotor speed for RSC control. The controller will
     *          ramp output to achieve this speed according to configured ramp and runup times.
     *          Input is normalized where 0.0 = stopped and 1.0 = full speed.
     * 
     * @param[in] desired_speed Target rotor speed, normalized range 0.0-1.0
     * 
     * @note Input is NOT automatically constrained - caller must ensure valid range
     * @note In SETPOINT mode, this overrides the RSC_SETPOINT parameter value
     */
    void        set_desired_speed(float desired_speed) { _desired_speed = desired_speed; }

    /**
     * @brief Set the governor output value for rotor speed control
     * 
     * @details Provides the governor-calculated output to the RSC system in AUTOTHROTTLE mode.
     *          The governor uses closed-loop feedback from RPM sensor to compute this value,
     *          which adjusts throttle to maintain constant rotor speed under varying loads.
     * 
     * @param[in] governor_output Computed governor output value (typically 0.0-1.0 range)
     * 
     * @note Only effective when control mode is ROTOR_CONTROL_MODE_AUTOTHROTTLE
     * @note Governor output is internally managed by autothrottle_run() in most cases
     */
    void        set_governor_output(float governor_output) {_governor_output = governor_output; }
    
    /**
     * @brief Reset the governor to initial state
     * 
     * @details Clears governor engagement status, fault flags, and integrator states.
     *          Used during mode changes, rotor startup, or to recover from governor faults.
     *          Governor will re-engage automatically when conditions are appropriate.
     * 
     * @note Causes temporary loss of governor control until re-engagement
     * @note Called automatically during state transitions
     */
    void        governor_reset();
    
    /**
     * @brief Get the current RSC control output value
     * 
     * @details Returns the final computed control output being sent to ESC/servo after
     *          all control logic, ramping, and limiting. Value is normalized where
     *          0.0 = minimum output and 1.0 = maximum output.
     * 
     * @return float Current control output in range 0.0-1.0
     * 
     * @note This is the actual output value after mode-specific calculations and ramp limiting
     */
    float       get_control_output() const { return _control_output; }
    
    /**
     * @brief Set the idle output level for RSC
     * 
     * @details Configures the ESC/servo output percentage when rotor is at idle state.
     *          This is the minimum output to keep rotor spinning at low speed.
     *          Sets the _idle_output parameter.
     * 
     * @param[in] idle_output Idle output level as percentage (typically 10-30)
     * 
     * @note Too low: rotor may stop during idle. Too high: excess wear and fuel consumption
     */
    void        set_idle_output(float idle_output) { _idle_output.set(idle_output); }
    
    /**
     * @brief Execute autothrottle/governor control algorithm
     * 
     * @details Runs the closed-loop governor algorithm using RPM feedback to maintain
     *          constant rotor speed. Computes torque compensation, handles governor
     *          engagement/disengagement, and manages fault detection for RPM sensor issues.
     *          Updates _governor_output with computed throttle adjustment.
     * 
     * @note Called each control loop cycle when in AUTOTHROTTLE mode
     * @note Requires valid RPM sensor feedback (H_RSC_GOV_RPM parameter configured)
     * @warning Governor faults will be triggered if RPM signal is lost for extended period
     * 
     * @see set_governor_output() for how computed output is applied
     */
    void        autothrottle_run();
    
    /**
     * @brief Calculate and configure the throttle curve for collective-based control
     * 
     * @details Computes spline polynomial coefficients for 5-point throttle curve interpolation.
     *          Uses H_RSC_THRCRV_0, _25, _50, _75, _100 parameters as control points.
     *          The resulting curve maps collective pitch input (0-1) to throttle output (0-1).
     * 
     * @note Throttle curve interpolation uses cubic spline for smooth transitions
     * @note Only relevant when control mode is ROTOR_CONTROL_MODE_THROTTLECURVE
     * @note Curve must be configured with monotonically increasing values for proper operation
     */
    void        set_throttle_curve();

    /**
     * @brief Set the ESC output ramp time
     * 
     * @details Configures the time in seconds for ESC/servo output to ramp from zero/idle
     *          to full setpoint. This controls how quickly the output signal changes,
     *          protecting ESC and motor from abrupt throttle changes. Sets _ramp_time parameter.
     * 
     * @param[in] ramp_time Ramp duration in seconds (typically 1-8 seconds)
     * 
     * @note Ramp time must be SHORTER than runup time for proper operation
     * @note Too short: potential ESC damage or motor stress. Too long: sluggish response
     * @warning Ramp time affects only output signal, not physical rotor acceleration
     */
    void        set_ramp_time(int8_t ramp_time) { _ramp_time.set(ramp_time); }
    
    /**
     * @brief Set the physical rotor runup time
     * 
     * @details Configures the time in seconds for physical rotor to accelerate from stopped
     *          to full flight speed. This accounts for rotor inertia and motor torque characteristics.
     *          Used to set _runup_complete flag timing. Sets _runup_time parameter.
     * 
     * @param[in] runup_time Runup duration in seconds (typically 5-15 seconds)
     * 
     * @note Runup time MUST be LONGER than ramp time to ensure physical rotor reaches speed
     * @note Value should be empirically determined for specific helicopter configuration
     * @warning Too short: flight attempted before rotor at safe speed. Too long: excessive startup delay
     */
    void        set_runup_time(int8_t runup_time) { _runup_time.set(runup_time); }
    
    /**
     * @brief Check if rotor runup sequence is complete
     * 
     * @details Returns true when rotor has been spinning long enough (runup_time elapsed)
     *          and has reached critical speed threshold. Indicates rotor is flight-ready.
     *          Flight controller uses this to enable aggressive maneuvers and full authority.
     * 
     * @return bool True if runup complete and rotor at flight-ready speed, false otherwise
     * 
     * @note Flag is set based on runup_time elapsed AND rotor speed above critical threshold
     * @warning Flying before runup complete can result in loss of control due to insufficient rotor speed
     */
    bool        is_runup_complete() const { return _runup_complete; }

    /**
     * @brief Check if rotor spooldown sequence is complete
     * 
     * @details Returns true when rotor has decelerated to stopped/idle condition.
     *          Indicates safe to disarm or transition to ground operations.
     * 
     * @return bool True if spooldown complete and rotor at safe idle/stopped state
     * 
     * @note Used to prevent disarm until rotor is safely stopped
     */
    bool        is_spooldown_complete() const { return _spooldown_complete; }

    /**
     * @brief Set collective pitch input for throttle curve calculation
     * 
     * @details Provides current collective pitch position to RSC for throttle curve interpolation.
     *          In THROTTLECURVE mode, this input is used to look up throttle output from the
     *          configured 5-point curve. Input is normalized collective position.
     * 
     * @param[in] collective Collective pitch input, normalized range 0.0-1.0
     *                       (0.0 = full negative/low pitch, 1.0 = full positive/high pitch)
     * 
     * @note Only affects output in ROTOR_CONTROL_MODE_THROTTLECURVE
     * @note Input is NOT constrained - caller must ensure valid range
     */
    void        set_collective(float collective) { _collective_in = collective; }

    /**
     * @brief Check if helicopter is currently in autorotation or autorotation bailout
     * 
     * @details Returns true if autorotation is active (unpowered descent) or if actively
     *          bailing out from autorotation (re-engaging power). Used to coordinate RSC
     *          behavior during engine-off emergency landings and subsequent power recovery.
     * 
     * @return bool True if autorotating or in bailout phase, false during normal powered flight
     * 
     * @note Autorotation detection coordinates with RSC_Autorotation object
     * @see RSC_Autorotation for autorotation state management
     */
    bool        in_autorotation(void) const;

    /**
     * @brief Initialize turbine engine startup sequence
     * 
     * @details Triggers turbine-specific startup sequence with appropriate delays and throttle
     *          progression for turbine engines. Sets internal _turbine_start flag to coordinate
     *          startup state machine for turbine helicopters.
     * 
     * @param[in] turbine_start True to initiate turbine startup sequence, false otherwise
     * 
     * @note Only relevant for turbine-powered helicopters with turbine-specific startup requirements
     * @note Turbine startup typically requires extended idle period before engaging governor
     */
    void        set_turbine_start(bool turbine_start) {_turbine_start = turbine_start; }

    /**
     * @brief Main output update method - computes and sends RSC output to ESC/Servo
     * 
     * @details Primary method called each control cycle to update RSC output. Implements the
     *          complete RSC state machine and control logic:
     *          - Processes current state (STOP, IDLE, ACTIVE)
     *          - Applies selected control mode algorithm (passthrough, setpoint, curve, governor)
     *          - Updates ramp and runup timing
     *          - Manages state transitions
     *          - Writes final output to servo channel via write_rsc()
     *          - Updates runup_complete and spooldown_complete flags
     * 
     * @param[in] state Desired RotorControlState (STOP, IDLE, or ACTIVE)
     * 
     * @note Must be called every control loop cycle (typically 400Hz) for proper operation
     * @note State parameter is a request - actual state transitions follow safe sequences
     * @warning This method directly controls ESC/servo output affecting rotor speed
     * @warning Failure to call this method results in no RSC output and rotor stoppage
     * @warning State transitions affect rotor speed - ensure proper sequencing during flight
     * 
     * @see write_rsc() for actual servo output mechanism
     * @see update_rotor_ramp() for output ramping logic
     * @see update_rotor_runup() for runup timing logic
     */
    void        output(RotorControlState state);

    /**
     * @brief Get bitmask of output channels used by RSC
     * 
     * @details Returns a bitmask indicating which servo output channels the RSC system
     *          is currently controlling. Used for output coordination and conflict detection
     *          with other systems that may use servo outputs.
     * 
     * @return uint32_t Bitmask of output channels (bit N set = channel N in use)
     * 
     * @note Typically returns mask with single bit set for RSC channel
     */
    uint32_t    get_output_mask() const;

    /**
     * @brief Check if rotor speed is above critical threshold for flight operations
     * 
     * @details Returns true when rotor runup output has reached the configured critical
     *          speed percentage. This is a key safety check - flight control authority
     *          and aggressive maneuvers should only be permitted when this returns true.
     *          Uses _rotor_runup_output (0.0-1.0) vs critical_speed parameter (0-100%).
     * 
     * @return bool True if rotor at or above critical speed for safe flight, false otherwise
     * 
     * @warning Critical for flight safety - do not attempt aggressive maneuvers when false
     * @warning Insufficient rotor speed leads to loss of control authority
     * @note Critical speed typically configured to 70-90% depending on helicopter characteristics
     * 
     * @see set_critical_speed() for configuring threshold
     * @see is_runup_complete() for full flight-ready status (includes timing)
     */
    bool        rotor_speed_above_critical(void) const { return _rotor_runup_output >= get_critical_speed(); }

#if HAL_LOGGING_ENABLED
    // RSC logging
    void write_log(void) const;
#endif

    RSC_Autorotation autorotation;

    /**
     * @brief Parameter information table for RSC configuration
     * @details Defines all configurable parameters for the RSC system including control mode,
     *          timing settings, speed setpoints, and governor tuning values.
     */
    static const struct AP_Param::GroupInfo var_info[];

    // Public configuration parameters
    
    /**
     * @brief Target rotor speed setpoint (H_RSC_SETPOINT parameter)
     * @details Rotor speed percentage when RSC mode is SETPOINT. Defines constant speed target.
     * @note Range: 0-100 (percentage), typical value: 70-80
     * @note Only active in ROTOR_CONTROL_MODE_SETPOINT
     */
    AP_Int16        _rsc_setpoint;
    
    /**
     * @brief Active RSC control mode selection (H_RSC_MODE parameter)
     * @details Selects which rotor speed control mode is active. Maps to RotorControlMode enum.
     * @note 0=Disabled, 1=Passthrough, 2=Setpoint, 3=Throttle Curve, 4=Autothrottle/Governor
     * @see RotorControlMode for mode descriptions
     */
    AP_Int8         _rsc_mode;
    
    /**
     * @brief ESC output ramp time in seconds (H_RSC_RAMP_TIME parameter)
     * @details Time for ESC/servo output signal to ramp from zero/idle to full setpoint.
     *          Controls rate of change of output signal to protect ESC and motor.
     * @note Range: 0-60 seconds, typical value: 1-8 seconds
     * @warning Must be SHORTER than _runup_time for proper rotor speed management
     */
    AP_Int8         _ramp_time;
    
    /**
     * @brief Physical rotor runup time in seconds (H_RSC_RUNUP_TIME parameter)
     * @details Time for physical rotor to accelerate from stopped to full flight speed.
     *          Accounts for rotor inertia and system dynamics. Determines when flight-ready.
     * @note Range: 0-60 seconds, typical value: 5-15 seconds depending on rotor size
     * @warning Must be LONGER than _ramp_time to ensure rotor physically reaches speed
     * @note Should be empirically determined for specific helicopter configuration
     */
    AP_Int8         _runup_time;
    
    /**
     * @brief Critical rotor speed threshold percentage (H_RSC_CRITICAL parameter)
     * @details Minimum rotor speed below which flight operations should not be attempted.
     *          Used as safety threshold for enabling aggressive maneuvers and full control authority.
     * @note Range: 0-100 (percentage), typical value: 70-90
     * @warning Setting too low permits flight at unsafe rotor speeds
     * @see rotor_speed_above_critical() for usage in flight safety checks
     */
    AP_Int16        _critical_speed;
    
    /**
     * @brief Idle output level percentage (H_RSC_IDLE_OUTPUT parameter)
     * @details ESC/servo output percentage when rotor is at idle state. Minimum output to
     *          keep rotor spinning at low speed during ground operations and state transitions.
     * @note Range: 0-100 (percentage), typical value: 10-30
     * @note Too low: rotor may stop during idle. Too high: excess wear and fuel consumption
     */
    AP_Int16        _idle_output;

private:
    uint64_t        _last_update_us;
    const uint8_t   _instance;

    // channel setup for aux function
    const SRV_Channel::Function _aux_fn;
    const uint8_t _default_channel;

    // internal variables
    RotorControlMode _control_mode = ROTOR_CONTROL_MODE_DISABLED;   // motor control mode, Passthrough or Setpoint
    float           _desired_speed;               // latest desired rotor speed from pilot
    float           _control_output;              // latest logic controlled output
    float           _rotor_ramp_output;           // scalar used to ramp rotor speed between _rsc_idle_output and full speed (0.0-1.0f)
    float           _rotor_runup_output;          // scalar used to store status of rotor run-up time (0.0-1.0f)
    bool            _runup_complete;              // flag for determining if runup is complete
    float           _thrcrv_poly[4][4];           // spline polynomials for throttle curve interpolation
    float           _collective_in;               // collective in for throttle curve calculation, range 0-1.0f
    float           _rotor_rpm;                   // rotor rpm from speed sensor for governor
    bool            _turbine_start;               // initiates starting sequence
    bool            _starting;                    // tracks if starting sequence has been used
    float           _governor_output;             // governor output for rotor speed control
    bool            _governor_engage;             // RSC governor status flag
    bool            _autothrottle;                // autothrottle status flag
    bool            _governor_fault;              // governor fault status flag
    bool            _spooldown_complete;          // flag for determining if spooldown is complete
    float           _fast_idle_timer;             // cooldown timer variable
    uint8_t         _governor_fault_count;        // variable for tracking governor speed sensor faults
    float           _governor_torque_reference;   // governor reference for load calculations
    float           _idle_throttle;               // current idle throttle setting

    RotorControlState _rsc_state;

    // update_rotor_ramp - slews rotor output scalar between 0 and 1, outputs float scalar to _rotor_ramp_output
    void            update_rotor_ramp(float rotor_ramp_input, float dt);

    // update_rotor_runup - function to slew rotor runup scalar, outputs float scalar to _rotor_runup_ouptut
    void            update_rotor_runup(float dt);

    // write_rsc - outputs pwm onto output rsc channel. servo_out parameter is of the range 0 ~ 1
    void            write_rsc(float servo_out);

    /**
     * @brief Calculate throttle output from throttle curve and collective input
     * @details Uses cubic spline interpolation of 5-point throttle curve to compute throttle
     *          output based on current collective pitch input. Interpolation provides smooth
     *          throttle transitions across collective range.
     * @param[in] collective_in Collective pitch input, normalized 0.0-1.0
     * @return float Computed throttle output, normalized 0.0-1.0
     * @note Only used in ROTOR_CONTROL_MODE_THROTTLECURVE
     */
    float           calculate_throttlecurve(float collective_in);

    // Private configuration parameters
    
    /**
     * @brief Throttle output slew rate limit (H_RSC_SLEWRATE parameter)
     * @details Maximum rate of change for throttle output in percentage per second.
     *          Limits how fast throttle can change to prevent abrupt load changes.
     * @note Range: 0-500 (%/s), typical value: 100-200 %/s
     */
    AP_Int16        _power_slewrate;
    
    /**
     * @brief Throttle curve control points (H_RSC_THRCRV_0/25/50/75/100 parameters)
     * @details Five-point throttle curve defining throttle output at 0%, 25%, 50%, 75%,
     *          and 100% collective. Cubic spline interpolation used between points.
     * @note Array index: [0]=0%, [1]=25%, [2]=50%, [3]=75%, [4]=100% collective
     * @note Values should be monotonically increasing for proper operation
     * @note Only used in ROTOR_CONTROL_MODE_THROTTLECURVE
     */
    AP_Int16        _thrcrv[5];
    
    /**
     * @brief Governor target RPM reference (H_RSC_GOV_RPM parameter)
     * @details Target rotor RPM for closed-loop governor control. Governor attempts to
     *          maintain this RPM under varying load conditions.
     * @note Typical range: 1000-3500 RPM depending on helicopter head speed
     * @note Requires RPM sensor providing valid feedback
     * @note Only used in ROTOR_CONTROL_MODE_AUTOTHROTTLE
     */
    AP_Int16        _governor_rpm;
    
    /**
     * @brief Governor torque rise gain (H_RSC_GOV_TORQUE parameter)
     * @details Gain for governor torque compensation. Higher values provide more aggressive
     *          response to load-induced RPM droops but may cause oscillations.
     * @note Range: 0-100, typical value: 20-40
     * @note Scaled by 0.01 in get_governor_torque()
     */
    AP_Float        _governor_torque;
    
    /**
     * @brief Governor torque compensator (H_RSC_GOV_COMP parameter)
     * @details Torque compensator coefficient for governor feed-forward control.
     *          Anticipates load changes based on collective input.
     * @note Range: 0-1000000, scaled by 0.000001 in get_governor_compensator()
     */
    AP_Float        _governor_compensator;
    
    /**
     * @brief Governor droop response gain (H_RSC_GOV_DROOP parameter)
     * @details Gain controlling governor response to RPM droop under load.
     *          Adjusts how aggressively governor corrects for speed loss.
     * @note Range: 0-100, typical value: 10-30
     */
    AP_Float        _governor_droop_response;
    
    /**
     * @brief Governor feedforward gain (H_RSC_GOV_FF parameter)
     * @details Feedforward gain for governor control. Adds predictive component to
     *          governor output based on collective input rate of change.
     * @note Range: 0-1, typical value: 0.1-0.5
     */
    AP_Float        _governor_ff;
    
    /**
     * @brief Governor operational RPM range (H_RSC_GOV_RANGE parameter)
     * @details RPM deviation range (+/-) from reference where governor remains engaged.
     *          Governor disengages if RPM exceeds this range, indicating fault or overspeed.
     * @note Range: 0-500 RPM, typical value: 100-200 RPM
     * @note Too narrow: frequent governor faults. Too wide: insufficient fault protection
     */
    AP_Float        _governor_range;
    
    /**
     * @brief Engine cooldown time in seconds (H_RSC_COOLDOWN parameter)
     * @details Time to maintain fast idle after flight for engine cooling before full shutdown.
     *          Particularly important for nitro/gas engines requiring cooldown period.
     * @note Range: 0-300 seconds, typical value: 10-60 seconds
     * @note Only applies when transitioning from ACTIVE to STOP state
     */
    AP_Int16        _cooldown_time;

    // parameter accessors to allow conversions
    float       get_critical_speed() const { return _critical_speed * 0.01; }
    float       get_idle_output() const { return _idle_output * 0.01; }
    float       get_governor_torque() const { return _governor_torque * 0.01; }
    float       get_governor_compensator() const { return _governor_compensator * 0.000001; }

};
