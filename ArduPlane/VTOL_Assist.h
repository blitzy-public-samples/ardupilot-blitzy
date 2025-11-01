/**
 * @file VTOL_Assist.h
 * @brief VTOL assistance system for fixed-wing flight modes in quadplane configurations
 * 
 * @details This file defines the VTOL_Assist class which provides automatic multicopter
 *          assistance during fixed-wing flight when the aircraft encounters challenging
 *          flight conditions. The system monitors airspeed, attitude angles, and altitude
 *          to determine when VTOL motor assistance is needed to maintain controlled flight.
 *          
 *          Key responsibilities:
 *          - Monitor flight conditions (speed, angle, altitude) against configured thresholds
 *          - Implement hysteresis-based triggering to prevent oscillations
 *          - Manage state transitions between assist modes
 *          - Provide spin recovery assistance using differential thrust
 *          - Support configuration options for different operational scenarios
 * 
 * @note This class is tightly integrated with the QuadPlane flight controller
 * @warning Assistance thresholds must be carefully tuned to avoid unwanted activation
 * 
 * Source: ArduPlane/VTOL_Assist.h
 */

#pragma once

// VTOL assistance in a forward flight mode

class QuadPlane;

/**
 * @class VTOL_Assist
 * @brief Automatic VTOL motor assistance during fixed-wing flight modes
 * 
 * @details The VTOL_Assist class monitors flight conditions and automatically engages
 *          quadplane VTOL motors when the aircraft is at risk of losing controlled flight.
 *          This provides a safety mechanism for quadplanes operating in fixed-wing modes.
 *          
 *          Assistance can be triggered by multiple conditions:
 *          - Low airspeed (below configured threshold)
 *          - Excessive attitude angles (pitch/roll beyond limits)
 *          - Altitude loss during climb attempt
 *          - Pilot force-enable command
 *          
 *          The system uses hysteresis timers to prevent rapid oscillations between
 *          assisted and unassisted flight. Each trigger condition has configurable
 *          thresholds and timing parameters.
 *          
 *          State Machine:
 *          - ASSIST_DISABLED: All automatic assistance disabled by pilot
 *          - ASSIST_ENABLED: Normal operation, assistance provided when needed
 *          - FORCE_ENABLED: Pilot forces assistance on regardless of conditions
 *          
 *          The class also provides specialized spin recovery assistance using
 *          differential motor thrust and flight control surface coordination.
 * 
 * @note Assistance is only available when in fixed-wing flight modes
 * @warning Improper configuration can lead to unwanted motor activation or delayed assistance
 */
class VTOL_Assist {
public:
    /**
     * @brief Construct a new VTOL_Assist object
     * 
     * @param[in] _quadplane Reference to parent QuadPlane object for accessing vehicle state
     * 
     * @details Initializes the VTOL assistance system with a reference to the quadplane
     *          flight controller. The reference is stored for accessing vehicle state,
     *          sensors, and control outputs during assistance operations.
     */
    VTOL_Assist(QuadPlane& _quadplane):quadplane(_quadplane) {};

    /**
     * @brief Check if VTOL assistance should be provided based on current flight conditions
     * 
     * @param[in] aspeed Current airspeed in m/s
     * @param[in] have_airspeed True if airspeed reading is valid and available
     * 
     * @return true if VTOL motors should provide assistance, false otherwise
     * 
     * @details This is the main decision function that evaluates all assistance criteria:
     *          - Speed assistance: Triggered when airspeed drops below configured threshold
     *          - Angle assistance: Triggered when pitch or roll exceeds configured limits
     *          - Altitude assistance: Triggered when altitude is lost during climb attempt
     *          - Force assistance: Triggered by pilot command override
     *          
     *          The function implements hysteresis timing to prevent rapid cycling. Each
     *          condition must persist for the configured delay period before assistance
     *          is activated, and must be clear for a period before deactivation.
     *          
     *          State machine logic:
     *          - ASSIST_DISABLED: Always returns false regardless of conditions
     *          - ASSIST_ENABLED: Returns true if any trigger condition is met
     *          - FORCE_ENABLED: Always returns true regardless of conditions
     * 
     * @note Called at main loop rate during fixed-wing flight modes
     * @warning Requires valid vehicle state from quadplane reference
     * 
     * @see reset()
     * @see set_state()
     */
    bool should_assist(float aspeed, bool have_airspeed);

    /**
     * @brief Reset all assistance state when assistance is no longer needed
     * 
     * @details Clears all internal state variables and hysteresis timers. This should
     *          be called when transitioning out of fixed-wing modes, when assistance
     *          conditions are no longer present, or when manually resetting the system.
     *          
     *          Resets:
     *          - Angle error hysteresis timer
     *          - Altitude error hysteresis timer
     *          - Force assist flag
     *          - Speed assist flag
     * 
     * @note Does not reset the pilot-commanded state (ASSIST_DISABLED/ENABLED/FORCE_ENABLED)
     * 
     * @see should_assist()
     */
    void reset();

    /**
     * @brief Airspeed threshold below which VTOL assistance is activated
     * 
     * @details When airspeed drops below this value (in m/s), the VTOL motors will
     *          engage to prevent stall or loss of control. This provides a safety
     *          margin above the aircraft's actual stall speed.
     *          
     *          Setting this to 0 disables speed-based assistance. Typical values are
     *          2-5 m/s above stall speed depending on aircraft characteristics.
     * 
     * @note Units: m/s (meters per second)
     * @warning Must be set above stall speed but below normal cruise speed
     */
    AP_Float speed;

    /**
     * @brief Maximum attitude angle error before VTOL assistance is activated
     * 
     * @details When pitch or roll angle exceeds this threshold (in degrees), the VTOL
     *          motors engage to help recover controlled flight. This prevents loss of
     *          control in extreme attitude situations or during aggressive maneuvering.
     *          
     *          Setting this to 0 disables angle-based assistance. Typical values are
     *          30-60 degrees depending on the aircraft's handling characteristics and
     *          pilot comfort level.
     * 
     * @note Units: degrees (angle magnitude, not rate)
     * @warning Too low a value may cause unwanted activation during normal maneuvering
     */
    AP_Int8 angle;

    /**
     * @brief Altitude loss threshold that triggers VTOL assistance during climbs
     * 
     * @details When the aircraft is commanded to climb but instead descends by more
     *          than this amount (in meters), VTOL assistance engages. This helps
     *          recover from situations where fixed-wing thrust is insufficient.
     *          
     *          Setting this to 0 disables altitude-based assistance. Typical values
     *          are 5-20 meters depending on desired responsiveness.
     * 
     * @note Units: meters (altitude loss relative to commanded climb)
     * @warning Requires valid altitude sensor readings
     */
    AP_Int16 alt;

    /**
     * @brief Time delay before assistance triggers to prevent false activation
     * 
     * @details Hysteresis delay time in seconds that a trigger condition must persist
     *          before assistance is activated. This prevents rapid cycling or false
     *          triggers from transient conditions like turbulence or brief maneuvers.
     *          
     *          A longer delay makes the system less sensitive but may delay assistance
     *          in genuine emergencies. Typical values are 0.5-2.0 seconds.
     * 
     * @note Units: seconds
     * @warning Too short may cause oscillations, too long may delay critical assistance
     */
    AP_Float delay;

    /**
     * @brief Bitmask of special options to modify assist behavior
     * 
     * @details Allows enabling/disabling specific assist features through bit flags.
     *          See OPTION enum for available flags. Multiple options can be combined
     *          using bitwise OR.
     * 
     * @note Default value 0 means all standard assist features are enabled
     * 
     * @see OPTION
     * @see option_is_set()
     */
    AP_Int16 options;

    /**
     * @enum OPTION
     * @brief Configuration options for modifying assist behavior
     * 
     * @details Bitmask flags that can be set in the options parameter to enable
     *          or disable specific assistance features. Multiple options can be
     *          combined using bitwise OR operations.
     */
    enum class OPTION {
        /**
         * @brief Disable all fixed-wing motor assistance (bit 0)
         * 
         * @details When set, prevents automatic engagement of VTOL motors during
         *          fixed-wing flight regardless of other trigger conditions. The
         *          pilot can still manually force assist mode.
         */
        FW_FORCE_DISABLED=(1U<<0),
        
        /**
         * @brief Disable spin recovery assistance (bit 1)
         * 
         * @details When set, disables the specialized spin recovery mode that uses
         *          differential motor thrust and coordinated control surfaces. Standard
         *          assistance modes remain available.
         */
        SPIN_DISABLED=(1U<<1),
    };
    
    /**
     * @brief Check if a specific option flag is enabled
     * 
     * @param[in] option The OPTION flag to check
     * 
     * @return true if the specified option is enabled, false otherwise
     * 
     * @details Performs bitwise AND operation to test if a specific option bit
     *          is set in the options bitmask. Used internally to query configuration.
     * 
     * @see OPTION
     */
    bool option_is_set(OPTION option) const {
        return (options.get() & int32_t(option)) != 0;
    }
    
    /**
     * @enum STATE
     * @brief Pilot-commanded assistance state machine modes
     * 
     * @details Defines the three operating modes for VTOL assistance. The state is
     *          typically commanded by the pilot through a switch or parameter, and
     *          determines whether automatic assistance is available.
     *          
     *          State machine transitions:
     *          - Any state can transition to any other state via pilot command
     *          - Default state on initialization is ASSIST_ENABLED
     *          - State persists across flight modes until changed by pilot
     */
    enum class STATE {
        /**
         * @brief All automatic VTOL assistance disabled by pilot command
         * 
         * @details In this state, should_assist() always returns false regardless
         *          of flight conditions. Used when pilot wants pure fixed-wing
         *          flight without automatic VTOL intervention.
         */
        ASSIST_DISABLED,
        
        /**
         * @brief Normal operation - assistance provided when trigger conditions are met
         * 
         * @details In this state, should_assist() evaluates speed, angle, and altitude
         *          conditions to determine when assistance is needed. This is the
         *          default and recommended state for normal operations.
         */
        ASSIST_ENABLED,
        
        /**
         * @brief Pilot forces VTOL assistance on regardless of conditions
         * 
         * @details In this state, should_assist() always returns true, engaging VTOL
         *          motors continuously. Used for testing or when pilot anticipates
         *          difficult flight conditions. Increases power consumption.
         */
        FORCE_ENABLED,
    };
    
    /**
     * @brief Set the pilot-commanded assistance state
     * 
     * @param[in] _state The desired assistance state (ASSIST_DISABLED, ASSIST_ENABLED, or FORCE_ENABLED)
     * 
     * @details Updates the state machine mode. This is typically called in response to
     *          pilot switch input or parameter change. The state persists until changed
     *          by another call to this function.
     *          
     *          State changes take effect immediately:
     *          - Setting ASSIST_DISABLED will immediately stop assistance
     *          - Setting FORCE_ENABLED will immediately engage assistance
     *          - Setting ASSIST_ENABLED will evaluate conditions on next should_assist() call
     * 
     * @note Does not reset hysteresis timers - use reset() for that
     * 
     * @see STATE
     * @see should_assist()
     */
    void set_state(STATE _state) { state = _state; }

    /**
     * @brief Query if assistance is active due to pilot force-enable command
     * 
     * @return true if in FORCE_ENABLED state, false otherwise
     * 
     * @details Used for telemetry logging to record when assistance was triggered
     *          by pilot command rather than automatic threshold detection. Helps
     *          with post-flight analysis of assistance usage.
     * 
     * @note This is independent of actual motor output - motors may be engaged
     *       even when this returns false if other assist conditions are met
     */
    bool in_force_assist() const { return force_assist; }
    
    /**
     * @brief Query if assistance is active due to low airspeed
     * 
     * @return true if airspeed is below threshold and triggering assistance, false otherwise
     * 
     * @details Used for telemetry logging to record when assistance was triggered
     *          by low airspeed condition. Helps diagnose performance issues or
     *          identify when aircraft is operating near stall speed.
     */
    bool in_speed_assist() const { return speed_assist; }
    
    /**
     * @brief Query if assistance is active due to altitude loss
     * 
     * @return true if altitude assist hysteresis is active, false otherwise
     * 
     * @details Used for telemetry logging to record when assistance was triggered
     *          by excessive altitude loss during climb attempts. Indicates insufficient
     *          fixed-wing thrust or adverse atmospheric conditions.
     */
    bool in_alt_assist() const { return alt_error.is_active(); }
    
    /**
     * @brief Query if assistance is active due to excessive attitude angles
     * 
     * @return true if angle assist hysteresis is active, false otherwise
     * 
     * @details Used for telemetry logging to record when assistance was triggered
     *          by pitch or roll angles exceeding configured limits. Indicates loss
     *          of controlled flight or aggressive maneuvering requiring assistance.
     */
    bool in_angle_assist() const { return angle_error.is_active(); }

    /**
     * @brief Check if vehicle is currently in VTOL-assisted recovery mode
     * 
     * @return true if VTOL motors are actively assisting fixed-wing flight, false otherwise
     * 
     * @details Evaluates whether any assistance condition is currently active and VTOL
     *          motors should be providing thrust. This is the master function that
     *          combines results from all individual assist checks.
     *          
     *          Returns true if any of:
     *          - Force assist is active (pilot command)
     *          - Speed assist is active (low airspeed)
     *          - Altitude assist is active (altitude loss)
     *          - Angle assist is active (excessive attitude error)
     *          
     *          Used by motor mixing code to determine if VTOL motors should be active
     *          during fixed-wing flight modes.
     * 
     * @note Called frequently during flight - must be computationally efficient
     * 
     * @see should_assist()
     */
    bool check_VTOL_recovery(void);

    /**
     * @brief Output coordinated control surface commands for spin recovery
     * 
     * @details Implements specialized control logic for recovering from spins using
     *          a combination of differential motor thrust and coordinated rudder/elevator
     *          inputs. This is more effective than standard assistance for true spin
     *          conditions where the aircraft has departed controlled flight.
     *          
     *          The function:
     *          - Applies differential motor thrust to counter rotation
     *          - Commands rudder opposite to spin direction
     *          - Commands elevator for angle of attack reduction
     *          - Coordinates inputs to prevent adverse yaw
     *          
     *          Spin recovery is automatically disabled if SPIN_DISABLED option is set.
     * 
     * @note Only called when spin condition is detected (high rotation rate + stall)
     * @warning Overrides normal control surface outputs during spin recovery
     * 
     * @see OPTION::SPIN_DISABLED
     */
    void output_spin_recovery(void);
    
private:

    /**
     * @brief Current pilot-commanded assistance state
     * 
     * @details Stores the state machine mode set by pilot command. Initialized to
     *          ASSIST_ENABLED on construction, providing automatic assistance by default.
     *          Modified by calls to set_state().
     * 
     * @see STATE
     * @see set_state()
     */
    STATE state = STATE::ASSIST_ENABLED;

    /**
     * @class Assist_Hysteresis
     * @brief Hysteresis timer for preventing rapid assistance cycling
     * 
     * @details Implements a dual-threshold hysteresis timer used to prevent oscillations
     *          when conditions hover near trigger thresholds. Requires a condition to
     *          persist for a trigger delay before activating, and to be absent for a
     *          clear delay before deactivating.
     *          
     *          This prevents nuisance activations from:
     *          - Turbulence causing brief attitude excursions
     *          - Momentary airspeed fluctuations
     *          - Short-duration altitude variations
     *          
     *          Each assist condition (angle, altitude) has its own hysteresis instance
     *          to provide independent timing control.
     * 
     * @note Speed and force assist do not use hysteresis for immediate response
     */
    class Assist_Hysteresis {
    public:
        /**
         * @brief Reset hysteresis state to inactive
         * 
         * @details Clears all timing information and sets the output to inactive.
         *          Called when exiting fixed-wing modes or when conditions are
         *          definitively resolved and timers should be cleared.
         */
        void reset();

        /**
         * @brief Update hysteresis state based on current trigger condition
         * 
         * @param[in] trigger Current state of the trigger condition (true = condition present)
         * @param[in] now_ms Current time in milliseconds (from AP_HAL::millis())
         * @param[in] trigger_delay_ms Time condition must persist before activation (milliseconds)
         * @param[in] clear_delay_ms Time condition must be absent before deactivation (milliseconds)
         * 
         * @return true if this update call causes transition from inactive to active, false otherwise
         * 
         * @details Implements the hysteresis timing logic:
         *          - When trigger becomes true: starts timing, activates after trigger_delay_ms
         *          - When trigger becomes false: starts timing, deactivates after clear_delay_ms
         *          - If trigger changes state during delay: restarts the timer
         *          
         *          Return value indicates the first update where activation occurs, useful
         *          for logging the exact moment assistance was triggered.
         * 
         * @note Must be called at consistent rate for accurate timing
         * @warning Requires monotonically increasing now_ms value
         */
        bool update(const bool trigger, const uint32_t &now_ms, const uint32_t &trigger_delay_ms, const uint32_t &clear_delay_ms);

        /**
         * @brief Query current hysteresis output state
         * 
         * @return true if hysteresis is in active state, false if inactive
         * 
         * @details Returns the current output state which may differ from the immediate
         *          trigger input due to hysteresis timing. Used to query if the condition
         *          has persisted long enough to be considered active.
         */
        bool is_active() const { return active; }

    private:
        /**
         * @brief Time (ms) when current condition state started
         * 
         * @details Records the timestamp when trigger input changed state, used to
         *          calculate elapsed time for hysteresis delays.
         */
        uint32_t start_ms;
        
        /**
         * @brief Time (ms) of most recent update call
         * 
         * @details Used to detect timing discontinuities or missed updates. Helps
         *          ensure timing accuracy in case of irregular update rates.
         */
        uint32_t last_ms;
        
        /**
         * @brief Current output state (true = active, false = inactive)
         * 
         * @details The hysteresis output state, updated after timing delays. This is
         *          what is_active() returns.
         */
        bool active;
    };
    
    /**
     * @brief Hysteresis timer for angle-based assistance trigger
     * 
     * @details Prevents rapid activation/deactivation when attitude angles fluctuate
     *          near the configured angle threshold due to turbulence or pilot input.
     */
    Assist_Hysteresis angle_error;
    
    /**
     * @brief Hysteresis timer for altitude-based assistance trigger
     * 
     * @details Prevents rapid activation/deactivation when altitude varies near the
     *          configured altitude loss threshold due to updrafts, downdrafts, or
     *          atmospheric disturbances.
     */
    Assist_Hysteresis alt_error;

    /**
     * @brief Flag indicating force assist is active (pilot command)
     * 
     * @details Set true when state is FORCE_ENABLED. No hysteresis applied - responds
     *          immediately to pilot command for testing or anticipated difficult conditions.
     */
    bool force_assist;
    
    /**
     * @brief Flag indicating speed assist is active (low airspeed)
     * 
     * @details Set true when airspeed drops below threshold. No hysteresis applied to
     *          ensure immediate response to stall risk - speed-based assistance needs
     *          to engage quickly to prevent loss of control.
     */
    bool speed_assist;

    /**
     * @brief Reference to parent QuadPlane object for vehicle state access
     * 
     * @details Provides access to vehicle state information including:
     *          - Current attitude (pitch, roll, yaw)
     *          - Altitude and climb rate
     *          - Sensor data (airspeed, GPS, IMU)
     *          - Motor control outputs
     *          - Flight mode information
     *          
     *          This reference is essential for the assistance system to monitor
     *          conditions and command motor outputs when assistance is needed.
     * 
     * @note Reference is const after construction - cannot be reassigned
     */
    QuadPlane& quadplane;
};
