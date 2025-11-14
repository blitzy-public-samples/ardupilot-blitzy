/**
 * @file pullup.h
 * @brief Glider pullup maneuver management after NAV_ALTITUDE_WAIT
 * 
 * @details This file defines the GliderPullup class which manages automated
 *          pullup maneuvers for gliders following thermal soaring or altitude
 *          wait operations. The pullup sequence helps gliders transition from
 *          high-speed descent to level flight safely by managing elevator
 *          deflection, pitch attitude, and load factor (negative G) limits.
 *          
 *          The pullup state machine progresses through stages to:
 *          - Build airspeed during descent
 *          - Apply elevator deflection to initiate pullup
 *          - Monitor pitch angle and load factor
 *          - Level off at target altitude
 *          - Push nose down if needed to prevent stall
 *          
 *          This feature is primarily used in soaring applications where gliders
 *          need to recover from steep descents after thermal exits.
 * 
 * @note Currently enabled only on SITL by default (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
 * @warning Improper configuration can lead to excessive G-loads or stall conditions
 * 
 * @see ArduPlane soaring documentation
 * @see NAV_ALTITUDE_WAIT mission command
 */

#pragma once

#ifndef AP_PLANE_GLIDER_PULLUP_ENABLED
#define AP_PLANE_GLIDER_PULLUP_ENABLED CONFIG_HAL_BOARD == HAL_BOARD_SITL
#endif

#if AP_PLANE_GLIDER_PULLUP_ENABLED

/**
 * @class GliderPullup
 * @brief Manages automated pullup maneuvers for gliders after descent or altitude wait
 * 
 * @details The GliderPullup class implements a state machine that controls the pullup
 *          sequence for gliders transitioning from high-speed descent to level flight.
 *          This is particularly useful after thermal soaring operations or NAV_ALTITUDE_WAIT
 *          mission commands where the glider has built up significant airspeed during descent.
 *          
 *          The pullup sequence manages:
 *          - Elevator deflection timing and magnitude
 *          - Pitch attitude targets and monitoring
 *          - Negative G-load limits to prevent structural stress
 *          - Airspeed management during the maneuver
 *          - Recovery actions if parameters are exceeded
 *          
 *          State Machine Flow:
 *          NONE → WAIT_AIRSPEED → WAIT_PITCH → WAIT_LEVEL → PUSH_NOSE_DOWN → NONE
 *          
 *          Configuration is done through AP_Param parameters including:
 *          - Enable/disable flag
 *          - Elevator offset fraction
 *          - Negative G limit
 *          - Airspeed and pitch thresholds
 *          - Jerk limits for load factor
 *          
 * @note The pullup is triggered when entering stabilize mode after NAV_ALTITUDE_WAIT
 * @warning Incorrect parameter tuning can result in excessive load factors or stalls
 */
class GliderPullup
{
public:
    /**
     * @brief Constructor for GliderPullup
     * 
     * @details Initializes the pullup state machine to NONE stage and sets up
     *          parameter storage for configuration values.
     */
    GliderPullup(void);

    /**
     * @brief Reset the pullup state machine to initial state
     * 
     * @details Sets the current stage to NONE, effectively canceling any
     *          active pullup maneuver and returning to normal flight operations.
     *          This is typically called when changing flight modes or when a
     *          pullup is manually aborted.
     */
    void reset(void) {
        stage = Stage::NONE;
    }
    
    /**
     * @brief Check if currently executing a pullup maneuver
     * 
     * @return true if pullup is active (stage != NONE), false otherwise
     * 
     * @note This is a const method and does not modify state
     */
    bool in_pullup() const;
    
    /**
     * @brief Verify and progress pullup state machine during mission execution
     * 
     * @details Called periodically during AUTO mode to check pullup completion
     *          criteria and advance through state machine stages. Monitors
     *          airspeed, pitch angle, and load factor to determine when each
     *          stage is complete.
     * 
     * @return true if pullup verification passes, false if conditions not met
     * 
     * @note Called from mission item verification code
     */
    bool verify_pullup(void);
    
    /**
     * @brief Execute pullup control logic in stabilize mode
     * 
     * @details Implements the actual control during pullup maneuver by:
     *          - Applying elevator offset during WAIT_AIRSPEED stage
     *          - Monitoring and controlling pitch during WAIT_PITCH
     *          - Managing load factor limits
     *          - Transitioning between stages based on flight conditions
     *          
     *          This method is called from the stabilize flight mode at the
     *          main loop rate (typically 50-400 Hz).
     * 
     * @warning Must be called from stabilize mode only
     * @note Modifies elevator demands and pitch targets
     */
    void stabilize_pullup(void);
    
    /**
     * @brief Check if pullup maneuver has completed successfully
     * 
     * @details Returns true when the pullup sequence has finished and the
     *          aircraft has leveled off at the target altitude. This indicates
     *          it's safe to transition to the next flight mode or mission item.
     * 
     * @return true if pullup is complete (stage returned to NONE), false otherwise
     */
    bool pullup_complete(void);
    
    /**
     * @brief Initiate a pullup maneuver
     * 
     * @details Checks if pullup is enabled and conditions are appropriate to
     *          start the pullup sequence. If conditions are met, transitions
     *          from NONE stage to WAIT_AIRSPEED stage to begin the maneuver.
     * 
     * @return true if pullup started successfully, false if disabled or conditions not met
     * 
     * @note Typically called when exiting NAV_ALTITUDE_WAIT or thermal soaring
     */
    bool pullup_start(void);

    /**
     * @enum Stage
     * @brief Pullup state machine stages
     * 
     * @details The pullup maneuver progresses through these stages sequentially
     *          to safely transition from high-speed descent to level flight.
     *          Each stage has specific entry and exit conditions based on
     *          airspeed, pitch angle, and load factor measurements.
     */
    enum class Stage : uint8_t {
        /**
         * @brief No pullup active - normal flight operations
         * 
         * @details Initial and final state. Pullup logic is inactive.
         */
        NONE=0,
        
        /**
         * @brief Waiting to reach target airspeed during descent
         * 
         * @details During this stage, elevator offset is applied to prepare
         *          for the pullup. The stage transitions to WAIT_PITCH once
         *          the target airspeed (airspeed_start parameter) is reached.
         *          
         * @note Elevator offset helps pre-load the wing before full pullup
         */
        WAIT_AIRSPEED,
        
        /**
         * @brief Actively pulling up and monitoring pitch angle
         * 
         * @details Elevator deflection is gradually released while monitoring
         *          pitch angle and load factor. Transitions to WAIT_LEVEL when
         *          target pitch (pitch_start parameter) is achieved and load
         *          factor is within limits.
         *          
         * @warning Monitor negative G limits (ng_limit) to prevent structural stress
         */
        WAIT_PITCH,
        
        /**
         * @brief Leveling off at target altitude
         * 
         * @details Maintains pitch attitude and monitors for level flight
         *          conditions. Transitions to PUSH_NOSE_DOWN if needed to
         *          prevent stall, or returns to NONE when maneuver complete.
         */
        WAIT_LEVEL,
        
        /**
         * @brief Emergency nose-down push to prevent stall
         * 
         * @details If airspeed drops too low or pitch becomes excessive during
         *          WAIT_LEVEL, this stage pushes the nose down to prevent stall.
         *          Returns to WAIT_LEVEL once safe conditions are restored.
         *          
         * @warning This is a safety recovery stage - indicates parameters may need tuning
         */
        PUSH_NOSE_DOWN,
    };

    /**
     * @brief Parameter table for configuration values
     * 
     * @details Defines the AP_Param group table containing all configurable
     *          parameters for the pullup maneuver. This table is used by the
     *          parameter system for storage, retrieval, and ground station access.
     *          
     * @see AP_Param documentation for parameter system details
     */
    static const struct AP_Param::GroupInfo var_info[];

private:
    /**
     * @brief Current stage in the pullup state machine
     * 
     * @details Tracks which stage of the pullup sequence is currently active.
     *          Updated by state machine logic in verify_pullup() and stabilize_pullup().
     */
    Stage stage;
    
    /**
     * @brief Enable/disable pullup functionality
     * 
     * @details Parameter controlling whether pullup maneuvers are allowed.
     *          - 0 = Disabled (pullup will not execute)
     *          - 1 = Enabled (pullup can be triggered)
     *          
     * @note Type: AP_Int8 (8-bit integer parameter with EEPROM storage)
     */
    AP_Int8  enable;
    
    /**
     * @brief Elevator offset fraction during initial pullup stages
     * 
     * @details Fraction of full elevator deflection (0.0 to 1.0) applied during
     *          WAIT_AIRSPEED stage and gradually released during WAIT_PITCH stage.
     *          Higher values provide more aggressive pullup but increase load factor.
     *          
     * @note Units: fraction (dimensionless, range typically 0.0 to 0.5)
     * @note Type: AP_Float (floating point parameter with EEPROM storage)
     * @warning Excessive values can cause structural stress or loss of control
     */
    AP_Float elev_offset;
    
    /**
     * @brief Negative G-load limit threshold
     * 
     * @details Maximum negative load factor (in G's) allowed during pullup.
     *          If exceeded, control logic will reduce pitch rate or transition
     *          to PUSH_NOSE_DOWN stage. Protects airframe from excessive stress.
     *          
     * @note Units: G (multiples of gravitational acceleration, typically negative values like -2.0)
     * @note Type: AP_Float (floating point parameter with EEPROM storage)
     * @warning Must be set based on airframe structural limits
     */
    AP_Float ng_limit;
    
    /**
     * @brief Target airspeed to begin pullup
     * 
     * @details Airspeed threshold (in m/s) that triggers transition from
     *          WAIT_AIRSPEED to WAIT_PITCH stage. Should be high enough to
     *          ensure safe maneuvering margin above stall speed.
     *          
     * @note Units: m/s (meters per second)
     * @note Type: AP_Float (floating point parameter with EEPROM storage)
     */
    AP_Float airspeed_start;
    
    /**
     * @brief Target pitch angle to reach during pullup
     * 
     * @details Pitch attitude (in degrees) that indicates successful pullup.
     *          Transition from WAIT_PITCH to WAIT_LEVEL occurs when this pitch
     *          is achieved. Positive values indicate nose-up attitude.
     *          
     * @note Units: degrees (positive = nose up, typical range 0 to 20 degrees)
     * @note Type: AP_Float (floating point parameter with EEPROM storage)
     */
    AP_Float pitch_start;
    
    /**
     * @brief Maximum rate of change of negative G-load (jerk limit)
     * 
     * @details Limits how quickly the load factor can change (in G/s) to provide
     *          smoother pullup and reduce structural stress. Lower values create
     *          gentler pullup but take longer to complete.
     *          
     * @note Units: G/s (G's per second)
     * @note Type: AP_Float (floating point parameter with EEPROM storage)
     */
    AP_Float ng_jerk_limit;
    
    /**
     * @brief Demanded pitch angle during pullup
     * 
     * @details Calculated pitch demand (in degrees) sent to attitude controller
     *          during pullup maneuver. Updated each control loop iteration based
     *          on current stage and flight conditions.
     *          
     * @note Units: degrees
     * @note Type: AP_Float (floating point parameter with EEPROM storage)
     */
    AP_Float pitch_dem;
    
    /**
     * @brief Current negative G-load demand
     * 
     * @details Calculated load factor demand used internally for control logic
     *          and rate limiting. Updated each loop based on ng_limit and ng_jerk_limit.
     *          
     * @note Units: G (multiples of gravitational acceleration)
     * @note Type: float (local variable, not stored in EEPROM)
     */
    float ng_demand;
};

#endif // AP_PLANE_GLIDER_PULLUP_ENABLED

