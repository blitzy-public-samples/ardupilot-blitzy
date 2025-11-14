/**
 * @file AP_Arming_Rover.h
 * @brief Rover-specific arming checks and pre-flight validation system
 * 
 * @details This file implements the rover-specific arming safety system that extends
 *          the base AP_Arming framework with ground vehicle-specific pre-flight checks.
 *          The arming system ensures that all critical systems (motors, RC, sensors, modes)
 *          are properly configured and functioning before allowing vehicle operation.
 *          
 *          Key responsibilities:
 *          - Pre-arm safety checks specific to ground vehicles
 *          - Motor and steering system validation
 *          - Object avoidance system checks
 *          - Mode-specific arming requirements
 *          - RC calibration verification
 *          - GPS and navigation readiness
 *          
 *          Arming Flow:
 *          1. Pre-arm checks: Validate all systems before allowing arm attempt
 *          2. Arm checks: Final validation during arm command
 *          3. Soft armed update: Continuous monitoring of armed state
 *          
 * @note Rover arming is less restrictive than multicopter arming since ground
 *       vehicles can safely stop without catastrophic consequences
 * 
 * @warning Arming checks are SAFETY-CRITICAL. Bypassing or weakening these checks
 *          can result in vehicle damage, property damage, or injury
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <AP_Arming/AP_Arming.h>
#include <AC_Fence/AC_Fence.h>

/**
 * @class AP_Arming_Rover
 * @brief Rover-specific arming check implementation
 * 
 * @details Extends the base AP_Arming class with ground vehicle-specific safety checks.
 *          This class implements pre-arm and arm validation specific to rovers including:
 *          - Steering and throttle motor configuration checks
 *          - Object avoidance system validation
 *          - Mode appropriateness for arming
 *          - RC input calibration verification
 *          - GPS readiness for guided/auto modes
 *          
 *          The rover arming system is designed to prevent operation with misconfigured
 *          or malfunctioning systems while being less restrictive than aerial vehicles
 *          since rovers can safely stop on the ground.
 *          
 *          Thread Safety: Arming checks are called from the main thread at scheduler rate.
 *          Do not call arming methods from interrupt context.
 * 
 * @see AP_Arming Base arming framework
 * @see Rover Vehicle implementation using this arming system
 */
class AP_Arming_Rover : public AP_Arming
{
public:

    /**
     * @brief Constructor for rover arming system
     * 
     * @details Initializes the rover-specific arming subsystem by calling the base
     *          AP_Arming constructor. Vehicle-specific checks are registered during
     *          initialization.
     */
    AP_Arming_Rover() : AP_Arming() { }

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Arming_Rover);

    /**
     * @brief Perform comprehensive pre-arm safety checks
     * 
     * @details Executes all rover-specific pre-arm checks before allowing an arm attempt.
     *          Checks include:
     *          - Base AP_Arming pre-arm checks (sensors, GPS, battery, etc.)
     *          - Motor configuration and safety checks
     *          - Object avoidance system validation
     *          - Parameter sanity checks
     *          - Mode appropriateness for arming
     *          
     *          Pre-arm checks are designed to catch configuration errors and system
     *          failures before the vehicle attempts to arm. These are more comprehensive
     *          than arm checks and may include tests that take longer to execute.
     * 
     * @param[in] report If true, failures are reported to GCS and logged
     * 
     * @return true if all pre-arm checks pass, false if any check fails
     * 
     * @note Called periodically when disarmed to update pre-arm status indicators
     * @warning This is a SAFETY-CRITICAL function. All failures must prevent arming
     * 
     * @see arm_checks() Final checks during arm command
     * @see motor_checks() Motor system validation
     * @see oa_check() Object avoidance validation
     */
    bool pre_arm_checks(bool report) override;

    /**
     * @brief Perform final arm-time safety checks
     * 
     * @details Executes quick validation checks at the moment of arm command.
     *          These checks are a subset of pre-arm checks focused on conditions
     *          that must be verified immediately before arming (e.g., throttle position,
     *          mode state, GCS heartbeat).
     *          
     *          Arm checks are faster than pre-arm checks since they run synchronously
     *          during the arm command.
     * 
     * @param[in] method Arming method (RUDDER, MAVLINK, SWITCH, etc.)
     * 
     * @return true if arm checks pass and arming is allowed, false otherwise
     * 
     * @note Called only during actual arm attempt, not continuously
     * @warning This is a SAFETY-CRITICAL function. Must complete quickly (<100ms)
     * 
     * @see pre_arm_checks() Comprehensive pre-flight validation
     * @see disarm() Corresponding disarm operation
     */
    bool arm_checks(AP_Arming::Method method) override;

    /**
     * @brief Validate RC input calibration for rover operation
     * 
     * @details Checks that RC inputs are properly calibrated for safe rover operation:
     *          - Steering channel min/max/trim values are reasonable
     *          - Throttle channel calibration allows full forward/reverse range
     *          - Dead zones are configured appropriately
     *          - Channel assignments are valid
     *          
     *          Proper RC calibration is essential for predictable vehicle control,
     *          especially for skid-steering rovers where poor calibration can cause
     *          unwanted rotation.
     * 
     * @param[in] display_failure If true, report failures to GCS with detailed messages
     * 
     * @return true if RC calibration is acceptable, false if recalibration needed
     * 
     * @note Rovers require less strict RC calibration than aircraft since they
     *       can safely stop on the ground
     * @warning Poor RC calibration can cause unexpected vehicle behavior
     * 
     * @see Rover::read_radio() RC input processing
     */
    bool rc_calibration_checks(const bool display_failure) override;

    /**
     * @brief Validate GPS readiness for rover navigation
     * 
     * @details Checks GPS system status for rover-specific requirements:
     *          - GPS fix quality sufficient for current/planned mode
     *          - Position accuracy acceptable for navigation
     *          - Compass calibration and health (if using GPS+compass)
     *          - EKF GPS fusion status
     *          
     *          GPS requirements vary by mode:
     *          - Manual/Acro/Steering: GPS not required
     *          - Guided/Auto/RTL: GPS fix required
     *          - Loiter/Circle: Good GPS accuracy required
     * 
     * @param[in] display_failure If true, report GPS status and failures to GCS
     * 
     * @return true if GPS meets requirements for current mode, false otherwise
     * 
     * @note Rovers can arm without GPS in manual modes, unlike aircraft
     * @warning Operating in GPS modes without adequate GPS fix can cause
     *          navigation failures and vehicle loss
     * 
     * @see AP_GPS GPS subsystem
     * @see AP_AHRS EKF GPS fusion
     */
    bool gps_checks(bool display_failure) override;

    /**
     * @brief Disarm the rover and disable motor outputs
     * 
     * @details Safely disarms the vehicle by:
     *          - Validating disarm conditions (if checks enabled)
     *          - Notifying subsystems of disarm event
     *          - Disabling motor outputs
     *          - Clearing armed state flags
     *          - Logging disarm event and method
     *          
     *          Disarming can be prevented by disarm checks if the vehicle is moving
     *          or in an unsafe state (configurable).
     * 
     * @param[in] method Disarm method (RUDDER, MAVLINK, SWITCH, etc.)
     * @param[in] do_disarm_checks If true, enforce disarm safety checks
     * 
     * @return true if disarm successful, false if disarm checks failed
     * 
     * @note Rovers can typically disarm at any time since they're on the ground
     * @warning Emergency disarm (checks disabled) immediately cuts motor power
     * 
     * @see arm() Corresponding arm operation
     * @see update_soft_armed() Armed state monitoring
     */
    bool disarm(AP_Arming::Method method, bool do_disarm_checks=true) override;

    /**
     * @brief Arm the rover and enable motor outputs
     * 
     * @details Arms the vehicle by:
     *          - Running arming checks (if enabled)
     *          - Validating mode allows arming
     *          - Notifying subsystems of arm event
     *          - Enabling motor outputs
     *          - Setting armed state flags
     *          - Logging arm event and method
     *          
     *          Successful arming transitions vehicle to operational state where
     *          throttle/steering inputs control motor outputs.
     * 
     * @param[in] method Arming method (RUDDER, MAVLINK, SWITCH, SCRIPTING, etc.)
     * @param[in] do_arming_checks If true, enforce all arming safety checks
     * 
     * @return true if arm successful, false if arming checks failed
     * 
     * @note Force-arm (checks disabled) bypasses all safety validation
     * @warning Force-arming bypasses SAFETY-CRITICAL checks and should only be
     *          used in controlled test environments. Never force-arm in field operations
     * 
     * @see disarm() Corresponding disarm operation
     * @see arm_checks() Arming validation logic
     * @see pre_arm_checks() Pre-flight validation
     */
    bool arm(AP_Arming::Method method, bool do_arming_checks=true) override;

    /**
     * @brief Update soft-armed state and monitoring
     * 
     * @details Continuously monitors armed state and enforces soft-armed conditions:
     *          - Checks for conditions that should trigger automatic disarm
     *          - Updates armed state indicators
     *          - Monitors for arming timeout conditions
     *          - Validates continued operation safety
     *          
     *          Soft-armed state allows the system to automatically disarm if safety
     *          conditions change (e.g., loss of RC, critical battery, fence breach).
     * 
     * @note Called at scheduler rate (typically 50Hz) when armed
     * @warning Automatic disarm can occur if safety conditions degrade
     * 
     * @see arm() Initial arming operation
     * @see disarm() Disarm operation
     */
    void update_soft_armed();

protected:
    /**
     * @brief Validate object avoidance system configuration and health
     * 
     * @details Checks the proximity sensor and object avoidance subsystem for:
     *          - Proximity sensor driver initialization
     *          - Sensor health and data validity
     *          - Object avoidance parameter configuration
     *          - Bendy ruler / path planning readiness
     *          
     *          Object avoidance must be functional if enabled, as rovers rely on
     *          proximity sensors to navigate safely around obstacles in auto modes.
     * 
     * @param[in] report If true, report failures to GCS and logs
     * 
     * @return true if OA system is ready or disabled, false if configured but not functional
     * 
     * @note This check does not call into base AP_Arming
     * @warning Operating with failed proximity sensors can cause collisions
     * 
     * @see AP_Proximity Proximity sensor subsystem
     * @see AP_OAPathPlanner Object avoidance path planning
     */
    bool oa_check(bool report);

    /**
     * @brief Validate rover-specific parameter configuration
     * 
     * @details Checks critical rover parameters for safe values:
     *          - Motor configuration parameters (min/max PWM, scaling)
     *          - Steering configuration (rate limits, angle limits)
     *          - Navigation parameters (speed limits, turn rates)
     *          - Failsafe configuration (timeouts, actions)
     *          - Frame class and type parameters
     *          
     *          Parameter validation catches configuration errors that could cause
     *          unsafe behavior (e.g., inverted motor direction, invalid frame type).
     * 
     * @param[in] report If true, report parameter issues to GCS with parameter names
     * 
     * @return true if all critical parameters are valid, false if misconfiguration detected
     * 
     * @note This check does not call into base AP_Arming
     * @warning Invalid parameters can cause vehicle instability or loss of control
     * 
     * @see Parameters.cpp Parameter definitions
     */
    bool parameter_checks(bool report);

    /**
     * @brief Validate current mode allows arming
     * 
     * @details Checks if the current flight mode permits arming:
     *          - Some modes may prevent arming (e.g., Initializing)
     *          - Validates mode-specific preconditions
     *          - Checks mode initialization completed successfully
     *          
     *          Mode checks ensure the vehicle is in an appropriate control mode
     *          before allowing motor activation.
     * 
     * @param[in] report If true, report mode issues to GCS
     * 
     * @return true if current mode allows arming, false otherwise
     * 
     * @note This check does not call into base AP_Arming
     * @note Most rover modes allow arming, unlike aircraft which restrict certain modes
     * 
     * @see Mode Flight mode base class
     * @see Rover::mode Current active mode
     */
    bool mode_checks(bool report);

    /**
     * @brief Validate motor and drive system configuration
     * 
     * @details Comprehensive motor system checks including:
     *          - Motor driver initialization and health
     *          - ESC communication and telemetry (if available)
     *          - Motor channel configuration and mapping
     *          - Safety switch state (if configured)
     *          - Steering servo configuration and limits
     *          - Throttle range and reversibility
     *          
     *          Motor checks ensure the drive system is properly configured and
     *          responding before allowing armed operation.
     * 
     * @param[in] report If true, report motor issues to GCS with detailed diagnostics
     * 
     * @return true if motor system is ready, false if configuration or hardware issues detected
     * 
     * @note This check does not call into base AP_Arming
     * @warning Motor configuration errors can cause unexpected vehicle motion or
     *          inability to stop (critical safety issue)
     * 
     * @see AP_Motors Motor control library
     * @see SRV_Channel Servo output management
     */
    bool motor_checks(bool report);

};
