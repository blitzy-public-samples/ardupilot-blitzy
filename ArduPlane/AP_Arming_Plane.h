/**
 * @file AP_Arming_Plane.h
 * @brief Plane-specific arming check implementation
 * 
 * @details This file implements the AP_Arming_Plane class which extends
 *          the base AP_Arming class with fixed-wing aircraft specific
 *          pre-arm and arming validation checks. These checks ensure
 *          that the aircraft is safe to arm and fly, including:
 *          - Airspeed sensor validation and calibration checks
 *          - Quadplane-specific safety checks (if enabled)
 *          - RC input validation for manual flight modes
 *          - Terrain database requirements for terrain following
 *          - Mission validity checks for AUTO mode
 *          - Inertial navigation system (INS) health checks
 *          - Flight surface and servo configuration validation
 * 
 *          The arming system is safety-critical and prevents motor
 *          activation until all configured checks pass successfully.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <AP_Arming/AP_Arming.h>

#ifndef AP_PLANE_BLACKBOX_LOGGING
#define AP_PLANE_BLACKBOX_LOGGING 0
#endif

/**
 * @class AP_Arming_Plane
 * @brief Fixed-wing aircraft specific arming check implementation
 * 
 * @details This class extends AP_Arming to provide comprehensive pre-arm
 *          and arming validation checks specific to fixed-wing aircraft.
 *          It implements plane-specific safety checks that must pass before
 *          the aircraft can be armed and the motor/propeller activated.
 * 
 *          Key plane-specific checks include:
 *          - Airspeed sensor presence and calibration (for non-quadplanes)
 *          - Quadplane transition and tilt rotor configuration validation
 *          - Rudder and aileron control surface configuration
 *          - Minimum throttle and servo output validation
 *          - Terrain database availability for terrain-following missions
 *          - Mission validity including takeoff commands for AUTO mode
 *          - RC receiver configuration for manual flight modes
 * 
 *          The class supports delayed arming for quadplane configurations
 *          where motor spool-up should be delayed until the aircraft is
 *          ready for takeoff. This prevents premature motor activation
 *          when tilting mechanisms are not in flight position.
 * 
 *          Arming checks can be partially bypassed using ARMING_CHECK
 *          parameter bits, but mandatory checks (safety-critical) cannot
 *          be bypassed and always run regardless of configuration.
 * 
 * @note This class is instantiated once per vehicle in the Plane object
 * @warning Modifying arming checks affects flight safety - changes must
 *          be thoroughly tested in SITL and with hardware on the ground
 * 
 * @see AP_Arming for base arming functionality
 * @see Plane::arming for the Plane vehicle arming instance
 */
class AP_Arming_Plane : public AP_Arming
{
public:
    /**
     * @brief Construct plane-specific arming checker
     * 
     * @details Initializes the arming system and sets up parameter defaults
     *          from the var_info table. The constructor chains to the base
     *          AP_Arming constructor and then initializes plane-specific
     *          parameter defaults.
     * 
     * @note Called once during Plane vehicle initialization
     */
    AP_Arming_Plane()
        : AP_Arming()
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Arming_Plane);

    /**
     * @brief Execute comprehensive pre-arm safety checks for fixed-wing aircraft
     * 
     * @details Performs all configured pre-arm validation checks before allowing
     *          the vehicle to be armed. This includes checks inherited from the
     *          base AP_Arming class plus plane-specific validations:
     *          - Airspeed sensor calibration and health (if configured)
     *          - Control surface configuration and neutral positions
     *          - Quadplane-specific checks (motors, tilt servos, transition)
     *          - Mission validity for AUTO mode (takeoff command required)
     *          - RC input validation for manual modes
     *          - Inertial sensor (IMU) health and calibration
     *          - Terrain database availability (if terrain following enabled)
     *          - Compass calibration and health
     *          - GPS fix quality and HDOP
     *          - Battery voltage and capacity
     *          - Parameter sanity checks
     * 
     *          Pre-arm checks are typically run continuously while disarmed and
     *          must all pass before the arm_checks() can succeed. Failures are
     *          reported to the ground station via status text messages.
     * 
     * @param[in] report If true, send failure reasons to GCS via status text.
     *                   If false, fail silently without reporting.
     * 
     * @return true if all configured pre-arm checks passed
     * @return false if any check failed (see GCS messages for details if report=true)
     * 
     * @note Called at approximately 1Hz while disarmed
     * @warning Bypassing pre-arm checks (ARMING_CHECK=0) significantly reduces
     *          flight safety and should only be used for ground testing
     * 
     * @see arm_checks() for checks performed during arming attempt
     * @see mandatory_checks() for checks that cannot be bypassed
     */
    bool pre_arm_checks(bool report) override;

    /**
     * @brief Execute arming checks during an arming attempt
     * 
     * @details Performs final validation checks when the pilot attempts to arm
     *          the vehicle. This includes all pre-arm checks plus additional
     *          checks that are only relevant at arming time:
     *          - Pilot is holding throttle at minimum position
     *          - Vehicle is not in a restricted flight mode
     *          - Geofence is not breached
     *          - Logging system is ready
     *          - Previous flight is properly logged
     * 
     *          For quadplanes with delayed arming enabled, this function sets
     *          the delay_arming flag to prevent immediate motor spool-up.
     * 
     * @param[in] method The arming method being attempted:
     *                   - NONE: Invalid method
     *                   - RUDDER: Rudder stick arming
     *                   - MAVLINK: MAVLink command arming
     *                   - SWITCH: Arm switch activation
     *                   - DroneCAN: CAN bus arming command
     * 
     * @return true if arming is allowed and motors will be armed
     * @return false if arming is denied (failure reported to GCS)
     * 
     * @note This is called once per arming attempt
     * @warning This is a safety-critical function - failures here prevent
     *          potentially dangerous flight conditions
     * 
     * @see pre_arm_checks() for continuous validation while disarmed
     * @see disarm() for the complementary disarming functionality
     */
    bool arm_checks(AP_Arming::Method method) override;

    /**
     * @brief Parameter table for plane-specific arming parameters
     * 
     * @details Defines AP_Param parameters specific to plane arming checks.
     *          Currently includes blackbox logging parameters when enabled.
     *          This table is registered with AP_Param during construction.
     */
    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @brief Disarm the vehicle motors
     * 
     * @details Disarms the aircraft, stopping all motors and servos. For planes,
     *          this means setting throttle to zero and disabling motor output.
     *          For quadplanes, this stops all lift motors and transitions servos
     *          to safe positions.
     * 
     *          Disarm checks validate that it's safe to disarm (vehicle on ground,
     *          not in critical flight phase). These checks can be bypassed using
     *          the do_disarm_checks parameter for emergency situations.
     * 
     * @param[in] method The disarm method being used:
     *                   - RUDDER: Rudder stick disarm
     *                   - MAVLINK: MAVLink command disarm
     *                   - SWITCH: Disarm switch activation
     *                   - DroneCAN: CAN bus disarm command
     * @param[in] do_disarm_checks If true, perform safety checks before disarming.
     *                             If false, force disarm regardless of state.
     * 
     * @return true if disarm succeeded
     * @return false if disarm was denied by safety checks
     * 
     * @note For forced disarm (emergency), set do_disarm_checks=false
     * @warning Forced disarm while flying will result in loss of control
     * 
     * @see arm() for the complementary arming functionality
     */
    bool disarm(AP_Arming::Method method, bool do_disarm_checks=true) override;

    /**
     * @brief Arm the vehicle motors
     * 
     * @details Arms the aircraft, enabling motor output and allowing throttle
     *          control. Calls arm_checks() to validate it's safe to arm unless
     *          do_arming_checks is false (forced arming).
     * 
     *          For quadplanes with OPTION_DELAY_ARMING or OPTION_TILT_DISARMED,
     *          this sets delay_arming flag to prevent immediate motor spool-up
     *          until the vehicle is ready for takeoff.
     * 
     *          Arming transitions the vehicle from DISARMED to ARMED state and
     *          begins logging the flight. All safety checks must pass unless
     *          explicitly bypassed.
     * 
     * @param[in] method The arming method being attempted (see arm_checks)
     * @param[in] do_arming_checks If true, perform full arming validation.
     *                             If false, force arm bypassing all checks.
     * 
     * @return true if arm succeeded and motors are now armed
     * @return false if arming was denied (see GCS messages for reason)
     * 
     * @note Forced arming (do_arming_checks=false) should only be used for
     *       emergency situations or authorized ground testing
     * @warning Forced arming bypasses safety checks and may result in unsafe
     *          flight conditions or immediate crashes
     * 
     * @see disarm() for disarming functionality
     * @see arm_checks() for validation logic
     */
    bool arm(AP_Arming::Method method, bool do_arming_checks=true) override;

    /**
     * @brief Update software arm state for quadplane delayed arming
     * 
     * @details Updates the software armed state to handle delayed motor spool-up
     *          for quadplanes. When delay_arming is true, this function clears
     *          the flag after the configured delay period (AP_ARMING_DELAY_MS),
     *          allowing motors to spool up.
     * 
     *          This delay prevents immediate motor activation when arming with
     *          tilts in non-flight position or when using OPTION_DELAY_ARMING.
     *          The delay gives the pilot time to prepare for motor activation
     *          and allows tilt servos to reach flight position.
     * 
     * @note Called from the main loop after arming while delay_arming is true
     * @note Delay duration is defined by AP_ARMING_DELAY_MS (default 2 seconds)
     * 
     * @see get_delay_arming() to query current delay state
     */
    void update_soft_armed();

    /**
     * @brief Get the delayed arming state for quadplanes
     * 
     * @details Returns whether arming is currently delayed. When true, the vehicle
     *          is armed but motors are not yet spooled up, waiting for the delay
     *          period to expire or for tilt servos to reach flight position.
     * 
     * @return true if arming is delayed (motors not spooled despite armed state)
     * @return false if not delayed or not applicable (normal arming behavior)
     * 
     * @note Only relevant for quadplanes with OPTION_DELAY_ARMING or
     *       OPTION_TILT_DISARMED options enabled
     * 
     * @see update_soft_armed() for the delay management logic
     */
    bool get_delay_arming() const { return delay_arming; };

    /**
     * @brief Perform mandatory arming checks that cannot be bypassed
     * 
     * @details Executes critical safety checks that must pass even when
     *          ARMING_CHECK is set to 0 (skip all checks) or when forced
     *          arming is requested. These are the absolute minimum checks
     *          required for flight safety:
     *          - Battery voltage above critical minimum
     *          - IMU sensors responding and producing valid data
     *          - RC failsafe not active (if RC is required)
     *          - Quadplane motor configuration valid
     *          - Critical parameters within safe ranges
     * 
     *          This function is only called when normal arming checks are
     *          bypassed (ARMING_CHECK=0 or forced arming). When full checks
     *          are enabled, this is redundant with pre_arm_checks().
     * 
     * @param[in] display_failure If true, send failure reasons to GCS
     *                            If false, fail silently without reporting
     * 
     * @return true if all mandatory safety checks passed
     * @return false if any critical safety check failed
     * 
     * @note These checks cannot be disabled - they always run when bypassing
     *       normal checks to prevent obviously dangerous conditions
     * @warning Even with these checks, bypassing ARMING_CHECK significantly
     *          increases risk and should only be used for ground testing
     * 
     * @see pre_arm_checks() for the full check suite
     * @see arm_checks() for arming-time validation
     */
    bool mandatory_checks(bool display_failure) override;

protected:
    /**
     * @brief Validate inertial navigation system (INS) health for planes
     * 
     * @details Checks that IMU sensors (gyroscopes and accelerometers) are
     *          functioning correctly and producing valid data. For planes,
     *          this validates:
     *          - At least one IMU is healthy and calibrated
     *          - Gyro and accelerometer offsets are within acceptable ranges
     *          - No excessive vibration detected
     *          - IMU sample rates are correct
     *          - Temperature calibration valid (if enabled)
     * 
     * @param[in] report If true, send failure details to GCS
     * 
     * @return true if INS checks passed
     * @return false if INS health check failed
     * 
     * @note Overrides base AP_Arming::ins_checks() with plane-specific validation
     * 
     * @see AP_InertialSensor for IMU management
     */
    bool ins_checks(bool report) override;

    /**
     * @brief Determine if terrain database is required for current mission
     * 
     * @details Checks whether the terrain database must be available based on
     *          the planned mission and configured features:
     *          - Returns true if mission contains terrain-relative waypoints
     *          - Returns true if terrain following is enabled (TERRAIN_FOLLOW)
     *          - Returns false for missions without terrain references
     * 
     *          If terrain database is required but not available, pre-arm checks
     *          will fail to prevent flying terrain-following missions without
     *          elevation data, which could cause terrain collision.
     * 
     * @return true if terrain database is required for flight safety
     * @return false if terrain database is not needed for current configuration
     * 
     * @note Overrides base class to provide plane-specific terrain requirements
     * 
     * @see AP_Terrain for terrain database management
     * @see mission_checks() for mission validation
     */
    bool terrain_database_required() const override;

    /**
     * @brief Validate quadplane-specific configuration and state
     * 
     * @details For quadplanes (planes with multicopter lift motors), performs
     *          comprehensive validation of the VTOL configuration:
     *          - Q_ENABLE parameter is set correctly
     *          - Lift motors are configured and responding
     *          - Tilt servos (if used) are configured and at valid positions
     *          - Transition parameters are within safe ranges
     *          - Forward and lift motor combinations are valid
     *          - Tailsitter configuration is correct (if applicable)
     *          - Battery capacity sufficient for VTOL operations
     * 
     *          This check only runs if Q_ENABLE is true (quadplane enabled).
     *          Failures indicate configuration errors that would prevent safe
     *          VTOL operation or transitions between hover and forward flight.
     * 
     * @param[in] display_failure If true, report failures to GCS
     * 
     * @return true if quadplane checks passed (or not a quadplane)
     * @return false if quadplane configuration is invalid
     * 
     * @note Only relevant for quadplane configurations (Q_ENABLE=1)
     * @warning Quadplane configuration errors can cause loss of control during
     *          transitions or VTOL flight - thorough ground testing required
     * 
     * @see QuadPlane class for VTOL control implementation
     */
    bool quadplane_checks(bool display_failure);

    /**
     * @brief Validate mission for AUTO mode flight
     * 
     * @details Checks that the loaded mission is valid and safe for autonomous
     *          execution. For planes, this validates:
     *          - Mission contains at least one valid command
     *          - First command is a takeoff command (NAV_TAKEOFF or VTOL_TAKEOFF)
     *          - All waypoints are within valid coordinate ranges
     *          - Mission does not contain unsupported commands
     *          - Terrain-following missions have valid terrain data
     *          - Rally points are configured if required
     *          - Mission fits within available storage
     * 
     *          Mission validation prevents arming in AUTO mode with an empty
     *          or invalid mission that would cause undefined behavior.
     * 
     * @param[in] report If true, send validation failures to GCS
     * 
     * @return true if mission is valid for autonomous flight
     * @return false if mission validation failed
     * 
     * @note Overrides base class with plane-specific mission requirements
     * @note Only checked when arming in AUTO mode
     * 
     * @see AP_Mission for mission management
     * @see terrain_database_required() for terrain mission validation
     */
    bool mission_checks(bool report) override;

    /**
     * @brief Check that RC input is available if configured for use
     * 
     * @details Validates that RC (radio control) receiver is providing valid
     *          input if the vehicle is configured to use RC control. This check:
     *          - Verifies RC receiver is bound and receiving valid data
     *          - Checks that control inputs are within expected ranges
     *          - Validates mode switch positions are readable
     *          - Confirms arming switch state is known (if used)
     * 
     *          If RC is required for the current flight mode (MANUAL, STABILIZE,
     *          FBWA, etc.) but no valid RC input is detected, arming is prevented
     *          to avoid loss of control.
     * 
     *          For autonomous modes that don't require RC (AUTO, GUIDED), this
     *          check may be skipped depending on configuration.
     * 
     * @param[in] display_failure If true, report RC failure to GCS
     * 
     * @return true if RC is available or not required
     * @return false if RC is required but not receiving valid input
     * 
     * @note RC check requirements depend on flight mode and configuration
     * @warning Flying manual modes without RC input will result in loss of control
     * 
     * @see RC_Channels for RC input management
     */
    bool rc_received_if_enabled_check(bool display_failure);

private:
    /**
     * @brief Handle internal state changes during arm/disarm transitions
     * 
     * @details Manages internal state machine transitions when arming or disarming
     *          the vehicle. This includes:
     *          - Updating armed flags and timers
     *          - Initializing subsystems on arming (logging, navigation)
     *          - Clearing state on disarm
     *          - Setting initial motor/servo positions
     *          - Resetting integrators and filters
     * 
     * @note Called internally by arm() and disarm() methods
     * @note Not directly called by external code
     */
    void change_arm_state(void);

    /**
     * @brief Delayed arming flag for quadplane motor spool-up control
     * 
     * @details When true, indicates that the vehicle is armed but motors should
     *          not spool up yet. This delay allows:
     *          - Tilt servos to move to flight position before motor start
     *          - Pilot preparation time after arming
     *          - Safe transition from disarmed to armed state
     * 
     *          Duration of delay is AP_ARMING_DELAY_MS (typically 2 seconds).
     *          Flag is cleared by update_soft_armed() after delay expires.
     * 
     *          Only used for quadplanes when OPTION_DELAY_ARMING or
     *          OPTION_TILT_DISARMED options are enabled. Ignored for
     *          conventional fixed-wing planes.
     * 
     * @note Oneshot flag - set once on arming, cleared after delay
     * @see update_soft_armed() for delay management
     * @see get_delay_arming() to query current state
     */
    bool delay_arming;

#if AP_PLANE_BLACKBOX_LOGGING
    /**
     * @brief Minimum 3D speed threshold for blackbox logging activation
     * 
     * @details Configurable speed threshold (in m/s) above which blackbox
     *          logging is triggered. Blackbox logging captures detailed
     *          vehicle state for analysis and debugging.
     * 
     * @note Only compiled when AP_PLANE_BLACKBOX_LOGGING is enabled
     */
    AP_Float blackbox_speed;

    /**
     * @brief Timestamp of last time 3D speed exceeded blackbox threshold
     * 
     * @details Records the system time (in milliseconds) when the vehicle
     *          last exceeded the blackbox_speed threshold. Used to manage
     *          blackbox logging state and prevent excessive logging.
     * 
     * @note Only compiled when AP_PLANE_BLACKBOX_LOGGING is enabled
     * @note Timestamp in milliseconds since system boot
     */
    uint32_t last_over_3dspeed_ms;
#endif
};
