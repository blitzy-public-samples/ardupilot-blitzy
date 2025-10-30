#pragma once

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/**
 * @file AP_AdvancedFailsafe.h
 * @brief Advanced Failsafe module for catastrophic failure handling
 * 
 * @details This module implements the Advanced Failsafe System (AFS), originally designed for
 *          the Outback Challenge competition. It provides comprehensive failure detection and
 *          response for safety-critical scenarios including:
 *          - GPS signal loss with altitude enforcement via barometric pressure
 *          - Ground Control Station (GCS) datalink loss
 *          - Dual-loss scenarios (GPS + GCS simultaneous failure)
 *          - Geofence breach immediate termination
 *          - RC transmitter failure with configurable timeout
 *          - Maximum range enforcement from launch location
 *          - AMSL (Above Mean Sea Level) altitude limit enforcement
 * 
 *          The system integrates with external failsafe hardware through GPIO pins:
 *          - Heartbeat pin: 10Hz toggle signal indicates autopilot is alive
 *          - Manual pin: Set high when vehicle is in manual/stabilized mode
 *          - Terminate pin: Set high when termination is commanded
 * 
 *          AFS maintains a state machine tracking flight phase and failure conditions,
 *          with configurable waypoint navigation responses and ultimate termination authority.
 *          Termination can be immediate motor shutdown or controlled landing depending on
 *          configuration.
 * 
 * @note This is safety-critical flight termination code. Any modifications must be thoroughly
 *       tested in SITL before hardware deployment. Incorrect configuration can result in
 *       unintended vehicle termination or failure to terminate when required.
 * 
 * @warning AFS termination authority overrides all other flight modes and pilot input.
 *          When termination is activated, vehicle control is permanently disabled until reboot.
 * 
 * @author Andrew Tridgell and CanberraUAV, August 2012
 * @copyright Copyright (c) 2012-2025 ArduPilot.org
 * 
 * Source: libraries/AP_AdvancedFailsafe/AP_AdvancedFailsafe.h
 */

#include "AP_AdvancedFailsafe_config.h"

#if AP_ADVANCEDFAILSAFE_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <inttypes.h>
#include <AP_Common/Location.h>

/**
 * @class AP_AdvancedFailsafe
 * @brief Advanced failsafe system for detecting and responding to catastrophic flight failures
 * 
 * @details The AP_AdvancedFailsafe class implements a comprehensive safety system that monitors
 *          multiple failure conditions and enforces safety limits with ultimate termination authority.
 *          This abstract base class must be derived by vehicle-specific implementations to provide
 *          platform-specific termination behavior and flight mode mapping.
 * 
 *          **Architecture:**
 *          - Singleton pattern with panic on duplicate instantiation (enforced for safety)
 *          - Abstract base class requiring vehicle-specific implementations of terminate_vehicle(),
 *            setup_IO_failsafe(), afs_mode(), and set_mode_auto()
 *          - Integrated with AP_Param system for persistent configuration storage
 *          - State machine tracks flight phase: PREFLIGHT → AUTO → DATA_LINK_LOSS/GPS_LOSS
 * 
 *          **Failure Detection:**
 *          - GPS loss: Detected via AP_GPS health check, triggers waypoint navigation or termination
 *          - GCS datalink loss: No MAVLink heartbeat within configured timeout
 *          - Dual loss: Simultaneous GPS and GCS failure triggers immediate termination
 *          - Geofence breach: Immediate termination if AC_Fence reports breach
 *          - RC failure: No valid RC input within configured timeout (optional)
 *          - Altitude limit: QNH pressure altitude or GPS altitude exceeds AMSL limit
 *          - Range limit: Distance from first arming location exceeds maximum range
 * 
 *          **Response Actions:**
 *          - Navigate to configured waypoint (WP_COMMS or WP_GPS_LOSS)
 *          - Force AUTO mode entry to execute mission failsafe sequence
 *          - Set GPIO pins for external failsafe board communication
 *          - Terminate vehicle: Either immediate motor cutoff (TERMINATE_ACTION_TERMINATE)
 *            or controlled landing (TERMINATE_ACTION_LAND)
 * 
 *          **Event Counting:**
 *          Loss events are counted with 30-second debounce window. If loss count exceeds
 *          configured maximums (MAX_GPS_LOSS, MAX_COMMS_LOSS), termination is triggered.
 * 
 *          **Integration Points:**
 *          - AP_GPS: GPS health and position monitoring
 *          - AP_Baro: QNH pressure altitude for AMSL limit enforcement
 *          - AP_Mission: Waypoint navigation for failsafe responses
 *          - AC_Fence: Geofence breach detection
 *          - GCS_MAVLink: Datalink loss detection and GCS termination commands
 *          - Vehicle main loop: Must call check() at 10Hz for state machine updates
 * 
 * @note The check() method must be called at exactly 10Hz from the vehicle main loop.
 *       The heartbeat() method should be called at 10Hz during sensor calibration to
 *       keep external failsafe boards satisfied.
 * 
 * @warning This class has ultimate termination authority. Once termination is activated,
 *          it cannot be reversed without vehicle reboot. Test all configurations thoroughly
 *          in SITL before flight testing. Incorrect parameter settings can cause unintended
 *          termination or failure to terminate when safety limits are exceeded.
 * 
 * Source: libraries/AP_AdvancedFailsafe/AP_AdvancedFailsafe.h
 * Source: libraries/AP_AdvancedFailsafe/AP_AdvancedFailsafe.cpp:183-342 (check() state machine)
 */
class AP_AdvancedFailsafe
{
public:
    /**
     * @enum control_mode
     * @brief Vehicle flight mode classification for AFS decision logic
     * 
     * @details Maps vehicle-specific flight modes to AFS control categories to determine
     *          appropriate failsafe responses. Vehicle implementations must provide afs_mode()
     *          method that returns the current control mode classification.
     * 
     *          - AFS_MANUAL: Pilot has direct control (e.g., Manual, Acro, Stabilize modes).
     *            In manual mode, some failsafe responses may be limited or require explicit
     *            RC failure detection before triggering termination.
     * 
     *          - AFS_STABILIZED: Pilot controls with stabilization assistance (e.g., AltHold,
     *            Loiter, PosHold). Intermediate autonomy level where some navigation assistance
     *            is active but pilot retains direct input authority.
     * 
     *          - AFS_AUTO: Fully autonomous flight mode (e.g., Auto, Guided, RTL). Vehicle
     *            executes pre-programmed mission or autonomous navigation. AFS datalink loss
     *            response typically forces entry to this mode for mission-based recovery.
     * 
     * Source: libraries/AP_AdvancedFailsafe/AP_AdvancedFailsafe.h:35-39
     */
    enum control_mode {
        AFS_MANUAL     = 0,  ///< Pilot direct control, minimal stabilization
        AFS_STABILIZED = 1,  ///< Pilot control with stabilization assistance
        AFS_AUTO       = 2   ///< Fully autonomous navigation mode
    };

    /**
     * @enum state
     * @brief AFS state machine states tracking current flight phase and failure condition
     * 
     * @details The AFS state machine transitions based on flight phase and detected failures.
     *          State transitions determine failsafe response actions and termination eligibility.
     * 
     *          **State Machine Transitions:**
     * 
     *          STATE_PREFLIGHT → STATE_AUTO:
     *            Triggered when vehicle enters AFS_AUTO control mode after arming.
     *            Records first GPS location for range limit enforcement.
     * 
     *          STATE_AUTO → STATE_DATA_LINK_LOSS:
     *            Triggered when GCS heartbeat timeout exceeded (_gcs_fail_time_seconds).
     *            Response: Navigate to WP_COMMS waypoint, force AUTO mode entry.
     *            If datalink recovers and CONTINUE_AFTER_RECOVERED option set, returns to STATE_AUTO.
     * 
     *          STATE_AUTO / STATE_DATA_LINK_LOSS → STATE_GPS_LOSS:
     *            Triggered when GPS loses 3D fix and health check fails.
     *            Response: Navigate to WP_GPS_LOSS waypoint.
     *            If GPS recovers and CONTINUE_AFTER_RECOVERED option set, returns to previous state.
     * 
     *          **Termination Conditions (any state):**
     *          - Dual loss: GPS + GCS simultaneously lost (if _enable_dual_loss set)
     *          - Event count exceeded: GPS loss count > MAX_GPS_LOSS or comms loss count > MAX_COMMS_LOSS
     *          - Geofence breach: AC_Fence reports boundary violation (if _enable_geofence_fs set)
     *          - Altitude limit: QNH or GPS altitude exceeds AMSL_LIMIT
     *          - Range limit: Distance from first location exceeds MAX_RANGE_KM
     *          - RC failure: No valid RC input for _rc_fail_time_seconds (if _enable_RC_fs set)
     *          - Manual termination: TERMINATE parameter set or gcs_terminate() called
     * 
     * @note Event counting uses 30-second debounce: repeated loss within 30s doesn't increment counter.
     * 
     * Source: libraries/AP_AdvancedFailsafe/AP_AdvancedFailsafe.h:41-46
     * Source: libraries/AP_AdvancedFailsafe/AP_AdvancedFailsafe.cpp:183-342 (state machine implementation)
     */
    enum state {
        STATE_PREFLIGHT       = 0,  ///< Pre-flight phase, before entering autonomous mode
        STATE_AUTO            = 1,  ///< Normal autonomous flight, all systems operational
        STATE_DATA_LINK_LOSS  = 2,  ///< GCS communication lost, executing comms loss response
        STATE_GPS_LOSS        = 3   ///< GPS fix lost, executing GPS loss response
    };

    /**
     * @enum terminate_action
     * @brief Specifies vehicle behavior when flight termination is triggered
     * 
     * @details Determines whether termination results in immediate motor cutoff or controlled
     *          landing sequence. This is configured via the AFS_TERM_ACTION parameter and
     *          affects the behavior of should_crash_vehicle() and terminate_vehicle() calls.
     * 
     *          **TERMINATE_ACTION_TERMINATE (42):**
     *          Immediate termination with complete motor shutdown. The vehicle will experience
     *          uncontrolled descent. This is appropriate when:
     *          - External failsafe board will handle termination (parachute, motor cutoff)
     *          - Safety requires immediate cessation of powered flight
     *          - Altitude is low enough that uncontrolled descent is acceptable
     * 
     *          **TERMINATE_ACTION_LAND (43):**
     *          Controlled landing sequence maintaining stabilization and controlled descent rate.
     *          The vehicle attempts to land safely at current location. This is appropriate when:
     *          - Controlled descent is safer than motor cutoff (e.g., over populated areas)
     *          - Vehicle has sufficient altitude and control authority for landing
     *          - Failure condition doesn't require immediate power removal
     * 
     * @note Magic numbers (42, 43) are used for compatibility with external failsafe boards
     *       and MAVLink termination protocols. These values should not be changed.
     * 
     * @warning Once termination is triggered, the action cannot be reversed without reboot.
     *          Ensure TERM_ACTION is configured appropriately for your operational environment.
     * 
     * Source: libraries/AP_AdvancedFailsafe/AP_AdvancedFailsafe.h:48-51
     * Source: libraries/AP_AdvancedFailsafe/AP_AdvancedFailsafe.cpp:420-443 (should_crash_vehicle)
     */
    enum terminate_action {
        TERMINATE_ACTION_TERMINATE = 42,  ///< Immediate motor cutoff (uncontrolled descent)
        TERMINATE_ACTION_LAND      = 43   ///< Controlled landing sequence at current location
    };

    /* Do not allow copies */
    CLASS_NO_COPY(AP_AdvancedFailsafe);

    /**
     * @brief Constructor initializes AFS state and enforces singleton pattern
     * 
     * @details Initializes AP_Param defaults, enforces singleton constraint (panics if
     *          duplicate instantiation attempted), sets initial state to PREFLIGHT, and
     *          clears termination flag. The singleton enforcement is critical for safety
     *          as multiple AFS instances could create conflicting termination decisions.
     * 
     * @warning Attempting to create a second AP_AdvancedFailsafe instance will trigger
     *          AP_HAL::panic() and halt the system. This is intentional safety behavior.
     */
    AP_AdvancedFailsafe()
        {
            AP_Param::setup_object_defaults(this, var_info);
            if (_singleton != nullptr) {
                AP_HAL::panic("AP_AdvancedFailsafe must be singleton");
            }

            _singleton = this;
            _state = STATE_PREFLIGHT;
            _terminate.set(0);

            _saved_wp = 0;
        }

    /**
     * @brief Get singleton instance of AP_AdvancedFailsafe
     * 
     * @return Pointer to the singleton AP_AdvancedFailsafe instance, or nullptr if not created
     * 
     * @note Typically accessed via AP::advancedfailsafe() namespace accessor rather than
     *       calling this method directly. Returns nullptr if AFS has not been instantiated.
     */
    static AP_AdvancedFailsafe *get_singleton(void) {
        return _singleton;
    }

    /**
     * @brief Check if Advanced Failsafe system is enabled
     * 
     * @return true if AFS is enabled (AFS_ENABLE parameter set), false otherwise
     * 
     * @details When disabled (AFS_ENABLE = 0), all AFS monitoring and failsafe actions are
     *          bypassed. This provides a master enable/disable switch for the entire system.
     *          Other AFS parameters have no effect when disabled.
     */
    bool enabled() { return _enable; }

    /**
     * @brief Main AFS state machine update and failure detection check
     * 
     * @details This is the primary AFS entry point that must be called at exactly 10Hz from
     *          the vehicle main loop. It performs comprehensive failure monitoring and drives
     *          the AFS state machine through appropriate transitions and termination decisions.
     * 
     *          **Failure Checks Performed (in order):**
     *          1. Manual termination: Check if TERMINATE parameter set or gcs_terminate() called
     *          2. Dual loss termination: GPS + GCS simultaneously lost (if enabled)
     *          3. Altitude limit: QNH pressure or GPS altitude exceeds AMSL_LIMIT
     *          4. Geofence breach: AC_Fence boundary violation (if enabled)
     *          5. Range limit: Distance from first location exceeds MAX_RANGE_KM
     *          6. GPS loss: 3D fix lost, health check failed
     *          7. GCS datalink loss: No MAVLink heartbeat within timeout
     *          8. RC failure: No valid RC input within timeout (if enabled)
     * 
     *          **State Machine Logic:**
     *          - PREFLIGHT → AUTO: When vehicle enters AFS_AUTO mode, records first location
     *          - AUTO → DATA_LINK_LOSS: On GCS timeout, navigates to WP_COMMS waypoint
     *          - AUTO/DATA_LINK_LOSS → GPS_LOSS: On GPS loss, navigates to WP_GPS_LOSS waypoint
     *          - Any state → Termination: On critical failure or event count exceeded
     * 
     *          **Event Counting:**
     *          GPS and GCS loss events are counted. If count exceeds MAX_GPS_LOSS or MAX_COMMS_LOSS,
     *          termination is triggered. Events within 30 seconds don't increment counter (debounce).
     * 
     *          **Termination Actions:**
     *          When termination triggered: Sets _terminate flag, calls setup_IO_failsafe(),
     *          sets GPIO pins, and invokes terminate_vehicle() if TERM_ACTION configured.
     * 
     * @param[in] last_valid_rc_ms Timestamp (ms) of last valid RC input from pilot
     * 
     * @note MUST be called at exactly 10Hz (every 100ms) from vehicle main loop. Timing is
     *       critical for heartbeat generation and failsafe timeout calculations.
     * 
     * @warning This method has ultimate termination authority. Once termination is triggered,
     *          it persists until reboot. Thoroughly test all parameter configurations in SITL
     *          before flight testing. Monitor GCS console for AFS status messages.
     * 
     * Source: libraries/AP_AdvancedFailsafe/AP_AdvancedFailsafe.cpp:183-342
     */
    void check(uint32_t last_valid_rc_ms);

    /**
     * @brief Generate heartbeat signal for external failsafe boards
     * 
     * @details Toggles the AFS_HB_PIN GPIO at 10Hz to indicate the autopilot is alive and
     *          functioning. This is used by external failsafe boards to detect autopilot crashes.
     *          If the heartbeat stops, the external board can take termination action.
     * 
     *          This method should be called at 10Hz during operations where check() is not being
     *          called, such as during sensor calibration at startup. During normal flight, check()
     *          handles heartbeat generation automatically.
     * 
     *          The heartbeat continues even after termination is triggered (if TERM_PIN is set),
     *          allowing the external board to distinguish between autopilot crash (no heartbeat)
     *          and commanded termination (heartbeat continues, TERM_PIN goes high).
     * 
     * @note Call at 10Hz during sensor calibration or other initialization sequences.
     *       Not required during normal flight as check() handles heartbeat internally.
     */
    void heartbeat(void);

    /**
     * @brief Check if vehicle termination has been triggered
     * 
     * @return true if termination is active (vehicle should be crashed), false otherwise
     * 
     * @details Returns the state of the internal _terminate flag, indicating whether flight
     *          termination has been commanded. When true, the vehicle is in termination mode
     *          and should have motors shut down or be executing controlled landing depending
     *          on TERM_ACTION configuration.
     * 
     *          This method is called by vehicle code to check if termination state requires
     *          motor shutdown. It also invokes setup_IO_failsafe() to configure hardware
     *          outputs for FMU failure scenarios.
     * 
     * @note Once termination is triggered, this will return true until vehicle reboot.
     *       There is no way to cancel termination in flight.
     * 
     * @warning This indicates a safety-critical state. Vehicle control should be completely
     *          disabled when this returns true (except for LAND termination action).
     * 
     * Source: libraries/AP_AdvancedFailsafe/AP_AdvancedFailsafe.cpp:420-443
     */
    bool should_crash_vehicle(void);

    /**
     * @brief Request termination or termination cancellation via ground control station
     * 
     * @param[in] should_terminate true to request termination, false to cancel termination request
     * @param[in] reason Human-readable string describing termination reason (for logging)
     * 
     * @return true if AFS is in the desired termination state, false otherwise
     * 
     * @details Allows ground control station to command flight termination via MAVLink or
     *          cancel a termination request (before termination actually triggered). Once
     *          termination is fully activated (_terminate flag set), it cannot be reversed.
     * 
     *          **Termination Request (should_terminate = true):**
     *          Sets internal flag requesting termination. On next check() call, if conditions
     *          are met, full termination sequence will be activated (setup_IO_failsafe(),
     *          GPIO pins set, terminate_vehicle() called).
     * 
     *          **Cancellation (should_terminate = false):**
     *          Only effective if termination request exists but _terminate flag not yet set.
     *          Cannot cancel termination once it has been fully activated.
     * 
     * @note The reason string is logged to help post-flight analysis understand why
     *       termination was requested.
     * 
     * @warning This provides ground station with termination authority. Ensure proper
     *          authentication and authorization before allowing GCS termination commands.
     *          Accidental or malicious termination commands can crash the vehicle.
     * 
     * Source: libraries/AP_AdvancedFailsafe/AP_AdvancedFailsafe.cpp:447-469
     */
    bool gcs_terminate(bool should_terminate, const char *reason);

    /**
     * @brief Execute vehicle-specific termination sequence
     * 
     * @details Pure virtual method that must be implemented by vehicle-specific AFS subclasses.
     *          This method is responsible for executing the actual termination action, which
     *          typically involves:
     *          - Immediate motor shutdown (for TERMINATE_ACTION_TERMINATE)
     *          - Initiating controlled landing sequence (for TERMINATE_ACTION_LAND)
     *          - Disabling further pilot input and mode changes
     *          - Setting failsafe outputs on all channels
     * 
     *          Implementation is vehicle-specific because motor control and landing procedures
     *          differ significantly between Copter, Plane, Rover, and Sub platforms.
     * 
     * @note Called by check() when termination is triggered. May be called multiple times.
     * 
     * @warning This is the ultimate safety action. Implementation must be robust and tested.
     *          Incorrect implementation can result in unintended vehicle crash or failure to
     *          terminate when required. Must handle all vehicle-specific motor types and
     *          configurations.
     */
    virtual void terminate_vehicle(void) = 0;

    /**
     * @brief Parameter table for AP_Param integration
     * 
     * @details Defines all AFS configuration parameters with metadata for ground station
     *          display, validation, and persistent storage. See AP_AdvancedFailsafe.cpp
     *          for complete parameter definitions including display names, descriptions,
     *          units, and valid ranges.
     */
    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @brief Check if termination is using controlled landing instead of immediate motor cutoff
     * 
     * @return true if TERM_ACTION is set to LAND, false if set to TERMINATE
     * 
     * @details Vehicle code uses this to determine whether to maintain stabilization during
     *          termination (landing) or immediately cut motors (terminate). When true, the
     *          vehicle attempts a controlled descent and landing at current location.
     * 
     * @note This does not indicate whether termination is active, only what type of
     *       termination action is configured. Check should_crash_vehicle() to determine
     *       if termination has been triggered.
     */
    bool terminating_vehicle_via_landing() const {
        return _terminate_action == TERMINATE_ACTION_LAND;
    };

protected:
    /**
     * @brief Configure hardware outputs for FMU failure scenarios
     * 
     * @details Pure virtual method that must be implemented by vehicle-specific AFS subclasses.
     *          Configures servo/motor output channels to safe failsafe values that will be
     *          output if the main FMU firmware crashes or stops executing. This typically
     *          involves:
     *          - Setting IOMCU or RCOutput failsafe values for all channels
     *          - Configuring motor outputs to motor-stop PWM values
     *          - Setting control surface outputs to neutral or safe positions
     *          - Ensuring parachute or termination channels are in correct state
     * 
     *          These values are latched into hardware and will be output automatically if
     *          the FMU stops sending updates, providing a hardware-level failsafe independent
     *          of firmware execution.
     * 
     * @note Called by check() when termination is first triggered and by should_crash_vehicle().
     *       Implementation is vehicle and hardware specific (different for IOMCU vs direct output).
     * 
     * @warning Critical for safety. If not configured correctly, FMU crash could leave motors
     *          running or control surfaces in dangerous positions. Test thoroughly with actual
     *          hardware FMU disconnection scenarios.
     */
    virtual void setup_IO_failsafe(void) = 0;

    /**
     * @brief Map current vehicle flight mode to AFS control_mode category
     * 
     * @return Current vehicle mode mapped to AFS_MANUAL, AFS_STABILIZED, or AFS_AUTO
     * 
     * @details Pure virtual method that must be implemented by vehicle-specific AFS subclasses.
     *          Maps vehicle-specific mode enums to AFS control mode categories. This mapping
     *          determines failsafe behavior and termination eligibility.
     * 
     *          **Typical Mappings:**
     *          - Copter: Manual/Acro → AFS_MANUAL, AltHold/Loiter → AFS_STABILIZED, Auto/RTL/Guided → AFS_AUTO
     *          - Plane: Manual/Acro → AFS_MANUAL, Stabilize/FlyByWire → AFS_STABILIZED, Auto/RTL/Guided → AFS_AUTO
     *          - Rover: Manual/Acro → AFS_MANUAL, Hold → AFS_STABILIZED, Auto/RTL/Guided → AFS_AUTO
     * 
     * @note Called frequently by check() to determine current flight phase and appropriate
     *       failsafe responses. Should be lightweight (simple mode comparison).
     */
    virtual enum control_mode afs_mode(void) = 0;

    /**
     * @brief Force vehicle into autonomous AUTO flight mode
     * 
     * @details Pure virtual method that must be implemented by vehicle-specific AFS subclasses.
     *          Called when datalink loss is detected to force the vehicle into AUTO mode for
     *          mission-based recovery. This typically means:
     *          - Setting vehicle mode to Auto/Mission mode
     *          - Resuming mission execution from saved waypoint
     *          - Disabling mode change requests until datalink recovers
     * 
     *          The forced AUTO mode entry allows the vehicle to execute pre-programmed failsafe
     *          mission sequences (navigate to rally point, execute landing sequence, etc.)
     *          without requiring continuous GCS communication.
     * 
     * @note Called by check() when transitioning to STATE_DATA_LINK_LOSS. May be called
     *       repeatedly during datalink loss state.
     * 
     * @warning Mode change should be forced regardless of pilot input or RC state. This is
     *          a safety override to ensure failsafe mission execution. Ensure AUTO mode
     *          implementation includes appropriate safeguards.
     */
    virtual void set_mode_auto(void) = 0;

    /// Current AFS state machine state (PREFLIGHT, AUTO, DATA_LINK_LOSS, GPS_LOSS)
    enum state _state;

    // ========== Configuration Parameters (AP_Param) ==========
    // See AP_AdvancedFailsafe.cpp var_info[] for complete parameter definitions with
    // display names, descriptions, units, and valid ranges.

    /// AFS_ENABLE: Master enable/disable for entire AFS system (0=disabled, 1=enabled)
    AP_Int8 _enable;

    // ----- GPIO Pins for External Failsafe Board Communication -----

    /// AFS_HB_PIN: Digital output pin for 10Hz heartbeat signal (-1=disabled, or GPIO pin number)
    AP_Int8 _heartbeat_pin;

    /// AFS_MAN_PIN: Digital output pin set high when in manual/stabilized mode (-1=disabled)
    AP_Int8 _manual_pin;

    /// AFS_TERM_PIN: Digital output pin set high when termination triggered (-1=disabled)
    AP_Int8 _terminate_pin;

    // ----- Termination Control -----

    /// AFS_TERMINATE: Manual termination flag, can be set in flight to force termination (0=off, 1=terminate)
    AP_Int8 _terminate;

    /// AFS_TERM_ACTION: Termination behavior (0=external board handles, 42=motor cutoff, 43=land)
    AP_Int8 _terminate_action;

    // ----- Failsafe Waypoint Navigation -----

    /// AFS_WP_COMMS: Mission waypoint number to navigate to on GCS datalink loss (0=disabled)
    AP_Int8 _wp_comms_hold;

    /// AFS_WP_GPS_LOSS: Mission waypoint number to navigate to on GPS signal loss (0=disabled)
    AP_Int8 _wp_gps_loss;

    // ----- Altitude Limit Enforcement (AMSL = Above Mean Sea Level) -----

    /// AFS_QNH_PRESSURE: QNH pressure for barometric altitude calculation (millibars, 0=use GPS only)
    AP_Float _qnh_pressure;

    /// AFS_AMSL_LIMIT: Maximum allowed AMSL altitude in meters (0=disabled), enforced via QNH or GPS
    AP_Int32 _amsl_limit;

    /// AFS_AMSL_ERR_GPS: GPS altitude error margin subtracted from limit if barometer fails (meters, -1=terminate on baro failure)
    AP_Int32 _amsl_margin_gps;

    // ----- Communication Loss Timeouts -----

    /// AFS_RC_FAIL_TIME: RC receiver timeout in seconds before RC failure declared (0=disabled)
    AP_Float _rc_fail_time_seconds;

    /// AFS_GCS_FAIL_TIME: GCS MAVLink heartbeat timeout in seconds before datalink loss declared
    AP_Float _gcs_fail_time_seconds;

    // ----- Event Counting and Termination Triggers -----

    /// AFS_MAX_GPS_LOSS: Maximum number of GPS loss events before termination (0=disabled)
    AP_Int8  _max_gps_loss;

    /// AFS_MAX_COMMS_LOSS: Maximum number of GCS datalink loss events before termination (0=disabled)
    AP_Int8  _max_comms_loss;

    // ----- Feature Enables -----

    /// AFS_ENABLE_GEOFENCE_FS: Enable immediate termination on geofence breach (0=disabled, 1=enabled)
    AP_Int8  _enable_geofence_fs;

    /// AFS_ENABLE_RC_FS: Enable RC failure monitoring and termination (0=disabled, 1=enabled)
    AP_Int8  _enable_RC_fs;

    /// AFS_RC_TERM_MANUAL_ONLY: Only trigger RC termination when in manual mode (0=all modes, 1=manual only)
    AP_Int8  _rc_term_manual_only;

    /// AFS_ENABLE_DUAL_LOSS: Enable termination on simultaneous GPS + GCS loss (0=disabled, 1=enabled)
    AP_Int8  _enable_dual_loss;

    // ----- Range Limit Enforcement -----

    /// AFS_MAX_RANGE_KM: Maximum allowed range from first arming location in kilometers (0=disabled)
    AP_Int16  _max_range_km;

    // ========== Runtime State Variables (non-parameter) ==========

    /// Current state of heartbeat pin output (toggles at 10Hz for external failsafe board)
    bool _heartbeat_pin_value;

    /// Mission waypoint number saved before entering failsafe, for resuming mission after recovery
    uint8_t _saved_wp;
    
    /// Count of GPS loss events (30-second debounce, terminates if exceeds MAX_GPS_LOSS)
    uint8_t _gps_loss_count;

    /// Count of GCS datalink loss events (30-second debounce, terminates if exceeds MAX_COMMS_LOSS)
    uint8_t _comms_loss_count;

    /// Timestamp (ms) of most recent GCS datalink loss event (for 30-second debounce window)
    uint32_t _last_comms_loss_ms;

    /// Timestamp (ms) of most recent GPS loss event (for 30-second debounce window)
    uint32_t _last_gps_loss_ms;

    /// Flag indicating setup_IO_failsafe() has been called (prevents redundant configuration)
    bool _failsafe_setup:1;

    /// First GPS location recorded when entering AFS_AUTO mode (used for range limit checks)
    Location _first_location;

    /// Flag indicating _first_location has been recorded (set on first AUTO mode entry)
    bool _have_first_location;

    /// Timestamp (ms) of last range limit warning message (rate limiting for GCS notifications)
    uint32_t _term_range_notice_ms;

    /**
     * @brief Check if vehicle altitude exceeds configured AMSL limit
     * 
     * @return true if altitude limit exceeded (termination required), false otherwise
     * 
     * @details Checks current altitude against AFS_AMSL_LIMIT using QNH pressure altitude
     *          (if QNH configured and barometer healthy) or GPS altitude (with error margin
     *          if barometer failed). Returns false if altitude limit disabled (AMSL_LIMIT=0).
     * 
     * @note Called by check() during failure monitoring. Uses AP_Baro for pressure altitude
     *       and AP_GPS for GPS altitude fallback.
     */
    bool check_altlimit(void);

private:
    /// Singleton instance pointer (enforced via constructor panic on duplicate)
    static AP_AdvancedFailsafe *_singleton;

    /**
     * @brief Check if vehicle has exceeded maximum range from first arming location
     * 
     * @details Called by check() to enforce AFS_MAX_RANGE_KM limit. Calculates distance
     *          from _first_location (recorded on first AUTO mode entry) to current GPS
     *          position. If distance exceeds limit, termination is triggered. Generates
     *          periodic warning messages to GCS when approaching or exceeding range limit.
     * 
     * @note Requires valid GPS position and _have_first_location flag set. Does nothing
     *       if MAX_RANGE_KM is 0 (disabled) or first location not yet recorded.
     */
    void max_range_update();

    /// AFS_OPTIONS: Bitmask of optional behaviors (see Option enum)
    AP_Int16 options;

    /**
     * @enum Option
     * @brief Optional AFS behavior flags configured via AFS_OPTIONS bitmask parameter
     * 
     * @details Provides configurability for AFS recovery behavior and failure detection modes.
     * 
     *          CONTINUE_AFTER_RECOVERED (bit 0):
     *          When set, allows AFS to return from DATA_LINK_LOSS or GPS_LOSS states to
     *          STATE_AUTO after communication/GPS recovery, resuming normal mission execution.
     *          When clear, failsafe state persists until landing/termination even after recovery.
     * 
     *          GCS_FS_ALL_AUTONOMOUS_MODES (bit 1):
     *          When set, enables GCS failsafe monitoring in all autonomous modes (AUTO, GUIDED, RTL).
     *          When clear, GCS failsafe only triggers in AUTO mode, allowing GUIDED/RTL without
     *          requiring continuous GCS heartbeat.
     */
    enum class Option {
        CONTINUE_AFTER_RECOVERED = (1U<<0),        ///< Resume AUTO state after link recovery
        GCS_FS_ALL_AUTONOMOUS_MODES = (1U<<1),     ///< Monitor GCS link in all autonomous modes
    };

    /**
     * @brief Check if specific AFS option flag is enabled
     * 
     * @param[in] option Option flag to test
     * @return true if option bit is set in AFS_OPTIONS parameter, false otherwise
     */
    bool option_is_set(Option option) const {
        return (options.get() & int16_t(option)) != 0;
    }

    /**
     * @brief Check if GPS altitude is available and reliable for AMSL limit enforcement
     * 
     * @return true if GPS has valid altitude data suitable for altitude limit checking
     * 
     * @details Used by check_altlimit() to determine if GPS altitude can be used as fallback
     *          when barometer fails or QNH not configured. Checks GPS fix status and vertical
     *          accuracy estimates.
     */
    bool gps_altitude_ok() const;
};

/**
 * @namespace AP
 * @brief ArduPilot singleton accessor namespace
 */
namespace AP {
    /**
     * @brief Get singleton instance of AP_AdvancedFailsafe
     * 
     * @return Pointer to singleton AP_AdvancedFailsafe instance, or nullptr if not instantiated
     * 
     * @details Provides convenient namespace-based access to AFS singleton. Typical usage:
     *          ```cpp
     *          AP_AdvancedFailsafe *afs = AP::advancedfailsafe();
     *          if (afs != nullptr && afs->enabled()) {
     *              afs->check(last_valid_rc_ms);
     *          }
     *          ```
     * 
     * @note Preferred access method over direct AP_AdvancedFailsafe::get_singleton() call.
     *       Always check for nullptr before dereferencing in case AFS not compiled in or
     *       not instantiated by vehicle.
     */
    AP_AdvancedFailsafe *advancedfailsafe();
};

#endif  // AP_ADVANCEDFAILSAFE_ENABLED
