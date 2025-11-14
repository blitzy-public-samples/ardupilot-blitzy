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
 * @file afs_copter.h
 * @brief Advanced Failsafe (AFS) system implementation for ArduCopter
 * 
 * @details This file implements the ArduCopter-specific advanced failsafe functionality,
 *          providing hardware-enforced motor termination capabilities for catastrophic
 *          failure scenarios that exceed normal failsafe recovery thresholds.
 * 
 *          The Advanced Failsafe system is a last-resort safety mechanism that uses
 *          external hardware pins to force motor termination independent of software
 *          control. This hardware-level safety net ensures vehicle shutdown even if
 *          the flight control software becomes unresponsive or enters an unrecoverable
 *          state.
 * 
 *          **Trigger Conditions:**
 *          - Sustained GPS loss beyond recovery timeout thresholds
 *          - Ground Control Station (GCS) communication loss for extended periods
 *          - Geofence breach beyond configurable recovery distances
 *          - Other catastrophic system failures defined in AP_AdvancedFailsafe
 * 
 *          **Hardware Integration:**
 *          The AFS system interfaces with external hardware termination pins on
 *          supported flight controllers. When triggered, these pins force immediate
 *          motor shutdown at the hardware level, bypassing all software control paths.
 *          This ensures termination even if the main CPU is hung or the flight stack
 *          is non-responsive.
 * 
 *          **Safety Philosophy:**
 *          AFS is designed as a last line of defense when normal failsafes (battery
 *          failsafe, radio failsafe, GCS failsafe, fence failsafe) cannot safely
 *          recover the vehicle. It prioritizes preventing flyaways and uncontrolled
 *          flight over vehicle recovery.
 * 
 * @note This feature must be explicitly enabled via AP_COPTER_ADVANCED_FAILSAFE_ENABLED
 * @warning AFS termination results in immediate loss of vehicle control and crash.
 *          Configure thresholds carefully to avoid false triggers during normal operations.
 * @warning Hardware termination pins must be properly configured in board hwdef files
 * 
 * @see libraries/AP_AdvancedFailsafe/AP_AdvancedFailsafe.h for base implementation
 * @see ArduCopter/failsafe.cpp for normal failsafe mechanisms
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "config.h"

#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
#include <AP_AdvancedFailsafe/AP_AdvancedFailsafe.h>

/**
 * @class AP_AdvancedFailsafe_Copter
 * @brief Multicopter-specific implementation of the Advanced Failsafe system
 * 
 * @details This class extends the base AP_AdvancedFailsafe to provide ArduCopter-specific
 *          termination behavior and failsafe configuration for multirotor aircraft. It
 *          implements hardware-enforced safety mechanisms that can force motor shutdown
 *          when catastrophic failure conditions persist beyond normal failsafe recovery.
 * 
 *          **Architecture and Design:**
 *          The class overrides key virtual methods from AP_AdvancedFailsafe to customize
 *          behavior for multicopter flight characteristics, including immediate motor
 *          termination (vs. controlled descent in fixed-wing), mode switching logic for
 *          datalink loss recovery attempts, and IO failsafe configuration for hardware
 *          watchdog systems.
 * 
 *          **Failure Detection and Response:**
 *          The base AP_AdvancedFailsafe class continuously monitors critical system health
 *          indicators (GPS status, GCS communication, geofence state). When thresholds are
 *          exceeded, this copter-specific implementation executes appropriate termination
 *          sequences optimized for multirotor dynamics.
 * 
 *          **Termination Sequence:**
 *          1. Detect sustained failure condition (GPS loss, GCS loss, fence breach)
 *          2. Attempt mode change to AUTO for recovery (set_mode_auto)
 *          3. If recovery fails or condition persists, trigger termination
 *          4. Execute vehicle-specific termination (disarm motors, hardware cutoff)
 *          5. Assert hardware termination pins for failsafe IO state
 * 
 *          **Hardware Requirements:**
 *          - Flight controller with AFS-capable IO processor (FMU + IO architecture)
 *          - Properly configured termination pins in board hardware definition
 *          - External hardware monitoring capabilities (optional but recommended)
 * 
 *          **Configuration Parameters:**
 *          AFS parameters (defined in AP_AdvancedFailsafe base class) control:
 *          - Enable/disable state (AFS_ENABLE)
 *          - Termination action selection (AFS_TERM_ACTION)
 *          - GPS loss timeout threshold (AFS_GEOFENCE, AFS_RC, AFS_GCS)
 *          - Manual recovery pin configuration (AFS_MAN_PIN)
 * 
 *          **Integration with Normal Failsafes:**
 *          AFS operates in parallel with standard ArduCopter failsafes but at higher
 *          severity thresholds. Normal failsafes (battery, radio, GCS) attempt recovery
 *          actions (RTL, LAND). AFS only activates when these recovery attempts fail or
 *          when the failure is so severe that controlled recovery is impossible.
 * 
 *          **Safety Considerations:**
 *          - AFS termination results in immediate vehicle crash from current altitude
 *          - False triggers can result from temporary GPS outages in urban environments
 *          - Hardware termination bypasses all software safety checks
 *          - Recovery from AFS termination requires manual intervention
 *          - Test AFS configuration thoroughly in SITL before flight testing
 * 
 *          **Testing and Validation:**
 *          Use SITL simulation to validate AFS behavior:
 *          ```
 *          # Enable AFS in SITL
 *          param set AFS_ENABLE 1
 *          # Simulate GPS failure
 *          param set SIM_GPS_DISABLE 1
 *          # Monitor for AFS activation after timeout
 *          ```
 * 
 * @note AFS is disabled by default and must be explicitly enabled via parameters
 * @note Hardware termination requires IO coprocessor; not available on all boards
 * @note Flight controllers without IO will use software-only disarm
 * 
 * @warning AFS termination causes immediate vehicle crash - use only in safety-critical scenarios
 * @warning Configure conservative thresholds to prevent false triggers
 * @warning Test thoroughly in simulation before enabling on physical vehicles
 * @warning Ensure adequate altitude and clear landing area when testing AFS
 * @warning AFS may trigger in GPS-denied environments (tunnels, urban canyons, indoor flight)
 * 
 * @see AP_AdvancedFailsafe base class for core AFS logic and monitoring
 * @see ArduCopter/failsafe.cpp for standard failsafe implementations
 * @see libraries/AP_HAL/AP_HAL_Boards.h for board-specific AFS capabilities
 * 
 * Source: ArduCopter/afs_copter.h
 */
class AP_AdvancedFailsafe_Copter : public AP_AdvancedFailsafe
{
public:

    /**
     * @brief Inherit constructor from base AP_AdvancedFailsafe class
     * 
     * @details Uses the base class constructor to initialize AFS monitoring systems,
     *          parameter storage, and hardware interface connections. The copter-specific
     *          implementation does not require additional initialization beyond the base
     *          class setup.
     */
    using AP_AdvancedFailsafe::AP_AdvancedFailsafe;

    /**
     * @brief Force vehicle into terminal state with immediate motor shutdown
     * 
     * @details This method is called by the AFS framework when catastrophic failure
     *          conditions require immediate vehicle termination. It implements multicopter-
     *          specific termination behavior, which differs from fixed-wing termination
     *          by immediately disarming motors rather than attempting controlled descent.
     * 
     *          **Termination Actions:**
     *          1. Disarm motors immediately (cut all motor outputs to zero)
     *          2. Set all servo/motor channels to failsafe PWM values
     *          3. Disable attitude and position controllers
     *          4. Log termination event to dataflash
     *          5. Assert hardware termination pins if available
     *          6. Set vehicle status flags to indicate AFS termination state
     * 
     *          **Hardware Integration:**
     *          On flight controllers with IO coprocessors (FMU + IO architecture), this
     *          method commands both the FMU and IO processor to enter termination state,
     *          ensuring motor cutoff even if the main processor subsequently hangs.
     * 
     *          **Post-Termination State:**
     *          Once terminated, the vehicle cannot be rearmed without power cycle or manual
     *          intervention via AFS recovery pin (if configured). This prevents automatic
     *          recovery from AFS termination events.
     * 
     *          **Logging:**
     *          Termination events are logged to dataflash with AFS error codes indicating
     *          the trigger condition (GPS loss, GCS loss, fence breach).
     * 
     * @note This method is called from the main scheduler loop when AFS thresholds exceeded
     * @note Motor termination is immediate with no controlled descent attempt
     * @note Vehicle will fall from current altitude after termination
     * 
     * @warning Results in immediate vehicle crash - all motor control is lost
     * @warning Cannot be reversed without power cycle or manual recovery pin
     * @warning Logs may be corrupted if termination occurs during log write
     * 
     * @see setup_IO_failsafe() for configuration of hardware failsafe outputs
     * @see libraries/AP_AdvancedFailsafe/AP_AdvancedFailsafe.cpp for trigger logic
     */
    void terminate_vehicle(void) override;
    
protected:
    /**
     * @brief Configure IO processor failsafe outputs for FMU watchdog protection
     * 
     * @details This method configures the IO coprocessor's failsafe output values that
     *          will be automatically applied if the main FMU processor stops sending
     *          updates. This provides hardware-level protection against FMU hangs or
     *          crashes, ensuring motors are cut even if the main flight code freezes.
     * 
     *          **Hardware Architecture:**
     *          On FMU + IO flight controllers (Pixhawk, Cube, etc.), the IO coprocessor
     *          monitors the FMU heartbeat. If the FMU stops updating the IO processor
     *          (indicating a hang or crash), the IO automatically switches outputs to
     *          pre-configured failsafe values set by this method.
     * 
     *          **Configuration Actions:**
     *          1. Set all motor channels to minimum throttle (motors off)
     *          2. Configure servo channels to safe positions (control surfaces neutral)
     *          3. Enable IO watchdog monitoring
     *          4. Set failsafe detection timeout (typically 100-200ms)
     *          5. Configure manual override capabilities if AFS_MAN_PIN set
     * 
     *          **Failsafe PWM Values:**
     *          - Motors: Set to motor minimum PWM (typically 1000μs) to ensure disarm
     *          - Servos: Set to trim/neutral positions to minimize aerodynamic forces
     *          - Parachute/termination outputs: Set to deployment values if configured
     * 
     *          **Watchdog Behavior:**
     *          The IO watchdog triggers if no FMU update received within timeout period.
     *          This catches software deadlocks, infinite loops, hard faults, and other
     *          failure modes where the FMU becomes unresponsive but power remains.
     * 
     *          **Board Compatibility:**
     *          Only flight controllers with separate IO coprocessors support this feature.
     *          Single-chip boards without IO will execute software disarm only.
     * 
     * @note Called during AFS initialization and when failsafe parameters change
     * @note IO failsafe is independent of software failsafe mechanisms
     * @note Requires FMU + IO board architecture (not available on all flight controllers)
     * 
     * @warning IO failsafe triggers on any FMU hang, including benign software delays
     * @warning Conservative timeout values can cause false triggers during high CPU load
     * @warning Test IO failsafe thoroughly on bench before flight
     * 
     * @see libraries/AP_HAL/AP_HAL_IO.h for IO coprocessor interface
     * @see terminate_vehicle() for software-initiated termination sequence
     */
    void setup_IO_failsafe(void) override;

    /**
     * @brief Map current ArduCopter flight mode to AFS control mode enumeration
     * 
     * @details Translates the vehicle's current control mode (STABILIZE, ALT_HOLD, LOITER,
     *          AUTO, etc.) into the generic control_mode enum used by the AFS framework
     *          for monitoring and decision making. The AFS system uses this mapping to
     *          determine appropriate failsafe responses based on current flight mode.
     * 
     *          **Control Mode Categories:**
     *          The AFS framework classifies modes into categories:
     *          - MANUAL: Direct pilot control (STABILIZE, ACRO, ALT_HOLD)
     *          - AUTO: Autonomous navigation (AUTO, GUIDED, RTL, LAND)
     *          - UNKNOWN: Unrecognized or transitional states
     * 
     *          **Usage in AFS Logic:**
     *          The AFS system adjusts termination thresholds based on control mode:
     *          - MANUAL modes: Allow longer GCS loss timeout (pilot still has control)
     *          - AUTO modes: Shorter GCS loss timeout (no pilot intervention available)
     *          - GPS loss: Critical in AUTO, less critical in MANUAL stabilized modes
     * 
     *          **Mode Transition Handling:**
     *          Called frequently (typically at 50Hz) to track mode changes. The AFS system
     *          monitors for successful mode transitions during recovery attempts.
     * 
     * @return control_mode enum value representing current copter mode category
     *         - AFS_MANUAL: Manual control modes with pilot input
     *         - AFS_AUTO: Autonomous or automated flight modes
     *         - AFS_UNKNOWN: Undefined or error state
     * 
     * @note Called at scheduler update rate to track mode changes
     * @note Mode mapping affects AFS timeout thresholds and recovery strategies
     * 
     * @see set_mode_auto() for AFS-initiated mode change to autonomous recovery
     * @see ArduCopter/mode.h for copter flight mode definitions
     */
    enum control_mode afs_mode(void) override;

    /**
     * @brief Attempt autonomous recovery by forcing transition to AUTO mode
     * 
     * @details When AFS detects datalink loss (GCS communication failure), this method
     *          attempts to recover by switching the vehicle to AUTO mode to execute the
     *          pre-programmed mission or RTL failsafe. This provides one recovery attempt
     *          before resorting to termination.
     * 
     *          **Recovery Strategy:**
     *          The AFS system provides a escalating response to datalink loss:
     *          1. Initial GCS loss: Continue current mode (may be temporary)
     *          2. Sustained loss (first threshold): Call this method to switch to AUTO
     *          3. Extended loss (second threshold): Execute termination if AUTO fails
     * 
     *          **AUTO Mode Behavior:**
     *          When forced to AUTO mode:
     *          - If mission loaded: Continue/resume mission execution
     *          - If no mission: Execute RTL (Return to Launch) failsafe
     *          - If RTL unavailable: Execute LAND at current position
     * 
     *          **Success Criteria:**
     *          The mode change is considered successful if:
     *          - Vehicle transitions to AUTO mode
     *          - Position control is active and stable
     *          - Mission or RTL is executing normally
     *          
     *          **Failure Conditions:**
     *          Mode change fails if:
     *          - GPS lock is lost (cannot navigate in AUTO)
     *          - Battery critically low (insufficient for RTL)
     *          - Arming checks fail
     *          - Mode switch is blocked by flight mode restrictions
     * 
     *          **Post-Recovery Monitoring:**
     *          Even after successful mode change, AFS continues monitoring. If datalink
     *          remains lost and GPS subsequently fails, termination may still be triggered.
     * 
     * @note This is called automatically by AFS framework when GCS loss threshold reached
     * @note Mode change attempt is logged to dataflash for post-flight analysis
     * @note May not succeed if GPS unavailable or other arming checks fail
     * 
     * @warning AUTO mode requires GPS lock - will fail in GPS-denied environment
     * @warning If mode change fails, AFS proceeds to termination on next timeout
     * @warning Mission must be pre-loaded for AUTO mode recovery to be effective
     * 
     * @see afs_mode() for mode classification logic
     * @see ArduCopter/mode_auto.cpp for AUTO mode implementation
     * @see ArduCopter/failsafe.cpp for standard GCS failsafe behavior
     */
    void set_mode_auto(void) override;
};

#endif // AP_COPTER_ADVANCED_FAILSAFE_ENABLED

