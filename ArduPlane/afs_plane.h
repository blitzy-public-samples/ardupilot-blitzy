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
 * @file afs_plane.h
 * @brief Advanced Failsafe (AFS) implementation for fixed-wing aircraft
 * 
 * @details This file defines the plane-specific implementation of the Advanced
 *          Failsafe system, which provides hardware-level termination capabilities
 *          for fixed-wing aircraft in case of complete system failure or loss of
 *          control. The AFS system is designed to meet safety requirements for
 *          operations in controlled airspace or over populated areas.
 * 
 *          The plane-specific implementation handles:
 *          - Emergency vehicle termination with appropriate control surface positions
 *          - IO failsafe configuration for fixed-wing flight characteristics
 *          - Automatic mode switching to AUTO mode on datalink loss
 *          - Control mode mapping for AFS states
 * 
 * @note This functionality is only available when AP_ADVANCEDFAILSAFE_ENABLED is defined
 * @warning AFS is a safety-critical system - improper configuration can result in
 *          loss of vehicle control or unintended termination
 * 
 * @see libraries/AP_AdvancedFailsafe/AP_AdvancedFailsafe.h for base implementation
 */
#pragma once

#if AP_ADVANCEDFAILSAFE_ENABLED
#include <AP_AdvancedFailsafe/AP_AdvancedFailsafe.h>

/**
 * @class AP_AdvancedFailsafe_Plane
 * @brief Fixed-wing aircraft implementation of Advanced Failsafe system
 * 
 * @details This class extends the base AP_AdvancedFailsafe implementation with
 *          plane-specific behavior for emergency termination and failsafe handling.
 *          The Advanced Failsafe system provides hardware-level safety mechanisms
 *          that operate independently of the main flight control software.
 * 
 *          Key plane-specific behaviors:
 *          - Termination: Configures control surfaces for controlled descent
 *          - IO Failsafe: Sets appropriate failsafe values for fixed-wing flight
 *          - Mode Management: Automatically enters AUTO mode on datalink loss
 *          - AFS Mode Mapping: Maps current flight mode to AFS control mode enum
 * 
 *          The AFS system monitors critical parameters including:
 *          - GPS status and position accuracy
 *          - Geofence violations
 *          - Datalink connectivity
 *          - Flight termination commands from ground control
 * 
 * @note The AFS system requires specific hardware support (typically IOMCU-equipped
 *       flight controllers) and must be enabled via parameters
 * 
 * @warning This is safety-critical code. Changes must be thoroughly tested and
 *          reviewed. Improper termination configuration can result in uncontrolled
 *          descent or vehicle loss.
 * 
 * @see AP_AdvancedFailsafe for base class implementation and parameter definitions
 * @see libraries/AP_AdvancedFailsafe/AP_AdvancedFailsafe.cpp for core AFS logic
 */
class AP_AdvancedFailsafe_Plane : public AP_AdvancedFailsafe
{
public:

    using AP_AdvancedFailsafe::AP_AdvancedFailsafe;

    /**
     * @brief Execute emergency vehicle termination for fixed-wing aircraft
     * 
     * @details This method is called when the AFS system determines that the vehicle
     *          must be terminated. For fixed-wing aircraft, termination involves:
     *          - Disabling the throttle (zero thrust)
     *          - Commanding control surfaces to induce a controlled descent
     *          - Disarming the vehicle to prevent motor restart
     *          - Setting all outputs to predetermined safe states
     * 
     *          Termination is triggered by conditions such as:
     *          - Geofence breach beyond recovery limits
     *          - Complete GPS failure with no recovery
     *          - Manual termination command from ground control
     *          - Critical system failures detected by AFS monitoring
     * 
     *          This override provides plane-specific termination behavior that differs
     *          from multicopter (which may maintain some lift) or ground vehicles.
     * 
     * @note This method is called once when termination is triggered and sets permanent
     *       failsafe states. Recovery requires vehicle power cycle.
     * 
     * @warning This is a safety-critical function. Once called, vehicle control is
     *          lost and the aircraft will descend. This action is IRREVERSIBLE during
     *          the flight. Test termination configuration thoroughly in simulation.
     * 
     * @see AP_AdvancedFailsafe::check() for termination trigger conditions
     * @see Plane::set_servos() for output application
     */
    void terminate_vehicle(void) override;
    
protected:
    /**
     * @brief Configure IO controller failsafe values for fixed-wing aircraft
     * 
     * @details This method configures the IOMCU (IO microcontroller) with failsafe
     *          output values that will be applied if the main FMU (Flight Management
     *          Unit) firmware stops running or loses communication with the IOMCU.
     *          This provides a hardware-level safety backup independent of the main
     *          flight control software.
     * 
     *          For fixed-wing aircraft, the IO failsafe configuration typically:
     *          - Sets throttle to zero or idle
     *          - Commands control surfaces for stable glide or controlled descent
     *          - Maintains neutral or safe positions for other outputs
     * 
     *          The IOMCU will automatically apply these values if it doesn't receive
     *          updates from the FMU within the configured timeout period.
     * 
     * @note This method is called during AFS initialization and whenever failsafe
     *       parameters are updated. Requires hardware with IOMCU support.
     * 
     * @warning Incorrect IO failsafe configuration can result in loss of control
     *          if FMU communication fails. Test failsafe behavior thoroughly.
     * 
     * @see AP_AdvancedFailsafe::setup_IO_failsafe() for base implementation
     * @see libraries/AP_HAL_ChibiOS/RCOutput.cpp for IOMCU communication
     */
    void setup_IO_failsafe(void) override;

    /**
     * @brief Map current plane flight mode to AFS control mode enumeration
     * 
     * @details The AFS system uses a simplified control_mode enumeration to track
     *          the general category of the current flight mode. This method translates
     *          the plane-specific flight mode into the AFS control mode categories:
     *          - AFS_MANUAL: Manual control modes (MANUAL, STABILIZE, etc.)
     *          - AFS_AUTO: Autonomous modes (AUTO, GUIDED, RTL, etc.)
     *          - AFS_STABILIZED: Stabilized modes with manual input
     * 
     *          The AFS system uses this information to:
     *          - Determine appropriate failsafe responses
     *          - Track mode transitions for datalink loss recovery
     *          - Validate that mode changes are safe given current AFS state
     * 
     * @return enum control_mode The AFS control mode corresponding to current flight mode
     * 
     * @note This method is called frequently during AFS monitoring. Implementation
     *       should be efficient with no complex calculations.
     * 
     * @see AP_AdvancedFailsafe::control_mode for enumeration definition
     * @see Plane::control_mode for plane-specific flight mode enumeration
     */
    enum control_mode afs_mode(void) override;

    /**
     * @brief Force vehicle into AUTO flight mode due to datalink loss
     * 
     * @details This method is called by the AFS system when a datalink loss is detected
     *          and the AFS configuration requires automatic entry into AUTO mode for
     *          autonomous recovery. In AUTO mode, the plane will:
     *          - Continue executing the current mission if one is loaded
     *          - Return to home (RTL) if no mission is available
     *          - Execute any configured failsafe actions
     * 
     *          Forcing AUTO mode on datalink loss allows the aircraft to continue
     *          autonomously, potentially reestablishing communications or executing
     *          a safe landing sequence without pilot intervention.
     * 
     *          This method bypasses normal mode change restrictions and safety checks
     *          that would prevent automatic mode changes, as it is executed under
     *          AFS emergency authority.
     * 
     * @note This method is only called when AFS_ENABLE parameter is set and datalink
     *       loss is detected. The vehicle must have a valid mission or rally point
     *       for meaningful autonomous operation.
     * 
     * @warning Automatic mode changes during datalink loss can result in unexpected
     *          vehicle behavior if missions are not properly configured. Ensure mission
     *          plans include appropriate failsafe actions.
     * 
     * @see AP_AdvancedFailsafe::check_datalink_loss() for datalink monitoring
     * @see Plane::set_mode() for mode change implementation
     */
    void set_mode_auto(void) override;
};

#endif // AP_ADVANCEDFAILSAFE_ENABLED
