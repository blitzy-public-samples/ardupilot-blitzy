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
#pragma once

/**
 * @file afs_rover.h
 * @brief Rover-specific Advanced Failsafe (AFS) implementation
 * 
 * @details This file implements the rover-specific extension of the Advanced
 *          Failsafe system, which provides hardware-level termination capabilities
 *          for ground vehicles in emergency situations. The Advanced Failsafe system
 *          is an optional safety feature that can force the vehicle into a safe
 *          termination state when critical failures are detected.
 * 
 * @note This feature is conditionally compiled and only available when
 *       AP_ROVER_ADVANCED_FAILSAFE_ENABLED is defined. Most rover configurations
 *       do not require this level of failsafe protection.
 * 
 * @warning Advanced Failsafe termination is irreversible and will cut all motor
 *          outputs. This is intended for emergency use only when vehicle recovery
 *          is not possible and termination is the safest option.
 * 
 * Source: Rover/afs_rover.h
 */

#if AP_ROVER_ADVANCED_FAILSAFE_ENABLED
#include <AP_AdvancedFailsafe/AP_AdvancedFailsafe.h>

/**
 * @class AP_AdvancedFailsafe_Rover
 * @brief Rover-specific implementation of the Advanced Failsafe system
 * 
 * @details This class extends the base AP_AdvancedFailsafe functionality with
 *          rover-specific behaviors for emergency termination and failsafe mode
 *          transitions. It provides:
 *          - Vehicle termination (cutting all motor outputs)
 *          - Automatic mode switching on datalink loss
 *          - AFS mode mapping for rover control modes
 * 
 *          The Advanced Failsafe system monitors critical datalink health and can
 *          trigger emergency termination when configured thresholds are exceeded.
 *          This is typically used in high-security applications where vehicle
 *          flyaway/driveaway must be prevented at all costs.
 * 
 * @note Setup of failsafe output values is handled by the motors library, so
 *       setup_IO_failsafe() is a no-op for rovers.
 * 
 * @warning Termination is permanent for the current power cycle. Once triggered,
 *          the vehicle will not respond to control inputs until power cycled.
 *          Use this feature only when required by operational safety requirements.
 * 
 * @see AP_AdvancedFailsafe base class for common AFS functionality
 * @see libraries/AP_AdvancedFailsafe/AP_AdvancedFailsafe.h for AFS architecture
 */
class AP_AdvancedFailsafe_Rover : public AP_AdvancedFailsafe
{
public:
    using AP_AdvancedFailsafe::AP_AdvancedFailsafe;

    /**
     * @brief Set all outputs to termination state
     * 
     * @details Called by the AFS system when termination conditions are met.
     *          For rovers, this immediately cuts throttle and steering outputs,
     *          bringing the vehicle to a stop and preventing further movement.
     *          This override implements the rover-specific termination sequence.
     * 
     * @warning This action is irreversible until power cycle. The vehicle will
     *          remain in terminated state regardless of RC or GCS commands.
     */
    void terminate_vehicle(void) override;

protected:
    /**
     * @brief Setup failsafe values for I/O outputs (no-op for rovers)
     * 
     * @details This method is required by the AP_AdvancedFailsafe interface but
     *          is not needed for rovers because failsafe output configuration is
     *          handled entirely by the motors library. The empty implementation
     *          satisfies the interface contract without duplicating functionality.
     * 
     * @note For rovers, motor failsafe behavior is configured through the
     *       SRV_Channel and motor control libraries.
     */
    void setup_IO_failsafe(void) override {}

    /**
     * @brief Return the AFS mapped control mode
     * 
     * @details Translates the current rover control mode into the standardized
     *          AFS control_mode enumeration. This allows the base AFS system to
     *          understand rover-specific modes and apply appropriate failsafe
     *          logic based on the current vehicle state.
     * 
     * @return enum control_mode The AFS-standard control mode corresponding to
     *         the current rover mode (e.g., MANUAL, AUTO, GUIDED)
     * 
     * @note This mapping is necessary because different vehicle types have
     *       different mode definitions, and AFS needs a common representation.
     */
    enum control_mode afs_mode(void) override;

    /**
     * @brief Force vehicle into AUTO mode on datalink loss
     * 
     * @details Called by the AFS system when datalink loss is detected and the
     *          configured response is to enter autonomous mode. This allows the
     *          rover to continue its mission autonomously or execute a predefined
     *          failsafe mission when communication with the ground station is lost.
     * 
     * @note This is called automatically by the AFS system based on datalink
     *       health monitoring and configured AFS parameters. The mode change
     *       bypasses normal mode change restrictions to ensure failsafe execution.
     * 
     * @see Mode::set_mode() for normal mode change procedures
     */
    void set_mode_auto(void) override;
};

#endif  // ADVANCED_FAILSAFE
