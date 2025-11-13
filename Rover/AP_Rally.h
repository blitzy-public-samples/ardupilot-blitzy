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
 * @file AP_Rally.h
 * @brief Rover-specific rally point interface extending base AP_Rally library
 * 
 * @details This file provides the rover-specific implementation of the rally point
 *          system, extending the base AP_Rally library with rover-appropriate
 *          validation and behavior. Rally points allow the rover to navigate to
 *          predefined safe locations and can be used for return-to-launch
 *          alternatives or as waypoint destinations.
 * 
 *          The rover implementation validates rally points based on ground vehicle
 *          constraints and integrates with the rover's navigation and home position
 *          management systems.
 * 
 * @note This file is conditionally compiled based on HAL_RALLY_ENABLED feature flag.
 *       Rally point support must be enabled in the board configuration for this
 *       functionality to be available.
 * 
 * @see libraries/AP_Rally/AP_Rally.h for base rally point functionality
 * @see Rover/mode.h for rover flight mode integration
 */

#pragma once

#include <AP_Rally/AP_Rally.h>

#if HAL_RALLY_ENABLED

/**
 * @class AP_Rally_Rover
 * @brief Rover-specific rally point management class
 * 
 * @details AP_Rally_Rover extends the base AP_Rally class to provide rally point
 *          support tailored for ground vehicles. Rally points are predefined
 *          geographic locations that serve as alternative safe destinations for
 *          return-to-launch operations or as navigation waypoints.
 * 
 *          This class inherits core rally point functionality from AP_Rally including:
 *          - get_rally_location_cm(): Retrieves rally point positions in centimeters
 *            from the EKF origin, used for navigation calculations
 *          - set_home_to_rally_location(): Sets the vehicle's home position to the
 *            nearest rally point, useful for mission operations without GPS home lock
 * 
 *          The rover implementation adds ground vehicle-specific validation through
 *          the is_valid() override, ensuring rally points meet rover operational
 *          constraints (e.g., accessible terrain, appropriate for wheeled navigation).
 * 
 *          Rally points are configured via MAVLink and stored in persistent memory,
 *          typically used in scenarios where:
 *          - Multiple safe return locations are needed
 *          - Home position is not ideal for landing/parking
 *          - Mission operations require flexible return destinations
 * 
 * @note Rally point support is optional and controlled by HAL_RALLY_ENABLED.
 *       Maximum number of rally points and storage is configured in AP_Rally.
 * 
 * @warning Rally points should be validated for rover accessibility. Points in
 *          inaccessible terrain (steep slopes, obstacles) may cause navigation
 *          failures or unsafe behavior.
 * 
 * @see AP_Rally base class in libraries/AP_Rally/AP_Rally.h
 * @see Rover mode implementations for rally point usage in RTL and guided modes
 */
class AP_Rally_Rover : public AP_Rally
{
public:
    /**
     * @brief Construct a new AP_Rally_Rover object
     * 
     * @details Initializes the rover-specific rally point manager by calling the
     *          base AP_Rally constructor. Rally point data is loaded from persistent
     *          storage during system initialization.
     */
    AP_Rally_Rover() : AP_Rally() { }

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Rally_Rover);

private:
    /**
     * @brief Validate a rally point for rover-specific operational constraints
     * 
     * @details This method overrides the base AP_Rally validation to apply
     *          rover-specific checks ensuring rally points are appropriate for
     *          ground vehicle navigation. Validation may include:
     *          - Geographic coordinate validity (lat/lon in valid ranges)
     *          - Altitude reasonableness for ground operations
     *          - Distance from current position or home
     *          - Terrain accessibility (if terrain data available)
     * 
     *          Invalid rally points are rejected during configuration and will not
     *          be stored or used for navigation. This prevents operators from
     *          accidentally setting inaccessible or inappropriate rally locations.
     * 
     * @param[in] rally_point Location structure containing the rally point coordinates
     *                        to validate (latitude, longitude, altitude)
     * 
     * @return true if the rally point meets rover operational requirements and
     *              can be safely used for navigation
     * @return false if the rally point is invalid, inaccessible, or inappropriate
     *               for rover operations
     * 
     * @note This method is called during rally point configuration via MAVLink
     *       and during mission upload operations
     * 
     * @see Location structure in libraries/AP_Common/Location.h
     * @see AP_Rally::is_valid() base class method
     */
    bool is_valid(const Location &rally_point) const override;
};

#endif  // HAL_RALLY_ENABLED
