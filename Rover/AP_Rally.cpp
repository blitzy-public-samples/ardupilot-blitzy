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
 * @file AP_Rally.cpp
 * @brief Implementation of rover-specific rally point functionality
 * 
 * @details This file provides the rover-specific implementation of the AP_Rally
 *          interface. Rally points are safe locations that the vehicle can navigate
 *          to during return-to-launch (RTL) operations or when other failsafe
 *          conditions are triggered.
 *          
 *          For ground vehicles (rovers), rally points must satisfy additional
 *          constraints compared to aerial vehicles:
 *          - Rally points must be within the configured geofence boundaries to
 *            prevent the vehicle from attempting to navigate through obstacles
 *            or restricted areas
 *          - Rally points should be accessible via ground navigation paths
 *          
 *          The rover implementation extends the base AP_Rally class to add
 *          geofence validation, ensuring that rally points are safe destinations
 *          for a ground vehicle to reach.
 *          
 * @note Rally point coordinates are stored in the Location format, which uses
 *       integer latitude/longitude in degrees * 1e7 and altitude in centimeters
 *       relative to the home altitude reference.
 *       
 * @see AP_Rally (base class in libraries/AP_Rally/)
 * @see AC_Fence (geofencing system used for validation)
 * @see Location (coordinate representation in AP_Common/)
 * 
 * Source: Rover/AP_Rally.cpp
 */

#include "AP_Rally.h"

#if HAL_RALLY_ENABLED

#include "Rover.h"

#include <AP_Common/Location.h>

/**
 * @brief Validate a rally point for rover-specific constraints
 * 
 * @details This method checks whether a given rally point is valid and safe for
 *          use by a ground vehicle. The validation ensures that the rally point
 *          is located within the configured geofence boundaries, preventing the
 *          vehicle from attempting to navigate to restricted or inaccessible areas.
 *          
 *          Validation process:
 *          1. If geofencing is enabled (AP_FENCE_ENABLED), verify the rally point
 *             is within all fence boundaries (circular, polygon, altitude, etc.)
 *          2. If geofencing is disabled, accept all rally points
 *          
 *          This override of the base AP_Rally::is_valid() method adds ground-vehicle
 *          specific safety checks that aren't required for aerial vehicles.
 * 
 * @param[in] rally_point The rally point location to validate, in Location format
 *                        (latitude/longitude in degrees * 1e7, altitude in cm relative
 *                        to home altitude)
 * 
 * @return true if the rally point is valid and safe for rover use, false otherwise
 * 
 * @note This method is called during rally point upload from ground control station
 *       and during RTL operations when selecting a rally point destination
 *       
 * @warning A false return will prevent the rally point from being used, which may
 *          cause the vehicle to use home location instead or trigger other failsafe
 *          actions
 * 
 * @see AC_Fence::check_destination_within_fence() for geofence validation logic
 * @see AP_Rally::is_valid() base class method
 * 
 * Source: Rover/AP_Rally.cpp:24-32
 */
bool AP_Rally_Rover::is_valid(const Location &rally_point) const
{
#if AP_FENCE_ENABLED
    // Check if the rally point location is within the configured geofence boundaries.
    // This is critical for ground vehicles to ensure rally points are reachable without
    // driving through fenced-off areas, obstacles, or restricted zones.
    // The fence check validates against all enabled fence types: circular, polygon,
    // altitude min/max, and inclusion/exclusion zones.
    if (!rover.fence.check_destination_within_fence(rally_point)) {
        // Rally point is outside geofence boundaries - reject it as unsafe
        return false;
    }
#endif
    // Rally point passed all validation checks (or geofencing is disabled)
    return true;
}

#endif  // HAL_RALLY_ENABLED
