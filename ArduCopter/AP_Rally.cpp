/**
 * @file AP_Rally.cpp
 * @brief Copter-specific rally point validation implementation
 * 
 * This file provides the ArduCopter-specific override of rally point validation
 * from the base AP_Rally library. Rally points are alternative landing locations
 * that can be used during Return-To-Launch (RTL) operations instead of returning
 * to the home position.
 * 
 * @details Rally Point System Overview:
 * 
 * Rally points serve as alternative safe landing locations for RTL operations.
 * When RTL is triggered (due to failsafe, user command, or mission completion),
 * the copter can choose to fly to the nearest rally point instead of home if:
 * - Rally points are enabled (RALLY_TOTAL > 0)
 * - A valid rally point exists within the configured limit (RALLY_LIMIT_KM)
 * - The selected rally point passes all validation checks
 * 
 * Copter-Specific Validation:
 * 
 * This file overrides the base AP_Rally::is_valid() method to add copter-specific
 * validation that ensures rally points are safe and reachable for multirotor aircraft.
 * The primary additional check is geofence validation - rally points must be within
 * any configured geofence boundaries to be considered valid.
 * 
 * Integration with RTL Mode:
 * 
 * During RTL operations, the flight controller:
 * 1. Calls AP_Rally::calc_best_rally_or_home_location() to select the best destination
 * 2. The base library evaluates all rally points considering distance and altitude
 * 3. Each candidate rally point is validated using this is_valid() override
 * 4. Invalid rally points are rejected; the copter uses the next best option or home
 * 5. The selected valid rally point becomes the RTL target destination
 * 
 * Rally Point Selection Criteria (implemented in base AP_Rally library):
 * - Distance from current position (prefer closer rally points)
 * - Required altitude gain (prefer rally points requiring less climb)
 * - Terrain clearance considerations
 * - Must be within RALLY_LIMIT_KM distance limit
 * - Must pass all validation checks (implemented here)
 * 
 * Safety Considerations:
 * 
 * Rally point validation is safety-critical. An invalid rally point could lead to:
 * - Flying outside geofence boundaries during RTL
 * - Attempting to land in an unreachable or unsafe location
 * - Geofence breach triggering additional failsafe actions
 * 
 * Therefore, conservative validation is essential - when in doubt, reject the rally point
 * and fall back to home position or the next best alternative.
 * 
 * @note This file only provides Copter-specific validation. Core rally point functionality
 *       (storage, retrieval, selection algorithms) is in libraries/AP_Rally/
 * 
 * @see libraries/AP_Rally/AP_Rally.h - Base rally point library
 * @see libraries/AP_Rally/AP_Rally.cpp - Rally point selection and storage
 * @see mode_rtl.cpp - RTL mode implementation that uses rally points
 * @see AC_Fence - Geofence library used for rally point validation
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_Rally/AP_Rally_config.h>

#if HAL_RALLY_ENABLED

#include <AP_Common/Location.h>

#include "Copter.h"

#include "AP_Rally.h"

/**
 * @brief Validate that a rally point is safe and reachable for copter operations
 * 
 * @details This method performs copter-specific validation of rally points beyond
 * the basic checks in the AP_Rally base class. Rally points must pass all validation
 * checks to be considered as potential RTL destinations.
 * 
 * Validation Checks Performed:
 * 
 * 1. **Geofence Validation** (if AP_FENCE_ENABLED):
 *    - Ensures the rally point location is within all configured geofence boundaries
 *    - Checks both inclusion fences (must be inside) and exclusion fences (must be outside)
 *    - Validates against cylindrical, polygon, and altitude fence types
 *    - Uses AC_Fence::check_destination_within_fence() for comprehensive validation
 * 
 * Validation Logic Flow:
 * 
 * - If geofence is enabled AND rally point is outside fence boundaries → INVALID
 * - If geofence is disabled OR rally point is within fence boundaries → VALID
 * - Conservative approach: reject if fence check fails (safer to use home position)
 * 
 * Rally Point Selection Context:
 * 
 * This validation is called by the base AP_Rally library during rally point selection,
 * typically when:
 * - RTL mode is initiated (user command, failsafe, mission end)
 * - calc_best_rally_or_home_location() evaluates potential rally destinations
 * - Each candidate rally point must pass validation before being considered
 * 
 * The selection algorithm (in AP_Rally base class) considers:
 * - Distance from current position (prefer closer points)
 * - Required altitude gain (prefer points requiring less climb)
 * - Terrain clearance and obstacle avoidance
 * - RALLY_LIMIT_KM distance restriction
 * - THIS validation check (is_valid override)
 * 
 * Integration with RTL Mode:
 * 
 * When RTL is triggered, the mode implementation:
 * 1. Requests best rally or home location from AP_Rally
 * 2. AP_Rally iterates through stored rally points
 * 3. For each rally point within RALLY_LIMIT_KM:
 *    a. Checks basic validity (altitude, coordinates)
 *    b. Calls THIS is_valid() method for vehicle-specific checks
 *    c. Evaluates distance and altitude cost
 * 4. Selects the best valid rally point or falls back to home
 * 5. RTL mode flies to the selected destination
 * 
 * Geofence Integration Details:
 * 
 * The geofence check ensures rally points respect all fence types:
 * - **Inclusion Fences**: Rally point must be inside (e.g., max radius from home)
 * - **Exclusion Fences**: Rally point must be outside (e.g., no-fly zones)
 * - **Altitude Fences**: Rally point altitude must be within limits
 * - **Polygon Fences**: Rally point must be inside/outside polygon boundaries
 * 
 * This prevents scenarios where:
 * - RTL would breach a circular fence while flying to rally point
 * - Rally point is in a no-fly zone (exclusion fence)
 * - Rally point altitude violates altitude limits
 * - Rally point is outside mission operation area
 * 
 * @param[in] rally_point Location to validate (lat/lon/alt in Location format)
 *                        - Latitude in degrees * 1e7
 *                        - Longitude in degrees * 1e7
 *                        - Altitude in centimeters (frame depends on rally point configuration)
 * 
 * @return true if rally point passes all copter-specific validation checks and is
 *              safe to use as an RTL destination
 * @return false if rally point fails any validation check:
 *               - Outside geofence boundaries (if fence enabled)
 *               - Not safe or reachable for copter operations
 * 
 * @note This is called frequently during rally point selection, especially during
 *       RTL initialization. The implementation must be efficient.
 * 
 * @note The base class AP_Rally::is_valid() always returns true. This override adds
 *       copter-specific safety checks that other vehicles (plane, rover) may not need.
 * 
 * @warning Rejecting a valid rally point is safer than accepting an invalid one.
 *          If validation fails, the copter will use home position or another rally point.
 *          If an invalid rally point is accepted, the copter could breach geofences
 *          or attempt to land in an unsafe location.
 * 
 * @warning Rally point validation happens BEFORE the copter starts flying to the point.
 *          This method does NOT check dynamic conditions (battery, weather, obstacles)
 *          that may change during flight. It only validates static geographic constraints.
 * 
 * @see AP_Rally::calc_best_rally_or_home_location() - Main rally selection algorithm
 * @see AC_Fence::check_destination_within_fence() - Geofence validation method
 * @see mode_rtl.cpp - RTL mode that uses rally point selection
 * @see RallyLocation - Rally point data structure with lat/lon/alt/flags
 * 
 * Source: ArduCopter/AP_Rally.cpp:26-34
 */
bool AP_Rally_Copter::is_valid(const Location &rally_point) const
{
#if AP_FENCE_ENABLED
    // Validate rally point is within all configured geofence boundaries
    // This check includes:
    // - Circular/cylindrical fences (max distance from home)
    // - Polygon fences (arbitrary boundary shapes)
    // - Altitude fences (min/max altitude limits)
    // - Inclusion fences (must be inside) and exclusion fences (must be outside)
    if (!copter.fence.check_destination_within_fence(rally_point)) {
        // Rally point is outside fence boundaries - REJECT as invalid
        // This prevents RTL from breaching geofence when flying to rally point
        // Copter will use home position or next best rally point instead
        return false;
    }
#endif
    
    // All copter-specific validation checks passed
    // Rally point is geographically safe and reachable
    // Note: This doesn't validate dynamic conditions (battery, weather, etc.)
    // that may affect the actual RTL flight to this rally point
    return true;
}

#endif  // HAL_RALLY_ENABLED
