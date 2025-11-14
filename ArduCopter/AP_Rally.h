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
 * @brief Copter-specific rally point system implementation
 * 
 * @details This file defines the multicopter-specific implementation of the rally point
 *          system. Rally points serve as alternative landing locations during RTL (Return
 *          To Launch) operations, providing safer landing options when the home location
 *          may be unsuitable due to obstacles, restricted airspace, or other hazards.
 *          
 *          Rally points enable the vehicle to land at pre-surveyed locations that offer:
 *          - Clear landing zones free from obstacles
 *          - Better GPS reception than the home location
 *          - Avoidance of restricted or hazardous areas near home
 *          - Strategic positioning for multi-vehicle operations
 *          
 *          The rally point system integrates with ArduCopter's RTL flight mode to provide
 *          intelligent selection of the safest and most accessible landing location based
 *          on current vehicle position, altitude, and configured rally point parameters.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 * 
 * @see libraries/AP_Rally/AP_Rally.h for base rally point implementation
 * @see ArduCopter/mode_rtl.cpp for RTL mode integration with rally points
 */

#pragma once

#include <AP_Rally/AP_Rally_config.h>

#if HAL_RALLY_ENABLED

#include <AP_Rally/AP_Rally.h>
#include <AP_AHRS/AP_AHRS.h>

/**
 * @class AP_Rally_Copter
 * @brief Multicopter-specific rally point management and validation
 * 
 * @details AP_Rally_Copter extends the base AP_Rally library to provide multicopter-specific
 *          rally point validation and behavior. Rally points are pre-defined GPS coordinates
 *          that serve as alternative landing locations during RTL operations, allowing the
 *          vehicle to land at safer locations than the original launch point.
 *          
 *          Rally Point System Overview:
 *          - Rally points are stored as Location objects containing latitude, longitude, and altitude
 *          - Multiple rally points can be defined (up to RALLY_TOTAL_MAX, typically 10)
 *          - Rally points can be uploaded via ground control station or MAVLink commands
 *          - Each rally point includes an altitude relative to home (ALT_BREAK parameter)
 *          
 *          Rally Point Selection Algorithm:
 *          When RTL mode is engaged with rally points enabled (RALLY_TOTAL > 0), the system
 *          selects the most appropriate rally point using the following criteria:
 *          
 *          1. Distance Consideration: Selects the closest rally point to the current position
 *             to minimize flight time and battery consumption during return
 *          
 *          2. Altitude Requirements: Ensures the selected rally point's altitude is appropriate
 *             for safe approach and landing based on terrain and obstacle clearance
 *          
 *          3. Validity Checks: Only rally points that pass is_valid() checks are considered,
 *             ensuring the location is within reasonable geographic bounds and has valid coordinates
 *          
 *          4. Bearing Optimization: When multiple rally points are equidistant, preference may
 *             be given to points requiring less course change from current heading
 *          
 *          Copter-Specific Rally Point Approach Procedures:
 *          
 *          1. Initial Climb: Vehicle climbs to RTL_ALT altitude if below this threshold,
 *             ensuring obstacle clearance during horizontal transit to rally point
 *          
 *          2. Horizontal Navigation: Vehicle flies directly to selected rally point using
 *             position controller, maintaining RTL_ALT altitude throughout transit
 *          
 *          3. Rally Point Arrival: Upon reaching the rally point horizontal position
 *             (within WPNAV_RADIUS), vehicle transitions to descent phase
 *          
 *          4. Controlled Descent: Vehicle descends at RTL_SPEED_DN (default 150 cm/s) while
 *             maintaining position over the rally point using GPS position hold
 *          
 *          5. Final Landing: At LAND_ALT_LOW altitude, vehicle switches to precision landing
 *             mode with reduced descent rate for safe touchdown
 *          
 *          6. Disarm Sequence: After touchdown detection, vehicle automatically disarms if
 *             configured (DISARM_DELAY parameter)
 *          
 *          Parameter Configuration:
 *          - RALLY_TOTAL: Number of rally points stored (0 = rally point system disabled)
 *          - RALLY_LIMIT_KM: Maximum distance from home for valid rally points (default 0.3 km)
 *          - RALLY_INCL_HOME: Include home position as a rally point option (0=no, 1=yes)
 *          
 *          Safety Considerations:
 *          - Rally points must be surveyed and verified as safe landing locations before use
 *          - Rally point altitude should account for terrain elevation at that location
 *          - Obstacles between current position and rally point are not detected automatically
 *          - GPS accuracy and HDOP should be verified at rally point locations before saving
 *          - Rally points should not be placed in restricted airspace or hazardous areas
 *          
 *          Integration with RTL Mode:
 *          The rally point system is automatically activated when RTL mode is engaged and
 *          RALLY_TOTAL > 0. The mode_rtl.cpp implementation queries AP_Rally_Copter to
 *          determine the target landing location and manages the approach sequence.
 *          
 *          Thread Safety:
 *          Rally point data is typically accessed from the main flight control thread during
 *          RTL mode execution. Rally point uploads via MAVLink are handled through the GCS
 *          interface and protected by the scheduler's thread-safe parameter system.
 * 
 * @note Rally points are most effective when placed in open areas with good GPS reception
 *       and minimal wind turbulence for stable landing
 * 
 * @warning Rally point altitude is relative to home position - ensure proper altitude configuration
 *          to avoid terrain collision, especially in areas with significant elevation changes
 * 
 * @warning Rally points do not provide obstacle avoidance during transit - clear flight path
 *          must be ensured through proper RTL_ALT configuration and mission planning
 * 
 * @see libraries/AP_Rally/AP_Rally.h for base class implementation and rally point storage
 * @see ArduCopter/mode_rtl.cpp for RTL mode logic and rally point integration
 * @see ArduCopter/Parameters.cpp for RALLY_* parameter definitions
 */
class AP_Rally_Copter : public AP_Rally
{
public:
    /**
     * @brief Construct a new AP_Rally_Copter object
     * 
     * @details Default constructor that initializes the copter-specific rally point manager
     *          by invoking the base AP_Rally constructor. The base class handles rally point
     *          storage initialization, parameter registration, and MAVLink message handler setup.
     *          
     *          Initialization Sequence:
     *          1. Base AP_Rally constructor initializes rally point storage array
     *          2. Rally point parameters (RALLY_TOTAL, RALLY_LIMIT_KM, etc.) are registered
     *          3. MAVLink mission item handlers are configured for rally point upload/download
     *          4. Rally point system is ready to accept rally point definitions from GCS
     *          
     *          The constructor is called during Copter object initialization early in the
     *          boot sequence, before flight mode initialization.
     * 
     * @note This is a lightweight constructor - actual rally point loading from storage
     *       occurs during the init() phase after storage systems are available
     * 
     * @see AP_Rally::AP_Rally() for base class initialization
     */
    AP_Rally_Copter() : AP_Rally() { }

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Rally_Copter);

private:
    /**
     * @brief Validate whether a rally point is suitable for multicopter use
     * 
     * @details This method overrides the base AP_Rally::is_valid() to provide copter-specific
     *          validation of rally point locations. Validation ensures that rally points meet
     *          safety and operational requirements for multicopter RTL operations.
     *          
     *          Validation Criteria:
     *          
     *          1. Geographic Validity:
     *             - Latitude must be within valid range: -90 to +90 degrees
     *             - Longitude must be within valid range: -180 to +180 degrees
     *             - Altitude must be within reasonable bounds (not excessively high/low)
     *          
     *          2. Distance Limits:
     *             - Rally point must be within RALLY_LIMIT_KM distance from home position
     *             - Distance check uses 2D horizontal distance (lat/lon only)
     *             - Default limit is 300 meters (0.3 km) but configurable via parameter
     *             - Prevents accidentally configured rally points at distant locations
     *          
     *          3. Altitude Reasonableness:
     *             - Rally point altitude should be within operational flight envelope
     *             - Typically validates altitude is not below terrain (if terrain data available)
     *             - Ensures altitude is not excessively high for battery capacity
     *          
     *          4. Location Sanity:
     *             - Checks that location is not at 0,0 coordinates (uninitialized)
     *             - Verifies location object has valid altitude frame (AMSL or relative)
     *             - Ensures location is not the same as home (redundant rally point)
     *          
     *          Invalid rally points are rejected during upload from ground control station,
     *          preventing storage of rally points that could cause unsafe RTL behavior.
     *          
     *          Validation Context:
     *          This method is called during:
     *          - Rally point upload via MAVLink MISSION_ITEM messages
     *          - Rally point validation checks before RTL mode uses rally point
     *          - Rally point editing operations via GCS interface
     *          - System startup rally point integrity verification
     *          
     *          Copter-Specific Considerations:
     *          Multicopters have different operational constraints than fixed-wing aircraft:
     *          - Multicopters can land vertically, so precise landing zone size is less critical
     *          - Multicopters are more sensitive to wind at rally point location
     *          - Multicopters require good GPS reception for position hold during landing
     *          - Altitude validation may be stricter to ensure battery capacity for climb
     * 
     * @param[in] rally_point The rally point Location to validate
     *                        Contains latitude, longitude, altitude, and altitude frame information
     * 
     * @return true if the rally point is valid and safe for multicopter RTL operations
     * @return false if the rally point fails any validation criteria and should be rejected
     * 
     * @note This method is const-qualified as it performs read-only validation without
     *       modifying rally point data or object state
     * 
     * @warning Validation does not check for obstacles at rally point location - site survey
     *          and manual verification of landing zone safety is required before setting rally points
     * 
     * @warning Validation cannot detect temporary hazards (construction, vehicles, etc.) that
     *          may appear at rally point location after initial configuration
     * 
     * @see AP_Rally::is_valid() for base class validation implementation
     * @see libraries/AP_Rally/AP_Rally.cpp for distance limit checking algorithm
     * @see Location class for coordinate representation and validation methods
     */
    bool is_valid(const Location &rally_point) const override;
};

#endif  // HAL_RALLY_ENABLED
