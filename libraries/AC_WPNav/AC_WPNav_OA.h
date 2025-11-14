/**
 * @file AC_WPNav_OA.h
 * @brief Object Avoidance (OA) extension for waypoint navigation
 * 
 * @details This file defines AC_WPNav_OA, which extends AC_WPNav with dynamic
 *          object avoidance capabilities. It integrates with AP_OAPathPlanner
 *          to compute safe intermediate waypoints when obstacles are detected
 *          between the vehicle and the final destination.
 *          
 *          The class transparently handles object avoidance by:
 *          - Maintaining the original final destination internally
 *          - Computing OA-adjusted intermediate waypoints when obstacles detected
 *          - Reporting distance/bearing to the final destination (not intermediate)
 *          - Automatically transitioning between normal and OA navigation modes
 *          
 *          OA State Machine:
 *          - OA_NOT_REQUIRED: Clear path, proceed directly to destination
 *          - PROCESSING: Path planning in progress, using previous path
 *          - SUCCESS: Valid OA path computed, following intermediate waypoint
 *          - ERROR: Path planning failed, may revert or use failsafe behavior
 *          
 * @warning This class is only compiled when AC_WPNAV_OA_ENABLED feature flag
 *          is defined. Code using this class must check feature availability
 *          or use conditional compilation.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AC_WPNav_config.h"

#if AC_WPNAV_OA_ENABLED

#include <AC_WPNav/AC_WPNav.h>
#include <AC_Avoidance/AP_OAPathPlanner.h>
#include <AC_Avoidance/AP_OABendyRuler.h>

/**
 * @class AC_WPNav_OA
 * @brief Waypoint navigation with dynamic object avoidance
 * 
 * @details AC_WPNav_OA extends the base AC_WPNav class to add dynamic object
 *          avoidance capabilities through integration with AP_OAPathPlanner.
 *          
 *          **Architecture**:
 *          This class inherits all standard waypoint navigation functionality
 *          from AC_WPNav and overrides key methods to insert object avoidance
 *          path planning between the vehicle and the final destination.
 *          
 *          **Operation**:
 *          - When a destination is set, the class stores it as the "final" destination
 *          - Each update cycle, AP_OAPathPlanner checks for obstacles in the path
 *          - If obstacles detected, an intermediate OA-adjusted waypoint is computed
 *          - The base class AC_WPNav then navigates to the intermediate waypoint
 *          - Distance/bearing queries always return values to the final destination
 *            (making OA transparent to higher-level vehicle code)
 *          
 *          **OA State Machine**:
 *          - OA_NOT_REQUIRED: Clear path to destination, proceed directly
 *          - PROCESSING: Path planner computing alternative route
 *          - SUCCESS: Valid OA path found, intermediate waypoint available
 *          - ERROR: Path planning failed (terrain data missing, no valid path)
 *          
 *          **Backup/Restore Mechanism**:
 *          When OA activates, original waypoint parameters are backed up to
 *          _origin_oabak_neu_cm, _destination_oabak_neu_cm, etc. When OA
 *          completes or is no longer needed, these values are restored to
 *          ensure seamless transition back to normal navigation.
 *          
 *          **Failsafe Behavior**:
 *          If terrain data is unavailable or path planner returns an error,
 *          the class may revert to direct navigation (depending on OA settings)
 *          or attempt alternative routing strategies.
 *          
 *          **Coordinate Frames**:
 *          - Internal waypoints use NEU (North-East-Up) coordinates in cm
 *            from EKF origin
 *          - Global destinations use Location objects (lat/lon/alt)
 *          - Terrain altitude mode supported when terrain data available
 *          
 *          **Thread Safety**:
 *          This class is called from the main vehicle loop at the scheduler rate
 *          (typically 50-400Hz). AP_OAPathPlanner runs asynchronously and returns
 *          the latest computed path.
 * 
 * @note Object avoidance is compute-intensive. Path planning may take multiple
 *       update cycles to complete. During PROCESSING state, the vehicle continues
 *       on the previous path.
 * 
 * @warning Only available when AC_WPNAV_OA_ENABLED and AP_OAPATHPLANNER_ENABLED
 *          are both true at compile time.
 * 
 * @see AC_WPNav for base waypoint navigation
 * @see AP_OAPathPlanner for path planning algorithms
 */
class AC_WPNav_OA : public AC_WPNav
{

public:
    /**
     * @brief Construct a new AC_WPNav_OA object with object avoidance
     * 
     * @details Initializes the waypoint navigation controller with OA capabilities.
     *          Inherits base class initialization and adds OA-specific state variables.
     * 
     * @param[in] ahrs Reference to Attitude and Heading Reference System for position/attitude
     * @param[in] pos_control Reference to position controller for executing waypoint navigation
     * @param[in] attitude_control Reference to attitude controller for vehicle orientation
     * 
     * @note This constructor is identical to AC_WPNav but enables OA path planning integration
     */
    AC_WPNav_OA(const AP_AHRS_View& ahrs, AC_PosControl& pos_control, const AC_AttitudeControl& attitude_control);

    /**
     * @brief Get object avoidance adjusted intermediate waypoint destination
     * 
     * @details Returns the current OA-adjusted intermediate waypoint in global
     *          coordinates. When object avoidance is active (OA state is SUCCESS),
     *          this returns the intermediate waypoint computed by the path planner.
     *          When OA is not active, this returns the final destination.
     *          
     *          This method converts the internal NEU cm representation to a
     *          global Location object (lat/lon/alt).
     * 
     * @param[out] destination Location object to be populated with OA-adjusted waypoint
     *                         (latitude, longitude, altitude)
     * 
     * @return true if conversion successful and destination is valid
     * @return false if unable to convert from NEU vector to global coordinates
     *               (may occur if EKF origin not set or position estimate unavailable)
     * 
     * @note This returns the *intermediate* OA waypoint, not the final destination.
     *       For the final destination, use the base class method or query directly.
     * 
     * @see set_wp_destination_NEU_cm for setting the final destination
     */
    bool get_oa_wp_destination(Location& destination) const override;

    /**
     * @brief Set waypoint destination and trigger object avoidance path planning
     * 
     * @details Sets the final waypoint destination using a position vector in NEU
     *          (North-East-Up) coordinates relative to the EKF origin. This method
     *          stores the destination as the "final" target and triggers the object
     *          avoidance path planner to compute intermediate waypoints if obstacles
     *          are detected.
     *          
     *          The destination can be specified as either:
     *          - Altitude above EKF origin (is_terrain_alt = false)
     *          - Altitude above terrain (is_terrain_alt = true, requires terrain data)
     *          
     *          When OA is active, the base class will navigate to intermediate OA
     *          waypoints, but this final destination is used for all distance/bearing
     *          calculations, making OA transparent to vehicle mode code.
     * 
     * @param[in] destination Target waypoint position vector in NEU frame
     *                       - North: cm north of EKF origin
     *                       - East: cm east of EKF origin
     *                       - Up: cm above EKF origin (or above terrain if is_terrain_alt=true)
     * @param[in] is_terrain_alt true if destination.z is altitude above terrain (default: false)
     *                          false if destination.z is altitude above EKF origin
     * 
     * @return true if destination set successfully
     * @return false on failure, likely causes:
     *               - Terrain data unavailable (when is_terrain_alt = true)
     *               - Invalid position estimate from EKF
     *               - Destination out of valid range
     * 
     * @note This method backs up the original destination parameters for restoration
     *       when OA completes. The OA state machine is initialized to check for obstacles.
     * 
     * @warning If terrain data is unavailable and is_terrain_alt=true, method returns
     *          false and waypoint navigation will not proceed. Vehicle mode should
     *          handle this failure appropriately.
     * 
     * @see get_wp_distance_to_destination_cm for distance to this final destination
     * @see update_wpnav for OA path planning integration
     */
    bool set_wp_destination_NEU_cm(const Vector3f& destination, bool is_terrain_alt = false) override;

    /**
     * @brief Get horizontal distance to final destination in centimeters
     * 
     * @details Calculates the horizontal (2D) distance from the vehicle's current
     *          position to the final waypoint destination. This method ALWAYS returns
     *          the distance to the original final destination set by
     *          set_wp_destination_NEU_cm(), NOT the distance to any intermediate
     *          OA-adjusted waypoint.
     *          
     *          This design makes object avoidance transparent to vehicle mode code:
     *          the mode can query distance to destination without needing to know
     *          whether OA is active or what intermediate waypoints exist.
     * 
     * @return Horizontal distance to final destination in cm
     *         Returns 0 if destination not set or position estimate unavailable
     * 
     * @note Distance calculation uses only North and East components (ignores altitude)
     * @note Distance is to the FINAL destination, not intermediate OA waypoint
     * 
     * @see get_oa_wp_destination for the current OA-adjusted intermediate waypoint
     * @see reached_wp_destination to check if final destination has been reached
     */
    float get_wp_distance_to_destination_cm() const override;

    /**
     * @brief Get bearing to final destination in centidegrees
     * 
     * @details Calculates the horizontal bearing from the vehicle's current position
     *          to the final waypoint destination. This method ALWAYS returns the
     *          bearing to the original final destination, NOT the bearing to any
     *          intermediate OA-adjusted waypoint.
     *          
     *          This maintains transparency of the OA system - vehicle mode code
     *          receives consistent bearing information regardless of whether OA
     *          is actively routing around obstacles.
     * 
     * @return Bearing to final destination in centidegrees (0-36000)
     *         - 0 cd = North
     *         - 9000 cd = East
     *         - 18000 cd = South
     *         - 27000 cd = West
     *         Returns current heading if destination not set
     * 
     * @note Bearing is calculated in the horizontal plane only (ignores altitude)
     * @note Uses centidegrees (1/100th of a degree) for precision
     * @note Bearing is to the FINAL destination, not intermediate OA waypoint
     * 
     * @see get_wp_bearing_to_destination_rad for bearing in radians
     */
    int32_t get_wp_bearing_to_destination_cd() const override;

    /**
     * @brief Get bearing to final destination in radians
     * 
     * @details Calculates the horizontal bearing from the vehicle's current position
     *          to the final waypoint destination, returned in radians. Like the
     *          centidegree version, this method ALWAYS returns bearing to the
     *          original final destination, NOT to any intermediate OA waypoint.
     *          
     *          This is the radian equivalent of get_wp_bearing_to_destination_cd().
     * 
     * @return Bearing to final destination in radians (0 to 2π)
     *         - 0 rad = North
     *         - π/2 rad = East
     *         - π rad = South
     *         - 3π/2 rad = West
     *         Returns current heading if destination not set
     * 
     * @note Bearing is calculated in the horizontal plane only (ignores altitude)
     * @note Returns value in range [0, 2π]
     * @note Bearing is to the FINAL destination, not intermediate OA waypoint
     * 
     * @see get_wp_bearing_to_destination_cd for bearing in centidegrees
     */
    virtual float get_wp_bearing_to_destination_rad() const override;

    /**
     * @brief Check if vehicle has reached the final destination
     * 
     * @details Determines whether the vehicle has come within the configured
     *          waypoint radius (WPNAV_RADIUS parameter) of the final destination.
     *          This method checks against the FINAL destination, not any
     *          intermediate OA waypoints.
     *          
     *          The vehicle is considered to have reached the destination when:
     *          - Horizontal distance <= WPNAV_RADIUS cm
     *          - Altitude is within acceptable vertical threshold (if applicable)
     *          
     *          When OA is active, the vehicle may pass through multiple intermediate
     *          OA waypoints before this method returns true. Each intermediate
     *          waypoint is reached based on internal OA logic, but the final
     *          destination is only considered reached when the vehicle arrives
     *          at the original target set by set_wp_destination_NEU_cm().
     * 
     * @return true if vehicle is within WPNAV_RADIUS cm of final destination
     * @return false if still en route or destination not set
     * 
     * @note WPNAV_RADIUS is a configurable parameter (typically 200-500 cm)
     * @note This checks the FINAL destination, making OA transparent to vehicle mode
     * @note May return false even if vehicle has reached an intermediate OA waypoint
     * 
     * @see get_wp_distance_to_destination_cm for current distance to destination
     */
    bool reached_wp_destination() const override;

    /**
     * @brief Main waypoint navigation update loop with OA path planning integration
     * 
     * @details This is the primary update method called each control cycle (typically
     *          at 50-400Hz depending on vehicle scheduler configuration). It integrates
     *          object avoidance path planning with the base waypoint navigation controller.
     *          
     *          **Update Sequence**:
     *          1. Query AP_OAPathPlanner for the current OA state and path
     *          2. Handle OA state transitions:
     *             - OA_NOT_REQUIRED: Restore original destination, proceed directly
     *             - PROCESSING: Continue with previous path while computing new route
     *             - SUCCESS: Update to new OA-adjusted intermediate waypoint
     *             - ERROR: Handle planning failure (restore direct path or failsafe)
     *          3. Call base class update_wpnav() to execute navigation to current target
     *          4. Update position controller with desired velocity/acceleration
     *          
     *          **State Machine Management**:
     *          The method manages transitions between OA and direct navigation by
     *          backing up and restoring waypoint parameters:
     *          - When OA activates: Back up original origin/destination to _*_oabak variables
     *          - When OA deactivates: Restore from backup to resume direct navigation
     *          
     *          **Failsafe Handling**:
     *          If path planner returns ERROR state (terrain data missing, no valid path),
     *          the behavior depends on OA configuration. May revert to direct navigation
     *          or trigger a higher-level failsafe.
     *          
     *          **Performance Considerations**:
     *          Path planning is compute-intensive and may not complete in a single
     *          cycle. During PROCESSING state, the vehicle continues on the previously
     *          computed path to maintain smooth flight.
     * 
     * @return true if waypoint navigation is active and update successful
     * @return false if navigation not active, position estimate unavailable,
     *               or critical error occurred
     * 
     * @note Called at main loop rate (typically 50-400Hz depending on vehicle)
     * @note Path planning runs asynchronously; this method retrieves latest results
     * @note Vehicle continues on previous path during PROCESSING state
     * 
     * @warning This method must be called regularly for OA to function. Missing
     *          updates may result in stale path information or delayed obstacle detection.
     * 
     * @see set_wp_destination_NEU_cm for initiating waypoint navigation
     * @see AP_OAPathPlanner for path planning algorithm details
     */
    bool update_wpnav() override;

protected:

    /**
     * @name Object Avoidance State Variables
     * @{
     */
    
    /**
     * @brief Current state of object avoidance path planning
     * 
     * @details Tracks the state machine for OA path planning:
     *          - OA_NOT_REQUIRED: Clear path, no obstacles detected
     *          - PROCESSING: Path planner computing alternative route
     *          - SUCCESS: Valid OA path found, using intermediate waypoint
     *          - ERROR: Path planning failed (terrain unavailable, no valid path)
     */
    AP_OAPathPlanner::OA_RetState _oa_state;
    
    /**
     * @brief Backup of original origin position before OA activation
     * 
     * @details Stores the original _origin_neu_cm from base class when OA becomes
     *          active. Restored when OA completes to resume direct navigation.
     *          Position in NEU frame (cm from EKF origin).
     */
    Vector3f    _origin_oabak_neu_cm;
    
    /**
     * @brief Backup of original final destination before OA activation
     * 
     * @details Stores the original _destination_neu_cm from base class when OA
     *          becomes active. This is the final destination set by
     *          set_wp_destination_NEU_cm(). Always used for distance/bearing
     *          calculations. Restored when OA completes.
     *          Position in NEU frame (cm from EKF origin).
     */
    Vector3f    _destination_oabak_neu_cm;
    
    /**
     * @brief Backup of original next destination before OA activation
     * 
     * @details Stores the original _next_destination_neu_cm from base class
     *          when OA becomes active. Used for multi-segment path navigation.
     *          Restored when OA completes.
     *          Position in NEU frame (cm from EKF origin).
     */
    Vector3f    _next_destination_oabak_neu_cm;
    
    /**
     * @brief Flag indicating if backup positions use terrain altitude
     * 
     * @details true if the z-axis of backed up origin and destination positions
     *          represent altitude above terrain (requires terrain data).
     *          false if z-axis is altitude above EKF origin.
     *          Stored to ensure correct altitude interpretation when restoring.
     */
    bool        _is_terrain_alt_oabak;
    
    /**
     * @brief Current OA-adjusted intermediate waypoint destination
     * 
     * @details When OA is active (state = SUCCESS), this contains the intermediate
     *          waypoint computed by AP_OAPathPlanner to avoid detected obstacles.
     *          The base class navigates to this location while distance/bearing
     *          queries return values for the final backed-up destination.
     *          Global coordinates (Location object with lat/lon/alt).
     */
    Location    _oa_destination;
    
    /**
     * @brief Next OA-adjusted intermediate waypoint for multi-segment paths
     * 
     * @details For complex OA paths requiring multiple waypoints, this stores
     *          the subsequent intermediate destination after _oa_destination.
     *          Supports smooth transitions between OA waypoints.
     *          Global coordinates (Location object with lat/lon/alt).
     */
    Location    _oa_next_destination;
    
    /** @} */ // end of Object Avoidance State Variables group
};

#endif  // AC_WPNAV_OA_ENABLED
