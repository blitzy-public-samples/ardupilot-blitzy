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
 * @file AP_Beacon.h
 * @brief Beacon positioning system frontend for GPS-denied navigation
 * 
 * @details This file defines the AP_Beacon frontend class which manages beacon-based
 *          positioning systems for indoor or GPS-denied navigation. The frontend
 *          provides a hardware-abstraction layer over various beacon backends including
 *          Pozyx, Marvelmind, Nooploop, and SITL simulation.
 *          
 *          Beacon systems consist of fixed infrastructure beacons at known positions
 *          and a vehicle-mounted receiver that measures distances to multiple beacons.
 *          The system provides vehicle position estimates in NED frame relative to
 *          a configured origin, which can be integrated with the EKF for navigation.
 *          
 *          Architecture: Frontend (this class) manages backend selection, parameter
 *          storage, position caching, and boundary fence computation. Backends handle
 *          hardware-specific communication and distance measurement protocols.
 * 
 * @note Integrated with AP_AHRS/EKF via get_vehicle_position_ned() for GPS-denied navigation
 * @warning Beacon configuration and placement directly affects position accuracy and fence boundaries
 */

#pragma once

#include "AP_Beacon_config.h"

#if AP_BEACON_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>

class AP_Beacon_Backend;

/**
 * @class AP_Beacon
 * @brief Beacon positioning system frontend managing multiple beacon backends
 * 
 * @details Frontend class implementing the hardware abstraction layer for beacon-based
 *          positioning systems. Manages backend lifecycle (initialization, selection,
 *          destruction), caches vehicle position estimates, maintains beacon states,
 *          and computes fence boundaries from beacon positions.
 *          
 *          Singleton Pattern: Access via AP::beacon() namespace accessor. Only one
 *          instance exists per vehicle, allocated via NEW_NOTHROW during init().
 *          
 *          Main Loop Integration: update() called periodically to refresh beacon data
 *          and vehicle position. Health determined by recent updates within 
 *          AP_BEACON_TIMEOUT_MS (300ms).
 *          
 *          Coordinate System: All positions in NED (North-East-Down) frame relative
 *          to configured origin (origin_lat, origin_lon, origin_alt parameters).
 *          
 *          Configuration: _TYPE parameter selects backend, origin parameters define
 *          reference point, orient_yaw rotates beacon coordinates if needed.
 * 
 * @note Maximum AP_BEACON_MAX_BEACONS beacons supported simultaneously
 * @warning Changing origin or beacon positions requires EKF reset for consistency
 */
class AP_Beacon
{
public:
    friend class AP_Beacon_Backend;

    /**
     * @brief Constructor - initializes beacon system frontend
     * 
     * @details Registers parameter table and initializes internal state. Does not
     *          allocate backend - call init() after construction to select and
     *          initialize backend based on _TYPE parameter.
     */
    AP_Beacon();

    /**
     * @brief Get singleton instance of AP_Beacon
     * 
     * @return Pointer to singleton instance, or nullptr if not instantiated
     * 
     * @note Prefer using AP::beacon() namespace accessor for cleaner code
     */
    static AP_Beacon *get_singleton() { return _singleton; }

    /**
     * @enum Type
     * @brief Beacon backend types for hardware-specific implementations
     * 
     * @details Selects which beacon positioning system backend to instantiate.
     *          Set via _TYPE parameter. Backend allocation occurs in init() using
     *          NEW_NOTHROW pattern for each supported hardware type.
     * 
     * @var Type::None
     * Beacon system disabled - no backend allocated
     * 
     * @var Type::Pozyx
     * Pozyx UWB (Ultra-Wideband) beacon system backend
     * 
     * @var Type::Marvelmind
     * Marvelmind Indoor GPS beacon system backend
     * 
     * @var Type::Nooploop
     * Nooploop UWB beacon system backend
     * 
     * @var Type::SITL
     * Software-in-the-loop simulation backend (only available when AP_BEACON_SITL_ENABLED)
     * 
     * @note Backend selection is runtime-configurable via parameter
     * @warning Changing _TYPE requires init() recall or vehicle reboot
     */
    enum class Type : uint8_t {
        None   = 0,
        Pozyx  = 1,
        Marvelmind = 2,
        Nooploop  = 3,
#if AP_BEACON_SITL_ENABLED
        SITL   = 10
#endif
    };

    /**
     * @struct BeaconState
     * @brief State information for an individual beacon
     * 
     * @details Populated by backend drivers with current beacon data. Frontend maintains
     *          array of BeaconState structures (up to AP_BEACON_MAX_BEACONS) representing
     *          all detected beacons. Backends update these structures during their update()
     *          cycle with fresh distance measurements and health status.
     *          
     *          Position coordinates are in NED (North-East-Down) frame relative to the
     *          beacon system origin (configured via origin_lat/lon/alt parameters).
     *          
     *          Health determined by recent updates and valid data from backend hardware.
     * 
     * @var BeaconState::id
     * Unique identifier for this beacon (hardware-specific, typically 0-255)
     * 
     * @var BeaconState::healthy
     * True if beacon is providing valid data and updated recently (within AP_BEACON_TIMEOUT_MS)
     * 
     * @var BeaconState::distance
     * Measured distance from vehicle to beacon in meters
     * 
     * @var BeaconState::distance_update_ms
     * System time (milliseconds since boot) of last distance measurement update
     * 
     * @var BeaconState::position
     * Beacon position in NED frame (meters) relative to beacon system origin
     */
    struct BeaconState {
        uint16_t id;            // unique id of beacon
        bool     healthy;       // true if beacon is healthy
        float    distance;      // distance from vehicle to beacon (in meters)
        uint32_t distance_update_ms;    // system time of last update from this beacon
        Vector3f position;      // location of beacon as an offset from origin in NED in meters
    };

    /**
     * @brief Initialize beacon positioning system and allocate backend
     * 
     * @details Reads _TYPE parameter to determine which backend to instantiate,
     *          allocates backend via NEW_NOTHROW, and calls backend->init().
     *          If _TYPE is None or allocation fails, no backend is created.
     *          
     *          Called once during vehicle initialization after parameter system
     *          is loaded. Safe to call multiple times - will not reallocate if
     *          backend already exists with same type.
     * 
     * @note Backend selection based on _TYPE parameter value (see Type enum)
     * @warning Must be called before using any positioning functions
     */
    void init(void);

    /**
     * @brief Check if beacon positioning system is enabled
     * 
     * @details Returns true if _TYPE parameter is not None, indicating that
     *          a beacon backend should be active. Does not verify backend
     *          health or data validity - use healthy() for that.
     * 
     * @return true if beacon system is enabled (_TYPE != None), false otherwise
     * 
     * @note Returns true even if backend initialization failed
     */
    bool enabled(void) const;

    /**
     * @brief Check if beacon system is providing valid data
     * 
     * @details Returns true if backend exists, is receiving data, and position
     *          estimate has been updated within AP_BEACON_TIMEOUT_MS (300ms).
     *          This is the primary health check for determining if beacon data
     *          is suitable for navigation.
     * 
     * @return true if beacon system is healthy and data is fresh, false otherwise
     * 
     * @note Checks veh_pos_update_ms against current time with 300ms timeout
     * @see AP_BEACON_TIMEOUT_MS for staleness threshold
     */
    bool healthy(void) const;

    /**
     * @brief Update beacon system state - call periodically from main loop
     * 
     * @details Calls backend->update() if backend exists, which refreshes beacon
     *          distance measurements, vehicle position estimate, and beacon health
     *          states. Typically called at main loop rate or scheduler task rate
     *          (10-50Hz depending on vehicle type).
     *          
     *          Backends use this call to process incoming data from hardware,
     *          update beacon_state[] array, and compute vehicle position.
     * 
     * @note Must be called regularly to maintain healthy() status
     * @warning If not called within AP_BEACON_TIMEOUT_MS, system becomes unhealthy
     */
    void update(void);

    /**
     * @brief Get beacon system origin in global coordinates
     * 
     * @details Returns the configured origin point for the beacon coordinate system.
     *          All beacon positions and vehicle position estimates are expressed in
     *          NED frame relative to this origin. Origin is set via origin_lat,
     *          origin_lon, and origin_alt parameters.
     *          
     *          This origin should be provided to the EKF to ensure consistent
     *          coordinate frame alignment between beacon positioning and inertial
     *          navigation.
     * 
     * @param[out] origin_loc Location object populated with origin lat/lon (degrees) and alt (meters AMSL)
     * 
     * @return true if origin is configured (parameters set), false if origin invalid
     * 
     * @note Origin altitude is in meters above mean sea level (AMSL)
     * @warning Changing origin parameters requires EKF reset for consistency
     */
    bool get_origin(Location &origin_loc) const;

    /**
     * @brief Get vehicle position estimate in NED frame from beacon system
     * 
     * @details Returns cached vehicle position from most recent backend update.
     *          Position is in NED (North-East-Down) frame in meters relative to
     *          beacon system origin. Accuracy estimate provides 1-sigma uncertainty
     *          radius in meters.
     *          
     *          This is the primary interface for EKF integration - EKF can use this
     *          position estimate as an external navigation source for GPS-denied
     *          operation. Only returns true if data is healthy (recent update within
     *          AP_BEACON_TIMEOUT_MS).
     *          
     *          Position calculation performed by backend using trilateration from
     *          multiple beacon distance measurements.
     * 
     * @param[out] pos Vehicle position in NED frame (meters) relative to origin
     * @param[out] accuracy_estimate Position accuracy estimate (1-sigma radius in meters)
     * 
     * @return true if position is valid and recent, false if stale or unavailable
     * 
     * @note Coordinate system: NED frame (North=X+, East=Y+, Down=Z+)
     * @note Units: position in meters, accuracy_estimate in meters
     * @warning Returns false if backend not initialized or data stale (>300ms)
     */
    bool get_vehicle_position_ned(Vector3f& pos, float& accuracy_estimate) const;

    /**
     * @brief Get number of beacons currently detected
     * 
     * @details Returns count of beacons in beacon_state[] array that have been
     *          populated by backend. Maximum count is AP_BEACON_MAX_BEACONS.
     *          Count may fluctuate as beacons come in and out of range.
     * 
     * @return Number of detected beacons (0 to AP_BEACON_MAX_BEACONS)
     * 
     * @note Count includes both healthy and unhealthy beacons
     */
    uint8_t count() const;

    /**
     * @brief Get complete state data for a specific beacon
     * 
     * @details Retrieves full BeaconState structure for specified beacon instance,
     *          including id, health, distance, last update time, and position.
     *          This is the preferred method for getting comprehensive beacon data.
     * 
     * @param[in]  beacon_instance Index of beacon (0 to count()-1)
     * @param[out] state BeaconState structure populated with beacon data
     * 
     * @return true if beacon_instance is valid and data copied, false if index out of range
     * 
     * @note beacon_instance must be < count() for valid data
     * @see BeaconState for structure field details
     */
    bool get_beacon_data(uint8_t beacon_instance, struct BeaconState& state) const;

    /**
     * @brief Get unique identifier for a specific beacon
     * 
     * @param[in] beacon_instance Index of beacon (0 to count()-1)
     * 
     * @return Beacon unique ID (hardware-specific, typically 0-255), or 0 if invalid instance
     * 
     * @note Returns 0 for invalid beacon_instance
     */
    uint8_t beacon_id(uint8_t beacon_instance) const;

    /**
     * @brief Check if a specific beacon is healthy
     * 
     * @details Returns health status from beacon_state[] array. Health determined
     *          by backend based on recent updates and data validity.
     * 
     * @param[in] beacon_instance Index of beacon (0 to count()-1)
     * 
     * @return true if beacon is healthy and providing valid data, false otherwise
     * 
     * @note Returns false for invalid beacon_instance
     */
    bool beacon_healthy(uint8_t beacon_instance) const;

    /**
     * @brief Get distance measurement to a specific beacon
     * 
     * @details Returns cached distance measurement from vehicle to beacon.
     *          Distance calculated by backend from time-of-flight or signal
     *          strength depending on hardware type.
     * 
     * @param[in] beacon_instance Index of beacon (0 to count()-1)
     * 
     * @return Distance to beacon in meters, or 0.0f if invalid instance
     * 
     * @note Units: meters
     * @note Returns 0.0f for invalid beacon_instance
     */
    float beacon_distance(uint8_t beacon_instance) const;

    /**
     * @brief Get position of a specific beacon in NED frame
     * 
     * @details Returns beacon position from beacon_state[] array. Position is
     *          in NED frame (meters) relative to beacon system origin. Beacon
     *          positions are typically fixed and configured during system setup.
     * 
     * @param[in] beacon_instance Index of beacon (0 to count()-1)
     * 
     * @return Beacon position Vector3f in NED frame (meters), or zero vector if invalid instance
     * 
     * @note Coordinate system: NED frame relative to origin
     * @note Units: meters
     * @note Returns (0,0,0) for invalid beacon_instance
     */
    Vector3f beacon_position(uint8_t beacon_instance) const;

    /**
     * @brief Get timestamp of last update from a specific beacon
     * 
     * @details Returns system time (milliseconds since boot) when this beacon's
     *          distance measurement was last updated by backend. Used to determine
     *          data freshness and beacon health.
     * 
     * @param[in] beacon_instance Index of beacon (0 to count()-1)
     * 
     * @return Timestamp in milliseconds since boot, or 0 if invalid instance
     * 
     * @note Units: milliseconds since system boot
     * @note Returns 0 for invalid beacon_instance
     * @see AP_BEACON_TIMEOUT_MS for staleness threshold
     */
    uint32_t beacon_last_update_ms(uint8_t beacon_instance) const;

    /**
     * @brief Compute fence boundary polygon from beacon positions
     * 
     * @details Constructs a convex hull polygon from current beacon positions to
     *          define an outer fence boundary. Uses iterative angular sweep algorithm
     *          implemented in get_next_boundary_point() - starts from beacon closest
     *          to origin, sweeps clockwise to find next boundary point, repeats until
     *          returning to start.
     *          
     *          Boundary is 2D (horizontal plane only, ignoring altitude). Result stored
     *          in boundary[] array with boundary_num_points count. Typically called
     *          when beacon positions change or periodically to update fence.
     *          
     *          Algorithm: Select starting point (closest to origin) → Find next point
     *          by clockwise angular sweep from current point → Repeat until closed loop.
     * 
     * @note Boundary computation is 2D only - uses beacon X,Y (North,East) positions
     * @note Maximum boundary points limited by AP_BEACON_MAX_BEACONS+1
     * @warning Requires at least 3 beacons for valid boundary polygon
     */
    void update_boundary_points();

    /**
     * @brief Get computed fence boundary polygon points
     * 
     * @details Returns pointer to boundary[] array containing convex hull polygon
     *          computed by update_boundary_points(). Can be used by fence system
     *          (AC_Fence) to create inclusion fence around beacon coverage area.
     *          
     *          Boundary points are 2D vectors in NED frame (North, East coordinates
     *          only, altitude ignored). Points ordered to form closed polygon.
     * 
     * @param[out] num_points Number of points in boundary array
     * 
     * @return Pointer to boundary points array (Vector2f[]), or nullptr if no boundary computed
     * 
     * @note Coordinate system: 2D NED (North=X, East=Y), units in meters
     * @note Array valid until next update_boundary_points() call
     * @warning Returns nullptr if boundary not yet computed or insufficient beacons
     */
    const Vector2f* get_boundary_points(uint16_t& num_points) const;

    /**
     * @brief Parameter table for AP_Beacon
     * 
     * @details Defines configurable parameters:
     *          - _TYPE: Backend selection (Type enum)
     *          - origin_lat: Origin latitude in degrees
     *          - origin_lon: Origin longitude in degrees
     *          - origin_alt: Origin altitude in meters AMSL
     *          - orient_yaw: Yaw rotation for beacon coordinates in centidegrees
     */
    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @brief Log beacon system state to dataflash/SD card
     * 
     * @details Creates structured log message containing beacon health, count,
     *          distance measurements, and vehicle position estimate. Log structure
     *          defined in log_Beacon format. Called by vehicle code to record beacon
     *          data for post-flight analysis.
     *          
     *          Typical logging rate: 1-10Hz depending on vehicle scheduler configuration.
     *          Logs written via AP::logger() interface.
     * 
     * @note Log format: time_us, health, count, distances (up to 4 beacons), position NED
     * @see AP::logger() for logging interface
     */
    void log();

private:

    static AP_Beacon *_singleton;  ///< Singleton instance pointer for AP_Beacon

    /**
     * @brief Check if backend device is ready for operation
     * 
     * @details Internal health check verifying backend exists and is initialized.
     *          Used by public health checks and position getters.
     * 
     * @return true if backend allocated and ready, false otherwise
     */
    bool device_ready(void) const;

    /**
     * @brief Find next boundary point using clockwise angular sweep algorithm
     * 
     * @details Helper function for update_boundary_points() implementing convex hull
     *          construction via iterative angular sweep. Starting from current_index,
     *          sweeps clockwise from start_angle to find the next boundary point that
     *          forms the outermost convex polygon edge.
     *          
     *          Algorithm: For each candidate point, calculate angle from current point.
     *          Select point with smallest clockwise angle difference from start_angle.
     *          Uses atan2f for angle calculation in range [-PI, PI].
     * 
     * @param[in]  boundary Array of 2D points (NED frame, meters) to process
     * @param[in]  num_points Number of points in boundary array
     * @param[in]  current_index Index of current point in boundary array
     * @param[in]  start_angle Starting angle in radians for clockwise sweep
     * @param[out] next_index Index of next boundary point found
     * @param[out] next_angle Angle (radians) from current point to next point
     * 
     * @return true if next boundary point found, false if search failed
     * 
     * @note Angle convention: radians, standard math convention (East=0, North=PI/2)
     * @note Uses atan2f(North, East) for angle calculation
     */
    static bool get_next_boundary_point(const Vector2f* boundary, uint8_t num_points, uint8_t current_index, float start_angle, uint8_t& next_index, float& next_angle);

    // Parameters - persistent configuration
    AP_Enum<Type> _type;        ///< Backend type selection (None, Pozyx, Marvelmind, Nooploop, SITL)
    AP_Float origin_lat;        ///< Beacon system origin latitude in degrees
    AP_Float origin_lon;        ///< Beacon system origin longitude in degrees
    AP_Float origin_alt;        ///< Beacon system origin altitude in meters AMSL
    AP_Int16 orient_yaw;        ///< Orientation yaw rotation in centidegrees for coordinate transformation

    // Backend driver
    AP_Beacon_Backend *_driver; ///< Pointer to hardware-specific backend (allocated via NEW_NOTHROW in init())

    // Cached vehicle position estimate (updated by backend)
    Vector3f veh_pos_ned;       ///< Vehicle position in NED frame (meters) relative to origin
    float veh_pos_accuracy;     ///< Position accuracy estimate (1-sigma radius in meters)
    uint32_t veh_pos_update_ms; ///< Timestamp (ms since boot) of last position update

    // Individual beacon state tracking
    uint8_t num_beacons = 0;    ///< Current number of detected beacons (0 to AP_BEACON_MAX_BEACONS)
    BeaconState beacon_state[AP_BEACON_MAX_BEACONS]; ///< Array of beacon states populated by backend

    // Fence boundary data (computed from beacon positions)
    Vector2f boundary[AP_BEACON_MAX_BEACONS+1]; ///< Convex hull boundary points in 2D NED (meters)
    uint8_t boundary_num_points;                ///< Number of valid points in boundary array
    uint8_t boundary_num_beacons;               ///< Number of beacons used in boundary computation
};

/**
 * @namespace AP
 * @brief ArduPilot core namespace for singleton accessors
 */
namespace AP {
    /**
     * @brief Get AP_Beacon singleton instance
     * 
     * @details Namespace accessor for cleaner singleton access pattern. Preferred
     *          over AP_Beacon::get_singleton() for consistency with other ArduPilot
     *          subsystems (AP::ahrs(), AP::logger(), AP::gps(), etc.).
     *          
     *          Example usage:
     *          @code
     *          if (AP::beacon() != nullptr && AP::beacon()->healthy()) {
     *              Vector3f pos;
     *              float accuracy;
     *              AP::beacon()->get_vehicle_position_ned(pos, accuracy);
     *          }
     *          @endcode
     * 
     * @return Pointer to AP_Beacon singleton, or nullptr if not instantiated
     * 
     * @note Always check for nullptr before dereferencing
     * @see AP_Beacon::get_singleton() for alternative accessor
     */
    AP_Beacon *beacon();
};

#endif  // AP_BEACON_ENABLED
