/**
 * @file terrain.cpp
 * @brief Terrain database integration for altitude-above-terrain flight operations
 * 
 * @details This file implements the interface between ArduCopter and the AP_Terrain
 *          library, which provides access to SRTM (Shuttle Radar Topography Mission)
 *          terrain elevation data. The terrain database enables:
 *          - Terrain-relative altitude control in AUTO missions (via waypoint option)
 *          - Terrain following for survey and mapping applications
 *          - Accurate altitude reporting relative to ground level
 *          - Enhanced safety through terrain awareness and collision avoidance
 * 
 *          Terrain data is loaded from the ground control station via MAVLink in
 *          4x4 grid chunks (TERRAIN_GRID_MAVLINK_SIZE) and cached locally in 32x28
 *          grid blocks. The database covers global SRTM data at approximately 30-90m
 *          resolution depending on location.
 * 
 *          Coordinate System: Terrain altitudes are AMSL (Above Mean Sea Level)
 *          Unit Convention: Heights in meters
 * 
 * @note Terrain database requires SD card or sufficient flash storage
 * @warning Terrain data may not be available in all locations or may be temporarily
 *          unavailable during flight. Missions using terrain-relative waypoints will
 *          trigger failsafe if terrain data cannot be obtained.
 * 
 * @see AP_Terrain for terrain database system implementation
 * @see libraries/AP_Terrain/AP_Terrain.h for terrain API documentation
 * 
 * Source: ArduCopter/terrain.cpp
 */

#include "Copter.h"

/**
 * @brief Update terrain database and communicate with rangefinder for power management
 * 
 * @details This function performs two critical operations:
 *          
 *          1. Terrain Database Update:
 *             - Processes incoming terrain data from ground control station via MAVLink
 *             - Manages terrain grid block cache (LRU cache of up to 12 blocks)
 *             - Loads terrain data from SD card if previously cached
 *             - Requests missing terrain data from GCS as vehicle moves
 *             - Interpolates terrain altitude for current vehicle position
 *          
 *          2. Rangefinder Power Management:
 *             - Provides rangefinder with estimated height above terrain
 *             - Allows rangefinder to enter power-saving mode when vehicle is high
 *             - Optimizes power consumption for long-duration flights
 *             - Uses terrain database or barometer-based estimation
 * 
 *          Mission Integration:
 *          When missions include waypoints with terrain-relative altitude flag set,
 *          this function ensures terrain data is available for the planned path. The
 *          system pre-fetches terrain data ahead of the vehicle's projected position.
 * 
 *          Failsafe Behavior:
 *          If terrain data becomes unavailable during a terrain-relative mission:
 *          - Vehicle will attempt to continue for short data gaps (extrapolation)
 *          - Extended unavailability triggers EKF terrain failsafe
 *          - Default action is RTL (Return to Launch) or LAND depending on configuration
 *          - Status is reported via TERRAIN_REPORT MAVLink message
 * 
 *          Altitude Relationships:
 *          - terrain_altitude: Vehicle altitude relative to terrain (AGL - Above Ground Level)
 *          - home_altitude: Vehicle altitude relative to home position (takeoff point)
 *          - current_altitude: Vehicle altitude AMSL from barometer and EKF
 *          - terrain_database_altitude: Ground elevation AMSL from SRTM data
 *          
 *          Formula: terrain_altitude = current_altitude - terrain_database_altitude
 * 
 * @note Called from main scheduler at 400Hz main loop rate, but actual terrain
 *       update occurs at approximately 1-10Hz depending on system load
 * @note Rangefinder integration is only active if AP_RANGEFINDER_ENABLED is defined
 * 
 * @warning Terrain database updates require active MAVLink connection to GCS
 * @warning SD card or flash storage must be available for terrain data caching
 * @warning First flight in a new area may experience delays while terrain data loads
 * 
 * @see AP_Terrain::update() for terrain database update implementation
 * @see AP_Terrain::height_above_terrain() for altitude calculation details
 * @see AC_Fence for terrain-based fence integration
 * @see mode_auto.cpp for terrain-relative waypoint mission execution
 * 
 * Source: ArduCopter/terrain.cpp:4-18
 */
void Copter::terrain_update()
{
#if AP_TERRAIN_AVAILABLE
    // Update terrain database: process incoming MAVLink terrain data, manage cache,
    // interpolate terrain altitude for current position, and request missing data
    // from ground control station. This is the main entry point for the terrain
    // database system, called periodically from the scheduler.
    terrain.update();

    // Rangefinder power management optimization: provide rangefinder with estimated
    // height above terrain so it can enter low-power mode when vehicle is too high
    // for useful measurements (typically >10m depending on rangefinder model)
#if AP_RANGEFINDER_ENABLED
    float height;  // Height above terrain in meters (AGL - Above Ground Level)
    
    // Get current height above terrain from database. Extrapolation enabled (true)
    // allows system to estimate height during brief terrain data gaps using vehicle
    // velocity and last known terrain gradient. This prevents rangefinder power
    // cycling during temporary data unavailability.
    if (terrain.height_above_terrain(height, true)) {
        // Successfully obtained terrain height - update rangefinder for power management
        // Rangefinder uses this to determine if it should enter sleep mode or reduce
        // sampling rate when vehicle is at high altitude where measurements would exceed
        // sensor range. This significantly reduces power consumption during cruise flight.
        rangefinder.set_estimated_terrain_height(height);
    }
    // Note: If terrain data unavailable, rangefinder continues using last known height
    // or defaults to always-on operation for safety
#endif
#endif
}

#if HAL_LOGGING_ENABLED
/**
 * @brief Log terrain database status and altitude data for post-flight analysis
 * 
 * @details This function logs terrain-related data to the onboard dataflash or SD card
 *          for post-flight analysis, debugging, and mission verification. Logging occurs
 *          at approximately 1Hz to balance data completeness with storage efficiency.
 * 
 *          Logged Information (TERR message):
 *          - TerrainStatus: Current terrain system health (Disabled/Unhealthy/OK)
 *          - CurrentHeight: Vehicle height above terrain in meters (AGL)
 *          - TerrainAltitude: Ground elevation from database in meters AMSL
 *          - PendingGrids: Number of terrain grid blocks awaiting download from GCS
 *          - Loaded: Whether terrain data is available for current location
 * 
 *          Log Analysis Use Cases:
 *          - Verify terrain-relative waypoints achieved desired AGL altitude
 *          - Debug terrain data availability issues during missions
 *          - Analyze terrain following performance in survey operations
 *          - Validate terrain database coverage for planned flight areas
 *          - Investigate terrain-related failsafe triggers
 *          - Compare terrain database altitude with rangefinder measurements
 * 
 *          Data Storage Considerations:
 *          At 1Hz logging rate, terrain data adds minimal storage overhead:
 *          - Approximately 20 bytes per second
 *          - ~1.2 KB per minute
 *          - ~70 KB per hour of flight
 *          Total storage impact is negligible compared to IMU and GPS data rates.
 * 
 *          Relationship to Other Logs:
 *          - GPS logs provide position and AMSL altitude (compare with terrain altitude)
 *          - RFND logs provide rangefinder height measurements (validate terrain data)
 *          - CTUN logs show altitude controller targets (verify terrain-relative control)
 *          - MODE logs indicate when terrain-relative missions are active
 * 
 * @note Logging only occurs if GPS logging mask (MASK_LOG_GPS) is enabled via LOG_BITMASK
 * @note This function should be called at 1Hz from the scheduler for consistent data
 * @note Logging disabled if HAL_LOGGING_ENABLED is not defined (minimizes firmware size)
 * 
 * @warning Do not call at higher rates - excessive logging can cause SD card wear and
 *          may impact real-time performance on slower storage devices
 * @warning Terrain logs may be empty if terrain system is disabled via TERRAIN_ENABLE=0
 * 
 * @see AP_Terrain::log_terrain_data() for actual logging implementation
 * @see AP_Logger for dataflash logging system architecture
 * @see LOG_BITMASK parameter documentation for logging control
 * @see Mission Planner / MAVProxy for log download and analysis tools
 * 
 * Source: ArduCopter/terrain.cpp:22-30
 */
void Copter::terrain_logging()
{
#if AP_TERRAIN_AVAILABLE
    // Check if GPS logging is enabled via LOG_BITMASK parameter
    // Terrain logging piggybacks on GPS logging mask since terrain data
    // is inherently position-dependent and typically analyzed alongside GPS logs
    if (should_log(MASK_LOG_GPS)) {
        // Write TERR message to dataflash with current terrain status, height above
        // terrain, ground elevation, and data availability metrics. This is the main
        // terrain logging entry point called at 1Hz from the 1Hz scheduler task.
        terrain.log_terrain_data();
    }
    // Note: If GPS logging disabled, terrain data is not logged to conserve storage
    // and reduce SD card write cycles. Enable GPS logging to capture terrain data.
#endif
}
#endif
