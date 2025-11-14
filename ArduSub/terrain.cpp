/**
 * @file terrain.cpp
 * @brief Terrain following system integration for ArduSub
 * 
 * @details This file implements terrain data management and altitude queries
 *          for the ArduSub vehicle, integrating with the AP_Terrain library
 *          to provide terrain-relative altitude information. While terrain
 *          following is less commonly used in underwater vehicles compared to
 *          aircraft, this module supports:
 *          
 *          - Terrain data updates and caching from external sources
 *          - Height-above-terrain calculations for rangefinder integration
 *          - Terrain data logging for mission analysis
 *          - Support for terrain-relative mission waypoints
 *          
 *          The terrain system relies on external terrain data (typically from
 *          ground control stations or pre-loaded databases) and provides
 *          altitude information relative to the terrain surface below the vehicle.
 * 
 * @note This module is conditionally compiled and only available when
 *       AP_TERRAIN_AVAILABLE is defined. Terrain functionality requires:
 *       - Terrain database loaded via ground control station
 *       - Valid GPS position for terrain tile lookup
 *       - Sufficient memory for terrain data caching
 * 
 * @see libraries/AP_Terrain/ for terrain library implementation
 * @see Sub::terrain member variable for terrain object instance
 * 
 * Source: ArduSub/terrain.cpp:1-32
 */

#include "Sub.h"

/**
 * @brief Update terrain data and integrate with rangefinder
 * 
 * @details This function performs periodic terrain data updates and manages
 *          the integration between the terrain system and rangefinder for
 *          power optimization. Called regularly from the main scheduler loop.
 *          
 *          The update process:
 *          1. Updates terrain database cache from available data sources
 *          2. Processes pending terrain tile requests
 *          3. Queries height-above-terrain for current vehicle position
 *          4. Provides terrain height to rangefinder for power saving mode
 *          
 *          When valid terrain data is available and the vehicle altitude above
 *          terrain is known, this information is passed to the rangefinder
 *          system, allowing rangefinders to enter power-saving modes when
 *          operating at altitudes where returns are not expected.
 * 
 * @note This function is conditionally compiled and only active when
 *       AP_TERRAIN_AVAILABLE is defined. Rangefinder integration requires
 *       both AP_TERRAIN_AVAILABLE and AP_RANGEFINDER_ENABLED.
 * 
 * @note Called at main loop rate (typically 50Hz for ArduSub). The terrain
 *       library internally rate-limits expensive operations like tile requests.
 * 
 * @warning Terrain data accuracy depends on:
 *          - Quality and resolution of terrain database
 *          - GPS position accuracy
 *          - Terrain data coverage for operating area
 * 
 * @see AP_Terrain::update() for terrain data update implementation
 * @see AP_Terrain::height_above_terrain() for altitude query
 * @see RangeFinder::set_estimated_terrain_height() for power saving
 */
void Sub::terrain_update()
{
#if AP_TERRAIN_AVAILABLE
    terrain.update();

    // tell the rangefinder our height, so it can go into power saving
    // mode if available
#if AP_RANGEFINDER_ENABLED
    float height;
    if (terrain.height_above_terrain(height, true)) {
        rangefinder.set_estimated_terrain_height(height);
    }
#endif
#endif
}

#if HAL_LOGGING_ENABLED
/**
 * @brief Log terrain data for mission analysis and debugging
 * 
 * @details Records terrain system status and altitude information to the
 *          onboard dataflash log for post-flight analysis. This function
 *          logs terrain-related data including:
 *          
 *          - Current height above terrain
 *          - Terrain system health status
 *          - Terrain data availability
 *          - Pending terrain tile requests
 *          
 *          Terrain logs are useful for:
 *          - Verifying terrain database coverage
 *          - Debugging terrain-relative mission commands
 *          - Analyzing altitude control performance
 *          - Post-mission terrain data validation
 * 
 * @note This function is conditionally compiled and requires both
 *       HAL_LOGGING_ENABLED and AP_TERRAIN_AVAILABLE to be defined.
 * 
 * @note Should be called at 1Hz from the scheduler. Logging only occurs
 *       when GPS logging is enabled (MASK_LOG_GPS is set), as terrain
 *       data is position-dependent.
 * 
 * @note Log message format is defined in AP_Terrain library and written
 *       as a TERRAIN log message type in the dataflash log.
 * 
 * @see AP_Terrain::log_terrain_data() for log message implementation
 * @see Sub::should_log() for logging mask checking
 */
void Sub::terrain_logging()
{
#if AP_TERRAIN_AVAILABLE
    if (should_log(MASK_LOG_GPS)) {
        terrain.log_terrain_data();
    }
#endif
}
#endif  // HAL_LOGGING_ENABLED

