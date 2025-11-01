/**
 * @file sensors.cpp
 * @brief Sensor integration and reading for ArduPlane
 * 
 * @details This file handles sensor integration for the ArduPlane fixed-wing autopilot,
 *          specifically managing rangefinder sensor data for terrain following and
 *          obstacle avoidance capabilities. The sensor readings are used to update
 *          height estimates for low-altitude flight operations including landing.
 *          
 *          Key responsibilities:
 *          - Rangefinder power management based on altitude
 *          - Height estimation using terrain data or barometric altitude
 *          - Integration with terrain following system
 *          - Sensor update coordination with flight stages
 *          
 * @note This module is conditionally compiled based on AP_RANGEFINDER_ENABLED
 * @see libraries/AP_RangeFinder/ for rangefinder driver implementations
 * @see libraries/AP_Terrain/ for terrain database integration
 * 
 * Source: ArduPlane/sensors.cpp
 */

#include "Plane.h"
#include <AP_RSSI/AP_RSSI.h>
#include <AP_OpticalFlow/AP_OpticalFlow.h>

#if AP_RANGEFINDER_ENABLED
/**
 * @brief Read rangefinder sensor and update height estimate for terrain following
 * 
 * @details This function manages rangefinder sensor readings for fixed-wing aircraft,
 *          coordinating between terrain database information and barometric altitude
 *          estimates to provide accurate height above ground measurements.
 *          
 *          The function performs the following operations:
 *          1. Determines appropriate altitude estimate for rangefinder power management
 *          2. Notifies rangefinder of estimated terrain height for auto power control
 *          3. Updates rangefinder measurements
 *          4. Propagates height data to navigation systems via rangefinder_height_update()
 *          
 *          Altitude Estimation Priority:
 *          - Primary: Terrain database height (AP_Terrain) if available and valid
 *          - Fallback (Landing): Target height during landing approach
 *          - Fallback (Other): Barometric altitude relative to home
 *          
 *          Power Management:
 *          The rangefinder can be configured to power down at high altitudes to conserve
 *          energy. This function provides altitude estimates to enable automatic power-up
 *          when approaching terrain during operations like landing or terrain following.
 *          
 * @note Called at main loop rate (typically 50-400Hz depending on vehicle configuration)
 * @note Rangefinder power management requires appropriate altitude estimation
 * 
 * @warning Accurate height estimation is critical for terrain following safety.
 *          Loss of terrain data during low-altitude flight may result in fallback
 *          to barometric altitude which can have significant errors near terrain.
 * 
 * @see Plane::rangefinder_height_update() for height data propagation
 * @see AP_RangeFinder::update() for sensor measurement acquisition
 * @see AP_Terrain::height_above_terrain() for terrain database queries
 * 
 * Source: ArduPlane/sensors.cpp:9-36
 */
void Plane::read_rangefinder(void)
{
    // Notify the rangefinder of our approximate altitude above ground to allow it to power on
    // during low-altitude flight when configured to power down during higher-altitude flight.
    // This enables power-saving at cruise altitude while ensuring sensor availability during
    // critical low-altitude operations (landing, terrain following, obstacle avoidance).
    float height;
#if AP_TERRAIN_AVAILABLE
    // Sensor Availability Check: Attempt to use terrain database for most accurate height
    // Terrain database provides pre-loaded elevation data for precise terrain following
    if (terrain.status() == AP_Terrain::TerrainStatusOK && terrain.height_above_terrain(height, true)) {
        // Terrain database is available and valid - use terrain-based height above ground
        // This is the preferred method as it accounts for actual terrain elevation changes
        rangefinder.set_estimated_terrain_height(height);
    } else
#endif
    {
        // Fallback: Use barometric altitude estimate when terrain database unavailable
        // This occurs when terrain data is not loaded, GPS position unknown, or terrain
        // system disabled. Barometric altitude relative to home is less accurate over
        // varying terrain but provides essential rangefinder power management.
        
        if (flight_stage == AP_FixedWing::FlightStage::LAND) {
            // Landing Stage: Use target altitude to ensure rangefinder powers on
            // During landing approach, target altitude is below current altitude and
            // decreasing. This ensures rangefinder activation even when landing site
            // elevation differs significantly from home elevation.
            height = height_above_target();
        } else {
            // Normal Flight: Use barometric altitude relative to home position
            // Provides reasonable estimate for rangefinder power management during
            // cruise, loiter, and other non-landing flight stages.
            height = relative_altitude;
        }
        rangefinder.set_estimated_terrain_height(height);
    }

    // Update rangefinder measurements: triggers sensor reading and data processing
    // This call updates the rangefinder state, reads new measurements from the sensor,
    // and applies filtering and validation to the raw distance data.
    rangefinder.update();

    // Propagate rangefinder height data to navigation and control systems
    // This function integrates rangefinder measurements with the Extended Kalman Filter (EKF)
    // and other navigation systems for terrain following, landing, and obstacle avoidance.
    // Height data is used for low-altitude flight control and terrain clearance monitoring.
    rangefinder_height_update();
}

#endif  // AP_RANGEFINDER_ENABLED
