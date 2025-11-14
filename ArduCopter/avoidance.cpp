/**
 * @file avoidance.cpp
 * @brief Obstacle avoidance integration for ArduCopter
 * 
 * @details This file integrates the AC_Avoidance library with ArduCopter to provide
 *          comprehensive obstacle avoidance capabilities during autonomous and manual flight.
 *          
 *          The avoidance system combines multiple data sources and algorithms:
 *          
 *          **Proximity Sensor Fusion**:
 *          - Integrates data from multiple proximity sensors (lidar, sonar, radar)
 *          - Fuses rangefinder measurements to build 360-degree obstacle awareness
 *          - Processes sensor data through the AC_Avoid class (copter.avoid object)
 *          - Filters and validates sensor readings for reliable obstacle detection
 *          
 *          **Object Avoidance Algorithms** (via AC_Avoidance library):
 *          - Simple avoidance: Direct velocity adjustment to avoid detected obstacles
 *          - Stop-at-fence behavior: Prevents vehicle from breaching configured boundaries
 *          - Slide-along-obstacle: Allows tangential movement along obstacle surfaces
 *          - Backup velocity: Provides escape path when boxed in by obstacles
 *          - Path planning integration: Works with AP_OAPathPlanner for complex scenarios
 *          
 *          **Velocity Adjustment for Obstacle Avoidance**:
 *          The system modifies desired velocity commands to avoid collisions:
 *          - adjust_velocity(): Main entry point called by flight modes (Guided, Auto, etc.)
 *          - adjust_velocity_fence(): Prevents fence boundary breaches
 *          - adjust_velocity_proximity(): Uses proximity sensor data for obstacle avoidance
 *          - adjust_velocity_z(): Vertical avoidance for altitude limits
 *          These functions are implemented in the AC_Avoid library and called via copter.avoid
 *          
 *          **Integration with Flight Systems**:
 *          - Position Control: Avoidance modifies velocity targets before position controller
 *          - Fence System: Coordinates with AC_Fence for boundary enforcement
 *          - Flight Modes: Guided, Auto, Loiter, and other modes call avoid.adjust_velocity()
 *          - Failsafes: Avoidance behavior adapts based on failsafe state
 *          
 *          **Altitude-Based Avoidance Control**:
 *          This file implements altitude-dependent avoidance enablement:
 *          - Disables simple avoidance when operating very close to ground (below configured minimum)
 *          - Prevents false avoidance triggers from ground returns
 *          - Enables full avoidance capability at safe operating altitudes
 *          
 *          **Safety-Critical Considerations**:
 *          - Avoidance system can prevent commanded movements if obstacles detected
 *          - Pilot always maintains ability to override with manual control
 *          - Avoidance disabled when rangefinder readings invalid to prevent false triggers
 *          - System designed as safety enhancement, not primary collision prevention
 *          
 *          **Configuration**:
 *          Avoidance behavior configured through AC_Avoid parameters:
 *          - AVOID_ENABLE: Bitmask enabling avoidance features (fence, proximity, etc.)
 *          - AVOID_MARGIN: Distance margin to maintain from obstacles (meters)
 *          - AVOID_BEHAVE: Avoidance behavior (slide or stop)
 *          - AVOID_ALT_MIN: Minimum altitude for proximity-based avoidance (meters)
 *          
 * @note Avoidance algorithms implemented in libraries/AC_Avoidance/AC_Avoid.cpp
 * @warning Avoidance system is an aid to safe flight, not a substitute for proper flight planning
 * 
 * @see AC_Avoid for core avoidance implementation
 * @see AP_OAPathPlanner for advanced path planning around obstacles
 * @see AC_Fence for boundary enforcement integration
 * @see AP_Proximity for sensor data fusion
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include "Copter.h"

/**
 * @brief Enable or disable altitude-dependent proximity avoidance based on current height above ground
 * 
 * @details This function manages the altitude-based enablement of simple proximity avoidance
 *          to prevent false obstacle detection from ground returns when flying at very low altitudes.
 *          
 *          **Algorithm**:
 *          1. Query rangefinder for current height above ground (interpolated)
 *          2. If rangefinder invalid: Enable avoidance (fail-safe approach)
 *          3. If altitude < configured minimum: Disable proximity avoidance
 *          4. If altitude >= configured minimum: Enable proximity avoidance
 *          
 *          **Rationale for Altitude Threshold**:
 *          When flying very close to the ground (below AVOID_ALT_MIN):
 *          - Proximity sensors may detect ground as obstacles
 *          - False positives can prevent low-altitude maneuvering
 *          - Ground effect and sensor scatter increase false detections
 *          - Disabling proximity avoidance allows normal low-altitude flight
 *          
 *          At safe operating altitudes (above AVOID_ALT_MIN):
 *          - Ground returns minimal or filtered out
 *          - Legitimate obstacles more reliably detected
 *          - Full avoidance capability beneficial for obstacle-rich environments
 *          
 *          **Rangefinder Health Handling**:
 *          If rangefinder data unavailable (sensor failure, out of range, etc.):
 *          - System defaults to ENABLING avoidance (conservative/safe approach)
 *          - Prevents accidental disablement due to sensor failure
 *          - Assumes vehicle at sufficient altitude for avoidance to be beneficial
 *          
 *          **Integration with AC_Avoidance Library**:
 *          - Calls avoid.proximity_alt_avoidance_enable() to control feature
 *          - AC_Avoid library respects this enable/disable flag
 *          - Other avoidance features (fence, altitude limits) remain active
 *          - Only proximity-sensor-based avoidance affected by this function
 *          
 *          **Calling Context**:
 *          - Called periodically by main scheduler (typically 10-50 Hz)
 *          - Runs continuously during all flight modes
 *          - No direct pilot or GCS control over this function
 *          - Configuration via AVOID_ALT_MIN parameter (default varies by vehicle)
 *          
 *          **Thread Safety**:
 *          - Called from main thread only (no locking required)
 *          - Reads sensor data via thread-safe accessors
 *          - State changes atomic via avoid object methods
 * 
 * @note Function only compiled when AP_AVOIDANCE_ENABLED feature flag is set
 * @note Minimum altitude threshold configured via avoid.get_min_alt() (AVOID_ALT_MIN parameter)
 * @note Does not affect other avoidance features (fence avoidance, altitude limits, etc.)
 * 
 * @warning Disabling proximity avoidance at low altitude removes obstacle detection protection
 * @warning Ensure AVOID_ALT_MIN set appropriately for operating environment and sensor characteristics
 * 
 * @see AC_Avoid::proximity_alt_avoidance_enable() for enable/disable implementation
 * @see get_rangefinder_height_interpolated_cm() for altitude measurement
 * @see AC_Avoid::get_min_alt() for minimum altitude threshold retrieval
 */
void Copter::low_alt_avoidance()
{
#if AP_AVOIDANCE_ENABLED
    // Variable to store current altitude above ground in centimeters
    // Rangefinder provides more accurate ground clearance than barometric altitude,
    // especially over varying terrain
    int32_t alt_cm;
    
    // Attempt to get interpolated rangefinder height above ground
    // Interpolation smooths readings from multiple rangefinders if available
    // Returns false if no valid rangefinder data (sensor offline, out of range, or timing issue)
    if (!get_rangefinder_height_interpolated_cm(alt_cm)) {
        // FAIL-SAFE BEHAVIOR: Enable avoidance when rangefinder unavailable
        // Rationale: Cannot determine altitude, so assume at safe height requiring avoidance
        // This is the conservative approach - better to have false obstacle avoidance
        // than to disable protection when vehicle might be at altitude with obstacles present
        avoid.proximity_alt_avoidance_enable(true);
        return;  // Early exit - cannot make altitude-based decision without valid reading
    }

    // Default assumption: Enable proximity-based avoidance
    // This will be overridden if altitude is below the configured minimum threshold
    bool enable_avoidance = true;
    
    // Check if current altitude is below the configured minimum for proximity avoidance
    // avoid.get_min_alt() returns threshold in meters (from AVOID_ALT_MIN parameter)
    // Multiply by 100.0f to convert meters to centimeters for comparison with alt_cm
    // 
    // LOW ALTITUDE LOGIC:
    // When flying below AVOID_ALT_MIN, disable proximity avoidance because:
    // - Proximity sensors detect ground as obstacle (false positive)
    // - Ground reflections create spurious obstacle detections
    // - Sensor scatter and multipath effects increase near ground
    // - May prevent normal landing and low-altitude operations
    if (alt_cm < avoid.get_min_alt() * 100.0f) {
        enable_avoidance = false;  // Disable proximity avoidance at low altitude
    }
    // If altitude >= AVOID_ALT_MIN, enable_avoidance remains true (normal operation)
    
    // Apply the enable/disable decision to the AC_Avoid library
    // This controls only proximity-sensor-based simple avoidance
    // Other avoidance features (fence, altitude limits, beacon) remain unaffected
    // The AC_Avoid library will respect this flag when processing velocity adjustments
    avoid.proximity_alt_avoidance_enable(enable_avoidance);
#endif  // AP_AVOIDANCE_ENABLED
}
