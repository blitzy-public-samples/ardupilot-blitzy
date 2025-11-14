/**
 * @file sensors.cpp
 * @brief Sensor management and integration for ArduSub underwater vehicles
 * 
 * @details This file implements sensor reading, health monitoring, and data processing
 *          for ArduSub underwater vehicles. Key sensor systems include:
 *          - Barometer/Pressure sensor: Used for depth measurement underwater
 *          - Rangefinder/Sonar: Used for terrain tracking and obstacle avoidance
 * 
 *          The underwater environment presents unique challenges:
 *          - Barometers measure water pressure to determine depth below surface
 *          - Rangefinders typically mounted facing downward (ROTATION_PITCH_270)
 *          - Sensor health monitoring is critical for safe underwater operation
 *          - Tilt correction needed for accurate rangefinder readings on tilted vehicles
 * 
 *          Integration Points:
 *          - AP_Baro library for pressure/depth sensing
 *          - AP_RangeFinder library for sonar/rangefinder sensors
 *          - Navigation systems (wp_nav, circle_nav) for terrain following
 *          - Inertial navigation for sensor fusion and altitude estimation
 * 
 * @note Depth hold mode requires a healthy barometer/pressure sensor
 * @note Terrain tracking modes require a healthy rangefinder sensor
 * 
 * @warning Sensor failures underwater can lead to loss of depth control
 *          Always ensure sensor health before attempting depth hold or autonomous modes
 * 
 * Source: ArduSub/sensors.cpp:1-86
 */

#include "Sub.h"

/**
 * @brief Read and update barometer (pressure sensor) for underwater depth measurement
 * 
 * @details This function updates the barometer reading and performs health monitoring
 *          for depth sensing in underwater vehicles. The barometer measures water
 *          pressure to calculate depth below the surface.
 * 
 *          Calibration Logic:
 *          - Positive altitude readings indicate the sensor needs calibration
 *          - When submerged, the barometer should always read negative altitude (depth)
 *          - Even a few meters above water should show minimal depth reading
 *          - Auto-calibration is triggered if positive altitude is detected
 * 
 *          Health Monitoring:
 *          - Updates sensor_health.depth status if depth sensor is present
 *          - Health status checked against the configured depth_sensor_idx
 *          - Used by flight modes to determine if depth hold is safe
 * 
 * @note This function is called at the main loop rate (typically 50-400Hz)
 * @note Depth sensor health is required for depth hold and altitude hold modes
 * @note The barometer uses AP_Baro library for pressure sensor abstraction
 * 
 * @warning Inaccurate depth readings can lead to uncontrolled ascent or descent
 * 
 * @see AP_Baro::update()
 * @see AP_Baro::healthy()
 * @see AP_Baro::get_altitude()
 * 
 * Source: ArduSub/sensors.cpp:4-16
 */
void Sub::read_barometer()
{
    barometer.update();
    // If we are reading a positive altitude, the sensor needs calibration
    // Even a few meters above the water we should have no significant depth reading
    if(barometer.get_altitude() > 0) {
        barometer.update_calibration();
    }

    if (ap.depth_sensor_present) {
        sensor_health.depth = barometer.healthy(depth_sensor_idx);
    }
}

/**
 * @brief Initialize rangefinder/sonar sensor for underwater terrain tracking
 * 
 * @details Configures the rangefinder sensor system for ArduSub operation.
 *          Rangefinders (typically sonar) are used for bottom tracking, terrain
 *          following, and obstacle avoidance in underwater environments.
 * 
 *          Initialization Steps:
 *          1. Configure logging: Set CTUN log bit for rangefinder data logging
 *          2. Initialize hardware: Set orientation to ROTATION_PITCH_270 (downward facing)
 *          3. Configure filter: Set low-pass filter cutoff for smooth altitude readings
 *          4. Verify availability: Check if rangefinder supports the configured orientation
 * 
 *          Orientation Configuration:
 *          - ROTATION_PITCH_270: Sensor facing straight down (typical for submarines)
 *          - Downward orientation enables bottom distance measurement
 *          - Used for terrain following and altitude above seafloor
 * 
 *          Filter Configuration:
 *          - Low-pass filter smooths noisy sonar readings
 *          - Cutoff frequency: RANGEFINDER_WPNAV_FILT_HZ
 *          - Provides stable altitude data for waypoint navigation
 * 
 * @note This function is called once during vehicle initialization
 * @note Rangefinder functionality only available if AP_RANGEFINDER_ENABLED is defined
 * @note rangefinder_state.enabled will be false if sensor doesn't support downward orientation
 * 
 * @see AP_RangeFinder::init()
 * @see AP_RangeFinder::has_orientation()
 * 
 * Source: ArduSub/sensors.cpp:18-26
 */
void Sub::init_rangefinder()
{
#if AP_RANGEFINDER_ENABLED
    // Enable rangefinder data logging in CTUN (control tuning) logs
    rangefinder.set_log_rfnd_bit(MASK_LOG_CTUN);
    
    // Initialize rangefinder with downward-facing orientation (270° pitch = pointing down)
    rangefinder.init(ROTATION_PITCH_270);
    
    // Configure low-pass filter for smooth altitude readings used by waypoint navigation
    rangefinder_state.alt_filt.set_cutoff_frequency(RANGEFINDER_WPNAV_FILT_HZ);
    
    // Verify rangefinder supports downward orientation and mark as enabled if available
    rangefinder_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_270);
#endif
}

/**
 * @brief Read and process rangefinder/sonar data with health monitoring and terrain offset calculation
 * 
 * @details Updates rangefinder sensor readings and performs comprehensive health checking,
 *          tilt correction, and terrain offset calculations for underwater navigation.
 *          The rangefinder provides distance to the seafloor/terrain below the vehicle.
 * 
 *          Processing Pipeline:
 *          1. Update sensor: Read latest rangefinder measurement
 *          2. Health check: Validate sensor status, signal quality, and data consistency
 *          3. Tilt correction: Compensate for vehicle tilt to get true vertical distance
 *          4. Terrain offset: Calculate terrain height relative to inertial reference
 *          5. Navigation update: Provide data to waypoint and circle navigation systems
 * 
 *          Health Criteria:
 *          - Sensor status must be "Good" (no errors, valid range)
 *          - Sufficient valid reading count (>= RANGEFINDER_HEALTH_MAX)
 *          - Signal quality above minimum threshold (if sensor provides quality metric)
 * 
 *          Tilt Correction (if RANGEFINDER_TILT_CORRECTION enabled):
 *          - Compensates for vehicle pitch/roll when measuring vertical distance
 *          - Uses rotation matrix Z-component (body to NED frame)
 *          - Prevents overestimation of altitude when vehicle is tilted
 *          - Minimum correction factor: 0.707 (prevents division by small numbers at 45°+ tilt)
 * 
 *          Terrain Offset Calculation:
 *          - Determines terrain elevation relative to inertial navigation reference
 *          - Uses low-pass filtered rangefinder data for stability
 *          - Calculates: terrain_offset = inertial_altitude - rangefinder_distance
 *          - Filter reset after RANGEFINDER_TIMEOUT_MS of no valid readings
 *          - Provides terrain-relative altitude for waypoint navigation
 * 
 * @note This function is called at the main loop rate (typically 50-400Hz)
 * @note Terrain following modes require rangefinder_state.alt_healthy = true
 * @note Tilt correction requires AHRS (attitude) data to be valid
 * @note Rangefinder functionality only available if AP_RANGEFINDER_ENABLED is defined
 * 
 * @warning Unhealthy rangefinder can cause terrain following failures
 * @warning Tilt correction disabled at extreme tilt angles (>45°) for safety
 * 
 * @see AP_RangeFinder::update()
 * @see AP_RangeFinder::distance_orient()
 * @see AC_WPNav::set_rangefinder_terrain_offset_cm()
 * 
 * Source: ArduSub/sensors.cpp:29-78
 */
void Sub::read_rangefinder()
{
#if AP_RANGEFINDER_ENABLED
    // Update rangefinder sensor to get latest measurements
    rangefinder.update();

    // Get signal quality percentage: 0 (worst) to 100 (perfect), -1 means not available
    int8_t signal_quality_pct = rangefinder.signal_quality_pct_orient(ROTATION_PITCH_270);

    // Perform comprehensive health check combining multiple criteria:
    // 1. Sensor status must be "Good" (no hardware errors, range valid)
    // 2. Must have sufficient consecutive valid readings for reliability
    // 3. Signal quality must meet minimum threshold (if quality metric supported)
    rangefinder_state.alt_healthy =
            (rangefinder.status_orient(ROTATION_PITCH_270) == RangeFinder::Status::Good) &&
            (rangefinder.range_valid_count_orient(ROTATION_PITCH_270) >= RANGEFINDER_HEALTH_MAX) &&
            (signal_quality_pct == -1 || signal_quality_pct >= g.rangefinder_signal_min);

    // Get raw distance reading from downward-facing rangefinder (in meters)
    float temp_alt_m = rangefinder.distance_orient(ROTATION_PITCH_270);

#if RANGEFINDER_TILT_CORRECTION
    // Apply tilt correction for non-vertical vehicle orientation
    // When vehicle is tilted, the slant range reading is longer than true vertical distance
    // Correction factor = rotation_matrix.c.z (cosine of tilt angle)
    // Minimum factor of 0.707 (~45° tilt) prevents correction at extreme angles for safety
    temp_alt_m = temp_alt_m * MAX(0.707f, ahrs.get_rotation_body_to_ned().c.z);
#endif

    // Store corrected altitude and sensor range limits
    rangefinder_state.alt = temp_alt_m;
    rangefinder_state.inertial_alt_cm = inertial_nav.get_position_z_up_cm();
    rangefinder_state.min = rangefinder.min_distance_orient(ROTATION_PITCH_270);
    rangefinder_state.max = rangefinder.max_distance_orient(ROTATION_PITCH_270);

    // Calculate terrain offset: difference between inertial altitude and rangefinder distance
    // This provides terrain elevation in the inertial navigation reference frame
    if (rangefinder_state.alt_healthy) {
        uint32_t now = AP_HAL::millis();
        if (now - rangefinder_state.last_healthy_ms > RANGEFINDER_TIMEOUT_MS) {
            // Reset filter if rangefinder was unhealthy for more than timeout period
            // This prevents using stale filtered values after sensor recovery
            rangefinder_state.alt_filt.reset(rangefinder_state.alt);
        } else {
            // Apply low-pass filter with 0.05 time constant for smooth terrain tracking
            rangefinder_state.alt_filt.apply(rangefinder_state.alt, 0.05f);
        }
        rangefinder_state.last_healthy_ms = now;
        
        // Calculate terrain offset: inertial_altitude_cm - filtered_rangefinder_cm
        // Positive offset means terrain is below the home/origin point
        // Negative offset means terrain is above the home/origin point
        rangefinder_state.rangefinder_terrain_offset_cm =
            sub.rangefinder_state.inertial_alt_cm - (sub.rangefinder_state.alt_filt.get() * 100);
    }

    // Provide rangefinder data to waypoint navigation for terrain-following missions
    // Waypoint navigation uses this data to maintain altitude above seafloor
    wp_nav.set_rangefinder_terrain_offset_cm(
            rangefinder_state.enabled,
            rangefinder_state.alt_healthy,
            rangefinder_state.rangefinder_terrain_offset_cm);
    
    // Provide rangefinder data to circle navigation mode
    // Only used if waypoint navigation is actively using rangefinder data
    circle_nav.set_rangefinder_terrain_offset_cm(
            rangefinder_state.enabled && wp_nav.rangefinder_used(),
            rangefinder_state.alt_healthy,
            rangefinder_state.rangefinder_terrain_offset_cm);
#endif  // AP_RANGEFINDER_ENABLED
}

/**
 * @brief Check if rangefinder altitude data is valid and safe to use
 * 
 * @details Determines whether the rangefinder sensor data is currently reliable
 *          for use in navigation and terrain following modes. This function
 *          performs a comprehensive validity check before allowing flight modes
 *          to use rangefinder data for altitude control.
 * 
 *          Validity Criteria (all must be true):
 *          1. Rangefinder enabled: Sensor is configured and detected at initialization
 *          2. Altitude healthy: Recent sensor reading passed health checks
 *          3. Recent data: Last healthy reading within RANGEFINDER_TIMEOUT_MS
 * 
 *          Timeout Protection:
 *          - Prevents using stale rangefinder data
 *          - RANGEFINDER_TIMEOUT_MS defines maximum age of valid data
 *          - Ensures real-time responsiveness to terrain changes
 * 
 *          Usage:
 *          - Called by flight modes before using rangefinder for altitude control
 *          - Prevents terrain following with invalid sensor data
 *          - Enables safe fallback to barometric altitude when rangefinder fails
 * 
 * @return true if rangefinder altitude is valid and safe to use
 * @return false if rangefinder unavailable, unhealthy, or data too old
 * 
 * @note This is a const function and does not modify vehicle state
 * @note Typically called at loop rate before terrain-relative altitude calculations
 * @note Terrain following modes should disable or switch to backup altitude source if false
 * 
 * @see read_rangefinder() for health checking logic
 * @see rangefinder_state for sensor state information
 * 
 * Source: ArduSub/sensors.cpp:81-85
 */
bool Sub::rangefinder_alt_ok() const
{
    uint32_t now = AP_HAL::millis();
    // Check all validity criteria: enabled, healthy, and recent data
    return (rangefinder_state.enabled && rangefinder_state.alt_healthy && now - rangefinder_state.last_healthy_ms < RANGEFINDER_TIMEOUT_MS);
}
