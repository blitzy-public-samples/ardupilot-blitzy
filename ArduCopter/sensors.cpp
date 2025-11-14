/**
 * @file sensors.cpp
 * @brief Sensor reading and health monitoring functions for ArduCopter
 * 
 * This file contains functions for reading and monitoring various sensors used by the
 * multicopter including barometer, rangefinders (downward and upward facing), and their
 * associated health checks. These sensors provide critical altitude and terrain data for
 * flight control, terrain following, and surface tracking modes.
 * 
 * Key responsibilities:
 * - Barometric altitude reading and conversion to centimeters
 * - Rangefinder initialization, reading, and health monitoring
 * - Terrain offset estimation for surface tracking
 * - Sensor data validation and failover logic
 * - Integration with proximity sensors and navigation systems
 * 
 * @note All altitude values are in centimeters to maintain precision and consistency
 *       with ArduPilot's internal unit conventions
 * 
 * @warning Sensor health checks are critical for flight safety. Unhealthy sensors
 *          trigger failover to alternative altitude sources (EKF, GPS, etc.)
 */

#include "Copter.h"

/**
 * @brief Read barometric altitude sensor and update vehicle altitude estimate
 * 
 * @details This function updates the barometer sensor reading and converts the altitude
 *          to centimeters for use throughout the flight controller. The barometer provides
 *          relative altitude referenced to the EKF origin (typically the arming location).
 *          
 *          The barometer reading is:
 *          - Used by the EKF for altitude estimation and sensor fusion
 *          - Affected by ground effect during takeoff/landing (pressure disturbance)
 *          - Subject to weather-related pressure changes during flight
 *          - Filtered internally by the AP_Baro library
 *          
 *          Altitude calculation: altitude_cm = barometer_altitude_m * 100.0
 * 
 * @note This function is called at 10Hz by the scheduler (see Copter.cpp scheduler tasks)
 * @note Barometric altitude is relative, not absolute - it measures height above the
 *       pressure altitude at initialization, not above mean sea level
 * 
 * @warning Ground effect during takeoff/landing can cause temporary pressure variations
 *          that affect barometric altitude readings. The EKF compensates for this through
 *          sensor fusion with accelerometer data.
 * 
 * @see AP_Baro::update() for barometer sensor update implementation
 * @see Copter::update_altitude() for how baro_alt is used in altitude control
 * 
 * Source: ArduCopter/sensors.cpp:4-9
 */
void Copter::read_barometer(void)
{
    // Update all configured barometer sensors and calculate altitude
    // This may involve multiple physical barometers with averaging/selection logic
    barometer.update();

    // Convert altitude from meters to centimeters for internal use
    // ArduPilot uses centimeters throughout for altitude to maintain precision
    baro_alt = barometer.get_altitude() * 100.0f;
}

#if AP_RANGEFINDER_ENABLED
/**
 * @brief Initialize rangefinder sensors for terrain sensing and object detection
 * 
 * @details This function initializes both downward-facing (ROTATION_PITCH_270) and
 *          upward-facing (ROTATION_PITCH_90) rangefinders if present on the vehicle.
 *          Rangefinders provide accurate distance measurements for:
 *          - Terrain following in altitude hold and auto modes
 *          - Precision landing on varying terrain
 *          - Surface tracking for maintaining height above ground
 *          - Obstacle detection (ceiling detection for upward sensor)
 *          
 *          Initialization sequence:
 *          1. Configure logging to CTUN (control tune) log messages
 *          2. Initialize rangefinder hardware with downward orientation
 *          3. Configure low-pass filter with user-defined cutoff frequency
 *          4. Check if downward rangefinder is present and enable tracking
 *          5. Configure upward rangefinder filter and enable if present
 *          
 *          Coordinate frame convention:
 *          - ROTATION_PITCH_270 = downward facing (nose=0°, pitch down 270° = pointing down)
 *          - ROTATION_PITCH_90 = upward facing (pitch up 90° = pointing up)
 * 
 * @note Low-pass filtering (configured via g2.rangefinder_filt parameter) reduces noise
 *       from rangefinder measurements, with typical cutoff around 0.5-5 Hz
 * 
 * @note This function is called once during vehicle initialization, not during flight
 * 
 * @warning If no rangefinder with correct orientation is detected, terrain following
 *          modes may fall back to barometric altitude or become unavailable
 * 
 * @see AP_RangeFinder::init() for hardware initialization
 * @see Copter::read_rangefinder() for periodic rangefinder updates
 * @see MASK_LOG_CTUN for control tuning log message definitions
 * 
 * Source: ArduCopter/sensors.cpp:12-22
 */
void Copter::init_rangefinder(void)
{
   // Configure rangefinder to log data in CTUN messages for tuning and analysis
   rangefinder.set_log_rfnd_bit(MASK_LOG_CTUN);
   
   // Initialize rangefinder hardware with downward-facing orientation (270° pitch)
   // This detects and configures all rangefinders pointing downward
   rangefinder.init(ROTATION_PITCH_270);
   
   // Configure low-pass filter for downward rangefinder measurements
   // Filter reduces sensor noise and spikes, improving altitude estimate quality
   // Cutoff frequency from g2.rangefinder_filt parameter (typically 0.5-5 Hz)
   rangefinder_state.alt_cm_filt.set_cutoff_frequency(g2.rangefinder_filt);
   
   // Enable downward rangefinder tracking if sensor with correct orientation exists
   // has_orientation() returns true if at least one rangefinder is facing downward
   rangefinder_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_270);

   // Initialize upward facing range finder for ceiling/obstacle detection
   // Upward sensor (90° pitch) detects overhead obstacles and ceiling distance
   rangefinder_up_state.alt_cm_filt.set_cutoff_frequency(g2.rangefinder_filt);
   rangefinder_up_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_90);
}

/**
 * @brief Read rangefinder sensors and update altitude/distance measurements
 * 
 * @details This function performs a complete update cycle for all rangefinder sensors:
 *          1. Polls rangefinder hardware for new distance measurements
 *          2. Updates downward rangefinder state (filtering, health checks, glitch protection)
 *          3. Updates upward rangefinder state (filtering, health checks, glitch protection)
 *          4. Provides rangefinder altitude to proximity sensor system if available
 *          
 *          The rangefinder_state and rangefinder_up_state objects maintain:
 *          - Filtered altitude measurements (low-pass filtered for noise reduction)
 *          - Glitch-protected readings (rejects outliers and sudden jumps)
 *          - Health status (sensor responding, range valid, data not stale)
 *          - Inertial altitude correlation for terrain offset estimation
 *          
 *          Integration with proximity sensors:
 *          When downward rangefinder is healthy or data is stale but recent, the
 *          altitude is provided to the proximity sensor system for 3D obstacle avoidance.
 *          This allows the proximity system to understand vehicle height above terrain
 *          and adjust obstacle detection accordingly.
 * 
 * @note This function is called at 20Hz by the scheduler (see Copter.cpp)
 * 
 * @note Rangefinder data quality checks include:
 *       - Range within sensor min/max limits
 *       - Signal quality above threshold
 *       - No timeouts or communication errors
 *       - Consistency with previous readings (glitch detection)
 * 
 * @warning If rangefinder becomes unhealthy, terrain following modes must fall back
 *          to barometric altitude, which may be less accurate over varying terrain
 * 
 * @see AP_RangeFinder::update() for hardware sensor reading
 * @see RangeFinderState::update() for state management and health monitoring
 * @see AP_Proximity::set_rangefinder_alt() for proximity sensor integration
 * 
 * Source: ArduCopter/sensors.cpp:25-37
 */
void Copter::read_rangefinder(void)
{
    // Poll all rangefinder sensors for new measurements
    // This triggers I2C/serial/analog reads from hardware
    rangefinder.update();

    // Update downward rangefinder state with new measurements
    // Performs filtering, glitch protection, health checks, and inertial correlation
    rangefinder_state.update();
    
    // Update upward rangefinder state (ceiling/obstacle detection)
    // Maintains separate filtering and health status from downward sensor
    rangefinder_up_state.update();

#if HAL_PROXIMITY_ENABLED
    // Provide rangefinder altitude to proximity sensor system for 3D obstacle mapping
    // Proximity sensors use this to understand vehicle height above terrain
    // Data provided even if stale (recent but not current) for continuity
    if (rangefinder_state.enabled_and_healthy() || rangefinder_state.data_stale()) {
        // Pass enabled status, health status, and filtered altitude to proximity system
        g2.proximity.set_rangefinder_alt(rangefinder_state.enabled, rangefinder_state.alt_healthy, rangefinder_state.alt_cm_filt.get());
    }
#endif
}
#endif  // AP_RANGEFINDER_ENABLED

/**
 * @brief Check if downward rangefinder altitude data is valid and safe to use
 * 
 * @details This health check function verifies that the downward-facing rangefinder
 *          is both enabled (sensor detected during initialization) and currently healthy
 *          (providing valid measurements). Used by flight modes and navigation controllers
 *          to determine if rangefinder-based terrain following is available.
 *          
 *          Health criteria checked by enabled_and_healthy():
 *          - Sensor is present and initialized (enabled flag)
 *          - Recent valid measurements received (no timeout)
 *          - Readings within sensor's valid range limits
 *          - No excessive glitches or outliers detected
 *          - Signal quality above minimum threshold
 *          
 *          Flight modes use this function to decide between:
 *          - Rangefinder-based altitude hold (accurate over terrain)
 *          - Barometric altitude hold (relative to takeoff point)
 * 
 * @return true if rangefinder altitude data can be safely used for flight control
 * @return false if rangefinder is disabled, unhealthy, or providing invalid data
 * 
 * @note This function is const and can be called from any context without side effects
 * 
 * @note Called frequently by altitude control loops and terrain following logic
 * 
 * @warning Returning false triggers failover to barometric altitude, which may be
 *          less accurate for terrain following but more stable over long distances
 * 
 * @see RangeFinderState::enabled_and_healthy() for detailed health check implementation
 * @see Copter::read_rangefinder() for rangefinder state updates
 * 
 * Source: ArduCopter/sensors.cpp:41-44
 */
bool Copter::rangefinder_alt_ok() const
{
    // Return combined enabled and health status for downward rangefinder
    // Used by flight modes for terrain following and altitude control decisions
    return rangefinder_state.enabled_and_healthy();
}

/**
 * @brief Check if upward rangefinder altitude data is valid and safe to use
 * 
 * @details This health check function verifies that the upward-facing rangefinder
 *          (ceiling sensor) is both enabled and currently healthy. Used for detecting
 *          overhead obstacles, ceiling height, and indoor navigation where vertical
 *          clearance must be monitored.
 *          
 *          Health criteria checked by enabled_and_healthy():
 *          - Upward sensor is present and initialized (enabled flag)
 *          - Recent valid measurements received (no timeout)
 *          - Readings within sensor's valid range limits
 *          - No excessive glitches or outliers detected
 *          - Signal quality above minimum threshold
 *          
 *          Applications for upward rangefinder:
 *          - Indoor ceiling detection and avoidance
 *          - Bridge/overpass clearance monitoring
 *          - Parking structure navigation
 *          - Vertical separation from overhead obstacles
 * 
 * @return true if upward rangefinder data can be safely used for obstacle detection
 * @return false if upward rangefinder is disabled, unhealthy, or providing invalid data
 * 
 * @note This function is const and can be called from any context without side effects
 * 
 * @note Less commonly used than downward rangefinder; many vehicles don't have upward sensors
 * 
 * @warning Upward rangefinder failure does not affect altitude control but disables
 *          ceiling detection, potentially allowing collision with overhead obstacles
 * 
 * @see RangeFinderState::enabled_and_healthy() for detailed health check implementation
 * @see Copter::read_rangefinder() for rangefinder state updates
 * 
 * Source: ArduCopter/sensors.cpp:47-50
 */
bool Copter::rangefinder_up_ok() const
{
    // Return combined enabled and health status for upward rangefinder
    // Used for ceiling detection and overhead obstacle avoidance
    return rangefinder_up_state.enabled_and_healthy();
}

/**
 * @brief Update terrain offset estimation using rangefinder and inertial data fusion
 * 
 * @details This function estimates the terrain height relative to the EKF origin (typically
 *          the vehicle's arming location) by combining rangefinder distance measurements with
 *          inertial altitude estimates. The terrain offset enables accurate terrain following
 *          even when the terrain elevation changes during flight.
 *          
 *          Algorithm for downward rangefinder:
 *          1. Calculate instantaneous terrain offset = inertial_altitude - rangefinder_distance
 *          2. Apply low-pass filter with time constant g2.surftrak_tc (surface tracking TC)
 *          3. Filtered_offset += (new_offset - filtered_offset) * (dt / max(TC, dt))
 *          
 *          Algorithm for upward rangefinder:
 *          1. Calculate ceiling offset = inertial_altitude + upward_rangefinder_distance
 *          2. Apply same low-pass filtering for smooth ceiling tracking
 *          
 *          The low-pass filter smooths terrain offset changes, preventing:
 *          - Sudden altitude changes from rangefinder noise
 *          - Oscillations when crossing small terrain features
 *          - Instability from intermittent rangefinder readings
 *          
 *          Terrain offset propagation:
 *          - Waypoint navigation (wp_nav) receives terrain offset for auto missions
 *          - Circle navigation (circle_nav) receives offset if waypoint nav uses rangefinder
 *          - Offset provided even with stale data to maintain continuity during brief dropouts
 *          
 *          Coordinate frame convention:
 *          - Inertial altitude: Height above EKF origin (positive up)
 *          - Downward rangefinder: Distance to ground (positive down, so subtracted)
 *          - Upward rangefinder: Distance to ceiling (positive up, so added)
 *          - Terrain offset: Ground elevation above EKF origin
 * 
 * @note Time constant g2.surftrak_tc controls terrain offset smoothing speed
 *       - Lower TC (e.g., 0.5s) = faster response, less smooth
 *       - Higher TC (e.g., 2.0s) = slower response, more smooth
 * 
 * @note Called at main loop rate (typically 400Hz) to maintain smooth terrain tracking
 * 
 * @warning Terrain offset can drift if rangefinder becomes unhealthy during flight.
 *          Navigation controllers must handle transitions between rangefinder and
 *          barometric altitude modes gracefully to avoid sudden altitude changes.
 * 
 * @see wp_nav->set_rangefinder_terrain_offset_cm() for waypoint navigation integration
 * @see circle_nav->set_rangefinder_terrain_offset_cm() for circle mode integration
 * @see RangeFinderState for inertial altitude correlation details
 * 
 * Source: ArduCopter/sensors.cpp:54-68
 */
void Copter::update_rangefinder_terrain_offset()
{
    // Calculate instantaneous terrain offset for downward rangefinder
    // Terrain offset = where the ground is relative to EKF origin
    // = vehicle's inertial altitude - distance to ground below
    float terrain_offset_cm = rangefinder_state.inertial_alt_cm - rangefinder_state.alt_cm_glitch_protected;
    
    // Apply low-pass filter to smooth terrain offset changes
    // Filter gain = dt / max(time_constant, dt) prevents instability when dt > TC
    // This creates exponential smoothing: new_value = old_value + gain * (measurement - old_value)
    rangefinder_state.terrain_offset_cm += (terrain_offset_cm - rangefinder_state.terrain_offset_cm) * (copter.G_Dt / MAX(copter.g2.surftrak_tc, copter.G_Dt));

    // Calculate ceiling offset for upward rangefinder
    // Ceiling offset = vehicle's inertial altitude + distance to ceiling above
    // Used for indoor navigation and overhead obstacle detection
    terrain_offset_cm = rangefinder_up_state.inertial_alt_cm + rangefinder_up_state.alt_cm_glitch_protected;
    
    // Apply same low-pass filtering to ceiling offset estimation
    rangefinder_up_state.terrain_offset_cm += (terrain_offset_cm - rangefinder_up_state.terrain_offset_cm) * (copter.G_Dt / MAX(copter.g2.surftrak_tc, copter.G_Dt));

    // Propagate terrain offset to navigation controllers if data is available
    // Provide data even if stale (recent but not current) for smooth transitions
    if (rangefinder_state.alt_healthy || rangefinder_state.data_stale()) {
        // Waypoint navigation uses terrain offset for terrain-following auto missions
        wp_nav->set_rangefinder_terrain_offset_cm(rangefinder_state.enabled, rangefinder_state.alt_healthy, rangefinder_state.terrain_offset_cm);
        
#if MODE_CIRCLE_ENABLED
        // Circle mode uses terrain offset only if waypoint nav is actively using rangefinder
        // This prevents circle mode from using rangefinder when other modes don't trust it
        circle_nav->set_rangefinder_terrain_offset_cm(rangefinder_state.enabled && wp_nav->rangefinder_used(), rangefinder_state.alt_healthy, rangefinder_state.terrain_offset_cm);
#endif
    }
}

/**
 * @brief Get inertially-interpolated rangefinder height estimate
 * 
 * @details This helper function retrieves a rangefinder height estimate that has been
 *          interpolated using inertial navigation data to provide a smooth, high-rate
 *          altitude estimate even when rangefinder updates are slower than the main loop.
 *          
 *          Inertial interpolation combines:
 *          - Rangefinder measurements (typically 10-20Hz)
 *          - Accelerometer-derived vertical velocity (400Hz)
 *          - EKF altitude estimates (400Hz)
 *          
 *          This provides:
 *          - Smooth altitude transitions between rangefinder updates
 *          - Reduced latency in altitude control loops
 *          - Better handling of rangefinder dropouts
 *          - Consistent altitude rate regardless of sensor update frequency
 *          
 *          The interpolated height is used by:
 *          - Altitude hold modes for smooth control
 *          - Terrain following for continuous ground tracking
 *          - Landing detection for precise touchdown sensing
 *          - Surface tracking for maintaining height above terrain
 * 
 * @param[out] ret Interpolated rangefinder height in centimeters (distance to ground)
 * 
 * @return true if interpolated height is available and valid
 * @return false if rangefinder is disabled, unhealthy, or AP_RANGEFINDER not compiled in
 * 
 * @note Height is returned in centimeters for consistency with ArduPilot altitude units
 * 
 * @note When AP_RANGEFINDER_ENABLED is not defined (rangefinder support compiled out),
 *       this function always returns false without attempting to access rangefinder_state
 * 
 * @warning The returned height may lag actual terrain by the rangefinder update period
 *          (typically 50-100ms). Control loops should account for this latency.
 * 
 * @see RangeFinderState::get_rangefinder_height_interpolated_cm() for interpolation algorithm
 * @see Copter::update_rangefinder_terrain_offset() for terrain offset estimation
 * 
 * Source: ArduCopter/sensors.cpp:71-78
 */
bool Copter::get_rangefinder_height_interpolated_cm(int32_t& ret) const
{
#if AP_RANGEFINDER_ENABLED
    // Retrieve inertially-interpolated height from rangefinder state manager
    // Returns smooth, high-rate altitude estimate between rangefinder updates
    return rangefinder_state.get_rangefinder_height_interpolated_cm(ret);
#else
    // Rangefinder support not compiled in - return false without accessing rangefinder_state
    // This prevents compilation errors when AP_RANGEFINDER_ENABLED is not defined
    return false;
#endif
}
