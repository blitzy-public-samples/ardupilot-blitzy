/**
 * @file surface_tracking.cpp
 * @brief Surface tracking implementation for maintaining constant height above ground or below ceiling
 * 
 * @details This file implements the surface tracking system that allows ArduCopter to maintain
 *          a constant altitude relative to the terrain below (ground tracking) or ceiling above
 *          (ceiling tracking) using rangefinder measurements. The system continuously adjusts
 *          the position controller's vertical target to compensate for changes in terrain elevation.
 * 
 *          Key Features:
 *          - Rangefinder-based altitude control for terrain following
 *          - Automatic adjustment of climb rate for sloped terrain
 *          - Sensor health monitoring with timeout and glitch detection
 *          - Failover behavior when rangefinder becomes unavailable
 *          - Support for both downward (ground) and upward (ceiling) tracking
 * 
 *          Integration with Position Controller:
 *          The surface tracking system works by updating the position controller's terrain
 *          offset (pos_terrain_target_U_cm) which represents the terrain altitude above the
 *          EKF origin. This offset is continuously updated based on rangefinder measurements,
 *          allowing the vehicle to climb or descend automatically to maintain constant height
 *          above the surface.
 * 
 *          Sensor Fusion:
 *          While the primary altitude reference comes from the rangefinder, the system operates
 *          in conjunction with the barometer-based altitude hold through the position controller.
 *          The terrain offset effectively shifts the altitude target, with the barometer providing
 *          the absolute altitude reference and the rangefinder providing the relative adjustment.
 * 
 * @note This feature requires AP_RANGEFINDER_ENABLED
 * 
 * @warning Surface tracking is safety-critical during low-altitude flight. Loss of rangefinder
 *          can result in sudden altitude changes. Always monitor sensor health and configure
 *          appropriate failsafe actions.
 * 
 * @see AC_PosControl for position controller integration
 * @see AP_SurfaceDistance for rangefinder altitude and terrain offset calculations
 */

#include "Copter.h"

#if AP_RANGEFINDER_ENABLED

/**
 * @brief Update the vertical offset of the position controller to follow measured surface
 * 
 * @details This is the main surface tracking update function called at the main loop rate.
 *          It manages the position controller's terrain offset to maintain constant height
 *          above the ground or below the ceiling using rangefinder measurements.
 * 
 *          Algorithm Overview:
 *          1. Check for sensor timeout (last update > SURFACE_TRACKING_TIMEOUT_MS)
 *          2. Verify rangefinder health and glitch state
 *          3. If healthy: Update position controller terrain target with current rangefinder offset
 *          4. If timeout/unhealthy: Reset terrain offset to zero and flag for re-initialization
 *          5. Handle initialization when first activated or after glitch recovery
 * 
 *          Terrain Following Mechanics:
 *          The vehicle maintains constant height above terrain by continuously updating the
 *          position controller's terrain offset (set_pos_terrain_target_U_cm). As terrain
 *          elevation changes, this offset adjusts the absolute altitude target, causing the
 *          position controller to command climb or descent to maintain constant surface clearance.
 * 
 *          Sloped Terrain Handling:
 *          On sloped terrain, the terrain offset changes continuously as the vehicle moves.
 *          The position controller responds to these offset changes by adjusting climb rate
 *          proportionally to the rate of terrain elevation change and vehicle ground speed.
 *          This results in smooth terrain following without requiring explicit slope detection.
 * 
 *          Sensor Fusion:
 *          The system uses the rangefinder for relative altitude (surface distance) while
 *          the position controller continues to use the EKF's barometric altitude estimate
 *          as the absolute reference. The terrain_offset_cm from AP_SurfaceDistance represents
 *          the terrain elevation in the EKF's vertical coordinate frame, allowing seamless
 *          integration between the two sensors.
 * 
 *          Failover Behavior:
 *          When the rangefinder becomes unavailable (timeout, unhealthy, or glitches detected):
 *          - Terrain offset is reset to zero
 *          - Position controller reverts to absolute altitude hold
 *          - Target reset flag is set for clean re-initialization
 *          - No altitude change is commanded; vehicle maintains last barometric altitude
 *          - Upon rangefinder recovery, target is re-initialized to current terrain offset
 * 
 * @note Called at main loop rate (typically 400Hz for multicopters)
 * @note Requires either downward rangefinder (ground tracking) or upward rangefinder (ceiling tracking)
 * 
 * @warning Rangefinder glitches can cause sudden altitude adjustments. The glitch detection
 *          in AP_SurfaceDistance helps filter these, but momentary altitude deviations may occur.
 * 
 * @see Copter::rangefinder_alt_ok() for downward rangefinder health check
 * @see Copter::rangefinder_up_ok() for upward rangefinder health check
 * @see AC_PosControl::set_pos_terrain_target_U_cm() for terrain offset update
 * @see AC_PosControl::init_pos_terrain_U_cm() for terrain target initialization
 */
void Copter::SurfaceTracking::update_surface_offset()
{
    // Check for sensor timeout - if rangefinder hasn't provided valid data recently,
    // we consider surface tracking unavailable to prevent using stale measurements
    const uint32_t now_ms = millis();
    const bool timeout = (now_ms - last_update_ms) > SURFACE_TRACKING_TIMEOUT_MS;

    // Verify rangefinder health and tracking mode:
    // - For GROUND tracking: downward rangefinder must be healthy with no glitches
    // - For CEILING tracking: upward rangefinder must be healthy with no glitches
    // Glitch detection (glitch_count > 0) indicates inconsistent rangefinder readings
    // that could cause erratic altitude behavior if used for surface tracking
    if (((surface == Surface::GROUND) && copter.rangefinder_alt_ok() && (copter.rangefinder_state.glitch_count == 0)) ||
        ((surface == Surface::CEILING) && copter.rangefinder_up_ok() && (copter.rangefinder_up_state.glitch_count == 0))) {

        // Get the appropriate surface distance state based on tracking direction
        // The terrain_offset_cm is calculated by AP_SurfaceDistance library and represents
        // the terrain elevation above the EKF origin in centimeters. This offset accounts
        // for the vehicle's current altitude and rangefinder distance to derive terrain altitude.
        AP_SurfaceDistance &rf_state = (surface == Surface::GROUND) ? copter.rangefinder_state : copter.rangefinder_up_state;

        // Update position controller with terrain offset - this is the core of surface tracking
        // The position controller will adjust the vehicle's altitude target to maintain constant
        // height above the terrain as terrain_offset_cm changes with terrain elevation.
        // On sloped terrain, this offset changes continuously, causing the position controller
        // to command climb/descent automatically to maintain surface clearance.
        copter.pos_control->set_pos_terrain_target_U_cm(rf_state.terrain_offset_cm);
        last_update_ms = now_ms;
        valid_for_logging = true;

        // Initialize/reset the terrain target under specific conditions to ensure clean engagement:
        // 1. timeout: Surface tracking was inactive and is now re-engaging
        // 2. reset_target: Flag set by mode changes or manual reset requests
        // 3. glitch_cleared_ms changed: Rangefinder glitch just cleared, need fresh initialization
        // 
        // Initialization sets both current position and target to the terrain offset, preventing
        // sudden altitude changes that would occur if only the target were updated. This ensures
        // smooth engagement of surface tracking without altitude jumps.
        if (timeout ||
            reset_target ||
            (last_glitch_cleared_ms != rf_state.glitch_cleared_ms)) {
            copter.pos_control->init_pos_terrain_U_cm(rf_state.terrain_offset_cm);
            reset_target = false;
            last_glitch_cleared_ms = rf_state.glitch_cleared_ms;
        }

    } else {
        // Rangefinder is unavailable or unhealthy - failover to barometric altitude hold
        // This branch executes when:
        // - Rangefinder timeout (no recent valid measurements)
        // - Rangefinder reports unhealthy status
        // - Glitch detection triggered (inconsistent readings)
        // 
        // Failover behavior: Reset terrain offset to zero, reverting position controller
        // to absolute altitude hold using barometric altitude. The vehicle will maintain
        // its current barometric altitude without surface tracking adjustments. This prevents
        // using stale or unreliable rangefinder data that could cause dangerous altitude errors.
        if (timeout && !reset_target) {
            copter.pos_control->init_pos_terrain_U_cm(0);
            valid_for_logging = false;
            reset_target = true;  // Flag for re-initialization when rangefinder recovers
        }
    }
}

/**
 * @brief Initialize surface tracking when transitioning from terrain following
 * 
 * @details This function should be called by flight modes when switching from terrain
 *          following to surface tracking (e.g., ZigZag mode). It prevents re-initialization
 *          of the terrain target since the position controller already has a valid terrain
 *          offset from the previous terrain following system.
 * 
 *          Purpose:
 *          Terrain following and surface tracking both use the position controller's terrain
 *          offset mechanism (pos_terrain_target_U_cm), but obtain terrain data from different
 *          sources (terrain database vs rangefinder). When transitioning between these systems,
 *          we want to maintain continuity to avoid altitude jumps.
 * 
 *          Difference from update_surface_offset() initialization:
 *          - update_surface_offset() calls init_pos_terrain_U_cm() which sets both current
 *            position and target to the new offset
 *          - external_init() only updates internal state (last_update_ms, reset_target) without
 *            calling init_pos_terrain_U_cm(), assuming the target is already correct
 * 
 *          This allows smooth handoff from terrain database altitude to rangefinder altitude
 *          without re-initializing the position controller target.
 * 
 * @note Only validates and initializes for GROUND tracking with downward rangefinder
 * @note Caller must ensure terrain following was active and position controller target is valid
 * 
 * @see update_surface_offset() for normal surface tracking initialization
 * @see Mode::ZigZag for example usage during terrain-to-surface transition
 */
void Copter::SurfaceTracking::external_init()
{
    // Only proceed if ground tracking is configured and downward rangefinder is healthy
    // This ensures we have valid sensor data before taking over from terrain following
    if ((surface == Surface::GROUND) && copter.rangefinder_alt_ok() && (copter.rangefinder_state.glitch_count == 0)) {
        // Mark surface tracking as active by updating timestamp
        // This prevents timeout detection in update_surface_offset() from resetting the target
        last_update_ms = millis();
        
        // Clear reset flag to indicate target is valid and should not be re-initialized
        // The terrain offset is already correct from terrain following, so we just continue using it
        reset_target = false;
    }
}

/**
 * @brief Get the target surface distance for logging purposes
 * 
 * @details Retrieves the desired distance to the surface (ground or ceiling) from the
 *          position controller for logging. This represents the target height the vehicle
 *          is trying to maintain above the ground or below the ceiling.
 * 
 *          The target distance is derived from the position controller's desired vertical
 *          position (pos_desired_U_cm), which incorporates the terrain offset. By reading
 *          this value, we can log what altitude the surface tracking system is commanding.
 * 
 * @param[out] target_dist Target distance to surface in meters (positive for ground, negative for ceiling)
 * 
 * @return true if target distance is valid and was retrieved successfully
 * @return false if surface tracking is inactive or not valid for logging
 * 
 * @note Returns false if valid_for_logging is false or surface mode is NONE
 * @note Distance is converted from centimeters to meters for logging
 * @note Sign convention: positive distance for ground tracking, negative for ceiling tracking
 * 
 * @see get_dist_for_logging() for actual measured distance to surface
 * @see AC_PosControl::get_pos_desired_U_cm() for position controller desired altitude
 */
bool Copter::SurfaceTracking::get_target_dist_for_logging(float &target_dist) const
{
    // Check if surface tracking is active and valid
    // valid_for_logging is set true when rangefinder is healthy and providing updates
    if (!valid_for_logging || (surface == Surface::NONE)) {
        return false;
    }

    // Apply sign convention: positive for ground tracking (downward), negative for ceiling tracking (upward)
    // This matches the standard direction convention where ground distances are positive
    const float dir = (surface == Surface::GROUND) ? 1.0f : -1.0f;
    
    // Get desired vertical position from position controller and convert to meters
    // pos_desired_U_cm is the altitude target in the EKF's Up coordinate frame (centimeters)
    target_dist = dir * copter.pos_control->get_pos_desired_U_cm() * 0.01f;
    return true;
}

/**
 * @brief Get the actual measured surface distance for logging purposes
 * 
 * @details Returns the current rangefinder measurement of distance to the surface
 *          (ground or ceiling) for logging. This represents the actual measured height,
 *          which can be compared with the target distance from get_target_dist_for_logging()
 *          to monitor surface tracking performance.
 * 
 *          The measurement comes directly from the appropriate rangefinder state:
 *          - rangefinder_state.alt_cm for ground tracking (downward rangefinder)
 *          - rangefinder_up_state.alt_cm for ceiling tracking (upward rangefinder)
 * 
 * @return Measured distance to surface in meters
 * 
 * @note Distance is always returned as positive value regardless of tracking direction
 * @note Converts from centimeters to meters for logging (alt_cm * 0.01)
 * @note No validity check - caller should verify surface tracking is active
 * 
 * @see get_target_dist_for_logging() for commanded target distance
 * @see AP_SurfaceDistance::alt_cm for rangefinder altitude measurement
 */
float Copter::SurfaceTracking::get_dist_for_logging() const
{
    // Return rangefinder measurement from appropriate sensor:
    // - Upward rangefinder (rangefinder_up_state) for ceiling tracking
    // - Downward rangefinder (rangefinder_state) for ground tracking
    // Convert from centimeters to meters for logging
    return ((surface == Surface::CEILING) ? copter.rangefinder_up_state.alt_cm : copter.rangefinder_state.alt_cm) * 0.01f;
}

/**
 * @brief Set the surface tracking direction (ground or ceiling)
 * 
 * @details Configures surface tracking to follow either the ground below (using downward
 *          rangefinder) or ceiling above (using upward rangefinder). Validates that the
 *          required rangefinder orientation is available before changing the surface mode.
 * 
 *          Hardware Validation:
 *          Before changing surface mode, this function verifies that a rangefinder with
 *          the appropriate orientation is configured:
 *          - GROUND tracking requires ROTATION_PITCH_270 (downward-facing) rangefinder
 *          - CEILING tracking requires ROTATION_PITCH_90 (upward-facing) rangefinder
 * 
 *          If the required sensor is not available, the mode change is rejected and:
 *          - Warning message sent to ground control station
 *          - User mode change failure event triggered (audible/visual notification)
 *          - Surface tracking mode remains unchanged
 * 
 *          Initialization Trigger:
 *          When surface direction changes successfully, reset_target flag is set to true,
 *          which triggers re-initialization in update_surface_offset() on the next call.
 *          This ensures clean engagement with the new sensor and prevents using stale data.
 * 
 * @param[in] new_surface Desired surface tracking mode (GROUND, CEILING, or NONE)
 * 
 * @note No action taken if new_surface equals current surface mode
 * @note Mode change failure triggers AP_Notify user_mode_change_failed event
 * 
 * @warning Changing surface tracking direction during flight may cause momentary altitude
 *          adjustments as the system switches between different rangefinder sensors
 * 
 * @see update_surface_offset() for re-initialization when reset_target is set
 * @see AP_RangeFinder::has_orientation() for rangefinder orientation detection
 */
void Copter::SurfaceTracking::set_surface(Surface new_surface)
{
    // No action needed if already in requested mode - avoid unnecessary resets
    if (surface == new_surface) {
        return;
    }
    
    // Validate that required rangefinder hardware is present before changing mode
    // GROUND tracking requires downward-facing rangefinder (ROTATION_PITCH_270 = -90° pitch)
    if ((new_surface == Surface::GROUND) && !copter.rangefinder.has_orientation(ROTATION_PITCH_270)) {
        copter.gcs().send_text(MAV_SEVERITY_WARNING, "SurfaceTracking: no downward rangefinder");
        AP_Notify::events.user_mode_change_failed = 1;  // Trigger audible/visual failure notification
        return;
    }
    
    // CEILING tracking requires upward-facing rangefinder (ROTATION_PITCH_90 = +90° pitch)
    if ((new_surface == Surface::CEILING) && !copter.rangefinder.has_orientation(ROTATION_PITCH_90)) {
        copter.gcs().send_text(MAV_SEVERITY_WARNING, "SurfaceTracking: no upward rangefinder");
        AP_Notify::events.user_mode_change_failed = 1;  // Trigger audible/visual failure notification
        return;
    }
    
    // Hardware validation passed - update surface tracking mode
    surface = new_surface;
    
    // Flag for re-initialization in update_surface_offset()
    // This ensures clean engagement with the new sensor and prevents altitude jumps
    // from using stale data or incorrect sensor readings
    reset_target = true;
}

#endif  // AP_RANGEFINDER_ENABLED
