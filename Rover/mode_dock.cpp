/**
 * @file mode_dock.cpp
 * @brief Implementation of Dock mode for autonomous docking using precision landing IR beacon or visual target tracking
 * 
 * @details This mode enables autonomous docking of the rover to a fixed docking station
 *          using visual or IR beacon target tracking provided by the AC_PrecLand library.
 *          
 *          **Docking Algorithm Overview:**
 *          1. **Target Detection**: Verify docking target is acquired via AC_PrecLand before entering mode
 *          2. **Alignment**: Optional heading correction to approach dock head-on from desired direction
 *          3. **Approach**: Navigate toward target with progressive speed reduction as distance decreases
 *          4. **Final Docking**: Stop when within configured stopping distance or complete docking
 *          
 *          **State Machine:**
 *          - APPROACHING: Vehicle actively navigating toward dock target
 *          - DOCKING_COMPLETE: Vehicle has reached stopping distance and halted (rovers) or loitering (boats)
 *          
 *          **Heading Correction Feature:**
 *          When enabled (DOCK_HDG_CORR_EN), the mode creates a virtual moving target along
 *          the desired approach line. This forces the vehicle to align with the specified
 *          approach direction (DOCK_DIR) even when entering the mode at an angle.
 *          
 *          **Integration with AC_PrecLand:**
 *          - Uses rover.precland.target_acquired() to verify target lock before mode entry
 *          - Calls rover.precland.get_target_position_cm() for real-time target position
 *          - Respects camera orientation via rover.precland.get_orient() for reversed operation
 *          
 *          **Abort Conditions:**
 *          - Target lost during final approach (when force_real_target is active)
 *          - Position estimate unavailable from AHRS
 *          - Invalid parameters (missing DOCK_DIR when heading correction enabled)
 * 
 * @note This mode is only available when compiled with MODE_DOCK_ENABLED and AC_PRECLAND_ENABLED.
 *       The AC_PrecLand library must be configured with an appropriate backend (IR-Lock, Simulator, etc.).
 * 
 * @warning Safe Testing Procedures:
 *          - Always test in SITL simulation first with sim_precland.py
 *          - Start with slow DOCK_SPEED (0.5 m/s or less) during initial hardware testing
 *          - Ensure DOCK_STOP_DIST provides adequate stopping margin (recommend 0.5m minimum)
 *          - Use manual mode override capability during testing in case of unexpected behavior
 *          - Test on flat, obstacle-free surface with soft docking target
 *          - Monitor via GCS during approach to abort if needed
 *          - For boats: verify loiter mode works correctly before testing dock mode
 * 
 * @see AC_PrecLand library for target detection implementation
 * @see Rover::precland for precision landing interface
 * @see ModeDock class definition in mode.h
 */

#include "Rover.h"

#if MODE_DOCK_ENABLED

const AP_Param::GroupInfo ModeDock::var_info[] = {
    // @Param: _SPEED
    // @DisplayName: Dock mode speed
    // @Description: Vehicle speed limit in dock mode
    // @Units: m/s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("_SPEED", 1, ModeDock, speed, 0.0f),

    // @Param: _DIR
    // @DisplayName: Dock mode direction of approach
    // @Description: Compass direction in which vehicle should approach towards dock. -1 value represents unset parameter
    // @Units: deg
    // @Range: 0 360
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_DIR", 2, ModeDock, desired_dir, -1.00f),

    // @Param: _HDG_CORR_EN
    // @DisplayName: Dock mode heading correction enable/disable
    // @Description: When enabled, the autopilot modifies the path to approach the target head-on along desired line of approach in dock mode
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("_HDG_CORR_EN", 3, ModeDock, hdg_corr_enable, 0),

    // @Param: _HDG_CORR_WT
    // @DisplayName: Dock mode heading correction weight
    // @Description: This value describes how aggressively vehicle tries to correct its heading to be on desired line of approach
    // @Range: 0.00 0.90
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("_HDG_CORR_WT", 4, ModeDock, hdg_corr_weight, 0.75f),

    // @Param: _STOP_DIST
    // @DisplayName: Distance from docking target when we should stop
    // @Description: The vehicle starts stopping when it is this distance away from docking target
    // @Units: m
    // @Range: 0 2
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("_STOP_DIST", 5, ModeDock, stopping_dist, 0.30f),

    AP_GROUPEND
};

ModeDock::ModeDock(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

/// Maximum acceleration in m/s² when user has not configured acceleration limits
#define AR_DOCK_ACCEL_MAX              20.0

/**
 * @brief Initialize dock mode and validate preconditions for safe docking
 * 
 * @details This function performs critical pre-flight checks before allowing dock mode entry:
 *          
 *          **Validation Steps:**
 *          1. Verify AC_PrecLand is enabled and has acquired the docking target
 *          2. If heading correction enabled, validate DOCK_DIR parameter is set
 *          3. Configure speed and acceleration limits for safe approach
 *          4. Initialize position controller with appropriate motion limits
 *          5. Configure reversed operation if camera mounted on vehicle's back
 *          6. Pre-calculate desired approach heading vector
 *          
 *          **Speed Limit Configuration:**
 *          Uses DOCK_SPEED parameter if set, otherwise falls back to waypoint navigation
 *          default speed. This allows independent tuning of docking approach speed.
 *          
 *          **Acceleration Limits:**
 *          Takes the minimum of configured waypoint navigation acceleration and
 *          attitude controller limits to ensure smooth, controlled approach. If no
 *          limit is configured, uses AR_DOCK_ACCEL_MAX (20 m/s²).
 *          
 *          **Camera Orientation:**
 *          Supports rear-facing cameras (orient = 4) by reversing position controller
 *          direction. This allows docking with rear-mounted precision landing cameras.
 *          
 *          **Heading Correction Setup:**
 *          Pre-computes unit vector in desired approach direction (NED frame) for use
 *          in virtual target calculation during update loop. This vector remains constant
 *          throughout the docking maneuver.
 * 
 * @return true if mode successfully entered and docking can proceed
 * @return false if preconditions not met (target not acquired, invalid parameters)
 * 
 * @note This function is called by the mode switching logic before transitioning to dock mode
 * @warning Mode entry will fail if precision landing target is not visible - ensure target
 *          is in camera field of view before attempting to enter dock mode
 */
bool ModeDock::_enter()
{
    // Refuse to enter the mode if dock is not in sight
    // AC_PrecLand must be enabled and actively tracking a target
    if (!rover.precland.enabled() || !rover.precland.target_acquired()) {
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Dock: target not acquired");
        return false;
    }

    // Validate heading correction parameters
    // If heading correction is enabled, DOCK_DIR must be set to a valid compass direction (0-360°)
    if (hdg_corr_enable && is_negative(desired_dir)) {
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Dock: Set DOCK_DIR or disable heading correction");
        return false;
    }

    // Configure speed limit for docking approach
    // Use DOCK_SPEED parameter if configured, otherwise fall back to waypoint navigation default
    // This allows independent tuning of docking speed vs normal navigation speed
    const float speed_max = is_positive(speed) ? speed : g2.wp_nav.get_default_speed();
    
    // Determine acceleration limits from attitude controller
    // Use the more conservative (minimum) of configured acceleration and deceleration limits
    float atc_accel_max = MIN(g2.attitude_control.get_accel_max(), g2.attitude_control.get_decel_max());
    if (!is_positive(atc_accel_max)) {
        // Acceleration limit of zero means no limit configured, use maximum safe acceleration
        atc_accel_max = AR_DOCK_ACCEL_MAX;
    }
    
    // Calculate final acceleration limit as minimum of waypoint nav and attitude control limits
    const float accel_max = is_positive(g2.wp_nav.get_default_accel()) ? MIN(g2.wp_nav.get_default_accel(), atc_accel_max) : atc_accel_max;
    
    // Configure jerk limit (rate of change of acceleration) for smooth motion
    // If not configured, use acceleration value as jerk limit
    const float jerk_max = is_positive(g2.wp_nav.get_default_jerk()) ? g2.wp_nav.get_default_jerk() : accel_max;

    // Initialize position controller with configured motion limits
    // This sets maximum speed, acceleration, lateral turn acceleration, and jerk for smooth docking approach
    g2.pos_control.set_limits(speed_max, accel_max, g2.attitude_control.get_turn_lat_accel_max(), jerk_max);
    g2.pos_control.init();

    // Configure position controller direction based on camera mounting orientation
    // If camera is rear-facing (orientation = 4), reverse controller to approach dock backwards
    // This allows docking with rear-mounted precision landing cameras
    g2.pos_control.set_reversed(rover.precland.get_orient() == 4);

    // Pre-compute unit vector in desired approach direction (NED frame)
    // This vector defines the line along which the vehicle should approach the dock
    // Used during heading correction to create virtual moving target
    // desired_dir is in degrees (compass heading), convert to radians for trig functions
    _desired_heading_NE = Vector2f{cosf(radians(desired_dir)), sinf(radians(desired_dir))};

    // Initialize docking complete flag - will be set true when stopping distance reached
    _docking_complete = false;

    return true;
}

/**
 * @brief Main docking state machine - executes docking approach or maintains docked state
 * 
 * @details This function implements the core docking algorithm and is called at the main
 *          loop rate (typically 50 Hz for Rover). It manages the complete docking sequence:
 *          
 *          **State Machine Logic:**
 *          
 *          **State 1: DOCKING_COMPLETE**
 *          - Reached when vehicle is within DOCK_STOP_DIST or target lost during final approach
 *          - Rovers: Stop motors completely (zero throttle and steering)
 *          - Boats: Enter loiter mode to maintain position in water
 *          - No further navigation updates performed until mode exit
 *          
 *          **State 2: APPROACHING**
 *          - Active navigation toward docking target
 *          - Continuously update target position from AC_PrecLand
 *          - Calculate distance to destination for slowdown and completion detection
 *          - Apply heading correction if enabled (virtual moving target algorithm)
 *          - Progressive speed reduction as distance decreases
 *          - Transition to DOCKING_COMPLETE when stopping distance reached
 *          
 *          **Target Position Tracking:**
 *          - Primary: AC_PrecLand real-time target detection (real_dock_in_sight)
 *          - Fallback: EKF-based position calculation using last known dock position
 *          - Abort if position estimate fails (safety measure)
 *          
 *          **Force Real Target Mode:**
 *          When within _force_real_target_limit_cm (~1m typically), disable heading correction
 *          and require real target visibility. This ensures final approach accuracy but may
 *          abort docking if target tracking is lost.
 *          
 *          **Heading Correction Algorithm:**
 *          Creates a virtual moving target along the desired approach line to force vehicle
 *          alignment with specified dock direction (DOCK_DIR parameter). The virtual target
 *          is positioned at (DOCK_HDG_COR_WT * distance) from dock along approach line.
 *          
 *          Example: Vehicle 100m from dock, DOCK_HDG_COR_WT = 0.75
 *          - Virtual target placed 75m from dock (25m in front of vehicle)
 *          - Vehicle navigates toward virtual target
 *          - As vehicle advances, virtual target appears to move toward dock
 *          - Vehicle naturally aligns with desired approach direction
 *          - Virtual target "collapses" to dock position as vehicle reaches it
 *          
 *          **Abort Conditions:**
 *          - Position estimate unavailable from AHRS (emergency stop)
 *          - Target lost during forced real target phase (<1m from dock)
 * 
 * @note Called at main loop rate, must complete quickly to maintain real-time performance
 * @warning Target loss during final approach (<1m) will abort docking for safety
 * 
 * @see apply_slowdown() for speed reduction algorithm
 * @see calc_dock_pos_rel_vehicle_NE() for position calculation method
 */
void ModeDock::update()
{
    // ***** DOCKING COMPLETE STATE *****
    // If docking is complete, maintain docked state without further navigation
    if (_docking_complete) {
        // Boats loiter to maintain position, rovers stop completely
        // Loiter mode must be successfully initialized before calling update
        if (_loitering) {
            // Delegate to loiter mode for position holding in water
            rover.mode_loiter.update();
        } else {
            // For ground rovers, stop all motion
            stop_vehicle();
        }
        return;
    }

    // ***** APPROACHING STATE - ACTIVE DOCKING *****
    
    // Query AC_PrecLand for current real-time target position relative to EKF origin
    // Returns true if target currently visible, false if lost
    // Updates _dock_pos_rel_origin_cm with latest position when target visible
    const bool real_dock_in_sight = rover.precland.get_target_position_cm(_dock_pos_rel_origin_cm);
    
    // Calculate dock position relative to vehicle's current position
    // Uses EKF position estimate and last known dock position from AC_PrecLand
    // This allows temporary target loss during heading correction phase
    Vector2f dock_pos_rel_vehicle_cm;
    if (!calc_dock_pos_rel_vehicle_NE(dock_pos_rel_vehicle_cm)) {
        // Position estimate failed - emergency stop for safety
        // This indicates EKF failure or invalid state
        g2.motors.set_throttle(0.0f);
        g2.motors.set_steering(0.0f);
        return;
    }

    // Calculate distance to dock in meters for slowdown and completion detection
    _distance_to_destination = dock_pos_rel_vehicle_cm.length() * 0.01f;

    // Force real target mode when very close to dock
    // Within this distance, heading correction is disabled and real target visibility is required
    // This ensures final approach accuracy but may abort if target tracking fails
    // Typical limit is ~1m (_force_real_target_limit_cm = 100cm)
    const bool force_real_target = _distance_to_destination < _force_real_target_limit_cm * 0.01f;

    // ***** DOCKING COMPLETION DETECTION *****
    // Transition to DOCKING_COMPLETE state when:
    // 1. Within configured stopping distance (DOCK_STOP_DIST parameter), OR
    // 2. In forced real target phase AND target is lost (abort for safety)
    if (_distance_to_destination <= stopping_dist || (force_real_target && !real_dock_in_sight)) {
        _docking_complete = true;

        // Notify ground control station of successful docking
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Dock: Docking complete");

        // Boats need position holding after docking, initialize loiter mode
        if (rover.is_boat()) {
            // Attempt to enter loiter mode for position holding
            // If loiter initialization fails, _loitering remains false and vehicle will stop
            _loitering = rover.mode_loiter.enter();
        }
        return;
    }

    // Initialize target position to real dock position (default, no heading correction)
    Vector2f target_cm = _dock_pos_rel_origin_cm;

    // ***** HEADING CORRECTION ALGORITHM *****
    // Virtual moving target approach to force alignment with desired dock direction
    // Only active when:
    // - NOT in forced real target zone (outside final ~1m)
    // - Heading correction enabled (DOCK_HDG_CORR_EN = 1)
    if  (!force_real_target && hdg_corr_enable) {
        // Project vehicle-to-dock vector onto desired approach direction
        // This gives the distance along the approach line
        const float correction_vec_mag = hdg_corr_weight * dock_pos_rel_vehicle_cm.projected(_desired_heading_NE).length();
        
        // Create virtual target by moving back from dock along desired approach direction
        // Virtual target distance from dock = DOCK_HDG_COR_WT * (distance along approach line)
        // Example: If 100m away on approach line and weight=0.75, virtual target is 75m from dock
        // This creates a target 25m in front of vehicle that moves toward dock as vehicle advances
        target_cm = _dock_pos_rel_origin_cm - _desired_heading_NE * correction_vec_mag;
    }

    // Convert target position from centimeters to meters and pass to position controller
    const Vector2p target_pos { target_cm.topostype() * 0.01 };
    g2.pos_control.input_pos_target(target_pos, rover.G_Dt);
    g2.pos_control.update(rover.G_Dt);

    // Get desired speed and turn rate from position controller
    // These are the raw outputs before slowdown is applied
    float desired_speed = g2.pos_control.get_desired_speed();
    float desired_turn_rate = g2.pos_control.get_desired_turn_rate_rads();

    // Apply progressive speed reduction as vehicle approaches dock
    // Speed decreases based on distance to dock and lateral position error
    desired_speed = apply_slowdown(desired_speed);

    // Execute steering and throttle control to achieve desired motion
    calc_steering_from_turn_rate(desired_turn_rate);
    calc_throttle(desired_speed, true);

#if HAL_LOGGING_ENABLED
    // Log docking telemetry for analysis and debugging
    // Captures dock position, target position, and control outputs at main loop rate
    // Useful for tuning heading correction weight and analyzing approach trajectories
    
    // @LoggerMessage: DOCK
    // @Description: Dock mode target information
    // @Field: TimeUS: Time since system startup
    // @Field: DockX: Docking Station position, X-Axis
    // @Field: DockY: Docking Station position, Y-Axis
    // @Field: DockDist: Distance to docking station
    // @Field: TPosX: Current Position Target, X-Axis
    // @Field: TPosY: Current Position Target, Y-Axis
    // @Field: DSpd: Desired speed
    // @Field: DTrnRt: Desired Turn Rate

        AP::logger().WriteStreaming(
            "DOCK",
            "TimeUS,DockX,DockY,DockDist,TPosX,TPosY,DSpd,DTrnRt",
            "smmmmmnE",
            "FBB0BB00",
            "Qfffffff",
            AP_HAL::micros64(),
            _dock_pos_rel_origin_cm.x,
            _dock_pos_rel_origin_cm.y,
            _distance_to_destination,
            target_cm.x,
            target_cm.y,
            desired_speed,
            desired_turn_rate);
#endif
}

/**
 * @brief Apply progressive speed reduction as vehicle approaches docking target
 * 
 * @details Implements a multi-factor speed control algorithm to ensure smooth, accurate
 *          final approach to the dock. Speed reduction is based on:
 *          1. Distance to target (closer = slower)
 *          2. Lateral position error (off-track = slower)
 *          3. Stopping distance requirement (prevent overshoot)
 *          
 *          **Speed Reduction Algorithm:**
 *          
 *          **Factor 1: Distance-Based Slowdown**
 *          - No slowdown beyond 15m from dock (dock_slow_dist_max_m)
 *          - Linear slowdown weight increase from 15m to 5m (dock_slow_dist_min_m)
 *          - Maximum slowdown within 5m of dock
 *          
 *          **Factor 2: Lateral Error Slowdown**
 *          - Calculates lateral (cross-track) error in vehicle body frame
 *          - Higher lateral error → greater speed reduction
 *          - Normalized error ratio against _acceptable_pos_error_cm
 *          - Combined with distance-based slowdown weight
 *          
 *          **Factor 3: Kinematic Stopping Constraint**
 *          - Applies physics-based speed limit to prevent overshoot
 *          - Maximum speed = sqrt(2 * remaining_distance * max_deceleration)
 *          - Ensures vehicle can stop before reaching DOCK_STOP_DIST
 *          
 *          **Minimum Speed Floor:**
 *          Speed never reduced below dock_speed_slowdown_lmt (0.5 m/s) to maintain
 *          controllability and prevent stalling.
 *          
 *          **Coordinate Frame Transformations:**
 *          Converts target vector from NED (North-East-Down) frame to vehicle body frame
 *          to calculate forward distance (body.x) and lateral error (body.y).
 * 
 * @param[in] desired_speed Raw desired speed from position controller (m/s)
 * @return float Modified speed with slowdown applied (m/s)
 * 
 * @note Speed sign preserved for reversed operation (rear-facing cameras)
 * @warning Ensure position controller max deceleration is achievable by vehicle to prevent overshoot
 */
float ModeDock::apply_slowdown(float desired_speed)
{
    // Minimum speed threshold - slowdown not applied below this value
    // Maintains minimum forward motion for controllability
    const float dock_speed_slowdown_lmt = 0.5f;

    // No slowdown for speeds already below minimum threshold
    if (fabsf(desired_speed) < dock_speed_slowdown_lmt) {
        return desired_speed;
    }

    // Calculate dock position relative to vehicle in NED frame
    Vector3f target_vec_rel_vehicle_NED;
    if(!calc_dock_pos_rel_vehicle_NE(target_vec_rel_vehicle_NED.xy())) {
        // Position calculation failed, return unmodified speed (safety fallback)
        return desired_speed;
    }

    // Transform target vector from NED frame to vehicle body frame
    // Body frame: X=forward, Y=right, Z=down
    // This allows calculation of forward distance and lateral error
    const Matrix3f &body_to_ned = AP::ahrs().get_rotation_body_to_ned();
    Vector3f target_vec_body = body_to_ned.mul_transpose(target_vec_rel_vehicle_NED);
    
    // Calculate lateral (cross-track) position error in centimeters
    // Absolute value since error magnitude matters, not direction
    const float target_error_cm = fabsf(target_vec_body.y);
    
    // Normalize lateral error against acceptable error threshold
    // Higher error ratio → more aggressive slowdown
    float error_ratio = target_error_cm / _acceptable_pos_error_cm;
    error_ratio = constrain_float(error_ratio, 0.0f, 1.0f);

    // Define distance thresholds for progressive slowdown
    const float dock_slow_dist_max_m = 15.0f;  // No slowdown beyond this distance
    const float dock_slow_dist_min_m = 5.0f;   // Full slowdown within this distance
    
    // Calculate distance-based slowdown weight
    // Beyond 15m: weight = 0 (no slowdown)
    // Between 15m and 5m: weight increases linearly
    // Within 5m: weight = 1 (maximum slowdown)
    float slowdown_weight = 1 - (target_vec_body.x * 0.01f - dock_slow_dist_min_m) / (dock_slow_dist_max_m - dock_slow_dist_min_m);
    slowdown_weight = constrain_float(slowdown_weight, 0.0f, 1.0f);
    
    // Apply combined slowdown based on lateral error and distance
    // Maintains minimum speed floor (dock_speed_slowdown_lmt)
    // Reduction formula: speed = speed * (1 - error_ratio * distance_weight)
    desired_speed = MAX(dock_speed_slowdown_lmt, fabsf(desired_speed) * (1 - error_ratio * slowdown_weight));

    // Apply kinematic constraint to prevent overshoot
    // Maximum speed limited by stopping distance physics: v_max = sqrt(2 * distance * deceleration)
    // Ensures vehicle can decelerate to stop before reaching DOCK_STOP_DIST
    desired_speed = MIN(desired_speed, safe_sqrt(2 * fabsf(_distance_to_destination - stopping_dist) * g2.pos_control.get_accel_max()));

    // Restore speed sign for reversed operation
    // If position controller is reversed (rear-facing camera), speed must be negative
    if (g2.pos_control.get_reversed()) {
        desired_speed *= -1;
    }

    return desired_speed;
}

/**
 * @brief Calculate position of dock relative to vehicle using EKF and last known dock position
 * 
 * @details This function provides dock position even when AC_PrecLand temporarily loses
 *          target visibility. This is essential during heading correction maneuvers where
 *          the vehicle may turn away from the target briefly while aligning approach direction.
 *          
 *          **Position Calculation Method:**
 *          1. Get vehicle's current position from AHRS relative to EKF origin (NED frame)
 *          2. Use last known dock position from AC_PrecLand (stored in _dock_pos_rel_origin_cm)
 *          3. Calculate dock position relative to vehicle: dock_pos - vehicle_pos
 *          
 *          **Key Assumptions:**
 *          - Dock is stationary relative to EKF origin (valid assumption for fixed docking stations)
 *          - EKF origin remains stable during docking maneuver
 *          - Last known dock position from AC_PrecLand is recent and accurate
 *          
 *          **Use Cases:**
 *          - Heading correction when vehicle turns away from target
 *          - Brief target occlusion during approach
 *          - Smooth navigation during momentary target tracking glitches
 *          
 *          **Failure Modes:**
 *          Returns false if AHRS cannot provide vehicle position, indicating:
 *          - EKF not initialized or unhealthy
 *          - No valid position estimate available
 *          - Severe navigation failure requiring emergency stop
 * 
 * @param[out] dock_pos_rel_vehicle Vector from vehicle to dock in NE (North-East) frame, units: cm
 * @return true Position calculated successfully
 * @return false AHRS position unavailable, docking should abort
 * 
 * @note Position is in centimeters to maintain precision for short-range docking
 * @note Coordinate frame is NED (North-East-Down) aligned with EKF origin
 * 
 * @see AP_AHRS::get_relative_position_NE_origin_float() for vehicle position source
 * @see AC_PrecLand::get_target_position_cm() for dock position source
 */
bool ModeDock::calc_dock_pos_rel_vehicle_NE(Vector2f &dock_pos_rel_vehicle) const {
    // Get vehicle's current position from AHRS relative to EKF origin (meters, NED frame)
    Vector2f current_pos_m;
    if (!AP::ahrs().get_relative_position_NE_origin_float(current_pos_m)) {
        // Position estimate unavailable - indicates EKF failure or uninitialized state
        return false;
    }
 
    // Calculate dock position relative to vehicle
    // _dock_pos_rel_origin_cm: Last known dock position from AC_PrecLand (cm, relative to EKF origin)
    // current_pos_m * 100: Vehicle position converted from meters to centimeters
    // Result: Vector from vehicle to dock in NE plane (cm)
    dock_pos_rel_vehicle = _dock_pos_rel_origin_cm - current_pos_m * 100.0f;
    return true;
}
#endif // MODE_DOCK_ENABLED
