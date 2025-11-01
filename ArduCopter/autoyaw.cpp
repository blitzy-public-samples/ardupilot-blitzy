/**
 * @file autoyaw.cpp
 * @brief Automatic yaw control implementation for ArduCopter
 * 
 * @details This file implements the automatic yaw control system for multicopters,
 *          managing vehicle heading during autonomous flight. The system supports
 *          multiple yaw control modes including:
 *          - HOLD: Maintain current heading
 *          - LOOK_AT_NEXT_WP: Point towards next waypoint during navigation
 *          - ROI: Point towards a Region Of Interest location
 *          - FIXED: Slew to and maintain a specific heading
 *          - LOOK_AHEAD: Point in the direction of travel
 *          - RATE: Control yaw at a specified rate
 *          - CIRCLE: Follow circle navigation heading
 *          - PILOT_RATE: Allow pilot yaw rate input during auto modes
 *          - WEATHERVANE: Automatic yaw into wind
 * 
 *          The auto yaw system integrates with waypoint navigation, ROI tracking,
 *          and supports rate limiting through yaw slewing for smooth heading changes.
 * 
 * @note This is a critical component of autonomous flight control, ensuring proper
 *       vehicle orientation for cameras, sensors, and flight path following.
 * 
 * @see Mode.h for AutoYaw class definition and Mode enumeration
 * @see Copter::attitude_control for low-level yaw control execution
 */

#include "Copter.h"

// Global auto_yaw instance for managing automatic yaw control state
Mode::AutoYaw Mode::auto_yaw;

/**
 * @brief Calculate heading towards Region Of Interest (ROI) location
 * 
 * @details Computes the yaw angle required to point the vehicle towards the
 *          stored ROI location. Uses vehicle's current position relative to
 *          the EKF origin to calculate bearing to the ROI.
 * 
 *          Coordinate system: NED (North-East-Down) frame
 *          Units: Position in meters, bearing in radians
 * 
 * @return float Yaw angle in radians pointing towards ROI, or current attitude
 *              target yaw if position is unavailable
 * 
 * @note Falls back to current attitude target if AHRS position is unavailable
 * @note ROI location is stored in centimeters, position is in meters (conversion applied)
 * 
 * @see set_roi() for setting the ROI location
 * @see get_bearing_rad() for bearing calculation
 */
float Mode::AutoYaw::roi_yaw_rad() const
{
    Vector2f pos_ne_m;
    // Get current position in NED frame relative to EKF origin
    if (AP::ahrs().get_relative_position_NE_origin_float(pos_ne_m)){
        // Calculate bearing from current position to ROI (convert position m to cm)
        return get_bearing_rad(pos_ne_m * 100.0, roi.xy());
    }
    // Fallback: return current attitude target yaw if position unavailable
    return copter.attitude_control->get_att_target_euler_rad().z;
}

/**
 * @brief Calculate yaw angle pointing in the direction of horizontal motion
 * 
 * @details Computes the heading to align with the vehicle's velocity vector,
 *          allowing the copter to face the direction it's traveling. Updates
 *          the look-ahead yaw only when horizontal speed exceeds the minimum
 *          threshold to avoid erratic yaw changes at low speeds.
 * 
 *          Algorithm:
 *          1. Get velocity vector in NED frame from AHRS
 *          2. Calculate horizontal speed squared (converts m/s to cm/s)
 *          3. If speed exceeds threshold, compute bearing from velocity
 *          4. Otherwise maintain previous look-ahead yaw
 * 
 * @return float Yaw angle in radians aligned with direction of travel
 * 
 * @note Speed threshold (YAW_LOOK_AHEAD_MIN_SPEED) prevents heading changes
 *       during hover or very slow movement
 * @note Velocity is in m/s, speed comparison uses cm/s (factor of 100)
 * @note Uses atan2 to preserve correct quadrant of velocity vector
 * 
 * @see YAW_LOOK_AHEAD_MIN_SPEED for minimum speed threshold
 * @see Mode::LOOK_AHEAD in yaw mode enumeration
 */
float Mode::AutoYaw::look_ahead_yaw_rad()
{
    // Get current velocity in NED frame (North-East-Down)
    Vector3f vel_ned_ms;
    if (copter.position_ok() && AP::ahrs().get_velocity_NED(vel_ned_ms)) {
        // Calculate horizontal speed squared in cm/s (vel is in m/s, multiply by 100)
        const float speed_sq = vel_ned_ms.xy().length_squared() * 10000.0;
        // Only update yaw when speed exceeds minimum threshold to avoid jitter
        if (speed_sq > (YAW_LOOK_AHEAD_MIN_SPEED * YAW_LOOK_AHEAD_MIN_SPEED)) {
            // Calculate bearing from velocity vector (atan2 preserves quadrant)
            _look_ahead_yaw_rad = atan2f(vel_ned_ms.y,vel_ned_ms.x);
        }
        // If below threshold, maintain previous look-ahead yaw
    }
    return _look_ahead_yaw_rad;
}

/**
 * @brief Set yaw mode to the default behavior based on WP_YAW_BEHAVIOR parameter
 * 
 * @param[in] rtl True if vehicle is in RTL (Return To Launch) mode, affects
 *                default mode selection for some parameter settings
 * 
 * @note This is typically called when clearing ROI or resetting yaw behavior
 * @see default_mode() for parameter-based mode selection logic
 */
void Mode::AutoYaw::set_mode_to_default(bool rtl)
{
    set_mode(default_mode(rtl));
}

/**
 * @brief Determine default yaw mode based on WP_YAW_BEHAVIOR parameter
 * 
 * @details Selects the appropriate yaw control mode based on the user-configured
 *          WP_YAW_BEHAVIOR parameter, which controls how the vehicle orients
 *          during waypoint navigation:
 * 
 *          WP_YAW_BEHAVIOR_NONE (0): Hold current heading (HOLD mode)
 *          WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP (1): Point towards next waypoint
 *          WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL (2): Point at waypoint
 *              except during RTL, where heading is held
 *          WP_YAW_BEHAVIOR_LOOK_AHEAD (3): Point in direction of travel
 * 
 * @param[in] rtl True if vehicle is executing RTL, which affects mode selection
 *                for WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL
 * 
 * @return Mode The default yaw control mode for current flight situation
 * 
 * @note Parameter WP_YAW_BEHAVIOR is user-configurable and affects autonomous
 *       flight behavior globally
 * @note RTL special case allows maintaining heading during return for better
 *       video/camera stability
 * 
 * @see set_mode_to_default() for applying the default mode
 * @see Mode enumeration for available yaw control modes
 */
Mode::AutoYaw::Mode Mode::AutoYaw::default_mode(bool rtl) const
{
    switch (copter.g.wp_yaw_behavior) {

    case WP_YAW_BEHAVIOR_NONE:
        // Hold current heading, no automatic yaw changes
        return Mode::HOLD;

    case WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL:
        // Point at next waypoint normally, but hold heading during RTL
        if (rtl) {
            return Mode::HOLD;
        } else {
            return Mode::LOOK_AT_NEXT_WP;
        }

    case WP_YAW_BEHAVIOR_LOOK_AHEAD:
        // Point in direction of velocity vector
        return Mode::LOOK_AHEAD;

    case WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP:
    default:
        // Default behavior: point towards next waypoint
        return Mode::LOOK_AT_NEXT_WP;
    }
}

/**
 * @brief Set the automatic yaw control mode and perform mode initialization
 * 
 * @details Central state machine controller for automatic yaw behavior. Manages
 *          transitions between different yaw control modes, storing the previous
 *          mode for potential restoration (e.g., after pilot override or weathervane).
 * 
 *          Yaw Mode State Machine:
 *          - HOLD: Maintain current heading (no changes)
 *          - LOOK_AT_NEXT_WP: Point at next waypoint (initialized by wpnav)
 *          - ROI: Point at Region Of Interest location
 *          - FIXED: Slew to specific heading with rate limiting
 *          - LOOK_AHEAD: Point in direction of travel
 *          - RESET_TO_ARMED_YAW: Return to heading at arming
 *          - ANGLE_RATE: Combined angle and rate control
 *          - RATE: Pure rate control (e.g., for spins)
 *          - CIRCLE: Follow circle mode navigation
 *          - PILOT_RATE: Pilot yaw input during auto
 *          - WEATHERVANE: Automatic yaw into wind
 * 
 * @param[in] yaw_mode The desired yaw control mode to activate
 * 
 * @note Mode transitions are skipped if already in the requested mode
 * @note Previous mode is stored in _last_mode for restoration after temporary
 *       mode changes (e.g., pilot override, weathervane)
 * @note Each mode may perform specific initialization when activated
 * 
 * @warning This is called at high frequency - initialization should be lightweight
 * 
 * @see Mode enumeration for all available yaw control modes
 * @see get_heading() for mode execution logic
 */
void Mode::AutoYaw::set_mode(Mode yaw_mode)
{
    // Skip mode change if already in requested mode (optimization)
    if (_mode == yaw_mode) {
        return;
    }
    
    // Save current mode for potential restoration (e.g., after weathervane)
    _last_mode = _mode;
    _mode = yaw_mode;

    // Perform mode-specific initialization
    switch (_mode) {

    case Mode::HOLD:
        // Maintain current heading - no initialization needed
        break;

    case Mode::LOOK_AT_NEXT_WP:
        // Waypoint navigation controller will initialize heading when
        // wpnav's set_destination method is called - no action here
        break;

    case Mode::ROI:
        // ROI heading will be calculated in roi_yaw_rad()
        // No initialization required
        break;

    case Mode::FIXED:
        // Fixed heading mode - caller must set target via set_fixed_yaw_rad()
        // Target heading and slew rate are configured separately
        break;

    case Mode::LOOK_AHEAD:
        // Initialize look-ahead yaw to current heading to prevent sudden changes
        _look_ahead_yaw_rad = copter.ahrs.get_yaw_rad();
        break;

    case Mode::RESET_TO_ARMED_YAW:
        // Return to heading at arming time
        // initial_armed_bearing_rad is set during arming - no init needed here
        break;

    case Mode::ANGLE_RATE:
        // Combined angle/rate control - targets set by caller
        break;

    case Mode::RATE:
        // Pure rate control - initialize rate to zero for smooth start
        _yaw_rate_rads = 0.0;
        break;

    case Mode::CIRCLE:
    case Mode::PILOT_RATE:
    case Mode::WEATHERVANE:
        // These modes update targets dynamically - no initialization required
        break;
    }
}

/**
 * @brief Set target heading for FIXED yaw mode with rate-limited slewing
 * 
 * @details Configures the vehicle to slew to a specific heading at a controlled
 *          rate, providing smooth yaw transitions. This is commonly used by
 *          MAVLink commands (e.g., MAV_CMD_CONDITION_YAW) to command heading
 *          changes during autonomous flight.
 * 
 *          Algorithm:
 *          1. Calculate yaw offset (relative or absolute) respecting direction
 *          2. Apply direction constraint (shortest/CW/CCW path)
 *          3. Set slew rate (rate limiting for smooth transitions)
 *          4. Activate FIXED mode
 * 
 *          The actual slewing is performed in yaw_rad() at each update cycle.
 * 
 * @param[in] yaw_rad Target heading in radians (relative or absolute based on
 *                    relative_angle parameter)
 * @param[in] yaw_rate_rads Maximum slew rate in rad/s (0 = use default max rate)
 * @param[in] direction Turn direction: >0 = CW, <0 = CCW, 0 = shortest path
 * @param[in] relative_angle True for relative angle (to current heading),
 *                           false for absolute angle (North-referenced)
 * 
 * @note Slew rate is capped at attitude controller's maximum to prevent instability
 * @note Direction parameter forces turn direction, overriding shortest path
 * @note For relative angles, if currently in HOLD mode, initializes from current heading
 * @note Updates are time-based, so slewing progresses at correct rate regardless
 *       of call frequency
 * 
 * @warning Excessive slew rates can cause attitude control instability
 * 
 * @see yaw_rad() for slewing implementation
 * @see reached_fixed_yaw_target() for checking completion
 * @see Mode::FIXED in yaw mode enumeration
 */
void Mode::AutoYaw::set_fixed_yaw_rad(float yaw_rad, float yaw_rate_rads, int8_t direction, bool relative_angle)
{
    // Initialize timestamp for rate-based slewing
    _last_update_ms = millis();
    const float angle_rad = yaw_rad;

    // Calculate yaw offset based on angle interpretation (relative vs absolute)
    if (relative_angle) {
        // Relative angle: offset from current heading
        if (_mode == Mode::HOLD) {
            // If holding, initialize base angle from current attitude
            _yaw_angle_rad = copter.ahrs.get_yaw_rad();
        }
        // Apply direction multiplier to relative offset
        _fixed_yaw_offset_rad = angle_rad * (direction >= 0 ? 1.0 : -1.0);
    } else {
        // Absolute angle: calculate shortest offset to target
        _fixed_yaw_offset_rad = wrap_PI(angle_rad - _yaw_angle_rad);
        
        // Apply direction constraints (force CW or CCW if specified)
        if (direction < 0 && is_positive(_fixed_yaw_offset_rad)) {
            // Force counter-clockwise: subtract full circle if offset is positive
            _fixed_yaw_offset_rad -= M_2PI;
        } else if (direction > 0 && is_negative(_fixed_yaw_offset_rad)) {
            // Force clockwise: add full circle if offset is negative
            _fixed_yaw_offset_rad += M_2PI;
        }
        // If direction == 0, use shortest path (no adjustment needed)
    }

    // Configure slew rate (rate limiting for smooth yaw changes)
    if (!is_positive(yaw_rate_rads)) {
        // No rate specified: use attitude controller's default maximum
        _fixed_yaw_slewrate_rads = copter.attitude_control->get_slew_yaw_max_rads();
    } else {
        // Use requested rate, capped at controller maximum to prevent instability
        _fixed_yaw_slewrate_rads = MIN(copter.attitude_control->get_slew_yaw_max_rads(), yaw_rate_rads);
    }

    // Activate FIXED yaw mode to begin slewing
    set_mode(Mode::FIXED);
}

/**
 * @brief Set combined yaw angle and rate targets for ANGLE_RATE mode
 * 
 * @details Sets both a target yaw angle and angular rate simultaneously,
 *          typically used by external position control systems (e.g., MAVLink
 *          SET_POSITION_TARGET commands with yaw and yaw_rate fields).
 * 
 *          In ANGLE_RATE mode, the yaw angle is integrated with the yaw rate
 *          over time, allowing smooth angle changes at specified rates.
 * 
 * @param[in] yaw_angle_rad Target yaw angle in radians (NED frame, 0 = North)
 * @param[in] yaw_rate_rads Target yaw rate in rad/s (positive = clockwise)
 * 
 * @note Angle is integrated with rate in yaw_rad() at each update
 * @note Timestamp is recorded to enable proper rate integration
 * 
 * @see Mode::ANGLE_RATE in yaw mode enumeration
 * @see yaw_rad() for integration implementation
 */
void Mode::AutoYaw::set_yaw_angle_and_rate_rad(float yaw_angle_rad, float yaw_rate_rads)
{
    // Record timestamp for rate integration
    _last_update_ms = millis();

    // Set both angle and rate targets
    _yaw_angle_rad = yaw_angle_rad;
    _yaw_rate_rads = yaw_rate_rads;

    // Activate ANGLE_RATE mode for combined control
    set_mode(Mode::ANGLE_RATE);
}

/**
 * @brief Apply a relative yaw angle offset from the current heading
 * 
 * @details Adjusts the current yaw angle target by a specified offset in degrees,
 *          useful for incremental heading adjustments during autonomous flight.
 *          Sets yaw rate to zero for immediate angle tracking.
 * 
 * @param[in] yaw_angle_offset_deg Relative yaw offset in degrees (positive = CW)
 * 
 * @note Offset is added to current _yaw_angle_rad and wrapped to [0, 2π]
 * @note Yaw rate is reset to zero (pure angle control after offset)
 * @note Angle is converted from degrees to radians internally
 * 
 * @see set_yaw_angle_and_rate_rad() for setting absolute angle and rate
 * @see Mode::ANGLE_RATE in yaw mode enumeration
 */
void Mode::AutoYaw::set_yaw_angle_offset_deg(const float yaw_angle_offset_deg)
{
    // Record timestamp for mode transition
    _last_update_ms = millis();

    // Apply relative offset and wrap to [0, 2π] range
    _yaw_angle_rad = wrap_2PI(_yaw_angle_rad + radians(yaw_angle_offset_deg));
    _yaw_rate_rads = 0.0f; // Reset rate for pure angle control

    // Activate ANGLE_RATE mode for angle tracking
    set_mode(Mode::ANGLE_RATE);
}

/**
 * @brief Set Region Of Interest (ROI) for automatic yaw and camera pointing
 * 
 * @details Configures the vehicle and camera mount to point at a specific
 *          geographic location. This is typically commanded via MAVLink
 *          MAV_CMD_DO_SET_ROI or MAV_CMD_NAV_ROI commands during missions.
 * 
 *          ROI Behavior:
 *          - If camera mount has pan control: mount points at ROI, vehicle
 *            yaw is not constrained (mount handles pointing)
 *          - If no pan control or no mount: vehicle yaws to point at ROI
 *          - Zero location (lat=0, lon=0, alt=0): clears ROI and returns to
 *            default yaw behavior
 * 
 *          Integration with Camera Mount:
 *          - ROI is sent to camera mount for gimbal control
 *          - Vehicle yaw control depends on mount capabilities
 *          - Clearing ROI also clears mount target
 * 
 * @param[in] roi_location Target location in geodetic coordinates (lat/lon/alt)
 *                         Use all zeros to clear/disable ROI
 * 
 * @note ROI position is stored in NEU (North-East-Up) frame in centimeters
 * @note Vehicle yaw control is only activated if mount lacks pan capability
 *       or no mount is present
 * @note Clearing ROI returns to default yaw mode (typically LOOK_AT_NEXT_WP)
 * 
 * @warning Currently only implements MAVLink ROI mode 3 (point at location)
 *          Other MAVLink modes (0,1,2,4) are not fully supported
 * 
 * @see roi_yaw_rad() for heading calculation towards ROI
 * @see Mode::ROI in yaw mode enumeration
 * @see MAV_CMD_DO_SET_ROI and MAV_CMD_NAV_ROI in MAVLink protocol
 */
void Mode::AutoYaw::set_roi(const Location &roi_location)
{
    // Check for ROI clear command (zero location)
    if (roi_location.alt == 0 && roi_location.lat == 0 && roi_location.lng == 0) {
        // Clear ROI: return to default yaw behavior
        // Note: This assumes waypoint navigation; more sophisticated logic could
        // restore mode based on active command type
        auto_yaw.set_mode_to_default(false);
#if HAL_MOUNT_ENABLED
        // Clear camera mount ROI target
        copter.camera_mount.clear_roi_target();
#endif  // HAL_MOUNT_ENABLED
    } else {
        // Set ROI to specified location
#if HAL_MOUNT_ENABLED
        // Check mount capabilities to determine if vehicle yaw is needed
        if (!copter.camera_mount.has_pan_control()) {
            // Mount cannot pan independently: vehicle must yaw to point at ROI
            if (roi_location.get_vector_from_origin_NEU_cm(roi)) {
                auto_yaw.set_mode(Mode::ROI);
            }
        }
        // Always send ROI to camera mount for gimbal pointing
        copter.camera_mount.set_roi_target(roi_location);

        // @todo Expand MAV_CMD_DO_SET_ROI handling to support all MAVLink modes:
        //      Mode 0: Reset ROI (do nothing)
        //      Mode 1: Point at next waypoint
        //      Mode 2: Point at waypoint specified by parameter
        //      Mode 3: Point at location (lat/lon/alt) - CURRENTLY IMPLEMENTED
        //      Mode 4: Point at target with given ID (requires target tracking)
#else
        // No camera mount: vehicle yaw must point at ROI
        if (roi_location.get_vector_from_origin_NEU_cm(roi)) {
            auto_yaw.set_mode(Mode::ROI);
        }
#endif  // HAL_MOUNT_ENABLED
    }
}

/**
 * @brief Set constant yaw rate for pure rate control
 * 
 * @details Activates RATE mode with a specified constant yaw rate, typically
 *          used for spins, spirals, or MAVLink velocity commands with yaw rate.
 *          The vehicle will continuously yaw at this rate until commanded otherwise.
 * 
 * @param[in] turn_rate_rads Desired yaw rate in rad/s (positive = CW, negative = CCW)
 * 
 * @note Unlike FIXED mode, no angle target is set - pure open-loop rate control
 * @note Rate continues indefinitely until mode change or new rate command
 * 
 * @see Mode::RATE in yaw mode enumeration
 * @see rate_rads() for rate output to attitude controller
 */
void Mode::AutoYaw::set_rate_rad(float turn_rate_rads)
{
    set_mode(Mode::RATE);
    _yaw_rate_rads = turn_rate_rads;
}

/**
 * @brief Check if FIXED mode yaw target has been reached
 * 
 * @details Determines if the vehicle has successfully slewed to the target
 *          heading set by set_fixed_yaw_rad(). Used by mission commands to
 *          verify heading change completion before proceeding.
 * 
 *          Completion Criteria:
 *          1. Remaining offset has been slewed to zero
 *          2. Actual heading is within 2° of target angle
 * 
 * @return true if target heading reached (within 2° tolerance), false if still slewing
 * @return true if not in FIXED mode (should not happen - safety return)
 * 
 * @note 2° tolerance provides reasonable completion detection without excessive precision
 * @note Returns true immediately if not in FIXED mode (unexpected condition)
 * 
 * @see set_fixed_yaw_rad() for setting target heading
 * @see Mode::FIXED in yaw mode enumeration
 */
bool Mode::AutoYaw::reached_fixed_yaw_target()
{
    if (mode() != Mode::FIXED) {
        // Safety: return true if not in correct mode (should not happen)
        return true;
    }

    if (!is_zero(_fixed_yaw_offset_rad)) {
        // Still slewing: offset not yet reduced to zero
        return false;
    }

    // Check if actual heading is within tolerance (2 degrees)
    return (fabsf(wrap_PI(_yaw_angle_rad - copter.ahrs.get_yaw_rad())) <= radians(2));
}

/**
 * @brief Calculate target yaw angle based on active yaw control mode
 * 
 * @details Central yaw computation function that executes mode-specific logic
 *          to determine the target heading. Called by get_heading() at each
 *          control loop iteration (typically 400Hz).
 * 
 *          Mode-Specific Yaw Calculations:
 * 
 *          ROI: Calculate bearing to Region Of Interest location
 *          FIXED: Slew towards target heading at limited rate (rate limiting)
 *          LOOK_AHEAD: Point in direction of velocity vector
 *          RESET_TO_ARMED_YAW: Return to heading at arming time
 *          CIRCLE: Follow circle navigation controller heading
 *          ANGLE_RATE: Integrate yaw angle with rate (angle += rate * dt)
 *          RATE/WEATHERVANE/PILOT_RATE: Use current attitude target (rate-only modes)
 *          LOOK_AT_NEXT_WP: Follow waypoint navigation controller heading
 * 
 * @return float Target yaw angle in radians (NED frame, 0 = North, positive = CW)
 * 
 * @note Called at main loop rate (typically 400Hz) - must be efficient
 * @note FIXED mode implements yaw slewing with rate limiting for smooth transitions
 * @note ANGLE_RATE mode integrates rate over time for combined angle/rate control
 * @note Rate-only modes (RATE, WEATHERVANE, PILOT_RATE) track current attitude
 *       target rather than computing new angle
 * 
 * @warning FIXED mode slewing depends on consistent timing - do not call irregularly
 * 
 * @see get_heading() for integration with attitude controller
 * @see set_mode() for mode activation and initialization
 */
float Mode::AutoYaw::yaw_rad()
{
    switch (_mode) {

    case Mode::ROI:
        // Calculate bearing to point at Region Of Interest location
        _yaw_angle_rad = roi_yaw_rad();
        break;

    case Mode::FIXED: {
        // Slew to target heading with rate limiting
        // Calculate time step for rate-based slewing
        const uint32_t now_ms = millis();
        float dt = (now_ms - _last_update_ms) * 0.001; // Convert ms to seconds
        _last_update_ms = now_ms;
        
        // Calculate yaw increment limited by slew rate
        // Constrain offset reduction to +/- (slew_rate * dt)
        float yaw_angle_step_rad = constrain_float(_fixed_yaw_offset_rad, 
                                                    - dt * _fixed_yaw_slewrate_rads, 
                                                    dt * _fixed_yaw_slewrate_rads);
        
        // Reduce remaining offset and update target angle
        _fixed_yaw_offset_rad -= yaw_angle_step_rad;
        _yaw_angle_rad += yaw_angle_step_rad;
        break;
    }

    case Mode::LOOK_AHEAD:
        // Point vehicle in direction of travel (velocity vector)
        _yaw_angle_rad = look_ahead_yaw_rad();
        break;

    case Mode::RESET_TO_ARMED_YAW:
        // Return to heading captured at arming time
        _yaw_angle_rad = copter.initial_armed_bearing_rad;
        break;

    case Mode::CIRCLE:
#if MODE_CIRCLE_ENABLED
        // Follow circle navigation controller heading
        if (copter.circle_nav->is_active()) {
            _yaw_angle_rad = copter.circle_nav->get_yaw_rad();
        }
#endif
        break;

    case Mode::ANGLE_RATE:{
        // Integrate yaw rate into angle target (angle += rate * dt)
        const uint32_t now_ms = millis();
        float dt = (now_ms - _last_update_ms) * 0.001; // Convert ms to seconds
        _last_update_ms = now_ms;
        
        // Update angle by integrating rate over time step
        _yaw_angle_rad += _yaw_rate_rads * dt;
        break;
    }

    case Mode::RATE:
    case Mode::WEATHERVANE:
    case Mode::PILOT_RATE:
        // Rate-only modes: track current attitude target (no new angle computation)
        // Attitude controller manages the actual yaw based on rate commands
        _yaw_angle_rad = copter.attitude_control->get_att_target_euler_rad().z;
        break;

    case Mode::LOOK_AT_NEXT_WP:
    default:
        // Point towards next waypoint in navigation path
        // Uses position controller's yaw target which provides smooth waypoint tracking
        // (not instantaneous wp_bearing_deg to avoid excessive yaw rates during flight)
        _yaw_angle_rad = copter.pos_control->get_yaw_rad();
    break;
    }
    return _yaw_angle_rad;
}

/**
 * @brief Get target yaw rate for attitude controller
 * 
 * @details Returns the yaw rate component for attitude control, which varies
 *          based on the active yaw mode. Used in conjunction with yaw angle
 *          by get_heading() to provide combined angle+rate or rate-only control.
 * 
 *          Mode-Specific Yaw Rates:
 * 
 *          HOLD/ROI/FIXED/LOOK_AHEAD/RESET_TO_ARMED_YAW/CIRCLE:
 *              Rate = 0 (angle-only control, controller tracks angle target)
 * 
 *          LOOK_AT_NEXT_WP:
 *              Rate from position controller (smooth waypoint tracking)
 * 
 *          PILOT_RATE:
 *              Rate from pilot stick input (manual override during auto)
 * 
 *          ANGLE_RATE/RATE/WEATHERVANE:
 *              Rate preserved from mode configuration (combined or rate-only)
 * 
 * @return float Target yaw rate in rad/s (positive = clockwise, negative = CCW)
 * 
 * @note Rate is typically combined with yaw angle in attitude controller for
 *       feedforward control, improving tracking performance
 * @note Angle-only modes set rate to 0, letting controller compute rate from angle error
 * @note This is often set by MAVLink SET_POSITION_TARGET messages with yaw_rate field
 * 
 * @see yaw_rad() for yaw angle calculation
 * @see get_heading() for combining angle and rate into attitude command
 */
float Mode::AutoYaw::rate_rads()
{
    switch (_mode) {

    case Mode::HOLD:
    case Mode::ROI:
    case Mode::FIXED:
    case Mode::LOOK_AHEAD:
    case Mode::RESET_TO_ARMED_YAW:
    case Mode::CIRCLE:
        // Angle-only modes: zero rate (controller tracks angle target)
        _yaw_rate_rads = 0.0f;
        break;

    case Mode::LOOK_AT_NEXT_WP:
        // Waypoint navigation: use position controller's computed yaw rate
        // for smooth tracking during waypoint approach
        _yaw_rate_rads = copter.pos_control->get_yaw_rate_rads();
        break;

    case Mode::PILOT_RATE:
        // Pilot override: use pilot's commanded yaw rate from stick input
        _yaw_rate_rads = _pilot_yaw_rate_rads;
        break;

    case Mode::ANGLE_RATE:
    case Mode::RATE:
    case Mode::WEATHERVANE:
        // Rate-based modes: preserve rate set by mode configuration
        // (no update needed, _yaw_rate_rads already set)
        break;
    }

    // Return computed or configured yaw rate
    return _yaw_rate_rads;
}

/**
 * @brief Primary automatic yaw control function - computes heading command for attitude controller
 * 
 * @details Main execution function for automatic yaw control, called by autonomous flight
 *          modes at each control loop iteration. Implements the complete yaw control pipeline:
 * 
 *          Execution Pipeline:
 *          1. Check for pilot yaw input override (if allowed by flight mode)
 *          2. Update weathervane automatic yaw-into-wind (if enabled)
 *          3. Compute yaw angle target based on active mode
 *          4. Compute yaw rate target based on active mode
 *          5. Select heading control mode (Rate_Only vs Angle_And_Rate)
 *          6. Return complete heading command to attitude controller
 * 
 *          Pilot Override Logic:
 *          - If RC valid + flight mode allows pilot yaw + pilot input detected:
 *            Switch to PILOT_RATE mode for manual control
 *          - If in PILOT_RATE mode but RC invalid: Return to HOLD mode (failsafe)
 * 
 *          Heading Control Modes:
 *          - Rate_Only: Pure rate control (HOLD, RATE, PILOT_RATE, WEATHERVANE)
 *            Attitude controller tracks rate target only
 *          - Angle_And_Rate: Combined control (most modes)
 *            Attitude controller uses angle error + rate feedforward
 * 
 * @return AC_AttitudeControl::HeadingCommand Complete heading command containing:
 *         - yaw_angle_rad: Target yaw angle in radians
 *         - yaw_rate_rads: Target/feedforward yaw rate in rad/s
 *         - heading_mode: Control mode (Rate_Only or Angle_And_Rate)
 * 
 * @note Called at main loop rate (typically 400Hz) - performance critical
 * @note Pilot override takes precedence over automatic yaw control
 * @note Weathervane can override automatic yaw to maintain heading into wind
 * @note Control mode selection affects how attitude controller processes targets
 * 
 * @warning RC failsafe during PILOT_RATE mode will revert to HOLD, not previous
 *          automatic mode - this is a safety measure
 * 
 * @see yaw_rad() for angle computation
 * @see rate_rads() for rate computation
 * @see update_weathervane() for wind-relative heading control
 * @see AC_AttitudeControl::input_euler_angle_roll_pitch_yaw() for command execution
 */
AC_AttitudeControl::HeadingCommand Mode::AutoYaw::get_heading()
{
    // Process pilot yaw input for potential override
    _pilot_yaw_rate_rads = 0.0;
    if (rc().has_valid_input() && copter.flightmode->use_pilot_yaw()) {
        // Flight mode allows pilot yaw input - check for pilot command
        _pilot_yaw_rate_rads = copter.flightmode->get_pilot_desired_yaw_rate_rads();
        if (!is_zero(_pilot_yaw_rate_rads)) {
            // Pilot is commanding yaw - override automatic control
            auto_yaw.set_mode(AutoYaw::Mode::PILOT_RATE);
        }
    } else if (auto_yaw.mode() == AutoYaw::Mode::PILOT_RATE) {
        // RC failsafe or pilot control disabled - exit PILOT_RATE mode
        // Safety: revert to HOLD rather than previous automatic mode
        auto_yaw.set_mode(AutoYaw::Mode::HOLD);
    }

#if WEATHERVANE_ENABLED
    // Update weathervane for automatic yaw-into-wind control
    // May override current mode if weathervane conditions are met
    update_weathervane(_pilot_yaw_rate_rads);
#endif

    // Build heading command for attitude controller
    AC_AttitudeControl::HeadingCommand heading;
    heading.yaw_angle_rad = auto_yaw.yaw_rad();    // Get target angle from active mode
    heading.yaw_rate_rads = auto_yaw.rate_rads();  // Get target rate from active mode

    // Select heading control mode based on yaw mode type
    switch (auto_yaw.mode()) {
        case Mode::HOLD:
        case Mode::RATE:
        case Mode::PILOT_RATE:
        case Mode::WEATHERVANE:
            // Rate-only modes: attitude controller tracks rate target only
            // Angle is informational (tracks current attitude)
            heading.heading_mode = AC_AttitudeControl::HeadingMode::Rate_Only;
            break;
            
        case Mode::LOOK_AT_NEXT_WP:
        case Mode::ROI:
        case Mode::FIXED:
        case Mode::LOOK_AHEAD:
        case Mode::RESET_TO_ARMED_YAW:
        case Mode::ANGLE_RATE:
        case Mode::CIRCLE:
            // Angle+Rate modes: attitude controller uses angle error feedback
            // plus rate feedforward for improved tracking
            heading.heading_mode = AC_AttitudeControl::HeadingMode::Angle_And_Rate;
            break;
    }

    return heading;
}

/**
 * @brief Update weathervane automatic yaw-into-wind control
 * 
 * @details Interfaces with the weathervane library to automatically yaw the vehicle
 *          into the wind, improving stability and control during windy conditions.
 *          Particularly useful for maintaining camera pointing or reducing drift
 *          during position hold.
 * 
 *          Weathervane Operation:
 *          1. Check if current flight mode allows weathervaning
 *          2. Call weathervane controller with current flight state:
 *             - Pilot yaw input (for override detection)
 *             - Altitude above ground (weathervane may be disabled near ground)
 *             - Roll/pitch angles (wind estimation indicators)
 *             - Takeoff/landing status (weathervane disabled during these phases)
 *          3. If weathervane active: Override yaw mode to WEATHERVANE
 *          4. If weathervane deactivates: Restore previous yaw mode
 * 
 *          Mode Transition Logic:
 *          - When weathervane activates: Save current mode, switch to WEATHERVANE
 *          - When weathervane deactivates: Restore previous mode (or default if was HOLD)
 * 
 * @param[in] pilot_yaw_rads Pilot yaw input in rad/s (angle, rate, or RC input)
 *                           Used to detect pilot override and disable weathervane
 * 
 * @note Weathervane is conditionally compiled (#if WEATHERVANE_ENABLED)
 * @note Flight mode must explicitly allow weathervaning via allows_weathervaning()
 * @note Pilot yaw input disables weathervane (pilot override takes precedence)
 * @note Weathervane typically disabled during takeoff and landing for stability
 * @note Roll/pitch attitudes are used by weathervane for wind direction estimation
 * 
 * @see Mode::WEATHERVANE in yaw mode enumeration
 * @see AC_Weathervane::get_yaw_out() for weathervane controller implementation
 */
#if WEATHERVANE_ENABLED
void Mode::AutoYaw::update_weathervane(const float pilot_yaw_rads)
{
    // Check if current flight mode permits weathervaning
    if (!copter.flightmode->allows_weathervaning()) {
        return;
    }

    // Query weathervane controller for desired yaw rate
    float yaw_rate_cds;
    if (copter.g2.weathervane.get_yaw_out(yaw_rate_cds,                                     // Output: yaw rate
                                           rad_to_cd(pilot_yaw_rads),                        // Pilot yaw (for override)
                                           copter.flightmode->get_alt_above_ground_cm()*0.01,// Altitude (meters)
                                           copter.pos_control->get_roll_cd()-copter.attitude_control->get_roll_trim_cd(), // Roll angle
                                           copter.pos_control->get_pitch_cd(),               // Pitch angle
                                           copter.flightmode->is_taking_off(),               // Takeoff flag
                                           copter.flightmode->is_landing())) {               // Landing flag
        // Weathervane is active - override yaw control
        set_mode(Mode::WEATHERVANE);
        _yaw_rate_rads = cd_to_rad(yaw_rate_cds);  // Convert centidegrees/s to rad/s
        return;
    }

    // Weathervane not active - restore previous yaw mode if currently in WEATHERVANE
    if (mode() == Mode::WEATHERVANE) {
        // Clear weathervane yaw rate
        _yaw_rate_rads = 0.0;
        
        // Restore appropriate yaw mode
        if (_last_mode == Mode::HOLD) {
            // Previous mode was HOLD - return to default automatic mode
            set_mode_to_default(false);
        } else {
            // Restore specific previous mode
            set_mode(_last_mode);
        }
    }
}
#endif // WEATHERVANE_ENABLED
