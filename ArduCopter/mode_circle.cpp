/**
 * @file mode_circle.cpp
 * @brief Circle flight mode implementation for multicopter
 * 
 * @details Circle mode autonomously flies the vehicle in a horizontal circle 
 *          around a point of interest at a specified radius and rate. This mode
 *          is commonly used for aerial photography and videography of stationary
 *          targets, allowing smooth orbital shots.
 * 
 *          Key Features:
 *          - Automatic circular orbit around center point
 *          - Configurable radius via CIRCLE_RADIUS parameter (default 1000cm/10m)
 *          - Configurable rate via CIRCLE_RATE parameter (deg/s, positive=clockwise)
 *          - Pilot retains altitude control via throttle stick
 *          - Pilot can adjust radius in real-time via pitch stick
 *          - Pilot can adjust rotation rate in real-time via roll stick
 *          - Automatic yaw-to-center tracking (vehicle faces circle center)
 *          - Optional mount/gimbal ROI (Region Of Interest) at circle center
 * 
 *          Circle Center Determination:
 *          The circle center is initialized based on the vehicle's velocity vector
 *          when the mode is entered. The AC_Circle library calculates the center
 *          position tangent to the current flight path.
 * 
 *          Coordinate System:
 *          - Positions in NED (North-East-Down) frame
 *          - Internal calculations in centimeters
 *          - Rates in degrees per second
 * 
 * @note This mode requires a good position estimate (GPS or non-GPS navigation)
 * @warning Ensure adequate space for the configured circle radius
 * 
 * @see AC_Circle library for core circle navigation implementation
 * @see CIRCLE_RADIUS parameter for radius configuration
 * @see CIRCLE_RATE parameter for rotation rate configuration
 * @see CIRCLE_OPTIONS parameter for additional behaviors (e.g., ROI at center)
 * 
 * Source: ArduCopter/mode_circle.cpp
 */

#include "Copter.h"
#include <AP_Mount/AP_Mount.h>

#if MODE_CIRCLE_ENABLED

/*
 * Init and run calls for circle flight mode
 */

/**
 * @brief Initialize Circle flight mode
 * 
 * @details Initializes the Circle mode controller and configures all necessary
 *          subsystems for autonomous circular flight. This function:
 *          
 *          1. Configures position controller speed and acceleration limits
 *             - Horizontal (NE) limits from WP_NAV parameters
 *             - Vertical (U/altitude) limits from pilot speed parameters
 *          
 *          2. Initializes AC_Circle navigation library
 *             - Calculates circle center based on current velocity vector
 *             - Sets initial radius from CIRCLE_RADIUS parameter
 *             - Sets initial rotation rate from CIRCLE_RATE parameter
 *          
 *          3. Optionally configures camera/gimbal mount ROI
 *             - If CIRCLE_OPTIONS includes roi_at_center bit
 *             - Points mount at circle center on terrain
 *             - Enables smooth tracking shots of ground target
 *          
 *          4. Configures automatic yaw control
 *             - Sets AutoYaw::Mode::CIRCLE mode
 *             - Vehicle continuously faces toward circle center
 * 
 * @param[in] ignore_checks If true, skip pre-flight checks (currently unused in this mode)
 * 
 * @return true if initialization successful
 * @return false if initialization failed (e.g., unable to calculate circle center location)
 * 
 * @note This function is called once when entering Circle mode from another mode
 * @note The circle center is calculated tangent to current velocity vector at mode entry
 * @note Speed and acceleration limits ensure smooth, stable circular flight
 * 
 * @warning Entering Circle mode at high speed may result in large initial radius
 * @warning Mount ROI calculation failure returns false, preventing mode entry
 * 
 * @see AC_Circle::init() for circle center calculation algorithm
 * @see AutoYaw::Mode::CIRCLE for automatic yaw-to-center behavior
 * @see CIRCLE_OPTIONS parameter for configuration options
 * 
 * Source: ArduCopter/mode_circle.cpp:11-43
 */
bool ModeCircle::init(bool ignore_checks)
{
    // Reset speed changing flag - tracks whether pilot is actively adjusting circle rate
    speed_changing = false;

    // Configure horizontal (North-East) speed and acceleration limits from WP_NAV parameters
    // These limits control how quickly the vehicle can move around the circle circumference
    pos_control->set_max_speed_accel_NE_cm(wp_nav->get_default_speed_NE_cms(), wp_nav->get_wp_acceleration_cmss());
    pos_control->set_correction_speed_accel_NE_cm(wp_nav->get_default_speed_NE_cms(), wp_nav->get_wp_acceleration_cmss());
    
    // Configure vertical (Up/altitude) speed and acceleration limits from pilot speed parameters
    // Allows pilot to control altitude independently during circular flight
    // Note: Down speed is negative, up speed is positive
    pos_control->set_max_speed_accel_U_cm(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_U_cmss(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // Initialize AC_Circle navigation library - calculates circle center based on current velocity
    // The center is positioned such that the vehicle's current velocity is tangent to the circle
    // This creates a smooth entry into circular flight without sharp direction changes
    copter.circle_nav->init();

#if HAL_MOUNT_ENABLED
    // Configure camera/gimbal mount to point at circle center if CIRCLE_OPTIONS bit is set
    // This enables automatic tracking shots with the camera continuously aimed at ground target
    if (copter.circle_nav->roi_at_center()) {
        // Get circle center position in NEU (North-East-Up) frame in centimeters
        const Vector3p &pos { copter.circle_nav->get_center_NEU_cm() };
        Location circle_center;
        
        // Convert from NED offset (cm) to global Location (lat/lon/alt)
        // Requires valid EKF origin - returns false if origin not available
        if (!AP::ahrs().get_location_from_origin_offset_NED(circle_center, pos * 0.01)) {
            return false;  // Cannot initialize Circle mode without valid position origin
        }
        
        // Set ROI altitude to ground level (terrain-relative)
        // This points the camera at the ground target rather than at vehicle altitude
        circle_center.set_alt_cm(0, Location::AltFrame::ABOVE_TERRAIN);
        
        // Command the mount/gimbal system to track this ROI (Region Of Interest)
        AP_Mount *s = AP_Mount::get_singleton();
        s->set_roi_target(circle_center);
    }
#endif

    // Configure automatic yaw control to face circle center continuously
    // Vehicle will rotate to maintain heading toward center throughout the orbit
    auto_yaw.set_mode(AutoYaw::Mode::CIRCLE);

    return true;
}

/**
 * @brief Main Circle mode control loop - called at scheduler rate (typically 400Hz)
 * 
 * @details Executes the Circle mode flight control algorithm every scheduler cycle.
 *          This function coordinates all subsystems required for autonomous circular
 *          flight while allowing pilot override of specific parameters.
 * 
 *          Control Flow:
 *          1. Update controller speed/acceleration limits from parameters
 *          2. Check for real-time parameter changes (CIRCLE_RADIUS, CIRCLE_RATE)
 *          3. Process pilot stick inputs (if valid RC signal):
 *             - Pitch stick: Adjust circle radius (forward=smaller, back=larger)
 *             - Roll stick: Adjust rotation rate (right=faster CW, left=faster CCW)
 *             - Throttle stick: Control altitude (standard climb/descend)
 *          4. Get pilot desired climb rate and apply avoidance adjustments
 *          5. Handle disarmed/landed state (safe ground handling)
 *          6. Update surface tracking for terrain following (if rangefinder enabled)
 *          7. Update AC_Circle navigation library with climb rate
 *          8. Update vertical position controller
 *          9. Send attitude targets to attitude controller with auto-yaw heading
 * 
 *          AC_Circle Library Integration:
 *          The AC_Circle library (copter.circle_nav) performs the core circular
 *          navigation calculations:
 *          - Calculates desired position on circle circumference
 *          - Generates velocity targets tangent to circle
 *          - Handles rate and radius changes smoothly
 *          - Provides bearing and distance to target for telemetry
 * 
 *          Pilot Control During Circle Flight:
 *          - Pitch Stick: Modifies radius in real-time at WP_NAV speed rate
 *                        (up/forward = decrease radius, down/back = increase radius)
 *          - Roll Stick: Modifies rotation rate from -90 to +90 deg/s
 *                       (right = increase clockwise rate, left = increase CCW rate)
 *          - Throttle Stick: Standard altitude control, independent of circle
 *          - Yaw Stick: Disabled (automatic yaw-to-center overrides pilot yaw)
 * 
 *          Radio Failsafe Behavior:
 *          If RC signal is lost, pilot control inputs are ignored but autonomous
 *          circle continues with last commanded radius and rate parameters.
 * 
 * @note Called at main scheduler rate (typically 400Hz for multicopters)
 * @note Minimum recommended call rate: 100Hz for stable circle control
 * @note Disarmed or landed state overrides all circle navigation
 * 
 * @warning Do not call at rates below 50Hz - may cause unstable flight
 * @warning Circle navigation requires valid position estimate (GPS or non-GPS nav)
 * @warning Large radius changes can momentarily exceed angle limits
 * 
 * @see AC_Circle::update_cms() for position target generation
 * @see AC_PosControl for position controller implementation
 * @see AutoYaw::Mode::CIRCLE for yaw control behavior
 * @see CIRCLE_RADIUS parameter (cm) for default radius
 * @see CIRCLE_RATE parameter (deg/s) for rotation rate
 * @see CIRCLE_OPTIONS parameter for pilot control enable/disable
 * 
 * Source: ArduCopter/mode_circle.cpp:47-127
 */
void ModeCircle::run()
{
    // Update position controller speed and acceleration limits each cycle
    // Horizontal limits (NE plane) control circle navigation responsiveness
    pos_control->set_max_speed_accel_NE_cm(wp_nav->get_default_speed_NE_cms(), wp_nav->get_wp_acceleration_cmss());
    // Vertical limits (U axis) allow pilot altitude control during circle flight
    pos_control->set_max_speed_accel_U_cm(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // Check for real-time changes to CIRCLE_RADIUS, CIRCLE_RATE, or CIRCLE_OPTIONS parameters
    // Allows parameter tuning during flight without mode exit/re-entry
    copter.circle_nav->check_param_change();

    // ============================================================================
    // PILOT CONTROL: Real-time adjustment of circle radius and rotation rate
    // ============================================================================
    // Only process pilot inputs if RC signal is valid and pilot control is enabled via CIRCLE_OPTIONS
    // In radio failsafe, circle continues autonomously with last commanded parameters
    if (rc().has_valid_input() && copter.circle_nav->pilot_control_enabled()) {
        
        // ------------------------------------------------------------------------
        // RADIUS CONTROL via Pitch Stick
        // ------------------------------------------------------------------------
        // Pilot can dynamically adjust circle radius using forward/back stick
        // Radius changes at rate proportional to WP_NAV speed parameter
        
        const float radius_current = copter.circle_nav->get_radius_cm();           // Current radius target in centimeters
        const float pitch_stick = channel_pitch->norm_input_dz();                 // Pitch stick input: -1.0 (back) to +1.0 (forward), deadzone applied
        const float nav_speed = copter.wp_nav->get_default_speed_NE_cms();        // Speed from WP_NAV parameter (cm/s)
        
        // Calculate radius change rate: forward stick (positive) decreases radius (moves toward center)
        // This matches intuitive "fly forward" behavior - pushing forward tightens the circle
        const float radius_pilot_change = (pitch_stick * nav_speed) * G_Dt;       // Change per cycle in cm (G_Dt = scheduler timestep)
        const float radius_new = MAX(radius_current + radius_pilot_change, 0);    // Constrain to non-negative radius

        // Apply radius change only if value actually changed (avoids unnecessary updates)
        if (!is_equal(radius_current, radius_new)) {
            copter.circle_nav->set_radius_cm(radius_new);
        }

        // ------------------------------------------------------------------------
        // ROTATION RATE CONTROL via Roll Stick
        // ------------------------------------------------------------------------
        // Pilot can dynamically adjust rotation rate (angular velocity around circle)
        // Skip this if transmitter tuning knob is assigned to circle rate (alternate control method)
        if (g.rc_tuning_param != TUNING_CIRCLE_RATE) {
            const float roll_stick = channel_roll->norm_input_dz();                // Roll stick input: -1.0 (left) to +1.0 (right), deadzone applied

            if (is_zero(roll_stick)) {
                // Pilot released roll stick - maintain current rate, reset state tracking flag
                speed_changing = false;
            } else {
                // Pilot is actively adjusting rotation rate with roll stick
                const float rate = copter.circle_nav->get_rate_degs();             // Configured rate from CIRCLE_RATE parameter (deg/s, + = CW, - = CCW)
                const float rate_current = copter.circle_nav->get_rate_current();  // Current smoothed rate target (may differ from parameter during transitions)
                const float rate_pilot_change = (roll_stick * G_Dt);               // Rate change per cycle: ±1.0 deg/s² maximum (gentle acceleration)
                float rate_new = rate_current;                                     // Initialize new rate target
                
                // Rate adjustment logic depends on current direction of travel:
                if (is_positive(rate)) {
                    // Currently orbiting clockwise (positive rate)
                    // Allow adjustment within clockwise range: 0 to +90 deg/s
                    // Right stick increases CW rate, left stick decreases toward zero (stop)
                    rate_new = constrain_float(rate_current + rate_pilot_change, 0, 90);

                } else if (is_negative(rate)) {
                    // Currently orbiting counter-clockwise (negative rate)
                    // Allow adjustment within CCW range: -90 to 0 deg/s
                    // Left stick increases CCW rate, right stick decreases toward zero (stop)
                    rate_new = constrain_float(rate_current + rate_pilot_change, -90, 0);

                } else if (is_zero(rate) && !speed_changing) {
                    // Special case: Vehicle stopped (zero rate) and pilot just started moving stick
                    // Use pilot input to determine initial direction (+ = CW, - = CCW)
                    // This allows starting rotation from stopped state
                    rate_new = rate_pilot_change;
                }

                // Mark that pilot is actively changing speed (affects zero-crossing logic)
                speed_changing = true;
                copter.circle_nav->set_rate_degs(rate_new);
            }
        }
    }

    // ============================================================================
    // ALTITUDE CONTROL: Pilot retains independent altitude control during circle
    // ============================================================================
    
    // Get pilot's desired climb rate from throttle stick input (cm/s)
    // Returns zero if in radio failsafe - maintains current altitude
    float target_climb_rate_cms = get_pilot_desired_climb_rate();

    // Apply obstacle avoidance adjustments to climb rate if avoidance system active
    // May reduce climb rate to avoid ceiling obstacles or increase descent to avoid ground
    target_climb_rate_cms = get_avoidance_adjusted_climbrate_cms(target_climb_rate_cms);

    // ============================================================================
    // SAFETY: Handle disarmed or landed state
    // ============================================================================
    // If motors are disarmed or vehicle has landed, perform safe ground handling
    // Zeros throttle output and exits immediately without running navigation
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // Enable full throttle range for circle flight (not landing or takeoff limited)
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

#if AP_RANGEFINDER_ENABLED
    // Update surface tracking offset if rangefinder available
    // Allows terrain-following circle flight maintaining constant AGL altitude
    copter.surface_tracking.update_surface_offset();
#endif

    // ============================================================================
    // CIRCLE NAVIGATION UPDATE: Core position target generation
    // ============================================================================
    // Call AC_Circle library to calculate desired position on circle circumference
    // Inputs: target_climb_rate_cms - desired vertical velocity
    // Returns: terrain failsafe status (true if terrain data valid, false if lost)
    // Side effects: Updates position controller with horizontal and vertical targets
    copter.failsafe_terrain_set_status(copter.circle_nav->update_cms(target_climb_rate_cms));
    
    // Update vertical (altitude) position controller based on climb rate target
    // Generates thrust component in Up/Down axis
    pos_control->update_U_controller();

    // ============================================================================
    // ATTITUDE CONTROL: Final output to motors
    // ============================================================================
    // Send combined thrust vector and heading command to attitude controller
    // - Thrust vector: 3D direction/magnitude from position controller (includes NE circle velocity + U climb)
    // - Heading: Auto-yaw angle facing circle center (from auto_yaw.get_heading())
    // Attitude controller converts to motor outputs considering vehicle dynamics
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

/**
 * @brief Get distance to circle target position
 * 
 * @details Returns the straight-line distance from current vehicle position to the
 *          target position on the circle circumference. This is used for telemetry
 *          reporting and ground station display. During normal circle flight, this
 *          distance should remain small (within a few meters) as the position
 *          controller tracks the moving target point.
 * 
 * @return Distance to target in meters
 * 
 * @note Target position moves continuously around circle circumference
 * @note Large distances indicate position controller is struggling to track circle
 * @note Used by MAVLink WP_DISTANCE field in telemetry
 * 
 * @see AC_Circle::get_distance_to_target_cm() for underlying calculation
 * 
 * Source: ArduCopter/mode_circle.cpp:129-132
 */
float ModeCircle::wp_distance_m() const
{
    return copter.circle_nav->get_distance_to_target_cm() * 0.01f;
}

/**
 * @brief Get bearing to circle target position
 * 
 * @details Returns the compass bearing from current vehicle position to the target
 *          position on the circle circumference. This bearing continuously changes
 *          as the vehicle orbits the circle. Used for telemetry reporting and
 *          navigation display on ground control station.
 * 
 * @return Bearing to target in degrees (0-360, 0=North, 90=East)
 * 
 * @note During circle flight, bearing rotates continuously as vehicle orbits
 * @note Bearing is approximately tangent to circle (direction of travel)
 * @note Used by MAVLink WP_BEARING field in telemetry
 * 
 * @see AC_Circle::get_bearing_to_target_rad() for underlying calculation
 * 
 * Source: ArduCopter/mode_circle.cpp:134-137
 */
float ModeCircle::wp_bearing_deg() const
{
    return degrees(copter.circle_nav->get_bearing_to_target_rad());
}

#endif
