/**
 * @file baro_ground_effect.cpp
 * @brief Barometer ground effect compensation and EKF terrain height management
 * 
 * @details This file implements ground effect detection and compensation for
 *          barometric altitude measurements. During hover near the ground, rotor
 *          downwash creates a high-pressure region beneath the vehicle that affects
 *          barometric pressure readings, causing altitude estimation errors. This
 *          system detects takeoff and landing phases and signals the EKF to apply
 *          appropriate compensation to maintain accurate altitude hold in ground effect.
 *          
 *          Ground Effect Physics:
 *          - Rotor downwash creates increased static pressure beneath vehicle
 *          - Barometric altimeters interpret higher pressure as lower altitude
 *          - Effect most pronounced within 1-2 rotor diameters of ground
 *          - Can cause 0.5-2m altitude errors during low hover
 *          
 *          Detection Strategy:
 *          - Monitors vehicle state for imminent takeoff or landing
 *          - Uses altitude, velocity, attitude, and throttle signals
 *          - Signals EKF to adjust barometer weighting and bias estimation
 *          
 * @note This is safety-critical code affecting altitude hold accuracy near ground
 * 
 * Source: ArduCopter/baro_ground_effect.cpp
 */
#include "Copter.h"

/**
 * @brief Detect ground effect conditions and signal EKF for barometer compensation
 * 
 * @details This function analyzes vehicle state to determine if the multicopter is in
 *          or approaching ground effect conditions during takeoff or landing. Ground
 *          effect occurs when rotor downwash reflects off the ground surface, creating
 *          a cushion of high-pressure air beneath the vehicle. This pressure increase
 *          causes barometric altimeters to report artificially low altitude readings
 *          (typically 0.5-2 meters error depending on vehicle size and hover height).
 *          
 *          Algorithm Overview:
 *          1. Detect imminent takeoff: Armed, landed, throttle commanded
 *          2. Detect active takeoff: Time since throttle up, altitude gained
 *          3. Detect imminent landing: Slow horizontal speed, controlled descent
 *          4. Signal EKF to apply appropriate barometer bias compensation
 *          
 *          Takeoff Detection Logic:
 *          - Monitors armed state with land_complete flag (on ground)
 *          - Tracks time since throttle application (5 second window)
 *          - Confirms takeoff when altitude increases >50cm from starting point
 *          - Excludes THROW mode which has unique ground release behavior
 *          
 *          Landing Detection Logic:
 *          - Requires slow horizontal motion (<1.25 m/s actual or <1.25 m/s commanded)
 *          - Requires slow descent (<1.0 m/s demanded or <0.6 m/s actual)
 *          - Validates small attitude angles (<7.5° from level in ALT_HOLD)
 *          - Combines velocity measurements with pilot/autopilot intent
 *          
 *          EKF Compensation Strategy:
 *          - set_takeoff_expected(): Reduces barometer weight during initial climb
 *          - set_touchdown_expected(): Prepares for pressure anomalies during descent
 *          - Allows EKF to estimate and remove ground effect barometer bias
 *          - Prevents altitude control oscillations and overshoot during landing
 *          
 *          State Machine Behavior:
 *          - takeoff_expected: true from armed+landed until 5s or 50cm altitude
 *          - touchdown_expected: true when slow+descending, false otherwise
 *          - Both states reset when disarmed or compensation disabled
 * 
 * @note Called at main loop rate (typically 400Hz for attitude control)
 * @note Requires g2.gndeffect_comp_enabled parameter to be active
 * @note Directly affects altitude hold performance within 1-2m of ground
 * 
 * @warning Ground effect compensation is safety-critical for:
 *          - Preventing hard landings due to altitude underestimation
 *          - Avoiding takeoff altitude oscillations
 *          - Maintaining stable hover at low altitude
 *          Improper detection can cause altitude control instability
 * 
 * @see Copter::update_ekf_terrain_height_stable() for related terrain height stabilization
 * @see AP_AHRS::set_takeoff_expected() for EKF takeoff compensation
 * @see AP_AHRS::set_touchdown_expected() for EKF landing compensation
 */
void Copter::update_ground_effect_detector(void)
{
    // Ground effect compensation can be disabled via parameter or when disarmed
    // When disabled, clear all ground effect states to prevent EKF from applying
    // inappropriate barometer bias compensation during subsequent flights
    if(!g2.gndeffect_comp_enabled || !motors->armed()) {
        // Clear takeoff/touchdown flags - no ground effect compensation needed
        // This ensures clean state when:
        // - Feature disabled by g2.gndeffect_comp_enabled parameter
        // - Vehicle disarmed (no thrust, no ground effect)
        // - Between flights (prevents state carryover)
        gndeffect_state.takeoff_expected = false;
        gndeffect_state.touchdown_expected = false;
        
        // Inform EKF to use normal barometer weighting without ground effect bias estimation
        ahrs.set_takeoff_expected(gndeffect_state.takeoff_expected);
        ahrs.set_touchdown_expected(gndeffect_state.touchdown_expected);
        return;
    }

    // ========================================================================
    // Variable Initialization: Gather current vehicle state for detection
    // ========================================================================
    
    // Timestamp for takeoff timing logic (5 second takeoff window)
    uint32_t tnow_ms = millis();
    
    // Horizontal velocity demand from position controller (cm/s)
    // Used to detect slow horizontal motion during landing
    float xy_des_speed_cms = 0.0f;
    
    // Vertical velocity demand (cm/s, positive = climb, negative = descend)
    // Key signal for detecting controlled descent during landing approach
    float des_climb_rate_cms = pos_control->get_vel_desired_NEU_cms().z;

    // Extract horizontal speed demand if position controller is active
    // North-East position control active in modes like LOITER, AUTO, GUIDED
    if (pos_control->is_active_NE()) {
        // Get full 3D velocity target in NED (North-East-Down) frame
        Vector3f vel_target = pos_control->get_vel_target_NEU_cms();
        vel_target.z = 0.0f;  // Zero vertical component to get horizontal speed only
        xy_des_speed_cms = vel_target.length();  // Magnitude of horizontal velocity vector
    }

    // ========================================================================
    // Takeoff Detection Logic: Identify when ground effect compensation needed during takeoff
    // ========================================================================
    // 
    // Ground Effect During Takeoff:
    // As rotors spin up, downwash velocity increases and pressure builds beneath vehicle.
    // Barometer interprets increased pressure as decreased altitude, causing:
    // - Initial altitude reads artificially LOW (vehicle appears lower than actual)
    // - Altitude controller commands MORE thrust to reach target altitude
    // - Vehicle climbs rapidly once clear of ground effect
    // - Can cause overshoot and oscillation in first few meters of climb
    // 
    // Compensation Strategy:
    // Signal EKF that takeoff is expected so it can:
    // - Temporarily reduce barometer weight in fusion
    // - Estimate ground effect pressure bias
    // - Transition smoothly to normal barometer weighting as altitude increases

    if (flightmode->mode_number() == Mode::Number::THROW) {
        // THROW mode has unique ground release behavior - vehicle is hand-launched
        // Ground effect not applicable as vehicle doesn't take off from stationary ground position
        // Disable takeoff_expected to prevent inappropriate barometer compensation
        gndeffect_state.takeoff_expected = false;
    } else if (motors->armed() && ap.land_complete) {
        // Vehicle is armed and sitting on ground - takeoff imminent
        // land_complete flag confirms vehicle is detected as landed (weight on ground)
        // Enable ground effect compensation before throttle increase to prepare EKF
        gndeffect_state.takeoff_expected = true;
    }

    // Get current altitude estimate relative to origin point (meters, Down is positive)
    // This altitude is used to detect when vehicle has climbed clear of ground effect
    float pos_d_m = 0;
    UNUSED_RESULT(AP::ahrs().get_relative_position_D_origin_float(pos_d_m));

    // Monitor throttle state to capture takeoff initiation time and starting altitude
    const bool throttle_up = flightmode->has_manual_throttle() && channel_throttle->get_control_in() > 0;
    if (!throttle_up && ap.land_complete) {
        // Still on ground with no throttle - update baseline continuously
        // Captures the moment before takeoff begins for accurate altitude gain measurement
        gndeffect_state.takeoff_time_ms = tnow_ms;
        // Convert altitude to centimeters and negate (Down positive to Up positive)
        gndeffect_state.takeoff_alt_cm = -pos_d_m * 100.0;
    }

    // Determine when vehicle has climbed sufficiently to exit ground effect zone
    // Ground effect diminishes significantly beyond 1-2 rotor diameters (~50cm for small copters)
    if (gndeffect_state.takeoff_expected && (tnow_ms-gndeffect_state.takeoff_time_ms > 5000 || (-pos_d_m * 100.0 - gndeffect_state.takeoff_alt_cm) > 50.0f)) {
        // Exit takeoff_expected state when either:
        // - 5 seconds elapsed since throttle up (timeout for delayed takeoff)
        // - Vehicle climbed >50cm above takeoff point (clear of ground effect)
        gndeffect_state.takeoff_expected = false;
    }

    // ========================================================================
    // Landing Detection Logic: Identify when ground effect compensation needed during landing
    // ========================================================================
    //
    // Ground Effect During Landing:
    // As vehicle descends toward ground, rotor downwash reflects off surface creating
    // pressure cushion that increases with proximity. Effects on barometric altitude:
    // - Barometer reads artificially LOW altitude (appears closer to ground than actual)
    // - Altitude controller sees negative altitude error and REDUCES thrust
    // - Reduced thrust causes faster descent than commanded
    // - Can result in hard landing or altitude oscillation in final meters
    //
    // Compensation Strategy:
    // Detect controlled landing approach (slow horizontal + slow descent) and signal EKF
    // to adjust barometer bias estimation, preventing altitude control instability
    //
    // Detection requires BOTH slow horizontal motion AND slow descent to avoid false
    // triggers during normal flight or aggressive descents away from landing areas

    // ---- Horizontal Motion Detection ----
    // Check attitude angles to detect level flight (important for ALT_HOLD mode)
    Vector3f angle_target_rad = attitude_control->get_att_target_euler_rad();
    // Calculate if vehicle is nearly level: cos(roll)*cos(pitch) > cos(7.5°)
    // Small angle approximation: vehicle within 7.5° of horizontal in both axes
    bool small_angle_request = cosf(angle_target_rad.x)*cosf(angle_target_rad.y) > cosf(radians(7.5f));
    
    // Get actual vehicle velocity in NED frame (m/s)
    Vector3f vel_ned_ms;
    // Check measured horizontal speed: <1.25 m/s indicates slow horizontal motion
    bool xy_speed_low = AP::ahrs().get_velocity_NED(vel_ned_ms) && (vel_ned_ms.xy().length() < 1.25);
    
    // Check commanded horizontal speed: <125 cm/s (1.25 m/s) when position control active
    bool xy_speed_demand_low = pos_control->is_active_NE() && xy_des_speed_cms <= 125.0f;
    
    // Horizontal motion considered "slow" (landing-like) if ANY of:
    // 1. Slow speed commanded by position controller (AUTO, GUIDED, LOITER landing)
    // 2. Measured speed is low AND position control not active (manual landing drift)
    // 3. ALT_HOLD mode with level attitude request (pilot centering sticks for landing)
    bool slow_horizontal = xy_speed_demand_low || (xy_speed_low && !pos_control->is_active_NE()) || (flightmode->mode_number() == Mode::Number::ALT_HOLD && small_angle_request);

    // ---- Vertical Motion Detection ----
    // Check if altitude controller is commanding descent
    bool descent_demanded = pos_control->is_active_U() && des_climb_rate_cms < 0.0f;
    
    // Slow descent demand: -100 cm/s to 0 cm/s (-1.0 m/s to 0 m/s)
    // Typical landing descent rates are 30-50 cm/s, well within this threshold
    bool slow_descent_demanded = descent_demanded && des_climb_rate_cms >= -100.0f;
    
    // Get actual vertical velocity (m/s, Down positive in NED frame)
    // vibration_check.high_vibes flag indicates if velocity estimate is reliable
    bool z_speed_low = AP::ahrs().get_velocity_D(vel_ned_ms.z, vibration_check.high_vibes) && fabsf(vel_ned_ms.z) <= 0.6f;
    
    // Vertical motion considered "slow descent" (landing-like) if EITHER:
    // 1. Slow descent commanded by altitude controller (controlled landing)
    // 2. Measured descent rate is low AND some descent is demanded (final touchdown phase)
    bool slow_descent = (slow_descent_demanded || (z_speed_low && descent_demanded));

    // ---- Final Landing Detection ----
    // Touchdown expected when vehicle exhibits both slow horizontal and slow vertical motion
    // This combination indicates controlled landing approach where ground effect matters
    // False triggers minimized by requiring both conditions simultaneously
    gndeffect_state.touchdown_expected = slow_horizontal && slow_descent;

    // ========================================================================
    // Signal EKF for Ground Effect Compensation
    // ========================================================================
    //
    // Inform the Extended Kalman Filter (EKF) about expected ground effect conditions
    // so it can apply appropriate barometer bias estimation and sensor weighting.
    //
    // EKF Compensation Mechanisms:
    // - When takeoff_expected: Reduces barometer innovation gate, estimates initial bias
    // - When touchdown_expected: Prepares for barometer anomalies, maintains altitude control
    // - Smooth transition between normal and ground-effect tuning prevents discontinuities
    //
    // This signaling is critical for:
    // - Preventing altitude overshoots after takeoff
    // - Avoiding hard landings due to altitude underestimation
    // - Maintaining stable hover within 1-2 meters of ground
    //
    // The EKF will autonomously manage barometer weighting based on these signals
    // combined with rangefinder data (if available) and GPS altitude observations
    ahrs.set_takeoff_expected(gndeffect_state.takeoff_expected);
    ahrs.set_touchdown_expected(gndeffect_state.touchdown_expected);
}

/**
 * @brief Update EKF terrain height stability flag for rangefinder-assisted altitude estimation
 * 
 * @details This function controls whether the EKF considers terrain height stable enough
 *          to use rangefinder measurements for improved altitude estimation during takeoff
 *          and landing. When enabled, the EKF can blend rangefinder data with barometric
 *          altitude to provide more accurate height-above-ground measurements that are
 *          less affected by barometric pressure variations and ground effect.
 *          
 *          Terrain Height Stable Concept:
 *          The "terrain height stable" flag tells the EKF that the ground surface beneath
 *          the vehicle is at a consistent, unchanging altitude. This assumption allows
 *          the EKF to use rangefinder (e.g., lidar, sonar) measurements to:
 *          - Stabilize barometric altitude estimates during pressure transients
 *          - Improve altitude accuracy when barometer is affected by ground effect
 *          - Provide better height control during takeoff and landing phases
 *          
 *          When Terrain is "Stable":
 *          - Vehicle taking off from flat surface (terrain not changing)
 *          - Vehicle landing on flat surface (terrain not changing)
 *          - Rangefinder can measure consistent height-above-ground
 *          
 *          When Terrain is NOT "Stable":
 *          - Normal flight over varying terrain
 *          - Position estimate unavailable (can't relate rangefinder to absolute altitude)
 *          - Terrain following where ground height is changing
 *          
 *          Implementation Note:
 *          This is NOT related to terrain following (TERRAIN_FOLLOW) which uses terrain
 *          database. This is purely about using rangefinder to stabilize EKF altitude
 *          estimation during phases when terrain height is known to be constant.
 * 
 * @note Called at main loop rate to update EKF configuration
 * @note Requires valid position estimate from EKF or GPS
 * @note Complements ground effect compensation by improving altitude source accuracy
 * 
 * @see Copter::update_ground_effect_detector() for ground effect compensation
 * @see AP_AHRS::set_terrain_hgt_stable() for EKF terrain height configuration
 * @see Mode::is_taking_off() for takeoff phase detection
 * @see Mode::is_landing() for landing phase detection
 */
void Copter::update_ekf_terrain_height_stable()
{
    // Terrain height stability requires valid position estimate to relate
    // rangefinder measurements to absolute altitude in the EKF
    // Without position lock, we cannot determine if rangefinder changes
    // are due to terrain variation or vehicle movement
    if (!position_ok() && !ekf_has_relative_position()) {
        // No valid position estimate available - disable terrain height stable
        // EKF will rely solely on barometer and GPS altitude without rangefinder stabilization
        ahrs.set_terrain_hgt_stable(false);
        return;
    }

    // Enable terrain height stable during takeoff and landing phases only
    // These are the phases where:
    // 1. Terrain height is known to be constant (flat takeoff/landing surface)
    // 2. Altitude accuracy is most critical (ground proximity)
    // 3. Barometer may be affected by ground effect (downwash pressure changes)
    // 4. Rangefinder provides accurate height-above-ground reference
    //
    // During normal flight, terrain height is NOT stable (flying over varying terrain)
    // so this flag is disabled to prevent inappropriate rangefinder fusion
    ahrs.set_terrain_hgt_stable(flightmode->is_taking_off() || flightmode->is_landing());
}
