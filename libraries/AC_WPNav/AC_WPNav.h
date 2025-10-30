/**
 * @file AC_WPNav.h
 * @brief Waypoint navigation library for multirotor aircraft
 * 
 * @details This file implements sophisticated waypoint navigation for multicopter vehicles
 *          using advanced trajectory generation techniques. The library provides smooth,
 *          efficient paths between waypoints with support for both straight-line segments
 *          and curved spline paths.
 * 
 * Key features:
 * - S-curve trajectory generation with jerk and snap limiting for smooth acceleration profiles
 * - Spline waypoint support using Hermite spline interpolation for curved paths
 * - Multi-leg trajectory blending for seamless corner cutting and path optimization
 * - Terrain following via rangefinder and AP_Terrain database integration
 * - Dynamic speed and acceleration limiting based on vehicle attitude constraints
 * - Integration with AC_PosControl for precise position/velocity/acceleration control
 * 
 * Architecture:
 * - Uses SCurve engine for jerk-limited straight-line trajectories
 * - Uses SplineCurve engine for smooth curved paths between waypoints
 * - Blends previous, current, and next leg trajectories for corner optimization
 * - Supports both EKF-origin-relative and terrain-relative altitude frames
 * 
 * Coordinate Frames:
 * - NEU (North-East-Up): Position vectors in cm from EKF origin
 * - NED (North-East-Down): Position vectors in meters from EKF origin
 * - Location: Global position with AMSL or terrain-relative altitude
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 * 
 * Source: libraries/AC_WPNav/AC_WPNav.h
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/SCurve.h>
#include <AP_Math/SplineCurve.h>
#include <AP_Common/Location.h>
#include <AC_AttitudeControl/AC_PosControl.h>      // Position control library
#include <AC_AttitudeControl/AC_AttitudeControl.h> // Attitude control library
#include <AP_Terrain/AP_Terrain.h>
#include <AC_Avoidance/AC_Avoid.h>                 // Stop at fence library

// maximum velocities and accelerations
#define WPNAV_ACCELERATION              250.0f      // maximum horizontal acceleration in cm/s/s that wp navigation will request

/**
 * @class AC_WPNav
 * @brief Waypoint navigation controller for multicopter aircraft
 * 
 * @details AC_WPNav provides sophisticated waypoint-to-waypoint navigation for multirotor
 *          vehicles using advanced trajectory generation with S-curve acceleration profiles
 *          and optional spline paths. The controller generates smooth, efficient trajectories
 *          that respect vehicle dynamics constraints while minimizing flight time.
 * 
 * Core Trajectory Generation:
 * - **SCurve Engine**: Generates jerk-limited and snap-limited acceleration profiles for
 *   smooth straight-line trajectories. Constraints (speed, acceleration, jerk, snap) are
 *   derived from vehicle attitude limits and GRAVITY_MSS to ensure achievable trajectories.
 * 
 * - **SplineCurve Engine**: Implements Hermite spline interpolation for curved paths between
 *   waypoints. Requires lookahead to next destination for smooth curve generation.
 * 
 * - **Multi-Leg Blending**: Maintains previous, current, and next leg trajectories to enable
 *   smooth corner cutting. Blends trajectories at waypoint transitions to eliminate abrupt
 *   direction changes and minimize total flight time.
 * 
 * Terrain Following:
 * - Supports altitude control relative to terrain using either rangefinder data or
 *   AP_Terrain database (SRTM elevation data)
 * - Terrain offset automatically compensates target altitude to maintain desired
 *   height above ground
 * - Safety margin parameter prevents flying too close to terrain
 * - Terrain following requires either rangefinder availability or terrain database coverage
 * 
 * Speed and Acceleration Control:
 * - Horizontal speed (WPNAV_SPEED): Default lateral velocity in cm/s
 * - Climb rate (WPNAV_SPEED_UP): Maximum ascent rate in cm/s
 * - Descent rate (WPNAV_SPEED_DN): Maximum descent rate in cm/s
 * - Horizontal acceleration (WPNAV_ACCEL): Linear acceleration in cm/s²
 * - Corner acceleration (WPNAV_ACCEL_C): Acceleration during cornering in cm/s²
 * - Vertical acceleration (WPNAV_ACCEL_Z): Climb/descent acceleration in cm/s²
 * - Jerk (WPNAV_JERK): Rate of acceleration change in m/s³
 * 
 * Typical Usage Pattern:
 * 1. Initialize: wp_and_spline_init_cm() - sets up controller and initial speeds
 * 2. Set destination: set_wp_destination_loc() or set_wp_destination_NEU_cm()
 * 3. Update loop: Call update_wpnav() at 100Hz or higher (matches main loop rate)
 * 4. Check progress: get_wp_distance_to_destination_cm(), reached_wp_destination()
 * 5. Retrieve outputs: get_roll_rad(), get_pitch_rad() for attitude targets
 * 
 * Integration with AC_PosControl:
 * - AC_WPNav generates position/velocity/acceleration targets along the trajectory
 * - AC_PosControl converts these targets to attitude (roll/pitch) and thrust commands
 * - Position controller runs at the same rate as waypoint controller (typically 100Hz)
 * 
 * Object Avoidance:
 * - Virtual methods can be overridden by AC_WPNav_OA for Dijkstra's path planning
 * - force_stop_at_next_wp() allows external avoidance system to halt at waypoints
 * - get_oa_wp_destination() provides avoidance-adjusted destination (in OA subclass)
 * 
 * Coordinate Frame Details:
 * - NEU (North-East-Up) vectors in cm: Primary internal representation
 * - NED (North-East-Down) vectors in meters: Alternative interface for some methods
 * - Location objects: Global position with altitude frame (AMSL or terrain-relative)
 * - origin_and_destination_are_terrain_alt() indicates if z-axis is terrain-relative
 * 
 * @note This controller is designed for multicopter vehicles. Fixed-wing aircraft use
 *       different navigation controllers (AP_L1_Control, AP_TECS) due to fundamentally
 *       different flight dynamics and constraints.
 * 
 * @warning Modifying speed, acceleration, jerk, or snap limits can affect vehicle stability
 *          and waypoint tracking performance. Conservative values ensure smooth flight but
 *          may increase mission time. Aggressive values reduce mission time but require
 *          careful tuning to avoid oscillations or overshoot.
 * 
 * @warning Terrain following requires either rangefinder data or AP_Terrain database.
 *          Mission planning must ensure terrain data availability along flight path.
 *          Loss of terrain data during terrain-following flight triggers failsafe behavior.
 * 
 * @see AC_PosControl for position control integration
 * @see SCurve for S-curve trajectory generation details
 * @see SplineCurve for spline interpolation mathematics
 * @see AP_Terrain for terrain database information
 */
class AC_WPNav
{
public:

    /**
     * @brief Constructor for waypoint navigation controller
     * 
     * @details Initializes the waypoint navigation controller with references to required
     *          subsystems. Loads parameters from persistent storage and sets up internal
     *          state for trajectory generation.
     * 
     * @param[in] ahrs Reference to AP_AHRS_View for vehicle attitude and position information
     * @param[in] pos_control Reference to AC_PosControl for position controller integration
     * @param[in] attitude_control Reference to AC_AttitudeControl for vehicle attitude constraints
     * 
     * @note This constructor does not initialize the navigation state. Call wp_and_spline_init_cm()
     *       before beginning waypoint navigation.
     */
    AC_WPNav(const AP_AHRS_View& ahrs, AC_PosControl& pos_control, const AC_AttitudeControl& attitude_control);

    /**
     * @brief Provide rangefinder-based terrain offset for terrain following
     * 
     * @details Updates the terrain offset used for altitude compensation during terrain following.
     *          The terrain offset represents the terrain's height above the EKF origin, allowing
     *          the controller to maintain a constant height above ground level.
     * 
     *          This method is typically called from the main vehicle code whenever new rangefinder
     *          data is available. The terrain offset is calculated as:
     *          terrain_offset = current_altitude - rangefinder_distance
     * 
     * @param[in] use True if rangefinder is enabled for terrain following (user setting)
     * @param[in] healthy True if rangefinder distance reading is valid (within min/max range)
     * @param[in] terrain_offset_cm Terrain height above EKF origin in cm (+ve = terrain above origin)
     * 
     * @note Rangefinder must be healthy and enabled for terrain following to be active
     * @note This method is called at the rate new rangefinder samples arrive (typically 10-50Hz)
     * 
     * @see get_terrain_source() to determine active terrain data source
     * @see rangefinder_used() to check if rangefinder is being used
     */
    void set_rangefinder_terrain_offset_cm(bool use, bool healthy, float terrain_offset_cm) { _rangefinder_available = use; _rangefinder_healthy = healthy; _rangefinder_terrain_offset_cm = terrain_offset_cm;}

    /**
     * @brief Check if rangefinder may be used for terrain following
     * 
     * @return true if rangefinder terrain following is enabled by parameter, false otherwise
     * 
     * @note This reflects the user parameter setting (WPNAV_RFND_USE), not current health status
     * @see rangefinder_used_and_healthy() to check both enabled and health status
     */
    bool rangefinder_used() const { return _rangefinder_use; }
    
    /**
     * @brief Check if rangefinder is both enabled and currently healthy
     * 
     * @return true if rangefinder is enabled and providing valid distance readings, false otherwise
     * 
     * @note Healthy means distance reading is within sensor min/max range
     */
    bool rangefinder_used_and_healthy() const { return _rangefinder_use && _rangefinder_healthy; }

    /**
     * @enum TerrainSource
     * @brief Identifies the source of terrain altitude data for terrain following
     * 
     * @details Terrain following requires altitude-above-ground information from either
     *          a downward-facing rangefinder or a terrain elevation database (SRTM data).
     *          This enum indicates which source is currently available and will be used.
     */
    enum class TerrainSource {
        TERRAIN_UNAVAILABLE,        ///< No terrain data available (terrain following not possible)
        TERRAIN_FROM_RANGEFINDER,   ///< Using rangefinder for terrain altitude
        TERRAIN_FROM_TERRAINDATABASE, ///< Using AP_Terrain database (SRTM elevation data)
    };
    
    /**
     * @brief Get expected source of terrain data for altitude-above-terrain commands
     * 
     * @details Determines which terrain data source will be used if an altitude-above-terrain
     *          waypoint command is executed. Prioritizes rangefinder over terrain database.
     *          Used by flight modes (e.g., Copter's ModeRTL) to verify terrain following capability.
     * 
     * @return TerrainSource indicating data source or TERRAIN_UNAVAILABLE if none available
     * 
     * @note Rangefinder is preferred over terrain database when both are available
     * @note This method checks current availability, not parameter settings
     */
    AC_WPNav::TerrainSource get_terrain_source() const;

    /**
     * @brief Get terrain altitude at current vehicle position
     * 
     * @details Returns the terrain's altitude in cm above the EKF origin. This value is used
     *          to offset target altitudes when performing terrain-following navigation.
     *          Positive values indicate terrain above the EKF origin altitude.
     * 
     * @param[out] offset_cm Terrain altitude above EKF origin in cm (+ve = terrain above origin)
     * 
     * @return true if terrain offset successfully retrieved, false if terrain data unavailable
     * 
     * @note Uses rangefinder data if available, otherwise falls back to terrain database
     * @note Terrain database requires GPS position and valid SRTM data for current location
     */
    bool get_terrain_offset_cm(float& offset_cm);

    /**
     * @brief Get terrain following altitude margin
     * 
     * @details Returns the safety margin for terrain following in meters. The vehicle will stop
     *          vertical movement if the distance from the target altitude becomes larger than
     *          this margin. This prevents excessive altitude deviations during terrain following.
     * 
     * @return Terrain following margin in meters (minimum 0.1m enforced)
     * 
     * @note Controlled by WPNAV_TER_MARGIN parameter
     * @note Larger margins provide more safety but may limit terrain following responsiveness
     */
    float get_terrain_margin_m() const { return MAX(_terrain_margin_m, 0.1); }

    /**
     * @brief Convert Location to position vector from EKF origin
     * 
     * @details Converts a global Location (latitude, longitude, altitude) to a position vector
     *          in the NEU (North-East-Up) frame relative to the EKF origin. Handles both
     *          absolute altitude (AMSL) and terrain-relative altitude frames.
     * 
     * @param[in]  loc Location to convert (global coordinates)
     * @param[out] pos_from_origin_NEU_cm Position vector from EKF origin in NEU frame (cm)
     * @param[out] is_terrain_alt Set to true if z-axis is altitude-above-terrain, false if above EKF origin
     * 
     * @return true if conversion successful, false if terrain data unavailable (for terrain-relative locations)
     * 
     * @note Coordinate frame: NEU (North-East-Up) with origin at EKF origin
     * @note Units: cm for all axes
     * @note Terrain-relative locations require terrain data availability
     * @note Failure typically indicates missing terrain database coverage or rangefinder data
     */
    bool get_vector_NEU_cm(const Location &loc, Vector3f &pos_from_origin_NEU_cm, bool &is_terrain_alt);

    ///
    /// waypoint controller
    ///

    /**
     * @brief Initialize straight-line and spline waypoint controllers
     * 
     * @details Initializes the waypoint navigation system for a new navigation sequence.
     *          Sets up trajectory generation parameters, resets internal state, and captures
     *          current vehicle attitude to initialize position controller I-terms for smooth
     *          transition into waypoint navigation.
     * 
     *          This method must be called once before beginning waypoint navigation but does
     *          not need to be called before subsequent destination updates during an active
     *          navigation sequence.
     * 
     * @param[in] speed_cms Desired maximum horizontal speed in cm/s (0 = use default from WPNAV_SPEED parameter)
     * @param[in] stopping_point Optional stopping point in NEU frame (cm from EKF origin)
     * 
     * @note Call this method when transitioning into AUTO, GUIDED, or RTL modes
     * @note Updates position controller target roll/pitch and I-terms based on current vehicle lean angles
     * @note Sets up SCurve and SplineCurve trajectory generators with current parameters
     * 
     * @see set_wp_destination_NEU_cm() to set first waypoint after initialization
     */
    void wp_and_spline_init_cm(float speed_cms = 0.0f, Vector3f stopping_point = Vector3f{});

    /**
     * @brief Set target horizontal speed during waypoint navigation
     * 
     * @details Dynamically adjusts the horizontal speed during active waypoint navigation.
     *          The trajectory is recalculated to smoothly transition to the new speed
     *          using jerk-limited acceleration profiles.
     * 
     * @param[in] speed_cms Target horizontal speed in cm/s (must be positive)
     * 
     * @note Speed change is applied smoothly using S-curve acceleration
     * @note New speed applies immediately to current trajectory segment
     * @note Setting speed to 0 pauses forward motion but maintains position hold
     * 
     * @see set_pause() for explicit pause control
     */
    void set_speed_NE_cms(float speed_cms);

    /**
     * @brief Pause waypoint navigation
     * 
     * @details Pauses forward progress along the trajectory while maintaining position hold.
     *          The vehicle will decelerate smoothly to a stop using jerk-limited braking
     *          and hold the current position until resumed.
     * 
     * @note Vehicle maintains altitude and position hold during pause
     * @note Trajectory progress can be resumed with set_resume()
     * 
     * @see set_resume() to resume navigation
     * @see paused() to check pause status
     */
    void set_pause() { _paused = true; }
    
    /**
     * @brief Resume waypoint navigation after pause
     * 
     * @details Resumes forward progress along the trajectory from the current position.
     *          The vehicle will accelerate smoothly from the stopped state using
     *          jerk-limited acceleration profiles.
     * 
     * @see set_pause() to pause navigation
     */
    void set_resume() { _paused = false; }

    /**
     * @brief Get current pause status
     * 
     * @return true if navigation is paused, false if active
     * 
     * @see set_pause(), set_resume()
     */
    bool paused() { return _paused; }

    /**
     * @brief Set target climb rate during waypoint navigation
     * 
     * @details Dynamically adjusts the vertical climb rate during active navigation.
     *          The climb rate is applied to altitude changes in the trajectory.
     * 
     * @param[in] speed_up_cms Target climb rate in cm/s (must be positive)
     * 
     * @note Climb rate is limited by WPNAV_ACCEL_Z parameter
     * @note Does not affect descent rate (see set_speed_down_cms)
     */
    void set_speed_up_cms(float speed_up_cms);
    
    /**
     * @brief Set target descent rate during waypoint navigation
     * 
     * @details Dynamically adjusts the vertical descent rate during active navigation.
     *          The descent rate is applied to altitude decreases in the trajectory.
     * 
     * @param[in] speed_down_cms Target descent rate in cm/s (must be positive)
     * 
     * @note Descent rate is limited by WPNAV_ACCEL_Z parameter
     * @note Does not affect climb rate (see set_speed_up_cms)
     */
    void set_speed_down_cms(float speed_down_cms);

    /**
     * @brief Get default horizontal velocity for waypoint navigation
     * 
     * @return Default horizontal speed in cm/s from WPNAV_SPEED parameter
     * 
     * @note This is the parameter value, not the current target speed
     * @see set_speed_NE_cms() to change current speed
     */
    float get_default_speed_NE_cms() const { return _wp_speed_cms; }

    /**
     * @brief Get default climb rate for missions
     * 
     * @return Default climb speed in cm/s from WPNAV_SPEED_UP parameter
     * 
     * @note This is the parameter value, not the current target climb rate
     */
    float get_default_speed_up_cms() const { return _wp_speed_up_cms; }

    /**
     * @brief Get default descent rate for missions
     * 
     * @return Default descent rate in cm/s from WPNAV_SPEED_DN parameter (always positive)
     * 
     * @note Return value is absolute (positive) regardless of parameter sign
     * @note This is the parameter value, not the current target descent rate
     */
    float get_default_speed_down_cms() const { return fabsf(_wp_speed_down_cms); }

    /**
     * @brief Get vertical acceleration for missions
     * 
     * @return Vertical acceleration in cm/s² from WPNAV_ACCEL_Z parameter (always positive)
     * 
     * @note Used for both climb and descent acceleration limiting
     * @note This limits how quickly vertical speed can change
     */
    float get_accel_U_cmss() const { return _wp_accel_z_cmss; }

    /**
     * @brief Get horizontal acceleration for waypoint navigation
     * 
     * @return Horizontal acceleration in cm/s² (WPNAV_ACCEL parameter or 250 cm/s² default)
     * 
     * @note Used for straight-line trajectory segments
     * @note Falls back to WPNAV_ACCELERATION (250) if parameter is not positive
     * @note This acceleration is used in S-curve trajectory generation
     */
    float get_wp_acceleration_cmss() const { return (is_positive(_wp_accel_cmss)) ? _wp_accel_cmss : WPNAV_ACCELERATION; }

    /**
     * @brief Get maximum acceleration during cornering
     * 
     * @return Corner acceleration in cm/s² (WPNAV_ACCEL_C parameter or 2× straight-line acceleration)
     * 
     * @note Corner acceleration can be higher than straight-line to minimize cornering time
     * @note Falls back to 2× straight-line acceleration if parameter is not positive
     * @note Used during waypoint transitions and multi-leg trajectory blending
     * 
     * @warning Higher corner acceleration reduces turn radius but increases vehicle tilt angle
     */
    float get_corner_acceleration_cmss() const { return (is_positive(_wp_accel_c_cmss)) ? _wp_accel_c_cmss : 2.0 * get_wp_acceleration_cmss(); }

    /**
     * @brief Get waypoint destination as position vector
     * 
     * @details Returns the current waypoint destination in NEU (North-East-Up) coordinates
     *          relative to the EKF origin. The z-axis may represent altitude above EKF origin
     *          or altitude above terrain depending on the mission command type.
     * 
     * @return Position vector from EKF origin in cm (NEU frame)
     * 
     * @note Coordinate frame: NEU (North-East-Up) with origin at EKF origin
     * @note Units: cm for all axes
     * @note Check origin_and_destination_are_terrain_alt() to determine z-axis reference
     * 
     * @see origin_and_destination_are_terrain_alt() to check z-axis frame
     * @see get_wp_origin_NEU_cm() for trajectory starting point
     */
    const Vector3f &get_wp_destination_NEU_cm() const { return _destination_neu_cm; }

    /**
     * @brief Get waypoint origin as position vector
     * 
     * @details Returns the trajectory starting point in NEU (North-East-Up) coordinates
     *          relative to the EKF origin. This is typically the vehicle position when
     *          the destination was set.
     * 
     * @return Origin position vector from EKF origin in cm (NEU frame)
     * 
     * @note Coordinate frame: NEU (North-East-Up) with origin at EKF origin
     * @note Units: cm for all axes
     * @note Origin is set automatically when destination is updated
     */
    const Vector3f &get_wp_origin_NEU_cm() const { return _origin_neu_cm; }

    /**
     * @brief Check if altitude is terrain-relative or EKF-origin-relative
     * 
     * @return true if origin.z and destination.z are altitude-above-terrain, false if above EKF origin
     * 
     * @note Both origin and destination use the same altitude frame
     * @note Terrain-relative altitude requires terrain data availability
     */
    bool origin_and_destination_are_terrain_alt() const { return _is_terrain_alt; }

    /**
     * @brief Set waypoint destination using Location (global coordinates)
     * 
     * @details Sets the waypoint destination from a Location object containing latitude,
     *          longitude, and altitude. Converts to NEU position vector for trajectory
     *          generation. Altitude frame (AMSL or terrain-relative) is determined from
     *          the Location object.
     * 
     * @param[in] destination Target waypoint as Location (global coordinates)
     * 
     * @return true if destination set successfully, false if conversion failed
     * 
     * @note Conversion can fail if terrain data is required but unavailable
     * @note Sets origin to current vehicle position automatically
     * @note Triggers trajectory recalculation with S-curve profile
     * 
     * @see set_wp_destination_next_loc() to provide lookahead for corner optimization
     */
    bool set_wp_destination_loc(const Location& destination);
    
    /**
     * @brief Set next waypoint destination for corner optimization
     * 
     * @details Provides lookahead information for multi-leg trajectory blending. Enables
     *          corner cutting by generating a trajectory that smoothly transitions from
     *          current destination toward the next destination.
     * 
     * @param[in] destination Next waypoint destination as Location (global coordinates)
     * 
     * @return true if next destination set successfully, false if conversion failed
     * 
     * @note Must be called after set_wp_destination_loc() for same trajectory
     * @note Enables corner cutting to reduce total flight time
     * @note Next destination is used for trajectory blending at current waypoint
     */
    bool set_wp_destination_next_loc(const Location& destination);

    /**
     * @brief Get destination as Location object
     * 
     * @details Converts the current destination from NEU position vector back to global
     *          Location (latitude, longitude, altitude). Altitude frame will be AMSL or
     *          terrain-relative depending on origin_and_destination_are_terrain_alt().
     * 
     * @param[out] destination Location object with destination coordinates
     * 
     * @return true if conversion successful, false if unable to convert (e.g., origin not set)
     * 
     * @note Altitude frame in returned Location matches the frame used when destination was set
     */
    bool get_wp_destination_loc(Location& destination) const;

    /**
     * @brief Get object-avoidance-adjusted destination
     * 
     * @details Returns the waypoint destination, potentially adjusted by object avoidance.
     *          In the base AC_WPNav class, this returns the same destination as
     *          get_wp_destination_loc(). The AC_WPNav_OA subclass overrides this method
     *          to return Dijkstra-adjusted paths around obstacles.
     * 
     *          Having this virtual method unifies the AC_WPNav and AC_WPNav_OA interfaces,
     *          making vehicle code simpler and allowing transparent object avoidance integration.
     * 
     * @param[out] destination Location object with (potentially adjusted) destination coordinates
     * 
     * @return true if destination retrieved successfully, false otherwise
     * 
     * @note Virtual method - overridden by AC_WPNav_OA for object avoidance
     * @note Base implementation returns unadjusted destination
     * 
     * @see AC_WPNav_OA for object avoidance implementation
     */
    virtual bool get_oa_wp_destination(Location& destination) const { return get_wp_destination_loc(destination); }

    /**
     * @brief Set waypoint destination using NEU position vector
     * 
     * @details Sets the waypoint destination from a position vector in NEU (North-East-Up)
     *          coordinates relative to the EKF origin. This is the primary method for
     *          setting destinations programmatically.
     * 
     * @param[in] destination_neu_cm Target position vector from EKF origin in cm (NEU frame)
     * @param[in] is_terrain_alt True if z-axis is altitude-above-terrain, false if above EKF origin
     * 
     * @return true if destination set successfully, false if terrain data unavailable (for terrain-relative)
     * 
     * @note Virtual method - can be overridden by AC_WPNav_OA for object avoidance
     * @note Coordinate frame: NEU (North-East-Up) with origin at EKF origin
     * @note Units: cm for all axes
     * @note Sets origin to current vehicle position automatically
     * @note Triggers trajectory recalculation with S-curve profile
     * 
     * @see set_wp_destination_next_NEU_cm() to provide lookahead for corner optimization
     */
    virtual bool set_wp_destination_NEU_cm(const Vector3f& destination_neu_cm, bool is_terrain_alt = false);
    
    /**
     * @brief Set next waypoint destination using NEU position vector
     * 
     * @details Provides lookahead information for multi-leg trajectory blending using NEU
     *          position vectors. Enables corner cutting by generating a trajectory that
     *          smoothly transitions toward the next destination.
     * 
     * @param[in] destination_neu_cm Next waypoint position vector from EKF origin in cm (NEU frame)
     * @param[in] is_terrain_alt True if z-axis is altitude-above-terrain, false if above EKF origin
     * 
     * @return true if next destination set successfully, false if terrain data unavailable
     * 
     * @note Coordinate frame: NEU (North-East-Up) with origin at EKF origin
     * @note Units: cm for all axes
     * @note Must be called after set_wp_destination_NEU_cm() for same trajectory
     * @note Enables corner cutting to reduce total flight time
     */
    bool set_wp_destination_next_NEU_cm(const Vector3f& destination_neu_cm, bool is_terrain_alt = false);

    /**
     * @brief Set waypoint destination using NED position vector (meters)
     * 
     * @details Sets the waypoint destination from a position vector in NED (North-East-Down)
     *          coordinates in meters. This is an alternative interface for systems using
     *          NED conventions. Internally converts to NEU centimeters.
     * 
     * @param[in] destination_NED_m Target position vector from EKF origin in meters (NED frame)
     * 
     * @return true if destination set successfully
     * 
     * @note Coordinate frame: NED (North-East-Down) with origin at EKF origin
     * @note Units: meters for all axes (converted internally to cm NEU)
     * @note Sets origin to current vehicle position automatically
     * @note Triggers trajectory recalculation with S-curve profile
     * 
     * @see set_wp_destination_next_NED_m() to provide lookahead for corner optimization
     */
    bool set_wp_destination_NED_m(const Vector3f& destination_NED_m);
    
    /**
     * @brief Set next waypoint destination using NED position vector (meters)
     * 
     * @details Provides lookahead information for multi-leg trajectory blending using NED
     *          position vectors in meters.
     * 
     * @param[in] destination_NED_m Next waypoint position vector from EKF origin in meters (NED frame)
     * 
     * @return true if next destination set successfully
     * 
     * @note Coordinate frame: NED (North-East-Down) with origin at EKF origin
     * @note Units: meters for all axes (converted internally to cm NEU)
     * @note Must be called after set_wp_destination_NED_m() for same trajectory
     */
    bool set_wp_destination_next_NED_m(const Vector3f& destination_NED_m);

    /**
     * @brief Shift origin and destination horizontally to current position
     * 
     * @details Resets the trajectory by shifting both origin and destination horizontally
     *          to match the current vehicle position. Used to reset the track when taking
     *          off without horizontal position control or when position control is temporarily
     *          disabled and needs to be reestablished.
     * 
     * @note Only shifts North and East components; altitude (Up) is unchanged
     * @note Requires set_wp_destination_NEU_cm() to have been called first
     * @note Maintains trajectory direction while resetting position reference
     * 
     * @todo Not currently used - may be removed in future versions
     */
    void shift_wp_origin_and_destination_to_current_pos_NE(); // todo: Not used

    /**
     * @brief Shift origin and destination to achievable stopping point
     * 
     * @details Resets the trajectory by shifting both origin and destination horizontally
     *          to the position where the vehicle can stop given current velocity and
     *          acceleration limits. Used when horizontal navigation is re-enabled after
     *          having been disabled (e.g., Copter's wp_navalt_min threshold).
     * 
     * @note Only shifts North and East components; altitude (Up) is unchanged
     * @note Stopping point calculation uses current velocity and WPNAV_ACCEL parameter
     * @note Requires set_wp_destination_NEU_cm() to have been called first
     * 
     * @todo Not currently used - may be removed in future versions
     * 
     * @see get_wp_stopping_point_NE_cm() for stopping point calculation
     */
    void shift_wp_origin_and_destination_to_stopping_point_NE(); // todo: Not used

    /**
     * @brief Calculate horizontal stopping point based on current velocity
     * 
     * @details Calculates the position where the vehicle will stop given current horizontal
     *          velocity and waypoint acceleration limits. Uses kinematic equations with
     *          jerk-limited deceleration profile.
     * 
     * @param[out] stopping_point_NE_cm Stopping position in North-East plane (cm from EKF origin)
     * 
     * @note Uses current horizontal velocity from position controller
     * @note Deceleration limited by get_wp_acceleration_cmss()
     * @note Does not account for altitude; use get_wp_stopping_point_NEU_cm() for 3D calculation
     */
    void get_wp_stopping_point_NE_cm(Vector2f& stopping_point_NE_cm) const;
    
    /**
     * @brief Calculate 3D stopping point based on current velocity
     * 
     * @details Calculates the 3D position where the vehicle will stop given current velocity
     *          and acceleration limits in all three axes. Uses kinematic equations with
     *          jerk-limited deceleration profiles.
     * 
     * @param[out] stopping_point 3D stopping position in NEU frame (cm from EKF origin)
     * 
     * @note Uses current 3D velocity from position controller
     * @note Horizontal deceleration limited by get_wp_acceleration_cmss()
     * @note Vertical deceleration limited by get_accel_U_cmss()
     */
    void get_wp_stopping_point_NEU_cm(Vector3f& stopping_point) const;

    /**
     * @brief Get horizontal distance to destination
     * 
     * @details Returns the straight-line horizontal distance from the current vehicle
     *          position to the waypoint destination in the North-East plane.
     * 
     * @return Horizontal distance to destination in cm
     * 
     * @note Virtual method - can be overridden by AC_WPNav_OA for object avoidance
     * @note Distance is in horizontal plane only (altitude ignored)
     * @note Used for arrival detection and progress monitoring
     * 
     * @see reached_wp_destination_NE() for arrival check using this distance
     */
    virtual float get_wp_distance_to_destination_cm() const;

    /**
     * @brief Get bearing to destination in centidegrees
     * 
     * @details Returns the bearing from current position to waypoint destination
     *          measured clockwise from North.
     * 
     * @return Bearing to destination in centidegrees (0-36000, where 0=North, 9000=East)
     * 
     * @note Virtual method - can be overridden by AC_WPNav_OA for object avoidance
     * @note Units: centidegrees (1/100th of a degree)
     * @note Range: 0 to 36000 centidegrees (0 to 360 degrees)
     * 
     * @see get_wp_bearing_to_destination_rad() for radians version
     */
    virtual int32_t get_wp_bearing_to_destination_cd() const;

    /**
     * @brief Get bearing to destination in radians
     * 
     * @details Returns the bearing from current position to waypoint destination
     *          measured clockwise from North in radians.
     * 
     * @return Bearing to destination in radians (0 to 2π, where 0=North, π/2=East)
     * 
     * @note Virtual method - can be overridden by AC_WPNav_OA for object avoidance
     * @note Units: radians
     * @note Range: 0 to 2π radians (0 to 360 degrees)
     * 
     * @see get_wp_bearing_to_destination_cd() for centidegrees version
     */
    virtual float get_wp_bearing_to_destination_rad() const;

    /**
     * @brief Check if vehicle has reached waypoint destination
     * 
     * @details Returns true when the vehicle has come within the waypoint acceptance
     *          radius (WPNAV_RADIUS parameter) of the destination. This indicates the
     *          waypoint has been successfully reached and navigation can proceed to
     *          the next waypoint.
     * 
     * @return true if within WPNAV_RADIUS of destination, false otherwise
     * 
     * @note Virtual method - can be overridden by AC_WPNav_OA for object avoidance
     * @note Acceptance radius set by WPNAV_RADIUS parameter (default varies by vehicle)
     * @note Uses horizontal distance only (altitude ignored)
     * 
     * @see reached_wp_destination_NE() for explicit horizontal-only check
     * @see get_wp_radius_cm() to get acceptance radius
     */
    virtual bool reached_wp_destination() const { return _flags.reached_destination; }

    /**
     * @brief Check if vehicle is within acceptance radius in horizontal plane
     * 
     * @details Explicit check for whether horizontal distance to destination is less
     *          than the waypoint acceptance radius. This is the core arrival detection
     *          logic used by reached_wp_destination().
     * 
     * @return true if horizontal distance < WPNAV_RADIUS, false otherwise
     * 
     * @note Uses horizontal distance only (altitude not considered)
     * @note More explicit than reached_wp_destination() for horizontal-only check
     * 
     * @see get_wp_distance_to_destination_cm() for distance calculation
     * @see get_wp_radius_cm() for acceptance radius value
     */
    bool reached_wp_destination_NE() const {
        return get_wp_distance_to_destination_cm() < _wp_radius_cm;
    }

    /**
     * @brief Get waypoint acceptance radius
     * 
     * @return Waypoint acceptance radius in cm from WPNAV_RADIUS parameter
     * 
     * @note This is the distance threshold for waypoint arrival detection
     * @note Typical values: 100-300 cm depending on vehicle size and mission requirements
     * @note Smaller radius = more precise waypoint tracking but may increase mission time
     */
    float get_wp_radius_cm() const { return _wp_radius_cm; }

    /**
     * @brief Main waypoint navigation controller update
     * 
     * @details Runs the complete waypoint navigation control loop. Advances the trajectory
     *          target along the path, handles terrain following, checks for waypoint arrival,
     *          and updates the position controller with target position/velocity/acceleration.
     * 
     *          This method must be called at 100Hz or higher (matching the main control loop)
     *          for smooth trajectory tracking. Lower update rates will result in degraded
     *          performance and potential instability.
     * 
     * @return true if update successful, false if navigation not initialized or inactive
     * 
     * @note Virtual method - can be overridden by AC_WPNav_OA for object avoidance
     * @note CRITICAL: Must be called at 100Hz or higher for proper operation
     * @note Updates position controller targets each call
     * @note Handles terrain altitude compensation automatically
     * @note Advances trajectory using advance_wp_target_along_track()
     * @note Checks for waypoint arrival and sets reached_destination flag
     * 
     * @warning Calling at rates below 100Hz may cause trajectory tracking errors and instability
     * 
     * @see advance_wp_target_along_track() for trajectory advancement details
     * @see AC_PosControl::update_xy_controller() for position controller integration
     */
    virtual bool update_wpnav();

    /**
     * @brief Check if waypoint navigation is active
     * 
     * @details Returns true if update_wpnav() has been called very recently (within last 200ms),
     *          indicating that waypoint navigation is currently active and running.
     * 
     * @return true if update_wpnav() called within last 200ms, false otherwise
     * 
     * @note Used to verify navigation controller is running
     * @note Timeout of 200ms allows for temporary interruptions
     */
    bool is_active() const;

    /**
     * @brief Force vehicle to stop at next waypoint
     * 
     * @details Commands the trajectory generator to come to a complete stop at the next
     *          waypoint instead of corner cutting. Used by Dijkstra's object avoidance
     *          when the path from current destination to next destination is not clear
     *          and corner cutting would intersect obstacles.
     * 
     * @return true if this had any effect on the path, false if already stopping or not applicable
     * 
     * @note Only affects regular (non-spline) waypoints
     * @note Disables corner cutting for current waypoint transition
     * @note Used primarily by AC_WPNav_OA object avoidance system
     * @note Vehicle will decelerate to zero velocity at waypoint
     * 
     * @see AC_WPNav_OA for object avoidance integration
     */
    bool force_stop_at_next_wp();

    ///
    /// spline methods
    ///

    /**
     * @brief Set spline waypoint destination using Location
     * 
     * @details Sets the destination for a curved spline trajectory using Hermite spline
     *          interpolation. Spline waypoints create smooth curved paths instead of
     *          straight-line segments, reducing aggressive direction changes and improving
     *          passenger comfort and tracking accuracy.
     * 
     *          Spline generation requires lookahead to the next destination to calculate
     *          curve tangent vectors. The next_is_spline parameter indicates whether the
     *          following segment should also be a spline (continuous curvature) or a
     *          straight line (tangent becomes straight at destination).
     * 
     * @param[in] destination Current spline waypoint as Location (global coordinates)
     * @param[in] next_destination Next waypoint for tangent calculation
     * @param[in] next_is_spline True if next segment is also a spline, false for straight line
     * 
     * @return true if destination set successfully, false if conversion failed
     * 
     * @note Requires next_destination for proper spline curve generation
     * @note Spline curves provide smoother flight than straight-line waypoints
     * @note Conversion can fail if terrain data is required but unavailable
     * 
     * @see SplineCurve for mathematical implementation details
     * @see set_wp_destination_loc() for straight-line waypoint alternative
     */
    bool set_spline_destination_loc(const Location& destination, const Location& next_destination, bool next_is_spline);

    /**
     * @brief Set next spline destination for extended lookahead
     * 
     * @details Updates the next destination (after current destination) for multi-segment
     *          spline curve optimization. Allows the trajectory generator to consider
     *          multiple waypoints ahead for smoother long-range curves.
     * 
     * @param[in] next_destination Next waypoint as Location
     * @param[in] next_next_destination Waypoint after next for tangent calculation
     * @param[in] next_next_is_spline True if segment after next is also a spline
     * 
     * @return true if next destination updated successfully, false if conversion failed
     * 
     * @note Provides two-waypoint lookahead for optimal curve generation
     * @note Must be called after set_spline_destination_loc()
     */
    bool set_spline_destination_next_loc(const Location& next_destination, const Location& next_next_destination, bool next_next_is_spline);

    /**
     * @brief Set spline waypoint destination using NEU position vector
     * 
     * @details Sets the destination for a curved spline trajectory using NEU position
     *          vectors. This is the primary method for setting spline destinations
     *          programmatically with position vectors.
     * 
     * @param[in] destination_neu_cm Current spline waypoint in NEU frame (cm from EKF origin)
     * @param[in] is_terrain_alt True if z-axis is altitude-above-terrain
     * @param[in] next_destination_neu_cm Next waypoint for tangent calculation
     * @param[in] next_is_terrain_alt True if next z-axis is altitude-above-terrain
     * @param[in] next_is_spline True if next segment is also a spline
     * 
     * @return true if destination set successfully, false if terrain data unavailable
     * 
     * @note Coordinate frame: NEU (North-East-Up) with origin at EKF origin
     * @note Units: cm for all axes
     * @note Both destination and next_destination must use same altitude frame
     * @note Requires next_destination for proper spline curve generation
     * 
     * @warning destination and next_destination must use same altitude frame (both terrain-relative
     *          or both EKF-relative). Mixing frames will cause altitude errors.
     */
    bool set_spline_destination_NEU_cm(const Vector3f& destination_neu_cm, bool is_terrain_alt, const Vector3f& next_destination_neu_cm, bool next_is_terrain_alt, bool next_is_spline);

    /**
     * @brief Set next spline destination using NEU position vector
     * 
     * @details Updates the next destination (after current destination) using NEU position
     *          vectors for multi-segment spline curve optimization.
     * 
     * @param[in] next_destination_neu_cm Next waypoint in NEU frame (cm from EKF origin)
     * @param[in] next_is_terrain_alt True if next z-axis is altitude-above-terrain
     * @param[in] next_next_destination_neu_cm Waypoint after next for tangent calculation
     * @param[in] next_next_is_terrain_alt True if next-next z-axis is altitude-above-terrain
     * @param[in] next_next_is_spline True if segment after next is also a spline
     * 
     * @return true if next destination updated successfully, false if terrain data unavailable
     * 
     * @note Coordinate frame: NEU (North-East-Up) with origin at EKF origin
     * @note Units: cm for all axes
     * @note All destinations must use same altitude frame
     * @note Must be called after set_spline_destination_NEU_cm()
     * 
     * @warning All waypoints must use same altitude frame (terrain-relative or EKF-relative)
     */
    bool set_spline_destination_next_NEU_cm(const Vector3f& next_destination_neu_cm, bool next_is_terrain_alt, const Vector3f& next_next_destination_neu_cm, bool next_next_is_terrain_alt, bool next_next_is_spline);

    ///
    /// shared methods
    ///

    /**
     * @brief Get desired roll angle in radians from position controller
     * 
     * @details Returns the roll angle target calculated by the position controller
     *          to track the waypoint trajectory. This is the attitude command that
     *          should be passed to the attitude controller.
     * 
     * @return Desired roll angle in radians (+ve = right wing down)
     * 
     * @note Body frame: Positive roll = right wing down (right bank)
     * @note Updated by update_wpnav() at controller rate (typically 100Hz)
     * 
     * @see get_roll() for centidegrees version
     * @see AC_PosControl::get_roll_rad() for calculation details
     */
    float get_roll_rad() const { return _pos_control.get_roll_rad(); }

    /**
     * @brief Get desired pitch angle in radians from position controller
     * 
     * @details Returns the pitch angle target calculated by the position controller
     *          to track the waypoint trajectory. This is the attitude command that
     *          should be passed to the attitude controller.
     * 
     * @return Desired pitch angle in radians (+ve = nose up)
     * 
     * @note Body frame: Positive pitch = nose up
     * @note Updated by update_wpnav() at controller rate (typically 100Hz)
     * 
     * @see get_pitch() for centidegrees version
     * @see AC_PosControl::get_pitch_rad() for calculation details
     */
    float get_pitch_rad() const { return _pos_control.get_pitch_rad(); }

    /**
     * @brief Get desired yaw angle in radians from position controller
     * 
     * @details Returns the yaw angle target from the position controller. For waypoint
     *          navigation, yaw is typically controlled independently (e.g., pointing
     *          toward destination or controlled by pilot).
     * 
     * @return Desired yaw angle in radians (+ve = clockwise viewed from above)
     * 
     * @note Earth frame: 0 = North, π/2 = East, π = South, 3π/2 = West
     * @note Yaw control is typically independent of position control during waypoint navigation
     * 
     * @see get_yaw() for centidegrees version
     */
    float get_yaw_rad() const { return _pos_control.get_yaw_rad(); }

    /**
     * @brief Get desired thrust direction vector from position controller
     * 
     * @details Returns the thrust direction unit vector calculated by the position
     *          controller for trajectory tracking. This is used for thrust vectoring
     *          control on advanced multirotors or tilt-rotor vehicles.
     * 
     * @return Thrust direction unit vector in body frame
     * 
     * @note Body frame: x=forward, y=right, z=down
     * @note Magnitude is normalized (unit vector); separate thrust magnitude control
     * @note Used for thrust vectoring and tilt control
     * 
     * @see AC_PosControl::get_thrust_vector() for calculation details
     */
    Vector3f get_thrust_vector() const { return _pos_control.get_thrust_vector(); }

    /**
     * @brief Get desired roll angle in centidegrees from position controller
     * 
     * @details Returns the roll angle target in centidegrees (1/100th of a degree).
     *          This is a convenience method providing the same information as
     *          get_roll_rad() in a different unit.
     * 
     * @return Desired roll angle in centidegrees (+ve = right wing down)
     * 
     * @note Units: centidegrees (1/100th of a degree)
     * @note Body frame: Positive roll = right wing down (right bank)
     * 
     * @see get_roll_rad() for radians version
     */
    float get_roll() const { return _pos_control.get_roll_cd(); }

    /**
     * @brief Get desired pitch angle in centidegrees from position controller
     * 
     * @details Returns the pitch angle target in centidegrees (1/100th of a degree).
     *          This is a convenience method providing the same information as
     *          get_pitch_rad() in a different unit.
     * 
     * @return Desired pitch angle in centidegrees (+ve = nose up)
     * 
     * @note Units: centidegrees (1/100th of a degree)
     * @note Body frame: Positive pitch = nose up
     * 
     * @see get_pitch_rad() for radians version
     */
    float get_pitch() const { return _pos_control.get_pitch_cd(); }

    /**
     * @brief Get desired yaw angle in centidegrees from position controller
     * 
     * @details Returns the yaw angle target in centidegrees (1/100th of a degree).
     *          This is a convenience method providing the same information as
     *          get_yaw_rad() in a different unit.
     * 
     * @return Desired yaw angle in centidegrees (+ve = clockwise from above)
     * 
     * @note Units: centidegrees (1/100th of a degree)
     * @note Earth frame: 0 = North, 9000 = East, 18000 = South, 27000 = West
     * 
     * @see get_yaw_rad() for radians version
     */
    float get_yaw() const { return _pos_control.get_yaw_cd(); }

    /**
     * @brief Advance trajectory target along track from origin to destination
     * 
     * @details Moves the target position/velocity/acceleration forward along the
     *          trajectory based on elapsed time and vehicle dynamics. This is the core
     *          trajectory tracking algorithm that generates smooth motion profiles using
     *          S-curve acceleration with jerk and snap limiting.
     * 
     *          The method handles:
     *          - Time-optimal trajectory generation constrained by speed/accel/jerk/snap
     *          - Multi-leg blending for corner cutting
     *          - Terrain altitude compensation
     *          - Pause/resume functionality
     *          - Trajectory recalculation when parameters change
     * 
     * @param[in] dt Time step in seconds since last call (typically 0.01s at 100Hz)
     * 
     * @return true if trajectory advancement successful, false if error or destination reached
     * 
     * @note Called internally by update_wpnav() each control cycle
     * @note Uses SCurve trajectory engine for jerk-limited profiles
     * @note Handles transitions between waypoint segments automatically
     * @note Compensates for terrain altitude when terrain following is active
     * 
     * @see SCurve for S-curve trajectory mathematics
     * @see update_wpnav() for main controller update loop
     */
    bool advance_wp_target_along_track(float dt);

    /**
     * @brief Recalculate trajectory with updated speed and/or acceleration limits
     * 
     * @details Forces recalculation of the trajectory when speed or acceleration
     *          parameters change during flight. Generates a new S-curve profile from
     *          the current position/velocity to the destination using updated constraints.
     * 
     *          This method is called automatically when:
     *          - Speed parameters change (WPNAV_SPEED, WPNAV_SPEED_UP, WPNAV_SPEED_DN)
     *          - Acceleration parameters change (WPNAV_ACCEL, WPNAV_ACCEL_Z)
     *          - set_speed_NE_cms() or similar methods are called
     * 
     * @note Maintains current position and velocity for smooth transition
     * @note Recalculates jerk and snap limits from new acceleration constraints
     * @note May be called mid-trajectory without discontinuities
     * 
     * @see calc_scurve_jerk_and_snap() for constraint calculation
     * @see set_speed_NE_cms() for runtime speed changes
     */
    void update_track_with_speed_accel_limits();

    /**
     * @brief Get crosstrack error from position controller
     * 
     * @details Returns the horizontal distance between the vehicle's actual position
     *          and the desired position on the trajectory. Positive crosstrack error
     *          indicates the vehicle is to the right of the desired track; negative
     *          indicates left of track.
     * 
     * @return Crosstrack error in cm (+ve = right of track, -ve = left of track)
     * 
     * @note Crosstrack error is the perpendicular distance to the trajectory line
     * @note Large crosstrack errors may indicate wind, poor tuning, or excessive speed
     * @note Used for monitoring tracking performance and diagnostics
     * 
     * @see AC_PosControl::crosstrack_error() for calculation details
     */
    float crosstrack_error() const { return _pos_control.crosstrack_error();}

    /**
     * @brief Parameter table for waypoint navigation configuration
     * 
     * @details Defines the parameter group for AC_WPNav, allowing runtime configuration
     *          of waypoint navigation behavior. Parameters are stored in persistent
     *          storage (EEPROM/flash) and can be modified via ground control station.
     * 
     * Parameters in var_info:
     * - WPNAV_SPEED: Default horizontal speed in cm/s for waypoint navigation
     * - WPNAV_SPEED_UP: Maximum climb rate in cm/s during missions
     * - WPNAV_SPEED_DN: Maximum descent rate in cm/s during missions
     * - WPNAV_RADIUS: Waypoint acceptance radius in cm (distance threshold for arrival)
     * - WPNAV_ACCEL: Horizontal acceleration in cm/s² for trajectory generation
     * - WPNAV_ACCEL_C: Cornering acceleration in cm/s² for waypoint transitions
     * - WPNAV_ACCEL_Z: Vertical acceleration in cm/s² for climb/descent
     * - WPNAV_JERK: Maximum jerk (rate of acceleration change) in m/s³ for S-curves
     * - WPNAV_TER_MARGIN: Terrain following altitude margin in meters (safety buffer)
     * - WPNAV_RFND_USE: Enable rangefinder for terrain following (user switch)
     * 
     * @note Parameter values are loaded during constructor and can be changed in-flight
     * @note Speed and acceleration changes trigger trajectory recalculation
     * @note Jerk parameter uses m/s³ (not cm/s³) for numerical stability
     * 
     * @see AP_Param for parameter system details
     * @see wp_and_spline_init_cm() for parameter initialization
     */
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // flags structure
    struct wpnav_flags {
        uint8_t reached_destination     : 1;    // true if we have reached the destination
        uint8_t fast_waypoint           : 1;    // true if we should ignore the waypoint radius and consider the waypoint complete once the intermediate target has reached the waypoint
        uint8_t wp_yaw_set              : 1;    // true if yaw target has been set
    } _flags;

    // helper function to calculate scurve jerk and jerk_time values
    // updates _scurve_jerk_max_msss and _scurve_snap_max_mssss
    void calc_scurve_jerk_and_snap();

    // references and pointers to external libraries
    const AP_AHRS_View&     _ahrs;
    AC_PosControl&          _pos_control;
    const AC_AttitudeControl& _attitude_control;

    // parameters
    AP_Float    _wp_speed_cms;      // default maximum horizontal speed in cm/s during missions
    AP_Float    _wp_speed_up_cms;   // default maximum climb rate in cm/s
    AP_Float    _wp_speed_down_cms; // default maximum descent rate in cm/s
    AP_Float    _wp_radius_cm;      // distance from a waypoint in cm that, when crossed, indicates the wp has been reached
    AP_Float    _wp_accel_cmss;     // horizontal acceleration in cm/s/s during missions
    AP_Float    _wp_accel_c_cmss;   // cornering acceleration in cm/s/s during missions
    AP_Float    _wp_accel_z_cmss;   // vertical acceleration in cm/s/s during missions
    AP_Float    _wp_jerk_msss;      // maximum jerk used to generate scurve trajectories in m/s/s/s
    AP_Float    _terrain_margin_m;  // terrain following altitude margin. vehicle will stop if distance from target altitude is larger than this margin

    // WPNAV_SPEED param change checker
    bool _check_wp_speed_change;    // if true WPNAV_SPEED param should be checked for changes in-flight
    float _last_wp_speed_cms;       // last recorded WPNAV_SPEED, used for changing speed in-flight
    float _last_wp_speed_up_cms;    // last recorded WPNAV_SPEED_UP, used for changing speed in-flight
    float _last_wp_speed_down_cms;  // last recorded WPNAV_SPEED_DN, used for changing speed in-flight

    // scurve
    SCurve _scurve_prev_leg;        // previous scurve trajectory used to blend with current scurve trajectory
    SCurve _scurve_this_leg;        // current scurve trajectory
    SCurve _scurve_next_leg;        // next scurve trajectory used to blend with current scurve trajectory
    float _scurve_jerk_max_msss;    // scurve jerk max in m/s/s/s
    float _scurve_snap_max_mssss;   // scurve snap in m/s/s/s/s

    // spline curves
    SplineCurve _spline_this_leg;   // spline curve for current segment
    SplineCurve _spline_next_leg;   // spline curve for next segment

    // the type of this leg
    bool _this_leg_is_spline;       // true if this leg is a spline
    bool _next_leg_is_spline;       // true if the next leg is a spline

    // waypoint controller internal variables
    uint32_t    _wp_last_update_ms;         // time of last update_wpnav call (in ms)
    float       _wp_desired_speed_ne_cms;   // desired wp speed in cm/sec
    Vector3f    _origin_neu_cm;             // starting point of trip to next waypoint in cm from ekf origin
    Vector3f    _destination_neu_cm;        // target destination in cm from ekf origin
    Vector3f    _next_destination_neu_cm;   // next target destination in cm from ekf origin
    float       _track_dt_scalar;           // time compression multiplier to slow the progress along the track
    float       _offset_vel_cms;            // horizontal velocity reference used to slow the aircraft for pause and to ensure the aircraft can maintain height above terrain
    float       _offset_accel_cmss;         // horizontal acceleration reference used to slow the aircraft for pause and to ensure the aircraft can maintain height above terrain
    bool        _paused;                    // flag for pausing waypoint controller

    // terrain following variables
    bool        _is_terrain_alt;                // true if origin and destination.z are alt-above-terrain, false if alt-above-ekf-origin
    bool        _rangefinder_available;         // true if rangefinder is enabled (user switch can turn this true/false)
    AP_Int8     _rangefinder_use;               // parameter that specifies if the range finder should be used for terrain following commands
    bool        _rangefinder_healthy;           // true if rangefinder distance is healthy (i.e. between min and maximum)
    float       _rangefinder_terrain_offset_cm; // latest rangefinder based terrain offset (e.g. terrain's height above EKF origin)
};
