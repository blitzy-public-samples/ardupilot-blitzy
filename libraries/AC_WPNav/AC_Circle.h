/**
 * @file AC_Circle.h
 * @brief Circular flight and panorama mode controller for autonomous vehicles
 * 
 * @details This file implements the AC_Circle class which provides autonomous circular
 *          flight control for multicopters and other vehicles. The controller manages
 *          position, velocity, and acceleration to maintain a circular flight path
 *          around a configurable center point with adjustable radius and rotation rate.
 *          
 *          Key Features:
 *          - Circular flight path control with configurable radius (10cm to 2km)
 *          - Adjustable rotation rate (clockwise or counter-clockwise)
 *          - Terrain following support via rangefinder and AP_Terrain database
 *          - Panorama mode for photography/videography with optional ROI at center
 *          - Manual pilot control of radius and rate via stick inputs
 *          - Integration with AC_PosControl for smooth trajectory generation
 *          
 *          Coordinate System: NEU (North-East-Up) in centimeters from EKF origin
 *          
 *          Typical Update Rate: ~100Hz (main loop rate)
 *          
 *          Integration: This class works in conjunction with AC_PosControl to generate
 *          velocity and acceleration targets that are fed to the attitude controller.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AC_AttitudeControl/AC_PosControl.h>      // Position control library

// loiter maximum velocities and accelerations
#define AC_CIRCLE_RADIUS_DEFAULT    1000.0f     // radius of the circle in cm that the vehicle will fly
#define AC_CIRCLE_RATE_DEFAULT      20.0f       // turn rate in deg/sec.  Positive to turn clockwise, negative for counter clockwise
#define AC_CIRCLE_ANGULAR_ACCEL_MIN 2.0f        // angular acceleration should never be less than 2deg/sec
#define AC_CIRCLE_RADIUS_MAX        200000.0f   // maximum allowed circle radius of 2km

/**
 * @class AC_Circle
 * @brief Autonomous circular flight controller with terrain following support
 * 
 * @details AC_Circle manages circular flight paths for autonomous vehicles, providing smooth
 *          and predictable circular motion with configurable parameters. The controller
 *          integrates with AC_PosControl to generate position, velocity, and acceleration
 *          targets that maintain a circular trajectory around a defined center point.
 *          
 *          Architecture:
 *          - Maintains circular path state (angle, angular velocity, angular acceleration)
 *          - Generates position/velocity targets for AC_PosControl at update rate
 *          - Supports terrain following via rangefinder or AP_Terrain database
 *          - Handles smooth transitions when radius or rate parameters change
 *          - Provides yaw control targeting circle center or direction of travel
 *          
 *          Usage Pattern:
 *          1. Construct with references to AHRS and PosControl
 *          2. Initialize with init() or init_NEU_cm() to set center and rate
 *          3. Call update_cms() at main loop rate (~100Hz) with climb rate
 *          4. Retrieve roll/pitch/yaw targets via get_roll_cd(), get_pitch_cd(), get_yaw_cd()
 *          5. Feed targets to attitude controller
 *          
 *          Configuration Options (CircleOptions enum):
 *          - MANUAL_CONTROL: Enable pilot stick control of radius and rotation rate
 *          - FACE_DIRECTION_OF_TRAVEL: Yaw follows tangent to circle path
 *          - INIT_AT_CENTER: Start circle at current position (vs 1 radius ahead)
 *          - ROI_AT_CENTER: Set mount/gimbal ROI at circle center for panoramas
 *          
 *          Coordinate System:
 *          - All positions in NEU (North-East-Up) frame
 *          - Units: centimeters from EKF origin
 *          - Altitude can be absolute (from EKF origin) or terrain-relative
 *          
 *          Units Convention:
 *          - Position: centimeters (cm)
 *          - Velocity: centimeters per second (cm/s) for linear, degrees per second (deg/s) for angular
 *          - Acceleration: centimeters per second squared (cm/s²) for linear, degrees per second squared (deg/s²) for angular
 *          - Angles: radians for internal calculations, degrees for user-facing parameters
 *          - Attitude outputs: centidegrees (cd)
 *          
 *          Terrain Following:
 *          - Supports terrain-relative altitude via rangefinder or AP_Terrain database
 *          - Automatically selects terrain source based on availability and health
 *          - Falls back to absolute altitude if terrain data unavailable
 *          - Returns terrain failsafe failure from update_cms() if terrain required but unavailable
 *          
 *          Performance Characteristics:
 *          - Update rate: Typically 100Hz (main loop rate)
 *          - Minimum angular acceleration: 2.0 deg/s² (AC_CIRCLE_ANGULAR_ACCEL_MIN)
 *          - Maximum radius: 2000 meters (AC_CIRCLE_RADIUS_MAX)
 *          - Default radius: 10 meters (AC_CIRCLE_RADIUS_DEFAULT)
 *          - Default rate: 20 deg/s clockwise (AC_CIRCLE_RATE_DEFAULT)
 *          
 *          Integration with AC_PosControl:
 *          - Generates position targets in NEU frame
 *          - Provides velocity and acceleration feedforward for smooth tracking
 *          - Relies on AC_PosControl for actual attitude/thrust control
 *          - PosControl speeds and accelerations should be configured before init
 *          
 * @note This controller is typically called at the main loop rate (~100Hz). Higher
 *       rates provide smoother circular motion and better trajectory tracking.
 * 
 * @warning Angular acceleration below AC_CIRCLE_ANGULAR_ACCEL_MIN (2.0 deg/s²) can
 *          cause instability in the trajectory generation. The controller enforces
 *          this minimum to ensure stable flight. Rapid changes to radius or rate
 *          parameters during flight may cause momentary tracking errors as the
 *          controller recalculates velocities and accelerations.
 * 
 * @see AC_PosControl for position/velocity control integration
 * @see AC_WPNav for related waypoint navigation functionality
 */
class AC_Circle
{
public:

    /**
     * @brief Construct a new AC_Circle controller
     * 
     * @details Creates a circular flight controller instance with references to the
     *          AHRS (Attitude and Heading Reference System) and position controller.
     *          The constructor initializes internal state but does not start circular
     *          flight - call init() or init_NEU_cm() to begin circle operation.
     * 
     * @param[in] ahrs Reference to AP_AHRS_View for vehicle attitude and position
     * @param[in] pos_control Reference to AC_PosControl for position/velocity control
     * 
     * @note The AHRS and PosControl references are stored and must remain valid for
     *       the lifetime of this AC_Circle object.
     */
    AC_Circle(const AP_AHRS_View& ahrs, AC_PosControl& pos_control);

    /**
     * @brief Initialize circle controller with explicit center position
     * 
     * @details Initializes the circular flight controller with a specified center point,
     *          terrain altitude mode, and rotation rate. This is the preferred initialization
     *          method when the circle center location is known. The controller will begin
     *          circular motion from the vehicle's current position, calculating the starting
     *          angle based on position relative to the center.
     *          
     *          Before calling this method, configure the position controller's maximum
     *          speeds and accelerations to appropriate values for circular flight.
     *          
     *          The vehicle will circle around center_neu_cm at the current configured
     *          radius (from parameters or set_radius_cm). Angular velocity and acceleration
     *          are automatically calculated based on radius and rate to ensure smooth motion.
     * 
     * @param[in] center_neu_cm Circle center position in NEU frame (North-East-Up), in centimeters from EKF origin
     * @param[in] is_terrain_alt True if center_neu_cm.z should be interpreted as altitude above terrain,
     *                           false for altitude above EKF origin
     * @param[in] rate_degs Rotation rate in degrees per second. Positive values turn clockwise (CW),
     *                      negative values turn counter-clockwise (CCW)
     * 
     * @note Caller must configure AC_PosControl's x, y, and z speeds and accelerations
     *       before calling this method to ensure appropriate velocity limits.
     * 
     * @note If is_terrain_alt is true, terrain following will be active and terrain data
     *       must be available (from rangefinder or AP_Terrain database) for successful operation.
     * 
     * @see init() for initialization using vehicle heading to determine center
     * @see set_radius_cm() to adjust circle radius
     * @see AC_PosControl::set_max_speed_xy() and related methods for speed configuration
     */
    void init_NEU_cm(const Vector3p& center_neu_cm, bool is_terrain_alt, float rate_degs);

    /**
     * @brief Initialize circle controller using vehicle heading to project center
     * 
     * @details Initializes circular flight by calculating the circle center based on the
     *          vehicle's current position and heading. The center is projected ahead of
     *          the vehicle by one radius distance along the current heading direction
     *          (unless INIT_AT_CENTER option is set, in which case center is current position).
     *          
     *          This initialization method is useful when entering circle mode during flight,
     *          as it creates a smooth transition by positioning the circle based on the
     *          vehicle's current state.
     *          
     *          Before calling this method, configure the position controller's maximum
     *          speeds and accelerations to appropriate values for circular flight.
     *          
     *          The rotation rate is taken from the RATE parameter (_rate_parm_degs).
     * 
     * @note Caller must configure AC_PosControl's x, y, and z speeds and accelerations
     *       before calling this method to ensure appropriate velocity limits.
     * 
     * @note If CircleOptions::INIT_AT_CENTER flag is set, the circle center will be the
     *       vehicle's current position. Otherwise, center is projected 1 radius ahead.
     * 
     * @see init_NEU_cm() for initialization with explicit center position
     * @see set_rate_degs() to adjust rotation rate after initialization
     * @see AC_PosControl::set_max_speed_xy() and related methods for speed configuration
     */
    void init();

    /**
     * @brief Set circle center using a Location
     * 
     * @details Updates the circle center to the specified Location, which includes
     *          latitude, longitude, altitude, and altitude frame (absolute or terrain-relative).
     *          The Location is internally converted to NEU coordinates from EKF origin.
     *          
     *          This method is useful when the center is specified in GPS coordinates
     *          or loaded from a mission waypoint.
     * 
     * @param[in] center Circle center as a Location (lat/lon/alt)
     * 
     * @note The altitude frame from the Location determines whether terrain-relative
     *       altitude is used (_is_terrain_alt flag).
     * 
     * @see set_center_NEU_cm() for setting center using NEU coordinates directly
     */
    void set_center(const Location& center);

    /**
     * @brief Set circle center as a vector from EKF origin
     * 
     * @details Updates the circle center position using NEU (North-East-Up) coordinates
     *          in centimeters from the EKF origin. This method provides direct control
     *          over the center position in the local coordinate frame.
     * 
     * @param[in] center_neu_cm Circle center position in NEU frame, in centimeters from EKF origin
     * @param[in] is_terrain_alt True if center_neu_cm.z is altitude above terrain,
     *                           false if altitude above EKF origin
     * 
     * @note When is_terrain_alt is true, the controller will maintain terrain-relative
     *       altitude using rangefinder or AP_Terrain database.
     * 
     * @see set_center() for setting center using GPS Location
     * @see get_center_NEU_cm() to retrieve current center position
     */
    void set_center_NEU_cm(const Vector3f& center_neu_cm, bool is_terrain_alt) { _center_neu_cm = center_neu_cm.topostype(); _is_terrain_alt = is_terrain_alt; }

    /**
     * @brief Get circle center position in NEU frame
     * 
     * @details Returns the current circle center position in NEU (North-East-Up)
     *          coordinates in centimeters from the EKF origin.
     * 
     * @return const Vector3p& Circle center position in NEU frame (cm from EKF origin)
     * 
     * @note The returned reference is valid as long as this AC_Circle object exists.
     * 
     * @see center_is_terrain_alt() to check if altitude is terrain-relative
     * @see set_center_NEU_cm() to update center position
     */
    const Vector3p& get_center_NEU_cm() const { return _center_neu_cm; }

    /**
     * @brief Check if circle center altitude is terrain-relative
     * 
     * @details Returns whether the circle center's altitude (_center_neu_cm.z) is
     *          interpreted as altitude above terrain (true) or altitude above EKF
     *          origin (false).
     * 
     * @return true if using terrain-relative altitude, false if using absolute altitude from EKF origin
     * 
     * @see set_center_NEU_cm() to configure altitude reference mode
     * @see get_terrain_source() for terrain data source information
     */
    bool center_is_terrain_alt() const { return _is_terrain_alt; }

    /**
     * @brief Get current circle radius
     * 
     * @details Returns the active circle radius in centimeters. If a runtime radius
     *          has been set (via set_radius_cm or pilot control), that value is returned.
     *          Otherwise, returns the radius from the RADIUS parameter (_radius_parm_cm).
     * 
     * @return float Circle radius in centimeters
     * 
     * @note The returned radius is always positive. Radius is clamped to maximum
     *       value of AC_CIRCLE_RADIUS_MAX (200000 cm = 2 km).
     * 
     * @see set_radius_cm() to update the circle radius
     */
    float get_radius_cm() const { return is_positive(_radius_cm)?_radius_cm:_radius_parm_cm; }

    /**
     * @brief Set circle radius
     * 
     * @details Updates the circle radius to the specified value in centimeters.
     *          The controller automatically recalculates angular velocity and
     *          acceleration to maintain the configured rotation rate with the new radius.
     *          
     *          Changing radius during flight causes smooth transition to the new
     *          circular path. The vehicle will spiral inward or outward to reach
     *          the new radius while maintaining angular rate.
     * 
     * @param[in] radius_cm Desired circle radius in centimeters (must be positive)
     * 
     * @note Radius is clamped to the range [0, AC_CIRCLE_RADIUS_MAX] where
     *       AC_CIRCLE_RADIUS_MAX is 200000 cm (2 km).
     * 
     * @note Changing radius triggers recalculation of angular velocity and
     *       acceleration via calc_velocities() to maintain smooth circular motion.
     * 
     * @warning Very large radius changes during flight may cause momentary tracking
     *          errors as the vehicle transitions to the new circular path.
     * 
     * @see get_radius_cm() to retrieve current radius
     * @see calc_velocities() for velocity recalculation details
     */
    void set_radius_cm(float radius_cm);

    /**
     * @brief Get configured rotation rate
     * 
     * @details Returns the target rotation rate in degrees per second as configured
     *          by set_rate_degs() or from the RATE parameter. This is the desired rate,
     *          which may differ from the actual instantaneous rate during acceleration.
     * 
     * @return float Target rotation rate in degrees per second (positive = clockwise, negative = counter-clockwise)
     * 
     * @note Positive values indicate clockwise rotation (when viewed from above),
     *       negative values indicate counter-clockwise rotation.
     * 
     * @see get_rate_current() for the actual instantaneous rotation rate
     * @see set_rate_degs() to update the rotation rate
     */
    float get_rate_degs() const { return _rate_degs; }

    /**
     * @brief Get actual instantaneous rotation rate
     * 
     * @details Returns the current calculated angular velocity in degrees per second,
     *          which represents the actual instantaneous rotation rate. This may be
     *          less than the target rate (_rate_degs) during acceleration or when
     *          the controller is limiting rate due to position/velocity constraints.
     * 
     * @return float Actual angular velocity in degrees per second (positive = clockwise, negative = counter-clockwise)
     * 
     * @note This value is calculated from _angular_vel_rads (internal radians/sec value)
     *       and represents the vehicle's actual rotation rate at the current moment.
     * 
     * @see get_rate_degs() for the configured target rate
     * @see get_angle_total_rad() for total angle traveled
     */
    float get_rate_current() const { return degrees(_angular_vel_rads); }

    /**
     * @brief Set rotation rate
     * 
     * @details Updates the target rotation rate in degrees per second. The controller
     *          automatically recalculates angular velocity and acceleration to achieve
     *          the new rate with the current radius, ensuring smooth circular motion.
     *          
     *          Positive rates produce clockwise rotation (when viewed from above),
     *          negative rates produce counter-clockwise rotation.
     *          
     *          Changing rate during flight causes smooth acceleration or deceleration
     *          to the new angular velocity.
     * 
     * @param[in] rate_degs Desired rotation rate in degrees per second
     *                      (positive = clockwise, negative = counter-clockwise)
     * 
     * @note Rate changes trigger recalculation of angular velocity and acceleration
     *       via calc_velocities() to maintain smooth circular motion.
     * 
     * @note Angular acceleration is subject to minimum limit of AC_CIRCLE_ANGULAR_ACCEL_MIN
     *       (2.0 deg/s²) to ensure stability.
     * 
     * @warning Rapid rate changes during flight may cause momentary tracking errors
     *          as the controller accelerates to the new angular velocity.
     * 
     * @see get_rate_degs() to retrieve configured rate
     * @see get_rate_current() to retrieve actual instantaneous rate
     * @see calc_velocities() for velocity recalculation details
     */
    void set_rate_degs(float rate_degs);

    /**
     * @brief Get total angle traveled around circle
     * 
     * @details Returns the cumulative angle in radians that the vehicle has traveled
     *          around the circle since initialization. This value continuously increases
     *          (or decreases for counter-clockwise) and is not wrapped to 2π.
     *          
     *          Useful for tracking progress in circle mode, detecting completion of
     *          full rotations, or implementing behaviors after specific angular travel.
     * 
     * @return float Total angle traveled in radians (positive = clockwise, negative = counter-clockwise)
     * 
     * @note This value is cumulative and not wrapped - after one complete clockwise
     *       rotation it will be approximately 2π (6.28), after two rotations ~4π, etc.
     * 
     * @note For panorama photography, this can be used to trigger camera shots at
     *       specific angular intervals.
     * 
     * @see get_rate_current() for instantaneous rotation rate
     */
    float get_angle_total_rad() const { return _angle_total_rad; }

    /**
     * @brief Update circle controller (main loop function)
     * 
     * @details This is the main update function that must be called at the main loop rate
     *          (typically ~100Hz) to maintain circular flight. The function performs:
     *          
     *          1. Calculates current position on circular path based on angular velocity
     *          2. Updates angular position and total angle traveled
     *          3. Handles terrain following (if enabled) via rangefinder or AP_Terrain
     *          4. Generates position, velocity, and acceleration targets for AC_PosControl
     *          5. Updates yaw target (toward center or direction of travel based on options)
     *          6. Calls AC_PosControl update to generate attitude targets
     *          
     *          The climb_rate_cms parameter allows independent vertical velocity control
     *          while maintaining horizontal circular motion.
     * 
     * @param[in] climb_rate_cms Desired vertical climb rate in centimeters per second.
     *                           Positive = climb, negative = descend, 0 = maintain altitude.
     *                           Default is 0.0 (maintain altitude).
     * 
     * @return true if update successful, false if terrain failsafe triggered
     * 
     * @note This function MUST be called at regular intervals (typically 100Hz) for
     *       smooth circular flight. Irregular update rates will cause jerky motion.
     * 
     * @note Return value should be checked (WARN_IF_UNUSED attribute). A false return
     *       indicates terrain data is required but unavailable, triggering a terrain failsafe.
     * 
     * @warning If terrain following is active (_is_terrain_alt = true) and terrain data
     *          becomes unavailable, this function returns false to indicate terrain failsafe.
     *          The calling code must handle this condition appropriately (e.g., switch to
     *          a safe flight mode).
     * 
     * @see get_roll_cd(), get_pitch_cd(), get_yaw_cd() to retrieve attitude targets after update
     * @see is_active() to check if update has been called recently
     */
    bool update_cms(float climb_rate_cms = 0.0f) WARN_IF_UNUSED;

    /**
     * @brief Get desired roll angle
     * 
     * @details Returns the desired roll angle in centidegrees that should be fed to
     *          the attitude controller. This value is generated by AC_PosControl
     *          based on the position and velocity targets calculated by the circle controller.
     * 
     * @return float Desired roll angle in centidegrees (positive = roll right)
     * 
     * @note This value is only valid after update_cms() has been called.
     * 
     * @see AC_PosControl::get_roll_cd() for implementation details
     * @see update_cms() which generates the roll target
     */
    float get_roll_cd() const { return _pos_control.get_roll_cd(); }

    /**
     * @brief Get desired pitch angle
     * 
     * @details Returns the desired pitch angle in centidegrees that should be fed to
     *          the attitude controller. This value is generated by AC_PosControl
     *          based on the position and velocity targets calculated by the circle controller.
     * 
     * @return float Desired pitch angle in centidegrees (positive = pitch up/nose up)
     * 
     * @note This value is only valid after update_cms() has been called.
     * 
     * @see AC_PosControl::get_pitch_cd() for implementation details
     * @see update_cms() which generates the pitch target
     */
    float get_pitch_cd() const { return _pos_control.get_pitch_cd(); }

    /**
     * @brief Get desired thrust vector
     * 
     * @details Returns the desired thrust vector (normalized acceleration) that should
     *          be applied to maintain the circular flight path. This vector is generated
     *          by AC_PosControl and includes both horizontal and vertical components.
     * 
     * @return Vector3f Desired thrust vector (normalized, body frame)
     * 
     * @note This value is only valid after update_cms() has been called.
     * 
     * @note The thrust vector is in body frame and normalized. The magnitude and
     *       direction indicate the required thrust to track the circular path.
     * 
     * @see AC_PosControl::get_thrust_vector() for implementation details
     * @see update_cms() which generates the thrust vector
     */
    Vector3f get_thrust_vector() const { return _pos_control.get_thrust_vector(); }

    /**
     * @brief Get desired yaw angle in centidegrees
     * 
     * @details Returns the desired yaw (heading) angle in centidegrees. Depending on
     *          CircleOptions configuration, yaw may point toward the circle center
     *          (default) or tangent to the circle in the direction of travel
     *          (FACE_DIRECTION_OF_TRAVEL option).
     * 
     * @return float Desired yaw angle in centidegrees (0 = North, increases clockwise)
     * 
     * @note This value is only valid after update_cms() has been called.
     * 
     * @note Default behavior: yaw points toward circle center (useful for panorama mode)
     *       With FACE_DIRECTION_OF_TRAVEL: yaw follows direction of motion
     * 
     * @see get_yaw_rad() for yaw in radians
     * @see update_cms() which generates the yaw target
     */
    float get_yaw_cd() const { return _yaw_cd; }

    /**
     * @brief Get desired yaw angle in radians
     * 
     * @details Returns the desired yaw (heading) angle in radians. This is a convenience
     *          function that converts the internal centidegree yaw value to radians.
     * 
     * @return float Desired yaw angle in radians (0 = North, increases clockwise)
     * 
     * @note This value is only valid after update_cms() has been called.
     * 
     * @see get_yaw_cd() for yaw in centidegrees
     */
    float get_yaw_rad() const { return cd_to_rad(_yaw_cd); }

    /**
     * @brief Check if circle controller is active
     * 
     * @details Returns true if update_cms() has been called recently (within a short
     *          time threshold). This is used by vehicle code to determine if the yaw
     *          and attitude outputs are valid and current.
     *          
     *          An inactive controller indicates that update_cms() has not been called,
     *          and output values (roll, pitch, yaw) should not be used.
     * 
     * @return true if update_cms() called recently, false if controller is stale
     * 
     * @note "Recently" is determined by comparing current time to _last_update_ms with
     *       a short timeout threshold.
     * 
     * @see update_cms() which updates the _last_update_ms timestamp
     */
    bool is_active() const;

    /**
     * @brief Get closest point on the circle to vehicle position
     * 
     * @details Calculates the point on the circular path that is closest to the vehicle's
     *          current horizontal position. This is useful for determining the nearest
     *          point on the circle for path following or recovery from position errors.
     *          
     *          The function projects the vehicle's horizontal position onto the circle
     *          and returns both the closest point and the distance from vehicle to center.
     *          
     *          Special case: If the vehicle is exactly at the circle center (or within
     *          numerical precision), the function returns the point directly behind the
     *          vehicle (based on heading) to provide a consistent reference point.
     * 
     * @param[out] result_NEU_cm Closest point on circle in NEU frame (cm from EKF origin).
     *                           Altitude (z) is set to circle center's altitude.
     * @param[out] dist_cm Distance from vehicle's horizontal position to circle center in centimeters
     * 
     * @note Circle center must be set via init(), init_NEU_cm(), or set_center() before calling.
     * 
     * @note Only horizontal position (N, E) is used for calculation. Result altitude
     *       is always set to circle center altitude.
     * 
     * @note If vehicle is at the circle center, returns edge point directly behind vehicle.
     * 
     * @see get_distance_to_target_cm() for distance to current path target
     */
    void get_closest_point_on_circle_NEU_cm(Vector3f& result_NEU_cm, float& dist_cm) const;

    /**
     * @brief Get horizontal distance to current path target
     * 
     * @details Returns the horizontal distance in centimeters from the vehicle's current
     *          position to the target position on the circular path. This is the position
     *          error being tracked by the AC_PosControl position controller.
     *          
     *          This value indicates how well the vehicle is tracking the circular path.
     *          Larger values indicate greater position error.
     * 
     * @return float Horizontal distance to target in centimeters
     * 
     * @note This is a 2D distance (North-East plane only), altitude error not included.
     * 
     * @see AC_PosControl::get_pos_error_NE_cm() for implementation details
     * @see get_bearing_to_target_rad() for direction to target
     */
    float get_distance_to_target_cm() const { return _pos_control.get_pos_error_NE_cm(); }

    /**
     * @brief Get bearing to current path target
     * 
     * @details Returns the bearing (direction) in radians from the vehicle's current
     *          position to the target position on the circular path. Bearing is measured
     *          clockwise from North (0 = North, π/2 = East, π = South, 3π/2 = West).
     * 
     * @return float Bearing to target in radians (0 to 2π, with 0 = North)
     * 
     * @see AC_PosControl::get_bearing_to_target_rad() for implementation details
     * @see get_distance_to_target_cm() for distance to target
     */
    float get_bearing_to_target_rad() const { return _pos_control.get_bearing_to_target_rad(); }

    /**
     * @brief Check if pilot manual control is enabled
     * 
     * @details Returns true if the CircleOptions::MANUAL_CONTROL flag is set, which
     *          enables pilot stick control of circle radius and rotation rate during
     *          flight. When enabled, pilot inputs can dynamically adjust these parameters.
     *          
     *          When disabled, radius and rate are fixed to parameter or init values.
     * 
     * @return true if pilot manual control enabled, false if radius/rate fixed
     * 
     * @note This is controlled by the CircleOptions::MANUAL_CONTROL bit in _options parameter.
     * 
     * @see CircleOptions enum for available option flags
     */
    bool pilot_control_enabled() const { return (_options.get() & CircleOptions::MANUAL_CONTROL) != 0; }

    /**
     * @brief Check if mount ROI (Region of Interest) is at circle center
     * 
     * @details Returns true if the CircleOptions::ROI_AT_CENTER flag is set, which
     *          configures the mount/gimbal to point at the circle center. This is useful
     *          for panorama photography or videography where the camera should track the
     *          center point while the vehicle circles around it.
     *          
     *          When disabled, mount/gimbal control is independent of circle operation.
     * 
     * @return true if mount ROI should be at circle center, false otherwise
     * 
     * @note This is controlled by the CircleOptions::ROI_AT_CENTER bit in _options parameter.
     * 
     * @note Vehicle code is responsible for actually commanding the mount - this flag
     *       only indicates the desired behavior.
     * 
     * @see CircleOptions enum for available option flags
     */
    bool roi_at_center() const { return (_options.get() & CircleOptions::ROI_AT_CENTER) != 0; }

    /**
     * @brief Provide rangefinder-based terrain offset for terrain following
     * 
     * @details Updates the terrain offset information from rangefinder for use in
     *          terrain-relative altitude control. The terrain offset is the height
     *          of the terrain above the EKF origin.
     *          
     *          This method is typically called by vehicle code that manages the
     *          rangefinder, providing current terrain data to the circle controller.
     *          When terrain-relative altitude is active (_is_terrain_alt = true),
     *          the controller uses this data to maintain altitude above terrain.
     * 
     * @param[in] use True if rangefinder should be used for terrain data
     * @param[in] healthy True if rangefinder data is currently valid and healthy
     * @param[in] terrain_offset_cm Terrain height above EKF origin in centimeters
     *                              (positive = terrain above EKF origin)
     * 
     * @note Terrain offset represents absolute terrain altitude, not distance to ground.
     *       If EKF origin is at sea level and terrain is 100m above sea level,
     *       terrain_offset_cm would be 10000 cm.
     * 
     * @note This is one of two terrain data sources (rangefinder or AP_Terrain database).
     *       The controller automatically selects the appropriate source via get_terrain_source().
     * 
     * @see get_terrain_source() for terrain data source selection
     * @see get_terrain_offset_cm() for retrieving terrain data
     */
    void set_rangefinder_terrain_offset_cm(bool use, bool healthy, float terrain_offset_cm) { _rangefinder_available = use; _rangefinder_healthy = healthy; _rangefinder_terrain_offset_cm = terrain_offset_cm;}

    /**
     * @brief Check for parameter changes and update internal state
     * 
     * @details Monitors the radius parameter (_radius_parm_cm) for changes and updates
     *          internal state if the parameter has been modified. This allows dynamic
     *          adjustment of circle radius via parameter updates without requiring
     *          re-initialization.
     *          
     *          Should be called periodically (typically from update_cms() or vehicle
     *          main loop) to detect parameter changes made via ground station.
     * 
     * @note Only the radius parameter is currently monitored. Rate parameter changes
     *       take effect through direct set_rate_degs() calls or during initialization.
     * 
     * @see set_radius_cm() which is called if parameter change detected
     */
    void check_param_change();

    /**
     * @brief Parameter table for AC_Circle
     * 
     * @details Defines the AP_Param parameter table for circle controller configuration.
     *          Parameters are stored in EEPROM and accessible via ground station.
     *          
     *          Parameters included:
     *          - RADIUS: Circle radius in centimeters (default: AC_CIRCLE_RADIUS_DEFAULT = 1000 cm = 10m)
     *          - RATE: Rotation rate in degrees per second (default: AC_CIRCLE_RATE_DEFAULT = 20 deg/s)
     *          - OPTIONS: CircleOptions bit flags for feature control
     * 
     * @see AP_Param for parameter system details
     * @see CircleOptions for available option flags
     */
    static const struct AP_Param::GroupInfo var_info[];

private:

    /**
     * @brief Calculate angular velocity and acceleration from radius and rate
     * 
     * @details Internal method that recalculates angular velocity maximum and angular
     *          acceleration based on the current circle radius and rotation rate. This
     *          ensures smooth circular motion with appropriate velocity limits.
     *          
     *          Called whenever radius or rate parameters change to update the trajectory
     *          generation. Also initializes yaw and current angular position.
     *          
     *          Angular acceleration is subject to minimum limit of AC_CIRCLE_ANGULAR_ACCEL_MIN
     *          (2.0 deg/s²) to ensure stability.
     * 
     * @param[in] init_velocity True if vehicle is just starting circle mode (initializes
     *                          velocity from zero), false if adjusting parameters during flight
     * 
     * @note This method updates _angular_vel_max_rads and _angular_accel_radss based
     *       on current _radius_cm and _rate_degs.
     * 
     * @note Called by init(), init_NEU_cm(), set_radius_cm(), and set_rate_degs().
     */
    void calc_velocities(bool init_velocity);

    /**
     * @brief Initialize starting angle around circle
     * 
     * @details Sets the initial angular position (_angle_rad) and resets the cumulative
     *          angle tracker (_angle_total_rad). The starting angle can be determined
     *          either from vehicle heading or from vehicle position relative to center.
     *          
     *          Using heading minimizes initial yaw movement and is appropriate when
     *          entering circle mode from forward flight. Using position is appropriate
     *          when the vehicle is already positioned on or near the circle.
     * 
     * @param[in] use_heading True to use vehicle heading to determine starting angle
     *                        (minimizes yaw movement), false to use position from center
     * 
     * @note When use_heading is true, starting angle is based on vehicle's current
     *       heading relative to North (0 = North, increases clockwise).
     * 
     * @note When use_heading is false, starting angle is calculated from the vector
     *       from circle center to vehicle position.
     * 
     * @note Called by init() and init_NEU_cm() during initialization.
     */
    void init_start_angle(bool use_heading);

    /**
     * @enum TerrainSource
     * @brief Source of terrain altitude data for terrain following
     * 
     * @details Indicates which source is providing terrain altitude data for
     *          terrain-relative altitude control. The controller automatically
     *          selects the best available source.
     */
    enum class TerrainSource {
        TERRAIN_UNAVAILABLE,        ///< No terrain data available
        TERRAIN_FROM_RANGEFINDER,   ///< Using rangefinder distance to ground
        TERRAIN_FROM_TERRAINDATABASE, ///< Using AP_Terrain database (pre-loaded terrain tiles)
    };

    /**
     * @brief Determine expected source of terrain data
     * 
     * @details Determines which terrain data source should be used for terrain-relative
     *          altitude control based on availability and health of rangefinder and
     *          AP_Terrain database.
     *          
     *          Priority order:
     *          1. Rangefinder (if available and healthy)
     *          2. AP_Terrain database (if available and location covered)
     *          3. TERRAIN_UNAVAILABLE (if neither source available)
     * 
     * @return TerrainSource Indicates which terrain source is expected to be used
     * 
     * @note This does not guarantee terrain data is currently valid, only indicates
     *       which source should be used if available.
     * 
     * @see set_rangefinder_terrain_offset_cm() for providing rangefinder data
     * @see get_terrain_offset_cm() for retrieving actual terrain offset
     */
    AC_Circle::TerrainSource get_terrain_source() const;

    /**
     * @brief Get terrain altitude offset from EKF origin
     * 
     * @details Retrieves the terrain's altitude in centimeters above the EKF origin at
     *          the vehicle's current position. Positive values mean terrain is above
     *          the EKF origin's altitude, negative values mean terrain is below.
     *          
     *          Automatically selects terrain data source (rangefinder or AP_Terrain
     *          database) based on availability via get_terrain_source().
     *          
     *          Used internally by update_cms() when terrain-relative altitude is active
     *          to maintain altitude above terrain.
     * 
     * @param[out] offset_cm Terrain altitude offset in centimeters above EKF origin
     * 
     * @return true if valid terrain offset retrieved, false if terrain data unavailable
     * 
     * @note Return value of false with _is_terrain_alt = true indicates terrain
     *       failsafe condition and should be handled by calling code.
     * 
     * @see get_terrain_source() for terrain source selection
     * @see set_rangefinder_terrain_offset_cm() for providing terrain data
     */
    bool get_terrain_offset_cm(float& offset_cm);

    /**
     * @brief Internal flags for circle controller state
     * 
     * @details Bit-packed flags structure for tracking internal circle controller state.
     *          Uses bit fields to minimize memory usage.
     */
    struct circle_flags {
        uint8_t panorama    : 1;    ///< True if panorama mode is active (typically with ROI_AT_CENTER)
    } _flags;

    // references to inertial nav and ahrs libraries
    const AP_AHRS_View&         _ahrs;           ///< Reference to AHRS for vehicle attitude and position
    AC_PosControl&              _pos_control;    ///< Reference to position controller for trajectory tracking

    /**
     * @enum CircleOptions
     * @brief Configuration flags for circle controller behavior
     * 
     * @details Bit flags that control various aspects of circle controller behavior.
     *          These flags are stored in the OPTIONS parameter and can be combined
     *          using bitwise OR operations to enable multiple features simultaneously.
     *          
     *          Usage: Set via OPTIONS parameter, accessed via pilot_control_enabled(),
     *          roi_at_center(), and internal checks.
     */
    enum CircleOptions {
        /**
         * @brief Enable pilot manual control of radius and rotation rate
         * 
         * When set, pilot stick inputs can dynamically adjust circle radius and
         * rotation rate during flight. When clear, radius and rate are fixed.
         */
        MANUAL_CONTROL           = 1U << 0,

        /**
         * @brief Yaw faces direction of travel (tangent to circle)
         * 
         * When set, vehicle yaw follows the tangent to the circular path (direction
         * of travel). When clear, yaw points toward circle center (default).
         * Facing direction of travel is useful for forward-facing cameras, while
         * facing center is useful for panorama photography.
         */
        FACE_DIRECTION_OF_TRAVEL = 1U << 1,

        /**
         * @brief Initialize circle center at current vehicle position
         * 
         * When set, init() places the circle center at the vehicle's current position,
         * so the vehicle starts on the circle edge. When clear (default), center is
         * projected one radius ahead of the vehicle based on heading, positioning the
         * vehicle to smoothly enter the circle.
         */
        INIT_AT_CENTER           = 1U << 2,

        /**
         * @brief Set mount/gimbal ROI (Region of Interest) at circle center
         * 
         * When set, indicates that the mount/gimbal should point at the circle center,
         * enabling panorama photography/videography where the camera tracks the center
         * while the vehicle circles around it. Vehicle code is responsible for actually
         * commanding the mount based on this flag.
         */
        ROI_AT_CENTER            = 1U << 3,
    };

    // parameters
    AP_Float    _radius_parm_cm;   ///< Circle radius parameter in cm (loaded from EEPROM, default AC_CIRCLE_RADIUS_DEFAULT)
    AP_Float    _rate_parm_degs;   ///< Rotation rate parameter in deg/sec (loaded from EEPROM, default AC_CIRCLE_RATE_DEFAULT)
    AP_Int16    _options;          ///< CircleOptions bit flags for feature control (manual control, ROI, etc.)

    // internal variables
    Vector3p    _center_neu_cm;    ///< Circle center position in NEU frame, cm from EKF origin
    float       _radius_cm;        ///< Active circle radius in cm (overrides _radius_parm_cm if set)
    float       _rate_degs;        ///< Active rotation rate in deg/sec (positive = CW, negative = CCW)
    float       _yaw_cd;           ///< Desired yaw heading in centidegrees (typically toward center or direction of travel)
    float       _angle_rad;        ///< Current angular position around circle in radians (0 = North of center, increases CW)
    float       _angle_total_rad;  ///< Cumulative angle traveled in radians (not wrapped, continuously increases/decreases)
    float       _angular_vel_rads; ///< Current angular velocity in radians/sec
    float       _angular_vel_max_rads; ///< Maximum angular velocity in radians/sec (calculated from rate and radius)
    float       _angular_accel_radss;  ///< Angular acceleration in radians/sec² (min AC_CIRCLE_ANGULAR_ACCEL_MIN)
    uint32_t    _last_update_ms;   ///< System time in milliseconds of last update_cms() call (for is_active check)
    float       _last_radius_param; ///< Previous value of _radius_parm_cm for detecting parameter changes

    // terrain following variables
    bool        _is_terrain_alt;   ///< True if _center_neu_cm.z is altitude above terrain, false if above EKF origin
    bool        _rangefinder_available; ///< True if rangefinder could be used for terrain data
    bool        _rangefinder_healthy;   ///< True if rangefinder data is currently valid and healthy
    float       _rangefinder_terrain_offset_cm; ///< Terrain altitude above EKF origin in cm (from rangefinder)
};
