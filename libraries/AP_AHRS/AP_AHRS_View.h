/**
 * @file AP_AHRS_View.h
 * @brief AHRS View class for creating rotated secondary views of vehicle attitude
 * 
 * @details This file defines the AP_AHRS_View class which provides a rotated view
 *          of the vehicle attitude. The View class is used to create an alternative
 *          representation of the vehicle's orientation by applying a rotation
 *          transformation to the primary AHRS attitude estimate.
 *          
 *          Primary use cases include:
 *          - Creating forward-right-down body frame views for cameras or sensors
 *          - Applying pitch trim for fixed-wing aircraft
 *          - Providing attitude estimates in alternative reference frames
 *          
 *          The View maintains its own rotation matrix and trig values for efficiency,
 *          but delegates position, velocity, and wind estimation to the parent AHRS.
 *          
 *          Coordinate frame conventions:
 *          - NED frame: North-East-Down earth-fixed reference frame
 *          - Body frame: Forward-Right-Down vehicle-fixed reference frame
 *          
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_AHRS.h"

// fwd declarations to avoid include errors
class AC_AttitudeControl;
class AC_PosControl;

/**
 * @class AP_AHRS_View
 * @brief Provides a rotated secondary view of the vehicle attitude
 * 
 * @details The AP_AHRS_View class creates an alternative reference frame for attitude
 *          by applying a rotation transformation to the primary AHRS attitude estimate.
 *          This is useful for:
 *          - Mounting sensors or cameras at non-standard orientations
 *          - Applying pitch trim corrections for fixed-wing aircraft
 *          - Creating custom body frame orientations for specific control algorithms
 *          
 *          The View maintains its own rotation matrix (rot_body_to_ned), Euler angles
 *          (roll, pitch, yaw), and cached trigonometric values for efficiency. Position,
 *          velocity, and wind estimates are passed through directly from the parent AHRS.
 *          
 *          Implementation notes:
 *          - The rotation is applied on top of the AHRS attitude estimate
 *          - Pitch trim is applied as an additional rotation around the body Y axis
 *          - All angular values use radians internally, degrees for user-facing APIs
 *          - Coordinate frames follow ArduPilot conventions (NED for earth frame)
 */
class AP_AHRS_View
{
public:
    /**
     * @brief Construct a new AP_AHRS_View object with specified rotation
     * 
     * @param[in] ahrs          Reference to parent AHRS object providing base attitude
     * @param[in] rotation      Rotation enum defining the transformation from AHRS frame to view frame
     * @param[in] pitch_trim_deg Additional pitch trim to apply in degrees (default: 0)
     *                          Positive values trim nose up, typically used for fixed-wing
     *                          aircraft to compensate for sensor mounting angle
     */
    AP_AHRS_View(AP_AHRS &ahrs, enum Rotation rotation, float pitch_trim_deg=0);

    /**
     * @brief Update the view state from parent AHRS
     * 
     * @details Retrieves the current attitude from the parent AHRS, applies the view
     *          rotation and pitch trim, then updates the view's rotation matrix, Euler
     *          angles, and cached trigonometric values. This should be called at the
     *          main loop rate to keep the view synchronized with AHRS updates.
     *          
     * @note This is typically called automatically by the vehicle code each loop iteration
     */
    void update();

    // empty virtual destructor
    virtual ~AP_AHRS_View() {}

    /**
     * @brief Get smoothed and corrected gyro vector in body frame
     * 
     * @details Returns the gyro rates transformed into the view's body frame reference.
     *          The gyro data is sourced from the parent AHRS and transformed by the
     *          view rotation.
     * 
     * @return const Vector3f& Gyro rates in body frame [rad/s]
     *         - X axis: Roll rate (positive = right wing down)
     *         - Y axis: Pitch rate (positive = nose up)
     *         - Z axis: Yaw rate (positive = nose right)
     */
    const Vector3f &get_gyro(void) const {
        return gyro;
    }

    /**
     * @brief Get latest gyro vector using most recent INS data
     * 
     * @details Returns gyro rates using the latest INS data which may not yet have been
     *          consumed by the EKF. This provides lower-latency gyro information for
     *          high-rate control loops.
     * 
     * @return Vector3f Latest gyro rates in body frame [rad/s]
     *         - X axis: Roll rate (positive = right wing down)
     *         - Y axis: Pitch rate (positive = nose up)
     *         - Z axis: Yaw rate (positive = nose right)
     */
    Vector3f get_gyro_latest(void) const;

    /**
     * @brief Get DCM rotation matrix from body frame to NED frame
     * 
     * @details Returns the Direction Cosine Matrix (DCM) representing the rotation
     *          from the vehicle body frame to the NED (North-East-Down) earth frame.
     *          This matrix includes the view rotation and pitch trim.
     * 
     * @return const Matrix3f& 3x3 rotation matrix for body-to-NED transformation
     *         - Use to transform body frame vectors to NED frame
     *         - Matrix is orthonormal (transpose equals inverse)
     */
    const Matrix3f &get_rotation_body_to_ned(void) const {
        return rot_body_to_ned;
    }

    /**
     * @brief Get quaternion representation of body to NED rotation
     * 
     * @details Converts the view's DCM rotation matrix to quaternion form and stores
     *          it in the provided quaternion object. Quaternions are useful for
     *          interpolation and avoiding gimbal lock.
     * 
     * @param[out] quat Quaternion representing body-to-NED rotation in NED frame
     *                  convention (w, x, y, z order)
     */
    void get_quat_body_to_ned(Quaternion &quat) const {
        quat.from_rotation_matrix(rot_body_to_ned);
    }

    /**
     * @brief Set pitch trim angle
     * 
     * @details Applies an additional pitch rotation to the view, typically used for
     *          fixed-wing aircraft to compensate for sensor mounting angles or to
     *          adjust for aircraft trim conditions. Positive values pitch nose up.
     * 
     * @param[in] trim_deg Pitch trim angle in degrees (positive = nose up)
     * 
     * @note Changes take effect on the next update() call
     */
    void set_pitch_trim(float trim_deg);

    /**
     * @brief Get roll angle in radians
     * @return float Roll angle in radians (positive = right wing down, range: -π to π)
     */
    float get_roll_rad() const { return roll; }
    
    /**
     * @brief Get pitch angle in radians
     * @return float Pitch angle in radians (positive = nose up, range: -π/2 to π/2)
     */
    float get_pitch_rad() const { return pitch; }
    
    /**
     * @brief Get yaw angle in radians
     * @return float Yaw angle in radians (positive = clockwise from North, range: -π to π)
     */
    float get_yaw_rad() const { return yaw; }

    /**
     * @brief Get cosine of roll angle
     * @details Cached trigonometric value for efficiency in control loops
     * @return float cos(roll) (range: -1.0 to 1.0)
     */
    float cos_roll() const {
        return trig.cos_roll;
    }
    
    /**
     * @brief Get cosine of pitch angle
     * @details Cached trigonometric value for efficiency in control loops
     * @return float cos(pitch) (range: -1.0 to 1.0)
     */
    float cos_pitch() const {
        return trig.cos_pitch;
    }
    
    /**
     * @brief Get cosine of yaw angle
     * @details Cached trigonometric value for efficiency in control loops
     * @return float cos(yaw) (range: -1.0 to 1.0)
     */
    float cos_yaw() const {
        return trig.cos_yaw;
    }
    
    /**
     * @brief Get sine of roll angle
     * @details Cached trigonometric value for efficiency in control loops
     * @return float sin(roll) (range: -1.0 to 1.0)
     */
    float sin_roll() const {
        return trig.sin_roll;
    }
    
    /**
     * @brief Get sine of pitch angle
     * @details Cached trigonometric value for efficiency in control loops
     * @return float sin(pitch) (range: -1.0 to 1.0)
     */
    float sin_pitch() const {
        return trig.sin_pitch;
    }
    
    /**
     * @brief Get sine of yaw angle
     * @details Cached trigonometric value for efficiency in control loops
     * @return float sin(yaw) (range: -1.0 to 1.0)
     */
    float sin_yaw() const {
        return trig.sin_yaw;
    }


    /*
      Pass-through wrappers around parent AHRS functions. These delegate directly to
      the parent AHRS object since position, velocity, and environmental estimates
      are not affected by the view rotation. See AP_AHRS.h for detailed descriptions.
     */
    
    /**
     * @brief Get current vehicle location
     * @param[out] loc Current GPS location with altitude
     * @return true if location is available and valid, false otherwise
     */
    bool get_location(Location &loc) const WARN_IF_UNUSED {
        return ahrs.get_location(loc);
    }

    /**
     * @brief Get wind velocity estimate
     * @param[out] wind Wind velocity vector in NED frame [m/s]
     * @return true if wind estimate is available, false otherwise
     */
    bool wind_estimate(Vector3f &wind) {
        return ahrs.wind_estimate(wind);
    }

    /**
     * @brief Get equivalent airspeed (EAS) estimate
     * @param[out] airspeed_ret Equivalent airspeed in m/s
     * @return true if airspeed estimate is available, false otherwise
     */
    bool airspeed_estimate(float &airspeed_ret) const WARN_IF_UNUSED {
        return ahrs.airspeed_estimate(airspeed_ret);
    }

    /**
     * @brief Get true airspeed (TAS) estimate
     * @param[out] airspeed_ret True airspeed in m/s
     * @return true if airspeed estimate is available, false otherwise
     */
    bool airspeed_estimate_true(float &airspeed_ret) const WARN_IF_UNUSED {
        return ahrs.airspeed_estimate_true(airspeed_ret);
    }

    /**
     * @brief Get equivalent airspeed to true airspeed ratio
     * @return float EAS2TAS conversion ratio (dimensionless)
     */
    float get_EAS2TAS(void) const {
        return ahrs.get_EAS2TAS();
    }

    /**
     * @brief Get 2D groundspeed vector
     * @return Vector2f Groundspeed in NED frame [m/s] (North and East components)
     */
    Vector2f groundspeed_vector(void) {
        return ahrs.groundspeed_vector();
    }

    /**
     * @brief Get 3D velocity in NED frame
     * @param[out] vec Velocity vector in NED frame [m/s]
     * @return true if velocity estimate is available, false otherwise
     */
    bool get_velocity_NED(Vector3f &vec) const WARN_IF_UNUSED {
        return ahrs.get_velocity_NED(vec);
    }

    /**
     * @brief Get position relative to home in NED frame
     * @param[out] vec Position offset from home in NED frame [meters]
     * @return true if position estimate is available, false otherwise
     */
    bool get_relative_position_NED_home(Vector3f &vec) const WARN_IF_UNUSED {
        return ahrs.get_relative_position_NED_home(vec);
    }

    /**
     * @brief Get position relative to EKF origin in NED frame
     * @param[out] vec Position offset from EKF origin in NED frame [meters]
     * @return true if position estimate is available, false otherwise
     */
    bool get_relative_position_NED_origin_float(Vector3f &vec) const WARN_IF_UNUSED {
        return ahrs.get_relative_position_NED_origin_float(vec);
    }

    /**
     * @brief Get 2D position relative to home (North-East only)
     * @param[out] vecNE Position offset from home, North and East components [meters]
     * @return true if position estimate is available, false otherwise
     */
    bool get_relative_position_NE_home(Vector2f &vecNE) const WARN_IF_UNUSED {
        return ahrs.get_relative_position_NE_home(vecNE);
    }

    /**
     * @brief Get 2D position relative to EKF origin (North-East only)
     * @param[out] vecNE Position offset from EKF origin, North and East components [meters]
     * @return true if position estimate is available, false otherwise
     */
    bool get_relative_position_NE_origin_float(Vector2f &vecNE) const WARN_IF_UNUSED {
        return ahrs.get_relative_position_NE_origin_float(vecNE);
    }

    /**
     * @brief Get Down position relative to home
     * @param[out] posD Down position offset from home [meters] (positive = below home)
     */
    void get_relative_position_D_home(float &posD) const {
        ahrs.get_relative_position_D_home(posD);
    }

    /**
     * @brief Get Down position relative to EKF origin
     * @param[out] posD Down position offset from EKF origin [meters] (positive = below origin)
     * @return true if position estimate is available, false otherwise
     */
    bool get_relative_position_D_origin_float(float &posD) const WARN_IF_UNUSED {
        return ahrs.get_relative_position_D_origin_float(posD);
    }

    /**
     * @brief Get scalar groundspeed magnitude
     * @return float Groundspeed in m/s (always positive)
     */
    float groundspeed(void) {
        return ahrs.groundspeed();
    }

    /**
     * @brief Get acceleration in earth frame
     * @return const Vector3f& Acceleration vector in NED frame [m/s²]
     */
    const Vector3f &get_accel_ef(void) const {
        return ahrs.get_accel_ef();
    }

    /**
     * @brief Get timestamp of last North-East position reset
     * @param[out] pos Position change from the reset [meters] in NED frame
     * @return uint32_t Timestamp of last reset in milliseconds
     */
    uint32_t getLastPosNorthEastReset(Vector2f &pos) WARN_IF_UNUSED {
        return ahrs.getLastPosNorthEastReset(pos);
    }

    /**
     * @brief Get timestamp of last Down position reset
     * @param[out] posDelta Position change from the reset [meters] in Down direction
     * @return uint32_t Timestamp of last reset in milliseconds
     */
    uint32_t getLastPosDownReset(float &posDelta) WARN_IF_UNUSED {
        return ahrs.getLastPosDownReset(posDelta);
    }

    /**
     * @brief Rotate 2D vector from earth frame to body frame
     * 
     * @details Performs 2D rotation transformation from NED earth frame to body frame.
     *          Uses only the yaw angle, projecting into the horizontal plane.
     * 
     * @param[in] ef_vector Input vector in earth frame (North-East components)
     * @return Vector2f Output vector in body frame where:
     *         - x component: Forward direction (aligned with nose)
     *         - y component: Right direction (aligned with right wing)
     */
    Vector2f earth_to_body2D(const Vector2f &ef_vector) const;

    /**
     * @brief Rotate 2D vector from body frame to earth frame
     * 
     * @details Performs 2D rotation transformation from body frame to NED earth frame.
     *          Uses only the yaw angle, projecting into the horizontal plane.
     * 
     * @param[in] bf Input vector in body frame where:
     *                x component: Forward direction (aligned with nose)
     *                y component: Right direction (aligned with right wing)
     * @return Vector2f Output vector in earth frame (North-East components)
     */
    Vector2f body_to_earth2D(const Vector2f &bf) const;

    /**
     * @brief Get average roll/pitch error estimate
     * 
     * @details Returns the average magnitude of attitude error in the roll and pitch
     *          axes since the last call. Used for monitoring AHRS health and convergence.
     * 
     * @return float Average roll/pitch error magnitude in radians
     */
    float get_error_rp(void) const {
        return ahrs.get_error_rp();
    }

    /**
     * @brief Get average yaw error estimate
     * 
     * @details Returns the average magnitude of attitude error in the yaw axis since
     *          the last call. Used for monitoring AHRS health and convergence.
     * 
     * @return float Average yaw error magnitude in radians
     */
    float get_error_yaw(void) const {
        return ahrs.get_error_yaw();
    }

    /**
     * @brief Write attitude view data to dataflash log
     * 
     * @details Logs the current view attitude along with target attitude values for
     *          analysis and debugging. Used by flight controllers to record attitude
     *          tracking performance.
     * 
     * @param[in] targets Target attitude vector (roll, pitch, yaw) in radians
     */
    void Write_AttitudeView(const Vector3f &targets) const;    

    /**
     * @brief Roll angle in radians
     * @details Current roll angle in radians (positive = right wing down, range: -π to π)
     */
    float roll;
    
    /**
     * @brief Pitch angle in radians
     * @details Current pitch angle in radians (positive = nose up, range: -π/2 to π/2)
     */
    float pitch;
    
    /**
     * @brief Yaw angle in radians
     * @details Current yaw angle in radians (positive = clockwise from North, range: -π to π)
     */
    float yaw;
    
    /**
     * @brief Roll angle in centidegrees for sensor output
     * @details Roll angle scaled to centidegrees (1/100th of a degree) for compatibility
     *          with legacy sensor interfaces and ground control stations
     */
    int32_t roll_sensor;
    
    /**
     * @brief Pitch angle in centidegrees for sensor output
     * @details Pitch angle scaled to centidegrees (1/100th of a degree) for compatibility
     *          with legacy sensor interfaces and ground control stations
     */
    int32_t pitch_sensor;
    
    /**
     * @brief Yaw angle in centidegrees for sensor output
     * @details Yaw angle scaled to centidegrees (1/100th of a degree) for compatibility
     *          with legacy sensor interfaces and ground control stations
     */
    int32_t yaw_sensor;


    /**
     * @brief Get the rotation enum for this view
     * 
     * @details Returns the base rotation transformation applied to create this view.
     *          Note that the actual rotation may differ due to pitch trim applied on
     *          top of this base rotation.
     * 
     * @return enum Rotation The rotation enum defining the view transformation
     * 
     * @note The effective rotation includes both this rotation and _pitch_trim_deg
     */
    enum Rotation get_rotation(void) const {
        return rotation;
    }

    /**
     * @brief Get the current pitch trim value
     * 
     * @return float Pitch trim in degrees (positive = nose up)
     */
    float get_pitch_trim() const { return _pitch_trim_deg; }

    /**
     * @brief Rotate a vector from AHRS reference frame to View reference frame
     * 
     * @details Applies the view's rotation transformation to a 3D vector, converting
     *          it from the parent AHRS reference frame to this view's reference frame.
     *          This includes both the base rotation and any pitch trim.
     * 
     * @param[in,out] vec Vector to rotate (modified in place)
     */
    void rotate(Vector3f &vec) const;

private:
    /// Base rotation transformation defining this view relative to AHRS frame
    const enum Rotation rotation;
    
    /// Reference to parent AHRS object providing base attitude and state estimates
    AP_AHRS &ahrs;

    /// Body frame rotation matrix for this view (transforms from AHRS frame to view frame)
    Matrix3f rot_view;
    
    /// Transpose of rot_view matrix (transforms from view frame to AHRS frame)
    Matrix3f rot_view_T;
    
    /// Current rotation matrix from body frame to NED earth frame (includes view rotation and pitch trim)
    Matrix3f rot_body_to_ned;
    
    /// Gyro rates transformed into view's body frame [rad/s]
    Vector3f gyro;

    /// Cached trigonometric values for efficient access in control loops
    struct {
        float cos_roll;   ///< Cosine of roll angle
        float cos_pitch;  ///< Cosine of pitch angle
        float cos_yaw;    ///< Cosine of yaw angle
        float sin_roll;   ///< Sine of roll angle
        float sin_pitch;  ///< Sine of pitch angle
        float sin_yaw;    ///< Sine of yaw angle
    } trig;

    /// Y-axis rotation angle used in view transformation calculations [radians]
    float y_angle;
    
    /// Additional pitch trim applied to view in degrees (positive = nose up)
    float _pitch_trim_deg;
};
