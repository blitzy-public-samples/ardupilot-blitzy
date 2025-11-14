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

// Copyright 2012 Andrew Tridgell, all rights reserved.
// Refactored by Jonathan Challinger

/**
 * @file quaternion.h
 * @brief Quaternion algebra for 3D rotations and attitude representation
 * 
 * @details This file provides a template-based quaternion implementation for representing
 *          3D rotations in ArduPilot. Quaternions avoid gimbal lock singularities inherent
 *          in Euler angle representations and provide efficient rotation composition and
 *          interpolation. Used extensively in attitude estimation (AP_AHRS) and control systems.
 *          
 *          Mathematical Foundation:
 *          A quaternion is represented as q = q1 + q2*i + q3*j + q4*k where:
 *          - q1 is the scalar (real) part
 *          - q2, q3, q4 are the vector (imaginary) parts
 *          
 *          Unit quaternions (|q| = 1) represent pure rotations without scaling.
 *          
 *          Coordinate Frames:
 *          - Earth frame: NED (North-East-Down) fixed reference frame
 *          - Body frame: Vehicle-fixed frame (forward-right-down)
 *          
 * @note Quaternion convention: q1 = scalar/real part, q2/q3/q4 = vector/imaginary parts
 * @note Rotation direction follows right-hand rule
 * @note Explicit float/double instantiations provided in quaternion.cpp
 * 
 * @see Matrix3 for alternative rotation representation using direction cosine matrices
 * @see AP_AHRS for usage in attitude estimation and sensor fusion
 * 
 * Source: libraries/AP_Math/quaternion.h
 */

#pragma once

#include "definitions.h"
#include "matrix3.h"
#include <cmath>
#if MATH_CHECK_INDEXES
#include <assert.h>
#endif
#include <math.h>

/**
 * @class QuaternionT
 * @brief Quaternion template class for 3D rotation representation without gimbal lock
 * 
 * @tparam T Numeric type (float or double) for quaternion components
 * 
 * @details QuaternionT provides a complete quaternion algebra implementation for representing
 *          and manipulating 3D rotations. Key advantages over Euler angles:
 *          - No gimbal lock singularities at ±90° pitch
 *          - Efficient composition of rotations via quaternion multiplication
 *          - Smooth interpolation (SLERP) for trajectory generation
 *          - Compact representation (4 elements vs 9 for rotation matrix)
 *          
 *          Internal Representation:
 *          The quaternion q = q1 + q2*i + q3*j + q4*k represents rotation where:
 *          - q1 (scalar part): cos(θ/2)
 *          - [q2, q3, q4] (vector part): sin(θ/2) * [axis_x, axis_y, axis_z]
 *          - θ is rotation angle, [axis_x, axis_y, axis_z] is unit rotation axis
 *          
 *          Unit Quaternion Constraint:
 *          For pure rotations: q1² + q2² + q3² + q4² = 1
 *          normalize() method enforces this constraint
 *          
 *          Coordinate Frame Conventions:
 *          - Earth frame: NED (North-East-Down) - fixed inertial reference
 *          - Body frame: FRD (Forward-Right-Down) - vehicle-fixed axes
 *          
 *          Usage Pattern:
 *          @code
 *          Quaternion q;
 *          q.from_euler(roll, pitch, yaw);  // Convert Euler angles to quaternion
 *          Vector3f body_vec(1.0f, 0.0f, 0.0f);
 *          q.earth_to_body(body_vec);  // Rotate vector from earth to body frame
 *          @endcode
 *          
 * @note Quaternion multiplication is non-commutative: q1*q2 ≠ q2*q1
 * @note Rotation direction: Right-hand rule (thumb along axis, fingers curl in rotation direction)
 * @warning Numerical precision: Near ±90° pitch, small numerical errors can accumulate
 *          during repeated operations. Periodically normalize to maintain unit constraint.
 * @warning Gimbal lock avoidance: While quaternions avoid gimbal lock, conversion to/from
 *          Euler angles still has singularities at ±90° pitch
 * 
 * @see Matrix3 for alternative DCM (Direction Cosine Matrix) rotation representation
 * @see Vector3 for vector operations in 3D space
 * @see AP_AHRS for attitude estimation using quaternions
 */
template <typename T>
class QuaternionT {
public:
    /**
     * @brief Quaternion components: q1 (scalar), q2/q3/q4 (vector)
     * 
     * @details For unit quaternions representing rotation by angle θ around axis [x,y,z]:
     *          - q1 = cos(θ/2) (scalar/real part)
     *          - q2 = sin(θ/2) * x (i component)
     *          - q3 = sin(θ/2) * y (j component)
     *          - q4 = sin(θ/2) * z (k component)
     * 
     * @note Identity rotation: q1=1, q2=q3=q4=0 (no rotation)
     * @note Unit constraint: q1² + q2² + q3² + q4² = 1 for pure rotations
     */
    T        q1, q2, q3, q4;

    /**
     * @brief Default constructor creates identity quaternion (no rotation)
     * 
     * @details Initializes quaternion to identity rotation equivalent to:
     *          - Roll = 0 radians
     *          - Pitch = 0 radians
     *          - Yaw = 0 radians
     *          
     *          Result: q1=1, q2=0, q3=0, q4=0 (no rotation applied)
     * 
     * @note This is the multiplicative identity: q * identity = q
     */
    QuaternionT()
    {
        q1 = 1;
        q2 = q3 = q4 = 0;
    }

    /**
     * @brief Constructor setting quaternion components explicitly
     * 
     * @param[in] _q1 Scalar (real) component
     * @param[in] _q2 First vector (imaginary) component (i)
     * @param[in] _q3 Second vector (imaginary) component (j)
     * @param[in] _q4 Third vector (imaginary) component (k)
     * 
     * @note No normalization performed - caller responsible for unit constraint if needed
     * @note Useful for creating quaternion from known components (e.g., from EKF state)
     */
    QuaternionT(const T _q1, const T _q2, const T _q3, const T _q4) :
        q1(_q1), q2(_q2), q3(_q3), q4(_q4)
    {
    }

    /**
     * @brief Constructor from array of 4 components
     * 
     * @param[in] _q Array of 4 elements: [q1, q2, q3, q4]
     * 
     * @note Array indexing: _q[0]=q1 (scalar), _q[1]=q2, _q[2]=q3, _q[3]=q4 (vector parts)
     * @note No normalization performed - caller responsible for unit constraint if needed
     */
    QuaternionT(const T _q[4]) :
        q1(_q[0]), q2(_q[1]), q3(_q[2]), q4(_q[3])
    {
    }

    /**
     * @brief Check if any quaternion component is NaN (Not a Number)
     * 
     * @return true if any component (q1, q2, q3, q4) is NaN, false if all valid
     * 
     * @note Used for validity checking after numerical operations
     * @note NaN can result from 0/0, infinity operations, or uninitialized memory
     * @warning If this returns true, quaternion is invalid and should not be used for rotations
     */
    bool        is_nan(void) const WARN_IF_UNUSED
    {
        return isnan(q1) || isnan(q2) || isnan(q3) || isnan(q4);
    }

    /**
     * @brief Convert quaternion to 3x3 rotation matrix (Direction Cosine Matrix)
     * 
     * @param[out] m Output Matrix3f rotation matrix populated from this quaternion
     * 
     * @details Converts unit quaternion to equivalent 3x3 orthogonal rotation matrix (DCM).
     *          The resulting matrix can rotate vectors: v_rotated = m * v_original
     *          
     *          DCM provides alternative rotation representation useful for:
     *          - Legacy code expecting rotation matrices
     *          - Certain geometric calculations
     *          - Direct axis projections
     * 
     * @note Quaternion should be normalized before conversion for accurate results
     * @note Resulting matrix is orthogonal: m * m^T = I (identity)
     * 
     * @see from_rotation_matrix() for inverse conversion
     * @see Matrix3 for rotation matrix operations
     */
    void        rotation_matrix(Matrix3f &m) const;

    /**
     * @brief Convert quaternion to 3x3 rotation matrix (double precision)
     * 
     * @param[out] m Output Matrix3d rotation matrix populated from this quaternion
     * 
     * @details Double-precision version for high-accuracy applications
     * 
     * @see rotation_matrix(Matrix3f &m) for detailed description
     */
    void        rotation_matrix(Matrix3d &m) const;

    /**
     * @brief Construct quaternion from 3x3 rotation matrix (DCM)
     * 
     * @param[in] m Input rotation matrix to convert to quaternion representation
     * 
     * @details Converts Direction Cosine Matrix (DCM) to equivalent unit quaternion.
     *          Uses numerically stable algorithm that avoids division by small numbers
     *          by selecting largest quaternion component for initial calculation.
     *          
     *          Common use cases:
     *          - Initializing attitude from DCM-based algorithms
     *          - Converting between rotation representations
     *          - Importing rotations from external systems
     * 
     * @note Input matrix should be orthogonal for accurate conversion
     * @note Resulting quaternion is automatically normalized
     * 
     * @see rotation_matrix() for inverse conversion
     */
    void		from_rotation_matrix(const Matrix3<T> &m);

    /**
     * @brief Create quaternion from predefined rotation enum (board orientation)
     * 
     * @param[in] rotation Rotation enum value representing fixed axis rotations
     * 
     * @details Converts standard rotation enumeration (e.g., ROTATION_ROLL_180,
     *          ROTATION_PITCH_90) to equivalent quaternion. Used for board orientation
     *          configuration and sensor alignment.
     * 
     * @note Replaces current quaternion value
     * @see rotate(enum Rotation) to apply rotation to existing quaternion
     */
    void        from_rotation(enum Rotation rotation);

    /**
     * @brief Rotate this quaternion by a predefined rotation enum
     * 
     * @param[in] rotation Rotation enum to apply to current quaternion
     * 
     * @details Applies additional rotation to this quaternion: q_new = q_rotation * q_current
     *          Used for applying board orientation corrections and sensor alignments.
     * 
     * @note Modifies this quaternion in-place
     * @see from_rotation(enum Rotation) to set quaternion directly
     */
    void        rotate(enum Rotation rotation);

    /**
     * @brief Rotate vector from earth frame (NED) to body frame (FRD)
     * 
     * @param[in,out] v Vector3 in earth frame (input), rotated to body frame (output)
     * 
     * @details Applies rotation transformation to convert vector from earth-fixed frame
     *          to vehicle body-fixed frame. Mathematical operation: v_body = q * v_earth * q*
     *          where q* is quaternion conjugate.
     *          
     *          Coordinate Frames:
     *          - Earth frame (input): NED (North-East-Down) inertial reference
     *          - Body frame (output): FRD (Forward-Right-Down) vehicle-fixed axes
     *          
     *          Common applications:
     *          - Transforming target waypoint from earth to body frame for control
     *          - Converting wind velocity to body frame
     *          - Computing relative positions in vehicle frame
     * 
     * @note Vector is modified in-place (input/output parameter)
     * @note Quaternion should be unit length for accurate rotation
     * @warning Ensure correct frame convention - ArduPilot uses NED earth frame
     * 
     * @see inverse() to rotate body frame to earth frame
     */
    void        earth_to_body(Vector3<T> &v) const;

    /**
     * @brief Create quaternion from Euler angles using 321 rotation sequence
     * 
     * @param[in] roll  Roll angle in radians (rotation about forward/x axis)
     * @param[in] pitch Pitch angle in radians (rotation about right/y axis)  
     * @param[in] yaw   Yaw angle in radians (rotation about down/z axis)
     * 
     * @details Converts Euler angles to quaternion using 321 rotation ordering (standard aerospace):
     *          1. First rotate by yaw about z-axis (heading)
     *          2. Then rotate by pitch about y-axis (elevation)
     *          3. Finally rotate by roll about x-axis (bank)
     *          
     *          This is the standard rotation sequence used throughout ArduPilot for attitude
     *          representation. Also called Tait-Bryan angles or "yaw-pitch-roll" ordering.
     *          
     *          Angle Conventions (NED frame):
     *          - Roll: positive = right wing down
     *          - Pitch: positive = nose up  
     *          - Yaw: positive = nose right (clockwise from above)
     * 
     * @note All angles in radians (not degrees or centidegrees)
     * @note Replaces current quaternion value
     * @warning Singularity at pitch = ±90° (gimbal lock) - use quaternions directly when possible
     * 
     * @see to_euler() for inverse conversion
     * @see from_vector312() for alternative 312 rotation sequence
     */
    void        from_euler(T roll, T pitch, T yaw);

    /**
     * @brief Create quaternion from Euler angles in Vector3
     * 
     * @param[in] v Vector3 containing [roll, pitch, yaw] in radians
     * 
     * @note Convenience wrapper for from_euler(roll, pitch, yaw)
     * @see from_euler(T roll, T pitch, T yaw) for detailed documentation
     */
    void        from_euler(const Vector3<T> &v);

    /**
     * @brief Create quaternion from Euler angles using 312 rotation sequence
     * 
     * @param[in] roll  Roll angle in radians
     * @param[in] pitch Pitch angle in radians
     * @param[in] yaw   Yaw angle in radians
     * 
     * @details Converts Euler angles using 312 rotation ordering (alternative sequence):
     *          1. First rotate by yaw about z-axis
     *          2. Then rotate by roll about x-axis  
     *          3. Finally rotate by pitch about y-axis
     *          
     *          This sequence is used in specific applications where the rotation order
     *          needs to differ from standard 321 aerospace convention.
     * 
     * @note All angles in radians
     * @note Less commonly used than standard 321 sequence
     * 
     * @see from_euler() for standard 321 rotation sequence
     * @see to_vector312() for inverse conversion
     */
    void        from_vector312(T roll, T pitch, T yaw);

    /**
     * @brief Convert quaternion to axis-angle representation (rotation vector)
     * 
     * @param[out] v Output Vector3 where direction is rotation axis, magnitude is angle in radians
     * 
     * @details Extracts axis-angle representation from unit quaternion:
     *          - Vector direction: axis of rotation (unit vector)
     *          - Vector length: rotation angle in radians
     *          
     *          Axis-angle form is intuitive for visualization and useful for:
     *          - Computing angular differences between orientations
     *          - Interpolating rotations
     *          - Representing small rotations compactly
     *          
     *          For quaternion q = cos(θ/2) + sin(θ/2)*[x,y,z]:
     *          - Rotation angle θ = 2*acos(q1)
     *          - Rotation axis = [q2, q3, q4] / sin(θ/2)
     * 
     * @note Output vector magnitude is in radians
     * @note For identity quaternion (no rotation), returns zero vector
     * @note Quaternion should be normalized for accurate results
     * 
     * @see from_axis_angle() for inverse conversion
     */
    void        to_axis_angle(Vector3<T> &v) const;

    /**
     * @brief Create quaternion from rotation vector (axis-angle)
     * 
     * @param[in] v Rotation vector where direction is axis, magnitude is angle in radians
     * 
     * @details Constructs unit quaternion from axis-angle representation where:
     *          - Vector direction defines rotation axis (normalized internally)
     *          - Vector magnitude defines rotation angle in radians
     *          
     *          Conversion formula:
     *          - q1 = cos(|v|/2)
     *          - [q2,q3,q4] = sin(|v|/2) * normalize(v)
     * 
     * @note Input vector magnitude is rotation angle in radians
     * @note Axis normalization handled automatically
     * 
     * @see to_axis_angle() for inverse conversion
     * @see from_axis_angle_fast() for small angle approximation
     */
    void        from_axis_angle(Vector3<T> v);

    /**
     * @brief Create quaternion from axis and angle (separate parameters)
     * 
     * @param[in] axis  Unit vector defining rotation axis (must be normalized)
     * @param[in] theta Rotation angle in radians
     * 
     * @details Constructs unit quaternion from explicit axis-angle representation:
     *          - q1 = cos(theta/2)
     *          - [q2,q3,q4] = sin(theta/2) * axis
     * 
     * @note axis must be unit length (normalized) - NOT verified internally
     * @note theta in radians
     * 
     * @warning Caller must ensure axis is normalized for correct results
     * 
     * @see from_axis_angle(Vector3<T>) for combined axis-angle vector
     */
    void        from_axis_angle(const Vector3<T> &axis, T theta);

    /**
     * @brief Rotate quaternion by rotation vector (axis-angle)
     * 
     * @param[in] v Rotation vector to apply (direction=axis, magnitude=angle in radians)
     * 
     * @details Applies rotation represented by vector v to this quaternion:
     *          q_new = q_rotation * q_current
     *          
     *          Used for applying incremental rotations, such as:
     *          - Integrating gyro measurements
     *          - Applying control corrections
     *          - Composing multiple rotations
     * 
     * @note Modifies quaternion in-place
     * @note For large rotations, use full trigonometric calculation
     * 
     * @see rotate_fast() for small angle approximation (better performance)
     */
    void        rotate(const Vector3<T> &v);

    /**
     * @brief Create quaternion from rotation vector using small angle approximation
     * 
     * @param[in] v Rotation vector (direction=axis, magnitude=angle in radians)
     * 
     * @details Fast approximation for small angle rotations using Taylor series:
     *          - cos(θ/2) ≈ 1
     *          - sin(θ/2) ≈ θ/2
     *          
     *          Provides significant performance improvement for incremental updates
     *          at high rates (e.g., 400Hz attitude control loop).
     * 
     * @note ONLY valid for small angles: |v| < 0.17 radians (≈10 degrees)
     * @warning Accuracy degrades rapidly beyond 10° - use from_axis_angle() for large rotations
     * @note Commonly used in gyro integration where time steps are small
     * 
     * @see from_axis_angle() for full-precision conversion
     */
    void        from_axis_angle_fast(Vector3<T> v);

    /**
     * @brief Create quaternion from axis and angle using small angle approximation
     * 
     * @param[in] axis  Unit vector rotation axis (must be normalized)
     * @param[in] theta Rotation angle in radians
     * 
     * @details Fast approximation: sin(theta/2) ≈ theta/2, cos(theta/2) ≈ 1
     * 
     * @note axis must be unit length (not verified)
     * @note theta must be < 0.17 radians (≈10 degrees)
     * 
     * @warning Do not use for angles > 10° - significant error accumulates
     * @warning Caller must ensure axis is normalized
     * 
     * @see from_axis_angle() for full-precision version
     */
    void        from_axis_angle_fast(const Vector3<T> &axis, T theta);

    /**
     * @brief Create quaternion by integrating angular velocity over time step
     * 
     * @param[in] angular_velocity Angular rate vector in radians/second (body frame)
     * @param[in] time_delta Time step in seconds (should be small, typically < 0.01s)
     * 
     * @details Integrates angular velocity to produce rotation quaternion representing
     *          attitude change over time interval. Used in:
     *          - Gyroscope integration for attitude propagation
     *          - Dead reckoning between EKF updates
     *          - High-rate attitude estimation
     *          
     *          Calculation: rotation_angle = angular_velocity * time_delta
     *          Then applies small-angle quaternion construction.
     * 
     * @note Uses small-angle approximation - time_delta should be small (< 10ms typical)
     * @note angular_velocity in body frame (rad/s)
     * @note time_delta in seconds
     * 
     * @warning Large time_delta or high angular rates reduce accuracy
     * @note Typical use: 400Hz loop (time_delta = 0.0025s) with angular rates < 500°/s
     */
    void        from_angular_velocity(const Vector3<T>& angular_velocity, float time_delta);

    /**
     * @brief Rotate quaternion by rotation vector using small angle approximation
     * 
     * @param[in] v Rotation vector (direction=axis, magnitude=angle in radians)
     * 
     * @details Fast rotation update using small angle approximation for high-rate control loops.
     *          Applies rotation: q_new ≈ q_current + 0.5 * q_current * [0, v]
     * 
     * @note Modifies quaternion in-place
     * @note ONLY valid for small angles: |v| < 0.17 radians (≈10 degrees)
     * @warning Accuracy degrades beyond 10° - use rotate() for large rotations
     * @note Common in IMU integration at 400Hz+ rates
     * 
     * @see rotate() for full-precision rotation
     */
    void        rotate_fast(const Vector3<T> &v);

    /**
     * @brief Extract roll angle from quaternion
     * 
     * @return Roll angle in radians (rotation about forward/x axis)
     * 
     * @details Extracts roll component from quaternion using standard 321 Euler decomposition.
     *          Roll represents bank angle (rotation about vehicle forward axis).
     *          
     *          Sign convention (NED frame):
     *          - Positive roll = right wing down
     *          - Negative roll = left wing down
     * 
     * @note Returns angle in radians, not degrees
     * @note Valid range: [-π, π]
     * 
     * @see get_euler_pitch() get_euler_yaw() for other Euler components
     * @see to_euler() to extract all three angles at once (more efficient)
     */
    T       get_euler_roll() const;

    /**
     * @brief Extract pitch angle from quaternion
     * 
     * @return Pitch angle in radians (rotation about right/y axis)
     * 
     * @details Extracts pitch component from quaternion using standard 321 Euler decomposition.
     *          Pitch represents elevation angle (rotation about vehicle right axis).
     *          
     *          Sign convention (NED frame):
     *          - Positive pitch = nose up
     *          - Negative pitch = nose down
     * 
     * @note Returns angle in radians, not degrees
     * @note Valid range: [-π/2, π/2]
     * @warning Singularity at ±90° pitch (gimbal lock)
     * 
     * @see get_euler_roll() get_euler_yaw() for other Euler components
     */
    T       get_euler_pitch() const;

    /**
     * @brief Extract yaw angle from quaternion
     * 
     * @return Yaw angle in radians (rotation about down/z axis)
     * 
     * @details Extracts yaw component from quaternion using standard 321 Euler decomposition.
     *          Yaw represents heading angle (rotation about vehicle down axis).
     *          
     *          Sign convention (NED frame):
     *          - Positive yaw = nose right (clockwise from above)
     *          - Negative yaw = nose left (counter-clockwise from above)
     * 
     * @note Returns angle in radians, not degrees
     * @note Valid range: [-π, π]
     * @note 0 radians = North, π/2 = East, π = South, -π/2 = West
     * 
     * @see get_euler_roll() get_euler_pitch() for other Euler components
     */
    T       get_euler_yaw() const;

    /**
     * @brief Convert quaternion to Euler angles using 321 rotation sequence (float)
     * 
     * @param[out] roll  Roll angle in radians (rotation about x-axis)
     * @param[out] pitch Pitch angle in radians (rotation about y-axis)
     * @param[out] yaw   Yaw angle in radians (rotation about z-axis)
     * 
     * @details Converts unit quaternion to Euler angles using 321 sequence (standard aerospace).
     *          Extracts all three Euler components simultaneously - more efficient than calling
     *          individual get_euler_*() methods.
     *          
     *          Euler Angle Ranges:
     *          - Roll: [-π, π]
     *          - Pitch: [-π/2, π/2]
     *          - Yaw: [-π, π]
     * 
     * @note All angles in radians
     * @warning Singularity at pitch = ±90° (gimbal lock)
     * @note More efficient than calling get_euler_roll/pitch/yaw separately
     * 
     * @see from_euler() for inverse conversion
     * @see to_vector312() for 312 rotation sequence
     */
    void        to_euler(float &roll, float &pitch, float &yaw) const;

    /**
     * @brief Convert quaternion to Euler angles in Vector3f
     * 
     * @param[out] rpy Vector3f to populate with [roll, pitch, yaw] in radians
     * 
     * @note Convenience wrapper for to_euler(roll, pitch, yaw)
     * @see to_euler(float &roll, float &pitch, float &yaw) for detailed documentation
     */
    void        to_euler(Vector3f &rpy) const {
        to_euler(rpy.x, rpy.y, rpy.z);
    }

    /**
     * @brief Convert quaternion to Euler angles using 321 rotation sequence (double)
     * 
     * @param[out] roll  Roll angle in radians
     * @param[out] pitch Pitch angle in radians
     * @param[out] yaw   Yaw angle in radians
     * 
     * @details Double-precision version for high-accuracy applications
     * 
     * @see to_euler(float &roll, float &pitch, float &yaw) for detailed documentation
     */
    void        to_euler(double &roll, double &pitch, double &yaw) const;

    /**
     * @brief Convert quaternion to Euler angles in Vector3d
     * 
     * @param[out] rpy Vector3d to populate with [roll, pitch, yaw] in radians
     * 
     * @note Double-precision convenience wrapper
     * @see to_euler(double &roll, double &pitch, double &yaw) for detailed documentation
     */
    void        to_euler(Vector3d &rpy) const {
        to_euler(rpy.x, rpy.y, rpy.z);
    }

    /**
     * @brief Convert quaternion to Euler angles using 312 rotation sequence
     * 
     * @return Vector3 containing [roll, pitch, yaw] in radians using 312 ordering
     * 
     * @details Extracts Euler angles using 312 rotation sequence (yaw-roll-pitch order)
     *          instead of standard 321 (yaw-pitch-roll).
     *          
     *          Used in specific applications requiring alternative rotation decomposition.
     * 
     * @note Less commonly used than standard 321 sequence
     * @note All angles in radians
     * 
     * @see to_euler() for standard 321 rotation sequence
     * @see from_vector312() for inverse conversion
     */
    Vector3<T>    to_vector312(void) const;

    /**
     * @brief Calculate squared magnitude of quaternion
     * 
     * @return Squared length: q1² + q2² + q3² + q4²
     * 
     * @details More efficient than length() as it avoids square root calculation.
     *          Useful for comparing quaternion magnitudes or checking near-unit-length.
     *          
     *          For unit quaternions (pure rotations): length_squared() should equal 1.0
     * 
     * @note Faster than length() - prefer when absolute magnitude not needed
     * @note Unit quaternion constraint: q1² + q2² + q3² + q4² = 1
     * 
     * @see length() for actual magnitude
     * @see is_unit_length() to check unit constraint with tolerance
     */
    T length_squared(void) const;

    /**
     * @brief Calculate magnitude (length) of quaternion
     * 
     * @return Length: sqrt(q1² + q2² + q3² + q4²)
     * 
     * @details Computes Euclidean norm of quaternion.
     *          For unit quaternions (representing pure rotations): length() = 1.0
     * 
     * @note Involves square root - use length_squared() when possible for performance
     * 
     * @see length_squared() for squared magnitude (faster)
     * @see normalize() to enforce unit length
     */
    T length(void) const;

    /**
     * @brief Normalize quaternion to unit length
     * 
     * @details Scales quaternion components so that q1² + q2² + q3² + q4² = 1.
     *          Essential for maintaining pure rotation representation as numerical
     *          errors can cause quaternion to drift from unit constraint.
     *          
     *          Normalization: q_normalized = q / |q|
     *          
     *          When to normalize:
     *          - After arithmetic operations (addition, scaling)
     *          - Periodically during integration (every 10-100 iterations)
     *          - After loading from storage
     *          - When is_unit_length() returns false
     * 
     * @note Modifies quaternion in-place
     * @warning If length is near zero, quaternion becomes invalid
     * @note Typically called periodically in attitude estimation loops
     * 
     * @see is_unit_length() to check if normalization needed
     */
    void normalize();

    /**
     * @brief Check if all quaternion components are zero
     * 
     * @return true if q1=0 and q2=0 and q3=0 and q4=0, false otherwise
     * 
     * @note Zero quaternion [0,0,0,0] is mathematically invalid (no rotation)
     * @note Distinct from identity quaternion [1,0,0,0] (no rotation but valid)
     * 
     * @warning Zero quaternion should not be used for rotations
     * @see initialise() to create identity quaternion (valid zero rotation)
     */
    bool is_zero(void) const;

    /**
     * @brief Set all quaternion components to zero (invalid quaternion)
     * 
     * @details Sets q1=0, q2=0, q3=0, q4=0 creating mathematically invalid quaternion.
     *          Used for error indication or uninitialized state.
     * 
     * @warning Creates invalid quaternion - do NOT use for rotations
     * @note To create identity rotation (roll=pitch=yaw=0), use initialise() instead
     * 
     * @see initialise() to create valid identity quaternion
     * @see is_zero() to check if quaternion is zeroed
     */
    void zero(void);

    /**
     * @brief Check if quaternion is approximately unit length
     * 
     * @return true if magnitude is within tolerance of 1.0, false otherwise
     * 
     * @details Checks if |q|² is close to 1.0 within tolerance ±1E-3.
     *          Used to verify quaternion represents pure rotation without scaling.
     *          
     *          Tolerance (1E-3) chosen to:
     *          - Exceed floating-point epsilon sqrt(FLT_EPSILON) ≈ 3.4E-4
     *          - Allow for accumulation of small numerical errors
     *          - Catch significant drift requiring normalization
     * 
     * @note Tolerance: ±0.001 (0.1%)
     * @note If returns false, call normalize() to restore unit constraint
     * @note More efficient than checking length() == 1.0 due to tolerance
     * 
     * @see normalize() to enforce unit length
     * @see length_squared() for exact magnitude check
     */
    bool is_unit_length(void) const;

    /**
     * @brief Initialize quaternion to identity (no rotation)
     * 
     * @details Sets quaternion to identity: q1=1, q2=0, q3=0, q4=0
     *          Represents zero rotation (roll=pitch=yaw=0).
     *          
     *          This is the multiplicative identity: q * identity = q
     * 
     * @note Creates valid unit quaternion (unlike zero())
     * @note Equivalent to default constructor behavior
     * 
     * @see zero() which creates invalid zero quaternion
     */
    void initialise()
    {
        q1 = 1.0f;
        q2 = q3 = q4 = 0.0f;
    }

    /**
     * @brief Return inverse (conjugate) of unit quaternion
     * 
     * @return Inverted quaternion representing opposite rotation
     * 
     * @details For unit quaternions, inverse equals conjugate: q^(-1) = q*
     *          Conjugate: q* = [q1, -q2, -q3, -q4]
     *          
     *          Properties:
     *          - q * q^(-1) = identity quaternion [1,0,0,0]
     *          - Inverts rotation: if q rotates A→B, then q^(-1) rotates B→A
     *          
     *          Common uses:
     *          - Rotating from body frame to earth frame
     *          - Undoing a rotation
     *          - Computing angular differences
     * 
     * @note Returns new quaternion, does not modify original
     * @note Only valid for unit quaternions - normalize first if needed
     * @note For non-unit quaternions: inverse = conjugate / length²
     * 
     * @see invert() to modify quaternion in-place
     * @see earth_to_body() uses this internally for frame transformations
     */
    QuaternionT<T> inverse(void) const;

    /**
     * @brief Invert quaternion in-place (reverse rotation)
     * 
     * @details Computes conjugate for unit quaternions: q* = [q1, -q2, -q3, -q4]
     *          Reverses the rotation this quaternion represents.
     * 
     * @note Modifies quaternion in-place
     * @note For unit quaternions, inversion is just conjugate (negate vector part)
     * 
     * @see inverse() for non-modifying version
     */
    void invert();

    /**
     * @brief Array subscript operator for quaternion component access (non-const)
     * 
     * @param[in] i Component index: 0=q1, 1=q2, 2=q3, 3=q4
     * 
     * @return Reference to quaternion component for read/write access
     * 
     * @note Index 0 accesses q1 (scalar part), indices 1-3 access q2/q3/q4 (vector parts)
     * @note If MATH_CHECK_INDEXES enabled, asserts i < 4
     * 
     * @warning No bounds checking in release builds - caller must ensure i < 4
     */
    T & operator[](uint8_t i)
    {
        T *_v = &q1;
#if MATH_CHECK_INDEXES
        assert(i < 4);
#endif
        return _v[i];
    }

    /**
     * @brief Array subscript operator for quaternion component access (const)
     * 
     * @param[in] i Component index: 0=q1, 1=q2, 2=q3, 3=q4
     * 
     * @return Const reference to quaternion component for read-only access
     * 
     * @note Index 0 accesses q1 (scalar part), indices 1-3 access q2/q3/q4 (vector parts)
     * @note If MATH_CHECK_INDEXES enabled, asserts i < 4
     * 
     * @warning No bounds checking in release builds - caller must ensure i < 4
     */
    const T & operator[](uint8_t i) const
    {
        const T *_v = &q1;
#if MATH_CHECK_INDEXES
        assert(i < 4);
#endif
        return _v[i];
    }

    /**
     * @brief Quaternion multiplication operator (composition of rotations)
     * 
     * @param[in] v Quaternion to multiply (right operand)
     * 
     * @return Result of quaternion multiplication: this * v
     * 
     * @details Composes two rotations using quaternion multiplication formula.
     *          Result represents applying rotation v first, then this rotation.
     *          
     *          Hamilton product: q1*q2 gives rotation q2 followed by q1
     *          
     *          Important: Quaternion multiplication is non-commutative!
     *          q1 * q2 ≠ q2 * q1 (order matters)
     * 
     * @note Does not modify this quaternion
     * @note Result should be normalized if used for rotations
     * @warning Non-commutative: order of multiplication matters
     * 
     * @see operator*=(const QuaternionT<T> &v) for in-place version
     */
    QuaternionT<T> operator*(const QuaternionT<T> &v) const;

    /**
     * @brief Rotate vector by quaternion using multiplication operator
     * 
     * @param[in] v Vector3 to rotate
     * 
     * @return Rotated vector
     * 
     * @details Applies quaternion rotation to vector: v_rotated = q * v * q*
     *          Convenient operator form of rotation transformation.
     * 
     * @note Returns rotated vector, does not modify original
     * @note Quaternion should be unit length for accurate rotation
     * 
     * @see earth_to_body() for frame transformation with in-place modification
     */
    Vector3<T> operator*(const Vector3<T> &v) const;

    /**
     * @brief Quaternion multiplication assignment (compose rotation in-place)
     * 
     * @param[in] v Quaternion to multiply with this quaternion
     * 
     * @return Reference to this quaternion after multiplication
     * 
     * @details Composes rotation in-place: this = this * v
     *          Accumulates rotation represented by v into this quaternion.
     * 
     * @note Modifies this quaternion
     * @note Equivalent to: *this = *this * v
     * 
     * @see operator*(const QuaternionT<T> &v) for non-modifying version
     */
    QuaternionT<T> &operator*=(const QuaternionT<T> &v);

    /**
     * @brief Quaternion division operator (rotation difference)
     * 
     * @param[in] v Quaternion divisor (right operand)
     * 
     * @return Result of quaternion division: this / v = this * v^(-1)
     * 
     * @details Computes rotation difference between quaternions.
     *          Result represents rotation from v to this.
     *          
     *          Equivalent to: this * v.inverse()
     * 
     * @note Does not modify this quaternion
     * @note Result represents relative rotation
     * 
     * @see angular_difference() for alternative difference computation
     * @see inverse() for quaternion inversion
     */
    QuaternionT<T> operator/(const QuaternionT<T> &v) const;

    /**
     * @brief Compute angular difference between two quaternions
     * 
     * @param[in] v Quaternion to compare with this quaternion
     * 
     * @return Quaternion representing rotation from v to this
     * 
     * @details Calculates relative rotation needed to transform from quaternion v
     *          to this quaternion. Result can be converted to axis-angle to get
     *          rotation magnitude and direction.
     *          
     *          Common applications:
     *          - Computing attitude error for control
     *          - Measuring rotation between two orientations
     *          - Tracking orientation changes
     *          
     *          Calculation: q_diff = this * v.inverse()
     * 
     * @note Returns difference quaternion (does not modify this)
     * @note Convert result to axis-angle to get rotation magnitude
     * 
     * @see roll_pitch_difference() for earth-frame roll-pitch difference only
     * @see to_axis_angle() to extract rotation magnitude from result
     */
    QuaternionT<T> angular_difference(const QuaternionT<T> &v) const;

    /**
     * @brief Compute earth-frame roll-pitch difference between quaternions
     * 
     * @param[in] v Quaternion to compare with this quaternion
     * 
     * @return Absolute roll-pitch difference in radians (always positive)
     * 
     * @details Calculates magnitude of roll-pitch orientation difference in earth frame,
     *          ignoring yaw component. Useful for:
     *          - Measuring attitude error excluding heading
     *          - Assessing vehicle tilt relative to reference
     *          - Monitoring roll-pitch stability independently of yaw
     *          
     *          Returns scalar angle representing combined roll-pitch deviation.
     * 
     * @note Returns magnitude only (always positive), not direction
     * @note Yaw difference not included in calculation
     * @note Result in radians
     * 
     * @see angular_difference() for full 3D rotation difference including yaw
     */
    T roll_pitch_difference(const QuaternionT<T> &v) const;

    /**
     * @brief Convert quaternion to double precision
     * 
     * @return QuaternionT<double> with same rotation, double precision
     * 
     * @details Creates double-precision copy of this quaternion for high-accuracy
     *          calculations. All components (q1, q2, q3, q4) converted to double.
     * 
     * @note Returns new quaternion, does not modify original
     * @note Useful when interfacing with double-precision math libraries
     */
    QuaternionT<double> todouble(void) const {
        return QuaternionT<double>(q1,q2,q3,q4);
    }

    /**
     * @brief Convert quaternion to single precision
     * 
     * @return QuaternionT<float> with same rotation, single precision
     * 
     * @details Creates single-precision copy of this quaternion for memory efficiency
     *          and performance. All components (q1, q2, q3, q4) converted to float.
     * 
     * @note Returns new quaternion, does not modify original
     * @note Precision loss may occur when converting from double to float
     * @warning Small numerical errors may accumulate with float precision
     */
    QuaternionT<float> tofloat(void) const {
        return QuaternionT<float>(q1,q2,q3,q4);
    }
};

/**
 * @typedef Quaternion
 * @brief Single-precision quaternion (default type)
 * 
 * @details Standard quaternion type used throughout ArduPilot for attitude representation.
 *          Uses float (32-bit) for memory efficiency and performance on embedded hardware.
 *          
 *          Sufficient precision for:
 *          - Real-time attitude control (400Hz loops)
 *          - Sensor fusion in AP_AHRS
 *          - Flight mode computations
 *          
 *          Typical usage: Quaternion q; q.from_euler(roll, pitch, yaw);
 * 
 * @note Float precision adequate for flight control applications
 * @see QuaternionD for double-precision alternative
 */
typedef QuaternionT<float> Quaternion;

/**
 * @typedef QuaternionD
 * @brief Double-precision quaternion
 * 
 * @details High-precision quaternion type for applications requiring extended accuracy:
 *          - Long-duration missions with accumulated integration
 *          - High-precision navigation calculations
 *          - Simulation and analysis tools
 *          
 *          Uses double (64-bit) at cost of:
 *          - 2x memory usage
 *          - Slower computation on some ARM processors
 * 
 * @note Use when precision requirements exceed float capability
 * @see Quaternion for standard single-precision type
 */
typedef QuaternionT<double> QuaternionD;



