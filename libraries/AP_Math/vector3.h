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

// Copyright 2010 Michael Smith, all rights reserved.

// Derived closely from:
/****************************************
* 3D Vector Classes
* By Bill Perone (billperone@yahoo.com)
* Original: 9-16-2002
* Revised: 19-11-2003
*          11-12-2003
*          18-12-2003
*          06-06-2004
*
* Copyright 2003, This code is provided "as is" and you can use it freely as long as
* credit is given to Bill Perone in the application it is used in
*
* Notes:
* if a*b = 0 then a & b are orthogonal
* a%b = -b%a
* a*(b%c) = (a%b)*c
* a%b = a(cast to matrix)*b
* (a%b).length() = area of parallelogram formed by a & b
* (a%b).length() = a.length()*b.length() * sin(angle between a & b)
* (a%b).length() = 0 if angle between a & b = 0 or a.length() = 0 or b.length() = 0
* a * (b%c) = volume of parallelpiped formed by a, b, c
* vector triple product: a%(b%c) = b*(a*c) - c*(a*b)
* scalar triple product: a*(b%c) = c*(a%b) = b*(c%a)
* vector quadruple product: (a%b)*(c%d) = (a*c)*(b*d) - (a*d)*(b*c)
* if a is unit vector along b then a%b = -b%a = -b(cast to matrix)*a = 0
* vectors a1...an are linearly dependent if there exists a vector of scalars (b) where a1*b1 + ... + an*bn = 0
*           or if the matrix (A) * b = 0
*
****************************************/

/**
 * @file vector3.h
 * @brief 3D vector class for spatial mathematics and physics calculations
 * 
 * @details This file provides a template-based 3D vector class used throughout ArduPilot
 *          for representing and manipulating spatial quantities with Cartesian coordinates (x, y, z).
 *          
 *          The Vector3 class implements comprehensive vector algebra including:
 *          - Arithmetic operations (addition, subtraction, scaling)
 *          - Vector products (dot product, cross product)
 *          - Geometric operations (magnitude, normalization, projection)
 *          - Rotations and coordinate transformations
 *          - Distance and angle calculations
 *          
 *          Common use cases include:
 *          - Position vectors (NED frame: North, East, Down in meters)
 *          - Velocity vectors (m/s in body or earth frame)
 *          - Acceleration vectors (m/s² including gravity)
 *          - Force vectors (Newtons in body frame)
 *          - Rotation axes and angular velocity (rad/s)
 *          - Magnetic field vectors (Gauss or milliGauss)
 *          
 *          Coordinate frame conventions:
 *          - Body frame: x forward, y right, z down (right-hand rule)
 *          - NED frame: x north, y east, z down (earth-fixed)
 *          - ENU frame: x east, y north, z up (can be converted via rfu_to_frd())
 *          
 *          Vector identities (from Bill Perone's original implementation):
 *          - Dot product: a*b = 0 if vectors are orthogonal
 *          - Cross product: a%b = -b%a (anti-commutative)
 *          - Scalar triple product: a*(b%c) = volume of parallelepiped
 *          - Vector triple product: a%(b%c) = b*(a*c) - c*(a*b)
 *          
 * @note This implementation is derived from Bill Perone's vector classes with
 *       ArduPilot-specific enhancements for embedded systems and flight control.
 * 
 * @note Explicit template instantiations for float and double are provided in vector3.cpp
 * 
 * @see Vector2 for 2D vector operations
 * @see Matrix3 for 3x3 matrix transformations
 * @see Quaternion for rotation representations
 * @see rotations.h for standard rotation enumerations
 * 
 * Source: libraries/AP_Math/vector3.h
 */
#pragma once

#ifndef MATH_CHECK_INDEXES
#define MATH_CHECK_INDEXES 0
#endif

#include <cmath>
#include <float.h>
#include <string.h>
#if MATH_CHECK_INDEXES
#include <assert.h>
#endif

#include "rotations.h"

#include "ftype.h"

template <typename T>
class Matrix3;

template <typename T>
class Vector2;

/**
 * @class Vector3
 * @brief 3D vector template class with Cartesian coordinates (x, y, z)
 * 
 * @tparam T Numeric type for vector components (typically float or double)
 * 
 * @details Vector3 provides a comprehensive 3D vector implementation for spatial mathematics
 *          and physics calculations in ArduPilot. The class supports all standard vector
 *          algebra operations and is used extensively for position, velocity, acceleration,
 *          force, and rotation representations.
 *          
 *          Key capabilities:
 *          - Vector arithmetic: addition (+), subtraction (-), negation (-)
 *          - Scalar operations: multiplication (*), division (/)
 *          - Dot product: a * b (returns scalar)
 *          - Cross product: a % b (returns perpendicular vector)
 *          - Magnitude: length(), length_squared()
 *          - Normalization: normalize(), normalized()
 *          - Projections: project(), projected(), perpendicular()
 *          - Rotations: rotate(), rotate_inverse(), rotate_xy()
 *          - Geometric queries: angle(), distance_to_segment(), closest_point()
 *          - Coordinate conversions: rfu_to_frd(), xy()
 *          - Type conversions: tofloat(), todouble()
 *          
 *          Memory layout: Components x, y, z are stored contiguously in memory,
 *          allowing array-style access via operator[] and pointer casting for
 *          efficient operations.
 *          
 *          Thread safety: Individual Vector3 objects are NOT thread-safe. External
 *          synchronization required if shared between threads.
 *          
 *          Performance considerations:
 *          - length() involves sqrt() calculation - use length_squared() when possible
 *          - normalize() and normalized() fail silently for zero-length vectors
 *          - Cross product (%) allocates temporary return object
 *          - AP_INLINE_VECTOR_OPS can be defined for inline hot-path operations
 *          
 *          Common instantiations:
 *          - Vector3f: float precision (most common in flight code)
 *          - Vector3d: double precision (for high-accuracy calculations)
 *          - Vector3i: 16-bit integer
 *          - Vector3l: 32-bit integer
 *          
 * @note Derived from Bill Perone's vector classes with ArduPilot-specific enhancements
 *       for embedded flight control systems.
 * 
 * @warning normalize() returns zero vector if length is zero or near-zero, which may
 *          produce invalid results in subsequent calculations. Always check is_zero()
 *          before normalizing if zero-length vectors are possible.
 * 
 * @warning Floating-point equality operators (==, !=) use epsilon tolerance for float
 *          and double types, but exact comparison for integer types.
 * 
 * Source: libraries/AP_Math/vector3.h:72-320
 */
template <typename T>
class Vector3
{

public:
    T        x, y, z;  ///< Cartesian coordinates: x (forward/north), y (right/east), z (down)

    /**
     * @brief Default constructor - creates zero vector (0, 0, 0)
     * 
     * @note Declared constexpr for compile-time initialization
     */
    constexpr Vector3()
        : x(0)
        , y(0)
        , z(0) {}

    /**
     * @brief Constructor with explicit component values
     * 
     * @param[in] x0 X-axis component (forward in body frame, north in NED frame)
     * @param[in] y0 Y-axis component (right in body frame, east in NED frame)
     * @param[in] z0 Z-axis component (down in both body and NED frames)
     * 
     * @note Declared constexpr for compile-time initialization
     */
    constexpr Vector3(const T x0, const T y0, const T z0)
        : x(x0)
        , y(y0)
        , z(z0) {}

    /**
     * @brief Constructor from 2D vector and z component
     * 
     * @param[in] v0 Vector2 providing x and y components
     * @param[in] z0 Z-axis component (down)
     * 
     * @details Useful for constructing 3D position from horizontal position (x, y)
     *          plus altitude/depth (z). Common in waypoint navigation where horizontal
     *          and vertical components are handled separately.
     * 
     * @note Declared constexpr for compile-time initialization
     */
    constexpr Vector3(const Vector2<T> &v0, const T z0)
        : x(v0.x)
        , y(v0.y)
        , z(z0) {}

    /**
     * @brief Test for equality with another vector
     * 
     * @param[in] v Vector to compare against
     * @return true if vectors are equal (within epsilon tolerance for float/double), false otherwise
     * 
     * @note For floating-point types (float, double), uses epsilon-based comparison to handle
     *       numerical precision issues. For integer types, uses exact comparison.
     */
    bool operator ==(const Vector3<T> &v) const;

    /**
     * @brief Test for inequality with another vector
     * 
     * @param[in] v Vector to compare against
     * @return true if vectors are not equal, false if equal
     * 
     * @note For floating-point types (float, double), uses epsilon-based comparison.
     */
    bool operator !=(const Vector3<T> &v) const;

    /**
     * @brief Unary negation - returns vector with all components negated
     * 
     * @return New vector (-x, -y, -z)
     * 
     * @details Useful for reversing direction. For example, negating a velocity vector
     *          gives the opposite velocity direction.
     */
    Vector3<T> operator -(void) const;

    /**
     * @brief Vector addition
     * 
     * @param[in] v Vector to add
     * @return New vector with component-wise sum (x+v.x, y+v.y, z+v.z)
     * 
     * @details Standard vector addition. Commonly used for:
     *          - Displacement: new_position = old_position + displacement
     *          - Velocity composition: total_velocity = wind_velocity + airspeed_velocity
     */
    Vector3<T> operator +(const Vector3<T> &v) const;

    /**
     * @brief Vector subtraction
     * 
     * @param[in] v Vector to subtract
     * @return New vector with component-wise difference (x-v.x, y-v.y, z-v.z)
     * 
     * @details Standard vector subtraction. Commonly used for:
     *          - Displacement: displacement = target_position - current_position
     *          - Relative velocity: relative_vel = vehicle_vel - wind_vel
     */
    Vector3<T> operator -(const Vector3<T> &v) const;

    /**
     * @brief Scalar multiplication (uniform scaling)
     * 
     * @param[in] num Scalar multiplier
     * @return New vector with all components scaled (x*num, y*num, z*num)
     * 
     * @details Scales vector magnitude by scalar factor while preserving direction.
     *          Common uses: converting units, computing forces (F = m * a)
     */
    Vector3<T> operator *(const T num) const;

    /**
     * @brief Scalar division (uniform scaling)
     * 
     * @param[in] num Scalar divisor
     * @return New vector with all components divided (x/num, y/num, z/num)
     * 
     * @warning No divide-by-zero check. Caller must ensure num != 0.
     * 
     * @details Scales vector magnitude inversely. Common uses: averaging, normalization
     */
    Vector3<T> operator  /(const T num) const;

    /**
     * @brief In-place vector addition
     * 
     * @param[in] v Vector to add
     * @return Reference to this vector after addition
     * 
     * @details Modifies this vector: this = this + v
     */
    Vector3<T> &operator +=(const Vector3<T> &v);

    /**
     * @brief In-place vector subtraction
     * 
     * @param[in] v Vector to subtract
     * @return Reference to this vector after subtraction
     * 
     * @details Modifies this vector: this = this - v
     */
    Vector3<T> &operator -=(const Vector3<T> &v);

    /**
     * @brief In-place scalar multiplication (uniform scaling)
     * 
     * @param[in] num Scalar multiplier
     * @return Reference to this vector after scaling
     * 
     * @details Modifies this vector: this = this * num
     */
    Vector3<T> &operator *=(const T num);

    /**
     * @brief In-place scalar division (uniform scaling)
     * 
     * @param[in] num Scalar divisor
     * @return Reference to this vector after division
     * 
     * @warning No divide-by-zero check. Caller must ensure num != 0.
     * 
     * @details Modifies this vector: this = this / num
     */
    Vector3<T> &operator /=(const T num);

    /**
     * @brief In-place component-wise multiplication (non-uniform scaling)
     * 
     * @param[in] v Vector with scale factors for each component
     * @return Reference to this vector after component-wise multiplication
     * 
     * @details Multiplies each component independently: x *= v.x, y *= v.y, z *= v.z
     *          Useful for scaling vectors differently in each axis.
     */
    Vector3<T> &operator *=(const Vector3<T> &v) {
        x *= v.x; y *= v.y; z *= v.z;
        return *this;
    }

    /**
     * @brief Array-style element access (mutable)
     * 
     * @param[in] i Index: 0=x, 1=y, 2=z
     * @return Reference to the indexed component
     * 
     * @note Allows vector[i] notation for accessing components
     * @note If MATH_CHECK_INDEXES is defined, asserts that 0 <= i < 3
     * 
     * @warning No bounds checking in production builds. Accessing i >= 3 is undefined behavior.
     */
    T & operator[](uint8_t i) {
        T *_v = &x;
#if MATH_CHECK_INDEXES
        assert(i >= 0 && i < 3);
#endif
        return _v[i];
    }

    /**
     * @brief Array-style element access (const)
     * 
     * @param[in] i Index: 0=x, 1=y, 2=z
     * @return Const reference to the indexed component
     * 
     * @note Allows vector[i] notation for accessing components in const contexts
     * @note If MATH_CHECK_INDEXES is defined, asserts that 0 <= i < 3
     * 
     * @warning No bounds checking in production builds. Accessing i >= 3 is undefined behavior.
     */
    const T & operator[](uint8_t i) const {
        const T *_v = &x;
#if MATH_CHECK_INDEXES
        assert(i >= 0 && i < 3);
#endif
        return _v[i];
    }

    /**
     * @brief Dot product (scalar product) with another vector
     * 
     * @param[in] v Second vector
     * @return Scalar result: x*v.x + y*v.y + z*v.z
     * 
     * @details The dot product measures the projection of one vector onto another.
     *          
     *          Geometric interpretation: a * b = ||a|| * ||b|| * cos(θ)
     *          where θ is the angle between vectors.
     *          
     *          Properties:
     *          - a * b = 0 if vectors are perpendicular (orthogonal)
     *          - a * b > 0 if angle < 90° (vectors point in similar directions)
     *          - a * b < 0 if angle > 90° (vectors point in opposite directions)
     *          - a * a = ||a||² (magnitude squared)
     *          
     *          Common uses:
     *          - Computing projections: proj_a(b) = (a*b / a*a) * a
     *          - Testing perpendicularity: if (a*b == 0) vectors are orthogonal
     *          - Computing work: work = force * displacement
     * 
     * @note For Lua scripting compatibility, also available as dot() method
     */
    T operator *(const Vector3<T> &v) const;

    /**
     * @brief Dot product - Lua-compatible method name
     * 
     * @param[in] v Second vector
     * @return Scalar dot product result
     * 
     * @details Identical to operator* but provided for Lua scripting compatibility
     *          where operator overloading is not available.
     * 
     * @see operator*(const Vector3<T>&) for detailed documentation
     */
    T dot(const Vector3<T> &v) const {
        return *this * v;
    }
    
    /**
     * @brief Multiply row vector by matrix: result = this * matrix
     * 
     * @param[in] m 3x3 matrix
     * @return Resulting row vector after multiplication
     * 
     * @details Treats this vector as a 1x3 row vector and multiplies by 3x3 matrix.
     *          Used for transforming vectors from one coordinate frame to another.
     */
    Vector3<T> row_times_mat(const Matrix3<T> &m) const;

    /**
     * @brief Outer product: column vector × row vector = 3x3 matrix
     * 
     * @param[in] v Row vector
     * @return 3x3 matrix result of outer product
     * 
     * @details Computes outer product: this (as column) × v (as row)
     *          Resulting matrix M[i][j] = this[i] * v[j]
     */
    Matrix3<T> mul_rowcol(const Vector3<T> &v) const;

    /**
     * @brief Cross product (vector product) with another vector
     * 
     * @param[in] v Second vector
     * @return Perpendicular vector following right-hand rule
     * 
     * @details The cross product produces a vector perpendicular to both input vectors.
     *          
     *          Formula: a % b = (a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x)
     *          
     *          Geometric interpretation: ||a % b|| = ||a|| * ||b|| * sin(θ)
     *          Direction: perpendicular to both a and b following right-hand rule
     *          
     *          Properties (from Bill Perone's notes):
     *          - a % b = -b % a (anti-commutative)
     *          - ||a % b|| = area of parallelogram formed by a and b
     *          - ||a % b|| = 0 if vectors are parallel or either is zero
     *          - a % b perpendicular to both a and b
     *          
     *          Common uses:
     *          - Computing torque: τ = r % F (position × force)
     *          - Finding perpendicular: normal = v1 % v2
     *          - Angular momentum: L = r % p (position × momentum)
     *          - Body frame rotations in flight dynamics
     * 
     * @note The % operator was chosen to represent cross product in C++
     * @note For Lua scripting compatibility, also available as cross() method
     * 
     * @see Vector triple product: a%(b%c) = b*(a*c) - c*(a*b)
     */
    Vector3<T> operator %(const Vector3<T> &v) const;

    /**
     * @brief Cross product - Lua-compatible method name
     * 
     * @param[in] v Second vector
     * @return Perpendicular vector result
     * 
     * @details Identical to operator% but provided for Lua scripting compatibility
     *          where operator overloading is not available.
     * 
     * @see operator%(const Vector3<T>&) for detailed documentation
     */
    Vector3<T> cross(const Vector3<T> &v) const {
        return *this % v;
    }

    /**
     * @brief Scale vector by scalar - Lua-compatible method
     * 
     * @param[in] v Scalar multiplier
     * @return Scaled vector
     * 
     * @details Identical to operator* but provided for Lua scripting compatibility.
     */
    Vector3<T> scale(const T v) const {
        return *this * v;
    }
    
    /**
     * @brief Compute angle between this vector and another vector
     * 
     * @param[in] v2 Second vector
     * @return Angle in radians [0, π]
     * 
     * @details Calculates the angle between two vectors using:
     *          θ = acos((a * b) / (||a|| * ||b||))
     *          
     *          The angle is always positive and ranges from 0 (parallel, same direction)
     *          to π (parallel, opposite directions).
     *          
     * @note Returns 0 if either vector has zero length
     * 
     * @warning Returned angle is in radians, not degrees
     */
    T angle(const Vector3<T> &v2) const;

    /**
     * @brief Check if any component is NaN (Not a Number)
     * 
     * @return true if x, y, or z is NaN, false otherwise
     * 
     * @details Detects invalid floating-point values resulting from undefined operations
     *          like 0/0 or sqrt(-1). Essential for error detection in numerical calculations.
     *          
     * @note WARN_IF_UNUSED attribute ensures compiler warning if result is ignored
     * 
     * @warning NaN values propagate through calculations. Always check critical values.
     */
    bool is_nan(void) const WARN_IF_UNUSED;

    /**
     * @brief Check if any component is infinite
     * 
     * @return true if x, y, or z is positive or negative infinity, false otherwise
     * 
     * @details Detects overflow conditions in floating-point calculations like division
     *          by very small numbers or exponential growth. Essential for numerical stability.
     *          
     * @note WARN_IF_UNUSED attribute ensures compiler warning if result is ignored
     * 
     * @warning Infinite values can propagate and cause further numerical issues
     */
    bool is_inf(void) const WARN_IF_UNUSED;

    /**
     * @brief Check if vector is effectively zero
     * 
     * @return true if all components are zero (within epsilon for float/double), false otherwise
     * 
     * @details For integer types: checks exact equality to zero
     *          For float/double: uses epsilon tolerance via ::is_zero() helper
     *          (see template specializations at end of file)
     *          
     *          Essential for:
     *          - Avoiding division by zero in normalization
     *          - Detecting zero velocity or acceleration
     *          - Validating numerical convergence
     *          
     * @note WARN_IF_UNUSED attribute ensures compiler warning if result is ignored
     * 
     * @note Template specializations for float and double use epsilon-based comparison
     */
    bool is_zero(void) const WARN_IF_UNUSED {
        return x == 0 && y == 0 && z == 0;
    }


    /**
     * @brief Apply standard rotation transformation
     * 
     * @param[in] rotation Rotation enumeration from rotations.h
     * 
     * @details Rotates vector according to one of the predefined rotations in the Rotation enum.
     *          Common rotations include 90°, 180°, 270° around each axis, used for sensor
     *          orientation corrections.
     *          
     *          Modifies this vector in place.
     *          
     *          Typical use: Correcting for IMU or compass mounting orientation
     *          
     * @see rotations.h for available rotation enumerations
     * @see rotate_inverse() to undo a rotation
     */
    void rotate(enum Rotation rotation);

    /**
     * @brief Apply inverse of standard rotation transformation
     * 
     * @param[in] rotation Rotation enumeration from rotations.h
     * 
     * @details Applies the inverse (reverse) of the specified rotation. If rotate(R)
     *          transforms A to B, then rotate_inverse(R) transforms B back to A.
     *          
     *          Modifies this vector in place.
     *          
     * @see rotate() for forward rotation
     * @see rotations.h for available rotation enumerations
     */
    void rotate_inverse(enum Rotation rotation);

    /**
     * @brief Rotate vector in XY plane (yaw rotation)
     * 
     * @param[in] rotation_rad Rotation angle in radians (positive = counterclockwise when viewed from above)
     * 
     * @details Rotates vector around the Z-axis, leaving Z component unchanged.
     *          Equivalent to yaw rotation in aerospace terminology.
     *          
     *          New coordinates:
     *          x' = x*cos(θ) - y*sin(θ)
     *          y' = x*sin(θ) + y*cos(θ)
     *          z' = z (unchanged)
     *          
     *          Modifies this vector in place.
     *          
     *          Common uses:
     *          - Heading changes for navigation
     *          - Transforming between geographic and body frames
     *          
     * @warning Angle must be in radians, not degrees
     */
    void rotate_xy(T rotation_rad);

    /**
     * @brief Extract 2D horizontal component (x, y) as Vector2
     * 
     * @return Const reference to xy components as Vector2
     * 
     * @details Returns reference to the original vector's xy data without copying.
     *          Relies on memory layout where x and y are stored contiguously at the
     *          beginning of the Vector3 structure.
     *          
     *          Common uses:
     *          - Horizontal distance calculations (ignoring altitude)
     *          - 2D navigation and waypoint operations
     *          - Extracting ground speed from 3D velocity
     *          
     * @note This is a zero-cost operation (no copying, just pointer cast)
     * 
     * @warning Returned reference is only valid while this Vector3 exists
     */
    const Vector2<T> &xy() const {
        return *(const Vector2<T> *)this;
    }

    /**
     * @brief Extract 2D horizontal component (x, y) as Vector2 (mutable)
     * 
     * @return Mutable reference to xy components as Vector2
     * 
     * @details Same as const version but allows modification of xy components
     *          through the returned Vector2 reference.
     *          
     * @note Modifying the returned Vector2 modifies this Vector3's x and y components
     * 
     * @warning Returned reference is only valid while this Vector3 exists
     */
    Vector2<T> &xy() {
        return *(Vector2<T> *)this;
    }

    /**
     * @brief Get squared magnitude of vector (avoids sqrt calculation)
     * 
     * @return Magnitude squared: x² + y² + z²
     * 
     * @details Computes ||v||² = v * v (dot product with itself)
     *          
     *          Faster than length() because it avoids expensive sqrt() operation.
     *          Use when only relative magnitudes matter (comparisons, thresholding).
     *          
     *          Common uses:
     *          - Distance comparisons: if (offset.length_squared() < threshold²)
     *          - Normalization checks: if (v.length_squared() > epsilon²)
     *          - Performance-critical magnitude calculations
     *          
     * @note Preferred over length() when absolute magnitude not needed
     */
    T  length_squared() const
    {
        return (T)(*this * *this);
    }

    /**
     * @brief Get Euclidean magnitude of vector
     * 
     * @return Magnitude: ||v|| = sqrt(x² + y² + z²)
     * 
     * @details Computes the Euclidean norm (L2 norm) of the vector.
     *          
     *          Involves sqrt() calculation which is relatively expensive on embedded systems.
     *          Consider using length_squared() for comparisons when possible.
     *          
     * @note Returns 0 for zero vector
     * 
     * @warning Performance consideration: sqrt() is expensive. Use length_squared() for comparisons.
     */
    T length(void) const;

    /**
     * @brief Limit horizontal (XY) component to maximum length
     * 
     * @param[in] max_length Maximum allowed horizontal magnitude
     * @return true if vector was limited (exceeded max_length), false otherwise
     * 
     * @details Scales the xy components proportionally to limit horizontal magnitude
     *          while leaving z component unchanged.
     *          
     *          Useful for limiting horizontal velocity or acceleration while allowing
     *          unrestricted vertical motion.
     *          
     *          Modifies this vector in place if limiting occurs.
     */
    bool limit_length_xy(T max_length);

    /**
     * @brief Normalize vector to unit length (magnitude = 1)
     * 
     * @details Modifies this vector in place: v = v / ||v||
     *          
     *          After normalization (if successful):
     *          - length() == 1.0
     *          - Direction preserved
     *          - Useful for unit direction vectors
     *          
     *          Common uses:
     *          - Direction vectors for navigation
     *          - Normal vectors for surfaces
     *          - Axis representations
     *          
     * @warning If vector length is zero or near-zero, result is undefined (likely zero vector).
     *          Always check is_zero() before normalizing if zero-length vectors are possible.
     * 
     * @warning No error checking. Division by near-zero length produces invalid results.
     * 
     * @note For safe normalization, check: if (!v.is_zero()) v.normalize();
     * 
     * @see normalized() for non-mutating version
     */
    void normalize()
    {
        *this /= length();
    }

    /**
     * @brief Set all components to zero
     * 
     * @details Sets x = y = z = 0. Common initialization or reset operation.
     */
    void zero()
    {
        x = y = z = 0;
    }

    /**
     * @brief Return normalized copy without modifying original
     * 
     * @return Unit vector in same direction as this vector
     * 
     * @details Returns new vector with same direction but magnitude 1.
     *          Does not modify this vector.
     *          
     * @warning If vector length is zero or near-zero, result is undefined.
     *          Always check is_zero() before normalizing if zero-length vectors are possible.
     * 
     * @see normalize() for in-place version
     */
    Vector3<T> normalized() const
    {
        return *this/length();
    }

    /**
     * @brief Reflect vector about normal vector
     * 
     * @param[in] n Normal vector defining reflection plane
     * 
     * @details Reflects this vector about the plane perpendicular to n.
     *          Equivalent to mirror reflection.
     *          
     *          Algorithm: v' = 2*proj_n(v) - v
     *          where proj_n(v) is the projection of v onto n.
     *          
     *          Modifies this vector in place.
     *          
     *          Common use: Simulating bounces off surfaces
     */
    void  reflect(const Vector3<T> &n)
    {
        Vector3<T>        orig(*this);
        project(n);
        *this = *this*2 - orig;
    }

    /**
     * @brief Project this vector onto another vector (in-place)
     * 
     * @param[in] v Vector to project onto
     * 
     * @details Replaces this vector with its projection onto v.
     *          
     *          Formula: proj_v(this) = v * (this * v) / (v * v)
     *          
     *          Result is parallel to v with magnitude = ||this|| * cos(θ)
     *          where θ is angle between this and v.
     *          
     *          Modifies this vector in place.
     *          
     * @warning No check for zero-length v. Division by zero if v.length_squared() == 0.
     * 
     * @see projected() for non-mutating version
     */
    void project(const Vector3<T> &v)
    {
        *this= v * (*this * v)/(v*v);
    }

    /**
     * @brief Return projection of this vector onto another vector
     * 
     * @param[in] v Vector to project onto
     * @return Projection of this vector onto v (parallel component)
     * 
     * @details Computes the component of this vector parallel to v.
     *          
     *          Formula: proj_v(this) = v * (this * v) / (v * v)
     *          
     *          Result is parallel to v with magnitude = ||this|| * cos(θ)
     *          where θ is angle between this and v.
     *          
     *          Does not modify this vector.
     *          
     *          Common uses:
     *          - Decomposing velocity into parallel and perpendicular components
     *          - Finding component of force in a given direction
     *          
     * @warning No check for zero-length v. Division by zero if v.length_squared() == 0.
     * 
     * @see project() for in-place version
     * @see perpendicular() for orthogonal component
     */
    Vector3<T> projected(const Vector3<T> &v) const
    {
        return v * (*this * v)/(v*v);
    }

    /**
     * @brief Squared distance from this point to another point
     * 
     * @param[in] v Target point
     * @return Distance squared: ||this - v||²
     * 
     * @details Computes squared Euclidean distance between two points.
     *          Faster than computing actual distance because it avoids sqrt().
     *          
     *          Use for distance comparisons and thresholding.
     *          
     * @note Preferred over sqrt(distance_squared()) for comparisons
     */
    T distance_squared(const Vector3<T> &v) const {
        const T dist_x = x-v.x;
        const T dist_y = y-v.y;
        const T dist_z = z-v.z;
        return (dist_x*dist_x + dist_y*dist_y + dist_z*dist_z);
    }

    /**
     * @brief Minimum distance from this point to a line segment
     * 
     * @param[in] seg_start Start point of line segment
     * @param[in] seg_end End point of line segment
     * @return Shortest distance from this point to any point on the segment
     * 
     * @details Computes perpendicular distance if perpendicular falls within segment,
     *          otherwise returns distance to nearest endpoint.
     *          
     *          Common uses:
     *          - Proximity detection to path segments
     *          - Collision detection
     *          - Path tracking errors
     */
    T distance_to_segment(const Vector3<T> &seg_start, const Vector3<T> &seg_end) const;

    /**
     * @brief Offset position by bearing and pitch at given distance
     * 
     * @param[in] bearing Horizontal direction in degrees (0=north, 90=east, clockwise)
     * @param[in] pitch Vertical angle in degrees (positive=up, negative=down)
     * @param[in] distance Distance to offset in same units as position
     * 
     * @details Modifies this vector (interpreted as position) by moving it in the direction
     *          specified by bearing and pitch for the given distance.
     *          
     *          Useful for:
     *          - Computing waypoint positions at bearing/range
     *          - Extrapolating positions for navigation
     *          - Offset calculations in geographic frames
     *          
     *          Modifies this vector in place.
     *          
     * @warning Bearing and pitch must be in degrees, not radians
     * @warning Assumes NED coordinate frame (x=north, y=east, z=down)
     */
    void offset_bearing(T bearing, T pitch, T distance);

    /**
     * @brief Convert to float precision vector
     * 
     * @return Vector3<float> with components cast to float
     * 
     * @details Useful for converting double-precision vectors to single-precision
     *          for memory efficiency or API compatibility.
     *          
     * @note May lose precision if T is double or larger type
     */
    Vector3<float> tofloat() const {
        return Vector3<float>{float(x),float(y),float(z)};
    }

    /**
     * @brief Convert to double precision vector
     * 
     * @return Vector3<double> with components cast to double
     * 
     * @details Useful for high-precision calculations or converting from
     *          lower-precision types (float, int).
     */
    Vector3<double> todouble() const {
        return Vector3<double>{x,y,z};
    }

    /**
     * @brief Convert from Right-Front-Up to Front-Right-Down coordinate frame
     * 
     * @return Vector in FRD frame (ArduPilot body frame convention)
     * 
     * @details Converts between coordinate frame conventions:
     *          - RFU (Right-Front-Up): Common in some robotics systems
     *          - FRD (Front-Right-Down): ArduPilot body frame standard
     *          
     *          Also converts from:
     *          - ENU (East-North-Up): Common geographic frame
     *          - NED (North-East-Down): ArduPilot earth frame standard
     *          
     *          Transformation: (x, y, z)_RFU → (y, x, -z)_FRD
     *          
     *          Does not modify this vector.
     *          
     * @note ArduPilot uses NED for earth frame and FRD for body frame
     */
    Vector3<T> rfu_to_frd() const {
        return Vector3<T>{y,x,-z};
    }

    /**
     * @brief Compute component of position perpendicular to velocity
     * 
     * @param[in] p1 Position vector
     * @param[in] v1 Velocity vector (direction vector)
     * @return Component of p1 perpendicular to v1
     * 
     * @details Given a position p1 and a velocity/direction v1, computes the
     *          component of p1 that is perpendicular to v1 (the orthogonal component).
     *          
     *          Formula: p_perp = p1 - proj_v1(p1)
     *          
     *          Common uses:
     *          - Cross-track error in path following (distance from path)
     *          - Perpendicular distance for collision avoidance
     *          - Decomposing vectors into parallel and perpendicular components
     *          
     * @return Zero vector if p1 is parallel to v1 or if p1 is zero vector
     * 
     * @note Static method (does not require instance)
     * 
     * @warning If p1 is zero vector, always returns zero vector (caller should check)
     */
    static Vector3<T> perpendicular(const Vector3<T> &p1, const Vector3<T> &v1)
    {
        const T d = p1 * v1;
        if (::is_zero(d)) {
            return p1;
        }
        const Vector3<T> parallel = (v1 * d) / v1.length_squared();
        Vector3<T> perpendicular = p1 - parallel;

        return perpendicular;
    }

    /**
     * @brief Shortest distance from point to line segment
     * 
     * @param[in] w1 Start point of line segment
     * @param[in] w2 End point of line segment  
     * @param[in] p Point to measure distance from
     * @return Minimum distance from p to any point on segment [w1, w2]
     * 
     * @details Computes perpendicular distance if foot of perpendicular lies within segment,
     *          otherwise returns distance to nearest endpoint.
     *          
     *          Common uses:
     *          - Path tracking error measurement
     *          - Proximity detection
     *          - Collision avoidance
     *          
     * @note Static method (does not require instance)
     */
    static T closest_distance_between_line_and_point(const Vector3<T> &w1, const Vector3<T> &w2, const Vector3<T> &p);

    /**
     * @brief Find closest point on line segment to given point
     * 
     * @param[in] w1 Start point of line segment
     * @param[in] w2 End point of line segment
     * @param[in] p Point to find closest segment point to
     * @return Point on segment [w1, w2] that is closest to p
     * 
     * @details Returns the point on the line segment that minimizes distance to p.
     *          - If perpendicular from p intersects segment, returns that point
     *          - Otherwise returns nearest endpoint (w1 or w2)
     *          
     *          Common uses:
     *          - Path following: finding target point on path
     *          - Snapping to paths or trajectories
     *          - Projection onto constrained paths
     *          
     * @note Static method (does not require instance)
     */
    static Vector3<T> point_on_line_closest_to_other_point(const Vector3<T> &w1, const Vector3<T> &w2, const Vector3<T> &p);

    /**
     * @brief Find closest points between two 3D line segments
     * 
     * @param[in] seg1_start Start point of first segment
     * @param[in] seg1_end End point of first segment
     * @param[in] seg2_start Start point of second segment
     * @param[in] seg2_end End point of second segment
     * @param[out] closest_point Closest point on segment 2 to segment 1
     * 
     * @details Computes the point on segment 2 that is closest to segment 1.
     *          This is the minimum distance configuration between two line segments in 3D space.
     *          
     *          Algorithm from: http://geomalgorithms.com/a07-_distance.html
     *          
     *          Common uses:
     *          - Collision detection between path segments
     *          - Proximity analysis
     *          - Distance calculations between trajectories
     *          
     * @note Static method (does not require instance)
     * @note Only returns closest point on segment 2, not segment 1
     */
    static void segment_to_segment_closest_point(const Vector3<T>& seg1_start, const Vector3<T>& seg1_end, const Vector3<T>& seg2_start, const Vector3<T>& seg2_end, Vector3<T>& closest_point);

    /**
     * @brief Test if line segment intersects a plane
     * 
     * @param[in] seg_start Start point of line segment
     * @param[in] seg_end End point of line segment
     * @param[in] plane_normal Normal vector defining plane orientation
     * @param[in] plane_point Any point on the plane
     * @return true if segment crosses plane, false otherwise
     * 
     * @details Determines whether a 3D line segment passes through a plane.
     *          The plane is defined by a normal vector and a point on the plane.
     *          
     *          Returns true if:
     *          - Segment endpoints are on opposite sides of plane, OR
     *          - Either endpoint lies exactly on the plane
     *          
     *          Common uses:
     *          - Geofence crossing detection
     *          - Altitude limit violations
     *          - Terrain intersection checks
     *          
     * @note Static method (does not require instance)
     * @note plane_normal does not need to be normalized
     */
    static bool segment_plane_intersect(const Vector3<T>& seg_start, const Vector3<T>& seg_end, const Vector3<T>& plane_normal, const Vector3<T>& plane_point);
};

/**
 * @brief Template specialization: is_zero() for float precision
 * 
 * @return true if all components are within epsilon of zero, false otherwise
 * 
 * @details Uses epsilon-based comparison via ::is_zero() helper to handle
 *          floating-point precision issues. More robust than exact comparison
 *          for accumulation errors and numerical calculations.
 */
template<> inline bool Vector3<float>::is_zero(void) const {
    return ::is_zero(x) && ::is_zero(y) && ::is_zero(z);
}

/**
 * @brief Template specialization: is_zero() for double precision
 * 
 * @return true if all components are within epsilon of zero, false otherwise
 * 
 * @details Uses epsilon-based comparison via ::is_zero() helper to handle
 *          floating-point precision issues. More robust than exact comparison
 *          for accumulation errors and numerical calculations.
 */
template<> inline bool Vector3<double>::is_zero(void) const {
    return ::is_zero(x) && ::is_zero(y) && ::is_zero(z);
}

/**
 * @brief Inline vector operations for performance-critical code paths
 * 
 * @details The creation of temporary vector objects as return types creates significant
 *          overhead in certain hot code paths (attitude control, EKF updates, etc.).
 *          
 *          When AP_INLINE_VECTOR_OPS is defined (and not in debug builds), these operators
 *          are inlined to eliminate function call overhead and enable better compiler optimization.
 *          
 *          Operations inlined:
 *          - Cross product (operator%)
 *          - Dot product (operator*)
 *          - Arithmetic operators (*, /, +, -, +=, -=, *=, /=)
 *          
 * @note Only enabled in non-debug builds for performance optimization
 * @note Debug builds keep operators as non-inline for easier debugging
 * 
 * Source: libraries/AP_Math/vector3.h:383-456
 */
#if defined(AP_INLINE_VECTOR_OPS) && !defined(HAL_DEBUG_BUILD)

// vector cross product
template <typename T>
inline Vector3<T> Vector3<T>::operator %(const Vector3<T> &v) const
{
    return Vector3<T>(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
}

// dot product
template <typename T>
inline T Vector3<T>::operator *(const Vector3<T> &v) const
{
    return x*v.x + y*v.y + z*v.z;
}

template <typename T>
inline Vector3<T> &Vector3<T>::operator *=(const T num)
{
    x*=num; y*=num; z*=num;
    return *this;
}

template <typename T>
inline Vector3<T> &Vector3<T>::operator /=(const T num)
{
    x /= num; y /= num; z /= num;
    return *this;
}

template <typename T>
inline Vector3<T> &Vector3<T>::operator -=(const Vector3<T> &v)
{
    x -= v.x; y -= v.y; z -= v.z;
    return *this;
}

template <typename T>
inline Vector3<T> &Vector3<T>::operator +=(const Vector3<T> &v)
{
    x+=v.x; y+=v.y; z+=v.z;
    return *this;
}

template <typename T>
inline Vector3<T> Vector3<T>::operator /(const T num) const
{
    return Vector3<T>(x/num, y/num, z/num);
}

template <typename T>
inline Vector3<T> Vector3<T>::operator *(const T num) const
{
    return Vector3<T>(x*num, y*num, z*num);
}

template <typename T>
inline Vector3<T> Vector3<T>::operator -(const Vector3<T> &v) const
{
    return Vector3<T>(x-v.x, y-v.y, z-v.z);
}

template <typename T>
inline Vector3<T> Vector3<T>::operator +(const Vector3<T> &v) const
{
    return Vector3<T>(x+v.x, y+v.y, z+v.z);
}

template <typename T>
inline Vector3<T> Vector3<T>::operator -(void) const
{
    return Vector3<T>(-x,-y,-z);
}
#endif

/**
 * @typedef Vector3i
 * @brief 16-bit signed integer vector (x, y, z)
 * 
 * @details Used for integer-based calculations where floating-point is unnecessary.
 *          Range per component: -32,768 to 32,767
 */
typedef Vector3<int16_t>                Vector3i;

/**
 * @typedef Vector3ui
 * @brief 16-bit unsigned integer vector (x, y, z)
 * 
 * @details Used for non-negative integer calculations.
 *          Range per component: 0 to 65,535
 */
typedef Vector3<uint16_t>               Vector3ui;

/**
 * @typedef Vector3l
 * @brief 32-bit signed integer vector (x, y, z)
 * 
 * @details Used for large integer calculations requiring extended range.
 *          Range per component: -2,147,483,648 to 2,147,483,647
 */
typedef Vector3<int32_t>                Vector3l;

/**
 * @typedef Vector3ul
 * @brief 32-bit unsigned integer vector (x, y, z)
 * 
 * @details Used for large non-negative integer calculations.
 *          Range per component: 0 to 4,294,967,295
 */
typedef Vector3<uint32_t>               Vector3ul;

/**
 * @typedef Vector3f
 * @brief Single-precision floating-point vector (x, y, z)
 * 
 * @details Most commonly used vector type in ArduPilot flight code.
 *          Provides good balance between precision and performance.
 *          
 *          Typical precision: ~7 decimal digits
 *          Range: ±3.4E38
 *          
 *          Used for:
 *          - Position vectors (meters)
 *          - Velocity vectors (m/s)
 *          - Acceleration vectors (m/s²)
 *          - Force vectors (Newtons)
 *          - Angular rates (rad/s)
 *          - Magnetic field (milliGauss)
 */
typedef Vector3<float>                  Vector3f;

/**
 * @typedef Vector3d
 * @brief Double-precision floating-point vector (x, y, z)
 * 
 * @details Used for high-accuracy calculations requiring extended precision.
 *          More memory and computationally expensive than Vector3f.
 *          
 *          Typical precision: ~15 decimal digits
 *          Range: ±1.7E308
 *          
 *          Used for:
 *          - High-precision geographic calculations
 *          - Long-term numerical integration
 *          - Coordinate transformations requiring high accuracy
 *          
 * @note Prefer Vector3f unless double precision is specifically required
 */
typedef Vector3<double>                 Vector3d;
