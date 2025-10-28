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
* 2D Vector Classes
* By Bill Perone (billperone@yahoo.com)
* Original: 9-16-2002
* Revised: 19-11-2003
*          18-12-2003
*          06-06-2004
*
* Copyright 2003, This code is provided "as is" and you can use it freely as long as
* credit is given to Bill Perone in the application it is used in
****************************************/

/**
 * @file vector2.h
 * @brief 2D vector class for planar geometry and horizontal navigation
 * 
 * @details This file provides a template-based 2D vector class used extensively
 *          throughout ArduPilot for horizontal position, velocity, and 2D geometric
 *          calculations. The Vector2 class supports standard vector algebra operations
 *          including addition, subtraction, dot product, cross product (z-component),
 *          magnitude, normalization, rotation, and various geometric utility functions.
 *          
 *          Common uses: horizontal position/velocity in NED frame (North-East plane),
 *          2D waypoint navigation, latitude/longitude offsets, wind vectors, and
 *          2D geometric computations.
 * 
 * @note Coordinate frames: Typically used for horizontal plane operations (NE in NED frame,
 *       xy in body frame). The 2D cross product returns the scalar z-component.
 * 
 * @see Vector3 for 3D vector operations
 * @see polygon.h for polygon geometry using Vector2
 */

#pragma once

#ifndef MATH_CHECK_INDEXES
#define MATH_CHECK_INDEXES 0
#endif
#if MATH_CHECK_INDEXES
#include <assert.h>
#endif

#include <cmath>
#include <float.h>
#include <AP_Common/AP_Common.h>
#include "ftype.h"

/**
 * @class Vector2
 * @brief 2D vector template class with Cartesian coordinates (x, y)
 * 
 * @details Provides comprehensive 2D vector algebra operations including:
 *          - Arithmetic: addition, subtraction, scalar multiplication/division
 *          - Products: dot product, 2D cross product (z-component scalar)
 *          - Geometric: magnitude, normalization, angle calculation, rotation
 *          - Utilities: line segment operations, intersections, projections
 *          
 *          The Vector2 class is extensively used for horizontal navigation calculations,
 *          2D waypoint operations, wind estimation, and planar geometry. It operates
 *          in the horizontal plane of various coordinate frames (typically NE in NED
 *          frame or xy in body frame).
 * 
 * @tparam T Numeric type (typically float or double)
 * 
 * @note The 2D cross product operator% returns a scalar representing the z-component
 *       of the cross product: x*v.y - y*v.x. Positive if v is counterclockwise from this.
 * 
 * @note Explicit float and double instantiations are provided in vector2.cpp
 * 
 * @warning Many geometric functions assume the vectors represent positions or directions
 *          in a Cartesian coordinate system with consistent units.
 */
template <typename T>
struct Vector2
{
    T x, y;  ///< Cartesian coordinates (x, y) representing 2D position or direction

    /**
     * @brief Default constructor - initializes vector to zero (0, 0)
     */
    constexpr Vector2()
        : x(0)
        , y(0) {}

    /**
     * @brief Constructor setting x and y coordinates
     * 
     * @param[in] x0 X-coordinate value
     * @param[in] y0 Y-coordinate value
     */
    constexpr Vector2(const T x0, const T y0)
        : x(x0)
        , y(y0) {}

    /**
     * @brief Equality test with epsilon tolerance
     * 
     * @param[in] v Vector to compare with
     * @return true if vectors are equal within tolerance, false otherwise
     * 
     * @note Uses epsilon comparison for floating-point types
     */
    bool operator ==(const Vector2<T> &v) const;

    /**
     * @brief Inequality test with epsilon tolerance
     * 
     * @param[in] v Vector to compare with
     * @return true if vectors are not equal within tolerance, false otherwise
     */
    bool operator !=(const Vector2<T> &v) const;

    /**
     * @brief Unary negation - returns vector with negated components
     * 
     * @return Negated vector (-x, -y)
     */
    Vector2<T> operator -(void) const;

    /**
     * @brief Vector addition
     * 
     * @param[in] v Vector to add
     * @return Sum vector (this + v)
     */
    Vector2<T> operator +(const Vector2<T> &v) const;

    /**
     * @brief Vector subtraction
     * 
     * @param[in] v Vector to subtract
     * @return Difference vector (this - v)
     */
    Vector2<T> operator -(const Vector2<T> &v) const;

    /**
     * @brief Scalar multiplication - uniform scaling
     * 
     * @param[in] num Scalar multiplier
     * @return Scaled vector (x*num, y*num)
     */
    Vector2<T> operator *(const T num) const;

    /**
     * @brief Scalar division - uniform scaling
     * 
     * @param[in] num Scalar divisor
     * @return Scaled vector (x/num, y/num)
     * 
     * @warning Division by zero is not checked
     */
    Vector2<T> operator  /(const T num) const;

    /**
     * @brief Vector addition assignment
     * 
     * @param[in] v Vector to add
     * @return Reference to this vector after addition
     */
    Vector2<T> &operator +=(const Vector2<T> &v);

    /**
     * @brief Vector subtraction assignment
     * 
     * @param[in] v Vector to subtract
     * @return Reference to this vector after subtraction
     */
    Vector2<T> &operator -=(const Vector2<T> &v);

    /**
     * @brief Scalar multiplication assignment
     * 
     * @param[in] num Scalar multiplier
     * @return Reference to this vector after scaling
     */
    Vector2<T> &operator *=(const T num);

    /**
     * @brief Scalar division assignment
     * 
     * @param[in] num Scalar divisor
     * @return Reference to this vector after scaling
     * 
     * @warning Division by zero is not checked
     */
    Vector2<T> &operator /=(const T num);

    /**
     * @brief Dot product (scalar product)
     * 
     * @param[in] v Vector to dot product with
     * @return Scalar result: x*v.x + y*v.y
     * 
     * @note Measures projection of one vector onto another
     */
    T operator *(const Vector2<T> &v) const;

    /**
     * @brief Dot product (named version for clarity)
     * 
     * @param[in] v Vector to dot product with
     * @return Scalar result: x*v.x + y*v.y
     * 
     * @note Same as operator*, but with more intuitive name
     */
    T dot(const Vector2<T> &v) const {
        return *this * v;
    }

    /**
     * @brief 2D cross product - returns z-component scalar
     * 
     * @param[in] v Vector to cross product with
     * @return Scalar z-component: x*v.y - y*v.x
     * 
     * @note Positive result means v is counterclockwise from this vector
     * @note In 2D, cross product produces a scalar (z-component of 3D cross product)
     * @note Useful for determining relative orientation and signed area calculations
     */
    T operator %(const Vector2<T> &v) const;

    /**
     * @brief Computes angle between this vector and another vector
     * 
     * @param[in] v2 Second vector to compute angle with
     * @return Angle in radians [0, π]
     * 
     * @note Returns 0 if vectors are parallel (same direction)
     * @note Returns π if vectors are antiparallel (opposite directions)
     * @note Always returns positive angle (unsigned)
     * 
     * @warning Returns shortest angle between vectors, range [0, π]
     */
    T angle(const Vector2<T> &v2) const;

    /**
     * @brief Computes angle from origin to this vector (atan2)
     * 
     * @return Angle in radians [-π, π] from positive x-axis
     * 
     * @note Uses atan2(y, x) - angle from unit vector (1,0)
     * @note A vector (1,1) has angle +π/4 (45 degrees)
     * @note A vector (-1,0) has angle π (180 degrees)
     * @note Useful for heading/bearing calculations
     * 
     * @warning Returns angle in range [-π, π], not [0, 2π]
     */
    T angle(void) const;

    /**
     * @brief Check if any elements are NaN (Not a Number)
     * 
     * @return true if x or y is NaN, false otherwise
     * 
     * @note Used for validity checking in calculations
     */
    bool is_nan(void) const WARN_IF_UNUSED;

    /**
     * @brief Check if any elements are infinity
     * 
     * @return true if x or y is infinite, false otherwise
     * 
     * @note Used for overflow detection and validity checking
     */
    bool is_inf(void) const WARN_IF_UNUSED;

    /**
     * @brief Check if vector is zero (0, 0)
     * 
     * @return true if both x and y are zero, false otherwise
     * 
     * @note For floating-point types, uses epsilon comparison (see specializations)
     * @note For integer types, checks exact equality to zero
     */
    bool is_zero(void) const WARN_IF_UNUSED {
        return x == 0 && y == 0;
    }

    /**
     * @brief Array-style access to vector components (mutable)
     * 
     * @param[in] i Index (0 for x, 1 for y)
     * @return Reference to component at index i
     * 
     * @note Allows vector to be used as array: v[0] accesses x, v[1] accesses y
     * @warning Index bounds checking only enabled if MATH_CHECK_INDEXES is defined
     */
    T & operator[](uint8_t i) {
        T *_v = &x;
#if MATH_CHECK_INDEXES
        assert(i >= 0 && i < 2);
#endif
        return _v[i];
    }

    /**
     * @brief Array-style access to vector components (const)
     * 
     * @param[in] i Index (0 for x, 1 for y)
     * @return Const reference to component at index i
     * 
     * @note Allows vector to be used as array: v[0] accesses x, v[1] accesses y
     * @warning Index bounds checking only enabled if MATH_CHECK_INDEXES is defined
     */
    const T & operator[](uint8_t i) const {
        const T *_v = &x;
#if MATH_CHECK_INDEXES
        assert(i >= 0 && i < 2);
#endif
        return _v[i];
    }
    
    /**
     * @brief Set vector to zero (0, 0)
     * 
     * @note Modifies this vector in place
     */
    void zero()
    {
        x = y = 0;
    }

    /**
     * @brief Compute magnitude squared (avoids sqrt for performance)
     * 
     * @return Length squared: x² + y²
     * 
     * @note More efficient than length() when only relative magnitudes are needed
     * @note Useful for distance comparisons without computing square root
     */
    T length_squared() const;

    /**
     * @brief Compute Euclidean magnitude (length) of vector
     * 
     * @return Length: sqrt(x² + y²)
     * 
     * @note Returns distance from origin to point (x, y)
     * @see length_squared() for more efficient relative comparisons
     */
    T length(void) const;

    /**
     * @brief Limit vector magnitude to maximum length
     * 
     * @param[in] max_length Maximum allowed magnitude
     * @return true if vector was limited (exceeded max_length), false otherwise
     * 
     * @note Modifies this vector in place if it exceeds max_length
     * @note Preserves direction but scales magnitude to max_length
     */
    bool limit_length(T max_length);

    /**
     * @brief Normalize vector to unit length (modifies in place)
     * 
     * @note Converts vector to unit vector: magnitude becomes 1.0
     * @note Direction is preserved, only magnitude changes
     * 
     * @warning If vector length is zero, results in zero vector (0, 0)
     * @warning Does not check for zero length before normalization
     */
    void normalize();

    /**
     * @brief Return normalized copy of vector (unit vector)
     * 
     * @return Unit vector in same direction as this vector
     * 
     * @note Does not modify this vector
     * @note Returns vector with magnitude 1.0 in same direction
     * 
     * @warning If vector length is zero, returns zero vector (0, 0)
     */
    Vector2<T> normalized() const;

    /**
     * @brief Reflect this vector about normal vector n
     * 
     * @param[in] n Normal vector defining reflection plane (should be normalized)
     * 
     * @note Modifies this vector in place
     * @note Computes reflection as: v' = v - 2*(v·n)*n
     */
    void reflect(const Vector2<T> &n);

    /**
     * @brief Project this vector onto vector v (modifies in place)
     * 
     * @param[in] v Vector to project onto
     * 
     * @note Modifies this vector to be its projection onto v
     * @note Result is component of this vector in direction of v
     * @note Formula: proj = (this·v / |v|²) * v
     */
    void project(const Vector2<T> &v);

    /**
     * @brief Return projection of this vector onto v
     * 
     * @param[in] v Vector to project onto
     * @return Projected vector (component of this in direction of v)
     * 
     * @note Does not modify this vector
     * @note Returns component of this vector in direction of v
     */
    Vector2<T> projected(const Vector2<T> &v) const;

    /**
     * @brief Adjust position by bearing and distance (navigation offset)
     * 
     * @param[in] bearing Bearing angle in degrees (0=North, 90=East)
     * @param[in] distance Distance to offset in same units as vector
     * 
     * @note Modifies this vector in place
     * @note Commonly used for waypoint offset calculations in NE frame
     * @note Bearing follows aviation convention: 0°=North, 90°=East
     */
    void offset_bearing(T bearing, T distance);

    /**
     * @brief Rotate vector by angle (positive counterclockwise)
     * 
     * @param[in] angle_rad Rotation angle in radians (positive = counterclockwise)
     * 
     * @note Modifies this vector in place
     * @note Applies 2D rotation matrix transformation
     * @note Positive angles rotate counterclockwise in standard x-y coordinates
     * 
     * @warning Ensure angle is in radians, not degrees
     */
    void rotate(T angle_rad);

    /**
     * @brief Convert vector to float precision
     * 
     * @return Vector2<float> with components cast to float
     * 
     * @note Useful for converting double-precision vectors to single precision
     */
    Vector2<float> tofloat() const {
        return Vector2<float>{float(x),float(y)};
    }

    /**
     * @brief Convert vector to double precision
     * 
     * @return Vector2<double> with components cast to double
     * 
     * @note Useful for converting single-precision vectors to double precision
     */
    Vector2<double> todouble() const {
        return Vector2<double>{x,y};
    }
    
    /**
     * @brief Compute perpendicular vector maximizing distance from position
     * 
     * @param[in] pos_delta Position difference vector
     * @param[in] v1 Velocity/direction vector
     * @return Vector perpendicular to v1 that maximizes distance from pos_delta
     * 
     * @note Static utility function for avoidance/separation calculations
     * @note Returns vector perpendicular to v1 (rotated 90°)
     */
    static Vector2<T> perpendicular(const Vector2<T> &pos_delta, const Vector2<T> &v1);

    /**
     * @brief Find closest point on line segment to a given point
     * 
     * @param[in] p Point to find closest approach to
     * @param[in] v Start point of line segment
     * @param[in] w End point of line segment
     * @return Point on segment [v,w] closest to p
     * 
     * @note If projection falls outside segment, returns nearest endpoint
     * @note Useful for path following and proximity detection
     * 
     * Source: http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
     */
    static Vector2<T> closest_point(const Vector2<T> &p, const Vector2<T> &v, const Vector2<T> &w);

    /**
     * @brief Find closest point on line segment from origin to w
     * 
     * @param[in] p Point to find closest approach to
     * @param[in] w End point of line segment (segment starts at origin)
     * @return Point on segment [0,w] closest to p
     * 
     * @note Simplified version of closest_point where v=(0,0)
     * @note More efficient for segments starting at origin
     */
    static Vector2<T> closest_point(const Vector2<T> &p, const Vector2<T> &w);

    /**
     * @brief Minimum distance squared from point to line segment
     * 
     * @param[in] w1 Start point of line segment
     * @param[in] w2 End point of line segment
     * @param[in] p Point to measure distance from
     * @return Squared distance from p to closest point on segment [w1,w2]
     * 
     * @note Returns squared distance (avoids sqrt for performance)
     * @note Useful for proximity checks and collision detection
     */
    static T closest_distance_between_line_and_point_squared(const Vector2<T> &w1,
                                                                 const Vector2<T> &w2,
                                                                 const Vector2<T> &p);

    /**
     * @brief Minimum distance from point to line segment
     * 
     * @param[in] w1 Start point of line segment
     * @param[in] w2 End point of line segment
     * @param[in] p Point to measure distance from
     * @return Distance from p to closest point on segment [w1,w2]
     * 
     * @note Uses sqrt - consider squared version for comparisons
     * @see closest_distance_between_line_and_point_squared()
     */
    static T closest_distance_between_line_and_point(const Vector2<T> &w1,
                                                         const Vector2<T> &w2,
                                                         const Vector2<T> &p);

    /**
     * @brief Minimum distance squared between two line segments
     * 
     * @param[in] a1 Start point of first segment
     * @param[in] a2 End point of first segment
     * @param[in] b1 Start point of second segment
     * @param[in] b2 End point of second segment
     * @return Squared distance between closest points on the two segments
     * 
     * @note Returns squared distance for performance
     * @note Useful for separation checks between path segments
     */
    static T closest_distance_between_lines_squared(const Vector2<T> &a1,
                                                        const Vector2<T> &a2,
                                                        const Vector2<T> &b1,
                                                        const Vector2<T> &b2);

    /**
     * @brief Minimum distance squared from point to radial line from origin
     * 
     * @param[in] w Direction vector defining radial from origin (line segment [0,w])
     * @param[in] p Point to measure distance from
     * @return Squared distance from p to closest point on radial [0,w]
     * 
     * @note Radial is line segment from origin (0,0) in direction of w
     * @note Returns squared distance for performance
     */
    static T closest_distance_between_radial_and_point_squared(const Vector2<T> &w,
                                                                   const Vector2<T> &p);

    /**
     * @brief Minimum distance from point to radial line from origin
     * 
     * @param[in] w Direction vector defining radial from origin (line segment [0,w])
     * @param[in] p Point to measure distance from
     * @return Distance from p to closest point on radial [0,w]
     * 
     * @note Radial is line segment from origin (0,0) in direction of w
     * @see closest_distance_between_radial_and_point_squared() for faster comparisons
     */
    static T closest_distance_between_radial_and_point(const Vector2<T> &w,
                                                           const Vector2<T> &p);

    /**
     * @brief Find intersection point of two line segments
     * 
     * @param[in]  seg1_start Start point of first segment
     * @param[in]  seg1_end   End point of first segment
     * @param[in]  seg2_start Start point of second segment
     * @param[in]  seg2_end   End point of second segment
     * @param[out] intersection Intersection point if segments intersect
     * @return true if segments intersect, false if parallel or non-intersecting
     * 
     * @note Only returns true if intersection point lies on both segments
     * @note If segments are parallel or don't intersect, intersection is undefined
     * @note Useful for path crossing detection and collision avoidance
     */
    static bool segment_intersection(const Vector2<T>& seg1_start, const Vector2<T>& seg1_end, const Vector2<T>& seg2_start, const Vector2<T>& seg2_end, Vector2<T>& intersection) WARN_IF_UNUSED;

    /**
     * @brief Find intersection between line segment and circle
     * 
     * @param[in]  seg_start     Start point of line segment
     * @param[in]  seg_end       End point of line segment
     * @param[in]  circle_center Center of circle
     * @param[in]  radius        Circle radius
     * @param[out] intersection  Intersection point closest to seg_start
     * @return true if segment intersects circle, false otherwise
     * 
     * @note If multiple intersections exist, returns closest to seg_start
     * @note Useful for geofence breach detection and obstacle avoidance
     * @note If no intersection, intersection parameter is undefined
     */
    static bool circle_segment_intersection(const Vector2<T>& seg_start, const Vector2<T>& seg_end, const Vector2<T>& circle_center, T radius, Vector2<T>& intersection) WARN_IF_UNUSED;

    /**
     * @brief Check if point lies on line segment
     * 
     * @param[in] point     Point to test
     * @param[in] seg_start Start of line segment
     * @param[in] seg_end   End of line segment
     * @return true if point lies on segment [seg_start, seg_end], false otherwise
     * 
     * @note Checks both collinearity (same slope) and bounding box containment
     * @note Uses epsilon tolerance (FLT_EPSILON) for floating-point comparison
     * @note Useful for waypoint validation and path verification
     */
    static bool point_on_segment(const Vector2<T>& point,
                                 const Vector2<T>& seg_start,
                                 const Vector2<T>& seg_end) WARN_IF_UNUSED {
        const T expected_run = seg_end.x-seg_start.x;
        const T intersection_run = point.x-seg_start.x;
        // check slopes are identical:
        if (::is_zero(expected_run)) {
            if (fabsF(intersection_run) > FLT_EPSILON) {
                return false;
            }
        } else {
            const T expected_slope = (seg_end.y-seg_start.y)/expected_run;
            const T intersection_slope = (point.y-seg_start.y)/intersection_run;
            if (fabsF(expected_slope - intersection_slope) > FLT_EPSILON) {
                return false;
            }
        }
        // check for presence in bounding box
        if (seg_start.x < seg_end.x) {
            if (point.x < seg_start.x || point.x > seg_end.x) {
                return false;
            }
        } else {
            if (point.x < seg_end.x || point.x > seg_start.x) {
                return false;
            }
        }
        if (seg_start.y < seg_end.y) {
            if (point.y < seg_start.y || point.y > seg_end.y) {
                return false;
            }
        } else {
            if (point.y < seg_end.y || point.y > seg_start.y) {
                return false;
            }
        }
        return true;
    }
};

/**
 * @brief Template specialization: is_zero() for float with epsilon tolerance
 * 
 * @return true if both x and y are within epsilon of zero, false otherwise
 * 
 * @note Uses ::is_zero() helper function with floating-point epsilon comparison
 * @note More appropriate for floating-point vectors than exact equality check
 */
template<> inline bool Vector2<float>::is_zero(void) const {
    return ::is_zero(x) && ::is_zero(y);
}

/**
 * @brief Template specialization: is_zero() for double with epsilon tolerance
 * 
 * @return true if both x and y are within epsilon of zero, false otherwise
 * 
 * @note Uses ::is_zero() helper function with floating-point epsilon comparison
 * @note More appropriate for floating-point vectors than exact equality check
 */
template<> inline bool Vector2<double>::is_zero(void) const {
    return ::is_zero(x) && ::is_zero(y);
}

// Common Vector2 type aliases for convenience
typedef Vector2<int16_t>        Vector2i;    ///< 2D vector with 16-bit signed integer components
typedef Vector2<uint16_t>       Vector2ui;   ///< 2D vector with 16-bit unsigned integer components
typedef Vector2<int32_t>        Vector2l;    ///< 2D vector with 32-bit signed integer components
typedef Vector2<uint32_t>       Vector2ul;   ///< 2D vector with 32-bit unsigned integer components
typedef Vector2<float>          Vector2f;    ///< 2D vector with single-precision float components (most common)
typedef Vector2<double>         Vector2d;    ///< 2D vector with double-precision float components
