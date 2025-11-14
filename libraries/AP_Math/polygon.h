/**
 * @file polygon.h
 * @brief 2D polygon geometric predicates and operations for geofencing and spatial analysis
 * 
 * @details This file provides a collection of geometric algorithms for working with 2D polygons,
 *          primarily used in geofencing applications for determining fence boundaries, inclusion/exclusion
 *          zones, and boundary breach detection.
 * 
 *          Key capabilities:
 *          - Point-in-polygon testing using ray casting algorithm
 *          - Polygon closure validation
 *          - Closest distance calculations from points/lines to polygon boundaries
 *          - Line-polygon intersection detection
 * 
 *          Coordinate system support:
 *          - Geographic coordinates (latitude/longitude in centidegrees for integer types)
 *          - Local Cartesian coordinates (meters for float types)
 *          - Template-based design supports both integer and floating-point precision
 * 
 * @note Polygon vertices should be ordered consistently (either clockwise or counterclockwise)
 *       for proper inside/outside determination
 * 
 * @note Primary use cases include:
 *       - AC_Fence geofencing breach detection
 *       - Rally point boundary calculations
 *       - Inclusion/exclusion zone validation
 *       - Mission waypoint containment checking
 * 
 * @warning Numerical precision near polygon edges can affect boundary detection results,
 *          especially with geographic coordinates spanning large distances
 * 
 * Copyright (C) Andrew Tridgell 2011
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "vector2.h"

/**
 * @brief Test if a point lies outside a polygon using ray casting algorithm
 * 
 * @details Determines whether a given point P is outside a closed polygon defined by n vertices.
 *          Uses the ray casting algorithm: casts a ray from the point to infinity and counts
 *          how many times it intersects the polygon edges. An odd number of intersections
 *          indicates the point is inside; an even number (including zero) indicates outside.
 * 
 *          Algorithm complexity: O(n) where n is the number of vertices
 * 
 *          This function supports both coordinate systems:
 *          - Geographic: Vector2<int32_t> with lat/lon in centidegrees (1e-7 degrees)
 *          - Cartesian: Vector2<float> with x/y in meters (local NED frame)
 * 
 * @tparam T Coordinate type (typically int32_t for lat/lon or float for local coordinates)
 * @param[in] P Point to test (lat/lon in centidegrees or x/y in meters depending on type T)
 * @param[in] V Array of polygon vertices ordered consistently (clockwise or counterclockwise)
 * @param[in] n Number of vertices in the polygon (must be >= 3 for a valid polygon)
 * 
 * @return true if point P is outside the polygon, false if inside or on the boundary
 * 
 * @note The polygon is assumed to be closed (last vertex connects back to first vertex)
 * @note For geographic polygons, vertex order matters for inside/outside determination
 * @note Points exactly on polygon edges may return either true or false due to floating-point precision
 * 
 * @warning Near polygon edges, numerical precision limitations may affect results,
 *          especially for geographic coordinates spanning large areas
 * 
 * @see Polygon_complete() to verify polygon is properly closed before testing
 * @see boundary_breached() in AC_Fence for fence breach detection using this function
 */
template <typename T>
bool        Polygon_outside(const Vector2<T> &P, const Vector2<T> *V, unsigned n) WARN_IF_UNUSED;

/**
 * @brief Check if a polygon is properly closed (first and last vertices match)
 * 
 * @details Verifies that a polygon is complete by checking if the last vertex in the array
 *          matches the first vertex, forming a closed loop. This is a prerequisite for
 *          many polygon operations including point-in-polygon testing and distance calculations.
 * 
 * @tparam T Coordinate type (typically int32_t for lat/lon or float for local coordinates)
 * @param[in] V Array of polygon vertices
 * @param[in] n Number of vertices in the array (must be >= 2 to form a closed polygon)
 * 
 * @return true if the polygon is complete (V[0] == V[n-1]), false otherwise
 * 
 * @note A complete polygon has at least 2 vertices where the first and last are identical,
 *       meaning at least 3 unique vertices are needed for a valid closed polygon
 * @note This function performs exact equality comparison, which works for integer types
 *       but may have precision issues with floating-point coordinates
 * 
 * @see Polygon_outside() which assumes a closed polygon
 */
template <typename T>
bool        Polygon_complete(const Vector2<T> *V, unsigned n) WARN_IF_UNUSED;

/**
 * @brief Determine if a line segment intersects a polygon and find the closest intersection point
 * 
 * @details Tests whether a line segment from p1 to p2 intersects any edge of a closed polygon
 *          defined by N vertices. If multiple intersections exist, returns the intersection
 *          point closest to p1.
 * 
 *          This function is used for:
 *          - Boundary crossing detection in geofencing
 *          - Path planning to detect polygon penetration
 *          - Determining entry/exit points for inclusion/exclusion zones
 * 
 *          Algorithm: Tests the line segment against each polygon edge using line-line
 *          intersection mathematics, tracking the intersection nearest to p1.
 * 
 * @param[in]  V            Array of polygon vertices in Cartesian coordinates (meters)
 * @param[in]  N            Number of vertices in the polygon (must be >= 3)
 * @param[in]  p1           Starting point of line segment (local frame, meters)
 * @param[in]  p2           Ending point of line segment (local frame, meters)
 * @param[out] intersection Returns the intersection point closest to p1 (only valid if function returns true)
 * 
 * @return true if the line segment intersects the polygon, false if no intersection
 * 
 * @note The polygon is assumed to be closed (edges connect consecutive vertices and last to first)
 * @note Coordinates are in local Cartesian frame (typically NED - North-East-Down)
 * @note If the line segment is entirely inside or entirely outside the polygon, returns false
 * 
 * @warning Performance is O(N) - may be expensive for polygons with many vertices if called frequently
 * 
 * @see Polygon_closest_distance_line() for distance-based boundary proximity checking
 */
bool Polygon_intersects(const Vector2f *V, unsigned N, const Vector2f &p1, const Vector2f &p2, Vector2f &intersection) WARN_IF_UNUSED;


/**
 * @brief Calculate the closest distance from a line segment to a polygon boundary
 * 
 * @details Computes the minimum distance between a line segment (from p1 to p2) and any edge
 *          of a closed polygon. This is used to determine how close a vehicle's path comes to
 *          a fence boundary or other geometric constraint.
 * 
 *          Distance calculation behavior:
 *          - Positive distance: Line segment does not cross into the polygon; value represents
 *            the minimum clearance distance to the nearest polygon edge
 *          - Negative distance: Line segment crosses into the polygon; absolute value represents
 *            the distance from p2 to the intersection point closest to p1
 * 
 *          Algorithm: For each polygon edge, computes the perpendicular distance from the line
 *          segment to that edge, tracking the minimum distance found.
 * 
 *          Common use cases:
 *          - Geofence proximity warnings (approaching boundary)
 *          - Path planning margin calculations
 *          - Boundary breach detection and measurement
 * 
 * @param[in] V  Array of polygon vertices in Cartesian coordinates (local frame, meters)
 * @param[in] N  Number of vertices in the polygon (must be >= 3)
 * @param[in] p1 Starting point of line segment (local frame, meters)
 * @param[in] p2 Ending point of line segment (local frame, meters)
 * 
 * @return Distance in meters: positive if line stays outside polygon, negative if crosses inside
 *         (negative magnitude = distance from p2 to closest intersection point to p1)
 * 
 * @note Coordinates are in local Cartesian frame (typically NED frame)
 * @note The polygon is assumed to be closed (last vertex connects to first vertex)
 * @note Algorithm complexity is O(N) where N is the number of polygon vertices
 * 
 * @warning This function uses perpendicular projection; for very short line segments relative
 *          to polygon size, results may be less intuitive
 * 
 * @see Polygon_intersects() for explicit intersection point calculation
 * @see Polygon_closest_distance_point() for point-to-polygon distance
 */
float Polygon_closest_distance_line(const Vector2f *V, unsigned N, const Vector2f &p1, const Vector2f &p2);

/**
 * @brief Calculate the closest distance from a point to a polygon boundary
 * 
 * @details Computes the minimum distance from a given point to the nearest edge of a closed
 *          polygon. This is fundamental for geofencing applications to determine how close
 *          a vehicle is to a fence boundary, enabling proximity warnings and margin enforcement.
 * 
 *          Algorithm: For each edge of the polygon, computes the perpendicular distance from
 *          the point to that edge (or distance to the nearest vertex if the perpendicular
 *          doesn't fall on the edge segment). Returns the minimum distance found across all edges.
 * 
 *          Common use cases:
 *          - Geofence proximity alerts (distance to fence boundary)
 *          - Inclusion zone margin calculations
 *          - Minimum separation enforcement
 *          - Rally point boundary distance
 * 
 * @param[in]  V       Array of polygon vertices in Cartesian coordinates (local frame, meters)
 * @param[in]  N       Number of vertices in the polygon (must be >= 3 for valid polygon)
 * @param[in]  p       Point to measure distance from (local frame, meters)
 * @param[out] closest Returns the minimum distance in meters from point to polygon boundary
 *                     (only valid if function returns true)
 * 
 * @return true if distance calculation successful, false if polygon is invalid or calculation fails
 * 
 * @note Coordinates are in local Cartesian frame (typically NED frame - North-East-Down)
 * @note The polygon is assumed to be closed (last vertex implicitly connects to first vertex)
 * @note Distance is always positive; use Polygon_outside() to determine if point is inside or outside
 * @note Algorithm complexity is O(N) where N is the number of polygon vertices
 * 
 * @warning For points inside the polygon, returns distance to boundary (not negative distance)
 * @warning Function may return false for degenerate polygons (< 3 vertices or collinear points)
 * 
 * @see Polygon_outside() to determine if point is inside or outside the polygon
 * @see Polygon_closest_distance_line() for line segment distance calculations
 */
 bool Polygon_closest_distance_point(const Vector2f *V, unsigned N, const Vector2f &p, float& closest);
 