/**
 * @file location.h
 * @brief Geographic location utilities for latitude/longitude/altitude operations
 * 
 * @details This file provides core utilities for working with geographic locations using
 *          WGS84 ellipsoid coordinates. Functions support distance calculations, bearing
 *          computations, and coordinate system transformations between geodetic (lat/lon/height)
 *          and Earth Centered, Earth Fixed (ECEF) Cartesian coordinates.
 * 
 *          Coordinate System: WGS84 ellipsoid
 *          - Latitude/Longitude: Can be expressed in degrees (float/double) or centidegrees (int32_t)
 *          - Altitude: Relative to home, AMSL (Above Mean Sea Level), or terrain
 *          - ECEF: Cartesian coordinates with origin at Earth's center
 * 
 *          Use Cases:
 *          - Waypoint navigation and mission planning
 *          - Distance and bearing calculations between positions
 *          - Great-circle distance computations
 *          - Geographic coordinate validation
 *          - High-precision coordinate transformations for navigation
 * 
 * @note Precision Considerations:
 *       - Use centidegrees (int32_t) for storage to avoid floating-point precision loss
 *       - Use degrees (float/double) for calculations
 *       - For high-accuracy ECEF ↔ LLH conversions, see location_double.cpp
 * 
 * @note Fukushima Algorithm:
 *       The WGS84 ECEF/LLH transformation functions use the Fukushima iterative algorithm
 *       for accurate geodetic coordinate conversion. This is more precise than simple
 *       spherical approximations.
 * 
 * @warning Geographic coordinate ranges:
 *          - Latitude: [-90°, +90°] for degrees, [-90e7, +90e7] for centidegrees
 *          - Longitude: [-180°, +180°] for degrees, [-180e7, +180e7] for centidegrees
 * 
 * @see definitions.h for WGS84 constants (RADIUS_OF_EARTH, etc.)
 * @see AP_Math.h for angle wrapping and mathematical utilities
 * @see location.cpp for function implementations
 * @see location_double.cpp for double-precision ECEF/LLH conversions
 */

#pragma once

#include <inttypes.h>

#include "vector2.h"
#include "vector3.h"

/*
 * LOCATION
 */

/**
 * @brief Computes straight-line Euclidean distance in the horizontal plane between two positions
 * 
 * @details Calculates the 2D distance between two points using the Pythagorean theorem.
 *          This is a simple Cartesian distance calculation, not a great-circle distance.
 *          Suitable for local planar approximations where curvature can be neglected.
 * 
 * @tparam T Numeric type (typically float or double)
 * @param[in] origin Starting position as 2D vector (x, y coordinates)
 * @param[in] destination Ending position as 2D vector (x, y coordinates)
 * @return Straight-line distance between the two points in same units as input
 * 
 * @note Unit Consistency: Input units (e.g., meters, centimeters) must match between
 *       origin and destination. No unit conversion is performed. Output is in same units.
 * 
 * @note Coordinate Frame: Works in any 2D Cartesian coordinate system. Typically used for:
 *       - Local NE (North-East) offsets in meters
 *       - Body frame xy positions
 *       - Planar approximations of geographic positions
 * 
 * @warning This is NOT a great-circle distance calculation. For geographic coordinates
 *          (lat/lon), use appropriate spherical distance functions instead.
 * 
 * @note Performance: Template instantiated at compile time, inline for zero overhead
 */
template <typename T>
float get_horizontal_distance(const Vector2<T> &origin, const Vector2<T> &destination)
{
    return (destination - origin).length();
}

/**
 * @brief Calculates bearing angle in radians from origin to destination
 * 
 * @details Computes the angle (bearing) from the origin point to the destination point
 *          in a 2D plane using atan2. The bearing is measured clockwise from the positive
 *          x-axis (typically North in NED frame) and wrapped to [0, 2π] range.
 * 
 * @param[in] origin Starting position as 2D vector (typically North-East in meters)
 * @param[in] destination Ending position as 2D vector (typically North-East in meters)
 * @return Bearing angle in radians, range [0, 2π], 0 = North, π/2 = East, π = South, 3π/2 = West
 * 
 * @note Coordinate Frame Convention:
 *       - x-axis = North (0 radians)
 *       - y-axis = East (π/2 radians)
 *       - Positive rotation is clockwise when viewed from above (NED frame standard)
 * 
 * @note Units: Input vectors must be in same units (typically meters). Output is always in radians.
 * 
 * @note Range: Result is wrapped to [0, 2π] using wrap_2PI() function
 * 
 * @warning For geographic lat/lon coordinates, convert to local NE offsets first
 *          using appropriate projection (not simple subtraction)
 * 
 * @see get_bearing_cd() for centidegree output
 * @see wrap_2PI() for angle wrapping implementation
 */
float        get_bearing_rad(const Vector2f &origin, const Vector2f &destination);

/**
 * @brief Calculates bearing angle in centidegrees from origin to destination
 * 
 * @details Computes the bearing from origin to destination and converts to centidegrees.
 *          Centidegrees are used throughout ArduPilot for integer angle representation
 *          (1 degree = 100 centidegrees). This avoids floating-point precision issues
 *          for parameter storage and telemetry.
 * 
 * @param[in] origin Starting position as 2D vector (typically North-East in meters)
 * @param[in] destination Ending position as 2D vector (typically North-East in meters)
 * @return Bearing angle in centidegrees, range [0, 36000], 0 = North, 9000 = East, 18000 = South
 * 
 * @note Centidegrees: 1 degree = 100 centidegrees, full circle = 36000 centidegrees
 * 
 * @note Precision: Centidegree representation provides 0.01° resolution, sufficient
 *       for most navigation and control applications
 * 
 * @note Implementation: Calls get_bearing_rad() and converts using rad_to_cd()
 * 
 * @note Range: Result is in [0, 36000) centidegrees (equivalent to [0°, 360°))
 * 
 * @see get_bearing_rad() for radian output
 * @see rad_to_cd() for conversion function
 */
float        get_bearing_cd(const Vector2f &origin, const Vector2f &destination);

/**
 * @brief Converts WGS84 geodetic coordinates (lat, lon, height) to ECEF Cartesian coordinates
 * 
 * @details Transforms from geodetic coordinates on the WGS84 ellipsoid to Earth Centered,
 *          Earth Fixed (ECEF) Cartesian coordinates. ECEF is a 3D Cartesian coordinate system
 *          with origin at Earth's center of mass, X-axis pointing to 0° lat/0° lon,
 *          Z-axis pointing to North Pole, and Y-axis completing right-handed system.
 * 
 *          The conversion accounts for Earth's ellipsoidal shape using WGS84 parameters:
 *          - Semi-major axis (equatorial radius): 6378137.0 m
 *          - Flattening: 1/298.257223563
 * 
 * @param[in] llh Geodetic coordinates as Vector3d:
 *                - x = latitude in degrees [-90, +90]
 *                - y = longitude in degrees [-180, +180]
 *                - z = height above WGS84 ellipsoid in meters
 * @param[out] ecef ECEF Cartesian coordinates as Vector3d:
 *                  - x = X coordinate in meters (towards 0°N, 0°E)
 *                  - y = Y coordinate in meters (towards 0°N, 90°E)
 *                  - z = Z coordinate in meters (towards North Pole)
 * 
 * @note Coordinate System: ECEF (Earth Centered, Earth Fixed)
 *       - Origin: Earth's center of mass
 *       - X-axis: Intersection of equatorial plane and prime meridian
 *       - Y-axis: Intersection of equatorial plane and 90°E meridian
 *       - Z-axis: North Pole direction
 *       - Right-handed coordinate system
 * 
 * @note Units:
 *       - Input latitude/longitude: degrees
 *       - Input/output height: meters
 *       - Output X/Y/Z: meters
 * 
 * @note Precision: Uses double-precision (Vector3d) for high-accuracy transformations
 *       required in navigation and surveying applications
 * 
 * @warning Height is above the WGS84 ellipsoid, NOT above sea level (geoid).
 *          For altitude above mean sea level, geoid separation must be considered.
 * 
 * @see wgsecef2llh() for inverse transformation
 * @see location_double.cpp for implementation using Fukushima algorithm
 */
void        wgsllh2ecef(const Vector3d &llh, Vector3d &ecef);

/**
 * @brief Converts WGS84 ECEF Cartesian coordinates to geodetic coordinates (lat, lon, height)
 * 
 * @details Transforms from Earth Centered, Earth Fixed (ECEF) Cartesian coordinates to
 *          geodetic coordinates on the WGS84 ellipsoid. Uses the Fukushima iterative
 *          algorithm for accurate conversion, which is more precise than closed-form
 *          approximations and handles all latitudes including poles.
 * 
 *          The Fukushima method provides:
 *          - Fast convergence (typically 2-3 iterations)
 *          - High accuracy (sub-millimeter precision)
 *          - Numerical stability at poles and near-zero radius
 * 
 * @param[in] ecef ECEF Cartesian coordinates as Vector3d:
 *                 - x = X coordinate in meters (towards 0°N, 0°E)
 *                 - y = Y coordinate in meters (towards 0°N, 90°E)
 *                 - z = Z coordinate in meters (towards North Pole)
 * @param[out] llh Geodetic coordinates as Vector3d:
 *                 - x = latitude in degrees [-90, +90]
 *                 - y = longitude in degrees [-180, +180]
 *                 - z = height above WGS84 ellipsoid in meters
 * 
 * @note Algorithm: Fukushima, T. (2006). "Fast transform from geocentric to geodetic coordinates"
 *       Journal of Geodesy, 79(12), 689-693. DOI: 10.1007/s00190-005-0487-z
 * 
 * @note Coordinate System: ECEF (Earth Centered, Earth Fixed) - see wgsllh2ecef() for details
 * 
 * @note Units:
 *       - Input X/Y/Z: meters
 *       - Output latitude/longitude: degrees
 *       - Output height: meters
 * 
 * @note Precision: Uses double-precision (Vector3d) throughout iteration for accuracy
 * 
 * @note Special Cases:
 *       - Handles exact poles (X=0, Y=0) correctly
 *       - Handles points at Earth's center (X=Y=Z=0) with zero lat/lon
 * 
 * @warning Height is above the WGS84 ellipsoid reference surface, NOT mean sea level.
 *          Geoid undulation (N) must be subtracted to get orthometric height: h_MSL = h_ellipsoid - N
 * 
 * @see wgsllh2ecef() for forward transformation
 * @see location_double.cpp for Fukushima algorithm implementation
 */
void        wgsecef2llh(const Vector3d &ecef, Vector3d &llh);

/**
 * @brief Validates that latitude value is within valid geographic range (float version)
 * 
 * @details Checks if a latitude value in degrees falls within the valid range for
 *          WGS84 coordinates. Valid latitudes range from -90° (South Pole) to +90° (North Pole).
 * 
 * @param[in] lat Latitude in degrees
 * @return true if latitude is in valid range [-90.0, +90.0], false otherwise
 * 
 * @note Valid Range: [-90°, +90°] where:
 *       - -90° = South Pole
 *       - 0° = Equator
 *       - +90° = North Pole
 * 
 * @note Units: Degrees (not radians or centidegrees)
 * 
 * @note Implementation: Uses fabsf(lat) <= 90 check
 * 
 * @warning Return value should not be ignored - marked with WARN_IF_UNUSED attribute.
 *          Always check validity before using latitude in calculations.
 * 
 * @see check_lat(int32_t) for centidegree version
 * @see check_latlng() for combined lat/lon validation
 */
bool        check_lat(float lat) WARN_IF_UNUSED;

/**
 * @brief Validates that longitude value is within valid geographic range (float version)
 * 
 * @details Checks if a longitude value in degrees falls within the valid range for
 *          WGS84 coordinates. Valid longitudes range from -180° to +180°.
 *          Note: -180° and +180° represent the same meridian (International Date Line).
 * 
 * @param[in] lng Longitude in degrees
 * @return true if longitude is in valid range [-180.0, +180.0], false otherwise
 * 
 * @note Valid Range: [-180°, +180°] where:
 *       - -180° = International Date Line (westward)
 *       - 0° = Prime Meridian (Greenwich)
 *       - +180° = International Date Line (eastward, same as -180°)
 * 
 * @note Units: Degrees (not radians or centidegrees)
 * 
 * @note Implementation: Uses fabsf(lng) <= 180 check
 * 
 * @note Convention: ArduPilot uses the [-180°, +180°] convention rather than [0°, 360°]
 * 
 * @warning Return value should not be ignored - marked with WARN_IF_UNUSED attribute.
 *          Always check validity before using longitude in calculations.
 * 
 * @see check_lng(int32_t) for centidegree version
 * @see check_latlng() for combined lat/lon validation
 */
bool        check_lng(float lng) WARN_IF_UNUSED;

/**
 * @brief Validates that latitude value is within valid geographic range (centidegree version)
 * 
 * @details Checks if a latitude value in centidegrees falls within the valid range for
 *          WGS84 coordinates. Centidegrees are integer representations where 1 degree = 100 centidegrees,
 *          used throughout ArduPilot to avoid floating-point precision issues in parameter storage.
 * 
 * @param[in] lat Latitude in centidegrees (degrees × 10^7 for high-precision integer representation)
 * @return true if latitude is in valid range [-90e7, +90e7], false otherwise
 * 
 * @note Valid Range: [-90e7, +90e7] centidegrees = [-900000000, +900000000] where:
 *       - -90e7 = South Pole
 *       - 0 = Equator  
 *       - +90e7 = North Pole
 * 
 * @note Units: Centidegrees × 10^7 for integer storage
 *       - 1 degree = 1e7 centidegrees (10,000,000)
 *       - 0.01° resolution = 1e5 centidegrees
 *       - Example: 45.123456° = 451234560 centidegrees
 * 
 * @note Precision: Provides ~1.1 cm resolution at equator, suitable for all navigation applications
 * 
 * @note Implementation: Uses labs(lat) <= 90*1e7 check
 * 
 * @warning Return value should not be ignored - marked with WARN_IF_UNUSED attribute.
 * 
 * @see check_lat(float) for degree version
 * @see check_latlng(int32_t, int32_t) for combined validation
 */
bool        check_lat(int32_t lat) WARN_IF_UNUSED;

/**
 * @brief Validates that longitude value is within valid geographic range (centidegree version)
 * 
 * @details Checks if a longitude value in centidegrees falls within the valid range for
 *          WGS84 coordinates. Uses integer representation to avoid floating-point precision issues.
 * 
 * @param[in] lng Longitude in centidegrees (degrees × 10^7 for high-precision integer representation)
 * @return true if longitude is in valid range [-180e7, +180e7], false otherwise
 * 
 * @note Valid Range: [-180e7, +180e7] centidegrees = [-1800000000, +1800000000] where:
 *       - -180e7 = International Date Line (westward)
 *       - 0 = Prime Meridian (Greenwich)
 *       - +180e7 = International Date Line (eastward)
 * 
 * @note Units: Centidegrees × 10^7 for integer storage
 *       - 1 degree = 1e7 centidegrees (10,000,000)
 *       - Example: -122.5° = -1225000000 centidegrees
 * 
 * @note Precision: Provides ~1.1 cm resolution at equator
 * 
 * @note Implementation: Uses labs(lng) <= 180*1e7 check
 * 
 * @note Convention: Uses [-180°, +180°] range (not [0°, 360°])
 * 
 * @warning Return value should not be ignored - marked with WARN_IF_UNUSED attribute.
 * 
 * @see check_lng(float) for degree version
 * @see check_latlng(int32_t, int32_t) for combined validation
 */
bool        check_lng(int32_t lng) WARN_IF_UNUSED;

/**
 * @brief Validates both latitude and longitude values (float version)
 * 
 * @details Convenience function that checks both latitude and longitude for validity
 *          in a single call. Equivalent to check_lat(lat) && check_lng(lng).
 * 
 * @param[in] lat Latitude in degrees
 * @param[in] lng Longitude in degrees
 * @return true if both lat and lng are within valid ranges, false if either is invalid
 * 
 * @note Valid Ranges:
 *       - Latitude: [-90.0°, +90.0°]
 *       - Longitude: [-180.0°, +180.0°]
 * 
 * @note Short-Circuit Evaluation: Returns false immediately if latitude is invalid,
 *       without checking longitude (standard && operator behavior)
 * 
 * @note Use Case: Parameter validation, waypoint validation, GPS position sanity checks
 * 
 * @warning Return value should not be ignored - marked with WARN_IF_UNUSED attribute.
 *          Invalid coordinates can cause navigation failures or vehicle crashes.
 * 
 * @see check_lat(float), check_lng(float) for individual validation
 * @see check_latlng(int32_t, int32_t) for centidegree version
 */
bool        check_latlng(float lat, float lng) WARN_IF_UNUSED;

/**
 * @brief Validates both latitude and longitude values (centidegree version)
 * 
 * @details Convenience function that checks both latitude and longitude for validity
 *          in a single call using integer centidegree representation. Equivalent to
 *          check_lat(lat) && check_lng(lng).
 * 
 * @param[in] lat Latitude in centidegrees (degrees × 10^7)
 * @param[in] lng Longitude in centidegrees (degrees × 10^7)
 * @return true if both lat and lng are within valid ranges, false if either is invalid
 * 
 * @note Valid Ranges:
 *       - Latitude: [-90e7, +90e7] centidegrees = [-900000000, +900000000]
 *       - Longitude: [-180e7, +180e7] centidegrees = [-1800000000, +1800000000]
 * 
 * @note Units: Centidegrees × 10^7 (1 degree = 10,000,000)
 * 
 * @note Short-Circuit Evaluation: Returns false immediately if latitude is invalid
 * 
 * @note Use Case: Validating parameters stored as integers, mission waypoint checks,
 *       GPS position validation from telemetry
 * 
 * @warning Return value should not be ignored - marked with WARN_IF_UNUSED attribute.
 *          Invalid coordinates can cause navigation failures.
 * 
 * @note Precision: Integer representation avoids floating-point rounding errors
 *       that could accumulate in parameter storage and retrieval
 * 
 * @see check_lat(int32_t), check_lng(int32_t) for individual validation
 * @see check_latlng(float, float) for degree version
 */
bool        check_latlng(int32_t lat, int32_t lng) WARN_IF_UNUSED;
