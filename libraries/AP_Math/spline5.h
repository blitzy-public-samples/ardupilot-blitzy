/**
 * @file spline5.h
 * @brief Natural cubic spline solver for exactly 5 sample points
 * 
 * @details This file provides a specialized cubic spline interpolation function
 *          optimized for exactly 5 sample points. The implementation uses a
 *          tridiagonal matrix solver (Thomas algorithm) with natural boundary
 *          conditions (zero second derivative at endpoints) to compute cubic
 *          spline coefficients efficiently without dynamic memory allocation.
 *          
 *          The specialization for 5 points allows for compile-time optimization
 *          and avoids runtime memory allocation, making it suitable for
 *          real-time flight control applications.
 * 
 * @note This is a lightweight alternative to the more general SplineCurve class
 *       when exactly 5 interpolation points are needed.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

/**
 * @brief Calculate cubic spline interpolation coefficients for 5 sample points
 * 
 * @details This function computes cubic spline interpolation coefficients for
 *          exactly 5 sample points using a specialized tridiagonal matrix solver
 *          (Thomas algorithm). The spline uses natural boundary conditions,
 *          meaning the second derivative is zero at both endpoints.
 *          
 *          Algorithm overview:
 *          1. Set up tridiagonal system for natural cubic spline
 *          2. Apply Thomas algorithm for efficient solution
 *          3. Compute cubic polynomial coefficients for each segment
 *          4. Store coefficients in 4x4 output matrix
 *          
 *          The output matrix contains coefficients for the 4 cubic polynomial
 *          segments connecting the 5 input points. Each row represents one
 *          segment with coefficients [a, b, c, d] for the cubic:
 *          y(t) = a + b*t + c*t^2 + d*t^3
 *          
 *          Use case: Smooth interpolation through 5 waypoints, sensor samples,
 *          or control points where continuity of first and second derivatives
 *          is required.
 * 
 * @param[in]  x      Array of 5 independent variable values (sample points)
 *                    Must be in increasing order for valid interpolation
 * @param[out] out    4x4 matrix of cubic polynomial coefficients
 *                    out[i][0..3] contains coefficients for segment i
 *                    Each segment i spans from x[i] to x[i+1]
 * 
 * @note This function is specialized for exactly 5 points for efficiency.
 *       No dynamic memory allocation is performed, making it suitable for
 *       real-time applications.
 * 
 * @note Natural boundary conditions are used: zero second derivative at
 *       endpoints x[0] and x[4]. This provides a "natural" curve that doesn't
 *       overshoot at the boundaries.
 * 
 * @note Works with any consistent unit system. Ensure input values are in
 *       appropriate units for your application (e.g., meters, seconds).
 * 
 * @warning The function expects exactly 5 input points. No bounds checking
 *          is performed. Passing arrays of different size will result in
 *          undefined behavior.
 * 
 * @warning Input array x must contain values in strictly increasing order.
 *          Non-monotonic input will produce invalid interpolation results.
 * 
 * @see SplineCurve for more general spline trajectory generation with
 *      variable number of points and additional features
 */
void splinterp5(const float x[5], float out[4][4]);


