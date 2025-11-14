/**
 * @file polyfit.h
 * @brief Online polynomial fitting for vector-valued outputs
 * 
 * @details This file provides a template-based polynomial fitting class
 *          that performs online least-squares regression for vector-valued
 *          dependent variables. The implementation uses an incremental
 *          algorithm that maintains only summary statistics (moment matrices)
 *          rather than storing all historical data points, making it suitable
 *          for embedded systems with limited memory.
 * 
 *          The polynomial fit solves: y = c0 + c1*x + c2*x^2 + ... + cn*x^n
 *          where y can be a scalar or vector type (e.g., Vector2f, Vector3f)
 *          allowing simultaneous fitting of multiple related curves.
 * 
 *          Originally written by Siddharth Bharat Purohit
 *          Re-written for ArduPilot by Andrew Tridgell
 * 
 * @note This is an online algorithm with minimal storage requirements -
 *       data points are processed incrementally without storing the full
 *       history, making it ideal for real-time trajectory estimation
 * 
 * @author Siddharth Bharat Purohit (original), Andrew Tridgell (ArduPilot)
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <stdint.h>

/**
 * @class PolyFit
 * @brief Template class for online polynomial fitting with vector-valued outputs
 * 
 * @details This class implements incremental least-squares polynomial regression
 *          using the normal equations approach. It maintains internal moment
 *          matrices that are updated with each new data point, allowing the
 *          polynomial coefficients to be computed without storing all historical
 *          observations.
 * 
 *          Algorithm: Incremental least-squares using normal equations
 *          - Maintains sum matrices: Σ(x^i * x^j) and Σ(x^i * y)
 *          - Solves: (X^T X) c = X^T y for coefficients c
 *          - Updates are O(order^2) per data point
 *          - Coefficient extraction is O(order^3) via Gaussian elimination
 * 
 *          Common applications:
 *          - Curve fitting for sensor calibration
 *          - Trajectory prediction and extrapolation
 *          - Smoothing and derivative estimation
 *          - Time-series trend analysis
 * 
 *          Example usage for 2D trajectory prediction:
 *          @code
 *          // Fit a quadratic (order=2) polynomial to Vector2f positions over time
 *          PolyFit<2, float, Vector2f> trajectory_fit;
 *          
 *          // Add measurements: time (seconds) and position (meters)
 *          trajectory_fit.update(0.0f, Vector2f(0.0f, 0.0f));
 *          trajectory_fit.update(1.0f, Vector2f(1.0f, 0.5f));
 *          trajectory_fit.update(2.0f, Vector2f(4.0f, 2.0f));
 *          
 *          // Get polynomial coefficients: [c0, c1, c2]
 *          // Represents: position(t) = c0 + c1*t + c2*t^2
 *          Vector2f coeffs[2];
 *          if (trajectory_fit.get_polynomial(coeffs)) {
 *              // Predict position at t=3.0: coeffs[0] + coeffs[1]*3.0 + coeffs[2]*9.0
 *              Vector2f predicted = coeffs[0] + coeffs[1]*3.0f + coeffs[2]*9.0f;
 *          }
 *          @endcode
 * 
 * @tparam order Polynomial order (degree of the polynomial, e.g., 2 for quadratic).
 *               Valid range: 1-255. Higher orders provide more flexible fitting
 *               but require more computation and can suffer numerical instability.
 * 
 * @tparam xtype Type of the independent variable (x-axis), typically float or double.
 *               Must support arithmetic operations (+, *, /) and comparison.
 * 
 * @tparam vtype Type of the dependent variable (y-axis), can be scalar or vector.
 *               Must support arithmetic operations and multiplication by xtype.
 *               Common types: float, double, Vector2f, Vector3f, Vector2d, Vector3d.
 *               Using vector types allows fitting multiple related curves simultaneously
 *               (e.g., x,y,z position components with shared time basis).
 * 
 * @note This is an online algorithm - data points are processed incrementally
 *       without storing the complete dataset. Only O(order^2) storage required
 *       regardless of the number of data points added.
 * 
 * @warning Numerical stability decreases for high-order polynomials (order > 4).
 *          For order > 4, consider using orthogonal polynomial bases or
 *          regularization techniques. Condition number of the moment matrix
 *          grows rapidly with polynomial degree, potentially causing inaccurate
 *          results for ill-conditioned data distributions.
 * 
 * @warning Requires at least (order+1) data points before get_polynomial()
 *          returns valid results. Adding fewer points will cause get_polynomial()
 *          to return false indicating insufficient data.
 */
template <uint8_t order, typename xtype, typename vtype>
class PolyFit
{
public:
    /**
     * @brief Add a new data point to the polynomial fit
     * 
     * @details Updates the internal moment matrices with a new (x, y) observation.
     *          This method incrementally adjusts the fit to incorporate the new
     *          data point without requiring access to previous observations.
     *          The update is O(order^2) complexity.
     * 
     *          The method accumulates:
     *          - mat[i][j] += x^(i+j+2) for moment matrix
     *          - vec[i] += x^(i+1) * y for right-hand side vector
     * 
     *          Call this method each time a new measurement is available.
     *          At least (order+1) calls are required before get_polynomial()
     *          can return a valid fit.
     * 
     * @param[in] x Independent variable value (e.g., time, position, index).
     *              Must be representable by xtype with sufficient precision.
     * 
     * @param[in] y Dependent variable value corresponding to x (e.g., measurement,
     *              position vector). Can be scalar or vector type. All components
     *              of y are fitted simultaneously against the same x values.
     * 
     * @note This method does not return a status - updates are always accepted.
     *       Numerical issues may only be detected when calling get_polynomial().
     * 
     * @note For best numerical stability, normalize x values to a reasonable
     *       range (e.g., [0, 1] or [-1, 1]) before calling update().
     *       Avoid extremely large or small x values that could cause overflow.
     */
    void update(xtype x, vtype y);
    
    /**
     * @brief Retrieve the fitted polynomial coefficients
     * 
     * @details Solves the normal equations (X^T X) c = X^T y to extract
     *          polynomial coefficients from the accumulated moment matrices.
     *          Uses Gaussian elimination to solve the linear system.
     *          Complexity is O(order^3).
     * 
     *          The returned coefficients represent the polynomial:
     *          y = res[0] + res[1]*x + res[2]*x^2 + ... + res[order-1]*x^(order-1)
     * 
     *          For vector-typed vtype (e.g., Vector3f), each coefficient is
     *          a vector, fitting all components simultaneously:
     *          y.x = res[0].x + res[1].x*x + res[2].x*x^2 + ...
     *          y.y = res[0].y + res[1].y*x + res[2].y*x^2 + ...
     *          y.z = res[0].z + res[1].z*x + res[2].z*x^2 + ...
     * 
     * @param[out] res Array of polynomial coefficients to be filled.
     *                 Must be pre-allocated with size [order].
     *                 - res[0]: constant term (y-intercept)
     *                 - res[1]: linear coefficient (slope)
     *                 - res[2]: quadratic coefficient
     *                 - res[i]: coefficient for x^i term
     *                 Array is only modified if the method returns true.
     * 
     * @return true if polynomial coefficients were successfully computed.
     *              Indicates sufficient data points (>= order+1) and the
     *              moment matrix is invertible (well-conditioned problem).
     * 
     * @return false if polynomial cannot be computed due to:
     *               - Insufficient data points (< order+1 calls to update())
     *               - Singular or near-singular moment matrix (ill-conditioned)
     *               - Numerical instability in Gaussian elimination
     *               In this case, res array contents are undefined.
     * 
     * @note This method can be called multiple times without affecting the fit.
     *       It is a const operation that only reads internal state.
     * 
     * @note For best results, ensure data points span the full range of x values
     *       of interest. Extrapolation beyond the fitted range may be unreliable,
     *       especially for higher-order polynomials.
     * 
     * @warning For high-order fits (order > 4), check return value carefully.
     *          Numerical instability becomes more likely as order increases.
     */
    bool get_polynomial(vtype res[order]) const;

private:
    xtype mat[order][order];
    vtype vec[order];
};

