/**
 * @file ftype.h
 * @brief Floating-point precision abstraction for EKF double/single precision builds
 * 
 * @details This header provides a type abstraction layer that allows ArduPilot's
 *          Extended Kalman Filter (EKF) implementations to be compiled with either
 *          single precision (float) or double precision (double) floating-point
 *          arithmetic. This enables flexibility between performance optimization
 *          (single precision) and numerical accuracy (double precision) depending
 *          on platform capabilities and application requirements.
 *          
 *          The ftype typedef and associated macro wrappers (sinF, cosF, etc.)
 *          automatically select the appropriate precision based on the
 *          HAL_WITH_EKF_DOUBLE compile-time flag.
 * 
 * @note Primary users: AP_NavEKF2, AP_NavEKF3 state estimation libraries
 * @note Default configuration: Single precision (float) for most platforms
 * @note Double precision requires ALLOW_DOUBLE_MATH_FUNCTIONS to be defined
 * 
 * @see libraries/AP_NavEKF2/ - EKF2 implementation using ftype
 * @see libraries/AP_NavEKF3/ - EKF3 implementation using ftype
 * @see definitions.h for FLT_EPSILON and related constants
 */

#pragma once

/*
  allow for builds with either single or double precision EKF
 */

#include <AP_HAL/AP_HAL.h>
#include <float.h>

/*
  capital F is used to denote the chosen float type (float or double)
 */

/**
 * @def HAL_WITH_EKF_DOUBLE
 * @brief Compile-time flag selecting double precision for EKF state estimation
 * 
 * @details When set to 1, enables double precision (64-bit) floating-point
 *          arithmetic for EKF calculations. When set to 0 (default), uses
 *          single precision (32-bit) floating-point arithmetic.
 *          
 *          Trade-offs:
 *          - Double precision: Higher numerical accuracy, more memory, slower computation
 *          - Single precision: Faster computation, less memory, sufficient for most applications
 *          
 *          This flag is typically defined in AP_HAL board configuration files.
 * 
 * @note Default value: 0 (single precision) for most embedded platforms
 * @note Set to 1 for high-accuracy navigation requirements or testing
 */
#if HAL_WITH_EKF_DOUBLE

/**
 * @typedef ftype
 * @brief Floating-point type abstraction: double precision mode
 * 
 * @details When HAL_WITH_EKF_DOUBLE is enabled, ftype is defined as double (64-bit).
 *          This allows EKF algorithms to operate with extended precision for improved
 *          numerical stability in state estimation, particularly for:
 *          - Long-duration flights with accumulated position errors
 *          - High-precision navigation applications
 *          - Numerical analysis and algorithm development
 *          
 *          All ftype-based code automatically uses double precision math functions
 *          and provides higher accuracy at the cost of performance and memory.
 * 
 * @note Memory impact: Doubles state vector and covariance matrix storage requirements
 * @note Performance impact: Math operations typically 2-4x slower than float on ARM
 * @see sinF, cosF, sqrtF and other math function macros for precision-aware operations
 */
typedef double ftype;

/** @brief Sine function using ftype precision (double precision mode) */
#define sinF(x) sin(x)

/** @brief Arc cosine (inverse cosine) function using ftype precision */
#define acosF(x) acos(x)

/** @brief Arc sine (inverse sine) function using ftype precision */
#define asinF(x) asin(x)

/** @brief Cosine function using ftype precision (double precision mode) */
#define cosF(x) cos(x)

/** @brief Tangent function using ftype precision (double precision mode) */
#define tanF(x) tan(x)

/** @brief Arc tangent (inverse tangent) function using ftype precision */
#define atanF(x) atan(x)

/** @brief Two-argument arc tangent function using ftype precision, computes atan2(x,y) */
#define atan2F(x,y) atan2(x,y)

/** @brief Square root function using ftype precision (double precision mode) */
#define sqrtF(x) sqrt(x)

/** @brief Maximum of two values using ftype precision */
#define fmaxF(x,y) fmax(x,y)

/** @brief Power function using ftype precision, computes x^y */
#define powF(x,y) pow(x,y)

/** @brief Natural logarithm function using ftype precision */
#define logF(x) log(x)

/** @brief Absolute value function using ftype precision (double precision mode) */
#define fabsF(x) fabs(x)

/** @brief Ceiling function using ftype precision, rounds up to nearest integer */
#define ceilF(x) ceil(x)

/** @brief Minimum of two values using ftype precision */
#define fminF(x,y) fmin(x,y)

/** @brief Floating-point remainder function using ftype precision, computes fmod(x,y) */
#define fmodF(x,y) fmod(x,y)

/** @brief Absolute value function using ftype precision (double precision mode) */
#define fabsF(x) fabs(x)

/** 
 * @brief Conversion to ftype (double precision mode)
 * @details Converts numeric value to double precision (todouble)
 */
#define toftype todouble

#else

/**
 * @typedef ftype
 * @brief Floating-point type abstraction: single precision mode (default)
 * 
 * @details When HAL_WITH_EKF_DOUBLE is disabled (default), ftype is defined as
 *          float (32-bit). This provides the standard precision mode for EKF
 *          algorithms, offering optimal performance on embedded ARM processors
 *          while maintaining sufficient accuracy for most navigation applications.
 *          
 *          Single precision is appropriate for:
 *          - Standard multicopter, plane, and rover navigation
 *          - Real-time flight control (400Hz+ update rates)
 *          - Memory-constrained embedded platforms
 *          - Applications where performance is critical
 *          
 *          All ftype-based code automatically uses single precision math functions
 *          optimized for ARM processors with hardware FPU support.
 * 
 * @note Default mode for production builds on embedded flight controllers
 * @note Single precision provides ~7 decimal digits of accuracy, sufficient for meter-level navigation
 * @see sinF, cosF, sqrtF and other math function macros for precision-aware operations
 */
typedef float ftype;

/** @brief Arc cosine (inverse cosine) function using ftype precision (single precision mode) */
#define acosF(x) acosf(x)

/** @brief Arc sine (inverse sine) function using ftype precision (single precision mode) */
#define asinF(x) asinf(x)

/** @brief Sine function using ftype precision (single precision mode) */
#define sinF(x) sinf(x)

/** @brief Cosine function using ftype precision (single precision mode) */
#define cosF(x) cosf(x)

/** @brief Tangent function using ftype precision (single precision mode) */
#define tanF(x) tanf(x)

/** @brief Arc tangent (inverse tangent) function using ftype precision (single precision mode) */
#define atanF(x) atanf(x)

/** @brief Two-argument arc tangent function using ftype precision, computes atan2(y,x) */
#define atan2F(y,x) atan2f(y,x)

/** @brief Square root function using ftype precision (single precision mode) */
#define sqrtF(x) sqrtf(x)

/** @brief Maximum of two values using ftype precision (single precision mode) */
#define fmaxF(x,y) fmaxf(x,y)

/** @brief Power function using ftype precision, computes x^y */
#define powF(x,y) powf(x,y)

/** @brief Natural logarithm function using ftype precision (single precision mode) */
#define logF(x) logf(x)

/** @brief Absolute value function using ftype precision (single precision mode) */
#define fabsF(x) fabsf(x)

/** @brief Ceiling function using ftype precision, rounds up to nearest integer */
#define ceilF(x) ceilf(x)

/** @brief Minimum of two values using ftype precision (single precision mode) */
#define fminF(x,y) fminf(x,y)

/** @brief Floating-point remainder function using ftype precision, computes fmod(x,y) */
#define fmodF(x,y) fmodf(x,y)

/** @brief Absolute value function using ftype precision (single precision mode) */
#define fabsF(x) fabsf(x)

/** 
 * @brief Conversion to ftype (single precision mode)
 * @details Converts numeric value to single precision float (tofloat)
 */
#define toftype tofloat

#endif

/**
 * @def ZERO_FARRAY
 * @brief Efficient array zeroing based on MATH_CHECK_INDEXES configuration
 * 
 * @details This macro provides an optimized way to zero floating-point arrays.
 *          The implementation strategy depends on whether array bounds checking
 *          is enabled:
 *          
 *          - When MATH_CHECK_INDEXES is enabled: Uses a.zero() method call
 *            which provides bounds-checked zeroing for safer debugging
 *          
 *          - When MATH_CHECK_INDEXES is disabled (production builds): Uses
 *            memset() for maximum performance with direct memory manipulation
 *          
 *          Typical usage in EKF state vector initialization and covariance
 *          matrix resets where performance is critical.
 * 
 * @param a Array or vector object to be zeroed
 * 
 * @note Production builds use memset for speed-critical EKF operations
 * @note Debug builds use method call for safety and bounds validation
 * @see MATH_CHECK_INDEXES configuration flag
 */
#if MATH_CHECK_INDEXES
#define ZERO_FARRAY(a) a.zero()
#else
#define ZERO_FARRAY(a) memset(a, 0, sizeof(a))
#endif

/**
 * @brief Check whether a float is effectively zero
 * 
 * @details Tests if a single precision floating-point value is close enough
 *          to zero to be considered effectively zero, accounting for floating-point
 *          representation limitations. Uses FLT_EPSILON (typically 1e-6) as the
 *          tolerance threshold.
 *          
 *          This function is essential for numerical stability in control algorithms
 *          where exact zero comparisons can fail due to floating-point rounding errors.
 * 
 * @param[in] x Single precision floating-point value to test
 * 
 * @return true if |x| < FLT_EPSILON (value is effectively zero)
 * @return false if |x| >= FLT_EPSILON (value is not zero)
 * 
 * @note Uses FLT_EPSILON as tolerance regardless of actual float precision
 * @note Typical usage: Division by zero checks, convergence tests, deadband logic
 * @see FLT_EPSILON in definitions.h for tolerance value
 */
inline bool is_zero(const float x) {
    return fabsf(x) < FLT_EPSILON;
}

/**
 * @brief Check whether a double is effectively zero
 * 
 * @details Tests if a double precision floating-point value is close enough
 *          to zero to be considered effectively zero. The implementation depends
 *          on whether ALLOW_DOUBLE_MATH_FUNCTIONS is defined:
 *          
 *          - With ALLOW_DOUBLE_MATH_FUNCTIONS: Uses native double precision fabs()
 *            for full precision comparison
 *          
 *          - Without ALLOW_DOUBLE_MATH_FUNCTIONS: Casts to float and uses fabsf()
 *            to avoid linking double precision math library
 *          
 *          Note that the tolerance threshold is FLT_EPSILON (single precision)
 *          regardless of the actual precision used, providing consistent behavior
 *          across float and double types.
 * 
 * @param[in] x Double precision floating-point value to test
 * 
 * @return true if |x| < FLT_EPSILON (value is effectively zero)
 * @return false if |x| >= FLT_EPSILON (value is not zero)
 * 
 * @note Always uses FLT_EPSILON tolerance even for double precision values
 * @note Performance: Single precision cast version avoids double math library dependency
 * @warning ALLOW_DOUBLE_MATH_FUNCTIONS must be defined for full double precision comparison
 * @see FLT_EPSILON in definitions.h for tolerance value
 */
inline bool is_zero(const double x) {
#ifdef ALLOW_DOUBLE_MATH_FUNCTIONS
  return fabs(x) < FLT_EPSILON;
#else
  return fabsf(static_cast<float>(x)) < FLT_EPSILON;
#endif
}
