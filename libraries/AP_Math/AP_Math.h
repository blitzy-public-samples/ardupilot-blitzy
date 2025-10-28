/**
 * @file AP_Math.h
 * @brief Main math library header providing numeric utilities, interpolation, angle wrapping, and safe math functions
 * 
 * @details This header provides the core mathematical utilities used throughout ArduPilot:
 *          - Floating-point comparison and zero-testing with epsilon tolerance
 *          - Safe mathematical functions (safe_asin, safe_sqrt) that handle edge cases
 *          - Matrix operations (multiplication, inversion, identity)
 *          - Angle wrapping functions for degrees, centidegrees, and radians
 *          - Constrain/clamp functions for various numeric types
 *          - Linear interpolation and curve generation utilities
 *          - Random number generation
 *          - Low-pass filter coefficient calculation
 *          - Unit conversion helpers (degrees/radians/centidegrees)
 *          
 *          The library supports both float and double precision through template functions
 *          and conditional compilation with HAL_WITH_EKF_DOUBLE.
 *          
 * @note Units: Angles are in radians unless explicitly specified (degrees, centidegrees)
 * @note Coordinate frames: Operations are coordinate-agnostic unless specified
 * @note Explicit template instantiations for float/double are in AP_Math.cpp
 * 
 * @see vector2.h, vector3.h, matrix3.h, quaternion.h for geometric types
 * @see control.h for control-specific math functions
 */

#pragma once

#include <cmath>
#include <limits>
#include <stdint.h>
#include <type_traits>

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

#include "definitions.h"
#include "crc.h"
#include "matrix3.h"
#include "polygon.h"
#include "quaternion.h"
#include "rotations.h"
#include "vector2.h"
#include "vector3.h"
#include "spline5.h"
#include "location.h"
#include "control.h"

static const float NaNf = nanf("0x4152");

/**
 * @brief Precision-abstracted 2D vector type
 * @details Maps to Vector2<double> if HAL_WITH_EKF_DOUBLE is defined, otherwise Vector2<float>.
 *          Used for EKF state estimation where double precision may be required.
 */
#if HAL_WITH_EKF_DOUBLE
typedef Vector2<double> Vector2F;
/**
 * @brief Precision-abstracted 3D vector type
 * @details Maps to Vector3<double> if HAL_WITH_EKF_DOUBLE is defined, otherwise Vector3<float>.
 *          Used for EKF state estimation where double precision may be required.
 */
typedef Vector3<double> Vector3F;
/**
 * @brief Precision-abstracted 3x3 matrix type
 * @details Maps to Matrix3<double> if HAL_WITH_EKF_DOUBLE is defined, otherwise Matrix3<float>.
 *          Used for rotation matrices in EKF where double precision may be required.
 */
typedef Matrix3<double> Matrix3F;
/**
 * @brief Precision-abstracted quaternion type
 * @details Maps to QuaternionD if HAL_WITH_EKF_DOUBLE is defined, otherwise Quaternion (float).
 *          Used for attitude representation in EKF where double precision may be required.
 */
typedef QuaternionD QuaternionF;
#else
typedef Vector2<float> Vector2F;
typedef Vector3<float> Vector3F;
typedef Matrix3<float> Matrix3F;
typedef Quaternion QuaternionF;
#endif

/**
 * @brief Parameter type registration for Vector3f
 * @details Registers Vector3f as an AP_Param type with ID AP_PARAM_VECTOR3F,
 *          enabling Vector3f parameters to be stored, loaded, and accessed via
 *          the parameter system.
 */
// define AP_Param type AP_Vector3f
AP_PARAMDEFV(Vector3f, Vector3f, AP_PARAM_VECTOR3F);

/**
 * @brief Floating-point equality test with epsilon tolerance
 * 
 * @details Checks whether two arithmetic values are equal, with special handling for
 *          floating-point types. For integral types, performs exact comparison.
 *          For floating-point types, uses epsilon tolerance to handle rounding errors.
 *          Template uses SFINAE to select appropriate implementation based on type.
 * 
 * @tparam Arithmetic1 First arithmetic type (integral or floating-point)
 * @tparam Arithmetic2 Second arithmetic type (integral or floating-point)
 * @param[in] v_1 First value to compare
 * @param[in] v_2 Second value to compare
 * @return true if values are equal (within epsilon for floating-point), false otherwise
 * 
 * @note For floating-point comparison, uses FLT_EPSILON tolerance
 * @note Common type is automatically determined from input types
 * 
 * @see is_zero(), is_positive(), is_negative()
 */
template <typename Arithmetic1, typename Arithmetic2>
typename std::enable_if<std::is_integral<typename std::common_type<Arithmetic1, Arithmetic2>::type>::value ,bool>::type
is_equal(const Arithmetic1 v_1, const Arithmetic2 v_2);

/**
 * @brief Floating-point equality test with epsilon tolerance (floating-point specialization)
 * 
 * @details Template specialization for floating-point types. Uses epsilon tolerance
 *          to handle numerical precision issues common in floating-point arithmetic.
 * 
 * @tparam Arithmetic1 First floating-point type
 * @tparam Arithmetic2 Second floating-point type
 * @param[in] v_1 First value to compare
 * @param[in] v_2 Second value to compare
 * @return true if values are equal within FLT_EPSILON tolerance, false otherwise
 * 
 * @note Epsilon tolerance prevents issues with rounding errors in sensor fusion and control loops
 */
template <typename Arithmetic1, typename Arithmetic2>
typename std::enable_if<std::is_floating_point<typename std::common_type<Arithmetic1, Arithmetic2>::type>::value, bool>::type
is_equal(const Arithmetic1 v_1, const Arithmetic2 v_2);

/**
 * @brief Check whether a float is zero using FLT_EPSILON tolerance
 * 
 * @details Tests if a floating-point value is effectively zero by checking if
 *          its absolute value is less than FLT_EPSILON. Handles numerical precision
 *          issues in floating-point comparisons.
 * 
 * @tparam T Floating-point type or AP_Float-derived type
 * @param[in] fVal1 Value to test
 * @return true if value is within FLT_EPSILON of zero, false otherwise
 * 
 * @note Uses FLT_EPSILON tolerance to handle floating-point rounding errors
 * @see is_equal(), is_positive(), is_negative()
 */
template <typename T>
inline bool is_zero(const T fVal1) {
    static_assert(std::is_floating_point<T>::value || std::is_base_of<T,AP_Float>::value,
                  "Template parameter not of type float");
    return is_zero(static_cast<float>(fVal1));
}

/**
 * @brief Check whether a float is greater than zero using FLT_EPSILON tolerance
 * 
 * @details Tests if a floating-point value is positive (greater than FLT_EPSILON).
 *          This is more reliable than testing > 0 for floating-point values
 *          due to numerical precision limitations.
 * 
 * @tparam T Floating-point type or AP_Float-derived type
 * @param[in] fVal1 Value to test
 * @return true if value is >= FLT_EPSILON, false otherwise
 * 
 * @note Used extensively in control loops to check for valid positive inputs
 * @see is_zero(), is_negative(), is_equal()
 */
template <typename T>
inline bool is_positive(const T fVal1) {
    static_assert(std::is_floating_point<T>::value || std::is_base_of<T,AP_Float>::value,
                  "Template parameter not of type float");
    return (static_cast<float>(fVal1) >= FLT_EPSILON);
}


/**
 * @brief Check whether a float is less than zero using FLT_EPSILON tolerance
 * 
 * @details Tests if a floating-point value is negative (less than -FLT_EPSILON).
 *          This is more reliable than testing < 0 for floating-point values
 *          due to numerical precision limitations.
 * 
 * @tparam T Floating-point type or AP_Float-derived type
 * @param[in] fVal1 Value to test
 * @return true if value is <= -FLT_EPSILON, false otherwise
 * 
 * @note Used in control loops and failsafe logic for sign checking
 * @see is_zero(), is_positive(), is_equal()
 */
template <typename T>
inline bool is_negative(const T fVal1) {
    static_assert(std::is_floating_point<T>::value || std::is_base_of<T,AP_Float>::value,
                  "Template parameter not of type float");
    return (static_cast<float>(fVal1) <= (-1.0 * FLT_EPSILON));
}

/**
 * @brief Check whether a double is greater than zero using FLT_EPSILON tolerance
 * 
 * @details Double-precision specialization for positive value testing.
 *          Uses FLT_EPSILON threshold for consistency with float version.
 * 
 * @param[in] fVal1 Double value to test
 * @return true if value is >= FLT_EPSILON, false otherwise
 */
inline bool is_positive(const double fVal1) {
    return (fVal1 >= static_cast<double>(FLT_EPSILON));
}

/**
 * @brief Check whether a double is less than zero using FLT_EPSILON tolerance
 * 
 * @details Double-precision specialization for negative value testing.
 *          Uses FLT_EPSILON threshold for consistency with float version.
 * 
 * @param[in] fVal1 Double value to test
 * @return true if value is <= -FLT_EPSILON, false otherwise
 */
inline bool is_negative(const double fVal1) {
    return (fVal1 <= static_cast<double>((-1.0 * FLT_EPSILON)));
}

/**
 * @brief Clamps input to [-1, 1] then computes asin, safely handling out-of-range inputs
 * 
 * @details A variant of asin() that checks input ranges and ensures a valid angle output.
 *          Handles common numerical errors in vector normalization and trigonometric
 *          calculations by clamping inputs to the valid domain [-1, 1].
 *          - If input > 1, clamps to 1 (returns π/2)
 *          - If input < -1, clamps to -1 (returns -π/2)
 *          - If input is NaN, returns 0
 * 
 * @tparam T Numeric type (typically float or double)
 * @param[in] v Input value (will be clamped to [-1, 1])
 * @return Angle in radians in range [-π/2, π/2]
 * 
 * @note Returns 0 for NaN inputs to prevent error propagation
 * @note Used in attitude calculation and sensor fusion where numerical errors can push
 *       cosine/sine values slightly outside valid range
 * 
 * @warning Clamping may hide numerical issues in the calling code - use carefully
 * @warning Silent clamping means no feedback when inputs are out of range
 * 
 * @see safe_sqrt()
 */
template <typename T>
float safe_asin(const T v);

/**
 * @brief Returns 0 for negative inputs, sqrt otherwise, safely handling numerical errors
 * 
 * @details A variant of sqrt() that checks input ranges and ensures a valid value output.
 *          If a negative number is given then 0 is returned. The reasoning is that a
 *          negative number for sqrt() in our code is usually caused by small numerical
 *          rounding errors (e.g., from (1 - cos²θ)), so the real input should have been zero.
 *          - If input >= 0, returns sqrt(input)
 *          - If input < 0, returns 0
 * 
 * @tparam T Numeric type (typically float or double)
 * @param[in] v Input value
 * @return Non-negative result: sqrt(v) if v >= 0, otherwise 0
 * 
 * @note Commonly used in vector magnitude calculations where numerical precision
 *       can produce small negative values
 * @note Prevents NaN propagation from negative sqrt inputs
 * 
 * @warning Clamping negative inputs to 0 may hide numerical issues - use carefully
 * @warning Consider logging when this function clamps to help detect numerical problems
 * 
 * @see safe_asin(), pythagorous2(), pythagorous3()
 */
template <typename T>
float safe_sqrt(const T v);

/**
 * @brief NxN matrix multiplication C = A * B
 * 
 * @details Performs matrix multiplication of two square matrices stored in row-major order.
 *          Computes C[i,j] = sum(A[i,k] * B[k,j]) for all i,j.
 *          Matrices are passed as flat arrays of size n*n.
 * 
 * @tparam T Numeric type (typically float or double)
 * @param[in]  A First input matrix (n x n), row-major order
 * @param[in]  B Second input matrix (n x n), row-major order
 * @param[out] C Output matrix (n x n), row-major order, result of A * B
 * @param[in]  n Dimension of square matrices
 * 
 * @note Matrix element at row i, column j is accessed as array[i*n + j]
 * @note Output matrix C must be pre-allocated with size n*n
 * @note C can be the same as A or B (in-place operation supported)
 * 
 * @see mat_inverse(), mat_identity()
 */
template <typename T>
void mat_mul(const T *A, const T *B, T *C, uint16_t n);

/**
 * @brief Matrix inversion using LU decomposition
 * 
 * @details Computes the inverse of a square matrix using LU decomposition with
 *          partial pivoting. Returns false if matrix is singular (non-invertible).
 *          Matrices are stored in row-major order as flat arrays.
 * 
 * @tparam T Numeric type (typically float or double)
 * @param[in]  x Input matrix (dim x dim), row-major order
 * @param[out] y Output inverse matrix (dim x dim), row-major order
 * @param[in]  dim Dimension of square matrix
 * @return true if inversion successful, false if matrix is singular
 * 
 * @note Matrix element at row i, column j is accessed as array[i*dim + j]
 * @note Output matrix y must be pre-allocated with size dim*dim
 * @note y must be different from x (in-place inversion not supported)
 * @note Numerical stability may be an issue for ill-conditioned matrices
 * 
 * @warning Returns false for singular matrices - caller must check return value
 * @warning Large matrices may have numerical precision issues
 * 
 * @see mat_mul(), mat_identity()
 */
template <typename T>
bool mat_inverse(const T *x, T *y, uint16_t dim) WARN_IF_UNUSED;

/**
 * @brief Creates identity matrix
 * 
 * @details Fills a square matrix with the identity matrix: 1 on diagonal, 0 elsewhere.
 *          Matrix is stored in row-major order as a flat array.
 *          I[i,j] = 1 if i==j, 0 otherwise
 * 
 * @tparam T Numeric type (typically float or double)
 * @param[out] x Output identity matrix (dim x dim), row-major order
 * @param[in]  dim Dimension of square matrix
 * 
 * @note Matrix element at row i, column j is accessed as array[i*dim + j]
 * @note Output matrix x must be pre-allocated with size dim*dim
 * 
 * @see mat_mul(), mat_inverse()
 */
template <typename T>
void mat_identity(T *x, uint16_t dim);

/**
 * @brief Constrains angle to [-180, 180] degrees
 * 
 * @details Wraps an angle to be within the range -180 to 180 degrees using modulo
 *          arithmetic. Useful for normalizing heading and attitude angles to
 *          principal range for comparison and control.
 * 
 * @tparam T Numeric type (typically int or float)
 * @param[in] angle Input angle in degrees (can be any value)
 * @return Wrapped angle in range [-180, 180] degrees
 * 
 * @note Used extensively in attitude control and navigation
 * @note Equivalent to wrap_PI() but for degrees instead of radians
 * 
 * @see wrap_180_cd(), wrap_360(), wrap_PI()
 */
template <typename T>
T wrap_180(const T angle);

/**
 * @brief Constrains angle to [-18000, 18000] centidegrees
 * 
 * @details Wraps an angle in centidegrees (1/100 degree) to range -18000 to 18000.
 *          Centidegrees are commonly used in ArduPilot for integer angle representation
 *          with 0.01 degree precision.
 * 
 * @tparam T Numeric type (typically int16_t, int32_t, or float)
 * @param[in] angle Input angle in centidegrees (can be any value)
 * @return Wrapped angle in range [-18000, 18000] centidegrees
 * 
 * @note 18000 centidegrees = 180 degrees
 * @note Used in MAVLink messages and parameter storage
 * 
 * @see wrap_180(), wrap_360_cd(), cd_to_rad()
 */
template <typename T>
T wrap_180_cd(const T angle);

/**
 * @brief Constrains angle to [0, 360] degrees
 * 
 * @details Wraps an angle to be within the range 0 to 360 degrees using modulo
 *          arithmetic. Useful for compass headings and other cyclic measurements
 *          where negative angles should be avoided.
 * 
 * @param[in] angle Input angle in degrees (can be any value)
 * @return Wrapped angle in range [0, 360] degrees
 * 
 * @note Used for compass headings and yaw angles
 * @note Equivalent to wrap_2PI() but for degrees instead of radians
 * 
 * @see wrap_360_cd(), wrap_180(), wrap_2PI()
 */
float wrap_360(const float angle);
#ifdef ALLOW_DOUBLE_MATH_FUNCTIONS
/**
 * @brief Constrains angle to [0, 360] degrees (double precision)
 * @param[in] angle Input angle in degrees
 * @return Wrapped angle in range [0, 360] degrees
 */
double wrap_360(const double angle);
#endif
/**
 * @brief Constrains angle to [0, 360] degrees (integer version)
 * @param[in] angle Input angle in degrees
 * @return Wrapped angle in range [0, 360] degrees
 */
int wrap_360(const int angle);

/**
 * @brief Constrains angle to [0, 36000] centidegrees (integer version)
 * @param[in] angle Input angle in centidegrees
 * @return Wrapped angle in range [0, 36000] centidegrees
 * @note 36000 centidegrees = 360 degrees
 */
int wrap_360_cd(const int angle);
/**
 * @brief Constrains angle to [0, 36000] centidegrees (long version)
 * @param[in] angle Input angle in centidegrees
 * @return Wrapped angle in range [0, 36000] centidegrees
 */
long wrap_360_cd(const long angle);
/**
 * @brief Constrains angle to [0, 36000] centidegrees (float version)
 * @param[in] angle Input angle in centidegrees
 * @return Wrapped angle in range [0, 36000] centidegrees
 */
float wrap_360_cd(const float angle);
#ifdef ALLOW_DOUBLE_MATH_FUNCTIONS
/**
 * @brief Constrains angle to [0, 36000] centidegrees (double version)
 * @param[in] angle Input angle in centidegrees
 * @return Wrapped angle in range [0, 36000] centidegrees
 */
double wrap_360_cd(const double angle);
#endif


/**
 * @brief Wraps angle to [-π, π] radians (equivalent to ±180 degrees)
 * 
 * @details Normalizes an angle in radians to the principal range -π to π.
 *          Essential for attitude representation and control where angles
 *          must be in standard range for correct error calculation.
 * 
 * @param[in] radian Input angle in radians (can be any value)
 * @return Wrapped angle in range [-π, π] radians
 * 
 * @note ftype is float or double depending on HAL_WITH_EKF_DOUBLE
 * @note Used in EKF and attitude control systems
 * @note Equivalent to wrap_180() but for radians
 * 
 * @see wrap_2PI(), wrap_180(), radians()
 */
ftype wrap_PI(const ftype radian);

/**
 * @brief Wraps angle to [0, 2π] radians
 * 
 * @details Normalizes an angle in radians to the range 0 to 2π.
 *          Useful for compass headings and other cyclic measurements
 *          where negative angles should be avoided.
 * 
 * @param[in] radian Input angle in radians (can be any value)
 * @return Wrapped angle in range [0, 2π] radians
 * 
 * @note ftype is float or double depending on HAL_WITH_EKF_DOUBLE
 * @note Equivalent to wrap_360() but for radians
 * 
 * @see wrap_PI(), wrap_360(), radians()
 */
ftype wrap_2PI(const ftype radian);

/**
 * @brief Clamps value to [low, high] range
 * 
 * @details Constrains a value to be within specified bounds. If value is below low,
 *          returns low. If value is above high, returns high. Otherwise returns
 *          value unchanged. Essential for parameter validation and safety limits.
 * 
 * @tparam T Numeric type (int, float, double, etc.)
 * @param[in] amt Value to constrain
 * @param[in] low Minimum allowed value (lower bound)
 * @param[in] high Maximum allowed value (upper bound)
 * @return Constrained value in range [low, high]
 * 
 * @note Used extensively in control loops to enforce safety limits
 * @note low should be <= high, behavior undefined if low > high
 * 
 * @warning Silently limits values - consider logging when constraints are hit
 * @warning Does not check if low <= high - caller must ensure valid range
 * 
 * @see constrain_int16(), constrain_float(), constrain_value_line()
 */
template <typename T>
T constrain_value(const T amt, const T low, const T high);

/**
 * @brief Clamps value to [low, high] range with line number tracking
 * 
 * @details Internal variant of constrain_value that records the source line number
 *          where constraint was applied. Used by constrain_float/constrain_ftype macros
 *          for debugging constraint violations.
 * 
 * @tparam T Numeric type
 * @param[in] amt Value to constrain
 * @param[in] low Minimum allowed value
 * @param[in] high Maximum allowed value
 * @param[in] line Source code line number (for debugging)
 * @return Constrained value in range [low, high]
 * 
 * @note Typically called via constrain_float() or constrain_ftype() macros
 */
template <typename T>
T constrain_value_line(const T amt, const T low, const T high, uint32_t line);

#define constrain_float(amt, low, high) constrain_value_line(float(amt), float(low), float(high), uint32_t(__AP_LINE__))
#define constrain_ftype(amt, low, high) constrain_value_line(ftype(amt), ftype(low), ftype(high), uint32_t(__AP_LINE__))

/**
 * @brief Clamps int16_t value to specified range
 * @param[in] amt Value to constrain
 * @param[in] low Minimum allowed value
 * @param[in] high Maximum allowed value
 * @return Constrained value in range [low, high]
 * @see constrain_value()
 */
inline int16_t constrain_int16(const int16_t amt, const int16_t low, const int16_t high)
{
    return constrain_value(amt, low, high);
}

/**
 * @brief Clamps uint16_t value to specified range
 * @param[in] amt Value to constrain
 * @param[in] low Minimum allowed value
 * @param[in] high Maximum allowed value
 * @return Constrained value in range [low, high]
 * @see constrain_value()
 */
inline uint16_t constrain_uint16(const uint16_t amt, const uint16_t low, const uint16_t high)
{
    return constrain_value(amt, low, high);
}

/**
 * @brief Clamps int32_t value to specified range
 * @param[in] amt Value to constrain
 * @param[in] low Minimum allowed value
 * @param[in] high Maximum allowed value
 * @return Constrained value in range [low, high]
 * @see constrain_value()
 */
inline int32_t constrain_int32(const int32_t amt, const int32_t low, const int32_t high)
{
    return constrain_value(amt, low, high);
}

/**
 * @brief Clamps uint32_t value to specified range
 * @param[in] amt Value to constrain
 * @param[in] low Minimum allowed value
 * @param[in] high Maximum allowed value
 * @return Constrained value in range [low, high]
 * @see constrain_value()
 */
inline uint32_t constrain_uint32(const uint32_t amt, const uint32_t low, const uint32_t high)
{
    return constrain_value(amt, low, high);
}

/**
 * @brief Clamps int64_t value to specified range
 * @param[in] amt Value to constrain
 * @param[in] low Minimum allowed value
 * @param[in] high Maximum allowed value
 * @return Constrained value in range [low, high]
 * @see constrain_value()
 */
inline int64_t constrain_int64(const int64_t amt, const int64_t low, const int64_t high)
{
    return constrain_value(amt, low, high);
}

/**
 * @brief Clamps uint64_t value to specified range
 * @param[in] amt Value to constrain
 * @param[in] low Minimum allowed value
 * @param[in] high Maximum allowed value
 * @return Constrained value in range [low, high]
 * @see constrain_value()
 */
inline uint64_t constrain_uint64(const uint64_t amt, const uint64_t low, const uint64_t high)
{
    return constrain_value(amt, low, high);
}

/**
 * @brief Clamps double value to specified range
 * @param[in] amt Value to constrain
 * @param[in] low Minimum allowed value
 * @param[in] high Maximum allowed value
 * @return Constrained value in range [low, high]
 * @see constrain_value()
 */
inline double constrain_double(const double amt, const double low, const double high)
{
    return constrain_value(amt, low, high);
}

/**
 * @brief Convert degrees to radians (double precision)
 * @param[in] deg Angle in degrees
 * @return Angle in radians
 * @note Uses DEG_TO_RAD constant (π/180)
 */
// degrees -> radians
static inline constexpr double radians(double deg)
{
    return deg * DEG_TO_RAD;
}

/**
 * @brief Convert degrees to radians (float precision)
 * @param[in] deg Angle in degrees
 * @return Angle in radians
 * @note Uses DEG_TO_RAD constant (π/180)
 */
static inline constexpr float radians(float deg)
{
    return deg * DEG_TO_RAD;
}

/**
 * @brief Convert degrees to radians (integer input)
 * @param[in] deg Angle in degrees
 * @return Angle in radians (float)
 * @note Uses DEG_TO_RAD constant (π/180)
 */
static inline constexpr float radians(int deg)
{
    return deg * DEG_TO_RAD;
}

/**
 * @brief Convert centidegrees to radians
 * @param[in] cdeg Angle in centidegrees (1/100 degree)
 * @return Angle in radians
 * @note Uses CDEG_TO_RAD constant (π/18000)
 * @note 18000 centidegrees = 180 degrees = π radians
 */
// centidegrees -> radians
static inline constexpr float cd_to_rad(float cdeg)
{
    return cdeg * CDEG_TO_RAD;
}

/**
 * @brief Convert radians to centidegrees
 * @param[in] rad Angle in radians
 * @return Angle in centidegrees (1/100 degree)
 * @note Uses RAD_TO_CDEG constant (18000/π)
 */
// radians -> centidegrees
static inline constexpr float rad_to_cd(float rad)
{
    return rad * RAD_TO_CDEG;
}

/**
 * @brief Convert radians to degrees
 * @param[in] rad Angle in radians
 * @return Angle in degrees
 * @note Uses RAD_TO_DEG constant (180/π)
 */
// radians -> degrees
static inline constexpr float degrees(float rad)
{
    return rad * RAD_TO_DEG;
}

/**
 * @brief Square function (x²)
 * 
 * @details Computes the square of a value. Generic template version that
 *          converts input to ftype (float or double depending on EKF precision).
 * 
 * @tparam T Input type (any numeric type)
 * @param[in] val Value to square
 * @return val² as ftype (float or double)
 * 
 * @note ftype is float or double depending on HAL_WITH_EKF_DOUBLE
 * @see norm(), pythagorous2(), pythagorous3()
 */
template<typename T>
ftype sq(const T val)
{
    ftype v = static_cast<ftype>(val);
    return v*v;
}
/**
 * @brief Square function for float (optimized)
 * @param[in] val Float value to square
 * @return val²
 */
static inline constexpr float sq(const float val)
{
    return val*val;
}

/**
 * @brief Variadic template for calculating sum of squares (squared norm)
 * 
 * @details Recursively computes sum of squares of all arguments:
 *          sq(a, b, c) = a² + b² + c²
 *          Used for vector magnitude calculations without constructing vector objects.
 * 
 * @tparam T Type of first parameter
 * @tparam Params Types of remaining parameters
 * @param[in] first First value
 * @param[in] parameters Remaining values
 * @return Sum of squares of all arguments
 * 
 * @note Used in pythagorous2(), pythagorous3(), norm()
 * @see norm()
 */
template<typename T, typename... Params>
ftype sq(const T first, const Params... parameters)
{
    return sq(first) + sq(parameters...);
}

/**
 * @brief Variadic template for calculating vector magnitude (Pythagorean theorem)
 * 
 * @details Computes sqrt(a² + b² + c² + ...) for any number of arguments.
 *          Generalized Pythagorean theorem for n-dimensional vectors.
 *          Example: norm(3, 4) = 5, norm(1, 2, 2) = 3
 * 
 * @tparam T Type of first parameter
 * @tparam U Type of second parameter
 * @tparam Params Types of remaining parameters
 * @param[in] first First component
 * @param[in] second Second component
 * @param[in] parameters Additional components (optional)
 * @return Magnitude sqrt(sum of squares)
 * 
 * @note Requires at least 2 arguments
 * @note Uses sqrtF() which adapts to float/double precision
 * 
 * @see sq(), pythagorous2(), pythagorous3(), safe_sqrt()
 */
template<typename T, typename U, typename... Params>
ftype norm(const T first, const U second, const Params... parameters)
{
    return sqrtF(sq(first, second, parameters...));
}

/**
 * @brief Returns minimum of two values
 * 
 * @details Template function that returns the smaller of two values.
 *          Uses decltype for automatic return type deduction.
 * 
 * @tparam A Type of first parameter
 * @tparam B Type of second parameter
 * @param[in] one First value
 * @param[in] two Second value
 * @return Smaller of the two values
 * 
 * @note Replaces standard MIN macro with type-safe template
 * @see MAX()
 */
#undef MIN
template<typename A, typename B>
static inline auto MIN(const A &one, const B &two) -> decltype(one < two ? one : two)
{
    return one < two ? one : two;
}

/**
 * @brief Returns maximum of two values
 * 
 * @details Template function that returns the larger of two values.
 *          Uses decltype for automatic return type deduction.
 * 
 * @tparam A Type of first parameter
 * @tparam B Type of second parameter
 * @param[in] one First value
 * @param[in] two Second value
 * @return Larger of the two values
 * 
 * @note Replaces standard MAX macro with type-safe template
 * @see MIN()
 */
#undef MAX
template<typename A, typename B>
static inline auto MAX(const A &one, const B &two) -> decltype(one > two ? one : two)
{
    return one > two ? one : two;
}

/**
 * @brief Convert frequency in Hz to period in nanoseconds
 * @param[in] freq Frequency in Hz
 * @return Period in nanoseconds
 * @note 1 Hz = 1,000,000,000 nsec period
 */
inline constexpr uint32_t hz_to_nsec(uint32_t freq)
{
    return AP_NSEC_PER_SEC / freq;
}

/**
 * @brief Convert period in nanoseconds to frequency in Hz
 * @param[in] nsec Period in nanoseconds
 * @return Frequency in Hz
 */
inline constexpr uint32_t nsec_to_hz(uint32_t nsec)
{
    return AP_NSEC_PER_SEC / nsec;
}

/**
 * @brief Convert microseconds to nanoseconds
 * @param[in] usec Time in microseconds
 * @return Time in nanoseconds
 * @note 1 µsec = 1000 nsec
 */
inline constexpr uint32_t usec_to_nsec(uint32_t usec)
{
    return usec * AP_NSEC_PER_USEC;
}

/**
 * @brief Convert nanoseconds to microseconds
 * @param[in] nsec Time in nanoseconds
 * @return Time in microseconds
 * @note 1000 nsec = 1 µsec
 */
inline constexpr uint32_t nsec_to_usec(uint32_t nsec)
{
    return nsec / AP_NSEC_PER_USEC;
}

/**
 * @brief Convert frequency in Hz to period in microseconds
 * @param[in] freq Frequency in Hz
 * @return Period in microseconds
 * @note 1 Hz = 1,000,000 µsec period
 */
inline constexpr uint32_t hz_to_usec(uint32_t freq)
{
    return AP_USEC_PER_SEC / freq;
}

/**
 * @brief Convert period in microseconds to frequency in Hz
 * @param[in] usec Period in microseconds
 * @return Frequency in Hz
 */
inline constexpr uint32_t usec_to_hz(uint32_t usec)
{
    return AP_USEC_PER_SEC / usec;
}

/**
 * @brief Linear interpolation (lerp) mapping input range to output range
 * 
 * @details Maps an input value from range [input_low, input_high] to output range
 *          [output_low, output_high] proportionally. Either polarity is supported,
 *          so input_low can be higher than input_high.
 *          
 *          Formula: output = output_low + (input_value - input_low) * 
 *                   (output_high - output_low) / (input_high - input_low)
 *          
 *          Example: linear_interpolate(0, 100, 5, 0, 10) = 50
 *          Maps value 5 in range [0,10] to 50 in range [0,100]
 * 
 * @param[in] output_low  Output value when input is at input_low
 * @param[in] output_high Output value when input is at input_high
 * @param[in] input_value Input value to map
 * @param[in] input_low   Lower bound of input range
 * @param[in] input_high  Upper bound of input range
 * @return Mapped value in range [output_low, output_high]
 * 
 * @note No clamping - extrapolates if input_value outside [input_low, input_high]
 * @note Used in RC input processing, parameter scaling, trajectory generation
 * 
 * @see norm(), constrain_value()
 */
float linear_interpolate(float output_low, float output_high,
                         float input_value,
                         float input_low, float input_high);

/**
 * @brief Cubic exponential curve generator for stick input shaping
 * 
 * @details Applies exponential curve to input, providing finer control near center
 *          and coarser control at extremes. Used for pilot stick feel customization.
 *          alpha=0 gives linear response, alpha=1 gives maximum expo.
 * 
 * @param[in] alpha Expo coefficient in range [0,1], 0=linear, 1=max expo
 * @param[in] input Input value in range [-1, 1]
 * @return Shaped output in range [-1, 1]
 * 
 * @note Used in RC input processing for improved stick feel
 * @see throttle_curve()
 */
float expo_curve(float alpha, float input);

/**
 * @brief Throttle curve generator with mid-stick and expo adjustment
 * 
 * @details Generates throttle curve that passes through specified mid-point with
 *          exponential shaping. Used for throttle stick feel customization where
 *          thr_mid sets hover throttle point.
 * 
 * @param[in] thr_mid Output value at mid stick position (typically hover throttle)
 * @param[in] alpha   Expo coefficient for curve shaping
 * @param[in] thr_in  Input throttle position in range [0, 1]
 * @return Shaped throttle output
 * 
 * @note Commonly used in multicopter flight modes
 * @note thr_mid typically 0.5 for hover throttle at mid-stick
 * 
 * @see expo_curve()
 */
float throttle_curve(float thr_mid, float alpha, float thr_in);

/**
 * @brief Fast pseudo-random 16-bit generator
 * 
 * @details Simple and fast 16-bit pseudo-random number generator.
 *          Not cryptographically secure, suitable for simulation and testing.
 * 
 * @return Random 16-bit unsigned integer
 * 
 * @note Used in SITL simulation for sensor noise
 * @note Not suitable for security applications
 * 
 * @see rand_float(), rand_vec3f()
 */
uint16_t get_random16(void);

/**
 * @brief Generate random float between -1 and 1, for use in SITL
 * 
 * @details Produces uniformly distributed random float in range [-1.0, 1.0].
 *          Used for simulation of sensor noise and disturbances.
 * 
 * @return Random float in range [-1.0, 1.0]
 * 
 * @note For SITL simulation only, not cryptographically secure
 * @see rand_vec3f(), get_random16()
 */
float rand_float(void);

/**
 * @brief Generate random Vector3f with each component between -1.0 and 1.0
 * 
 * @details Creates 3D vector with uniformly distributed random components.
 *          Used for simulating 3D disturbances and noise vectors.
 * 
 * @return Vector3f with each component in range [-1.0, 1.0]
 * 
 * @note For SITL simulation only
 * @see rand_float(), get_random16()
 */
Vector3f rand_vec3f(void);

/**
 * @brief Return true if two rotations are equal
 * 
 * @details Compares two rotation enumerations to determine if they represent
 *          the same physical rotation. Handles rotation equivalences.
 * 
 * @param[in] r1 First rotation
 * @param[in] r2 Second rotation
 * @return true if rotations are equivalent, false otherwise
 * 
 * @see enum Rotation in rotations.h
 */
bool rotation_equal(enum Rotation r1, enum Rotation r2) WARN_IF_UNUSED;

/**
 * @brief Return velocity correction (in m/s in NED frame) for sensor's position offset
 * 
 * @details Computes velocity correction needed when a sensor is offset from the vehicle's
 *          center of rotation. This correction accounts for rotational motion causing
 *          additional velocity at the sensor location.
 *          
 *          Formula: v_correction = angular_rate × sensor_offset
 *          
 *          This correction should be added to the sensor's NED measurement to reference
 *          it to the vehicle center.
 * 
 * @param[in] sensor_offset_bf Sensor position offset in body frame (Forward, Right, Down) in meters
 * @param[in] rot_ef_to_bf     Rotation matrix from earth-frame (NED) to body frame
 * @param[in] angular_rate     Vehicle angular rate in rad/s
 * @return Velocity correction in m/s in NED frame to add to sensor measurement
 * 
 * @note sensor_offset_bf uses body frame: Forward=X, Right=Y, Down=Z
 * @note Output is in NED (North-East-Down) earth frame
 * @note Essential for accurate velocity estimation with offset sensors (GPS, optical flow)
 * 
 * @see AP_AHRS, AP_InertialNav
 */
Vector3F get_vel_correction_for_sensor_offset(const Vector3F &sensor_offset_bf, const Matrix3F &rot_ef_to_bf, const Vector3F &angular_rate);

/**
 * @brief Computes low-pass filter alpha coefficient from sample time and cutoff frequency
 * 
 * @details Calculates the alpha coefficient for a first-order low-pass filter:
 *          alpha = dt / (dt + 1/(2*π*cutoff_freq))
 *          
 *          Used with discrete-time filter update:
 *          output = output * (1 - alpha) + input * alpha
 *          
 *          Lower cutoff_freq gives more filtering (smoother but more lag).
 *          Higher cutoff_freq gives less filtering (noisier but more responsive).
 * 
 * @param[in] dt          Sample time in seconds
 * @param[in] cutoff_freq Cutoff frequency in Hz (-3dB point)
 * @return Alpha coefficient in range [0, 1] for filter update equation
 * 
 * @note cutoff_freq is -3dB frequency (where gain is 0.707)
 * @note alpha near 0 gives heavy filtering, near 1 gives light filtering
 * @note Used extensively in sensor filtering and control loops
 * 
 * @see Filter library for more sophisticated filters
 */
float calc_lowpass_alpha_dt(float dt, float cutoff_freq);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
/**
 * @brief Fill array of floats with NaN, used to invalidate memory in SITL
 * @param[out] f     Pointer to float array
 * @param[in]  count Number of elements to fill
 * @note SITL only - helps detect use of uninitialized data
 */
void fill_nanf(float *f, uint16_t count);
/**
 * @brief Fill array of doubles with NaN, used to invalidate memory in SITL
 * @param[out] f     Pointer to double array
 * @param[in]  count Number of elements to fill
 * @note SITL only - helps detect use of uninitialized data
 */
void fill_nanf(double *f, uint16_t count);
#endif

/**
 * @brief Convert 16-bit fixed-point to float
 * 
 * @details Converts fixed-point representation to floating-point.
 *          Fixed-point format: integer bits + fractional bits = 16 total bits.
 *          From: https://embeddedartistry.com/blog/2018/07/12/simple-fixed-point-conversion-in-c/
 * 
 * @param[in] input          16-bit fixed-point value
 * @param[in] fractional_bits Number of fractional bits (default 8 = 8.8 format)
 * @return Float representation
 * 
 * @note Default 8 fractional bits gives range [-128, 127.996] with 1/256 resolution
 * @see float2fixed()
 */
float fixed2float(const uint16_t input, const uint8_t fractional_bits = 8);
/**
 * @brief Convert float to 16-bit fixed-point
 * 
 * @details Converts floating-point to fixed-point representation.
 *          Fixed-point format: integer bits + fractional bits = 16 total bits.
 * 
 * @param[in] input          Float value to convert
 * @param[in] fractional_bits Number of fractional bits (default 8 = 8.8 format)
 * @return 16-bit fixed-point representation
 * 
 * @note Value will be clamped to representable range
 * @see fixed2float()
 */
uint16_t float2fixed(const float input, const uint8_t fractional_bits = 8);

/**
 * @brief Calculate turn rate in deg/s given bank angle and airspeed for fixed-wing
 * 
 * @details Computes coordinated turn rate for fixed-wing aircraft using:
 *          turn_rate = (g * tan(bank_angle)) / airspeed
 *          where g is gravitational acceleration (9.80665 m/s²)
 * 
 * @param[in] bank_angle_deg Bank angle in degrees
 * @param[in] airspeed       Airspeed in m/s
 * @return Turn rate in degrees/second
 * 
 * @note Used in fixed-wing navigation and L1 controller
 * @note Assumes coordinated (no-slip) turn
 * 
 * @see AP_L1_Control, APM_Control
 */
float fixedwing_turn_rate(float bank_angle_deg, float airspeed);

/**
 * @brief Convert degrees Fahrenheit to Kelvin
 * @param[in] temp_f Temperature in degrees Fahrenheit
 * @return Temperature in Kelvin
 * @note Formula: K = (F - 32) * 5/9 + 273.15
 */
float degF_to_Kelvin(float temp_f);

/**
 * @brief Safely convert float to int16_t with range clamping
 * @param[in] v Float value
 * @return int16_t value, clamped to [INT16_MIN, INT16_MAX]
 * @note Prevents undefined behavior from out-of-range conversions
 */
int16_t float_to_int16(const float v);
/**
 * @brief Safely convert float to uint16_t with range clamping
 * @param[in] v Float value
 * @return uint16_t value, clamped to [0, UINT16_MAX]
 * @note Negative values clamped to 0
 */
uint16_t float_to_uint16(const float v);
/**
 * @brief Safely convert float to int32_t with range clamping
 * @param[in] v Float value
 * @return int32_t value, clamped to [INT32_MIN, INT32_MAX]
 * @note Prevents undefined behavior from out-of-range conversions
 */
int32_t float_to_int32(const float v);
/**
 * @brief Safely convert float to uint32_t with range clamping
 * @param[in] v Float value
 * @return uint32_t value, clamped to [0, UINT32_MAX]
 * @note Negative values clamped to 0
 */
uint32_t float_to_uint32(const float v);
/**
 * @brief Safely convert double to uint32_t with range clamping
 * @param[in] v Double value
 * @return uint32_t value, clamped to [0, UINT32_MAX]
 * @note Negative values clamped to 0
 */
uint32_t double_to_uint32(const double v);
/**
 * @brief Safely convert double to int32_t with range clamping
 * @param[in] v Double value
 * @return int32_t value, clamped to [INT32_MIN, INT32_MAX]
 */
int32_t double_to_int32(const double v);

/**
 * @brief Convert float to int32_t bit pattern (little-endian) without type punning
 * 
 * @details Reinterprets float bits as int32_t without breaking strict aliasing rules.
 *          Used for binary serialization and bit-level operations.
 * 
 * @param[in] value Float value to reinterpret
 * @return int32_t with same bit pattern as input float
 * 
 * @note Does not convert the numeric value, reinterprets bit representation
 * @see int32_to_float_le()
 */
int32_t float_to_int32_le(const float& value) WARN_IF_UNUSED;

/**
 * @brief Convert uint32_t bit pattern to float (little-endian) without type punning
 * 
 * @details Reinterprets uint32_t bits as float without breaking strict aliasing rules.
 *          Used for binary deserialization and bit-level operations.
 * 
 * @param[in] value uint32_t value to reinterpret
 * @return float with same bit pattern as input uint32_t
 * 
 * @note Does not convert the numeric value, reinterprets bit representation
 * @see float_to_int32_le()
 */
float int32_to_float_le(const uint32_t& value) WARN_IF_UNUSED;

/**
 * @brief Convert uint64_t bit pattern to double (little-endian) without type punning
 * 
 * @details Reinterprets uint64_t bits as double without breaking strict aliasing rules.
 *          Used for binary deserialization and bit-level operations.
 * 
 * @param[in] value uint64_t value to reinterpret
 * @return double with same bit pattern as input uint64_t
 * 
 * @note Does not convert the numeric value, reinterprets bit representation
 * @see float_to_int32_le()
 */
double uint64_to_double_le(const uint64_t& value) WARN_IF_UNUSED;

/**
 * @brief Extract two's complement signed value from first 'length' bits of uint32_t
 * 
 * @details Interprets the first 'length' bits of raw as a two's complement signed integer.
 *          Useful for parsing packed binary sensor data where values may be non-standard bit widths.
 *          With thanks to betaflight.
 * 
 * @param[in] raw    Raw unsigned value containing packed bits
 * @param[in] length Number of bits in the two's complement value (1-32)
 * @return Signed integer extracted from first 'length' bits
 * 
 * @note Example: get_twos_complement(0xFF, 8) = -1
 * @note Used in sensor driver parsing (e.g., magnetometer, IMU raw data)
 */
int32_t get_twos_complement(uint32_t raw, uint8_t length) WARN_IF_UNUSED;
