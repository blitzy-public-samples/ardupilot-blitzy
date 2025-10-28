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

/**
 * @file vectorN.h
 * @brief N-dimensional vector template for small fixed-size vectors
 * 
 * @details Provides a template class for fixed-size N-dimensional vectors with
 *          basic vector algebra operations. Designed for small vectors (typically
 *          N=2-6) used in EKF state vectors and small linear algebra computations.
 *          
 *          Storage is a fixed-size array allocated at compile time, ensuring no
 *          dynamic memory allocation, making it suitable for embedded systems.
 *          
 *          For optimized 2D and 3D operations, use Vector2 and Vector3 classes.
 * 
 * Source: libraries/AP_Math/vectorN.h
 */

#pragma once

#include <cmath>
#include <string.h>
#include "matrixN.h"

#ifndef MATH_CHECK_INDEXES
# define MATH_CHECK_INDEXES 0
#endif

#if MATH_CHECK_INDEXES
#include <assert.h>
#endif

template <typename T, uint8_t N>
class MatrixN;

/**
 * @class VectorN
 * @brief Fixed-size N-dimensional vector template for general vector operations
 * 
 * @details Provides basic vector algebra for dimensions N (typically 2-5). Used for
 *          EKF state vectors and small linear algebra operations where the dimension
 *          is known at compile time.
 *          
 *          This class supports standard vector operations including:
 *          - Element access with optional bounds checking
 *          - Vector addition, subtraction, and scalar multiplication
 *          - Dot product (scalar product)
 *          - Matrix-vector multiplication
 *          
 *          Use cases:
 *          - EKF state vectors (typically N=4,5,6)
 *          - Small system solving
 *          - Control state vectors
 *          
 * @tparam T Numeric type (float or double)
 * @tparam N Vector dimension (fixed at compile time)
 * 
 * @note Storage uses fixed-size array _v[N] for compile-time allocation
 * @note No dynamic memory allocation (suitable for embedded systems)
 * @note Typically used with N=4,5,6 for small state vectors
 * 
 * @warning MATH_CHECK_INDEXES=1 enables runtime bounds checking but adds overhead
 * 
 * @see Vector2 for optimized 2D operations
 * @see Vector3 for optimized 3D operations
 * @see MatrixN for matrix operations
 */
template <typename T, uint8_t N>
class VectorN
{
public:
    /**
     * @brief Default constructor - initializes all elements to zero
     * 
     * @details Creates a zero vector by initializing all N elements to the
     *          default value of type T (typically 0 for numeric types).
     */
    inline VectorN() {
        for (auto i = 0; i < N; i++) {
            _v[i] = T{};
        }
    }

    /**
     * @brief Construct vector from array
     * 
     * @details Copies N elements from the provided array into the vector.
     * 
     * @param[in] v Pointer to array of at least N elements of type T
     * 
     * @note No bounds checking - caller must ensure array has at least N elements
     */
    inline VectorN(const T *v) {
        memcpy(_v, v, sizeof(T)*N);
    }
    
    /**
     * @brief Element access by index (non-const)
     * 
     * @details Provides direct access to vector elements for reading and writing.
     * 
     * @param[in] i Index of element to access, valid range [0, N-1]
     * 
     * @return Reference to element at index i
     * 
     * @note Bounds checking only performed when MATH_CHECK_INDEXES is enabled
     * @warning Out-of-bounds access causes undefined behavior when MATH_CHECK_INDEXES=0
     */
    inline T & operator[](uint8_t i) {
#if MATH_CHECK_INDEXES
        assert(i >= 0 && i < N);
#endif
        return _v[i];
    }

    /**
     * @brief Element access by index (const)
     * 
     * @details Provides read-only access to vector elements.
     * 
     * @param[in] i Index of element to access, valid range [0, N-1]
     * 
     * @return Const reference to element at index i
     * 
     * @note Bounds checking only performed when MATH_CHECK_INDEXES is enabled
     * @warning Out-of-bounds access causes undefined behavior when MATH_CHECK_INDEXES=0
     */
    inline const T & operator[](uint8_t i) const {
#if MATH_CHECK_INDEXES
        assert(i >= 0 && i < N);
#endif
        return _v[i];
    }

    /**
     * @brief Equality test
     * 
     * @details Tests if all elements of this vector equal corresponding elements
     *          of another vector. Uses element-wise comparison with operator!=.
     * 
     * @param[in] v Vector to compare against
     * 
     * @return true if all elements are equal, false otherwise
     */
    bool operator ==(const VectorN<T,N> &v) const {
        for (uint8_t i=0; i<N; i++) {
            if (_v[i] != v[i]) return false;
        }
        return true;
    }

    /**
     * @brief Set all elements to zero
     * 
     * @details Efficiently zeros the entire vector using memset.
     * 
     * @note Uses memset for performance - suitable for numeric types
     */
    inline void zero()
    {
        memset(_v, 0, sizeof(T)*N);
    }

    /**
     * @brief Unary negation operator
     * 
     * @details Returns a new vector with all elements negated (multiplied by -1).
     * 
     * @return Negated vector (-v for each element)
     */
    VectorN<T,N> operator -(void) const {
        VectorN<T,N> v2;
        for (uint8_t i=0; i<N; i++) {
            v2[i] = - _v[i];
        }
        return v2;
    }

    /**
     * @brief Vector addition
     * 
     * @details Performs element-wise addition of two vectors.
     * 
     * @param[in] v Vector to add to this vector
     * 
     * @return Result vector where result[i] = this[i] + v[i]
     */
    VectorN<T,N> operator +(const VectorN<T,N> &v) const {
        VectorN<T,N> v2;
        for (uint8_t i=0; i<N; i++) {
            v2[i] = _v[i] + v[i];
        }
        return v2;
    }

    /**
     * @brief Vector subtraction
     * 
     * @details Performs element-wise subtraction of two vectors.
     * 
     * @param[in] v Vector to subtract from this vector
     * 
     * @return Result vector where result[i] = this[i] - v[i]
     */
    VectorN<T,N> operator -(const VectorN<T,N> &v) const {
        VectorN<T,N> v2;
        for (uint8_t i=0; i<N; i++) {
            v2[i] = _v[i] - v[i];
        }
        return v2;
    }

    /**
     * @brief Scalar multiplication
     * 
     * @details Multiplies all vector elements by a scalar value (uniform scaling).
     * 
     * @param[in] num Scalar value to multiply by
     * 
     * @return Scaled vector where result[i] = this[i] * num
     */
    VectorN<T,N> operator *(const T num) const {
        VectorN<T,N> v2;
        for (uint8_t i=0; i<N; i++) {
            v2[i] = _v[i] * num;
        }
        return v2;
    }

    /**
     * @brief Scalar division
     * 
     * @details Divides all vector elements by a scalar value (uniform scaling).
     * 
     * @param[in] num Scalar value to divide by (non-zero)
     * 
     * @return Scaled vector where result[i] = this[i] / num
     * 
     * @warning Division by zero causes undefined behavior
     */
    VectorN<T,N> operator  /(const T num) const {
        VectorN<T,N> v2;
        for (uint8_t i=0; i<N; i++) {
            v2[i] = _v[i] / num;
        }
        return v2;
    }

    /**
     * @brief In-place vector addition
     * 
     * @details Adds another vector to this vector, modifying this vector.
     * 
     * @param[in] v Vector to add to this vector
     * 
     * @return Reference to this vector after addition
     */
    VectorN<T,N> &operator +=(const VectorN<T,N> &v) {
        for (uint8_t i=0; i<N; i++) {
            _v[i] += v[i];
        }
        return *this;
    }

    /**
     * @brief In-place vector subtraction
     * 
     * @details Subtracts another vector from this vector, modifying this vector.
     * 
     * @param[in] v Vector to subtract from this vector
     * 
     * @return Reference to this vector after subtraction
     */
    VectorN<T,N> &operator -=(const VectorN<T,N> &v) {
        for (uint8_t i=0; i<N; i++) {
            _v[i] -= v[i];
        }
        return *this;
    }

    /**
     * @brief In-place scalar multiplication
     * 
     * @details Multiplies this vector by a scalar value, modifying this vector.
     * 
     * @param[in] num Scalar value to multiply by
     * 
     * @return Reference to this vector after multiplication
     */
    VectorN<T,N> &operator *=(const T num) {
        for (uint8_t i=0; i<N; i++) {
            _v[i] *= num;
        }
        return *this;
    }

    /**
     * @brief In-place scalar division
     * 
     * @details Divides this vector by a scalar value, modifying this vector.
     * 
     * @param[in] num Scalar value to divide by (non-zero)
     * 
     * @return Reference to this vector after division
     * 
     * @warning Division by zero causes undefined behavior
     */
    VectorN<T,N> &operator /=(const T num) {
        for (uint8_t i=0; i<N; i++) {
            _v[i] /= num;
        }
        return *this;
    }

    /**
     * @brief Dot product (scalar product)
     * 
     * @details Computes the dot product of two vectors, which is the sum of
     *          element-wise products: result = sum(this[i] * v[i]).
     * 
     * @param[in] v Other vector for dot product computation
     * 
     * @return Scalar result equal to sum of element-wise products
     */
    T operator *(const VectorN<T,N> &v) const {
        float ret = 0;
        for (uint8_t i=0; i<N; i++) {
            ret += _v[i] * v._v[i];
        }
        return ret;
    }
    
    /**
     * @brief Matrix-vector multiplication in-place
     * 
     * @details Computes matrix-vector product and stores result in this vector.
     *          This vector is set to: this = A * B
     * 
     * @param[in] A Matrix of size NxN
     * @param[in] B Vector of size N
     * 
     * @note This vector's previous contents are overwritten
     * @note Result stored in-place for memory efficiency
     */
    void mult(const MatrixN<T,N> &A, const VectorN<T,N> &B) {
        for (uint8_t i = 0; i < N; i++) {
            _v[i] = 0;
            for (uint8_t k = 0; k < N; k++) {
                _v[i] += A.v[i][k] * B[k];
            }
        }
    }

protected:
    T _v[N];
};
