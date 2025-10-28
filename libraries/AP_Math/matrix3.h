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

// Inspired by:
/****************************************
 * 3D Vector Classes
 * By Bill Perone (billperone@yahoo.com)
 */

/**
 * @file matrix3.h
 * @brief 3x3 matrix implementation for rotation matrices and linear transformations
 * 
 * @details This file provides a template-based 3x3 matrix class used extensively
 *          throughout ArduPilot for representing rotations (Direction Cosine Matrix),
 *          coordinate frame transformations, and general linear algebra operations.
 *          The matrix is organized in row-major form matching standard array indexing.
 *          
 *          Common use cases:
 *          - DCM (Direction Cosine Matrix) in AHRS for attitude representation
 *          - Coordinate frame transformations (body ↔ earth/NED frames)
 *          - Sensor orientation corrections
 *          - Inertial navigation transformations
 *          
 *          The template is instantiated for multiple numeric types including
 *          float and double with explicit instantiations in matrix3.cpp.
 * 
 * @note Matrix organization: row-major form with three Vector3 rows (a, b, c)
 * @note For rotation matrices: rows are unit vectors, mutually perpendicular (orthonormal)
 * 
 * @see quaternion.h for alternative rotation representation
 * @see AP_AHRS for DCM usage in attitude and heading reference system
 * @see vector3.h for the Vector3 type used to represent matrix rows
 */
#pragma once

#include "ftype.h"

#include "vector3.h"
#include "vector2.h"

template <typename T>
class Vector3;

/**
 * @class Matrix3
 * @brief 3x3 matrix template class for rotation matrices and linear transformations
 * 
 * @tparam T Numeric type (typically float or double)
 * 
 * @details Matrix3 provides a comprehensive 3x3 matrix implementation used primarily
 *          for rotation matrices (Direction Cosine Matrix) and coordinate transformations.
 *          The matrix is stored in row-major form with three Vector3 rows: a, b, c.
 *          
 *          Matrix structure:
 *          \code
 *          [ a.x  a.y  a.z ]     [ row 0 ]
 *          [ b.x  b.y  b.z ]  =  [ row 1 ]
 *          [ c.x  c.y  c.z ]     [ row 2 ]
 *          \endcode
 *          
 *          For rotation matrices (DCM):
 *          - Each row is a unit vector (length = 1)
 *          - Rows are mutually perpendicular (orthogonal)
 *          - The matrix represents a transformation from one coordinate frame to another
 *          - Typically maps from body frame to earth frame (NED) or vice versa
 *          
 *          Row access:
 *          - a = first row (row 0)
 *          - b = second row (row 1)
 *          - c = third row (row 2)
 *          - Individual elements: a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z
 * 
 * @note Coordinate frame convention: When used as a DCM, typically represents
 *       body frame vectors expressed in earth frame (NED: North-East-Down)
 * @note Rotation matrices are orthonormal and require periodic normalization
 *       to prevent accumulation of numerical errors
 * @warning Use normalize() periodically when integrating gyroscope rates to
 *          prevent the matrix from becoming non-orthonormal due to numerical drift
 * 
 * @see Quaternion for an alternative rotation representation without gimbal lock
 * @see AP_AHRS for usage in attitude estimation
 * @see from_euler() and to_euler() for Euler angle conversions
 */
template <typename T>
class Matrix3 {
public:

    /**
     * @brief Vectors comprising the rows of the matrix
     * 
     * @details The three public row vectors (a, b, c) allow direct access to
     *          matrix elements. For rotation matrices, these rows represent the
     *          basis vectors of the rotated coordinate frame expressed in the
     *          reference frame.
     */
    Vector3<T>        a, b, c;

    /**
     * @brief Default constructor - creates a zero matrix
     * 
     * @details Constructs a matrix with all elements initialized to zero.
     *          The Vector3 default constructor zeros all vector elements.
     * 
     * @note For an identity matrix, call identity() after construction
     * @see identity()
     */
    constexpr Matrix3() {}

    /**
     * @brief Constructor from three row vectors
     * 
     * @param a0 First row vector (row 0)
     * @param b0 Second row vector (row 1)
     * @param c0 Third row vector (row 2)
     * 
     * @details Constructs a matrix from three Vector3 objects representing
     *          the matrix rows in row-major order.
     */
    constexpr Matrix3(const Vector3<T> &a0, const Vector3<T> &b0, const Vector3<T> &c0)
        : a(a0)
        , b(b0)
        , c(c0) {}

    /**
     * @brief Constructor from nine scalar elements
     * 
     * @param ax First row, first column (element [0,0])
     * @param ay First row, second column (element [0,1])
     * @param az First row, third column (element [0,2])
     * @param bx Second row, first column (element [1,0])
     * @param by Second row, second column (element [1,1])
     * @param bz Second row, third column (element [1,2])
     * @param cx Third row, first column (element [2,0])
     * @param cy Third row, second column (element [2,1])
     * @param cz Third row, third column (element [2,2])
     * 
     * @details Constructs a matrix by specifying all nine elements explicitly
     *          in row-major order. Elements are organized as:
     *          \code
     *          [ ax ay az ]
     *          [ bx by bz ]
     *          [ cx cy cz ]
     *          \endcode
     */
    constexpr Matrix3(const T ax, const T ay, const T az,
                      const T bx, const T by, const T bz,
                      const T cx, const T cy, const T cz)
        : a(ax,ay,az)
        , b(bx,by,bz)
        , c(cx,cy,cz) {}

    /**
     * @brief Function call operator for setting matrix rows
     * 
     * @param a0 New first row vector
     * @param b0 New second row vector
     * @param c0 New third row vector
     * 
     * @details Allows matrix initialization using function call syntax:
     *          matrix(a_vec, b_vec, c_vec);
     */
    void operator        () (const Vector3<T> &a0, const Vector3<T> &b0, const Vector3<T> &c0)
    {
        a = a0; b = b0; c = c0;
    }

    /**
     * @brief Equality test operator
     * 
     * @param m Matrix to compare against
     * @return true if all elements are equal (within epsilon tolerance), false otherwise
     * 
     * @details Compares all three row vectors using Vector3 equality operator,
     *          which uses epsilon tolerance for floating-point comparison.
     * 
     * @note Uses Vector3::operator== which applies FLT_EPSILON tolerance
     */
    bool operator        == (const Matrix3<T> &m)
    {
        return (a==m.a && b==m.b && c==m.c);
    }

    /**
     * @brief Inequality test operator
     * 
     * @param m Matrix to compare against
     * @return true if any elements differ (beyond epsilon tolerance), false otherwise
     * 
     * @details Returns true if any row vector differs from the corresponding
     *          row in the comparison matrix.
     */
    bool operator        != (const Matrix3<T> &m)
    {
        return (a!=m.a || b!=m.b || c!=m.c);
    }

    /**
     * @brief Unary negation operator
     * 
     * @return New matrix with all elements negated
     * 
     * @details Returns a matrix where each element is the negative of the
     *          corresponding element in this matrix.
     */
    Matrix3<T> operator        - (void) const
    {
        return Matrix3<T>(-a,-b,-c);
    }

    /**
     * @brief Matrix addition operator
     * 
     * @param m Matrix to add
     * @return Sum of this matrix and m
     * 
     * @details Performs element-wise addition of two matrices.
     */
    Matrix3<T> operator        + (const Matrix3<T> &m) const
    {
        return Matrix3<T>(a+m.a, b+m.b, c+m.c);
    }

    /**
     * @brief In-place matrix addition operator
     * 
     * @param m Matrix to add to this matrix
     * @return Reference to this matrix after addition
     */
    Matrix3<T> &operator        += (const Matrix3<T> &m)
    {
        return *this = *this + m;
    }

    /**
     * @brief Matrix subtraction operator
     * 
     * @param m Matrix to subtract
     * @return Difference of this matrix and m
     * 
     * @details Performs element-wise subtraction of two matrices.
     */
    Matrix3<T> operator        - (const Matrix3<T> &m) const
    {
        return Matrix3<T>(a-m.a, b-m.b, c-m.c);
    }

    /**
     * @brief In-place matrix subtraction operator
     * 
     * @param m Matrix to subtract from this matrix
     * @return Reference to this matrix after subtraction
     */
    Matrix3<T> &operator        -= (const Matrix3<T> &m)
    {
        return *this = *this - m;
    }

    /**
     * @brief Scalar multiplication operator
     * 
     * @param num Scalar value to multiply by
     * @return New matrix with all elements scaled by num
     * 
     * @details Multiplies every element of the matrix by the scalar value.
     *          Note: For rotation matrices, scaling destroys orthonormality.
     */
    Matrix3<T> operator        * (const T num) const
    {
        return Matrix3<T>(a*num, b*num, c*num);
    }

    /**
     * @brief In-place scalar multiplication operator
     * 
     * @param num Scalar value to multiply by
     * @return Reference to this matrix after scaling
     */
    Matrix3<T> &operator        *= (const T num)
    {
        return *this = *this * num;
    }

    /**
     * @brief Scalar division operator
     * 
     * @param num Scalar value to divide by
     * @return New matrix with all elements divided by num
     * 
     * @details Divides every element of the matrix by the scalar value.
     * @warning Does not check for division by zero
     */
    Matrix3<T> operator        / (const T num) const
    {
        return Matrix3<T>(a/num, b/num, c/num);
    }

    /**
     * @brief In-place scalar division operator
     * 
     * @param num Scalar value to divide by
     * @return Reference to this matrix after division
     * @warning Does not check for division by zero
     */
    Matrix3<T> &operator        /= (const T num)
    {
        return *this = *this / num;
    }

    /**
     * @brief Array subscript operator for row access (non-const)
     * 
     * @param i Row index (0, 1, or 2)
     * @return Reference to row vector at index i
     * 
     * @details Allows matrix rows to be accessed as an array:
     *          row0 = matrix[0], row1 = matrix[1], row2 = matrix[2]
     * 
     * @note Index 0 returns row a, 1 returns row b, 2 returns row c
     * @note Bounds checking performed only when MATH_CHECK_INDEXES is enabled
     * @warning Out-of-bounds access causes undefined behavior in release builds
     */
    Vector3<T> & operator[](uint8_t i) {
        Vector3<T> *_v = &a;
#if MATH_CHECK_INDEXES
        assert(i >= 0 && i < 3);
#endif
        return _v[i];
    }

    /**
     * @brief Array subscript operator for row access (const)
     * 
     * @param i Row index (0, 1, or 2)
     * @return Const reference to row vector at index i
     * 
     * @details Const version of array subscript operator for read-only access.
     * 
     * @note Bounds checking performed only when MATH_CHECK_INDEXES is enabled
     */
    const Vector3<T> & operator[](uint8_t i) const {
        const Vector3<T> *_v = &a;
#if MATH_CHECK_INDEXES
        assert(i >= 0 && i < 3);
#endif
        return _v[i];
    }

    /**
     * @brief Matrix-vector multiplication operator
     * 
     * @param v Vector to multiply (column vector)
     * @return Resulting transformed vector
     * 
     * @details Multiplies this matrix by a column vector: result = M * v
     *          Computes the dot product of each row with the input vector.
     *          For rotation matrices, this transforms a vector from one frame to another.
     * 
     * @note For DCM: transforms from body frame to earth frame
     */
    Vector3<T> operator         *(const Vector3<T> &v) const;

    /**
     * @brief Transpose-vector multiplication
     * 
     * @param v Vector to multiply
     * @return Resulting transformed vector
     * 
     * @details Multiplies the transpose of this matrix by a vector: result = M^T * v
     *          Equivalent to using the columns of M instead of rows.
     *          For rotation matrices (orthonormal), this is the inverse transformation.
     * 
     * @note For DCM: transforms from earth frame to body frame
     * @note For orthonormal matrices: transpose equals inverse
     */
    Vector3<T>                  mul_transpose(const Vector3<T> &v) const;

    /**
     * @brief Matrix-vector multiplication returning 2D result
     * 
     * @param v Vector to multiply
     * @return Vector2 containing only X and Y components of the result
     * 
     * @details Performs matrix-vector multiplication but returns only the
     *          first two components (X and Y). Useful for projections onto
     *          horizontal plane or when Z component is not needed.
     * 
     * @note More efficient than full multiplication when Z is discarded
     */
    Vector2<T> mulXY(const Vector3<T> &v) const;

    /**
     * @brief Extract first column (X column)
     * 
     * @return Vector3 containing the first column elements
     * 
     * @details Returns a vector containing [a.x, b.x, c.x], which is the
     *          first column of the matrix. For rotation matrices, this
     *          represents the X-axis of the rotated frame.
     * 
     * @note Matrix columns are not directly accessible like rows
     */
    Vector3<T>                  colx(void) const
    {
        return Vector3<T>(a.x, b.x, c.x);
    }

    /**
     * @brief Extract second column (Y column)
     * 
     * @return Vector3 containing the second column elements
     * 
     * @details Returns a vector containing [a.y, b.y, c.y], which is the
     *          second column of the matrix. For rotation matrices, this
     *          represents the Y-axis of the rotated frame.
     */
    Vector3<T>        coly(void) const
    {
        return Vector3<T>(a.y, b.y, c.y);
    }

    /**
     * @brief Extract third column (Z column)
     * 
     * @return Vector3 containing the third column elements
     * 
     * @details Returns a vector containing [a.z, b.z, c.z], which is the
     *          third column of the matrix. For rotation matrices, this
     *          represents the Z-axis of the rotated frame.
     */
    Vector3<T>        colz(void) const
    {
        return Vector3<T>(a.z, b.z, c.z);
    }

    /**
     * @brief Matrix-matrix multiplication operator
     * 
     * @param m Matrix to multiply by (right operand)
     * @return Product matrix (this * m)
     * 
     * @details Performs standard matrix multiplication: result[i,j] = sum(this[i,k] * m[k,j])
     *          For rotation matrices, this composes rotations (applies m first, then this).
     * 
     * @note Matrix multiplication is not commutative: A*B ≠ B*A in general
     * @note For rotation composition: result transforms from frame C through B to A
     *       where this matrix is A, m is B, and we're going from C
     */
    Matrix3<T> operator *(const Matrix3<T> &m) const;

    /**
     * @brief In-place matrix multiplication operator
     * 
     * @param m Matrix to multiply by
     * @return Reference to this matrix after multiplication
     * 
     * @details Multiplies this matrix by m and stores result in this: this = this * m
     */
    Matrix3<T> &operator        *=(const Matrix3<T> &m)
    {
        return *this = *this * m;
    }

    /**
     * @brief Return transposed matrix
     * 
     * @return New matrix that is the transpose of this matrix
     * 
     * @details Returns a matrix where rows and columns are swapped.
     *          For rotation matrices (orthonormal), the transpose equals
     *          the inverse, representing the reverse transformation.
     * 
     * @note For orthonormal matrices: M^T = M^(-1)
     * @note Element mapping: result[i,j] = this[j,i]
     */
    Matrix3<T>          transposed(void) const;

    /**
     * @brief Transpose this matrix in place
     * 
     * @details Replaces this matrix with its transpose by swapping rows and columns.
     */
    void transpose(void)
    {
        *this = transposed();
    }

    /**
     * @brief Calculate the determinant of this matrix
     *
     * @return The value of the determinant
     * 
     * @details Computes det(M) using the rule of Sarrus or cofactor expansion.
     *          For rotation matrices, the determinant should be +1 (proper rotation)
     *          or -1 (improper rotation/reflection).
     * 
     * @note For orthonormal matrices: |det| = 1
     * @note Zero determinant indicates singular (non-invertible) matrix
     */
    T det() const;

    /**
     * @brief Calculate the inverse of this matrix
     *
     * @param inv Where to store the result
     *
     * @return true if matrix is invertible, false if singular
     * 
     * @details Computes the matrix inverse using cofactor method.
     *          If the matrix is singular (determinant near zero), the
     *          operation fails and inv is unmodified.
     *          
     *          For rotation matrices, prefer using transposed() which is
     *          more efficient since transpose equals inverse for orthonormal matrices.
     * 
     * @note For rotation matrices: use transposed() instead (faster)
     * @warning Returns false for singular matrices (det ≈ 0)
     */
    bool inverse(Matrix3<T>& inv) const WARN_IF_UNUSED;

    /**
     * @brief Invert this matrix in place if it is invertible
     *
     * @return true if matrix was successfully inverted, false if singular
     * 
     * @details Replaces this matrix with its inverse. If the matrix is
     *          singular, this matrix remains unchanged and false is returned.
     * 
     * @warning Matrix remains unchanged if inversion fails
     */
    bool invert() WARN_IF_UNUSED;

    /**
     * @brief Set all matrix elements to zero
     * 
     * @details Efficiently zeros the entire matrix using memset.
     *          Results in a 3x3 zero matrix (all elements = 0).
     * 
     * @note After calling zero(), the matrix is not a valid rotation matrix
     */
    void        zero(void) {
        memset((void*)this, 0, sizeof(*this));
    }

    /**
     * @brief Set matrix to identity matrix
     * 
     * @details Sets the matrix to the 3x3 identity matrix:
     *          \code
     *          [ 1  0  0 ]
     *          [ 0  1  0 ]
     *          [ 0  0  1 ]
     *          \endcode
     *          
     *          The identity matrix represents no rotation/transformation.
     * 
     * @note Identity matrix: M * I = I * M = M for any matrix M
     * @note For rotations: represents zero rotation (roll=pitch=yaw=0)
     */
    void        identity(void) {
        zero();
        a.x = b.y = c.z = 1;
    }

    /**
     * @brief Check if any matrix elements are NaN
     * 
     * @return true if any element is NaN, false if all elements are valid
     * 
     * @details Checks all three row vectors for NaN (Not-a-Number) values.
     *          NaN can result from invalid operations like 0/0 or sqrt(-1).
     * 
     * @warning Matrices with NaN values are invalid and should not be used
     * @note Useful for detecting numerical errors in rotation matrix updates
     */
    bool        is_nan(void) WARN_IF_UNUSED
    {
        return a.is_nan() || b.is_nan() || c.is_nan();
    }

    /**
     * @brief Create rotation matrix from Euler angles (321 sequence)
     * 
     * @param roll Roll angle in radians (rotation about X-axis)
     * @param pitch Pitch angle in radians (rotation about Y-axis)
     * @param yaw Yaw angle in radians (rotation about Z-axis)
     * 
     * @details Constructs a rotation matrix from Euler angles using the 321
     *          (yaw-pitch-roll or Z-Y-X) rotation sequence. This is the standard
     *          aerospace sequence: yaw first, then pitch, then roll.
     *          
     *          Rotation order: R = R_z(yaw) * R_y(pitch) * R_x(roll)
     *          
     *          The resulting matrix transforms vectors from body frame to earth frame (NED).
     * 
     * @note Angles in radians, not degrees
     * @note 321 sequence is the standard used throughout ArduPilot
     * @note Positive angles follow right-hand rule
     * @see to_euler() for the inverse operation
     */
    void        from_euler(T roll, T pitch, T yaw);

    /**
     * @brief Extract Euler angles from rotation matrix (321 sequence)
     * 
     * @param[out] roll Pointer to store roll angle in radians [-π, π]
     * @param[out] pitch Pointer to store pitch angle in radians [-π/2, π/2]
     * @param[out] yaw Pointer to store yaw angle in radians [-π, π]
     * 
     * @details Extracts Euler angles from this rotation matrix using 321 (Z-Y-X)
     *          convention. The matrix is assumed to represent a rotation from
     *          body frame to earth frame.
     * 
     * @note Singularity at pitch = ±90°: roll and yaw become coupled (gimbal lock)
     * @note Roll range: [-π, π], Pitch range: [-π/2, π/2], Yaw range: [-π, π]
     * @warning Gimbal lock occurs at pitch = ±π/2, where roll and yaw are ambiguous
     * @see from_euler() for constructing matrix from Euler angles
     */
    void        to_euler(T *roll, T *pitch, T *yaw) const;

    /**
     * @brief Create matrix from predefined rotation enumeration
     * 
     * @param rotation Rotation enum value (e.g., ROTATION_NONE, ROTATION_YAW_45)
     * 
     * @details Constructs a rotation matrix from one of the predefined standard
     *          rotations defined in the Rotation enum. These represent common
     *          sensor mounting orientations (e.g., rotated 90°, 180°, or combinations).
     * 
     * @note Used primarily for sensor orientation corrections
     * @see rotations.h for available rotation enum values
     * @see rotate() to apply a rotation to an existing matrix
     */
    void from_rotation(enum Rotation rotation);
    
    /**
     * @brief Calculate Euler angles using 312 convention
     * 
     * @return Vector3 containing [roll, pitch, yaw] in radians
     * 
     * @details Extracts Euler angles using 312 (X-Y-Z) rotation sequence instead
     *          of the standard 321 sequence. This is an alternative parameterization
     *          useful in certain applications.
     *          
     *          See: http://www.atacolorado.com/eulersequences.doc
     * 
     * @note Less commonly used than 321 convention
     * @note Vector components: [0]=roll, [1]=pitch, [2]=yaw
     * @see to_euler() for standard 321 convention
     */
    Vector3<T> to_euler312() const;

    /**
     * @brief Construct matrix from Euler angles in 312 convention
     * 
     * @param roll Roll angle in radians
     * @param pitch Pitch angle in radians
     * @param yaw Yaw angle in radians
     * 
     * @details Creates a rotation matrix from Euler angles using 312 (X-Y-Z)
     *          rotation sequence instead of the standard 321 sequence.
     * 
     * @note Less commonly used than 321 convention
     * @see from_euler() for standard 321 convention
     */
    void from_euler312(T roll, T pitch, T yaw);

    /**
     * @brief Apply additional rotation from gyroscope rates
     * 
     * @param g Rotation rate vector in radians/second (body frame)
     * 
     * @details Applies an incremental rotation to this matrix based on gyroscope
     *          rates. This integrates the angular velocity to update the DCM.
     *          Used in AHRS to update attitude from gyroscope measurements.
     *          
     *          The rotation is applied as: M_new = M_old * R(g * dt)
     *          where dt is implicit in the magnitude of g.
     * 
     * @note Input g typically has units rad/s multiplied by time step
     * @warning Repeated application accumulates numerical errors; call normalize()
     *          periodically to maintain orthonormality
     * @see normalize() to correct accumulated numerical drift
     */
    void        rotate(const Vector3<T> &g);

    /**
     * @brief Create rotation matrix from axis-angle representation
     * 
     * @param v Rotation axis (should be unit vector)
     * @param theta Rotation angle in radians about the axis
     * 
     * @details Constructs a rotation matrix representing a rotation by angle theta
     *          about axis v using Rodrigues' rotation formula.
     *          
     *          Reference: https://en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle
     * 
     * @note v should be a unit vector; if not, the result will be incorrect
     * @note Positive theta follows right-hand rule (thumb along axis)
     * @warning If v is not normalized, the resulting matrix will not be a valid rotation
     * @see Vector3::normalized() to ensure axis is unit length
     */
    void        from_axis_angle(const Vector3<T> &v, T theta);
    
    /**
     * @brief Orthonormalize the rotation matrix
     * 
     * @details Corrects numerical drift in a rotation matrix to restore orthonormality.
     *          Uses a Gram-Schmidt-like process to ensure rows are unit length and
     *          mutually perpendicular. Essential when integrating gyroscope rates,
     *          as numerical errors accumulate over time.
     *          
     *          Process:
     *          1. Corrects rows to be perpendicular
     *          2. Normalizes each row to unit length
     * 
     * @note Should be called periodically (e.g., every few seconds) when using rotate()
     * @warning Without periodic normalization, DCM will drift and become invalid
     * @note Typical usage: normalize every 1-5 seconds in AHRS updates
     */
    void        normalize(void);

    /**
     * @brief Convert matrix to double precision
     * 
     * @return Matrix3<double> with all elements converted to double precision
     * 
     * @details Converts each element of this matrix from type T to double precision.
     *          Each row vector is converted using Vector3::todouble().
     *          Useful when mixing float and double precision matrices.
     * 
     * @note No-op if T is already double
     * @see tofloat() for conversion to single precision
     */
    Matrix3<double> todouble(void) const {
        return Matrix3<double>(a.todouble(), b.todouble(), c.todouble());
    }

    /**
     * @brief Convert matrix to single precision
     * 
     * @return Matrix3<float> with all elements converted to float precision
     * 
     * @details Converts each element of this matrix from type T to single (float) precision.
     *          Each row vector is converted using Vector3::tofloat().
     *          Useful when mixing float and double precision matrices.
     * 
     * @note No-op if T is already float
     * @note May lose precision if converting from double to float
     * @see todouble() for conversion to double precision
     */
    Matrix3<float> tofloat(void) const {
        return Matrix3<float>(a.tofloat(), b.tofloat(), c.tofloat());
    }
};

/**
 * @typedef Matrix3i
 * @brief 3x3 matrix of 16-bit signed integers
 * 
 * @note Integer matrices used for discrete computations or lookup tables
 * @warning Integer division truncates - use with care in algorithms requiring precision
 */
typedef Matrix3<int16_t>                Matrix3i;

/**
 * @typedef Matrix3ui
 * @brief 3x3 matrix of 16-bit unsigned integers
 * 
 * @note Unsigned variant for non-negative values or bit operations
 */
typedef Matrix3<uint16_t>               Matrix3ui;

/**
 * @typedef Matrix3l
 * @brief 3x3 matrix of 32-bit signed integers (long)
 * 
 * @note Larger integer range than Matrix3i for extended precision integer computations
 */
typedef Matrix3<int32_t>                Matrix3l;

/**
 * @typedef Matrix3ul
 * @brief 3x3 matrix of 32-bit unsigned integers (unsigned long)
 * 
 * @note Unsigned 32-bit variant for large positive values
 */
typedef Matrix3<uint32_t>               Matrix3ul;

/**
 * @typedef Matrix3f
 * @brief 3x3 matrix of single-precision floating-point (float)
 * 
 * @details Most commonly used matrix type in ArduPilot for rotation matrices (DCM),
 *          coordinate transformations, and general 3D linear algebra.
 * 
 * @note Default precision for most flight control calculations
 * @note Explicit template instantiation in matrix3.cpp ensures efficient code generation
 * @see Matrix3d for double precision
 * @see AP_AHRS for DCM usage in attitude estimation
 */
typedef Matrix3<float>                  Matrix3f;

/**
 * @typedef Matrix3d
 * @brief 3x3 matrix of double-precision floating-point (double)
 * 
 * @details Used for high-precision calculations where single precision is insufficient,
 *          such as long-duration integration or GPS coordinate transformations.
 * 
 * @note Higher computational cost than Matrix3f on embedded processors
 * @note Explicit template instantiation in matrix3.cpp ensures efficient code generation
 * @see Matrix3f for single precision
 * @see HAL_WITH_EKF_DOUBLE for double precision EKF configuration
 */
typedef Matrix3<double>                 Matrix3d;
