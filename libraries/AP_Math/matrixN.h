/**
 * @file matrixN.h
 * @brief Fixed-size N×N matrix template for small matrices (N=2,3,4,5)
 * 
 * @details Provides a template class for square matrices of dimension N×N,
 *          optimized for small matrix sizes commonly used in navigation and
 *          control algorithms. Elements are stored in row-major order.
 *          
 *          This class supports basic matrix operations including addition,
 *          subtraction, and multiplication. For matrices requiring inversion
 *          or determinant calculation, use the methods in matrix_alg.cpp or
 *          consider Matrix3 for optimized 3×3 operations.
 * 
 * @note Row-major storage: element (i,j) is stored at index i*N + j
 * @note Explicit instantiations for 4×4 float matrices are provided in matrixN.cpp
 * @note For larger matrices (N>5), consider sparse matrix representations for efficiency
 * 
 * @see Matrix3 for optimized 3×3 operations
 * @see matrix_alg.cpp for generic matrix functions including LU decomposition
 */

#pragma once

#include "math.h"
#include <stdint.h>
#include "vectorN.h"

template <typename T, uint8_t N>
class VectorN;


/**
 * @class MatrixN
 * @brief Fixed-size N×N square matrix template for general linear algebra
 * 
 * @details MatrixN provides a compact representation of small square matrices
 *          with compile-time fixed dimensions. Elements are stored in row-major
 *          order in a 2D array v[N][N], where element (i,j) is accessed as v[i][j].
 *          
 *          Common use cases:
 *          - Covariance matrices in Extended Kalman Filters (EKF)
 *          - Small linear system solving (Ax = b)
 *          - Coordinate frame transformation matrices
 *          - State transition matrices in control systems
 *          
 *          The class supports basic operations:
 *          - Construction (zero initialization, diagonal initialization)
 *          - Addition and subtraction (+=, -=)
 *          - Outer product (mult of two vectors)
 *          - Symmetry enforcement for covariance matrices
 *          
 *          For advanced operations like matrix inversion, determinant calculation,
 *          or LU decomposition, use the functions provided in matrix_alg.cpp.
 * 
 * @tparam T Numeric type (typically float or double)
 * @tparam N Matrix dimension (creates N×N square matrix)
 * 
 * @note Storage is row-major: element (i,j) is at memory location v[i][j]
 * @note Total memory footprint: N×N×sizeof(T) bytes
 * @note For N=4, T=float: 64 bytes per matrix
 * 
 * @warning For larger matrices (N>10), consider sparse representations or
 *          heap allocation as stack usage becomes significant
 * @warning Matrix operations do not check for numerical overflow or underflow
 * 
 * @see Matrix3 for optimized 3×3 matrix operations with explicit methods
 * @see VectorN for N-dimensional vector operations
 * @see matrix_alg.cpp for LU decomposition, inversion, and determinant algorithms
 */
template <typename T, uint8_t N>
class MatrixN {
  
    friend class VectorN<T,N>;

public:
    /**
     * @brief Default constructor - initializes all elements to zero
     * 
     * @details Creates a zero matrix (null matrix) with all elements set to 0.
     *          This is the most common initialization for matrices that will be
     *          populated later or accumulated through addition operations.
     * 
     * @note Uses memset for efficient zero-initialization
     * @note Called implicitly when declaring: MatrixN<float, 4> m;
     */
    MatrixN(void) {
        memset(v, 0, sizeof(v));        
    }

    /**
     * @brief Constructor from diagonal elements
     * 
     * @details Creates a diagonal matrix with specified diagonal elements and
     *          all off-diagonal elements set to zero. This is useful for creating
     *          identity matrices (all diagonal elements = 1) or scaling matrices.
     *          
     *          Example:
     *          @code
     *          float diag[4] = {1.0f, 2.0f, 3.0f, 4.0f};
     *          MatrixN<float, 4> m(diag);  // Creates diagonal matrix with 1,2,3,4 on diagonal
     *          @endcode
     * 
     * @param[in] d Array of N diagonal elements (d[0] to d[N-1])
     * 
     * @note Array size must match template parameter N
     * @note All off-diagonal elements are initialized to zero
     * @note To create identity matrix, pass array of all 1.0 values
     */
    MatrixN(const float d[N]) {
        memset(v, 0, sizeof(v));
        for (uint8_t i = 0; i < N; i++) {
            v[i][i] = d[i];
        }
    }

    /**
     * @brief Multiply two vectors to give a matrix (outer product), in-place
     * 
     * @details Computes the outer product of vectors A and B, storing the result
     *          in this matrix. The outer product is defined as: M[i][j] = A[i] * B[j]
     *          
     *          This operation is commonly used in:
     *          - Kalman filter covariance updates
     *          - Computing rank-1 updates to matrices
     *          - Building projection matrices
     *          
     *          Example:
     *          @code
     *          VectorN<float, 3> a = {1, 2, 3};
     *          VectorN<float, 3> b = {4, 5, 6};
     *          MatrixN<float, 3> m;
     *          m.mult(a, b);  // m[i][j] = a[i] * b[j]
     *          @endcode
     * 
     * @param[in] A First vector (column vector in mathematical notation)
     * @param[in] B Second vector (row vector in mathematical notation)
     * 
     * @note Previous contents of the matrix are overwritten
     * @note Result is a rank-1 matrix (all rows are scalar multiples of B)
     * @note For element-wise multiplication, this is NOT the operation you want
     */
    void mult(const VectorN<T,N> &A, const VectorN<T,N> &B);

    /**
     * @brief Subtract matrix B from this matrix (element-wise subtraction)
     * 
     * @details Performs in-place element-wise subtraction: this[i][j] -= B[i][j]
     *          for all elements. This is the standard matrix subtraction operation.
     *          
     *          Commonly used in:
     *          - Kalman filter innovation calculations
     *          - Computing matrix differences for convergence checks
     *          - Error matrix calculations
     * 
     * @param[in] B Matrix to subtract from this matrix (must be same dimensions)
     * 
     * @return Reference to this matrix after subtraction (for chaining operations)
     * 
     * @note Modifies this matrix in-place
     * @note Both matrices must have identical dimensions N×N
     * @note Supports operation chaining: m -= A -= B;
     */
    MatrixN<T,N> &operator -=(const MatrixN<T,N> &B);

    /**
     * @brief Add matrix B to this matrix (element-wise addition)
     * 
     * @details Performs in-place element-wise addition: this[i][j] += B[i][j]
     *          for all elements. This is the standard matrix addition operation.
     *          
     *          Commonly used in:
     *          - Accumulating covariance matrices in filters
     *          - Summing state transition contributions
     *          - Building composite transformation matrices
     * 
     * @param[in] B Matrix to add to this matrix (must be same dimensions)
     * 
     * @return Reference to this matrix after addition (for chaining operations)
     * 
     * @note Modifies this matrix in-place
     * @note Both matrices must have identical dimensions N×N
     * @note Supports operation chaining: m += A += B;
     */
    MatrixN<T,N> &operator +=(const MatrixN<T,N> &B);
    
    /**
     * @brief Force matrix to be symmetric by averaging across diagonal
     * 
     * @details Enforces matrix symmetry by averaging each pair of elements
     *          across the diagonal: new_value = (M[i][j] + M[j][i]) / 2
     *          
     *          This operation is critical for maintaining numerical stability
     *          in covariance matrices, which must be symmetric by definition
     *          but can become slightly asymmetric due to floating-point errors
     *          during repeated operations.
     *          
     *          Use cases:
     *          - Maintaining covariance matrix symmetry in Kalman filters
     *          - Correcting numerical errors in symmetric matrix computations
     *          - Ensuring positive-definite matrix properties are preserved
     *          
     *          Example:
     *          @code
     *          MatrixN<float, 4> cov;
     *          // ... many covariance update operations ...
     *          cov.force_symmetry();  // Restore exact symmetry
     *          @endcode
     * 
     * @note Modifies the matrix in-place
     * @note Should be called periodically during Kalman filter updates
     * @note Diagonal elements are unchanged
     * @note After this operation: M[i][j] == M[j][i] for all i,j
     * 
     * @warning Only use on matrices that should be symmetric - applying to
     *          non-symmetric matrices will corrupt the data
     */
    void force_symmetry(void);

private:
    /**
     * @brief Matrix elements stored in row-major order
     * 
     * @details 2D array storing matrix elements where v[i][j] represents
     *          the element at row i, column j (using 0-based indexing).
     *          
     *          Memory layout: rows are stored contiguously
     *          - v[0][0], v[0][1], ..., v[0][N-1]  (first row)
     *          - v[1][0], v[1][1], ..., v[1][N-1]  (second row)
     *          - ...
     *          - v[N-1][0], v[N-1][1], ..., v[N-1][N-1]  (last row)
     * 
     * @note Row-major ordering is cache-friendly for row-wise operations
     * @note Total memory: N×N×sizeof(T) bytes allocated on stack
     */
    T v[N][N];
};
