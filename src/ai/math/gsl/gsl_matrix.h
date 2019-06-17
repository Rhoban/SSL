
/*
    This file is part of SSL.

    Copyright 2019 Georges Nicolas (ngeorges@enseirb-matmeca.fr)

    SSL is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SSL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with SSL.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_blas.h>

namespace rhoban_ssl
{
/**
 * @brief The Matrix class, heavily based on the gsl GNU library. A vector is defined as a matrix with only one column.
 */
class GslMatrix
{
private:
  /**
   * @brief the gsl matrix beneath the class
   */
  gsl_matrix* m_;

public:
  GslMatrix(const size_t i, const size_t j);

  /**
   * @brief Set all element of the instance's matrix with value x
   * @param setting value
   */
  void setAll(double x);

  /**
   * @brief Set all element of the matrix with 0
   */
  void setZero();

  /**
   * @brief Set the matrix as identity matrix
   */
  void setIdentity();

  /**
   * @brief Get the number of rows in the matrix
   * @return the number of rows
   */
  size_t getRows() const;

  /**
   * @brief Get the number of columns in the matrix
   * @return the number of columns
   */
  size_t getColumns() const;

  /**
   * @brief Get the element in position (i,j) in the matrix
   * @param line of the element to get
   * @param column of the element to get
   * @return the element in the (line, column) position specified
   */
  double getElement(const size_t i, const size_t j) const;

  /**
   * @brief Set the element in position (i,j) in the matrix
   * @param line of the element to set
   * @param column of the element to set
   * @param setting value
   */
  void setElement(const size_t i, const size_t j, double x);

  /**
   * @brief Copy the current matrix in the dest matrix. This function works provided that the dimensions of dest matrix
   * match those of the current matrix
   * @param the matrix containing the copy at the end of the execution
   * @return 0 when success, non-zero value when failure, with corresponding gsl error code
   */
  int copyMatrix(GslMatrix* dest) const;

  /**
   * @brief Copy the transposate of the current matrix in the dest matrix. This function works provided that the
   * dimensions of dest matrix match those of the current matrix
   * @param the matrix containing transposate of the initial matrix at the end of the execution
   * @return 0 when success, non-zero value when failure, with corresponding gsl error code
   */
  int transposeMatrix(GslMatrix* dest) const;

  /**
   * @brief Add every element (i,j) of the current matrix to the element (i,j) of the matrix a. The result is stored in
   * the matrix a
   * @param the other matrix of the addition, and the matrix receiving the result of the operation
   * @return 0 when success, non-zero value when failure, with corresponding gsl error code
   */
  int addMatrix(GslMatrix* a) const;

  /**
   * @brief Substract every element (i,j) of the current matrix to the element (i,j) of the matrix a. The result is
   * stored in the matrix a
   * @param the other matrix of the substraction, and the matrix receiving the result of the operation
   * @return 0 when success, non-zero value when failure, with corresponding gsl error code
   */
  int subMatrix(GslMatrix* a) const;

  /**
   * @brief Multiply every element (i,j) of the current matrix by x.
   * @return 0 when success, non-zero value when failure, with corresponding gsl error code
   */
  int scaleMatrix(const double x);

  /**
   * @brief Process the matrix multiplication and sum [dest = alpha.A*B + beta.C] where B is the instance executing the
   * method. The result is stored in the matrix dest. Uses Level 3 BLAS library in gsl. Matrices can be transposed
   * during the computing thanks to optional parameters (better optimized)
   * @param the result-receiving matrix
   * @param the other matrix of the multiplication
   * @param the matrix in the sum
   * @param the scalar parameter of the product part of the equation ; initial value is 1
   * @param the scalar parameter of the addition part of the equation ; initial value is 1
   * @param specifies if the A matrix shall be transposed or not ; initial value is false
   * @param specifies if the B matrix shall be transposed or not ; initial value is false
   * @return 0 when success, non-zero value when failure, with corresponding gsl error code
   */
  int multMatrixBLAS(GslMatrix* dest, const GslMatrix* a, const GslMatrix* c, double alpha = 1.0, double beta = 1.0,
                     bool transA = false, bool transB = false) const;

  /**
   * @brief Process the matrix multiplication and sum [dest = alpha*A*B*C + beta.D] where B is the instance executing
   * the method. The result is stored in the matrix dest. Uses Level 3 BLAS library in gsl. Matrices can be transposed
   * during the computing thanks to optional parameters (better optimized)
   * @param the result-receiving matrix
   * @param the left-member matrix of the multiplication
   * @param the right-member matrix of the multiplication
   * @param the matrix in the sum
   * @param the scalar parameter of the product part of the equation ; initial value is 1
   * @param the scalar parameter of the addition part of the equation ; initial value is 1
   * @param specifies if the A matrix shall be transposed or not ; initial value is false
   * @param specifies if the B matrix shall be transposed or not ; initial value is false
   * @param specifies if the C matrix shall be transposed or not ; initial value is false
   * @return 0 when success, non-zero value when failure, with corresponding gsl error code
   */
  int multMatrixBLASKalman(GslMatrix* dest, const GslMatrix* a, const GslMatrix* c, const GslMatrix* d,
                           double alpha = 1.0, double beta = 1.0, bool transA = false, bool transB = false,
                           bool transC = false) const;

  /**
   * @brief Assess whether the matrix is a Null matrix or not
   * @return true when success, false when failure
   */
  bool isNull() const;

  /**
   * @brief Assess whether every element of the matrix is positive
   * @return true when success, false when failure
   */
  bool isPos() const;

  /**
   * @brief Assess whether every element of the matrix is negative
   * @return true when success, false when failure
   */
  bool isNeg() const;

  /**
   * @brief Assess whether the matrix is defined positive using Cholesky decomposition. Because the algorithm uses
   * Cholesky decomposition, the current matrix MUST be a symetric, positive-definite square matrix.
   * @return true when success,false when failure and a GSL_EDOM error code
   */
  bool isPositiveDefinite() const;

  /**
   * @brief Inverse the current matrix. The inverse matrix is stocked in dest. Because the algorithm uses Cholesky
   * decomposition, the current matrix MUST be a symetric, positive-definite square matrix.
   * @param the matrix which will contains the resultant inverse matrix
   * @return 0 when success, non-zero value when failure, with corresponding gsl error code
   */
  int inverseMatrix(GslMatrix* dest) const;

  ~GslMatrix();
};
}  // namespace rhoban_ssl